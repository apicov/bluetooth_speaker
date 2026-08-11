/*
 * Firmware half of the visualiser: get aligned blocks of audio out of the
 * playback path, hand them to the shared pipeline, put the result on the strip.
 *
 * Everything that decides what the lights DO lives in analysis.cpp and
 * patterns.cpp, which have no platform dependencies and are driven identically
 * by tools/pattern_lab on a laptop. This file owns only the parts that cannot
 * be: the stream buffer, the block alignment, the task, and the strip.
 */
#include "visualiser.h"

#include <atomic>
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <optional>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"
#include "esp_timer.h"
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER
#include "driver/gpio.h"
#endif

#include "analysis.hpp"
#include "patterns.hpp"
#include "ml_lane.hpp"
#include "resample.h"
#include "result_latch.hpp"
#include "led_strip_wrapper.hpp"

namespace {

constexpr const char *TAG = "vis";

/*
 * The two capabilities the source choice implies, named separately because they
 * are two things and not one.
 *
 * Every conditional in this file used to test CONFIG_DANCEFLOOR_LED_SOURCE_REMOTE
 * directly, and about half of them meant "does this unit transform samples"
 * while the other half meant "does this unit take frames off the wire". Today
 * those are exact complements, so one symbol served for both and the difference
 * never showed.
 *
 * They stop being complements as soon as a unit is given a spectrum by the hub
 * and runs its own detector on it -- which is a mode this is deliberately being
 * left ready for. Such a unit takes remote frames AND decides, while analysing
 * no audio and needing none of the FFT's buffers. Adding it should be adding a
 * mode, not re-deriving which of ten conditionals meant which thing.
 *
 * So each site below says what it depends on. Nothing about the current builds
 * changes; these are still one symbol apart.
 */
#if CONFIG_DANCEFLOOR_LED_SOURCE_REMOTE
#define DF_ANALYSES_AUDIO      0
#define DF_TAKES_REMOTE_FRAMES 1
#else
#define DF_ANALYSES_AUDIO      1
#define DF_TAKES_REMOTE_FRAMES 0
#endif

/*
 * The same question for the pluggable analysers, and deliberately a SEPARATE
 * answer.
 *
 * A unit may compute its own FFT frames and still be given a model's results:
 * the FFT is cheap and deterministic, a model may be neither, and a satellite
 * that cannot hold an arena can still light up in step with one that can. The
 * reverse -- being given frames but running analysers locally -- cannot work,
 * because a unit taking frames analyses no audio and there is nothing to run
 * them on. Kconfig enforces that; this is only the derivation.
 */
#if CONFIG_DANCEFLOOR_ML_SOURCE_REMOTE || CONFIG_DANCEFLOOR_LED_SOURCE_REMOTE
#define DF_RUNS_ANALYSERS 0
#else
#define DF_RUNS_ANALYSERS 1
#endif

using df::FFT_N;
using df::HOP_N;
using df::TAIL_N;
using df::RATE;
using df::CHANNELS;

/*
 * Sized by how bursty the audio ARRIVES, which is not how smoothly it plays.
 *
 * This was four analysis frames -- 93 ms -- and that was right while the feed
 * came from the DAC, where audio turns up one chunk per playback pass and the
 * buffer only had to cover the analysis task being descheduled.
 *
 * Fed from the arrival side it has to cover the source's delivery pattern
 * instead, and the hub's own sbc_in line reports what that is: bursts with
 * gaps of 77 to 115 ms between them. A burst after a 105 ms gap is ~18.5 kB
 * against a 16 kB buffer, so it overflowed by about the 512 B seen dropped in
 * nearly every window on the hub and none on the satellite -- which never sees
 * it, because the hub re-sends packets paced.
 *
 * A short send is not just lost audio. It sets s_align_pending, so the unit
 * re-derives its origin and drops a block, and it does that on one unit and not
 * the other -- which is the exact divergence this whole path exists to avoid.
 *
 * Eight windows is 186 ms, comfortably past the worst gap observed and about the
 * lead the audio itself carries. The cost is 16 kB more of a 16 kB buffer, on a
 * unit with ~50 kB free.
 *
 * Sized in windows rather than hops on purpose: this covers how audio ARRIVES,
 * which is a number of bytes per burst, and bytes do not care how often they are
 * analysed. Overlapping the windows raises the frame rate without changing what
 * has to be held.
 */
constexpr int STREAM_BYTES = FFT_N * CHANNELS * (int)sizeof(int16_t) * 8;
constexpr uint32_t FRAME_BYTES = CHANNELS * sizeof(int16_t);

/* The hop and the carried-over tail in bytes, which is the unit the accumulator
 * works in. */
constexpr size_t HOP_BYTES  = (size_t)HOP_N * CHANNELS * sizeof(int16_t);
constexpr size_t TAIL_BYTES = (size_t)TAIL_N * CHANNELS * sizeof(int16_t);

constexpr uint32_t LED_COUNT = CONFIG_DANCEFLOOR_LED_COUNT;

/*
 * One definition, in components/dancefloor_sync/Kconfig. Kconfig symbols are
 * global in ESP-IDF, so this reads it without including the audio protocol or
 * depending on that component -- which matters, because everything here has to
 * stay buildable on its own.
 *
 * The fallback keeps that true: if this component is ever built in a project
 * that has no dancefloor_sync, it compiles rather than failing on a missing
 * symbol.
 */
#ifndef CONFIG_DANCEFLOOR_LOG_PERIOD_S
#define CONFIG_DANCEFLOOR_LOG_PERIOD_S 20
#endif
constexpr int64_t LED_LOG_PERIOD_US = CONFIG_DANCEFLOOR_LOG_PERIOD_S * 1000000LL;

#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER
/*
 * One flash per second of MASTER-CLOCK time, on every unit at once.
 *
 * The second boundary is taken from due_us, which is derived from the scheduled
 * instant the audio carries -- so every unit crosses the same boundary on the
 * same block of audio, with nothing sent between them and no local clock read.
 * That is the same property the strips themselves rely on; this just makes it
 * visible.
 *
 * Watch two boards side by side: if the onboard LEDs flash together the whole
 * chain agrees, and if one lags it is obvious without a console. The eye
 * resolves maybe 10-20 ms, so this is a presence check rather than a
 * measurement -- the numbers live in AUDIO SYNC and TRACK DIVERGENCE.
 */
constexpr int64_t LED_MARKER_PERIOD_US = 1000000;

/*
 * Held for two blocks -- ~46 ms -- rather than delayed for.
 *
 * The audio marker can afford a 200 us busy-wait; this cannot. It has to be
 * long enough to see, and blocking the render task for 40 ms would drop two
 * analysis frames and break the very alignment being demonstrated. Raising the
 * pin on one frame and lowering it a few later costs nothing.
 *
 * Derived from a DURATION rather than a frame count, because "two frames" is
 * 46 ms at hop 1024 and 12 ms at hop 256 -- and a marker that exists to be seen
 * by eye must not quietly shrink below what an eye resolves when the frame rate
 * changes. Rounded up, so this is still exactly 2 at hop 1024.
 */
constexpr int64_t LED_MARKER_HIGH_US = 40000;
constexpr int64_t LED_MARKER_HOP_US  = (int64_t)HOP_N * 1000000 / RATE;
constexpr int LED_MARKER_BLOCKS_HIGH =
    (int)((LED_MARKER_HIGH_US + LED_MARKER_HOP_US - 1) / LED_MARKER_HOP_US);

/*
 * Which level lights the LED, which is a property of the wiring and not of the
 * marker.
 *
 * With the LED's anode on 3V3 and its cathode on the pin, the pin sinks the
 * current and a LOW level lights it. Everything below is written in terms of ON
 * and OFF rather than 1 and 0 so that the timing above -- fire on the second
 * boundary, hold for LED_MARKER_BLOCKS_HIGH -- reads the same either way, and
 * so that a rewired board is a Kconfig change rather than a code change.
 *
 * Getting this wrong shows as an INVERTED marker rather than a dark one: lit
 * for the 960 ms between flashes, dark for the 40 ms of the flash. That looks
 * like a working LED until you count, and against a correctly wired unit beside
 * it, it looks like a sync fault.
 */
#if CONFIG_DANCEFLOOR_LED_MARKER_ACTIVE_LOW
constexpr int LED_MARKER_ON  = 0;
constexpr int LED_MARKER_OFF = 1;
#else
constexpr int LED_MARKER_ON  = 1;
constexpr int LED_MARKER_OFF = 0;
#endif
#endif

/* Applied to the pattern's output on the way to the strip. A device concern,
 * not a pattern one -- patterns work in full range and this scales it. */
constexpr float BRIGHTNESS = CONFIG_DANCEFLOOR_LED_BRIGHTNESS / 100.0f;

#if   CONFIG_DANCEFLOOR_LED_TYPE_SK6812
constexpr LedStrip::Type STRIP_TYPE = LedStrip::Type::SK6812_RGBW;
#elif CONFIG_DANCEFLOOR_LED_TYPE_WS2811
constexpr LedStrip::Type STRIP_TYPE = LedStrip::Type::WS2811;
#else
constexpr LedStrip::Type STRIP_TYPE = LedStrip::Type::WS2812;
#endif

StreamBufferHandle_t pcm_stream;
std::optional<LedStrip> strip;
/*
 * Not built where nothing analyses.
 *
 * process() is only ever called from visualiser_task, which such a unit does not
 * run -- so no FFT and no detector has ever executed there. What did happen is
 * that the object was defined anyway, reserving the whole window-sized state in
 * .bss for the entire uptime: buf_ 8 kB, win_ 4 kB, mag_ 2 kB, plus the band and
 * spectrum tables, two detectors and a Frame. init() then filled a Hann window
 * and derived band edges at boot for something that would never be asked
 * anything.
 *
 * Guarded on the capability rather than on the source option, so it stays right
 * for a unit that is given a spectrum and runs its own detector on it: that unit
 * needs the detectors and still has no use for any of this.
 */
#if DF_ANALYSES_AUDIO
df::Analysis analysis;
#endif

/*
 * The rate of the audio being fed, which is whatever the source chose.
 *
 * Atomic because two tasks read it: the playback task converts due_us to a
 * sample position in visualiser_feed(), and the analysis task converts a block
 * index back to due_us. They are the two halves of one conversion and must use
 * the same number, so there is exactly one of it.
 *
 * Starts at the rate the tuning was measured at, which is also what a source
 * almost always picks, so a unit that is never told anything behaves as it
 * always did.
 */
std::atomic<uint32_t> s_rate{df::RATE};
df::Pattern *pattern = nullptr;
uint8_t pixels[LED_COUNT * 3];

/*
 * Block alignment.
 *
 * The detector cuts a window of FFT_N every HOP_N samples, and every unit must
 * cut at the same positions or a transient near a boundary is split on one board
 * and centred on another -- the marginal onsets then fire on one strip and not
 * the other. Boundaries are derived from the instant audio is SCHEDULED to be
 * heard, which all units agree on, and never from a clock read here, which they
 * do not: they reach this code milliseconds apart.
 *
 * The grid is the HOP, not the window. Windows may overlap; what every unit has
 * to agree on is where one is allowed to START, and that is a multiple of HOP_N.
 * Since FFT_N is a whole number of hops, the old window grid is a subset of this
 * one -- a finer hop refines the positions rather than moving them.
 *
 * Both sides count bytes that actually passed through the buffer, so a drop
 * cannot make them disagree about position.
 */
/*
 * Atomic because three tasks raise it: the audio task on a short send, this
 * component's own task when it starves, and the drift servo after a retune.
 * Only the audio task lowers it, with an exchange, so a request raised while an
 * alignment is being computed is not lost.
 */
std::atomic<bool> s_align_pending{true};
bool     s_mark_align_point;
int32_t  s_skip_frames;
int64_t  s_pending_block_index;

std::atomic<uint32_t> s_align_at_byte;      /* byte index where aligned audio starts */
std::atomic<long long> s_align_block_index; /* block number of that first sample */
/*
 * Bumped on every publish, and what the analysis task actually watches.
 *
 * It used to watch s_align_at_byte, on the assumption that a new alignment
 * always carries a new byte index. It does not, and the case where it does not
 * is the common one: while this task is behind, the buffer is full and feeds are
 * rejected ENTIRELY, so s_sent_total does not move. Every rejected feed re-arms
 * the alignment, and each one then publishes the same byte index with a later
 * block index. The reader took the first and could not see any of the rest --
 * they compared equal to what it had already adopted -- so it went on labelling
 * from an origin two or three blocks stale.
 *
 * Boundaries survived that, which is why it went unnoticed for so long: the
 * discard arithmetic still lands on a multiple of HOP_N, so both units cut
 * identical windows and simply dated them differently. due_us is what carries
 * the date, and every time-driven pattern is built on it, so the strips ran the
 * same animation from different points of its cycle -- stepping further apart at
 * each burst of drops and never recovering. Modelled in test_align.c, where the
 * byte-index reader mislabels ~53% of blocks over a run and this one mislabels
 * none.
 *
 * The first publish was ignored too, for the duller reason that a byte count of
 * zero is indistinguishable from the initial value. A generation counter has
 * neither problem.
 */
std::atomic<uint32_t> s_align_gen;
uint32_t s_sent_total;                      /* feed side only */

/*
 * The safety net: the origin this file is counting from, kept beside the
 * scheduled instant it was derived from, so the two can be compared.
 *
 * Everything above depends on the count of what has passed through here staying
 * true, and the only defence was that each caller REMEMBERS to report the events
 * that break it. Three do (a short send below, a splice, a retune). The list is
 * not closed, and the ones nobody thought of fail silently and for good:
 *
 *   - content dropped before it ever reaches the playback path, so the audio
 *     the timeline accounts for never arrives here (the hub's local ring and
 *     the satellite's receive ring both drop when full, and both only count it)
 *   - a short read from the playback ring, zero-filled to a whole chunk: audio
 *     the timeline does NOT account for, invented and fed here as though it were
 *     real
 *
 * Each leaves the count and the timeline permanently offset, on one unit only.
 * Measured on the host over forro-shaped material, that is not subtle: two units
 * offset by 2.9 ms already render 15% of their frames visibly differently, and
 * one packet's worth -- 42 ms -- puts 3.4% of frames in the state where one
 * strip is lit and the other is dark. It never recovers on its own.
 *
 * So stop relying on the callers being exhaustive. `due_master_us` arrives on
 * every feed and says where the timeline thinks this audio is; the count says
 * where this file thinks it is. If they disagree, the count is wrong, whatever
 * caused it, and the next scheduled instant re-derives it.
 */
int64_t  s_ref_due;                         /* scheduled instant of the aligned frame */
uint32_t s_ref_byte;                        /* ... and its byte index */
bool     s_ref_valid;

/*
 * How far the two may disagree before the origin is re-derived.
 *
 * This is a BOUND on how far the count may be from the timeline, so it is also
 * the bound on how far two units can be from each other -- and smaller is better
 * on both counts, which is the opposite of what it first looks like.
 *
 * The scheduled timeline is not exactly the content rate. The hub slews it by up
 * to 1 ms/s to walk it back to real time, and `next_play_at += frames * 1000000
 * / rate` truncates, which alone runs it ~20 ppm slow. Neither is a fault and
 * neither is worth correcting: every unit is handed the same play_at values and
 * counts the same audio, so both errors are common to all of them and cost
 * nothing at all while they stay common.
 *
 * They stop being common the moment one unit re-derives and another has not.
 * That is why the threshold wants to be small: two units differ by at most what
 * one of them has accumulated since its own last alignment, and that is bounded
 * by exactly this number. Where their origins agree -- the usual case, since a
 * track boundary re-derives every unit at once -- the trigger is a function of
 * data every unit shares, so they cross it on the same audio and stay identical
 * however often it fires.
 *
 * The cost of firing is one dropped analysis frame, one hop -- ~23 ms at hop
 * 1024 -- of the strip holding
 * its previous frame. At 20 ppm this crosses about once every 100 s on its own.
 * That is affordable, so this is set by what is NOT drift: the per-packet slew
 * step is ~20 us and the interpolation rounds to the microsecond, so 2 ms is two
 * orders of magnitude above the noise and still under one chunk of audio.
 *
 * Measured for reference, on the host over forro-shaped material: two units
 * whose audio is offset by 2.9 ms render 15% of frames visibly differently, and
 * at 42 ms -- one packet, which is what a single unreported drop used to cost
 * for good -- 3.4% of frames have one strip lit and the other dark.
 */
constexpr int64_t ALIGN_DRIFT_US = 2000;

/* Statistics, so a disagreement between two units is diagnosable rather than a
 * matter of opinion. Atomic: incremented by the audio task, read and cleared by
 * the analysis task. */
std::atomic<uint32_t> s_dropped;
std::atomic<uint32_t> s_aligns;
std::atomic<uint32_t> s_onsets;
std::atomic<uint32_t> s_frames;
/*
 * The boom detector's own counts, because `onsets` is the wideband detector and
 * BoomPattern does not use it -- so on a floor running "boom" the log described
 * a decision nobody could see and said nothing about the one on the strip.
 *
 * `marginal` is the reason this is here rather than just `booms`. Two units
 * analyse audio a few ms apart, so their blocks overlap ~91% and their flux
 * values are close but never equal. Any block whose flux sits near its own
 * threshold can therefore fall either way, and that population -- not the
 * timeline -- is what puts one strip lit and the other dark. Counting it needs
 * one unit rather than two boards and two log windows lined up by hand.
 *
 * Within 10% of threshold is a proxy, not a derivation: the real width is the
 * flux difference between the units, which no unit can see on its own.
 */
std::atomic<uint32_t> s_booms;
std::atomic<uint32_t> s_marginal;
/* Re-alignments nobody asked for -- see ALIGN_DRIFT_US. Reported separately
 * from s_aligns because they mean something different: an align is an event
 * being handled, a drift is an event that was never reported at all. */
std::atomic<uint32_t> s_drifts;
std::atomic<int32_t>  s_last_drift_us;

[[maybe_unused]] [[maybe_unused]] uint32_t take(std::atomic<uint32_t> &c) { return c.exchange(0, std::memory_order_relaxed); }
void bump(std::atomic<uint32_t> &c, uint32_t n = 1) { c.fetch_add(n, std::memory_order_relaxed); }
/* Running maximum. Four counters need one, and the CAS loop is the sort of thing
 * that is right three times and subtly wrong the fourth. */
void note_max(std::atomic<uint32_t> &c, uint32_t v)
{
    uint32_t prev = c.load(std::memory_order_relaxed);
    while (v > prev && !c.compare_exchange_weak(prev, v, std::memory_order_relaxed)) {
    }
}

/*
 * Frames computed but not yet due.
 *
 * Analysis and display are separate stages. A frame is computed whenever the
 * audio for it is available and drawn when the instant it describes comes
 * round, which is the same schedule-the-future trick docs/clock-sync.md section
 * 4 uses for the audio itself: nothing passes between the boards at the moment
 * of drawing, they each keep the same appointment.
 *
 * What that buys is worth stating plainly, because it is not obvious. Before
 * this, a frame was drawn as soon as it was computed, so two strips agreed only
 * as closely as the two units' PLAYBACK agreed -- and each unit tolerates
 * PHASE_DEADBAND_US (7 ms) of its own error, so up to 14 ms between them, which
 * is inside what an eye resolves. Drawn on the label instead, they agree as
 * closely as the two CLOCKS do, which TSF puts under a millisecond. The servo
 * stops being in the path at all.
 *
 * Single producer (the analysis task), single consumer (the render task), so
 * the indices need no locking. They are free-running counts, not wrapped
 * positions: the difference is the depth and stays right across a uint32 wrap.
 *
 * Sized in TIME, not in frames, which is the whole point of the expression.
 *
 * 32 slots is ~740 ms at hop 1024 and comfortably more than the 200 ms of lead
 * the audio carries -- but at hop 256 the same 32 slots are 186 ms, which is
 * LESS than that lead. The analysis runs as far ahead as it has audio for, so
 * the queue would fill, overrun, and drop frames that the strip then never
 * draws. Scaling with FFT_N / HOP_N holds ~740 ms whatever the hop and leaves
 * hop 1024 exactly as it was.
 *
 * The cost is RAM, and it is not nothing: df::Frame is ~136 bytes, so this is
 * 4.4 kB at hop 1024, 8.7 kB at 512 and 17.4 kB at 256.
 *
 * Must stay a power of two. head and tail are free-running uint32 counters and
 * `% FRAME_RING` is only continuous across their wrap because 2^32 is a whole
 * number of rings.
 */
constexpr uint32_t FRAME_RING = 32 * (FFT_N / HOP_N);
static_assert((FRAME_RING & (FRAME_RING - 1)) == 0,
              "FRAME_RING must be a power of two -- the uint32 wrap depends on it");
df::Frame s_fq[FRAME_RING];
std::atomic<uint32_t> s_fq_head;    /* analysis task writes */
std::atomic<uint32_t> s_fq_tail;    /* render task writes */
std::atomic<uint32_t> s_fq_flush;   /* bumped to discard what is queued */
std::atomic<uint32_t> s_late;       /* frames that came due before we got to them */
std::atomic<uint32_t> s_overrun;    /* frames dropped because the queue was full */
/* Render-side cost, published for the analysis task's log line. The two halves
 * scale with different things -- see the note where they are logged -- and are
 * only useful next to each other. */
std::atomic<uint32_t> s_render_sum, s_render_max, s_render_n;
/*
 * The render cost, split, and how long the task was kept from running.
 *
 * These exist to separate two explanations of the same number that call for
 * completely different fixes. At hop 512 a frame is due every 11.6 ms, and the
 * measured render max is 20 ms against a mean of 0.7 -- so one draw in a
 * thousand overruns a whole frame period and pushes the next past
 * RENDER_LATE_US. Either the work is occasionally slow, or the task is
 * occasionally not running. Timing the strip write apart from the pattern says
 * which half; timing the naps says whether the core was available at all.
 *
 * `s_wake_*` is the OVERSHOOT of a vTaskDelay, not its length, and it is a
 * one-sided measurement on purpose. vTaskDelay(N) unblocks the task at a tick
 * boundary between (N-1) and N ticks away, so quantisation at
 * CONFIG_FREERTOS_HZ=1000 can only make a nap come up SHORT. Sleeping longer
 * than asked therefore means the task was ready and not running -- there is no
 * benign explanation for a positive value beyond a tick of measurement noise,
 * which is what makes it worth reading.
 */
std::atomic<uint32_t> s_pattern_sum, s_pattern_max;
std::atomic<uint32_t> s_show_sum, s_show_max;
std::atomic<uint32_t> s_wake_sum, s_wake_max, s_wake_n;
/* Times the strip went dark because nothing arrived -- see RENDER_IDLE_US. */
std::atomic<uint32_t> s_idle_dark;

/*
 * Master -> local, or null on a unit where they are the same thing.
 *
 * A function rather than a stored offset because the satellite's is slewed
 * continuously; see visualiser_set_clock().
 */
std::atomic<int64_t (*)(int64_t)> s_to_local{nullptr};

/* Set on a unit that sends its frames onward. Null publishes nothing. */
std::atomic<void (*)(const vis_frame_t *)> s_publish{nullptr};
/* The same, for analyser results. Written from two lanes, so atomic. */
std::atomic<void (*)(const ml_result_t *)> s_ml_publish{nullptr};

static_assert(VIS_RESULT_SCORES == df::RESULT_SCORES,
              "the wire result and df::Result disagree about how many scores");

/* df::Result -> the form that can leave this unit, and back. Plain copies:
 * unlike a frame there is nothing here that cannot travel. */
void to_wire(const df::Result &r, ml_result_t *w)
{
    w->show_at_us = r.show_at_us;
    w->index      = r.index;
    w->analyser   = r.analyser;
    w->model_id   = r.model_id;
    w->n          = r.n;
    w->unit       = 0;                  /* the hub is the only publisher today */
    std::memcpy(w->label, r.label, sizeof(w->label));
    std::memcpy(w->score, r.score, sizeof(w->score));
}

[[maybe_unused]] void from_wire(const ml_result_t *w, df::Result &r)
{
    r = df::result_none();
    r.show_at_us = w->show_at_us;
    r.index      = w->index;
    r.analyser   = w->analyser;
    r.model_id   = w->model_id;
    r.n          = w->n > df::RESULT_SCORES ? df::RESULT_SCORES : w->n;
    std::memcpy(r.label, w->label, sizeof(r.label));
    std::memcpy(r.score, w->score, sizeof(r.score));
}

/*
 * Send a result onward, if anything is listening.
 *
 * Called from whichever lane produced it. Both lanes hand results to the local
 * latch first and come here afterwards, so this unit's own strip never waits on
 * a radio and a send that fails costs a neighbour a result rather than costing
 * this unit one too -- the same ordering publish_frame() has.
 */
void publish_result(const df::Result &r)
{
    if (const auto publish = s_ml_publish.load(std::memory_order_relaxed)) {
        ml_result_t w;
        to_wire(r, &w);
        publish(&w);
    }
}

/* BEAT_BANDS is a macro from beat_detect.h, not a df:: member. */
static_assert(VIS_BANDS == BEAT_BANDS, "wire frame lost a band");
static_assert(VIS_SPEC_BINS == df::SPEC_BINS, "wire frame and spectrum disagree");

/* df::Frame -> the form that can leave this unit. mag is the only field with
 * nowhere to go; everything else is a copy.
 *
 * Only compiled where frames are produced here -- a unit that is given them
 * never converts one out. */
#if DF_ANALYSES_AUDIO
void to_wire(const df::Frame &f, vis_frame_t *w)
{
    w->due_us = f.due_us;
    w->index  = f.index;
    std::memcpy(w->band, f.band, sizeof(w->band));
    std::memcpy(w->spec, f.spec, sizeof(w->spec));
    w->flux           = f.flux;
    w->threshold      = f.threshold;
    w->strength       = f.strength;
    w->boom_strength  = f.boom_strength;
    w->boom_flux      = f.boom_flux;
    w->boom_threshold = f.boom_threshold;
    w->onset = f.onset ? 1 : 0;
    w->boom  = f.boom ? 1 : 0;
    w->unit  = f.unit;
}
#endif

/* ... and back. mag stays null: it did not travel and a Pattern cannot read it
 * on a locally analysed frame either, since rendering is deferred.
 *
 * Only compiled where frames are taken from elsewhere -- a unit doing its own
 * analysis never converts one back. */
#if DF_TAKES_REMOTE_FRAMES
void from_wire(const vis_frame_t *w, df::Frame &f)
{
    f.due_us = w->due_us;
    f.index  = w->index;
    std::memcpy(f.band, w->band, sizeof(f.band));
    std::memcpy(f.spec, w->spec, sizeof(f.spec));
    f.mag            = nullptr;
    f.flux           = w->flux;
    f.threshold      = w->threshold;
    f.strength       = w->strength;
    f.boom_strength  = w->boom_strength;
    f.boom_flux      = w->boom_flux;
    f.boom_threshold = w->boom_threshold;
    f.onset = w->onset != 0;
    f.boom  = w->boom != 0;
    f.unit  = w->unit;
}
#endif

/*
 * Put a frame in the queue for the instant it names.
 *
 * One path for both sources, so a frame taken from the radio and a frame
 * computed here are treated identically from this point on -- which is what
 * makes them interchangeable rather than merely similar.
 *
 * Dropping the newest when full is deliberate. Evicting the oldest would
 * discard a frame about to be due in order to keep one that is not, so a strip
 * already behind skips forward instead of catching up.
 */
bool enqueue(const df::Frame &f, const df::Result *fast_ml)
{
    const uint32_t head = s_fq_head.load(std::memory_order_relaxed);
    const uint32_t tail = s_fq_tail.load(std::memory_order_acquire);
    if (head - tail >= FRAME_RING) {
        bump(s_overrun);
        return false;
    }
    df::Frame &dst = s_fq[head % FRAME_RING];
    dst = f;
    /*
     * Fast-lane results belong to THIS frame -- their window is this frame's
     * window and their presentation delay is zero -- so they are carried with
     * it rather than latched. Slow-lane slots are left empty here and filled by
     * the render task when their show_at_us comes round.
     *
     * Overwritten unconditionally, including with result_none(), because the
     * queue slot is reused and whatever a previous frame left in it describes
     * audio 740 ms ago.
     */
    for (int i = 0; i < df::ML_SLOTS; i++) {
        dst.ml[i] = fast_ml ? fast_ml[i] : df::result_none();
    }
    /* Release, against the acquire on the reader: the frame must be fully
     * written before the index that publishes it moves. */
    s_fq_head.store(head + 1, std::memory_order_release);
    return true;
}

/*
 * Where a slow analyser's answer waits for the moment it describes, and the
 * slots this unit fills itself rather than waiting for.
 *
 * The mechanism, and the argument for why it stays in step across units, is in
 * result_latch.hpp -- it is pure logic and lives where the host tests can drive
 * it, like the analysis it serves.
 */
df::ResultLatch s_latch;

/* Results produced by this unit, for the log line. Unused on a unit that is
 * given its results -- it produces none. */
[[maybe_unused]] std::atomic<uint32_t> s_ml_results;

/*
 * How early is close enough to draw now rather than sleep again.
 *
 * The audio marker busy-waits to hit its deadline within microseconds. This
 * does not need to: vTaskDelay resolves to 1 ms at CONFIG_FREERTOS_HZ=1000,
 * and an eye resolves 10-20 ms, so a millisecond is already an order of
 * magnitude better than the thing being served. Spinning for it would burn a
 * core to move an error nobody can see.
 */
constexpr int64_t RENDER_SLACK_US = 1000;

/* Sleep at most this long in one go, so a flush is acted on promptly even when
 * the frame at the head is not due for a while. */
constexpr int64_t RENDER_NAP_MS = 20;

/* Past this, the frame was not merely a little late -- something stalled. Drawn
 * anyway, because a stale frame is better than a gap, but counted. */
constexpr int64_t RENDER_LATE_US = 20000;

/*
 * How long the queue may stay empty before the strip goes dark.
 *
 * Only reachable on a unit that is GIVEN its frames: one doing its own analysis
 * notices a stopped stream itself and flushes, which blanks. A unit fed from
 * elsewhere has no analysis task to notice anything, so without this it holds
 * its last frame indefinitely -- a lit strip that looks like it is working
 * while the thing driving it has gone.
 *
 * Half a second is well past any gap normal delivery produces at 43 or more frames a
 * second, and short enough that a stopped floor reads as stopped.
 *
 * Going dark rather than falling back to local analysis is the deliberate
 * choice: a unit told to follow another should stop when it cannot, not start
 * improvising. Improvising is what a locally analysing unit is for, and that is
 * a build-time decision.
 */
constexpr int64_t RENDER_IDLE_US = 500000;

void show(const uint8_t *rgb)
{
    for (uint32_t i = 0; i < LED_COUNT; i++) {
        strip->set(i, static_cast<uint8_t>(rgb[3 * i + 0] * BRIGHTNESS),
                      static_cast<uint8_t>(rgb[3 * i + 1] * BRIGHTNESS),
                      static_cast<uint8_t>(rgb[3 * i + 2] * BRIGHTNESS));
    }
    /* Refresh failures are silent otherwise, and a wedged strip driver fails
     * every single frame -- log the first rather than flooding the console. */
    if (const esp_err_t err = strip->show(); err != ESP_OK) {
        static bool reported = false;
        if (!reported) {
            reported = true;
            ESP_LOGE(TAG, "strip refresh failed: %s", esp_err_to_name(err));
        }
    }
}

/* Not built at all on a unit that is given its frames: there is no audio to
 * analyse there, and the FFT and detectors would be pure cost. */
#if DF_ANALYSES_AUDIO

/*
 * Run every fast-lane analyser over the window just transformed, counting what
 * came back.
 *
 * The lane itself is df::run_fast_lane() in analysers.cpp, so tools/pattern_lab
 * drives exactly this and not a copy of it. All that belongs here is which
 * slots this unit computes -- which is firmware state -- and the counter.
 */
void run_fast_lane(const int16_t *mono, int64_t index, int64_t due_us,
                   df::Result out[df::ML_SLOTS])
{
    bool skip[df::ML_SLOTS];
    for (int i = 0; i < df::ML_SLOTS; i++) {
        skip[i] = s_latch.latched(i);
    }

    df::run_fast_lane(mono, FFT_N, index, due_us, skip, out);

    for (int i = 0; i < df::ML_SLOTS; i++) {
        if (df::result_valid(out[i])) {
            bump(s_ml_results);
            /* After the frame it travels in has been filled, so a radio that is
             * busy costs a neighbour a result and not this unit's own strip. */
            publish_result(out[i]);
        }
    }
}

void visualiser_task(void *arg)
{
    (void)arg;
    static int16_t raw[FFT_N * CHANNELS];
    /* Downmixed once per frame and used twice: by the fast lane, which analyses
     * exactly the window the FFT did, and by the resampler feeding the slow one.
     * Static, not stack -- this task has 4 kB and this is 2 kB of it. */
    static int16_t mono[FFT_N];
    df::Result fast_ml[df::ML_SLOTS];

    /*
     * The feed to the slow lane.
     *
     * Absent entirely on a unit that is given its results -- the resampler
     * table alone is 4 kB and the decimation buffer another 2, which is real
     * money on a satellite with ~52 kB free. Also absent, at runtime, in a
     * build with no slow analyser: ml_lane_rate() returns 0 and nothing below
     * runs.
     */
#if DF_RUNS_ANALYSERS
    static resampler_t ml_rs;
    static int16_t     ml_dec[FFT_N + 8];   /* decimating only, so never more */
    int  ml_rate = df::ml_lane_rate();
    bool ml_restart = true;
#endif
    size_t   filled = 0;
    uint32_t recv_total = 0;
    uint32_t seen_gen = 0;
    int64_t  block_index = 0;
    int64_t  last_report_us = esp_timer_get_time();
    bool     starved_shown = false;
    uint32_t seen_rate = s_rate.load(std::memory_order_relaxed);
    /*
     * What a frame costs, so a change to the analysis rate can be judged
     * against a measurement rather than an expectation.
     *
     * Split because the two halves scale with different things and only one of
     * them is CPU. `analysis` is the FFT and the detectors, and it doubles if
     * the hop halves. `render` is mostly show(), which blocks in
     * spi_device_transmit() for about 30 us per LED -- so it scales with strip
     * length, not with the frame rate, and at the 8 LEDs on the bench it is
     * nearly free. On a real floor it is the term that decides whether a faster
     * analysis rate fits.
     *
     * Task-local: only this task writes or reads them.
     */
    int64_t  cost_analysis = 0, cost_analysis_max = 0;
    uint32_t cost_n = 0;
    /* The fast lane, kept apart from the FFT it runs beside. They share a task
     * and a deadline, so the only useful question about a new analyser is how
     * much of the frame period it took that the FFT was not already using. */
    int64_t  cost_fast = 0, cost_fast_max = 0;

    while (true) {
        /*
         * A rate change re-cuts the analysis bands, so every flux figure in the
         * detector's history was measured against different frequencies and has
         * to go. Handled here rather than in the setter because this task owns
         * `analysis` and the setter can be called from anywhere.
         */
        const uint32_t rate = s_rate.load(std::memory_order_relaxed);
        if (rate != seen_rate) {
            seen_rate = rate;
            analysis.init(static_cast<int>(rate));
            /* Same argument one level up: an analyser's features were derived
             * at the old rate and the state built from them describes different
             * frequencies now. init() rather than reset() because the rate is
             * the one thing an analyser is told about the stream. */
            for (int i = 0; i < df::ML_SLOTS; i++) {
                if (df::Analyser *a = df::analyser_at(i);
                    a && !s_latch.latched(i) && !a->init(static_cast<int>(rate))) {
                    /* Refused the new rate. Marking the slot latched retires it
                     * cleanly -- the render task then reports result_none()
                     * there rather than this unit publishing answers derived at
                     * a rate the analyser has disowned. */
                    s_latch.set_latched(i, true);
                    ESP_LOGE(TAG, "analyser \"%s\" cannot run at %" PRIu32 " Hz -- retired",
                             a->spec().name, rate);
                }
            }
#if DF_RUNS_ANALYSERS
            if (ml_rate > 0) {
                if (ml_rate > (int)rate) {
                    /* Upsampling into a model is not something this lane does,
                     * and ml_dec is sized on the assumption it never happens.
                     * Refusing loudly beats a buffer that overflows at 48 kHz. */
                    ESP_LOGE(TAG, "slow analyser wants %d Hz from a %" PRIu32 " Hz "
                                  "stream -- lane stopped", ml_rate, rate);
                    ml_rate = 0;
                } else if (resample_init(&ml_rs, (int)rate, ml_rate) != 0) {
                    ESP_LOGE(TAG, "no resampler for %" PRIu32 " -> %d Hz -- lane stopped",
                             rate, ml_rate);
                    ml_rate = 0;
                } else {
                    ml_restart = true;
                    ESP_LOGW(TAG, "slow lane resampling %" PRIu32 " -> %d Hz, "
                                  "filter 0x%08" PRIx32,
                             rate, ml_rate, resample_table_checksum(&ml_rs));
                }
            }
#endif
            if (pattern) pattern->reset();
            filled = 0;
            ESP_LOGW(TAG, "analysing at %" PRIu32 " Hz", rate);
        }

        const size_t got = xStreamBufferReceive(pcm_stream,
                                                reinterpret_cast<uint8_t *>(raw) + filled,
                                                sizeof(raw) - filled, pdMS_TO_TICKS(100));
        filled += got;
        recv_total += got;

        /*
         * Has the feed re-aligned? Everything before its published byte index is
         * stale and must not contribute to a block, this partial block included.
         * Dropping only the feed side leaves the boundaries offset by whatever
         * is held here, and the re-alignment silently does nothing.
         */
        /* Acquire, against the release on the publishing side: the byte index
         * and the block index must both be the ones this generation stored. */
        const uint32_t gen = s_align_gen.load(std::memory_order_acquire);
        if (gen != seen_gen) {
            const uint32_t align_at = s_align_at_byte.load(std::memory_order_relaxed);
            const int32_t ahead = static_cast<int32_t>(recv_total - align_at);
            if (ahead < 0) {
                filled = 0;             /* still draining pre-alignment audio */
                continue;
            }
            seen_gen = gen;
            /* The audio after this point does not continue the audio before it,
             * so the resampler's history would smear across the join and the
             * lane's window grid would keep counting from a dead origin. Both
             * are re-derived from the first frame of the new timeline below. */
#if DF_RUNS_ANALYSERS
            ml_restart = true;
#endif
            block_index = s_align_block_index.load(std::memory_order_relaxed);
            const size_t keep = static_cast<size_t>(ahead) < filled
                                ? static_cast<size_t>(ahead) : filled;
            std::memmove(raw, reinterpret_cast<uint8_t *>(raw) + (filled - keep), keep);
            filled = keep;
        }

        if (filled < sizeof(raw)) {
            if (got == 0 && !starved_shown) {
                /* Genuinely starved rather than mid-block. Go dark once instead
                 * of holding the last frame; bounded to one refresh per timeout,
                 * so it cannot become a spin. */
                /*
                 * Genuinely starved rather than mid-block. Going dark is the
                 * render task's job now -- the strip and the pattern belong to
                 * it, and drawing to them from here would race it. A flush is
                 * exactly the right request: what is queued describes a stream
                 * that has stopped.
                 */
                starved_shown = true;
                visualiser_flush();
                /* the stream broke; re-align on return */
                s_align_pending.store(true, std::memory_order_relaxed);
            }
            continue;
        }
        starved_shown = false;

        /* due_us is derived from the hop index, not read from a clock, so it
         * is the same value on every unit for this same audio -- at the rate
         * this unit was told the audio is, which is the same rate its playback
         * used to date the audio in the first place. It names the START of the
         * window, which is what the grid is counted in. */
        const int64_t due_us = block_index * HOP_N * 1000000LL / rate;
        const int64_t t_in = esp_timer_get_time();
        const df::Frame &f = analysis.process(raw, block_index, due_us, 0);
        const int64_t t_analysed = esp_timer_get_time();

        /*
         * The fast lane.
         *
         * Runs on the window df::Analysis has just transformed, so it sees
         * exactly the audio the FFT saw and its answer belongs to exactly this
         * frame -- which is why a fast analyser's presentation delay is zero
         * and why its result travels in the frame rather than through the latch.
         *
         * Mono, because that is what every Analyser is handed; Analysis does
         * the same downmix internally on its own copy. Done once here rather
         * than per analyser, and only when there is a fast analyser to want it.
         */
        df::downmix(raw, FFT_N, mono);
        run_fast_lane(mono, block_index, due_us, fast_ml);

        /*
         * Feed the slow lane the audio that is NEW in this window.
         *
         * Steady state that is the last HOP_N samples: window w covers
         * [w*HOP_N, w*HOP_N+FFT_N) and window w-1 ended at w*HOP_N+TAIL_N, so
         * the tail is what both saw and only the last hop is fresh. The first
         * window after a restart has no predecessor, so all of it is fresh --
         * and it is the one that establishes the origin, which is this window's
         * due_us because its first sample is the first the lane will see.
         *
         * Contiguity matters more than it looks: the lane derives due_us by
         * COUNTING what it is given, so feeding a sample twice or missing one
         * moves every result after it against the timeline for good.
         */
#if DF_RUNS_ANALYSERS
        if (ml_rate > 0) {
            const int16_t *src = mono + TAIL_N;
            int n = HOP_N;
            if (ml_restart) {
                ml_restart = false;
                resample_reset(&ml_rs);
                df::ml_lane_restart(due_us);
                src = mono;
                n = FFT_N;
            }
            const int m = resample_push(&ml_rs, src, n, ml_dec,
                                        (int)(sizeof(ml_dec) / sizeof(ml_dec[0])));
            if (!df::ml_lane_feed(ml_dec, m)) {
                /* The lane could not take it all, so its count no longer
                 * describes the timeline. Re-anchor on the next frame rather
                 * than carrying a grid that is wrong by the amount lost. */
                ml_restart = true;
            }
        }
#endif
        const int64_t t_fast = esp_timer_get_time();

        /*
         * Advance by one HOP, keeping the newest TAIL_BYTES.
         *
         * This was `filled = 0`, and it sat ABOVE process() -- which was safe
         * only because the window and the hop were the same thing, so resetting
         * the counter discarded the whole window and moved nothing. A hop that
         * is shorter than the window has to physically slide the tail down, so
         * it has to happen after the window has been read, not before.
         *
         * The newest bytes, not the oldest. `raw` must remain a contiguous
         * SUFFIX of what has been received, because that is the invariant the
         * realign path above depends on -- it trims `raw` by a byte count
         * measured from the far end. Keeping the oldest would leave the right
         * number of bytes holding the wrong audio, and every check here looks at
         * where a window starts rather than what is in it, so nothing on the
         * board would report it. test_align.c grew a window-contiguity check for
         * exactly that reason.
         *
         * `if constexpr` so that at TAIL_N == 0 this is absent rather than a
         * zero-length memmove off the end of the buffer.
         */
        if constexpr (TAIL_BYTES > 0) {
            std::memmove(raw, reinterpret_cast<uint8_t *>(raw) + HOP_BYTES, TAIL_BYTES);
        }
        filled = TAIL_BYTES;

        block_index++;

        bump(s_frames);
        if (f.onset) bump(s_onsets);
        if (f.boom) bump(s_booms);
        if (f.boom_threshold > 0.0f &&
            std::fabs(f.boom_flux - f.boom_threshold) < 0.1f * f.boom_threshold) {
            bump(s_marginal);
        }

        /*
         * Queue it for the instant it names, rather than drawing it now.
         *
         * Dropping the newest when full is deliberate. The alternative --
         * evicting the oldest -- discards a frame that is about to be due in
         * order to keep one that is not, so a strip that is already behind
         * skips forward instead of catching up. Full means the render task is
         * not keeping pace, and the honest response is to stop adding to its
         * backlog.
         */
        enqueue(f, fast_ml);

        /*
         * Onward, if anything is listening. After the local queue, so this
         * unit's own strip never waits on a radio, and outside it, so a send
         * that fails costs a neighbour a frame rather than costing this unit
         * one too.
         */
        if (const auto publish = s_publish.load(std::memory_order_relaxed)) {
            vis_frame_t w;
            to_wire(f, &w);
            publish(&w);
        }
        {
            const int64_t a = t_analysed - t_in;
            cost_analysis += a;
            if (a > cost_analysis_max) cost_analysis_max = a;
            const int64_t m = t_fast - t_analysed;
            cost_fast += m;
            if (m > cost_fast_max) cost_fast_max = m;
            cost_n++;
        }

        /*
         * Quiet when nothing is wrong, immediate when something is.
         *
         * A healthy window reports the same ~215 frames and 0 dropped bytes
         * every time, and two units doing that every 5 s is most of a console.
         * A window that dropped audio, or re-aligned more than the one time a
         * splice or retune explains, is worth seeing at once -- those are the
         * two things that put the strips out of step with each other.
         */
        const int64_t now = esp_timer_get_time();
        const uint32_t dropped = s_dropped.load(std::memory_order_relaxed);
        const uint32_t aligns  = s_aligns.load(std::memory_order_relaxed);
        const uint32_t drifts  = s_drifts.load(std::memory_order_relaxed);
        /* A drift is always worth seeing at once: it means audio was gained or
         * lost somewhere that does not know it has to say so, which is the one
         * fault in here that used to be permanent and invisible. */
        if (dropped || drifts || aligns > 1 || now - last_report_us >= LED_LOG_PERIOD_US) {
            ESP_LOGI(TAG, "frames %" PRIu32 " | onsets %" PRIu32
                          " | booms %" PRIu32 " (marginal %" PRIu32 ")"
                          " | drop %" PRIu32 " B | aligns %" PRIu32
                          " | drift %" PRIu32 " (last %+ld us) | %s",
                     take(s_frames), take(s_onsets), take(s_booms), take(s_marginal),
                     take(s_dropped), take(s_aligns),
                     take(s_drifts), (long)s_last_drift_us.load(std::memory_order_relaxed),
                     pattern ? pattern->name() : "no pattern");
            /*
             * Its own line, so the one above stays comparable with every log
             * ever captured from this component. `n` is printed because an
             * anomaly can trigger this block early, and a mean over three
             * frames is not a mean.
             */
            /* Taken unconditionally, then divided -- reading them inside the
             * call would leave the sum uncleared on a window with no frames. */
            const uint32_t rn = take(s_render_n), rsum = take(s_render_sum);
            const uint32_t wn = take(s_wake_n), wsum = take(s_wake_sum);
            const uint32_t psum = take(s_pattern_sum), ssum = take(s_show_sum);
            /*
             * The pluggable analysers, on their own line for the same reason
             * `cost:` is on its own: it must stay comparable with every log
             * captured before analysers existed. `late` is the one that matters
             * -- it counts results that missed the frame they named, and the
             * only fix for it is a larger present_delay_us.
             */
            const df::MlLaneStats ml = df::ml_lane_take_stats();
            ESP_LOGI(TAG, "ml: fast %lld/%lld us (mean/max) | slow %" PRIu32 "/%" PRIu32
                          " us | results %" PRIu32 "+%" PRIu32
                          " | late %" PRIu32 " | overrun %" PRIu32
                          " | lane drop %" PRIu32 " | restarts %" PRIu32,
                     cost_n ? cost_fast / cost_n : 0, cost_fast_max,
                     ml.cost_mean_us, ml.cost_max_us,
                     take(s_ml_results), ml.results,
                     s_latch.take_late(), s_latch.take_overrun(),
                     ml.dropped, ml.restarts);
            cost_fast = cost_fast_max = 0;
            ESP_LOGI(TAG, "cost: analysis %lld/%lld us (mean/max) | render %" PRIu32
                          "/%" PRIu32 " us | pat %" PRIu32 "/%" PRIu32
                          " | show %" PRIu32 "/%" PRIu32
                          " | wake +%" PRIu32 "/%" PRIu32 " us"
                          " | queued %" PRIu32 " | late %" PRIu32
                          " | overrun %" PRIu32 " | dark %" PRIu32
                          " | n %" PRIu32 "/%" PRIu32
                          " | stack free %" PRIu32,
                     cost_analysis / cost_n, cost_analysis_max,
                     rn ? rsum / rn : 0, take(s_render_max),
                     rn ? psum / rn : 0, take(s_pattern_max),
                     rn ? ssum / rn : 0, take(s_show_max),
                     wn ? wsum / wn : 0, take(s_wake_max),
                     s_fq_head.load(std::memory_order_relaxed) -
                         s_fq_tail.load(std::memory_order_relaxed),
                     take(s_late), take(s_overrun), take(s_idle_dark), cost_n, rn,
                     uxTaskGetStackHighWaterMark(nullptr));
            cost_analysis = cost_analysis_max = 0;
            cost_n = 0;
            last_report_us = now;
        }
    }
}

#endif  /* DF_ANALYSES_AUDIO */

/*
 * Draw each frame at the instant it names.
 *
 * Nothing passes between the boards here and nothing is corrected against
 * anything: each unit converts due_us with its own offset and keeps its own
 * appointment, exactly as the audio does. That is what makes two strips agree
 * to the accuracy of the clocks rather than to the accuracy of the servo.
 */
void render_task(void *arg)
{
    (void)arg;
    uint32_t seen_flush = s_fq_flush.load(std::memory_order_relaxed);
    int64_t  drawn_at = 0;      /* when the strip last showed something */
#if !DF_ANALYSES_AUDIO
    int64_t  last_report_us = esp_timer_get_time();
#endif
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER
    int64_t last_sec = -1;
    int     lower_in = -1;
#endif

    while (true) {
        const uint32_t flush = s_fq_flush.load(std::memory_order_acquire);
        if (flush != seen_flush) {
            seen_flush = flush;
            /* Only this task writes the tail, so catching it up to the head is
             * safe whatever the producer is doing. Frames added after this are
             * on the new timeline and are kept. */
            s_fq_tail.store(s_fq_head.load(std::memory_order_acquire),
                            std::memory_order_release);
            /*
             * Dark, not the last frame held.
             *
             * A flush means the audio behind what is on the strip has stopped
             * or moved to a new origin. Holding the last frame leaves a lit
             * strip that looks like it is working; going dark says plainly that
             * there is nothing to show, which is what the starve path did
             * before rendering was deferred.
             */
            if (pattern) pattern->reset();
            /* Same reason the queue is emptied: a show_at_us established before
             * the restart names an instant on a timeline that no longer exists,
             * and latching it would put a stale answer on the strip at the
             * moment the new timeline starts. */
            s_latch.flush();
            std::memset(pixels, 0, sizeof(pixels));
            show(pixels);
        }

#if !DF_ANALYSES_AUDIO
        /*
         * The periodic line lives in the analysis task, which is not built on a
         * unit that analyses no audio -- so without this such a unit reports
         * nothing at all, and the numbers that say whether frames are arriving
         * are exactly the ones missing.
         *
         * Same cadence, and only the fields that mean anything here: no audio
         * is being dropped, there is no block grid to re-align, and there is no
         * analysis to time.
         */
        {
            const int64_t now_us = esp_timer_get_time();
            if (now_us - last_report_us >= LED_LOG_PERIOD_US) {
                last_report_us = now_us;
                const uint32_t rn = take(s_render_n), rsum = take(s_render_sum);
                const uint32_t wn = take(s_wake_n), wsum = take(s_wake_sum);
                const uint32_t psum = take(s_pattern_sum), ssum = take(s_show_sum);
                ESP_LOGI(TAG, "frames %" PRIu32 " | onsets %" PRIu32
                              " | booms %" PRIu32 " | render %" PRIu32 "/%" PRIu32
                              " us | pat %" PRIu32 "/%" PRIu32
                              " | show %" PRIu32 "/%" PRIu32
                              " | wake +%" PRIu32 "/%" PRIu32 " us"
                              " | queued %" PRIu32 " | late %" PRIu32
                              " | overrun %" PRIu32 " | dark %" PRIu32
                              " | ml %" PRIu32 " (late %" PRIu32
                              ", overrun %" PRIu32 ") | %s",
                         take(s_frames), take(s_onsets), take(s_booms),
                         rn ? rsum / rn : 0, take(s_render_max),
                         rn ? psum / rn : 0, take(s_pattern_max),
                         rn ? ssum / rn : 0, take(s_show_max),
                         wn ? wsum / wn : 0, take(s_wake_max),
                         s_fq_head.load(std::memory_order_relaxed) -
                             s_fq_tail.load(std::memory_order_relaxed),
                         take(s_late), take(s_overrun), take(s_idle_dark),
                         take(s_ml_results), s_latch.take_late(),
                         s_latch.take_overrun(),
                         pattern ? pattern->name() : "no pattern");
            }
        }
#endif
        const uint32_t tail = s_fq_tail.load(std::memory_order_relaxed);
        const uint32_t head = s_fq_head.load(std::memory_order_acquire);
        if (head == tail) {
            if (drawn_at && esp_timer_get_time() - drawn_at > RENDER_IDLE_US) {
                drawn_at = 0;                /* once, not every nap */
                if (pattern) pattern->reset();
                std::memset(pixels, 0, sizeof(pixels));
                show(pixels);
                bump(s_idle_dark);
            }
            vTaskDelay(pdMS_TO_TICKS(RENDER_NAP_MS));
            continue;
        }

        /*
         * How long until this frame is due, in local time.
         *
         * A label of 0 means the feed had no timeline to date the audio
         * against, so there is nothing to wait for -- draw it and move on. An
         * unset hook means master time IS local time, which is the hub's case
         * and the safe default anywhere.
         */
        const int64_t due = s_fq[tail % FRAME_RING].due_us;
        int64_t wait = 0;
        if (due > 0) {
            const auto to_local = s_to_local.load(std::memory_order_relaxed);
            wait = (to_local ? to_local(due) : due) - esp_timer_get_time();
        }
        if (wait > RENDER_SLACK_US) {
            /* Bounded, so a flush is acted on promptly even when the frame at
             * the head is not due for a couple of hundred milliseconds. */
            const int64_t nap = wait / 1000 < RENDER_NAP_MS ? wait / 1000 : RENDER_NAP_MS;
            const int64_t asked = nap ? nap : 1;
            const int64_t before = esp_timer_get_time();
            vTaskDelay(pdMS_TO_TICKS(asked));
            /* Only the naps taken while a frame is waiting to come due. The
             * idle nap above is a different question -- an empty queue, not a
             * task that could not run. */
            const int64_t over = (esp_timer_get_time() - before) - asked * 1000;
            bump(s_wake_n);
            if (over > 0) {
                bump(s_wake_sum, static_cast<uint32_t>(over));
                note_max(s_wake_max, static_cast<uint32_t>(over));
            }
            continue;
        }
        if (wait < -RENDER_LATE_US) {
            bump(s_late);
        }

        const int64_t t0 = esp_timer_get_time();
        df::Frame f = s_fq[tail % FRAME_RING];
        /* Points into the Analysis that produced it and was overwritten several
         * frames ago. Null rather than dangling -- see Frame::mag. */
        f.mag = nullptr;
        s_fq_tail.store(tail + 1, std::memory_order_release);

        /*
         * Bring the slow slots up to this frame.
         *
         * Here, at the moment of drawing, and not where the frame was produced
         * -- that is what gives a slow analyser the whole presentation lead to
         * work in. See ResultLatch for why it stays in step across units.
         *
         * Slots this unit fills itself in the fast lane are already in the
         * frame and are left alone.
         */
        {
            /* One frame period, for the latch's own lateness check. Computed
             * from the live rate rather than df::RATE: at 48 kHz a frame is
             * 10.7 ms, not 11.6, and a check told the wrong period would report
             * results as late that were not. */
            const int64_t hop_us =
                (int64_t)HOP_N * 1000000LL /
                (int64_t)s_rate.load(std::memory_order_relaxed);
            s_latch.take(f.due_us, hop_us, f.ml);
        }

#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER
        /*
         * Raise when due_us crosses a second, lower two frames later.
         *
         * It fires here rather than at analysis because what it exists to show
         * is when a unit DRAWS, which is now a different instant from when it
         * computed. Two boards side by side answer "do the strips share a
         * timeline" without a console, and after this change the answer is a
         * property of the clocks alone.
         */
        const int64_t sec = f.due_us / LED_MARKER_PERIOD_US;
        if (sec != last_sec) {
            last_sec = sec;
            lower_in = LED_MARKER_BLOCKS_HIGH;
            gpio_set_level(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_MARKER_GPIO),
                           LED_MARKER_ON);
            /*
             * Say so for the first few, then go quiet.
             *
             * Without this, "the LED is not blinking" is three different faults
             * wearing the same face: the code never runs (no audio, so no frame
             * ever reaches here), the code runs but the pin is not wired to an
             * LED on this board, or the option was not built in. Those need
             * completely different fixes and the console could not tell them
             * apart. If these lines appear, the firmware is doing its job and
             * the question is the pin.
             */
            static int told;
            if (told < 3) {
                told++;
                ESP_LOGW(TAG, "LED marker fired on GPIO %d (%d of 3) -- if the "
                              "LED is dark, this board's LED is not on that pin",
                         CONFIG_DANCEFLOOR_LED_MARKER_GPIO, told);
            }
        } else if (lower_in > 0 && --lower_in == 0) {
            gpio_set_level(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_MARKER_GPIO),
                           LED_MARKER_OFF);
        }
#endif

        if (pattern) {
            /* Split, because the two are different kinds of thing: the pattern
             * is arithmetic over LED_COUNT pixels, and show() hands the buffer
             * to a driver that may block. Their sum is still reported as
             * `render`, unchanged, so the figure stays comparable with every
             * log captured before this split existed. */
            const int64_t t_pat = esp_timer_get_time();
            pattern->render(f, pixels, LED_COUNT);
            const int64_t t_show = esp_timer_get_time();
            show(pixels);
            const int64_t t_end = esp_timer_get_time();
            bump(s_pattern_sum, static_cast<uint32_t>(t_show - t_pat));
            note_max(s_pattern_max, static_cast<uint32_t>(t_show - t_pat));
            bump(s_show_sum, static_cast<uint32_t>(t_end - t_show));
            note_max(s_show_max, static_cast<uint32_t>(t_end - t_show));
        }

        drawn_at = esp_timer_get_time();
        const uint32_t took = static_cast<uint32_t>(drawn_at - t0);
        s_render_sum.fetch_add(took, std::memory_order_relaxed);
        bump(s_render_n);
        note_max(s_render_max, took);
    }
}

}  // namespace

void visualiser_set_clock(int64_t (*master_to_local)(int64_t))
{
    s_to_local.store(master_to_local, std::memory_order_relaxed);
}

void visualiser_set_publish(void (*publish)(const vis_frame_t *))
{
    s_publish.store(publish, std::memory_order_relaxed);
}

void visualiser_set_ml_publish(void (*publish)(const ml_result_t *r))
{
    s_ml_publish.store(publish, std::memory_order_relaxed);
}

const char *visualiser_ml_source_name(void)
{
#if DF_RUNS_ANALYSERS
    return "local";
#else
    return "remote";
#endif
}

void visualiser_submit_ml(const ml_result_t *r)
{
#if DF_RUNS_ANALYSERS
    /* This unit computes its own. Accepting somebody else's as well would put
     * two answers to the same question in one slot, alternating by whichever
     * arrived last -- the same reason visualiser_submit_frame() is a no-op on a
     * unit doing its own analysis. */
    (void)r;
#else
    if (!r || r->analyser >= df::ML_SLOTS) {
        return;
    }

    /*
     * Is this the model this unit expects in that slot?
     *
     * A result from a different model is the one difference that makes two
     * strips disagree while every counter looks healthy, so it is said out loud
     * -- once, because it is a property of the pair of builds and will not stop
     * happening, and a complaint per result would bury everything else.
     *
     * Reported, not refused. A floor deliberately running a bigger model on the
     * hub than a satellite could hold is a case this is meant to serve; what it
     * must not be is a surprise.
     */
    if (df::Analyser *a = df::analyser_at(r->analyser)) {
        if (a->spec().model_id != r->model_id) {
            static bool told[df::ML_SLOTS];
            if (!told[r->analyser]) {
                told[r->analyser] = true;
                ESP_LOGW(TAG, "slot %u carries model %u, this build expects %u "
                              "(\"%s\") -- the strips will follow the sender",
                         r->analyser, r->model_id, a->spec().model_id, a->spec().name);
            }
        }
    }

    df::Result local;
    from_wire(r, local);
    /* A failed publish is already counted as a latch overrun, which is where
     * a reader would look for it -- it means the render stage is not draining,
     * and that is the same fault whichever side filled the slot. */
    if (s_latch.publish(r->analyser, local)) {
        bump(s_ml_results);
    }
#endif
}

void visualiser_submit_frame(const vis_frame_t *f)
{
#if DF_TAKES_REMOTE_FRAMES
    if (!f) {
        return;
    }
    /*
     * Is the sender cutting the same grid this unit is built for?
     *
     * Nothing here analyses audio, so the hop is not a setting this unit obeys
     * -- it is a statement about the unit SENDING the frames. It still has to be
     * right, because FRAME_RING is sized from it: a hub at hop 256 feeding a
     * satellite left at 1024 gives that satellite 186 ms of queue against
     * ~200 ms of audio lead, so it silently overruns and drops frames it should
     * have drawn.
     *
     * No protocol change is needed to check it. Consecutive frames are one hop
     * apart by construction, so the difference of their due_us IS the sender's
     * hop. The index test skips the first frame and any gap or flush, where the
     * difference would be several hops and mean nothing.
     *
     * An eighth is a deliberately loose band. The sender's due_us comes from an
     * integer division, so successive differences alternate by a microsecond,
     * while the supported hops are a factor of two apart -- rounding cannot trip
     * this and a real mismatch cannot hide under it.
     *
     * Said once, and not acted on: the sizes are compiled in, so the honest
     * response is to name the fault rather than half-absorb it.
     */
    {
        static int64_t prev_index = -1, prev_due = 0;
        const int64_t want = (int64_t)HOP_N * 1000000 /
                             s_rate.load(std::memory_order_relaxed);
        if (f->index == prev_index + 1 && want > 0) {
            const int64_t got = f->due_us - prev_due;
            const int64_t err = got > want ? got - want : want - got;
            if (err > want / 8) {
                static bool told = false;
                if (!told) {
                    told = true;
                    ESP_LOGE(TAG, "sender's hop is %lld us, this build expects %lld us "
                                  "(hop %d) -- frames will be queued for the wrong "
                                  "depth; set DANCEFLOOR_LED_HOP to match the hub",
                             (long long)got, (long long)want, HOP_N);
                }
            }
        }
        prev_index = f->index;
        prev_due   = f->due_us;
    }

    df::Frame local;
    from_wire(f, local);
    /* A frame off the wire carries no results: this unit's slots are all
     * latched, and the results that fill them arrive as their own messages. */
    if (enqueue(local, nullptr)) {
        bump(s_frames);
        if (local.onset) bump(s_onsets);
        if (local.boom) bump(s_booms);
    }
#else
    /*
     * Ignored, and silently, because it is configuration rather than a fault: a
     * unit doing its own analysis has a complete timeline already, and drawing
     * somebody else's alongside it would interleave two.
     */
    (void)f;
#endif
}

int visualiser_hop(void)
{
    /*
     * Reported rather than merely compiled in, because two units on different
     * hops cut different windows and reach different decisions -- and on the
     * LOCAL path nothing crosses between the boards, so no unit can detect that
     * for itself. Making it printable is the most that path can offer, and it
     * is enough: it is the same way the audio's own agreement is checked, by
     * putting two consoles side by side.
     */
    return HOP_N;
}

const char *visualiser_source_name(void)
{
#if CONFIG_DANCEFLOOR_LED_SOURCE_REMOTE
    return "remote";
#else
    return "local";
#endif
}

void visualiser_flush(void)
{
    /* Release, so everything the caller did to establish the new timeline is
     * visible to the render task before it acts on this. */
    s_fq_flush.fetch_add(1, std::memory_order_release);
}

void visualiser_feed(const uint8_t *pcm, uint32_t len, int64_t due_master_us)
{
    /*
     * Null on a unit built to take frames from elsewhere -- no stream buffer is
     * created and no analysis task runs, so audio handed here has nothing to be
     * handed to. Callers are not conditionalised on the source: the audio path
     * should not have to know, and the check was already here for the case
     * where the visualiser is disabled entirely.
     */
    if (!pcm_stream) {
        return;
    }

    /*
     * Is the count still describing the timeline?
     *
     * Cheap, because both halves are already here: `due_master_us` is where the
     * timeline says this audio is, and the byte count since the last alignment
     * is where this file believes it is. Nothing else has to be told anything.
     *
     * Checked before the exchange below so a drift found here is served by the
     * same alignment path as an explicitly requested one, in this same call.
     *
     * Not while one is already in flight: an alignment that is still discarding
     * frames has not published its new origin yet, so this would measure the
     * same drift against the old one and re-arm on every feed until the discard
     * finished -- harmless, since the grid it recomputes is absolute, but it
     * would make the counter report four events where there was one.
     */
    const int64_t rate = s_rate.load(std::memory_order_relaxed);

    if (due_master_us > 0 && s_ref_valid && s_skip_frames <= 0 && !s_mark_align_point &&
        !s_align_pending.load(std::memory_order_relaxed)) {
        const int64_t frames = static_cast<int64_t>(s_sent_total - s_ref_byte) / FRAME_BYTES;
        const int64_t drift = due_master_us - (s_ref_due + frames * 1000000LL / rate);
        if (drift > ALIGN_DRIFT_US || drift < -ALIGN_DRIFT_US) {
            s_last_drift_us.store(static_cast<int32_t>(drift), std::memory_order_relaxed);
            bump(s_drifts);
            s_align_pending.store(true, std::memory_order_relaxed);
        }
    }

    /* Exchange rather than test-then-clear: a request raised by another task
     * while this one is mid-alignment would otherwise be overwritten. */
    if (due_master_us > 0 && s_align_pending.exchange(false, std::memory_order_relaxed)) {
        const int64_t idx = (due_master_us * rate) / 1000000;
        const int32_t into_hop = static_cast<int32_t>(idx % HOP_N);
        s_skip_frames = into_hop ? (HOP_N - into_hop) : 0;
        s_pending_block_index = (idx + s_skip_frames) / HOP_N;
        s_mark_align_point = true;
        bump(s_aligns);
    }
    if (s_skip_frames > 0) {
        const uint32_t have = len / FRAME_BYTES;
        const uint32_t drop = s_skip_frames < static_cast<int32_t>(have)
                              ? static_cast<uint32_t>(s_skip_frames) : have;
        s_skip_frames -= static_cast<int32_t>(drop);
        pcm += drop * FRAME_BYTES;
        len -= drop * FRAME_BYTES;
        if (len == 0) {
            return;
        }
    }

    /* Publish where aligned audio begins, and which block that is. The
     * generation goes last, and with release ordering: it is what the analysis
     * task watches, so the two values must already be visible when it changes. */
    if (s_mark_align_point) {
        s_align_block_index.store(s_pending_block_index, std::memory_order_relaxed);
        s_align_at_byte.store(s_sent_total, std::memory_order_relaxed);
        s_align_gen.fetch_add(1, std::memory_order_release);
        /* The same origin the reader is about to adopt, in the units the drift
         * check needs: the scheduled instant of the first aligned frame, beside
         * the byte it starts at. Plain stores -- the feed task is the only
         * reader of these. */
        s_ref_due = s_pending_block_index * HOP_N * 1000000LL / rate;
        s_ref_byte = s_sent_total;
        s_ref_valid = true;
        s_mark_align_point = false;
    }

    /* 0 ticks: never block the audio task. A short send means the analysis fell
     * behind and audio was lost, which both breaks the alignment and makes this
     * unit analyse different audio from its neighbours -- count it and re-align
     * rather than losing it silently. */
    const size_t sent = xStreamBufferSend(pcm_stream, pcm, len, 0);
    s_sent_total += sent;
    if (sent < len) {
        bump(s_dropped, len - sent);
        s_align_pending.store(true, std::memory_order_relaxed);
    }
}

void visualiser_realign(void)
{
    /* Callable from any task: the flag is atomic and the audio task clears it
     * with an exchange. The next feed carrying a scheduled instant re-derives
     * the origin from it. */
    s_align_pending.store(true, std::memory_order_relaxed);
}

void visualiser_set_rate(uint32_t hz)
{
    /* Nothing computed from a stream field may divide the conversions below.
     * Same guard as retune_dac(): a rate outside this is a broken number,
     * whoever sent it, and using it would be worse than ignoring it. */
    if (hz < 8000 || hz > 192000) {
        ESP_LOGE(TAG, "ignoring a stream rate of %" PRIu32 " Hz -- not a sample rate", hz);
        return;
    }
    /* Called from wherever each unit learns the rate, which is often and mostly
     * says the same thing. */
    if (s_rate.exchange(hz, std::memory_order_relaxed) == hz) {
        return;
    }
    /*
     * The conversion between an instant and a sample position has just changed,
     * so the origin derived under the old one describes nothing. The analysis
     * task re-cuts its bands when it sees the new value; this side only has to
     * ask for a fresh origin.
     */
    s_align_pending.store(true, std::memory_order_relaxed);
    ESP_LOGW(TAG, "stream rate is %" PRIu32 " Hz", hz);
}

void visualiser_set_pattern(const char *name)
{
    df::Pattern *p = df::pattern_by_name(name);
    if (!p) {
        ESP_LOGW(TAG, "no pattern named \"%s\"", name ? name : "(null)");
        return;
    }
    p->reset();
    pattern = p;
    ESP_LOGI(TAG, "pattern: %s", p->name());
}

void visualiser_start(void)
{
#if DF_ANALYSES_AUDIO
    /* Trigger level is one HOP: waking the task for a handful of bytes at a time
     * is pure overhead now that partial reads accumulate, and once the first
     * window is held a frame needs only a hop of fresh audio to produce the
     * next one. */
    pcm_stream = xStreamBufferCreate(STREAM_BYTES, HOP_BYTES);
    assert(pcm_stream);
#endif

#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER
    {
        gpio_config_t m = {};
        m.pin_bit_mask = 1ULL << CONFIG_DANCEFLOOR_LED_MARKER_GPIO;
        m.mode = GPIO_MODE_OUTPUT;
        ESP_ERROR_CHECK(gpio_config(&m));
        /*
         * Drive it dark before anything else can look at it. gpio_config()
         * leaves an output at 0, which on an active-low LED is LIT -- so
         * without this the marker glows from boot and stays glowing until the
         * first frame arrives to lower it. On a unit that never receives audio
         * it would glow forever, which is the one state this LED must not have:
         * a solid LED is what "no timeline" is supposed to look like.
         */
        gpio_set_level(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_MARKER_GPIO),
                       LED_MARKER_OFF);
        ESP_LOGW(TAG, "LED marker on GPIO %d, active %s, one flash per "
                      "master-clock second -- every unit flashes together; "
                      "nothing corrects on it",
                 CONFIG_DANCEFLOOR_LED_MARKER_GPIO,
                 LED_MARKER_ON == 0 ? "low" : "high");
    }
#endif

#if DF_ANALYSES_AUDIO
    /* Whatever has been set by now, which is the default unless the stream was
     * already running when this was called. Either way the task re-inits if it
     * changes. */
    analysis.init(static_cast<int>(s_rate.load(std::memory_order_relaxed)));
#endif

    /*
     * Decide once, here, which slots this unit fills itself and which it waits
     * for -- see ResultLatch::set_latched().
     *
     * Every slot is latched by default, so a slot with no analyser behind it,
     * and every slot on a unit that is given its results, reports result_none()
     * forever rather than whatever the array happened to hold. Only a fast-lane
     * analyser this unit actually runs is cleared.
     *
     * Printed, because an unintended mismatch across a floor is the expensive
     * bug and this is half of what decides it -- the same reasoning that puts
     * the source and the hop on the startup line below.
     */
    {
        const int rate = static_cast<int>(s_rate.load(std::memory_order_relaxed));
        for (int i = 0; i < df::ML_SLOTS; i++) {
            s_latch.set_latched(i, true);
            df::Analyser *a = df::analyser_at(i);
            if (!a) {
                continue;
            }
            const df::AnalyserSpec &sp = a->spec();
#if DF_ANALYSES_AUDIO && DF_RUNS_ANALYSERS
            if (sp.lane == df::Lane::Fast) {
                if (a->init(rate)) {
                    s_latch.set_latched(i, false);
                } else {
                    ESP_LOGE(TAG, "analyser \"%s\" refused to start -- slot %d idle",
                             sp.name, i);
                    continue;
                }
            }
#endif
            ESP_LOGI(TAG, "analyser %d: \"%s\" model %u | %s lane | %d-sample window, "
                          "hop %d @ %d Hz | shown %lld us late | %s",
                     i, sp.name, (unsigned)sp.model_id,
                     sp.lane == df::Lane::Fast ? "fast" : "slow",
                     sp.window_n, sp.hop_n,
                     sp.rate_hz ? sp.rate_hz : rate,
                     (long long)sp.present_delay_us,
                     DF_RUNS_ANALYSERS
                         ? (sp.lane == df::Lane::Fast ? "computed here, in the frame"
                                                      : "computed here, through the latch")
                         : "given to this unit");
        }

#if DF_ANALYSES_AUDIO && DF_RUNS_ANALYSERS
        /* Starts nothing if no analyser is slow, so a build without one pays no
         * task, no stack and no 4 kB filter table. */
        df::ml_lane_start(&s_latch, rate, publish_result);
#endif
    }

    /* Configured name if it resolves, first pattern otherwise. A typo should
     * cost a line in the log, not a dark floor. */
    pattern = df::pattern_at(0);
    if (df::Pattern *p = df::pattern_by_name(CONFIG_DANCEFLOOR_LED_PATTERN)) {
        pattern = p;
    } else {
        ESP_LOGW(TAG, "no pattern named \"%s\" -- falling back to \"%s\"",
                 CONFIG_DANCEFLOOR_LED_PATTERN, pattern ? pattern->name() : "none");
    }

    /*
     * SPI + DMA, not bit-banging and not RMT.
     *
     * Bit-banging disables interrupts for the whole strip update, starving the
     * I2S DMA. RMT does not survive continuous refresh on the ESP32 -- see
     * components/led_strip_wrapper/README.md for the failure and why SPI has no
     * equivalent.
     */
    strip.emplace(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_GPIO), LED_COUNT,
                  STRIP_TYPE, LedStrip::Backend::SPI);
    strip->clear();
    strip->show();

    /*
     * Core 1 alongside playback, at a lower priority than it: a dropped frame
     * here is invisible, dropped audio is not.
     *
     * Two tasks, because the two halves are paced by different things. Analysis
     * runs when audio is available and may run well ahead of the sound; the
     * render runs on the clock, and its whole job is to be late for nothing.
     * Render sits one priority above analysis for that reason -- if the board is
     * busy, a frame drawn on time from slightly stale analysis beats a frame
     * drawn late from fresh analysis, and only one of the two is visible.
     */
#if DF_ANALYSES_AUDIO
    xTaskCreatePinnedToCore(visualiser_task, "vis", 4096, nullptr, 4, nullptr, 1);
#endif
    xTaskCreatePinnedToCore(render_task, "vis-draw", 3072, nullptr, 5, nullptr, 1);
    /* The source and the hop are on this line because an unintended mismatch
     * across a floor is the expensive bug, and two consoles side by side should
     * settle it. The frame rate is spelled out rather than left to be divided,
     * since it is the number that actually changed. */
    ESP_LOGW(TAG, "started: %lu LEDs on GPIO %d at %d%% brightness, pattern %s, "
                  "frames %s | window %d, hop %d (%.1f frames/s at %" PRIu32 " Hz)",
             LED_COUNT, CONFIG_DANCEFLOOR_LED_GPIO, CONFIG_DANCEFLOOR_LED_BRIGHTNESS,
             pattern ? pattern->name() : "none", visualiser_source_name(),
             FFT_N, HOP_N,
             (double)s_rate.load(std::memory_order_relaxed) / HOP_N,
             s_rate.load(std::memory_order_relaxed));
}
