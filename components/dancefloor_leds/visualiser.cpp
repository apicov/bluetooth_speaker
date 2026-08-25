
/**
 * @file visualiser.cpp
 * @brief The firmware half of the visualiser: the tasks, the clock, the strip
 *        and the block alignment that keeps two units cutting the same
 *        windows.
 *
 * Everything portable is next door -- df::Analysis, the patterns, the analyser
 * lanes and the latch all touch no clock and no hardware, and the host harness
 * compiles them unchanged. What is here is what only a board can do.
 *
 * TWO STAGES, AND THE SPLIT IS THE DESIGN. visualiser_task() ANALYSES as fast
 * as audio arrives, which is up to a whole playback lead ahead of when that
 * audio is heard; render_task() DRAWS each frame at the instant the frame
 * itself names. That is what lets analysis be fed from the arrival path
 * (visualiser_feed()) without the lights running ahead of the speaker, and it
 * is what gives a slow analyser the whole lead as working time.
 *
 * ALIGNMENT IS BY COUNTING, NOT BY CLOCK. A frame's index and its instant are
 * derived from how much audio has been fed since an origin established once
 * against the scheduled timeline. Every unit gets the same stamps for the same
 * audio, so every unit cuts its windows at the same sample positions without
 * exchanging anything -- and nothing in the analysis path reads a clock, which
 * is what stops two boards a few milliseconds apart from splitting transients
 * differently.
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

#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER
#include "driver/gpio.h"
#endif

#include "analysis.hpp"
#include "patterns.hpp"
#include "ml_lane.hpp"
#include "result_latch.hpp"
#include "led_strip_wrapper.hpp"

namespace {

/** @brief Log tag. */
constexpr const char *TAG = "vis";

/*
 * THREE CAPABILITIES, NAMED SEPARATELY, because they are three questions and
 * a unit answers them independently: does it compute frames from audio, does
 * it accept frames computed elsewhere, and does it run the pluggable
 * analysers. The source choice decides the first two and they are mutually
 * exclusive -- a unit doing its own analysis AND accepting somebody else's
 * would draw two timelines at once.
 */
#if CONFIG_DANCEFLOOR_LED_SOURCE_REMOTE
#define DF_ANALYSES_AUDIO      0
#define DF_TAKES_REMOTE_FRAMES 1
#else
/** @brief Whether this unit computes its own frames from audio. */
#define DF_ANALYSES_AUDIO      1
/** @brief Whether this unit takes frames computed elsewhere. Mutually
 *         exclusive with #DF_ANALYSES_AUDIO: a unit doing both would draw two
 *         timelines at once. */
#define DF_TAKES_REMOTE_FRAMES 0
#endif

#if CONFIG_DANCEFLOOR_ML
/** @brief Whether this unit runs the pluggable analysers. */
#define DF_RUNS_ANALYSERS 1
#else
#define DF_RUNS_ANALYSERS 0
#endif

/* A frame off the wire carries no spectrum, and the spectrum is an analyser's
 * entire input -- see vis_frame_t. So the combination is refused at build time
 * rather than producing an analyser that silently reads zeros. */
#if DF_TAKES_REMOTE_FRAMES && DF_RUNS_ANALYSERS
#error "DANCEFLOOR_ML needs LED_SOURCE_LOCAL: a frame off the wire carries no spectrum for an analyser to read (see vis_frame_t)."
#endif

using df::FFT_N;
using df::HOP_N;
using df::TAIL_N;
using df::RATE;
using df::CHANNELS;

/** @brief The analysis stream buffer: several windows deep, so a burst of
 *         audio from a decoder lump is absorbed rather than dropped. Dropping
 *         here breaks the count the alignment rests on, which is why
 *         visualiser_feed() re-aligns whenever it happens. */
constexpr int STREAM_BYTES = FFT_N * CHANNELS * (int)sizeof(int16_t) * 8;
/** @brief Bytes per interleaved audio frame. */
constexpr uint32_t FRAME_BYTES = CHANNELS * sizeof(int16_t);

/** @brief Bytes the analysis advances between windows -- the stream buffer's
 *         trigger level, so the task wakes with a whole hop available. */
constexpr size_t HOP_BYTES  = (size_t)HOP_N * CHANNELS * sizeof(int16_t);
/** @brief Bytes one window carries over into the next; zero when they do not
 *         overlap, which is why the slide that keeps it compiles away. */
constexpr size_t TAIL_BYTES = (size_t)TAIL_N * CHANNELS * sizeof(int16_t);

/** @brief LEDs on the strip. */
constexpr uint32_t LED_COUNT = CONFIG_DANCEFLOOR_LED_COUNT;

#ifndef CONFIG_DANCEFLOOR_LOG_PERIOD_S
#define CONFIG_DANCEFLOOR_LOG_PERIOD_S 20
#endif
/** @brief How often the periodic lines print, when nothing has gone wrong.
 *         Anything that HAS gone wrong prints immediately instead. */
constexpr int64_t LED_LOG_PERIOD_US = CONFIG_DANCEFLOOR_LOG_PERIOD_S * 1000000LL;

#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER

/**
 * @brief One flash per master-clock second, while audio is being drawn.
 *
 * The flash instant is derived from the frame's own master-clock label, not
 * from a local timer, so two units flash together if and only if they are
 * drawing the same audio at the same time. That is the whole instrument: the
 * eye compares the FLASHES between units and nothing else.
 */
constexpr int64_t LED_MARKER_PERIOD_US = 1000000;

/** @brief How long the flash stays lit -- long enough to see across a field,
 *         short enough not to blur into the next one. */
constexpr int64_t LED_MARKER_HIGH_US = 40000;

/** @brief With no frame drawn for this long, the marker goes back to its idle
 *         level: dark if not joined, solid if joined. See
 *         visualiser_marker_set_link(). */
constexpr int64_t MARKER_IDLE_US = 1500000;

/* An LED wired to VCC is lit by pulling the pin LOW, so the two levels are a
 * wiring fact rather than a preference. */
#if CONFIG_DANCEFLOOR_LED_MARKER_ACTIVE_LOW
constexpr int LED_MARKER_ON  = 0;
constexpr int LED_MARKER_OFF = 1;
#else
constexpr int LED_MARKER_ON  = 1;
constexpr int LED_MARKER_OFF = 0;
#endif

/** @brief Whether this unit is on the floor; see visualiser_marker_set_link().
 *         Written from any task, read by the render task. */
std::atomic<bool> s_marker_link{false};

/**
 * @brief Drive the marker pin, skipping a write that would change nothing.
 *
 * The cache starts at a value no level can equal, so the first write always
 * reaches the pin whatever it asks for -- which is what lets
 * visualiser_marker_busy() touch the pin directly beforehand without leaving
 * this out of step with the hardware.
 *
 * @param level  LED_MARKER_ON or LED_MARKER_OFF.
 */
void marker_write(int level)
{
    static int shown = -1;
    if (level == shown) return;
    shown = level;
    gpio_set_level(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_MARKER_GPIO), level);
}

/** @brief Half-period of the boot-survey blink: fast enough to read as "busy"
 *         rather than as the one-per-second flash it must not be mistaken
 *         for. */
constexpr int64_t MARKER_BUSY_TOGGLE_US = 80 * 1000;

/** @brief The blink's own timer; non-null only while it is running, which is
 *         also how visualiser_marker_busy() is made idempotent. */
static esp_timer_handle_t s_marker_busy_timer;
/** @brief Its current level. */
static int s_marker_busy_level = LED_MARKER_OFF;

/**
 * @brief Toggle the marker pin. Touches the pin directly rather than through
 *        marker_write(), so its cache is left untouched -- see marker_write().
 * @param  Unused; the timer callback signature.
 */
static void marker_busy_cb(void *)
{
    s_marker_busy_level = (s_marker_busy_level == LED_MARKER_ON) ? LED_MARKER_OFF
                                                                 : LED_MARKER_ON;
    gpio_set_level(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_MARKER_GPIO),
                   s_marker_busy_level);
}

/** @brief What the marker shows between flashes: solid when joined, dark when
 *         not. @return The level. */
int marker_idle_level()
{
    return s_marker_link.load(std::memory_order_relaxed) ? LED_MARKER_ON
                                                         : LED_MARKER_OFF;
}

/**
 * @brief Raise and lower the marker as its scheduled instants come round.
 *
 * @param now              Local-clock now.
 * @param[in,out] flash_at Local instant to raise it at; cleared when done.
 * @param[in,out] lower_at ...and to lower it at, set by the raise.
 */
void marker_service(int64_t now, int64_t *flash_at, int64_t *lower_at)
{
    if (*lower_at && now >= *lower_at) {
        *lower_at = 0;
        marker_write(LED_MARKER_OFF);
    }
    if (*flash_at && now >= *flash_at) {
        *flash_at = 0;
        *lower_at = now + LED_MARKER_HIGH_US;
        marker_write(LED_MARKER_ON);

        /* Said a few times at the start of a run, because a dark marker is
         * ambiguous between "not firing" and "not on this pin", and only the
         * console can tell them apart. */
        static int told;
        if (told < 3) {
            told++;
            ESP_LOGW(TAG, "LED marker fired on GPIO %d (%d of 3) -- if the "
                          "LED is dark, this board's LED is not on that pin",
                     CONFIG_DANCEFLOOR_LED_MARKER_GPIO, told);
        }
    }
}

/**
 * @brief Shorten a nap so the render task wakes for the next marker edge.
 *
 * The marker is serviced from the render loop rather than from a timer, so a
 * long nap would delay a flash past the instant that gives it its meaning.
 *
 * @param nap_ms    The nap the loop wanted.
 * @param flash_at  Next raise, or 0.
 * @param lower_at  Next lower, or 0.
 * @return The nap to take; at least 1 ms when an edge is due.
 */
int64_t marker_clamp_nap(int64_t nap_ms, int64_t flash_at, int64_t lower_at)
{
    int64_t edge = flash_at;
    if (!edge || (lower_at && lower_at < edge)) {
        edge = lower_at;
    }
    if (!edge) {
        return nap_ms;
    }
    const int64_t left = (edge - esp_timer_get_time()) / 1000;
    if (left >= nap_ms) {
        return nap_ms;
    }
    return left > 1 ? left : 1;
}
#endif

/** @brief Global scale applied at the last moment, in show(). A cap rather
 *         than a preference: these strips at full scale draw more current than
 *         the supply this floor runs on. */
constexpr float BRIGHTNESS = CONFIG_DANCEFLOOR_LED_BRIGHTNESS / 100.0f;

/** @brief Which strip is wired to this board; see LedStrip::Type. */
#if   CONFIG_DANCEFLOOR_LED_TYPE_SK6812
constexpr LedStrip::Type STRIP_TYPE = LedStrip::Type::SK6812_RGBW;
#elif CONFIG_DANCEFLOOR_LED_TYPE_WS2811
constexpr LedStrip::Type STRIP_TYPE = LedStrip::Type::WS2811;
#else
constexpr LedStrip::Type STRIP_TYPE = LedStrip::Type::WS2812;
#endif

/** @brief Audio in, from visualiser_feed() to the analysis task. */
StreamBufferHandle_t pcm_stream;
/** @brief The strip, constructed in visualiser_start() -- optional because it
 *         has no default constructor and must not be built before the GPIO
 *         configuration around it. */
std::optional<LedStrip> strip;

/** @brief The FFT and the detectors. Owned by the analysis task alone. */
#if DF_ANALYSES_AUDIO
df::Analysis analysis;
#endif

/** @brief The stream rate, as visualiser_set_rate() was told it. Every
 *         instant-to-position conversion here goes through it. */
std::atomic<uint32_t> s_rate{df::RATE};
/** @brief The pattern being drawn. */
df::Pattern *pattern = nullptr;
/** @brief The render task's pixel buffer. */
uint8_t pixels[LED_COUNT * 3];

/*
 * THE BLOCK ALIGNMENT.
 *
 * A window may begin only at a sample position that is a whole number of hops
 * from an origin every unit derives the same way. visualiser_feed() picks that
 * origin from the first scheduled instant it is handed, drops the part-hop in
 * front of it, and tells the analysis task where in the byte stream the new
 * origin falls; the task then counts from there. Nothing is exchanged between
 * units, and nothing reads a clock.
 */
/** @brief An alignment is wanted: at startup, after a rate change, after a
 *         splice (visualiser_realign()), after a dropped feed, and whenever
 *         the drift check below fires. */
std::atomic<bool> s_align_pending{true};
/** @brief Set once the part-hop has been dropped, so the next bytes fed carry
 *         the new origin. Feed-side only. */
bool     s_mark_align_point;
/** @brief Audio frames still to drop before the origin. Feed-side only. */
int32_t  s_skip_frames;
/** @brief The block index the origin lands on. Feed-side only. */
int64_t  s_pending_block_index;

/** @brief Where in the byte stream the origin falls, for the analysis task. */
std::atomic<uint32_t> s_align_at_byte;
/** @brief ...and which block index it is. */
std::atomic<long long> s_align_block_index;

/** @brief Bumped last, with release ordering, so the analysis task never reads
 *         a half-published origin: it sees the generation change only after
 *         both values above are in place. */
std::atomic<uint32_t> s_align_gen;
/** @brief Bytes handed to the stream buffer since boot. Wraps, and is only
 *         ever used as a DIFFERENCE, which is why that is harmless. */
uint32_t s_sent_total;

/** @brief The instant the origin's block is due... */
int64_t  s_ref_due;
/** @brief ...and the byte position it sat at. Together they are what the drift
 *         check below extrapolates from. */
uint32_t s_ref_byte;
/** @brief Whether that pair means anything yet. */
bool     s_ref_valid;

/**
 * @brief How far the counted position may drift from the scheduled one before
 *        the origin is re-derived on its own.
 *
 * Counting is only correct while every sample fed was also heard. A splice
 * breaks that, and visualiser_realign() is the caller saying so -- but a
 * caller that forgets, or an event nobody thought to report, would otherwise
 * mislabel every frame from then on, silently and for good.
 *
 * So the feed also CHECKS: it extrapolates from the reference pair above and
 * compares against the instant it was handed. Past this tolerance it
 * re-derives. Set well above the jitter in a scheduled instant and well below
 * anything a listener could see.
 */
constexpr int64_t ALIGN_DRIFT_US = 2000;

/** @brief Bytes the stream buffer refused; each one forces a re-alignment. */
std::atomic<uint32_t> s_dropped;
/** @brief Alignments performed. More than one per window is a unit that cannot
 *         hold its count, which is why the log line prints immediately then. */
std::atomic<uint32_t> s_aligns;
/** @brief Onsets this window. */
std::atomic<uint32_t> s_onsets;
/** @brief Frames this window. */
std::atomic<uint32_t> s_frames;

/** @brief Booms this window; see df::Frame::boom. */
std::atomic<uint32_t> s_booms;
/** @brief Booms whose flux landed within a tenth of the threshold -- the ones
 *         where two units with slightly different histories could disagree, so
 *         a high count is the warning that the detector is running on the
 *         margin rather than on the signal. */
std::atomic<uint32_t> s_marginal;

/** @brief Times the drift check re-derived the origin. */
std::atomic<uint32_t> s_drifts;
/** @brief ...and by how much, last time. Signed, because which way it drifted
 *         says whether audio was inserted or skipped. */
std::atomic<int32_t>  s_last_drift_us;

/** @brief Read a counter and clear it. @param c The counter.
 *  @return What it held. */
[[maybe_unused]] uint32_t take(std::atomic<uint32_t> &c) { return c.exchange(0, std::memory_order_relaxed); }
/** @brief Add to a counter. @param c The counter. @param n How much. */
void bump(std::atomic<uint32_t> &c, uint32_t n = 1) { c.fetch_add(n, std::memory_order_relaxed); }

/** @brief Keep the larger of a gauge and a new reading, against a concurrent
 *         reader that clears it.
 *  @param c  The gauge.
 *  @param v  The reading. */
void note_max(std::atomic<uint32_t> &c, uint32_t v)
{
    uint32_t prev = c.load(std::memory_order_relaxed);
    while (v > prev && !c.compare_exchange_weak(prev, v, std::memory_order_relaxed)) {
    }
}

/**
 * @brief Frames the queue between the two stages holds.
 *
 * Deep enough to cover the whole playback lead, since that is exactly how far
 * ahead the analysis stage may run. Scaled by the overlap so a shorter hop --
 * which produces proportionally more frames for the same audio -- covers the
 * same span of TIME rather than the same count.
 */
constexpr uint32_t FRAME_RING = 32 * (FFT_N / HOP_N);
static_assert((FRAME_RING & (FRAME_RING - 1)) == 0,
              "FRAME_RING must be a power of two -- the uint32 wrap depends on it");
/** @brief Analysis to render. Single-producer, single-consumer, so no lock. */
df::Frame s_fq[FRAME_RING];
/** @brief The producer writes. */
std::atomic<uint32_t> s_fq_head;
/** @brief The consumer writes. */
std::atomic<uint32_t> s_fq_tail;
/** @brief Bumped by visualiser_flush(); the render task empties the queue when
 *         it changes. A counter rather than a flag, so a flush cannot be lost
 *         between two reads. */
std::atomic<uint32_t> s_fq_flush;
/** @brief Frames drawn well past the instant they named. */
std::atomic<uint32_t> s_late;
/** @brief Frames the queue could not take -- the render task is not keeping
 *         up, and the strip is about to fall behind the audio. */
std::atomic<uint32_t> s_overrun;

/** @brief Frames the slow lane could not take. */
[[maybe_unused]] std::atomic<uint32_t> s_ml_dropped;

#if DF_RUNS_ANALYSERS

void run_fast_lane(const uint8_t (&spec)[df::SPEC_BINS], int64_t index,
                   int64_t due_us, df::Result out[df::ML_SLOTS]);
#endif

/* The render stage's cost, split three ways, because they fail differently: a
 * slow PATTERN is a code problem, a slow SHOW is the strip driver or the bus,
 * and a late WAKE is the scheduler. A single total cannot tell them apart. */
/** @brief Total render time this window. */
std::atomic<uint32_t> s_render_sum;
/** @brief ...and the worst single one. */
std::atomic<uint32_t> s_render_max;
/** @brief ...over this many frames. */
std::atomic<uint32_t> s_render_n;

/** @brief Time in the pattern. */
std::atomic<uint32_t> s_pattern_sum;
/** @brief ...and the worst. */
std::atomic<uint32_t> s_pattern_max;
/** @brief Time pushing pixels to the strip. */
std::atomic<uint32_t> s_show_sum;
/** @brief ...and the worst. */
std::atomic<uint32_t> s_show_max;
/** @brief How late a nap overslept, summed -- the scheduler's own error, which
 *         bounds how precisely a frame can be drawn at its instant. */
std::atomic<uint32_t> s_wake_sum;
/** @brief ...and the worst. */
std::atomic<uint32_t> s_wake_max;
/** @brief ...over this many naps. */
std::atomic<uint32_t> s_wake_n;

/** @brief Times the strip was blanked for want of frames. */
std::atomic<uint32_t> s_idle_dark;

/** @brief Master-clock to local; see visualiser_set_clock(). Null on the hub,
 *         where the two are the same. */
std::atomic<int64_t (*)(int64_t)> s_to_local{nullptr};

/** @brief Where computed frames go; see visualiser_set_publish(). */
std::atomic<void (*)(const vis_frame_t *)> s_publish{nullptr};

static_assert(VIS_BANDS == BEAT_BANDS, "wire frame lost a band");

#if DF_ANALYSES_AUDIO
/** @brief Reduce a frame to what can leave this unit. See vis_frame_t for what
 *         is deliberately left behind.
 *  @param f       The frame.
 *  @param[out] w  The wire form. */
void to_wire(const df::Frame &f, vis_frame_t *w)
{
    w->due_us = f.due_us;
    w->index  = f.index;
    std::memcpy(w->band, f.band, sizeof(w->band));
}
#endif

#if DF_TAKES_REMOTE_FRAMES
/** @brief The detector a unit taking frames runs over the bands it is sent.
 *         Owned by whichever task calls visualiser_submit_frame(). */
df::RemoteDetect s_remote_detect;
#endif

#if DF_TAKES_REMOTE_FRAMES
/**
 * @brief Expand a wire frame into a local one.
 *
 * The spectrum is ZEROED rather than left undefined: it does not travel, and a
 * unit taking frames cannot run the analysers that would read it, so zero is
 * the honest value rather than a placeholder.
 *
 * @param w      The wire form.
 * @param[out] f The frame; its detector fields are filled afterwards, by
 *               df::RemoteDetect.
 */
void from_wire(const vis_frame_t *w, df::Frame &f)
{
    f.due_us = w->due_us;
    f.index  = w->index;
    std::memcpy(f.band, w->band, sizeof(f.band));
    f.mag  = nullptr;

    std::memset(f.spec, 0, sizeof(f.spec));
    f.unit = 0;
}
#endif

/**
 * @brief Put one frame on the queue for the render stage, filling its analyser
 *        slots on the way.
 *
 * The fast lane runs HERE rather than in the render stage, because its window
 * IS this frame's window and there is nothing to wait for. The slow lane is
 * only OFFERED the frame; its answer arrives later, through the latch.
 *
 * @param f  The frame.
 * @return false if the queue was full, counted as an overrun.
 */
bool enqueue(const df::Frame &f)
{
    const uint32_t head = s_fq_head.load(std::memory_order_relaxed);
    const uint32_t tail = s_fq_tail.load(std::memory_order_acquire);
    if (head - tail >= FRAME_RING) {
        bump(s_overrun);
        return false;
    }
    df::Frame &dst = s_fq[head % FRAME_RING];
    dst = f;

#if DF_RUNS_ANALYSERS
    df::Result fast_ml[df::ML_SLOTS];
    run_fast_lane(f.spec, f.index, f.due_us, fast_ml);
    for (int i = 0; i < df::ML_SLOTS; i++) {
        dst.ml[i] = fast_ml[i];
    }
    if (!df::ml_lane_feed(f.spec, f.index, f.due_us)) {
        bump(s_ml_dropped);
    }
#else
    /* Every slot cleared, so a unit that runs no analysers reports "nothing
     * said" rather than whatever the ring's previous occupant left. */
    for (int i = 0; i < df::ML_SLOTS; i++) {
        dst.ml[i] = df::result_none();
    }
#endif

    s_fq_head.store(head + 1, std::memory_order_release);
    return true;
}

/** @brief Where slow-lane and remote results wait for the frames they name. */
df::ResultLatch s_latch;

/** @brief Fast-lane results produced this window. */
[[maybe_unused]] std::atomic<uint32_t> s_ml_results;

/** @brief A frame this close to due is drawn now rather than napped for: a nap
 *         has coarser resolution than this anyway, so waiting would only make
 *         it later. */
constexpr int64_t RENDER_SLACK_US = 1000;

/** @brief Longest single nap. Bounds how long a flush or a marker edge waits,
 *         which is why the loop re-checks rather than sleeping to the next
 *         frame's instant in one go. */
constexpr int64_t RENDER_NAP_MS = 20;

/** @brief Past this much late, a frame is counted -- the strip is no longer
 *         showing what the speaker is playing. */
constexpr int64_t RENDER_LATE_US = 20000;

/** @brief With no frame for this long, blank the strip and reset the pattern:
 *         a stopped stream should leave the floor dark rather than frozen on
 *         its last colour. */
constexpr int64_t RENDER_IDLE_US = 500000;

/**
 * @brief Scale by BRIGHTNESS and push to the strip.
 *
 * A refresh failure is reported ONCE: a wedged strip fails on every frame, and
 * a line per frame would bury everything else on the console.
 *
 * @param rgb  LED_COUNT RGB triples.
 */
void show(const uint8_t *rgb)
{
    for (uint32_t i = 0; i < LED_COUNT; i++) {
        strip->set(i, static_cast<uint8_t>(rgb[3 * i + 0] * BRIGHTNESS),
                      static_cast<uint8_t>(rgb[3 * i + 1] * BRIGHTNESS),
                      static_cast<uint8_t>(rgb[3 * i + 2] * BRIGHTNESS));
    }

    if (const esp_err_t err = strip->show(); err != ESP_OK) {
        static bool reported = false;
        if (!reported) {
            reported = true;
            ESP_LOGE(TAG, "strip refresh failed: %s", esp_err_to_name(err));
        }
    }
}

#if DF_RUNS_ANALYSERS

/**
 * @brief Drive the portable fast lane and count what it produced.
 *
 * A thin wrapper over df::run_fast_lane(): the SKIP set is the firmware's to
 * know -- it is whichever slots the latch fills -- and the counting is a
 * firmware concern too. The lane itself stays portable so the host harness
 * runs the same code.
 *
 * @param spec      The frame's quantised spectrum.
 * @param index     Its place on the shared grid.
 * @param due_us    When its audio is heard.
 * @param[out] out  One result per slot.
 */
void run_fast_lane(const uint8_t (&spec)[df::SPEC_BINS], int64_t index,
                   int64_t due_us, df::Result out[df::ML_SLOTS])
{
    bool skip[df::ML_SLOTS];
    for (int i = 0; i < df::ML_SLOTS; i++) {
        skip[i] = s_latch.latched(i);
    }

    df::run_fast_lane(spec, index, due_us, skip, out);

    for (int i = 0; i < df::ML_SLOTS; i++) {
        if (df::result_valid(out[i])) {
            bump(s_ml_results);
        }
    }
}
#endif

#if DF_ANALYSES_AUDIO

/**
 * @brief The analysis stage: cut windows on the shared grid, transform them,
 *        and queue the frames.
 *
 * Runs as far ahead of the audio as the playback lead allows, which is the
 * whole point of separating it from the render stage.
 *
 * @param arg  Unused; the FreeRTOS task signature.
 */
void visualiser_task(void *arg)
{
    (void)arg;
    /* Static, not stack: a window is kilobytes and this task's stack is not. */
    static int16_t raw[FFT_N * CHANNELS];

    size_t   filled = 0;
    uint32_t recv_total = 0;
    uint32_t seen_gen = 0;
    int64_t  block_index = 0;
    int64_t  last_report_us = esp_timer_get_time();
    bool     starved_shown = false;
    uint32_t seen_rate = s_rate.load(std::memory_order_relaxed);

    int64_t  cost_analysis = 0, cost_analysis_max = 0;
    uint32_t cost_n = 0;

    int64_t  cost_fast = 0, cost_fast_max = 0;

    while (true) {

        const uint32_t rate = s_rate.load(std::memory_order_relaxed);
        if (rate != seen_rate) {
            seen_rate = rate;
            analysis.init(static_cast<int>(rate));

            for (int i = 0; i < df::ML_SLOTS; i++) {
                if (df::Analyser *a = df::analyser_at(i);
                    a && !s_latch.latched(i) && !a->init(static_cast<int>(rate))) {

                    s_latch.set_latched(i, true);
                    ESP_LOGE(TAG, "analyser \"%s\" cannot run at %" PRIu32 " Hz -- retired",
                             a->spec().name, rate);
                }
            }
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
         * A new origin has been published. Acquire, against the release in
         * visualiser_feed(): the byte position and the block index below are
         * only safe to read because the generation changed after they were
         * written.
         */
        const uint32_t gen = s_align_gen.load(std::memory_order_acquire);
        if (gen != seen_gen) {
            const uint32_t align_at = s_align_at_byte.load(std::memory_order_relaxed);
            /* Signed difference, so this survives the byte counter wrapping.
             * Negative means the origin is still ahead of what has been
             * received: drop what is held and wait for it. */
            const int32_t ahead = static_cast<int32_t>(recv_total - align_at);
            if (ahead < 0) {
                filled = 0;
                continue;
            }
            seen_gen = gen;

#if DF_RUNS_ANALYSERS
            df::ml_lane_restart();
#endif
            block_index = s_align_block_index.load(std::memory_order_relaxed);
            /* Keep only the bytes at or after the origin. Everything before
             * it belongs to the old count and would shift every window. */
            const size_t keep = static_cast<size_t>(ahead) < filled
                                ? static_cast<size_t>(ahead) : filled;
            std::memmove(raw, reinterpret_cast<uint8_t *>(raw) + (filled - keep), keep);
            filled = keep;
        }

        if (filled < sizeof(raw)) {
            /*
             * The stream ran dry mid-window. The audio either stopped or was
             * interrupted, and in both cases what arrives next does not
             * continue what is held -- so the queued frames are dropped and
             * the origin is re-derived rather than carrying a gap into the
             * count.
             *
             * Once per dry spell, not once per timeout: a stopped floor would
             * otherwise flush continuously.
             */
            if (got == 0 && !starved_shown) {
                starved_shown = true;
                visualiser_flush();

                s_align_pending.store(true, std::memory_order_relaxed);
            }
            continue;
        }
        starved_shown = false;

        /* The instant this window's first sample is heard, derived by COUNTING
         * from the shared origin -- never read from a clock. See the file
         * header. */
        const int64_t due_us = block_index * HOP_N * 1000000LL / rate;
        const int64_t t_in = esp_timer_get_time();
        const df::Frame &f = analysis.process(raw, block_index, due_us, 0);
        const int64_t t_analysed = esp_timer_get_time();

        const int64_t t_fast = esp_timer_get_time();

        /* Slide the overlap down and keep it for the next window. Compiles to
         * nothing when the windows do not overlap. */
        if constexpr (TAIL_BYTES > 0) {
            std::memmove(raw, reinterpret_cast<uint8_t *>(raw) + HOP_BYTES, TAIL_BYTES);
        }
        filled = TAIL_BYTES;

        block_index++;

        bump(s_frames);
        if (f.onset) bump(s_onsets);
        if (f.boom) bump(s_booms);
        /* A boom whose flux landed close to the threshold: the case where two
         * units with slightly different histories could decide differently.
         * See s_marginal. */
        if (f.boom_threshold > 0.0f &&
            std::fabs(f.boom_flux - f.boom_threshold) < 0.1f * f.boom_threshold) {
            bump(s_marginal);
        }

        enqueue(f);

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

        const int64_t now = esp_timer_get_time();
        const uint32_t dropped = s_dropped.load(std::memory_order_relaxed);
        const uint32_t aligns  = s_aligns.load(std::memory_order_relaxed);
        const uint32_t drifts  = s_drifts.load(std::memory_order_relaxed);

        /* Printed immediately when something has gone wrong, and otherwise on
         * the period: a drop, a drift or more than one alignment all mean the
         * count is not holding, and waiting for the next window to say so
         * loses the context it happened in. */
        if (dropped || drifts || aligns > 1 || now - last_report_us >= LED_LOG_PERIOD_US) {
            ESP_LOGI(TAG, "frames %" PRIu32 " | onsets %" PRIu32
                          " | booms %" PRIu32 " (marginal %" PRIu32 ")"
                          " | drop %" PRIu32 " B | aligns %" PRIu32
                          " | drift %" PRIu32 " (last %+ld us) | %s",
                     take(s_frames), take(s_onsets), take(s_booms), take(s_marginal),
                     take(s_dropped), take(s_aligns),
                     take(s_drifts), (long)s_last_drift_us.load(std::memory_order_relaxed),
                     pattern ? pattern->name() : "no pattern");

            const uint32_t rn = take(s_render_n), rsum = take(s_render_sum);
            const uint32_t wn = take(s_wake_n), wsum = take(s_wake_sum);
            const uint32_t psum = take(s_pattern_sum), ssum = take(s_show_sum);

#if DF_RUNS_ANALYSERS
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
#else
            ESP_LOGI(TAG, "ml: fast %lld/%lld us (mean/max) | no local lane"
                          " | results %" PRIu32
                          " | late %" PRIu32 " | overrun %" PRIu32,
                     cost_n ? cost_fast / cost_n : 0, cost_fast_max,
                     take(s_ml_results),
                     s_latch.take_late(), s_latch.take_overrun());
#endif
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

#endif

/**
 * @brief The render stage: draw each frame at the instant it names.
 *
 * Higher priority than the analysis task and on the same core: analysis may
 * run late without anything being visible, but a frame drawn late is the strip
 * losing step with the speaker.
 *
 * @param arg  Unused; the FreeRTOS task signature.
 */
void render_task(void *arg)
{
    (void)arg;
    uint32_t seen_flush = s_fq_flush.load(std::memory_order_relaxed);
    int64_t  drawn_at = 0;
#if !DF_ANALYSES_AUDIO
    int64_t  last_report_us = esp_timer_get_time();
#endif
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER

    int64_t flash_at = 0;   /* local instant to raise the marker at */
    int64_t lower_at = 0;   /* ...and to lower it */

    /* The last master-clock second a flash was scheduled for, so one second
     * produces one flash however many frames fall in it. */
    int64_t last_flash_sec = -1;

    int64_t marker_at   = 0;      /* when a frame was last drawn */
    bool    marker_idle = true;   /* ...and whether that was long enough ago */
#endif

    while (true) {
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER

        /* Serviced first, before anything that might nap: a flash delayed past
         * its instant is the one thing this instrument must not do. */
        if (!marker_idle) {
            marker_service(esp_timer_get_time(), &flash_at, &lower_at);
        }
#endif
        const uint32_t flush = s_fq_flush.load(std::memory_order_acquire);
        if (flush != seen_flush) {
            seen_flush = flush;

            /* Moving the TAIL is the flush: it is the consumer's index, so
             * this races nothing the producer is doing. */
            s_fq_tail.store(s_fq_head.load(std::memory_order_acquire),
                            std::memory_order_release);

            if (pattern) pattern->reset();

            s_latch.flush();
            std::memset(pixels, 0, sizeof(pixels));
            show(pixels);
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER

            /* The flash schedule was derived from a timeline that no longer
             * exists, so it goes with the frames. */
            last_flash_sec = -1;
            flash_at = 0;
            lower_at = 0;
            marker_write(marker_idle ? marker_idle_level() : LED_MARKER_OFF);
#endif
        }

#if !DF_ANALYSES_AUDIO

        /* On a unit that takes frames there is no analysis task, so the
         * periodic line has to come from here instead. */
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
                drawn_at = 0;
                if (pattern) pattern->reset();
                std::memset(pixels, 0, sizeof(pixels));
                show(pixels);
                bump(s_idle_dark);
            }
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER

            /* Nothing drawn for a while: back to the idle level, which says
             * whether this unit is still on the floor. */
            if (esp_timer_get_time() - marker_at > MARKER_IDLE_US) {
                marker_idle = true;
                flash_at = 0;
                lower_at = 0;
                marker_write(marker_idle_level());
            }
#endif
            {
                int64_t nap_ms = RENDER_NAP_MS;
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER

                nap_ms = marker_clamp_nap(nap_ms, flash_at, lower_at);
#endif
                vTaskDelay(pdMS_TO_TICKS(nap_ms));
            }
            continue;
        }

        /* When the frame at the tail is due, converted into THIS board's
         * clock. A frame with no instant -- one produced before any timeline
         * existed -- is drawn at once. */
        const int64_t due = s_fq[tail % FRAME_RING].due_us;
        int64_t wait = 0;
        if (due > 0) {
            const auto to_local = s_to_local.load(std::memory_order_relaxed);
            wait = (to_local ? to_local(due) : due) - esp_timer_get_time();
        }
        if (wait > RENDER_SLACK_US) {
            /* Napped in bounded steps rather than straight to the instant, so
             * a flush or a marker edge is never waited past. */
            int64_t nap = wait / 1000 < RENDER_NAP_MS ? wait / 1000 : RENDER_NAP_MS;
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER

            nap = marker_clamp_nap(nap, flash_at, lower_at);
#endif
            const int64_t asked = nap ? nap : 1;
            const int64_t before = esp_timer_get_time();
            vTaskDelay(pdMS_TO_TICKS(asked));

            /* How far the scheduler overslept, which is the floor on how
             * precisely any frame can be drawn at its instant. */
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

        /* The raw spectrum pointed into the Analysis that produced it and was
         * overwritten long ago -- see df::Frame::mag. Nulled rather than left
         * dangling. */
        f.mag = nullptr;
        s_fq_tail.store(tail + 1, std::memory_order_release);

        /* Latched HERE, at the moment of drawing, which is what gives the slow
         * lane the whole playback lead as working time -- see
         * df::ResultLatch. */
        {
            const int64_t hop_us =
                (int64_t)HOP_N * 1000000LL /
                (int64_t)s_rate.load(std::memory_order_relaxed);
            s_latch.take(f.due_us, hop_us, f.ml);
        }

#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER

        marker_at = t0;
        if (marker_idle) {
            /* Leaving idle: drop the solid "joined" level, so what follows is
             * unambiguously a flash rather than a level that happened to be
             * lit already. */
            marker_idle = false;
            lower_at = 0;
            marker_write(LED_MARKER_OFF);
        }

        if (f.due_us > 0) {
            const int64_t next_sec = f.due_us / LED_MARKER_PERIOD_US + 1;

            /* Scheduled from the FRAME's master-clock label, so two units
             * flash together exactly when they are drawing the same audio at
             * the same time. Re-derived while one is still pending, because
             * the clock conversion may have moved under it. */
            if (next_sec > last_flash_sec || flash_at) {
                last_flash_sec = next_sec;
                const int64_t at = next_sec * LED_MARKER_PERIOD_US;
                const auto to_local = s_to_local.load(std::memory_order_relaxed);
                flash_at = to_local ? to_local(at) : at;
            }
        }
#endif

        if (pattern) {
            /* Timed in two halves: the pattern and the strip write fail
             * differently and a single total cannot tell them apart. */
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

}

/* Declared in visualiser.h, like every entry point below it. */
void visualiser_marker_busy(bool on)
{
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER
    if (on) {
        if (s_marker_busy_timer) {
            return;
        }
        gpio_config_t m = {};
        m.pin_bit_mask = 1ULL << CONFIG_DANCEFLOOR_LED_MARKER_GPIO;
        m.mode = GPIO_MODE_OUTPUT;
        if (gpio_config(&m) != ESP_OK) {
            return;
        }
        esp_timer_create_args_t a = {};
        a.callback = marker_busy_cb;
        a.name = "marker-busy";
        if (esp_timer_create(&a, &s_marker_busy_timer) != ESP_OK) {
            s_marker_busy_timer = nullptr;
            return;
        }
        s_marker_busy_level = LED_MARKER_OFF;
        esp_timer_start_periodic(s_marker_busy_timer, MARKER_BUSY_TOGGLE_US);
    } else {
        if (!s_marker_busy_timer) {
            return;
        }
        esp_timer_stop(s_marker_busy_timer);
        esp_timer_delete(s_marker_busy_timer);
        s_marker_busy_timer = nullptr;

        /* Left dark, which is where visualiser_start() drives it anyway --
         * and marker_write()'s cache is untouched, so the render task's first
         * write still reaches the pin whatever it asks for. */
        s_marker_busy_level = LED_MARKER_OFF;
        gpio_set_level(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_MARKER_GPIO),
                       LED_MARKER_OFF);
    }
#else
    (void)on;
#endif
}

void visualiser_marker_set_link(bool up)
{
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER
    s_marker_link.store(up, std::memory_order_relaxed);
#else

    (void)up;
#endif
}

void visualiser_set_clock(int64_t (*master_to_local)(int64_t))
{
    s_to_local.store(master_to_local, std::memory_order_relaxed);
}

void visualiser_set_publish(void (*publish)(const vis_frame_t *))
{
    s_publish.store(publish, std::memory_order_relaxed);
}

void visualiser_submit_frame(const vis_frame_t *f)
{
#if DF_TAKES_REMOTE_FRAMES
    if (!f) {
        return;
    }

    /*
     * Does the sender's hop match this build's?
     *
     * Two units on different hops cut different windows and reach different
     * decisions, and nothing else here would notice: the frames arrive, they
     * are well formed, and they are queued for a depth this build's arithmetic
     * derives wrongly. The gap between two consecutive frames' instants IS the
     * sender's hop, so it can be checked against what this build expects.
     *
     * Said once. It is a build mismatch, so it will not stop being true.
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

    /* The detector's history was measured against bands cut at the old rate,
     * so a rate change drops it -- exactly where a local unit calls
     * df::Analysis::init(), and nowhere else. Matching those reset points is
     * part of matching its decisions. */
    {
        static uint32_t seen_rate = 0;
        const uint32_t rate = s_rate.load(std::memory_order_relaxed);
        if (rate != seen_rate) {
            seen_rate = rate;
            s_remote_detect.init();
        }
    }

    df::Frame local;
    from_wire(f, local);

    /* The onset and boom are DERIVED here, from the bands the sender put on
     * the wire, rather than being sent decided -- see vis_frame_t. */
    s_remote_detect.process(local.band, local.due_us, &local);

    if (enqueue(local)) {
        bump(s_frames);
        if (local.onset) bump(s_onsets);
        if (local.boom) bump(s_booms);
    }
#else

    (void)f;
#endif
}

int visualiser_hop(void)
{
    /* Compiled in, not configured at runtime: it is the grid the windows are
     * cut on, and a unit cannot change it without re-deriving everything. */
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
    /* A counter, not a flag, so two flushes close together cannot collapse
     * into one. The render task does the work; this is callable from any
     * task. */
    s_fq_flush.fetch_add(1, std::memory_order_release);
}

void visualiser_feed(const uint8_t *pcm, uint32_t len, int64_t due_master_us)
{
    /* Silently ignored before visualiser_start(), and on a unit that takes
     * frames rather than audio: a caller should not have to know which. */
    if (!pcm_stream) {
        return;
    }

    const int64_t rate = s_rate.load(std::memory_order_relaxed);

    /*
     * THE DRIFT CHECK. Extrapolate from the reference pair and compare against
     * the instant this chunk was handed: if the count and the timeline have
     * come apart, audio was skipped or inserted without anyone saying so, and
     * the origin has to be re-derived. See ALIGN_DRIFT_US.
     *
     * Skipped while an alignment is already in flight, where the two are
     * legitimately out of step until it completes.
     */
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

    /*
     * Derive the origin: turn the instant into a sample position, round it UP
     * to the next whole hop, and arrange to drop the part-hop in front of it.
     *
     * Rounding to the hop grid is what makes two units agree. The position
     * comes from the shared instant, so every unit computes the same one, and
     * every unit therefore starts its windows at the same sample.
     */
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

    /* The next byte fed IS the origin. Published index and position first,
     * generation last with release ordering -- the analysis task reads the
     * generation with acquire and so never sees a half-published origin. */
    if (s_mark_align_point) {
        s_align_block_index.store(s_pending_block_index, std::memory_order_relaxed);
        s_align_at_byte.store(s_sent_total, std::memory_order_relaxed);
        s_align_gen.fetch_add(1, std::memory_order_release);

        s_ref_due = s_pending_block_index * HOP_N * 1000000LL / rate;
        s_ref_byte = s_sent_total;
        s_ref_valid = true;
        s_mark_align_point = false;
    }

    const size_t sent = xStreamBufferSend(pcm_stream, pcm, len, 0);
    s_sent_total += sent;
    /* A refused byte breaks the count, so the origin has to be re-derived --
     * the alternative is every frame after it being mislabelled by the amount
     * dropped. */
    if (sent < len) {
        bump(s_dropped, len - sent);
        s_align_pending.store(true, std::memory_order_relaxed);
    }
}

void visualiser_realign(void)
{
    /* Just a flag: the work happens in visualiser_feed(), on the task that
     * feeds, so this is callable from anywhere. */
    s_align_pending.store(true, std::memory_order_relaxed);
}

void visualiser_set_rate(uint32_t hz)
{
    /* Refused rather than clamped, and said loudly: a figure outside this
     * range is not a sample rate, and adopting the nearest plausible one would
     * put the count and the timeline permanently out of step at whatever the
     * difference turned out to be. */
    if (hz < 8000 || hz > 192000) {
        ESP_LOGE(TAG, "ignoring a stream rate of %" PRIu32 " Hz -- not a sample rate", hz);
        return;
    }

    /* Cheap when nothing changed, which matters because a caller may report
     * the rate on every packet. */
    if (s_rate.exchange(hz, std::memory_order_relaxed) == hz) {
        return;
    }

    /* A new rate re-cuts the bands and changes the instant-to-position
     * conversion, so the origin is derived again. The analysis task notices
     * the rate itself and re-inits everything downstream of it. */
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

    /*
     * The analysis stream, in PSRAM where there is any.
     *
     * It is large, it is touched by one task at audio cadence, and it competes
     * for exactly the pool the WiFi driver's transmit buffers come out of. The
     * before-and-after figures are logged because that internal headroom is
     * what bounds how many transmit buffers this board can afford -- so the
     * line is the measurement, not decoration.
     */
    const size_t largest_before =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    const char *where = "internal";
#if CONFIG_SPIRAM

    pcm_stream = xStreamBufferCreateWithCaps(STREAM_BYTES, HOP_BYTES,
                                             MALLOC_CAP_SPIRAM);
    if (pcm_stream) {
        where = "PSRAM";
    } else {
        /* Falling back is correct but not free, and it is worth saying so: the
         * buffer has just taken internal RAM that was budgeted elsewhere. */
        ESP_LOGW(TAG, "analysis stream: PSRAM refused %d bytes, falling back to "
                      "internal -- ESP_WIFI_STATIC_TX_BUFFER_NUM may now be too "
                      "high for this board", STREAM_BYTES);
        pcm_stream = xStreamBufferCreate(STREAM_BYTES, HOP_BYTES);
    }
#else
    pcm_stream = xStreamBufferCreate(STREAM_BYTES, HOP_BYTES);
#endif

    if (pcm_stream) {
        ESP_LOGI(TAG, "analysis stream: %d bytes in %s, largest internal block "
                      "%u -> %u -- the internal figure is the headroom for "
                      "ESP_WIFI_STATIC_TX_BUFFER_NUM, ~1.6 kB a buffer",
                 STREAM_BYTES, where, (unsigned)largest_before,
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    } else {
        ESP_LOGE(TAG, "analysis stream: %d bytes REFUSED -- largest internal "
                      "block was only %u. This unit will not draw. Lower "
                      "ESP_WIFI_STATIC_TX_BUFFER_NUM, or move this allocation "
                      "ahead of esp_wifi_init().",
                 STREAM_BYTES, (unsigned)largest_before);
    }
#endif

#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER
    {
        gpio_config_t m = {};
        m.pin_bit_mask = 1ULL << CONFIG_DANCEFLOOR_LED_MARKER_GPIO;
        m.mode = GPIO_MODE_OUTPUT;
        ESP_ERROR_CHECK(gpio_config(&m));

        gpio_set_level(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_MARKER_GPIO),
                       LED_MARKER_OFF);
        ESP_LOGW(TAG, "LED marker on GPIO %d, active %s -- dark until joined, "
                      "solid when joined and idle, one flash per master-clock "
                      "second while playing; nothing corrects on it",
                 CONFIG_DANCEFLOOR_LED_MARKER_GPIO,
                 LED_MARKER_ON == 0 ? "low" : "high");
    }
#endif

#if DF_ANALYSES_AUDIO

    analysis.init(static_cast<int>(s_rate.load(std::memory_order_relaxed)));
#endif

    /*
     * Every slot starts LATCHED, so one with no analyser behind it reports
     * nothing rather than whatever the array held. A slot is unlatched only
     * when this unit computes it itself, inline, in the fast lane -- see
     * df::ResultLatch::set_latched().
     */
    {
        const int rate = static_cast<int>(s_rate.load(std::memory_order_relaxed));

        /* Frames a second, which is the only rate an analyser can still
         * meaningfully want -- it is handed frames, not PCM. */
        const int frames_per_s = rate / HOP_N;
        for (int i = 0; i < df::ML_SLOTS; i++) {
            s_latch.set_latched(i, true);
            df::Analyser *a = df::analyser_at(i);
            if (!a) {
                continue;
            }
            const df::AnalyserSpec &sp = a->spec();
#if DF_RUNS_ANALYSERS
            if (sp.lane == df::Lane::Fast) {
                if (a->init(frames_per_s)) {
                    s_latch.set_latched(i, false);
                } else {
                    ESP_LOGE(TAG, "analyser \"%s\" refused to start -- slot %d idle",
                             sp.name, i);
                    continue;
                }
            }
#endif
            ESP_LOGI(TAG, "analyser %d: \"%s\" model %u | %s lane | %d bins @ "
                          "%d frames/s | shown %lld us late | %s",
                     i, sp.name, (unsigned)sp.model_id,
                     sp.lane == df::Lane::Fast ? "fast" : "slow",
                     df::SPEC_BINS, frames_per_s,
                     (long long)sp.present_delay_us,
                     DF_RUNS_ANALYSERS
                         ? (sp.lane == df::Lane::Fast ? "computed here, in the frame"
                                                      : "computed here, through the latch")
                         : "given to this unit");
        }

#if DF_RUNS_ANALYSERS

        /* After the fast lane, because the lane names any second slow analyser
         * it finds and that message reads better beside the slot lines
         * above. */
        df::ml_lane_start(&s_latch, frames_per_s);
#endif
    }

    pattern = df::pattern_at(0);
    if (df::Pattern *p = df::pattern_by_name(CONFIG_DANCEFLOOR_LED_PATTERN)) {
        pattern = p;
    } else {
        ESP_LOGW(TAG, "no pattern named \"%s\" -- falling back to \"%s\"",
                 CONFIG_DANCEFLOOR_LED_PATTERN, pattern ? pattern->name() : "none");
    }

    /* SPI rather than RMT: RMT's per-frame enable/disable races its own
     * transmit-done interrupt under a continuous refresh and eventually wedges
     * the channel. See LedStrip::Backend. */
    strip.emplace(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_GPIO), LED_COUNT,
                  STRIP_TYPE, LedStrip::Backend::SPI);
    strip->clear();
    strip->show();

#if DF_ANALYSES_AUDIO

    /* Both task creations are checked and both messages say what is lost,
     * because either failure leaves a unit that boots normally and simply
     * never lights up. */
    if (!pcm_stream) {
        ESP_LOGE(TAG, "TASK \"vis\" NOT STARTED -- no analysis stream to read");
    } else if (xTaskCreatePinnedToCore(visualiser_task, "vis", 4096, nullptr, 4, nullptr, 1) != pdPASS) {
        ESP_LOGE(TAG, "TASK \"vis\" FAILED TO START -- no analysis, the strip "
                      "will not react to audio");
    }
#endif
    if (xTaskCreatePinnedToCore(render_task, "vis-draw", 3072, nullptr, 5, nullptr, 1) != pdPASS) {
        ESP_LOGE(TAG, "TASK \"vis-draw\" FAILED TO START -- the strip will stay dark");
    }

    ESP_LOGW(TAG, "started: %lu LEDs on GPIO %d at %d%% brightness, pattern %s, "
                  "frames %s | window %d, hop %d (%.1f frames/s at %" PRIu32 " Hz)",
             LED_COUNT, CONFIG_DANCEFLOOR_LED_GPIO, CONFIG_DANCEFLOOR_LED_BRIGHTNESS,
             pattern ? pattern->name() : "none", visualiser_source_name(),
             FFT_N, HOP_N,
             (double)s_rate.load(std::memory_order_relaxed) / HOP_N,
             s_rate.load(std::memory_order_relaxed));
}
