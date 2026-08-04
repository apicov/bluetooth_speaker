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
#include "led_strip_wrapper.hpp"

namespace {

constexpr const char *TAG = "vis";

using df::FFT_N;
using df::RATE;
using df::CHANNELS;

/* Four analysis frames of headroom. A short send loses audio AND breaks the
 * block alignment, so the buffer is sized to make that rare. */
constexpr int STREAM_BYTES = FFT_N * CHANNELS * (int)sizeof(int16_t) * 4;
constexpr uint32_t FRAME_BYTES = CHANNELS * sizeof(int16_t);

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
 * pin on one block and lowering it two later costs nothing.
 */
constexpr int LED_MARKER_BLOCKS_HIGH = 2;
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
df::Analysis analysis;

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
 * The detector chops the stream into fixed FFT_N blocks, and every unit must cut
 * them at the same positions or a transient near a boundary is split on one
 * board and centred on another -- the marginal onsets then fire on one strip and
 * not the other. Boundaries are derived from the instant audio is SCHEDULED to
 * be heard, which all units agree on, and never from a clock read here, which
 * they do not: they reach this code milliseconds apart.
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
 * discard arithmetic still lands on a multiple of FFT_N, so both units cut
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
 * The cost of firing is one dropped analysis block, ~23 ms of the strip holding
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

uint32_t take(std::atomic<uint32_t> &c) { return c.exchange(0, std::memory_order_relaxed); }
void bump(std::atomic<uint32_t> &c, uint32_t n = 1) { c.fetch_add(n, std::memory_order_relaxed); }

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
 * 32 frames is ~740 ms at the current hop, comfortably more than the 200 ms of
 * lead the audio carries, so the queue is sized by the audio buffer rather than
 * the other way round.
 */
constexpr uint32_t FRAME_RING = 32;
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
 * Master -> local, or null on a unit where they are the same thing.
 *
 * A function rather than a stored offset because the satellite's is slewed
 * continuously; see visualiser_set_clock().
 */
std::atomic<int64_t (*)(int64_t)> s_to_local{nullptr};

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

void visualiser_task(void *arg)
{
    (void)arg;
    static int16_t raw[FFT_N * CHANNELS];
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
        filled = 0;
        starved_shown = false;

        /* due_us is derived from the block index, not read from a clock, so it
         * is the same value on every unit for this same audio -- at the rate
         * this unit was told the audio is, which is the same rate its playback
         * used to date the audio in the first place. */
        const int64_t due_us = block_index * FFT_N * 1000000LL / rate;
        const int64_t t_in = esp_timer_get_time();
        const df::Frame &f = analysis.process(raw, block_index, due_us, 0);
        const int64_t t_analysed = esp_timer_get_time();

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
        {
            const uint32_t head = s_fq_head.load(std::memory_order_relaxed);
            const uint32_t tail = s_fq_tail.load(std::memory_order_acquire);
            if (head - tail >= FRAME_RING) {
                bump(s_overrun);
            } else {
                s_fq[head % FRAME_RING] = f;
                /* Release, against the acquire on the reader: the frame must be
                 * fully written before the index that publishes it moves. */
                s_fq_head.store(head + 1, std::memory_order_release);
            }
        }
        {
            const int64_t a = t_analysed - t_in;
            cost_analysis += a;
            if (a > cost_analysis_max) cost_analysis_max = a;
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
            ESP_LOGI(TAG, "cost: analysis %lld/%lld us (mean/max) | render %" PRIu32
                          "/%" PRIu32 " us | queued %" PRIu32 " | late %" PRIu32
                          " | overrun %" PRIu32 " | n %" PRIu32 "/%" PRIu32
                          " | stack free %" PRIu32,
                     cost_analysis / cost_n, cost_analysis_max,
                     rn ? rsum / rn : 0, take(s_render_max),
                     s_fq_head.load(std::memory_order_relaxed) -
                         s_fq_tail.load(std::memory_order_relaxed),
                     take(s_late), take(s_overrun), cost_n, rn,
                     uxTaskGetStackHighWaterMark(nullptr));
            cost_analysis = cost_analysis_max = 0;
            cost_n = 0;
            last_report_us = now;
        }
    }
}

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
            std::memset(pixels, 0, sizeof(pixels));
            show(pixels);
        }

        const uint32_t tail = s_fq_tail.load(std::memory_order_relaxed);
        const uint32_t head = s_fq_head.load(std::memory_order_acquire);
        if (head == tail) {
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
            vTaskDelay(pdMS_TO_TICKS(nap ? nap : 1));
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
            gpio_set_level(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_MARKER_GPIO), 1);
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
            gpio_set_level(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_MARKER_GPIO), 0);
        }
#endif

        if (pattern) {
            pattern->render(f, pixels, LED_COUNT);
            show(pixels);
        }

        const uint32_t took = static_cast<uint32_t>(esp_timer_get_time() - t0);
        s_render_sum.fetch_add(took, std::memory_order_relaxed);
        bump(s_render_n);
        uint32_t prev = s_render_max.load(std::memory_order_relaxed);
        while (took > prev &&
               !s_render_max.compare_exchange_weak(prev, took, std::memory_order_relaxed)) {
        }
    }
}

}  // namespace

void visualiser_set_clock(int64_t (*master_to_local)(int64_t))
{
    s_to_local.store(master_to_local, std::memory_order_relaxed);
}

void visualiser_flush(void)
{
    /* Release, so everything the caller did to establish the new timeline is
     * visible to the render task before it acts on this. */
    s_fq_flush.fetch_add(1, std::memory_order_release);
}

void visualiser_feed(const uint8_t *pcm, uint32_t len, int64_t due_master_us)
{
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
        const int32_t into_block = static_cast<int32_t>(idx % FFT_N);
        s_skip_frames = into_block ? (FFT_N - into_block) : 0;
        s_pending_block_index = (idx + s_skip_frames) / FFT_N;
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
        s_ref_due = s_pending_block_index * FFT_N * 1000000LL / rate;
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
    /* Trigger level is one full analysis frame: waking the task for a handful
     * of bytes at a time is pure overhead now that partial reads accumulate. */
    pcm_stream = xStreamBufferCreate(STREAM_BYTES, FFT_N * CHANNELS * sizeof(int16_t));
    assert(pcm_stream);

#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER
    {
        gpio_config_t m = {};
        m.pin_bit_mask = 1ULL << CONFIG_DANCEFLOOR_LED_MARKER_GPIO;
        m.mode = GPIO_MODE_OUTPUT;
        ESP_ERROR_CHECK(gpio_config(&m));
        ESP_LOGW(TAG, "LED marker on GPIO %d, one flash per master-clock second "
                      "-- every unit flashes together; nothing corrects on it",
                 CONFIG_DANCEFLOOR_LED_MARKER_GPIO);
    }
#endif

    /* Whatever has been set by now, which is the default unless the stream was
     * already running when this was called. Either way the task re-inits if it
     * changes. */
    analysis.init(static_cast<int>(s_rate.load(std::memory_order_relaxed)));

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
    xTaskCreatePinnedToCore(visualiser_task, "vis", 4096, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(render_task, "vis-draw", 3072, nullptr, 5, nullptr, 1);
    ESP_LOGI(TAG, "started: %lu LEDs on GPIO %d at %d%% brightness, pattern %s",
             LED_COUNT, CONFIG_DANCEFLOOR_LED_GPIO, CONFIG_DANCEFLOOR_LED_BRIGHTNESS,
             pattern ? pattern->name() : "none");
}
