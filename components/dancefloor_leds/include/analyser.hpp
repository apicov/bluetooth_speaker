/*
 * The pluggable processing stage: audio in, a Result out, displayed in step
 * with the audio it describes.
 *
 * This sits beside df::Analysis rather than replacing it. Analysis is the FFT,
 * the four bands and the two onset detectors, it runs on every frame, and its
 * output is df::Frame -- the thing patterns are written against and the thing
 * that travels as vis_frame_t. None of that changes. What this adds is a way to
 * run OTHER algorithms over the same audio, on their own window, their own hop
 * and their own sample rate, and have their answers reach the strip at the
 * right moment.
 *
 * Like everything else in this directory it touches no clock, no task and no
 * strip. The firmware half owns those.
 *
 * ---------------------------------------------------------------------------
 * The one rule for analysers
 * ---------------------------------------------------------------------------
 *
 * Same rule as Pattern, one level up: an Analyser must be a pure function of
 * the audio it is handed and the constants it was compiled with.
 *
 * Two units running the same analyser over the same audio must produce the same
 * Result, bit for bit, or their strips separate. That is not a soft target. The
 * hub is an ESP32-S3 (LX7) and a satellite is a classic ESP32 (LX6), so the same
 * source compiles to different instruction sequences and, where floating point
 * or a vendor kernel is involved, potentially to different rounding. Integer and
 * fixed-point arithmetic is identical on both; float is identical only as long
 * as nothing selects a different kernel per target, which esp-nn and esp-dsp
 * both do.
 *
 * So: quantised integer models are a CORRECTNESS requirement here, not a size
 * optimisation. See docs/architecture.md and the note in Kconfig under
 * DANCEFLOOR_ML_SOURCE for what a mixed floor is and is not promised.
 */
#pragma once

#include <stdint.h>

#include "analysis_config.h"

namespace df {

/*
 * How many analysers may run at once. One slot per registered analyser, at the
 * same index, so f.ml[i] is always analyser i. Set in analysis_config.h, which
 * explains what a slot costs.
 */
constexpr int ML_SLOTS = DF_ML_SLOTS;

/*
 * How many scores a Result carries.
 *
 * Eight, because the thing on the other end is an LED strip. A classifier with
 * 500 classes still only has a handful worth showing, and the top-k is what a
 * pattern can express -- so the reduction happens in the analyser, where the
 * model's own labels are known, rather than shipping everything and reducing at
 * the far end. Same reasoning as Frame::spec being 64 bins instead of 512.
 */
constexpr int RESULT_SCORES = 8;

/* `Result::analyser` when nothing has been produced yet. Patterns must handle
 * it: at startup, after a flush, and on any unit whose analyser has not filled
 * its context, this is what they will see. */
constexpr uint8_t RESULT_NONE = 0xFF;

/*
 * One analyser's answer about one window of audio.
 *
 * Deliberately small and deliberately plain data. It is copied into every
 * df::Frame that carries it, it goes on the wire as ml_result_t, and it is
 * latched across tasks -- three places where a pointer would need a lifetime
 * rule, which Frame::mag already demonstrates the cost of.
 */
struct Result {
    /* Window number on this analyser's own grid, from an origin all units
     * share. The analogue of Frame::index, and not the same number: a slow
     * analyser's grid is coarser. */
    int64_t index;

    /*
     * The master-clock instant this result is DISPLAYED.
     *
     * Not the instant the window started, and not the instant the inference
     * finished. It is `window_due_us + spec().present_delay_us` -- see the long
     * note on that field, which is where the whole design of this lives.
     */
    int64_t show_at_us;

    uint8_t analyser;    /* index into the registry; RESULT_NONE if unfilled */
    uint8_t model_id;    /* see AnalyserSpec::model_id */
    uint8_t n;           /* how many entries of label[]/score[] are valid */
    uint8_t label[RESULT_SCORES];
    uint8_t score[RESULT_SCORES];   /* quantised 0..255, highest first */
};

/* Whether this Result says anything. A pattern should check it rather than
 * reading score[0] and getting whatever was left in the struct. */
constexpr bool result_valid(const Result &r) { return r.analyser != RESULT_NONE; }

/* A Result that says nothing.
 *
 * constexpr so that a latch holding these can be constant-initialised rather
 * than needing a constructor to run before app_main -- it is a namespace-scope
 * object in the firmware, and the fewer of those with dynamic initialisation
 * the fewer questions there are about what has run by the time a task starts. */
constexpr Result result_none()
{
    Result r{};
    r.analyser = RESULT_NONE;
    return r;
}

/*
 * Which lane an analyser runs in.
 *
 * Fast runs inline on the analysis task, on the same window df::Analysis just
 * transformed, once per hop. It costs nothing to schedule and has no delay --
 * but it shares a task with the FFT, and at hop 512 that task has 11.6 ms per
 * frame and has been measured taking up to 21 ms of it already. Anything that
 * is not comfortably free belongs in the other lane.
 *
 * Slow runs on its own task, off the core the audio and the strip share, at a
 * priority below both. It may take as long as its declared presentation delay
 * allows, and it is the only lane that may ask for a different sample rate or a
 * window longer than the FFT's.
 */
enum class Lane { Fast, Slow };

/*
 * What an analyser needs from the firmware, and what it promises in return.
 *
 * Every field is a compile-time constant of the build. Nothing here may vary
 * per unit, per track or per run -- these numbers are half of what two units
 * use to agree with each other, the audio being the other half.
 */
struct AnalyserSpec {
    /* For Kconfig selection, logs and the HEALTH line. */
    const char *name;

    /*
     * Which algorithm AND which trained weights this is.
     *
     * It travels in every Result. A unit given a result whose model_id differs
     * from its own says so once, because that is the difference that makes two
     * strips disagree while every other diagnostic looks healthy -- the same
     * class of fault as the analysis-hop mismatch, and detected the same way.
     *
     * Bump it when the weights change, not only when the algorithm does. A
     * retrained model with the same architecture is a different model to
     * anything downstream of it.
     */
    uint8_t model_id;

    /*
     * The rate the audio must be at when it reaches process(), or 0 for
     * "whatever the stream is".
     *
     * 0 is the only value a Fast analyser may use: it is handed the window
     * df::Analysis just used, at the stream rate, and resampling that would
     * mean doing it twice per frame for no gain.
     *
     * A Slow analyser normally wants 16000 -- almost every published audio
     * model does -- and gets there through the fixed-point resampler in
     * resample.h. The stream is 44.1 kHz stereo by default but the bridge
     * advertises 16, 32, 44.1 and 48 kHz and takes what the phone gives it, so
     * the resampler's ratio is a runtime value even though this field is not.
     */
    int rate_hz;

    /* Samples handed to process(), at rate_hz. Always MONO. */
    int window_n;

    /* Samples advanced between calls to process(), at rate_hz. Equal to
     * window_n for non-overlapping windows. */
    int hop_n;

    /*
     * How long after a window's audio is heard its Result is shown.
     *
     * ---------------------------------------------------------------------
     * This is the field that makes slow algorithms possible at all, and it is
     * the one that is easy to get wrong in a way nothing catches for hours.
     * ---------------------------------------------------------------------
     *
     * A frame's due_us names the instant its window's FIRST sample is heard.
     * That works for the FFT because its window is 1024 samples -- 23 ms -- and
     * the audio arrives about 200 ms before it is due (LEAD_US in the hub's
     * streamer), so the answer exists long before the moment it describes.
     *
     * It does not work for a model with a one-second context. The window's last
     * sample does not exist until a second after its first, so the answer
     * cannot exist until ~800 ms AFTER the instant it would have been shown at.
     * No amount of CPU changes that; it is arithmetic about when audio arrives.
     *
     * So a result is shown late on purpose, by exactly this much, and the
     * amount is DECLARED rather than measured. It must satisfy
     *
     *     present_delay_us >= window_n * 1e6 / rate_hz      (the window must arrive)
     *                         + worst_case_compute_us
     *                         + publish_latency_us
     *                         - LEAD_US                     (the audio arrives early)
     *
     * and it must be the same number on every unit running this analyser.
     *
     * The whole window, not the window minus the hop: nothing can be computed
     * until the window's LAST sample has arrived, and that sample is heard
     * `window_n / rate_hz` after the instant the window is labelled with.
     *
     * Worked, for the FFT: 1024 samples at 44.1 kHz is 23 ms against a 200 ms
     * lead, so the bound is negative and zero is correct -- which is why the
     * fast lane needs no delay and why this whole field could be ignored until
     * something with a real context window arrived.
     *
     * Worked, for a one-second model at 16 kHz: 1000 - 200 = 800 ms before
     * compute is counted at all.
     *
     * Using the MEASURED inference time instead is the trap. It is different on
     * an LX6 and an LX7, it is different on a busy board and an idle one, and
     * it would date each unit's results by how that unit's afternoon was going
     * -- which is the same failure as a Pattern reading a wall clock, described
     * at the top of analysis.hpp. It fails the way that one does too: strips
     * that agree on the bench and separate over an evening.
     *
     * Do not guess it either. The slow lane logs the margin between when a
     * result was actually ready and when it was due to be shown; set this from
     * that measurement and record the number in a comment beside the value.
     *
     * Zero is correct for a Fast analyser and for nothing else.
     */
    int64_t present_delay_us;

    Lane lane;
};

/*
 * Implement this to plug in a new algorithm.
 *
 * The lifecycle is: init() once when the stream rate is known, process() per
 * window, reset() when the timeline restarts. All three are called from one
 * task -- whichever lane the spec asks for -- so an implementation needs no
 * locking and may keep whatever state it likes in members, as long as that
 * state is a function of the audio it has been given.
 */
class Analyser {
public:
    virtual ~Analyser() = default;

    virtual const AnalyserSpec &spec() const = 0;

    /*
     * `stream_rate_hz` is the rate of the audio arriving, which is NOT
     * necessarily spec().rate_hz -- the lane resamples between them. It is
     * passed because an analyser may need to know what it was derived from.
     *
     * Return false if this analyser cannot run on this build -- no arena, no
     * model linked in, a rate it cannot serve. The lane then skips it and says
     * so once, which is a great deal easier to diagnose than a silent absence.
     */
    virtual bool init(int stream_rate_hz) = 0;

    /* Drop accumulated state. Called on a timeline restart, a rate change, and
     * a realignment -- anything after which previous audio no longer precedes
     * the next audio. */
    virtual void reset() {}

    /*
     * What a label id means, for logs and the collector. Null if this analyser
     * has no names for its classes.
     *
     * Here rather than in a table beside the registry because the names belong
     * to the model, and the model and its labels change together -- a retrained
     * classifier with reordered classes is exactly the case where a separate
     * table silently goes stale.
     */
    virtual const char *label_name(uint8_t label) const { (void)label; return nullptr; }

    /*
     * `in` is exactly spec().window_n mono int16 samples at spec().rate_hz.
     *
     * `index` and `due_us` are the lane's statement of where this window sits
     * on the shared grid: due_us names the instant the window's FIRST sample is
     * heard, in master-clock microseconds, derived by counting from an origin
     * every unit shares. Never read a clock in here -- see the rule at the top.
     *
     * Return true if *out was filled. Returning false is normal and expected:
     * a model that accumulates context returns false on every call until its
     * context is complete, which is how an analyser with a 1 s context and a
     * 10 ms hop reports at 1 Hz without the lane needing to know that.
     *
     * The lane fills out->show_at_us, out->analyser and out->model_id from the
     * spec. An implementation sets index, n, label[] and score[].
     */
    virtual bool process(const int16_t *in, int64_t index, int64_t due_us,
                         Result *out) = 0;
};

/*
 * The registry, in the shape patterns.cpp already uses: a static table, no
 * dynamic registration. Adding an analyser means adding a class and an entry.
 */
int       analyser_count();
Analyser *analyser_at(int i);
Analyser *analyser_by_name(const char *name);

/*
 * Drive every fast-lane analyser over one window, and fill in the slots.
 *
 * Here rather than in the firmware because tools/pattern_lab must run the same
 * code the boards do -- that is the whole claim the harness makes, and an
 * analyser lane reimplemented on the laptop would quietly break it the first
 * time one of the two was changed.
 *
 * `mono` is `window_n` samples -- the same window the FFT was handed, downmixed
 * by df::downmix(). The caller does the downmix because the slow lane needs the
 * same buffer and doing it twice per frame would be pure cost.
 *
 * `skip[i]` is true for a slot this unit does not compute itself -- a slow
 * analyser, an absent one, or any slot at all on a unit that is given its
 * results. Those are filled from the latch instead and are set to result_none()
 * here, because the caller's buffer is reused and whatever a previous frame
 * left in it describes audio long past.
 *
 * show_at_us, analyser and model_id are filled in from the spec rather than by
 * the analyser, deliberately: an analyser that could set them could date its
 * own results, which is the one thing the presentation-delay rule forbids.
 *
 * Called from one task only -- the analysis task in the firmware.
 */
void run_fast_lane(const int16_t *mono, int window_n, int64_t index,
                   int64_t due_us, const bool skip[ML_SLOTS], Result out[ML_SLOTS]);

/*
 * Interleaved stereo to mono, for `n` frames.
 *
 * One definition, used by both lanes and by tools/pattern_lab, and it must stay
 * the same downmix Analysis::process() does internally -- two front ends
 * disagreeing about what mono means would be a difference nothing downstream
 * could see.
 */
void downmix(const int16_t *stereo, int n, int16_t *mono);

}  // namespace df
