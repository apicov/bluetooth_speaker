/**
 * @file analyser.hpp
 * @brief The pluggable processing stage: audio in, a Result out, displayed in
 *        step with the audio it describes.
 *
 * This sits BESIDE df::Analysis rather than replacing it. Analysis is the FFT,
 * the bands and the two onset detectors; it runs on every frame, and its
 * output is df::Frame -- the thing patterns are written against and the thing
 * that travels. None of that changes. What this adds is a way to run OTHER
 * algorithms over the same audio, on their own window and their own hop, and
 * have their answers reach the strip at the right moment.
 *
 * Like everything else in this directory it touches no clock, no task and no
 * strip. The firmware half owns those.
 *
 * THE ONE RULE FOR ANALYSERS, the same as for a Pattern one level up: an
 * Analyser must be a pure function of the audio it is handed and the constants
 * it was compiled with. Two units running the same analyser over the same
 * audio must produce the same Result, BIT FOR BIT, or their strips separate.
 * That is not a soft target -- the hub and a satellite are different cores, so
 * the same source compiles to different instruction sequences and, where
 * floating point or a vendor kernel is involved, potentially to different
 * rounding. Integer and fixed-point arithmetic is identical on both; float is
 * identical only as long as nothing selects a different kernel per target,
 * which the vendor's own libraries do. So quantised integer models are a
 * CORRECTNESS requirement here, not a size optimisation.
 */
#pragma once

#include <stdint.h>

#include "analysis_config.h"

namespace df {

/** @brief The portable spectrum's width, from analysis_config.h so that this
 *         header and analysis.hpp cannot disagree about it. */
constexpr int SPEC_BINS = DF_SPEC_BINS;

/** @brief How many analysers may run at once. One slot per registered
 *         analyser, at the same index, so f.ml[i] is always analyser i. Set in
 *         analysis_config.h, which explains what a slot costs. */
constexpr int ML_SLOTS = DF_ML_SLOTS;

/**
 * @brief How many scores a Result carries.
 *
 * A handful, because the thing on the other end is an LED strip. A classifier
 * with hundreds of classes still only has a few worth showing, and the top-k
 * is what a pattern can express -- so the reduction happens in the ANALYSER,
 * where the model's own labels are known, rather than shipping everything and
 * reducing at the far end. Same reasoning as the portable spectrum being far
 * narrower than the FFT's output.
 */
constexpr int RESULT_SCORES = 8;

/** @brief Result::analyser when nothing has been produced yet. Patterns must
 *         handle it: at startup, after a flush, and on any unit whose analyser
 *         has not filled its context, this is what they will see. */
constexpr uint8_t RESULT_NONE = 0xFF;

/**
 * @brief One analyser's answer about one window of audio.
 *
 * Deliberately small and deliberately PLAIN DATA. It is copied into every
 * df::Frame that carries it and latched across tasks -- two places where a
 * pointer would need a lifetime rule, which df::Frame::mag already
 * demonstrates the cost of.
 */
struct Result {
    /** @brief Window number on this analyser's OWN grid, from an origin all
     *         units share. The analogue of df::Frame::index, and not the same
     *         number: a slow analyser's grid is coarser. */
    int64_t index;

    /**
     * @brief The master-clock instant this result is DISPLAYED.
     *
     * Not the instant the window started, and not the instant the inference
     * finished. It is the window's own instant plus
     * AnalyserSpec::present_delay_us -- see that field, which is where the
     * whole design of this lives.
     */
    int64_t show_at_us;

    uint8_t analyser;    /**< Index into the registry; RESULT_NONE if unfilled. */
    uint8_t model_id;    /**< See AnalyserSpec::model_id. */
    uint8_t n;           /**< How many entries of #label and #score are valid. */
    uint8_t label[RESULT_SCORES];   /**< Class ids; see Analyser::label_name(). */
    uint8_t score[RESULT_SCORES];   /**< Quantised 0..255, highest first. */
};

/** @brief Whether a Result says anything. A pattern should check this rather
 *         than reading score[0] and getting whatever was left in the struct.
 *  @param r  The result.
 *  @return Whether it was filled. */
constexpr bool result_valid(const Result &r) { return r.analyser != RESULT_NONE; }

/**
 * @brief A Result that says nothing.
 *
 * constexpr so a latch holding these can be constant-initialised rather than
 * needing a constructor to run before the firmware starts -- it is a
 * namespace-scope object there, and the fewer of those with dynamic
 * initialisation the fewer questions there are about what has run by the time
 * a task starts.
 *
 * @return The empty result.
 */
constexpr Result result_none()
{
    Result r{};
    r.analyser = RESULT_NONE;
    return r;
}

/**
 * @brief Which lane an analyser runs in.
 *
 * Fast runs INLINE on the analysis task, on the same window df::Analysis just
 * transformed, once per hop. It costs nothing to schedule and has no delay --
 * but it shares a task with the FFT, and that task has only one frame period
 * per frame and has been measured using most of it already. Anything not
 * comfortably free belongs in the other lane.
 *
 * Slow runs on its OWN task, off the core the audio and the strip share, at a
 * priority below both. It may take as long as its declared presentation delay
 * allows.
 */
enum class Lane { Fast, Slow };

/**
 * @brief What an analyser needs from the firmware, and what it promises in
 *        return.
 *
 * Every field is a compile-time constant of the build. Nothing here may vary
 * per unit, per track or per run -- these numbers are half of what two units
 * use to agree with each other, the audio being the other half.
 *
 * Note what is NOT here: a sample rate, a window length and a hop. An analyser
 * is handed FRAMES, not PCM -- the quantised spectrum of one analysis frame,
 * on the grid the FFT already cut -- and one wanting more context accumulates
 * frames itself. What the grid is, is therefore not this struct's business.
 */
struct AnalyserSpec {
    /** @brief For Kconfig selection, logs and the health line. */
    const char *name;

    /**
     * @brief Which algorithm AND which trained weights this is.
     *
     * It travels in every Result. A unit given a result whose model_id differs
     * from its own says so once, because that is the difference that makes two
     * strips disagree while every other diagnostic looks healthy.
     *
     * Bump it when the WEIGHTS change, not only when the algorithm does. A
     * retrained model with the same architecture is a different model to
     * anything downstream of it.
     */
    uint8_t model_id;

    /**
     * @brief How long after a window's audio is heard its Result is shown.
     *
     * THIS IS THE FIELD THAT MAKES SLOW ALGORITHMS POSSIBLE AT ALL, and the
     * one that is easy to get wrong in a way nothing catches for hours.
     *
     * A frame's due_us names the instant its window's FIRST sample is heard.
     * That works for the FFT because its window is milliseconds long and the
     * audio arrives a playback lead before it is due, so the answer exists
     * long before the moment it describes.
     *
     * It does not work for a model with a one-second context. The window's
     * last sample does not exist until a second after its first, so the answer
     * cannot exist until well AFTER the instant it would have been shown at.
     * No amount of CPU changes that; it is arithmetic about when audio
     * arrives.
     *
     * So a result is shown late on purpose, by exactly this much, and the
     * amount is DECLARED rather than measured. It must satisfy
     *
     *     present_delay_us >= context_frames * 1e6 / frames_per_s
     *                         + worst_case_compute_us
     *                         + publish_latency_us
     *                         - the playback lead
     *
     * and it must be the same number on every unit running this analyser.
     *
     * The WHOLE context, not the context minus one frame: nothing can be
     * computed until the last frame of it has arrived. Unless the analyser
     * labels by its LAST frame, in which case the context is already behind
     * and the bound is slack.
     *
     * USING THE MEASURED INFERENCE TIME INSTEAD IS THE TRAP. It differs
     * between the two cores, it differs between a busy board and an idle one,
     * and it would date each unit's results by how that unit's afternoon was
     * going -- the same failure as a Pattern reading a wall clock, and it
     * fails the same way: strips that agree on the bench and separate over an
     * evening.
     *
     * Do not guess it either. The slow lane logs the margin between when a
     * result was actually ready and when it was due to be shown; set this from
     * that measurement and record the number beside the value.
     *
     * Zero is correct for a Fast analyser and for nothing else.
     */
    int64_t present_delay_us;

    Lane lane;   /**< Which lane runs it. */
};

/**
 * @brief Implement this to plug in a new algorithm.
 *
 * The lifecycle is init() once when the stream rate is known, process() per
 * window, reset() when the timeline restarts. All three are called from ONE
 * task -- whichever lane the spec asks for -- so an implementation needs no
 * locking and may keep whatever state it likes in members, as long as that
 * state is a function of the audio it has been given.
 */
class Analyser {
public:
    virtual ~Analyser() = default;

    /** @brief This analyser's constants. @return The spec. */
    virtual const AnalyserSpec &spec() const = 0;

    /**
     * @brief Prepare for a stream.
     *
     * @param frames_per_s  How many analysis frames arrive each second -- the
     *                      stream rate divided by the hop. An analyser sizing
     *                      a context in TIME converts through it; one counting
     *                      frames can ignore it. It is a runtime value even
     *                      though the hop is compiled in, because the bridge
     *                      advertises several rates and takes what the phone
     *                      gives, so an analyser that hard-codes one is wrong
     *                      on the others.
     * @return false if this analyser cannot run on this build -- no arena, no
     *         model linked in, a rate it cannot serve. The lane then skips it
     *         and SAYS SO once, which is far easier to diagnose than a silent
     *         absence.
     */
    virtual bool init(int frames_per_s) = 0;

    /** @brief Drop accumulated state. Called on a timeline restart, a rate
     *         change, and a realignment -- anything after which previous audio
     *         no longer precedes the next audio. */
    virtual void reset() {}

    /**
     * @brief What a label id means, for logs and the collector.
     *
     * Here rather than in a table beside the registry because the names belong
     * to the MODEL, and the model and its labels change together -- a
     * retrained classifier with reordered classes is exactly the case where a
     * separate table silently goes stale.
     *
     * @param label  The class id.
     * @return Its name, or null if this analyser has no names for its classes.
     */
    virtual const char *label_name(uint8_t label) const { (void)label; return nullptr; }

    /**
     * @brief Process one analysis frame.
     *
     * @param spec      One frame's quantised spectrum: SPEC_BINS log-spaced
     *                  bins, 0 for silence and 255 for the top of the
     *                  normalised range. IT DOES NOT MATTER WHERE IT CAME
     *                  FROM -- the identical bytes are produced by a unit
     *                  analysing its own audio and unpacked from a frame off
     *                  the radio, which is what lets a unit with no audio at
     *                  all run models. The compression is FIXED rather than an
     *                  AGC against a running maximum, so absolute level
     *                  survives it, monotonically; a feature wanting loudness
     *                  may take it from these bins.
     * @param index     The frame's place on the shared grid.
     * @param due_us    The master-clock instant this frame's window is heard,
     *                  derived by counting from an origin every unit shares.
     *                  NEVER read a clock in here.
     * @param[out] out  The result. An implementation sets index, n, label[]
     *                  and score[]; the LANE fills show_at_us, analyser and
     *                  model_id from the spec, deliberately -- an analyser
     *                  that could set them could date its own results, which
     *                  is the one thing the presentation-delay rule forbids.
     * @return true if @p out was filled. Returning false is NORMAL: a model
     *         that accumulates context returns false on every call until its
     *         context is complete, which is how an analyser with a long
     *         context reports slowly without the lane needing to know that.
     */
    virtual bool process(const uint8_t (&spec)[SPEC_BINS], int64_t index,
                         int64_t due_us, Result *out) = 0;
};

/** @brief How many analysers are registered. A static table, no dynamic
 *         registration: adding one means adding a class and an entry.
 *  @return The count. */
int       analyser_count();
/** @brief The analyser at an index. @param i Index. @return It, or null. */
Analyser *analyser_at(int i);
/** @brief Look one up by AnalyserSpec::name. @param name The name.
 *  @return It, or null. */
Analyser *analyser_by_name(const char *name);

/**
 * @brief Drive every fast-lane analyser over one window, and fill in the
 *        slots.
 *
 * Here rather than in the firmware because tools/pattern_lab must run the same
 * code the boards do -- that is the whole claim the harness makes, and an
 * analyser lane reimplemented on the laptop would quietly break it the first
 * time one of the two was changed.
 *
 * Called from one task only.
 *
 * @param spec      The frame's quantised spectrum, whether this unit computed
 *                  it or took it off the radio.
 * @param index     The frame's place on the shared grid.
 * @param due_us    The instant its window is heard.
 * @param skip      True for a slot this unit does not compute itself -- a slow
 *                  analyser, an absent one, or any slot at all on a unit that
 *                  is given its results. Those are filled from the latch
 *                  instead, and are set to result_none() here because the
 *                  caller's buffer is reused and whatever a previous frame
 *                  left in it describes audio long past.
 * @param[out] out  One result per slot.
 */
void run_fast_lane(const uint8_t (&spec)[SPEC_BINS], int64_t index,
                   int64_t due_us, const bool skip[ML_SLOTS], Result out[ML_SLOTS]);

/**
 * @brief Interleaved stereo to mono.
 *
 * ONE definition, used by both lanes and by the host harness, and it must stay
 * the same downmix df::Analysis::process() does internally -- two front ends
 * disagreeing about what mono means would be a difference nothing downstream
 * could see.
 *
 * @param stereo    Interleaved input.
 * @param n         Frames.
 * @param[out] mono @p n mono samples.
 */
void downmix(const int16_t *stereo, int n, int16_t *mono);

}  // namespace df
