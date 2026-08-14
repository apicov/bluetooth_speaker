/*
 * Dropping a TensorFlow Lite Micro model into the analyser lane.
 *
 * Everything an analyser needs from the firmware is already in analyser.hpp;
 * this only removes the boilerplate that every TFLM analyser would otherwise
 * repeat -- getting an arena from the right heap, checking the schema version,
 * building an interpreter, and failing cleanly enough that a unit which cannot
 * run the model still lights up from its neighbours.
 *
 * ---------------------------------------------------------------------------
 * Adding a model
 * ---------------------------------------------------------------------------
 *
 * 1. Enable the dependency. components/dancefloor_leds/idf_component.yml has
 *    the exact lines, commented out -- they are commented rather than present
 *    because esp-tflite-micro is a large component and no build in this tree
 *    needed it until yours does.
 *
 * 2. Turn on DANCEFLOOR_ML_TFLM, and set DANCEFLOOR_ML_ARENA_KB from what the
 *    interpreter reports (see arena_used() -- ask for too much once, read the
 *    number, then set it).
 *
 * 3. Convert the model to a C array: `xxd -i model.tflite > model_data.cc`, or
 *    the equivalent from the converter. It must be 16-byte aligned; declare it
 *    `alignas(16) const unsigned char ...[]`.
 *
 * 4. Subclass TflmAnalyser, fill in the four hooks, and add it to s_analysers[]
 *    in analysers.cpp. The spec is the same one every analyser declares -- a
 *    name, a model_id, the lane, and above all present_delay_us, whose rule is
 *    on AnalyserSpec and is the thing most likely to be got wrong.
 *
 * 5. process() is handed one frame's SPEC_BINS quantised spectrum, not audio.
 *    A model wanting a spectrogram accumulates frames itself and returns false
 *    until it has enough -- Mood in analysers.cpp is the worked example, and
 *    init() is told how many frames a second arrive so a context can be sized
 *    in time. The bins are already uint8 on a fixed compression curve, which is
 *    most of the way to the int8 input a quantised model wants.
 *
 * ---------------------------------------------------------------------------
 * The model must be INT8, and that is not about size
 * ---------------------------------------------------------------------------
 *
 * A unit running a model locally must produce the same answer as its
 * neighbours, bit for bit, or the strips separate -- the rule at the top of
 * analyser.hpp. The hub is an LX7 and a satellite is an LX6, and esp-nn selects
 * different kernels per target. Integer arithmetic is identical on both;
 * float32 kernels are identical only by coincidence, and nothing checks.
 *
 * So a float model here is not slow, it is WRONG on a mixed floor, and wrong in
 * the way that takes an evening to find: two strips that agree on the bench and
 * drift apart over a set. Use full-integer quantisation.
 *
 * A float model is still fine on a floor where exactly one unit runs it at all
 * -- there is only one copy of the decision then, so there is nothing for it to
 * disagree with. That was previously arranged by building the other units
 * DANCEFLOOR_ML_SOURCE_REMOTE; with that setting gone it means, literally, that
 * only one unit has DANCEFLOOR_ML_TFLM. Say so in a comment if you rely on it,
 * because nothing enforces it.
 */
#pragma once

#include "sdkconfig.h"

#if CONFIG_DANCEFLOOR_ML_TFLM

#include <stddef.h>

#include "analyser.hpp"
#include "ml_arena.h"

namespace tflite {
class MicroOpResolver;
class MicroInterpreter;
class Model;
}
struct TfLiteTensor;

namespace df {

class TflmAnalyser : public Analyser {
public:
    ~TflmAnalyser() override;

    /*
     * Brings up the arena and the interpreter on the first call, then hands
     * over to on_rate().
     *
     * Returns false if the model cannot run here -- no arena, a schema the
     * runtime does not know, an op the resolver was not given. The lane then
     * skips this analyser and says so once, which is a great deal easier to
     * diagnose than a silent absence, and the unit still draws whatever its
     * neighbours send it.
     */
    bool init(int frames_per_s) final;

protected:
    /* The .tflite flatbuffer, 16-byte aligned. */
    virtual const void *model_bytes() const = 0;

    /* How much working memory the interpreter may have. Ask for too much once,
     * read arena_used(), then set DANCEFLOOR_ML_ARENA_KB from it. */
    virtual size_t arena_bytes() const = 0;

    /*
     * The ops this model uses.
     *
     * Owned by the subclass because MicroMutableOpResolver is templated on how
     * many it holds, and only the subclass knows. Add exactly the ops the model
     * needs -- the resolver is where an unquantised layer will announce itself,
     * as a missing op rather than as wrong answers.
     */
    virtual tflite::MicroOpResolver &ops() = 0;

    /* Called after the interpreter is ready, and again whenever the frame rate
     * changes. Return false to retire the analyser. */
    virtual bool on_rate(int frames_per_s) { (void)frames_per_s; return true; }

    /* Run it. False means the interpreter refused, which is counted by the
     * caller and should not be treated as a result. */
    bool invoke();

    TfLiteTensor *in(int index = 0);
    TfLiteTensor *out(int index = 0);

    /* What the interpreter actually needed, once allocated -- the number to set
     * DANCEFLOOR_ML_ARENA_KB from. Zero before init(). */
    size_t arena_used() const;

    /* "psram" / "internal" / "none". Worth putting in a log line: a model that
     * quietly took a satellite's internal SRAM is the failure this reports. */
    const char *arena_where() const;

private:
    ml_arena_t                 arena_{};
    const tflite::Model       *model_ = nullptr;
    tflite::MicroInterpreter  *interp_ = nullptr;
    bool                       ready_ = false;
};

}  // namespace df

#endif  /* CONFIG_DANCEFLOOR_ML_TFLM */
