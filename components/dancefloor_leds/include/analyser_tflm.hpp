/**
 * @file analyser_tflm.hpp
 * @brief Dropping a TensorFlow Lite Micro model into the analyser lane.
 *
 * Everything an analyser needs from the firmware is already in analyser.hpp;
 * this only removes the boilerplate every TFLM analyser would otherwise
 * repeat -- getting an arena from the right heap, checking the schema version,
 * building an interpreter, and failing cleanly enough that a unit which cannot
 * run the model still lights up from its neighbours.
 *
 * ADDING A MODEL
 *
 * 1. Enable the dependency. This component's idf_component.yml has the exact
 *    lines, commented out -- commented rather than present because the TFLM
 *    component is large and no build in this tree needs it until yours does.
 * 2. Turn on the TFLM Kconfig option, and set the arena size from what the
 *    interpreter reports: ask for too much once, read arena_used(), then set
 *    it.
 * 3. Convert the model to a C array. It must be 16-byte aligned; declare it
 *    `alignas(16) const unsigned char ...[]`.
 * 4. Subclass TflmAnalyser, fill in the hooks below, and add it to the
 *    registry in analysers.cpp. The spec is the same one every analyser
 *    declares -- and above all df::AnalyserSpec::present_delay_us, whose rule
 *    is the thing most likely to be got wrong.
 * 5. process() is handed one frame's quantised SPECTRUM, not audio. A model
 *    wanting a spectrogram accumulates frames itself and returns false until
 *    it has enough; the bins are already uint8 on a fixed compression curve,
 *    which is most of the way to the int8 input a quantised model wants.
 *
 * THE MODEL MUST BE INT8, AND THAT IS NOT ABOUT SIZE. A unit running a model
 * locally must produce the same answer as its neighbours, bit for bit -- the
 * rule at the top of analyser.hpp. The hub and a satellite are different
 * cores, and the vendor's kernel library selects different kernels per target.
 * Integer arithmetic is identical on both; float kernels are identical only by
 * coincidence, and nothing checks. So a float model here is not slow, it is
 * WRONG on a mixed floor, and wrong in the way that takes an evening to find:
 * two strips that agree on the bench and separate in the room.
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

/** @brief Base for an analyser backed by a TFLM model. Subclass it and fill in
 *         the hooks; init() does the rest. */
class TflmAnalyser : public Analyser {
public:
    /** @brief Tear down the interpreter and give the arena back. */
    ~TflmAnalyser() override;

    /**
     * @brief Take an arena, check the model, and build an interpreter.
     *
     * final, because the sequence is the whole point of this class. A
     * subclass wanting to do something with the rate overrides on_rate()
     * instead.
     *
     * @param frames_per_s  Analysis frames per second; passed to on_rate().
     * @return false if the arena could not be had, the model's schema version
     *         is not the one this TFLM build speaks, tensor allocation failed,
     *         or on_rate() refused. The lane then skips this analyser and says
     *         so once.
     */
    bool init(int frames_per_s) final;

protected:
    /** @brief The model flatbuffer. Must be 16-byte aligned and must outlive
     *         this object -- in practice a `const` array in flash.
     *  @return Its address. */
    virtual const void *model_bytes() const = 0;

    /** @brief How much working memory the interpreter needs. Ask for too much
     *         once, read arena_used(), then set this.
     *  @return The arena size in bytes. */
    virtual size_t arena_bytes() const = 0;

    /** @brief The operator resolver, carrying exactly the kernels this model
     *         uses -- a full resolver would link every kernel TFLM has.
     *  @return A reference the interpreter keeps, so it must outlive this
     *          object. */
    virtual tflite::MicroOpResolver &ops() = 0;

    /** @brief Optional hook: check or size something against the frame rate.
     *  @param frames_per_s  Analysis frames per second.
     *  @return false to refuse this build, which fails init(). */
    virtual bool on_rate(int frames_per_s) { (void)frames_per_s; return true; }

    /** @brief Run the interpreter over whatever is in the input tensor.
     *  @return false if it failed or the analyser never became ready. */
    bool invoke();

    /** @brief An input tensor. @param index Which. @return It, or null if the
     *         analyser is not ready. */
    TfLiteTensor *in(int index = 0);
    /** @brief An output tensor. @param index Which. @return It, or null if the
     *         analyser is not ready. */
    TfLiteTensor *out(int index = 0);

    /** @brief What the interpreter actually used of the arena -- the number to
     *         set arena_bytes() from. @return Bytes used, or 0. */
    size_t arena_used() const;

    /** @brief Which heap the arena came from, for the log line.
     *  @return "psram", "internal" or "none". */
    const char *arena_where() const;

private:
    ml_arena_t                 arena_{};          /**< See ml_arena.h. */
    const tflite::Model       *model_ = nullptr;  /**< The parsed flatbuffer. */
    tflite::MicroInterpreter  *interp_ = nullptr; /**< Placement-new'd; see init(). */
    bool                       ready_ = false;    /**< Whether init() succeeded. */
};

}  // namespace df

#endif /* CONFIG_DANCEFLOOR_ML_TFLM */
