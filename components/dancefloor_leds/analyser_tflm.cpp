/*
 * The TFLM boilerplate. See analyser_tflm.hpp for how to add a model.
 *
 * NOT COMPILED BY ANY BUILD IN THIS TREE YET: DANCEFLOOR_ML_TFLM defaults to n
 * and the esp-tflite-micro dependency is commented out in idf_component.yml,
 * because nothing here has a model and the component is large.
 *
 * It does compile. Checked against esp-tflite-micro 1.3.4 headers with the
 * option forced on -- this file clean, and a throwaway subclass filling in all
 * four hooks clean beside it, with every symbol it needed from here resolving
 * including the vtable. So the API names below are real and the base class is
 * usable as one; what has NOT been proved is that a model runs, because there
 * is no model. The first build that turns this on is still the first build that
 * links against the real component.
 */
#include "analyser_tflm.hpp"

#if CONFIG_DANCEFLOOR_ML_TFLM

#include "esp_log.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

namespace df {
namespace {
constexpr const char *TAG = "tflm";
}

TflmAnalyser::~TflmAnalyser()
{
    delete interp_;
    interp_ = nullptr;
    ml_arena_give(&arena_);
}

bool TflmAnalyser::init(int stream_rate_hz)
{
    if (ready_) {
        return on_rate(stream_rate_hz);
    }

    const char *name = spec().name;

    model_ = tflite::GetModel(model_bytes());
    if (!model_) {
        ESP_LOGE(TAG, "%s: no model", name);
        return false;
    }
    /*
     * A schema mismatch is a converter/runtime version skew, and it is worth
     * failing on rather than reading through: the flatbuffer would still parse
     * into plausible-looking tensors and give answers that are simply wrong.
     */
    if (model_->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "%s: model schema %u, runtime expects %d -- reconvert it",
                 name, (unsigned)model_->version(), TFLITE_SCHEMA_VERSION);
        return false;
    }

    if (!ml_arena_take(&arena_, arena_bytes(), name)) {
        return false;                    /* ml_arena_take has already said why */
    }

    interp_ = new (std::nothrow) tflite::MicroInterpreter(
        model_, ops(), arena_.base, arena_.bytes);
    if (!interp_) {
        ESP_LOGE(TAG, "%s: no interpreter", name);
        ml_arena_give(&arena_);
        return false;
    }

    if (interp_->AllocateTensors() != kTfLiteOk) {
        /*
         * Almost always one of two things, and the log should not make anyone
         * guess which: the arena is too small, or the resolver was not given an
         * op the model uses. TFLM prints the missing op itself; the size is
         * printed here beside what was asked for.
         */
        ESP_LOGE(TAG, "%s: AllocateTensors failed with a %u B arena -- either it "
                      "is too small, or an op is missing from ops()",
                 name, (unsigned)arena_.bytes);
        delete interp_;
        interp_ = nullptr;
        ml_arena_give(&arena_);
        return false;
    }

    /*
     * Say what it needed, not just that it worked.
     *
     * arena_used() is the number DANCEFLOOR_ML_ARENA_KB should be set from, and
     * it is only knowable after allocation -- so the workflow is deliberately
     * "ask for too much once, read this line, set the option". Printing it
     * every boot means nobody has to remember that.
     */
    ESP_LOGI(TAG, "%s: ready, %u of %u B used, arena in %s",
             name, (unsigned)arena_used(), (unsigned)arena_.bytes, arena_where());

    /* An int8 input is the shape a mixed floor requires -- see the note in the
     * header. Said once, as a warning rather than a refusal: a float model is
     * correct on a floor where only one unit runs it. */
    if (const TfLiteTensor *t = in(0)) {
        if (t->type != kTfLiteInt8 && t->type != kTfLiteUInt8) {
            ESP_LOGW(TAG, "%s: input is not quantised. Two units running this "
                          "locally on different chips may disagree -- see "
                          "analyser_tflm.hpp", name);
        }
    }

    ready_ = true;
    return on_rate(stream_rate_hz);
}

bool TflmAnalyser::invoke()
{
    if (!ready_ || !interp_) {
        return false;
    }
    return interp_->Invoke() == kTfLiteOk;
}

TfLiteTensor *TflmAnalyser::in(int index)
{
    return interp_ ? interp_->input(index) : nullptr;
}

TfLiteTensor *TflmAnalyser::out(int index)
{
    return interp_ ? interp_->output(index) : nullptr;
}

size_t TflmAnalyser::arena_used() const
{
    return interp_ ? interp_->arena_used_bytes() : 0;
}

const char *TflmAnalyser::arena_where() const
{
    return ml_arena_where(&arena_);
}

}  // namespace df

#endif  /* CONFIG_DANCEFLOOR_ML_TFLM */
