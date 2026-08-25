
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

bool TflmAnalyser::init(int frames_per_s)
{
    if (ready_) {
        return on_rate(frames_per_s);
    }

    const char *name = spec().name;

    model_ = tflite::GetModel(model_bytes());
    if (!model_) {
        ESP_LOGE(TAG, "%s: no model", name);
        return false;
    }

    if (model_->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "%s: model schema %u, runtime expects %d -- reconvert it",
                 name, (unsigned)model_->version(), TFLITE_SCHEMA_VERSION);
        return false;
    }

    if (!ml_arena_take(&arena_, arena_bytes(), name)) {
        return false;
    }

    interp_ = new (std::nothrow) tflite::MicroInterpreter(
        model_, ops(), arena_.base, arena_.bytes);
    if (!interp_) {
        ESP_LOGE(TAG, "%s: no interpreter", name);
        ml_arena_give(&arena_);
        return false;
    }

    if (interp_->AllocateTensors() != kTfLiteOk) {

        ESP_LOGE(TAG, "%s: AllocateTensors failed with a %u B arena -- either it "
                      "is too small, or an op is missing from ops()",
                 name, (unsigned)arena_.bytes);
        delete interp_;
        interp_ = nullptr;
        ml_arena_give(&arena_);
        return false;
    }

    ESP_LOGI(TAG, "%s: ready, %u of %u B used, arena in %s",
             name, (unsigned)arena_used(), (unsigned)arena_.bytes, arena_where());

    if (const TfLiteTensor *t = in(0)) {
        if (t->type != kTfLiteInt8 && t->type != kTfLiteUInt8) {
            ESP_LOGW(TAG, "%s: input is not quantised. Two units running this "
                          "locally on different chips may disagree -- see "
                          "analyser_tflm.hpp", name);
        }
    }

    ready_ = true;
    return on_rate(frames_per_s);
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

}

#endif
