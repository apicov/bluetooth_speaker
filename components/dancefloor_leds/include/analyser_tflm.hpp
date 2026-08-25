
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

    bool init(int frames_per_s) final;

protected:

    virtual const void *model_bytes() const = 0;

    virtual size_t arena_bytes() const = 0;

    virtual tflite::MicroOpResolver &ops() = 0;

    virtual bool on_rate(int frames_per_s) { (void)frames_per_s; return true; }

    bool invoke();

    TfLiteTensor *in(int index = 0);
    TfLiteTensor *out(int index = 0);

    size_t arena_used() const;

    const char *arena_where() const;

private:
    ml_arena_t                 arena_{};
    const tflite::Model       *model_ = nullptr;
    tflite::MicroInterpreter  *interp_ = nullptr;
    bool                       ready_ = false;
};

}

#endif
