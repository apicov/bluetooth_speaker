/*
 * pattern_lab -- run the firmware's LED pipeline on a laptop, against a WAV.
 *
 * The point is turnaround. Designing a pattern is a taste problem, and taste
 * needs fast feedback far more than it needs hardware; reflashing two boards
 * and listening to a track is a minute per idea, and this is a second.
 *
 * It is not a reimplementation. analysis.cpp, beat_detect.c and patterns.cpp are
 * compiled straight from components/dancefloor_leds, so what you see here is
 * what the strips do. The only difference is the FFT -- esp-dsp on the board, a
 * plain radix-2 here -- which agrees to float rounding (see test_fft.c), so a
 * threshold decision balanced on a knife edge could in principle differ. Nothing
 * that matters for designing a pattern.
 *
 *   ./pattern_lab track.wav                    live in the terminal
 *   ./pattern_lab track.wav --png out.png      whole track as an image
 *   ./pattern_lab track.wav --csv trace.csv    per-frame numbers for tuning
 *   ./pattern_lab --list                       available patterns
 *
 * Feed it the WAVs that the desktop client writes -- it lives in its own
 * repository now, ../dancefloor-tools, and needs nothing from this tree.
 */
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "analyser.hpp"
#include "analysis.hpp"
#include "result_latch.hpp"
#include "patterns.hpp"
#include "wav.hpp"
#include "out_png.hpp"
#include "out_tty.hpp"

namespace {

void usage()
{
    std::fprintf(stderr,
        "usage: pattern_lab <file.wav> [options]\n"
        "       pattern_lab --list\n\n"
        "  --pattern NAME   which pattern to run (default: the first)\n"
        "  --leds N         strip length (default 8)\n"
        "  --brightness P   percent, as the firmware applies it (default 10)\n"
        "  --png FILE       write the whole track as an image, one row per frame\n"
        "  --csv FILE       write per-frame bands, flux, threshold and onsets\n"
        "  --no-tty         skip the live terminal render\n"
        "  --speed X        terminal playback rate (default 1.0, 0 = as fast as possible)\n"
        "  --unit N         value of Frame::unit, for cross-unit effects (default 0)\n"
        "  --boom-floor X   boom detector flux floor\n"
        "  --boom-k X       boom detector threshold, in std devs\n"
        "  --boom-refr MS   boom detector refractory period\n"
        "                   any --boom-* left unset keeps the firmware's value\n"
        "  --beat-floor X   wideband detector flux floor (its k and history are\n"
        "                   compile-time -- see BEAT_THRESHOLD_K and `make HIST=`)\n");
}

}  // namespace

int main(int argc, char **argv)
{
    std::string wav_path, pattern_name, png_path, csv_path;
    int  leds = 8, unit = 0, brightness = 10;
    bool tty = true, list = false;
    double speed = 1.0;
    double boom_floor = -1, boom_k = -1, boom_refr = -1;   /* <0 = leave alone */
    double beat_floor = -1;

    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        auto next = [&](const char *what) -> std::string {
            if (i + 1 >= argc) { std::fprintf(stderr, "%s needs a value\n", what); std::exit(2); }
            return argv[++i];
        };
        if      (a == "--list")       list = true;
        else if (a == "--pattern")    pattern_name = next("--pattern");
        else if (a == "--leds")       leds = std::stoi(next("--leds"));
        else if (a == "--brightness") brightness = std::stoi(next("--brightness"));
        else if (a == "--png")        png_path = next("--png");
        else if (a == "--csv")        csv_path = next("--csv");
        else if (a == "--no-tty")     tty = false;
        else if (a == "--speed")      speed = std::stod(next("--speed"));
        else if (a == "--unit")       unit = std::stoi(next("--unit"));
        else if (a == "--boom-floor") boom_floor = std::stod(next("--boom-floor"));
        else if (a == "--boom-k")     boom_k = std::stod(next("--boom-k"));
        else if (a == "--boom-refr")  boom_refr = std::stod(next("--boom-refr"));
        else if (a == "--beat-floor") beat_floor = std::stod(next("--beat-floor"));
        else if (a == "-h" || a == "--help") { usage(); return 0; }
        else if (a.rfind("--", 0) == 0) { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
        else wav_path = a;
    }

    if (list) {
        for (int i = 0; i < df::pattern_count(); i++) {
            std::printf("%s\n", df::pattern_at(i)->name());
        }
        return 0;
    }
    if (wav_path.empty()) { usage(); return 2; }
    if (leds < 1 || leds > 4096) { std::fprintf(stderr, "--leds out of range\n"); return 2; }

    df::Pattern *pattern = pattern_name.empty() ? df::pattern_at(0)
                                                : df::pattern_by_name(pattern_name.c_str());
    if (!pattern) {
        std::fprintf(stderr, "no pattern named \"%s\" (try --list)\n", pattern_name.c_str());
        return 2;
    }

    Wav wav;
    std::string err;
    if (!wav_read(wav_path, wav, err)) {
        std::fprintf(stderr, "%s: %s\n", wav_path.c_str(), err.c_str());
        return 1;
    }
    /* The pipeline follows the file now rather than assuming 44.1 kHz, so this
     * is a note about what the tuning was measured against, not a warning that
     * the numbers below are wrong. */
    if (wav.rate != df::RATE) {
        std::fprintf(stderr,
            "note: %s is %d Hz -- analysed at that rate; the tuning defaults were "
            "measured at %d\n", wav_path.c_str(), wav.rate, df::RATE);
    }

    const size_t frames = wav.samples.size() / df::CHANNELS;
    /* Windows start every HOP_N and are FFT_N long, so the last one that fits
     * begins FFT_N from the end -- not frames/FFT_N, which only happened to be
     * right while the two were the same number. */
    const size_t blocks = frames < size_t(df::FFT_N)
                          ? 0
                          : (frames - df::FFT_N) / df::HOP_N + 1;
    std::fprintf(stderr, "%s: %.1f s, %zu analysis blocks, pattern \"%s\", %d LEDs\n",
                 wav_path.c_str(), double(frames) / wav.rate, blocks, pattern->name(), leds);
    if (blocks == 0) { std::fprintf(stderr, "too short to analyse\n"); return 1; }

    df::Analysis analysis;
    analysis.init(wav.rate);
    /* Unset flags fall back to what the firmware actually uses, not to a second
     * set of literals kept here -- sweeping one value must hold the others at
     * the shipped ones or the run measures a configuration nothing runs. */
    if (boom_floor >= 0 || boom_k >= 0 || boom_refr >= 0) {
        analysis.set_boom_tuning(
            boom_k     >= 0 ? float(boom_k)             : df::BOOM_THRESHOLD_K,
            boom_floor >= 0 ? float(boom_floor)         : df::BOOM_FLUX_FLOOR,
            boom_refr  >= 0 ? int64_t(boom_refr * 1000) : df::BOOM_REFRACTORY_US);
    }
    /* Nothing to fall back to here -- set_beat_floor touches only the floor, so
     * an unset flag simply leaves init()'s BEAT_FLUX_FLOOR in place. */
    if (beat_floor >= 0) {
        analysis.set_beat_floor(float(beat_floor));
    }
    pattern->reset();

    /*
     * The analyser lane, set up the way the firmware sets it up: a slot this
     * unit computes itself in the fast lane is cleared, and everything else --
     * slow slots included -- stays latched, because a slow result arrives
     * through the latch here exactly as it does on a board.
     */
    df::ResultLatch latch;
    bool ml_skip[df::ML_SLOTS];
    for (int i = 0; i < df::ML_SLOTS; i++) {
        df::Analyser *a = df::analyser_at(i);
        const bool fast = a && a->spec().lane == df::Lane::Fast && a->init(wav.rate);
        latch.set_latched(i, !fast);
        ml_skip[i] = !fast;
        if (a) {
            const df::AnalyserSpec &sp = a->spec();
            std::fprintf(stderr, "analyser %d: \"%s\" model %u | %s lane | shown %lld us late | %s\n",
                         i, sp.name, unsigned(sp.model_id),
                         sp.lane == df::Lane::Fast ? "fast" : "slow",
                         (long long)sp.present_delay_us,
                         fast ? "in the frame"
                              : (sp.lane == df::Lane::Slow ? "through the latch"
                                                           : "not run"));
        }
    }

    /*
     * The slow lane, driven inline rather than on a task -- there is no audio
     * clock here, so a frame is processed the moment it exists.
     *
     * There is no grid and no resampler to match any more. Analysers read the
     * frame's spectrum, and a frame is a frame: the firmware's lane does the
     * same call on its own task purely so a long inference does not stall the
     * analysis one, which is not a concern on a laptop.
     */
    df::Analyser *slow = nullptr;
    int           slow_slot = -1;
    for (int i = 0; i < df::ML_SLOTS; i++) {
        df::Analyser *a = df::analyser_at(i);
        if (a && a->spec().lane == df::Lane::Slow) {
            slow = a;
            slow_slot = i;
            break;
        }
    }
    /* Frames a second, as the firmware computes it and hands it to init(). */
    const int frames_per_s = wav.rate / df::HOP_N;
    if (slow) {
        slow->init(frames_per_s);
        std::fprintf(stderr, "slow lane: \"%s\" over %d frames/s\n",
                     slow->spec().name, frames_per_s);
    }

    std::vector<uint8_t> rgb(size_t(leds) * 3);
    std::vector<uint8_t> image;          /* blocks x leds, RGB */
    if (!png_path.empty()) image.reserve(blocks * size_t(leds) * 3);

    std::FILE *csv = nullptr;
    if (!csv_path.empty()) {
        csv = std::fopen(csv_path.c_str(), "w");
        if (!csv) { std::perror(csv_path.c_str()); return 1; }
        /* What this run was cut by, so two CSVs can never be compared across
         * hops by accident -- the row count and every flux figure change with
         * it, and nothing else in the file would say why. */
        std::fprintf(csv, "# window=%d hop=%d rate=%d\n", df::FFT_N, df::HOP_N, wav.rate);
        std::fprintf(csv, "block,time_s,band0,band1,band2,band3,flux,threshold,onset,strength,"
                             "boom_flux,boom_threshold,boom,boom_strength");
        /* One pair of columns per analyser slot, named after the analyser -- so
         * the file says which model produced which column rather than leaving a
         * reader to remember what slot 1 was on the day it was written. The
         * shape follows DF_ML_SLOTS, which the provenance line above records. */
        for (int i = 0; i < df::ML_SLOTS; i++) {
            df::Analyser *a = df::analyser_at(i);
            std::fprintf(csv, ",ml%d_%s_label,ml%d_%s_score",
                         i, a ? a->spec().name : "none",
                         i, a ? a->spec().name : "none");
        }
        std::fprintf(csv, "\n");
    }

    TtyRender tty_render;
    if (tty) tty_render.begin(leds, pattern->name());

    const float scale = brightness / 100.0f;
    size_t onsets = 0, booms = 0, ml_results = 0;

    for (size_t b = 0; b < blocks; b++) {
        const int16_t *chunk = &wav.samples[b * df::HOP_N * df::CHANNELS];
        /* From the file's rate, exactly as the firmware derives it from the
         * stream's -- otherwise every time-based pattern runs at the wrong
         * speed here and looks right on the boards. */
        const int64_t due_us = int64_t(b) * df::HOP_N * 1000000LL / wav.rate;
        /* Copied rather than referenced: the analyser slots are filled in
         * below, exactly as the firmware fills them on the way into its frame
         * queue. f.mag still points into `analysis`, which is fine -- it is
         * read before the next process(). */
        df::Frame f = analysis.process(chunk, int64_t(b), due_us, uint8_t(unit));
        if (f.onset) onsets++;
        if (f.boom)  booms++;

        /* The same lanes the boards run, from the same source files, over the
         * same bytes -- f.spec is what travels to a satellite, so this drives
         * analysers on exactly what one of those would see. */
        df::run_fast_lane(f.spec, int64_t(b), due_us, ml_skip, f.ml);

        if (slow) {
            const df::AnalyserSpec &sp = slow->spec();
            df::Result r{};
            if (slow->process(f.spec, int64_t(b), due_us, &r)) {
                r.analyser   = uint8_t(slow_slot);
                r.model_id   = sp.model_id;
                r.show_at_us = due_us + sp.present_delay_us;
                latch.publish(slow_slot, r);
            }
        }
        /* And the same latch, so a slot waiting on a result behaves here the
         * way it behaves on a strip. */
        latch.take(due_us, int64_t(df::HOP_N) * 1000000LL / wav.rate, f.ml);
        for (int i = 0; i < df::ML_SLOTS; i++) {
            if (df::result_valid(f.ml[i])) ml_results++;
        }

        pattern->render(f, rgb.data(), uint32_t(leds));

        /* The firmware scales on the way to the strip, so do it here too or the
         * image will not look like the hardware. */
        for (auto &v : rgb) v = uint8_t(v * scale);

        if (!image.empty() || !png_path.empty()) {
            image.insert(image.end(), rgb.begin(), rgb.end());
        }
        if (csv) {
            std::fprintf(csv, "%zu,%.3f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%d,%.3f,"
                              "%.5f,%.5f,%d,%.3f",
                         b, due_us / 1e6, f.band[0], f.band[1], f.band[2], f.band[3],
                         f.flux, f.threshold, f.onset ? 1 : 0, f.strength,
                         f.boom_flux, f.boom_threshold, f.boom ? 1 : 0, f.boom_strength);
            /* The label is printed as its NAME where the analyser has one -- a
             * bare class id is unreadable a week later. */
            for (int i = 0; i < df::ML_SLOTS; i++) {
                if (!df::result_valid(f.ml[i])) { std::fprintf(csv, ",,"); continue; }
                df::Analyser *a = df::analyser_at(i);
                const char *nm = a ? a->label_name(f.ml[i].label[0]) : nullptr;
                if (nm) std::fprintf(csv, ",%s,%u", nm, unsigned(f.ml[i].score[0]));
                else    std::fprintf(csv, ",%u,%u", unsigned(f.ml[i].label[0]),
                                     unsigned(f.ml[i].score[0]));
            }
            std::fprintf(csv, "\n");
        }
        if (tty) tty_render.frame(rgb.data(), leds, f, speed);
    }

    if (tty) tty_render.end();
    if (csv) { std::fclose(csv); std::fprintf(stderr, "wrote %s\n", csv_path.c_str()); }

    if (!png_path.empty()) {
        if (png_write(png_path, image.data(), leds, int(blocks), err)) {
            std::fprintf(stderr, "wrote %s (%d x %zu)\n", png_path.c_str(), leds, blocks);
        } else {
            std::fprintf(stderr, "%s: %s\n", png_path.c_str(), err.c_str());
            return 1;
        }
    }

    const double secs = double(frames) / wav.rate;
    std::fprintf(stderr, "%zu onsets in %.1f s = %.1f per minute\n",
                 onsets, secs, onsets * 60.0 / secs);
    std::fprintf(stderr, "%zu booms  in %.1f s = %.1f per minute (low band only)\n",
                 booms, secs, booms * 60.0 / secs);
    return 0;
}
