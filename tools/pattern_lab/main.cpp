/**
 * @file main.cpp
 * @brief pattern_lab: run the firmware's LED pipeline on a laptop, against a
 *        WAV file, and show or record what it produces.
 *
 * Not a reimplementation. analysis.cpp, patterns.cpp, analysers.cpp and
 * beat_detect.c are compiled straight out of components/dancefloor_leds, so
 * what this renders is what the strips render. The one substitution is the
 * FFT: esp-dsp on the board, the portable radix-2 in fft_host.c here.
 *
 * Four outputs, which can be combined:
 *
 *   --png     the whole track as an image, one row per analysis frame
 *   --csv     per-frame bands, flux, thresholds, onsets and analyser results
 *   --dump    per-frame spec, mag and band as binary, for a notebook
 *   (default) a live render in the terminal, paced to the audio
 *
 * @see out_tty.hpp, out_png.hpp, dump_load.py
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

/** @brief Print the option list to stderr. Used for -h and for a bad call. */
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
        "  --dump PREFIX    write per-frame spec (u8 x64), mag (f32 x512) and\n"
        "                   band (f32 x4) as binary, plus a PREFIX.meta sidecar\n"
        "                   carrying the grid -- usually with --no-tty; read back\n"
        "                   with dump_load.py\n"
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

}

/**
 * @brief Parse the arguments, then run one pass over the file.
 *
 * The pass is a single loop over analysis blocks that does, in order, what a
 * satellite does per frame: analyse, run the fast analyser lane, poll the slow
 * one through the latch, render the pattern, scale for brightness, and hand
 * the pixels to whichever outputs are enabled.
 *
 * @param argc  As usual.
 * @param argv  As usual; see usage() for what is accepted.
 * @return 0 on success, 1 on an I/O or too-short-file failure, 2 on a bad
 *         argument.
 */
int main(int argc, char **argv)
{
    std::string wav_path, pattern_name, png_path, csv_path, dump_prefix;
    int  leds = 8, unit = 0, brightness = 10;
    bool tty = true, list = false;
    double speed = 1.0;
    /* Negative means "not given", which is how an unset tuning flag is told
     * apart from one deliberately set to zero. */
    double boom_floor = -1, boom_k = -1, boom_refr = -1;
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
        else if (a == "--dump")       dump_prefix = next("--dump");
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
    /* The pipeline follows the file's rate rather than assuming df::RATE, so
     * this says what the tuning was measured against. It is not a warning that
     * the numbers below are wrong. */
    if (wav.rate != df::RATE) {
        std::fprintf(stderr,
            "note: %s is %d Hz -- analysed at that rate; the tuning defaults were "
            "measured at %d\n", wav_path.c_str(), wav.rate, df::RATE);
    }

    const size_t frames = wav.samples.size() / df::CHANNELS;
    /* Windows start every HOP_N and are FFT_N long, so the last one that fits
     * begins FFT_N from the end. Not frames/FFT_N, which is only the same
     * number while the window and the hop are. */
    const size_t blocks = frames < size_t(df::FFT_N)
                          ? 0
                          : (frames - df::FFT_N) / df::HOP_N + 1;
    std::fprintf(stderr, "%s: %.1f s, %zu analysis blocks, pattern \"%s\", %d LEDs\n",
                 wav_path.c_str(), double(frames) / wav.rate, blocks, pattern->name(), leds);
    if (blocks == 0) { std::fprintf(stderr, "too short to analyse\n"); return 1; }

    df::Analysis analysis;
    analysis.init(wav.rate);
    /* An unset flag falls back to what the firmware uses, not to a second set
     * of literals kept here: sweeping one value has to hold the others at the
     * shipped ones or the run measures a configuration nothing runs. */
    if (boom_floor >= 0 || boom_k >= 0 || boom_refr >= 0) {
        analysis.set_boom_tuning(
            boom_k     >= 0 ? float(boom_k)             : df::BOOM_THRESHOLD_K,
            boom_floor >= 0 ? float(boom_floor)         : df::BOOM_FLUX_FLOOR,
            boom_refr  >= 0 ? int64_t(boom_refr * 1000) : df::BOOM_REFRACTORY_US);
    }
    /* Nothing to fall back to here -- set_beat_floor() touches only the floor,
     * so an unset flag leaves init()'s BEAT_FLUX_FLOOR in place. */
    if (beat_floor >= 0) {
        analysis.set_beat_floor(float(beat_floor));
    }
    pattern->reset();

    /* The analyser lane, set up the way the firmware sets it up: a slot this
     * unit computes itself in the fast lane is cleared each frame, and
     * everything else -- slow slots included -- stays latched, so a slow result
     * arrives here through the latch exactly as it does on a board. */
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

    /* The slow lane, driven inline rather than on a task. There is no audio
     * clock here, so a frame is processed the moment it exists; the firmware
     * puts this on its own task purely so a long inference cannot stall the
     * analysis one, which is not a concern on a laptop. */
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
        /* What this run was cut by, so two CSVs cannot be compared across
         * hops by accident -- the row count and every flux figure change with
         * it, and nothing else in the file would say why. */
        std::fprintf(csv, "# window=%d hop=%d rate=%d\n", df::FFT_N, df::HOP_N, wav.rate);
        std::fprintf(csv, "block,time_s,band0,band1,band2,band3,flux,threshold,onset,strength,"
                             "boom_flux,boom_threshold,boom,boom_strength");
        /* One pair of columns per analyser slot, named after the analyser, so
         * the file says which model produced which column. */
        for (int i = 0; i < df::ML_SLOTS; i++) {
            df::Analyser *a = df::analyser_at(i);
            std::fprintf(csv, ",ml%d_%s_label,ml%d_%s_score",
                         i, a ? a->spec().name : "none",
                         i, a ? a->spec().name : "none");
        }
        std::fprintf(csv, "\n");
    }

    /*
     * The per-frame dump, written by the same loop that feeds the strips, so a
     * notebook trains against what the firmware computed rather than against a
     * re-derivation of it.
     *
     * spec is byte-for-byte what a satellite's analyser lane is handed. mag is
     * the one field of df::Frame no later consumer could recover: it points
     * into `analysis` and dies at the next process(), so this loop is the last
     * place it exists. band rides along because it costs sixteen bytes and the
     * wire carries it at full float precision.
     *
     * Binary rather than CSV, because a float32 survives a decimal round-trip
     * only at nine significant digits and pipeline.py's validator measures
     * differences below what a printed column could hold. The sidecar carries
     * the grid for the same reason the CSV's provenance line does, and its
     * dtype strings let dump_load.py size the arrays if SPEC_BINS or FFT_N
     * ever move. Native byte order, little-endian everywhere this builds.
     */
    std::FILE *dump_spec = nullptr, *dump_mag = nullptr, *dump_band = nullptr;
    if (!dump_prefix.empty()) {
        const std::string spec_path = dump_prefix + ".spec.bin";
        const std::string mag_path  = dump_prefix + ".mag.bin";
        const std::string band_path = dump_prefix + ".band.bin";
        dump_spec = std::fopen(spec_path.c_str(), "wb");
        if (!dump_spec) { std::perror(spec_path.c_str()); return 1; }
        dump_mag = std::fopen(mag_path.c_str(), "wb");
        if (!dump_mag)  { std::perror(mag_path.c_str()); return 1; }
        dump_band = std::fopen(band_path.c_str(), "wb");
        if (!dump_band) { std::perror(band_path.c_str()); return 1; }

        const std::string meta_path = dump_prefix + ".meta";
        std::FILE *meta = std::fopen(meta_path.c_str(), "w");
        if (!meta) { std::perror(meta_path.c_str()); return 1; }
        std::fprintf(meta,
            "# pattern_lab per-frame dump; written by --dump, read by dump_load.py\n"
            "# Row b is block b. index is 0..frames-1 and due_us is\n"
            "# index*hop*1000000/rate in int64 -- derivable, but recorded so the\n"
            "# derivation lives in one place.\n"
            "window=%d\n"
            "hop=%d\n"
            "rate=%d\n"
            "frames=%zu\n"
            "spec_bins=%d\n"
            "spec_dtype=u1\n"
            "mag_bins=%d\n"
            "mag_dtype=<f4\n"
            "band_bins=%d\n"
            "band_dtype=<f4\n",
            df::FFT_N, df::HOP_N, wav.rate, blocks,
            df::SPEC_BINS, df::BINS, BEAT_BANDS);
        std::fclose(meta);
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

        /* Before anything else touches the frame: f.mag is only readable
         * until the next process(), and it is the whole reason for the dump. */
        if (dump_spec) {
            std::fwrite(f.spec, 1, df::SPEC_BINS, dump_spec);
            std::fwrite(f.mag, sizeof(float), df::BINS, dump_mag);
            std::fwrite(f.band, sizeof(float), BEAT_BANDS, dump_band);
        }

        /* The same lanes the boards run, from the same source files, over the
         * same bytes -- f.spec is what travels to a satellite, so this drives
         * the analysers on exactly what one of those would see. */
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

        /* The firmware scales on the way to the strip, so scale here too or
         * the image will not look like the hardware. */
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
            /* The label is printed as its NAME where the analyser has one; a
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
    if (dump_spec) {
        std::fclose(dump_spec);
        std::fclose(dump_mag);
        std::fclose(dump_band);
        std::fprintf(stderr, "wrote %s.spec.bin, %s.mag.bin, %s.band.bin, %s.meta "
                             "(%zu frames)\n",
                     dump_prefix.c_str(), dump_prefix.c_str(), dump_prefix.c_str(),
                     dump_prefix.c_str(), blocks);
    }

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
