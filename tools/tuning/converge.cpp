/**
 * @file converge.cpp
 * @brief converge: how long a detector that missed frames takes to agree with
 *        one that did not.
 *
 * Two units applying the same constants to the same bands make the same
 * decisions only if they hold the same state, and the state is BEAT_HIST
 * frames of flux history plus a refractory instant. So a unit that missed
 * frames disagrees with its neighbours until that history has turned over.
 * The turnover is BEAT_HIST frames exactly and needs no measuring; what needs
 * measuring is how much of it is VISIBLE, since a difference in the threshold
 * changes a decision only when the flux lands between the two thresholds. That
 * smaller number is what a strip shows, and it is what this reports.
 *
 * The input is a pattern_lab --csv, and only its band0..band3 columns are
 * read. That is a constraint rather than a convenience: Frame::band is four
 * floats that already travel between units, while Frame::mag is 512 floats
 * that are local-only and null on a received frame. A probe that needed mag
 * would be measuring something a satellite could not do.
 *
 * C++ rather than C so the boom detector's constants can come from
 * analysis.hpp. A tool with its own copy of what the firmware does measures
 * something other than what it reports.
 *
 *   make && ./converge trace.csv --detector boom --gap 10
 *
 * @see BEAT_HIST in components/dancefloor_leds/include/beat_detect.h, which
 *      carries the tuning argument this probe supplies the other half of.
 */
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "analysis.hpp"      /* BOOM_* and, through it, beat_detect.h */

namespace {

/** @brief One pattern_lab --csv, reduced to what this probe reads. */
struct Trace {
    int window = 0;   /**< FFT length the trace was cut with. */
    int hop = 0;      /**< Samples between frames; the only thing that turns frames into milliseconds. */
    int rate = 0;     /**< Sample rate the trace was analysed at. */
    std::vector<int64_t> t_us;                            /**< Each frame's instant. */
    std::vector<std::array<float, BEAT_BANDS>> band;      /**< Each frame's four band values. */
};

/**
 * @brief Read a pattern_lab --csv into a Trace.
 *
 * The "# window=... hop=... rate=..." header is required, not decoration:
 * every figure this program prints is in frames, and frames are only
 * milliseconds once the hop is known. A trace without it is rejected rather
 * than assumed.
 *
 * @param path  The CSV.
 * @param out   Filled in on success.
 * @param err   Set to a short reason when this returns false.
 * @return True if the header was found and at least 64 frames were read.
 */
bool read_trace(const char *path, Trace &out, std::string &err)
{
    std::FILE *f = std::fopen(path, "r");
    if (!f) { err = std::string(path) + ": cannot open"; return false; }

    char line[4096];
    bool have_header = false;
    while (std::fgets(line, sizeof line, f)) {
        if (line[0] == '#') {
            if (std::sscanf(line, "# window=%d hop=%d rate=%d",
                            &out.window, &out.hop, &out.rate) == 3) {
                have_header = true;
            }
            continue;
        }
        if (line[0] == 'b') continue;            /* the column header */

        /* block,time_s,band0,band1,band2,band3,... -- everything past the
         * bands is the firmware's own decisions, which this recomputes and
         * must not read. */
        double time_s;
        float b[BEAT_BANDS];
        size_t blk;
        if (std::sscanf(line, "%zu,%lf,%f,%f,%f,%f",
                        &blk, &time_s, &b[0], &b[1], &b[2], &b[3]) != 6) {
            continue;
        }
        out.t_us.push_back(int64_t(time_s * 1e6));
        std::array<float, BEAT_BANDS> a;
        for (int i = 0; i < BEAT_BANDS; i++) a[i] = b[i];
        out.band.push_back(a);
    }
    std::fclose(f);

    if (!have_header) { err = std::string(path) + ": no '# window=... hop=...' header"; return false; }
    if (out.t_us.size() < 64) { err = std::string(path) + ": too few frames"; return false; }
    return true;
}

/** @brief Which of the firmware's two detectors is under test. */
enum class Which {
    Beat,   /**< The wideband one, over all four bands. */
    Boom,   /**< The low-band one; the upper three bands are masked to zero. */
};

/**
 * @brief Set a detector up exactly as Analysis::init() sets it up.
 *
 * @param d               Zeroed and initialised in place.
 * @param w               Which detector's constants to install.
 * @param floor_override  Applied last when non-negative, so a sweep over the
 *                        floor holds everything else at the shipped values.
 */
void configure(beat_det_t *d, Which w, float floor_override)
{
    beat_det_init(d);                       /* installs the wideband defaults */
    if (w == Which::Boom) {
        d->threshold_k   = df::BOOM_THRESHOLD_K;
        d->refractory_us = df::BOOM_REFRACTORY_US;
        d->flux_floor    = df::BOOM_FLUX_FLOOR;
    }
    if (floor_override >= 0.0f) d->flux_floor = floor_override;
}

/**
 * @brief Reduce a frame to what the chosen detector sees.
 * @param in  All four bands.
 * @param w   Beat passes them through; Boom keeps band 0 and zeroes the rest,
 *            which is how analysis.cpp reduces the same detector to the low
 *            band alone.
 * @return The bands to feed beat_det_update().
 */
std::array<float, BEAT_BANDS> masked(const std::array<float, BEAT_BANDS> &in, Which w)
{
    if (w == Which::Beat) return in;
    return { in[0], 0.0f, 0.0f, 0.0f };
}

/**
 * @brief A percentile, by nearest rank.
 * @param v  Taken by value because it is sorted in place.
 * @param p  0..1.
 * @return The value at that rank, or 0 for an empty input.
 */
double pct(std::vector<int> v, double p)
{
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    size_t i = size_t(std::ceil(p * double(v.size()))) - 1;
    return v[std::min(i, v.size() - 1)];
}

/** @brief Print the option list to stderr. Used for -h and for a bad call. */
void usage()
{
    std::fprintf(stderr,
        "usage: converge <trace.csv> [options]\n\n"
        "  --detector beat|boom   which of the firmware's two (default beat)\n"
        "  --gap N                frames the lagging unit misses (default 10)\n"
        "  --trials N             loss events, spread over the track (default 40)\n"
        "  --floor X              override the detector's flux floor\n");
}

}

/**
 * @brief Run one reference detector over the whole trace, then one lagging
 *        detector per trial, and report how they differ.
 *
 * Three figures come out, all per trial and reported as percentiles:
 * state_*, the frames until the two thresholds are bit-equal, which is history
 * equality and is bounded by BEAT_HIST by construction; converge_*, the frames
 * until the last DISAGREEING decision, which is the visible number; and
 * disagree_mean, how many decisions differed at all.
 *
 * @param argc  As usual.
 * @param argv  As usual; see usage() for what is accepted.
 * @return 0 on success, 1 on an unreadable or too-short trace, 2 on a bad
 *         argument.
 */
int main(int argc, char **argv)
{
    std::string path;
    Which which = Which::Beat;
    int gap = 10, trials = 40;
    float floor_override = -1.0f;

    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        auto next = [&](const char *what) -> std::string {
            if (i + 1 >= argc) { std::fprintf(stderr, "%s needs a value\n", what); std::exit(2); }
            return argv[++i];
        };
        if      (a == "--detector") {
            const std::string v = next("--detector");
            if (v == "beat") which = Which::Beat;
            else if (v == "boom") which = Which::Boom;
            else { std::fprintf(stderr, "--detector must be beat or boom\n"); return 2; }
        }
        else if (a == "--gap")    gap    = std::stoi(next("--gap"));
        else if (a == "--trials") trials = std::stoi(next("--trials"));
        else if (a == "--floor")  floor_override = std::stof(next("--floor"));
        else if (a == "-h" || a == "--help") { usage(); return 0; }
        else if (a.rfind("--", 0) == 0) { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
        else path = a;
    }
    if (path.empty()) { usage(); return 2; }
    if (gap < 1 || trials < 1) { std::fprintf(stderr, "--gap and --trials must be positive\n"); return 2; }

    Trace tr;
    std::string err;
    if (!read_trace(path.c_str(), tr, err)) { std::fprintf(stderr, "%s\n", err.c_str()); return 1; }

    const int n = int(tr.t_us.size());

    /* The reference unit: fed every frame, once, since it is the same run for
     * every trial. */
    std::vector<char>  ref_onset(n);
    std::vector<float> ref_thresh(n);
    {
        beat_det_t d;
        configure(&d, which, floor_override);
        for (int i = 0; i < n; i++) {
            const auto b = masked(tr.band[i], which);
            ref_onset[i]  = beat_det_update(&d, b.data(), tr.t_us[i], nullptr) ? 1 : 0;
            ref_thresh[i] = beat_det_last_threshold(&d);
        }
    }

    /* Trials are spread over the track rather than repeated at one point,
     * because a loss during a quiet passage and a loss during a chorus are
     * different events and the interesting figure is the tail. The margins
     * keep every resume point clear of the warm-up at the start and leave room
     * to observe convergence before the end. */
    const int lo = std::max(4 * BEAT_HIST, 64);
    const int hi = n - (8 * BEAT_HIST + gap) - 1;
    if (hi <= lo) { std::fprintf(stderr, "track too short for %d trials at BEAT_HIST %d\n", trials, BEAT_HIST); return 1; }

    std::vector<int> last_disagree, disagree_count, state_frames;
    for (int t = 0; t < trials; t++) {
        const int s = lo + int(int64_t(t) * (hi - lo) / std::max(1, trials - 1));
        const int resume = s + gap;

        beat_det_t d;
        configure(&d, which, floor_override);
        int last = -1, count = 0, state = -1;
        for (int i = 0; i < n; i++) {
            if (i >= s && i < resume) continue;           /* the frames it never saw */
            const auto b = masked(tr.band[i], which);
            const bool on = beat_det_update(&d, b.data(), tr.t_us[i], nullptr);
            if (i < resume) continue;
            if ((on ? 1 : 0) != ref_onset[i]) { last = i - resume; count++; }
            /* The pure-state figure: the threshold is a function of the
             * history alone, so bit-equality of it is history equality.
             * Recorded to confirm the BEAT_HIST bound, not to discover it. */
            if (state < 0 && beat_det_last_threshold(&d) == ref_thresh[i]) state = i - resume;
        }
        last_disagree.push_back(last < 0 ? 0 : last + 1);
        disagree_count.push_back(count);
        state_frames.push_back(state < 0 ? n : state);
    }

    const double frame_ms = 1000.0 * double(tr.hop) / double(tr.rate);
    long total = 0;
    int zero = 0;
    for (size_t i = 0; i < disagree_count.size(); i++) {
        total += disagree_count[i];
        if (disagree_count[i] == 0) zero++;
    }

    /* One line of key=value, so a sweep over BEAT_HIST reads as a table and
     * sweep.py can parse it by splitting on whitespace. */
    std::printf("hist=%d detector=%s hop=%d gap=%d trials=%d "
                "state_p50=%.0f state_p95=%.0f "
                "converge_p50=%.0f converge_p95=%.0f converge_p95_ms=%.1f "
                "disagree_mean=%.2f clean_trials=%d/%d\n",
                BEAT_HIST, which == Which::Beat ? "beat" : "boom", tr.hop, gap, trials,
                pct(state_frames, 0.50), pct(state_frames, 0.95),
                pct(last_disagree, 0.50), pct(last_disagree, 0.95),
                pct(last_disagree, 0.95) * frame_ms,
                double(total) / double(disagree_count.size()), zero, trials);
    return 0;
}
