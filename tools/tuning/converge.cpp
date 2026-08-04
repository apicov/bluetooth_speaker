/*
 * converge -- how long a detector that missed frames takes to agree again.
 *
 * BEAT_HIST is the only tuning constant whose MEANING changed when the hop
 * halved rather than its scale: it is a frame count, so the wall-clock it spans
 * went from ~1.0 s to ~0.5 s. Deciding it has two axes. The first is ordinary
 * tuning -- a shorter history adapts faster but estimates the mean and standard
 * deviation from fewer samples -- and the corpus answers that, in sweep.py.
 *
 * This answers the second. Under the coming third source mode (the hub does the
 * FFT, each satellite runs its own detector on the bands it is sent), two units
 * apply these constants to the SAME received bands, so identical decisions
 * require identical state -- and the state is BEAT_HIST frames of flux history
 * plus a refractory instant. A unit that missed frames therefore disagrees with
 * its neighbours until that history has turned over, and BEAT_HIST sets how
 * long. See the note at the top of components/dancefloor_leds/include/beat_detect.h.
 *
 * The turnover is BEAT_HIST frames exactly, and that much needs no measuring.
 * What needs measuring is how much of it is VISIBLE: a difference in the
 * threshold only changes a decision when the flux lands between the two
 * thresholds, so the frames that actually disagree are far fewer than the frames
 * whose state differs. That number is what a strip shows.
 *
 * It reads a pattern_lab --csv and uses band0..band3 and nothing else. That is
 * not a convenience -- it is the third mode's constraint made executable.
 * Frame::band is four floats at full precision and already travels; Frame::mag
 * is 512 floats, local-only and null on any received frame; Frame::spec is 8-bit
 * through x/(1+x) and quantises exactly the signal the detector runs on. A probe
 * that needed either of the latter two would be measuring something that mode
 * cannot do.
 *
 * C++ rather than C for one reason: the boom detector's constants live in
 * analysis.hpp, and duplicating them here is the precise mistake that header
 * records pattern_lab having made -- a tool with its own idea of what the
 * firmware does measures something other than what it reports.
 *
 *   make && ./converge trace.csv --detector boom --gap 10
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

struct Trace {
    int window = 0, hop = 0, rate = 0;
    std::vector<int64_t> t_us;
    std::vector<std::array<float, BEAT_BANDS>> band;
};

/*
 * The header pattern_lab writes is "# window=1024 hop=512 rate=44100", and it
 * is not decoration: every figure below is in frames, and frames are only
 * milliseconds once the hop is known. A trace whose header is missing is
 * rejected rather than assumed.
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

        /* block,time_s,band0,band1,band2,band3,... -- the rest is the
         * firmware's own decisions, which this recomputes and must not read. */
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

/* The two detectors the firmware runs, configured exactly as Analysis::init()
 * configures them. `boom` additionally masks the upper three bands to zero,
 * which is how analysis.cpp reduces the same detector to the low band alone. */
enum class Which { Beat, Boom };

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

std::array<float, BEAT_BANDS> masked(const std::array<float, BEAT_BANDS> &in, Which w)
{
    if (w == Which::Beat) return in;
    return { in[0], 0.0f, 0.0f, 0.0f };
}

double pct(std::vector<int> v, double p)
{
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    size_t i = size_t(std::ceil(p * double(v.size()))) - 1;
    return v[std::min(i, v.size() - 1)];
}

void usage()
{
    std::fprintf(stderr,
        "usage: converge <trace.csv> [options]\n\n"
        "  --detector beat|boom   which of the firmware's two (default beat)\n"
        "  --gap N                frames the lagging unit misses (default 10)\n"
        "  --trials N             loss events, spread over the track (default 40)\n"
        "  --floor X              override the detector's flux floor\n");
}

}  // namespace

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

    /*
     * Trials are spread over the track rather than repeated at one point,
     * because a loss during a quiet passage and a loss during a chorus are
     * different events and the interesting figure is the tail, not the mean.
     * The margins keep every trial's resume point clear of the warm-up at the
     * start and leave room to observe convergence before the end.
     */
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
            /* The pure-state figure: the threshold is a function of the history
             * alone, so bit-equality of it is history equality. Bounded by
             * BEAT_HIST by construction; recorded to confirm that, not to
             * discover it. */
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

    /* One line, so a sweep over BEAT_HIST reads as a table. */
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
