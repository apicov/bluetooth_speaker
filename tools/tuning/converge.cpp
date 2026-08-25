#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "analysis.hpp"

namespace {

struct Trace {
    int window = 0, hop = 0, rate = 0;
    std::vector<int64_t> t_us;
    std::vector<std::array<float, BEAT_BANDS>> band;
};

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
        if (line[0] == 'b') continue;

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

enum class Which { Beat, Boom };

void configure(beat_det_t *d, Which w, float floor_override)
{
    beat_det_init(d);
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

}

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
            if (i >= s && i < resume) continue;
            const auto b = masked(tr.band[i], which);
            const bool on = beat_det_update(&d, b.data(), tr.t_us[i], nullptr);
            if (i < resume) continue;
            if ((on ? 1 : 0) != ref_onset[i]) { last = i - resume; count++; }
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
