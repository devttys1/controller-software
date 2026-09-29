// Host test for yaskawa_index.h, run by run_host_tests.sh.
//
// Replays real frames from the bench (data/yaskawa_index_frames.csv, built by make_yaskawa_index_fixtures.py
// from three recordings of 2026-09-26: 13 encoder power-ups, 83 index crossings) and checks that the
// index is found where the encoder latched it, that the shaft angle from the index reads ~0 at every
// crossing, that whole turns step by exactly one between crossings, and that the mark sits at the same
// electrical angle in every power-up. Synthetic cases cover what the recordings do not.
//
// sources:

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "yaskawa_index.h"

namespace {

constexpr int32_t CPR = yaskawa_index::counts_per_rev;
int failures = 0;

void expect(bool ok, const std::string& what){
    if(!ok){
        std::printf("  FAIL %s\n", what.c_str());
        failures++;
    }
}

struct Frame {
    int segment;
    uint32_t single_turn;
    int64_t position;
    uint32_t raw[3];
    double elec;
    int hallref;
};

std::vector<Frame> load(){
    std::vector<Frame> frames;
    std::ifstream f("data/yaskawa_index_frames.csv");
    std::string line;
    std::getline(f, line);
    while(std::getline(f, line)){
        std::stringstream ss(line);
        std::string v[8];
        for(auto& s : v) std::getline(ss, s, ',');
        frames.push_back({std::stoi(v[0]), uint32_t(std::stoul(v[1])), std::stoll(v[2]),
                          {uint32_t(std::stoul(v[3])), uint32_t(std::stoul(v[4])), uint32_t(std::stoul(v[5]))},
                          std::stod(v[6]), std::stoi(v[7])});
    }
    return frames;
}

double frac(double x){ return x - std::floor(x); }

// wrap to (-0.5, 0.5]
double wrap_turn(double x){ return x - std::floor(x + 0.5); }

void test_decode(){
    // a frame from the bench: not found yet, then the first crossing (index_position 118, count -1)
    uint32_t before[3] = {0, 0, 0};
    before[0] = 1u << 6;
    auto f = yaskawa_index::decode(before);
    expect(f.not_found && !f.pulse && f.coarse == 0 && f.count == 0, "power-up frame");

    uint32_t after[3] = {0, 1u << (56 - 32), (118u << (80 - 64)) | (127u << (89 - 64))};
    f = yaskawa_index::decode(after);
    expect(!f.not_found && f.pulse && f.coarse == 118 && f.count == -1, "first crossing frame: pulse, 118, -1");
}

void test_bench_frames(){
    auto frames = load();
    expect(frames.size() > 10000, "fixture loaded (" + std::to_string(frames.size()) + " frames)");

    yaskawa_index idx;
    int crossings = 0, placed = 0, worst_angle = 0, worst_excess = -1000, bad_turn_steps = 0, turn_steps = 0;
    int out_of_window = 0;
    std::map<uint32_t, std::vector<double>> index_elec;     // per power-up (latched coarse): the mark's hall-frame angle
    double last_turns[2] = {NAN, NAN};
    int last_segment = -1;
    uint32_t last_coarse = 0;

    for(size_t i = 0; i < frames.size(); i++){
        const Frame& fr = frames[i];
        idx.update(fr.single_turn, fr.position, fr.raw);
        if(!idx.is_found()){
            continue;
        }
        auto f = yaskawa_index::decode(fr.raw);
        if(f.coarse != last_coarse){
            last_turns[0] = last_turns[1] = NAN;
            last_coarse = f.coarse;
        }
        int32_t rel = idx.index_single_turn() - int32_t(f.coarse) * 256;
        rel = ((rel + CPR / 2) % CPR + CPR) % CPR - CPR / 2;
        out_of_window += rel < -256 || rel >= 512;

        if(i > 0 && fr.segment == frames[i - 1].segment && frames[i - 1].raw[2] != fr.raw[2]){
            auto p = yaskawa_index::decode(frames[i - 1].raw);
            if(p.count != f.count && !p.not_found){
                crossings++;
                int64_t mid = (frames[i - 1].position + fr.position) / 2;
                int64_t step = std::llabs(fr.position - frames[i - 1].position);
                if(step <= yaskawa_index::max_step){
                    placed++;
                    // the shaft is at the mark: angle ~0, to within half the frame's step (where between the
                    // two frames it crossed) and the ~30 count difference between the directions
                    int err = int(std::lround(std::fabs(wrap_turn(idx.turns(mid))) * CPR));
                    worst_angle = std::max(worst_angle, err);
                    worst_excess = std::max(worst_excess, err - int(step / 2));
                    // same-direction crossings are whole turns apart
                    int d = fr.position > frames[i - 1].position ? 0 : 1;
                    double t = idx.turns(mid);
                    if(!std::isnan(last_turns[d]) && fr.segment - last_segment < 1000){
                        turn_steps++;
                        double gap = t - last_turns[d];
                        bad_turn_steps += std::fabs(gap - std::round(gap)) > 0.005 || std::round(gap) == 0;
                    }
                    last_turns[d] = t;
                    last_segment = fr.segment;
                }
            }
        }
        // where the mark is in the hall frame (4 pole pairs), from frames with a hall lock
        if(fr.hallref == 2){
            index_elec[f.coarse].push_back(frac(fr.elec - 4 * idx.angle(fr.position)));
        }
    }

    std::printf("  %d crossings in the bench frames, %d slow enough to place: worst shaft angle at a crossing %d counts "
                "(%.2f deg), at most %d beyond half the frame step; %d/%d same-direction steps not a whole turn\n",
                crossings, placed, worst_angle, worst_angle * 360.0 / CPR, worst_excess, bad_turn_steps, turn_steps);
    expect(crossings >= 80, "at least 80 crossings in the fixture");
    expect(out_of_window == 0, "the refined mark stays in the encoder's 256-count window");
    expect(worst_excess <= 64, "shaft angle ~0 at every crossing (worst " + std::to_string(worst_excess) + " counts beyond half the step)");
    expect(bad_turn_steps == 0, "same-direction crossings are whole turns apart");

    // the mark's hall-frame electrical angle, per power-up: the same rotor position every time
    std::vector<double> medians;
    std::printf("  mark in the hall frame per power-up (electrical degrees):");
    for(auto& [coarse, v] : index_elec){
        if(v.size() < 50) continue;
        std::sort(v.begin(), v.end());
        double m = v[v.size() / 2] * 360.0;
        medians.push_back(m);
        std::printf(" %.1f", m);
    }
    std::printf("\n");
    std::sort(medians.begin(), medians.end());
    double centre = medians[medians.size() / 2];
    int agree = 0;
    for(double m : medians) agree += std::fabs(m - centre) <= 4.0;
    expect(medians.size() >= 7, "at least 7 power-ups with a hall lock near a crossing");
    // one power-up of the 2026-09-26 runs had its hall lock ~10 degrees out; the rest agree to a few degrees
    expect(agree >= int(medians.size()) - 1, "all but one power-up put the mark within 4 electrical degrees of the median");
}

// a synthetic encoder: the mark at rotor position MARK, single_turn = rotor - zero; count +-1 at each crossing
struct SimEncoder {
    static constexpr int64_t MARK = 50000;
    int64_t zero;
    int64_t rotor;
    bool found = false;
    uint32_t coarse = 0;
    int32_t count = 0;
    int64_t tracker = 0;
    bool primed = false;
    uint32_t last_st = 0;

    void power_up(int64_t z, int64_t r){ zero = z; rotor = r; found = false; coarse = 0; count = 0; primed = false; }

    void move(int64_t to_rotor, yaskawa_index& idx, int64_t step){
        while(rotor != to_rotor){
            int64_t d = to_rotor - rotor;
            int64_t s = d > step ? step : d < -step ? -step : d;
            int64_t before = rotor;
            rotor += s;
            // crossings of MARK + k * CPR between before and rotor
            auto k_of = [](int64_t x){ return int64_t(std::floor(double(x - MARK) / CPR)); };
            int64_t kb = k_of(before), ka = k_of(rotor);
            if(ka != kb){
                count += ka > kb ? 1 : -1;
                if(!found){
                    found = true;
                    int64_t st_mark = ((MARK - zero) % CPR + CPR) % CPR;
                    coarse = uint32_t(st_mark >> 8);
                }
            }
            frame(idx);
        }
    }

    void frame(yaskawa_index& idx){
        uint32_t st = uint32_t(((rotor - zero) % CPR + CPR) % CPR);
        if(!primed){ tracker = st; primed = true; }
        else{
            int32_t d = int32_t(st) - int32_t(last_st);
            if(d > CPR / 2) d -= CPR;
            if(d < -CPR / 2) d += CPR;
            tracker += d;
        }
        last_st = st;
        uint32_t raw[3] = {found ? 0u : 1u << 6, 0, (coarse << (80 - 64)) | (uint32_t(count & 0x7F) << (89 - 64))};
        idx.update(st, tracker, raw);
    }
};

void test_synthetic(){
    yaskawa_index idx;
    SimEncoder e;

    // power-ups with different zeros: the angle from the index is the rotor's angle from the mark every time
    double worst = 0;
    for(int p = 0; p < 20; p++){
        e.power_up(1000 + p * 7919, 20000 + p * 3001);
        e.frame(idx);
        expect(!idx.is_found(), "not found at power-up");
        e.move(e.rotor + CPR + 5000, idx, 37 + p);      // forward across the mark
        e.move(e.rotor - 2 * CPR, idx, 41 + p);         // back across it twice
        e.move(e.rotor + CPR / 3, idx, 29);
        expect(idx.is_found(), "found after crossing");
        double truth = frac(double(e.rotor - SimEncoder::MARK) / CPR);
        worst = std::max(worst, std::fabs(wrap_turn(idx.angle(e.tracker) - truth)));
    }
    std::printf("  20 synthetic power-ups: shaft angle from the index within %.1f counts of the rotor's\n", worst * CPR);
    expect(worst * CPR <= 40, "angle from the index is the rotor angle");

    // the controller starts while the encoder is already indexed: found on the first frame, to +-128 counts
    yaskawa_index late;
    late.update(1000, 1000, (const uint32_t[3]){0, 0, (200u << (80 - 64)) | (5u << (89 - 64))});
    expect(late.is_found() && late.index_single_turn() == 200 * 256 + 128, "already indexed: the middle of the window");
    expect(std::fabs(late.turns(200 * 256 + 128 + CPR / 4) - 0.25) < 1e-9, "turns from the mark");

    // a power-up (not_found again) forgets the mark
    late.update(1000, 1000, (const uint32_t[3]){1u << 6, 0, 0});
    expect(!late.is_found() && late.turns(12345) == 0.0, "power-up forgets the mark");

    // a count step against the direction the shaft moved, or two at once, is not used
    yaskawa_index odd;
    odd.update(51000, 51000, (const uint32_t[3]){0, 0, (200u << (80 - 64)) | (5u << (89 - 64))});
    odd.update(51100, 51100, (const uint32_t[3]){0, 0, (200u << (80 - 64)) | (4u << (89 - 64))});
    odd.update(51200, 51200, (const uint32_t[3]){0, 0, (200u << (80 - 64)) | (6u << (89 - 64))});
    expect(odd.crossings() == 0, "inconsistent count steps are ignored");
}

}  // namespace

int main(){
    const std::pair<const char*, void (*)()> tests[] = {
        {"decode", test_decode},
        {"bench frames", test_bench_frames},
        {"synthetic power-ups", test_synthetic},
    };
    for(const auto& [name, test] : tests){
        int before = failures;
        test();
        std::printf("%s %s\n", failures == before ? "  PASS" : "  FAIL", name);
    }
    std::printf("%s\n", failures ? (std::to_string(failures) + " failed").c_str() : "all passed");
    return failures ? 1 : 0;
}
