// Host test for yaskawa_velocity.h, run by run_host_tests.sh.
//
// A simulated encoder runs on its own 6 MHz clock, 6001 ticks per controller millisecond as on the bench,
// with an internal position twice as fine as the reported count and sensor noise that makes it flicker at
// rest, like the UTSIH-B17CK. Each 1 ms frame carries the count, ts_fast and position_age. The estimate from
// the encoder's step times is compared with the frame-based one (count change per frame), both through the
// driver's default 100 Hz low-pass, at rest, at 0.5 to 1500 rpm and through a stop; then on real standstill
// frames from the bench (data/yaskawa_index_frames.csv).
//
// sources:

#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "yaskawa_velocity.h"

namespace {

constexpr int32_t CPR = yaskawa_velocity::counts_per_rev;
constexpr double TICKS_PER_MS = 6001.0;         // encoder ticks per controller millisecond
constexpr double DT = 0.001;
const double ALPHA = 1.0 - std::exp(-2.0 * M_PI * 100.0 * DT);    // the driver's default filter
int failures = 0;

void expect(bool ok, const std::string& what){
    if(!ok){
        std::printf("  FAIL %s\n", what.c_str());
        failures++;
    }
}

struct Result {
    double rms_old, rms_new, mean_err_new;
};

// speed(t) in rpm, t in seconds; returns errors after `settle` seconds
Result simulate(const std::function<double(double)>& speed, double seconds, double settle, double noise_steps,
                yaskawa_velocity* out = nullptr){
    std::mt19937 rng(12345);
    std::normal_distribution<double> white(0.0, 1.0);
    yaskawa_velocity vel;
    vel.configure(0.02, 64);
    vel.reset();

    const double tick_s = DT / TICKS_PER_MS;    // controller seconds per encoder tick
    double x = 1000.25;                          // true position, internal steps (2 per count)
    double noise = 0.0;
    int64_t internal = int64_t(std::floor(x));
    int64_t tick = 0, last_change = 0;
    int64_t prev_count = internal >> 1;
    double old_est = 0, new_est = 0, sum_old = 0, sum_new = 0, sum_err = 0;
    int n = 0;
    int64_t frames = int64_t(seconds / DT);
    double next_frame_tick = TICKS_PER_MS;
    // noise: low-pass filtered white noise, ~3 us correlation
    const double a = std::exp(-1.0 / 18.0), b = std::sqrt(1 - a * a);

    for(int64_t f = 1; f <= frames; f++){
        while(tick < int64_t(next_frame_tick)){
            tick++;
            double t = tick * tick_s;
            x += speed(t) / 60.0 * CPR * 2 * tick_s;
            noise = a * noise + b * noise_steps * white(rng);
            int64_t now = int64_t(std::floor(x + noise));
            if(now != internal){
                internal = now;
                last_change = tick;
            }
        }
        next_frame_tick += TICKS_PER_MS;
        int64_t count = internal >> 1;
        uint16_t ts = uint16_t(tick);
        uint16_t age = uint16_t(tick - last_change);
        vel.update(count, ts, age, f == 1 ? 0.0 : DT);

        double raw_old = double(count - prev_count) / CPR / DT * 60.0;
        prev_count = count;
        old_est += ALPHA * (raw_old - old_est);
        new_est += ALPHA * (vel.rpm() - new_est);

        double t = f * DT;
        if(t >= settle){
            double truth = speed(t);
            sum_old += (old_est - truth) * (old_est - truth);
            sum_new += (new_est - truth) * (new_est - truth);
            sum_err += new_est - truth;
            n++;
        }
    }
    if(out) *out = vel;
    return {std::sqrt(sum_old / n), std::sqrt(sum_new / n), sum_err / n};
}

void test_steady_speeds(){
    yaskawa_velocity v;
    Result r = simulate([](double){ return 0.0; }, 2.0, 0.2, 0.35, &v);
    std::printf("  at rest (flickering):  rms old %.4f rpm, new %.4f rpm\n", r.rms_old, r.rms_new);
    expect(r.rms_new < r.rms_old * 0.3, "at rest the new estimate is at least 3x quieter");
    expect(std::fabs(v.ticks_per_second() / 6.001e6 - 1) < 1e-4,
           "encoder clock learned: " + std::to_string(v.ticks_per_second()));

    for(double rpm : {0.5, 5.0, 50.0, 1500.0}){
        r = simulate([rpm](double){ return rpm; }, 2.0, 0.3, 0.35);
        std::printf("  %7.1f rpm:  rms old %.4f rpm, new %.4f rpm, mean error %+.5f rpm\n",
                    rpm, r.rms_old, r.rms_new, r.mean_err_new);
        if(rpm <= 5.0){
            // a 64-count window at 5 rpm is ~6 ms, so its half-count truncation is ~0.8%
            expect(r.rms_new < r.rms_old * 0.4, "at " + std::to_string(rpm) + " rpm at least 2.5x quieter");
        }
        else{
            expect(r.rms_new <= r.rms_old * 1.05, "at " + std::to_string(rpm) + " rpm no worse");
        }
        expect(std::fabs(r.mean_err_new) < std::max(0.002 * rpm, 0.005), "mean speed right at " + std::to_string(rpm) + " rpm");
    }
}

void test_stop(){
    // 30 rpm, then decelerate to a stop at 0.5 s, then still: the estimate must fall to ~0 promptly
    auto profile = [](double t){ return t < 0.4 ? 30.0 : t < 0.5 ? 30.0 * (0.5 - t) / 0.1 : 0.0; };
    Result r = simulate(profile, 0.6, 0.55, 0.0);
    std::printf("  stop: rms after 50 ms at rest old %.4f, new %.4f rpm\n", r.rms_old, r.rms_new);
    expect(r.rms_new < 0.05, "stops within 50 ms (rms " + std::to_string(r.rms_new) + " rpm)");
}

void test_bench_standstill(){
    // the first 80 frames after each encoder power-up in the bench fixture: the shaft is still
    std::ifstream f("data/yaskawa_index_frames.csv");
    std::string line;
    std::getline(f, line);
    struct Frame { int segment; int64_t position; uint32_t raw1, raw0; };
    std::vector<Frame> frames;
    while(std::getline(f, line)){
        std::stringstream ss(line);
        std::string v[8];
        for(auto& s : v) std::getline(ss, s, ',');
        frames.push_back({std::stoi(v[0]), std::stoll(v[2]), uint32_t(std::stoul(v[4])), uint32_t(std::stoul(v[3]))});
    }
    double sum_old = 0, sum_new = 0;
    int n = 0;
    size_t i = 0;
    while(i < frames.size()){
        size_t j = i;
        while(j + 1 < frames.size() && frames[j + 1].segment == frames[i].segment) j++;
        // a still segment: the position moves no more than a few counts over it
        int64_t lo = frames[i].position, hi = lo;
        for(size_t k = i; k <= j; k++){ lo = std::min(lo, frames[k].position); hi = std::max(hi, frames[k].position); }
        if(hi - lo <= 4 && j - i >= 60){
            yaskawa_velocity vel;
            vel.reset();
            double old_est = 0, new_est = 0;
            for(size_t k = i; k <= j; k++){
                uint16_t ts = uint16_t((frames[k].raw0 >> 24) | ((frames[k].raw1 & 0xFF) << 8));
                uint16_t age = uint16_t(frames[k].raw1 >> 8);
                vel.update(frames[k].position, ts, age, k == i ? 0.0 : DT);
                if(k == i) continue;
                double raw_old = double(frames[k].position - frames[k - 1].position) / CPR / DT * 60.0;
                old_est += ALPHA * (raw_old - old_est);
                new_est += ALPHA * (vel.rpm() - new_est);
                if(k - i >= 30){
                    sum_old += old_est * old_est;
                    sum_new += new_est * new_est;
                    n++;
                }
            }
        }
        i = j + 1;
    }
    double rms_old = std::sqrt(sum_old / n), rms_new = std::sqrt(sum_new / n);
    std::printf("  bench, shaft still (%d frames): rms old %.4f rpm, new %.4f rpm\n", n, rms_old, rms_new);
    expect(n > 300, "enough still frames in the fixture");
    expect(rms_new < rms_old * 0.5, "on real frames at rest the new estimate is at least 2x quieter");
}

}  // namespace

int main(){
    const std::pair<const char*, void (*)()> tests[] = {
        {"steady speeds", test_steady_speeds},
        {"stopping", test_stop},
        {"bench frames at rest", test_bench_standstill},
    };
    for(const auto& [name, test] : tests){
        int before = failures;
        test();
        std::printf("%s %s\n", failures == before ? "  PASS" : "  FAIL", name);
    }
    std::printf("%s\n", failures ? (std::to_string(failures) + " failed").c_str() : "all passed");
    return failures ? 1 : 0;
}
