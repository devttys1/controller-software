// Host test for spindle_orient_core.h (M19), run by run_host_tests.sh.
//
// A simulated spindle: its speed loop follows the command with a 20 ms lag, limited to 3000 rpm/s; the
// encoder reports continuous turns, and turns from its index mark once the shaft has passed it (as the
// yaskawa_encoders driver does), quantised to 17 bits.
//
// sources:

#include <cmath>
#include <cstdio>
#include <string>

#include "spindle_orient_core.h"

namespace {

int failures = 0;

void expect(bool ok, const std::string& what){
    if(!ok){
        std::printf("  FAIL %s\n", what.c_str());
        failures++;
    }
}

constexpr double DT = 0.001;
constexpr double CPR = 131072.0;

struct Spindle {
    double rotor = 0.0;         // turns
    double rpm = 0.0;
    double mark = 0.37;         // the index mark's rotor position (mod 1)
    bool indexed = false;
    bool has_mark = true;       // false: an encoder that never finds its index
    double start = 0.0;

    void step(double command_rpm){
        double target = rpm + (command_rpm - rpm) * (DT / 0.02);
        double max_step = 3000.0 * DT;
        rpm += std::clamp(target - rpm, -max_step, max_step);
        double before = rotor;
        rotor += rpm / 60.0 * DT;
        if(has_mark && std::floor(before - mark) != std::floor(rotor - mark)){
            indexed = true;
        }
    }

    spindle_orient_core::inputs_t inputs(bool enable, double angle, uint32_t mode) const {
        spindle_orient_core::inputs_t in;
        in.enable = enable;
        in.angle_deg = angle;
        in.mode = mode;
        in.indexed = indexed;
        in.index_turns = indexed ? std::round((rotor - mark) * CPR) / CPR : 0.0;
        in.turns = std::round((rotor - start) * CPR) / CPR;
        in.velocity_rpm = rpm;
        return in;
    }

    // the shaft's angle from the mark, degrees [0, 360)
    double angle() const { double a = rotor - mark; return (a - std::floor(a)) * 360.0; }
};

double angle_error(double a, double b){
    double d = std::fmod(a - b + 540.0, 360.0) - 180.0;
    return d;
}

struct Run {
    bool oriented = false, fault = false;
    double time = 0, max_rpm = 0, min_rpm = 0, max_accel = 0, final_error = 0;
    double overshoot_deg = 0;       // furthest past the goal
    double back_deg = 0;            // furthest travelled away from the goal's direction, from the start
    double search_min_rpm = 0, search_max_rpm = 0;
};

// the tests' spindle has a 20 ms speed loop; the orienter is told so unless a test says otherwise
spindle_orient_core make(double lag = 0.02){
    spindle_orient_core o;
    o.cfg.speed_loop_lag_s = lag;
    return o;
}

Run orient(spindle_orient_core& o, Spindle& s, double angle, uint32_t mode, double seconds = 6.0){
    Run r;
    double prev_rpm = s.rpm;
    double first_error = 0;
    double start = s.rotor;
    double dir = 0;
    for(int k = 0; k < int(seconds / DT); k++){
        const auto& out = o.update(s.inputs(true, angle, mode));
        if(out.state == spindle_orient_core::SEARCH){
            r.search_min_rpm = std::min(r.search_min_rpm, s.rpm);
            r.search_max_rpm = std::max(r.search_max_rpm, s.rpm);
        }
        else if(dir == 0 && out.error_deg != 0){
            dir = out.error_deg > 0 ? 1.0 : -1.0;     // the way to the goal, once there is one
            start = s.rotor;
        }
        if(dir != 0){
            r.back_deg = std::max(r.back_deg, -dir * (s.rotor - start) * 360.0);
        }
        s.step(out.velocity_command_rpm);
        r.max_rpm = std::max(r.max_rpm, s.rpm);
        r.min_rpm = std::min(r.min_rpm, s.rpm);
        if(k > 0) r.max_accel = std::max(r.max_accel, std::fabs(s.rpm - prev_rpm) / DT);   // the spindle's own
        prev_rpm = s.rpm;
        if(k == 0) first_error = out.error_deg;
        // past the goal, the wrong side of where it started from
        if(out.state != spindle_orient_core::SEARCH && first_error != 0 && out.error_deg * first_error < 0){
            r.overshoot_deg = std::max(r.overshoot_deg, std::fabs(out.error_deg));
        }
        if(out.oriented && !r.oriented){
            r.oriented = true;
            r.time = k * DT;
        }
        if(out.fault){
            r.fault = true;
            break;
        }
    }
    r.final_error = angle_error(s.angle(), angle + o.cfg.angle_offset_deg);
    return r;
}

void test_shortest_way(){
    double worst = 0, slowest = 0, worst_over = 0;
    int wrong_way = 0, not_oriented = 0;
    for(int i = 0; i < 36; i++){
        for(double target : {0.0, 90.0, 179.0, 181.0, 359.5}){
            spindle_orient_core o = make();
            Spindle s;
            s.rotor = s.mark + i * 10.0 / 360.0 + 0.3;     // already indexed, somewhere around the turn
            s.indexed = true;
            double start_angle = s.angle();
            Run r = orient(o, s, target, 0);
            not_oriented += !r.oriented;
            worst = std::max(worst, std::fabs(r.final_error));
            slowest = std::max(slowest, r.time);
            double shortest = angle_error(target, start_angle);
            // the shorter way: set off towards the goal that way, and do not run far past it
            if(std::fabs(shortest) > 1.0){
                bool forward = r.max_rpm > -r.min_rpm;
                wrong_way += forward != (shortest > 0) || r.back_deg > 0.5;
            }
            worst_over = std::max(worst_over, r.overshoot_deg);
        }
    }
    std::printf("  180 orients the shorter way: worst final error %.3f deg, slowest %.2f s, worst overshoot %.3f deg\n", worst, slowest, worst_over);
    expect(not_oriented == 0, std::to_string(not_oriented) + " never oriented");
    expect(worst <= 0.1, "within the 0.1 degree tolerance");
    expect(wrong_way == 0, std::to_string(wrong_way) + " went the long way");
    expect(worst_over <= 1.0, "overshoot within a degree (" + std::to_string(worst_over) + ")");
}

void test_modes_and_limits(){
    for(uint32_t mode : {1u, 2u}){
        int wrong = 0;
        double worst = 0, fastest = 0, accel = 0, back = 0;
        for(int i = 0; i < 24; i++){
            spindle_orient_core o = make();
            Spindle s;
            s.rotor = s.mark + i * 15.0 / 360.0 + 0.01;
            s.indexed = true;
            Run r = orient(o, s, 45.0, mode);
            worst = std::max(worst, std::fabs(r.final_error));
            fastest = std::max(fastest, std::max(r.max_rpm, -r.min_rpm));
            accel = std::max(accel, r.max_accel);
            // turning the wrong way is not allowed; settling back a fraction of a degree past the goal is
            wrong += mode == 1 ? r.max_rpm < -r.min_rpm : r.max_rpm > -r.min_rpm;
            back = std::max(back, r.back_deg);
            expect(r.oriented, "mode " + std::to_string(mode) + " oriented from " + std::to_string(i * 15));
        }
        std::printf("  mode %u (%s only): worst error %.3f deg, top speed %.1f rpm, spindle accel %.0f rpm/s, back %.3f deg\n",
                    mode, mode == 1 ? "forward" : "reverse", worst, fastest, accel, back);
        expect(wrong == 0 && back <= 0.5, "mode " + std::to_string(mode) + " never turned the wrong way");
        expect(fastest <= 120.0 * 1.02, "top speed within max_rpm");
        // the feedforward is exact for a continuous lag; on this 1 ms model it accelerates ~10% harder
        expect(accel <= 600.0 * 1.15, "spindle acceleration within accel");
    }
}

void test_search(){
    for(uint32_t mode : {0u, 2u}){
        spindle_orient_core o = make();
        Spindle s;
        s.rotor = s.mark + 0.5;     // half a turn from the mark, not indexed yet
        Run r = orient(o, s, 200.0, mode);
        std::printf("  not indexed, mode %u: oriented %d after %.2f s, error %.3f deg\n", mode, r.oriented, r.time, r.final_error);
        expect(r.oriented && std::fabs(r.final_error) <= 0.1, "searches for the index, then orients");
        expect(mode == 2 ? r.search_max_rpm <= 0.0 && r.search_min_rpm < -20.0 : r.search_min_rpm >= 0.0 && r.search_max_rpm > 20.0,
               "searches in the mode's direction");
    }

    spindle_orient_core o;
    Spindle s;
    s.has_mark = false;
    Run r = orient(o, s, 10.0, 0);
    const auto& out = o.outputs();
    expect(r.fault && !out.oriented, "no index within search_turns_max is a fault");
    expect(std::fabs(s.rotor) < 1.4, "stops searching after ~1.25 turns (" + std::to_string(s.rotor) + ")");
    // the fault holds zero speed until enable drops
    for(int k = 0; k < 500; k++) s.step(o.update(s.inputs(true, 10.0, 0)).velocity_command_rpm);
    expect(std::fabs(o.outputs().velocity_command_rpm) < 1e-9 && o.outputs().fault, "fault ramps to zero and holds");
    o.update(s.inputs(false, 10.0, 0));
    expect(!o.outputs().fault && o.outputs().state == spindle_orient_core::IDLE, "dropping enable clears the fault");
}

void test_while_running(){
    // M19 while the spindle runs forward at 300 rpm: the shorter way must not mean reversing
    spindle_orient_core o = make();
    Spindle s;
    s.indexed = true;
    s.rotor = s.mark + 0.1;
    s.rpm = 300.0;
    for(int k = 0; k < 200; k++) s.step(300.0);
    // the node starts from a stopped command; the speed loop follows it down
    Run r = orient(o, s, 10.0, 0);
    std::printf("  from 300 rpm forward: oriented %d after %.2f s, back %.3f deg, overshoot %.3f deg\n", r.oriented, r.time, r.back_deg, r.overshoot_deg);
    expect(r.oriented && std::fabs(r.final_error) <= 0.1, "orients from running");
    expect(r.back_deg <= 0.5 && r.overshoot_deg <= 1.0, "carries on forward and brakes onto the angle");
}

void test_lag_not_matched(){
    // speed_loop_lag_s left at 0 on the same spindle: still orients, with a small overshoot
    double worst_over = 0, worst_rev = 0;
    bool all = true;
    for(int i = 0; i < 12; i++){
        spindle_orient_core o = make(0.0);
        Spindle s;
        s.indexed = true;
        s.rotor = s.mark + i * 30.0 / 360.0 + 0.05;
        Run r = orient(o, s, 200.0, 0);
        all &= r.oriented && std::fabs(r.final_error) <= 0.1;
        worst_over = std::max(worst_over, r.overshoot_deg);
        worst_rev = std::max(worst_rev, std::min(-r.min_rpm, r.max_rpm));
    }
    std::printf("  without feedforward: worst overshoot %.2f deg, worst reversal %.1f rpm\n", worst_over, worst_rev);
    expect(all, "orients without feedforward too");
    // this is why speed_loop_lag_s is worth setting
    expect(worst_over < 5.0, "overshoot a few degrees at most without feedforward");
}

void test_disturbance_and_loss(){
    spindle_orient_core o = make();
    Spindle s;
    s.indexed = true;
    s.rotor = s.mark + 0.2;
    Run r = orient(o, s, 300.0, 0);
    expect(r.oriented, "oriented");
    s.rotor += 3.0 / 360.0;     // knocked 3 degrees
    bool left = false, back = false;
    for(int k = 0; k < 2000; k++){
        const auto& out = o.update(s.inputs(true, 300.0, 0));
        s.step(out.velocity_command_rpm);
        left |= !out.oriented;
        if(left && out.oriented){ back = true; break; }
    }
    expect(left && back && std::fabs(angle_error(s.angle(), 300.0)) <= 0.1, "pushed off, reports it, and returns");

    // a new angle while holding
    Run r2 = orient(o, s, 30.0, 0, 3.0);
    expect(std::fabs(r2.final_error) <= 0.1 && o.outputs().oriented, "a new angle while holding moves there");

    // the encoder loses its index (powered down) mid-move
    spindle_orient_core o2;
    Spindle s2;
    s2.indexed = true;
    s2.rotor = s2.mark;
    for(int k = 0; k < 100; k++) s2.step(o2.update(s2.inputs(true, 180.0, 0)).velocity_command_rpm);
    s2.indexed = false;
    o2.update(s2.inputs(true, 180.0, 0));
    expect(o2.outputs().fault, "losing the index while orienting is a fault");

    // an invalid mode
    spindle_orient_core o3;
    o3.update(s2.inputs(true, 0.0, 7));
    expect(o3.outputs().fault, "mode > 2 is a fault");
}

void test_offset_and_timeout(){
    spindle_orient_core o;
    o.cfg.angle_offset_deg = 37.5;
    Spindle s;
    s.indexed = true;
    s.rotor = s.mark + 0.6;
    Run r = orient(o, s, 0.0, 0);
    expect(r.oriented && std::fabs(angle_error(s.angle(), 37.5)) <= 0.1, "angle 0 is angle_offset_deg past the mark");

    // a spindle that cannot move (the drive is off): timeout
    spindle_orient_core t;
    t.cfg.timeout_s = 0.5;
    Spindle stuck;
    stuck.indexed = true;
    stuck.rotor = stuck.mark;
    bool fault = false;
    for(int k = 0; k < 1000 && !fault; k++) fault = t.update(stuck.inputs(true, 90.0, 0)).fault;
    expect(fault, "not oriented within timeout_s is a fault");
}

}  // namespace

int main(){
    const std::pair<const char*, void (*)()> tests[] = {
        {"the shorter way", test_shortest_way},
        {"forward and reverse only, limits", test_modes_and_limits},
        {"searching for the index", test_search},
        {"orient while running", test_while_running},
        {"speed loop lag not matched", test_lag_not_matched},
        {"disturbance, new angle, index lost", test_disturbance_and_loss},
        {"angle offset and timeout", test_offset_and_timeout},
    };
    for(const auto& [name, test] : tests){
        int before = failures;
        test();
        std::printf("%s %s\n", failures == before ? "  PASS" : "  FAIL", name);
    }
    std::printf("%s\n", failures ? (std::to_string(failures) + " failed").c_str() : "all passed");
    return failures ? 1 : 0;
}
