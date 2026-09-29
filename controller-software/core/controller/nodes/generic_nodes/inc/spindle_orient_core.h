#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

// Spindle orientation (M19): bring the spindle to an angle measured from the encoder's index mark and hold
// it there. Kept free of controller dependencies so it can be tested on the host (core/tests); the
// spindle_orient node wraps it.
//
// Follows LinuxCNC's M19 R<angle> P<mode>: angle in degrees, mode 0 the shorter way, 1 forward (increasing
// position) only, 2 reverse only; is_oriented once there, fault if it cannot get there. Dropping enable
// releases the spindle and clears a fault.
//
// Output is a speed command (rpm) for the spindle's speed loop: a reference motion's speed plus kp times how
// far the spindle is behind it. The reference moves to the goal as fast as max_rpm and accel allow and stops
// exactly on it; it starts from the spindle's own position and speed, so an M19 while the spindle runs
// carries on the way it is turning and brakes, instead of stepping the command. Because the spindle follows
// the reference, the speed loop's lag becomes a small following error that kp closes, rather than an
// overshoot; with speed_loop_lag_s set to the speed loop's time constant, the reference's acceleration is fed
// forward and the spindle tracks it without reversing at the end of the move. The goal is fixed in continuous turns from the index (the yaskawa_encoders driver's
// index_turns) when the move starts, so it never jumps at the 0/360 degree wrap.
//
// If the encoder has not found its index since power-up, the spindle first turns at search_rpm (forward, or
// reverse for mode 2) until it does; not finding it within search_turns_max turns is a fault. So is losing
// the index while orienting (the encoder powered down), or not being oriented within timeout_s.
class spindle_orient_core {
    public:
        enum state_t : uint32_t {
            IDLE = 0,       // enable low: no command
            SEARCH = 1,     // turning to find the index
            MOVE = 2,       // moving to the angle
            SETTLE = 3,     // within tolerance, waiting for settle_s
            ORIENTED = 4,   // there, holding
            FAULT = 5,      // could not orient; holds zero speed until enable drops
        };

        struct config_t {
            double angle_offset_deg = 0.0;  // where orient angle 0 is, degrees past the index mark
            double search_rpm = 30.0;
            double max_rpm = 120.0;
            double accel_rpm_per_s = 600.0;
            double kp_per_s = 20.0;         // speed (turns/s) per turn behind the reference
            double speed_loop_lag_s = 0.0;  // the speed loop's time constant, for acceleration feedforward
            double tolerance_deg = 0.1;
            double settle_s = 0.05;
            double still_rpm = 1.0;         // the spindle counts as stopped below this
            double timeout_s = 10.0;
            double search_turns_max = 1.25;
            double cycle_s = 0.001;
        };

        struct inputs_t {
            bool enable = false;
            double angle_deg = 0.0;
            uint32_t mode = 0;
            bool indexed = false;
            double index_turns = 0.0;       // continuous turns from the index mark (valid when indexed)
            double turns = 0.0;             // continuous turns since startup (for the search limit)
            double velocity_rpm = 0.0;
        };

        struct outputs_t {
            double velocity_command_rpm = 0.0;
            double error_deg = 0.0;         // goal - position while moving or holding
            bool oriented = false;
            bool fault = false;
            state_t state = IDLE;
        };

        config_t cfg;

        const outputs_t& update(const inputs_t& in){
            if(!in.enable){
                release();
                prev_enable = false;
                return out;
            }
            bool start = !prev_enable;
            prev_enable = true;

            if(start){
                elapsed = 0.0;
                out.fault = false;
                // take over from wherever the spindle is and however fast it turns
                v_ref = in.velocity_rpm / 60.0;
                p_ref = in.indexed ? in.index_turns : 0.0;
                if(in.mode > 2){
                    fail();
                }
                else if(in.indexed){
                    plan(in);
                }
                else{
                    out.state = SEARCH;
                    search_from = in.turns;
                    search_dir = in.mode == 2 ? -1.0 : 1.0;
                }
            }
            elapsed += cfg.cycle_s;

            switch(out.state){
                case SEARCH:
                    if(in.indexed){
                        p_ref = in.index_turns;     // positions from the index exist from here on
                        plan(in);
                        break;
                    }
                    if(std::fabs(in.turns - search_from) > cfg.search_turns_max){
                        fail();
                        break;
                    }
                    ramp_to(search_dir * cfg.search_rpm / 60.0);
                    v_cmd = v_ref + cfg.speed_loop_lag_s * a_ref;
                    break;

                case MOVE:
                case SETTLE:
                case ORIENTED: {
                    if(!in.indexed){
                        fail();     // the encoder lost its index: it powered down, so the angle is gone
                        break;
                    }
                    if(in.angle_deg != planned_angle || in.mode != planned_mode){
                        plan(in);   // a new angle while orienting
                    }
                    // the reference: as fast as allowed towards the goal, braking to land on it exactly
                    double d = goal - p_ref;
                    double ad = std::fabs(d);
                    double v = std::min({max_rps(), std::sqrt(2.0 * accel_rps2() * ad), ad / cfg.cycle_s});
                    ramp_to(d >= 0 ? v : -v);
                    p_ref += v_ref * cfg.cycle_s;
                    if(std::fabs(goal - p_ref) < 1e-9 || (d > 0) != (goal - p_ref > 0)){
                        if(std::fabs(v_ref) <= accel_rps2() * cfg.cycle_s){
                            p_ref = goal;       // arrived
                            v_ref = 0.0;
                            a_ref = 0.0;
                        }
                    }
                    // follow it
                    v_cmd = v_ref + cfg.speed_loop_lag_s * a_ref + cfg.kp_per_s * (p_ref - in.index_turns);
                    v_cmd = std::clamp(v_cmd, -1.5 * max_rps(), 1.5 * max_rps());

                    double e = goal - in.index_turns;
                    out.error_deg = e * 360.0;

                    bool within = std::fabs(out.error_deg) <= cfg.tolerance_deg;
                    bool still = std::fabs(in.velocity_rpm) <= cfg.still_rpm;
                    if(out.state == ORIENTED){
                        if(std::fabs(out.error_deg) > 3.0 * cfg.tolerance_deg){
                            out.state = MOVE;   // pushed off: go back (and report not oriented meanwhile)
                            out.oriented = false;
                        }
                    }
                    else if(within && still){
                        if(out.state != SETTLE){
                            out.state = SETTLE;
                            settled = 0.0;
                        }
                        settled += cfg.cycle_s;
                        if(settled >= cfg.settle_s){
                            out.state = ORIENTED;
                            out.oriented = true;
                        }
                    }
                    else{
                        out.state = MOVE;
                    }
                    if(!out.oriented && elapsed > cfg.timeout_s){
                        fail();
                    }
                    break;
                }

                case FAULT:
                    ramp_to(0.0);
                    v_cmd = v_ref + cfg.speed_loop_lag_s * a_ref;
                    break;

                case IDLE:
                    break;
            }
            out.velocity_command_rpm = v_cmd * 60.0;
            return out;
        }

        const outputs_t& outputs() const { return out; }
        double goal_turns() const { return goal; }

    private:
        outputs_t out;
        bool prev_enable = false;
        double v_cmd = 0.0;         // turns per second: the output
        double v_ref = 0.0;         // the reference motion, turns per second and turns from the index
        double p_ref = 0.0;
        double a_ref = 0.0;         // the reference's acceleration this cycle, turns/s^2
        double goal = 0.0;          // index_turns to reach
        double planned_angle = 0.0;
        uint32_t planned_mode = 0;
        double elapsed = 0.0;
        double settled = 0.0;
        double search_from = 0.0;
        double search_dir = 1.0;

        double max_rps() const { return cfg.max_rpm / 60.0; }
        double accel_rps2() const { return cfg.accel_rpm_per_s / 60.0; }

        void ramp_to(double target){
            double step = accel_rps2() * cfg.cycle_s;
            double dv = std::clamp(target - v_ref, -step, step);
            v_ref += dv;
            a_ref = dv / cfg.cycle_s;
        }

        void release(){
            out = outputs_t{};
            v_cmd = 0.0;
            v_ref = 0.0;
            a_ref = 0.0;
        }

        void fail(){
            out.state = FAULT;
            out.fault = true;
            out.oriented = false;
        }

        void plan(const inputs_t& in){
            planned_angle = in.angle_deg;
            planned_mode = in.mode;
            double target = frac((in.angle_deg + cfg.angle_offset_deg) / 360.0);
            double here = frac(p_ref);
            double d = target - here;
            // the way still turning: going on to a later pass of the angle if it cannot stop before this one
            double stop = v_ref * v_ref / (2.0 * accel_rps2());
            if(in.mode == 1 || (in.mode == 0 && v_ref > cfg.still_rpm / 60.0)){
                d -= std::floor(d);                         // forward: [0, 1)
                if(v_ref > 0) while(d < stop) d += 1.0;
            }
            else if(in.mode == 2 || (in.mode == 0 && v_ref < -cfg.still_rpm / 60.0)){
                d -= std::ceil(d);                          // reverse: (-1, 0]
                if(v_ref < 0) while(-d < stop) d -= 1.0;
            }
            else{
                d -= std::floor(d + 0.5);                   // the shorter way: [-0.5, 0.5)
            }
            goal = p_ref + d;
            out.state = MOVE;
            out.oriented = false;
        }

        static double frac(double x){ return x - std::floor(x); }
};
