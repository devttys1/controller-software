#pragma once

#include <cmath>
#include <cstdint>

// Speed of a Yaskawa 17-bit incremental encoder (UTSIH-B17CK) from the times the encoder itself reports
// its position was reached, instead of the controller's frame times.
// Kept free of controller dependencies so it can be tested on the host (core/tests).
//
// Every frame carries ts_fast (the encoder's 6 MHz clock when it built the frame) and position_age (payload
// bits 40:56: ticks since its internal position last changed; see Documentation/yaskawa-incremental-encoder.md,
// "Payload map"). So ts_fast - position_age is when the reported position was reached, to one 6 MHz tick.
// A frame whose count changed since the previous frame is an anchor: (count, time it was reached).
//
// The speed is the count difference between the newest anchor and an older one, over the time between them:
// the older one is the oldest within window_ticks, or the first at least window_counts away, whichever comes
// first. At speed that is the previous frame; at low speed it measures the time between counts (the period
// method), so the count's truncation (about half a count, as the encoder's internal position steps twice
// per count) is spread over up to the whole window, instead of one count per frame. When no count arrives
// for longer than the last measured speed allows, the speed is capped at one count over the time since the
// last one, so it falls to zero when the shaft stops instead of holding.
//
// The encoder's clock is not the controller's (6001 ticks per controller ms on the bench): ticks_per_second()
// is learned from the frames, against the controller's update period.
class yaskawa_velocity {
    public:
        static constexpr int32_t counts_per_rev = 1 << 17;
        static constexpr double nominal_ticks_per_s = 6.0e6;

        // window_s: longest time to measure over; window_counts: stop reaching back once this many counts apart
        void configure(double window_s, int32_t counts){
            window_ticks = int64_t(window_s * nominal_ticks_per_s);
            window_counts = counts < 1 ? 1 : counts;
        }

        void reset(){
            primed = false;
            n_anchors = 0;
            head = 0;
            speed = 0.0;
            have_speed = false;
        }

        // fold in one new, CRC-valid frame: the tracker's continuous position (counts), ts_fast, position_age,
        // and the controller time since the previous new frame (seconds; 0 on the first)
        void update(int64_t position, uint16_t ts_fast, uint16_t position_age, double dt_controller){
            if(!primed){
                primed = true;
                time = ts_fast;
                last_position = position;
                last_ts = ts_fast;
                return;
            }
            int64_t dts = uint16_t(ts_fast - last_ts);
            // frames more than one ts_fast wrap (10.9 ms) apart cannot be placed in time: start over
            if(dt_controller > 0.0 && dt_controller * ticks_per_s > 60000.0){
                reset();
                update(position, ts_fast, position_age, 0.0);
                return;
            }
            time += dts;
            if(dt_controller > 0.0 && dts > 0){
                // learn the encoder's clock rate against the controller's: the ratio of all ticks to all time,
                // exact to a few ppm after 0.1 s (a rate error is a speed error); halved at 20 s to follow drift
                double rate = double(dts) / dt_controller;
                if(rate > 0.9 * nominal_ticks_per_s && rate < 1.1 * nominal_ticks_per_s){
                    rate_ticks += double(dts);
                    rate_time += dt_controller;
                    if(rate_time >= 0.1){
                        ticks_per_s = rate_ticks / rate_time;
                    }
                    if(rate_time >= 20.0){
                        rate_ticks *= 0.5;
                        rate_time *= 0.5;
                    }
                }
            }

            // the count changed since the last frame, so the position was reached after it: position_age is
            // shorter than a frame and cannot have wrapped
            if(position != last_position && position_age <= dts){
                add_anchor(position, time - position_age);
                measure();
            }
            last_position = position;
            last_ts = ts_fast;
        }

        // counts per second (0 until two anchors), capped by the time since the last count arrived
        double counts_per_second() const {
            if(!have_speed || n_anchors == 0){
                return 0.0;
            }
            double v = speed;   // counts per tick
            double since = double(time - anchor(0).t);
            if(since > 0.0 && std::fabs(v) * since > 1.0){
                v = (v > 0 ? 1.0 : -1.0) / since;   // a count is overdue: at most one count over the time since
            }
            if(since > double(window_ticks) * 4){
                v = 0.0;
            }
            return v * ticks_per_s;
        }

        double rpm() const {
            return counts_per_second() / counts_per_rev * 60.0;
        }

        double ticks_per_second() const { return ticks_per_s; }
        bool ready() const { return have_speed; }

    private:
        struct anchor_t {
            int64_t position;
            int64_t t;          // encoder ticks, continuous
        };
        static constexpr int capacity = 64;

        int64_t window_ticks = int64_t(0.02 * nominal_ticks_per_s);
        int32_t window_counts = 64;
        double ticks_per_s = nominal_ticks_per_s;
        double rate_ticks = 0.0;
        double rate_time = 0.0;

        bool primed = false;
        int64_t time = 0;
        int64_t last_position = 0;
        uint16_t last_ts = 0;

        anchor_t anchors[capacity] = {};
        int head = 0;           // next slot
        int n_anchors = 0;
        double speed = 0.0;     // counts per tick, from the anchors
        bool have_speed = false;

        const anchor_t& anchor(int back) const {    // 0 = newest
            return anchors[(head - 1 - back + 2 * capacity) % capacity];
        }

        void add_anchor(int64_t position, int64_t t){
            if(n_anchors > 0 && t <= anchor(0).t){
                t = anchor(0).t + 1;    // two steps within a tick
            }
            anchors[head] = {position, t};
            head = (head + 1) % capacity;
            if(n_anchors < capacity) n_anchors++;
        }

        void measure(){
            if(n_anchors < 2){
                return;
            }
            const anchor_t& now = anchor(0);
            int use = -1;
            for(int back = 1; back < n_anchors; back++){
                const anchor_t& a = anchor(back);
                if(now.t - a.t > window_ticks){
                    break;
                }
                use = back;
                int64_t dp = now.position - a.position;
                if(dp >= window_counts || dp <= -window_counts){
                    break;
                }
            }
            if(use < 0){
                use = 1;    // the previous anchor is already outside the window: measure over it anyway
            }
            const anchor_t& a = anchor(use);
            speed = double(now.position - a.position) / double(now.t - a.t);
            have_speed = true;
        }
};
