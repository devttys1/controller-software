#pragma once

#include <cmath>
#include <cstdint>

// The once-per-turn index of Yaskawa 17-bit incremental encoders (UTSIH-B17CK), from the raw payload.
// Kept free of controller dependencies so it can be tested on the host (core/tests).
//
// Decoded on the bench 2026-09-26 (Documentation/yaskawa-incremental-encoder.md, "Payload map"):
//   bit 6       index_not_found  1 from power-up until the shaft first passes the index mark
//   bit 56      index_pulse      1 within ~0.3 degrees of the mark
//   bits 80:89  index_position   single_turn of the mark >> 8, latched at the first crossing
//   bits 89:96  index_count      signed 7-bit count of crossings since power-up, +1 forward, -1 back
// The mark sat at the same rotor position (119.0-121.1 electrical degrees in the hall frame) across every
// power-up tested, while single_turn's zero moved each time. So once the index is found, the angle from the
// index is the true shaft angle, which survives power cycles.
//
// The encoder only gives the mark to 256 counts (index_position). Each frame pair across which index_count
// steps places a crossing at the midpoint of the two single_turn values (if the shaft moved less than
// max_step between them); the index is the mean of the forward crossings and the mean of the backward
// crossings, averaged, which cancels the ~30 count difference between the two directions. Until a crossing
// is seen (the controller started after the encoder was already indexed) it is the middle of the 256
// count window.
class yaskawa_index {
    public:
        static constexpr int32_t counts_per_rev = 1 << 17;
        static constexpr int32_t max_step = 512;

        struct fields {
            bool not_found;
            bool pulse;
            uint32_t coarse;    // index_position, single_turn >> 8
            int32_t count;      // index_count, sign-extended
        };

        // payload words as the FPGA exposes them (raw_payload_0..2: payload bit 0 is bit 0 of word 0)
        static fields decode(const uint32_t raw[3]){
            fields f;
            f.not_found = (raw[0] >> 6) & 1u;
            f.pulse = (raw[1] >> (56 - 32)) & 1u;
            f.coarse = (raw[2] >> (80 - 64)) & 0x1FFu;
            uint32_t c = (raw[2] >> (89 - 64)) & 0x7Fu;
            f.count = c >= 64 ? int32_t(c) - 128 : int32_t(c);
            return f;
        }

        void reset(){
            found = false;
            have_prev = false;
            coarse = 0;
            offset = 128;
            anchor_turns = 0;
            for(int d = 0; d < 2; d++){
                sum[d] = 0;
                n[d] = 0;
            }
            last = {};
        }

        // fold in one new, CRC-valid frame: its single_turn, the tracker's continuous position (which must be
        // congruent to single_turn modulo a turn, as yaskawa_incremental_tracker's is) and the raw payload
        void update(uint32_t single_turn, int64_t position, const uint32_t raw[3]){
            int32_t st = int32_t(single_turn & (counts_per_rev - 1));
            fields f = decode(raw);
            last = f;

            if(f.not_found){
                if(found){
                    reset();    // the encoder powered up again: its zero has moved, start over
                    last = f;
                }
            }
            else if(!found || f.coarse != coarse){
                // newly found, or the encoder latched a different mark (a power cycle too short to see)
                found = true;
                coarse = f.coarse;
                offset = 128;
                for(int d = 0; d < 2; d++){
                    sum[d] = 0;
                    n[d] = 0;
                }
                // fix which turn of the tracker is turn 0 from the index: the one the shaft is nearest to now
                int64_t index_now = int64_t(coarse) * 256 + offset;
                anchor_turns = int64_t(std::floor(double(position - index_now) / counts_per_rev + 0.5));
            }

            if(found && have_prev && f.count != prev_count){
                crossing(prev_st, st, f.count - prev_count);
            }

            have_prev = true;
            prev_st = st;
            prev_count = f.count;
        }

        bool is_found() const { return found; }
        const fields& last_fields() const { return last; }
        uint32_t crossings() const { return uint32_t(n[0] + n[1]); }

        // the index mark in single_turn counts, 0..counts_per_rev-1
        int32_t index_single_turn() const {
            int64_t v = int64_t(coarse) * 256 + offset;
            return int32_t(((v % counts_per_rev) + counts_per_rev) % counts_per_rev);
        }

        // continuous turns from the index mark: whole turns since it was found, and the fraction is the shaft angle
        double turns(int64_t position) const {
            if(!found){
                return 0.0;
            }
            int64_t index_pos = int64_t(coarse) * 256 + offset + anchor_turns * counts_per_rev;
            return double(position - index_pos) / counts_per_rev;
        }

        // shaft angle from the index mark, turns [0, 1)
        double angle(int64_t position) const {
            double t = turns(position);
            return t - std::floor(t);
        }

    private:
        bool found = false;
        uint32_t coarse = 0;
        double offset = 128;        // the mark is coarse * 256 + offset single_turn counts
        int64_t anchor_turns = 0;
        double sum[2] = {0, 0};     // crossing positions relative to coarse * 256: [0] forward, [1] backward
        int32_t n[2] = {0, 0};
        fields last = {};

        bool have_prev = false;
        int32_t prev_st = 0;
        int32_t prev_count = 0;

        static int32_t wrap(int32_t x, int32_t m){
            x %= m;
            if(x > m / 2) x -= m;
            else if(x <= -m / 2) x += m;
            return x;
        }

        void crossing(int32_t from_st, int32_t to_st, int32_t count_step){
            int32_t step = wrap(to_st - from_st, counts_per_rev);
            int32_t c = wrap(count_step, 128);
            if(step == 0 || step > max_step || step < -max_step || (c != 1 && c != -1) || (c > 0) != (step > 0)){
                return;     // too fast to place, skipped a crossing, or the count moved against the shaft
            }
            double mid = from_st + step / 2.0;
            double rel = mid - double(coarse) * 256;
            rel -= std::floor((rel + counts_per_rev / 2.0) / counts_per_rev) * counts_per_rev;   // to (-half, half]
            if(rel < -256 || rel >= 512){
                return;     // nowhere near the mark the encoder latched
            }
            int d = step > 0 ? 0 : 1;
            sum[d] += rel;
            n[d]++;
            if(n[0] && n[1]){
                offset = (sum[0] / n[0] + sum[1] / n[1]) / 2.0;
            }
            else{
                offset = sum[d] / n[d];
            }
        }
};
