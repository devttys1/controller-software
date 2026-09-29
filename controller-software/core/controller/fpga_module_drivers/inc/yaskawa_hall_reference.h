#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

// Rotor-referenced angle for Yaskawa 17-bit incremental encoders (UTSIH-B17CK), from the
// encoder's hall (commutation) track. Kept free of controller dependencies so it can be
// tested on the host (core/tests).
//
// Port of yaskawa_hallref.c in the yaskawa-ffboard project (Firmware/Targets/F767ZI/Yaskawa),
// where the behaviour was measured on the bench (2026-09-24):
//   - single_turn's zero moves every time the encoder powers up (by ~170 electrical degrees
//     between two power-ups), so a commutation offset or spindle angle stored against it is
//     only valid until the encoder next loses power;
//   - within one power-up the six hall edges sit at fixed single_turn values, repeatable to
//     a few counts. The hall track is on the disk, so it IS fixed to the rotor.
//
// So the published angle is single_turn plus a correction that puts the hall 2|3 edge at
// electrical zero:
//   SECTOR  first frame: assume the shaft is mid-sector, at most ~33 electrical degrees out.
//   LOCKED  first clean crossing of any hall edge: exact (within ~3 electrical degrees) and
//           final until reset. Each edge's real position comes from a measured table, since
//           the sectors are uneven (637..730 of a nominal 683 angle14 counts).
// A clean edge is one where the shaft moved no more than max_step between the frames either
// side of it; the edge is placed at their midpoint. Edges are 60 electrical degrees apart
// (15 mechanical with 4 pole pairs), so any motion of more than that locks.
//
// The correction is slewed (slew counts per frame), never stepped, so whatever follows the
// angle never sees a jump. The hall track repeats every electrical revolution, so the angle
// is tied to the rotor only modulo 1/pole_pairs of a turn: whole-turn orientation is still
// lost when the encoder powers down.
//
// The edge table belongs to one encoder's disk. measured_edges() builds it from the edges
// crossed after the lock, and deviation() (the worst disagreement of a later edge with the
// lock, counts) is the check that the configured table fits: up to ~300 is normal,
// thousands means the table is from another encoder.
class yaskawa_hall_reference {
    public:
        static constexpr int32_t counts_per_rev = 1 << 17;
        static constexpr int32_t max_step = 512;   // 2.8 electrical degrees of edge placement error, ~230 rpm at 1 kHz
        static constexpr int32_t slew = 8;         // a full sector-to-lock correction takes ~0.4 s at 1 kHz

        enum state_t : uint32_t {
            NONE = 0,       // no valid hall reading yet
            SECTOR = 1,     // estimated from the sector, within ~33 electrical degrees
            LOCKED = 2,     // from a clean edge, final
            INDEXED = 3,    // from the encoder's index mark (lock_to_index), which is placed ~10x more precisely
        };

        struct config_t {
            uint32_t pole_pairs = 4;
            // hall states (U=bit0, V=bit1, W=bit2) in the order they occur as single_turn increases
            std::array<uint8_t, 6> sequence{3, 1, 5, 4, 6, 2};
            // where edge k (between sequence[k-1] and sequence[k]) sits, in single_turn counts from
            // edge 0, which is electrical zero. Measured on the yaskawa-ffboard encoder: the midpoints
            // of the rising and falling crossings over ~60 turns, repeatable to ~+-60 counts.
            std::array<int32_t, 6> edges{0, 5234, 11080, 16192, 21880, 26990};
        };

        // false (and nothing changes) if the table is not usable
        bool configure(const config_t& c){
            if(c.pole_pairs == 0 || counts_per_rev % int32_t(c.pole_pairs) != 0){
                return false;
            }
            int32_t e = counts_per_rev / int32_t(c.pole_pairs);
            uint8_t seen = 0;
            for(auto h : c.sequence){
                if(h < 1 || h > 6) return false;
                seen |= uint8_t(1u << h);
            }
            if(seen != 0x7E) return false;  // each of 1..6 once
            if(c.edges[0] != 0) return false;
            for(int k = 1; k < 6; k++){
                if(c.edges[k] <= c.edges[k - 1] || c.edges[k] >= e) return false;
            }
            cfg = c;
            elec = e;
            reset();
            return true;
        }

        void reset(){
            state = NONE;
            prev_sector = -1;
            prev_st = 0;
            target = 0;
            applied = 0;
            edge_count = 0;
            dev_max = 0;
            last_out = 0;
            stats = {};
        }

        // fold in one new, CRC-valid frame; returns single_turn with the correction applied
        uint32_t update(uint32_t single_turn, uint8_t hall){
            int32_t st = int32_t(single_turn & (counts_per_rev - 1));
            int sec = sector_of(hall);

            if(sec >= 0){
                if(state == NONE){
                    // applied at once: there is no earlier angle for this one to jump from
                    target = nearest_correction(sector_centre(sec), st);
                    applied = target;
                    state = SECTOR;
                }
                else if(prev_sector >= 0 && sec != prev_sector){
                    edge(prev_sector, sec, prev_st, st);
                }
                prev_sector = sec;
                prev_st = st;
            }
            else{
                prev_sector = -1;   // never measure an edge across an invalid reading (hall 0 or 7)
            }

            int32_t d = target - applied;
            if(d > slew) d = slew;
            if(d < -slew) d = -slew;
            applied += d;

            last_out = mod(int64_t(st) + applied, counts_per_rev);
            return uint32_t(last_out);
        }

        // Take the angle from the encoder's index instead of the hall edges: single_turn index_st is at hall-frame
        // electrical angle index_elec (electrical turns, a property of the motor and encoder: where the mark sits
        // relative to the hall 2|3 edge). Call on every frame once the index is found; as its position is refined
        // the correction moves with it, slewed like any other. Later hall edges are still checked against it
        // (deviation()).
        void lock_to_index(int32_t index_st, double index_elec){
            int32_t pos = int32_t(std::lround((index_elec - std::floor(index_elec)) * elec)) % elec;
            target = nearest_correction(pos, index_st & (counts_per_rev - 1));
            if(state == NONE){
                applied = target;   // nothing has been published yet, so nothing to slew from
            }
            state = INDEXED;
        }

        // where the index mark sits in the hall frame, from the current lock: the value lock_to_index wants. Only
        // meaningful while the lock came from hall edges (LOCKED)
        double index_electrical_angle(int32_t index_st) const {
            return double(mod(int64_t(index_st) + target, elec)) / double(elec);
        }

        state_t get_state() const { return state; }
        uint32_t edges_seen() const { return edge_count; }
        uint32_t deviation() const { return dev_max; }
        int32_t electrical_counts() const { return elec; }

        // correction being applied (unwrapped): tracker position + applied is fixed to the rotor modulo an
        // electrical revolution, as long as the tracker started from single_turn
        int32_t get_applied() const { return applied; }

        // electrical angle of the last frame, in electrical turns [0, 1), zero at the hall 2|3 edge
        double electrical_angle() const {
            return double(mod(last_out, elec)) / double(elec);
        }

        // Edge table measured from the clean edges crossed since the lock, in the form config_t::edges takes
        // (edge 0 = 0). Each edge is the midpoint of its mean rising and falling crossing when both have been
        // seen. min_crossings is the fewest crossings of any edge in either direction; false until every
        // edge has been crossed at least once.
        bool measured_edges(std::array<int32_t, 6>* out, uint32_t* min_crossings = nullptr) const {
            std::array<double, 6> at{};
            uint32_t fewest = UINT32_MAX;
            for(int k = 0; k < 6; k++){
                const auto& s = stats[k];
                if(s.count[0] + s.count[1] == 0) return false;
                double mean_dev;
                if(s.count[0] && s.count[1]){
                    mean_dev = (double(s.sum[0]) / s.count[0] + double(s.sum[1]) / s.count[1]) / 2.0;
                }
                else{
                    mean_dev = double(s.sum[0] + s.sum[1]) / double(s.count[0] + s.count[1]);
                }
                at[k] = cfg.edges[k] + mean_dev;
                fewest = std::min(fewest, std::min(s.count[0], s.count[1]));
            }
            for(int k = 0; k < 6; k++){
                double rel = at[k] - at[0];
                int64_t r = int64_t(rel < 0 ? rel - 0.5 : rel + 0.5);
                (*out)[k] = mod(r, elec);
            }
            if(min_crossings != nullptr) *min_crossings = fewest;
            return true;
        }

    private:
        struct edge_stats {
            int64_t sum[2] = {0, 0};        // deviation from the table, counts; [0] rising, [1] falling single_turn
            uint32_t count[2] = {0, 0};
        };

        config_t cfg;
        int32_t elec = counts_per_rev / 4;

        state_t state = NONE;
        int prev_sector = -1;
        int32_t prev_st = 0;
        // both unwrapped: reducing them would let applied wrap mid-slew, which would be a jump
        int32_t target = 0;
        int32_t applied = 0;
        uint32_t edge_count = 0;
        uint32_t dev_max = 0;
        int32_t last_out = 0;
        std::array<edge_stats, 6> stats{};

        static int32_t mod(int64_t x, int32_t m){
            x %= m;
            return int32_t(x < 0 ? x + m : x);
        }

        // x reduced to (-m/2, m/2]
        static int32_t wrap_signed(int32_t x, int32_t m){
            x %= m;
            if(x > m / 2) x -= m;
            else if(x <= -m / 2) x += m;
            return x;
        }

        int sector_of(uint8_t hall) const {
            for(int i = 0; i < 6; i++){
                if(cfg.sequence[i] == hall) return i;
            }
            return -1;
        }

        int32_t sector_centre(int j) const {
            int32_t next = j == 5 ? elec : cfg.edges[j + 1];
            return (cfg.edges[j] + next) / 2;
        }

        // the correction that puts single_turn st at hall-frame angle pos, nearest to what is applied
        int32_t nearest_correction(int32_t pos, int32_t st) const {
            int32_t want = pos - st % elec;
            return applied + wrap_signed(want - applied, elec);
        }

        void edge(int from, int to, int32_t from_st, int32_t to_st){
            int32_t step = wrap_signed(to_st - from_st, counts_per_rev);
            if(step > max_step || step < -max_step){
                return;     // moved too far to say where the edge was
            }

            int b;
            if(to == (from + 1) % 6) b = to;
            else if(from == (to + 1) % 6) b = from;
            else return;    // skipped a sector: not a single edge

            int32_t at = mod(int64_t(from_st) + step / 2, counts_per_rev);
            edge_count++;

            if(state == NONE || state == SECTOR){
                target = nearest_correction(cfg.edges[b], at);
                state = LOCKED;
                return;
            }

            // final already: how well does this edge agree with the lock
            int32_t dev = wrap_signed(mod(int64_t(at) + target, elec) - cfg.edges[b], elec);
            uint32_t mag = uint32_t(dev < 0 ? -dev : dev);
            if(mag > dev_max) dev_max = mag;
            if(step != 0){
                int dir = step > 0 ? 0 : 1;
                stats[b].sum[dir] += dev;
                stats[b].count[dir]++;
            }
        }
};
