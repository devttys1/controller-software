#pragma once

#include <cstdint>

// Running position for Yaskawa 17-bit incremental encoders (UTSIH-B17CK).
// Kept free of controller dependencies so it can be tested on the host (core/tests).
//
// The encoder's 3 bit revolution counter (rot3, payload bits 77:80) steps exactly
// when single_turn wraps, so (rot3, single_turn) is the shaft position modulo 8
// revolutions. Tracking the shortest path of that combined value stays correct for
// up to 4 revolutions between frames (about 80 lost frames at 3000 rpm with a 1 kHz update).
//
// This replaces the approach of the reference implementation (yask_track_update() in
// the yaskawa-incremental-encoder project), which tracks single_turn alone and falls
// back on the 4 bit mt4 counter near half a revolution. mt4 steps near 290 degrees
// rather than at the wrap, and on thinned-out captures that fallback lost or invented
// whole revolutions where rot3 tracking stayed exact (see core/tests).
class yaskawa_incremental_tracker {
    public:
        static constexpr int32_t counts_per_rev = 1 << 17;
        static constexpr int32_t combined_range = counts_per_rev * 8;   // rot3 wraps every 8 revolutions

        // fold one CRC-valid frame into the position (counts)
        // the first frame sets the origin so position == single_turn, i.e. the fraction of a turn is the shaft angle
        int64_t update(uint32_t single_turn, uint8_t rot3){
            int32_t combined = (int32_t(rot3 & 0x7) << 17) | int32_t(single_turn & (counts_per_rev - 1));

            if(!primed){
                position = combined & (counts_per_rev - 1);
                last_combined = combined;
                primed = true;
                return position;
            }

            int32_t delta = combined - last_combined;
            if(delta > combined_range / 2) delta -= combined_range;
            if(delta < -combined_range / 2) delta += combined_range;

            position += delta;
            last_combined = combined;
            return position;
        }

        void reset(){
            primed = false;
            position = 0;
        }

        int64_t get_position() const { return position; }
        bool is_primed() const { return primed; }

    private:
        int64_t position = 0;
        int32_t last_combined = 0;
        bool primed = false;
};
