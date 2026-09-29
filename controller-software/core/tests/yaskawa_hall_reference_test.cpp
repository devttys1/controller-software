// Host test for yaskawa_hall_reference.h, run by run_host_tests.sh.
//
// Does the published angle end up tied to the ROTOR, whatever the encoder's power-up offset?
// A port of test_hallref.c in the yaskawa-ffboard project: the motor is simulated with the hall
// map measured there (uneven sectors, and rising and falling crossings ~76 counts apart), and a
// random single_turn zero per "power-up". Added here: measuring the edge table of an encoder
// whose disk does not match the configured table, and rejecting unusable tables.
//
// sources:

#include <cstdio>
#include <cstdlib>
#include <string>

#include "yaskawa_hall_reference.h"

namespace {

using hall_ref = yaskawa_hall_reference;
constexpr int32_t RES = hall_ref::counts_per_rev;
constexpr int32_t ELEC = RES / 4;
constexpr int32_t HYST = 38;

// the simulated motor's hall map, from its own edge table: sector i (hall seq[i]) lies from edge i
// to edge i+1, edges in rotor counts from the 2|3 edge; crossings rise HYST late and fall HYST early
const uint8_t seq[6] = {3, 1, 5, 4, 6, 2};
const hall_ref::config_t default_config;
int32_t true_edges[6];
int sim_dir = 1;    // direction of the last motion: picks rising or falling crossings

int failures = 0;

void expect(bool ok, const std::string& what){
    if(!ok){
        std::printf("  FAIL %s\n", what.c_str());
        failures++;
    }
}

int32_t wrap(int32_t x, int32_t m){
    x %= m;
    if(x > m / 2) x -= m;
    else if(x <= -m / 2) x += m;
    return x;
}

int32_t mod(int64_t x, int32_t m){
    x %= m;
    return int32_t(x < 0 ? x + m : x);
}

void set_motor(const std::array<int32_t, 6>& edges){
    for(int i = 0; i < 6; i++) true_edges[i] = edges[i];
}

uint8_t hall_at(int64_t rotor){
    int32_t e = mod(rotor - (sim_dir >= 0 ? HYST : -HYST), ELEC);
    for(int i = 5; i >= 0; i--){
        if(e >= true_edges[i]) return seq[i];
    }
    return seq[5];
}

uint32_t rng = 12345;
uint32_t rnd(){
    rng = rng * 1103515245u + 12345u;
    return rng >> 8;
}

// one simulated power-up: the encoder reports rotor - zero. Returns the final rotor-referenced error.
int32_t run(hall_ref& r, int32_t zero, int64_t rotor, int32_t speed, int32_t travel,
            int32_t* worst_jump, int32_t* first_err){
    r.reset();
    int64_t prev_rotor = rotor;
    int32_t prev_pub = 0;
    int32_t moved = 0;
    uint32_t st = 0;
    *worst_jump = 0;
    for(int n = 0; n < 200000; n++){
        if(n >= 50 && moved < travel){
            rotor += speed;
            moved += std::abs(speed);
            sim_dir = speed > 0 ? 1 : -1;
        }
        st = uint32_t(mod(rotor - zero, RES));
        int32_t pub = int32_t(r.update(st, hall_at(rotor)));
        if(n == 0){
            *first_err = wrap(pub - mod(rotor, ELEC), ELEC);
        }
        else{
            int32_t j = std::abs(wrap((pub - prev_pub) - int32_t(rotor - prev_rotor), RES));
            if(j > *worst_jump) *worst_jump = j;
        }
        prev_pub = pub;
        prev_rotor = rotor;
        if(moved >= travel && n > 50 + 5000) break;
    }
    expect(std::abs(wrap(int32_t(r.electrical_angle() * ELEC + 0.5) - mod(int64_t(st) + r.get_applied(), ELEC), ELEC)) <= 1,
           "electrical_angle is the published angle within an electrical revolution");
    return wrap(int32_t(st) + r.get_applied() - mod(rotor, ELEC), ELEC);
}

void test_power_ups(){
    set_motor(default_config.edges);
    hall_ref r;
    int32_t worst_final = 0, worst_first = 0, worst_jump = 0, not_locked = 0;
    for(int t = 0; t < 2000; t++){
        int32_t zero = int32_t(rnd() % RES);
        int64_t rotor = rnd() % RES;
        int32_t speed = 1 + int32_t(rnd() % hall_ref::max_step);
        if(rnd() & 1) speed = -speed;
        int32_t jump, first;
        // a quarter turn plus a bit crosses every edge
        int32_t err = run(r, zero, rotor, speed, ELEC + ELEC / 6, &jump, &first);
        not_locked += r.get_state() != hall_ref::LOCKED;
        worst_final = std::max(worst_final, std::abs(err));
        worst_first = std::max(worst_first, std::abs(first));
        worst_jump = std::max(worst_jump, jump);
    }
    std::printf("  2000 random power-ups, speeds 1..%d counts/frame both ways: locked error %d counts (%.2f elec deg), "
                "first frame %d (%.1f elec deg), published jump %d beyond the shaft's motion\n",
                hall_ref::max_step, worst_final, worst_final * 360.0 / ELEC, worst_first, worst_first * 360.0 / ELEC, worst_jump);
    expect(not_locked == 0, std::to_string(not_locked) + " power-ups never locked");
    // midpoint of a step no bigger than max_step, plus the hysteresis the table's midpoints cannot know the sign of
    expect(worst_final <= hall_ref::max_step / 2 + HYST + 1, "locked error " + std::to_string(worst_final));
    // half the widest sector plus the widest centre offset
    expect(worst_first <= 3300, "first-frame error " + std::to_string(worst_first));
    expect(worst_jump <= hall_ref::slew, "published angle jumped " + std::to_string(worst_jump));

    // too fast to trust an edge: must not lock, and must not move the estimate
    int32_t jump, first;
    run(r, 777, 0, hall_ref::max_step + 1, 2 * ELEC, &jump, &first);
    expect(r.get_state() == hall_ref::SECTOR, "locked from edges it cannot place");
}

void test_dither_and_invalid(){
    set_motor(default_config.edges);
    hall_ref r;
    int32_t zero = 54321;
    int64_t rotor = 0;
    for(int n = 0; n < 4000; n++){
        // wide enough to cross the reference edge's hysteresis band both ways
        rotor = 3 * ELEC + ((n / 7) & 1 ? 60 : -60);
        sim_dir = (n / 7) & 1 ? 1 : -1;
        r.update(uint32_t(mod(rotor - zero, RES)), hall_at(rotor));
    }
    int32_t err = wrap(mod(rotor - zero, RES) + r.get_applied() - mod(rotor, ELEC), ELEC);
    expect(r.get_state() == hall_ref::LOCKED && std::abs(err) <= HYST + 10 && r.deviation() <= 2 * HYST + 10,
           "dither on the reference edge: error " + std::to_string(err) + " deviation " + std::to_string(r.deviation()));

    hall_ref bad;
    bad.update(1000, 0);
    expect(bad.get_state() == hall_ref::NONE, "hall 0 started a reference");
    bad.update(1000, 7);
    expect(bad.get_state() == hall_ref::NONE && bad.edges_seen() == 0, "hall 7 made an edge");
}

// sweep a quarter turn and a bit back and forth at varying speeds
void sweep(hall_ref& r, int32_t zero, int64_t& rotor, int sweeps){
    for(int s = 0; s < sweeps; s++){
        int dir = s & 1 ? -1 : 1;
        int32_t speed = 20 + int32_t(rnd() % 300);
        sim_dir = dir;
        for(int32_t moved = 0; moved < ELEC + ELEC / 4; moved += speed){
            rotor += dir * speed;
            r.update(uint32_t(mod(rotor - zero, RES)), hall_at(rotor));
        }
    }
}

void test_edge_measurement(){
    // another encoder: its edges up to 200 counts away from the configured (default) table
    const std::array<int32_t, 6> other{0, 5384, 10960, 16282, 21680, 27050};
    set_motor(other);

    hall_ref r;
    int32_t zero = int32_t(rnd() % RES);
    int64_t rotor = rnd() % RES;
    std::array<int32_t, 6> measured;
    expect(!r.measured_edges(&measured), "no table before any edges");
    sweep(r, zero, rotor, 100);
    uint32_t crossings = 0;
    expect(r.measured_edges(&measured, &crossings), "table measured after 100 sweeps");
    int32_t worst = 0;
    for(int k = 0; k < 6; k++) worst = std::max(worst, std::abs(measured[k] - other[k]));
    std::printf("  another encoder's edges: deviation %u with the default table, measured table within %d counts "
                "(fewest crossings per direction %u)\n", r.deviation(), worst, crossings);
    expect(r.deviation() > 150, "a table from another encoder shows as deviation (" + std::to_string(r.deviation()) + ")");
    // each crossing is placed within half a frame step (up to 160 counts here), so the mean of ~50 is good to
    // ~10-20 counts: well inside the +-60 counts the bench measurement repeats to
    expect(worst <= 30, "measured edges within 30 counts, worst " + std::to_string(worst));
    expect(crossings >= 40, "crossings counted per direction");

    // with the measured table: a fresh power-up locks where the rotor is, and later edges agree
    hall_ref::config_t c;
    c.edges = measured;
    expect(r.configure(c), "measured table accepted");
    int32_t jump, first, worst_err = 0;
    uint32_t worst_dev = 0;
    for(int t = 0; t < 200; t++){
        int32_t speed = (1 + int32_t(rnd() % 300)) * (rnd() & 1 ? 1 : -1);
        worst_err = std::max(worst_err, std::abs(run(r, int32_t(rnd() % RES), rnd() % RES, speed, 2 * ELEC, &jump, &first)));
        worst_dev = std::max(worst_dev, r.deviation());
    }
    std::printf("  with the measured table: locked error %d counts, deviation %u\n", worst_err, worst_dev);
    expect(worst_err <= hall_ref::max_step / 2 + HYST + 12, "locked error with the measured table " + std::to_string(worst_err));
    expect(worst_dev <= 300, "deviation with the measured table " + std::to_string(worst_dev));
}

void test_configure(){
    hall_ref r;
    auto rejects = [&](hall_ref::config_t c, const std::string& what){
        expect(!r.configure(c), "accepted " + what);
    };
    hall_ref::config_t c;
    c.pole_pairs = 0;
    rejects(c, "0 pole pairs");
    c.pole_pairs = 3;
    rejects(c, "3 pole pairs (not a divisor of 131072)");
    c = {};
    c.sequence = {3, 1, 5, 4, 6, 3};
    rejects(c, "a repeated hall state");
    c.sequence = {3, 1, 5, 4, 6, 7};
    rejects(c, "hall state 7");
    c = {};
    c.edges[0] = 10;
    rejects(c, "edge 0 not at 0");
    c = {};
    c.edges[3] = c.edges[2];
    rejects(c, "edges not increasing");
    c = {};
    c.edges[5] = ELEC;
    rejects(c, "an edge beyond an electrical revolution");
    c = {};
    c.pole_pairs = 2;
    expect(r.configure(c) && r.electrical_counts() == RES / 2, "2 pole pairs: half a turn per electrical revolution");
}

}  // namespace

int main(){
    const std::pair<const char*, void (*)()> tests[] = {
        {"power-ups lock to the rotor", test_power_ups},
        {"dither and invalid hall states", test_dither_and_invalid},
        {"edge table measurement", test_edge_measurement},
        {"configuration checks", test_configure},
    };
    for(const auto& [name, test] : tests){
        int before = failures;
        test();
        std::printf("%s %s\n", failures == before ? "  PASS" : "  FAIL", name);
    }
    std::printf("%s\n", failures ? (std::to_string(failures) + " failed").c_str() : "all passed");
    return failures ? 1 : 0;
}
