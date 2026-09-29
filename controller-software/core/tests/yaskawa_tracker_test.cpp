// Host test for yaskawa_incremental_tracker.h, run by run_host_tests.sh.
//
// Replays real frame sequences (data/yaskawa_tracker_sequences.csv, built by
// make_yaskawa_tracker_fixtures.py) and checks the tracked position against the
// true position, including thinned-out sequences with several revolutions between
// frames. At full rate it must also agree with the reference implementation.

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "yaskawa_incremental_tracker.h"

namespace {

struct Row {
    uint32_t single_turn;
    uint8_t rot3;
    int64_t reference;
    int64_t truth;
};

int failures = 0;

void expect(bool ok, const std::string& what){
    if(!ok){
        std::cout << "  FAIL " << what << std::endl;
        failures++;
    }
}

void test_basics(){
    constexpr int32_t rev = yaskawa_incremental_tracker::counts_per_rev;
    yaskawa_incremental_tracker t;

    expect(t.update(1000, 5) == 1000, "first frame sets the origin to single_turn");
    expect(t.update(1500, 5) == 1500, "small forward step");
    expect(t.update(rev - 100, 4) == -100, "backward across the wrap (rot3 steps down)");
    expect(t.update(200, 5) == 200, "forward across the wrap (rot3 steps up)");
    expect(t.update(200, 0) == 200 + 3 * rev, "3 revolutions between frames");
    expect(t.update(200, 5) == 200, "3 revolutions back, across the rot3 wrap");

    t.reset();
    expect(!t.is_primed(), "reset clears the origin");
    expect(t.update(42, 1) == 42, "origin after reset");
}

void test_sequences(const std::string& path){
    std::ifstream file(path);
    if(!file.is_open()){
        std::cout << "  FAIL cannot open " << path << std::endl;
        failures++;
        return;
    }

    // (sequence, decimation) -> rows, in file order
    std::map<std::pair<std::string, int>, std::vector<Row>> sequences;
    std::vector<std::pair<std::string, int>> order;
    std::string line;
    while(std::getline(file, line)){
        if(line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::string name, field;
        std::getline(ss, name, ',');
        std::vector<int64_t> v;
        while(std::getline(ss, field, ',')) v.push_back(std::stoll(field));
        auto key = std::make_pair(name, int(v[0]));
        if(!sequences.count(key)) order.push_back(key);
        sequences[key].push_back({uint32_t(v[1]), uint8_t(v[2]), v[4], v[5]});
    }

    for(const auto& key : order){
        const auto& rows = sequences[key];
        yaskawa_incremental_tracker t;
        int64_t origin = 0;
        int truth_errors = 0, reference_errors = 0;
        int64_t last = 0;
        for(size_t i = 0; i < rows.size(); i++){
            int64_t pos = t.update(rows[i].single_turn, rows[i].rot3);
            if(i == 0) origin = pos;
            last = pos - origin;
            truth_errors += last != rows[i].truth;
            reference_errors += last != rows[i].reference;
        }
        // full-rate sequences: every step is well under half a revolution, so the reference is exact too
        bool full_rate = key.second <= 6;
        char summary[160];
        std::snprintf(summary, sizeof summary, "%-9s /%-3d %4zu frames  %+9.3f rev  truth mismatches %d  reference mismatches %d",
                      key.first.c_str(), key.second, rows.size(), double(last) / yaskawa_incremental_tracker::counts_per_rev,
                      truth_errors, reference_errors);
        std::cout << "  " << summary << std::endl;
        expect(truth_errors == 0, key.first + " /" + std::to_string(key.second) + " matches the true position");
        if(full_rate){
            expect(reference_errors == 0, key.first + " /" + std::to_string(key.second) + " matches the reference implementation");
        }
    }
}

}  // namespace

int main(int argc, char** argv){
    std::string data = argc > 1 ? argv[1] : "data/yaskawa_tracker_sequences.csv";

    std::cout << "yaskawa_incremental_tracker basics" << std::endl;
    test_basics();
    std::cout << "yaskawa_incremental_tracker captured sequences" << std::endl;
    test_sequences(data);

    std::cout << (failures ? std::to_string(failures) + " failed" : "all passed") << std::endl;
    return failures ? 1 : 0;
}
