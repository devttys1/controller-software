// Host test for the spindle_orient node, run by run_host_tests.sh: created by name through the node factory,
// configured from JSON, wired to outputs of the types the yaskawa_encoders driver publishes, and run against
// a simulated spindle. The orientation logic itself is tested in spindle_orient_test.cpp.
//
// sources: ../controller/nodes/generic_nodes/src/spindle_orient.cpp ../controller/nodes/src/base_node.cpp ../controller/nodes/src/node_io.cpp

#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>

#include "spindle_orient.h"

namespace {

int failures = 0;

void expect(bool ok, const std::string& what){
    if(!ok){
        std::printf("  FAIL %s\n", what.c_str());
        failures++;
    }
}

template <typename T>
T out(std::shared_ptr<base_node>& n, const std::string& name){
    output* o = nullptr;
    if(n->get_output_object(name, o) != 0 || o == nullptr){
        throw std::runtime_error("no output " + name);
    }
    return *reinterpret_cast<T*>(o->data_pointer);
}

void test_node(){
    auto* old = std::cout.rdbuf(nullptr);
    auto node = Node_Factory::create_shared("spindle_orient");
    std::cout.rdbuf(old);

    json cfg = {{"angle", 90.0}, {"mode", 0}, {"speed_loop_lag_s", 0.02}, {"angle_offset_deg", 10.0}};
    expect(node->configure_settings(&cfg) == 0, "configures");
    json bad = {{"max_rpm", -1.0}};
    auto* err = std::cerr.rdbuf(nullptr);
    expect(Node_Factory::create_shared("spindle_orient")->configure_settings(&bad) != 0, "a negative max_rpm is refused");
    std::cerr.rdbuf(err);

    // what the driver publishes (yaskawa_encoders:0.indexed, index_turns, total_turns, velocity_rpm) and an enable
    bool enable = false, indexed = true;
    double index_turns = 0.2, turns = 0.0, velocity = 0.0;
    int src_exec = 0;
    output o_enable(io_type::BOOL, &enable, &src_exec), o_indexed(io_type::BOOL, &indexed, &src_exec);
    output o_index_turns(io_type::DOUBLE, &index_turns, &src_exec), o_turns(io_type::DOUBLE, &turns, &src_exec);
    output o_velocity(io_type::DOUBLE, &velocity, &src_exec);
    err = std::cerr.rdbuf(nullptr);
    bool wired = node->connect_input("enable", &o_enable) == 0 && node->connect_input("indexed", &o_indexed) == 0
              && node->connect_input("index_turns", &o_index_turns) == 0 && node->connect_input("turns", &o_turns) == 0
              && node->connect_input("velocity", &o_velocity) == 0;
    float wrong = 0;
    output o_float(io_type::FLOAT, &wrong, &src_exec);
    bool mismatch_refused = node->connect_input("angle", &o_float) != 0;
    std::cerr.rdbuf(err);
    expect(wired, "wires to the driver's variable types");
    expect(mismatch_refused, "a float wired to angle is refused");

    // idle until enabled
    node->run();
    expect(out<double>(node, "velocity_command") == 0.0 && out<uint32_t>(node, "state") == 0, "idle while not enabled");

    // enabled: orient to the configured angle (unconnected angle and mode read the config), plus the offset
    enable = true;
    double rpm = 0.0;
    bool oriented = false;
    for(int k = 0; k < 3000 && !oriented; k++){
        node->run();
        double cmd = out<double>(node, "velocity_command");
        rpm += (cmd - rpm) * (0.001 / 0.02);
        index_turns += rpm / 60.0 * 0.001;
        turns += rpm / 60.0 * 0.001;
        velocity = rpm;
        oriented = out<bool>(node, "is_oriented");
    }
    double angle = (index_turns - std::floor(index_turns)) * 360.0;
    std::printf("  node oriented %d at %.3f deg (config angle 90 + offset 10), state %u\n", oriented, angle, out<uint32_t>(node, "state"));
    expect(oriented && std::fabs(angle - 100.0) <= 0.1 && out<uint32_t>(node, "state") == 4, "orients to angle + angle_offset_deg");
    expect(!out<bool>(node, "fault"), "no fault");

    enable = false;
    node->run();
    expect(out<double>(node, "velocity_command") == 0.0 && !out<bool>(node, "is_oriented"), "disable releases");
}

}  // namespace

int main(){
    int before = failures;
    test_node();
    std::printf("%s spindle_orient node\n", failures == before ? "  PASS" : "  FAIL");
    std::printf("%s\n", failures ? (std::to_string(failures) + " failed").c_str() : "all passed");
    return failures ? 1 : 0;
}
