// Host test for Node_Core, run by run_host_tests.sh.
//
// sources: ../controller/nodes/src/node_core.cpp ../controller/nodes/src/node_network.cpp ../controller/nodes/src/base_node.cpp ../controller/nodes/src/node_io.cpp

#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include "node_core.h"

namespace {

int failures = 0;

void expect(bool ok, const std::string& what){
    if(!ok){
        std::cout << "  FAIL " << what << std::endl;
        failures++;
    }
}

// a node config with no networks, as used by the spindle_bench example, used to crash the first cycle
void test_empty_node_config_runs(){
    const std::string fpga_config = "controller/config/fpga_configs/example";
    const std::string file = std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp") + "/empty_node_config.json";
    std::ofstream(file) << R"({"info": {"fpga_config": ")" << fpga_config
                        << R"("}, "user_node_vars": {}, "node_var_values": {}, "networks": {}})";

    Node_Core core;
    core.set_fpga_config(fpga_config);
    expect(core.load(file) == 0, "empty node config loads");
    core.set_enable(true);
    for(int i = 0; i < 10; i++){
        core.run_update();
    }
    std::remove(file.c_str());
}

}  // namespace

int main(){
    test_empty_node_config_runs();
    std::cout << (failures ? "  FAIL" : "  PASS") << " empty node config runs" << std::endl;
    std::cout << (failures ? std::to_string(failures) + " failed" : "all passed") << std::endl;
    return failures ? 1 : 0;
}
