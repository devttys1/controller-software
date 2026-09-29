// Host test for telemetry_ring.h, run by run_host_tests.sh.
//
// Checks the shared memory layout the monitor web server (em_monitor.py) reads: header fields,
// schema, sample slots and wrap-around, and that closing a ring only removes its own file.
//
// sources:

#include <cstdio>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "telemetry_ring.h"

namespace {

int failures = 0;

void expect(bool ok, const std::string& what){
    if(!ok){
        std::printf("  FAIL %s\n", what.c_str());
        failures++;
    }
}

const std::string NAME = "em_telemetry_ring_test_" + std::to_string(getpid());

// a reader's copy of /dev/shm/<name>, as em_monitor.py sees it
struct Reader {
    uint8_t* mem = nullptr;
    size_t size = 0;
    ino_t inode = 0;

    bool open(const std::string& name){
        int fd = shm_open(("/" + name).c_str(), O_RDONLY, 0);
        if(fd < 0) return false;
        struct stat st;
        fstat(fd, &st);
        size = size_t(st.st_size);
        inode = st.st_ino;
        void* p = mmap(nullptr, size, PROT_READ, MAP_SHARED, fd, 0);
        ::close(fd);
        mem = p == MAP_FAILED ? nullptr : static_cast<uint8_t*>(p);
        return mem != nullptr;
    }
    ~Reader(){ if(mem) munmap(mem, size); }

    template <typename T> T at(size_t offset) const { T v; std::memcpy(&v, mem + offset, sizeof v); return v; }
};

bool exists(const std::string& name, ino_t* inode = nullptr){
    int fd = shm_open(("/" + name).c_str(), O_RDONLY, 0);
    if(fd < 0) return false;
    struct stat st;
    fstat(fd, &st);
    if(inode) *inode = st.st_ino;
    ::close(fd);
    return true;
}

struct Sample {
    uint64_t n;
    double x;
};

void test_layout(){
    telemetry_ring ring;
    const std::string schema = R"({"fields":[{"name":"n","type":"Q","offset":0},{"name":"x","type":"d","offset":8}]})";
    expect(ring.open(NAME, sizeof(Sample), 5, schema), "ring opens");
    ring.set_update_frequency(1000);

    Reader r;
    expect(r.open(NAME), "reader opens the ring");
    expect(r.size == telemetry_ring::header_size + 5 * sizeof(Sample), "size is header + capacity samples");
    expect(std::memcmp(r.mem, "EMTR", 4) == 0, "magic");
    expect(r.at<uint32_t>(4) == 1 && r.at<uint32_t>(8) == sizeof(Sample) && r.at<uint32_t>(12) == 5, "version, sample size, capacity");
    expect(r.at<uint32_t>(24) == 1000, "update frequency");
    expect(std::string(reinterpret_cast<const char*>(r.mem + telemetry_ring::schema_offset)) == schema, "schema");
    expect(r.at<uint64_t>(16) == 0, "nothing written yet");

    for(uint64_t n = 0; n < 12; n++){
        Sample s{n, n * 0.5};
        ring.push(&s);
    }
    expect(r.at<uint64_t>(16) == 12, "write count after 12 samples");
    bool slots_ok = true;
    for(uint64_t n = 12 - 5; n < 12; n++){
        size_t offset = telemetry_ring::header_size + (n % 5) * sizeof(Sample);
        slots_ok &= r.at<uint64_t>(offset) == n && r.at<double>(offset + 8) == n * 0.5;
    }
    expect(slots_ok, "sample n is in slot n % capacity after wrapping");

    expect(!ring.open(NAME, sizeof(Sample), 5, std::string(5000, 'x')), "a schema larger than the header is refused");
    expect(!exists(NAME), "a refused open leaves no ring behind");
}

void test_replace_and_close(){
    const std::string schema = R"({"fields":[]})";
    auto* first = new telemetry_ring();
    expect(first->open(NAME, 8, 4, schema), "first ring");
    ino_t first_inode = 0, second_inode = 0;
    exists(NAME, &first_inode);

    // a second ring under the same name (the controller restarted, or two drivers in a test) replaces it
    telemetry_ring second;
    expect(second.open(NAME, 8, 4, schema), "second ring");
    exists(NAME, &second_inode);
    expect(second_inode != first_inode, "the second ring is a new file, so readers can tell");

    delete first;
    expect(exists(NAME), "closing the replaced ring leaves the newer one");
    second.close();
    expect(!exists(NAME), "closing a ring removes its file");
}

}  // namespace

int main(){
    const std::pair<const char*, void (*)()> tests[] = {
        {"layout and wrap-around", test_layout},
        {"replace and close", test_replace_and_close},
    };
    for(const auto& [name, test] : tests){
        int before = failures;
        test();
        std::printf("%s %s\n", failures == before ? "  PASS" : "  FAIL", name);
    }
    shm_unlink(("/" + NAME).c_str());
    std::printf("%s\n", failures ? (std::to_string(failures) + " failed").c_str() : "all passed");
    return failures ? 1 : 0;
}
