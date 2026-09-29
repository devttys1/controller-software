#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

// A ring of fixed-size samples in POSIX shared memory (/dev/shm/<name>), written by the realtime
// loop and read by the monitor web server (zynq_files/controller/monitor/em_monitor.py).
//
// push() is a memcpy and an atomic store: no system calls, and the mapping is locked (the
// controller runs under mlockall(MCL_FUTURE), and open() touches every page), so it cannot block.
// Readers never write to the mapping.
//
// Layout (all little endian):
//   0     char[4]   magic "EMTR"
//   4     uint32    version (1)
//   8     uint32    sample_size, bytes
//   12    uint32    capacity, samples
//   16    uint64    write_count: samples written so far; sample n is at slot n % capacity.
//                   Stored after the sample is written, so a reader that reads write_count, then
//                   the samples, then write_count again can keep every sample that is newer than
//                   (second write_count - capacity).
//   24    uint32    update_frequency, Hz (samples per second when pushed every cycle)
//   64    char[]    schema: NUL-terminated JSON, {"fields": [{"name", "type", "offset"}, ...], ...}
//                   types are the struct module's: d (double), Q (uint64), I (uint32), H (uint16), B (uint8)
//   4096  samples
class telemetry_ring {
    public:
        static constexpr uint32_t header_size = 4096;
        static constexpr uint32_t schema_offset = 64;

        telemetry_ring() = default;
        telemetry_ring(const telemetry_ring&) = delete;
        telemetry_ring& operator=(const telemetry_ring&) = delete;

        ~telemetry_ring(){
            close();
        }

        // create (or replace) /dev/shm/<name>; false if shared memory is unavailable
        bool open(const std::string& shm_name, uint32_t sample_bytes, uint32_t sample_capacity, const std::string& schema_json){
            close();
            if(schema_json.size() + 1 > header_size - schema_offset || sample_bytes == 0 || sample_capacity == 0){
                return false;
            }
            name = "/" + shm_name;
            size = size_t(header_size) + size_t(sample_bytes) * sample_capacity;

            shm_unlink(name.c_str());   // a reader of the previous run keeps its own copy until it reopens
            int fd = shm_open(name.c_str(), O_CREAT | O_RDWR, 0644);
            if(fd < 0){
                std::cerr << "telemetry_ring: shm_open " << name << " failed: " << std::strerror(errno) << std::endl;
                return false;
            }
            fchmod(fd, 0644);   // readable by the monitor, whatever the umask
            if(ftruncate(fd, off_t(size)) != 0){
                std::cerr << "telemetry_ring: ftruncate " << name << " failed: " << std::strerror(errno) << std::endl;
                ::close(fd);
                shm_unlink(name.c_str());
                return false;
            }
            struct stat st;
            inode = fstat(fd, &st) == 0 ? st.st_ino : 0;
            void* p = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            ::close(fd);
            if(p == MAP_FAILED){
                std::cerr << "telemetry_ring: mmap " << name << " failed: " << std::strerror(errno) << std::endl;
                shm_unlink(name.c_str());
                return false;
            }
            mem = static_cast<uint8_t*>(p);
            std::memset(mem, 0, size);  // also faults every page in now, not in the realtime loop

            sample_size = sample_bytes;
            capacity = sample_capacity;
            std::memcpy(mem, "EMTR", 4);
            put32(4, 1);
            put32(8, sample_size);
            put32(12, capacity);
            std::memcpy(mem + schema_offset, schema_json.c_str(), schema_json.size() + 1);
            count = 0;
            return true;
        }

        bool is_open() const { return mem != nullptr; }

        void set_update_frequency(uint32_t hz){
            if(mem != nullptr) put32(24, hz);
        }

        void push(const void* sample){
            if(mem == nullptr) return;
            std::memcpy(mem + header_size + size_t(count % capacity) * sample_size, sample, sample_size);
            count++;
            reinterpret_cast<std::atomic<uint64_t>*>(mem + 16)->store(count, std::memory_order_release);
        }

        uint64_t written() const { return count; }

        void close(){
            if(mem != nullptr){
                munmap(mem, size);
                mem = nullptr;
                // only if the name is still ours, not a newer ring's
                int fd = shm_open(name.c_str(), O_RDONLY, 0);
                if(fd >= 0){
                    struct stat st;
                    bool ours = fstat(fd, &st) == 0 && st.st_ino == inode;
                    ::close(fd);
                    if(ours) shm_unlink(name.c_str());
                }
            }
        }

    private:
        std::string name;
        uint8_t* mem = nullptr;
        size_t size = 0;
        ino_t inode = 0;
        uint32_t sample_size = 0;
        uint32_t capacity = 0;
        uint64_t count = 0;

        void put32(size_t offset, uint32_t v){
            std::memcpy(mem + offset, &v, sizeof v);
        }
};
