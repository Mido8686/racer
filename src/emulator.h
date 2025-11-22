// -----------------------------------------------------------
// emulator.h
// -----------------------------------------------------------
// Racer SGI Octane1 Emulator
// Master system controller for CPU/MMU/MEM/Devices
// -----------------------------------------------------------

#pragma once
#include <cstdint>
#include <string>

class CPU;
class MMU;
class Memory;
class CP0;

class Emulator {
public:
    Emulator();
    ~Emulator();

    bool init(uint64_t ram_size);
    bool load_prom(const std::string& path);

    void run(uint64_t cycles);

    // Physical read/write used by CPU/MMU
    uint32_t sys_read32(uint64_t phys);
    void     sys_write32(uint64_t phys, uint32_t val);

    // For Part 2 MMIO
    uint32_t mmio_read32(uint64_t phys);
    void     mmio_write32(uint64_t phys, uint32_t val);

    Memory& memory_ref() { return *mem; }
    CPU&    cpu_ref()    { return *cpu; }

private:
    CPU*    cpu  = nullptr;
    MMU*    mmu  = nullptr;
    CP0*    cp0  = nullptr;
    Memory* mem  = nullptr;
};
