// -----------------------------------------------------------
// memory.h
// -----------------------------------------------------------
// Speedracer SGI Octane1 Emulator
// Simple byte-addressable RAM + ROM memory system
// Supports dynamic RAM sizing (set from TUI Settings)
// -----------------------------------------------------------

#pragma once
#include <cstdint>
#include <vector>
#include <stdexcept>
#include <cstring>
#include <string>

class Memory {
public:
    Memory();
    ~Memory();

    // Initialize RAM size (in bytes)
    void init(uint64_t size_bytes);

    // Read/write (virtual already translated to physical)
    uint8_t   read8(uint64_t phys);
    uint16_t  read16(uint64_t phys);
    uint32_t  read32(uint64_t phys);
    uint64_t  read64(uint64_t phys);

    void      write8(uint64_t phys, uint8_t v);
    void      write16(uint64_t phys, uint16_t v);
    void      write32(uint64_t phys, uint32_t v);
    void      write64(uint64_t phys, uint64_t v);

    // Load binary blob directly into RAM (PROM, ROM, etc.)
    void load_blob(uint64_t phys, const void* data, size_t size);

    // Clear region of RAM
    void clear_region(uint64_t phys, uint64_t size);

    // Query RAM size
    uint64_t size() const { return ram.size(); }

private:
    std::vector<uint8_t> ram;

    inline void check_bounds(uint64_t phys, uint64_t width) {
        if (phys + width > ram.size()) {
            throw std::out_of_range(
                "[Memory] Out-of-bounds access at 0x" + std::to_string(phys));
        }
    }
};

