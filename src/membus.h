// speedracer/src/membus.h
// Minimal MemoryBus abstraction: ROM + MMIO device registration.
//
// Usage:
//  - Create MemoryBus
//  - register ROM region with add_rom(base, buffer)
//  - register MMIO handlers (lambda or object) with add_mmio(base, size, read/write callbacks)
//  - CPU will call read32/write32 on the bus

#pragma once
#include <cstdint>
#include <vector>
#include <functional>
#include <map>
#include <cstring>

struct MMIOHandler {
    // 32-bit read and write callbacks
    std::function<uint32_t(uint32_t offset)> read32;
    std::function<void(uint32_t offset, uint32_t value)> write32;
};

class MemoryBus {
public:
    MemoryBus() = default;

    // Add a ROM mapping: base virtual address -> buffer (copy)
    // read-only: true
    void add_rom(uint32_t base, const std::vector<uint8_t> &buf) {
        Region r;
        r.base = base;
        r.size = static_cast<uint32_t>(buf.size());
        r.data = buf;
        r.readonly = true;
        regions.push_back(r);
    }

    // Add a RAM mapping (read/write)
    void add_ram(uint32_t base, uint32_t size) {
        Region r;
        r.base = base;
        r.size = size;
        r.data.resize(size);
        r.readonly = false;
        regions.push_back(r);
    }

    // Register an MMIO handler at base..base+size-1
    void add_mmio(uint32_t base, uint32_t size, MMIOHandler handler) {
        mmio[base] = { size, handler };
    }

    // Read 32-bit word (big-endian) at virtual address addr
    uint32_t read32(uint32_t addr) {
        // check ROM/RAM regions
        for (const auto &r : regions) {
            if (addr >= r.base && addr + 4 <= r.base + r.size) {
                uint32_t off = addr - r.base;
                uint32_t w = (uint32_t(r.data[off]) << 24) |
                             (uint32_t(r.data[off+1]) << 16) |
                             (uint32_t(r.data[off+2]) << 8) |
                             (uint32_t(r.data[off+3]));
                return w;
            }
        }
        // check MMIO
        for (const auto &kv : mmio) {
            uint32_t mbase = kv.first;
            uint32_t msize = kv.second.first;
            if (addr >= mbase && addr + 4 <= mbase + msize) {
                uint32_t offset = addr - mbase;
                if (kv.second.second.read32) return kv.second.second.read32(offset);
                else return 0xffffffff;
            }
        }
        // unmapped -> return all-ones
        return 0xffffffffu;
    }

    // Write 32-bit word (big-endian) to virtual address addr
    void write32(uint32_t addr, uint32_t value) {
        // check RAM regions only; ROM is readonly
        for (auto &r : regions) {
            if (!r.readonly && addr >= r.base && addr + 4 <= r.base + r.size) {
                uint32_t off = addr - r.base;
                r.data[off]     = static_cast<uint8_t>((value >> 24) & 0xff);
                r.data[off + 1] = static_cast<uint8_t>((value >> 16) & 0xff);
                r.data[off + 2] = static_cast<uint8_t>((value >> 8) & 0xff);
                r.data[off + 3] = static_cast<uint8_t>((value) & 0xff);
                return;
            }
        }
        // check MMIO
        for (auto &kv : mmio) {
            uint32_t mbase = kv.first;
            uint32_t msize = kv.second.first;
            if (addr >= mbase && addr + 4 <= mbase + msize) {
                uint32_t offset = addr - mbase;
                if (kv.second.second.write32) kv.second.second.write32(offset, value);
                return;
            }
        }
        // otherwise ignore writes to unmapped
    }

private:
    struct Region {
        uint32_t base = 0;
        uint32_t size = 0;
        std::vector<uint8_t> data;
        bool readonly = true;
    };
    std::vector<Region> regions;

    // mmio map: base -> (size, handler)
    std::map<uint32_t, std::pair<uint32_t, MMIOHandler>> mmio;
};
