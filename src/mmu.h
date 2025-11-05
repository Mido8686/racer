#pragma once
#include <cstdint>
#include <optional>
#include <array>

/*
  Minimal MMU / TLB representation.

  Usage:
    MMU mmu(memory, cp0); // or pass references as needed
    auto phys = mmu.translate(vaddr, accessSize, isWrite);
    if (!phys) -> TLB miss / protection exception

  This implementation is intentionally simple: fixed-size TLB with wired entries,
  basic page-mask/ASID not yet handled. Extend as needed for real SGI Octane behavior.
*/

struct TLBEntry {
    bool valid = false;
    uint32_t vpn = 0;      // virtual page number (high bits of vaddr)
    uint32_t pfn = 0;      // physical frame number (high bits of paddr)
    uint32_t asid = 0;     // address space id
    bool global = false;
    bool dirty = true;
    bool validP = true;    // PFN valid
    uint32_t pageMask = 0x00000000; // small pages only for now
};

class Memory; // forward

class MMU {
public:
    MMU(Memory *mem, class CP0 *cp0, unsigned tlbSize = 64);

    // Translate a virtual address into physical address.
    // Returns std::optional<phys_addr>. If empty, caller should raise a TLB miss/invalid exception.
    std::optional<uint64_t> translate(uint64_t vaddr, unsigned accessSize, bool isWrite);

    // Insert / update TLB entry (software-managed)
    void insertTLBEntry(unsigned index, const TLBEntry &e);

    // Simple utilities
    void flushAll();

private:
    Memory *memory;
    CP0 *cp0;
    std::vector<TLBEntry> tlb;
    unsigned tlbSize;
};
