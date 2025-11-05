#include "mmu.h"
#include "cp0.h"
#include "memory.h" // your Memory implementation
#include <cassert>
#include <iostream>

MMU::MMU(Memory *mem, CP0 *cp0_, unsigned tlbSize_)
  : memory(mem), cp0(cp0_), tlbSize(tlbSize_) {
    tlb.reserve(tlbSize);
    for (unsigned i = 0; i < tlbSize; ++i) {
        tlb.push_back(TLBEntry{});
    }
}

std::optional<uint64_t> MMU::translate(uint64_t vaddr, unsigned accessSize, bool isWrite) {
    // Very small page assumption: 4KB pages — derive VPN/PFN accordingly.
    const uint64_t pageSize = 4096ULL;
    uint64_t vpn = vaddr / pageSize;
    uint64_t pageOffset = vaddr % pageSize;

    // Linear search TLB — fine for small TLB sizes in this prototype
    for (unsigned i = 0; i < tlbSize; ++i) {
        const TLBEntry &e = tlb[i];
        if (!e.valid) continue;

        // For now ignore ASID/global; a fuller model should check CP0[ASID] and e.asid.
        if (e.vpn == static_cast<uint32_t>(vpn)) {
            if (!e.validP) {
                // invalid PTE -> trap
                if (true) std::cerr << "[MMU] TLB entry invalid P for vpn=0x" << std::hex << vpn << std::dec << "\n";
                return std::nullopt;
            }
            // if write and not dirty -> write-protect trap
            if (isWrite && !e.dirty) {
                if (true) std::cerr << "[MMU] TLB write to read-only page vpn=0x" << std::hex << vpn << std::dec << "\n";
                return std::nullopt;
            }
            uint64_t phys = (static_cast<uint64_t>(e.pfn) * pageSize) + pageOffset;
            return phys;
        }
    }

    // No matching TLB entry — TLB miss
    if (true) std::cerr << "[MMU] TLB miss for vaddr=0x" << std::hex << vaddr << std::dec << "\n";
    return std::nullopt;
}

void MMU::insertTLBEntry(unsigned index, const TLBEntry &e) {
    if (index >= tlb.size()) {
        std::cerr << "[MMU] insertTLBEntry: index out of range, wrapping\n";
        index = index % tlb.size();
    }
    tlb[index] = e;
}

void MMU::flushAll() {
    for (auto &e : tlb) e.valid = false;
}
