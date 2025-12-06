// -----------------------------------------------------------
// mmu.cpp
// -----------------------------------------------------------
// Racer (SGI Octane1) Emulator - MMU implementation
// Implements basic TLB behavior and a flat fallback mode.
// -----------------------------------------------------------

#include "mmu.h"
#include "memory.h"
#include "cp0.h"
#include <iostream>
#include <cstring>

// -----------------------------------------------------------
// Constructor / Destructor
// -----------------------------------------------------------
MMU::MMU()
{
    for (auto &e : tlb) e = TLBEntry{};
    random_index = 0;
}

MMU::~MMU() = default;

// -----------------------------------------------------------
// attach subsystems
// -----------------------------------------------------------
void MMU::attach_memory(Memory* m) { mem = m; }
void MMU::attach_cp0(CP0* c)       { cp0 = c; }

// -----------------------------------------------------------
// reset
// -----------------------------------------------------------
void MMU::reset() {
    enable_tlb = false;
    for (auto& e : tlb) e = TLBEntry{};
    random_index = 0;
    std::cout << "[MMU] Reset — flat mode enabled\n";
}

// -----------------------------------------------------------
// translate_flat - simple 1:1-ish mapping used before TLB enabled
// This mirrors earlier simple behavior but keeps bounds safer.
// -----------------------------------------------------------
uint64_t MMU::translate_flat(uint64_t vaddr)
{
    // Keep the low 29 bits (as earlier stub did)
    return vaddr & 0x1FFFFFFFULL;
}

// -----------------------------------------------------------
// lookup_tlb - CPU calls this to find physical mapping
// Returns: TLB_OK, TLB_MISS, or TLB_INVALID
// -----------------------------------------------------------
int MMU::lookup_tlb(uint64_t vaddr, uint64_t& paddr, bool write)
{
    // Extract VPN2 (use 4K pages here for simplicity)
    uint64_t vpn2 = vaddr >> 12;

    // Iterate TLB for first match
    for (size_t i = 0; i < tlb.size(); ++i) {
        const auto &e = tlb[i];
        if (!e.valid) continue;

        // Match by vpn2 (we stored vpn2 as full >>12)
        if (e.vpn2 != (vpn2 & ~0ULL)) {
            // We stored exact vpn2 earlier; compare directly
            // (we keep simple semantics)
        }

        // Simpler: check both halves
        uint64_t entry_vpn2 = e.vpn2;
        if (entry_vpn2 == (vaddr >> 12)) {
            // ASID/global check
            uint32_t cur_asid = 0;
            if (cp0) cur_asid = (uint32_t)cp0->read_reg(10); // EntryHi low bits hold ASID in many implementations (read_reg index may differ)
            bool asid_match = (e.asid == cur_asid) || e.global0 || e.global1;
            if (!asid_match) continue;

            // Choose PFN based on low bit of page (even/odd)
            uint32_t page_bit = (vaddr >> 12) & 1;
            uint64_t pfn = page_bit ? e.pfn1 : e.pfn0;
            bool valid = page_bit ? e.valid1 : e.valid0;
            bool dirty = page_bit ? e.dirty1 : e.dirty0;

            if (!valid) return TLB_INVALID;
            // If write and not dirty -> write-protect -> treat as invalid
            if (write && !dirty) return TLB_INVALID;

            // Physical address = pfn<<12 | offset
            paddr = (pfn << 12) | (vaddr & 0xFFF);
            return TLB_OK;
        }
    }

    return TLB_MISS;
}

// -----------------------------------------------------------
// translate - the CPU-facing API.
// If TLB disabled -> flat mapping.
// If enabled -> call lookup_tlb and return status.
// -----------------------------------------------------------
int MMU::translate(uint64_t vaddr, uint64_t& paddr, bool write)
{
    if (!enable_tlb) {
        paddr = translate_flat(vaddr);
        return TLB_OK;
    }

    // TLB enabled -> lookup
    int r = lookup_tlb(vaddr, paddr, write);
    return r;
}

// -----------------------------------------------------------
// Read / write wrappers (used by CPU)
// These call translate() and then memory accesses.
// -----------------------------------------------------------
uint8_t MMU::read8(uint64_t vaddr)
{
    if (!mem) throw std::runtime_error("[MMU] Memory not attached");
    uint64_t paddr;
    int r = translate(vaddr, paddr, false);
    if (r != TLB_OK) {
        throw std::runtime_error("[MMU] read8 TLB fault");
    }
    return mem->read8(paddr);
}

uint16_t MMU::read16(uint64_t vaddr)
{
    if (!mem) throw std::runtime_error("[MMU] Memory not attached");
    uint64_t paddr;
    int r = translate(vaddr, paddr, false);
    if (r != TLB_OK) {
        throw std::runtime_error("[MMU] read16 TLB fault");
    }
    return mem->read16(paddr);
}

uint32_t MMU::read32(uint64_t vaddr)
{
    if (!mem) throw std::runtime_error("[MMU] Memory not attached");
    uint64_t paddr;
    int r = translate(vaddr, paddr, false);
    if (r != TLB_OK) {
        throw std::runtime_error("[MMU] read32 TLB fault");
    }
    return mem->read32(paddr);
}

uint64_t MMU::read64(uint64_t vaddr)
{
    if (!mem) throw std::runtime_error("[MMU] Memory not attached");
    uint64_t paddr;
    int r = translate(vaddr, paddr, false);
    if (r != TLB_OK) {
        throw std::runtime_error("[MMU] read64 TLB fault");
    }
    return mem->read64(paddr);
}

void MMU::write8(uint64_t vaddr, uint8_t v)
{
    if (!mem) throw std::runtime_error("[MMU] Memory not attached");
    uint64_t paddr;
    int r = translate(vaddr, paddr, true);
    if (r != TLB_OK) {
        throw std::runtime_error("[MMU] write8 TLB fault");
    }
    mem->write8(paddr, v);
}

void MMU::write16(uint64_t vaddr, uint16_t v)
{
    if (!mem) throw std::runtime_error("[MMU] Memory not attached");
    uint64_t paddr;
    int r = translate(vaddr, paddr, true);
    if (r != TLB_OK) {
        throw std::runtime_error("[MMU] write16 TLB fault");
    }
    mem->write16(paddr, v);
}

void MMU::write32(uint64_t vaddr, uint32_t v)
{
    if (!mem) throw std::runtime_error("[MMU] Memory not attached");
    uint64_t paddr;
    int r = translate(vaddr, paddr, true);
    if (r != TLB_OK) {
        throw std::runtime_error("[MMU] write32 TLB fault");
    }
    mem->write32(paddr, v);
}

void MMU::write64(uint64_t vaddr, uint64_t v)
{
    if (!mem) throw std::runtime_error("[MMU] Memory not attached");
    uint64_t paddr;
    int r = translate(vaddr, paddr, true);
    if (r != TLB_OK) {
        throw std::runtime_error("[MMU] write64 TLB fault");
    }
    mem->write64(paddr, v);
}


// -----------------------------------------------------------
// TLB operations (TLBR, TLBWI, TLBWR, TLBP)
// These read/write CP0 registers and update the local tlb[] array.
// -----------------------------------------------------------

static inline uint64_t extract_vpn2_from_entryhi(uint64_t entryhi) {
    // in our simplified layout: EntryHi contains VPN2 aligned >>12
    return (entryhi >> 13); // VPN2 is bits 31:13 for 4K pages; shifting to get page index might vary
}

void MMU::tlbr(CP0* cp0)
{
    if (!cp0) return;
    // Read CP0.Index to know which entry to read
    uint64_t idx = cp0->read_reg(0);
    if (idx >= MAX_TLB) idx = 0;

    const TLBEntry &e = tlb[idx];

    if (!e.valid) {
        // Clear CP0 registers to zero for non-present entry
        cp0->write_reg(2, 0); // EntryLo0
        cp0->write_reg(3, 0); // EntryLo1
        cp0->write_reg(10, 0); // EntryHi
        return;
    }

    // Write EntryLo0/1 and EntryHi and PageMask into CP0
    cp0->write_reg(2, (e.pfn0 & 0xFFFFFFFFFFFFFULL)); // simplification
    cp0->write_reg(3, (e.pfn1 & 0xFFFFFFFFFFFFFULL));
    // Pack vpn2 in EntryHi for later
    cp0->write_reg(10, (e.vpn2 << 12) | (uint64_t)e.asid);
    cp0->write_reg(5, e.pagemask);
}

void MMU::tlbwi(CP0* cp0)
{
    if (!cp0) return;
    uint64_t idx = cp0->read_reg(0) & 0x3F;
    if (idx >= MAX_TLB) idx = idx % MAX_TLB;

    TLBEntry e;
    read_cp0_tlb_from_cp0(cp0, e);
    e.valid = true;
    tlb[idx] = e;
    std::cout << "[MMU] TLBWI wrote entry index=" << idx << "\n";
}

void MMU::tlbwr(CP0* cp0)
{
    if (!cp0) return;
    // Use random_index
    random_index = (random_index + 1) % MAX_TLB;
    TLBEntry e;
    read_cp0_tlb_from_cp0(cp0, e);
    e.valid = true;
    tlb[random_index] = e;
    std::cout << "[MMU] TLBWR wrote entry index=" << random_index << "\n";
}

void MMU::tlbp(CP0* cp0)
{
    if (!cp0) return;
    // Read EntryHi to probe VPN/ASID
    uint64_t entryhi = cp0->read_reg(10);
    uint32_t asid = (uint32_t)(entryhi & 0xFF);
    uint64_t vpn2 = (entryhi >> 12);

    int found = -1;
    for (int i = 0; i < (int)MAX_TLB; ++i) {
        const TLBEntry &e = tlb[i];
        if (!e.valid) continue;
        if (e.vpn2 == vpn2 && (e.asid == asid || e.global0 || e.global1)) {
            found = i;
            break;
        }
    }

    if (found >= 0) {
        cp0->write_reg(0, (uint64_t)found);
    } else {
        // Set Index to -1 (implementation-dependent)
        cp0->write_reg(0, (uint64_t)0xFFFFFFFFULL);
    }
}

// -----------------------------------------------------------
// Helper: pack/unpack CP0 <-> TLBEntry
// -----------------------------------------------------------
void MMU::read_cp0_tlb_from_cp0(CP0* cp0, TLBEntry& e)
{
    // Read CP0 registers: EntryHi (10), EntryLo0 (2), EntryLo1 (3), PageMask (5)
    uint64_t entryhi = cp0->read_reg(10);
    uint64_t lo0     = cp0->read_reg(2);
    uint64_t lo1     = cp0->read_reg(3);
    uint64_t pagemask = cp0->read_reg(5);

    // Simple extraction: interpret lo0/lo1 as PFN values
    e.vpn2 = (entryhi >> 12);
    e.asid = (uint32_t)(entryhi & 0xFF);
    e.pfn0 = (uint64_t)(lo0 & 0xFFFFFFFFFFFFFULL);
    e.pfn1 = (uint64_t)(lo1 & 0xFFFFFFFFFFFFFULL);
    e.valid0 = (lo0 & 0x2) != 0;
    e.valid1 = (lo1 & 0x2) != 0;
    e.dirty0 = (lo0 & 0x4) != 0;
    e.dirty1 = (lo1 & 0x4) != 0;
    e.pagemask = (uint32_t)pagemask;
    e.global0 = (lo0 & 0x1) != 0;
    e.global1 = (lo1 & 0x1) != 0;
}

void MMU::write_cp0_tlb_to_cp0(CP0* cp0, const TLBEntry& e)
{
    cp0->write_reg(2, e.pfn0); // EntryLo0
    cp0->write_reg(3, e.pfn1); // EntryLo1
    cp0->write_reg(10, (e.vpn2 << 12) | (uint64_t)e.asid); // EntryHi
    cp0->write_reg(5, e.pagemask); // PageMask
}
