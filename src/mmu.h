// -----------------------------------------------------------
// mmu.h
// -----------------------------------------------------------
// Racer (SGI Octane1) Emulator - MMU header
// Implements flat-mode and simple TLB table with basic MIPS semantics.
// -----------------------------------------------------------

#pragma once
#include <cstdint>
#include <array>
#include <stdexcept>

class Memory;
class CP0;

struct TLBEntry {
    uint64_t vpn2 = 0;       // virtual page number (VPN2)
    uint32_t asid = 0;       // ASID
    uint64_t pfn0 = 0;       // physical frame 0
    uint64_t pfn1 = 0;       // physical frame 1
    bool valid0 = false;
    bool valid1 = false;
    bool dirty0 = false;
    bool dirty1 = false;
    uint32_t pagemask = 0;
    bool global0 = false;
    bool global1 = false;
    bool valid = false;      // indicates entry used
};

class MMU {
public:
    MMU();
    ~MMU();

    void attach_memory(Memory* m);
    void attach_cp0(CP0* c);

    void reset();

    // Status codes for translate()
    static constexpr int TLB_OK = 0;
    static constexpr int TLB_MISS = 1;
    static constexpr int TLB_INVALID = 2;

    // New translate API that CPU Part12 expects:
    //  - returns TLB_OK / TLB_MISS / TLB_INVALID
    //  - writes physical address to paddr (if TLB_OK)
    int translate(uint64_t vaddr, uint64_t& paddr, bool write);

    // Legacy helper for flat mode (keeps earlier compatibility)
    uint64_t translate_flat(uint64_t vaddr);

    // Read/write helpers used by CPU (big-endian handling is done in CPU)
    uint8_t  read8(uint64_t vaddr);
    uint16_t read16(uint64_t vaddr);
    uint32_t read32(uint64_t vaddr);
    uint64_t read64(uint64_t vaddr);

    void write8(uint64_t vaddr, uint8_t  v);
    void write16(uint64_t vaddr, uint16_t v);
    void write32(uint64_t vaddr, uint32_t v);
    void write64(uint64_t vaddr, uint64_t v);

    // TLB operations called by the CPU (COP0 handlers)
    void tlbr(CP0* cp0);   // read TLB entry -> CP0 registers
    void tlbwi(CP0* cp0);  // write CP0 registers -> TLB[index]
    void tlbwr(CP0* cp0);  // write CP0 registers -> TLB[random]
    void tlbp(CP0* cp0);   // probe TLB for match -> set CP0.Index

    // For CP0/Emulator to toggle TLB mode
    void set_tlb_enabled(bool en) { enable_tlb = en; }
    bool is_tlb_enabled() const { return enable_tlb; }

    // expose constants
    static constexpr int MAX_TLB = 64;

private:
    Memory* mem = nullptr;
    CP0*    cp0 = nullptr;
    bool    enable_tlb = false;

    // Simple TLB table
    std::array<TLBEntry, MAX_TLB> tlb;

    // Random pointer (simple implementation)
    uint32_t random_index = 0;

    // Internal helpers
    int lookup_tlb(uint64_t vaddr, uint64_t& paddr, bool write);
    void read_cp0_tlb_from_cp0(CP0* cp0, TLBEntry& e);
    void write_cp0_tlb_to_cp0(CP0* cp0, const TLBEntry& e);
};
