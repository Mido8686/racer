#include "cp0.h"
#include <cassert>
#include <iostream>

CP0::CP0() {
    regs.fill(0);
    // Typical default: EXL clear, IE may be set by firmware. Leave 0 here.
}

uint32_t CP0::read(unsigned reg) const {
    assert(reg < regs.size());
    return regs[reg];
}

void CP0::write(unsigned reg, uint32_t value) {
    assert(reg < regs.size());
    // Special handling for some registers:
    switch (reg) {
        case REG_COUNT:
            regs[REG_COUNT] = value;
            break;
        case REG_COMPARE:
            regs[REG_COMPARE] = value;
            // Writing to Compare usually clears the timer interrupt bit in Cause
            // Clear IP7..IP0 bits (implementation detail): here we clear the timer flag bit 7 in Cause.
            regs[REG_CAUSE] &= ~(1u << 15); // simple example: clear one IP bit
            break;
        case REG_STATUS:
            // Mask out read-only/reserved bits in a fuller implementation.
            regs[REG_STATUS] = value;
            break;
        case REG_CAUSE:
            // In many cores cause is mostly read-only; allow writing some bits (e.g., software-set IP)
            regs[REG_CAUSE] = (regs[REG_CAUSE] & ~0xFF00u) | (value & 0xFF00u);
            break;
        default:
            regs[reg] = value;
            break;
    }
}

void CP0::setEXL(bool v) {
    if (v) regs[REG_STATUS] |= (1u << 1);
    else regs[REG_STATUS] &= ~(1u << 1);
}

bool CP0::getEXL() const {
    return (regs[REG_STATUS] & (1u << 1)) != 0;
}

void CP0::setEPC(uint32_t v) {
    regs[REG_EPC] = v;
}

uint32_t CP0::getEPC() const {
    return regs[REG_EPC];
}

void CP0::setCause(uint32_t v) {
    regs[REG_CAUSE] = v;
}

uint32_t CP0::getCause() const {
    return regs[REG_CAUSE];
}

void CP0::tickCount() {
    // Increment Count register; on overflow compared to Compare, set timer interrupt IPx bit in Cause.
    // This is a very simple model — in a more accurate model Count increments once per two cycles, etc.
    uint32_t count = ++regs[REG_COUNT];
    uint32_t compare = regs[REG_COMPARE];

    if (compare != 0 && count == compare) {
        // Set a software timer interrupt bit in Cause register: use IP7 as example (bit 15).
        regs[REG_CAUSE] |= (1u << 15);
        // Optionally, signal to CPU/MMU via some callback — we'll let CPU poll CP0::getHWPending or read Cause.
        if (false) { // placeholder to illustrate where one could add a callback hook
            std::cout << "[CP0] Timer interrupt set (Count==Compare)\n";
        }
    }
}

void CP0::setHWPending(uint32_t mask) {
    // Place mask into Cause IP bits area (commonly bits 8..15)
    regs[REG_CAUSE] |= ((mask & 0xFFu) << 8);
}

void CP0::clearHWPending(uint32_t mask) {
    regs[REG_CAUSE] &= ~((mask & 0xFFu) << 8);
}

uint32_t CP0::getHWPending() const {
    return (regs[REG_CAUSE] >> 8) & 0xFFu;
}
