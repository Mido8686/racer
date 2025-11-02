// speedracer/src/cpu.h
#pragma once
#include <cstdint>
#include <array>
#include "membus.h"

// Simple MIPS CPU skeleton (big-endian)
// - 32 general purpose registers (GPRs) 64-bit capable (we use uint64_t but operations here are kept 32-bit for simplicity)
// - Program Counter (PC) 32-bit (for MIPS32/compat boot code)
// - Simple run loop with step limit to avoid infinite loops in early development

class MIPSCpu {
public:
    MIPSCpu(MemoryBus &bus);
    ~MIPSCpu() = default;

    // Initialize cpu state and set PC (entry point)
    void reset(uint32_t entry_pc);

    // Run until halted or steps exhausted
    void run(uint64_t max_instructions = 1000000);

    // Single-step (execute one instruction)
    void step();

    // Expose internal state (for debugging)
    uint32_t pc() const { return PC; }
    const std::array<uint64_t, 32>& regs() const { return GPR; }

private:
    MemoryBus &bus;
    std::array<uint64_t, 32> GPR;
    uint32_t PC;
    bool running;

    // Helpers
    uint32_t fetch32(uint32_t addr);
    void decode_execute(uint32_t instr);

    // Basic instruction implementations
    void op_lui(uint32_t instr);
    void op_addi(uint32_t instr);
    void op_jal(uint32_t instr);
    void op_j(uint32_t instr);
    void op_rtype(uint32_t instr); // includes jr
};
