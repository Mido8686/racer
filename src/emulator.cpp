// -----------------------------------------------------------
// emulator.cpp (Part 1 — Core System Setup)
// -----------------------------------------------------------
// Speedracer SGI Octane1 Emulator
// Core initialization:
//   - Create CPU, MMU, CP0, Memory
//   - Wire all subsystems together
//   - Provide run loop
//   - Provide physical read/write for devices
// -----------------------------------------------------------

#include "emulator.h"
#include "cpu.h"
#include "mmu.h"
#include "memory.h"
#include "cp0.h"
#include <iostream>

Emulator::Emulator()
{
    cpu  = new CPU();
    mmu  = new MMU();
    cp0  = new CP0();
    mem  = new Memory();
}

Emulator::~Emulator()
{
    delete cpu;
    delete mmu;
    delete cp0;
    delete mem;
}

// -----------------------------------------------------------
// Initialize emulator
// -----------------------------------------------------------
bool Emulator::init(uint64_t ram_size)
{
    std::cout << "[Emu] Initializing system...\n";

    // Setup RAM
    mem->init(ram_size);

    // Attach subsystems
    cpu->attach_mmu(mmu);
    cpu->attach_cp0(cp0);

    mmu->attach_memory(mem);
    mmu->attach_cp0(cp0);

    cp0->attach_cpu(cpu);

    // Reset all components
    cp0->reset();
    mmu->reset();
    cpu->reset();

    std::cout << "[Emu] System ready.\n";
    return true;
}

// -----------------------------------------------------------
// Load PROM (declared in Part 2)
// -----------------------------------------------------------
bool Emulator::load_prom(const std::string& path);

// -----------------------------------------------------------
// Main execution loop
// -----------------------------------------------------------
void Emulator::run(uint64_t cycles)
{
    std::cout << "[Emu] Starting CPU...\n";

    for (uint64_t i = 0; i < cycles; i++)
    {
        cpu->step();

        // For future: handle timers, interrupts, VBLANK, DMA
    }
}

// -----------------------------------------------------------
// System read/write (to be filled by Part 2: MMIO)
// Default behavior: direct RAM access
// -----------------------------------------------------------
uint32_t Emulator::sys_read32(uint64_t phys)
{
    return mem->read32(phys);
}

void Emulator::sys_write32(uint64_t phys, uint32_t val)
{
    mem->write32(phys, val);
}
