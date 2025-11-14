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

// -----------------------------------------------------------
// emulator.cpp  (Part 2 — Octane1 SI Hardware Map)
// -----------------------------------------------------------
// Speedracer SGI Octane1 Emulator
// Implements:
//   - RAM region
//   - HEART MMIO
//   - HUB MMIO
//   - MACE I/O (serial, RTC stub)
//   - SI/CRM Graphics device (framebuffer base)
//   - PROM-compatible memory routing
// -----------------------------------------------------------
// -----------------------------------------------------------
// Memory map constants for Octane1 (IP30)
// -----------------------------------------------------------

// RAM is mapped at physical 0x0000_0000 ...  (size configurable)

static constexpr uint64_t HEART_BASE = 0x1F000000;
static constexpr uint64_t HUB_BASE   = 0x1F200000;
static constexpr uint64_t MACE_BASE  = 0x1F400000;
static constexpr uint64_t CRM_BASE   = 0x1F600000;

static constexpr uint64_t MMIO_SIZE  = 0x00200000;  // each block 2MB

// Special: Octane PROM maps framebuffer at CRM_BASE
// PROM requests 1280x1024 at 32-bit color by default.

// -----------------------------------------------------------
// MMIO read32
// -----------------------------------------------------------
uint32_t Emulator::mmio_read32(uint64_t phys)
{
    // HEART
    if (phys >= HEART_BASE && phys < HEART_BASE + MMIO_SIZE) {
        uint32_t off = phys - HEART_BASE;

        // PROM probes HEART revision register
        if (off == 0x0000) {
            // Fake a simple HEART version (PROM only checks non-zero)
            return 0x00010001; 
        }

        // default stub
        return 0;
    }

    // HUB
    if (phys >= HUB_BASE && phys < HUB_BASE + MMIO_SIZE) {
        uint32_t off = phys - HUB_BASE;

        // PROM probes memory controller registers
        if (off == 0x0010) {
            return 0x00000002;  // 2 memory banks
        }

        return 0;
    }

    // MACE (serial, RTC, PS/2, etc)
    if (phys >= MACE_BASE && phys < MACE_BASE + MMIO_SIZE) {
        uint32_t off = phys - MACE_BASE;

        // Serial port read (PROM uses this!)
        if (off == 0x50000) {
            // Return "no character"
            return 0xFFFFFFFF;
        }

        return 0;
    }

    // CRM / SI Graphics
    if (phys >= CRM_BASE && phys < CRM_BASE + MMIO_SIZE) {
        uint32_t off = phys - CRM_BASE;

        // PROM reads CRM status register
        if (off == 0x0000) {
            return 0x00000001; // "present"
        }

        // PROM reads board ID
        if (off == 0x0004) {
            return 0x00000020; // SI board type
        }

        return 0;
    }

    std::cerr << "[MMIO] Unknown read32 @ phys=0x" 
              << std::hex << phys << std::dec << "\n";
    return 0;
}

// -----------------------------------------------------------
// MMIO write32
// -----------------------------------------------------------
void Emulator::mmio_write32(uint64_t phys, uint32_t val)
{
    // HEART
    if (phys >= HEART_BASE && phys < HEART_BASE + MMIO_SIZE) {
        uint32_t off = phys - HEART_BASE;

        // PROM writes watchdog registers, ignore safely
        if (off < 0x1000) return;

        return;
    }

    // HUB
    if (phys >= HUB_BASE && phys < HUB_BASE + MMIO_SIZE) {
        uint32_t off = phys - HUB_BASE;

        // ignore for now, safe stub
        return;
    }

    // MACE I/O
    if (phys >= MACE_BASE && phys < MACE_BASE + MMIO_SIZE) {
        uint32_t off = phys - MACE_BASE;

        // Serial output: PROM prints characters here
        if (off == 0x50000) {
            char c = (char)(val & 0xFF);
            std::cout << c;
            return;
        }

        return;
    }

    // CRM Graphics
    if (phys >= CRM_BASE && phys < CRM_BASE + MMIO_SIZE) {
        uint32_t off = phys - CRM_BASE;

        // PROM sometimes writes display config registers
        return;
    }

    std::cerr << "[MMIO] Unknown write32 @ phys=0x"
              << std::hex << phys << " = 0x" << val << std::dec << "\n";
}

// -----------------------------------------------------------
// SYSTEM READ (CPU -> MMU -> RAM/MMIO dispatcher)
// -----------------------------------------------------------
uint32_t Emulator::sys_read32(uint64_t phys)
{
    // MMIO?
    if (phys >= HEART_BASE)
        return mmio_read32(phys);

    // RAM
    return memory.read32(phys);
}

// -----------------------------------------------------------
// SYSTEM WRITE
// -----------------------------------------------------------
void Emulator::sys_write32(uint64_t phys, uint32_t val)
{
    if (phys >= HEART_BASE) {
        mmio_write32(phys, val);
        return;
    }

    memory.write32(phys, val);
}

// -----------------------------------------------------------
// PROM load helper
// -----------------------------------------------------------
bool Emulator::load_prom(const std::string& path)
{
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) {
        std::cerr << "[Emu] Failed to open PROM file: " << path << "\n";
        return false;
    }

    std::vector<uint8_t> buf;
    buf.resize(4 * 1024 * 1024);  // up to 4 MB PROM

    size_t n = fread(buf.data(), 1, buf.size(), f);
    fclose(f);

    if (n == 0) {
        std::cerr << "[Emu] PROM file empty\n";
        return false;
    }

    // PROM is mapped at high virtual, but physically loaded at 0x1FC00000
    uint64_t prom_phys_base = 0x1FC00000;

    // load into RAM
    memory.load_blob(prom_phys_base, buf.data(), n);

    std::cout << "[Emu] PROM loaded: " << n << " bytes\n";
    return true;
}
