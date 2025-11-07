// emulator.h
// ---------------------------------------------
// Speedracer SGI Octane1 Emulator - Core Header
// Defines the main Emulator class that coordinates
// CPU, MMU, CP0, devices, and the system bus.
//
// This header is compatible with previous parts (cpu.cpp/mmu.cpp/etc.)
// and with tui_single_part2.cpp and emulator_main().
//
// Place this file in: src/emulator.h
// ---------------------------------------------

#pragma once
#include <cstdint>
#include <string>
#include <memory>
#include <vector>
#include <iostream>

// Forward declarations of major subsystems
class CPU;
class CP0;
class MMU;
class Memory;

// Forward declaration for framebuffer device & window if they exist
namespace dev {
    class FramebufferDevice;
}
class SDLFramebufferWindow;

// ---------------------------------------------
// Emulator class definition
// ---------------------------------------------
class Emulator {
public:
    Emulator();
    ~Emulator();

    // Initialize all subsystems (CPU, MMU, CP0, Memory, etc.)
    void init();

    // Reset the system to power-on state (like PROM reset)
    void reset();

    // Run main emulation loop until halted
    void run();

    // Load PROM binary into physical memory (for IP30)
    bool load_prom(const std::string &prom_path);

    // Attach framebuffer device (optional, for graphics)
    bool attach_framebuffer(uint64_t fb_mmio_base = 0x1F000000ULL,
                            uint64_t fb_phys_base = 0x10000000ULL);

    // Load disk or ISO image (optional)
    void load_disk_image(const std::string &path);

    // Simple state query (for debugging)
    bool is_running() const { return running; }

    // Stop emulation cleanly
    void stop() { running = false; }

    // Pointers to subsystems for external access if needed
    Memory *get_memory() { return &mem; }
    CPU    *get_cpu()    { return &cpu; }
    MMU    *get_mmu()    { return &mmu; }
    CP0    *get_cp0()    { return &cp0; }

private:
    bool running = false;

    // Core components
    Memory mem;
    CP0    cp0;
    MMU    mmu;
    CPU    cpu;

    // Devices
    dev::FramebufferDevice *fbdev = nullptr;
    SDLFramebufferWindow   *fbWindow = nullptr;

    // Internal helpers
    void run_one_frame();
};

