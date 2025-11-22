#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <cstdint>
#include <string>
#include "emulator.h"
#include <iostream>

int emulator_main(const std::string &prom_path, const std::string &irix_iso_path) {
    try {
        Emulator emu;
        emu.init();

        // Attach a framebuffer device so user sees display (optional).
        // Choose MMIO base and framebuffer physical base consistent with dev/framebuffer defaults.
        const uint64_t fb_mmio = 0x1F000000ULL;
        const uint64_t fb_phys = 0x10000000ULL;
        emu.attach_framebuffer(fb_mmio, fb_phys);

        // Load PROM
        if (!emu.load_prom(prom_path)) {
            std::cerr << "[MAIN] Failed to load PROM: " << prom_path << "\n";
            return 2;
        }

        // Optionally, if an IRIX ISO path was provided, register it as a virtual CD-ROM.
        // This requires SCSI/CD emulation not included here; provide hook for later:
        if (!irix_iso_path.empty()) {
            std::cout << "[MAIN] IRIX ISO provided: " << irix_iso_path << "\n";
            // Hook point: emu.attach_cdrom(irix_iso_path) or mount ISO into disk subsystem.
            // If you implemented load_disk_image to accept ISO, call it:
            // emu.load_disk_image(irix_iso_path);
        }

        // Reset CPU & start running
        emu.reset();
        emu.run();

        // Cleanup happens in Emulator destructor
        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "[MAIN] Uncaught exception: " << ex.what() << "\n";
        return 99;
    } catch (...) {
        std::cerr << "[MAIN] Unknown fatal error\n";
        return 100;
    }
}

// -----------------------------------------------------------
// Racer Emulator (SGI Octane1 / IP30)
// Initial Boot Framework - PROM Loader
// -----------------------------------------------------------
// Author: Mohamed Zeidan
// Date: November 2025
// -----------------------------------------------------------

constexpr uint32_t PROM_START_ADDR = 0xBFC00000;
constexpr size_t PROM_SIZE = 1024 * 1024; // 1 MB

// Simple memory class for PROM region
class Memory {
public:
    Memory(size_t size) : data(size, 0) {}

    bool loadFile(const std::string& path) {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file.is_open()) {
            std::cerr << "❌ Error: Cannot open PROM file: " << path << "\n";
            return false;
        }
        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);
        if (size > static_cast<std::streamsize>(data.size())) {
            std::cerr << "⚠️ Warning: PROM file too large, truncating to "
                      << data.size() << " bytes\n";
            size = data.size();
        }
        file.read(reinterpret_cast<char*>(data.data()), size);
        std::cout << "✅ Loaded PROM (" << size << " bytes)\n";
        return true;
    }

    uint8_t read8(uint32_t addr) const {
        if (addr < PROM_START_ADDR || addr >= PROM_START_ADDR + data.size()) {
            std::cerr << "⚠️ Invalid memory read @0x" << std::hex << addr << "\n";
            return 0xFF;
        }
        return data[addr - PROM_START_ADDR];
    }

    uint32_t read32(uint32_t addr) const {
        uint32_t val = 0;
        for (int i = 0; i < 4; ++i)
            val = (val << 8) | read8(addr + i);
        return val;
    }

    void dump(size_t count = 64) const {
        std::cout << "PROM Dump (first " << count << " bytes):\n";
        for (size_t i = 0; i < count; ++i) {
            if (i % 16 == 0)
                std::cout << "\n" << std::hex << std::setw(8) << (PROM_START_ADDR + i) << ": ";
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(data[i]) << " ";
        }
        std::cout << std::dec << "\n\n";
    }

private:
    std::vector<uint8_t> data;
};

// -----------------------------------------------------------
// Main Entry
// -----------------------------------------------------------
int main() {
    std::cout << "=====================================\n";
    std::cout << "  Racer Emulator (SGI Octane1)\n";
    std::cout << "  Boot Framework - PROM Loader\n";
    std::cout << "=====================================\n";

    const std::string romPath = "../roms/ip30prom.rev4.9.bin";

    Memory prom(PROM_SIZE);

    if (!prom.loadFile(romPath)) {
        std::cerr << "❌ Failed to load PROM. Ensure the file exists at:\n   " << romPath << "\n";
        return 1;
    }

    prom.dump(128); // Show first 128 bytes

    std::cout << "✅ PROM successfully loaded into memory.\n";
    std::cout << "Next step: Initialize CPU skeleton & instruction fetch loop.\n";

    return 0;
}
