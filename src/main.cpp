#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <cstdint>
#include <string>

// -----------------------------------------------------------
// Speedracer Emulator (SGI Octane1 / IP30)
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
    std::cout << "  Speedracer Emulator (SGI Octane1)\n";
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
