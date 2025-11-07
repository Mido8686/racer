// tui_single.cpp
// Speedracer (SGI Octane1) TUI launcher
// -------------------------------------
// Text-based user interface to launch the SGI Octane emulator.
// Checks for required PROM file: ip30prom.rev4.9.bin
// Future: integrate full emulator launch here.

#include <iostream>
#include <string>
#include <filesystem>
#include <thread>
#include <chrono>

namespace fs = std::filesystem;

static const std::string PROM_FILENAME = "ip30prom.rev4.9.bin";

void clear_screen() {
#ifdef _WIN32
    system("cls");
#else
    std::cout << "\033[2J\033[H";
#endif
}

void pause_for_key() {
    std::cout << "\nPress ENTER to continue...";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void draw_header() {
    std::cout << "┌────────────────────────────────────┐\n";
    std::cout << "│  Speedracer SGI Octane1 Emulator   │\n";
    std::cout << "├────────────────────────────────────┤\n";
}

void launch_sgi_octane() {
    clear_screen();
    draw_header();
    std::cout << "Checking for PROM file: " << PROM_FILENAME << "\n";

    if (!fs::exists(PROM_FILENAME)) {
        std::cout << "❌ PROM not found.\n";
        std::cout << "   Please place " << PROM_FILENAME
                  << " in the current directory.\n";
        pause_for_key();
        return;
    }

    std::cout << "✅ PROM found. Preparing to launch SGI Octane...\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Stub emulator launch:
    // Replace this section with call to your emulator (e.g. run_emulator())
    clear_screen();
    draw_header();
    std::cout << "Booting from PROM: " << PROM_FILENAME << "\n";
    std::cout << "(Simulation stub — replace with actual launch)\n";
    std::cout << "\n[Initializing hardware...]\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    std::cout << "[CPU] R10000 195 MHz OK\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "[MEM] 512 MB OK\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "[GFX] SI/SE board detected (1280×1024 @ 60 Hz)\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "[PROM] Boot sequence starting…\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "\n→ Hand-off to IRIX (if ISO provided)\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "\n(End of stub — emulator integration point)\n";
    pause_for_key();
}

void show_settings() {
    clear_screen();
    draw_header();
    std::cout << "Settings:\n";
    std::cout << "----------------------------------------\n";
    std::cout << " Screen Resolution : 1280 × 1024\n";
    std::cout << " Refresh Rate      : 60 Hz\n";
    std::cout << " PROM File         : " << PROM_FILENAME << "\n";
    std::cout << " IRIX ISO (optional): irix_6.5.30_overlays.iso\n";
    std::cout << "----------------------------------------\n";
    std::cout << "\n(Editing not yet implemented — modify config manually.)\n";
    pause_for_key();
}

int main() {
    while (true) {
        clear_screen();
        draw_header();
        std::cout << "│ 1. Launch SGI Octane               │\n";
        std::cout << "│ 2. Settings                        │\n";
        std::cout << "│ 3. Exit                            │\n";
        std::cout << "└────────────────────────────────────┘\n";
        std::cout << "\nSelect option: ";
        std::string choice;
        std::getline(std::cin, choice);
        if (choice == "1") {
            launch_sgi_octane();
        } else if (choice == "2") {
            show_settings();
        } else if (choice == "3" || choice == "q" || choice == "Q") {
            clear_screen();
            std::cout << "Goodbye.\n";
            break;
        } else {
            std::cout << "Invalid choice.\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(800));
        }
    }
    return 0;
}
