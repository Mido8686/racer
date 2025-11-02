# 🏎️ Speedracer Emulator  
### SGI Octane1 (IP30) Hardware Emulator  

**Speedracer Emulator** is an open-source project that aims to **emulate the Silicon Graphics Octane1 (IP30)** workstation system from scratch.  
It is designed to be **fully implemented**, **accurate**, and **user-friendly** — allowing enthusiasts and developers to explore SGI hardware and firmware behavior.

---

## 🚧 Project Status: In Development

This project is currently in **early development**.  
Core goals include:

- ✅ Accurate MIPS (R10000-class) CPU emulation  
- 🧠 PROM (firmware) loading and analysis  
- 🖥️ UART console output (early boot messages)  
- 💾 Basic device emulation (SCSI, Ethernet, RTC, Graphics planned)  
- 🧩 Modular architecture for easy debugging and expansion  

---

## 📦 Requirements

To run **Speedracer Emulator**, you will need a valid SGI Octane1 PROM file:

`ip30prom.rev4.9.bin`




> ⚠️ **Note:** This firmware image is *not included* in the repository.  
> You must provide your own legally obtained copy of the Octane1 PROM file.

---

## 🛠️ Planned Features

| Component | Description | Status |
|------------|--------------|--------|
| MIPS R10000 CPU | Core CPU emulation | ⏳ Planned |
| PROM Loader | Loads and maps `ip30prom.rev4.9.bin` | ✅ Working |
| UART Console | Serial output for PROM logs | ✅ Prototype |
| Memory System | 1 MiB ROM mapping at `0xbfc00000` | ✅ Working |
| Ethernet / SCSI | Basic stubs for boot devices | ⏳ In Progress |
| Graphics (Odyssey / VPro) | Framebuffer + GPU stubs | ⏳ Planned |
| User Interface | Friendly terminal & GUI options | ⏳ Planned |

---

## 📚 Development Notes

- PROM base address: `0xBFC00000` (1 MiB region)  
- Compatible firmware: `SGI Version 6.5 Rev 4.9 IP30 (May 22, 2003)`  
- Emulator name: **Speedracer** 🏎️ — designed for both **speed** and **accuracy**

---

## 🧑‍💻 Contributing

Contributions are welcome!  
If you’d like to help implement devices, improve emulation accuracy, or enhance documentation:

1. Fork this repository  
2. Create a new branch (`feature/device-uart`, `fix/memory-map`, etc.)  
3. Submit a pull request with your improvements  

Please follow the contribution guidelines in [`CONTRIBUTING.md`](CONTRIBUTING.md) (if available).

---

## 📜 License

This project will be released under an open-source license once development reaches a stable stage.  
For now, all source code and documentation are © 2025 Mohamed Zeidan, All Rights Reserved.

---

## 🧩 Acknowledgments

- Silicon Graphics, Inc. — for the original Octane1 hardware  
- Community contributors working to preserve SGI systems  

---

### 💡 Project Motto
> “Rebuilding the power of SGI — one instruction at a time.”
