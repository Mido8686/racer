// -----------------------------------------------------------
// cpu.cpp  (Speedracer SGI Octane Emulator)
// Part 1 — Includes, CPU State, Constructor, Reset
// -----------------------------------------------------------

#include <iostream>
#include <cstring>
#include "cpu.h"
#include "mmu.h"
#include "cp0.h"

// -----------------------------------------------------------
// CPU Constructor
// -----------------------------------------------------------
CPU::CPU()
{
    // CPU state init
    memset(regs, 0, sizeof(regs));
    hi = lo = 0;
    pc = nextPC = 0;

    mmu = nullptr;
    cp0 = nullptr;

    // decode tables will be set in Part 2
    init_decode_tables();

    reset();
}

// -----------------------------------------------------------
// CPU Reset (PROM requires correct defaults)
// -----------------------------------------------------------
void CPU::reset()
{
    // Set registers to safe initial state
    memset(regs, 0, sizeof(regs));

    hi = 0;
    lo = 0;

    // PROM entry point is always 0x1fc00000 on IP30 (Octane1)
    pc     = 0x1fc00000;
    nextPC = pc + 4;

    // CP0 reset state (Status, Config, Cause) will be handled in cp0.cpp
    if (cp0)
        cp0->reset();

    regs[0] = 0; // MIPS $zero
}

// -----------------------------------------------------------
// Set MMU and CP0 pointers from Emulator
// -----------------------------------------------------------
void CPU::connect(MMU* m, CP0* c)
{
    mmu = m;
    cp0 = c;
}

// -----------------------------------------------------------
// Placeholder for instruction decoder (implemented in Part 2)
// -----------------------------------------------------------
void CPU::decode_and_execute(uint32_t instr)
{
    // Implemented fully in Part 2 / 3 / 4
    std::cerr << "[CPU] ERROR: decode_and_execute called before implementation.\n";
}

// -----------------------------------------------------------
// Placeholder for decode table init (implemented in Part 2)
// -----------------------------------------------------------
void CPU::init_decode_tables()
{
    // implemented in Part 2
}

// -----------------------------------------------------------
// PART 1 END
// paste code here in Part 2
//------------------------------------------------------------
// -----------------------------------------------------------
// cpu.cpp  (Speedracer SGI Octane Emulator)
// Part 2 — Decode Tables + Initialization
// -----------------------------------------------------------

// Function type for instruction handlers
typedef void (*InstrFunc)(CPU*, uint32_t);

// Primary opcode tables
static InstrFunc OPC_MAIN[64];
static InstrFunc OPC_SPECIAL[64];
static InstrFunc OPC_REGIMM[32];

// Helper extract macros
#define OP(instr)     (((instr) >> 26) & 0x3F)
#define RS(instr)     (((instr) >> 21) & 0x1F)
#define RT(instr)     (((instr) >> 16) & 0x1F)
#define RD(instr)     (((instr) >> 11) & 0x1F)
#define SA(instr)     (((instr) >> 6)  & 0x1F)
#define FN(instr)     ((instr) & 0x3F)
#define IMM(instr)    ((int16_t)((instr) & 0xFFFF))
#define UIMM(instr)   ((uint16_t)((instr) & 0xFFFF))
#define TARGET(instr) ((instr) & 0x03FFFFFF)

inline int64_t SE16(int16_t x) { return (int64_t)x; }

// Forward declarations for instruction functions
// (They will be implemented fully in Part 4 + Part 5)

static void instr_UNIMP(CPU*, uint32_t);

// PROM / IRIX critical opcodes:
static void instr_J(CPU*, uint32_t);
static void instr_JAL(CPU*, uint32_t);
static void instr_BEQ(CPU*, uint32_t);
static void instr_BNE(CPU*, uint32_t);
static void instr_ADDIU(CPU*, uint32_t);
static void instr_SLTI(CPU*, uint32_t);
static void instr_SLTIU(CPU*, uint32_t);
static void instr_LUI(CPU*, uint32_t);
static void instr_LW(CPU*, uint32_t);
static void instr_SW(CPU*, uint32_t);
static void instr_LB(CPU*, uint32_t);
static void instr_SB(CPU*, uint32_t);

// SPECIAL opcodes:
static void instr_JR(CPU*, uint32_t);
static void instr_SYSCALL(CPU*, uint32_t);
static void instr_ADDU(CPU*, uint32_t);
static void instr_AND(CPU*, uint32_t);
static void instr_OR(CPU*, uint32_t);
static void instr_XOR(CPU*, uint32_t);
static void instr_NOR(CPU*, uint32_t);
static void instr_SLL(CPU*, uint32_t);
static void instr_SRL(CPU*, uint32_t);
static void instr_SRA(CPU*, uint32_t);

// -----------------------------------------------------------
// Unimplemented instruction handler
// -----------------------------------------------------------
static void instr_UNIMP(CPU* c, uint32_t instr)
{
    std::cerr << "[CPU] Unimplemented instruction opcode=0x"
              << std::hex << instr << std::dec
              << " at PC=0x" << std::hex << c->pc << std::dec << "\n";
}

// -----------------------------------------------------------
// Initialize MIPS opcode decode tables
// -----------------------------------------------------------
void CPU::init_decode_tables()
{
    // Fill all entries with unimplemented handler
    for (int i = 0; i < 64; i++)
        OPC_MAIN[i] = instr_UNIMP;

    for (int i = 0; i < 64; i++)
        OPC_SPECIAL[i] = instr_UNIMP;

    for (int i = 0; i < 32; i++)
        OPC_REGIMM[i] = instr_UNIMP;

    // -------------------------------------------------------
    // MAIN opcodes
    // -------------------------------------------------------
    OPC_MAIN[0x02] = instr_J;
    OPC_MAIN[0x03] = instr_JAL;
    OPC_MAIN[0x04] = instr_BEQ;
    OPC_MAIN[0x05] = instr_BNE;
    OPC_MAIN[0x08] = instr_ADDIU;
    OPC_MAIN[0x09] = instr_ADDIU;
    OPC_MAIN[0x0A] = instr_SLTI;
    OPC_MAIN[0x0B] = instr_SLTIU;
    OPC_MAIN[0x0F] = instr_LUI;

    OPC_MAIN[0x20] = instr_LB;
    OPC_MAIN[0x23] = instr_LW;
    OPC_MAIN[0x28] = instr_SB;
    OPC_MAIN[0x2B] = instr_SW;

    // -------------------------------------------------------
    // SPECIAL opcodes (funct field)
    // -------------------------------------------------------
    OPC_SPECIAL[0x08] = instr_JR;
    OPC_SPECIAL[0x0C] = instr_SYSCALL;
    OPC_SPECIAL[0x20] = instr_ADDU;
    OPC_SPECIAL[0x24] = instr_AND;
    OPC_SPECIAL[0x25] = instr_OR;
    OPC_SPECIAL[0x26] = instr_XOR;
    OPC_SPECIAL[0x27] = instr_NOR;

    OPC_SPECIAL[0x00] = instr_SLL;
    OPC_SPECIAL[0x02] = instr_SRL;
    OPC_SPECIAL[0x03] = instr_SRA;

    // -------------------------------------------------------
    // REGIMM opcodes (PROM uses BLTZ/BGEZ later)
    // Will be implemented in Part 5
    // -------------------------------------------------------

    // end Part 2
}

// -----------------------------------------------------------
// PART 2 END
// paste code here in Part 3
// -----------------------------------------------------------
// -----------------------------------------------------------
// cpu.cpp  (Speedracer SGI Octane Emulator)
// Part 3 — Endian Helpers + Exceptions + MMU Safe Access
// -----------------------------------------------------------

// -----------------------------------------------------------
// BIG ENDIAN read/write helpers
// -----------------------------------------------------------
// PROM and IRIX REQUIRE correct big-endian load/store behavior.

uint32_t CPU::load32_be(uint64_t addr)
{
    // MMU handles memory mapping; we only fix endianness here.
    uint32_t v = mmu->read32(addr);
    return __builtin_bswap32(v);
}

uint8_t CPU::load8(uint64_t addr)
{
    return mmu->read8(addr);
}

uint16_t CPU::load16_be(uint64_t addr)
{
    uint16_t v = mmu->read16(addr);
    return __builtin_bswap16(v);
}

void CPU::store32_be(uint64_t addr, uint32_t val)
{
    uint32_t v = __builtin_bswap32(val);
    mmu->write32(addr, v);
}

void CPU::store8(uint64_t addr, uint8_t val)
{
    mmu->write8(addr, val);
}

void CPU::store16_be(uint64_t addr, uint16_t val)
{
    uint16_t v = __builtin_bswap16(val);
    mmu->write16(addr, v);
}


// -----------------------------------------------------------
// Exception helpers (PROM and IRIX required)
// -----------------------------------------------------------
//
// Minimal model here, full TLB/refill exceptions added in Part F
//

void CPU::raise_exception(int code)
{
    // Set CP0 Cause and EPC
    if (cp0)
        cp0->raise_exception(code, pc);

    // Jump to exception vector
    // PROM uses 0x1fc00380
    pc     = 0x1fc00380;
    nextPC = pc + 4;
}

// For syscall specifically:
void CPU::raise_syscall()
{
    raise_exception(8); // MIPS EXC_SYSCALL = 8
}

// For break instruction:
void CPU::raise_break()
{
    raise_exception(9); // MIPS EXC_BREAK = 9
}

// Address error (misaligned)
void CPU::address_error()
{
    raise_exception(4); // MIPS EXC_ADEL = 4
}


// -----------------------------------------------------------
// PART 3 END
// paste code here in Part 4
// -----------------------------------------------------------
// -----------------------------------------------------------
// cpu.cpp  (Speedracer SGI Octane Emulator)
// Part 4 — Instruction Implementations (PROM Mandatory Subset)
// -----------------------------------------------------------
//
// These instructions are the *minimum* required for:
// • IP30 PROM 4.9 boot
// • Clearing framebuffer
// • ARCS menu input/output
// • Entering sash / fx / inst loader
//
// IRIX kernel boot instructions follow in Part 5.
// -----------------------------------------------------------


// -----------------------------------------------------------
// J — Jump
// -----------------------------------------------------------
static void instr_J(CPU* c, uint32_t ins)
{
    uint64_t target = (uint64_t)(TARGET(ins) << 2);
    c->nextPC = (c->pc & 0xF0000000ULL) | target;
}

// -----------------------------------------------------------
// JAL — Jump and link
// -----------------------------------------------------------
static void instr_JAL(CPU* c, uint32_t ins)
{
    c->regs[31] = c->pc + 4;
    uint64_t target = (uint64_t)(TARGET(ins) << 2);
    c->nextPC = (c->pc & 0xF0000000ULL) | target;
}

// -----------------------------------------------------------
// JR — Jump register
// -----------------------------------------------------------
static void instr_JR(CPU* c, uint32_t ins)
{
    c->nextPC = c->regs[RS(ins)];
}


// -----------------------------------------------------------
// ADDIU — Add immediate unsigned
// (PROM uses this constantly for pointer math)
// -----------------------------------------------------------
static void instr_ADDIU(CPU* c, uint32_t ins)
{
    c->regs[RT(ins)] = c->regs[RS(ins)] + SE16(IMM(ins));
}


// -----------------------------------------------------------
// ADDU — Add registers unsigned
// -----------------------------------------------------------
static void instr_ADDU(CPU* c, uint32_t ins)
{
    c->regs[RD(ins)] = c->regs[RS(ins)] + c->regs[RT(ins)];
}


// -----------------------------------------------------------
// SLTI — Set less-than immediate (signed)
// SLTIU — Set less-than immediate (unsigned)
// -----------------------------------------------------------
static void instr_SLTI(CPU* c, uint32_t ins)
{
    c->regs[RT(ins)] =
        ((int64_t)c->regs[RS(ins)] < SE16(IMM(ins))) ? 1 : 0;
}

static void instr_SLTIU(CPU* c, uint32_t ins)
{
    c->regs[RT(ins)] =
        (c->regs[RS(ins)] < (uint64_t)UIMM(ins)) ? 1 : 0;
}


// -----------------------------------------------------------
// LUI — Load upper immediate
// PROM uses this heavily when building addresses
// -----------------------------------------------------------
static void instr_LUI(CPU* c, uint32_t ins)
{
    c->regs[RT(ins)] = ((uint64_t)UIMM(ins)) << 16;
}


// -----------------------------------------------------------
// BEQ / BNE — PROM-critical branch instructions
// -----------------------------------------------------------
static void instr_BEQ(CPU* c, uint32_t ins)
{
    if (c->regs[RS(ins)] == c->regs[RT(ins)])
        c->nextPC = c->pc + 4 + ((int64_t)IMM(ins) << 2);
}

static void instr_BNE(CPU* c, uint32_t ins)
{
    if (c->regs[RS(ins)] != c->regs[RT(ins)])
        c->nextPC = c->pc + 4 + ((int64_t)IMM(ins) << 2);
}


// -----------------------------------------------------------
// AND / OR / XOR / NOR — PROM bit operations
// -----------------------------------------------------------
static void instr_AND(CPU* c, uint32_t ins)
{
    c->regs[RD(ins)] = c->regs[RS(ins)] & c->regs[RT(ins)];
}

static void instr_OR(CPU* c, uint32_t ins)
{
    c->regs[RD(ins)] = c->regs[RS(ins)] | c->regs[RT(ins)];
}

static void instr_XOR(CPU* c, uint32_t ins)
{
    c->regs[RD(ins)] = c->regs[RS(ins)] ^ c->regs[RT(ins)];
}

static void instr_NOR(CPU* c, uint32_t ins)
{
    c->regs[RD(ins)] = ~(c->regs[RS(ins)] | c->regs[RT(ins)]);
}


// -----------------------------------------------------------
// Shift operations
// -----------------------------------------------------------
static void instr_SLL(CPU* c, uint32_t ins)
{
    c->regs[RD(ins)] = c->regs[RT(ins)] << SA(ins);
}

static void instr_SRL(CPU* c, uint32_t ins)
{
    c->regs[RD(ins)] = c->regs[RT(ins)] >> SA(ins);
}

static void instr_SRA(CPU* c, uint32_t ins)
{
    c->regs[RD(ins)] = ((int64_t)c->regs[RT(ins)]) >> SA(ins);
}


// -----------------------------------------------------------
// LW / LB — PROM uses these constantly
// -----------------------------------------------------------
static void instr_LW(CPU* c, uint32_t ins)
{
    uint64_t addr = c->regs[RS(ins)] + SE16(IMM(ins));
    c->regs[RT(ins)] = c->load32_be(addr);
}

static void instr_LB(CPU* c, uint32_t ins)
{
    uint64_t addr = c->regs[RS(ins)] + SE16(IMM(ins));
    int8_t v = (int8_t)c->load8(addr);
    c->regs[RT(ins)] = (int64_t)v;
}


// -----------------------------------------------------------
// SW / SB — PROM writes console buffer + stack frames
// -----------------------------------------------------------
static void instr_SW(CPU* c, uint32_t ins)
{
    uint64_t addr = c->regs[RS(ins)] + SE16(IMM(ins));
    c->store32_be(addr, (uint32_t)c->regs[RT(ins)]);
}

static void instr_SB(CPU* c, uint32_t ins)
{
    uint64_t addr = c->regs[RS(ins)] + SE16(IMM(ins));
    c->store8(addr, (uint8_t)c->regs[RT(ins)]);
}


// -----------------------------------------------------------
// SYSCALL — PROM uses ARCS calls through SYSCALL instruction
// -----------------------------------------------------------
static void instr_SYSCALL(CPU* c, uint32_t)
{
    c->raise_syscall();
}


// -----------------------------------------------------------
// BREAK — PROM uses BREAK during debug traps
// -----------------------------------------------------------
static void instr_BREAK(CPU* c, uint32_t)
{
    c->raise_break();
}


// -----------------------------------------------------------
// PART 4 END
// paste code here in Part 5
// -----------------------------------------------------------
// -----------------------------------------------------------
// cpu.cpp  (Speedracer SGI Octane Emulator)
// Part 5 — IRIX Required Instructions (HI/LO, MULT/DIV, REGIMM)
// -----------------------------------------------------------
//
// IRIX loader (sash, fx) and early kernel bootstrap require:
//
// • MULT / MULTU
// • DIV / DIVU
// • MFHI / MFLO
// • MTHI / MTLO
// • SLT / SLTU
// • BLTZ / BGEZ (+AL variants)
// • NOP (SLL r0,r0,0)
// -----------------------------------------------------------


// -----------------------------------------------------------
// NOP — Encoded as SLL r0,r0,0
// -----------------------------------------------------------
static void instr_NOP(CPU* c, uint32_t)
{
    // Do nothing
}


// -----------------------------------------------------------
// SLT / SLTU — Required before kernel boot
// -----------------------------------------------------------
static void instr_SLT(CPU* c, uint32_t ins)
{
    c->regs[RD(ins)] =
        ((int64_t)c->regs[RS(ins)] < (int64_t)c->regs[RT(ins)]) ? 1 : 0;
}

static void instr_SLTU(CPU* c, uint32_t ins)
{
    c->regs[RD(ins)] =
        (c->regs[RS(ins)] < c->regs[RT(ins)]) ? 1 : 0;
}


// -----------------------------------------------------------
// HI / LO register access
// -----------------------------------------------------------
static void instr_MFHI(CPU* c, uint32_t ins)
{
    c->regs[RD(ins)] = c->hi;
}

static void instr_MFLO(CPU* c, uint32_t ins)
{
    c->regs[RD(ins)] = c->lo;
}

static void instr_MTHI(CPU* c, uint32_t ins)
{
    c->hi = c->regs[RS(ins)];
}

static void instr_MTLO(CPU* c, uint32_t ins)
{
    c->lo = c->regs[RS(ins)];
}


// -----------------------------------------------------------
// MULT — Signed multiply (IRIX loader uses it)
// -----------------------------------------------------------
static void instr_MULT(CPU* c, uint32_t ins)
{
    int64_t a = (int64_t)c->regs[RS(ins)];
    int64_t b = (int64_t)c->regs[RT(ins)];
    __int128 r = (__int128)a * (__int128)b;
    c->lo = (uint64_t)(r & 0xFFFFFFFFFFFFFFFFULL);
    c->hi = (uint64_t)(r >> 64);
}


// -----------------------------------------------------------
// MULTU — Unsigned multiply
// -----------------------------------------------------------
static void instr_MULTU(CPU* c, uint32_t ins)
{
    uint64_t a = c->regs[RS(ins)];
    uint64_t b = c->regs[RT(ins)];
    __uint128_t r = (__uint128_t)a * (__uint128_t)b;
    c->lo = (uint64_t)(r & 0xFFFFFFFFFFFFFFFFULL);
    c->hi = (uint64_t)(r >> 64);
}


// -----------------------------------------------------------
// DIV / DIVU — Early kernel uses DIV for modulo and hash functions
// -----------------------------------------------------------
static void instr_DIV(CPU* c, uint32_t ins)
{
    int64_t a = (int64_t)c->regs[RS(ins)];
    int64_t b = (int64_t)c->regs[RT(ins)];
    if (b != 0) {
        c->lo = a / b;
        c->hi = a % b;
    }
}

static void instr_DIVU(CPU* c, uint32_t ins)
{
    uint64_t a = c->regs[RS(ins)];
    uint64_t b = c->regs[RT(ins)];
    if (b != 0) {
        c->lo = a / b;
        c->hi = a % b;
    }
}


// -----------------------------------------------------------
// REGIMM Branches — IRIX requires all of these
// -----------------------------------------------------------
//
// 0x01 opcode → uses RT field:
//
// RT = 0x00 → BLTZ
// RT = 0x01 → BGEZ
// RT = 0x10 → BLTZAL
// RT = 0x11 → BGEZAL
//
// -----------------------------------------------------------

static void instr_BLTZ(CPU* c, uint32_t ins)
{
    if ((int64_t)c->regs[RS(ins)] < 0)
        c->nextPC = c->pc + 4 + ((int64_t)IMM(ins) << 2);
}

static void instr_BGEZ(CPU* c, uint32_t ins)
{
    if ((int64_t)c->regs[RS(ins)] >= 0)
        c->nextPC = c->pc + 4 + ((int64_t)IMM(ins) << 2);
}

static void instr_BLTZAL(CPU* c, uint32_t ins)
{
    if ((int64_t)c->regs[RS(ins)] < 0) {
        c->regs[31] = c->pc + 4;  // link
        c->nextPC = c->pc + 4 + ((int64_t)IMM(ins) << 2);
    }
}

static void instr_BGEZAL(CPU* c, uint32_t ins)
{
    if ((int64_t)c->regs[RS(ins)] >= 0) {
        c->regs[31] = c->pc + 4;  // link
        c->nextPC = c->pc + 4 + ((int64_t)IMM(ins) << 2);
    }
}


// -----------------------------------------------------------
// Now map REGIMM instructions into decode table
// (But not in this part — done in Part 6 with decode setup)
// -----------------------------------------------------------


// -----------------------------------------------------------
// PART 5 END
// paste code here in Part 6
// -----------------------------------------------------------
