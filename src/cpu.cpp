// -----------------------------------------------------------
// cpu.cpp  (Racer SGI Octane Emulator)
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
// cpu.cpp  (Racer SGI Octane Emulator)
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
// cpu.cpp  (Racer SGI Octane Emulator)
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
// cpu.cpp  (Racer SGI Octane Emulator)
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
// cpu.cpp  (Racer SGI Octane Emulator)
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
// -----------------------------------------------------------
// cpu.cpp (Racer SGI Octane Emulator)
// Part 7 — CPU Execution Loop (stepOnce, PC update, delay slots)
// -----------------------------------------------------------
//
// MIPS R10000 uses delayed branches. PROM and IRIX both depend
// on correct behavior of:
//
//   1. Execute instruction at PC
//   2. If branch, execute *next* instruction as delay slot
//   3. After delay slot, jump to target
//
// This model is cycle-approximate, not cycle-accurate.
// -----------------------------------------------------------


// -----------------------------------------------------------
// Step ONE instruction
// -----------------------------------------------------------
void CPU::stepOnce()
{
    // Fetch next instruction
    uint32_t ins = load32_be(pc);

    // The instruction after this one
    uint64_t oldPC   = pc;
    uint64_t oldNext = nextPC;

    // Default nextPC = PC + 4
    nextPC = pc + 4;

    // Execute instruction (does NOT update PC)
    decode_and_execute(ins);

    // Enforce register $0 = 0
    regs[0] = 0;

    // ------------------------------------------
    // Handle delayed branch slot
    // ------------------------------------------
    //
    // If decode_and_execute() changed nextPC away from pc+4,
    // then this instruction is a branch/jump.
    //
    // Behavior:
    //   • Execute delay-slot instruction from pc+4
    //   • Then pc = nextPC computed by branch
    //
    // ------------------------------------------
    if (nextPC != oldPC + 4)
    {
        uint64_t branchTarget = nextPC;

        // Execute delay slot instruction (one instruction)
        uint32_t delayIns = load32_be(oldPC + 4);
        decode_and_execute(delayIns);

        // Enforce $0 again
        regs[0] = 0;

        // Now jump to branch target
        pc     = branchTarget;
        nextPC = pc + 4;
        return;
    }

    // ------------------------------------------
    // No branch → normal sequential execution
    // ------------------------------------------
    pc = nextPC;
}


// -----------------------------------------------------------
// PART 7 END
// paste code here in Part 8
// -----------------------------------------------------------
// -----------------------------------------------------------
// cpu.cpp (Racer SGI Octane Emulator)
// Part 8 — Basic timing + MULT/DIV latency + CP0 integration
// -----------------------------------------------------------
//
// This part adds:
//
// • Basic cycle counter
// • MULT / DIV latency handling
// • CP0 EPC integration for exceptions
// • Safe step() wrapper for whole emulator
//
// NOT included yet:
// • Full R10000 out-of-order pipeline (future Part 10+)
//
// -----------------------------------------------------------


// -----------------------------------------------------------
// Cycle counting
// -----------------------------------------------------------

void CPU::addCycles(uint32_t c)
{
    cycles += c;
}


// -----------------------------------------------------------
// MULT/DIV latency counters
// -----------------------------------------------------------
//
// True R10000 latencies:
//   MULT  →  4 cycles
//   DIV   → 35 cycles
//
// We simplify to match PROM + IRIX expectations.
//
// -----------------------------------------------------------

void CPU::tick_mult_div()
{
    if (multBusy > 0)
        multBusy--;

    if (divBusy > 0)
        divBusy--;
}


// -----------------------------------------------------------
// Overwrite MULT/MULTU to add latency
// Called automatically by decode table logic
// -----------------------------------------------------------

static void instr_MULT_latency(CPU* c, uint32_t ins)
{
    instr_MULT(c, ins);
    c->multBusy = 4;   // Simplified latency
}

static void instr_MULTU_latency(CPU* c, uint32_t ins)
{
    instr_MULTU(c, ins);
    c->multBusy = 4;
}

static void instr_DIV_latency(CPU* c, uint32_t ins)
{
    instr_DIV(c, ins);
    c->divBusy = 35;   // Simplified safe latency
}

static void instr_DIVU_latency(CPU* c, uint32_t ins)
{
    instr_DIVU(c, ins);
    c->divBusy = 35;
}


// -----------------------------------------------------------
// Check if HI/LO reg is available
// PROM and IRIX depend on this behavior
// -----------------------------------------------------------
bool CPU::hiloBusy() const
{
    return (multBusy > 0) || (divBusy > 0);
}


// -----------------------------------------------------------
// Exception + CP0 integration
// -----------------------------------------------------------

void CPU::handle_exception(int code)
{
    // CP0 must calculate EPC
    if (cp0)
        cp0->raise_exception(code, pc);

    // Jump to general exception vector
    // PROM uses bootstrap vector 0x1fc00380
    pc     = 0x1FC00380ULL;
    nextPC = pc + 4;
}


// -----------------------------------------------------------
// CPU step - including latency tick
// -----------------------------------------------------------

void CPU::step()
{
    // Tick MULT/DIV latency
    tick_mult_div();

    // Execute a single instruction
    stepOnce();

    // Add cycle for this instruction
    addCycles(1);
}


// -----------------------------------------------------------
// PART 8 END
// paste code here in Part 9
// -----------------------------------------------------------
// -----------------------------------------------------------
// cpu.cpp (Racer SGI Octane Emulator)
// Part 9 — CP0 / COP0 integration (MFC0, MTC0, ERET)
// -----------------------------------------------------------
//
// This part implements minimal COP0 support required for PROM/IRIX:
//  - MFC0 (move from CP0)  : COP0, rs == 0
//  - MTC0 (move to CP0)    : COP0, rs == 4
//  - ERET (exception return): canonical opcode 0x42000018
//
// Behavior notes:
//  - MFC0: RT <- CP0[rd]
//  - MTC0: CP0[rd] <- RT
//  - ERET: clear EXL in CP0 status and set PC = CP0.EPC
//
// These are conservative implementations sufficient for early IRIX:
// -----------------------------------------------------------


static void instr_COP0(CPU* c, uint32_t ins)
{
    // If the instruction equals canonical ERET encoding, handle it.
    // ERET common encoding: 0x42000018
    if (ins == 0x42000018u) {
        // ERET: clear EXL and set PC = EPC
        if (c->cp0) {
            uint64_t epc = c->cp0->read_reg(14); // EPC reg
            // Clear EXL bit (bit 1) in Status (reg 12)
            uint64_t status = c->cp0->read_reg(12);
            status &= ~(1ULL << 1);
            c->cp0->write_reg(12, status);
            // Jump to EPC
            c->nextPC = epc;
            return;
        } else {
            // No CP0 connected; treat like NOP or raise exception
            std::cerr << "[CPU] ERET executed but CP0 missing\n";
            return;
        }
    }

    // Otherwise decode MFC0 / MTC0 by rs field
    uint32_t rs = RS(ins);
    uint32_t rt = RT(ins);
    uint32_t rd = RD(ins);

    if (rs == 0x00) {
        // MFC0: move from CP0 register rd -> GPR rt
        if (!c->cp0) {
            std::cerr << "[CPU] MFC0 but CP0 missing\n";
            c->regs[rt] = 0;
            return;
        }
        uint64_t v = c->cp0->read_reg(rd);
        c->regs[rt] = v;
        return;
    }

    if (rs == 0x04) {
        // MTC0: move GPR rt -> CP0 register rd
        if (!c->cp0) {
            std::cerr << "[CPU] MTC0 but CP0 missing\n";
            return;
        }
        c->cp0->write_reg(rd, c->regs[rt]);
        return;
    }

    // Unhandled COP0 sub-op: log and ignore
    std::cerr << "[CPU] Unhandled COP0 ins rs=0x" << std::hex << rs
              << " rt=0x" << rt << " rd=0x" << rd << " ins=0x"
              << ins << std::dec << "\n";
}


// Patch the primary table entry for COP0 (opcode 0x10).
// We do this here so init_decode_tables() doesn't have to know about COP0
// (but if init_decode_tables() runs again, it will be overridden by it).
static void ensure_cop0_mapped()
{
    // primary_table is defined in Part 6
    extern instr_func primary_table[];
    primary_table[0x10] = instr_COP0;
}

// Make sure COP0 is mapped at CPU construction time.
// We add a small shim: if init_decode_tables already called, ensure mapping here.
// If CPU constructor calls init_decode_tables (it does), calling this function
// again will simply overwrite primary_table[0x10] -> instr_COP0.
struct _EnsureCOP0Mapped {
    _EnsureCOP0Mapped() { ensure_cop0_mapped(); }
} _ensure_cop0_mapped_instance;


// -----------------------------------------------------------
// PART 9 END
// paste code here in Part 10
// -----------------------------------------------------------
// -----------------------------------------------------------
// cpu.cpp (Racer SGI Octane Emulator)
// Part 10 — Full CP0 Exception Flow (EPC, Status, Cause)
// -----------------------------------------------------------
//
// REQUIRED FOR:
//   • PROM exception jumps
//   • Early IRIX kernel exception handling
//   • Proper EPC return with ERET
//   • Correct EXL handling
//
// This implements:
//   CPU::enter_exception()
//   CPU::raise_exception()
//   CP0 Status / Cause updates
//
// -----------------------------------------------------------


// -----------------------------------------------------------
// Helper: push CPU into exception state
// -----------------------------------------------------------
void CPU::enter_exception(int code, uint64_t badPC)
{
    if (!cp0) {
        std::cerr << "[CPU] enter_exception() but CP0 missing\n";
        return;
    }

    // Read status, cause, epc from CP0
    uint64_t status = cp0->read_reg(12); // Status
    uint64_t cause  = cp0->read_reg(13); // Cause

    // -------------------------------------------------------
    // If EXL is already set, EPC must not be overwritten
    // -------------------------------------------------------
    bool exl = (status >> 1) & 1;

    if (!exl) {
        // First exception → write EPC = PC at exception time
        cp0->write_reg(14, badPC); // EPC
    }

    // -------------------------------------------------------
    // Set EXL = 1 (bit 1)
    // -------------------------------------------------------
    status |= (1ULL << 1);
    cp0->write_reg(12, status);

    // -------------------------------------------------------
    // Cause.ExcCode = exception code (bits 6..2)
    // -------------------------------------------------------
    cause &= ~0x7C;           // Clear ExcCode field
    cause |= ((code & 0x1F) << 2);
    cp0->write_reg(13, cause);

    // -------------------------------------------------------
    // Jump to exception vector:
    // PROM uses: 0x1FC00380
    // IRIX kernel uses BEV=0 bit to choose different vector
    // -------------------------------------------------------
    uint64_t vector = 0x1FC00380;

    // If BEV bit (Status bit 22) is set → use alternate vector
    if (status & (1ULL << 22)) {
        vector = 0x1FC00000;  // Boot exception vector
    }

    pc     = vector;
    nextPC = vector + 4;
}


// -----------------------------------------------------------
// Wrapper: raise an exception by code
// -----------------------------------------------------------
void CPU::raise_exception(int code)
{
    enter_exception(code, pc);
}


// -----------------------------------------------------------
// CP0 integrated raise_exception for SYSCALL/BREAK/address errors
// -----------------------------------------------------------

void CPU::raise_syscall()
{
    raise_exception(8); // ExcCode 8 = Syscall
}

void CPU::raise_break()
{
    raise_exception(9); // ExcCode 9 = Breakpoint
}

void CPU::address_error()
{
    raise_exception(4); // ExcCode 4 = Address error (load)
}


// -----------------------------------------------------------
// PART 10 END
// paste code here in Part 11
// -----------------------------------------------------------
// -----------------------------------------------------------
// cpu.cpp (Racer SGI Octane Emulator)
// Part 11 — TLB Operations (TLBR, TLBWI, TLBWR, TLBP)
// -----------------------------------------------------------
//
// Implements all CP0 TLB instructions needed by IRIX:
//
//   • TLBR  – read indexed TLB entry into CP0 registers
//   • TLBWI – write CP0 registers into TLB at Index
//   • TLBWR – write CP0 registers into TLB at Random
//   • TLBP  – probe TLB for matching entry and set Index
//
// MMU performs actual address translation.
// CPU triggers TLB refill exceptions here when necessary.
//
// -----------------------------------------------------------


// The COP0 opcode for all TLB ops: 0x10 with rs = 0x10, funct field decides.
//
// Encoding:
//   opcode = 0x10
//   rs     = 0x10
//   funct:
//       0x01 = TLBR
//       0x02 = TLBWI
//       0x06 = TLBWR
//       0x08 = TLBP
//
// IRIX kernel uses all four during boot.
// -----------------------------------------------------------


static void instr_TLBR(CPU* c)
{
    if (!c->mmu || !c->cp0) {
        std::cerr << "[CPU] TLBR but MMU/CP0 missing\n";
        return;
    }
    c->mmu->tlbr(c->cp0); // read TLB into CP0.EntryHi/EntryLo registers
}

static void instr_TLBWI(CPU* c)
{
    if (!c->mmu || !c->cp0) {
        std::cerr << "[CPU] TLBWI but MMU/CP0 missing\n";
        return;
    }
    c->mmu->tlbwi(c->cp0); // write CP0.EntryHi/EntryLo into TLB index
}

static void instr_TLBWR(CPU* c)
{
    if (!c->mmu || !c->cp0) {
        std::cerr << "[CPU] TLBWR but MMU/CP0 missing\n";
        return;
    }
    c->mmu->tlbwr(c->cp0); // write CP0.EntryHi/EntryLo into TLB random slot
}

static void instr_TLBP(CPU* c)
{
    if (!c->mmu || !c->cp0) {
        std::cerr << "[CPU] TLBP but MMU/CP0 missing\n";
        return;
    }
    c->mmu->tlbp(c->cp0); // probe TLB and set Index
}


// -----------------------------------------------------------
// Extend COP0 handler to include TLB ops
// -----------------------------------------------------------
static void instr_COP0_extended(CPU* c, uint32_t ins)
{
    uint32_t rs = RS(ins);
    uint32_t funct = FUNCT(ins);

    // TLB operations: rs=0x10 (COP0 TLB function group)
    if (rs == 0x10)
    {
        switch (funct)
        {
            case 0x01: instr_TLBR(c);  return;
            case 0x02: instr_TLBWI(c); return;
            case 0x06: instr_TLBWR(c); return;
            case 0x08: instr_TLBP(c);  return;
        }
    }

    // Otherwise fall back to regular COP0 logic (MFC0/MTC0/ERET)
    instr_COP0(c, ins);
}


// -----------------------------------------------------------
// Hook extended COP0 handler into primary opcode table
// -----------------------------------------------------------
static void ensure_cop0_extended()
{
    extern instr_func primary_table[];
    primary_table[0x10] = instr_COP0_extended;
}

// Auto-run mapping on startup
struct _EnsureCOP0Ext {
    _EnsureCOP0Ext() { ensure_cop0_extended(); }
} _ensure_cop0extended_instance;



// -----------------------------------------------------------
// TLB Refill Exception Trigger Helpers
// -----------------------------------------------------------
//
// These are called by mmu when accessing unmapped addresses.
// CPU must enter exception vector TLBL or TLBS.
//
// IRIX expects:
//   TLBL = 2
//   TLBS = 3
// -----------------------------------------------------------

void CPU::raise_tlbl(uint64_t addr)
{
    // BadVAddr = the address that caused fault
    if (cp0) cp0->write_reg(8, addr);

    raise_exception(2); // TLBL
}

void CPU::raise_tlbs(uint64_t addr)
{
    if (cp0) cp0->write_reg(8, addr);

    raise_exception(3); // TLBS
}


// -----------------------------------------------------------
// PART 11 END
// paste code here in Part 12
// -----------------------------------------------------------
// -----------------------------------------------------------
// cpu.cpp (Racer SGI Octane Emulator)
// Part 12 — CPU <-> MMU Glue (Address Translation, Segments)
// -----------------------------------------------------------
//
// This part defines:
//   • CPU::mmu_read8/16/32 and mmu_write8/16/32
//   • Segment rules (KUSEG/KSEG0/KSEG1/KSEG2)
//   • Hooks for TLB exceptions (TLBL/TLBS)
//   • Integration with Part 11's TLB refill exception helpers
//
// IRIX heavily depends on correct segment layout:
//
//   0x00000000 - 0x7FFFFFFF   KUSEG   (TLB mapped, user mode)
//   0x80000000 - 0x9FFFFFFF   KSEG0   (mapped, cached, direct phys)
//   0xA0000000 - 0xBFFFFFFF   KSEG1   (mapped, uncached, direct phys)
//   0xC0000000 - 0xFFFFFFFF   KSEG2   (TLB mapped, kernel mode)
//
// The PROM (IP30) always lives in KSEG1 unmapped region.
//
// -----------------------------------------------------------


// -----------------------------------------------------------
// Convert virtual → physical addressing
//   This calls mmu->translate() but also handles KSEG0/KSEG1
// -----------------------------------------------------------
bool CPU::translate_address(uint64_t vaddr, uint64_t& paddr, bool write)
{
    // KSEG0: 0x80000000 - 0x9FFFFFFF  → direct physical, cached
    if (vaddr >= 0x80000000ULL && vaddr <= 0x9FFFFFFFULL)
    {
        paddr = vaddr - 0x80000000ULL;
        return true;
    }

    // KSEG1: 0xA0000000 - 0xBFFFFFFF  → direct physical, uncached
    if (vaddr >= 0xA0000000ULL && vaddr <= 0xBFFFFFFFULL)
    {
        paddr = vaddr - 0xA0000000ULL;
        return true;
    }

    // Otherwise use MMU TLB translation
    if (!mmu)
    {
        std::cerr << "[CPU] MMU missing for mapped address\n";
        return false;
    }

    int res = mmu->translate(vaddr, paddr, write);

    // TLB MISS
    if (res == MMU::TLB_MISS)
    {
        if (write)
            raise_tlbs(vaddr);
        else
            raise_tlbl(vaddr);
        return false;
    }

    // TLB INVALID
    if (res == MMU::TLB_INVALID)
    {
        if (write)
            raise_tlbs(vaddr);
        else
            raise_tlbl(vaddr);
        return false;
    }

    // TLB OK
    return true;
}


// -----------------------------------------------------------
// CPU-side MMU read/write helpers
// -----------------------------------------------------------

uint8_t CPU::mmu_read8(uint64_t vaddr)
{
    uint64_t paddr;
    if (!translate_address(vaddr, paddr, false))
        return 0;

    return memory->read8(paddr);
}

uint16_t CPU::mmu_read16(uint64_t vaddr)
{
    uint64_t paddr;
    if (!translate_address(vaddr, paddr, false))
        return 0;

    return memory->read16(paddr);
}

uint32_t CPU::mmu_read32(uint64_t vaddr)
{
    uint64_t paddr;
    if (!translate_address(vaddr, paddr, false))
        return 0;

    return memory->read32(paddr);
}


void CPU::mmu_write8(uint64_t vaddr, uint8_t val)
{
    uint64_t paddr;
    if (!translate_address(vaddr, paddr, true))
        return;

    memory->write8(paddr, val);
}

void CPU::mmu_write16(uint64_t vaddr, uint16_t val)
{
    uint64_t paddr;
    if (!translate_address(vaddr, paddr, true))
        return;

    memory->write16(paddr, val);
}

void CPU::mmu_write32(uint64_t vaddr, uint32_t val)
{
    uint64_t paddr;
    if (!translate_address(vaddr, paddr, true))
        return;

    memory->write32(paddr, val);
}


// -----------------------------------------------------------
// Override load/store helpers from earlier parts
// -----------------------------------------------------------
uint32_t CPU::load32_be(uint64_t addr)
{
    uint32_t v = mmu_read32(addr);
    return __builtin_bswap32(v);
}

uint16_t CPU::load16_be(uint64_t addr)
{
    uint16_t v = mmu_read16(addr);
    return __builtin_bswap16(v);
}

uint8_t CPU::load8(uint64_t addr)
{
    return mmu_read8(addr);
}

void CPU::store32_be(uint64_t addr, uint32_t val)
{
    mmu_write32(addr, __builtin_bswap32(val));
}

void CPU::store16_be(uint64_t addr, uint16_t val)
{
    mmu_write16(addr, __builtin_bswap16(val));
}

void CPU::store8(uint64_t addr, uint8_t val)
{
    mmu_write8(addr, val);
}


// -----------------------------------------------------------
// PART 12 END
// paste code here in Part 13
// -----------------------------------------------------------
// -----------------------------------------------------------
// cpu.cpp (Racer SGI Octane Emulator)
// Part 13 — Final plumbing: memory attach, run loop, logging
// -----------------------------------------------------------
//
// This final part completes the CPU implementation by:
//  - providing attach_memory()
//  - providing a convenience connect_all()
//  - implementing run() loop
//  - adding debug dump functions
//  - safety checks and destructor
//
// Logs use the emulator name "Racer" (not "Speedracer").
// -----------------------------------------------------------

#include "memory.h"

// -----------------------------------------------------------
// Attach Memory pointer (used by MMU glue in Part 12)
// -----------------------------------------------------------
void CPU::attach_memory(Memory* m)
{
    memory = m;
}

// -----------------------------------------------------------
// Convenience: attach all subsystems at once
// Emulator should call this when wiring components.
// -----------------------------------------------------------
void CPU::connect_all(MMU* m, CP0* c, Memory* memptr)
{
    mmu = m;
    cp0 = c;
    memory = memptr;

    // Ensure CP0 and MMU are aware of CPU where needed
    if (cp0) cp0->attach_cpu(this);

    // Initialize decode tables if not already done
    init_decode_tables();
}

// -----------------------------------------------------------
// CPU run loop: run 'n' instructions (or until halted)
// Uses step_with_exceptions() which handles MUL/DIV timing.
// -----------------------------------------------------------
void CPU::run(uint64_t instr_count)
{
    std::cout << "[Racer][CPU] Starting run: " << instr_count << " instructions\n";
    for (uint64_t i = 0; i < instr_count; ++i)
    {
        step_with_exceptions();

        // Optional: allow external break/halt points in future
        if (halted) {
            std::cout << "[Racer][CPU] halted at PC=0x" << std::hex << pc << std::dec << "\n";
            break;
        }
    }
    std::cout << "[Racer][CPU] Run complete. cycles=" << cycles << "\n";
}

// -----------------------------------------------------------
// Debug helpers
// -----------------------------------------------------------
void CPU::dump_regs()
{
    std::cout << "[Racer][CPU] Register file dump:\n";
    for (int i = 0; i < 32; i += 4)
    {
        printf("r%02d: 0x%016llx  r%02d: 0x%016llx  r%02d: 0x%016llx  r%02d: 0x%016llx\n",
               i,  (unsigned long long)regs[i],
               i+1,(unsigned long long)regs[i+1],
               i+2,(unsigned long long)regs[i+2],
               i+3,(unsigned long long)regs[i+3]);
    }
    std::cout << "HI: 0x" << std::hex << hi << "  LO: 0x" << lo << std::dec << "\n";
    std::cout << "PC: 0x" << std::hex << pc << "  nextPC: 0x" << nextPC << std::dec << "\n";
}

void CPU::dump_state()
{
    std::cout << "[Racer][CPU] State dump:\n";
    dump_regs();
    if (cp0) {
        std::cout << "[Racer][CPU] CP0 registers (partial):\n";
        std::cout << " Status: 0x" << std::hex << cp0->read_reg(12) << std::dec << "\n";
        std::cout << " Cause : 0x" << std::hex << cp0->read_reg(13) << std::dec << "\n";
        std::cout << " EPC   : 0x" << std::hex << cp0->read_reg(14) << std::dec << "\n";
    } else {
        std::cout << "[Racer][CPU] CP0 not attached\n";
    }
}

// -----------------------------------------------------------
// Destructor
// -----------------------------------------------------------
CPU::~CPU()
{
    // Nothing to free; owned resources are managed by Emulator
}

// -----------------------------------------------------------
// Final marker for CPU file assembly
// -----------------------------------------------------------
 // paste code here in Part 14

