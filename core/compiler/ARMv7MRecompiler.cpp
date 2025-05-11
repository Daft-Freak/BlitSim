#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <utility>

#ifdef __linux__
#include <sys/mman.h>
#include <unistd.h>
#elif defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

#include "ARMv7MRecompiler.h"

#include "ARMv7MCore.h"
#include "MemoryBus.h"

// reg 0 is special tmp register
enum class GenReg
{
    Temp = 0, // special temp

    R0,
    R1,
    R2,
    R3,
    R4,
    R5,
    R6,
    R7,
    R8,
    R9,
    R10,
    R11,
    R12,
    R13,
    R14,
    // R15 = PC

    CPSR,

    Control,
    PriMask,

    Temp2, // used by POP, LDM
};

// flag bits (need to match cpu)
constexpr int preserveV = (1u << 28) >> 28;
constexpr int preserveC = (1u << 29) >> 28;

constexpr int writeV = (1u << 28) >> 24;
constexpr int writeC = (1u << 29) >> 24;
constexpr int writeZ = (1u << 30) >> 24;
constexpr int writeN = (1u << 31) >> 24;

// opcode building helpers
// these would be generic if they didn't use GenReg
inline GenOpInfo loadImm(uint32_t imm, int cycles = 0)
{
    GenOpInfo ret{};
    ret.opcode = GenOpcode::LoadImm;
    ret.cycles = cycles;
    ret.imm = imm;

    return ret;
}

inline GenOpInfo move(GenReg src, GenReg dst, int cycles = 0)
{
    GenOpInfo ret{};
    ret.opcode = GenOpcode::Move;
    ret.cycles = cycles;
    ret.src[0] = static_cast<uint8_t>(src);
    ret.dst[0] = static_cast<uint8_t>(dst);

    return ret;
}

inline GenOpInfo load(int size, GenReg addr, GenReg dst, int cycles = 0)
{
    GenOpInfo ret{};
    if(size == 1)
        ret.opcode = GenOpcode::Load;
    else if(size == 2)
        ret.opcode = GenOpcode::Load2;
    else if(size == 4)
        ret.opcode = GenOpcode::Load4;

    ret.cycles = cycles;
    ret.src[0] = static_cast<uint8_t>(addr);
    ret.dst[0] = static_cast<uint8_t>(dst);

    return ret;
}

inline GenOpInfo store(int size, GenReg addr, GenReg data, int cycles = 0)
{
    GenOpInfo ret{};
    if(size == 1)
        ret.opcode = GenOpcode::Store;
    else if(size == 2)
        ret.opcode = GenOpcode::Store2;
    else if(size == 4)
        ret.opcode = GenOpcode::Store4;

    ret.cycles = cycles;
    ret.src[0] = static_cast<uint8_t>(addr);
    ret.src[1] = static_cast<uint8_t>(data);

    return ret;
}

inline GenOpInfo alu(GenOpcode op, GenReg src0, GenReg src1, GenReg dst, int cycles = 0)
{
    GenOpInfo ret{};
    ret.opcode = op;
    ret.cycles = cycles;
    ret.src[0] = static_cast<uint8_t>(src0);
    ret.src[1] = static_cast<uint8_t>(src1);
    ret.dst[0] = static_cast<uint8_t>(dst);

    return ret;
}

inline GenOpInfo alu(GenOpcode op, GenReg src0, GenReg dst, int cycles = 0)
{
    GenOpInfo ret{};
    ret.opcode = op;
    ret.cycles = cycles;
    ret.src[0] = static_cast<uint8_t>(src0);
    ret.dst[0] = static_cast<uint8_t>(dst);

    return ret;
}

inline GenOpInfo compare(GenReg src0, GenReg src1, int cycles = 0)
{
    GenOpInfo ret{};
    ret.opcode = GenOpcode::Compare;
    ret.cycles = cycles;
    ret.src[0] = static_cast<uint8_t>(src0);
    ret.src[1] = static_cast<uint8_t>(src1);

    return ret;
}

inline GenOpInfo jump(GenCondition cond = GenCondition::Always, GenReg src = GenReg::Temp, int cycles = 0)
{
    GenOpInfo ret{};
    ret.opcode = GenOpcode::Jump;
    ret.cycles = cycles;
    ret.src[0] = static_cast<uint8_t>(cond);
    ret.src[1] = static_cast<uint8_t>(src);

    return ret;
}

// helper to add instruction to block
static void addInstruction(GenBlockInfo &genBlock, GenOpInfo op, uint8_t len = 0, uint16_t flags = 0)
{
    if(flags & GenOp_Exit)
        assert(op.opcode == GenOpcode::Jump);

    op.len = len;
    op.flags = flags;
    genBlock.instructions.emplace_back(std::move(op));
}

// common patterns
void loadWithOffset(GenBlockInfo &genBlock, int size, GenReg base, GenReg offset, int shift, GenReg dst, int len = 0, int flags = 0)
{
    // base + off << shift
    // TODO: optimise shift == 0?
    addInstruction(genBlock, loadImm(shift));
    addInstruction(genBlock, alu(GenOpcode::ShiftLeft, offset, GenReg::Temp, GenReg::Temp));
    addInstruction(genBlock, alu(GenOpcode::Add, base, GenReg::Temp, GenReg::Temp));
    
    addInstruction(genBlock, load(size, GenReg::Temp, dst), len, flags);
}

void loadWithOffset(GenBlockInfo &genBlock, int size, GenReg base, int offset, bool writeback, bool index, GenReg dst, int len = 0, int flags = 0)
{
    if(index)
    {
        addInstruction(genBlock, loadImm(offset));
        addInstruction(genBlock, alu(GenOpcode::Add, base, GenReg::Temp, GenReg::Temp));
    }

    addInstruction(genBlock, load(size, index ? GenReg::Temp : base, dst), writeback ? 0 : len, flags);

    // write back adjusted base
    if(writeback)
    {
        // need to redo add even if index is true (can't reuse the temp)
        addInstruction(genBlock, loadImm(offset));
        addInstruction(genBlock, alu(GenOpcode::Add, base, GenReg::Temp, base), len);
    }
}

void loadWithOffset(GenBlockInfo &genBlock, int size, GenReg base, int offset, GenReg dst, int cycles, int len = 0, int flags = 0)
{
    loadWithOffset(genBlock, size, base, offset, false, offset != 0, dst, len, flags);
}

void storeWithOffset(GenBlockInfo &genBlock, int size, GenReg base, int offset, GenReg dst, int cycles, int len = 0, int flags = 0)
{
    if(offset)
    {
        addInstruction(genBlock, loadImm(offset));
        addInstruction(genBlock, alu(GenOpcode::Add, base, GenReg::Temp, GenReg::Temp));
    }

    addInstruction(genBlock, store(size, offset ? GenReg::Temp : base, dst, cycles), len, flags);
}

// register mapping
uint16_t getRegOffset(void *cpuPtr, uint8_t reg)
{
    auto cpu = reinterpret_cast<ARMv7MCore *>(cpuPtr);

    auto cpuPtrInt = reinterpret_cast<uintptr_t>(cpu);

    auto genReg = static_cast<GenReg>(reg);

    if(reg < 16)
    {
        uint16_t regsOffset = reinterpret_cast<uintptr_t>(&cpu->regs) - cpuPtrInt;

        auto mapped = cpu->mapReg(static_cast<ARMv7MCore::Reg>(reg - 1));

        return regsOffset + static_cast<int>(mapped) * 4;
    }

    if(genReg == GenReg::Control)
        return reinterpret_cast<uintptr_t>(&cpu->control) - cpuPtrInt;
    if(genReg == GenReg::PriMask)
        return reinterpret_cast<uintptr_t>(&cpu->primask) - cpuPtrInt;

    assert(!"invalid register!");
    return 0;
}

ARMv7MRecompiler::ARMv7MRecompiler(ARMv7MCore &cpu) : cpu(cpu)
{
    SourceInfo sourceInfo{};

    auto cpuPtrInt = reinterpret_cast<uintptr_t>(&cpu);

    uint16_t regsOffset = reinterpret_cast<uintptr_t>(&cpu.regs) - cpuPtrInt;

    sourceInfo.registers.emplace_back(SourceRegInfo{"tmp", 32, SourceRegType::Temp, 0xFFFF});

    // main regs
    sourceInfo.registers.emplace_back(SourceRegInfo{"R0 ", 32, SourceRegType::General, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R1 ", 32, SourceRegType::General, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R2 ", 32, SourceRegType::General, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R3 ", 32, SourceRegType::General, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R4 ", 32, SourceRegType::General, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R5 ", 32, SourceRegType::General, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R6 ", 32, SourceRegType::General, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R7 ", 32, SourceRegType::General, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R8 ", 32, SourceRegType::General, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R9 ", 32, SourceRegType::General, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R10", 32, SourceRegType::General, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R11", 32, SourceRegType::General, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R12", 32, SourceRegType::General, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R13", 32, SourceRegType::General, 0xFFFF}); // SP (may require mapping)
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R14", 32, SourceRegType::General, regsOffset}); // LR
    // R15 is PC

    regsOffset = reinterpret_cast<uintptr_t>(&cpu.cpsr) - cpuPtrInt;
    sourceInfo.registers.emplace_back(SourceRegInfo{"PSR", 32, SourceRegType::Flags, regsOffset});

    // CONTROL/PRIMASK (3 chars is not helping here)
    sourceInfo.registers.emplace_back(SourceRegInfo{"CTL", 32, SourceRegType::General, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"PMK", 32, SourceRegType::General, 0xFFFF});

    sourceInfo.registers.emplace_back(SourceRegInfo{"tm2", 32, SourceRegType::Temp, 0xFFFF});

    // condition flags
    sourceInfo.flags.emplace_back(SourceFlagInfo{'V', 28, SourceFlagType::Overflow});
    sourceInfo.flags.emplace_back(SourceFlagInfo{'C', 29, SourceFlagType::Carry});
    sourceInfo.flags.emplace_back(SourceFlagInfo{'Z', 30, SourceFlagType::Zero});
    sourceInfo.flags.emplace_back(SourceFlagInfo{'N', 31, SourceFlagType::Negative});

    sourceInfo.pcSize = 32;
    sourceInfo.pcPrefetch = 2;
    sourceInfo.pcOffset = reinterpret_cast<uintptr_t>(&cpu.regs[15]) - cpuPtrInt;

    sourceInfo.getRegOffset = getRegOffset;

    sourceInfo.exitCallFlag = &exitCallFlag;
    sourceInfo.savedExitPtr = &tmpSavedPtr;

    sourceInfo.readMem8 = reinterpret_cast<uint8_t (*)(void *, uint32_t)>(ARMv7MRecompiler::readMem8);
    sourceInfo.readMem16 = reinterpret_cast<uint32_t (*)(void *, uint32_t)>(ARMv7MRecompiler::readMem16);
    sourceInfo.readMem32 = reinterpret_cast<uint32_t (*)(void *, uint32_t)>(ARMv7MRecompiler::readMem32);

    sourceInfo.writeMem8 = reinterpret_cast<void (*)(void *, uint32_t, uint8_t)>(ARMv7MRecompiler::writeMem8);
    sourceInfo.writeMem16 = reinterpret_cast<void (*)(void *, uint32_t, uint16_t)>(ARMv7MRecompiler::writeMem16);
    sourceInfo.writeMem32 = reinterpret_cast<void (*)(void *, uint32_t, uint32_t)>(ARMv7MRecompiler::writeMem32);

    target.init(sourceInfo, &cpu);

#if defined(__linux__)
    // allocate some memory
    auto pageSize = sysconf(_SC_PAGE_SIZE);
    int numPages = 256 * 8;

    // FIXME: alloc RW, switch to RX
    codeBufSize = pageSize * numPages;
    codeBuf = reinterpret_cast<uint8_t *>(mmap(0, codeBufSize, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));

    if(codeBuf == MAP_FAILED)
        perror("failed to allocate code buffer (mmap failed)");
#elif defined(_WIN32)
    SYSTEM_INFO sysInfo;
    GetSystemInfo(&sysInfo);
    auto pageSize = sysInfo.dwPageSize;
    int numPages = 256;

    codeBufSize = pageSize * numPages;
    codeBuf = reinterpret_cast<uint8_t *>(VirtualAlloc(nullptr, codeBufSize, MEM_COMMIT, PAGE_EXECUTE_READWRITE));
#endif

    curCodePtr = codeBuf;

    for(auto &saved : savedExits)
        saved = {nullptr, 0, 0};
}

void ARMv7MRecompiler::run()
{
    if(!codeBuf)
        return;

    bool ran = false;

    while(true)
    {

        auto inPC = cpu.loReg(ARMv7MCore::Reg::PC);

        if(!attemptToRun())
            break;

        ran = true;

        // handle long branches
        auto outPC = cpu.loReg(ARMv7MCore::Reg::PC);
        if(inPC >> 24 != outPC >> 24)
            cpu.pcPtr = nullptr; // force remap later

        // might have tried to change mode
        if(!(cpu.cpsr & ARMv7MCore::Flag_T))
        {
            if(cpu.loReg(ARMv7MCore::Reg::PC) & 1)
            {
                // nope, it's still thumb (we just did a bx)
                cpu.cpsr |= ARMv7MCore::Flag_T;
                cpu.loReg(ARMv7MCore::Reg::PC) &= ~1;
            }
            else
                abort(); // there is no ARM mode
        }
    }

    // if we executed anything, we need to update PC
    if(ran)
    {
        auto pc = cpu.loReg(ARMv7MCore::Reg::PC) - 2;

        if(!cpu.pcPtr)
            cpu.updateTHUMBPC(pc, true);
        else
        {
            auto thumbPCPtr = reinterpret_cast<const uint16_t *>(cpu.pcPtr + pc);
            cpu.decodeOp = *thumbPCPtr++;
            cpu.fetchOp = *thumbPCPtr;
        }
    }
}

bool ARMv7MRecompiler::attemptToRun()
{
    uint8_t *codePtr = nullptr;

    auto cpuPC = cpu.loReg(ARMv7MCore::Reg::PC) - 2;
    auto blockStartPC = cpuPC;

    // this is the fake return addr
    if(cpuPC == 0x8FFFFFE)
        return false;

    // API calls
    if(cpuPC >> 16 == 0x08BA)
        return false;

    // attempt to re-enter previous code
    int savedIndex = curSavedExit - 1;
    for(int i = 0; i < savedExitsSize; i++, savedIndex--)
    {
        // wrap
        if(savedIndex < 0)
            savedIndex += savedExitsSize;

        uint8_t *ptr;
        uint32_t pc;
        uint32_t startPC;
        std::tie(ptr, pc, startPC) = savedExits[savedIndex];

        if(pc == cpuPC && ptr)
        {
            codePtr = ptr;
            curSavedExit = savedIndex;

            cpu.loReg(ARMv7MCore::Reg::PC) = startPC + 2; // compiled code depends on PC pointing to the start of the block
            blockStartPC = startPC;

            savedExits[savedIndex] = {nullptr, 0, 0};
            break;
        }
    }

    if(!codePtr)
    {
        // lookup compiled code
        auto it = compiled.find(cpuPC);

        if(it == compiled.end())
        {
            if(!entryFunc)
                compileEntry();

            // attempt compile
            auto ptr = curCodePtr;
            auto startPtr = ptr;
            auto pc = cpuPC;

            GenBlockInfo genBlock;
            genBlock.flags = 0;

            convertTHUMBToGeneric(pc, genBlock);

            analyseGenBlock(cpuPC, pc, genBlock, target.getSourceInfo());

#ifdef RECOMPILER_DEBUG
            printf("analysed %04X-%04X (%zi instructions)\n", cpuPC, pc, genBlock.instructions.size());
            printGenBlock(cpuPC, genBlock, target.getSourceInfo());
            printf("\n\n");
#endif

            FuncInfo info{};

            info.cpsrMode = cpu.cpsr & 0x1F;

            if(target.compile(ptr, codeBuf + codeBufSize, cpuPC, genBlock))
            {
                info.startPtr = startPtr;
                info.endPtr = curCodePtr = ptr;
                info.endPC = pc;
            }
            
            it = compiled.emplace(cpuPC, info).first;

            // track range of code in RAM
            // technically includes internal flash
            if(cpuPC < 0x9000000)
            {
                if(cpuPC < minRAMCode)
                    minRAMCode = cpuPC;
                if(pc > maxRAMCode)
                    maxRAMCode = pc;
            }
        }

        // reject code if compiled for a different CPU mode
        if(it->second.cpsrMode == (cpu.cpsr & 0x1F))
            codePtr = it->second.startPtr;
    }

    // run the code if valid, or stop
    if(codePtr)
        entryFunc(0, codePtr);
    else
        return false;

    // code exited with a saved address for re-entry, store PC for later
    if(tmpSavedPtr)
    {
        auto savedPC = cpu.loReg(ARMv7MCore::Reg::PC) - 2;

        // get return address
        if(exitCallFlag)
            savedPC = cpu.reg(ARMv7MCore::Reg::LR) & ~1;

        savedExits[curSavedExit++] = {tmpSavedPtr, savedPC, blockStartPC};
        curSavedExit %= savedExitsSize;

        tmpSavedPtr = nullptr;
        exitCallFlag = false;
    }

    return true;
}

void ARMv7MRecompiler::convertTHUMBToGeneric(uint32_t &pc, GenBlockInfo &genBlock)
{
    bool done = false;

    auto &mem = cpu.getMem();

    auto maxBranch = pc;

    auto reg = [](int reg)
    {
        assert(reg < 15);

        return static_cast<GenReg>(reg + 1);
    };

    auto addInstruction = [&genBlock](GenOpInfo op, uint8_t len = 0, uint16_t flags = 0)
    {
        ::addInstruction(genBlock, op, len, flags);
    };

    auto pcPtr = reinterpret_cast<const uint16_t *>(std::as_const(mem).mapAddress(pc));

    while(!done)
    {
        uint16_t opcode = *pcPtr++;
        pc += 2;

        switch(opcode >> 12)
        {
            case 0x0: // format 1, move shifted
            case 0x1: // formats 1-2
            {
                auto instOp = (opcode >> 11) & 0x3;
                auto srcReg = reg((opcode >> 3) & 7);
                auto dstReg = reg(opcode & 7);

                if(instOp == 3) // format 2, add/sub
                {
                    bool isImm = opcode & (1 << 10);
                    bool isSub = opcode & (1 << 9);

                    auto src1Reg = GenReg::Temp;

                    if(isImm)
                        addInstruction(loadImm((opcode >> 6) & 7));
                    else
                        src1Reg = reg((opcode >> 6) & 7);

                    if(isSub)
                        addInstruction(alu(GenOpcode::Subtract, srcReg, src1Reg, dstReg), 2, writeV | writeC | writeZ | writeN);
                    else
                        addInstruction(alu(GenOpcode::Add, srcReg, src1Reg, dstReg), 2, writeV | writeC | writeZ | writeN);
                }
                else // format 1, move shifted register
                {
                    auto offset = (opcode >> 6) & 0x1F;

                    switch(instOp)
                    {
                        case 0: // LSL
                        {
                            if(offset == 0)
                                addInstruction(move(srcReg, dstReg), 2, preserveV | preserveC | writeZ | writeN);
                            else
                            {
                                addInstruction(loadImm(offset));
                                addInstruction(alu(GenOpcode::ShiftLeft, srcReg, GenReg::Temp, dstReg), 2, preserveV | writeC | writeZ | writeN);
                            }
                            break;
                        }

                        case 1: // LSR
                            addInstruction(loadImm(offset ? offset : 32));
                            addInstruction(alu(GenOpcode::ShiftRightLogic, srcReg, GenReg::Temp, dstReg), 2, preserveV | writeC | writeZ | writeN);
                            break;

                        case 2: // ASR
                            addInstruction(loadImm(offset ? offset : 32));
                            addInstruction(alu(GenOpcode::ShiftRightArith, srcReg, GenReg::Temp, dstReg), 2, preserveV | writeC | writeZ | writeN);
                            break;
                    }
                }
                
                break;
            }

            case 0x2: // format 3, mov/cmp immediate
            case 0x3: // format 3, add/sub immediate
            {
                auto instOp = (opcode >> 11) & 0x3;
                auto dstReg = reg((opcode >> 8) & 7);

                addInstruction(loadImm(opcode & 0xFF));

                switch(instOp)
                {
                    case 0: // MOV
                        addInstruction(move(GenReg::Temp, dstReg), 2, preserveV | preserveC | writeZ | writeN);
                        break;
                    case 1: // CMP
                        addInstruction(compare(dstReg, GenReg::Temp), 2, writeV | writeC | writeZ | writeN);
                        break;
                    case 2: // ADD
                        addInstruction(alu(GenOpcode::Add, dstReg, GenReg::Temp, dstReg), 2, writeV | writeC | writeZ | writeN);
                        break;
                    case 3: // SUB
                        addInstruction(alu(GenOpcode::Subtract, dstReg, GenReg::Temp, dstReg), 2, writeV | writeC | writeZ | writeN);
                        break;
                }

                break;
            }

            case 0x4: // formats 4-6
            {
                if(opcode & (1 << 11)) // format 6, PC-relative load
                {
                    // this is almost certainly going to cause more timing problems
                    auto dstReg = reg((opcode >> 8) & 7);
                    uint8_t word = opcode & 0xFF;
                    auto addr = ((pc + 2) & ~2) + (word << 2);

                    addInstruction(loadImm(mem.read<uint32_t>(addr)));
                    addInstruction(move(GenReg::Temp, dstReg), 2);
                }
                else if(opcode & (1 << 10)) // format 5, Hi reg/branch exchange
                {
                    auto op = (opcode >> 8) & 3;
                    bool h1 = opcode & (1 << 7);
                    bool h2 = opcode & (1 << 6);

                    auto srcReg = ((opcode >> 3) & 7) + (h2 ? 8 : 0);
                    auto dstReg = (opcode & 7) + (h1 ? 8 : 0);

                    switch(op)
                    {
                        case 0: // ADD
                            if(srcReg == 15) // read pc
                            {
                                assert(dstReg != 15);
                                addInstruction(loadImm(pc + 2));
                                addInstruction(alu(GenOpcode::Add, reg(dstReg), GenReg::Temp, reg(dstReg)), 2);
                            }
                            else if(dstReg == 15)
                            {
                                printf("unhandled add pc in convertToGeneric %i -> %i\n", srcReg, dstReg);
                                done = true;
                            }
                            else if(dstReg == 13) // dest SP
                            {
                                addInstruction(alu(GenOpcode::Add, reg(dstReg), reg(srcReg), reg(dstReg)));
                                // mask out low bits
                                addInstruction(loadImm(~3u));
                                addInstruction(alu(GenOpcode::And, reg(dstReg), GenReg::Temp, reg(dstReg)), 2);
                            }
                            else
                                addInstruction(alu(GenOpcode::Add, reg(dstReg), reg(srcReg), reg(dstReg)), 2);
                            break;
                        case 1: // CMP
                            if(srcReg == 15 || dstReg == 15)
                            {
                                printf("unhandled cmp pc in convertToGeneric %i -> %i\n", srcReg, dstReg);
                                done = true;
                            }
                            else
                                addInstruction(compare(reg(dstReg), reg(srcReg)), 2, writeV | writeC | writeZ | writeN);
                            break;

                        case 2: // MOV
                        {
                            if(srcReg == 15) // read pc
                            {
                                assert(dstReg != 15);
                                addInstruction(loadImm(pc + 2));
                                addInstruction(move(GenReg::Temp, reg(dstReg)), 2);
                            }
                            else if(dstReg == 15)
                            {
                                // clear low bit (no interworking here)
                                addInstruction(loadImm(~1u));
                                addInstruction(alu(GenOpcode::And, GenReg::Temp, reg(srcReg), GenReg::Temp));
                                addInstruction(jump(GenCondition::Always, GenReg::Temp), 2);
                                if(pc > maxBranch)
                                    done = true;
                            }
                            else if(dstReg == 13) // dest SP
                            {
                                addInstruction(move(reg(srcReg), reg(dstReg)));
                                // mask out low bits
                                addInstruction(loadImm(~3u));
                                addInstruction(alu(GenOpcode::And, reg(dstReg), GenReg::Temp, reg(dstReg)), 2);
                            }
                            else
                                addInstruction(move(reg(srcReg), reg(dstReg)), 2);
                            break;
                        }

                        case 3: // BX
                        {
                            if(srcReg == 15)
                            {
                                // there is no reason to do this, it would fault
                                printf("unhandled BX PC in convertToGeneric\n");
                                done = true;
                            }
                            else
                            {
                                if(h1) // BLX
                                {
                                    addInstruction(loadImm(pc | 1));
                                    addInstruction(move(GenReg::Temp, GenReg::R14));
                                }

                                // clear T flag
                                addInstruction(loadImm(~ARMv7MCore::Flag_T));
                                addInstruction(alu(GenOpcode::And, GenReg::CPSR, GenReg::Temp, GenReg::CPSR));

                                // jump (without clearing low bit, will use to correct flags later)
                                addInstruction(jump(GenCondition::Always, reg(srcReg)), 2, h1 ? GenOp_Call : 0);

                                if(pc > maxBranch && !h1)
                                    done = true;
                            }
                            break;
                        }
                    }
                }
                else // format 4, alu
                {
                    auto instOp = (opcode >> 6) & 0xF;
                    auto srcReg = reg((opcode >> 3) & 7);
                    auto dstReg = reg(opcode & 7);

                    switch(instOp)
                    {
                        case 0x0: // AND
                            addInstruction(alu(GenOpcode::And, dstReg, srcReg, dstReg), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        case 0x1: // EOR
                            addInstruction(alu(GenOpcode::Xor, dstReg, srcReg, dstReg), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        case 0x2: // LSL
                            addInstruction(alu(GenOpcode::ShiftLeft, dstReg, srcReg, dstReg), 2, preserveV | preserveC | writeC | writeZ | writeN);
                            break;
                        case 0x3: // LSR
                            addInstruction(alu(GenOpcode::ShiftRightLogic, dstReg, srcReg, dstReg), 2, preserveV | preserveC | writeC | writeZ | writeN);
                            break;
                        case 0x4: // ASR
                            addInstruction(alu(GenOpcode::ShiftRightArith, dstReg, srcReg, dstReg), 2, preserveV | preserveC | writeC | writeZ | writeN);
                            break;
                        case 0x5: // ADC
                            addInstruction(alu(GenOpcode::AddWithCarry, dstReg, srcReg, dstReg), 2, writeV | writeC | writeZ | writeN);
                            break;
                        case 0x6: // SBC
                            addInstruction(alu(GenOpcode::SubtractWithCarry, dstReg, srcReg, dstReg), 2, writeV | writeC | writeZ | writeN);
                            break;
                        case 0x7: // ROR
                            addInstruction(alu(GenOpcode::RotateRight, dstReg, srcReg, dstReg), 2, preserveV | preserveC | writeC | writeZ | writeN);
                            break;
                        case 0x8: // TST
                            addInstruction(alu(GenOpcode::And, dstReg, srcReg, GenReg::Temp), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        case 0x9: // NEG
                            addInstruction(loadImm(0));
                            addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, srcReg, dstReg), 2, writeV | writeC | writeZ | writeN);
                            break;
                        case 0xA: // CMP
                            addInstruction(compare(dstReg, srcReg), 2, writeV | writeC | writeZ | writeN);
                            break;
                        case 0xB: // CMN
                            addInstruction(alu(GenOpcode::Add, dstReg, srcReg, GenReg::Temp), 2, writeV | writeC | writeZ | writeN);
                            break;
                        case 0xC: // ORR
                            addInstruction(alu(GenOpcode::Or, dstReg, srcReg, dstReg), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        case 0xD: // MUL
                            addInstruction(alu(GenOpcode::Multiply, dstReg, srcReg, dstReg), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        case 0xE: // BIC
                        {
                            addInstruction(alu(GenOpcode::Not, srcReg, GenReg::Temp));
                            addInstruction(alu(GenOpcode::And, dstReg, GenReg::Temp, dstReg), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        }
                        case 0xF: // MVN
                        {
                            addInstruction(alu(GenOpcode::Not, srcReg, dstReg), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        }

                        default:
                            printf("unhandled alu op in convertToGeneric %x\n", instOp);
                            done = true;
                    }
                    
                }
                break;
            }

            case 0x5: // formats 7-8
            {
                auto offReg = reg((opcode >> 6) & 7);
                auto baseReg = reg((opcode >> 3) & 7);
                auto dstReg = reg(opcode & 7);

                addInstruction(alu(GenOpcode::Add, baseReg, offReg, GenReg::Temp));

                if(opcode & (1 << 9)) // format 8, load/store sign-extended byte/halfword
                {
                    bool hFlag = opcode & (1 << 11);
                    bool signEx = opcode & (1 << 10);

                    if(signEx)
                    {
                        if(hFlag) // LDRSH
                            addInstruction(load(2, GenReg::Temp, dstReg), 2, GenOp_SignExtend);
                        else // LDRSB
                            addInstruction(load(1, GenReg::Temp, dstReg), 2, GenOp_SignExtend);
                    }
                    else
                    {
                        if(hFlag) // LDRH
                            addInstruction(load(2, GenReg::Temp, dstReg), 2);
                        else // STRH
                            addInstruction(store(2, GenReg::Temp, dstReg), 2, GenOp_UpdateCycles);
                    }
                }
                else // format 7, load/store with reg offset
                {
                    bool isLoad = opcode & (1 << 11);
                    bool isByte = opcode & (1 << 10);

                    if(isLoad)
                        addInstruction(load(isByte ? 1 : 4, GenReg::Temp, dstReg), 2);
                    else
                        addInstruction(store(isByte ? 1 : 4, GenReg::Temp, dstReg), 2, GenOp_UpdateCycles);
                }

                break;
            }

            case 0x6: // format 9, load/store with imm offset (words)
            {
                bool isLoad = opcode & (1 << 11);
                auto offset = ((opcode >> 6) & 0x1F);
                auto baseReg = reg((opcode >> 3) & 7);
                auto dstReg = reg(opcode & 7);

                if(isLoad)
                    loadWithOffset(genBlock, 4, baseReg, offset * 4, dstReg, 0, 2);
                else
                    storeWithOffset(genBlock, 4, baseReg, offset * 4, dstReg, 0, 2, GenOp_UpdateCycles);

                break;
            }

            case 0x7: // ... (bytes)
            {
                bool isLoad = opcode & (1 << 11);
                auto offset = ((opcode >> 6) & 0x1F);
                auto baseReg = reg((opcode >> 3) & 7);
                auto dstReg = reg(opcode & 7);

                if(isLoad)
                    loadWithOffset(genBlock, 1, baseReg, offset, dstReg, 0, 2);
                else
                    storeWithOffset(genBlock, 1, baseReg, offset, dstReg, 0, 2, GenOp_UpdateCycles);

                break;
            }

            case 0x8: // format 10, load/store halfword
            {
                bool isLoad = opcode & (1 << 11);
                auto offset = ((opcode >> 6) & 0x1F);
                auto baseReg = reg((opcode >> 3) & 7);
                auto dstReg = reg(opcode & 7);

                if(isLoad)
                    loadWithOffset(genBlock, 2, baseReg, offset * 2, dstReg, 0, 2);
                else
                    storeWithOffset(genBlock, 2, baseReg, offset * 2, dstReg, 0, 2, GenOp_UpdateCycles);

                break;
            }
    
            case 0x9: // format 11, SP-relative load/store
            {
                bool isLoad = opcode & (1 << 11);
                auto offset = opcode & 0xFF;
                auto baseReg = GenReg::R13;
                auto dstReg = reg((opcode >> 8) & 7);

                if(isLoad)
                    loadWithOffset(genBlock, 4, baseReg, offset * 4, dstReg, 0, 2);
                else
                    storeWithOffset(genBlock, 4, baseReg, offset * 4, dstReg, 0, 2, GenOp_UpdateCycles);

                break;
            }

            case 0xA: // format 12, load address
            {
                bool isSP = opcode & (1 << 11);
                auto dstReg = reg((opcode >> 8) & 7);
                auto word = (opcode & 0xFF) << 2;

                if(isSP)
                {
                    addInstruction(loadImm(word));
                    addInstruction(alu(GenOpcode::Add, GenReg::R13, GenReg::Temp, dstReg), 2);
                }
                else // PC
                {
                    addInstruction(loadImm(((pc + 2) & ~2) + word));
                    addInstruction(move(GenReg::Temp, dstReg), 2);
                }

                break;
            }

            case 0xB: // misc
            {
                switch((opcode >> 8) & 0xF)
                {
                    case 0x0: // add/sub imm to SP
                    {
                        bool isNeg = opcode & (1 << 7);
                        int off = (opcode & 0x7F) << 2;

                        addInstruction(loadImm(off));

                        if(isNeg)
                            addInstruction(alu(GenOpcode::Subtract, GenReg::R13, GenReg::Temp, GenReg::R13), 2);
                        else
                            addInstruction(alu(GenOpcode::Add, GenReg::R13, GenReg::Temp, GenReg::R13), 2);
                        break;
                    }

                    case 0x1: // CBZ
                    case 0x3:
                    case 0x9: // CBNZ
                    case 0xB:
                    {
                        bool nz = opcode & (1 << 11);
                        int offset = (opcode & 0xF8) >> 2 | (opcode & (1 << 9)) >> 3;
                        auto src = reg(opcode & 7);

                        auto addr = pc + 2 + offset;
                        addInstruction(loadImm(addr));

                        GenOpInfo jump{};
                        jump.opcode = GenOpcode::CompareJump;
                        jump.src[0] = static_cast<uint8_t>(nz ? GenCondition::NotEqual : GenCondition::Equal);
                        jump.src[1] = static_cast<uint8_t>(GenReg::Temp);
                        jump.dst[0] = static_cast<uint8_t>(src);

                        addInstruction(jump, 2);

                        maxBranch = std::max(maxBranch, addr);

                        break;
                    }
                    
                    case 0x2:
                    {
                        auto srcReg = reg((opcode >> 3) & 7);
                        auto dstReg = reg(opcode & 7);

                        switch((opcode >> 6) & 3)
                        {
                            case 0x0: // SXTH
                            case 0x1: // SXTB
                            {
                                GenOpInfo extOp{};
                                extOp.opcode = ((opcode >> 6) & 3) == 0 ? GenOpcode::SignExtend16 : GenOpcode::SignExtend8;
                                extOp.src[0] = static_cast<uint8_t>(srcReg);
                                extOp.dst[0] = static_cast<uint8_t>(dstReg);
                                addInstruction(extOp, 2);
                                break;
                            }
                            case 0x2: // UXTH
                                addInstruction(loadImm(0xFFFF));
                                addInstruction(alu(GenOpcode::And, srcReg, GenReg::Temp, dstReg), 2);
                                break;
                            case 0x3: // UXTB
                                addInstruction(loadImm(0xFF));
                                addInstruction(alu(GenOpcode::And, srcReg, GenReg::Temp, dstReg), 2);
                                break;
                        }

                        break;
                    }

                    case 0x4: // PUSH
                    case 0x5:
                    case 0xC: // POP
                    case 0xD:
                    {
                        bool isLoad = opcode & (1 << 11);
                        bool pclr = opcode & (1 << 8); // store LR/load PC
                        uint8_t regList = opcode & 0xFF;
                        auto baseReg = GenReg::R13;

                        if(isLoad) // POP
                        {
                            uint32_t offset = 0;
                            for(int i = 0; i < 8; i++)
                            {
                                if(regList & (1 << i))
                                {
                                    // load
                                    loadWithOffset(genBlock, 4, baseReg, offset * 4, reg(i), 0, 0, (offset ? GenOp_Sequential : 0) | GenOp_ForceAlign);
                                    offset++;
                                }
                            }

                            if(pclr)
                            {
                                // load pc (to temp)
                                loadWithOffset(genBlock, 4, baseReg, offset * 4, GenReg::Temp2, 0, 0, (offset ? GenOp_Sequential : 0) | GenOp_ForceAlign);

                                // clear thumb bit
                                addInstruction(loadImm(~1u));
                                addInstruction(alu(GenOpcode::And, GenReg::Temp2, GenReg::Temp, GenReg::Temp2));
        
                                offset++;

                                if(pc > maxBranch)
                                    done = true;
                            }

                            // update SP
                            addInstruction(loadImm(offset * 4));
                            addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, baseReg), pclr ? 0 : 2);

                            if(pclr)
                                addInstruction(jump(GenCondition::Always, GenReg::Temp2), 2); // TODO: cycles for branch (not implemented in CPU either)
                        }
                        else
                        {
                            uint32_t offset = 0;

                            if(pclr)
                                offset += 4;

                            for(int i = 0; i < 8; i++)
                            {
                                if(regList & (1 << i))
                                    offset += 4;
                            }

                            // update SP
                            addInstruction(loadImm(offset));
                            addInstruction(alu(GenOpcode::Subtract, baseReg, GenReg::Temp, baseReg));

                            offset = 0;
                            for(int i = 0; i < 8; i++)
                            {
                                if(regList & (1 << i))
                                {
                                    // store
                                    bool last = !pclr && (regList >> i) == 1;
                                    storeWithOffset(genBlock, 4, baseReg, offset * 4, reg(i), 0, 0, (offset ? GenOp_Sequential : 0) | (last ? GenOp_UpdateCycles : 0));
                                    offset++;
                                }
                            }

                            if(pclr) // store LR
                                storeWithOffset(genBlock, 4, baseReg, offset * 4, GenReg::R14, 0, 0, (offset ? GenOp_Sequential : 0) | GenOp_UpdateCycles);

                            genBlock.instructions.back().len = 2;
                        }
                        break;   
                    }

                    case 0xF: // hints
                    {
                        auto opA = (opcode >> 4) & 0xF;
                        auto opB = opcode & 0xF;

                        if(opB == 0)
                        {
                            switch(opA)
                            {
                                case 0: // NOP
                                {
                                    GenOpInfo op{};
                                    op.opcode = GenOpcode::NOP;
                                    op.cycles = 1;

                                    addInstruction(op, 2);
                                    break;
                                }

                                default:
                                    done = true;
                                    printf("unhandled hint op in convertToGeneric %X %X\n", opA, opB);
                            }
                        }
                        else
                        {
                            done = true;
                            printf("unhandled hint op in convertToGeneric %X %X\n", opA, opB);
                        }
                        break;
                    }

                    default:
                        done = true;
                        printf("unhandled op in convertToGeneric %04X\n", opcode & 0xFF00);
                }

                break;
            }

            case 0xC: // format 15, multiple load/store
            {
                bool isLoad = opcode & (1 << 11);
                int baseRegIndex = (opcode >> 8) & 7;
                auto baseReg = reg(baseRegIndex);
                uint8_t regList = opcode & 0xFF;

                if(!regList)
                {
                    if(isLoad)
                    {
                        // load PC
                        addInstruction(load(4, baseReg, GenReg::Temp2, 0));

                        // clear thumb bit
                        addInstruction(loadImm(~1u));
                        addInstruction(alu(GenOpcode::And, GenReg::Temp2, GenReg::Temp, GenReg::Temp2));

                        if(pc > maxBranch)
                            done = true;

                        // update base
                        addInstruction(loadImm(0x40));
                        addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, baseReg));

                        addInstruction(jump(GenCondition::Always, GenReg::Temp2), 2); // TODO: cycles for branch (not implemented in CPU either)
                    }
                    else
                    {
                        // store PC
                        addInstruction(loadImm(pc + 4));
                        addInstruction(store(4, baseReg, GenReg::Temp, 0), 0, GenOp_UpdateCycles);

                        // update base
                        addInstruction(loadImm(0x40));
                        addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, baseReg), 2);
                    }
                }
                else
                {
                    int regCount = 0;
                    int lastRegIndex = 0;
                    for(int i = 0; i < 8; i++)
                    {
                        if(regList & (1 << i))
                        {
                            regCount++;
                            lastRegIndex = i;
                        }
                    }
                   
                    bool first = true;
                    bool wroteBack = false;

                    // prevent overriding base for loads
                    // "A LDM will always overwrite the updated base if the base is in the list."
                    if(isLoad && (regList & (1 << baseRegIndex)))
                    {
                        first = false;

                        // move the base to the last loaded reg so it doesn't get overwritten until we're done
                        auto lastReg = reg(lastRegIndex);
                        if(lastReg != baseReg)
                        {
                            addInstruction(move(baseReg, lastReg));
                            baseReg = lastReg;
                        }
                    }

                    uint32_t offset = 0;

                    for(int i = 0; i < 8; i++)
                    {
                        if(!(regList & (1 << i)))
                            continue;

                        bool last = (regList >> i) == 1;

                        if(offset)
                        {
                            if(wroteBack)
                            {
                                // we updated it so index backwards
                                addInstruction(loadImm(regCount * 4 - offset));
                                addInstruction(alu(GenOpcode::Subtract, baseReg, GenReg::Temp, GenReg::Temp));
                            }
                            else
                            {
                                addInstruction(loadImm(offset));
                                addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, GenReg::Temp));
                            }
                        }

                        int flags = (offset ? GenOp_Sequential : 0) | GenOp_ForceAlign;
                        if(isLoad)
                            addInstruction(load(4, offset ? GenReg::Temp : baseReg, reg(i), 0), 0, flags);
                        else
                            addInstruction(store(4, offset ? GenReg::Temp : baseReg, reg(i), 0), 0, flags | (last ? GenOp_UpdateCycles : 0));

                        // base write-back is on the second cycle of the instruction
                        // which is when the first reg is written
                        if(first)
                        {
                            addInstruction(loadImm(regCount * 4));
                            addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, baseReg));
                            wroteBack = true; // unfortunately, we're still using it to index
                        }

                        first = false;

                        offset += 4;
                    }

                    genBlock.instructions.back().len = 2;
                }

                break;
            }

            case 0xD: // formats 16-17, conditional branch + SWI
            {
                auto cond = (opcode >> 8) & 0xF;
                int offset = static_cast<int8_t>(opcode & 0xFF);

                if(cond == 15)
                {
                    printf("unhandled SWI in convertToGeneric %04X\n", opcode & 0xF000);
                    done = true;
                }
                else
                {
                    auto genCond = static_cast<GenCondition>(cond); // they happen to match
                    auto addr = pc + 2 + offset * 2;
                    addInstruction(loadImm(addr));
                    addInstruction(jump(genCond, GenReg::Temp), 2);

                    maxBranch = std::max(maxBranch, addr);
                }

                break;
            }

            case 0xE: // format 18, unconditional branch
            {
                if(!(opcode & (1 << 11)))
                {
                    uint32_t offset = static_cast<int16_t>(opcode << 5) >> 4; // sign extend and * 2
                    addInstruction(loadImm(pc + 2 + offset));
                    addInstruction(jump(GenCondition::Always, GenReg::Temp), 2);

                    if(pc > maxBranch)
                        done = true;
                        
                    break;
                }
                // otherwise 32-bit encoding
                [[fallthrough]];
            }

            case 0xF: // 32-bit encoding
            {
                uint32_t opcode32 = opcode << 16 | *pcPtr++;
                pc += 2;

                done = convertTHUMB32BitToGeneric(pc, genBlock, opcode32, maxBranch);
                break;
            }

            default:
            {
                printf("invalid op in convertToGeneric %Xxxx\n", opcode >> 12);
                done = true;
            }
        }
    }
}

bool ARMv7MRecompiler::convertTHUMB32BitToGeneric(uint32_t &pc, GenBlockInfo &genBlock, uint32_t opcode32, uint32_t &maxBranch)
{
    auto reg = [](int reg)
    {
        assert(reg < 15);

        return static_cast<GenReg>(reg + 1);
    };

    auto addInstruction = [&genBlock](GenOpInfo op, uint8_t len = 0, uint16_t flags = 0)
    {
        ::addInstruction(genBlock, op, len, flags);
    };

    auto &mem = cpu.getMem();

    auto op1 = opcode32 >> 27;

    if(op1 == 0b11101)
    {
        if(opcode32 & (1 << 26)) // coprocessor
        {
            printf("unhandled op in convertToGeneric %08X (coproc)\n", opcode32 & 0xFF000000);
            return true;
        }
        else if(opcode32 & (1 << 25)) // data processing (shifted register)
        {
            auto op = (opcode32 >> 21) & 0xF;
            bool setFlags = opcode32 & (1 << 20);
            
            auto nReg = (opcode32 >> 16) & 0xF;
            auto mReg = opcode32 & 0xF;
            auto dstReg = (opcode32 >> 8) & 0xF;

            // get shift
            auto imm = ((opcode32 >> 10) & 0x1C) | ((opcode32 >> 6) & 0x3);
            auto type = (opcode32 >> 4) & 3;

            bool shiftCarry = setFlags && op < 8/*not add/sub*/;

            if(type == 3 && !imm) // RRX
            {
                printf("unhandled dp (shift reg) op in convertToGeneric %i (rrx)\n", op);
                return true;
            }

            auto doShift = [&]()
            {
                switch(type)
                {
                    case 0: // LSL
                    {
                        if(imm == 0) // do nothing
                            addInstruction(move(reg(mReg), GenReg::Temp));
                        else
                        {
                            addInstruction(loadImm(imm));
                            addInstruction(alu(GenOpcode::ShiftLeft, reg(mReg), GenReg::Temp, GenReg::Temp), 0, shiftCarry ? (preserveV | writeC) : 0);
                        }
                        break;
                    }
                    case 1: // LSR
                    {
                        addInstruction(loadImm(imm ? imm : 32));
                        addInstruction(alu(GenOpcode::ShiftRightLogic, reg(mReg), GenReg::Temp, GenReg::Temp), 0, shiftCarry ? (preserveV | writeC) : 0);
                        break;
                    }
                    case 2: // ASR
                    {
                        addInstruction(loadImm(imm ? imm : 32));
                        addInstruction(alu(GenOpcode::ShiftRightArith, reg(mReg), GenReg::Temp, GenReg::Temp), 0, shiftCarry ? (preserveV | writeC) : 0);
                        break;
                    }
                    case 3:
                    {
                        if(!imm) // RRX
                        {
                            
                        }
                        else // ROR
                        {
                            addInstruction(loadImm(imm));
                            addInstruction(alu(GenOpcode::RotateRight, reg(mReg), GenReg::Temp, GenReg::Temp), 0, shiftCarry ? (preserveV | writeC) : 0);
                        }
                    }
                }
            };

            switch(op)
            {
                case 0x0: // AND/TST
                {
                    auto dst = dstReg == 15 ? GenReg::Temp : reg(dstReg); // dst == PC is TST
                    doShift();
                    addInstruction(alu(GenOpcode::And, reg(nReg), GenReg::Temp, dst), 4, setFlags ? (preserveV | preserveC | writeZ | writeN) : 0);
                    break;
                }
                case 0x1: // BIC
                {
                    doShift();
                    addInstruction(alu(GenOpcode::Not, GenReg::Temp, GenReg::Temp));
                    addInstruction(alu(GenOpcode::And, reg(nReg), GenReg::Temp, reg(dstReg)), 4, setFlags ? (preserveV | preserveC | writeZ | writeN) : 0);
                    break;
                }
                case 0x2: // MOV/ORR
                {
                    doShift();
                    if(nReg == 15) // MOV
                        addInstruction(move(GenReg::Temp, reg(dstReg)), 4, setFlags ? (preserveV | preserveC | writeZ | writeN) : 0);
                    else // ORR
                        addInstruction(alu(GenOpcode::Or, reg(nReg), GenReg::Temp, reg(dstReg)), 4, setFlags ? (preserveV | preserveC | writeZ | writeN) : 0);
                    break;
                }
                case 0x3: // MVN/ORN
                {
                    doShift();

                    if(nReg == 15) // MVN
                        addInstruction(alu(GenOpcode::Not, GenReg::Temp, reg(dstReg)), 4, setFlags ? (preserveV | preserveC | writeZ | writeN) : 0);
                    else // ORN
                    {
                        addInstruction(alu(GenOpcode::Not, GenReg::Temp, GenReg::Temp));
                        addInstruction(alu(GenOpcode::Or, reg(nReg), GenReg::Temp, reg(dstReg)), 4, setFlags ? (preserveV | preserveC | writeZ | writeN) : 0);
                    }
                    break;
                }
                case 0x4: // EOR/TEQ
                {
                    auto dst = dstReg == 15 ? GenReg::Temp : reg(dstReg); // dst == PC is TEQ
                    doShift();
                    addInstruction(alu(GenOpcode::Xor, reg(nReg), GenReg::Temp, dst), 4, setFlags ? (preserveV | preserveC | writeZ | writeN) : 0);
                    break;
                }

                case 0x8: // ADD/CMN
                {
                    doShift();
                    if(dstReg == 15) // CMN
                    {
                        assert(setFlags);
                        addInstruction(alu(GenOpcode::Add, GenReg::Temp, reg(nReg), GenReg::Temp), 4, setFlags ? (writeV | writeC | writeZ | writeN) : 0);
                    }
                    else
                        addInstruction(alu(GenOpcode::Add, reg(nReg), GenReg::Temp, reg(dstReg)), 4, setFlags ? (writeV | writeC | writeZ | writeN) : 0);
                    break;
                }

                case 0xA: // ADC
                {
                    doShift();
                    addInstruction(alu(GenOpcode::AddWithCarry, reg(nReg), GenReg::Temp, reg(dstReg)), 4, setFlags ? (writeV | writeC | writeZ | writeN) : 0);
                    break;
                }
                case 0xB: // SBC
                {
                    doShift();
                    addInstruction(alu(GenOpcode::SubtractWithCarry, reg(nReg), GenReg::Temp, reg(dstReg)), 4, setFlags ? (writeV | writeC | writeZ | writeN) : 0);
                    break;
                }

                case 0xD: // SUB/CMP
                {
                    doShift();

                    if(dstReg == 15) // CMP
                        addInstruction(compare(reg(nReg), GenReg::Temp), 4, setFlags ? (writeV | writeC | writeZ | writeN) : 0);
                    else // SUB
                        addInstruction(alu(GenOpcode::Subtract, reg(nReg), GenReg::Temp, reg(dstReg)), 4, setFlags ? (writeV | writeC | writeZ | writeN) : 0);
                    break;
                }
                case 0xE: // RSB
                {
                    assert(dstReg != 15);
                    auto dst = reg(dstReg);
                    doShift();
                    addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, reg(nReg), dst), 4, setFlags ? (writeV | writeC | writeZ | writeN) : 0);
                    break;
                }

                default:
                    printf("unhandled dp (shift reg) op in convertToGeneric %i\n", op);            
                    return true;
            }
        }
        else if(opcode32 & (1 << 22)) // load/store dual or exclusive
        {
            auto op1 = (opcode32 >> 23) & 3;
            auto op2 = (opcode32 >> 20) & 3;
            //auto op3 = (opcode32 >> 4) & 0xF;

            if(op1 == 1 && op2 == 1) // TBB/TBH/LDREXB/LDREXH
            {
                printf("unhandled op in convertToGeneric %08X (tb/ldrex)\n", opcode32 & 0xFFF00000);
                return true;
            }
            else if(op1 == 0 && !(op2 & 2)) // LDREX/STREX
            {
                printf("unhandled op in convertToGeneric %08X (l/s ex)\n", opcode32 & 0xFFF00000);
                return true;
            }
            else if((op1 & 2) || (op2 & 2)) // LDRD/STRD
            {
                bool isLoad = op2 & 1;
                auto baseReg = reg((opcode32 >> 16) & 0xF);
                auto dstReg = reg((opcode32 >> 12) & 0xF);
                auto dstReg2 = reg((opcode32 >> 8) & 0xF);

                int offset = (opcode32 & 0xFF) << 2;

                bool writeback = opcode32 & (1 << 21);
                bool add = opcode32 & (1 << 23);
                bool index = opcode32 & (1 << 24);

                if(isLoad)
                {
                    // save base if it's the first loaded
                    bool baseDst = baseReg == dstReg;

                    if(baseDst)
                        addInstruction(move(baseReg, GenReg::Temp2));
                    
                    // first load
                    if(index)
                    {
                        addInstruction(loadImm(add ? offset : -offset));
                        addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, GenReg::Temp));
                    }
                    
                    addInstruction(load(4, index ? GenReg::Temp : baseReg, dstReg));

                    // second load
                    if(index)
                        addInstruction(loadImm((add ? offset : -offset) + 4));
                    else
                        addInstruction(loadImm(4));

                    addInstruction(alu(GenOpcode::Add, baseDst ? GenReg::Temp2 :  baseReg, GenReg::Temp, GenReg::Temp));
                    addInstruction(load(4, GenReg::Temp, dstReg2), writeback ? 0 : 4);
                    
                    // write back adjusted base
                    if(writeback)
                    {
                        // need to redo add even if index is true (can't reuse the temp)
                        addInstruction(loadImm(add ? offset : -offset));
                        addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, baseReg), 4);
                    }
                }
                else
                {
                    // first store
                    if(index)
                    {
                        addInstruction(loadImm(add ? offset : -offset));
                        addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, GenReg::Temp));
                    }
                    
                    addInstruction(store(4, index ? GenReg::Temp : baseReg, dstReg));

                    // second store
                    if(index)
                        addInstruction(loadImm((add ? offset : -offset) + 4));
                    else
                        addInstruction(loadImm(4));

                    addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, GenReg::Temp));
                    addInstruction(store(4, GenReg::Temp, dstReg2), writeback ? 0 : 4);
                    
                    // write back adjusted base
                    if(writeback)
                    {
                        // need to redo add even if index is true (can't reuse the temp)
                        addInstruction(loadImm(add ? offset : -offset));
                        addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, baseReg), 4);
                    }
                }
            }
            else
            {
                printf("unhandled op in convertToGeneric %08X (l/s d/e)\n", opcode32 & 0xFFF00000);
                return true;
            }
        }
        else // load/store multiple
        {
            auto op = (opcode32 >> 23) & 3;
            bool writeback = opcode32 & (1 << 21);   
            bool isLoad = opcode32 & (1 << 20);
            auto baseReg = (opcode32 >> 16) & 0xF;
            uint16_t regList = opcode32 & 0xFFFF;

            assert(!(regList & (1 << 13)));

            if(isLoad)
            {
                bool baseInList = regList & (1 << baseReg);

                if(op == 1) // IA
                {
                    addInstruction(move(reg(baseReg), GenReg::Temp2));

                    for(int i = 0; i < 15; i++)
                    {
                        if(!(regList & (1 << i)))
                            continue;

                        addInstruction(load(4, GenReg::Temp2, reg(i), 0));
                    
                        // addr += 4
                        addInstruction(loadImm(4));
                        addInstruction(alu(GenOpcode::Add, GenReg::Temp2, GenReg::Temp, GenReg::Temp2));
                    }

                    bool writePC = regList >> 15;

                    if(writeback && !baseInList)
                    {
                        if(writePC)
                        {
                            // do the last +4
                            addInstruction(loadImm(4));
                            addInstruction(alu(GenOpcode::Add, GenReg::Temp2, GenReg::Temp, reg(baseReg)));
                        }
                        else
                            addInstruction(move(GenReg::Temp2, reg(baseReg)));
                    }

                    if(writePC)
                    {
                        // jump
                        addInstruction(load(4, GenReg::Temp2, GenReg::Temp2, 0));
                        addInstruction(loadImm(~1u));
                        addInstruction(alu(GenOpcode::And, GenReg::Temp, GenReg::Temp2, GenReg::Temp2));
                        addInstruction(jump(GenCondition::Always, GenReg::Temp2, 0));

                        if(pc > maxBranch)
                            return true;
                    }

                    genBlock.instructions.back().len = 4;
                }
                else if(op == 2) // DB
                {
                    // pre-decrement address
                    int offset = 0;
                    for(uint16_t t = regList; t; t >>= 1)
                    {
                        if(t & 1)
                            offset += 4;
                    }

                    addInstruction(loadImm(offset));
                    addInstruction(alu(GenOpcode::Subtract, reg(baseReg), GenReg::Temp, GenReg::Temp2));

                    // this loop is the same as above...
                    for(int i = 0; i < 15; i++)
                    {
                        if(!(regList & (1 << i)))
                            continue;

                        addInstruction(load(4, GenReg::Temp2, reg(i), 0));
                    
                        // addr += 4
                        addInstruction(loadImm(4));
                        addInstruction(alu(GenOpcode::Add, GenReg::Temp2, GenReg::Temp, GenReg::Temp2));
                    }

                    bool writePC = regList >> 15;

                    if(writeback && !baseInList)
                    {
                        addInstruction(loadImm(offset));
                        addInstruction(alu(GenOpcode::Subtract, reg(baseReg), GenReg::Temp, reg(baseReg)));
                    }

                    if(writePC)
                    {
                        // jump
                        addInstruction(load(4, GenReg::Temp2, GenReg::Temp2, 0));
                        addInstruction(loadImm(~1u));
                        addInstruction(alu(GenOpcode::And, GenReg::Temp, GenReg::Temp2, GenReg::Temp2));
                        addInstruction(jump(GenCondition::Always, GenReg::Temp2, 0));

                        if(pc > maxBranch)
                            return true;
                    }

                    genBlock.instructions.back().len = 4;
                }
                else
                {
                    printf("unhandled op in convertToGeneric %08X (LDM %i)\n", opcode32 & 0xFF000000, op);
                    return true;
                }
            }
            else
            {
                assert(!(regList & (1 << 15)));

                if(op == 1) // IA
                {
                    addInstruction(move(reg(baseReg), GenReg::Temp2));

                    for(int i = 0; i < 15; i++)
                    {
                        if(!(regList & (1 << i)))
                            continue;

                        addInstruction(store(4, GenReg::Temp2, reg(i), 0));
                    
                        // addr += 4
                        addInstruction(loadImm(4));
                        addInstruction(alu(GenOpcode::Add, GenReg::Temp2, GenReg::Temp, GenReg::Temp2));
                    }

                    if(writeback)
                        addInstruction(move(GenReg::Temp2, reg(baseReg)));

                    genBlock.instructions.back().len = 4;
                }
                else if(op == 2) // DB
                {
                    // pre-decrement address
                    int offset = 0;
                    for(uint16_t t = regList; t; t >>= 1)
                    {
                        if(t & 1)
                            offset += 4;
                    }

                    addInstruction(loadImm(offset));
                    addInstruction(alu(GenOpcode::Subtract, reg(baseReg), GenReg::Temp, GenReg::Temp2));

                    for(int i = 0; i < 15; i++)
                    {
                        if(!(regList & (1 << i)))
                            continue;

                        addInstruction(store(4, GenReg::Temp2, reg(i), 0));
                    
                        // addr += 4
                        addInstruction(loadImm(4));
                        addInstruction(alu(GenOpcode::Add, GenReg::Temp2, GenReg::Temp, GenReg::Temp2));
                    }

                    if(writeback)
                    {
                        addInstruction(loadImm(offset));
                        addInstruction(alu(GenOpcode::Subtract, reg(baseReg), GenReg::Temp, reg(baseReg)));
                    }

                    genBlock.instructions.back().len = 4;
                }
                else
                {
                    printf("unhandled op in convertToGeneric %08X (STM %i)\n", opcode32 & 0xFF000000, op);
                    return true;
                }
            }
        }
    }
    else if(op1 == 0b11110)
    {
        if(opcode32 & (1 << 15))
        {
            // branch and misc control
            auto op1 = (opcode32 >> 20) & 0x7F;
            auto op2 = (opcode32 >> 12) & 0x7;

            if(op2 & 1) // B/BL
            {
                bool link = op2 & 0b100;
                auto imm11 = opcode32 & 0x7FF;
                auto imm10 = (opcode32 >> 16) & 0x3FF;

                auto s = opcode32 & (1 << 26);
                auto i1 = (opcode32 >> 13) & 1;
                auto i2 = (opcode32 >> 11) & 1;

                if(!s)
                {
                    i1 ^= 1;
                    i2 ^= 1;
                }

                uint32_t offset = imm11 << 1 | imm10 << 12 | i2 << 22 | i1 << 23;

                if(s)
                    offset |= 0xFF000000; // sign extend

                if(link) // BL
                {
                    // LR = PC | 1
                    addInstruction(loadImm(pc | 1));
                    addInstruction(move(GenReg::Temp, GenReg::R14));
                }

                // jump
                addInstruction(loadImm(pc + offset));
                addInstruction(jump(GenCondition::Always, GenReg::Temp), 4, link ? GenOp_Call : 0);

                if(!link && pc > maxBranch)
                    return true;
            }
            else if((op1 & 0b111000) != 0b111000) // Bcc
            {
                auto cond = (op1 >> 2) & 0xF;

                auto imm11 = opcode32 & 0x7FF;
                auto imm6 = (opcode32 >> 16) & 0x3F;
        
                auto s = opcode32 & (1 << 26);
                auto i1 = (opcode32 >> 13) & 1;
                auto i2 = (opcode32 >> 11) & 1;
        
                uint32_t offset = imm11 << 1 | imm6 << 12 | i2 << 18 | i1 << 19;
        
                if(s)
                    offset |= 0xFFF00000; // sign extend

                auto genCond = static_cast<GenCondition>(cond); // they happen to match
                addInstruction(loadImm(pc + offset));
                addInstruction(jump(genCond, GenReg::Temp), 4);

                maxBranch = std::max(maxBranch, pc + offset);
            }
            else
            {
                switch(op1)
                {
                    case 0x38: // MSR
                    case 0x39:
                    {
                        auto srcReg = reg((opcode32 >> 16) & 0xF);
                        auto sysm = opcode32 & 0xFF;

                        if((sysm >> 3) == 0)
                        {
                            // APSR
                            addInstruction(loadImm(~0xF8000000));
                            addInstruction(alu(GenOpcode::And, GenReg::CPSR, GenReg::Temp, GenReg::CPSR));
                            addInstruction(loadImm(0xF8000000));
                            addInstruction(alu(GenOpcode::And, GenReg::Temp, srcReg, GenReg::Temp));
                            addInstruction(alu(GenOpcode::Or, GenReg::CPSR, GenReg::Temp, GenReg::CPSR), 4);
                        }
                        else if((sysm >> 3) == 1)
                        {
                            // MSP/PSP
                            // assuming priviledged
                            if(sysm == 8) // MSP
                            {
                                addInstruction(loadImm(~3));
                                addInstruction(alu(GenOpcode::And, GenReg::Temp, srcReg, GenReg::Temp));
                                addInstruction(move(GenReg::Temp, GenReg::R13), 4);
                            }
                            else if(sysm == 9)
                            {
                                printf("unhandled MSR PSP in convertToGeneric\n");
                                return true;
                            }
                        }
                        else if((sysm >> 3) == 2)
                        {
                            // PRIMASK/CONTROL
                            // assuming priviledged

                            if(sysm == 0x10) // PRIMASK
                            {
                                addInstruction(loadImm(1));
                                addInstruction(alu(GenOpcode::And, GenReg::Temp, srcReg, GenReg::Temp));
                                addInstruction(move(GenReg::Temp, GenReg::PriMask), 4);
                            }
                            else
                            {
                                printf("unhandled MSR CONTROL in convertToGeneric\n");    
                                return true;
                            }
                        }
                        break;
                    }

                    case 0x3A: // hints
                    {
                        if((opcode32 & 0x7FF) == 0) // NOP
                        {
                            GenOpInfo op{};
                            op.opcode = GenOpcode::NOP;
                            op.cycles = 1;

                            addInstruction(op, 4);
                        }
                        else
                        {
                            printf("unhandled hint op in convertToGeneric %08X\n", opcode32 & 0xFFF0FFFF);
                            return true;
                        }
                        break;
                    }
                    case 0x3B: // misc
                    {
                        auto op = (opcode32 >> 4) & 0xF;

                        if(op == 0x4 || op == 0x5) // DSB/DMB
                        {
                            GenOpInfo op{};
                            op.opcode = GenOpcode::NOP;
                            op.cycles = 1;

                            addInstruction(op, 4);
                        }
                        else
                            printf("unhandled misc ctrl op in convertToGeneric %X\n", op);

                        break;
                    }

                    case 0x3E: // MRS
                    case 0x3F:
                    {
                        auto dstReg = reg((opcode32 >> 8) & 0xF);
                        auto sysm = opcode32 & 0xFF;

                        if((sysm >> 3) == 0)
                        {
                            // xPSR
                            uint32_t mask = 0;
                            if(sysm & 1) // IPSR
                                mask |= 0x1FF;

                            // if(sysm & 2) // T bit reads as 0 so do nothing

                            if((sysm & 4) == 0) // APSR
                                mask |= 0xF8000000;

                            // psr & mask
                            addInstruction(loadImm(mask));
                            addInstruction(alu(GenOpcode::And, GenReg::CPSR, GenReg::Temp, GenReg::Temp));
                            // separate mov to help the target
                            addInstruction(move(GenReg::Temp, dstReg), 4);

                        }
                        else if((sysm >> 3) == 2)
                        {
                            // PRIMASK/CONTROL
                            if(sysm == 0x10)
                            {
                                // primask & 1
                                addInstruction(loadImm(1));
                                addInstruction(alu(GenOpcode::And, GenReg::PriMask, GenReg::Temp, GenReg::Temp));
                                addInstruction(move(GenReg::Temp, dstReg), 4);
                            }
                            else if(sysm == 0x14)
                            {
                                // control & 3
                                addInstruction(loadImm(3));
                                addInstruction(alu(GenOpcode::And, GenReg::Control, GenReg::Temp, GenReg::Temp));
                                addInstruction(move(GenReg::Temp, dstReg), 4);
                            }
                        }
                        else
                        {
                            printf("unhandled MRS in convertToGeneric %02X\n", sysm);
                            return true;
                        }
                        break;
                    }

                    default:
                        printf("unhandled op in convertToGeneric %08X\n", opcode32 & 0xF7F0F000);
                        return true;
                }
            }
        }
        else if(opcode32 & (1 << 25)) // data processing (plain immediate)
        {
            auto op = (opcode32 >> 21) & 0xF;

            assert(!(opcode32 & (1 << 20))); // S = 0

            auto nReg = (opcode32 >> 16) & 0xF;
            auto dstReg = (opcode32 >> 8) & 0xF;

            switch(op)
            {
                case 0x0:
                {
                    if(nReg == 15) // ADR
                    {
                        printf("unhandled dp imm op in convertToGeneric %x\n", op);
                        return true;
                    }
                    else // ADDW
                    {
                        auto imm = ((opcode32 >> 15) & 0x800) | ((opcode32 >> 4) & 0x700) | (opcode32 & 0xFF);
                        addInstruction(loadImm(imm));
                        addInstruction(alu(GenOpcode::Add, reg(nReg), GenReg::Temp, reg(dstReg)), 4);
                    }
                    break;
                }

                case 0x2: // MOVW
                {
                    auto imm = ((opcode32 >> 4) & 0xF000) | ((opcode32 >> 15) & 0x800) | ((opcode32 >> 4) & 0x700) | (opcode32 & 0xFF);
                    addInstruction(loadImm(imm));
                    addInstruction(move(GenReg::Temp, reg(dstReg)), 4);
                    break;
                }

                case 0x5:
                {
                    if(nReg == 15) // ADR
                    {
                        printf("unhandled dp imm op in convertToGeneric %x\n", op);
                        return true;
                    }
                    else // SUBW
                    {
                        auto imm = ((opcode32 >> 15) & 0x800) | ((opcode32 >> 4) & 0x700) | (opcode32 & 0xFF);
                        addInstruction(loadImm(imm));
                        addInstruction(alu(GenOpcode::Subtract, reg(nReg), GenReg::Temp, reg(dstReg)), 4);
                    }
                    break;
                }

                case 0x6: // MOVT
                {
                    auto imm = ((opcode32 >> 4) & 0xF000) | ((opcode32 >> 15) & 0x800) | ((opcode32 >> 4) & 0x700) | (opcode32 & 0xFF);
                    // mask out top half
                    addInstruction(loadImm(0xFFFF));
                    addInstruction(alu(GenOpcode::And, reg(dstReg), GenReg::Temp, reg(dstReg)));
                    // or in new top half
                    addInstruction(loadImm(imm << 16));
                    addInstruction(alu(GenOpcode::Or, reg(dstReg), GenReg::Temp, reg(dstReg)), 4);
                    break;
                }

                case 0xA: // SBFX
                case 0xE: // UBFX
                {
                    int lsbit = ((opcode32 >> 10) & 0x1C) | ((opcode32 >> 6) & 3);
                    int width = (opcode32 & 0x1F) + 1;

                    assert(lsbit + width <= 32); // "UNPREDICTABLE"
        
                    // shift out high bits
                    addInstruction(loadImm(32 - (width + lsbit)));
                    addInstruction(alu(GenOpcode::ShiftLeft, reg(nReg), GenReg::Temp, reg(dstReg)));

                    // shift down and sign extend (if SBFX)
                    addInstruction(loadImm(32 - width));
                    addInstruction(alu(op == 0xA ? GenOpcode::ShiftRightArith : GenOpcode::ShiftRightLogic, reg(dstReg), GenReg::Temp, reg(dstReg)), 4);

                    break;
                }

                case 0xB: // BFI/BFC
                {
                    int msb = (opcode32 & 0x1F);
                    int lsb = ((opcode32 >> 10) & 0x1C) | ((opcode32 >> 6) & 3);

                    assert(msb >= lsb);

                    auto mask = (msb - lsb == 31) ? ~ 0 : (1 << (msb - lsb + 1)) - 1;

                    // d &= ~(mask << lsb)
                    addInstruction(loadImm(~(mask << lsb)));
                    addInstruction(alu(GenOpcode::And, reg(dstReg), GenReg::Temp, reg(dstReg)), nReg == 15 ? 4 : 0);

                    if(nReg != 15) // not BFC (BFI)
                    {
                        // n & mask
                        addInstruction(loadImm(mask));
                        addInstruction(alu(GenOpcode::And, GenReg::Temp, reg(nReg), GenReg::Temp2));
                        // ... << lsb
                        addInstruction(loadImm(lsb));
                        addInstruction(alu(GenOpcode::ShiftLeft, GenReg::Temp2, GenReg::Temp, GenReg::Temp2));

                        // d |= ...
                        addInstruction(alu(GenOpcode::Or, reg(dstReg), GenReg::Temp2, reg(dstReg)), 4);
                    }

                    break;
                }

                default:
                    printf("unhandled dp imm op in convertToGeneric %x\n", op);
                    return true;
            }
        }
        else
        {
            auto op = (opcode32 >> 21) & 0xF;
            bool setFlags = opcode32 & (1 << 20);

            auto nReg = (opcode32 >> 16) & 0xF;
            auto dstReg = (opcode32 >> 8) & 0xF;

            auto imm = ((opcode32 >> 12) & 7) | ((opcode32 >> 23) & 8);
            auto val8 = opcode32 & 0xFF;

            // get the modified imm
            uint32_t val;

            switch(imm)
            {
                case 0:
                    val = val8;
                    break;
                case 1:
                    val = val8 | val8 << 16;
                    break;
                case 2:
                    val = val8 << 8 | val8 << 24;
                    break;
                case 3:
                    val = val8 | val8 << 8 | val8 << 16 | val8 << 24;
                    break;
                default:
                {
                    //ROR
                    int rot = imm << 1 | val8 >> 7;
                    val = (val8 & 0x7F) | 0x80;

                    val = (val >> rot) | (val << (32 - rot));
                    bool carry = val & (1 << 31);

                    if(setFlags && op < 8/*not add/sub*/)
                    {
                        // update carry from shift if not something that sets it
                        if(carry)
                        {
                            addInstruction(loadImm(ARMv7MCore::Flag_C));
                            addInstruction(alu(GenOpcode::Or, GenReg::CPSR, GenReg::Temp, GenReg::CPSR));
                        }
                        else
                        {
                            addInstruction(loadImm(~ARMv7MCore::Flag_C));
                            addInstruction(alu(GenOpcode::And, GenReg::CPSR, GenReg::Temp, GenReg::CPSR));
                        }
                    }

                    break;
                }
            }

            switch(op)
            {
                case 0x0: // AND/TST
                {
                    auto dst = dstReg == 15 ? GenReg::Temp : reg(dstReg); // dst == PC is TST
                    addInstruction(loadImm(val));
                    addInstruction(alu(GenOpcode::And, reg(nReg), GenReg::Temp, dst), 4, setFlags ? (preserveV | preserveC | writeZ | writeN) : 0);
                    break;
                }
                case 0x1: // BIC
                {
                    addInstruction(loadImm(~val));
                    addInstruction(alu(GenOpcode::And, reg(nReg), GenReg::Temp, reg(dstReg)), 4, setFlags ? (preserveV | preserveC | writeZ | writeN) : 0);
                    break;
                }
                case 0x2: // MOV/ORR
                {
                    addInstruction(loadImm(val));
                    if(nReg == 15) // MOV
                        addInstruction(move(GenReg::Temp, reg(dstReg)), 4, setFlags ? (preserveV | preserveC | writeZ | writeN) : 0);
                    else // ORR
                        addInstruction(alu(GenOpcode::Or, reg(nReg), GenReg::Temp, reg(dstReg)), 4, setFlags ? (preserveV | preserveC | writeZ | writeN) : 0);
                    break;
                }
                case 0x3: // MVN/ORN
                {
                    addInstruction(loadImm(~val));
                    if(nReg == 15) // MVN
                        addInstruction(move(GenReg::Temp, reg(dstReg)), 4, setFlags ? (preserveV | preserveC | writeZ | writeN) : 0);
                    else // ORN
                        addInstruction(alu(GenOpcode::Or, reg(nReg), GenReg::Temp, reg(dstReg)), 4, setFlags ? (preserveV | preserveC | writeZ | writeN) : 0);
                    break;
                }
                case 0x4: // EOR/TEQ
                {
                    auto dst = dstReg == 15 ? GenReg::Temp : reg(dstReg); // dst == PC is TEQ
                    addInstruction(loadImm(val));
                    addInstruction(alu(GenOpcode::Xor, reg(nReg), GenReg::Temp, dst), 4, setFlags ? (preserveV | preserveC | writeZ | writeN) : 0);
                    break;
                }

                case 0x8: // ADD/CMN
                {
                    addInstruction(loadImm(val));
                    if(dstReg == 15) // CMN
                    {
                        assert(setFlags);
                        addInstruction(alu(GenOpcode::Add, GenReg::Temp, reg(nReg), GenReg::Temp), 4, setFlags ? (writeV | writeC | writeZ | writeN) : 0);
                    }
                    else
                        addInstruction(alu(GenOpcode::Add, reg(nReg), GenReg::Temp, reg(dstReg)), 4, setFlags ? (writeV | writeC | writeZ | writeN) : 0);
                    break;
                }

                case 0xA: // ADC
                {
                    addInstruction(loadImm(val));
                    addInstruction(alu(GenOpcode::AddWithCarry, reg(nReg), GenReg::Temp, reg(dstReg)), 4, setFlags ? (writeV | writeC | writeZ | writeN) : 0);
                    break;
                }
                case 0xB: // SBC
                {
                    addInstruction(loadImm(val));
                    addInstruction(alu(GenOpcode::SubtractWithCarry, reg(nReg), GenReg::Temp, reg(dstReg)), 4, setFlags ? (writeV | writeC | writeZ | writeN) : 0);
                    break;
                }

                case 0xD: // SUB/CMP
                {
                    addInstruction(loadImm(val));
                    if(dstReg == 15) // CMP
                        addInstruction(compare(reg(nReg), GenReg::Temp), 4, setFlags ? (writeV | writeC | writeZ | writeN) : 0);
                    else
                        addInstruction(alu(GenOpcode::Subtract, reg(nReg), GenReg::Temp, reg(dstReg)), 4, setFlags ? (writeV | writeC | writeZ | writeN) : 0);
                    break;
                }
                case 0xE: // RSB
                {
                    assert(dstReg != 15);
                    auto dst = reg(dstReg);
                    addInstruction(loadImm(val));
                    addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, reg(nReg), dst), 4, setFlags ? (writeV | writeC | writeZ | writeN) : 0);
                    break;
                }
                default:
                    printf("unhandled dp (mod imm) op in convertToGeneric %i\n", op);
                    return true;
            }
        }
    }
    else if(op1 == 0b11111)
    {
        if(opcode32 & (1 << 26)) // coprocessor
        {
            printf("unhandled op in convertToGeneric %08X (coproc)\n", opcode32 & 0xFF000000);
            return true;
        }
        else if((opcode32 & 0x3800000) == 0x3800000) // long multiply (accumulate), divide
        {
            printf("unhandled op in convertToGeneric %08X (lmuldiv)\n", opcode32 & 0xFF000000);
            return true;
        }
        else if((opcode32 & 0x3800000) == 0x3000000) // multiply (accumulate), diff
        {
            auto op1 = (opcode32 >> 20) & 7;
            auto op2 = (opcode32 >> 4) & 3;
        
            auto nReg = reg((opcode32 >> 16) & 0xF);
            auto aReg = (opcode32 >> 12) & 0xF;
            auto dstReg = reg((opcode32 >> 8) & 0xF);
            auto mReg = reg(opcode32 & 0xF);
        
            assert(((opcode32 >> 6) & 3) == 0);

            if(op1 == 0 && op2 < 2) // MUL/MLA/MLS
            {
                bool hasA = aReg != 15;

                addInstruction(alu(GenOpcode::Multiply, nReg, mReg, hasA ? GenReg::Temp : dstReg), hasA ? 0 : 4);

                if(hasA) // MLA/MLS
                    addInstruction(alu(op2 == 1 ? GenOpcode::Subtract : GenOpcode::Add, reg(aReg), GenReg::Temp, dstReg), 4);
            }
            else
            {
                printf("unhandled op in convertToGeneric %08X (muldiff)\n", opcode32 & 0xFFF00000);
                return true;
            }
        }
        else if(opcode32 & (1 << 25)) // data processing (register)
        {
            auto op1 = (opcode32 >> 20) & 0xF;
            auto op2 = (opcode32 >> 4) & 0xF;
        
            auto nReg = (opcode32 >> 16) & 0xF;
        
            if((op1 & 0b1000) && !(op2 & 0b1000)) // parallel add/sub
            {
                printf("unhandled op in convertToGeneric %08X (dp reg p add/sub)\n", opcode32 & 0xFFF000F0);
                return true;
            }
            else if(op2 == 0)
            {
                bool setFlags = opcode32 & (1 << 20);
                auto dReg = reg((opcode32 >> 8) & 0xF);
                auto mReg = reg(opcode32 & 0xF);

                switch(op1 >> 1)
                {
                    // LSL, LSR, ASR, ROR
                    case 0: // LSL
                        addInstruction(alu(GenOpcode::ShiftLeft, reg(nReg), mReg, dReg), 4, setFlags ? (preserveV | preserveC | writeC | writeZ | writeN) : 0);
                        break;
                    case 1: // LSR
                        addInstruction(alu(GenOpcode::ShiftRightLogic, reg(nReg), mReg, dReg), 4, setFlags ? (preserveV | preserveC | writeC | writeZ | writeN) : 0);
                        break;
                    case 2: // ASR
                        addInstruction(alu(GenOpcode::ShiftRightArith, reg(nReg), mReg, dReg), 4, setFlags ? (preserveV | preserveC | writeC | writeZ | writeN) : 0);
                        break;
                    case 3: // ROR
                        addInstruction(alu(GenOpcode::RotateRight, reg(nReg), mReg, dReg), 4, setFlags ? (preserveV | preserveC | writeC | writeZ | writeN) : 0);
                        break;
                    
                    default:
                        printf("unhandled op in convertToGeneric %08X (dp reg shift)\n", opcode32 & 0xFFF00000);
                }

                return true;
            }
            else if(op2 & 0b1000)
            {
                switch(op1)
                {
                    case 0: // SXTAH/SXTH
                    case 4: // SXTAB/SXTB
                    {
                        auto dReg = reg((opcode32 >> 8) & 0xF);
                        auto mReg = reg(opcode32 & 0xF);
                        int rotation = (opcode32 >> 1) & 0x18;

                        // rotate
                        addInstruction(loadImm(rotation));
                        addInstruction(alu(GenOpcode::RotateRight, mReg, GenReg::Temp, GenReg::Temp));

                        // extend
                        GenOpInfo extOp{};
                        extOp.opcode = op1 == 0 ? GenOpcode::SignExtend16 : GenOpcode::SignExtend8;
                        
                        if(nReg == 15) // SXTH
                        {
                            extOp.dst[0] = static_cast<uint8_t>(dReg);
                            addInstruction(extOp, 4);
                        }
                        else // SXTAH
                        {
                            addInstruction(extOp);
                            // add
                            addInstruction(alu(GenOpcode::Add, GenReg::Temp, reg(nReg), dReg), 4);
                        }
                        break;
                    }
                    case 1: // UXTAH/UXTH
                    case 5: // UXTAB/UXTB
                    {
                        auto dReg = reg((opcode32 >> 8) & 0xF);
                        auto mReg = reg(opcode32 & 0xF);
                        int rotation = (opcode32 >> 1) & 0x18;

                        // rotate
                        addInstruction(loadImm(rotation));
                        addInstruction(alu(GenOpcode::RotateRight, mReg, GenReg::Temp, GenReg::Temp2));

                        // mask
                        addInstruction(loadImm(op1 == 1 ? 0xFFFF : 0xFF));
                        auto maskOp = alu(GenOpcode::And, GenReg::Temp, GenReg::Temp2, GenReg::Temp);

                        if(nReg == 15) // UXTH
                        {
                            maskOp.dst[0] = static_cast<uint8_t>(dReg);
                            addInstruction(maskOp, 4);
                        }
                        else // UXTAH
                        {
                            addInstruction(maskOp);
                            // add
                            addInstruction(alu(GenOpcode::Add, GenReg::Temp, reg(nReg), dReg), 4);
                        }
                        break;
                    }


                    default:
                        printf("unhandled dp (reg) op in convertToGeneric %i\n", op1);
                        return true;
                }
            }
            else
            {
                printf("unhandled op in convertToGeneric %08X (dp reg)\n", opcode32 & 0xFFF000F0);
                return true;
            }
        }
        else if((opcode32 & 0x700000) == 0x500000) // load word
        {
            auto op1 = (opcode32 >> 23) & 3;
            auto op2 = (opcode32 >> 6) & 0x3F;

            auto baseReg = (opcode32 >> 16) & 0xF;
            auto dstReg = (opcode32 >> 12) & 0xF;

            if(baseReg == 15) // LDR (literal)
            {
                bool add = opcode32 & (1 << 23);

                auto offset = (opcode32 & 0xFFF);

                uint32_t addr = pc & ~2;
                
                if(add)
                    addr += offset;
                else
                    addr -= offset;

                addInstruction(loadImm(mem.read<uint32_t>(addr)));
                addInstruction(move(GenReg::Temp, reg(dstReg)), 4);
            }
            else if(op1 == 0 && op2 == 0) // LDR (register)
            {
                auto mReg = reg(opcode32 & 0xF);
                auto shift = (opcode32 >> 4) & 3;

                loadWithOffset(genBlock, 4, reg(baseReg), mReg, shift, reg(dstReg), 4);
            }
            else if(op1 == 0 && (op2 & 0x3C) == 0x38) // LDRT
            {
                printf("unhandled op in convertToGeneric %08X (ldrt)\n", opcode32 & 0xFF000000);
                return true;
            }
            else // LDR (immediate)
            {
                bool isPC = dstReg == 15;
                auto dst = isPC ? GenReg::Temp2 : reg(dstReg);

                if(op1 == 1) // +12 bit imm
                {
                    auto offset = (opcode32 & 0xFFF);
                    loadWithOffset(genBlock, 4, reg(baseReg), offset, dst, 0, 4);
                }
                else
                {
                    auto offset = (opcode32 & 0xFF);

                    bool writeback = opcode32 & (1 << 8);
                    bool add = opcode32 & (1 << 9);
                    bool index = opcode32 & (1 << 10);

                    loadWithOffset(genBlock, 4, reg(baseReg), add ? offset : -offset, writeback, index, dst, 4);
                }

                if(isPC)
                {
                    genBlock.instructions.back().len = 0;

                    // clear thumb bit and jump
                    addInstruction(loadImm(~1u));
                    addInstruction(alu(GenOpcode::And, GenReg::Temp, GenReg::Temp2, GenReg::Temp));
                    addInstruction(jump(GenCondition::Always, GenReg::Temp, 0), 4);

                    if(pc > maxBranch)
                        return true;
                }
            }
        }
        else if((opcode32 & 0x700000) == 0x300000) // load halfword, memory hints
        {
            auto op1 = (opcode32 >> 23) & 3;
            auto op2 = (opcode32 >> 6) & 0x3F;
        
            auto baseReg = (opcode32 >> 16) & 0xF;
            auto dstReg = (opcode32 >> 12) & 0xF;

            bool isSigned = op1 & 2;

            if(dstReg == 15) // unallocated hints
            {
                assert(false);
                return true;
            }
            else if(baseReg == 15) // LDR(S)H (literal)
            {
                printf("unhandled op in convertToGeneric %08X (load h lit)\n", opcode32 & 0xFF000000);
                return true;
            }
            else if(!(op1 & 1) && op2 == 0) // LDR(S)H (register)
            {
                auto mReg = reg(opcode32 & 0xF);
                auto shift = (opcode32 >> 4) & 3;

                loadWithOffset(genBlock, 2, reg(baseReg), mReg, shift, reg(dstReg), 4, isSigned ? GenOp_SignExtend : 0);
            }
            else // LDR(S)H (immediate)
            {
                if(op1 & 1) // + 12 bit imm
                {
                    auto offset = (opcode32 & 0xFFF);
                    loadWithOffset(genBlock, 2, reg(baseReg), offset, reg(dstReg), 0, 4, isSigned ? GenOp_SignExtend : 0);
                }
                else // +/- 8 bit imm
                {
                    auto offset = (opcode32 & 0xFF);

                    bool writeback = opcode32 & (1 << 8);
                    bool add = opcode32 & (1 << 9);
                    bool index = opcode32 & (1 << 10);
                    // !writeback && add && index is LDR(S)HT, but we can just handle it the same as LDR(S)H

                    loadWithOffset(genBlock, 2, reg(baseReg), add ? offset : -offset, writeback, index, reg(dstReg), 4, isSigned ? GenOp_SignExtend : 0);
                }
            }
        }
        else if((opcode32 & 0x700000) == 0x100000) // load byte, memory hints
        {
            auto op1 = (opcode32 >> 23) & 3;
            auto op2 = (opcode32 >> 6) & 0x3F;
        
            auto baseReg = (opcode32 >> 16) & 0xF;
            auto dstReg = (opcode32 >> 12) & 0xF;

            bool isSigned = op1 & 2;

            if(dstReg == 15) // preload
            {
                // this is a hint
                // we do need to emit something though
                GenOpInfo op{};
                op.opcode = GenOpcode::NOP;
                op.cycles = 1;

                addInstruction(op, 4);
            }
            else if(baseReg == 15) // LDR(S)B (literal)
            {
                printf("unhandled op in convertToGeneric %08X (load b lit)\n", opcode32 & 0xFF000000);
                return true;
            }
            else if(!(op1 & 1) && op2 == 0) // LDR(S)B (register)
            {
                auto mReg = reg(opcode32 & 0xF);
                auto shift = (opcode32 >> 4) & 3;

                loadWithOffset(genBlock, 1, reg(baseReg), mReg, shift, reg(dstReg), 4, isSigned ? GenOp_SignExtend : 0);
            }
            else // LDR(S)B (immediate)
            {
                if(op1 & 1) // + 12 bit imm
                {
                    auto offset = (opcode32 & 0xFFF);
                    loadWithOffset(genBlock, 1, reg(baseReg), offset, reg(dstReg), 0, 4, isSigned ? GenOp_SignExtend : 0);
                }
                else // +/- 8 bit imm
                {
                    auto offset = (opcode32 & 0xFF);

                    bool writeback = opcode32 & (1 << 8);
                    bool add = opcode32 & (1 << 9);
                    bool index = opcode32 & (1 << 10);
                    // !writeback && add && index is LDR(S)BT, but we can just handle it the same as LDR(S)B

                    loadWithOffset(genBlock, 1, reg(baseReg), add ? offset : -offset, writeback, index, reg(dstReg), 4, isSigned ? GenOp_SignExtend : 0);
                }
            }
        }
        else // store single data item
        {
            auto op1 = (opcode32 >> 21) & 7;
            auto op2 = (opcode32 >> 6) & 0x3F;

            auto baseReg = reg((opcode32 >> 16) & 0xF);
            int dst = (opcode32 >> 12) & 0xF;

            if(dst == 15)
            {
                printf("unhandled str pc in convertToGeneric %08X\n", opcode32 & 0xFFF00000);
                return true;
            }

            auto dstReg = reg(dst);

            int width = 1 << (op1 & 3);

            if(op1 & 4) // 12 bit immediate
            {
                auto offset = (opcode32 & 0xFFF);
                storeWithOffset(genBlock, width, baseReg, offset, dstReg, 0, 4);
            }
            else if(op2 & 0x20) // 8 bit immediate
            {
                auto offset = (opcode32 & 0xFF);

                bool writeback = opcode32 & (1 << 8);
                bool add = opcode32 & (1 << 9);
                bool index = opcode32 & (1 << 10);

                if(index)
                {
                    addInstruction(loadImm(add ? offset : -offset));
                    addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, GenReg::Temp));
                }

                addInstruction(store(width, index ? GenReg::Temp : baseReg, dstReg, 0), writeback ? 0 : 4);

                // write back adjusted base
                if(writeback)
                {
                    // need to redo add even if index is true (can't reuse the temp)
                    addInstruction(loadImm(add ? offset : -offset));
                    addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, baseReg), 4);
                }
            }
            else // register
            {
                auto offReg = reg(opcode32 & 0xF);
                auto shift = (opcode32 >> 4) & 3;

                // base + off << shift
                addInstruction(loadImm(shift));
                addInstruction(alu(GenOpcode::ShiftLeft, offReg, GenReg::Temp, GenReg::Temp));
                addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, GenReg::Temp));

                // do the store
                addInstruction(store(width, GenReg::Temp, dstReg, 0), 4);
            }
        }
    }
    else
    {
        printf("unhandled op in convertToGeneric %08X\n", opcode32 & 0xF8000000);
        return true;
    }

    return false;
}

void ARMv7MRecompiler::compileEntry()
{
    auto entryPtr = target.compileEntry(curCodePtr, codeBufSize);
    entryFunc = reinterpret_cast<CompiledFunc>(entryPtr);
}

// wrappers around member funcs
uint8_t ARMv7MRecompiler::readMem8(ARMv7MCore *cpu, uint32_t addr)
{
    return cpu->readMem8(addr);
}

uint32_t ARMv7MRecompiler::readMem16(ARMv7MCore *cpu, uint32_t addr)
{
    return cpu->readMem16(addr);
}

uint32_t ARMv7MRecompiler::readMem32(ARMv7MCore *cpu, uint32_t addr)
{
    return cpu->readMem32(addr);
}

void ARMv7MRecompiler::writeMem8(ARMv7MCore *cpu, uint32_t addr, uint8_t data)
{
    invalidateCode(cpu, addr);
    cpu->writeMem8(addr, data);
}

void ARMv7MRecompiler::writeMem16(ARMv7MCore *cpu, uint32_t addr, uint16_t data)
{
    invalidateCode(cpu, addr);
    cpu->writeMem16(addr, data);
}

void ARMv7MRecompiler::writeMem32(ARMv7MCore *cpu, uint32_t addr, uint32_t data)
{
    invalidateCode(cpu, addr);
    cpu->writeMem32(addr, data);
}

void ARMv7MRecompiler::invalidateCode(ARMv7MCore *cpu, uint32_t addr)
{
    auto &compiler = cpu->compiler;
    
    if(addr < compiler.minRAMCode || addr >= compiler.maxRAMCode)
        return;

    auto end = compiler.compiled.end();

    for(auto it = compiler.compiled.begin(); it != end;)
    {
        if(addr >= it->first && addr < it->second.endPC)
        {
#ifdef RECOMPILER_DEBUG
            printf("invalidate compiled code @%08X in %08X-%08X\n", addr, it->first, it->second.endPC);
#endif
            // rewind if last code compiled
            // TODO: reclaim memory in other cases
            if(it->second.endPtr == compiler.curCodePtr)
                compiler.curCodePtr = it->second.startPtr;

            // invalidate any saved return addresses
            for(auto &saved : compiler.savedExits)
            {
                if(std::get<1>(saved) >= it->first && std::get<1>(saved) <= it->second.endPC)
                    saved = {nullptr, 0, 0};
            }

            it = compiler.compiled.erase(it);

            continue; // might have compiled the same code more than once
        }

        // past this address, stop
        if(it->first > addr)
            break;

        ++it;
    }
}
