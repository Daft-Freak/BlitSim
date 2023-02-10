#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib> //exit
#include <cstring>
#include <limits>
#include <utility>

#include "ARMv6MCore.h"

#ifdef _MSC_VER
#define __builtin_unreachable() __assume(false)
#endif

// FIXME: this still thinks it's an ARMv4T

ARMv6MCore::ARMv6MCore(MemoryBus &mem) : mem(mem)
{}

void ARMv6MCore::reset()
{
    for(auto &reg: regs)
        reg = 0;

    cpsr = Flag_T;
    primask = control = 0;
    curSP = Reg::MSP;

    sleeping = false;
    eventFlag = false;

    itState = 0;

    clock.reset();

    exceptionActive = exceptionPending = 0;
    needException = false;

    for(auto &reg : sysTickRegs)
        reg = 0;

    nvicEnabled = 0;
    for(auto &reg : nvicPriority)
        reg = 0;

    for(auto &reg : scbRegs)
        reg = 0;

    for(auto &reg : mpuRegs)
        reg = 0;

    // CPUID
    // TODO: maybe not harcoded M0+ if I ever reuse this...
    scbRegs[0] = 0x41 << 24 | 0xC << 16/*ARMv6-M*/ | 0xC60 << 4/*Cortex-M0+*/ | 1;
    // CCR
    scbRegs[5] = 0x3F8;

    // MPU_TYPE
    mpuRegs[0] = 8 << 8;

    mem.reset();

    int cycles;
    reg(Reg::SP) = mem.read<uint32_t>(0, cycles, false); // MSP
    updateTHUMBPC(mem.read<uint32_t>(4, cycles, false) & ~ 1); // Reset vector
}

unsigned int ARMv6MCore::run(int ms)
{
    auto targetTime = clock.getTargetTime(ms);
    return update(targetTime);
}

unsigned int ARMv6MCore::update(uint64_t target)
{
    unsigned int cycles = 0;

    while(clock.getTime() < target)
    {
        uint32_t exec = 1;

        if(!sleeping)
        {
            // CPU
            exec = executeTHUMBInstruction();

            // advance IT
            // outside of executeTHUMBInstruction as it needs to be after executing the instruction...
            if(inIT())
            {
                if(!itStart)
                    advanceIT();
                itStart = false;
            }
        }

        // loop until not halted or DMA was triggered
        uint64_t curTime;
        do
        {
            // interrupts?

            if(!(primask & 1) && needException)
                exec += handleException();

            clock.addCycles(exec);
            cycles += exec;

            // update systick if using cpu clock
            uint32_t mask = (1 << 0)/*ENABLE*/ | (1 << 2)/*CLKSOURCE*/;
            if((sysTickRegs[0]/*SYST_CSR*/ & mask) == mask)
                updateSysTick(exec);

            curTime = clock.getTime();

            if(sleeping && curTime < target)
            {
                // skip ahead
                auto skipTarget = target;
                exec = std::max(UINT32_C(1), clock.getCyclesToTime(skipTarget, true));
            }
        }
        while(sleeping && curTime < target);
    }

    return cycles;
}

void ARMv6MCore::setSP(uint32_t val)
{
    reg(Reg::SP) = val;
}

void ARMv6MCore::runCall(uint32_t addr, uint32_t r0)
{
    // TODO: save/restore state for faked interrupts?

    loReg(Reg::R0) = r0;

    // fake a BL
    loReg(Reg::LR) = 0x8FFFFFF; // somewhere invalid in flash
    updateTHUMBPC(addr & ~ 1);

    while(loReg(Reg::PC) != 0x8FFFFFE + 2)
    {
        uint32_t exec = 1;

        if(!sleeping)
        {
            // CPU
            exec = executeTHUMBInstruction();

            // advance IT
            // outside of executeTHUMBInstruction as it needs to be after executing the instruction...
            if(inIT())
            {
                if(!itStart)
                    advanceIT();
                itStart = false;
            }
        }

        // update systick if using cpu clock
        uint32_t mask = (1 << 0)/*ENABLE*/ | (1 << 2)/*CLKSOURCE*/;
        if((sysTickRegs[0]/*SYST_CSR*/ & mask) == mask)
            updateSysTick(exec);
    }
}

void ARMv6MCore::setPendingIRQ(int n)
{
    exceptionPending |= 1ull << (n + 16);
    checkPendingExceptions();
}

void ARMv6MCore::setEvent()
{
    eventFlag = true;
    if(sleeping)
        sleeping = false;
}

uint32_t ARMv6MCore::readReg(uint32_t addr)
{
    switch(addr & 0xFFFFFFF)
    {
        case 0xE010: // SYST_CSR
        case 0xE014: // SYST_RVR
        case 0xE018: // SYST_CVR
        case 0xE01C: // SYST_CALIB
            updateSysTick();
            return sysTickRegs[(addr & 0xF) / 4];
        
        case 0xE100: // NVIC_ISER
        case 0xE180: // NVIC_ICER
            return nvicEnabled;
        case 0xE200: // NVIC_ISPR
        case 0xE280: // NVIC_IPCR
            return exceptionPending >> 16;
        case 0xE400: // NVIC_IPR0
        case 0xE404: // NVIC_IPR1
        case 0xE408: // NVIC_IPR2
        case 0xE40C: // NVIC_IPR3
        case 0xE410: // NVIC_IPR4
        case 0xE414: // NVIC_IPR5
        case 0xE418: // NVIC_IPR6
        case 0xE41C: // NVIC_IPR7
            return nvicPriority[(addr & 0xFF) / 4];

        case 0xED00: // CPUID
        case 0xED04: // ICSR
        case 0xED08: // VTOR
        case 0xED0C: // AIRCR
        case 0xED10: // SCR
        case 0xED14: // CCR
        case 0xED1C: // SHPR2
        case 0xED20: // CHPR3
        case 0xED24: // SHCSR
            return scbRegs[(addr & 0xFF) / 4];

        case 0xED90: // MPU_TYPE
        case 0xED94: // MPU_CTRL
        case 0xED98: // MPU_RNR
        case 0xED9C: // MPU_RBAR
        case 0xEDA0: // MPU_RASR
            return mpuRegs[((addr & 0xFF) - 0x90) / 4];
    }

    printf("CPUI R %08X\n", addr);
    return 0;
}

void ARMv6MCore::writeReg(uint32_t addr, uint32_t data)
{
    switch(addr & 0xFFFFFFF)
    {
        case 0xE010: // SYST_CSR
        case 0xE014: // SYST_RVR
            updateSysTick();
            sysTickRegs[(addr & 0xF) / 4] = data;
            return;
        case 0xE018: // SYST_CVR
            updateSysTick();
            sysTickRegs[2] = sysTickRegs[1];
            return;
        
        case 0xE100: // NVIC_ISER
            nvicEnabled |= data;
            checkPendingExceptions();
            return;
        case 0xE180: // NVIC_ICER
            nvicEnabled &= ~data; //
            checkPendingExceptions();
            return;
        case 0xE200: // NVIC_ISPR
            exceptionPending |= static_cast<uint64_t>(data) << 16;
            checkPendingExceptions();
            return;
        case 0xE280: // NVIC_IPCR
            exceptionPending &= ~(static_cast<uint64_t>(data) << 16);
            checkPendingExceptions();
            return;
        case 0xE400: // NVIC_IPR0
        case 0xE404: // NVIC_IPR1
        case 0xE408: // NVIC_IPR2
        case 0xE40C: // NVIC_IPR3
        case 0xE410: // NVIC_IPR4
        case 0xE414: // NVIC_IPR5
        case 0xE418: // NVIC_IPR6
        case 0xE41C: // NVIC_IPR7
            nvicPriority[(addr & 0xFF) / 4] = data;
            return;

        //case 0xED04: // ICSR
        case 0xED08: // VTOR
        //case 0xED0C: // AIRCR
        case 0xED10: // SCR
        case 0xED1C: // SHPR2
        case 0xED20: // CHPR3
        case 0xED24: // SHCSR
            scbRegs[(addr & 0xFF) / 4] = data;
            return;

        case 0xED94: // MPU_CTRL
        case 0xED98: // MPU_RNR
        case 0xED9C: // MPU_RBAR
        case 0xEDA0: // MPU_RASR
            mpuRegs[((addr & 0xFF) - 0x90) / 4] = data;
            return;
    }

    printf("CPUI W %08X = %08X\n", addr, data);
}

uint8_t ARMv6MCore::readMem8(uint32_t addr, int &cycles, bool sequential)
{
    return mem.read<uint8_t>(addr, cycles, sequential);
}

uint16_t ARMv6MCore::readMem16(uint32_t addr, int &cycles, bool sequential)
{
    return mem.read<uint16_t>(addr, cycles, sequential);
}


uint32_t ARMv6MCore::readMem32(uint32_t addr, int &cycles, bool sequential)
{
    return mem.read<uint32_t>(addr, cycles, sequential);
}

void ARMv6MCore::writeMem8(uint32_t addr, uint8_t data, int &cycles, bool sequential)
{
    mem.write<uint8_t>(addr, data, cycles, sequential);
}

void ARMv6MCore::writeMem16(uint32_t addr, uint16_t data, int &cycles, bool sequential)
{
    mem.write<uint16_t>(addr, data, cycles, sequential);
}

void ARMv6MCore::writeMem32(uint32_t addr, uint32_t data, int &cycles, bool sequential)
{
    mem.write<uint32_t>(addr, data, cycles, sequential);
}

void ARMv6MCore::advanceIT()
{
    if((itState & 7) == 0)
        itState = 0; // done
    else
        itState = (itState & 0xE0) | ((itState << 1) & 0x1F);
}

int ARMv6MCore::executeTHUMBInstruction()
{
    auto &pc = loReg(Reg::PC); // not a low reg, but not banked
    uint16_t opcode = decodeOp;

    decodeOp = fetchOp;

    pc += 2;
    if(pcPtr)
    {
        auto thumbPCPtr = reinterpret_cast<const uint16_t *>(pcPtr + pc);
        assert(mem.verifyPointer(thumbPCPtr, pc));
        fetchOp = *thumbPCPtr;
    }
    else
    {
        int tmp;
        fetchOp = mem.read<uint16_t>(pc, tmp, true);
    }

    if(inIT())
    {
        auto cond = itState >> 4;

        // check condition
        bool result = false;

        switch(cond >> 1)
        {
            case 0: // EQ/NE
                result = cpsr & Flag_Z;
                break;
            case 1: // CS/CC
                result = cpsr & Flag_C;
                break;
            case 2: // MI/PL
                result = cpsr & Flag_N;
                break;
            case 3: // VS/VC
                result = cpsr & Flag_V;
                break;
            case 4: // HI/LS
                result = (cpsr & Flag_C) && !(cpsr & Flag_Z);
                break;
            case 5: // GE/LT
                result = !!(cpsr & Flag_N) == !!(cpsr & Flag_V);
                break;
            case 6: // GT/LE
                result = !!(cpsr & Flag_N) == !!(cpsr & Flag_V) && !(cpsr & Flag_Z);
                break;
            case 7: // AL
                result = true;
                break;
        }

        if((cond & 1) && cond != 0xF)
            result = !result;

        if(!result)
        {
            if((opcode >> 12) == 0xF || (opcode >> 11) == 0x1D) //32bit
            {
                // skip second half
                decodeOp = fetchOp;

                pc += 2;
                if(pcPtr)
                {
                    auto thumbPCPtr = reinterpret_cast<const uint16_t *>(pcPtr + pc);
                    assert(mem.verifyPointer(thumbPCPtr, pc));
                    fetchOp = *thumbPCPtr;
                }
                else
                {
                    int tmp;
                    fetchOp = mem.read<uint16_t>(pc, tmp, true);
                }

                return pcSCycles * 2;
            }
            return pcSCycles;
        }
    }

    switch(opcode >> 12)
    {
        case 0x0: // format 1
            return doTHUMB01MoveShifted(opcode, pc);
        case 0x1: // formats 1-2
            return doTHUMB0102(opcode, pc);
        case 0x2: // format 3, mov/cmp immediate
        case 0x3: // format 3, add/sub immediate
            return doTHUMB03(opcode, pc);
        case 0x4: // formats 4-6
            return doTHUMB040506(opcode, pc);
        case 0x5: // formats 7-8
            return doTHUMB0708(opcode, pc);
        case 0x6: // format 9, load/store with imm offset (words)
            return doTHUMB09LoadStoreWord(opcode, pc);
        case 0x7: // ... (bytes)
            return doTHUMB09LoadStoreByte(opcode, pc);
        case 0x8: // format 10, load/store halfword
            return doTHUMB10LoadStoreHalf(opcode, pc);
        case 0x9: // format 11, SP-relative load/store
            return doTHUMB11SPRelLoadStore(opcode, pc);
        case 0xA: // format 12, load address
            return doTHUMB12LoadAddr(opcode, pc);
        case 0xB: // formats 13-14
            return doTHUMBMisc(opcode, pc);
        case 0xC: // format 15, multiple load/store
            return doTHUMB15MultiLoadStore(opcode, pc);
        case 0xD: // formats 16-17
            return doTHUMB1617(opcode, pc);
        case 0xE: // format 18, unconditional branch
            return doTHUMB18UncondBranch(opcode, pc);
        case 0xF: // format 19, long branch with link
            return doTHUMB32BitInstruction(opcode, pc);
    }

    __builtin_unreachable();
}

int ARMv6MCore::doTHUMB01MoveShifted(uint16_t opcode, uint32_t pc)
{
    auto instOp = (opcode >> 11) & 0x1;
    auto srcReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto offset = (opcode >> 6) & 0x1F;
    auto res = loReg(srcReg);

    uint32_t carry;
    switch(instOp)
    {
        case 0: // LSL
            if(offset != 0)
            {
                carry = res & (1 << (32 - offset)) ? Flag_C : 0;
                res <<= offset;
            }
            else
                carry = cpsr & Flag_C; // preserve
            break;
        case 1: // LSR
            if(!offset) offset = 32; // shift by 0 is really 32

            carry = res & (1 << (offset - 1)) ? Flag_C : 0;
            if(offset == 32)
                res = 0;
            else
                res >>= offset;
            break;
        default:
            assert(!"Invalid format 1 shift type");
    }

    loReg(dstReg) = res;

    if(!inIT())
    {
        cpsr = (cpsr & 0x1FFFFFFF)
             | (res & signBit ? Flag_N : 0)
             | (res == 0 ? Flag_Z : 0)
             | carry;
    }

    return pcSCycles;
}

int ARMv6MCore::doTHUMB0102(uint16_t opcode, uint32_t pc)
{
    auto instOp = (opcode >> 11) & 0x3;
    auto srcReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    if(instOp == 3) // format 2, add/sub
    {
        bool isImm = opcode & (1 << 10);
        bool isSub = opcode & (1 << 9);
        uint32_t op1 = loReg(srcReg), op2 = (opcode >> 6) & 7;

        uint32_t res;

        if(!isImm)
            op2 = loReg(static_cast<Reg>(op2));

        uint32_t carry, overflow;

        cpsr &= 0x0FFFFFFF;

        if(isSub)
        {
            res = op1 - op2;
            carry = !(res > op1) ? Flag_C : 0;
            overflow = ((op1 ^ op2) & (op1 ^ res)) & signBit;
        }
        else
        {
            res = op1 + op2;
            carry = res < op1 ? Flag_C : 0;
            overflow = (~(op1 ^ op2) & (op1 ^ res)) & signBit;
        }

        loReg(dstReg) = res;

        if(!inIT())
        {
            cpsr |= (res & signBit ? Flag_N : 0)
                 | (res == 0 ? Flag_Z : 0)
                 | carry
                 | (overflow >> 3);
        }
    }
    else // format 1, move shifted register
    {
        auto offset = (opcode >> 6) & 0x1F;
        auto res = loReg(srcReg);

        assert(instOp == 2); // others are handled elsewhere

        uint32_t carry;

        if(!offset) offset = 32;

        auto sign = res & signBit;
        carry = res & (1 << (offset - 1)) ? Flag_C : 0;
        if(offset == 32)
            res = sign ? 0xFFFFFFFF : 0;
        else
            res = static_cast<int32_t>(res) >> offset;

        loReg(dstReg) = res;

        if(!inIT())
        {
            cpsr = (cpsr & 0x1FFFFFFF)
                 | (res & signBit ? Flag_N : 0)
                 | (res == 0 ? Flag_Z : 0)
                 | carry;
        }
    }

    return pcSCycles;
}

int ARMv6MCore::doTHUMB03(uint16_t opcode, uint32_t pc)
{
    auto instOp = (opcode >> 11) & 0x3;
    auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
    uint8_t offset = opcode & 0xFF;

    auto dst = loReg(dstReg);

    uint32_t res;
    uint32_t carry, overflow;

    switch(instOp)
    {
        case 0: // MOV
            loReg(dstReg) = offset;
            if(!inIT())
                cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (offset == 0 ? Flag_Z : 0); // N not possible
            break;
        case 1: // CMP
            res = dst - offset;
            carry = !(res > dst) ? Flag_C : 0;
            overflow = (dst & ~res) & signBit; // offset cannot be negative, simplifies overflow checks
            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C | Flag_V)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3); // overflow is either 0 or 0x80000000, shift it down
            break;
        case 2: // ADD
            loReg(dstReg) = res = dst + offset;
            carry = res < dst ? Flag_C : 0;
            overflow = (~dst & res) & signBit;
            if(!inIT())
                cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C | Flag_V)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        case 3: // SUB
            loReg(dstReg) = res = dst - offset;
            carry = !(res > dst) ? Flag_C : 0;
            overflow = (dst & ~res) & signBit;
            if(!inIT())
                cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C | Flag_V)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        default:
            __builtin_unreachable();
    }

    return pcSCycles;
}

int ARMv6MCore::doTHUMB040506(uint16_t opcode, uint32_t pc)
{
    if(opcode & (1 << 11)) // format 6, PC-relative load
        return doTHUMB06PCRelLoad(opcode, pc);
    else if(opcode & (1 << 10)) // format 5, Hi reg/branch exchange
        return doTHUMB05HiReg(opcode, pc);
    else // format 4, alu
        return doTHUMB04ALU(opcode, pc);
}

int ARMv6MCore::doTHUMB04ALU(uint16_t opcode, uint32_t pc)
{
    auto instOp = (opcode >> 6) & 0xF;
    auto srcReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto op1 = loReg(dstReg);
    auto op2 = loReg(srcReg);

    uint32_t res;
    uint32_t carry, overflow; // preserved if logical op

    switch(instOp)
    {
        case 0x0: // AND
            loReg(dstReg) = res = op1 & op2;
            if(!inIT())
                cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
        case 0x1: // EOR
            loReg(dstReg) = res = op1 ^ op2;
            if(!inIT())
                cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
        case 0x2: // LSL
            carry = cpsr & Flag_C;

            if(op2 >= 32)
            {
                carry = op2 == 32 ? (op1 & 1) : 0;
                carry = carry ? Flag_C : 0;
                loReg(dstReg) = res = 0;
            }
            else if(op2)
            {
                carry = op1 & (1 << (32 - op2)) ? Flag_C : 0;
                loReg(dstReg) = res = op1 << op2;
            }
            else
                loReg(dstReg) = res = op1;

            if(!inIT())
                cpsr = (cpsr & ~(Flag_C | Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry;
            break;
        case 0x3: // LSR
            carry = cpsr & Flag_C;

            if(op2 >= 32)
            {
                carry = op2 == 32 ? (op1 & (1 << 31)) : 0;
                carry = carry ? Flag_C : 0;
                loReg(dstReg) = res = 0;
            }
            else if(op2)
            {
                carry = op1 & (1 << (op2 - 1)) ? Flag_C : 0;
                loReg(dstReg) = res = op1 >> op2;
            }
            else
                loReg(dstReg) = res = op1;

            if(!inIT())
                cpsr = (cpsr & ~(Flag_C | Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry;
            break;
        case 0x4: // ASR
        {
            carry = cpsr & Flag_C;
            auto sign = op1 & signBit;
            if(op2 >= 32)
            {
                carry = sign ? Flag_C : 0;
                loReg(dstReg) = res = sign ? 0xFFFFFFFF : 0;
            }
            else if(op2)
            {
                carry = op1 & (1 << (op2 - 1)) ? Flag_C : 0;
                res = static_cast<int32_t>(op1) >> op2;

                loReg(dstReg) = res;
            }
            else
                loReg(dstReg) = res = op1;

            if(!inIT())
                cpsr = (cpsr & ~(Flag_C | Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry;
            break;
        }
        case 0x5: // ADC
        {
            int c = (cpsr & Flag_C) ? 1 : 0;
            loReg(dstReg) = res = op1 + op2 + c;
            carry = res < op1 || (res == op1 && c) ? Flag_C : 0;
            overflow = ~((op1 ^ op2) & signBit) & ((op1 ^ res) & signBit);
            if(!inIT())
                cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        }
        case 0x6: // SBC
        {
            int c = (cpsr & Flag_C) ? 1 : 0;
            loReg(dstReg) = res = op1 - op2 + c - 1;
            carry = !(op2 > op1 || (op2 == op1 && !c)) ? Flag_C : 0;
            overflow = ((op1 ^ op2) & signBit) & ((op1 ^ res) & signBit);
            if(!inIT())
                cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        }
        case 0x7: // ROR
        {
            carry = cpsr & Flag_C;
            int shift = op2 & 0x1F;

            loReg(dstReg) = res = (op1 >> shift) | (op1 << (32 - shift));

            if(op2)
                carry = res & (1 << 31) ? Flag_C : 0;

            if(!inIT())
                cpsr = (cpsr & ~(Flag_C | Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry;
            return pcSCycles + 1;
        }
        case 0x8: // TST
            res = op1 & op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
        case 0x9: // NEG
        {
            loReg(dstReg) = res = 0 - op2;
            carry = !(op2 > 0) ? Flag_C : 0; //?
            overflow = (op2 & signBit) & (res & signBit);
            if(!inIT())
                cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        }
        case 0xA: // CMP
            res = op1 - op2;
            carry = !(op2 > op1) ? Flag_C : 0;
            overflow = ((op1 ^ op2) & signBit) & ((op1 ^ res) & signBit); // different signs and sign changed
            cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        case 0xB: // CMN
            res = op1 + op2;
            carry = res < op1 ? Flag_C : 0;
            overflow = ~((op1 ^ op2) & signBit) & ((op1 ^ res) & signBit); // same signs and sign changed
            cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        case 0xC: // ORR
            loReg(dstReg) = res = op1 | op2;
            if(!inIT())
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
                break;
        case 0xD: // MUL
        {
            // carry is meaningless, v is unaffected
            loReg(dstReg) = res = op1 * op2;
            if(!inIT())
                cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);

            break; // single-cycle multiply
        }
        case 0xE: // BIC
            loReg(dstReg) = res = op1 & ~op2;
            if(!inIT())
                cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
        case 0xF: // MVN
            loReg(dstReg) = res = ~op2;
            if(!inIT())
                cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
    }

    return pcSCycles;
}

int ARMv6MCore::doTHUMB05HiReg(uint16_t opcode, uint32_t pc)
{
    auto op = (opcode >> 8) & 3;
    bool h1 = opcode & (1 << 7);
    bool h2 = opcode & (1 << 6);

    auto srcReg = static_cast<Reg>(((opcode >> 3) & 7) + (h2 ? 8 : 0));
    auto dstReg = static_cast<Reg>((opcode & 7) + (h1 ? 8 : 0));

    auto src = reg(srcReg);

    switch(op)
    {
        case 0: // ADD
            if(dstReg == Reg::PC)
            {
                updateTHUMBPC((loReg(Reg::PC) + src) & ~1);
                return pcSCycles * 2 + pcNCycles;
            }
            else
                reg(dstReg) += src;

            break;
        case 1: // CMP
        {
            auto dst = reg(dstReg);

            auto res = dst - src;
            bool carry = !(src > dst);

            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C | Flag_V))
                    | ((res & signBit) ? Flag_N : 0)
                    | (res == 0 ? Flag_Z : 0)
                    | (carry ? Flag_C : 0)
                    | (((dst ^ src) & signBit) && ((dst ^ res) & signBit) ? Flag_V : 0);
            break;
        }
        case 2: // MOV
        {
            if(dstReg == Reg::PC)
            {
                updateTHUMBPC(src & ~1);
                return pcSCycles * 2 + pcNCycles;
            }
            else
                reg(dstReg) = src;

            break;
        }
        case 3: // BX/BLX
        {
            if(h1) // BLX
                loReg(Reg::LR) = (pc - 2) | 1; 

            assert(src & 1);
            int cycles = pcSCycles * 2 + pcNCycles;

            if(src >> 28 == 0xF)
                cycles += handleExceptionReturn(src);
            else
                updateTHUMBPC(src & ~1);

            return cycles;
        }

        default:
            assert(!"Invalid format 5 op!");
    }

    return pcSCycles;
}

int ARMv6MCore::doTHUMB06PCRelLoad(uint16_t opcode, uint32_t pc)
{
    auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
    uint8_t word = opcode & 0xFF;

    // pc + 4, bit 1 forced to 0
    int cycles = 0;
    loReg(dstReg) = readMem32((pc & ~2) + (word << 2), cycles);

    return cycles + pcSCycles;
}

int ARMv6MCore::doTHUMB0708(uint16_t opcode, uint32_t pc)
{
    auto offReg = static_cast<Reg>((opcode >> 6) & 7);
    auto baseReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto addr = loReg(baseReg) + loReg(offReg);

    if(opcode & (1 << 9)) // format 8, load/store sign-extended byte/halfword
    {
        bool hFlag = opcode & (1 << 11);
        bool signEx = opcode & (1 << 10);

        if(signEx)
        {
            if(hFlag && !(addr & 1)) // LDRSH, (misaligned gets treated as a byte!)
            {
                int cycles = 0;
                auto val = readMem16(addr, cycles);
                if(val & 0x8000)
                    loReg(dstReg) = val | 0xFFFF0000;
                else
                    loReg(dstReg) = val;

                return cycles + pcSCycles;
            }
            else // LDRSB
            {
                int cycles = 0;
                auto val = readMem8(addr, cycles);
                if(val & 0x80)
                    loReg(dstReg) = val | 0xFFFFFF00;
                else
                    loReg(dstReg) = val;

                return cycles + pcSCycles;
            }
        }
        else
        {
            if(hFlag) // LDRH
            {
                int cycles = 0;
                loReg(dstReg) = readMem16(addr, cycles);
                return cycles + pcSCycles;
            }
            else // STRH
            {
                int cycles = 0;
                writeMem16(addr, loReg(dstReg), cycles);
                return cycles + pcNCycles;
            }
        }
    }
    else // format 7, load/store with reg offset
    {
        bool isLoad = opcode & (1 << 11);
        bool isByte = opcode & (1 << 10);

        if(isLoad)
        {
            int cycles = 0;
            if(isByte) // LDRB
                loReg(dstReg) = readMem8(addr, cycles);
            else // LDR
                loReg(dstReg) = readMem32(addr, cycles);

            return cycles + pcSCycles;
        }
        else
        {
            int cycles = 0;
            if(isByte) // STRB
                writeMem8(addr, loReg(dstReg), cycles);
            else // STR
                writeMem32(addr, loReg(dstReg), cycles);

            return cycles + pcNCycles;
        }
    }
}

int ARMv6MCore::doTHUMB09LoadStoreWord(uint16_t opcode, uint32_t pc)
{
    bool isLoad = opcode & (1 << 11);
    auto offset = ((opcode >> 6) & 0x1F);
    auto baseReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto addr = loReg(baseReg) + (offset << 2);
    if(isLoad) // LDR
    {
        int cycles = 0;
        loReg(dstReg) = readMem32(addr, cycles);
        return cycles + pcSCycles;
    }
    else // STR
    {
        int cycles = 0;
        writeMem32(addr, loReg(dstReg), cycles);
        return cycles + pcNCycles;
    }
}

int ARMv6MCore::doTHUMB09LoadStoreByte(uint16_t opcode, uint32_t pc)
{
    bool isLoad = opcode & (1 << 11);
    auto offset = ((opcode >> 6) & 0x1F);
    auto baseReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto addr = loReg(baseReg) + offset;
    if(isLoad) // LDRB
    {
        int cycles = 0;
        loReg(dstReg) = readMem8(addr, cycles);
        return cycles + pcSCycles;
    }
    else // STRB
    {
        int cycles = 0;
        writeMem8(addr, loReg(dstReg), cycles);
        return cycles + pcNCycles;
    }
}

int ARMv6MCore::doTHUMB10LoadStoreHalf(uint16_t opcode, uint32_t pc)
{
    bool isLoad = opcode & (1 << 11);
    auto offset = ((opcode >> 6) & 0x1F) << 1;
    auto baseReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto addr = loReg(baseReg) + offset;
    if(isLoad) // LDRH
    {
        int cycles = 0;
        loReg(dstReg) = readMem16(addr, cycles);
        return cycles + pcSCycles;
    }
    else // STRH
    {
        int cycles = 0;
        writeMem16(addr, loReg(dstReg), cycles);
        return cycles + pcNCycles;
    }
}

int ARMv6MCore::doTHUMB11SPRelLoadStore(uint16_t opcode, uint32_t pc)
{
    bool isLoad = opcode & (1 << 11);
    auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
    auto word = (opcode & 0xFF) << 2;

    auto addr = loReg(curSP) + word;

    if(isLoad)
    {
        int cycles = 0;
        loReg(dstReg) = readMem32(addr, cycles);
        return cycles + pcSCycles;
    }
    else
    {
        int cycles = 0;
        writeMem32(addr, loReg(dstReg), cycles);
        return cycles + pcNCycles;
    }
}

int ARMv6MCore::doTHUMB12LoadAddr(uint16_t opcode, uint32_t pc)
{
    bool isSP = opcode & (1 << 11);
    auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
    auto word = (opcode & 0xFF) << 2;

    if(isSP)
        loReg(dstReg) = loReg(curSP) + word;
    else
        loReg(dstReg) = (pc & ~2) + word; // + 4, bit 1 forced to 0

    return pcSCycles;
}

int ARMv6MCore::doTHUMBMisc(uint16_t opcode, uint32_t pc)
{
    switch((opcode >> 8) & 0xF)
    {
        case 0x0: // add/sub imm to SP
            return doTHUMB13SPOffset(opcode, pc);

        case 0x1: // CBZ
        case 0x3:
        case 0x9: // CBNZ
        case 0xB:
        {
            bool nz = opcode & (1 << 11);
            int offset = (opcode & 0xF8) >> 2 | (opcode & (1 << 9)) >> 3;
            auto src = loReg(static_cast<Reg>(opcode & 7));

            if(!!src == nz)
            {
                updateTHUMBPC(pc + offset);
                return pcSCycles * 2 + pcNCycles;
            }

            return pcSCycles;
        }

        case 0x2:
        {
            auto src = loReg(static_cast<Reg>((opcode >> 3) & 7));
            auto dstReg = static_cast<Reg>(opcode & 7);

            switch((opcode >> 6) & 3)
            {
                case 0x0: // SXTH
                    loReg(dstReg) = (src & 0x8000) ? src | 0xFFFF0000 : src & 0xFFFF;
                    break;
                case 0x1: // SXTB
                    loReg(dstReg) = (src & 0x80) ? src | 0xFFFFFF00 : src & 0xFF;
                    break;

                case 0x2: // UXTH
                    loReg(dstReg) = src & 0xFFFF;
                    break;
                case 0x3: // UXTB
                    loReg(dstReg) = src & 0xFF;
                    break;
            }

            return pcSCycles;
        }

        case 0x4: // PUSH
        case 0x5:
            return doTHUMB14PushPop(opcode, pc);

        case 0x6: // CPS
        {
            assert((opcode & 0xFFEF) == 0xB662);
            bool isPrivileged = (cpsr & 0x3F) != 0 || !(control & (1 << 0));

            if(isPrivileged)
                primask = (opcode & (1 << 4)) ? 1 : 0;

            return pcSCycles;
        }

        case 0xA:
        {
            auto src = loReg(static_cast<Reg>((opcode >> 3) & 7));
            auto dstReg = static_cast<Reg>(opcode & 7);

            switch((opcode >> 6) & 3)
            {
                case 0x0: // REV
                    loReg(dstReg) = src >> 24 | src << 24 | ((src << 8) & 0xFF0000) | ((src >> 8) & 0xFF00);
                    break;
                case 0x1: // REV16
                    loReg(dstReg) = ((src >> 8) & 0x00FF00FF) | ((src << 8) & 0xFF00FF00);
                    break;

                case 0x2: 
                    printf("Invalid opcode %04X @%08X\n", opcode, pc - 4);
                    exit(1);
                    break;

                case 0x3: // REVSH
                    loReg(dstReg) = ((src >> 8) & 0x00FF) | ((src << 8) & 0xFF00);
                    if(src & 0x80)
                        loReg(dstReg) |= 0xFFFF0000; // sign extend

                    break;
            }
            return pcSCycles;
        }

        case 0xC: // POP
        case 0xD:
            return doTHUMB14PushPop(opcode, pc);

        case 0xE: // BKPT
            printf("BKPT @%08X\n", pc - 4);
            exit(1);
            return pcSCycles;

        case 0xF:
        {
            auto opA = (opcode >> 4) & 0xF;
            auto opB = opcode & 0xF;

            if(opB != 0) // IT
            {
                itState = opcode & 0xFF;
                itStart = true;
                return pcSCycles;
            }
            else // hints
            {
                switch(opA)
                {
                    case 0: // NOP
                        return pcSCycles;

                    case 1: // YIELD
                        return pcSCycles;

                    case 2: // WFE
                        if(eventFlag)
                            eventFlag = false;
                        else
                            sleeping = true;

                        return pcSCycles * 2;
                    
                    case 3: // WFI
                        // TODO: a bit different
                        return pcSCycles * 2;
                }
            }
        }
    }

    printf("Unhandled opcode %04X @%08X\n", opcode, pc - 4);
    exit(1);
}

int ARMv6MCore::doTHUMB13SPOffset(uint16_t opcode, uint32_t pc)
{
    bool isNeg = opcode & (1 << 7);
    int off = (opcode & 0x7F) << 2;

    if(isNeg)
        loReg(curSP) -= off;
    else
        loReg(curSP) += off;

    return pcSCycles;
}

int ARMv6MCore::doTHUMB14PushPop(uint16_t opcode, uint32_t pc)
{
    // timings here are probably off

    bool isLoad = opcode & (1 << 11);
    bool pclr = opcode & (1 << 8); // store LR/load PC
    uint8_t regList = opcode & 0xFF;

    int numRegs = pclr ? 1 : 0;
    for(uint8_t t = regList; t; t >>= 1)
    {
        if(t & 1)
            numRegs++;
    }

    int cycles = 0;

    if(isLoad) // POP
    {
        auto addr = loReg(curSP);
        auto ptr = reinterpret_cast<uint32_t *>(mem.mapAddress(addr & ~3));
        auto loadCycles = 1;

        loReg(curSP) = addr + numRegs * 4;

        int i = 0;
        for(; regList; regList >>= 1, i++)
        {
            if(regList & 1)
            {
                regs[i] = *ptr++;
                cycles += loadCycles;
            }
        }

        if(pclr)
        {
            auto newPC = *ptr++;
            if(newPC >> 28 == 0xF)
                cycles += handleExceptionReturn(newPC);
            else
                updateTHUMBPC(newPC & ~1); /*ignore thumb bit*/

            cycles += loadCycles; // TODO
        }

        return cycles + pcSCycles;
    }
    else // PUSH
    {
        auto addr = loReg(curSP) - numRegs * 4;
        loReg(curSP) = addr;

        auto ptr = reinterpret_cast<uint32_t *>(mem.mapAddress(addr & ~3));
        auto storeCycles = 1;

        int i = 0;
        for(; regList; regList >>= 1, i++)
        {
            if(regList & 1)
            {
                *ptr++ = regs[i];
                cycles += storeCycles;
            }
        }

        if(pclr)
        {
            *ptr++ = loReg(Reg::LR);
            cycles += storeCycles;
        }

        return cycles +  pcNCycles;
    }
}

int ARMv6MCore::doTHUMB15MultiLoadStore(uint16_t opcode, uint32_t pc)
{
    bool isLoad = opcode & (1 << 11);
    auto baseReg = static_cast<Reg>((opcode >> 8) & 7);
    uint8_t regList = opcode & 0xFF;

    auto addr = loReg(baseReg);

    int cycles = 0;

    if(!regList)
    {
        // empty list loads/stores PC... even though it isn't usually possible here
        if(isLoad)
            updateTHUMBPC(readMem32(addr & ~3, cycles));
        else
            writeMem32(addr & ~3, pc + 2, cycles);

        reg(baseReg) = addr + 0x40;

        return cycles;
    }

    auto endAddr = addr;
    for(uint8_t t = regList; t; t >>=1)
    {
        if(t & 1)
            endAddr += 4;
    }

    // force alingment for everything but SRAM...
    if(addr < 0xE000000)
        addr &= ~3;

    int i = 0;
    bool first = true, seq = false;

    // prevent overriding base for loads
    // "A LDM will always overwrite the updated base if the base is in the list."
    if(isLoad && (regList & (1 << static_cast<int>(baseReg))))
        first = false;

    for(; regList; regList >>= 1, i++)
    {
        if(!(regList & 1))
            continue;

        if(isLoad)
            regs[i] = readMem32(addr, cycles, seq);
        else
            writeMem32(addr, regs[i], cycles, seq);

        // base write-back is on the second cycle of the instruction
        // which is when the first reg is written
        if(first)
            reg(baseReg) = endAddr;

        first = false;
        seq = true;

        addr += 4;
    }

    if(isLoad)
        cycles += pcSCycles;
    else
        cycles += pcNCycles;

    return cycles;
}

int ARMv6MCore::doTHUMB1617(uint16_t opcode, uint32_t pc)
{
    // format 16, conditional branch (+ SWI)
    auto cond = (opcode >> 8) & 0xF;

    int offset = static_cast<int8_t>(opcode & 0xFF);
    bool condVal = false;
    switch(cond)
    {
        case 0x0: // BEQ
            condVal = cpsr & Flag_Z;
            break;
        case 0x1: // BNE
            condVal = !(cpsr & Flag_Z);
            break;
        case 0x2: // BCS
            condVal = cpsr & Flag_C;
            break;
        case 0x3: // BCC
            condVal = !(cpsr & Flag_C);
            break;
        case 0x4: // BMI
            condVal = cpsr & Flag_N;
            break;
        case 0x5: // BPL
            condVal = !(cpsr & Flag_N);
            break;
        case 0x6: // BVS
            condVal = cpsr & Flag_V;
            break;
        case 0x7: // BVC
            condVal = !(cpsr & Flag_V);
            break;
        case 0x8: // BHI
            condVal = (cpsr & Flag_C) && !(cpsr & Flag_Z);
            break;
        case 0x9: // BLS
            condVal = !(cpsr & Flag_C) || (cpsr & Flag_Z);
            break;
        case 0xA: // BGE
            condVal = !!(cpsr & Flag_N) == !!(cpsr & Flag_V);
            break;
        case 0xB: // BLT
            condVal = !!(cpsr & Flag_N) != !!(cpsr & Flag_V);
            break;
        case 0xC: // BGT
            condVal = !(cpsr & Flag_Z) && !!(cpsr & Flag_N) == !!(cpsr & Flag_V);
            break;
        case 0xD: // BLE
            condVal = (cpsr & Flag_Z) || !!(cpsr & Flag_N) != !!(cpsr & Flag_V);
            break;

        // E undefined

        /*case 0xF: // format 17, SWI
        {
            auto ret = (pc - 2) & ~1;
            spsr[1/ *svc* /] = cpsr;

            cpsr = (cpsr & ~(0x1F | Flag_T)) | Flag_I | 0x13; //supervisor mode
            modeChanged();
            loReg(curLR) = ret;
            updateARMPC(8);

            return pcSCycles * 2 + pcNCycles;
        }*/

        default:
            assert(!"Invalid THUMB cond");
    }

    if(!condVal)
        return pcSCycles; // no extra cycles if branch not taken
    
    updateTHUMBPC(pc + offset * 2);

    return pcSCycles * 2 + pcNCycles;
}

int ARMv6MCore::doTHUMB18UncondBranch(uint16_t opcode, uint32_t pc)
{
    if(opcode & (1 << 11))
        return doTHUMB32BitInstruction(opcode, pc);

    uint32_t offset = static_cast<int16_t>(opcode << 5) >> 4; // sign extend and * 2

    updateTHUMBPC(pc + offset);

    return pcSCycles * 2 + pcNCycles; // 2S + 1N
}

uint32_t ARMv6MCore::getShiftedReg(uint32_t opcode, bool &carry)
{
    auto imm = ((opcode >> 10) & 0x1C) | ((opcode >> 6) & 0x3);
    auto type = (opcode >> 4) & 3;

    auto r = static_cast<Reg>(opcode & 0xF);
    auto ret = loReg(r);

    // left shift by immediate 0, do nothing and preserve carry
    if(imm == 0 && type)
    {
        carry = cpsr & Flag_C;
        return ret;
    }

    switch(type)
    {
        case 0: // LSL
            carry = ret & (1 << (32 - imm));
            ret <<= imm;
            break;
        case 1: // LSR
            if(!imm) // shift by 32
            {
                carry = ret & (1 << 31);
                ret = 0;
            }
            else
            {
                carry = ret & (1 << (imm - 1));
                ret >>= imm;
            }
            break;
        case 2: // ASR
        {
            if(!imm) // shift by 32
            {
                auto sign = ret & signBit;
                ret = sign ? 0xFFFFFFFF : 0;
                carry = sign;
            }
            else
            {
                carry = ret & (1 << (imm - 1));
                ret = static_cast<int32_t>(ret) >> imm;
            }
            break;
        }
        case 3:
            if(!imm) // RRX (immediate 0)
            {
                carry = ret & 1; // carry out

                ret >>= 1;

                if(cpsr & Flag_C) // carry in
                    ret |= 0x80000000;
            }
            else // ROR
            {
                ret = (ret >> imm) | (ret << (32 - imm));
                carry = ret & (1 << 31);
            }
            break;

        default:
            assert(!"Invalid shift type!");
    }
    
    return ret;
}

int ARMv6MCore::doDataProcessing(int op, Reg nReg, uint32_t op2, Reg dReg, bool carry, bool setFlags)
{
    switch(op)
    {
        case 0x0: // AND/TST
        {
            auto res = loReg(nReg) & op2;

            if(dReg != Reg::PC) // AND
                loReg(dReg) = res;
            else  // TST
                assert(setFlags);

            if(setFlags)
            {
                cpsr = (cpsr & 0x1FFFFFFF)
                    | (res & Flag_N)
                    | (res == 0 ? Flag_Z : 0)
                    | (carry ? Flag_C : 0);
            }
            return pcSCycles * 2;
        }

        case 0x1: // BIC
        {
            auto res = loReg(nReg) & ~op2;
            loReg(dReg) = res;

            if(setFlags)
            {
                cpsr = (cpsr & 0x1FFFFFFF)
                     | (res & Flag_N)
                     | (res == 0 ? Flag_Z : 0)
                     | (carry ? Flag_C : 0);
            }
            return pcSCycles * 2;
        }

        case 0x2:
        {
            if(nReg == Reg::PC) // MOV / shift
            {
                loReg(dReg) = op2;

                if(setFlags)
                {
                    cpsr = (cpsr & 0x1FFFFFFF)
                         | (op2 & Flag_N)
                         | (op2 == 0 ? Flag_Z : 0)
                         | (carry ? Flag_C : 0);
                }
            }
            else // ORR
            {
                auto res = loReg(nReg) | op2;
                loReg(dReg) = res;

                if(setFlags)
                {
                    cpsr = (cpsr & 0x1FFFFFFF)
                        | (res & Flag_N)
                        | (res == 0 ? Flag_Z : 0)
                        | (carry ? Flag_C : 0);
                }
            }

            return pcSCycles * 2;
        }

        case 0x3:
        {
            uint32_t res;
            if(nReg == Reg::PC) // MVN
                res = ~op2;
            else // ORN
                res = loReg(nReg) | ~op2;

            loReg(dReg) = res;

            if(setFlags)
            {
                cpsr = (cpsr & 0x1FFFFFFF)
                    | (res & Flag_N)
                    | (res == 0 ? Flag_Z : 0)
                    | (carry ? Flag_C : 0);
            }
            return pcSCycles * 2;
        }

        case 0x4:
        {
            auto res = loReg(nReg) ^ op2;

            if(dReg != Reg::PC) // EOR
                loReg(dReg) = res;
            else  // TEQ
                assert(setFlags);

            if(setFlags)
            {
                cpsr = (cpsr & 0x1FFFFFFF)
                    | (res & Flag_N)
                    | (res == 0 ? Flag_Z : 0)
                    | (carry ? Flag_C : 0);
            }
            return pcSCycles * 2;
        }

        // 6: PKH* (reg)

        case 0x8: // ADD
        {
            auto op1 = loReg(nReg);

            auto res = op1 + op2;

            auto carry = res < op1 ? Flag_C : 0;
            auto overflow = ~((op1 ^ op2) & signBit) & ((op1 ^ res) & signBit);

            if(setFlags)
                cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | (carry ? Flag_C : 0) | (overflow >> 3);

            if(dReg != Reg::PC) // ADD
                loReg(dReg) = res;
            else // CMN
                assert(setFlags);

            return pcSCycles * 2;
        }

        case 0xA: // ADC
        {
            auto op1 = loReg(nReg);

            int c = (cpsr & Flag_C) ? 1 : 0;
            auto res = loReg(dReg) = op1 + op2 + c;

            auto carry = res < op1 || (res == op1 && c) ? Flag_C : 0;
            auto overflow = ~((op1 ^ op2) & signBit) & ((op1 ^ res) & signBit);

            if(setFlags)
                cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | (carry ? Flag_C : 0) | (overflow >> 3);
            
            return pcSCycles * 2;
        }

        case 0xB: // SBC
        {
            auto op1 = loReg(nReg);

            int c = (cpsr & Flag_C) ? 1 : 0;
            auto res = loReg(dReg) = op1 - op2 + c - 1;

            auto carry = !(op2 > op1 || (op2 == op1 && !c)) ? Flag_C : 0;
            auto overflow = ((op1 ^ op2) & signBit) & ((op1 ^ res) & signBit);
            if(setFlags)
                cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            
            return pcSCycles * 2;
        }

        case 0xD: // SUB/CMP
        {
            auto op1 = loReg(nReg);
            auto res = op1 - op2;

            if(setFlags)
            {
                carry = !(op2 > op1);
                auto overflow = ((op1 ^ op2) & signBit) & ((op1 ^ res) & signBit);
                cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C | Flag_V)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | (carry ? Flag_C : 0) | (overflow >> 3);
            }

            if(dReg != Reg::PC) // SUB
                loReg(dReg) = res;
            else // CMP
                assert(setFlags);
    
            return pcSCycles * 2;
        }

        case 0xE: // RSB
        {
            auto op1 = loReg(nReg);
            auto res = loReg(dReg) = op2 - op1;

            if(setFlags)
            {
                carry = !(op1 > op2);
                auto overflow = ((op1 ^ op2) & signBit) & ((op2 ^ res) & signBit);
                cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C | Flag_V)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | (carry ? Flag_C : 0) | (overflow >> 3);
            }

            return pcSCycles * 2;
        }
    }

    printf("Unhandled dp op %X @%08X\n", op, loReg(Reg::PC) - 6);
    exit(1);
}

int ARMv6MCore::doTHUMB32BitInstruction(uint16_t opcode, uint32_t pc)
{
    // fetch second half
    uint32_t opcode32 = opcode << 16 | decodeOp;

    decodeOp = fetchOp;

    pc += 2;
    if(pcPtr)
    {
        auto thumbPCPtr = reinterpret_cast<const uint16_t *>(pcPtr + pc);
        assert(mem.verifyPointer(thumbPCPtr, pc));
        fetchOp = *thumbPCPtr;
    }
    else
    {
        int tmp;
        fetchOp = mem.read<uint16_t>(pc, tmp, true);
    }

    loReg(Reg::PC) = pc;

    // decode
    auto op1 = (opcode32 >> 27) & 3;
    auto op2 = (opcode32 >> 20) & 0x7F;

    assert(op1); // 0 should be a 16-bit instruction

    if(op1 == 1)
    {
        if(op2 & 0x40) // coprocessor
            return doTHUMB32BitCoprocessor(opcode32, pc);
        else if(op2 & 0x20) // data processing (shifted register)
            return doTHUMB32BitDataProcessingShiftedReg(opcode32, pc);
        else if(op2 & 4) // load/store dual or exclusive
            return doTHUMB32BitLoadStoreDualEx(opcode32, pc);
        else // load/store multiple
            return doTHUMB32BitLoadStoreMultiple(opcode32, pc);
    }
    else if(op1 == 2)
    {
        if(opcode32 & 0x8000)
            return doTHUMB32BitBranchMisc(opcode32, pc);
        else if(op2 & 0x20) // data processing (plain binary immediate)
            return doTHUMB32BitDataProcessingPlainImm(opcode32, pc);
        else // data processing (modified immediate)
            return doTHUMB32BitDataProcessingModifiedImm(opcode32, pc);
    }
    else if(op1 == 3)
    {
        if(op2 & 0x40) // coprocessor
            return doTHUMB32BitCoprocessor(opcode32, pc);
        else if((op2 & 0x78) == 0x38) // long multiply (accumulate), divide
            return doTHUMB32BitLongMultiplyDiv(opcode32, pc);
        else if((op2 & 0x78) == 0x30) // multiply (accumulate), diff
            return doTHUMB32BitMultiplyDiff(opcode32, pc);
        else if(op2 & 0x20) // data processing (register)
            return doTHUMB32BitDataProcessingReg(opcode32, pc);
        else if((op2 & 7) == 5) // load word
            return doTHUMB32BitLoadWord(opcode32, pc);
        else if((op2 & 7) == 3) // load halfword, memory hints
            return doTHUMB32BitLoadHalfHint(opcode32, pc);
        else if((op2 & 7) == 1) // load byte, memory hints
            return doTHUMB32BitLoadByteHint(opcode32, pc);
        else // store single data item
            return doTHUMB32BitStoreSingle(opcode32, pc);
    }

    __builtin_unreachable();
}

int ARMv6MCore::doTHUMB32BitLoadStoreMultiple(uint32_t opcode, uint32_t pc)
{
    auto op = (opcode >> 23) & 3;
    bool writeback = opcode & (1 << 21);   
    bool isLoad = opcode & (1 << 20);
    auto baseReg = static_cast<Reg>((opcode >> 16) & 0xF);
    uint16_t regList = opcode & 0xFFFF;

    auto addr = loReg(baseReg);

    bool seq = true;
    int cycles = pcSCycles * 2;

    bool baseInList = regList & (1 << static_cast<int>(baseReg));

    if(isLoad) // LDM
    {
        assert(!(regList & (1 << 13)));

        if(op == 1) // IA
        {
            int i = 0;
            for(; regList; regList >>= 1, i++)
            {
                if(!(regList & 1))
                    continue;

                if(i == 15)
                    updateTHUMBPC(readMem32(addr, cycles, seq) & ~1);
                else
                    regs[i] = readMem32(addr, cycles, seq);

                seq = false;
                addr += 4;
            }

            if(writeback && !baseInList)
                loReg(baseReg) = addr;

            return cycles;
        }
        else if(op == 2) // DB
        {
            for(uint16_t t = regList; t; t >>= 1)
            {
                if(t & 1)
                    addr -= 4;
            }

            auto endAddr = addr;

            int i = 0;
            for(; regList; regList >>= 1, i++)
            {
                if(!(regList & 1))
                    continue;

                if(i == 15)
                    updateTHUMBPC(readMem32(addr, cycles, seq) & ~1);
                else
                    regs[i] = readMem32(addr, cycles, seq);

                seq = false;
                addr += 4;
            }

            if(writeback && !baseInList)
                loReg(baseReg) = endAddr;

            return cycles;
        }
    }
    else // STM
    {
        assert(!(regList & (1 << 13)));
        assert(!(regList & (1 << 15)));

        if(op == 1) // IA
        {
            int i = 0;
            for(; regList; regList >>= 1, i++)
            {
                if(!(regList & 1))
                    continue;

                writeMem32(addr, regs[i], cycles, seq);
                seq = false;
                addr += 4;
            }

            if(writeback)
                loReg(baseReg) = addr;

            return cycles;
        }
        else if(op == 2) // DB
        {
            for(uint16_t t = regList; t; t >>= 1)
            {
                if(t & 1)
                    addr -= 4;
            }

            auto endAddr = addr;

            int i = 0;
            for(; regList; regList >>= 1, i++)
            {
                if(!(regList & 1))
                    continue;

                writeMem32(addr, regs[i], cycles, seq);
                seq = false;
                addr += 4;
            }

            if(writeback)
                loReg(baseReg) = endAddr;

            return cycles;
        }
    }

    printf("Unhandled %s multiple opcode %08X (%X) @%08X\n", isLoad ? "load" : "store", opcode, op, pc - 6);
    exit(1);
}

int ARMv6MCore::doTHUMB32BitLoadStoreDualEx(uint32_t opcode, uint32_t pc)
{
    auto op1 = (opcode >> 23) & 3;
    auto op2 = (opcode >> 20) & 3;
    auto op3 = (opcode >> 4) & 0xF;

    auto baseReg = static_cast<Reg>((opcode >> 16) & 0xF);


    if(op1 == 1 && op2 == 1)
    {
        if(!(op3 & 0b1110)) // TBB/TBH
        {
            assert((opcode & 0xFF00) == 0xF000);

            auto indexReg = static_cast<Reg>(opcode & 0xF);

            auto addr = loReg(baseReg);

            if(baseReg == Reg::PC)
                addr -= 2;

            int cycles = pcSCycles * 3 + pcNCycles;
            int offset;
            if(op3 & 1) // TBH
                offset = readMem16(addr + loReg(indexReg) * 2, cycles);
            else
                offset = readMem8(addr + loReg(indexReg), cycles);

            updateTHUMBPC((pc - 2) + offset * 2);

            return cycles;
        }

    }
    else if(((op1 & 2) || (op2 & 2)) && (op2 & 1) == 0) // STRD (immediate)
    {
        auto offset = (opcode & 0xFF) << 2;
        auto dstReg = static_cast<Reg>((opcode >> 12) & 0xF);
        auto dstReg2 = static_cast<Reg>((opcode >> 8) & 0xF);

        bool writeback = opcode & (1 << 21);
        bool add = opcode & (1 << 23);
        bool index = opcode & (1 << 24);

        uint32_t offsetAddr = add ? loReg(baseReg) + offset : loReg(baseReg) - offset;
        uint32_t addr = index ? offsetAddr : loReg(baseReg);

        int cycles = pcSCycles * 2;

        writeMem32(addr, loReg(dstReg), cycles);
        writeMem32(addr + 4, loReg(dstReg2), cycles);

        if(writeback)
            loReg(baseReg) = offsetAddr;

        return cycles;
    }
    else if(((op1 & 2) || (op2 & 2)) && (op2 & 1)) // LDRD (immediate)
    {
        auto offset = (opcode & 0xFF) << 2;
        auto dstReg = static_cast<Reg>((opcode >> 12) & 0xF);
        auto dstReg2 = static_cast<Reg>((opcode >> 8) & 0xF);

        bool writeback = opcode & (1 << 21);
        bool add = opcode & (1 << 23);
        bool index = opcode & (1 << 24);

        uint32_t offsetAddr = add ? loReg(baseReg) + offset : loReg(baseReg) - offset;
        uint32_t addr = index ? offsetAddr : loReg(baseReg);

        int cycles = pcSCycles * 2;

        loReg(dstReg) = readMem32(addr, cycles);
        loReg(dstReg2) = readMem32(addr + 4, cycles);

        if(writeback)
            loReg(baseReg) = offsetAddr;

        return cycles;
    }

    printf("Unhandled load/store dual/exclusive opcode %08X (%X %X %X) @%08X\n", opcode, op1, op2, op3, pc - 6);
    exit(1);
}

int ARMv6MCore::doTHUMB32BitDataProcessingShiftedReg(uint32_t opcode, uint32_t pc)
{
    auto op = (opcode >> 21) & 0xF;
    bool setFlags = opcode & (1 << 20);

    auto nReg = static_cast<Reg>((opcode >> 16) & 0xF);
    auto dstReg = static_cast<Reg>((opcode >> 8) & 0xF);

    bool carry;
    auto val = getShiftedReg(opcode, carry);

    assert(op != 5 && op != 7 && op != 9 && op != 12 && op != 15); // undefined

    return doDataProcessing(op, nReg, val, dstReg, carry, setFlags);
}

int ARMv6MCore::doTHUMB32BitCoprocessor(uint32_t opcode, uint32_t pc)
{
    bool op = opcode & (1 << 4);
    auto coproc = (opcode >> 8) & 0xF;
    auto op1 = (opcode >> 20) & 0x3F;

    int cycles = pcSCycles * 2;

    if(coproc != 0xA && coproc != 0xB) // VFP
    {
        printf("Unhandled coprocessor %X (opcode %08X) @%08X\n", coproc, opcode, pc - 6);
        exit(1);
    }

    bool dWidth = coproc & 1;

    auto expandImm = [](uint8_t imm8, int width)
    {
        int e = width == 32 ? 8 : 11;
        int f = width - e - 1;

        auto eMask = ((1 << (e - 1)) - 1) & ~ 3;

        bool sign = imm8 & 0x80;
        auto exp = ((~imm8 << 1) & 0x80) | ((imm8 & 0x40) ? eMask  : 0)| ((imm8 >> 4) & 3);
        auto frac = static_cast<uint64_t>(imm8 & 0xF) << (f - 4);

        return (sign ? 1ULL : 0ULL) << (e + f) | static_cast<uint64_t>(exp) << f | frac;
    };

    auto getVReg = [opcode](int pos, int hiLoPos, bool dWidth)
    {
        if(dWidth) // n nnnn
            return ((opcode >> pos) & 0xF) | ((opcode >> (hiLoPos - 4)) & 0x10);
        
        // nnnn n
        if(pos == 0)
            return ((opcode << 1) & 0x1E) | ((opcode >> hiLoPos) & 1);
        
        return ((opcode >> (pos - 1)) & 0x1E) | ((opcode >> hiLoPos) & 1);
    };

    if(op)
    {
        auto a = (op1 >> 1) & 7;
        bool c = dWidth;
        auto b = (opcode >> 5) & 3;

        if((op1 & 0x31) == 0x20) // move to coprocessor
        {
            if(c & !b) // VMOV to scalar
            {}
            else if(a == 7) // VMSR
            {}
            else if(a == 0) // VMOV
            {
                assert((opcode & 0x6F) == 0);

                auto tReg = static_cast<Reg>((opcode >> 12) & 0xF);
                auto n = getVReg(16, 7, false);

                fpRegs[n] = loReg(tReg);
                return cycles;
            }
        }
        else if((op1 & 0x31) == 0x21) // move from coprocessor
        {
            if(c && !b) // VMOV from scalar
            {}
            else if(a == 7) // VMRS
            {
                assert((opcode & 0xF00FF) == 0x10010);
                auto tReg = static_cast<Reg>((opcode >> 12) & 0xF);

                // transfer flags
                if(tReg == Reg::PC)
                    cpsr = (cpsr & 0x0FFFFFFF) | (fpscr & 0xF0000000);
                else
                    loReg(tReg) = fpscr;

                return cycles;
            }
            else if(a == 0) // VMOV
            {
                assert((opcode & 0x6F) == 0);

                auto tReg = static_cast<Reg>((opcode >> 12) & 0xF);
                auto n = getVReg(16, 7, false);

                loReg(tReg) = fpRegs[n];
                return cycles;
            }
        }
        else if((op1 & 0x3E) == 4)
        {
            assert(((opcode >> 4) & 0b1101) == 1);

            bool toArm = op1 & 1;

            auto tReg = static_cast<Reg>((opcode >> 12) & 0xF);
            auto t2Reg = static_cast<Reg>((opcode >> 16) & 0xF);

            auto m = getVReg(0, 5, dWidth);

            if(dWidth) // two regs <-> d reg
            {
                if(toArm)
                {
                    loReg(tReg) = fpRegs[m * 2];
                    loReg(t2Reg) = fpRegs[m * 2 + 1];
                }
                else
                {
                    fpRegs[m * 2] = loReg(tReg);
                    fpRegs[m * 2 + 1] = loReg(t2Reg);
                }

                return cycles;
            }
            else // two regs <-> two s regs
            {}
        }
    }
    else if(!op)
    {
        if((op1 & 0x30) == 0x20) // coprocessor data ops
        {
            bool t = opcode & (1 << 28);
            auto opc1 = op1 & 0xF;
            auto opc2 = (opcode >> 16) & 0xF;
            auto opc3 = (opcode >> 6) & 3;
            auto opc4 = opcode & 0xF;

            if(t)
            {
                if((opc1 & 0b1000) == 0) // VSEL
                {
                    auto n = getVReg(16, 7, dWidth);
                    auto d = getVReg(12, 22, dWidth);
                    auto m = getVReg(0, 5, dWidth);

                    int cc = (opcode >> 20) & 3;
                    int cond = cc << 2 | ((cc ^ (cc << 1)) & 2);

                    // only four valid conditions
                    bool condVal = false;
                    switch(cond)
                    {
                        case 0x0: // EQ
                            condVal = cpsr & Flag_Z;
                            break;
                        case 0x6: // VS
                            condVal = cpsr & Flag_V;
                            break;
                        case 0xA: // GE
                            condVal = !!(cpsr & Flag_N) == !!(cpsr & Flag_V);
                            break;
                        case 0xC: // GT
                            condVal = !(cpsr & Flag_Z) && !!(cpsr & Flag_N) == !!(cpsr & Flag_V);
                            break;
                    }

                    if(dWidth)
                        dReg(d) = dReg(condVal ? n : m);
                    else
                        sReg(d) = sReg(condVal ? n : m);
                    
                    return cycles;
                }
                else if((opc1 & 0b1011) == 0b1011)
                {
                    if((opc2 & 0b1100) == 0b1000) // VRINT[ANPM]
                    {
                        assert(opc3  == 1);

                        auto rm = (opcode >> 16) & 3; // away, even, +inf, -inf

                        auto d = getVReg(12, 22, dWidth);
                        auto m = getVReg(0, 5, dWidth);

                        if(dWidth)
                        {
                            if(rm == 0) // away from 0
                                dReg(d) = round(dReg(m));
                            else if(rm == 1) // to even
                                dReg(d) = dReg(m) - remainder(dReg(m), 1.0);
                            else if(rm == 2) // to +infinity
                                dReg(d) = floor(dReg(m) + 0.5);
                            else if(rm == 3) // to -infinity
                                dReg(d) = ceil(dReg(m) - 0.5);
                        }
                        else
                        {
                            if(rm == 0) // away from 0
                                sReg(d) = roundf(sReg(m));
                            else if(rm == 1) // to even
                                sReg(d) = sReg(m) - remainderf(sReg(m), 1.0f);
                            else if(rm == 2) // to +infinity
                                sReg(d) = floorf(sReg(m) + 0.5f);
                            else if(rm == 3) // to -infinity
                                sReg(d) = ceilf(sReg(m) - 0.5f);
                        }

                        return cycles;
                    }
                }
            }
            else
            {
                switch(opc1 & 0b1011)
                {
                    case 0b0010:
                    {
                        auto n = getVReg(16, 7, dWidth);
                        auto d = getVReg(12, 22, dWidth);
                        auto m = getVReg(0, 5, dWidth);

                        if(opc3 & 1) // VNMUL
                        {
                            if(dWidth)
                                dReg(d) = -(dReg(n) * dReg(m));
                            else
                                sReg(d) = -(sReg(n) * sReg(m));
                        }
                        else // VMUL
                        {
                            if(dWidth)
                                dReg(d) = dReg(n) * dReg(m);
                            else
                                sReg(d) = sReg(n) * sReg(m);
                        }

                        return cycles;
                    }
                    case 0b0011:
                    {
                        auto n = getVReg(16, 7, dWidth);
                        auto d = getVReg(12, 22, dWidth);
                        auto m = getVReg(0, 5, dWidth);

                        if(opc3 & 1) // VSUB
                        {
                            if(dWidth)
                                dReg(d) = dReg(n) - dReg(m);
                            else
                                sReg(d) = sReg(n) - sReg(m);
                        }
                        else // VADD
                        {
                            if(dWidth)
                                dReg(d) = dReg(n) + dReg(m);
                            else
                                sReg(d) = sReg(n) + sReg(m);
                        }

                        return cycles;
                    }

                    case 0b1000: // VDIV
                    {
                        assert(!(opc3 & 1));
                        auto n = getVReg(16, 7, dWidth);
                        auto d = getVReg(12, 22, dWidth);
                        auto m = getVReg(0, 5, dWidth);

                        if(dWidth)
                            dReg(d) = dReg(n) / dReg(m);
                        else
                            sReg(d) = sReg(n) / sReg(m);

                        return cycles;
                    }

                    case 0b1001: // VFNMA/VFNMS
                    {
                        bool isSub = opcode & (1 << 6);

                        auto n = getVReg(16, 7, dWidth);
                        auto d = getVReg(12, 22, dWidth);
                        auto m = getVReg(0, 5, dWidth);

                        if(dWidth)
                        {
                            auto op1 = dReg(n);
                            if(isSub)
                                op1 = -op1;

                            dReg(d) = -dReg(d) + op1 * dReg(m);
                        }
                        else
                        {
                            auto op1 = sReg(n);
                            if(isSub)
                                op1 = -op1;

                            sReg(d) = -sReg(d) + op1 * sReg(m);
                        }
                        return cycles;
                    }

                    case 0b1010: // VFMA/VFMS
                    {
                        bool isSub = opcode & (1 << 6);

                        auto n = getVReg(16, 7, dWidth);
                        auto d = getVReg(12, 22, dWidth);
                        auto m = getVReg(0, 5, dWidth);

                        if(dWidth)
                        {
                            auto op1 = dReg(n);
                            if(isSub)
                                op1 = -op1;

                            dReg(d) = dReg(d) + op1 * dReg(m);
                        }
                        else
                        {
                            auto op1 = sReg(n);
                            if(isSub)
                                op1 = -op1;

                            sReg(d) = sReg(d) + op1 * sReg(m);
                        }
                        return cycles;
                    }
                    case 0b1011:
                    {
                        if((opc3 & 1) == 0) // VMOV (immediate)
                        {
                            auto imm = opc4 | opc2 << 4;

                            auto d = getVReg(12, 22, dWidth);

                            if(dWidth)
                            {
                                uint64_t imm64 = expandImm(imm, 64);
                                fpRegs[d * 2] = imm64 & 0xFFFFFFFF;
                                fpRegs[d * 2 + 1] = imm64 >> 32;
                            }
                            else
                            {
                                uint32_t imm32 = expandImm(imm, 32);
                                fpRegs[d] = imm32;
                            }
                            return cycles;
                        }
                        else if(opc2 == 0)
                        {
                            auto d = getVReg(12, 22, dWidth);
                            auto m = getVReg(0, 5, dWidth);

                            if(opc3 == 1) // VMOV (register)
                            {
                                if(dWidth)
                                    dReg(d) = dReg(m);
                                else
                                    fpRegs[d] = fpRegs[m];
                            }
                            else // VABS
                            {
                                if(dWidth)
                                    dReg(d) = abs(dReg(m));
                                else
                                    sReg(d) = fabs(sReg(m));
                            }

                            return cycles;
                        }
                        else if(opc2 == 1)
                        {
                            auto d = getVReg(12, 22, dWidth);
                            auto m = getVReg(0, 5, dWidth);

                            if(opc3 == 1) // VNEG
                            {
                                if(dWidth)
                                    dReg(d) = -dReg(m);
                                else
                                    sReg(d) = -sReg(m);
                            }
                            else // VSQRT
                            {
                                if(dWidth)
                                    dReg(d) = sqrt(dReg(m));
                                else
                                    sReg(d) = sqrtf(sReg(m));
                            }
                            return cycles;
                        }
                        else if(opc2 == 4 || opc2 == 5) // VCMP(E)
                        {
                            //bool e = opcode & (1 << 7); // nans
                            bool withZero = opcode & (1 << 16);

                            auto d = getVReg(12, 22, dWidth);

                            if(dWidth)
                            {
                                double val = 0.0;

                                if(withZero)
                                    assert((opcode & 0x2F) == 0);
                                else
                                {
                                    auto m = getVReg(0, 5, dWidth);
                                    val = dReg(m);
                                }

                                if(dReg(d) == val)
                                    fpscr = (fpscr & 0x0FFFFFFF) | Flag_C | Flag_Z;
                                else if(dReg(d) < val)
                                    fpscr = (fpscr & 0x0FFFFFFF) | Flag_N;
                                else
                                    fpscr = (fpscr & 0x0FFFFFFF) | Flag_C;

                                return cycles;
                            }
                            else
                            {
                                float val = 0.0f;

                                if(withZero)
                                    assert((opcode & 0x2F) == 0);
                                else
                                {
                                    auto m = getVReg(0, 5, dWidth);
                                    val = sReg(m);
                                }

                                if(sReg(d) == val)
                                    fpscr = (fpscr & 0x0FFFFFFF) | Flag_C | Flag_Z;
                                else if(sReg(d) < val)
                                    fpscr = (fpscr & 0x0FFFFFFF) | Flag_N;
                                else
                                    fpscr = (fpscr & 0x0FFFFFFF) | Flag_C;

                                return cycles;
                            }
                        }
                        else if(opc2 == 7 && opc3 == 3) // VCVT (single <-> double)
                        {
                            auto d = getVReg(12, 22, !dWidth);
                            auto m = getVReg(0, 5, dWidth);

                            if(dWidth) // D -> S
                                sReg(d) = dReg(m);
                            else // S -> D
                                dReg(d) = sReg(m);

                            return cycles;
                        }
                        else if(opc2 == 8) // VCVT (int -> fp)
                        {
                            //bool toInt = false; // opc2 & 4;
                            bool isUnsigned = !(opcode & (1 << 7));
                            // TODO: rounding mode

                            auto d = getVReg(12, 22, dWidth);
                            auto m = getVReg(0, 5, false);

                            if(dWidth)
                            {
                                if(isUnsigned)
                                    dReg(d) = fpRegs[m];
                                else
                                    dReg(d) = static_cast<int32_t>(fpRegs[m]);
                            }
                            else
                            {
                                if(isUnsigned)
                                    sReg(d) = fpRegs[m];
                                else
                                    sReg(d) = static_cast<int32_t>(fpRegs[m]);
                            }

                            return cycles;
                        }
                        else if((opc2 & 0xA) == 0xA) // VCVT (fixed <-> float)
                        {
                            auto d = getVReg(12, 22, dWidth);
                            bool toFixed = (opc2 & 4);
                            bool isUnsigned = (opc2 & 1);

                            int size = (opcode & (1 << 7)) ? 32 : 16;
                            int imm = ((opcode << 1) & 0x1E) | ((opcode >> 5) & 1);
                            int fracBits = size - imm;

                            uint32_t mask = (1ull << size) - 1;

                            if(toFixed)
                            {
                                if(dWidth)
                                {
                                    uint32_t fixed = dReg(d) * (1 << fracBits);

                                    if(!isUnsigned && (fixed & (1 << (size - 1))))
                                        fixed |= ~mask;
                                    else
                                        fixed &= mask;

                                    fpRegs[d * 2] = fixed;
                                    fpRegs[d * 2 + 1] = 0; // result is 32-bit
                                }
                                else
                                {
                                    uint32_t fixed = sReg(d) * (1 << fracBits);

                                    if(!isUnsigned && (fixed & (1 << (size - 1))))
                                        fixed |= ~mask;
                                    else
                                        fixed &= mask;

                                    fpRegs[d] = fixed;
                                }

                                return cycles;
                            }
                            else
                            {
                                uint32_t fixed = (dWidth ? fpRegs[d * 2] : fpRegs[d]) & mask;
                                if(!isUnsigned && size == 16 && (fixed & 0x8000))
                                    fixed |= ~mask;

                                if(dWidth)
                                {
                                    double f;
                                    if(isUnsigned)
                                        f = static_cast<double>(fixed) / (1 << fracBits);
                                    else
                                        f = static_cast<double>(static_cast<int32_t>(fixed)) / (1 << fracBits);

                                    dReg(d) = f;
                                }
                                else
                                {
                                    float f;
                                    if(isUnsigned)
                                        f = static_cast<float>(fixed) / (1 << fracBits);
                                    else
                                        f = static_cast<float>(static_cast<int32_t>(fixed)) / (1 << fracBits);

                                    sReg(d) = f;
                                }

                                return cycles;
                            }
                        }
                        else if(opc2 == 0xC || opc2 == 0xD) // VCVT (fp -> int)
                        {
                            bool isUnsigned = !(opc2 & 1);
                            // TODO: rounding mode (1 << 7)

                            auto d = getVReg(12, 22, false);
                            auto m = getVReg(0, 5, dWidth);

                            if(dWidth)
                            {
                                if(isUnsigned)
                                    fpRegs[d] = static_cast<uint32_t>(dReg(m));
                                else
                                    fpRegs[d] = static_cast<int32_t>(dReg(m));
                            }
                            else
                            {
                                if(isUnsigned)
                                    fpRegs[d] = static_cast<uint32_t>(sReg(m));
                                else
                                    fpRegs[d] = static_cast<int32_t>(sReg(m));
                            }

                            return cycles;
                        }
                        break;
                    }
                }
            }

            printf("cdp %x %x %x %x %x\n", t, opc1, opc2, opc3, opc4);
        }
    }

    if(op1 == 4) // move to coprocessor from two arm regs
    {}
    else if(op1 == 5) // move to two arm regs from coprocessor
    {}
    else if((op1 & 0x21) == 1) // load coprocessor
    {
        bool t2 = opcode & (1 << 28);

        bool index = opcode & (1 << 24);
        bool add = opcode & (1 << 23);
        //bool d = opcode & (1 << 22);
        bool writeback = opcode & (1 << 21);

        assert(!t2);

        auto baseReg = static_cast<Reg>((opcode >> 16) & 0xF);
        auto d = getVReg(12, 22, dWidth);

        auto imm = (opcode & 0xFF) << 2;

        int regs = (opcode & 0xFF);

        if(index && !writeback) // VLDR
            regs = 1;
        else // VLDM
        {
            assert(!(index == add && writeback));

            if(dWidth)
            {
                assert(!(regs & 1)); // FLDMX
                regs /= 2;
            }
        }

        uint32_t offsetAddr = loReg(baseReg);

        // align PC for literal load
        if(baseReg == Reg::PC)
            offsetAddr = (offsetAddr - 2) & ~2;

        if(add)
            offsetAddr += imm;
        else
            offsetAddr -= imm;

        uint32_t addr = index ? offsetAddr : loReg(baseReg);

        if(writeback)
            loReg(baseReg) = offsetAddr;
            
        for(int i = 0; i < regs; i++)
        {
            if(dWidth)
            {
                fpRegs[(d + i) * 2] = readMem32(addr, cycles);
                fpRegs[(d + i) * 2 + 1] = readMem32(addr + 4, cycles);
                addr += 8;
            }
            else
            {
                fpRegs[d + i] = readMem32(addr, cycles);
                addr += 4;
            }
        }

        return cycles;
    }
    else if((op1 & 0x21) == 0) // store coprocessor
    {
        bool t2 = opcode & (1 << 28);

        bool index = opcode & (1 << 24);
        bool add = opcode & (1 << 23);
        //bool n = opcode & (1 << 22);
        bool writeback = opcode & (1 << 21);

        assert(!t2);

        auto baseReg = static_cast<Reg>((opcode >> 16) & 0xF);
        auto d = getVReg(12, 22, dWidth);

        auto imm = (opcode & 0xFF) << 2;

        int regs = (opcode & 0xFF);

        if(index && !writeback) // VSTR
            regs = 1;
        else // VSTM
        {
            assert(!(index == add && !writeback));

            if(dWidth)
            {
                assert(!(regs & 1)); // FSTMX
                regs /= 2;
            }
        }

        uint32_t offsetAddr = add ? loReg(baseReg) + imm : loReg(baseReg) - imm;

        uint32_t addr = index ? offsetAddr : loReg(baseReg);

        if(writeback)
            loReg(baseReg) = offsetAddr;
            
        for(int i = 0; i < regs; i++)
        {
            if(dWidth)
            {
                writeMem32(addr, fpRegs[(d + i) * 2], cycles);
                writeMem32(addr + 4, fpRegs[(d + i) * 2 + 1], cycles);
                addr += 8;
            }
            else
            {
                writeMem32(addr, fpRegs[d + i], cycles);
                addr += 4;
            }
        }

        return cycles;
    }

    printf("Unhandled coprocessor opcode %08X (%X %X %X) @%08X\n", opcode, op, coproc, op1, pc - 6);
    exit(1);
}

int ARMv6MCore::doTHUMB32BitDataProcessingModifiedImm(uint32_t opcode, uint32_t pc)
{
    auto op = (opcode >> 21) & 0xF;
    bool setFlags = opcode & (1 << 20);

    auto nReg = static_cast<Reg>((opcode >> 16) & 0xF);
    auto dstReg = static_cast<Reg>((opcode >> 8) & 0xF);

    auto imm = ((opcode >> 12) & 7) | ((opcode >> 23) & 8);
    auto val8 = opcode & 0xFF;

    // get the modified imm
    uint32_t val;
    bool carry = cpsr & Flag_C;
    
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
            carry = val & (1 << 31);
            break;
        }
    }

    assert(op != 5 && op != 6 && op != 7 && op != 9 && op != 12 && op != 15); // undefined

    return doDataProcessing(op, nReg, val, dstReg, carry, setFlags);
}

int ARMv6MCore::doTHUMB32BitDataProcessingPlainImm(uint32_t opcode, uint32_t pc)
{
    auto op = (opcode >> 21) & 0xF;

    assert(!(opcode & (1 << 20))); // S = 0

    auto nReg = static_cast<Reg>((opcode >> 16) & 0xF);
    auto dstReg = static_cast<Reg>((opcode >> 8) & 0xF);

    switch(op)
    {
        case 0x0:
        {
            if(nReg == Reg::PC) // ADR
            {}
            else // ADDW
            {
                auto imm = ((opcode >> 15) & 0x800) | ((opcode >> 4) & 0x700) | (opcode & 0xFF);

                loReg(dstReg) = loReg(nReg) + imm;
                return pcSCycles * 2;
            }
            break;
        }
        case 0x5:
        {
            if(nReg == Reg::PC) // ADR
            {}
            else // SUBW
            {
                auto imm = ((opcode >> 15) & 0x800) | ((opcode >> 4) & 0x700) | (opcode & 0xFF);

                loReg(dstReg) = loReg(nReg) - imm;
                return pcSCycles * 2;
            }
            break;
        }
        case 0x2: // MOVW
        {
            auto imm = ((opcode >> 4) & 0xF000) |((opcode >> 15) & 0x800) | ((opcode >> 4) & 0x700) | (opcode & 0xFF);

            loReg(dstReg) = imm;
            return pcSCycles * 2;
        }
        case 0xB: // BFI/BFC
        {
            int msb = (opcode & 0x1F);
            int lsb = ((opcode >> 10) & 0x1C) | ((opcode >> 6) & 3);

            assert(msb >= lsb);

            auto mask = (1 << (msb - lsb + 1)) - 1;
            if(nReg == Reg::PC) // BFC
                loReg(dstReg) = (loReg(dstReg) & ~(mask << lsb));
            else // BFI
                loReg(dstReg) = (loReg(dstReg) & ~(mask << lsb)) | loReg(nReg) << lsb;

            return pcSCycles * 2;
        }
        case 0xE: // UBFX
        {
            int lsbit = ((opcode >> 10) & 0x1C) | ((opcode >> 6) & 3);
            int width = (opcode & 0x1F) + 1;

            auto mask = (1 << width) - 1;

            loReg(dstReg) = (loReg(nReg) >> lsbit) & mask;
            
            return pcSCycles * 2;
        }
    }

    printf("Unhandled dp plain imm opcode %08X (%X) @%08X\n", opcode, op, pc - 6);
    exit(1);
}

int ARMv6MCore::doTHUMB32BitBranchMisc(uint32_t opcode, uint32_t pc)
{
    // branch and misc control

    auto op1 = (opcode >> 20) & 0x7F;
    auto op2 = (opcode >> 12) & 0x7;

    if((op2 & 1)) // B/BL
    {
        bool link = (op2 & 0b100);

        auto imm11 = opcode & 0x7FF;
        auto imm10 = (opcode >> 16) & 0x3FF;

        auto s = opcode & (1 << 26);
        auto i1 = (opcode >> 13) & 1;
        auto i2 = (opcode >> 11) & 1;

        if(!s)
        {
            i1 ^= 1;
            i2 ^= 1;
        }

        uint32_t offset = imm11 << 1 | imm10 << 12 | i2 << 22 | i1 << 23;

        if(s)
            offset |= 0xFF000000; // sign extend

        if(link)
            loReg(Reg::LR) = (pc - 2) | 1; // magic switch to thumb bit...
        updateTHUMBPC((pc - 2) + offset);

        return pcNCycles + pcSCycles * 3;
    }

    assert((op2 & 0b100) == 0);

    if((op1 & 0b111000) != 0b111000) // B
    {
        auto cond = (op1 >> 2) & 0xF;

        auto imm11 = opcode & 0x7FF;
        auto imm6 = (opcode >> 16) & 0x3F;

        auto s = opcode & (1 << 26);
        auto i1 = (opcode >> 13) & 1;
        auto i2 = (opcode >> 11) & 1;

        uint32_t offset = imm11 << 1 | imm6 << 12 | i2 << 18 | i1 << 19;

        if(s)
            offset |= 0xFFF00000; // sign extend

        bool condVal = false;
        switch(cond)
        {
            case 0x0: // BEQ
                condVal = cpsr & Flag_Z;
                break;
            case 0x1: // BNE
                condVal = !(cpsr & Flag_Z);
                break;
            case 0x2: // BCS
                condVal = cpsr & Flag_C;
                break;
            case 0x3: // BCC
                condVal = !(cpsr & Flag_C);
                break;
            case 0x4: // BMI
                condVal = cpsr & Flag_N;
                break;
            case 0x5: // BPL
                condVal = !(cpsr & Flag_N);
                break;
            case 0x6: // BVS
                condVal = cpsr & Flag_V;
                break;
            case 0x7: // BVC
                condVal = !(cpsr & Flag_V);
                break;
            case 0x8: // BHI
                condVal = (cpsr & Flag_C) && !(cpsr & Flag_Z);
                break;
            case 0x9: // BLS
                condVal = !(cpsr & Flag_C) || (cpsr & Flag_Z);
                break;
            case 0xA: // BGE
                condVal = !!(cpsr & Flag_N) == !!(cpsr & Flag_V);
                break;
            case 0xB: // BLT
                condVal = !!(cpsr & Flag_N) != !!(cpsr & Flag_V);
                break;
            case 0xC: // BGT
                condVal = !(cpsr & Flag_Z) && !!(cpsr & Flag_N) == !!(cpsr & Flag_V);
                break;
            case 0xD: // BLE
                condVal = (cpsr & Flag_Z) || !!(cpsr & Flag_N) != !!(cpsr & Flag_V);
                break;
        }

        if(condVal)
        {
            updateTHUMBPC((pc - 2) + offset);

            return pcNCycles + pcSCycles * 3;
        }

        return pcSCycles * 2;
    }

    switch(op1)
    {
        case 0x38: // MSR
        case 0x39:
        {
            auto srcReg = static_cast<Reg>((opcode >> 16) & 0xF);
            auto sysm = opcode & 0xFF;
            bool isPrivileged = (cpsr & 0x3F) != 0 || !(control & (1 << 0));

            if((sysm >> 3) == 0)
            {
                // APSR
            }
            else if((sysm >> 3) == 1)
            {
                // write MSP/PSP
                if(isPrivileged)
                {
                    if(sysm == 8)
                        loReg(Reg::MSP) = reg(srcReg) & ~3;
                    else if(sysm == 9)
                        loReg(Reg::PSP) = reg(srcReg) & ~3;
                }
                return pcSCycles * 2 + 1;
            }
            else if((sysm >> 3) == 2)
            {
                // PRIMASK/CONTROL
                if(isPrivileged)
                {
                    if(sysm == 0x10)
                        primask = reg(srcReg) & 1;
                    else if(sysm == 0x14 && (cpsr & 0x3F) == 0)
                        control = reg(srcReg) & 3;
                }
                return pcSCycles * 2 + 1;
            }

            break;
        }

        case 0x3B: // misc
        {
            auto op = (opcode >> 4) & 0xF;

            if(op == 0x4 || op == 0x5) // DSB/DMB
            {
                //do something?
                return pcSCycles * 2 + 1;
            }

            break;
        }

        case 0x3E: // MRS
        case 0x3F:
        {
            auto dstReg = static_cast<Reg>((opcode >> 8) & 0xF);
            auto sysm = opcode & 0xFF;

            if((sysm >> 3) == 0)
            {
                // xPSR
                uint32_t mask = 0;
                if(sysm & 1) // IPSR
                    mask |= 0x1FF;

                // if(sysm & 2) // T bit reads as 0 so do nothing

                if(sysm & 4) // APSR
                    mask |= 0xF8000000;

                reg(dstReg) = cpsr & mask;

                return pcSCycles * 2 + 1;
            }
            else if((sysm >> 3) == 1)
            {
                // MSP/PSP
                if(sysm == 8)
                    reg(dstReg) = loReg(Reg::MSP);
                else if(sysm == 9)
                    reg(dstReg) = loReg(Reg::PSP);

                return pcSCycles * 2 + 1;
            }
            else if((sysm >> 3) == 2)
            {
                // PRIMASK/CONTROL
                if(sysm == 0x10)
                    reg(dstReg) = primask & 1;
                else if(sysm == 0x14)
                    reg(dstReg) = control & 3;

                return pcSCycles * 2 + 1;
            }
            break;
        }
    }

    printf("Unhandled branch/misc opcode %08X (%X %X) @%08X\n", opcode, op1, op2, pc - 6);
    exit(1);
}

int ARMv6MCore::doTHUMB32BitStoreSingle(uint32_t opcode, uint32_t pc)
{
    auto op1 = (opcode >> 21) & 7;
    auto op2 = (opcode >> 6) & 0x3F;

    auto baseReg = static_cast<Reg>((opcode >> 16) & 0xF);
    auto dstReg = static_cast<Reg>((opcode >> 12) & 0xF);

    if(op1 & 4) // 12 bit immediate
    {
        auto offset = (opcode & 0xFFF);

        uint32_t addr = loReg(baseReg) + offset;

        int cycles = pcSCycles * 2;

        switch(op1 & 3)
        {
            case 0:
                writeMem8(addr, loReg(dstReg), cycles); break; // STRB
            case 1:
                writeMem16(addr, loReg(dstReg), cycles); break; // STRH
            case 2:
                writeMem32(addr, loReg(dstReg), cycles); break; // STR
        }

        return cycles;
    }
    else if(op2 & 0x20) // 8 bit immediate
    {
        auto offset = (opcode & 0xFF);

        bool writeback = opcode & (1 << 8);
        bool add = opcode & (1 << 9);
        bool index = opcode & (1 << 10);

        uint32_t offsetAddr = add ? loReg(baseReg) + offset : loReg(baseReg) - offset;
        uint32_t addr = index ? offsetAddr : loReg(baseReg);

        int cycles = pcSCycles * 2;

        switch(op1 & 3)
        {
            case 0:
                writeMem8(addr, loReg(dstReg), cycles); break; // STRB
            case 1:
                writeMem16(addr, loReg(dstReg), cycles); break; // STRH
            case 2:
                writeMem32(addr, loReg(dstReg), cycles); break; // STR
        }

        if(writeback)
            loReg(baseReg) = offsetAddr;

        return cycles;
    }
    else // register
    {
        auto mReg = static_cast<Reg>(opcode & 0xF);
        auto shift = (opcode >> 4) & 3;

        uint32_t addr = loReg(baseReg) + (loReg(mReg) << shift);

        int cycles = pcSCycles * 2;

        switch(op1 & 3)
        {
            case 0:
                writeMem8(addr, loReg(dstReg), cycles); break; // STRB
            case 1:
                writeMem16(addr, loReg(dstReg), cycles); break; // STRH
            case 2:
                writeMem32(addr, loReg(dstReg), cycles); break; // STR
        }

        return cycles;
    }
}

int ARMv6MCore::doTHUMB32BitLoadByteHint(uint32_t opcode, uint32_t pc)
{
    auto op1 = (opcode >> 23) & 3;
    auto op2 = (opcode >> 6) & 0x3F;

    auto baseReg = static_cast<Reg>((opcode >> 16) & 0xF);
    auto dstReg = static_cast<Reg>((opcode >> 12) & 0xF);

    int cycles = pcSCycles * 2;

    if(dstReg == Reg::PC) // preload
    {

    }
    else if(baseReg == Reg::PC) // LDR(S)B (literal)
    {}
    else if(!(op1 & 1) && op2 == 0) // LDR(S)B (register)
    {
        bool isSigned = op1 & 2;

        auto mReg = static_cast<Reg>(opcode & 0xF);
        auto shift = (opcode >> 4) & 3;

        uint32_t addr = loReg(baseReg) + (loReg(mReg) << shift);

        uint32_t data = readMem8(addr, cycles);

        if(isSigned && (data & 0x80))
            data |= 0xFFFFFF00;

        loReg(dstReg) = data;

        return cycles;
    }
    else if(!(op1 & 1) && (op2 & 0x3C) == 0x38) // LDR(S)BT
    {}
    else // LDR(S)B (immediate)
    {
        bool isSigned = op1 & 2;

        if(op1 & 1) // + 12 bit imm
        {
            auto offset = (opcode & 0xFFF);

            uint32_t addr = loReg(baseReg) + offset;

            uint32_t data = readMem8(addr, cycles);

            if(isSigned && (data & 0x80))
                data |= 0xFFFFFF00;

            loReg(dstReg) = data;

            return cycles;
        }
        else // +/- 8 bit imm
        {
            auto offset = (opcode & 0xFF);

            bool writeback = opcode & (1 << 8);
            bool add = opcode & (1 << 9);
            bool index = opcode & (1 << 10);

            uint32_t offsetAddr = add ? loReg(baseReg) + offset : loReg(baseReg) - offset;
            uint32_t addr = index ? offsetAddr : loReg(baseReg);

            uint32_t data = readMem8(addr, cycles);

            if(isSigned && (data & 0x80))
                data |= 0xFFFFFF00;

            if(writeback)
                loReg(baseReg) = offsetAddr;

            loReg(dstReg) = data;

            return cycles;
        }
    }

    printf("Unhandled load byte/hint opcode %08X (%X %X) @%08X\n", opcode, op1, op2, pc - 6);
    exit(1);
}

int ARMv6MCore::doTHUMB32BitLoadHalfHint(uint32_t opcode, uint32_t pc)
{
    auto op1 = (opcode >> 23) & 3;
    auto op2 = (opcode >> 6) & 0x3F;

    auto baseReg = static_cast<Reg>((opcode >> 16) & 0xF);
    auto dstReg = static_cast<Reg>((opcode >> 12) & 0xF);

    int cycles = pcSCycles * 2;

    if(dstReg == Reg::PC) // unallocated hints
    {

    }
    else if(baseReg == Reg::PC) // LDR(S)H (literal)
    {}
    else if(!(op1 & 1) && op2 == 0) // LDR(S)H (register)
    {
        bool isSigned = op1 & 2;

        auto mReg = static_cast<Reg>(opcode & 0xF);
        auto shift = (opcode >> 4) & 3;

        uint32_t addr = loReg(baseReg) + (loReg(mReg) << shift);

        uint32_t data = readMem16(addr, cycles);

        if(isSigned && (data & 0x8000))
            data |= 0xFFFF0000;

        loReg(dstReg) = data;

        return cycles;
    }
    else if(!(op1 & 1) && (op2 & 0x3C) == 0x38) // LDR(S)HT
    {}
    else // LDR(S)H (immediate)
    {
        bool isSigned = op1 & 2;

        if(op1 & 1) // + 12 bit imm
        {
            auto offset = (opcode & 0xFFF);

            uint32_t addr = loReg(baseReg) + offset;

            uint32_t data = readMem16(addr, cycles);

            if(isSigned && (data & 0x8000))
                data |= 0xFFFF0000;

            loReg(dstReg) = data;

            return cycles;
        }
        else // +/- 8 bit imm
        {
            auto offset = (opcode & 0xFF);

            bool writeback = opcode & (1 << 8);
            bool add = opcode & (1 << 9);
            bool index = opcode & (1 << 10);

            uint32_t offsetAddr = add ? loReg(baseReg) + offset : loReg(baseReg) - offset;
            uint32_t addr = index ? offsetAddr : loReg(baseReg);

            uint32_t data = readMem16(addr, cycles);

            if(isSigned && (data & 0x8000))
                data |= 0xFFFF0000;

            if(writeback)
                loReg(baseReg) = offsetAddr;

            loReg(dstReg) = data;

            return cycles;
        }
    }

    printf("Unhandled load half/hint opcode %08X (%X %X) @%08X\n", opcode, op1, op2, pc - 6);
    exit(1);
}

int ARMv6MCore::doTHUMB32BitLoadWord(uint32_t opcode, uint32_t pc)
{
    auto op1 = (opcode >> 23) & 3;
    auto op2 = (opcode >> 6) & 0x3F;

    auto baseReg = static_cast<Reg>((opcode >> 16) & 0xF);
    auto dstReg = static_cast<Reg>((opcode >> 12) & 0xF);

    assert(!(op1 & 2));

    int cycles = pcSCycles * 2;
    uint32_t data;

    if(baseReg == Reg::PC) // LDR (literal)
    {
        bool add = opcode & (1 << 23);

        auto offset = (opcode & 0xFFF);

        uint32_t addr = (pc - 2) & ~2;
        
        if(add)
            addr += offset;
        else
            addr -= offset;
        
        data = readMem32(addr, cycles);
    }
    else if(op1 == 0 && op2 == 0) // LDR (register)
    {
        auto mReg = static_cast<Reg>(opcode & 0xF);
        auto shift = (opcode >> 4) & 3;

        uint32_t addr = loReg(baseReg) + (loReg(mReg) << shift);

        data = readMem32(addr, cycles);
    }
    else if(op1 == 0 && (op2 & 0x3C) == 0x38) // LDRT
    {
        printf("Unhandled load word opcode %08X (%X %X) @%08X\n", opcode, op1, op2, pc - 6);
        exit(1);
    }
    else // LDR (immediate)
    {
        if(op1 == 1) // + 12 bit imm
        {
            auto offset = (opcode & 0xFFF);

            uint32_t addr = loReg(baseReg) + offset;

            data = readMem32(addr, cycles);
        }
        else // +/- 8 bit imm
        {
            auto offset = (opcode & 0xFF);

            bool writeback = opcode & (1 << 8);
            bool add = opcode & (1 << 9);
            bool index = opcode & (1 << 10);

            uint32_t offsetAddr = add ? loReg(baseReg) + offset : loReg(baseReg) - offset;
            uint32_t addr = index ? offsetAddr : loReg(baseReg);

            int cycles = pcSCycles * 2;
            data = readMem32(addr, cycles);

            if(writeback)
                loReg(baseReg) = offsetAddr;
        }
    }

    if(dstReg == Reg::PC)
    {
        assert(data & 1);
        // address should be aligned
        updateTHUMBPC(data & ~1);
    }
    else
        loReg(dstReg) = data;

    return cycles;
}

int ARMv6MCore::doTHUMB32BitDataProcessingReg(uint32_t opcode, uint32_t pc)
{
    auto op1 = (opcode >> 20) & 0xF;
    auto op2 = (opcode >> 4) & 0xF;

    auto nReg = static_cast<Reg>((opcode >> 16) & 0xF);

    if((op1 & 0b1000) && !(op2 & 0b1000)) // parallel add/sub
    {
        assert((opcode & 0xF000) == 0xF000);

        auto nReg = static_cast<Reg>((opcode >> 16) & 0xF);
        auto dstReg = static_cast<Reg>((opcode >> 8) & 0xF);
        auto mReg = static_cast<Reg>(opcode & 0xF);

        if(op2 & 0b100) // unsigned
        {
            switch(op1 & 7)
            {
                case 0: // U*ADD8
                {
                    int sum[4];

                    for(int i = 0; i < 4; i++)
                    {
                        uint8_t n = loReg(nReg) >> (i * 8);
                        uint8_t m = loReg(mReg) >> (i * 8);
                        sum[i] = n + m;
                    }

                    loReg(dstReg) = (sum[0] & 0xFF) | (sum[1] & 0xFF) << 8 | (sum[2] & 0xFF) << 16 | (sum[3] & 0xFF) << 24;

                    cpsr = (cpsr & ~(Flag_GE0 | Flag_GE1 | Flag_GE2 | Flag_GE3))
                         | ((sum[0] >= 0x100) ? Flag_GE0 : 0)
                         | ((sum[1] >= 0x100) ? Flag_GE1 : 0)
                         | ((sum[2] >= 0x100) ? Flag_GE2 : 0)
                         | ((sum[3] >= 0x100) ? Flag_GE3 : 0);

                    return pcSCycles * 2;
                }
                //1: U*ADD16
                //2: U*ASX
                //4: U*SUB8
                //5: U*SUB16
                //6: U*SAX
            }
        }
        else // signed
        {}
    }
    else if(op2 == 0)
    {
        bool setFlags = opcode & (1 << 20);
        auto dReg = static_cast<Reg>((opcode >> 8) & 0xF);
        auto mReg = static_cast<Reg>(opcode & 0xF);
    
        switch(op1 >> 1)
        {
            case 0: // LSL
            {
                auto shift = loReg(mReg) & 0xFF;
                auto val = loReg(nReg);

                auto carry = cpsr & Flag_C;
                uint32_t res;

                if(shift >= 32)
                {
                    carry = shift == 32 ? (val & 1) : 0;
                    carry = carry ? Flag_C : 0;
                    loReg(dReg) = res = 0;
                }
                else if(shift)
                {
                    carry = val & (1 << (32 - shift)) ? Flag_C : 0;
                    res = val << shift;

                    loReg(dReg) = res;
                }
                else
                    loReg(dReg) = res = val;

                if(setFlags)
                    cpsr = (cpsr & ~(Flag_C | Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry;
                
                return pcSCycles * 2;
            }

            case 1: // LSR
            {
                auto shift = loReg(mReg) & 0xFF;
                auto val = loReg(nReg);

                auto carry = cpsr & Flag_C;
                uint32_t res;

                if(shift >= 32)
                {
                    carry = shift == 32 ? (val & (1 << 31)) : 0;
                    carry = carry ? Flag_C : 0;
                    loReg(dReg) = res = 0;
                }
                else if(shift)
                {
                    carry = val & (1 << (shift - 1)) ? Flag_C : 0;
                    res = val >> shift;

                    loReg(dReg) = res;
                }
                else
                    loReg(dReg) = res = val;

                if(setFlags)
                    cpsr = (cpsr & ~(Flag_C | Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry;
                
                return pcSCycles * 2;
            }

            case 2: // ASR
            {
                auto shift = loReg(mReg) & 0xFF;
                auto val = loReg(nReg);

                auto carry = cpsr & Flag_C;
                auto sign = val & signBit;
                uint32_t res;

                if(shift >= 32)
                {
                    carry = sign ? Flag_C : 0;
                    loReg(dReg) = res = sign ? 0xFFFFFFFF : 0;
                }
                else if(shift)
                {
                    carry = val & (1 << (shift - 1)) ? Flag_C : 0;
                    res = static_cast<int32_t>(val) >> shift;

                    loReg(dReg) = res;
                }
                else
                    loReg(dReg) = res = val;

                if(setFlags)
                    cpsr = (cpsr & ~(Flag_C | Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry;
                
                return pcSCycles * 2;
            }
            // 3: ROR
        }
    }
    else if(op2 & 0b1000)
    {
        if(op1 == 0) // SXTAH/SXTH
        {
            auto dReg = static_cast<Reg>((opcode >> 8) & 0xF);
            auto mReg = static_cast<Reg>(opcode & 0xF);
            int rotation = (opcode >> 1) & 0x18;

            auto val = (loReg(mReg) >> rotation) | (loReg(mReg) << (32 - rotation));
            val = (val & 0x8000) ? val | 0xFFFF0000 : val & 0xFFFF;

            if(nReg == Reg::PC) // SXTH
                loReg(dReg) = val;
            else // SXTAH
                loReg(dReg) = loReg(nReg) + val;

            return pcSCycles * 2;
        }
        else if(op1 == 1) // UXTAH/UXTH
        {
            auto dReg = static_cast<Reg>((opcode >> 8) & 0xF);
            auto mReg = static_cast<Reg>(opcode & 0xF);
            int rotation = (opcode >> 1) & 0x18;

            auto val = (loReg(mReg) >> rotation) | (loReg(mReg) << (32 - rotation));
            val &= 0xFFFF;

            if(nReg == Reg::PC) // UXTH
                loReg(dReg) = val;
            else // UXTAH
                loReg(dReg) = loReg(nReg) + val;

            return pcSCycles * 2;
        }
        else if(op1 == 4) // SXTAB/SXTB
        {
            auto dReg = static_cast<Reg>((opcode >> 8) & 0xF);
            auto mReg = static_cast<Reg>(opcode & 0xF);
            int rotation = (opcode >> 1) & 0x18;

            auto val = (loReg(mReg) >> rotation) | (loReg(mReg) << (32 - rotation));
            val = (val & 0x80) ? val | 0xFFFFFF00 : val & 0xFF;

            if(nReg == Reg::PC) // SXTB
                loReg(dReg) = val;
            else // SXTAB
                loReg(dReg) = loReg(nReg) + val;

            return pcSCycles * 2;
        }
        else if(op1 == 5) // UXTAB/UXTB
        {
            auto dReg = static_cast<Reg>((opcode >> 8) & 0xF);
            auto mReg = static_cast<Reg>(opcode & 0xF);
            int rotation = (opcode >> 1) & 0x18;

            auto val = (loReg(mReg) >> rotation) | (loReg(mReg) << (32 - rotation));
            val &= 0xFF;

            if(nReg == Reg::PC) // UXTB
                loReg(dReg) = val;
            else // UXTAB
                loReg(dReg) = loReg(nReg) + val;

            return pcSCycles * 2;
        }
        else if(op1 & 0b1000) // misc ops
        {
            assert(!(op1 & 0b100));
            assert(!(op2 & 0b100));

            switch(op1 & 3)
            {
                case 2: // SEL
                {
                    auto nReg = static_cast<Reg>((opcode >> 16) & 0xF);
                    auto dReg = static_cast<Reg>((opcode >> 8) & 0xF);
                    auto mReg = static_cast<Reg>(opcode & 0xF);

                    uint32_t nMask = ((cpsr & Flag_GE0) ? 0xFF : 0)
                                   | ((cpsr & Flag_GE1) ? 0xFF00 : 0)
                                   | ((cpsr & Flag_GE2) ? 0xFF0000 : 0)
                                   | ((cpsr & Flag_GE3) ? 0xFF000000 : 0);

                    loReg(dReg) = (loReg(nReg) & nMask) | (loReg(mReg) & ~nMask);

                    return pcSCycles * 2;
                }
                case 3: // CLZ
                {
                    assert(op2 == 0b1000);
                    assert(((opcode >> 16) & 0xF) == (opcode & 0xF)); // m encoded twice

                    auto dReg = static_cast<Reg>((opcode >> 8) & 0xF);
                    auto mReg = static_cast<Reg>(opcode & 0xF);

                    loReg(dReg) = __builtin_clz(loReg(mReg));

                    return pcSCycles * 2;
                }
            }
        }
    }

    printf("Unhandled dp reg opcode %08X (%X %X) @%08X\n", opcode, op1, op2, pc - 6);
    exit(1);
}

int ARMv6MCore::doTHUMB32BitMultiplyDiff(uint32_t opcode, uint32_t pc)
{
    auto op1 = (opcode >> 20) & 7;
    auto op2 = (opcode >> 4) & 3;

    auto nReg = static_cast<Reg>((opcode >> 16) & 0xF);
    auto aReg = static_cast<Reg>((opcode >> 12) & 0xF);
    auto dstReg = static_cast<Reg>((opcode >> 8) & 0xF);
    auto mReg = static_cast<Reg>(opcode & 0xF);

    assert(((opcode >> 6) & 3) == 0);

    if(op1 == 0)
    {
        if(op2 == 0)
        {
            if(aReg == Reg::PC) // MUL
                loReg(dstReg) = loReg(nReg) * loReg(mReg);
            else // MLA
                loReg(dstReg) = loReg(nReg) * loReg(mReg) + loReg(aReg);

            return pcSCycles * 2;
        }
        else if(op2 == 1) // MLS
        {
            loReg(dstReg) = loReg(aReg) - loReg(nReg) * loReg(mReg);

            return pcSCycles * 2;
        }
    }
    else if(op1 == 1)
    {
        bool nHigh = opcode & (1 << 5);
        bool mHigh = opcode & (1 << 4);

        auto op1 = static_cast<int16_t>(loReg(nReg) >> (nHigh ? 16 : 0));
        auto op2 = static_cast<int16_t>(loReg(mReg) >> (mHigh ? 16 : 0));

        if(aReg == Reg::PC) // SMUL[BT][BT]
            loReg(dstReg) = static_cast<int32_t>(op1) * static_cast<int32_t>(op2);
        else // SMLA[BT][BT]
            loReg(dstReg) = static_cast<int32_t>(op1) * static_cast<int32_t>(op2) + loReg(aReg);

        return pcSCycles * 2;
    }

    printf("Unhandled mul/diff opcode %08X (%X %X) @%08X\n", opcode, op1, op2, pc - 6);
    exit(1);
}

int ARMv6MCore::doTHUMB32BitLongMultiplyDiv(uint32_t opcode, uint32_t pc)
{
    auto op1 = (opcode >> 20) & 7;
    auto op2 = (opcode >> 4) & 0xF;

    auto nReg = static_cast<Reg>((opcode >> 16) & 0xF);
    auto dstLoReg = static_cast<Reg>((opcode >> 12) & 0xF);
    auto dstHiReg = static_cast<Reg>((opcode >> 8) & 0xF);
    auto mReg = static_cast<Reg>(opcode & 0xF);

    if(op1 == 0) // SMULL
    {
        assert(op2 == 0);

        int64_t res = static_cast<int64_t>(loReg(nReg)) * static_cast<int64_t>(loReg(mReg));

        loReg(dstLoReg) = res & 0xFFFFFFFF;
        loReg(dstHiReg) = res >> 32;

        return pcSCycles * 2;
    }
    else if(op1 == 1) // SDIV
    {
        assert(op2 == 0xF);
        assert(dstLoReg == Reg::PC);

        int32_t res = static_cast<int32_t>(loReg(nReg)) / static_cast<int32_t>(loReg(mReg));

        loReg(dstHiReg) = res;

        return pcSCycles * 2;
    }
    else if(op1 == 2) // UMULL
    {
        assert(op2 == 0);

        uint64_t res = static_cast<uint64_t>(loReg(nReg)) * static_cast<uint64_t>(loReg(mReg));

        loReg(dstLoReg) = res & 0xFFFFFFFF;
        loReg(dstHiReg) = res >> 32;

        return pcSCycles * 2;
    }
    else if(op1 == 3) // UDIV
    {
        assert(op2 == 0xF);
        assert(dstLoReg == Reg::PC);

        auto res = loReg(nReg) / loReg(mReg);

        loReg(dstHiReg) = res;

        return pcSCycles * 2;
    }
    else if(op1 == 6)
    {
        if(op2 == 0) // UMLAL
        {
            uint64_t a = loReg(dstLoReg) | static_cast<uint64_t>(loReg(dstHiReg)) << 32;
            uint64_t res = static_cast<uint64_t>(loReg(nReg)) * static_cast<uint64_t>(loReg(mReg)) + a;

            loReg(dstLoReg) = res & 0xFFFFFFFF;
            loReg(dstHiReg) = res >> 32;

            return pcSCycles * 2;
        }
    }

    printf("Unhandled long mul/div opcode %08X (%X %X) @%08X\n", opcode, op1, op2, pc - 6);
    exit(1);
}

void ARMv6MCore::updateTHUMBPC(uint32_t pc)
{
    // called when PC is updated in THUMB mode (except for incrementing)
    assert(!(pc & 1));

    if(pc >> 16 == 0x08BA)
    {
        if(apiCallback)
            apiCallback((pc & 0xFFFF) >> 1, regs);

        // fake the return (if this isn't a BL)
        if(loReg(Reg::PC) != (loReg(Reg::LR) & ~1))
            updateTHUMBPC(loReg(Reg::LR) & ~1);
        return;
    }

    if(pcPtr && pc >> 24 == loReg(Reg::PC) >> 24)
    {
        // memory region didn't change, skip recaclculating ptr/cycles
        [[maybe_unused]] auto thumbPCPtr = reinterpret_cast<const uint16_t *>(pcPtr + pc);
        assert(mem.verifyPointer(thumbPCPtr, pc));
    }
    else
    {
        pcPtr = std::as_const(mem).mapAddress(pc); // force const mapAddress
        if(pcPtr)
            pcPtr -= pc;
        pcSCycles = 1;
        pcNCycles = 1;
    }

    // refill the pipeline
    if(pcPtr)
    {
        auto thumbPCPtr = reinterpret_cast<const uint16_t *>(pcPtr + pc);
        decodeOp = *thumbPCPtr++;
        fetchOp = *thumbPCPtr;
    }
    else
    {
        // TODO: either fix the optimisation or remove it, this is messy
        int tmp;
        decodeOp = mem.read<uint16_t>(pc, tmp, true);
        fetchOp = mem.read<uint16_t>(pc + 2, tmp, true);
    }

    loReg(Reg::PC) = pc + 2; // pointing at last fetch
}

int ARMv6MCore::handleException()
{
    // get cur priority
    int curException = cpsr & 0x3F;
    int curPrio = getExceptionPriority(curException);

    // find highest priority pending exception
    int newException = 0;
    int newPrio = 4;

    for(int i = 2; i < 48 && newPrio; i++)
    {
        // skip not pending
        if(!(exceptionPending & (1ull << i)))
            continue;

        // skip not enabled external interrupt
        if(i >= 16 && !(nvicEnabled & (1 << (i - 16))))
            continue;

        int prio = getExceptionPriority(i);

        if(prio < newPrio)
        {
            newPrio = prio;
            newException = i;
        }
    }

    // no higher priority exception
    if(newPrio >= curPrio && newException >= curException)
        return 0;

    // push to stack
    auto &sp = reg(Reg::SP);
    auto spAlign = sp & 4;
    sp = (sp - 0x20) & ~4;

    int cycles = 0;
    writeMem32(sp +  0, loReg(Reg::R0 ), cycles);
    writeMem32(sp +  4, loReg(Reg::R1 ), cycles, true);
    writeMem32(sp +  8, loReg(Reg::R2 ), cycles, true);
    writeMem32(sp + 12, loReg(Reg::R3 ), cycles, true);
    writeMem32(sp + 16, loReg(Reg::R12), cycles, true);
    writeMem32(sp + 20, loReg(Reg::LR ), cycles, true);

    writeMem32(sp + 24, loReg(Reg::PC) - 2, cycles, true);
    writeMem32(sp + 28, cpsr | spAlign << 7, cycles, true);

    if(cpsr & 0x3F) // in handler
        loReg(Reg::LR) = 0xFFFFFFF1;
    else if(control & (1 << 1)/*SPSEL*/)
        loReg(Reg::LR) = 0xFFFFFFFD;
    else
        loReg(Reg::LR) = 0xFFFFFFF9;

    // take exception
    cpsr = (cpsr & ~0x3F) | newException;

    exceptionActive |= 1ull << newException;
    exceptionPending &= ~(1ull << newException);
    needException = false;

    // set event/wake up
    eventFlag = true;
    if(sleeping)
        sleeping = false;

    auto vtor = scbRegs[2];
    auto addr = readMem32(vtor + newException * 4, cycles);

    assert(addr & 1);
    updateTHUMBPC(addr & ~1);

    return cycles + pcSCycles * 2 + pcNCycles;
}

int ARMv6MCore::handleExceptionReturn(uint32_t excRet)
{
    assert((excRet & 0xFFFFFF0) == 0xFFFFFF0);

    int exception = cpsr & 0x3F;
    auto &sp = loReg((excRet & 0xF) == 0xD ? Reg::PSP : Reg::MSP);

    exceptionActive &= ~(1ull << exception);

    // pop from stack
    int cycles = 0;
    loReg(Reg::R0)  = readMem32(sp +  0, cycles);
    loReg(Reg::R1)  = readMem32(sp +  4, cycles, true);
    loReg(Reg::R2)  = readMem32(sp +  8, cycles, true);
    loReg(Reg::R3)  = readMem32(sp + 12, cycles, true);
    loReg(Reg::R12) = readMem32(sp + 16, cycles, true);
    loReg(Reg::LR)  = readMem32(sp + 20, cycles, true);
    
    auto newPC = readMem32(sp + 24, cycles, true);
    auto newPSR = readMem32(sp + 28, cycles, true);

    sp = (sp + 0x20) | (newPSR & (1 << 9)) >> 7;

    cpsr = newPSR & 0xF100003F;

    // set event
    eventFlag = true;
    if(sleeping)
        sleeping = false;

    // TODO: sleep on exit

    updateTHUMBPC(newPC & ~1);

    checkPendingExceptions();

    return cycles; // caller should handle the branch
}

int ARMv6MCore::getExceptionPriority(int exception) const
{
    switch(exception)
    {
        case 0: // thread/no exception
            return 4;

        case 2: // NMI
            return -2;
        
        case 3: // HardFault
            return -1;

        case 11: // SVCall
            return scbRegs[7]/*SHPR2*/ >> 30;
        case 14: // PendSV
            return  (scbRegs[8]/*SHPR3*/ >> 22) & 3;
        case 15: // SysTick
            return scbRegs[8]/*SHPR3*/ >> 30;
        
        default:
            assert(exception >= 16);
            // external interrupt
            int shift = 6 + (exception & 3) * 8;
            return (nvicPriority[(exception - 16) / 4] >> shift) & 3;
    }
}

void ARMv6MCore::checkPendingExceptions()
{
    needException = false;

    // mask disabled
    uint64_t mask = nvicEnabled << 16 | 0xFFFF;
    if(!(exceptionPending & mask))
        return;

    // get cur priority
    int curException = cpsr & 0x3F;
    int curPrio = getExceptionPriority(curException);

    // find highest priority pending exception
    int newException = 0;
    int newPrio = 4;

    uint64_t maskedExceptions = exceptionPending & mask;

    for(int i = 2; i < 48 && newPrio; i++)
    {
        // skip not pending
        if(!(maskedExceptions & (1ull << i)))
            continue;

        int prio = getExceptionPriority(i);

        if(prio < newPrio)
        {
            newPrio = prio;
            newException = i;
        }
    }

    // higher priority exception
    if(newPrio < curPrio || newException < curException)
        needException = true;
}

void ARMv6MCore::updateSysTick(int sysCycles)
{
    if(!(sysTickRegs[0]/*SYST_CSR*/ & (1 << 2)/*CLKSOURCE*/))
        return; // TODO: watchdog tick

    sysTickRegs[2]/*CVR*/ = (sysTickRegs[2] - sysCycles) & 0xFFFFFF;

    if(!sysTickRegs[2])
        sysTickRegs[2] = sysTickRegs[1] /*RVR*/;
}