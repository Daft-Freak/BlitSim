#pragma once
#include <cstdint>
#include <mutex>

#include "MemoryBus.h"

class ARMv7MCore final
{
public:
    using APICallback = void(*)(int index, uint32_t *regs);

    ARMv7MCore(MemoryBus &mem);

    void reset();

    void setSP(uint32_t val);

    void runCall(uint32_t addr, uint32_t r0 = 0, uint32_t r1 = 0);
    void runCallThread(uint32_t addr, uint32_t r0 = 0, uint32_t r1 = 0);
    void runCallLocked(uint32_t addr, uint32_t r0 = 0, uint32_t r1 = 0);

    void pause();
    void resume();
    bool getPaused() const {return paused;}

    MemoryBus &getMem() {return mem;}

    void setAPICallback(APICallback cb){apiCallback = cb;}

private:
    enum class Reg
    {
        R0 = 0,
        R1,
        R2,
        R3,
        R4,
        R5,
        R6,
        R7,
        // ARM mode/high
        R8,
        R9,
        R10,
        R11,
        R12,
        R13,
        R14,
        R15,

        // the other SP
        PSP,

        // aliases
        SP = R13,
        MSP = SP,
        LR = R14,
        PC = R15
    };

    enum Flags
    {
        // control
        Flag_T = (1 << 24), // thumb

        Flag_GE0 = (1 << 16),
        Flag_GE1 = (1 << 17),
        Flag_GE2 = (1 << 18),
        Flag_GE3 = (1 << 19),

        // condition codes
        Flag_Q = (1 << 27),
        Flag_V = (1 << 28),
        Flag_C = (1 << 29),
        Flag_Z = (1 << 30),
        Flag_N = (1 << 31)
    };

    Reg mapReg(Reg r) const
    {
        if(r == Reg::SP && (cpsr & 0x3F) == 0 && control & (1 << 1))
            return Reg::PSP;

        return r;
    }

    uint32_t reg(Reg r) const {return regs[static_cast<int>(mapReg(r))];}
    uint32_t &reg(Reg r) {return regs[static_cast<int>(mapReg(r))];}

    // THUMB, first 8 regs, also used when we don't want to map
    uint32_t loReg(Reg r) const {return regs[static_cast<int>(r)];}
    uint32_t &loReg(Reg r) {return regs[static_cast<int>(r)];}

    float sReg(int r) const {return reinterpret_cast<const float *>(fpRegs)[r];}
    float &sReg(int r) {return reinterpret_cast<float *>(fpRegs)[r];}

    double dReg(int r) const {return reinterpret_cast<const double *>(fpRegs)[r];}
    double &dReg(int r) {return reinterpret_cast<double *>(fpRegs)[r];}

    void doRunCall(uint32_t addr, uint32_t r0, uint32_t r1);

    uint8_t readMem8(uint32_t addr);
    uint16_t readMem16(uint32_t addr);
    uint32_t readMem32(uint32_t addr);
    void writeMem8(uint32_t addr, uint8_t data);
    void writeMem16(uint32_t addr, uint16_t data);
    void writeMem32(uint32_t addr, uint32_t data);

    inline bool inIT() {return itState & 0xF;}
    bool checkIT(uint16_t opcode);
    void advanceIT();

    void executeTHUMBInstruction();

    void doTHUMB01MoveShifted(uint16_t opcode, uint32_t pc);
    void doTHUMB0102(uint16_t opcode, uint32_t pc);
    void doTHUMB03(uint16_t opcode, uint32_t pc);
    void doTHUMB040506(uint16_t opcode, uint32_t pc);
    void doTHUMB04ALU(uint16_t opcode, uint32_t pc);
    void doTHUMB05HiReg(uint16_t opcode, uint32_t pc);
    void doTHUMB06PCRelLoad(uint16_t opcode, uint32_t pc);
    void doTHUMB0708(uint16_t opcode, uint32_t pc);
    void doTHUMB09LoadStoreWord(uint16_t opcode, uint32_t pc);
    void doTHUMB09LoadStoreByte(uint16_t opcode, uint32_t pc);
    void doTHUMB10LoadStoreHalf(uint16_t opcode, uint32_t pc);
    void doTHUMB11SPRelLoadStore(uint16_t opcode, uint32_t pc);
    void doTHUMB12LoadAddr(uint16_t opcode, uint32_t pc);
    void doTHUMBMisc(uint16_t opcode, uint32_t pc);
    void doTHUMB13SPOffset(uint16_t opcode, uint32_t pc);
    void doTHUMB14PushPop(uint16_t opcode, uint32_t pc);
    void doTHUMB15MultiLoadStore(uint16_t opcode, uint32_t pc);
    void doTHUMB1617(uint16_t opcode, uint32_t pc);
    void doTHUMB18UncondBranch(uint16_t opcode, uint32_t pc);

    uint32_t getShiftedReg(uint32_t opcode, bool &carry);
    void doDataProcessing(int op, Reg nReg, uint32_t op2, Reg dReg, bool carry, bool setFlags);

    void doTHUMB32BitInstruction(uint16_t opcode, uint32_t pc);
    void doTHUMB32BitLoadStoreMultiple(uint32_t opcode, uint32_t pc);
    void doTHUMB32BitLoadStoreDualEx(uint32_t opcode, uint32_t pc);
    void doTHUMB32BitDataProcessingShiftedReg(uint32_t opcode, uint32_t pc);
    void doTHUMB32BitCoprocessor(uint32_t opcode, uint32_t pc);
    void doTHUMB32BitDataProcessingModifiedImm(uint32_t opcode, uint32_t pc);
    void doTHUMB32BitDataProcessingPlainImm(uint32_t opcode, uint32_t pc);
    void doTHUMB32BitBranchMisc(uint32_t opcode, uint32_t pc);
    void doTHUMB32BitStoreSingle(uint32_t opcode, uint32_t pc);
    void doTHUMB32BitLoadByteHint(uint32_t opcode, uint32_t pc);
    void doTHUMB32BitLoadHalfHint(uint32_t opcode, uint32_t pc);
    void doTHUMB32BitLoadWord(uint32_t opcode, uint32_t pc);
    void doTHUMB32BitDataProcessingReg(uint32_t opcode, uint32_t pc);
    void doTHUMB32BitMultiplyDiff(uint32_t opcode, uint32_t pc);
    void doTHUMB32BitLongMultiplyDiv(uint32_t opcode, uint32_t pc);

    void doVFPDataProcessing(uint32_t opcode, uint32_t pc, bool dWidth);

    void updateTHUMBPC(uint32_t pc);

    static const uint32_t signBit = 0x80000000;

    // registers
    uint32_t regs[17]{};
    uint32_t cpsr;
    uint32_t primask, control;

    uint32_t fpRegs[32]; // 32x float/16x double
    uint32_t fpscr;

    Reg curSP = Reg::SP;

    const uint8_t *pcPtr = nullptr;

    // pipeline
    uint16_t fetchOp = 0, decodeOp = 0;

    uint8_t itState = 0;
    bool itStart = false;

    MemoryBus &mem;

    // high level 32blit firmware API emulation
    APICallback apiCallback;

    volatile bool pauseForIntr = false;
    std::mutex execMutex;

    bool paused = false;
};