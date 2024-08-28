#pragma once
#include <cstdint>
#include <map>
#include <vector>

#include "RecompilerGeneric.h"

#if defined(RECOMPILER_X86)
#include "X86Target.h"
#elif defined(RECOMPILER_THUMB)
#include "ThumbTarget.h"
#else
#error No recompiler target!
#endif

class ARMv7MCore;

class ARMv7MRecompiler
{
public:
    ARMv7MRecompiler(ARMv7MCore &cpu);
    ~ARMv7MRecompiler() = default;

    void run();

protected:
    ARMv7MCore &cpu;

    bool attemptToRun();

    void convertTHUMBToGeneric(uint32_t &pc, GenBlockInfo &genBlock);

    void compileEntry();

    static uint8_t readMem8(ARMv7MCore *cpu, uint32_t addr);
    static uint32_t readMem16(ARMv7MCore *cpu, uint32_t addr);
    static uint32_t readMem32(ARMv7MCore *cpu, uint32_t addr);

    static void writeMem8(ARMv7MCore *cpu, uint32_t addr, uint8_t data);
    static void writeMem16(ARMv7MCore *cpu, uint32_t addr, uint16_t data);
    static void writeMem32(ARMv7MCore *cpu, uint32_t addr, uint32_t data);

    static void invalidateCode(ARMv7MCore *cpu, uint32_t addr);

    uint8_t *codeBuf = nullptr, *curCodePtr;
    unsigned int codeBufSize;

    // cycles, entryAddr
    using CompiledFunc = void(*)(int, uint8_t *);

    struct FuncInfo
    {
        uint8_t *startPtr;
        uint8_t *endPtr;
        uint32_t endPC;
        uint8_t cpsrMode;
    };

    std::map<uint32_t, FuncInfo> compiled;
    uint32_t minRAMCode = 0xFFFFFFFF, maxRAMCode = 0;

    // common code
    CompiledFunc entryFunc = nullptr;

    // saved pointer on exit
    uint8_t *tmpSavedPtr = nullptr;
    bool exitCallFlag = false; // if we exited because of a call

    static const int savedExitsSize = 16;
    int curSavedExit = 0;
    std::tuple<uint8_t *, uint32_t, uint32_t> savedExits[savedExitsSize];

#if defined(RECOMPILER_X86)
    X86Target target;
#elif defined(RECOMPILER_THUMB)
    ThumbTarget target;
#endif
};