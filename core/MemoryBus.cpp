#include <cassert>
#include <cstdio>
#include <cstring>

#include "MemoryBus.h"

#include "ARMv6MCore.h"

bool updateReg(uint32_t &curVal, uint32_t newVal, int atomic)
{
    auto oldVal = curVal;

    if(atomic == 0)
        curVal = newVal;
    else if(atomic == 1)
        curVal ^= newVal;
    else if(atomic == 2)
        curVal |= newVal;
    else
        curVal &= ~newVal;

    return curVal != oldVal;
}

enum MemoryRegion
{
    Region_ITCM        = 0x00,
    Region_Flash       = 0x08,
    Region_DTCM        = 0x20,
    Region_D1          = 0x24,
    Region_D2          = 0x30,
    Region_D3_Backup   = 0x38,
    Region_QSPI        = 0x90,
};

template uint8_t MemoryBus::read(uint32_t addr, int &cycles, bool sequential);
template uint16_t MemoryBus::read(uint32_t addr, int &cycles, bool sequential);
template uint32_t MemoryBus::read(uint32_t addr, int &cycles, bool sequential);
template void MemoryBus::write(uint32_t addr, uint8_t val, int &cycles, bool sequential);
template void MemoryBus::write(uint32_t addr, uint16_t val, int &cycles, bool sequential);
template void MemoryBus::write(uint32_t addr, uint32_t val, int &cycles, bool sequential);

MemoryBus::MemoryBus()
{
}

void MemoryBus::reset()
{
}

template<class T>
T MemoryBus::read(uint32_t addr, int &cycles, bool sequential)
{
    auto accessCycles = [&cycles, this](int c)
    {
        cycles += c;
    };

    switch(addr >> 24)
    {
        case Region_ITCM:
            accessCycles(1);
            return doRead<T>(itcm, addr);

        case Region_Flash:
            accessCycles(1);
            return doRead<T>(flash, addr);

        case Region_DTCM:
            accessCycles(1);
            return doRead<T>(dtcm, addr);

        case Region_D1:
            accessCycles(1);
            return doRead<T>(d1RAM, addr);

        case Region_D2:
            accessCycles(1);
            return doD2Read<T>(addr);

        case Region_D3_Backup:
            accessCycles(1);
            return doD3BackupRead<T>(addr);

        case Region_QSPI:
            accessCycles(1);
            return doRead<T>(qspiFlash, addr);
    }

    return doOpenRead<T>(addr);
}

template<class T>
void MemoryBus::write(uint32_t addr, T data, int &cycles, bool sequential)
{
    auto accessCycles = [&cycles, this](int c)
    {
        cycles += c;
    };

    switch(addr >> 24)
    {
        case Region_ITCM:
            accessCycles(1);
            doWrite<T>(itcm, addr, data);
            return;

        case Region_Flash:
            accessCycles(1);
            doWrite<T>(flash, addr, data);
            return;

        case Region_DTCM:
            accessCycles(1);
            doWrite<T>(dtcm, addr, data);
            return;

        case Region_D1:
            accessCycles(1);
            doWrite<T>(d1RAM, addr, data);
            return;

        case Region_D2:
            accessCycles(1);
            doD2Write<T>(addr, data);
            return;

        case Region_D3_Backup:
            accessCycles(1);
            doD3BackupWrite<T>(addr, data);
            return;

        case Region_QSPI:
            accessCycles(1);
            doWrite<T>(qspiFlash, addr, data);
            return;
    }
}

const uint8_t *MemoryBus::mapAddress(uint32_t addr) const
{
    switch(addr >> 24)
    {
        case Region_ITCM:
            return itcm + (addr & 0xFFFF);

        case Region_Flash:
            return flash + (addr & 0x1FFFF);

        case Region_DTCM:
            return dtcm + (addr & 0x1FFFF);

        case Region_D1:
            return d1RAM + (addr & 0x7FFFF);

        //case Region_D2:
        //    return d2RAM + (addr & 0);

        //case Region_D3_Backup:
        //    return d3RAM + (addr & 0);

        case Region_QSPI:
            return qspiFlash + (addr & 0x1FFFFFF);
    }

    return reinterpret_cast<const uint8_t *>(&dummy);
}

uint8_t *MemoryBus::mapAddress(uint32_t addr)
{
    switch(addr >> 24)
    {
        case Region_ITCM:
            return itcm + (addr & 0xFFFF);

        case Region_Flash:
            return flash + (addr & 0x1FFFF);

        case Region_DTCM:
            return dtcm + (addr & 0x1FFFF);

        case Region_D1:
            return d1RAM + (addr & 0x7FFFF);

        //case Region_D2:
        //    return d2RAM + (addr & 0);

        //case Region_D3_Backup:
        //    return d3RAM + (addr & 0);

        case Region_QSPI:
            return qspiFlash + (addr & 0x1FFFFFF);
    }

    return nullptr;
}

template<class T, size_t size>
T MemoryBus::doRead(const uint8_t (&mem)[size], uint32_t addr) const
{
    // use size of type for alignment
    return *reinterpret_cast<const T *>(mem + (addr & (size - sizeof(T))));
}

template<class T, size_t size>
void MemoryBus::doWrite(uint8_t (&mem)[size], uint32_t addr, T data)
{
    // use size of type for alignment
    *reinterpret_cast< T *>(mem + (addr & (size - sizeof(T)))) = data;
}

template<class T>
T MemoryBus::doD2Read(uint32_t addr) const
{
    addr &= 0x7FFFF;

    if(addr < sizeof(d2RAM))
        return *reinterpret_cast<const T *>(d2RAM + addr);

    return doOpenRead<T>(addr);
}

template<class T>
void MemoryBus::doD2Write(uint32_t addr, T data)
{
    addr &= 0x7FFFF;
    if(addr < sizeof(d2RAM))
    {
        *reinterpret_cast<T *>(d2RAM + addr) = data;
        return;
    }
}

template<class T>
T MemoryBus::doD3BackupRead(uint32_t addr) const
{
    if(addr >= 0x38800000)
        return doRead<T>(backupRAM, addr);

    return doRead<T>(d3RAM, addr);
}

template<class T>
void MemoryBus::doD3BackupWrite(uint32_t addr, T data)
{
    if(addr >= 0x38800000)
        return doWrite(backupRAM, addr, data);

    return doWrite(d3RAM, addr, data);
}

template<class T>
T MemoryBus::doOpenRead(uint32_t addr) const
{
    return static_cast<T>(0xBADADD55); // TODO
}
