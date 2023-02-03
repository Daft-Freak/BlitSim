#pragma once
#include <cstddef>
#include <cstdint>


class MemoryBus
{
public:
    MemoryBus();

    void reset();

    template<class T>
    T read(uint32_t addr, int &cycles, bool sequential);
    template<class T>
    void write(uint32_t addr, T data, int &cycles, bool sequential);

    const uint8_t *mapAddress(uint32_t addr) const;
    uint8_t *mapAddress(uint32_t addr);

    // verify that pointer returns the same as a regular read to the address
    // without affecting prefetch (used for asserts)
    template<class T>
    bool verifyPointer(const T *ptr, uint32_t addr)
    {
        int tmp;
        bool ret = read<T>(addr, tmp, false) == *ptr;
        return ret;
    }

private:
    template<class T, size_t size>
    T doRead(const uint8_t (&mem)[size], uint32_t addr) const;
    template<class T, size_t size>
    void doWrite(uint8_t (&mem)[size], uint32_t addr, T data);

    template<class T>
    T doROMRead(uint32_t addr) const;

    template<class T>
    T doD2Read(uint32_t addr) const;
    template<class T>
    void doD2Write(uint32_t addr, T data);

    template<class T>
    T doD3BackupRead(uint32_t addr) const;
    template<class T>
    void doD3BackupWrite(uint32_t addr, T data);

    template<class T>
    T doOpenRead(uint32_t addr) const;

    uint8_t itcm[64 * 1024];             // @ 00000000
    uint8_t flash[128 * 1024];           // @ 08000000
    uint8_t dtcm[128 * 1024];            // @ 20000000
    uint8_t d1RAM[512 * 1024];           // @ 24000000
    uint8_t d2RAM[288 * 1024];           // @ 30000000
    uint8_t d3RAM[64 * 1024];            // @ 38000000
    uint8_t backupRAM[4 * 1024];         // @ 38800000
    uint8_t qspiFlash[32 * 1024 * 1024]; // @ 90000000

    uint32_t dummy = 0xBADADD55;
};