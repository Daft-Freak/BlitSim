#include <cassert>
#include <cstdio>
#include <cstring>

#include "MemoryBus.h"

#include "32blitAPI.h"
#include "ARMv6MCore.h"

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
    auto api = reinterpret_cast<blithw::API *>(mapAddress(0xF800));

    *api = {};
    api->version_major = blithw::api_version_major;
    api->version_minor = blithw::api_version_minor;

    // fake addresses in flash region
    api->set_screen_mode = 0x08BA0001;
    api->set_screen_palette = 0x08BA0003;
    api->now = 0x08BA0005;
    api->random = 0x08BA0007;
    api->exit = 0x08BA0009;

    api->debug = 0x08BA000B;

    api->open_file = 0x08BA000D;
    api->read_file = 0x08BA000F;
    api->write_file = 0x08BA0011;
    api->close_file = 0x08BA0013;
    api->get_file_length = 0x08BA0015;
    api->list_files = 0x08BA0017;
    api->file_exists = 0x08BA0019;
    api->directory_exists = 0x08BA001B;
    api->create_directory = 0x08BA001D;
    api->rename_file = 0x08BA001F;
    api->remove_file = 0x08BA0021;
    api->get_save_path = 0x08BA0023;
    api->is_storage_available = 0x08BA0025;

    api->enable_us_timer = 0x08BA0027;
    api->get_us_timer = 0x08BA0029;
    api->get_max_us_timer = 0x08BA002B;

    api->decode_jpeg_buffer = 0x08BA002D;
    api->decode_jpeg_file = 0x08BA002F;

    api->launch = 0x08BA0031;
    api->erase_game = 0x08BA0033;
    api->get_type_handler_metadata = 0x08BA0035;

    api->get_launch_path = 0x08BA0037;

    // multiplayer
    api->is_multiplayer_connected = 0x08BA0039;
    api->set_multiplayer_enabled = 0x08BA003B;
    api->send_message = 0x08BA003D;
    //api->message_received; // set by user

    api->flash_to_tmp = 0x08BA003F;
    api->tmp_file_closed = 0x08BA0041;

    api->get_metadata = 0x08BA0043;

    api->set_screen_mode_format = 0x08BA0045;
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

        case Region_D2:
            if((addr & 0x7FFFF) < sizeof(d2RAM))
                return d2RAM + (addr & 0x7FFFF);
            break;

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
    return *reinterpret_cast<const T *>(mem + (addr & (size - 1)));
}

template<class T, size_t size>
void MemoryBus::doWrite(uint8_t (&mem)[size], uint32_t addr, T data)
{
    *reinterpret_cast< T *>(mem + (addr & (size - 1))) = data;
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
