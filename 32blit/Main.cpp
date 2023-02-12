#include <cstdint>
#include <cstring>

#include "32blit.hpp"
#include "engine/api_private.hpp"

#include "32blitAPI.h"
#include "ARMv6MCore.h"
#include "MemoryBus.h"

constexpr uint32_t blit_game_magic = 0x54494C42; // "BLIT"

struct BlitGameHeader {
  uint32_t magic;

  uint32_t render;
  uint32_t tick;
  uint32_t init;

  uint32_t end;
  uint32_t start;
};

// missing the "BLITMETA" header and size
struct RawMetadata {
  uint32_t crc32;
  char datetime[16];
  char title[25];
  char description[129];
  char version[17];
  char author[17];
};

static MemoryBus mem;
static ARMv6MCore cpuCore(mem);

static bool fileLoaded = false;

static BlitGameHeader blitHeader;

static bool parseBlit(blit::File &file)
{
    uint8_t buf[10];

    file.read(0, 8, reinterpret_cast<char *>(buf));

    if(memcmp(buf, "RELO", 4) != 0)
    {
        blit::debugf("Missing RELO header!\n");
        return false;
    }

    uint32_t numRelocs = buf[4] | buf[5] << 8 | buf[6] << 16 | buf[7] << 24;

    uint32_t relocsEnd = numRelocs * 4 + 8;

    // read header
    file.read(relocsEnd, sizeof(blitHeader), reinterpret_cast<char *>(&blitHeader));

    if(blitHeader.magic != blit_game_magic)
    {
        blit::debugf("Incorrect blit header magic!\n");
        return false;
    }

    uint32_t length = blitHeader.end - 0x90000000;

    // read metadata
    RawMetadata meta;

    auto offset = relocsEnd + length;

    file.read(offset, 10, reinterpret_cast<char *>(buf));

    if(memcmp(buf, "BLITMETA", 4) != 0)
    {
        blit::debugf("Incorrect metadata header!\n");
        return false;
    }

    uint16_t metadataLen = buf[8] | buf[9] << 8;

    length += metadataLen + 10;
    offset += 10;

    file.read(offset, sizeof(meta), reinterpret_cast<char *>(&meta));

    blit::debugf("Loading \"%s\" %s by %s\n", meta.title, meta.version, meta.author);

    // write directly to start of qspi flash
    // (so we don't have to apply relocs)
    auto flashPtr = mem.mapAddress(0x90000000);

    file.read(relocsEnd, length, reinterpret_cast<char *>(flashPtr));

    return true;
}

void apiCallback(int index, uint32_t *regs)
{
    using namespace blit;

    switch(index)
    {
        case 0: // set_screen_mode
        {
            auto screenPtr = 0x30000000; // in D2
            int cycles = 0;
            set_screen_mode(static_cast<ScreenMode>(regs[0]));
            
            mem.write<uint32_t>(screenPtr, 0x3000FC00, cycles, false); // .data = framebuffer

            mem.write<uint32_t>(screenPtr + 4, screen.bounds.w, cycles, false); // .bounds.w
            mem.write<uint32_t>(screenPtr + 8, screen.bounds.h, cycles, false); // .bounds.h

            mem.write<uint32_t>(screenPtr + 36, static_cast<int>(screen.format), cycles, false); // .format
            mem.write<uint32_t>(screenPtr + 48, 0, cycles, false); // .palette = null (TODO: palette mode)

            regs[0] = screenPtr; // return screen ptr
            break;
        }

        case 2: // now
            regs[0] = now();
            break;

        case 3: // random
            regs[0] = blit::random();
            break;

        case 5: // debug
        {
            auto message = reinterpret_cast<char *>(mem.mapAddress(regs[0]));
            api.debug(message);
            break;
        }

        default:
            debugf("blit API %i\n", index);
            break;
    }
}

static bool openFile(const std::string &filename)
{
    static blit::File blitFile;

    if(!blitFile.open(filename))
        return false;
    
    if(!parseBlit(blitFile))
    {
        blit::debugf("Failed to parse blit!\n");
        return false;
    }

    cpuCore.reset();
    cpuCore.setAPICallback(apiCallback);

    cpuCore.setSP(0x20020000); // end of DTCM
    cpuCore.runCall(blitHeader.init);

    return true;
}

void init()
{
    auto launchPath = blit::get_launch_path();

    if(launchPath)
        fileLoaded = openFile(launchPath);
    else if(blit::file_exists("launcher.blit"))
        fileLoaded = openFile("launcher.blit");

    if(!fileLoaded)
        return;

    cpuCore.reset();
    cpuCore.setAPICallback(apiCallback);

    cpuCore.setSP(0x20020000); // end of DTCM
    cpuCore.runCall(blitHeader.init);
}

void render(uint32_t time)
{
    if(!fileLoaded)
    {
        blit::screen.pen = {255, 0, 0};
        blit::screen.clear();
        return;
    }

    cpuCore.runCall(blitHeader.render, time);

    auto screenData = mem.mapAddress(0x3000FC00); // framebuffer
    memcpy(blit::screen.data, screenData, blit::screen.bounds.area() * blit::screen.pixel_stride);
}

void update(uint32_t time)
{
    if(!fileLoaded)
        return;

    // avoid catch-up logic (emulated tick will also do it)
    if(blit::now() - time >= 20)
        return;

    auto api = reinterpret_cast<blithw::API *>(mem.mapAddress(0xF800));
    api->buttons.state = blit::buttons.state;
    api->joystick.x = blit::joystick.x;
    api->joystick.y = blit::joystick.y;

    // this isn't update, it's the layer above
    cpuCore.runCall(blitHeader.tick, blit::now());
}