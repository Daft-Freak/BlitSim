#include <cstdint>
#include <cstring>

#include "32blit.hpp"

#include "API.h"
#include "RemoteFiles.h"

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

MemoryBus mem;
ARMv6MCore cpuCore(mem);

static bool fileLoaded = false;
static uint32_t homeDownTime = 0;

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

    metadataOffset = length;

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

    mem.setFirmwareRAMCallbacks(firmwareMemRead, firmwareMemWrite);

    for(int i = 0; i < CHANNEL_COUNT; i++)
        blit::channels[i] = blit::AudioChannel();

    cpuCore.reset();
    cpuCore.setAPICallback(apiCallback);

    cpuCore.setSP(0x20020000); // end of DTCM
    cpuCore.runCall(blitHeader.init);

#ifdef SCREEN_SPEED_HACKS
    hookScreenBlend();
#endif
    return true;
}

void init()
{
    auto launchPath = blit::get_launch_path();

    if(launchPath)
        fileLoaded = openFile(launchPath);
    else if(blit::file_exists("launcher.blit"))
        fileLoaded = openFile("launcher.blit");

    apiInit();
    initRemoteFiles();
}

void render(uint32_t time)
{
    if(!fileLoaded)
    {
        blit::screen.pen = {255, 0, 0};
        blit::screen.clear();
        return;
    }

    if(!cpuCore.getPaused())
        cpuCore.runCall(blitHeader.render, time);

    auto screenData = mem.mapAddress(0x3000FC00); // framebuffer
    memcpy(blit::screen.data, screenData, blit::screen.bounds.area() * blit::screen.pixel_stride);

    if(cpuCore.getPaused())
    {
        blit::screen.pen = {0, 0, 0, 200};
        blit::screen.clear();

        blit::screen.pen = {255, 255, 255};
        blit::screen.text("Waiting...", blit::minimal_font, blit::screen.clip, true, blit::TextAlign::center_center);
    }
}

void update(uint32_t time)
{
    if(!fileLoaded)
        return;


    // simulate a reset if home is held
    if(blit::buttons.pressed & blit::Button::HOME)
        homeDownTime = time;
    else if(homeDownTime && time - homeDownTime > 1000)
    {
        homeDownTime = 0;
        launchFile = "launcher.blit";
    }

    // avoid catch-up logic (emulated tick will also do it)
    if(blit::now() - time >= 20)
        return;

    // sync inputs
    syncInput();

    // this isn't update, it's the layer above
    if(!cpuCore.getPaused())
        cpuCore.runCall(blitHeader.tick, blit::now());

    if(!launchFile.empty())
    {
        if(launchFile[0] == '~')
            downloadRemoteFile(launchFile, "/tmp/", [](const std::string &path){openFile(path);});
        else
            openFile(launchFile);
        launchFile.clear();
    }
}