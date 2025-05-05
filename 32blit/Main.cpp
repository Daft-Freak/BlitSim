#include <cstdint>
#include <cstring>

#include "32blit.hpp"

#include "API.h"
#include "Metadata.h"
#include "RemoteFiles.h"

#include "ARMv7MCore.h"
#include "MemoryBus.h"

MemoryBus mem;
ARMv7MCore cpuCore(mem);

static bool fileLoaded = false;
static uint32_t homeDownTime = 0;
static bool menuOpen = false;

static BlitGameHeader blitHeader;

static bool parseBlit(blit::File &file)
{
    RawMetadata meta;
    if(!parseBlitMetadata(file, meta, metadataOffset))
        return false;

    blit::debugf("Loading \"%s\" %s by %s\n", meta.title, meta.version, meta.author);

    // write directly to start of qspi flash
    // (so we don't have to apply relocs)
    auto flashPtr = mem.mapAddress(0x90000000);

    // re-read headers
    uint8_t buf[8];
    file.read(0, 8, reinterpret_cast<char *>(buf));
    uint32_t numRelocs = buf[4] | buf[5] << 8 | buf[6] << 16 | buf[7] << 24;
    uint32_t relocsEnd = numRelocs * 4 + 8;

    file.read(relocsEnd, sizeof(blitHeader), reinterpret_cast<char *>(&blitHeader));

    metadataOffset -= relocsEnd;
    auto length = metadataOffset + meta.length + 10;

    // read data
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
    else if(menuOpen)
    {
        // fake menu overlay
        // TODO: palette mode?
        blit::screen.pen = {50, 50, 50, 200};
        blit::screen.clear();

        blit::screen.pen = {255, 255, 255};
        blit::screen.text("Menu is open", blit::minimal_font, blit::screen.clip, true, blit::TextAlign::center_center);
    }
}

void update(uint32_t time)
{
    // simulate a reset if home is held
    if(blit::buttons.pressed & blit::Button::HOME)
        homeDownTime = time;
    else if(homeDownTime && time - homeDownTime > 1000)
    {
        homeDownTime = 0;
        launchFile = "launcher.blit";
    }
    else if(blit::buttons.released & blit::Button::HOME)
    {
        homeDownTime = 0;
        menuOpen = !menuOpen;
    }

    // avoid catch-up logic (emulated tick will also do it)
    if(blit::now() - time >= 20)
        return;

    // sync inputs
    syncInput();

    if(menuOpen)
        return;

    // this isn't update, it's the layer above
    if(fileLoaded && !cpuCore.getPaused())
        cpuCore.runCall(blitHeader.tick, blit::now());

    if(!launchFile.empty())
    {
        if(launchFile[0] == '~')
            downloadRemoteFile(launchFile, "/tmp/", [](const std::string &path){openFile(path);});
        else
            fileLoaded = openFile(launchFile);
        launchFile.clear();
    }
}