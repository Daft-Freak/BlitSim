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
std::string launchFile;

static uint32_t metadataOffset;
static BlitGameHeader blitHeader;

static std::map<uint32_t, void*> fileMap; // ptr size mismatch
static uint32_t nextFileId = 1;

static uint32_t waveChannelData[CHANNEL_COUNT][2]; // data/callback

// firmware ram in D2
const uint32_t screenPtr    = 0x30000000; // (52 bytes)
const uint32_t paletteAddr  = 0x30000040; // (1024 bytes)
const uint32_t savePathAddr = 0x30000440; // (1024 bytes)
const uint32_t channelsAddr = 0x30000840; // (1504 bytes (188 * 8))
const uint32_t tmpAddr      = 0x30000E40; // (1024+ bytes)
const uint32_t fbAddr = 0x3000FC00;

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

static void waveBufferCallback(blit::AudioChannel &channel)
{
    int ch = &channel - blit::channels;
    auto cb = waveChannelData[ch][1];

    auto chAddr = channelsAddr + ch * 188;

    cpuCore.runCallThread(cb, chAddr);
}

static void multiplayerMessageReceived(const uint8_t *data, uint16_t len)
{
    auto api = reinterpret_cast<blithw::API *>(mem.mapAddress(0xF800));

    if(api->message_received)
    {
        memcpy(mem.mapAddress(tmpAddr), data, len);
        cpuCore.runCall(api->message_received, tmpAddr, len);
    }
}

void apiCallback(int index, uint32_t *regs)
{
    using namespace blit;

    auto getStringData = [](uint32_t strPtr)
    {
        int c;
        auto strDataPtr = mem.read<uint32_t>(strPtr, c, false);
        auto strLen = mem.read<uint32_t>(strPtr + 4, c, false);
        auto strData = reinterpret_cast<const char *>(mem.mapAddress(strDataPtr));
        
        return std::string_view(strData, strLen);
    };

    switch(index)
    {
        case 0: // set_screen_mode
        {
            int cycles = 0;
            set_screen_mode(static_cast<ScreenMode>(regs[0]));
            
            mem.write<uint32_t>(screenPtr, fbAddr, cycles, false); // .data = framebuffer

            mem.write<uint32_t>(screenPtr + 4, screen.bounds.w, cycles, false); // .bounds.w
            mem.write<uint32_t>(screenPtr + 8, screen.bounds.h, cycles, false); // .bounds.h

            mem.write<uint32_t>(screenPtr + 36, static_cast<int>(screen.format), cycles, false); // .format
            mem.write<uint32_t>(screenPtr + 48, screen.palette ? paletteAddr : 0, cycles, false); // .palette

            regs[0] = screenPtr; // return screen ptr
            break;
        }

        case 1: // set_screen_palette
        {
            api.set_screen_palette(reinterpret_cast<Pen *>(mem.mapAddress(regs[0])), regs[1]);
            memcpy(mem.mapAddress(paletteAddr), mem.mapAddress(regs[0]), regs[1] * 4);
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

        case 6: // open_file
        {
            auto file = getStringData(regs[0]);
            int mode = regs[1];

            auto ret = api.open_file(std::string(file), mode);

            if(ret)
            {
                fileMap.emplace(nextFileId, ret);
                regs[0] = nextFileId++;
            }
            else
                regs[0] = 0;
            break;
        }

        case 7: // read_file
        {
            auto fh = fileMap.at(regs[0]);
            auto offset = regs[1];
            auto length = regs[2];
            auto buffer = reinterpret_cast<char *>(mem.mapAddress(regs[3]));

            regs[0] = api.read_file(fh, offset, length, buffer);
            break;
        }

        case 8: // write_file
        {
            auto fh = fileMap.at(regs[0]);
            auto offset = regs[1];
            auto length = regs[2];
            auto buffer = reinterpret_cast<char *>(mem.mapAddress(regs[3]));

            regs[0] = api.write_file(fh, offset, length, buffer);
            break;
        }

        case 9: // close_file
        {
            auto fh = fileMap.at(regs[0]);
            fileMap.erase(regs[0]);
            regs[0] = api.close_file(fh);
            break;
        }

        case 10: // get_file_length
        {
            auto fh = fileMap.at(regs[0]);
            regs[0] = api.get_file_length(fh);
            break;
        }

        case 11: // list_files
        {
            auto path = getStringData(regs[0]);
            auto callback = regs[1];

            int c;
            auto invoker = mem.read<uint32_t>(callback + 12, c, false);

            /*
            struct FileInfo {
                std::string name;
                int flags;
                uint32_t size;
            };
            */
            auto outFileInfo = reinterpret_cast<uint32_t *>(mem.mapAddress(tmpAddr)); // stack allocate?
            auto outFileName = mem.mapAddress(tmpAddr + 32);

            api.list_files(std::string(path), [&](FileInfo &info)
            {
                auto len = std::min(info.name.length(), size_t(1024 - 32));
                memcpy(outFileName, info.name.data(), len);

                outFileInfo[0] = tmpAddr + 32; // string data
                outFileInfo[1] = len; // string len

                outFileInfo[6] = info.flags; // flags = dir
                outFileInfo[7] = info.size; // size

                cpuCore.runCallLocked(invoker, callback, tmpAddr);
            });
        
            break;
        }
        
        case 12: // file_exists
        {
            regs[0] = api.file_exists(std::string(getStringData(regs[0])));
            break;
        }

        case 13: // directory_exists
        {
            regs[0] = api.directory_exists(std::string(getStringData(regs[0])));
            break;
        }

        case 14: // create_directory
        {
            regs[0] = api.create_directory(std::string(getStringData(regs[0])));
            break;
        }

        case 15: // rename_file
        {
            regs[0] = api.rename_file(std::string(getStringData(regs[0])), std::string(getStringData(regs[1])));
            break;
        }

        case 16: // remove_file
        {
            regs[0] = api.remove_file(std::string(getStringData(regs[0])));
            break;
        }

        case 17: // get_save_path
        {
            auto path = api.get_save_path();
            auto outPath = reinterpret_cast<char *>(mem.mapAddress(savePathAddr));

            // append the real title
            auto metadataAddr = 0x90000000 + metadataOffset + 10/*magic/len*/;
            auto metaTitle  = reinterpret_cast<char *>(mem.mapAddress(metadataAddr +  20));
            snprintf(reinterpret_cast<char *>(mem.mapAddress(savePathAddr)), 1024, "%s%s/", path, metaTitle);

            if(!directory_exists(outPath))
                create_directory(outPath);

            regs[0] = savePathAddr;
            break;
        }

        case 18: // is_storage_available
            regs[0] = 1;
            break;

        case 19: // enable_us_timer
            api.enable_us_timer();
            break;

        case 20: // get_us_timer
            regs[0] = api.get_us_timer();
            break;

        case 21: // get_max_us_timer
            regs[0] = api.get_max_us_timer();
            break;

        case 22: // decode_jpeg_buffer
        {
            auto retPtr = regs[0];
            auto ptr = mem.mapAddress(regs[1]);
            auto len = regs[2];
            auto alloc = regs[3];

            auto ret = decode_jpeg_buffer(ptr, len);

            // run alloc callback
            cpuCore.runCallLocked(alloc, ret.size.area() * 3);

            // copy data
            auto outPtr = mem.mapAddress(regs[0]);
            memcpy(outPtr, ret.data, ret.size.area() * 3);
            delete[] ret.data;

            int c;
            mem.write<uint32_t>(retPtr + 0, ret.size.w, c, false); // size.w
            mem.write<uint32_t>(retPtr + 4, ret.size.h, c, false); // size.h
            mem.write<uint32_t>(retPtr + 8, regs[0], c, false); // data
            break;
        }

        // 23 is decode_jpeg_file, but I don't think anything uses that?

        case 24: // launch
        {
            launchFile = reinterpret_cast<char *>(mem.mapAddress(regs[0]));
            break;
        }

        case 26: // get_type_handler_metadata
            regs[0] = 0;
            break;

        case 27: // get_launch_path
            regs[0] = 0;
            break;

        case 28: // is_multiplayer_connected
            regs[0] = api.is_multiplayer_connected();
            break;

        case 29: // set_multiplayer_enabled
            api.set_multiplayer_enabled(regs[0]);
            break;

        case 30: // send_message
            api.send_message(mem.mapAddress(regs[0]), regs[1]);
            break;

        case 33: // get_metadata
        {
            auto ptr = regs[0];
            int c;
            auto metadataAddr = 0x90000000 + metadataOffset + 10/*magic/len*/;

            mem.write<uint32_t>(ptr +  0, metadataAddr +  20, c, false); // title
            mem.write<uint32_t>(ptr +  4, metadataAddr + 191, c, false); // author
            mem.write<uint32_t>(ptr +  8, metadataAddr +  45, c, false); // description
            mem.write<uint32_t>(ptr + 12, metadataAddr + 174, c, false); // version

            // extended meta
            bool hasType = memcmp(mem.mapAddress(metadataAddr + sizeof(RawMetadata)), "BLITTYPE", 8) == 0;
            if(hasType)
            {
                auto typeMetadataAddr = metadataAddr + sizeof(RawMetadata) + 8;
                mem.write<uint32_t>(ptr + 16, typeMetadataAddr + 17, c, false); // url
                mem.write<uint32_t>(ptr + 20, typeMetadataAddr, c, false); // category
            }
            else
            {
                mem.write<uint32_t>(ptr + 16, 0, c, false); // url
                mem.write<uint32_t>(ptr + 20, 0, c, false); // category
            }

            break;
        }

        case 34: // set_screen_mode_format
        {
            auto inTemp = reinterpret_cast<blithw::SurfaceTemplate *>(mem.mapAddress(regs[1]));

            SurfaceTemplate temp{nullptr, {inTemp->bounds.w, inTemp->bounds.h}, static_cast<PixelFormat>(inTemp->format), nullptr};
            regs[0] = api.set_screen_mode_format(static_cast<ScreenMode>(regs[0]), temp);

            // sync screen
            screen = Surface(temp.data, temp.format, temp.bounds);
            screen.palette = temp.palette;

            // copy template out
            inTemp->data = fbAddr;
            inTemp->bounds = {temp.bounds.w, temp.bounds.h};
            inTemp->format = static_cast<blithw::PixelFormat>(temp.format);
            if(temp.palette)
                inTemp->palette = paletteAddr;

            break;
        }

        default:
            debugf("blit API %i\n", index);
            break;
    }
}

static uint32_t firmwareMemRead(uint32_t addr, uint32_t val, int width)
{
    if(addr >= channelsAddr && addr < channelsAddr + 188 * 8)
    {
        int ch = (addr - channelsAddr) / 188;
        unsigned int chOff = (addr - channelsAddr) % 188;
        
        if(chOff < offsetof(blithw::AudioChannel, user_data))
        {
            // everything matches until the user callback ptrs
            auto chanData = reinterpret_cast<uint8_t *>(blit::channels + ch);

            val = 0;
            for(int i = 0; i < width; i++)
                val |= chanData[chOff++] << (i * 8);
        }
        else if(chOff == 180) // user_data
            val = waveChannelData[ch][0]; // assume 32-bit read
        else if(chOff == 184) // wave_buffer_callback
            val = waveChannelData[ch][1]; // assume 32-bit read
        else
            blit::debugf("audio r %08X (%i)\n", addr, width);
    }
    return val;
}

static uint32_t firmwareMemWrite(uint32_t addr, uint32_t val, int width)
{
    if(addr >= channelsAddr && addr < channelsAddr + 188 * 8)
    {
        int ch = (addr - channelsAddr) / 188;
        unsigned int chOff = (addr - channelsAddr) % 188;
        
        if(chOff < offsetof(blithw::AudioChannel, user_data))
        {
            // everything matches until the user callback ptrs
            auto chanData = reinterpret_cast<uint8_t *>(blit::channels + ch);

            for(int i = 0; i < width; i++, val >>= 8)
                chanData[chOff++] = val;
        }
        else if(chOff == 180) // user_data
            waveChannelData[ch][0] = val; // assume 32-bit write
        else if(chOff == 184) // wave_buffer_callback
        {
            waveChannelData[ch][1] = val; // assume 32-bit write
            blit::channels[ch].wave_buffer_callback = val ? waveBufferCallback : nullptr;
        }
        else
            blit::debugf("audio w %08X(ch %i off %i) = %08X (%i)\n", addr, ch, chOff, val, width);
    }
    return val;
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

    blit::message_received = multiplayerMessageReceived;

    for(int i = 0; i < CHANNEL_COUNT; i++)
        blit::channels[i].user_data = waveChannelData[i];
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

    if(!launchFile.empty())
    {
        openFile(launchFile);
        launchFile.clear();
    }
}