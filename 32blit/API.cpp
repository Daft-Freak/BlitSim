#include <cstring>
#include <map>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#endif

#include "32blit.hpp"
#include "engine/api_private.hpp"

#include "API.h"

#include "Metadata.h"
#include "RemoteFiles.h"

#include "32blitAPI.h"
#include "ARMv7MCore.h"
#include "MemoryBus.h"

extern ARMv7MCore cpuCore;
extern MemoryBus mem;

std::string launchFile;
static std::string launchPath;

uint32_t metadataOffset;

struct FileData
{
    void *fh; // real handle
    int mode;
    bool isRemote;
};

static std::map<uint32_t, FileData> fileMap;
static uint32_t nextFileId = 1;

static uint32_t waveChannelData[CHANNEL_COUNT][2]; // data/callback

static bool fileInTemp = false;

struct TypeHandler
{
    std::string filename;
    uint32_t metadataOffset;
};

static std::map<std::string, TypeHandler> typeHandlers;

// screen speed hacks
static uint32_t gameScreenPtr = 0;
static uint32_t origPBF, origBBF;
static blit::Surface emuScreen(nullptr, blit::PixelFormat::P, {0, 0});

// firmware ram in D2
const uint32_t fwScreenAddr = 0x30000000; // (52 bytes)
const uint32_t paletteAddr  = 0x30000040; // (1024 bytes)
const uint32_t savePathAddr = 0x30000440; // (1024 bytes)
const uint32_t channelsAddr = 0x30000840; // (1504 bytes (188 * 8))
const uint32_t tmpAddr      = 0x30000E40; // (1024+ bytes)
const uint32_t fbAddr = 0x3000FC00;

// temp area in flash
const unsigned int flashTmpSize = 4 * 1024 * 1024;
const uint32_t flashTmpAddr = 0x90000000 + (32 * 1024 * 1024) - flashTmpSize;

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
void apiInit()
{
    blit::message_received = multiplayerMessageReceived;

    for(int i = 0; i < CHANNEL_COUNT; i++)
        blit::channels[i].user_data = waveChannelData[i];

    // write the metadata for the handlers near the end of flash
    uint32_t endAddr = flashTmpAddr; 

    // list top-level blits
    auto blits = blit::list_files("/", [](const blit::FileInfo &f){
        auto dot = f.name.find_last_of('.');

        if(dot == std::string::npos)
            return false;

        return f.name.compare(dot, f.name.length() - dot, ".blit") == 0;
    });

    for(auto &file : blits)
    {
        blit::File f(file.name);

        RawMetadata meta;
        uint32_t metaOffset;
        if(!parseBlitMetadata(f, meta, metaOffset))
            continue;
        
        // check for type metadata
        char buf[8];
        uint32_t offset = metaOffset + sizeof(RawMetadata) + 8;
        f.read(metaOffset + sizeof(RawMetadata) + 8, 8, buf);

        if(memcmp(buf, "BLITTYPE", 8) != 0)
            continue;

        RawTypeMetadata typeMeta;
        offset += 8;
        f.read(offset, sizeof(typeMeta), reinterpret_cast<char *>(&typeMeta));
        offset += sizeof(typeMeta);

        if(!typeMeta.num_filetypes)
            continue;

        // copy metadata
        auto metaAddr = endAddr - meta.length;
        f.read(metaOffset, meta.length, reinterpret_cast<char *>(mem.mapAddress(metaAddr)));
        endAddr = metaAddr;

        // check for filetypes
        for(int i = 0; i < typeMeta.num_filetypes; i++)
        {
            f.read(offset, 5, buf);
            offset += 5;
            blit::debugf("Setting %s as handler for %s\n", meta.title, buf);
            typeHandlers.emplace(buf, TypeHandler{file.name, metaAddr});
        }
    }
}

void apiCallback(int index, uint32_t *regs)
{
    using namespace blit;

    auto getStringData = [](uint32_t strPtr)
    {
        auto strDataPtr = mem.read<uint32_t>(strPtr);
        auto strLen = mem.read<uint32_t>(strPtr + 4);
        auto strData = reinterpret_cast<const char *>(mem.mapAddress(strDataPtr));
        
        return std::string_view(strData, strLen);
    };

    switch(index)
    {
        case 0: // set_screen_mode
        {
            set_screen_mode(static_cast<ScreenMode>(regs[0]));
            
            mem.write<uint32_t>(fwScreenAddr, fbAddr); // .data = framebuffer

            mem.write<uint32_t>(fwScreenAddr + 4, screen.bounds.w); // .bounds.w
            mem.write<uint32_t>(fwScreenAddr + 8, screen.bounds.h); // .bounds.h

            mem.write<uint32_t>(fwScreenAddr + 36, static_cast<int>(screen.format)); // .format
            mem.write<uint32_t>(fwScreenAddr + 48, screen.palette ? paletteAddr : 0); // .palette

            regs[0] = fwScreenAddr; // return screen ptr
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

            void *ret;
            bool isRemote = false;

            if(file[0] == '~' && mode == OpenMode::read)
            {
                ret = openRemoteFile(std::string(file));
                isRemote = true;
            }
            else
                ret = api.open_file(std::string(file), mode);

            if(ret)
            {
                fileMap.emplace(nextFileId, FileData{ret, mode, isRemote});
                regs[0] = nextFileId++;
            }
            else
                regs[0] = 0;
            break;
        }

        case 7: // read_file
        {
            auto &file = fileMap.at(regs[0]);
            auto offset = regs[1];
            auto length = regs[2];
            auto buffer = reinterpret_cast<char *>(mem.mapAddress(regs[3]));

            if(file.isRemote)
                regs[0] = readRemoteFile(file.fh, offset, length, buffer);
            else
                regs[0] = api.read_file(file.fh, offset, length, buffer);
            break;
        }

        case 8: // write_file
        {
            auto &file = fileMap.at(regs[0]);
            auto offset = regs[1];
            auto length = regs[2];
            auto buffer = reinterpret_cast<char *>(mem.mapAddress(regs[3]));

            regs[0] = api.write_file(file.fh, offset, length, buffer);
            break;
        }

        case 9: // close_file
        {
            auto &file = fileMap.at(regs[0]);

            if(file.isRemote)
                regs[0] = closeRemoteFile(file.fh);
            else
            {
                regs[0] = api.close_file(file.fh);
#ifdef __EMSCRIPTEN__
                if(file.mode & OpenMode::write)
                {
                    // TODO: we only have persistance on /libsdl
                    EM_ASM(
                        FS.syncfs(function(err) {});
                    );
                }
#endif
            }
            fileMap.erase(regs[0]);
            break;
        }

        case 10: // get_file_length
        {
            auto fh = fileMap.at(regs[0]).fh;
            regs[0] = api.get_file_length(fh);
            break;
        }

        case 11: // list_files
        {
            auto path = getStringData(regs[0]);
            auto callback = regs[1];

            auto invoker = mem.read<uint32_t>(callback + 12);

            /*
            struct FileInfo {
                std::string name;
                int flags;
                uint32_t size;
            };
            */
            auto outFileInfo = reinterpret_cast<uint32_t *>(mem.mapAddress(tmpAddr)); // stack allocate?
            auto outFileName = mem.mapAddress(tmpAddr + 32);

            auto cb = [&](FileInfo &info)
            {
                auto len = std::min(info.name.length(), size_t(1024 - 32));
                memcpy(outFileName, info.name.data(), len);

                outFileInfo[0] = tmpAddr + 32; // string data
                outFileInfo[1] = len; // string len

                outFileInfo[6] = info.flags; // flags = dir
                outFileInfo[7] = info.size; // size

                cpuCore.runCallLocked(invoker, callback, tmpAddr);
            };

            api.list_files(std::string(path), cb);

#ifdef __EMSCRIPTEN__
            if(path == "/")
            {
                FileInfo i{"~cupboard", FileFlags::directory, 0};
                cb(i);
            }
#endif
            if(path[0] == '~')
                listRemoteFiles(cb);

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
            auto metaTitle = reinterpret_cast<char *>(mem.mapAddress(metadataAddr +  20));
            snprintf(outPath, 1024, "%s%s/", path, metaTitle);

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

            mem.write<uint32_t>(retPtr + 0, ret.size.w); // size.w
            mem.write<uint32_t>(retPtr + 4, ret.size.h); // size.h
            mem.write<uint32_t>(retPtr + 8, regs[0]); // data
            break;
        }

        // 23 is decode_jpeg_file, but I don't think anything uses that?

        case 24: // launch
        {
            launchFile = reinterpret_cast<char *>(mem.mapAddress(regs[0]));
            launchPath = "";

            auto ext = std::string(launchFile.substr(launchFile.find_last_of('.') + 1));
            for(auto &c : ext)
                c = tolower(c);

            if(ext != "blit")
            {
                auto it = typeHandlers.find(ext);
                if(it != typeHandlers.end())
                {
                    launchPath = launchFile;
                    launchFile = it->second.filename;
                }
            }

            break;
        }

        case 26: // get_type_handler_metadata
        {
            auto filetype = reinterpret_cast<char *>(mem.mapAddress(regs[0]));
            auto it = typeHandlers.find(filetype);

            if(it != typeHandlers.end())
                regs[0] = it->second.metadataOffset;
            else
                regs[0] = 0;

            break;
        }

        case 27: // get_launch_path
        {
            if(!launchPath.empty())
            {
                auto addr = 0x38800000; // backup ram (right region, wrong offset)
                strncpy(reinterpret_cast<char *>(mem.mapAddress(addr)), launchPath.c_str(), 256);
                regs[0] = addr;
            }
            else
                regs[0] = 0;

            break;
        }

        case 28: // is_multiplayer_connected
            regs[0] = api.is_multiplayer_connected();
            break;

        case 29: // set_multiplayer_enabled
            api.set_multiplayer_enabled(regs[0]);
            break;

        case 30: // send_message
            api.send_message(mem.mapAddress(regs[0]), regs[1]);
            break;

        case 31: // flash_to_tmp
        {
            auto filename = getStringData(regs[0]);
            auto sizePtr = regs[1];

            regs[0] = 0;

            File f{std::string(filename)};
            auto fileLen = f.get_length();

            if (fileLen <= flashTmpSize && !fileInTemp)
            {
                f.read(0, fileLen, reinterpret_cast<char *>(mem.mapAddress(flashTmpAddr)));

                mem.write<uint32_t>(sizePtr, fileLen);

                regs[0] = flashTmpAddr;
                fileInTemp = true;
            }
            else
                regs[0] = 0;

            break;
        }

        case 32: // tmp_file_closed
        {
            fileInTemp = false;
            break;
        }

        case 33: // get_metadata
        {
            auto ptr = regs[0];
            auto metadataAddr = 0x90000000 + metadataOffset + 10/*magic/len*/;
            int metadataSize = 208;

            mem.write<uint32_t>(ptr +  0, metadataAddr +  20); // title
            mem.write<uint32_t>(ptr +  4, metadataAddr + 191); // author
            mem.write<uint32_t>(ptr +  8, metadataAddr +  45); // description
            mem.write<uint32_t>(ptr + 12, metadataAddr + 174); // version

            // extended meta
            bool hasType = memcmp(mem.mapAddress(metadataAddr + metadataSize), "BLITTYPE", 8) == 0;
            if(hasType)
            {
                auto typeMetadataAddr = metadataAddr + metadataSize + 8;
                mem.write<uint32_t>(ptr + 16, typeMetadataAddr + 17); // url
                mem.write<uint32_t>(ptr + 20, typeMetadataAddr); // category
            }
            else
            {
                mem.write<uint32_t>(ptr + 16, 0); // url
                mem.write<uint32_t>(ptr + 20, 0); // category
            }

            break;
        }

        case 34: // set_screen_mode_format
        {
            auto inTemp = reinterpret_cast<blithw::SurfaceTemplate *>(mem.mapAddress(regs[1]));

            [[maybe_unused]]
            auto oldFormat = screen.format;

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

#ifdef SCREEN_SPEED_HACKS
            if(gameScreenPtr)
            {
                // can't handle format changes as we don't know what the new blend funcs are
                // (this wouldn't be a problem if we didn't have the fallback for masks)
                if(temp.format == oldFormat)
                {
                    inTemp->pen_blend = 0x08BA1001;
                    inTemp->blit_blend = 0x08BA1003;
                }
                else
                    blit::debugf("screen format changed, losing speed hacks\n");
            }
#endif

            break;
        }

        case 35: // list_installed_games
        {
            auto callback = regs[0];
            auto invoker = mem.read<uint32_t>(callback + 12);

            auto outPtr = reinterpret_cast<uint32_t *>(mem.mapAddress(tmpAddr)); // stack allocate?

            // get size
            auto metadataAddr = 0x90000000 + metadataOffset;
            auto metadataSize = *reinterpret_cast<uint16_t *>(mem.mapAddress(metadataAddr + 8));

            // claim only current blit installed
            outPtr[0] = 0x90000000; // ptr
            outPtr[1] = 0; // block
            outPtr[2] =  metadataOffset + metadataSize + 10; // size

            regs[2] = tmpAddr + 4;
            regs[3] = tmpAddr + 8;
            cpuCore.runCallLocked(invoker, callback, tmpAddr);
            break;
        }

        case 36: // can_launch
        {
            std::string_view path = reinterpret_cast<char *>(mem.mapAddress(regs[0]));

            auto ext = std::string(path.substr(path.find_last_of('.') + 1));
            for(auto &c : ext)
                c = tolower(c);

            regs[0] = int(CanLaunchResult::UnknownType);

            // assume flashed is compatible (it's the thing that's running...)
            if(path.substr(0, 7) == "flash:/")
                regs[0] = int(CanLaunchResult::Success);
            else if(ext == "blit")
            {
                // TODO: check header
                regs[0] = int(CanLaunchResult::Success);
            }
            else
            {
                if(typeHandlers.count(ext))
                    regs[0] = int(CanLaunchResult::Success);
            }

            break;
        }

        case 2048: // patched screen.pbf
        {
            auto maskPtr = mem.read<uint32_t>(gameScreenPtr + 44);
            Surface maskTmp(nullptr, PixelFormat::M, {0, 0});

            // wrap mask
            if(maskPtr)
            {
                maskTmp.data = mem.mapAddress(mem.read<uint32_t>(maskPtr));
                maskTmp.bounds.w = mem.read<uint32_t>(maskPtr + 4);
                maskTmp.bounds.h = mem.read<uint32_t>(maskPtr + 8);
                emuScreen.mask = &maskTmp;
            }

            auto pen = mem.read<uint32_t>(regs[0]);
            emuScreen.alpha = mem.read<uint8_t>(gameScreenPtr + 28);
            emuScreen.pbf(reinterpret_cast<Pen *>(&pen), &emuScreen, regs[2], regs[3]);
            emuScreen.mask = nullptr;

            break;
        }
        case 2049: // patched screen.bbf
        {
            auto maskPtr = mem.read<uint32_t>(gameScreenPtr + 44);
            Surface maskTmp(nullptr, PixelFormat::M, {0, 0});

            // wrap mask
            if(maskPtr)
            {
                maskTmp.data = mem.mapAddress(mem.read<uint32_t>(maskPtr));
                maskTmp.bounds.w = mem.read<uint32_t>(maskPtr + 4);
                maskTmp.bounds.h = mem.read<uint32_t>(maskPtr + 8);
                emuScreen.mask = &maskTmp;
            }

            auto stack = reinterpret_cast<uint32_t *>(mem.mapAddress(regs[13]));
            auto cnt = stack[0];
            auto srcStep = stack[1];

            // get src surface
            auto srcPtr = reinterpret_cast<uint32_t *>(mem.mapAddress(regs[0]));
            auto srcData = mem.mapAddress(srcPtr[0]);
            Size srcBounds(srcPtr[1], srcPtr[2]);
            auto srcFormat = static_cast<PixelFormat>(srcPtr[9]);

            Surface src(srcData, srcFormat, srcBounds);

            if(srcFormat == PixelFormat::P)
                src.palette = reinterpret_cast<Pen *>(mem.mapAddress(srcPtr[12]));

            emuScreen.alpha = mem.read<uint8_t>(gameScreenPtr + 28);
            emuScreen.bbf(&src, regs[1], &emuScreen, regs[3], cnt, srcStep);
        
            emuScreen.mask = nullptr;
            break;
        }

        default:
            debugf("blit API %i\n", index);
            break;
    }
}


uint32_t firmwareMemRead(uint32_t addr, uint32_t val, int width)
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

uint32_t firmwareMemWrite(uint32_t addr, uint32_t val, int width)
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

static bool tryScreenHook(uint32_t addr)
{
    auto data = mem.read<uint32_t>(addr);

    // first word should be framebuffer address
    if(data != fbAddr)
        return false;

    // check bounds/format to be sure
    auto w = mem.read<uint32_t>(addr + 4);
    auto h = mem.read<uint32_t>(addr + 8);
    auto fmt = mem.read<uint32_t>(addr + 36);

    if(blit::screen.bounds == blit::Size(w, h) && blit::screen.format == static_cast<blit::PixelFormat>(fmt))
    {
        // TODO: need to re-patch whenever screen mode changes

        auto curPBF = mem.read<uint32_t>(addr + 60);
        auto curBBF = mem.read<uint32_t>(addr + 64);

        // make sure we haven't already patched
        if(curPBF != 0x08BA1001 && curBBF != 0x08BA1003)
        {
            // extra check, make sure the old pointers point to flash (or the patched addr to handle launches)
            if((curPBF & 0xFE000000) != 0x90000000 || (curBBF & 0xFE000000) != 0x90000000)
                return false;

            origPBF = curPBF;
            origBBF = curBBF;
        }

        gameScreenPtr = addr;
        blit::debugf("patching screen at %x\n", addr);

        mem.write<uint32_t>(addr + 60, 0x08BA1001); // overwrite pbf
        mem.write<uint32_t>(addr + 64, 0x08BA1003); // overwrite bbf

        emuScreen = blit::Surface(mem.mapAddress(fbAddr), blit::screen.format, blit::screen.bounds);
        emuScreen.palette = reinterpret_cast<blit::Pen *>(mem.mapAddress(paletteAddr));
        return true;
    }

    return false;
}

void hookScreenBlend()
{
    gameScreenPtr = 0;

    // attempt to find screen
    // there's a reference to this in .data before SDK v 0.3.3
    auto ram = mem.mapAddress(0x24000000); //.bss
    // assume 4 byte alignment
    // also hope it's early (seems more likely with the new SDK)
    for(int off = 0; off < 362 * 1024; off += 4)
    {
        // look for the right region
        if(ram[off + 3] != fbAddr >> 24)
            continue;

        if(tryScreenHook(0x24000000 + off))
            break;
    }
}

void syncInput()
{
    auto api = reinterpret_cast<blithw::API *>(mem.mapAddress(0xF800));
    api->buttons.state = blit::buttons.state;
    api->joystick.x = blit::joystick.x;
    api->joystick.y = blit::joystick.y;
}