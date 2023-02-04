#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>

#include <SDL.h>

#include "ARMv6MCore.h"

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

static bool quit = false;

static MemoryBus mem;
static ARMv6MCore cpuCore(mem);

static std::ifstream blitFile;

static BlitGameHeader blitHeader;

static void pollEvents()
{
    SDL_Event event;
    while(SDL_PollEvent(&event))
    {
        switch(event.type)
        {
            case SDL_KEYDOWN:
            {
                /*auto it = picosystemKeyMap.find(event.key.keysym.sym);
                if(it != picosystemKeyMap.end())
                    buttonState |= it->second;*/
                break;
            }
            case SDL_KEYUP:
            {
                /*auto it = picosystemKeyMap.find(event.key.keysym.sym);
                if(it != picosystemKeyMap.end())
                    buttonState &= ~it->second;*/
                break;
            }
            case SDL_QUIT:
                quit = true;
                break;
        }
    }
}

static bool parseBlit(std::ifstream &file)
{
    uint8_t buf[10];

    file.read(reinterpret_cast<char *>(buf), 8);

    if(memcmp(buf, "RELO", 4) != 0)
    {
        std::cerr << "Missing RELO header!\n";
        return false;
    }

    uint32_t numRelocs = buf[4] | buf[5] << 8 | buf[6] << 16 | buf[7] << 24;

    uint32_t relocsEnd = numRelocs * 4 + 8;

    file.seekg(relocsEnd);

    // read header
    file.read(reinterpret_cast<char *>(&blitHeader), sizeof(blitHeader));

    if(blitHeader.magic != blit_game_magic)
    {
        std::cerr << "Incorrect blit header magic!\n";
        return false;
    }

    uint32_t length = blitHeader.end - 0x90000000;

    printf("%i %i\n", relocsEnd, length);

    // read metadata
    RawMetadata meta;

    file.seekg(relocsEnd + length);

    file.read(reinterpret_cast<char *>(buf), 10);

    if(memcmp(buf, "BLITMETA", 4) != 0)
    {
        std::cerr << "Incorrect metadata header!\n";
        return false;
    }

    uint16_t metadataLen = buf[8] | buf[9] << 8;

    length += metadataLen + 10;

    file.read(reinterpret_cast<char *>(&meta), sizeof(meta));

    std::cout << "Loading \"" << meta.title << "\" " << meta.version << " by " << meta.author << "\n";

    // write directly to start of qspi flash
    // (so we don't have to apply relocs)
    auto flashPtr = mem.mapAddress(0x90000000);

    file.seekg(relocsEnd);
    file.read(reinterpret_cast<char *>(flashPtr), length);

    return true;
}

int main(int argc, char *argv[])
{
    int screenWidth = 240;
    int screenHeight = 240;
    int screenScale = 5;

    std::string romFilename;

    int i = 1;

    for(; i < argc; i++)
    {
        std::string arg(argv[i]);

        if(arg == "--scale" && i + 1 < argc)
            screenScale = std::stoi(argv[++i]);
        else
            break;
    }

    if(i == argc)
        std::cout << "No ROM specified!\n";
    else
        romFilename = argv[i];

    // get base path
    std::string basePath;
    auto tmp = SDL_GetBasePath();
    if(tmp)
    {
        basePath = tmp;
        SDL_free(tmp);
    }

    if(!romFilename.empty())
    {
        blitFile.open(romFilename, std::ifstream::in | std::ifstream::binary);

        if(!blitFile || !parseBlit(blitFile))
        {
            std::cerr << "Failed to open .blit \"" << romFilename << "\"\n";
            return 1;
        }
    }

    // emu init
    
    cpuCore.reset();

    // SDL init
    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0)
    {
        std::cerr << "Failed to init SDL!\n";
        return 1;
    }

    auto window = SDL_CreateWindow("DaftBoySDL", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                   screenWidth * screenScale, screenHeight * screenScale,
                                   SDL_WINDOW_RESIZABLE);

    auto renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(renderer, screenWidth, screenHeight);
    SDL_RenderSetIntegerScale(renderer, SDL_TRUE);

    auto texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGR565, SDL_TEXTUREACCESS_STREAMING, screenWidth, screenHeight);

    auto lastTick = SDL_GetTicks();

    uint32_t cpuCycles = 0;

    cpuCore.setSP(0x20020000); // end of DTCM
    cpuCore.runCall(blitHeader.init);

    while(!quit)
    {
        pollEvents();

        auto now = SDL_GetTicks();

        auto elapsed = now - lastTick;

        // clamp if running behind
        if(elapsed > 30)
            elapsed = 30;

        //cpuCycles += cpuCore.run(elapsed);

        lastTick = now;

        // TODO: sync
        //SDL_UpdateTexture(texture, nullptr, screenData, screenWidth * 2);
        SDL_RenderClear(renderer);
        //SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    return 0;
}
