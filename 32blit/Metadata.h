#pragma once
#include <cstdint>

#include "engine/file.hpp"

constexpr uint32_t blit_game_magic = 0x54494C42; // "BLIT"

struct BlitGameHeader {
  uint32_t magic;

  uint32_t render;
  uint32_t tick;
  uint32_t init;

  uint32_t end;
  uint32_t start;
};

// missing the "BLITMETA" header
#pragma pack(push, 1)
struct RawMetadata
{
    uint16_t length;
    uint32_t crc32;
    char datetime[16];
    char title[25];
    char description[129];
    char version[17];
    char author[17];
};
#pragma pack(pop)

bool parseBlitMetadata(blit::File &file, RawMetadata &meta, uint32_t &metadataOffset);