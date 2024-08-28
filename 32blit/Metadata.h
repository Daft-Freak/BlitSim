#pragma once
#include <cstdint>

#include "engine/file.hpp"

constexpr uint32_t blit_game_magic = 0x54494C42; // "BLIT"

enum class BlitDevice : uint8_t {
  STM32H7_32BlitOld = 0, // 32blit hw, old header
  STM32H7_32Blit = 1, // 32blit hw
  RP2040 = 2, // any RP2040-based device
};

struct BlitGameHeader {
  uint32_t magic;

  uint32_t render;
  uint32_t tick;
  uint32_t init;

  uint32_t end;

  BlitDevice device_id;
  uint8_t unused[3];

  // if device_id != 0
  uint16_t api_version_major;
  uint16_t api_version_minor;
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

// "BLITTYPE"
struct RawTypeMetadata
{
    char category[17];
    char url[129];
    uint8_t num_filetypes;
    char filetypes[][5];
};

bool parseBlitMetadata(blit::File &file, RawMetadata &meta, uint32_t &metadataOffset);