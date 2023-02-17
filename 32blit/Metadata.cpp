#include <cstring>

#include "engine/engine.hpp"

#include "Metadata.h"

bool parseBlitMetadata(blit::File &file, RawMetadata &meta, uint32_t &metadataOffset)
{
    uint8_t buf[8];

    file.read(0, 8, reinterpret_cast<char *>(buf));

    if(memcmp(buf, "RELO", 4) != 0)
    {
        blit::debugf("Missing RELO header!\n");
        return false;
    }

    uint32_t numRelocs = buf[4] | buf[5] << 8 | buf[6] << 16 | buf[7] << 24;

    uint32_t relocsEnd = numRelocs * 4 + 8;

    // read header
    BlitGameHeader blitHeader;
    file.read(relocsEnd, sizeof(blitHeader), reinterpret_cast<char *>(&blitHeader));

    if(blitHeader.magic != blit_game_magic)
    {
        blit::debugf("Incorrect blit header magic!\n");
        return false;
    }

    uint32_t length = blitHeader.end - 0x90000000;

    // read metadata
    auto offset = relocsEnd + length;

    file.read(offset, 8, reinterpret_cast<char *>(buf));

    if(memcmp(buf, "BLITMETA", 8) != 0)
    {
        blit::debugf("Incorrect metadata header!\n");
        return false;
    }

    metadataOffset = relocsEnd + length;

    file.read(offset + 8, sizeof(meta), reinterpret_cast<char *>(&meta));

    return true;
}