#include "32blit.hpp"

#include "RemoteFiles.h"

#include "ARMv6MCore.h"

#ifdef __EMSCRIPTEN__

#include <emscripten/fetch.h>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

extern ARMv6MCore cpuCore;

struct RemoteFileInfo
{
    std::string filename;
    uint32_t size = 0;

    uint32_t metadataOffset = 0;
    uint8_t *metadata = nullptr;
};

static const char *cupboardFirebaseUrl = "https://firebasestorage.googleapis.com/v0/b/daft-games.appspot.com/o";

static std::map<std::string, RemoteFileInfo> cupboardFiles;
static std::string cupboardVersionDir;

static void fetchSuccess(emscripten_fetch_t *fetch)
{
    auto fun = reinterpret_cast<std::function<void(emscripten_fetch_t *)> *>(fetch->userData);

    (*fun)(fetch);

    delete fun;
    emscripten_fetch_close(fetch);
}

static void fetch(const char *url, std::function<void(emscripten_fetch_t *)> onSuccess, const char *const *headers = nullptr)
{
    auto newFunc = new std::function(onSuccess);

    emscripten_fetch_attr_t attr;
    emscripten_fetch_attr_init(&attr);
    strcpy(attr.requestMethod, "GET");
    attr.attributes = EMSCRIPTEN_FETCH_LOAD_TO_MEMORY;
    attr.onsuccess = fetchSuccess;
    attr.userData = newFunc;
    attr.requestHeaders = headers;

    emscripten_fetch(&attr, url);
}

static void fetchRemoteBlitList()
{
    // list versions
    char buf[200];
    snprintf(buf, sizeof(buf), "%s?prefix=blit/&delimiter=/", cupboardFirebaseUrl);
    fetch(buf, [](emscripten_fetch_t *fetchRes)
    {
        auto j = json::parse(std::string(fetchRes->data, fetchRes->numBytes));

        std::string maxVerStr;
        int maxVer = 0;

        // get the latest version
        for(auto item : j["prefixes"])
        {
            auto verStr = item.get<std::string>();
            int ver = 0;

            // parse vX.Y.Z/ -> XXYYZZ
            auto start = verStr.find("v") + 1;
            auto end = verStr.find(".", start);
            ver = std::stoi(verStr.substr(start, end - start)) * 10000;

            start = end + 1;
            end = verStr.find(".", start);
            ver += std::stoi(verStr.substr(start, end - start)) * 100;

            start = end + 1;
            end = verStr.find("/", start);
            ver += std::stoi(verStr.substr(start, end - start));

            if(ver > maxVer)
            {
                maxVer = ver;
                maxVerStr = verStr;
            }
        }

        blit::debugf("Latest version on cupboard.daftgames.net: %s\n", maxVerStr.c_str());

        // encode the slashes
        cupboardVersionDir = maxVerStr;
        auto pos = cupboardVersionDir.find("/");
        while(pos != std::string::npos)
        {
            cupboardVersionDir.replace(pos, 1, "%2F");
            pos = cupboardVersionDir.find("/");
        }

        // fetch the list file info
        char buf[200];
        snprintf(buf, sizeof(buf), "%s/%slist.json", cupboardFirebaseUrl, cupboardVersionDir.c_str());
        fetch(buf, [](emscripten_fetch_t *fetchRes)
        {
            auto token = json::parse(std::string(fetchRes->data, fetchRes->numBytes))["downloadTokens"].get<std::string>();

            // fetch the actual data
            char buf[200];
            snprintf(buf, sizeof(buf), "%s/%slist.json?alt=media&token=%s", cupboardFirebaseUrl, cupboardVersionDir.c_str(), token.c_str());
            fetch(buf, [](emscripten_fetch_t *fetchRes)
            {
                auto j = json::parse(std::string(fetchRes->data, fetchRes->numBytes));
                for(auto item : j)
                {
                    auto blit = item["blit"].get<std::string>();
                    auto metadataOffset = item["metadataOffset"].get<uint32_t>();

                    RemoteFileInfo info;
                    info.filename = blit;
                    info.metadataOffset = metadataOffset;

                    cupboardFiles.emplace(blit, info);
                }
            });
        });
    });
}

static void fetchRemoteBlitMetadata(RemoteFileInfo &info, std::function<void()> onDone)
{
    // fetch the file info
    char buf[200];
    snprintf(buf, sizeof(buf), "%s/%s%s", cupboardFirebaseUrl, cupboardVersionDir.c_str(), info.filename.c_str());
    fetch(buf, [&info, onDone](emscripten_fetch_t *fetchRes)
    {
        auto token = json::parse(std::string(fetchRes->data, fetchRes->numBytes))["downloadTokens"].get<std::string>();

        char rangeHeader[20];
        snprintf(rangeHeader, sizeof(rangeHeader), "bytes=%u-", info.metadataOffset);
        const char* headers[] = {"Range", rangeHeader, nullptr};

        // fetch the actual data
        char buf[200];
        snprintf(buf, sizeof(buf), "%s/%s%s?alt=media&token=%s", cupboardFirebaseUrl, cupboardVersionDir.c_str(), info.filename.c_str(), token.c_str());
        fetch(buf, [&info, onDone](emscripten_fetch_t *fetchRes)
        {
            info.metadata = new uint8_t[fetchRes->numBytes];
            memcpy(info.metadata, fetchRes->data, fetchRes->numBytes);

            onDone();
        }, headers);
    });
}

void initRemoteFiles()
{
    fetchRemoteBlitList();
}

void listRemoteFiles(std::function<void(blit::FileInfo &)> callback)
{
    for(auto &f : cupboardFiles)
    {
        blit::FileInfo info;
        info.name = f.first;
        info.flags = 0;
        if(f.second.metadata)
        {
            uint16_t metadataSize = f.second.metadata[8] | f.second.metadata[9] << 8;
            info.size  = f.second.metadataOffset + 10 + metadataSize;
        }
        else
            info.size = f.second.metadataOffset + 0x10000; // guess

        callback(info);
    }
}

void *openRemoteFile(const std::string &file)
{
    auto slash = file.find("/");

    if(file.compare(0, slash, "~cupboard") != 0)
        return nullptr;

    auto it = cupboardFiles.find(file.substr(slash + 1));

    if(it == cupboardFiles.end())
        return nullptr;

    return &it->second;
}

int32_t readRemoteFile(void *fh, uint32_t offset, uint32_t length, char* buffer)
{
    auto info = reinterpret_cast<RemoteFileInfo *>(fh);
    if(offset == 0 && length == 24)
    {
        // fake relocs header
        buffer[0] = 'R';
        buffer[1] = 'E';
        buffer[2] = 'L';
        buffer[3] = 'O';

        // launcher ignores the rest
        memset(buffer + 4, 0, length - 4);
    }
    else if(offset == 4 && length == 4)
        memset(buffer, 0, length); // 0 relocs
    else if(offset == 8 && length == 24)
    {
        // fake header
        buffer[0] = 'B';
        buffer[1] = 'L';
        buffer[2] = 'I';
        buffer[3] = 'T';

        buffer[ 4] = buffer[ 5] = buffer[ 6] = buffer[ 7] = 0; // render
        buffer[ 8] = buffer[ 9] = buffer[10] = buffer[11] = 0; // tick
        buffer[12] = buffer[13] = buffer[14] = buffer[15] = 0; // init

        // end
        auto len = info->metadataOffset - 8;
        buffer[16] = len;
        buffer[17] = len >> 8;
        buffer[18] = len >> 16;
        buffer[19] = (len >> 24) | 0x90;

        buffer[20] = buffer[21] = buffer[22] = buffer[23] = 0; // start
    }
    else if(offset >= info->metadataOffset)
    {
        if(info->metadata)
            memcpy(buffer, info->metadata + (offset - info->metadataOffset), length);
        else
        {
            // stop the emulator until we have the data
            cpuCore.pause();

            fetchRemoteBlitMetadata(*info, [info, buffer, offset, length]()
            {
                memcpy(buffer, info->metadata + (offset - info->metadataOffset), length);
                cpuCore.resume();
            });
        }
    }
    else
        blit::debugf("remote read %p(%x) %u %u\n", fh, info->metadataOffset, offset, length);

    return length;
}

int32_t closeRemoteFile(void *fh)
{
    return 0;
}

void downloadRemoteFile(const std::string &file, const std::string &destDir, std::function<void(const std::string &)> onDone)
{
    auto slash = file.find("/");

    if(file.compare(0, slash, "~cupboard") != 0)
        return;

    auto it = cupboardFiles.find(file.substr(slash + 1));

    if(it == cupboardFiles.end())
        return;

    std::string destPath = destDir + file.substr(slash + 1);

    // fetch the file info
    char buf[200];
    snprintf(buf, sizeof(buf), "%s/%s%s", cupboardFirebaseUrl, cupboardVersionDir.c_str(), it->first.c_str());
    fetch(buf, [it, destPath, onDone](emscripten_fetch_t *fetchRes)
    {
        auto token = json::parse(std::string(fetchRes->data, fetchRes->numBytes))["downloadTokens"].get<std::string>();

        // fetch the actual data
        char buf[200];
        snprintf(buf, sizeof(buf), "%s/%s%s?alt=media&token=%s", cupboardFirebaseUrl, cupboardVersionDir.c_str(), it->first.c_str(), token.c_str());
        fetch(buf, [destPath, onDone](emscripten_fetch_t *fetchRes)
        {
            blit::File f(destPath, blit::OpenMode::write);
            f.write(0, fetchRes->numBytes, fetchRes->data);
            f.close();
            onDone(destPath);
        });
    });
}

#else
void initRemoteFiles()
{
}

void listRemoteFiles(std::function<void(blit::FileInfo &)> callback)
{
}

void *openRemoteFile(const std::string &file)
{
    return nullptr;
}

int32_t readRemoteFile(void *fh, uint32_t offset, uint32_t length, char* buffer)
{
    return -1;
}

int32_t closeRemoteFile(void *fh)
{
    return -1;
}

void downloadRemoteFile(const std::string &file, const std::string &destDir, std::function<void(const std::string &)> onDone)
{
}
#endif