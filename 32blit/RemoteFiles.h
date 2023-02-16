#pragma once
#include <cstdint>

#include "engine/file.hpp"

void initRemoteFiles();

void listRemoteFiles(std::function<void(blit::FileInfo &)> callback);

void *openRemoteFile(const std::string &file);
int32_t readRemoteFile(void *fh, uint32_t offset, uint32_t length, char* buffer);
int32_t closeRemoteFile(void *fh);

void downloadRemoteFile(const std::string &file, const std::string &destDir, std::function<void(const std::string &)> onDone);