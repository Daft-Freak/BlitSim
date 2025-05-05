#pragma once
#include <cstdint>
#include <string>

extern std::string launchFile;
extern uint32_t metadataOffset;

void apiInit();

void apiCallback(int index, uint32_t *regs);

uint32_t firmwareMemRead(uint32_t addr, uint32_t val, int width);
uint32_t firmwareMemWrite(uint32_t addr, uint32_t val, int width);

void hookScreenBlend();

void syncInput();