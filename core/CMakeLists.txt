add_library(BlitSimCore INTERFACE)

target_sources(BlitSimCore INTERFACE
    ARMv7MCore.cpp
    MemoryBus.cpp
)

target_include_directories(BlitSimCore INTERFACE ${CMAKE_CURRENT_LIST_DIR})
