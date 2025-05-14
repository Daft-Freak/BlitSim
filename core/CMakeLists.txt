add_library(BlitSimCore INTERFACE)

target_sources(BlitSimCore INTERFACE
    ARMv7MCore.cpp
    MemoryBus.cpp
)

target_include_directories(BlitSimCore INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# recompiler
add_library(BlitSimRecompiler INTERFACE)
target_compile_definitions(BlitSimRecompiler INTERFACE RECOMPILER)

# common sources
target_sources(BlitSimRecompiler INTERFACE
    compiler/RecompilerGeneric.cpp
    # builders might be here if they didn't define enums with the same name
)

# detect target arch

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "AMD64")
    target_sources(BlitSimRecompiler INTERFACE compiler/X86Builder.cpp compiler/X86Target.cpp)
    target_compile_definitions(BlitSimRecompiler INTERFACE RECOMPILER_X86)
    set(HAVE_RECOMPILER TRUE)
endif()

if(HAVE_RECOMPILER)
    # enable recompiler
    target_sources(BlitSimCore INTERFACE
        compiler/ARMv7MRecompiler.cpp
    )
    target_link_libraries(BlitSimCore INTERFACE BlitSimRecompiler)
endif()