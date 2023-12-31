if(32BLIT_PICO OR 32BLIT_HW)
    message(FATAL_ERROR "Can't run on hardware, also there's no point.")
endif()

option(ENABLE_SCREEN_SPEED_HACKS "Patch screen functions for more speed" OFF)

blit_executable(BlitSim
    API.cpp
    Main.cpp
    Metadata.cpp
    RemoteFiles.cpp
)
#blit_assets_yaml(DaftBoy32 assets.yml)
blit_metadata(BlitSim metadata.yml)

target_link_libraries(BlitSim BlitSimCore)

set_target_properties(BlitSim PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)

if(ENABLE_SCREEN_SPEED_HACKS)
    target_compile_definitions(BlitSim PRIVATE SCREEN_SPEED_HACKS)
endif()

if(EMSCRIPTEN)
    include(FetchContent)

    FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
    FetchContent_MakeAvailable(json)

    target_link_libraries(BlitSim nlohmann_json::nlohmann_json)

    target_link_options(BlitSim PRIVATE -sINITIAL_MEMORY=48MB --preload-file "${CMAKE_CURRENT_LIST_DIR}/em-files@/" -sFETCH=1)
endif()