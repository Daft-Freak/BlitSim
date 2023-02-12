if(32BLIT_PICO OR 32BLIT_HW)
    message(FATAL_ERROR "Can't run on hardware, also there's no point.")
endif()

blit_executable(BlitSim
    Main.cpp
)
#blit_assets_yaml(DaftBoy32 assets.yml)
#blit_metadata(DaftBoy32 metadata.yml)

target_link_libraries(BlitSim BlitSimCore)

set_target_properties(BlitSim PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)