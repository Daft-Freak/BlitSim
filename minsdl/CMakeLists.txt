# minimal SDL shell

add_executable(BlitSim_SDL Main.cpp)

find_package(SDL2 REQUIRED)

target_link_libraries(BlitSim_SDL BlitSimCore SDL2::SDL2)

if(TARGET SDL2::SDL2main)
    target_link_libraries(BlitSim_SDL SDL2::SDL2main)
endif()

install(TARGETS BlitSim_SDL
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    BUNDLE DESTINATION bin
)

