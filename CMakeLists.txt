cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(chip8)


pico_sdk_init()

add_subdirectory(external)

add_executable(chip8
	chip8.c
)

pico_add_extra_outputs(chip8)

if (CMAKE_BUILD_TYPE EQUAL "Debug")
    target_compile_definitions(chip8 PUBLIC CHIP8_DEBUG)
endif()
target_link_libraries(chip8 ssd1306 pico_stdlib hardware_flash)

