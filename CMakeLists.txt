cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)
include(cmake/VersionFromGit.cmake)

get_version(chip8)
message(STATUS "Chip8 version: ${chip8_VERSION}")
configure_file(include/chip8/version.h.in include/chip8/version.h @ONLY)

project(chip8)

pico_sdk_init()

add_subdirectory(external)

add_executable(chip8
	chip8.c
)

pico_add_extra_outputs(chip8)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    message(STATUS "Configuring debug build of chip8")
    target_compile_definitions(chip8 PUBLIC CHIP8_DEBUG)
endif()
target_include_directories(chip8 PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/include)
target_link_libraries(chip8 ssd1306 pico_stdlib hardware_flash)

