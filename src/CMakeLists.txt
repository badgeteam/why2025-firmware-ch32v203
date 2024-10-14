cmake_minimum_required(VERSION 3.21.0)
cmake_policy(VERSION 3.21)

set(PROJECT_NAME "tanmatsu_coprocessor")
set(PREFIX "riscv64-elf-")

project(${PROJECT_NAME} LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS On)

set(CMAKE_C_COMPILER "${PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${PREFIX}g++")

#add_subdirectory(platform)
#add_subdirectory(hal)
add_subdirectory(application)
