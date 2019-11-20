cmake_minimum_required(VERSION 2.8)
project(pong++)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set (CMAKE_CXX_STANDARD 20)

include_directories(
    ${PROJECT_SOURCE_DIR}/src
)

file(GLOB SOURCES
    "${PROJECT_SOURCE_DIR}/src/*.cc"
)

add_executable(pong++ ${SOURCES})
target_link_libraries (pong++ sfml-graphics sfml-window sfml-system)



