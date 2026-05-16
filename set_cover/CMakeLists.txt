cmake_minimum_required(VERSION 3.10)
project(SetCover)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include(FetchContent)
FetchContent_Declare(
    cli11
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
    GIT_TAG v2.6.0
)
FetchContent_MakeAvailable(cli11)

add_executable(setcover src/greedy.cpp)
target_compile_options(setcover PRIVATE -O3)
target_link_libraries(setcover PRIVATE CLI11::CLI11)