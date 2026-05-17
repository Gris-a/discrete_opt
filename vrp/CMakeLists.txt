cmake_minimum_required(VERSION 3.10)
project(VRP)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include(FetchContent)
FetchContent_Declare(
    cli11
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
    GIT_TAG v2.6.0
)
FetchContent_MakeAvailable(cli11)

add_executable(vrp src/greedy.cpp)
target_compile_options(vrp PRIVATE -O3)
target_link_libraries(vrp PRIVATE CLI11::CLI11)