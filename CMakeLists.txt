cmake_minimum_required(VERSION 3.14)  # FetchContent is available since CMake 3.11
set (CMAKE_CXX_STANDARD 17)

project(fieldcal)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# Add your executable or library
add_executable(main main.cpp optimization.cpp)

# Enable the generation of compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable FetchContent module
include(FetchContent)

# Declare Eigen as a dependency
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0  # Replace with the desired version
)

# Make the Eigen content available
FetchContent_MakeAvailable(eigen)

# Link Eigen to target
target_link_libraries(main PRIVATE Eigen3::Eigen)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

target_link_libraries(main PRIVATE nlohmann_json::nlohmann_json)