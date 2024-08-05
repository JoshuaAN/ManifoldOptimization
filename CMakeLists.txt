cmake_minimum_required(VERSION 3.14)  # FetchContent is available since CMake 3.11
set (CMAKE_CXX_STANDARD 17)

project(fieldcal)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

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

# Add your executable or library
add_executable(main main.cpp)

# Link Eigen to your target
target_link_libraries(main PRIVATE Eigen3::Eigen)