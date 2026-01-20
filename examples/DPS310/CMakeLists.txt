# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name and version
project(DPS310_Sensor VERSION 1.0 LANGUAGES CXX)

# Set C++ standard to C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find pigpio library (required for i2c communication)
find_library(PIGPIO_LIBRARY NAMES pigpio)
if(NOT PIGPIO_LIBRARY)
    message(FATAL_ERROR "pigpio library not found. Please install it using 'sudo apt install pigpio'.")
endif()

# Find rt library (required for pigpio)
find_library(RT_LIBRARY NAMES rt)
if(NOT RT_LIBRARY)
    message(FATAL_ERROR "rt library not found. Please ensure librt is installed.")
endif()

# Add the executable target
add_executable(dps310_test
    dps310.cpp
    main.cpp
)

# Include directories (if you have header files in a separate directory)
target_include_directories(dps310_test PRIVATE ${CMAKE_SOURCE_DIR})

# Link the pigpio and rt libraries to the executable
target_link_libraries(dps310_test PRIVATE
    ${PIGPIO_LIBRARY}
    ${RT_LIBRARY}
)

# Add compiler flags for warnings and optimization
target_compile_options(dps310_test PRIVATE
    -Wall
    -Wextra
)
