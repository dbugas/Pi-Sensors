cmake_minimum_required(VERSION 3.25.1)
project(imu)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------- Dependencies ----------------

find_library(PIGPIO_LIBRARY NAMES pigpio libpigpio REQUIRED)
find_path(PIGPIO_INCLUDE_DIR NAMES pigpio.h PATHS /usr/include /usr/local/include)

if(NOT PIGPIO_LIBRARY OR NOT PIGPIO_INCLUDE_DIR)
    message(FATAL_ERROR "pigpio library not found!")
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ---------------- Sources ----------------

file(GLOB_RECURSE IMU_SOURCES
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)

# ---------------- Executable ----------------

add_executable(imu main.cpp ${IMU_SOURCES})

target_include_directories(imu PRIVATE
    ${CMAKE_SOURCE_DIR}/header
    ${PIGPIO_INCLUDE_DIR}
)

target_link_libraries(imu PRIVATE
    ${PIGPIO_LIBRARY}
    pthread
    rt
    Eigen3::Eigen
)

target_compile_options(imu PRIVATE
    -Wall -Wextra -O2 -pthread
)

set_target_properties(imu PROPERTIES
    OUTPUT_NAME "main"
)

# ---------------- Python bindings ----------------

add_subdirectory(python)
