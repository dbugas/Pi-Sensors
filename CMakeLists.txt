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

# ---------------- Generate config header (MUST be early) ----------------
configure_file(
    ${CMAKE_SOURCE_DIR}/header/imu_config.h.in
    ${CMAKE_BINARY_DIR}/imu_config.h
    @ONLY
)

# Optional: print where it should be generated (helpful for debugging)
message(STATUS "Generated config header will be at: ${CMAKE_BINARY_DIR}/imu_config.h")

# ---------------- Sources ----------------
file(GLOB_RECURSE IMU_SOURCES
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)

# ---------------- Main Executable ----------------
add_executable(imu main.cpp ${IMU_SOURCES})

target_include_directories(imu PUBLIC
    ${CMAKE_SOURCE_DIR}/header
    ${CMAKE_BINARY_DIR}              
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

# You can keep or remove this line now
# target_compile_definitions(imu PUBLIC SOURCE_DIR="${PROJECT_SOURCE_DIR}")

# ---------------- Python bindings ----------------
add_subdirectory(python)