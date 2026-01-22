cmake_minimum_required(VERSION 3.25.1)
project(imu)

# Set C/C++ standards
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(imu main.cpp)

# Find pigpio library (usually installed system-wide)
find_library(PIGPIO_LIBRARY NAMES pigpio libpigpio REQUIRED)
find_path(PIGPIO_INCLUDE_DIR NAMES pigpio.h PATHS /usr/include /usr/local/include)

if(NOT PIGPIO_LIBRARY OR NOT PIGPIO_INCLUDE_DIR)
    message(FATAL_ERROR "pigpio library not found! Please install it first:\n"
                        "   sudo apt update\n"
                        "   sudo apt install pigpio libpigpio-dev")
endif()

#  Add include directories 
target_include_directories(imu PRIVATE
    ${CMAKE_SOURCE_DIR}/header
    ${PIGPIO_INCLUDE_DIR}
)

#  Collect additional source files 
file(GLOB_RECURSE SOURCES
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)

#  Find Eigen3 
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

#  Add sources and link libraries 
target_sources(imu PRIVATE ${SOURCES})
target_link_libraries(imu PRIVATE
    ${PIGPIO_LIBRARY}
    #Eigen3::Eigen
)



# Link libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${PIGPIO_LIBRARY}
         pthread
         rt
)

# Most common extra flags when working with pigpio
target_compile_options(${PROJECT_NAME}
    PRIVATE
        -Wall
        -Wextra
        -O2               # uncomment for release build
        -pthread          
)

# Optional: copy binary to simpler name
set_target_properties(${PROJECT_NAME} PROPERTIES
    OUTPUT_NAME "main"
)