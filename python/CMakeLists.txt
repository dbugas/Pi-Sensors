# python/CMakeLists.txt

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

add_library(imupy MODULE
    imupy.cpp
    ${IMU_SOURCES}
)

target_include_directories(imupy PRIVATE
    ${CMAKE_SOURCE_DIR}/header
    ${PIGPIO_INCLUDE_DIR}
)

target_link_libraries(imupy PRIVATE
    pybind11::module
    Python3::Python
    ${PIGPIO_LIBRARY}
    pthread
    rt
    Eigen3::Eigen
)

set_target_properties(imupy PROPERTIES
    PREFIX ""
    OUTPUT_NAME "imupy"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/python/
    POSITION_INDEPENDENT_CODE ON
)