cmake_minimum_required(VERSION 3.16)

project(contact_sensor VERSION 1.0 LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "contact_sensor requires Linux termios2 support")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(lcm REQUIRED)
find_package(yaml-cpp REQUIRED)

include("${LCM_USE_FILE}")

set(CONTACT_SENSOR_MESSAGE_DEFINITION
    "${CMAKE_CURRENT_SOURCE_DIR}/../lcm_msgs/contact_sensor_msg.lcm")

lcm_wrap_types(
    CPP_HEADERS contact_sensor_message_headers
    "${CONTACT_SENSOR_MESSAGE_DEFINITION}"
)

lcm_add_library(
    contact_sensor_message
    CPP
    ${contact_sensor_message_headers}
)
target_include_directories(
    contact_sensor_message
    INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
)

function(contact_sensor_enable_warnings target_name)
    target_compile_options(
        "${target_name}"
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
    )
endfunction()

add_library(contact_sensor_core src/core.cpp)
target_compile_features(contact_sensor_core PUBLIC cxx_std_20)
target_include_directories(
    contact_sensor_core
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
target_link_libraries(contact_sensor_core PRIVATE yaml-cpp::yaml-cpp)
contact_sensor_enable_warnings(contact_sensor_core)

add_executable(contact_sensor src/contact_sensor.cpp)
target_compile_features(contact_sensor PRIVATE cxx_std_20)
lcm_target_link_libraries(
    contact_sensor
    PRIVATE
        contact_sensor_core
        contact_sensor_message
        ${LCM_NAMESPACE}lcm
)
contact_sensor_enable_warnings(contact_sensor)

include(CTest)
if(BUILD_TESTING)
    add_executable(core_test tests/core_test.cpp)
    target_compile_features(core_test PRIVATE cxx_std_20)
    target_link_libraries(core_test PRIVATE contact_sensor_core)
    contact_sensor_enable_warnings(core_test)

    add_test(NAME core_test COMMAND core_test)
endif()
