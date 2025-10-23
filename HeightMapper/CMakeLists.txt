# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
cmake_minimum_required(VERSION 3.1.0)

project(HeightMapper)

# Find librealsense2 installed package


# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto=auto -DEIGEN_STACK_ALLOCATION_LIMIT=0 -O3 -Wall -fPIC")

# add_definitions(-DCOMPILEDWITHC11)
message(STATUS "Using flag -std=c++20")

find_package(Eigen3 REQUIRED)
# find_package(OsqpEigen REQUIRED)
find_package(lcm REQUIRED)
find_package(realsense2 REQUIRED)

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_MakeAvailable(yaml-cpp)

include(${LCM_USE_FILE})
include_directories("include/")
include_directories("../common/include/")
include_directories("/home/$ENV{USER}/lcm_msgs/")
include_directories(include /home/$ENV{USER}/vbcontrol/include)

# Put all message definition files in the type directory in one list
# FILE(GLOB message_definitions "${CMAKE_CURRENT_LIST_DIR}/lcm_msgs/mors_msgs/*.lcm")
FILE(GLOB message_definitions "/home/$ENV{USER}/lcm_msgs/mors_msgs/*.lcm")

# Generate headers from message definition
lcm_wrap_types(CPP_HEADERS cpp_headers
  ${message_definitions})

# Create library from all the messages
lcm_add_library(messages-cpp CPP ${cpp_headers})
target_include_directories(messages-cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

# Add the application sources to the target
set(EXEC_SOURCES src/main.cpp 
            ../common/src/system_functions.cpp
)
add_executable(${PROJECT_NAME} ${EXEC_SOURCES})

# Link librealsense2 to the target
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
target_link_libraries(${PROJECT_NAME} yaml-cpp::yaml-cpp)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
lcm_target_link_libraries(${PROJECT_NAME} messages-cpp ${LCM_NAMESPACE}lcm)