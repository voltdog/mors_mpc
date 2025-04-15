cmake_minimum_required(VERSION 3.28.3)
# set the project name
project(bhi350_imu VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED True)


# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto=auto -DEIGEN_STACK_ALLOCATION_LIMIT=0 -O3 -Wall -fPIC")

# add_definitions(-DCOMPILEDWITHC11)
message(STATUS "Using flag -std=c++20")

find_package(Eigen3 REQUIRED)
# find_package(OsqpEigen REQUIRED)
find_package(lcm REQUIRED)

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_MakeAvailable(yaml-cpp)

include(${LCM_USE_FILE})
include_directories("include/")
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

# executable
# add_subdirectory(/home/$ENV{USER}/vbcontrol/matplotplusplus)
add_subdirectory(/home/$ENV{USER}/vbcontrol ${PROJECT_BINARY_DIR}/build/vbcontrol)

add_executable(bhi360_imu "src/bhi360_imu.cpp")
lcm_target_link_libraries(bhi360_imu messages-cpp ${LCM_NAMESPACE}lcm)
target_link_libraries(bhi360_imu -lhidapi-libusb)
target_link_libraries(bhi360_imu yaml-cpp::yaml-cpp)

# set(LEG_SOURCES src/main.cpp 
#             src/leg_control.cpp 
#             src/lcm_data_exchange.cpp
# )
# add_executable(leg_control ${LEG_SOURCES})
# # target_link_libraries(leg_control OsqpEigen::OsqpEigen)
# target_link_libraries(leg_control vbcontrol)
# target_link_libraries(leg_control Eigen3::Eigen)
# target_link_libraries(leg_control yaml-cpp::yaml-cpp)
# lcm_target_link_libraries(leg_control messages-cpp ${LCM_NAMESPACE}lcm)

