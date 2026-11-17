cmake_minimum_required(VERSION 3.28.3)
project(FSMLocomotion VERSION 1.0 LANGUAGES CXX)

option(FSM_ENABLE_IPO "Enable IPO/LTO on optimized builds" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(lcm REQUIRED)
include(${LCM_USE_FILE})

set(LCM_MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lcm_msgs" CACHE PATH "Path to LCM schemas")
file(GLOB message_definitions "${LCM_MSGS_DIR}/*.lcm")
lcm_wrap_types(CPP_HEADERS cpp_headers ${message_definitions})
lcm_add_library(fsm_messages CPP ${cpp_headers})
target_include_directories(fsm_messages SYSTEM INTERFACE "${CMAKE_CURRENT_BINARY_DIR}")

add_library(fsm_locomotion STATIC
  src/FSM/ActionState.cpp
  src/FSM/ExternalState.cpp
  src/FSM/FailureState.cpp
  src/FSM/FSMTypes.cpp
  src/FSM/IdleState.cpp
  src/FSM/LaydownState.cpp
  src/FSM/LocomotionState.cpp
  src/FSM/StandingState.cpp
  src/FSM/StandupState.cpp
  src/FSM/State.cpp
  src/LcmDataExchange.cpp
  src/LocomotionControlBase.cpp
  src/LocomotionNode.cpp
  src/LocomotionTransitionMonitor.cpp
  src/ReadinessChecker.cpp
  src/RobotFSM.cpp
  src/SafetyMonitor.cpp
)
target_include_directories(fsm_locomotion PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/../common/include"
)
target_compile_features(fsm_locomotion PUBLIC cxx_std_20)
get_filename_component(config_dir "${CMAKE_CURRENT_SOURCE_DIR}/../config" ABSOLUTE)
target_compile_definitions(fsm_locomotion PRIVATE FSM_DEFAULT_CONFIG_DIR="${config_dir}")
lcm_target_link_libraries(fsm_locomotion fsm_messages ${LCM_NAMESPACE}lcm
  Eigen3::Eigen Threads::Threads yaml-cpp::yaml-cpp)

add_executable(fsm_locomotion_node src/main.cpp)
target_link_libraries(fsm_locomotion_node PRIVATE fsm_locomotion)

add_executable(fsm_locomotion_demo src/demo.cpp)
target_link_libraries(fsm_locomotion_demo PRIVATE fsm_locomotion)

include(CTest)
if(BUILD_TESTING)
  add_executable(fsm_tests tests/fsm_tests.cpp)
  target_link_libraries(fsm_tests PRIVATE fsm_locomotion)
  add_test(NAME fsm_tests COMMAND fsm_tests)
  add_test(NAME fsm_demo COMMAND fsm_locomotion_demo)
  set_tests_properties(fsm_tests fsm_demo PROPERTIES TIMEOUT 10)

  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_Interpreter_FOUND)
    add_test(NAME fsm_lcm COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_lcm.py"
      "$<TARGET_FILE:fsm_locomotion_node>"
      "$<TARGET_FILE:${LCM_NAMESPACE}lcm-gen>" "${LCM_MSGS_DIR}")
    set_tests_properties(fsm_lcm PROPERTIES TIMEOUT 20 SKIP_RETURN_CODE 77)
  endif()
endif()

if(FSM_ENABLE_IPO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error LANGUAGES CXX)
  if(NOT ipo_supported)
    message(STATUS "IPO/LTO is not supported: ${ipo_error}")
  endif()
endif()

set(fsm_targets fsm_locomotion fsm_locomotion_node fsm_locomotion_demo)
if(BUILD_TESTING)
  list(APPEND fsm_targets fsm_tests)
endif()
foreach(target IN LISTS fsm_targets)
  target_compile_options(${target} PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
  )
  if(FSM_ENABLE_IPO AND ipo_supported)
    set_target_properties(${target} PROPERTIES
      INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE
      INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO TRUE
    )
  endif()
endforeach()
