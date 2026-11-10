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

add_library(fsm_locomotion STATIC
  src/FSMTypes.cpp
  src/LocomotionControl.cpp
  src/ReadinessChecker.cpp
  src/RobotFSM.cpp
  src/SafetyMonitor.cpp
  src/States.cpp
)
target_include_directories(fsm_locomotion PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_features(fsm_locomotion PUBLIC cxx_std_20)

add_executable(fsm_locomotion_demo src/main.cpp)
target_link_libraries(fsm_locomotion_demo PRIVATE fsm_locomotion)

include(CTest)
if(BUILD_TESTING)
  add_executable(fsm_tests tests/fsm_tests.cpp)
  target_link_libraries(fsm_tests PRIVATE fsm_locomotion)
  add_test(NAME fsm_tests COMMAND fsm_tests)
  add_test(NAME fsm_demo COMMAND fsm_locomotion_demo)
endif()

if(FSM_ENABLE_IPO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error LANGUAGES CXX)
  if(NOT ipo_supported)
    message(STATUS "IPO/LTO is not supported: ${ipo_error}")
  endif()
endif()

set(fsm_targets fsm_locomotion fsm_locomotion_demo)
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
