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
find_package(yaml-cpp CONFIG QUIET)
if(NOT yaml-cpp_FOUND)
  include(FetchContent)
  set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "Build yaml-cpp tools" FORCE)
  FetchContent_Declare(yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG 0.8.0)
  FetchContent_MakeAvailable(yaml-cpp)
endif()
find_package(lcm REQUIRED)
include(${LCM_USE_FILE})

set(LCM_MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lcm_msgs" CACHE PATH "Path to LCM schemas")
file(GLOB message_definitions "${LCM_MSGS_DIR}/*.lcm")
lcm_wrap_types(CPP_HEADERS cpp_headers ${message_definitions})
lcm_add_library(fsm_messages CPP ${cpp_headers})
target_include_directories(fsm_messages SYSTEM INTERFACE "${CMAKE_CURRENT_BINARY_DIR}")

find_package(OsqpEigen REQUIRED)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)
set(VBCONTROL_DIR "$ENV{HOME}/vbcontrol" CACHE PATH "Path to vbcontrol checkout")
add_subdirectory("${VBCONTROL_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/vbcontrol")
set(mpc_src src/LocomotionControllerMPC)
set(mpc_sources
  ${mpc_src}/ConvexMpc.cpp
  ${mpc_src}/ConvexMpcThread.cpp
  ${mpc_src}/SimpleGaitScheduler.cpp
  ${mpc_src}/GaitTransition.cpp
  ${mpc_src}/ContactStateFSM.cpp
  ${mpc_src}/SwingController.cpp
  ${mpc_src}/SwingTrajectoryGenerator.cpp
  ${mpc_src}/FootStepPlanner.cpp
  ${mpc_src}/ReferenceGenerator.cpp
  ${mpc_src}/WBICThread.cpp
  ${mpc_src}/wbic/wbic_control.cpp
  ${mpc_src}/wbic/tasks/wbc_task.cpp
  ${mpc_src}/wbic/tasks/body_ori_task.cpp
  ${mpc_src}/wbic/tasks/body_pos_task.cpp
  ${mpc_src}/wbic/tasks/tip_pos_task.cpp
  ../common/src/Robot.cpp
  ../common/src/system_functions.cpp
)
set(mpc_includes include/LocomotionControllerMPC ../common/include "${VBCONTROL_DIR}/include")
# Match the Eigen alignment used by the prebuilt Pinocchio libraries.
set(mpc_definitions EIGEN_DONT_PARALLELIZE EIGEN_MAX_ALIGN_BYTES=16
  EIGEN_MAX_STATIC_ALIGN_BYTES=16 COAL_DISABLE_HPP_FCL_WARNINGS)
set(mpc_libraries Eigen3::Eigen Threads::Threads OsqpEigen::OsqpEigen vbcontrol
  pinocchio::pinocchio pinocchio::pinocchio_parsers eiquadprog::eiquadprog yaml-cpp::yaml-cpp)

add_library(vision_mpc STATIC ${mpc_sources})
target_include_directories(vision_mpc PUBLIC ${mpc_includes})
target_compile_definitions(vision_mpc PUBLIC ${mpc_definitions})
target_link_libraries(vision_mpc PUBLIC ${mpc_libraries})

add_library(fsm_locomotion STATIC
  ${mpc_src}/LocomotionControllerMPC.cpp
  ${mpc_src}/VisionConfig.cpp
  src/actions/laydown.cpp
  src/actions/standup.cpp
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
  Eigen3::Eigen Threads::Threads yaml-cpp::yaml-cpp vision_mpc)

add_executable(fsm_locomotion_node src/main.cpp)
target_link_libraries(fsm_locomotion_node PRIVATE fsm_locomotion)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_locomotion_stop tests/test_locomotion_stop.cpp)
  target_link_libraries(test_locomotion_stop PRIVATE fsm_locomotion)
  target_compile_options(test_locomotion_stop PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
  add_test(NAME fsm_locomotion_stop COMMAND test_locomotion_stop)
  set_tests_properties(fsm_locomotion_stop PROPERTIES TIMEOUT 10)
  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_Interpreter_FOUND)
    add_test(NAME fsm_lcm COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_lcm.py"
      "$<TARGET_FILE:fsm_locomotion_node>"
      "$<TARGET_FILE:${LCM_NAMESPACE}lcm-gen>" "${LCM_MSGS_DIR}")
    set_tests_properties(fsm_lcm PROPERTIES TIMEOUT 70 SKIP_RETURN_CODE 77)
  endif()
endif()

if(FSM_ENABLE_IPO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error LANGUAGES CXX)
  if(NOT ipo_supported)
    message(STATUS "IPO/LTO is not supported: ${ipo_error}")
  endif()
endif()

foreach(target IN ITEMS fsm_locomotion fsm_locomotion_node)
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
