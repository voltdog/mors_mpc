cmake_minimum_required(VERSION 3.16)
project(ros2lcm_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(mors_ros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_program(LCM_GEN lcm-gen)
if(NOT LCM_GEN)
  message(FATAL_ERROR "lcm-gen is required to build ros2lcm_bridge")
endif()

# Use the repository schemas as the only source of the wire protocol.
get_filename_component(MORS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(SCHEMAS
  "${MORS_ROOT}/lcm_msgs/robot_mode_cmd_msg.lcm"
  "${MORS_ROOT}/lcm_msgs/robot_mode_state_msg.lcm"
  "${MORS_ROOT}/lcm_msgs/robot_mode_result_msg.lcm"
  "${MORS_ROOT}/lcm_msgs/robot_cmd_msg.lcm"
  "${MORS_ROOT}/lcm_msgs/gait_params_msg.lcm")
set(GENERATED "${CMAKE_CURRENT_BINARY_DIR}/lcm_types")
set(OUTPUTS "${GENERATED}/mors_msgs/__init__.py")
foreach(NAME robot_mode_cmd_msg robot_mode_state_msg robot_mode_result_msg
             robot_cmd_msg gait_params_msg)
  list(APPEND OUTPUTS "${GENERATED}/mors_msgs/${NAME}.py")
endforeach()
add_custom_command(OUTPUT ${OUTPUTS}
  COMMAND ${CMAKE_COMMAND} -E make_directory "${GENERATED}"
  COMMAND ${LCM_GEN} -p --ppath "${GENERATED}" ${SCHEMAS}
  DEPENDS ${SCHEMAS} VERBATIM)
add_custom_target(bridge_lcm_types ALL DEPENDS ${OUTPUTS})

# A distinct runtime module avoids colliding with the rosidl-generated package.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY ros2lcm_bridge_runtime DESTINATION "${PYTHON_INSTALL_DIR}"
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY "${GENERATED}/mors_msgs"
  DESTINATION "${PYTHON_INSTALL_DIR}/ros2lcm_bridge_runtime/lcm_types"
  PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(bridge_protocol test/test_protocol.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${GENERATED} TIMEOUT 30)
  set(FSM_TEST_BINARY "" CACHE FILEPATH "Optional fsm_lcm binary for ROS/LCM integration tests")
  if(FSM_TEST_BINARY)
    ament_add_pytest_test(bridge_integration test/test_integration.py
      ENV FSM_BINARY=${FSM_TEST_BINARY} CONFIGPATH=${MORS_ROOT}/config TIMEOUT 90)
  endif()
endif()
ament_package()
