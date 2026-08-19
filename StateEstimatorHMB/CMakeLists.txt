cmake_minimum_required(VERSION 3.28.3)
# set the project name
project(state_estimator_hmb VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED True)


# check AVX2 
#===============================================================================
    set(AVX_FLAGS)

    include(CheckCXXSourceRuns)
    set(CMAKE_REQUIRED_FLAGS)

    # Check for AVX
    if (MSVC)
        if (NOT MSVC_VERSION LESS 1600)
            set(CMAKE_REQUIRED_FLAGS "/arch:AVX")
        endif ()
    else ()
        set(CMAKE_REQUIRED_FLAGS "-mavx")
    endif ()

    check_cxx_source_runs("
        #include <immintrin.h>
        int main()
        {
          __m256 a, b, c;
          const float src[8] = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f };
          float dst[8];
          a = _mm256_loadu_ps( src );
          b = _mm256_loadu_ps( src );
          c = _mm256_add_ps( a, b );
          _mm256_storeu_ps( dst, c );
          for( int i = 0; i < 8; i++ ){
            if( ( src[i] + src[i] ) != dst[i] ){
              return -1;
            }
          }
          return 0;
        }"
            HAVE_AVX_EXTENSIONS)

    # Check for AVX2
    if (MSVC)
        if (NOT MSVC_VERSION LESS 1800)
            set(CMAKE_REQUIRED_FLAGS "/arch:AVX2")
        endif ()
    else ()
        set(CMAKE_REQUIRED_FLAGS "-mavx2")
    endif ()

    check_cxx_source_runs("
        #include <immintrin.h>
        int main()
        {
          __m256i a, b, c;
          const int src[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
          int dst[8];
          a =  _mm256_loadu_si256( (__m256i*)src );
          b =  _mm256_loadu_si256( (__m256i*)src );
          c = _mm256_add_epi32( a, b );
          _mm256_storeu_si256( (__m256i*)dst, c );
          for( int i = 0; i < 8; i++ ){
            if( ( src[i] + src[i] ) != dst[i] ){
              return -1;
            }
          }
          return 0;
        }"
            HAVE_AVX2_EXTENSIONS)

    # Set Flags according to check results
    if (MSVC)
        if (HAVE_AVX2_EXTENSIONS AND NOT MSVC_VERSION LESS 1800)
            set(AVX_FLAGS "${AVX_FLAGS} /arch:AVX2")
        elseif (HAVE_AVX_EXTENSIONS AND NOT MSVC_VERSION LESS 1600)
            set(AVX_FLAGS "${AVX_FLAGS} /arch:AVX")
        endif ()
    else ()
        if (HAVE_AVX2_EXTENSIONS)
            set(AVX_FLAGS "${AVX_FLAGS} -mavx2")
            message(STATUS "AVX2 is supported")
        elseif (HAVE_AVX_EXTENSIONS)
            set(AVX_FLAGS "${AVX_FLAGS} -mavx")
            message(STATUS "AVX is supported")
        else()
            message(WARNING "AVX is not supported")
        endif ()
    endif ()
#===============================================================================

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto=auto -DEIGEN_STACK_ALLOCATION_LIMIT=0 -O3 -Wall -fPIC ${AVX_FLAGS}")

# add_definitions(-DCOMPILEDWITHC11)
message(STATUS "Using flag -std=c++20")

find_package(Eigen3 3.2.10 REQUIRED)
# find_package(OsqpEigen REQUIRED)
find_package(lcm REQUIRED)
find_package(pinocchio REQUIRED)
find_package(realsense2 QUIET)
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)

include(${LCM_USE_FILE})
set(LCM_MSGS_DIR "${CMAKE_CURRENT_LIST_DIR}/../lcm_msgs")
include_directories(BEFORE "${CMAKE_CURRENT_BINARY_DIR}")
include_directories("include/")
include_directories("../common/include/")
include_directories("${LCM_MSGS_DIR}")
include_directories(include /home/$ENV{USER}/vbcontrol/include)

# Put all message definition files in the type directory in one list
FILE(GLOB message_definitions "${LCM_MSGS_DIR}/*.lcm")

# Generate headers from message definition
lcm_wrap_types(CPP_HEADERS cpp_headers
  ${message_definitions})

# Create library from all the messages
lcm_add_library(messages-cpp CPP ${cpp_headers})
target_include_directories(messages-cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

add_library(state_estimator_hmb_contact_source src/contact_source.cpp)
target_include_directories(state_estimator_hmb_contact_source PUBLIC include)
target_compile_features(state_estimator_hmb_contact_source PUBLIC cxx_std_20)

add_library(state_estimator_hmb_heightmap_residual
  src/heightmap_residual_estimator.cpp)
target_include_directories(state_estimator_hmb_heightmap_residual PUBLIC include)
target_compile_features(state_estimator_hmb_heightmap_residual PUBLIC cxx_std_20)

if(realsense2_FOUND)
  add_subdirectory(/home/$ENV{USER}/vbcontrol ${PROJECT_BINARY_DIR}/build/vbcontrol)

  set(EXEC_SOURCES
      src/main.cpp
      src/z_pos_estimator.cpp
      src/sensor_fusion.cpp
      src/low_pass_filtering.cpp
      src/leg_state.cpp
      src/gm_force_observer.cpp
      src/HeightMapBuilder.cpp
      ../common/src/system_functions.cpp
      ../common/src/leg_model.cpp
      ../common/src/reliable_contact.cpp
      ../common/src/Robot.cpp
  )
  add_executable(${PROJECT_NAME} ${EXEC_SOURCES})
  target_link_libraries(${PROJECT_NAME}
    state_estimator_hmb_contact_source
    state_estimator_hmb_heightmap_residual
    vbcontrol
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    ZLIB::ZLIB
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
  )

  if(TARGET realsense2::realsense2)
    target_link_libraries(${PROJECT_NAME} realsense2::realsense2)
  else()
    target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
  endif()

  target_compile_definitions(${PROJECT_NAME} PRIVATE
      PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
      COAL_DISABLE_HPP_FCL_WARNINGS
      # Match Eigen's allocation and layout policy to precompiled Pinocchio.
      EIGEN_MALLOC_ALREADY_ALIGNED=1
      EIGEN_MAX_ALIGN_BYTES=16
      EIGEN_MAX_STATIC_ALIGN_BYTES=16
  )
else()
  message(WARNING
      "realsense2 was not found; building simulation-only state_estimator_hmb")
  add_executable(${PROJECT_NAME}
      src/sim_main.cpp
      src/HeightMapBuilder.cpp)
  target_link_libraries(${PROJECT_NAME}
      yaml-cpp::yaml-cpp
      ZLIB::ZLIB)
endif()

lcm_target_link_libraries(${PROJECT_NAME} messages-cpp ${LCM_NAMESPACE}lcm)

include(CTest)
if(BUILD_TESTING)
  add_executable(contact_source_test tests/contact_source_test.cpp)
  target_link_libraries(contact_source_test PRIVATE state_estimator_hmb_contact_source)
  add_test(NAME contact_source_test COMMAND contact_source_test)

  add_executable(heightmap_residual_estimator_test
    tests/heightmap_residual_estimator_test.cpp)
  target_link_libraries(heightmap_residual_estimator_test PRIVATE
    state_estimator_hmb_heightmap_residual)
  add_test(
    NAME heightmap_residual_estimator_test
    COMMAND heightmap_residual_estimator_test)

  add_executable(heightmap_builder_correction_test
    tests/heightmap_builder_correction_test.cpp
    src/HeightMapBuilder.cpp)
  target_link_libraries(heightmap_builder_correction_test
    yaml-cpp::yaml-cpp
    ZLIB::ZLIB)
  target_compile_definitions(heightmap_builder_correction_test PRIVATE
    "HEIGHTMAP_BUILDER_CORRECTION_TEST_CONFIG=\"${CMAKE_CURRENT_LIST_DIR}/tests/heightmap_builder_correction_test.yaml\"")
  lcm_target_link_libraries(
    heightmap_builder_correction_test messages-cpp ${LCM_NAMESPACE}lcm)
  add_test(
    NAME heightmap_builder_correction_test
    COMMAND heightmap_builder_correction_test)

  add_executable(z_pos_estimator_test
    tests/z_pos_estimator_test.cpp
    src/z_pos_estimator.cpp
    ../common/src/reliable_contact.cpp
    ../common/src/Robot.cpp
    ../common/src/system_functions.cpp
  )
  target_include_directories(z_pos_estimator_test PRIVATE
    include
    ../common/include
  )
  target_link_libraries(z_pos_estimator_test PRIVATE
    Eigen3::Eigen
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
  )
  target_compile_definitions(z_pos_estimator_test PRIVATE
    PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
    COAL_DISABLE_HPP_FCL_WARNINGS
    EIGEN_MALLOC_ALREADY_ALIGNED=1
    EIGEN_MAX_ALIGN_BYTES=16
    EIGEN_MAX_STATIC_ALIGN_BYTES=16
  )
  add_test(NAME z_pos_estimator_test COMMAND z_pos_estimator_test)
  set_tests_properties(z_pos_estimator_test PROPERTIES
    ENVIRONMENT "CONFIGPATH=${CMAKE_CURRENT_LIST_DIR}/../config"
  )
endif()
