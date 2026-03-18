cmake_minimum_required(VERSION 3.28.3)
# set the project name
project(wbic_ctrl VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_BUILD_TYPE Release)
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")


# check AVX2 support

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O2 -g -DNDEBUG")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g3")

# add_definitions(-DCOMPILEDWITHC11)
message(STATUS "Using flag -std=c++20")

find_package(Eigen3 REQUIRED)
find_package(lcm REQUIRED)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(yaml-cpp CONFIG QUIET)

include(FetchContent)

if(NOT yaml-cpp_FOUND)
    FetchContent_Declare(
      yaml-cpp
      GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
      GIT_TAG master
    )
    FetchContent_MakeAvailable(yaml-cpp)
endif()

include(${LCM_USE_FILE})
include_directories("include/")
include_directories("../common/include/")
include_directories("/home/$ENV{USER}/lcm_msgs/")
include_directories(include /home/$ENV{USER}/vbcontrol/include)

# Put all message definition files in the type directory in one list
FILE(GLOB message_definitions "/home/$ENV{USER}/lcm_msgs/mors_msgs/*.lcm")

# Generate headers from message definition
lcm_wrap_types(CPP_HEADERS cpp_headers
  ${message_definitions})

# Create library from all the messages
lcm_add_library(messages-cpp CPP ${cpp_headers})
target_include_directories(messages-cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

# add librarires
add_subdirectory(/home/$ENV{USER}/vbcontrol ${PROJECT_BINARY_DIR}/build/vbcontrol)

# Для eiquadprog нужно использовать FetchContent, а не add_subdirectory
# FetchContent_Declare(
#   eiquadprog
#   GIT_REPOSITORY https://github.com/stack-of-tasks/eiquadprog.git
#   GIT_TAG master
# )
# FetchContent_MakeAvailable(eiquadprog)

set(WBC_SOURCES src/main.cpp 
            src/wbic_control.cpp 
            src/lcm_data_exchange_wbic.cpp
            src/tasks/wbc_task.cpp
            src/tasks/body_ori_task.cpp
            src/tasks/body_pos_task.cpp
            src/tasks/tip_pos_task.cpp
            ../common/src/system_functions.cpp
            ../common/src/Robot.cpp
)
add_executable(${PROJECT_NAME} ${WBC_SOURCES})


target_link_libraries(${PROJECT_NAME} 
    vbcontrol
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    pinocchio::pinocchio 
    pinocchio::pinocchio_parsers
    eiquadprog::eiquadprog
    )

target_compile_definitions(${PROJECT_NAME}  PRIVATE
    PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
    COAL_DISABLE_HPP_FCL_WARNINGS
    # EIGEN_USE_AVX
    # EIGEN_USE_AVX2
    # EIGEN_DONT_ALIGN
    # EIGEN_MAKE_ALIGNED_OPERATOR_NEW
)

lcm_target_link_libraries(${PROJECT_NAME} messages-cpp ${LCM_NAMESPACE}lcm)
