cmake_minimum_required(VERSION 3.14)
project(dynamics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
# Eigen 3.4's CMakeLists pins an old minimum and ignores non-cache option() values
# unless CMP0077 is NEW. Without this, configure walks Eigen's docs and tests.
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(EIGEN_BUILD_DOC OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
set(BUILD_TESTING OFF)

FetchContent_Declare(
  Eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(Eigen3)

add_library(dynamics
  src/quaternion.cpp
  src/eom.cpp
  src/integrator.cpp
  src/dynamics.cpp
)
target_include_directories(dynamics PUBLIC include)
target_link_libraries(dynamics PUBLIC Eigen3::Eigen)

add_executable(test_step tests/test_step.cpp)
target_link_libraries(test_step PRIVATE dynamics)
