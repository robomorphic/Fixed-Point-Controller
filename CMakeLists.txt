# Copyright 2021 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)

# INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
# Default to GLVND if available.
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -ferror-limit=10000")

# This line has to appear before 'PROJECT' in order to be able to disable incremental linking
set(MSVC_INCREMENTAL_DEFAULT ON)

project(
  mujoco_samples
  VERSION 3.1.2
  DESCRIPTION "MuJoCo samples binaries"
  HOMEPAGE_URL "https://mujoco.org"
)

enable_language(C)
enable_language(CXX)
if(APPLE)
  enable_language(OBJC)
  enable_language(OBJCXX)
endif()

# Check if we are building as standalone project.
set(SAMPLE_STANDALONE OFF)
set(_INSTALL_SAMPLES ON)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(SAMPLE_STANDALONE ON)
  # If standalone, do not install the samples.
  set(_INSTALL_SAMPLES OFF)
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

if(SAMPLE_STANDALONE)
  include(SampleOptions)
else()
  enforce_mujoco_macosx_min_version()
endif()
include(SampleDependencies)

set(MUJOCO_SAMPLE_COMPILE_OPTIONS "${AVX_COMPILE_OPTIONS}" "${EXTRA_COMPILE_OPTIONS}")
set(MUJOCO_SAMPLE_LINK_OPTIONS "${EXTRA_LINK_OPTIONS}")

if(MUJOCO_HARDEN)
  if(WIN32)
    set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -Wl,/DYNAMICBASE)
  else()
    set(MUJOCO_SAMPLE_COMPILE_OPTIONS "${MUJOCO_SAMPLE_COMPILE_OPTIONS}" -fPIE)
    if(APPLE)
      set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -Wl,-pie)
    else()
      set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -pie)
    endif()
  endif()
endif()

# Build sample binaries
add_executable(compile src/compile.cc)
target_compile_options(compile PUBLIC ${MUJOCO_SAMPLE_COMPILE_OPTIONS})
target_link_libraries(compile Threads::Threads)
target_link_options(compile PRIVATE ${MUJOCO_SAMPLE_LINK_OPTIONS})
target_link_libraries(compile mujoco::mujoco)

# This mujoco is built by me and installed in /Users/alp/mujoco/install
set(MUJOCO_FOLDER /Users/alp/mujoco/install/lib/cmake)
find_package(mujoco REQUIRED PATHS ${MUJOCO_FOLDER})
find_package(pinocchio REQUIRED) # cmake installs this in _deps
find_package(Eigen3 3.3 REQUIRED NO_MODULE) # system installation

add_executable(pinocchio_example src/pinocchio_example.cc)
target_compile_options(pinocchio_example PUBLIC ${MUJOCO_SAMPLE_COMPILE_OPTIONS})
target_link_libraries(pinocchio_example mujoco::mujoco pinocchio)

add_executable(intn_tests src/intn_tests.cpp src/intn.cpp)
target_compile_options(intn_tests PUBLIC ${MUJOCO_SAMPLE_COMPILE_OPTIONS})
target_link_libraries(intn_tests mujoco::mujoco Eigen3::Eigen)
target_include_directories(intn_tests PUBLIC /usr/local/include ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Build samples that require GLFW.

find_package(osqp)

add_executable(basic_gravity src/basic_gravity.cc)
target_compile_options(basic_gravity PUBLIC ${MUJOCO_SAMPLE_COMPILE_OPTIONS})
target_link_libraries(
  basic_gravity
  mujoco::mujoco
  glfw
  Threads::Threads
  pinocchio
  osqp::osqp
)
target_include_directories(basic_gravity PUBLIC /usr/local/include ${CMAKE_CURRENT_SOURCE_DIR}/include)


add_executable(findbug src/findbug.cpp)
target_compile_options(findbug PUBLIC ${MUJOCO_SAMPLE_COMPILE_OPTIONS})
target_link_libraries(
  findbug
  mujoco::mujoco
  glfw
  Threads::Threads
  pinocchio
  osqp::osqp
)
target_include_directories(findbug PUBLIC /usr/local/include ${CMAKE_CURRENT_SOURCE_DIR}/include)


add_executable(basic src/basic.cc)
target_compile_options(basic PUBLIC ${MUJOCO_SAMPLE_COMPILE_OPTIONS})
target_link_libraries(
  basic
  mujoco::mujoco
  glfw
  Threads::Threads
)
target_link_options(basic PRIVATE ${MUJOCO_SAMPLE_LINK_OPTIONS})

add_executable(record src/record.cc src/array_safety.h)
target_compile_options(record PUBLIC ${MUJOCO_SAMPLE_COMPILE_OPTIONS})
target_link_libraries(
  record
  mujoco::mujoco
  glfw
  Threads::Threads
)
target_link_options(record PRIVATE ${MUJOCO_SAMPLE_LINK_OPTIONS})

if(APPLE AND MUJOCO_BUILD_MACOS_FRAMEWORKS)
  embed_in_bundle(basic simulate)
  embed_in_bundle(compile simulate)
  embed_in_bundle(record simulate)
endif()

# Do not install if macOS Bundles are created as RPATH is managed manually there.
if(APPLE AND MUJOCO_BUILD_MACOS_FRAMEWORKS)
  set(_INSTALL_SAMPLES OFF)
endif()

if(_INSTALL_SAMPLES)

  include(TargetAddRpath)

  # Add support to RPATH for the samples.
  target_add_rpath(
    TARGETS
    basic
    compile
    record
    INSTALL_DIRECTORY
    "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
    LIB_DIRS
    "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
    DEPENDS
    MUJOCO_ENABLE_RPATH
  )

  install(
    TARGETS basic
            compile
            record
    EXPORT ${PROJECT_NAME}
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT samples
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT samples
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT samples
    BUNDLE DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT samples
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT samples
  )

  if(NOT MUJOCO_SAMPLES_USE_SYSTEM_GLFW)
    # We downloaded GLFW. Depending if it is a static or shared LIBRARY we might
    # need to install it.
    get_target_property(MJ_GLFW_LIBRARY_TYPE glfw TYPE)
    if(MJ_GLFW_LIBRARY_TYPE STREQUAL SHARED_LIBRARY)
      install(
        TARGETS glfw
        EXPORT ${PROJECT_NAME}
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT samples
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT samples
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT samples
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT samples
      )
    endif()
  endif()
endif()
