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

SET(CMAKE_CXX_FLAGS_DEBUG "-O0")
SET(CMAKE_C_FLAGS_DEBUG "-O0")


# Build sample binaries

# This mujoco is built by me and installed in /Users/alp/mujoco/install
set(MUJOCO_FOLDER /Users/alp/mujoco/install/lib/cmake)
find_package(mujoco REQUIRED PATHS ${MUJOCO_FOLDER})
find_package(pinocchio REQUIRED) # cmake installs this in _deps
find_package(Eigen3 3.3 REQUIRED NO_MODULE) # system installation


find_package(boost)


find_package(osqp)

add_executable(trajectory_tracking_only_pos src/trajectory_tracking_only_pos.cc)
target_compile_options(trajectory_tracking_only_pos PUBLIC ${MUJOCO_SAMPLE_COMPILE_OPTIONS})
target_link_libraries(
  trajectory_tracking_only_pos
  mujoco::mujoco
  glfw
  Threads::Threads
  pinocchio
  osqp::osqp
  ${Boost_LIBRARIES}
)
target_include_directories(trajectory_tracking_only_pos PUBLIC /usr/local/include ${CMAKE_CURRENT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})


#add_executable(first_controllers src/first_controllers.cc)
#target_compile_options(first_controllers PUBLIC ${MUJOCO_SAMPLE_COMPILE_OPTIONS})
#target_link_libraries(
#  first_controllers
#  mujoco::mujoco
#  glfw
#  Threads::Threads
#  pinocchio
#  osqp::osqp
#  ${Boost_LIBRARIES}
#)
#target_include_directories(first_controllers PUBLIC /usr/local/include ${CMAKE_CURRENT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})

#add_executable(osqp_type_form src/osqp_type_form.cc)
#target_compile_options(osqp_type_form PUBLIC ${MUJOCO_SAMPLE_COMPILE_OPTIONS})
#target_link_libraries(
#  osqp_type_form
#  mujoco::mujoco
#  glfw
#  Threads::Threads
#  pinocchio
#  osqp::osqp
#  ${Boost_LIBRARIES}
#)
#target_include_directories(osqp_type_form PUBLIC /usr/local/include ${CMAKE_CURRENT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})






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
