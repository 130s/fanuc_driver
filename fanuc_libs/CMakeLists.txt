# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(fanuc_libs VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake QUIET)
if (ament_cmake_FOUND)
    ament_package()
endif()

# ---------------------------------------------------------------------------
# Third-party dependencies.
#
# These sources are NOT vendored as git submodules. They are declared in the
# top-level `dep.repos` file and fetched with vcs2l *before* the build:
#
#     vcs import . < dep.repos      # run from the fanuc_driver repo root
#
# CMake then consumes the already-present sources in place through
# FetchContent's SOURCE_DIR, so no `git clone` happens during `colcon build`.
# ---------------------------------------------------------------------------
include(FetchContent)

set(FANUC_DEPS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/dependencies)

# Fail early, with an actionable message, if the sources were not imported.
foreach(_dep sockpp readerwriterqueue yaml-cpp reflect-cpp)
  if (NOT EXISTS ${FANUC_DEPS_DIR}/${_dep}/CMakeLists.txt)
    message(FATAL_ERROR
      "Third-party dependency '${_dep}' was not found under\n"
      "  ${FANUC_DEPS_DIR}/${_dep}\n"
      "Import the sources first by running, from the fanuc_driver repo root:\n"
      "  vcs import . < dep.repos")
  endif ()
endforeach()

# Applies a dependency's patch idempotently; a non-zero exit fails the
# configure step so a bad patch surfaces immediately instead of as a
# confusing downstream compile error.
function(fanuc_apply_dependency_patch dep patch)
  execute_process(
      COMMAND bash ${FANUC_DEPS_DIR}/apply_patch.bash ${FANUC_DEPS_DIR}/${patch}
      WORKING_DIRECTORY ${FANUC_DEPS_DIR}/${dep}
      COMMAND_ERROR_IS_FATAL ANY)
endfunction()

# Build the sockpp library
fanuc_apply_dependency_patch(sockpp sockpp.patch)
FetchContent_Declare(sockpp SOURCE_DIR ${FANUC_DEPS_DIR}/sockpp)
set(SOCKPP_BUILD_SHARED OFF CACHE BOOL "Build sockpp as a shared library")
set(SOCKPP_BUILD_STATIC ON CACHE BOOL "Build sockpp as a static library")
FetchContent_MakeAvailable(sockpp)
set_target_properties(sockpp-static PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Create readerwriterqueue target
fanuc_apply_dependency_patch(readerwriterqueue readerwriterqueue.patch)
FetchContent_Declare(readerwriterqueue SOURCE_DIR ${FANUC_DEPS_DIR}/readerwriterqueue)
FetchContent_MakeAvailable(readerwriterqueue)

FetchContent_Declare(yaml-cpp SOURCE_DIR ${FANUC_DEPS_DIR}/yaml-cpp)
FetchContent_MakeAvailable(yaml-cpp)
set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "Build yaml-cpp shared library")
set(YAML_CPP_INSTALL OFF CACHE BOOL "Enable generation of yaml-cpp install targets")

# Build the reflectcpp library
fanuc_apply_dependency_patch(reflect-cpp reflectcpp.patch)
set(REFLECTCPP_BUILD_SHARED OFF CACHE BOOL "Build reflectcpp as a shared library")
set(REFLECTCPP_YAML ON CACHE BOOL "Enable YAML support")
FetchContent_Declare(reflect-cpp SOURCE_DIR ${FANUC_DEPS_DIR}/reflect-cpp)
FetchContent_MakeAvailable(reflect-cpp)
set_target_properties(reflectcpp PROPERTIES CXX_STANDARD 20)
set_target_properties(reflectcpp PROPERTIES POSITION_INDEPENDENT_CODE ON)

if (BUILD_TESTING)
  include(FetchContent)
  FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "Enable installation of googletest." FORCE)
  set(BUILD_GMOCK ON CACHE BOOL "Builds the googlemock subproject." FORCE)
  FetchContent_MakeAvailable(googletest)
  include(GoogleTest)
  enable_testing()
endif ()

add_subdirectory(stream_motion)
add_subdirectory(rmi)
add_subdirectory(fanuc_client)
add_subdirectory(gpio_config)

if (BUILD_EXAMPLES)
  add_subdirectory(examples)
endif ()

# Create cmake config and version files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/fanuc_libsConfigVersion.cmake
    VERSION
    ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    ARCH_INDEPENDENT
)
configure_package_config_file(fanuc_libsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/fanuc_libsConfig.cmake
    INSTALL_DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/fanuc_libs/
)

# --- Install ---
include(GNUInstallDirs)
install(TARGETS stream_motion rmi fanuc_client gpio_config
    EXPORT fanuc_libsExports
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(EXPORT fanuc_libsExports
    FILE
    fanuc_libsTargets.cmake
    NAMESPACE
    fanuc_libs::
    DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/fanuc_libs
)
install(DIRECTORY
    rmi/include/
    stream_motion/include/
    fanuc_client/include/
    gpio_config/include/
    DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/fanuc_libsConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/fanuc_libsConfigVersion.cmake
    DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/fanuc_libs
)
