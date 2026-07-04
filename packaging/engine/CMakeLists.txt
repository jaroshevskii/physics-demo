# Drop-in build for the portable physics engine, shipped as a release artifact
# (see the Release CI). It builds ONLY the rendering-free core — the deterministic
# `PhysicsEngine` and the fixed-timestep `Simulation` driver — with no SDL, GPU,
# or app-framework dependencies. Drop this folder into your project (or add it
# with `add_subdirectory`) and link `PhysicsEngine` / `Simulation`.
#
# Requires a C++20-modules + `import std` toolchain (upstream LLVM/Clang, or MSVC
# at C++23), CMake 4.x, and Ninja — the same requirements as the full repo. See
# UsingTheEngine.md (bundled alongside this file) for API usage and examples.

cmake_minimum_required(VERSION 4.0)

# Enable experimental `import std;` support (gate UUID is CMake-version specific;
# this one matches CMake 4.2.x — keep it in sync with the toolchain).
set(CMAKE_EXPERIMENTAL_CXX_IMPORT_STD "d0edc3af-4c50-42ea-a356-e2862fe7a444")

project(PhysicsEngine LANGUAGES CXX)

if(MSVC)
  set(CMAKE_CXX_STANDARD 23) # CMake 4.2 supports neither C++26 nor import std at C++26 on MSVC
else()
  set(CMAKE_CXX_STANDARD 26)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# On Clang/GCC the `std` module is built with the default (gnu++NN) dialect, so
# our module units must enable extensions to load the same std.pcm.
if(MSVC)
  set(CMAKE_CXX_EXTENSIONS OFF)
else()
  set(CMAKE_CXX_EXTENSIONS ON)
endif()

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# Declare a C++20-module library from a list of module interface units.
function(add_module_library name)
  add_library(${name})
  target_sources(${name} PUBLIC FILE_SET CXX_MODULES FILES ${ARGN})
  set_target_properties(${name} PROPERTIES CXX_MODULE_STD ON)
endfunction()

# Globbed so the bundle never drifts from the shipped sources. CONFIGURE_DEPENDS
# re-globs on rebuild; module dependency order is resolved by CMake's scanner, so
# the file order here does not matter.
file(GLOB PHYSICS_ENGINE_MODULES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/PhysicsEngine/*.cppm")
add_module_library(PhysicsEngine ${PHYSICS_ENGINE_MODULES})

file(GLOB SIMULATION_MODULES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/Simulation/*.cppm")
add_module_library(Simulation ${SIMULATION_MODULES})
target_link_libraries(Simulation PUBLIC PhysicsEngine)
