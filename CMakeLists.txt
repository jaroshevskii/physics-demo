cmake_minimum_required(VERSION 4.0)

# Enable experimental `import std;` support (gate UUID is CMake-version specific;
# this one matches CMake 4.2.x).
set(CMAKE_EXPERIMENTAL_CXX_IMPORT_STD "d0edc3af-4c50-42ea-a356-e2862fe7a444")

project(PhysicsDemo LANGUAGES C CXX)

# Default to an optimized build when none is requested (matches the reference
# project; bare -O0 with this Clang + `import std` toolchain can miscompile
# module code). RelWithDebInfo keeps debug info for stepping through the solver.
get_property(_is_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT _is_multi_config AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
  message(STATUS "No CMAKE_BUILD_TYPE set; defaulting to RelWithDebInfo")
endif()

if(MSVC)
  set(CMAKE_CXX_STANDARD 23)
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

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# Helper: declare a C++20-module library from a list of module interface units.
function(add_module_library name)
  add_library(${name})
  target_sources(${name} PUBLIC FILE_SET CXX_MODULES FILES ${ARGN})
  set_target_properties(${name} PROPERTIES CXX_MODULE_STD ON)
endfunction()

# --- Reused architecture infrastructure (ported from the fifteen-puzzle repo) -

add_module_library(Dependencies
  src/Dependencies/Dependencies-Core.cppm
  src/Dependencies/Dependencies-DateGenerator.cppm
  src/Dependencies/Dependencies-RandomNumberGenerator.cppm
  src/Dependencies/Dependencies.cppm
)

add_module_library(ComposableArchitecture
  src/ComposableArchitecture/ComposableArchitecture-CasePath.cppm
  src/ComposableArchitecture/ComposableArchitecture-Store.cppm
  src/ComposableArchitecture/ComposableArchitecture-Feature.cppm
  src/ComposableArchitecture/ComposableArchitecture-Runtime.cppm
  src/ComposableArchitecture/ComposableArchitecture-Scope.cppm
  src/ComposableArchitecture/ComposableArchitecture-Navigation.cppm
  src/ComposableArchitecture/ComposableArchitecture-TestStore.cppm
  src/ComposableArchitecture/ComposableArchitecture.cppm
)
target_link_libraries(ComposableArchitecture PUBLIC Dependencies)

# --- The physics engine (pure, deterministic, no rendering) -------------------

add_module_library(PhysicsEngine
  src/PhysicsEngine/PhysicsEngine-Math.cppm
  src/PhysicsEngine/PhysicsEngine-Components.cppm
  src/PhysicsEngine/PhysicsEngine-ContactCache.cppm
  src/PhysicsEngine/PhysicsEngine-World.cppm
  src/PhysicsEngine/PhysicsEngine-Collision.cppm
  src/PhysicsEngine/PhysicsEngine-Broadphase.cppm
  src/PhysicsEngine/PhysicsEngine-Threading.cppm
  src/PhysicsEngine/PhysicsEngine-Systems.cppm
  src/PhysicsEngine/PhysicsEngine-Query.cppm
  src/PhysicsEngine/PhysicsEngine-Forces.cppm
  src/PhysicsEngine/PhysicsEngine-Events.cppm
  src/PhysicsEngine/PhysicsEngine-Scheduler.cppm
  src/PhysicsEngine/PhysicsEngine.cppm
)

# --- Simulation driver + feature reducers (pure, no rendering) ----------------

add_module_library(Simulation src/Simulation/Simulation.cppm)
target_link_libraries(Simulation PUBLIC PhysicsEngine)

# --- Feature reducers (pure logic; no rendering). Always built — the tests link
#     them, and they must compile on every platform, including headless CI. ------

function(add_feature name dir)
  add_module_library(${name} src/Features/${dir}/${name}.cppm)
  target_sources(${name} PRIVATE src/Features/${dir}/${name}.cpp)
  target_link_libraries(${name} PUBLIC ComposableArchitecture Dependencies PhysicsEngine Simulation)
endfunction()

add_feature(SandboxFeature Sandbox)
add_feature(BallPitFeature BallPit)
add_feature(CradleFeature Cradle)
add_feature(ExplosionFeature Explosion)
add_feature(CarFeature Car)
add_feature(FireworksFeature Fireworks)
add_feature(ShooterFeature Shooter)
add_feature(CorridorFeature Corridor)

# --- Windowed demo apps (SDL3 + a GPU backend) --------------------------------
# Everything above is portable and headless-testable. The apps need SDL3, a GPU
# renderer, and a display, so they are opt-out: Linux/headless CI configures with
# -DPHYSICS_BUILD_APPS=OFF and still builds + tests the whole portable core.
option(PHYSICS_BUILD_APPS "Build the windowed demo apps (needs SDL3 + a GPU backend)" ON)

if(PHYSICS_BUILD_APPS)
  # SDL3 for windowing + input. Resolved system-first (brew/vcpkg), falling back
  # to a pinned source build so a fresh checkout still builds anywhere.
  find_package(SDL3 QUIET)
  if(NOT SDL3_FOUND)
    message(STATUS "SDL3: building from source")
    include(FetchContent)
    FetchContent_Declare(SDL3
      GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
      GIT_TAG release-3.2.10
      GIT_SHALLOW TRUE)
    set(SDL_TEST OFF CACHE BOOL "" FORCE)
    set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(SDL3)
  else()
    message(STATUS "SDL3: using system package")
  endif()

  find_package(OpenGL REQUIRED)

  # RenderBackend: the abstract renderer interface + its OpenGL implementation.
  # A plain (non-module) library so it can share one interface with the Metal
  # backend; GL headers are confined to its .cpp.
  add_library(RenderBackend STATIC
    src/Render/RenderMath.cpp
    src/Render/GLRenderer.cpp
    src/Render/RenderBackendFactory.cpp)
  target_include_directories(RenderBackend PUBLIC src/Render)
  target_link_libraries(RenderBackend PUBLIC OpenGL::GL z SDL3::SDL3) # z: PNG capture; SDL: window/present

  # SceneView: draws a Sim::State through the abstract RenderBackend.
  add_module_library(SceneView src/SceneView/SceneView.cppm)
  target_link_libraries(SceneView PUBLIC PhysicsEngine Simulation RenderBackend)

  # The Metal backend (macOS): metal-cpp instanced rendering, confined to a plain
  # library so it never meets `import std`. It implements the same RenderBackend.
  option(PHYSICS_ENABLE_METAL "Build the Metal rendering backend (macOS only)" ${APPLE})
  if(PHYSICS_ENABLE_METAL AND APPLE)
    include(FetchContent)
    FetchContent_Declare(metal_cpp
      GIT_REPOSITORY https://github.com/bkaradzic/metal-cpp.git
      GIT_TAG metal-cpp_macOS14.2_iOS17.2
      GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(metal_cpp)
    target_sources(RenderBackend PRIVATE src/Render/MetalRenderer.cpp)
    target_include_directories(RenderBackend PRIVATE ${metal_cpp_SOURCE_DIR})
    target_compile_definitions(RenderBackend PUBLIC PHYSICS_HAS_METAL=1)
    target_link_libraries(RenderBackend PUBLIC
      "-framework Metal" "-framework Foundation" "-framework QuartzCore")
    message(STATUS "Metal backend: enabled")
  endif()

  # One executable per demo. Each runs on either backend via --backend metal|gl.
  function(add_demo_app exe src feature)
    add_executable(${exe} apps/${src})
    set_target_properties(${exe} PROPERTIES CXX_MODULE_STD ON)
    target_link_libraries(${exe} PRIVATE
      ${feature} ComposableArchitecture Dependencies PhysicsEngine Simulation
      RenderBackend SceneView SDL3::SDL3)
  endfunction()

  add_demo_app(sandbox   SandboxApp.cpp   SandboxFeature)
  add_demo_app(ballpit   BallPitApp.cpp   BallPitFeature)
  add_demo_app(cradle    CradleApp.cpp    CradleFeature)
  add_demo_app(explosion ExplosionApp.cpp ExplosionFeature)
  add_demo_app(car       CarApp.cpp       CarFeature)
  add_demo_app(fireworks FireworksApp.cpp FireworksFeature)
  add_demo_app(shooter   ShooterApp.cpp   ShooterFeature)
  add_demo_app(corridor  CorridorApp.cpp  CorridorFeature)
endif()

# --- Tests --------------------------------------------------------------------
# EXCLUDE_FROM_ALL so a normal build never recompiles them.

enable_testing()

add_executable(CollisionTests EXCLUDE_FROM_ALL tests/CollisionTests.cpp)
set_target_properties(CollisionTests PROPERTIES CXX_MODULE_STD ON)
target_link_libraries(CollisionTests PRIVATE PhysicsEngine)
add_test(NAME CollisionTests COMMAND CollisionTests)

add_executable(PhysicsEngineTests EXCLUDE_FROM_ALL tests/PhysicsEngineTests.cpp)
set_target_properties(PhysicsEngineTests PROPERTIES CXX_MODULE_STD ON)
target_link_libraries(PhysicsEngineTests PRIVATE PhysicsEngine)
add_test(NAME PhysicsEngineTests COMMAND PhysicsEngineTests)

add_executable(SolverUpgradeTests EXCLUDE_FROM_ALL tests/SolverUpgradeTests.cpp)
set_target_properties(SolverUpgradeTests PROPERTIES CXX_MODULE_STD ON)
target_link_libraries(SolverUpgradeTests PRIVATE PhysicsEngine)
add_test(NAME SolverUpgradeTests COMMAND SolverUpgradeTests)

add_executable(BroadphaseTests EXCLUDE_FROM_ALL tests/BroadphaseTests.cpp)
set_target_properties(BroadphaseTests PROPERTIES CXX_MODULE_STD ON)
target_link_libraries(BroadphaseTests PRIVATE PhysicsEngine)
add_test(NAME BroadphaseTests COMMAND BroadphaseTests)

add_executable(EngineFeaturesTests EXCLUDE_FROM_ALL tests/EngineFeaturesTests.cpp)
set_target_properties(EngineFeaturesTests PROPERTIES CXX_MODULE_STD ON)
target_link_libraries(EngineFeaturesTests PRIVATE PhysicsEngine)
add_test(NAME EngineFeaturesTests COMMAND EngineFeaturesTests)

add_executable(SchedulerTests EXCLUDE_FROM_ALL tests/SchedulerTests.cpp)
set_target_properties(SchedulerTests PROPERTIES CXX_MODULE_STD ON)
target_link_libraries(SchedulerTests PRIVATE PhysicsEngine)
add_test(NAME SchedulerTests COMMAND SchedulerTests)

add_executable(CradleFeatureTests EXCLUDE_FROM_ALL tests/CradleFeatureTests.cpp)
set_target_properties(CradleFeatureTests PROPERTIES CXX_MODULE_STD ON)
target_link_libraries(CradleFeatureTests PRIVATE CradleFeature ComposableArchitecture PhysicsEngine Simulation)
add_test(NAME CradleFeatureTests COMMAND CradleFeatureTests)

add_executable(DemoTests EXCLUDE_FROM_ALL tests/DemoTests.cpp)
set_target_properties(DemoTests PROPERTIES CXX_MODULE_STD ON)
target_link_libraries(DemoTests PRIVATE CarFeature ExplosionFeature FireworksFeature PhysicsEngine Simulation)
add_test(NAME DemoTests COMMAND DemoTests)

add_executable(SimulationTests EXCLUDE_FROM_ALL tests/SimulationTests.cpp)
set_target_properties(SimulationTests PROPERTIES CXX_MODULE_STD ON)
target_link_libraries(SimulationTests PRIVATE Simulation PhysicsEngine)
add_test(NAME SimulationTests COMMAND SimulationTests)

add_executable(CorridorTests EXCLUDE_FROM_ALL tests/CorridorTests.cpp)
set_target_properties(CorridorTests PROPERTIES CXX_MODULE_STD ON)
target_link_libraries(CorridorTests PRIVATE CorridorFeature PhysicsEngine Simulation)
add_test(NAME CorridorTests COMMAND CorridorTests)

add_executable(ShooterFeatureTests EXCLUDE_FROM_ALL tests/ShooterFeatureTests.cpp)
set_target_properties(ShooterFeatureTests PROPERTIES CXX_MODULE_STD ON)
target_link_libraries(ShooterFeatureTests PRIVATE ShooterFeature ComposableArchitecture PhysicsEngine Simulation)
add_test(NAME ShooterFeatureTests COMMAND ShooterFeatureTests)

add_executable(SandboxFeatureTests EXCLUDE_FROM_ALL tests/SandboxFeatureTests.cpp)
set_target_properties(SandboxFeatureTests PROPERTIES CXX_MODULE_STD ON)
target_link_libraries(SandboxFeatureTests PRIVATE SandboxFeature ComposableArchitecture PhysicsEngine Simulation)
add_test(NAME SandboxFeatureTests COMMAND SandboxFeatureTests)

add_executable(BallPitFeatureTests EXCLUDE_FROM_ALL tests/BallPitFeatureTests.cpp)
set_target_properties(BallPitFeatureTests PROPERTIES CXX_MODULE_STD ON)
target_link_libraries(BallPitFeatureTests PRIVATE BallPitFeature ComposableArchitecture PhysicsEngine Simulation)
add_test(NAME BallPitFeatureTests COMMAND BallPitFeatureTests)

# Timing micro-benchmark for the solver hot path (not a ctest — machine-dependent).
add_executable(PerfBench EXCLUDE_FROM_ALL tests/PerfBench.cpp)
set_target_properties(PerfBench PROPERTIES CXX_MODULE_STD ON)
target_link_libraries(PerfBench PRIVATE PhysicsEngine)
