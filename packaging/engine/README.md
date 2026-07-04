# Physics Engine — portable core bundle

This is the rendering-free, deterministic core of the
[physics-demo](https://github.com/jaroshevskii/physics-demo) project, packaged as
a drop-in library: the `PhysicsEngine` module (SoA ECS, sub-stepped warm-started
sequential-impulse solver, sweep-and-prune broadphase, SAT box collision,
raycast, force fields, collision events, an ordered system scheduler) plus the
fixed-timestep `Simulation` driver. No SDL, no GPU, no app framework.

## Build

Requires a C++20-modules + `import std` toolchain (upstream LLVM/Clang, or MSVC
at C++23), CMake 4.x, and Ninja.

```sh
cmake -S . -B build -G Ninja \
  -DCMAKE_CXX_COMPILER=<clang++-with-modules> \
  -DCMAKE_CXX_STDLIB_MODULES_JSON=<path/to/libc++.modules.json>   # Clang/libc++
cmake --build build          # builds libPhysicsEngine + libSimulation
```

(On MSVC, run from a Visual Studio developer environment and omit the modules-json
flag — the toolset provides `import std`.)

## Use

```cpp
import PhysicsEngine;
import Simulation;

Sim::State sim;
Physics::addStaticBox(sim.world, {0, -1, 0}, {50, 1, 50}, 0.2f, 0.8f);
Physics::addSphere(sim.world, {0, 5, 0}, 0.5f, 1.0f, 0.4f, 0.5f);
sim.previous = sim.world;

// Fixed-timestep loop (feed it your frame's wall-clock time):
Sim::advance(sim, nowSeconds);
Physics::Vec3 p = Sim::interpolatedPosition(sim, /*entity*/ 1); // smooth render pose
```

See **UsingTheEngine.md** (bundled alongside this file) for the full API tour:
creating bodies, the fixed-timestep loop, tuning `StepContext`, raycasting, force
effects, joints, collision events, determinism, and gotchas.
