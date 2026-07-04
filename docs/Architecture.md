# Architecture

This guide explains **how the project is put together** — the build, the
libraries, and the application architecture — and, for every decision, *why* it
was made that way. Read it top to bottom; each section builds on the last.

If you only want to use the physics engine in your own game, skip to
[UsingTheEngine.md](UsingTheEngine.md). If you want to understand the physics
itself, read [PhysicsEngine.md](PhysicsEngine.md) next.

---

## 1. The big picture: one-way data flow, layered libraries

The whole project is a stack of layers, and **data only ever flows in one
direction** through them:

```
   ┌─────────────────────────────────────────────────────────────┐
   │  apps/*App.cpp        the executable: window, backend, loop    │
   │      │  reads input → makes Actions → send() → draw(state)     │
   │      ▼                                                         │
   │  *Feature             a "reducer": State + Action + body()     │  ← app logic
   │      │  (holds the physics World; asks \.date for the time)    │
   │      ▼                                                         │
   │  Simulation           fixed-timestep accumulator + interpolation│
   │      │                                                         │
   │      ▼                                                         │
   │  PhysicsEngine        pure, deterministic simulation (no I/O)  │  ← the engine
   │      ▲                                                         │
   │  SceneView            reads the World, emits backend-agnostic  │  ← rendering
   │      │                draw calls                               │
   │      ▼                                                         │
   │  RenderBackend        MetalRenderer ‖ GLRenderer (only GPU here)│
   └─────────────────────────────────────────────────────────────┘
```

**Why layer it like this?** Each layer has exactly one job and depends only on
the layers *below* it. That means:

- The **physics engine** knows nothing about SDL, OpenGL, or the app framework.
  You can test it, reuse it, or run it headless with zero graphics.
- The **app logic** (a feature) knows nothing about pixels. It's a pure function
  of "current state + an action → new state," so it's trivially unit-testable.
- Only a **render backend** touches a GPU API, so all the platform-specific code
  is quarantined in one place — and there are two interchangeable ones (Metal and
  OpenGL) behind a single interface.

This is the single most important idea in the codebase: **keep the simulation
pure and push all the messy I/O (clock, input, graphics) to the edges.**

---

## 2. The build: CMake + C++20 modules + `import std`

### 2.1 Why C++20 *modules* instead of headers

Every `.cppm` file is a **C++20 module interface unit**. Modules replace the old
`#include` model. The practical wins here:

- **No preprocessor leakage.** A module only exports what it says `export`. There
  are no macros bleeding across files, no include-order bugs, no header guards.
- **Faster, cleaner builds.** A module is compiled once into a binary interface
  (a `.pcm`), and importers read that instead of re-parsing text.
- **Real encapsulation.** Anything not marked `export` is invisible to importers,
  even if it's in a shared file.

We also use `import std;` — the entire standard library as one module — instead
of `#include <vector>`, `<cmath>`, etc. It's faster and it's the modern idiom.

### 2.2 The toolchain constraints (and why the CMake looks unusual)

Building modules + `import std` is still bleeding-edge, so the build is specific:

- **Homebrew LLVM Clang**, *not* Apple Clang. Apple's compiler supports neither
  C++20 modules nor `import std`. `CMakePresets.json` points `CMAKE_CXX_COMPILER`
  at `/opt/homebrew/opt/llvm/bin/clang++`.
- **CMake 4.x + Ninja.** `import std` is gated behind an experimental flag whose
  UUID (`CMAKE_EXPERIMENTAL_CXX_IMPORT_STD` in `CMakeLists.txt`) is tied to the
  CMake version. Ninja is required — the Makefiles/Xcode generators don't do
  module dependency scanning.
- **C++26 + extensions on.** The `std` module CMake builds for us is compiled in
  the default `gnu++` dialect, so our units must enable extensions to load the
  same `std.pcm`.

> **A real gotcha worth knowing.** At `-O2` (the default `RelWithDebInfo`) this
> pre-release toolchain has a *layout-sensitive codegen bug*: some module code
> can miscompile depending on unrelated code around it. We hit it once (a
> `std::unordered_map`-based broadphase) and it vanished under `-O3`, sanitizers,
> or a small code change. The lesson baked into the code: **keep hot module code
> simple** — which is why the broadphase is a plain sweep-and-prune, not a hash
> map. If you ever see a test flip pass/fail when you add an unrelated `print`,
> suspect this, not your logic.

### 2.3 The `add_module_library` helper

Declaring a module library by hand is verbose, so `CMakeLists.txt` defines one
helper and uses it everywhere:

```cmake
function(add_module_library name)
  add_library(${name})
  target_sources(${name} PUBLIC FILE_SET CXX_MODULES FILES ${ARGN})
  set_target_properties(${name} PROPERTIES CXX_MODULE_STD ON)
endfunction()
```

`FILE_SET CXX_MODULES` is how you tell CMake "these sources are module interface
units" (not ordinary `.cpp` files). `CXX_MODULE_STD ON` enables `import std`.

### 2.4 Interface units (`.cppm`) vs. implementation units (`.cpp`)

Some modules have **both** a `.cppm` and a `.cpp`, e.g. `SandboxFeature.cppm` +
`SandboxFeature.cpp`. The split follows one rule:

- The `.cppm` **declares** the public surface (types, function signatures).
- The `.cpp` (a *module implementation unit*, starting with `module SandboxFeature;`)
  **defines** the heavy or awkward bits.

Why bother? Two reasons:

1. **Build hygiene.** Editing the body of a reducer recompiles only that one
   `.cpp`; the module's binary interface is unchanged, so nothing downstream
   rebuilds.
2. **A modules pitfall.** An `inline` function *exported* from a `.cppm` must not
   reference an internal-linkage helper (like a lambda-based `overloaded`
   visitor). Doing so triggers a "TU-local entity exposure" error. Putting the
   reducer body in the `.cpp` sidesteps it entirely. If you ever hit that error,
   this is the fix.

---

## 3. Why the code is split into so many libraries

Open `CMakeLists.txt` and you'll see ~20 small libraries rather than one big
target. The dependency graph (arrows = "depends on"):

```
Dependencies ─────► ComposableArchitecture
                          ▲
PhysicsEngine ────────────┼────────────► Simulation ─► *Feature ─► (apps)
     │                    │                                 ▲
     └─► RenderBackend ─► SceneView ──────────────────────┘
         (GL + Metal)
```

The rules that produce this graph:

- **A library depends only on what it truly uses.** `PhysicsEngine` links nothing
  but the standard library — no SDL, no GL, no app framework. That's what makes it
  reusable and testable in isolation. With `-DPHYSICS_BUILD_APPS=OFF`, everything
  from `RenderBackend` rightward is dropped and the portable core still builds and
  tests (this is the Linux CI configuration).
- **`RenderBackend` is the *only* place a GPU API is linked.** `GLRenderer.cpp`
  includes OpenGL, `MetalRenderer.cpp` includes metal-cpp — each in its global
  module fragment — so a GPU header never leaks into `import std` or any module.
  Both are plain (non-module) libraries; `SceneView` (a module) draws through the
  abstract `RenderBackend` and never sees a GPU type.
- **Each demo is its own feature library + executable.** `CradleFeature`,
  `ExplosionFeature`, etc. are independent; they share the engine and render
  backend but not each other. Adding a demo can't break an existing one.

**Why so granular?** Small libraries make the dependency edges *explicit and
enforced by the compiler*. If someone tries to call OpenGL from inside the
physics engine, it simply won't link — the mistake is caught at build time, not
in code review. Granularity is how you keep a layered design from quietly rotting
into a tangle.

The engine itself is one library made of **module partitions** (`PhysicsEngine-Math.cppm`
is `export module PhysicsEngine:Math;`). Partitions are sub-modules of one module:
they can see each other's non-exported details, but the outside world only sees
the umbrella `PhysicsEngine` module. It's the C++ way to say "these files are one
cohesive unit with internal structure."

---

## 4. The application architecture: the Composable Architecture (TCA), in C++

The app layer is a C++ port of Point-Free's **Composable Architecture (TCA)** —
the same pattern the reference `fifteen-puzzle` project uses. It rests on four
ideas.

### 4.1 State, Action, and the reducer (`body()`)

A **feature** is three things:

- **`State`** — a plain value type holding *everything* the feature knows. For a
  demo, that's mostly the physics `World` plus a little UI state. Because it's a
  value type with `operator==`, you can copy it, compare it, and snapshot it.
- **`Action`** — a `std::variant` of every event that can happen: `Ticked`,
  `SpawnBox`, `TogglePause`, `Detonate`, `CollisionOccurred`, …. An action is
  *data describing something that happened*, never a function call.
- **`body()`** — the **reducer**: a pure function `(State&, Action) → void` that
  is the *only* place state changes. Given the current state and one action, it
  computes the next state.

```cpp
// Sketch of a reducer (see any *Feature.cpp)
std::visit(overloaded{
  [&](const Ticked&)      { Sim::advance(state.sim, date->now()); },
  [&](const SpawnBox& s)  { Physics::addBox(state.sim.world, s.position, ...); },
  [&](const TogglePause&) { state.sim.paused = !state.sim.paused; },
}, action);
```

**Why do it this way?** Because *all* state transitions funnel through one pure
function, the behavior of the entire app is (a) easy to reason about — there's
one place to look — and (b) **exhaustively testable** without a window, a GPU, or
a clock. That's section 4.4.

### 4.2 The Store and the run loop

The reducer is pure, but *something* has to actually hold the state, feed it
actions, and let background work talk back. That's the **`Store`**
(`RootStore` at runtime). The app's frame loop is always the same four beats:

```cpp
for (const Action& a : collectInput(...)) store.send(a); // input → actions
store.send(Ticked{});                                    // advance the sim
store.pump();                                            // apply queued work
draw(store.state());                                     // state → pixels
```

Nothing outside the store mutates state; the view only *reads* `store.state()`.

### 4.3 Dependencies: controlling time and randomness

A pure function can't call `clock()` or `rand()` — those are hidden inputs that
make behavior non-reproducible. The **Dependencies** library solves this: the
reducer reads the current time through an injected `\.date` dependency
(`DateGenerator`), never a global clock.

```cpp
Dependencies::Dependency<Dependencies::DateGeneratorKey> date;
// ... in the reducer:
Sim::advance(state.sim, date->now());
```

In production, `\.date` is the real monotonic clock. **In a test, you swap it for
a clock you control** and pin time to exact values. Same idea for randomness
(`\.withRandomNumberGenerator`, seeded in tests). This is *dependency injection*,
but for the ambient things (time, randomness) people usually forget to inject —
and it's the reason the whole app is deterministic under test.

### 4.4 Testing with `TestStore`

`TestStore` runs a feature exactly like the real store but lets a test **assert
the exact state after every action**. The pattern used throughout `tests/`:

```cpp
withDependencies([](auto& v){ v.context = test; /* pin the clock */ }, [] {
  TestStore<State, Action> store(initialState(), body);
  store.send(Ticked{}, [&](State& expected){ Sim::advance(expected.sim, now); });
  //                     ^ the assertion: replay the SAME pure function.
});
```

The clever bit: because the reducer's effects are pure functions (`Sim::advance`,
`addBox`, `initialState`), the test's assertion just **replays that same function**
on the expected state. So the test verifies the reducer wires things up correctly
*without hard-coding any physics numbers* — if the engine's math changes, the
test still holds, because both sides call the same code.

`TestStore` is also **exhaustive**: if a reducer sends an action (e.g. the cradle
re-emits a collision as a `CollisionOccurred`) and the test doesn't `receive()`
it, the test fails. That's how `CradleFeatureTests` proves the RealityKit-style
event → action loop end to end.

### 4.5 Composition (`Scope` / `CasePath`)

TCA features compose: a parent feature can embed a child by *scoping* into a slice
of its state and a case of its action. The `ComposableArchitecture` module ports
this (`Scope`, `CasePath`, `ifCaseLet`). The demos here are single-feature, so
they don't need it, but it's available and it's how you'd build a bigger app
(e.g. a menu feature that embeds the sandbox feature). The idea: **big features
are assembled from small ones the same way `State` is assembled from small value
types** — composition all the way down.

---

## 5. The Simulation layer: fixed timestep + interpolation

Between the feature and the engine sits one small but crucial module,
`Simulation`. It owns the **accumulator** that decouples the physics rate from
the (variable) frame rate — the classic "Fix Your Timestep" pattern:

```cpp
accumulator += realElapsedTime;           // however long the last frame took
while (accumulator >= fixedDt) {           // fixedDt = 1/60, always
    previousWorld = world;                 // save for interpolation
    Physics::step(world, context);         // one deterministic fixed step
    accumulator -= fixedDt;
}
alpha = accumulator / fixedDt;             // 0..1: how far into the next step
```

**Why?** The physics must always see the *same* `dt` (1/60 s), or stiffness,
bounciness, and tunnelling all become frame-rate dependent and unstable. But
displays refresh at wildly varying rates. The accumulator runs the sim in fixed
chunks and leaves a leftover `alpha`; the renderer uses `alpha` to *interpolate*
between the previous and current physics state, so motion looks smooth even when
the physics rate ≠ the screen rate. The renderer interpolates — it never touches
the simulation.

---

## 6. Rendering: two backends behind one interface

Rendering is split into three plain (non-module) libraries so no GPU header ever
meets `import std;`, and one module that bridges physics to draw calls:

- **`Render::RenderBackend`** (in `src/Render/RenderBackend.hpp`) is the abstract
  interface every demo draws through:
  `init` → per-frame `beginFrame` → `drawSphere`/`drawBox`/`drawPoints` →
  `endFrame` (present) *or* `captureFrame(png)` (offscreen read-back for
  screenshots / headless verification). It speaks only in plain `Render::` value
  types (`Vec3`, `Color`, `Camera`) — no `Physics` and no GPU types.
- **`MetalRenderer`** (metal-cpp) and **`GLRenderer`** (core OpenGL 4.1) are the
  two implementations. Each includes its GPU headers only in its own `.cpp`
  (metal-cpp / GL live in the *global module fragment*), so both stay ordinary C++
  libraries. `Render::makeRenderBackend(backend)` returns one behind a
  `unique_ptr`; `Render::createWindow(...)` makes an SDL window with the matching
  flags (`SDL_WINDOW_METAL` vs `SDL_WINDOW_OPENGL`).
- **`SceneView`** (a module) turns a `Sim::State` into backend-agnostic
  `RenderBackend` draw calls. It reads *interpolated* poses (section 5) so motion
  is smooth, colours dynamic bodies by id, and converts `Physics::Vec3`/`Mat3` to
  `Render::` types at this boundary. `drawWorld` draws bodies + a ground plane;
  `drawDynamicBodies` draws only the dynamic bodies (for fully-enclosed scenes
  like the corridor, whose walls should stay invisible).

The apps (`apps/*App.cpp`) own the SDL window and the frame loop, parse
`--backend metal|gl`, and translate input into feature actions. That's the only
place SDL lives. **The same demo runs on either backend** — a demo's draw code is
just `beginFrame(...)` → `SceneView::drawWorld(*renderer, sim)` → `endFrame()`.

**Metal-first, OpenGL-fills-in.** The interface is shaped the way Metal wants to
work: `SceneView` *records* a frame's spheres, boxes and points, and the backend
flushes them. `MetalRenderer` uploads the per-instance transforms once and issues
a single **instanced** draw per shape kind (so the corridor's ~1400 balls are one
draw call); `GLRenderer` fulfils the same contract by issuing a draw per object.
Adding a whole GPU backend without touching the simulation — or even the demos —
is the layered design paying off. See [BuildGuide.md](BuildGuide.md#part-6).

---

## 7. A note on naming

The code avoids abbreviations so it reads as prose: `context`, not `ctx`;
`collision`, not `col`; `normalImpulse`, not `nImp`. File names are PascalCase.

The **one exception** is physics math, where single-letter names mirror the
textbook formulas the code implements: `rA`/`rB` (contact vectors from each
center of mass), `normal`, `restitution` `e`, impulse `lambda`. Spelling those
out would make the code diverge from every reference you'd read alongside it, so
they're kept short *and* explained in comments. See
[PhysicsEngine.md](PhysicsEngine.md) for the notation.
