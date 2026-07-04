# How to build a system like this, from scratch

This is a **tutorial**, not just a description. It walks through building a 3D
game-physics engine *and* the application architecture around it, in the order
you'd actually build them, and for every step it explains **what** to build and
**why**. The code in this repository is the reference implementation — each part
points at the files that realize it, so you can read the finished version
alongside the explanation.

You don't need to copy this project. You need to understand the *ideas*, because
they're the ones that make a physics engine correct, a codebase testable, and a
renderer swappable. By the end you'll know how to build your own.

**The one principle behind everything:** keep the simulation a *pure, deterministic
function*, and push all the messy real-world stuff (the clock, input, the GPU) to
the edges. Almost every decision below follows from that.

Contents:

- [Part 0 — Project setup: modules + CMake](#part-0)
- [Part 1 — The data model: math and an ECS](#part-1)
- [Part 2 — Making things move: the fixed timestep](#part-2)
- [Part 3 — Collision detection](#part-3)
- [Part 4 — Collision response: the solver](#part-4)
- [Part 5 — The application architecture (Point-Free / TCA)](#part-5)
- [Part 6 — Rendering (OpenGL, then Metal)](#part-6)
- [Part 7 — Testing a physics engine](#part-7)
- [Part 8 — Putting it together: demos](#part-8)
- [The recurring principles](#principles)

---

<a name="part-0"></a>
## Part 0 — Project setup: modules + CMake

**What to build:** a CMake project using **C++20 modules** and `import std`, split
into small libraries.

**Why modules.** Headers leak: macros, include order, and re-parsing everything.
A module exports only what you mark `export`; everything else is invisible to
importers. You get real encapsulation, faster builds (a module compiles to a
binary interface once), and no preprocessor surprises. `import std;` brings in the
whole standard library as one module — faster and cleaner than a pile of
`#include`s.

**How.** Each `.cppm` is a *module interface unit*. A tiny CMake helper declares
one:

```cmake
function(add_module_library name)
  add_library(${name})
  target_sources(${name} PUBLIC FILE_SET CXX_MODULES FILES ${ARGN})
  set_target_properties(${name} PROPERTIES CXX_MODULE_STD ON)  # enables import std
endfunction()
```

Two conventions worth adopting from the start:

- **Split declarations (`.cppm`) from heavy definitions (`.cpp`)** for anything
  substantial (see any `*Feature.cppm` + `.cpp`). Editing a body then recompiles
  one object, not every importer. It also dodges a real modules trap: an `inline`
  function *exported* from a `.cppm` must not reference an internal-linkage helper
  (like a lambda visitor), or you get a "TU-local entity exposure" error — putting
  the body in the `.cpp` avoids it.
- **Confine third-party headers to one implementation unit's global module
  fragment.** OpenGL headers live only in `GLRenderer.cpp`; metal-cpp only in
  `MetalRenderer.cpp`. They never meet `import std`.

**Split into many small libraries** (`CMakeLists.txt` has ~20). The point isn't
tidiness — it's that dependency edges become *compiler-enforced*. The physics
engine links nothing but the standard library, so if you accidentally call the
GPU from inside it, it won't link. Granularity is how a layered design stays
layered. Reference: [Architecture.md](Architecture.md).

> **Toolchain reality:** modules + `import std` need a recent LLVM (not Apple
> Clang) and CMake 4.x + Ninja. And keep *hot* module code simple: this
> pre-release toolchain has an `-O2`-only, layout-sensitive codegen bug (we hit it
> once with a hash-map broadphase; a plain sweep-and-prune dodged it).

---

<a name="part-1"></a>
## Part 1 — The data model: math and an ECS

**What to build:** value-type math, plain-data components, and a
structure-of-arrays world. Files: `PhysicsEngine-Math.cppm`,
`-Components.cppm`, `-World.cppm`.

**Start with math.** `Vec3`, a quaternion `Quat` (for orientation), and a 3×3
`Mat3` (for rotation and the inertia tensor). Make them **value types with
`operator==`** — this pays off hugely later (a whole world becomes comparable and
snapshottable). Store `Mat3` as three column vectors so a rotation matrix is
literally `[xAxis yAxis zAxis]`.

**Model bodies as an ECS, borrowing RealityKit's ideas.** An *entity* is just an
index. A *component* is a plain struct with no methods:

- `Transform` — position + orientation.
- `Collider` — the shape (sphere radius, or box half-extents).
- `Body` — `inverseMass`, inverse inertia, `restitution`, `friction`, and a
  `Dynamic`/`Static` type.
- `Motion` — linear + angular velocity.

The **`World`** is *structure-of-arrays*: one `std::vector` per component, indexed
by entity. Body 7's transform is `world.transforms[7]`.

**Why value types + SoA?**

- Value types make the whole `World` copyable and `==`-comparable. That's what
  lets it live inside an app's state, be snapshotted for interpolation, and be
  asserted against in tests. No hidden identity.
- SoA keeps each component contiguous — cache-friendly when a system sweeps "all
  velocities," and the natural shape for data-parallel updates later.

**Why store *inverse* mass?** Two reasons that recur everywhere:

1. A static body (wall, floor) is simply `inverseMass == 0` — "infinite mass."
   Impulses divided among bodies give it zero velocity change, *with no
   special-case code*.
2. The impulse math divides by mass; multiplying by a stored inverse is faster and
   makes "infinite mass = 0" fall out naturally (no divide-by-zero).

Derive mass and inertia from shape + density (`shapeMass`, `shapeInverseInertia`)
so a caller picks a size and material and the physics follows.

---

<a name="part-2"></a>
## Part 2 — Making things move: the fixed timestep

**What to build:** a `step(world, dt)` that integrates motion, and above it an
accumulator that runs it at a fixed rate. Files: `PhysicsEngine-Systems.cppm`
(integration), `Simulation.cppm` (the accumulator).

**Integrate with semi-implicit Euler:** update velocity first (`v += gravity·dt`),
then position with the *new* velocity (`x += v·dt`). Plain (explicit) Euler injects
energy and blows up oscillators; semi-implicit is one line different, same cost,
and approximately conserves energy. It's what every engine uses.

**Always integrate with a *fixed* `dt`.** This is non-negotiable: if `dt` varies
with frame rate, stiffness, bounciness, and tunnelling all become frame-rate
dependent and unstable. But screens refresh at variable rates. The fix is an
**accumulator** ("Fix Your Timestep"):

```
accumulator += realElapsedTime;      // however long the frame took
while (accumulator >= fixedDt) {      // fixedDt = 1/60, always
    previous = world;                 // snapshot one step back (see below)
    step(world, {dt = fixedDt});
    accumulator -= fixedDt;
}
alpha = accumulator / fixedDt;        // 0..1 leftover
```

**Then interpolate for rendering.** The renderer draws each body at
`lerp(previous, current, alpha)`, so motion is smooth even when the physics rate ≠
the display rate. The interpolation lives in the *renderer*; it never touches the
simulation.

> **Get the snapshot right.** Take `previous = world` *inside* the loop, right
> before each step, so after the loop `previous` is exactly **one** fixed step
> behind `current`. Snapshotting once *before* the loop is a classic bug: on a
> frame that runs several steps, `previous` ends up many steps behind while `alpha`
> only covers one, and everything visibly judders/"doubles." (We shipped that bug
> and a user caught it — there's now a regression test, `SimulationTests`.)

Crucially, `step` takes `dt` as a parameter and **never reads a clock**. All time
enters from outside. That's what keeps the simulation pure and deterministic.

---

<a name="part-3"></a>
## Part 3 — Collision detection

Detection has two stages, for a good reason: testing every pair of N bodies is
O(N²), but almost no pairs can touch.

**Broadphase — cheaply reject impossible pairs.** File:
`PhysicsEngine-Broadphase.cppm`. Compute each body's axis-aligned bounding box,
then use **Sweep-and-Prune**: sort bodies by their box's minimum x and, for each,
only compare the ones after it until one starts past its maximum x. O(n log n),
simple, no hash map. Keep a brute-force version as a **test oracle** — a test
asserts the two produce identical pair lists on random scenes, which proves the
fast path is correct *and* that swapping broadphases never changes the sim.

**Narrowphase — exact contact, as a *manifold*.** File:
`PhysicsEngine-Collision.cppm`. For each candidate pair, produce the one thing the
solver needs — the **contact manifold**: a unit `normal` (A→B), and 1–4 contact
points, each with a penetration depth.

- Sphere–sphere and sphere–box are closest-point tests.
- Box–box uses the **Separating Axis Theorem**: two boxes are disjoint iff some
  axis separates their projections (15 axes for two oriented boxes). If none does,
  the least-overlapping axis is the normal. To get *stable* contact, clip the
  incident face against the reference face (Sutherland–Hodgman) → up to 4 points.

**Why up to 4 points?** A box resting on the floor touches along a face. Resolve
one point and it pivots and jitters; resolve the whole face and it sits still.
Multi-point manifolds are why stacks are stable. Give each point a **stable id**
(from which features touch) — you'll need it for warm-starting.

---

<a name="part-4"></a>
## Part 4 — Collision response: the solver

This is the heart of the engine. File: `PhysicsEngine-Systems.cppm`. Build it in
layers; each layer is a real technique with a clear payoff.

**Layer 1 — impulses.** Don't compute contact *forces*; compute **impulses**
(instant momentum changes) and apply them to velocities. The impulse that cancels
the approach speed with bounce `e` is

```
j = -(1 + e) · vₙ / invMassSum        vₙ = relative normal velocity
```

`invMassSum` includes angular terms (hitting off-center makes a body spin). Apply
`+j·n` to B, `−j·n` to A. Because everything divides by `invMassSum`, heavy bodies
move less automatically and statics (inverse mass 0) don't move.

**Layer 2 — sequential impulses (Erin Catto).** Many contacts fight each other. So
loop over all contacts a few times, nudging each toward satisfying its own
constraint, accumulating the impulse per contact and **clamping it ≥ 0** (a
contact can only push, never pull). After a handful of iterations it converges.
This is how a pile of a hundred balls settles.

**Layer 3 — friction.** A second impulse along the surface tangent (two
perpendicular tangents in 3D), each clamped by the Coulomb limit
`|friction| ≤ μ·normalImpulse`. This stops sliding and lets piles heap up instead
of flowing.

**Layer 4 — warm-starting (the stack-stabilizer).** Store each contact's impulse
in a persistent cache (`PhysicsEngine-ContactCache.cppm`), keyed by body pair +
contact id. Next step, re-apply last step's impulse as a first guess. A resting
stack needs nearly the same impulses every frame, so the solver starts almost
right and converges in one iteration instead of visibly trembling. This is the
single biggest difference between "jittery" and "rock-solid."

**Layer 5 — sub-stepping.** Run the solve N times per frame at `h = dt/N` (default
4) instead of once at `dt`. Smaller steps = less penetration per step = stiffer,
more stable stacks and high mass ratios, for the same solver work. The Newton's
cradle visibly needs it (compare `subStepCount = 1` vs `4`).

**Layer 6 — speculative contacts (anti-tunnelling).** A fast body can skip through
a thin wall between frames. Instead of expensive continuous collision detection,
let the narrowphase emit contacts for bodies that are *near but not touching* (a
negative penetration = a gap), and have the solver allow the bodies to close *at
most that gap* per sub-step. A body screaming at a wall brakes exactly at the
surface. Cheap, and it reuses code you already have.

**Position correction, done safely.** Velocity impulses stop approach but leave
residual overlap. Fix it by nudging positions apart along the normal (a small
`slop` left uncorrected). Do it as a *position* change, not a velocity one —
otherwise you inject energy and stacks bounce forever.

> **A subtlety worth internalizing (we got it wrong first):** restitution must use
> the approach velocity captured **before** you apply gravity this sub-step.
> Sample it after, and an elastic ball leaves the floor faster than it arrived and
> slowly gains energy. Order matters. Regression test:
> `testElasticBounceConservesEnergyUnderGravity`.

Detail reference with the formulas: [PhysicsEngine.md](PhysicsEngine.md).

---

<a name="part-5"></a>
## Part 5 — The application architecture (Point-Free / TCA)

Now the *app* around the engine. We use a C++ port of Point-Free's **Composable
Architecture**. It rests on four ideas; adopt them and your game logic becomes as
testable as the engine.

**1. State + Action + reducer.** A feature is: a value-type **`State`** (holds
everything, including the physics `World`); an **`Action`** — a `std::variant` of
every event that can happen (`Ticked`, `SpawnBox`, `Detonate`, …); and a
**reducer** `body()` — a pure function `(State&, Action) → void` that is the *only*
place state changes. See any `*Feature.cppm`/`.cpp`.

**Why?** All state transitions funnel through one pure function. The app's whole
behavior is in one place, and it's testable without a window, GPU, or clock.

**2. The Store + a fixed frame loop.** The store owns the state and feeds it
actions. Every app's loop is the same four beats:

```cpp
for (auto a : inputToActions(events)) store.send(a);
store.send(Ticked{});     // advance the sim
store.pump();             // apply queued/background work
draw(store.state());      // read-only render
```

**3. Dependencies — inject the ambient stuff.** A pure reducer can't call
`clock()` or `rand()`. So it reads time through an injected `\.date` dependency and
randomness through a seeded generator. In production they're the real clock/RNG;
**in a test you swap them** for ones you control. This is why the whole app is
deterministic under test. Files: `Dependencies-*.cppm`.

**4. `TestStore` — exhaustive tests.** It runs a feature like the real store but
asserts the exact state after each action, and *fails if the reducer sends an
action the test doesn't consume*. The trick used throughout `tests/`: because the
reducer's effects are pure functions (`Sim::advance`, `addBox`), the test's
assertion just **replays the same function** on the expected state — so tests
verify the wiring without hard-coding physics numbers. The cradle test proves the
collision-event → action loop end to end this way.

**Composition** (`Scope`, `CasePath`) lets a big feature embed small ones — big
features assemble from small ones just like `State` assembles from small value
types. Reference: [Architecture.md](Architecture.md).

---

<a name="part-6"></a>
## Part 6 — Rendering (one interface, two backends)

**The rule:** only a *render backend* touches the GPU. Features and `SceneView`
speak in values; a backend turns them into draw calls. This is what lets you swap
or add backends without touching the simulation — which we literally do, shipping
**Metal and OpenGL side by side** behind one interface, chosen at runtime with
`--backend metal|gl`.

**The interface** (`src/Render/RenderBackend.hpp`): an abstract class —
`init(window)` → per-frame `beginFrame(size, camera, background)` →
`drawSphere`/`drawBox`/`drawPoints` → `endFrame()` (present) or
`captureFrame(png)` (offscreen read-back, for screenshots + headless tests). It
traffics only in plain `Render::` value types (`Vec3`, `Color`, `Camera`,
`RenderMath` matrices) — no `Physics`, no GPU types. `SceneView` (a module) is the
bridge from "a world of physics values" to backend-agnostic draw calls; it reads
*interpolated* poses (Part 2) and converts `Physics::Vec3`/`Mat3` to `Render::`
types at the boundary, so **the same demo draws on either backend unchanged**.

**Design it Metal-first.** The interface *records* a frame's draws so a backend
can flush them the way its GPU wants. That shape falls out of what makes Metal
fast — **instanced rendering** — and OpenGL then fills in the same contract:

- **`MetalRenderer`** (`metal-cpp`, Apple's C++ bindings) uploads all N per-instance
  transforms + colours to one `Shared` buffer and issues **one** instanced draw per
  shape kind — so the corridor's ~1400 balls are a single draw call. The vertex
  shader indexes the per-instance struct by `[[instance_id]]`. Shading is one pass:
  Lambert + Blinn–Phong specular + a fresnel rim + depth fog.
- **`GLRenderer`** (core OpenGL 4.1) fulfils the same `RenderBackend` contract with
  one shader and a shared cube + UV-sphere mesh, drawn per object — simple and
  fine for dozens to a few hundred bodies.

Key decisions that keep this clean:

- **Confine each GPU library to a plain, non-module library.** metal-cpp lives only
  in `MetalRenderer.cpp`, GL only in `GLRenderer.cpp`, each in the global module
  fragment. Neither is module-friendly, so the rest of the project (which uses
  `import std`) never sees a GPU header. `RenderBackendFactory.cpp` picks the
  implementation and makes the SDL window with matching flags.
- **metal-cpp ownership rule (bites everyone):** methods starting with
  `alloc`/`new`/`copy` return a +1 reference you must `release()`; *factory*
  methods like `texture2DDescriptor` return an **autoreleased** object you must
  **not** release, and each frame needs an `NS::AutoreleasePool`. (Over-releasing a
  factory descriptor is exactly the double-free we hit and fixed.)
- **`instanceCount` must be > 0.** Metal's validation layer asserts on a zero-count
  instanced draw — guard the batch so an empty frame (e.g. right after a reset)
  skips the draw instead of crashing. (Another bug we hit and fixed.)

Every scene (`CorridorFeature`, etc.) is engine-only and testable — the app just
draws its `World` through whichever backend. That separation is the whole point:
the flashy GPU demo reuses the same pure, tested engine as every other demo, and
the choice of GPU API is a one-flag runtime decision.

---

<a name="part-7"></a>
## Part 7 — Testing a physics engine

The highest-value tests for an engine aren't "does this function return X" — they
assert **physical laws over whole runs**. This is where a physics engine earns
trust. Files: `tests/`.

- **Primitive/manifold unit tests** — overlap and contact data vs. hand-computed
  values (`CollisionTests`).
- **Invariant / property tests** (the crown jewels): energy never *rises* with
  restitution < 1; momentum and energy are conserved in an elastic collision;
  resting bodies don't drift; fast bodies don't tunnel (`PhysicsEngineTests`,
  `SolverUpgradeTests`).
- **Determinism** — same inputs → *bit-identical* world, twice; and parallel
  narrowphase == serial (`testDeterminism`, `testThreadedMatchesSerial`).
- **Parity oracles** — the fast broadphase must equal the brute-force one exactly.
- **Feature tests via `TestStore`** — reducers under a pinned clock.
- **Containment/stress** — the corridor's hundreds of balls stay in the tube under
  sloshing gravity (`CorridorTests`).

And beyond automated tests: an **adversarial review** (have something actively try
to *disprove* correctness) caught two real bugs here — restitution energy
injection and a raycast phantom-hit — that ordinary tests missed. Both now have
regression tests. Physics bugs love to hide in plausible-looking code; make
something attack it.

---

<a name="part-8"></a>
## Part 8 — Putting it together: demos

Each demo is a thin feature + an app entry point, and each *exercises a specific
capability* so you can see it in isolation:

| Demo | Proves |
| --- | --- |
| Cubes | box–box SAT + manifolds |
| Ball Pit | warm-started resting stacks, many bodies |
| Newton's Cradle | constraints + restitution + the event→action loop |
| Explosions | force fields + speculative contacts |
| Raycast Car | raycasting + a spring-damper controller run *per fixed step* |
| Fireworks | CPU-parallel particles |
| Shooting Gallery | fast projectiles (speculative contacts, no tunnel) + FPS input |
| Metal Corridor | GPU instanced rendering + a many-body stress scene |

To add your own: write a `Feature` (State + Action + reducer over a `Sim::State`),
an app that runs the four-beat loop, and draw its `World`. The engine and renderer
are already done.

---

<a name="principles"></a>
## The recurring principles

If you take away only a handful of things, take these — they reappear at every
layer:

1. **Purity & determinism.** The simulation is a function of `(state, dt)` and
   never reads a clock. Same inputs → same result. Everything testable flows from
   this.
2. **One-way data flow.** input → action → reducer → state → render. Nothing skips
   a layer; nothing lower knows about anything higher.
3. **Inject the ambient dependencies** (time, randomness) so tests can control
   them.
4. **Value types with `==`** make state comparable, snapshottable, and testable.
5. **Push I/O to the edges.** Only the renderer touches the GPU; only the app
   touches SDL. The engine is pure C++.
6. **Confine third-party headers** to one implementation unit so they never
   pollute the rest.
7. **Keep math in textbook notation** (`rA`, `normal`, `e`, `h`) so the code lines
   up with the references — and spell out *everything else* in full.
8. **Attack your own correctness.** Invariant tests + adversarial review catch the
   bugs that plausible code hides.
