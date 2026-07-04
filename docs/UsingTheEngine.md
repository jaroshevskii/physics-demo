# Using the physics engine in your own game

This is a standalone, copy-paste guide to reusing the `PhysicsEngine` library on
its own — **without** SDL, OpenGL, or the Composable-Architecture app layer. The
engine is pure C++ with no dependency beyond the standard library, so it drops
into any renderer or game loop.

For *how it works internally*, see [PhysicsEngine.md](PhysicsEngine.md). This
document is just "how do I use it."

---

## 1. What you need

Only the `PhysicsEngine` module library. In your CMake:

```cmake
# however you vendor it — add the PhysicsEngine module target, then:
target_link_libraries(your_game PRIVATE PhysicsEngine)
```

In your code:

```cpp
import PhysicsEngine;   // brings in the whole engine under namespace Physics
using namespace Physics;
```

That's it. No SDL, no OpenGL, no `Simulation`/`ComposableArchitecture` targets —
those are the *demo's* app framework, not the engine.

---

## 2. The 30-second example

```cpp
import PhysicsEngine;
using namespace Physics;

int main() {
  World world;

  // A static floor (infinite mass): top surface at y = 0.
  addStaticBox(world, /*center*/ {0, -0.5f, 0}, /*halfExtents*/ {50, 0.5f, 50},
               /*restitution*/ 0.2f, /*friction*/ 0.6f);

  // A dynamic ball dropped from y = 5.
  Entity ball = addSphere(world, /*position*/ {0, 5, 0}, /*radius*/ 0.5f,
                          /*density*/ 1.0f, /*restitution*/ 0.5f, /*friction*/ 0.5f);

  StepContext context;                 // sensible defaults (dt = 1/60, gravity down)
  for (int frame = 0; frame < 300; ++frame) {
    step(world, context);              // advance one fixed step
    Vec3 p = world.transforms[ball].position;
    // ... hand p to your renderer ...
  }
}
```

`step` is a pure function of `(world, context)`. Call it, then read the
`transforms` back out to draw. That's the whole contract.

---

## 3. Creating bodies

Helpers in `:World` build a body's components (and derive its mass + inertia from
the shape + density) and return its `Entity` id:

```cpp
// Dynamic sphere. density sets the mass (mass = density · volume).
Entity addSphere(World&, Vec3 position, float radius, float density,
                 float restitution, float friction, Vec3 velocity = {});

// Dynamic box. Orientation is a quaternion (identity = axis-aligned).
Entity addBox(World&, Vec3 position, Vec3 halfExtents, float density,
              float restitution, float friction,
              Quat orientation = {}, Vec3 velocity = {});

// Immovable collider (walls, floors, ramps). Infinite mass.
Entity addStaticBox(World&, Vec3 position, Vec3 halfExtents,
                    float restitution, float friction, Quat orientation = {});
```

- **`restitution`** ∈ [0, 1]: 0 = no bounce (a beanbag), 1 = perfectly elastic
  (a superball). The pair's effective restitution is the *min* of the two bodies'.
- **`friction`** ≥ 0: Coulomb coefficient; the pair's is the geometric mean.
- **`density`** sets mass. A heavy body has a small `inverseMass` and shrugs off
  impacts; a light one gets flung. Static bodies have `inverseMass == 0`.

Orientations use quaternions; build one with `fromAxisAngle(axis, radians)`:

```cpp
addBox(world, {0, 3, 0}, {0.5f, 0.5f, 0.5f}, 1.0f, 0.2f, 0.5f,
       fromAxisAngle({0, 0, 1}, 0.4f));   // tilted 0.4 rad about Z
```

Everything you can read/write lives in the four parallel arrays, indexed by the
returned id: `world.transforms[id]`, `world.motions[id]` (`.linearVelocity`,
`.angularVelocity`), `world.bodies[id]`, `world.colliders[id]`.

---

## 4. The fixed-timestep loop (do this)

Your renderer runs at a variable frame rate, but the physics must always see the
same `dt`. Use an accumulator (this is what the demo's `Simulation` module does):

```cpp
const float fixedDt = 1.0f / 60.0f;
double accumulator = 0.0;
World previous = world;                 // for interpolation

// each frame:
accumulator += realFrameSeconds;        // however long the last frame took
if (accumulator >= fixedDt) {
  previous = world;                     // snapshot before stepping
  StepContext context;                  // dt defaults to 1/60
  while (accumulator >= fixedDt) {
    step(world, context);
    accumulator -= fixedDt;
  }
}
float alpha = float(accumulator / fixedDt);   // 0..1

// when drawing body i, interpolate for smooth motion:
Vec3 drawPos = previous.transforms[i].position
             + (world.transforms[i].position - previous.transforms[i].position) * alpha;
```

**Do not** just call `step` once per frame with `dt = realFrameSeconds` — variable
`dt` makes bounciness and stability frame-rate dependent and eventually blows up.

---

## 5. Tuning: the `StepContext`

`step` takes a `StepContext`. The defaults are good; reach for these when you need
to:

```cpp
StepContext context;
context.gravity = {0, -9.81f, 0};   // change or zero-out gravity
context.subStepCount = 4;           // more = stiffer/steadier stacks (cost: linear). 1 = fastest.
context.velocityIterations = 8;     // solver relaxation passes per sub-step
context.restitutionThreshold = 1.0f;// closing speed below which bounce is suppressed (kills rest jitter)
context.speculativeDistance = 0.02f;// anti-tunnel reach for near-contacts
context.threadCount = 8;            // parallelize narrowphase (result is identical to 1 thread)
```

Rules of thumb:

- Wobbly or sinking stacks → raise `subStepCount` (4 → 8) first, then
  `velocityIterations`.
- Fast objects tunnelling → raise `subStepCount` and/or `speculativeDistance`.
- Resting objects buzzing/micro-bouncing → that's what `restitutionThreshold` is
  for; the default already suppresses it.
- Lots of bodies → set `threadCount` to your core count; the result stays
  bit-identical to single-threaded.

---

## 6. Querying the world

### Raycasting (picking, line-of-sight, vehicle wheels)

```cpp
std::optional<RayHit> hit =
    raycastNearest(world, /*origin*/ {0, 10, 0}, /*direction*/ {0, -1, 0},
                   /*maxDistance*/ 20.0f, /*ignore*/ someEntityToSkip);
if (hit) {
  hit->entity;    // what was hit
  hit->position;  // world hit point
  hit->normal;    // surface normal
  hit->distance;  // along the ray
}
```

The optional `ignore` parameter skips one entity — useful when a body casts a ray
from inside itself (e.g. a vehicle chassis casting wheel rays; see the car demo).

### Applying force effects

```cpp
// An explosion at `center`: outward impulse to every dynamic body within radius,
// falling off with distance. Heavier bodies move less.
applyRadialImpulse(world, center, /*strength*/ 40.0f, /*radius*/ 8.0f, /*falloffExp*/ 1.0f);

// Global air drag over a timestep (stable; never overshoots past zero).
applyDrag(world, /*linear*/ 0.5f, /*angular*/ 0.5f, fixedDt);
```

Apply these *before* `step` in the same frame.

### Joints (ropes, pendulums, ragdoll-ish links)

```cpp
// Rigid rod (stiffness 0) or spring (stiffness > 0) between two bodies.
addDistanceConstraint(world, bodyA, bodyB, /*restLength*/ 2.0f,
                      /*stiffness*/ 0.0f /* 0 = rigid */);
```

Constraints live on `world.constraints` and are solved inside `step`. Chain
several to make a rope; anchor one end to a static body to make a pendulum.

### Collision events (sounds, gameplay triggers)

The engine records contacts frame-to-frame; diff two frames to get
Began/Persisted/Ended events:

```cpp
ContactCache before = world.contactCache;   // snapshot before stepping
step(world, context);
for (const CollisionEvent& e : collisionEvents(before, world.contactCache)) {
  if (e.phase == CollisionEvent::Phase::Began) {
    // e.a, e.b (the entities), e.position, e.normal, e.impulse (impact strength)
    playClinkSound(e.impulse);
  }
}
```

`e.impulse` scales with how hard they hit — great for impact-scaled sound or
particles.

---

## 7. Determinism (lockstep, replays, tests)

Because `step` never reads a clock and iterates in a fixed order, the same
`World` + the same sequence of `StepContext`s produces a **bit-identical** result
every run — even across thread counts. That means you get, for free:

- **Replays** — record the initial world + inputs, replay exactly.
- **Lockstep networking** — every client simulates the same steps and stays in
  sync (note: this is *run-to-run* determinism on one platform; the engine does
  not guarantee cross-CPU float identity, so it's not drop-in rollback-netcode).
- **Testable game logic** — assert exact world states, like the demo's tests do.

Keep *your* game logic clock-free too (feed it `fixedDt`, not wall time), and the
determinism extends to your whole simulation.

---

## 8. What to copy

- **Minimum:** the `PhysicsEngine` module (all its `.cppm` partitions) and its
  CMake `add_module_library(PhysicsEngine ...)` block. Nothing else.
- **Recommended:** also copy the tiny `Simulation` module for the ready-made
  fixed-timestep accumulator + interpolation (§4).
- **Optional:** the `RenderBackend` libraries (`GLRenderer` and/or
  `MetalRenderer`) + the `SceneView` module if you want cube/sphere/point drawing
  on either GPU backend to get on screen fast.

You do **not** need `ComposableArchitecture`, `Dependencies`, or the `*Feature`
libraries — those are the demo's app framework, not the engine.

---

## 9. Gotchas

- **Build toolchain.** The engine uses C++20 modules + `import std`, so it needs a
  recent Clang/MSVC and CMake 4.x + Ninja (see [Architecture.md](Architecture.md)).
  If you can't adopt modules yet, the engine's logic is header-portable in spirit,
  but as shipped it's modules-only.
- **Keep hot code simple** if you extend the engine on this pre-release toolchain
  (the `-O2` codegen quirk in [Architecture.md](Architecture.md#22-the-toolchain-constraints-and-why-the-cmake-looks-unusual)).
- **Static vs. dynamic:** a body you never want to move must be added with
  `addStaticBox` (inverse mass 0). A dynamic body with huge density is *not* the
  same — it can still be shoved.
- **Read positions after `step`,** not mid-step. During `step` the world passes
  through intermediate sub-step states.
