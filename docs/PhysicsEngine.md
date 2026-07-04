# How the physics engine works

This is a guided tour of `src/PhysicsEngine/` — what each piece does, the math
behind it, and **why** each design choice was made. It assumes you've skimmed
[Architecture.md](Architecture.md) (so you know the engine is a pure, deterministic
library with no I/O). To *use* the engine without this much detail, see
[UsingTheEngine.md](UsingTheEngine.md).

The engine is one module (`PhysicsEngine`) made of partitions, in dependency
order:

```
Math → Components → ContactCache → World → Collision → Broadphase
     → Threading → Systems(solver) → Query → Forces → Events → Scheduler
```

---

## 1. The data model: an ECS of value types

The design borrows the *ideas* of Apple's **RealityKit** Entity-Component-System,
in plain C++:

- An **entity** is just an index (`using Entity = std::size_t`). No object, no
  vtable — a body is body number 7.
- A **component** is a plain data struct, no methods (`:Components`):
  - `Transform` — position + orientation (a quaternion).
  - `Collider` — the shape: a sphere (radius) or a box (half-extents).
  - `Body` — mass/material: `inverseMass`, `inverseInertiaBody`, `restitution`
    (bounciness), `friction`, and a `BodyType` (Dynamic or Static).
  - `Motion` — `linearVelocity` + `angularVelocity`.
- The **`World`** (`:World`) is the store. It's **structure-of-arrays**: one
  `std::vector` per component (`transforms`, `colliders`, `bodies`, `motions`),
  all indexed by entity. Body 7's transform is `world.transforms[7]`.

**Why value types and structure-of-arrays?**

- **Value types** mean the whole `World` is copyable and `operator==`-comparable.
  That's what lets it live inside a Composable-Architecture `State`, be snapshotted
  for interpolation, and be asserted against in tests. No hidden identity, no
  aliasing surprises.
- **Structure-of-arrays** keeps each component contiguous in memory, which is
  cache-friendly when a system sweeps over "all velocities" or "all transforms."
  It's also the natural shape for data-parallel updates.

The RealityKit parallel: their `Transform` / `PhysicsBodyComponent` /
`PhysicsMotionComponent` / `CollisionComponent` map almost one-to-one onto our
`Transform` / `Body` / `Motion` / `Collider`. Their **systems** (pure functions
over a query + a per-frame context) map onto our free functions over the `World`
+ a `StepContext`.

### Inverse mass and inertia (why "inverse"?)

`Body` stores `inverseMass` (1/m) and `inverseInertiaBody` (the inverse inertia
tensor), not mass and inertia. Two reasons:

1. **Static bodies are just `inverseMass == 0`.** A wall has "infinite mass"; its
   inverse mass is zero, so any impulse divided among the bodies gives it zero
   velocity change — *with no special-case code*. The floor, walls, and cradle
   anchors are all `Static` this way.
2. The impulse formulas *divide* by mass, and dividing by `inverseMass` would be a
   divide-by-zero for statics. Multiplying by a stored inverse is both faster and
   the natural way to express "infinite mass = 0."

Mass and inertia are derived from a shape + a density in `:Components`
(`shapeMass`, `shapeInverseInertia`) so you specify a size and material, and the
physically-correct mass/inertia follow (solid-sphere `I = 2/5 m r²`, solid-box
`I = 1/12 m (·)`).

---

## 2. `step()` — the pipeline, and the fixed timestep

Everything happens in one function, `Physics::step(World&, StepContext)`. The
`StepContext` is the per-frame input (RealityKit's `SceneUpdateContext`): the
timestep `dt`, `gravity`, and solver tuning knobs. **`dt` is always passed in —
the engine never reads a clock** — which is what makes `step` a pure,
deterministic function of `(world, context)`.

`step` runs `subStepCount` sub-steps, each at `h = dt / subStepCount`:

```
step(world, context):
  for i in 0 .. subStepCount:            # default 4
    subStep(world, context with dt = h)
```

Each `subStep` is the classic rigid-body pipeline:

```
1. build contacts   broadphase → narrowphase → prepare (capture approach speed)
2. integrate velocities   (apply gravity)          ← AFTER (1); see §6 restitution
3. warm-start       re-apply last step's contact impulses
4. solve velocity   sequential impulses: normal + friction
5. solve constraints joints (distance/spring)
6. restitution      the bounce, as a separate pass
7. integrate positions   move bodies by their new velocities
8. position correction    gently push apart residual overlap
9. store impulses   save them for next frame's warm-start
```

The rest of this document is those stages, in order.

---

## 3. Broadphase: find *candidate* pairs cheaply (`:Broadphase`)

With N bodies there are N² possible pairs, but almost none can actually touch.
The broadphase quickly discards the impossible ones so the expensive narrowphase
only runs on plausible pairs.

We use **Sweep-and-Prune**: compute each body's axis-aligned bounding box (AABB),
sort the bodies by their box's minimum x, then walk the list — for each body, only
compare against the following bodies until one *starts* past the current body's
maximum x (they can't overlap on x, and since the list is x-sorted, neither can
anything after it). Overlapping candidates go into the pair list, which is then
sorted and de-duplicated.

**Why sweep-and-prune?** It's `O(n log n)` instead of `O(n²)`, it's simple, and
it has no hash map or lambda-heavy machinery — which matters because of the
toolchain codegen bug mentioned in [Architecture.md](Architecture.md#22-the-toolchain-constraints-and-why-the-cmake-looks-unusual).
A brute-force version (`bruteForceBroadphase`) is kept as a **test oracle**: a
test asserts the two produce byte-identical pair lists on random scenes, which
proves the fast path is correct *and* that swapping broadphases never changes the
simulation.

AABBs are fattened by a small margin (`aabbMargin`) so pairs are found a frame
early — this feeds the speculative-contact anti-tunnelling in §5/§6.

---

## 4. Narrowphase: exact contact, as a *manifold* (`:Collision`)

For each candidate pair the narrowphase computes the **contact manifold** — the
one piece of data the solver needs:

```cpp
struct Collision {
  Vec3 normal;                        // unit, points from body A to body B
  std::array<ContactPoint, 4> contacts;
  int count;
};
struct ContactPoint {
  Vec3 position;      // where they touch, in world space
  float penetration;  // >0 = overlapping this deep; <0 = a small gap (speculative)
  std::uint16_t id;   // a stable name for this point (for warm-starting, §6)
};
```

Three shape pairs are handled:

- **Sphere–sphere** — trivial: they touch if the distance between centers is less
  than the sum of radii; the normal is the line between centers.
- **Sphere–box** — find the closest point on the box to the sphere's center; if
  it's within the radius, that's the contact.
- **Box–box** — the hard one, done with the **Separating Axis Theorem (SAT)**:
  two convex boxes are disjoint if and only if some axis exists on which their
  projections don't overlap. For two oriented boxes there are 15 axes to test (3
  faces of A, 3 of B, 9 edge-edge cross products). If any axis separates them,
  there's no collision. Otherwise the axis of *least* overlap is the contact
  normal. To get stable **multiple** contact points (needed so a resting box
  doesn't rock on a single point), the incident face is **clipped** against the
  reference face (Sutherland–Hodgman clipping), yielding up to 4 contact points.

**Why a manifold with up to 4 points?** A box resting flat on the floor touches
along a whole face. If you only resolved one contact point, the box would pivot
and jitter. Multiple points let the solver hold the face down evenly. This is why
stacks of boxes are stable and stacks-with-one-contact aren't.

**Contact ids.** Each point gets a small integer `id` derived from *which*
features touch (which reference face, which clipped vertex). The id is stable
frame-to-frame, which is what lets warm-starting (§6) match "this point" to "the
same point last frame."

---

## 5. Speculative contacts (anti-tunnelling, CCD-lite)

A fast body can pass entirely through a thin wall in one step ("tunnelling") if it
never overlaps on the exact frame we check. Full continuous collision detection
(sweeping shapes through time) is expensive. Instead we use **speculative
contacts** (the modern Box2D-v3 approach):

- The narrowphase emits a contact even when shapes are *near but not touching* —
  up to `speculativeDistance` apart. Such a point has **negative penetration**
  (i.e. a positive *gap*).
- The solver (§6) then allows the bodies to close *at most that gap* in the
  sub-step. A body screaming toward a wall gets braked exactly at the surface
  instead of flying through.

**Why this instead of real CCD?** It's a few lines in code you already have (the
narrowphase and solver), it's cheap, and it handles the common case (moderate
speeds) well. It won't stop a bullet through paper — that needs time-of-impact
sweeps — but combined with sub-stepping (smaller `h` = smaller travel per step)
it covers everything these demos throw at it. There's a test that fires a sphere
at 120 m/s at a thin wall and asserts it never crosses.

---

## 6. The solver — where the real physics lives (`:Systems`)

This is the heart of the engine. The goal: given the contacts, adjust every
body's velocity so that things stop interpenetrating, bounce correctly, and rub
with friction — all while conserving momentum and never inventing energy.

### 6.1 Impulses, not forces

We don't compute forces and accelerations at contacts; we compute **impulses**
(instantaneous changes in momentum) and apply them directly to velocities. For a
single contact with normal `n`, the impulse magnitude that exactly cancels the
approaching velocity is:

```
        -(1 + e) · vₙ
  j =  ───────────────      where  vₙ = (v_B − v_A) · n   (relative normal speed)
        invMassSum          e = restitution (bounciness)
```

`invMassSum` is `inverseMassA + inverseMassB + (angular terms)`. The angular
terms account for the fact that hitting a body off-center makes it *spin* rather
than just translate; they involve the contact's lever arms `rA`, `rB` (vectors
from each body's center of mass to the contact point) and the inverse inertia
tensors. Apply `+j·n` to B and `−j·n` to A, plus the matching angular impulses.

Because everything is divided by `invMassSum`, **heavy bodies move less
automatically** and a static body (inverse mass 0) doesn't move at all.

### 6.2 Sequential impulses (solving many contacts at once)

Real scenes have many contacts that fight each other (a box in a corner touches
two walls). We use Erin Catto's **sequential impulses**: loop over all contacts a
few times (`velocityIterations`), each time nudging the impulse toward satisfying
*that* contact. The impulse per contact is **accumulated and clamped ≥ 0** (a
contact can only *push*, never pull), and re-applied as a delta each iteration.
After a handful of iterations the whole system converges to a consistent set of
impulses. This is how a pile of a hundred balls settles into a stable heap.

### 6.3 Friction (two tangents)

Friction is a second impulse, along the contact *tangent* (the surface plane). In
3D the tangential direction can be anything in that plane, so we build two
perpendicular tangent directions and solve friction along each. Each tangent
impulse is clamped by the **Coulomb limit** `|friction| ≤ μ · normalImpulse` — you
can't have more friction than the normal force allows. This is what stops a
sliding box, lets the ball pit pile up instead of flowing like liquid, and gives
the car grip.

### 6.4 Warm-starting (why stacks don't jitter)

Here's the trick that makes tall stacks *stable*. At the end of each step we
**store** the impulse we found at each contact into a persistent
`ContactCache` (`:ContactCache`), keyed by the body pair and the contact `id`
(§4). Next step, before solving, we **re-apply** last step's impulse as a first
guess ("warm start"). Since a resting stack needs almost the same impulses every
frame, the solver starts from nearly the right answer and converges in one or two
iterations instead of slowly and visibly settling.

Without warm-starting, a stack of 5 boxes visibly trembles and slowly sinks. With
it, it sits rock-still. There's a test (`testBoxStackRests`) that asserts a 5-box
stack settles and stays put with near-zero residual velocity.

### 6.5 Sub-stepping (why we run 4 small steps, not 1 big one)

Instead of one solve at `dt = 1/60`, we run **`subStepCount` (default 4) solves
at `h = dt/4`**. Smaller steps mean less penetration to fix per step, stiffer
effective contacts, and far more stability for tall stacks, high mass ratios, and
fast motion — for a fixed amount of solver work. It's the single biggest quality
lever; the cradle visibly degrades at `subStepCount = 1` and works at 4 (you can
A/B it by changing the knob).

### 6.6 Restitution done right (a bug we caught, and why the order matters)

Restitution (the bounce) is applied as a **separate pass** after the normal
solve, reversing the approach velocity by `e`. Subtlety: the "approach velocity"
must be captured **before gravity is applied this sub-step.** Originally we
integrated gravity first and *then* recorded the approach speed — so an `e = 1`
ball left the floor faster than it arrived (it kept `g·h` of extra speed each
bounce) and slowly gained energy. An adversarial code review caught it; the fix
was to record each contact's approach velocity *before* the gravity integration
(see the comment in `subStep`). There's now a regression test
(`testElasticBounceConservesEnergyUnderGravity`) that drops an elastic ball under
gravity and asserts its total energy never grows. **Lesson: with restitution,
sample the incoming speed before you touch the velocity for anything else.**

### 6.7 Position correction (fixing sink without adding energy)

Velocity impulses stop bodies from *approaching*, but a body can still be slightly
overlapping from a previous step. We fix that with a small **positional
projection**: push the pair apart along the normal by a fraction of the excess
penetration (leaving a tiny `slop` uncorrected to avoid jitter). Crucially this
moves *positions*, not velocities, so it removes overlap **without injecting
energy** — a naive "just add a velocity to separate them" approach would make
stacks bounce forever.

---

## 7. Determinism and parallelism (`:Threading`)

The engine is **deterministic**: same world + same `dt` sequence → *bit-identical*
result, every run. This requires discipline — no wall-clock reads, no
`unordered_map` iteration reaching an output, fixed iteration order over contacts.
A test (`testDeterminism`) runs a scene twice and asserts the two worlds are
exactly equal.

Narrowphase is embarrassingly parallel (each pair's manifold is independent), so
`step` computes them with `parallelFor` when `context.threadCount > 1`. The key
to keeping it deterministic: each worker writes only `results[i]` (its own slot),
and the solver consumes them in index order — so the output is **identical whether
you use 1 thread or 8**. A test asserts exactly that. The solver itself, which has
cross-body coupling, stays single-threaded on purpose. The fireworks demo runs its
particle update through the same `parallelFor`.

### 7.1 The allocation-free hot path

A real-time simulation must not allocate on the heap every step — malloc jitter
shows up as frame-time spikes. So a steady simulation here allocates **nothing**
per step: every per-substep scratch buffer (broadphase pairs + AABBs, narrowphase
results, the hit manifolds, the solver contacts, and the next warm-start cache)
lives in a single reused `StepWorkspace`, and box-face clipping runs on
fixed-size stack buffers (`std::array<Vec3, 8>`) instead of `std::vector`.

The subtle part is *where* the workspace lives. It cannot be a member of `World`:
`World` is a value type, copied every frame for render interpolation and compared
with `operator==` in the golden tests, so its identity must not depend on
transient buffers. Instead `step` owns a `static thread_local StepWorkspace` — one
recycled pool per thread, safe even if independent worlds are stepped on different
threads. The warm-start cache is swapped (not copied) into `World::contactCache`
each substep, so the two buffers ping-pong forever.

`tests/PerfBench.cpp` is a headless timing harness for this path. It also prints a
position **checksum**, which is how the optimization was validated: the allocation
rewrite had to leave the checksum bit-identical (it did), proving it changed
performance and nothing else.

### 7.2 Why the solver stays single-threaded

Profiling the hot path (a 1400-body pile) shows the time going to the
sequential-impulse **solver** and the broadphase **sort** — both inherently
serial. The solver is Gauss–Seidel: each contact's impulse reads the velocities
the *previous* contacts just wrote, so bodies are coupled through shared contacts.
Parallelizing that would make the result depend on thread scheduling and destroy
the bit-for-bit determinism that warm-starting and the golden tests rely on. That
determinism is a first-class correctness property, so the solver is serial *by
design*, not by omission — and Amdahl's law then caps what threading the
narrowphase alone can buy. (Detecting collisions once per fixed step and reusing
them across sub-steps — Box2D-v3-style TGS — is the real lever to cut the serial
work further; it changes the simulation's numeric results, so it would re-baseline
the goldens and is left as a documented next step.)

> **GPU: rendering vs. physics.** The experimental **Metal corridor demo** moves
> *rendering* to the GPU — hundreds of balls in one instanced draw call via
> metal-cpp (`MetalRenderer`, see [BuildGuide.md](BuildGuide.md#part-6)). That's
> GPU *drawing*, not GPU *physics*: the simulation stays on the CPU (deterministic,
> testable), parallelized via `parallelFor`. Moving the *solver* itself to the GPU
> (compute shaders) is a much bigger, non-deterministic step and is intentionally
> left out — it would slot in behind the same `parallelFor` seam.

---

## 8. The extras: raycast, forces, events, constraints, scheduler

These are additive systems, each mirroring a RealityKit idea:

- **Raycast (`:Query`)** — `raycastNearest(world, origin, direction, maxDistance)`
  returns the nearest hit (entity, point, surface normal, distance), testing the
  ray against every sphere/box. It's RealityKit's `scene.raycast`. The car casts
  four of these (one per wheel) for its suspension. *(Watch out for a ray whose
  origin is inside a box — we explicitly return "no hit" there rather than a
  bogus zero-distance hit; another bug the review caught.)*
- **Forces (`:Forces`)** — `applyRadialImpulse` (an explosion: an outward impulse
  that falls off with distance, so heavier debris moves less) and `applyDrag`
  (velocity damping, folded in the stable implicit way so it never overshoots
  past zero). RealityKit's force effects.
- **Collision events (`:Events`)** — instead of polling, the engine can report
  `Began` / `Persisted` / `Ended` events by diffing the contact cache between
  frames. The demos surface these as Composable-Architecture *actions* (the
  cradle counts each "click"), which is the C++ analog of
  `scene.subscribe(to: CollisionEvents.Began)`.
- **Constraints (`:World` + solver)** — a distance/spring joint holds two bodies'
  anchor points a fixed distance apart (rigid) or springs them toward it. The
  cradle's strings are rigid distance constraints; there's a pendulum energy test.
- **Scheduler (`:Scheduler`)** — an ordered system registry (`.before` / `.after`
  dependencies, topologically sorted). It mirrors RealityKit's system scheduling
  and is the documented extension point for injecting your own systems into the
  pipeline.

---

## 9. The math-notation convention

Physics code uses short names that match the textbooks, on purpose:

| symbol | meaning |
| --- | --- |
| `n` / `normal` | contact normal, unit, from A to B |
| `rA`, `rB` | vector from body A's / B's center of mass to the contact point |
| `vₙ` (`normalVelocity`) | relative velocity along the normal |
| `e` (`restitution`) | coefficient of restitution (0 = dead, 1 = perfectly elastic) |
| `μ` (`friction`) | Coulomb friction coefficient |
| `j` / `lambda` | impulse magnitude |
| `h` | sub-step timestep, `dt / subStepCount` |

Everything else in the codebase is spelled out in full (`context`, not `ctx`;
`inverseMass`, not `invMass`). The math stays short so it lines up with any
reference you read alongside it — see the sources in the [README](../README.md).
