> **Status: historical design record.** This is the planning document for the
> advanced engine upgrades (warm-starting, sub-stepping, speculative contacts,
> broadphase, raycast, forces, events, scheduler) — all of which have since
> shipped. It is kept for the rationale and the trade-off discussion. A few
> implementation details drifted during the build: in particular the rendering
> was later refactored into a two-backend abstraction, so paths like
> `src/Renderer/…` in the screenshot section now live under `src/Render/`
> (`GLRenderer.cpp` / `MetalRenderer.cpp` behind `RenderBackend`). For the
> *current* architecture, read [Architecture.md](Architecture.md) and
> [BuildGuide.md](BuildGuide.md); for what actually landed, the code and its
> `///` docs are authoritative.

# Design Doc: Advanced Physics Engine Upgrade Set

**Target:** `/Users/jaroshevskii/Developer/physics-demo` — C++26 named modules, `import std`, SDL3 + GL 4.1, deterministic ECS mirroring TCA + RealityKit.
**Ground rules that constrain every choice below:**
- `World` (`PhysicsEngine-World.cppm:19`) is a value type with `operator==` and lives inside `Sim::State` (`Simulation.cppm:21`) inside feature `State`. **Anything the solver persists across frames (warm-start cache, contact events) must either live in `World` and be `==`-comparable, or be threaded through `StepContext` as an out-param.** This is the single biggest architectural constraint — it decides where the persistent contact cache goes.
- `step(World&, const StepContext&)` (`PhysicsEngine-Systems.cppm:192`) is the pure reducer. `Sim::advance` (`Simulation.cppm:36`) drives it at `fixedDt = 1/60` with an accumulator. Determinism is asserted by `PhysicsEngineTests.cpp` (bit-exact world) and `TestStore` golden-state assertions.
- Solver already accumulates `normalImpulse`/`tangentImpulse` per `SolverContact` (`PhysicsEngine-Systems.cppm:70-71`) — warm-start needs to *carry these across frames*, not invent them.

I use three effort/risk tags per item: **Effort** (S/M/L), **Risk** (Low/Med/High), **Verdict** (BUILD NOW / SCAFFOLD+DEFER).

---

## Cross-cutting change: persistent state on `World`

Several features (warm-start, collision events) need frame-to-frame memory. Add one field to `World` and keep it `==`-comparable so golden tests still work:

```cpp
// PhysicsEngine-World.cppm, inside struct World
ContactCache contactCache;   // defined in new :ContactCache partition; has operator==
```

`ContactCache` is a `std::vector<PersistentManifold>` sorted by `PairKey` (see §1). Because it is deterministically ordered and value-comparable, `World::operator==` (defaulted) keeps working and TestStore assertions still pass — but note the golden *values* change once warm-start is on (expected; re-baseline once).

New module partitions added this cycle (all registered in `CMakeLists.txt` `add_module_library(PhysicsEngine ...)` list, order matters — dependencies first):

```
:ContactCache   (after :Collision)      persistent manifolds, PairKey, feature ids
:Solver         (extracted from :Systems) TGS-soft loop, warm-start, restitution
:Query          (after :World)          raycast, ray-vs-shape, sweeps
:Forces         (after :Systems)        force fields, explosion, drag
:Events         (after :ContactCache)   CollisionEvent stream
:Constraints    (after :Solver)         distance/spring joint
:Scheduler      (after :Systems)        ordered system registry
:SpatialHash    (replaces body of :Broadphase)
:Threading      (new small module, sibling of PhysicsEngine)
```

`StepContext` (`PhysicsEngine-World.cppm:104`) gains these fields (all pure inputs, defaulted so existing call sites compile unchanged):

```cpp
struct StepContext {
  float dt = 1.0f / 60.0f;
  Vec3 gravity{0.0f, -9.81f, 0.0f};
  // --- solver (existing) ---
  int velocityIterations = 8;         // used only by legacy solver path
  float baumgarte = 0.2f;             // DEAD once TGS is default; kept for A/B tests
  float penetrationSlop = 0.005f;
  float restitutionThreshold = 1.0f;
  // --- NEW: sub-stepping / TGS-soft ---
  int   subStepCount = 4;             // 0 => use legacy single-solve+Baumgarte path
  float contactHertz = 30.0f;
  float contactDampingRatio = 10.0f;
  float speculativeDistance = 0.02f;  // 4 * linearSlop
  float aabbMargin = 0.1f;            // broadphase fattening
  float maxBiasVelocity = 3.0f;       // clamp on penetration recovery speed
  // --- NEW: broadphase ---
  float broadphaseCellSize = 0.0f;    // 0 => auto-derive
  // --- NEW: threading (tests force 1) ---
  unsigned threadCount = 1;
};
```

**Determinism rule for the whole doc:** any new list (pairs, manifolds, events, cache entries) is produced/consumed in a fixed sorted order (by `PairKey`, then feature id). No `unordered_map` iteration order ever reaches an observable output.

---

## (1) Warm-starting + persistent contact cache + contact IDs

**Approach chosen.** Persistent per-pair manifold cache keyed by `PairKey = {min(a,b), max(a,b)}`, with per-point stable feature ids `uint16_t id = (refIdx<<8)|incIdx` derived from the box-box SAT+clip. Each step: run narrowphase → match new manifold points against last frame's by equal id → copy `normalImpulse`/`tangentImpulse` in → warm-start (re-apply cached impulses to velocities before the first solve). This is Commit 1 of 3 (per the finding's recommendation) and lands *before* TGS — biggest stability win, smallest change.

**Feature ids are the make-or-break.** The existing `collideBoxBox` (`PhysicsEngine-Collision.cppm:181`) already computes `refAxisIdx`, `refSign`, `incAxis`, `incSign`, and clips a polygon. I extend `ContactPoint` to carry an id, assigned deterministically at clip time. Canonicalize so a reference-face flip doesn't scramble ids: the id encodes `(referenceFaceCode, clippedVertexOrdinal)` where `referenceFaceCode = refAxisIdx*2 + (refSign>0)`, and A/B are canonicalized by `PairKey` (low entity first) *before* narrowphase is called. Sphere contacts get a constant id `0` (single point, always matches).

**New module: `PhysicsEngine-ContactCache.cppm` (`:ContactCache`).**

```cpp
export module PhysicsEngine:ContactCache;
import :Math; import :Collision; import :World;
export namespace Physics {

struct PairKey {
  Entity a, b;                                  // a < b, canonical
  auto operator<=>(const PairKey&) const = default;
  bool operator==(const PairKey&) const = default;
};
inline PairKey makeKey(Entity i, Entity j) { return i<j ? PairKey{i,j} : PairKey{j,i}; }

struct CachedPoint {
  std::uint16_t id = 0;
  float normalImpulse = 0.0f;
  float tangentImpulse = 0.0f;
  bool operator==(const CachedPoint&) const = default;
};

struct PersistentManifold {
  PairKey key;
  std::array<CachedPoint, 4> points;
  int count = 0;
  std::uint64_t lastStep = 0;                   // for event Began/Ended diffing (§8)
  bool operator==(const PersistentManifold&) const = default;
};

struct ContactCache {
  std::vector<PersistentManifold> manifolds;    // kept sorted by key
  bool operator==(const ContactCache&) const = default;
};

// Binary-search the sorted cache.
const PersistentManifold* find(const ContactCache&, PairKey);
// Copy old impulses into `cur` (matched by equal id); returns matched manifold or defaults.
void matchManifold(const ContactCache& prev, PairKey, Collision& cur);
}
```

**Extend `Collision`/`ContactPoint`** (`PhysicsEngine-Collision.cppm:18`): add `std::uint16_t id = 0;` to `ContactPoint`, plus `float normalImpulse=0, tangentImpulse=0;` (scratch the solver reads/writes for warm-start). Assign `id` in `collideBoxBox`'s clip loop (line 316-321): `col.contacts[col.count].id = uint16_t((referenceFaceCode<<8) | col.count);`.

**Solver changes (`SolverContact`).** Add `std::uint16_t id;`. In `solveContacts` build loop, before solving, seed `sc.normalImpulse`/`sc.tangentImpulse` from the matched cache; after the iteration loop, write them back into `world.contactCache`. New signatures in `:Solver`:

```cpp
void warmStart(World&, std::vector<SolverContact>&, const std::vector<Mat3>& invI);
void storeImpulses(World&, const std::vector<SolverContact>&,
                   const std::vector<Pair>&, std::uint64_t stepCount);
```

**How tests verify it.** (a) New `WarmStartTests.cpp`: a 5-box vertical stack, run 300 steps, assert the top box's resting `position.y` settles to within `2*slop` of analytic rest and that per-step position delta → 0 (no jitter) — warm-start off vs on comparison shows strictly lower residual velocity. (b) Determinism: existing bit-exact test in `PhysicsEngineTests.cpp:testDeterminism` must still pass (cache is deterministically ordered). (c) Feature-id stability unit test: take two boxes in fixed contact, call `collide` twice with a tiny orientation nudge, assert the returned `ContactPoint.id`s are identical set (this catches the "flipped reference face renumbers ids" trap directly). (d) Re-baseline golden trajectories.

**Effort M · Risk Med · Verdict BUILD NOW.** Risk is entirely in feature-id stability; the id-stability unit test is the mitigation and must be written first.

---

## (2) Sub-stepping / TGS-soft

**Approach chosen.** Replace the top-level solve with the sub-stepped soft-step loop from Box2D v3, gated by `ctx.subStepCount` (0 = keep the legacy `solveContacts` + `correctPositions` path for A/B teaching comparison). This is Commit 3 of 3 and depends on §1 (warm-start) being in. **The Baumgarte `correctPositions` pass (`PhysicsEngine-Systems.cppm:171`) is deleted from the TGS path** — position error re-enters as the velocity bias `biasRate*separation`. Do NOT run both (the finding's #1 risk).

**Restructure `step`** (`PhysicsEngine-Systems.cppm:192`) into `:Solver`:

```cpp
void step(World& w, const StepContext& ctx) {
  if (ctx.subStepCount <= 0) { legacyStep(w, ctx); return; }   // old pipeline, A/B
  const float h = ctx.dt / ctx.subStepCount, inv_h = ctx.subStepCount / ctx.dt;

  const auto pairs = broadphase(w);                            // once, fattened AABBs
  auto manifolds = narrowphaseAll(w, pairs, ctx);              // once, speculative (§3)
  auto contacts  = prepareContacts(w, manifolds, pairs, ctx, h); // makeSoft, anchors,
                                                               // baseSeparation, relVel, warm-start match
  for (int s = 0; s < ctx.subStepCount; ++s) {
    integrateVelocities(w, ctx, h);        // gravity + Padé damping, dt=h
    warmStart(w, contacts);                // re-apply cached impulses
    solveVelocity(w, contacts, inv_h, /*useBias=*/true);   // soft normal bias, no friction
    integratePositions(w, ctx, h);
    solveVelocity(w, contacts, inv_h, /*useBias=*/false);  // relax + friction
    applyRestitution(w, contacts, ctx);
  }
  storeImpulses(w, contacts, pairs, w.stepCountThisFrame);
}
```

**New free functions/signatures in `:Solver`:**

```cpp
struct Soft { float biasRate, massScale, impulseScale; };
Soft makeSoft(float hertz, float zeta, float h);            // rigid if hertz<=0

// SolverContact gains: Soft soft; float baseSeparation; Vec3 rA0,rB0; float relVel;
//                      float totalNormalImpulse; (per point when count>1)
std::vector<SolverContact> prepareContacts(World&, const std::vector<Collision>&,
                                           const std::vector<Pair>&, const StepContext&, float h);
void integrateVelocities(World&, const StepContext&, float h);   // now takes h, Padé damping
void integratePositions(World&, const StepContext&, float h);
void solveVelocity(World&, std::vector<SolverContact>&, float inv_h, bool useBias);
void applyRestitution(World&, std::vector<SolverContact>&, const StepContext&);
```

**Key correctness details (straight from the finding, non-negotiable):**
- `makeSoft`: `omega=2π·hertz; a1=2ζ+h·omega; a2=h·omega·a1; a3=1/(1+a2); {omega/a1, a2·a3, a3}`.
- Clamp `hertz = min(ctx.contactHertz, 0.25f*inv_h)`; use `2×` hertz ("staticSoftness") when one body is static. This Nyquist clamp is why more substeps buys stiffness.
- **Separation is recomputed each substep from moved anchors**, not re-running narrowphase: store `rA0,rB0` (world anchor deltas at prepare) and `baseSeparation`; each substep `s = baseSeparation + dot(currentAnchorDelta, n)`.
- Two-branch bias inside `solveVelocity`: `s>0` (speculative) → `bias=s*inv_h, mScale=1, iScale=0`; `s<=0 && useBias` → `bias=max(soft.biasRate*s, -ctx.maxBiasVelocity), mScale=soft.massScale, iScale=soft.impulseScale`; relax pass with `s<=0` → `bias=0, mScale=1, iScale=0`.
- Impulse update, clamp accumulate ≥0, apply delta; friction **only in relax pass**, clamped `±friction*normalImpulse`.
- Restitution: capture approach `relVel` at prepare, apply as dedicated pass only if `relVel < -restitutionThreshold && totalNormalImpulse > 0`.
- Integration is semi-implicit Euler at `h` with Padé damping `v *= 1/(1+h·linearDamping)`.

**How tests verify it.** (a) All existing `PhysicsEngineTests.cpp` invariants must pass under the TGS path (free-fall closed form still holds — gravity integration is unchanged; elastic momentum/KE; inelastic KE-non-increasing; resting box no-sink; **fast-body-no-tunnel now passes at higher speeds** thanks to speculative bias). Run the whole suite twice: once `subStepCount=0` (legacy), once `=4`. (b) New `TunnelTests.cpp`: fire a sphere at 200 m/s at a 0.1 m-thick static wall; assert it stops on the near side (`position` never crosses). (c) Determinism preserved (fixed iteration order). (d) Stack test from §1 should be *stiffer/less jittery* than warm-start-only.

**Effort L · Risk High · Verdict BUILD NOW** (it is the headline teaching feature — sub-stepping + soft constraints — and the finding is explicit it's doable in a separable commit). High risk mitigated by keeping `legacyStep` behind `subStepCount=0` so you always have a working baseline to diff against.

---

## (3) Speculative contacts (CCD-lite)

**Approach chosen.** Fatten broadphase AABBs by `ctx.aabbMargin` and emit contacts out to `ctx.speculativeDistance` with **positive** separation stored; the `s*inv_h` bias in `solveVelocity` (§2) does the anti-tunneling. No separate CCD sweep. This is Commit 2 of 3 — landable independently on the legacy solver as a "clamp closing velocity" pass, but its full value shows with TGS.

**Changes.**
- Broadphase: after `computeAABB` (`PhysicsEngine-World.cppm:77`), inflate each box by `ctx.aabbMargin` before `overlaps`. (Broadphase currently takes only `World`; it will need `ctx` — see §4, where I add the overload.)
- Narrowphase: each `collide*` routine currently returns `nullopt` when separated. Add a variant that, when `0 < separation < speculativeDistance`, returns a `Collision` with `count>0`, the correct normal, and **positive** `penetration` (i.e. negative penetration = a gap). Simplest: change `ContactPoint.penetration` semantics to "signed: positive = overlap, negative = gap up to speculativeDistance", and have the solver treat `s = -penetration`. For sphere-sphere (`:Collision:118`) this is trivial (`dist < r + specDist`); for box-box, keep the SAT early-outs but relax the `> 0.0f` reject to `> speculativeDistance`.

**Sign-slip guard (the finding's key risk):** speculative points (`s>0`) must never apply friction (friction is relax-pass + gated on `normalImpulse>0`) and never apply an attractive normal impulse (the `max(...,0)` clamp handles it). Add a debug assert in `solveVelocity`: if `s>0` then applied normal impulse delta must keep `normalImpulse>=0`.

**How tests verify it.** The `TunnelTests.cpp` from §2 is the primary test. Add a "speculative doesn't glue" test: two spheres 0.015 m apart (inside specDistance) with zero velocity and gravity off — assert they do **not** attract (positions unchanged over 60 steps).

**Effort M · Risk Med · Verdict BUILD NOW.**

---

## (4) Spatial-hash broadphase (drop-in)

**Approach chosen.** Flat dense 3D grid for the bounded ball-pit path, behind the unchanged `broadphase` signature, with the brute-force version kept as `bruteForceBroadphase` for a parity oracle. `std::sort` + `std::unique` makes output byte-identical to brute force. Defer the hash-map (unbounded) and dynamic-BVH (mixed sizes) variants.

**Signature note:** current `broadphase(const World&)` (`PhysicsEngine-Broadphase.cppm:18`) is called from `step` (`:Systems:195`) with no `ctx`. Since §3 needs `aabbMargin` and §4 needs `broadphaseCellSize` from `ctx`, I add an overload `broadphase(const World&, const StepContext&)` and have `step` call that; keep the 1-arg version delegating with a default `StepContext{}` for existing tests (`CollisionTests`/`PhysicsEngineTests` call the 1-arg form implicitly via `step`). Both live in `:SpatialHash` which replaces the body of `:Broadphase` (module name unchanged so no CMake/import churn).

**Body** (dense grid; `cellSize` auto = `2 × maxColliderRadius` for spheres or `1.5× mean AABB extent`):

```cpp
std::vector<Pair> broadphase(const World& w, const StepContext& ctx) {
  const Entity n = w.count();
  std::vector<AABB> boxes; boxes.reserve(n);
  for (Entity i=0;i<n;++i) {
    AABB b = computeAABB(w.transforms[i], w.colliders[i]);
    const Vec3 m{ctx.aabbMargin, ctx.aabbMargin, ctx.aabbMargin};   // §3 fattening
    boxes.push_back({b.min - m, b.max + m});
  }
  const float h = ctx.broadphaseCellSize>0 ? ctx.broadphaseCellSize : chooseCellSize(w, boxes);
  // insert into cells; large-body guard: if a body spans > 64 cells, push to a
  // linear "big" list swept against all dynamics instead of inserting everywhere.
  // gather candidate pairs; skip static-static; re-test overlaps(boxes[i],boxes[j]).
  std::sort(pairs.begin(), pairs.end());
  pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
  return pairs;   // identical order to bruteForceBroadphase(w, ctx)
}
```

Cell math in `int64_t` (positions go negative; `Entity` stays `size_t`); the large-static-body guard is mandatory (the pit floor/walls are big static boxes). `overlaps` re-test is **not** skipped, so the candidate set is identical to brute force.

**How tests verify it.** New `BroadphaseTests.cpp`: seed many random ball-pit `World`s via the existing `RandomNumberGenerator` dependency; assert `broadphase(w,ctx) == bruteForceBroadphase(w,ctx)` (exact `std::vector` equality) — this proves correctness *and* determinism in one shot. Because output is identical, all existing physics tests pass unchanged.

**Effort M · Risk Low · Verdict BUILD NOW** (highest-leverage perf upgrade for the ball pit, zero downstream churn).

---

## (5) Deterministic multithreaded narrowphase + GPU particle verdict

**Approach chosen (CPU).** A ~150-line `std::jthread` persistent pool in a new tiny module `Threading` (sibling library, not part of `PhysicsEngine` so the pure engine stays dependency-light; passed in via `ctx.threadCount`). Parallelize **only** narrowphase manifold generation, which is pure over `const World` + one pair. Everything else (broadphase pair gather, the sequential-impulse/TGS solver with its cross-body read-modify-write, integration) stays single-threaded.

**Do NOT use `std::execution::par`** — the finding verifies it is disabled/serial in Homebrew libc++ (clang 22) and non-deterministic when enabled.

**Determinism rule (load-bearing):** preallocate `std::vector<std::optional<Collision>> results(pairs.size())`; worker for pair `i` writes **only** `results[i]`; no `push_back`, no shared accumulator; solver consumes in ascending `i`. Bit-identical to serial because each manifold is a pure function and merge order is fixed.

```cpp
// module Threading
class ThreadPool {
public:
  explicit ThreadPool(unsigned n);            // n==1 => run inline (default, tests)
  void parallel_for(std::size_t count, const std::function<void(std::size_t)>&);
};
// in :Systems / :Solver narrowphaseAll:
std::vector<std::optional<Collision>> results(pairs.size());
pool.parallel_for(pairs.size(), [&](std::size_t i){
  results[i] = collide(w.transforms[pairs[i].first], w.colliders[pairs[i].first],
                       w.transforms[pairs[i].second], w.colliders[pairs[i].second]);
});
// then compact in index order into manifolds + hitPairs (single-threaded)
```

`ctx.threadCount` defaults to 1 so all tests run serial and deterministic; add a regression test asserting `step(threadCount=1) == step(threadCount=8)` bit-exact.

**GPU particle path — verdict.** Three options were researched:
- **SDL_GPU compute (Metal):** real compute shaders, but adds SDL_shadercross + a SPIR-V toolchain, a second GPU stack beside GL, an async shader-build step in CMake, and GL↔Metal buffer interop pain. Medium-high effort/risk. Non-deterministic FP → visual-only anyway.
- **GL 4.1 transform feedback:** GL 4.1 has **no** compute shaders (Apple's hard cap), but transform feedback (GL 3.0+) is the macOS-native way to integrate thousands of independent particles on GPU: ping-pong two VBOs, update VS with `GL_RASTERIZER_DISCARD` + `glBeginTransformFeedback`, draw as `GL_POINTS`. ~1 day, low risk, self-contained in the existing `Renderer`. Great for per-particle integration (gravity, drag, force fields); can't do inter-particle (no scatter/atomics) — which the fireworks demo doesn't need.
- **CPU-parallel:** reuse the `ThreadPool`; fully deterministic; fine up to tens of thousands.

**Recommended choice: GL 4.1 transform feedback for the "GPU particle system" deliverable, CPU-parallel as the deterministic fallback, SDL_GPU/Metal scaffolded OFF by default.** Reasons: transform feedback is honest ("thousands of particles on GPU, macOS-native"), low risk, and doesn't drag a SPIR-V toolchain into a teaching build. Keep particles strictly visual-only (never route the tested rigid-body core through GPU FP).

**Scaffold for SDL_GPU:** a `ComputeBackend` interface + a CMake option `PHYSICS_ENABLE_SDL_GPU` default OFF + a short design note. Not a hard dependency.

**Effort:** ThreadPool S · transform-feedback particles M · SDL_GPU scaffold S. **Risk:** ThreadPool Low, TF Low, SDL_GPU (if built) High. **Verdict: BUILD NOW** ThreadPool + transform-feedback particles; **SCAFFOLD+DEFER** SDL_GPU/Metal compute.

---

## (6) Raycast + ray-vs-shape

**Approach chosen.** New `:Query` partition with `World::raycast`-style free functions mirroring RealityKit's `scene.raycast`. Broadphase AABB slab-cull, then analytic ray-vs-sphere and ray-vs-OBB. Add a swept-sphere cast as the `convexCast` analog (also used by the character controller if ever built).

**New module `PhysicsEngine-Query.cppm` (`:Query`):**

```cpp
export module PhysicsEngine:Query;
import :Math; import :Components; import :World;
export namespace Physics {

enum class RayQuery : std::uint8_t { Nearest, All };
using CollisionMask = std::uint32_t;                 // reserved; default ~0

struct RayHit {
  Entity entity;
  Vec3 position;
  Vec3 normal;                                       // world, from ENTRY slab
  float distance;
  bool operator==(const RayHit&) const = default;
};

std::vector<RayHit> raycast(const World& w, Vec3 origin, Vec3 dir, float maxLen,
                            RayQuery q = RayQuery::Nearest, CollisionMask mask = ~0u);
std::optional<RayHit> raycastNearest(const World&, Vec3 o, Vec3 d, float maxLen, CollisionMask=~0u);
std::optional<RayHit> sweptSphereCast(const World&, Vec3 o, Vec3 d, float radius, float maxLen);
}
```

**Ray-vs-shape internals** (module-local): ray-vs-sphere solves `|o+td-c|²=r²` (smaller positive root); ray-vs-OBB transforms the ray into box-local space (`transpose(toMat3(orientation))`), slab-tests the 3 axes, and — the finding's explicit trap — takes the **normal from the entry slab (`t_min` axis)**, signed opposite the ray, then rotates back to world. `dir` normalized on entry; guard `denom≈0`.

**How tests verify it.** New `QueryTests.cpp`: (a) ray straight down at a unit sphere at origin → hit at `distance = originY - 1`, `normal = +Y`; (b) ray at a rotated box (45° about Z) → assert the entry-slab normal equals the rotated face normal (this directly guards the "wrong normal silently breaks suspension" risk that would poison the raycast car); (c) `Nearest` vs `All` ordering (All sorted by distance ascending); (d) a miss returns empty/`nullopt`. All pure functions, trivially deterministic.

**Effort S · Risk Low · Verdict BUILD NOW** (purely additive, unblocks the raycast car and explosion targeting).

---

## (7) Force fields (radial explosion, drag)

**Approach chosen.** A force-accumulation phase run **before** integration, modeled as pure functions over the SoA `Motion`/`Body`/`Transform` arrays, mirroring RealityKit's `ForceEffectProtocol` + `ForceMode`. Drag is a per-step force; explosion is a one-shot impulse.

**New module `PhysicsEngine-Forces.cppm` (`:Forces`):**

```cpp
export module PhysicsEngine:Forces;
import :Math; import :Components; import :World; import :Query;
export namespace Physics {

enum class ForceMode : std::uint8_t { Force, Acceleration, Impulse, Velocity };

// SpatialForceFalloff mirrors Apple: strength * pow(clamp(1 - dist/radius,0,1), rate)
struct RadialFalloff { float radius; float rate = 1.0f; float distanceOffset = 0.0f; };

// Instantaneous radial explosion: applies an impulse to every dynamic body in R.
void applyExplosion(World& w, Vec3 center, float strength, RadialFalloff, ForceMode = ForceMode::Impulse);

// Per-step global drag: F = -kLin*v, torque = -kAng*omega. Folded into velocities (semi-implicit).
void applyDrag(World& w, const StepContext&, float kLin, float kAng);
}
```

`applyExplosion`: `dir = normalized(pos-center); f = pow(clamp(1 - dist/R,0,1), rate); v += (strength*f*invMass)*dir` (Impulse mode). `ForceMode::Acceleration` skips the `invMass` multiply (matches RealityKit).

**Where it runs.** For the explosion demo it's triggered by an Action (see §8-style wiring) and applied *after* `integratePositions` so it doesn't get immediately damped, but the finding warns an instantaneous velocity edit can shove bodies into penetration — mitigation: for the teaching demo, apply the explosion impulse into the velocity state *at the start of a step* (before the solver) so the very next solve resolves any resulting overlap. Drag folds into `integrateVelocities`.

**How tests verify it.** New `ForcesTests.cpp`: (a) explosion on a ring of 8 equal spheres around the center → assert all gain equal-magnitude radially-outward velocity and net linear momentum stays ~0 (symmetry); (b) drag alone on a moving body with gravity off → assert velocity decays monotonically toward 0 and never overshoots/negates (Padé/semi-implicit stability); (c) `Acceleration` mode gives mass-independent velocity change.

**Effort S · Risk Low · Verdict BUILD NOW.**

---

## (8) Collision events (Began/Persisted/Ended) as CA Actions

**Approach chosen.** Diff the persistent contact cache (§1) by step-epoch to classify Began/Persisted/Ended, drain into a **sorted** `std::vector<CollisionEvent>`, surface it out of `step` via an out-param on `StepContext` (or a return channel on `World`), and have the feature reducer translate events into follow-up CA Actions — the C++ analog of RealityKit's `scene.subscribe(to: CollisionEvents.Began.self)`.

**New module `PhysicsEngine-Events.cppm` (`:Events`):**

```cpp
export module PhysicsEngine:Events;
import :Math; import :World; import :ContactCache;
export namespace Physics {

struct CollisionEvent {
  enum class Phase : std::uint8_t { Began, Persisted, Ended };
  Phase phase;
  Entity a, b;                          // canonical a<b
  Vec3 position;                        // contact centroid
  Vec3 impulseDirection;               // manifold normal
  float impulse = 0.0f;                 // sum of accumulated normal impulses (post-solve)
  float penetrationDistance = 0.0f;     // max contact depth
  bool operator==(const CollisionEvent&) const = default;
};
}
```

**Emission.** `step` already runs the solver last, so accumulated impulses are final. After `storeImpulses`, diff `world.contactCache` by `lastStep`: `key present this step, absent last => Began; present both => Persisted; present last, absent this => Ended (then drop)`. **Iterate the cache in sorted `PairKey` order** (it already is) so event order is deterministic — the finding's #1 event risk. Events go into a `std::vector<CollisionEvent>* out` field on `StepContext` (nullable; nullptr = don't collect, keeps the pure signature clean and lets tests opt in). `impulse` = sum of `CachedPoint.normalImpulse`; `position` = centroid of contact points; `impulseDirection` = manifold normal.

**Surfacing as CA Actions.** In `BallPitFeature`/new demo features, add an Action case and have the `Ticked` handler collect events then re-`send` them:

```cpp
// Action variant gains:
struct CollisionOccurred { Physics::CollisionEvent event; bool operator==(...) const = default; };
// In the Ticked reducer (BallPitFeature.cpp analog):
std::vector<Physics::CollisionEvent> events;
Sim::advance(s.sim, date->now(), &events);        // advance/step thread the out-vec
for (const auto& e : events) store.send(CollisionOccurred{e});   // deterministic order
```

`Sim::advance` (`Simulation.cppm:36`) gains an optional `std::vector<CollisionEvent>* = nullptr` param that it forwards into the `StepContext` for each fixed step (append across sub-steps within a frame).

**How tests verify it.** This is where `TestStore` shines. New `CollisionEventTests.cpp` as a feature test: drop one sphere onto the floor, `store.send(Ticked{}, assert)` until contact, then `store.receive(...)` asserting exactly one `CollisionOccurred` with `phase==Began`, then `Persisted` on subsequent ticks, then `Ended` after removing the floor. Because `TestStore::receive` asserts the produced-action stream, event ordering and payload are pinned deterministically. Plus an engine-level unit test that Began/Persisted/Ended fire in the right frames for a two-body approach/separate.

**Effort M · Risk Med · Verdict BUILD NOW** (depends on §1; the sorted-order discipline is the whole risk).

---

## (9) Distance / spring constraint

**Approach chosen.** A soft (or rigid) distance constraint between two bodies at local anchor points, solved as an extra pass inside the sub-step loop (reusing the §2 `makeSoft` machinery so a spring is just a low-hertz distance constraint). New `:Constraints` partition; constraints stored on `World` as a value-type vector so they're `==`-comparable and testable.

```cpp
export module PhysicsEngine:Constraints;
import :Math; import :World; import :Solver;
export namespace Physics {

struct DistanceConstraint {
  Entity a, b;
  Vec3 localAnchorA, localAnchorB;
  float restLength;
  float hertz = 0.0f;        // 0 => rigid (hard distance); >0 => spring
  float dampingRatio = 1.0f;
  // solver scratch (accumulated impulse for warm-start)
  float impulse = 0.0f;
  bool operator==(const DistanceConstraint&) const = default;
};
// World gains: std::vector<DistanceConstraint> distanceConstraints;

void prepareConstraint(World&, DistanceConstraint&, float h);
void solveConstraint(World&, DistanceConstraint&, float inv_h, bool useBias);
}
```

Solve: axis `n = normalized(anchorB - anchorA)`; `C = length(anchorB-anchorA) - restLength`; effective mass via the existing `effectiveMass` helper (`:Systems:78`, promote it to `:Solver` exported); soft bias `= makeSoft(hertz,ζ,h)`. Rigid (`hertz==0`) → hard constraint (`Soft{0,1,0}`). Solved in the same substep loop as contacts (both bias and relax passes), warm-started via `impulse`.

**How tests verify it.** New `ConstraintTests.cpp`: (a) rigid distance constraint between a static anchor and a hanging body → assert steady-state length == restLength within slop after settling; (b) pendulum energy: rigid constraint, no damping, gravity on → assert total energy (`linearKineticEnergy + potentialEnergy`, both already in `PhysicsEngine.cppm:21,30`) does not grow over 600 steps; (c) spring (`hertz>0`) → assert it oscillates about restLength and, with damping, decays. Determinism via fixed constraint iteration order.

**Effort M · Risk Med · Verdict BUILD NOW** (reuses `makeSoft`; enables the car suspension conceptually and the cradle string variant). Depends on §2.

---

## (10) RealityKit-style ordered system registry

**Approach chosen.** A `SystemRegistry` mirroring RealityKit's `System.dependencies` (`.before`/`.after`), topologically sorted **once** at registration (Kahn), ties broken by registration index → a fixed `std::vector<SystemFn>` that `step` iterates. This reframes the hard-coded pipeline in `step` (`:Systems:192`) as an explicit ordered registry — high teaching value (shows the RealityKit scheduling model) with the current pipeline as the default registration.

**New module `PhysicsEngine-Scheduler.cppm` (`:Scheduler`):**

```cpp
export module PhysicsEngine:Scheduler;
import :World;
export namespace Physics {

using SystemFn = std::function<void(World&, const StepContext&)>;
using SystemId = std::size_t;

struct SystemRegistry {
  SystemId add(std::string name, SystemFn fn,
               std::vector<SystemId> before = {}, std::vector<SystemId> after = {});
  void finalize();                       // Kahn topo-sort; throws on cycle (fail loud)
  void run(World&, const StepContext&) const;   // iterate ordered vector
};

// Default registration reproduces today's pipeline exactly:
SystemRegistry defaultPipeline();        // integrateVel -> broadphase+narrow -> solve -> integratePos [-> correct]
}
```

**Determinism:** no-dependency systems run in registration order; topo-sort ties broken by registration index; cycle detection **throws at `finalize()`** (the finding's explicit risk — a silent dropped edge would reorder nondeterministically).

**How tests verify it.** New `SchedulerTests.cpp`: (a) register systems with `.before/.after` and assert `run` order matches expected topo order; (b) assert `defaultPipeline().run(w,ctx)` produces a world bit-identical to the current hard-coded `step(w,ctx)` (proves the refactor is behavior-preserving — run against a golden from the existing suite); (c) a cyclic registration throws.

**Effort M · Risk Low · Verdict SCAFFOLD+DEFER the general registry, BUILD NOW the default-pipeline extraction.** Rationale: the *value* is teaching the RealityKit scheduling model, but making `step` fully registry-driven risks the deterministic golden traces mid-upgrade. Extract `defaultPipeline()` now (behavior-preserving, tested by (b)), expose `SystemRegistry` as the documented extension point, defer letting demos inject custom systems until the core solver upgrades (§1-3) have settled.

---

## (11) Screenshot capture (`--screenshot`)

**Approach chosen.** Self-contained zlib-backed PNG encoder (`-lz` already on the box), reading `GL_BACK` before `SwapWindow`. Not the `sips` fallback (adds a fork/exec + temp file, fails silently in automation).

**New source `src/Renderer/Screenshot.cpp`** (implementation unit of a new `:Screenshot` partition or a free function in `Renderer` — put `#include <OpenGL/gl3.h>` and `#include <zlib.h>` in the **global module fragment**, never across the module interface into `import std`):

```cpp
// Renderer module surface (Renderer.cppm) gains:
namespace Render {
  bool captureAndWritePng(const char* path, int widthPx, int heightPx);
}
```

Capture: `glFinish(); glReadBuffer(GL_BACK); glReadPixels(0,0,w,h,GL_RGBA,GL_UNSIGNED_BYTE,buf)`. **Size with `SDL_GetWindowSizeInPixels`** (the app already fetches `w,h` at `main_ballpit.cpp:122-123` — reuse exactly those, NOT `SDL_GetWindowSize`; the Retina points-vs-pixels mismatch is the #1 bug here). Use `GL_RGBA` (core-safe, 4-byte aligned, no `glPixelStorei` needed). Flip rows bottom-up→top-down. Encode: prepend filter byte `0x00` per scanline, `compress2(..., Z_BEST_COMPRESSION)` sized via `compressBound`, wrap in IHDR (colortype 6) + IDAT + IEND with big-endian lengths and zlib `crc32` over chunk-type+data.

**Wiring in `main_ballpit.cpp`** (and each new demo): parse `--screenshot PATH` alongside the existing `--frames N` (`main_ballpit.cpp:26`), then on the final frame, **before** `SDL_GL_SwapWindow` (line 127):

```cpp
if (!shotPath.empty() && maxFrames>0 && frame==maxFrames-1) {
  int w=0,h=0; SDL_GetWindowSizeInPixels(window,&w,&h);
  Render::captureAndWritePng(shotPath.c_str(), w, h);
}
```

`--frames N` already drives deterministic Ticked steps, so choosing N large enough lets the sim settle before the shot.

**CMake:** `target_link_libraries(Renderer PUBLIC PhysicsEngine OpenGL::GL z)` (SDK `libz.tbd`, no `find_package`).

**How tests verify it.** Not a unit test (it's I/O + GL) — verified by a `scripts/` smoke check: run `ballpit --frames 300 --screenshot out.png`, assert the file exists, is non-empty, starts with the PNG signature `89 50 4E 47`, and decodes (via `sips -g pixelWidth out.png` in the script only, not the binary) to the expected pixel dimensions. This is a build/CI artifact check, not a golden-image comparison (GL FP varies by driver).

**Effort S · Risk Low · Verdict BUILD NOW** (unblocks documenting every new demo with real screenshots).

---

## New demo targets

Each is a new `add_module_library(<X>Feature ...)` + `add_executable` in `CMakeLists.txt`, same shape as `ballpit` (`CMakeLists.txt:132`), each with `--frames`/`--screenshot`.

| Demo | Engine features exercised | New feature module |
|---|---|---|
| **Explosions** | (7) radial explosion impulse + falloff, (8) collision events (debris impacts → sound/spawn Actions), (3) speculative (fast debris no-tunnel), (4) spatial hash (many bodies). A stacked wall/pile is detonated by a `Detonate{center}` Action. | `ExplosionFeature` |
| **Raycast car w/ suspension** | (6) raycast (4 wheel rays), (9) spring constraint math (suspension spring-damper), (2) sub-stepping (stiff-suspension stability — the finding warns raycast-car is dt-sensitive; tune springK/damperK against sub-step dt), friction-circle tire impulses on the single chassis Body via existing `effectiveMass`. Wheels are render-only. Input: `WASD` → steer/throttle Actions. | `CarFeature` |
| **Particle fireworks** | (5) GL 4.1 transform-feedback GPU particles, (7) force fields (gravity + drag + radial burst as a per-particle force in the update VS). Visual-only, not the tested core — a `LaunchFirework{pos}` Action seeds emitters; GPU integrates. | `FireworksFeature` (thin; particle state in Renderer, not `World`) |
| **Newton's cradle / dominoes** | (1) warm-starting + (2) TGS-soft (the headline: cradle needs restitution≈1 + stable resting contacts + momentum transfer through a line of touching spheres — this is the definitive warm-start/soft-step showcase), (8) collision events (click clack), (10) ordered pipeline. Dominoes exercise stacking + toppling chain reaction. | `CradleFeature`, `DominoesFeature` |

The **cradle** is the single best teaching demo for the whole solver upgrade: it visibly fails on the legacy Baumgarte solver (energy loss, no clean momentum transfer) and works on warm-start + TGS-soft, so you can A/B it live via `ctx.subStepCount = 0` vs `4`.

---

## RECOMMENDED CUT LINE

Prioritizing **correctness + teaching value**, and respecting the dependency chain (warm-start → TGS → constraints/events; raycast → car; broadphase is independent):

### Build NOW (core correctness + the demos that prove it)
1. **§1 Warm-starting + contact cache + feature ids** — biggest stability win, smallest change; write the feature-id stability test *first*.
2. **§3 Speculative contacts** — small, fixes tunneling, independently landable.
3. **§2 TGS-soft sub-stepping** — the headline teaching feature; keep `legacyStep` behind `subStepCount=0` for live A/B.
4. **§4 Spatial-hash broadphase** — Low risk, parity-tested, unlocks high body counts for explosions/pit.
5. **§6 Raycast** — Low risk, purely additive, unblocks the car.
6. **§8 Collision events → CA Actions** — Med risk (sorted-order discipline), high teaching value, showcases the TCA event→Action loop.
7. **§7 Force fields (drag + explosion)** — S/Low.
8. **§11 Screenshot capture** — S/Low, needed to document everything.
9. **§5 ThreadPool + GL transform-feedback particles** — deterministic parallel narrowphase + honest GPU particles.
10. **Demos: Newton's cradle/dominoes** (proves §1+§2), **explosions** (proves §7+§8+§4), **raycast car** (proves §6+§9), **fireworks** (proves §5 GPU path).

### Scaffold + Defer (extension points, not load-bearing)
- **§9 Distance/spring constraint** — build the module + rigid/spring solve + tests, but the car's suspension can ship as inline chassis impulses (finding: car needs only chassis-body + ray) so the general constraint isn't blocking. Land it right after §2 since it reuses `makeSoft`; defer *only* if schedule slips.
- **§10 SystemRegistry** — extract `defaultPipeline()` now (behavior-preserving, tested), but keep `step` calling the hard-coded order until §1-3 golden traces re-baseline; expose `SystemRegistry` as documented API, defer demo-injected custom systems.
- **§5 SDL_GPU/Metal compute** — `ComputeBackend` interface + `PHYSICS_ENABLE_SDL_GPU` CMake option OFF by default + design note. Do **not** take the SDL_shadercross/SPIR-V dependency now; transform feedback covers the teaching need.

### Explicitly out of scope this cycle
- CharacterControllerComponent capsule sweep (finding: the raycast car needs only chassis+ray; build only if a walking-character demo is requested).
- Hash-map (unbounded) broadphase and dynamic BVH for mixed sizes (dense grid covers the bounded pit; revisit when the Sandbox needs heterogeneous/unbounded broadphase).

### Sequencing note (determinism hygiene)
Land §1, §3, §2 as three separate commits, re-baselining the `PhysicsEngineTests.cpp` golden values *once* after §2 (warm-start + soft-step legitimately change resting-state numbers). Everything else (§4/§6/§7/§8/§9/§11) is additive and must **not** change existing goldens — §4 has an exact-parity test, §6/§7/§9 add new tests only, §8 is opt-in via a nullable out-vec. Keep `ctx.threadCount=1` and `ctx.subStepCount` explicit in every test.

---

**Files that change / are added (all absolute):**
- Edit `/Users/jaroshevskii/Developer/physics-demo/src/PhysicsEngine/PhysicsEngine-World.cppm` (StepContext fields, `World::contactCache`, `World::distanceConstraints`).
- Edit `/Users/jaroshevskii/Developer/physics-demo/src/PhysicsEngine/PhysicsEngine-Collision.cppm` (`ContactPoint.id`, speculative separation, per-point impulse scratch).
- Edit `/Users/jaroshevskii/Developer/physics-demo/src/PhysicsEngine/PhysicsEngine-Broadphase.cppm` (dense grid body + `broadphase(World,StepContext)` overload + `bruteForceBroadphase`).
- Split solver out of `/Users/jaroshevskii/Developer/physics-demo/src/PhysicsEngine/PhysicsEngine-Systems.cppm` into new `PhysicsEngine-Solver.cppm` (TGS loop).
- New partitions under `/Users/jaroshevskii/Developer/physics-demo/src/PhysicsEngine/`: `-ContactCache.cppm`, `-Query.cppm`, `-Forces.cppm`, `-Events.cppm`, `-Constraints.cppm`, `-Scheduler.cppm`.
- New sibling module `/Users/jaroshevskii/Developer/physics-demo/src/Threading/Threading.cppm`.
- Edit `/Users/jaroshevskii/Developer/physics-demo/src/Renderer/Renderer.cppm` + new `/Users/jaroshevskii/Developer/physics-demo/src/Renderer/Screenshot.cpp` (PNG capture; link `z`).
- Edit `/Users/jaroshevskii/Developer/physics-demo/src/Simulation/Simulation.cppm` (`advance` forwards a `CollisionEvent*` out-vec).
- New features under `/Users/jaroshevskii/Developer/physics-demo/src/Features/`: `Explosion/`, `Car/`, `Fireworks/`, `Cradle/`, `Dominoes/`.
- New apps: `/Users/jaroshevskii/Developer/physics-demo/apps/main_explosion.cpp`, `main_car.cpp`, `main_fireworks.cpp`, `main_cradle.cpp`, `main_dominoes.cpp` (each with `--frames`/`--screenshot`).
- New tests under `/Users/jaroshevskii/Developer/physics-demo/tests/`: `WarmStartTests.cpp`, `TunnelTests.cpp`, `BroadphaseTests.cpp`, `QueryTests.cpp`, `ForcesTests.cpp`, `CollisionEventTests.cpp`, `ConstraintTests.cpp`, `SchedulerTests.cpp`.
- Edit `/Users/jaroshevskii/Developer/physics-demo/CMakeLists.txt` (register new partitions in the `PhysicsEngine` module list in dependency order; add `Threading` lib; add `z` to `Renderer`; add feature libs, apps, and test targets).