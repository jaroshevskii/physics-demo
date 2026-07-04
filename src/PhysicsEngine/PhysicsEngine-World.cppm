export module PhysicsEngine:World;

import std;
import :Math;
import :Components;
import :ContactCache;

// The simulation `World` — the ECS store (RealityKit's `Scene`). Storage is
// structure-of-arrays: one parallel vector per component, indexed by an entity
// id (its position). Every entity carries every component here (no archetypes),
// which keeps the model approachable while still teaching data-oriented layout.
//
// `World` is a value type with `operator==`, so it can live inside a Composable-
// Architecture feature `State`, be copied cheaply for render interpolation, and
// be asserted against in tests.
export namespace Physics {

/// An entity id — its index into the World's parallel component arrays.
using Entity = std::size_t;

/// A distance / spring joint between two bodies at local anchor points. Keeps the
/// anchors a fixed `restLength` apart (rigid when `stiffness == 0`) or acts as a
/// spring pulling toward `restLength` (when `stiffness > 0`). Stored on the World
/// as a value type so it stays comparable/testable.
struct DistanceConstraint {
  Entity a = 0, b = 0;
  /// Anchor in A's body frame (relative to its COM).
  Vec3 localAnchorA;
  /// Anchor in B's body frame (relative to its COM).
  Vec3 localAnchorB;
  float restLength = 1.0f;
  /// `0` => rigid rod; `> 0` => spring (Hz-like).
  float stiffness = 0.0f;
  /// Spring damping ratio.
  float damping = 1.0f;
  /// Accumulated impulse (warm-started across steps).
  float impulse = 0.0f;
  bool operator==(const DistanceConstraint &) const = default;
};

/// The simulation store: structure-of-arrays ECS keyed by `Entity` (RealityKit's
/// `Scene`). A value type with `operator==` for cheap copies and testability.
struct World {
  std::vector<Transform> transforms;
  std::vector<Collider> colliders;
  std::vector<Body> bodies;
  std::vector<Motion> motions;
  std::vector<DistanceConstraint> constraints;

  /// Frame-to-frame contact memory for warm-starting + collision events. A value
  /// type kept sorted by pair, so `World` stays comparable/copyable.
  ContactCache contactCache;
  /// Total fixed steps taken (the event epoch).
  std::uint64_t stepCount = 0;

  bool operator==(const World &) const = default;

  Entity count() const { return transforms.size(); }
  bool empty() const { return transforms.empty(); }
  void clear() {
    transforms.clear();
    colliders.clear();
    bodies.clear();
    motions.clear();
    constraints.clear();
    contactCache.manifolds.clear();
  }
};

/// Connects two bodies with a distance/spring joint at their centers (or given
/// local anchors). `stiffness == 0` => a rigid rod; `> 0` => a spring.
inline void addDistanceConstraint(World &w, Entity a, Entity b, float restLength,
                                  float stiffness = 0.0f, Vec3 localAnchorA = {},
                                  Vec3 localAnchorB = {}) {
  w.constraints.push_back({a, b, localAnchorA, localAnchorB, restLength, stiffness, 1.0f, 0.0f});
}

/// Adds an entity from its four components, returning its id.
inline Entity add(World &w, Transform t, Collider c, Body b, Motion m = {}) {
  const Entity id = w.count();
  w.transforms.push_back(t);
  w.colliders.push_back(c);
  w.bodies.push_back(b);
  w.motions.push_back(m);
  return id;
}

// --- Convenience spawners ----------------------------------------------------

/// Spawns a dynamic sphere and returns its id.
inline Entity addSphere(World &w, Vec3 position, float radius, float density, float restitution,
                        float friction, Vec3 velocity = {}) {
  const Collider c{.type = ShapeType::Sphere, .radius = radius};
  return add(w, Transform{.position = position}, c, dynamicBody(c, density, restitution, friction),
             Motion{.linearVelocity = velocity});
}

/// Spawns a dynamic box and returns its id.
inline Entity addBox(World &w, Vec3 position, Vec3 halfExtents, float density, float restitution,
                     float friction, Quat orientation = {}, Vec3 velocity = {}) {
  const Collider c{.type = ShapeType::Box, .halfExtents = halfExtents};
  return add(w, Transform{.position = position, .orientation = orientation}, c,
             dynamicBody(c, density, restitution, friction), Motion{.linearVelocity = velocity});
}

/// Spawns an immovable static box (a wall or floor) and returns its id.
inline Entity addStaticBox(World &w, Vec3 position, Vec3 halfExtents, float restitution,
                           float friction, Quat orientation = {}) {
  const Collider c{.type = ShapeType::Box, .halfExtents = halfExtents};
  return add(w, Transform{.position = position, .orientation = orientation}, c,
             staticBody(restitution, friction));
}

// --- Axis-aligned bounding box (broadphase input) ----------------------------

/// An axis-aligned bounding box, `[min, max]`. Broadphase works on these.
struct AABB {
  Vec3 min;
  Vec3 max;
};

/// World-space AABB enclosing a collider at a given transform.
inline AABB computeAABB(const Transform &t, const Collider &c) {
  switch (c.type) {
  case ShapeType::Sphere: {
    const Vec3 r{c.radius, c.radius, c.radius};
    return {t.position - r, t.position + r};
  }
  case ShapeType::Box: {
    // World half-extent along each axis = sum_j |R_col_j| * h_j.
    const Mat3 r = toMat3(t.orientation);
    const Vec3 ext = absVec(r.c0) * c.halfExtents.x + absVec(r.c1) * c.halfExtents.y +
                     absVec(r.c2) * c.halfExtents.z;
    return {t.position - ext, t.position + ext};
  }
  }
  return {t.position, t.position};
}

/// True if two AABBs intersect (touching counts as overlapping).
inline bool overlaps(const AABB &a, const AABB &b) {
  return a.min.x <= b.max.x && a.max.x >= b.min.x && a.min.y <= b.max.y && a.max.y >= b.min.y &&
         a.min.z <= b.max.z && a.max.z >= b.min.z;
}

// --- Step configuration ------------------------------------------------------

/// The per-step context (RealityKit's `SceneUpdateContext`): the fixed timestep,
/// gravity, and solver tuning. `dt` is always passed in — the engine never reads
/// a wall clock — so the simulation is a pure, deterministic function of state.
struct StepContext {
  /// Fixed timestep, in seconds.
  float dt = 1.0f / 60.0f;
  /// World-space gravitational acceleration.
  Vec3 gravity{0.0f, -9.81f, 0.0f};

  // --- legacy solver path (used only when subStepCount <= 0) ---
  /// Sequential-impulse relaxation passes.
  int velocityIterations = 8;
  /// Positional-correction stiffness.
  float baumgarte = 0.2f;
  /// Allowed overlap; avoids jitter.
  float penetrationSlop = 0.005f;

  // --- TGS soft-step solver (the default) ---
  /// Substeps per step; `0` => legacy single-solve + Baumgarte (for A/B teaching).
  int subStepCount = 4;
  /// Soft-contact stiffness (clamped to `0.25/subDt`).
  float contactHertz = 30.0f;
  /// Soft-contact damping.
  float contactDampingRatio = 10.0f;
  /// Closing speed below which bounce is dropped (bodies come to rest).
  float restitutionThreshold = 1.0f;
  /// Cap on penetration-recovery speed.
  float maxBiasVelocity = 4.0f;

  // --- speculative contacts (CCD-lite) ---
  /// Emit contacts out to this gap (anti-tunnel).
  float speculativeDistance = 0.02f;
  /// Broadphase AABB fattening.
  float aabbMargin = 0.1f;

  // --- broadphase / threading ---
  /// Broadphase grid cell size; `0` => auto-derive from body sizes.
  float broadphaseCellSize = 0.0f;
  /// Narrowphase threads; `1` => run inline (tests pin this).
  unsigned threadCount = 1;
};

} // namespace Physics
