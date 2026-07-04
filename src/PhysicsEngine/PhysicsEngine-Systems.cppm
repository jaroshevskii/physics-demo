export module PhysicsEngine:Systems;

import std;
import :Math;
import :Components;
import :World;
import :ContactCache;
import :Collision;
import :Broadphase;
import :Threading;

// The systems that advance the simulation, in the spirit of RealityKit's
// `System.update(context:)`: each is a pure function over the world plus a
// `StepContext`. `step` runs the pipeline:
//
//   [ per fixed step, repeated `subStepCount` times at h = dt/subStepCount ]
//     integrate velocities (gravity)
//       -> broadphase -> narrowphase (speculative manifolds)
//       -> build + WARM-START contacts from the persistent cache
//       -> solve velocity constraints (normal + two-tangent friction)
//       -> restitution
//       -> integrate positions
//       -> positional correction (residual overlap only)
//       -> store impulses back into the cache (next-frame warm start)
//
// Sub-stepping (running the whole solve at a smaller dt several times) buys
// stable tall stacks; warm-starting (seeding each contact with last step's
// impulse) makes them converge in a few iterations; speculative contacts (from
// the narrowphase's negative-penetration "gap" points) brake fast bodies at a
// surface so they don't tunnel. `dt` is always passed in — never a clock — so
// the whole thing stays a deterministic function of (world, context).
export namespace Physics {

/// Applies gravity to every dynamic body's linear velocity for the interval
/// `context.dt` (an explicit Euler velocity update). Static and kinematic bodies
/// are skipped. Run once per sub-step, after each contact's approach speed has
/// been recorded so restitution reflects the true incoming velocity.
void integrateVelocities(World &w, const StepContext &context) {
  for (Entity i = 0; i < w.count(); ++i) {
    if (w.bodies[i].type != BodyType::Dynamic) {
      continue;
    }
    w.motions[i].linearVelocity += context.gravity * context.dt;
  }
}

/// Advances every dynamic body's transform by its current velocity over
/// `context.dt`: position by linear velocity, orientation by angular velocity
/// (via `integrateOrientation`). Static and kinematic bodies are skipped. Run
/// once per sub-step, after the velocity constraints have been solved.
void integratePositions(World &w, const StepContext &context) {
  for (Entity i = 0; i < w.count(); ++i) {
    if (w.bodies[i].type != BodyType::Dynamic) {
      continue;
    }
    w.transforms[i].position += w.motions[i].linearVelocity * context.dt;
    w.transforms[i].orientation =
        integrateOrientation(w.transforms[i].orientation, w.motions[i].angularVelocity, context.dt);
  }
}

/// Advances the world by one fixed step: runs the full solver pipeline
/// `context.subStepCount` times at `h = context.dt / subStepCount` (sub-stepping
/// for stable stacks). A pure, deterministic function of `(w, context)` — `dt` is
/// always supplied, never read from a clock.
void step(World &w, const StepContext &context);

} // namespace Physics

// --- Solver internals (module-local) -----------------------------------------
namespace Physics {
namespace {

Mat3 worldInverseInertia(const Body &b, const Transform &t) {
  const Mat3 R = toMat3(t.orientation);
  return R * b.inverseInertiaBody * transpose(R);
}

Vec3 velocityAt(const World &w, Entity i, Vec3 r) {
  return w.motions[i].linearVelocity + cross(w.motions[i].angularVelocity, r);
}

float effectiveMass(float inverseMassA, float inverseMassB, const Mat3 &inverseInertiaA,
                    const Mat3 &inverseInertiaB, Vec3 rA, Vec3 rB, Vec3 dir) {
  const Vec3 rAxD = cross(rA, dir);
  const Vec3 rBxD = cross(rB, dir);
  const float denom = inverseMassA + inverseMassB + dot(rAxD, inverseInertiaA * rAxD) +
                      dot(rBxD, inverseInertiaB * rBxD);
  return denom > 0.0f ? 1.0f / denom : 0.0f;
}

// An orthonormal tangent basis for two-direction friction, from the normal.
void tangentBasis(Vec3 n, Vec3 &t1, Vec3 &t2) {
  if (std::abs(n.x) >= 0.57735f) {
    t1 = normalized(Vec3{n.y, -n.x, 0.0f});
  } else {
    t1 = normalized(Vec3{0.0f, n.z, -n.y});
  }
  t2 = cross(n, t1);
}

// One solver contact point (a manifold point unpacked + solver scratch).
struct SolverPoint {
  Vec3 rA, rB;
  float sep = 0.0f;         // separation at prepare: >0 gap (speculative), <=0 overlap
  float penetration = 0.0f; // >0 overlap (for positional correction)
  float normalMass = 0.0f, tangentMass1 = 0.0f, tangentMass2 = 0.0f;
  float approachSpeed = 0.0f; // approach normal velocity at prepare (for restitution)
  float normalImpulse = 0.0f, tangentImpulse1 = 0.0f,
        tangentImpulse2 = 0.0f; // accumulated (warm-started)
  std::uint16_t id = 0;
};

struct SolverContact {
  Entity a, b;
  Vec3 normal, t1, t2;
  float friction = 0.0f, restitution = 0.0f;
  std::array<SolverPoint, 4> points{};
  int count = 0;
};

// Look up last frame's accumulated impulse for a given pair + feature id.
void seedWarmStart(const ContactCache &cache, PairKey key, SolverPoint &p) {
  const PersistentManifold *m = find(cache, key);
  if (!m) {
    return;
  }
  for (int i = 0; i < m->count; ++i) {
    if (m->points[i].id == p.id) {
      p.normalImpulse = m->points[i].normalImpulse;
      p.tangentImpulse1 = m->points[i].tangentImpulse1;
      p.tangentImpulse2 = m->points[i].tangentImpulse2;
      return;
    }
  }
}

void prepareContacts(const World &w, const std::vector<Collision> &manifolds,
                     const std::vector<Pair> &pairs, const std::vector<Mat3> &inverseInertia,
                     std::vector<SolverContact> &out) {
  out.clear();
  out.reserve(manifolds.size());
  for (std::size_t m = 0; m < manifolds.size(); ++m) {
    const Collision &collision = manifolds[m];
    const Entity a = pairs[m].first, b = pairs[m].second;
    SolverContact contact;
    contact.a = a;
    contact.b = b;
    contact.normal = collision.normal;
    tangentBasis(contact.normal, contact.t1, contact.t2);
    contact.friction = std::sqrt(w.bodies[a].friction * w.bodies[b].friction);
    contact.restitution = std::min(w.bodies[a].restitution, w.bodies[b].restitution);
    for (int c = 0; c < collision.count; ++c) {
      SolverPoint p;
      p.id = collision.contacts[c].id;
      p.penetration = collision.contacts[c].penetration;
      p.sep = -collision.contacts[c].penetration;
      p.rA = collision.contacts[c].position - w.transforms[a].position;
      p.rB = collision.contacts[c].position - w.transforms[b].position;
      p.normalMass =
          effectiveMass(w.bodies[a].inverseMass, w.bodies[b].inverseMass, inverseInertia[a],
                        inverseInertia[b], p.rA, p.rB, contact.normal);
      p.tangentMass1 = effectiveMass(w.bodies[a].inverseMass, w.bodies[b].inverseMass,
                                     inverseInertia[a], inverseInertia[b], p.rA, p.rB, contact.t1);
      p.tangentMass2 = effectiveMass(w.bodies[a].inverseMass, w.bodies[b].inverseMass,
                                     inverseInertia[a], inverseInertia[b], p.rA, p.rB, contact.t2);
      const Vec3 relativeVelocity = velocityAt(w, b, p.rB) - velocityAt(w, a, p.rA);
      p.approachSpeed = dot(relativeVelocity, contact.normal);
      seedWarmStart(w.contactCache, makeKey(a, b), p);
      contact.points[contact.count++] = p;
    }
    out.push_back(contact);
  }
}

void applyImpulse(World &w, const std::vector<Mat3> &inverseInertia, Entity i, Vec3 impulse,
                  Vec3 r) {
  if (w.bodies[i].type != BodyType::Dynamic) {
    return;
  }
  w.motions[i].linearVelocity += impulse * w.bodies[i].inverseMass;
  w.motions[i].angularVelocity += inverseInertia[i] * cross(r, impulse);
}

void warmStart(World &w, const std::vector<Mat3> &inverseInertia,
               std::vector<SolverContact> &contacts) {
  for (SolverContact &contact : contacts) {
    for (int c = 0; c < contact.count; ++c) {
      SolverPoint &p = contact.points[c];
      const Vec3 P = contact.normal * p.normalImpulse + contact.t1 * p.tangentImpulse1 +
                     contact.t2 * p.tangentImpulse2;
      applyImpulse(w, inverseInertia, contact.a, -P, p.rA);
      applyImpulse(w, inverseInertia, contact.b, P, p.rB);
    }
  }
}

void solveVelocity(World &w, const std::vector<Mat3> &inverseInertia,
                   std::vector<SolverContact> &contacts, const StepContext &context) {
  const float inverseTimestep = context.dt > 0.0f ? 1.0f / context.dt : 0.0f;
  for (int iter = 0; iter < context.velocityIterations; ++iter) {
    for (SolverContact &contact : contacts) {
      for (int c = 0; c < contact.count; ++c) {
        SolverPoint &p = contact.points[c];
        // --- normal ---
        {
          const Vec3 relativeVelocity =
              velocityAt(w, contact.b, p.rB) - velocityAt(w, contact.a, p.rA);
          const float normalVelocity = dot(relativeVelocity, contact.normal);
          // Desired post-impulse normal velocity. For a speculative gap, allow
          // the bodies to close AT MOST the gap this substep (brakes fast bodies
          // at the surface — anti-tunnel); for overlap the target is 0.
          const float target = p.sep > 0.0f ? -p.sep * inverseTimestep : 0.0f;
          // NB: no early-out for separating speculative points — we still run the
          // clamp so an over-eager warm-start impulse gets relaxed back to 0.
          float lambda = p.normalMass * (target - normalVelocity);
          const float newN = std::max(p.normalImpulse + lambda, 0.0f);
          lambda = newN - p.normalImpulse;
          p.normalImpulse = newN;
          const Vec3 P = contact.normal * lambda;
          applyImpulse(w, inverseInertia, contact.a, -P, p.rA);
          applyImpulse(w, inverseInertia, contact.b, P, p.rB);
        }
        // --- friction (two tangents), clamped by the current normal impulse ---
        const float frictionLimit = contact.friction * p.normalImpulse;
        auto solveFriction = [&](Vec3 t, float tMass, float &acc) {
          const Vec3 relativeVelocity =
              velocityAt(w, contact.b, p.rB) - velocityAt(w, contact.a, p.rA);
          const float tangentVelocity = dot(relativeVelocity, t);
          float lambda = -tMass * tangentVelocity;
          const float newT = clampf(acc + lambda, -frictionLimit, frictionLimit);
          lambda = newT - acc;
          acc = newT;
          const Vec3 P = t * lambda;
          applyImpulse(w, inverseInertia, contact.a, -P, p.rA);
          applyImpulse(w, inverseInertia, contact.b, P, p.rB);
        };
        solveFriction(contact.t1, p.tangentMass1, p.tangentImpulse1);
        solveFriction(contact.t2, p.tangentMass2, p.tangentImpulse2);
      }
    }
  }
}

// Restitution as a dedicated pass: reverse the approach velocity by `e`, only
// for genuinely-approaching, in-contact points (kills resting jitter).
void applyRestitution(World &w, const std::vector<Mat3> &inverseInertia,
                      std::vector<SolverContact> &contacts, const StepContext &context) {
  for (SolverContact &contact : contacts) {
    if (contact.restitution <= 0.0f) {
      continue;
    }
    for (int c = 0; c < contact.count; ++c) {
      SolverPoint &p = contact.points[c];
      if (p.approachSpeed > -context.restitutionThreshold || p.sep > 0.0f ||
          p.normalImpulse <= 0.0f) {
        continue;
      }
      const Vec3 relativeVelocity = velocityAt(w, contact.b, p.rB) - velocityAt(w, contact.a, p.rA);
      const float normalVelocity = dot(relativeVelocity, contact.normal);
      // Restitution is a one-shot separating impulse (>= 0). It deliberately does
      // NOT modify p.normalImpulse: the stored/warm-started impulse must stay the persistent
      // contact impulse, or next substep's warm-start would re-inject the bounce.
      const float lambda =
          std::max(p.normalMass * (-contact.restitution * p.approachSpeed - normalVelocity), 0.0f);
      const Vec3 P = contact.normal * lambda;
      applyImpulse(w, inverseInertia, contact.a, -P, p.rA);
      applyImpulse(w, inverseInertia, contact.b, P, p.rB);
    }
  }
}

// Distance/spring joints, solved once per substep. Rigid joints use a Baumgarte
// bias (pull the anchors back to restLength); springs apply a penalty impulse
// F = -(k*C + c*normalVelocity)*dt. Bilateral (impulse may be negative — push or pull).
void solveConstraints(World &w, const std::vector<Mat3> &inverseInertia,
                      const StepContext &context) {
  const float inverseTimestep = context.dt > 0.0f ? 1.0f / context.dt : 0.0f;
  for (DistanceConstraint &con : w.constraints) {
    const Vec3 rA = rotate(w.transforms[con.a].orientation, con.localAnchorA);
    const Vec3 rB = rotate(w.transforms[con.b].orientation, con.localAnchorB);
    const Vec3 anchorA = w.transforms[con.a].position + rA;
    const Vec3 anchorB = w.transforms[con.b].position + rB;
    const Vec3 d = anchorB - anchorA;
    const float len = length(d);
    if (len < 1e-6f) {
      continue;
    }
    const Vec3 n = d / len;
    const float C = len - con.restLength;
    const float m = effectiveMass(w.bodies[con.a].inverseMass, w.bodies[con.b].inverseMass,
                                  inverseInertia[con.a], inverseInertia[con.b], rA, rB, n);
    const Vec3 relativeVelocity = velocityAt(w, con.b, rB) - velocityAt(w, con.a, rA);
    const float normalVelocity = dot(relativeVelocity, n);
    float lambda;
    if (con.stiffness > 0.0f) {
      lambda = -(con.stiffness * C + con.damping * normalVelocity) * context.dt; // penalty spring
    } else {
      const float bias = 0.2f * inverseTimestep * C; // rigid: Baumgarte position feedback
      lambda = -m * (normalVelocity + bias);
    }
    con.impulse = lambda;
    const Vec3 P = n * lambda;
    applyImpulse(w, inverseInertia, con.a, -P, rA);
    applyImpulse(w, inverseInertia, con.b, P, rB);
  }
}

// Position projection out of residual overlap (no velocity change => no energy).
void correctPositions(World &w, const std::vector<SolverContact> &contacts,
                      const StepContext &context) {
  for (const SolverContact &contact : contacts) {
    const float inverseMassSum = w.bodies[contact.a].inverseMass + w.bodies[contact.b].inverseMass;
    if (inverseMassSum <= 0.0f) {
      continue;
    }
    float maxPenetration = 0.0f;
    for (int c = 0; c < contact.count; ++c) {
      maxPenetration = std::max(maxPenetration, contact.points[c].penetration);
    }
    const float corr = std::max(maxPenetration - context.penetrationSlop, 0.0f) *
                       context.baumgarte / inverseMassSum;
    const Vec3 delta = contact.normal * corr;
    w.transforms[contact.a].position -= delta * w.bodies[contact.a].inverseMass;
    w.transforms[contact.b].position += delta * w.bodies[contact.b].inverseMass;
  }
}

// Persist the accumulated impulses for next step's warm start. `scratch` is a
// caller-owned buffer we build into and then swap with the cache's storage, so a
// steady simulation recycles the same two allocations forever instead of
// allocating a fresh cache each substep.
void storeImpulses(World &w, const std::vector<SolverContact> &contacts,
                   std::vector<PersistentManifold> &scratch) {
  scratch.clear();
  scratch.reserve(contacts.size());
  for (const SolverContact &contact : contacts) {
    PersistentManifold m;
    m.key = makeKey(contact.a, contact.b);
    m.count = contact.count;
    m.lastStep = w.stepCount;
    m.normal = contact.normal;
    Vec3 centroid{};
    float impulse = 0.0f;
    for (int c = 0; c < contact.count; ++c) {
      m.points[c] = {contact.points[c].id, contact.points[c].normalImpulse,
                     contact.points[c].tangentImpulse1, contact.points[c].tangentImpulse2};
      centroid += w.transforms[contact.a].position + contact.points[c].rA; // world contact point
      impulse += contact.points[c].normalImpulse;
    }
    if (contact.count > 0) {
      centroid *= 1.0f / static_cast<float>(contact.count);
    }
    m.point = centroid;
    m.totalImpulse = impulse;
    scratch.push_back(m);
  }
  // The cache must stay sorted by key (its `find` binary-searches). Contacts are
  // built in `hitPairs` order, a subsequence of the broadphase pair list, which
  // is emitted ascending with first < second — so `scratch` is already sorted by
  // key and no explicit sort is needed. Swap it into place (old storage stays in
  // `scratch` for reuse next substep).
  std::swap(w.contactCache.manifolds, scratch);
}

// Reusable per-substep scratch. Kept deliberately OUTSIDE `World`: `World` is a
// value type — copied every frame for render interpolation and compared with
// `operator==` in the golden tests — so its size and identity must not depend on
// transient solver buffers. `step` owns one workspace per thread and recycles
// its allocations across steps, so a steady simulation allocates nothing here.
struct StepWorkspace {
  std::vector<Mat3> inverseInertia;
  std::vector<Pair> pairs;                       // broadphase output
  std::vector<AABB> boxes;                       // broadphase scratch
  std::vector<Entity> order;                     // broadphase scratch
  std::vector<std::optional<Collision>> results; // narrowphase output, per pair
  std::vector<Collision> manifolds;              // hits only
  std::vector<Pair> hitPairs;                    // pairs parallel to `manifolds`
  std::vector<SolverContact> contacts;
  std::vector<PersistentManifold> cacheScratch; // storeImpulses ping-pong buffer
};

// The zero inverse-inertia tensor shared by all static/infinite-mass bodies.
const Mat3 kZeroInertia{Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.0f}};

// One fixed sub-step at context.dt (which the caller has already set to h).
void subStep(World &w, const StepContext &context, StepWorkspace &ws) {
  const Entity n = w.count();
  ws.inverseInertia.resize(n);
  for (Entity i = 0; i < n; ++i) {
    // Statics carry a zero inverse-inertia tensor (infinite mass), for which the
    // R·I·Rᵀ triple product is just the zero matrix — skip it.
    ws.inverseInertia[i] = w.bodies[i].type == BodyType::Dynamic
                               ? worldInverseInertia(w.bodies[i], w.transforms[i])
                               : kZeroInertia;
  }

  broadphaseInto(w, context, ws.pairs, ws.boxes, ws.order);
  // Narrowphase in parallel: each pair's manifold is independent. Results are
  // written per-index then consumed in order, so the outcome is identical for
  // any threadCount (deterministic).
  ws.results.assign(ws.pairs.size(), std::nullopt);
  parallelFor(ws.pairs.size(), context.threadCount, [&](std::size_t k) {
    ws.results[k] = collide(w.transforms[ws.pairs[k].first], w.colliders[ws.pairs[k].first],
                            w.transforms[ws.pairs[k].second], w.colliders[ws.pairs[k].second],
                            context.speculativeDistance);
  });
  ws.manifolds.clear();
  ws.hitPairs.clear();
  for (std::size_t k = 0; k < ws.pairs.size(); ++k) {
    if (ws.results[k]) {
      ws.manifolds.push_back(*ws.results[k]);
      ws.hitPairs.push_back(ws.pairs[k]);
    }
  }

  // Build contacts (which captures each contact's approach velocity for
  // restitution) BEFORE applying gravity, so restitution reflects the true
  // incoming speed. Applying gravity first would inflate the approach velocity
  // by g*h and inject energy on every bounce (a bug the review caught).
  prepareContacts(w, ws.manifolds, ws.hitPairs, ws.inverseInertia, ws.contacts);

  integrateVelocities(w, context); // gravity, after the approach speed is recorded
  warmStart(w, ws.inverseInertia, ws.contacts);
  solveVelocity(w, ws.inverseInertia, ws.contacts, context);
  solveConstraints(w, ws.inverseInertia, context);
  applyRestitution(w, ws.inverseInertia, ws.contacts, context);
  integratePositions(w, context);
  correctPositions(w, ws.contacts, context);
  storeImpulses(w, ws.contacts, ws.cacheScratch);
  ++w.stepCount;
}

} // namespace
} // namespace Physics

export namespace Physics {

void step(World &w, const StepContext &context) {
  // One recycled workspace per thread — see `StepWorkspace`. `thread_local` keeps
  // it safe if independent worlds are stepped on different threads.
  static thread_local StepWorkspace ws;
  const int subSteps = context.subStepCount > 0 ? context.subStepCount : 1;
  StepContext sub = context;
  sub.dt = context.dt / static_cast<float>(subSteps);
  for (int i = 0; i < subSteps; ++i) {
    subStep(w, sub, ws);
  }
}

} // namespace Physics
