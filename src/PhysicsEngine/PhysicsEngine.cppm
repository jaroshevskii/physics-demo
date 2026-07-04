export module PhysicsEngine;

export import :Math;
export import :Components;
export import :ContactCache;
export import :World;
export import :Collision;
export import :Broadphase;
export import :Threading;
export import :Systems;
export import :Query;
export import :Forces;
export import :Events;
export import :Scheduler;

import std;

// Umbrella module for the engine. It `export import`s every engine partition
// (Math, Components, World, Collision, Systems, ...) so a single `import
// PhysicsEngine;` brings in the whole `Physics` API. Beyond re-exporting, it
// offers a few whole-world analysis helpers that the property/invariant tests
// assert against (energy must not grow when restitution < 1, momentum is
// conserved in an elastic collision, etc.).
export namespace Physics {

/// Mass of a body: reciprocal of `inverseMass`, or 0 for static/infinite-mass
/// bodies (`inverseMass == 0`).
inline float mass(const Body &b) { return b.inverseMass > 0.0f ? 1.0f / b.inverseMass : 0.0f; }

/// Translational kinetic energy of the whole world (static bodies contribute 0).
inline float linearKineticEnergy(const World &w) {
  float ke = 0.0f;
  for (Entity i = 0; i < w.count(); ++i) {
    ke += 0.5f * mass(w.bodies[i]) * lengthSquared(w.motions[i].linearVelocity);
  }
  return ke;
}

/// Gravitational potential energy: PE = -Σ mᵢ · (g · xᵢ).
inline float potentialEnergy(const World &w, Vec3 gravity) {
  float pe = 0.0f;
  for (Entity i = 0; i < w.count(); ++i) {
    pe += -mass(w.bodies[i]) * dot(gravity, w.transforms[i].position);
  }
  return pe;
}

/// Total linear momentum of the whole world: Σ mᵢ · vᵢ.
inline Vec3 linearMomentum(const World &w) {
  Vec3 p;
  for (Entity i = 0; i < w.count(); ++i) {
    p += w.motions[i].linearVelocity * mass(w.bodies[i]);
  }
  return p;
}

} // namespace Physics
