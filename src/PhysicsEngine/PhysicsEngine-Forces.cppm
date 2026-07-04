export module PhysicsEngine:Forces;

import std;
import :Math;
import :Components;
import :World;

// Force effects, in the spirit of RealityKit's ForceEffect / PhysicsForceEffect.
// These are pure functions over the world, applied before the solver runs.
//   * applyRadialImpulse — an explosion: an outward impulse on every dynamic
//     body within a radius, falling off with distance.
//   * applyDrag — a global velocity damping (air resistance), folded in the
//     stable (implicit) way so it can never overshoot past zero.
export namespace Physics {

/// Applies an instantaneous outward impulse to every dynamic body within
/// `radius` of `center` — an explosion, in the spirit of a RealityKit
/// `ForceEffect`. `falloffExp` shapes the fade: strength * (1 - dist/radius)^
/// falloffExp. Impulse mode => heavier bodies move less (divided by mass),
/// which reads as physically correct debris.
inline void applyRadialImpulse(World &w, Vec3 center, float strength, float radius,
                               float falloffExp = 1.0f) {
  for (Entity i = 0; i < w.count(); ++i) {
    if (w.bodies[i].type != BodyType::Dynamic) {
      continue;
    }
    const Vec3 d = w.transforms[i].position - center;
    const float dist = length(d);
    if (dist >= radius) {
      continue;
    }
    const Vec3 dir = dist > 1e-4f ? d / dist : Vec3{0.0f, 1.0f, 0.0f};
    const float f = strength * std::pow(clampf(1.0f - dist / radius, 0.0f, 1.0f), falloffExp);
    w.motions[i].linearVelocity += dir * (f * w.bodies[i].inverseMass);
  }
}

/// Damps every dynamic body's linear and angular velocity over a timestep (air
/// resistance): v *= 1/(1 + dt*k). Folded in the implicit way, so it is
/// unconditionally stable and can never overshoot past zero.
inline void applyDrag(World &w, float linearDrag, float angularDrag, float dt) {
  const float ls = 1.0f / (1.0f + dt * linearDrag);
  const float as = 1.0f / (1.0f + dt * angularDrag);
  for (Entity i = 0; i < w.count(); ++i) {
    if (w.bodies[i].type != BodyType::Dynamic) {
      continue;
    }
    w.motions[i].linearVelocity *= ls;
    w.motions[i].angularVelocity *= as;
  }
}

} // namespace Physics
