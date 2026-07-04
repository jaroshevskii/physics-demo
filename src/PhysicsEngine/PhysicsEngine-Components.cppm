export module PhysicsEngine:Components;

import std;
import :Math;

// Value-type components, in the spirit of RealityKit's ECS: plain data, no
// behavior. Systems (see :Systems) are the functions that act on them. The
// component split mirrors RealityKit's:
//   Transform          — pose (position + orientation)
//   Collider           — collision shape (RealityKit's CollisionComponent)
//   Body               — mass/material/mode (RealityKit's PhysicsBodyComponent)
//   Motion             — linear/angular velocity (RealityKit's PhysicsMotionComponent)
export namespace Physics {

/// How a body responds to forces. A `Static` body is immovable: infinite mass,
/// modeled as `inverseMass == 0` and a zero inverse-inertia, so the solver needs
/// no special cases (a dynamic body bounces off it as if it had infinite mass).
/// Walls and floors are `Static`.
enum class BodyType : std::uint8_t { Dynamic, Static };

/// The kind of collision shape a `Collider` holds.
enum class ShapeType : std::uint8_t { Sphere, Box };

/// A body's pose: position plus orientation (RealityKit's `Transform`).
struct Transform {
  Vec3 position;
  Quat orientation;
  bool operator==(const Transform &) const = default;
};

/// A collision shape — a sphere or a box (RealityKit's `CollisionComponent`).
struct Collider {
  ShapeType type = ShapeType::Sphere;
  /// Sphere radius (used when `type == Sphere`).
  float radius = 0.5f;
  /// Box half-extents along each axis (used when `type == Box`).
  Vec3 halfExtents{0.5f, 0.5f, 0.5f};
  bool operator==(const Collider &) const = default;
};

/// Mass, material, and mode (RealityKit's `PhysicsBodyComponent`).
struct Body {
  BodyType type = BodyType::Dynamic;
  /// Reciprocal mass; `0` means static / infinite mass.
  float inverseMass = 1.0f;
  /// Inverse inertia tensor in body space (`0` => static).
  Mat3 inverseInertiaBody;
  /// Bounciness `e` in `[0, 1]`.
  float restitution = 0.2f;
  /// Coulomb friction coefficient.
  float friction = 0.5f;
  bool operator==(const Body &) const = default;
};

/// Linear and angular velocity (RealityKit's `PhysicsMotionComponent`).
struct Motion {
  Vec3 linearVelocity;
  Vec3 angularVelocity;
  bool operator==(const Motion &) const = default;
};

// --- Mass properties ---------------------------------------------------------
//
// Mass and inertia are derived from a collider's shape and a density, so a demo
// only picks a size + material and the physically-correct mass/inertia follow.

/// Mass of a collider filled at a uniform `density` (sphere volume or box volume).
inline float shapeMass(const Collider &c, float density) {
  switch (c.type) {
  case ShapeType::Sphere:
    return density * (4.0f / 3.0f) * std::numbers::pi_v<float> * c.radius * c.radius * c.radius;
  case ShapeType::Box: {
    const Vec3 h = c.halfExtents;
    return density * 8.0f * h.x * h.y * h.z;
  }
  }
  return 0.0f;
}

/// Inverse inertia tensor in body space (diagonal). Solid sphere: `I = 2/5 m r²`.
/// Solid box with half-extents `h`: `I_x = 1/3 m (h_y² + h_z²)`, and cyclically.
inline Mat3 shapeInvInertia(const Collider &c, float mass) {
  if (mass <= 0.0f) {
    return diagonal(0.0f, 0.0f, 0.0f);
  }
  switch (c.type) {
  case ShapeType::Sphere: {
    const float i = (2.0f / 5.0f) * mass * c.radius * c.radius;
    const float inv = 1.0f / i;
    return diagonal(inv, inv, inv);
  }
  case ShapeType::Box: {
    const Vec3 h = c.halfExtents;
    const float ix = (1.0f / 3.0f) * mass * (h.y * h.y + h.z * h.z);
    const float iy = (1.0f / 3.0f) * mass * (h.x * h.x + h.z * h.z);
    const float iz = (1.0f / 3.0f) * mass * (h.x * h.x + h.y * h.y);
    return diagonal(1.0f / ix, 1.0f / iy, 1.0f / iz);
  }
  }
  return diagonal(0.0f, 0.0f, 0.0f);
}

/// Builds a dynamic body, deriving its mass and inertia from a collider + density.
inline Body dynamicBody(const Collider &c, float density, float restitution, float friction) {
  const float mass = shapeMass(c, density);
  return Body{.type = BodyType::Dynamic,
              .inverseMass = mass > 0.0f ? 1.0f / mass : 0.0f,
              .inverseInertiaBody = shapeInvInertia(c, mass),
              .restitution = restitution,
              .friction = friction};
}

/// Builds an immovable static body (infinite mass, zero inverse-inertia).
inline Body staticBody(float restitution, float friction) {
  return Body{.type = BodyType::Static,
              .inverseMass = 0.0f,
              .inverseInertiaBody = diagonal(0.0f, 0.0f, 0.0f),
              .restitution = restitution,
              .friction = friction};
}

} // namespace Physics
