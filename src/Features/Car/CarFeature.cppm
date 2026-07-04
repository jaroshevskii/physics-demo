export module CarFeature;

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;

// Demo — "Raycast Car". A single rigid-body chassis floats on four raycast
// wheels: each frame a ray is cast down from each wheel mount, and a spring +
// damper suspension force (plus throttle and lateral grip) is applied to the
// chassis at that point. This is the classic arcade-vehicle model (Bullet's
// btRaycastVehicle): the wheels are not rigid bodies, just rays + forces.
// Exercises raycasting and applying forces/torques to a body.
export namespace CarFeature {

namespace Config {
constexpr Physics::Vec3 chassisHalf{1.0f, 0.35f, 2.0f};
constexpr float wheelRadius = 0.45f;
constexpr float restLength = 0.55f;    // suspension rest travel
constexpr float stiffness = 260.0f;    // spring constant (per wheel)
constexpr float damping = 40.0f;       // suspension damper (near-critical: kills spring bounce)
constexpr float driveForce = 42.0f;    // throttle force
constexpr float grip = 0.30f;          // lateral grip per wheel (0..1)
constexpr float steerAngle = 0.5f;     // max steering (radians)
constexpr Physics::Entity chassis = 1; // entity 0 is the floor
} // namespace Config

struct State {
  Sim::State sim;
  float throttle = 0.0f; // -1..1
  float steer = 0.0f;    // -1..1
  bool operator==(const State &) const = default;
};

struct Ticked {};
struct SetControls {
  float throttle = 0.0f;
  float steer = 0.0f;
  bool operator==(const SetControls &) const = default;
};
struct ResetTapped {};
struct TogglePause {};

using Action = std::variant<Ticked, SetControls, ResetTapped, TogglePause>;

// The four wheel mount points in chassis-local space (FL, FR, RL, RR).
inline std::array<Physics::Vec3, 4> wheelMounts() {
  const Physics::Vec3 h = Config::chassisHalf;
  return {Physics::Vec3{-h.x, -h.y * 0.5f, h.z * 0.7f}, Physics::Vec3{h.x, -h.y * 0.5f, h.z * 0.7f},
          Physics::Vec3{-h.x, -h.y * 0.5f, -h.z * 0.7f},
          Physics::Vec3{h.x, -h.y * 0.5f, -h.z * 0.7f}};
}

// Applies suspension + drive + grip forces to the chassis for one frame `dt`.
// Pure over the world; called before the solver integrates. Returns nothing —
// it mutates the chassis's velocity/angular velocity via impulses.
inline void applyVehicle(Physics::World &w, float throttle, float steer, float dt) {
  using namespace Physics;
  const Entity ch = Config::chassis;
  if (ch >= w.count()) {
    return;
  }
  Transform &t = w.transforms[ch];
  Motion &motion = w.motions[ch];
  const Body &body = w.bodies[ch];
  const Mat3 R = toMat3(t.orientation);
  const Vec3 up = R.c1;
  const Vec3 forward = R.c2;
  const Mat3 inverseInertia = R * body.inverseInertiaBody * transpose(R);
  const float maxSuspensionLength = Config::restLength + Config::wheelRadius;

  const auto mounts = wheelMounts();
  for (int i = 0; i < 4; ++i) {
    const bool front = i < 2;
    const Vec3 worldMount = t.position + R * mounts[i];
    const Vec3 down = -up;
    const auto hit = raycastNearest(w, worldMount, down, maxSuspensionLength, ch);
    if (!hit) {
      continue;
    }
    const float compression = maxSuspensionLength - hit->distance;
    if (compression <= 0.0f) {
      continue;
    }
    const Vec3 n = hit->normal;
    const Vec3 r = worldMount - t.position;
    const Vec3 velocityAtWheel = motion.linearVelocity + cross(motion.angularVelocity, r);

    // Suspension spring + damper along the ground normal.
    const float normalVelocity = dot(velocityAtWheel, n);
    const float springForce =
        std::max(Config::stiffness * compression - Config::damping * normalVelocity, 0.0f);
    const Vec3 suspensionImpulse = n * (springForce * dt);

    // Wheel forward (front wheels steer), projected onto the ground plane.
    Vec3 wheelForward =
        front ? rotate(fromAxisAngle(up, steer * Config::steerAngle), forward) : forward;
    wheelForward = normalized(wheelForward - n * dot(wheelForward, n));
    const Vec3 wheelRight = normalized(cross(n, wheelForward));

    const Vec3 driveImpulse = wheelForward * (throttle * Config::driveForce * dt);
    const float lateralVelocity = dot(velocityAtWheel, wheelRight);
    const Vec3 gripImpulse = wheelRight * (-lateralVelocity * Config::grip /
                                           (body.inverseMass > 0 ? body.inverseMass : 1));

    const Vec3 impulse = suspensionImpulse + driveImpulse + gripImpulse;
    motion.linearVelocity += impulse * body.inverseMass;
    motion.angularVelocity += inverseInertia * cross(r, impulse);
  }
}

inline State initialState() {
  State s;
  s.sim.gravity = {0, -9.81f, 0};
  Physics::World &w = s.sim.world;
  Physics::addStaticBox(w, {0, -0.5f, 0}, {30, 0.5f, 30}, 0.4f, 0.8f);      // ground (entity 0)
  Physics::addBox(w, {0, 1.05f, 0}, Config::chassisHalf, 1.0f, 0.1f, 0.6f); // chassis (entity 1)
  // A stack of light cubes directly AHEAD of the car (the chassis drives toward
  // +z), so driving straight ploughs into them head-on and scatters them — the
  // collision is the point, so it must be in the car's path, not off to the side.
  for (int depth = 0; depth < 3; ++depth) {
    for (int col = -1; col <= 1; ++col) {
      for (int height = 0; height < 2; ++height) {
        Physics::addBox(w,
                        {static_cast<float>(col) * 1.05f, 0.5f + static_cast<float>(height) * 1.05f,
                         8.0f + static_cast<float>(depth) * 1.05f},
                        {0.5f, 0.5f, 0.5f}, 0.3f, 0.2f, 0.5f);
      }
    }
  }
  s.sim.previous = w;
  return s;
}

ComposableArchitecture::Feature<State, Action> body();

} // namespace CarFeature
