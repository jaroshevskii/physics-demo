export module ShooterFeature;

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;

// Demo — "Shooting Gallery" (first-person). You look around with a yaw/pitch
// camera and fire fast, heavy sphere "bullets" that knock down a wall of light
// boxes. It exercises speculative contacts (fast bullets don't tunnel through
// thin boxes) and impulse/momentum transfer (a small fast projectile scatters a
// stack). The FPS camera lives in the feature State, so shooting is deterministic
// and testable (the bullet's spawn velocity is a pure function of the camera).
export namespace ShooterFeature {

namespace Config {
constexpr float bulletRadius = 0.2f;
constexpr float bulletSpeed = 55.0f;
constexpr float bulletDensity = 8.0f; // heavy: carries momentum to knock boxes
constexpr float moveSpeed = 9.0f;     // units per second (app scales by dt)
constexpr float lookSpeed = 0.0035f;  // radians per mouse pixel
constexpr int wallWide = 5;
constexpr int wallTall = 4;
constexpr float box = 0.5f; // half-extent
constexpr int maxBullets = 120;
} // namespace Config

struct State {
  Sim::State sim;
  Physics::Vec3 cameraPosition{0.0f, 2.0f, 12.0f};
  float yaw = 0.0f;   // around +Y; 0 looks toward -Z
  float pitch = 0.0f; // around the camera's right axis
  int bulletCount = 0;
  bool operator==(const State &) const = default;
};

struct Ticked {};
struct MoveLocal {
  float forward = 0.0f; // signed distance this frame
  float right = 0.0f;
  bool operator==(const MoveLocal &) const = default;
};
struct LookDelta {
  float dyaw = 0.0f;
  float dpitch = 0.0f;
  bool operator==(const LookDelta &) const = default;
};
struct Shoot {};
struct ResetTapped {};
struct TogglePause {};

using Action = std::variant<Ticked, MoveLocal, LookDelta, Shoot, ResetTapped, TogglePause>;

// Camera forward direction from yaw/pitch (yaw 0, pitch 0 => -Z).
inline Physics::Vec3 forwardDirection(float yaw, float pitch) {
  return {std::sin(yaw) * std::cos(pitch), std::sin(pitch), -std::cos(yaw) * std::cos(pitch)};
}

inline void buildScene(Sim::State &sim) {
  sim.gravity = {0, -9.81f, 0};
  Physics::World &w = sim.world;
  Physics::addStaticBox(w, {0, -0.5f, 0}, {20, 0.5f, 20}, 0.3f, 0.7f); // floor
  Physics::addStaticBox(w, {0, 6, -8}, {20, 6, 0.5f}, 0.2f, 0.6f);     // back wall (catcher)
  // A wall of light boxes to knock down, centred at the origin facing the camera.
  const float b = Config::box;
  const float x0 = -(Config::wallWide - 1) * b;
  for (int row = 0; row < Config::wallTall; ++row) {
    for (int col = 0; col < Config::wallWide; ++col) {
      Physics::addBox(
          w, {x0 + static_cast<float>(col) * 2.0f * b, b + static_cast<float>(row) * 2.0f * b, 0},
          {b, b, b}, 0.4f, 0.1f, 0.5f);
    }
  }
}

inline State initialState() {
  State s;
  buildScene(s.sim);
  s.sim.previous = s.sim.world;
  return s;
}

ComposableArchitecture::Feature<State, Action> body();

} // namespace ShooterFeature
