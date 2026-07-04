export module ExplosionFeature;

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;

// Demo — "Explosions". A wall of stacked boxes sits in a walled arena; a
// `Detonate` applies a radial impulse (RealityKit-style force effect) that blows
// the debris outward. Exercises force fields, the sub-stepped solver (lots of
// fast bodies), and collision events (each debris impact is counted).
export namespace ExplosionFeature {

namespace Config {
constexpr int wallWide = 6;
constexpr int wallTall = 5;
constexpr float box = 0.5f; // half-extent
constexpr float arenaHalf = 9.0f;
} // namespace Config

struct State {
  Sim::State sim;
  int impacts = 0;
  bool operator==(const State &) const = default;
};

struct Ticked {};
struct Detonate {
  Physics::Vec3 center{0, 1, 0};
  float strength = 42.0f;
  float radius = 9.0f;
  bool operator==(const Detonate &) const = default;
};
struct ResetTapped {};
struct TogglePause {};
struct CollisionOccurred {
  Physics::CollisionEvent event;
  bool operator==(const CollisionOccurred &) const = default;
};

using Action = std::variant<Ticked, Detonate, ResetTapped, TogglePause, CollisionOccurred>;

inline State initialState() {
  State s;
  s.sim.gravity = {0, -9.81f, 0};
  Physics::World &w = s.sim.world;
  const float a = Config::arenaHalf;
  Physics::addStaticBox(w, {0, -0.5f, 0}, {a, 0.5f, a}, 0.3f, 0.7f); // floor
  // A wall of boxes centered at the origin.
  const float b = Config::box;
  const float x0 = -(Config::wallWide - 1) * b;
  for (int iy = 0; iy < Config::wallTall; ++iy) {
    for (int ix = 0; ix < Config::wallWide; ++ix) {
      const float x = x0 + static_cast<float>(ix) * 2.0f * b;
      const float y = b + static_cast<float>(iy) * 2.0f * b;
      Physics::addBox(w, {x, y, 0}, {b, b, b}, 1.0f, 0.1f, 0.6f);
    }
  }
  s.sim.previous = w;
  return s;
}

ComposableArchitecture::Feature<State, Action> body();

} // namespace ExplosionFeature
