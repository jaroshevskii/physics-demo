export module BallPitFeature;

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;

// Demo B — "Ball Pit". Many spheres of differing radius (hence differing mass)
// rain into an open box, pile up, and bounce off one another and the walls with
// per-body restitution. Spheres are the most stable many-body case, so this is
// the reliable large-count showcase. Same feature shape as SandboxFeature.
export namespace BallPitFeature {

namespace Config {
constexpr float arenaHalf = 6.0f;
constexpr float wallHeight = 6.0f;
constexpr float wallThick = 0.5f;
constexpr int maxBodies = 400; // spawning stops here to keep it real-time
} // namespace Config

struct State {
  Sim::State sim;
  int spawnCount = 0;
  bool operator==(const State &) const = default;
};

// --- Actions -----------------------------------------------------------------
struct Ticked {};
struct SpawnSphere {
  Physics::Vec3 position;
  float radius = 0.4f;
  float density = 1.0f;
  float restitution = 0.5f;
  Physics::Vec3 velocity;
  bool operator==(const SpawnSphere &) const = default;
};
struct ResetTapped {};
struct TogglePause {};

using Action = std::variant<Ticked, SpawnSphere, ResetTapped, TogglePause>;

// The open-topped box the spheres collect in (floor + four walls).
inline void buildBowl(Sim::State &sim) {
  const float a = Config::arenaHalf, t = Config::wallThick, h = Config::wallHeight;
  Physics::addStaticBox(sim.world, {0, -t, 0}, {a, t, a}, 0.4f, 0.4f);
  Physics::addStaticBox(sim.world, {a + t, h, 0}, {t, h, a}, 0.4f, 0.4f);
  Physics::addStaticBox(sim.world, {-a - t, h, 0}, {t, h, a}, 0.4f, 0.4f);
  Physics::addStaticBox(sim.world, {0, h, a + t}, {a, h, t}, 0.4f, 0.4f);
  Physics::addStaticBox(sim.world, {0, h, -a - t}, {a, h, t}, 0.4f, 0.4f);
}

inline State initialState() {
  State s;
  buildBowl(s.sim);
  s.sim.previous = s.sim.world;
  return s;
}

// Whether another sphere may be spawned (keeps the body count bounded).
inline bool canSpawn(const State &s) {
  return static_cast<int>(s.sim.world.count()) < Config::maxBodies;
}

// Reducer defined in the module implementation unit (BallPitFeature.cpp).
ComposableArchitecture::Feature<State, Action> body();

} // namespace BallPitFeature
