export module SandboxFeature;

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;

// Demo A — "Cubes". A handful of oriented boxes fall into a walled arena,
// collide, and bounce; the user can spawn more and toss them. A Composable-
// Architecture feature: pure `State`, a `body()` reducer, time injected through
// the `\.date` dependency so the whole thing runs deterministically in tests.
export namespace SandboxFeature {

namespace Config {
constexpr float arenaHalf = 6.0f; // half-width of the floor
constexpr float wallHeight = 4.0f;
constexpr float wallThick = 0.5f;
} // namespace Config

struct State {
  Sim::State sim;
  int spawnCount = 0;
  bool operator==(const State &) const = default;
};

// --- Actions -----------------------------------------------------------------
struct Ticked {}; // advance the simulation to the current wall-clock time
struct SpawnBox {
  Physics::Vec3 position;
  Physics::Vec3 halfExtents{0.5f, 0.5f, 0.5f};
  float density = 1.0f;
  Physics::Vec3 velocity;
  Physics::Quat orientation;
  bool operator==(const SpawnBox &) const = default;
};
struct ResetTapped {};
struct TogglePause {};

using Action = std::variant<Ticked, SpawnBox, ResetTapped, TogglePause>;

// Builds the static arena (floor + four walls) — all infinite-mass bodies.
inline void buildArena(Sim::State &sim) {
  const float a = Config::arenaHalf, t = Config::wallThick, h = Config::wallHeight;
  Physics::addStaticBox(sim.world, {0, -t, 0}, {a, t, a}, 0.3f, 0.6f);     // floor
  Physics::addStaticBox(sim.world, {a + t, h, 0}, {t, h, a}, 0.3f, 0.6f);  // +x wall
  Physics::addStaticBox(sim.world, {-a - t, h, 0}, {t, h, a}, 0.3f, 0.6f); // -x wall
  Physics::addStaticBox(sim.world, {0, h, a + t}, {a, h, t}, 0.3f, 0.6f);  // +z wall
  Physics::addStaticBox(sim.world, {0, h, -a - t}, {a, h, t}, 0.3f, 0.6f); // -z wall
}

inline State initialState() {
  State s;
  buildArena(s.sim);
  // A few starter cubes stacked slightly askew, so they topple and collide.
  Physics::addBox(s.sim.world, {0, 1.0f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f, 0.2f, 0.5f);
  Physics::addBox(s.sim.world, {0.3f, 2.2f, 0.1f}, {0.5f, 0.5f, 0.5f}, 1.0f, 0.2f, 0.5f,
                  Physics::fromAxisAngle({0, 0, 1}, 0.3f));
  Physics::addBox(s.sim.world, {-0.2f, 3.4f, -0.1f}, {0.5f, 0.5f, 0.5f}, 1.0f, 0.2f, 0.5f,
                  Physics::fromAxisAngle({1, 0, 0}, 0.4f));
  s.sim.previous = s.sim.world;
  return s;
}

// The reducer. Defined in the module implementation unit (SandboxFeature.cpp)
// so the visitor's internal helpers are not exposed through the module
// interface — editing the reducer also recompiles only that one object.
ComposableArchitecture::Feature<State, Action> body();

} // namespace SandboxFeature
