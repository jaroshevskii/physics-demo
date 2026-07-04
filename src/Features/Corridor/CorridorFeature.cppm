export module CorridorFeature;

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;

// Experimental "technical" demo — a long closed corridor that fills with
// hundreds of spheres while the gravity direction slowly precesses, so the whole
// mass of balls perpetually sloshes from wall to wall and "reconfigures" itself.
// It's a stress test for the engine (many bodies) and for instanced GPU
// rendering (see the Metal renderer). Pure Composable-Architecture feature: the
// physics here is engine-only and testable; the Metal app just draws its World.
export namespace CorridorFeature {

namespace Config {
constexpr float halfX = 4.0f;  // corridor cross-section (x)
constexpr float halfY = 4.0f;  // cross-section (y)
constexpr float halfZ = 12.0f; // corridor length (z)
constexpr float wallThick = 0.5f;
constexpr float ballRadius = 0.32f;
constexpr int maxBalls = 1400; // spawn cap (keeps it real-time)
constexpr int burst = 40;      // balls per spawn burst
} // namespace Config

struct State {
  Sim::State sim;
  int ballCount = 0;
  std::uint32_t bursts = 0;
  bool operator==(const State &) const = default;
};

struct Ticked {};
struct SpawnBurst {};
struct ResetTapped {};
struct TogglePause {};

using Action = std::variant<Ticked, SpawnBurst, ResetTapped, TogglePause>;

// The six static walls of the closed corridor (a long box).
inline void buildCorridor(Sim::State &sim) {
  const float x = Config::halfX, y = Config::halfY, z = Config::halfZ, t = Config::wallThick;
  const float e = 0.3f, f = 0.4f;                                    // restitution, friction
  Physics::addStaticBox(sim.world, {0, -y - t, 0}, {x, t, z}, e, f); // floor
  Physics::addStaticBox(sim.world, {0, y + t, 0}, {x, t, z}, e, f);  // ceiling
  Physics::addStaticBox(sim.world, {-x - t, 0, 0}, {t, y, z}, e, f); // -x wall
  Physics::addStaticBox(sim.world, {x + t, 0, 0}, {t, y, z}, e, f);  // +x wall
  Physics::addStaticBox(sim.world, {0, 0, -z - t}, {x, y, t}, e, f); // -z cap
  Physics::addStaticBox(sim.world, {0, 0, z + t}, {x, y, t}, e, f);  // +z cap
}

// Gravity that slowly precesses (rotates) as a function of the step count, so it
// is deterministic and makes the ball mass continuously flow around the tube.
inline Physics::Vec3 precessingGravity(std::uint64_t stepCount) {
  const float angle = static_cast<float>(stepCount) * 0.010f;
  const float drift = static_cast<float>(stepCount) * 0.0037f;
  const float magnitude = 16.0f;
  return {std::sin(angle) * magnitude, -std::cos(angle) * magnitude,
          std::sin(drift) * magnitude * 0.35f};
}

inline bool canSpawn(const State &s) {
  return static_cast<int>(s.sim.world.count()) < Config::maxBalls + 6; // +6 static walls
}

inline State initialState() {
  State s;
  s.sim.gravity = {0, -16.0f, 0};
  s.sim.threadCount = 8; // many bodies: parallelize the narrowphase
  buildCorridor(s.sim);
  s.sim.previous = s.sim.world;
  return s;
}

ComposableArchitecture::Feature<State, Action> body();

} // namespace CorridorFeature
