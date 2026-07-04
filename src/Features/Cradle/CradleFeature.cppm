export module CradleFeature;

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;

// Demo — "Newton's Cradle". A row of touching spheres, each hung from an anchor
// by a rigid distance constraint (a pendulum). The end ball starts pulled aside;
// released, it swings in and momentum transfers through the line so the far ball
// swings out. This is the definitive showcase of the solver upgrades: it needs
// warm-started resting contacts, near-elastic restitution, and clean momentum
// transfer through a chain of touching bodies.
//
// It also demonstrates the RealityKit-style event -> Composable-Architecture
// Action loop: each `Began` collision the engine reports is re-sent as a
// `CollisionOccurred` Action (the "click"), which the reducer counts.
export namespace CradleFeature {

namespace Config {
constexpr int ballCount = 5;
constexpr float radius = 0.5f;
constexpr float stringLength = 4.0f;
constexpr float topY = 6.0f;
constexpr float pullAngle = 0.9f; // radians the end ball starts pulled aside
} // namespace Config

struct State {
  Sim::State sim;
  int impactCount = 0; // number of Began collisions seen (the "clicks")
  bool operator==(const State &) const = default;
};

struct Ticked {};
struct ResetTapped {};
struct TogglePause {};
struct CollisionOccurred {
  Physics::CollisionEvent event;
  bool operator==(const CollisionOccurred &) const = default;
};

using Action = std::variant<Ticked, ResetTapped, TogglePause, CollisionOccurred>;

inline State initialState() {
  State s;
  s.sim.gravity = {0.0f, -9.81f, 0.0f};
  Physics::World &w = s.sim.world;
  const float spacing = 2.0f * Config::radius; // touching
  const float x0 = -(Config::ballCount - 1) * spacing * 0.5f;
  for (int i = 0; i < Config::ballCount; ++i) {
    const float ax = x0 + static_cast<float>(i) * spacing;
    const Physics::Entity anchor =
        Physics::addStaticBox(w, {ax, Config::topY, 0}, {0.05f, 0.05f, 0.05f}, 0.0f, 0.0f);
    Physics::Vec3 ballPos{ax, Config::topY - Config::stringLength, 0};
    if (i == 0) {
      // Pull the end ball aside (up and to the left) by pullAngle.
      ballPos = {ax - std::sin(Config::pullAngle) * Config::stringLength,
                 Config::topY - std::cos(Config::pullAngle) * Config::stringLength, 0};
    }
    const Physics::Entity ball = Physics::addSphere(w, ballPos, Config::radius, 1.0f, 0.97f,
                                                    0.0f); // near-elastic, frictionless
    Physics::addDistanceConstraint(w, anchor, ball, Config::stringLength);
  }
  s.sim.previous = w;
  return s;
}

ComposableArchitecture::Feature<State, Action> body();

} // namespace CradleFeature
