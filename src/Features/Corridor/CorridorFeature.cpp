module CorridorFeature;

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;

namespace CorridorFeature {
namespace {
template <class... Ts> struct overloaded : Ts... {
  using Ts::operator()...;
};

// Deterministic PRNG so bursts are reproducible.
float randomFloat(std::uint32_t &state) {
  state = state * 1664525u + 1013904223u;
  return static_cast<float>(state >> 8) * (1.0f / 16777216.0f);
}
} // namespace

ComposableArchitecture::Feature<State, Action> body() {
  using namespace ComposableArchitecture;
  Dependencies::Dependency<Dependencies::DateGeneratorKey> date;

  auto feature = Update<State, Action>([date](State &s, const Action &action,
                                              Store<State, Action> &) {
    std::visit(overloaded{
                   [&](const Ticked &) {
                     // Precess gravity, then advance. Gravity is a function
                     // of the (deterministic) step count, so the sloshing is
                     // reproducible.
                     s.sim.gravity = precessingGravity(s.sim.stepCount);
                     Sim::advance(s.sim, date->now());
                   },
                   [&](const SpawnBurst &) {
                     std::uint32_t rng = s.bursts * 2654435761u + 12345u;
                     for (int i = 0; i < Config::burst && canSpawn(s); ++i) {
                       const float radius = Config::ballRadius * (0.7f + 0.6f * randomFloat(rng));
                       const Physics::Vec3 position{
                           (randomFloat(rng) - 0.5f) * 2.0f * Config::halfX * 0.6f,
                           (randomFloat(rng) - 0.5f) * 2.0f * Config::halfY * 0.6f,
                           (randomFloat(rng) - 0.5f) * 2.0f * Config::halfZ * 0.8f};
                       const float restitution = 0.2f + 0.5f * randomFloat(rng);
                       Physics::addSphere(s.sim.world, position, radius, 1.0f, restitution, 0.4f);
                       ++s.ballCount;
                     }
                     ++s.bursts;
                   },
                   [&](const ResetTapped &) {
                     const double now = date->now();
                     s = initialState();
                     s.sim.lastTime = now;
                   },
                   [&](const TogglePause &) { s.sim.paused = !s.sim.paused; },
               },
               action);
  });

  feature.onMount([date](State &s, Store<State, Action> &) { s.sim.lastTime = date->now(); });
  return feature;
}

} // namespace CorridorFeature
