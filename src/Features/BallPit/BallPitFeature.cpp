module BallPitFeature; // implementation unit for module BallPitFeature

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;

namespace BallPitFeature {
namespace {
template <class... Ts> struct overloaded : Ts... {
  using Ts::operator()...;
};
} // namespace

ComposableArchitecture::Feature<State, Action> body() {
  using namespace ComposableArchitecture;
  Dependencies::Dependency<Dependencies::DateGeneratorKey> date;

  auto feature =
      Update<State, Action>([date](State &s, const Action &action, Store<State, Action> &) {
        std::visit(overloaded{
                       [&](const Ticked &) { Sim::advance(s.sim, date->now()); },
                       [&](const SpawnSphere &spawn) {
                         if (!canSpawn(s)) {
                           return;
                         }
                         Physics::addSphere(s.sim.world, spawn.position, spawn.radius,
                                            spawn.density, spawn.restitution, 0.4f, spawn.velocity);
                         ++s.spawnCount;
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

} // namespace BallPitFeature
