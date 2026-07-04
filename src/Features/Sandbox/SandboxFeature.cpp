module SandboxFeature; // implementation unit for module SandboxFeature

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;

namespace SandboxFeature {
namespace {
template <class... Ts> struct overloaded : Ts... {
  using Ts::operator()...;
};
} // namespace

ComposableArchitecture::Feature<State, Action> body() {
  using namespace ComposableArchitecture;
  Dependencies::Dependency<Dependencies::DateGeneratorKey> date;

  auto feature = Update<State, Action>([date](State &s, const Action &action,
                                              Store<State, Action> &) {
    std::visit(overloaded{
                   [&](const Ticked &) { Sim::advance(s.sim, date->now()); },
                   [&](const SpawnBox &spawn) {
                     Physics::addBox(s.sim.world, spawn.position, spawn.halfExtents, spawn.density,
                                     0.2f, 0.5f, spawn.orientation, spawn.velocity);
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

  // Start the clock at mount, mirroring the current wall time (TCA 2.0 lifecycle).
  feature.onMount([date](State &s, Store<State, Action> &) { s.sim.lastTime = date->now(); });
  return feature;
}

} // namespace SandboxFeature
