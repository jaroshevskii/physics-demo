module CradleFeature;

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;

namespace CradleFeature {
namespace {
template <class... Ts> struct overloaded : Ts... {
  using Ts::operator()...;
};
} // namespace

ComposableArchitecture::Feature<State, Action> body() {
  using namespace ComposableArchitecture;
  Dependencies::Dependency<Dependencies::DateGeneratorKey> date;

  auto feature =
      Update<State, Action>([date](State &s, const Action &action, Store<State, Action> &store) {
        std::visit(overloaded{
                       [&](const Ticked &) {
                         // Advance, collecting collision events, and re-send each new
                         // contact as a CollisionOccurred Action (RealityKit-style).
                         std::vector<Physics::CollisionEvent> events;
                         Sim::advance(s.sim, date->now(), &events);
                         for (const Physics::CollisionEvent &e : events) {
                           if (e.phase == Physics::CollisionEvent::Phase::Began) {
                             store.send(CollisionOccurred{e});
                           }
                         }
                       },
                       [&](const CollisionOccurred &) { ++s.impactCount; },
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

} // namespace CradleFeature
