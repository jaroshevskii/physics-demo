module CarFeature;

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;

namespace CarFeature {
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
                       [&](const Ticked &) {
                         // Apply the vehicle forces ONCE PER FIXED STEP (via the
                         // beforeStep hook), not once per frame — otherwise the
                         // suspension is applied an unpredictable number of times
                         // relative to the steps that integrate it, and the car
                         // bobs/judders. Capture the controls by value.
                         const float throttle = s.throttle, steer = s.steer;
                         Sim::advance(s.sim, date->now(), nullptr,
                                      [throttle, steer](Physics::World &world) {
                                        applyVehicle(world, throttle, steer, Sim::fixedDt);
                                      });
                       },
                       [&](const SetControls &c) {
                         s.throttle = c.throttle;
                         s.steer = c.steer;
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

} // namespace CarFeature
