module FireworksFeature;

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;

namespace Fireworks {
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
    std::visit(
        overloaded{
            [&](const Ticked &) {
              // Fixed-timestep accumulator (reusing Sim::State's clock
              // fields), stepping the particle system each fixed step.
              const double now = date->now();
              double frame = now - s.sim.lastTime;
              s.sim.lastTime = now;
              if (frame < 0.0) {
                frame = 0.0;
              }
              if (frame > Sim::maxFrame) {
                frame = Sim::maxFrame;
              }
              s.sim.accumulator += frame;
              while (s.sim.accumulator >= Sim::fixedDt) {
                stepParticles(s.particles, Sim::fixedDt, s.sim.gravity, 0.4f, s.sim.threadCount);
                s.sim.accumulator -= Sim::fixedDt;
                ++s.sim.stepCount;
              }
            },
            [&](const Launch &l) { launchBurst(s.particles, l.center, 400, s.launchCount++); },
            [&](const ResetTapped &) {
              const double now = date->now();
              s = initialState();
              s.sim.lastTime = now;
            },
        },
        action);
  });

  feature.onMount([date](State &s, Store<State, Action> &) { s.sim.lastTime = date->now(); });
  return feature;
}

} // namespace Fireworks
