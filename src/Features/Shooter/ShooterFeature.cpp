module ShooterFeature;

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;

namespace ShooterFeature {
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
                       [&](const MoveLocal &m) {
                         const Physics::Vec3 forward = forwardDirection(s.yaw, s.pitch);
                         const Physics::Vec3 right =
                             Physics::normalized(Physics::cross(forward, {0, 1, 0}));
                         s.cameraPosition += forward * m.forward + right * m.right;
                       },
                       [&](const LookDelta &l) {
                         s.yaw += l.dyaw;
                         s.pitch = Physics::clampf(s.pitch + l.dpitch, -1.4f, 1.4f);
                       },
                       [&](const Shoot &) {
                         if (s.bulletCount >= Config::maxBullets) {
                           return;
                         }
                         const Physics::Vec3 forward = forwardDirection(s.yaw, s.pitch);
                         const Physics::Vec3 origin = s.cameraPosition + forward * 1.2f; // muzzle
                         Physics::addSphere(s.sim.world, origin, Config::bulletRadius,
                                            Config::bulletDensity, 0.2f, 0.4f,
                                            forward * Config::bulletSpeed);
                         ++s.bulletCount;
                       },
                       [&](const ResetTapped &) {
                         const double now = date->now();
                         const Physics::Vec3 camera = s.cameraPosition;
                         const float yaw = s.yaw, pitch = s.pitch;
                         s = initialState();
                         s.cameraPosition = camera; // keep the player where they are
                         s.yaw = yaw;
                         s.pitch = pitch;
                         s.sim.lastTime = now;
                       },
                       [&](const TogglePause &) { s.sim.paused = !s.sim.paused; },
                   },
                   action);
      });

  feature.onMount([date](State &s, Store<State, Action> &) { s.sim.lastTime = date->now(); });
  return feature;
}

} // namespace ShooterFeature
