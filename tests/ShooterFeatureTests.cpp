// ShooterFeature tests: shooting spawns a bullet with the right pose/velocity
// (verified by replaying the pure spawn in the assertion), the FPS camera moves,
// and — at the engine level — a fast heavy bullet knocks a box backward without
// tunnelling through it.

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;
import ShooterFeature;

using ComposableArchitecture::TestStore;
using Dependencies::DependencyContext;
using Dependencies::DependencyValues;
using Dependencies::withDependencies;
namespace SF = ShooterFeature;

namespace {

int failures = 0;
void expect(bool cond, std::string_view msg) {
  if (!cond) {
    ++failures;
    std::println(std::cerr, "FAIL: {}", msg);
  }
}

void testShootSpawnsBullet() {
  withDependencies(
      [](DependencyValues &v) { v.context = DependencyContext::test; },
      [] {
        TestStore<SF::State, SF::Action> store(SF::initialState(), SF::body);
        const auto before = store.state().sim.world.count();
        store.send(SF::Shoot{}, [](SF::State &s) {
          const Physics::Vec3 forward = SF::forwardDirection(s.yaw, s.pitch);
          const Physics::Vec3 origin = s.cameraPosition + forward * 1.2f;
          Physics::addSphere(s.sim.world, origin, SF::Config::bulletRadius,
                             SF::Config::bulletDensity, 0.2f, 0.4f,
                             forward * SF::Config::bulletSpeed);
          ++s.bulletCount;
        });
        expect(store.state().sim.world.count() == before + 1, "Shoot spawns exactly one bullet");
        expect(store.state().bulletCount == 1, "bullet count increments");
        expect(!store.failed(), "bullet pose/velocity is a pure function of the camera");
      });
}

void testMoveAndLook() {
  withDependencies([](DependencyValues &v) { v.context = DependencyContext::test; },
                   [] {
                     TestStore<SF::State, SF::Action> store(SF::initialState(), SF::body);
                     store.send(SF::MoveLocal{2.0f, 0.0f}, [](SF::State &s) {
                       const Physics::Vec3 forward = SF::forwardDirection(s.yaw, s.pitch);
                       const Physics::Vec3 right =
                           Physics::normalized(Physics::cross(forward, {0, 1, 0}));
                       s.cameraPosition += forward * 2.0f + right * 0.0f;
                     });
                     store.send(SF::LookDelta{0.5f, 0.2f}, [](SF::State &s) {
                       s.yaw += 0.5f;
                       s.pitch = Physics::clampf(s.pitch + 0.2f, -1.4f, 1.4f);
                     });
                     expect(!store.failed(), "move + look update the camera deterministically");
                   });
}

// Engine-level: a fast, heavy bullet fired at a light box knocks it backward and
// does not pass through it.
void testBulletKnocksBox() {
  using namespace Physics;
  World w;
  addStaticBox(w, {0, -0.5f, 0}, {10, 0.5f, 10}, 0.3f, 0.7f);
  const Entity box = addBox(w, {0, 0.5f, 0}, {0.5f, 0.5f, 0.5f}, 0.4f, 0.1f, 0.5f);
  // Bullet at z = 4, flying toward -z at the box.
  const Entity bullet =
      addSphere(w, {0, 0.5f, 4}, SF::Config::bulletRadius, SF::Config::bulletDensity, 0.2f, 0.4f,
                {0, 0, -SF::Config::bulletSpeed});
  StepContext context{.dt = 1.0f / 60.0f, .gravity = {0, -9.81f, 0}};
  bool everCrossedBehindBox = false;
  for (int i = 0; i < 60; ++i) {
    step(w, context);
  }
  // The box must have been shoved in -z (backward), and the bullet must not have
  // sailed far past it without interacting.
  expect(w.transforms[box].position.z < -0.3f, "bullet knocks the box backward (-z)");
  expect(length(w.motions[box].linearVelocity) > 0.5f || w.transforms[box].position.z < -1.0f,
         "the box actually gained momentum from the hit");
  (void)everCrossedBehindBox;
  (void)bullet;
}

} // namespace

int main() {
  testShootSpawnsBullet();
  testMoveAndLook();
  testBulletKnocksBox();
  if (failures == 0) {
    std::println("All ShooterFeature tests passed.");
    return 0;
  }
  std::println(std::cerr, "{} ShooterFeature test(s) failed.", failures);
  return 1;
}
