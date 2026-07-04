// BallPitFeature tests (TestStore + controlled clock). Same replay-the-pure-
// function approach as the Sandbox tests, plus the spawn cap invariant.

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;
import BallPitFeature;

using ComposableArchitecture::TestStore;
using Dependencies::DateGenerator;
using Dependencies::DateGeneratorKey;
using Dependencies::DependencyContext;
using Dependencies::DependencyValues;
using Dependencies::withDependencies;
namespace BP = BallPitFeature;

namespace {

int failures = 0;
void expect(bool cond, std::string_view msg) {
  if (!cond) {
    ++failures;
    std::println(std::cerr, "FAIL: {}", msg);
  }
}

void testSpawnSphereAddsBody() {
  withDependencies(
      [](DependencyValues &v) { v.context = DependencyContext::test; },
      [] {
        TestStore<BP::State, BP::Action> store(BP::initialState(), BP::body);
        const auto before = store.state().sim.world.count();
        const BP::SpawnSphere spawn{.position = {0, 5, 0}, .radius = 0.3f, .restitution = 0.6f};
        store.send(spawn, [spawn](BP::State &s) {
          Physics::addSphere(s.sim.world, spawn.position, spawn.radius, spawn.density,
                             spawn.restitution, 0.4f, spawn.velocity);
          ++s.spawnCount;
        });
        expect(store.state().sim.world.count() == before + 1, "SpawnSphere adds one body");
        expect(!store.failed(), "SpawnSphere mutates the world deterministically");
      });
}

void testTickAdvancesSimulation() {
  double now = 0.0;
  withDependencies(
      [&now](DependencyValues &v) {
        v.context = DependencyContext::test;
        v.set<DateGeneratorKey>(DateGenerator{[&now] { return now; }});
      },
      [&now] {
        TestStore<BP::State, BP::Action> store(BP::initialState(), BP::body);
        // Drop a sphere, then advance ~6 steps of gravity.
        const BP::SpawnSphere spawn{.position = {0, 5, 0}, .radius = 0.4f};
        store.send(spawn, [spawn](BP::State &s) {
          Physics::addSphere(s.sim.world, spawn.position, spawn.radius, spawn.density,
                             spawn.restitution, 0.4f, spawn.velocity);
          ++s.spawnCount;
        });
        now = 0.1;
        store.send(BP::Ticked{}, [&now](BP::State &s) { Sim::advance(s.sim, now); });
        expect(store.state().sim.stepCount >= 5 && store.state().sim.stepCount <= 6,
               "0.1s accumulates ~6 fixed steps");
        expect(store.state().sim.world.transforms.back().position.y < 5.0f,
               "the dropped sphere has fallen under gravity");
        expect(!store.failed(), "Ticked advances the ball pit deterministically");
      });
}

void testSpawnCapIsEnforced() {
  withDependencies([](DependencyValues &v) { v.context = DependencyContext::test; },
                   [] {
                     BP::State initial = BP::initialState();
                     // Pre-fill to the cap so the next spawn must be a no-op.
                     for (int i = 0; i < BP::Config::maxBodies; ++i) {
                       Physics::addSphere(initial.sim.world, {0, 100.0f + static_cast<float>(i), 0},
                                          0.2f, 1.0f, 0.5f, 0.4f);
                     }
                     TestStore<BP::State, BP::Action> store(std::move(initial), BP::body);
                     const auto before = store.state().sim.world.count();
                     // {} = expect no state change: the spawn is rejected at the cap.
                     store.send(BP::SpawnSphere{.position = {0, 5, 0}}, {});
                     expect(store.state().sim.world.count() == before,
                            "spawning at the cap is a no-op");
                     expect(!store.failed(), "spawn cap invariant holds");
                   });
}

} // namespace

int main() {
  testSpawnSphereAddsBody();
  testTickAdvancesSimulation();
  testSpawnCapIsEnforced();
  if (failures == 0) {
    std::println("All BallPitFeature tests passed.");
    return 0;
  }
  std::println(std::cerr, "{} BallPitFeature test(s) failed.", failures);
  return 1;
}
