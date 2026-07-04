// SandboxFeature tests, driven by TestStore with a controlled `\.date` clock.
// Because the reducer's effects are pure functions (Sim::advance, addBox,
// initialState), each assertion simply replays that same function on the
// expected state — so the tests verify the reducer wires the engine up
// correctly (right dt, right step count, pause gating, reset) without baking in
// any physics constants.

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;
import SandboxFeature;

using ComposableArchitecture::TestStore;
using Dependencies::DateGenerator;
using Dependencies::DateGeneratorKey;
using Dependencies::DependencyContext;
using Dependencies::DependencyValues;
using Dependencies::withDependencies;
namespace SF = SandboxFeature;

namespace {

int failures = 0;
void expect(bool cond, std::string_view msg) {
  if (!cond) {
    ++failures;
    std::println(std::cerr, "FAIL: {}", msg);
  }
}

void testTickAdvancesSimulation() {
  double now = 0.0;
  withDependencies(
      [&now](DependencyValues &v) {
        v.context = DependencyContext::test;
        v.set<DateGeneratorKey>(DateGenerator{[&now] { return now; }});
      },
      [&now] {
        TestStore<SF::State, SF::Action> store(SF::initialState(), SF::body);
        now = 0.05; // ~3 fixed steps of 1/60 (2–3, depending on float rounding of 1/60)
        store.send(SF::Ticked{}, [&now](SF::State &s) { Sim::advance(s.sim, now); });
        expect(!store.failed(), "Ticked advances the simulation by the accumulated time");
        expect(store.state().sim.stepCount >= 2 && store.state().sim.stepCount <= 3,
               "0.05s accumulates ~3 fixed steps");
      });
}

void testPauseGatesStepping() {
  double now = 0.0;
  withDependencies(
      [&now](DependencyValues &v) {
        v.context = DependencyContext::test;
        v.set<DateGeneratorKey>(DateGenerator{[&now] { return now; }});
      },
      [&now] {
        TestStore<SF::State, SF::Action> store(SF::initialState(), SF::body);
        store.send(SF::TogglePause{}, [](SF::State &s) { s.sim.paused = true; });
        now = 1.0;
        // Paused: advance just swallows elapsed time; no steps run.
        store.send(SF::Ticked{}, [&now](SF::State &s) { Sim::advance(s.sim, now); });
        expect(store.state().sim.stepCount == 0, "paused simulation takes no steps");
        expect(!store.failed(), "pause gates stepping");
      });
}

void testSpawnBoxAddsBody() {
  withDependencies(
      [](DependencyValues &v) { v.context = DependencyContext::test; },
      [] {
        TestStore<SF::State, SF::Action> store(SF::initialState(), SF::body);
        const auto before = store.state().sim.world.count();
        const SF::SpawnBox spawn{.position = {0, 5, 0}, .halfExtents = {0.4f, 0.4f, 0.4f}};
        store.send(spawn, [spawn](SF::State &s) {
          Physics::addBox(s.sim.world, spawn.position, spawn.halfExtents, spawn.density, 0.2f, 0.5f,
                          spawn.orientation, spawn.velocity);
          ++s.spawnCount;
        });
        expect(store.state().sim.world.count() == before + 1, "SpawnBox adds one body");
        expect(!store.failed(), "SpawnBox mutates the world deterministically");
      });
}

void testResetRestoresInitialScene() {
  double now = 0.0;
  withDependencies(
      [&now](DependencyValues &v) {
        v.context = DependencyContext::test;
        v.set<DateGeneratorKey>(DateGenerator{[&now] { return now; }});
      },
      [&now] {
        TestStore<SF::State, SF::Action> store(SF::initialState(), SF::body);
        now = 0.5;
        store.send(SF::Ticked{}, [&now](SF::State &s) { Sim::advance(s.sim, now); });
        now = 0.6;
        store.send(SF::ResetTapped{}, [&now](SF::State &s) {
          s = SF::initialState();
          s.sim.lastTime = now;
        });
        expect(!store.failed(), "Reset restores the initial scene");
      });
}

} // namespace

int main() {
  testTickAdvancesSimulation();
  testPauseGatesStepping();
  testSpawnBoxAddsBody();
  testResetRestoresInitialScene();
  if (failures == 0) {
    std::println("All SandboxFeature tests passed.");
    return 0;
  }
  std::println(std::cerr, "{} SandboxFeature test(s) failed.", failures);
  return 1;
}
