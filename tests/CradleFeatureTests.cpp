// CradleFeature tests. The headline one exercises the RealityKit-style
// collision-event -> Composable-Architecture Action loop end-to-end under
// TestStore: a Ticked that produces a contact must emit a CollisionOccurred
// Action, which TestStore's exhaustive `receive` verifies.

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;
import CradleFeature;

using ComposableArchitecture::TestStore;
using Dependencies::DateGenerator;
using Dependencies::DateGeneratorKey;
using Dependencies::DependencyContext;
using Dependencies::DependencyValues;
using Dependencies::withDependencies;
namespace CF = CradleFeature;

namespace {

int failures = 0;
void expect(bool cond, std::string_view msg) {
  if (!cond) {
    ++failures;
    std::println(std::cerr, "FAIL: {}", msg);
  }
}

// Two overlapping balls (gravity off): the first Ticked detects a new contact,
// which must surface as exactly one CollisionOccurred Action.
void testCollisionSurfacesAsAction() {
  double now = 0.0;
  withDependencies(
      [&now](DependencyValues &v) {
        v.context = DependencyContext::test;
        v.set<DateGeneratorKey>(DateGenerator{[&now] { return now; }});
      },
      [&now] {
        CF::State s;
        s.sim.gravity = {0, 0, 0};
        Physics::addSphere(s.sim.world, {-0.4f, 0, 0}, 0.5f, 1000.0f, 0.5f, 0.0f); // overlapping
        Physics::addSphere(s.sim.world, {0.4f, 0, 0}, 0.5f, 1000.0f, 0.5f, 0.0f);
        s.sim.previous = s.sim.world;
        TestStore<CF::State, CF::Action> store(std::move(s), CF::body);

        now = 0.02; // one fixed step
        store.send(CF::Ticked{},
                   [&now](CF::State &randomState) { Sim::advance(randomState.sim, now); });
        // The new contact was re-sent as a CollisionOccurred; receiving it counts it.
        store.receive([](CF::State &randomState) { ++randomState.impactCount; });

        expect(store.state().impactCount == 1, "a new contact surfaces as one CollisionOccurred");
        expect(!store.failed(), "collision event -> Action loop is exhaustively correct");
      });
}

void testPauseStopsClicks() {
  double now = 0.0;
  withDependencies(
      [&now](DependencyValues &v) {
        v.context = DependencyContext::test;
        v.set<DateGeneratorKey>(DateGenerator{[&now] { return now; }});
      },
      [&now] {
        TestStore<CF::State, CF::Action> store(CF::initialState(), CF::body);
        store.send(CF::TogglePause{},
                   [](CF::State &randomState) { randomState.sim.paused = true; });
        now = 2.0;
        store.send(CF::Ticked{},
                   [&now](CF::State &randomState) { Sim::advance(randomState.sim, now); });
        expect(store.state().sim.stepCount == 0, "paused cradle produces no steps (and no clicks)");
        expect(!store.failed(), "pause gates the cradle");
      });
}

// Engine-level: the pulled end ball swings in and sets the rest of the row in
// motion (momentum reaches the far ball).
void testCradleTransfersMotion() {
  Physics::World w = CF::initialState().sim.world;
  Physics::StepContext context{.dt = 1.0f / 60.0f, .gravity = {0, -9.81f, 0}, .subStepCount = 8};
  const Physics::Entity last = w.count() - 1; // final ball
  float maxLastSpeed = 0.0f;
  for (int i = 0; i < 240; ++i) {
    Physics::step(w, context);
    maxLastSpeed = std::max(maxLastSpeed, Physics::length(w.motions[last].linearVelocity));
  }
  expect(maxLastSpeed > 0.2f, "momentum transfers through the row to the far ball");
}

} // namespace

int main() {
  testCollisionSurfacesAsAction();
  testPauseStopsClicks();
  testCradleTransfersMotion();
  if (failures == 0) {
    std::println("All CradleFeature tests passed.");
    return 0;
  }
  std::println(std::cerr, "{} CradleFeature test(s) failed.", failures);
  return 1;
}
