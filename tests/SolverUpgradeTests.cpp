// Tests for the solver upgrades: warm-starting (stable stacks), speculative
// contacts (fast bodies don't tunnel, near bodies don't stick), and
// sub-stepping (stable at any sub-step count).

import std;
import PhysicsEngine;

using namespace Physics;

namespace {

int failures = 0;
void expect(bool cond, std::string_view msg) {
  if (!cond) {
    ++failures;
    std::println(std::cerr, "FAIL: {}", msg);
  }
}

// A vertical stack of boxes must settle and stay put (no sink, no jitter/blow-up)
// — the payoff of warm-starting + sub-stepping.
void testBoxStackRests() {
  World w;
  addStaticBox(w, {0, -0.5f, 0}, {10, 0.5f, 10}, 0.2f, 0.6f); // floor top at y=0
  const int n = 5;
  for (int i = 0; i < n; ++i) {
    addBox(w, {0, 0.5f + static_cast<float>(i) * 1.0f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f, 0.0f, 0.6f);
  }
  StepContext context{.dt = 1.0f / 60.0f, .gravity = {0, -9.81f, 0}};
  for (int s = 0; s < 400; ++s) {
    step(w, context);
  }
  // Each box should stay near its resting slot (a little settling is fine).
  bool ok = true;
  float maxSpeed = 0.0f;
  for (int i = 0; i < n; ++i) {
    const float y = w.transforms[1 + i].position.y;
    const float expected = 0.5f + static_cast<float>(i) * 1.0f;
    if (std::abs(y - expected) > 0.15f) {
      ok = false;
    }
    maxSpeed = std::max(maxSpeed, length(w.motions[1 + i].linearVelocity));
  }
  expect(ok, "box stack settles near its resting heights (no sink/explode)");
  expect(maxSpeed < 0.2f, "box stack comes to rest (low residual velocity)");
}

// A very fast sphere must not tunnel through a thin wall (speculative contacts +
// sub-stepping brake it at the surface).
void testHighSpeedNoTunnel() {
  World w;
  addStaticBox(w, {0, 0, 0}, {0.5f, 5, 5}, 0.0f, 0.0f);             // wall x in [-0.5, 0.5]
  addSphere(w, {-6, 0, 0}, 0.5f, 1000.0f, 0.0f, 0.0f, {120, 0, 0}); // 120 m/s toward +x
  StepContext context{.dt = 1.0f / 60.0f, .gravity = {0, 0, 0}};
  bool crossed = false;
  for (int i = 0; i < 200; ++i) {
    step(w, context);
    if (w.transforms[1].position.x > 0.0f) {
      crossed = true;
    }
  }
  expect(!crossed, "fast sphere (120 m/s) never crosses the wall (no tunneling)");
  expect(w.transforms[1].position.x < -0.4f, "fast sphere is stopped at the wall face");
}

// Two spheres within the speculative distance but not touching, at rest, must
// NOT attract each other (speculative contacts push, never pull).
void testSpeculativeDoesNotGlue() {
  World w;
  addSphere(w, {-0.505f, 0, 0}, 0.5f, 1000.0f, 0.0f, 0.0f); // 0.01 gap (< speculativeDistance)
  addSphere(w, {0.505f, 0, 0}, 0.5f, 1000.0f, 0.0f, 0.0f);
  StepContext context{.dt = 1.0f / 60.0f, .gravity = {0, 0, 0}};
  for (int i = 0; i < 60; ++i) {
    step(w, context);
  }
  const float gap = w.transforms[1].position.x - w.transforms[0].position.x;
  expect(gap > 1.0f, "near-but-separate spheres do not get glued/pulled together");
}

// Stable at any sub-step count (no NaN, positions bounded).
void testSubStepStability() {
  for (int subs : {1, 2, 8}) {
    World w;
    addStaticBox(w, {0, -0.5f, 0}, {10, 0.5f, 10}, 0.2f, 0.6f);
    for (int i = 0; i < 6; ++i) {
      addBox(w, {0.02f * i, 0.5f + i * 1.0f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f, 0.1f, 0.6f);
    }
    StepContext context{.dt = 1.0f / 60.0f, .gravity = {0, -9.81f, 0}, .subStepCount = subs};
    for (int s = 0; s < 300; ++s) {
      step(w, context);
    }
    bool bounded = true;
    for (Entity i = 0; i < w.count(); ++i) {
      const Vec3 p = w.transforms[i].position;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || std::abs(p.y) > 50.0f) {
        bounded = false;
      }
    }
    expect(bounded, std::format("stack stays bounded at subStepCount={}", subs));
  }
}

// Parallel narrowphase must be bit-identical to serial: same inputs + same
// steps produce the same world regardless of thread count.
void testThreadedMatchesSerial() {
  auto build = [] {
    World w;
    addStaticBox(w, {0, -0.5f, 0}, {12, 0.5f, 12}, 0.3f, 0.5f);
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> p(-4.0f, 4.0f), y(0.5f, 6.0f), r(0.25f, 0.6f);
    for (int i = 0; i < 120; ++i) {
      addSphere(w, {p(rng), y(rng), p(rng)}, r(rng), 1.0f, 0.4f, 0.5f);
    }
    return w;
  };
  World serial = build();
  World threaded = build();
  StepContext c1{.dt = 1.0f / 60.0f, .gravity = {0, -9.81f, 0}, .threadCount = 1};
  StepContext c8 = c1;
  c8.threadCount = 8;
  for (int i = 0; i < 120; ++i) {
    step(serial, c1);
    step(threaded, c8);
  }
  expect(serial == threaded, "parallel narrowphase (8 threads) == serial (bit-identical world)");
}

// An elastic (e=1) sphere bouncing on an elastic floor UNDER GRAVITY must not
// gain energy over many bounces. (Regression: restitution once reflected a
// gravity-inflated approach velocity, adding g*h of energy per bounce.)
void testElasticBounceConservesEnergyUnderGravity() {
  World w;
  addStaticBox(w, {0, -0.5f, 0}, {10, 0.5f, 10}, 1.0f, 0.0f); // e=1 floor, top at y=0
  addSphere(w, {0, 3.0f, 0}, 0.5f, 1000.0f, 1.0f, 0.0f);      // e=1, frictionless
  const Vec3 gravity{0, -9.81f, 0};
  StepContext context{.dt = 1.0f / 60.0f, .gravity = gravity, .restitutionThreshold = 0.1f};
  const float energy0 = linearKineticEnergy(w) + potentialEnergy(w, gravity);
  float maxEnergy = energy0;
  for (int i = 0; i < 900; ++i) { // ~15s, many bounces
    step(w, context);
    maxEnergy = std::max(maxEnergy, linearKineticEnergy(w) + potentialEnergy(w, gravity));
  }
  expect(maxEnergy <= energy0 * 1.05f, "elastic bounce under gravity never gains energy");
}

} // namespace

int main() {
  testBoxStackRests();
  testHighSpeedNoTunnel();
  testSpeculativeDoesNotGlue();
  testSubStepStability();
  testThreadedMatchesSerial();
  testElasticBounceConservesEnergyUnderGravity();
  if (failures == 0) {
    std::println("All SolverUpgradeTests passed.");
    return 0;
  }
  std::println(std::cerr, "{} SolverUpgradeTests failed.", failures);
  return 1;
}
