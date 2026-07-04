// Property / invariant tests for the simulation — the highest-value tests for a
// physics engine. They assert physical laws over full runs rather than single
// function outputs:
//   * free-fall matches the closed-form semi-implicit Euler trajectory (golden)
//   * momentum is conserved and KE is preserved in an elastic (e=1) collision
//   * KE does not increase in an inelastic (e<1) collision
//   * a body resting on a static floor neither sinks nor drifts
//   * a fast body does not tunnel through a thick static wall
//   * the same inputs produce a bit-exact-identical world (determinism)

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
bool approx(float a, float b, float eps) { return std::abs(a - b) < eps; }

// --- Golden trajectory: free fall matches semi-implicit Euler exactly --------
// With v_{k}=v_{k-1}+g·dt and x_k=x_{k-1}+v_k·dt, after k steps
//   y_k = y0 - |g|·dt²·(k(k+1)/2).
void testFreeFallTrajectory() {
  World w;
  addSphere(w, {0, 100, 0}, 0.5f, 1000.0f, 0.0f, 0.0f);
  const float g = 10.0f, dt = 1.0f / 60.0f;
  // subStepCount=1 => one integration per step, so the single-step closed form
  // applies exactly (the default 4 sub-steps integrate more finely).
  StepContext context{.dt = dt, .gravity = {0, -g, 0}, .subStepCount = 1};
  for (int k = 1; k <= 120; ++k) {
    step(w, context);
    const float expected = 100.0f - g * dt * dt * (static_cast<float>(k) * (k + 1) / 2.0f);
    if (k == 120) {
      expect(approx(w.transforms[0].position.y, expected, 1e-2f),
             "free-fall y matches semi-implicit Euler closed form");
    }
  }
}

// --- Elastic head-on collision: momentum + KE conserved, velocities swap -----
void testElasticCollisionConservesMomentumAndEnergy() {
  World w;
  addSphere(w, {-0.5f, 0, 0}, 0.5f, 1000.0f, 1.0f, 0.0f, {2, 0, 0});
  addSphere(w, {0.5f, 0, 0}, 0.5f, 1000.0f, 1.0f, 0.0f, {-2, 0, 0});
  StepContext context{.dt = 1.0f / 60.0f, .gravity = {0, 0, 0}, .restitutionThreshold = 0.0f};

  const Vec3 p0 = linearMomentum(w);
  const float ke0 = linearKineticEnergy(w);
  for (int i = 0; i < 10; ++i) {
    step(w, context);
  }
  const Vec3 p1 = linearMomentum(w);
  const float ke1 = linearKineticEnergy(w);

  expect(approx(p1.x, p0.x, 1e-2f) && approx(p0.x, 0.0f, 1e-3f),
         "elastic: momentum conserved (~0)");
  expect(approx(ke1, ke0, 1e-1f), "elastic: kinetic energy conserved");
  // Equal masses swap velocities: both now moving apart at ~2.
  expect(w.motions[0].linearVelocity.x < -1.0f && w.motions[1].linearVelocity.x > 1.0f,
         "elastic equal masses swap velocities");
}

// --- Inelastic collision: kinetic energy must not increase -------------------
void testInelasticCollisionLosesEnergy() {
  World w;
  addSphere(w, {-0.5f, 0, 0}, 0.5f, 1000.0f, 0.3f, 0.0f, {2, 0, 0});
  addSphere(w, {0.5f, 0, 0}, 0.5f, 1000.0f, 0.3f, 0.0f, {-2, 0, 0});
  StepContext context{.dt = 1.0f / 60.0f, .gravity = {0, 0, 0}, .restitutionThreshold = 0.0f};

  const float ke0 = linearKineticEnergy(w);
  float maxKE = ke0;
  for (int i = 0; i < 30; ++i) {
    step(w, context);
    maxKE = std::max(maxKE, linearKineticEnergy(w));
  }
  expect(maxKE <= ke0 + 1e-2f, "inelastic: kinetic energy never increases");
  expect(linearKineticEnergy(w) < ke0, "inelastic: kinetic energy decreases after impact");
}

// --- Resting stability: a sphere on a static floor neither sinks nor drifts ---
void testRestingStability() {
  World w;
  addStaticBox(w, {0, -0.5f, 0}, {10, 0.5f, 10}, 0.0f, 0.5f); // floor top at y=0
  addSphere(w, {0, 0.6f, 0}, 0.5f, 1000.0f, 0.0f, 0.5f);      // just above the floor
  StepContext context{.dt = 1.0f / 60.0f, .gravity = {0, -9.81f, 0}};

  for (int i = 0; i < 600; ++i) { // ~10s
    step(w, context);
  }
  const Vec3 pos = w.transforms[1].position;
  const Vec3 vel = w.motions[1].linearVelocity;
  expect(pos.y > 0.45f && pos.y < 0.55f, "resting: sphere settles at floor top (~y=0.5)");
  expect(std::abs(vel.y) < 0.05f, "resting: sphere is at rest (no residual velocity)");
  expect(std::abs(pos.x) < 0.05f && std::abs(pos.z) < 0.05f, "resting: no lateral drift");
}

// --- No tunneling: a fast sphere is stopped by a thick static wall -----------
void testNoTunneling() {
  World w;
  addStaticBox(w, {0, 0, 0}, {0.5f, 5, 5}, 0.0f, 0.0f);            // wall spanning x in [-0.5, 0.5]
  addSphere(w, {-5, 0, 0}, 0.5f, 1000.0f, 0.0f, 0.0f, {30, 0, 0}); // fast, heading +x
  StepContext context{.dt = 1.0f / 60.0f, .gravity = {0, 0, 0}};

  bool crossed = false;
  for (int i = 0; i < 120; ++i) {
    step(w, context);
    if (w.transforms[1].position.x > 0.0f) {
      crossed = true; // sphere center passed the wall center => tunneled
    }
  }
  expect(!crossed, "no tunneling: fast sphere never passes through the wall");
  expect(w.transforms[1].position.x < -0.9f, "no tunneling: sphere stopped at the wall face");
}

// --- Determinism: identical inputs => bit-exact identical world --------------
void testDeterminism() {
  auto build = [] {
    World w;
    addStaticBox(w, {0, -0.5f, 0}, {10, 0.5f, 10}, 0.2f, 0.5f);
    for (int i = 0; i < 8; ++i) {
      const float x = -1.5f + 0.4f * static_cast<float>(i);
      addSphere(w, {x, 2.0f + 0.3f * static_cast<float>(i), 0.0f}, 0.3f, 1000.0f, 0.4f, 0.5f);
    }
    return w;
  };
  World a = build();
  World b = build();
  StepContext context{.dt = 1.0f / 60.0f, .gravity = {0, -9.81f, 0}};
  for (int i = 0; i < 300; ++i) {
    step(a, context);
    step(b, context);
  }
  expect(a == b, "determinism: same inputs produce a bit-exact identical world");
}

} // namespace

int main() {
  testFreeFallTrajectory();
  testElasticCollisionConservesMomentumAndEnergy();
  testInelasticCollisionLosesEnergy();
  testRestingStability();
  testNoTunneling();
  testDeterminism();
  if (failures == 0) {
    std::println("All PhysicsEngineTests passed.");
    return 0;
  }
  std::println(std::cerr, "{} PhysicsEngineTests failed.", failures);
  return 1;
}
