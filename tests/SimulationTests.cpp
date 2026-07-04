// Tests for the fixed-timestep driver's render interpolation. The key one guards
// the judder/"doubling" fix: after a frame that runs several fixed steps, the
// interpolation baseline `previous` must be exactly ONE step behind `world` (not
// many), or the interpolated pose is wrong and objects appear to double.

import std;
import PhysicsEngine;
import Simulation;

using namespace Physics;

namespace {

int failures = 0;
void expect(bool cond, std::string_view msg) {
  if (!cond) {
    ++failures;
    std::println(std::cerr, "FAIL: {}", msg);
  }
}
bool approx(float a, float b, float eps = 1e-3f) { return std::abs(a - b) < eps; }

// A single frame worth 3.5 fixed steps must run exactly 3 steps and leave
// `previous` one step behind `world` (world = 3 steps, previous = 2 steps).
void testInterpolationBaselineIsExactlyOneStepBehind() {
  Sim::State s;
  s.gravity = {0, -9.81f, 0};
  addSphere(s.world, {0, 10, 0}, 0.5f, 1000.0f, 0.0f, 0.0f);
  s.previous = s.world;
  s.lastTime = 0.0;

  const World initial = s.world; // reference to step manually
  Sim::advance(s, 3.5 * static_cast<double>(Sim::fixedDt));

  StepContext context{.dt = Sim::fixedDt, .gravity = s.gravity};
  World afterThree = initial, afterTwo = initial;
  for (int i = 0; i < 3; ++i) {
    step(afterThree, context);
  }
  for (int i = 0; i < 2; ++i) {
    step(afterTwo, context);
  }

  expect(s.world == afterThree, "advance runs floor(3.5) = 3 fixed steps");
  expect(s.previous == afterTwo, "interpolation baseline is exactly one fixed step behind world");
  expect(s.alpha > 0.45f && s.alpha < 0.55f, "leftover alpha ~= 0.5");
}

// interpolatedPosition/Orientation must hit the endpoints and the midpoint.
void testInterpolationEndpoints() {
  Sim::State s;
  addSphere(s.world, {10, 0, 0}, 0.5f, 1000.0f, 0.0f, 0.0f);
  s.previous = s.world;
  s.world.transforms[0].position = {20, 0, 0};

  s.alpha = 0.0f;
  expect(approx(Sim::interpolatedPosition(s, 0).x, 10.0f), "alpha 0 -> previous position");
  s.alpha = 1.0f;
  expect(approx(Sim::interpolatedPosition(s, 0).x, 20.0f), "alpha 1 -> current position");
  s.alpha = 0.5f;
  expect(approx(Sim::interpolatedPosition(s, 0).x, 15.0f), "alpha 0.5 -> midpoint");
}

// A body that does not move interpolates to a perfectly steady pose (no doubling
// even when many steps run in one frame).
void testStationaryBodyDoesNotJitter() {
  Sim::State s;
  s.gravity = {0, 0, 0};
  addStaticBox(s.world, {0, 0, 0}, {1, 1, 1}, 0.0f, 0.5f);
  s.previous = s.world;
  s.lastTime = 0.0;
  Sim::advance(s, 5.0 * static_cast<double>(Sim::fixedDt));
  const Vec3 p = Sim::interpolatedPosition(s, 0);
  expect(approx(p.x, 0.0f) && approx(p.y, 0.0f) && approx(p.z, 0.0f),
         "a stationary body's interpolated position is rock-steady");
}

} // namespace

int main() {
  testInterpolationBaselineIsExactlyOneStepBehind();
  testInterpolationEndpoints();
  testStationaryBodyDoesNotJitter();
  if (failures == 0) {
    std::println("All SimulationTests passed.");
    return 0;
  }
  std::println(std::cerr, "{} SimulationTests failed.", failures);
  return 1;
}
