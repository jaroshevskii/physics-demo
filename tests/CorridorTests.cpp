// Corridor stress test: hundreds of balls under a precessing gravity that
// sloshes them against every wall must stay CONTAINED in the tube — none may
// tunnel out — and the sim must stay finite. Validates the many-body scene the
// Metal demo renders.

import std;
import PhysicsEngine;
import Simulation;
import CorridorFeature;

using namespace Physics;
namespace CF = CorridorFeature;

namespace {

int failures = 0;
void expect(bool cond, std::string_view msg) {
  if (!cond) {
    ++failures;
    std::println(std::cerr, "FAIL: {}", msg);
  }
}

void testBallsStayContained() {
  CF::State s = CF::initialState();
  const std::size_t wallCount = s.sim.world.count(); // the 6 static walls
  // Drop a grid of balls into the tube.
  std::uint32_t rng = 99;
  auto rand01 = [&rng] {
    rng = rng * 1664525u + 1013904223u;
    return static_cast<float>(rng >> 8) * (1.0f / 16777216.0f);
  };
  for (int i = 0; i < 400; ++i) {
    addSphere(s.sim.world,
              {(rand01() - 0.5f) * 5.0f, (rand01() - 0.5f) * 5.0f, (rand01() - 0.5f) * 18.0f},
              CF::Config::ballRadius, 1.0f, 0.4f, 0.4f);
  }

  StepContext context{.dt = Sim::fixedDt, .subStepCount = 6, .threadCount = 8};
  for (std::uint64_t stepIndex = 0; stepIndex < 600; ++stepIndex) {
    context.gravity = CF::precessingGravity(stepIndex);
    step(s.sim.world, context);
  }

  const float mx = CF::Config::halfX + CF::Config::ballRadius + 0.2f;
  const float my = CF::Config::halfY + CF::Config::ballRadius + 0.2f;
  const float mz = CF::Config::halfZ + CF::Config::ballRadius + 0.2f;
  bool contained = true, finite = true;
  for (Entity i = wallCount; i < s.sim.world.count(); ++i) {
    const Vec3 p = s.sim.world.transforms[i].position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      finite = false;
    }
    if (std::abs(p.x) > mx || std::abs(p.y) > my || std::abs(p.z) > mz) {
      contained = false;
    }
  }
  expect(finite, "corridor: simulation stays finite (no NaN blow-up)");
  expect(contained, "corridor: no ball tunnels out through a wall despite precessing gravity");
}

void testSpawnCapEnforced() {
  CF::State s = CF::initialState();
  for (int i = 0; i < CF::Config::maxBalls + 100; ++i) {
    if (CF::canSpawn(s)) {
      addSphere(s.sim.world, {0, 0, 0}, CF::Config::ballRadius, 1.0f, 0.3f, 0.4f);
    }
  }
  expect(static_cast<int>(s.sim.world.count()) <= CF::Config::maxBalls + 6,
         "corridor: spawn cap keeps the body count bounded");
}

} // namespace

int main() {
  testBallsStayContained();
  testSpawnCapEnforced();
  if (failures == 0) {
    std::println("All CorridorTests passed.");
    return 0;
  }
  std::println(std::cerr, "{} CorridorTests failed.", failures);
  return 1;
}
