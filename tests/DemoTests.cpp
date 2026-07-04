// Tests for the demo-specific physics: car suspension holds the chassis up and
// level, the explosion imparts outward velocity, and the fireworks particle
// update is deterministic under parallelism.

import std;
import PhysicsEngine;
import Simulation;
import CarFeature;
import ExplosionFeature;
import FireworksFeature;

using namespace Physics;

namespace {

int failures = 0;
void expect(bool cond, std::string_view msg) {
  if (!cond) {
    ++failures;
    std::println(std::cerr, "FAIL: {}", msg);
  }
}

// The raycast suspension should hold the chassis floating above the ground and
// roughly level after settling (throttle off).
void testCarSuspensionHoldsChassis() {
  CarFeature::State s = CarFeature::initialState();
  StepContext context{.dt = Sim::fixedDt, .gravity = {0, -9.81f, 0}};
  for (int i = 0; i < 180; ++i) {
    CarFeature::applyVehicle(s.sim.world, 0.0f, 0.0f, Sim::fixedDt);
    step(s.sim.world, context);
  }
  const Transform &t = s.sim.world.transforms[CarFeature::Config::chassis];
  const Vec3 up = toMat3(t.orientation).c1;
  expect(t.position.y > 0.4f && t.position.y < 1.5f, "car chassis floats on its suspension");
  expect(up.y > 0.85f, "car chassis stays roughly level (not tipped)");

  // Ride-height stability: over a further stretch the chassis y must barely move.
  // A bobbing/juddering suspension would show a large peak-to-peak swing.
  float minY = t.position.y, maxY = t.position.y;
  for (int i = 0; i < 120; ++i) {
    CarFeature::applyVehicle(s.sim.world, 0.0f, 0.0f, Sim::fixedDt);
    step(s.sim.world, context);
    const float y = s.sim.world.transforms[CarFeature::Config::chassis].position.y;
    minY = std::min(minY, y);
    maxY = std::max(maxY, y);
  }
  expect(maxY - minY < 0.03f, "car ride height is steady (no suspension bob/judder)");
}

// A detonation gives every box an outward velocity from the blast center.
void testExplosionPushesOutward() {
  ExplosionFeature::State s = ExplosionFeature::initialState();
  const Vec3 center{0, 1, 0};
  applyRadialImpulse(s.sim.world, center, 42.0f, 9.0f);
  int outward = 0, dynamic = 0;
  for (Entity i = 0; i < s.sim.world.count(); ++i) {
    if (s.sim.world.bodies[i].type != BodyType::Dynamic) {
      continue;
    }
    ++dynamic;
    const Vec3 fromCenter = s.sim.world.transforms[i].position - center;
    if (dot(s.sim.world.motions[i].linearVelocity, fromCenter) > 0.0f) {
      ++outward;
    }
  }
  expect(dynamic > 0 && outward == dynamic, "explosion pushes every box outward from the blast");
}

// Particle integration is bit-identical whether run on 1 thread or 8.
void testFireworksParallelDeterminism() {
  auto build = [] {
    Fireworks::ParticleSystem particles;
    Fireworks::launchBurst(particles, {0, 8, 0}, 2000, 7);
    return particles;
  };
  Fireworks::ParticleSystem serial = build();
  Fireworks::ParticleSystem par = build();
  for (int i = 0; i < 60; ++i) {
    Fireworks::stepParticles(serial, Sim::fixedDt, {0, -6, 0}, 0.4f, 1);
    Fireworks::stepParticles(par, Sim::fixedDt, {0, -6, 0}, 0.4f, 8);
  }
  expect(serial == par, "fireworks: 8-thread particle update == serial (deterministic)");
  expect(length(serial.positions[0] - Vec3{0, 8, 0}) > 0.5f, "fireworks: particles move over time");
}

} // namespace

int main() {
  testCarSuspensionHoldsChassis();
  testExplosionPushesOutward();
  testFireworksParallelDeterminism();
  if (failures == 0) {
    std::println("All DemoTests passed.");
    return 0;
  }
  std::println(std::cerr, "{} DemoTests failed.", failures);
  return 1;
}
