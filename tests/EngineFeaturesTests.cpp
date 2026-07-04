// Tests for the additive engine features: raycasting, force fields (explosion /
// drag), and the distance/spring constraint.

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
bool approx(float a, float b, float eps = 1e-2f) { return std::abs(a - b) < eps; }
bool approxVec(Vec3 a, Vec3 b, float eps = 1e-2f) {
  return approx(a.x, b.x, eps) && approx(a.y, b.y, eps) && approx(a.z, b.z, eps);
}

// --- Raycast -----------------------------------------------------------------
void testRaycastSphere() {
  World w;
  addSphere(w, {0, 0, 0}, 1.0f, 1000.0f, 0.0f, 0.5f);
  auto hit = raycastNearest(w, {0, 5, 0}, {0, -1, 0}, 10.0f);
  expect(hit.has_value(), "raycast hits the sphere");
  if (hit) {
    expect(approx(hit->distance, 4.0f), "raycast distance is 4 (5 - radius 1)");
    expect(approxVec(hit->normal, {0, 1, 0}), "raycast normal points up");
    expect(approxVec(hit->position, {0, 1, 0}), "raycast hit point on the sphere top");
  }
  expect(!raycastNearest(w, {5, 5, 0}, {0, -1, 0}, 10.0f).has_value(), "raycast misses => nullopt");
}

void testRaycastRotatedBox() {
  World w;
  // A box rotated 45° about Z; a downward ray should hit its (rotated) top face.
  addBox(w, {0, 0, 0}, {1, 1, 1}, 1000.0f, 0.0f, 0.5f, fromAxisAngle({0, 0, 1}, 0.785398f));
  auto hit = raycastNearest(w, {0, 5, 0}, {0, -1, 0}, 10.0f);
  expect(hit.has_value(), "raycast hits the rotated box");
  if (hit) {
    // A 45°-rotated unit cube reaches up to sqrt(2) ~ 1.414; ray hits near there.
    expect(hit->position.y > 1.2f && hit->position.y < 1.5f,
           "rotated box hit at its rotated corner");
    expect(hit->normal.y > 0.4f, "rotated box entry normal faces upward");
  }
}

// A ray whose origin is INSIDE a box must not report a phantom distance-0 hit
// (regression: rayBox used to return t=0 with a stale +X normal, shadowing all
// real hits — which would break a vehicle wheel ray starting inside geometry).
void testRaycastFromInsideBoxIsNotPhantom() {
  World w;
  addBox(w, {0, 0, 0}, {1, 1, 1}, 1000.0f, 0.0f, 0.5f);       // box around the origin
  addSphere(w, {0, -5, 0}, 1.0f, 1000.0f, 0.0f, 0.5f);        // a real target below
  auto hit = raycastNearest(w, {0, 0, 0}, {0, -1, 0}, 10.0f); // origin inside the box
  expect(hit.has_value() && hit->entity == 1 && hit->distance > 3.0f,
         "ray from inside a box skips the phantom t=0 hit and finds the real target");
}

// --- Force fields ------------------------------------------------------------
void testExplosionIsSymmetric() {
  World w;
  const int n = 8;
  for (int i = 0; i < n; ++i) {
    const float a = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i) / n;
    addSphere(w, {std::cos(a) * 3.0f, 0, std::sin(a) * 3.0f}, 0.3f, 1000.0f, 0.5f, 0.5f);
  }
  applyRadialImpulse(w, {0, 0, 0}, 50.0f, 10.0f);
  Vec3 netMomentum;
  bool allOutward = true;
  for (Entity i = 0; i < w.count(); ++i) {
    netMomentum += w.motions[i].linearVelocity * mass(w.bodies[i]);
    if (dot(w.motions[i].linearVelocity, w.transforms[i].position) <= 0.0f) {
      allOutward = false; // velocity should point away from the center
    }
  }
  expect(allOutward, "explosion pushes every body radially outward");
  expect(length(netMomentum) < 1e-2f, "symmetric explosion conserves net momentum (~0)");
}

void testDragDecaysVelocity() {
  World w;
  addSphere(w, {0, 0, 0}, 0.5f, 1000.0f, 0.0f, 0.5f, {10, 0, 0});
  const float dt = 1.0f / 60.0f;
  float prev = length(w.motions[0].linearVelocity);
  for (int i = 0; i < 120; ++i) {
    applyDrag(w, 2.0f, 2.0f, dt);
    const float now = length(w.motions[0].linearVelocity);
    if (now > prev + 1e-4f) {
      expect(false, "drag never increases speed");
    }
    prev = now;
  }
  expect(prev < 1.0f, "drag decays velocity toward zero");
}

// --- Distance constraint (pendulum) ------------------------------------------
void testRigidPendulum() {
  World w;
  addStaticBox(w, {0, 5, 0}, {0.1f, 0.1f, 0.1f}, 0.0f, 0.5f);            // anchor (entity 0)
  const Entity bob = addSphere(w, {2, 5, 0}, 0.3f, 1000.0f, 0.0f, 0.5f); // (entity 1)
  addDistanceConstraint(w, 0, bob, 2.0f);                                // rigid rod, length 2

  StepContext context{.dt = 1.0f / 60.0f, .gravity = {0, -9.81f, 0}};
  const float m = mass(w.bodies[bob]);
  const float e0 = 0.5f * m * lengthSquared(w.motions[bob].linearVelocity) +
                   -m * dot(context.gravity, w.transforms[bob].position);
  float maxLenErr = 0.0f, maxEnergy = e0;
  for (int i = 0; i < 600; ++i) {
    step(w, context);
    const float len = length(w.transforms[bob].position - w.transforms[0].position);
    maxLenErr = std::max(maxLenErr, std::abs(len - 2.0f));
    const float e = 0.5f * m * lengthSquared(w.motions[bob].linearVelocity) +
                    -m * dot(context.gravity, w.transforms[bob].position);
    maxEnergy = std::max(maxEnergy, e);
  }
  expect(maxLenErr < 0.1f, "rigid pendulum keeps its length (~restLength)");
  expect(maxEnergy < e0 * 1.15f + 1.0f, "rigid pendulum does not gain energy");
}

} // namespace

int main() {
  testRaycastSphere();
  testRaycastRotatedBox();
  testRaycastFromInsideBoxIsNotPhantom();
  testExplosionIsSymmetric();
  testDragDecaysVelocity();
  testRigidPendulum();
  if (failures == 0) {
    std::println("All EngineFeaturesTests passed.");
    return 0;
  }
  std::println(std::cerr, "{} EngineFeaturesTests failed.", failures);
  return 1;
}
