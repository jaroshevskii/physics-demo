// Narrowphase unit tests: overlap booleans and contact-manifold correctness
// (normal direction, penetration depth, contact count) for every shape pair,
// checked against hand-computed values.

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

bool approx(float a, float b, float eps = 1e-3f) { return std::abs(a - b) < eps; }
bool approxVec(Vec3 a, Vec3 b, float eps = 1e-3f) {
  return approx(a.x, b.x, eps) && approx(a.y, b.y, eps) && approx(a.z, b.z, eps);
}

Transform at(Vec3 p, Quat q = {}) { return Transform{.position = p, .orientation = q}; }
Collider sphere(float r) { return Collider{.type = ShapeType::Sphere, .radius = r}; }
Collider box(Vec3 h) { return Collider{.type = ShapeType::Box, .halfExtents = h}; }

void testSphereSphere() {
  // Overlapping along +x: centers 1.5 apart, radii 1+1 => penetration 0.5.
  auto c = collide(at({0, 0, 0}), sphere(1.0f), at({1.5f, 0, 0}), sphere(1.0f));
  expect(c.has_value(), "sphere-sphere overlap detected");
  if (c) {
    expect(approxVec(c->normal, {1, 0, 0}), "sphere-sphere normal is +x (A->B)");
    expect(c->count == 1, "sphere-sphere has one contact");
    expect(approx(c->contacts[0].penetration, 0.5f), "sphere-sphere penetration 0.5");
  }
  // Separated.
  expect(!collide(at({0, 0, 0}), sphere(1.0f), at({3, 0, 0}), sphere(1.0f)).has_value(),
         "sphere-sphere separated => no collision");
}

void testSphereBox() {
  // Sphere above a box, resting-overlap. Box top face at y=0.5; sphere center
  // at y=0.9, radius 0.5 => bottom at 0.4, penetration 0.1. Normal A(sphere)->
  // B(box) points down (-y).
  auto c = collide(at({0, 0.9f, 0}), sphere(0.5f), at({0, 0, 0}), box({1, 0.5f, 1}));
  expect(c.has_value(), "sphere-box overlap detected");
  if (c) {
    expect(approxVec(c->normal, {0, -1, 0}), "sphere-box normal is -y (sphere->box)");
    expect(approx(c->contacts[0].penetration, 0.1f), "sphere-box penetration 0.1");
  }
  // Box (A) vs sphere (B): normal must flip to +y (A=box -> B=sphere upward).
  auto c2 = collide(at({0, 0, 0}), box({1, 0.5f, 1}), at({0, 0.9f, 0}), sphere(0.5f));
  expect(c2.has_value() && approxVec(c2->normal, {0, 1, 0}), "box-sphere normal is +y (A->B)");
  // Separated.
  expect(!collide(at({0, 2, 0}), sphere(0.5f), at({0, 0, 0}), box({1, 0.5f, 1})).has_value(),
         "sphere-box separated => no collision");
}

void testBoxBoxFace() {
  // Two axis-aligned unit cubes overlapping along x by 0.1.
  auto c =
      collide(at({0, 0, 0}), box({0.5f, 0.5f, 0.5f}), at({0.9f, 0, 0}), box({0.5f, 0.5f, 0.5f}));
  expect(c.has_value(), "box-box face overlap detected");
  if (c) {
    expect(approxVec(c->normal, {1, 0, 0}), "box-box normal is +x (A->B)");
    expect(c->count >= 1, "box-box has at least one contact");
    expect(approx(c->contacts[0].penetration, 0.1f, 1e-2f), "box-box penetration ~0.1");
  }
  // A stacked box produces a stable multi-point (face) manifold.
  auto stack =
      collide(at({0, 0, 0}), box({0.5f, 0.5f, 0.5f}), at({0, 0.95f, 0}), box({0.5f, 0.5f, 0.5f}));
  expect(stack.has_value(), "box-box stack overlap detected");
  if (stack) {
    expect(approxVec(stack->normal, {0, 1, 0}), "stacked box normal is +y");
    expect(stack->count == 4, "stacked box face manifold has 4 contacts");
  }
  // Separated.
  expect(!collide(at({0, 0, 0}), box({0.5f, 0.5f, 0.5f}), at({2, 0, 0}), box({0.5f, 0.5f, 0.5f}))
              .has_value(),
         "box-box separated => no collision");
}

void testAABB() {
  const AABB a{{-1, -1, -1}, {1, 1, 1}};
  expect(overlaps(a, {{0, 0, 0}, {2, 2, 2}}), "AABB overlap");
  expect(!overlaps(a, {{2, 2, 2}, {3, 3, 3}}), "AABB disjoint");
  // Rotated box AABB grows (a 45° cube's AABB half-extent is ~sqrt(2)/2 wider).
  const AABB rot =
      computeAABB(at({0, 0, 0}, fromAxisAngle({0, 0, 1}, 0.785398f)), box({0.5f, 0.5f, 0.5f}));
  expect(rot.max.x > 0.6f, "rotated box AABB expands");
}

} // namespace

int main() {
  testSphereSphere();
  testSphereBox();
  testBoxBoxFace();
  testAABB();
  if (failures == 0) {
    std::println("All CollisionTests passed.");
    return 0;
  }
  std::println(std::cerr, "{} CollisionTests failed.", failures);
  return 1;
}
