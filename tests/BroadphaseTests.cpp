// Broadphase parity: the spatial-hash grid must emit EXACTLY the same candidate
// pair list (same order, same de-duplication) as the brute-force oracle, across
// many randomized scenes. Identical output proves both correctness and that
// swapping the broadphase never changes the simulation.

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

World randomScene(std::mt19937 &rng, int nSpheres, int nBoxes) {
  World w;
  addStaticBox(w, {0, -0.5f, 0}, {12, 0.5f, 12}, 0.3f, 0.5f); // a big static floor (the "big body")
  std::uniform_real_distribution<float> pos(-6.0f, 6.0f), y(0.5f, 8.0f), r(0.2f, 0.7f);
  for (int i = 0; i < nSpheres; ++i) {
    addSphere(w, {pos(rng), y(rng), pos(rng)}, r(rng), 1.0f, 0.3f, 0.5f);
  }
  for (int i = 0; i < nBoxes; ++i) {
    const float h = r(rng);
    addBox(w, {pos(rng), y(rng), pos(rng)}, {h, h, h}, 1.0f, 0.3f, 0.5f);
  }
  return w;
}

void testGridMatchesBruteForce() {
  std::mt19937 rng(12345);
  StepContext context; // default aabbMargin
  int mismatches = 0;
  for (int trial = 0; trial < 40; ++trial) {
    const World w = randomScene(rng, 60, 40); // > 24 bodies => SAP path is used
    const auto sap = sweepAndPruneBroadphase(w, context);
    const auto brute = bruteForceBroadphase(w, context);
    if (sap != brute) {
      ++mismatches;
    }
  }
  expect(mismatches == 0, "sweep-and-prune broadphase == brute-force broadphase on all scenes");
}

void testDispatchMatchesToo() {
  std::mt19937 rng(999);
  StepContext context;
  const World w = randomScene(rng, 80, 20);
  expect(broadphase(w, context) == bruteForceBroadphase(w, context),
         "broadphase() dispatch agrees with the brute-force oracle");
}

// The allocation-free hot-path variant (buffers owned by the caller, reused
// across steps) must emit exactly the same pair list as the returning form —
// both the small-scene brute-force branch and the large-scene SAP branch.
void testBroadphaseIntoMatches() {
  std::mt19937 rng(2024);
  StepContext context;
  std::vector<Pair> pairs;
  std::vector<AABB> boxes;
  std::vector<Entity> order;
  int mismatches = 0;
  for (int trial = 0; trial < 40; ++trial) {
    const World small = randomScene(rng, 8, 6);   // <= 24 bodies => brute-force branch
    const World large = randomScene(rng, 60, 40); // > 24 bodies => SAP branch
    for (const World &w : {small, large}) {
      broadphaseInto(w, context, pairs, boxes, order); // reuse buffers across calls
      if (pairs != broadphase(w, context)) {
        ++mismatches;
      }
    }
  }
  expect(mismatches == 0, "broadphaseInto() == broadphase() across scenes and reused buffers");
}

} // namespace

int main() {
  testGridMatchesBruteForce();
  testDispatchMatchesToo();
  testBroadphaseIntoMatches();
  if (failures == 0) {
    std::println("All BroadphaseTests passed.");
    return 0;
  }
  std::println(std::cerr, "{} BroadphaseTests failed.", failures);
  return 1;
}
