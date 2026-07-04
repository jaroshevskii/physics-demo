// Headless micro-benchmark for the fixed-step solver hot path. Not a CI test
// (timing is machine-dependent) — a tool for the performance pass. Builds a busy
// scene (a floor + many dynamic boxes/spheres piling up) and times a fixed
// number of `Physics::step`s, reporting steps/sec and a checksum so we can also
// confirm behaviour is byte-identical before/after an optimization.
//
//   build:  cmake --build build --target PerfBench
//   run:    ./build/PerfBench [bodies] [steps] [threads]

import std;
import PhysicsEngine;

using namespace Physics;

namespace {

// A deterministic pile: a static floor plus a grid of dynamic boxes and spheres
// dropped from varying heights so they collide, stack, and rest.
World makeScene(int bodies) {
  World w;
  addStaticBox(w, {0, -1, 0}, {40, 1, 40}, 0.2f, 0.6f);
  std::uint32_t seed = 12345u;
  auto rnd = [&seed] {
    seed = seed * 1664525u + 1013904223u;
    return static_cast<float>(seed >> 8) / static_cast<float>(1u << 24); // [0,1)
  };
  for (int i = 0; i < bodies; ++i) {
    const float x = (rnd() - 0.5f) * 24.0f;
    const float z = (rnd() - 0.5f) * 24.0f;
    const float y = 1.0f + rnd() * 30.0f;
    if (i % 2 == 0) {
      addBox(w, {x, y, z}, {0.5f, 0.5f, 0.5f}, 1.0f, 0.2f, 0.5f);
    } else {
      addSphere(w, {x, y, z}, 0.5f, 1.0f, 0.3f, 0.5f);
    }
  }
  return w;
}

// A cheap order-independent checksum of all body positions (behaviour fingerprint).
double checksum(const World &w) {
  double sum = 0.0;
  for (Entity i = 0; i < w.count(); ++i) {
    const Vec3 p = w.transforms[i].position;
    sum += static_cast<double>(p.x) * 1.0 + static_cast<double>(p.y) * 3.0 +
           static_cast<double>(p.z) * 7.0;
  }
  return sum;
}

} // namespace

int main(int argc, char **argv) {
  const int bodies = argc > 1 ? std::atoi(argv[1]) : 600;
  const int steps = argc > 2 ? std::atoi(argv[2]) : 600;
  const unsigned threads = argc > 3 ? static_cast<unsigned>(std::atoi(argv[3])) : 1u;

  World w = makeScene(bodies);
  StepContext context;
  context.threadCount = threads;

  const auto t0 = std::chrono::steady_clock::now();
  for (int s = 0; s < steps; ++s) {
    step(w, context);
  }
  const auto t1 = std::chrono::steady_clock::now();

  const double seconds = std::chrono::duration<double>(t1 - t0).count();
  const double perStepMs = seconds * 1000.0 / steps;
  std::println("bodies={} steps={} threads={}", w.count(), steps, threads);
  std::println("total={:.3f}s  per-step={:.4f}ms  steps/sec={:.1f}", seconds, perStepMs,
               steps / seconds);
  std::println("checksum={:.6f}", checksum(w));
  return 0;
}
