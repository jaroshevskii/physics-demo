export module FireworksFeature;

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;

// Demo — "Fireworks". A large particle system (thousands of points) integrated
// under gravity + drag. Each particle is independent, so the update runs in
// PARALLEL via the engine's `parallelFor` (the same deterministic thread helper
// used for narrowphase) — the CPU-parallel showcase. Particles are visual only
// (not rigid bodies), rendered as a point cloud.
//
// (A GPU path — OpenGL transform-feedback or SDL_GPU compute — is the documented
// next step; see the README. This CPU-parallel version keeps the build free of a
// shader-cross toolchain while still demonstrating data-parallel simulation.)
export namespace Fireworks {

inline constexpr std::size_t maxParticles = 6000;

struct ParticleSystem {
  std::vector<Physics::Vec3> positions;
  std::vector<Physics::Vec3> velocities;
  std::vector<Physics::Vec3> colors; // rgb
  std::vector<float> life;           // seconds remaining (<=0 => dead slot)
  bool operator==(const ParticleSystem &) const = default;

  std::size_t size() const { return positions.size(); }
};

// Advances every particle one step (parallel over particles; deterministic).
inline void stepParticles(ParticleSystem &particles, float dt, Physics::Vec3 gravity, float drag,
                          unsigned threadCount) {
  const float dragScale = 1.0f / (1.0f + dt * drag);
  Physics::parallelFor(particles.size(), threadCount, [&](std::size_t i) {
    if (particles.life[i] <= 0.0f) {
      return;
    }
    particles.velocities[i] += gravity * dt;
    particles.velocities[i] *= dragScale;
    particles.positions[i] += particles.velocities[i] * dt;
    particles.life[i] -= dt;
  });
}

// Deterministic PRNG so bursts are reproducible (testable) given a seed.
inline float randomFloat(std::uint32_t &state) {
  state = state * 1664525u + 1013904223u;
  return static_cast<float>(state >> 8) * (1.0f / 16777216.0f);
}

// Spawns an outward burst of `count` particles from `center`, reusing dead slots.
inline void launchBurst(ParticleSystem &particles, Physics::Vec3 center, int count,
                        std::uint32_t seed) {
  std::uint32_t randomState = seed * 2654435761u + 1u;
  const Physics::Vec3 color{0.4f + 0.6f * randomFloat(randomState),
                            0.4f + 0.6f * randomFloat(randomState),
                            0.4f + 0.6f * randomFloat(randomState)};
  for (int k = 0; k < count; ++k) {
    const float theta = randomFloat(randomState) * 6.2831853f;
    const float z = randomFloat(randomState) * 2.0f - 1.0f;
    const float rr = std::sqrt(std::max(0.0f, 1.0f - z * z));
    Physics::Vec3 dir{rr * std::cos(theta), std::abs(z) * 0.6f + 0.4f, rr * std::sin(theta)};
    const float speed = 6.0f + randomFloat(randomState) * 6.0f;
    const Physics::Vec3 vel = dir * speed;
    const float lifespan = 1.6f + randomFloat(randomState) * 1.4f;
    // Find a dead slot; else append (up to the cap).
    std::size_t slot = particles.size();
    for (std::size_t i = 0; i < particles.size(); ++i) {
      if (particles.life[i] <= 0.0f) {
        slot = i;
        break;
      }
    }
    if (slot == particles.size()) {
      if (particles.size() >= maxParticles) {
        continue;
      }
      particles.positions.push_back(center);
      particles.velocities.push_back(vel);
      particles.colors.push_back(color);
      particles.life.push_back(lifespan);
    } else {
      particles.positions[slot] = center;
      particles.velocities[slot] = vel;
      particles.colors[slot] = color;
      particles.life[slot] = lifespan;
    }
  }
}

struct State {
  Sim::State sim; // reused only for the fixed-timestep clock/accumulator
  ParticleSystem particles;
  std::uint32_t launchCount = 0;
  bool operator==(const State &) const = default;
};

struct Ticked {};
struct Launch {
  Physics::Vec3 center{0, 8, 0};
  bool operator==(const Launch &) const = default;
};
struct ResetTapped {};

using Action = std::variant<Ticked, Launch, ResetTapped>;

inline State initialState() {
  State s;
  s.sim.gravity = {0, -6.0f, 0};
  s.sim.threadCount = 8; // particle update runs parallel
  return s;
}

ComposableArchitecture::Feature<State, Action> body();

} // namespace Fireworks
