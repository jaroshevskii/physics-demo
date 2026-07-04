export module Simulation;

import std;
import PhysicsEngine;

// The fixed-timestep driver that sits between the pure engine and the feature
// reducers. It owns the accumulator that decouples the physics rate from the
// (variable) frame rate — Gaffer's "Fix Your Timestep": accumulate real elapsed
// time, consume it in fixed `fixedDt` chunks, and expose `alpha` so the view can
// interpolate between the previous and current physics states (no render jitter,
// and the simulation never sees a variable dt).
//
// `advance` is a pure function of (state, now): all elapsed time is passed in,
// never read from a clock here, so a feature that calls it stays deterministic
// and testable (the test replays `advance` in its assertion).
export namespace Sim {

/// The fixed physics step: 60 Hz. Every call to `Physics::step` advances by this.
inline constexpr float fixedDt = 1.0f / 60.0f;
/// Per-frame clamp on elapsed time — caps catch-up steps to avoid the "spiral of
/// death" where a slow frame schedules more steps than the next frame can afford.
inline constexpr double maxFrame = 0.25;

/// The full driver state: the live `world`, its one-step-back snapshot, and the
/// accumulator bookkeeping `advance` needs. Value type; compares by all members.
struct State {
  /// The current physics world — the authoritative state, stepped by `advance`.
  Physics::World world;
  /// Snapshot exactly one fixed step behind `world`; the interpolation baseline.
  Physics::World previous;
  /// Wall-clock time (seconds) of the last `advance`, for computing elapsed time.
  double lastTime = 0.0;
  /// Unconsumed real time (seconds) carried between frames, drained in `fixedDt`
  /// chunks.
  double accumulator = 0.0;
  /// Interpolation factor in [0,1): how far `world` is past `previous` this frame.
  float alpha = 0.0f;
  /// When true, `advance` runs no steps and swallows elapsed time (no catch-up).
  bool paused = false;
  /// Gravity applied each step; fed into the `Physics::StepContext`.
  Physics::Vec3 gravity{0.0f, -9.81f, 0.0f};
  /// Total fixed steps run since construction — a monotonic simulation clock.
  std::uint64_t stepCount = 0;
  /// Narrowphase worker threads (1 = serial; demos may raise it).
  unsigned threadCount = 1;

  bool operator==(const State &) const = default;
};

/// Advances the simulation to wall-clock time `now` (seconds). Runs zero or more
/// fixed steps depending on how much time has accumulated, then updates `alpha`.
/// Pure in (state, now): elapsed time is derived from `s.lastTime`, never a clock,
/// so callers stay deterministic and testable.
///
/// `outEvents` (optional): collision events from each fixed step are appended for
/// the feature to drain into Actions. `beforeStep` (optional): a controller run
/// once per fixed step just before the solver — this is where per-step forces
/// (e.g. a vehicle's suspension) belong, so they stay in lockstep with the fixed
/// timestep instead of being applied an unpredictable number of times per frame.
inline void advance(State &s, double now, std::vector<Physics::CollisionEvent> *outEvents = nullptr,
                    const std::function<void(Physics::World &)> &beforeStep = {}) {
  if (s.paused) {
    s.lastTime = now; // freeze: swallow elapsed time so we don't fast-forward on resume
    return;
  }
  double frame = now - s.lastTime;
  s.lastTime = now;
  if (frame < 0.0) {
    frame = 0.0;
  }
  if (frame > maxFrame) {
    frame = maxFrame;
  }
  s.accumulator += frame;

  Physics::StepContext context{.dt = fixedDt, .gravity = s.gravity};
  context.threadCount = s.threadCount;
  while (s.accumulator >= fixedDt) {
    // Snapshot the interpolation baseline right BEFORE each step, so after the
    // loop `previous` is exactly ONE fixed step behind `world`. (Snapshotting
    // once before the loop is a classic bug: on a frame that runs several steps,
    // `previous` ends up many steps behind while `alpha` only covers one step,
    // so the interpolated pose is wrong and jitters — visible "doubling".)
    s.previous = s.world;
    if (beforeStep) {
      beforeStep(s.world);
    }
    const Physics::ContactCache before = outEvents ? s.world.contactCache : Physics::ContactCache{};
    Physics::step(s.world, context);
    if (outEvents) {
      const auto events = Physics::collisionEvents(before, s.world.contactCache);
      outEvents->insert(outEvents->end(), events.begin(), events.end());
    }
    s.accumulator -= fixedDt;
    ++s.stepCount;
  }
  s.alpha = static_cast<float>(s.accumulator / fixedDt);
}

/// Render position for entity `i`: `previous` and current lerped by `alpha`.
/// Returns the current position unblended for entities absent from `previous`.
inline Physics::Vec3 interpolatedPosition(const State &s, Physics::Entity i) {
  const Physics::Vec3 cur = s.world.transforms[i].position;
  if (i >= s.previous.count()) {
    return cur;
  }
  const Physics::Vec3 prev = s.previous.transforms[i].position;
  return prev + (cur - prev) * s.alpha;
}

/// Render orientation for entity `i`: shortest-path normalized-lerp (nlerp) of the
/// previous and current quaternions — smooth for the small per-step rotations here.
/// Returns the current orientation unblended for entities absent from `previous`.
inline Physics::Quat interpolatedOrientation(const State &s, Physics::Entity i) {
  const Physics::Quat cur = s.world.transforms[i].orientation;
  if (i >= s.previous.count()) {
    return cur;
  }
  Physics::Quat prev = s.previous.transforms[i].orientation;
  const float dot = prev.w * cur.w + prev.x * cur.x + prev.y * cur.y + prev.z * cur.z;
  const float sign = dot < 0.0f ? -1.0f : 1.0f; // take the shorter arc
  const float a = s.alpha;
  return Physics::normalized(
      Physics::Quat{prev.w + (cur.w * sign - prev.w) * a, prev.x + (cur.x * sign - prev.x) * a,
                    prev.y + (cur.y * sign - prev.y) * a, prev.z + (cur.z * sign - prev.z) * a});
}

} // namespace Sim
