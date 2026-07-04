export module PhysicsEngine:Events;

import std;
import :Math;
import :ContactCache;

// Collision events, mirroring RealityKit's CollisionEvents.Began / .Updated /
// .Ended. Rather than a subscription callback, the engine exposes a pure diff of
// two contact caches (previous frame vs current frame); a pair present now but
// not before is `Began`, present in both is `Persisted`, gone now is `Ended`.
// Features then surface these as Composable-Architecture Actions (see the demo
// features) — the C++ analog of `scene.subscribe(to: CollisionEvents.Began)`.
//
// Both caches are sorted by `PairKey`, so the merge below is a linear, fully
// deterministic sweep — event order never depends on hashing or timing.
export namespace Physics {

/// One collision between a pair of bodies, tagged with its lifecycle phase —
/// the engine's analog of a RealityKit `CollisionEvents.Began/.Updated/.Ended`
/// payload.
struct CollisionEvent {
  /// Whether the contact just started, is continuing, or just ended — mirroring
  /// `CollisionEvents.Began` / `.Updated` / `.Ended`.
  enum class Phase : std::uint8_t { Began, Persisted, Ended };
  Phase phase = Phase::Began;
  /// The colliding pair, in canonical order (a < b).
  std::size_t a = 0, b = 0;
  /// Contact centroid, in world space.
  Vec3 position;
  /// Contact normal, pointing from A to B.
  Vec3 normal;
  /// Impact strength: the summed normal impulse (0 for `Ended`).
  float impulse = 0.0f;
  bool operator==(const CollisionEvent &) const = default;
};

/// Diffs two frames' contact caches into an ordered event list: a pair present
/// now but not before is `Began`, present in both is `Persisted`, and gone now
/// is `Ended`. The pure-function equivalent of subscribing to RealityKit's
/// collision events. Both caches are sorted by `PairKey`, so this is a linear,
/// fully deterministic merge.
inline std::vector<CollisionEvent> collisionEvents(const ContactCache &prev,
                                                   const ContactCache &cur) {
  std::vector<CollisionEvent> events;
  std::size_t i = 0, j = 0;
  const auto &P = prev.manifolds;
  const auto &C = cur.manifolds;
  while (i < P.size() || j < C.size()) {
    const bool haveP = i < P.size();
    const bool haveC = j < C.size();
    if (haveC && (!haveP || C[j].key < P[i].key)) {
      // In current only => Began.
      events.push_back({CollisionEvent::Phase::Began, C[j].key.a, C[j].key.b, C[j].point,
                        C[j].normal, C[j].totalImpulse});
      ++j;
    } else if (haveP && (!haveC || P[i].key < C[j].key)) {
      // In previous only => Ended.
      events.push_back(
          {CollisionEvent::Phase::Ended, P[i].key.a, P[i].key.b, P[i].point, P[i].normal, 0.0f});
      ++i;
    } else {
      // In both => Persisted.
      events.push_back({CollisionEvent::Phase::Persisted, C[j].key.a, C[j].key.b, C[j].point,
                        C[j].normal, C[j].totalImpulse});
      ++i;
      ++j;
    }
  }
  return events;
}

} // namespace Physics
