export module PhysicsEngine:ContactCache;

import std;
import :Math;

// Persistent, frame-to-frame contact memory — the backbone of **warm-starting**
// (seed each contact with last frame's impulse so stacks converge in a few
// iterations instead of jittering) and of **collision events** (diff this
// frame's contacts against last frame's to classify Began / Persisted / Ended).
//
// These types are deliberately dependency-free (only `std`) so `World` can embed
// a `ContactCache` by value without an import cycle, and so the whole cache is
// `operator==`-comparable — which keeps `World` a value type and the golden
// tests meaningful. The cache is always kept sorted by `PairKey`, so every
// derived output (events, warm-start order) is deterministic.
export namespace Physics {

/// A body-pair key, canonicalized so (i,j) and (j,i) map to the same entry.
struct PairKey {
  std::size_t a = 0;
  std::size_t b = 0;
  auto operator<=>(const PairKey &) const = default;
  bool operator==(const PairKey &) const = default;
};

/// Builds a canonical `PairKey` (smaller index first) so pair order never
/// matters.
inline PairKey makeKey(std::size_t i, std::size_t j) {
  return i < j ? PairKey{i, j} : PairKey{j, i};
}

/// One cached contact point: a stable feature id (so it can be matched to the
/// same point next frame) plus the accumulated impulses to warm-start with.
struct CachedPoint {
  std::uint16_t id = 0;
  float normalImpulse = 0.0f;
  float tangentImpulse1 = 0.0f;
  float tangentImpulse2 = 0.0f;
  bool operator==(const CachedPoint &) const = default;
};

/// One body pair's remembered contacts, carried across frames for warm-starting
/// and event diffing.
struct PersistentManifold {
  PairKey key;
  std::array<CachedPoint, 4> points{};
  int count = 0;
  /// Last step index this pair was in contact (drives event diffing).
  std::uint64_t lastStep = 0;
  // Summary for collision events (filled when impulses are stored):
  Vec3 normal;               ///< contact normal (A -> B)
  Vec3 point;                ///< contact centroid, world space
  float totalImpulse = 0.0f; ///< summed accumulated normal impulse (impact strength)
  bool operator==(const PersistentManifold &) const = default;
};

/// The whole persistent contact store: one manifold per touching pair, always
/// kept sorted by `PairKey` for deterministic output.
struct ContactCache {
  std::vector<PersistentManifold> manifolds; ///< kept sorted by key
  bool operator==(const ContactCache &) const = default;
};

/// Binary-searches the sorted cache for `key`; returns nullptr if absent.
inline const PersistentManifold *find(const ContactCache &cache, PairKey key) {
  auto it = std::lower_bound(cache.manifolds.begin(), cache.manifolds.end(), key,
                             [](const PersistentManifold &m, PairKey k) { return m.key < k; });
  if (it != cache.manifolds.end() && it->key == key) {
    return &*it;
  }
  return nullptr;
}

} // namespace Physics
