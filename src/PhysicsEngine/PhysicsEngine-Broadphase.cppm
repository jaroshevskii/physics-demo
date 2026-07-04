export module PhysicsEngine:Broadphase;

import std;
import :Math;
import :Components;
import :World;

// Broadphase: cheaply reject pairs that cannot touch, producing a short list of
// candidate pairs for the (more expensive) narrowphase.
//
// Two implementations live behind the SAME `broadphase(World, StepContext)`
// signature: a brute-force O(n^2) AABB sweep (`bruteForceBroadphase`, the test
// oracle) and Sweep-and-Prune (`sweepAndPruneBroadphase`, the default —
// sort bodies on one axis, then only compare bodies whose x-intervals overlap;
// O(n log n + pairs) and cache-friendly, the classic Bullet/Chipmunk approach).
// Both fatten AABBs by `context.aabbMargin` so speculative contacts (CCD-lite) get
// found a frame early, and both emit the SAME sorted, de-duplicated pair list,
// so swapping them never changes the simulation.
export namespace Physics {

/// A candidate colliding pair, `(i, j)` with `i < j` into the `World` arrays.
using Pair = std::pair<Entity, Entity>;

namespace detail {

inline std::vector<AABB> fattenedAABBs(const World &w, float margin) {
  const Vec3 m{margin, margin, margin};
  std::vector<AABB> boxes;
  boxes.reserve(w.count());
  for (Entity i = 0; i < w.count(); ++i) {
    AABB b = computeAABB(w.transforms[i], w.colliders[i]);
    boxes.push_back({b.min - m, b.max + m});
  }
  return boxes;
}

inline bool bothStatic(const World &w, Entity i, Entity j) {
  return w.bodies[i].type == BodyType::Static && w.bodies[j].type == BodyType::Static;
}

} // namespace detail

/// Brute-force O(n^2) sweep. The correctness/determinism oracle for SAP.
inline std::vector<Pair> bruteForceBroadphase(const World &w, const StepContext &context) {
  const std::vector<AABB> boxes = detail::fattenedAABBs(w, context.aabbMargin);
  std::vector<Pair> pairs;
  const Entity n = w.count();
  for (Entity i = 0; i < n; ++i) {
    for (Entity j = i + 1; j < n; ++j) {
      if (detail::bothStatic(w, i, j)) {
        continue;
      }
      if (overlaps(boxes[i], boxes[j])) {
        pairs.emplace_back(i, j);
      }
    }
  }
  return pairs; // already sorted by (i, j) and duplicate-free
}

/// Sweep-and-Prune: sort body indices by AABB min.x, then for each body only
/// test the bodies that follow it until one starts past its max.x. Emits the
/// same set as brute force (then sorted + de-duplicated to match its order).
inline std::vector<Pair> sweepAndPruneBroadphase(const World &w, const StepContext &context) {
  const std::vector<AABB> boxes = detail::fattenedAABBs(w, context.aabbMargin);
  const Entity n = w.count();

  std::vector<Entity> order(n);
  for (Entity i = 0; i < n; ++i) {
    order[i] = i;
  }
  std::sort(order.begin(), order.end(),
            [&boxes](Entity a, Entity b) { return boxes[a].min.x < boxes[b].min.x; });

  std::vector<Pair> pairs;
  for (std::size_t a = 0; a < order.size(); ++a) {
    const Entity i = order[a];
    const float iMaxX = boxes[i].max.x;
    for (std::size_t b = a + 1; b < order.size(); ++b) {
      const Entity j = order[b];
      if (boxes[j].min.x > iMaxX) {
        break; // sorted on x: nothing further can overlap i on the x axis
      }
      if (!detail::bothStatic(w, i, j) && overlaps(boxes[i], boxes[j])) {
        pairs.emplace_back(std::min(i, j), std::max(i, j));
      }
    }
  }

  std::sort(pairs.begin(), pairs.end());
  pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
  return pairs; // identical to bruteForceBroadphase(w, context)
}

/// Picks the broadphase for this scene: brute force below the threshold, SAP
/// above it. Both emit the identical pair list, so the choice is pure
/// performance.
inline std::vector<Pair> broadphase(const World &w, const StepContext &context) {
  // Brute force wins for tiny scenes; SAP wins as body count grows. Output is
  // identical either way, so the threshold is purely a performance choice.
  return w.count() <= 24 ? bruteForceBroadphase(w, context) : sweepAndPruneBroadphase(w, context);
}

/// Convenience overload for call sites / tests without an explicit context.
inline std::vector<Pair> broadphase(const World &w) { return broadphase(w, StepContext{}); }

/// Allocation-free broadphase for the per-substep hot path: the caller owns the
/// output `pairs` list and the `boxes` / `order` scratch buffers and reuses them
/// across steps, so a steady simulation does zero broadphase allocation. Produces
/// the exact same pair list as `broadphase(w, context)`.
inline void broadphaseInto(const World &w, const StepContext &context, std::vector<Pair> &pairs,
                           std::vector<AABB> &boxes, std::vector<Entity> &order) {
  const Vec3 margin{context.aabbMargin, context.aabbMargin, context.aabbMargin};
  const Entity n = w.count();
  boxes.clear();
  boxes.reserve(n);
  for (Entity i = 0; i < n; ++i) {
    const AABB b = computeAABB(w.transforms[i], w.colliders[i]);
    boxes.push_back({b.min - margin, b.max + margin});
  }

  pairs.clear();
  if (n <= 24) { // brute force: already sorted by (i, j) and duplicate-free
    for (Entity i = 0; i < n; ++i) {
      for (Entity j = i + 1; j < n; ++j) {
        if (!detail::bothStatic(w, i, j) && overlaps(boxes[i], boxes[j])) {
          pairs.emplace_back(i, j);
        }
      }
    }
    return;
  }

  order.resize(n);
  for (Entity i = 0; i < n; ++i) {
    order[i] = i;
  }
  std::sort(order.begin(), order.end(),
            [&boxes](Entity a, Entity b) { return boxes[a].min.x < boxes[b].min.x; });
  for (std::size_t a = 0; a < order.size(); ++a) {
    const Entity i = order[a];
    const float iMaxX = boxes[i].max.x;
    for (std::size_t b = a + 1; b < order.size(); ++b) {
      const Entity j = order[b];
      if (boxes[j].min.x > iMaxX) {
        break;
      }
      if (!detail::bothStatic(w, i, j) && overlaps(boxes[i], boxes[j])) {
        pairs.emplace_back(std::min(i, j), std::max(i, j));
      }
    }
  }
  std::sort(pairs.begin(), pairs.end());
  pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
}

} // namespace Physics
