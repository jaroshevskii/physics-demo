export module PhysicsEngine:Query;

import std;
import :Math;
import :Components;
import :World;

// Spatial queries against the world, mirroring RealityKit's `scene.raycast`.
// A ray is cast against every body's shape (sphere or OBB) and the nearest hit
// (entity, world position, surface normal, distance) is returned. Used by the
// raycast-vehicle suspension (a ray per wheel) and for picking/targeting.
export namespace Physics {

/// The nearest body a ray struck, as returned by `raycastNearest` — the analog
/// of RealityKit's `CollisionCastHit` from `Scene.raycast`.
struct RayHit {
  /// The entity that was hit.
  Entity entity = 0;
  /// World-space point where the ray met the surface.
  Vec3 position;
  /// World-space surface normal at the hit.
  Vec3 normal;
  /// Distance from the ray origin to `position` (unit-length ray).
  float distance = 0.0f;
  bool operator==(const RayHit &) const = default;
};

} // namespace Physics

namespace Physics {
namespace {

// Ray (origin + t*dir, dir unit) vs sphere. Nearest t in [0,maxDist] or nullopt.
std::optional<float> raySphere(Vec3 o, Vec3 d, Vec3 center, float radius, float maxDist) {
  const Vec3 oc = o - center;
  const float b = dot(oc, d);
  const float c = dot(oc, oc) - radius * radius;
  const float disc = b * b - c;
  if (disc < 0.0f) {
    return std::nullopt;
  }
  const float s = std::sqrt(disc);
  float t = -b - s;
  if (t < 0.0f) {
    t = -b + s; // origin inside the sphere: take the far root
  }
  if (t < 0.0f || t > maxDist) {
    return std::nullopt;
  }
  return t;
}

// Ray vs oriented box. Returns t and the world entry normal.
std::optional<std::pair<float, Vec3>> rayBox(Vec3 o, Vec3 d, const Transform &t, const Collider &c,
                                             float maxDist) {
  const Mat3 R = toMat3(t.orientation);
  const Mat3 Rt = transpose(R);
  const Vec3 lo = Rt * (o - t.position); // ray origin in box space
  const Vec3 ld = Rt * d;                // ray dir in box space
  const Vec3 h = c.halfExtents;

  float tmin = 0.0f, tmax = maxDist;
  int entryAxis = 0;
  float entrySign = 1.0f;
  const float lod[3] = {ld.x, ld.y, ld.z};
  const float loo[3] = {lo.x, lo.y, lo.z};
  const float he[3] = {h.x, h.y, h.z};
  for (int a = 0; a < 3; ++a) {
    if (std::abs(lod[a]) < 1e-8f) {
      if (loo[a] < -he[a] || loo[a] > he[a]) {
        return std::nullopt; // parallel and outside this slab
      }
      continue;
    }
    const float inv = 1.0f / lod[a];
    float t1 = (-he[a] - loo[a]) * inv;
    float t2 = (he[a] - loo[a]) * inv;
    float sign = -1.0f;
    if (t1 > t2) {
      std::swap(t1, t2);
      sign = 1.0f;
    }
    if (t1 > tmin) {
      tmin = t1;
      entryAxis = a;
      entrySign = sign;
    }
    tmax = std::min(tmax, t2);
    if (tmin > tmax) {
      return std::nullopt;
    }
  }
  // tmin stays 0 (its initial value) only when the origin is already inside the
  // box — no forward entry face was crossed. Returning that would be a phantom
  // distance-0 hit with a meaningless normal, so report a miss instead.
  if (tmin <= 0.0f) {
    return std::nullopt;
  }
  Vec3 nLocal{};
  (entryAxis == 0 ? nLocal.x : (entryAxis == 1 ? nLocal.y : nLocal.z)) = entrySign;
  return std::pair<float, Vec3>{tmin, R * nLocal};
}

} // namespace
} // namespace Physics

export namespace Physics {

/// Casts a ray against every body's shape (sphere or OBB) and returns the
/// nearest hit within `maxDist`, or `nullopt` on a miss — the engine's
/// `Scene.raycast`. `dir` need not be normalized; `ignore` skips one entity
/// (e.g. a vehicle chassis casting its own wheel rays).
std::optional<RayHit> raycastNearest(const World &w, Vec3 origin, Vec3 dir, float maxDist,
                                     Entity ignore = std::numeric_limits<Entity>::max()) {
  const Vec3 d = normalized(dir);
  std::optional<RayHit> best;
  for (Entity i = 0; i < w.count(); ++i) {
    if (i == ignore) {
      continue;
    }
    const Collider &c = w.colliders[i];
    if (c.type == ShapeType::Sphere) {
      if (auto t = raySphere(origin, d, w.transforms[i].position, c.radius, maxDist)) {
        if (!best || *t < best->distance) {
          const Vec3 p = origin + d * *t;
          best = RayHit{i, p, normalized(p - w.transforms[i].position), *t};
        }
      }
    } else {
      if (auto hit = rayBox(origin, d, w.transforms[i], c, maxDist)) {
        if (!best || hit->first < best->distance) {
          best = RayHit{i, origin + d * hit->first, hit->second, hit->first};
        }
      }
    }
  }
  return best;
}

} // namespace Physics
