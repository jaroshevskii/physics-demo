export module PhysicsEngine:Collision;

import std;
import :Math;
import :Components;

// Narrowphase collision detection. Each `collide*` routine takes two shapes and
// returns a `Collision` describing how to separate them: a unit `normal`
// (pointing from A to B) and 1–4 contact points, each with a penetration depth.
// This `{normal, depth, points}` bundle is the *contact manifold* — the single
// input the impulse solver needs.
//
// Shapes handled: sphere–sphere, sphere–box (closest-point), and box–box via
// the Separating Axis Theorem (15 candidate axes) followed by face clipping
// (Sutherland–Hodgman) for stable multi-point contacts.
export namespace Physics {

/// A single point of a contact manifold: where the shapes touch, how deeply,
/// and a feature id for frame-to-frame matching.
struct ContactPoint {
  Vec3 position;
  /// Signed overlap: positive = interpenetration, negative = a gap (a
  /// "speculative" contact, up to `speculativeDistance`). The solver treats the
  /// separation as `-penetration`.
  float penetration = 0.0f;
  /// Stable feature id so the same contact can be matched frame-to-frame for
  /// warm-starting (0 for single-point sphere contacts).
  std::uint16_t id = 0;
};

/// A contact manifold: the shared `normal` plus up to four contact points — the
/// single input the impulse solver needs to separate a colliding pair.
struct Collision {
  Vec3 normal;                          ///< unit, from A to B
  std::array<ContactPoint, 4> contacts; ///< up to 4 (box faces); 1 for spheres
  int count = 0;
};

} // namespace Physics

// --- Internal helpers (module-local, not exported) ---------------------------
namespace Physics {
namespace {

// An oriented box in world space, unpacked for the SAT routines.
struct OBB {
  Vec3 center;
  Mat3 axes; // columns = local x/y/z axes in world space
  Vec3 half;
};

OBB toOBB(const Transform &t, const Collider &c) {
  return {t.position, toMat3(t.orientation), c.halfExtents};
}

Vec3 axis(const Mat3 &m, int i) { return i == 0 ? m.c0 : (i == 1 ? m.c1 : m.c2); }
float component(Vec3 v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }

// Projection radius of an OBB onto a world-space direction L.
float projectedRadius(const OBB &b, Vec3 L) {
  return std::abs(dot(L, b.axes.c0)) * b.half.x + std::abs(dot(L, b.axes.c1)) * b.half.y +
         std::abs(dot(L, b.axes.c2)) * b.half.z;
}

// Separation along axis L (assumed unit): >0 means the boxes are disjoint here.
float separationOn(const OBB &a, const OBB &b, Vec3 L) {
  const Vec3 t = b.center - a.center;
  return std::abs(dot(t, L)) - (projectedRadius(a, L) + projectedRadius(b, L));
}

// The four world-space vertices of one box face (axisIndex + sign pick a face).
std::array<Vec3, 4> faceVertices(const OBB &b, int axisIndex, float sign) {
  const int u = (axisIndex + 1) % 3;
  const int v = (axisIndex + 2) % 3;
  const Vec3 n = axis(b.axes, axisIndex) * (sign * component(b.half, axisIndex));
  const Vec3 du = axis(b.axes, u) * component(b.half, u);
  const Vec3 dv = axis(b.axes, v) * component(b.half, v);
  const Vec3 c = b.center + n;
  return {c - du - dv, c + du - dv, c + du + dv, c - du + dv};
}

// Clip a convex polygon to the half-space {p : dot(p - planePoint, inward) >= 0}.
// Sutherland–Hodgman on fixed-size stack buffers: the incident face has <= 4
// vertices and each of the 4 clip planes adds at most one, so 8 slots always
// suffice. Reads `count` vertices from `in`, writes the result to `out`, and
// returns the new count — no heap allocation on this per-pair hot path.
int clipToPlane(const Vec3 *in, int count, Vec3 planePoint, Vec3 inward, Vec3 *out) {
  int outCount = 0;
  for (int i = 0; i < count; ++i) {
    const Vec3 cur = in[i];
    const Vec3 nxt = in[(i + 1) % count];
    const float dCur = dot(cur - planePoint, inward);
    const float dNxt = dot(nxt - planePoint, inward);
    if (dCur >= 0.0f) {
      out[outCount++] = cur;
    }
    if ((dCur >= 0.0f) != (dNxt >= 0.0f)) {
      const float tt = dCur / (dCur - dNxt);
      out[outCount++] = cur + (nxt - cur) * tt;
    }
  }
  return outCount;
}

// Closest points between two segments (p1->q1) and (p2->q2). Returns their
// midpoint — the contact point for an edge–edge collision.
Vec3 closestPointsMidpoint(Vec3 p1, Vec3 q1, Vec3 p2, Vec3 q2) {
  const Vec3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
  const float a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
  float s = 0.0f, t = 0.0f;
  const float c = dot(d1, r);
  const float b = dot(d1, d2);
  const float denom = a * e - b * b;
  if (denom > 1e-8f) {
    s = clampf((b * f - c * e) / denom, 0.0f, 1.0f);
  }
  t = (b * s + f) / (e > 1e-8f ? e : 1.0f);
  t = clampf(t, 0.0f, 1.0f);
  s = clampf((b * t - c) / (a > 1e-8f ? a : 1.0f), 0.0f, 1.0f);
  const Vec3 c1 = p1 + d1 * s;
  const Vec3 c2 = p2 + d2 * t;
  return (c1 + c2) * 0.5f;
}

} // namespace
} // namespace Physics

export namespace Physics {

// --- Sphere vs sphere --------------------------------------------------------
/// Collides two spheres, returning a one-point manifold with the normal from A
/// to B, or `nullopt` if they are apart by more than `speculativeDistance`.
std::optional<Collision> collideSphereSphere(const Transform &ta, const Collider &ca,
                                             const Transform &tb, const Collider &cb,
                                             float speculativeDistance = 0.0f) {
  const Vec3 d = tb.position - ta.position;
  const float r = ca.radius + cb.radius;
  const float reach = r + speculativeDistance;
  const float dist2 = lengthSquared(d);
  if (dist2 >= reach * reach) {
    return std::nullopt;
  }
  const float dist = std::sqrt(dist2);
  const Vec3 normal = dist > 1e-8f ? d / dist : Vec3{0.0f, 1.0f, 0.0f};
  const float penetration = r - dist; // may be negative (a gap) for speculative
  Collision collision;
  collision.normal = normal;
  collision.count = 1;
  collision.contacts[0] = {ta.position + normal * (ca.radius - penetration * 0.5f), penetration, 0};
  return collision;
}

// --- Sphere (A) vs box (B) ---------------------------------------------------
/// Collides sphere A against oriented box B via the box's closest surface point,
/// returning a one-point manifold with the normal A->B (sphere toward box), or
/// `nullopt` if they are apart by more than `speculativeDistance`.
std::optional<Collision> collideSphereBox(const Transform &ts, const Collider &cs,
                                          const Transform &tb, const Collider &cb,
                                          float speculativeDistance = 0.0f) {
  const Mat3 R = toMat3(tb.orientation);
  const Mat3 Rt = transpose(R);
  const Vec3 local = Rt * (ts.position - tb.position); // sphere center in box space
  const Vec3 h = cb.halfExtents;
  const Vec3 closest{clampf(local.x, -h.x, h.x), clampf(local.y, -h.y, h.y),
                     clampf(local.z, -h.z, h.z)};
  const Vec3 diff = local - closest;
  const float dist2 = lengthSquared(diff);
  const float reach = cs.radius + speculativeDistance;
  if (dist2 > reach * reach) {
    return std::nullopt;
  }

  Vec3 outwardLocal; // from box surface toward sphere center
  float penetration;
  if (dist2 > 1e-8f) {
    const float dist = std::sqrt(dist2);
    outwardLocal = diff / dist;
    penetration = cs.radius - dist;
  } else {
    // Sphere center inside the box: push out along the least-penetrating face.
    const Vec3 faceDist = h - absVec(local);
    int a = 0;
    if (faceDist.y < faceDist.x) {
      a = 1;
    }
    if (component(faceDist, 2) < component(faceDist, a)) {
      a = 2;
    }
    const float sign = component(local, a) < 0.0f ? -1.0f : 1.0f;
    outwardLocal = a == 0 ? Vec3{sign, 0, 0} : (a == 1 ? Vec3{0, sign, 0} : Vec3{0, 0, sign});
    penetration = cs.radius + component(faceDist, a);
  }
  const Vec3 outward = R * outwardLocal; // world, box -> sphere
  Collision collision;
  collision.normal = -outward; // A(sphere) -> B(box)
  collision.count = 1;
  collision.contacts[0] = {tb.position + R * closest, penetration};
  return collision;
}

// --- Box vs box (SAT + face clipping) ----------------------------------------
/// Collides two oriented boxes via the Separating Axis Theorem, returning an
/// edge-edge (one-point) or clipped face (up to four-point) manifold with the
/// normal A->B, or `nullopt` if a separating axis exceeds `speculativeDistance`.
std::optional<Collision> collideBoxBox(const Transform &ta, const Collider &ca, const Transform &tb,
                                       const Collider &cb, float speculativeDistance = 0.0f) {
  const OBB A = toOBB(ta, ca);
  const OBB B = toOBB(tb, cb);
  const Vec3 tAB = B.center - A.center;

  // 1) SAT over 6 face axes + 9 edge cross-axes. Track the best (largest,
  //    i.e. least-overlapping) separation in each category.
  float bestFaceSep = -std::numeric_limits<float>::max();
  int bestFaceAxis = 0; // 0..2 => A face, 3..5 => B face
  Vec3 bestFaceNormal;  // oriented A->B

  auto considerFace = [&](Vec3 L, int id) {
    const float len = length(L);
    if (len < 1e-6f) {
      return;
    }
    L = L / len;
    const float sep = separationOn(A, B, L);
    if (sep > bestFaceSep) {
      bestFaceSep = sep;
      bestFaceAxis = id;
      bestFaceNormal = dot(tAB, L) < 0.0f ? -L : L; // orient A->B
    }
  };
  for (int i = 0; i < 3; ++i) {
    considerFace(axis(A.axes, i), i);
  }
  for (int i = 0; i < 3; ++i) {
    considerFace(axis(B.axes, i), 3 + i);
  }
  if (bestFaceSep > speculativeDistance) {
    return std::nullopt; // a face axis separates them beyond the speculative reach
  }

  float bestEdgeSep = -std::numeric_limits<float>::max();
  Vec3 bestEdgeNormal;
  int bestEdgeA = 0, bestEdgeB = 0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Vec3 L = cross(axis(A.axes, i), axis(B.axes, j));
      const float len = length(L);
      if (len < 1e-6f) {
        continue; // parallel edges — degenerate axis
      }
      L = L / len;
      const float sep = separationOn(A, B, L);
      if (sep > speculativeDistance) {
        return std::nullopt; // an edge axis separates them beyond the speculative reach
      }
      if (sep > bestEdgeSep) {
        bestEdgeSep = sep;
        bestEdgeA = i;
        bestEdgeB = j;
        bestEdgeNormal = dot(tAB, L) < 0.0f ? -L : L;
      }
    }
  }

  // 2) Prefer a face contact unless an edge axis is clearly deeper-separating
  //    (bias avoids normal flip-flap between near-equal axes).
  constexpr float kEdgeBias = 0.01f;
  if (bestEdgeSep > bestFaceSep + kEdgeBias) {
    // --- Edge–edge: one contact at the closest points of the two edges. ---
    const Vec3 n = bestEdgeNormal;
    // Support edge on A along +n and on B along -n.
    const Vec3 eaDir = axis(A.axes, bestEdgeA);
    const Vec3 ebDir = axis(B.axes, bestEdgeB);
    // Build the two edge midlines by picking the box extreme in n (A) / -n (B).
    auto supportEdge = [](const OBB &box, Vec3 dir, int edgeAxis) {
      const int u = (edgeAxis + 1) % 3, v = (edgeAxis + 2) % 3;
      Vec3 base = box.center;
      base += axis(box.axes, u) *
              (dot(dir, axis(box.axes, u)) >= 0 ? component(box.half, u) : -component(box.half, u));
      base += axis(box.axes, v) *
              (dot(dir, axis(box.axes, v)) >= 0 ? component(box.half, v) : -component(box.half, v));
      const Vec3 half = axis(box.axes, edgeAxis) * component(box.half, edgeAxis);
      return std::pair<Vec3, Vec3>{base - half, base + half};
    };
    const auto [pa, qa] = supportEdge(A, n, bestEdgeA);
    const auto [pb, qb] = supportEdge(B, -n, bestEdgeB);
    (void)eaDir;
    (void)ebDir;
    Collision collision;
    collision.normal = n;
    collision.count = 1;
    const std::uint16_t edgeId =
        static_cast<std::uint16_t>(0x8000u | (bestEdgeA << 4) | bestEdgeB); // stable, distinct
    collision.contacts[0] = {closestPointsMidpoint(pa, qa, pb, qb), -bestEdgeSep, edgeId};
    return collision;
  }

  // --- Face contact: clip the incident face against the reference face. ---
  const bool refIsA = bestFaceAxis < 3;
  const OBB &ref = refIsA ? A : B;
  const OBB &inc = refIsA ? B : A;
  const int refAxisIdx = bestFaceAxis % 3;
  // Reference face normal, pointing from the reference box toward the incident.
  Vec3 refNormalWorld = axis(ref.axes, refAxisIdx);
  const Vec3 refToInc = inc.center - ref.center;
  const float refSign = dot(refToInc, refNormalWorld) < 0.0f ? -1.0f : 1.0f;
  refNormalWorld = refNormalWorld * refSign;

  // Incident face: the incident box face whose outward normal most opposes the
  // reference normal.
  int incAxis = 0;
  float incBest = std::numeric_limits<float>::max();
  float incSign = 1.0f;
  for (int i = 0; i < 3; ++i) {
    for (float s : {1.0f, -1.0f}) {
      const float d = dot(axis(inc.axes, i) * s, refNormalWorld);
      if (d < incBest) {
        incBest = d;
        incAxis = i;
        incSign = s;
      }
    }
  }

  // Ping-pong between two stack buffers as we clip against each plane (see
  // `clipToPlane`): starts with the 4 incident-face vertices, grows to <= 8.
  std::array<Vec3, 8> bufA{}, bufB{};
  int count = 0;
  for (const Vec3 &vtx : faceVertices(inc, incAxis, incSign)) {
    bufA[count++] = vtx;
  }
  Vec3 *poly = bufA.data();
  Vec3 *scratch = bufB.data();

  // Clip against the reference face's four side planes (inward normals).
  const int u = (refAxisIdx + 1) % 3, v = (refAxisIdx + 2) % 3;
  const Vec3 refFaceCenter = ref.center + refNormalWorld * component(ref.half, refAxisIdx);
  for (int a : {u, v}) {
    const Vec3 ax = axis(ref.axes, a);
    const float ext = component(ref.half, a);
    count = clipToPlane(poly, count, ref.center + ax * ext, -ax, scratch); // + side, inward = -ax
    std::swap(poly, scratch);
    count = clipToPlane(poly, count, ref.center - ax * ext, ax, scratch); //  - side, inward = +ax
    std::swap(poly, scratch);
  }

  // Keep clipped points that lie within the speculative reach of the reference
  // face (penetrating, or a small gap for speculative contacts). The feature id
  // encodes the reference face + which incident vertex, so the same point can be
  // matched next frame for warm-starting.
  Collision collision;
  collision.normal = refIsA ? refNormalWorld : -refNormalWorld; // A->B
  const std::uint16_t faceCode = static_cast<std::uint16_t>(
      ((refIsA ? 1 : 0) << 3) | (refAxisIdx << 1) | (refSign > 0.0f ? 1 : 0));
  for (int i = 0; i < count; ++i) {
    const Vec3 p = poly[i];
    const float pen = -dot(p - refFaceCenter, refNormalWorld);
    if (pen >= -speculativeDistance && collision.count < 4) {
      collision.contacts[collision.count++] = {
          p, pen, static_cast<std::uint16_t>((faceCode << 8) | static_cast<std::uint16_t>(i))};
    }
  }
  if (collision.count == 0) {
    return std::nullopt;
  }
  return collision;
}

// --- Dispatch: any pair, always returns the normal oriented A(i)->B(j) -------
/// Dispatches to the right narrowphase routine for the shape pair, always
/// returning the normal oriented A->B (flipping the box-sphere case as needed).
std::optional<Collision> collide(const Transform &ta, const Collider &ca, const Transform &tb,
                                 const Collider &cb, float speculativeDistance = 0.0f) {
  const bool aSphere = ca.type == ShapeType::Sphere;
  const bool bSphere = cb.type == ShapeType::Sphere;
  if (aSphere && bSphere) {
    return collideSphereSphere(ta, ca, tb, cb, speculativeDistance);
  }
  if (aSphere && !bSphere) {
    return collideSphereBox(ta, ca, tb, cb, speculativeDistance);
  }
  if (!aSphere && bSphere) {
    // Reuse sphere-box with roles swapped, then flip the normal to stay A->B.
    if (auto r = collideSphereBox(tb, cb, ta, ca, speculativeDistance)) {
      r->normal = -r->normal;
      return r;
    }
    return std::nullopt;
  }
  return collideBoxBox(ta, ca, tb, cb, speculativeDistance);
}

} // namespace Physics
