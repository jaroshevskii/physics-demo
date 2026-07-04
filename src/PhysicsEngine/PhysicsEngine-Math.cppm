export module PhysicsEngine:Math;

import std;

// Minimal 3D linear-algebra used throughout the engine. Everything here is a
// plain value type (POD-like, trivially comparable) so the simulation `World`
// that embeds it stays `operator==`-comparable and copyable — which is what
// lets it live inside a Composable-Architecture feature `State` and be asserted
// against in tests.
//
// `Mat3` is stored as three column vectors: `M * v == c0*v.x + c1*v.y + c2*v.z`.
// A rotation matrix is therefore `[xAxis yAxis zAxis]`.
export namespace Physics {

// --- Vec3 --------------------------------------------------------------------

/// A 3D vector / point. Plain value type, trivially comparable.
struct Vec3 {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  bool operator==(const Vec3 &) const = default;
};

constexpr Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
constexpr Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
constexpr Vec3 operator-(Vec3 v) { return {-v.x, -v.y, -v.z}; }
constexpr Vec3 operator*(Vec3 v, float s) { return {v.x * s, v.y * s, v.z * s}; }
constexpr Vec3 operator*(float s, Vec3 v) { return v * s; }
constexpr Vec3 operator/(Vec3 v, float s) { return {v.x / s, v.y / s, v.z / s}; }
constexpr Vec3 &operator+=(Vec3 &a, Vec3 b) { return a = a + b; }
constexpr Vec3 &operator-=(Vec3 &a, Vec3 b) { return a = a - b; }
constexpr Vec3 &operator*=(Vec3 &a, float s) { return a = a * s; }

/// Dot product `a·b`.
constexpr float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
/// Cross product `a×b`.
constexpr Vec3 cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
/// Squared length `v·v` (avoids the sqrt when only comparing magnitudes).
constexpr float lengthSquared(Vec3 v) { return dot(v, v); }
/// Euclidean length `|v|`.
inline float length(Vec3 v) { return std::sqrt(lengthSquared(v)); }

/// Safe normalize: a zero-length vector normalizes to zero rather than NaN.
inline Vec3 normalized(Vec3 v) {
  const float len = length(v);
  return len > 0.0f ? v / len : Vec3{};
}

/// Component-wise absolute value. Used by box collision.
constexpr Vec3 absVec(Vec3 v) { return {std::abs(v.x), std::abs(v.y), std::abs(v.z)}; }
/// Component-wise minimum of two vectors.
constexpr Vec3 minVec(Vec3 a, Vec3 b) {
  return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}
/// Component-wise maximum of two vectors.
constexpr Vec3 maxVec(Vec3 a, Vec3 b) {
  return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}
/// Clamps a scalar to `[lo, hi]`.
constexpr float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// --- Mat3 (column-major: M*v = c0*v.x + c1*v.y + c2*v.z) ----------------------

/// A 3x3 matrix stored as three column vectors; a rotation is `[xAxis yAxis zAxis]`.
struct Mat3 {
  Vec3 c0{1.0f, 0.0f, 0.0f};
  Vec3 c1{0.0f, 1.0f, 0.0f};
  Vec3 c2{0.0f, 0.0f, 1.0f};
  bool operator==(const Mat3 &) const = default;
};

/// Matrix-vector product `M*v`.
constexpr Vec3 operator*(const Mat3 &m, Vec3 v) { return m.c0 * v.x + m.c1 * v.y + m.c2 * v.z; }
/// Matrix-matrix product `a*b`.
constexpr Mat3 operator*(const Mat3 &a, const Mat3 &b) { return {a * b.c0, a * b.c1, a * b.c2}; }
/// Scales every element by `s`.
constexpr Mat3 operator*(const Mat3 &m, float s) { return {m.c0 * s, m.c1 * s, m.c2 * s}; }

/// Transpose: rows of the original become columns (the inverse of a rotation).
constexpr Mat3 transpose(const Mat3 &m) {
  return {{m.c0.x, m.c1.x, m.c2.x}, {m.c0.y, m.c1.y, m.c2.y}, {m.c0.z, m.c1.z, m.c2.z}};
}

/// A diagonal matrix with the given entries on its diagonal.
constexpr Mat3 diagonal(float x, float y, float z) {
  return {{x, 0.0f, 0.0f}, {0.0f, y, 0.0f}, {0.0f, 0.0f, z}};
}

// --- Quaternion (w + xi + yj + zk), unit = orientation -----------------------

/// A quaternion `w + xi + yj + zk`; a unit quaternion represents an orientation.
struct Quat {
  float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
  bool operator==(const Quat &) const = default;
};

/// Hamilton product `a*b` (composes the two rotations).
constexpr Quat operator*(Quat a, Quat b) {
  return {
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

/// Safe normalize to a unit quaternion; a zero quaternion normalizes to zero.
inline Quat normalized(Quat q) {
  const float len = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  return len > 0.0f ? Quat{q.w / len, q.x / len, q.y / len, q.z / len} : Quat{};
}

/// Rotates a vector by a unit quaternion (`v' = q v q*`), via the standard
/// two-cross-product form.
inline Vec3 rotate(Quat q, Vec3 v) {
  const Vec3 u{q.x, q.y, q.z};
  const Vec3 t = 2.0f * cross(u, v);
  return v + q.w * t + cross(u, t);
}

/// A unit quaternion rotating `radians` about `axis` (right-handed).
inline Quat fromAxisAngle(Vec3 axis, float radians) {
  const Vec3 n = normalized(axis);
  const float h = radians * 0.5f;
  const float s = std::sin(h);
  return {std::cos(h), n.x * s, n.y * s, n.z * s};
}

/// Integrates an orientation by a world-space angular velocity over `dt`:
/// `dq/dt = 0.5 * (0, ω) * q`. Re-normalized to stay a unit quaternion.
inline Quat integrateOrientation(Quat q, Vec3 omega, float dt) {
  const Quat dq = Quat{0.0f, omega.x, omega.y, omega.z} * q;
  return normalized(Quat{q.w + dq.w * 0.5f * dt, q.x + dq.x * 0.5f * dt, q.y + dq.y * 0.5f * dt,
                         q.z + dq.z * 0.5f * dt});
}

/// Rotation matrix for a unit quaternion (columns are the rotated basis axes).
inline Mat3 toMat3(Quat q) {
  const float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
  const float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
  const float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
  return {{1.0f - 2.0f * (yy + zz), 2.0f * (xy + wz), 2.0f * (xz - wy)},
          {2.0f * (xy - wz), 1.0f - 2.0f * (xx + zz), 2.0f * (yz + wx)},
          {2.0f * (xz + wy), 2.0f * (yz - wx), 1.0f - 2.0f * (xx + yy)}};
}

} // namespace Physics
