#pragma once

// Minimal column-major 4x4 matrix + vector math for the rendering backends.
// Deliberately a PLAIN header with its own POD types (no dependency on the
// PhysicsEngine module), so the render backends are fully standalone C++ and can
// be compiled as ordinary (non-module) libraries alongside GL / metal-cpp
// headers. Callers convert `Physics::Vec3` -> `Render::Vec3` at the boundary.

#include <array>

namespace Render {

struct Vec3 {
  float x = 0.0f, y = 0.0f, z = 0.0f;
};

struct Color {
  float r = 1.0f, g = 1.0f, b = 1.0f;
};

/// A camera pose + lens. The backend turns this into a view-projection matrix
/// using the clip-space convention appropriate to its API.
struct Camera {
  Vec3 eye{0.0f, 8.0f, 16.0f};
  Vec3 target{0.0f, 2.0f, 0.0f};
  Vec3 up{0.0f, 1.0f, 0.0f};
  float fovYRadians = 1.0472f; // 60 degrees
  float nearZ = 0.1f;
  float farZ = 200.0f;
};

using Mat4 = std::array<float, 16>; // column-major (GL / Metal buffer layout)

Vec3 normalized(Vec3 v);
Vec3 cross(Vec3 a, Vec3 b);
float dot(Vec3 a, Vec3 b);

Mat4 multiply(const Mat4 &a, const Mat4 &b);
Mat4 lookAt(Vec3 eye, Vec3 target, Vec3 up);
Mat4 perspectiveGL(float fovYRadians, float aspect, float nearZ, float farZ);    // z in [-1, 1]
Mat4 perspectiveMetal(float fovYRadians, float aspect, float nearZ, float farZ); // z in [0, 1]

} // namespace Render
