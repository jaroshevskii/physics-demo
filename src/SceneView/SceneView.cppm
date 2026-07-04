module;

#include "RenderBackend.hpp"

export module SceneView;

import std;
import PhysicsEngine;
import Simulation;

// Turns a simulation `World` into `RenderBackend` draw calls — the state→pixels
// half of a demo (input→actions lives in each app). It reads render-interpolated
// poses from `Sim::State` so motion is smooth regardless of how many fixed steps
// ran this frame, and it speaks only in `Render` values, so it works unchanged on
// either backend (OpenGL or Metal). The physics `Vec3`/`Mat3` are converted to
// the renderer's plain types at this boundary.
export namespace SceneView {

/// A pleasant palette; dynamic bodies are coloured by id so neighbours differ.
inline Render::Color paletteFor(Physics::Entity index) {
  static const Render::Color colors[] = {
      {0.91f, 0.30f, 0.24f}, {0.20f, 0.60f, 0.86f}, {0.18f, 0.80f, 0.44f}, {0.95f, 0.77f, 0.06f},
      {0.61f, 0.35f, 0.71f}, {0.90f, 0.49f, 0.13f}, {0.10f, 0.74f, 0.61f}, {0.93f, 0.51f, 0.93f}};
  return colors[index % (sizeof(colors) / sizeof(colors[0]))];
}

/// A slowly-orbiting camera framing the arena from above and in front.
inline Render::Camera orbitCamera(double timeSeconds, float height = 9.0f, float radius = 17.0f) {
  const float angle = static_cast<float>(timeSeconds) * 0.15f;
  return Render::Camera{.eye = {std::sin(angle) * radius, height, std::cos(angle) * radius},
                        .target = {0.0f, 2.0f, 0.0f},
                        .up = {0.0f, 1.0f, 0.0f}};
}

inline Render::Vec3 toRender(Physics::Vec3 v) { return {v.x, v.y, v.z}; }

/// Flattens a `Physics::Mat3` into the column-major 9-float form the backend's
/// `drawBox` expects.
inline void toColumnMajor(const Physics::Mat3 &rotation, float out[9]) {
  out[0] = rotation.c0.x;
  out[1] = rotation.c0.y;
  out[2] = rotation.c0.z;
  out[3] = rotation.c1.x;
  out[4] = rotation.c1.y;
  out[5] = rotation.c1.z;
  out[6] = rotation.c2.x;
  out[7] = rotation.c2.y;
  out[8] = rotation.c2.z;
}

/// Draws one dynamic body at its render-interpolated pose, coloured by id.
inline void drawBody(Render::RenderBackend &renderer, const Sim::State &sim, Physics::Entity i) {
  const Physics::Collider &collider = sim.world.colliders[i];
  const Render::Vec3 position = toRender(Sim::interpolatedPosition(sim, i));
  const Render::Color color = paletteFor(i);
  if (collider.type == Physics::ShapeType::Sphere) {
    renderer.drawSphere(position, collider.radius, color);
  } else {
    float rotation[9];
    toColumnMajor(Physics::toMat3(Sim::interpolatedOrientation(sim, i)), rotation);
    renderer.drawBox(position, rotation, toRender(collider.halfExtents), color);
  }
}

/// Draws only the dynamic bodies (interpolated), leaving all static geometry
/// invisible. Used by demos like the corridor whose walls fully enclose the
/// scene and would otherwise occlude it.
inline void drawDynamicBodies(Render::RenderBackend &renderer, const Sim::State &sim) {
  for (Physics::Entity i = 0; i < sim.world.count(); ++i) {
    if (sim.world.bodies[i].type != Physics::BodyType::Static) {
      drawBody(renderer, sim, i);
    }
  }
}

/// Draws every dynamic body (interpolated) plus the ground plane. Static tall
/// walls are left invisible so they don't occlude the scene — the floor is
/// distinguished by sitting below `y = 1`.
inline void drawWorld(Render::RenderBackend &renderer, const Sim::State &sim) {
  const Physics::World &world = sim.world;
  for (Physics::Entity i = 0; i < world.count(); ++i) {
    if (world.bodies[i].type == Physics::BodyType::Static) {
      if (world.transforms[i].position.y < 1.0f) { // the floor only
        float rotation[9];
        toColumnMajor(Physics::toMat3(world.transforms[i].orientation), rotation);
        renderer.drawBox(toRender(world.transforms[i].position), rotation,
                         toRender(world.colliders[i].halfExtents), {0.16f, 0.17f, 0.20f});
      }
      continue;
    }
    drawBody(renderer, sim, i);
  }
}

} // namespace SceneView
