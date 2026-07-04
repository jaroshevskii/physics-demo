#pragma once

#include "RenderBackend.hpp"

namespace Render {

/// OpenGL (core 4.1) implementation of `RenderBackend`. Draws a shared cube and
/// UV-sphere mesh per object with a single directional-light shader; issues the
/// recorded draws immediately. GL/SDL headers are confined to GLRenderer.cpp.
class GLRenderer final : public RenderBackend {
public:
  GLRenderer();
  ~GLRenderer() override;

  bool init(void *sdlWindow) override;
  void beginFrame(int pixelWidth, int pixelHeight, const Camera &camera, Color background) override;
  void drawSphere(Vec3 center, float radius, Color color) override;
  void drawBox(Vec3 center, const float rotationColumnMajor[9], Vec3 halfExtents,
               Color color) override;
  void drawPoints(const Vec3 *positions, const Color *colors, unsigned count,
                  float pointSize) override;
  void endFrame() override;
  bool captureFrame(const char *pngPath) override;
  void shutdown() override;
  const char *name() const override { return "OpenGL"; }

private:
  struct Impl;
  Impl *impl_ = nullptr;
};

} // namespace Render
