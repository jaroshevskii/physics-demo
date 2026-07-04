#pragma once

#include "RenderBackend.hpp"

namespace Render {

/// Metal (metal-cpp) implementation of `RenderBackend` — the reference backend.
/// `drawSphere` / `drawBox` record per-instance transforms and colours; the
/// frame is flushed as two GPU-**instanced** draws (all spheres in one call, all
/// boxes in one call), which is how Metal is meant to be driven. metal-cpp and
/// SDL Metal headers are confined to MetalRenderer.cpp.
class MetalRenderer final : public RenderBackend {
public:
  MetalRenderer();
  ~MetalRenderer() override;

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
  const char *name() const override { return "Metal"; }

private:
  struct Impl;
  Impl *impl_ = nullptr;
};

} // namespace Render
