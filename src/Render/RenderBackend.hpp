#pragma once

// The abstract rendering interface every demo draws through. Two backends
// implement it: `MetalRenderer` (the reference — GPU instanced rendering via
// metal-cpp) and `GLRenderer` (OpenGL, filling in what it must to match). Both
// are plain (non-module) libraries, so metal-cpp and GL headers stay confined to
// their `.cpp`s and never meet `import std`.
//
// The interface is shaped Metal-first: `drawSphere` / `drawBox` / `drawPoints`
// are recorded during a frame and flushed as a few *instanced* draws — which is
// how Metal wants to work — and the GL backend simply issues them per-object.
// A frame is `beginFrame(...)`, some draws, then `endFrame()` (present) OR
// `captureFrame(path)` (render one frame to a PNG for screenshots / recording).

#include "RenderMath.hpp"

#include <memory>
#include <string_view>

namespace Render {

enum class Backend { OpenGL, Metal };

class RenderBackend {
public:
  virtual ~RenderBackend() = default;

  /// Binds the backend to an SDL window (created with the matching flags by
  /// `createWindow`). Returns false if the GPU/context is unavailable.
  virtual bool init(void *sdlWindow) = 0;

  virtual void beginFrame(int pixelWidth, int pixelHeight, const Camera &camera,
                          Color background) = 0;
  virtual void drawSphere(Vec3 center, float radius, Color color) = 0;
  /// `rotationColumnMajor` is a 3x3 rotation matrix, 3 column vectors (9 floats).
  virtual void drawBox(Vec3 center, const float rotationColumnMajor[9], Vec3 halfExtents,
                       Color color) = 0;
  virtual void drawPoints(const Vec3 *positions, const Color *colors, unsigned count,
                          float pointSize) = 0;

  /// Presents the frame to the window.
  virtual void endFrame() = 0;
  /// Renders this frame to a PNG (offscreen / read-back) instead of presenting —
  /// used for screenshots and GIF recording, and for headless verification.
  virtual bool captureFrame(const char *pngPath) = 0;

  virtual void shutdown() = 0;
  virtual const char *name() const = 0;
};

/// True if a Metal backend was compiled in (macOS with PHYSICS_HAS_METAL).
bool metalAvailable();
/// Metal on macOS when available, otherwise OpenGL.
Backend defaultBackend();
/// Parses "metal" / "gl" / "opengl"; anything else returns `fallback`.
Backend parseBackend(std::string_view name, Backend fallback);

/// Creates an SDL window with the flags/attributes the backend needs (returns an
/// `SDL_Window*` as `void*`; the caller owns it and destroys it with SDL).
void *createWindow(const char *title, int width, int height, Backend backend);

std::unique_ptr<RenderBackend> makeRenderBackend(Backend backend);

/// Writes an RGBA8 image (top-down) as a PNG. Shared by both backends' capture.
bool writePng(const char *path, const unsigned char *rgba, int width, int height);

} // namespace Render
