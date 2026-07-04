#include "RenderBackend.hpp"

#include "GLRenderer.hpp"
#if PHYSICS_HAS_METAL
#include "MetalRenderer.hpp"
#endif

#include <SDL3/SDL.h>

namespace Render {

bool metalAvailable() {
#if PHYSICS_HAS_METAL
  return true;
#else
  return false;
#endif
}

Backend defaultBackend() { return metalAvailable() ? Backend::Metal : Backend::OpenGL; }

Backend parseBackend(std::string_view name, Backend fallback) {
  if (name == "metal") {
    return Backend::Metal;
  }
  if (name == "gl" || name == "opengl") {
    return Backend::OpenGL;
  }
  return fallback;
}

void *createWindow(const char *title, int width, int height, Backend backend) {
  SDL_WindowFlags flags = SDL_WINDOW_RESIZABLE;
  if (backend == Backend::Metal) {
    flags |= SDL_WINDOW_METAL;
  } else {
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    flags |= SDL_WINDOW_OPENGL;
  }
  return SDL_CreateWindow(title, width, height, flags);
}

std::unique_ptr<RenderBackend> makeRenderBackend(Backend backend) {
#if PHYSICS_HAS_METAL
  if (backend == Backend::Metal) {
    return std::make_unique<MetalRenderer>();
  }
#else
  (void)backend;
#endif
  return std::make_unique<GLRenderer>();
}

} // namespace Render
