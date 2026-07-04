// Explosions demo.  Space detonate · R reset · P pause · Esc quit
// Backend: --backend metal|gl (Metal is the default on macOS).
// Headless: --frames N [--screenshot p.png] [--record dir] — auto-detonates once.

#include <SDL3/SDL.h>

#include "RenderBackend.hpp"

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;
import SceneView;
import ExplosionFeature;

using ComposableArchitecture::RootStore;
namespace EF = ExplosionFeature;

int main(int argc, char **argv) {
  int maxFrames = 0;
  std::string shot, record;
  Render::Backend backend = Render::defaultBackend();
  for (int i = 1; i < argc; ++i) {
    const std::string_view a(argv[i]);
    if (a == "--frames" && i + 1 < argc) {
      maxFrames = std::atoi(argv[++i]);
    } else if (a == "--screenshot" && i + 1 < argc) {
      shot = argv[++i];
    } else if (a == "--record" && i + 1 < argc) {
      record = argv[++i];
    } else if (a == "--backend" && i + 1 < argc) {
      backend = Render::parseBackend(argv[++i], backend);
    }
  }

  if (!SDL_Init(SDL_INIT_VIDEO)) {
    std::println(std::cerr, "SDL_Init failed: {}", SDL_GetError());
    return 1;
  }
  SDL_Window *window = static_cast<SDL_Window *>(
      Render::createWindow("Physics Demo — Explosions", 1280, 800, backend));
  auto renderer = Render::makeRenderBackend(backend);
  if (!window || !renderer->init(window)) {
    std::println(std::cerr, "renderer init failed ({} backend)", renderer->name());
    return 1;
  }

  RootStore<EF::State, EF::Action> store(EF::initialState(), EF::body);

  const double start = static_cast<double>(SDL_GetTicks()) / 1000.0;
  bool running = true;
  int frame = 0;
  while (running) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_EVENT_QUIT) {
        running = false;
      } else if (e.type == SDL_EVENT_KEY_DOWN && !e.key.repeat) {
        switch (e.key.scancode) {
        case SDL_SCANCODE_ESCAPE:
          running = false;
          break;
        case SDL_SCANCODE_SPACE:
          store.send(EF::Detonate{});
          break;
        case SDL_SCANCODE_R:
          store.send(EF::ResetTapped{});
          break;
        case SDL_SCANCODE_P:
          store.send(EF::TogglePause{});
          break;
        default:
          break;
        }
      }
    }
    if (frame == 45) { // auto-detonate so the run (and screenshot) shows debris
      store.send(EF::Detonate{});
    }
    store.send(EF::Ticked{});
    store.pump();

    int w = 0, h = 0;
    SDL_GetWindowSizeInPixels(window, &w, &h);
    const double t = static_cast<double>(SDL_GetTicks()) / 1000.0 - start;
    renderer->beginFrame(w, h, SceneView::orbitCamera(t, 8.0f, 20.0f), {0.06f, 0.06f, 0.09f});
    SceneView::drawWorld(*renderer, store.state().sim);
    if (!record.empty()) {
      char path[256];
      std::snprintf(path, sizeof(path), "%s/%04d.png", record.c_str(), frame);
      renderer->captureFrame(path);
    } else if (!shot.empty() && maxFrames > 0 && frame == maxFrames - 1) {
      renderer->captureFrame(shot.c_str());
    } else {
      renderer->endFrame();
    }

    ++frame;
    if (maxFrames > 0 && frame >= maxFrames) {
      running = false;
    }
  }

  renderer->shutdown();
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
