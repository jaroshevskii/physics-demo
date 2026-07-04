// Corridor demo — a closed tube that fills with balls and continuously
// reconfigures: the gravity vector precesses, sloshing the whole mass of balls
// around the tube. Up to ~1400 balls, all drawn in a few instanced draws.
//
// This is the stress demo — it favours the Metal backend (its whole point is one
// instanced draw for the entire ball cloud), but runs on OpenGL too via
// --backend gl.
//
//   Space spawn a burst · P pause · R reset · Esc quit
//   Headless: --frames N [--screenshot p.png] [--record dir].

#include <SDL3/SDL.h>

#include "RenderBackend.hpp"

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;
import SceneView;
import CorridorFeature;

using ComposableArchitecture::RootStore;
namespace CF = CorridorFeature;

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
      Render::createWindow("Physics Demo — Corridor", 1280, 800, backend));
  auto renderer = Render::makeRenderBackend(backend);
  if (!window || !renderer->init(window)) {
    std::println(std::cerr, "renderer init failed ({} backend)", renderer->name());
    return 1;
  }

  RootStore<CF::State, CF::Action> store(CF::initialState(), CF::body);

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
          store.send(CF::SpawnBurst{});
          break;
        case SDL_SCANCODE_R:
          store.send(CF::ResetTapped{});
          break;
        case SDL_SCANCODE_P:
          store.send(CF::TogglePause{});
          break;
        default:
          break;
        }
      }
    }
    if (frame % 6 == 0) { // steadily fill the corridor
      store.send(CF::SpawnBurst{});
    }
    store.send(CF::Ticked{});
    store.pump();

    int w = 0, h = 0;
    SDL_GetWindowSizeInPixels(window, &w, &h);
    // Slow orbit around the tube, looking at its centre.
    const double t = static_cast<double>(SDL_GetTicks()) / 1000.0 - start;
    const float angle = static_cast<float>(t) * 0.25f;
    Render::Camera cam;
    cam.eye = {std::sin(angle) * 26.0f, 9.0f, std::cos(angle) * 26.0f};
    cam.target = {0.0f, 0.0f, 0.0f};
    renderer->beginFrame(w, h, cam, {0.05f, 0.05f, 0.08f});
    SceneView::drawDynamicBodies(*renderer, store.state().sim); // tube walls stay invisible
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
