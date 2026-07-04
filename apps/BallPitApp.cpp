// Demo B entry point — "Ball Pit". Same host/loop shape as the cubes demo, but
// it rains many spheres of differing radius (hence differing mass) and
// restitution into an open box, where they pile and bounce off one another.
//
//   Space  toggle the rain      B  drop a burst of balls
//   P      pause / resume       R  reset       Esc / close  quit
//
// Backend: --backend metal|gl (Metal is the default on macOS).
// Headless: --frames N [--screenshot p.png] [--record dir].

#include <SDL3/SDL.h>

#include "RenderBackend.hpp"

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;
import SceneView;
import BallPitFeature;

using ComposableArchitecture::RootStore;
namespace BP = BallPitFeature;

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
      Render::createWindow("Physics Demo — Ball Pit", 1280, 800, backend));
  auto renderer = Render::makeRenderBackend(backend);
  if (!window || !renderer->init(window)) {
    std::println(std::cerr, "renderer init failed ({} backend)", renderer->name());
    return 1;
  }

  RootStore<BP::State, BP::Action> store(BP::initialState(), BP::body);

  std::mt19937 rng(2025);
  auto randf = [&rng](float lo, float hi) {
    return lo + (hi - lo) * std::uniform_real_distribution<float>(0.0f, 1.0f)(rng);
  };
  // Varied radius => varied mass; varied restitution => some bounce more.
  auto dropOne = [&] {
    const float radius = randf(0.25f, 0.6f);
    store.send(BP::SpawnSphere{.position = {randf(-4.0f, 4.0f), 9.0f, randf(-4.0f, 4.0f)},
                               .radius = radius,
                               .density = 1.0f,
                               .restitution = randf(0.2f, 0.85f),
                               .velocity = {0.0f, 0.0f, 0.0f}});
  };

  bool raining = true;
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
          raining = !raining;
          break;
        case SDL_SCANCODE_B:
          for (int k = 0; k < 24; ++k) {
            dropOne();
          }
          break;
        case SDL_SCANCODE_R:
          store.send(BP::ResetTapped{});
          break;
        case SDL_SCANCODE_P:
          store.send(BP::TogglePause{});
          break;
        default:
          break;
        }
      }
    }

    if (raining && frame % 4 == 0) { // a steady drizzle
      dropOne();
      dropOne();
    }

    store.send(BP::Ticked{});
    store.pump();

    int w = 0, h = 0;
    SDL_GetWindowSizeInPixels(window, &w, &h);
    const double t = static_cast<double>(SDL_GetTicks()) / 1000.0 - start;
    renderer->beginFrame(w, h, SceneView::orbitCamera(t, 11.0f, 20.0f), {0.05f, 0.06f, 0.09f});
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
