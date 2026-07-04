// Demo A entry point — "Cubes". Boxes fall into a walled arena, collide, topple
// and settle. This app owns the SDL window and the frame loop; the simulation
// itself lives in `SandboxFeature` (a Composable-Architecture store). Each frame
// it translates input to actions, `send`s them, advances via `Ticked`, then
// draws the render-interpolated world through a `Render::RenderBackend`.
//
//   Space  spawn a tumbling cube      P  pause / resume
//   R      reset the scene            Esc / window close  quit
//
// The backend is chosen with `--backend metal|gl` (Metal is the default on
// macOS). Run headless for verification with `--frames N`, optionally writing a
// PNG of the last frame with `--screenshot p.png` or every frame with
// `--record dir`.

#include <SDL3/SDL.h>

#include "RenderBackend.hpp"

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;
import SceneView;
import SandboxFeature;

using ComposableArchitecture::RootStore;
namespace SF = SandboxFeature;

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
  SDL_Window *window =
      static_cast<SDL_Window *>(Render::createWindow("Physics Demo — Cubes", 1280, 800, backend));
  auto renderer = Render::makeRenderBackend(backend);
  if (!window || !renderer->init(window)) {
    std::println(std::cerr, "renderer init failed ({} backend)", renderer->name());
    return 1;
  }

  RootStore<SF::State, SF::Action> store(SF::initialState(), SF::body);

  std::mt19937 rng(1234);
  auto randf = [&rng](float lo, float hi) {
    return lo + (hi - lo) * std::uniform_real_distribution<float>(0.0f, 1.0f)(rng);
  };
  auto spawnCube = [&] {
    store.send(
        SF::SpawnBox{.position = {randf(-3.0f, 3.0f), 8.0f, randf(-3.0f, 3.0f)},
                     .halfExtents = {randf(0.4f, 0.8f), randf(0.4f, 0.8f), randf(0.4f, 0.8f)},
                     .density = randf(0.5f, 2.0f),
                     .velocity = {randf(-2.0f, 2.0f), 0.0f, randf(-2.0f, 2.0f)},
                     .orientation = Physics::fromAxisAngle(
                         {randf(-1, 1), randf(-1, 1), randf(-1, 1)}, randf(0.0f, 3.14f))});
  };

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
          spawnCube();
          break;
        case SDL_SCANCODE_R:
          store.send(SF::ResetTapped{});
          break;
        case SDL_SCANCODE_P:
          store.send(SF::TogglePause{});
          break;
        default:
          break;
        }
      }
    }

    store.send(SF::Ticked{});
    store.pump();

    int w = 0, h = 0;
    SDL_GetWindowSizeInPixels(window, &w, &h);
    const double t = static_cast<double>(SDL_GetTicks()) / 1000.0 - start;
    renderer->beginFrame(w, h, SceneView::orbitCamera(t), {0.07f, 0.08f, 0.10f});
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
