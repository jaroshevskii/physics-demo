// First-person shooting-gallery demo. Fire heavy fast bullets that knock down a
// wall of boxes.
//   Mouse look · W/A/S/D move · Left-click or Space shoot · P pause · R reset · Esc quit
//   Backend: --backend metal|gl (Metal is the default on macOS).
//   Headless: --frames N [--screenshot p.png] [--record dir] — auto-fires, panning.

#include <SDL3/SDL.h>

#include "RenderBackend.hpp"

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;
import SceneView;
import ShooterFeature;

using ComposableArchitecture::RootStore;
namespace SF = ShooterFeature;

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
  const bool headless = maxFrames > 0;

  if (!SDL_Init(SDL_INIT_VIDEO)) {
    std::println(std::cerr, "SDL_Init failed: {}", SDL_GetError());
    return 1;
  }
  SDL_Window *window = static_cast<SDL_Window *>(
      Render::createWindow("Physics Demo — Shooting Gallery", 1280, 800, backend));
  auto renderer = Render::makeRenderBackend(backend);
  if (!window || !renderer->init(window)) {
    std::println(std::cerr, "renderer init failed ({} backend)", renderer->name());
    return 1;
  }
  if (!headless) {
    SDL_SetWindowRelativeMouseMode(window, true); // capture the mouse for FPS look
  }

  RootStore<SF::State, SF::Action> store(SF::initialState(), SF::body);

  double lastTicks = static_cast<double>(SDL_GetTicks()) / 1000.0;
  bool running = true;
  int frame = 0;
  while (running) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_EVENT_QUIT) {
        running = false;
      } else if (e.type == SDL_EVENT_MOUSE_MOTION && !headless) {
        store.send(SF::LookDelta{e.motion.xrel * SF::Config::lookSpeed,
                                 -e.motion.yrel * SF::Config::lookSpeed});
      } else if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN && e.button.button == SDL_BUTTON_LEFT) {
        store.send(SF::Shoot{});
      } else if (e.type == SDL_EVENT_KEY_DOWN && !e.key.repeat) {
        switch (e.key.scancode) {
        case SDL_SCANCODE_ESCAPE:
          running = false;
          break;
        case SDL_SCANCODE_SPACE:
          store.send(SF::Shoot{});
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

    const double now = static_cast<double>(SDL_GetTicks()) / 1000.0;
    const float dt = static_cast<float>(now - lastTicks);
    lastTicks = now;

    if (headless) {
      // Auto-fire while gently panning across the wall, for the recording.
      store.send(SF::LookDelta{0.006f * std::sin(static_cast<float>(frame) * 0.08f), 0.0f});
      if (frame % 10 == 0) {
        store.send(SF::Shoot{});
      }
    } else {
      const bool *keys = SDL_GetKeyboardState(nullptr);
      const float move = SF::Config::moveSpeed * dt;
      const float forward =
          (keys[SDL_SCANCODE_W] ? move : 0.0f) - (keys[SDL_SCANCODE_S] ? move : 0.0f);
      const float right =
          (keys[SDL_SCANCODE_D] ? move : 0.0f) - (keys[SDL_SCANCODE_A] ? move : 0.0f);
      if (forward != 0.0f || right != 0.0f) {
        store.send(SF::MoveLocal{forward, right});
      }
    }

    store.send(SF::Ticked{});
    store.pump();

    int w = 0, h = 0;
    SDL_GetWindowSizeInPixels(window, &w, &h);
    const SF::State &state = store.state();
    Render::Camera cam;
    cam.eye = SceneView::toRender(state.cameraPosition);
    cam.target =
        SceneView::toRender(state.cameraPosition + SF::forwardDirection(state.yaw, state.pitch));
    renderer->beginFrame(w, h, cam, {0.07f, 0.08f, 0.11f});
    SceneView::drawWorld(*renderer, state.sim);
    if (!record.empty()) {
      char path[256];
      std::snprintf(path, sizeof(path), "%s/%04d.png", record.c_str(), frame);
      renderer->captureFrame(path);
    } else if (!shot.empty() && frame == maxFrames - 1) {
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
