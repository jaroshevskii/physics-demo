// Raycast car demo.  W/S throttle · A/D steer · R reset · P pause · Esc quit
// Backend: --backend metal|gl (Metal is the default on macOS).
// Headless: --frames N [--screenshot p.png] [--record dir] — auto-drives forward.

#include <SDL3/SDL.h>

#include "RenderBackend.hpp"

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;
import SceneView;
import CarFeature;

using ComposableArchitecture::RootStore;
namespace CF = CarFeature;

namespace {
// Draws the four wheels as spheres at their raycast-resolved positions, using
// the interpolated chassis pose so they track the smoothly-rendered chassis
// (not the stepwise physics state).
void drawWheels(Render::RenderBackend &renderer, const Physics::World &w, Physics::Vec3 chassisPos,
                const Physics::Mat3 &R) {
  using namespace Physics;
  const Entity ch = CF::Config::chassis;
  if (ch >= w.count()) {
    return;
  }
  const Vec3 up = R.c1;
  const float maxSuspensionLength = CF::Config::restLength + CF::Config::wheelRadius;
  for (const Vec3 &m : CF::wheelMounts()) {
    const Vec3 worldMount = chassisPos + R * m;
    const auto hit = raycastNearest(w, worldMount, -up, maxSuspensionLength, ch);
    const float travel = hit ? hit->distance - CF::Config::wheelRadius : CF::Config::restLength;
    renderer.drawSphere(SceneView::toRender(worldMount - up * travel), CF::Config::wheelRadius,
                        {0.1f, 0.1f, 0.12f});
  }
}
} // namespace

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
      Render::createWindow("Physics Demo — Raycast Car", 1280, 800, backend));
  auto renderer = Render::makeRenderBackend(backend);
  if (!window || !renderer->init(window)) {
    std::println(std::cerr, "renderer init failed ({} backend)", renderer->name());
    return 1;
  }

  RootStore<CF::State, CF::Action> store(CF::initialState(), CF::body);

  bool running = true;
  int frame = 0;
  while (running) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_EVENT_QUIT) {
        running = false;
      } else if (e.type == SDL_EVENT_KEY_DOWN && !e.key.repeat) {
        if (e.key.scancode == SDL_SCANCODE_ESCAPE) {
          running = false;
        } else if (e.key.scancode == SDL_SCANCODE_R) {
          store.send(CF::ResetTapped{});
        } else if (e.key.scancode == SDL_SCANCODE_P) {
          store.send(CF::TogglePause{});
        }
      }
    }
    // Continuous controls from held keys (or auto-drive when headless).
    const bool *keys = SDL_GetKeyboardState(nullptr);
    float throttle = (keys[SDL_SCANCODE_W] ? 1.0f : 0.0f) - (keys[SDL_SCANCODE_S] ? 1.0f : 0.0f);
    float steer = (keys[SDL_SCANCODE_D] ? 1.0f : 0.0f) - (keys[SDL_SCANCODE_A] ? 1.0f : 0.0f);
    if (maxFrames > 0) {
      throttle = 1.0f; // drive straight ahead for the headless run/screenshot,
      steer = 0.0f;    // so the car ploughs head-on into the cube stack
    }
    store.send(CF::SetControls{throttle, steer});
    store.send(CF::Ticked{});
    store.pump();

    // Chase camera behind + above the chassis. Use the INTERPOLATED chassis pose
    // (the same one SceneView renders the chassis at), so the camera and the car
    // move in lockstep — otherwise the camera jumps by whole physics steps while
    // the car glides, and everything appears to judder/double.
    const Physics::World &w = store.state().sim.world;
    const Physics::Vec3 cpos = Sim::interpolatedPosition(store.state().sim, CF::Config::chassis);
    const Physics::Mat3 cr =
        Physics::toMat3(Sim::interpolatedOrientation(store.state().sim, CF::Config::chassis));
    const Physics::Vec3 back = cr.c2 * -1.0f; // chassis -forward
    Render::Camera cam;
    cam.eye = SceneView::toRender(cpos + back * 9.0f + Physics::Vec3{0, 5.0f, 0});
    cam.target = SceneView::toRender(cpos);

    int wpx = 0, hpx = 0;
    SDL_GetWindowSizeInPixels(window, &wpx, &hpx);
    renderer->beginFrame(wpx, hpx, cam, {0.07f, 0.08f, 0.11f});
    SceneView::drawWorld(*renderer, store.state().sim);
    drawWheels(*renderer, w, cpos, cr);
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
