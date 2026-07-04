// Fireworks demo — thousands of CPU-parallel particles.
//   Space launch a burst · R reset · Esc quit
// Backend: --backend metal|gl (Metal is the default on macOS).
// Headless: --frames N [--screenshot p.png] [--record dir] — auto-launches.

#include <SDL3/SDL.h>

#include "RenderBackend.hpp"

import std;
import ComposableArchitecture;
import Dependencies;
import PhysicsEngine;
import Simulation;
import SceneView;
import FireworksFeature;

using ComposableArchitecture::RootStore;
namespace FW = Fireworks;

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
      Render::createWindow("Physics Demo — Fireworks", 1280, 800, backend));
  auto renderer = Render::makeRenderBackend(backend);
  if (!window || !renderer->init(window)) {
    std::println(std::cerr, "renderer init failed ({} backend)", renderer->name());
    return 1;
  }

  RootStore<FW::State, FW::Action> store(FW::initialState(), FW::body);

  std::mt19937 rng(4);
  auto randomInRange = [&rng](float lo, float hi) {
    return lo + (hi - lo) * std::uniform_real_distribution<float>(0, 1)(rng);
  };

  // Reused each frame to avoid per-frame allocations for the point cloud.
  std::vector<Render::Vec3> points;
  std::vector<Render::Color> colors;

  const double start = static_cast<double>(SDL_GetTicks()) / 1000.0;
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
        } else if (e.key.scancode == SDL_SCANCODE_SPACE) {
          store.send(
              FW::Launch{{randomInRange(-6, 6), randomInRange(6, 11), randomInRange(-6, 6)}});
        } else if (e.key.scancode == SDL_SCANCODE_R) {
          store.send(FW::ResetTapped{});
        }
      }
    }
    if (frame % 40 == 0) { // a steady stream of bursts
      store.send(FW::Launch{{randomInRange(-6, 6), randomInRange(6, 11), randomInRange(-6, 6)}});
    }
    store.send(FW::Ticked{});
    store.pump();

    // Build the alive-particle point cloud for this frame.
    const FW::ParticleSystem &particles = store.state().particles;
    points.clear();
    colors.clear();
    points.reserve(particles.size());
    colors.reserve(particles.size());
    for (std::size_t i = 0; i < particles.size(); ++i) {
      if (particles.life[i] > 0.0f) {
        points.push_back(
            {particles.positions[i].x, particles.positions[i].y, particles.positions[i].z});
        colors.push_back({particles.colors[i].x, particles.colors[i].y, particles.colors[i].z});
      }
    }

    int w = 0, h = 0;
    SDL_GetWindowSizeInPixels(window, &w, &h);
    const double t = static_cast<double>(SDL_GetTicks()) / 1000.0 - start;
    renderer->beginFrame(w, h, SceneView::orbitCamera(t, 6.0f, 22.0f), {0.02f, 0.02f, 0.05f});
    renderer->drawPoints(points.data(), colors.data(), static_cast<unsigned>(points.size()), 6.0f);
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
