// OpenGL backend. GL + SDL headers are confined to this translation unit's
// global module fragment; the rest of the project only sees GLRenderer.hpp.
//
// Function loading goes through GLAD (vendored, gl core 4.1) on EVERY platform,
// resolved from SDL's GL context via SDL_GL_GetProcAddress. Using one loader
// everywhere (including macOS) means the exact code path that ships on
// Linux/Windows is the one exercised on macOS — no per-platform GL divergence.
#define GL_SILENCE_DEPRECATION
#include <glad/gl.h>

#include <SDL3/SDL.h>

#include "GLRenderer.hpp"
#include "RenderBackend.hpp"
#include "RenderMath.hpp"

#include <cmath>
#include <cstdio>
#include <numbers>
#include <vector>

namespace Render {
namespace {

const char *kMeshVertexShader = R"(#version 410 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
uniform mat4 uViewProj;
uniform mat4 uModel;
uniform mat3 uNormal;
out vec3 vNormal;
out vec3 vWorld;
void main() {
  vec4 world = uModel * vec4(aPos, 1.0);
  vWorld = world.xyz;
  vNormal = uNormal * aNormal;
  gl_Position = uViewProj * world;
}
)";

const char *kMeshFragmentShader = R"(#version 410 core
in vec3 vNormal;
in vec3 vWorld;
uniform vec3 uColor;
uniform vec3 uLightDir;
uniform vec3 uEye;
out vec4 fragColor;
void main() {
  vec3 N = normalize(vNormal);
  vec3 L = normalize(-uLightDir);
  float diff = max(dot(N, L), 0.0);
  vec3 V = normalize(uEye - vWorld);
  vec3 H = normalize(L + V);
  float spec = pow(max(dot(N, H), 0.0), 32.0) * 0.25;
  fragColor = vec4(uColor * (0.28 + 0.72 * diff) + vec3(spec), 1.0);
}
)";

const char *kPointVertexShader = R"(#version 410 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aColor;
uniform mat4 uViewProj;
uniform float uPointSize;
out vec3 vColor;
void main() {
  vColor = aColor;
  gl_Position = uViewProj * vec4(aPos, 1.0);
  gl_PointSize = uPointSize;
}
)";

const char *kPointFragmentShader = R"(#version 410 core
in vec3 vColor;
out vec4 fragColor;
void main() {
  vec2 d = gl_PointCoord - vec2(0.5);
  if (dot(d, d) > 0.25) discard;
  fragColor = vec4(vColor, 1.0);
}
)";

unsigned compile(unsigned type, const char *source) {
  const unsigned shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);
  int ok = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char log[1024];
    glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
    std::fprintf(stderr, "GL shader error: %s\n", log);
  }
  return shader;
}

unsigned linkProgram(const char *vertex, const char *fragment) {
  const unsigned vs = compile(GL_VERTEX_SHADER, vertex);
  const unsigned fs = compile(GL_FRAGMENT_SHADER, fragment);
  const unsigned program = glCreateProgram();
  glAttachShader(program, vs);
  glAttachShader(program, fs);
  glLinkProgram(program);
  glDeleteShader(vs);
  glDeleteShader(fs);
  return program;
}

struct Mesh {
  std::vector<float> vertices; // position(3) + normal(3)
  std::vector<unsigned> indices;
};

Mesh makeCube() {
  Mesh m;
  const Vec3 normals[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  for (const Vec3 n : normals) {
    const Vec3 a = std::abs(n.x) > 0.5f ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
    const Vec3 b = cross(n, a);
    const unsigned base = static_cast<unsigned>(m.vertices.size() / 6);
    for (int i = 0; i < 4; ++i) {
      const float su = (i == 0 || i == 3) ? -1.0f : 1.0f;
      const float sv = (i < 2) ? -1.0f : 1.0f;
      const Vec3 p{n.x + a.x * su + b.x * sv, n.y + a.y * su + b.y * sv, n.z + a.z * su + b.z * sv};
      m.vertices.insert(m.vertices.end(), {p.x, p.y, p.z, n.x, n.y, n.z});
    }
    for (unsigned idx : {0u, 1u, 2u, 0u, 2u, 3u}) {
      m.indices.push_back(base + idx);
    }
  }
  return m;
}

Mesh makeSphere(int stacks, int sectors) {
  Mesh m;
  const float pi = std::numbers::pi_v<float>;
  for (int i = 0; i <= stacks; ++i) {
    const float phi = pi * static_cast<float>(i) / stacks;
    for (int j = 0; j <= sectors; ++j) {
      const float theta = 2.0f * pi * static_cast<float>(j) / sectors;
      const Vec3 p{std::sin(phi) * std::cos(theta), std::cos(phi), std::sin(phi) * std::sin(theta)};
      m.vertices.insert(m.vertices.end(), {p.x, p.y, p.z, p.x, p.y, p.z});
    }
  }
  const int row = sectors + 1;
  for (int i = 0; i < stacks; ++i) {
    for (int j = 0; j < sectors; ++j) {
      const unsigned a = static_cast<unsigned>(i * row + j);
      const unsigned b = static_cast<unsigned>((i + 1) * row + j);
      for (unsigned idx : {a, b, a + 1, a + 1, b, b + 1}) {
        m.indices.push_back(idx);
      }
    }
  }
  return m;
}

void uploadMesh(const Mesh &mesh, unsigned &vao, unsigned &vbo, unsigned &ebo) {
  glGenVertexArrays(1, &vao);
  glGenBuffers(1, &vbo);
  glGenBuffers(1, &ebo);
  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, static_cast<long>(mesh.vertices.size() * sizeof(float)),
               mesh.vertices.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<long>(mesh.indices.size() * sizeof(unsigned)),
               mesh.indices.data(), GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void *>(0));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        reinterpret_cast<void *>(3 * sizeof(float)));
  glEnableVertexAttribArray(1);
  glBindVertexArray(0);
}

} // namespace

struct GLRenderer::Impl {
  SDL_Window *window = nullptr;
  SDL_GLContext context = nullptr;
  unsigned meshProgram = 0, pointProgram = 0;
  unsigned cubeVao = 0, cubeVbo = 0, cubeEbo = 0, sphereVao = 0, sphereVbo = 0, sphereEbo = 0;
  unsigned pointVao = 0, pointPosVbo = 0, pointColVbo = 0;
  int cubeIndexCount = 0, sphereIndexCount = 0;
  int uViewProj = -1, uModel = -1, uNormal = -1, uColor = -1, uLightDir = -1, uEye = -1;
  int uPointViewProj = -1, uPointSize = -1;
  float viewProjection[16] = {};
  Vec3 eye;
  int width = 0, height = 0;

  void drawMesh(unsigned vao, int indexCount, const float model[16], const float normal[9],
                Color color) {
    glUseProgram(meshProgram);
    glUniformMatrix4fv(uViewProj, 1, GL_FALSE, viewProjection);
    glUniformMatrix4fv(uModel, 1, GL_FALSE, model);
    glUniformMatrix3fv(uNormal, 1, GL_FALSE, normal);
    glUniform3f(uColor, color.r, color.g, color.b);
    glUniform3f(uLightDir, -0.4f, -1.0f, -0.5f);
    glUniform3f(uEye, eye.x, eye.y, eye.z);
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
  }
};

GLRenderer::GLRenderer() : impl_(new Impl()) {}
GLRenderer::~GLRenderer() { shutdown(); }

bool GLRenderer::init(void *sdlWindow) {
  impl_->window = static_cast<SDL_Window *>(sdlWindow);
  impl_->context = SDL_GL_CreateContext(impl_->window);
  if (!impl_->context) {
    std::fprintf(stderr, "SDL_GL_CreateContext failed: %s\n", SDL_GetError());
    return false;
  }
  SDL_GL_MakeCurrent(impl_->window, impl_->context);

  // Load the GL 4.1 core entry points for this context (all platforms).
  if (!gladLoadGL(reinterpret_cast<GLADloadfunc>(SDL_GL_GetProcAddress))) {
    std::fprintf(stderr, "gladLoadGL failed: could not load OpenGL 4.1 functions\n");
    return false;
  }

  SDL_GL_SetSwapInterval(1);

  impl_->meshProgram = linkProgram(kMeshVertexShader, kMeshFragmentShader);
  impl_->uViewProj = glGetUniformLocation(impl_->meshProgram, "uViewProj");
  impl_->uModel = glGetUniformLocation(impl_->meshProgram, "uModel");
  impl_->uNormal = glGetUniformLocation(impl_->meshProgram, "uNormal");
  impl_->uColor = glGetUniformLocation(impl_->meshProgram, "uColor");
  impl_->uLightDir = glGetUniformLocation(impl_->meshProgram, "uLightDir");
  impl_->uEye = glGetUniformLocation(impl_->meshProgram, "uEye");

  impl_->pointProgram = linkProgram(kPointVertexShader, kPointFragmentShader);
  impl_->uPointViewProj = glGetUniformLocation(impl_->pointProgram, "uViewProj");
  impl_->uPointSize = glGetUniformLocation(impl_->pointProgram, "uPointSize");

  const Mesh cube = makeCube();
  impl_->cubeIndexCount = static_cast<int>(cube.indices.size());
  uploadMesh(cube, impl_->cubeVao, impl_->cubeVbo, impl_->cubeEbo);
  const Mesh sphere = makeSphere(16, 24);
  impl_->sphereIndexCount = static_cast<int>(sphere.indices.size());
  uploadMesh(sphere, impl_->sphereVao, impl_->sphereVbo, impl_->sphereEbo);

  glGenVertexArrays(1, &impl_->pointVao);
  glGenBuffers(1, &impl_->pointPosVbo);
  glGenBuffers(1, &impl_->pointColVbo);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_PROGRAM_POINT_SIZE);
  return true;
}

void GLRenderer::beginFrame(int pixelWidth, int pixelHeight, const Camera &camera, Color bg) {
  impl_->width = pixelWidth;
  impl_->height = pixelHeight;
  glViewport(0, 0, pixelWidth, pixelHeight);
  glClearColor(bg.r, bg.g, bg.b, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  const float aspect = pixelHeight > 0 ? static_cast<float>(pixelWidth) / pixelHeight : 1.0f;
  const Mat4 vp = multiply(perspectiveGL(camera.fovYRadians, aspect, camera.nearZ, camera.farZ),
                           lookAt(camera.eye, camera.target, camera.up));
  for (int i = 0; i < 16; ++i) {
    impl_->viewProjection[i] = vp[i];
  }
  impl_->eye = camera.eye;
}

void GLRenderer::drawSphere(Vec3 center, float radius, Color color) {
  const float model[16] = {radius, 0, 0,      0, 0,        radius,   0,        0,
                           0,      0, radius, 0, center.x, center.y, center.z, 1};
  const float normal[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  impl_->drawMesh(impl_->sphereVao, impl_->sphereIndexCount, model, normal, color);
}

void GLRenderer::drawBox(Vec3 center, const float r[9], Vec3 h, Color color) {
  const float model[16] = {r[0] * h.x, r[1] * h.x, r[2] * h.x, 0,          r[3] * h.y, r[4] * h.y,
                           r[5] * h.y, 0,          r[6] * h.z, r[7] * h.z, r[8] * h.z, 0,
                           center.x,   center.y,   center.z,   1};
  impl_->drawMesh(impl_->cubeVao, impl_->cubeIndexCount, model, r, color);
}

void GLRenderer::drawPoints(const Vec3 *positions, const Color *colors, unsigned count,
                            float pointSize) {
  if (count == 0) {
    return;
  }
  glUseProgram(impl_->pointProgram);
  glUniformMatrix4fv(impl_->uPointViewProj, 1, GL_FALSE, impl_->viewProjection);
  glUniform1f(impl_->uPointSize, pointSize);
  glBindVertexArray(impl_->pointVao);
  glBindBuffer(GL_ARRAY_BUFFER, impl_->pointPosVbo);
  glBufferData(GL_ARRAY_BUFFER, static_cast<long>(count * sizeof(Vec3)), positions,
               GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), reinterpret_cast<void *>(0));
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, impl_->pointColVbo);
  glBufferData(GL_ARRAY_BUFFER, static_cast<long>(count * sizeof(Color)), colors, GL_DYNAMIC_DRAW);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Color), reinterpret_cast<void *>(0));
  glEnableVertexAttribArray(1);
  glDrawArrays(GL_POINTS, 0, static_cast<int>(count));
  glBindVertexArray(0);
}

void GLRenderer::endFrame() { SDL_GL_SwapWindow(impl_->window); }

bool GLRenderer::captureFrame(const char *pngPath) {
  const int w = impl_->width, h = impl_->height;
  if (w <= 0 || h <= 0) {
    return false;
  }
  std::vector<unsigned char> pixels(static_cast<std::size_t>(w) * h * 4);
  glFinish();
  glReadBuffer(GL_BACK);
  glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
  // GL is bottom-up; flip to top-down for the PNG.
  std::vector<unsigned char> flipped(pixels.size());
  for (int y = 0; y < h; ++y) {
    std::copy_n(pixels.data() + static_cast<std::size_t>(h - 1 - y) * w * 4, w * 4,
                flipped.data() + static_cast<std::size_t>(y) * w * 4);
  }
  const bool ok = writePng(pngPath, flipped.data(), w, h);
  SDL_GL_SwapWindow(impl_->window);
  return ok;
}

void GLRenderer::shutdown() {
  if (!impl_) {
    return;
  }
  if (impl_->context) {
    glDeleteProgram(impl_->meshProgram);
    glDeleteProgram(impl_->pointProgram);
    SDL_GL_DestroyContext(impl_->context);
  }
  delete impl_;
  impl_ = nullptr;
}

} // namespace Render
