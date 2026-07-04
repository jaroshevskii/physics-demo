// Metal backend (the reference). This is the single metal-cpp translation unit;
// the three macros make it emit metal-cpp's out-of-line implementations once.
// metal-cpp + SDL headers are confined here, so `import std` never meets them.
#define NS_PRIVATE_IMPLEMENTATION
#define CA_PRIVATE_IMPLEMENTATION
#define MTL_PRIVATE_IMPLEMENTATION

#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>

#include <SDL3/SDL.h>

#include "MetalRenderer.hpp"
#include "RenderBackend.hpp"
#include "RenderMath.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <numbers>
#include <vector>

namespace Render {
namespace {

// Instanced mesh shader (Lambert + specular + fresnel rim + depth fog) and a
// point-sprite shader for particle clouds.
const char *kShaderSource = R"METAL(
#include <metal_stdlib>
using namespace metal;

struct VertexIn { packed_float3 position; packed_float3 normal; };
struct Instance { float4x4 model; float4 color; };
struct Camera   { float4x4 viewProj; float4 lightDir; float4 eye; };

struct MeshOut { float4 pos [[position]]; float3 nrm; float3 world; float4 color; };

vertex MeshOut mesh_vertex(uint vid [[vertex_id]], uint iid [[instance_id]],
                           device const VertexIn* verts     [[buffer(0)]],
                           device const Instance* instances [[buffer(1)]],
                           constant Camera&       cam        [[buffer(2)]]) {
    Instance inst = instances[iid];
    float4 world = inst.model * float4(verts[vid].position, 1.0);
    MeshOut o;
    o.pos   = cam.viewProj * world;
    o.world = world.xyz;
    o.nrm   = (inst.model * float4(verts[vid].normal, 0.0)).xyz;
    o.color = inst.color;
    return o;
}

fragment float4 mesh_fragment(MeshOut in [[stage_in]], constant Camera& cam [[buffer(2)]]) {
    float3 N = normalize(in.nrm);
    float3 L = normalize(-cam.lightDir.xyz);
    float3 V = normalize(cam.eye.xyz - in.world);
    float3 H = normalize(L + V);
    float diff = max(dot(N, L), 0.0);
    float spec = pow(max(dot(N, H), 0.0), 40.0) * 0.35;
    float rim  = pow(1.0 - max(dot(N, V), 0.0), 3.0) * 0.4;
    float3 lit = in.color.rgb * (0.2 + 0.8 * diff) + spec + rim * float3(0.4, 0.55, 1.0);
    float fog = clamp(in.pos.z, 0.0, 1.0); fog = fog * fog * 0.55;
    return float4(mix(lit, float3(0.02, 0.02, 0.05), fog), 1.0);
}

struct PointOut { float4 pos [[position]]; float size [[point_size]]; float3 color; };
vertex PointOut point_vertex(uint vid [[vertex_id]],
                             device const packed_float3* positions [[buffer(0)]],
                             device const packed_float3* colors    [[buffer(1)]],
                             constant Camera&            cam        [[buffer(2)]],
                             constant float&             pointSize  [[buffer(3)]]) {
    PointOut o;
    o.pos = cam.viewProj * float4(positions[vid], 1.0);
    o.size = pointSize;
    o.color = colors[vid];
    return o;
}
fragment float4 point_fragment(PointOut in [[stage_in]], float2 uv [[point_coord]]) {
    if (dot(uv - float2(0.5), uv - float2(0.5)) > 0.25) discard_fragment();
    return float4(in.color, 1.0);
}
)METAL";

struct alignas(16) InstanceGPU {
  float model[16];
  float color[4];
};

void makeSphere(int stacks, int sectors, std::vector<float> &vertices,
                std::vector<std::uint32_t> &indices) {
  const float pi = std::numbers::pi_v<float>;
  for (int i = 0; i <= stacks; ++i) {
    const float phi = pi * static_cast<float>(i) / stacks;
    for (int j = 0; j <= sectors; ++j) {
      const float theta = 2.0f * pi * static_cast<float>(j) / sectors;
      const float x = std::sin(phi) * std::cos(theta), y = std::cos(phi),
                  z = std::sin(phi) * std::sin(theta);
      vertices.insert(vertices.end(), {x, y, z, x, y, z});
    }
  }
  const int row = sectors + 1;
  for (int i = 0; i < stacks; ++i) {
    for (int j = 0; j < sectors; ++j) {
      const std::uint32_t a = static_cast<std::uint32_t>(i * row + j);
      const std::uint32_t b = static_cast<std::uint32_t>((i + 1) * row + j);
      for (std::uint32_t idx : {a, b, a + 1, a + 1, b, b + 1}) {
        indices.push_back(idx);
      }
    }
  }
}

void makeCube(std::vector<float> &vertices, std::vector<std::uint32_t> &indices) {
  const float n[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  for (const auto &nn : n) {
    const Vec3 normal{nn[0], nn[1], nn[2]};
    const Vec3 a = std::abs(normal.x) > 0.5f ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
    const Vec3 b = cross(normal, a);
    const std::uint32_t base = static_cast<std::uint32_t>(vertices.size() / 6);
    for (int i = 0; i < 4; ++i) {
      const float su = (i == 0 || i == 3) ? -1.0f : 1.0f, sv = (i < 2) ? -1.0f : 1.0f;
      vertices.insert(vertices.end(),
                      {normal.x + a.x * su + b.x * sv, normal.y + a.y * su + b.y * sv,
                       normal.z + a.z * su + b.z * sv, normal.x, normal.y, normal.z});
    }
    for (std::uint32_t idx : {0u, 1u, 2u, 0u, 2u, 3u}) {
      indices.push_back(base + idx);
    }
  }
}

NS::String *nsstr(const char *s) { return NS::String::string(s, NS::UTF8StringEncoding); }

} // namespace

struct MetalRenderer::Impl {
  SDL_Window *window = nullptr;
  SDL_MetalView view = nullptr;
  CA::MetalLayer *layer = nullptr;
  MTL::Device *device = nullptr;
  MTL::CommandQueue *queue = nullptr;
  MTL::RenderPipelineState *meshPipeline = nullptr;
  MTL::RenderPipelineState *pointPipeline = nullptr;
  MTL::DepthStencilState *depthState = nullptr;
  MTL::Texture *depthTexture = nullptr;
  MTL::Buffer *cubeVertices = nullptr, *cubeIndices = nullptr;
  MTL::Buffer *sphereVertices = nullptr, *sphereIndices = nullptr;
  MTL::Buffer *cubeInstances = nullptr, *sphereInstances = nullptr;
  MTL::Buffer *pointPositions = nullptr, *pointColors = nullptr;
  MTL::Buffer *cameraBuffer = nullptr, *pointSizeBuffer = nullptr;
  int cubeIndexCount = 0, sphereIndexCount = 0;
  unsigned instanceCapacity = 8192, pointCapacity = 16384;
  int depthWidth = 0, depthHeight = 0, width = 0, height = 0;
  float clearColor[3] = {0, 0, 0};
  float pointSize = 6.0f;

  std::vector<InstanceGPU> cubeBatch, sphereBatch;
  std::vector<Vec3> pointPositionBatch;
  std::vector<Color> pointColorBatch;

  MTL::Texture *makeDepth(int w, int h) {
    if (depthTexture && depthWidth == w && depthHeight == h) {
      return depthTexture;
    }
    if (depthTexture) {
      depthTexture->release();
    }
    MTL::TextureDescriptor *td = MTL::TextureDescriptor::texture2DDescriptor(
        MTL::PixelFormatDepth32Float, static_cast<NS::UInteger>(w), static_cast<NS::UInteger>(h),
        false); // autoreleased factory object — do not release
    td->setUsage(MTL::TextureUsageRenderTarget);
    td->setStorageMode(MTL::StorageModePrivate);
    depthTexture = device->newTexture(td);
    depthWidth = w;
    depthHeight = h;
    return depthTexture;
  }

  void encode(MTL::Texture *colorTarget) {
    // Upload the frame's instance + point batches (Shared storage => CPU-visible).
    const unsigned cubes = std::min<unsigned>(cubeBatch.size(), instanceCapacity);
    const unsigned spheres = std::min<unsigned>(sphereBatch.size(), instanceCapacity);
    const unsigned points = std::min<unsigned>(pointPositionBatch.size(), pointCapacity);
    if (cubes) {
      std::memcpy(cubeInstances->contents(), cubeBatch.data(), cubes * sizeof(InstanceGPU));
    }
    if (spheres) {
      std::memcpy(sphereInstances->contents(), sphereBatch.data(), spheres * sizeof(InstanceGPU));
    }
    if (points) {
      std::memcpy(pointPositions->contents(), pointPositionBatch.data(), points * sizeof(Vec3));
      std::memcpy(pointColors->contents(), pointColorBatch.data(), points * sizeof(Color));
    }

    MTL::RenderPassDescriptor *pass = MTL::RenderPassDescriptor::alloc()->init();
    auto *color = pass->colorAttachments()->object(0);
    color->setTexture(colorTarget);
    color->setLoadAction(MTL::LoadActionClear);
    color->setStoreAction(MTL::StoreActionStore);
    color->setClearColor(MTL::ClearColor(clearColor[0], clearColor[1], clearColor[2], 1.0));
    auto *depth = pass->depthAttachment();
    depth->setTexture(makeDepth(width, height));
    depth->setLoadAction(MTL::LoadActionClear);
    depth->setStoreAction(MTL::StoreActionDontCare);
    depth->setClearDepth(1.0);

    MTL::CommandBuffer *commandBuffer = queue->commandBuffer();
    MTL::RenderCommandEncoder *enc = commandBuffer->renderCommandEncoder(pass);
    enc->setDepthStencilState(depthState);
    enc->setCullMode(MTL::CullModeNone);
    enc->setRenderPipelineState(meshPipeline);
    enc->setVertexBuffer(cameraBuffer, 0, 2);
    enc->setFragmentBuffer(cameraBuffer, 0, 2);
    if (cubes) {
      enc->setVertexBuffer(cubeVertices, 0, 0);
      enc->setVertexBuffer(cubeInstances, 0, 1);
      enc->drawIndexedPrimitives(MTL::PrimitiveTypeTriangle,
                                 static_cast<NS::UInteger>(cubeIndexCount), MTL::IndexTypeUInt32,
                                 cubeIndices, NS::UInteger(0), NS::UInteger(cubes));
    }
    if (spheres) {
      enc->setVertexBuffer(sphereVertices, 0, 0);
      enc->setVertexBuffer(sphereInstances, 0, 1);
      enc->drawIndexedPrimitives(MTL::PrimitiveTypeTriangle,
                                 static_cast<NS::UInteger>(sphereIndexCount), MTL::IndexTypeUInt32,
                                 sphereIndices, NS::UInteger(0), NS::UInteger(spheres));
    }
    if (points) {
      *static_cast<float *>(pointSizeBuffer->contents()) = pointSize;
      enc->setRenderPipelineState(pointPipeline);
      enc->setVertexBuffer(pointPositions, 0, 0);
      enc->setVertexBuffer(pointColors, 0, 1);
      enc->setVertexBuffer(cameraBuffer, 0, 2);
      enc->setVertexBuffer(pointSizeBuffer, 0, 3);
      enc->drawPrimitives(MTL::PrimitiveTypePoint, NS::UInteger(0), NS::UInteger(points));
    }
    enc->endEncoding();
    lastCommandBuffer = commandBuffer;
    lastPass = pass;
  }

  MTL::CommandBuffer *lastCommandBuffer = nullptr;
  MTL::RenderPassDescriptor *lastPass = nullptr;
};

MetalRenderer::MetalRenderer() : impl_(new Impl()) {}
MetalRenderer::~MetalRenderer() { shutdown(); }

bool MetalRenderer::init(void *sdlWindow) {
  NS::AutoreleasePool *setup = NS::AutoreleasePool::alloc()->init();
  struct Guard {
    NS::AutoreleasePool *p;
    ~Guard() { p->release(); }
  } guard{setup};

  impl_->window = static_cast<SDL_Window *>(sdlWindow);
  impl_->view = SDL_Metal_CreateView(impl_->window);
  impl_->layer = reinterpret_cast<CA::MetalLayer *>(SDL_Metal_GetLayer(impl_->view));
  impl_->device = MTL::CreateSystemDefaultDevice();
  if (!impl_->device) {
    return false;
  }
  impl_->layer->setDevice(impl_->device);
  impl_->layer->setPixelFormat(MTL::PixelFormatBGRA8Unorm);
  impl_->queue = impl_->device->newCommandQueue();

  NS::Error *error = nullptr;
  MTL::CompileOptions *options = MTL::CompileOptions::alloc()->init();
  MTL::Library *library = impl_->device->newLibrary(nsstr(kShaderSource), options, &error);
  options->release();
  if (!library) {
    if (error) {
      std::fprintf(stderr, "Metal shader error: %s\n", error->localizedDescription()->utf8String());
    }
    return false;
  }
  auto pipeline = [&](const char *vs, const char *fs, bool depth) -> MTL::RenderPipelineState * {
    MTL::Function *vfn = library->newFunction(nsstr(vs));
    MTL::Function *ffn = library->newFunction(nsstr(fs));
    MTL::RenderPipelineDescriptor *pd = MTL::RenderPipelineDescriptor::alloc()->init();
    pd->setVertexFunction(vfn);
    pd->setFragmentFunction(ffn);
    pd->colorAttachments()->object(0)->setPixelFormat(MTL::PixelFormatBGRA8Unorm);
    if (depth) {
      pd->setDepthAttachmentPixelFormat(MTL::PixelFormatDepth32Float);
    }
    NS::Error *e = nullptr;
    MTL::RenderPipelineState *state = impl_->device->newRenderPipelineState(pd, &e);
    pd->release();
    vfn->release();
    ffn->release();
    return state;
  };
  impl_->meshPipeline = pipeline("mesh_vertex", "mesh_fragment", true);
  impl_->pointPipeline = pipeline("point_vertex", "point_fragment", true);
  library->release();
  if (!impl_->meshPipeline || !impl_->pointPipeline) {
    return false;
  }

  MTL::DepthStencilDescriptor *dd = MTL::DepthStencilDescriptor::alloc()->init();
  dd->setDepthCompareFunction(MTL::CompareFunctionLess);
  dd->setDepthWriteEnabled(true);
  impl_->depthState = impl_->device->newDepthStencilState(dd);
  dd->release();

  std::vector<float> v;
  std::vector<std::uint32_t> idx;
  makeCube(v, idx);
  impl_->cubeIndexCount = static_cast<int>(idx.size());
  impl_->cubeVertices =
      impl_->device->newBuffer(v.data(), v.size() * 4, MTL::ResourceStorageModeShared);
  impl_->cubeIndices =
      impl_->device->newBuffer(idx.data(), idx.size() * 4, MTL::ResourceStorageModeShared);
  v.clear();
  idx.clear();
  makeSphere(16, 24, v, idx);
  impl_->sphereIndexCount = static_cast<int>(idx.size());
  impl_->sphereVertices =
      impl_->device->newBuffer(v.data(), v.size() * 4, MTL::ResourceStorageModeShared);
  impl_->sphereIndices =
      impl_->device->newBuffer(idx.data(), idx.size() * 4, MTL::ResourceStorageModeShared);

  impl_->cubeInstances = impl_->device->newBuffer(impl_->instanceCapacity * sizeof(InstanceGPU),
                                                  MTL::ResourceStorageModeShared);
  impl_->sphereInstances = impl_->device->newBuffer(impl_->instanceCapacity * sizeof(InstanceGPU),
                                                    MTL::ResourceStorageModeShared);
  impl_->pointPositions =
      impl_->device->newBuffer(impl_->pointCapacity * sizeof(Vec3), MTL::ResourceStorageModeShared);
  impl_->pointColors = impl_->device->newBuffer(impl_->pointCapacity * sizeof(Color),
                                                MTL::ResourceStorageModeShared);
  impl_->cameraBuffer =
      impl_->device->newBuffer(24 * sizeof(float), MTL::ResourceStorageModeShared);
  impl_->pointSizeBuffer = impl_->device->newBuffer(sizeof(float), MTL::ResourceStorageModeShared);
  return true;
}

void MetalRenderer::beginFrame(int pixelWidth, int pixelHeight, const Camera &camera, Color bg) {
  impl_->width = pixelWidth;
  impl_->height = pixelHeight;
  impl_->clearColor[0] = bg.r;
  impl_->clearColor[1] = bg.g;
  impl_->clearColor[2] = bg.b;
  impl_->cubeBatch.clear();
  impl_->sphereBatch.clear();
  impl_->pointPositionBatch.clear();
  impl_->pointColorBatch.clear();

  const float aspect = pixelHeight > 0 ? static_cast<float>(pixelWidth) / pixelHeight : 1.0f;
  const Mat4 vp = multiply(perspectiveMetal(camera.fovYRadians, aspect, camera.nearZ, camera.farZ),
                           lookAt(camera.eye, camera.target, camera.up));
  float cameraData[24] = {};
  for (int i = 0; i < 16; ++i) {
    cameraData[i] = vp[i];
  }
  cameraData[16] = -0.4f;
  cameraData[17] = -1.0f;
  cameraData[18] = -0.5f; // light dir
  cameraData[20] = camera.eye.x;
  cameraData[21] = camera.eye.y;
  cameraData[22] = camera.eye.z;
  std::memcpy(impl_->cameraBuffer->contents(), cameraData, sizeof(cameraData));
}

void MetalRenderer::drawSphere(Vec3 center, float radius, Color color) {
  InstanceGPU inst{
      {radius, 0, 0, 0, 0, radius, 0, 0, 0, 0, radius, 0, center.x, center.y, center.z, 1},
      {color.r, color.g, color.b, 1}};
  impl_->sphereBatch.push_back(inst);
}

void MetalRenderer::drawBox(Vec3 center, const float r[9], Vec3 h, Color color) {
  InstanceGPU inst{{r[0] * h.x, r[1] * h.x, r[2] * h.x, 0, r[3] * h.y, r[4] * h.y, r[5] * h.y, 0,
                    r[6] * h.z, r[7] * h.z, r[8] * h.z, 0, center.x, center.y, center.z, 1},
                   {color.r, color.g, color.b, 1}};
  impl_->cubeBatch.push_back(inst);
}

void MetalRenderer::drawPoints(const Vec3 *positions, const Color *colors, unsigned count,
                               float pointSize) {
  impl_->pointSize = pointSize;
  impl_->pointPositionBatch.assign(positions, positions + count);
  impl_->pointColorBatch.assign(colors, colors + count);
}

void MetalRenderer::endFrame() {
  NS::AutoreleasePool *pool = NS::AutoreleasePool::alloc()->init();
  CA::MetalDrawable *drawable = impl_->layer->nextDrawable();
  if (drawable) {
    impl_->encode(drawable->texture());
    impl_->lastCommandBuffer->presentDrawable(drawable);
    impl_->lastCommandBuffer->commit();
    impl_->lastPass->release();
  }
  pool->release();
}

bool MetalRenderer::captureFrame(const char *pngPath) {
  NS::AutoreleasePool *pool = NS::AutoreleasePool::alloc()->init();
  const int w = impl_->width, h = impl_->height;
  MTL::TextureDescriptor *cd = MTL::TextureDescriptor::texture2DDescriptor(
      MTL::PixelFormatBGRA8Unorm, static_cast<NS::UInteger>(w), static_cast<NS::UInteger>(h),
      false);
  cd->setUsage(MTL::TextureUsageRenderTarget | MTL::TextureUsageShaderRead);
  cd->setStorageMode(MTL::StorageModeShared);
  MTL::Texture *target = impl_->device->newTexture(cd);

  impl_->encode(target);
  impl_->lastCommandBuffer->commit();
  impl_->lastCommandBuffer->waitUntilCompleted();
  impl_->lastPass->release();

  std::vector<unsigned char> bgra(static_cast<std::size_t>(w) * h * 4);
  target->getBytes(bgra.data(), static_cast<NS::UInteger>(w) * 4,
                   MTL::Region(0, 0, 0, NS::UInteger(w), NS::UInteger(h), 1), 0);
  std::vector<unsigned char> rgba(bgra.size());
  for (std::size_t i = 0; i < bgra.size(); i += 4) {
    rgba[i] = bgra[i + 2];
    rgba[i + 1] = bgra[i + 1];
    rgba[i + 2] = bgra[i];
    rgba[i + 3] = bgra[i + 3];
  }
  const bool ok = writePng(pngPath, rgba.data(), w, h);
  target->release();
  pool->release();
  return ok;
}

void MetalRenderer::shutdown() {
  if (!impl_) {
    return;
  }
  for (MTL::Buffer *b :
       {impl_->cubeVertices, impl_->cubeIndices, impl_->sphereVertices, impl_->sphereIndices,
        impl_->cubeInstances, impl_->sphereInstances, impl_->pointPositions, impl_->pointColors,
        impl_->cameraBuffer, impl_->pointSizeBuffer}) {
    if (b) {
      b->release();
    }
  }
  if (impl_->depthTexture) {
    impl_->depthTexture->release();
  }
  if (impl_->meshPipeline) {
    impl_->meshPipeline->release();
  }
  if (impl_->pointPipeline) {
    impl_->pointPipeline->release();
  }
  if (impl_->depthState) {
    impl_->depthState->release();
  }
  if (impl_->queue) {
    impl_->queue->release();
  }
  if (impl_->device) {
    impl_->device->release();
  }
  if (impl_->view) {
    SDL_Metal_DestroyView(impl_->view);
  }
  delete impl_;
  impl_ = nullptr;
}

} // namespace Render
