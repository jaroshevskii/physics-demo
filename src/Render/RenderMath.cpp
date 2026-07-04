#include "RenderMath.hpp"
#include "RenderBackend.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <vector>
#include <zlib.h>

namespace Render {

// --- shared PNG writer (RGBA8, top-down) -------------------------------------
namespace {
void putU32(std::vector<unsigned char> &out, std::uint32_t v) {
  out.push_back(static_cast<unsigned char>(v >> 24));
  out.push_back(static_cast<unsigned char>(v >> 16));
  out.push_back(static_cast<unsigned char>(v >> 8));
  out.push_back(static_cast<unsigned char>(v));
}
void writeChunk(std::vector<unsigned char> &out, const char tag[4],
                const std::vector<unsigned char> &data) {
  putU32(out, static_cast<std::uint32_t>(data.size()));
  const std::size_t typeStart = out.size();
  out.insert(out.end(), tag, tag + 4);
  out.insert(out.end(), data.begin(), data.end());
  const uLong crc =
      crc32(crc32(0L, Z_NULL, 0), out.data() + typeStart, static_cast<uInt>(4 + data.size()));
  putU32(out, static_cast<std::uint32_t>(crc));
}
} // namespace

bool writePng(const char *path, const unsigned char *rgba, int width, int height) {
  std::vector<unsigned char> raw;
  raw.reserve(static_cast<std::size_t>(height) * (1 + width * 4));
  for (int y = 0; y < height; ++y) {
    raw.push_back(0); // filter type: none
    const unsigned char *row = rgba + static_cast<std::size_t>(y) * width * 4;
    raw.insert(raw.end(), row, row + static_cast<std::size_t>(width) * 4);
  }
  uLong bound = compressBound(static_cast<uLong>(raw.size()));
  std::vector<unsigned char> comp(bound);
  if (compress2(comp.data(), &bound, raw.data(), static_cast<uLong>(raw.size()),
                Z_BEST_COMPRESSION) != Z_OK) {
    return false;
  }
  comp.resize(bound);
  std::vector<unsigned char> png;
  const unsigned char signature[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
  png.insert(png.end(), signature, signature + 8);
  std::vector<unsigned char> ihdr;
  putU32(ihdr, static_cast<std::uint32_t>(width));
  putU32(ihdr, static_cast<std::uint32_t>(height));
  ihdr.insert(ihdr.end(), {8, 6, 0, 0, 0}); // 8-bit, RGBA, no filters/interlace
  writeChunk(png, "IHDR", ihdr);
  writeChunk(png, "IDAT", comp);
  writeChunk(png, "IEND", {});
  std::ofstream file(path, std::ios::binary);
  if (!file) {
    return false;
  }
  file.write(reinterpret_cast<const char *>(png.data()), static_cast<std::streamsize>(png.size()));
  return file.good();
}

Vec3 normalized(Vec3 v) {
  const float length = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  return length > 0.0f ? Vec3{v.x / length, v.y / length, v.z / length} : Vec3{};
}

Vec3 cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Mat4 multiply(const Mat4 &a, const Mat4 &b) {
  Mat4 result{};
  for (int column = 0; column < 4; ++column) {
    for (int row = 0; row < 4; ++row) {
      float sum = 0.0f;
      for (int k = 0; k < 4; ++k) {
        sum += a[k * 4 + row] * b[column * 4 + k];
      }
      result[column * 4 + row] = sum;
    }
  }
  return result;
}

Mat4 lookAt(Vec3 eye, Vec3 target, Vec3 up) {
  const Vec3 forward = normalized(Vec3{target.x - eye.x, target.y - eye.y, target.z - eye.z});
  const Vec3 side = normalized(cross(forward, up));
  const Vec3 upward = cross(side, forward);
  Mat4 m{};
  m[0] = side.x;
  m[4] = side.y;
  m[8] = side.z;
  m[12] = -dot(side, eye);
  m[1] = upward.x;
  m[5] = upward.y;
  m[9] = upward.z;
  m[13] = -dot(upward, eye);
  m[2] = -forward.x;
  m[6] = -forward.y;
  m[10] = -forward.z;
  m[14] = dot(forward, eye);
  m[15] = 1.0f;
  return m;
}

Mat4 perspectiveGL(float fovYRadians, float aspect, float nearZ, float farZ) {
  const float f = 1.0f / std::tan(fovYRadians * 0.5f);
  Mat4 m{};
  m[0] = f / aspect;
  m[5] = f;
  m[10] = -(farZ + nearZ) / (farZ - nearZ);
  m[11] = -1.0f;
  m[14] = -(2.0f * farZ * nearZ) / (farZ - nearZ);
  return m;
}

Mat4 perspectiveMetal(float fovYRadians, float aspect, float nearZ, float farZ) {
  const float f = 1.0f / std::tan(fovYRadians * 0.5f);
  Mat4 m{};
  m[0] = f / aspect;
  m[5] = f;
  m[10] = farZ / (nearZ - farZ);
  m[11] = -1.0f;
  m[14] = (farZ * nearZ) / (nearZ - farZ);
  return m;
}

} // namespace Render
