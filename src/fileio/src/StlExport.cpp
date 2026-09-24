#include "horizon/fileio/StlExport.h"

#include <bit>
#include <cstdint>
#include <cstring>

#include "horizon/fileio/AtomicFile.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/SolidTessellator.h"

namespace hz::io {

namespace {

static_assert(std::endian::native == std::endian::little,
              "binary STL is little-endian; this writer copies native floats");

void put(std::string& out, const void* data, size_t size) {
    out.append(static_cast<const char*>(data), size);
}

void putFloat(std::string& out, double value) {
    const auto f = static_cast<float>(value);
    put(out, &f, sizeof f);
}

}  // namespace

std::string StlExport::toBinary(const topo::Solid& solid, double tolerance) {
    const geo::MeshData mesh = model::SolidTessellator::tessellate(solid, tolerance);
    const size_t triangles = mesh.indices.size() / 3;

    std::string out;
    out.reserve(84 + triangles * 50);
    // 80 bytes of header text. It must not begin with "solid", which marks an
    // ASCII STL to the readers that sniff for it.
    char header[80] = {};
    const char text[] = "Horizon CAD binary STL";
    std::memcpy(header, text, sizeof text - 1);
    put(out, header, sizeof header);
    const auto count = static_cast<uint32_t>(triangles);
    put(out, &count, sizeof count);

    const auto vertex = [&mesh](uint32_t i) {
        const size_t at = static_cast<size_t>(i) * 3;
        return math::Vec3(mesh.positions[at], mesh.positions[at + 1], mesh.positions[at + 2]);
    };
    for (size_t t = 0; t < triangles; ++t) {
        const math::Vec3 a = vertex(mesh.indices[t * 3]);
        const math::Vec3 b = vertex(mesh.indices[t * 3 + 1]);
        const math::Vec3 c = vertex(mesh.indices[t * 3 + 2]);
        math::Vec3 n = (b - a).cross(c - a);
        const double len = n.length();
        n = len > 0.0 ? n * (1.0 / len) : math::Vec3(0, 0, 0);
        for (const math::Vec3& v : {n, a, b, c}) {
            putFloat(out, v.x);
            putFloat(out, v.y);
            putFloat(out, v.z);
        }
        const uint16_t attributes = 0;
        put(out, &attributes, sizeof attributes);
    }
    return out;
}

bool StlExport::save(const std::string& path, const topo::Solid& solid, std::string* error) {
    const std::string bytes = toBinary(solid);
    if (bytes.size() <= 84) {
        if (error) *error = "the part has no surface to write";
        return false;
    }
    return writeFileAtomically(pathFromUtf8(path), bytes, error);
}

}  // namespace hz::io
