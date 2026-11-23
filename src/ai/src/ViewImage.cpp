#include "horizon/ai/ViewImage.h"

#include <zlib.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

#include "CadToolsCommon.h"
#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/geometry/MeshData.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/topology/Solid.h"

namespace hz::ai {

namespace {

using math::Vec3;
using nlohmann::json;

/// Where the camera looks: across the image (right), up it, and into it.
struct Basis {
    Vec3 right;
    Vec3 up;
    Vec3 into;
};

std::optional<Basis> basisFor(const std::string& view) {
    Vec3 into;
    Vec3 up(0.0, 0.0, 1.0);
    if (view == "front") {
        into = {0.0, 1.0, 0.0};
    } else if (view == "back") {
        into = {0.0, -1.0, 0.0};
    } else if (view == "right") {
        into = {-1.0, 0.0, 0.0};
    } else if (view == "left") {
        into = {1.0, 0.0, 0.0};
    } else if (view == "top") {
        into = {0.0, 0.0, -1.0};
        up = {0.0, 1.0, 0.0};
    } else if (view == "bottom") {
        into = {0.0, 0.0, 1.0};
        up = {0.0, 1.0, 0.0};
    } else if (view == "iso") {
        into = Vec3(-1.0, 1.0, -1.0).normalized();  // from the front, right and top
    } else {
        return std::nullopt;
    }
    const Vec3 right = into.cross(up).normalized();
    return Basis{right, right.cross(into), into};
}

struct Rgb {
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
};

constexpr Rgb kBackground{250, 250, 252};
constexpr Rgb kEdge{35, 40, 52};
constexpr std::array<double, 3> kBase{150.0, 176.0, 208.0};  // a light steel blue

/// A triangle as drawn: its corners on the image, their depths, its shade
/// and the face it belongs to (-1: none known, so its normal is compared).
struct Drawn {
    std::array<double, 3> x{};
    std::array<double, 3> y{};
    std::array<double, 3> z{};
    Vec3 normal;
    Rgb colour;
    long face = -1;
};

std::uint8_t channel(double value) {
    return static_cast<std::uint8_t>(std::clamp(std::lround(value), 0L, 255L));
}

void putPixel(std::vector<std::uint8_t>& rgb, int width, int height, int x, int y, Rgb c) {
    if (x < 0 || y < 0 || x >= width || y >= height) return;
    const std::size_t at = (static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                            static_cast<std::size_t>(x)) *
                           3;
    rgb[at] = c.r;
    rgb[at + 1] = c.g;
    rgb[at + 2] = c.b;
}

/// A line two pixels thick from (x0, y0) to (x1, y1).
void drawLine(std::vector<std::uint8_t>& rgb, int width, int height, double x0, double y0,
              double x1, double y1, Rgb c) {
    const double length = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
    const int steps = std::max(1, static_cast<int>(std::ceil(length)));
    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const int x = static_cast<int>(std::lround(x0 + (x1 - x0) * t));
        const int y = static_cast<int>(std::lround(y0 + (y1 - y0) * t));
        for (int dy = 0; dy <= 1; ++dy) {
            for (int dx = 0; dx <= 1; ++dx) putPixel(rgb, width, height, x + dx, y + dy, c);
        }
    }
}

void appendBigEndian(std::vector<std::uint8_t>& out, std::uint32_t value) {
    for (int shift = 24; shift >= 0; shift -= 8) {
        out.push_back(static_cast<std::uint8_t>((value >> shift) & 0xFFU));
    }
}

void appendChunk(std::vector<std::uint8_t>& out, const char* type,
                 const std::vector<std::uint8_t>& data) {
    appendBigEndian(out, static_cast<std::uint32_t>(data.size()));
    std::vector<std::uint8_t> body(type, type + 4);
    body.insert(body.end(), data.begin(), data.end());
    out.insert(out.end(), body.begin(), body.end());
    const uLong crc = crc32(0L, body.data(), static_cast<uInt>(body.size()));
    appendBigEndian(out, static_cast<std::uint32_t>(crc));
}

}  // namespace

const std::vector<std::string>& viewNames() {
    static const std::vector<std::string> names = {"iso",   "front", "back",  "left",
                                                   "right", "top",   "bottom"};
    return names;
}

std::vector<std::uint8_t> ViewImage::png() const {
    if (width <= 0 || height <= 0 ||
        rgb.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3) {
        return {};
    }
    // Each row: filter type 0 (none), then its pixels.
    const std::size_t row = static_cast<std::size_t>(width) * 3;
    std::vector<std::uint8_t> raw;
    raw.reserve((row + 1) * static_cast<std::size_t>(height));
    for (int y = 0; y < height; ++y) {
        raw.push_back(0);
        const auto start =
            rgb.begin() + static_cast<std::ptrdiff_t>(static_cast<std::size_t>(y) * row);
        raw.insert(raw.end(), start, start + static_cast<std::ptrdiff_t>(row));
    }
    uLongf packedSize = compressBound(static_cast<uLong>(raw.size()));
    std::vector<std::uint8_t> packed(packedSize);
    if (compress2(packed.data(), &packedSize, raw.data(), static_cast<uLong>(raw.size()), 6) !=
        Z_OK) {
        return {};
    }
    packed.resize(packedSize);

    std::vector<std::uint8_t> out = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
    std::vector<std::uint8_t> header;
    appendBigEndian(header, static_cast<std::uint32_t>(width));
    appendBigEndian(header, static_cast<std::uint32_t>(height));
    header.insert(header.end(),
                  {8, 2, 0, 0, 0});  // 8 bits, RGB, deflate, no filter set, no interlace
    appendChunk(out, "IHDR", header);
    appendChunk(out, "IDAT", packed);
    appendChunk(out, "IEND", {});
    return out;
}

std::optional<ViewImage> renderView(const std::vector<ViewMesh>& meshes, const std::string& view,
                                    int width, int height) {
    const auto basis = basisFor(view);
    if (!basis || width < 16 || height < 16) return std::nullopt;

    // Light from over the viewer's shoulder.
    const Vec3 light = (basis->into * -0.8 + basis->up * 0.5 + basis->right * 0.3).normalized();

    // Every triangle, placed, on the image plane (in model units for now).
    std::vector<Drawn> triangles;
    std::map<std::string, long> faces;  // a logical face's number, across meshes
    for (std::size_t m = 0; m < meshes.size(); ++m) {
        const auto& placed = meshes[m];
        if (!placed.mesh) continue;
        const geo::MeshData& mesh = *placed.mesh;
        const auto& p = mesh.positions;
        const bool named = mesh.triangleFaces.size() * 3 == mesh.indices.size();
        for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            std::array<Vec3, 3> corner;
            bool inside = true;
            for (std::size_t k = 0; k < 3; ++k) {
                const std::size_t v = static_cast<std::size_t>(mesh.indices[t + k]) * 3;
                if (v + 2 >= p.size()) {
                    inside = false;
                    break;
                }
                corner[k] = placed.transform.transformPoint(Vec3(static_cast<double>(p[v]),
                                                                 static_cast<double>(p[v + 1]),
                                                                 static_cast<double>(p[v + 2])));
            }
            if (!inside) continue;
            Vec3 normal = (corner[1] - corner[0]).cross(corner[2] - corner[0]);
            if (normal.length() < 1e-12) continue;  // no area
            normal = normal.normalized();
            if (normal.dot(basis->into) > 0.0) normal = normal * -1.0;  // the side seen
            Drawn d;
            for (std::size_t k = 0; k < 3; ++k) {
                d.x[k] = corner[k].dot(basis->right);
                d.y[k] = corner[k].dot(basis->up);
                d.z[k] = corner[k].dot(basis->into);
            }
            d.normal = normal;
            const double lit = 0.35 + 0.65 * std::max(0.0, normal.dot(light));
            d.colour = {channel(kBase[0] * lit), channel(kBase[1] * lit), channel(kBase[2] * lit)};
            if (named) {
                const std::size_t f = mesh.triangleFaces[t / 3];
                if (f < mesh.faceTags.size()) {
                    // A curved face's facets are one face: no edges between them.
                    const std::string key =
                        std::to_string(m) + "#" + model::logicalFace(mesh.faceTags[f]);
                    d.face = faces.emplace(key, static_cast<long>(faces.size())).first->second;
                }
            }
            triangles.push_back(d);
        }
    }
    if (triangles.empty()) return std::nullopt;

    // Fitted, with a margin, the same scale both ways.
    double minX = std::numeric_limits<double>::max();
    double minY = minX;
    double maxX = -minX;
    double maxY = -minX;
    double minZ = minX;
    double maxZ = -minX;
    for (const auto& d : triangles) {
        for (std::size_t k = 0; k < 3; ++k) {
            minX = std::min(minX, d.x[k]);
            maxX = std::max(maxX, d.x[k]);
            minY = std::min(minY, d.y[k]);
            maxY = std::max(maxY, d.y[k]);
            minZ = std::min(minZ, d.z[k]);
            maxZ = std::max(maxZ, d.z[k]);
        }
    }
    const double margin = 0.1;
    const double spanX = std::max(maxX - minX, 1e-9);
    const double spanY = std::max(maxY - minY, 1e-9);
    const double scale = std::min(static_cast<double>(width) * (1.0 - 2.0 * margin) / spanX,
                                  static_cast<double>(height) * (1.0 - 2.0 * margin) / spanY);
    const double midX = (minX + maxX) / 2.0;
    const double midY = (minY + maxY) / 2.0;
    for (auto& d : triangles) {
        for (std::size_t k = 0; k < 3; ++k) {
            d.x[k] = (d.x[k] - midX) * scale + static_cast<double>(width) / 2.0;
            d.y[k] = static_cast<double>(height) / 2.0 - (d.y[k] - midY) * scale;
        }
    }

    // Each pixel's nearest triangle.
    const std::size_t count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    std::vector<double> depth(count, std::numeric_limits<double>::max());
    std::vector<long> owner(count, -1);
    for (std::size_t i = 0; i < triangles.size(); ++i) {
        const Drawn& d = triangles[i];
        const double area =
            (d.x[1] - d.x[0]) * (d.y[2] - d.y[0]) - (d.x[2] - d.x[0]) * (d.y[1] - d.y[0]);
        if (std::abs(area) < 1e-12) continue;  // edge-on
        const int x0 =
            std::max(0, static_cast<int>(std::floor(std::min({d.x[0], d.x[1], d.x[2]}))));
        const int x1 =
            std::min(width - 1, static_cast<int>(std::ceil(std::max({d.x[0], d.x[1], d.x[2]}))));
        const int y0 =
            std::max(0, static_cast<int>(std::floor(std::min({d.y[0], d.y[1], d.y[2]}))));
        const int y1 =
            std::min(height - 1, static_cast<int>(std::ceil(std::max({d.y[0], d.y[1], d.y[2]}))));
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                const double px = static_cast<double>(x) + 0.5;
                const double py = static_cast<double>(y) + 0.5;
                const double w0 =
                    ((d.x[1] - px) * (d.y[2] - py) - (d.x[2] - px) * (d.y[1] - py)) / area;
                const double w1 =
                    ((d.x[2] - px) * (d.y[0] - py) - (d.x[0] - px) * (d.y[2] - py)) / area;
                const double w2 = 1.0 - w0 - w1;
                if (w0 < -1e-9 || w1 < -1e-9 || w2 < -1e-9) continue;
                const double z = w0 * d.z[0] + w1 * d.z[1] + w2 * d.z[2];
                const std::size_t at =
                    static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                    static_cast<std::size_t>(x);
                if (z < depth[at]) {
                    depth[at] = z;
                    owner[at] = static_cast<long>(i);
                }
            }
        }
    }

    ViewImage image;
    image.width = width;
    image.height = height;
    image.rgb.assign(count * 3, 0);
    // Where two pixels show different faces, a face and the background, or
    // surfaces apart in depth, there is an edge.
    const double step = 0.02 * std::max({spanX, spanY, maxZ - minZ});
    const double crease = std::cos(30.0 * detail::kDegree);
    const double flat = std::cos(2.0 * detail::kDegree);
    const auto differs = [&](std::size_t a, std::size_t b) {
        if ((owner[a] < 0) != (owner[b] < 0)) return true;
        if (owner[a] < 0 || owner[a] == owner[b]) return false;
        const Drawn& da = triangles[static_cast<std::size_t>(owner[a])];
        const Drawn& db = triangles[static_cast<std::size_t>(owner[b])];
        if (std::abs(depth[a] - depth[b]) > step) return true;
        const double facing = da.normal.dot(db.normal);
        // Two faces meet at an edge where they turn: pieces of one flat face
        // (split by the holes in it) lie flat together, and show none.
        if (da.face >= 0 && db.face >= 0) return da.face != db.face && facing < flat;
        return facing < crease;
    };
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const std::size_t at = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                                   static_cast<std::size_t>(x);
            bool edge = false;
            if (x + 1 < width) edge = differs(at, at + 1);
            if (!edge && y + 1 < height) edge = differs(at, at + static_cast<std::size_t>(width));
            Rgb c = kBackground;
            if (edge) {
                c = kEdge;
            } else if (owner[at] >= 0) {
                c = triangles[static_cast<std::size_t>(owner[at])].colour;
            }
            putPixel(image.rgb, width, height, x, y, c);
        }
    }

    // The world axes, from a point in the lower left.
    const double ax = 0.06 * static_cast<double>(std::min(width, height)) + 6.0;
    const double ay = static_cast<double>(height) - ax;
    const double reach = 0.06 * static_cast<double>(std::min(width, height));
    const std::array<std::pair<Vec3, Rgb>, 3> axes = {{{{1.0, 0.0, 0.0}, {220, 40, 40}},
                                                       {{0.0, 1.0, 0.0}, {30, 160, 50}},
                                                       {{0.0, 0.0, 1.0}, {40, 80, 230}}}};
    for (const auto& [axis, colour] : axes) {
        drawLine(image.rgb, width, height, ax, ay, ax + axis.dot(basis->right) * reach,
                 ay - axis.dot(basis->up) * reach, colour);
    }
    return image;
}

std::string base64(const std::vector<std::uint8_t>& bytes) {
    static constexpr char kAlphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((bytes.size() + 2) / 3 * 4);
    for (std::size_t i = 0; i < bytes.size(); i += 3) {
        const std::uint32_t a = bytes[i];
        const std::uint32_t b = i + 1 < bytes.size() ? bytes[i + 1] : 0U;
        const std::uint32_t c = i + 2 < bytes.size() ? bytes[i + 2] : 0U;
        const std::uint32_t triple = (a << 16U) | (b << 8U) | c;
        out.push_back(kAlphabet[(triple >> 18U) & 0x3FU]);
        out.push_back(kAlphabet[(triple >> 12U) & 0x3FU]);
        out.push_back(i + 1 < bytes.size() ? kAlphabet[(triple >> 6U) & 0x3FU] : '=');
        out.push_back(i + 2 < bytes.size() ? kAlphabet[triple & 0x3FU] : '=');
    }
    return out;
}

namespace {

json viewSchema() {
    return detail::object(
        {{"view", detail::enumSchema(viewNames(), "which way it is seen from (default iso)")},
         {"width", detail::integerSchema("pixels across, 256 to 1024 (default 768)")}});
}

constexpr const char* kViewDescription =
    "A picture of the model as it is now, seen from one side or in iso: shaded, its edges drawn "
    "dark, the axes marked in the lower left (X red, Y green, Z blue). Look at it to check what "
    "you made matches what was asked: shape, holes, which side features are on.";

ToolResult viewResult(const std::vector<ViewMesh>& meshes, const json& in, const char* nothing) {
    const std::string view = detail::choice(in, "view", viewNames(), std::string("iso"));
    const int width = static_cast<int>(detail::integer(in, "width", 256, 1024, 768));
    const int height = width * 3 / 4;
    const auto image = renderView(meshes, view, width, height);
    if (!image) return ToolResult::error(nothing);
    const std::vector<std::uint8_t> png = image->png();
    if (png.empty()) return ToolResult::error("The view could not be drawn.");
    ToolResult result = ToolResult::ok(json{{"view", view}, {"width", width}, {"height", height}});
    result.images.push_back({"image/png", base64(png)});
    return result;
}

}  // namespace

void registerViewTool(ToolRegistry& registry, const doc::Document& document) {
    registry.add({"get_view_image", kViewDescription, viewSchema()}, ToolAccess::Read,
                 [&document](const json& in) {
                     const topo::Solid* solid = document.solid();
                     std::vector<ViewMesh> meshes;
                     if (solid != nullptr) {
                         meshes.push_back({std::make_shared<const geo::MeshData>(
                                               model::SolidTessellator::tessellate(*solid)),
                                           math::Mat4::identity()});
                     }
                     return viewResult(meshes, in,
                                       "The part has no solid yet: there is nothing to see.");
                 });
}

void registerViewTool(ToolRegistry& registry, const doc::AssemblyDocument& assembly) {
    registry.add({"get_view_image", kViewDescription, viewSchema()}, ToolAccess::Read,
                 [&assembly](const json& in) {
                     std::vector<ViewMesh> meshes;
                     for (const auto& comp : assembly.components()) {
                         if (comp.suppressed) continue;
                         if (auto mesh = comp.mesh()) {
                             meshes.push_back({std::move(mesh), assembly.displayTransform(comp)});
                         }
                     }
                     return viewResult(meshes, in,
                                       "No component of the assembly can be shown: there is "
                                       "nothing to see.");
                 });
}

}  // namespace hz::ai
