#include "horizon/render/MeshPicker.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "horizon/math/Vec4.h"
#include "horizon/render/Camera.h"

namespace hz::render {

namespace {

using math::Vec3;

/// Möller–Trumbore: the ray's distance to triangle (a, b, c), if it hits it
/// in front of its origin.
std::optional<double> rayTriangle(const Vec3& origin, const Vec3& dir, const Vec3& a, const Vec3& b,
                                  const Vec3& c) {
    const Vec3 e1 = b - a;
    const Vec3 e2 = c - a;
    const Vec3 p = dir.cross(e2);
    const double det = e1.dot(p);
    const double scale = std::max(e1.length() * e2.length(), 1e-300);
    if (std::abs(det) < 1e-12 * scale) return std::nullopt;  // edge-on
    const double inv = 1.0 / det;
    const Vec3 s = origin - a;
    const double u = s.dot(p) * inv;
    if (u < 0.0 || u > 1.0) return std::nullopt;
    const Vec3 q = s.cross(e1);
    const double v = dir.dot(q) * inv;
    if (v < 0.0 || u + v > 1.0) return std::nullopt;
    const double t = e2.dot(q) * inv;
    if (t <= 0.0) return std::nullopt;
    return t;
}

/// A world point on screen, in Qt's coordinates, if it is in front of the
/// camera.
std::optional<std::pair<double, double>> onScreen(const math::Mat4& viewProjection, const Vec3& p,
                                                  int width, int height) {
    const math::Vec4 clip = viewProjection * math::Vec4(p, 1.0);
    if (!(clip.w > 1e-12)) return std::nullopt;
    const Vec3 ndc = clip.perspectiveDivide();
    return std::pair{(ndc.x + 1.0) * 0.5 * width, (1.0 - ndc.y) * 0.5 * height};
}

}  // namespace

std::optional<MeshHit> MeshPicker::pickFace(const MeshData& mesh, const math::Mat4& model,
                                            const math::Vec3& origin, const math::Vec3& direction) {
    if (direction.length() <= 0.0 || mesh.positions.empty()) return std::nullopt;
    const Vec3 dir = direction.normalized();
    const auto vertex = [&](uint32_t i) {
        const size_t at = static_cast<size_t>(i) * 3;
        return model.transformPoint(
            Vec3(mesh.positions[at], mesh.positions[at + 1], mesh.positions[at + 2]));
    };
    const size_t vertices = mesh.positions.size() / 3;
    std::optional<MeshHit> best;
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const uint32_t i0 = mesh.indices[t];
        const uint32_t i1 = mesh.indices[t + 1];
        const uint32_t i2 = mesh.indices[t + 2];
        if (i0 >= vertices || i1 >= vertices || i2 >= vertices) continue;
        const auto hit = rayTriangle(origin, dir, vertex(i0), vertex(i1), vertex(i2));
        if (!hit || (best && *hit >= best->distance)) continue;
        MeshHit h;
        h.distance = *hit;
        h.point = origin + dir * *hit;
        h.face = mesh.hasFaces() ? static_cast<int>(mesh.triangleFaces[t / 3]) : -1;
        best = h;
    }
    return best;
}

std::optional<MeshHit> MeshPicker::pickEdge(const MeshData& mesh, const math::Mat4& model,
                                            const Camera& camera, double x, double y, int width,
                                            int height, double tolerancePx, double hiddenBeyond) {
    if (width <= 0 || height <= 0) return std::nullopt;
    const math::Mat4 viewProjection = camera.viewProjectionMatrix();
    const auto [origin, dir] = camera.screenToRay(x, y, width, height);
    const Vec3 unit = dir.normalized();
    // How wide a pixel is at a distance along the ray: the ray one pixel
    // over, compared there. An edge a few pixels from the cursor may sit that
    // far behind the face the ray meets beside it and still be the one seen.
    const auto [origin1, dir1] = camera.screenToRay(x + 1.0, y, width, height);
    const Vec3 unit1 = dir1.normalized();
    const auto pixelAt = [&](double along) {
        return ((origin1 + unit1 * along) - (origin + unit * along)).length();
    };
    std::optional<MeshHit> best;
    double bestPixels = tolerancePx;
    for (size_t e = 0; e < mesh.edges.size(); ++e) {
        const auto& pts = mesh.edges[e].points;
        for (size_t k = 0; k + 5 < pts.size(); k += 3) {
            const Vec3 a = model.transformPoint(Vec3(pts[k], pts[k + 1], pts[k + 2]));
            const Vec3 b = model.transformPoint(Vec3(pts[k + 3], pts[k + 4], pts[k + 5]));
            const auto sa = onScreen(viewProjection, a, width, height);
            const auto sb = onScreen(viewProjection, b, width, height);
            if (!sa || !sb) continue;
            // The nearest point of the segment on screen, and how far it is.
            const double dx = sb->first - sa->first;
            const double dy = sb->second - sa->second;
            const double len2 = dx * dx + dy * dy;
            double s = 0.0;
            if (len2 > 0.0) {
                s = std::clamp(((x - sa->first) * dx + (y - sa->second) * dy) / len2, 0.0, 1.0);
            }
            const double px = sa->first + s * dx - x;
            const double py = sa->second + s * dy - y;
            const double pixels = std::sqrt(px * px + py * py);
            if (pixels > bestPixels) continue;
            // Behind the face the ray meets first: hidden, so not picked.
            const Vec3 at = a + (b - a) * s;
            const double along = (at - origin).dot(unit);
            const double slack = 1e-6 * std::max((at - origin).length(), 1.0) +
                                 pixelAt(along) * (tolerancePx + 1.0) * 2.0;
            if (along > hiddenBeyond + slack) continue;
            bestPixels = pixels;
            MeshHit h;
            h.distance = along;
            h.point = at;
            h.edge = static_cast<int>(e);
            best = h;
        }
    }
    return best;
}

}  // namespace hz::render
