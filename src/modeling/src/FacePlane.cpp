#include "horizon/modeling/FacePlane.h"

#include <algorithm>
#include <cmath>
#include <string_view>
#include <vector>

#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/topology/GeometryValidator.h"

namespace hz::model {

using math::Vec3;

namespace {

/// Whether @p surface is a plane: its normal the same across it, sampled on
/// a 3 x 3 grid (a degenerate sample, as at an apex, left out).
bool flatSurface(const geo::NurbsSurface& surface) {
    const double u0 = surface.uMin();
    const double u1 = surface.uMax();
    const double v0 = surface.vMin();
    const double v1 = surface.vMax();
    std::optional<Vec3> first;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const Vec3 n = surface.normal(u0 + (u1 - u0) * 0.5 * i, v0 + (v1 - v0) * 0.5 * j);
            if (n.length() < 1e-12) continue;
            if (!first) {
                first = n.normalized();
            } else if (n.normalized().dot(*first) < 1.0 - 1e-9) {
                return false;
            }
        }
    }
    return first.has_value();
}

}  // namespace

double outwardSign(const topo::Solid& solid) {
    double sixVolume = 0.0;
    for (const auto& face : solid.faces()) {
        if (!face.outerLoop || !face.outerLoop->halfEdge) continue;
        const topo::HalfEdge* start = face.outerLoop->halfEdge;
        if (!start->origin || !start->next) continue;
        const Vec3& p0 = start->origin->point;
        for (const topo::HalfEdge* he = start->next; he && he->next && he->next != start;
             he = he->next) {
            if (!he->origin || !he->next->origin) break;
            sixVolume += p0.dot(he->origin->point.cross(he->next->origin->point));
        }
    }
    return sixVolume < 0.0 ? -1.0 : 1.0;
}

std::optional<FacePlane> planeOf(const topo::Face& face, double outward) {
    if (!face.outerLoop || !face.outerLoop->halfEdge) return std::nullopt;
    // Not flat, though its corners are in one plane: a facet of a curved
    // face (its ideal, a cylinder's side, curved), or a face on a curved
    // carrier.
    if (face.analyticSurface != nullptr && !flatSurface(*face.analyticSurface)) {
        return std::nullopt;
    }
    Vec3 carrierOrigin;
    Vec3 carrierNormal;
    if (face.surface != nullptr &&
        !topo::GeometryValidator::facePlane(face, carrierOrigin, carrierNormal)) {
        return std::nullopt;
    }
    std::vector<Vec3> points;
    Vec3 normal;
    Vec3 centre;
    const topo::HalfEdge* start = face.outerLoop->halfEdge;
    const topo::HalfEdge* he = start;
    do {
        if (!he->origin || !he->next || !he->next->origin) break;
        const Vec3& a = he->origin->point;
        const Vec3& b = he->next->origin->point;
        normal.x += (a.y - b.y) * (a.z + b.z);
        normal.y += (a.z - b.z) * (a.x + b.x);
        normal.z += (a.x - b.x) * (a.y + b.y);
        centre = centre + a;
        points.push_back(a);
        he = he->next;
    } while (he && he != start && points.size() < 100000);
    if (points.size() < 3 || normal.length() < 1e-12) return std::nullopt;
    normal = normal.normalized() * outward;
    centre = centre / static_cast<double>(points.size());
    double size = 0.0;
    for (const auto& q : points) size = std::max(size, (q - centre).length());
    const bool flat = std::all_of(points.begin(), points.end(), [&](const Vec3& q) {
        return std::abs((q - centre).dot(normal)) <= 1e-7 * std::max(size, 1.0);
    });
    if (!flat) return std::nullopt;
    return FacePlane{centre, normal};
}

std::string wholeFaceName(const std::string& tag) {
    static constexpr std::string_view kPiece = "/piece:";
    std::string out;
    size_t from = 0;
    while (from < tag.size()) {
        const size_t at = tag.find(kPiece, from);
        if (at == std::string::npos) {
            out.append(tag, from, std::string::npos);
            break;
        }
        out.append(tag, from, at - from);
        from = tag.find('/', at + 1);  // what follows the piece's number
        if (from == std::string::npos) break;
    }
    return out;
}

std::optional<FacePlane> planeOfFace(const topo::Solid& solid, const std::string& name,
                                     std::string* why) {
    const double outward = outwardSign(solid);
    std::optional<FacePlane> plane;
    double size = 1.0;  // how far apart its pieces are, for the tolerance
    for (const auto& face : solid.faces()) {
        if (!face.topoId.isValid() || wholeFaceName(face.topoId.tag()) != name) continue;
        const auto piece = planeOf(face, outward);
        if (!piece) {
            if (why != nullptr) *why = "is no longer flat";
            return std::nullopt;
        }
        if (!plane) {
            plane = piece;
            continue;
        }
        // Each piece in the plane of the first, facing the same way.
        const Vec3 apart = piece->origin - plane->origin;
        size = std::max(size, apart.length());
        if (piece->normal.dot(plane->normal) < 1.0 - 1e-9 ||
            std::abs(apart.dot(plane->normal)) > 1e-7 * size) {
            if (why != nullptr) *why = "is no longer flat";
            return std::nullopt;
        }
    }
    if (!plane && why != nullptr) *why = "is not there";
    return plane;
}

}  // namespace hz::model
