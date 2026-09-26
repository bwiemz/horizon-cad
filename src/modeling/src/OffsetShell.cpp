// Phase 163: a shell made by offsetting each face, not the profile.
//
// The cavity is the part itself with every face moved inward by the wall's
// thickness, and each face to open moved outward, so the cavity comes out
// through it. Each of the cavity's corners is where its faces' offset
// surfaces meet: a flat face's plane moved along its normal, a facet of a
// cylinder, cone or sphere on that surface grown or shrunk by the thickness.
// The shell is then the part less the cavity, by the Boolean, which keeps
// the part's own faces, their names and their true surfaces, and gives the
// cavity's the offset ones.

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/FacePlane.h"
#include "horizon/modeling/MateGeometry.h"
#include "horizon/modeling/Shell.h"
#include "horizon/modeling/SolidSewer.h"
#include "horizon/topology/GeometryValidator.h"
#include "horizon/topology/Queries.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

using math::Vec3;
using topo::Face;
using topo::Solid;

namespace {

/// One surface a face lies on, and where its offset is: the signed distance
/// along its outward normal the cavity's face is from it.
struct Surface {
    enum class Kind { Plane, Cylinder, Cone, Sphere };
    Kind kind = Kind::Plane;
    Vec3 origin;     ///< a plane's point, an axis point, a cone's apex, a centre
    Vec3 direction;  ///< a plane's outward normal; an axis (a cone's, into it)
    double radius = 0.0;
    double angle = 0.0;  ///< a cone's half-angle
    /// +1 where the outward normal points away from the axis or centre (a
    /// boss), -1 where it points toward it (a hole's wall).
    double sense = 1.0;
    double offset = 0.0;  ///< -thickness for a wall; +thickness for an opening
    bool open = false;

    /// The signed distance of @p p from this surface, along its outward
    /// normal, less the offset: nothing on the offset surface.
    double residual(const Vec3& p) const { return outwardDistance(p) - offset; }
    double outwardDistance(const Vec3& p) const {
        const Vec3 d = p - origin;
        if (kind == Kind::Plane) return d.dot(direction);
        if (kind == Kind::Sphere) return sense * (d.length() - radius);
        const double along = d.dot(direction);
        const double across = (d - direction * along).length();
        if (kind == Kind::Cylinder) return sense * (across - radius);
        return sense * (across * std::cos(angle) - along * std::sin(angle));
    }
    /// The gradient of residual() at @p p.
    Vec3 gradient(const Vec3& p) const {
        const Vec3 d = p - origin;
        if (kind == Kind::Plane) return direction;
        if (kind == Kind::Sphere) {
            const double length = d.length();
            return length > 1e-15 ? d * (sense / length) : Vec3();
        }
        const double along = d.dot(direction);
        const Vec3 radial = d - direction * along;
        const double across = radial.length();
        const Vec3 outward = across > 1e-15 ? radial * (1.0 / across) : Vec3();
        if (kind == Kind::Cylinder) return outward * sense;
        return (outward * std::cos(angle) - direction * std::sin(angle)) * sense;
    }
    /// The same as @p other: one plane, or one ideal's surface.
    bool same(const Surface& other, double tolerance) const {
        if (kind != other.kind) return false;
        if (kind == Kind::Plane) {
            return direction.dot(other.direction) > 1.0 - 1e-9 &&
                   std::abs((other.origin - origin).dot(direction)) < tolerance;
        }
        return false;  // curved ones are told apart by their ideal
    }
};

/// A face's loops' points, the outer first.
std::vector<std::vector<Vec3>> loopsOf(const Face& face) {
    std::vector<std::vector<Vec3>> out;
    const auto walk = [&out](const topo::Wire* wire) {
        std::vector<Vec3> points;
        if (wire != nullptr && wire->halfEdge != nullptr) {
            const topo::HalfEdge* h = wire->halfEdge;
            do {
                points.push_back(h->origin->point);
                h = h->next;
            } while (h != nullptr && h != wire->halfEdge);
        }
        out.push_back(std::move(points));
    };
    walk(face.outerLoop);
    for (const topo::Wire* inner : face.innerLoops) walk(inner);
    return out;
}

Vec3 newell(const std::vector<Vec3>& loop) {
    Vec3 n;
    for (size_t i = 0; i < loop.size(); ++i) {
        const Vec3& a = loop[i];
        const Vec3& b = loop[(i + 1) % loop.size()];
        n += Vec3((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y));
    }
    return n;
}

Vec3 centroidOf(const std::vector<Vec3>& loop) {
    Vec3 c;
    for (const Vec3& p : loop) c += p;
    return loop.empty() ? c : c * (1.0 / static_cast<double>(loop.size()));
}

/// @p ideal moved to where the cavity's face is: grown or shrunk about its
/// axis or centre, or (a cone) slid along its axis. Exact: a rational
/// circle's control points scale with it.
std::shared_ptr<geo::NurbsSurface> offsetIdeal(const geo::NurbsSurface& ideal,
                                               const Surface& surface) {
    std::vector<std::vector<Vec3>> points = ideal.controlPoints();
    const double grown = surface.radius + surface.sense * surface.offset;
    for (auto& row : points) {
        for (Vec3& p : row) {
            const Vec3 d = p - surface.origin;
            if (surface.kind == Surface::Kind::Sphere) {
                p = surface.origin + d * (grown / surface.radius);
            } else if (surface.kind == Surface::Kind::Cylinder) {
                const double along = d.dot(surface.direction);
                const Vec3 axisPoint = surface.origin + surface.direction * along;
                p = axisPoint + (p - axisPoint) * (grown / surface.radius);
            } else {
                // A cone offset by D along its normal is the cone slid by
                // -D / sin(angle) along its axis.
                p = p +
                    surface.direction * (-surface.sense * surface.offset / std::sin(surface.angle));
            }
        }
    }
    return std::make_shared<geo::NurbsSurface>(std::move(points), ideal.weights(), ideal.knotsU(),
                                               ideal.knotsV(), ideal.degreeU(), ideal.degreeV());
}

ShellResult fail(std::string message) {
    ShellResult r;
    r.message = std::move(message);
    return r;
}

}  // namespace

ShellResult Shell::executeOffset(const Solid& solid, double thickness,
                                 const std::vector<topo::TopologyID>& openFaceIds,
                                 const std::string& featureID, NamingScheme naming) {
    if (!(thickness > 0.0) || !std::isfinite(thickness)) {
        return fail("thickness must be positive");
    }
    if (openFaceIds.empty()) {
        return fail("shell requires at least one face to open (closed hollow deferred)");
    }
    if (solid.shells().size() != 1) {
        return fail("the part has several bodies; shelling one of them is not supported yet");
    }
    math::BoundingBox box;
    for (const auto& v : solid.vertices()) box.expand(v.point);
    if (!box.isValid()) return fail("empty solid");
    const double size = (box.max() - box.min()).length();
    const double tolerance = 1e-9 * std::max(size, 1.0);
    const double outward = outwardSign(solid);

    // Each face's surface, the same one for the facets of one ideal and the
    // pieces of one plane.
    std::vector<Surface> surfaces;
    std::map<const geo::NurbsSurface*, size_t> byIdeal;
    std::map<const Face*, size_t> surfaceOf;
    bool anyOpen = false;
    for (const Face& face : solid.faces()) {
        const bool open = std::any_of(
            openFaceIds.begin(), openFaceIds.end(), [&face](const topo::TopologyID& id) {
                return face.topoId == id || face.topoId.isDescendantOf(id);
            });
        anyOpen = anyOpen || open;
        const auto loops = loopsOf(face);
        if (loops.empty() || loops.front().size() < 3) return fail("a face is not a polygon");
        const Vec3 normal = newell(loops.front()).normalized() * outward;
        const Vec3 middle = centroidOf(loops.front());

        Surface surface;
        std::optional<size_t> known;
        const auto frame =
            face.analyticSurface ? MateGeometry::frameForFace(face) : std::optional<MateFrame>{};
        const bool curved = frame && frame->kind != MateFrameKind::Planar;
        if (curved) {
            const auto found = byIdeal.find(face.analyticSurface.get());
            if (found != byIdeal.end()) known = found->second;
            switch (frame->kind) {
                case MateFrameKind::Cylindrical:
                    surface.kind = Surface::Kind::Cylinder;
                    break;
                case MateFrameKind::Conical:
                    surface.kind = Surface::Kind::Cone;
                    break;
                case MateFrameKind::Spherical:
                    surface.kind = Surface::Kind::Sphere;
                    break;
                default:
                    return fail(
                        "a face on a surface that is not a plane, cylinder, cone or sphere "
                        "cannot be offset yet");
            }
            surface.origin = frame->origin;
            surface.direction = frame->direction;
            surface.radius = frame->radius;
            surface.angle = frame->angle;
            // Which way its outward normal points: from the axis, or to it.
            Surface away = surface;
            away.sense = 1.0;
            surface.sense = away.gradient(middle).dot(normal) >= 0.0 ? 1.0 : -1.0;
            if (surface.kind == Surface::Kind::Cone &&
                !(surface.angle > 1e-6 && surface.angle < 1.5707963267948966 - 1e-6)) {
                return fail("a cone too near a cylinder or a plane cannot be offset yet");
            }
        } else {
            const auto plane = planeOf(face, outward);
            if (!plane) {
                return fail(
                    "a face that is not flat, and is no cylinder, cone or sphere, cannot "
                    "be offset yet");
            }
            surface.kind = Surface::Kind::Plane;
            surface.origin = plane->origin;
            surface.direction = plane->normal;
            for (size_t i = 0; i < surfaces.size() && !known; ++i) {
                if (surfaces[i].same(surface, tolerance)) known = i;
            }
        }
        surface.open = open;
        surface.offset = open ? thickness : -thickness;
        if (known) {
            if (surfaces[*known].open != open) {
                return fail("a face is opened in part: open all of it, or none");
            }
            surfaceOf[&face] = *known;
            continue;
        }
        surfaces.push_back(surface);
        if (curved) byIdeal[face.analyticSurface.get()] = surfaces.size() - 1;
        surfaceOf[&face] = surfaces.size() - 1;
    }
    if (!anyOpen) return fail("the face to open is not there");

    // The surfaces each corner lies on.
    std::map<const topo::Vertex*, std::vector<size_t>> around;
    for (const Face& face : solid.faces()) {
        const size_t s = surfaceOf.at(&face);
        const auto visit = [&](const topo::Wire* wire) {
            if (wire == nullptr || wire->halfEdge == nullptr) return;
            const topo::HalfEdge* h = wire->halfEdge;
            do {
                auto& list = around[h->origin];
                if (std::find(list.begin(), list.end(), s) == list.end()) list.push_back(s);
                h = h->next;
            } while (h != nullptr && h != wire->halfEdge);
        };
        visit(face.outerLoop);
        for (const topo::Wire* inner : face.innerLoops) visit(inner);
    }

    // Each corner moved to where its surfaces' offsets meet: from where it
    // is, by least steps (Gauss-Newton, the minimum-norm step), so a corner
    // on fewer than three surfaces keeps its place along the others.
    std::map<const topo::Vertex*, Vec3> moved;
    for (const auto& [vertex, list] : around) {
        Vec3 p = vertex->point;
        const auto rows = static_cast<Eigen::Index>(list.size());
        double worst = 0.0;
        for (int iteration = 0; iteration < 30; ++iteration) {
            Eigen::MatrixXd jacobian(rows, 3);
            Eigen::VectorXd residual(rows);
            for (Eigen::Index i = 0; i < rows; ++i) {
                const Surface& s = surfaces[list[static_cast<size_t>(i)]];
                const Vec3 g = s.gradient(p);
                jacobian.row(i) << g.x, g.y, g.z;
                residual(i) = s.residual(p);
            }
            worst = residual.cwiseAbs().maxCoeff();
            if (worst < tolerance) break;
            const Eigen::Vector3d step =
                jacobian.completeOrthogonalDecomposition().solve(-residual);
            p = p + Vec3(step.x(), step.y(), step.z());
        }
        if (!(worst < 1e3 * tolerance)) {
            return fail(
                "the faces round a corner do not offset to one point (where four or more "
                "meet at a point, as at a pyramid's tip)");
        }
        moved[vertex] = p;
    }

    // The cavity: each face again, its corners moved, facing out, on its
    // offset surface, and named as the shell's inner face of it.
    std::map<size_t, std::shared_ptr<geo::NurbsSurface>> offsetIdeals;
    std::vector<SolidSewer::InputFace> cavity;
    for (const Face& face : solid.faces()) {
        const auto loops = loopsOf(face);
        SolidSewer::InputFace input;
        for (size_t k = 0; k < loops.size(); ++k) {
            std::vector<Vec3> points;
            points.reserve(loops[k].size());
            // By vertex, not position: the loop's own corners.
            const topo::Wire* wire = k == 0 ? face.outerLoop : face.innerLoops[k - 1];
            const topo::HalfEdge* h = wire->halfEdge;
            do {
                points.push_back(moved.at(h->origin));
                h = h->next;
            } while (h != nullptr && h != wire->halfEdge);
            if (outward < 0.0) std::reverse(points.begin(), points.end());
            // Still facing the way it did, and not collapsed: too thick a
            // wall turns a face over, or shrinks it to nothing.
            const Vec3 was = newell(loops[k]) * outward;
            const Vec3 is = newell(points);
            // A hole's too: one bounded by several faces has no face of its
            // own whose check stands for it.
            if (!(is.dot(was) > 1e-9 * was.lengthSquared())) {
                return fail("the wall is too thick for this part: a face of the cavity collapses");
            }
            if (k == 0) {
                input.points = std::move(points);
            } else {
                input.holes.push_back(std::move(points));
            }
        }
        const size_t s = surfaceOf.at(&face);
        if (surfaces[s].kind != Surface::Kind::Plane && face.analyticSurface) {
            auto& ideal = offsetIdeals[s];
            if (!ideal) ideal = offsetIdeal(*face.analyticSurface, surfaces[s]);
            input.analyticSurface = ideal;
        }
        input.topoId = topo::TopologyID::fromTag(featureID + "/inner:" + face.topoId.tag());
        cavity.push_back(std::move(input));
    }
    // Each edge still runs the way it did: none reversed by the offset.
    for (const auto& edge : solid.edges()) {
        const topo::HalfEdge* h = edge.halfEdge;
        if (h == nullptr || h->twin == nullptr) continue;
        const Vec3 was = h->twin->origin->point - h->origin->point;
        const Vec3 is = moved.at(h->twin->origin) - moved.at(h->origin);
        if (!(is.dot(was) > 0.0)) {
            return fail("the wall is too thick for this part: an edge of the cavity turns over");
        }
    }
    // Its linkage is the part's, so the manifold check is no check here: it
    // is the geometry that can go wrong. A loop that crosses itself, or leaves
    // its plane, is caught; two faces crossing each other is not (no check
    // in the kernel sees that yet), past the local checks above.
    auto hollow = SolidSewer::sew(cavity);
    if (!hollow || !hollow->checkManifold()) {
        return fail("the cavity could not be made from the part's faces");
    }
    if (!topo::GeometryValidator::isGeometricallyValid(*hollow)) {
        return fail("the wall is too thick for this part: a face of the cavity crosses itself");
    }
    std::string why;
    auto shelled = BooleanOp::execute(solid, *hollow, BooleanType::Subtract, &why, naming);
    if (!shelled) return fail("the cavity could not be cut: " + why);
    ShellResult result;
    result.ok = true;
    result.solid = std::move(shelled);
    return result;
}

}  // namespace hz::model
