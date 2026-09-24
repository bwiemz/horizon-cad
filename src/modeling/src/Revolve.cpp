#include "horizon/modeling/Revolve.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <string>
#include <utility>

#include "RingStack.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/ProfileValidator.h"
#include "horizon/modeling/SolidSewer.h"
#include "horizon/topology/TopologyID.h"

namespace hz::model {

using namespace hz::topo;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

/// Tolerance on the profile-lies-in-a-half-plane test, relative to the
/// profile's own size so it means the same thing at any modelling scale.
constexpr double kPlanarityTol = 1e-9;

/// Rotate @p point around the axis through @p axisPoint along @p axisDir
/// (normalized) by @p angle radians.  Rodrigues' rotation formula.
Vec3 rotateAroundAxis(const Vec3& point, const Vec3& axisPoint, const Vec3& axisDir, double angle) {
    const Vec3 p = point - axisPoint;
    const double cosA = std::cos(angle);
    const double sinA = std::sin(angle);
    return axisPoint + p * cosA + axisDir.cross(p) * sinA +
           axisDir * (axisDir.dot(p) * (1.0 - cosA));
}

/// A quad, dropping to a triangle where one side is degenerate — which is
/// what a profile vertex sitting on the axis produces, since rotating it
/// leaves it where it was.
void addQuad(std::vector<SolidSewer::InputFace>& faces, const Vec3& a, const Vec3& b, const Vec3& c,
             const Vec3& d, const TopologyID& id) {
    std::vector<Vec3> loop;
    for (const Vec3& p : {a, b, c, d}) {
        if (loop.empty() || p.distanceTo(loop.back()) > SolidSewer::kDefaultWeldTol) {
            loop.push_back(p);
        }
    }
    while (loop.size() > 1 && loop.front().distanceTo(loop.back()) <= SolidSewer::kDefaultWeldTol) {
        loop.pop_back();
    }
    if (loop.size() < 3) {
        return;
    }
    SolidSewer::InputFace f;
    f.points = std::move(loop);
    f.topoId = id;
    faces.push_back(std::move(f));
}

/// The profile expressed in the half-plane it must lie in: radius from the
/// axis and signed distance along it.
struct Cylindrical {
    double radius = 0.0;
    double height = 0.0;
};

/// The exact surface swept by the profile edge from @p a to @p b.
///
/// Only the bands that are curved get one.  A profile edge parallel to the
/// axis sweeps a cylinder and one oblique to it sweeps a cone, and in both
/// cases the band's planar carrier is an approximation worth recording the
/// ideal for.  A profile edge perpendicular to the axis sweeps a flat annulus,
/// whose carrier plane is already exact — only its rims are approximated, and
/// those are recorded as edge curves instead.
std::shared_ptr<geo::NurbsSurface> sweptSurface(const Cylindrical& a, const Cylindrical& b,
                                                const Vec3& axisPoint, const Vec3& axisDir,
                                                double tol) {
    const double dr = b.radius - a.radius;
    const double dh = b.height - a.height;

    if (std::abs(dh) <= tol) {
        return nullptr;  // Flat annulus: the planar carrier is exact.
    }
    if (std::abs(dr) <= tol) {
        const double h0 = std::min(a.height, b.height);
        return std::make_shared<geo::NurbsSurface>(geo::NurbsSurface::makeCylinder(
            axisPoint + axisDir * h0, axisDir, a.radius, std::abs(dh)));
    }

    // Oblique: a cone whose apex is where the profile edge's line meets the
    // axis.  Measure the half angle and height at whichever end is wider.
    const double t = a.radius / (a.radius - b.radius);
    const double apexH = a.height + t * dh;
    const Cylindrical& far = (a.radius > b.radius) ? a : b;
    const double height = far.height - apexH;
    if (std::abs(height) <= tol) {
        return nullptr;
    }
    const Vec3 coneAxis = (height > 0.0) ? axisDir : axisDir * -1.0;
    const double halfAngle = std::atan2(far.radius, std::abs(height));
    return std::make_shared<geo::NurbsSurface>(geo::NurbsSurface::makeCone(
        axisPoint + axisDir * apexH, coneAxis, halfAngle, std::abs(height)));
}

/// Record on every face whose full tag ("source/role") starts with
/// @p facePrefix the ideal surface its planar facets approximate.
void tagAnalyticSurface(topo::Solid& solid, const std::string& facePrefix,
                        const std::shared_ptr<geo::NurbsSurface>& surface) {
    if (surface == nullptr) {
        return;
    }
    for (auto& f : const_cast<std::deque<Face>&>(solid.faces())) {
        if (f.topoId.tag().rfind(facePrefix, 0) == 0) {
            f.analyticSurface = surface;
        }
    }
}

/// Record on every rim edge — one whose two ends share an axial height and a
/// radius, so it is a chord of the circle a profile vertex traced — the arc it
/// approximates.
void tagRimArcs(topo::Solid& solid, const Vec3& axisPoint, const Vec3& axisDir, double tol) {
    for (auto& e : const_cast<std::deque<Edge>&>(solid.edges())) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) {
            continue;
        }
        const Vec3 a = e.halfEdge->origin->point;
        const Vec3 b = e.halfEdge->twin->origin->point;

        const double ha = (a - axisPoint).dot(axisDir);
        const double hb = (b - axisPoint).dot(axisDir);
        if (std::abs(ha - hb) > tol) {
            continue;
        }
        const Vec3 ca = axisPoint + axisDir * ha;
        const double ra = (a - ca).length();
        const double rb = (b - ca).length();
        if (ra <= tol || std::abs(ra - rb) > tol) {
            continue;
        }
        e.analyticCurve =
            std::make_shared<geo::NurbsCurve>(geo::NurbsCurve::makeCircle(ca, ra, axisDir));
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// segmentsForTolerance
// ---------------------------------------------------------------------------

int Revolve::segmentsForTolerance(double radius, double tolerance) {
    if (!(radius > 0.0) || !(tolerance > 0.0)) {
        return kDefaultSegments;
    }
    const double ratio = 1.0 - tolerance / radius;
    if (ratio <= -1.0) {
        return 3;  // Tolerance exceeds the diameter: any polygon will do.
    }
    const double n = math::kPi / std::acos(std::min(1.0, ratio));
    return std::clamp(static_cast<int>(std::ceil(n)), 3, 4096);
}

// ---------------------------------------------------------------------------
// profileRadius
// ---------------------------------------------------------------------------

double Revolve::profileRadius(const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
                              const draft::SketchPlane& plane, const Vec3& axisPoint,
                              const Vec3& axisDirection) {
    if (axisDirection.length() <= 0.0) {
        return 0.0;
    }
    const Vec3 axisDir = axisDirection.normalized();
    const ProfileRegions found = ProfileValidator::regions(profile);
    if (!found.ok()) {
        return 0.0;
    }
    double maxRadius = 0.0;
    for (const auto& region : found.regions) {
        // The outer loops reach furthest; a hole lies inside one.
        for (const auto& v : ringstack::extractProfileVertices(region.outer.orderedEdges, 1e-6)) {
            const Vec3 rel = plane.localToWorld(v) - axisPoint;
            maxRadius = std::max(maxRadius, (rel - axisDir * rel.dot(axisDir)).length());
        }
    }
    return maxRadius;
}

// ---------------------------------------------------------------------------
// execute
// ---------------------------------------------------------------------------

namespace {

/// One closed loop, revolved: the solid execute() builds for a profile that
/// is a single loop. @p axisDir is a unit vector.
std::unique_ptr<topo::Solid> revolveLoop(const ProfileValidationResult& validation,
                                         const draft::SketchPlane& plane, const Vec3& axisPoint,
                                         const Vec3& axisDir, double angle,
                                         const std::string& featureID, int segments,
                                         double chordTolerance, std::string* reason) {
    const auto fail = [reason](std::string why) -> std::unique_ptr<topo::Solid> {
        if (reason) *reason = std::move(why);
        return nullptr;
    };
    const ringstack::SampledProfile sampled = ringstack::sampleProfile(
        validation.orderedEdges, 1e-6, ringstack::ProfileResolution{segments, chordTolerance});
    const std::vector<Vec2>& verts2D = sampled.vertices;
    const size_t N = verts2D.size();
    if (N < 3) {
        return fail("the profile has fewer than three distinct points");
    }

    std::vector<Vec3> profilePts(N);
    for (size_t i = 0; i < N; ++i) {
        profilePts[i] = plane.localToWorld(verts2D[i]);
    }

    // -----------------------------------------------------------------------
    // The profile must lie in a half-plane bounded by the axis.  A profile
    // that crosses the axis sweeps through itself, and one whose plane the
    // axis does not lie in sweeps a self-intersecting shell; both are caught
    // by requiring every radial vector to point the same way.
    // -----------------------------------------------------------------------
    std::vector<Vec3> radial(N);
    double maxRadius = 0.0;
    size_t widest = 0;
    for (size_t i = 0; i < N; ++i) {
        const Vec3 rel = profilePts[i] - axisPoint;
        radial[i] = rel - axisDir * rel.dot(axisDir);
        if (radial[i].length() > maxRadius) {
            maxRadius = radial[i].length();
            widest = i;
        }
    }
    if (maxRadius <= 0.0) {
        return fail("the whole profile lies on the axis");
    }
    const double tol = kPlanarityTol * maxRadius;
    const Vec3 refDir = radial[widest].normalized();

    std::vector<Cylindrical> cyl(N);
    for (size_t i = 0; i < N; ++i) {
        const double r = radial[i].dot(refDir);
        if (r < -tol || (radial[i] - refDir * r).length() > tol) {
            return fail(
                "the profile must lie on one side of the axis, in a plane the axis passes "
                "through");
        }
        cyl[i].radius = std::max(0.0, r);
        cyl[i].height = (profilePts[i] - axisPoint).dot(axisDir);
    }

    // -----------------------------------------------------------------------
    // Sweep the profile through the angle, at the same angular resolution a
    // full turn would use.
    // -----------------------------------------------------------------------
    const bool fullTurn = std::abs(angle - 2.0 * math::kPi) <= 1e-9;
    const int steps = fullTurn
                          ? segments
                          : std::max(1, static_cast<int>(std::ceil(static_cast<double>(segments) *
                                                                   angle / (2.0 * math::kPi))));
    const size_t ringCount = fullTurn ? static_cast<size_t>(steps) : static_cast<size_t>(steps) + 1;

    std::vector<std::vector<Vec3>> rings(ringCount);
    for (size_t k = 0; k < ringCount; ++k) {
        const double a = angle * static_cast<double>(k) / static_cast<double>(steps);
        rings[k].reserve(N);
        for (size_t i = 0; i < N; ++i) {
            rings[k].push_back(rotateAroundAxis(profilePts[i], axisPoint, axisDir, a));
        }
    }

    // -----------------------------------------------------------------------
    // Bands.  Every one of these quads is exactly planar: rotating two points
    // about a common axis keeps all four on the plane whose normal combines
    // the angular bisector with the axis, so the faces the sewer builds carry
    // the surface their loops really lie on.
    // -----------------------------------------------------------------------
    std::vector<SolidSewer::InputFace> faces;
    faces.reserve(ringCount * N + 2);

    for (size_t k = 0; k < ringCount; ++k) {
        const size_t kNext = (k + 1) % ringCount;
        if (!fullTurn && kNext == 0) {
            break;
        }
        for (size_t i = 0; i < N; ++i) {
            const size_t j = (i + 1) % N;
            addQuad(faces, rings[k][i], rings[k][j], rings[kNext][j], rings[kNext][i],
                    TopologyID::make(featureID,
                                     "revolved_" + std::to_string(i) + "_" + std::to_string(k)));
        }
    }

    // A partial turn is capped at both ends.  The bands traverse each ring in
    // profile order, so the starting cap must traverse it the other way for
    // the shared edges to pair into twins; the ending ring is already opposed.
    if (!fullTurn) {
        SolidSewer::InputFace start;
        start.points.assign(rings.front().rbegin(), rings.front().rend());
        start.topoId = TopologyID::make(featureID, "cap_start");
        faces.push_back(std::move(start));

        SolidSewer::InputFace end;
        end.points = rings.back();
        end.topoId = TopologyID::make(featureID, "cap_end");
        faces.push_back(std::move(end));
    }

    ringstack::orientOutward(faces);
    auto solid = SolidSewer::sew(faces);
    if (solid == nullptr) {
        return fail("the revolved faces could not be joined into a closed solid");
    }

    // -----------------------------------------------------------------------
    // Geometry: the ideal each band of facets approximates.
    // -----------------------------------------------------------------------
    for (size_t i = 0; i < N; ++i) {
        const size_t j = (i + 1) % N;
        // A chord of a profile arc sweeps a cone, but what it approximates is
        // the torus-like surface of the arc, so there is no cone to record.
        if (sampled.edgeArc[i] >= 0) continue;
        tagAnalyticSurface(*solid, featureID + "/revolved_" + std::to_string(i) + "_",
                           sweptSurface(cyl[i], cyl[j], axisPoint, axisDir, tol));
    }
    tagRimArcs(*solid, axisPoint, axisDir, tol);

    {
        int idx = 0;
        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            e.topoId = TopologyID::make(featureID, "edge" + std::to_string(idx));
            ++idx;
        }
    }

    return solid;
}

}  // namespace

std::unique_ptr<topo::Solid> Revolve::execute(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
    const draft::SketchPlane& plane, const Vec3& axisPoint, const Vec3& axisDirection, double angle,
    const std::string& featureID, int segments, double chordTolerance, std::string* reason) {
    const auto fail = [reason](std::string why) -> std::unique_ptr<topo::Solid> {
        if (reason) *reason = std::move(why);
        return nullptr;
    };
    if (segments < 3) return fail("a revolve needs at least 3 steps per turn");
    if (!(angle > 0.0) || angle > 2.0 * math::kPi + 1e-9) {
        return fail("the revolve angle must be more than 0 and at most 360 degrees");
    }
    if (axisDirection.length() <= 0.0) {
        return fail("the revolve axis has no direction");
    }
    const Vec3 axisDir = axisDirection.normalized();

    const ProfileRegions found = ProfileValidator::regions(profile);
    if (!found.ok()) return fail(found.errorMessage);
    if (found.isSingleLoop()) {
        return revolveLoop(found.regions.front().outer, plane, axisPoint, axisDir, angle, featureID,
                           segments, chordTolerance, reason);
    }

    // Holes and separate regions, as Extrude builds them: each region less
    // its holes, the regions joined. A hole's cutter turns a twentieth
    // further at each end, so no face of it lies in an end face; through a
    // full turn it is a ring inside the solid, a cavity.
    const bool fullTurn = std::abs(angle - 2.0 * math::kPi) <= 1e-9;
    const double extra = fullTurn ? 0.0 : 0.05 * angle;
    const double cutterAngle = std::min(angle + 2.0 * extra, 2.0 * math::kPi);
    const auto turned = [&](const Vec3& v) {
        return rotateAroundAxis(axisPoint + v, axisPoint, axisDir, -extra) - axisPoint;
    };
    const draft::SketchPlane cutterPlane(
        rotateAroundAxis(plane.origin(), axisPoint, axisDir, -extra), turned(plane.normal()),
        turned(plane.xAxis()));
    std::unique_ptr<topo::Solid> result;
    int holeCount = 0;
    for (size_t r = 0; r < found.regions.size(); ++r) {
        const ProfileRegion& region = found.regions[r];
        const std::string regionID = r == 0 ? featureID : featureID + "~region" + std::to_string(r);
        std::string why;
        auto solid = revolveLoop(region.outer, plane, axisPoint, axisDir, angle, regionID, segments,
                                 chordTolerance, &why);
        if (!solid) return fail(why);
        for (const auto& hole : region.holes) {
            auto cutter = revolveLoop(hole, cutterPlane, axisPoint, axisDir, cutterAngle,
                                      featureID + "~hole" + std::to_string(holeCount++), segments,
                                      chordTolerance, &why);
            if (!cutter) return fail("a hole in the profile: " + why);
            auto cut = BooleanOp::execute(*solid, *cutter, BooleanType::Subtract, &why,
                                          NamingScheme::FromGeometry);
            if (!cut) return fail("a hole in the profile could not be cut: " + why);
            solid = std::move(cut);
        }
        if (!result) {
            result = std::move(solid);
            continue;
        }
        auto joined = BooleanOp::execute(*result, *solid, BooleanType::Union, &why,
                                         NamingScheme::FromGeometry);
        if (!joined) return fail("the profile's regions could not be joined: " + why);
        result = std::move(joined);
    }
    return result;
}

}  // namespace hz::model
