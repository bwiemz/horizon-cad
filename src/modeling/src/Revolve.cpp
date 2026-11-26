#include "horizon/modeling/Revolve.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ExactProfile.h"
#include "RingStack.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/geometry/Frame.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Circle2D.h"
#include "horizon/geometry/curves/Line2D.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/ExactBuild.h"
#include "horizon/modeling/ProfileValidator.h"
#include "horizon/modeling/SolidSewer.h"
#include "horizon/topology/ExactTolerance.h"
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
    // A budget too fine to tell from nothing at this radius (ratio rounds to
    // 1) makes n infinite, which as an int was the least count, not the most.
    if (!(n < 4096.0)) return 4096;
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

/// The name's prefix master's faceted revolve gave the band over chord @p i of @p sampled, the
/// ring of @p validation's loop: by position, or (Stable) after the profile element the chord was
/// cut from, as a facet of the surface it sweeps (`revolved:<source>/facet:`, and the chord's
/// number on an arc, `<arcfacet>.`). A band is also found by it, for its ideal surface.
std::string bandPrefix(const ringstack::SampledProfile& sampled,
                       const ProfileValidationResult& validation, size_t i, bool stable) {
    if (!stable) return "revolved_" + std::to_string(i) + "_";
    const int s = i < sampled.edgeSource.size() ? sampled.edgeSource[i] : -1;
    std::string prefix =
        "revolved:" +
        (s < 0 ? std::string("closing") : validation.edgeSources[static_cast<size_t>(s)]) +
        "/facet:";
    if (s >= 0 && sampled.sourceFacets[static_cast<size_t>(s)] > 1) {
        prefix += std::to_string(sampled.edgeFacet[i]) + ".";
    }
    return prefix;
}

/// Master's faceted revolve of one closed loop, before it is sewn: its sampled ring, each point
/// in the half-plane it turns in, and the polygons it sews (outward, named as its scheme names
/// them).
struct RevolveSoup {
    ringstack::SampledProfile sampled;
    std::vector<Cylindrical> cyl;
    double tol = 0.0;  ///< its planarity tolerance, at its widest radius
    std::vector<SolidSewer::InputFace> faces;
};

/// @p validation's loop turned as master's faceted revolve turns it, not yet sewn: nullopt, with
/// @p reason in master's words, where master refuses it. @p axisDir is a unit vector.
std::optional<RevolveSoup> revolveSoup(const ProfileValidationResult& validation,
                                       const draft::SketchPlane& plane, const Vec3& axisPoint,
                                       const Vec3& axisDir, double angle,
                                       const std::string& featureID, int segments,
                                       double chordTolerance, std::string* reason,
                                       NamingScheme naming) {
    const auto fail = [reason](std::string why) -> std::optional<RevolveSoup> {
        if (reason) *reason = std::move(why);
        return std::nullopt;
    };
    RevolveSoup soup;
    soup.sampled = ringstack::sampleProfile(validation.orderedEdges, 1e-6,
                                            ringstack::ProfileResolution{segments, chordTolerance});
    const ringstack::SampledProfile& sampled = soup.sampled;
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
    soup.tol = kPlanarityTol * maxRadius;
    const double tol = soup.tol;
    const Vec3 refDir = radial[widest].normalized();

    soup.cyl.resize(N);
    std::vector<Cylindrical>& cyl = soup.cyl;
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
    // The band over chord i, at step k: by position, or (Stable) after the
    // profile element the chord was cut from, as a facet of the surface it
    // sweeps. A band is also found by the prefix its facets share
    // (bandPrefix), for its ideal surface.
    const bool stable = naming == NamingScheme::Stable;
    const auto bandRole = [&](size_t i, size_t k) {
        return bandPrefix(sampled, validation, i, stable) + std::to_string(k);
    };

    std::vector<SolidSewer::InputFace>& faces = soup.faces;
    faces.reserve(ringCount * N + 2);

    // A band square to the axis is flat. In a full turn, stably named, it is
    // one face, a disk or a ring, not a facet per step (Phase 164): a fillet
    // on its rim then meets one face all round, as on a cylinder's cap.
    std::vector<bool> flat(N, false);
    if (fullTurn && stable) {
        for (size_t i = 0; i < N; ++i) {
            const size_t j = (i + 1) % N;
            flat[i] = sampled.edgeArc[i] < 0 && std::abs(cyl[i].height - cyl[j].height) <= tol &&
                      std::abs(cyl[i].radius - cyl[j].radius) > tol;
        }
    }
    for (size_t k = 0; k < ringCount; ++k) {
        const size_t kNext = (k + 1) % ringCount;
        if (!fullTurn && kNext == 0) {
            break;
        }
        for (size_t i = 0; i < N; ++i) {
            if (flat[i]) continue;
            const size_t j = (i + 1) % N;
            addQuad(faces, rings[k][i], rings[k][j], rings[kNext][j], rings[kNext][i],
                    TopologyID::make(featureID, bandRole(i, k)));
        }
    }
    for (size_t i = 0; i < N; ++i) {
        if (!flat[i]) continue;
        const size_t j = (i + 1) % N;
        // Its quads run i@k -> j@k -> j@k+1 -> i@k+1: round i's circle one
        // way, round j's the other. The wider is its outline, the other (if
        // it is not a point on the axis) its hole.
        std::vector<Vec3> iCircle;
        std::vector<Vec3> jCircle;
        for (size_t k = 0; k < ringCount; ++k) {
            iCircle.push_back(rings[k][i]);
            jCircle.push_back(rings[k][j]);
        }
        std::reverse(iCircle.begin(), iCircle.end());
        const bool iWider = cyl[i].radius > cyl[j].radius;
        SolidSewer::InputFace face;
        face.points = iWider ? iCircle : jCircle;
        if (std::min(cyl[i].radius, cyl[j].radius) > tol) {
            face.holes.push_back(iWider ? jCircle : iCircle);
        }
        std::string name = bandPrefix(sampled, validation, i, stable);
        const std::string facet = "/facet:";
        if (name.size() > facet.size() &&
            name.compare(name.size() - facet.size(), facet.size(), facet) == 0) {
            name.resize(name.size() - facet.size());
        }
        face.topoId = TopologyID::make(featureID, name);
        faces.push_back(std::move(face));
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
    return soup;
}

/// One closed loop, revolved: the solid execute() builds for a profile that
/// is a single loop. @p axisDir is a unit vector.
std::unique_ptr<topo::Solid> revolveLoop(const ProfileValidationResult& validation,
                                         const draft::SketchPlane& plane, const Vec3& axisPoint,
                                         const Vec3& axisDir, double angle,
                                         const std::string& featureID, int segments,
                                         double chordTolerance, std::string* reason,
                                         NamingScheme naming) {
    const auto fail = [reason](std::string why) -> std::unique_ptr<topo::Solid> {
        if (reason) *reason = std::move(why);
        return nullptr;
    };
    const auto soup = revolveSoup(validation, plane, axisPoint, axisDir, angle, featureID, segments,
                                  chordTolerance, reason, naming);
    if (!soup) return nullptr;
    const RevolveSoup& made = *soup;
    auto solid = SolidSewer::sew(made.faces);
    if (solid == nullptr) {
        return fail("the revolved faces could not be joined into a closed solid");
    }

    // -----------------------------------------------------------------------
    // Geometry: the ideal each band of facets approximates.
    // -----------------------------------------------------------------------
    const bool stable = naming == NamingScheme::Stable;
    const size_t N = made.sampled.vertices.size();
    for (size_t i = 0; i < N; ++i) {
        const size_t j = (i + 1) % N;
        // A chord of a profile arc sweeps a cone, but what it approximates is
        // the torus-like surface of the arc, so there is no cone to record.
        if (made.sampled.edgeArc[i] >= 0) continue;
        tagAnalyticSurface(*solid,
                           featureID + "/" + bandPrefix(made.sampled, validation, i, stable),
                           sweptSurface(made.cyl[i], made.cyl[j], axisPoint, axisDir, made.tol));
    }
    tagRimArcs(*solid, axisPoint, axisDir, made.tol);

    if (stable) {
        nameEdgesLogically(*solid);
    } else {
        int idx = 0;
        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            e.topoId = TopologyID::make(featureID, "edge" + std::to_string(idx));
            ++idx;
        }
    }

    return solid;
}

}  // namespace

namespace {

// The exact revolve's (below).
std::optional<std::string> placementRefusal(const ProfileRegions& found,
                                            const draft::SketchPlane& plane, const Vec3& axisPoint,
                                            const Vec3& axisDir);
ExactAndBridge buildExactAndBridge(const ProfileRegions& found, const draft::SketchPlane& plane,
                                   const Vec3& axisPoint, const Vec3& axisDir, double angle,
                                   const std::string& featureID, int segments,
                                   double chordTolerance, std::string* reason);

}  // namespace

std::unique_ptr<topo::Solid> Revolve::execute(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
    const draft::SketchPlane& plane, const Vec3& axisPoint, const Vec3& axisDirection, double angle,
    const std::string& featureID, int segments, double chordTolerance, std::string* reason,
    NamingScheme naming) {
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
    if (naming == NamingScheme::Stable) {
        // The exact revolve's bridge copy: master's facets, named for the exact faces and edges
        // they lie on, and numbered as master's.
        if (auto refused = placementRefusal(found, plane, axisPoint, axisDir)) {
            return fail(std::move(*refused));
        }
        return buildExactAndBridge(found, plane, axisPoint, axisDir, angle, featureID, segments,
                                   chordTolerance, reason)
            .faceted;
    }
    if (found.isSingleLoop()) {
        return revolveLoop(found.regions.front().outer, plane, axisPoint, axisDir, angle, featureID,
                           segments, chordTolerance, reason, naming);
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
    const NamingScheme booleanNaming =
        naming == NamingScheme::Stable ? NamingScheme::Stable : NamingScheme::FromGeometry;
    std::unique_ptr<topo::Solid> result;
    int holeCount = 0;
    for (size_t r = 0; r < found.regions.size(); ++r) {
        const ProfileRegion& region = found.regions[r];
        const std::string regionID = r == 0 ? featureID : featureID + "~region" + std::to_string(r);
        std::string why;
        auto solid = revolveLoop(region.outer, plane, axisPoint, axisDir, angle, regionID, segments,
                                 chordTolerance, &why, naming);
        if (!solid) return fail(why);
        for (const auto& hole : region.holes) {
            auto cutter = revolveLoop(hole, cutterPlane, axisPoint, axisDir, cutterAngle,
                                      featureID + "~hole" + std::to_string(holeCount++), segments,
                                      chordTolerance, &why, naming);
            if (!cutter) return fail("a hole in the profile: " + why);
            auto cut =
                BooleanOp::execute(*solid, *cutter, BooleanType::Subtract, &why, booleanNaming);
            if (!cut) return fail("a hole in the profile could not be cut: " + why);
            solid = std::move(cut);
        }
        if (!result) {
            result = std::move(solid);
            continue;
        }
        auto joined = BooleanOp::execute(*result, *solid, BooleanType::Union, &why, booleanNaming);
        if (!joined) return fail("the profile's regions could not be joined: " + why);
        result = std::move(joined);
    }
    return result;
}

// ---------------------------------------------------------------------------
// The exact revolve (stage 3): whole faces carrying exact geometry.
// ---------------------------------------------------------------------------

namespace {

/// Why master refuses to revolve what @p found holds by @p angle about @p axisDirection, in its
/// words, before it looks at where the profile lies; nullopt where it goes on.
std::optional<std::string> refusal(const ProfileRegions& found, const Vec3& axisDirection,
                                   double angle) {
    if (!(angle > 0.0) || angle > 2.0 * math::kPi + 1e-9) {
        return std::string("the revolve angle must be more than 0 and at most 360 degrees");
    }
    if (axisDirection.length() <= 0.0) return std::string("the revolve axis has no direction");
    if (!found.ok()) return found.errorMessage;
    return std::nullopt;
}

/// The radial vector from the axis through @p axisPoint along the unit @p axisDir to @p p.
Vec3 radialOf(const Vec3& p, const Vec3& axisPoint, const Vec3& axisDir) {
    const Vec3 rel = p - axisPoint;
    return rel - axisDir * rel.dot(axisDir);
}

/// The points of @p loop on @p plane that say where it lies about the axis, as master's ring of
/// it did: each curve's ends and, along an arc or a circle, its middle and its quarter points,
/// then the points where it comes nearest the axis and goes furthest from it.
std::vector<Vec3> keyPoints(const ProfileValidationResult& loop, const draft::SketchPlane& plane,
                            const Vec3& axisPoint, const Vec3& axisDir) {
    struct Round {
        Vec2 center;
        double radius = 0.0;
        double start = 0.0;
        double sweep = 0.0;
    };
    std::vector<Vec2> points;
    std::vector<Round> rounds;
    for (const auto& e : loop.orderedEdges) {
        if (const auto* l = dynamic_cast<const draft::DraftLine*>(e.get())) {
            points.push_back(l->start());
            points.push_back(l->end());
        } else if (const auto* a = dynamic_cast<const draft::DraftArc*>(e.get())) {
            points.push_back(a->startPoint());
            points.push_back(a->endPoint());
            rounds.push_back({a->center(), a->radius(), a->startAngle(), a->sweepAngle()});
        } else if (const auto* c = dynamic_cast<const draft::DraftCircle*>(e.get())) {
            rounds.push_back({c->center(), c->radius(), 0.0, math::kTwoPi});
        }
    }
    const auto on = [](const Round& r, double angle) {
        return Vec2(r.center.x + r.radius * std::cos(angle),
                    r.center.y + r.radius * std::sin(angle));
    };
    // Whether @p angle is within @p r's sweep, a whole turn round.
    const auto within = [](const Round& r, double angle) {
        const double d =
            std::fmod(std::fmod(angle - r.start, math::kTwoPi) + math::kTwoPi, math::kTwoPi);
        return d <= r.sweep;
    };
    for (const Round& r : rounds) {
        points.push_back(on(r, r.start + r.sweep / 2.0));
        for (int k = 0; k < 4; ++k) {
            const double q = math::kHalfPi * static_cast<double>(k);
            if (within(r, q)) points.push_back(on(r, q));
        }
    }
    std::vector<Vec3> out;
    out.reserve(points.size() + 2 * rounds.size());
    Vec3 widest;
    for (const Vec2& q : points) {
        out.push_back(plane.localToWorld(q));
        const Vec3 radial = radialOf(out.back(), axisPoint, axisDir);
        if (radial.length() > widest.length()) widest = radial;
    }
    // Along the side the profile lies to, in the sketch: an arc is furthest from the axis and
    // nearest it there and opposite.
    const double sx = widest.dot(plane.xAxis());
    const double sy = widest.dot(plane.yAxis());
    if (std::hypot(sx, sy) > 0.0) {
        const double side = std::atan2(sy, sx);
        for (const Round& r : rounds) {
            for (const double a : {side, side + math::kPi}) {
                if (within(r, a)) out.push_back(plane.localToWorld(on(r, a)));
            }
        }
    }
    return out;
}

/// The side of the axis @p points lie to, as master's revolveLoop found it: the unit radial
/// vector of the one furthest from the axis, @p reach that distance.
Vec3 sideOf(const std::vector<Vec3>& points, const Vec3& axisPoint, const Vec3& axisDir,
            double& reach) {
    reach = 0.0;
    Vec3 widest;
    for (const Vec3& p : points) {
        const Vec3 radial = radialOf(p, axisPoint, axisDir);
        if (radial.length() > reach) {
            reach = radial.length();
            widest = radial;
        }
    }
    return reach > 0.0 ? widest / reach : Vec3();
}

/// Master's check that the loop whose points are @p points lies in a half-plane bounded by the
/// axis (revolveLoop's): its reason, in its words, where it does not.
std::optional<std::string> halfPlaneRefusal(const std::vector<Vec3>& points, const Vec3& axisPoint,
                                            const Vec3& axisDir) {
    double reach = 0.0;
    const Vec3 refDir = sideOf(points, axisPoint, axisDir, reach);
    if (reach <= 0.0) return std::string("the whole profile lies on the axis");
    const double tol = kPlanarityTol * reach;
    for (const Vec3& p : points) {
        const Vec3 radial = radialOf(p, axisPoint, axisDir);
        const double r = radial.dot(refDir);
        if (r < -tol || (radial - refDir * r).length() > tol) {
            return std::string(
                "the profile must lie on one side of the axis, in a plane the axis passes "
                "through");
        }
    }
    return std::nullopt;
}

/// Master's checks of where @p found lies about the axis, loop by loop as its revolveLoop made
/// them (a hole's reason "a hole in the profile: <why>"); its reason where one fails.
std::optional<std::string> placementRefusal(const ProfileRegions& found,
                                            const draft::SketchPlane& plane, const Vec3& axisPoint,
                                            const Vec3& axisDir) {
    for (const ProfileRegion& region : found.regions) {
        if (auto why = halfPlaneRefusal(keyPoints(region.outer, plane, axisPoint, axisDir),
                                        axisPoint, axisDir)) {
            return why;
        }
        for (const auto& hole : region.holes) {
            if (auto why = halfPlaneRefusal(keyPoints(hole, plane, axisPoint, axisDir), axisPoint,
                                            axisDir)) {
                return "a hole in the profile: " + *why;
            }
        }
    }
    return std::nullopt;
}

/// What a curve of the profile sweeps, turned about the axis.
enum class Sweeps {
    Nothing,     ///< a line on the axis
    Plane,       ///< a line square to the axis: a disc, a ring or a sector of one
    Cylinder,    ///< a line along the axis
    Cone,        ///< any other line, to the apex where it meets the axis
    Sphere,      ///< an arc centred on the axis
    Torus,       ///< an arc whose circle clears the axis
    Revolution,  ///< an arc whose circle reaches the axis (Ruling 2): a NURBS surface
};

/// A point of the half-plane the profile turns in: its distance from the axis and its height
/// along it.
struct RH {
    double r = 0.0;
    double h = 0.0;
};

/// Builds the exact revolve of a profile's regions by an angle about an axis in the profile's
/// plane: a shell for each region, and, through a whole turn, one for each hole (a cavity).
///
/// The profile lies in the half-plane at u = 0 spanned by refDir and the axis, where (r, h) are a
/// point's distance from the axis and height along it. Its loops run with their material on
/// their left in (r, h) (an outer loop counter-clockwise, a hole clockwise), so that a curve
/// from corner i to corner i + 1 has the solid to the left as it runs, and its face's outward
/// normal is its direction turned a quarter clockwise in (r, h). Each face lies on a surface
/// turned about the axis, u its angle (theta) round it, with the turn (sigma = +1) or against it
/// (-1, a cone whose frame's z points down the axis to its wider end: u = 2 pi - theta), and v
/// along the curve; it is reversed where v runs against the loop as u runs with the turn, or
/// with it as u runs against. Its outer loop, counter-clockwise in (theta, along the loop):
/// along the rim its first corner traces (u from theta 0 to the angle), along the curve turned
/// by the angle, back along the rim its second corner traces, and back along the curve at theta
/// 0. A corner on the axis traces no rim: the loop goes along the pole or the apex by a jump in
/// (u, v). Through a whole turn the curve at 0 and at the angle is one edge, the seam, and a
/// flat face is a disc or a ring (its wider rim its outer loop); through part of one, each is an
/// edge of a cap, and a line on the axis is the edge where the caps meet.
class ExactRevolution {
public:
    ExactRevolution(topo::Solid& solid, const draft::SketchPlane& plane, const Vec3& axisPoint,
                    const Vec3& axis, const Vec3& refDir, double angle, double flatTol,
                    double axisTol)
        : solid_(solid),
          plane_(plane),
          rotated_(plane),
          axisPoint_(axisPoint),
          axis_(axis),
          refDir_(refDir),
          yRot_(axis.cross(refDir)),
          whole_(std::abs(angle - math::kTwoPi) <= 1e-9),
          angle_(whole_ ? math::kTwoPi : angle),
          flatTol_(flatTol),
          axisTol_(axisTol) {
        const double xr = plane.xAxis().dot(refDir);
        const double xa = plane.xAxis().dot(axis);
        const double yr = plane.yAxis().dot(refDir);
        const double ya = plane.yAxis().dot(axis);
        mirrored_ = xr * ya - yr * xa < 0.0;
        if (!whole_) {
            rotated_ = draft::SketchPlane(turned(plane.origin()), turnedDirection(plane.normal()),
                                          turnedDirection(plane.xAxis()));
        }
    }

    /// Whether the sketch's (x, y) run the other way round to (r, h): then each loop is turned
    /// round to run with its material on its left in (r, h).
    bool mirrored() const { return mirrored_; }

    /// Adds @p region, its loops running as the class says: a shell of it (through a whole turn,
    /// a shell for it and one for each hole), its outer loop's faces named for @p source, each
    /// hole's for the source @p holeSources gives it. False, with @p why, where a curve has no
    /// exact sweep here.
    bool addRegion(const exact::Region& region, const std::string& source,
                   const std::vector<std::string>& holeSources, std::string& why) {
        topo::Shell* shell = newShell();
        Face* start = nullptr;
        Face* end = nullptr;
        if (!whole_) {
            // The caps face away from the turn: the start's outward normal is -yRot (the way u
            // leaves theta 0 by, reversed), the end's yRot turned by the angle. Each is a plane in
            // its sketch's own frame, so its (u, v) are the sketch's (x, y), reversed where the
            // sketch's normal points the other way.
            const bool startReversed = plane_.normal().dot(yRot_) > 0.0;
            start = face(shell, source, "cap_start", std::make_shared<geo::Plane>(frameOf(plane_)),
                         startReversed);
            end = face(shell, source, "cap_end", std::make_shared<geo::Plane>(frameOf(rotated_)),
                       !startReversed);
        }
        if (!addLoop(region.outer, source, shell, start, end, true, why)) return false;
        for (size_t k = 0; k < region.holes.size(); ++k) {
            topo::Shell* holeShell = whole_ ? newShell() : shell;
            if (!addLoop(region.holes[k], holeSources[k], holeShell, start, end, false, why)) {
                return false;
            }
        }
        return true;
    }

    /// Where the profile's curves miss the corners they meet at by more than the model's
    /// resolution (a sketch's ends join within the profile's tolerance), the corner's vertices
    /// and the rim it traces are given that gap as their tolerance (topo::ToleranceGrowth: as far
    /// as a thousandth of the solid's size). False, with @p why, past that.
    bool coverGaps(std::string& why) {
        if (gaps_.empty()) return true;
        const topo::ToleranceGrowth growth(topo::exactExtent(solid_));
        for (const Gap& g : gaps_) {
            bool covered = true;
            for (Vertex* v : {g.start, g.end}) {
                if (v != nullptr) covered = covered && growth.grow(v->tolerance, g.gap);
            }
            if (g.rim != nullptr) covered = covered && growth.grow(g.rim->tolerance, g.gap);
            if (!covered) {
                std::ostringstream out;
                out.imbue(std::locale::classic());
                out << "the profile's curves miss each other at (" << std::setprecision(6)
                    << g.where.x << ", " << g.where.y << ") by " << g.gap
                    << ", more than its exact form can take at its size";
                why = out.str();
                return false;
            }
        }
        return true;
    }

private:
    /// A corner where a curve's own end misses its vertices.
    struct Gap {
        Vertex* start = nullptr;  ///< at theta 0
        Vertex* end = nullptr;    ///< at the angle (the same through a whole turn)
        Edge* rim = nullptr;
        double gap = 0.0;
        Vec2 where;
    };

    /// The face a curve sweeps, and how its (u, v) run.
    struct Band {
        Sweeps kind = Sweeps::Nothing;
        std::shared_ptr<const geo::Surface> surface;
        bool reversed = false;
        int sigma = 1;    ///< u runs with the turn (+1) or against it (-1)
        double v0 = 0.0;  ///< v at the curve's first corner, as the loop runs it
        double v1 = 0.0;  ///< and at its second
        bool up = false;  ///< a plane's: its outward normal points up the axis
        double h = 0.0;   ///< a plane's height
        Face* face = nullptr;

        /// Whether it has a seam through a whole turn: any curved face.
        bool seamed() const { return kind != Sweeps::Nothing && kind != Sweeps::Plane; }
    };

    static geo::Frame frameOf(const draft::SketchPlane& plane) {
        geo::Frame f;
        f.origin = plane.origin();
        f.x = plane.xAxis();
        f.y = plane.yAxis();
        f.z = plane.normal();
        return f;
    }

    Vec3 turned(const Vec3& p) const { return rotateAroundAxis(p, axisPoint_, axis_, angle_); }

    Vec3 turnedDirection(const Vec3& d) const {
        return rotateAroundAxis(axisPoint_ + d, axisPoint_, axis_, angle_) - axisPoint_;
    }

    RH rh(const Vec2& q) const {
        const Vec3 rel = plane_.localToWorld(q) - axisPoint_;
        return {rel.dot(refDir_), rel.dot(axis_)};
    }

    /// The angle in (r, h) of the direction at angle @p alpha in the sketch's (x, y).
    double phiOf(double alpha) const {
        const Vec3 d = plane_.xAxis() * std::cos(alpha) + plane_.yAxis() * std::sin(alpha);
        return std::atan2(d.dot(axis_), d.dot(refDir_));
    }

    /// u at angle @p theta round the axis on a face whose u runs @p sigma.
    static double uAt(double theta, int sigma) { return sigma > 0 ? theta : math::kTwoPi - theta; }

    /// The frame of the circles about the axis at height @p h: x along refDir, z up the axis.
    geo::Frame rimFrame(double h) const {
        geo::Frame f;
        f.origin = axisPoint_ + axis_ * h;
        f.x = refDir_;
        f.y = yRot_;
        f.z = axis_;
        return f;
    }

    topo::Shell* newShell() {
        topo::Shell* shell = solid_.allocShell();
        shell->solid = &solid_;
        return shell;
    }

    Vertex* vertex(const Vec3& p) {
        Vertex* v = solid_.allocVertex();
        v->point = p;
        return v;
    }

    Face* face(topo::Shell* shell, const std::string& source, const std::string& role,
               std::shared_ptr<const geo::Surface> surface, bool reversed) {
        Face* f = solid_.allocFace();
        f->topoId = TopologyID::make(source, role);
        f->exactSurface = std::move(surface);
        f->reversed = reversed;
        f->shell = shell;
        shell->faces.push_back(f);
        return f;
    }

    HalfEdge* halfEdge(Vertex* origin) {
        HalfEdge* he = solid_.allocHalfEdge();
        he->origin = origin;
        if (origin->halfEdge == nullptr) origin->halfEdge = he;
        return he;
    }

    /// Closes @p hes into a loop of @p f: its outer loop, or one more inner loop.
    void close(Face* f, const std::vector<HalfEdge*>& hes, bool outer) {
        topo::Wire* w = solid_.allocWire();
        w->halfEdge = hes.front();
        for (size_t i = 0; i < hes.size(); ++i) {
            HalfEdge* he = hes[i];
            he->face = f;
            he->next = hes[(i + 1) % hes.size()];
            he->next->prev = he;
        }
        if (outer) {
            f->outerLoop = w;
        } else {
            f->innerLoops.push_back(w);
        }
    }

    /// Joins @p a and @p b as the two sides of an edge along @p course, which runs from a's
    /// origin to b's.
    Edge* edge(HalfEdge* a, HalfEdge* b, const exact::Course<geo::Curve3>& course) {
        Edge* e = solid_.allocEdge();
        e->halfEdge = a;
        a->twin = b;
        b->twin = a;
        a->edge = b->edge = e;
        e->exactCurve = course.curve;
        e->curveStart = course.start;
        e->curveEnd = course.end;
        return e;
    }

    static void setPcurve(HalfEdge* he, std::shared_ptr<const geo::Curve2D> curve, double start,
                          double end) {
        he->pcurve = std::move(curve);
        he->pcurveStart = start;
        he->pcurveEnd = end;
    }

    static void setLinePcurve(HalfEdge* he, const Vec2& from, const Vec2& to) {
        setPcurve(he, std::make_shared<geo::Line2D>(from, to), 0.0, 1.0);
    }

    /// What @p c, from corner @p a to corner @p b (on the axis where @p aOn, @p bOn say), sweeps.
    /// False, with @p why, for an arc that reaches the axis between its ends.
    bool band(const exact::LoopCurve& c, const RH& a, const RH& b, bool aOn, bool bOn, Band& out,
              std::string& why) const {
        if (!c.curved()) {
            if (aOn && bOn) {
                out.kind = Sweeps::Nothing;
                return true;
            }
            if (std::abs(b.h - a.h) <= flatTol_) {
                // Square to the axis: a plane facing down where the line runs out from the axis.
                out.kind = Sweeps::Plane;
                out.up = b.r < a.r;
                out.h = a.h;
                out.surface = std::make_shared<geo::Plane>(geo::Frame::fromAxis(
                    axisPoint_ + axis_ * a.h, out.up ? axis_ : axis_ * -1.0, refDir_));
                return true;
            }
            if (std::abs(b.r - a.r) <= flatTol_) {
                out.kind = Sweeps::Cylinder;
                out.surface = std::make_shared<geo::Cylinder>(
                    geo::Frame::fromAxis(axisPoint_, axis_, refDir_), a.r);
                out.v0 = a.h;
                out.v1 = b.h;
                out.reversed = out.v1 < out.v0;
                return true;
            }
            // A cone from its narrower end (an apex where that is on the axis), its frame's z
            // toward its wider end, so that its half-angle is positive.
            const bool aNarrow = a.r < b.r;
            const RH& n = aNarrow ? a : b;
            const RH& w = aNarrow ? b : a;
            const double rn = (aNarrow ? aOn : bOn) ? 0.0 : n.r;
            const double s = w.h > n.h ? 1.0 : -1.0;
            out.kind = Sweeps::Cone;
            out.sigma = w.h > n.h ? 1 : -1;
            out.surface = std::make_shared<geo::Cone>(
                geo::Frame::fromAxis(axisPoint_ + axis_ * n.h, axis_ * s, refDir_), rn,
                std::atan2(w.r - rn, std::abs(w.h - n.h)));
            out.v0 = (a.h - n.h) * s;
            out.v1 = (b.h - n.h) * s;
            out.reversed = (out.v1 - out.v0) * s < 0.0;
            return true;
        }
        const RH centre = rh(c.center);
        const double rho = c.radius;
        // The angles in (r, h) the curve runs between, as the loop runs it.
        const double alpha = c.forward ? c.ownStart : c.ownStart + c.sweep;
        const double turn = (c.forward ? 1.0 : -1.0) * (mirrored_ ? -1.0 : 1.0);
        const double phi0 = phiOf(alpha);
        const double phi1 = phi0 + turn * c.sweep;
        // Where it comes nearest the axis (phi = pi), if between its ends.
        const double lo = std::min(phi0, phi1);
        const double hi = std::max(phi0, phi1);
        const double nearest =
            math::kPi + math::kTwoPi * std::ceil((lo - math::kPi) / math::kTwoPi);
        if (nearest > lo && nearest < hi && centre.r - rho <= axisTol_) {
            std::ostringstream out;
            out.imbue(std::locale::classic());
            out << "an arc of the profile about (" << std::setprecision(6) << c.center.x << ", "
                << c.center.y << ") reaches the axis between its ends";
            why = out.str();
            return false;
        }
        const geo::Frame frame =
            geo::Frame::fromAxis(axisPoint_ + axis_ * centre.h, axis_, refDir_);
        if (std::abs(centre.r) <= axisTol_) {
            out.kind = Sweeps::Sphere;
            out.surface = std::make_shared<geo::Sphere>(frame, rho);
        } else if (centre.r > rho) {
            out.kind = Sweeps::Torus;
            out.surface = std::make_shared<geo::Torus>(frame, centre.r, rho);
        } else {
            // Ruling 2: a circle that reaches the axis is no typed torus.
            out.kind = Sweeps::Revolution;
            const geo::CircleCurve own(c.ownFrame(plane_, Vec3{}), rho);
            const auto profile = own.toNurbs(0.0, c.sweep);
            auto surface = std::make_shared<geo::NurbsSurface>(
                geo::NurbsSurface::makeRevolution(*profile, axisPoint_, axis_, angle_));
            out.v0 = c.forward ? surface->vMin() : surface->vMax();
            out.v1 = c.forward ? surface->vMax() : surface->vMin();
            out.reversed = out.v1 < out.v0;
            out.surface = std::move(surface);
            return true;
        }
        out.v0 = phi0;
        out.v1 = phi1;
        out.reversed = out.v1 < out.v0;
        return true;
    }

    /// Point (u, v) of a plane's own coordinates at radius @p r and angle @p theta round the axis.
    static double circleAt(double theta, bool up) { return up ? theta : math::kTwoPi - theta; }

    bool addLoop(const exact::Loop& loop, const std::string& source, topo::Shell* shell,
                 Face* capStart, Face* capEnd, bool outer, std::string& why) {
        const size_t m = loop.curves.size();
        std::vector<RH> corner(m);
        std::vector<bool> onAxis(m);
        std::vector<Vec3> at0(m);
        for (size_t j = 0; j < m; ++j) {
            corner[j] = rh(loop.curves[j].start);
            onAxis[j] = corner[j].r <= axisTol_;
            at0[j] = plane_.localToWorld(loop.curves[j].start);
        }
        std::vector<Band> bands(m);
        for (size_t i = 0; i < m; ++i) {
            const size_t j = (i + 1) % m;
            if (!band(loop.curves[i], corner[i], corner[j], onAxis[i], onAxis[j], bands[i], why)) {
                return false;
            }
        }
        // A corner's vertex at theta 0 and at the angle: through a whole turn one, and none on
        // the axis where no seam ends.
        std::vector<Vertex*> v0(m, nullptr);
        std::vector<Vertex*> vA(m, nullptr);
        for (size_t j = 0; j < m; ++j) {
            const bool seamEnds = bands[j].seamed() || bands[(j + m - 1) % m].seamed();
            if (whole_ && onAxis[j] && !seamEnds) continue;
            v0[j] = vertex(at0[j]);
            vA[j] = whole_ || onAxis[j] ? v0[j] : vertex(turned(at0[j]));
        }
        for (size_t i = 0; i < m; ++i) {
            if (bands[i].kind == Sweeps::Nothing) continue;
            bands[i].face = face(shell, source, "revolved:" + loop.curves[i].source,
                                 bands[i].surface, bands[i].reversed);
        }

        std::vector<HalfEdge*> rimOut(m, nullptr);   // along a corner's rim, with the turn
        std::vector<HalfEdge*> rimBack(m, nullptr);  // and back
        std::vector<HalfEdge*> along(m, nullptr);    // along a curve turned by the angle
        std::vector<HalfEdge*> back(m, nullptr);     // back along it at theta 0
        std::vector<HalfEdge*> capStartHes(m, nullptr);
        std::vector<HalfEdge*> capEndHes(m, nullptr);
        for (size_t i = 0; i < m; ++i) {
            const exact::LoopCurve& c = loop.curves[i];
            const Band& b = bands[i];
            const size_t j0 = i;
            const size_t j1 = (i + 1) % m;
            if (!whole_) {
                capStartHes[i] = halfEdge(v0[j0]);
                capEndHes[i] = halfEdge(vA[j1]);
                const auto forth = c.pcurve(true);
                const auto backward = c.pcurve(false);
                setPcurve(capStartHes[i], forth.curve, forth.start, forth.end);
                setPcurve(capEndHes[i], backward.curve, backward.start, backward.end);
            }
            if (b.kind == Sweeps::Nothing) continue;
            if (!onAxis[j0]) rimOut[j0] = halfEdge(v0[j0]);
            if (!onAxis[j1]) rimBack[j1] = halfEdge(vA[j1]);
            if (b.kind == Sweeps::Plane) {
                // In the plane's own (u, v): a rim is a circle about its origin, its angle theta
                // where the plane faces up the axis and 2 pi - theta where it faces down.
                const double t0 = circleAt(0.0, b.up);
                const double tA = circleAt(angle_, b.up);
                if (rimOut[j0] != nullptr) {
                    setPcurve(rimOut[j0],
                              std::make_shared<geo::Circle2D>(Vec2{0.0, 0.0}, corner[j0].r), t0,
                              tA);
                }
                if (rimBack[j1] != nullptr) {
                    setPcurve(rimBack[j1],
                              std::make_shared<geo::Circle2D>(Vec2{0.0, 0.0}, corner[j1].r), tA,
                              t0);
                }
                if (whole_) {
                    // A disc or a ring: the wider rim its outer loop, the other its hole.
                    HalfEdge* wide = corner[j0].r > corner[j1].r ? rimOut[j0] : rimBack[j1];
                    HalfEdge* narrow = wide == rimOut[j0] ? rimBack[j1] : rimOut[j0];
                    close(b.face, {wide}, true);
                    if (narrow != nullptr) close(b.face, {narrow}, false);
                    continue;
                }
            }
            along[i] = halfEdge(vA[j0]);
            back[i] = halfEdge(v0[j1]);
            std::vector<HalfEdge*> hes;
            if (rimOut[j0] != nullptr) hes.push_back(rimOut[j0]);
            hes.push_back(along[i]);
            if (rimBack[j1] != nullptr) hes.push_back(rimBack[j1]);
            hes.push_back(back[i]);
            close(b.face, hes, true);
            if (b.kind == Sweeps::Plane) {
                const auto& plane = static_cast<const geo::Plane&>(*b.surface);
                setLinePcurve(along[i], plane.invert(vA[j0]->point), plane.invert(vA[j1]->point));
                setLinePcurve(back[i], plane.invert(v0[j1]->point), plane.invert(v0[j0]->point));
                continue;
            }
            const double u0 = uAt(0.0, b.sigma);
            const double uA = uAt(angle_, b.sigma);
            if (rimOut[j0] != nullptr) setLinePcurve(rimOut[j0], {u0, b.v0}, {uA, b.v0});
            setLinePcurve(along[i], {uA, b.v0}, {uA, b.v1});
            if (rimBack[j1] != nullptr) setLinePcurve(rimBack[j1], {uA, b.v1}, {u0, b.v1});
            setLinePcurve(back[i], {u0, b.v1}, {u0, b.v0});
        }
        if (!whole_) {
            std::vector<HalfEdge*> reversed(capEndHes.rbegin(), capEndHes.rend());
            close(capStart, capStartHes, outer);
            close(capEnd, reversed, outer);
        }

        // Edges: each corner's rim, and each curve at theta 0 and at the angle (one edge, the
        // seam, through a whole turn; the caps' edges through part of one).
        std::vector<Edge*> rims(m, nullptr);
        for (size_t j = 0; j < m; ++j) {
            if (onAxis[j]) continue;
            rims[j] = edge(rimOut[j], rimBack[j],
                           {std::make_shared<geo::CircleCurve>(rimFrame(corner[j].h), corner[j].r),
                            0.0, angle_});
        }
        for (size_t i = 0; i < m; ++i) {
            const exact::LoopCurve& c = loop.curves[i];
            const Band& b = bands[i];
            if (whole_) {
                if (b.seamed()) edge(along[i], back[i], c.curve(plane_, Vec3{}, true));
                continue;
            }
            if (b.kind == Sweeps::Nothing) {
                edge(capStartHes[i], capEndHes[i], c.curve(plane_, Vec3{}, true));
                continue;
            }
            edge(capStartHes[i], back[i], c.curve(plane_, Vec3{}, true));
            edge(along[i], capEndHes[i], c.curve(rotated_, Vec3{}, true));
        }
        for (size_t j = 0; j < m; ++j) {
            const double gap = exact::cornerGap(loop, j);
            if (gap > 0.0)
                gaps_.push_back(
                    {v0[j], vA[j] == v0[j] ? nullptr : vA[j], rims[j], gap, loop.curves[j].start});
        }
        return true;
    }

    topo::Solid& solid_;
    const draft::SketchPlane& plane_;
    draft::SketchPlane rotated_;  ///< the sketch's plane turned by the angle
    Vec3 axisPoint_;
    Vec3 axis_;
    Vec3 refDir_;  ///< from the axis toward the profile, at theta 0
    Vec3 yRot_;    ///< the way a point at theta 0 turns: axis x refDir
    bool whole_;
    double angle_;
    double flatTol_;  ///< master's: a line within it of square to or along the axis is
    double axisTol_;  ///< the model's resolution: a corner within it of the axis is on it
    bool mirrored_ = false;
    std::vector<Gap> gaps_;
};

/// The exact revolve of the regions @p found (which master turns) by @p angle about the axis
/// through @p axisPoint along @p axisDirection, or null with why it cannot be built exactly.
std::unique_ptr<topo::Solid> revolveExactly(const ProfileRegions& found,
                                            const draft::SketchPlane& plane, const Vec3& axisPoint,
                                            const Vec3& axisDirection, double angle,
                                            const std::string& featureID, std::string& why) {
    const Vec3 axis = axisDirection.normalized();
    if (!std::isfinite(axis.length()) || !std::isfinite(axisPoint.length()) ||
        !std::isfinite(angle)) {
        why = "its axis is not a finite line";
        return nullptr;
    }
    // Which side of the axis the whole profile lies to: each loop lies to one (master checked
    // that), and every loop to the same one.
    std::vector<Vec3> points;
    for (const ProfileRegion& region : found.regions) {
        auto outer = keyPoints(region.outer, plane, axisPoint, axis);
        points.insert(points.end(), outer.begin(), outer.end());
        for (const auto& hole : region.holes) {
            auto p = keyPoints(hole, plane, axisPoint, axis);
            points.insert(points.end(), p.begin(), p.end());
        }
    }
    double reach = 0.0;
    const Vec3 refDir = sideOf(points, axisPoint, axis, reach);
    if (!(reach > 0.0)) {
        why = "the profile has no extent off the axis";
        return nullptr;
    }
    double lowest = std::numeric_limits<double>::infinity();
    double highest = -lowest;
    for (const Vec3& p : points) {
        if (radialOf(p, axisPoint, axis).dot(refDir) < -kPlanarityTol * reach) {
            why = "its regions lie on both sides of the axis";
            return nullptr;
        }
        const double h = (p - axisPoint).dot(axis);
        lowest = std::min(lowest, h);
        highest = std::max(highest, h);
    }
    const math::ModelTolerance resolution =
        math::ModelTolerance::forExtent(std::hypot(2.0 * reach, highest - lowest));

    auto solid = std::make_unique<topo::Solid>();
    ExactRevolution build(*solid, plane, axisPoint, axis, refDir, angle, kPlanarityTol * reach,
                          resolution.linear);
    // Named as master named its faceted revolve's: the first region for the feature, the others
    // `<feature>~region<r>`, and each hole's faces `<feature>~hole<k>`, counted through the
    // regions.
    int holeCount = 0;
    for (size_t r = 0; r < found.regions.size(); ++r) {
        auto region = exact::regionOf(found.regions[r], &why);
        if (!region) return nullptr;
        if (build.mirrored()) {
            exact::reverse(region->outer);
            for (exact::Loop& hole : region->holes) exact::reverse(hole);
        }
        std::vector<std::string> holeSources;
        holeSources.reserve(region->holes.size());
        for (size_t k = 0; k < region->holes.size(); ++k) {
            holeSources.push_back(featureID + "~hole" + std::to_string(holeCount++));
        }
        if (!build.addRegion(*region,
                             r == 0 ? featureID : featureID + "~region" + std::to_string(r),
                             holeSources, why)) {
            return nullptr;
        }
    }
    if (!build.coverGaps(why)) return nullptr;
    nameEdgesLogically(*solid);
    return solid;
}

/// Points welded as SolidSewer::sew welds them, within its weld tolerance: a point of master's
/// ring on the axis, turned to each step, is one point however its rounding left it. Every other
/// point master's revolve shares between its polygons is one value, found again at once.
class Welder {
public:
    /// The index of @p p's point: the same value's, or one within the weld tolerance of it, or
    /// @p p's, added. @p p's coordinates must be finite.
    std::size_t at(const Vec3& p) {
        const Exact key{p.x, p.y, p.z};
        const auto same = exact_.find(key);
        if (same != exact_.end()) return same->second;
        const Cell k = cellOf(p);
        for (std::int64_t dx = -1; dx <= 1; ++dx) {
            for (std::int64_t dy = -1; dy <= 1; ++dy) {
                for (std::int64_t dz = -1; dz <= 1; ++dz) {
                    const auto found = cells_.find({k[0] + dx, k[1] + dy, k[2] + dz});
                    if (found == cells_.end()) continue;
                    for (const std::size_t i : found->second) {
                        if (points[i].distanceTo(p) <= SolidSewer::kDefaultWeldTol) {
                            exact_.emplace(key, i);
                            return i;
                        }
                    }
                }
            }
        }
        points.push_back(p);
        cells_[k].push_back(points.size() - 1);
        exact_.emplace(key, points.size() - 1);
        return points.size() - 1;
    }

    std::vector<Vec3> points;

private:
    using Exact = std::array<double, 3>;
    using Cell = std::array<std::int64_t, 3>;
    struct ExactHash {
        std::size_t operator()(const Exact& e) const {
            const std::hash<double> h;
            return (h(e[0]) * 1000003u ^ h(e[1])) * 1000003u ^ h(e[2]);
        }
    };
    struct CellHash {
        std::size_t operator()(const Cell& c) const {
            const std::hash<std::int64_t> h;
            return (h(c[0]) * 73856093u) ^ (h(c[1]) * 19349663u) ^ (h(c[2]) * 83492791u);
        }
    };
    static Cell cellOf(const Vec3& p) {
        const double cell = SolidSewer::kDefaultWeldTol;
        return {static_cast<std::int64_t>(std::floor(p.x / cell)),
                static_cast<std::int64_t>(std::floor(p.y / cell)),
                static_cast<std::int64_t>(std::floor(p.z / cell))};
    }
    std::unordered_map<Exact, std::size_t, ExactHash> exact_;
    std::unordered_map<Cell, std::vector<std::size_t>, CellHash> cells_;
};

/// Master's faceted revolve of @p found as it named what it made (Ruling 21): the polygons its
/// Stable revolveLoop sewed (revolveSoup at {segments, chordTolerance}) for each region's outer
/// loop and, for a profile of several loops, each hole's cutter (turned a twentieth further at
/// each end through part of a turn, as master's execute turned it), in order, each named as it
/// named the face sewn from it, its points welded as the sewer welded them; the edges in the order
/// the sewer met them in each. A profile of several loops was then master's Booleans: what they
/// split or merged has no place here, and keeps the bridge's own numbers (Ruling 23). Empty where
/// master refused it, or a point of it is not finite (or not within the welder's reach).
/// @p axisDir is a unit vector.
MastersFacets mastersRevolve(const ProfileRegions& found, const draft::SketchPlane& plane,
                             const Vec3& axisPoint, const Vec3& axisDir, double angle,
                             const std::string& featureID, int segments, double chordTolerance) {
    // The loops master turned, as its execute turned them.
    const bool fullTurn = std::abs(angle - 2.0 * math::kPi) <= 1e-9;
    const double extra = fullTurn ? 0.0 : 0.05 * angle;
    const double cutterAngle = std::min(angle + 2.0 * extra, 2.0 * math::kPi);
    const auto turned = [&](const Vec3& v) {
        return rotateAroundAxis(axisPoint + v, axisPoint, axisDir, -extra) - axisPoint;
    };
    const draft::SketchPlane cutterPlane(
        rotateAroundAxis(plane.origin(), axisPoint, axisDir, -extra), turned(plane.normal()),
        turned(plane.xAxis()));
    std::vector<RevolveSoup> soups;
    int holeCount = 0;
    for (size_t r = 0; r < found.regions.size(); ++r) {
        const ProfileRegion& region = found.regions[r];
        const std::string regionID = r == 0 ? featureID : featureID + "~region" + std::to_string(r);
        std::string why;
        auto soup = revolveSoup(region.outer, plane, axisPoint, axisDir, angle, regionID, segments,
                                chordTolerance, &why, NamingScheme::Stable);
        if (!soup) return {};
        soups.push_back(std::move(*soup));
        for (const auto& hole : region.holes) {
            auto cutter = revolveSoup(hole, cutterPlane, axisPoint, axisDir, cutterAngle,
                                      featureID + "~hole" + std::to_string(holeCount++), segments,
                                      chordTolerance, &why, NamingScheme::Stable);
            if (!cutter) return {};
            soups.push_back(std::move(*cutter));
        }
    }
    MastersFacets out;
    // A cell of the welder must be had for every point: its coordinates finite and within reach.
    const double reach = 1e15 * SolidSewer::kDefaultWeldTol;
    for (const RevolveSoup& soup : soups) {
        for (const auto& face : soup.faces) {
            for (const Vec3& p : face.points) {
                if (!(std::abs(p.x) < reach && std::abs(p.y) < reach && std::abs(p.z) < reach)) {
                    return out;
                }
            }
        }
    }
    Welder welder;
    const auto loopOf = [&welder](const std::vector<Vec3>& points) {
        std::vector<std::size_t> indices;
        indices.reserve(points.size());
        for (const Vec3& p : points) {
            const std::size_t i = welder.at(p);
            if (indices.empty() || indices.back() != i) indices.push_back(i);
        }
        while (indices.size() > 1 && indices.front() == indices.back()) indices.pop_back();
        return indices;
    };
    for (const RevolveSoup& soup : soups) {
        for (const auto& face : soup.faces) {
            std::vector<std::size_t> outer = loopOf(face.points);
            if (outer.size() < 3) continue;
            std::vector<std::vector<std::size_t>> holes;
            for (const auto& hole : face.holes) {
                std::vector<std::size_t> h = loopOf(hole);
                if (h.size() >= 3) holes.push_back(std::move(h));
            }
            out.faces.push_back(std::move(outer));
            out.holes.push_back(std::move(holes));
            out.faceNames.push_back(face.topoId.tag());
        }
    }
    out.points = std::move(welder.points);
    return out;
}

/// The exact revolve of @p found (which master turns) and its bridge copy at {@p segments,
/// @p chordTolerance}, @p segments round the axis (buildWithBridge; FacetResolution::aroundAxis),
/// numbered as master's revolve numbered the same places (mastersRevolve). @p axisDir is a unit
/// vector.
ExactAndBridge buildExactAndBridge(const ProfileRegions& found, const draft::SketchPlane& plane,
                                   const Vec3& axisPoint, const Vec3& axisDir, double angle,
                                   const std::string& featureID, int segments,
                                   double chordTolerance, std::string* reason) {
    FacetResolution resolution;
    resolution.segmentsPerTurn = segments;
    resolution.chordTolerance = chordTolerance;
    // Master turned every point of the profile through its segments at equal angles, whatever
    // the chord tolerance (Ruling 24): so is every circle and face about the axis cut.
    resolution.aroundAxis = segments;
    resolution.axisPoint = axisPoint;
    resolution.axisDirection = axisDir;
    const MastersConstruction masters = [&] {
        return mastersRevolve(found, plane, axisPoint, axisDir, angle, featureID, segments,
                              chordTolerance);
    };
    return buildWithBridge(
        [&](std::string& why) {
            return revolveExactly(found, plane, axisPoint, axisDir, angle, featureID, why);
        },
        resolution, masters, reason);
}

}  // namespace

std::unique_ptr<topo::Solid> Revolve::exact(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
    const draft::SketchPlane& plane, const Vec3& axisPoint, const Vec3& axisDirection, double angle,
    const std::string& featureID, std::string* reason) {
    const ProfileRegions found = ProfileValidator::regions(profile);
    auto refused = refusal(found, axisDirection, angle);
    if (!refused) refused = placementRefusal(found, plane, axisPoint, axisDirection.normalized());
    if (refused) {
        if (reason) *reason = std::move(*refused);
        return nullptr;
    }
    return buildExactly(
        [&](std::string& why) {
            return revolveExactly(found, plane, axisPoint, axisDirection, angle, featureID, why);
        },
        reason);
}

Revolve::ExactAndBridge Revolve::exactAndBridge(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
    const draft::SketchPlane& plane, const Vec3& axisPoint, const Vec3& axisDirection, double angle,
    const std::string& featureID, int segments, double chordTolerance, std::string* reason) {
    const auto fail = [reason](std::string why) {
        if (reason) *reason = std::move(why);
        return ExactAndBridge{};
    };
    if (segments < 3) return fail("a revolve needs at least 3 steps per turn");
    const ProfileRegions found = ProfileValidator::regions(profile);
    auto refused = refusal(found, axisDirection, angle);
    if (!refused) refused = placementRefusal(found, plane, axisPoint, axisDirection.normalized());
    if (refused) return fail(std::move(*refused));
    return buildExactAndBridge(found, plane, axisPoint, axisDirection.normalized(), angle,
                               featureID, segments, chordTolerance, reason);
}

}  // namespace hz::model
