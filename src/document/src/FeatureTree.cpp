#include "horizon/document/FeatureTree.h"

#include <algorithm>
#include <cassert>
#include <charconv>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <system_error>

#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Quaternion.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/ChamferOp.h"
#include "horizon/modeling/Draft.h"
#include "horizon/modeling/EdgeProjection.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/FacePlane.h"
#include "horizon/modeling/Faceting.h"
#include "horizon/modeling/FilletOp.h"
#include "horizon/modeling/Loft.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/modeling/Shell.h"
#include "horizon/modeling/Sweep.h"
#include "horizon/topology/GeometryValidator.h"

namespace hz::doc {

namespace {

/// Report `why` through a feature's optional reason out-parameter, and fail.
std::unique_ptr<topo::Solid> failWith(std::string* reason, std::string why) {
    if (reason) *reason = std::move(why);
    return nullptr;
}

/// A count parameter as an int. False for NaN, infinity or a value below
/// `min`; clamped to `max` above it. static_cast<int> of a double outside the
/// range of int is undefined behaviour, and a file can hold any double.
bool countParameter(double value, int min, int max, int& out) {
    if (!std::isfinite(value) || value < min) return false;
    out = value >= max ? max : static_cast<int>(value);
    return true;
}

}  // namespace

std::vector<std::string> Feature::parameterChoices(const std::string& name) const {
    if (name == "operation") return {"Union", "Subtract", "Intersect"};
    return {};
}

Feature::ParameterKind Feature::parameterKind(const std::string& name) const {
    if (name == "angle") return ParameterKind::Angle;
    if (name == "segments" || name == "arcSegments" || name == "count") return ParameterKind::Count;
    if (name == "operation") return ParameterKind::Choice;
    return ParameterKind::Length;
}

namespace {

bool finite(const math::Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

/// @p value as a unit direction, if it is one: finite and of some length.
std::optional<math::Vec3> unitDirection(const math::Vec3& value) {
    if (!finite(value) || !(value.length() > 1e-12)) return std::nullopt;
    return value.normalized();
}

}  // namespace

namespace {

// Accept a chord-sag budget: a non-negative finite distance, where 0 switches
// the feature back to its explicit facet count.
bool setChordTolerance(double& target, double value) {
    if (!(value >= 0.0) || !std::isfinite(value)) return false;
    target = value;
    return true;
}

// Keep the per-class ID counter ahead of restored IDs so future features
// never collide with loaded ones (mirrors the Sketch ID counter fix).
void bumpCounter(math::IdCounter<int>& counter, const std::string& id, const std::string& prefix) {
    if (id.rfind(prefix, 0) != 0) return;
    int n = 0;
    const char* digits = id.data() + prefix.size();
    if (std::from_chars(digits, id.data() + id.size(), n).ec == std::errc()) {
        counter.reserveThrough(n);
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// ExtrudeFeature
// ---------------------------------------------------------------------------

math::IdCounter<int> ExtrudeFeature::s_nextID{1};

ExtrudeFeature::ExtrudeFeature(std::shared_ptr<Sketch> sketch, const math::Vec3& direction,
                               double distance)
    : m_sketch(std::move(sketch)),
      m_direction(direction),
      m_distance(distance),
      m_featureID("extrude_" + std::to_string(s_nextID.next())) {}

std::string ExtrudeFeature::name() const {
    return "Extrude";
}

std::map<std::string, double> ExtrudeFeature::parameters() const {
    std::map<std::string, double> params = {{"distance", m_distance},
                                            {"extent", static_cast<double>(m_extent)}};
    if (hasCurvedProfile()) {
        params["segments"] = static_cast<double>(m_segments);
        params["chordTolerance"] = m_chordTolerance;
    }
    return params;
}

bool ExtrudeFeature::setParameter(const std::string& name, double value) {
    if (name == "distance" && value > 0.0) {
        m_distance = value;
        return true;
    }
    if (name == "extent") {
        int code = 0;
        if (!countParameter(value, 0, static_cast<int>(Extent::UpToFace), code)) return false;
        m_extent = static_cast<Extent>(code);
        return true;
    }
    // Profile arcs are faceted, so these decide how close the extrusion gets
    // to the exact solid.  A polygon profile is exact and has neither.
    if (name == "segments" && hasCurvedProfile()) {
        if (!countParameter(value, 3, kMaxFacetSegments, m_segments)) return false;
        m_chordTolerance = 0.0;
        return true;
    }
    if (name == "chordTolerance" && hasCurvedProfile()) {
        return setChordTolerance(m_chordTolerance, value);
    }
    return false;
}

Feature::ParameterKind ExtrudeFeature::parameterKind(const std::string& name) const {
    if (name == "extent") return ParameterKind::Choice;
    return Feature::parameterKind(name);
}

std::vector<std::string> ExtrudeFeature::parameterChoices(const std::string& name) const {
    if (name == "extent") {
        return {"To the distance", "Both ways, half each", "Through all", "Through all, both ways",
                "Up to a face"};
    }
    return Feature::parameterChoices(name);
}

bool ExtrudeFeature::setReference(const std::string& name, const std::string& value) {
    if (name != "upToFace") return false;
    m_upToFace = value;
    return true;
}

std::map<std::string, math::Vec3> ExtrudeFeature::vectors() const {
    return {{"direction", m_direction}};
}

bool ExtrudeFeature::setVector(const std::string& name, const math::Vec3& value) {
    if (name != "direction") return false;
    const auto unit = unitDirection(value);
    if (!unit) return false;
    // Along the sketch's own plane it sweeps nothing. Kept as the sketch was
    // drawn, and so checked against the plane it was drawn on.
    if (m_sketch && std::abs(unit->dot(m_sketch->drawnPlane().normal())) < 1e-9) return false;
    m_direction = *unit;
    return true;
}

bool ExtrudeFeature::hasCurvedProfile() const {
    if (!m_sketch) return false;
    for (const auto& ent : m_sketch->entities()) {
        if (ent->construction()) continue;  // not part of the profile
        if (dynamic_cast<const draft::DraftArc*>(ent.get()) ||
            dynamic_cast<const draft::DraftCircle*>(ent.get())) {
            return true;
        }
    }
    return false;
}

std::string ExtrudeFeature::featureID() const {
    return m_featureID;
}

void ExtrudeFeature::restoreFeatureID(const std::string& id) {
    if (id.empty()) return;
    m_featureID = id;
    bumpCounter(s_nextID, id, "extrude_");
}

std::unique_ptr<topo::Solid> ExtrudeFeature::execute(std::unique_ptr<topo::Solid> inputSolid,
                                                     std::string* reason) const {
    return executeIn(BuildContext{}, std::move(inputSolid), reason);
}

std::unique_ptr<topo::Solid> ExtrudeFeature::executeIn(const BuildContext& context,
                                                       std::unique_ptr<topo::Solid> /*inputSolid*/,
                                                       std::string* reason) const {
    // The extrusion alone; the tree combines it with the part according to
    // operation() (see applyFeature), as for every body-creating feature.
    const draft::SketchPlane& plane = m_sketch->plane();
    // Placed on a face that has turned, the sketch takes its direction along.
    const math::Vec3 direction = m_sketch->placement().transformDirection(m_direction);
    const auto extrude = [&](const draft::SketchPlane& from, double distance) {
        return model::Extrude::execute(m_sketch->entities(), from, direction, distance, m_featureID,
                                       m_segments, m_chordTolerance, reason, naming());
    };
    // Along the direction, a plane the sketch's, moved @p by.
    const auto moved = [&plane](const math::Vec3& by) {
        return draft::SketchPlane(plane.origin() + by, plane.normal(), plane.xAxis());
    };
    const math::Vec3 unit = direction.normalized();
    switch (m_extent) {
        case Extent::Blind:
            break;
        case Extent::Symmetric:
            return extrude(moved(unit * (-m_distance / 2.0)), m_distance);
        case Extent::ThroughAll:
        case Extent::ThroughAllBoth: {
            if (!context.part || context.part->vertices().empty()) {
                return failWith(reason, "there is no part before it to go through");
            }
            // How far the part reaches along the direction, from the
            // sketch, each way, and a little past it at each end.
            double lo = std::numeric_limits<double>::infinity();
            double hi = -lo;
            for (const auto& v : context.part->vertices()) {
                const double along = (v.point - plane.origin()).dot(unit);
                lo = std::min(lo, along);
                hi = std::max(hi, along);
            }
            const double margin = 0.01 * (hi - lo) + 1e-6;
            if (m_extent == Extent::ThroughAll) {
                if (hi <= 1e-9) {
                    return failWith(reason, "the part is not in front of the sketch that way");
                }
                return extrude(plane, hi + margin);
            }
            const double back = std::min(lo, 0.0) - margin;
            return extrude(moved(unit * back), std::max(hi, 0.0) + margin - back);
        }
        case Extent::UpToFace: {
            // Phase 157: as far as a flat face of the part, parallel to the
            // sketch, where the part now puts it.
            if (m_upToFace.empty()) return failWith(reason, "no face is chosen to go up to");
            const std::string face = "the face it goes up to (" + m_upToFace + ")";
            if (!context.part) {
                return failWith(reason, face + " is not there: there is no part before it");
            }
            std::string why;
            const auto to = model::planeOfFace(*context.part, m_upToFace, &why);
            if (!to) return failWith(reason, face + " " + why);
            if (to->normal.cross(plane.normal()).length() > 1e-9) {
                return failWith(reason, face +
                                            " is at a slant to the sketch: it goes up to a face "
                                            "parallel to the sketch");
            }
            // Parallel, so every point of the profile is as far from it.
            const double reach =
                (to->origin - plane.origin()).dot(to->normal) / unit.dot(to->normal);
            const double size = std::max(1.0, (to->origin - plane.origin()).length());
            if (!(reach > 1e-9 * size)) {
                return failWith(reason, face + " is not in front of the sketch that way");
            }
            return extrude(plane, reach / direction.length());
        }
    }
    return extrude(plane, m_distance);
}

// ---------------------------------------------------------------------------
// RevolveFeature
// ---------------------------------------------------------------------------

math::IdCounter<int> RevolveFeature::s_nextID{1};

RevolveFeature::RevolveFeature(std::shared_ptr<Sketch> sketch, const math::Vec3& axisPoint,
                               const math::Vec3& axisDir, double angle)
    : m_sketch(std::move(sketch)),
      m_axisPoint(axisPoint),
      m_axisDir(axisDir),
      m_angle(angle),
      m_featureID("revolve_" + std::to_string(s_nextID.next())) {}

std::string RevolveFeature::name() const {
    return "Revolve";
}

std::map<std::string, double> RevolveFeature::parameters() const {
    return {{"angle", m_angle},
            {"segments", static_cast<double>(segments())},
            {"chordTolerance", m_chordTolerance}};
}

bool RevolveFeature::setParameter(const std::string& name, double value) {
    if (name == "angle" && value > 0.0) {
        m_angle = value;
        return true;
    }
    // A revolve is faceted, so the step count decides how close its volume
    // gets to the exact one.  Three steps is the fewest that bounds a volume.
    if (name == "segments") {
        if (!countParameter(value, 3, kMaxFacetSegments, m_segments)) return false;
        m_chordTolerance = 0.0;
        return true;
    }
    if (name == "chordTolerance") return setChordTolerance(m_chordTolerance, value);
    return false;
}

std::map<std::string, math::Vec3> RevolveFeature::vectors() const {
    return {{"axisPoint", m_axisPoint}, {"axisDirection", m_axisDir}};
}

bool RevolveFeature::setVector(const std::string& name, const math::Vec3& value) {
    if (name == "axisPoint" && finite(value)) {
        m_axisPoint = value;
        return true;
    }
    if (name != "axisDirection") return false;
    const auto unit = unitDirection(value);
    if (!unit) return false;
    m_axisDir = *unit;
    return true;
}

int RevolveFeature::segments() const {
    if (m_chordTolerance > 0.0 && m_sketch) {
        // As drawn: the axis as it is kept, with the plane it was drawn on.
        const double radius = model::Revolve::profileRadius(
            m_sketch->entities(), m_sketch->drawnPlane(), m_axisPoint, m_axisDir);
        if (radius > 0.0) return model::Revolve::segmentsForTolerance(radius, m_chordTolerance);
    }
    return m_segments;
}

std::string RevolveFeature::featureID() const {
    return m_featureID;
}

void RevolveFeature::restoreFeatureID(const std::string& id) {
    if (id.empty()) return;
    m_featureID = id;
    bumpCounter(s_nextID, id, "revolve_");
}

std::unique_ptr<topo::Solid> RevolveFeature::execute(std::unique_ptr<topo::Solid> /*inputSolid*/,
                                                     std::string* reason) const {
    // Placed on a face that has moved, the sketch takes its axis along.
    const math::Mat4 placement = m_sketch->placement();
    return model::Revolve::execute(m_sketch->entities(), m_sketch->plane(),
                                   placement.transformPoint(m_axisPoint),
                                   placement.transformDirection(m_axisDir), m_angle, m_featureID,
                                   segments(), m_chordTolerance, reason, naming());
}

// ---------------------------------------------------------------------------
// LoftFeature
// ---------------------------------------------------------------------------

math::IdCounter<int> LoftFeature::s_nextID{1};

LoftFeature::LoftFeature(std::vector<std::shared_ptr<Sketch>> sections)
    : m_sections(std::move(sections)), m_featureID("loft_" + std::to_string(s_nextID.next())) {}

std::string LoftFeature::name() const {
    return "Loft";
}

std::string LoftFeature::featureID() const {
    return m_featureID;
}

void LoftFeature::restoreFeatureID(const std::string& id) {
    if (id.empty()) return;
    m_featureID = id;
    bumpCounter(s_nextID, id, "loft_");
}

std::unique_ptr<topo::Solid> LoftFeature::execute(std::unique_ptr<topo::Solid> /*inputSolid*/,
                                                  std::string* reason) const {
    std::vector<model::LoftSection> sections;
    sections.reserve(m_sections.size());
    for (const auto& sk : m_sections) {
        if (!sk) return failWith(reason, "a section's sketch is missing");
        sections.push_back({sk->entities(), sk->plane()});
    }
    std::string why;
    auto solid = model::Loft::execute(sections, m_featureID, model::Loft::kDefaultTwistSegments,
                                      &why, naming());
    if (!solid) return failWith(reason, why);
    return solid;
}

// ---------------------------------------------------------------------------
// SweepFeature
// ---------------------------------------------------------------------------

math::IdCounter<int> SweepFeature::s_nextID{1};

SweepFeature::SweepFeature(std::shared_ptr<Sketch> profile, std::shared_ptr<Sketch> path)
    : m_profile(std::move(profile)),
      m_path(std::move(path)),
      m_featureID("sweep_" + std::to_string(s_nextID.next())) {}

std::string SweepFeature::name() const {
    return "Sweep";
}

std::string SweepFeature::featureID() const {
    return m_featureID;
}

void SweepFeature::restoreFeatureID(const std::string& id) {
    if (id.empty()) return;
    m_featureID = id;
    bumpCounter(s_nextID, id, "sweep_");
}

namespace {

// Extract an open polyline of 3D path points from a path sketch: chain the
// endpoints of its line/arc entities, or expand a single polyline entity.
// Points along arc @p arc from its start to its end, excluding the start,
// at @p segmentsPerTurn steps per full turn (at least one step).
std::vector<math::Vec2> sampleArc(const draft::DraftArc& arc, int segmentsPerTurn) {
    const double sweep = arc.sweepAngle();
    const int steps =
        std::max(1, static_cast<int>(std::ceil(sweep / math::kTwoPi * segmentsPerTurn - 1e-9)));
    std::vector<math::Vec2> pts;
    pts.reserve(static_cast<size_t>(steps));
    for (int i = 1; i <= steps; ++i) {
        // Land the last sample on the stored end point exactly, so it welds
        // with whatever entity follows the arc.
        if (i == steps) {
            pts.push_back(arc.endPoint());
            break;
        }
        const double a = arc.startAngle() + sweep * static_cast<double>(i) / steps;
        pts.emplace_back(arc.center().x + arc.radius() * std::cos(a),
                         arc.center().y + arc.radius() * std::sin(a));
    }
    return pts;
}

std::vector<math::Vec3> extractPathPoints(const Sketch& sketch, int arcSegmentsPerTurn,
                                          double chordTolerance) {
    const auto& plane = sketch.plane();
    std::vector<math::Vec2> pts2D;

    for (const auto& ent : sketch.entities()) {
        if (ent->construction()) continue;  // guides the drawing; not the path
        if (auto* pl = dynamic_cast<const draft::DraftPolyline*>(ent.get())) {
            for (const auto& p : pl->points()) pts2D.push_back(p);
            continue;
        }
        math::Vec2 s, e;
        std::vector<math::Vec2> interior;  // points after s, ending at e
        if (auto* line = dynamic_cast<const draft::DraftLine*>(ent.get())) {
            s = line->start();
            e = line->end();
            interior.push_back(e);
        } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(ent.get())) {
            // An arc is followed along its curve, not across its chord.
            s = arc->startPoint();
            e = arc->endPoint();
            // With a tolerance each arc gets the count its own radius needs.
            const int perTurn =
                chordTolerance > 0.0
                    ? model::PrimitiveFactory::segmentsForTolerance(arc->radius(), chordTolerance)
                    : arcSegmentsPerTurn;
            interior = sampleArc(*arc, perTurn);
        } else {
            continue;
        }
        if (pts2D.empty()) {
            pts2D.push_back(s);
        } else {
            const double ds = (pts2D.back() - s).length();
            const double de = (pts2D.back() - e).length();
            if (de < ds) {
                // Traversed end to start: walk the samples backwards.
                interior.pop_back();
                std::reverse(interior.begin(), interior.end());
                interior.push_back(s);
            }
        }
        pts2D.insert(pts2D.end(), interior.begin(), interior.end());
    }

    std::vector<math::Vec3> pts3D;
    pts3D.reserve(pts2D.size());
    for (const auto& p : pts2D) pts3D.push_back(plane.localToWorld(p));
    return pts3D;
}

}  // namespace

std::map<std::string, double> SweepFeature::parameters() const {
    return {{"segments", static_cast<double>(m_segments)}, {"chordTolerance", m_chordTolerance}};
}

bool SweepFeature::setParameter(const std::string& name, double value) {
    // Arcs in the path are swept as mitered chords, so this decides how close
    // a curved sweep gets to the exact one.  Three steps per turn is the
    // fewest that still turns a full circle.
    if (name == "segments") {
        if (!countParameter(value, 3, kMaxFacetSegments, m_segments)) return false;
        m_chordTolerance = 0.0;
        return true;
    }
    if (name == "chordTolerance") return setChordTolerance(m_chordTolerance, value);
    return false;
}

std::unique_ptr<topo::Solid> SweepFeature::execute(std::unique_ptr<topo::Solid> /*inputSolid*/,
                                                   std::string* reason) const {
    if (!m_profile || !m_path) return failWith(reason, "the profile or path sketch is missing");
    std::vector<math::Vec3> pathPoints = extractPathPoints(*m_path, m_segments, m_chordTolerance);
    std::string why;
    auto solid = model::Sweep::execute(m_profile->entities(), m_profile->plane(), pathPoints,
                                       m_featureID, m_segments, m_chordTolerance, &why, naming());
    if (!solid) return failWith(reason, why);
    return solid;
}

// ---------------------------------------------------------------------------
// DraftFeature
// ---------------------------------------------------------------------------

math::IdCounter<int> DraftFeature::s_nextID{1};

DraftFeature::DraftFeature(const math::Vec3& pullDir, const math::Vec3& neutralPoint, double angle)
    : m_pullDir(pullDir),
      m_neutralPoint(neutralPoint),
      m_angle(angle),
      m_featureID("draft_" + std::to_string(s_nextID.next())) {}

std::string DraftFeature::name() const {
    return "Draft";
}

std::string DraftFeature::featureID() const {
    return m_featureID;
}

void DraftFeature::restoreFeatureID(const std::string& id) {
    if (id.empty()) return;
    m_featureID = id;
    bumpCounter(s_nextID, id, "draft_");
}

std::map<std::string, double> DraftFeature::parameters() const {
    return {{"angle", m_angle}};
}

bool DraftFeature::setParameter(const std::string& name, double value) {
    if (name == "angle") {
        m_angle = value;
        return true;
    }
    return false;
}

std::map<std::string, math::Vec3> DraftFeature::vectors() const {
    return {{"pullDirection", m_pullDir}, {"neutralPoint", m_neutralPoint}};
}

bool DraftFeature::setVector(const std::string& name, const math::Vec3& value) {
    if (name == "neutralPoint" && finite(value)) {
        m_neutralPoint = value;
        return true;
    }
    if (name != "pullDirection") return false;
    const auto unit = unitDirection(value);
    if (!unit) return false;
    m_pullDir = *unit;
    return true;
}

std::unique_ptr<topo::Solid> DraftFeature::execute(std::unique_ptr<topo::Solid> inputSolid,
                                                   std::string* reason) const {
    if (!inputSolid) return failWith(reason, "there is no body to draft");
    auto solid = model::Draft::execute(std::move(inputSolid), m_pullDir, m_neutralPoint, m_angle);
    if (!solid) return failWith(reason, "the draft could not be applied");
    return solid;
}

// ---------------------------------------------------------------------------
// ShellFeature
// ---------------------------------------------------------------------------

math::IdCounter<int> ShellFeature::s_nextID{1};

ShellFeature::ShellFeature(double thickness, std::vector<topo::TopologyID> removedFaceIds)
    : m_thickness(thickness),
      m_removedFaceIds(std::move(removedFaceIds)),
      m_featureID("shell_" + std::to_string(s_nextID.next())) {}

std::string ShellFeature::name() const {
    return "Shell";
}

std::string ShellFeature::featureID() const {
    return m_featureID;
}

void ShellFeature::restoreFeatureID(const std::string& id) {
    if (id.empty()) return;
    m_featureID = id;
    bumpCounter(s_nextID, id, "shell_");
}

std::map<std::string, double> ShellFeature::parameters() const {
    return {{"thickness", m_thickness}};
}

bool ShellFeature::setParameter(const std::string& name, double value) {
    if (name == "thickness" && value > 0.0) {
        m_thickness = value;
        return true;
    }
    return false;
}

std::unique_ptr<topo::Solid> ShellFeature::execute(std::unique_ptr<topo::Solid> inputSolid,
                                                   std::string* reason) const {
    if (!inputSolid) return failWith(reason, "there is no body to shell");
    auto result = model::Shell::execute(std::move(inputSolid), m_thickness, m_removedFaceIds);
    if (!result.ok) return failWith(reason, result.message);
    if (naming() == model::NamingScheme::Stable && result.solid) {
        // This shell's names, not every shell's (`shell/rim_0`); edges after
        // their faces, not their storage order.
        model::scopeToFeature(*result.solid, featureID());
        model::nameEdgesLogically(*result.solid);
    }
    return std::move(result.solid);
}

// ---------------------------------------------------------------------------
// FilletFeature
// ---------------------------------------------------------------------------

math::IdCounter<int> FilletFeature::s_nextID{1};

FilletFeature::FilletFeature(std::vector<topo::TopologyID> edgeIds, double radius)
    : m_edgeIds(std::move(edgeIds)),
      m_radius(radius),
      m_featureID("fillet_" + std::to_string(s_nextID.next())) {}

std::string FilletFeature::name() const {
    return "Fillet";
}

std::string FilletFeature::featureID() const {
    return m_featureID;
}

void FilletFeature::restoreFeatureID(const std::string& id) {
    if (id.empty()) return;
    m_featureID = id;
    bumpCounter(s_nextID, id, "fillet_");
}

std::map<std::string, double> FilletFeature::parameters() const {
    return {{"radius", m_radius},
            {"arcSegments", static_cast<double>(arcSegments())},
            {"chordTolerance", m_chordTolerance}};
}

bool FilletFeature::setParameter(const std::string& name, double value) {
    if (name == "radius" && value > 0.0) {
        m_radius = value;
        return true;
    }
    // Blends are faceted across the arc; one chord is the degenerate case that
    // removes a chamfer's worth of material rather than a fillet's.
    if (name == "arcSegments") {
        if (!countParameter(value, 1, kMaxArcSegments, m_arcSegments)) return false;
        m_chordTolerance = 0.0;
        return true;
    }
    if (name == "chordTolerance") return setChordTolerance(m_chordTolerance, value);
    return false;
}

int FilletFeature::arcSegments() const {
    return m_chordTolerance > 0.0
               ? model::FilletOp::arcSegmentsForTolerance(m_radius, m_chordTolerance)
               : m_arcSegments;
}

std::unique_ptr<topo::Solid> FilletFeature::execute(std::unique_ptr<topo::Solid> inputSolid,
                                                    std::string* reason) const {
    if (!inputSolid) return failWith(reason, "there is no body to fillet");
    auto result = model::FilletOp::execute(*inputSolid, m_edgeIds, m_radius, m_featureID,
                                           arcSegments(), naming());
    if (!result.solid) {
        return failWith(reason, result.errorMessage.empty() ? "the fillet could not be built"
                                                            : result.errorMessage);
    }
    return std::move(result.solid);
}

// ---------------------------------------------------------------------------
// ChamferFeature
// ---------------------------------------------------------------------------

math::IdCounter<int> ChamferFeature::s_nextID{1};

ChamferFeature::ChamferFeature(std::vector<topo::TopologyID> edgeIds, double distance)
    : m_edgeIds(std::move(edgeIds)),
      m_distance(distance),
      m_featureID("chamfer_" + std::to_string(s_nextID.next())) {}

std::string ChamferFeature::name() const {
    return "Chamfer";
}

std::string ChamferFeature::featureID() const {
    return m_featureID;
}

void ChamferFeature::restoreFeatureID(const std::string& id) {
    if (id.empty()) return;
    m_featureID = id;
    bumpCounter(s_nextID, id, "chamfer_");
}

std::map<std::string, double> ChamferFeature::parameters() const {
    return {{"distance", m_distance}};
}

bool ChamferFeature::setParameter(const std::string& name, double value) {
    if (name == "distance" && value > 0.0) {
        m_distance = value;
        return true;
    }
    return false;
}

std::unique_ptr<topo::Solid> ChamferFeature::execute(std::unique_ptr<topo::Solid> inputSolid,
                                                     std::string* reason) const {
    if (!inputSolid) return failWith(reason, "there is no body to chamfer");
    auto result = model::ChamferOp::executeEqual(*inputSolid, m_edgeIds, m_distance, m_featureID);
    if (!result.solid) {
        return failWith(reason, result.errorMessage.empty() ? "the chamfer could not be built"
                                                            : result.errorMessage);
    }
    // The edges it did not touch keep their names (the sewer named them all
    // afresh, in the order it met them).
    if (naming() == model::NamingScheme::Stable) {
        model::nameBlendFaces(*result.solid, m_featureID + "/chamfer/");
        model::keepEdgeNames(*result.solid, *inputSolid);
    }
    return std::move(result.solid);
}

// ---------------------------------------------------------------------------
// BooleanFeature
// ---------------------------------------------------------------------------

math::IdCounter<int> BooleanFeature::s_nextID{1};

BooleanFeature::BooleanFeature(model::BooleanType type)
    : m_type(type), m_featureID("boolean_" + std::to_string(s_nextID.next())) {}

std::string BooleanFeature::name() const {
    switch (m_type) {
        case model::BooleanType::Union:
            return "Boolean Union";
        case model::BooleanType::Subtract:
            return "Boolean Subtract";
        case model::BooleanType::Intersect:
            return "Boolean Intersect";
    }
    return "Boolean";
}

std::string BooleanFeature::featureID() const {
    return m_featureID;
}

void BooleanFeature::restoreFeatureID(const std::string& id) {
    if (id.empty()) return;
    m_featureID = id;
    bumpCounter(s_nextID, id, "boolean_");
}

std::map<std::string, double> BooleanFeature::parameters() const {
    return {{"operation", static_cast<double>(static_cast<int>(m_type))}};
}

bool BooleanFeature::setParameter(const std::string& name, double value) {
    if (name == "operation") {
        int v = 0;
        if (countParameter(value, 0, 2, v) && value <= 2.0) {
            m_type = static_cast<model::BooleanType>(v);
            return true;
        }
    }
    return false;
}

std::unique_ptr<topo::Solid> BooleanFeature::execute(std::unique_ptr<topo::Solid> inputSolid,
                                                     std::string* reason) const {
    // The part's bodies are its shells (see BodyOperation::NewBody). Fewer
    // than two leave nothing to combine, and the part passes through.
    if (!inputSolid) return failWith(reason, "there are no bodies to combine");
    auto bodies = model::Pattern::separate(*inputSolid);
    if (bodies.size() < 2) return inputSolid;

    // Fold in body order: for Subtract the first body is the one cut from.
    auto result = std::move(bodies[0]);
    for (size_t i = 1; i < bodies.size(); ++i) {
        std::string why;
        auto combined = model::BooleanOp::execute(*result, *bodies[i], m_type, &why, naming());
        if (!combined) {
            return failWith(reason, "combining body " + std::to_string(i + 1) + " failed: " + why);
        }
        result = std::move(combined);
    }
    return result;
}

std::vector<std::unique_ptr<topo::Solid>> BooleanFeature::executeMulti(
    std::vector<std::unique_ptr<topo::Solid>> bodies) const {
    if (bodies.size() < 2) {
        return bodies;  // nothing to combine
    }

    // Fold left-to-right: the first body is the running result (for Subtract it
    // is the target), each later body is combined into it.
    auto accumulator = std::move(bodies[0]);
    for (size_t i = 1; i < bodies.size(); ++i) {
        if (!accumulator || !bodies[i]) continue;
        auto combined = model::BooleanOp::execute(*accumulator, *bodies[i], m_type);
        if (combined) {
            accumulator = std::move(combined);
        }
        // If the op fails, keep the current accumulator and skip this operand.
    }

    std::vector<std::unique_ptr<topo::Solid>> result;
    if (accumulator) {
        result.push_back(std::move(accumulator));
    }
    return result;
}

// ---------------------------------------------------------------------------
// PatternFeature
// ---------------------------------------------------------------------------

math::IdCounter<int> PatternFeature::s_nextID{1};

std::unique_ptr<PatternFeature> PatternFeature::makeLinear(const math::Vec3& direction,
                                                           double spacing, int count,
                                                           std::vector<int> suppressed) {
    std::unique_ptr<PatternFeature> f(new PatternFeature());
    f->m_kind = Kind::Linear;
    f->m_vecA = direction;
    f->m_scalar = spacing;
    f->m_count = std::clamp(count, 1, kMaxPatternCount);
    f->m_suppressed = std::move(suppressed);
    f->m_featureID = "pattern_" + std::to_string(s_nextID.next());
    return f;
}

std::unique_ptr<PatternFeature> PatternFeature::makeCircular(const math::Vec3& axisPoint,
                                                             const math::Vec3& axisDir,
                                                             double angleStepRad, int count,
                                                             std::vector<int> suppressed) {
    std::unique_ptr<PatternFeature> f(new PatternFeature());
    f->m_kind = Kind::Circular;
    f->m_vecA = axisPoint;
    f->m_vecB = axisDir;
    f->m_scalar = angleStepRad;
    f->m_count = std::clamp(count, 1, kMaxPatternCount);
    f->m_suppressed = std::move(suppressed);
    f->m_featureID = "pattern_" + std::to_string(s_nextID.next());
    return f;
}

std::string PatternFeature::name() const {
    return m_kind == Kind::Linear ? "LinearPattern" : "CircularPattern";
}

std::string PatternFeature::featureID() const {
    return m_featureID;
}

void PatternFeature::restoreFeatureID(const std::string& id) {
    if (id.empty()) return;
    m_featureID = id;
    bumpCounter(s_nextID, id, "pattern_");
}

std::map<std::string, double> PatternFeature::parameters() const {
    return {{"count", static_cast<double>(m_count)}, {"spacing", m_scalar}};
}

bool PatternFeature::setParameter(const std::string& name, double value) {
    if (name == "count") return countParameter(value, 1, kMaxPatternCount, m_count);
    if (name == "spacing") {
        m_scalar = value;
        return true;
    }
    return false;
}

Feature::ParameterKind PatternFeature::parameterKind(const std::string& name) const {
    // A circular pattern's step is the angle between copies.
    if (name == "spacing" && m_kind == Kind::Circular) return ParameterKind::Angle;
    return Feature::parameterKind(name);
}

std::map<std::string, math::Vec3> PatternFeature::vectors() const {
    if (m_kind == Kind::Circular) return {{"axisPoint", m_vecA}, {"axisDirection", m_vecB}};
    return {{"direction", m_vecA}};
}

bool PatternFeature::setVector(const std::string& name, const math::Vec3& value) {
    if (m_kind == Kind::Circular && name == "axisPoint" && finite(value)) {
        m_vecA = value;
        return true;
    }
    const bool direction = m_kind == Kind::Circular ? name == "axisDirection" : name == "direction";
    if (!direction) return false;
    const auto unit = unitDirection(value);
    if (!unit) return false;
    (m_kind == Kind::Circular ? m_vecB : m_vecA) = *unit;
    return true;
}

std::unique_ptr<topo::Solid> PatternFeature::execute(std::unique_ptr<topo::Solid> inputSolid,
                                                     std::string* reason) const {
    if (!inputSolid) return failWith(reason, "there is no body to pattern");
    // Instances that meet are joined at Stable names under Stable; older
    // documents' patterns join by position, as their references expect.
    const model::NamingScheme join = naming() == model::NamingScheme::Stable
                                         ? model::NamingScheme::Stable
                                         : model::NamingScheme::Positional;
    auto solid =
        m_kind == Kind::Linear
            ? model::Pattern::linear(*inputSolid, m_vecA, m_scalar, m_count, m_suppressed, join)
            : model::Pattern::circular(*inputSolid, m_vecA, m_vecB, m_scalar, m_count, m_suppressed,
                                       join);
    if (!solid) return failWith(reason, "overlapping instances could not be merged into one body");
    return solid;
}

// ---------------------------------------------------------------------------
// PrimitiveFeature
// ---------------------------------------------------------------------------

math::IdCounter<int> PrimitiveFeature::s_nextID{1};

std::unique_ptr<PrimitiveFeature> PrimitiveFeature::makeBox(double width, double height,
                                                            double depth) {
    std::unique_ptr<PrimitiveFeature> f(new PrimitiveFeature());
    f->m_kind = Kind::Box;
    f->m_p0 = width;
    f->m_p1 = height;
    f->m_p2 = depth;
    f->m_featureID = "primitive_" + std::to_string(s_nextID.next());
    return f;
}
std::unique_ptr<PrimitiveFeature> PrimitiveFeature::makeCylinder(double radius, double height) {
    std::unique_ptr<PrimitiveFeature> f(new PrimitiveFeature());
    f->m_kind = Kind::Cylinder;
    f->m_p0 = radius;
    f->m_p1 = height;
    f->m_featureID = "primitive_" + std::to_string(s_nextID.next());
    return f;
}
std::unique_ptr<PrimitiveFeature> PrimitiveFeature::makeSphere(double radius) {
    std::unique_ptr<PrimitiveFeature> f(new PrimitiveFeature());
    f->m_kind = Kind::Sphere;
    f->m_p0 = radius;
    f->m_featureID = "primitive_" + std::to_string(s_nextID.next());
    return f;
}
std::unique_ptr<PrimitiveFeature> PrimitiveFeature::makeCone(double bottomRadius, double topRadius,
                                                             double height) {
    std::unique_ptr<PrimitiveFeature> f(new PrimitiveFeature());
    f->m_kind = Kind::Cone;
    f->m_p0 = bottomRadius;
    f->m_p1 = topRadius;
    f->m_p2 = height;
    f->m_featureID = "primitive_" + std::to_string(s_nextID.next());
    return f;
}
std::unique_ptr<PrimitiveFeature> PrimitiveFeature::makeTorus(double majorRadius,
                                                              double minorRadius) {
    std::unique_ptr<PrimitiveFeature> f(new PrimitiveFeature());
    f->m_kind = Kind::Torus;
    f->m_p0 = majorRadius;
    f->m_p1 = minorRadius;
    f->m_featureID = "primitive_" + std::to_string(s_nextID.next());
    return f;
}

std::string PrimitiveFeature::name() const {
    switch (m_kind) {
        case Kind::Box:
            return "Box";
        case Kind::Cylinder:
            return "Cylinder";
        case Kind::Sphere:
            return "Sphere";
        case Kind::Cone:
            return "Cone";
        case Kind::Torus:
            return "Torus";
    }
    return "Primitive";
}

std::string PrimitiveFeature::featureID() const {
    return m_featureID;
}

void PrimitiveFeature::restoreFeatureID(const std::string& id) {
    if (id.empty()) return;
    m_featureID = id;
    bumpCounter(s_nextID, id, "primitive_");
}

std::map<std::string, double> PrimitiveFeature::parameters() const {
    std::map<std::string, double> params;
    switch (m_kind) {
        case Kind::Box:
            params = {{"width", m_p0}, {"height", m_p1}, {"depth", m_p2}};
            break;
        case Kind::Cylinder:
            params = {{"radius", m_p0}, {"height", m_p1}};
            break;
        case Kind::Sphere:
            params = {{"radius", m_p0}};
            break;
        case Kind::Cone:
            params = {{"bottomRadius", m_p0}, {"topRadius", m_p1}, {"height", m_p2}};
            break;
        case Kind::Torus:
            params = {{"majorRadius", m_p0}, {"minorRadius", m_p1}};
            break;
    }
    // A box is exact; the curved kinds are tessellated at construction, so the
    // facet count is a real parameter of the shape rather than a display
    // setting.  Only they report it.
    if (isFaceted()) {
        params["segments"] = static_cast<double>(segments());
        params["chordTolerance"] = m_chordTolerance;
    }
    return params;
}

bool PrimitiveFeature::setParameter(const std::string& name, double value) {
    if (name == "segments") {
        if (!isFaceted() || !countParameter(value, 3, kMaxFacetSegments, m_segments)) {
            return false;
        }
        m_chordTolerance = 0.0;
        return true;
    }
    if (name == "chordTolerance") {
        return isFaceted() && setChordTolerance(m_chordTolerance, value);
    }
    switch (m_kind) {
        case Kind::Box:
            if (name == "width") return (m_p0 = value, true);
            if (name == "height") return (m_p1 = value, true);
            if (name == "depth") return (m_p2 = value, true);
            break;
        case Kind::Cylinder:
            if (name == "radius") return (m_p0 = value, true);
            if (name == "height") return (m_p1 = value, true);
            break;
        case Kind::Sphere:
            if (name == "radius") return (m_p0 = value, true);
            break;
        case Kind::Cone:
            if (name == "bottomRadius") return (m_p0 = value, true);
            if (name == "topRadius") return (m_p1 = value, true);
            if (name == "height") return (m_p2 = value, true);
            break;
        case Kind::Torus:
            if (name == "majorRadius") return (m_p0 = value, true);
            if (name == "minorRadius") return (m_p1 = value, true);
            break;
    }
    return false;
}

int PrimitiveFeature::segments() const {
    if (!(m_chordTolerance > 0.0)) return m_segments;
    // The widest circle the facets approximate carries the largest sag.
    double radius = m_p0;
    if (m_kind == Kind::Cone) radius = std::max(m_p0, m_p1);
    if (m_kind == Kind::Torus) radius = m_p0 + m_p1;
    return model::PrimitiveFactory::segmentsForTolerance(radius, m_chordTolerance);
}

std::unique_ptr<topo::Solid> PrimitiveFeature::execute(std::unique_ptr<topo::Solid> /*inputSolid*/,
                                                       std::string* reason) const {
    const int n = segments();
    std::unique_ptr<topo::Solid> solid;
    switch (m_kind) {
        case Kind::Box:
            solid = model::PrimitiveFactory::makeBox(m_p0, m_p1, m_p2);
            break;
        case Kind::Cylinder:
            solid = model::PrimitiveFactory::makeCylinder(m_p0, m_p1, n);
            break;
        case Kind::Sphere:
            solid = model::PrimitiveFactory::makeSphere(m_p0, n);
            break;
        case Kind::Cone:
            solid = model::PrimitiveFactory::makeCone(m_p0, m_p1, m_p2, n);
            break;
        case Kind::Torus:
            solid = model::PrimitiveFactory::makeTorus(m_p0, m_p1, n);
            break;
    }
    if (!solid) {
        return failWith(reason,
                        "these dimensions do not make a solid: they must be positive (a cone may "
                        "have one zero radius, a torus's tube must be thinner than its ring)");
    }
    if (naming() == model::NamingScheme::Stable) {
        // This feature's names, not the kind's: every box's top was
        // `box/top`, so a second box's was the first's too. Edges after the
        // faces they part, not their storage order.
        model::scopeToFeature(*solid, featureID());
        model::nameFacetsLogically(*solid);  // a cylinder's side is one face, in facets
        model::nameEdgesLogically(*solid);
    }
    if (!isPlaced()) return solid;
    // Stood where it goes: its z axis turned onto the axis direction (the
    // shortest turn, half round for -Z), then moved to the base point.
    const math::Vec3 z(0.0, 0.0, 1.0);
    const math::Vec3 to = m_axisDirection.normalized();
    const math::Vec3 turnAxis = z.cross(to);
    const double angle = std::atan2(turnAxis.length(), z.dot(to));
    const math::Quaternion turn =
        turnAxis.length() > 1e-12
            ? math::Quaternion::fromAxisAngle(turnAxis.normalized(), angle)
            : math::Quaternion::fromAxisAngle(math::Vec3(1.0, 0.0, 0.0), angle);
    return model::Pattern::transformed(
        *solid, math::Mat4::translation(m_basePoint) * math::Mat4::rotation(turn));
}

bool PrimitiveFeature::isPlaced() const {
    return m_basePoint.length() > 0.0 ||
           (m_axisDirection - math::Vec3(0.0, 0.0, 1.0)).length() > 1e-12;
}

std::map<std::string, math::Vec3> PrimitiveFeature::vectors() const {
    return {{"basePoint", m_basePoint}, {"axisDirection", m_axisDirection}};
}

bool PrimitiveFeature::setVector(const std::string& name, const math::Vec3& value) {
    if (name == "basePoint" && finite(value)) {
        m_basePoint = value;
        return true;
    }
    if (name != "axisDirection") return false;
    const auto unit = unitDirection(value);
    if (!unit) return false;
    m_axisDirection = *unit;
    return true;
}

math::IdCounter<int> DatumFeature::s_nextID{1};

std::unique_ptr<DatumFeature> DatumFeature::makePlane(const model::DatumPlane& plane) {
    std::unique_ptr<DatumFeature> f(new DatumFeature());
    f->m_kind = DatumKind::Plane;
    f->m_origin = plane.origin;
    f->m_dirA = plane.normal;
    f->m_dirB = plane.xAxis;
    f->m_featureID = "datum_" + std::to_string(s_nextID.next());
    return f;
}

std::unique_ptr<DatumFeature> DatumFeature::makeAxis(const model::DatumAxis& axis) {
    std::unique_ptr<DatumFeature> f(new DatumFeature());
    f->m_kind = DatumKind::Axis;
    f->m_origin = axis.origin;
    f->m_dirA = axis.direction;
    f->m_featureID = "datum_" + std::to_string(s_nextID.next());
    return f;
}

std::unique_ptr<DatumFeature> DatumFeature::makePoint(const model::DatumPoint& point) {
    std::unique_ptr<DatumFeature> f(new DatumFeature());
    f->m_kind = DatumKind::Point;
    f->m_origin = point.position;
    f->m_featureID = "datum_" + std::to_string(s_nextID.next());
    return f;
}

std::string DatumFeature::name() const {
    switch (m_kind) {
        case DatumKind::Plane:
            return "DatumPlane";
        case DatumKind::Axis:
            return "DatumAxis";
        case DatumKind::Point:
            return "DatumPoint";
    }
    return "Datum";
}

std::string DatumFeature::featureID() const {
    return m_featureID;
}

void DatumFeature::restoreFeatureID(const std::string& id) {
    if (id.empty()) return;
    m_featureID = id;
    bumpCounter(s_nextID, id, "datum_");
}

model::DatumPlane DatumFeature::asPlane() const {
    return model::DatumPlane{m_origin, m_dirA, m_dirB};
}

model::DatumAxis DatumFeature::asAxis() const {
    return model::DatumAxis{m_origin, m_dirA};
}

model::DatumPoint DatumFeature::asPoint() const {
    return model::DatumPoint{m_origin};
}

std::unique_ptr<topo::Solid> DatumFeature::execute(std::unique_ptr<topo::Solid> inputSolid,
                                                   std::string* /*reason*/) const {
    // Non-geometric: pass the body through unchanged. The feature tree skips
    // construction features when building, so this is only reached if called
    // directly.
    return inputSolid;
}

// ---------------------------------------------------------------------------
// ImportedBodyFeature
// ---------------------------------------------------------------------------

math::IdCounter<int> ImportedBodyFeature::s_nextID{1};

ImportedBodyFeature::ImportedBodyFeature(std::shared_ptr<const topo::Solid> solid,
                                         std::string source)
    : m_solid(std::move(solid)),
      m_source(std::move(source)),
      m_featureID("imported_" + std::to_string(s_nextID.next())) {}

void ImportedBodyFeature::restoreFeatureID(const std::string& id) {
    m_featureID = id;
    bumpCounter(s_nextID, id, "imported_");
}

std::unique_ptr<topo::Solid> ImportedBodyFeature::execute(
    std::unique_ptr<topo::Solid> /*inputSolid*/, std::string* reason) const {
    if (!m_solid) return failWith(reason, "the imported body is missing");
    auto body = model::Pattern::transformed(*m_solid, math::Mat4::identity());
    int index = 0;
    for (auto& face : body->faces()) {
        face.topoId = topo::TopologyID::make(m_featureID, "face:" + std::to_string(index++));
    }
    // Its curved faces in facets, as the kernel makes its own (Phase 141): a
    // cylinder's face bounded by two circles is no loop of points to measure,
    // cut or draw. The body as read where that cannot be done; the import
    // said so.
    auto faceted = model::facetCurved(*body);
    if (faceted.solid) body = std::move(faceted.solid);
    if (naming() == model::NamingScheme::Stable) {
        model::nameEdgesLogically(*body);
    } else {
        model::nameEdgesByFaces(*body);
    }
    return body;
}

// ---------------------------------------------------------------------------
// FeatureTree
// ---------------------------------------------------------------------------

void FeatureTree::addFeature(std::unique_ptr<Feature> feature) {
    m_features.push_back(std::move(feature));
    ++m_revision;
}

void FeatureTree::insertFeature(size_t index, std::unique_ptr<Feature> feature) {
    index = std::min(index, m_features.size());
    m_features.insert(m_features.begin() + static_cast<ptrdiff_t>(index), std::move(feature));
    if (m_rollbackIndex >= 0 && static_cast<int>(index) <= m_rollbackIndex) ++m_rollbackIndex;
    ++m_revision;
}

void FeatureTree::removeFeature(size_t index) {
    takeFeature(index);
}

std::unique_ptr<Feature> FeatureTree::takeFeature(size_t index) {
    assert(index < m_features.size());
    auto feature = std::move(m_features[index]);
    m_features.erase(m_features.begin() + static_cast<ptrdiff_t>(index));
    if (m_features.empty()) {
        m_rollbackIndex = -1;
    } else if (m_rollbackIndex >= 0 && static_cast<int>(index) <= m_rollbackIndex) {
        // The features before the removed one stay active. (When the removed
        // one was the only active feature, the one after it takes its place:
        // "nothing active" is not a state the index can express.)
        m_rollbackIndex = std::max(0, m_rollbackIndex - 1);
    }
    ++m_revision;
    return feature;
}

std::optional<size_t> FeatureTree::indexOf(const Feature* feature) const {
    for (size_t i = 0; i < m_features.size(); ++i) {
        if (m_features[i].get() == feature) return i;
    }
    return std::nullopt;
}

void FeatureTree::setRollbackIndex(int index) {
    if (index == m_rollbackIndex) return;
    m_rollbackIndex = index;
    ++m_revision;
}

size_t FeatureTree::featureCount() const {
    return m_features.size();
}

const Feature* FeatureTree::feature(size_t index) const {
    assert(index < m_features.size());
    return m_features[index].get();
}

Feature* FeatureTree::feature(size_t index) {
    assert(index < m_features.size());
    return m_features[index].get();
}

void FeatureTree::clear() {
    m_features.clear();
    m_rollbackIndex = -1;
    ++m_revision;
}

namespace {

/// Why `solid` is not a valid solid, in the user's terms, or empty when it is.
/// Both kinds of check: the combinatorial ones (every edge between exactly two
/// faces, closed loops, counts Euler–Poincaré allows) and the geometric ones
/// (flat faces flat, no boundary crossing itself, a closed skin; see
/// GeometryValidator), at a tolerance that grows with the part.
std::string solidProblem(const topo::Solid& solid) {
    if (!solid.checkManifold()) {
        return "its faces do not close up: an edge is not shared by exactly two faces";
    }
    if (!solid.checkEulerFormula()) {
        return "it has vertex, edge and face counts no solid can have";
    }
    math::BoundingBox box;
    for (const auto& v : solid.vertices()) box.expand(v.point);
    const double size = box.isValid() ? (box.max() - box.min()).length() : 0.0;
    const double tol = std::max(topo::GeometryValidator::kDefaultTol, 1e-9 * size);
    const auto issues =
        topo::GeometryValidator::check(solid, tol, topo::GeometryValidator::Scope::FailingOnly);
    if (issues.selfIntersectingLoops > 0) return "a face's boundary crosses itself";
    if (issues.nonPlanarLoops > 0) return "a flat face is not flat";
    if (issues.openShells > 0) return "its skin is not closed";
    if (issues.degenerateFaces > 0) return "a face has no area";
    if (issues.degenerateEdges > 0) return "an edge has no length";
    if (issues.vertexChainErrors > 0 || issues.twinCoincidenceErrors > 0) {
        return "its edges do not meet where they should";
    }
    return {};
}

/// Pass `solid` on if it is a valid solid; otherwise fail with why. Every
/// feature's result is held to this, so a malformed solid stops at the
/// feature that made it instead of corrupting everything built on it.
std::unique_ptr<topo::Solid> checked(std::unique_ptr<topo::Solid> solid, std::string* reason) {
    if (!solid) return nullptr;
    const std::string problem = solidProblem(*solid);
    if (problem.empty()) return solid;
    if (reason) *reason = "the result is not a valid solid: " + problem;
    return nullptr;
}

/// Run a feature, turning an exception from the kernel (the NURBS constructors
/// throw on invalid input, for one) into a failure with its reason, instead of
/// letting it unwind into the caller — ultimately the Qt event loop, which
/// cannot carry an exception and would terminate the application.
std::unique_ptr<topo::Solid> executeContained(const Feature& feat,
                                              std::unique_ptr<topo::Solid> input,
                                              std::string* reason = nullptr,
                                              const BuildContext& context = {}) {
    try {
        return checked(feat.executeIn(context, std::move(input), reason), reason);
    } catch (const std::exception& e) {
        if (reason) *reason = e.what();
    } catch (...) {
        if (reason) *reason = "unknown error";
    }
    return nullptr;
}

/// executeMulti() for buildBodies(), which has no way to say why a feature
/// failed: a combine that throws leaves no bodies, as one that fails does.
std::vector<std::unique_ptr<topo::Solid>> executeMultiContained(
    const Feature& feat, std::vector<std::unique_ptr<topo::Solid>> bodies) {
    try {
        return feat.executeMulti(std::move(bodies));
    } catch (...) {
        return {};
    }
}

/// Combine the body a creating feature built (`tool`) with the part so far,
/// as the feature's operation says. nullptr, with `reason` set, when the
/// operation cannot be carried out — never an empty part passed off as a
/// result.
std::unique_ptr<topo::Solid> combine(BodyOperation operation, std::unique_ptr<topo::Solid> part,
                                     std::unique_ptr<topo::Solid> tool, std::string* reason,
                                     model::NamingScheme naming) {
    const auto fail = [reason](const char* why) -> std::unique_ptr<topo::Solid> {
        if (reason) *reason = why;
        return nullptr;
    };
    if (!part) {
        if (operation == BodyOperation::Cut) return fail("there is no body to cut from");
        if (operation == BodyOperation::Intersect)
            return fail("there is no body to intersect with");
        return tool;  // the first body, whatever it was asked to join
    }
    if (operation == BodyOperation::NewBody) return model::Pattern::collect(*part, *tool);

    const model::BooleanType type = operation == BodyOperation::Join ? model::BooleanType::Union
                                    : operation == BodyOperation::Cut
                                        ? model::BooleanType::Subtract
                                        : model::BooleanType::Intersect;
    std::unique_ptr<topo::Solid> result;
    std::string booleanReason;
    try {
        result = model::BooleanOp::execute(*part, *tool, type, &booleanReason, naming);
    } catch (const std::exception& e) {
        if (reason) *reason = std::string("the Boolean failed: ") + e.what();
        return nullptr;
    }
    if (!result && reason) *reason = booleanReason;
    return checked(std::move(result), reason);
}

/// Whether a feature takes part in the build. Reference geometry (datums) has
/// no effect on the solid, and a suppressed feature is left out.
bool takesPart(const Feature& feature) {
    return !feature.isConstruction() && !feature.isSuppressed();
}

/// The sketches made ready so far in one build (Phase 157), by id.
using Placed = std::map<uint64_t, std::shared_ptr<Sketch>>;

/// Whether @p sketch has an edge of the part projected into it.
bool projects(const Sketch& sketch) {
    return std::any_of(sketch.entities().begin(), sketch.entities().end(),
                       [](const auto& entity) { return !entity->sourceEdge().empty(); });
}

/// Each edge projected into @p sketch drawn again from @p part, where the
/// sketch now is: the same entity (its id, its style) in the edge's shape
/// now. An edge that is gone leaves construction geometry where it was;
/// one that shapes the part fails it, false and why.
bool projectAgain(Sketch& sketch, const topo::Solid* part, std::string* reason) {
    // Taken first: each is replaced in the drawing as it is drawn again.
    std::vector<std::shared_ptr<draft::DraftEntity>> projected;
    for (const auto& entity : sketch.entities()) {
        if (!entity->sourceEdge().empty()) projected.push_back(entity);
    }
    for (const auto& entity : projected) {
        const std::string& edge = entity->sourceEdge();
        std::string why = "is not there: there is no part before it";
        auto again =
            part != nullptr ? model::projectEdge(*part, edge, sketch.plane(), &why) : nullptr;
        if (!again) {
            if (entity->construction()) continue;  // a guide only: it stays
            if (reason) {
                *reason = "sketch '" + sketch.name() + "' has the part's edge " + edge +
                          " projected into it, and it " + why;
            }
            return false;
        }
        again->setId(entity->id());
        again->copyStyleFrom(*entity);
        again->setSourceEdge(edge);
        sketch.drawing().replaceEntity(entity->id(), std::move(again));
    }
    return true;
}

/// Each sketch @p feature is made from made ready for it, as @p part stands
/// before it: placed on the face it follows, and the edges projected into
/// it projected again. Once in a build, by the first feature made from it
/// (a pattern builds its feature again against a later part; a second
/// feature from the sketch builds on it as it was made ready). False, and
/// why, when a face is not there or not flat, or an edge that shapes the
/// part is gone.
bool prepareSketches(const Feature& feature, const topo::Solid* part, Placed& placed,
                     std::string* reason) {
    for (const auto& sketch : feature.sketches()) {
        if (!sketch || placed.count(sketch->id()) != 0) continue;
        const bool follows = !sketch->face().empty();
        if (!follows && !projects(*sketch)) continue;
        if (follows) {
            const std::string on =
                "sketch '" + sketch->name() + "' is on a face (" + sketch->face() + ")";
            if (part == nullptr) {
                if (reason) *reason = on + ", and there is no part before it";
                return false;
            }
            std::string why;
            const auto face = model::planeOfFace(*part, sketch->face(), &why);
            if (!face) {
                if (reason) *reason = on + " that " + why;
                return false;
            }
            sketch->placeOn(face->origin, face->normal);
        }
        if (!projectAgain(*sketch, part, reason)) return false;
        placed[sketch->id()] = sketch;
    }
    return true;
}

/// One step of the regeneration rule every build path shares: a creating
/// feature builds a tool body, combined with the part by its operation; any
/// other feature transforms the part.
///
/// Nothing escapes it. The feature itself is contained (executeContained),
/// but combining its body with the part (collecting a new body, checking the
/// result) is kernel code too, and an exception from it would unwind through
/// every build: out of a worker, or out of a trial build that has put the
/// feature into the tree and not yet taken it out.
std::unique_ptr<topo::Solid> applyFeature(const Feature& feature, std::unique_ptr<topo::Solid> part,
                                          std::string* reason,
                                          const std::vector<const Feature*>& before,
                                          Placed& placed) {
    try {
        if (!prepareSketches(feature, part.get(), placed, reason)) return nullptr;
        BuildContext context;
        context.before = before;
        if (!feature.createsNewBody()) {
            return executeContained(feature, std::move(part), reason, context);
        }
        context.part = part.get();  // what a through-all extrusion goes through
        auto tool = executeContained(feature, nullptr, reason, context);
        if (!tool) return nullptr;
        return combine(feature.operation(), std::move(part), std::move(tool), reason,
                       feature.naming());
    } catch (const std::exception& e) {
        if (reason) *reason = e.what();
    } catch (...) {
        if (reason) *reason = "unknown error";
    }
    return nullptr;
}

}  // namespace

void PatternFeature::setTargets(std::vector<std::string> targets) {
    m_targets.clear();
    for (auto& id : targets) {
        if (std::find(m_targets.begin(), m_targets.end(), id) == m_targets.end()) {
            m_targets.push_back(std::move(id));
        }
    }
}

math::Mat4 PatternFeature::instanceTransform(int k) const {
    const double at = static_cast<double>(k);
    if (m_kind == Kind::Linear)
        return math::Mat4::translation(m_vecA.normalized() * (m_scalar * at));
    return math::Mat4::translation(m_vecA) *
           math::Mat4::rotation(
               math::Quaternion::fromAxisAngle(m_vecB.normalized(), m_scalar * at)) *
           math::Mat4::translation(m_vecA * -1.0);
}

std::unique_ptr<topo::Solid> PatternFeature::executeIn(const BuildContext& context,
                                                       std::unique_ptr<topo::Solid> inputSolid,
                                                       std::string* reason) const {
    if (m_targets.empty()) return execute(std::move(inputSolid), reason);
    if (!inputSolid) return failWith(reason, "there is no body to pattern");
    std::unique_ptr<topo::Solid> part = std::move(inputSolid);
    for (const std::string& id : m_targets) {
        const Feature* target = nullptr;
        for (const Feature* feature : context.before) {
            if (feature->featureID() == id) target = feature;
        }
        if (!target) {
            return failWith(reason, "the feature to repeat (" + id +
                                        ") is not before the pattern, or is suppressed");
        }
        if (!target->createsNewBody()) {
            return failWith(reason, target->name() +
                                        " cannot be repeated: only a feature that adds or cuts "
                                        "material can be");
        }
        // Its body as it was built: against the part as it stands now, for
        // one that goes through all of it.
        BuildContext targetContext;
        targetContext.part = part.get();
        targetContext.before = context.before;
        std::string why;
        auto tool = target->executeIn(targetContext, nullptr, &why);
        if (!tool) return failWith(reason, target->name() + " to repeat: " + why);
        for (int k = 1; k < m_count; ++k) {
            if (std::find(m_suppressed.begin(), m_suppressed.end(), k) != m_suppressed.end()) {
                continue;
            }
            auto copy = model::Pattern::transformed(*tool, instanceTransform(k));
            // Each copy's faces and edges named as this pattern's instance k.
            // By this pattern, not "pattern" as a whole-part pattern names
            // them: two patterns of one separate body would otherwise make
            // copies named alike, and nothing after renames them apart.
            for (auto& face : copy->faces()) {
                if (face.topoId.isValid()) face.topoId = face.topoId.child(featureID(), k);
            }
            for (auto& edge : copy->edges()) {
                if (edge.topoId.isValid()) edge.topoId = edge.topoId.child(featureID(), k);
            }
            part = combine(target->operation(), std::move(part), std::move(copy), reason, naming());
            if (!part) return nullptr;
        }
    }
    return part;
}

// ---------------------------------------------------------------------------
// MirrorFeature (Phase 162)
// ---------------------------------------------------------------------------

math::IdCounter<int> MirrorFeature::s_nextID{1};

std::unique_ptr<MirrorFeature> MirrorFeature::make(const math::Vec3& planePoint,
                                                   const math::Vec3& planeNormal) {
    std::unique_ptr<MirrorFeature> f(new MirrorFeature());
    f->m_point = planePoint;
    if (const auto unit = unitDirection(planeNormal)) f->m_normal = *unit;
    f->m_featureID = "mirror_" + std::to_string(s_nextID.next());
    return f;
}

void MirrorFeature::restoreFeatureID(const std::string& id) {
    if (id.empty()) return;
    m_featureID = id;
    bumpCounter(s_nextID, id, "mirror_");
}

std::map<std::string, math::Vec3> MirrorFeature::vectors() const {
    return {{"planePoint", m_point}, {"planeNormal", m_normal}};
}

bool MirrorFeature::setVector(const std::string& name, const math::Vec3& value) {
    if (name == "planePoint" && finite(value)) {
        m_point = value;
        return true;
    }
    if (name != "planeNormal") return false;
    const auto unit = unitDirection(value);
    if (!unit) return false;
    m_normal = *unit;
    return true;
}

bool MirrorFeature::setReference(const std::string& name, const std::string& value) {
    if (name != "planeFace") return false;
    m_face = value;
    return true;
}

void MirrorFeature::setTargets(std::vector<std::string> targets) {
    m_targets.clear();
    for (auto& id : targets) {
        if (std::find(m_targets.begin(), m_targets.end(), id) == m_targets.end()) {
            m_targets.push_back(std::move(id));
        }
    }
}

std::optional<math::Mat4> MirrorFeature::mirrorIn(const topo::Solid& part, std::string* why) const {
    if (m_face.empty()) return math::Mat4::reflection(m_point, m_normal);
    const auto plane = model::planeOfFace(part, m_face, why);
    if (!plane) return std::nullopt;
    return math::Mat4::reflection(plane->origin, plane->normal);
}

void MirrorFeature::nameImage(topo::Solid& image) const {
    for (auto& face : image.faces()) {
        if (face.topoId.isValid()) face.topoId = face.topoId.child(featureID(), 1);
    }
    for (auto& edge : image.edges()) {
        if (edge.topoId.isValid()) edge.topoId = edge.topoId.child(featureID(), 1);
    }
}

std::unique_ptr<topo::Solid> MirrorFeature::execute(std::unique_ptr<topo::Solid> inputSolid,
                                                    std::string* reason) const {
    return executeIn(BuildContext{}, std::move(inputSolid), reason);
}

std::unique_ptr<topo::Solid> MirrorFeature::executeIn(const BuildContext& context,
                                                      std::unique_ptr<topo::Solid> inputSolid,
                                                      std::string* reason) const {
    if (!inputSolid) return failWith(reason, "there is no body to mirror");
    std::string why;
    const auto mirror = mirrorIn(*inputSolid, &why);
    if (!mirror) return failWith(reason, "the plane to mirror in: " + why);
    std::unique_ptr<topo::Solid> part = std::move(inputSolid);
    if (m_targets.empty()) {
        // The whole part and its image, one body where they meet.
        auto image = model::Pattern::transformed(*part, *mirror);
        nameImage(*image);
        return combine(BodyOperation::Join, std::move(part), std::move(image), reason, naming());
    }
    for (const std::string& id : m_targets) {
        const Feature* target = nullptr;
        for (const Feature* feature : context.before) {
            if (feature->featureID() == id) target = feature;
        }
        if (!target) {
            return failWith(reason, "the feature to mirror (" + id +
                                        ") is not before the mirror, or is suppressed");
        }
        if (!target->createsNewBody()) {
            return failWith(reason, target->name() +
                                        " cannot be mirrored: only a feature that adds or cuts "
                                        "material can be");
        }
        // Its body as it was built, against the part as it stands now, as a
        // pattern repeats it.
        BuildContext targetContext;
        targetContext.part = part.get();
        targetContext.before = context.before;
        auto tool = target->executeIn(targetContext, nullptr, &why);
        if (!tool) return failWith(reason, target->name() + " to mirror: " + why);
        auto image = model::Pattern::transformed(*tool, *mirror);
        nameImage(*image);
        part = combine(target->operation(), std::move(part), std::move(image), reason, naming());
        if (!part) return nullptr;
    }
    return part;
}

// ---------------------------------------------------------------------------
// HoleFeature (Phase 162)
// ---------------------------------------------------------------------------

math::IdCounter<int> HoleFeature::s_nextID{1};

std::unique_ptr<HoleFeature> HoleFeature::make(const std::string& face, const math::Vec3& position,
                                               double diameter, double depth) {
    std::unique_ptr<HoleFeature> f(new HoleFeature());
    f->m_face = face;
    f->m_position = position;
    if (diameter > 0.0) f->m_diameter = diameter;
    if (depth > 0.0) f->m_depth = depth;
    f->setOperation(BodyOperation::Cut);
    f->m_featureID = "hole_" + std::to_string(s_nextID.next());
    return f;
}

void HoleFeature::restoreFeatureID(const std::string& id) {
    if (id.empty()) return;
    m_featureID = id;
    bumpCounter(s_nextID, id, "hole_");
}

std::map<std::string, double> HoleFeature::parameters() const {
    return {{"type", static_cast<double>(m_type)},
            {"extent", static_cast<double>(m_extent)},
            {"diameter", m_diameter},
            {"depth", m_depth},
            {"boreDiameter", m_boreDiameter},
            {"boreDepth", m_boreDepth},
            {"sinkDiameter", m_sinkDiameter},
            {"sinkAngle", m_sinkAngle},
            {"pointAngle", m_pointAngle},
            {"segments", static_cast<double>(m_segments)}};
}

bool HoleFeature::setParameter(const std::string& name, double value) {
    if (name == "type") {
        int code = 0;
        if (!countParameter(value, 0, static_cast<int>(Type::Countersink), code)) return false;
        m_type = static_cast<Type>(code);
        return true;
    }
    if (name == "extent") {
        int code = 0;
        if (!countParameter(value, 0, static_cast<int>(Extent::UpToFace), code)) return false;
        m_extent = static_cast<Extent>(code);
        return true;
    }
    if (name == "segments") return countParameter(value, 3, kMaxFacetSegments, m_segments);
    // Sizes are more than nothing; a point's angle may be 0 (a flat bottom),
    // a countersink's may not; neither reaches a half turn.
    double* size = name == "diameter"       ? &m_diameter
                   : name == "depth"        ? &m_depth
                   : name == "boreDiameter" ? &m_boreDiameter
                   : name == "boreDepth"    ? &m_boreDepth
                   : name == "sinkDiameter" ? &m_sinkDiameter
                                            : nullptr;
    if (size != nullptr) {
        if (!(std::isfinite(value) && value > 0.0)) return false;
        *size = value;
        return true;
    }
    if (name == "sinkAngle" || name == "pointAngle") {
        const bool flat = name == "pointAngle" && value == 0.0;
        if (!(std::isfinite(value) && value < math::kPi && (value > 0.0 || flat))) return false;
        (name == "sinkAngle" ? m_sinkAngle : m_pointAngle) = value;
        return true;
    }
    return false;
}

Feature::ParameterKind HoleFeature::parameterKind(const std::string& name) const {
    if (name == "type" || name == "extent") return ParameterKind::Choice;
    if (name == "sinkAngle" || name == "pointAngle") return ParameterKind::Angle;
    return Feature::parameterKind(name);
}

std::vector<std::string> HoleFeature::parameterChoices(const std::string& name) const {
    if (name == "type") return {"Simple", "Counterbore", "Countersink"};
    if (name == "extent") return {"To the depth", "Through all", "Up to a face"};
    return Feature::parameterChoices(name);
}

bool HoleFeature::setVector(const std::string& name, const math::Vec3& value) {
    if (name != "positionPoint" || !finite(value)) return false;
    m_position = value;
    return true;
}

bool HoleFeature::setReference(const std::string& name, const std::string& value) {
    if (name == "face") {
        m_face = value;
        return true;
    }
    if (name == "upToFace") {
        m_upToFace = value;
        return true;
    }
    return false;
}

std::unique_ptr<topo::Solid> HoleFeature::execute(std::unique_ptr<topo::Solid> inputSolid,
                                                  std::string* reason) const {
    BuildContext context;
    context.part = inputSolid.get();
    return executeIn(context, nullptr, reason);
}

namespace {

/// A hole's half-section's lines, by what each sweeps: fixed ids, so each
/// surface of the hole has a name of its own (and keeps it), and each is
/// renamed for what it is.
enum HoleLine : uint64_t { Top = 1, Wall, Bottom, Axis, Bore, BoreFloor, Sink };

const char* holeLineName(uint64_t id) {
    switch (id) {
        case Top:
            return "top";
        case Wall:
            return "wall";
        case Bottom:
            return "bottom";
        case Axis:
            return "axis";
        case Bore:
            return "bore";
        case BoreFloor:
            return "boreFloor";
        case Sink:
            return "sink";
        default:
            return nullptr;
    }
}

/// @p tag with each "revolved:e<id>" of a hole's line named for it.
std::string holeName(const std::string& tag) {
    static const std::string kSource = "revolved:e";
    std::string out;
    size_t from = 0;
    for (size_t at = tag.find(kSource); at != std::string::npos; at = tag.find(kSource, from)) {
        size_t end = at + kSource.size();
        while (end < tag.size() && std::isdigit(static_cast<unsigned char>(tag[end])) != 0) ++end;
        uint64_t id = 0;
        std::from_chars(tag.data() + at + kSource.size(), tag.data() + end, id);
        const char* name = holeLineName(id);
        out += tag.substr(from, at - from);
        out += name != nullptr ? std::string(name) : tag.substr(at, end - at);
        from = end;
    }
    return out + tag.substr(from);
}

}  // namespace

std::unique_ptr<topo::Solid> HoleFeature::executeIn(const BuildContext& context,
                                                    std::unique_ptr<topo::Solid> /*inputSolid*/,
                                                    std::string* reason) const {
    if (!context.part) {
        return failWith(reason, "a hole is drilled into the part: there is no part before it");
    }
    if (m_face.empty()) return failWith(reason, "no face is chosen to drill into");
    std::string why;
    const auto face = model::planeOfFace(*context.part, m_face, &why);
    if (!face) return failWith(reason, "the face it is drilled into (" + m_face + ") " + why);
    const math::Vec3 n = face->normal.normalized();  // out of the part
    const math::Vec3 into = n * -1.0;
    // Where it is, on the face as the face now is.
    const math::Vec3 at = m_position - n * (m_position - face->origin).dot(n);
    const double r = m_diameter / 2.0;

    // How deep the wall goes, and whether it ends in a point.
    double depth = m_depth;
    bool pointed = m_pointAngle > 0.0;
    if (m_extent == Extent::ThroughAll) {
        double reach = 0.0;
        for (const auto& v : context.part->vertices()) {
            reach = std::max(reach, (v.point - at).dot(into));
        }
        if (!(reach > 1e-9)) return failWith(reason, "the part is not behind its face");
        depth = reach * 1.01 + 1e-6;  // out of the far side
        pointed = false;
    } else if (m_extent == Extent::UpToFace) {
        if (m_upToFace.empty()) return failWith(reason, "no face is chosen to go up to");
        const std::string upTo = "the face it goes up to (" + m_upToFace + ")";
        const auto to = model::planeOfFace(*context.part, m_upToFace, &why);
        if (!to) return failWith(reason, upTo + " " + why);
        if (to->normal.cross(n).length() > 1e-9) {
            return failWith(reason, upTo + " is at a slant to the face it is drilled into");
        }
        depth = (to->origin - at).dot(to->normal) / into.dot(to->normal);
        if (!(depth > 1e-9)) return failWith(reason, upTo + " is not behind the face");
        pointed = false;
    }

    // The half-section, in a plane through the axis: x across, from the
    // axis; y down it, into the part. It starts a little above the face, so
    // the cut shares no plane with it.
    const double lead = 0.02 * std::max(m_diameter, depth) + 1e-6;
    std::vector<std::pair<math::Vec2, uint64_t>> section;  // each point, and the line from it
    section.push_back({{0.0, -lead}, Top});
    switch (m_type) {
        case Type::Simple:
            section.push_back({{r, -lead}, Wall});
            break;
        case Type::Counterbore: {
            const double rb = m_boreDiameter / 2.0;
            if (!(rb > r)) return failWith(reason, "the counterbore must be wider than the hole");
            if (!(m_boreDepth < depth)) {
                return failWith(reason, "the counterbore must be shallower than the hole");
            }
            section.push_back({{rb, -lead}, Bore});
            section.push_back({{rb, m_boreDepth}, BoreFloor});
            section.push_back({{r, m_boreDepth}, Wall});
            break;
        }
        case Type::Countersink: {
            const double rs = m_sinkDiameter / 2.0;
            if (!(rs > r)) return failWith(reason, "the countersink must be wider than the hole");
            const double slope = std::tan(m_sinkAngle / 2.0);  // across, per depth
            const double sinkDepth = (rs - r) / slope;
            if (!(sinkDepth < depth)) {
                return failWith(reason, "the countersink must be shallower than the hole");
            }
            section.push_back({{rs + lead * slope, -lead}, Sink});
            section.push_back({{r, sinkDepth}, Wall});
            break;
        }
    }
    section.push_back({{r, depth}, Bottom});
    const double tip = pointed ? r / std::tan(m_pointAngle / 2.0) : 0.0;
    section.push_back({{0.0, depth + tip}, Axis});

    std::vector<std::shared_ptr<draft::DraftEntity>> profile;
    for (size_t i = 0; i < section.size(); ++i) {
        const auto& [from, id] = section[i];
        auto line =
            std::make_shared<draft::DraftLine>(from, section[(i + 1) % section.size()].first);
        line->setId(id);
        profile.push_back(std::move(line));
    }
    // Across: any direction square to the axis; the plane's y is then down it.
    const math::Vec3 seed = std::abs(n.x) < 0.9 ? math::Vec3::UnitX : math::Vec3::UnitY;
    const math::Vec3 across = (seed - n * seed.dot(n)).normalized();
    const draft::SketchPlane plane(at, across.cross(into), across);
    auto hole = model::Revolve::execute(profile, plane, at, into, 2.0 * math::kPi, m_featureID,
                                        m_segments, 0.0, &why, naming());
    if (!hole) return failWith(reason, "the hole could not be made: " + why);
    for (auto& f : hole->faces()) {
        if (f.topoId.isValid()) f.topoId = topo::TopologyID::fromTag(holeName(f.topoId.tag()));
    }
    for (auto& e : hole->edges()) {
        if (e.topoId.isValid()) e.topoId = topo::TopologyID::fromTag(holeName(e.topoId.tag()));
    }
    return hole;
}

const char* bodyOperationName(BodyOperation operation) {
    switch (operation) {
        case BodyOperation::NewBody:
            return "new";
        case BodyOperation::Join:
            return "join";
        case BodyOperation::Cut:
            return "cut";
        case BodyOperation::Intersect:
            return "intersect";
    }
    return "new";
}

std::optional<BodyOperation> bodyOperationFromName(std::string_view name) {
    for (const BodyOperation op : {BodyOperation::NewBody, BodyOperation::Join, BodyOperation::Cut,
                                   BodyOperation::Intersect}) {
        if (name == bodyOperationName(op)) return op;
    }
    return std::nullopt;
}

std::unique_ptr<topo::Solid> FeatureTree::build() const {
    if (m_features.empty()) {
        return nullptr;
    }

    std::unique_ptr<topo::Solid> solid;
    std::vector<const Feature*> before;
    Placed placed;
    for (const auto& feat : m_features) {
        if (!takesPart(*feat)) continue;
        solid = applyFeature(*feat, std::move(solid), nullptr, before, placed);
        if (!solid) {
            return nullptr;  // Feature failed
        }
        before.push_back(feat.get());
    }
    return solid;
}

std::vector<std::unique_ptr<topo::Solid>> FeatureTree::buildBodies() const {
    std::vector<std::unique_ptr<topo::Solid>> bodies;
    BuildContext context;
    Placed placed;
    for (const auto& feat : m_features) {
        if (!takesPart(*feat)) continue;
        context.part = bodies.empty() ? nullptr : bodies.back().get();
        if (!prepareSketches(*feat, context.part, placed, nullptr)) continue;

        if (feat->consumesAllBodies()) {
            // Boolean-style combine: replace the whole body list with its result.
            bodies = executeMultiContained(*feat, std::move(bodies));
        } else if (feat->createsNewBody() && feat->operation() != BodyOperation::NewBody &&
                   !bodies.empty()) {
            // Join / Cut / Intersect the active body, as the product path does.
            auto tool = executeContained(*feat, nullptr, nullptr, context);
            if (!tool) continue;
            auto combined = combine(feat->operation(), std::move(bodies.back()), std::move(tool),
                                    nullptr, feat->naming());
            bodies.pop_back();
            if (combined) bodies.push_back(std::move(combined));
        } else if (feat->createsNewBody() || bodies.empty()) {
            // Start a fresh body. Create features ignore any input solid; a
            // transform with no active body (bodies.empty()) has nothing to act
            // on, so it too is executed against a null input and simply fails.
            auto solid = executeContained(*feat, nullptr, nullptr, context);
            if (solid) {
                bodies.push_back(std::move(solid));
            }
        } else {
            // Transform the active (most-recently-created) body in place.
            context.part = nullptr;  // the body is the feature's input
            auto solid = executeContained(*feat, std::move(bodies.back()), nullptr, context);
            bodies.pop_back();
            if (solid) {
                bodies.push_back(std::move(solid));
            }
            // If the transform failed, the active body is dropped; the next
            // create feature starts a new one.
        }
        context.before.push_back(feat.get());
    }
    return bodies;
}

BuildResult FeatureTree::buildWithDiagnostics(BuildControl* control) const {
    BuildResult result;
    if (m_features.empty()) {
        return result;
    }

    const int limit = (m_rollbackIndex >= 0)
                          ? std::min(m_rollbackIndex + 1, static_cast<int>(m_features.size()))
                          : static_cast<int>(m_features.size());
    if (control) control->total = limit;

    std::unique_ptr<topo::Solid> solid;
    std::vector<const Feature*> before;
    Placed placed;
    // Where each sketch was placed, and the edges projected into it as they
    // are now, for the document this tree is a copy of.
    const auto report = [&placed, &result] {
        for (const auto& [id, sketch] : placed) {
            if (!sketch->face().empty()) result.placements.emplace(id, sketch->plane());
            auto& projected = result.projections[id];
            for (const auto& entity : sketch->entities()) {
                if (!entity->sourceEdge().empty()) projected.push_back(entity);
            }
            if (projected.empty()) result.projections.erase(id);
        }
    };
    for (int i = 0; i < limit; ++i) {
        if (control) {
            if (control->cancel) {
                result.cancelled = true;
                return result;
            }
            control->done = i;
        }
        if (!takesPart(*m_features[static_cast<size_t>(i)])) {
            result.lastSuccessfulFeature = i;  // nothing to fail
            continue;
        }
        const Feature& feature = *m_features[static_cast<size_t>(i)];
        std::string reason = feature.expressionError();  // Phase 155
        auto next = reason.empty()
                        ? applyFeature(feature, std::move(solid), &reason, before, placed)
                        : nullptr;
        if (!next) {
            result.failedFeatureIndex = i;
            result.failureMessage = reason.empty()
                                        ? "Feature '" + feature.name() + "' failed to execute"
                                        : "Feature '" + feature.name() + "' failed: " + reason;
            // The part as it stood before the failing feature: the feature
            // consumed the solid it failed on, so build it again up to there.
            // Only a failed build pays for this; a successful one copies
            // nothing.
            result.solid = replayUpTo(i, control);
            if (control && control->cancel) result.cancelled = true;
            report();
            return result;
        }
        solid = std::move(next);
        before.push_back(&feature);
        result.lastSuccessfulFeature = i;
    }

    if (control) control->done = limit;
    result.solid = std::move(solid);
    report();
    return result;
}

std::unique_ptr<topo::Solid> FeatureTree::replayUpTo(int limit, BuildControl* control) const {
    std::unique_ptr<topo::Solid> solid;
    std::vector<const Feature*> before;
    Placed placed;
    for (int i = 0; i < limit; ++i) {
        if (control && control->cancel) return nullptr;
        const Feature& feature = *m_features[static_cast<size_t>(i)];
        if (!takesPart(feature)) continue;
        std::string reason;
        solid = applyFeature(feature, std::move(solid), &reason, before, placed);
        if (!solid) return nullptr;  // built the first time; cannot fail now
        before.push_back(&feature);
    }
    return solid;
}

void FeatureTree::moveFeature(int fromIndex, int toIndex) {
    if (fromIndex < 0 || fromIndex >= static_cast<int>(m_features.size())) return;
    if (toIndex < 0 || toIndex >= static_cast<int>(m_features.size())) return;
    if (fromIndex == toIndex) return;

    // A move is a removal and an insertion, and the rollback index follows it
    // as it follows those: the features that did not move keep their side of
    // the rollback bar, and the moved one lands on whichever side it was
    // dropped.
    auto feat = takeFeature(static_cast<size_t>(fromIndex));
    insertFeature(static_cast<size_t>(toIndex), std::move(feat));
}

}  // namespace hz::doc
