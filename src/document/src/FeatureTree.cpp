#include "horizon/document/FeatureTree.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/ChamferOp.h"
#include "horizon/modeling/Draft.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/FilletOp.h"
#include "horizon/modeling/Loft.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/modeling/Shell.h"
#include "horizon/modeling/Sweep.h"

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
void bumpCounter(int& counter, const std::string& id, const std::string& prefix) {
    if (id.rfind(prefix, 0) != 0) return;
    try {
        int n = std::stoi(id.substr(prefix.size()));
        if (n >= counter) counter = n + 1;
    } catch (...) {
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// ExtrudeFeature
// ---------------------------------------------------------------------------

int ExtrudeFeature::s_nextID = 1;

ExtrudeFeature::ExtrudeFeature(std::shared_ptr<Sketch> sketch, const math::Vec3& direction,
                               double distance)
    : m_sketch(std::move(sketch)),
      m_direction(direction),
      m_distance(distance),
      m_featureID("extrude_" + std::to_string(s_nextID++)) {}

std::string ExtrudeFeature::name() const {
    return "Extrude";
}

std::map<std::string, double> ExtrudeFeature::parameters() const {
    std::map<std::string, double> params = {{"distance", m_distance}};
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

bool ExtrudeFeature::hasCurvedProfile() const {
    if (!m_sketch) return false;
    for (const auto& ent : m_sketch->entities()) {
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

std::unique_ptr<topo::Solid> ExtrudeFeature::execute(std::unique_ptr<topo::Solid> /*inputSolid*/,
                                                     std::string* reason) const {
    // The extrusion alone; the tree combines it with the part according to
    // operation() (see applyFeature), as for every body-creating feature.
    return model::Extrude::execute(m_sketch->entities(), m_sketch->plane(), m_direction, m_distance,
                                   m_featureID, m_segments, m_chordTolerance, reason);
}

// ---------------------------------------------------------------------------
// RevolveFeature
// ---------------------------------------------------------------------------

int RevolveFeature::s_nextID = 1;

RevolveFeature::RevolveFeature(std::shared_ptr<Sketch> sketch, const math::Vec3& axisPoint,
                               const math::Vec3& axisDir, double angle)
    : m_sketch(std::move(sketch)),
      m_axisPoint(axisPoint),
      m_axisDir(axisDir),
      m_angle(angle),
      m_featureID("revolve_" + std::to_string(s_nextID++)) {}

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

int RevolveFeature::segments() const {
    if (m_chordTolerance > 0.0 && m_sketch) {
        const double radius = model::Revolve::profileRadius(m_sketch->entities(), m_sketch->plane(),
                                                            m_axisPoint, m_axisDir);
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
    return model::Revolve::execute(m_sketch->entities(), m_sketch->plane(), m_axisPoint, m_axisDir,
                                   m_angle, m_featureID, segments(), m_chordTolerance, reason);
}

// ---------------------------------------------------------------------------
// LoftFeature
// ---------------------------------------------------------------------------

int LoftFeature::s_nextID = 1;

LoftFeature::LoftFeature(std::vector<std::shared_ptr<Sketch>> sections)
    : m_sections(std::move(sections)), m_featureID("loft_" + std::to_string(s_nextID++)) {}

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
    auto solid = model::Loft::execute(sections, m_featureID);
    if (!solid) return failWith(reason, "the sections could not be lofted into a solid");
    return solid;
}

// ---------------------------------------------------------------------------
// SweepFeature
// ---------------------------------------------------------------------------

int SweepFeature::s_nextID = 1;

SweepFeature::SweepFeature(std::shared_ptr<Sketch> profile, std::shared_ptr<Sketch> path)
    : m_profile(std::move(profile)),
      m_path(std::move(path)),
      m_featureID("sweep_" + std::to_string(s_nextID++)) {}

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
    auto solid = model::Sweep::execute(m_profile->entities(), m_profile->plane(), pathPoints,
                                       m_featureID, m_segments, m_chordTolerance);
    if (!solid) {
        return failWith(reason,
                        "the profile cannot follow the path: it may turn more tightly than the "
                        "profile allows, double back, or cross itself");
    }
    return solid;
}

// ---------------------------------------------------------------------------
// DraftFeature
// ---------------------------------------------------------------------------

int DraftFeature::s_nextID = 1;

DraftFeature::DraftFeature(const math::Vec3& pullDir, const math::Vec3& neutralPoint, double angle)
    : m_pullDir(pullDir),
      m_neutralPoint(neutralPoint),
      m_angle(angle),
      m_featureID("draft_" + std::to_string(s_nextID++)) {}

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

int ShellFeature::s_nextID = 1;

ShellFeature::ShellFeature(double thickness, std::vector<topo::TopologyID> removedFaceIds)
    : m_thickness(thickness),
      m_removedFaceIds(std::move(removedFaceIds)),
      m_featureID("shell_" + std::to_string(s_nextID++)) {}

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
    return std::move(result.solid);
}

// ---------------------------------------------------------------------------
// FilletFeature
// ---------------------------------------------------------------------------

int FilletFeature::s_nextID = 1;

FilletFeature::FilletFeature(std::vector<topo::TopologyID> edgeIds, double radius)
    : m_edgeIds(std::move(edgeIds)),
      m_radius(radius),
      m_featureID("fillet_" + std::to_string(s_nextID++)) {}

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
    auto result =
        model::FilletOp::execute(*inputSolid, m_edgeIds, m_radius, m_featureID, arcSegments());
    if (!result.solid) {
        return failWith(reason, result.errorMessage.empty() ? "the fillet could not be built"
                                                            : result.errorMessage);
    }
    return std::move(result.solid);
}

// ---------------------------------------------------------------------------
// ChamferFeature
// ---------------------------------------------------------------------------

int ChamferFeature::s_nextID = 1;

ChamferFeature::ChamferFeature(std::vector<topo::TopologyID> edgeIds, double distance)
    : m_edgeIds(std::move(edgeIds)),
      m_distance(distance),
      m_featureID("chamfer_" + std::to_string(s_nextID++)) {}

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
    return std::move(result.solid);
}

// ---------------------------------------------------------------------------
// BooleanFeature
// ---------------------------------------------------------------------------

int BooleanFeature::s_nextID = 1;

BooleanFeature::BooleanFeature(model::BooleanType type)
    : m_type(type), m_featureID("boolean_" + std::to_string(s_nextID++)) {}

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
        auto combined = model::BooleanOp::execute(*result, *bodies[i], m_type, &why);
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

int PatternFeature::s_nextID = 1;

std::unique_ptr<PatternFeature> PatternFeature::makeLinear(const math::Vec3& direction,
                                                           double spacing, int count,
                                                           std::vector<int> suppressed) {
    std::unique_ptr<PatternFeature> f(new PatternFeature());
    f->m_kind = Kind::Linear;
    f->m_vecA = direction;
    f->m_scalar = spacing;
    f->m_count = std::clamp(count, 1, kMaxPatternCount);
    f->m_suppressed = std::move(suppressed);
    f->m_featureID = "pattern_" + std::to_string(s_nextID++);
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
    f->m_featureID = "pattern_" + std::to_string(s_nextID++);
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

std::unique_ptr<topo::Solid> PatternFeature::execute(std::unique_ptr<topo::Solid> inputSolid,
                                                     std::string* reason) const {
    if (!inputSolid) return failWith(reason, "there is no body to pattern");
    auto solid = m_kind == Kind::Linear
                     ? model::Pattern::linear(*inputSolid, m_vecA, m_scalar, m_count, m_suppressed)
                     : model::Pattern::circular(*inputSolid, m_vecA, m_vecB, m_scalar, m_count,
                                                m_suppressed);
    if (!solid) return failWith(reason, "overlapping instances could not be merged into one body");
    return solid;
}

// ---------------------------------------------------------------------------
// PrimitiveFeature
// ---------------------------------------------------------------------------

int PrimitiveFeature::s_nextID = 1;

std::unique_ptr<PrimitiveFeature> PrimitiveFeature::makeBox(double width, double height,
                                                            double depth) {
    std::unique_ptr<PrimitiveFeature> f(new PrimitiveFeature());
    f->m_kind = Kind::Box;
    f->m_p0 = width;
    f->m_p1 = height;
    f->m_p2 = depth;
    f->m_featureID = "primitive_" + std::to_string(s_nextID++);
    return f;
}
std::unique_ptr<PrimitiveFeature> PrimitiveFeature::makeCylinder(double radius, double height) {
    std::unique_ptr<PrimitiveFeature> f(new PrimitiveFeature());
    f->m_kind = Kind::Cylinder;
    f->m_p0 = radius;
    f->m_p1 = height;
    f->m_featureID = "primitive_" + std::to_string(s_nextID++);
    return f;
}
std::unique_ptr<PrimitiveFeature> PrimitiveFeature::makeSphere(double radius) {
    std::unique_ptr<PrimitiveFeature> f(new PrimitiveFeature());
    f->m_kind = Kind::Sphere;
    f->m_p0 = radius;
    f->m_featureID = "primitive_" + std::to_string(s_nextID++);
    return f;
}
std::unique_ptr<PrimitiveFeature> PrimitiveFeature::makeCone(double bottomRadius, double topRadius,
                                                             double height) {
    std::unique_ptr<PrimitiveFeature> f(new PrimitiveFeature());
    f->m_kind = Kind::Cone;
    f->m_p0 = bottomRadius;
    f->m_p1 = topRadius;
    f->m_p2 = height;
    f->m_featureID = "primitive_" + std::to_string(s_nextID++);
    return f;
}
std::unique_ptr<PrimitiveFeature> PrimitiveFeature::makeTorus(double majorRadius,
                                                              double minorRadius) {
    std::unique_ptr<PrimitiveFeature> f(new PrimitiveFeature());
    f->m_kind = Kind::Torus;
    f->m_p0 = majorRadius;
    f->m_p1 = minorRadius;
    f->m_featureID = "primitive_" + std::to_string(s_nextID++);
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
    return solid;
}

int DatumFeature::s_nextID = 1;

std::unique_ptr<DatumFeature> DatumFeature::makePlane(const model::DatumPlane& plane) {
    std::unique_ptr<DatumFeature> f(new DatumFeature());
    f->m_kind = DatumKind::Plane;
    f->m_origin = plane.origin;
    f->m_dirA = plane.normal;
    f->m_dirB = plane.xAxis;
    f->m_featureID = "datum_" + std::to_string(s_nextID++);
    return f;
}

std::unique_ptr<DatumFeature> DatumFeature::makeAxis(const model::DatumAxis& axis) {
    std::unique_ptr<DatumFeature> f(new DatumFeature());
    f->m_kind = DatumKind::Axis;
    f->m_origin = axis.origin;
    f->m_dirA = axis.direction;
    f->m_featureID = "datum_" + std::to_string(s_nextID++);
    return f;
}

std::unique_ptr<DatumFeature> DatumFeature::makePoint(const model::DatumPoint& point) {
    std::unique_ptr<DatumFeature> f(new DatumFeature());
    f->m_kind = DatumKind::Point;
    f->m_origin = point.position;
    f->m_featureID = "datum_" + std::to_string(s_nextID++);
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

/// Run a feature, turning an exception from the kernel (the NURBS constructors
/// throw on invalid input, for one) into a failure with its reason, instead of
/// letting it unwind into the caller — ultimately the Qt event loop, which
/// cannot carry an exception and would terminate the application.
std::unique_ptr<topo::Solid> executeContained(const Feature& feat,
                                              std::unique_ptr<topo::Solid> input,
                                              std::string* reason = nullptr) {
    try {
        return feat.execute(std::move(input), reason);
    } catch (const std::exception& e) {
        if (reason) *reason = e.what();
    } catch (...) {
        if (reason) *reason = "unknown error";
    }
    return nullptr;
}

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
                                     std::unique_ptr<topo::Solid> tool, std::string* reason) {
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
        result = model::BooleanOp::execute(*part, *tool, type, &booleanReason);
    } catch (const std::exception& e) {
        if (reason) *reason = std::string("the Boolean failed: ") + e.what();
        return nullptr;
    }
    if (!result && reason) *reason = booleanReason;
    return result;
}

/// Whether a feature takes part in the build. Reference geometry (datums) has
/// no effect on the solid, and a suppressed feature is left out.
bool takesPart(const Feature& feature) {
    return !feature.isConstruction() && !feature.isSuppressed();
}

/// One step of the regeneration rule every build path shares: a creating
/// feature builds a tool body, combined with the part by its operation; any
/// other feature transforms the part.
std::unique_ptr<topo::Solid> applyFeature(const Feature& feature, std::unique_ptr<topo::Solid> part,
                                          std::string* reason) {
    if (!feature.createsNewBody()) return executeContained(feature, std::move(part), reason);
    auto tool = executeContained(feature, nullptr, reason);
    if (!tool) return nullptr;
    return combine(feature.operation(), std::move(part), std::move(tool), reason);
}

}  // namespace

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
    for (const auto& feat : m_features) {
        if (!takesPart(*feat)) continue;
        solid = applyFeature(*feat, std::move(solid), nullptr);
        if (!solid) {
            return nullptr;  // Feature failed
        }
    }
    return solid;
}

std::vector<std::unique_ptr<topo::Solid>> FeatureTree::buildBodies() const {
    std::vector<std::unique_ptr<topo::Solid>> bodies;
    for (const auto& feat : m_features) {
        if (!takesPart(*feat)) continue;

        if (feat->consumesAllBodies()) {
            // Boolean-style combine: replace the whole body list with its result.
            bodies = executeMultiContained(*feat, std::move(bodies));
        } else if (feat->createsNewBody() && feat->operation() != BodyOperation::NewBody &&
                   !bodies.empty()) {
            // Join / Cut / Intersect the active body, as the product path does.
            auto tool = executeContained(*feat, nullptr);
            if (!tool) continue;
            auto combined =
                combine(feat->operation(), std::move(bodies.back()), std::move(tool), nullptr);
            bodies.pop_back();
            if (combined) bodies.push_back(std::move(combined));
        } else if (feat->createsNewBody() || bodies.empty()) {
            // Start a fresh body. Create features ignore any input solid; a
            // transform with no active body (bodies.empty()) has nothing to act
            // on, so it too is executed against a null input and simply fails.
            auto solid = executeContained(*feat, nullptr);
            if (solid) {
                bodies.push_back(std::move(solid));
            }
        } else {
            // Transform the active (most-recently-created) body in place.
            auto solid = executeContained(*feat, std::move(bodies.back()));
            bodies.pop_back();
            if (solid) {
                bodies.push_back(std::move(solid));
            }
            // If the transform failed, the active body is dropped; the next
            // create feature starts a new one.
        }
    }
    return bodies;
}

BuildResult FeatureTree::buildWithDiagnostics() const {
    BuildResult result;
    if (m_features.empty()) {
        return result;
    }

    const int limit = (m_rollbackIndex >= 0)
                          ? std::min(m_rollbackIndex + 1, static_cast<int>(m_features.size()))
                          : static_cast<int>(m_features.size());

    std::unique_ptr<topo::Solid> solid;
    for (int i = 0; i < limit; ++i) {
        if (!takesPart(*m_features[static_cast<size_t>(i)])) {
            result.lastSuccessfulFeature = i;  // nothing to fail
            continue;
        }
        const Feature& feature = *m_features[static_cast<size_t>(i)];
        std::string reason;
        auto next = applyFeature(feature, std::move(solid), &reason);
        if (!next) {
            result.failedFeatureIndex = i;
            result.failureMessage = reason.empty()
                                        ? "Feature '" + feature.name() + "' failed to execute"
                                        : "Feature '" + feature.name() + "' failed: " + reason;
            return result;
        }
        solid = std::move(next);
        result.lastSuccessfulFeature = i;
    }

    result.solid = std::move(solid);
    return result;
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
