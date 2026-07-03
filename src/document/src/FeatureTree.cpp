#include "horizon/document/FeatureTree.h"

#include <cassert>

#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/modeling/Draft.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/Loft.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/modeling/Shell.h"
#include "horizon/modeling/Sweep.h"

namespace hz::doc {

namespace {

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
    return {{"distance", m_distance}};
}

bool ExtrudeFeature::setParameter(const std::string& name, double value) {
    if (name == "distance" && value > 0.0) {
        m_distance = value;
        return true;
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

std::unique_ptr<topo::Solid> ExtrudeFeature::execute(
    std::unique_ptr<topo::Solid> /*inputSolid*/) const {
    // For now, extrude always creates a new solid from the sketch.
    // Boolean combination with inputSolid comes in a future phase.
    return model::Extrude::execute(m_sketch->entities(), m_sketch->plane(), m_direction, m_distance,
                                   m_featureID);
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
    return {{"angle", m_angle}};
}

bool RevolveFeature::setParameter(const std::string& name, double value) {
    if (name == "angle" && value > 0.0) {
        m_angle = value;
        return true;
    }
    return false;
}

std::string RevolveFeature::featureID() const {
    return m_featureID;
}

void RevolveFeature::restoreFeatureID(const std::string& id) {
    if (id.empty()) return;
    m_featureID = id;
    bumpCounter(s_nextID, id, "revolve_");
}

std::unique_ptr<topo::Solid> RevolveFeature::execute(
    std::unique_ptr<topo::Solid> /*inputSolid*/) const {
    return model::Revolve::execute(m_sketch->entities(), m_sketch->plane(), m_axisPoint, m_axisDir,
                                   m_angle, m_featureID);
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

std::unique_ptr<topo::Solid> LoftFeature::execute(
    std::unique_ptr<topo::Solid> /*inputSolid*/) const {
    std::vector<model::LoftSection> sections;
    sections.reserve(m_sections.size());
    for (const auto& sk : m_sections) {
        if (!sk) return nullptr;
        sections.push_back({sk->entities(), sk->plane()});
    }
    return model::Loft::execute(sections, m_featureID);
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
std::vector<math::Vec3> extractPathPoints(const Sketch& sketch) {
    const auto& plane = sketch.plane();
    std::vector<math::Vec2> pts2D;

    for (const auto& ent : sketch.entities()) {
        if (auto* pl = dynamic_cast<const draft::DraftPolyline*>(ent.get())) {
            for (const auto& p : pl->points()) pts2D.push_back(p);
            continue;
        }
        math::Vec2 s, e;
        if (auto* line = dynamic_cast<const draft::DraftLine*>(ent.get())) {
            s = line->start();
            e = line->end();
        } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(ent.get())) {
            s = arc->startPoint();
            e = arc->endPoint();
        } else {
            continue;
        }
        if (pts2D.empty()) {
            pts2D.push_back(s);
        } else {
            const double ds = (pts2D.back() - s).length();
            const double de = (pts2D.back() - e).length();
            if (de < ds) std::swap(s, e);
        }
        pts2D.push_back(e);
    }

    std::vector<math::Vec3> pts3D;
    pts3D.reserve(pts2D.size());
    for (const auto& p : pts2D) pts3D.push_back(plane.localToWorld(p));
    return pts3D;
}

}  // namespace

std::unique_ptr<topo::Solid> SweepFeature::execute(
    std::unique_ptr<topo::Solid> /*inputSolid*/) const {
    if (!m_profile || !m_path) return nullptr;
    std::vector<math::Vec3> pathPoints = extractPathPoints(*m_path);
    return model::Sweep::execute(m_profile->entities(), m_profile->plane(), pathPoints,
                                 m_featureID);
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

std::unique_ptr<topo::Solid> DraftFeature::execute(std::unique_ptr<topo::Solid> inputSolid) const {
    if (!inputSolid) return nullptr;
    return model::Draft::execute(std::move(inputSolid), m_pullDir, m_neutralPoint, m_angle);
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

std::unique_ptr<topo::Solid> ShellFeature::execute(std::unique_ptr<topo::Solid> inputSolid) const {
    if (!inputSolid) return nullptr;
    auto result = model::Shell::execute(std::move(inputSolid), m_thickness, m_removedFaceIds);
    return result.ok ? std::move(result.solid) : nullptr;
}

// ---------------------------------------------------------------------------
// FeatureTree
// ---------------------------------------------------------------------------

void FeatureTree::addFeature(std::unique_ptr<Feature> feature) {
    m_features.push_back(std::move(feature));
}

void FeatureTree::removeFeature(size_t index) {
    assert(index < m_features.size());
    m_features.erase(m_features.begin() + static_cast<ptrdiff_t>(index));
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
}

std::unique_ptr<topo::Solid> FeatureTree::build() const {
    if (m_features.empty()) {
        return nullptr;
    }

    std::unique_ptr<topo::Solid> solid;
    for (const auto& feat : m_features) {
        solid = feat->execute(std::move(solid));
        if (!solid) {
            return nullptr;  // Feature failed
        }
    }
    return solid;
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
        auto next = m_features[static_cast<size_t>(i)]->execute(std::move(solid));
        if (!next) {
            result.failedFeatureIndex = i;
            result.failureMessage =
                "Feature '" + m_features[static_cast<size_t>(i)]->name() + "' failed to execute";
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

    auto feat = std::move(m_features[static_cast<size_t>(fromIndex)]);
    m_features.erase(m_features.begin() + fromIndex);
    m_features.insert(m_features.begin() + toIndex, std::move(feat));
}

}  // namespace hz::doc
