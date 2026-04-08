#include "horizon/document/FeatureTree.h"

#include "horizon/document/Sketch.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/Revolve.h"

#include <cassert>

namespace hz::doc {

// ---------------------------------------------------------------------------
// ExtrudeFeature
// ---------------------------------------------------------------------------

int ExtrudeFeature::s_nextID = 1;

ExtrudeFeature::ExtrudeFeature(std::shared_ptr<Sketch> sketch,
                               const math::Vec3& direction, double distance)
    : m_sketch(std::move(sketch)),
      m_direction(direction),
      m_distance(distance),
      m_featureID("extrude_" + std::to_string(s_nextID++)) {}

std::string ExtrudeFeature::name() const { return "Extrude"; }

std::string ExtrudeFeature::featureID() const { return m_featureID; }

std::unique_ptr<topo::Solid> ExtrudeFeature::execute(
    std::unique_ptr<topo::Solid> /*inputSolid*/) const {
    // For now, extrude always creates a new solid from the sketch.
    // Boolean combination with inputSolid comes in a future phase.
    return model::Extrude::execute(m_sketch->entities(), m_sketch->plane(),
                                    m_direction, m_distance, m_featureID);
}

// ---------------------------------------------------------------------------
// RevolveFeature
// ---------------------------------------------------------------------------

int RevolveFeature::s_nextID = 1;

RevolveFeature::RevolveFeature(std::shared_ptr<Sketch> sketch,
                               const math::Vec3& axisPoint,
                               const math::Vec3& axisDir, double angle)
    : m_sketch(std::move(sketch)),
      m_axisPoint(axisPoint),
      m_axisDir(axisDir),
      m_angle(angle),
      m_featureID("revolve_" + std::to_string(s_nextID++)) {}

std::string RevolveFeature::name() const { return "Revolve"; }

std::string RevolveFeature::featureID() const { return m_featureID; }

std::unique_ptr<topo::Solid> RevolveFeature::execute(
    std::unique_ptr<topo::Solid> /*inputSolid*/) const {
    return model::Revolve::execute(m_sketch->entities(), m_sketch->plane(),
                                    m_axisPoint, m_axisDir, m_angle, m_featureID);
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

size_t FeatureTree::featureCount() const { return m_features.size(); }

const Feature* FeatureTree::feature(size_t index) const {
    assert(index < m_features.size());
    return m_features[index].get();
}

void FeatureTree::clear() { m_features.clear(); }

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

}  // namespace hz::doc
