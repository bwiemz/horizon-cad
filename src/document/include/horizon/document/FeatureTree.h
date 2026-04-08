#pragma once

#include "horizon/math/Vec3.h"
#include "horizon/topology/Solid.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace hz::doc {

class Sketch;

/// Abstract base class for parametric modeling features.
///
/// Each feature can produce a solid from an optional input solid.
/// The FeatureTree replays all features sequentially to rebuild the model.
class Feature {
public:
    virtual ~Feature() = default;

    /// Human-readable feature type name (e.g. "Extrude", "Revolve").
    virtual std::string name() const = 0;

    /// Unique identifier for TopologyID generation.
    virtual std::string featureID() const = 0;

    /// Execute this feature.
    /// @param inputSolid  The solid produced by the previous feature (nullptr for the first).
    /// @return The resulting solid, or nullptr on failure.
    virtual std::unique_ptr<topo::Solid> execute(
        std::unique_ptr<topo::Solid> inputSolid) const = 0;

    /// Return editable parameters as name/value pairs.
    virtual std::map<std::string, double> parameters() const { return {}; }

    /// Set a parameter by name.  Returns true if accepted.
    virtual bool setParameter(const std::string& name, double value) {
        (void)name;
        (void)value;
        return false;
    }
};

/// Extrude feature: creates a solid by extruding a sketch profile along a direction.
class ExtrudeFeature : public Feature {
public:
    ExtrudeFeature(std::shared_ptr<Sketch> sketch,
                   const math::Vec3& direction, double distance);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(
        std::unique_ptr<topo::Solid> inputSolid) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;

private:
    std::shared_ptr<Sketch> m_sketch;
    math::Vec3 m_direction;
    double m_distance;
    std::string m_featureID;

    static int s_nextID;
};

/// Revolve feature: creates a solid by revolving a sketch profile around an axis.
class RevolveFeature : public Feature {
public:
    RevolveFeature(std::shared_ptr<Sketch> sketch,
                   const math::Vec3& axisPoint, const math::Vec3& axisDir,
                   double angle);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(
        std::unique_ptr<topo::Solid> inputSolid) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;

private:
    std::shared_ptr<Sketch> m_sketch;
    math::Vec3 m_axisPoint;
    math::Vec3 m_axisDir;
    double m_angle;
    std::string m_featureID;

    static int s_nextID;
};

/// Result of building the feature tree with diagnostics.
struct BuildResult {
    std::unique_ptr<topo::Solid> solid;
    int lastSuccessfulFeature = -1;
    std::string failureMessage;
    int failedFeatureIndex = -1;
};

/// Ordered list of parametric features that can be replayed to rebuild a solid.
///
/// `build()` executes all features sequentially, passing each feature's output
/// as the next feature's input.  This is the foundation for parametric/history-
/// based modeling.
class FeatureTree {
public:
    FeatureTree() = default;

    /// Append a feature to the end of the tree.
    void addFeature(std::unique_ptr<Feature> feature);

    /// Remove the feature at the given index.
    void removeFeature(size_t index);

    /// Number of features in the tree.
    size_t featureCount() const;

    /// Access a feature by index (read-only).
    const Feature* feature(size_t index) const;

    /// Access a feature by index (mutable).
    Feature* feature(size_t index);

    /// Remove all features.
    void clear();

    /// Rebuild the solid by replaying all features from scratch.
    /// Returns nullptr if the tree is empty or any feature fails.
    std::unique_ptr<topo::Solid> build() const;

    /// Rebuild with diagnostics: records which feature failed and why.
    /// Respects the rollback index (features beyond it are skipped).
    BuildResult buildWithDiagnostics() const;

    /// Rollback index: features after this index are suppressed.
    /// -1 means no rollback (all features active).
    int rollbackIndex() const { return m_rollbackIndex; }
    void setRollbackIndex(int index) { m_rollbackIndex = index; }

    /// Move a feature from one position to another.
    void moveFeature(int fromIndex, int toIndex);

private:
    std::vector<std::unique_ptr<Feature>> m_features;
    int m_rollbackIndex = -1;
};

}  // namespace hz::doc
