#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "horizon/math/Vec3.h"
#include "horizon/modeling/ReferenceGeometry.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

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

    /// Restore a persisted feature ID (used by file loaders). Face
    /// TopologyIDs derive from this ID, so it must survive save/load or
    /// every mate and downstream reference breaks.
    virtual void restoreFeatureID(const std::string& id) { (void)id; }

    /// Execute this feature.
    /// @param inputSolid  The solid produced by the previous feature (nullptr for the first).
    /// @return The resulting solid, or nullptr on failure.
    virtual std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid) const = 0;

    /// True for non-geometric construction features (datum planes, axes,
    /// points). The feature tree skips these when building the solid, so they
    /// can appear anywhere — including before any solid exists — without
    /// affecting the body.
    virtual bool isConstruction() const { return false; }

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
    ExtrudeFeature(std::shared_ptr<Sketch> sketch, const math::Vec3& direction, double distance);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;

    const std::shared_ptr<Sketch>& sketch() const { return m_sketch; }
    const math::Vec3& direction() const { return m_direction; }
    double distance() const { return m_distance; }
    void restoreFeatureID(const std::string& id) override;

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
    RevolveFeature(std::shared_ptr<Sketch> sketch, const math::Vec3& axisPoint,
                   const math::Vec3& axisDir, double angle);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;

    const std::shared_ptr<Sketch>& sketch() const { return m_sketch; }
    const math::Vec3& axisPoint() const { return m_axisPoint; }
    const math::Vec3& axisDir() const { return m_axisDir; }
    double angle() const { return m_angle; }
    void restoreFeatureID(const std::string& id) override;

private:
    std::shared_ptr<Sketch> m_sketch;
    math::Vec3 m_axisPoint;
    math::Vec3 m_axisDir;
    double m_angle;
    std::string m_featureID;

    static int s_nextID;
};

/// Loft feature: creates a solid by interpolating through ordered sketch
/// sections (each a closed profile on its own plane).
class LoftFeature : public Feature {
public:
    explicit LoftFeature(std::vector<std::shared_ptr<Sketch>> sections);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid) const override;
    void restoreFeatureID(const std::string& id) override;

    const std::vector<std::shared_ptr<Sketch>>& sections() const { return m_sections; }

private:
    std::vector<std::shared_ptr<Sketch>> m_sections;
    std::string m_featureID;

    static int s_nextID;
};

/// Sweep feature: creates a solid by transporting a profile sketch along a
/// path sketch (open polyline). Translation transport (Era-2 scope).
class SweepFeature : public Feature {
public:
    SweepFeature(std::shared_ptr<Sketch> profile, std::shared_ptr<Sketch> path);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid) const override;
    void restoreFeatureID(const std::string& id) override;

    const std::shared_ptr<Sketch>& profile() const { return m_profile; }
    const std::shared_ptr<Sketch>& path() const { return m_path; }

private:
    std::shared_ptr<Sketch> m_profile;
    std::shared_ptr<Sketch> m_path;
    std::string m_featureID;

    static int s_nextID;
};

/// Draft feature: tapers the input solid's lateral faces about a neutral
/// plane. Consumes the previous feature's solid.
class DraftFeature : public Feature {
public:
    DraftFeature(const math::Vec3& pullDir, const math::Vec3& neutralPoint, double angle);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;
    void restoreFeatureID(const std::string& id) override;

    const math::Vec3& pullDir() const { return m_pullDir; }
    const math::Vec3& neutralPoint() const { return m_neutralPoint; }
    double angle() const { return m_angle; }

private:
    math::Vec3 m_pullDir;
    math::Vec3 m_neutralPoint;
    double m_angle;
    std::string m_featureID;

    static int s_nextID;
};

/// Shell feature: hollows the input solid to a thin wall, removing the given
/// faces (by TopologyID). Consumes the previous feature's solid.
class ShellFeature : public Feature {
public:
    ShellFeature(double thickness, std::vector<topo::TopologyID> removedFaceIds);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;
    void restoreFeatureID(const std::string& id) override;

    double thickness() const { return m_thickness; }
    const std::vector<topo::TopologyID>& removedFaceIds() const { return m_removedFaceIds; }

private:
    double m_thickness;
    std::vector<topo::TopologyID> m_removedFaceIds;
    std::string m_featureID;

    static int s_nextID;
};

/// Pattern feature: replicates the input solid linearly or circularly.
/// Consumes the previous feature's solid.
class PatternFeature : public Feature {
public:
    enum class Kind { Linear, Circular };

    /// Linear pattern constructor.
    static std::unique_ptr<PatternFeature> makeLinear(const math::Vec3& direction, double spacing,
                                                      int count, std::vector<int> suppressed = {});
    /// Circular pattern constructor.
    static std::unique_ptr<PatternFeature> makeCircular(const math::Vec3& axisPoint,
                                                        const math::Vec3& axisDir,
                                                        double angleStepRad, int count,
                                                        std::vector<int> suppressed = {});

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;
    void restoreFeatureID(const std::string& id) override;

    Kind kind() const { return m_kind; }
    const math::Vec3& vecA() const { return m_vecA; }
    const math::Vec3& vecB() const { return m_vecB; }
    double scalar() const { return m_scalar; }
    int count() const { return m_count; }
    const std::vector<int>& suppressed() const { return m_suppressed; }

private:
    PatternFeature() = default;

    Kind m_kind = Kind::Linear;
    math::Vec3 m_vecA;    ///< Linear: direction. Circular: axis point.
    math::Vec3 m_vecB;    ///< Linear: unused. Circular: axis direction.
    double m_scalar = 0;  ///< Linear: spacing. Circular: angle step (rad).
    int m_count = 1;
    std::vector<int> m_suppressed;
    std::string m_featureID;

    static int s_nextID;
};

/// Reference-geometry feature: a datum plane, axis, or point. Non-geometric —
/// it lives in the feature tree as construction geometry that sketches and
/// features reference, but does not alter the solid body.
class DatumFeature : public Feature {
public:
    enum class DatumKind { Plane, Axis, Point };

    static std::unique_ptr<DatumFeature> makePlane(const model::DatumPlane& plane);
    static std::unique_ptr<DatumFeature> makeAxis(const model::DatumAxis& axis);
    static std::unique_ptr<DatumFeature> makePoint(const model::DatumPoint& point);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid) const override;
    bool isConstruction() const override { return true; }
    void restoreFeatureID(const std::string& id) override;

    DatumKind datumKind() const { return m_kind; }
    model::DatumPlane asPlane() const;
    model::DatumAxis asAxis() const;
    model::DatumPoint asPoint() const;

    // Uniform storage slots (also the serialization shape).
    const math::Vec3& origin() const { return m_origin; }
    const math::Vec3& dirA() const { return m_dirA; }  ///< Plane: normal. Axis: direction.
    const math::Vec3& dirB() const { return m_dirB; }  ///< Plane: xAxis.

private:
    DatumFeature() = default;

    DatumKind m_kind = DatumKind::Plane;
    math::Vec3 m_origin;
    math::Vec3 m_dirA;
    math::Vec3 m_dirB;
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
