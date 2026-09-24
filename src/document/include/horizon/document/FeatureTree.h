#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "horizon/math/IdCounter.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/FilletOp.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/ReferenceGeometry.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/modeling/Sweep.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

namespace hz::doc {

class Sketch;

/// Upper bounds on feature resolution and repetition. Feature parameters come
/// from files as well as from the UI, and every step or instance costs memory
/// and time in every later operation, so past these a value is clamped rather
/// than trusted. (A count of 1e9 from a file would otherwise try to allocate
/// it on open.)
inline constexpr int kMaxFacetSegments = 4096;  ///< steps per full turn; see segmentsForTolerance
inline constexpr int kMaxArcSegments = 1024;    ///< chords across one fillet arc
inline constexpr int kMaxPatternCount = 10000;  ///< instances in one pattern

/// How a feature that builds solid geometry (`createsNewBody()`) combines it
/// with the part built so far.
enum class BodyOperation {
    NewBody,    ///< Keep it as a separate body alongside the others.
    Join,       ///< Union it with the part.
    Cut,        ///< Subtract it from the part.
    Intersect,  ///< Keep only the material both share.
};

/// The persisted spelling: "new", "join", "cut" or "intersect".
const char* bodyOperationName(BodyOperation operation);
/// Parse a persisted spelling; nullopt when it is not one of the four.
std::optional<BodyOperation> bodyOperationFromName(std::string_view name);

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
    /// @param reason      When given, receives why the feature failed — in the
    ///                    user's terms, e.g. "the profile has a gap: nothing
    ///                    continues from (3, 4)".
    /// @return The resulting solid, or nullptr on failure.
    virtual std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                                 std::string* reason = nullptr) const = 0;

    /// True for non-geometric construction features (datum planes, axes,
    /// points). The feature tree skips these when building the solid, so they
    /// can appear anywhere — including before any solid exists — without
    /// affecting the body.
    virtual bool isConstruction() const { return false; }

    /// True if this feature starts a new body rather than transforming the
    /// current one. Create features (primitives, extrude, revolve, loft, sweep)
    /// return true; transforms (fillet, chamfer, shell, draft, pattern) return
    /// false. Used by `buildBodies()` for multi-body trees; the single-solid
    /// `build()` ignores it.
    virtual bool createsNewBody() const { return false; }

    /// True if this feature consumes the entire current body list and replaces
    /// it (e.g. a Boolean combine, which needs two or more operands). When true,
    /// `buildBodies()` routes the feature through `executeMulti()` instead of the
    /// single-solid `execute()`. The single-solid `build()` ignores such
    /// features (via `execute()` returning its input unchanged).
    virtual bool consumesAllBodies() const { return false; }

    /// Multi-body execution: given all currently live bodies, return the
    /// replacement body list. Default is identity (no change). Only called by
    /// `buildBodies()` when `consumesAllBodies()` is true.
    virtual std::vector<std::unique_ptr<topo::Solid>> executeMulti(
        std::vector<std::unique_ptr<topo::Solid>> bodies) const {
        return bodies;
    }

    /// Return editable parameters as name/value pairs.
    virtual std::map<std::string, double> parameters() const { return {}; }

    /// Set a parameter by name.  Returns true if accepted.
    virtual bool setParameter(const std::string& name, double value) {
        (void)name;
        (void)value;
        return false;
    }

    /// How the body this feature builds combines with the part so far.
    /// Meaningful only when `createsNewBody()`; a new feature starts a
    /// separate body.
    BodyOperation operation() const { return m_operation; }
    void setOperation(BodyOperation operation) { m_operation = operation; }

    /// A suppressed feature stays in the history but takes no part in the
    /// build, as if it were not there.
    bool isSuppressed() const { return m_suppressed; }
    void setSuppressed(bool suppressed) { m_suppressed = suppressed; }

    /// How the faces and edges this feature builds — or the Join / Cut /
    /// Intersect it makes — are named. A new feature names them from what
    /// generated them; one loaded from a file saved before persistent naming
    /// keeps the positional names its references were made against.
    model::NamingScheme naming() const { return m_naming; }
    void setNaming(model::NamingScheme naming) { m_naming = naming; }

private:
    BodyOperation m_operation = BodyOperation::NewBody;
    bool m_suppressed = false;
    model::NamingScheme m_naming = model::NamingScheme::FromGeometry;
};

/// Extrude feature: creates a solid by extruding a sketch profile along a direction.
class ExtrudeFeature : public Feature {
public:
    ExtrudeFeature(std::shared_ptr<Sketch> sketch, const math::Vec3& direction, double distance);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;

    const std::shared_ptr<Sketch>& sketch() const { return m_sketch; }
    bool createsNewBody() const override { return true; }
    const math::Vec3& direction() const { return m_direction; }
    double distance() const { return m_distance; }
    void restoreFeatureID(const std::string& id) override;

    /// Chords per full turn of the profile's arcs and circles, the "segments"
    /// parameter.  Reported only when the profile has any: a polygon extrudes
    /// exactly.
    int segments() const { return m_segments; }
    /// Chord-sag budget, the "chordTolerance" parameter: when positive each
    /// arc is faceted at the count its own radius needs.  0 uses the count.
    double chordTolerance() const { return m_chordTolerance; }
    /// Whether the profile has an arc or circle, and so a resolution.
    bool hasCurvedProfile() const;

private:
    std::shared_ptr<Sketch> m_sketch;
    math::Vec3 m_direction;
    double m_distance;
    int m_segments = model::Extrude::kDefaultSegments;
    double m_chordTolerance = 0.0;
    std::string m_featureID;

    static math::IdCounter<int> s_nextID;
};

/// Revolve feature: creates a solid by revolving a sketch profile around an axis.
class RevolveFeature : public Feature {
public:
    RevolveFeature(std::shared_ptr<Sketch> sketch, const math::Vec3& axisPoint,
                   const math::Vec3& axisDir, double angle);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;

    const std::shared_ptr<Sketch>& sketch() const { return m_sketch; }
    const math::Vec3& axisPoint() const { return m_axisPoint; }
    bool createsNewBody() const override { return true; }
    const math::Vec3& axisDir() const { return m_axisDir; }
    double angle() const { return m_angle; }

    /// Angular steps per full turn, the "segments" parameter.  This is the
    /// accuracy knob: a revolve is faceted, so its volume converges to the
    /// exact value from below as this rises.
    /// With a chord tolerance set, this is the count derived from it for the
    /// profile's widest radius.
    int segments() const;
    /// Chord-sag budget, the "chordTolerance" parameter.  When positive, the
    /// facet count is derived from it and the governing radius on every
    /// rebuild, so accuracy is a distance that holds at any size rather than a
    /// count whose error grows with the radius.  0 means the count is used as
    /// given.  Setting the count explicitly returns to count mode.
    double chordTolerance() const { return m_chordTolerance; }
    void restoreFeatureID(const std::string& id) override;

private:
    std::shared_ptr<Sketch> m_sketch;
    math::Vec3 m_axisPoint;
    math::Vec3 m_axisDir;
    double m_angle;
    int m_segments = model::Revolve::kDefaultSegments;
    double m_chordTolerance = 0.0;
    std::string m_featureID;

    static math::IdCounter<int> s_nextID;
};

/// Loft feature: creates a solid by interpolating through ordered sketch
/// sections (each a closed profile on its own plane).
class LoftFeature : public Feature {
public:
    explicit LoftFeature(std::vector<std::shared_ptr<Sketch>> sections);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    void restoreFeatureID(const std::string& id) override;

    bool createsNewBody() const override { return true; }
    const std::vector<std::shared_ptr<Sketch>>& sections() const { return m_sections; }

private:
    std::vector<std::shared_ptr<Sketch>> m_sections;
    std::string m_featureID;

    static math::IdCounter<int> s_nextID;
};

/// Sweep feature: creates a solid by transporting a profile sketch along a
/// path sketch (open polyline). Translation transport (Era-2 scope).
class SweepFeature : public Feature {
public:
    SweepFeature(std::shared_ptr<Sketch> profile, std::shared_ptr<Sketch> path);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;
    void restoreFeatureID(const std::string& id) override;

    const std::shared_ptr<Sketch>& profile() const { return m_profile; }
    bool createsNewBody() const override { return true; }
    const std::shared_ptr<Sketch>& path() const { return m_path; }

    /// Steps per full turn used to sample arcs in the path, the "segments"
    /// parameter.  A straight path is exact and ignores it.
    /// Also used for arcs in the profile.  With a chord tolerance set, each
    /// arc is instead sampled at the count its own radius needs.
    int segments() const { return m_segments; }
    /// Chord-sag budget, the "chordTolerance" parameter.  When positive, the
    /// facet count is derived from it and the governing radius on every
    /// rebuild, so accuracy is a distance that holds at any size rather than a
    /// count whose error grows with the radius.  0 means the count is used as
    /// given.  Setting the count explicitly returns to count mode.
    double chordTolerance() const { return m_chordTolerance; }

private:
    std::shared_ptr<Sketch> m_profile;
    std::shared_ptr<Sketch> m_path;
    int m_segments = model::Sweep::kDefaultArcSegments;
    double m_chordTolerance = 0.0;
    std::string m_featureID;

    static math::IdCounter<int> s_nextID;
};

/// Draft feature: tapers the input solid's lateral faces about a neutral
/// plane. Consumes the previous feature's solid.
class DraftFeature : public Feature {
public:
    DraftFeature(const math::Vec3& pullDir, const math::Vec3& neutralPoint, double angle);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
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

    static math::IdCounter<int> s_nextID;
};

/// Shell feature: hollows the input solid to a thin wall, removing the given
/// faces (by TopologyID). Consumes the previous feature's solid.
class ShellFeature : public Feature {
public:
    ShellFeature(double thickness, std::vector<topo::TopologyID> removedFaceIds);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;
    void restoreFeatureID(const std::string& id) override;

    double thickness() const { return m_thickness; }
    const std::vector<topo::TopologyID>& removedFaceIds() const { return m_removedFaceIds; }

private:
    double m_thickness;
    std::vector<topo::TopologyID> m_removedFaceIds;
    std::string m_featureID;

    static math::IdCounter<int> s_nextID;
};

/// Fillet feature: rounds the given edges (by TopologyID) of the input solid
/// with a constant radius. Consumes the previous feature's solid.
class FilletFeature : public Feature {
public:
    FilletFeature(std::vector<topo::TopologyID> edgeIds, double radius);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;
    void restoreFeatureID(const std::string& id) override;

    double radius() const { return m_radius; }
    const std::vector<topo::TopologyID>& edgeIds() const { return m_edgeIds; }

    /// Chords across each blend arc, the "arcSegments" parameter.  A blend is
    /// faceted across its arc, so this is what decides how much material the
    /// fillet actually removes relative to the exact one.
    /// With a chord tolerance set, this is the count derived from it for the
    /// fillet radius.
    int arcSegments() const;
    /// Chord-sag budget, the "chordTolerance" parameter.  When positive, the
    /// chord count is derived from it and the governing radius on every
    /// rebuild, so accuracy is a distance that holds at any size rather than a
    /// count whose error grows with the radius.  0 means the count is used as
    /// given.  Setting the count explicitly returns to count mode.
    double chordTolerance() const { return m_chordTolerance; }

private:
    std::vector<topo::TopologyID> m_edgeIds;
    double m_radius;
    int m_arcSegments = model::FilletOp::kDefaultArcSegments;
    double m_chordTolerance = 0.0;
    std::string m_featureID;

    static math::IdCounter<int> s_nextID;
};

/// Chamfer feature: bevels the given edges (by TopologyID) of the input solid
/// by an equal setback distance on both faces. Consumes the previous solid.
class ChamferFeature : public Feature {
public:
    ChamferFeature(std::vector<topo::TopologyID> edgeIds, double distance);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;
    void restoreFeatureID(const std::string& id) override;

    double distance() const { return m_distance; }
    const std::vector<topo::TopologyID>& edgeIds() const { return m_edgeIds; }

private:
    std::vector<topo::TopologyID> m_edgeIds;
    double m_distance;
    std::string m_featureID;

    static math::IdCounter<int> s_nextID;
};

/// Boolean feature: combines the part's bodies into one by a Union, Subtract
/// or Intersect, folded in body order (for Subtract, the first body is the
/// one cut from and every later body is cut from it). On the product path the
/// bodies are the separate shells of the part's solid (see
/// `BodyOperation::NewBody`); `buildBodies()` drives the same fold through
/// `executeMulti()`. With fewer than two bodies there is nothing to combine
/// and the part passes through unchanged.
class BooleanFeature : public Feature {
public:
    explicit BooleanFeature(model::BooleanType type);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;
    void restoreFeatureID(const std::string& id) override;

    bool consumesAllBodies() const override { return true; }
    std::vector<std::unique_ptr<topo::Solid>> executeMulti(
        std::vector<std::unique_ptr<topo::Solid>> bodies) const override;

    model::BooleanType booleanType() const { return m_type; }

private:
    model::BooleanType m_type;
    std::string m_featureID;

    static math::IdCounter<int> s_nextID;
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
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;
    void restoreFeatureID(const std::string& id) override;

    Kind kind() const { return m_kind; }
    const math::Vec3& vecA() const { return m_vecA; }
    const math::Vec3& vecB() const { return m_vecB; }
    double scalar() const { return m_scalar; }
    int count() const { return m_count; }
    /// Instance indices the pattern skips (not to be confused with
    /// suppressing the whole feature, `isSuppressed()`).
    const std::vector<int>& suppressedInstances() const { return m_suppressed; }

private:
    PatternFeature() = default;

    Kind m_kind = Kind::Linear;
    math::Vec3 m_vecA;    ///< Linear: direction. Circular: axis point.
    math::Vec3 m_vecB;    ///< Linear: unused. Circular: axis direction.
    double m_scalar = 0;  ///< Linear: spacing. Circular: angle step (rad).
    int m_count = 1;
    std::vector<int> m_suppressed;
    std::string m_featureID;

    static math::IdCounter<int> s_nextID;
};

/// Primitive feature: creates a solid primitive (box, cylinder, sphere, cone,
/// torus). A base feature — it ignores the input solid — so toolbar primitives
/// become parametric, editable tree features that persist and rebuild instead
/// of one-shot scene edits.
class PrimitiveFeature : public Feature {
public:
    enum class Kind { Box, Cylinder, Sphere, Cone, Torus };

    static std::unique_ptr<PrimitiveFeature> makeBox(double width, double height, double depth);
    static std::unique_ptr<PrimitiveFeature> makeCylinder(double radius, double height);
    static std::unique_ptr<PrimitiveFeature> makeSphere(double radius);
    static std::unique_ptr<PrimitiveFeature> makeCone(double bottomRadius, double topRadius,
                                                      double height);
    static std::unique_ptr<PrimitiveFeature> makeTorus(double majorRadius, double minorRadius);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;
    void restoreFeatureID(const std::string& id) override;

    Kind kind() const { return m_kind; }
    bool createsNewBody() const override { return true; }
    double p0() const { return m_p0; }  ///< Box:width Cyl:radius Sph:radius Cone:botR Torus:majR
    double p1() const { return m_p1; }  ///< Box:height Cyl:height Cone:topR Torus:minR
    double p2() const { return m_p2; }  ///< Box:depth Cone:height

    /// Facets around the axis, the "segments" parameter.  Curved primitives
    /// are tessellated at construction, so this is the accuracy knob: their
    /// volumes converge to the analytic value from below as it rises.  A box
    /// has no such knob and does not report the parameter.  With a chord
    /// tolerance set, this is the count derived from it for the widest radius.
    int segments() const;
    /// Chord-sag budget, the "chordTolerance" parameter.  When positive, the
    /// facet count is derived from it and the governing radius on every
    /// rebuild, so accuracy is a distance that holds at any size rather than a
    /// count whose error grows with the radius.  0 means the count is used as
    /// given.  Setting the count explicitly returns to count mode.
    double chordTolerance() const { return m_chordTolerance; }

    /// Whether this kind is faceted, and so has a "segments" parameter.
    bool isFaceted() const { return m_kind != Kind::Box; }

private:
    PrimitiveFeature() = default;

    Kind m_kind = Kind::Box;
    double m_p0 = 1.0;
    double m_p1 = 1.0;
    double m_p2 = 1.0;
    int m_segments = model::PrimitiveFactory::kDefaultSegments;
    double m_chordTolerance = 0.0;
    std::string m_featureID;

    static math::IdCounter<int> s_nextID;
};

/// A body brought in from another file (a STEP import): fixed geometry, with
/// no sketch or parameters behind it. It builds a body like any other and
/// takes the usual body operation. Its faces are named by their order in the
/// solid, which never changes, and its edges after their faces.
class ImportedBodyFeature : public Feature {
public:
    /// @param source  Where it came from, for the user: "bracket.step".
    ImportedBodyFeature(std::shared_ptr<const topo::Solid> solid, std::string source);

    std::string name() const override { return "Imported"; }
    std::string featureID() const override { return m_featureID; }
    void restoreFeatureID(const std::string& id) override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    bool createsNewBody() const override { return true; }

    const std::shared_ptr<const topo::Solid>& solid() const { return m_solid; }
    const std::string& source() const { return m_source; }

private:
    std::shared_ptr<const topo::Solid> m_solid;
    std::string m_source;
    std::string m_featureID;

    static math::IdCounter<int> s_nextID;
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
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
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

    static math::IdCounter<int> s_nextID;
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

    /// Insert a feature before `index` (at the end when `index` is past it).
    /// An insertion at or before the rollback index moves the index with the
    /// feature it pointed at.
    void insertFeature(size_t index, std::unique_ptr<Feature> feature);

    /// Remove the feature at the given index.
    void removeFeature(size_t index);

    /// Remove the feature at `index` and hand it back. Removing at or before
    /// the rollback index moves the index back by one, so the features before
    /// it stay active; it clears once no feature is left.
    std::unique_ptr<Feature> takeFeature(size_t index);

    /// Where `feature` is in the tree, if it is there.
    std::optional<size_t> indexOf(const Feature* feature) const;

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

    /// Rebuild as a multi-body model: each `createsNewBody()` feature starts a
    /// new body; transforms modify the current (most-recently-created) body,
    /// following the standard "active body" convention. Construction features
    /// (datums) are skipped. A feature that fails to execute drops the current
    /// body and starts fresh. Returns one solid per surviving body.
    std::vector<std::unique_ptr<topo::Solid>> buildBodies() const;

    /// Rebuild with diagnostics: records which feature failed and why.
    /// Respects the rollback index (features beyond it are skipped).
    BuildResult buildWithDiagnostics() const;

    /// Rollback index: features after this index are suppressed.
    /// -1 means no rollback (all features active).
    int rollbackIndex() const { return m_rollbackIndex; }
    void setRollbackIndex(int index);

    /// Move a feature from one position to another. The rollback index moves
    /// as for a removal followed by an insertion: the other features stay on
    /// their side of it.
    void moveFeature(int fromIndex, int toIndex);

    /// Changes whenever anything that affects the build changes: features
    /// added, removed or moved, the rollback index, or — through
    /// `markChanged()` — a feature edited in place. Compare two readings to
    /// learn whether the model needs rebuilding.
    uint64_t revision() const { return m_revision; }

    /// Record an in-place edit of a feature (a parameter, its operation, its
    /// suppression), which the tree cannot see for itself.
    void markChanged() { ++m_revision; }

private:
    std::vector<std::unique_ptr<Feature>> m_features;
    int m_rollbackIndex = -1;
    uint64_t m_revision = 0;
};

}  // namespace hz::doc
