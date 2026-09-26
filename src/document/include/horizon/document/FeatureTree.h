#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/SketchPlane.h"
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
class Feature;

/// What a feature is built against: the part as it stands before it, and
/// the features applied before it, in order. Through-all needs the one, a
/// pattern of features the other.
struct BuildContext {
    const topo::Solid* part = nullptr;
    std::vector<const Feature*> before;
};

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

    /// execute(), knowing what it is built against. The tree builds every
    /// feature this way; a feature that needs the part or the features
    /// before it overrides it, and execute() alone builds it against nothing.
    virtual std::unique_ptr<topo::Solid> executeIn(const BuildContext& context,
                                                   std::unique_ptr<topo::Solid> inputSolid,
                                                   std::string* reason = nullptr) const {
        (void)context;
        return execute(std::move(inputSolid), reason);
    }

    /// The sketches it is made from (Phase 157). Before it builds the
    /// feature, the tree places each that follows a face on that face, and
    /// projects again the part's edges projected into it, from the part as
    /// it stands before the feature.
    virtual std::vector<std::shared_ptr<Sketch>> sketches() const { return {}; }

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

    /// How a parameter is read and shown.
    enum class ParameterKind {
        Length,  ///< a distance, in the model's millimetres
        Angle,   ///< kept in radians; shown and typed in degrees
        Count,   ///< a whole number
        Choice,  ///< the code of one of a few named choices (a Boolean's operation)
    };
    /// The names of a Choice parameter's values, in code order.
    virtual std::vector<std::string> parameterChoices(const std::string& name) const;

    /// The kind of parameter @p name: by default an "angle" is an angle,
    /// "segments", "arcSegments" and "count" are counts, "operation" a
    /// choice, and anything else a length.
    virtual ParameterKind parameterKind(const std::string& name) const;

    /// The directions and points that place the feature, in world
    /// coordinates: an extrusion's direction, a revolve's axis. A point's
    /// name ends in "Point"; every other one is a direction.
    virtual std::map<std::string, math::Vec3> vectors() const { return {}; }
    /// Set one. Returns false for a value the feature cannot use (a
    /// direction of no length, an extrusion along its own sketch).
    virtual bool setVector(const std::string& name, const math::Vec3& value) {
        (void)name;
        (void)value;
        return false;
    }
    /// What it refers to by name (Phase 157): an extrusion's face to go up
    /// to, by its whole name (model::wholeFaceName()), "" for none. Each is a
    /// face of the part.
    virtual std::map<std::string, std::string> references() const { return {}; }
    /// Refer to @p value as @p name. Returns false for a name it has not.
    virtual bool setReference(const std::string& name, const std::string& value) {
        (void)name;
        (void)value;
        return false;
    }

    /// Whether vector @p name is a point (not a direction).
    static bool isPoint(const std::string& name) {
        return name.size() >= 5 && name.compare(name.size() - 5, 5, "Point") == 0;
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

    /// Parameters given as expressions of the document's variables (Phase
    /// 155), by name, each as it is kept ("(wall * 2)"). The parameter
    /// itself holds the value it last worked out to, which a file keeps
    /// too, for a build that reads no expressions.
    const std::map<std::string, std::string>& parameterExpressions() const { return m_expressions; }
    /// Parameter @p name given as @p expression; an empty one: its number
    /// again.
    void setParameterExpression(const std::string& name, const std::string& expression) {
        if (expression.empty()) {
            m_expressions.erase(name);
        } else {
            m_expressions[name] = expression;
        }
    }
    void setParameterExpressions(std::map<std::string, std::string> expressions) {
        m_expressions = std::move(expressions);
    }
    /// Why one of its expressions could not be worked out when the part
    /// was last built; empty when each was. The build fails at it.
    const std::string& expressionError() const { return m_expressionError; }
    void setExpressionError(std::string error) { m_expressionError = std::move(error); }

private:
    BodyOperation m_operation = BodyOperation::NewBody;
    bool m_suppressed = false;
    model::NamingScheme m_naming = model::NamingScheme::Stable;
    std::map<std::string, std::string> m_expressions;
    std::string m_expressionError;
};

/// Extrude feature: creates a solid by extruding a sketch profile along a direction.
class ExtrudeFeature : public Feature {
public:
    ExtrudeFeature(std::shared_ptr<Sketch> sketch, const math::Vec3& direction, double distance);

    /// How far it goes: the distance along the direction; half of it each
    /// way; through the part (one way, or both), whatever the distance; or up
    /// to a flat face of the part parallel to the sketch (Phase 157).
    enum class Extent { Blind, Symmetric, ThroughAll, ThroughAllBoth, UpToFace };
    Extent extent() const { return m_extent; }
    void setExtent(Extent extent) { m_extent = extent; }
    /// The face it goes up to, by its whole name; "" for none. Used only
    /// when the extent is UpToFace. A reference, "upToFace".
    const std::string& upToFace() const { return m_upToFace; }
    void setUpToFace(std::string face) { m_upToFace = std::move(face); }
    std::map<std::string, std::string> references() const override {
        return {{"upToFace", m_upToFace}};
    }
    bool setReference(const std::string& name, const std::string& value) override;

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    std::unique_ptr<topo::Solid> executeIn(const BuildContext& context,
                                           std::unique_ptr<topo::Solid> inputSolid,
                                           std::string* reason = nullptr) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;
    ParameterKind parameterKind(const std::string& name) const override;
    std::vector<std::string> parameterChoices(const std::string& name) const override;
    std::map<std::string, math::Vec3> vectors() const override;
    bool setVector(const std::string& name, const math::Vec3& value) override;

    const std::shared_ptr<Sketch>& sketch() const { return m_sketch; }
    std::vector<std::shared_ptr<Sketch>> sketches() const override { return {m_sketch}; }
    bool createsNewBody() const override { return true; }
    /// Which way it goes, as the sketch was drawn: a sketch placed on a face
    /// that has turned takes it along (Sketch::placement()).
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
    Extent m_extent = Extent::Blind;
    std::string m_upToFace;
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
    std::map<std::string, math::Vec3> vectors() const override;
    bool setVector(const std::string& name, const math::Vec3& value) override;

    const std::shared_ptr<Sketch>& sketch() const { return m_sketch; }
    std::vector<std::shared_ptr<Sketch>> sketches() const override { return {m_sketch}; }
    /// The axis, as the sketch was drawn: a sketch placed on a face takes it
    /// along (Sketch::placement()).
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
    std::vector<std::shared_ptr<Sketch>> sketches() const override { return m_sections; }

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
    std::vector<std::shared_ptr<Sketch>> sketches() const override { return {m_profile, m_path}; }

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
    std::map<std::string, math::Vec3> vectors() const override;
    bool setVector(const std::string& name, const math::Vec3& value) override;
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
    /// How it hollows the part. Offset (Phase 163): each face moved inward,
    /// any shape of one body, the part's faces and their names kept. Prism:
    /// as a shell read from a file older than that was built, a right prism
    /// only, so it builds as it did.
    enum class Method { Prism, Offset };

    ShellFeature(double thickness, std::vector<topo::TopologyID> removedFaceIds,
                 Method method = Method::Offset);

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;
    void restoreFeatureID(const std::string& id) override;

    double thickness() const { return m_thickness; }
    const std::vector<topo::TopologyID>& removedFaceIds() const { return m_removedFaceIds; }
    Method method() const { return m_method; }

private:
    double m_thickness;
    std::vector<topo::TopologyID> m_removedFaceIds;
    Method m_method;
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
    /// With targets, the pattern repeats what those features add or cut, not
    /// the whole part: each target's body, moved to each instance and
    /// combined as the target combines.
    std::unique_ptr<topo::Solid> executeIn(const BuildContext& context,
                                           std::unique_ptr<topo::Solid> inputSolid,
                                           std::string* reason = nullptr) const override;
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;
    std::map<std::string, math::Vec3> vectors() const override;
    bool setVector(const std::string& name, const math::Vec3& value) override;
    ParameterKind parameterKind(const std::string& name) const override;
    void restoreFeatureID(const std::string& id) override;

    /// The features it repeats, by featureID: none repeats the whole part.
    const std::vector<std::string>& targets() const { return m_targets; }
    /// Each feature is repeated once, however often it is listed.
    void setTargets(std::vector<std::string> targets);
    /// Where instance @p k is moved to from instance 0.
    math::Mat4 instanceTransform(int k) const;

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

    std::vector<std::string> m_targets;
    Kind m_kind = Kind::Linear;
    math::Vec3 m_vecA;    ///< Linear: direction. Circular: axis point.
    math::Vec3 m_vecB;    ///< Linear: unused. Circular: axis direction.
    double m_scalar = 0;  ///< Linear: spacing. Circular: angle step (rad).
    int m_count = 1;
    std::vector<int> m_suppressed;
    std::string m_featureID;

    static math::IdCounter<int> s_nextID;
};

/// Mirror feature (Phase 162): the part, or what some features add or cut,
/// mirrored in a plane and joined to it as each joins. The plane is a point
/// and a normal, or a flat face of the part followed by its whole name, as a
/// sketch on a face is. A mirror image faces out as the original does
/// (model::Pattern::transformed reverses it), so it joins as any body does.
class MirrorFeature : public Feature {
public:
    /// About the plane through @p planePoint facing @p planeNormal.
    static std::unique_ptr<MirrorFeature> make(const math::Vec3& planePoint,
                                               const math::Vec3& planeNormal);

    std::string name() const override { return "Mirror"; }
    std::string featureID() const override { return m_featureID; }
    void restoreFeatureID(const std::string& id) override;
    /// The whole part, about its point and normal (no face: that needs the
    /// part it is on, which executeIn has).
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    /// With targets, each one's own body mirrored and combined as it
    /// combines; with none, the whole part, joined to its image.
    std::unique_ptr<topo::Solid> executeIn(const BuildContext& context,
                                           std::unique_ptr<topo::Solid> inputSolid,
                                           std::string* reason = nullptr) const override;
    /// "planePoint" and "planeNormal".
    std::map<std::string, math::Vec3> vectors() const override;
    bool setVector(const std::string& name, const math::Vec3& value) override;
    /// "planeFace": the flat face mirrored in, by its whole name; empty for
    /// the plane its point and normal give.
    std::map<std::string, std::string> references() const override {
        return {{"planeFace", m_face}};
    }
    bool setReference(const std::string& name, const std::string& value) override;

    const std::vector<std::string>& targets() const { return m_targets; }
    /// Each feature is mirrored once, however often it is listed.
    void setTargets(std::vector<std::string> targets);
    const math::Vec3& planePoint() const { return m_point; }
    const math::Vec3& planeNormal() const { return m_normal; }
    const std::string& planeFace() const { return m_face; }
    /// The mirror for @p part: in its face, when it has one (none, and why,
    /// for a face gone or no longer flat), else in its point and normal.
    std::optional<math::Mat4> mirrorIn(const topo::Solid& part, std::string* why) const;

private:
    MirrorFeature() = default;
    /// @p image's faces and edges named as this mirror's image.
    void nameImage(topo::Solid& image) const;

    std::vector<std::string> m_targets;
    math::Vec3 m_point;
    math::Vec3 m_normal = math::Vec3::UnitX;
    std::string m_face;
    std::string m_featureID;

    static math::IdCounter<int> s_nextID;
};

/// Hole feature (Phase 162): a drilled hole, simple, counterbored or
/// countersunk, into a flat face of the part at a point, as deep as its
/// depth, through all, or up to a face. It follows its face by the face's
/// whole name, as a sketch on a face does: the point is projected onto the
/// face where each build finds it.
///
/// Built as a half-section revolved about the hole's axis, so its walls are
/// true cylinders and cones (STEP writes them as designed, a drawing draws
/// their centre lines). It is a body cut from the part: the tree subtracts
/// it, and a pattern or a mirror of it repeats it.
class HoleFeature : public Feature {
public:
    enum class Type { Simple, Counterbore, Countersink };
    enum class Extent { Blind, ThroughAll, UpToFace };

    /// A simple blind hole into @p face (a whole face name) at @p position,
    /// @p diameter wide and @p depth deep, its point 118 degrees.
    static std::unique_ptr<HoleFeature> make(const std::string& face, const math::Vec3& position,
                                             double diameter, double depth);

    std::string name() const override { return "Hole"; }
    std::string featureID() const override { return m_featureID; }
    void restoreFeatureID(const std::string& id) override;
    bool createsNewBody() const override { return true; }
    /// Needs the part it is in (executeIn): alone, it says so.
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    /// The hole's body alone, to be cut: from the face of context.part.
    std::unique_ptr<topo::Solid> executeIn(const BuildContext& context,
                                           std::unique_ptr<topo::Solid> inputSolid,
                                           std::string* reason = nullptr) const override;

    /// "type" and "extent" (choices), "diameter", "depth", "boreDiameter",
    /// "boreDepth", "sinkDiameter", "sinkAngle" and "pointAngle" (0: a flat
    /// bottom) in radians, and "segments".
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;
    ParameterKind parameterKind(const std::string& name) const override;
    std::vector<std::string> parameterChoices(const std::string& name) const override;
    /// "positionPoint": where it is, projected onto its face.
    std::map<std::string, math::Vec3> vectors() const override {
        return {{"positionPoint", m_position}};
    }
    bool setVector(const std::string& name, const math::Vec3& value) override;
    /// "face": the face it is drilled into; "upToFace": the one it goes up
    /// to, for that extent.
    std::map<std::string, std::string> references() const override {
        return {{"face", m_face}, {"upToFace", m_upToFace}};
    }
    bool setReference(const std::string& name, const std::string& value) override;

    Type type() const { return m_type; }
    Extent extent() const { return m_extent; }
    const std::string& face() const { return m_face; }
    const std::string& upToFace() const { return m_upToFace; }
    const math::Vec3& position() const { return m_position; }

private:
    HoleFeature() = default;

    std::string m_face;
    std::string m_upToFace;
    math::Vec3 m_position;
    Type m_type = Type::Simple;
    Extent m_extent = Extent::Blind;
    double m_diameter = 5.0;
    double m_depth = 10.0;
    double m_boreDiameter = 9.0;
    double m_boreDepth = 3.0;
    double m_sinkDiameter = 10.0;
    double m_sinkAngle = 1.5707963267948966;   ///< 90 degrees
    double m_pointAngle = 2.0594885173533086;  ///< 118 degrees
    int m_segments = 32;
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
    /// Where it stands (Phase 134): its base point, and the way its own z
    /// axis (a cylinder's or cone's axis, a box's height) points. By default
    /// at the origin, along +Z.
    std::map<std::string, math::Vec3> vectors() const override;
    bool setVector(const std::string& name, const math::Vec3& value) override;
    const math::Vec3& basePoint() const { return m_basePoint; }
    const math::Vec3& axisDirection() const { return m_axisDirection; }
    /// Whether it stands anywhere but at the origin along +Z.
    bool isPlaced() const;
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
    math::Vec3 m_basePoint{0.0, 0.0, 0.0};
    math::Vec3 m_axisDirection{0.0, 0.0, 1.0};
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
    /// The part. When a feature fails, the part as it stands before that
    /// feature (null if nothing before it made a solid), so the user sees
    /// what they had, not an empty viewport.
    std::unique_ptr<topo::Solid> solid;
    int lastSuccessfulFeature = -1;
    std::string failureMessage;
    int failedFeatureIndex = -1;
    bool cancelled = false;  ///< stopped by BuildControl::cancel; no solid
    /// Where each sketch that follows a face was placed (Phase 157), by id:
    /// what a build of a copy (on a worker) gives the document it copied.
    std::map<uint64_t, draft::SketchPlane> placements;
    /// The edges projected into each sketch (Phase 157), drawn again from the
    /// part, by sketch id: each entity keeps the id of the one it replaces.
    std::map<uint64_t, std::vector<std::shared_ptr<draft::DraftEntity>>> projections;
};

/// How a build running on another thread says how far it has got, and learns
/// it should stop. The build looks between features: a feature already
/// running finishes first.
struct BuildControl {
    std::atomic<bool> cancel{false};
    std::atomic<int> done{0};   ///< features applied so far
    std::atomic<int> total{0};  ///< features the build will apply
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
    BuildResult buildWithDiagnostics(BuildControl* control = nullptr) const;

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
    /// Apply the features before @p limit again, for the part as it stood
    /// before a feature that failed. Null if cancelled or if nothing made a
    /// solid.
    std::unique_ptr<topo::Solid> replayUpTo(int limit, BuildControl* control) const;
    std::vector<std::unique_ptr<Feature>> m_features;
    int m_rollbackIndex = -1;
    uint64_t m_revision = 0;
};

}  // namespace hz::doc
