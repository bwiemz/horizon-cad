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
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/FacetResolution.h"
#include "horizon/modeling/FilletOp.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/ReferenceGeometry.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/modeling/SheetMetal.h"
#include "horizon/modeling/Sweep.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

namespace hz::doc {

class Sketch;
struct ExactCounterpart;

/// Why a part whose bodies would give it an exact counterpart is shown from its facets instead,
/// as a kind (Document::exactNoteKind): the window says it in its own words, translated, and the
/// note's detail (Document::exactNote) beside them. The kind is what is translated; the document
/// builds no words for it. The detail stays a technical reason in English, as an import's report
/// and a `problem` string are (Ruling 38): which face, and why, or the two volumes.
enum class ExactNoteKind {
    None,             ///< no note: the part is shown exactly, or has no counterpart by design
    ShownFromFacets,  ///< its exact form could not be had, meshed or measured
    NotChecked,       ///< its exact form could not be held to its facets (exactNote says why)
    Disagrees,        ///< its exact form measures otherwise than its facets
};

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

/// A face or an edge of the part a feature keeps by its name (format 32's migration): what an old
/// part's migration judges, and drops where the name can no longer mean what it meant
/// (io::migrateToExact).
struct StoredReference {
    enum class Kind { Face, Edge };
    Kind kind = Kind::Face;
    /// Its whole name (a TopologyID's tag); empty where none is kept there.
    std::string name;
    /// What it is to the feature, for a report: "the face it goes up to", "an edge it rounds".
    std::string what;
};

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
    /// Whether the build keeps the exact body this feature builds (stage 3): it wants the part's
    /// exact counterpart, the part is exact so far, and the feature makes a new body of it (or
    /// the first). An import copies its body as read for it only then (Ruling 20). A converted
    /// feature builds its exact body whatever this says: its faceted copy is cut from it.
    bool keepExact = false;
};

/// What a feature builds (stage 3): its body in facets, as every operation not yet converted
/// reads it, and, for a feature built exactly, the exact body those facets are the bridge copy of
/// (model::facetExact at the feature's facetResolution()). `exact` is null for a feature master's
/// construction still builds; both are null when the feature fails.
struct FeatureOutput {
    std::unique_ptr<topo::Solid> faceted;
    std::unique_ptr<topo::Solid> exact;
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

    /// executeIn(), with the exact body where the feature is built exactly (stage 3): the tree
    /// builds every feature this way. A converted feature builds its exact body, named for the
    /// feature, and gives its bridge copy (model::facetExact at facetResolution()) as `faceted`,
    /// which its executeIn() gives too. Where its exact body cannot be built it fails, with
    /// "could not be built exactly: <why>", and never builds master's faceted body in its place.
    /// By default: executeIn(), and no exact body.
    virtual FeatureOutput executeWithExact(const BuildContext& context,
                                           std::unique_ptr<topo::Solid> inputSolid,
                                           std::string* reason = nullptr) const {
        return {executeIn(context, std::move(inputSolid), reason), nullptr};
    }

    /// Master's faceting rule for this feature: how finely its faceted construction cut its
    /// curves, and so how finely the bridge cuts its exact body, for master's facet, edge and
    /// vertex counts. By default FacetResolution's own.
    virtual model::FacetResolution facetResolution() const { return {}; }

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
        Number,  ///< a plain number, of no unit (a sheet's K-factor)
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
    /// Every face and edge of the part it refers to by name (format 32's migration), in a fixed
    /// order: an extrusion's face to go up to; a hole's face, and its face to go up to; a
    /// mirror's face; a shell's removed faces; a fillet's or a chamfer's edges. One for each place
    /// it can keep one (an empty name: none kept there), and one for each of a list's. A face to
    /// go up to is one only while the extent goes up to a face: kept beside another extent, it
    /// refers to nothing. Its sketches' (the face one is on, the edges projected into one) are
    /// theirs, not its.
    virtual std::vector<StoredReference> storedReferences() const { return {}; }
    /// Keep @p name in place of storedReferences()[@p index]; an empty one drops it: a place
    /// keeps none, and a list is one shorter, so that the indices after it move down (drop from
    /// the last to the first). An index it has not is passed over.
    virtual void setStoredReference(std::size_t index, std::string name) {
        (void)index;
        (void)name;
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
    std::vector<StoredReference> storedReferences() const override;
    void setStoredReference(std::size_t index, std::string name) override;

    std::string name() const override;
    std::string featureID() const override;
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    /// Its extrusion in facets: under Stable naming, its exact extrusion's bridge copy
    /// (executeWithExact); under the older schemes, master's faceted extrusion.
    std::unique_ptr<topo::Solid> executeIn(const BuildContext& context,
                                           std::unique_ptr<topo::Solid> inputSolid,
                                           std::string* reason = nullptr) const override;
    /// Its plane and distance found from its extent as before; then, under Stable naming, built
    /// exactly (stage 3): model::Extrude::exactAndBridge, its faces named for this feature, and
    /// its bridge copy cut at facetResolution(), master's counts, numbered as master's prism
    /// numbered its facets, chords and seams (Ruling 21). An extrusion only the exact construction
    /// cannot build fails with "could not be built exactly: <why>". Under the older schemes,
    /// master's faceted extrusion and no exact body.
    FeatureOutput executeWithExact(const BuildContext& context,
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
    /// {segments(), chordTolerance()}: its profile's resolution.
    model::FacetResolution facetResolution() const override;

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
    /// Its revolve in facets: under Stable naming, its exact revolve's bridge copy
    /// (executeWithExact); under the older schemes, master's faceted revolve.
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    /// Its axis placed with its sketch as before; then, under Stable naming, built exactly (stage
    /// 3): model::Revolve::exactAndBridge, its faces named for this feature, and its bridge copy
    /// cut at facetResolution(), master's counts, numbered as master's revolve numbered its
    /// facets, chords and seams (Ruling 21). A revolve only the exact construction cannot build
    /// fails with "could not be built exactly: <why>". Under the older schemes, master's faceted
    /// revolve and no exact body.
    FeatureOutput executeWithExact(const BuildContext& context,
                                   std::unique_ptr<topo::Solid> inputSolid,
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
    /// {segments(), chordTolerance()}: its profile's resolution; and round its axis (as its
    /// sketch places it) segments(), master's one count for every rim (Ruling 24).
    model::FacetResolution facetResolution() const override;
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
    std::vector<StoredReference> storedReferences() const override;
    void setStoredReference(std::size_t index, std::string name) override;
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
    std::vector<StoredReference> storedReferences() const override;
    void setStoredReference(std::size_t index, std::string name) override;

    /// Chords across each blend arc, the "arcSegments" parameter.  A blend is
    /// faceted across its arc, so this is what decides how much material the
    /// fillet actually removes relative to the exact one.
    /// With a chord tolerance set, this is the count derived from it for the
    /// fillet radius at a square edge; each blend is then cut for the arc it
    /// spans, so a sharper edge's gets more.
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
    std::vector<StoredReference> storedReferences() const override;
    void setStoredReference(std::size_t index, std::string name) override;

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

    /// A linear pattern's second direction (SolidWorks' Direction 2): the
    /// first direction's row repeated @p count times, @p spacing apart along
    /// @p direction, making a grid. A count of 1 is none, as before. The
    /// grid's instances together are held to kMaxPatternCount: the count is
    /// cut to fit. False, and nothing changed, for a circular pattern or a
    /// direction or spacing that is not a finite number.
    bool setSecondDirection(const math::Vec3& direction, double spacing, int count);
    const math::Vec3& secondDirection() const { return m_vecC; }
    double secondSpacing() const { return m_scalar2; }
    int secondCount() const { return m_count2; }
    /// Whether copies clear of one another are combined with the part
    /// together, in one Boolean, rather than each in turn: the same part,
    /// in a fraction of the time for many copies, but split faces are cut
    /// into other pieces, and their pieces and edges named otherwise. A
    /// pattern made now does; one read from a file made before does not,
    /// so that what refers to its names finds them as it did.
    bool combinesTogether() const { return m_together; }
    void setCombinesTogether(bool together) { m_together = together; }
    /// How many instances in all, the first among them: count() by
    /// secondCount() for a grid. Instance k of a grid is column
    /// k % count() of row k / count().
    int instanceCount() const { return m_count * m_count2; }

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
    math::Vec3 m_vecC = math::Vec3::UnitY;  ///< Linear: the second direction.
    double m_scalar2 = 0;                   ///< Linear: the second spacing.
    int m_count2 = 1;                       ///< Linear: rows of the first; 1 is none.
    bool m_together = true;                 ///< see combinesTogether()
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
    std::vector<StoredReference> storedReferences() const override;
    void setStoredReference(std::size_t index, std::string name) override;

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
    /// The hole's body alone, to be cut: from the face of context.part (executeWithExact's
    /// faceted body).
    std::unique_ptr<topo::Solid> executeIn(const BuildContext& context,
                                           std::unique_ptr<topo::Solid> inputSolid,
                                           std::string* reason = nullptr) const override;
    /// Its half-section found from its face, its type and its extent as before, turned a whole
    /// turn about its axis: under Stable naming built exactly (stage 3, model::
    /// Revolve::exactAndBridge), its tool the bridge copy cut at facetResolution(), master's
    /// counts and numbers (Ruling 21); under the older schemes, master's faceted revolve and no
    /// exact body. Either way each surface is named for the line of the half-section it sweeps
    /// (`top`, `wall`, `bottom`, `bore`, `boreFloor`, `sink`), the exact body's too. A hole stays a
    /// Cut: the part after it is faceted, as on master.
    FeatureOutput executeWithExact(const BuildContext& context,
                                   std::unique_ptr<topo::Solid> inputSolid,
                                   std::string* reason = nullptr) const override;
    /// {segments, 0}, and segments round its axis (Ruling 24): its revolve's resolution. Its axis
    /// is where its face puts it, passed as it is built.
    model::FacetResolution facetResolution() const override;

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
    std::vector<StoredReference> storedReferences() const override;
    void setStoredReference(std::size_t index, std::string name) override;

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
/// Base flange (sheet metal): an open sketch of lines, end to end, folded into
/// a sheet of a thickness, each corner a bend of an inside radius, extruded
/// to a width along the sketch's normal. The lines are the virtual sharps of
/// the sheet's surface its thickness grows from; the material is to the left
/// of the lines as they run, or to the right when flipped. Its flat pattern
/// is strip() developed (model::developedLength, bendLinePositions).
///
/// Its names (Stable naming; Ruling 28), after its own sketch, so that they are
/// the same in every build and every session: with S a segment of the sketch
/// as the lines run, `e<id>` for a line and `e<id>.<k>` for a polyline's
/// segment k (as a profile names them),
///   - `<featureID>/flat:<S>`: the flat along S, its face on the sketch's side
///     (the surface the thickness grows from); `<featureID>/flat:<S>:back`, its
///     face a thickness away;
///   - `<featureID>/bend:<S>-<S'>` and `<featureID>/bend:<S>-<S'>:back`: the
///     bend round the corner from S to the next segment S', on the sketch's
///     side (its outside where it bends toward the material, its inside where
///     it bends away) and a thickness away;
///   - `<featureID>/start:<S>` and `<featureID>/end:<S>`: the ends across the
///     thickness, at the first segment's start and the last one's end;
///   - `<featureID>/cap_bottom` and `<featureID>/cap_top`: the faces at the
///     sketch and a width along its normal, as master named them.
/// Edges are named after the faces they part (model::nameEdgesLogically); the
/// bridge copy's facets, chords and seams after the face or edge they are of.
/// Master named every other face after a line it made anew in each build, a
/// name no two builds shared.
class SheetMetalFeature : public Feature {
public:
    SheetMetalFeature(std::shared_ptr<Sketch> sketch, double thickness, double bendRadius,
                      double width);

    std::string name() const override;
    std::string featureID() const override;
    void restoreFeatureID(const std::string& id) override;
    /// Its flange in facets: under Stable naming, its exact flange's bridge copy
    /// (executeWithExact); under the older schemes, master's faceted flange.
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    /// Under Stable naming, built exactly (stage 3): the strip's exact profile
    /// (model::SheetMetalSolid::profile), each bend an inside and an outside arc,
    /// laid on the sketch as master laid its cross-section (its flats master's
    /// very lines, in master's order, so the bridge's vertices and where its
    /// caps' loops start are master's), and extruded by model::Extrude::exact,
    /// so each bend is two cylinders; its faces named after the sketch (see the
    /// class), its edges after them, and its bridge copy cut at
    /// facetResolution() (model::buildWithBridge), its facets, chords and seams
    /// numbered as the bridge numbers them (`/facet:<k>`, `/chord:<k>`,
    /// `/seam:<k>`). One whose exact form cannot be built fails, with "could not
    /// be built exactly: <why>". Under the older schemes, master's faceted flange
    /// (each bend in segmentsPerBend() chords, every side face named after a
    /// line made in that build) and no exact body.
    FeatureOutput executeWithExact(const BuildContext& context,
                                   std::unique_ptr<topo::Solid> inputSolid,
                                   std::string* reason = nullptr) const override;
    /// {4 x segmentsPerBend(), 0}: a 90-degree bend is cut into master's
    /// segmentsPerBend() chords, a bend of another angle in proportion
    /// (FacetResolution::stepsFor), where master cut every bend into
    /// segmentsPerBend().
    model::FacetResolution facetResolution() const override;
    std::vector<std::shared_ptr<Sketch>> sketches() const override { return {m_sketch}; }
    bool createsNewBody() const override { return true; }
    /// thickness, bendRadius, kFactor, width, bendSegments (chords per bend) and
    /// flip (the material's side: 0 left of the lines, 1 right).
    std::map<std::string, double> parameters() const override;
    bool setParameter(const std::string& name, double value) override;
    ParameterKind parameterKind(const std::string& name) const override;
    std::vector<std::string> parameterChoices(const std::string& name) const override;

    const std::shared_ptr<Sketch>& sketch() const { return m_sketch; }
    model::SheetMetalParams params() const { return m_params; }
    double width() const { return m_width; }
    bool flipped() const { return m_flip; }
    int segmentsPerBend() const { return m_segments; }
    /// The strip its sketch folds into; nullopt, with why, when it folds into
    /// none (lines that do not join, too short for their bends...).
    std::optional<model::SheetMetalStrip> strip(std::string* why = nullptr) const;
    /// Its sketch's lines in order, end to end, in the sketch plane (the
    /// virtual sharps); empty, with why, when they are not one open chain.
    std::vector<math::Vec2> profile(std::string* why = nullptr) const;

private:
    std::shared_ptr<Sketch> m_sketch;
    model::SheetMetalParams m_params;
    double m_width;
    bool m_flip = false;
    int m_segments = 8;
    std::string m_featureID;

    static math::IdCounter<int> s_nextID;
};

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
    /// Its body in facets: under Stable naming, its exact body's bridge copy (executeWithExact);
    /// under the older schemes, master's faceted primitive.
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    /// Under Stable naming, built exactly (stage 3): the exact primitive (model::exact) in the
    /// frame it stands in, its faces named for this feature (`<featureID>/side`), its edges
    /// after them (model::nameEdgesLogically), and its bridge copy cut at facetResolution(),
    /// master's counts, its edges named by model::nameBridgeEdges and then, with its facets,
    /// numbered as master's faceted primitive numbered the same places (model::nameAsMasters,
    /// Ruling 21; an exact seam is renamed with them). Under the older schemes, master's faceted
    /// primitive and no exact body.
    FeatureOutput executeWithExact(const BuildContext& context,
                                   std::unique_ptr<topo::Solid> inputSolid,
                                   std::string* reason = nullptr) const override;
    /// {segments(), 0}, and for a sphere its rows from pole to pole, for a torus its count round
    /// its tube, at half the segments as master's PrimitiveFactory cut them (at least 2 and 3).
    model::FacetResolution facetResolution() const override;
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

    /// Facets around the axis, the "segments" parameter. Under Stable naming
    /// the primitive is exact (stage 3), and this shapes only its faceted copy,
    /// which the operations not yet converted read (facetResolution()); under
    /// the older schemes it is tessellated at construction, and this is the
    /// accuracy knob: their volumes converge to the analytic value from below
    /// as it rises. A box has no such knob and does not report the parameter.
    /// With a chord tolerance set, this is the count derived from it for the
    /// widest radius.
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

    /// Master's faceted primitive, named as its scheme names it (the Positional and FromGeometry
    /// schemes build it still).
    std::unique_ptr<topo::Solid> facetedAsMaster(std::string* reason) const;
    /// The turn and move that stand it where it goes: its z axis turned onto the axis direction
    /// (the shortest turn, half round for -Z), then moved to the base point.
    math::Mat4 placement() const;

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
    /// The body with its curved faces in facets, as every feature works on it
    /// (the faceted copy; Phase 141).
    std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> inputSolid,
                                         std::string* reason = nullptr) const override;
    /// execute()'s faceted copy, and, where the build keeps it (BuildContext::keepExact),
    /// exactBody() (null where its body is not exact).
    FeatureOutput executeWithExact(const BuildContext& context,
                                   std::unique_ptr<topo::Solid> inputSolid,
                                   std::string* reason = nullptr) const override;
    bool createsNewBody() const override { return true; }

    /// The body as read, exact, for the part's exact counterpart
    /// (BuildResult::exact): a copy whose faces are named as execute() names
    /// them, `<featureID>/face:<i>`, the logical face of their facets there,
    /// and whose edges are named by the same pass. So model::logicalEdge names
    /// one curve from its chords there and from its edge here, or from its
    /// pieces here where the body holds it in several (a rim a seam was
    /// inserted on): `<edge>/chord:<k>` on both sides. Null when not every face
    /// of it is exact: a body a part file kept as it was read before the exact
    /// kernel (note() says why).
    std::unique_ptr<topo::Solid> exactBody() const;

    const std::shared_ptr<const topo::Solid>& solid() const { return m_solid; }
    const std::string& source() const { return m_source; }

    /// What its import said of a body that is not exact, as the import's
    /// report said it (a part file saved before the exact kernel, whose body
    /// no exact form holds: `imported body "<source>": not exact (<why>);
    /// kept as it was`). Empty for an exact body. Not saved: it is said again
    /// whenever the body is read.
    const std::string& note() const { return m_note; }
    void setNote(std::string note) { m_note = std::move(note); }

private:
    /// Its faces named `<featureID>/face:<i>`, by their order in the solid.
    void nameFaces(topo::Solid& body) const;

    std::shared_ptr<const topo::Solid> m_solid;
    std::string m_source;
    std::string m_note;
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
    /// The part's exact counterpart (stage 2), the part built exactly (stage
    /// 3), beside `solid`, its faceted copy, which every feature, Boolean and
    /// reader of ideal labels not yet converted works on: while every feature
    /// that has made the part made a new body (or the first) and built it
    /// exactly (FeatureOutput::exact: an imported body as read,
    /// ImportedBodyFeature::exactBody; a Stable primitive), those exact bodies
    /// gathered as `solid` gathers their facets, named as its facets' logical
    /// faces and edges are, and meshed once for the display. Shared: a later
    /// build reuses it while the exact part it was made from is as it was
    /// (BuildOptions::previous). Null when the part has none by design (a
    /// feature not built exactly, or any other feature or a Boolean after
    /// one) or the build did not make one (BuildOptions::exactCounterpart).
    /// One that could not be had (an import not exact, a face that could not
    /// be meshed or measured, a volume other than its facets') holds no solid
    /// and says why. Document::exactSolid(), exactMesh() and exactNote() read
    /// it.
    std::shared_ptr<const ExactCounterpart> exact;
    int lastSuccessfulFeature = -1;
    std::string failureMessage;
    int failedFeatureIndex = -1;
    bool cancelled = false;  ///< stopped by BuildControl::cancel; no solid
    /// Built without the exact counterpart it would have had (BuildOptions::
    /// exactCounterpart false, the part made of bodies built exactly alone): a
    /// part to be built again before it is shown.
    bool withoutExact = false;
    /// Where each sketch that follows a face was placed (Phase 157), by id:
    /// what a build of a copy (on a worker) gives the document it copied.
    std::map<uint64_t, draft::SketchPlane> placements;
    /// The edges projected into each sketch (Phase 157), drawn again from the
    /// part, by sketch id: each entity keeps the id of the one it replaces.
    std::map<uint64_t, std::vector<std::shared_ptr<draft::DraftEntity>>> projections;
};

/// How a build running on another thread says how far it has got, and learns
/// it should stop. The build looks between features: a feature already
/// running finishes first. The part's exact counterpart, when the build makes
/// it, is a step of its own after the features (counted in `total` once it
/// begins), and stops between faces.
struct BuildControl {
    std::atomic<bool> cancel{false};
    std::atomic<int> done{0};   ///< features applied so far
    std::atomic<int> total{0};  ///< features the build will apply
};

/// What a build makes beside the part.
struct BuildOptions {
    /// Make the part's exact counterpart (BuildResult::exact). A reader that
    /// draws and measures the faceted part alone (an assembly's component)
    /// leaves it out, and does not pay for its mesh.
    bool exactCounterpart = true;
    /// A counterpart an earlier build made, reused where it was made from the
    /// exact part the part has now: a rebuild for any other reason meshes
    /// nothing again.
    std::shared_ptr<const ExactCounterpart> previous;
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
    /// Where the feature with `featureId` (Feature::featureID()) is, if it
    /// is there: what finds a feature again after a form has been open, in
    /// whose event loop a build may have taken it away.
    std::optional<size_t> indexOfId(const std::string& featureId) const;

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
    /// Respects the rollback index (features beyond it are skipped). Makes the
    /// part's exact counterpart as @p options says.
    BuildResult buildWithDiagnostics(BuildControl* control = nullptr,
                                     const BuildOptions& options = {}) const;

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
