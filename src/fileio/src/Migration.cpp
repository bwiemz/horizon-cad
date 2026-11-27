#include "horizon/fileio/Migration.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/FacetResolution.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/ProfileValidator.h"
#include "horizon/topology/Solid.h"

namespace hz::io {

namespace {

using doc::Feature;
using doc::StoredReference;

// Why a reference is not carried over: the class the migration finds it in.
constexpr const char* kSheetNamedAnew = "a sheet's faces are named anew";  // Ruling 28
constexpr const char* kNamedTheOldWay = "named the old way";
constexpr const char* kBuiltFromPieces = "built from pieces before";  // Ruling 23

/// Chords a turn from which a curve drawn with them looked round; a count under it looked like
/// intent (spec: Migration).
constexpr int kRoundFrom = 12;

/// The parts a name is made of, between its separators: its source (`extrude_3` in
/// `extrude_3/cap_top` and in `extrude_3~hole0/side:e5`), the sources of the faces an edge parts
/// (`edge:<a>|<b>`), the edge a blend is named after, the mirror an image is named for, and its
/// roles and numbers.
std::vector<std::string> partsOf(const std::string& name) {
    std::vector<std::string> parts;
    std::string part;
    for (const char c : name) {
        if (c == '/' || c == '|' || c == ':' || c == '~' || c == '#' || c == '.') {
            if (!part.empty()) parts.push_back(part);
            part.clear();
        } else {
            part += c;
        }
    }
    if (!part.empty()) parts.push_back(part);
    return parts;
}

bool holds(const std::string& name, const char* part) {
    return name.find(part) != std::string::npos;
}

/// The source master's faceted primitive of @p kind named its faces after under the old schemes,
/// every primitive of the kind alike (`box/top`; Stable names are the feature's own).
const char* oldSourceOf(doc::PrimitiveFeature::Kind kind) {
    switch (kind) {
        case doc::PrimitiveFeature::Kind::Box:
            return "box";
        case doc::PrimitiveFeature::Kind::Cylinder:
            return "cylinder";
        case doc::PrimitiveFeature::Kind::Sphere:
            return "sphere";
        case doc::PrimitiveFeature::Kind::Cone:
            return "cone";
        case doc::PrimitiveFeature::Kind::Torus:
            return "torus";
    }
    return "";
}

/// What the migration knows of a feature as a maker of names, found before it is named Stable.
struct Maker {
    bool old = false;     ///< named Positional or FromGeometry
    bool sheet = false;   ///< a sheet (Ruling 28)
    bool pieces = false;  ///< an extrusion or revolve of several loops: master's Booleans
};

/// Judges each name kept against the features that made the names it holds.
class Judge {
public:
    explicit Judge(const doc::FeatureTree& tree);

    /// Why @p name is not carried over; null when it is.
    const char* lost(const std::string& name);

private:
    std::vector<Maker> m_makers;
    /// The features each part of a name may be the source of: a feature's id, and an old
    /// primitive's (or prism shell's) shared source.
    std::map<std::string, std::vector<std::size_t>> m_byPart;
};

Judge::Judge(const doc::FeatureTree& tree) {
    const std::size_t count = tree.featureCount();
    m_makers.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        const Feature& feature = *tree.feature(i);
        Maker& maker = m_makers[i];
        maker.old = feature.naming() != model::NamingScheme::Stable;
        maker.sheet = dynamic_cast<const doc::SheetMetalFeature*>(&feature) != nullptr;
        m_byPart[feature.featureID()].push_back(i);
        if (maker.old) {
            if (const auto* primitive = dynamic_cast<const doc::PrimitiveFeature*>(&feature)) {
                m_byPart[oldSourceOf(primitive->kind())].push_back(i);
            }
            if (const auto* shell = dynamic_cast<const doc::ShellFeature*>(&feature);
                shell != nullptr && shell->method() == doc::ShellFeature::Method::Prism) {
                m_byPart["shell"].push_back(i);
            }
        }
        std::shared_ptr<doc::Sketch> profile;
        if (const auto* extrude = dynamic_cast<const doc::ExtrudeFeature*>(&feature)) {
            profile = extrude->sketch();
        }
        if (const auto* revolve = dynamic_cast<const doc::RevolveFeature*>(&feature)) {
            profile = revolve->sketch();
        }
        if (profile) {
            const model::ProfileRegions found =
                model::ProfileValidator::regions(profile->entities());
            maker.pieces = found.ok() && !found.isSingleLoop();
        }
    }
}

const char* Judge::lost(const std::string& name) {
    std::vector<std::size_t> makers;
    for (const std::string& part : partsOf(name)) {
        const auto found = m_byPart.find(part);
        if (found != m_byPart.end()) {
            makers.insert(makers.end(), found->second.begin(), found->second.end());
        }
    }
    // A name no feature of the part made is not the migration's to judge: it resolves as it did,
    // or does not, and the build says so.
    if (makers.empty()) return nullptr;
    bool sheet = false;
    bool old = false;
    bool pieces = false;
    for (const std::size_t i : makers) {
        const Maker& maker = m_makers[i];
        sheet = sheet || maker.sheet;
        old = old || maker.old;
        pieces = pieces || maker.pieces;
    }
    if (sheet) return kSheetNamedAnew;
    // A name an operation named the old ways made or rewrote carries its feature's id (a fillet's
    // `<fillet>/edge<N>`, an old primitive's `box/top`). One that carries a Stable feature's alone
    // was made by it, and a change named the old ways after it (a Positional Join) leaves it: it
    // means what it meant (the review of Task 11, finding 1).
    if (old) return kNamedTheOldWay;
    if (pieces && (holds(name, "/piece:") || holds(name, "~region") || holds(name, "/chord:") ||
                   holds(name, "/seam:"))) {
        return kBuiltFromPieces;
    }
    return nullptr;
}

/// What converted @p feature, drawn with fewer than kRoundFrom chords a turn before, was, in
/// words for what it is; nullopt for one drawn finer, or with nothing round.
std::optional<std::string> roundedOff(const Feature& feature) {
    const std::string roundNow = " before; round now";
    if (const auto* primitive = dynamic_cast<const doc::PrimitiveFeature*>(&feature)) {
        const int n = primitive->segments();
        if (!primitive->isFaceted() || n >= kRoundFrom) return std::nullopt;
        const std::string sides = std::to_string(n);
        switch (primitive->kind()) {
            case doc::PrimitiveFeature::Kind::Cylinder:
                return "drawn as a " + sides + "-sided prism" + roundNow;
            case doc::PrimitiveFeature::Kind::Cone:
                return "drawn as a " + sides + "-sided pyramid" + roundNow;
            default:
                return "drawn with " + sides + " sides a turn" + roundNow;
        }
    }
    if (const auto* extrude = dynamic_cast<const doc::ExtrudeFeature*>(&feature)) {
        if (!extrude->sketch() || !extrude->hasCurvedProfile()) return std::nullopt;
        // Each arc as master cut it: the count, or under a chord tolerance its own radius's.
        const model::FacetResolution resolution = extrude->facetResolution();
        int fewest = kRoundFrom;
        std::size_t inProfile = 0;
        bool circle = false;
        for (const auto& entity : extrude->sketch()->entities()) {
            if (entity->construction()) continue;
            ++inProfile;
            double radius = 0.0;
            if (const auto* arc = dynamic_cast<const draft::DraftArc*>(entity.get())) {
                radius = arc->radius();
            } else if (const auto* whole = dynamic_cast<const draft::DraftCircle*>(entity.get())) {
                radius = whole->radius();
                circle = true;
            } else {
                continue;
            }
            fewest = std::min(fewest, resolution.stepsFor(radius, math::kTwoPi));
        }
        if (fewest >= kRoundFrom) return std::nullopt;
        const std::string chords = std::to_string(fewest);
        if (inProfile == 1 && circle) return "drawn as a " + chords + "-sided prism" + roundNow;
        return "its arcs were drawn with " + chords + " chords a turn" + roundNow;
    }
    if (const auto* revolve = dynamic_cast<const doc::RevolveFeature*>(&feature)) {
        const int n = revolve->segments();
        if (n >= kRoundFrom) return std::nullopt;
        return "turned in " + std::to_string(n) + " steps a turn" + roundNow;
    }
    if (const auto* hole = dynamic_cast<const doc::HoleFeature*>(&feature)) {
        const int n = hole->facetResolution().segmentsPerTurn;
        if (n >= kRoundFrom) return std::nullopt;
        return "drawn as a " + std::to_string(n) + "-sided hole" + roundNow;
    }
    if (const auto* sheet = dynamic_cast<const doc::SheetMetalFeature*>(&feature)) {
        // Master cut every bend into segmentsPerBend() chords: a quarter turn's, four a turn.
        const int chords = sheet->segmentsPerBend();
        if (4 * chords >= kRoundFrom) return std::nullopt;
        const auto strip = sheet->strip();
        if (!strip || strip->bendAngles.empty()) return std::nullopt;  // no bend, nothing round
        return "its bends were drawn with " + std::to_string(chords) +
               (chords == 1 ? " chord each" : " chords each") + roundNow;
    }
    return std::nullopt;
}

/// "feature <k> (<its name>)": feature @p index of @p tree, as a report names it.
std::string ownerOf(const doc::FeatureTree& tree, std::size_t index) {
    return "feature " + std::to_string(index + 1) + " (" + tree.feature(index)->name() + ")";
}

std::string notCarried(const std::string& owner, const std::string& name, const std::string& what,
                       const char* why) {
    return owner + ": its reference to " + name + ", " + what + ", is not carried over (" + why +
           ")";
}

}  // namespace

ImportReport migrateToExact(doc::Document& doc, int fileVersion) {
    ImportReport report;
    if (fileVersion > kLastFacetedFormat) return report;
    // Its first build is its first exact one: what that fails on is said too (Document::migrated).
    doc.setMigrated(true);
    doc::FeatureTree& tree = doc.featureTree();
    const std::size_t count = tree.featureCount();
    Judge judge(tree);  // of the names as they were made, before any feature is named Stable

    // Each sketch's face and projected edges are found before the first feature made from it.
    std::map<const doc::Sketch*, std::size_t> firstUse;
    for (std::size_t i = 0; i < count; ++i) {
        for (const auto& sketch : tree.feature(i)->sketches()) {
            if (sketch) firstUse.emplace(sketch.get(), i);  // the first is kept
        }
    }

    bool changed = false;
    std::set<const doc::Sketch*> judged;
    const auto judgeSketch = [&](doc::Sketch& sketch, std::size_t holder) {
        if (!judged.insert(&sketch).second) return;
        const bool used = holder < count;
        const std::string owner = used ? ownerOf(tree, holder) : "sketch '" + sketch.name() + "'";
        const std::string& face = sketch.face();
        if (!face.empty()) {
            if (const char* why = judge.lost(face)) {
                report.skipped.push_back(notCarried(
                    owner, face,
                    used ? "the face sketch '" + sketch.name() + "' is on" : "the face it is on",
                    why));
                // It no longer follows the face, and stays where its last build placed it.
                sketch.setFace("");
                changed = true;
            }
        }
        for (const auto& entity : sketch.entities()) {
            const std::string& edge = entity->sourceEdge();
            if (edge.empty()) continue;
            if (const char* why = judge.lost(edge)) {
                report.skipped.push_back(
                    notCarried(owner, edge,
                               used ? "an edge projected into sketch '" + sketch.name() + "'"
                                    : "an edge projected into it",
                               why));
                // It is no longer drawn again from the edge, and stays as it was last drawn.
                entity->setSourceEdge("");
                changed = true;
            }
        }
    };

    for (std::size_t i = 0; i < count; ++i) {
        Feature& feature = *tree.feature(i);
        for (const auto& sketch : feature.sketches()) {
            if (sketch && firstUse.at(sketch.get()) == i) judgeSketch(*sketch, i);
        }
        const std::vector<StoredReference> references = feature.storedReferences();
        std::vector<std::size_t> dropped;
        for (std::size_t r = 0; r < references.size(); ++r) {
            const StoredReference& reference = references[r];
            if (reference.name.empty()) continue;
            if (const char* why = judge.lost(reference.name)) {
                report.skipped.push_back(
                    notCarried(ownerOf(tree, i), reference.name, reference.what, why));
                dropped.push_back(r);
            }
        }
        // From the last to the first: a list is one shorter after each.
        for (auto r = dropped.rbegin(); r != dropped.rend(); ++r)
            feature.setStoredReference(*r, "");
        changed = changed || !dropped.empty();
        if (auto rounded = roundedOff(feature)) {
            report.approximated.push_back(ownerOf(tree, i) + ": " + *rounded);
        }
    }
    // A sketch no feature is made from is placed by no build: judged against the whole part.
    for (const auto& sketch : doc.sketches()) {
        if (sketch) judgeSketch(*sketch, count);
    }

    for (std::size_t i = 0; i < count; ++i) {
        Feature& feature = *tree.feature(i);
        if (feature.naming() == model::NamingScheme::Stable) continue;
        feature.setNaming(model::NamingScheme::Stable);
        changed = true;
    }
    if (changed) tree.markChanged();
    return report;
}

}  // namespace hz::io
