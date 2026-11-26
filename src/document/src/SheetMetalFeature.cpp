// The base flange (sheet metal): an open sketch of lines folded into a sheet.
// The fold is model::SheetMetalSolid's cross-section, laid on the sketch and
// extruded along its normal, so the flange is named and placed as an
// extrusion of its sketch is. Under Stable naming it is built exactly (stage
// 3): the fold's exact profile, each bend two arcs, so each bend is two
// cylinders, its flats master's own lines (Ruling 26).

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/math/MathUtils.h"
#include "horizon/modeling/ExactBuild.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/FacetResolution.h"
#include "horizon/modeling/SheetMetal.h"
#include "horizon/modeling/SheetMetalSolid.h"

namespace hz::doc {

namespace {

constexpr const char* kPrefix = "sheetmetal_";
/// No more chords than this to a bend: past it a bend is round to the eye.
constexpr int kMaxSegments = 64;

/// Two sketch points within this of each other are one (model mm).
constexpr double kJoin = 1e-6;

bool same(const math::Vec2& a, const math::Vec2& b) {
    return std::abs(a.x - b.x) <= kJoin && std::abs(a.y - b.y) <= kJoin;
}

/// How the strip's cross-section is laid on the sketch: the strip runs from the origin along +x,
/// its material toward +y; laid from the sketch's first point along its first line, its material
/// to the left of the lines, or (flipped) the right.
struct Laying {
    math::Vec2 origin;
    math::Vec2 along;
    math::Vec2 side;

    math::Vec2 operator()(const math::Vec2& q) const {
        return {origin.x + along.x * q.x + side.x * q.y, origin.y + along.y * q.x + side.y * q.y};
    }
};

/// The flange's exact outline on the sketch (stage 3, Ruling 26), in master's order: @p bent, the
/// strip's exact profile (model::SheetMetalSolid::profile, in crossSection's order and frame),
/// each flat and end replaced by master's own line along it, from @p outline (master's
/// cross-section at @p perBend chords a bend, laid and made in master's order, which a flipped
/// flange runs the other way round), and each bend's arc laid by @p lay. Master's lines keep
/// their ids, so the faces of the flats are named as master's were; the arcs are entities made
/// after them. Empty, with why, where a flat of @p bent is not one of master's lines.
std::vector<std::shared_ptr<draft::DraftEntity>> exactOutline(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& bent,
    const std::vector<std::shared_ptr<draft::DraftEntity>>& outline, int perBend, bool flipped,
    const Laying& lay, std::string& why) {
    const std::size_t n = outline.size();
    std::vector<std::shared_ptr<draft::DraftEntity>> laid;
    laid.reserve(bent.size());
    std::size_t edge = 0;  // the first of crossSection's edges the piece spans
    for (const auto& piece : bent) {
        if (auto* arc = dynamic_cast<draft::DraftArc*>(piece.get())) {
            // Mirrored, an arc turns the other way round: its start is its end.
            const math::Vec2 centre = lay(arc->center());
            const math::Vec2 start = lay(flipped ? arc->endPoint() : arc->startPoint());
            const math::Vec2 end = lay(flipped ? arc->startPoint() : arc->endPoint());
            arc->setCenter(centre);
            arc->setStartAngle(
                math::normalizeAngle(std::atan2(start.y - centre.y, start.x - centre.x)));
            arc->setEndAngle(math::normalizeAngle(std::atan2(end.y - centre.y, end.x - centre.x)));
            laid.push_back(piece);
            edge += static_cast<std::size_t>(perBend);
            continue;
        }
        const auto* line = dynamic_cast<const draft::DraftLine*>(piece.get());
        if (line == nullptr || edge >= n) {
            why = "the sheet's exact profile does not follow its cross-section";
            return {};
        }
        // Master's line along that edge: flipped, master's polygon ran backwards from its last
        // corner but one, its closing line still last.
        const std::size_t k = !flipped || edge + 1 == n ? edge : n - 2 - edge;
        const auto* masters = dynamic_cast<const draft::DraftLine*>(outline[k].get());
        const math::Vec2 a = lay(line->start());
        const math::Vec2 b = lay(line->end());
        if (masters == nullptr || !((same(masters->start(), a) && same(masters->end(), b)) ||
                                    (same(masters->start(), b) && same(masters->end(), a)))) {
            why = "the sheet's exact profile does not follow its cross-section";
            return {};
        }
        laid.push_back(outline[k]);
        ++edge;
    }
    if (edge != n || laid.empty()) {
        why = "the sheet's exact profile does not follow its cross-section";
        return {};
    }
    // Master's polygon, flipped, ran from the top's first flat back to the bottom's, then its
    // near end.
    if (flipped) std::reverse(laid.begin(), laid.end() - 1);
    return laid;
}

}  // namespace

math::IdCounter<int> SheetMetalFeature::s_nextID{1};

SheetMetalFeature::SheetMetalFeature(std::shared_ptr<Sketch> sketch, double thickness,
                                     double bendRadius, double width)
    : m_sketch(std::move(sketch)),
      m_params{thickness, bendRadius, 0.44},
      m_width(width),
      m_featureID(kPrefix + std::to_string(s_nextID.next())) {}

std::string SheetMetalFeature::name() const {
    return "Base Flange";
}

std::string SheetMetalFeature::featureID() const {
    return m_featureID;
}

void SheetMetalFeature::restoreFeatureID(const std::string& id) {
    if (id.empty()) return;
    m_featureID = id;
    // The counter kept ahead of a restored id, so a new flange never takes it.
    if (id.rfind(kPrefix, 0) == 0) {
        int n = 0;
        const std::string prefix(kPrefix);
        if (std::from_chars(id.data() + prefix.size(), id.data() + id.size(), n).ec ==
            std::errc()) {
            s_nextID.reserveThrough(n);
        }
    }
}

std::map<std::string, double> SheetMetalFeature::parameters() const {
    return {{"thickness", m_params.thickness},
            {"bendRadius", m_params.bendRadius},
            {"kFactor", m_params.kFactor},
            {"width", m_width},
            {"bendSegments", static_cast<double>(m_segments)},
            {"flip", m_flip ? 1.0 : 0.0}};
}

bool SheetMetalFeature::setParameter(const std::string& name, double value) {
    if (!std::isfinite(value)) return false;
    if (name == "thickness") {
        if (!(value > 0.0)) return false;
        m_params.thickness = value;
    } else if (name == "bendRadius") {
        if (value < 0.0) return false;
        m_params.bendRadius = value;
    } else if (name == "kFactor") {
        if (value < 0.0 || value > 1.0) return false;
        m_params.kFactor = value;
    } else if (name == "width") {
        if (!(value > 0.0)) return false;
        m_width = value;
    } else if (name == "bendSegments") {
        if (value < 1.0) return false;
        m_segments = value >= kMaxSegments ? kMaxSegments : static_cast<int>(value);
    } else if (name == "flip") {
        if (value != 0.0 && value != 1.0) return false;
        m_flip = value == 1.0;
    } else {
        return Feature::setParameter(name, value);
    }
    return true;
}

Feature::ParameterKind SheetMetalFeature::parameterKind(const std::string& name) const {
    if (name == "kFactor") return ParameterKind::Number;
    if (name == "bendSegments") return ParameterKind::Count;
    if (name == "flip") return ParameterKind::Choice;
    return Feature::parameterKind(name);
}

std::vector<std::string> SheetMetalFeature::parameterChoices(const std::string& name) const {
    if (name == "flip") return {"Left", "Right"};
    return Feature::parameterChoices(name);
}

std::vector<math::Vec2> SheetMetalFeature::profile(std::string* why) const {
    const auto refuse = [why](std::string message) {
        if (why != nullptr) *why = std::move(message);
        return std::vector<math::Vec2>{};
    };
    if (!m_sketch) return refuse("it has no sketch");
    std::vector<std::pair<math::Vec2, math::Vec2>> lines;
    for (const auto& entity : m_sketch->entities()) {
        if (!entity) continue;
        if (const auto* line = dynamic_cast<const draft::DraftLine*>(entity.get())) {
            lines.emplace_back(line->start(), line->end());
        } else if (const auto* run = dynamic_cast<const draft::DraftPolyline*>(entity.get());
                   run != nullptr) {
            // A polyline is its segments, a closed one closed.
            const auto& at = run->points();
            for (std::size_t i = 0; i + 1 < at.size(); ++i) lines.emplace_back(at[i], at[i + 1]);
            if (run->closed() && at.size() > 2) lines.emplace_back(at.back(), at.front());
        } else {
            return refuse(
                "its sketch has something other than lines: a base flange is sketched with "
                "lines only, and its bends come from the bend radius");
        }
    }
    if (lines.empty()) return refuse("its sketch has no lines");

    // An end of the chain is a point only one line reaches; the chain starts
    // at the first line's own start when that is an end, else at the end
    // nearest the first line in the sketch's order.
    const auto meets = [&lines](const math::Vec2& p) {
        int n = 0;
        for (const auto& [a, b] : lines) n += (same(a, p) ? 1 : 0) + (same(b, p) ? 1 : 0);
        return n;
    };
    std::optional<math::Vec2> start;
    for (const auto& [a, b] : lines) {
        if (meets(a) == 1) {
            start = a;
            break;
        }
        if (meets(b) == 1) {
            start = b;
            break;
        }
    }
    if (!start) {
        return refuse(
            "its sketch is closed: a base flange is sketched open, as an L, a U or a Z is");
    }
    std::vector<math::Vec2> points{*start};
    std::vector<bool> used(lines.size(), false);
    for (std::size_t step = 0; step < lines.size(); ++step) {
        const math::Vec2 at = points.back();
        std::size_t next = lines.size();
        bool reversed = false;
        for (std::size_t i = 0; i < lines.size(); ++i) {
            if (used[i]) continue;
            if (same(lines[i].first, at)) {
                next = i;
                break;
            }
            if (same(lines[i].second, at)) {
                next = i;
                reversed = true;
                break;
            }
        }
        if (next == lines.size()) break;
        if (meets(at) > 2) {
            return refuse("its sketch's lines branch: a base flange's lines run end to end");
        }
        used[next] = true;
        points.push_back(reversed ? lines[next].first : lines[next].second);
    }
    if (std::find(used.begin(), used.end(), false) != used.end()) {
        return refuse(
            "its sketch's lines do not all join end to end: a base flange is one run of lines");
    }
    return points;
}

std::optional<model::SheetMetalStrip> SheetMetalFeature::strip(std::string* why) const {
    const std::vector<math::Vec2> points = profile(why);
    if (points.empty()) return std::nullopt;
    return model::stripFromPolyline(points, m_params, m_flip, why);
}

std::unique_ptr<topo::Solid> SheetMetalFeature::execute(std::unique_ptr<topo::Solid> inputSolid,
                                                        std::string* reason) const {
    return executeWithExact(BuildContext{}, std::move(inputSolid), reason).faceted;
}

model::FacetResolution SheetMetalFeature::facetResolution() const {
    model::FacetResolution resolution;
    resolution.segmentsPerTurn = 4 * m_segments;
    resolution.chordTolerance = 0.0;
    return resolution;
}

FeatureOutput SheetMetalFeature::executeWithExact(const BuildContext& /*context*/,
                                                  std::unique_ptr<topo::Solid> /*inputSolid*/,
                                                  std::string* reason) const {
    // The flange alone; the tree combines it with the part as its operation
    // says, as for every feature that makes a body.
    const auto fail = [reason](std::string why) {
        if (reason != nullptr) *reason = std::move(why);
        return FeatureOutput{};
    };
    std::string why;
    const std::vector<math::Vec2> points = profile(&why);
    if (points.empty()) return fail(why);
    const auto folded = model::stripFromPolyline(points, m_params, m_flip, &why);
    if (!folded) return fail(why);
    if (!(m_width > 0.0)) return fail("the width must be more than 0");
    const std::vector<math::Vec2> section =
        model::SheetMetalSolid::crossSection(*folded, m_params, m_segments);
    if (section.size() < 3) return fail("the sheet could not be folded");

    // The strip runs from the origin along +x, its material toward +y: laid
    // on the sketch from the first point along the first line, its material
    // to the left of the lines, or the right.
    const math::Vec2 origin = points[0];
    const double length = std::hypot(points[1].x - origin.x, points[1].y - origin.y);
    const math::Vec2 along{(points[1].x - origin.x) / length, (points[1].y - origin.y) / length};
    const math::Vec2 side = m_flip ? math::Vec2(along.y, -along.x) : math::Vec2(-along.y, along.x);
    const Laying lay{origin, along, side};
    std::vector<math::Vec2> laid;
    laid.reserve(section.size());
    for (const math::Vec2& q : section) laid.push_back(lay(q));
    // Mirrored onto the right, the outline runs the other way round.
    if (m_flip) std::reverse(laid.begin(), laid.end());
    // Master's lines, one to each edge of the polygon, made in its order: under Stable naming the
    // flats and ends are these very lines (exactOutline).
    std::vector<std::shared_ptr<draft::DraftEntity>> outline;
    outline.reserve(laid.size());
    for (std::size_t i = 0; i < laid.size(); ++i) {
        outline.push_back(std::make_shared<draft::DraftLine>(laid[i], laid[(i + 1) % laid.size()]));
    }
    const draft::SketchPlane& plane = m_sketch->plane();
    FeatureOutput out;
    if (naming() != model::NamingScheme::Stable) {
        // Named by geometry or by position: master's faceted flange.
        out.faceted =
            model::Extrude::execute(outline, plane, plane.normal(), m_width, m_featureID,
                                    model::Extrude::kDefaultSegments, 0.0, &why, naming());
        if (!out.faceted) return fail(why.empty() ? "the sheet could not be made" : why);
        return out;
    }
    // Built exactly (stage 3): each bend two arcs, so two cylinders, and the faceted flange every
    // operation not yet converted reads is its bridge copy, cut at facetResolution(). Its flats'
    // faces and edges are master's by their lines; a bend's are new, numbered as the bridge
    // numbers them (master built no faces of these arcs to number them after).
    const std::vector<std::shared_ptr<draft::DraftEntity>> bent = exactOutline(
        model::SheetMetalSolid::profile(*folded, m_params), outline, m_segments, m_flip, lay, why);
    if (bent.empty()) return fail("could not be built exactly: " + why);
    // Master's refusal in its words, or "could not be built exactly: <why>".
    auto exact = model::Extrude::exact(bent, plane, plane.normal(), m_width, m_featureID, &why);
    if (!exact) return fail(why.empty() ? "the sheet could not be made" : why);
    auto built = model::buildWithBridge([&exact](std::string& /*why*/) { return std::move(exact); },
                                        facetResolution(), {}, &why);
    if (!built.faceted || !built.exact) {
        return fail(why.empty() ? "the sheet could not be made" : why);
    }
    out.faceted = std::move(built.faceted);
    out.exact = std::move(built.exact);
    return out;
}

}  // namespace hz::doc
