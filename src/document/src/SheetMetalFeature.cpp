// The base flange (sheet metal): an open sketch of lines folded into a sheet.
// The fold is model::SheetMetalSolid's cross-section, laid on the sketch and
// extruded along its normal, so the flange is named and placed as an
// extrusion of its sketch is. Under Stable naming it is built exactly (stage
// 3): the fold's exact profile, each bend two arcs, so each bend is two
// cylinders, its faces named after its sketch's segments (Ruling 28).

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <unordered_map>
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
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/SheetMetal.h"
#include "horizon/modeling/SheetMetalSolid.h"
#include "horizon/topology/TopologyID.h"

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

/// What each piece of model::SheetMetalSolid::profile(@p strip, ...) is named (Ruling 28), in
/// profile()'s order, after @p sources, the sketch's segment each of the strip's segments runs
/// along (runOf): a flat's face on the sketch's side `flat:<S>`, a thickness away
/// `flat:<S>:back`; a bend's from segment S to the next, S', `bend:<S>-<S'>` and
/// `bend:<S>-<S'>:back`; the near end `start:<S>` of the first, the far end `end:<S>` of the
/// last. Empty where there is not a source to each segment.
std::vector<std::string> rolesOf(const model::SheetMetalStrip& strip,
                                 const std::vector<std::string>& sources) {
    const std::size_t n = strip.segments.size();
    if (n == 0 || sources.size() != n || strip.bendAngles.size() + 1 != n) return {};
    const auto bend = [&sources](std::size_t j) {
        return "bend:" + sources[j] + "-" + sources[j + 1];
    };
    std::vector<std::string> roles;
    // The sketch's side walked forward, the far end, the other side walked back, the near end.
    for (std::size_t i = 0; i < n; ++i) {
        roles.push_back("flat:" + sources[i]);
        if (i + 1 < n && strip.bendAngles[i] != 0.0) roles.push_back(bend(i));
    }
    roles.push_back("end:" + sources[n - 1]);
    for (std::size_t i = n; i-- > 0;) {
        roles.push_back("flat:" + sources[i] + ":back");
        if (i > 0 && strip.bendAngles[i - 1] != 0.0) roles.push_back(bend(i - 1) + ":back");
    }
    roles.push_back("start:" + sources[0]);
    return roles;
}

/// The flange's exact outline on the sketch, its pieces and what each is named.
struct ExactOutline {
    std::vector<std::shared_ptr<draft::DraftEntity>> pieces;
    std::vector<std::string> roles;
    /// The name of the face each of master's lines (the outline's, by index) extrudes to under
    /// Ruling 28: a flat's or an end's its piece's role, a bend's chord `<bend's role>/facet:<c>`,
    /// c counted along the bend from the flat before it in the strip's order, as the bridge
    /// numbers a bend's facets (master's own faceted flange, Ruling 44).
    std::vector<std::string> lineRoles;
};

/// The flange's exact outline on the sketch (stage 3), in master's order: @p bent, the strip's
/// exact profile (model::SheetMetalSolid::profile, in crossSection's order and frame), each flat
/// and end replaced by master's own line along it, from @p outline (master's cross-section at
/// @p perBend chords a bend, laid and made in master's order, which a flipped flange runs the
/// other way round), and each bend's arc laid by @p lay; each piece with its name from @p roles
/// (rolesOf). Master's lines and master's order keep the bridge's vertices and where its caps'
/// loops start master's. Empty, with why, where a flat of @p bent is not one of master's lines,
/// or the names are not one to each piece.
ExactOutline exactOutline(const std::vector<std::shared_ptr<draft::DraftEntity>>& bent,
                          const std::vector<std::string>& roles,
                          const std::vector<std::shared_ptr<draft::DraftEntity>>& outline,
                          int perBend, bool flipped, const Laying& lay, std::string& why) {
    const std::size_t n = outline.size();
    const auto refuse = [&why]() {
        why = "the sheet's exact profile does not follow its cross-section";
        return ExactOutline{};
    };
    if (roles.size() != bent.size()) return refuse();
    ExactOutline laid;
    laid.pieces.reserve(bent.size());
    laid.roles = roles;
    laid.lineRoles.assign(n, std::string());
    // Master's line along crossSection's edge @p e: flipped, master's polygon ran backwards from
    // its last corner but one, its closing line still last.
    const auto lineOf = [n, flipped](std::size_t e) {
        return !flipped || e + 1 == n ? e : n - 2 - e;
    };
    std::size_t edge = 0;  // the first of crossSection's edges the piece spans
    for (std::size_t j = 0; j < bent.size(); ++j) {
        const auto& piece = bent[j];
        const bool bend = roles[j].rfind("bend:", 0) == 0;
        if (auto* arc = dynamic_cast<draft::DraftArc*>(piece.get())) {
            if (!bend) return refuse();
            // Mirrored, an arc turns the other way round: its start is its end.
            const math::Vec2 centre = lay(arc->center());
            const math::Vec2 start = lay(flipped ? arc->endPoint() : arc->startPoint());
            const math::Vec2 end = lay(flipped ? arc->startPoint() : arc->endPoint());
            arc->setCenter(centre);
            arc->setStartAngle(
                math::normalizeAngle(std::atan2(start.y - centre.y, start.x - centre.x)));
            arc->setEndAngle(math::normalizeAngle(std::atan2(end.y - centre.y, end.x - centre.x)));
            laid.pieces.push_back(piece);
            // Counted from the flat before the bend in the strip's order, as the bridge numbers
            // a bend's facets: forward along the bottom, which crossSection walks forward, and
            // back along the top, which it walks back (after the far end, its middle piece).
            const auto chords = static_cast<std::size_t>(perBend);
            const bool top = j > (bent.size() - 2) / 2;
            for (std::size_t c = 0; c < chords; ++c) {
                if (edge + c >= n) return refuse();
                laid.lineRoles[lineOf(edge + c)] =
                    roles[j] + "/facet:" + std::to_string(top ? chords - 1 - c : c);
            }
            edge += static_cast<std::size_t>(perBend);
            continue;
        }
        const auto* line = dynamic_cast<const draft::DraftLine*>(piece.get());
        if (line == nullptr || bend || edge >= n) return refuse();
        // Master's line along that edge.
        const std::size_t k = lineOf(edge);
        const auto* masters = dynamic_cast<const draft::DraftLine*>(outline[k].get());
        const math::Vec2 a = lay(line->start());
        const math::Vec2 b = lay(line->end());
        if (masters == nullptr || !((same(masters->start(), a) && same(masters->end(), b)) ||
                                    (same(masters->start(), b) && same(masters->end(), a)))) {
            return refuse();
        }
        laid.pieces.push_back(outline[k]);
        laid.lineRoles[k] = roles[j];
        ++edge;
    }
    if (edge != n || laid.pieces.empty()) return refuse();
    // Master's polygon, flipped, ran from the top's first flat back to the bottom's, then its
    // near end.
    if (flipped) {
        std::reverse(laid.pieces.begin(), laid.pieces.end() - 1);
        std::reverse(laid.roles.begin(), laid.roles.end() - 1);
    }
    return laid;
}

/// @p sketch's lines in order, end to end (SheetMetalFeature::profile), and, in @p sources where
/// given, the segment of the sketch each runs along, in the same order: "e<id>" for a line,
/// "e<id>.<k>" for a polyline's segment k (as ProfileValidator names them). Empty, with why, when
/// they are not one open run.
std::vector<math::Vec2> runOf(const Sketch* sketch, std::vector<std::string>* sources,
                              std::string* why) {
    const auto refuse = [why](std::string message) {
        if (why != nullptr) *why = std::move(message);
        return std::vector<math::Vec2>{};
    };
    if (sketch == nullptr) return refuse("it has no sketch");
    std::vector<std::pair<math::Vec2, math::Vec2>> lines;
    std::vector<std::string> named;
    for (const auto& entity : sketch->entities()) {
        if (!entity) continue;
        const std::string source = "e" + std::to_string(entity->id());
        if (const auto* line = dynamic_cast<const draft::DraftLine*>(entity.get())) {
            lines.emplace_back(line->start(), line->end());
            named.push_back(source);
        } else if (const auto* run = dynamic_cast<const draft::DraftPolyline*>(entity.get());
                   run != nullptr) {
            // A polyline is its segments, a closed one closed.
            const auto& at = run->points();
            for (std::size_t i = 0; i + 1 < at.size(); ++i) {
                lines.emplace_back(at[i], at[i + 1]);
                named.push_back(source + "." + std::to_string(i));
            }
            if (run->closed() && at.size() > 2) {
                lines.emplace_back(at.back(), at.front());
                named.push_back(source + "." + std::to_string(at.size() - 1));
            }
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
    std::vector<std::string> along;
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
        along.push_back(named[next]);
    }
    if (std::find(used.begin(), used.end(), false) != used.end()) {
        return refuse(
            "its sketch's lines do not all join end to end: a base flange is one run of lines");
    }
    if (sources != nullptr) *sources = std::move(along);
    return points;
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
    return runOf(m_sketch.get(), nullptr, why);
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
    // Its lines end to end, and the segment of the sketch each runs along.
    std::vector<std::string> sources;
    const std::vector<math::Vec2> points = runOf(m_sketch.get(), &sources, &why);
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
    // operation not yet converted reads is its bridge copy, cut at facetResolution(). Its faces
    // are named after the sketch (Ruling 28): the same in every build and every session.
    const ExactOutline bent =
        exactOutline(model::SheetMetalSolid::profile(*folded, m_params), rolesOf(*folded, sources),
                     outline, m_segments, m_flip, lay, why);
    if (bent.pieces.empty()) return fail("could not be built exactly: " + why);
    // Master's refusal in its words, or "could not be built exactly: <why>".
    auto exact =
        model::Extrude::exact(bent.pieces, plane, plane.normal(), m_width, m_featureID, &why);
    if (!exact) return fail(why.empty() ? "the sheet could not be made" : why);
    // Each side face the extrusion named after its piece's entity, `side:e<id>`, is named for
    // what the piece is; the caps keep their names; the edges are named after the faces again.
    std::unordered_map<std::string, std::string> roleOf;
    for (std::size_t k = 0; k < bent.pieces.size(); ++k) {
        roleOf.emplace("e" + std::to_string(bent.pieces[k]->id()), bent.roles[k]);
    }
    const std::string sideOf = m_featureID + "/side:";
    for (auto& face : exact->faces()) {
        const std::string tag = face.topoId.tag();
        if (tag.compare(0, sideOf.size(), sideOf) != 0) continue;
        const auto found = roleOf.find(tag.substr(sideOf.size()));
        if (found == roleOf.end()) {
            return fail("could not be built exactly: its face " + tag +
                        " is no piece of its sheet");
        }
        face.topoId = topo::TopologyID::make(m_featureID, found->second);
    }
    model::nameEdgesLogically(*exact);
    // The faceted flange is master's own: its cross-section's polygon extruded as master's Stable
    // path built it, every bend in master's chords, its faces named after the sketch as the exact
    // flange's are (Ruling 28), then as the bridge names the same places (Ruling 44).
    const auto own = [&](std::string& ownWhy) -> std::unique_ptr<topo::Solid> {
        auto made =
            model::Extrude::mastersSolid(outline, plane, plane.normal(), m_width, m_featureID,
                                         model::Extrude::kDefaultSegments, 0.0, &ownWhy);
        if (!made) return made;
        std::unordered_map<std::string, std::string> lineRole;
        for (std::size_t k = 0; k < outline.size(); ++k) {
            lineRole.emplace("e" + std::to_string(outline[k]->id()), bent.lineRoles[k]);
        }
        for (auto& face : made->faces()) {
            const std::string tag = face.topoId.tag();
            if (tag.compare(0, sideOf.size(), sideOf) != 0) continue;
            const auto found = lineRole.find(tag.substr(sideOf.size()));
            if (found == lineRole.end() || found->second.empty()) {
                ownWhy = "its face " + tag + " is no line of its sheet's cross-section";
                return nullptr;
            }
            face.topoId = topo::TopologyID::make(m_featureID, found->second);
        }
        model::nameEdgesLogically(*made);
        return made;
    };
    auto built = model::buildWithBridge([&exact](std::string& /*why*/) { return std::move(exact); },
                                        facetResolution(), {}, &why, own);
    if (!built.faceted || !built.exact) {
        return fail(why.empty() ? "the sheet could not be made" : why);
    }
    out.faceted = std::move(built.faceted);
    out.exact = std::move(built.exact);
    return out;
}

}  // namespace hz::doc
