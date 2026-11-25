#include "ExactCounterpart.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iomanip>
#include <sstream>
#include <unordered_map>
#include <utility>

#include "horizon/geometry/Frame.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec2.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/HalfEdge.h"

namespace hz::doc {

namespace {

/// An FNV-1a hash of 64-bit words, and of the things a body is defined by, bit for bit.
class Hash {
public:
    std::uint64_t value() const { return m_hash; }

    Hash& word(std::uint64_t w) {
        for (int i = 0; i < 8; ++i) {
            m_hash ^= (w >> (8 * i)) & 0xFFU;
            m_hash *= 1099511628211ULL;
        }
        return *this;
    }
    Hash& real(double value) {
        if (value == 0.0) value = 0.0;  // -0 as 0: a read may give either for a direction's 0
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof bits);
        return word(bits);
    }
    Hash& point(const math::Vec3& p) { return real(p.x).real(p.y).real(p.z); }
    Hash& frame(const geo::Frame& f) { return point(f.origin).point(f.x).point(f.y).point(f.z); }
    Hash& reals(const std::vector<double>& values) {
        word(values.size());
        for (const double v : values) real(v);
        return *this;
    }

    /// A surface by its definition: its kind, frame and sizes, or a B-spline's degrees, control
    /// points, weights and knots.
    Hash& surface(const geo::Surface& s) {
        word(static_cast<std::uint64_t>(s.kind()));
        if (const auto* plane = dynamic_cast<const geo::Plane*>(&s)) {
            return frame(plane->frame()).real(plane->scale());
        }
        if (const auto* cylinder = dynamic_cast<const geo::Cylinder*>(&s)) {
            return frame(cylinder->frame()).real(cylinder->radius()).real(cylinder->axialScale());
        }
        if (const auto* cone = dynamic_cast<const geo::Cone*>(&s)) {
            return frame(cone->frame())
                .real(cone->radius())
                .real(cone->halfAngle())
                .real(cone->axialScale());
        }
        if (const auto* sphere = dynamic_cast<const geo::Sphere*>(&s)) {
            return frame(sphere->frame()).real(sphere->radius());
        }
        if (const auto* torus = dynamic_cast<const geo::Torus*>(&s)) {
            return frame(torus->frame()).real(torus->majorRadius()).real(torus->minorRadius());
        }
        if (const auto* spline = dynamic_cast<const geo::NurbsSurface*>(&s)) {
            word(static_cast<std::uint64_t>(spline->degreeU()));
            word(static_cast<std::uint64_t>(spline->degreeV()));
            for (const auto& row : spline->controlPoints()) {
                word(row.size());
                for (const math::Vec3& p : row) point(p);
            }
            for (const auto& row : spline->weights()) reals(row);
            return reals(spline->knotsU()).reals(spline->knotsV());
        }
        // A kind with no definition read here: its map, at points within its ranges.
        for (const double fu : {0.13, 0.5, 0.87}) {
            for (const double fv : {0.21, 0.5, 0.79}) {
                point(s.evaluate(within(s.uRange(), fu), within(s.vRange(), fv)));
            }
        }
        return *this;
    }

    /// A curve by its definition: its kind, its line, frame and radii, or a B-spline's degree,
    /// control points, weights and knots.
    Hash& curve(const geo::Curve3& c) {
        word(static_cast<std::uint64_t>(c.kind()));
        if (const auto* line = dynamic_cast<const geo::LineCurve*>(&c)) {
            return point(line->origin()).point(line->direction()).real(line->scale());
        }
        if (const auto* circle = dynamic_cast<const geo::CircleCurve*>(&c)) {
            return frame(circle->frame()).real(circle->radius());
        }
        if (const auto* ellipse = dynamic_cast<const geo::EllipseCurve*>(&c)) {
            return frame(ellipse->frame())
                .real(ellipse->majorRadius())
                .real(ellipse->minorRadius());
        }
        if (const auto* spline = dynamic_cast<const geo::NurbsCurve*>(&c)) {
            word(static_cast<std::uint64_t>(spline->degree()));
            word(spline->controlPoints().size());
            for (const math::Vec3& p : spline->controlPoints()) point(p);
            return reals(spline->weights()).reals(spline->knots());
        }
        for (const double f : {0.13, 0.5, 0.87}) point(c.evaluate(within(c.range(), f)));
        return *this;
    }

private:
    /// A parameter @p f of the way along @p range, or about @p f where it is unbounded.
    static double within(const geo::Interval& range, double f) {
        if (!std::isfinite(range.min) || !std::isfinite(range.max)) return f;
        return range.min + f * (range.max - range.min);
    }

    std::uint64_t m_hash = 14695981039346656037ULL;
};

/// A body's fingerprint: its counts, and each face in order (its names follow that order) by its
/// sense, its holes and its exact surface's definition, with every edge and vertex of its loops:
/// each edge's exact curve's definition, its range and its tolerance, and each vertex's point and
/// tolerance, bit for bit (0 and -0 alike). Two bodies that differ in any of these differ in it,
/// as near as 64 bits can tell, and their counterparts are never taken one for the other.
///
/// A body read from the STEP text a part keeps of it (a worker's copy of the document) is read
/// the same every time, and gives the same: its faces in the same order, each definition written
/// to 17 digits and read back. The read stores a face's edges and vertices in another order and
/// walks its loops from other half-edges, so they count as a set, each edge's range whichever way
/// it runs. The first such read may define a body a little otherwise than the import it was
/// written from (a frame renormalised in its last bit, a surface's scale folded into its
/// parameters, a range moved with it): a worker's first build after an import makes the
/// counterpart again, and each later one reuses that.
std::uint64_t fingerprint(const topo::Solid& body) {
    Hash hash;
    hash.word(body.vertices().size()).word(body.edges().size());
    hash.word(body.faces().size()).word(body.shells().size());
    const std::size_t limit = 2 * body.edges().size() + 1;
    for (const auto& f : body.faces()) {
        hash.word(f.alive ? 1U : 0U).word(f.reversed ? 1U : 0U).word(f.innerLoops.size());
        if (f.exactSurface) {
            hash.surface(*f.exactSurface);
        } else {
            hash.word(0U);
        }
        // Its loops' edges and vertices, in whichever order they are walked: a sum of their own.
        std::uint64_t around = 0;
        std::vector<const topo::Wire*> loops{f.outerLoop};
        loops.insert(loops.end(), f.innerLoops.begin(), f.innerLoops.end());
        for (const topo::Wire* loop : loops) {
            if (loop == nullptr || loop->halfEdge == nullptr) continue;
            const topo::HalfEdge* he = loop->halfEdge;
            std::size_t guard = 0;
            do {
                Hash piece;
                if (he->origin != nullptr) {
                    piece.point(he->origin->point).real(he->origin->tolerance);
                }
                if (const topo::Edge* e = he->edge; e != nullptr) {
                    if (e->exactCurve) piece.curve(*e->exactCurve);
                    piece.real(std::min(e->curveStart, e->curveEnd))
                        .real(std::max(e->curveStart, e->curveEnd))
                        .real(e->tolerance);
                }
                around += piece.value();
                he = he->next;
            } while (he != nullptr && he != loop->halfEdge && ++guard <= limit);
        }
        hash.word(around);
    }
    return hash.value();
}

/// What a counterpart of @p imports is made from (ExactCounterpart::madeFrom).
std::string madeFrom(const std::vector<const ImportedBodyFeature*>& imports) {
    std::ostringstream out;
    for (const ImportedBodyFeature* imported : imports) {
        out << imported->featureID() << ' ' << static_cast<int>(imported->naming()) << ' '
            << std::hex << (imported->solid() ? fingerprint(*imported->solid()) : 0U) << std::dec
            << '\n';
    }
    return out.str();
}

/// A volume as a note says it.
std::string figure(double value) {
    std::ostringstream out;
    out << std::setprecision(10) << value;
    return out.str();
}

/// What holding a counterpart to its facets found.
struct Check {
    bool cancelled = false;
    std::string note;  ///< empty when they agree
};

/// A face's outer loop's corners, at most one for each of @p limit's edges.
std::vector<math::Vec3> cornersOf(const topo::Face& face, std::size_t limit) {
    std::vector<math::Vec3> corners;
    if (face.outerLoop == nullptr || face.outerLoop->halfEdge == nullptr) return corners;
    const topo::HalfEdge* start = face.outerLoop->halfEdge;
    const topo::HalfEdge* he = start;
    do {
        if (he->origin == nullptr) break;
        corners.push_back(he->origin->point);
        he = he->next;
    } while (he != nullptr && he != start && corners.size() <= limit);
    return corners;
}

/// Whether @p faceted, the part's faceted copy, measures as @p exact, its exact counterpart,
/// does: their volumes within what the faceting itself may move a volume by.
///
/// The bound. Each face of the faceted copy is a flat polygon whose corners are the import's
/// vertices, and points along its edges, and lie on its face's surface: a curved face's facets
/// (model::facetCurved), a flat face whole, or in facets where its corners leave its plane. The
/// material between such a polygon and the surface over it is at most its area A times how far
/// the surface stands off it, d: the slab of thickness d over it (between a chord and its arc it
/// is two thirds of that). A polygon inscribed in a curved surface stands off it most at its
/// middle, where the surface has curved furthest from its corners: at its centre for one about
/// as long as it is wide, at the middle of its longest side for a sliver. A flat face's polygon
/// stands off its plane at its corners, by the gap its reader accepted between a vertex and the
/// planes it is on. d is measured at all three, as the distance to the nearest point of the
/// counterpart's face of the same logical name, and doubled for the curving between: 2 A d.
/// Each face adds as well its area times the tolerance of its exact face (the largest of the
/// model's and of its edges' and vertices' own), for what a gap does to the exact measure. To
/// the sum is added the exact integration's own error, 1e-9 of its volume (spec goal 6). On the
/// corpus the facets' volumes stand at most a third of this bound from the exact ones (a chord's
/// sagitta closes two thirds of A d). What it catches is a measure, not a shape: the distances
/// it is measured by already hold any surface standing off its facets, so it is a sign that is
/// wrong (a body turned inside out, which moves the volume by twice its own) or an integration,
/// far beyond the bound.
///
/// The bound is summed until it covers the difference: the tolerances first, then the facets on
/// closed-form surfaces (their nearest points are cheap), then those on B-splines (a search), so
/// a counterpart that agrees seldom measures every face. A face of the faceted copy whose logical
/// name no exact face has cannot be measured: that is said, never bounded by guess.
Check holdToFacets(const topo::Solid& exact, double exactVolume, const topo::Solid& faceted,
                   double facetedVolume, const BuildControl* control) {
    Check check;
    const double difference = std::abs(exactVolume - facetedVolume);
    double bound = 1e-9 * std::abs(exactVolume);
    struct Exact {
        const geo::Surface* surface = nullptr;
        double tolerance = 0.0;
    };
    const double model = topo::exactTolerance(exact).linear;
    std::unordered_map<std::string, Exact> faces;
    for (const auto& face : exact.faces()) {
        if (!face.alive || !face.exactSurface) continue;
        Exact found{face.exactSurface.get(), model};
        std::vector<const topo::Wire*> loops{face.outerLoop};
        loops.insert(loops.end(), face.innerLoops.begin(), face.innerLoops.end());
        for (const topo::Wire* loop : loops) {
            if (loop == nullptr || loop->halfEdge == nullptr) continue;
            const topo::HalfEdge* he = loop->halfEdge;
            std::size_t guard = 0;
            do {
                if (he->edge != nullptr)
                    found.tolerance = std::max(found.tolerance, he->edge->tolerance);
                if (he->origin != nullptr) {
                    found.tolerance = std::max(found.tolerance, he->origin->tolerance);
                }
                he = he->next;
            } while (he != nullptr && he != loop->halfEdge && ++guard <= exact.edges().size() * 2);
        }
        faces.emplace(face.topoId.tag(), found);
    }
    const std::size_t limit = faceted.edges().size();
    const auto areaOf = [](const std::vector<math::Vec3>& corners) {
        math::Vec3 normal;
        for (std::size_t i = 1; i + 1 < corners.size(); ++i) {
            normal = normal + (corners[i] - corners[0]).cross(corners[i + 1] - corners[0]);
        }
        return 0.5 * normal.length();
    };
    // The tolerances, and every face's name.
    for (const auto& face : faceted.faces()) {
        if (!face.alive) continue;
        const auto found = faces.find(model::logicalFace(face.topoId.tag()));
        if (found == faces.end()) {
            check.note =
                "exact form not checked: its facet " + face.topoId.tag() + " names no face of it";
            return check;
        }
        const auto corners = cornersOf(face, limit);
        if (corners.size() >= 3) bound += areaOf(corners) * found->second.tolerance;
    }
    if (difference <= bound) return check;
    // How far each face's surface stands off it, measured.
    std::size_t measured = 0;
    for (const bool spline : {false, true}) {
        for (const auto& face : faceted.faces()) {
            if (!face.alive) continue;
            const geo::Surface* surface = faces.at(model::logicalFace(face.topoId.tag())).surface;
            if ((surface->kind() == geo::SurfaceKind::Nurbs) != spline) continue;
            if (++measured % 64 == 0 && control != nullptr && control->cancel) {
                check.cancelled = true;
                return check;
            }
            const auto corners = cornersOf(face, limit);
            if (corners.size() < 3) continue;
            math::Vec3 centre;
            math::Vec3 longestMiddle = corners[0];
            double longest = -1.0;
            for (std::size_t i = 0; i < corners.size(); ++i) {
                const math::Vec3& a = corners[i];
                const math::Vec3& b = corners[(i + 1) % corners.size()];
                centre = centre + a;
                if ((b - a).length() > longest) {
                    longest = (b - a).length();
                    longestMiddle = (a + b) * 0.5;
                }
            }
            centre = centre / static_cast<double>(corners.size());
            const auto off = [surface](const math::Vec3& at) {
                const math::Vec2 uv = surface->invert(at);
                return (surface->evaluate(uv.x, uv.y) - at).length();
            };
            double d = std::max(off(centre), off(longestMiddle));
            for (const math::Vec3& corner : corners) d = std::max(d, off(corner));
            if (!std::isfinite(d)) {
                check.note = "exact form not checked: its facet " + face.topoId.tag() +
                             " could not be measured against it";
                return check;
            }
            bound += 2.0 * areaOf(corners) * d;
            if (bound >= difference) return check;
        }
    }
    check.note = "exact form disagrees with its facets: " + figure(exactVolume) + " vs " +
                 figure(facetedVolume);
    return check;
}

}  // namespace

void ExactFollower::follow(const Feature& feature, bool first) noexcept {
    try {
        const auto* imported = dynamic_cast<const ImportedBodyFeature*>(&feature);
        if (imported == nullptr || (!first && feature.operation() != BodyOperation::NewBody)) {
            // The part is no longer only bodies as read: none, by design.
            m_open = false;
            m_imports.clear();
            m_note.clear();
            return;
        }
        if (m_open) m_imports.push_back(imported);
    } catch (...) {
        m_open = false;
        m_imports.clear();
        m_note = "shown from facets: its exact form could not be gathered";
    }
}

std::shared_ptr<const ExactCounterpart> ExactFollower::make(
    const topo::Solid& faceted, const std::shared_ptr<const ExactCounterpart>& previous,
    BuildControl* control, bool& cancelled) const noexcept {
    cancelled = false;
    try {
        if (m_imports.empty() && m_note.empty()) return nullptr;
        auto made = std::make_shared<ExactCounterpart>();
        made->madeFrom = madeFrom(m_imports);
        if (!m_note.empty()) {
            made->note = m_note;
            return made;
        }
        if (previous && !previous->madeFrom.empty() && previous->madeFrom == made->madeFrom) {
            return previous;
        }

        // Gathered, meshed, measured and held to its facets: a step of its own, done however it
        // ends.
        struct Step {
            BuildControl* control;
            explicit Step(BuildControl* counted) : control(counted) {}
            Step(const Step&) = delete;
            Step& operator=(const Step&) = delete;
            ~Step() {
                if (control != nullptr) control->done = control->total.load();
            }
        };
        if (control != nullptr) control->total.fetch_add(1);
        const Step step(control);
        const auto stopped = [control] { return control != nullptr && control->cancel; };
        if (stopped()) {
            cancelled = true;
            return nullptr;
        }
        // Gathered: each import's exact body, as the part gathers their facets.
        std::unique_ptr<topo::Solid> gathered;
        for (const ImportedBodyFeature* imported : m_imports) {
            auto body = imported->exactBody();
            if (!body) {
                made->note = "shown from facets: " +
                             (imported->note().empty()
                                  ? "imported body \"" + imported->source() + "\" has no exact form"
                                  : imported->note());
                return made;
            }
            if (!gathered) {
                gathered = std::move(body);
            } else {
                model::Pattern::append(*gathered, *body, math::Mat4::identity());
            }
        }
        if (stopped()) {
            cancelled = true;
            return nullptr;
        }
        std::string problem;
        auto mesh = model::SolidTessellator::tessellate(
            *gathered, model::SolidTessellator::kDisplayTolerance, &problem,
            control != nullptr ? &control->cancel : nullptr);
        if (stopped()) {
            cancelled = true;
            return nullptr;
        }
        if (!problem.empty()) {
            made->note = "shown from facets: " + problem;
            return made;
        }
        const auto shells = model::MassPropertiesCalculator::shellVolumes(
            *gathered, false, &problem, control != nullptr ? &control->cancel : nullptr);
        if (stopped()) {
            cancelled = true;
            return nullptr;
        }
        if (shells.empty()) {
            made->note =
                "shown from facets: " +
                (problem.empty() ? std::string("its exact form encloses nothing") : problem);
            return made;
        }
        double exactVolume = 0.0;
        for (const auto& shell : shells) exactVolume += shell.volume;
        exactVolume = std::abs(exactVolume);
        const auto facets = model::MassPropertiesCalculator::compute(faceted);
        if (!facets.valid) {
            made->note =
                "exact form not checked: its facets cannot be measured (" +
                (facets.problem.empty() ? std::string("they enclose nothing") : facets.problem) +
                ")";
            return made;
        }
        const Check check = holdToFacets(*gathered, exactVolume, faceted, facets.volume, control);
        if (check.cancelled) {
            cancelled = true;
            return nullptr;
        }
        if (!check.note.empty()) {
            made->note = check.note;
            return made;
        }
        // Both, or neither.
        auto shown = std::make_shared<const geo::MeshData>(std::move(mesh));
        made->solid = std::shared_ptr<const topo::Solid>(std::move(gathered));
        made->mesh = std::move(shown);
        return made;
    } catch (const std::exception& e) {
        // Said, but not kept for reuse (no madeFrom): what threw (memory, say) may not again.
        try {
            auto failed = std::make_shared<ExactCounterpart>();
            failed->note = std::string("shown from facets: ") + e.what();
            return failed;
        } catch (...) {
            return nullptr;
        }
    } catch (...) {
        return nullptr;
    }
}

}  // namespace hz::doc
