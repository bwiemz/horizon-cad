#include "ExactCounterpart.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iomanip>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

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
    Hash& text(const std::string& s) {
        word(s.size());
        for (const char c : s) {
            m_hash ^= static_cast<unsigned char>(c);
            m_hash *= 1099511628211ULL;
        }
        return *this;
    }
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

/// What a body of the exact part is made from: its fingerprint, and its names: each face's in
/// order (fingerprint's order), and its edges' as a set (a read stores them in another order). Two
/// features alike but for their names (their IDs, or how they name edges) differ in it.
std::uint64_t definitionOf(const topo::Solid& body) {
    Hash hash;
    hash.word(fingerprint(body));
    for (const auto& f : body.faces()) hash.text(f.topoId.tag());
    std::uint64_t edges = 0;
    for (const auto& e : body.edges()) edges += Hash().text(e.topoId.tag()).value();
    return hash.word(edges).value();
}

/// What a counterpart of the exact part whose bodies are defined by @p definitions
/// (definitionOf, in order) is made from (ExactCounterpart::madeFrom).
std::string madeFrom(const std::vector<std::uint64_t>& definitions) {
    std::ostringstream out;
    out << std::hex;
    for (std::size_t k = 0; k < definitions.size(); ++k) {
        if (k > 0) out << ' ';
        out << definitions[k];
    }
    return out.str();
}

/// A body defined by @p definition (definitionOf) meshed at @p tolerance: its
/// BodyMesh::fingerprint.
std::uint64_t meshedFrom(std::uint64_t definition, double tolerance) {
    return Hash().word(definition).real(tolerance).value();
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

/// Whether @p facets, a body's faces of the part's faceted copy (@p limit its edges, at most, round
/// each), measure as @p exact, the body's exact form, does: their volumes within what the faceting
/// itself may move a volume by.
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
Check holdToFacets(const topo::Solid& exact, double exactVolume,
                   const std::vector<const topo::Face*>& facets, std::size_t limit,
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
    const auto areaOf = [](const std::vector<math::Vec3>& corners) {
        math::Vec3 normal;
        for (std::size_t i = 1; i + 1 < corners.size(); ++i) {
            normal = normal + (corners[i] - corners[0]).cross(corners[i + 1] - corners[0]);
        }
        return 0.5 * normal.length();
    };
    // The tolerances, and every face's name.
    for (const topo::Face* face : facets) {
        if (!face->alive) continue;
        const auto found = faces.find(model::logicalFace(face->topoId.tag()));
        if (found == faces.end()) {
            check.note =
                "exact form not checked: its facet " + face->topoId.tag() + " names no face of it";
            return check;
        }
        const auto corners = cornersOf(*face, limit);
        if (corners.size() >= 3) bound += areaOf(corners) * found->second.tolerance;
    }
    if (difference <= bound) return check;
    // How far each face's surface stands off it, measured.
    std::size_t measured = 0;
    for (const bool spline : {false, true}) {
        for (const topo::Face* face : facets) {
            if (!face->alive) continue;
            const geo::Surface* surface = faces.at(model::logicalFace(face->topoId.tag())).surface;
            if ((surface->kind() == geo::SurfaceKind::Nurbs) != spline) continue;
            if (++measured % 64 == 0 && control != nullptr && control->cancel) {
                check.cancelled = true;
                return check;
            }
            const auto corners = cornersOf(*face, limit);
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
                check.note = "exact form not checked: its facet " + face->topoId.tag() +
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

/// Each body's faces of the part's faceted copy, and the shells they make, found by their names.
struct Facets {
    std::vector<std::vector<const topo::Face*>> faces;  ///< by body
    std::vector<std::vector<bool>> shells;  ///< by body: which shells of the faceted copy are its
    std::string note;  ///< a facet that names no body's face, as holdToFacets would say it
};

/// The faces and shells of @p faceted, the part's faceted copy, that each of @p bodies, its exact
/// bodies, has, for those @p wanted marks. Each faceted face is named after its exact face
/// (model::logicalFace of its name, as holdToFacets reads it), and each exact face after its
/// feature (`<feature>/...`), so that its name is one of one body's faces'. A body is a shell of
/// the faceted copy, or more (a cavity's), and never part of one, as no Boolean has joined two:
/// each shell is the body its first face names, with all its faces, and holdToFacets says any of
/// them that names a face of no other of that body's. A shell whose first face names no body's
/// face is said in `note`.
Facets facetsOf(const topo::Solid& faceted, const std::vector<std::unique_ptr<topo::Solid>>& bodies,
                const std::vector<bool>& wanted) {
    Facets out;
    out.faces.resize(bodies.size());
    out.shells.assign(bodies.size(), std::vector<bool>(faceted.shells().size(), false));
    std::unordered_map<std::string, std::size_t> bodyOf;
    for (std::size_t k = 0; k < bodies.size(); ++k) {
        for (const auto& face : bodies[k]->faces()) {
            if (face.alive) bodyOf.emplace(face.topoId.tag(), k);
        }
    }
    std::size_t index = 0;
    for (const auto& shell : faceted.shells()) {
        const std::size_t at = index++;
        const auto first =
            std::find_if(shell.faces.begin(), shell.faces.end(),
                         [](const topo::Face* f) { return f != nullptr && f->alive; });
        if (first == shell.faces.end()) continue;
        const std::string& name = (*first)->topoId.tag();
        const auto found = bodyOf.find(model::logicalFace(name));
        if (found == bodyOf.end()) {
            if (out.note.empty()) {
                out.note = "exact form not checked: its facet " + name + " names no face of it";
            }
            continue;
        }
        const std::size_t body = found->second;
        if (!wanted[body]) continue;
        out.shells[body][at] = true;
        for (const topo::Face* face : shell.faces) {
            if (face != nullptr && face->alive) out.faces[body].push_back(face);
        }
    }
    return out;
}

/// A body meshed and held to its facets (meshBody).
struct Meshed {
    bool cancelled = false;
    std::shared_ptr<const geo::MeshData> mesh;  ///< null with a note
    std::string note;
};

/// @p body, one of the part's exact bodies, meshed on its own at @p tolerance (the part's),
/// measured, and held to its facets alone: @p facets, its faces of @p faceted, which make the
/// shells @p shells marks (facetsOf). Its note says why it cannot be shown exactly, as the part's
/// would for it alone. Stops between faces once @p control is cancelled.
Meshed meshBody(const topo::Solid& body, double tolerance, const topo::Solid& faceted,
                const std::vector<const topo::Face*>& facets, const std::vector<bool>& shells,
                const BuildControl* control) {
    Meshed meshed;
    const std::atomic<bool>* cancel = control != nullptr ? &control->cancel : nullptr;
    const auto stopped = [control] { return control != nullptr && control->cancel; };
    std::string problem;
    auto mesh = model::SolidTessellator::tessellate(body, tolerance, &problem, cancel);
    if (stopped()) {
        meshed.cancelled = true;
        return meshed;
    }
    if (!problem.empty()) {
        meshed.note = "shown from facets: " + problem;
        return meshed;
    }
    const auto volumes =
        model::MassPropertiesCalculator::shellVolumes(body, false, &problem, cancel);
    if (stopped()) {
        meshed.cancelled = true;
        return meshed;
    }
    if (volumes.empty()) {
        meshed.note = "shown from facets: " +
                      (problem.empty() ? std::string("its exact form encloses nothing") : problem);
        return meshed;
    }
    // Signed as its faces face: a body faces out as its own faces say (Pattern::append copies an
    // exact body as it is), and encloses a positive volume. A negative one, beyond its error, is a
    // body whose exact form faces into what it bounds (a body written inside out that its reader
    // did not turn): its facets, gathered by their corners, face out and measure the same volume
    // positive, so only its sign tells.
    double exactVolume = 0.0;
    double exactError = 0.0;
    for (const auto& shell : volumes) {
        exactVolume += shell.volume;
        exactError += shell.error;
    }
    if (exactVolume < 0.0 && -exactVolume > exactError) {
        meshed.note = "shown from facets: the exact form faces inward (it encloses " +
                      figure(exactVolume) + ")";
        return meshed;
    }
    exactVolume = std::abs(exactVolume);
    if (facets.empty()) {
        // Nothing to hold it to: no face of the faceted copy is named after one of its own.
        meshed.note = "exact form not checked: no facet of the part names a face of it";
        return meshed;
    }
    // Its facets' volume: the shells its faces make, by their polygons, their sign normalised
    // away as MassPropertiesCalculator::compute normalises a solid's.
    const auto facetShells =
        model::MassPropertiesCalculator::shellVolumes(faceted, false, &problem, cancel, shells);
    if (stopped()) {
        meshed.cancelled = true;
        return meshed;
    }
    double facetedVolume = 0.0;
    for (std::size_t i = 0; i < facetShells.size() && i < shells.size(); ++i) {
        if (shells[i]) facetedVolume += facetShells[i].volume;
    }
    facetedVolume = std::abs(facetedVolume);
    if (facetShells.empty() || !(facetedVolume > 0.0) || !std::isfinite(facetedVolume)) {
        meshed.note = "exact form not checked: its facets cannot be measured (" +
                      (problem.empty() ? std::string("they enclose nothing") : problem) + ")";
        return meshed;
    }
    const Check check =
        holdToFacets(body, exactVolume, facets, faceted.edges().size(), facetedVolume, control);
    if (check.cancelled) {
        meshed.cancelled = true;
        return meshed;
    }
    if (!check.note.empty()) {
        meshed.note = check.note;
        return meshed;
    }
    meshed.mesh = std::make_shared<const geo::MeshData>(std::move(mesh));
    return meshed;
}

/// The meshes of @p bodies, each with one, as one: each one's vertices, triangles (their corners
/// and faces counted on from those of the meshes before it), faces' names and edges after the
/// last's. The names are as they are: each body's are its own feature's.
geo::MeshData joined(const std::vector<BodyMesh>& bodies) {
    geo::MeshData out;
    std::size_t positions = 0;
    std::size_t indices = 0;
    std::size_t faces = 0;
    std::size_t edges = 0;
    for (const BodyMesh& body : bodies) {
        positions += body.mesh->positions.size();
        indices += body.mesh->indices.size();
        faces += body.mesh->faceTags.size();
        edges += body.mesh->edges.size();
    }
    out.positions.reserve(positions);
    out.normals.reserve(positions);
    out.indices.reserve(indices);
    out.triangleFaces.reserve(indices / 3);
    out.faceTags.reserve(faces);
    out.edges.reserve(edges);
    for (const BodyMesh& body : bodies) {
        const geo::MeshData& mesh = *body.mesh;
        const auto vertexBase = static_cast<std::uint32_t>(out.positions.size() / 3);
        const auto faceBase = static_cast<std::uint32_t>(out.faceTags.size());
        out.positions.insert(out.positions.end(), mesh.positions.begin(), mesh.positions.end());
        out.normals.insert(out.normals.end(), mesh.normals.begin(), mesh.normals.end());
        for (const std::uint32_t i : mesh.indices) out.indices.push_back(vertexBase + i);
        for (const std::uint32_t f : mesh.triangleFaces) out.triangleFaces.push_back(faceBase + f);
        out.faceTags.insert(out.faceTags.end(), mesh.faceTags.begin(), mesh.faceTags.end());
        out.edges.insert(out.edges.end(), mesh.edges.begin(), mesh.edges.end());
    }
    return out;
}

/// A counterpart that could not be made because something threw, saying @p why: shown from facets,
/// and not kept for reuse (no madeFrom), since what threw (memory, say) may not again. Null only
/// where even that cannot be made.
std::shared_ptr<const ExactCounterpart> unmade(const char* why) noexcept {
    try {
        auto failed = std::make_shared<ExactCounterpart>();
        failed->note = std::string("shown from facets: ") + why;
        return failed;
    } catch (...) {
        return nullptr;
    }
}

}  // namespace

void ExactFollower::follow(const Feature& feature, bool first,
                           std::unique_ptr<topo::Solid> exactTool) noexcept {
    try {
        const auto* imported = dynamic_cast<const ImportedBodyFeature*>(&feature);
        const bool newBody = first || feature.operation() == BodyOperation::NewBody;
        if (!newBody || (exactTool == nullptr && imported == nullptr)) {
            // The part is no longer bodies built exactly: none, by design.
            m_open = false;
            m_bodies.clear();
            m_note.clear();
            m_noteKept = false;
            m_would = false;
            return;
        }
        if (!m_open || !m_note.empty()) return;
        if (!m_keep) {
            // A body it would have had (an import's is not copied to tell whether it is exact,
            // as a build keeping none never shows a note).
            m_would = true;
            return;
        }
        if (exactTool == nullptr) {
            // An import whose body has no exact form: the part has none, and says why. Made
            // from what it says, it is kept for reuse as one made from a body.
            m_bodies.clear();
            m_noteKept = true;
            m_note = "shown from facets: " +
                     (imported->note().empty()
                          ? "imported body \"" + imported->source() + "\" has no exact form"
                          : imported->note());
            return;
        }
        // Kept as it is, a body of its own: meshed on its own, then joined to the others (make()).
        m_bodies.push_back(std::move(exactTool));
    } catch (...) {
        m_open = false;
        m_bodies.clear();
        m_noteKept = false;
        m_note = "shown from facets: its exact form could not be gathered";
    }
}

std::shared_ptr<const ExactCounterpart> ExactFollower::make(
    const topo::Solid& faceted, const std::shared_ptr<const ExactCounterpart>& previous,
    BuildControl* control, bool& cancelled) noexcept {
    cancelled = false;
    try {
        if (m_bodies.empty() && m_note.empty()) return nullptr;
        auto made = std::make_shared<ExactCounterpart>();
        std::vector<std::uint64_t> definitions;  // each body's, in order
        if (!m_note.empty()) {
            made->note = m_note;
            if (m_noteKept) made->madeFrom = "note: " + m_note;
        } else {
            definitions.reserve(m_bodies.size());
            for (const auto& body : m_bodies) definitions.push_back(definitionOf(*body));
            made->madeFrom = madeFrom(definitions);
        }
        if (previous && !previous->madeFrom.empty() && previous->madeFrom == made->madeFrom) {
            // Reused whole: its part, its mesh and its bodies; none meshed now.
            if (previous->bodiesMeshed == 0) return previous;
            auto reused = std::make_shared<ExactCounterpart>(*previous);
            reused->bodiesMeshed = 0;
            return reused;
        }
        if (!m_note.empty()) return made;

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

        // Each body at the part's display tolerance, of all its bodies; a previous body meshed at
        // it, alike and named alike, is taken as it was.
        std::vector<const topo::Solid*> parts;
        parts.reserve(m_bodies.size());
        for (const auto& body : m_bodies) parts.push_back(body.get());
        const double tolerance = model::SolidTessellator::displayTolerance(parts);
        std::unordered_map<std::uint64_t, const BodyMesh*> before;
        if (previous) {
            for (const BodyMesh& body : previous->bodies) before.emplace(body.fingerprint, &body);
        }
        made->bodies.resize(m_bodies.size());
        std::vector<bool> fresh(m_bodies.size(), false);
        for (std::size_t k = 0; k < m_bodies.size(); ++k) {
            BodyMesh& body = made->bodies[k];
            body.fingerprint = meshedFrom(definitions[k], tolerance);
            const auto found = before.find(body.fingerprint);
            if (found == before.end()) {
                fresh[k] = true;
                continue;
            }
            const BodyMesh& kept = *found->second;
            body.mesh = kept.mesh;
            body.note = kept.note;
        }
        // The others meshed now, each on its own, and each held to its own facets.
        const Facets facets = facetsOf(faceted, m_bodies, fresh);
        for (std::size_t k = 0; k < m_bodies.size(); ++k) {
            if (!fresh[k]) continue;
            if (stopped()) {
                cancelled = true;
                return nullptr;
            }
            Meshed meshed = meshBody(*m_bodies[k], tolerance, faceted, facets.faces[k],
                                     facets.shells[k], control);
            if (meshed.cancelled) {
                cancelled = true;
                return nullptr;
            }
            ++made->bodiesMeshed;
            made->bodies[k].mesh = std::move(meshed.mesh);
            made->bodies[k].note = std::move(meshed.note);
        }
        // Shown exactly only while every body is, and every facet is a body's; else each body's
        // note, in order, and then the facet's.
        std::string notes;
        for (const BodyMesh& body : made->bodies) {
            if (body.note.empty()) continue;
            if (!notes.empty()) notes += "; ";
            notes += body.note;
        }
        if (!facets.note.empty()) {
            if (!notes.empty()) notes += "; ";
            notes += facets.note;
        }
        if (!notes.empty()) {
            made->note = std::move(notes);
            return made;
        }

        // The part, its bodies joined as the part gathers their facets (follow()), each freed once
        // it is joined; and its mesh, theirs one after the other. Both, or neither.
        std::shared_ptr<const geo::MeshData> shown =
            made->bodies.size() == 1 ? made->bodies.front().mesh
                                     : std::make_shared<const geo::MeshData>(joined(made->bodies));
        std::unique_ptr<topo::Solid> part = std::move(m_bodies.front());
        for (std::size_t k = 1; k < m_bodies.size(); ++k) {
            model::Pattern::append(*part, *m_bodies[k], math::Mat4::identity());
            m_bodies[k].reset();
        }
        m_bodies.clear();
        made->solid = std::shared_ptr<const topo::Solid>(std::move(part));
        made->mesh = std::move(shown);
        return made;
    } catch (const std::exception& e) {
        return unmade(e.what());
    } catch (...) {
        return unmade("its exact form could not be made");
    }
}

}  // namespace hz::doc
