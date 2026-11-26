#pragma once

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/SolidSewer.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// How a feature names the faces and edges it builds.
enum class NamingScheme {
    /// By position: side faces `lateral_<i>` in profile order, edges
    /// `edge<N>` in storage order. Any change in how many there are renames
    /// the rest. Kept for documents saved before persistent naming, so their
    /// references resolve as they always did.
    Positional = 1,
    /// From what generated them: side faces after the sketch entity they come
    /// from, edges after the two faces they separate (see `nameEdgesByFaces`).
    /// An edit elsewhere in the sketch leaves a name alone.
    FromGeometry = 2,
    /// FromGeometry, and more that lasts (Phase 139): a primitive's names
    /// are its feature's (`primitive_3/top`, where every box's was `box/top`);
    /// a fillet or chamfer leaves the names of the edges it did not touch;
    /// Revolve, Loft and Sweep name faces after the profile element they
    /// come from; and the facets of one curved surface, and the chords of one
    /// curve, are named as parts of it (`<face>/facet:<k>`, `<edge>/chord:<k>`),
    /// so a reference to the surface or curve takes them all. Documents saved
    /// with FromGeometry keep FromGeometry's names.
    Stable = 3,
};

/// Whether @p naming names from geometry (FromGeometry or Stable), not by
/// position.
inline bool namesFromGeometry(NamingScheme naming) {
    return naming != NamingScheme::Positional;
}

/// Name every edge of @p solid after the two faces it separates, so the name
/// lasts as long as theirs do — whichever operation last built the edge. With
/// the face tags a and b in sorted order: `<src>/edge:<a'>|<b'>` when both
/// start with the same `<src>/` (a' and b' are the rest), else
/// `edge:<a>|<b>`. Edges between the same two faces are told apart by `#<k>`
/// in storage order. An edge missing a named face keeps its name.
void nameEdgesByFaces(topo::Solid& solid);

/// The logical face a face name belongs to: the name without its
/// `/facet:<k>` (a facet of a curved face is `<face>/facet:<k>`). A piece of
/// a facet a boolean split, `<face>/facet:<k>/piece:<n>`, is of the face as
/// well: a cylinder cut in two is one side. What else follows the facet is
/// kept: a pattern copy's facet, `<face>/facet:<k>/pattern:<n>`, belongs to
/// the copy's face, and a piece of it, `.../pattern:<n>/piece:<j>`, too.
///
/// Older names have it too: a loft's twisted level has always been cut into
/// triangles named `<side>/facet:<k>`, in every scheme. They are facets of
/// their side's ruled patch, which each carries as its ideal, and so of that
/// side: a reference to it finds a triangle with the same ideal.
std::string logicalFace(const std::string& faceTag);

/// The logical edge an edge name belongs to: the name without its
/// `/chord:<k>` (a chord of a curve is `<edge>/chord:<k>`), what follows it
/// kept, as `logicalFace` keeps it.
std::string logicalEdge(const std::string& edgeTag);

/// Name every edge of @p solid as the Stable scheme does: after the logical
/// faces it parts (as `nameEdgesByFaces` does after the faces). An edge
/// between two facets of one face is a seam of it, `<face>/seam:<n>`;
/// several edges between the same two logical faces are the chords of one
/// curve, `<edge>/chord:<n>`, so a reference to the curve takes them all.
void nameEdgesLogically(topo::Solid& solid);

/// Name the edges of @p faceted, the bridge copy (model::facetExact) of @p exact, whose edges are
/// named (nameEdgesLogically), as a feature built exactly names them (stage 3, Ruling 18). Each
/// chord lying on an exact edge keeps the name facetExact gave it, `<edge>/chord:<k>` (or the
/// edge's own, where it is one chord), so `logicalEdge` takes it to that edge, a seam's chords
/// included. Each other edge between two facets of one face, a line of its grid, is a seam of
/// it, `<face>/seam:<n>` in storage order, as nameEdgesLogically names one, its numbers skipping
/// every name an edge of @p exact has: no line is taken for an exact edge it does not lie on.
/// What else the bridge holds keeps its name.
void nameBridgeEdges(topo::Solid& faceted, const topo::Solid& exact);

/// A faceted construction of master's as it named what it made (stage 3, Ruling 21): its points,
/// its faces (each a loop of points) with master's names for them, and its edges in the order
/// master made them, each a pair of points, which is the order nameEdgesLogically numbered them
/// in. With no edges given, the order a sewer (SolidSewer) first meets them going round the
/// faces in order, as it makes them.
struct MastersFacets {
    std::vector<math::Vec3> points;
    std::vector<std::vector<std::size_t>> faces;
    std::vector<std::string> faceNames;
    std::vector<std::pair<std::size_t, std::size_t>> edges;
};

/// @p soup, the polygons one of master's faceted primitives sewed in order
/// (PrimitiveFactory::cylinderFaces and the like), named as master's Stable primitive named the
/// faces sewn from them: each name scoped to @p featureID (scopeToFeature), and the faces of its
/// curved side, those whose names hold @p curvedSide (`cylinder/side`, the faces
/// PrimitiveFactory gives one ideal surface), `<featureID>/<role>/facet:<k>` in order
/// (nameFacetsLogically); every point moved by @p placement, as master's placed primitive was.
MastersFacets mastersFacets(const std::vector<SolidSewer::InputFace>& soup,
                            const std::string& curvedSide, const std::string& featureID,
                            const math::Mat4& placement);

/// Name @p bridge, the bridge copy (model::facetExact) of @p exact, whose names nameBridgeEdges
/// gave, as @p masters named the same places (stage 3, Ruling 21): a saved name of master's
/// facets and edges binds where it bound. Each facet whose corners are a face of master's takes
/// its name; each edge between two of master's points takes the name master's edge between them
/// had, numbered as nameEdgesLogically numbered master's (a curve's chords `<edge>/chord:<k>` and
/// the lines inside a face `<face>/seam:<k>` in the order master made them). An exact edge inside
/// one face (a seam: a cylinder's line at u = 0, a sphere's meridian) is renamed in @p exact
/// too: after master's edge where its first chord lies (from its half-edge's origin), its chords
/// `<that name>/chord:<k>` where it has several, so that logicalEdge still takes each to it and
/// no line of a grid takes its name (Ruling 18). Master's names of the other chords of such a
/// seam are no name of the bridge's. What has no place in master's construction (a facet cut in
/// two where it was not flat, its diagonal) keeps its name, renumbered past every name now taken.
/// Nothing is renamed when @p masters has no points.
void nameAsMasters(topo::Solid& bridge, topo::Solid& exact, const MastersFacets& masters);

/// Name @p solid's edges as @p naming does: logically (Stable), after their
/// faces (FromGeometry). Positional names are an operation's own; they are
/// left as they are.
void nameEdges(topo::Solid& solid, NamingScheme naming);

/// Name the facets of each curved face of @p solid as parts of it: faces
/// that share one ideal surface (`analyticSurface`) become
/// `<source>/<role>/facet:<k>`, where <role> is what their names share with
/// the digits and underscores at its end trimmed (a cylinder's `side3` is
/// `side/facet:3`). A face alone on its surface keeps its name.
void nameFacetsLogically(topo::Solid& solid);

/// Name the edges of @p result, which an operation built from @p input,
/// keeping what they were: an edge between two faces that were joined by
/// exactly one edge in @p input, and are by exactly one here, keeps that
/// edge's name; every other edge is named after its faces
/// (`nameEdgesByFaces`). A fillet so leaves the names of the edges it did not
/// touch, where it renamed every edge in storage order. Used by the Stable
/// scheme, so the rest are named logically (`nameEdgesLogically`).
void keepEdgeNames(topo::Solid& result, const topo::Solid& input);

/// Name the faces a fillet or chamfer added as the Stable scheme does: one
/// face for each curve it rounds, not one for each chord and band of it.
/// They are `<prefix><edge>:<n>`, @p prefix being `<feature>/fillet/` or
/// `<feature>/chamfer/`; they become `<prefix><curve>/facet:<k>`, <curve>
/// the logical edge of <edge>, or `<prefix><curve>` when it is one face.
/// Named after their chords, the rounded rim of a cylinder was 256 faces,
/// and the names of the edges between them held the chords' `/chord:`, which
/// `logicalEdge` then took for theirs. Run before `keepEdgeNames`.
void nameBlendFaces(topo::Solid& solid, const std::string& prefix);

/// Scope every face and edge name of @p solid to the feature @p featureID:
/// the source a name starts with (`box` in `box/top`) becomes @p featureID
/// (`primitive_3/top`), so two features of one kind no longer share names.
/// A name without a source is scoped whole (`primitive_3/<name>`).
void scopeToFeature(topo::Solid& solid, const std::string& featureID);

/// Give each piece of a face that an operation split its own name,
/// `<face>/piece:<k>` in storage order; a face in one piece keeps its name. A
/// reference to the whole face resolves to its pieces as descendants.
void nameFacePieces(topo::Solid& solid);

}  // namespace hz::model
