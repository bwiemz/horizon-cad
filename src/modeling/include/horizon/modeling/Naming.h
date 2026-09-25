#pragma once

#include <string>

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
