#pragma once

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
};

/// Name every edge of @p solid after the two faces it separates, so the name
/// lasts as long as theirs do — whichever operation last built the edge. With
/// the face tags a and b in sorted order: `<src>/edge:<a'>|<b'>` when both
/// start with the same `<src>/` (a' and b' are the rest), else
/// `edge:<a>|<b>`. Edges between the same two faces are told apart by `#<k>`
/// in storage order. An edge missing a named face keeps its name.
void nameEdgesByFaces(topo::Solid& solid);

/// Give each piece of a face that an operation split its own name,
/// `<face>/piece:<k>` in storage order; a face in one piece keeps its name. A
/// reference to the whole face resolves to its pieces as descendants.
void nameFacePieces(topo::Solid& solid);

}  // namespace hz::model
