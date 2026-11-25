#pragma once

// Seams for periodic faces that have none. Solid Edge, Alibre, NX and Onshape write a cylinder's
// side between its two rims as a band: two loops, each once round the surface, and no seam. The
// exact kernel's loops close in (u, v), so such a face is given the seam its file left out.

#include <string>

#include "horizon/math/ModelTolerance.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// Gives every face of @p solid whose surface is periodic in u, and whose loops wind round it,
/// a seam: each winding loop's edges are split where they cross u0 (the outer loop's first
/// vertex's u), and a seam edge along the surface's u0 iso-curve joins the splits, used once
/// each way. Exact fields and carriers are both split. Empty on success; else why.
///
/// - **Or v.** A face whose loops wind round in v instead (round a torus's tube, a pipe's elbow;
///   a B-spline closed that way) gets its seam along v the same way. A loop that winds round both
///   ways (a helical band on a torus) is refused, with why: it needs a seam each way, which this
///   stage does not lay.
/// - **The seam's curve** is the surface's own iso-curve, parametrised by the other parameter:
///   a LineCurve on a cylinder or a cone, a CircleCurve meridian on a sphere or a torus (a
///   parallel for a torus's v seam), and on a B-spline the NurbsCurve it gives exactly. Where
///   none is exact, no seam is laid, and why says so.
/// - **Where it runs.** u0 is tried first. Where u0 runs through another loop of the face (a
///   hole), or a winding loop crosses it more than once, the seam takes the middle of one of the
///   widest gaps between the loops' vertices and holes instead, up to 16 lines in all.
/// - **Which way round.** On a surface periodic in the seam's direction too (a torus), a seam
///   between two loops can run either way round; it runs the way that keeps the face on the side
///   its `reversed` says.
/// - **What it needs.** Exactly two loops wind round, once each and opposite ways. A face that
///   already has a seam is untouched, as is one without an exact surface, or with a loop that
///   does not close or a half-edge without a pcurve: the exact validator says why of those.
/// - **The splits.** Each new vertex lies on its edge's exact curve. Both of the edge's
///   half-edges keep their pcurves, split at the matching parameters, and the vertex's tolerance
///   covers how far those pcurves have it. Each new edge keeps its edge's tolerance, and the
///   seam's tolerance covers how far its curve passes from the vertices it joins.
/// - **Failure.** A face that cannot be given its seam may be left with edges split. The solid
///   stays combinatorially valid.
std::string insertSeams(topo::Solid& solid, const math::ModelTolerance& tolerance);

/// insertSeams for the one face @p face of @p solid, so that a caller can say which face could
/// not have its seam. Splitting an edge also splits its twin's half-edge, on the face next to it.
std::string insertSeams(topo::Solid& solid, topo::Face& face,
                        const math::ModelTolerance& tolerance);

}  // namespace hz::model
