#pragma once

#include <string>
#include <vector>

#include "horizon/math/ModelTolerance.h"
#include "horizon/topology/Solid.h"

namespace hz::topo {

enum class ExactIssueKind {
    MissingGeometry,  ///< A face has no exact surface, an edge no curve, a half-edge no pcurve.
    VertexOffCurve,   ///< An edge's curve misses one of its vertices at its end parameter.
    PcurveOffCurve,   ///< A pcurve, mapped onto its surface, strays from the edge's curve.
    PcurveOffVertex,  ///< A pcurve's end, mapped onto its surface, misses its vertex.
    LoopNotClosed,    ///< The half-edge cycle breaks, or consecutive pcurves do not join.
    LoopWrongOrientation,  ///< A loop's signed area in (u, v) has the wrong sign for its face.
    LoopSelfIntersects,    ///< A loop crosses itself.
    LoopsCross,            ///< Two loops of one face cross.
    HoleOutsideOuterLoop,  ///< An inner loop lies outside the face's outer loop.
    HoleInsideHole         ///< An inner loop lies inside another inner loop of the face.
};

struct ExactIssue {
    ExactIssueKind kind = ExactIssueKind::MissingGeometry;
    const Face* face = nullptr;
    const Edge* edge = nullptr;
    std::string detail;
};

/// Every way @p solid's exact geometry disagrees with its topology; empty when it agrees.
/// The solid must be combinatorially valid (`Solid::isValid`): twins, next pointers and origins
/// are followed without null checks. Each pcurve is sampled at 17 parameters (its two ends and
/// 15 between). Closeness tests use @p tolerance together with the edge's and vertices' own
/// tolerances. Pass topo::exactTolerance(solid) (horizon/topology/ExactTolerance.h): the validator
/// then lays the loops out in parameter space exactly as the mesher does. That is not a promise
/// that the mesher succeeds: it has a triangle cap and a refinement limit, and loops touching at
/// one point pass here but are reported by the mesher.
std::vector<ExactIssue> validateExact(const Solid& solid, const math::ModelTolerance& tolerance);

}  // namespace hz::topo
