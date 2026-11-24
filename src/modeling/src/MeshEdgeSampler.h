#pragma once

// Internal to TrimmedFaceMesher: each edge discretised once for both its faces, and each face's
// grid planned first so that straight edges along a surface bent one way meet its rows.

#include <algorithm>
#include <cstddef>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/TrimmedFaceMesher.h"
#include "horizon/topology/Solid.h"

namespace hz::model::detail {

using math::Vec2;
using math::Vec3;
using topo::Edge;
using topo::Face;
using topo::HalfEdge;

inline constexpr double kInfinity = std::numeric_limits<double>::infinity();

template <typename V>  // Vec2 or Vec3
double distanceToSegment(const V& p, const V& a, const V& b) {
    const V ab = b - a;
    const double len2 = ab.lengthSquared();
    const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
    return (p - (a + ab * t)).length();
}

/// The angle between two directions, neither of them zero.
inline double angleBetween(const Vec3& a, const Vec3& b) {
    return std::atan2(a.cross(b).length(), a.dot(b));
}

/// An edge discretised once for both its faces.
struct EdgeSamples {
    std::vector<double> t;     // curve parameters, curveStart to curveEnd
    std::vector<Vec3> points;  // the curve at t; at the ends, the vertices' own points
    std::string problem;
};

/// How the face's (u, v) is scaled for triangulating, and the grid steps its bending allows.
struct Spacing {
    double su = 1.0;  // w = (u su, v sv): near lengths on the surface, for Delaunay to judge
    double sv = 1.0;
    double du = kInfinity;
    double dv = kInfinity;
    int flat = -1;  // for a surface bent one way, the parameter (0 u, 1 v) it is flat along
};

/// A face's grid, planned before its edges are sampled so that straight edges along a surface
/// bent one way can be sampled at its rows.
struct Plan {
    Vec2 lo;
    Vec2 hi;
    Spacing spacing;
};

/// One segment of an edge's samples, by its index in the edge's own order.
using Segment = std::pair<const Edge*, std::size_t>;

/// Samples every edge of a solid once, for both its faces, and plans every face's grid.
class EdgeSampler {
public:
    EdgeSampler(const topo::Solid& solid, const MeshOptions& options)
        : solid_(solid), opt_(options) {}

    /// Plans every live face's grid, then samples every live edge.
    void sampleEdges();

    /// Halves the given segments of each edge, for both its faces, and suits the edge to them
    /// again. The faces whose boundaries changed, in no particular order.
    std::vector<const Face*> halveSegments(const std::vector<Segment>& segments);

    /// @p e's samples; null for an edge that was not sampled.
    const EdgeSamples* samples(const Edge* e) const {
        const auto it = edges_.find(e);
        return it == edges_.end() ? nullptr : &it->second;
    }
    /// @p h's pcurve parameter at each of its edge's samples, in its own direction; null for a
    /// half-edge that was not sampled.
    const std::vector<double>* parameters(const HalfEdge* h) const {
        const auto it = params_.find(h);
        return it == params_.end() ? nullptr : &it->second;
    }
    /// @p f's grid; null for a face that could not be planned.
    const Plan* plan(const Face* f) const {
        const auto it = plans_.find(f);
        return it == plans_.end() ? nullptr : &it->second;
    }
    /// The model tolerance loops are laid out with.
    const math::ModelTolerance& tolerance() const { return tol_; }

private:
    /// How far one direction may sag: an edge's chords (also as their (u, v) midpoints map onto
    /// each face), a grid cell's sides. A triangle's error combines two directions, and near an
    /// edge its own; with each held to half the tolerance, refinement at a boundary converges
    /// (a triangle on a boundary chord errs by at most 8/9 of the chord's sag there).
    double sideTolerance() const { return 0.5 * opt_.chordTolerance; }

    /// How far one step may turn where the angle, not the tolerance, sets it: an edge's chord, a
    /// grid cell's side. Half maxAngle, so that the first mesh already keeps triangles that
    /// share an edge within maxAngle of one another (each within a half step's turn of the
    /// surface there). Steps at the whole angle left about a fifth of the triangles on a thin
    /// torus turned, and refining them cascaded to 15 times the triangles.
    double stepAngle() const { return 0.5 * opt_.maxAngle; }

    void planFace(const Face& f);
    double rowsAlong(const HalfEdge& h) const;
    bool turns(const geo::Surface& s, const Vec2& a, const Vec2& c, const Vec2& b) const;
    void sampleEdge(const Edge& e, EdgeSamples& es);
    void suitEdge(const Edge& e, EdgeSamples& es);
    Spacing spacingOf(const geo::Surface& s, const Vec2& lo, const Vec2& hi) const;

    const topo::Solid& solid_;
    const MeshOptions& opt_;
    math::ModelTolerance tol_;
    std::unordered_map<const Edge*, EdgeSamples> edges_;
    std::unordered_map<const Face*, Plan> plans_;
    /// Each sampled half-edge's pcurve parameter at each of its edge's samples, in its own
    /// direction.
    std::unordered_map<const HalfEdge*, std::vector<double>> params_;
};

}  // namespace hz::model::detail
