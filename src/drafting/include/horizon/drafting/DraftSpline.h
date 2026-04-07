#pragma once

#include "DraftEntity.h"
#include <vector>

namespace hz::draft {

/// Cubic B-spline entity defined by control points.
///
/// The curve is evaluated as a uniform cubic B-spline.  At least 4 control
/// points are required for a valid open spline; fewer points degenerate to
/// a simple polyline through the control points.
///
/// Optional per-control-point weights allow NURBS-style rational curves.
/// Default weight for every point is 1.0 (unweighted B-spline, backward
/// compatible).  When all weights equal 1.0 the curve is identical to the
/// previous non-weighted evaluation.
class DraftSpline : public DraftEntity {
public:
    explicit DraftSpline(const std::vector<math::Vec2>& controlPoints, bool closed = false);

    math::BoundingBox boundingBox() const override;
    bool hitTest(const math::Vec2& point, double tolerance) const override;
    std::vector<math::Vec2> snapPoints() const override;
    void translate(const math::Vec2& delta) override;
    std::shared_ptr<DraftEntity> clone() const override;
    void mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) override;
    void rotate(const math::Vec2& center, double angle) override;
    void scale(const math::Vec2& center, double factor) override;

    const std::vector<math::Vec2>& controlPoints() const { return m_controlPoints; }
    void setControlPoints(const std::vector<math::Vec2>& points);

    /// Per-control-point rational weights (must match controlPoints().size()).
    /// Default is all 1.0. When any weight != 1.0 the evaluation uses
    /// rational (NURBS) blending.
    const std::vector<double>& weights() const { return m_weights; }
    void setWeights(const std::vector<double>& weights);

    /// Returns true if any weight differs from 1.0 by more than 1e-12.
    bool hasNonUniformWeights() const;

    bool closed() const { return m_closed; }
    void setClosed(bool closed) { m_closed = closed; }

    size_t controlPointCount() const { return m_controlPoints.size(); }

    /// Evaluate the B-spline to a polyline.
    /// @param segmentsPerSpan  Number of line segments per B-spline span.
    std::vector<math::Vec2> evaluate(int segmentsPerSpan = 16) const;

private:
    std::vector<math::Vec2> m_controlPoints;
    std::vector<double> m_weights;  ///< parallel to m_controlPoints, all 1.0 by default
    bool m_closed;

    /// Ensure m_weights is sized to match m_controlPoints, padding with 1.0.
    void syncWeights();
};

}  // namespace hz::draft
