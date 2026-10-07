#define _USE_MATH_DEFINES
#include "horizon/constraint/Constraint.h"

#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

#include "horizon/constraint/ParameterTable.h"

namespace hz::cstr {

// ---------------------------------------------------------------------------
// Constraint base
// ---------------------------------------------------------------------------

math::IdCounter<uint64_t> Constraint::s_nextId{1};

Constraint::Constraint() : m_id(s_nextId.next()) {}

// ---------------------------------------------------------------------------
// Helper: collect unique entity IDs from two refs
// ---------------------------------------------------------------------------

static std::vector<uint64_t> uniqueIds(uint64_t a, uint64_t b) {
    if (a == b) return {a};
    return {a, b};
}

/// @p v over its length; nothing for a zero vector, which has no direction.
static math::Vec2 unit(const math::Vec2& v) {
    const double len = v.length();
    return len > 0.0 ? v / len : math::Vec2{};
}

/// Add to row @p row of @p jac the derivative of an equation that depends on
/// a line only through its direction, end - start, as @p byDirection.
static void addByDirection(const std::pair<PointJacobian, PointJacobian>& line,
                           Eigen::MatrixXd& jac, int row, const math::Vec2& byDirection) {
    line.first.addTo(jac, row, -byDirection);
    line.second.addTo(jac, row, byDirection);
}

// ---------------------------------------------------------------------------
// CoincidentConstraint: pA == pB  (2 eqs)
// ---------------------------------------------------------------------------

CoincidentConstraint::CoincidentConstraint(const GeometryRef& pointA, const GeometryRef& pointB)
    : m_pointA(pointA), m_pointB(pointB) {}

std::vector<uint64_t> CoincidentConstraint::referencedEntityIds() const {
    return uniqueIds(m_pointA.entityId, m_pointB.entityId);
}

void CoincidentConstraint::evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                                    int offset) const {
    auto pA = params.pointPosition(m_pointA);
    auto pB = params.pointPosition(m_pointB);
    residuals(offset + 0) = pA.x - pB.x;
    residuals(offset + 1) = pA.y - pB.y;
}

void CoincidentConstraint::jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                                    int offset) const {
    // Through each point's own Jacobian: an arc's end moves with its centre,
    // radius and angle, not as two parameters of its own.
    const PointJacobian jA = params.pointJacobian(m_pointA);
    const PointJacobian jB = params.pointJacobian(m_pointB);
    // dF0/d(pA) = (1, 0),  dF0/d(pB) = (-1, 0)
    jA.addTo(jac, offset + 0, {1.0, 0.0});
    jB.addTo(jac, offset + 0, {-1.0, 0.0});
    // dF1/d(pA) = (0, 1),  dF1/d(pB) = (0, -1)
    jA.addTo(jac, offset + 1, {0.0, 1.0});
    jB.addTo(jac, offset + 1, {0.0, -1.0});
}

std::shared_ptr<Constraint> CoincidentConstraint::clone() const {
    return std::make_shared<CoincidentConstraint>(m_pointA, m_pointB);
}

// ---------------------------------------------------------------------------
// HorizontalConstraint: pA.y == pB.y  (1 eq)
// ---------------------------------------------------------------------------

HorizontalConstraint::HorizontalConstraint(const GeometryRef& refA, const GeometryRef& refB)
    : m_refA(refA), m_refB(refB) {}

std::vector<uint64_t> HorizontalConstraint::referencedEntityIds() const {
    return uniqueIds(m_refA.entityId, m_refB.entityId);
}

void HorizontalConstraint::evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                                    int offset) const {
    auto pA = params.pointPosition(m_refA);
    auto pB = params.pointPosition(m_refB);
    residuals(offset) = pA.y - pB.y;
}

void HorizontalConstraint::jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                                    int offset) const {
    params.pointJacobian(m_refA).addTo(jac, offset, {0.0, 1.0});   // d/d(pA.y)
    params.pointJacobian(m_refB).addTo(jac, offset, {0.0, -1.0});  // d/d(pB.y)
}

std::shared_ptr<Constraint> HorizontalConstraint::clone() const {
    return std::make_shared<HorizontalConstraint>(m_refA, m_refB);
}

// ---------------------------------------------------------------------------
// VerticalConstraint: pA.x == pB.x  (1 eq)
// ---------------------------------------------------------------------------

VerticalConstraint::VerticalConstraint(const GeometryRef& refA, const GeometryRef& refB)
    : m_refA(refA), m_refB(refB) {}

std::vector<uint64_t> VerticalConstraint::referencedEntityIds() const {
    return uniqueIds(m_refA.entityId, m_refB.entityId);
}

void VerticalConstraint::evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                                  int offset) const {
    auto pA = params.pointPosition(m_refA);
    auto pB = params.pointPosition(m_refB);
    residuals(offset) = pA.x - pB.x;
}

void VerticalConstraint::jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                                  int offset) const {
    params.pointJacobian(m_refA).addTo(jac, offset, {1.0, 0.0});   // d/d(pA.x)
    params.pointJacobian(m_refB).addTo(jac, offset, {-1.0, 0.0});  // d/d(pB.x)
}

std::shared_ptr<Constraint> VerticalConstraint::clone() const {
    return std::make_shared<VerticalConstraint>(m_refA, m_refB);
}

// ---------------------------------------------------------------------------
// PerpendicularConstraint: d1.dot(d2) / (|d1| |d2|) == 0  (1 eq)
// d1 = lineA.end - lineA.start,  d2 = lineB.end - lineB.start
// The cosine of the angle between them, as the angle constraint's residual
// is an angle: d1.dot(d2) alone was a length squared, and the solver's
// tolerance is absolute.
// ---------------------------------------------------------------------------

PerpendicularConstraint::PerpendicularConstraint(const GeometryRef& lineA, const GeometryRef& lineB)
    : m_lineA(lineA), m_lineB(lineB) {}

std::vector<uint64_t> PerpendicularConstraint::referencedEntityIds() const {
    return uniqueIds(m_lineA.entityId, m_lineB.entityId);
}

void PerpendicularConstraint::evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                                       int offset) const {
    auto [sA, eA] = params.lineEndpoints(m_lineA);
    auto [sB, eB] = params.lineEndpoints(m_lineB);
    const math::Vec2 d1 = eA - sA, d2 = eB - sB;
    const double n = d1.length() * d2.length();
    // A line of no length has no direction to hold.
    residuals(offset) = n > 0.0 ? d1.dot(d2) / n : 0.0;
}

void PerpendicularConstraint::jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                                       int offset) const {
    auto [sA, eA] = params.lineEndpoints(m_lineA);
    auto [sB, eB] = params.lineEndpoints(m_lineB);
    const math::Vec2 d1 = eA - sA, d2 = eB - sB;
    const double n1 = d1.length(), n2 = d2.length();
    if (n1 == 0.0 || n2 == 0.0) return;

    // F = d1.d2 / (n1 n2)
    // dF/d(d1) = d2 / (n1 n2) - F d1 / n1^2,  dF/d(d2) = d1 / (n1 n2) - F d2 / n2^2
    const double f = d1.dot(d2) / (n1 * n2);
    addByDirection(params.lineJacobian(m_lineA), jac, offset,
                   d2 / (n1 * n2) - d1 * (f / (n1 * n1)));
    addByDirection(params.lineJacobian(m_lineB), jac, offset,
                   d1 / (n1 * n2) - d2 * (f / (n2 * n2)));
}

std::shared_ptr<Constraint> PerpendicularConstraint::clone() const {
    return std::make_shared<PerpendicularConstraint>(m_lineA, m_lineB);
}

// ---------------------------------------------------------------------------
// ParallelConstraint: d1.cross(d2) / (|d1| |d2|) == 0  (1 eq)
// cross = dx1*dy2 - dy1*dx2
// The sine of the angle between them, as Perpendicular takes its cosine.
// ---------------------------------------------------------------------------

ParallelConstraint::ParallelConstraint(const GeometryRef& lineA, const GeometryRef& lineB)
    : m_lineA(lineA), m_lineB(lineB) {}

std::vector<uint64_t> ParallelConstraint::referencedEntityIds() const {
    return uniqueIds(m_lineA.entityId, m_lineB.entityId);
}

void ParallelConstraint::evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                                  int offset) const {
    auto [sA, eA] = params.lineEndpoints(m_lineA);
    auto [sB, eB] = params.lineEndpoints(m_lineB);
    const math::Vec2 d1 = eA - sA, d2 = eB - sB;
    const double n = d1.length() * d2.length();
    residuals(offset) = n > 0.0 ? d1.cross(d2) / n : 0.0;
}

void ParallelConstraint::jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                                  int offset) const {
    auto [sA, eA] = params.lineEndpoints(m_lineA);
    auto [sB, eB] = params.lineEndpoints(m_lineB);
    const math::Vec2 d1 = eA - sA, d2 = eB - sB;
    const double n1 = d1.length(), n2 = d2.length();
    if (n1 == 0.0 || n2 == 0.0) return;

    // F = (dx1*dy2 - dy1*dx2) / (n1 n2)
    // dF/d(d1) = (dy2, -dx2) / (n1 n2) - F d1 / n1^2
    // dF/d(d2) = (-dy1, dx1) / (n1 n2) - F d2 / n2^2
    const double f = d1.cross(d2) / (n1 * n2);
    addByDirection(params.lineJacobian(m_lineA), jac, offset,
                   math::Vec2{d2.y, -d2.x} / (n1 * n2) - d1 * (f / (n1 * n1)));
    addByDirection(params.lineJacobian(m_lineB), jac, offset,
                   math::Vec2{-d1.y, d1.x} / (n1 * n2) - d2 * (f / (n2 * n2)));
}

std::shared_ptr<Constraint> ParallelConstraint::clone() const {
    return std::make_shared<ParallelConstraint>(m_lineA, m_lineB);
}

// ---------------------------------------------------------------------------
// TangentConstraint: |signed_dist(line, center)| - radius == 0  (1 eq)
// signed_dist = ((center - lineStart) cross d) / |d|,  d = lineEnd - lineStart
// A length, as the solver's tolerance is. It was signed_dist^2 |d|^2 -
// radius^2 |d|^2, a length to the fourth: for a line of 300 on a circle of
// 500 about 2e10, whose last place alone (4e-6) is 40,000 times the
// tolerance. Either side of the line is tangent, as before.
// ---------------------------------------------------------------------------

TangentConstraint::TangentConstraint(const GeometryRef& refA, const GeometryRef& refB)
    : m_lineRef(refA), m_circleRef(refB) {
    // Picked in either order. Kept as picked, a circle picked first was read
    // as the line, and every solve threw; refused here, a pair that is not a
    // line and a circle never reaches one.
    if (m_lineRef.featureType == FeatureType::Circle) std::swap(m_lineRef, m_circleRef);
    if (m_lineRef.featureType != FeatureType::Line ||
        m_circleRef.featureType != FeatureType::Circle) {
        throw std::invalid_argument("a tangent is between a line and a circle or arc");
    }
}

std::vector<uint64_t> TangentConstraint::referencedEntityIds() const {
    return uniqueIds(m_lineRef.entityId, m_circleRef.entityId);
}

void TangentConstraint::evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                                 int offset) const {
    auto [s, e] = params.lineEndpoints(m_lineRef);
    auto [center, radius] = params.circleData(m_circleRef);
    const math::Vec2 d = e - s;
    const double len = d.length();
    // A line of no length has no direction to be tangent along.
    residuals(offset) = len > 0.0 ? std::abs((center - s).cross(d)) / len - radius : 0.0;
}

void TangentConstraint::jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                                 int offset) const {
    auto [s, e] = params.lineEndpoints(m_lineRef);
    auto [center, radius] = params.circleData(m_circleRef);
    const math::Vec2 d = e - s, dc = center - s;
    const double len = d.length();
    const int iC = params.parameterIndex(m_circleRef);  // [cx, cy, r]
    // dF/d(r) = -1
    jac(offset, iC + 2) += -1.0;
    if (len == 0.0) return;

    // F = sign * h - r,  h = cross / len,  cross = dc.cross(d) = dcx*dy - dcy*dx,
    // sign the side of the line the centre is on (+ on it).
    // d(cross)/d(s) = (dcy - dy, dx - dcx),  d(cross)/d(e) = (-dcy, dcx),
    // d(cross)/d(c) = (dy, -dx);  d(len)/d(s) = -d / len,  d(len)/d(e) = d / len.
    // dh/d(var) = d(cross)/d(var) / len - cross d(len)/d(var) / len^2
    const double cross = dc.cross(d);
    const double sign = cross < 0.0 ? -1.0 : 1.0;
    const double k = cross / (len * len * len);
    const auto [jS, jE] = params.lineJacobian(m_lineRef);
    jS.addTo(jac, offset, math::Vec2{dc.y - d.y, d.x - dc.x} * (sign / len) + d * (sign * k));
    jE.addTo(jac, offset, math::Vec2{-dc.y, dc.x} * (sign / len) - d * (sign * k));
    jac(offset, iC + 0) += sign * d.y / len;
    jac(offset, iC + 1) += -sign * d.x / len;
}

std::shared_ptr<Constraint> TangentConstraint::clone() const {
    return std::make_shared<TangentConstraint>(m_lineRef, m_circleRef);
}

// ---------------------------------------------------------------------------
// EqualConstraint: equal length (lines) or equal radius (circles)  (1 eq)
// Lines:   |d1| - |d2| == 0  (a length, as the radii's is; the squares
//          were a length squared)
// Circles: r1 - r2 == 0
// ---------------------------------------------------------------------------

EqualConstraint::EqualConstraint(const GeometryRef& refA, const GeometryRef& refB)
    : m_refA(refA), m_refB(refB) {
    // Lengths or radii, as the first ref is: a line beside a circle was read
    // as a second line, and every solve threw.
    if (refA.featureType != refB.featureType || refA.featureType == FeatureType::Point) {
        throw std::invalid_argument("equal is between two lines or two circles");
    }
}

std::vector<uint64_t> EqualConstraint::referencedEntityIds() const {
    return uniqueIds(m_refA.entityId, m_refB.entityId);
}

void EqualConstraint::evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                               int offset) const {
    if (m_refA.featureType == FeatureType::Line) {
        auto [sA, eA] = params.lineEndpoints(m_refA);
        auto [sB, eB] = params.lineEndpoints(m_refB);
        residuals(offset) = (eA - sA).length() - (eB - sB).length();
    } else {
        auto [cA, rA] = params.circleData(m_refA);
        auto [cB, rB] = params.circleData(m_refB);
        residuals(offset) = rA - rB;
    }
}

void EqualConstraint::jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                               int offset) const {
    if (m_refA.featureType == FeatureType::Line) {
        auto [sA, eA] = params.lineEndpoints(m_refA);
        auto [sB, eB] = params.lineEndpoints(m_refB);
        // F = |dA| - |dB|:  dF/d(dA) = dA / |dA|,  dF/d(dB) = -dB / |dB| (none
        // for a line of no length, which has no direction to grow along)
        addByDirection(params.lineJacobian(m_refA), jac, offset, unit(eA - sA));
        addByDirection(params.lineJacobian(m_refB), jac, offset, -unit(eB - sB));
    } else {
        int iA = params.parameterIndex(m_refA);
        int iB = params.parameterIndex(m_refB);
        // F = rA - rB; radius is 3rd param in circle: [cx, cy, r]
        jac(offset, iA + 2) += 1.0;
        jac(offset, iB + 2) += -1.0;
    }
}

std::shared_ptr<Constraint> EqualConstraint::clone() const {
    return std::make_shared<EqualConstraint>(m_refA, m_refB);
}

// ---------------------------------------------------------------------------
// FixedConstraint: p == target  (2 eqs)
// ---------------------------------------------------------------------------

FixedConstraint::FixedConstraint(const GeometryRef& pointRef, const math::Vec2& position)
    : m_pointRef(pointRef), m_position(position) {}

std::vector<uint64_t> FixedConstraint::referencedEntityIds() const {
    return {m_pointRef.entityId};
}

void FixedConstraint::evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                               int offset) const {
    auto p = params.pointPosition(m_pointRef);
    residuals(offset + 0) = p.x - m_position.x;
    residuals(offset + 1) = p.y - m_position.y;
}

void FixedConstraint::jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                               int offset) const {
    const PointJacobian j = params.pointJacobian(m_pointRef);
    j.addTo(jac, offset + 0, {1.0, 0.0});
    j.addTo(jac, offset + 1, {0.0, 1.0});
}

std::shared_ptr<Constraint> FixedConstraint::clone() const {
    return std::make_shared<FixedConstraint>(m_pointRef, m_position);
}

// ---------------------------------------------------------------------------
// DistanceConstraint: dist(A,B) - |value| == 0  (1 eq)
// A length, as the solver's tolerance is. The squared form, dist^2 -
// value^2, was a length squared: met, at a distance of 1000, its rounding
// alone (one unit in the last place of a million) was more than the
// tolerance. Nor did it spare the square root's corner at zero distance: its
// derivative, 2 (A - B), is zero there too, and a distance of 0 was taken as
// met with the points up to 1e-5 apart.
// ---------------------------------------------------------------------------

DistanceConstraint::DistanceConstraint(const GeometryRef& refA, const GeometryRef& refB,
                                       double distance)
    : m_refA(refA), m_refB(refB), m_distance(distance) {}

std::vector<uint64_t> DistanceConstraint::referencedEntityIds() const {
    return uniqueIds(m_refA.entityId, m_refB.entityId);
}

void DistanceConstraint::evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                                  int offset) const {
    auto pA = params.pointPosition(m_refA);
    auto pB = params.pointPosition(m_refB);
    residuals(offset) = (pA - pB).length() - std::abs(m_distance);
}

void DistanceConstraint::jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                                  int offset) const {
    auto pA = params.pointPosition(m_refA);
    auto pB = params.pointPosition(m_refB);
    // F = |A - B| - |value|:  dF/dA = (A - B) / |A - B| = -dF/dB, none where
    // the points meet
    const math::Vec2 u = unit(pA - pB);
    params.pointJacobian(m_refA).addTo(jac, offset, u);
    params.pointJacobian(m_refB).addTo(jac, offset, -u);
}

std::shared_ptr<Constraint> DistanceConstraint::clone() const {
    auto c = std::make_shared<DistanceConstraint>(m_refA, m_refB, m_distance);
    c->setVariableReference(variableReference());
    return c;
}

// ---------------------------------------------------------------------------
// AngleConstraint: atan2(cross, dot) - value == 0  (1 eq)
// ---------------------------------------------------------------------------

AngleConstraint::AngleConstraint(const GeometryRef& lineA, const GeometryRef& lineB,
                                 double angleRad)
    : m_lineA(lineA), m_lineB(lineB), m_angle(angleRad) {}

std::vector<uint64_t> AngleConstraint::referencedEntityIds() const {
    return uniqueIds(m_lineA.entityId, m_lineB.entityId);
}

void AngleConstraint::evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                               int offset) const {
    auto [sA, eA] = params.lineEndpoints(m_lineA);
    auto [sB, eB] = params.lineEndpoints(m_lineB);
    double dx1 = eA.x - sA.x, dy1 = eA.y - sA.y;
    double dx2 = eB.x - sB.x, dy2 = eB.y - sB.y;
    double dot = dx1 * dx2 + dy1 * dy2;
    double cross = dx1 * dy2 - dy1 * dx2;
    double angle = std::atan2(cross, dot);
    // Normalize difference to [-pi, pi]
    double diff = angle - m_angle;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    residuals(offset) = diff;
}

void AngleConstraint::jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                               int offset) const {
    auto [sA, eA] = params.lineEndpoints(m_lineA);
    auto [sB, eB] = params.lineEndpoints(m_lineB);
    double dx1 = eA.x - sA.x, dy1 = eA.y - sA.y;
    double dx2 = eB.x - sB.x, dy2 = eB.y - sB.y;
    double dot = dx1 * dx2 + dy1 * dy2;
    double cross = dx1 * dy2 - dy1 * dx2;
    double denom = dot * dot + cross * cross;
    if (denom < 1e-30) return;  // Degenerate

    // theta = atan2(cross, dot)
    // d(theta)/d(var) = (dot * d(cross)/d(var) - cross * d(dot)/d(var)) / (dot^2 + cross^2)
    // d(dot)/d(d1) = d2,            d(dot)/d(d2) = d1
    // d(cross)/d(d1) = (dy2, -dx2), d(cross)/d(d2) = (-dy1, dx1)
    const math::Vec2 byD1{(dot * dy2 - cross * dx2) / denom, (-dot * dx2 - cross * dy2) / denom};
    const math::Vec2 byD2{(-dot * dy1 - cross * dx1) / denom, (dot * dx1 - cross * dy1) / denom};
    addByDirection(params.lineJacobian(m_lineA), jac, offset, byD1);
    addByDirection(params.lineJacobian(m_lineB), jac, offset, byD2);
}

std::shared_ptr<Constraint> AngleConstraint::clone() const {
    auto c = std::make_shared<AngleConstraint>(m_lineA, m_lineB, m_angle);
    c->setVariableReference(variableReference());
    return c;
}

}  // namespace hz::cstr
