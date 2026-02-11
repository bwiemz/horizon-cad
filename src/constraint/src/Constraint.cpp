#define _USE_MATH_DEFINES
#include <cmath>

#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ParameterTable.h"
#include <set>

namespace hz::cstr {

// ---------------------------------------------------------------------------
// Constraint base
// ---------------------------------------------------------------------------

uint64_t Constraint::s_nextId = 1;

Constraint::Constraint() : m_id(s_nextId++) {}

// ---------------------------------------------------------------------------
// Helper: collect unique entity IDs from two refs
// ---------------------------------------------------------------------------

static std::vector<uint64_t> uniqueIds(uint64_t a, uint64_t b) {
    if (a == b) return {a};
    return {a, b};
}

// ---------------------------------------------------------------------------
// CoincidentConstraint: pA == pB  (2 eqs)
// ---------------------------------------------------------------------------

CoincidentConstraint::CoincidentConstraint(const GeometryRef& pointA,
                                           const GeometryRef& pointB)
    : m_pointA(pointA), m_pointB(pointB) {}

std::vector<uint64_t> CoincidentConstraint::referencedEntityIds() const {
    return uniqueIds(m_pointA.entityId, m_pointB.entityId);
}

void CoincidentConstraint::evaluate(const ParameterTable& params,
                                     Eigen::VectorXd& residuals, int offset) const {
    auto pA = params.pointPosition(m_pointA);
    auto pB = params.pointPosition(m_pointB);
    residuals(offset + 0) = pA.x - pB.x;
    residuals(offset + 1) = pA.y - pB.y;
}

void CoincidentConstraint::jacobian(const ParameterTable& params,
                                     Eigen::MatrixXd& jac, int offset) const {
    int iA = params.parameterIndex(m_pointA);
    int iB = params.parameterIndex(m_pointB);
    // dF0/d(pA.x) = 1,  dF0/d(pB.x) = -1
    jac(offset + 0, iA + 0) += 1.0;
    jac(offset + 0, iB + 0) += -1.0;
    // dF1/d(pA.y) = 1,  dF1/d(pB.y) = -1
    jac(offset + 1, iA + 1) += 1.0;
    jac(offset + 1, iB + 1) += -1.0;
}

std::shared_ptr<Constraint> CoincidentConstraint::clone() const {
    return std::make_shared<CoincidentConstraint>(m_pointA, m_pointB);
}

// ---------------------------------------------------------------------------
// HorizontalConstraint: pA.y == pB.y  (1 eq)
// ---------------------------------------------------------------------------

HorizontalConstraint::HorizontalConstraint(const GeometryRef& refA,
                                           const GeometryRef& refB)
    : m_refA(refA), m_refB(refB) {}

std::vector<uint64_t> HorizontalConstraint::referencedEntityIds() const {
    return uniqueIds(m_refA.entityId, m_refB.entityId);
}

void HorizontalConstraint::evaluate(const ParameterTable& params,
                                     Eigen::VectorXd& residuals, int offset) const {
    auto pA = params.pointPosition(m_refA);
    auto pB = params.pointPosition(m_refB);
    residuals(offset) = pA.y - pB.y;
}

void HorizontalConstraint::jacobian(const ParameterTable& params,
                                     Eigen::MatrixXd& jac, int offset) const {
    int iA = params.parameterIndex(m_refA);
    int iB = params.parameterIndex(m_refB);
    jac(offset, iA + 1) += 1.0;   // d/d(pA.y)
    jac(offset, iB + 1) += -1.0;  // d/d(pB.y)
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

void VerticalConstraint::evaluate(const ParameterTable& params,
                                   Eigen::VectorXd& residuals, int offset) const {
    auto pA = params.pointPosition(m_refA);
    auto pB = params.pointPosition(m_refB);
    residuals(offset) = pA.x - pB.x;
}

void VerticalConstraint::jacobian(const ParameterTable& params,
                                   Eigen::MatrixXd& jac, int offset) const {
    int iA = params.parameterIndex(m_refA);
    int iB = params.parameterIndex(m_refB);
    jac(offset, iA + 0) += 1.0;   // d/d(pA.x)
    jac(offset, iB + 0) += -1.0;  // d/d(pB.x)
}

std::shared_ptr<Constraint> VerticalConstraint::clone() const {
    return std::make_shared<VerticalConstraint>(m_refA, m_refB);
}

// ---------------------------------------------------------------------------
// PerpendicularConstraint: d1.dot(d2) == 0  (1 eq)
// d1 = lineA.end - lineA.start,  d2 = lineB.end - lineB.start
// ---------------------------------------------------------------------------

PerpendicularConstraint::PerpendicularConstraint(const GeometryRef& lineA,
                                                 const GeometryRef& lineB)
    : m_lineA(lineA), m_lineB(lineB) {}

std::vector<uint64_t> PerpendicularConstraint::referencedEntityIds() const {
    return uniqueIds(m_lineA.entityId, m_lineB.entityId);
}

void PerpendicularConstraint::evaluate(const ParameterTable& params,
                                        Eigen::VectorXd& residuals, int offset) const {
    auto [sA, eA] = params.lineEndpoints(m_lineA);
    auto [sB, eB] = params.lineEndpoints(m_lineB);
    double dx1 = eA.x - sA.x, dy1 = eA.y - sA.y;
    double dx2 = eB.x - sB.x, dy2 = eB.y - sB.y;
    residuals(offset) = dx1 * dx2 + dy1 * dy2;
}

void PerpendicularConstraint::jacobian(const ParameterTable& params,
                                        Eigen::MatrixXd& jac, int offset) const {
    auto [sA, eA] = params.lineEndpoints(m_lineA);
    auto [sB, eB] = params.lineEndpoints(m_lineB);
    double dx1 = eA.x - sA.x, dy1 = eA.y - sA.y;
    double dx2 = eB.x - sB.x, dy2 = eB.y - sB.y;

    int iA = params.parameterIndex(m_lineA);  // [sAx, sAy, eAx, eAy]
    int iB = params.parameterIndex(m_lineB);  // [sBx, sBy, eBx, eBy]

    // F = dx1*dx2 + dy1*dy2
    // dF/d(sA.x) = -dx2,  dF/d(sA.y) = -dy2
    // dF/d(eA.x) = +dx2,  dF/d(eA.y) = +dy2
    // dF/d(sB.x) = -dx1,  dF/d(sB.y) = -dy1
    // dF/d(eB.x) = +dx1,  dF/d(eB.y) = +dy1
    jac(offset, iA + 0) += -dx2;
    jac(offset, iA + 1) += -dy2;
    jac(offset, iA + 2) += dx2;
    jac(offset, iA + 3) += dy2;
    jac(offset, iB + 0) += -dx1;
    jac(offset, iB + 1) += -dy1;
    jac(offset, iB + 2) += dx1;
    jac(offset, iB + 3) += dy1;
}

std::shared_ptr<Constraint> PerpendicularConstraint::clone() const {
    return std::make_shared<PerpendicularConstraint>(m_lineA, m_lineB);
}

// ---------------------------------------------------------------------------
// ParallelConstraint: d1.cross(d2) == 0  (1 eq)
// cross = dx1*dy2 - dy1*dx2
// ---------------------------------------------------------------------------

ParallelConstraint::ParallelConstraint(const GeometryRef& lineA,
                                       const GeometryRef& lineB)
    : m_lineA(lineA), m_lineB(lineB) {}

std::vector<uint64_t> ParallelConstraint::referencedEntityIds() const {
    return uniqueIds(m_lineA.entityId, m_lineB.entityId);
}

void ParallelConstraint::evaluate(const ParameterTable& params,
                                   Eigen::VectorXd& residuals, int offset) const {
    auto [sA, eA] = params.lineEndpoints(m_lineA);
    auto [sB, eB] = params.lineEndpoints(m_lineB);
    double dx1 = eA.x - sA.x, dy1 = eA.y - sA.y;
    double dx2 = eB.x - sB.x, dy2 = eB.y - sB.y;
    residuals(offset) = dx1 * dy2 - dy1 * dx2;
}

void ParallelConstraint::jacobian(const ParameterTable& params,
                                   Eigen::MatrixXd& jac, int offset) const {
    auto [sA, eA] = params.lineEndpoints(m_lineA);
    auto [sB, eB] = params.lineEndpoints(m_lineB);
    double dx1 = eA.x - sA.x, dy1 = eA.y - sA.y;
    double dx2 = eB.x - sB.x, dy2 = eB.y - sB.y;

    int iA = params.parameterIndex(m_lineA);
    int iB = params.parameterIndex(m_lineB);

    // F = dx1*dy2 - dy1*dx2
    // dF/d(sA.x) = -dy2,  dF/d(sA.y) = +dx2
    // dF/d(eA.x) = +dy2,  dF/d(eA.y) = -dx2
    // dF/d(sB.x) = +dy1,  dF/d(sB.y) = -dx1
    // dF/d(eB.x) = -dy1,  dF/d(eB.y) = +dx1
    jac(offset, iA + 0) += -dy2;
    jac(offset, iA + 1) += dx2;
    jac(offset, iA + 2) += dy2;
    jac(offset, iA + 3) += -dx2;
    jac(offset, iB + 0) += dy1;
    jac(offset, iB + 1) += -dx1;
    jac(offset, iB + 2) += -dy1;
    jac(offset, iB + 3) += dx1;
}

std::shared_ptr<Constraint> ParallelConstraint::clone() const {
    return std::make_shared<ParallelConstraint>(m_lineA, m_lineB);
}

// ---------------------------------------------------------------------------
// TangentConstraint: signed_dist(line, center)^2 - radius^2 == 0  (1 eq)
// signed_dist = ((center - lineStart) cross d) / |d|
// F = ((cx-sx)*(ey-sy) - (cy-sy)*(ex-sx))^2 - r^2 * ((ex-sx)^2 + (ey-sy)^2)
// ---------------------------------------------------------------------------

TangentConstraint::TangentConstraint(const GeometryRef& lineRef,
                                     const GeometryRef& circleRef)
    : m_lineRef(lineRef), m_circleRef(circleRef) {}

std::vector<uint64_t> TangentConstraint::referencedEntityIds() const {
    return uniqueIds(m_lineRef.entityId, m_circleRef.entityId);
}

void TangentConstraint::evaluate(const ParameterTable& params,
                                  Eigen::VectorXd& residuals, int offset) const {
    auto [s, e] = params.lineEndpoints(m_lineRef);
    auto [center, radius] = params.circleData(m_circleRef);
    double dx = e.x - s.x, dy = e.y - s.y;
    double cross = (center.x - s.x) * dy - (center.y - s.y) * dx;
    double lenSq = dx * dx + dy * dy;
    // F = cross^2 - radius^2 * lenSq == 0
    residuals(offset) = cross * cross - radius * radius * lenSq;
}

void TangentConstraint::jacobian(const ParameterTable& params,
                                  Eigen::MatrixXd& jac, int offset) const {
    auto [s, e] = params.lineEndpoints(m_lineRef);
    auto [center, radius] = params.circleData(m_circleRef);
    double dx = e.x - s.x, dy = e.y - s.y;
    double dcx = center.x - s.x, dcy = center.y - s.y;
    double cross = dcx * dy - dcy * dx;
    double lenSq = dx * dx + dy * dy;

    int iL = params.parameterIndex(m_lineRef);   // [sx, sy, ex, ey]
    int iC = params.parameterIndex(m_circleRef);  // [cx, cy, r]

    // F = cross^2 - r^2 * lenSq
    // d(cross)/d(sx) = -dy,  d(cross)/d(sy) = dx
    // d(cross)/d(ex) = dcy,  d(cross)/d(ey) = -dcx  (wait, let me re-derive)
    // cross = (cx-sx)*dy - (cy-sy)*dx = (cx-sx)*(ey-sy) - (cy-sy)*(ex-sx)
    // d(cross)/d(sx) = -(ey-sy) + (cy-sy)*(-(-1)) = -dy + (cy-sy) ... no, let's be careful
    // cross = (cx-sx)*(ey-sy) - (cy-sy)*(ex-sx)
    // d(cross)/d(sx) = -(ey-sy) - (-(cy-sy)) = -dy + dcy = -(dy - dcy)
    // Wait: cross = dcx*dy - dcy*dx where dcx=cx-sx, dcy=cy-sy, dx=ex-sx, dy=ey-sy
    // d(cross)/d(sx): dcx depends on sx as d(dcx)/d(sx) = -1, dx depends on sx as d(dx)/d(sx) = -1
    //   = -1*dy + dcx*0 - (dcy*(-1) + 0*dx) = -dy + dcy
    // d(cross)/d(sy): d(dcy)/d(sy) = -1, d(dy)/d(sy) = -1
    //   = dcx*(-1) - (-1*dx) = -dcx + dx = dx - dcx
    // d(cross)/d(ex): d(dx)/d(ex) = 1
    //   = 0 - dcy*1 = -dcy
    // d(cross)/d(ey): d(dy)/d(ey) = 1
    //   = dcx*1 - 0 = dcx
    // d(cross)/d(cx): d(dcx)/d(cx) = 1
    //   = 1*dy = dy
    // d(cross)/d(cy): d(dcy)/d(cy) = 1
    //   = -1*dx = -dx

    double dc_dsx = -dy + dcy;
    double dc_dsy = dx - dcx;
    double dc_dex = -dcy;
    double dc_dey = dcx;
    double dc_dcx = dy;
    double dc_dcy = -dx;

    // d(lenSq)/d(sx) = -2*dx, d(lenSq)/d(sy) = -2*dy
    // d(lenSq)/d(ex) = 2*dx,  d(lenSq)/d(ey) = 2*dy
    double dl_dsx = -2.0 * dx, dl_dsy = -2.0 * dy;
    double dl_dex = 2.0 * dx, dl_dey = 2.0 * dy;

    // dF/d(var) = 2*cross*d(cross)/d(var) - r^2*d(lenSq)/d(var)
    double r2 = radius * radius;
    jac(offset, iL + 0) += 2.0 * cross * dc_dsx - r2 * dl_dsx;
    jac(offset, iL + 1) += 2.0 * cross * dc_dsy - r2 * dl_dsy;
    jac(offset, iL + 2) += 2.0 * cross * dc_dex - r2 * dl_dex;
    jac(offset, iL + 3) += 2.0 * cross * dc_dey - r2 * dl_dey;

    // dF/d(cx) = 2*cross*dy, dF/d(cy) = 2*cross*(-dx)
    jac(offset, iC + 0) += 2.0 * cross * dc_dcx;
    jac(offset, iC + 1) += 2.0 * cross * dc_dcy;
    // dF/d(r) = -2*r*lenSq
    jac(offset, iC + 2) += -2.0 * radius * lenSq;
}

std::shared_ptr<Constraint> TangentConstraint::clone() const {
    return std::make_shared<TangentConstraint>(m_lineRef, m_circleRef);
}

// ---------------------------------------------------------------------------
// EqualConstraint: equal length (lines) or equal radius (circles)  (1 eq)
// Lines:   |d1|^2 - |d2|^2 == 0
// Circles: r1 - r2 == 0
// ---------------------------------------------------------------------------

EqualConstraint::EqualConstraint(const GeometryRef& refA, const GeometryRef& refB)
    : m_refA(refA), m_refB(refB) {}

std::vector<uint64_t> EqualConstraint::referencedEntityIds() const {
    return uniqueIds(m_refA.entityId, m_refB.entityId);
}

void EqualConstraint::evaluate(const ParameterTable& params,
                                Eigen::VectorXd& residuals, int offset) const {
    if (m_refA.featureType == FeatureType::Line) {
        auto [sA, eA] = params.lineEndpoints(m_refA);
        auto [sB, eB] = params.lineEndpoints(m_refB);
        double lenSqA = (eA.x - sA.x) * (eA.x - sA.x) + (eA.y - sA.y) * (eA.y - sA.y);
        double lenSqB = (eB.x - sB.x) * (eB.x - sB.x) + (eB.y - sB.y) * (eB.y - sB.y);
        residuals(offset) = lenSqA - lenSqB;
    } else {
        auto [cA, rA] = params.circleData(m_refA);
        auto [cB, rB] = params.circleData(m_refB);
        residuals(offset) = rA - rB;
    }
}

void EqualConstraint::jacobian(const ParameterTable& params,
                                Eigen::MatrixXd& jac, int offset) const {
    if (m_refA.featureType == FeatureType::Line) {
        auto [sA, eA] = params.lineEndpoints(m_refA);
        auto [sB, eB] = params.lineEndpoints(m_refB);
        int iA = params.parameterIndex(m_refA);
        int iB = params.parameterIndex(m_refB);
        double dxA = eA.x - sA.x, dyA = eA.y - sA.y;
        double dxB = eB.x - sB.x, dyB = eB.y - sB.y;
        // F = lenSqA - lenSqB
        jac(offset, iA + 0) += -2.0 * dxA;
        jac(offset, iA + 1) += -2.0 * dyA;
        jac(offset, iA + 2) += 2.0 * dxA;
        jac(offset, iA + 3) += 2.0 * dyA;
        jac(offset, iB + 0) += 2.0 * dxB;
        jac(offset, iB + 1) += 2.0 * dyB;
        jac(offset, iB + 2) += -2.0 * dxB;
        jac(offset, iB + 3) += -2.0 * dyB;
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

void FixedConstraint::evaluate(const ParameterTable& params,
                                Eigen::VectorXd& residuals, int offset) const {
    auto p = params.pointPosition(m_pointRef);
    residuals(offset + 0) = p.x - m_position.x;
    residuals(offset + 1) = p.y - m_position.y;
}

void FixedConstraint::jacobian(const ParameterTable& params,
                                Eigen::MatrixXd& jac, int offset) const {
    int idx = params.parameterIndex(m_pointRef);
    jac(offset + 0, idx + 0) += 1.0;
    jac(offset + 1, idx + 1) += 1.0;
}

std::shared_ptr<Constraint> FixedConstraint::clone() const {
    return std::make_shared<FixedConstraint>(m_pointRef, m_position);
}

// ---------------------------------------------------------------------------
// DistanceConstraint: dist(A,B)^2 - value^2 == 0  (1 eq)
// Using squared form to avoid sqrt singularity at zero distance.
// ---------------------------------------------------------------------------

DistanceConstraint::DistanceConstraint(const GeometryRef& refA, const GeometryRef& refB,
                                       double distance)
    : m_refA(refA), m_refB(refB), m_distance(distance) {}

std::vector<uint64_t> DistanceConstraint::referencedEntityIds() const {
    return uniqueIds(m_refA.entityId, m_refB.entityId);
}

void DistanceConstraint::evaluate(const ParameterTable& params,
                                   Eigen::VectorXd& residuals, int offset) const {
    auto pA = params.pointPosition(m_refA);
    auto pB = params.pointPosition(m_refB);
    double dx = pA.x - pB.x, dy = pA.y - pB.y;
    residuals(offset) = dx * dx + dy * dy - m_distance * m_distance;
}

void DistanceConstraint::jacobian(const ParameterTable& params,
                                   Eigen::MatrixXd& jac, int offset) const {
    auto pA = params.pointPosition(m_refA);
    auto pB = params.pointPosition(m_refB);
    double dx = pA.x - pB.x, dy = pA.y - pB.y;
    int iA = params.parameterIndex(m_refA);
    int iB = params.parameterIndex(m_refB);
    // F = dx^2 + dy^2 - d^2
    jac(offset, iA + 0) += 2.0 * dx;
    jac(offset, iA + 1) += 2.0 * dy;
    jac(offset, iB + 0) += -2.0 * dx;
    jac(offset, iB + 1) += -2.0 * dy;
}

std::shared_ptr<Constraint> DistanceConstraint::clone() const {
    return std::make_shared<DistanceConstraint>(m_refA, m_refB, m_distance);
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

void AngleConstraint::evaluate(const ParameterTable& params,
                                Eigen::VectorXd& residuals, int offset) const {
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

void AngleConstraint::jacobian(const ParameterTable& params,
                                Eigen::MatrixXd& jac, int offset) const {
    auto [sA, eA] = params.lineEndpoints(m_lineA);
    auto [sB, eB] = params.lineEndpoints(m_lineB);
    double dx1 = eA.x - sA.x, dy1 = eA.y - sA.y;
    double dx2 = eB.x - sB.x, dy2 = eB.y - sB.y;
    double dot = dx1 * dx2 + dy1 * dy2;
    double cross = dx1 * dy2 - dy1 * dx2;
    double denom = dot * dot + cross * cross;
    if (denom < 1e-30) return;  // Degenerate

    int iA = params.parameterIndex(m_lineA);
    int iB = params.parameterIndex(m_lineB);

    // theta = atan2(cross, dot)
    // d(theta)/d(var) = (dot * d(cross)/d(var) - cross * d(dot)/d(var)) / (dot^2 + cross^2)
    // d(dot)/d(sA.x) = -dx2,  d(dot)/d(sA.y) = -dy2
    // d(dot)/d(eA.x) = +dx2,  d(dot)/d(eA.y) = +dy2
    // d(dot)/d(sB.x) = -dx1,  d(dot)/d(sB.y) = -dy1
    // d(dot)/d(eB.x) = +dx1,  d(dot)/d(eB.y) = +dy1
    // d(cross)/d(sA.x) = -dy2, d(cross)/d(sA.y) = +dx2
    // d(cross)/d(eA.x) = +dy2, d(cross)/d(eA.y) = -dx2
    // d(cross)/d(sB.x) = +dy1, d(cross)/d(sB.y) = -dx1
    // d(cross)/d(eB.x) = -dy1, d(cross)/d(eB.y) = +dx1

    auto addJac = [&](int col, double dDot, double dCross) {
        jac(offset, col) += (dot * dCross - cross * dDot) / denom;
    };

    addJac(iA + 0, -dx2, -dy2);  // sA.x
    addJac(iA + 1, -dy2, dx2);   // sA.y
    addJac(iA + 2, dx2, dy2);    // eA.x
    addJac(iA + 3, dy2, -dx2);   // eA.y
    addJac(iB + 0, -dx1, dy1);   // sB.x
    addJac(iB + 1, -dy1, -dx1);  // sB.y
    addJac(iB + 2, dx1, -dy1);   // eB.x
    addJac(iB + 3, dy1, dx1);    // eB.y
}

std::shared_ptr<Constraint> AngleConstraint::clone() const {
    return std::make_shared<AngleConstraint>(m_lineA, m_lineB, m_angle);
}

}  // namespace hz::cstr
