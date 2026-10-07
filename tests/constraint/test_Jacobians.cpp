// Each constraint's Jacobian against its own residuals, by central
// differences: every row, every column of the table. The solver steps by the
// Jacobian and the analysis of what is free counts its rank, so a partial
// derivative missing or in the wrong column is a solve that fails, or a
// sketch shown over-constrained, where the geometry is fine.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/constraint/ParameterTable.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/math/Vec2.h"

using namespace hz;
using cstr::FeatureType;
using math::Vec2;

namespace {

cstr::GeometryRef point(const draft::DraftEntity& e, int index) {
    return {e.id(), FeatureType::Point, index};
}
cstr::GeometryRef edge(const draft::DraftEntity& e, int index = 0) {
    return {e.id(), FeatureType::Line, index};
}
cstr::GeometryRef rim(const draft::DraftEntity& e) {
    return {e.id(), FeatureType::Circle, 0};
}

/// @p c's Jacobian agrees with the central differences of its residuals in
/// every column of @p params, to a part in a million of the larger.
void expectJacobianOfResiduals(const cstr::Constraint& c, cstr::ParameterTable params,
                               const std::string& what) {
    const int rows = c.equationCount();
    const int cols = params.parameterCount();
    Eigen::MatrixXd analytic = Eigen::MatrixXd::Zero(rows, cols);
    c.jacobian(params, analytic, 0);
    for (int j = 0; j < cols; ++j) {
        const double x = params.values()(j);
        const double h = 1e-6 * std::max(1.0, std::abs(x));
        Eigen::VectorXd plus = Eigen::VectorXd::Zero(rows);
        Eigen::VectorXd minus = Eigen::VectorXd::Zero(rows);
        params.values()(j) = x + h;
        c.evaluate(params, plus, 0);
        params.values()(j) = x - h;
        c.evaluate(params, minus, 0);
        params.values()(j) = x;
        for (int i = 0; i < rows; ++i) {
            const double numeric = (plus(i) - minus(i)) / (2.0 * h);
            const double scale = std::max({1.0, std::abs(numeric), std::abs(analytic(i, j))});
            EXPECT_NEAR(analytic(i, j), numeric, 1e-6 * scale)
                << what << " (" << c.typeName() << "): row " << i << ", column " << j;
        }
    }
}

}  // namespace

// Lines and circles, in no special position: what every constraint is
// differentiated on.
TEST(ConstraintJacobian, EachMatchesItsResidualsOnLinesAndCircles) {
    draft::DraftLine a(Vec2(1, 2), Vec2(7, 5));
    draft::DraftLine b(Vec2(-3, 4), Vec2(2, -6));
    draft::DraftCircle c(Vec2(4, 9), 3.0);
    draft::DraftArc arc(Vec2(2, -3), 4.0, 0.3, 2.1);
    cstr::ParameterTable params;
    for (const draft::DraftEntity* e : std::vector<const draft::DraftEntity*>{&a, &b, &c, &arc}) {
        params.registerEntity(*e);
    }

    std::vector<std::pair<std::string, std::shared_ptr<cstr::Constraint>>> all{
        {"a's end on b's start",
         std::make_shared<cstr::CoincidentConstraint>(point(a, 1), point(b, 0))},
        {"a's start level with b's end",
         std::make_shared<cstr::HorizontalConstraint>(point(a, 0), point(b, 1))},
        {"a's start above b's end",
         std::make_shared<cstr::VerticalConstraint>(point(a, 0), point(b, 1))},
        {"a's start held", std::make_shared<cstr::FixedConstraint>(point(a, 0), Vec2(0, 0))},
        {"a's start to b's end",
         std::make_shared<cstr::DistanceConstraint>(point(a, 0), point(b, 1), 7.0)},
        {"c's centre to a's end",
         std::make_shared<cstr::DistanceConstraint>(point(c, 0), point(a, 1), 2.0)},
        {"a and b", std::make_shared<cstr::PerpendicularConstraint>(edge(a), edge(b))},
        {"a and b", std::make_shared<cstr::ParallelConstraint>(edge(a), edge(b))},
        {"a and b", std::make_shared<cstr::AngleConstraint>(edge(a), edge(b), 0.7)},
        {"a and c", std::make_shared<cstr::TangentConstraint>(edge(a), rim(c))},
        {"the arc and a, on a's other side",
         std::make_shared<cstr::TangentConstraint>(rim(arc), edge(a))},
        {"b and the arc", std::make_shared<cstr::TangentConstraint>(edge(b), rim(arc))},
        {"a and b", std::make_shared<cstr::EqualConstraint>(edge(a), edge(b))},
        {"c and the arc", std::make_shared<cstr::EqualConstraint>(rim(c), rim(arc))},
    };
    for (const auto& [what, constraint] : all) expectJacobianOfResiduals(*constraint, params, what);
}

// An arc's start and end are its centre plus its radius at its start and end
// angles: they move with all four. Their rows had the centre's two columns
// alone, so a constraint on an arc's end could not be met by turning or
// sizing the arc.
TEST(ConstraintJacobian, AnArcsEndsMoveWithItsRadiusAndAngles) {
    draft::DraftArc arc(Vec2(2, -3), 4.0, 0.3, 2.1);
    draft::DraftLine a(Vec2(1, 2), Vec2(7, 5));
    cstr::ParameterTable params;
    params.registerEntity(arc);
    params.registerEntity(a);

    std::vector<std::pair<std::string, std::shared_ptr<cstr::Constraint>>> all{
        {"the arc's start on a's end",
         std::make_shared<cstr::CoincidentConstraint>(point(arc, 1), point(a, 1))},
        {"the arc's start level with its end",
         std::make_shared<cstr::HorizontalConstraint>(point(arc, 1), point(arc, 2))},
        {"the arc's end above a's start",
         std::make_shared<cstr::VerticalConstraint>(point(arc, 2), point(a, 0))},
        {"the arc's end held", std::make_shared<cstr::FixedConstraint>(point(arc, 2), Vec2(1, 1))},
        {"the arc's start to a's start",
         std::make_shared<cstr::DistanceConstraint>(point(arc, 1), point(a, 0), 5.0)},
        {"the arc's centre on a's end",
         std::make_shared<cstr::CoincidentConstraint>(point(arc, 0), point(a, 1))},
    };
    for (const auto& [what, constraint] : all) expectJacobianOfResiduals(*constraint, params, what);
}

// A rectangle is its two corners, as drawn; its four corners and edges are
// made of their x and y, whichever corner is the lower or the left. Their
// rows went to the first corner's columns.
TEST(ConstraintJacobian, ARectanglesCornersAndEdgesMoveWithTheCornersTheyAreMadeOf) {
    // Drawn from the bottom right to the top left: corner1 is the right and
    // the bottom, corner2 the left and the top.
    draft::DraftRectangle rect(Vec2(6, 1), Vec2(1, 4));
    draft::DraftLine a(Vec2(-2, 3), Vec2(9, 8));
    draft::DraftCircle c(Vec2(3, 9), 2.0);
    cstr::ParameterTable params;
    params.registerEntity(rect);
    params.registerEntity(a);
    params.registerEntity(c);

    for (int k = 0; k < 4; ++k) {
        const std::string corner = "corner " + std::to_string(k);
        expectJacobianOfResiduals(cstr::CoincidentConstraint(point(rect, k), point(a, 0)), params,
                                  corner);
        expectJacobianOfResiduals(cstr::DistanceConstraint(point(rect, k), point(a, 1), 3.0),
                                  params, corner);
        const std::string side = "edge " + std::to_string(k);
        expectJacobianOfResiduals(cstr::ParallelConstraint(edge(rect, k), edge(a)), params, side);
        expectJacobianOfResiduals(cstr::AngleConstraint(edge(rect, k), edge(a), 0.4), params, side);
        expectJacobianOfResiduals(cstr::TangentConstraint(edge(rect, k), rim(c)), params, side);
    }
}

// A closed polyline's last segment runs from its last point back to its
// first. It was read from the two parameters after the polyline's: another
// entity's, or past the end of the table.
TEST(ConstraintJacobian, AClosedPolylinesLastSegmentEndsAtItsFirstPoint) {
    draft::DraftPolyline poly({Vec2(0, 0), Vec2(5, 1), Vec2(4, 6)}, true);
    draft::DraftLine a(Vec2(-2, 3), Vec2(9, 8));
    cstr::ParameterTable params;
    params.registerEntity(poly);
    params.registerEntity(a);

    const auto [s, e] = params.lineEndpoints(edge(poly, 2));
    EXPECT_EQ(s.x, 4.0);
    EXPECT_EQ(s.y, 6.0);
    EXPECT_EQ(e.x, 0.0) << "its first point, not the line's start";
    EXPECT_EQ(e.y, 0.0);
    for (int k = 0; k < 3; ++k) {
        const std::string side = "segment " + std::to_string(k);
        expectJacobianOfResiduals(cstr::PerpendicularConstraint(edge(poly, k), edge(a)), params,
                                  side);
        expectJacobianOfResiduals(cstr::EqualConstraint(edge(poly, k), edge(a)), params, side);
        expectJacobianOfResiduals(cstr::CoincidentConstraint(point(poly, k), point(a, 1)), params,
                                  "point " + std::to_string(k));
    }
}
