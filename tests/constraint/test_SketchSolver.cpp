#define _USE_MATH_DEFINES
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/constraint/ParameterTable.h"
#include "horizon/constraint/SketchSolver.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/math/Vec2.h"

using namespace hz;

TEST(SketchSolver, NoConstraintsReturnsNoConstraints) {
    cstr::ConstraintSystem sys;
    cstr::ParameterTable params;
    cstr::SketchSolver solver;

    auto result = solver.solve(params, sys);
    EXPECT_EQ(result.status, cstr::SolveStatus::NoConstraints);
}

TEST(SketchSolver, CoincidentSolve) {
    draft::DraftDocument doc;
    // Two lines: line1 end at (10, 0), line2 start at (10.5, 0.3).
    auto line1 = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 0});
    auto line2 = std::make_shared<draft::DraftLine>(math::Vec2{10.5, 0.3}, math::Vec2{20, 0});
    doc.addEntity(line1);
    doc.addEntity(line2);

    cstr::ConstraintSystem sys;
    cstr::GeometryRef refA{line1->id(), cstr::FeatureType::Point, 1};  // line1 end
    cstr::GeometryRef refB{line2->id(), cstr::FeatureType::Point, 0};  // line2 start
    sys.addConstraint(std::make_shared<cstr::CoincidentConstraint>(refA, refB));

    // Fix line1 start and end, and line2 end so the solver only moves line2 start.
    cstr::GeometryRef fixLine1Start{line1->id(), cstr::FeatureType::Point, 0};
    cstr::GeometryRef fixLine1End{line1->id(), cstr::FeatureType::Point, 1};
    cstr::GeometryRef fixLine2End{line2->id(), cstr::FeatureType::Point, 1};
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(fixLine1Start, math::Vec2{0.0, 0.0}));
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(fixLine1End, math::Vec2{10.0, 0.0}));
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(fixLine2End, math::Vec2{20.0, 0.0}));

    auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);

    cstr::SketchSolver solver;
    auto result = solver.solve(params, sys);

    EXPECT_EQ(result.status, cstr::SolveStatus::Success);
    EXPECT_LT(result.residualNorm, 1e-8);

    params.applyToEntities(doc.entities());

    // line2's start should now be at (10, 0).
    auto* updatedLine2 = dynamic_cast<draft::DraftLine*>(doc.entities()[1].get());
    ASSERT_NE(updatedLine2, nullptr);
    EXPECT_NEAR(updatedLine2->start().x, 10.0, 1e-6);
    EXPECT_NEAR(updatedLine2->start().y, 0.0, 1e-6);
}

TEST(SketchSolver, HorizontalConstraint) {
    draft::DraftDocument doc;
    // Line from (0, 0) to (10, 2) — not horizontal.
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 2});
    doc.addEntity(line);

    cstr::ConstraintSystem sys;
    cstr::GeometryRef refA{line->id(), cstr::FeatureType::Point, 0};
    cstr::GeometryRef refB{line->id(), cstr::FeatureType::Point, 1};
    sys.addConstraint(std::make_shared<cstr::HorizontalConstraint>(refA, refB));

    // Fix the start point.
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(refA, math::Vec2{0.0, 0.0}));

    auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);

    cstr::SketchSolver solver;
    auto result = solver.solve(params, sys);

    EXPECT_TRUE(result.status == cstr::SolveStatus::Success ||
                result.status == cstr::SolveStatus::UnderConstrained);

    params.applyToEntities(doc.entities());

    auto* updated = dynamic_cast<draft::DraftLine*>(doc.entities().front().get());
    ASSERT_NE(updated, nullptr);
    // The Y coordinates should be equal (horizontal).
    EXPECT_NEAR(updated->start().y, updated->end().y, 1e-6);
}

TEST(SketchSolver, DistanceConstraint) {
    draft::DraftDocument doc;
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{8, 0});
    doc.addEntity(line);

    cstr::ConstraintSystem sys;
    cstr::GeometryRef refA{line->id(), cstr::FeatureType::Point, 0};
    cstr::GeometryRef refB{line->id(), cstr::FeatureType::Point, 1};

    // Fix start, constrain distance to 10.
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(refA, math::Vec2{0.0, 0.0}));
    sys.addConstraint(std::make_shared<cstr::DistanceConstraint>(refA, refB, 10.0));

    auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);

    cstr::SketchSolver solver;
    auto result = solver.solve(params, sys);

    EXPECT_TRUE(result.status == cstr::SolveStatus::Success ||
                result.status == cstr::SolveStatus::UnderConstrained);

    params.applyToEntities(doc.entities());

    auto* updated = dynamic_cast<draft::DraftLine*>(doc.entities().front().get());
    ASSERT_NE(updated, nullptr);

    double dx = updated->end().x - updated->start().x;
    double dy = updated->end().y - updated->start().y;
    double dist = std::sqrt(dx * dx + dy * dy);
    EXPECT_NEAR(dist, 10.0, 1e-6);
}

TEST(SketchSolver, FixedConstraint) {
    draft::DraftDocument doc;
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0.1, 0.2}, math::Vec2{10, 0});
    doc.addEntity(line);

    cstr::ConstraintSystem sys;
    cstr::GeometryRef ref{line->id(), cstr::FeatureType::Point, 0};
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(ref, math::Vec2{0.0, 0.0}));

    auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);

    cstr::SketchSolver solver;
    auto result = solver.solve(params, sys);

    EXPECT_TRUE(result.status == cstr::SolveStatus::Success ||
                result.status == cstr::SolveStatus::UnderConstrained);

    params.applyToEntities(doc.entities());

    auto* updated = dynamic_cast<draft::DraftLine*>(doc.entities().front().get());
    ASSERT_NE(updated, nullptr);
    EXPECT_NEAR(updated->start().x, 0.0, 1e-6);
    EXPECT_NEAR(updated->start().y, 0.0, 1e-6);
}

TEST(SketchSolver, OverConstrainedDetection) {
    draft::DraftDocument doc;
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 0});
    doc.addEntity(line);

    cstr::ConstraintSystem sys;
    cstr::GeometryRef refA{line->id(), cstr::FeatureType::Point, 0};
    cstr::GeometryRef refB{line->id(), cstr::FeatureType::Point, 1};

    // Fix both endpoints AND add a distance constraint — over-constrained if distance doesn't
    // match.
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(refA, math::Vec2{0.0, 0.0}));
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(refB, math::Vec2{10.0, 0.0}));
    sys.addConstraint(
        std::make_shared<cstr::DistanceConstraint>(refA, refB, 5.0));  // Contradicts fixed.

    auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);

    cstr::SketchSolver solver;
    auto result = solver.solve(params, sys);

    // Should detect inconsistency or over-constraint.
    EXPECT_TRUE(result.status == cstr::SolveStatus::OverConstrained ||
                result.status == cstr::SolveStatus::Inconsistent ||
                result.status == cstr::SolveStatus::FailedToConverge);
}

// Phase 157: an edge of the part projected into the sketch is where the part
// puts it. What is tied to it moves to it; it never moves, and is not free.
TEST(SketchSolver, AProjectedEdgeIsHeldWhereThePartPutsIt) {
    draft::DraftDocument doc;
    auto edge = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 0});
    edge->setSourceEdge("extrude_1/edge:cap_top|side:e2");
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{3, 4}, math::Vec2{6, 8});
    doc.addEntity(edge);
    doc.addEntity(line);

    cstr::ConstraintSystem sys;
    sys.addConstraint(std::make_shared<cstr::CoincidentConstraint>(
        cstr::GeometryRef{edge->id(), cstr::FeatureType::Point, 1},
        cstr::GeometryRef{line->id(), cstr::FeatureType::Point, 0}));

    auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);
    EXPECT_EQ(params.fixedCount(), 4) << "the edge's two ends";
    cstr::SketchSolver solver;
    const auto result = solver.solve(params, sys);
    ASSERT_TRUE(result.status == cstr::SolveStatus::Success ||
                result.status == cstr::SolveStatus::UnderConstrained)
        << result.message;
    params.applyToEntities(doc.entities());
    EXPECT_NEAR(edge->end().x, 10.0, 1e-9) << "held";
    EXPECT_NEAR(edge->end().y, 0.0, 1e-9);
    EXPECT_NEAR(line->start().x, 10.0, 1e-6) << "the line's end moved to the edge's";
    EXPECT_NEAR(line->start().y, 0.0, 1e-6);
    EXPECT_EQ(result.degreesOfFreedom, 2) << "the line's other end; none of the edge";

    const auto dof = solver.analyzeDOF(params, sys);
    EXPECT_EQ(dof.totalDOF, 2);
    EXPECT_EQ(dof.entityStatus.at(edge->id()), cstr::EntityDOFStatus::FullyConstrained);
    EXPECT_EQ(dof.entityStatus.at(line->id()), cstr::EntityDOFStatus::Free);
}

// An arc's ends are met by turning and sizing the arc, not only by moving its
// centre. Its start held level with its end was over-constrained (residual
// 9.09), and its end on a point with its centre held (residual 3.74): the
// Jacobian had the centre's columns alone.
TEST(SketchSolver, AnArcsEndsAreSolvedWithItsRadiusAndAngles) {
    {
        draft::DraftDocument doc;
        auto arc = std::make_shared<draft::DraftArc>(math::Vec2{0, 0}, 10.0, 0.0, 2.0);
        doc.addEntity(arc);
        cstr::ConstraintSystem sys;
        sys.addConstraint(std::make_shared<cstr::HorizontalConstraint>(
            cstr::GeometryRef{arc->id(), cstr::FeatureType::Point, 1},
            cstr::GeometryRef{arc->id(), cstr::FeatureType::Point, 2}));
        auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);
        const auto result = cstr::SketchSolver().solve(params, sys);
        ASSERT_TRUE(result.status == cstr::SolveStatus::Success ||
                    result.status == cstr::SolveStatus::UnderConstrained)
            << result.message;
        params.applyToEntities(doc.entities());
        EXPECT_NEAR(arc->startPoint().y, arc->endPoint().y, 1e-6);
    }
    {
        draft::DraftDocument doc;
        auto arc = std::make_shared<draft::DraftArc>(math::Vec2{0, 0}, 10.0, 0.0, 2.0);
        auto line = std::make_shared<draft::DraftLine>(math::Vec2{6, 6}, math::Vec2{20, 20});
        doc.addEntity(arc);
        doc.addEntity(line);
        cstr::ConstraintSystem sys;
        const cstr::GeometryRef centre{arc->id(), cstr::FeatureType::Point, 0};
        const cstr::GeometryRef end{arc->id(), cstr::FeatureType::Point, 2};
        const cstr::GeometryRef start{line->id(), cstr::FeatureType::Point, 0};
        sys.addConstraint(std::make_shared<cstr::FixedConstraint>(centre, math::Vec2{0, 0}));
        sys.addConstraint(std::make_shared<cstr::FixedConstraint>(start, math::Vec2{6, 6}));
        sys.addConstraint(std::make_shared<cstr::CoincidentConstraint>(end, start));
        auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);
        const auto result = cstr::SketchSolver().solve(params, sys);
        ASSERT_TRUE(result.status == cstr::SolveStatus::Success ||
                    result.status == cstr::SolveStatus::UnderConstrained)
            << result.message;
        params.applyToEntities(doc.entities());
        EXPECT_NEAR(arc->endPoint().x, 6.0, 1e-6);
        EXPECT_NEAR(arc->endPoint().y, 6.0, 1e-6);
        EXPECT_NEAR(arc->center().x, 0.0, 1e-9) << "held";
        EXPECT_NEAR(arc->radius(), std::sqrt(72.0), 1e-6);

        const auto dof = cstr::SketchSolver().analyzeDOF(params, sys);
        EXPECT_NE(dof.entityStatus.at(arc->id()), cstr::EntityDOFStatus::OverConstrained);
    }
}

// A tangent picked circle first is solved as one picked line first.
TEST(SketchSolver, ATangentPickedCircleFirstIsSolved) {
    draft::DraftDocument doc;
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{-10, 0}, math::Vec2{10, 0});
    auto circle = std::make_shared<draft::DraftCircle>(math::Vec2{0, 8}, 5.0);
    doc.addEntity(line);
    doc.addEntity(circle);

    cstr::ConstraintSystem sys;
    const cstr::GeometryRef start{line->id(), cstr::FeatureType::Point, 0};
    const cstr::GeometryRef end{line->id(), cstr::FeatureType::Point, 1};
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(start, math::Vec2{-10, 0}));
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(end, math::Vec2{10, 0}));
    sys.addConstraint(std::make_shared<cstr::TangentConstraint>(
        cstr::GeometryRef{circle->id(), cstr::FeatureType::Circle, 0},
        cstr::GeometryRef{line->id(), cstr::FeatureType::Line, 0}));

    auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);
    cstr::SketchSolver solver;
    const auto result = solver.solve(params, sys);
    ASSERT_TRUE(result.status == cstr::SolveStatus::Success ||
                result.status == cstr::SolveStatus::UnderConstrained)
        << result.message;
    params.applyToEntities(doc.entities());
    EXPECT_NEAR(std::abs(circle->center().y), circle->radius(), 1e-6) << "the line touches it";
}

// A constraint its entity cannot hold (a circle read as a line, a third end
// of a line: a hand-edited file) is reported, not thrown. Every solve after
// it threw, out of the move or edit that ran it, and the analysis out of
// every repaint.
TEST(SketchSolver, AConstraintItsEntityCannotHoldIsReportedNotThrown) {
    draft::DraftDocument doc;
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 1});
    auto circle = std::make_shared<draft::DraftCircle>(math::Vec2{5, 5}, 2.0);
    doc.addEntity(line);
    doc.addEntity(circle);
    const cstr::GeometryRef start{line->id(), cstr::FeatureType::Point, 0};
    const cstr::GeometryRef end{line->id(), cstr::FeatureType::Point, 1};
    const cstr::GeometryRef edge{line->id(), cstr::FeatureType::Line, 0};

    const std::vector<std::shared_ptr<cstr::Constraint>> unfit{
        std::make_shared<cstr::PerpendicularConstraint>(
            edge, cstr::GeometryRef{circle->id(), cstr::FeatureType::Line, 0}),
        std::make_shared<cstr::CoincidentConstraint>(
            start, cstr::GeometryRef{line->id(), cstr::FeatureType::Point, 2}),
    };
    for (const auto& bad : unfit) {
        cstr::ConstraintSystem sys;
        sys.addConstraint(std::make_shared<cstr::HorizontalConstraint>(start, end));
        sys.addConstraint(bad);
        auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);
        const Eigen::VectorXd before = params.values();

        cstr::SketchSolver solver;
        cstr::SolveResult result;
        ASSERT_NO_THROW(result = solver.solve(params, sys)) << bad->typeName();
        EXPECT_EQ(result.status, cstr::SolveStatus::InvalidReference) << bad->typeName();
        EXPECT_FALSE(result.message.empty());
        EXPECT_EQ(params.values(), before) << "nothing moved";

        cstr::DOFAnalysis dof;
        ASSERT_NO_THROW(dof = solver.analyzeDOF(params, sys)) << bad->typeName();
        EXPECT_EQ(dof.entityStatus.at(line->id()), cstr::EntityDOFStatus::OverConstrained)
            << bad->typeName() << ": it cannot be met";
    }
}

namespace {

bool solved(const cstr::SolveResult& result) {
    return result.status == cstr::SolveStatus::Success ||
           result.status == cstr::SolveStatus::UnderConstrained;
}

}  // namespace

// A distance of 1000 is met, whichever way the line points. Its residual was
// |d|^2 - 1000^2: met, the rounding of a million alone (1.16e-10, one unit
// in its last place) was more than the solver's 1e-10, and 6 of these 20
// failed to converge, their geometry left where it was.
TEST(SketchSolver, ADistanceOfAThousandIsMetEveryWay) {
    int failed = 0;
    for (int k = 0; k < 20; ++k) {
        const double angle = 0.1 + 0.31 * k;
        draft::DraftDocument doc;
        auto line = std::make_shared<draft::DraftLine>(
            math::Vec2{0, 0}, math::Vec2{900 * std::cos(angle), 900 * std::sin(angle)});
        doc.addEntity(line);
        cstr::ConstraintSystem sys;
        const cstr::GeometryRef start{line->id(), cstr::FeatureType::Point, 0};
        const cstr::GeometryRef end{line->id(), cstr::FeatureType::Point, 1};
        sys.addConstraint(std::make_shared<cstr::FixedConstraint>(start, math::Vec2{0, 0}));
        sys.addConstraint(std::make_shared<cstr::DistanceConstraint>(start, end, 1000.0));
        auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);
        const auto result = cstr::SketchSolver().solve(params, sys);
        if (!solved(result)) ++failed;
        EXPECT_TRUE(solved(result)) << "at " << angle << " rad: " << result.message;
        params.applyToEntities(doc.entities());
        EXPECT_NEAR(line->start().distanceTo(line->end()), 1000.0, 1e-6);
    }
    EXPECT_EQ(failed, 0);
}

// A distance of 0 brings the points together. Squared, its residual was
// |d|^2, met below 1e-10 with the points still 1e-5 apart (9.7e-6 here).
TEST(SketchSolver, ADistanceOfNothingBringsThePointsTogether) {
    draft::DraftDocument doc;
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{3, 4});
    doc.addEntity(line);
    cstr::ConstraintSystem sys;
    const cstr::GeometryRef start{line->id(), cstr::FeatureType::Point, 0};
    const cstr::GeometryRef end{line->id(), cstr::FeatureType::Point, 1};
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(start, math::Vec2{0, 0}));
    sys.addConstraint(std::make_shared<cstr::DistanceConstraint>(start, end, 0.0));
    auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);
    const auto result = cstr::SketchSolver().solve(params, sys);
    ASSERT_TRUE(solved(result)) << result.message;
    params.applyToEntities(doc.entities());
    EXPECT_LT(line->start().distanceTo(line->end()), 1e-10);
}

// A line tangent to a circle of radius 50, or 500, is met. The residual was
// cross^2 - r^2 |d|^2, a length to the fourth: about 2e10 for a line of 300
// on a circle of 500, whose rounding alone is far beyond the solver's 1e-10.
// Every one of these solves, on either circle, stopped short of the circle
// and was called inconsistent.
TEST(SketchSolver, ATangentIsMetOnLargeCircles) {
    for (const double radius : {50.0, 500.0}) {
        int failed = 0;
        for (int k = 0; k < 20; ++k) {
            const double at = 0.2 + 0.29 * k;  // where the line starts, round the circle
            const double heading = at + 1.3;   // and the way it points, not yet tangent
            draft::DraftDocument doc;
            auto circle = std::make_shared<draft::DraftCircle>(math::Vec2{0, 0}, radius);
            circle->setSourceEdge("extrude_1/edge:rim");  // held where it is
            const math::Vec2 s{1.5 * radius * std::cos(at), 1.5 * radius * std::sin(at)};
            auto line = std::make_shared<draft::DraftLine>(
                s, s + math::Vec2{300 * std::cos(heading), 300 * std::sin(heading)});
            doc.addEntity(circle);
            doc.addEntity(line);
            cstr::ConstraintSystem sys;
            const cstr::GeometryRef start{line->id(), cstr::FeatureType::Point, 0};
            sys.addConstraint(std::make_shared<cstr::FixedConstraint>(start, s));
            sys.addConstraint(std::make_shared<cstr::TangentConstraint>(
                cstr::GeometryRef{line->id(), cstr::FeatureType::Line, 0},
                cstr::GeometryRef{circle->id(), cstr::FeatureType::Circle, 0}));
            auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);
            const auto result = cstr::SketchSolver().solve(params, sys);
            if (!solved(result)) ++failed;
            EXPECT_TRUE(solved(result))
                << "r " << radius << ", at " << at << " rad: " << result.message;
            params.applyToEntities(doc.entities());
            const math::Vec2 d = line->end() - line->start();
            const double apart = std::abs((circle->center() - line->start()).cross(d)) / d.length();
            EXPECT_NEAR(apart, radius, 1e-6 * radius);
        }
        EXPECT_EQ(failed, 0) << "of 20, on radius " << radius;
    }
}

// Far from the origin, as a site plan in survey coordinates is (thousands
// of kilometres east, in metres), a chain of two lines is solved: its ends
// met, its lengths set and the lines parallel. There one unit in the last
// place of a coordinate is more than the solver's 1e-10 (9e-10 at 5e6), so
// a residual rounding leaves is allowed for: without that, 37 of these 40
// failed to converge at 5e6, all 40 at 2e7, and one at 1e6.
TEST(SketchSolver, AChainIsSolvedFarFromTheOrigin) {
    for (const double far : {1.0e5, 1.0e6, 5.0e6, 2.0e7}) {
        int failed = 0;
        for (int k = 0; k < 40; ++k) {
            const double t = 0.1 + 0.37 * k;
            const math::Vec2 o{far + 13.7 * k, 0.8 * far - 7.1 * k};
            draft::DraftDocument doc;
            auto a = std::make_shared<draft::DraftLine>(
                o, o + math::Vec2{10.3 * std::cos(t), 10.3 * std::sin(t)});
            auto b = std::make_shared<draft::DraftLine>(
                a->end() + math::Vec2{0.37, -0.21},
                a->end() + math::Vec2{6.0 * std::cos(2 * t), 6.0 * std::sin(2 * t)});
            doc.addEntity(a);
            doc.addEntity(b);
            cstr::ConstraintSystem sys;
            const cstr::GeometryRef aStart{a->id(), cstr::FeatureType::Point, 0};
            const cstr::GeometryRef aEnd{a->id(), cstr::FeatureType::Point, 1};
            const cstr::GeometryRef bStart{b->id(), cstr::FeatureType::Point, 0};
            const cstr::GeometryRef bEnd{b->id(), cstr::FeatureType::Point, 1};
            sys.addConstraint(std::make_shared<cstr::FixedConstraint>(aStart, a->start()));
            sys.addConstraint(std::make_shared<cstr::CoincidentConstraint>(aEnd, bStart));
            sys.addConstraint(std::make_shared<cstr::DistanceConstraint>(aStart, aEnd, 12.5));
            sys.addConstraint(std::make_shared<cstr::DistanceConstraint>(bStart, bEnd, 4.5));
            sys.addConstraint(std::make_shared<cstr::ParallelConstraint>(
                cstr::GeometryRef{a->id(), cstr::FeatureType::Line, 0},
                cstr::GeometryRef{b->id(), cstr::FeatureType::Line, 0}));
            auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);
            const auto result = cstr::SketchSolver().solve(params, sys);
            if (!solved(result)) ++failed;
            EXPECT_TRUE(solved(result)) << far << ", " << k << ": " << result.message;
            params.applyToEntities(doc.entities());
            // Met to what rounding leaves there, a few units in the last place.
            EXPECT_LT(a->end().distanceTo(b->start()), 1e-7);
            EXPECT_NEAR(a->start().distanceTo(a->end()), 12.5, 1e-7);
            EXPECT_NEAR(b->start().distanceTo(b->end()), 4.5, 1e-7);
        }
        EXPECT_EQ(failed, 0) << "of 40, at " << far;
    }
}
