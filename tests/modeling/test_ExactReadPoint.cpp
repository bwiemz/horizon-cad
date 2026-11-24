#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "horizon/geometry/Frame.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Mat4.h"
#include "horizon/modeling/BoundaryMesh.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

using namespace hz;
using model::BoundaryMesh;
using model::BoundaryPolygon;
using model::MassPropertiesCalculator;
namespace exact = model::exact;

namespace {

std::unique_ptr<topo::Solid> exactCylinder() {
    return exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
}

double area(const std::vector<BoundaryPolygon>& polys) {
    double total = 0.0;
    for (const auto& p : polys) {
        for (std::size_t i = 1; i + 1 < p.points.size(); ++i) {
            total +=
                0.5 * (p.points[i] - p.points[0]).cross(p.points[i + 1] - p.points[0]).length();
        }
    }
    return total;
}

/// An exact cylinder with one face's boundary tampered so the mesher cannot lay the face out:
/// the first half-edge of the first face with a pcurve loses its pcurve. Returns that face's tag.
std::string breakOneFace(topo::Solid& solid) {
    for (auto& face : solid.faces()) {
        if (!face.alive || !face.exactSurface || !face.outerLoop || !face.outerLoop->halfEdge) {
            continue;
        }
        face.outerLoop->halfEdge->pcurve = nullptr;
        return face.topoId.tag();
    }
    return {};
}

}  // namespace

TEST(ExactReadPoint, MassPropertiesOfExactSolidsAreTheirShapes) {
    const auto cyl = exactCylinder();
    EXPECT_NEAR(MassPropertiesCalculator::compute(*cyl).volume, math::kPi * 25 * 10,
                math::kPi * 250 * 1e-3);
    const auto plate = exact::makePlateWithHole({0, 0, 0}, {40, 40, 10}, {20, 20}, 5.0);
    const double v = 40.0 * 40 * 10 - math::kPi * 25 * 10;
    EXPECT_NEAR(MassPropertiesCalculator::compute(*plate).volume, v, v * 1e-3);
}

TEST(ExactReadPoint, EveryPolygonNamesItsFace) {
    const auto cyl = exactCylinder();
    std::set<std::string> names;
    for (const auto& p : BoundaryMesh::extractFacePolygons(*cyl)) {
        EXPECT_EQ(p.points.size(), 3u);
        EXPECT_NE(p.exactSurface, nullptr);
        names.insert(p.topoId.tag());
    }
    EXPECT_EQ(names.size(), 3u);  // side, top, bottom
}

TEST(ExactReadPoint, FacetedSolidsReadAsBefore) {
    const auto faceted = model::PrimitiveFactory::makeCylinder(5.0, 10.0);  // today's, 32 facets
    const auto polys = BoundaryMesh::extractFacePolygons(*faceted);
    EXPECT_EQ(polys.size(), faceted->faceCount());
    for (const auto& p : polys) EXPECT_EQ(p.exactSurface, nullptr);
}

TEST(ExactReadPoint, CylinderSurfaceAreaFromThePolygons) {
    const auto cyl = exactCylinder();
    const double expected = 2 * math::kPi * 5 * 10 + 2 * math::kPi * 25;
    const auto polys = BoundaryMesh::extractFacePolygons(*cyl);
    EXPECT_NEAR(area(polys), expected, expected * 1e-3);
    EXPECT_GT(BoundaryMesh::signedVolume(polys), 0.0);  // wound outward
}

TEST(ExactReadPoint, AMirroredExactCylinderStillMeasuresPositive) {
    const auto cyl = exactCylinder();
    const double expected = math::kPi * 25 * 10;
    for (const math::Mat4& m :
         {math::Mat4::scale({-1, 1, 1}), math::Mat4::reflection({0, 0, 0}, {0, 1, 0})}) {
        const auto mirrored = model::Pattern::transformed(*cyl, m);
        ASSERT_NE(mirrored, nullptr);
        const auto polys = BoundaryMesh::extractFacePolygons(*mirrored);
        EXPECT_NEAR(BoundaryMesh::signedVolume(polys), expected, expected * 1e-3);
        EXPECT_NEAR(MassPropertiesCalculator::compute(*mirrored).volume, expected, expected * 1e-3);
    }
}

TEST(ExactReadPoint, AFaceTheMesherCannotMeshThrowsWithoutAProblemPointer) {
    auto cyl = exactCylinder();
    const std::string tag = breakOneFace(*cyl);
    ASSERT_FALSE(tag.empty());
    try {
        (void)BoundaryMesh::extractFacePolygons(*cyl);
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find(tag), std::string::npos) << e.what();
    }
    EXPECT_THROW((void)BoundaryMesh::extractFacePolygons(*cyl, 1e-3), std::runtime_error);
}

TEST(ExactReadPoint, AProblemPointerReceivesTheReasonAndTheRestIsStillRead) {
    auto cyl = exactCylinder();
    const std::string tag = breakOneFace(*cyl);
    ASSERT_FALSE(tag.empty());
    std::string problem;
    const auto polys = BoundaryMesh::extractFacePolygons(*cyl, 1e-3, &problem);
    EXPECT_NE(problem.find(tag), std::string::npos) << problem;
    EXPECT_GT(problem.size(), tag.size());
    EXPECT_FALSE(polys.empty());
    std::set<std::string> names;
    for (const auto& p : polys) names.insert(p.topoId.tag());
    EXPECT_EQ(names.count(tag), 0u);  // the failed face is absent, never half-meshed
    EXPECT_EQ(names.size(), 2u);

    std::string none = "untouched";
    const auto fine = BoundaryMesh::extractFacePolygons(*exactCylinder(), 1e-3, &none);
    EXPECT_TRUE(none.empty());
    EXPECT_FALSE(fine.empty());
}
