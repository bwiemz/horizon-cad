#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <vector>

#include "horizon/modeling/DrawingDimension.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using hz::model::AngularDimension;
using hz::model::DrawingDimensioner;
using hz::model::LinearDimension;
using hz::model::PrimitiveFactory;
using hz::topo::TopologyID;

namespace {
constexpr double kPi = 3.14159265358979323846;
}

namespace {
// Count edge lengths (rounded) across a solid, measured via the dimensioner.
std::map<int, int> edgeLengthHistogram(const hz::topo::Solid& solid) {
    std::map<int, int> hist;
    for (const auto& e : solid.edges()) {
        double len = 0.0;
        if (DrawingDimensioner::measureEdge(solid, e.topoId, len)) {
            hist[static_cast<int>(std::lround(len))]++;
        }
    }
    return hist;
}
}  // namespace

// A box has four edges along each axis, so its edge lengths measure to the box
// dimensions with the right multiplicities.
TEST(DrawingDimensionTest, MeasuresBoxEdgeLengths) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    auto hist = edgeLengthHistogram(*box);
    EXPECT_EQ(hist[4], 4);  // four edges of length 4 (X)
    EXPECT_EQ(hist[3], 4);  // four edges of length 3 (Y)
    EXPECT_EQ(hist[2], 4);  // four edges of length 2 (Z)
}

TEST(DrawingDimensionTest, DimensionEdgePopulatesValue) {
    auto box = PrimitiveFactory::makeBox(5.0, 5.0, 5.0);
    ASSERT_FALSE(box->edges().empty());
    const TopologyID id = box->edges().front().topoId;

    LinearDimension dim;
    ASSERT_TRUE(DrawingDimensioner::dimensionEdge(*box, id, dim));
    EXPECT_TRUE(dim.edge == id);
    EXPECT_NEAR(dim.value, 5.0, 1e-9);  // every edge of a 5-cube is length 5
}

TEST(DrawingDimensionTest, UnknownEdgeIdIsNotFound) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    double len = -1.0;
    EXPECT_FALSE(DrawingDimensioner::measureEdge(*box, TopologyID::make("box", "edge999"), len));
    EXPECT_DOUBLE_EQ(len, -1.0);                                             // left untouched
    EXPECT_FALSE(DrawingDimensioner::measureEdge(*box, TopologyID(), len));  // invalid id
}

// Model-driven behavior: because dimensions anchor to a TopologyID (stable across
// rebuilds), the SAME anchors measured against a differently-sized model report
// the updated lengths — the dimension follows the model.
TEST(DrawingDimensionTest, DimensionFollowsModelChange) {
    auto small = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    auto wide = PrimitiveFactory::makeBox(8.0, 3.0, 2.0);  // X doubled; same anchors

    // Anchor each dimension to a box1 edge, then re-measure it against box2.
    std::map<int, int> updated;
    int reanchored = 0;
    for (const auto& e : small->edges()) {
        double len = 0.0;
        if (DrawingDimensioner::measureEdge(*wide, e.topoId, len)) {
            updated[static_cast<int>(std::lround(len))]++;
            ++reanchored;
        }
    }
    EXPECT_EQ(reanchored, static_cast<int>(small->edges().size()));  // all anchors persist
    EXPECT_EQ(updated[8], 4);  // the four X edges now measure 8, not 4
    EXPECT_EQ(updated[3], 4);
    EXPECT_EQ(updated[2], 4);
}

// Box edges are axis-aligned, so the angle between any two of them is 0 or 90
// degrees; both occur among the edges sharing a reference edge.
TEST(DrawingDimensionTest, MeasuresBoxEdgeAngles) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    ASSERT_GE(box->edges().size(), 12u);
    const TopologyID ref = box->edges().front().topoId;

    std::map<int, int> angles;  // rounded degrees -> count
    for (const auto& e : box->edges()) {
        if (e.topoId == ref) continue;
        double rad = 0.0;
        if (DrawingDimensioner::measureAngle(*box, ref, e.topoId, rad)) {
            angles[static_cast<int>(std::lround(rad * 180.0 / kPi))]++;
        }
    }
    for (const auto& [deg, count] : angles) {
        EXPECT_TRUE(deg == 0 || deg == 90) << "unexpected angle " << deg;
    }
    EXPECT_GT(angles[0], 0);   // parallel edges
    EXPECT_GT(angles[90], 0);  // perpendicular edges
}

TEST(DrawingDimensionTest, DimensionAnglePopulatesRightAngle) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    const TopologyID ref = box->edges().front().topoId;

    // Find a perpendicular partner edge.
    TopologyID perp;
    for (const auto& e : box->edges()) {
        if (e.topoId == ref) continue;
        double rad = 0.0;
        if (DrawingDimensioner::measureAngle(*box, ref, e.topoId, rad) &&
            std::abs(rad - kPi / 2.0) < 1e-6) {
            perp = e.topoId;
            break;
        }
    }
    ASSERT_TRUE(perp.isValid());

    AngularDimension dim;
    ASSERT_TRUE(DrawingDimensioner::dimensionAngle(*box, ref, perp, dim));
    EXPECT_TRUE(dim.edgeA == ref);
    EXPECT_TRUE(dim.edgeB == perp);
    EXPECT_NEAR(dim.value, kPi / 2.0, 1e-6);
}

TEST(DrawingDimensionTest, AngleUnknownEdgeIsNotFound) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    const TopologyID ref = box->edges().front().topoId;
    double rad = -1.0;
    EXPECT_FALSE(
        DrawingDimensioner::measureAngle(*box, ref, TopologyID::make("box", "edge999"), rad));
    EXPECT_DOUBLE_EQ(rad, -1.0);  // untouched
}
