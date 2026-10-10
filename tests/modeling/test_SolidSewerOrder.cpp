// Welding is the first thing the sewer does to the positions a Boolean hands
// it, and it used to depend on the order those positions arrived in. A
// Boolean's faces come out of its split in whatever order the split produced
// them, so the same two solids could sew to different vertex counts, and a
// solid's face could then say it had more vertices than the solid did.
//
// The test sews one real Boolean result twice — once as listed, once in
// reverse — and asks for the same solid both times. That is the property that
// matters: not that any particular count is right, but that the count does not
// depend on something outside the solid.
#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/SolidSewer.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

using namespace hz::math;
using namespace hz::model;
using namespace hz::topo;

namespace {

std::vector<Vec3> loopPoints(const Wire* wire) {
    std::vector<Vec3> points;
    if (wire == nullptr || wire->halfEdge == nullptr) {
        return points;
    }
    const HalfEdge* const start = wire->halfEdge;
    const HalfEdge* cur = start;
    do {
        points.push_back(cur->origin->point);
        cur = cur->next;
    } while (cur != nullptr && cur != start);
    return points;
}

std::vector<SolidSewer::InputFace> asSewerInput(const Solid& solid, bool reverse) {
    std::vector<SolidSewer::InputFace> input;
    for (const auto& face : solid.faces()) {
        SolidSewer::InputFace in;
        in.topoId = face.topoId;
        in.surface = face.surface;
        in.analyticSurface = face.analyticSurface;
        in.points = loopPoints(face.outerLoop);
        for (const Wire* hole : face.innerLoops) {
            in.holes.push_back(loopPoints(hole));
        }
        input.push_back(std::move(in));
    }
    if (reverse) {
        std::reverse(input.begin(), input.end());
    }
    return input;
}

}  // namespace

// A box with a cylinder cut through it. The cut splits faces at points that
// are near, but not equal to, one another — which is where welding decides
// whether two splits are one vertex or two, and so where an order-dependent
// weld shows. Swept over the cylinders a part is actually made of, since it
// is the one whose split points land awkwardly that shows it.
TEST(SolidSewerTest, SewingTheSameSolidGivesTheSameResultWhicheverWayRoundItComes) {
    for (const int segments : {7, 11, 13, 17, 19, 23, 29}) {
        for (const double radius : {3.3, 5.0, 6.1, 7.5}) {
            SCOPED_TRACE("segments " + std::to_string(segments) + ", radius " +
                         std::to_string(radius));

            const auto box = PrimitiveFactory::makeBox(20.0, 20.0, 20.0);
            const auto cylinder = PrimitiveFactory::makeCylinder(radius, 40.0, segments);
            ASSERT_NE(box, nullptr);
            ASSERT_NE(cylinder, nullptr);

            const auto cut = BooleanOp::execute(*box, *cylinder, BooleanType::Subtract);
            if (cut == nullptr) {
                continue;  // this combination is not a solid; nothing to sew
            }

            const auto forward = SolidSewer::sew(asSewerInput(*cut, false));
            const auto reversed = SolidSewer::sew(asSewerInput(*cut, true));
            ASSERT_NE(forward, nullptr);
            ASSERT_NE(reversed, nullptr);

            EXPECT_EQ(forward->vertexCount(), reversed->vertexCount())
                << "the vertex count depends on the order the faces arrived in";
            EXPECT_EQ(forward->edgeCount(), reversed->edgeCount());
            EXPECT_EQ(forward->faceCount(), reversed->faceCount());
            EXPECT_EQ(forward->vertexCount(), cut->vertexCount())
                << "and on a solid that already sewed the way it came";
        }
    }
}

// Three points each within the weld tolerance of the next, and the outer two
// further apart than it. Which of them is one vertex used to depend entirely
// on which arrived first: the first pair welded and the third stood alone, or
// the first stood alone and the other two welded. They are all within the
// tolerance of something, so they are one vertex whichever order they come
// in.
TEST(SolidSewerTest, ThreeNearCoincidentPointsAreOneVertexWhicheverOrderTheyComeIn) {
    const double tol = SolidSewer::kDefaultWeldTol;
    const Vec3 first(0.0, 0.0, 0.0);
    const Vec3 second(tol * 0.9, 0.0, 0.0);
    const Vec3 third(tol * 1.8, 0.0, 0.0);

    const auto sewn = [&](const std::vector<Vec3>& corners) {
        SolidSewer::InputFace face;
        for (const Vec3& c : corners) {
            face.points.insert(face.points.end(),
                               {c, c + Vec3(10.0, 0.0, 0.0), c + Vec3(0.0, 10.0, 0.0)});
        }
        const auto solid = SolidSewer::sew({face});
        EXPECT_NE(solid, nullptr);
        return solid ? solid->vertexCount() : 0u;
    };

    const size_t forward = sewn({first, second, third});
    const size_t reversed = sewn({third, second, first});
    const size_t middleFirst = sewn({second, third, first});

    EXPECT_EQ(forward, reversed);
    EXPECT_EQ(forward, middleFirst);
}