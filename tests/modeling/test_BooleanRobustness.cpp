// Booleans held to strict terms (Phase 142): every case either gives a valid
// solid that conserves volume, or refuses with a reason; never a null
// without one. They were tested at thresholds (5 of 20 random transforms
// must work), and never far from the origin, at very small or large
// scales, or with faces in exact contact.

#include <gtest/gtest.h>

#if defined(__unix__) || defined(__APPLE__)
#include <pthread.h>
#endif

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <numbers>
#include <random>
#include <string>
#include <vector>

#include "../PortableRandom.h"
#include "../TimeLimits.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Quaternion.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/InterferenceChecker.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/modeling/SolidSewer.h"
#include "horizon/topology/GeometryValidator.h"
#include "horizon/topology/Solid.h"

using hz::math::Mat4;
using hz::math::Vec3;
using hz::model::BooleanOp;
using hz::model::BooleanType;
using hz::model::MassPropertiesCalculator;
using hz::model::NamingScheme;
using hz::model::Pattern;
using hz::model::PrimitiveFactory;

#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define HZ_SANITIZED 1
#elif defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer)
#define HZ_SANITIZED 1
#endif
#endif
#ifndef HZ_SANITIZED
#define HZ_SANITIZED 0
#endif

namespace {

double volumeOf(const hz::topo::Solid& solid) {
    return MassPropertiesCalculator::compute(solid).volume;
}

/// A and B combined three ways: each result valid, or refused with a reason;
/// the three together conserving volume. @p what says which case.
void expectSound(const hz::topo::Solid& a, const hz::topo::Solid& b, double scale,
                 const std::string& what) {
    double volume[3] = {0.0, 0.0, 0.0};
    const BooleanType types[3] = {BooleanType::Union, BooleanType::Subtract,
                                  BooleanType::Intersect};
    for (int k = 0; k < 3; ++k) {
        std::string reason;
        const auto result = BooleanOp::execute(a, b, types[k], &reason, NamingScheme::Stable);
        if (!result) {
            EXPECT_FALSE(reason.empty()) << what << ": a null with no reason, type " << k;
            // Only an empty result may be refused: an intersection that is
            // not there, or a cut that removes everything.
            EXPECT_TRUE(k != 0) << what << ": a union refused: " << reason;
            continue;
        }
        EXPECT_TRUE(result->isValid())
            << what << ", type " << k << ": " << result->validationReport();
        EXPECT_TRUE(result->checkManifold()) << what << ", type " << k;
        EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*result))
            << what << ", type " << k << ": " << hz::topo::GeometryValidator::report(*result);
        volume[k] = volumeOf(*result);
    }
    // A = (A - B) + (A n B); A u B = A + B - (A n B).
    const double va = volumeOf(a);
    const double vb = volumeOf(b);
    const double tol = 1e-7 * std::pow(scale, 3.0);
    EXPECT_NEAR(volume[1] + volume[2], va, tol) << what << ": A in two";
    EXPECT_NEAR(volume[0], va + vb - volume[2], tol) << what << ": the union";
}

/// @p solid moved by @p m.
std::unique_ptr<hz::topo::Solid> moved(const hz::topo::Solid& solid, const Mat4& m) {
    return Pattern::transformed(solid, m);
}

/// A rigid motion from @p rng: a turn about a random axis, then a shift.
Mat4 randomMotion(std::mt19937& rng, double reach) {
    hz::test::Uniform unit(-1.0, 1.0);
    hz::test::Uniform angle(0.0, 2.0 * std::numbers::pi);
    Vec3 axis(unit(rng), unit(rng), unit(rng));
    if (axis.length() < 1e-3) axis = Vec3(0, 0, 1);
    const auto turn = hz::math::Quaternion::fromAxisAngle(axis.normalized(), angle(rng));
    const Vec3 shift(reach * unit(rng), reach * unit(rng), reach * unit(rng));
    return Mat4::translation(shift) * Mat4::rotation(turn);
}

}  // namespace

// Every random placement of a cylinder against a box: sound, not 5 of 20.
TEST(BooleanRobustnessTest, EveryRandomPlacementIsSound) {
    std::mt19937 rng(20260925);
    const auto box = PrimitiveFactory::makeBox(10, 10, 10);
    const auto cylinder = PrimitiveFactory::makeCylinder(3.0, 12.0, 24);
    for (int i = 0; i < 20; ++i) {
        const Mat4 m = Mat4::translation(Vec3(5, 5, 5)) * randomMotion(rng, 4.0);
        expectSound(*box, *moved(*cylinder, m), 10.0, "placement " + std::to_string(i));
    }
}

// Far from the origin: the same at a million millimetres out.
TEST(BooleanRobustnessTest, FarFromTheOriginIsSound) {
    std::mt19937 rng(7);
    const Mat4 far = Mat4::translation(Vec3(1e6, -2e6, 1.5e6));
    const auto box = moved(*PrimitiveFactory::makeBox(10, 10, 10), far);
    const auto cylinder = PrimitiveFactory::makeCylinder(3.0, 12.0, 24);
    for (int i = 0; i < 10; ++i) {
        const Mat4 m = far * Mat4::translation(Vec3(5, 5, 5)) * randomMotion(rng, 4.0);
        expectSound(*box, *moved(*cylinder, m), 10.0, "far placement " + std::to_string(i));
    }
}

// Very small and very large parts: a thousandth and a hundred thousand
// times the size.
TEST(BooleanRobustnessTest, SmallAndLargeScalesAreSound) {
    for (const double scale : {1e-3, 1e5}) {
        std::mt19937 rng(11);
        const Mat4 size = Mat4::scale(scale);
        const auto box = moved(*PrimitiveFactory::makeBox(10, 10, 10), size);
        const auto cylinder = PrimitiveFactory::makeCylinder(3.0, 12.0, 24);
        for (int i = 0; i < 6; ++i) {
            const Mat4 m = size * Mat4::translation(Vec3(5, 5, 5)) * randomMotion(rng, 4.0);
            expectSound(*box, *moved(*cylinder, m), 10.0 * scale,
                        "scale " + std::to_string(scale) + ", placement " + std::to_string(i));
        }
    }
}

/// Whether A and B combine soundly three ways, as expectSound checks, but
/// counted rather than failed.
bool combinesSoundly(const hz::topo::Solid& a, const hz::topo::Solid& b, double scale) {
    double volume[3] = {0.0, 0.0, 0.0};
    const BooleanType types[3] = {BooleanType::Union, BooleanType::Subtract,
                                  BooleanType::Intersect};
    for (int k = 0; k < 3; ++k) {
        const auto result = BooleanOp::execute(a, b, types[k], nullptr, NamingScheme::Stable);
        if (!result) {
            if (k == 0) return false;
            continue;
        }
        if (!result->isValid() || !result->checkManifold() ||
            !hz::topo::GeometryValidator::isGeometricallyValid(*result)) {
            return false;
        }
        volume[k] = volumeOf(*result);
    }
    const double va = volumeOf(a);
    const double vb = volumeOf(b);
    const double tol = 1e-7 * std::pow(scale, 3.0);
    return std::abs(volume[1] + volume[2] - va) <= tol &&
           std::abs(volume[0] - (va + vb - volume[2])) <= tol;
}

// How often a random placement comes out unsound, over a hundred of each
// kind: the tests above see only the few placements their seeds draw.
// Measured over two thousand of each (Phase 169): none at the origin, none a
// million millimetres out, none at a hundred thousand times the size, and 14
// at a thousandth of it. The bounds leave room for another platform's
// arithmetic, and catch a slide: two validator faults did, one small
// placement in five (its crossing test compared lengths squared with a
// length) and every far one on Apple silicon (its area vectors, taken from
// coordinates a million millimetres out, did not survive a fused
// multiply-add).
TEST(BooleanRobustnessTest, HowOftenARandomPlacementIsUnsound) {
    const auto box = PrimitiveFactory::makeBox(10, 10, 10);
    const auto cylinder = PrimitiveFactory::makeCylinder(3.0, 12.0, 24);
    const auto unsound = [&](const Mat4& frame, double scale) {
        const auto framedBox = moved(*box, frame);
        int count = 0;
        for (unsigned seed = 1; seed <= 10; ++seed) {
            std::mt19937 rng(seed);
            for (int i = 0; i < 10; ++i) {
                const Mat4 m = frame * Mat4::translation(Vec3(5, 5, 5)) * randomMotion(rng, 4.0);
                if (!combinesSoundly(*framedBox, *moved(*cylinder, m), 10.0 * scale)) ++count;
            }
        }
        return count;
    };
    const int origin = unsound(Mat4::identity(), 1.0);
    const int far = unsound(Mat4::translation(Vec3(1e6, -2e6, 1.5e6)), 1.0);
    const int small = unsound(Mat4::scale(1e-3), 1e-3);
    const int large = unsound(Mat4::scale(1e5), 1e5);
    std::printf("[   INFO   ] unsound of 100: at the origin %d, far %d, small %d, large %d\n",
                origin, far, small, large);
    EXPECT_LE(origin, 2) << "at the origin";
    EXPECT_LE(far, 2) << "far from it";
    EXPECT_LE(small, 4) << "a thousandth of the size";
    EXPECT_LE(large, 2) << "a hundred thousand times";
}

// Faces in exact contact: two boxes meeting face to face, over the whole
// face and over part of it, and sharing a face in part.
TEST(BooleanRobustnessTest, FacesInExactContactAreSound) {
    const auto a = PrimitiveFactory::makeBox(10, 10, 10);
    const auto whole =
        moved(*PrimitiveFactory::makeBox(10, 10, 10), Mat4::translation(Vec3(10, 0, 0)));
    const auto part = moved(*PrimitiveFactory::makeBox(6, 4, 3), Mat4::translation(Vec3(10, 2, 5)));
    const auto shared =
        moved(*PrimitiveFactory::makeBox(6, 4, 3), Mat4::translation(Vec3(7, 2, 5)));
    const auto flush =
        moved(*PrimitiveFactory::makeBox(4, 4, 10), Mat4::translation(Vec3(3, 3, 0)));
    expectSound(*a, *whole, 10.0, "face to face");
    expectSound(*a, *part, 10.0, "part of a face");
    expectSound(*a, *shared, 10.0, "a face shared in part");
    expectSound(*a, *flush, 10.0, "flush top and bottom");

    // Face to face, the union is one box twice as long.
    const auto joined =
        BooleanOp::execute(*a, *whole, BooleanType::Union, nullptr, NamingScheme::Stable);
    ASSERT_NE(joined, nullptr);
    EXPECT_NEAR(volumeOf(*joined), 2000.0, 1e-7);
}

// Boxes that meet only along an edge, or at a corner. The edge's four faces
// were paired as they were found, as often across the two boxes as within
// one, and the corner's vertex was shared by both: a union that read as one
// closed body with every count right (or, at a corner, as counts no solid
// has), and was not one. It is two bodies now, each sound, sharing nothing.
TEST(BooleanRobustnessTest, BoxesMeetingAlongAnEdgeOrAtAPointAreTwoBodies) {
    const auto a = PrimitiveFactory::makeBox(10, 10, 10);
    const struct {
        Vec3 at;
        const char* what;
    } cases[] = {{Vec3(10, 10, 0), "along an edge"}, {Vec3(10, 10, 10), "at a corner"}};
    for (const auto& c : cases) {
        const auto b = moved(*PrimitiveFactory::makeBox(10, 10, 10), Mat4::translation(c.at));
        for (const NamingScheme naming : {NamingScheme::Stable, NamingScheme::Positional}) {
            std::string why;
            const auto joined = BooleanOp::execute(*a, *b, BooleanType::Union, &why, naming);
            ASSERT_NE(joined, nullptr) << c.what << ": " << why;
            EXPECT_EQ(joined->shellCount(), 2u) << c.what;
            EXPECT_EQ(joined->vertexCount(), 16u) << c.what << ": no vertex shared";
            EXPECT_TRUE(joined->isValid()) << c.what << ": " << joined->validationReport();
            EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*joined))
                << c.what << ": " << hz::topo::GeometryValidator::report(*joined);
            // Each shell's half-edges twin within it.
            for (const auto& face : joined->faces()) {
                const hz::topo::HalfEdge* start = face.outerLoop->halfEdge;
                const hz::topo::HalfEdge* he = start;
                do {
                    ASSERT_NE(he->twin, nullptr);
                    EXPECT_EQ(he->twin->face->shell, face.shell) << c.what;
                    he = he->next;
                } while (he != start);
            }
            EXPECT_NEAR(volumeOf(*joined), 2000.0, 1e-7) << c.what;
        }
        expectSound(*a, *b, 10.0, c.what);
    }
}

// Bodies touching face to face, sewn together (as faceting a STEP file of
// touching bodies sews them): the face of the one lying back to back on the
// other's is met at no turn about their edges, and on faces not quite flat
// (a curved face's facets) a hair either side of none. Taken first, it sewed
// the two bodies into one shell; it is taken last, and they are two. Each
// box's contact face is the same quad, its far corner a thousandth off the
// plane, its loop started at each corner in turn, in either order.
TEST(BooleanRobustnessTest, BodiesTouchingFaceToFaceAreSewnApart) {
    using Face = hz::model::SolidSewer::InputFace;
    const auto turned = [](const std::vector<Vec3>& loop, int start) {
        std::vector<Vec3> out;
        for (size_t i = 0; i < loop.size(); ++i) {
            out.push_back(loop[(i + static_cast<size_t>(start)) % loop.size()]);
        }
        return out;
    };
    // A box's faces, outward wound, from its corners (bottom 0-3, top 4-7).
    const auto box = [&turned](const std::vector<Vec3>& c, int bottomStart, int topStart) {
        std::vector<Face> faces(6);
        faces[0].points = turned({c[0], c[3], c[2], c[1]}, bottomStart);
        faces[1].points = turned({c[4], c[5], c[6], c[7]}, topStart);
        faces[2].points = {c[0], c[1], c[5], c[4]};
        faces[3].points = {c[1], c[2], c[6], c[5]};
        faces[4].points = {c[2], c[3], c[7], c[6]};
        faces[5].points = {c[3], c[0], c[4], c[7]};
        return faces;
    };
    for (const double off : {1e-3, -1e-3}) {
        const Vec3 corner(1, 1, 1 + off);  // shared by both contact faces
        const std::vector<Vec3> lower = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                                         {0, 0, 1}, {1, 0, 1}, corner,    {0, 1, 1}};
        const std::vector<Vec3> upper = {{0, 0, 1}, {1, 0, 1}, corner,    {0, 1, 1},
                                         {0, 0, 2}, {1, 0, 2}, {1, 1, 2}, {0, 1, 2}};
        for (int start = 0; start < 16; ++start) {
            for (const bool lowerFirst : {true, false}) {
                const auto a = box(lower, 0, start % 4);
                const auto b = box(upper, start / 4, 0);
                std::vector<Face> faces = lowerFirst ? a : b;
                const auto& rest = lowerFirst ? b : a;
                faces.insert(faces.end(), rest.begin(), rest.end());
                const auto sewn = hz::model::SolidSewer::sew(faces);
                ASSERT_NE(sewn, nullptr);
                const std::string what = "off " + std::to_string(off) + ", starts " +
                                         std::to_string(start) +
                                         (lowerFirst ? ", lower first" : ", upper first");
                ASSERT_EQ(sewn->shellCount(), 2u) << what;
                for (const auto& shell : sewn->shells()) EXPECT_EQ(shell.faces.size(), 6u) << what;
                EXPECT_TRUE(sewn->isValid()) << what << "\n" << sewn->validationReport();
            }
        }
    }
}

// A pattern merges instances whose boxes meet; boxes in a row edge to edge
// are as many bodies, each sound.
TEST(BooleanRobustnessTest, APatternOfBoxesEdgeToEdgeBuilds) {
    const auto box = PrimitiveFactory::makeBox(10, 10, 10);
    const auto row = Pattern::linear(*box, Vec3(1, 1, 0).normalized(), 10.0 * std::sqrt(2.0), 3, {},
                                     NamingScheme::Stable);
    ASSERT_NE(row, nullptr);
    EXPECT_EQ(row->shellCount(), 3u);
    EXPECT_TRUE(row->isValid()) << row->validationReport();
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*row))
        << hz::topo::GeometryValidator::report(*row);
    EXPECT_NEAR(volumeOf(*row), 3000.0, 1e-6);
}

// A face with a hole, kept as it is on a path that splits nothing: its
// keyhole outline, a bridge out to the hole and back, was sewn into a face
// with a bridge edge it bounded on both sides, and the union of a holed
// plate with a box that does not reach it was refused for a boundary that
// crosses itself.
TEST(BooleanRobustnessTest, AHoledFaceIsKeptWhereNothingIsCut) {
    // A washer: a rectangle off the axis turned a whole turn, its flat ends
    // each one face with the bore a hole in it.
    std::vector<std::shared_ptr<hz::draft::DraftEntity>> profile;
    const std::vector<hz::math::Vec2> corners = {{5, 0}, {10, 0}, {10, 2}, {5, 2}};
    for (size_t i = 0; i < corners.size(); ++i) {
        profile.push_back(
            std::make_shared<hz::draft::DraftLine>(corners[i], corners[(i + 1) % corners.size()]));
    }
    const auto holed = hz::model::Revolve::execute(profile, hz::draft::SketchPlane(), Vec3(0, 0, 0),
                                                   Vec3(0, 1, 0), 2.0 * std::numbers::pi, "washer",
                                                   32, 0.0, nullptr, NamingScheme::Stable);
    ASSERT_NE(holed, nullptr);
    size_t holes = 0;
    for (const auto& face : holed->faces()) holes += face.innerLoops.size();
    ASSERT_EQ(holes, 2u) << "the washer's two flat ends have the bore in them";

    const auto far = moved(*PrimitiveFactory::makeBox(2, 2, 2), Mat4::translation(Vec3(50, 0, 0)));
    for (const NamingScheme naming : {NamingScheme::Stable, NamingScheme::Positional}) {
        std::string why;
        const auto joined = BooleanOp::execute(*holed, *far, BooleanType::Union, &why, naming);
        ASSERT_NE(joined, nullptr) << why;
        EXPECT_TRUE(joined->isValid()) << joined->validationReport();
        EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*joined))
            << hz::topo::GeometryValidator::report(*joined);
        size_t kept = 0;
        for (const auto& face : joined->faces()) kept += face.innerLoops.size();
        EXPECT_EQ(kept, holes) << "each hole is a hole still";
        EXPECT_NEAR(volumeOf(*joined), volumeOf(*holed) + 8.0, 1e-7);

        const auto cut = BooleanOp::execute(*holed, *far, BooleanType::Subtract, &why, naming);
        ASSERT_NE(cut, nullptr) << why;
        EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*cut))
            << hz::topo::GeometryValidator::report(*cut);
        EXPECT_NEAR(volumeOf(*cut), volumeOf(*holed), 1e-7);
    }
}

// A plate with 81 holes cut at once: its top and bottom are put back
// together in strips between the holes. Past 60 holes they were left in the
// CSG's fragments, hundreds of triangles, and their names with them.
TEST(BooleanRobustnessTest, AFaceWithManyHolesIsMerged) {
    const auto plate = PrimitiveFactory::makeBox(10, 10, 1);
    auto pin =
        moved(*PrimitiveFactory::makeCylinder(0.25, 3.0, 12), Mat4::translation(Vec3(1, 1, -1)));
    const auto row = Pattern::linear(*pin, Vec3(1, 0, 0), 1.0, 9);
    const auto grid = Pattern::linear(*row, Vec3(0, 1, 0), 1.0, 9);
    std::string reason;
    const auto drilled =
        BooleanOp::execute(*plate, *grid, BooleanType::Subtract, &reason, NamingScheme::Stable);
    ASSERT_NE(drilled, nullptr) << reason;
    EXPECT_TRUE(drilled->isValid()) << drilled->validationReport();
    const double hole = 6.0 * 0.25 * 0.25 * std::sin(2.0 * std::numbers::pi / 12.0);  // 12-gon
    EXPECT_NEAR(volumeOf(*drilled), 100.0 - 81.0 * hole, 1e-9);
    size_t top = 0;
    for (const auto& face : drilled->faces()) {
        top += face.topoId.tag().rfind("box/top", 0) == 0 ? 1 : 0;
    }
    EXPECT_GT(top, 0u);
    EXPECT_LE(top, 2u * 81u + 2u) << "strips between the holes, not fragments";
}

namespace {

/// Runs @p work on a thread whose stack is @p bytes; whether it could.
bool onStackOf(size_t bytes, void (*work)(void*), void* arg) {
#if defined(__unix__) || defined(__APPLE__)
    pthread_attr_t attr;
    if (pthread_attr_init(&attr) != 0) return false;
    const bool sized = pthread_attr_setstacksize(&attr, bytes) == 0;
    pthread_t thread;
    struct Call {
        void (*work)(void*);
        void* arg;
    } call{work, arg};
    const bool started = sized && pthread_create(
                                      &thread, &attr,
                                      [](void* p) -> void* {
                                          auto* c = static_cast<Call*>(p);
                                          c->work(c->arg);
                                          return nullptr;
                                      },
                                      &call) == 0;
    pthread_attr_destroy(&attr);
    if (started) pthread_join(thread, nullptr);
    return started;
#else
    (void)bytes;
    (void)work;
    (void)arg;
    return false;
#endif
}

}  // namespace

// No part of a Boolean recurses as deep as its BSP: a cylinder of 1,024
// facets, whose tree is a chain 1,024 nodes deep, is cut from a box on a
// 128 KB stack. The recursive traversal needed several hundred.
TEST(BooleanRobustnessTest, ADeepTreeNeedsNoDeepStack) {
    struct Case {
        std::unique_ptr<hz::topo::Solid> result;
        std::string reason;
    } c;
    // Sanitizers' frames are several times larger.
    const size_t stack = HZ_SANITIZED ? size_t{1} << 20 : size_t{128} << 10;
    const bool ran = onStackOf(
        stack,
        [](void* p) {
            auto& c = *static_cast<Case*>(p);
            const auto box = PrimitiveFactory::makeBox(20, 20, 10);
            const auto pin = moved(*PrimitiveFactory::makeCylinder(5.0, 20.0, 1024),
                                   Mat4::translation(Vec3(10, 10, -5)));
            c.result = BooleanOp::execute(*box, *pin, BooleanType::Subtract, &c.reason,
                                          NamingScheme::Stable);
        },
        &c);
    if (!ran) GTEST_SKIP() << "a thread's stack size is set here with POSIX threads";
    ASSERT_NE(c.result, nullptr) << c.reason;
    const double hole = 0.5 * 1024 * 25.0 * std::sin(2.0 * std::numbers::pi / 1024);
    EXPECT_NEAR(volumeOf(*c.result), 4000.0 - 10.0 * hole, 1e-6);
}

// Phase 165: a box less a finely faceted pin. Building the pin's tree
// split-tested each of its polygons against every plane before it; a convex
// operand's tree is a chain of its planes, now built in one pass, the same
// tree. Measured on one machine: 0.81 s to 0.30 s optimised, 4.0 s to 1.7 s
// in a Debug build. The limits leave room for CI's shared runners: they
// catch a slide back into a quadratic build at this size, not the old code,
// which a looser machine could not tell from the new one.
TEST(BooleanRobustnessTest, AFinelyFacetedPinCutsQuickly) {
    const auto box = PrimitiveFactory::makeBox(20, 20, 10);
    const auto pin = moved(*PrimitiveFactory::makeCylinder(5.0, 20.0, 2048),
                           Mat4::translation(Vec3(10, 10, -5)));
    const auto start = std::chrono::steady_clock::now();
    std::string why;
    const auto result =
        BooleanOp::execute(*box, *pin, BooleanType::Subtract, &why, NamingScheme::Stable);
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    ASSERT_NE(result, nullptr) << why;
    const double hole = 0.5 * 2048 * 25.0 * std::sin(2.0 * std::numbers::pi / 2048);
    EXPECT_NEAR(volumeOf(*result), 4000.0 - 10.0 * hole, 1e-6);
#if HZ_TIME_LIMITS
#ifdef NDEBUG
    EXPECT_LT(seconds, 1.0);
#else
    EXPECT_LT(seconds, 10.0) << "unoptimized: a generous bound";
#endif
#else
    (void)seconds;
#endif
}

// A part of many facets cut by another: a hole through a cylinder of 2,048
// facets, made with another as fine. Each cap, fanned into slivers, was cut
// by every plane of the other into hundreds of thousands of fragments, merged
// back after: 10 s in a build with some optimisation, where it now takes
// under one. A convex face now goes into the cut whole, what lies clear of
// the other operand is not cut, and what is left is not cut again.
TEST(BooleanRobustnessTest, ACylinderOfManyFacetsIsBoredQuickly) {
    constexpr int kFacets = 2048;
    const auto rod = PrimitiveFactory::makeCylinder(10.0, 20.0, kFacets);
    const auto drill = moved(*PrimitiveFactory::makeCylinder(4.0, 40.0, kFacets),
                             Mat4::translation(Vec3(0, 0, -10)));
    const auto start = std::chrono::steady_clock::now();
    std::string why;
    const auto result =
        BooleanOp::execute(*rod, *drill, BooleanType::Subtract, &why, NamingScheme::Stable);
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    ASSERT_NE(result, nullptr) << why;
    EXPECT_TRUE(result->isValid()) << result->validationReport();
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*result))
        << hz::topo::GeometryValidator::report(*result);
    // Two inscribed polygons' areas apart, through the rod's height.
    const double ring = 0.5 * kFacets * (100.0 - 16.0) * std::sin(2.0 * std::numbers::pi / kFacets);
    EXPECT_NEAR(volumeOf(*result), 20.0 * ring, 1e-6);
    // The rod's sides, the bore's sides, and each ring cap in two: a face
    // has one loop, so a cap with a hole is cut through it (FragmentMerge).
    // As many faces as the cut made before it was faster.
    EXPECT_EQ(result->faces().size(), static_cast<size_t>(2 * kFacets + 4));
#if HZ_TIME_LIMITS
#ifdef NDEBUG
    EXPECT_LT(seconds, 1.5);
#else
    // CI's debug build runs it unoptimized, four tests at a time: 15 s
    // there. It took 48 s optimized before the cut was made faster.
    EXPECT_LT(seconds, 45.0 * HZ_UNOPTIMIZED_SLOWDOWN) << "unoptimized: a generous bound";
#endif
#else
    (void)seconds;
#endif
}

// The faster cut is the same cut: for a cylinder through a box at a slant,
// with faces that are not all convex once a hole is in them, the pieces
// merged agree with the volume the operands give, three ways.
TEST(BooleanRobustnessTest, ACutOfFewerPiecesConservesVolume) {
    const auto box = PrimitiveFactory::makeBox(20, 20, 10);
    const auto pin = moved(
        *PrimitiveFactory::makeCylinder(4.0, 30.0, 96),
        Mat4::translation(Vec3(10, 10, -8)) *
            Mat4::rotation(hz::math::Quaternion::fromAxisAngle(Vec3(1, 1, 0).normalized(), 0.3)));
    expectSound(*box, *pin, 20.0, "slanted pin");
    // Bored, then bored again across the first hole: faces with holes, and
    // faces no longer convex, go in as triangles.
    const auto bored =
        BooleanOp::execute(*box, *pin, BooleanType::Subtract, nullptr, NamingScheme::Stable);
    ASSERT_NE(bored, nullptr);
    const auto cross = moved(
        *PrimitiveFactory::makeCylinder(3.0, 30.0, 64),
        Mat4::translation(Vec3(-5, 10, 5)) * Mat4::rotation(hz::math::Quaternion::fromAxisAngle(
                                                 Vec3(0, 1, 0), std::numbers::pi / 2)));
    expectSound(*bored, *cross, 20.0, "cross-bored");
}

// A pattern of finely faceted cylinders, each overlapping the one before:
// each union cut every face of the part along the planes of the next
// cylinder, however far off, and eight of 4,096 facets took most of two
// minutes. A face clear of every face of the other body is kept or left out
// whole now, by where one point of it lies. Each step joins soundly, and
// the last, three ways, conserves volume.
TEST(BooleanRobustnessTest, APatternOfOverlappingCylindersJoinsSoundly) {
    constexpr int kFacets = 512;
    constexpr int kCount = 6;
    const auto cylinder = PrimitiveFactory::makeCylinder(5.0, 10.0, kFacets);
    std::unique_ptr<hz::topo::Solid> part = moved(*cylinder, Mat4::translation(Vec3()));
    for (int i = 1; i < kCount; ++i) {
        const auto next = moved(*cylinder, Mat4::translation(Vec3(3.0 * i, 1.0 * i, 0.5 * i)));
        if (i == kCount - 1) expectSound(*part, *next, 30.0, "the last of the pattern");
        std::string why;
        auto joined =
            BooleanOp::execute(*part, *next, BooleanType::Union, &why, NamingScheme::Stable);
        ASSERT_NE(joined, nullptr) << "instance " << i << ": " << why;
        EXPECT_TRUE(joined->isValid()) << joined->validationReport();
        EXPECT_GT(volumeOf(*joined), volumeOf(*part));
        part = std::move(joined);
    }
}

// Pieces clear of the other body, kept or left out whole: a box inside
// another touches none of its faces, and a box in the notch of an L lies
// in its bounds but outside it. Each combines three ways, conserving volume.
TEST(BooleanRobustnessTest, BodiesNearNoFaceOfEachOtherCombineSoundly) {
    const auto outer = PrimitiveFactory::makeBox(10, 10, 10);
    const auto inner = moved(*PrimitiveFactory::makeBox(2, 2, 2), Mat4::translation(Vec3(4, 4, 4)));
    expectSound(*outer, *inner, 10.0, "a box inside a box");
    std::string why;
    const auto joined =
        BooleanOp::execute(*outer, *inner, BooleanType::Union, &why, NamingScheme::Stable);
    ASSERT_NE(joined, nullptr) << why;
    EXPECT_NEAR(volumeOf(*joined), 1000.0, 1e-9) << "the inner box is inside: nothing added";
    const auto common =
        BooleanOp::execute(*outer, *inner, BooleanType::Intersect, &why, NamingScheme::Stable);
    ASSERT_NE(common, nullptr) << why;
    EXPECT_NEAR(volumeOf(*common), 8.0, 1e-9);

    // An L: two boxes joined, and a third in its notch, within its bounds.
    const auto foot = PrimitiveFactory::makeBox(10, 3, 3);
    const auto leg = PrimitiveFactory::makeBox(3, 10, 3);
    const auto ell =
        BooleanOp::execute(*foot, *leg, BooleanType::Union, &why, NamingScheme::Stable);
    ASSERT_NE(ell, nullptr) << why;
    const auto notch =
        moved(*PrimitiveFactory::makeBox(2, 2, 2), Mat4::translation(Vec3(6, 6, 0.5)));
    expectSound(*ell, *notch, 10.0, "a box in the notch of an L");
}

// Bodies too finely faceted to combine in time are refused, saying why,
// where the cut ran on for an hour or until memory ran out; and an
// interference check that cannot finish reports the pair, to be looked at,
// rather than passing it as clear. (Here, under limits far below the
// defaults, a small cut does.)
TEST(BooleanRobustnessTest, ACutTooLargeToMakeIsRefusedWithAReason) {
    const auto rod = PrimitiveFactory::makeCylinder(10.0, 20.0, 256);
    const auto drill =
        moved(*PrimitiveFactory::makeCylinder(4.0, 40.0, 256), Mat4::translation(Vec3(1, 0, -10)));
    {
        const hz::model::BooleanWorkLimits limits(20'000, 100);
        std::string why;
        const auto result =
            BooleanOp::execute(*rod, *drill, BooleanType::Subtract, &why, NamingScheme::Stable);
        EXPECT_EQ(result, nullptr);
        EXPECT_NE(why.find("too many facets"), std::string::npos) << why;
        EXPECT_TRUE(hz::model::InterferenceChecker::solidsInterfere(*rod, *drill));
    }
    std::string why;
    const auto result =
        BooleanOp::execute(*rod, *drill, BooleanType::Subtract, &why, NamingScheme::Stable);
    ASSERT_NE(result, nullptr) << "the limits are lifted with their scope: " << why;
    EXPECT_TRUE(result->isValid());
}
