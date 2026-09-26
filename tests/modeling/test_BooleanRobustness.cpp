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
#include <memory>
#include <numbers>
#include <random>
#include <string>

#include "../TimeLimits.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Quaternion.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
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
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    std::uniform_real_distribution<double> angle(0.0, 2.0 * std::numbers::pi);
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
