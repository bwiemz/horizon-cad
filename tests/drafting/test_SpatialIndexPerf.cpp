#include <gtest/gtest.h>

#include <chrono>
#include <iostream>

#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SnapEngine.h"
#include "horizon/drafting/SpatialIndex.h"
#include "horizon/math/BoundingBox.h"

#if defined(__SANITIZE_THREAD__)
#define HZ_UNDER_TSAN 1
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer)
#define HZ_UNDER_TSAN 1
#endif
#endif

using namespace hz::draft;
using namespace hz::math;

TEST(SpatialIndexPerfTest, TenThousandEntitySnapUnder1ms) {
#ifdef HZ_UNDER_TSAN
    GTEST_SKIP()
        << "a time limit means nothing under ThreadSanitizer, which runs code 5-15x slower";
#endif
    std::vector<std::shared_ptr<DraftEntity>> entities;
    entities.reserve(10000);
    for (uint64_t i = 0; i < 10000; ++i) {
        double x = static_cast<double>(i % 100) * 5.0;
        double y = static_cast<double>(i / 100) * 5.0;
        auto line = std::make_shared<DraftLine>(Vec2(x, y), Vec2(x + 3, y + 3));
        line->setId(i + 1);
        entities.push_back(line);
    }

    SpatialIndex index;
    index.rebuild(entities);

    SnapEngine engine;
    engine.setSnapTolerance(2.0);
    engine.setGridSpacing(5.0);

    Vec2 cursor(250.0, 250.0);
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; ++i) {
        engine.snap(cursor, index, entities);
    }
    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgMs = totalMs / 1000.0;
    std::cout << "[PERF] 10k entities, avg snap query: " << avgMs << " ms" << std::endl;
#ifdef NDEBUG
    EXPECT_LT(avgMs, 1.0) << "Snap query too slow: " << avgMs << " ms average";
#else
    // Debug builds inline nothing, and MSVC's check every iterator: this snap
    // took 2.7 ms on the Windows CI runner and 0.08 ms in a GCC Debug build.
    // The 1 ms budget binds in Release; here only a regression of an order of
    // magnitude fails.
    EXPECT_LT(avgMs, 20.0) << "Snap query too slow: " << avgMs << " ms average";
#endif
}

TEST(SpatialIndexPerfTest, TenThousandEntityInsertUnder100ms) {
#ifdef HZ_UNDER_TSAN
    GTEST_SKIP()
        << "a time limit means nothing under ThreadSanitizer, which runs code 5-15x slower";
#endif
    SpatialIndex index;
    auto start = std::chrono::high_resolution_clock::now();
    for (uint64_t i = 0; i < 10000; ++i) {
        double x = static_cast<double>(i % 100) * 5.0;
        double y = static_cast<double>(i / 100) * 5.0;
        auto line = std::make_shared<DraftLine>(Vec2(x, y), Vec2(x + 3, y + 3));
        line->setId(i + 1);
        index.insert(line);
    }
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << "[PERF] 10k entity insert: " << ms << " ms" << std::endl;
#ifdef NDEBUG
    EXPECT_LT(ms, 100.0);
#else
    // Debug builds on shared CI runners (especially MSVC with iterator
    // debugging) run several times slower, and CI runs four tests at once:
    // this took 570 ms under ASan and on MSVC Debug. The perf target only
    // binds in Release; here only a much worse regression fails.
    EXPECT_LT(ms, 2000.0);
#endif
}

TEST(SpatialIndexPerfTest, TenThousandEntityBoxSelectUnder5ms) {
#ifdef HZ_UNDER_TSAN
    GTEST_SKIP()
        << "a time limit means nothing under ThreadSanitizer, which runs code 5-15x slower";
#endif
    SpatialIndex index;
    for (uint64_t i = 0; i < 10000; ++i) {
        double x = static_cast<double>(i % 100) * 5.0;
        double y = static_cast<double>(i / 100) * 5.0;
        auto line = std::make_shared<DraftLine>(Vec2(x, y), Vec2(x + 3, y + 3));
        line->setId(i + 1);
        index.insert(line);
    }

    BoundingBox queryBox(Vec3(200, 200, -1e9), Vec3(250, 250, 1e9));

    std::size_t resultCount = 0;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; ++i) {
        resultCount = index.query(queryBox).size();
    }
    auto end = std::chrono::high_resolution_clock::now();
    (void)resultCount;

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgMs = totalMs / 1000.0;
    std::cout << "[PERF] 10k entities, avg box query: " << avgMs << " ms" << std::endl;
#ifdef NDEBUG
    EXPECT_LT(avgMs, 5.0) << "Box query too slow: " << avgMs << " ms average";
#else
    EXPECT_LT(avgMs, 20.0) << "Box query too slow: " << avgMs << " ms average";  // as the snap's
#endif
}
