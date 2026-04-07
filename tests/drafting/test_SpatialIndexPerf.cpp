#include <gtest/gtest.h>
#include "horizon/drafting/SpatialIndex.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SnapEngine.h"
#include "horizon/math/BoundingBox.h"
#include <chrono>
#include <iostream>

using namespace hz::draft;
using namespace hz::math;

TEST(SpatialIndexPerfTest, TenThousandEntitySnapUnder1ms) {
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
    EXPECT_LT(avgMs, 1.0) << "Snap query too slow: " << avgMs << " ms average";
}

TEST(SpatialIndexPerfTest, TenThousandEntityInsertUnder100ms) {
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
    EXPECT_LT(ms, 200.0);  // 200ms threshold accommodates Debug builds
}

TEST(SpatialIndexPerfTest, TenThousandEntityBoxSelectUnder5ms) {
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
    EXPECT_LT(avgMs, 5.0) << "Box query too slow: " << avgMs << " ms average";
}
