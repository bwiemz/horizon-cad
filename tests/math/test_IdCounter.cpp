// IDs stay unique when handed out from several threads at once (Phase 114):
// a document is loaded and rebuilt on a worker while the window makes
// entities on the main thread.

#include <gtest/gtest.h>

#include <cstdint>
#include <set>
#include <thread>
#include <vector>

#include "horizon/math/IdCounter.h"

using hz::math::IdCounter;

TEST(IdCounterTest, IdsFromManyThreadsAreAllDifferent) {
    IdCounter<std::uint64_t> counter{1};
    constexpr int kThreads = 8;
    constexpr int kEach = 5000;
    std::vector<std::vector<std::uint64_t>> got(kThreads);
    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&counter, &got, t] {
            for (int i = 0; i < kEach; ++i) got[static_cast<size_t>(t)].push_back(counter.next());
        });
    }
    for (auto& thread : threads) thread.join();
    std::set<std::uint64_t> all;
    for (const auto& ids : got) all.insert(ids.begin(), ids.end());
    EXPECT_EQ(all.size(), static_cast<size_t>(kThreads * kEach));
    EXPECT_EQ(*all.begin(), 1u);
    EXPECT_EQ(counter.peek(), static_cast<std::uint64_t>(kThreads * kEach + 1));
}

TEST(IdCounterTest, ReservingPastLoadedIdsNeverGoesBack) {
    IdCounter<int> counter{1};
    counter.reserveThrough(41);
    EXPECT_EQ(counter.next(), 42);
    counter.reserveThrough(10);  // already past it
    EXPECT_EQ(counter.next(), 43);

    // Racing reservations settle on the largest.
    std::vector<std::thread> threads;
    for (int t = 0; t < 8; ++t)
        threads.emplace_back([&counter, t] { counter.reserveThrough(100 + t * 10); });
    for (auto& thread : threads) thread.join();
    EXPECT_EQ(counter.next(), 171);
}
