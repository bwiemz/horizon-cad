#pragma once

#include <atomic>

namespace hz::math {

/// Hands out increasing IDs, safely from any thread: a document can be loaded
/// or rebuilt on a worker while the window makes entities on the main thread.
template <typename T>
class IdCounter {
public:
    explicit constexpr IdCounter(T first) noexcept : m_next(first) {}

    IdCounter(const IdCounter&) = delete;
    IdCounter& operator=(const IdCounter&) = delete;

    /// An ID not handed out before.
    T next() noexcept { return m_next.fetch_add(1, std::memory_order_relaxed); }

    /// Never hand out @p id or anything below it again (after IDs are read
    /// back from a file).
    void reserveThrough(T id) noexcept {
        T current = m_next.load(std::memory_order_relaxed);
        while (current <= id &&
               !m_next.compare_exchange_weak(current, id + 1, std::memory_order_relaxed)) {
        }
    }

    /// The ID next() would hand out now.
    T peek() const noexcept { return m_next.load(std::memory_order_relaxed); }

private:
    std::atomic<T> m_next;
};

}  // namespace hz::math
