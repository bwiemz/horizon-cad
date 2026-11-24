#pragma once

// Random draws that are the same on every platform. std::mt19937's numbers are
// fixed by the standard, but the distributions over them are not: libc++ and
// libstdc++ draw differently from the same engine, so a test's random cases
// were other cases on macOS than on Linux (Phase 169).

#include <cstdint>
#include <random>

namespace hz::test {

/// A double in [lo, hi), as std::uniform_real_distribution<double>(lo, hi)
/// is used, from 53 bits of two 32-bit draws.
class Uniform {
public:
    Uniform(double lo, double hi) : m_lo(lo), m_hi(hi) {}

    template <class Engine>
    double operator()(Engine& rng) const {
        static_assert(Engine::max() - Engine::min() == 0xFFFFFFFFu, "a 32-bit engine");
        const std::uint64_t high = static_cast<std::uint64_t>(rng() - Engine::min()) >> 5;
        const std::uint64_t low = static_cast<std::uint64_t>(rng() - Engine::min()) >> 6;
        const double unit = static_cast<double>((high << 26) | low) / 9007199254740992.0;  // 2^53
        return m_lo + (m_hi - m_lo) * unit;
    }

private:
    double m_lo;
    double m_hi;
};

/// An integer in [lo, hi], both included, as std::uniform_int_distribution<std::int64_t>(lo, hi)
/// is used, from 64 bits of two 32-bit draws. A draw below 2^64 mod (hi - lo + 1) is drawn
/// again, so every value is equally likely.
class UniformInt {
public:
    UniformInt(std::int64_t lo, std::int64_t hi) : m_lo(lo), m_hi(hi) {}

    template <class Engine>
    std::int64_t operator()(Engine& rng) const {
        static_assert(Engine::max() - Engine::min() == 0xFFFFFFFFu, "a 32-bit engine");
        // hi - lo + 1, modulo 2^64: 0 when the range is every 64-bit integer.
        const std::uint64_t range =
            static_cast<std::uint64_t>(m_hi) - static_cast<std::uint64_t>(m_lo) + 1u;
        const std::uint64_t reject = range == 0u ? 0u : (0u - range) % range;
        for (;;) {
            const std::uint64_t high = static_cast<std::uint64_t>(rng() - Engine::min());
            const std::uint64_t low = static_cast<std::uint64_t>(rng() - Engine::min());
            const std::uint64_t x = (high << 32) | low;
            if (x < reject) continue;
            const std::uint64_t offset = range == 0u ? x : x % range;
            return static_cast<std::int64_t>(static_cast<std::uint64_t>(m_lo) + offset);
        }
    }

private:
    std::int64_t m_lo;
    std::int64_t m_hi;
};

}  // namespace hz::test
