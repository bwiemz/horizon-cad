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

}  // namespace hz::test
