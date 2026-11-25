// Fuzz target: the constrained Delaunay triangulation of a region. The bytes decode to an outer
// loop, up to two holes and a merge distance, all finite and of modest size so the fuzzer
// explores structure, not overflow. Any input may be refused (a non-empty `problem`); when it
// is not, every boundary segment between distinct merged points must be an edge of some
// triangle and every triangle must be counter-clockwise. A violation aborts.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <set>
#include <utility>
#include <vector>

#include "horizon/geometry/Triangulate2D.h"
#include "horizon/math/RobustPredicates.h"

namespace {

using hz::math::Vec2;

/// Reads bytes in order; past the end, zeros.
struct Reader {
    const uint8_t* data;
    size_t size;
    size_t pos = 0;

    uint8_t byte() { return pos < size ? data[pos++] : uint8_t{0}; }

    /// A coordinate in [-128, 128), in steps of 1/256.
    double coordinate() {
        const auto lo = static_cast<uint16_t>(byte());
        const auto hi = static_cast<uint16_t>(byte());
        return static_cast<double>(static_cast<int16_t>(static_cast<uint16_t>(lo | (hi << 8)))) /
               256.0;
    }

    std::vector<Vec2> loop(size_t count) {
        std::vector<Vec2> points;
        for (size_t i = 0; i < count; ++i) {
            const double x = coordinate();
            points.push_back({x, coordinate()});
        }
        return points;
    }
};

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    Reader in{data, size};
    const double merge = static_cast<double>(in.byte() % 8) / 64.0;
    const size_t outerCount = 3 + in.byte() % 6;
    const size_t holeCount = in.byte() % 3;
    const std::vector<Vec2> outer = in.loop(outerCount);
    std::vector<std::vector<Vec2>> holes;
    for (size_t h = 0; h < holeCount; ++h) {
        holes.push_back(in.loop(3 + in.byte() % 4));
    }

    const hz::geo::Triangulation2D t = hz::geo::triangulateRegion(outer, holes, {}, merge);
    if (!t.problem.empty()) return 0;

    std::set<std::pair<uint32_t, uint32_t>> edges;
    for (const auto& tri : t.triangles) {
        for (int k = 0; k < 3; ++k) {
            const uint32_t a = tri[static_cast<size_t>(k)];
            const uint32_t b = tri[static_cast<size_t>((k + 1) % 3)];
            edges.insert({std::min(a, b), std::max(a, b)});
        }
        if (hz::math::orient2d(t.points[tri[0]], t.points[tri[1]], t.points[tri[2]]) <= 0) {
            std::abort();
        }
    }

    size_t start = 0;
    const auto checkLoop = [&](size_t n) {
        for (size_t i = 0; i < n; ++i) {
            const uint32_t a = t.boundaryVertex[start + i];
            const uint32_t b = t.boundaryVertex[start + (i + 1) % n];
            if (a != b && edges.count({std::min(a, b), std::max(a, b)}) == 0) std::abort();
        }
        start += n;
    };
    checkLoop(outer.size());
    for (const auto& h : holes) checkLoop(h.size());
    return 0;
}
