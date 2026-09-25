#include "FragmentMerge.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hz::model {

using math::Vec3;

namespace {

using Loop = std::vector<Vec3>;

/// Newell normal of a loop: its direction is the loop's normal, its length
/// twice the loop's area. Summed about the first point, so its error does
/// not grow with the loop's distance from the origin.
Vec3 newell(const Loop& pts) {
    Vec3 n(0, 0, 0);
    for (size_t i = 0; i < pts.size(); ++i) {
        const Vec3 p = pts[i] - pts[0];
        const Vec3 q = pts[(i + 1) % pts.size()] - pts[0];
        n.x += (p.y - q.y) * (p.z + q.z);
        n.y += (p.z - q.z) * (p.x + q.x);
        n.z += (p.x - q.x) * (p.y + q.y);
    }
    return n;
}

/// The points of one plane, each welded to the first within `tol` of it.
class Welder {
public:
    explicit Welder(double tol) : m_tol(tol), m_cell(std::max(tol, 1e-12) * 2.0) {}

    int index(const Vec3& p) {
        const Key key = cellOf(p);
        for (int64_t dx = -1; dx <= 1; ++dx) {
            for (int64_t dy = -1; dy <= 1; ++dy) {
                for (int64_t dz = -1; dz <= 1; ++dz) {
                    const auto it = m_grid.find({key.x + dx, key.y + dy, key.z + dz});
                    if (it == m_grid.end()) continue;
                    for (const int id : it->second) {
                        if ((m_points[static_cast<size_t>(id)] - p).length() <= m_tol) return id;
                    }
                }
            }
        }
        const int id = static_cast<int>(m_points.size());
        m_points.push_back(p);
        m_grid[key].push_back(id);
        return id;
    }

    const std::vector<Vec3>& points() const { return m_points; }

private:
    struct Key {
        int64_t x, y, z;
        bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct KeyHash {
        size_t operator()(const Key& k) const {
            const auto h = [](int64_t v) { return std::hash<int64_t>{}(v); };
            return h(k.x) ^ (h(k.y) * 0x9E3779B97F4A7C15ULL) ^ (h(k.z) * 0xC2B2AE3D27D4EB4FULL);
        }
    };
    Key cellOf(const Vec3& p) const {
        return {static_cast<int64_t>(std::floor(p.x / m_cell)),
                static_cast<int64_t>(std::floor(p.y / m_cell)),
                static_cast<int64_t>(std::floor(p.z / m_cell))};
    }

    double m_tol;
    double m_cell;
    std::vector<Vec3> m_points;
    std::unordered_map<Key, std::vector<int>, KeyHash> m_grid;
};

/// Whether b stands in a straight run from a to c.
bool inStraightRun(const Vec3& a, const Vec3& b, const Vec3& c, double tol) {
    const Vec3 ac = c - a;
    const double len = ac.length();
    if (len <= tol) return false;
    return (b - a).cross(ac).length() / len <= tol && (b - a).dot(c - b) > 0.0;
}

/// Drop vertices that stand in a straight run between their neighbours, a
/// whole pass at a time.
void removeCollinear(Loop& loop, double tol) {
    bool changed = true;
    while (changed && loop.size() > 3) {
        changed = false;
        const size_t n = loop.size();
        Loop kept;
        kept.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            const Vec3& before = kept.empty() ? loop[n - 1] : kept.back();
            if (inStraightRun(before, loop[i], loop[(i + 1) % n], tol)) {
                changed = true;
                continue;
            }
            kept.push_back(loop[i]);
        }
        if (kept.size() < 3) return;
        loop = std::move(kept);
    }
}

using EdgeCounts = std::map<std::pair<int, int>, int>;

/// Split what is left unmatched at the loose ends lying on it. Once shared
/// edges have cancelled, what remains is the boundary plus the edges the CSG
/// split on one side only — a->b here, a->c->b on the other (a T-junction) —
/// and splitting a->b at c cancels those too. Only the ends of edges still
/// unmatched can be such a c, so the search stays among the boundary, not the
/// whole face; a grid over those ends keeps each edge's search local.
template <typename AddEdge>
void resolveTJunctions(EdgeCounts& open, const std::vector<Vec3>& pts, double tol,
                       const AddEdge& addEdge) {
    for (int round = 0; round < 3; ++round) {
        std::vector<std::pair<int, int>> edges;
        std::vector<int> ends;
        double total = 0.0;
        for (const auto& [edge, count] : open) {
            for (int k = 0; k < count; ++k) edges.push_back(edge);
            ends.push_back(edge.first);
            ends.push_back(edge.second);
            total += (pts[static_cast<size_t>(edge.second)] - pts[static_cast<size_t>(edge.first)])
                         .length();
        }
        if (edges.empty()) return;
        std::sort(ends.begin(), ends.end());
        ends.erase(std::unique(ends.begin(), ends.end()), ends.end());

        const double cell = std::max(total / static_cast<double>(edges.size()), tol * 4.0);
        const auto cellOf = [cell](double v) { return static_cast<int64_t>(std::floor(v / cell)); };
        std::map<std::tuple<int64_t, int64_t, int64_t>, std::vector<int>> grid;
        for (const int id : ends) {
            const Vec3& p = pts[static_cast<size_t>(id)];
            grid[{cellOf(p.x), cellOf(p.y), cellOf(p.z)}].push_back(id);
        }

        bool changed = false;
        for (const auto& edge : edges) {
            const auto it = open.find(edge);
            if (it == open.end()) continue;  // cancelled by a split made meanwhile
            // Plain copies, not a structured binding: the lambda below uses
            // them, and clang before 16 cannot capture a binding.
            const int a = edge.first;
            const int b = edge.second;
            const Vec3& pa = pts[static_cast<size_t>(a)];
            const Vec3 ab = pts[static_cast<size_t>(b)] - pa;
            const double len2 = ab.dot(ab);
            if (len2 <= 0.0) continue;

            std::vector<std::pair<double, int>> on;
            const auto consider = [&](int c) {
                if (c == a || c == b) return;
                const Vec3 ac = pts[static_cast<size_t>(c)] - pa;
                const double t = ac.dot(ab) / len2;
                if (t > 0.0 && t < 1.0 && (ac - ab * t).length() <= tol) on.emplace_back(t, c);
            };
            const Vec3& pb = pts[static_cast<size_t>(b)];
            const int64_t x0 = cellOf(std::min(pa.x, pb.x) - tol),
                          x1 = cellOf(std::max(pa.x, pb.x) + tol);
            const int64_t y0 = cellOf(std::min(pa.y, pb.y) - tol),
                          y1 = cellOf(std::max(pa.y, pb.y) + tol);
            const int64_t z0 = cellOf(std::min(pa.z, pb.z) - tol),
                          z1 = cellOf(std::max(pa.z, pb.z) + tol);
            const double cells = static_cast<double>(x1 - x0 + 1) *
                                 static_cast<double>(y1 - y0 + 1) *
                                 static_cast<double>(z1 - z0 + 1);
            if (cells > static_cast<double>(ends.size())) {
                for (const int c : ends) consider(c);  // a long edge: the list is shorter
            } else {
                for (int64_t x = x0; x <= x1; ++x) {
                    for (int64_t y = y0; y <= y1; ++y) {
                        for (int64_t z = z0; z <= z1; ++z) {
                            const auto found = grid.find({x, y, z});
                            if (found == grid.end()) continue;
                            for (const int c : found->second) consider(c);
                        }
                    }
                }
            }
            if (on.empty()) continue;

            if (--it->second == 0) open.erase(it);
            std::sort(on.begin(), on.end());
            int from = a;
            for (const auto& [t, c] : on) {
                addEdge(from, c);
                from = c;
            }
            addEdge(from, b);
            changed = true;
        }
        if (!changed) return;
    }
}

/// A set of coplanar fragments merged: the loops that bound it, split into
/// outer boundaries (wound as the fragments are) and holes (wound the other
/// way). `ok` is false when the boundary does not chain into simple loops.
struct Region {
    bool ok = false;
    std::vector<Loop> outer;
    std::vector<Loop> holes;
};

Region mergeRegion(const std::vector<Loop>& polys, const Vec3& normal, double tol) {
    Region region;
    Welder welder(tol);
    std::vector<std::vector<int>> loops;
    loops.reserve(polys.size());
    for (const auto& poly : polys) {
        std::vector<int> ids;
        for (const auto& p : poly) {
            const int id = welder.index(p);
            if (ids.empty() || ids.back() != id) ids.push_back(id);
        }
        while (ids.size() > 1 && ids.front() == ids.back()) ids.pop_back();
        if (ids.size() >= 3) loops.push_back(std::move(ids));
    }
    const std::vector<Vec3>& pts = welder.points();

    // Every edge cancels against the same edge run the other way; what is
    // left, once the T-junctions are split, is the boundary of the whole.
    EdgeCounts open;
    const auto addEdge = [&open](int a, int b) {
        const auto reverse = open.find({b, a});
        if (reverse != open.end()) {
            if (--reverse->second == 0) open.erase(reverse);
            return;
        }
        ++open[{a, b}];
    };
    for (const auto& loop : loops) {
        for (size_t i = 0; i < loop.size(); ++i) addEdge(loop[i], loop[(i + 1) % loop.size()]);
    }
    resolveTJunctions(open, pts, tol, addEdge);

    std::map<int, int> next;
    std::set<int> targets;
    for (const auto& [edge, count] : open) {
        if (count != 1 || next.count(edge.first) || targets.count(edge.second)) {
            return region;  // a pinch or an overlap: not simple loops
        }
        next[edge.first] = edge.second;
        targets.insert(edge.second);
    }

    std::set<int> visited;
    for (const auto& entry : next) {
        const int start = entry.first;
        if (visited.count(start)) continue;
        Loop loop;
        int at = start;
        do {
            if (!visited.insert(at).second) return region;  // not a loop
            loop.push_back(pts[static_cast<size_t>(at)]);
            const auto it = next.find(at);
            if (it == next.end()) return region;
            at = it->second;
        } while (at != start);
        removeCollinear(loop, tol);
        if (loop.size() < 3) continue;
        (newell(loop).dot(normal) > 0.0 ? region.outer : region.holes).push_back(std::move(loop));
    }
    region.ok = true;
    return region;
}

/// Split a convex loop by the plane {x : side·x = offset}: the part in front
/// (side·x > offset) and the part behind. Either may be empty.
std::pair<Loop, Loop> splitConvex(const Loop& loop, const Vec3& side, double offset, double tol) {
    Loop front;
    Loop back;
    const size_t n = loop.size();
    for (size_t i = 0; i < n; ++i) {
        const Vec3& a = loop[i];
        const Vec3& b = loop[(i + 1) % n];
        const double da = side.dot(a) - offset;
        const double db = side.dot(b) - offset;
        if (da >= -tol) front.push_back(a);
        if (da <= tol) back.push_back(a);
        if ((da > tol && db < -tol) || (da < -tol && db > tol)) {
            const Vec3 x = a + (b - a) * (da / (da - db));
            front.push_back(x);
            back.push_back(x);
        }
    }
    const auto area = [](const Loop& l) { return l.size() >= 3 ? newell(l).length() : 0.0; };
    if (area(front) <= tol * tol) front.clear();
    if (area(back) <= tol * tol) back.clear();
    return {std::move(front), std::move(back)};
}

/// Merge one face's coplanar fragments into the fewest simple loops, cutting
/// through any hole; nullopt when they do not merge cleanly.
std::optional<std::vector<Loop>> mergeGroup(const std::vector<Loop>& polys, const Vec3& normal,
                                            double tol) {
    Region whole = mergeRegion(polys, normal, tol);
    if (!whole.ok) return std::nullopt;
    if (whole.holes.empty()) return whole.outer;

    // Cut along the region's longest outer edge direction, through each hole.
    Vec3 along(0, 0, 0);
    double longest = 0.0;
    for (const auto& loop : whole.outer) {
        for (size_t i = 0; i < loop.size(); ++i) {
            const Vec3 e = loop[(i + 1) % loop.size()] - loop[i];
            if (e.length() > longest) {
                longest = e.length();
                along = e;
            }
        }
    }
    const Vec3 side = normal.cross(along).normalized();

    // Which side of each cut a piece is on, a bit for each hole: as many
    // words as it takes (one word allowed 60 holes, and a plate with more
    // stayed in fragments).
    using Cell = std::vector<uint64_t>;
    const size_t words = (whole.holes.size() + 63) / 64;
    struct Piece {
        Loop loop;
        Cell cell;
    };
    std::vector<Piece> pieces;
    pieces.reserve(polys.size());
    for (const auto& poly : polys) pieces.push_back({poly, Cell(words, 0)});
    for (size_t k = 0; k < whole.holes.size(); ++k) {
        Vec3 middle(0, 0, 0);
        for (const auto& p : whole.holes[k]) middle = middle + p;
        middle = middle * (1.0 / static_cast<double>(whole.holes[k].size()));
        const double offset = side.dot(middle);
        std::vector<Piece> cut;
        cut.reserve(pieces.size() * 2);
        for (const auto& piece : pieces) {
            auto [front, back] = splitConvex(piece.loop, side, offset, tol);
            if (!front.empty()) {
                Cell cell = piece.cell;
                cell[k / 64] |= uint64_t{1} << (k % 64);
                cut.push_back({std::move(front), std::move(cell)});
            }
            if (!back.empty()) cut.push_back({std::move(back), piece.cell});
        }
        pieces = std::move(cut);
    }

    std::map<Cell, std::vector<Loop>> cells;
    for (auto& piece : pieces) cells[piece.cell].push_back(std::move(piece.loop));
    std::vector<Loop> merged;
    for (const auto& entry : cells) {
        Region part = mergeRegion(entry.second, normal, tol);
        if (!part.ok || !part.holes.empty()) return std::nullopt;
        for (auto& loop : part.outer) merged.push_back(std::move(loop));
    }
    return merged;
}

}  // namespace

std::vector<CsgPolygon> mergeFragments(std::vector<CsgPolygon> fragments, double tol) {
    std::vector<CsgPolygon> out;
    out.reserve(fragments.size());

    // Group by source face, in a fixed order so the output (and the piece
    // names given from it) does not vary from run to run.
    std::map<std::pair<std::string, bool>, std::vector<size_t>> byFace;
    for (size_t i = 0; i < fragments.size(); ++i) {
        if (!fragments[i].topoId.isValid()) {
            out.push_back(std::move(fragments[i]));
            continue;
        }
        byFace[{fragments[i].topoId.tag(), fragments[i].fromA}].push_back(i);
    }

    for (const auto& entry : byFace) {
        const std::vector<size_t>& members = entry.second;
        // One face's fragments, by the plane they lie in (a faceted face's
        // fragments share its name across several planes).
        struct Plane {
            Vec3 normal;
            Vec3 point;  ///< a point of it: offsets n·p differ by the normals'
                         ///< rounding times the distance from the origin
            std::vector<size_t> members;
        };
        std::vector<Plane> planes;
        for (const size_t i : members) {
            const Loop& pts = fragments[i].points;
            const Vec3 n = newell(pts);
            const double len = n.length();
            if (pts.size() < 3 || len <= tol * tol) {
                out.push_back(std::move(fragments[i]));  // a sliver: leave it be
                continue;
            }
            const Vec3 unit = n * (1.0 / len);
            auto it = std::find_if(planes.begin(), planes.end(), [&](const Plane& p) {
                return p.normal.dot(unit) > 1.0 - 1e-9 &&
                       std::abs(p.normal.dot(pts[0] - p.point)) <= tol;
            });
            if (it == planes.end()) {
                planes.push_back({unit, pts[0], {}});
                it = planes.end() - 1;
            }
            it->members.push_back(i);
        }

        for (const auto& plane : planes) {
            if (plane.members.size() == 1) {
                out.push_back(std::move(fragments[plane.members[0]]));
                continue;
            }
            std::vector<Loop> polys;
            polys.reserve(plane.members.size());
            for (const size_t i : plane.members) polys.push_back(fragments[i].points);
            const auto merged = mergeGroup(polys, plane.normal, tol);
            if (!merged) {
                for (const size_t i : plane.members) out.push_back(std::move(fragments[i]));
                continue;
            }
            const CsgPolygon& source = fragments[plane.members[0]];
            for (const auto& loop : *merged) {
                CsgPolygon polygon;
                polygon.points = loop;
                polygon.topoId = source.topoId;
                polygon.fromA = source.fromA;
                out.push_back(std::move(polygon));
            }
        }
    }
    return out;
}

}  // namespace hz::model
