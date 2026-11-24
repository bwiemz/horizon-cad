#include "horizon/geometry/Triangulate2D.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "horizon/math/RobustPredicates.h"

namespace hz::geo {

namespace {

using math::Vec2;

constexpr int kNone = -1;

struct Tri {
    std::array<int, 3> v{};                     // counter-clockwise
    std::array<int, 3> n{kNone, kNone, kNone};  // n[i] is across the edge opposite v[i]
    bool alive = true;
};

/// An incremental triangulation: Bowyer-Watson insertion, constraint recovery by flips, then
/// Lawson flips. Vertex ids index m_pos; the last three are the super-triangle.
class Cdt {
public:
    Cdt(std::vector<Vec2> positions, const Vec2& lo, const Vec2& hi) : m_pos(std::move(positions)) {
        m_first_super = static_cast<int>(m_pos.size());
        const double extent = std::max({hi.x - lo.x, hi.y - lo.y, 1e-300});
        const double m = 1e4 * extent;
        const Vec2 c{(lo.x + hi.x) / 2.0, (lo.y + hi.y) / 2.0};
        m_pos.emplace_back(c.x - 3.0 * m, c.y - m);
        m_pos.emplace_back(c.x + 3.0 * m, c.y - m);
        m_pos.emplace_back(c.x, c.y + 3.0 * m);
        m_vtri.assign(m_pos.size(), kNone);
        Tri t;
        t.v = {m_first_super, m_first_super + 1, m_first_super + 2};
        m_tris.push_back(t);
        m_stamp.push_back(0);
        for (const int v : t.v) {
            m_vtri[static_cast<std::size_t>(v)] = 0;
        }
        m_last = 0;
    }

    void insert(int p) {
        const int start = locate(p);
        ++m_epoch;
        std::vector<int> cavity{start};
        m_stamp[static_cast<std::size_t>(start)] = m_epoch;
        for (std::size_t i = 0; i < cavity.size(); ++i) {
            const Tri& t = tri(cavity[i]);
            for (int k = 0; k < 3; ++k) {
                const int nb = t.n[static_cast<std::size_t>(k)];
                if (nb == kNone || m_stamp[static_cast<std::size_t>(nb)] == m_epoch) {
                    continue;
                }
                const Tri& u = tri(nb);
                if (math::incircle(pos(u.v[0]), pos(u.v[1]), pos(u.v[2]), pos(p)) > 0) {
                    m_stamp[static_cast<std::size_t>(nb)] = m_epoch;
                    cavity.push_back(nb);
                }
            }
        }
        struct Rim {
            int a;
            int b;
            int outside;
        };
        std::vector<Rim> rim;
        for (const int ti : cavity) {
            const Tri& t = tri(ti);
            for (int k = 0; k < 3; ++k) {
                const int nb = t.n[static_cast<std::size_t>(k)];
                if (nb == kNone || m_stamp[static_cast<std::size_t>(nb)] != m_epoch) {
                    rim.push_back({t.v[static_cast<std::size_t>((k + 1) % 3)],
                                   t.v[static_cast<std::size_t>((k + 2) % 3)], nb});
                }
            }
        }
        for (const int ti : cavity) {
            tri(ti).alive = false;
            m_free.push_back(ti);
        }
        std::unordered_map<int, int> by_start;
        std::unordered_map<int, int> by_end;
        std::vector<int> made;
        for (const Rim& r : rim) {
            const int id = allocate();
            Tri& t = tri(id);
            t.v = {r.a, r.b, p};
            t.n = {kNone, kNone, r.outside};
            by_start[r.a] = id;
            by_end[r.b] = id;
            made.push_back(id);
            if (r.outside != kNone) {
                Tri& o = tri(r.outside);
                for (int k = 0; k < 3; ++k) {
                    if (o.v[static_cast<std::size_t>((k + 1) % 3)] == r.b &&
                        o.v[static_cast<std::size_t>((k + 2) % 3)] == r.a) {
                        o.n[static_cast<std::size_t>(k)] = id;
                    }
                }
            }
        }
        for (const int id : made) {
            Tri& t = tri(id);
            t.n[0] = by_start.at(t.v[1]);  // across (b, p)
            t.n[1] = by_end.at(t.v[0]);    // across (p, a)
            for (const int v : t.v) {
                m_vtri[static_cast<std::size_t>(v)] = id;
            }
        }
        m_last = made.front();
    }

    /// Makes a-b an edge and marks it constrained. False when it cannot be (the segment passes
    /// through another vertex or crosses another constraint).
    bool constrain(int a, int b) {
        if (a == b) {
            return true;
        }
        if (findEdge(a, b).first == kNone && !recover(a, b)) {
            return false;
        }
        m_constrained.insert(key(a, b));
        return true;
    }

    void lawson() {
        std::vector<std::pair<int, int>> stack;
        for (std::size_t t = 0; t < m_tris.size(); ++t) {
            if (m_tris[t].alive) {
                for (int k = 0; k < 3; ++k) {
                    stack.emplace_back(static_cast<int>(t), k);
                }
            }
        }
        while (!stack.empty()) {
            const auto [ti, k] = stack.back();
            stack.pop_back();
            const Tri& t = tri(ti);
            const int nb = t.n[static_cast<std::size_t>(k)];
            if (!t.alive || nb == kNone) {
                continue;
            }
            if (isConstrained(t.v[static_cast<std::size_t>((k + 1) % 3)],
                              t.v[static_cast<std::size_t>((k + 2) % 3)])) {
                continue;
            }
            const Tri& u = tri(nb);
            const int w = u.v[static_cast<std::size_t>(opposite(u, ti))];
            if (math::incircle(pos(t.v[0]), pos(t.v[1]), pos(t.v[2]), pos(w)) > 0) {
                flip(ti, k);
                for (int e = 0; e < 3; ++e) {
                    stack.emplace_back(ti, e);
                    stack.emplace_back(nb, e);
                }
            }
        }
    }

    /// The triangles inside the region: those an odd number of constraint edges away from the
    /// super-triangle, as a flood fill that flips at each constraint edge.
    std::vector<int> insideTriangles() const {
        std::vector<int> parity(m_tris.size(), kNone);
        std::vector<int> queue;
        for (std::size_t t = 0; t < m_tris.size(); ++t) {
            if (m_tris[t].alive && touchesSuper(m_tris[t])) {
                parity[t] = 0;
                queue.push_back(static_cast<int>(t));
            }
        }
        for (std::size_t i = 0; i < queue.size(); ++i) {
            const Tri& t = tri(queue[i]);
            for (int k = 0; k < 3; ++k) {
                const int nb = t.n[static_cast<std::size_t>(k)];
                if (nb == kNone || parity[static_cast<std::size_t>(nb)] != kNone) {
                    continue;
                }
                const bool wall = isConstrained(t.v[static_cast<std::size_t>((k + 1) % 3)],
                                                t.v[static_cast<std::size_t>((k + 2) % 3)]);
                parity[static_cast<std::size_t>(nb)] =
                    parity[static_cast<std::size_t>(queue[i])] ^ (wall ? 1 : 0);
                queue.push_back(nb);
            }
        }
        std::vector<int> inside;
        for (std::size_t t = 0; t < m_tris.size(); ++t) {
            if (m_tris[t].alive && parity[t] == 1 && !touchesSuper(m_tris[t])) {
                inside.push_back(static_cast<int>(t));
            }
        }
        return inside;
    }

    const Tri& triangle(int i) const { return tri(i); }

private:
    const Vec2& pos(int v) const { return m_pos[static_cast<std::size_t>(v)]; }
    Tri& tri(int i) { return m_tris[static_cast<std::size_t>(i)]; }
    const Tri& tri(int i) const { return m_tris[static_cast<std::size_t>(i)]; }
    int orient(int a, int b, int c) const { return math::orient2d(pos(a), pos(b), pos(c)); }
    bool touchesSuper(const Tri& t) const {
        return t.v[0] >= m_first_super || t.v[1] >= m_first_super || t.v[2] >= m_first_super;
    }
    static std::uint64_t key(int a, int b) {
        const auto lo = static_cast<std::uint64_t>(std::min(a, b));
        const auto hi = static_cast<std::uint64_t>(std::max(a, b));
        return (lo << 32) | hi;
    }
    bool isConstrained(int a, int b) const { return m_constrained.count(key(a, b)) != 0; }

    static int indexOf(const Tri& t, int v) {
        return t.v[0] == v ? 0 : (t.v[1] == v ? 1 : (t.v[2] == v ? 2 : kNone));
    }
    /// The index in u of the neighbour that is triangle ti.
    static int opposite(const Tri& u, int ti) { return u.n[0] == ti ? 0 : (u.n[1] == ti ? 1 : 2); }

    int allocate() {
        if (!m_free.empty()) {
            const int id = m_free.back();
            m_free.pop_back();
            tri(id) = Tri{};
            return id;
        }
        m_tris.emplace_back();
        m_stamp.push_back(0);
        return static_cast<int>(m_tris.size()) - 1;
    }

    /// A triangle containing p (on its boundary included), by walking from the last one.
    int locate(int p) {
        int t = m_last;
        if (!tri(t).alive) {
            t = firstAlive();
        }
        const std::size_t limit = 4 * m_tris.size() + 16;
        for (std::size_t step = 0; step < limit; ++step) {
            const Tri& cur = tri(t);
            bool moved = false;
            for (int j = 0; j < 3 && !moved; ++j) {
                const int k = static_cast<int>((step + static_cast<std::size_t>(j)) % 3);
                const int nb = cur.n[static_cast<std::size_t>(k)];
                if (nb != kNone && orient(cur.v[static_cast<std::size_t>((k + 1) % 3)],
                                          cur.v[static_cast<std::size_t>((k + 2) % 3)], p) < 0) {
                    t = nb;
                    moved = true;
                }
            }
            if (!moved) {
                return t;
            }
        }
        for (std::size_t i = 0; i < m_tris.size(); ++i) {  // a walk that cycles: scan
            const Tri& cur = m_tris[i];
            if (cur.alive && orient(cur.v[0], cur.v[1], p) >= 0 &&
                orient(cur.v[1], cur.v[2], p) >= 0 && orient(cur.v[2], cur.v[0], p) >= 0) {
                return static_cast<int>(i);
            }
        }
        return firstAlive();
    }

    int firstAlive() const {
        for (std::size_t i = 0; i < m_tris.size(); ++i) {
            if (m_tris[i].alive) {
                return static_cast<int>(i);
            }
        }
        return 0;
    }

    /// Every triangle around vertex p.
    std::vector<int> fan(int p) const {
        std::vector<int> out;
        const int start = m_vtri[static_cast<std::size_t>(p)];
        if (start == kNone) {
            return out;
        }
        int t = start;
        do {
            out.push_back(t);
            t = tri(t).n[static_cast<std::size_t>((indexOf(tri(t), p) + 1) % 3)];
        } while (t != kNone && t != start);
        if (t == kNone) {  // an open fan: the other way round
            t = tri(start).n[static_cast<std::size_t>((indexOf(tri(start), p) + 2) % 3)];
            while (t != kNone) {
                out.push_back(t);
                t = tri(t).n[static_cast<std::size_t>((indexOf(tri(t), p) + 2) % 3)];
            }
        }
        return out;
    }

    /// A triangle with a-b as an edge, and the index of its vertex opposite that edge.
    std::pair<int, int> findEdge(int a, int b) const {
        for (const int t : fan(a)) {
            const int kb = indexOf(tri(t), b);
            if (kb != kNone) {
                return {t, 3 - kb - indexOf(tri(t), a)};
            }
        }
        return {kNone, kNone};
    }

    /// Replaces the diagonal shared by triangle ti and its neighbour across index i.
    void flip(int ti, int i) {
        Tri& t = tri(ti);
        const int ui = t.n[static_cast<std::size_t>(i)];
        Tri& u = tri(ui);
        const auto si = static_cast<std::size_t>(i);
        const int j = opposite(u, ti);
        const auto sj = static_cast<std::size_t>(j);
        const int a = t.v[si];
        const int b = t.v[(si + 1) % 3];
        const int c = t.v[(si + 2) % 3];
        const int w = u.v[sj];
        const int n_b = t.n[(si + 1) % 3];  // across (c, a)
        const int n_c = t.n[(si + 2) % 3];  // across (a, b)
        const int u_c = u.n[(sj + 1) % 3];  // across (b, w)
        const int u_b = u.n[(sj + 2) % 3];  // across (w, c)
        t.v = {a, b, w};
        t.n = {u_c, ui, n_c};
        u.v = {a, w, c};
        u.n = {u_b, n_b, ti};
        relink(u_c, ui, ti);
        relink(n_b, ti, ui);
        for (const int v : t.v) {
            m_vtri[static_cast<std::size_t>(v)] = ti;
        }
        for (const int v : u.v) {
            m_vtri[static_cast<std::size_t>(v)] = ui;
        }
    }

    void relink(int nb, int from, int to) {
        if (nb == kNone) {
            return;
        }
        for (int& x : tri(nb).n) {
            if (x == from) {
                x = to;
                return;
            }
        }
    }

    /// Flips the edges that cross the segment a-b until it is an edge.
    bool recover(int a, int b) {
        int start = kNone;
        int start_index = kNone;
        for (const int t : fan(a)) {
            const int k = indexOf(tri(t), a);
            const int c = tri(t).v[static_cast<std::size_t>((k + 1) % 3)];
            const int d = tri(t).v[static_cast<std::size_t>((k + 2) % 3)];
            const int o1 = orient(a, c, b);
            const int o2 = orient(a, d, b);
            if ((o1 == 0 && alongSegment(a, c, b)) || (o2 == 0 && alongSegment(a, d, b))) {
                return false;  // through a vertex
            }
            if (o1 > 0 && o2 < 0) {
                start = t;
                start_index = k;
            }
        }
        if (start == kNone) {
            return false;
        }
        std::vector<std::pair<int, int>> crossed;
        int p = tri(start).v[static_cast<std::size_t>((start_index + 1) % 3)];
        int q = tri(start).v[static_cast<std::size_t>((start_index + 2) % 3)];
        int cur = tri(start).n[static_cast<std::size_t>(start_index)];
        for (std::size_t guard = 0; guard <= m_tris.size(); ++guard) {
            if (isConstrained(p, q) || cur == kNone) {
                return false;
            }
            crossed.emplace_back(p, q);
            const Tri& t = tri(cur);
            int e = kNone;
            for (const int v : t.v) {
                if (v != p && v != q) {
                    e = v;
                }
            }
            if (e == b) {
                break;
            }
            const int s = orient(a, b, e);
            if (s == 0) {
                return false;  // through a vertex
            }
            int left = q;  // the vertex that leaves the crossed edge
            if (s == orient(a, b, p)) {
                left = p;
                p = e;
            } else {
                q = e;
            }
            cur = t.n[static_cast<std::size_t>(indexOf(t, left))];
        }
        std::vector<std::pair<int, int>> queue = crossed;
        const std::size_t cap = 8 * crossed.size() * crossed.size() + 64;
        for (std::size_t head = 0, steps = 0; head < queue.size(); ++head, ++steps) {
            if (steps > cap) {
                return false;
            }
            const auto [x, y] = queue[head];
            const auto [ti, i] = findEdge(x, y);
            if (ti == kNone) {
                return false;
            }
            const int ui = tri(ti).n[static_cast<std::size_t>(i)];
            const int apex_t = tri(ti).v[static_cast<std::size_t>(i)];
            const int apex_u = tri(ui).v[static_cast<std::size_t>(opposite(tri(ui), ti))];
            if (orient(apex_t, apex_u, x) * orient(apex_t, apex_u, y) < 0) {
                flip(ti, i);
                if (orient(a, b, apex_t) * orient(a, b, apex_u) < 0) {
                    queue.emplace_back(apex_t, apex_u);
                }
            } else {
                queue.emplace_back(x, y);
            }
        }
        return findEdge(a, b).first != kNone;
    }

    bool alongSegment(int a, int c, int b) const {
        const Vec2 ab = pos(b) - pos(a);
        const Vec2 ac = pos(c) - pos(a);
        return ab.x * ac.x + ab.y * ac.y > 0.0;
    }

    std::vector<Vec2> m_pos;
    std::vector<Tri> m_tris;
    std::vector<int> m_vtri;
    std::vector<int> m_free;
    std::vector<int> m_stamp;
    std::unordered_set<std::uint64_t> m_constrained;
    int m_first_super = 0;
    int m_last = 0;
    int m_epoch = 0;
};

/// Finds an earlier point within a distance, by grid cells of that size.
class PointGrid {
public:
    explicit PointGrid(double merge) : m_merge(merge), m_cell(merge > 0.0 ? merge : 1.0) {}

    /// The index of a point within merge of p, or -1.
    int find(const Vec2& p) const {
        const auto [cx, cy] = cell(p);
        for (std::int64_t dx = -1; dx <= 1; ++dx) {
            for (std::int64_t dy = -1; dy <= 1; ++dy) {
                const auto it = m_cells.find({cx + dx, cy + dy});
                if (it == m_cells.end()) {
                    continue;
                }
                for (const auto& [q, index] : it->second) {
                    if ((q - p).length() <= m_merge) {
                        return index;
                    }
                }
            }
        }
        return -1;
    }

    void add(const Vec2& p, int index) { m_cells[cell(p)].emplace_back(p, index); }

private:
    std::pair<std::int64_t, std::int64_t> cell(const Vec2& p) const {
        const auto to_cell = [this](double v) {
            return static_cast<std::int64_t>(std::clamp(std::floor(v / m_cell), -9e18, 9e18));
        };
        return {to_cell(p.x), to_cell(p.y)};
    }

    double m_merge;
    double m_cell;
    std::map<std::pair<std::int64_t, std::int64_t>, std::vector<std::pair<Vec2, int>>> m_cells;
};

std::string loopName(std::size_t loop) {
    return loop == 0 ? std::string("outer") : "hole " + std::to_string(loop - 1);
}

std::size_t loopOf(const std::vector<std::size_t>& starts, std::size_t index) {
    std::size_t li = 0;
    while (li + 1 < starts.size() && starts[li + 1] <= index) {
        ++li;
    }
    return li;
}

std::string pointName(std::size_t loop, std::size_t i) {
    return loopName(loop) + " point " + std::to_string(i);
}

std::string segmentName(std::size_t loop, std::size_t i, std::size_t loop_size) {
    return loopName(loop) + " segment " + std::to_string(i) + " (points " + std::to_string(i) +
           "-" + std::to_string((i + 1) % loop_size) + ")";
}

/// A segment that crosses, or has an endpoint on, segment (loop, i): named for the message.
std::string partnerOf(const std::vector<Vec2>& pts, const std::vector<int>& rep,
                      const std::vector<std::size_t>& starts,
                      const std::vector<const std::vector<Vec2>*>& loops, std::size_t loop,
                      std::size_t i) {
    const auto endpoints = [&](std::size_t l, std::size_t k) {
        const std::size_t n = loops[l]->size();
        return std::pair<const Vec2&, const Vec2&>(
            pts[static_cast<std::size_t>(rep[starts[l] + k])],
            pts[static_cast<std::size_t>(rep[starts[l] + (k + 1) % n])]);
    };
    const auto [a, b] = endpoints(loop, i);
    const auto between = [](const Vec2& p, const Vec2& q, const Vec2& r) {  // r on p-q's box
        return std::min(p.x, q.x) <= r.x && r.x <= std::max(p.x, q.x) &&
               std::min(p.y, q.y) <= r.y && r.y <= std::max(p.y, q.y);
    };
    for (std::size_t l = 0; l < loops.size(); ++l) {
        for (std::size_t k = 0; k < loops[l]->size(); ++k) {
            if (l == loop && k == i) {
                continue;
            }
            const auto [c, d] = endpoints(l, k);
            const int o1 = math::orient2d(a, b, c);
            const int o2 = math::orient2d(a, b, d);
            const int o3 = math::orient2d(c, d, a);
            const int o4 = math::orient2d(c, d, b);
            const bool shares = (a.x == c.x && a.y == c.y) || (a.x == d.x && a.y == d.y) ||
                                (b.x == c.x && b.y == c.y) || (b.x == d.x && b.y == d.y);
            const bool crosses = o1 * o2 < 0 && o3 * o4 < 0;
            const bool on = (o1 == 0 && between(a, b, c)) || (o2 == 0 && between(a, b, d)) ||
                            (o3 == 0 && between(c, d, a)) || (o4 == 0 && between(c, d, b));
            if (crosses || (on && !shares)) {
                return std::string(crosses ? "it crosses " : "it touches ") + loopName(l) +
                       " segment " + std::to_string(k);
            }
        }
    }
    return {};
}

}  // namespace

Triangulation2D triangulateRegion(const std::vector<Vec2>& outer,
                                  const std::vector<std::vector<Vec2>>& holes,
                                  const std::vector<Vec2>& interior, double merge) {
    Triangulation2D result;
    const double tol = std::max(merge, 0.0);

    // Boundary points first, in input order; one that is within tol of an earlier one stands
    // for that one.
    std::vector<const std::vector<Vec2>*> loops{&outer};
    for (const auto& h : holes) {
        loops.push_back(&h);
    }
    PointGrid grid(tol);
    std::vector<int> rep;
    std::vector<std::size_t> loop_starts;  // the first boundary index of each loop
    for (const auto* loop : loops) {
        loop_starts.push_back(result.points.size());
        for (std::size_t i = 0; i < loop->size(); ++i) {
            const Vec2& p = (*loop)[i];
            const int index = static_cast<int>(result.points.size());
            const int earlier = grid.find(p);
            if (earlier >= 0 && result.problem.empty()) {
                const std::size_t li = loop_starts.size() - 1;
                const std::size_t ej = loopOf(loop_starts, static_cast<std::size_t>(earlier));
                if (ej != li) {
                    result.problem =
                        pointName(li, i) + " coincides with " +
                        pointName(ej, static_cast<std::size_t>(earlier) - loop_starts[ej]) +
                        ": the loops touch";
                }
            }
            rep.push_back(earlier >= 0 ? earlier : index);
            if (earlier < 0) {
                grid.add(p, index);
            }
            result.points.push_back(p);
        }
    }
    const std::size_t boundary_count = result.points.size();

    // Interior points that are strictly inside the region and clear of every other point.
    std::vector<int> inserted;
    for (std::size_t i = 0; i < boundary_count; ++i) {
        if (rep[i] == static_cast<int>(i)) {
            inserted.push_back(static_cast<int>(i));
        }
    }
    for (const Vec2& p : interior) {
        if (grid.find(p) >= 0 || math::classifyPoint(outer, p) != math::Containment::Inside) {
            continue;
        }
        const bool in_hole = std::any_of(holes.begin(), holes.end(), [&p](const auto& h) {
            return math::classifyPoint(h, p) != math::Containment::Outside;
        });
        if (in_hole) {
            continue;
        }
        const int index = static_cast<int>(result.points.size());
        grid.add(p, index);
        result.points.push_back(p);
        inserted.push_back(index);
    }
    if (inserted.size() < 3) {
        return result;
    }

    Vec2 lo = result.points[static_cast<std::size_t>(inserted.front())];
    Vec2 hi = lo;
    for (const int i : inserted) {
        const Vec2& p = result.points[static_cast<std::size_t>(i)];
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y)};
    }
    Cdt cdt(result.points, lo, hi);
    for (const int i : inserted) {
        cdt.insert(i);
    }
    for (std::size_t li = 0; li < loops.size(); ++li) {
        const std::size_t base = loop_starts[li];
        const std::size_t n = loops[li]->size();
        for (std::size_t i = 0; i < n; ++i) {
            const int a = rep[base + i];
            const int b = rep[base + (i + 1) % n];
            if (!cdt.constrain(a, b) && result.problem.empty()) {
                const std::string partner =
                    partnerOf(result.points, rep, loop_starts, loops, li, i);
                result.problem = segmentName(li, i, n) + " cannot be kept: " +
                                 (partner.empty() ? "the loops cross or touch" : partner);
            }
        }
    }
    cdt.lawson();

    for (const int t : cdt.insideTriangles()) {
        const auto& v = cdt.triangle(t).v;
        result.triangles.push_back({static_cast<std::uint32_t>(v[0]),
                                    static_cast<std::uint32_t>(v[1]),
                                    static_cast<std::uint32_t>(v[2])});
    }
    return result;
}

}  // namespace hz::geo
