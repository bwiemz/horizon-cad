#include "horizon/geometry/Triangulate2D.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "horizon/math/RobustPredicates.h"

namespace hz::geo {

namespace {

using math::Vec2;

constexpr int kNone = -1;

/// The widest a set of points may spread. The super-triangle reaches 3e4 extents out, and the
/// incircle predicate multiplies four coordinate differences together: beyond about 1e72 that
/// product passes the largest double and the exact predicates are no longer exact.
constexpr double kWidest = 1e70;

/// The narrowest non-zero spread. The incircle predicate is exact for coordinate differences
/// down to about 1e-70; the super-triangle's and the triangles' products need margin above it.
constexpr double kNarrowest = 1e-60;

/// The smallest nonzero coordinate magnitude kept (Ruling 4). Below it a predicate's sign can
/// come out wrong even where the differences are in range (math/RobustPredicates.h: incircle's
/// floor), so such a coordinate is flushed to exactly 0 before any predicate sees it.
constexpr double kFloor = 1e-60;

Vec2 flushed(const Vec2& p) {
    const auto flush = [](double c) { return std::fabs(c) < kFloor ? 0.0 : c; };
    return {flush(p.x), flush(p.y)};
}

std::vector<Vec2> flushed(const std::vector<Vec2>& ps) {
    std::vector<Vec2> out;
    out.reserve(ps.size());
    for (const Vec2& p : ps) {
        out.push_back(flushed(p));
    }
    return out;
}

std::uint64_t edgeKey(int a, int b) {
    const auto lo = static_cast<std::uint64_t>(std::min(a, b));
    const auto hi = static_cast<std::uint64_t>(std::max(a, b));
    return (lo << 32) | hi;
}

/// How constraint recovery ended.
enum class Recovery { Done, Blocked, GaveUp };

struct Tri {
    std::array<int, 3> v{};                     // counter-clockwise
    std::array<int, 3> n{kNone, kNone, kNone};  // n[i] is across the edge opposite v[i]
    bool alive = true;
    bool inside = false;  // in the region, once it is filled
};

/// An incremental triangulation: Bowyer-Watson insertion, constraint recovery by flips, then
/// Lawson flips; once the region is filled, more points by insertInside. Vertex ids index m_pos:
/// the points it was built with, the super-triangle's three, then the points inserted.
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
        const int start = locate(pos(p));
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
        std::vector<Rim> rim;
        rimOf(cavity, rim);
        fill(cavity, rim, p);
    }

    /// Adds @p q where it lies strictly inside a triangle of the region, or on an edge between
    /// two that is not constrained: splits the triangle (or both) and flips the edges across from
    /// it that are not Delaunay, never a constrained one, so the triangulation stays constrained
    /// Delaunay. The cavity of Bowyer-Watson could reach round a slit's free end to its far side;
    /// flips cannot. The new vertex, with the triangles made or altered in @p changed; or kNone
    /// with nothing changed where q lies outside the region, on a constraint or on a vertex.
    int insertInside(const Vec2& q, std::vector<int>& changed) {
        const int t = locate(q);
        const Tri& found = tri(t);
        if (!found.alive || !found.inside) {
            return kNone;
        }
        std::vector<int> cavity{t};
        for (int k = 0; k < 3; ++k) {
            const int a = found.v[static_cast<std::size_t>((k + 1) % 3)];
            const int b = found.v[static_cast<std::size_t>((k + 2) % 3)];
            const int s = math::orient2d(pos(a), pos(b), q);
            if (s < 0 || (s == 0 && (cavity.size() > 1 || isConstrained(a, b)))) {
                return kNone;  // outside it, on a vertex, or on a constraint
            }
            const int across = found.n[static_cast<std::size_t>(k)];
            if (s == 0) {
                if (across == kNone || !tri(across).inside) {
                    return kNone;
                }
                cavity.push_back(across);
            }
        }
        const int p = static_cast<int>(m_pos.size());
        m_pos.push_back(q);
        m_vtri.push_back(kNone);
        ++m_epoch;
        for (const int ti : cavity) {
            m_stamp[static_cast<std::size_t>(ti)] = m_epoch;
        }
        std::vector<Rim> rim;
        rimOf(cavity, rim);
        std::vector<int> stack = fill(cavity, rim, p);
        for (const int id : stack) {
            tri(id).inside = true;
        }
        changed = stack;
        while (!stack.empty()) {
            const int ti = stack.back();
            stack.pop_back();
            const Tri& cur = tri(ti);
            const int k = indexOf(cur, p);
            const int nb = cur.n[static_cast<std::size_t>(k)];
            if (nb == kNone || isConstrained(cur.v[static_cast<std::size_t>((k + 1) % 3)],
                                             cur.v[static_cast<std::size_t>((k + 2) % 3)])) {
                continue;
            }
            const Tri& u = tri(nb);
            const int w = u.v[static_cast<std::size_t>(opposite(u, ti))];
            if (math::incircle(pos(cur.v[0]), pos(cur.v[1]), pos(cur.v[2]), pos(w)) > 0) {
                flip(ti, k);  // both keep p, and both lie in the region
                changed.push_back(nb);
                stack.push_back(ti);
                stack.push_back(nb);
            }
        }
        return p;
    }

    /// The triangles of the region, once fillRegion has found them.
    void markInside(const std::vector<int>& inside) {
        for (const int t : inside) {
            tri(t).inside = true;
        }
    }

    /// The index in the points of vertex @p v, which is not the super-triangle's.
    int pointOf(int v) const { return v < m_first_super ? v : v - 3; }
    std::size_t triangleCount() const { return m_tris.size(); }

private:
    struct Rim {
        int a;
        int b;
        int outside;
    };

    /// The edges of the stamped cavity whose far side is not in it.
    void rimOf(const std::vector<int>& cavity, std::vector<Rim>& rim) const {
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
    }

    /// Replaces the cavity's triangles with one from each rim edge to @p p: the new triangles,
    /// each (a, b, p) with its rim edge's outer neighbour across from p.
    std::vector<int> fill(const std::vector<int>& cavity, const std::vector<Rim>& rim, int p) {
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
        return made;
    }

public:
    /// Makes a-b an edge and marks it constrained. Blocked when it cannot be (the segment
    /// passes through another vertex or crosses another constraint), GaveUp when the flips did
    /// not finish.
    Recovery constrain(int a, int b) {
        if (a == b) {
            return Recovery::Done;
        }
        if (findEdge(a, b).first == kNone) {
            const Recovery r = recover(a, b);
            if (r != Recovery::Done) {
                return r;
            }
        }
        m_constrained.insert(key(a, b));
        return Recovery::Done;
    }

    /// Makes a-b a wall of the region: crossing it flips inside and outside. A segment that a
    /// loop runs along twice (a slit, a bridge) stays an edge but is not a wall.
    void markWall(int a, int b) { m_walls.insert(key(a, b)); }

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

    struct Fill {
        std::vector<int> inside;
        /// The first edge across which the parities of two triangles disagree, if any.
        std::pair<int, int> conflict{kNone, kNone};
    };

    /// The triangles inside the region: those an odd number of walls away from the
    /// super-triangle, as a flood fill that flips at each wall. Where a neighbour is already
    /// labelled it checks the labels agree, which they do exactly when the region is well
    /// defined.
    Fill fillRegion() const {
        Fill fill;
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
                if (nb == kNone) {
                    continue;
                }
                const int ea = t.v[static_cast<std::size_t>((k + 1) % 3)];
                const int eb = t.v[static_cast<std::size_t>((k + 2) % 3)];
                const int expected =
                    parity[static_cast<std::size_t>(queue[i])] ^ (isWall(ea, eb) ? 1 : 0);
                int& label = parity[static_cast<std::size_t>(nb)];
                if (label == kNone) {
                    label = expected;
                    queue.push_back(nb);
                } else if (label != expected && fill.conflict.first == kNone) {
                    fill.conflict = {ea, eb};
                }
            }
        }
        for (std::size_t t = 0; t < m_tris.size(); ++t) {
            if (m_tris[t].alive && parity[t] == 1 && !touchesSuper(m_tris[t])) {
                fill.inside.push_back(static_cast<int>(t));
            }
        }
        return fill;
    }

    const Tri& triangle(int i) const { return tri(i); }

private:
    const Vec2& pos(int v) const { return m_pos[static_cast<std::size_t>(v)]; }
    Tri& tri(int i) { return m_tris[static_cast<std::size_t>(i)]; }
    const Tri& tri(int i) const { return m_tris[static_cast<std::size_t>(i)]; }
    int orient(int a, int b, int c) const { return math::orient2d(pos(a), pos(b), pos(c)); }
    bool isSuper(int v) const { return v >= m_first_super && v < m_first_super + 3; }
    bool touchesSuper(const Tri& t) const {
        return isSuper(t.v[0]) || isSuper(t.v[1]) || isSuper(t.v[2]);
    }
    static std::uint64_t key(int a, int b) { return edgeKey(a, b); }
    bool isConstrained(int a, int b) const { return m_constrained.count(key(a, b)) != 0; }
    bool isWall(int a, int b) const { return m_walls.count(key(a, b)) != 0; }

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
    int locate(const Vec2& p) {
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
                if (nb != kNone &&
                    math::orient2d(pos(cur.v[static_cast<std::size_t>((k + 1) % 3)]),
                                   pos(cur.v[static_cast<std::size_t>((k + 2) % 3)]), p) < 0) {
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
            if (cur.alive && math::orient2d(pos(cur.v[0]), pos(cur.v[1]), p) >= 0 &&
                math::orient2d(pos(cur.v[1]), pos(cur.v[2]), p) >= 0 &&
                math::orient2d(pos(cur.v[2]), pos(cur.v[0]), p) >= 0) {
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
    Recovery recover(int a, int b) {
        int start = kNone;
        int start_index = kNone;
        for (const int t : fan(a)) {
            const int k = indexOf(tri(t), a);
            const int c = tri(t).v[static_cast<std::size_t>((k + 1) % 3)];
            const int d = tri(t).v[static_cast<std::size_t>((k + 2) % 3)];
            const int o1 = orient(a, c, b);
            const int o2 = orient(a, d, b);
            if ((o1 == 0 && alongSegment(a, c, b)) || (o2 == 0 && alongSegment(a, d, b))) {
                return Recovery::Blocked;  // through a vertex
            }
            if (o1 > 0 && o2 < 0) {
                start = t;
                start_index = k;
            }
        }
        if (start == kNone) {
            return Recovery::GaveUp;
        }
        std::vector<std::pair<int, int>> crossed;
        int p = tri(start).v[static_cast<std::size_t>((start_index + 1) % 3)];
        int q = tri(start).v[static_cast<std::size_t>((start_index + 2) % 3)];
        int cur = tri(start).n[static_cast<std::size_t>(start_index)];
        for (std::size_t guard = 0; guard <= m_tris.size(); ++guard) {
            if (cur == kNone) {
                return Recovery::GaveUp;
            }
            if (isConstrained(p, q)) {
                return Recovery::Blocked;  // crosses a constraint
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
                return Recovery::Blocked;  // through a vertex
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
                return Recovery::GaveUp;
            }
            const auto [x, y] = queue[head];
            const auto [ti, i] = findEdge(x, y);
            if (ti == kNone) {
                return Recovery::GaveUp;
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
        return findEdge(a, b).first != kNone ? Recovery::Done : Recovery::GaveUp;
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
    std::unordered_set<std::uint64_t> m_walls;
    int m_first_super = 0;
    int m_last = 0;
    int m_epoch = 0;
};

/// Finds the earliest point within a distance of a given one. With a distance of 0 it hashes
/// the exact coordinates; otherwise it buckets points in cells of that size.
class PointGrid {
public:
    explicit PointGrid(double merge) : m_merge(merge), m_cell(merge > 0.0 ? merge : 1.0) {}

    /// The lowest index of a point within merge of p, or -1.
    int find(const Vec2& p) const {
        if (m_merge <= 0.0) {
            const auto it = m_cells.find(exactKey(p));
            return it == m_cells.end() ? -1 : it->second.front().second;
        }
        const auto [cx, cy] = cell(p);
        int best = -1;
        for (std::int64_t dx = -1; dx <= 1; ++dx) {
            for (std::int64_t dy = -1; dy <= 1; ++dy) {
                const auto it = m_cells.find({cx + dx, cy + dy});
                if (it == m_cells.end()) {
                    continue;
                }
                for (const auto& [q, index] : it->second) {
                    if ((q - p).length() <= m_merge && (best < 0 || index < best)) {
                        best = index;
                    }
                }
            }
        }
        return best;
    }

    void add(const Vec2& p, int index) {
        m_cells[m_merge <= 0.0 ? exactKey(p) : cell(p)].emplace_back(p, index);
    }

private:
    using Key = std::pair<std::int64_t, std::int64_t>;
    struct KeyHash {
        std::size_t operator()(const Key& k) const {
            auto h = static_cast<std::uint64_t>(k.first) * 0x9E3779B97F4A7C15ULL;
            h ^= static_cast<std::uint64_t>(k.second) + 0x9E3779B97F4A7C15ULL + (h << 6) + (h >> 2);
            return static_cast<std::size_t>(h ^ (h >> 29));
        }
    };

    static Key exactKey(const Vec2& p) {
        const double x = p.x + 0.0;  // -0.0 becomes 0.0
        const double y = p.y + 0.0;
        Key k;
        std::memcpy(&k.first, &x, sizeof(double));
        std::memcpy(&k.second, &y, sizeof(double));
        return k;
    }

    Key cell(const Vec2& p) const {
        const auto to_cell = [this](double v) {
            return static_cast<std::int64_t>(std::clamp(std::floor(v / m_cell), -9e18, 9e18));
        };
        return {to_cell(p.x), to_cell(p.y)};
    }

    double m_merge;
    double m_cell;
    std::unordered_map<Key, std::vector<std::pair<Vec2, int>>, KeyHash> m_cells;
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

/// Why points spread over the box from @p lo to @p hi are beyond the exact predicates; empty
/// when they are not.
std::string spreadProblem(const Vec2& lo, const Vec2& hi) {
    if (!(hi.x - lo.x <= kWidest) || !(hi.y - lo.y <= kWidest)) {
        return "the points spread too far for exact predicates";
    }
    const double spread = std::max(hi.x - lo.x, hi.y - lo.y);
    return spread > 0.0 && spread < kNarrowest ? "the points spread too little for exact predicates"
                                               : "";
}

void extend(Vec2& lo, Vec2& hi, const Vec2& p) {
    lo = {std::min(lo.x, p.x), std::min(lo.y, p.y)};
    hi = {std::max(hi.x, p.x), std::max(hi.y, p.y)};
}

}  // namespace

struct RegionTriangulation::State {
    Triangulation2D result;
    PointGrid grid{0.0};     // the points kept, for merge
    std::optional<Cdt> cdt;  // once the points are in it
    Vec2 lo;                 // the box of the points it was built from
    Vec2 hi;
    std::vector<int> slot;  // each triangle's index in result.triangles, or kNone

    /// Writes triangle @p t of @p c (alive, in the region) into result.triangles.
    void refresh(const Cdt& c, int t) {
        slot.resize(c.triangleCount(), kNone);
        const auto& v = c.triangle(t).v;
        const std::array<std::uint32_t, 3> tri = {static_cast<std::uint32_t>(c.pointOf(v[0])),
                                                  static_cast<std::uint32_t>(c.pointOf(v[1])),
                                                  static_cast<std::uint32_t>(c.pointOf(v[2]))};
        int& at = slot[static_cast<std::size_t>(t)];
        if (at == kNone) {
            at = static_cast<int>(result.triangles.size());
            result.triangles.push_back(tri);
        } else {
            result.triangles[static_cast<std::size_t>(at)] = tri;
        }
    }
};

RegionTriangulation::RegionTriangulation(const std::vector<Vec2>& givenOuter,
                                         const std::vector<std::vector<Vec2>>& givenHoles,
                                         const std::vector<Vec2>& givenInterior, double merge)
    : m_state(std::make_unique<State>()) {
    Triangulation2D& result = m_state->result;
    const double tol = std::max(merge, 0.0);
    const auto finite = [](const std::vector<Vec2>& ps) {
        return std::all_of(ps.begin(), ps.end(),
                           [](const Vec2& q) { return std::isfinite(q.x) && std::isfinite(q.y); });
    };
    if (!finite(givenOuter) || !finite(givenInterior) ||
        !std::all_of(givenHoles.begin(), givenHoles.end(), finite)) {
        result.problem = "a point is not finite";
        return;
    }
    // The spread as given, before flushing: a region wholly below the floor would flush to a
    // point and be reported as one.
    if (!givenOuter.empty()) {
        Vec2 lo = givenOuter.front();
        Vec2 hi = lo;
        for (const Vec2& p : givenOuter) {
            extend(lo, hi, p);
        }
        for (const auto& h : givenHoles) {
            for (const Vec2& p : h) {
                extend(lo, hi, p);
            }
        }
        result.problem = spreadProblem(lo, hi);
        if (!result.problem.empty()) {
            return;
        }
    }
    const std::vector<Vec2> outer = flushed(givenOuter);
    const std::vector<Vec2> interior = flushed(givenInterior);
    std::vector<std::vector<Vec2>> holes;
    holes.reserve(givenHoles.size());
    for (const auto& h : givenHoles) {
        holes.push_back(flushed(h));
    }

    // Boundary points first, in input order; one that is within tol of an earlier one stands
    // for that one.
    std::vector<const std::vector<Vec2>*> loops{&outer};
    for (const auto& h : holes) {
        loops.push_back(&h);
    }
    m_state->grid = PointGrid(tol);
    PointGrid& grid = m_state->grid;
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
    for (const int r : rep) {
        result.boundaryVertex.push_back(static_cast<std::uint32_t>(r));
    }
    std::vector<std::vector<Vec2>> merged(loops.size());
    for (std::size_t li = 0; li < loops.size(); ++li) {
        for (std::size_t i = 0; i < loops[li]->size(); ++i) {
            merged[li].push_back(result.points[static_cast<std::size_t>(rep[loop_starts[li] + i])]);
        }
    }
    {
        std::vector<int> distinct(rep.begin(),
                                  rep.begin() + static_cast<std::ptrdiff_t>(outer.size()));
        std::sort(distinct.begin(), distinct.end());
        distinct.erase(std::unique(distinct.begin(), distinct.end()), distinct.end());
        if (distinct.size() < 3) {
            if (result.problem.empty()) {
                result.problem = "the outer loop has fewer than 3 distinct points";
            }
            return;
        }
    }

    // Interior points that are strictly inside the region (as the merged loops bound it) and
    // clear of every other point.
    std::vector<int> inserted;
    for (std::size_t i = 0; i < boundary_count; ++i) {
        if (rep[i] == static_cast<int>(i)) {
            inserted.push_back(static_cast<int>(i));
        }
    }
    for (const Vec2& p : interior) {
        if (grid.find(p) >= 0 || math::classifyPoint(merged[0], p) != math::Containment::Inside) {
            continue;
        }
        bool in_hole = false;
        for (std::size_t li = 1; li < merged.size() && !in_hole; ++li) {
            in_hole = math::classifyPoint(merged[li], p) != math::Containment::Outside;
        }
        if (in_hole) {
            continue;
        }
        const int index = static_cast<int>(result.points.size());
        grid.add(p, index);
        result.points.push_back(p);
        inserted.push_back(index);
    }

    Vec2& lo = m_state->lo;
    Vec2& hi = m_state->hi;
    lo = result.points[static_cast<std::size_t>(inserted.front())];
    hi = lo;
    for (const int i : inserted) {
        extend(lo, hi, result.points[static_cast<std::size_t>(i)]);
    }
    if (std::string why = spreadProblem(lo, hi); !why.empty()) {
        result.problem = std::move(why);
        return;
    }
    // In a biased randomized order (BRIO): rounds of doubling size drawn at random, each sorted
    // along a Z-order curve. As given, a loop's points arrive along a line and every later
    // point's cavity sweeps the long thin triangles they leave; at random, each point's walk
    // from the last crosses those triangles instead. Two rows of n points took O(n^2) either
    // way; this bounds both. The shuffle is a fixed sequence, so a triangulation does not
    // change from run to run.
    std::uint64_t state = 0x9E3779B97F4A7C15ULL;
    for (std::size_t i = inserted.size(); i > 1; --i) {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        std::swap(inserted[i - 1], inserted[static_cast<std::size_t>(state % i)]);
    }
    // Each point's Z-order code, once; ties go by index, so every sort orders them alike.
    std::vector<std::uint64_t> zorder(result.points.size(), 0);
    const auto cell = [](double v, double from, double to) {
        const double f = to > from ? (v - from) / (to - from) : 0.0;
        return std::isfinite(f) ? static_cast<std::uint32_t>(std::clamp(f, 0.0, 1.0) * 65535.0)
                                : 0U;
    };
    for (const int index : inserted) {
        const Vec2& p = result.points[static_cast<std::size_t>(index)];
        const std::uint32_t cx = cell(p.x, lo.x, hi.x);
        const std::uint32_t cy = cell(p.y, lo.y, hi.y);
        std::uint64_t& code = zorder[static_cast<std::size_t>(index)];
        for (int bit = 0; bit < 16; ++bit) {
            code |= static_cast<std::uint64_t>((cx >> bit) & 1U) << (2 * bit);
            code |= static_cast<std::uint64_t>((cy >> bit) & 1U) << (2 * bit + 1);
        }
    }
    for (std::size_t from = 0, size = 1; from < inserted.size(); from += size, size *= 2) {
        const auto first = inserted.begin() + static_cast<std::ptrdiff_t>(from);
        const auto last =
            inserted.begin() + static_cast<std::ptrdiff_t>(std::min(inserted.size(), from + size));
        std::sort(first, last, [&](int a, int b) {
            const std::uint64_t za = zorder[static_cast<std::size_t>(a)];
            const std::uint64_t zb = zorder[static_cast<std::size_t>(b)];
            return za < zb || (za == zb && a < b);
        });
    }
    Cdt& cdt = m_state->cdt.emplace(result.points, lo, hi);
    for (const int i : inserted) {
        cdt.insert(i);
    }
    // A segment a loop runs along an even number of times (a slit, a bridge) is an edge but
    // not a wall.
    std::unordered_map<std::uint64_t, int> uses;
    for (std::size_t li = 0; li < loops.size(); ++li) {
        const std::size_t n = loops[li]->size();
        for (std::size_t i = 0; i < n; ++i) {
            const int a = rep[loop_starts[li] + i];
            const int b = rep[loop_starts[li] + (i + 1) % n];
            if (a != b) {
                ++uses[edgeKey(a, b)];
            }
        }
    }
    for (std::size_t li = 0; li < loops.size(); ++li) {
        const std::size_t base = loop_starts[li];
        const std::size_t n = loops[li]->size();
        for (std::size_t i = 0; i < n; ++i) {
            const int a = rep[base + i];
            const int b = rep[base + (i + 1) % n];
            const Recovery r = cdt.constrain(a, b);
            if (r == Recovery::Done) {
                if (a != b && uses[edgeKey(a, b)] % 2 == 1) {
                    cdt.markWall(a, b);
                }
            } else if (result.problem.empty()) {
                const std::string partner =
                    r == Recovery::Blocked
                        ? partnerOf(result.points, rep, loop_starts, loops, li, i)
                        : std::string();
                result.problem = segmentName(li, i, n) + " cannot be kept: " +
                                 (r == Recovery::GaveUp
                                      ? "edge recovery gave up"
                                      : (partner.empty() ? "the loops cross or touch" : partner));
            }
        }
    }
    cdt.lawson();

    // With the loops simple, one vertex of a hole says where the hole is.
    for (std::size_t k = 1; k < merged.size() && result.problem.empty(); ++k) {
        if (merged[k].empty()) {
            continue;
        }
        const Vec2& p = merged[k].front();
        if (math::classifyPoint(merged[0], p) != math::Containment::Inside) {
            result.problem = loopName(k) + " is not inside the outer loop";
            break;
        }
        for (std::size_t j = 1; j < merged.size(); ++j) {
            if (j != k && math::classifyPoint(merged[j], p) != math::Containment::Outside) {
                result.problem = loopName(k) + " is inside " + loopName(j);
                break;
            }
        }
    }

    const Cdt::Fill fill = cdt.fillRegion();
    if (fill.conflict.first != kNone && result.problem.empty()) {
        const auto name = [&](int v) {
            const auto index = static_cast<std::size_t>(v);
            if (index < boundary_count) {
                const std::size_t li = loopOf(loop_starts, index);
                return pointName(li, index - loop_starts[li]);
            }
            return index < result.points.size() ? "interior point " + std::to_string(index)
                                                : std::string("the outside");
        };
        result.problem = "parity of the region is ill-defined near the segment from " +
                         name(fill.conflict.first) + " to " + name(fill.conflict.second);
    }
    cdt.markInside(fill.inside);
    result.triangles.reserve(fill.inside.size());
    for (const int t : fill.inside) {
        m_state->refresh(cdt, t);
    }

    // Every boundary segment must be an edge of the result, as the header promises. One that no
    // triangle has lies outside the region (a spike run along both ways) or bounds no area at
    // all (a loop with none, or one run round twice).
    if (result.problem.empty()) {
        std::unordered_set<std::uint64_t> kept;
        for (const auto& tri : result.triangles) {
            for (std::size_t k = 0; k < 3; ++k) {
                kept.insert(edgeKey(static_cast<int>(tri[k]), static_cast<int>(tri[(k + 1) % 3])));
            }
        }
        for (std::size_t li = 0; li < loops.size() && result.problem.empty(); ++li) {
            const std::size_t n = loops[li]->size();
            for (std::size_t i = 0; i < n; ++i) {
                const int a = rep[loop_starts[li] + i];
                const int b = rep[loop_starts[li] + (i + 1) % n];
                if (a != b && kept.count(edgeKey(a, b)) == 0) {
                    result.problem =
                        segmentName(li, i, n) + " is an edge of no triangle of the region";
                    break;
                }
            }
        }
    }
}

RegionTriangulation::~RegionTriangulation() = default;
RegionTriangulation::RegionTriangulation(RegionTriangulation&&) noexcept = default;
RegionTriangulation& RegionTriangulation::operator=(RegionTriangulation&&) noexcept = default;

bool RegionTriangulation::insert(const Vec2& point) {
    State& s = *m_state;
    const Vec2 q = flushed(point);
    // A point strictly inside the region lies strictly inside the box of its points; one that
    // does not (or is not finite) meets no predicate.
    if (!s.cdt || !s.result.problem.empty() ||
        !(s.lo.x < q.x && q.x < s.hi.x && s.lo.y < q.y && q.y < s.hi.y) || s.grid.find(q) >= 0) {
        return false;
    }
    // The cavity's triangles are all made again (fill frees them last and allocate takes the last
    // freed first, and it makes two more than it frees), so none of the region's goes away.
    Cdt& cdt = *s.cdt;
    std::vector<int> changed;
    if (cdt.insertInside(q, changed) == kNone) {
        return false;
    }
    s.grid.add(q, static_cast<int>(s.result.points.size()));
    s.result.points.push_back(q);
    for (const int t : changed) {
        s.refresh(cdt, t);
    }
    return true;
}

const Triangulation2D& RegionTriangulation::result() const {
    return m_state->result;
}

Triangulation2D triangulateRegion(const std::vector<Vec2>& outer,
                                  const std::vector<std::vector<Vec2>>& holes,
                                  const std::vector<Vec2>& interior, double merge) {
    return RegionTriangulation(outer, holes, interior, merge).result();
}

}  // namespace hz::geo
