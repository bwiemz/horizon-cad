#include "horizon/render/PathTracer.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <future>
#include <thread>

#include "horizon/math/Constants.h"

namespace hz::render {

using hz::math::Vec3;

namespace {

constexpr double kRayEpsilon = 1e-6;

// -- Deterministic RNG (splitmix64 → [0,1)) ----------------------------------

uint64_t splitmix64(uint64_t& state) {
    state += 0x9E3779B97F4A7C15ULL;
    uint64_t z = state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

double rand01(uint64_t& state) {
    return static_cast<double>(splitmix64(state) >> 11) * (1.0 / 9007199254740992.0);
}

// -- Sampling ------------------------------------------------------------------

/// Orthonormal frame around @p n.
void makeFrame(const Vec3& n, Vec3& t, Vec3& b) {
    const Vec3 seed = std::abs(n.x) < 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
    t = n.cross(seed).normalized();
    b = n.cross(t);
}

Vec3 cosineHemisphere(const Vec3& n, uint64_t& rng) {
    const double u1 = rand01(rng);
    const double u2 = rand01(rng);
    const double r = std::sqrt(u1);
    const double phi = 2.0 * hz::math::kPi * u2;
    Vec3 t, b;
    makeFrame(n, t, b);
    const double z = std::sqrt(std::max(0.0, 1.0 - u1));
    return (t * (r * std::cos(phi)) + b * (r * std::sin(phi)) + n * z).normalized();
}

/// GGX half-vector sample around normal @p n for roughness @p alpha.
Vec3 ggxHalfVector(const Vec3& n, double alpha, uint64_t& rng) {
    const double u1 = rand01(rng);
    const double u2 = rand01(rng);
    const double phi = 2.0 * hz::math::kPi * u1;
    const double cosTheta = std::sqrt((1.0 - u2) / (1.0 + (alpha * alpha - 1.0) * u2));
    const double sinTheta = std::sqrt(std::max(0.0, 1.0 - cosTheta * cosTheta));
    Vec3 t, b;
    makeFrame(n, t, b);
    return (t * (sinTheta * std::cos(phi)) + b * (sinTheta * std::sin(phi)) + n * cosTheta)
        .normalized();
}

Vec3 reflect(const Vec3& d, const Vec3& n) {
    return d - n * (2.0 * d.dot(n));
}

Vec3 schlick(const Vec3& f0, double cosTheta) {
    const double f = std::pow(1.0 - std::clamp(cosTheta, 0.0, 1.0), 5.0);
    return f0 + (Vec3(1, 1, 1) - f0) * f;
}

}  // namespace

// -- Image output ---------------------------------------------------------------

bool RtImage::savePpm(const std::string& path) const {
    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) return false;
    file << "P6\n" << width << " " << height << "\n255\n";
    for (const Vec3& p : pixels) {
        for (double c : {p.x, p.y, p.z}) {
            const double mapped = c / (1.0 + c);                             // Reinhard
            const double srgb = std::pow(std::max(0.0, mapped), 1.0 / 2.2);  // gamma
            file.put(static_cast<char>(std::clamp(srgb, 0.0, 1.0) * 255.0 + 0.5));
        }
    }
    file.close();
    return !file.fail();
}

// -- Scene assembly ---------------------------------------------------------------

void PathTracer::addMesh(const MeshData& mesh, const Material& material,
                         const math::Mat4& transform) {
    const auto materialIndex = static_cast<uint32_t>(m_materials.size());
    m_materials.push_back(material);

    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        Triangle tri;
        const uint32_t ia = mesh.indices[i];
        const uint32_t ib = mesh.indices[i + 1];
        const uint32_t ic = mesh.indices[i + 2];
        tri.a = transform.transformPoint(
            Vec3(mesh.positions[3 * ia], mesh.positions[3 * ia + 1], mesh.positions[3 * ia + 2]));
        tri.b = transform.transformPoint(
            Vec3(mesh.positions[3 * ib], mesh.positions[3 * ib + 1], mesh.positions[3 * ib + 2]));
        tri.c = transform.transformPoint(
            Vec3(mesh.positions[3 * ic], mesh.positions[3 * ic + 1], mesh.positions[3 * ic + 2]));
        const Vec3 n = (tri.b - tri.a).cross(tri.c - tri.a);
        const double len = n.length();
        if (len < 1e-14) continue;  // degenerate
        tri.normal = n * (1.0 / len);
        tri.materialIndex = materialIndex;
        m_triangles.push_back(tri);
    }
    m_bvhDirty = true;
}

void PathTracer::addScene(const SceneGraph& scene) {
    for (const SceneNode* node : scene.collectVisibleMeshNodes()) {
        if (node->hasMesh()) {
            addMesh(node->mesh(), node->material(), node->worldTransform());
        }
    }
}

// -- BVH ---------------------------------------------------------------------------

void PathTracer::buildBvh() const {
    m_nodes.clear();
    m_order.resize(m_triangles.size());
    for (uint32_t i = 0; i < m_order.size(); ++i) m_order[i] = i;
    if (m_triangles.empty()) {
        m_bvhDirty = false;
        return;
    }

    struct Range {
        int node;
        int first;
        int count;
    };

    auto centroid = [&](uint32_t t) {
        return (m_triangles[t].a + m_triangles[t].b + m_triangles[t].c) * (1.0 / 3.0);
    };

    m_nodes.reserve(m_triangles.size() * 2);
    m_nodes.push_back({});
    std::vector<Range> stack{{0, 0, static_cast<int>(m_order.size())}};

    while (!stack.empty()) {
        const Range range = stack.back();
        stack.pop_back();
        BvhNode& node = m_nodes[static_cast<size_t>(range.node)];

        Vec3 mn(1e300, 1e300, 1e300);
        Vec3 mx(-1e300, -1e300, -1e300);
        for (int i = range.first; i < range.first + range.count; ++i) {
            const Triangle& tri = m_triangles[m_order[static_cast<size_t>(i)]];
            for (const Vec3* p : {&tri.a, &tri.b, &tri.c}) {
                mn = Vec3(std::min(mn.x, p->x), std::min(mn.y, p->y), std::min(mn.z, p->z));
                mx = Vec3(std::max(mx.x, p->x), std::max(mx.y, p->y), std::max(mx.z, p->z));
            }
        }
        node.boundsMin = mn;
        node.boundsMax = mx;

        if (range.count <= 4) {
            node.first = range.first;
            node.count = range.count;
            continue;
        }

        // Median split along the widest centroid axis.
        const Vec3 extent = mx - mn;
        int axis = 0;
        if (extent.y > extent.x) axis = 1;
        if (extent.z > (axis == 0 ? extent.x : extent.y)) axis = 2;

        const int mid = range.first + range.count / 2;
        std::nth_element(m_order.begin() + range.first, m_order.begin() + mid,
                         m_order.begin() + range.first + range.count, [&](uint32_t l, uint32_t r) {
                             const Vec3 cl = centroid(l);
                             const Vec3 cr = centroid(r);
                             const double vl = axis == 0 ? cl.x : (axis == 1 ? cl.y : cl.z);
                             const double vr = axis == 0 ? cr.x : (axis == 1 ? cr.y : cr.z);
                             return vl < vr;
                         });

        const int leftIndex = static_cast<int>(m_nodes.size());
        m_nodes.push_back({});
        const int rightIndex = static_cast<int>(m_nodes.size());
        m_nodes.push_back({});
        // Re-fetch: push_back may reallocate.
        m_nodes[static_cast<size_t>(range.node)].left = leftIndex;
        m_nodes[static_cast<size_t>(range.node)].right = rightIndex;
        stack.push_back({leftIndex, range.first, range.count / 2});
        stack.push_back({rightIndex, mid, range.count - range.count / 2});
    }
    m_bvhDirty = false;
}

namespace {

bool intersectAabb(const Vec3& origin, const Vec3& invDir, const Vec3& mn, const Vec3& mx,
                   double maxT) {
    double t0 = 0.0;
    double t1 = maxT;
    for (int axis = 0; axis < 3; ++axis) {
        const double o = axis == 0 ? origin.x : (axis == 1 ? origin.y : origin.z);
        const double inv = axis == 0 ? invDir.x : (axis == 1 ? invDir.y : invDir.z);
        const double lo = axis == 0 ? mn.x : (axis == 1 ? mn.y : mn.z);
        const double hi = axis == 0 ? mx.x : (axis == 1 ? mx.y : mx.z);
        double ta = (lo - o) * inv;
        double tb = (hi - o) * inv;
        if (ta > tb) std::swap(ta, tb);
        t0 = std::max(t0, ta);
        t1 = std::min(t1, tb);
        if (t0 > t1) return false;
    }
    return true;
}

/// Möller–Trumbore. On hit fills @p t (distance) and returns true.
bool intersectTriangle(const Vec3& origin, const Vec3& dir, const Vec3& a, const Vec3& b,
                       const Vec3& c, double& t) {
    const Vec3 e1 = b - a;
    const Vec3 e2 = c - a;
    const Vec3 p = dir.cross(e2);
    const double det = e1.dot(p);
    if (std::abs(det) < 1e-14) return false;
    const double invDet = 1.0 / det;
    const Vec3 s = origin - a;
    const double u = s.dot(p) * invDet;
    if (u < 0.0 || u > 1.0) return false;
    const Vec3 q = s.cross(e1);
    const double v = dir.dot(q) * invDet;
    if (v < 0.0 || u + v > 1.0) return false;
    const double tt = e2.dot(q) * invDet;
    if (tt <= kRayEpsilon) return false;
    t = tt;
    return true;
}

}  // namespace

bool PathTracer::intersect(const Vec3& origin, const Vec3& dir, Hit& hit) const {
    if (m_bvhDirty) buildBvh();
    if (m_nodes.empty()) return false;

    const Vec3 invDir(1.0 / (dir.x != 0.0 ? dir.x : 1e-300), 1.0 / (dir.y != 0.0 ? dir.y : 1e-300),
                      1.0 / (dir.z != 0.0 ? dir.z : 1e-300));

    double best = 1e300;
    int bestTri = -1;
    int stack[64];
    int top = 0;
    stack[top++] = 0;
    while (top > 0) {
        const BvhNode& node = m_nodes[static_cast<size_t>(stack[--top])];
        if (!intersectAabb(origin, invDir, node.boundsMin, node.boundsMax, best)) continue;
        if (node.left < 0) {
            for (int i = node.first; i < node.first + node.count; ++i) {
                const uint32_t triIndex = m_order[static_cast<size_t>(i)];
                const Triangle& tri = m_triangles[triIndex];
                double t = 0.0;
                if (intersectTriangle(origin, dir, tri.a, tri.b, tri.c, t) && t < best) {
                    best = t;
                    bestTri = static_cast<int>(triIndex);
                }
            }
        } else if (top + 2 <= 64) {
            stack[top++] = node.left;
            stack[top++] = node.right;
        }
    }

    if (bestTri < 0) return false;
    const Triangle& tri = m_triangles[static_cast<size_t>(bestTri)];
    hit.t = best;
    hit.point = origin + dir * best;
    hit.normal = tri.normal.dot(dir) < 0.0 ? tri.normal : tri.normal * -1.0;
    hit.materialIndex = tri.materialIndex;
    return true;
}

bool PathTracer::occluded(const Vec3& origin, const Vec3& dir, double maxT) const {
    Hit hit;
    return intersect(origin, dir, hit) && hit.t < maxT;
}

// -- Path tracing --------------------------------------------------------------------

Vec3 PathTracer::trace(Vec3 origin, Vec3 dir, const RtSettings& settings, uint64_t& rng) const {
    Vec3 throughput(1, 1, 1);
    Vec3 radiance(0, 0, 0);
    const Vec3 sunDir = settings.sunDirection.normalized();

    for (int bounce = 0; bounce <= settings.maxBounces; ++bounce) {
        Hit hit;
        if (!intersect(origin, dir, hit)) {
            // Environment: hemisphere gradient.
            const double h = 0.5 + 0.5 * dir.z;
            const Vec3 env = settings.groundColor * (1.0 - h) + settings.skyColor * h;
            radiance =
                radiance + Vec3(throughput.x * env.x, throughput.y * env.y, throughput.z * env.z);
            break;
        }

        const Material& mat = m_materials[hit.materialIndex];
        const Vec3 albedo = mat.color;
        const double metallic = mat.metallic;
        const double roughness = std::clamp(static_cast<double>(mat.roughness), 0.05, 1.0);
        const Vec3 f0 = Vec3(0.04, 0.04, 0.04) * (1.0 - metallic) + albedo * metallic;

        // Next-event estimation toward the sun.
        const double nDotSun = hit.normal.dot(sunDir);
        if (nDotSun > 0.0 &&
            !occluded(hit.point + hit.normal * kRayEpsilon * 10.0, sunDir, 1e300)) {
            const Vec3 diffuse = albedo * ((1.0 - metallic) / hz::math::kPi);
            const Vec3 direct =
                Vec3(diffuse.x * settings.sunRadiance.x, diffuse.y * settings.sunRadiance.y,
                     diffuse.z * settings.sunRadiance.z) *
                nDotSun;
            radiance = radiance + Vec3(throughput.x * direct.x, throughput.y * direct.y,
                                       throughput.z * direct.z);
        }

        // Sample the BRDF: choose specular vs diffuse by Fresnel weight.
        const double specProb = std::clamp(0.25 + 0.5 * metallic, 0.0, 1.0);
        Vec3 nextDir;
        Vec3 weight;
        if (rand01(rng) < specProb) {
            const Vec3 h = ggxHalfVector(hit.normal, roughness * roughness, rng);
            nextDir = reflect(dir, h);
            if (nextDir.dot(hit.normal) <= 0.0) break;  // absorbed into the surface
            const Vec3 f = schlick(f0, nextDir.dot(h));
            weight = f * (1.0 / specProb);
        } else {
            nextDir = cosineHemisphere(hit.normal, rng);
            weight = albedo * ((1.0 - metallic) / (1.0 - specProb));
        }

        throughput =
            Vec3(throughput.x * weight.x, throughput.y * weight.y, throughput.z * weight.z);

        // Russian roulette after a few bounces.
        if (bounce >= 2) {
            const double p = std::clamp(
                std::max(throughput.x, std::max(throughput.y, throughput.z)), 0.05, 0.95);
            if (rand01(rng) > p) break;
            throughput = throughput * (1.0 / p);
        }

        origin = hit.point + hit.normal * kRayEpsilon * 10.0;
        dir = nextDir;
    }
    return radiance;
}

RtImage PathTracer::render(const RtCamera& camera, const RtSettings& settings) const {
    RtImage image;
    image.width = settings.width;
    image.height = settings.height;
    image.pixels.assign(static_cast<size_t>(settings.width) * settings.height, Vec3(0, 0, 0));
    if (settings.width <= 0 || settings.height <= 0 || settings.samplesPerPixel <= 0) {
        return image;
    }
    if (m_bvhDirty) buildBvh();

    const Vec3 forward = (camera.target - camera.eye).normalized();
    const Vec3 right = forward.cross(camera.up).normalized();
    const Vec3 up = right.cross(forward);
    const double tanHalfFov = std::tan(camera.fovYDegrees * hz::math::kDegToRad * 0.5);
    const double aspect = static_cast<double>(settings.width) / settings.height;

    auto renderRow = [&](int y) {
        for (int x = 0; x < settings.width; ++x) {
            // Per-pixel seeding keeps multithreaded output bit-identical.
            uint64_t rng = settings.seed * 0x100000001B3ULL +
                           static_cast<uint64_t>(y) * 2654435761ULL + static_cast<uint64_t>(x);
            Vec3 sum(0, 0, 0);
            for (int s = 0; s < settings.samplesPerPixel; ++s) {
                const double jx = rand01(rng);
                const double jy = rand01(rng);
                const double px = (2.0 * ((x + jx) / settings.width) - 1.0) * tanHalfFov * aspect;
                const double py = (1.0 - 2.0 * ((y + jy) / settings.height)) * tanHalfFov;
                const Vec3 dir = (forward + right * px + up * py).normalized();
                sum = sum + trace(camera.eye, dir, settings, rng);
            }
            image.pixels[static_cast<size_t>(y) * settings.width + x] =
                sum * (1.0 / settings.samplesPerPixel);
        }
    };

    if (settings.multithreaded) {
        const unsigned bands = std::max(1u, std::thread::hardware_concurrency());
        std::vector<std::future<void>> futures;
        futures.reserve(bands);
        for (unsigned b = 0; b < bands; ++b) {
            futures.push_back(std::async(std::launch::async, [&, b]() {
                for (int y = static_cast<int>(b); y < settings.height;
                     y += static_cast<int>(bands)) {
                    renderRow(y);
                }
            }));
        }
        for (auto& f : futures) f.get();
    } else {
        for (int y = 0; y < settings.height; ++y) renderRow(y);
    }
    return image;
}

}  // namespace hz::render
