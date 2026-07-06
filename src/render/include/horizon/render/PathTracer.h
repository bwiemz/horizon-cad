#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"
#include "horizon/render/SceneGraph.h"

namespace hz::render {

/// Camera for offline rendering: position/target/up + vertical field of view.
struct RtCamera {
    math::Vec3 eye{20.0, -20.0, 15.0};
    math::Vec3 target{0.0, 0.0, 0.0};
    math::Vec3 up{0.0, 0.0, 1.0};
    double fovYDegrees = 40.0;
};

/// Progressive rendering settings.
struct RtSettings {
    int width = 640;
    int height = 480;
    int samplesPerPixel = 16;  ///< Monte Carlo samples; quality scales with it.
    int maxBounces = 4;
    uint64_t seed = 7;                         ///< Deterministic: same seed → same image.
    math::Vec3 skyColor{0.75, 0.82, 0.92};     ///< Environment radiance (up).
    math::Vec3 groundColor{0.25, 0.22, 0.20};  ///< Environment radiance (down).
    math::Vec3 sunDirection{0.3, 0.5, 0.8};    ///< Toward the sun.
    math::Vec3 sunRadiance{2.5, 2.4, 2.2};
    bool multithreaded = true;
};

/// Linear-radiance image produced by the tracer.
struct RtImage {
    int width = 0;
    int height = 0;
    std::vector<math::Vec3> pixels;  ///< Row-major, linear radiance.

    /// Tone-mapped (Reinhard + gamma 2.2) 24-bit PPM — dependency-free output.
    bool savePpm(const std::string& path) const;
};

/// CPU Monte Carlo path tracer (Phase 68, roadmap §7.4).
///
/// In-house instead of Embree — the same dependency-light trade as the FEA
/// solver and the STEP parser (documented in the roadmap findings note). A
/// median-split BVH accelerates Möller–Trumbore triangle intersection;
/// materials use the viewport's metallic-roughness model with cosine-weighted
/// diffuse and GGX specular importance sampling; lighting combines a
/// hemisphere environment with a directional sun (next-event estimation).
/// Rows render in parallel with per-pixel seeding, so multithreaded output is
/// bit-identical to single-threaded.
class PathTracer {
public:
    /// Add one triangle mesh with a material and world transform.
    void addMesh(const MeshData& mesh, const Material& material,
                 const math::Mat4& transform = math::Mat4::identity());

    /// Add every visible mesh node of a scene graph (world transforms baked).
    void addScene(const SceneGraph& scene);

    /// Number of triangles across all added meshes.
    size_t triangleCount() const { return m_triangles.size(); }

    /// Render the scene. Progressive quality via settings.samplesPerPixel.
    RtImage render(const RtCamera& camera, const RtSettings& settings) const;

private:
    struct Triangle {
        math::Vec3 a, b, c;
        math::Vec3 normal;
        uint32_t materialIndex = 0;
    };

    struct BvhNode {
        math::Vec3 boundsMin, boundsMax;
        int left = -1;  ///< Child index, or -1 for leaves.
        int right = -1;
        int first = 0;  ///< Leaf: first triangle index (into m_order).
        int count = 0;  ///< Leaf: triangle count.
    };

    struct Hit {
        double t = 0.0;
        math::Vec3 point;
        math::Vec3 normal;
        uint32_t materialIndex = 0;
    };

    void buildBvh() const;
    bool intersect(const math::Vec3& origin, const math::Vec3& dir, Hit& hit) const;
    bool occluded(const math::Vec3& origin, const math::Vec3& dir, double maxT) const;
    math::Vec3 trace(math::Vec3 origin, math::Vec3 dir, const RtSettings& settings,
                     uint64_t& rng) const;

    std::vector<Triangle> m_triangles;
    std::vector<Material> m_materials;

    // BVH built lazily on first render; the mutex makes concurrent const
    // render() calls safe (the build is double-checked).
    mutable std::vector<BvhNode> m_nodes;
    mutable std::vector<uint32_t> m_order;
    mutable bool m_bvhDirty = true;
    mutable std::mutex m_bvhMutex;
};

}  // namespace hz::render
