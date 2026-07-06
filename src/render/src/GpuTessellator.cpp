#ifdef HZ_ENABLE_GPU_TESSELLATION

#include "horizon/render/GpuTessellator.h"

#include <cstring>

#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/render/VulkanBackend.h"
#include "nurbs_eval_spv.h"

namespace hz::render {

using hz::math::Vec3;

GpuTessellator::GpuTessellator(VulkanBackend& backend) : m_backend(backend) {}

std::vector<Vec3> GpuTessellator::evaluateGrid(const geo::NurbsSurface& surface, uint32_t nx,
                                               uint32_t ny) {
    // The kernel's basis-function scratch arrays support degrees up to 7.
    if (!m_backend.isAvailable() || nx < 2 || ny < 2 || surface.degreeU() > 7 ||
        surface.degreeV() > 7) {
        return {};
    }

    const auto& cps = surface.controlPoints();
    const auto& weights = surface.weights();
    const uint32_t countU = static_cast<uint32_t>(surface.controlPointCountU());
    const uint32_t countV = static_cast<uint32_t>(surface.controlPointCountV());

    // Meta block (must match the shader's std430 Meta layout).
    const uint32_t meta[8] = {static_cast<uint32_t>(surface.degreeU()),
                              static_cast<uint32_t>(surface.degreeV()),
                              countU,
                              countV,
                              nx,
                              ny,
                              0,
                              0};

    std::vector<float> knotsU(surface.knotsU().begin(), surface.knotsU().end());
    std::vector<float> knotsV(surface.knotsV().begin(), surface.knotsV().end());

    // Control net row-major [u][v], xyz + weight.
    std::vector<float> net;
    net.reserve(static_cast<size_t>(countU) * countV * 4);
    for (uint32_t iu = 0; iu < countU; ++iu) {
        for (uint32_t iv = 0; iv < countV; ++iv) {
            const Vec3& p = cps[iu][iv];
            net.push_back(static_cast<float>(p.x));
            net.push_back(static_cast<float>(p.y));
            net.push_back(static_cast<float>(p.z));
            net.push_back(static_cast<float>(weights[iu][iv]));
        }
    }

    const size_t outFloats = static_cast<size_t>(nx) * ny * 4;

    const BufferHandle metaBuf = m_backend.createBuffer(BufferUsage::Storage, meta, sizeof(meta));
    const BufferHandle knotsUBuf =
        m_backend.createBuffer(BufferUsage::Storage, knotsU.data(), knotsU.size() * sizeof(float));
    const BufferHandle knotsVBuf =
        m_backend.createBuffer(BufferUsage::Storage, knotsV.data(), knotsV.size() * sizeof(float));
    const BufferHandle netBuf =
        m_backend.createBuffer(BufferUsage::Storage, net.data(), net.size() * sizeof(float));
    const BufferHandle outBuf =
        m_backend.createBuffer(BufferUsage::Storage, nullptr, outFloats * sizeof(float));

    std::vector<Vec3> result;
    if (metaBuf.isValid() && knotsUBuf.isValid() && knotsVBuf.isValid() && netBuf.isValid() &&
        outBuf.isValid()) {
        const uint32_t groupsX = (nx + 7) / 8;
        const uint32_t groupsY = (ny + 7) / 8;
        if (m_backend.runComputeSpirv(
                kNurbsEvalSpv, sizeof(kNurbsEvalSpv) / sizeof(kNurbsEvalSpv[0]),
                {metaBuf, knotsUBuf, knotsVBuf, netBuf, outBuf}, groupsX, groupsY, 1)) {
            std::vector<float> raw(outFloats);
            if (m_backend.readBuffer(outBuf, raw.data(), raw.size() * sizeof(float))) {
                result.reserve(static_cast<size_t>(nx) * ny);
                for (size_t i = 0; i < static_cast<size_t>(nx) * ny; ++i) {
                    result.emplace_back(raw[4 * i], raw[4 * i + 1], raw[4 * i + 2]);
                }
            }
        }
    }

    m_backend.destroyBuffer(metaBuf);
    m_backend.destroyBuffer(knotsUBuf);
    m_backend.destroyBuffer(knotsVBuf);
    m_backend.destroyBuffer(netBuf);
    m_backend.destroyBuffer(outBuf);
    return result;
}

}  // namespace hz::render

#endif  // HZ_ENABLE_GPU_TESSELLATION
