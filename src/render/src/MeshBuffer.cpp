#include "horizon/render/MeshBuffer.h"

#include <QOpenGLExtraFunctions>

namespace hz::render {

MeshBuffer::MeshBuffer() = default;
MeshBuffer::~MeshBuffer() = default;


void MeshBuffer::create(QOpenGLExtraFunctions* gl,
                         const std::vector<float>& positions,
                         const std::vector<float>& normals,
                         const std::vector<uint32_t>& indices) {
    if (positions.empty()) return;

    m_vertexCount = static_cast<int>(positions.size() / 3);
    m_indexCount = static_cast<int>(indices.size());
    m_hasIndices = !indices.empty();

    // Interleave position + normal data: [px, py, pz, nx, ny, nz, ...]
    std::vector<float> interleaved;
    interleaved.reserve(m_vertexCount * 6);
    for (int i = 0; i < m_vertexCount; ++i) {
        interleaved.push_back(positions[i * 3 + 0]);
        interleaved.push_back(positions[i * 3 + 1]);
        interleaved.push_back(positions[i * 3 + 2]);
        if (normals.size() >= static_cast<size_t>((i + 1) * 3)) {
            interleaved.push_back(normals[i * 3 + 0]);
            interleaved.push_back(normals[i * 3 + 1]);
            interleaved.push_back(normals[i * 3 + 2]);
        } else {
            interleaved.push_back(0.0f);
            interleaved.push_back(0.0f);
            interleaved.push_back(1.0f);
        }
    }

    // Create VAO
    m_vao.create();
    m_vao.bind();

    // Create VBO
    m_vbo.create();
    m_vbo.bind();
    m_vbo.allocate(interleaved.data(),
                   static_cast<int>(interleaved.size() * sizeof(float)));

    // Position attribute (location 0)
    gl->glEnableVertexAttribArray(0);
    gl->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                              nullptr);

    // Normal attribute (location 1)
    gl->glEnableVertexAttribArray(1);
    gl->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                              reinterpret_cast<void*>(3 * sizeof(float)));

    // Create EBO if we have indices
    if (m_hasIndices) {
        m_ebo.create();
        m_ebo.bind();
        m_ebo.allocate(indices.data(),
                       static_cast<int>(indices.size() * sizeof(uint32_t)));
    }

    m_vao.release();
    m_vbo.release();
    if (m_hasIndices) m_ebo.release();
}

void MeshBuffer::createPositionOnly(QOpenGLExtraFunctions* gl,
                                     const std::vector<float>& positions) {
    if (positions.empty()) return;

    m_vertexCount = static_cast<int>(positions.size() / 3);
    m_indexCount = 0;
    m_hasIndices = false;

    m_vao.create();
    m_vao.bind();

    m_vbo.create();
    m_vbo.bind();
    m_vbo.allocate(positions.data(),
                   static_cast<int>(positions.size() * sizeof(float)));

    // Position attribute (location 0)
    gl->glEnableVertexAttribArray(0);
    gl->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                              nullptr);

    m_vao.release();
    m_vbo.release();
}

void MeshBuffer::bind() {
    if (m_vao.isCreated()) m_vao.bind();
}

void MeshBuffer::draw(QOpenGLExtraFunctions* gl) {
    if (!m_vao.isCreated()) return;

    if (m_hasIndices) {
        gl->glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, nullptr);
    } else {
        gl->glDrawArrays(GL_TRIANGLES, 0, m_vertexCount);
    }
}

void MeshBuffer::release() {
    if (m_vao.isCreated()) m_vao.release();
}

}  // namespace hz::render
