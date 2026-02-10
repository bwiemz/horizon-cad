#pragma once

#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>

#include <cstdint>
#include <vector>

class QOpenGLExtraFunctions;

namespace hz::render {

/// Wraps a VAO + VBO + EBO for mesh rendering using Qt's OpenGL objects.
class MeshBuffer {
public:
    MeshBuffer();
    ~MeshBuffer();

    MeshBuffer(const MeshBuffer&) = delete;
    MeshBuffer& operator=(const MeshBuffer&) = delete;
    MeshBuffer(MeshBuffer&&) = delete;
    MeshBuffer& operator=(MeshBuffer&&) = delete;

    /// Create GPU buffers from interleaved position+normal data and index data.
    /// positions: vec3 per vertex, normals: vec3 per vertex, indices: triangle list.
    void create(QOpenGLExtraFunctions* gl,
                const std::vector<float>& positions,
                const std::vector<float>& normals,
                const std::vector<uint32_t>& indices);

    /// Create GPU buffers from position-only data (no normals, no indices).
    void createPositionOnly(QOpenGLExtraFunctions* gl,
                            const std::vector<float>& positions);

    void bind();
    void draw(QOpenGLExtraFunctions* gl);
    void release();

    int vertexCount() const { return m_vertexCount; }
    int indexCount() const { return m_indexCount; }

    bool isValid() const { return m_vao.isCreated(); }

private:
    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_vbo{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer m_ebo{QOpenGLBuffer::IndexBuffer};

    int m_vertexCount = 0;
    int m_indexCount = 0;
    bool m_hasIndices = false;
};

}  // namespace hz::render
