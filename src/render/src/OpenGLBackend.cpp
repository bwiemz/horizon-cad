#include "horizon/render/OpenGLBackend.h"

#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>

namespace hz::render {

#ifndef GL_SHADER_STORAGE_BUFFER
#define GL_SHADER_STORAGE_BUFFER 0x90D2
#endif

namespace {

uint32_t glTargetFor(BufferUsage usage) {
    switch (usage) {
        case BufferUsage::Vertex:
            return GL_ARRAY_BUFFER;
        case BufferUsage::Index:
            return GL_ELEMENT_ARRAY_BUFFER;
        case BufferUsage::Uniform:
            return GL_UNIFORM_BUFFER;
        case BufferUsage::Storage:
            return GL_SHADER_STORAGE_BUFFER;  // requires a GL 4.3 context
    }
    return GL_ARRAY_BUFFER;
}

}  // namespace

OpenGLBackend::OpenGLBackend(QOpenGLExtraFunctions* gl) : m_gl(gl) {}

OpenGLBackend::~OpenGLBackend() {
    // Callers own handle lifecycles, but free stragglers rather than leaking
    // GPU memory when the context is still current.
    for (auto& [id, rec] : m_buffers) {
        m_gl->glDeleteBuffers(1, &rec.glId);
    }
    for (auto& [id, tex] : m_textures) {
        m_gl->glDeleteTextures(1, &tex);
    }
    if (m_vao != 0) {
        m_gl->glDeleteVertexArrays(1, &m_vao);
    }
    m_shaders.clear();  // QOpenGLShaderProgram frees on destruction
}

std::string OpenGLBackend::name() const {
    const auto* version = reinterpret_cast<const char*>(m_gl->glGetString(GL_VERSION));
    return std::string("OpenGL ") + (version != nullptr ? version : "(unknown)");
}

BufferHandle OpenGLBackend::createBuffer(BufferUsage usage, const void* data, size_t size) {
    if (size == 0) return {};
    BufferRecord rec;
    rec.glTarget = glTargetFor(usage);
    rec.size = size;
    m_gl->glGenBuffers(1, &rec.glId);
    if (rec.glId == 0) return {};
    // Upload through GL_ARRAY_BUFFER regardless of usage — GL buffers are
    // typeless, and binding GL_ELEMENT_ARRAY_BUFFER here would mutate
    // whatever VAO happens to be bound.
    m_gl->glBindBuffer(GL_ARRAY_BUFFER, rec.glId);
    m_gl->glBufferData(GL_ARRAY_BUFFER, static_cast<qopengl_GLsizeiptr>(size), data,
                       GL_STATIC_DRAW);
    m_gl->glBindBuffer(GL_ARRAY_BUFFER, 0);

    const BufferHandle handle{m_nextHandle++};
    m_buffers.emplace(handle.id, rec);
    return handle;
}

void OpenGLBackend::updateBuffer(BufferHandle handle, const void* data, size_t size) {
    auto it = m_buffers.find(handle.id);
    // Contract: updates must fit the creation size (matches VulkanBackend).
    if (it == m_buffers.end() || size > it->second.size) return;
    m_gl->glBindBuffer(GL_ARRAY_BUFFER, it->second.glId);
    m_gl->glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<qopengl_GLsizeiptr>(size), data);
    m_gl->glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void OpenGLBackend::destroyBuffer(BufferHandle handle) {
    auto it = m_buffers.find(handle.id);
    if (it == m_buffers.end()) return;
    m_gl->glDeleteBuffers(1, &it->second.glId);
    m_buffers.erase(it);
}

TextureHandle OpenGLBackend::createTexture(int width, int height, const void* rgbaPixels) {
    if (width <= 0 || height <= 0) return {};
    uint32_t tex = 0;
    m_gl->glGenTextures(1, &tex);
    if (tex == 0) return {};
    m_gl->glBindTexture(GL_TEXTURE_2D, tex);
    m_gl->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                       rgbaPixels);
    m_gl->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    m_gl->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    m_gl->glBindTexture(GL_TEXTURE_2D, 0);

    const TextureHandle handle{m_nextHandle++};
    m_textures.emplace(handle.id, tex);
    return handle;
}

void OpenGLBackend::destroyTexture(TextureHandle handle) {
    auto it = m_textures.find(handle.id);
    if (it == m_textures.end()) return;
    m_gl->glDeleteTextures(1, &it->second);
    m_textures.erase(it);
}

ShaderHandle OpenGLBackend::createShader(const std::string& vertexSrc,
                                         const std::string& fragmentSrc) {
    auto program = std::make_unique<QOpenGLShaderProgram>();
    if (!program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexSrc.c_str()) ||
        !program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentSrc.c_str()) ||
        !program->link()) {
        return {};
    }
    const ShaderHandle handle{m_nextHandle++};
    m_shaders.emplace(handle.id, std::move(program));
    return handle;
}

void OpenGLBackend::destroyShader(ShaderHandle handle) {
    m_shaders.erase(handle.id);
}

void OpenGLBackend::beginPass(const RenderPassDesc& desc) {
    m_inPass = true;
    GLbitfield mask = 0;
    if (desc.clearColorBuffer) {
        m_gl->glClearColor(desc.clearColor[0], desc.clearColor[1], desc.clearColor[2],
                           desc.clearColor[3]);
        mask |= GL_COLOR_BUFFER_BIT;
    }
    if (desc.clearDepthBuffer) {
        mask |= GL_DEPTH_BUFFER_BIT;
    }
    if (mask != 0) m_gl->glClear(mask);
}

void OpenGLBackend::draw(const DrawCall& call) {
    if (!m_inPass || call.elementCount == 0) return;
    auto shaderIt = m_shaders.find(call.shader.id);
    auto vertexIt = m_buffers.find(call.vertexBuffer.id);
    if (shaderIt == m_shaders.end() || vertexIt == m_buffers.end()) return;

    // Core Profile removed the default vertex array: attribute setup and draw
    // calls are invalid with VAO 0 bound, so route everything through a
    // backend-owned scratch VAO.
    if (m_vao == 0) {
        m_gl->glGenVertexArrays(1, &m_vao);
        if (m_vao == 0) return;
    }
    m_gl->glBindVertexArray(m_vao);

    shaderIt->second->bind();
    m_gl->glBindBuffer(GL_ARRAY_BUFFER, vertexIt->second.glId);

    const int stride = call.layout.stride();
    intptr_t offset = 0;
    for (int i = 0; i < call.layout.attributeCount; ++i) {
        m_gl->glEnableVertexAttribArray(static_cast<GLuint>(i));
        m_gl->glVertexAttribPointer(static_cast<GLuint>(i), call.layout.attributeComponents[i],
                                    GL_FLOAT, GL_FALSE, stride,
                                    reinterpret_cast<const void*>(offset));
        offset += call.layout.attributeComponents[i] * static_cast<intptr_t>(sizeof(float));
    }

    const GLenum mode = call.topology == PrimitiveTopology::Triangles ? GL_TRIANGLES : GL_LINES;
    auto indexIt = m_buffers.find(call.indexBuffer.id);
    if (indexIt != m_buffers.end()) {
        m_gl->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexIt->second.glId);
        m_gl->glDrawElements(mode, static_cast<GLsizei>(call.elementCount), GL_UNSIGNED_INT,
                             nullptr);
        m_gl->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    } else {
        m_gl->glDrawArrays(mode, 0, static_cast<GLsizei>(call.elementCount));
    }

    for (int i = 0; i < call.layout.attributeCount; ++i) {
        m_gl->glDisableVertexAttribArray(static_cast<GLuint>(i));
    }
    m_gl->glBindBuffer(GL_ARRAY_BUFFER, 0);
    shaderIt->second->release();
    m_gl->glBindVertexArray(0);
}

void OpenGLBackend::endPass() {
    m_inPass = false;
}

bool OpenGLBackend::submitCompute(const std::string& computeSrc, uint32_t groupsX, uint32_t groupsY,
                                  uint32_t groupsZ) {
    // Compute shaders need GL 4.3+; the viewport context is 3.3 Core on some
    // platforms, so probe the actual context version at runtime.
    const auto* ctx = QOpenGLContext::currentContext();
    if (ctx == nullptr) return false;
    const auto fmt = ctx->format();
    if (fmt.majorVersion() < 4 || (fmt.majorVersion() == 4 && fmt.minorVersion() < 3)) {
        return false;
    }

    QOpenGLShaderProgram program;
    if (!program.addShaderFromSourceCode(QOpenGLShader::Compute, computeSrc.c_str()) ||
        !program.link()) {
        return false;
    }
    program.bind();
    m_gl->glDispatchCompute(groupsX, groupsY, groupsZ);
    m_gl->glMemoryBarrier(GL_ALL_BARRIER_BITS);
    program.release();
    return true;
}

}  // namespace hz::render
