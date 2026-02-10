#pragma once

#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"

#include <QOpenGLShaderProgram>

#include <memory>
#include <string>

namespace hz::render {

/// Wrapper around QOpenGLShaderProgram providing a convenient API for Horizon types.
class ShaderProgram {
public:
    ShaderProgram();
    ~ShaderProgram();

    ShaderProgram(const ShaderProgram&) = delete;
    ShaderProgram& operator=(const ShaderProgram&) = delete;
    ShaderProgram(ShaderProgram&&) noexcept;
    ShaderProgram& operator=(ShaderProgram&&) noexcept;

    /// Compile and link a vertex + fragment shader from source strings.
    /// Returns true on success.
    bool create(const std::string& vertSrc, const std::string& fragSrc);

    /// Compile and link from source files on disk.
    bool createFromFiles(const std::string& vertPath, const std::string& fragPath);

    void bind();
    void release();

    void setUniform(const char* name, const math::Mat4& mat);
    void setUniform(const char* name, const math::Vec3& vec);
    void setUniform(const char* name, float value);
    void setUniform(const char* name, int value);

    bool isValid() const;
    QOpenGLShaderProgram* handle() const { return m_program.get(); }

private:
    std::unique_ptr<QOpenGLShaderProgram> m_program;
};

}  // namespace hz::render
