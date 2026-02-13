#include "horizon/render/ShaderProgram.h"

#include <QFile>
#include <QTextStream>

namespace hz::render {

ShaderProgram::ShaderProgram() = default;
ShaderProgram::~ShaderProgram() = default;

ShaderProgram::ShaderProgram(ShaderProgram&&) noexcept = default;
ShaderProgram& ShaderProgram::operator=(ShaderProgram&&) noexcept = default;

bool ShaderProgram::create(const std::string& vertSrc, const std::string& fragSrc) {
    m_program = std::make_unique<QOpenGLShaderProgram>();

    if (!m_program->addShaderFromSourceCode(QOpenGLShader::Vertex,
                                             vertSrc.c_str())) {
        qWarning("ShaderProgram: vertex shader compilation failed:\n%s",
                 qPrintable(m_program->log()));
        m_program.reset();
        return false;
    }

    if (!m_program->addShaderFromSourceCode(QOpenGLShader::Fragment,
                                             fragSrc.c_str())) {
        qWarning("ShaderProgram: fragment shader compilation failed:\n%s",
                 qPrintable(m_program->log()));
        m_program.reset();
        return false;
    }

    if (!m_program->link()) {
        qWarning("ShaderProgram: linking failed:\n%s",
                 qPrintable(m_program->log()));
        m_program.reset();
        return false;
    }

    return true;
}

bool ShaderProgram::createFromFiles(const std::string& vertPath,
                                     const std::string& fragPath) {
    auto readFile = [](const std::string& path) -> std::string {
        QFile file(QString::fromStdString(path));
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            qWarning("ShaderProgram: cannot open file: %s", path.c_str());
            return {};
        }
        QTextStream in(&file);
        return in.readAll().toStdString();
    };

    std::string vertSrc = readFile(vertPath);
    std::string fragSrc = readFile(fragPath);
    if (vertSrc.empty() || fragSrc.empty()) return false;

    return create(vertSrc, fragSrc);
}

void ShaderProgram::bind() {
    if (m_program) m_program->bind();
}

void ShaderProgram::release() {
    if (m_program) m_program->release();
}

void ShaderProgram::setUniform(const char* name, const math::Mat4& mat) {
    if (!m_program) return;
    // Convert double[4][4] to float[16] in row-major order for QMatrix4x4 constructor.
    // Our Mat4::data() returns row-major (m[row][col]), and QMatrix4x4(const float*)
    // expects row-major input.
    float glMat[16];
    const double* src = mat.data();
    for (int i = 0; i < 16; ++i) {
        glMat[i] = static_cast<float>(src[i]);
    }
    m_program->setUniformValue(name, QMatrix4x4(glMat));
}

void ShaderProgram::setUniform(const char* name, const math::Vec3& vec) {
    if (!m_program) return;
    m_program->setUniformValue(name, QVector3D(static_cast<float>(vec.x),
                                                static_cast<float>(vec.y),
                                                static_cast<float>(vec.z)));
}

void ShaderProgram::setUniform(const char* name, const math::Vec4& vec) {
    if (!m_program) return;
    m_program->setUniformValue(name, QVector4D(static_cast<float>(vec.x),
                                                static_cast<float>(vec.y),
                                                static_cast<float>(vec.z),
                                                static_cast<float>(vec.w)));
}

void ShaderProgram::setUniform(const char* name, float value) {
    if (!m_program) return;
    m_program->setUniformValue(name, value);
}

void ShaderProgram::setUniform(const char* name, int value) {
    if (!m_program) return;
    m_program->setUniformValue(name, value);
}

bool ShaderProgram::isValid() const {
    return m_program && m_program->isLinked();
}

}  // namespace hz::render
