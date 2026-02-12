#include "horizon/drafting/DraftText.h"
#include "horizon/math/MathUtils.h"
#include <algorithm>
#include <cmath>

namespace hz::draft {

DraftText::DraftText(const math::Vec2& position, const std::string& text,
                     double textHeight)
    : m_position(position), m_text(text), m_textHeight(textHeight) {}

double DraftText::approxWidth() const {
    // Average character width ≈ 0.6 × textHeight.  Minimum 1 char.
    return std::max(1.0, static_cast<double>(m_text.size())) * m_textHeight * 0.6;
}

math::BoundingBox DraftText::boundingBox() const {
    double w = approxWidth();
    double h = m_textHeight;

    // Compute local-space corners relative to anchor based on alignment.
    double x0 = 0.0, x1 = 0.0;
    switch (m_alignment) {
        case TextAlignment::Left:   x0 = 0.0; x1 = w;        break;
        case TextAlignment::Center: x0 = -w * 0.5; x1 = w * 0.5; break;
        case TextAlignment::Right:  x0 = -w; x1 = 0.0;       break;
    }
    double y0 = -h * 0.25;  // baseline offset
    double y1 = h * 0.75;

    // Rotate corners and build bbox.
    double c = std::cos(m_rotation), s = std::sin(m_rotation);
    math::Vec2 corners[4] = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};

    math::BoundingBox bbox;
    for (const auto& corner : corners) {
        math::Vec2 rotated = {corner.x * c - corner.y * s,
                              corner.x * s + corner.y * c};
        bbox.expand(math::Vec3(m_position.x + rotated.x,
                               m_position.y + rotated.y, 0.0));
    }
    return bbox;
}

bool DraftText::hitTest(const math::Vec2& point, double tolerance) const {
    // Inverse-rotate point into text-local space.
    math::Vec2 d = point - m_position;
    double c = std::cos(-m_rotation), s = std::sin(-m_rotation);
    math::Vec2 local = {d.x * c - d.y * s, d.x * s + d.y * c};

    double w = approxWidth();
    double h = m_textHeight;
    double x0 = 0.0, x1 = 0.0;
    switch (m_alignment) {
        case TextAlignment::Left:   x0 = 0.0; x1 = w;        break;
        case TextAlignment::Center: x0 = -w * 0.5; x1 = w * 0.5; break;
        case TextAlignment::Right:  x0 = -w; x1 = 0.0;       break;
    }
    double y0 = -h * 0.25;
    double y1 = h * 0.75;

    return local.x >= x0 - tolerance && local.x <= x1 + tolerance &&
           local.y >= y0 - tolerance && local.y <= y1 + tolerance;
}

std::vector<math::Vec2> DraftText::snapPoints() const {
    return {m_position};
}

void DraftText::translate(const math::Vec2& delta) {
    m_position += delta;
}

std::shared_ptr<DraftEntity> DraftText::clone() const {
    auto copy = std::make_shared<DraftText>(m_position, m_text, m_textHeight);
    copy->setRotation(m_rotation);
    copy->setAlignment(m_alignment);
    copy->setLayer(layer());
    copy->setColor(color());
    copy->setLineWidth(lineWidth());
    return copy;
}

static math::Vec2 mirrorPoint(const math::Vec2& p,
                               const math::Vec2& axisP1,
                               const math::Vec2& axisP2) {
    math::Vec2 d = (axisP2 - axisP1).normalized();
    math::Vec2 v = p - axisP1;
    return axisP1 + d * (2.0 * v.dot(d)) - v;
}

void DraftText::mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) {
    m_position = mirrorPoint(m_position, axisP1, axisP2);

    // Reflect rotation about the axis.
    math::Vec2 d = (axisP2 - axisP1).normalized();
    double axisAngle = std::atan2(d.y, d.x);
    m_rotation = math::normalizeAngle(2.0 * axisAngle - m_rotation);

    // Flip horizontal alignment.
    if (m_alignment == TextAlignment::Left)
        m_alignment = TextAlignment::Right;
    else if (m_alignment == TextAlignment::Right)
        m_alignment = TextAlignment::Left;
}

void DraftText::rotate(const math::Vec2& center, double angle) {
    double c = std::cos(angle), s = std::sin(angle);
    math::Vec2 v = m_position - center;
    m_position = {center.x + v.x * c - v.y * s,
                  center.y + v.x * s + v.y * c};
    m_rotation = math::normalizeAngle(m_rotation + angle);
}

void DraftText::scale(const math::Vec2& center, double factor) {
    m_position = center + (m_position - center) * factor;
    m_textHeight *= std::abs(factor);
}

}  // namespace hz::draft
