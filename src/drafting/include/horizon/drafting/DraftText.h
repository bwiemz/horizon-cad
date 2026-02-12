#pragma once

#include "DraftEntity.h"
#include <string>

namespace hz::draft {

enum class TextAlignment { Left, Center, Right };

class DraftText : public DraftEntity {
public:
    DraftText(const math::Vec2& position, const std::string& text,
              double textHeight = 2.5);

    math::BoundingBox boundingBox() const override;
    bool hitTest(const math::Vec2& point, double tolerance) const override;
    std::vector<math::Vec2> snapPoints() const override;
    void translate(const math::Vec2& delta) override;
    std::shared_ptr<DraftEntity> clone() const override;
    void mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) override;
    void rotate(const math::Vec2& center, double angle) override;
    void scale(const math::Vec2& center, double factor) override;

    const math::Vec2& position() const { return m_position; }
    void setPosition(const math::Vec2& pos) { m_position = pos; }

    const std::string& text() const { return m_text; }
    void setText(const std::string& text) { m_text = text; }

    double textHeight() const { return m_textHeight; }
    void setTextHeight(double height) { m_textHeight = height; }

    double rotation() const { return m_rotation; }
    void setRotation(double angle) { m_rotation = angle; }

    TextAlignment alignment() const { return m_alignment; }
    void setAlignment(TextAlignment align) { m_alignment = align; }

private:
    /// Approximate width of the text in world units.
    double approxWidth() const;

    math::Vec2 m_position;
    std::string m_text;
    double m_textHeight = 2.5;
    double m_rotation = 0.0;
    TextAlignment m_alignment = TextAlignment::Center;
};

}  // namespace hz::draft
