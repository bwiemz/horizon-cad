#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Fillet tool: click two lines near their intersection to create a tangent arc.
///
/// - First click selects the first line
/// - Second click selects the second line and creates the fillet
/// - Type a number + Enter to change the fillet radius
class FilletTool : public Tool {
public:
    std::string name() const override { return "Fillet"; }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;
    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<ArcPreview> getPreviewArcs() const override;

    std::string promptText() const override;
    bool wantsCrosshair() const override;

private:
    enum class State { SelectFirstLine, SelectSecondLine };
    State m_state = State::SelectFirstLine;

    double m_filletRadius = 1.0;
    std::string m_radiusInput;

    uint64_t m_firstEntityId = 0;
    math::Vec2 m_firstClickPos;

    // Preview state.
    math::Vec2 m_currentPos;
    mutable bool m_hasPreview = false;
    mutable math::Vec2 m_previewArcCenter;
    mutable double m_previewArcRadius = 0.0;
    mutable double m_previewArcStart = 0.0;
    mutable double m_previewArcEnd = 0.0;

    bool computeFillet(uint64_t lineAId, const math::Vec2& clickA,
                       uint64_t lineBId, const math::Vec2& clickB,
                       math::Vec2& arcCenter, double& arcRadius,
                       double& arcStartAngle, double& arcEndAngle,
                       math::Vec2& trimA_start, math::Vec2& trimA_end,
                       math::Vec2& trimB_start, math::Vec2& trimB_end) const;
};

}  // namespace hz::ui
