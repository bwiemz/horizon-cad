#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Chamfer tool: click two lines near their intersection to create a bevel.
///
/// - First click selects the first line
/// - Second click selects the second line and creates the chamfer
/// - Type a number + Enter to change the chamfer distance
class ChamferTool : public Tool {
public:
    std::string name() const override { return "Chamfer"; }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;
    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;

    std::string promptText() const override;
    bool wantsCrosshair() const override;

private:
    enum class State { SelectFirstLine, SelectSecondLine };
    State m_state = State::SelectFirstLine;

    double m_chamferDist = 1.0;
    std::string m_distInput;

    uint64_t m_firstEntityId = 0;
    math::Vec2 m_firstClickPos;

    math::Vec2 m_currentPos;

    bool computeChamfer(uint64_t lineAId, const math::Vec2& clickA,
                        uint64_t lineBId, const math::Vec2& clickB,
                        math::Vec2& chamferPtA, math::Vec2& chamferPtB,
                        math::Vec2& trimA_start, math::Vec2& trimA_end,
                        math::Vec2& trimB_start, math::Vec2& trimB_end) const;
};

}  // namespace hz::ui
