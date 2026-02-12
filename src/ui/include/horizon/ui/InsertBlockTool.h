#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/drafting/BlockDefinition.h"
#include "horizon/math/Vec2.h"
#include <memory>
#include <string>

namespace hz::ui {

/// Insert-block tool: click to place block reference instances.
///
/// Stays active for repeated placements until Escape.
class InsertBlockTool : public Tool {
public:
    InsertBlockTool(std::shared_ptr<draft::BlockDefinition> definition,
                    double rotation, double scale);

    std::string name() const override { return "Insert Block"; }

    void deactivate() override;
    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;
    std::vector<std::pair<math::Vec2, double>> getPreviewCircles() const override;

    std::string promptText() const override;
    bool wantsCrosshair() const override;

private:
    std::shared_ptr<draft::BlockDefinition> m_definition;
    double m_rotation;
    double m_scale;
    math::Vec2 m_currentPos;
};

}  // namespace hz::ui
