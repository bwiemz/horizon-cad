#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"
#include "horizon/drafting/DraftEntity.h"
#include <memory>

namespace hz::ui {

/// Offset tool: click an entity, then drag to set offset distance.
///
/// Creates a parallel copy of the entity at the specified distance.
class OffsetTool : public Tool {
public:
    std::string name() const override { return "Offset"; }

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void deactivate() override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;
    std::vector<std::pair<math::Vec2, double>> getPreviewCircles() const override;
    std::vector<ArcPreview> getPreviewArcs() const override;

    std::string promptText() const override;
    bool wantsCrosshair() const override;

private:
    enum class State { SelectEntity, SpecifyDistance };
    State m_state = State::SelectEntity;

    std::shared_ptr<draft::DraftEntity> m_sourceEntity;
    math::Vec2 m_currentPos;

    std::shared_ptr<draft::DraftEntity> computeOffset() const;
    double computeDistanceAndSide(int& side) const;
};

}  // namespace hz::ui
