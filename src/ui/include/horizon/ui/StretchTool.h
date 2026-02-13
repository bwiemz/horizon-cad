#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"
#include "horizon/drafting/DraftEntity.h"

#include <memory>
#include <vector>

namespace hz::ui {

/// Stretch tool: draw a crossing window to select vertices, then specify
/// base point + destination.  Vertices inside the window move by the
/// displacement; vertices outside stay fixed.  If all vertices of an entity
/// are inside the window, the entity translates entirely.
class StretchTool : public Tool {
public:
    std::string name() const override { return "Stretch"; }

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void deactivate() override;
    void cancel() override;

    std::string promptText() const override;
    bool wantsCrosshair() const override;
    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;
    math::Vec3 previewColor() const override;

private:
    enum class State {
        SelectingWindow,   // Waiting for first click of crossing window
        DraggingWindow,    // Drawing the crossing rectangle
        WaitingBasePoint,  // Window defined, waiting for base point click
        Dragging           // Base point set, dragging to destination
    };
    State m_state = State::SelectingWindow;

    // Crossing window corners
    math::Vec2 m_windowStart;
    math::Vec2 m_windowEnd;

    // Base point / current drag position
    math::Vec2 m_basePoint;
    math::Vec2 m_currentPos;

    /// Data for one entity affected by the stretch.
    struct StretchEntity {
        uint64_t entityId = 0;
        std::shared_ptr<draft::DraftEntity> beforeClone;
        std::vector<int> insideIndices;  // which stretch point indices are in window
        int totalPoints = 0;             // total stretch points for this entity
    };
    std::vector<StretchEntity> m_stretchEntities;

    void collectStretchEntities();
    void applyCurrentStretch();
    void restoreAllEntities();
    void resetState();
};

}  // namespace hz::ui
