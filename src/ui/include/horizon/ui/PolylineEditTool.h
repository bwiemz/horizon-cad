#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"
#include "horizon/drafting/DraftEntity.h"
#include <memory>

namespace hz::ui {

/// Polyline edit tool: click a polyline to enter vertex editing mode.
///
/// Sub-modes (keyboard while editing):
/// - Default: move vertices (click+drag)
/// - A: add vertex on nearest segment
/// - D: remove vertex (min 2 preserved)
/// - C: toggle closed/open
/// - J: join with another polyline
/// - Escape/Enter: finish editing
class PolylineEditTool : public Tool {
public:
    std::string name() const override { return "PolylineEdit"; }

    void activate(ViewportWidget* viewport) override;
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
    enum class Mode { MoveVertex, AddVertex, RemoveVertex, JoinPolyline };
    Mode m_mode = Mode::MoveVertex;

    // Currently-editing polyline.
    uint64_t m_editEntityId = 0;

    // Vertex dragging state.
    bool m_dragging = false;
    int m_dragVertexIndex = -1;
    std::shared_ptr<draft::DraftEntity> m_beforeClone;
    math::Vec2 m_currentPos;

    void finishEditing();
    void pushSnapshot(const std::string& desc);
    int findNearestVertex(const math::Vec2& worldPos, double tolerance) const;
    int findNearestSegment(const math::Vec2& worldPos, math::Vec2& closestPt) const;
};

}  // namespace hz::ui
