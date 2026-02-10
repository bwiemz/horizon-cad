#include "horizon/ui/SelectTool.h"

#include <QMouseEvent>

namespace hz::ui {

bool SelectTool::mousePressEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    // Stub: selection logic to be implemented in a future phase.
    return false;
}

bool SelectTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool SelectTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

}  // namespace hz::ui
