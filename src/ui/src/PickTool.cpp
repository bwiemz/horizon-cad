#include "horizon/ui/PickTool.h"

#include <QKeyEvent>
#include <QMouseEvent>
#include <algorithm>

#include "horizon/ui/ViewportWidget.h"

namespace hz::ui {

PickTool::PickTool(std::string name, std::vector<std::string> prompts, Accept accept, Done done)
    : m_name(std::move(name)),
      m_prompts(std::move(prompts)),
      m_accept(std::move(accept)),
      m_done(std::move(done)) {}

void PickTool::setPreview(Lines lines, Circles circles) {
    m_lines = std::move(lines);
    m_circles = std::move(circles);
}

void PickTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    reset();
}

void PickTool::deactivate() {
    reset();
    if (m_viewport) m_viewport->setLastSnapResult({});
    Tool::deactivate();
}

void PickTool::reset() {
    m_picked.clear();
    m_refusal.clear();
    m_complete = false;
}

math::Vec2 PickTool::snapped(const math::Vec2& p) {
    if (m_viewport == nullptr || m_viewport->document() == nullptr) return p;
    const auto result = m_viewport->snap(p);
    m_viewport->setLastSnapResult(result);
    return result.point;
}

bool PickTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton || m_complete) return false;
    const math::Vec2 point = snapped(worldPos);
    std::string why;
    if (m_accept && !m_accept(m_picked.size(), point, why)) {
        m_refusal = why;
        return true;  // taken, as a click: its release is this tool's too
    }
    m_refusal.clear();
    m_picked.push_back(point);
    m_pointer = point;
    m_complete = m_picked.size() >= m_prompts.size();
    return true;
}

bool PickTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (m_picked.empty()) return false;
    m_pointer = snapped(worldPos);
    return true;
}

bool PickTool::mouseReleaseEvent(QMouseEvent* event, const math::Vec2& /*worldPos*/) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_complete) return !m_picked.empty() || !m_refusal.empty();
    // Handed over when the button is up: the command may end this tool.
    const std::vector<math::Vec2> points = m_picked;
    reset();
    if (m_done) m_done(points);
    return true;
}

bool PickTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void PickTool::cancel() {
    reset();
    if (m_viewport) m_viewport->setLastSnapResult({});
}

std::optional<math::Vec2> PickTool::basePoint() const {
    if (m_picked.empty()) return std::nullopt;
    return m_picked.front();
}

std::vector<std::pair<math::Vec2, math::Vec2>> PickTool::getPreviewLines() const {
    if (m_picked.empty() || !m_lines) return {};
    return m_lines(m_picked, m_pointer);
}

std::vector<std::pair<math::Vec2, double>> PickTool::getPreviewCircles() const {
    if (m_picked.empty() || !m_circles) return {};
    return m_circles(m_picked, m_pointer);
}

std::string PickTool::promptText() const {
    const std::size_t step = std::min(m_picked.size(), m_prompts.size() - 1);
    const std::string prompt = m_prompts.empty() ? std::string() : m_prompts[step];
    return m_refusal.empty() ? prompt : m_refusal + ". " + prompt;
}

}  // namespace hz::ui
