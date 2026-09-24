#include "horizon/ui/ChainDimensionTool.h"

#include <QKeyEvent>
#include <QMouseEvent>
#include <memory>

#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/ui/ViewportWidget.h"

namespace hz::ui {

using Orientation = draft::DraftLinearDimension::Orientation;

namespace {

/// Whether a tool may work from @p e: its layer is shown and not locked.
bool usable(const doc::Document& doc, const draft::DraftEntity& e) {
    const auto* layer = doc.layerManager().getLayer(e.layer());
    return layer == nullptr || (layer->visible && !layer->locked);
}

}  // namespace

void ChainDimensionTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_chain.reset();
    if (const auto* last = lastDimension()) m_chain = chainFrom(*last);
}

void ChainDimensionTool::deactivate() {
    m_chain.reset();
    if (m_viewport) m_viewport->setLastSnapResult({});
    Tool::deactivate();
}

std::optional<ChainDimensionTool::Chain> ChainDimensionTool::chainFrom(
    const draft::DraftLinearDimension& base) const {
    if (base.orientation() == Orientation::Aligned) return std::nullopt;
    const bool horizontal = base.orientation() == Orientation::Horizontal;
    // The coordinate across the dimension: y for a horizontal one.
    const auto across = [horizontal](const math::Vec2& p) { return horizontal ? p.y : p.x; };
    Chain chain;
    chain.orientation = base.orientation();
    chain.lineAt = across(base.dimLinePoint());
    const double points = (across(base.defPoint1()) + across(base.defPoint2())) / 2.0;
    chain.outward = chain.lineAt >= points ? 1.0 : -1.0;
    // A line and a half of text apart, as drafting standards space them.
    double textHeight = 2.5;
    if (m_viewport && m_viewport->document()) {
        textHeight = m_viewport->document()->draftDocument().dimensionStyle().textHeight;
    }
    chain.step = 1.5 * textHeight;
    chain.from = m_mode == Mode::Continue ? base.defPoint2() : base.defPoint1();
    return chain;
}

std::shared_ptr<draft::DraftLinearDimension> ChainDimensionTool::next(
    const math::Vec2& point) const {
    if (!m_chain) return nullptr;
    const Chain& c = *m_chain;
    const double lineAt = m_mode == Mode::Baseline ? c.lineAt + c.outward * c.step : c.lineAt;
    const math::Vec2 dimLine = c.orientation == Orientation::Horizontal
                                   ? math::Vec2(point.x, lineAt)
                                   : math::Vec2(lineAt, point.y);
    return std::make_shared<draft::DraftLinearDimension>(c.from, point, dimLine, c.orientation);
}

std::optional<math::Vec2> ChainDimensionTool::basePoint() const {
    if (!m_chain) return std::nullopt;
    return m_chain->from;
}

const draft::DraftLinearDimension* ChainDimensionTool::lastDimension() const {
    if (!m_viewport || !m_viewport->document()) return nullptr;
    const doc::Document& doc = *m_viewport->document();
    const auto& entities = doc.draftDocument().entities();
    for (auto it = entities.rbegin(); it != entities.rend(); ++it) {
        const auto* dim = dynamic_cast<const draft::DraftLinearDimension*>(it->get());
        if (dim && dim->orientation() != Orientation::Aligned && usable(doc, *dim)) return dim;
    }
    return nullptr;
}

const draft::DraftLinearDimension* ChainDimensionTool::dimensionAt(const math::Vec2& point) const {
    if (!m_viewport || !m_viewport->document()) return nullptr;
    const doc::Document& doc = *m_viewport->document();
    const double reach = m_viewport->pickTolerance();
    const math::BoundingBox around(math::Vec3(point.x - reach, point.y - reach, -1.0),
                                   math::Vec3(point.x + reach, point.y + reach, 1.0));
    for (const uint64_t id : doc.draftDocument().spatialIndex().query(around)) {
        const auto* dim =
            dynamic_cast<const draft::DraftLinearDimension*>(doc.draftDocument().findEntity(id));
        if (dim && dim->orientation() != Orientation::Aligned && usable(doc, *dim) &&
            dim->hitTest(point, reach)) {
            return dim;
        }
    }
    return nullptr;
}

bool ChainDimensionTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton || !m_viewport || !m_viewport->document()) return false;
    if (!m_chain) {
        if (const auto* picked = dimensionAt(worldPos)) m_chain = chainFrom(*picked);
        return true;
    }
    const auto result = m_viewport->snap(worldPos);
    m_viewport->setLastSnapResult(result);
    if (result.point.distanceTo(m_chain->from) < 1e-9) return true;  // nothing to measure

    auto dim = next(result.point);
    doc::Document& doc = *m_viewport->document();
    dim->setLayer(doc.layerManager().currentLayer());
    doc.undoStack().push(std::make_unique<doc::AddEntityCommand>(doc.draftDocument(), dim));

    // The next one continues from this one's end, or steps further out.
    if (m_mode == Mode::Continue) {
        m_chain->from = result.point;
    } else {
        m_chain->lineAt += m_chain->outward * m_chain->step;
    }
    return true;
}

bool ChainDimensionTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (!m_chain || !m_viewport || !m_viewport->document()) return false;
    const auto result = m_viewport->snap(worldPos);
    m_viewport->setLastSnapResult(result);
    m_cursor = result.point;
    return true;
}

bool ChainDimensionTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool ChainDimensionTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
        m_chain.reset();  // pick another dimension to go on from
        return true;
    }
    return false;
}

void ChainDimensionTool::cancel() {
    m_chain.reset();
    if (m_viewport) m_viewport->setLastSnapResult({});
}

std::vector<std::pair<math::Vec2, math::Vec2>> ChainDimensionTool::getPreviewLines() const {
    if (!m_chain || !m_viewport || !m_viewport->document()) return {};
    if (m_cursor.distanceTo(m_chain->from) < 1e-9) return {};
    const auto dim = next(m_cursor);
    const auto& style = m_viewport->document()->draftDocument().dimensionStyle();
    auto lines = dim->extensionLines(style);
    for (const auto& line : dim->dimensionLines(style)) lines.push_back(line);
    return lines;
}

std::string ChainDimensionTool::promptText() const {
    if (!m_chain) {
        return m_mode == Mode::Continue
                   ? "Select a horizontal or vertical dimension to continue"
                   : "Select a horizontal or vertical dimension to measure from";
    }
    return "Specify the next point (Enter: another dimension)";
}

}  // namespace hz::ui
