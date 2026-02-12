#include "horizon/ui/TextTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/drafting/DraftText.h"

#include <QInputDialog>
#include <QMouseEvent>
#include <QKeyEvent>

namespace hz::ui {

bool TextTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

    // Snap.
    math::Vec2 pos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        pos = result.point;
        m_viewport->setLastSnapResult(result);
    }

    // Ask for text content.
    bool ok = false;
    QString text = QInputDialog::getText(
        m_viewport, QObject::tr("Text"), QObject::tr("Enter text:"),
        QLineEdit::Normal, QString(), &ok);
    if (!ok || text.trimmed().isEmpty()) return true;

    // Get default text height from dimension style.
    double textHeight = 2.5;
    if (m_viewport && m_viewport->document()) {
        textHeight = m_viewport->document()->draftDocument().dimensionStyle().textHeight;
    }

    // Create and add the text entity.
    if (m_viewport && m_viewport->document()) {
        auto txt = std::make_shared<draft::DraftText>(
            pos, text.toStdString(), textHeight);
        txt->setLayer(m_viewport->document()->layerManager().currentLayer());
        auto cmd = std::make_unique<doc::AddEntityCommand>(
            m_viewport->document()->draftDocument(), txt);
        m_viewport->document()->undoStack().push(std::move(cmd));
        m_viewport->update();
    }
    return true;
}

bool TextTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        m_viewport->setLastSnapResult(result);
    }
    return true;
}

bool TextTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool TextTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void TextTool::cancel() {
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
}

std::string TextTool::promptText() const {
    return "Click to place text";
}

bool TextTool::wantsCrosshair() const { return true; }

}  // namespace hz::ui
