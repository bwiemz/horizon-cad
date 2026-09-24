// The Extrude command asks how the new body combines with the part, and a
// feature that would fail (a cut that leaves nothing) is refused, not added.

#include <gtest/gtest.h>

#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QStatusBar>
#include <QTimer>
#include <memory>
#include <optional>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/MainWindow.h"

using hz::doc::BodyOperation;
using hz::math::Vec2;
using hz::ui::MainWindow;

namespace {

/// Fills in the next feature dialog titled `title`: remembers the operation
/// it proposed, then sets `value` and (if given) `operation` and accepts.
class FeatureDialogFiller {
public:
    FeatureDialogFiller(QString title, double value, std::optional<BodyOperation> operation)
        : m_title(std::move(title)), m_value(value), m_operation(operation) {
        QObject::connect(&m_timer, &QTimer::timeout, [this] { poll(); });
        m_timer.start(5);
        m_clock.start();
    }
    bool seen() const { return m_proposed.has_value(); }
    std::optional<BodyOperation> proposed() const { return m_proposed; }

private:
    void poll() {
        auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
        if (dialog == nullptr || dialog->windowTitle() != m_title) {
            if (m_clock.elapsed() > 5000) m_timer.stop();
            return;
        }
        m_timer.stop();
        auto* combo = dialog->findChild<QComboBox*>(QStringLiteral("bodyOperation"));
        auto* spin = dialog->findChild<QDoubleSpinBox*>();
        if (combo == nullptr || spin == nullptr) {
            dialog->reject();
            return;
        }
        m_proposed = static_cast<BodyOperation>(combo->currentData().toInt());
        spin->setValue(m_value);
        if (m_operation) combo->setCurrentIndex(combo->findData(static_cast<int>(*m_operation)));
        dialog->accept();
    }

    QString m_title;
    double m_value;
    std::optional<BodyOperation> m_operation;
    std::optional<BodyOperation> m_proposed;
    QTimer m_timer;
    QElapsedTimer m_clock;
};

void drawRectangle(hz::doc::Document& doc, double x0, double y0, double x1, double y1) {
    auto& d = doc.draftDocument();
    d.clear();
    d.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(x0, y0), Vec2(x1, y0)));
    d.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(x1, y0), Vec2(x1, y1)));
    d.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(x1, y1), Vec2(x0, y1)));
    d.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(x0, y1), Vec2(x0, y0)));
}

double partVolume(const hz::doc::Document& doc) {
    return doc.solid() ? hz::model::MassPropertiesCalculator::compute(*doc.solid()).volume : 0.0;
}

}  // namespace

TEST(BodyFeatureCommandsTest, ExtrudeAsksForTheOperationAndRefusesACutThatLeavesNothing) {
    MainWindow w;
    hz::doc::Document& doc = *w.activeDocument();
    auto* extrude = w.findChild<QAction*>(QStringLiteral("action_extrude"));
    ASSERT_NE(extrude, nullptr);

    // First body: nothing to join yet, so the dialog proposes a new body.
    drawRectangle(doc, 0, 0, 10, 10);
    {
        FeatureDialogFiller filler(QStringLiteral("Extrude"), 2.0, std::nullopt);
        extrude->trigger();
        ASSERT_TRUE(filler.seen());
        EXPECT_EQ(filler.proposed(), BodyOperation::NewBody);
    }
    ASSERT_EQ(doc.featureTree().featureCount(), 1u);
    EXPECT_NEAR(partVolume(doc), 200.0, 1e-6);
    const size_t sketches = doc.sketches().size();

    // A cut through everything would leave nothing: refused, part unchanged.
    drawRectangle(doc, -5, -5, 15, 15);
    {
        FeatureDialogFiller filler(QStringLiteral("Extrude"), 5.0, BodyOperation::Cut);
        extrude->trigger();
        ASSERT_TRUE(filler.seen());
        EXPECT_EQ(filler.proposed(), BodyOperation::Join) << "a part with a body proposes Join";
    }
    EXPECT_EQ(doc.featureTree().featureCount(), 1u) << "the failing cut is not kept";
    EXPECT_EQ(doc.sketches().size(), sketches) << "nor the profile sketch made for it";
    EXPECT_NEAR(partVolume(doc), 200.0, 1e-6);
    EXPECT_TRUE(w.statusBar()->currentMessage().contains("not added"))
        << w.statusBar()->currentMessage().toStdString();

    // A real pocket goes in.
    drawRectangle(doc, 4, 4, 6, 6);
    {
        FeatureDialogFiller filler(QStringLiteral("Extrude"), 2.0, BodyOperation::Cut);
        extrude->trigger();
        ASSERT_TRUE(filler.seen());
    }
    EXPECT_EQ(doc.featureTree().featureCount(), 2u);
    EXPECT_NEAR(partVolume(doc), 200.0 - 8.0, 1e-6);
    EXPECT_TRUE(doc.isDirty());
}
