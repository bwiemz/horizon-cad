// File ▸ Import and File ▸ Export (Phase 107): STEP, STL, glTF and DXF are
// reachable from the window, and a file that is not read whole says so.

#include <gtest/gtest.h>

#include <QAction>
#include <QFile>
#include <QFileInfo>
#include <QMessageBox>
#include <QStatusBar>
#include <QTemporaryDir>
#include <fstream>
#include <memory>
#include <string>
#include <utility>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/BlockDefinition.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/Layer.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/MainWindow.h"

using hz::math::Vec2;
using hz::test::DialogResponder;
using hz::test::FilePicker;
using hz::test::FormAnswers;
using hz::test::FormFiller;
using hz::ui::MainWindow;

namespace {

QAction* action(MainWindow& w, const char* name) {
    auto* found = w.findChild<QAction*>(QString::fromLatin1(name));
    EXPECT_NE(found, nullptr) << name;
    return found;
}

QAction* menuAction(MainWindow& w, const QString& text) {
    for (QAction* a : w.findChildren<QAction*>()) {
        if (a->text() == text) return a;
    }
    ADD_FAILURE() << "no menu item " << text.toStdString();
    return nullptr;
}

double partVolume(const hz::doc::Document& doc) {
    return doc.solid() ? hz::model::MassPropertiesCalculator::compute(*doc.solid()).volume : 0.0;
}

/// A 2 x 3 x 4 box made with the Box command.
void makeBox(MainWindow& w) {
    FormFiller filler(QStringLiteral("Box"), FormAnswers()
                                                 .number(QStringLiteral("size0"), 2.0)
                                                 .number(QStringLiteral("size1"), 3.0)
                                                 .number(QStringLiteral("size2"), 4.0));
    action(w, "action_box")->trigger();
    ASSERT_TRUE(filler.seen());
    ASSERT_NEAR(partVolume(*w.activeDocument()), 24.0, 1e-9);
}

}  // namespace

TEST(ImportExportTest, APartExportsAsStepStlAndGltf) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    MainWindow w;
    makeBox(w);

    for (const auto& [name, file] :
         {std::pair{"export_step", "part.step"}, std::pair{"export_stl", "part.stl"},
          std::pair{"export_gltf", "part.glb"}}) {
        const QString path = dir.filePath(QString::fromLatin1(file));
        {
            FilePicker picker(path);
            action(w, name)->trigger();
        }
        EXPECT_TRUE(QFileInfo::exists(path)) << file;
        EXPECT_GT(QFileInfo(path).size(), 84) << file;
    }

    // And what was written as STEP reads back as the same body.
    const auto solids =
        hz::io::StepFormat::load(dir.filePath(QStringLiteral("part.step")).toStdString());
    ASSERT_EQ(solids.size(), 1u);
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*solids[0]).volume, 24.0, 1e-9);
}

TEST(ImportExportTest, ImportingStepMakesAPartOfItsBodies) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString path = dir.filePath(QStringLiteral("bracket.step"));
    auto box = hz::model::PrimitiveFactory::makeBox(3, 4, 5);
    ASSERT_TRUE(hz::io::StepFormat::save(path.toStdString(), {box.get()}));

    MainWindow w;
    {
        FilePicker picker(path);
        action(w, "import_step")->trigger();
    }
    hz::doc::Document& doc = *w.activeDocument();
    ASSERT_EQ(doc.featureTree().featureCount(), 1u);
    EXPECT_NE(dynamic_cast<const hz::doc::ImportedBodyFeature*>(doc.featureTree().feature(0)),
              nullptr);
    EXPECT_NEAR(partVolume(doc), 60.0, 1e-9);
    EXPECT_TRUE(doc.isDirty()) << "an imported part has not been saved anywhere";
}

TEST(ImportExportTest, ImportingDxfAddsToTheDrawingInOneUndoableStep) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString path = dir.filePath(QStringLiteral("plan.dxf"));
    {
        hz::doc::Document source;
        source.draftDocument().addEntity(
            std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(5, 0)));
        source.draftDocument().addEntity(
            std::make_shared<hz::draft::DraftLine>(Vec2(5, 0), Vec2(5, 5)));
        std::string error;
        ASSERT_TRUE(hz::io::DxfFormat::save(path.toStdString(), source, &error)) << error;
    }

    MainWindow w;
    hz::doc::Document& doc = *w.activeDocument();
    doc.draftDocument().addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(-1, 0)));
    {
        FilePicker picker(path);
        action(w, "import_dxf")->trigger();
    }
    EXPECT_EQ(doc.draftDocument().entities().size(), 3u);
    action(w, "action_undo")->trigger();
    EXPECT_EQ(doc.draftDocument().entities().size(), 1u) << "one step takes the import away";
}

TEST(ImportExportTest, UndoingADxfImportTakesItsLayersAndBlocksToo) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString path = dir.filePath(QStringLiteral("bolts.dxf"));
    {
        hz::doc::Document source;
        hz::draft::LayerProperties dims;
        dims.name = "Dimensions";
        source.layerManager().addLayer(dims);
        auto bolt = std::make_shared<hz::draft::BlockDefinition>();
        bolt->name = "Bolt";
        bolt->entities.push_back(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 0)));
        source.draftDocument().blockTable().addBlock(bolt);
        source.draftDocument().addEntity(
            std::make_shared<hz::draft::DraftBlockRef>(bolt, Vec2(5, 5)));
        auto line = std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(2, 0));
        line->setLayer("Dimensions");
        source.draftDocument().addEntity(line);
        std::string error;
        ASSERT_TRUE(hz::io::DxfFormat::save(path.toStdString(), source, &error)) << error;
    }

    MainWindow w;
    hz::doc::Document& doc = *w.activeDocument();
    {
        FilePicker picker(path);
        action(w, "import_dxf")->trigger();
    }
    ASSERT_NE(doc.layerManager().getLayer("Dimensions"), nullptr);
    ASSERT_NE(doc.draftDocument().blockTable().findBlock("Bolt"), nullptr);

    action(w, "action_undo")->trigger();
    EXPECT_TRUE(doc.draftDocument().entities().empty());
    EXPECT_EQ(doc.layerManager().getLayer("Dimensions"), nullptr) << "the import's layer goes too";
    EXPECT_EQ(doc.draftDocument().blockTable().findBlock("Bolt"), nullptr) << "and its block";
}

TEST(ImportExportTest, OpeningAFileWithDamagedItemsSaysWhatWasLeftOut) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString path = dir.filePath(QStringLiteral("damaged.hcad"));
    {
        std::ofstream out(path.toStdString());
        out << R"({"version": 16, "type": "hcad", "entities": [
            {"type": "line", "id": 1, "start": {"x": 0, "y": 0}, "end": {"x": 1, "y": 0}},
            {"type": "line", "id": 2, "start": {"x": 0, "y": 0}}]})";
    }

    MainWindow w;
    DialogResponder report(QMessageBox::Ok, QStringLiteral("Not Everything Was Read"));
    {
        FilePicker picker(path);
        menuAction(w, QStringLiteral("&Open..."))->trigger();
    }
    report.waitForDialog(5000);
    ASSERT_TRUE(report.seen()) << "the omission is reported";
    EXPECT_TRUE(report.text().contains(QStringLiteral("1 item was left out")))
        << report.text().toStdString();
    EXPECT_EQ(w.activeDocument()->draftDocument().entities().size(), 1u);
}

TEST(ImportExportTest, AnInchDrawingIsScaledAndSaysSoWithoutAWarning) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString path = dir.filePath(QStringLiteral("inches.dxf"));
    {
        std::ofstream out(path.toStdString());
        out << "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n1\n0\nENDSEC\n"
               "0\nSECTION\n2\nENTITIES\n0\nLINE\n8\n0\n10\n0\n20\n0\n11\n1\n21\n0\n"
               "0\nENDSEC\n0\nEOF\n";
    }

    MainWindow w;
    DialogResponder warning(QMessageBox::Ok, QStringLiteral("Not Everything Was Read"));
    {
        FilePicker picker(path);
        action(w, "import_dxf")->trigger();
    }
    EXPECT_FALSE(warning.seen()) << "nothing was lost, so nothing to warn about";
    EXPECT_TRUE(w.statusBar()->currentMessage().contains(QStringLiteral("scaled by 25.4")))
        << w.statusBar()->currentMessage().toStdString();

    const auto& entities = w.activeDocument()->draftDocument().entities();
    ASSERT_EQ(entities.size(), 1u);
    const auto* line = dynamic_cast<const hz::draft::DraftLine*>(entities[0].get());
    ASSERT_NE(line, nullptr);
    EXPECT_NEAR((line->end() - line->start()).length(), 25.4, 1e-9);
}

TEST(ImportExportTest, AStepFileInMetresComesInAsMillimetresAndSaysSo) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString path = dir.filePath(QStringLiteral("metres.step"));
    {
        auto box = hz::model::PrimitiveFactory::makeBox(1, 2, 3);
        std::string text = hz::io::StepFormat::toString({box.get()});
        const std::string mm = "SI_UNIT(.MILLI.,.METRE.)";
        const size_t at = text.find(mm);
        ASSERT_NE(at, std::string::npos);
        text.replace(at, mm.size(), "SI_UNIT($,.METRE.)");
        std::ofstream out(path.toStdString(), std::ios::binary);
        out << text;
    }

    MainWindow w;
    DialogResponder warning(QMessageBox::Ok, QStringLiteral("Not Everything Was Read"));
    {
        FilePicker picker(path);
        action(w, "import_step")->trigger();
    }
    EXPECT_FALSE(warning.seen()) << "nothing was lost, so nothing to warn about";
    EXPECT_TRUE(w.statusBar()->currentMessage().contains(QStringLiteral("metres")))
        << w.statusBar()->currentMessage().toStdString();
    EXPECT_NEAR(partVolume(*w.activeDocument()), 6.0e9, 6.0e9 * 1e-9);
}

// -- Plotting (Phase 127) ------------------------------------------------------

namespace {

hz::test::FormAnswers plotAnswers(const char* scale) {
    return hz::test::FormAnswers()
        .choose(QStringLiteral("paper"), QStringLiteral("A4"))
        .choose(QStringLiteral("orientation"), QStringLiteral("Landscape"))
        .choose(QStringLiteral("scale"), QString::fromLatin1(scale))
        .choose(QStringLiteral("colours"), QStringLiteral("As drawn"));
}

void drawALine(MainWindow& w) {
    w.activeDocument()->draftDocument().addEntity(
        std::make_shared<hz::draft::DraftLine>(hz::math::Vec2(0, 0), hz::math::Vec2(100, 40)));
}

QByteArray contents(const QString& path) {
    QFile file(path);
    return file.open(QIODevice::ReadOnly) ? file.readAll() : QByteArray();
}

}  // namespace

// A drawing plots to PDF and to SVG from the Export menu.
TEST(ImportExportTest, ADrawingPlotsToPdfAndSvg) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    MainWindow w;
    drawALine(w);
    for (const char* format : {"pdf", "svg"}) {
        const QString path = dir.filePath(QStringLiteral("plot.") + QString::fromLatin1(format));
        hz::test::FormFiller form(QStringLiteral("Export ") + QString::fromLatin1(format).toUpper(),
                                  plotAnswers("Fit to paper"));
        {
            FilePicker picker(path);
            w.findChild<QAction*>(QStringLiteral("export_") + QString::fromLatin1(format))
                ->trigger();
        }
        ASSERT_TRUE(form.seen()) << format;
        const QByteArray bytes = contents(path);
        if (QString::fromLatin1(format) == QStringLiteral("pdf")) {
            EXPECT_TRUE(bytes.startsWith("%PDF-")) << "a PDF";
        } else {
            EXPECT_TRUE(bytes.contains("<svg ")) << "an SVG";
            EXPECT_TRUE(bytes.contains("<polyline ")) << "with the line";
        }
    }
}

// At a scale the drawing does not fit on the paper, the user is told before
// anything is cut off; Cancel writes nothing.
TEST(ImportExportTest, APlotThatDoesNotFitIsSaidFirst) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    MainWindow w;
    drawALine(w);
    const QString path = dir.filePath(QStringLiteral("big.svg"));
    hz::test::FormFiller form(QStringLiteral("Export SVG"), plotAnswers("10:1"));
    DialogResponder cancel(QMessageBox::Cancel, QStringLiteral("Export SVG"), 5000);
    w.findChild<QAction*>(QStringLiteral("export_svg"))->trigger();
    ASSERT_TRUE(form.seen());
    EXPECT_TRUE(cancel.seen()) << "the drawing is 1000 mm across at 10:1";
    EXPECT_TRUE(cancel.text().contains(QStringLiteral("cut off"))) << cancel.text().toStdString();
    EXPECT_FALSE(QFileInfo::exists(path));
}
