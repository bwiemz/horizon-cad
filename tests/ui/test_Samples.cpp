// Samples (Phase 168b): what File > Open Sample lists, that each sample opens
// and builds, and that it is opened as a copy, which can be saved and is kept.

#include <gtest/gtest.h>

#include <QAction>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMenu>
#include <QMessageBox>
#include <QStringList>
#include <QTemporaryDir>
#include <string>

#include "UiTestSupport.h"
#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/RecentFiles.h"

using hz::ui::MainWindow;

#ifdef HZ_BUILT_SAMPLES_DIR

namespace {

/// The samples are the built ones, and their copies go to a folder of the
/// test's own, while this lives.
class SampleFolders {
public:
    SampleFolders() {
        qputenv("HZ_SAMPLES_DIR", HZ_BUILT_SAMPLES_DIR);
        qputenv("HZ_SAMPLE_COPIES_DIR", QFile::encodeName(m_copies.path()));
    }
    ~SampleFolders() {
        qunsetenv("HZ_SAMPLES_DIR");
        qunsetenv("HZ_SAMPLE_COPIES_DIR");
    }
    SampleFolders(const SampleFolders&) = delete;
    SampleFolders& operator=(const SampleFolders&) = delete;

    QString copy(const QString& file) const { return QDir(m_copies.path()).filePath(file); }

private:
    QTemporaryDir m_copies;
};

size_t countOn(const hz::doc::Document& doc, const std::string& layer) {
    size_t n = 0;
    for (const auto& entity : doc.draftDocument().entities()) n += entity->layer() == layer;
    return n;
}

double volumeOf(const hz::doc::Document& doc) {
    return doc.solid() ? hz::model::MassPropertiesCalculator::compute(*doc.solid()).volume : 0.0;
}

/// Open sample @p file in @p w, built on this thread.
hz::doc::Document* open(MainWindow& w, const char* file) {
    w.setRebuildMode(MainWindow::RebuildMode::Never);
    EXPECT_TRUE(w.openSample(QString::fromLatin1(file))) << file;
    return w.activeDocument();
}

}  // namespace

TEST(SamplesTest, OpenSampleListsEverySample) {
    SampleFolders folders;
    MainWindow w;
    auto* menu = w.findChild<QMenu*>(QStringLiteral("sampleMenu"));
    ASSERT_NE(menu, nullptr);
    emit menu->aboutToShow();
    QStringList names;
    for (const QAction* item : menu->actions()) names << item->text();
    EXPECT_EQ(names, (QStringList{"Bracket (part)", "Bracket drawing (drawing sheet)",
                                  "Gasket (2D drawing)", "Pin (part)", "Plate (part)",
                                  "Plate and pin (assembly)"}));
}

// Each part is built, every feature of it.
TEST(SamplesTest, EveryPartBuilds) {
    SampleFolders folders;
    for (const auto& [file, features] :
         {std::pair{"bracket.hzpart", 4u}, std::pair{"plate.hzpart", 3u},
          std::pair{"pin.hzpart", 1u}}) {
        MainWindow w;
        const hz::doc::Document* doc = open(w, file);
        ASSERT_NE(doc, nullptr);
        EXPECT_EQ(doc->featureTree().featureCount(), features) << file;
        EXPECT_EQ(doc->failedFeatureIndex(), -1) << file << ": " << doc->lastBuildMessage();
        EXPECT_GT(volumeOf(*doc), 0.0) << file;
    }
}

// The bracket: an L of 736 mm² over 30 mm, less its rounded corner and two
// holes of 8 through 8 mm legs.
TEST(SamplesTest, TheBracketIsWhatItSaysItIs) {
    SampleFolders folders;
    MainWindow w;
    const hz::doc::Document* doc = open(w, "bracket.hzpart");
    ASSERT_NE(doc, nullptr);
    const double pi = 3.14159265358979323846;
    const double holes = 2 * pi * 4.0 * 4.0 * 8.0;
    const double corner = (36.0 - pi * 9.0) * 30.0;  // 6 x 6 less a quarter disc
    EXPECT_NEAR(volumeOf(*doc), 736.0 * 30.0 - holes - corner, 0.01 * 736.0 * 30.0);
}

// The assembly opens with its pin in the plate's first hole, where its mates
// put it: on the hole's axis, its end flush with the plate's underside.
TEST(SamplesTest, TheAssemblyOpensWithItsPinInItsHole) {
    SampleFolders folders;
    MainWindow w;
    open(w, "plate-and-pin.hzasm");
    const hz::doc::AssemblyDocument* assembly = w.activeAssembly();
    ASSERT_NE(assembly, nullptr);
    ASSERT_EQ(assembly->components().size(), 2u);
    EXPECT_EQ(assembly->mates().size(), 3u);
    for (const auto& component : assembly->components()) {
        if (component.name != "Pin") continue;
        const auto at = component.transform.transformPoint(hz::math::Vec3(0, 0, 0));
        EXPECT_NEAR(at.x, 20.0, 1e-6);
        EXPECT_NEAR(at.y, 30.0, 1e-6);
        EXPECT_NEAR(at.z, 0.0, 1e-6);
    }
}

TEST(SamplesTest, TheDrawingsOpen) {
    SampleFolders folders;
    {
        MainWindow w;
        const hz::doc::Document* sheet = open(w, "bracket-drawing.hzdwg");
        ASSERT_NE(sheet, nullptr);
        EXPECT_GT(countOn(*sheet, "Visible"), 0u) << "the bracket's views are drawn";
        EXPECT_GT(countOn(*sheet, "TitleBlock"), 0u);
    }
    MainWindow w;
    const hz::doc::Document* gasket = open(w, "gasket.hcad");
    ASSERT_NE(gasket, nullptr);
    EXPECT_EQ(countOn(*gasket, "Outline"), 13u);  // 4 sides, 4 corners, 5 holes
    EXPECT_EQ(countOn(*gasket, "Centre lines"), 2u);
    EXPECT_EQ(countOn(*gasket, "Dimensions"), 4u);  // 3 dimensions and a note
}

// A sample is opened as a copy in the user's folder, with every other sample
// beside it (an assembly's parts); a copy already there is kept, and the
// shipped sample is not touched.
TEST(SamplesTest, ASampleIsOpenedAsACopyThatIsKept) {
    SampleFolders folders;
    MainWindow w;
    open(w, "bracket.hzpart");
    // As Open Recent keeps it: the temporary folder is behind a link on
    // macOS (/var is /private/var).
    EXPECT_EQ(QDir::cleanPath(hz::ui::RecentFiles::list().value(0)),
              QFileInfo(folders.copy(QStringLiteral("bracket.hzpart"))).canonicalFilePath());
    for (const char* file : {"bracket.hzpart", "bracket-drawing.hzdwg", "plate.hzpart",
                             "pin.hzpart", "plate-and-pin.hzasm", "gasket.hcad"}) {
        EXPECT_TRUE(QFile::exists(folders.copy(QString::fromLatin1(file)))) << file;
    }

    QFile changed(folders.copy(QStringLiteral("gasket.hcad")));
    ASSERT_TRUE(changed.open(QIODevice::WriteOnly | QIODevice::Truncate));
    changed.write("the user's own");
    changed.close();
    open(w, "pin.hzpart");
    ASSERT_TRUE(changed.open(QIODevice::ReadOnly));
    EXPECT_EQ(changed.readAll(), QByteArray("the user's own"));
    QFile shipped(
        QDir(QStringLiteral(HZ_BUILT_SAMPLES_DIR)).filePath(QStringLiteral("gasket.hcad")));
    ASSERT_TRUE(shipped.open(QIODevice::ReadOnly));
    EXPECT_NE(shipped.readAll(), QByteArray("the user's own"));
}

// A sample that cannot be copied is not opened, and the user is told why:
// an assembly opened without its parts would look broken, not blocked.
TEST(SamplesTest, ASampleThatCannotBeCopiedIsNotOpened) {
    SampleFolders folders;
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    {
        // Where the copies go is a file, not a folder: it cannot be made.
        const QString file = QDir(dir.path()).filePath(QStringLiteral("a-file"));
        QFile made(file);
        ASSERT_TRUE(made.open(QIODevice::WriteOnly));
        made.close();
        qputenv("HZ_SAMPLE_COPIES_DIR", QFile::encodeName(file));
        MainWindow w;
        hz::test::DialogResponder told(QMessageBox::Ok);
        EXPECT_FALSE(w.openSample(QStringLiteral("plate-and-pin.hzasm")));
        EXPECT_TRUE(told.seen());
        EXPECT_EQ(w.activeAssembly(), nullptr) << "not opened";
    }
    {
        // A folder nothing can be written in: the copies fail.
        const QString folder = QDir(dir.path()).filePath(QStringLiteral("read-only"));
        ASSERT_TRUE(QDir().mkpath(folder));
        QFile::setPermissions(folder, QFileDevice::ReadOwner | QFileDevice::ExeOwner);
        QFile probe(QDir(folder).filePath(QStringLiteral("probe")));
        if (probe.open(QIODevice::WriteOnly)) {
            probe.close();
            probe.remove();
            QFile::setPermissions(
                folder, QFileDevice::ReadOwner | QFileDevice::WriteOwner | QFileDevice::ExeOwner);
            GTEST_SKIP() << "a read-only folder is still writable here (root, or Windows)";
        }
        qputenv("HZ_SAMPLE_COPIES_DIR", QFile::encodeName(folder));
        MainWindow w;
        hz::test::DialogResponder told(QMessageBox::Ok);
        const bool opened = w.openSample(QStringLiteral("plate-and-pin.hzasm"));
        QFile::setPermissions(
            folder, QFileDevice::ReadOwner | QFileDevice::WriteOwner | QFileDevice::ExeOwner);
        EXPECT_FALSE(opened);
        EXPECT_TRUE(told.seen());
        EXPECT_TRUE(told.text().contains(QStringLiteral("could not be copied")))
            << told.text().toStdString();
        EXPECT_EQ(w.activeAssembly(), nullptr) << "not opened";
    }
    MainWindow w;
    hz::test::DialogResponder told(QMessageBox::Ok);
    EXPECT_FALSE(w.openSample(QStringLiteral("no-such-sample.hzpart")));
    EXPECT_TRUE(told.text().contains(QStringLiteral("not among the samples")))
        << told.text().toStdString();
}

#endif  // HZ_BUILT_SAMPLES_DIR
