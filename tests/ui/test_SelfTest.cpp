// horizon --self-test's checks (docs/BETA.md): what a beta tester is asked
// to do, done with the code the commands use. Each passes here, and leaves
// what it made; each that cannot says why.

#include <gtest/gtest.h>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QString>
#include <QTemporaryDir>
#include <vector>

#include "horizon/ui/SelfTest.h"

using hz::ui::selftest::runWorkflows;
using hz::ui::selftest::Shipped;
using hz::ui::selftest::Step;

namespace {

const Step* named(const std::vector<Step>& steps, const char* name) {
    for (const Step& step : steps) {
        if (step.name == QLatin1String(name)) return &step;
    }
    return nullptr;
}

/// The samples and catalogs this build ships, as the application finds them.
Shipped built() {
    Shipped shipped;
#ifdef HZ_BUILT_SAMPLES_DIR
    shipped.samples = QStringLiteral(HZ_BUILT_SAMPLES_DIR);
#endif
#if defined(HZ_SHIPPED_TRANSLATIONS_DIR) && defined(HZ_TRANSLATIONS_SRC_DIR)
    // One catalog for each source, when this build compiled them.
    shipped.translations = QStringLiteral(HZ_SHIPPED_TRANSLATIONS_DIR);
    if (!QDir(shipped.translations).entryList({QStringLiteral("horizon_*.qm")}).isEmpty()) {
        shipped.catalogs = static_cast<int>(QDir(QStringLiteral(HZ_TRANSLATIONS_SRC_DIR))
                                                .entryList({QStringLiteral("horizon_*.ts")})
                                                .size());
    }
#endif
    return shipped;
}

}  // namespace

TEST(SelfTestTest, WhatABetaTesterDoesPasses) {
    const Shipped shipped = built();
    if (shipped.samples.isEmpty()) GTEST_SKIP() << "this build makes no samples";
    QTemporaryDir dir;
    const std::vector<Step> steps = runWorkflows(dir.path(), shipped);
    ASSERT_EQ(steps.size(), 6u);
    for (const Step& step : steps) {
        EXPECT_TRUE(step.passed) << step.name.toStdString() << ": " << step.detail.toStdString();
    }
    const QDir made(dir.path());
    for (const char* file : {"block.hzpart", "block.step", "blocks.hzasm", "block-drawing.hzdwg",
                             "block-drawing.pdf", "block-drawing.svg", "block-drawing.dxf"}) {
        EXPECT_GT(QFileInfo(made.filePath(QString::fromLatin1(file))).size(), 0) << file;
    }
    EXPECT_TRUE(named(steps, "part")->detail.contains(QStringLiteral("mm³")));
}

// Nowhere to write: the part is not made, and what needs it says so rather
// than failing on something else.
TEST(SelfTestTest, AFolderThatCannotBeWrittenIsSaid) {
    QTemporaryDir dir;
    QFile blocker(QDir(dir.path()).filePath(QStringLiteral("a file")));
    ASSERT_TRUE(blocker.open(QIODevice::WriteOnly));
    blocker.close();
    const std::vector<Step> steps =
        runWorkflows(QDir(blocker.fileName()).filePath(QStringLiteral("inside")), built());
    const Step* part = named(steps, "part");
    ASSERT_NE(part, nullptr);
    EXPECT_FALSE(part->passed);
    EXPECT_TRUE(part->detail.contains(QStringLiteral("could not be saved")))
        << part->detail.toStdString();
    for (const char* after : {"step", "assembly", "drawing"}) {
        const Step* step = named(steps, after);
        ASSERT_NE(step, nullptr) << after;
        EXPECT_FALSE(step->passed) << after;
        EXPECT_TRUE(step->detail.startsWith(QStringLiteral("there is no part")))
            << after << ": " << step->detail.toStdString();
    }
}

// A package without its samples, or short of a catalog, fails.
TEST(SelfTestTest, MissingShippedFilesFail) {
    QTemporaryDir dir;
    QTemporaryDir empty;
    Shipped shipped;
    shipped.samples = empty.path();
    shipped.translations = empty.path();
    shipped.catalogs = 6;
    const std::vector<Step> steps = runWorkflows(dir.path(), shipped);
    const Step* samples = named(steps, "samples");
    ASSERT_NE(samples, nullptr);
    EXPECT_FALSE(samples->passed);
    EXPECT_TRUE(samples->detail.startsWith(QStringLiteral("there are none")))
        << samples->detail.toStdString();
    const Step* translations = named(steps, "translations");
    ASSERT_NE(translations, nullptr);
    EXPECT_FALSE(translations->passed);
    EXPECT_TRUE(translations->detail.startsWith(QStringLiteral("0 in")))
        << translations->detail.toStdString();

    shipped.catalogs = 0;  // a build without them: none looked for
    EXPECT_TRUE(named(runWorkflows(dir.path(), shipped), "translations")->passed);
}
