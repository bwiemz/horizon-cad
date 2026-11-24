// Reading DWG files through LibreDWG's dwg2dxf (DwgImport).
//
// The fixtures are drawings AutoCAD made (LibreDWG's test data; see
// tests/fileio/fixtures/dwg/README.md), each with one entity whose values
// AutoCAD itself reported. The drawings are in inches, which the reading
// scales into millimetres.

#include <gtest/gtest.h>

#include <QByteArray>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QString>
#include <QTemporaryDir>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/math/Vec2.h"
#include "horizon/ui/DwgImport.h"

using hz::doc::Document;
using hz::ui::DwgImport;

namespace {

constexpr double kInch = 25.4;
/// AutoCAD reported its values to four or five decimals of an inch.
constexpr double kTolerance = 0.002;

QString fixture(const char* name) {
    return QDir(QStringLiteral(HZ_DWG_FIXTURES_DIR)).filePath(QString::fromLatin1(name));
}

/// Runs a test with @p program as dwg2dxf, and puts the search back after.
class ConverterScope {
public:
    explicit ConverterScope(const QString& program) { DwgImport::setConverter(program); }
    ~ConverterScope() {
        DwgImport::setConverter({});
        DwgImport::setTimeoutMs(5 * 60 * 1000);
    }
    ConverterScope(const ConverterScope&) = delete;
    ConverterScope& operator=(const ConverterScope&) = delete;
};

/// The entities of @p doc of type T.
template <typename T>
std::vector<std::shared_ptr<T>> all(const Document& doc) {
    std::vector<std::shared_ptr<T>> found;
    for (const auto& entity : doc.draftDocument().entities()) {
        if (auto typed = std::dynamic_pointer_cast<T>(entity)) found.push_back(typed);
    }
    return found;
}

#ifdef HZ_DWG2DXF

/// Tests that run LibreDWG's dwg2dxf, the one this build ships.
class DwgImportTest : public ::testing::Test {
protected:
    ConverterScope converter{QStringLiteral(HZ_DWG2DXF)};

    /// @p name read into a new document, which the test fails without.
    std::unique_ptr<Document> read(const char* name) {
        auto doc = std::make_unique<Document>();
        std::string error;
        EXPECT_TRUE(DwgImport::load(fixture(name).toStdString(), *doc, &error))
            << name << ": " << error;
        return doc;
    }
};

TEST_F(DwgImportTest, ALineIsWhereAutoCadDrewItInEveryVersionSince2004) {
    // AutoCAD 2004, 2007, 2010, 2013 and 2018 formats: every version saved
    // since 2004 is one of these.
    for (const char* name :
         {"line_2004.dwg", "line_2007.dwg", "line_2010.dwg", "line_2013.dwg", "line_2018.dwg"}) {
        SCOPED_TRACE(name);
        const auto doc = read(name);
        const auto lines = all<hz::draft::DraftLine>(*doc);
        ASSERT_EQ(lines.size(), 1U);
        EXPECT_NEAR(lines[0]->start().x, 21.6078 * kInch, kTolerance);
        EXPECT_NEAR(lines[0]->start().y, 2.81192 * kInch, kTolerance);
        EXPECT_NEAR(lines[0]->end().x, 11.0307 * kInch, kTolerance);
        EXPECT_NEAR(lines[0]->end().y, 16.6375 * kInch, kTolerance);
        EXPECT_EQ(lines[0]->layer(), "0");
    }
}

TEST_F(DwgImportTest, CirclesAndArcsKeepTheirCentresRadiiAndAngles) {
    const auto circleDoc = read("circle_2018.dwg");
    const auto circles = all<hz::draft::DraftCircle>(*circleDoc);
    ASSERT_EQ(circles.size(), 1U);
    EXPECT_NEAR(circles[0]->center().x, 23.4493 * kInch, kTolerance);
    EXPECT_NEAR(circles[0]->center().y, -1.09239 * kInch, kTolerance);
    EXPECT_NEAR(circles[0]->radius(), 13.5304 * kInch, kTolerance);

    const auto arcDoc = read("arc_2018.dwg");
    const auto arcs = all<hz::draft::DraftArc>(*arcDoc);
    ASSERT_EQ(arcs.size(), 1U);
    EXPECT_NEAR(arcs[0]->center().x, -18.1933 * kInch, kTolerance);
    EXPECT_NEAR(arcs[0]->center().y, 6.68652 * kInch, kTolerance);
    EXPECT_NEAR(arcs[0]->radius(), 8.2931 * kInch, kTolerance);
    // It runs counterclockwise from 2.00242 rad round to 1.51876: its ends
    // are where AutoCAD put them, whichever way the angles are written.
    EXPECT_NEAR(arcs[0]->startPoint().x, -21.6627 * kInch, kTolerance);
    EXPECT_NEAR(arcs[0]->startPoint().y, 14.219 * kInch, kTolerance);
    EXPECT_NEAR(arcs[0]->endPoint().x, -17.762 * kInch, kTolerance);
    EXPECT_NEAR(arcs[0]->endPoint().y, 14.9684 * kInch, kTolerance);
    EXPECT_NEAR(arcs[0]->sweepAngle(), 5.79952, 1e-4);
}

TEST_F(DwgImportTest, AParagraphOfTextKeepsItsWordsAndPlace) {
    const auto doc = read("text_2018.dwg");
    const auto texts = all<hz::draft::DraftText>(*doc);
    ASSERT_EQ(texts.size(), 1U);
    std::string words;
    for (const std::string& line : texts[0]->lines()) {
        if (!words.empty()) words += ' ';
        words += line;
    }
    EXPECT_EQ(words,
              "This a multiline text to check if libredwg is able to read this in the current "
              "file format version");
    EXPECT_NEAR(texts[0]->textHeight(), 0.2 * kInch, kTolerance);
}

TEST_F(DwgImportTest, APolylineEllipseAndSplineComeInWhole) {
    const auto polylineDoc = read("polyline_2018.dwg");
    const auto polylines = all<hz::draft::DraftPolyline>(*polylineDoc);
    ASSERT_EQ(polylines.size(), 1U);
    EXPECT_FALSE(polylines[0]->closed());
    const auto& points = polylines[0]->points();
    ASSERT_GE(points.size(), 3U);
    EXPECT_NEAR(points[0].x, 9.00535 * kInch, kTolerance);
    EXPECT_NEAR(points[0].y, 12.2913 * kInch, kTolerance);
    EXPECT_NEAR(points[1].x, 17.1819 * kInch, kTolerance);
    EXPECT_NEAR(points[1].y, 14.7267 * kInch, kTolerance);
    EXPECT_NEAR(points[2].x, 20.7826 * kInch, kTolerance);
    EXPECT_NEAR(points[2].y, 8.24476 * kInch, kTolerance);

    const auto ellipseDoc = read("ellipse_2018.dwg");
    const auto ellipses = all<hz::draft::DraftEllipse>(*ellipseDoc);
    ASSERT_EQ(ellipses.size(), 1U);
    EXPECT_NEAR(ellipses[0]->center().x, 11.1433 * kInch, kTolerance);
    EXPECT_NEAR(ellipses[0]->center().y, 9.5374 * kInch, kTolerance);
    EXPECT_NEAR(ellipses[0]->semiMajor(), 11.8799 * kInch, kTolerance);
    EXPECT_NEAR(ellipses[0]->semiMinor(), 3.99294 * kInch, kTolerance);
    // Its major axis points along (-8.7508, -8.03465), either way along it.
    const double along = std::atan2(-8.03465, -8.7508);
    const double turned = std::remainder(ellipses[0]->rotation() - along, std::numbers::pi);
    EXPECT_NEAR(turned, 0.0, 1e-4);

    // Drawn through points, which is all the file keeps of it: it runs
    // through the ones AutoCAD gave, starting at the first.
    const auto splineDoc = read("spline_2018.dwg");
    const auto splines = all<hz::draft::DraftSpline>(*splineDoc);
    ASSERT_EQ(splines.size(), 1U);
    const auto drawn = splines[0]->evaluate();
    ASSERT_FALSE(drawn.empty());
    EXPECT_NEAR(drawn.front().x, 28.059 * kInch, kTolerance);
    EXPECT_NEAR(drawn.front().y, 8.80677 * kInch, kTolerance);
    const hz::math::Vec2 second(40.0613 * kInch, 6.22149 * kInch);
    const bool throughSecond = std::any_of(drawn.begin(), drawn.end(), [&](const auto& p) {
        return (p - second).length() < kTolerance;
    });
    EXPECT_TRUE(throughSecond);
}

TEST_F(DwgImportTest, AnAutoCad2000DrawingReads) {
    const auto doc = read("sample_2000.dwg");
    EXPECT_FALSE(doc->draftDocument().entities().empty());
}

TEST_F(DwgImportTest, ANameNoProgramWouldTakeAsGivenStillReads) {
    // Spaces, letters outside ASCII, and a leading dash that dwg2dxf would
    // read as an option: it is given a copy, by a plain name.
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString odd = dir.filePath(QStringLiteral("-Zeichnung für Ü 図面.dwg"));
    ASSERT_TRUE(QFile::copy(fixture("line_2018.dwg"), odd));
    Document doc;
    std::string error;
    ASSERT_TRUE(DwgImport::load(odd.toStdString(), doc, &error)) << error;
    EXPECT_EQ(all<hz::draft::DraftLine>(doc).size(), 1U);
}

TEST_F(DwgImportTest, AFileThatIsNotADrawingIsRefusedWithWhy) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString notes = dir.filePath(QStringLiteral("notes.dwg"));
    {
        QFile file(notes);
        ASSERT_TRUE(file.open(QIODevice::WriteOnly));
        file.write("These are notes, not a drawing.\n");
    }
    Document doc;
    std::string error;
    EXPECT_FALSE(DwgImport::load(notes.toStdString(), doc, &error));
    EXPECT_EQ(error, "it is not a DWG file, or not one LibreDWG can read");
    EXPECT_TRUE(doc.draftDocument().entities().empty());
}

TEST_F(DwgImportTest, ADrawingCutShortIsRefusedWithWhy) {
    QFile whole(fixture("line_2018.dwg"));
    ASSERT_TRUE(whole.open(QIODevice::ReadOnly));
    const QByteArray start = whole.read(9000);
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString cut = dir.filePath(QStringLiteral("cut.dwg"));
    {
        QFile file(cut);
        ASSERT_TRUE(file.open(QIODevice::WriteOnly));
        file.write(start);
    }
    Document doc;
    std::string error;
    EXPECT_FALSE(DwgImport::load(cut.toStdString(), doc, &error));
    EXPECT_EQ(error, "parts of it are missing: it may be damaged, or cut short");
}

TEST_F(DwgImportTest, ADrawingOrDxfIsReadAsItsExtensionSays) {
    Document fromDwg;
    std::string error;
    // In capitals, as Windows often writes them.
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString upper = dir.filePath(QStringLiteral("LINE.DWG"));
    ASSERT_TRUE(QFile::copy(fixture("line_2018.dwg"), upper));
    ASSERT_TRUE(DwgImport::loadDrawing(upper.toStdString(), fromDwg, &error)) << error;
    EXPECT_EQ(all<hz::draft::DraftLine>(fromDwg).size(), 1U);

    // The same bytes named .dxf are read as DXF, which they are not.
    const QString misnamed = dir.filePath(QStringLiteral("line.dxf"));
    ASSERT_TRUE(QFile::copy(fixture("line_2018.dwg"), misnamed));
    Document fromDxf;
    EXPECT_FALSE(DwgImport::loadDrawing(misnamed.toStdString(), fromDxf, &error));
}

#endif  // HZ_DWG2DXF

TEST(DwgImportReasons, AMissingFileIsSaidToBeMissing) {
    // Whatever runs as dwg2dxf, it is not run for a file that is not there.
    ConverterScope converter(QStringLiteral("dwg2dxf-that-is-never-run"));
    Document doc;
    std::string error;
    EXPECT_FALSE(DwgImport::load(fixture("no-such-drawing.dwg").toStdString(), doc, &error));
    EXPECT_EQ(error, "the file does not exist");
}

TEST(DwgImportReasons, WithoutLibreDwgTheReasonSaysHowToGetIt) {
    // No dwg2dxf beside the program (a test has no application) or on the
    // PATH.
    ConverterScope converter({});
    const QByteArray path = qgetenv("PATH");
    qputenv("PATH", QByteArray());
    const QString found = DwgImport::converter();
    Document doc;
    std::string error;
    const bool read = DwgImport::load(fixture("line_2018.dwg").toStdString(), doc, &error);
    qputenv("PATH", path);
    EXPECT_TRUE(found.isEmpty());
    EXPECT_FALSE(read);
    EXPECT_NE(error.find("install LibreDWG"), std::string::npos) << error;
}

TEST(DwgImportReasons, AConverterThatCannotStartIsNamed) {
    ConverterScope converter(QStringLiteral("/no/such/folder/dwg2dxf"));
    Document doc;
    std::string error;
    EXPECT_FALSE(DwgImport::load(fixture("line_2018.dwg").toStdString(), doc, &error));
    EXPECT_NE(error.find("could not be started"), std::string::npos) << error;
}

#ifndef _WIN32

/// A stand-in for dwg2dxf: a shell script, run with the arguments dwg2dxf
/// would be given ("-o drawing.dxf drawing.dwg") in the folder it would run
/// in.
class FakeConverter {
public:
    explicit FakeConverter(const QByteArray& body) {
        m_path = m_dir.filePath(QStringLiteral("dwg2dxf"));
        QFile script(m_path);
        if (script.open(QIODevice::WriteOnly)) {
            script.write("#!/bin/sh\n" + body + "\n");
            script.close();
            script.setPermissions(QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner);
        }
    }
    const QString& path() const { return m_path; }

private:
    QTemporaryDir m_dir;
    QString m_path;
};

std::string readWith(const QByteArray& body, int timeoutMs = 60'000) {
    const FakeConverter fake(body);
    ConverterScope converter(fake.path());
    DwgImport::setTimeoutMs(timeoutMs);
    Document doc;
    std::string error;
    EXPECT_FALSE(DwgImport::load(fixture("line_2018.dwg").toStdString(), doc, &error));
    EXPECT_TRUE(doc.draftDocument().entities().empty());
    return error;
}

TEST(DwgImportReasons, AConverterThatCrashesCostsOnlyThisFile) {
    // The status it stopped with follows, for the log.
    const std::string error = readWith("kill -SEGV $$");
    EXPECT_EQ(error.rfind("LibreDWG stopped while reading it: the file may be damaged, or hold "
                          "something LibreDWG cannot read yet [0x",
                          0),
              0U)
        << error;
}

TEST(DwgImportReasons, AConverterThatNeverFinishesIsStopped) {
    EXPECT_EQ(readWith("exec sleep 60", 300),
              "LibreDWG had not read it after 1 second(s), and was stopped");
}

TEST(DwgImportReasons, ACancelledReadingStopsAtOnce) {
    // As a window closing does: it waits for its worker, which waited for
    // dwg2dxf, for as long as dwg2dxf took.
    const FakeConverter fake("exec sleep 60");
    const ConverterScope converter(fake.path());
    const std::atomic<bool> cancelled{true};
    Document doc;
    std::string error;
    QElapsedTimer clock;
    clock.start();
    EXPECT_FALSE(
        DwgImport::load(fixture("line_2018.dwg").toStdString(), doc, &error, nullptr, &cancelled));
    EXPECT_EQ(error, "the reading was cancelled");
    EXPECT_LT(clock.elapsed(), 10'000);
}

TEST(DwgImportReasons, AConverterThatWritesNothingIsCaught) {
    EXPECT_EQ(readWith("exit 0"), "LibreDWG read it, but wrote nothing of it");
}

TEST(DwgImportReasons, WhatItWroteIsReadAsDxfAndJudgedSo) {
    const std::string error = readWith("printf '  0\\nSECTION\\n' > \"$2\"");
    EXPECT_EQ(error.rfind("LibreDWG read it, but what it made of it could not be read: ", 0), 0U)
        << error;
}

TEST(DwgImportReasons, AReadErrorWithNoCodeGivesItsLastLine) {
    EXPECT_EQ(readWith("echo 'first' >&2; echo 'the reason' >&2; exit 1"),
              "LibreDWG could not read it (the reason)");
}

TEST(DwgImportReasons, ReadErrorCodesAreSaidInWords) {
    EXPECT_EQ(readWith("echo 'READ ERROR 0x2000' >&2; exit 1"),
              "there was not memory enough to read it");
    EXPECT_EQ(readWith("echo 'READ ERROR 0x1000' >&2; exit 1"), "LibreDWG could not read the file");
    EXPECT_EQ(readWith("echo 'READ ERROR 0x400' >&2; exit 1"),
              "LibreDWG met something in it that it cannot read yet");
}

TEST(DwgImportReasons, TracingInTheEnvironmentIsNotPassedOn) {
    // LIBREDWG_TRACE has dwg2dxf write megabytes of tracing.
    qputenv("LIBREDWG_TRACE", "9");
    const std::string error =
        readWith("[ -z \"$LIBREDWG_TRACE\" ] && echo 'no tracing' >&2; exit 1");
    qunsetenv("LIBREDWG_TRACE");
    EXPECT_EQ(error, "LibreDWG could not read it (no tracing)");
}

#endif  // _WIN32

}  // namespace
