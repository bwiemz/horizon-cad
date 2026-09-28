#pragma once

#include <QString>
#include <vector>

namespace hz::ui::selftest {

/// One check `horizon --self-test` made, and what it found.
struct Step {
    QString name;  ///< "part", "step", "assembly", "drawing", "samples", "translations"
    bool passed = false;
    QString detail;  ///< what was found, or why it failed
};

/// Where the files a package ships are, and how many catalogs it has.
struct Shipped {
    QString samples;       ///< the samples' folder (MainWindow::sampleDirectory)
    QString translations;  ///< the catalogs' folder
    int catalogs = 0;      ///< how many: none are looked for when 0 (a build without them)
};

/// What `horizon --self-test` checks besides its window: what a beta tester
/// is asked to do (docs/BETA.md), done with the code the commands use, in
/// @p folder. A part is modelled, edited (undone and done again), saved and
/// read back; sent out as STEP and read in again; placed twice in an
/// assembly, which is saved and read back; and drawn on a sheet, which is
/// saved, read back, and exported to PDF, SVG and DXF. Then the files the
/// package ships (@p shipped): every sample opens, and its catalogs load.
///
/// A package is started this way on a machine that has never built it, so
/// what the build machine lent it (a library, a plugin) is not there.
std::vector<Step> runWorkflows(const QString& folder, const Shipped& shipped);

}  // namespace hz::ui::selftest
