#pragma once

#include <QString>
#include <atomic>
#include <string>

#include "horizon/fileio/ImportReport.h"

namespace hz::doc {
class Document;
}  // namespace hz::doc

namespace hz::ui {

/// Reads a DWG file, AutoCAD's own format, into a document. LibreDWG's
/// dwg2dxf turns it into DXF, and DxfFormat reads that: what a DXF of the
/// drawing would bring in, this brings in, with the same report of what it
/// left out.
///
/// dwg2dxf is a program of its own, run for each file. A file LibreDWG cannot
/// read, or that has it crash or never finish, costs that program and not the
/// application with the user's other documents in it: the reading fails, and
/// says why.
///
/// Horizon CAD does not write DWG: a drawing read from one is saved as DXF or
/// as a Horizon CAD drawing.
class DwgImport {
public:
    /// Read the DWG file at @p path (UTF-8) into @p doc. On failure returns
    /// false, with why in @p error as a clause ("the file does not exist").
    /// With @p report, what the reading left out or approximated. Blocks
    /// until dwg2dxf finishes, timeoutMs() passes, or @p cancelled is set
    /// (then dwg2dxf is stopped at once): call it off the GUI thread for a
    /// large file.
    static bool load(const std::string& path, doc::Document& doc, std::string* error = nullptr,
                     io::ImportReport* report = nullptr,
                     const std::atomic<bool>* cancelled = nullptr);

    /// The dwg2dxf to run: the one set by setConverter(), else the one beside
    /// the application (a package that ships LibreDWG puts it there), else
    /// one on the PATH (an installed LibreDWG); empty when there is none.
    static QString converter();
    /// Run @p program as dwg2dxf instead of looking for it; empty looks again.
    static void setConverter(const QString& program);

    /// How long one file may take before dwg2dxf is stopped and the reading
    /// fails (default five minutes).
    static int timeoutMs();
    static void setTimeoutMs(int milliseconds);

    /// Whether @p fileName names a DWG file, by its extension.
    static bool isDwg(const QString& fileName);

    /// Read a drawing from a DXF or a DWG file, as its extension says, as
    /// DxfFormat::load() and load() do.
    static bool loadDrawing(const std::string& path, doc::Document& doc,
                            std::string* error = nullptr, io::ImportReport* report = nullptr,
                            const std::atomic<bool>* cancelled = nullptr);
};

}  // namespace hz::ui
