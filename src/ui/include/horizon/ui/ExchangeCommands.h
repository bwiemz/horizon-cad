#pragma once

#include <QObject>
#include <QString>
#include <QtGlobal>
#include <atomic>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "horizon/fileio/ImportReport.h"
#include "horizon/fileio/StepAssemblyFiles.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/math/Units.h"
#include "horizon/ui/BackgroundTask.h"

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::ui {

class AssemblyWorkbench;
class DrawingWorkbench;
class WorkbenchHost;

/// File ▸ Import and File ▸ Export: a STEP or DXF file read into a document,
/// and the document written out as STEP, STL, glTF, DXF, PDF or SVG. The
/// window holds the menus; these hold the commands, and a STEP import running
/// on a worker (Phases 107, 153). They reach the window only through its
/// WorkbenchHost, and the assemblies' and drawings' workbenches for what an
/// assembly exports and the paper a sheet is plotted on.
class ExchangeCommands : public QObject {
    Q_OBJECT

public:
    ExchangeCommands(WorkbenchHost& host, AssemblyWorkbench& assemblies, DrawingWorkbench& drawings,
                     QObject* parent = nullptr);
    ~ExchangeCommands() override;

    // --- File ▸ Import ---
    void onImportStep();
    void onImportStepAssembly();
    /// A DXF's layers, blocks and entities into the active drawing, as one
    /// undoable step.
    void onImportDxf();

    // --- File ▸ Export ---
    void onExportStep();
    void onExportStl();
    void onExportGltf();
    void onExportDxf();
    /// The drawing plotted to a PDF (@p pdf) or an SVG, on a paper, scale and
    /// colours chosen in a form.
    void onExportPlot(bool pdf);

    /// A STEP file this big or more is read on a worker in Auto; a STEP
    /// assembly is, whatever its size.
    static constexpr qint64 kWorkerImportBytes = 1'000'000;

    /// A STEP import is running on a worker.
    bool busy() const { return m_importTask != nullptr; }
    /// Stop it; what it wrote goes too.
    void cancelWork();

    /// How far a STEP assembly import has got, for the progress bar: its
    /// parts written, of how many. Nullopt while it is still read, and when
    /// nothing runs.
    struct Progress {
        QString file;
        std::size_t written = 0;
        std::size_t total = 0;
    };
    std::optional<Progress> progress() const;

    /// A STEP file read, on the window's thread or a worker: into solids, or,
    /// to be kept as an assembly at `assemblyPath`, into its parts and their
    /// placements (Phase 153).
    struct StepLoad {
        std::vector<std::unique_ptr<topo::Solid>> solids;
        io::StepAssembly assembly;
        std::string assemblyPath;
        io::ImportReport report;
        std::string error;  ///< why nothing was read (lastError is per thread)
        /// Nothing was read: the file does not say which unit its lengths
        /// are in (StepFormat::lastLengthUnitUnknown).
        bool unitUnknown = false;
        /// Kept as an assembly: its files, written where it was read.
        io::StepAssemblyFiles files;
        bool kept = false;
    };
    /// How far a STEP assembly's parts are written, as the worker writing
    /// them says.
    struct ImportProgress {
        std::atomic<std::size_t> written{0};
        std::atomic<std::size_t> total{0};
    };
    /// Read @p path; to be kept as an assembly at @p assemblyPath, its parts
    /// built and written as files there too, on the same thread (they took
    /// the window's time once it was read), each counted into @p progress.
    static StepLoad loadStep(const std::string& path, const std::string& assemblyPath = {},
                             const std::atomic<bool>* cancelled = nullptr,
                             const io::StepReadOptions& options = {},
                             const std::shared_ptr<ImportProgress>& progress = nullptr);

private:
    /// Read @p fileName into a new part, or keep it as an assembly at
    /// @p assemblyPath when one is given: on a worker when the file is large,
    /// and for an assembly however small.
    void startStepImport(const QString& fileName, const QString& assemblyPath,
                         const io::StepReadOptions& options = {});
    void onImportFinished();
    void finishStepImport(const QString& fileName, StepLoad load);
    /// @p load's parts written as part files beside its assembly, and the
    /// assembly opened (Phase 153).
    void finishStepAssemblyImport(const QString& fileName, StepLoad load);
    /// Ask which unit @p fileName's lengths are in, the file not saying:
    /// none when its user cancels.
    std::optional<math::LengthUnit> askStepLengthUnit(const QString& fileName);

    /// The active part's body, or null, said in the status bar.
    const topo::Solid* solidToExport(const QString& format);
    /// Where to export as @p format: asked, @p suffix added when none is
    /// typed. Empty when its user cancels, or will not have the file the
    /// document was read from replaced.
    QString askExportPath(const QString& format, const QString& filter, const QString& suffix);
    /// The active assembly written as a STEP assembly (Phase 153).
    void exportAssemblyStep();
    /// What a STEP export could not write as asked: the curved faces kept in
    /// facets, and the components left out (@p unread, whose parts could
    /// not be read, and those in @p report).
    void showStepExportReport(const io::StepWriteReport& report,
                              const std::vector<std::string>& unread = {});

    WorkbenchHost& m_host;
    AssemblyWorkbench& m_assemblies;
    DrawingWorkbench& m_drawings;
    std::unique_ptr<BackgroundTask<StepLoad>> m_importTask;
    QString m_importFile;                              ///< what m_importTask reads
    std::shared_ptr<ImportProgress> m_importProgress;  ///< of m_importTask
};

}  // namespace hz::ui
