#include "horizon/ui/ExchangeCommands.h"

#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QLayout>
#include <QMessageBox>
#include <QMetaObject>
#include <QStringList>
#include <algorithm>
#include <cmath>
#include <utility>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/PlotScene.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/GltfExport.h"
#include "horizon/fileio/StlExport.h"
#include "horizon/fileio/SvgExport.h"
#include "horizon/modeling/Sheet.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/AssemblyWorkbench.h"
#include "horizon/ui/DrawingWorkbench.h"
#include "horizon/ui/FeatureForm.h"
#include "horizon/ui/PdfExport.h"
#include "horizon/ui/TypedUnits.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/ui/WorkbenchHost.h"

namespace hz::ui {

ExchangeCommands::ExchangeCommands(WorkbenchHost& host, AssemblyWorkbench& assemblies,
                                   DrawingWorkbench& drawings, QObject* parent)
    : QObject(parent), m_host(host), m_assemblies(assemblies), m_drawings(drawings) {}

ExchangeCommands::~ExchangeCommands() = default;

void ExchangeCommands::cancelWork() {
    if (m_importTask) m_importTask->cancel();
}

std::optional<ExchangeCommands::Progress> ExchangeCommands::progress() const {
    if (!m_importTask || !m_importProgress) return std::nullopt;
    const std::size_t total = m_importProgress->total.load();
    if (total == 0) return std::nullopt;  // still reading: nothing written to count
    return Progress{QFileInfo(m_importFile).fileName(),
                    std::min(m_importProgress->written.load(), total), total};
}

ExchangeCommands::StepLoad ExchangeCommands::loadStep(
    const std::string& path, const std::string& assemblyPath, const std::atomic<bool>* cancelled,
    const io::StepReadOptions& options, const std::shared_ptr<ImportProgress>& progress) {
    StepLoad load;
    load.assemblyPath = assemblyPath;
    if (assemblyPath.empty()) {
        load.solids = io::StepFormat::load(path, &load.report, cancelled, options);
        if (load.solids.empty()) load.error = io::StepFormat::lastError();
    } else {
        load.assembly = io::StepFormat::loadAssembly(path, &load.report, cancelled, options);
        if (load.assembly.parts.empty()) load.error = io::StepFormat::lastError();
    }
    // Asked on the thread that read: what it says is that thread's.
    load.unitUnknown = io::StepFormat::lastLengthUnitUnknown();
    const bool stopped = cancelled != nullptr && cancelled->load();
    if (!assemblyPath.empty() && !load.assembly.parts.empty() && !stopped) {
        // Each part a part file, in a folder named for the assembly beside
        // it: the assembly refers to its parts by their files. Built and
        // written here, where it was read; stopped, or failed, it leaves
        // nothing behind.
        const QFileInfo kept(QString::fromStdString(assemblyPath));
        const QString partsDir =
            kept.dir().filePath(kept.completeBaseName() + QStringLiteral(" parts"));
        io::StepAssemblyProgress told;
        if (progress) {
            told = [progress](std::size_t written, std::size_t total) {
                progress->total = total;
                progress->written = written;
            };
        }
        load.kept =
            io::saveStepAssembly(load.assembly, assemblyPath, partsDir.toStdString(),
                                 QFileInfo(QString::fromStdString(path)).fileName().toStdString(),
                                 &load.files, &load.error, cancelled, told);
    }
    return load;
}

std::optional<math::LengthUnit> ExchangeCommands::askStepLengthUnit(const QString& fileName) {
    QStringList names;
    int current = 0;
    for (std::size_t k = 0; k < math::kLengthUnits.size(); ++k) {
        names << lengthUnitName(math::kLengthUnits[k]);
        // The unit new documents are made in: the likeliest.
        if (math::kLengthUnits[k] == m_host.documents().newDocumentUnit())
            current = static_cast<int>(k);
    }
    FeatureForm form(m_host.dialogParent(), tr("Length Unit Not Given"),
                     m_host.documents().newDocumentUnit());
    auto* note = new QLabel(tr("\"%1\" does not say which unit its lengths are in, or says it in "
                               "one this version cannot read. Read wrong, every length is the "
                               "wrong size.")
                                .arg(QFileInfo(fileName).fileName()),
                            &form.dialog());
    note->setWordWrap(true);
    form.dialog().layout()->addWidget(note);
    QComboBox* unit = form.choice(QStringLiteral("unit"), tr("Its lengths are in:"), names);
    unit->setCurrentIndex(current);
    if (!form.exec()) return std::nullopt;
    return math::kLengthUnits[static_cast<std::size_t>(std::max(unit->currentIndex(), 0))];
}

void ExchangeCommands::onImportStep() {
    const QString fileName =
        QFileDialog::getOpenFileName(m_host.dialogParent(), tr("Import STEP"), QString(),
                                     tr("STEP Files (*.step *.stp);;All Files (*)"));
    if (fileName.isEmpty()) return;
    startStepImport(fileName, QString());
}

void ExchangeCommands::onImportStepAssembly() {
    const QString fileName =
        QFileDialog::getOpenFileName(m_host.dialogParent(), tr("Import STEP as an Assembly"),
                                     QString(), tr("STEP Files (*.step *.stp);;All Files (*)"));
    if (fileName.isEmpty()) return;
    // Where the assembly goes; its parts go in a folder beside it.
    const QFileInfo step(fileName);
    QString assemblyPath = QFileDialog::getSaveFileName(
        m_host.dialogParent(), tr("Save the Assembly As"),
        step.dir().filePath(step.completeBaseName() + QStringLiteral(".hzasm")),
        tr("Horizon Assemblies (*.hzasm)"));
    if (assemblyPath.isEmpty()) return;
    if (QFileInfo(assemblyPath).suffix().isEmpty()) assemblyPath += QStringLiteral(".hzasm");
    startStepImport(fileName, assemblyPath);
}

void ExchangeCommands::startStepImport(const QString& fileName, const QString& assemblyPath,
                                       const io::StepReadOptions& options) {
    const std::string path = fileName.toStdString();
    const std::string keptAt = assemblyPath.toStdString();
    // An assembly goes to a worker however small its file: a few kilobytes
    // can name many parts, each built and written before the window is free.
    const bool onWorker =
        m_host.onWorker(!keptAt.empty() || QFileInfo(fileName).size() >= kWorkerImportBytes);
    if (!onWorker) {
        finishStepImport(fileName, loadStep(path, keptAt, nullptr, options));
        return;
    }
    if (m_importTask) {
        m_host.showStatus(tr("A STEP import is already running"));
        return;
    }
    m_importFile = fileName;
    m_importProgress = std::make_shared<ImportProgress>();
    m_importTask = std::make_unique<BackgroundTask<StepLoad>>(
        [path, keptAt, options, progress = m_importProgress](const std::atomic<bool>& cancelled) {
            return loadStep(path, keptAt, &cancelled, options, progress);
        });
    m_importTask->start([this] {
        QMetaObject::invokeMethod(this, &ExchangeCommands::onImportFinished, Qt::QueuedConnection);
    });
    m_host.setPrompt(tr("Importing %1...").arg(QFileInfo(fileName).fileName()));
    m_host.backgroundWorkChanged();  // and the parts written, as they are
}

void ExchangeCommands::onImportFinished() {
    if (!m_importTask || !m_importTask->finished()) return;
    const std::unique_ptr<BackgroundTask<StepLoad>> task = std::move(m_importTask);
    m_importProgress.reset();
    m_host.backgroundWorkChanged();
    if (task->cancelled()) {
        // Cancelled once its files were written: they go too.
        StepLoad late = task->take();
        if (late.kept) io::removeStepAssemblyFiles(late.files);
        m_host.setPrompt(tr("Ready"));
        m_host.showStatus(tr("Import cancelled"), 10000);
        return;
    }
    StepLoad load = task->take();
    if (!task->error().empty()) load.error = task->error();
    finishStepImport(m_importFile, std::move(load));
}

void ExchangeCommands::finishStepImport(const QString& fileName, StepLoad load) {
    if (load.unitUnknown) {
        // Not guessed: a file in inches read as millimetres is 25.4 times
        // too small. Its user says which, or it is not read.
        const auto unit = askStepLengthUnit(fileName);
        if (!unit) {
            m_host.setPrompt(tr("Ready"));
            m_host.showStatus(
                tr("\"%1\" was not imported: which unit its lengths are in was not given")
                    .arg(QFileInfo(fileName).fileName()),
                15000);
            return;
        }
        io::StepReadOptions options;
        options.unknownLengthUnit = *unit;
        startStepImport(fileName, QString::fromStdString(load.assemblyPath), options);
        return;
    }
    if (!load.assemblyPath.empty()) {
        finishStepAssemblyImport(fileName, std::move(load));
        return;
    }
    const std::string path = fileName.toStdString();
    if (load.solids.empty()) {
        m_host.reportFileError(tr("Could not import"), path, load.error);
        return;
    }

    // A new part, one body per solid, each kept in the part itself so it does
    // not depend on the STEP file any more.
    auto document = m_host.documents().newDocument(doc::DocumentType::Part);
    const std::string source = QFileInfo(fileName).fileName().toStdString();
    const auto count = static_cast<int>(load.solids.size());
    for (auto& solid : load.solids) {
        document->featureTree().addFeature(std::make_unique<doc::ImportedBodyFeature>(
            std::shared_ptr<const topo::Solid>(std::move(solid)), source));
    }
    document->rebuildModel();
    document->setDirty(true);  // it has not been saved anywhere yet
    m_host.addTab(std::move(document), QFileInfo(fileName).completeBaseName());
    m_host.rebuildModel();
    m_host.viewport().camera().setIsometricView();
    m_host.viewport().update();
    m_host.showImportReport(QFileInfo(fileName).fileName(), load.report);
    m_host.setPrompt(tr("Imported %n bodies.", "", count));
}

void ExchangeCommands::finishStepAssemblyImport(const QString& fileName, StepLoad load) {
    if (load.assembly.parts.empty()) {
        m_host.reportFileError(tr("Could not import"), fileName.toStdString(), load.error);
        return;
    }
    // Its files were written where it was read (loadStep); one that could not
    // be took the others with it.
    if (!load.kept) {
        m_host.reportFileError(tr("Could not import"), load.assemblyPath, load.error);
        return;
    }
    const QString partsDir = QString::fromStdString(load.files.partsDir);
    const auto parts = static_cast<int>(load.assembly.parts.size());
    const auto placed = static_cast<int>(load.assembly.occurrences.size());
    if (!m_host.openPath(QString::fromStdString(load.files.assembly))) return;
    m_host.showImportReport(QFileInfo(fileName).fileName(), load.report);
    m_host.setPrompt(
        // Two sentences, each whole: one split across two messages could not
        // be translated.
        tr("Imported %n part(s) into \"%1\".", "", parts).arg(QDir::toNativeSeparators(partsDir)) +
        QLatin1Char(' ') + tr("The assembly has %n component(s).", "", placed));
}

void ExchangeCommands::onImportDxf() {
    if (m_host.currentAssembly()) {
        m_host.showStatus(tr("A DXF is imported into a drawing or part, not an assembly"));
        return;
    }
    const QString fileName = QFileDialog::getOpenFileName(
        m_host.dialogParent(), tr("Import DXF"), QString(), tr("DXF Files (*.dxf);;All Files (*)"));
    if (fileName.isEmpty()) return;
    const std::string path = fileName.toStdString();
    doc::Document imported;
    std::string error;
    io::ImportReport report;
    if (!io::DxfFormat::load(path, imported, &error, &report)) {
        m_host.reportFileError(tr("Could not import"), path, error);
        return;
    }

    // Its layers and block definitions (one of the same name already here is
    // kept) and its entities come in as one undoable step.
    auto composite = std::make_unique<doc::CompositeCommand>(tr("Import DXF").toStdString());
    auto& layers = m_host.currentDocument()->layerManager();
    for (const auto& name : imported.layerManager().layerNames()) {
        if (!layers.getLayer(name)) {
            composite->addCommand(std::make_unique<doc::AddLayerCommand>(
                layers, *imported.layerManager().getLayer(name)));
        }
    }
    auto& target = m_host.currentDocument()->draftDocument();
    for (const auto& name : imported.draftDocument().blockTable().blockNames()) {
        if (!target.blockTable().findBlock(name)) {
            composite->addCommand(std::make_unique<doc::AddBlockDefinitionCommand>(
                target, imported.draftDocument().blockTable().findBlock(name)));
        }
    }
    for (const auto& entity : imported.draftDocument().entities()) {
        composite->addCommand(std::make_unique<doc::AddEntityCommand>(target, entity));
    }
    const auto count = static_cast<int>(imported.draftDocument().entities().size());
    if (!composite->empty()) m_host.currentDocument()->undoStack().push(std::move(composite));
    m_host.refreshPanels();
    m_host.viewport().update();
    m_host.setPrompt(tr("Imported %n entities.", "", count));
    m_host.showImportReport(QFileInfo(fileName).fileName(), report);
}

const topo::Solid* ExchangeCommands::solidToExport(const QString& format) {
    const topo::Solid* solid =
        m_host.currentAssembly() ? nullptr : m_host.currentDocument()->solid();
    if (!solid) {
        m_host.showStatus(tr("%1 export writes a part's body; this document has none").arg(format));
    }
    return solid;
}

QString ExchangeCommands::askExportPath(const QString& format, const QString& filter,
                                        const QString& suffix) {
    QString fileName = QFileDialog::getSaveFileName(m_host.dialogParent(),
                                                    tr("Export %1").arg(format), QString(), filter);
    if (!fileName.isEmpty() && QFileInfo(fileName).suffix().isEmpty()) fileName += suffix;
    // An export is written over the file the document was read from in part
    // (a DXF, exported as one) only as a save is: when its user says so.
    if (!fileName.isEmpty() && !m_host.mayReplaceSource(fileName)) return {};
    return fileName;
}

void ExchangeCommands::onExportStep() {
    if (m_host.currentAssembly()) {
        exportAssemblyStep();
        return;
    }
    const topo::Solid* solid = solidToExport(tr("STEP"));
    if (!solid) return;
    const QString fileName =
        askExportPath(tr("STEP"), tr("STEP Files (*.step *.stp)"), QStringLiteral(".step"));
    if (fileName.isEmpty()) return;
    io::StepFormat::WriteReport report;
    if (!io::StepFormat::save(fileName.toStdString(), {solid}, {}, &report)) {
        m_host.reportFileError(tr("Could not export"), fileName.toStdString(),
                               io::StepFormat::lastError());
        return;
    }
    m_host.setPrompt(tr("Exported %1.").arg(QFileInfo(fileName).fileName()));
    showStepExportReport(report);
}

void ExchangeCommands::exportAssemblyStep() {
    // Each part once, and each component a use of it where it is placed.
    const AssemblyWorkbench::StepExport gathered = m_assemblies.stepExport();
    if (gathered.occurrences.empty()) {
        m_host.showStatus(
            tr("STEP export writes an assembly's components; none of this one's parts can be "
               "read"));
        return;
    }
    const QString fileName =
        askExportPath(tr("STEP"), tr("STEP Files (*.step *.stp)"), QStringLiteral(".step"));
    if (fileName.isEmpty()) return;
    const QString title =
        m_host.currentAssembly()->filePath().empty()
            ? tr("Assembly")
            : QFileInfo(QString::fromStdString(m_host.currentAssembly()->filePath()))
                  .completeBaseName();
    io::StepFormat::WriteReport report;
    if (!io::StepFormat::saveAssembly(fileName.toStdString(), title.toStdString(), gathered.parts,
                                      gathered.occurrences, {}, &report)) {
        m_host.reportFileError(tr("Could not export"), fileName.toStdString(),
                               io::StepFormat::lastError());
        return;
    }
    m_host.setPrompt(tr("Exported %1.").arg(QFileInfo(fileName).fileName()));
    showStepExportReport(report, gathered.unread);
}

void ExchangeCommands::showStepExportReport(const io::StepWriteReport& report,
                                            const std::vector<std::string>& unread) {
    // Written as designed, but for faces that could not be: said, not
    // left for the other system to find as a mesh of facets. And any
    // component left out.
    QStringList lines;
    for (const std::string& name : unread) {
        lines
            << tr("Left out: \"%1\": its part could not be read").arg(QString::fromStdString(name));
    }
    for (const std::string& why : report.leftOut) {
        lines << tr("Left out: %1").arg(QString::fromStdString(why));
    }
    for (const std::string& why : report.faceted) lines << QString::fromStdString(why);
    if (lines.isEmpty()) return;
    const auto leftOut = static_cast<int>(unread.size() + report.leftOut.size());
    const QString faceted =
        report.faceted.empty()
            ? QString()
            : tr("%n curved face(s) were written as their facets, not on their surfaces.", nullptr,
                 static_cast<int>(report.faceted.size()));
    QMessageBox box(leftOut > 0 ? QMessageBox::Warning : QMessageBox::Information,
                    tr("Export STEP"),
                    leftOut > 0 ? (tr("%n component(s) were not written.", nullptr, leftOut) +
                                   (faceted.isEmpty() ? QString() : QStringLiteral(" ") + faceted))
                                : faceted,
                    QMessageBox::Ok, m_host.dialogParent());
    if (!report.faceted.empty()) {
        box.setInformativeText(
            tr("The part is exact as modelled; other systems will see those "
               "faces as flat facets."));
    }
    box.setDetailedText(lines.join(QLatin1Char('\n')));
    box.exec();
}

void ExchangeCommands::onExportStl() {
    const topo::Solid* solid = solidToExport(tr("STL"));
    if (!solid) return;
    const QString fileName =
        askExportPath(tr("STL"), tr("STL Files (*.stl)"), QStringLiteral(".stl"));
    if (fileName.isEmpty()) return;
    std::string error;
    if (!io::StlExport::save(fileName.toStdString(), *solid, &error)) {
        m_host.reportFileError(tr("Could not export"), fileName.toStdString(), error);
        return;
    }
    m_host.setPrompt(tr("Exported %1.").arg(QFileInfo(fileName).fileName()));
}

void ExchangeCommands::onExportGltf() {
    const topo::Solid* solid = solidToExport(tr("glTF"));
    if (!solid) return;
    const QString fileName =
        askExportPath(tr("glTF"), tr("glTF Binary (*.glb)"), QStringLiteral(".glb"));
    if (fileName.isEmpty()) return;
    const QString title = m_host.currentTitle();
    const std::string name = title.isEmpty() ? std::string("Part") : title.toStdString();
    if (!io::GltfExport::saveSolid(
            fileName.toStdString(), *solid,
            render::Material{math::Vec3{0.6, 0.75, 0.85}, 0.15f, 0.5f, 32.0f}, name)) {
        m_host.reportFileError(tr("Could not export"), fileName.toStdString(),
                               "the file could not be written");
        return;
    }
    m_host.setPrompt(tr("Exported %1.").arg(QFileInfo(fileName).fileName()));
}

void ExchangeCommands::onExportDxf() {
    if (m_host.currentAssembly() || m_host.currentDocument()->draftDocument().entities().empty()) {
        m_host.showStatus(tr("DXF export writes a drawing; this document has nothing drawn"));
        return;
    }
    const QString fileName =
        askExportPath(tr("DXF"), tr("DXF Files (*.dxf)"), QStringLiteral(".dxf"));
    if (fileName.isEmpty()) return;
    std::string error;
    if (!io::DxfFormat::save(fileName.toStdString(), *m_host.currentDocument(), &error)) {
        m_host.reportFileError(tr("Could not export"), fileName.toStdString(), error);
        return;
    }
    m_host.setPrompt(tr("Exported %1.").arg(QFileInfo(fileName).fileName()));
}

namespace {

/// "1:50" as paper millimetres per drawing millimetre (0.02); "2:1" as 2; 0
/// for anything else ("Fit to paper").
double plotScaleFrom(const QString& text) {
    const QStringList parts = text.split(QLatin1Char(':'));
    if (parts.size() != 2) return 0.0;
    bool paperOk = false;
    bool drawingOk = false;
    const double paper = parts[0].toDouble(&paperOk);
    const double drawing = parts[1].toDouble(&drawingOk);
    return paperOk && drawingOk && paper > 0.0 && drawing > 0.0 ? paper / drawing : 0.0;
}

}  // namespace

void ExchangeCommands::onExportPlot(bool pdf) {
    const QString format = pdf ? tr("PDF") : tr("SVG");
    if (m_host.currentAssembly()) {
        m_host.showStatus(tr("%1 export plots a drawing, not an assembly").arg(format));
        return;
    }
    const draft::DraftDocument& drawing = m_host.currentDocument()->draftDocument();
    const draft::PlotScene scene = draft::buildPlotScene(
        drawing, m_host.currentDocument()->layerManager(), drawing.dimensionStyle());
    if (scene.empty()) {
        m_host.showStatus(tr("%1 export plots a drawing; nothing visible is drawn").arg(format));
        return;
    }

    FeatureForm form(m_host.dialogParent(), tr("Export %1").arg(format),
                     m_host.currentDocument()->lengthUnit());
    QStringList papers;
    for (const auto& size : draft::standardPaperSizes()) papers << QString::fromLatin1(size.name);
    auto* paper = form.choice(QStringLiteral("paper"), tr("Paper:"), papers);
    auto* orientation = form.choice(QStringLiteral("orientation"), tr("Orientation:"),
                                    {tr("Landscape"), tr("Portrait")});
    auto* scale =
        form.choice(QStringLiteral("scale"), tr("Scale:"),
                    {tr("Fit to paper"), QStringLiteral("1:1"), QStringLiteral("1:2"),
                     QStringLiteral("1:5"), QStringLiteral("1:10"), QStringLiteral("1:20"),
                     QStringLiteral("1:50"), QStringLiteral("1:100"), QStringLiteral("1:200"),
                     QStringLiteral("2:1"), QStringLiteral("5:1"), QStringLiteral("10:1")});
    auto* colours =
        form.choice(QStringLiteral("colours"), tr("Colours:"), {tr("As drawn"), tr("Black")});
    // A drawing sheet is drawn to size: plotted on its own paper, at 1:1.
    if (const model::Sheet* sheet = m_drawings.paperOf(m_host.currentDocument())) {
        const double shortSide = std::min(sheet->widthMm(), sheet->heightMm());
        const double longSide = std::max(sheet->widthMm(), sheet->heightMm());
        const auto& sizes = draft::standardPaperSizes();
        for (size_t i = 0; i < sizes.size(); ++i) {
            if (std::abs(sizes[i].widthMm - shortSide) < 0.5 &&
                std::abs(sizes[i].heightMm - longSide) < 0.5) {
                paper->setCurrentIndex(static_cast<int>(i));
            }
        }
        orientation->setCurrentIndex(sheet->widthMm() >= sheet->heightMm() ? 0 : 1);
        scale->setCurrentIndex(scale->findText(QStringLiteral("1:1")));
    }
    if (!form.exec()) return;

    draft::PlotLayout layout;
    const draft::PaperSize& size =
        draft::standardPaperSizes()[static_cast<size_t>(std::max(paper->currentIndex(), 0))];
    const bool landscape = orientation->currentIndex() == 0;
    layout.paperWidthMm = landscape ? size.heightMm : size.widthMm;
    layout.paperHeightMm = landscape ? size.widthMm : size.heightMm;
    layout.scale = plotScaleFrom(scale->currentText());
    layout.monochrome = colours->currentIndex() == 1;

    // At a chosen scale the drawing may not fit: say so before cutting it off.
    bool fits = true;
    draft::plotTransform(scene, layout, &fits);
    if (!fits) {
        QMessageBox box(QMessageBox::Warning, tr("Export %1").arg(format),
                        tr("At %1 the drawing is larger than the paper: what is outside it will be "
                           "cut off.")
                            .arg(scale->currentText()),
                        QMessageBox::Save | QMessageBox::Cancel, m_host.dialogParent());
        box.setDefaultButton(QMessageBox::Cancel);
        if (box.exec() != QMessageBox::Save) return;
    }

    const QString fileName =
        pdf ? askExportPath(format, tr("PDF Files (*.pdf)"), QStringLiteral(".pdf"))
            : askExportPath(format, tr("SVG Files (*.svg)"), QStringLiteral(".svg"));
    if (fileName.isEmpty()) return;
    std::string error;
    bool ok = false;
    if (pdf) {
        QString why;
        ok = exportPdf(fileName, scene, layout, &why);
        error = why.toStdString();
    } else {
        ok = io::SvgExport::save(fileName.toStdString(), scene, layout, &error);
    }
    if (!ok) {
        m_host.reportFileError(tr("Could not export"), fileName.toStdString(), error);
        return;
    }
    m_host.setPrompt(tr("Exported %1.").arg(QFileInfo(fileName).fileName()));
}

}  // namespace hz::ui
