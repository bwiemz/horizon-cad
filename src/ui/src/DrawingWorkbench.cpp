#include "horizon/ui/DrawingWorkbench.h"

#include <QComboBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QLineEdit>
#include <algorithm>
#include <cmath>
#include <utility>

#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/Layer.h"
#include "horizon/fileio/DrawingExport.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/render/Camera.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/FeatureForm.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/ui/WorkbenchHost.h"

namespace hz::ui {

namespace {

const std::vector<model::PaperSize>& paperSizes() {
    static const std::vector<model::PaperSize> sizes{
        model::PaperSize::A0,    model::PaperSize::A1,    model::PaperSize::A2,
        model::PaperSize::A3,    model::PaperSize::A4,    model::PaperSize::AnsiA,
        model::PaperSize::AnsiB, model::PaperSize::AnsiC, model::PaperSize::AnsiD};
    return sizes;
}

bool ownLayer(const std::string& layer) {
    const auto& own = io::DrawingExport::layers();
    return std::find(own.begin(), own.end(), layer) != own.end();
}

}  // namespace

DrawingWorkbench::DrawingWorkbench(WorkbenchHost& host, QObject* parent)
    : QObject(parent), m_host(host) {}

const DrawingWorkbench::Sheet* DrawingWorkbench::sheetOf(const doc::Document* document) const {
    if (document == nullptr) return nullptr;
    for (const Sheet& sheet : m_sheets) {
        if (sheet.document.lock().get() == document) return &sheet;
    }
    return nullptr;
}

DrawingWorkbench::Sheet* DrawingWorkbench::sheetOf(const doc::Document* document) {
    return const_cast<Sheet*>(std::as_const(*this).sheetOf(document));
}

bool DrawingWorkbench::isSheet(const doc::Document* document) const {
    return sheetOf(document) != nullptr;
}

const model::Sheet* DrawingWorkbench::paperOf(const doc::Document* document) const {
    const Sheet* sheet = sheetOf(document);
    return sheet != nullptr ? &sheet->spec.sheet : nullptr;
}

DrawingWorkbench::Sheet* DrawingWorkbench::activeSheet(const QString& verb) {
    Sheet* sheet = m_host.currentAssembly() ? nullptr : sheetOf(m_host.currentDocument());
    if (sheet == nullptr) {
        m_host.showStatus(
            tr("%1 works on a drawing sheet: open one, or make one from a part").arg(verb));
    }
    return sheet;
}

bool DrawingWorkbench::draw(doc::Document& document, io::DrawingDocumentSpec& spec,
                            std::string* error) {
    // The part as it is now: its tab's document when it is open and built,
    // else its file.
    const topo::Solid* solid = nullptr;
    doc::Document fromFile;
    if (const auto open = m_host.documents().findByPath(spec.partPath);
        open != nullptr && open->solid() != nullptr) {
        solid = open->solid();
    } else {
        std::string reason;
        if (!io::NativeFormat::load(spec.partPath, fromFile, &reason)) {
            if (error != nullptr) *error = "its part could not be read: " + reason;
            return false;
        }
        if (!fromFile.rebuildModel() || fromFile.solid() == nullptr) {
            if (error != nullptr) *error = "its part did not rebuild";
            return false;
        }
        solid = fromFile.solid();
    }

    // The layout, and the scale it chose for the title block to state.
    model::TitleBlock titleBlock = spec.titleBlock;
    model::Drawing drawing;
    if (spec.version < 2) {
        // Laid out as version 1 did, and kept so: as its views, which a save
        // writes. Saved without them, it would open again laid out anew.
        drawing = model::DrawingGenerator::standardViews(*solid, spec.gap);
        spec.views = io::DrawingDocumentIO::viewsOf(drawing);
        spec.version = 2;
    } else if (spec.views.empty()) {
        double chosen = 1.0;
        drawing = model::DrawingGenerator::sheetLayout(*solid, spec.sheet, titleBlock, spec.gap,
                                                       &chosen, spec.scale);
        titleBlock.scale = model::DrawingGenerator::scaleName(chosen);
    } else {
        drawing = io::DrawingDocumentIO::build(*solid, spec);
    }

    // What the sheet drew before goes; what was drawn on it by hand stays.
    std::vector<uint64_t> drawn;
    for (const auto& entity : document.draftDocument().entities()) {
        if (ownLayer(entity->layer())) drawn.push_back(entity->id());
    }
    const bool wasDirty = document.isDirty();
    // The sheet's layers as they were set (shown or not, their colours):
    // populate() makes them anew.
    std::vector<draft::LayerProperties> set;
    for (const std::string& name : io::DrawingExport::layers()) {
        if (const auto* layer = document.layerManager().getLayer(name)) set.push_back(*layer);
    }
    document.draftDocument().removeEntities(drawn);
    io::DrawingExport::populate(document, drawing, &spec.sheet, &titleBlock);
    for (const draft::LayerProperties& layer : set) document.layerManager().addLayer(layer);
    for (const std::string& name : io::DrawingExport::layers()) {
        if (auto* layer = document.layerManager().getLayer(name)) layer->locked = true;
    }
    // Drawn from the part: the sheet's file is no more out of date than it was.
    document.setDirty(wasDirty);
    return true;
}

void DrawingWorkbench::onNewDrawingFromPart() {
    // The active part, if it has a file to name; else one chosen.
    std::string partPath;
    const doc::Document* active = m_host.currentDocument();
    if (!m_host.currentAssembly() && active != nullptr &&
        active->type() == doc::DocumentType::Part && !active->filePath().empty()) {
        partPath = active->filePath();
    }
    if (partPath.empty()) {
        const QString file =
            QFileDialog::getOpenFileName(m_host.dialogParent(), tr("New Drawing from Part"),
                                         QString(), tr("Horizon Parts (*.hzpart);;All Files (*)"));
        if (file.isEmpty()) return;
        partPath = file.toStdString();
    }

    io::DrawingDocumentSpec spec;
    spec.partPath = partPath;
    const QString stem = QFileInfo(QString::fromStdString(partPath)).completeBaseName();
    spec.titleBlock.title = stem.toStdString();

    auto document = m_host.documents().newDocument(doc::DocumentType::Drawing);
    std::string error;
    if (!draw(*document, spec, &error)) {
        m_host.documents().closeDocument(document);
        m_host.reportFileError(tr("Could not make a drawing of"), partPath, error);
        return;
    }
    document->setDirty(true);  // new, and not saved
    m_sheets.push_back({document, spec});
    m_host.documents().watch(partPath);
    m_host.addTab(document, tr("Drawing of %1").arg(stem));
    m_host.setPrompt(tr("Drawing made."));
}

bool DrawingWorkbench::open(const QString& fileName) {
    io::DrawingDocumentSpec spec;
    std::string error;
    const std::string path = fileName.toStdString();
    if (!io::DrawingDocumentIO::readSpec(path, spec, &error)) {
        m_host.reportFileError(tr("Could not open"), path, error);
        return false;
    }
    auto document = m_host.documents().newDocument(doc::DocumentType::Drawing);
    if (!draw(*document, spec, &error)) {
        m_host.documents().closeDocument(document);
        m_host.reportFileError(tr("Could not open"), path, error);
        return false;
    }
    document->setFilePath(path);
    document->setDirty(false);
    m_sheets.push_back({document, spec});
    m_host.documents().noteSaved(document);  // found by its path, and watched
    m_host.documents().watch(spec.partPath);
    m_host.addTab(document, QFileInfo(fileName).fileName());
    return true;
}

bool DrawingWorkbench::save(doc::Document& document, const std::string& path, std::string* error) {
    Sheet* sheet = sheetOf(&document);
    if (sheet == nullptr) {
        if (error != nullptr) *error = "it is not a drawing sheet";
        return false;
    }
    if (!io::DrawingDocumentIO::save(path, sheet->spec)) {
        if (error != nullptr) *error = "the file could not be written";
        return false;
    }
    // Said, not lost unsaid: a sheet keeps its part, layout and title block;
    // what was drawn on it by hand is not in the file yet.
    const bool drawnByHand = std::any_of(
        document.draftDocument().entities().begin(), document.draftDocument().entities().end(),
        [](const auto& entity) { return !ownLayer(entity->layer()); });
    if (drawnByHand) {
        m_host.showStatus(tr("Drawing saved; what was drawn on the sheet by hand is not saved "
                             "with it"),
                          15000);
    }
    return true;
}

void DrawingWorkbench::readAgain(doc::Document& document) {
    Sheet* sheet = sheetOf(&document);
    if (sheet == nullptr) return;
    const std::string path = document.filePath();
    io::DrawingDocumentSpec spec;
    std::string error;
    if (!io::DrawingDocumentIO::readSpec(path, spec, &error) || !draw(document, spec, &error)) {
        m_host.reportFileError(tr("Could not read again"), path, error);
        return;
    }
    sheet->spec = std::move(spec);
    m_host.documents().watch(sheet->spec.partPath);
    document.setDirty(false);
    m_host.refreshModifiedIndicators();
    // Its paper may be another size now.
    if (m_host.currentDocument() == &document) shown(document);
}

void DrawingWorkbench::shown(doc::Document& document) {
    const Sheet* sheet = sheetOf(&document);
    if (sheet == nullptr) return;
    // A sheet is paper: seen from above, all of it in view.
    auto& camera = m_host.viewport().camera();
    camera.setTopView();
    math::BoundingBox paper;
    paper.expand(math::Vec3(0.0, 0.0, 0.0));
    paper.expand(math::Vec3(sheet->spec.sheet.widthMm(), sheet->spec.sheet.heightMm(), 0.0));
    camera.fitAll(paper);
    m_host.viewport().update();
}

void DrawingWorkbench::refreshDrawingsOf(const std::string& path) {
    int redrawn = 0;
    for (Sheet& sheet : m_sheets) {
        const auto document = sheet.document.lock();
        if (!document || !doc::DocumentManager::samePath(sheet.spec.partPath, path)) continue;
        std::string error;
        if (draw(*document, sheet.spec, &error)) {
            ++redrawn;
        } else {
            m_host.showStatus(tr("A drawing of \"%1\" could not be drawn again: %2")
                                  .arg(QFileInfo(QString::fromStdString(path)).fileName(),
                                       QString::fromStdString(error)),
                              15000);
        }
    }
    // Sheets whose tabs have closed are forgotten.
    std::erase_if(m_sheets, [](const Sheet& sheet) { return sheet.document.expired(); });
    if (redrawn > 0) m_host.viewport().update();
}

void DrawingWorkbench::redraw(Sheet& sheet, const QString& verb) {
    const auto document = sheet.document.lock();
    if (!document) return;
    std::string error;
    if (!draw(*document, sheet.spec, &error)) {
        m_host.showStatus(tr("%1: the drawing could not be drawn again: %2")
                              .arg(verb, QString::fromStdString(error)));
        return;
    }
    document->setDirty(true);
    m_host.refreshModifiedIndicators();
    m_host.viewport().update();
}

void DrawingWorkbench::onTitleBlock() {
    const QString verb = tr("Title Block");
    Sheet* sheet = activeSheet(verb);
    if (sheet == nullptr) return;
    model::TitleBlock& tb = sheet->spec.titleBlock;
    FeatureForm form(m_host.dialogParent(), verb);
    const auto field = [&form](const char* name, const QString& label, const std::string& value) {
        return form.text(QString::fromLatin1(name), label, QString::fromStdString(value));
    };
    auto* title = field("title", tr("Title:"), tb.title);
    auto* number = field("partNumber", tr("Part number:"), tb.partNumber);
    auto* revision = field("revision", tr("Revision:"), tb.revision);
    auto* drawnBy = field("drawnBy", tr("Drawn by:"), tb.drawnBy);
    auto* date = field("date", tr("Date:"), tb.date);
    auto* material = field("material", tr("Material:"), tb.material);
    auto* company = field("company", tr("Company:"), tb.company);
    auto* sheetNumber = field("sheetNumber", tr("Sheet:"), tb.sheetNumber);
    QStringList papers;
    int current = 0;
    for (const model::PaperSize size : paperSizes()) {
        if (size == sheet->spec.sheet.size) current = static_cast<int>(papers.size());
        papers << QString::fromStdString(model::paperSizeName(size));
    }
    auto* paper = form.choice(QStringLiteral("paper"), tr("Paper:"), papers);
    paper->setCurrentIndex(current);
    auto* orientation = form.choice(QStringLiteral("orientation"), tr("Orientation:"),
                                    {tr("Landscape"), tr("Portrait")});
    orientation->setCurrentIndex(
        sheet->spec.sheet.orientation == model::Orientation::Landscape ? 0 : 1);
    if (!form.exec()) return;

    const auto read = [](QLineEdit* edit) { return edit->text().trimmed().toStdString(); };
    tb.title = read(title);
    tb.partNumber = read(number);
    tb.revision = read(revision);
    tb.drawnBy = read(drawnBy);
    tb.date = read(date);
    tb.material = read(material);
    tb.company = read(company);
    tb.sheetNumber = read(sheetNumber);
    sheet->spec.sheet.size = paperSizes()[static_cast<size_t>(std::max(paper->currentIndex(), 0))];
    sheet->spec.sheet.orientation = orientation->currentIndex() == 1
                                        ? model::Orientation::Portrait
                                        : model::Orientation::Landscape;
    // A new paper: the automatic layout follows it.
    redraw(*sheet, verb);
    if (auto document = sheet->document.lock()) shown(*document);
}

void DrawingWorkbench::onScale() {
    const QString verb = tr("Drawing Scale");
    Sheet* sheet = activeSheet(verb);
    if (sheet == nullptr) return;
    QStringList scales{tr("Largest that fits")};
    int current = 0;
    for (const double s : model::DrawingGenerator::standardScales()) {
        if (std::abs(s - sheet->spec.scale) < 1e-12) current = static_cast<int>(scales.size());
        scales << QString::fromStdString(model::DrawingGenerator::scaleName(s));
    }
    FeatureForm form(m_host.dialogParent(), verb);
    auto* choice = form.choice(QStringLiteral("scale"), tr("Scale:"), scales);
    choice->setCurrentIndex(current);
    if (!form.exec()) return;
    const int index = choice->currentIndex();
    sheet->spec.scale =
        index <= 0 ? 0.0
                   : model::DrawingGenerator::standardScales()[static_cast<size_t>(index - 1)];
    sheet->spec.views.clear();  // laid out again, at the scale chosen
    sheet->spec.version = 2;
    redraw(*sheet, verb);
}

void DrawingWorkbench::onUpdateFromPart() {
    Sheet* sheet = activeSheet(tr("Update from Part"));
    if (sheet == nullptr) return;
    const auto document = sheet->document.lock();
    if (!document) return;
    std::string error;
    if (!draw(*document, sheet->spec, &error)) {
        m_host.showStatus(
            tr("The drawing could not be drawn again: %1").arg(QString::fromStdString(error)));
        return;
    }
    m_host.viewport().update();
    m_host.showStatus(tr("Drawn again from its part"), 5000);
}

}  // namespace hz::ui
