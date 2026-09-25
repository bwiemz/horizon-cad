#include "horizon/ui/DrawingWorkbench.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QLineEdit>
#include <QTimer>
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
#include "horizon/modeling/DrawingProjection.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/SectionView.h"
#include "horizon/render/Camera.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/FeatureForm.h"
#include "horizon/ui/PickTool.h"
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

/// A view as the forms list it: "2: Top", "5: Section A-A", "6: Detail B".
QString viewName(const model::DrawingView& view, std::size_t index) {
    QString name;
    switch (view.role) {
        case model::ViewRole::Section:
            name = QObject::tr("Section %1-%1").arg(QString::fromStdString(view.label));
            break;
        case model::ViewRole::Detail:
            name = QObject::tr("Detail %1").arg(QString::fromStdString(view.label));
            break;
        case model::ViewRole::Projection:
            switch (view.kind) {
                case model::StandardView::Front:
                    name = QObject::tr("Front");
                    break;
                case model::StandardView::Top:
                    name = QObject::tr("Top");
                    break;
                case model::StandardView::Right:
                    name = QObject::tr("Right");
                    break;
                case model::StandardView::Isometric:
                    name = QObject::tr("Isometric");
                    break;
            }
            break;
    }
    return QStringLiteral("%1: %2").arg(index + 1).arg(name);
}

/// A label as typed, as a caption shows it: trimmed, a few characters.
std::string labelFrom(const QString& typed) {
    constexpr int kMaxLabel = 8;
    return typed.trimmed().left(kMaxLabel).toStdString();
}

double distance(const math::Vec2& a, const math::Vec2& b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

}  // namespace

DrawingWorkbench::DrawingWorkbench(WorkbenchHost& host, QObject* parent)
    : QObject(parent), m_host(host) {}

const DrawingWorkbench::Sheet* DrawingWorkbench::sheetOf(const doc::Document* document) const {
    if (document == nullptr) return nullptr;
    for (const auto& sheet : m_sheets) {
        if (sheet->document.lock().get() == document) return sheet.get();
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

const topo::Solid* DrawingWorkbench::partSolid(const std::string& path, doc::Document& holder,
                                               std::string* error) {
    // The part as it is now: its tab's document when it is open and built,
    // else its file.
    if (const auto open = m_host.documents().findByPath(path);
        open != nullptr && open->solid() != nullptr) {
        return open->solid();
    }
    std::string reason;
    if (!io::NativeFormat::load(path, holder, &reason)) {
        if (error != nullptr) *error = "its part could not be read: " + reason;
        return nullptr;
    }
    if (!holder.rebuildModel() || holder.solid() == nullptr) {
        if (error != nullptr) *error = "its part did not rebuild";
        return nullptr;
    }
    return holder.solid();
}

bool DrawingWorkbench::draw(doc::Document& document, io::DrawingDocumentSpec& spec,
                            std::string* error, model::Drawing* drawn) {
    doc::Document fromFile;
    const topo::Solid* solid = partSolid(spec.partPath, fromFile, error);
    if (solid == nullptr) return false;

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
        // The sheet's scale is its projections'.
        for (const model::DrawingView& view : drawing.views) {
            if (view.role != model::ViewRole::Projection) continue;
            titleBlock.scale = model::DrawingGenerator::scaleName(view.scale);
            break;
        }
    }

    // What the sheet drew before goes; what was drawn on it by hand stays.
    std::vector<uint64_t> before;
    for (const auto& entity : document.draftDocument().entities()) {
        if (ownLayer(entity->layer())) before.push_back(entity->id());
    }
    const bool wasDirty = document.isDirty();
    // The sheet's layers as they were set (shown or not, their colours):
    // populate() makes them anew.
    std::vector<draft::LayerProperties> set;
    for (const std::string& name : io::DrawingExport::layers()) {
        if (const auto* layer = document.layerManager().getLayer(name)) set.push_back(*layer);
    }
    document.draftDocument().removeEntities(before);
    io::DrawingExport::populate(document, drawing, &spec.sheet, &titleBlock);
    for (const draft::LayerProperties& layer : set) document.layerManager().addLayer(layer);
    for (const std::string& name : io::DrawingExport::layers()) {
        if (auto* layer = document.layerManager().getLayer(name)) layer->locked = true;
    }
    // Drawn from the part: the sheet's file is no more out of date than it was.
    document.setDirty(wasDirty);
    if (drawn != nullptr) *drawn = std::move(drawing);
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
    model::Drawing drawing;
    if (!draw(*document, spec, &error, &drawing)) {
        m_host.documents().closeDocument(document);
        m_host.reportFileError(tr("Could not make a drawing of"), partPath, error);
        return;
    }
    document->setDirty(true);  // new, and not saved
    m_sheets.push_back(std::make_unique<Sheet>(Sheet{document, spec, std::move(drawing)}));
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
    model::Drawing drawing;
    if (!draw(*document, spec, &error, &drawing)) {
        m_host.documents().closeDocument(document);
        m_host.reportFileError(tr("Could not open"), path, error);
        return false;
    }
    document->setFilePath(path);
    document->setDirty(false);
    m_sheets.push_back(std::make_unique<Sheet>(Sheet{document, spec, std::move(drawing)}));
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
    model::Drawing drawing;
    if (!io::DrawingDocumentIO::readSpec(path, spec, &error) ||
        !draw(document, spec, &error, &drawing)) {
        m_host.reportFileError(tr("Could not read again"), path, error);
        return;
    }
    sheet->spec = std::move(spec);
    sheet->drawing = std::move(drawing);
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
    for (const auto& sheet : m_sheets) {
        const auto document = sheet->document.lock();
        if (!document || !doc::DocumentManager::samePath(sheet->spec.partPath, path)) continue;
        std::string error;
        if (draw(*document, sheet->spec, &error, &sheet->drawing)) {
            ++redrawn;
        } else {
            m_host.showStatus(tr("A drawing of \"%1\" could not be drawn again: %2")
                                  .arg(QFileInfo(QString::fromStdString(path)).fileName(),
                                       QString::fromStdString(error)),
                              15000);
        }
    }
    // Sheets whose tabs have closed are forgotten.
    std::erase_if(m_sheets, [](const auto& sheet) { return sheet->document.expired(); });
    if (redrawn > 0) m_host.viewport().update();
}

void DrawingWorkbench::redraw(Sheet& sheet, const QString& verb) {
    const auto document = sheet.document.lock();
    if (!document) return;
    std::string error;
    if (!draw(*document, sheet.spec, &error, &sheet.drawing)) {
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
    const Sheet* active = activeSheet(verb);
    if (active == nullptr) return;
    // What the form starts from; the sheet is found again when it closes.
    const std::shared_ptr<doc::Document> document = active->document.lock();
    const model::TitleBlock tb = active->spec.titleBlock;
    const model::Sheet paperWas = active->spec.sheet;
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
        if (size == paperWas.size) current = static_cast<int>(papers.size());
        papers << QString::fromStdString(model::paperSizeName(size));
    }
    auto* paper = form.choice(QStringLiteral("paper"), tr("Paper:"), papers);
    paper->setCurrentIndex(current);
    auto* orientation = form.choice(QStringLiteral("orientation"), tr("Orientation:"),
                                    {tr("Landscape"), tr("Portrait")});
    orientation->setCurrentIndex(paperWas.orientation == model::Orientation::Landscape ? 0 : 1);
    if (!form.exec()) return;
    // The form ran the events: the sheet is found again, not remembered.
    Sheet* sheet = sheetOf(document.get());
    if (sheet == nullptr) return;

    const auto read = [](QLineEdit* edit) { return edit->text().trimmed().toStdString(); };
    model::TitleBlock& fields = sheet->spec.titleBlock;
    fields.title = read(title);
    fields.partNumber = read(number);
    fields.revision = read(revision);
    fields.drawnBy = read(drawnBy);
    fields.date = read(date);
    fields.material = read(material);
    fields.company = read(company);
    fields.sheetNumber = read(sheetNumber);
    sheet->spec.sheet.size = paperSizes()[static_cast<size_t>(std::max(paper->currentIndex(), 0))];
    sheet->spec.sheet.orientation = orientation->currentIndex() == 1
                                        ? model::Orientation::Portrait
                                        : model::Orientation::Landscape;
    // A new paper: the views are laid out on it again.
    bool fits = true;
    if (sheet->spec.sheet.size != paperWas.size ||
        sheet->spec.sheet.orientation != paperWas.orientation) {
        std::string error;
        if (!layOutAgain(*sheet, sheet->spec.scale, &fits, &error)) {
            m_host.showStatus(tr("%1: %2").arg(verb, QString::fromStdString(error)), 15000);
        }
    }
    redraw(*sheet, verb);
    shown(*document);
    if (!fits) {
        m_host.showStatus(tr("Not every view has room on this paper: those without are beside it"),
                          15000);
    }
}

void DrawingWorkbench::onScale() {
    const QString verb = tr("Drawing Scale");
    const Sheet* active = activeSheet(verb);
    if (active == nullptr) return;
    const std::shared_ptr<doc::Document> document = active->document.lock();
    const double scaleWas = active->spec.scale;
    QStringList scales{tr("Largest that fits")};
    int current = 0;
    for (const double s : model::DrawingGenerator::standardScales()) {
        if (std::abs(s - scaleWas) < 1e-12) current = static_cast<int>(scales.size());
        scales << QString::fromStdString(model::DrawingGenerator::scaleName(s));
    }
    FeatureForm form(m_host.dialogParent(), verb);
    auto* choice = form.choice(QStringLiteral("scale"), tr("Scale:"), scales);
    choice->setCurrentIndex(current);
    if (!form.exec()) return;
    // The form ran the events: the sheet is found again, not remembered.
    Sheet* sheet = sheetOf(document.get());
    if (sheet == nullptr) return;
    const int index = choice->currentIndex();
    const double scale =
        index <= 0 ? 0.0
                   : model::DrawingGenerator::standardScales()[static_cast<size_t>(index - 1)];
    bool fits = true;
    std::string error;
    if (!layOutAgain(*sheet, scale, &fits, &error)) {
        m_host.showStatus(tr("%1: %2").arg(verb, QString::fromStdString(error)), 15000);
        return;
    }
    redraw(*sheet, verb);
    if (!fits) {
        m_host.showStatus(tr("At this scale not every view has room on the sheet: those without "
                             "are beside it"),
                          15000);
    }
}

bool DrawingWorkbench::layOutAgain(Sheet& sheet, double scale, bool* fits, std::string* error) {
    if (fits != nullptr) *fits = true;
    io::DrawingDocumentSpec& spec = sheet.spec;
    const bool added = std::any_of(spec.views.begin(), spec.views.end(), [](const auto& v) {
        return v.role != model::ViewRole::Projection;
    });
    if (!added) {
        // The standard views alone: the automatic layout, at the scale.
        spec.scale = scale;
        spec.views.clear();
        spec.version = 3;
        return true;
    }
    doc::Document holder;
    const topo::Solid* solid = partSolid(spec.partPath, holder, error);
    if (solid == nullptr) return false;
    spec.scale = scale;
    spec.version = 3;

    double chosen = 1.0;
    const model::Drawing layout = model::DrawingGenerator::sheetLayout(
        *solid, spec.sheet, spec.titleBlock, spec.gap, &chosen, scale);
    std::vector<io::DrawingViewSpec> views = io::DrawingDocumentIO::viewsOf(layout);
    const std::size_t standard = views.size();
    // Each section and detail, taken from the same view laid out anew: a
    // section at the sheet's scale, a detail as many times its view's.
    const std::vector<io::DrawingViewSpec> before = spec.views;
    for (const io::DrawingViewSpec& v : before) {
        if (v.role == model::ViewRole::Projection) continue;
        io::DrawingViewSpec kept = v;
        kept.source = -1;
        double ratio = 1.0;
        if (v.source >= 0 && static_cast<std::size_t>(v.source) < before.size()) {
            const io::DrawingViewSpec& from = before[static_cast<std::size_t>(v.source)];
            for (std::size_t i = 0; i < standard; ++i) {
                if (views[i].kind == from.kind) kept.source = static_cast<int>(i);
            }
            ratio = v.scale / from.scale;
        }
        if (v.role == model::ViewRole::Detail && kept.source < 0) continue;  // its view is gone
        kept.scale = chosen;
        if (v.role == model::ViewRole::Detail) {
            // As many times its view's as before, at the standard scale that
            // is at least that.
            const auto& scales = model::DrawingGenerator::standardScales();  // largest first
            kept.scale = scales.front();
            for (const double s : scales) {
                if (s >= ratio * chosen * (1.0 - 1e-9)) kept.scale = s;
            }
        }
        views.push_back(kept);
    }
    spec.views = std::move(views);

    // Placed again, one after another, where there is room.
    const model::Drawing built = io::DrawingDocumentIO::build(*solid, spec);
    model::Drawing placed;
    placed.views.assign(built.views.begin(),
                        built.views.begin() + static_cast<std::ptrdiff_t>(standard));
    for (std::size_t i = standard; i < built.views.size(); ++i) {
        model::DrawingView view = built.views[i];
        bool fit = true;
        view.placement = model::DrawingGenerator::freePlacement(placed, view, spec.sheet,
                                                                spec.titleBlock, spec.gap, &fit);
        spec.views[i].placement = view.placement;
        if (!fit && fits != nullptr) *fits = false;
        placed.views.push_back(std::move(view));
    }
    return true;
}

int DrawingWorkbench::viewAt(const Sheet& sheet, const math::Vec2& at, bool projectionOnly) {
    const auto& views = sheet.drawing.views;
    for (std::size_t i = views.size(); i-- > 0;) {  // the last drawn is on top
        const model::DrawingView& v = views[i];
        if (projectionOnly && v.role != model::ViewRole::Projection) continue;
        const math::Vec2 low = v.placement;
        const math::Vec2 high{low.x + v.sheetWidth(), low.y + v.sheetHeight()};
        if (at.x >= low.x && at.x <= high.x && at.y >= low.y && at.y <= high.y) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

QString DrawingWorkbench::nextLabel(const Sheet& sheet) {
    // Letters that are not read as numbers or as each other are left out.
    for (char c = 'A'; c <= 'Z'; ++c) {
        if (c == 'I' || c == 'O' || c == 'Q') continue;
        const std::string letter(1, c);
        const bool used = std::any_of(sheet.spec.views.begin(), sheet.spec.views.end(),
                                      [&](const auto& v) { return v.label == letter; });
        if (!used) return QString(QLatin1Char(c));
    }
    return QStringLiteral("Z1");
}

void DrawingWorkbench::addView(Sheet& sheet, model::DrawingView view, io::DrawingViewSpec spec,
                               const QString& verb) {
    // An automatic sheet becomes the views it shows, and this one with them.
    if (sheet.spec.views.empty()) sheet.spec.views = io::DrawingDocumentIO::viewsOf(sheet.drawing);
    sheet.spec.version = 3;
    bool fits = true;
    spec.placement = model::DrawingGenerator::freePlacement(
        sheet.drawing, view, sheet.spec.sheet, sheet.spec.titleBlock, sheet.spec.gap, &fits);
    sheet.spec.views.push_back(std::move(spec));
    redraw(sheet, verb);
    m_host.showStatus(fits ? tr("%1: placed where there was room; Move View moves it").arg(verb)
                           : tr("%1: no room on the sheet, so it is beside it; move it with Move "
                                "View, or choose a smaller scale")
                                 .arg(verb),
                      15000);
}

void DrawingWorkbench::onAddSectionView() {
    const QString verb = tr("Add Section View");
    const Sheet* active = activeSheet(verb);
    if (active == nullptr) return;
    const std::shared_ptr<doc::Document> document = active->document.lock();
    // Cut through a view seen square: front, top or side.
    std::vector<std::size_t> through;
    QStringList names;
    for (std::size_t i = 0; i < active->drawing.views.size(); ++i) {
        const model::DrawingView& v = active->drawing.views[i];
        if (v.role != model::ViewRole::Projection || v.kind == model::StandardView::Isometric) {
            continue;
        }
        through.push_back(i);
        names << viewName(v, i);
    }
    if (through.empty()) {
        m_host.showStatus(tr("%1: a section is cut through a front, top or side view").arg(verb));
        return;
    }
    FeatureForm form(m_host.dialogParent(), verb);
    auto* view = form.choice(QStringLiteral("view"), tr("Through view:"), names);
    auto* cut = form.choice(QStringLiteral("cut"), tr("Cut:"), {tr("Vertical"), tr("Horizontal")});
    auto* offset =
        form.number(QStringLiteral("offset"), tr("From the view's centre:"), 0.0, -1e6, 1e6);
    auto* looking = form.choice(QStringLiteral("direction"), tr("Looking:"),
                                {tr("Left or down"), tr("Right or up")});
    auto* label = form.text(QStringLiteral("label"), tr("Label:"), nextLabel(*active));
    if (!form.exec()) return;
    // The form ran the events: the sheet is found again, not remembered.
    Sheet* sheet = sheetOf(document.get());
    const std::size_t chosen = through[static_cast<std::size_t>(std::max(view->currentIndex(), 0))];
    if (sheet == nullptr || chosen >= sheet->drawing.views.size()) return;
    const model::DrawingView from = sheet->drawing.views[chosen];

    // The cut in the model: square to the view, through the point at the
    // offset from the view's centre, across or up it.
    const auto [right, up] = model::DrawingProjection::axes(from.projection);
    const bool vertical = cut->currentIndex() == 0;
    const math::Vec3 normal = vertical ? right : up;
    const math::Vec2 centre{(from.boundsMin.x + from.boundsMax.x) / 2.0,
                            (from.boundsMin.y + from.boundsMax.y) / 2.0};
    const double at = offset->value();
    const math::Vec3 point =
        from.projection.origin + right * centre.x + up * centre.y + normal * at;
    const math::Vec3 sight = looking->currentIndex() == 0 ? normal * -1.0 : normal;

    doc::Document holder;
    std::string error;
    const topo::Solid* solid = partSolid(sheet->spec.partPath, holder, &error);
    if (solid == nullptr) {
        m_host.showStatus(tr("%1: %2").arg(verb, QString::fromStdString(error)), 15000);
        return;
    }
    model::DrawingView section =
        model::SectionGenerator::sectionView(*solid, point, sight * -1.0, 3.0 / from.scale);
    if (section.sectionLoops.empty()) {
        m_host.showStatus(tr("%1: the cut misses the part").arg(verb), 15000);
        return;
    }
    io::DrawingViewSpec spec;
    spec.kind = from.kind;
    spec.role = model::ViewRole::Section;
    spec.label = labelFrom(label->text());
    spec.source = static_cast<int>(chosen);
    spec.projection = section.projection;
    spec.scale = from.scale;
    spec.showHidden = false;
    section.role = spec.role;
    section.label = spec.label;
    section.scale = spec.scale;
    addView(*sheet, std::move(section), std::move(spec), verb);
}

void DrawingWorkbench::onAddDetailView() {
    const QString verb = tr("Add Detail View");
    const Sheet* active = activeSheet(verb);
    if (active == nullptr) return;
    const std::weak_ptr<doc::Document> document = active->document;
    const auto source = std::make_shared<int>(-1);
    auto tool = std::make_unique<PickTool>(
        "Detail View",
        std::vector<std::string>{tr("Click the centre of the detail, on a view").toStdString(),
                                 tr("Click its radius").toStdString()},
        [this, document, source](std::size_t step, const math::Vec2& point, std::string& why) {
            const auto shown = document.lock();
            const Sheet* sheet = sheetOf(shown.get());
            if (sheet == nullptr || m_host.currentDocument() != shown.get()) {
                why = tr("Not on the drawing the detail is of").toStdString();
                return false;
            }
            if (step > 0) return true;
            *source = viewAt(*sheet, point, true);
            if (*source < 0) why = tr("Not on a view").toStdString();
            return *source >= 0;
        },
        [this, document, source](const std::vector<math::Vec2>& points) {
            // Out of the tool's own event before it ends.
            QTimer::singleShot(0, this, [this, document, source, points] {
                m_host.endTool();
                if (points.size() == 2) addDetail(document, *source, points[0], points[1]);
            });
        });
    tool->setPreview(nullptr, [](const std::vector<math::Vec2>& picked, const math::Vec2& pointer) {
        return std::vector<std::pair<math::Vec2, double>>{
            {picked.front(), distance(picked.front(), pointer)}};
    });
    m_host.runTool(std::move(tool));
}

void DrawingWorkbench::addDetail(const std::weak_ptr<doc::Document>& document, int source,
                                 const math::Vec2& centre, const math::Vec2& onCircle) {
    const QString verb = tr("Add Detail View");
    const std::shared_ptr<doc::Document> shown = document.lock();
    const Sheet* active = sheetOf(shown.get());
    if (active == nullptr || source < 0 ||
        static_cast<std::size_t>(source) >= active->drawing.views.size()) {
        return;
    }
    const model::DrawingView from = active->drawing.views[static_cast<std::size_t>(source)];
    const double radius = distance(centre, onCircle) / from.scale;
    if (radius < 1e-9) {
        m_host.showStatus(tr("%1: a detail needs a radius").arg(verb));
        return;
    }
    // Larger than its view: the standard scales above the view's.
    std::vector<double> larger;
    for (const double s : model::DrawingGenerator::standardScales()) {
        if (s > from.scale * (1.0 + 1e-9)) larger.push_back(s);
    }
    if (larger.empty()) {
        m_host.showStatus(tr("%1: the view is drawn at the largest scale already").arg(verb));
        return;
    }
    std::sort(larger.begin(), larger.end());
    QStringList scales;
    int initial = static_cast<int>(larger.size()) - 1;
    for (std::size_t i = 0; i < larger.size(); ++i) {
        scales << QString::fromStdString(model::DrawingGenerator::scaleName(larger[i]));
        if (initial == static_cast<int>(larger.size()) - 1 &&
            larger[i] >= 2.0 * from.scale - 1e-9) {
            initial = static_cast<int>(i);  // twice the view's, or the next above
        }
    }
    FeatureForm form(m_host.dialogParent(), verb);
    auto* scale = form.choice(QStringLiteral("scale"), tr("Scale:"), scales);
    scale->setCurrentIndex(initial);
    auto* label = form.text(QStringLiteral("label"), tr("Label:"), nextLabel(*active));
    if (!form.exec()) return;
    Sheet* sheet = sheetOf(shown.get());
    if (sheet == nullptr || static_cast<std::size_t>(source) >= sheet->drawing.views.size()) {
        return;
    }

    // The circle in the view's own space (model millimetres).
    const math::Vec2 at{(centre.x - from.placement.x) / from.scale + from.boundsMin.x,
                        (centre.y - from.placement.y) / from.scale + from.boundsMin.y};
    model::DrawingView detail = model::DrawingGenerator::detailView(
        sheet->drawing.views[static_cast<std::size_t>(source)], at, radius, 1.0);
    if (detail.edges.empty()) {
        m_host.showStatus(tr("%1: the circle holds nothing of the view").arg(verb), 15000);
        return;
    }
    io::DrawingViewSpec spec;
    spec.kind = from.kind;
    spec.role = model::ViewRole::Detail;
    spec.label = labelFrom(label->text());
    spec.source = source;
    spec.projection = from.projection;
    spec.detailCenter = at;
    spec.detailRadius = radius;
    spec.scale = larger[static_cast<std::size_t>(std::max(scale->currentIndex(), 0))];
    spec.showHidden = from.showHidden;
    spec.showTangentEdges = from.showTangentEdges;
    detail.role = spec.role;
    detail.label = spec.label;
    detail.scale = spec.scale;
    addView(*sheet, std::move(detail), std::move(spec), verb);
}

void DrawingWorkbench::onMoveView() {
    const QString verb = tr("Move View");
    const Sheet* active = activeSheet(verb);
    if (active == nullptr) return;
    const std::weak_ptr<doc::Document> document = active->document;
    const auto moving = std::make_shared<int>(-1);
    auto tool = std::make_unique<PickTool>(
        "Move View",
        std::vector<std::string>{tr("Click a view").toStdString(),
                                 tr("Click where that point goes").toStdString()},
        [this, document, moving](std::size_t step, const math::Vec2& point, std::string& why) {
            const auto shown = document.lock();
            const Sheet* sheet = sheetOf(shown.get());
            if (sheet == nullptr || m_host.currentDocument() != shown.get()) {
                why = tr("Not on the drawing the view is on").toStdString();
                return false;
            }
            if (step > 0) return true;
            *moving = viewAt(*sheet, point, false);
            if (*moving < 0) why = tr("Not on a view").toStdString();
            return *moving >= 0;
        },
        [this, document, moving](const std::vector<math::Vec2>& points) {
            QTimer::singleShot(0, this, [this, document, moving, points] {
                m_host.endTool();
                const auto shown = document.lock();
                Sheet* sheet = sheetOf(shown.get());
                if (sheet == nullptr || points.size() != 2 || *moving < 0) return;
                if (sheet->spec.views.empty()) {
                    sheet->spec.views = io::DrawingDocumentIO::viewsOf(sheet->drawing);
                }
                const auto index = static_cast<std::size_t>(*moving);
                if (index >= sheet->spec.views.size()) return;
                math::Vec2& at = sheet->spec.views[index].placement;
                at = {at.x + points[1].x - points[0].x, at.y + points[1].y - points[0].y};
                sheet->spec.version = 3;
                redraw(*sheet, tr("Move View"));
            });
        });
    // The view's outline, carried with the pointer.
    tool->setPreview(
        [this, document, moving](const std::vector<math::Vec2>& picked, const math::Vec2& pointer) {
            std::vector<std::pair<math::Vec2, math::Vec2>> lines;
            const Sheet* sheet = sheetOf(document.lock().get());
            if (sheet == nullptr || *moving < 0 ||
                static_cast<std::size_t>(*moving) >= sheet->drawing.views.size()) {
                return lines;
            }
            const model::DrawingView& v = sheet->drawing.views[static_cast<std::size_t>(*moving)];
            const double dx = pointer.x - picked.front().x;
            const double dy = pointer.y - picked.front().y;
            const math::Vec2 a{v.placement.x + dx, v.placement.y + dy};
            const math::Vec2 b{a.x + v.sheetWidth(), a.y + v.sheetHeight()};
            lines.push_back({a, {b.x, a.y}});
            lines.push_back({{b.x, a.y}, b});
            lines.push_back({b, {a.x, b.y}});
            lines.push_back({{a.x, b.y}, a});
            return lines;
        },
        nullptr);
    m_host.runTool(std::move(tool));
}

void DrawingWorkbench::onRemoveView() {
    const QString verb = tr("Remove View");
    const Sheet* active = activeSheet(verb);
    if (active == nullptr) return;
    const std::shared_ptr<doc::Document> document = active->document.lock();
    if (active->drawing.views.size() < 2) {
        m_host.showStatus(tr("%1: a sheet keeps at least one view").arg(verb));
        return;
    }
    QStringList names;
    for (std::size_t i = 0; i < active->drawing.views.size(); ++i) {
        names << viewName(active->drawing.views[i], i);
    }
    FeatureForm form(m_host.dialogParent(), verb);
    auto* view = form.choice(QStringLiteral("view"), tr("View:"), names);
    if (!form.exec()) return;
    Sheet* sheet = sheetOf(document.get());
    if (sheet == nullptr) return;
    if (sheet->spec.views.empty())
        sheet->spec.views = io::DrawingDocumentIO::viewsOf(sheet->drawing);
    auto& views = sheet->spec.views;
    const auto removed = static_cast<std::size_t>(std::max(view->currentIndex(), 0));
    if (removed >= views.size()) return;

    // It goes, and the views taken from it with it; the rest keep their
    // sources, counted again.
    std::vector<bool> gone(views.size(), false);
    gone[removed] = true;
    for (std::size_t i = 0; i < views.size(); ++i) {
        if (views[i].source == static_cast<int>(removed)) gone[i] = true;
    }
    if (std::all_of(gone.begin(), gone.end(), [](bool g) { return g; })) {
        m_host.showStatus(tr("%1: a sheet keeps at least one view").arg(verb));
        return;
    }
    std::vector<int> renumbered(views.size(), -1);
    std::vector<io::DrawingViewSpec> kept;
    for (std::size_t i = 0; i < views.size(); ++i) {
        if (gone[i]) continue;
        renumbered[i] = static_cast<int>(kept.size());
        kept.push_back(views[i]);
    }
    for (io::DrawingViewSpec& v : kept) {
        if (v.source >= 0) v.source = renumbered[static_cast<std::size_t>(v.source)];
    }
    const auto taken = std::count(gone.begin(), gone.end(), true) - 1;
    views = std::move(kept);
    sheet->spec.version = 3;
    redraw(*sheet, verb);
    if (taken > 0) {
        m_host.showStatus(tr("%1: the views taken from it went with it (%2)").arg(verb).arg(taken),
                          15000);
    }
}

void DrawingWorkbench::onUpdateFromPart() {
    Sheet* sheet = activeSheet(tr("Update from Part"));
    if (sheet == nullptr) return;
    const auto document = sheet->document.lock();
    if (!document) return;
    std::string error;
    if (!draw(*document, sheet->spec, &error, &sheet->drawing)) {
        m_host.showStatus(
            tr("The drawing could not be drawn again: %1").arg(QString::fromStdString(error)));
        return;
    }
    m_host.viewport().update();
    m_host.showStatus(tr("Drawn again from its part"), 5000);
}

}  // namespace hz::ui
