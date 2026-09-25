#include "horizon/ui/DrawingWorkbench.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QLineEdit>
#include <QTimer>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <map>
#include <utility>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/BillOfMaterials.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/Layer.h"
#include "horizon/fileio/DrawingExport.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/Mat4.h"
#include "horizon/modeling/DrawingDimension.h"
#include "horizon/modeling/DrawingProjection.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/SectionView.h"
#include "horizon/render/Camera.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/FeatureForm.h"
#include "horizon/ui/PickTool.h"
#include "horizon/ui/QuantitySpinBox.h"
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

/// Whether @p path names an assembly: its sheet is drawn from its parts.
bool isAssemblyPath(const std::string& path) {
    return QString::fromStdString(path).endsWith(QStringLiteral(".hzasm"), Qt::CaseInsensitive);
}

/// A label as typed, as a caption shows it: trimmed, a few characters.
std::string labelFrom(const QString& typed) {
    constexpr int kMaxLabel = 8;
    return typed.trimmed().left(kMaxLabel).toStdString();
}

double distance(const math::Vec2& a, const math::Vec2& b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

/// What was drawn on @p sheet by hand, as a document of its own: its
/// entities, the layers and blocks they use, and the dimension style. Null
/// when nothing was.
std::shared_ptr<doc::Document> annotationsOf(const doc::Document& sheet) {
    auto notes = std::make_shared<doc::Document>();
    notes->draftDocument().setDimensionStyle(sheet.draftDocument().dimensionStyle());
    for (const std::string& name : sheet.layerManager().layerNames()) {
        if (ownLayer(name)) continue;
        if (const auto* layer = sheet.layerManager().getLayer(name)) {
            notes->layerManager().addLayer(*layer);
        }
    }
    for (const std::string& name : sheet.draftDocument().blockTable().blockNames()) {
        notes->draftDocument().blockTable().addBlock(
            sheet.draftDocument().blockTable().findBlock(name));
    }
    bool any = false;
    for (const auto& entity : sheet.draftDocument().entities()) {
        if (ownLayer(entity->layer())) continue;
        notes->draftDocument().addEntity(entity->clone());
        any = true;
    }
    return any ? notes : nullptr;
}

/// @p notes drawn on @p sheet again: its layers and blocks where the sheet
/// has none of those names, its dimension style, and its entities.
void placeAnnotations(doc::Document& sheet, const doc::Document& notes) {
    // Each layer as it was saved, "0" too: a document starts with a "0" of
    // its own, and keeping that one lost how the user had set theirs.
    for (const std::string& name : notes.layerManager().layerNames()) {
        if (ownLayer(name)) continue;
        if (const auto* layer = notes.layerManager().getLayer(name)) {
            sheet.layerManager().addLayer(*layer);
        }
    }
    auto& blocks = sheet.draftDocument().blockTable();
    for (const std::string& name : notes.draftDocument().blockTable().blockNames()) {
        if (!blocks.findBlock(name))
            blocks.addBlock(notes.draftDocument().blockTable().findBlock(name));
    }
    sheet.draftDocument().setDimensionStyle(notes.draftDocument().dimensionStyle());
    for (const auto& entity : notes.draftDocument().entities()) {
        if (ownLayer(entity->layer())) continue;  // the sheet draws those itself
        sheet.draftDocument().addEntity(entity->clone());
    }
}

/// The room @p source's parts list takes above the title block.
double listedHeight(const model::PartsList& list) {
    return list.rows.empty() ? 0.0 : list.height();
}

/// Where @p drawing's views are on the sheet, as saved with its annotations.
std::vector<io::DrawingViewFrame> framesOf(const model::Drawing& drawing) {
    std::vector<io::DrawingViewFrame> frames;
    frames.reserve(drawing.views.size());
    for (const model::DrawingView& v : drawing.views) {
        frames.push_back({v.role,
                          v.kind,
                          v.label,
                          v.placement,
                          {v.placement.x + v.sheetWidth(), v.placement.y + v.sheetHeight()}});
    }
    return frames;
}

/// The views where @p frames say they were: enough of each to carry what
/// was drawn in it (carryAnnotations).
model::Drawing drawingOf(const std::vector<io::DrawingViewFrame>& frames) {
    model::Drawing drawing;
    drawing.views.reserve(frames.size());
    for (const io::DrawingViewFrame& f : frames) {
        model::DrawingView v;
        v.role = f.role;
        v.kind = f.kind;
        v.label = f.label;
        v.placement = f.low;
        v.boundsMax = f.high - f.low;
        drawing.views.push_back(std::move(v));
    }
    return drawing;
}

/// What was drawn by hand inside a view moves with the view: each view of
/// @p before found in @p after (by what it is), and what lay in its
/// footprint carried by as far as it moved.
void carryAnnotations(doc::Document& sheet, const model::Drawing& before,
                      const model::Drawing& after) {
    std::vector<bool> taken(after.views.size(), false);
    std::vector<bool> carried(sheet.draftDocument().entities().size(), false);
    for (const model::DrawingView& was : before.views) {
        std::size_t match = after.views.size();
        for (std::size_t j = 0; j < after.views.size(); ++j) {
            const model::DrawingView& now = after.views[j];
            if (!taken[j] && now.role == was.role && now.kind == was.kind &&
                now.label == was.label) {
                match = j;
                break;
            }
        }
        if (match == after.views.size()) continue;  // gone: what was on it stays
        taken[match] = true;
        const model::DrawingView& now = after.views[match];
        const math::Vec2 delta{
            now.placement.x + now.sheetWidth() / 2.0 - (was.placement.x + was.sheetWidth() / 2.0),
            now.placement.y + now.sheetHeight() / 2.0 -
                (was.placement.y + was.sheetHeight() / 2.0)};
        if (std::abs(delta.x) < 1e-12 && std::abs(delta.y) < 1e-12) continue;
        const auto [low, high] = was.sheetFootprint();
        auto& entities = sheet.draftDocument().entities();
        for (std::size_t k = 0; k < entities.size(); ++k) {
            const auto& entity = entities[k];
            if (carried[k] || ownLayer(entity->layer())) continue;
            const math::BoundingBox box = entity->boundingBox();
            if (!box.isValid()) continue;
            const math::Vec3 centre = box.center();
            if (centre.x < low.x || centre.x > high.x || centre.y < low.y || centre.y > high.y) {
                continue;
            }
            entity->translate(delta);
            sheet.draftDocument().updateEntityBounds(entity->id());
            carried[k] = true;
        }
    }
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

std::shared_ptr<const DrawingWorkbench::Source> DrawingWorkbench::sourceOf(const std::string& path,
                                                                           std::string* error) {
    auto source = std::make_shared<Source>();
    source->files = {path};
    if (!isAssemblyPath(path)) {
        doc::Document holder;
        const topo::Solid* part = partSolid(path, holder, error);
        if (part == nullptr) return nullptr;
        source->solid = model::Pattern::transformed(*part, math::Mat4::identity());  // its own copy
        return source;
    }

    // The assembly as it is now: its tab's when it is open, else its file's.
    std::shared_ptr<doc::AssemblyDocument> assembly;
    for (const auto& open : m_host.openAssemblies()) {
        if (open && doc::DocumentManager::samePath(open->filePath(), path)) assembly = open;
    }
    if (!assembly) {
        assembly = std::make_shared<doc::AssemblyDocument>();
        std::string why;
        if (!io::NativeFormat::loadAssembly(path, *assembly, &why)) {
            if (error != nullptr) *error = "its assembly could not be read: " + why;
            return nullptr;
        }
        if (assembly->filePath().empty()) assembly->setFilePath(path);
    }

    // Each part read once, from its tab or its file, however many times the
    // assembly places it; the gathered solid is a copy, so what was read
    // goes when this is done.
    const std::filesystem::path folder = std::filesystem::path(path).parent_path();
    const auto resolved = [&folder](const std::string& partPath) {
        const std::filesystem::path part(partPath);
        return (part.is_relative() ? folder / part : part).lexically_normal().string();
    };
    std::vector<std::unique_ptr<doc::Document>> read;
    std::map<std::string, const topo::Solid*> parts;
    const auto partOf = [&](const doc::ComponentInstance& c) -> const topo::Solid* {
        const std::string file = resolved(c.partPath);
        const auto known = parts.find(file);
        if (known != parts.end()) return known->second;
        read.push_back(std::make_unique<doc::Document>());
        const topo::Solid* solid = partSolid(file, *read.back(), nullptr);
        parts.emplace(file, solid);
        source->files.push_back(file);
        return solid;
    };
    source->solid = assembly->drawingSolid(partOf, &source->missing);
    if (!source->solid) {
        if (error != nullptr) *error = "none of its components could be drawn";
        return nullptr;
    }

    // Its bill of materials: the parts list, and a balloon on the first of
    // each part drawn.
    for (const doc::BomLine& line : doc::BomGenerator::generate(*assembly).lines) {
        source->partsList.rows.push_back({line.item, line.partName, line.quantity});
        for (const doc::ComponentInstance& c : assembly->components()) {
            if (c.suppressed ||
                std::find(source->missing.begin(), source->missing.end(), c.id) !=
                    source->missing.end() ||
                !doc::DocumentManager::samePath(resolved(c.partPath), resolved(line.partPath))) {
                continue;
            }
            source->balloons.emplace_back(doc::AssemblyDocument::namePrefix(c.id), line.item);
            break;
        }
    }
    return source;
}

std::shared_ptr<const DrawingWorkbench::Source> DrawingWorkbench::sourceFor(Sheet& sheet,
                                                                            std::string* error) {
    if (!sheet.source) sheet.source = sourceOf(sheet.spec.partPath, error);
    return sheet.source;
}

model::TitleBlock DrawingWorkbench::roomFor(const Sheet& sheet) {
    model::TitleBlock room = sheet.spec.titleBlock;
    room.height += sheet.partsListHeight;
    return room;
}

void DrawingWorkbench::draw(doc::Document& document, io::DrawingDocumentSpec& spec,
                            const Source& source, model::Drawing* drawn) {
    const topo::Solid* solid = source.solid.get();
    const model::PartsList& partsList = source.partsList;
    // The views keep clear of the parts list, above the title block.
    model::TitleBlock room = spec.titleBlock;
    if (!partsList.rows.empty()) room.height += partsList.height();

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
        drawing = model::DrawingGenerator::sheetLayout(*solid, spec.sheet, room, spec.gap, &chosen,
                                                       spec.scale);
        titleBlock.scale = model::DrawingGenerator::scaleName(chosen);
    } else {
        std::vector<std::string> lost;
        drawing = io::DrawingDocumentIO::build(*solid, spec, &lost);
        // Kept, and drawn again should the part have the edges again.
        if (!lost.empty()) {
            QStringList which;
            for (const std::string& l : lost) which << QString::fromStdString(l);
            m_host.showStatus(tr("Dimensions whose edges the part no longer has are not drawn: %1")
                                  .arg(which.join(QStringLiteral("; "))),
                              15000);
        }
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
    // An assembly's balloons, on its isometric view if it has one.
    if (!source.balloons.empty()) {
        model::DrawingView* on = nullptr;
        for (model::DrawingView& view : drawing.views) {
            if (view.role != model::ViewRole::Projection) continue;
            if (on == nullptr || view.kind == model::StandardView::Isometric) on = &view;
            if (view.kind == model::StandardView::Isometric) break;
        }
        for (const auto& [prefix, item] : source.balloons) {
            if (on == nullptr) break;
            if (auto balloon = model::DrawingGenerator::balloonFor(*on, prefix, item)) {
                on->balloons.push_back(*balloon);
            }
        }
    }
    if (!source.missing.empty()) {
        m_host.showStatus(tr("%n component(s) could not be drawn: their parts could not be read",
                             nullptr, static_cast<int>(source.missing.size())),
                          15000);
    }

    document.draftDocument().removeEntities(before);
    io::DrawingExport::populate(document, drawing, &spec.sheet, &titleBlock,
                                partsList.rows.empty() ? nullptr : &partsList);
    for (const draft::LayerProperties& layer : set) document.layerManager().addLayer(layer);
    for (const std::string& name : io::DrawingExport::layers()) {
        if (auto* layer = document.layerManager().getLayer(name)) layer->locked = true;
    }
    // Drawn from the part: the sheet's file is no more out of date than it was.
    document.setDirty(wasDirty);
    if (drawn != nullptr) *drawn = std::move(drawing);
}

void DrawingWorkbench::onNewDrawingFromPart() {
    // The active part or assembly, if it has a file to name; else one chosen.
    std::string partPath;
    const doc::Document* active = m_host.currentDocument();
    if (const auto assembly = m_host.currentAssembly()) {
        partPath = assembly->filePath();
    } else if (active != nullptr && active->type() == doc::DocumentType::Part) {
        partPath = active->filePath();
    }
    if (partPath.empty()) {
        const QString file = QFileDialog::getOpenFileName(
            m_host.dialogParent(), tr("New Drawing from Part or Assembly"), QString(),
            tr("Horizon Parts and Assemblies (*.hzpart *.hzasm);;All Files (*)"));
        if (file.isEmpty()) return;
        partPath = file.toStdString();
    }

    io::DrawingDocumentSpec spec;
    spec.partPath = partPath;
    const QString stem = QFileInfo(QString::fromStdString(partPath)).completeBaseName();
    spec.titleBlock.title = stem.toStdString();

    std::string error;
    const auto source = sourceOf(partPath, &error);
    if (!source) {
        m_host.reportFileError(tr("Could not make a drawing of"), partPath, error);
        return;
    }
    auto document = m_host.documents().newDocument(doc::DocumentType::Drawing);
    model::Drawing drawing;
    draw(*document, spec, *source, &drawing);
    document->setDirty(true);  // new, and not saved
    for (const std::string& file : source->files) m_host.documents().watch(file);
    m_sheets.push_back(
        std::make_unique<Sheet>(Sheet{document, spec, std::move(drawing), source->files,
                                      listedHeight(source->partsList), source}));
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
    const auto source = sourceOf(spec.partPath, &error);
    if (!source) {
        m_host.reportFileError(tr("Could not open"), path, error);
        return false;
    }
    auto document = m_host.documents().newDocument(doc::DocumentType::Drawing);
    model::Drawing drawing;
    draw(*document, spec, *source, &drawing);
    // What was drawn on it by hand, back where it was: the document holds it
    // from here on, and a save writes it from there.
    if (spec.annotations) {
        placeAnnotations(*document, *spec.annotations);
        // The part may have changed since: from where the views were then
        // to where they are drawn now.
        carryAnnotations(*document, drawingOf(spec.frames), drawing);
    }
    spec.annotations.reset();
    spec.frames.clear();
    document->setLengthUnit(spec.lengthUnit);
    document->setFilePath(path);
    document->setDirty(false);
    for (const std::string& file : source->files) m_host.documents().watch(file);
    m_sheets.push_back(
        std::make_unique<Sheet>(Sheet{document, spec, std::move(drawing), source->files,
                                      listedHeight(source->partsList), source}));
    m_host.documents().noteSaved(document);  // found by its path, and watched
    m_host.addTab(document, QFileInfo(fileName).fileName());
    return true;
}

bool DrawingWorkbench::save(doc::Document& document, const std::string& path, std::string* error) {
    Sheet* sheet = sheetOf(&document);
    if (sheet == nullptr) {
        if (error != nullptr) *error = "it is not a drawing sheet";
        return false;
    }
    // Its spec, and what was drawn on it by hand.
    io::DrawingDocumentSpec written = sheet->spec;
    written.lengthUnit = document.lengthUnit();
    written.annotations = annotationsOf(document);
    if (written.annotations) written.frames = framesOf(sheet->drawing);
    if (!io::DrawingDocumentIO::save(path, written)) {
        if (error != nullptr) *error = "the file could not be written";
        return false;
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
    std::shared_ptr<const Source> source;
    if (io::DrawingDocumentIO::readSpec(path, spec, &error))
        source = sourceOf(spec.partPath, &error);
    if (!source) {
        m_host.reportFileError(tr("Could not read again"), path, error);
        return;
    }
    draw(document, spec, *source, &drawing);
    // The file's hand-drawn entities in place of these, and no undoing back
    // into what was given up: read again is the file as it is.
    std::vector<uint64_t> byHand;
    for (const auto& entity : document.draftDocument().entities()) {
        if (!ownLayer(entity->layer())) byHand.push_back(entity->id());
    }
    document.draftDocument().removeEntities(byHand);
    document.undoStack().clear();
    if (spec.annotations) {
        placeAnnotations(document, *spec.annotations);
        carryAnnotations(document, drawingOf(spec.frames), drawing);
    }
    spec.annotations.reset();
    spec.frames.clear();
    document.setLengthUnit(spec.lengthUnit);
    sheet->spec = std::move(spec);
    sheet->drawing = std::move(drawing);
    for (const std::string& file : source->files) m_host.documents().watch(file);
    sheet->files = source->files;
    sheet->partsListHeight = listedHeight(source->partsList);
    sheet->source = source;
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
        const bool from =
            doc::DocumentManager::samePath(sheet->spec.partPath, path) ||
            std::any_of(sheet->files.begin(), sheet->files.end(), [&](const std::string& file) {
                return doc::DocumentManager::samePath(file, path);
            });
        if (!document || !from) continue;
        sheet->source.reset();  // one of its files changed: read them again
        std::string error;
        if (drawAgain(*sheet, &error)) {
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

bool DrawingWorkbench::drawAgain(Sheet& sheet, std::string* error) {
    const auto document = sheet.document.lock();
    if (!document) return false;
    const auto source = sourceFor(sheet, error);
    if (!source) return false;
    model::Drawing drawing;
    draw(*document, sheet.spec, *source, &drawing);
    carryAnnotations(*document, sheet.drawing, drawing);
    sheet.drawing = std::move(drawing);
    // An assembly's parts may be others now: each is watched.
    for (const std::string& file : source->files) m_host.documents().watch(file);
    sheet.files = source->files;
    sheet.partsListHeight = listedHeight(source->partsList);
    return true;
}

void DrawingWorkbench::redraw(Sheet& sheet, const QString& verb) {
    const auto document = sheet.document.lock();
    if (!document) return;
    std::string error;
    if (!drawAgain(sheet, &error)) {
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
    if (spec.views.empty()) {
        // The automatic layout: laid out at the scale when it is drawn.
        spec.scale = scale;
        spec.version = 3;
        return true;
    }
    const auto source = sourceFor(sheet, error);
    if (!source) return false;
    const topo::Solid* solid = source->solid.get();
    spec.scale = scale;
    spec.version = 3;
    const model::TitleBlock room = roomFor(sheet);

    double chosen = 1.0;
    const model::Drawing layout =
        model::DrawingGenerator::sheetLayout(*solid, spec.sheet, room, spec.gap, &chosen, scale);
    // The standard views the sheet still has, laid out anew, each keeping
    // its switches and dimensions; one removed stays removed.
    const std::vector<io::DrawingViewSpec> before = spec.views;
    std::vector<io::DrawingViewSpec> views;
    for (const io::DrawingViewSpec& laid : io::DrawingDocumentIO::viewsOf(layout)) {
        const auto was = std::find_if(before.begin(), before.end(), [&](const auto& v) {
            return v.role == model::ViewRole::Projection && v.kind == laid.kind;
        });
        if (was == before.end()) continue;
        io::DrawingViewSpec kept = laid;
        kept.showHidden = was->showHidden;
        kept.showTangentEdges = was->showTangentEdges;
        kept.showCentreLines = was->showCentreLines;
        kept.dimensions = was->dimensions;
        views.push_back(std::move(kept));
    }
    const std::size_t standard = views.size();
    // Each section and detail, taken from the same view laid out anew: a
    // section at the sheet's scale, a detail as many times its view's.
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
        if (kept.source < 0) continue;  // its view is gone, and it with it
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
        view.placement =
            model::DrawingGenerator::freePlacement(placed, view, spec.sheet, room, spec.gap, &fit);
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

void DrawingWorkbench::addView(Sheet& sheet, const model::DrawingView& view,
                               io::DrawingViewSpec spec, const QString& verb) {
    // An automatic sheet becomes the views it shows, and this one with them.
    if (sheet.spec.views.empty()) sheet.spec.views = io::DrawingDocumentIO::viewsOf(sheet.drawing);
    sheet.spec.version = 3;
    bool fits = true;
    spec.placement = model::DrawingGenerator::freePlacement(sheet.drawing, view, sheet.spec.sheet,
                                                            roomFor(sheet), sheet.spec.gap, &fits);
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
    FeatureForm form(m_host.dialogParent(), verb, m_host.currentDocument()->lengthUnit());
    auto* view = form.choice(QStringLiteral("view"), tr("Through view:"), names);
    auto* cut = form.choice(QStringLiteral("cut"), tr("Cut:"), {tr("Vertical"), tr("Horizontal")});
    auto* offset =
        form.length(QStringLiteral("offset"), tr("From the view's centre:"), 0.0, -1e6, 1e6);
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

    std::string error;
    const auto source = sourceFor(*sheet, &error);
    if (!source) {
        m_host.showStatus(tr("%1: %2").arg(verb, QString::fromStdString(error)), 15000);
        return;
    }
    const topo::Solid* solid = source->solid.get();
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
    addView(*sheet, section, std::move(spec), verb);
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
    addView(*sheet, detail, std::move(spec), verb);
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

std::optional<DrawingWorkbench::PickedEdge> DrawingWorkbench::edgeAt(const Sheet& sheet,
                                                                     const math::Vec2& at) {
    constexpr double kReach = 2.0;  // sheet millimetres
    std::optional<PickedEdge> best;
    double nearest = kReach;
    const auto& views = sheet.drawing.views;
    for (std::size_t i = 0; i < views.size(); ++i) {
        const model::DrawingView& view = views[i];
        for (const model::ProjectedEdge& e : view.edges) {
            // Only what is drawn, and only an edge with a name to keep.
            const bool visible = e.visibility == model::ProjectedEdge::Visibility::Visible;
            if (!visible && !view.showHidden) continue;
            if (e.kind == model::ProjectedEdge::Kind::Tangent && !view.showTangentEdges) continue;
            if (e.sourceEdge.tag().empty()) continue;
            const math::Vec2 a = view.toSheet(e.a);
            const math::Vec2 b = view.toSheet(e.b);
            const math::Vec2 ab = b - a;
            const double length2 = ab.dot(ab);
            const double t =
                length2 > 1e-18 ? std::clamp((at - a).dot(ab) / length2, 0.0, 1.0) : 0.0;
            const double d = distance(at, a + ab * t);
            if (d <= nearest) {
                nearest = d;
                best = PickedEdge{static_cast<int>(i), e.sourceEdge};
            }
        }
    }
    return best;
}

void DrawingWorkbench::pickEdges(const QString& verb, const std::weak_ptr<doc::Document>& document,
                                 void (DrawingWorkbench::*use)(const std::weak_ptr<doc::Document>&,
                                                               const PickedEdge&)) {
    const auto picked = std::make_shared<PickedEdge>();
    auto tool = std::make_unique<PickTool>(
        verb.toStdString(),
        std::vector<std::string>{tr("Click an edge of a view (Esc when done)").toStdString()},
        [this, document, picked](std::size_t /*step*/, const math::Vec2& point, std::string& why) {
            const auto shown = document.lock();
            const Sheet* sheet = sheetOf(shown.get());
            if (sheet == nullptr || m_host.currentDocument() != shown.get()) {
                why = tr("Not on the drawing being dimensioned").toStdString();
                return false;
            }
            const auto edge = edgeAt(*sheet, point);
            if (!edge) {
                why = tr("No edge there").toStdString();
                return false;
            }
            *picked = *edge;
            return true;
        },
        [this, document, picked, use](const std::vector<math::Vec2>& /*points*/) {
            // Out of the tool's own event; the tool stays for the next edge.
            QTimer::singleShot(
                0, this, [this, document, use, edge = *picked] { (this->*use)(document, edge); });
        });
    m_host.runTool(std::move(tool));
}

void DrawingWorkbench::onAddDimension() {
    const QString verb = tr("Add Dimension");
    const Sheet* active = activeSheet(verb);
    if (active == nullptr) return;
    pickEdges(verb, active->document, &DrawingWorkbench::addDimension);
}

void DrawingWorkbench::onRemoveDimension() {
    const QString verb = tr("Remove Dimension");
    const Sheet* active = activeSheet(verb);
    if (active == nullptr) return;
    pickEdges(verb, active->document, &DrawingWorkbench::removeDimensions);
}

void DrawingWorkbench::addDimension(const std::weak_ptr<doc::Document>& document,
                                    const PickedEdge& picked) {
    const QString verb = tr("Add Dimension");
    const std::shared_ptr<doc::Document> shown = document.lock();
    Sheet* sheet = sheetOf(shown.get());
    if (sheet == nullptr) return;
    if (sheet->spec.views.empty())
        sheet->spec.views = io::DrawingDocumentIO::viewsOf(sheet->drawing);
    if (picked.view < 0 || static_cast<std::size_t>(picked.view) >= sheet->spec.views.size()) {
        return;
    }
    auto& dimensions = sheet->spec.views[static_cast<std::size_t>(picked.view)].dimensions;
    // A curve is dimensioned whole, by its own name, not one chord's.
    const std::string tag = model::logicalEdge(picked.edge.tag());
    if (std::any_of(dimensions.begin(), dimensions.end(),
                    [&](const auto& d) { return d.edge == tag; })) {
        m_host.showStatus(
            tr("%1: that edge is dimensioned already; Remove Dimension removes it").arg(verb));
        return;
    }

    // A circle or arc is dimensioned by its diameter or radius, anything else
    // by its length: measured from the part as it is.
    std::string error;
    const auto source = sourceFor(*sheet, &error);
    if (!source) {
        m_host.showStatus(tr("%1: %2").arg(verb, QString::fromStdString(error)), 15000);
        return;
    }
    const topo::Solid* solid = source->solid.get();
    io::DrawingDimensionSpec dimension{tag, io::DrawingDimensionSpec::Kind::Length};
    double measured = 0.0;
    if (model::DrawingDimensioner::measureRadius(*solid, picked.edge, measured)) {
        // A whole circle, or an arc: a curve is one edge of many chords by
        // one name. It is whole when its chords close, every end shared.
        std::map<const topo::Vertex*, int> ends;
        for (const topo::Edge& e : solid->edges()) {
            if (model::logicalEdge(e.topoId.tag()) != tag || e.halfEdge == nullptr ||
                e.halfEdge->twin == nullptr) {
                continue;
            }
            ++ends[e.halfEdge->origin];
            ++ends[e.halfEdge->twin->origin];
        }
        const bool closed =
            !ends.empty() && std::all_of(ends.begin(), ends.end(),
                                         [](const auto& end) { return end.second % 2 == 0; });
        dimension.kind = closed ? io::DrawingDimensionSpec::Kind::Diameter
                                : io::DrawingDimensionSpec::Kind::Radius;
    } else if (!model::DrawingDimensioner::isStraight(*solid, tag)) {
        // Neither a circle nor straight (an ellipse, a lofted edge): no one
        // number says how large it is.
        m_host.showStatus(
            tr("%1: that edge is curved, but not a circle or arc; its size is not stated")
                .arg(verb));
        return;
    } else if (!model::DrawingDimensioner::measureEdge(*solid, picked.edge, measured) ||
               measured < 1e-9) {
        m_host.showStatus(tr("%1: that edge has no length to state").arg(verb));
        return;
    }
    dimensions.push_back(std::move(dimension));
    sheet->spec.version = 3;
    redraw(*sheet, verb);
}

void DrawingWorkbench::removeDimensions(const std::weak_ptr<doc::Document>& document,
                                        const PickedEdge& picked) {
    const QString verb = tr("Remove Dimension");
    const std::shared_ptr<doc::Document> shown = document.lock();
    Sheet* sheet = sheetOf(shown.get());
    if (sheet == nullptr || picked.view < 0 ||
        static_cast<std::size_t>(picked.view) >= sheet->spec.views.size()) {
        m_host.showStatus(tr("%1: that edge has no dimension").arg(verb));
        return;
    }
    auto& dimensions = sheet->spec.views[static_cast<std::size_t>(picked.view)].dimensions;
    const std::string tag = model::logicalEdge(picked.edge.tag());
    const auto removed =
        std::erase_if(dimensions, [&](const io::DrawingDimensionSpec& d) { return d.edge == tag; });
    if (removed == 0) {
        m_host.showStatus(tr("%1: that edge has no dimension").arg(verb));
        return;
    }
    redraw(*sheet, verb);
}

void DrawingWorkbench::onViewProperties() {
    const QString verb = tr("View Properties");
    const Sheet* active = activeSheet(verb);
    if (active == nullptr) return;
    const std::shared_ptr<doc::Document> document = active->document.lock();
    std::vector<std::array<bool, 3>> shownNow;
    QStringList names;
    for (std::size_t i = 0; i < active->drawing.views.size(); ++i) {
        const model::DrawingView& v = active->drawing.views[i];
        names << viewName(v, i);
        shownNow.push_back({v.showHidden, v.showTangentEdges, v.showCentreLines});
    }
    if (names.isEmpty()) return;
    FeatureForm form(m_host.dialogParent(), verb);
    auto* view = form.choice(QStringLiteral("view"), tr("View:"), names);
    const QStringList either{tr("Shown"), tr("Left out")};
    std::array<QComboBox*, 3> switches{
        form.choice(QStringLiteral("hidden"), tr("Hidden edges:"), either),
        form.choice(QStringLiteral("tangent"), tr("Tangent edges:"), either),
        form.choice(QStringLiteral("centreLines"), tr("Centre lines:"), either)};
    // Each switch shows the view chosen as it is.
    const auto showFor = [switches, shownNow](int index) {
        const auto& now = shownNow[static_cast<std::size_t>(std::max(index, 0))];
        for (std::size_t k = 0; k < switches.size(); ++k) {
            switches[k]->setCurrentIndex(now[k] ? 0 : 1);
        }
    };
    showFor(0);
    QObject::connect(view, &QComboBox::currentIndexChanged, view, showFor);
    if (!form.exec()) return;
    Sheet* sheet = sheetOf(document.get());
    if (sheet == nullptr) return;
    if (sheet->spec.views.empty())
        sheet->spec.views = io::DrawingDocumentIO::viewsOf(sheet->drawing);
    const auto index = static_cast<std::size_t>(std::max(view->currentIndex(), 0));
    if (index >= sheet->spec.views.size()) return;
    io::DrawingViewSpec& chosen = sheet->spec.views[index];
    chosen.showHidden = switches[0]->currentIndex() == 0;
    chosen.showTangentEdges = switches[1]->currentIndex() == 0;
    chosen.showCentreLines = switches[2]->currentIndex() == 0;
    sheet->spec.version = 3;
    redraw(*sheet, verb);
}

void DrawingWorkbench::onUpdateFromPart() {
    Sheet* sheet = activeSheet(tr("Update from Part"));
    if (sheet == nullptr) return;
    const auto document = sheet->document.lock();
    if (!document) return;
    // The part as it is now, in its tab or its file: read again, not kept.
    sheet->source.reset();
    std::string error;
    if (!drawAgain(*sheet, &error)) {
        m_host.showStatus(
            tr("The drawing could not be drawn again: %1").arg(QString::fromStdString(error)));
        return;
    }
    m_host.viewport().update();
    m_host.showStatus(tr("Drawn again from its part"), 5000);
}

}  // namespace hz::ui
