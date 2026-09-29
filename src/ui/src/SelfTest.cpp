#include "horizon/ui/SelfTest.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTranslator>
#include <cmath>
#include <exception>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/Sketch.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/PlotScene.h"
#include "horizon/fileio/DrawingDocumentIO.h"
#include "horizon/fileio/DrawingExport.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/fileio/SvgExport.h"
#include "horizon/math/Mat4.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/FacePlane.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/PdfExport.h"

namespace hz::ui::selftest {

namespace {

using math::Vec2;
using math::Vec3;

// The part: a block 40 by 30, first 10 thick, then made 20, with a hole of
// 8 through it from its top.
constexpr double kWidth = 40.0;
constexpr double kDepth = 30.0;
constexpr double kFirstThickness = 10.0;
constexpr double kThickness = 20.0;
constexpr double kHoleDiameter = 8.0;
// A hole's wall is faceted: its volume is a little under the circle's.
constexpr double kVolumeTolerance = 0.005;  // relative

QString number(double value) {
    return QString::number(value, 'f', 1);
}

std::string utf8(const QString& path) {
    return path.toStdString();
}

double volumeOf(const doc::Document& document) {
    return document.solid() ? model::MassPropertiesCalculator::compute(*document.solid()).volume
                            : 0.0;
}

bool near(double value, double expected) {
    return std::abs(value - expected) <= kVolumeTolerance * expected;
}

/// @p document built, every feature of it: why not, when it is not.
std::optional<QString> buildFailure(doc::Document& document) {
    if (document.rebuildModel() && document.failedFeatureIndex() == -1 &&
        document.solid() != nullptr) {
        return std::nullopt;
    }
    return QStringLiteral("it does not build: %1")
        .arg(QString::fromStdString(document.lastBuildMessage()));
}

/// The whole name of @p solid's flat face facing @p way through @p point.
std::optional<std::string> faceAt(const topo::Solid& solid, const Vec3& way, const Vec3& point) {
    const double outward = model::outwardSign(solid);
    for (const auto& face : solid.faces()) {
        const auto plane = model::planeOf(face, outward);
        if (plane && plane->normal.dot(way) > 0.999 &&
            std::abs((point - plane->origin).dot(plane->normal)) < 1e-6) {
            return model::wholeFaceName(face.topoId.tag());
        }
    }
    return std::nullopt;
}

Step fail(Step step, const QString& why) {
    step.passed = false;
    step.detail = why;
    return step;
}

/// Modelled, edited, saved as @p path and read back.
Step part(const QString& path, double& volume) {
    Step step{QStringLiteral("part"), false, {}};
    doc::Document document;
    document.setType(doc::DocumentType::Part);
    auto sketch = std::make_shared<doc::Sketch>();
    sketch->setName("Profile");
    const Vec2 corners[] = {Vec2(0, 0), Vec2(kWidth, 0), Vec2(kWidth, kDepth), Vec2(0, kDepth)};
    for (std::size_t i = 0; i < 4; ++i) {
        sketch->addEntity(std::make_shared<draft::DraftLine>(corners[i], corners[(i + 1) % 4]));
    }
    auto& undo = document.undoStack();
    auto add = std::make_unique<doc::AddFeatureCommand>(
        document, std::make_unique<doc::ExtrudeFeature>(sketch, Vec3(0, 0, 1), kFirstThickness),
        sketch);
    const doc::Feature* block = add->feature();
    undo.push(std::move(add));
    if (auto why = buildFailure(document)) return fail(step, *why);
    const double first = kWidth * kDepth * kFirstThickness;
    if (!near(volumeOf(document), first)) {
        return fail(step, QStringLiteral("extruded, its volume is %1 mm³, not %2")
                              .arg(number(volumeOf(document)), number(first)));
    }

    // Edited as its form edits it: thicker, then undone and done again.
    const double thick = kWidth * kDepth * kThickness;
    const auto builtAs = [&](const char* what, double expected) -> std::optional<QString> {
        if (auto why = buildFailure(document)) return why;
        if (near(volumeOf(document), expected)) return std::nullopt;
        return QStringLiteral("%1, its volume is %2 mm³, not %3")
            .arg(QString::fromLatin1(what), number(volumeOf(document)), number(expected));
    };
    undo.push(std::make_unique<doc::EditFeatureCommand>(
        document, block, std::map<std::string, double>{{"distance", kThickness}}));
    if (auto why = builtAs("made thicker", thick)) return fail(step, *why);
    undo.undo();
    if (auto why = builtAs("undone", first)) return fail(step, *why);
    undo.redo();
    if (auto why = builtAs("redone", thick)) return fail(step, *why);

    // A hole drilled through it from its top.
    const Vec3 at(kWidth / 2, kDepth / 2, kThickness);
    const auto top = faceAt(*document.solid(), Vec3(0, 0, 1), at);
    if (!top) return fail(step, QStringLiteral("its top face was not found"));
    auto hole = doc::HoleFeature::make(*top, at, kHoleDiameter, kThickness);
    hole->setParameter("extent", 1);  // through all
    undo.push(std::make_unique<doc::AddFeatureCommand>(document, std::move(hole)));
    if (auto why = buildFailure(document)) return fail(step, *why);
    const double radius = kHoleDiameter / 2;
    const double drilled = thick - std::numbers::pi * radius * radius * kThickness;
    volume = volumeOf(document);
    if (!near(volume, drilled)) {
        return fail(step, QStringLiteral("drilled, its volume is %1 mm³, not %2")
                              .arg(number(volume), number(drilled)));
    }

    std::string error;
    if (!io::NativeFormat::save(utf8(path), document, &error)) {
        return fail(step,
                    QStringLiteral("it could not be saved: %1").arg(QString::fromStdString(error)));
    }
    doc::Document read;
    if (!io::NativeFormat::load(utf8(path), read, &error)) {
        return fail(
            step,
            QStringLiteral("it could not be read back: %1").arg(QString::fromStdString(error)));
    }
    if (auto why = buildFailure(read)) return fail(step, QStringLiteral("read back, ") + *why);
    if (read.featureTree().featureCount() != 2 || std::abs(volumeOf(read) - volume) > 1e-6) {
        return fail(step, QStringLiteral("read back, it has %1 features and %2 mm³, not 2 and %3")
                              .arg(read.featureTree().featureCount())
                              .arg(number(volumeOf(read)), number(volume)));
    }
    step.passed = true;
    step.detail =
        QStringLiteral("modelled, edited, undone and redone, saved and read back (%1 mm³)")
            .arg(number(volume));
    return step;
}

/// The part at @p partPath sent out as STEP to @p path and read in again.
Step exchange(const QString& partPath, const QString& path, double volume) {
    Step step{QStringLiteral("step"), false, {}};
    doc::Document document;
    std::string error;
    if (!io::NativeFormat::load(utf8(partPath), document, &error) ||
        buildFailure(document).has_value()) {
        return fail(step, QStringLiteral("there is no part to send out"));
    }
    if (!io::StepFormat::save(utf8(path), {document.solid()})) {
        return fail(step, QStringLiteral("it could not be written: %1")
                              .arg(QString::fromStdString(io::StepFormat::lastError())));
    }
    const auto solids = io::StepFormat::load(utf8(path));
    if (solids.size() != 1 || !solids.front()) {
        return fail(step, QStringLiteral("it was read back as %1 solids, not 1: %2")
                              .arg(solids.size())
                              .arg(QString::fromStdString(io::StepFormat::lastError())));
    }
    const double read = model::MassPropertiesCalculator::compute(*solids.front()).volume;
    if (!near(read, volume)) {
        return fail(step, QStringLiteral("read back, its volume is %1 mm³, not %2")
                              .arg(number(read), number(volume)));
    }
    step.passed = true;
    step.detail = QStringLiteral("written and read back (%1 mm³)").arg(number(read));
    return step;
}

/// The part at @p partPath placed twice, one fixed where it is and the other
/// 60 along; saved as @p path and read back.
Step assembly(const QString& partPath, const QString& path) {
    Step step{QStringLiteral("assembly"), false, {}};
    if (!QFileInfo::exists(partPath))
        return fail(step, QStringLiteral("there is no part to place"));
    doc::AssemblyDocument assembly;
    doc::ComponentInstance first;
    first.name = "Block 1";
    first.partPath = utf8(partPath);
    const std::uint64_t fixedId = assembly.addComponent(first);
    doc::ComponentInstance second;
    second.name = "Block 2";
    second.partPath = utf8(partPath);
    second.transform = math::Mat4::translation(Vec3(60, 0, 0));
    assembly.addComponent(second);
    doc::Mate fixed;
    fixed.type = doc::MateType::Fixed;
    fixed.a.componentId = fixedId;
    assembly.addMate(fixed);

    std::string error;
    if (!io::NativeFormat::saveAssembly(utf8(path), assembly, &error)) {
        return fail(step,
                    QStringLiteral("it could not be saved: %1").arg(QString::fromStdString(error)));
    }
    doc::AssemblyDocument read;
    if (!io::NativeFormat::loadAssembly(utf8(path), read, &error)) {
        return fail(
            step,
            QStringLiteral("it could not be read back: %1").arg(QString::fromStdString(error)));
    }
    if (read.components().size() != 2 || read.mates().size() != 1) {
        return fail(step,
                    QStringLiteral("read back, it has %1 components and %2 mates, not 2 and 1")
                        .arg(read.components().size())
                        .arg(read.mates().size()));
    }
    const Vec3 at = read.components()[1].transform.transformPoint(Vec3());
    if ((at - Vec3(60, 0, 0)).length() > 1e-9) {
        return fail(step, QStringLiteral("read back, its second component is at (%1, %2, %3)")
                              .arg(number(at.x), number(at.y), number(at.z)));
    }
    // Shown from the part's own file, as an assembly shows its parts.
    for (const auto& component : read.components()) {
        const auto mesh = io::NativeFormat::loadPartMesh(component.partPath);
        if (!mesh || mesh->indices.empty()) {
            return fail(step, QStringLiteral("the part of \"%1\" cannot be shown from its file")
                                  .arg(QString::fromStdString(component.name)));
        }
    }
    step.passed = true;
    step.detail = QStringLiteral("two components and a mate, saved and read back");
    return step;
}

/// The first bytes of @p path.
QByteArray headOf(const QString& path) {
    QFile file(path);
    return file.open(QIODevice::ReadOnly) ? file.read(64) : QByteArray();
}

/// A sheet of the part at @p partPath, saved in @p folder and read back,
/// then exported to PDF, SVG and DXF there.
Step drawing(const QString& partPath, const QString& folder) {
    Step step{QStringLiteral("drawing"), false, {}};
    if (!QFileInfo::exists(partPath)) return fail(step, QStringLiteral("there is no part to draw"));
    const QDir dir(folder);
    io::DrawingDocumentSpec spec;
    spec.partPath = utf8(partPath);
    spec.titleBlock.title = "Self-test block";
    spec.titleBlock.partNumber = "HZ-SELF-TEST";
    const QString sheetPath = dir.filePath(QStringLiteral("block-drawing.hzdwg"));
    if (!io::DrawingDocumentIO::save(utf8(sheetPath), spec)) {
        return fail(step, QStringLiteral("its sheet could not be saved"));
    }
    io::DrawingDocumentSpec read;
    model::Drawing views;
    std::string error;
    if (!io::DrawingDocumentIO::load(utf8(sheetPath), read, views, &error)) {
        return fail(step, QStringLiteral("its sheet could not be read back: %1")
                              .arg(QString::fromStdString(error)));
    }
    if (views.views.size() < 3) {
        return fail(step,
                    QStringLiteral("read back, its sheet has %1 views").arg(views.views.size()));
    }

    // What the drawing's tab shows, prints and exports.
    doc::Document sheet;
    io::DrawingExport::populate(sheet, views, &read.sheet, &read.titleBlock);
    const draft::PlotScene scene = draft::buildPlotScene(
        sheet.draftDocument(), sheet.layerManager(), sheet.draftDocument().dimensionStyle());
    if (scene.empty()) return fail(step, QStringLiteral("its sheet draws nothing"));
    draft::PlotLayout layout;
    layout.paperWidthMm = read.sheet.widthMm();
    layout.paperHeightMm = read.sheet.heightMm();
    layout.scale = 0.0;  // fitted

    const QString pdf = dir.filePath(QStringLiteral("block-drawing.pdf"));
    QString why;
    if (!exportPdf(pdf, scene, layout, &why) || !headOf(pdf).startsWith("%PDF-")) {
        return fail(step, QStringLiteral("its PDF could not be written: %1").arg(why));
    }
    const QString svg = dir.filePath(QStringLiteral("block-drawing.svg"));
    if (!io::SvgExport::save(utf8(svg), scene, layout, &error) || !headOf(svg).contains("<")) {
        return fail(
            step,
            QStringLiteral("its SVG could not be written: %1").arg(QString::fromStdString(error)));
    }
    const QString dxf = dir.filePath(QStringLiteral("block-drawing.dxf"));
    if (!io::DxfFormat::save(utf8(dxf), sheet, &error)) {
        return fail(
            step,
            QStringLiteral("its DXF could not be written: %1").arg(QString::fromStdString(error)));
    }
    doc::Document again;
    if (!io::DxfFormat::load(utf8(dxf), again, &error) ||
        again.draftDocument().entities().empty()) {
        return fail(step, QStringLiteral("its DXF could not be read back: %1")
                              .arg(QString::fromStdString(error)));
    }
    step.passed = true;
    step.detail = QStringLiteral("%1 views, saved and read back, exported to PDF, SVG and DXF")
                      .arg(views.views.size());
    return step;
}

/// Every sample in @p folder opens, and a part builds.
Step samples(const QString& folder) {
    Step step{QStringLiteral("samples"), false, {}};
    const QDir dir(folder);
    const QFileInfoList files =
        dir.entryInfoList({QStringLiteral("*.hzpart"), QStringLiteral("*.hzasm"),
                           QStringLiteral("*.hzdwg"), QStringLiteral("*.hcad")},
                          QDir::Files, QDir::Name);
    if (files.isEmpty()) {
        return fail(
            step, QStringLiteral("there are none in %1").arg(QDir::toNativeSeparators(dir.path())));
    }
    for (const QFileInfo& file : files) {
        const std::string path = utf8(file.filePath());
        const QString suffix = file.suffix().toLower();
        std::string error;
        bool opened = false;
        if (suffix == QLatin1String("hzasm")) {
            doc::AssemblyDocument assembly;
            opened = io::NativeFormat::loadAssembly(path, assembly, &error);
        } else if (suffix == QLatin1String("hzdwg")) {
            io::DrawingDocumentSpec spec;
            model::Drawing views;
            opened = io::DrawingDocumentIO::load(path, spec, views, &error);
        } else {
            doc::Document document;
            opened = io::NativeFormat::load(path, document, &error);
            if (opened && suffix == QLatin1String("hzpart")) {
                if (auto why = buildFailure(document)) {
                    error = utf8(*why);
                    opened = false;
                }
            }
        }
        if (!opened) {
            return fail(step, QStringLiteral("%1 does not open: %2")
                                  .arg(file.fileName(), QString::fromStdString(error)));
        }
    }
    step.passed = true;
    step.detail = QStringLiteral("%1 open").arg(files.size());
    return step;
}

/// @p expected catalogs in @p folder, each loaded.
Step translations(const QString& folder, int expected) {
    Step step{QStringLiteral("translations"), false, {}};
    if (expected <= 0) {
        step.passed = true;
        step.detail = QStringLiteral("none looked for: this build has none");
        return step;
    }
    const QDir dir(folder);
    const QFileInfoList catalogs =
        dir.entryInfoList({QStringLiteral("horizon_*.qm")}, QDir::Files, QDir::Name);
    for (const QFileInfo& catalog : catalogs) {
        QTranslator translator;
        if (!translator.load(catalog.filePath())) {
            return fail(step, QStringLiteral("%1 does not load").arg(catalog.fileName()));
        }
    }
    if (catalogs.size() != expected) {
        return fail(step, QStringLiteral("%1 in %2, not %3")
                              .arg(catalogs.size())
                              .arg(QDir::toNativeSeparators(dir.path()))
                              .arg(expected));
    }
    step.passed = true;
    step.detail = QStringLiteral("%1 load").arg(catalogs.size());
    return step;
}

/// @p check run, whatever it throws said as its failure.
template <typename Check>
Step guarded(const QString& name, Check check) {
    try {
        return check();
    } catch (const std::exception& e) {
        return fail(Step{name, false, {}}, QStringLiteral("it threw: %1").arg(e.what()));
    } catch (...) {
        return fail(Step{name, false, {}}, QStringLiteral("it threw"));
    }
}

}  // namespace

std::vector<Step> runWorkflows(const QString& folder, const Shipped& shipped) {
    const QDir dir(folder);
    const QString partPath = dir.filePath(QStringLiteral("block.hzpart"));
    double volume = 0.0;
    std::vector<Step> steps;
    steps.push_back(guarded(QStringLiteral("part"), [&] { return part(partPath, volume); }));
    steps.push_back(guarded(QStringLiteral("step"), [&] {
        return exchange(partPath, dir.filePath(QStringLiteral("block.step")), volume);
    }));
    steps.push_back(guarded(QStringLiteral("assembly"), [&] {
        return assembly(partPath, dir.filePath(QStringLiteral("blocks.hzasm")));
    }));
    steps.push_back(
        guarded(QStringLiteral("drawing"), [&] { return drawing(partPath, dir.path()); }));
    steps.push_back(guarded(QStringLiteral("samples"), [&] { return samples(shipped.samples); }));
    steps.push_back(guarded(QStringLiteral("translations"),
                            [&] { return translations(shipped.translations, shipped.catalogs); }));
    return steps;
}

}  // namespace hz::ui::selftest
