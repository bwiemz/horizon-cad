// An imported part is shown, measured and exported from its exact faces (stage 2): the window
// draws the counterpart's mesh its build made, a pick on it names the faceted part's faces and
// edges, Mass Properties integrates the exact faces, File > Export > STEP writes them as their
// kind, and a part that has no counterpart says why on the status line. With none, or once a
// feature follows the import, all is as before: the faceted part.

#include <gtest/gtest.h>

#include <QAction>
#include <QApplication>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QLabel>
#include <QMessageBox>
#include <QTemporaryDir>
#include <QTranslator>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "StepTestText.h"
#include "UiTestSupport.h"
#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/fileio/StlExport.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Units.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/render/MeshPicker.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/ViewportWidget.h"

using hz::doc::Document;
using hz::math::Vec3;
using hz::test::DialogResponder;
using hz::test::FilePicker;
using hz::test::FormAnswers;
using hz::test::FormFiller;
using hz::ui::MainWindow;

namespace {

constexpr double kPi = hz::math::kPi;
constexpr double kRadius = 10.0;
constexpr double kHeight = 20.0;

/// The cylinder other systems write (StepTestText.h), read in centimetres: radius 10 mm, height
/// 20 mm, about +z from the origin; its side, two caps, a seam.
std::unique_ptr<hz::topo::Solid> cylinder() {
    hz::io::StepReadOptions inCentimetres;
    inCentimetres.unknownLengthUnit = hz::math::LengthUnit::Centimetre;
    auto solids = hz::io::StepFormat::fromString(hz::step_text::externalStyleCylinder(), nullptr,
                                                 nullptr, inCentimetres);
    EXPECT_EQ(solids.size(), 1u) << hz::io::StepFormat::lastError();
    return solids.empty() ? nullptr : std::move(solids.front());
}

/// @p part made of the cylinder alone, imported.
void importCylinder(Document& part) {
    part.setType(hz::doc::DocumentType::Part);
    part.featureTree().addFeature(std::make_unique<hz::doc::ImportedBodyFeature>(
        std::shared_ptr<const hz::topo::Solid>(cylinder()), "cylinder.step"));
}

/// The cylinder written to @p path as Horizon writes STEP: in millimetres, its faces exact.
void writeCylinder(const QString& path) {
    const auto body = cylinder();
    ASSERT_NE(body, nullptr);
    ASSERT_TRUE(hz::io::StepFormat::save(path.toStdString(), {body.get()}))
        << hz::io::StepFormat::lastError();
}

void trigger(MainWindow& w, const char* name) {
    auto* action = w.findChild<QAction*>(QString::fromLatin1(name));
    ASSERT_NE(action, nullptr) << name;
    action->trigger();
}

/// Until nothing runs on a worker (a rebuild, a measurement).
void settle(MainWindow& w) {
    QElapsedTimer waited;
    waited.start();
    while (w.backgroundWorkRunning() && waited.elapsed() < 60'000) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    }
    QCoreApplication::processEvents();
}

/// File > Import > STEP of @p path into @p w: a part of its bodies, built.
void importInto(MainWindow& w, const QString& path) {
    {
        FilePicker picker(path);
        trigger(w, "import_step");
    }
    settle(w);
    ASSERT_NE(w.activeDocument(), nullptr);
    ASSERT_NE(w.activeDocument()->solid(), nullptr) << w.activeDocument()->lastBuildMessage();
}

/// A cylinder of radius 5 and height 10, made with the Cylinder command: Stable, so exact (stage
/// 3).
void cylinderPrimitive(MainWindow& w) {
    {
        FormFiller filler(QStringLiteral("Cylinder"), FormAnswers()
                                                          .number(QStringLiteral("size0"), 5.0)
                                                          .number(QStringLiteral("size1"), 10.0));
        trigger(w, "action_cylinder");
        ASSERT_TRUE(filler.seen());
    }
    settle(w);
    ASSERT_NE(w.activeDocument(), nullptr);
    ASSERT_NE(w.activeDocument()->exactSolid(), nullptr) << w.activeDocument()->exactNote();
}

/// The Mass Properties box open now, not blocking (measured on a worker).
QMessageBox* openMassBox() {
    for (QWidget* widget : QApplication::topLevelWidgets()) {
        auto* box = qobject_cast<QMessageBox*>(widget);
        // By its name, not its title: macOS drops a message box's title.
        if (box != nullptr && box->isVisible() &&
            box->objectName() == QStringLiteral("massPropertiesBox")) {
            return box;
        }
    }
    return nullptr;
}

/// The mesh the window draws the part with.
std::shared_ptr<const hz::geo::MeshData> shownMesh(MainWindow& w) {
    auto* view = w.findChild<hz::ui::ViewportWidget*>();
    EXPECT_NE(view, nullptr);
    if (view == nullptr) return nullptr;
    const auto nodes = view->sceneGraph().collectVisibleMeshNodes();
    EXPECT_EQ(nodes.size(), 1u);
    return nodes.empty() ? nullptr : nodes.front()->sharedMesh();
}

std::set<std::string> distinct(const std::vector<std::string>& tags) {
    return {tags.begin(), tags.end()};
}

std::set<std::string> faceNames(const hz::topo::Solid& solid) {
    std::set<std::string> names;
    for (const auto& face : solid.faces()) names.insert(face.topoId.tag());
    return names;
}

/// Every corner of @p mesh lies on the cylinder, within @p within: on its side, or on a cap
/// inside its rim.
void expectOnTheCylinder(const hz::geo::MeshData& mesh, double within) {
    ASSERT_FALSE(mesh.positions.empty());
    for (std::size_t i = 0; i + 2 < mesh.positions.size(); i += 3) {
        const Vec3 p(mesh.positions[i], mesh.positions[i + 1], mesh.positions[i + 2]);
        const double r = std::hypot(p.x, p.y);
        const bool onSide =
            std::abs(r - kRadius) <= within && p.z >= -within && p.z <= kHeight + within;
        const bool onCap =
            r <= kRadius + within && (std::abs(p.z) <= within || std::abs(p.z - kHeight) <= within);
        ASSERT_TRUE(onSide || onCap) << "(" << p.x << ", " << p.y << ", " << p.z << ")";
    }
}

/// A part saved before stage 2 whose body's STEP is not exact, written into @p dir: a box with one
/// corner raised off its planes, kept as it was. Opened, it is shown from its facets.
QString writeOldPart(const QTemporaryDir& dir) {
    auto box = hz::model::PrimitiveFactory::makeBox(7.0, 11.0, 13.0);
    std::string text = hz::io::StepFormat::toString({box.get()});
    const std::string corner = "(7.,11.,13.)";
    EXPECT_NE(text.find(corner), std::string::npos);
    if (text.find(corner) == std::string::npos) return {};
    text.replace(text.find(corner), corner.size(), "(7.,11.,13.2)");
    hz::io::StepReadOptions keep;
    keep.keepWhenNotExact = true;
    auto solids = hz::io::StepFormat::fromString(text, nullptr, nullptr, keep);
    EXPECT_EQ(solids.size(), 1u) << hz::io::StepFormat::lastError();
    if (solids.size() != 1) return {};
    Document old;
    old.setType(hz::doc::DocumentType::Part);
    old.featureTree().addFeature(std::make_unique<hz::doc::ImportedBodyFeature>(
        std::shared_ptr<const hz::topo::Solid>(std::move(solids.front())), "creo.step"));
    const QString path = dir.filePath(QStringLiteral("old.hzpart"));
    std::string error;
    EXPECT_TRUE(hz::io::NativeFormat::save(path.toStdString(), old, &error)) << error;
    return path;
}

/// A catalog that translates the status line's words for each kind of exact note, each into
/// itself in guillemets, and nothing else: what reaches the window through tr() shows so.
class MarkingCatalog : public QTranslator {
public:
    static QString marked(const char* english) {
        return QStringLiteral("\u00AB%1\u00BB").arg(QString::fromLatin1(english));
    }
    bool isEmpty() const override { return false; }
    QString translate(const char* /*context*/, const char* sourceText,
                      const char* /*disambiguation*/, int /*n*/) const override {
        for (const char* english : kWords) {
            if (std::string_view(sourceText) == english) return marked(english);
        }
        return {};
    }
    static constexpr const char* kWords[] = {"Shown from facets", "Exact form not checked",
                                             "Exact form disagrees with its facets"};
};

}  // namespace

// DocumentManager's mesh for the part, which an assembly draws it with, is its exact
// counterpart's: three faces, not a face for each facet, its corners on the cylinder. The part
// shown in a tab is built with its counterpart, and placed, resolved or light, it is drawn with
// that very mesh, not meshed again.
TEST(ExactDisplay, AnImportedCylinderIsMeshedFromItsExactFaces) {
    const std::string path =
        (std::filesystem::temp_directory_path() / "hz_exact_display_cylinder.hzpart").string();
    std::ofstream(path) << "{}";
    hz::doc::DocumentManager manager;
    manager.setPartLoader([](const std::string& /*path*/, Document& doc) {
        importCylinder(doc);
        return true;
    });
    const auto shown = manager.openPart(path);
    ASSERT_NE(shown, nullptr);

    hz::doc::ComponentInstance resolved;
    resolved.partPath = path;
    ASSERT_TRUE(manager.resolveComponent(resolved, hz::doc::ComponentState::Resolved));
    ASSERT_NE(shown->exactSolid(), nullptr) << shown->exactNote();
    ASSERT_NE(resolved.cachedMesh, nullptr);
    const hz::geo::MeshData& mesh = *resolved.cachedMesh;
    ASSERT_TRUE(mesh.hasFaces());
    EXPECT_EQ(mesh.faceTags.size(), 3u);
    EXPECT_EQ(distinct(mesh.faceTags), faceNames(*shown->exactSolid()));
    const double diagonal = std::sqrt(4.0 * kRadius * kRadius + kHeight * kHeight);
    expectOnTheCylinder(mesh, 1e-4 * diagonal);
    EXPECT_EQ(resolved.cachedMesh, shown->exactMesh()) << "the mesh its build made";

    hz::doc::ComponentInstance light;
    light.partPath = path;
    ASSERT_TRUE(manager.resolveComponent(light, hz::doc::ComponentState::Lightweight));
    ASSERT_NE(light.cachedMesh, nullptr);
    EXPECT_EQ(light.cachedMesh->faceTags.size(), 3u);
    EXPECT_EQ(light.cachedMesh, shown->exactMesh());
    std::filesystem::remove(path);
}

// A part file keeps the mesh the part is shown with, for an assembly that draws it without
// building it: the counterpart's, its three faces by name.
TEST(ExactDisplay, APartFileKeepsTheMeshItIsShownWith) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    Document part;
    importCylinder(part);
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    ASSERT_NE(part.exactMesh(), nullptr) << part.exactNote();
    const std::string path = dir.filePath(QStringLiteral("cylinder.hzpart")).toStdString();
    std::string error;
    ASSERT_TRUE(hz::io::NativeFormat::save(path, part, &error)) << error;
    const auto kept = hz::io::NativeFormat::loadPartMesh(path);
    ASSERT_NE(kept, nullptr);
    EXPECT_EQ(kept->faceTags, part.exactMesh()->faceTags);
    EXPECT_EQ(kept->triangleFaces, part.exactMesh()->triangleFaces);
    EXPECT_EQ(kept->edges.size(), part.exactMesh()->edges.size());
}

// The window draws the kept mesh: the part's counterpart's, made once by its build, never
// tessellated again for the view.
TEST(ExactDisplay, TheWindowDrawsTheMeshItsBuildMade) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString step = dir.filePath(QStringLiteral("cylinder.step"));
    writeCylinder(step);
    MainWindow w;
    importInto(w, step);
    const Document& part = *w.activeDocument();
    ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
    const auto mesh = shownMesh(w);
    ASSERT_NE(mesh, nullptr);
    EXPECT_EQ(distinct(mesh->faceTags), faceNames(*part.exactSolid()));
    EXPECT_EQ(mesh, part.exactMesh()) << "the very mesh its build made";
    EXPECT_EQ(w.tessellations(), 0u) << "drawn as its build meshed it";
}

// A click on the exact mesh names what a click on the faceted part named: the side's tag is the
// exact face's, and through logicalFace it names the side's facets in solid(), which features
// work on. Its edges likewise; and its seam, whose name "<side>/seam:0" is also the name of a
// line between two of the faceted side's facets, is not drawn, so no click resolves to that line.
TEST(ExactDisplay, APickOnTheExactMeshNamesTheFacetedFace) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString step = dir.filePath(QStringLiteral("cylinder.step"));
    writeCylinder(step);
    MainWindow w;
    importInto(w, step);
    const Document& part = *w.activeDocument();
    ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
    const hz::topo::Solid& faceted = *part.solid();
    const hz::topo::Solid& exact = *part.exactSolid();
    const auto mesh = shownMesh(w);
    ASSERT_NE(mesh, nullptr);

    // A click from +x, halfway up: the side.
    const auto hit = hz::render::MeshPicker::pickFace(
        *mesh, hz::math::Mat4::identity(), Vec3(5 * kRadius, 0.0, kHeight / 2), Vec3(-1, 0, 0));
    ASSERT_TRUE(hit.has_value());
    ASSERT_GE(hit->face, 0);
    const std::string tag = mesh->faceTags[static_cast<std::size_t>(hit->face)];
    EXPECT_EQ(faceNames(exact).count(tag), 1u) << tag << " is a face of the exact counterpart";
    const std::string picked = hz::model::logicalFace(tag);
    std::size_t facets = 0;
    for (const auto& face : faceted.faces()) {
        if (hz::model::logicalFace(face.topoId.tag()) != picked) continue;
        ++facets;
        const hz::topo::HalfEdge* he = face.outerLoop->halfEdge;
        EXPECT_NEAR(std::hypot(he->origin->point.x, he->origin->point.y), kRadius, 1e-6)
            << face.topoId.tag() << " is not on the side";
    }
    EXPECT_GT(facets, 1u) << picked << " names the side's facets";

    // The hazard is real: the exact seam's name is an inter-facet line's in the faceted part.
    const auto logicalFacesOf = [](const hz::topo::Edge& edge) {
        const hz::topo::HalfEdge* he = edge.halfEdge;
        return std::pair{hz::model::logicalFace(he->face->topoId.tag()),
                         hz::model::logicalFace(he->twin->face->topoId.tag())};
    };
    std::string seam;
    for (const auto& edge : exact.edges()) {
        if (edge.halfEdge->face == edge.halfEdge->twin->face) seam = edge.topoId.tag();
    }
    ASSERT_FALSE(seam.empty());
    bool sharedName = false;
    for (const auto& edge : faceted.edges()) {
        if (edge.topoId.tag() != seam) continue;
        const auto [a, b] = logicalFacesOf(edge);
        sharedName = sharedName || a == b;
    }
    EXPECT_TRUE(sharedName) << seam << " also names a line between two facets of the side";

    // Every edge drawn, as a click names it, is an edge of the faceted part between two faces.
    ASSERT_FALSE(mesh->edges.empty());
    for (const auto& drawn : mesh->edges) {
        const std::string name = hz::model::logicalEdge(drawn.tag);
        std::size_t found = 0;
        for (const auto& edge : faceted.edges()) {
            if (hz::model::logicalEdge(edge.topoId.tag()) != name) continue;
            ++found;
            const auto [a, b] = logicalFacesOf(edge);
            EXPECT_NE(a, b) << name << " resolves to a line inside the face " << a;
        }
        EXPECT_GT(found, 0u) << name << " names no edge of the part";
    }
}

// File > Export > STEP writes the exact counterpart: the side as the CYLINDRICAL_SURFACE it was
// read as, not its facets, and read back it is the cylinder to 1e-9.
TEST(ExactDisplay, ExportWritesTheExactCounterpart) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString step = dir.filePath(QStringLiteral("cylinder.step"));
    writeCylinder(step);
    MainWindow w;
    importInto(w, step);
    ASSERT_NE(w.activeDocument()->exactSolid(), nullptr) << w.activeDocument()->exactNote();

    const QString out = dir.filePath(QStringLiteral("out.step"));
    {
        FilePicker picker(out);
        trigger(w, "export_step");
    }
    QFile file(out);
    ASSERT_TRUE(file.open(QIODevice::ReadOnly));
    const std::string text = file.readAll().toStdString();
    EXPECT_NE(text.find("CYLINDRICAL_SURFACE"), std::string::npos);
    EXPECT_EQ(text.find("B_SPLINE_SURFACE"), std::string::npos) << "no face in facets";
    const auto back = hz::io::StepFormat::load(out.toStdString());
    ASSERT_EQ(back.size(), 1u) << hz::io::StepFormat::lastError();
    const double volume = kPi * kRadius * kRadius * kHeight;
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*back.front()).volume, volume,
                1e-9 * volume);
}

// Mass Properties measures the exact faces: the cylinder's volume, said as exact, not the
// facets' as modelled beside an ideal.
TEST(ExactDisplay, MassPropertiesMeasuresTheExactFaces) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString step = dir.filePath(QStringLiteral("cylinder.step"));
    writeCylinder(step);
    MainWindow w;
    importInto(w, step);
    ASSERT_NE(w.activeDocument()->exactSolid(), nullptr) << w.activeDocument()->exactNote();

    DialogResponder answer(QMessageBox::Ok, QStringLiteral("Mass Properties"));
    FormFiller filler(
        QStringLiteral("Mass Properties"),
        FormAnswers().choose(QStringLiteral("material"), QStringLiteral("None (volume only)")));
    trigger(w, "action_mass_properties");
    ASSERT_TRUE(filler.seen());
    ASSERT_TRUE(answer.seen());
    const std::string text = answer.text().toStdString();
    EXPECT_NE(text.find("Exact"), std::string::npos) << text;
    EXPECT_NE(text.find("Volume: 6283.185 "), std::string::npos) << "2000 pi: " << text;
    EXPECT_NE(text.find("Surface area: 1884.956 "), std::string::npos) << "600 pi: " << text;
    EXPECT_EQ(text.find("As modelled"), std::string::npos) << text;
    EXPECT_EQ(text.find("Exact to"), std::string::npos) << "a clean file: " << text;
}

// M8 (goal 6): a part exact only to the gaps its file gives its edges is said to be: Mass
// Properties gives how far its edges stand off its faces, and the integration's estimate of its
// error, which counts them, never a bare "Exact". A 4 mm block whose corner stands 1e-4 off its
// three faces' planes, read as exact to that.
TEST(ExactDisplay, MassPropertiesSaysWhatATolerantPartIsExactTo) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    std::string text = hz::step_text::block(false);
    const std::string corner = "CARTESIAN_POINT('',(4.,4.,4.))";
    ASSERT_NE(text.find(corner), std::string::npos);
    text.replace(text.find(corner), corner.size(), "CARTESIAN_POINT('',(4.,4.,4.0001))");
    hz::io::StepReadOptions inMillimetres;
    inMillimetres.unknownLengthUnit = hz::math::LengthUnit::Millimetre;
    const auto read = hz::io::StepFormat::fromString(text, nullptr, nullptr, inMillimetres);
    ASSERT_EQ(read.size(), 1u) << hz::io::StepFormat::lastError();
    const QString step = dir.filePath(QStringLiteral("block.step"));
    ASSERT_TRUE(hz::io::StepFormat::save(step.toStdString(), {read.front().get()}));
    MainWindow w;
    importInto(w, step);
    ASSERT_NE(w.activeDocument()->exactSolid(), nullptr) << w.activeDocument()->exactNote();

    DialogResponder answer(QMessageBox::Ok, QStringLiteral("Mass Properties"));
    FormFiller filler(
        QStringLiteral("Mass Properties"),
        FormAnswers().choose(QStringLiteral("material"), QStringLiteral("None (volume only)")));
    trigger(w, "action_mass_properties");
    ASSERT_TRUE(filler.seen());
    ASSERT_TRUE(answer.seen());
    const std::string said = answer.text().toStdString();
    EXPECT_NE(said.find("Exact to 0.0001 mm, as its file gives its edges"), std::string::npos)
        << said;
    EXPECT_NE(said.find("its volume and area to within "), std::string::npos) << said;
    EXPECT_EQ(said.find("Exact (integrated"), std::string::npos) << said;
}

// A feature after the import drops the counterpart: the window draws the faceted part, meshed
// for the view, as it always has. Its undo brings the counterpart's mesh back.
TEST(ExactDisplay, AFeatureAfterTheImportDrawsTheFacetsAndItsUndoTheExactFaces) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString step = dir.filePath(QStringLiteral("cylinder.step"));
    writeCylinder(step);
    MainWindow w;
    importInto(w, step);
    Document& part = *w.activeDocument();
    ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
    const auto exactShown = shownMesh(w);
    ASSERT_NE(exactShown, nullptr);
    EXPECT_EQ(exactShown->faceTags.size(), 3u);

    {
        FormFiller filler(QStringLiteral("Box"), FormAnswers()
                                                     .number(QStringLiteral("size0"), 5.0)
                                                     .number(QStringLiteral("size1"), 5.0)
                                                     .number(QStringLiteral("size2"), 5.0)
                                                     .number(QStringLiteral("atX"), 30.0));
        trigger(w, "action_box");
        ASSERT_TRUE(filler.seen());
    }
    settle(w);
    EXPECT_EQ(part.exactSolid(), nullptr);
    EXPECT_TRUE(part.exactNote().empty()) << "none, by design: " << part.exactNote();
    const auto facetsShown = shownMesh(w);
    ASSERT_NE(facetsShown, nullptr);
    EXPECT_GT(facetsShown->faceTags.size(), 3u + 6u) << "the side in facets, and the box";
    const auto tessellated = w.tessellations();
    EXPECT_GT(tessellated, 0u) << "the faceted part, meshed for the view";

    trigger(w, "action_undo");
    settle(w);
    ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
    const auto again = shownMesh(w);
    ASSERT_NE(again, nullptr);
    EXPECT_EQ(distinct(again->faceTags), faceNames(*part.exactSolid()));
    EXPECT_EQ(again, part.exactMesh());
    EXPECT_EQ(w.tessellations(), tessellated) << "the counterpart's mesh, not meshed again";
}

// A part whose imports would give it a counterpart, but cannot, says why on the status line; the
// note goes with its part. Here a part saved before stage 2 whose body's STEP is not exact.
// (Re-baselined for Task 10: the note is a kind and its detail; the tooltip is the detail, which
// no longer begins with the kind's words.)
TEST(ExactDisplay, WhyAPartIsShownFromFacetsIsOnTheStatusLine) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString oldPath = writeOldPart(dir);
    ASSERT_FALSE(oldPath.isEmpty());

    MainWindow w;
    auto* note = w.findChild<QLabel*>(QStringLiteral("exactNote"));
    ASSERT_NE(note, nullptr);
    EXPECT_TRUE(note->isHidden()) << "a drawing has none";
    ASSERT_TRUE(w.openPath(oldPath));
    settle(w);
    ASSERT_NE(w.activeDocument()->solid(), nullptr);
    const std::string why = w.activeDocument()->exactNote();
    ASSERT_EQ(w.activeDocument()->exactNoteKind(), hz::doc::ExactNoteKind::ShownFromFacets) << why;
    ASSERT_FALSE(why.empty());
    EXPECT_EQ(why.find("shown from facets"), std::string::npos) << "the detail alone: " << why;
    EXPECT_FALSE(note->isHidden());
    EXPECT_EQ(note->text(), QStringLiteral("Shown from facets"));
    EXPECT_EQ(note->toolTip().toStdString(), why);

    // Another part, with its counterpart: no note.
    const QString step = dir.filePath(QStringLiteral("cylinder.step"));
    writeCylinder(step);
    importInto(w, step);
    ASSERT_NE(w.activeDocument()->exactSolid(), nullptr) << w.activeDocument()->exactNote();
    EXPECT_TRUE(note->isHidden());
}

// Task 10: the status line says the kind of note in the window's own words, through tr(), for
// each kind; nothing it shows is English built in the document. Its tooltip is the note's detail,
// the technical reason. Under a catalog that translates those words, the line shows them
// translated.
TEST(ExactDisplay, TheNoteIsTranslatable) {
    MarkingCatalog catalog;
    ASSERT_TRUE(QCoreApplication::installTranslator(&catalog));
    struct Uninstalled {
        QTranslator* catalog;
        ~Uninstalled() { QCoreApplication::removeTranslator(catalog); }
    } const uninstalled{&catalog};

    using hz::doc::ExactNoteKind;
    const std::vector<std::pair<ExactNoteKind, const char*>> kinds = {
        {ExactNoteKind::ShownFromFacets, "Shown from facets"},
        {ExactNoteKind::NotChecked, "Exact form not checked"},
        {ExactNoteKind::Disagrees, "Exact form disagrees with its facets"},
    };
    for (const auto& kind : kinds) {
        EXPECT_EQ(MainWindow::exactNoteText(kind.first), MarkingCatalog::marked(kind.second))
            << kind.second;
    }
    EXPECT_TRUE(MainWindow::exactNoteText(ExactNoteKind::None).isEmpty());

    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString oldPath = writeOldPart(dir);
    ASSERT_FALSE(oldPath.isEmpty());
    MainWindow w;
    auto* note = w.findChild<QLabel*>(QStringLiteral("exactNote"));
    ASSERT_NE(note, nullptr);
    ASSERT_TRUE(w.openPath(oldPath));
    settle(w);
    ASSERT_EQ(w.activeDocument()->exactNoteKind(), ExactNoteKind::ShownFromFacets);
    EXPECT_FALSE(note->isHidden());
    EXPECT_EQ(note->text(), MarkingCatalog::marked("Shown from facets"));
    EXPECT_EQ(note->toolTip().toStdString(), w.activeDocument()->exactNote());
}

// Task 10 (the Stable path, the default since stage 3): a cylinder made with the Cylinder command
// is exact, and Mass Properties integrates its exact faces, here on the window's thread: 25 pi 10
// and 150 pi, said as exact, and as integrated on its exact faces, not "as they were imported".
TEST(ExactDisplay, MassPropertiesOfAPrimitiveIsExact) {
    MainWindow w;
    w.setRebuildMode(MainWindow::RebuildMode::Never);
    cylinderPrimitive(w);

    DialogResponder answer(QMessageBox::Ok, QStringLiteral("Mass Properties"));
    FormFiller filler(
        QStringLiteral("Mass Properties"),
        FormAnswers().choose(QStringLiteral("material"), QStringLiteral("None (volume only)")));
    trigger(w, "action_mass_properties");
    ASSERT_TRUE(filler.seen());
    ASSERT_TRUE(answer.seen());
    const std::string text = answer.text().toStdString();
    EXPECT_NE(text.find("Exact (integrated on its exact faces):"), std::string::npos) << text;
    EXPECT_NE(text.find("Volume: 785.3982 "), std::string::npos) << "25 pi 10: " << text;
    EXPECT_NE(text.find("Surface area: 471.2389 "), std::string::npos) << "150 pi: " << text;
    EXPECT_EQ(text.find("imported"), std::string::npos) << "a primitive, built: " << text;
    EXPECT_EQ(text.find("As modelled"), std::string::npos) << text;
}

// Task 10: an exact part is measured on a worker under the faceted path's rule (Always; Auto with
// kWorkerIdealFaces or more curved faces): the box opens at once, not blocking, and the exact
// measure comes into it through the task.
TEST(ExactDisplay, ExactMassRunsOnAWorkerWhenAsked) {
    MainWindow w;
    w.setRebuildMode(MainWindow::RebuildMode::Always);
    cylinderPrimitive(w);

    FormFiller filler(QStringLiteral("Mass Properties"),
                      FormAnswers().choose(QStringLiteral("material"), QStringLiteral("Steel")));
    trigger(w, "action_mass_properties");
    QMessageBox* box = openMassBox();
    ASSERT_NE(box, nullptr) << "open while it is measured";
    EXPECT_TRUE(box->text().contains(QStringLiteral("Measuring..."))) << box->text().toStdString();
    EXPECT_FALSE(box->text().contains(QStringLiteral("Volume:"))) << box->text().toStdString();
    EXPECT_TRUE(w.backgroundWorkRunning());
    settle(w);
    ASSERT_FALSE(w.backgroundWorkRunning());
    const std::string text = box->text().toStdString();
    EXPECT_NE(text.find("Exact (integrated on its exact faces):"), std::string::npos) << text;
    EXPECT_NE(text.find("Volume: 785.3982 "), std::string::npos) << text;
    EXPECT_NE(text.find("Mass: 6.165376 g"), std::string::npos) << "7.85 g/cm3: " << text;
    EXPECT_NE(text.find("density 7.85 g/cm"), std::string::npos) << text;
    EXPECT_EQ(text.find("Measuring..."), std::string::npos) << text;
    box->accept();
}

// Task 10 (Task 13's floor note): a part whose estimate stops short of the tolerance asked for,
// with every face taken as far as the integration goes, is said to be exact to within it, not
// plainly "Exact", even where the estimate is under a billionth: a coin a thousandth as thick as
// it is wide, its axis along (1, 1, 0), 1e-3 by 10 in radius.
TEST(ExactDisplay, MassPropertiesSaysWhereAThinPartsEstimateStops) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    Document coin;
    coin.setType(hz::doc::DocumentType::Part);
    auto cylinder = hz::doc::PrimitiveFeature::makeCylinder(10.0, 1e-3);
    ASSERT_TRUE(cylinder->setVector("axisDirection", Vec3(1, 1, 0)));
    coin.featureTree().addFeature(std::move(cylinder));
    const QString path = dir.filePath(QStringLiteral("coin.hzpart"));
    std::string error;
    ASSERT_TRUE(hz::io::NativeFormat::save(path.toStdString(), coin, &error)) << error;

    MainWindow w;
    w.setRebuildMode(MainWindow::RebuildMode::Never);
    ASSERT_TRUE(w.openPath(path));
    settle(w);
    ASSERT_NE(w.activeDocument()->exactSolid(), nullptr) << w.activeDocument()->exactNote();
    const auto measured =
        hz::model::MassPropertiesCalculator::computeIdeal(*w.activeDocument()->exactSolid());
    ASSERT_FALSE(measured.note.empty()) << "its estimate stops short: " << measured.estimatedError;

    DialogResponder answer(QMessageBox::Ok, QStringLiteral("Mass Properties"));
    FormFiller filler(
        QStringLiteral("Mass Properties"),
        FormAnswers().choose(QStringLiteral("material"), QStringLiteral("None (volume only)")));
    trigger(w, "action_mass_properties");
    ASSERT_TRUE(answer.seen());
    const std::string text = answer.text().toStdString();
    EXPECT_NE(text.find("Exact (integrated on its exact faces; its volume and area to within "),
              std::string::npos)
        << text;
    EXPECT_NE(text.find("Volume: 0.3141593 "), std::string::npos) << "pi 100 1e-3: " << text;
}

// Task 10: File > Export > STL and glTF write the mesh the part's build kept (Document::exactMesh)
// where it was made at the export's tolerance (the part's display tolerance,
// SolidTessellator::displayTolerance, compared exactly; STL's is 0.1, a 5 mm cylinder's): not
// meshed again, so the window's count of tessellations does not move, and the STL's triangles are
// the kept mesh's. A part with no exact counterpart is meshed for the export, and counted.
TEST(ExactDisplay, StlExportReusesTheKeptMesh) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    MainWindow w;
    cylinderPrimitive(w);
    Document& part = *w.activeDocument();
    ASSERT_NE(part.exactMesh(), nullptr);
    ASSERT_EQ(hz::model::SolidTessellator::displayTolerance(*part.exactSolid()),
              hz::io::StlExport::kTolerance);
    const auto before = w.tessellations();

    const QString stl = dir.filePath(QStringLiteral("part.stl"));
    {
        FilePicker picker(stl);
        trigger(w, "export_stl");
    }
    EXPECT_EQ(w.tessellations(), before) << "the kept mesh, not meshed again";
    QFile file(stl);
    ASSERT_TRUE(file.open(QIODevice::ReadOnly));
    const QByteArray bytes = file.readAll();
    ASSERT_GE(bytes.size(), 84);
    std::uint32_t triangles = 0;
    std::memcpy(&triangles, bytes.constData() + 80, sizeof triangles);
    EXPECT_EQ(triangles, part.exactMesh()->indices.size() / 3);
    EXPECT_EQ(static_cast<std::size_t>(bytes.size()), 84 + 50 * std::size_t{triangles});

    const QString glb = dir.filePath(QStringLiteral("part.glb"));
    {
        FilePicker picker(glb);
        trigger(w, "export_gltf");
    }
    EXPECT_GT(QFileInfo(glb).size(), 0);
    EXPECT_EQ(w.tessellations(), before) << "glTF writes it too";

    // Named by position, as a part saved before persistent naming: no exact counterpart, so its
    // facets are meshed for the export, and counted.
    part.featureTree().feature(0)->setNaming(hz::model::NamingScheme::Positional);
    part.featureTree().markChanged();
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    ASSERT_EQ(part.exactSolid(), nullptr);
    const auto faceted = w.tessellations();
    {
        FilePicker picker(dir.filePath(QStringLiteral("faceted.stl")));
        trigger(w, "export_stl");
    }
    EXPECT_EQ(w.tessellations(), faceted + 1) << "meshed for the export";
    EXPECT_TRUE(QFileInfo::exists(dir.filePath(QStringLiteral("faceted.stl"))));
}

// Owner amendment A1 (Task 10): on an exact part, a chosen or hovered face is highlighted from its
// faceted copy (Document::solid()), every face of it whose logical name is the pick's, not from
// the exact mesh drawn, so a coarse part shows the facets a command on the face acts on. A
// cylinder primitive cut in 6 round its axis: its side is drawn round, and highlighted as the
// faceted copy's 6 flat facets, two triangles each, every corner one of theirs, filled where the
// round side drawn is seen (its mask, a stencil: Ruling 39), as they lie inside it.
TEST(ExactDisplay, AFaceHighlightIsDrawnFromTheFacetedCopy) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    Document coarse;
    coarse.setType(hz::doc::DocumentType::Part);
    auto made = hz::doc::PrimitiveFeature::makeCylinder(kRadius, kHeight);
    ASSERT_TRUE(made->setParameter("segments", 6.0));
    coarse.featureTree().addFeature(std::move(made));
    const QString path = dir.filePath(QStringLiteral("coarse.hzpart"));
    std::string error;
    ASSERT_TRUE(hz::io::NativeFormat::save(path.toStdString(), coarse, &error)) << error;

    MainWindow w;
    ASSERT_TRUE(w.openPath(path));
    settle(w);
    const Document& part = *w.activeDocument();
    ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
    const auto mesh = shownMesh(w);
    ASSERT_NE(mesh, nullptr);
    ASSERT_EQ(mesh, part.exactMesh()) << "drawn round";

    // A click from +x, halfway up: the side.
    const auto hit = hz::render::MeshPicker::pickFace(
        *mesh, hz::math::Mat4::identity(), Vec3(5 * kRadius, 0.0, kHeight / 2), Vec3(-1, 0, 0));
    ASSERT_TRUE(hit.has_value());
    ASSERT_GE(hit->face, 0);
    const std::string side =
        hz::model::logicalFace(mesh->faceTags[static_cast<std::size_t>(hit->face)]);
    std::size_t drawn = 0;
    for (const std::uint32_t face : mesh->triangleFaces) {
        drawn += hz::model::logicalFace(mesh->faceTags[face]) == side ? 1 : 0;
    }
    EXPECT_GT(drawn, 12u) << "the side drawn round, in more triangles than its facets'";

    // The faceted copy's side: 6 facets, and their corners.
    std::set<std::string> facets;
    std::vector<Vec3> corners;
    for (const auto& face : part.solid()->faces()) {
        if (!face.alive || hz::model::logicalFace(face.topoId.tag()) != side) continue;
        facets.insert(face.topoId.tag());
        const hz::topo::HalfEdge* he = face.outerLoop->halfEdge;
        do {
            corners.push_back(he->origin->point);
            he = he->next;
        } while (he != face.outerLoop->halfEdge);
    }
    ASSERT_EQ(facets.size(), 6u);

    auto* view = w.findChild<hz::ui::ViewportWidget*>();
    ASSERT_NE(view, nullptr);
    const auto highlight = view->faceHighlight({0, side, false});
    EXPECT_EQ(distinct(highlight.faces), facets);
    ASSERT_EQ(highlight.triangles.size(), 6u * 2u * 9u) << "two triangles a facet";
    for (std::size_t i = 0; i + 2 < highlight.triangles.size(); i += 3) {
        const Vec3 p(highlight.triangles[i], highlight.triangles[i + 1],
                     highlight.triangles[i + 2]);
        const bool atAFacetsCorner =
            std::any_of(corners.begin(), corners.end(),
                        [&p](const Vec3& c) { return (c - p).length() < 1e-4; });
        EXPECT_TRUE(atAFacetsCorner) << "(" << p.x << ", " << p.y << ", " << p.z << ")";
    }
    // Filled where the side drawn is seen (Ruling 39): its mask is the round side as drawn, the
    // exact mesh's triangles of it, every corner on the cylinder.
    ASSERT_EQ(highlight.mask.size(), drawn * 9u);
    for (std::size_t i = 0; i + 2 < highlight.mask.size(); i += 3) {
        EXPECT_NEAR(std::hypot(highlight.mask[i], highlight.mask[i + 1]), kRadius, 1e-4);
    }
}
