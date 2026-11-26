// Drawings from the exact part (stage 3): a sheet of a part that has an exact part
// (Document::exactSolid) draws it from its exact faces, cut into facets so finely that no chord
// sags more than 0.01 mm on paper at the sheet's largest scale (model::facetExact at 0.01 / scale
// mm, its edges named logically), where it was drawn from the faceted part, whose 32 sides put a
// cylinder's outline up to a quarter of a millimetre inside on a 1:1 sheet of a 50 mm radius. A
// part that has none (a cut makes it faceted) is drawn as before. Dimensions find their edges on
// the copy by their logical names; an import beside a primitive is cut as its own bridge cuts it
// (Ruling 17).

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "StepTestText.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DimensionStyle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/fileio/DrawingDocumentIO.h"
#include "horizon/fileio/DrawingExport.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/math/Units.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/Solid.h"

using hz::doc::Document;
using hz::doc::PrimitiveFeature;
using hz::io::DrawingDocumentIO;
using hz::io::DrawingDocumentSpec;
using hz::io::DrawingViewSpec;
using hz::math::Vec2;
using hz::math::Vec3;
using hz::model::Drawing;
using hz::model::DrawingView;
using hz::model::ProjectedEdge;
using hz::model::StandardView;

namespace {

constexpr double kOnPaper = 0.01;  ///< the most a chord may sag on paper, in millimetres

/// A file in a folder of its own under the temporary one, gone with it.
class TempFolder {
public:
    explicit TempFolder(const std::string& name)
        : m_path(std::filesystem::temp_directory_path() / ("hz_drawing_exact_" + name)) {
        std::filesystem::remove_all(m_path);
        std::filesystem::create_directories(m_path);
    }
    ~TempFolder() {
        std::error_code ignored;
        std::filesystem::remove_all(m_path, ignored);
    }
    TempFolder(const TempFolder&) = delete;
    TempFolder& operator=(const TempFolder&) = delete;
    std::string file(const std::string& name) const { return (m_path / name).string(); }

private:
    std::filesystem::path m_path;
};

/// A cylinder primitive of radius 50 and height 40 standing on the origin along +z, its
/// faceted copy @p segments round (45: its outline at -x falls in the middle of a facet).
std::unique_ptr<PrimitiveFeature> cylinder(int segments = 45) {
    auto feature = PrimitiveFeature::makeCylinder(50.0, 40.0);
    feature->restoreFeatureID("primitive_1");
    EXPECT_TRUE(feature->setParameter("segments", segments));
    return feature;
}

/// A part of @p features, built, saved at @p path.
void savePart(const std::string& path, Document& part) {
    part.setType(hz::doc::DocumentType::Part);
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    ASSERT_TRUE(hz::io::NativeFormat::save(path, part));
}

/// The sheet @p spec describes, of the part at @p partPath, saved beside it and read back as a
/// drawing document is opened (DrawingDocumentIO::load).
Drawing loadSheet(const TempFolder& folder, const std::string& partPath, DrawingDocumentSpec spec) {
    spec.partPath = partPath;
    const std::string sheet = folder.file("sheet.hzdwg");
    EXPECT_TRUE(DrawingDocumentIO::save(sheet, spec));
    DrawingDocumentSpec read;
    Drawing drawing;
    std::string error;
    EXPECT_TRUE(DrawingDocumentIO::load(sheet, read, drawing, &error)) << error;
    return drawing;
}

/// A sheet laid out anew, its four standard views at @p scale (0: the largest that fits).
DrawingDocumentSpec standardSheet(double scale) {
    DrawingDocumentSpec spec;
    spec.scale = scale;
    return spec;
}

const DrawingView& viewOf(const Drawing& drawing, StandardView kind) {
    for (const DrawingView& v : drawing.views) {
        if (v.role == hz::model::ViewRole::Projection && v.kind == kind) return v;
    }
    ADD_FAILURE() << "no view " << static_cast<int>(kind);
    return drawing.views.front();
}

/// How far inside |x| = @p radius, about view-space x = @p centre, the silhouette edges of
/// @p view that lie within @p near of it reach at most (their ends'), and how many there are.
std::pair<double, int> silhouetteInside(const DrawingView& view, double centre, double radius,
                                        double near = 1.0) {
    double worst = 0.0;
    int count = 0;
    for (const ProjectedEdge& e : view.edges) {
        if (e.kind != ProjectedEdge::Kind::Silhouette) continue;
        for (const Vec2& p : {e.a, e.b}) {
            const double inside = radius - std::abs(p.x - centre);
            if (std::abs(inside) > near) continue;
            worst = std::max(worst, inside);
            ++count;
        }
    }
    return {worst, count};
}

/// How far off the circle of @p radius about the view's origin any end of @p view's edges is.
double offTheCircle(const DrawingView& view, double radius) {
    double worst = 0.0;
    for (const ProjectedEdge& e : view.edges) {
        for (const Vec2& p : {e.a, e.b}) {
            worst = std::max(worst, std::abs(std::hypot(p.x, p.y) - radius));
        }
    }
    return worst;
}

/// How many edges of each kind and visibility each view of @p drawing has, view by view.
using Kinds = std::map<std::pair<int, int>, int>;
std::vector<Kinds> kindsOf(const Drawing& drawing) {
    std::vector<Kinds> out;
    for (const DrawingView& v : drawing.views) {
        Kinds kinds;
        for (const ProjectedEdge& e : v.edges) {
            ++kinds[{static_cast<int>(e.kind), static_cast<int>(e.visibility)}];
        }
        out.push_back(std::move(kinds));
    }
    return out;
}

}  // namespace

// A cylinder of radius 50 on a 1:1 sheet. Drawn from its faceted part, 45 sides round, its outline
// at -x lay in the middle of a facet, 0.12 mm inside; drawn from its exact faces, both sides of
// the front view's outline lie within 0.01 mm of x = +-50, and seen from above every end of the
// rims' edges within 0.01 mm of its circle.
TEST(DrawingExact, ACylindersSilhouetteIsSmooth) {
    const TempFolder folder("silhouette");
    Document part;
    part.featureTree().addFeature(cylinder());
    savePart(folder.file("cylinder.hzpart"), part);
    ASSERT_NE(part.exactSolid(), nullptr);

    // Master's: drawn from the faceted part.
    const Drawing faceted = hz::model::DrawingGenerator::sheetLayout(
        *part.solid(), hz::model::Sheet{}, hz::model::TitleBlock{}, 10.0, nullptr, 1.0);
    const double before = silhouetteInside(viewOf(faceted, StandardView::Front), 0.0, 50.0).first;
    EXPECT_NEAR(before, 50.0 * (1.0 - std::cos(hz::math::kPi / 45.0)), 1e-9) << "0.12 inside";

    const Drawing drawing = loadSheet(folder, folder.file("cylinder.hzpart"), standardSheet(1.0));
    ASSERT_EQ(drawing.views.size(), 4u);
    const DrawingView& front = viewOf(drawing, StandardView::Front);
    EXPECT_DOUBLE_EQ(front.scale, 1.0);
    const auto [inside, ends] = silhouetteInside(front, 0.0, 50.0);
    EXPECT_EQ(ends, 4) << "an outline on each side, from rim to rim";
    EXPECT_LE(inside, kOnPaper);
    EXPECT_GE(inside, -1e-9);
    EXPECT_LE(offTheCircle(viewOf(drawing, StandardView::Top), 50.0), kOnPaper);
}

// The tolerance is 0.01 mm on paper at the sheet's largest scale: at 2:1 the copy is cut twice as
// finely, and every outline of the front, right and isometric views lies within 0.005 mm of the
// cylinder's (0.01 mm on paper), where at 1:1 the right view's lay 0.0099 mm inside.
TEST(DrawingExact, ATwoToOneViewHalvesTheTolerance) {
    const TempFolder folder("twotoone");
    Document part;
    part.featureTree().addFeature(cylinder());
    savePart(folder.file("cylinder.hzpart"), part);

    const Drawing atOne = loadSheet(folder, folder.file("cylinder.hzpart"), standardSheet(1.0));
    const double rightAtOne = silhouetteInside(viewOf(atOne, StandardView::Right), 0.0, 50.0).first;
    EXPECT_GT(rightAtOne, kOnPaper / 2.0);
    EXPECT_LE(rightAtOne, kOnPaper);

    // Its views as saved, at 2:1: the sheet's largest scale.
    DrawingDocumentSpec spec;
    spec.views = DrawingDocumentIO::viewsOf(atOne);
    for (DrawingViewSpec& v : spec.views) v.scale = 2.0;
    const Drawing atTwo = loadSheet(folder, folder.file("cylinder.hzpart"), spec);
    ASSERT_EQ(atTwo.views.size(), 4u);
    for (const StandardView kind :
         {StandardView::Front, StandardView::Right, StandardView::Isometric}) {
        SCOPED_TRACE(static_cast<int>(kind));
        const DrawingView& view = viewOf(atTwo, kind);
        EXPECT_DOUBLE_EQ(view.scale, 2.0);
        const auto [inside, ends] = silhouetteInside(view, 0.0, 50.0);
        EXPECT_EQ(ends, 4);
        EXPECT_LE(inside * view.scale, kOnPaper);
        EXPECT_GE(inside, -1e-9);
    }
    EXPECT_LE(offTheCircle(viewOf(atTwo, StandardView::Top), 50.0) * 2.0, kOnPaper);
}

// A part with a cut has no exact part: it is drawn from its faceted part, as before, each view
// with the edges of each kind master drew (recorded before this change, at d89f6422, whose drawing
// code and faceted part are master's, 25b1947b).
TEST(DrawingExact, AFacetedPartIsDrawnAsBefore) {
    const TempFolder folder("faceted");
    Document part;
    part.featureTree().addFeature(cylinder(32));
    auto sketch = std::make_shared<hz::doc::Sketch>();
    const auto line = [&sketch](Vec2 a, Vec2 b) {
        sketch->addEntity(std::make_shared<hz::draft::DraftLine>(a, b));
    };
    line({0, -60}, {60, -60});
    line({60, -60}, {60, 60});
    line({60, 60}, {0, 60});
    line({0, 60}, {0, -60});
    part.addSketch(sketch);
    auto slab = std::make_unique<hz::doc::ExtrudeFeature>(sketch, Vec3(0, 0, 1), 20.0);
    slab->setOperation(hz::doc::BodyOperation::Cut);
    part.featureTree().addFeature(std::move(slab));
    savePart(folder.file("cut.hzpart"), part);
    ASSERT_EQ(part.exactSolid(), nullptr) << "a cut leaves the part faceted";

    const Drawing drawing = loadSheet(folder, folder.file("cut.hzpart"), standardSheet(0.0));
    const Drawing master = hz::model::DrawingGenerator::sheetLayout(
        *part.solid(), hz::model::Sheet{}, hz::model::TitleBlock{}, 10.0);
    EXPECT_EQ(kindsOf(drawing), kindsOf(master));
    ASSERT_EQ(drawing.views.size(), 4u);
    EXPECT_DOUBLE_EQ(drawing.views[0].scale, 1.0);
    // Edge, Tangent, Silhouette (0, 1, 2); Visible, Hidden (0, 1): Front, Top, Right, Isometric.
    const std::vector<Kinds> masters{
        {{{0, 0}, 364}, {{0, 1}, 328}, {{2, 0}, 2}},
        {{{0, 0}, 672}, {{0, 1}, 18}},
        {{{0, 0}, 257}, {{0, 1}, 437}, {{2, 0}, 2}},
        {{{0, 0}, 518}, {{0, 1}, 175}, {{2, 0}, 2}},
    };
    EXPECT_EQ(kindsOf(drawing), masters);
}

// A radius dimension on the rim, saved by its logical name, finds the rim on the copy and reads
// 50.000: the circle the rim's chords record, not a chord.
TEST(DrawingExact, ARadiusDimensionIsExact) {
    const TempFolder folder("radius");
    Document part;
    part.featureTree().addFeature(cylinder());
    savePart(folder.file("cylinder.hzpart"), part);

    DrawingDocumentSpec spec;
    spec.views = DrawingDocumentIO::viewsOf(
        loadSheet(folder, folder.file("cylinder.hzpart"), standardSheet(1.0)));
    std::size_t top = 0;
    for (std::size_t i = 0; i < spec.views.size(); ++i) {
        if (spec.views[i].kind == StandardView::Top) top = i;
    }
    spec.views[top].dimensions.push_back(
        {"primitive_1/edge:side|top", hz::io::DrawingDimensionSpec::Kind::Radius});
    const Drawing drawing = loadSheet(folder, folder.file("cylinder.hzpart"), spec);
    ASSERT_EQ(drawing.views[top].radialDimensions.size(), 1u);
    EXPECT_NEAR(drawing.views[top].radialDimensions[0].value, 50.0, 1e-9);

    Document sheet;
    hz::draft::DimensionStyle style = sheet.draftDocument().dimensionStyle();
    style.precision = 3;
    sheet.draftDocument().setDimensionStyle(style);
    hz::io::DrawingExport::populate(sheet, drawing);
    std::vector<std::string> texts;
    for (const auto& entity : sheet.draftDocument().entities()) {
        const auto* text = dynamic_cast<const hz::draft::DraftText*>(entity.get());
        if (text != nullptr && entity->layer() == "Dimensions") texts.push_back(text->text());
    }
    EXPECT_EQ(texts, std::vector<std::string>{"R50.000"});
}

// Ruling 17: a part of an import, a cylinder cut off on a slant (its side a trimmed curved face,
// which the bridge from exact faces refused), and a primitive cylinder beside it is drawn from
// its exact part: the import's faces cut as its own bridge cuts them, the primitive's from their
// exact fields, both within the tolerance. The front view's outlines of both lie within 0.01 mm
// on paper of their cylinders', at the scale the sheet chose.
TEST(DrawingExact, AnImportBesideAPrimitiveIsDrawnFromItsExactFaces) {
    const TempFolder folder("import");
    hz::io::StepReadOptions inMillimetres;
    inMillimetres.unknownLengthUnit = hz::math::LengthUnit::Millimetre;
    auto read = hz::io::StepFormat::fromString(hz::step_text::slantCylinder(10.0, 20.0, 0.3, 0.1),
                                               nullptr, nullptr, inMillimetres);
    ASSERT_EQ(read.size(), 1u) << hz::io::StepFormat::lastError();
    Document part;
    part.featureTree().addFeature(std::make_unique<hz::doc::ImportedBodyFeature>(
        std::shared_ptr<const hz::topo::Solid>(std::move(read.front())), "slant.step"));
    auto pin = PrimitiveFeature::makeCylinder(10.0, 20.0);
    ASSERT_TRUE(pin->setVector("basePoint", Vec3(40.0, 0.0, 0.0)));
    ASSERT_TRUE(pin->setParameter("segments", 45));
    part.featureTree().addFeature(std::move(pin));
    savePart(folder.file("mixed.hzpart"), part);
    ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();

    const auto started = std::chrono::steady_clock::now();
    const Drawing drawing = loadSheet(folder, folder.file("mixed.hzpart"), standardSheet(0.0));
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    ASSERT_EQ(drawing.views.size(), 4u);
    const DrawingView& front = viewOf(drawing, StandardView::Front);
    RecordProperty("sheetSeconds", std::to_string(seconds));
    RecordProperty("scale", std::to_string(front.scale));
    const double tolerance = kOnPaper / front.scale;
    for (const double centre : {0.0, 40.0}) {
        SCOPED_TRACE(centre);
        const auto [inside, ends] = silhouetteInside(front, centre, 10.0, 0.5);
        EXPECT_GE(ends, 4);
        EXPECT_LE(inside, tolerance);
        EXPECT_GE(inside, -1e-9);
    }
}

// Ruling 31: on a sheet's copy of an exact part the lines inside a face are `<face>/line:<n>`,
// names of the copy alone, for its lines are elsewhere at every scale. A dimension saved on one of
// the faceted part's lines, `<face>/seam:<n>` (the 45-sided cylinder's outline at -x, measuring
// its height), is lost on the exact part, and said to be: it is not measured on whichever line of
// the copy took that number. The faceted part still measured it.
TEST(DrawingExact, ADimensionOnAFacetedLineIsLostOnTheExactPart) {
    const TempFolder folder("line");
    Document part;
    part.featureTree().addFeature(cylinder());
    savePart(folder.file("cylinder.hzpart"), part);
    ASSERT_NE(part.exactSolid(), nullptr);

    // The faceted part's line at its outline at -x.
    std::string line;
    for (const auto& e : part.solid()->edges()) {
        const std::string& tag = e.topoId.tag();
        if (tag.rfind("primitive_1/side/seam:", 0) != 0) continue;
        if (e.halfEdge->origin->point.x < -49.0 && e.halfEdge->twin->origin->point.x < -49.0) {
            line = tag;
            break;
        }
    }
    ASSERT_FALSE(line.empty());
    for (const auto& e : part.exactSolid()->edges()) ASSERT_NE(e.topoId.tag(), line);

    DrawingDocumentSpec spec;
    spec.views = DrawingDocumentIO::viewsOf(
        loadSheet(folder, folder.file("cylinder.hzpart"), standardSheet(1.0)));
    ASSERT_EQ(spec.views[0].kind, StandardView::Front);
    spec.views[0].dimensions.push_back({line, hz::io::DrawingDimensionSpec::Kind::Length});

    std::vector<std::string> lost;
    const Drawing faceted = DrawingDocumentIO::build(*part.solid(), spec, &lost);
    EXPECT_TRUE(lost.empty());
    ASSERT_EQ(faceted.views[0].dimensions.size(), 1u);
    EXPECT_NEAR(faceted.views[0].dimensions[0].value, 40.0, 1e-9);

    const auto copy = DrawingDocumentIO::cutForSheet(*part.exactSolid(), 1.0);
    ASSERT_NE(copy, nullptr);
    int lines = 0;
    for (const auto& e : copy->edges()) {
        const std::string& tag = e.topoId.tag();
        if (tag.rfind("primitive_1/side/line:", 0) == 0) ++lines;
        if (tag.rfind("primitive_1/side/seam:", 0) == 0) {
            EXPECT_NEAR(e.halfEdge->origin->point.x, 50.0, 1e-9) << tag << ": the exact seam alone";
        }
    }
    EXPECT_GT(lines, 100);
    const Drawing exact = DrawingDocumentIO::build(*copy, spec, &lost);
    EXPECT_EQ(lost, std::vector<std::string>{"view 1: a length"});
    EXPECT_TRUE(exact.views[0].dimensions.empty());
    EXPECT_TRUE(
        loadSheet(folder, folder.file("cylinder.hzpart"), spec).views[0].dimensions.empty());
}

// A section of an exact part is cut from its exact faces too (the section's callers draw from the
// sheet's copy): the cylinder cut across at half its height is a ring within 0.01 mm of its
// circle on a 1:1 sheet, points and chords, where the faceted part's lay 0.09 mm inside.
TEST(DrawingExact, ASectionIsCutFromTheExactFaces) {
    const TempFolder folder("section");
    Document part;
    part.featureTree().addFeature(cylinder());
    savePart(folder.file("cylinder.hzpart"), part);

    DrawingDocumentSpec spec;
    spec.views = DrawingDocumentIO::viewsOf(
        loadSheet(folder, folder.file("cylinder.hzpart"), standardSheet(1.0)));
    DrawingViewSpec section;
    section.role = hz::model::ViewRole::Section;
    section.label = "A";
    section.source = 0;
    section.projection.origin = Vec3(0, 0, 20);
    section.projection.dir = Vec3(0, 0, -1);
    section.projection.up = Vec3(0, 1, 0);
    section.placement = {300, 200};
    spec.views.push_back(section);

    // How far inside the circle the cut's chords reach at most, and how far off it its points.
    const auto sag = [](const DrawingView& view) {
        double inside = 0.0;
        double off = 0.0;
        for (const auto& loop : view.sectionLoops) {
            for (std::size_t k = 0; k < loop.size(); ++k) {
                const Vec2& a = loop[k];
                const Vec2& b = loop[(k + 1) % loop.size()];
                off = std::max(off, std::abs(std::hypot(a.x, a.y) - 50.0));
                inside = std::max(inside, 50.0 - std::hypot(0.5 * (a.x + b.x), 0.5 * (a.y + b.y)));
            }
        }
        return std::make_pair(inside, off);
    };
    const Drawing faceted = DrawingDocumentIO::build(*part.solid(), spec);
    ASSERT_EQ(faceted.views.size(), 5u);
    EXPECT_GT(sag(faceted.views[4]).first, 0.09);

    const Drawing drawing = loadSheet(folder, folder.file("cylinder.hzpart"), spec);
    ASSERT_EQ(drawing.views.size(), 5u);
    ASSERT_EQ(drawing.views[4].sectionLoops.size(), 1u);
    const auto [inside, off] = sag(drawing.views[4]);
    EXPECT_LE(inside, kOnPaper);
    EXPECT_LE(off, kOnPaper);
}

// Ruling 34: an outline line of a sheet's copy is named for the tolerance the copy was cut at,
// `<face>/line:<n>/at:<k>` (k = 4 log2 of the scale, rounded up). A dimension placed on one, the
// cylinder's outline at -x, measuring its height, is measured again when the sheet is saved and
// opened at the same scale; at 2:1 the copy is cut finer, has no line of that name, and the
// dimension is lost and said to be, not measured on whichever line took that number.
TEST(DrawingExact, ADimensionOnAnOutlineHoldsAtItsScaleAndIsLostAtAnother) {
    const TempFolder folder("outline");
    Document part;
    part.featureTree().addFeature(cylinder());
    savePart(folder.file("cylinder.hzpart"), part);
    EXPECT_EQ(DrawingDocumentIO::toleranceStep(1.0), 0);
    EXPECT_EQ(DrawingDocumentIO::toleranceStep(2.0), 4);
    EXPECT_EQ(DrawingDocumentIO::toleranceStep(0.5), -4);
    EXPECT_EQ(DrawingDocumentIO::toleranceStep(5.0), 10);

    const Drawing atOne = loadSheet(folder, folder.file("cylinder.hzpart"), standardSheet(1.0));
    const DrawingView& front = viewOf(atOne, StandardView::Front);
    std::string outline;
    for (const ProjectedEdge& e : front.edges) {
        if (e.kind == ProjectedEdge::Kind::Silhouette && e.a.x < -49.0)
            outline = e.sourceEdge.tag();
    }
    ASSERT_EQ(outline.rfind("primitive_1/side/line:", 0), 0u) << outline;
    ASSERT_EQ(outline.substr(outline.size() - 5), "/at:0") << outline;
    EXPECT_EQ(hz::model::logicalEdge(outline), outline) << "its step is no chord's number";

    DrawingDocumentSpec spec;
    spec.views = DrawingDocumentIO::viewsOf(atOne);
    ASSERT_EQ(spec.views[0].kind, StandardView::Front);
    spec.views[0].dimensions.push_back({outline, hz::io::DrawingDimensionSpec::Kind::Length});
    const Drawing again = loadSheet(folder, folder.file("cylinder.hzpart"), spec);
    ASSERT_EQ(again.views[0].dimensions.size(), 1u) << "saved and opened at 1:1";
    EXPECT_NEAR(again.views[0].dimensions[0].value, 40.0, 1e-9);

    for (DrawingViewSpec& v : spec.views) v.scale = 2.0;
    EXPECT_TRUE(
        loadSheet(folder, folder.file("cylinder.hzpart"), spec).views[0].dimensions.empty());
    const auto finer = DrawingDocumentIO::cutForSheet(*part.exactSolid(), 2.0);
    ASSERT_NE(finer, nullptr);
    for (const auto& e : finer->edges()) {
        EXPECT_NE(hz::model::logicalEdge(e.topoId.tag()), outline);
    }
    std::vector<std::string> lost;
    const Drawing atTwo = DrawingDocumentIO::build(*finer, spec, &lost);
    EXPECT_TRUE(atTwo.views[0].dimensions.empty());
    EXPECT_EQ(lost, std::vector<std::string>{"view 1: a length"});
}

// Ruling 35: the copy is cut for the sheet's views but its details, each cut from its view as
// drawn: a 10:1 detail of a 1:1 front view leaves the copy cut for 1:1 (the right view's outline
// 0.0099 mm inside, as at 1:1 alone), not ten times finer, and the detail is drawn from it.
TEST(DrawingExact, ADetailIsCutFromTheCopyOfItsView) {
    const TempFolder folder("detail");
    Document part;
    part.featureTree().addFeature(cylinder());
    savePart(folder.file("cylinder.hzpart"), part);
    const Drawing atOne = loadSheet(folder, folder.file("cylinder.hzpart"), standardSheet(1.0));
    const double alone = silhouetteInside(viewOf(atOne, StandardView::Right), 0.0, 50.0).first;
    EXPECT_GT(alone, kOnPaper / 2.0);

    DrawingDocumentSpec spec;
    spec.views = DrawingDocumentIO::viewsOf(atOne);
    DrawingViewSpec detail;
    detail.role = hz::model::ViewRole::Detail;
    detail.label = "B";
    detail.source = 0;
    detail.kind = spec.views[0].kind;
    detail.projection = spec.views[0].projection;
    detail.detailCenter = {-50.0, 20.0};  // on the front view's outline at -x
    detail.detailRadius = 5.0;
    detail.scale = 10.0;
    detail.placement = {300, 200};
    spec.views.push_back(detail);
    EXPECT_DOUBLE_EQ(DrawingDocumentIO::sheetScale(spec, *part.solid(), spec.titleBlock), 1.0);
    const Drawing drawing = loadSheet(folder, folder.file("cylinder.hzpart"), spec);
    ASSERT_EQ(drawing.views.size(), 5u);
    EXPECT_DOUBLE_EQ(silhouetteInside(viewOf(drawing, StandardView::Right), 0.0, 50.0).first,
                     alone);
    EXPECT_EQ(drawing.views[4].role, hz::model::ViewRole::Detail);
    EXPECT_FALSE(drawing.views[4].edges.empty());
}
