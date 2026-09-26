// Makes the sample documents Horizon CAD ships (Phase 168b), with the same
// API the application and the tests use: a sample always opens in the
// version that made it, and a change of format is a change here too.
//
//   hz_make_samples <output directory>
//
// A part is rebuilt before it is saved, and the program fails when any
// feature does: a sample that does not build is not shipped.

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/drafting/DraftRadialDimension.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/fileio/DrawingDocumentIO.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/math/Mat4.h"
#include "horizon/modeling/FacePlane.h"
#include "horizon/topology/Solid.h"

namespace fs = std::filesystem;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

constexpr double kPi = 3.14159265358979323846;

/// The whole name of @p solid's flat face facing @p way through @p point.
std::optional<std::string> faceAt(const hz::topo::Solid& solid, const Vec3& way,
                                  const Vec3& point) {
    const double outward = hz::model::outwardSign(solid);
    for (const auto& face : solid.faces()) {
        const auto plane = hz::model::planeOf(face, outward);
        if (plane && plane->normal.dot(way) > 0.999 &&
            std::abs((point - plane->origin).dot(plane->normal)) < 1e-6) {
            return hz::model::wholeFaceName(face.topoId.tag());
        }
    }
    return std::nullopt;
}

/// The name of @p solid's edge standing on (x, y), parallel to Z.
std::optional<hz::topo::TopologyID> verticalEdgeAt(const hz::topo::Solid& solid, double x,
                                                   double y) {
    for (const auto& edge : solid.edges()) {
        const Vec3& a = edge.halfEdge->origin->point;
        const Vec3& b = edge.halfEdge->next->origin->point;
        if (std::abs(a.x - x) < 1e-9 && std::abs(a.y - y) < 1e-9 && std::abs(b.x - x) < 1e-9 &&
            std::abs(b.y - y) < 1e-9) {
            return edge.topoId;
        }
    }
    return std::nullopt;
}

bool fail(const std::string& what) {
    std::fprintf(stderr, "hz_make_samples: %s\n", what.c_str());
    return false;
}

/// Build @p doc, and fail on any feature that did not build.
bool built(hz::doc::Document& doc, const std::string& name) {
    if (!doc.rebuildModel() || doc.failedFeatureIndex() != -1 || doc.solid() == nullptr) {
        return fail(name + " does not build: " + doc.lastBuildMessage());
    }
    return true;
}

bool save(const fs::path& path, const hz::doc::Document& doc) {
    std::string error;
    if (!hz::io::NativeFormat::save(path.string(), doc, &error)) {
        return fail("cannot save " + path.string() + ": " + error);
    }
    std::printf("wrote %s\n", path.filename().string().c_str());
    return true;
}

/// An L-shaped bracket, 60 by 40, 8 thick and 30 wide, with its outer
/// corner rounded and a hole through each leg.
bool bracket(const fs::path& dir) {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    auto sketch = std::make_shared<hz::doc::Sketch>();
    sketch->setName("Profile");
    const std::vector<Vec2> outline = {Vec2(0, 0), Vec2(60, 0), Vec2(60, 8),
                                       Vec2(8, 8), Vec2(8, 40), Vec2(0, 40)};
    for (std::size_t i = 0; i < outline.size(); ++i) {
        sketch->addEntity(
            std::make_shared<hz::draft::DraftLine>(outline[i], outline[(i + 1) % outline.size()]));
    }
    doc.addSketch(sketch);
    doc.featureTree().addFeature(
        std::make_unique<hz::doc::ExtrudeFeature>(sketch, Vec3(0, 0, 1), 30.0));
    if (!built(doc, "bracket.hzpart")) return false;

    const auto corner = verticalEdgeAt(*doc.solid(), 0.0, 0.0);
    if (!corner) return fail("the bracket has no outer corner edge");
    doc.featureTree().addFeature(
        std::make_unique<hz::doc::FilletFeature>(std::vector<hz::topo::TopologyID>{*corner}, 6.0));
    if (!built(doc, "bracket.hzpart")) return false;

    // A hole through the foot, from its top, and one through the upright.
    const auto foot = faceAt(*doc.solid(), Vec3(0, 1, 0), Vec3(40, 8, 15));
    const auto upright = faceAt(*doc.solid(), Vec3(1, 0, 0), Vec3(8, 26, 15));
    if (!foot || !upright) return fail("the bracket's faces for its holes are not there");
    for (const auto& [face, at] :
         {std::pair{*foot, Vec3(40, 8, 15)}, std::pair{*upright, Vec3(8, 26, 15)}}) {
        auto hole = hz::doc::HoleFeature::make(face, at, 8.0, 10.0);
        hole->setParameter("extent", 1);  // through all
        doc.featureTree().addFeature(std::move(hole));
    }
    return built(doc, "bracket.hzpart") && save(dir / "bracket.hzpart", doc);
}

/// A plate, 100 by 60 by 10, with a row of three counterbored holes: one
/// hole, repeated by a pattern. Returns the box's and the first hole's
/// feature ids, which name their faces.
bool plate(const fs::path& dir, std::string& boxId, std::string& holeId) {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    auto box = hz::doc::PrimitiveFeature::makeBox(100, 60, 10);
    boxId = box->featureID();
    doc.featureTree().addFeature(std::move(box));
    if (!built(doc, "plate.hzpart")) return false;
    const auto top = faceAt(*doc.solid(), Vec3(0, 0, 1), Vec3(0, 0, 10));
    if (!top) return fail("the plate has no top face");
    auto hole = hz::doc::HoleFeature::make(*top, Vec3(20, 30, 10), 6.5, 10.0);
    hole->setParameter("extent", 1);  // through all
    hole->setParameter("type", 1);    // counterbored
    hole->setParameter("boreDiameter", 11.0);
    hole->setParameter("boreDepth", 4.0);
    holeId = hole->featureID();
    doc.featureTree().addFeature(std::move(hole));
    auto row = hz::doc::PatternFeature::makeLinear(Vec3(1, 0, 0), 30.0, 3);
    row->setTargets({holeId});
    doc.featureTree().addFeature(std::move(row));
    return built(doc, "plate.hzpart") && save(dir / "plate.hzpart", doc);
}

/// A pin, 6 across and 30 long. Returns its feature id.
bool pin(const fs::path& dir, std::string& pinId) {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    auto cylinder = hz::doc::PrimitiveFeature::makeCylinder(3.0, 30.0);
    pinId = cylinder->featureID();
    doc.featureTree().addFeature(std::move(cylinder));
    return built(doc, "pin.hzpart") && save(dir / "pin.hzpart", doc);
}

/// The plate, fixed, and a pin in its first hole: concentric with the hole,
/// its end flush with the plate's underside. Saved where the mates put it.
bool plateAndPin(const fs::path& dir, const std::string& holeId, const std::string& pinId,
                 const std::string& plateBoxId) {
    hz::doc::AssemblyDocument assembly;
    hz::doc::ComponentInstance plateComponent;
    plateComponent.name = "Plate";
    plateComponent.partPath = (dir / "plate.hzpart").string();
    const uint64_t plateId = assembly.addComponent(plateComponent);
    hz::doc::ComponentInstance pinComponent;
    pinComponent.name = "Pin";
    pinComponent.partPath = (dir / "pin.hzpart").string();
    pinComponent.transform = hz::math::Mat4::translation(Vec3(20, 30, 0));
    const uint64_t pinComponentId = assembly.addComponent(pinComponent);

    hz::doc::Mate fixed;
    fixed.type = hz::doc::MateType::Fixed;
    fixed.a.componentId = plateId;
    assembly.addMate(fixed);
    hz::doc::Mate onAxis;
    onAxis.type = hz::doc::MateType::Concentric;
    onAxis.a = {pinComponentId, hz::topo::TopologyID::fromTag(pinId + "/side")};
    onAxis.b = {plateId, hz::topo::TopologyID::fromTag(holeId + "/wall")};
    assembly.addMate(onAxis);
    hz::doc::Mate flush;
    flush.type = hz::doc::MateType::Coincident;
    flush.a = {pinComponentId, hz::topo::TopologyID::fromTag(pinId + "/bottom")};
    flush.b = {plateId, hz::topo::TopologyID::fromTag(plateBoxId + "/bottom")};
    assembly.addMate(flush);

    std::string error;
    const fs::path path = dir / "plate-and-pin.hzasm";
    if (!hz::io::NativeFormat::saveAssembly(path.string(), assembly, &error)) {
        return fail("cannot save " + path.string() + ": " + error);
    }
    std::printf("wrote %s\n", path.filename().string().c_str());
    return true;
}

/// A drawing sheet of the bracket: the standard four views, and its title
/// block filled in.
bool bracketDrawing(const fs::path& dir) {
    hz::io::DrawingDocumentSpec spec;
    spec.partPath = (dir / "bracket.hzpart").string();
    spec.titleBlock.title = "Bracket";
    spec.titleBlock.partNumber = "HZ-1001";
    spec.titleBlock.material = "Aluminium 6061";
    spec.titleBlock.drawnBy = "Horizon CAD";
    spec.titleBlock.company = "Horizon CAD samples";
    const fs::path path = dir / "bracket-drawing.hzdwg";
    if (!hz::io::DrawingDocumentIO::save(path.string(), spec)) {
        return fail("cannot save " + path.string());
    }
    std::printf("wrote %s\n", path.filename().string().c_str());
    return true;
}

/// A 2D gasket: a rounded outline, a bore, four bolt holes, centre lines,
/// dimensions and a note, on layers of their own.
bool gasket(const fs::path& dir) {
    hz::doc::Document doc;
    auto& layers = doc.layerManager();
    for (const auto& [name, color] :
         {std::pair{"Outline", 0xFFFFFFFFu}, std::pair{"Centre lines", 0xFFFF6060u},
          std::pair{"Dimensions", 0xFF60C060u}}) {
        hz::draft::LayerProperties layer;
        layer.name = name;
        layer.color = color;
        layers.addLayer(layer);
    }
    auto& drawing = doc.draftDocument();
    const auto on = [](std::shared_ptr<hz::draft::DraftEntity> entity, const char* layer) {
        entity->setLayer(layer);
        return entity;
    };
    // 80 by 60, corners of radius 10, centred on the origin.
    const double w = 40, h = 30, r = 10;
    drawing.addEntity(
        on(std::make_shared<hz::draft::DraftLine>(Vec2(-w + r, -h), Vec2(w - r, -h)), "Outline"));
    drawing.addEntity(
        on(std::make_shared<hz::draft::DraftLine>(Vec2(w, -h + r), Vec2(w, h - r)), "Outline"));
    drawing.addEntity(
        on(std::make_shared<hz::draft::DraftLine>(Vec2(w - r, h), Vec2(-w + r, h)), "Outline"));
    drawing.addEntity(
        on(std::make_shared<hz::draft::DraftLine>(Vec2(-w, h - r), Vec2(-w, -h + r)), "Outline"));
    const Vec2 corners[4] = {Vec2(w - r, -h + r), Vec2(w - r, h - r), Vec2(-w + r, h - r),
                             Vec2(-w + r, -h + r)};
    for (int i = 0; i < 4; ++i) {
        const double start = -kPi / 2 + i * kPi / 2;
        drawing.addEntity(
            on(std::make_shared<hz::draft::DraftArc>(corners[i], r, start, start + kPi / 2),
               "Outline"));
    }
    drawing.addEntity(on(std::make_shared<hz::draft::DraftCircle>(Vec2(0, 0), 15.0), "Outline"));
    for (const Vec2& at : corners) {
        drawing.addEntity(on(std::make_shared<hz::draft::DraftCircle>(at, 3.0), "Outline"));
    }
    drawing.addEntity(on(std::make_shared<hz::draft::DraftLine>(Vec2(-w - 5, 0), Vec2(w + 5, 0)),
                         "Centre lines"));
    drawing.addEntity(on(std::make_shared<hz::draft::DraftLine>(Vec2(0, -h - 5), Vec2(0, h + 5)),
                         "Centre lines"));
    drawing.addEntity(on(std::make_shared<hz::draft::DraftLinearDimension>(
                             Vec2(-w, -h), Vec2(w, -h), Vec2(0, -h - 12),
                             hz::draft::DraftLinearDimension::Orientation::Horizontal),
                         "Dimensions"));
    drawing.addEntity(on(std::make_shared<hz::draft::DraftLinearDimension>(
                             Vec2(w, -h), Vec2(w, h), Vec2(w + 12, 0),
                             hz::draft::DraftLinearDimension::Orientation::Vertical),
                         "Dimensions"));
    drawing.addEntity(on(std::make_shared<hz::draft::DraftRadialDimension>(
                             Vec2(0, 0), 15.0, Vec2(18, 18), /*isDiameter=*/true),
                         "Dimensions"));
    drawing.addEntity(on(std::make_shared<hz::draft::DraftText>(Vec2(-w, h + 10),
                                                                "GASKET, 2 mm, 4 HOLES DIA 6", 3.0),
                         "Dimensions"));
    return save(dir / "gasket.hcad", doc);
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: hz_make_samples <output directory>\n");
        return 2;
    }
    const fs::path dir = fs::absolute(argv[1]);
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) {
        fail("cannot make " + dir.string() + ": " + ec.message());
        return 1;
    }

    std::string boxId;
    std::string holeId;
    std::string pinId;
    const bool ok = bracket(dir) && bracketDrawing(dir) && plate(dir, boxId, holeId) &&
                    pin(dir, pinId) && plateAndPin(dir, holeId, pinId, boxId) && gasket(dir);
    return ok ? 0 : 1;
}
