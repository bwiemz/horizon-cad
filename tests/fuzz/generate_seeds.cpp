// Writes the fuzz seed corpus: small but real documents produced by the real
// writers, so fuzzing starts from inputs that reach deep into each reader.
//
// Usage: hz_fuzz_seedgen <tests/fuzz/corpus>
// The output is committed; rerun after a format change and commit the diff.

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftLeader.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

namespace fs = std::filesystem;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

bool write(const fs::path& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary);
    out << text;
    if (!out) {
        std::fprintf(stderr, "cannot write %s\n", path.string().c_str());
        return false;
    }
    std::printf("wrote %s (%zu bytes)\n", path.string().c_str(), text.size());
    return true;
}

std::shared_ptr<hz::doc::Sketch> rectangleSketch(double w, double h) {
    auto sketch = std::make_shared<hz::doc::Sketch>();
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(w, 0)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(w, 0), Vec2(w, h)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(w, h), Vec2(0, h)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, h), Vec2(0, 0)));
    return sketch;
}

std::string drawing() {
    hz::doc::Document doc;
    hz::draft::LayerProperties dims;
    dims.name = "Dimensions";
    dims.color = 0xFF00FF00;
    doc.layerManager().addLayer(dims);

    auto& d = doc.draftDocument();
    d.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(10, 5)));
    d.addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(3, 3), 2.0));
    d.addEntity(std::make_shared<hz::draft::DraftArc>(Vec2(0, 0), 4.0, 0.0, 1.5));
    d.addEntity(std::make_shared<hz::draft::DraftRectangle>(Vec2(-5, -5), Vec2(-1, -2)));
    d.addEntity(std::make_shared<hz::draft::DraftPolyline>(
        std::vector<Vec2>{{0, 0}, {1, 2}, {3, 1}, {4, 4}}, true));
    d.addEntity(std::make_shared<hz::draft::DraftSpline>(
        std::vector<Vec2>{{0, 0}, {1, 3}, {4, 3}, {5, 0}}, false));
    d.addEntity(std::make_shared<hz::draft::DraftEllipse>(Vec2(8, 8), 3.0, 1.5, 0.4));
    d.addEntity(std::make_shared<hz::draft::DraftText>(Vec2(1, 9), "R 2.5 \xC2\xB0", 2.5));
    d.addEntity(std::make_shared<hz::draft::DraftLeader>(std::vector<Vec2>{{6, 6}, {8, 9}, {11, 9}},
                                                         "note"));
    return hz::io::NativeFormat::documentToJson(doc, false);
}

std::string part() {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    auto sketch = rectangleSketch(10.0, 5.0);
    sketch->setName("Profile");
    doc.addSketch(sketch);
    auto& tree = doc.featureTree();
    tree.addFeature(std::make_unique<hz::doc::ExtrudeFeature>(sketch, Vec3(0, 0, 1), 4.0));
    auto cylinder = hz::doc::PrimitiveFeature::makeCylinder(2.0, 6.0);
    cylinder->setParameter("chordTolerance", 0.05);
    tree.addFeature(std::move(cylinder));
    tree.addFeature(hz::doc::PrimitiveFeature::makeBox(3.0, 3.0, 3.0));
    tree.addFeature(std::make_unique<hz::doc::BooleanFeature>(hz::model::BooleanType::Union));
    doc.rebuildModel();
    return hz::io::NativeFormat::documentToJson(doc, false);
}

std::string filletedPart() {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    doc.featureTree().addFeature(hz::doc::PrimitiveFeature::makeBox(10.0, 10.0, 10.0));
    doc.rebuildModel();
    if (doc.solid() && !doc.solid()->edges().empty()) {
        const auto edge = doc.solid()->edges().front().topoId;
        doc.featureTree().addFeature(
            std::make_unique<hz::doc::FilletFeature>(std::vector<hz::topo::TopologyID>{edge}, 1.5));
        doc.rebuildModel();
    }
    return hz::io::NativeFormat::documentToJson(doc, false);
}

std::string assembly() {
    hz::doc::AssemblyDocument asmDoc;
    hz::doc::ComponentInstance a;
    a.name = "base";
    a.partPath = "base.hzpart";
    hz::doc::ComponentInstance b;
    b.name = "pin";
    b.partPath = "pin.hzpart";
    b.transform = hz::math::Mat4::translation(Vec3(5, 0, 0));
    asmDoc.addComponent(a);
    asmDoc.addComponent(b);
    return hz::io::NativeFormat::assemblyToJson(asmDoc, "");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s <corpus directory>\n", argv[0]);
        return 2;
    }
    const fs::path root(argv[1]);
    for (const char* dir : {"native", "dxf", "step", "expression"})
        fs::create_directories(root / dir);

    bool ok = true;
    ok &= write(root / "native" / "drawing.hcad", drawing());
    ok &= write(root / "native" / "part.hzpart", part());
    ok &= write(root / "native" / "fillet.hzpart", filletedPart());
    ok &= write(root / "native" / "assembly.hzasm", assembly());

    // DXF: the same drawing through the DXF writer.
    {
        hz::doc::Document doc;
        std::string error;
        if (!hz::io::NativeFormat::documentFromJson(drawing(), doc, &error) ||
            !hz::io::DxfFormat::save((root / "dxf" / "drawing.dxf").string(), doc, &error)) {
            std::fprintf(stderr, "dxf seed: %s\n", error.c_str());
            ok = false;
        } else {
            std::printf("wrote %s\n", (root / "dxf" / "drawing.dxf").string().c_str());
        }
    }

    using hz::model::PrimitiveFactory;
    const auto box = PrimitiveFactory::makeBox(10.0, 20.0, 30.0);
    const auto cylinder = PrimitiveFactory::makeCylinder(5.0, 10.0, 8);
    ok &= write(root / "step" / "box.step", hz::io::StepFormat::toString({box.get()}));
    ok &= write(root / "step" / "cylinder.step", hz::io::StepFormat::toString({cylinder.get()}));

    const std::vector<std::pair<std::string, std::string>> expressions = {
        {"arith", "2*(x+3)^2 - y/4"},
        {"functions", "sqrt(abs(sin(a)*cos(b))) + atan2(width, 10)"},
        {"nested", "((((1+2)*3)-4)/5)"},
        {"unary", "--x - -(-y)"},
        {"pi", "pi*d^2/4"},
    };
    for (const auto& [name, text] : expressions) {
        ok &= write(root / "expression" / (name + ".txt"), text);
    }
    return ok ? 0 : 1;
}
