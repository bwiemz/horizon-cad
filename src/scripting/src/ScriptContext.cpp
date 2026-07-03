#include "horizon/scripting/ScriptContext.h"

#include <memory>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/DrawingExport.h"
#include "horizon/math/Vec2.h"

namespace hz::script {

using hz::math::Vec2;
using hz::math::Vec3;

ScriptContext::ScriptContext(doc::Document& document) : m_document(document) {}

int ScriptContext::featureCount() const {
    return static_cast<int>(m_document.featureTree().featureCount());
}

int ScriptContext::sketchCount() const {
    return static_cast<int>(m_document.sketches().size());
}

bool ScriptContext::hasSolid() const {
    return m_document.solid() != nullptr;
}

int ScriptContext::solidFaceCount() const {
    const auto* s = m_document.solid();
    return s ? static_cast<int>(s->faceCount()) : -1;
}

int ScriptContext::solidShellCount() const {
    const auto* s = m_document.solid();
    return s ? static_cast<int>(s->shellCount()) : -1;
}

std::string ScriptContext::lastError() const {
    return m_document.lastBuildMessage();
}

int ScriptContext::addRectangleSketch(double w, double h) {
    auto sketch = std::make_shared<doc::Sketch>();
    sketch->addEntity(std::make_shared<draft::DraftLine>(Vec2(0, 0), Vec2(w, 0)));
    sketch->addEntity(std::make_shared<draft::DraftLine>(Vec2(w, 0), Vec2(w, h)));
    sketch->addEntity(std::make_shared<draft::DraftLine>(Vec2(w, h), Vec2(0, h)));
    sketch->addEntity(std::make_shared<draft::DraftLine>(Vec2(0, h), Vec2(0, 0)));
    m_document.addSketch(sketch);
    return static_cast<int>(m_document.sketches().size()) - 1;
}

void ScriptContext::addBox(double width, double height, double depth) {
    m_document.featureTree().addFeature(doc::PrimitiveFeature::makeBox(width, height, depth));
}
void ScriptContext::addCylinder(double radius, double height) {
    m_document.featureTree().addFeature(doc::PrimitiveFeature::makeCylinder(radius, height));
}
void ScriptContext::addSphere(double radius) {
    m_document.featureTree().addFeature(doc::PrimitiveFeature::makeSphere(radius));
}
void ScriptContext::addCone(double bottomRadius, double topRadius, double height) {
    m_document.featureTree().addFeature(
        doc::PrimitiveFeature::makeCone(bottomRadius, topRadius, height));
}
void ScriptContext::addTorus(double majorRadius, double minorRadius) {
    m_document.featureTree().addFeature(doc::PrimitiveFeature::makeTorus(majorRadius, minorRadius));
}

bool ScriptContext::addExtrude(int sketchIndex, const Vec3& direction, double distance) {
    if (sketchIndex < 0 || sketchIndex >= static_cast<int>(m_document.sketches().size())) {
        return false;
    }
    auto sketch = m_document.sketches()[static_cast<size_t>(sketchIndex)];
    m_document.featureTree().addFeature(
        std::make_unique<doc::ExtrudeFeature>(sketch, direction, distance));
    return true;
}

bool ScriptContext::addLinearPattern(const Vec3& direction, double spacing, int count) {
    if (count < 1) return false;
    m_document.featureTree().addFeature(doc::PatternFeature::makeLinear(direction, spacing, count));
    return true;
}

void ScriptContext::addDatumPlane(const Vec3& origin, const Vec3& normal, const Vec3& xAxis) {
    m_document.featureTree().addFeature(
        doc::DatumFeature::makePlane(model::DatumPlane{origin, normal, xAxis}));
}

void ScriptContext::addDatumAxis(const Vec3& origin, const Vec3& direction) {
    m_document.featureTree().addFeature(
        doc::DatumFeature::makeAxis(model::DatumAxis{origin, direction}));
}

void ScriptContext::addDatumPoint(const Vec3& position) {
    m_document.featureTree().addFeature(doc::DatumFeature::makePoint(model::DatumPoint{position}));
}

model::MassProperties ScriptContext::massProperties(double density) const {
    const auto* solid = m_document.solid();
    if (!solid) return {};
    const model::Material material{"custom", density};
    return model::MassPropertiesCalculator::compute(*solid, &material);
}

bool ScriptContext::rebuild() {
    return m_document.rebuildModel();
}

bool ScriptContext::exportDrawingDxf(const std::string& path) const {
    const auto* solid = m_document.solid();
    if (!solid) return false;
    return io::DrawingExport::standardViewsToDxf(path, *solid);
}

}  // namespace hz::script
