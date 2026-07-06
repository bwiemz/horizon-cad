#include "horizon/scripting/ScriptContext.h"

#include <memory>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/DrawingExport.h"
#include "horizon/math/Vec2.h"
#include "horizon/simulation/LinearStaticSolver.h"
#include "horizon/simulation/Material.h"
#include "horizon/simulation/ModalSolver.h"
#include "horizon/simulation/SolidMesher.h"

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

ScriptContext::StaticAnalysisResult ScriptContext::staticAnalysis(double force,
                                                                  double youngsModulus,
                                                                  double poissonRatio, int axis,
                                                                  int resolution) const {
    StaticAnalysisResult out;
    const auto* solid = m_document.solid();
    if (!solid || axis < 0 || axis > 2 || resolution < 1) return out;

    sim::TetMesh mesh = sim::meshSolidBoundingBox(*solid, resolution, resolution, resolution);
    if (mesh.elements.empty()) return out;

    const sim::Aabb box = sim::solidAabb(*solid);
    const double lo = axis == 0 ? box.min.x : (axis == 1 ? box.min.y : box.min.z);
    const double hi = axis == 0 ? box.max.x : (axis == 1 ? box.max.y : box.max.z);

    const auto fixed = sim::nodesOnPlane(mesh, axis, lo);
    const auto loaded = sim::nodesOnPlane(mesh, axis, hi);
    if (fixed.empty() || loaded.empty()) return out;

    sim::ElasticMaterial mat;
    mat.youngsModulus = youngsModulus;
    mat.poissonRatio = poissonRatio;
    mat.density = 1.0;
    if (!mat.isValid()) return out;

    const double per = force / static_cast<double>(loaded.size());
    const Vec3 dir(axis == 0 ? per : 0.0, axis == 1 ? per : 0.0, axis == 2 ? per : 0.0);
    std::vector<sim::NodalLoad> loads;
    for (int n : loaded) loads.push_back(sim::NodalLoad{n, dir});

    const sim::StaticResult r = sim::LinearStaticSolver::solve(mesh, mat, fixed, loads);
    out.converged = r.converged;
    out.maxDisplacement = r.maxDisplacementMagnitude;
    out.maxVonMises = r.maxVonMises;
    return out;
}

ScriptContext::ModalAnalysisResult ScriptContext::modalAnalysis(double youngsModulus,
                                                                double poissonRatio, double density,
                                                                int axis, int numModes,
                                                                int resolution) const {
    ModalAnalysisResult out;
    const auto* solid = m_document.solid();
    if (!solid || axis < 0 || axis > 2 || numModes < 1 || resolution < 1) return out;

    sim::TetMesh mesh = sim::meshSolidBoundingBox(*solid, resolution, resolution, resolution);
    if (mesh.elements.empty()) return out;

    const sim::Aabb box = sim::solidAabb(*solid);
    const double lo = axis == 0 ? box.min.x : (axis == 1 ? box.min.y : box.min.z);
    const auto fixed = sim::nodesOnPlane(mesh, axis, lo);
    if (fixed.empty()) return out;

    sim::ElasticMaterial mat;
    mat.youngsModulus = youngsModulus;
    mat.poissonRatio = poissonRatio;
    mat.density = density;
    if (!mat.isValid() || density <= 0.0) return out;

    const sim::ModalResult r = sim::ModalSolver::solve(mesh, mat, fixed, numModes);
    out.converged = r.converged;
    out.naturalFrequencies = r.naturalFrequencies;
    return out;
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
