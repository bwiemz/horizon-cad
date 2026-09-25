#include "horizon/document/Document.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <string>

#include "horizon/document/UndoStack.h"
#include "horizon/math/Expression.h"
#include "horizon/math/Quantity.h"

namespace hz::doc {

namespace {
std::atomic<std::uint64_t> g_nextSerial{1};
}  // namespace

Document::Document()
    : m_undoStack(std::make_unique<UndoStack>()),
      m_serial(g_nextSerial.fetch_add(1, std::memory_order_relaxed)) {}

Document::~Document() = default;

uint64_t Document::addEntity(std::shared_ptr<draft::DraftEntity> entity) {
    if (!entity) return 0;
    uint64_t id = entity->id();
    m_draftDoc.addEntity(std::move(entity));
    m_dirty = true;
    return id;
}

std::shared_ptr<draft::DraftEntity> Document::removeEntity(uint64_t id) {
    const auto& entities = m_draftDoc.entities();
    std::shared_ptr<draft::DraftEntity> found;
    for (const auto& e : entities) {
        if (e->id() == id) {
            found = e;
            break;
        }
    }
    m_draftDoc.removeEntity(id);
    m_dirty = true;
    return found;
}

bool Document::isDirty() const {
    return m_dirty || !m_undoStack->isClean();
}

void Document::setDirty(bool dirty) {
    m_dirty = dirty;
    if (!dirty)
        m_undoStack->setClean();  // notifies through the undo stack
    else if (m_onChange)
        m_onChange();
}

void Document::setChangeCallback(std::function<void()> callback) {
    m_onChange = std::move(callback);
    if (m_onChange) {
        m_undoStack->setChangeCallback([this] { m_onChange(); });
    } else {
        m_undoStack->setChangeCallback(nullptr);
    }
}

void Document::clear() {
    m_draftDoc.clear();
    m_layerManager.clear();
    m_constraintSystem.clear();
    m_parameterRegistry.clear();
    m_configurations = {};
    m_undoStack->clear();
    m_dirty = false;
    m_filePath.clear();
    m_lengthUnit = math::LengthUnit::Millimetre;

    m_editedSketch.reset();
    m_sketches.clear();

    m_featureTree.clear();
    m_solid.reset();
    m_lastBuildMessage.clear();
    m_failedFeatureIndex = -1;
    m_built = false;
}

bool Document::rebuildModel() {
    return applyBuild(buildWithDiagnostics());
}

BuildResult Document::buildWithDiagnostics(BuildControl* control) {
    applyExpressions();
    return m_featureTree.buildWithDiagnostics(control);
}

std::map<std::string, std::string> Document::effectiveDefinitions() const {
    return m_configurations.overlay(m_parameterRegistry.definitions(), m_configurations.active());
}

std::map<std::string, math::Quantity> Document::variables(
    std::map<std::string, std::string>* errors) const {
    if (m_configurations.active().empty()) return m_parameterRegistry.quantities(errors);
    ParameterRegistry laid;
    laid.setDefinitions(effectiveDefinitions());
    return laid.quantities(errors);
}

std::map<std::string, double> Document::variableValues() const {
    std::map<std::string, double> values;
    for (const auto& [name, quantity] : variables()) values[name] = quantity.value;
    return values;
}

std::function<double(const std::string&)> Document::variableResolver() const {
    return [values = variableValues()](const std::string& name) {
        const auto found = values.find(name);
        return found != values.end() ? found->second : std::numeric_limits<double>::quiet_NaN();
    };
}

void Document::applyExpressions() {
    std::map<std::string, math::Quantity> variables;
    bool worked = false;  // worked out once, if any feature has an expression
    for (size_t index = 0; index < m_featureTree.featureCount(); ++index) {
        Feature* feature = m_featureTree.feature(index);
        if (feature == nullptr) continue;
        if (feature->parameterExpressions().empty()) {
            feature->setExpressionError({});
            continue;
        }
        if (!worked) {
            variables = this->variables();
            worked = true;
        }
        std::string error;
        for (const auto& [name, text] : feature->parameterExpressions()) {
            std::string why;
            const auto expression = math::Expression::parse(text);
            const auto q =
                expression ? math::evaluateQuantity(*expression, variables, &why) : std::nullopt;
            if (!expression) why = "it is not an expression";
            std::optional<double> value;
            if (q) {
                switch (feature->parameterKind(name)) {
                    case Feature::ParameterKind::Length:
                        if (q->length == 1 && q->angle == 0)
                            value = q->value;
                        else
                            why = "it gives " + math::measureName(*q) + ", not a length";
                        break;
                    case Feature::ParameterKind::Angle:
                        if (q->length == 0 && q->angle == 1)
                            value = q->value;
                        else
                            why = "it gives " + math::measureName(*q) + ", not an angle";
                        break;
                    case Feature::ParameterKind::Count:
                    case Feature::ParameterKind::Choice:
                        if (q->pure())
                            value = std::round(q->value);
                        else
                            why = "it gives " + math::measureName(*q) + ", not a number";
                        break;
                }
            }
            if (value && !feature->setParameter(name, *value)) {
                why = "the feature cannot take " + std::to_string(*value);
                value.reset();
            }
            if (!value && error.empty()) {
                error = "its " + name + ", " + text + ", cannot be worked out: " + why;
            }
        }
        feature->setExpressionError(std::move(error));
    }
}

bool Document::applyBuild(BuildResult result) {
    if (result.cancelled) return m_failedFeatureIndex < 0;
    ++m_builds;
    m_solid = std::move(result.solid);
    m_lastBuildMessage = result.failureMessage;
    m_failedFeatureIndex = result.failedFeatureIndex;
    m_built = true;
    m_builtRevision = m_featureTree.revision();
    // Each sketch where the build placed it on its face (Phase 157): for a
    // build of a copy, the copy's were placed, not these.
    for (const auto& [id, plane] : result.placements) {
        if (const auto sketch = findSketch(id)) sketch->setPlaced(plane);
    }
    return m_failedFeatureIndex < 0;
}

void Document::addSketch(std::shared_ptr<Sketch> sketch) {
    if (sketch) m_sketches.push_back(std::move(sketch));
}

std::shared_ptr<Sketch> Document::removeSketch(uint64_t sketchId) {
    auto it = std::find_if(m_sketches.begin(), m_sketches.end(),
                           [sketchId](const auto& s) { return s->id() == sketchId; });
    if (it != m_sketches.end()) {
        auto sketch = *it;
        m_sketches.erase(it);
        if (sketch == m_editedSketch) m_editedSketch.reset();
        return sketch;
    }
    return nullptr;
}

std::shared_ptr<Sketch> Document::findSketch(uint64_t sketchId) const {
    for (const auto& sketch : m_sketches) {
        if (sketch->id() == sketchId) return sketch;
    }
    return nullptr;
}

std::vector<draft::DraftDocument*> Document::drawings() {
    std::vector<draft::DraftDocument*> all{&m_draftDoc};
    for (const auto& sketch : m_sketches) all.push_back(&sketch->drawing());
    return all;
}

void Document::editSketch(std::shared_ptr<Sketch> sketch) {
    // Only a sketch of this document: another's would outlive nothing here.
    if (sketch && std::find(m_sketches.begin(), m_sketches.end(), sketch) == m_sketches.end()) {
        sketch.reset();
    }
    m_editedSketch = std::move(sketch);
}

UndoStack& Document::undoStack() {
    return *m_undoStack;
}
const UndoStack& Document::undoStack() const {
    return *m_undoStack;
}

}  // namespace hz::doc
