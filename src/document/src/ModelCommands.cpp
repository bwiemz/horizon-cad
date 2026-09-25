#include "horizon/document/ModelCommands.h"

#include <algorithm>
#include <utility>

#include "horizon/document/Document.h"
#include "horizon/document/Sketch.h"

namespace hz::doc {

namespace {

/// The feature, when it is in the tree — mutable, for the commands that
/// edit it in place.
Feature* inTree(Document& doc, const Feature* feature) {
    auto& tree = doc.featureTree();
    const auto index = tree.indexOf(feature);
    return index ? tree.feature(*index) : nullptr;
}

std::string nameOf(const Feature* feature) {
    return feature ? feature->name() : std::string("Feature");
}

bool hasSketch(const Document& doc, const Sketch& sketch) {
    return std::any_of(doc.sketches().begin(), doc.sketches().end(),
                       [&sketch](const auto& s) { return s.get() == &sketch; });
}

}  // namespace

// --- AddFeatureCommand ---

AddFeatureCommand::AddFeatureCommand(Document& doc, std::unique_ptr<Feature> feature,
                                     std::shared_ptr<Sketch> newSketch)
    : m_doc(doc),
      m_owned(std::move(feature)),
      m_feature(m_owned.get()),
      m_sketch(std::move(newSketch)) {}

void AddFeatureCommand::execute() {
    if (!m_owned) return;  // already in the tree
    auto& tree = m_doc.featureTree();
    if (m_sketch && !hasSketch(m_doc, *m_sketch)) {
        m_doc.addSketch(m_sketch);
    } else {
        m_sketch.reset();  // not ours to take away again
    }
    m_rollbackBefore = tree.rollbackIndex();
    tree.setRollbackIndex(-1);
    tree.addFeature(std::move(m_owned));
}

void AddFeatureCommand::undo() {
    auto& tree = m_doc.featureTree();
    const auto index = tree.indexOf(m_feature);
    if (!index) return;
    m_owned = tree.takeFeature(*index);
    tree.setRollbackIndex(m_rollbackBefore);
    if (m_sketch) m_doc.removeSketch(m_sketch->id());
}

std::string AddFeatureCommand::description() const {
    return "Add " + nameOf(m_feature);
}

// --- RemoveFeatureCommand ---

RemoveFeatureCommand::RemoveFeatureCommand(Document& doc, const Feature* feature)
    : m_doc(doc), m_feature(feature), m_name(nameOf(feature)) {}

void RemoveFeatureCommand::execute() {
    auto& tree = m_doc.featureTree();
    const auto index = tree.indexOf(m_feature);
    if (!index) return;
    m_index = *index;
    m_rollbackBefore = tree.rollbackIndex();
    m_owned = tree.takeFeature(m_index);
}

void RemoveFeatureCommand::undo() {
    if (!m_owned) return;
    auto& tree = m_doc.featureTree();
    tree.insertFeature(m_index, std::move(m_owned));
    tree.setRollbackIndex(m_rollbackBefore);
}

std::string RemoveFeatureCommand::description() const {
    return "Delete " + m_name;
}

// --- MoveFeatureCommand ---

MoveFeatureCommand::MoveFeatureCommand(Document& doc, const Feature* feature, size_t toIndex)
    : m_doc(doc), m_feature(feature), m_to(toIndex) {}

void MoveFeatureCommand::execute() {
    auto& tree = m_doc.featureTree();
    const auto index = tree.indexOf(m_feature);
    if (!index) return;
    m_from = *index;
    m_rollbackBefore = tree.rollbackIndex();
    const size_t to = std::min(m_to, tree.featureCount() - 1);
    tree.moveFeature(static_cast<int>(m_from), static_cast<int>(to));
}

void MoveFeatureCommand::undo() {
    auto& tree = m_doc.featureTree();
    const auto index = tree.indexOf(m_feature);
    if (!index) return;
    tree.moveFeature(static_cast<int>(*index), static_cast<int>(m_from));
    tree.setRollbackIndex(m_rollbackBefore);
}

std::string MoveFeatureCommand::description() const {
    return "Reorder " + nameOf(m_feature);
}

// --- Parameters ---

std::vector<std::string> setParameters(Feature& feature,
                                       const std::map<std::string, double>& values) {
    std::vector<std::string> refused;
    const auto set = [&](const std::string& name, double value) {
        if (!feature.setParameter(name, value)) refused.push_back(name);
    };
    for (const auto& [name, value] : values) {
        if (name != "chordTolerance") set(name, value);
    }
    if (const auto it = values.find("chordTolerance"); it != values.end()) {
        set(it->first, it->second);
    }
    return refused;
}

std::vector<std::string> refusedParameters(Feature& feature,
                                           const std::map<std::string, double>& values) {
    const auto before = feature.parameters();
    auto refused = setParameters(feature, values);
    setParameters(feature, before);
    return refused;
}

// --- EditFeatureCommand ---

std::vector<std::string> refusedVectors(Feature& feature,
                                        const std::map<std::string, math::Vec3>& values) {
    const auto before = feature.vectors();
    std::vector<std::string> refused;
    for (const auto& [name, value] : values) {
        if (!feature.setVector(name, value)) refused.push_back(name);
    }
    for (const auto& [name, value] : before) feature.setVector(name, value);
    return refused;
}

EditFeatureCommand::EditFeatureCommand(Document& doc, const Feature* feature,
                                       std::map<std::string, double> parameters,
                                       std::optional<BodyOperation> operation,
                                       std::map<std::string, math::Vec3> vectors,
                                       std::map<std::string, std::string> expressions)
    : m_doc(doc),
      m_feature(feature),
      m_new(std::move(parameters)),
      m_newOperation(operation),
      m_newVectors(std::move(vectors)),
      m_newExpressions(std::move(expressions)) {}

void EditFeatureCommand::execute() {
    Feature* feature = inTree(m_doc, m_feature);
    if (!feature) return;
    // What the edit replaces, read now — all of it, since setting one
    // parameter can change another. It is what undo restores.
    m_old = feature->parameters();
    m_oldOperation = feature->operation();
    m_oldVectors = feature->vectors();
    m_oldExpressions = feature->parameterExpressions();
    std::map<std::string, std::string> expressions = m_oldExpressions;
    for (const auto& [name, text] : m_newExpressions) {
        if (text.empty()) {
            expressions.erase(name);
        } else {
            expressions[name] = text;
        }
    }
    apply(m_new, m_newOperation.value_or(m_oldOperation), m_newVectors, expressions);
}

void EditFeatureCommand::undo() {
    apply(m_old, m_oldOperation, m_oldVectors, m_oldExpressions);
}

void EditFeatureCommand::apply(const std::map<std::string, double>& parameters,
                               BodyOperation operation,
                               const std::map<std::string, math::Vec3>& vectors,
                               const std::map<std::string, std::string>& expressions) {
    Feature* feature = inTree(m_doc, m_feature);
    if (!feature) return;
    feature->setParameterExpressions(expressions);
    setParameters(*feature, parameters);
    for (const auto& [name, value] : vectors) feature->setVector(name, value);
    feature->setOperation(operation);
    m_doc.featureTree().markChanged();
}

std::string EditFeatureCommand::description() const {
    return "Edit " + nameOf(m_feature);
}

// --- SetRollbackCommand ---

SetRollbackCommand::SetRollbackCommand(Document& doc, int index) : m_doc(doc), m_index(index) {}

void SetRollbackCommand::execute() {
    auto& tree = m_doc.featureTree();
    m_before = tree.rollbackIndex();
    const int last = static_cast<int>(tree.featureCount()) - 1;
    // The last feature, or past it, is no rollback at all.
    tree.setRollbackIndex(m_index < 0 || m_index >= last ? -1 : m_index);
}

void SetRollbackCommand::undo() {
    m_doc.featureTree().setRollbackIndex(m_before);
}

std::string SetRollbackCommand::description() const {
    return m_index < 0 ? "Roll Forward" : "Roll Back";
}

// --- SetFeatureSuppressedCommand ---

SetFeatureSuppressedCommand::SetFeatureSuppressedCommand(Document& doc, const Feature* feature,
                                                         bool suppressed)
    : m_doc(doc), m_feature(feature), m_suppressed(suppressed) {}

void SetFeatureSuppressedCommand::execute() {
    if (const Feature* feature = inTree(m_doc, m_feature)) m_before = feature->isSuppressed();
    set(m_suppressed);
}

void SetFeatureSuppressedCommand::undo() {
    set(m_before);
}

void SetFeatureSuppressedCommand::set(bool suppressed) {
    Feature* feature = inTree(m_doc, m_feature);
    if (!feature) return;
    feature->setSuppressed(suppressed);
    m_doc.featureTree().markChanged();
}

std::string SetFeatureSuppressedCommand::description() const {
    return (m_suppressed ? "Suppress " : "Unsuppress ") + nameOf(m_feature);
}

// --- AddSketchCommand ---

AddSketchCommand::AddSketchCommand(Document& doc, std::shared_ptr<Sketch> sketch)
    : m_doc(doc), m_sketch(std::move(sketch)) {}

void AddSketchCommand::execute() {
    if (!m_sketch) return;
    if (hasSketch(m_doc, *m_sketch)) {
        m_sketch.reset();  // already there: not ours to take away again
        return;
    }
    m_doc.addSketch(m_sketch);
}

void AddSketchCommand::undo() {
    if (m_sketch) m_doc.removeSketch(m_sketch->id());
}

std::string AddSketchCommand::description() const {
    return "Add Sketch";
}

// --- AssemblyEditCommand ---

SetConfigurationsCommand::SetConfigurationsCommand(Document& document, ConfigurationTable after,
                                                   std::string description)
    : m_document(document), m_after(std::move(after)), m_description(std::move(description)) {}

void SetConfigurationsCommand::execute() {
    m_before = m_document.configurations();
    m_document.configurations() = m_after;
    m_document.featureTree().markChanged();
}

void SetConfigurationsCommand::undo() {
    m_document.configurations() = m_before;
    m_document.featureTree().markChanged();
}

std::string SetConfigurationsCommand::description() const {
    return m_description;
}

SetVariablesCommand::SetVariablesCommand(Document& document,
                                         std::map<std::string, std::string> after)
    : m_document(document), m_after(std::move(after)) {}

void SetVariablesCommand::execute() {
    m_before = m_document.parameterRegistry().definitions();
    m_document.parameterRegistry().setDefinitions(m_after);
    m_document.featureTree().markChanged();
}

void SetVariablesCommand::undo() {
    m_document.parameterRegistry().setDefinitions(m_before);
    m_document.featureTree().markChanged();
}

std::string SetVariablesCommand::description() const {
    return "Variables";
}

SetLengthUnitCommand::SetLengthUnitCommand(Document& document, math::LengthUnit unit,
                                           AssemblyDocument* assembly)
    : m_document(document), m_assembly(assembly), m_new(unit) {}

void SetLengthUnitCommand::set(math::LengthUnit unit) {
    m_document.setLengthUnit(unit);
    if (m_assembly != nullptr) m_assembly->setLengthUnit(unit);
}

void SetLengthUnitCommand::execute() {
    m_old = m_document.lengthUnit();
    set(m_new);
}

void SetLengthUnitCommand::undo() {
    set(m_old);
}

std::string SetLengthUnitCommand::description() const {
    return "Document Units";
}

AssemblyEditCommand::AssemblyEditCommand(AssemblyDocument& assembly, AssemblyState before,
                                         AssemblyState after, std::string description)
    : m_assembly(assembly),
      m_before(std::move(before)),
      m_after(std::move(after)),
      m_description(std::move(description)) {}

void AssemblyEditCommand::execute() {
    m_assembly.restore(m_after);
}

void AssemblyEditCommand::undo() {
    m_assembly.restore(m_before);
}

std::string AssemblyEditCommand::description() const {
    return m_description;
}

}  // namespace hz::doc
