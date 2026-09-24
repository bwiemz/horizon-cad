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

// --- EditFeatureCommand ---

EditFeatureCommand::EditFeatureCommand(Document& doc, const Feature* feature,
                                       std::map<std::string, double> parameters,
                                       std::optional<BodyOperation> operation)
    : m_doc(doc), m_feature(feature), m_new(std::move(parameters)), m_newOperation(operation) {}

void EditFeatureCommand::execute() {
    Feature* feature = inTree(m_doc, m_feature);
    if (!feature) return;
    // What the edit replaces, read now: it is what undo must restore.
    const auto current = feature->parameters();
    m_old.clear();
    for (const auto& [name, value] : m_new) {
        const auto it = current.find(name);
        if (it != current.end()) m_old[name] = it->second;
    }
    m_oldOperation = feature->operation();
    apply(m_new, m_newOperation.value_or(m_oldOperation));
}

void EditFeatureCommand::undo() {
    apply(m_old, m_oldOperation);
}

void EditFeatureCommand::apply(const std::map<std::string, double>& parameters,
                               BodyOperation operation) {
    Feature* feature = inTree(m_doc, m_feature);
    if (!feature) return;
    for (const auto& [name, value] : parameters) feature->setParameter(name, value);
    feature->setOperation(operation);
    m_doc.featureTree().markChanged();
}

std::string EditFeatureCommand::description() const {
    return "Edit " + nameOf(m_feature);
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
