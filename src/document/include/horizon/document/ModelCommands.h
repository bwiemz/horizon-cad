#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/UndoStack.h"

namespace hz::doc {

class Document;
class Sketch;

// Undoable edits to a part's history and to an assembly.
//
// A feature command finds its feature by identity, not by position: an edit
// made outside the undo stack (a script adding a feature) moves positions,
// and an undo must still act on the feature it was pushed for. A feature a
// command has taken out of the tree is owned by the command until it goes
// back.
//
// None of these rebuild the model. The tree's revision() tells the caller
// that it has to.

/// Add a feature at the end of the history, together with the sketch it was
/// made from when that sketch is new to the document. The new feature is
/// active: the rollback index is cleared, and undo puts it back.
class AddFeatureCommand : public Command {
public:
    AddFeatureCommand(Document& doc, std::unique_ptr<Feature> feature,
                      std::shared_ptr<Sketch> newSketch = nullptr);

    void execute() override;
    void undo() override;
    std::string description() const override;

    /// The feature, in the tree or not.
    const Feature* feature() const { return m_feature; }

private:
    Document& m_doc;
    std::unique_ptr<Feature> m_owned;  ///< Held while the feature is out of the tree.
    const Feature* m_feature;
    std::shared_ptr<Sketch> m_sketch;
    int m_rollbackBefore = -1;
};

/// Delete a feature from the history. Its sketch stays in the document.
class RemoveFeatureCommand : public Command {
public:
    RemoveFeatureCommand(Document& doc, const Feature* feature);

    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    Document& m_doc;
    const Feature* m_feature;
    std::unique_ptr<Feature> m_owned;
    size_t m_index = 0;
    int m_rollbackBefore = -1;
    std::string m_name;
};

/// Move a feature to another position in the history.
class MoveFeatureCommand : public Command {
public:
    MoveFeatureCommand(Document& doc, const Feature* feature, size_t toIndex);

    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    Document& m_doc;
    const Feature* m_feature;
    size_t m_to;
    size_t m_from = 0;
    int m_rollbackBefore = -1;
};

/// Set several of a feature's parameters, in an order that holds: a facet
/// count set explicitly returns a faceted feature to count mode, clearing its
/// chord tolerance, so the tolerance is set after everything else. Returns the
/// names the feature refused (a zero distance, too few segments).
std::vector<std::string> setParameters(Feature& feature,
                                       const std::map<std::string, double>& values);

/// The names among `values` that the feature would refuse. It tries them,
/// then puts every parameter back as it was.
std::vector<std::string> refusedParameters(Feature& feature,
                                           const std::map<std::string, double>& values);

/// Change a feature's parameters and, for a feature that builds a body, how
/// that body combines with the part. Undo puts back every parameter, not only
/// the edited ones: setting one can change another (a facet count clears a
/// chord tolerance).
class EditFeatureCommand : public Command {
public:
    EditFeatureCommand(Document& doc, const Feature* feature,
                       std::map<std::string, double> parameters,
                       std::optional<BodyOperation> operation = std::nullopt);

    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    void apply(const std::map<std::string, double>& parameters, BodyOperation operation);

    Document& m_doc;
    const Feature* m_feature;
    std::map<std::string, double> m_new;
    std::map<std::string, double> m_old;
    std::optional<BodyOperation> m_newOperation;
    BodyOperation m_oldOperation = BodyOperation::NewBody;
};

/// Suppress a feature (leave it out of the build) or bring it back.
class SetFeatureSuppressedCommand : public Command {
public:
    SetFeatureSuppressedCommand(Document& doc, const Feature* feature, bool suppressed);

    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    void set(bool suppressed);

    Document& m_doc;
    const Feature* m_feature;
    bool m_suppressed;
    bool m_before = false;
};

/// Add a sketch to the document.
class AddSketchCommand : public Command {
public:
    AddSketchCommand(Document& doc, std::shared_ptr<Sketch> sketch);

    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    Document& m_doc;
    std::shared_ptr<Sketch> m_sketch;
};

/// Any edit to an assembly's components and mates, as the states before and
/// after it. The caller makes the edit (inserting, mating and solving), takes
/// a snapshot, and pushes this, which leaves the assembly as it is.
class AssemblyEditCommand : public Command {
public:
    AssemblyEditCommand(AssemblyDocument& assembly, AssemblyState before, AssemblyState after,
                        std::string description);

    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    AssemblyDocument& m_assembly;
    AssemblyState m_before;
    AssemblyState m_after;
    std::string m_description;
};

}  // namespace hz::doc
