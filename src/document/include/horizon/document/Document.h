#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/document/ConfigurationTable.h"
#include "horizon/document/ExpressionEngine.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/ParameterRegistry.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/Units.h"

namespace hz::doc {

class UndoStack;

/// Kind of content a Document represents.
///
/// Drawing  — 2D drafting document (.hcad, .dxf)
/// Part     — parametric 3D part: sketches + feature tree (.hzpart)
/// Assembly — component references + mates (.hzasm); see AssemblyDocument
enum class DocumentType {
    Drawing,
    Part,
    Assembly,
};

/// Central document model for Horizon CAD.
/// Owns the DraftDocument (entity storage) and UndoStack.
class Document {
public:
    Document();
    ~Document();

    // --- Entity operations ---

    uint64_t addEntity(std::shared_ptr<draft::DraftEntity> entity);
    std::shared_ptr<draft::DraftEntity> removeEntity(uint64_t id);
    void clear();

    // --- Accessors ---

    const draft::DraftDocument& draftDocument() const { return m_draftDoc; }
    draft::DraftDocument& draftDocument() { return m_draftDoc; }

    UndoStack& undoStack();
    const UndoStack& undoStack() const;

    draft::LayerManager& layerManager() { return m_layerManager; }
    const draft::LayerManager& layerManager() const { return m_layerManager; }

    cstr::ConstraintSystem& constraintSystem() { return m_constraintSystem; }
    const cstr::ConstraintSystem& constraintSystem() const { return m_constraintSystem; }

    ParameterRegistry& parameterRegistry() { return m_parameterRegistry; }
    const ParameterRegistry& parameterRegistry() const { return m_parameterRegistry; }

    // --- Configurations (Phase 156) ---

    /// Its design table: configurations laid over its variables.
    ConfigurationTable& configurations() { return m_configurations; }
    const ConfigurationTable& configurations() const { return m_configurations; }

    /// Its variables as the part is built: its own, with the active
    /// configuration's laid over them.
    std::map<std::string, std::string> effectiveDefinitions() const;
    /// Those worked out (see ParameterRegistry::quantities).
    std::map<std::string, math::Quantity> variables(
        std::map<std::string, std::string>* errors = nullptr) const;
    /// Their values as constraints read them, in the model's units.
    std::map<std::string, double> variableValues() const;
    /// Those as the constraint solver asks for them, by name: taken now, and
    /// kept by the command that solves. NaN for a name it has not (one that
    /// is gone, or that the active configuration cannot work out), so the
    /// solver leaves that constraint's value as it is.
    std::function<double(const std::string&)> variableResolver() const;

    ExpressionEngine& expressionEngine() { return m_parameterRegistry.engine(); }
    const ExpressionEngine& expressionEngine() const { return m_parameterRegistry.engine(); }

    // --- Sketch management ---

    void addSketch(std::shared_ptr<Sketch> sketch);
    /// Take the sketch out of the document. If it was being edited, editing
    /// stops.
    std::shared_ptr<Sketch> removeSketch(uint64_t sketchId);
    const std::vector<std::shared_ptr<Sketch>>& sketches() const { return m_sketches; }
    std::vector<std::shared_ptr<Sketch>>& sketches() { return m_sketches; }
    /// The sketch with @p sketchId, or null.
    std::shared_ptr<Sketch> findSketch(uint64_t sketchId) const;

    // --- The drawing being edited ---

    /// Edit @p sketch (one of sketches()): activeDrawing() and
    /// activeConstraints() become its own. Null stops editing. Editing is a
    /// mode of the window, not a change to the document: it is not undone.
    void editSketch(std::shared_ptr<Sketch> sketch);
    /// The sketch being edited, or null.
    const std::shared_ptr<Sketch>& editedSketch() const { return m_editedSketch; }

    /// What the drawing tools draw into and edit: the sketch being edited, in
    /// its plane's coordinates, or else the top-level drawing. File formats,
    /// plotting and export use draftDocument(), the top level, whatever is
    /// being edited.
    draft::DraftDocument& activeDrawing() {
        return m_editedSketch ? m_editedSketch->drawing() : m_draftDoc;
    }
    const draft::DraftDocument& activeDrawing() const {
        return m_editedSketch ? m_editedSketch->drawing() : m_draftDoc;
    }
    /// Every drawing of the document: the top level, then each sketch's.
    /// What they share (the layers) is changed in all of them.
    std::vector<draft::DraftDocument*> drawings();

    /// The constraints of activeDrawing().
    cstr::ConstraintSystem& activeConstraints() {
        return m_editedSketch ? m_editedSketch->constraintSystem() : m_constraintSystem;
    }
    const cstr::ConstraintSystem& activeConstraints() const {
        return m_editedSketch ? m_editedSketch->constraintSystem() : m_constraintSystem;
    }

    // --- Feature tree (parametric history) ---

    FeatureTree& featureTree() { return m_featureTree; }
    const FeatureTree& featureTree() const { return m_featureTree; }

    // --- Built model (result of replaying the feature tree) ---

    /// Rebuild the solid by replaying the feature tree.
    /// Stores the result (and failure diagnostics) on the document.
    /// Returns true when no feature failed. An empty tree succeeds
    /// with a null solid.
    bool rebuildModel();

    /// Each feature's parameter expressions worked out against the
    /// document's variables, and its parameters set to them (Phase 155).
    /// One that cannot be (a missing variable, a length where an angle
    /// goes) is its feature's expressionError, where the build fails.
    void applyExpressions();

    /// The part built, its expressions first worked out: what
    /// rebuildModel() applies, and a worker builds from a snapshot.
    BuildResult buildWithDiagnostics(BuildControl* control = nullptr);

    /// Take the result of a build made elsewhere (a worker's, from a
    /// snapshot of this document), as rebuildModel() takes its own. A
    /// cancelled build leaves the model as it was. Returns true when no
    /// feature failed.
    bool applyBuild(BuildResult result);

    /// The solid produced by the last rebuildModel() call (may be null).
    const topo::Solid* solid() const { return m_solid.get(); }
    topo::Solid* solid() { return m_solid.get(); }

    /// Take ownership of the built solid (e.g. loaded from a cache): a new
    /// solid, counted as a build.
    void setSolid(std::unique_ptr<topo::Solid> solid) {
        m_solid = std::move(solid);
        ++m_builds;
    }

    /// How many builds have been applied (rebuildModel or applyBuild, not
    /// cancelled): what is made from the solid, its mesh, knows it is new.
    std::uint64_t builds() const { return m_builds; }
    /// This document object, unique in the process: two reads of one file
    /// are two serials, where their build counts may be equal. With
    /// builds(), it says which solid a mesh was made from.
    std::uint64_t serial() const { return m_serial; }
    /// Failure message from the last rebuildModel() call (empty on success).
    const std::string& lastBuildMessage() const { return m_lastBuildMessage; }

    /// Index of the feature that failed in the last rebuild (-1 = none).
    int failedFeatureIndex() const { return m_failedFeatureIndex; }

    /// Whether the model has not been built since the feature tree last
    /// changed. A build that failed counts: its partial solid and its failure
    /// stand until the tree changes, rather than being built again.
    bool needsBuild() const {
        return m_featureTree.featureCount() > 0 &&
               (!m_built || m_builtRevision != m_featureTree.revision());
    }

    // --- Document type ---

    DocumentType type() const { return m_type; }
    void setType(DocumentType type) { m_type = type; }

    // --- Dirty tracking ---

    /// True when the document differs from what was last saved (or loaded):
    /// either the undo stack has moved away from its clean state, or a change
    /// that is not an undoable command was marked with setDirty(true).
    bool isDirty() const;

    /// setDirty(false) records the current state as saved — it also marks the
    /// undo stack clean, so undoing past this point makes the document
    /// modified again. setDirty(true) marks a change made outside the undo
    /// stack; it stays set until the next setDirty(false).
    void setDirty(bool dirty);

    /// Called whenever the modified state may have changed: after every undo
    /// stack change and every setDirty() call. Replaces the undo stack's own
    /// change callback.
    void setChangeCallback(std::function<void()> callback);

    // --- File path ---

    const std::string& filePath() const { return m_filePath; }
    void setFilePath(const std::string& path) { m_filePath = path; }

    // --- Unit (Phase 154) ---

    /// The unit its lengths are shown and typed in, saved with it. The model
    /// is in millimetres whatever it is.
    math::LengthUnit lengthUnit() const { return m_lengthUnit; }
    void setLengthUnit(math::LengthUnit unit) { m_lengthUnit = unit; }

private:
    draft::DraftDocument m_draftDoc;
    draft::LayerManager m_layerManager;
    cstr::ConstraintSystem m_constraintSystem;
    ParameterRegistry m_parameterRegistry;
    ConfigurationTable m_configurations;
    std::unique_ptr<UndoStack> m_undoStack;
    bool m_dirty = false;
    std::function<void()> m_onChange;
    std::string m_filePath;
    std::vector<std::shared_ptr<Sketch>> m_sketches;
    std::shared_ptr<Sketch> m_editedSketch;
    FeatureTree m_featureTree;
    std::unique_ptr<topo::Solid> m_solid;
    std::string m_lastBuildMessage;
    int m_failedFeatureIndex = -1;
    std::uint64_t m_builds = 0;
    std::uint64_t m_serial = 0;
    bool m_built = false;          ///< a build has been applied...
    uint64_t m_builtRevision = 0;  ///< ...for this revision of the feature tree
    DocumentType m_type = DocumentType::Drawing;
    math::LengthUnit m_lengthUnit = math::LengthUnit::Millimetre;
};

}  // namespace hz::doc
