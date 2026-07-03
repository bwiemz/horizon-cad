#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/document/ExpressionEngine.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/ParameterRegistry.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/Layer.h"

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

    ExpressionEngine& expressionEngine() { return m_parameterRegistry.engine(); }
    const ExpressionEngine& expressionEngine() const { return m_parameterRegistry.engine(); }

    // --- Sketch management ---

    void addSketch(std::shared_ptr<Sketch> sketch);
    std::shared_ptr<Sketch> removeSketch(uint64_t sketchId);
    const std::vector<std::shared_ptr<Sketch>>& sketches() const { return m_sketches; }
    std::vector<std::shared_ptr<Sketch>>& sketches() { return m_sketches; }

    /// The default sketch (XY plane at origin). Always exists.
    Sketch& defaultSketch() { return *m_defaultSketch; }
    const Sketch& defaultSketch() const { return *m_defaultSketch; }

    // --- Feature tree (parametric history) ---

    FeatureTree& featureTree() { return m_featureTree; }
    const FeatureTree& featureTree() const { return m_featureTree; }

    // --- Built model (result of replaying the feature tree) ---

    /// Rebuild the solid by replaying the feature tree.
    /// Stores the result (and failure diagnostics) on the document.
    /// Returns true when no feature failed. An empty tree succeeds
    /// with a null solid.
    bool rebuildModel();

    /// The solid produced by the last rebuildModel() call (may be null).
    const topo::Solid* solid() const { return m_solid.get(); }
    topo::Solid* solid() { return m_solid.get(); }

    /// Take ownership of the built solid (e.g. loaded from a cache).
    void setSolid(std::unique_ptr<topo::Solid> solid) { m_solid = std::move(solid); }

    /// Failure message from the last rebuildModel() call (empty on success).
    const std::string& lastBuildMessage() const { return m_lastBuildMessage; }

    /// Index of the feature that failed in the last rebuild (-1 = none).
    int failedFeatureIndex() const { return m_failedFeatureIndex; }

    // --- Document type ---

    DocumentType type() const { return m_type; }
    void setType(DocumentType type) { m_type = type; }

    // --- Dirty tracking ---

    bool isDirty() const { return m_dirty; }
    void setDirty(bool dirty) { m_dirty = dirty; }

    // --- File path ---

    const std::string& filePath() const { return m_filePath; }
    void setFilePath(const std::string& path) { m_filePath = path; }

private:
    draft::DraftDocument m_draftDoc;
    draft::LayerManager m_layerManager;
    cstr::ConstraintSystem m_constraintSystem;
    ParameterRegistry m_parameterRegistry;
    std::unique_ptr<UndoStack> m_undoStack;
    bool m_dirty = false;
    std::string m_filePath;
    std::vector<std::shared_ptr<Sketch>> m_sketches;
    std::shared_ptr<Sketch> m_defaultSketch;
    FeatureTree m_featureTree;
    std::unique_ptr<topo::Solid> m_solid;
    std::string m_lastBuildMessage;
    int m_failedFeatureIndex = -1;
    DocumentType m_type = DocumentType::Drawing;
};

}  // namespace hz::doc
