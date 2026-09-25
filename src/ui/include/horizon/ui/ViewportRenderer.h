#pragma once

#include <QImage>
#include <QPointF>
#include <QSize>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "horizon/constraint/SketchSolver.h"
#include "horizon/math/Vec2.h"
#include "horizon/render/GLRenderer.h"
#include "horizon/ui/DrawingCache.h"
#include "horizon/ui/ViewCube.h"

class QOpenGLExtraFunctions;

namespace hz::render {
class Camera;
class GLRenderer;
class SelectionManager;
}  // namespace hz::render

namespace hz::doc {
class Document;
}  // namespace hz::doc

namespace hz::draft {
class DraftDocument;
}  // namespace hz::draft

namespace hz::ui {

class Tool;

/// Handles all rendering logic for the viewport: entity batching, GL draw calls,
/// text overlay, constraint annotations, DOF visualization, grips, and tool preview.
/// Extracted from ViewportWidget to keep the widget class focused on coordination.
class ViewportRenderer {
public:
    ViewportRenderer() = default;

    /// Initialize GL resources for text overlay (shader, VAO, VBO, texture).
    void initTextOverlayGL(QOpenGLExtraFunctions* gl);

    /// Clean up GL resources.
    void destroyGL(QOpenGLExtraFunctions* gl);

    /// Draw the document's drawing: what the drawing cache holds, built again
    /// only when the drawing changed, and only the chunks in view.
    void renderEntities(QOpenGLExtraFunctions* gl, render::GLRenderer& renderer,
                        const render::Camera& camera, doc::Document& doc,
                        const render::SelectionManager& selection);

    /// Render tool preview (rubber-band lines, circles, arcs, selection rectangle).
    void renderToolPreview(QOpenGLExtraFunctions* gl, render::GLRenderer& renderer,
                           const render::Camera& camera, Tool* activeTool);

    /// Render grip squares on selected entities.
    void renderGrips(QOpenGLExtraFunctions* gl, render::GLRenderer& renderer,
                     const render::Camera& camera, doc::Document& doc,
                     const render::SelectionManager& selection, double pixelToWorldScale);

    /// Bring the drawing cache up to date with @p doc (the CPU half of
    /// renderEntities). Returns whether it was built again.
    bool prepareEntities(const doc::Document& doc, const render::SelectionManager& selection);
    const DrawingCache& drawingCache() const { return m_drawing; }

    /// Paint the text overlay (texts, constraint annotations, the view cube)
    /// into its image, if anything it shows changed since the last time:
    /// the view, its size, or the drawing cache. Returns whether it painted.
    bool prepareTextOverlay(const render::Camera& camera, doc::Document* doc,
                            const render::SelectionManager& selection, int viewportWidth,
                            int viewportHeight, double pixelToWorldScale,
                            qreal devicePixelRatio = 1.0);
    /// How many times the overlay has been painted (for tests).
    std::uint64_t overlayPaints() const { return m_overlayPaints; }

    /// Draw the text overlay (prepareTextOverlay) over the view: one
    /// full-view quad, its texture sent to the GPU only when it was painted
    /// again. The image is drawn at @p devicePixelRatio, so text is sharp on
    /// a high-DPI screen; @p viewportWidth and @p viewportHeight are logical.
    void blitTextOverlay(QOpenGLExtraFunctions* gl, const render::Camera& camera,
                         doc::Document* doc, const render::SelectionManager& selection,
                         int viewportWidth, int viewportHeight, double pixelToWorldScale,
                         qreal devicePixelRatio = 1.0);

    /// Recompute DOF analysis from the document's constraint system, when the
    /// document or its undo history changed since the last one: it runs the
    /// constraint solver, and a frame is drawn on every mouse move.
    void recomputeDOF(doc::Document* doc);
    /// Whether recomputeDOF(@p doc) would run the analysis: the document,
    /// its history or its drawing changed since the last one.
    bool dofStale(const doc::Document* doc) const;

    /// Make the next recomputeDOF() run, whatever it is given.
    void invalidateDOF() { m_dofDirty = true; }

    /// How many analyses have run (for tests).
    std::uint64_t dofComputations() const { return m_dofComputations; }

    /// Access current DOF analysis.
    const cstr::DOFAnalysis& dofAnalysis() const { return m_dofAnalysis; }

    /// Access the orientation gizmo (for click hit-testing from the widget).
    ViewCube& viewCube() { return m_viewCube; }

private:
    // What the view draws of the drawing, and the GPU's copy of its batches:
    // a buffer for each, as of the cache's build m_uploadedBuild.
    DrawingCache m_drawing;
    std::vector<render::GLRenderer::LineBuffer> m_lineBuffers;
    std::uint64_t m_uploadedBuild = 0;

    // The text overlay as last painted, and what it was painted from.
    struct OverlayStamp {
        std::array<double, 16> viewProjection{};
        int width = 0;
        int height = 0;
        double ratio = 0.0;
        double pixelToWorld = 0.0;
        const doc::Document* document = nullptr;
        std::uint64_t drawing = 0;
        bool operator==(const OverlayStamp&) const = default;
    };
    QImage m_overlayImage;
    OverlayStamp m_overlayStamp;
    bool m_overlayPainted = false;
    std::uint64_t m_overlayPaints = 0;
    bool m_overlayUploaded = false;  ///< the texture holds the image as painted
    QSize m_overlayTextureSize;

    /// Generate vertices for a circle approximation.
    std::vector<float> circleVertices(const math::Vec2& center, double radius,
                                      int segments = 64) const;

    /// Generate vertices for an arc (partial circle).
    std::vector<float> arcVertices(const math::Vec2& center, double radius, double startAngle,
                                   double endAngle, int segments = 64) const;

    /// Render text to a QImage using QPainter.
    void renderTextToImage(QImage& image, const render::Camera& camera, doc::Document* doc,
                           const render::SelectionManager& selection, int viewportWidth,
                           int viewportHeight, double pixelToWorldScale);

    /// Project a world-space 2D point to screen coordinates.
    static QPointF worldToScreen(const render::Camera& camera, const math::Vec2& wp,
                                 int viewportWidth, int viewportHeight);

    // DOF visualization
    cstr::DOFAnalysis m_dofAnalysis;
    bool m_dofDirty = true;
    const doc::Document* m_dofDocument = nullptr;        ///< what the analysis is of
    std::uint64_t m_dofRevision = 0;                     ///< its undo revision then
    const draft::DraftDocument* m_dofDrawing = nullptr;  ///< ...and drawing
    std::uint64_t m_dofComputations = 0;

    // Top-right orientation gizmo, drawn in the text-overlay QImage.
    ViewCube m_viewCube;

    // Text overlay GL resources (renders to QImage, uploads as texture)
    unsigned int m_textOverlayTex = 0;
    unsigned int m_textOverlayVAO = 0;
    unsigned int m_textOverlayVBO = 0;
    unsigned int m_textOverlayShader = 0;
};

}  // namespace hz::ui
