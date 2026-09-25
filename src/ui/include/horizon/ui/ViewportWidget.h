#pragma once

#include <QOpenGLWidget>
#include <QPoint>
#include <QPointF>
#include <QWidget>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "horizon/document/Sketch.h"
#include "horizon/drafting/SnapEngine.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/math/Vec4.h"
#include "horizon/render/Camera.h"
#include "horizon/render/DisplayMode.h"
#include "horizon/render/SceneGraph.h"
#include "horizon/render/SelectionManager.h"
#include "horizon/ui/OverlayRenderer.h"
#include "horizon/ui/TypedPoint.h"
#include "horizon/ui/ViewportInputHandler.h"
#include "horizon/ui/ViewportRenderer.h"

class QOpenGLExtraFunctions;
class QImage;

namespace hz::render {
class GLRenderer;
}  // namespace hz::render

namespace hz::doc {
class Document;
}  // namespace hz::doc

namespace hz::ui {

class Tool;

/// Saved camera state for restoring after sketch editing.
struct CameraState {
    math::Vec3 eye;
    math::Vec3 target;
    math::Vec3 up;
};

/// Drafting aids (the status bar's toggles, F3/F8/F9/F10).
struct DraftingAids {
    bool objectSnap = true;    ///< snap to points on entities
    bool gridSnap = true;      ///< snap to the grid when no entity is near
    bool ortho = false;        ///< hold the direction from the last point to 0/90/180/270°
    bool polar = false;        ///< hold it to the nearest multiple of polarAngle
    double polarAngle = 15.0;  ///< polar tracking's step, in degrees
};

/// The main 2D/3D viewport widget backed by OpenGL.
///
/// Provides camera navigation (orbit, pan, zoom) and delegates left-click
/// interaction to the currently active Tool.
class ViewportWidget : public QOpenGLWidget {
    Q_OBJECT

public:
    explicit ViewportWidget(QWidget* parent = nullptr);
    ~ViewportWidget() override;

    /// Empty while the viewport can draw; otherwise why it cannot (no OpenGL
    /// 3.3 context, or the shaders failed to compile). Known after the first
    /// show; the user is told once.
    const QString& graphicsProblem() const { return m_graphicsProblem; }
    /// The viewport has drawn a whole frame with OpenGL 3.3: a context, its
    /// shaders, and a paint that reached the end. (A context alone is not
    /// enough: some platforms give one and then have nothing to draw into.)
    bool hasDrawn() const { return m_framesDrawn > 0; }

    // ---- Document ----

    void setDocument(doc::Document* doc);
    doc::Document* document() const { return m_document; }

    // ---- Camera ----

    render::Camera& camera() { return m_camera; }
    const render::Camera& camera() const { return m_camera; }

    // ---- Selection ----

    render::SelectionManager& selectionManager() { return m_selectionManager; }
    const render::SelectionManager& selectionManager() const { return m_selectionManager; }

    // ---- Scene Graph (3D) ----

    render::SceneGraph& sceneGraph() { return m_sceneGraph; }
    const render::SceneGraph& sceneGraph() const { return m_sceneGraph; }

    // ---- Snapping ----

    draft::SnapEngine& snapEngine() { return m_snapEngine; }
    void setLastSnapResult(const draft::SnapResult& result) { m_lastSnapResult = result; }

    /// How near the cursor, on screen, a snap (by default) or a pick reaches.
    static constexpr double kSnapPixels = 10.0;
    static constexpr double kPickPixels = 10.0;

    /// How far a snap reaches on screen (Edit ▸ Preferences).
    double snapPixels() const { return m_snapPixels; }
    void setSnapPixels(double pixels) {
        if (pixels > 0.0) m_snapPixels = pixels;
    }

    /// The snap for a cursor at @p worldPos: to what is drawn on visible,
    /// unlocked layers, within snapPixels() of it on screen at any zoom; else
    /// the grid. Then, with ortho or polar tracking on and no entity snapped
    /// to, the direction from the active tool's base point is held to the
    /// nearest allowed angle. While a typed point is being placed, that point
    /// exactly.
    draft::SnapResult snap(const math::Vec2& worldPos);

    /// The drafting aids in force.
    const DraftingAids& draftingAids() const { return m_aids; }
    void setDraftingAids(const DraftingAids& aids);

    // ---- Typed input ----

    /// What is being typed for the active tool's next point.
    TypedPoint& typedPoint() { return m_typedPoint; }
    /// A value is part way typed: a point, or one a tool reads itself. Its
    /// keys go to it, before the window's one-key shortcuts (Phase 154).
    bool typingText() const;
    const TypedPoint& typedPoint() const { return m_typedPoint; }

    /// Give @p point to the active tool as a click there, without snapping
    /// or tracking. True when the tool used it.
    bool applyTypedPoint(const math::Vec2& point);

    /// Where the cursor last was over the viewport, in world coordinates.
    const std::optional<math::Vec2>& cursorWorld() const { return m_cursorWorld; }
    void setCursorWorld(const math::Vec2& world) { m_cursorWorld = world; }

    /// The world distance @p pixels screen pixels span at the current zoom:
    /// how far a click reaches to pick an entity, the same on screen at any
    /// zoom.
    double pickTolerance(double pixels = kPickPixels) const { return pixels * pixelToWorldScale(); }

    // ---- Tools ----

    /// Set the active tool.  The viewport does NOT take ownership.
    void setActiveTool(Tool* tool);

    /// Returns the active tool, or nullptr.
    Tool* activeTool() const { return m_activeTool; }

    // ---- Active Sketch ----

    /// Show @p sketch being edited: the view then works in the sketch's own
    /// coordinates (its plane is the view's XY, where the drawing tools draw)
    /// and looks straight down on it; the camera is saved first. The window
    /// places the solids in that frame. Passing nullptr restores the saved
    /// camera. Either way the selection is cleared.
    void setActiveSketch(doc::Sketch* sketch);

    /// Returns the currently active sketch, or nullptr.
    doc::Sketch* activeSketch() const { return m_activeSketch; }

    // ---- Overlay ----

    /// Access the overlay renderer (crosshair, snap markers, axis indicator).
    OverlayRenderer& overlayRenderer() { return m_overlayRenderer; }

    // ---- Coordinate helpers ----

    /// Project a screen-space position to the world XY plane (Z = 0).
    math::Vec2 worldPositionAtCursor(int screenX, int screenY) const;

    /// Returns the world-space distance that corresponds to one pixel at the current zoom.
    double pixelToWorldScale() const;

    /// Project a world-space 2D point to screen coordinates.
    QPointF worldToScreen(const math::Vec2& wp) const;
    /// Project a world point to screen coordinates (Qt's, 0 at the top).
    QPointF projectToScreen(const math::Vec3& world) const;

    // ---- Seeing the part (Phase 135) ----

    /// Orthographic or perspective, kept across resizes: a resize used to
    /// put the camera back to perspective. Switching keeps what is framed.
    void setOrthographic(bool orthographic);
    bool isOrthographic() const { return m_orthographic; }
    /// Give the camera the projection for a @p w by @p h view.
    void applyProjection(int w, int h);
    /// How solids are drawn: shaded, with their edges, or as wireframe.
    void setDisplayMode(render::DisplayMode mode);
    render::DisplayMode displayMode() const { return m_displayMode; }
    /// A section plane (normal, offset: points with n.p + w < 0 are cut
    /// away), or none.
    void setSectionPlane(const std::optional<math::Vec4>& plane);
    const std::optional<math::Vec4>& sectionPlane() const { return m_sectionPlane; }

    // ---- The part in 3D (Phase 132) ----

    /// A face or an edge of a solid in the scene, by its persistent name.
    struct ModelPick {
        uint64_t owner = 0;  ///< the scene node's ownerId: a component's id, or 0
        std::string tag;     ///< the face's or edge's TopologyID tag; empty: all of it
        bool edge = false;
        bool operator==(const ModelPick&) const = default;
    };
    /// What is under the screen point @p at: an edge drawn within the pick
    /// distance and not hidden, else the face the ray through it meets first.
    /// Nothing while a sketch is edited: clicks there are the sketch's.
    std::optional<ModelPick> pickModel(const QPointF& at) const;
    /// The faces and edges chosen by clicking, in the order chosen.
    const std::vector<ModelPick>& modelSelection() const { return m_modelSelection; }
    /// A click on @p pick: with @p add (Shift), it is added, or taken out if
    /// it was chosen; otherwise it alone is chosen. A click on nothing
    /// without @p add clears the choice.
    void chooseModel(const std::optional<ModelPick>& pick, bool add);
    void clearModelSelection();
    /// What the cursor is over, drawn highlighted.
    void setModelHover(const std::optional<ModelPick>& pick);
    const std::optional<ModelPick>& modelHover() const { return m_modelHover; }

signals:
    /// Emitted when the mouse moves.  Carries the world-space position on the XY plane.
    void mouseMoved(const hz::math::Vec2& worldPos);

    /// Emitted when the selection changes.
    void selectionChanged();

    /// Emitted when the faces and edges chosen in 3D change.
    void modelSelectionChanged();

    /// Emitted when what is typed for the active tool changes, or is taken
    /// or refused: the prompt shows it.
    void typedInputChanged();

protected:
    // QOpenGLWidget overrides
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void showEvent(QShowEvent* event) override;

    // Input events
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
    /// Tab goes to the active tool first (the dimension tools cycle their
    /// kind with it): QWidget spends it on moving focus before
    /// keyPressEvent sees it. A tool that does not take it leaves it to
    /// move focus, as before.
    bool event(QEvent* event) override;

private:
    /// Record a context Qt could not create at all, and tell the user once.
    void checkGraphics();

    DraftingAids m_aids;
    TypedPoint m_typedPoint;
    /// Set while applyTypedPoint() hands the tool its point: snap() returns it.
    std::optional<math::Vec2> m_typedOverride;
    std::optional<math::Vec2> m_cursorWorld;

    QString m_graphicsProblem;
    /// initializeGL() got an OpenGL 3.3 context and built the renderer:
    /// paintGL() and resizeGL() may use it.
    bool m_glReady = false;
    std::uint64_t m_framesDrawn = 0;  ///< whole frames painted with OpenGL 3.3
    bool m_dofQueued = false;         ///< the DOF analysis is to run after this paint
    bool m_graphicsCheckScheduled = false;
    bool m_graphicsProblemReported = false;
    /// Snap the camera to the standard view requested by a view-cube click.
    void applyViewCubeRegion(ViewCube::Region region);

    /// True while a left-press was consumed by the view cube, so the matching
    /// release is swallowed instead of reaching the active tool (which would
    /// otherwise pick/clear the selection at the release point).
    bool m_viewCubeCapturedPress = false;

    // Camera
    render::Camera m_camera;

    // Renderer
    std::unique_ptr<render::GLRenderer> m_renderer;

    // Document (non-owning)
    doc::Document* m_document = nullptr;

    // Selection
    render::SelectionManager m_selectionManager;

    // Snapping
    draft::SnapEngine m_snapEngine;
    /// The last finite, positive pixelToWorldScale(): what a viewport with no
    /// size reports.
    mutable double m_lastPixelScale = 0.01;
    double m_snapPixels = kSnapPixels;
    draft::SnapResult m_lastSnapResult;

    // 3D scene graph
    render::SceneGraph m_sceneGraph;
    std::vector<ModelPick> m_modelSelection;
    std::optional<ModelPick> m_modelHover;
    bool m_orthographic = false;
    render::DisplayMode m_displayMode = render::DisplayMode::ShadedWithEdges;
    std::optional<math::Vec4> m_sectionPlane;

    /// Draw the chosen and hovered faces and edges over the solids.
    void drawModelHighlights(QOpenGLExtraFunctions* gl);
    /// Draw the part's datum planes, axes and points (construction
    /// geometry, not the solid).
    void drawDatums(QOpenGLExtraFunctions* gl);

    // Active tool
    Tool* m_activeTool = nullptr;

    // Active sketch (non-owning)
    doc::Sketch* m_activeSketch = nullptr;

    // Saved camera state for restore on sketch exit
    std::optional<CameraState> m_savedCameraState;

    // GL overlay renderer (crosshair, snap markers, axis indicator)
    OverlayRenderer m_overlayRenderer;

    // Extracted input handler (pan, orbit, zoom, tool dispatch)
    ViewportInputHandler m_inputHandler;

    // Extracted renderer (entity batching, text overlay, grips, DOF, tool preview)
    ViewportRenderer m_viewportRenderer;
};

}  // namespace hz::ui
