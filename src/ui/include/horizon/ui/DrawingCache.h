#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "horizon/drafting/DimensionStyle.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec2.h"

namespace hz::cstr {
struct DOFAnalysis;
}  // namespace hz::cstr

namespace hz::doc {
class Document;
}  // namespace hz::doc

namespace hz::draft {
class DraftDocument;
}  // namespace hz::draft

namespace hz::render {
class SelectionManager;
}  // namespace hz::render

namespace hz::ui {

/// What the 2D view draws of a drawing: its lines, circles and arcs as line
/// vertices, batched by pen, and its text. Built once and kept while nothing
/// it was built from changes, so a frame that only moves the view or the
/// cursor draws what is already built.
///
/// The drawing is cut into chunks, a grid over its extent, and each batch
/// keeps its vertices in chunk order: a frame draws only the runs of the
/// chunks it can see (visibleChunks, visibleRanges).
class DrawingCache {
public:
    /// How a line is drawn: lines drawn alike go in one batch.
    struct Pen {
        uint32_t color = 0;
        float width = 1.0f;
        int lineType = 1;
        bool operator==(const Pen&) const = default;
    };

    /// A run of a batch's vertices, counted in vertices, not floats.
    struct Range {
        std::uint32_t first = 0;
        std::uint32_t count = 0;
        bool operator==(const Range&) const = default;
    };

    /// Every line drawn with one pen. Vertices are (x, y, z, distance
    /// along the line), two per segment, in chunk order.
    struct Batch {
        Pen pen;
        std::vector<float> vertices;
        /// For each chunk with lines in this batch, in order: its chunk and
        /// its run of vertices. The runs follow one another.
        std::vector<std::pair<std::uint32_t, Range>> runs;
    };

    /// A text drawn over the view: dimension values and text entities.
    struct Text {
        math::Vec2 position;
        std::string text;
        uint32_t color = 0;
        double height = 0.0;  ///< 0 = the dimension style's
        double rotation = 0.0;
        int alignment = 1;  ///< 0 left, 1 centre, 2 right
    };

    /// Build again if anything the cache was built from changed: the
    /// drawing (another one, or a change to it), the document's history,
    /// the selection, the constraint analysis behind the DOF colours
    /// (@p dofRevision moves with it), the layers or the dimension style.
    /// Returns whether it built.
    bool update(const doc::Document& doc, const render::SelectionManager& selection,
                const cstr::DOFAnalysis& dof, std::uint64_t dofRevision);

    /// Build now, from what is given, whatever was built before.
    void build(const doc::Document& doc, const render::SelectionManager& selection,
               const cstr::DOFAnalysis& dof);

    /// Forget what was built: the next update() builds.
    void clear();

    const std::vector<Batch>& batches() const { return m_batches; }
    const std::vector<Text>& texts() const { return m_texts; }
    /// Each chunk's extent, in the drawing's coordinates (z = 0).
    const std::vector<math::BoundingBox>& chunkBounds() const { return m_chunkBounds; }
    /// How many times it has been built: moves with every build, so what is
    /// made from it (the GPU's copy, the text overlay) knows to follow.
    std::uint64_t builds() const { return m_builds; }

    /// Which chunks can be seen through @p viewProjection: all but those
    /// wholly outside one side of the view.
    std::vector<bool> visibleChunks(const math::Mat4& viewProjection) const;
    /// The runs of @p batch in the chunks marked @p visible, runs that follow
    /// one another joined into one.
    static std::vector<Range> visibleRanges(const Batch& batch, const std::vector<bool>& visible);

private:
    /// What a build was made from, to tell whether it still stands.
    struct Stamp {
        const doc::Document* document = nullptr;
        const draft::DraftDocument* drawing = nullptr;
        std::uint64_t drawingRevision = 0;
        std::uint64_t history = 0;
        std::uint64_t selection = 0;
        std::uint64_t dof = 0;
        bool operator==(const Stamp&) const = default;
    };
    Stamp stampOf(const doc::Document& doc, const render::SelectionManager& selection,
                  std::uint64_t dofRevision) const;

    std::vector<Batch> m_batches;
    std::vector<Text> m_texts;
    std::vector<math::BoundingBox> m_chunkBounds;
    std::uint64_t m_builds = 0;

    bool m_built = false;
    Stamp m_stamp;
    // Small enough to compare every frame; handed out by mutable pointer, so
    // a change to them leaves no other trace.
    draft::LayerManager m_layers;
    draft::DimensionStyle m_style;
};

}  // namespace hz::ui
