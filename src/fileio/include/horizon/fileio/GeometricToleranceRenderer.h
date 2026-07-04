#pragma once

#include <memory>

namespace hz::draft {
class DraftText;
}  // namespace hz::draft

namespace hz::model {
struct DrawingView;
struct FeatureControlFrame;
struct DatumFeature;
}  // namespace hz::model

namespace hz::io {

/// Renders model-driven GD&T annotations onto a drawing view as draftable text,
/// which the DXF writer emits as TEXT entities on export.
///
/// Each annotation is anchored to a model feature by TopologyID (the same
/// associative basis as dimensions). The feature is located in the view's
/// projected geometry and the annotation is placed offset from that edge's 2D
/// projection, so it tracks the feature as the model is regenerated.
class GeometricToleranceRenderer {
public:
    /// A feature control frame (e.g. "⎶ 0.05 | A | B") placed offset from the
    /// toleranced edge as it appears in @p view. Returns nullptr if the frame's
    /// feature does not appear in the view.
    static std::shared_ptr<draft::DraftText> render(const model::DrawingView& view,
                                                    const model::FeatureControlFrame& frame,
                                                    double offset);

    /// A datum feature symbol (e.g. "[A]") placed offset from the datum feature's
    /// edge as it appears in @p view. Returns nullptr if the feature does not
    /// appear in the view.
    static std::shared_ptr<draft::DraftText> render(const model::DrawingView& view,
                                                    const model::DatumFeature& datum,
                                                    double offset);
};

}  // namespace hz::io
