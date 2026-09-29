#pragma once

#include <QString>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Vec3.h"
#include "horizon/topology/TopologyID.h"
#include "horizon/ui/ViewportWidget.h"

class QComboBox;
class QListWidget;

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::ui {

class FeatureForm;

// What a command's form offers to choose on the part: its edges and faces,
// the planes of its flat faces, and the six axis directions; and which of
// them was clicked in the view. The part commands and the window's sketch
// and feature-editing commands share them. Their words are the window's to
// translate (hz::ui::MainWindow), where they were first.

/// Edges or faces of the part to choose from in a dialog, until the viewport
/// can pick them: each one's name, and how it is listed ({text, tooltip}).
struct PickList {
    std::vector<topo::TopologyID> ids;
    std::vector<std::pair<QString, QString>> items;
};

/// The part's edges, listed by their end points: a curve in chords once, as
/// the curve (Stable names), and the seams between the facets of a curved
/// face, which are no edge of the part, not at all.
PickList edgesOf(const topo::Solid& solid);

/// The part's faces, listed by which way they face and where their middle is.
PickList facesOf(const topo::Solid& solid);

/// Check, in a list of @p picks, the part's own edges (@p edges) or faces
/// chosen by clicking in the viewport, so a command offers what was clicked.
void checkClicked(QListWidget* list, const PickList& picks,
                  const std::vector<ViewportWidget::ModelPick>& clicked, bool edges);

/// How a feature's parameter or vector is labelled in its edit form.
QString parameterLabel(const std::string& name);

/// A plane to sketch on, and how it is listed.
struct PlaneChoice {
    QString text;
    draft::SketchPlane plane;
    std::string tag;  ///< the face's persistent name
};

/// The part's flat faces as planes to sketch on: through the middle of the
/// face, facing out of the part, x along the world's x (or y, for a face
/// that faces along x). A face that is not flat (a loft's or a fillet's can
/// be twisted) is left out.
std::vector<PlaneChoice> planarFacesOf(const topo::Solid& solid);

/// The part's flat faces, each once (by its whole name: the pieces of a face
/// a Boolean split are one), for a command that works on a face by its name
/// (Phase 162: a hole drilled into one, a mirror in one).
struct FlatFace {
    QString text;
    math::Vec3 middle;
    math::Vec3 normal;  ///< out of the part
    std::string name;   ///< its whole name
};
std::vector<FlatFace> flatFacesOf(const topo::Solid& solid);

/// Which of @p faces was clicked first in @p picks; @p otherwise when none.
int clickedFlatFace(const std::vector<FlatFace>& faces,
                    const std::vector<ViewportWidget::ModelPick>& picks, int otherwise);

/// The way the first face or edge clicked on the part points: the face's
/// outward normal, or the edge from its first end to its second.
std::optional<math::Vec3> clickedDirection(const topo::Solid& solid,
                                           const std::vector<ViewportWidget::ModelPick>& picks);

/// A choice of the six axis directions in @p form, @p initial ("+Z")
/// chosen, for a pull, a pattern or an axis.
QComboBox* directionChoice(FeatureForm& form, const QString& name, const QString& label,
                           const QString& initial);
/// The direction chosen in a directionChoice().
math::Vec3 chosenDirection(const QComboBox* combo);

}  // namespace hz::ui
