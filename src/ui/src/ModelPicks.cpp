#include "horizon/ui/ModelPicks.h"

#include <QComboBox>
#include <QCoreApplication>
#include <QListWidget>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "horizon/modeling/FacePlane.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/FeatureForm.h"

namespace hz::ui {

PickList edgesOf(const topo::Solid& solid) {
    PickList list;
    std::map<std::string, size_t> rowOf;
    std::vector<int> pieces;
    for (const auto& edge : solid.edges()) {
        const topo::HalfEdge* he = edge.halfEdge;
        if (!edge.topoId.isValid() || !he || !he->origin || !he->next || !he->next->origin) {
            continue;
        }
        if (edge.topoId.tag().find("/seam:") != std::string::npos) continue;
        const std::string logical = model::logicalEdge(edge.topoId.tag());
        const auto found = rowOf.find(logical);
        if (found != rowOf.end()) {
            ++pieces[found->second];
            continue;
        }
        rowOf.emplace(logical, list.ids.size());
        pieces.push_back(1);
        list.ids.push_back(topo::TopologyID::fromTag(logical));
        list.items.emplace_back(formatPoint(he->origin->point) + QStringLiteral(" – ") +
                                    formatPoint(he->next->origin->point),
                                QString::fromStdString(logical));
    }
    for (size_t row = 0; row < pieces.size(); ++row) {
        if (pieces[row] < 2) continue;
        auto& text = list.items[row].first;
        text = QCoreApplication::translate("hz::ui::MainWindow", "a curve of %1 pieces, from %2")
                   .arg(pieces[row])
                   .arg(text.section(QStringLiteral(" – "), 0, 0));
    }
    return list;
}

PickList facesOf(const topo::Solid& solid) {
    PickList list;
    const double outward = model::outwardSign(solid);
    struct Curved {
        int facets = 0;
        math::Vec3 centre;  ///< the sum of its facets' middles
    };
    std::map<std::string, size_t> rowOf;
    std::map<size_t, Curved> curvedRows;
    for (const auto& face : solid.faces()) {
        if (!face.topoId.isValid() || !face.outerLoop || !face.outerLoop->halfEdge) continue;
        // Newell's normal and the vertex average of the outer loop.
        math::Vec3 normal, centre;
        int count = 0;
        const topo::HalfEdge* start = face.outerLoop->halfEdge;
        const topo::HalfEdge* he = start;
        do {
            if (!he->origin || !he->next || !he->next->origin) break;
            const math::Vec3& a = he->origin->point;
            const math::Vec3& b = he->next->origin->point;
            normal.x += (a.y - b.y) * (a.z + b.z);
            normal.y += (a.z - b.z) * (a.x + b.x);
            normal.z += (a.x - b.x) * (a.y + b.y);
            centre = centre + a;
            ++count;
            he = he->next;
        } while (he && he != start && count < 100000);
        if (count == 0 || normal.length() < 1e-12) continue;
        // A curved face in facets once, as the face (Stable names).
        const std::string logical = model::logicalFace(face.topoId.tag());
        const auto found = rowOf.find(logical);
        if (found != rowOf.end()) {
            Curved& curved = curvedRows[found->second];
            ++curved.facets;
            curved.centre = curved.centre + centre / count;
            continue;
        }
        rowOf.emplace(logical, list.ids.size());
        curvedRows[list.ids.size()] = Curved{1, centre / count};
        list.ids.push_back(topo::TopologyID::fromTag(logical));
        list.items.emplace_back(
            QCoreApplication::translate("hz::ui::MainWindow", "facing %1 at %2")
                .arg(formatPoint(normal.normalized() * outward), formatPoint(centre / count)),
            QString::fromStdString(logical));
    }
    for (const auto& [row, curved] : curvedRows) {
        if (curved.facets < 2) continue;
        list.items[row].first = QCoreApplication::translate("hz::ui::MainWindow",
                                                            "a curved face of %1 facets, around %2")
                                    .arg(curved.facets)
                                    .arg(formatPoint(curved.centre / curved.facets));
    }
    return list;
}

void checkClicked(QListWidget* list, const PickList& picks,
                  const std::vector<ViewportWidget::ModelPick>& clicked, bool edges) {
    for (size_t row = 0; row < picks.ids.size(); ++row) {
        const std::string& tag = picks.ids[row].tag();
        const bool wasClicked =
            std::any_of(clicked.begin(), clicked.end(), [&](const ViewportWidget::ModelPick& p) {
                return p.owner == 0 && p.edge == edges && p.tag == tag;
            });
        if (!wasClicked) continue;
        if (QListWidgetItem* item = list->item(static_cast<int>(row))) {
            item->setCheckState(Qt::Checked);
        }
    }
}

QString parameterLabel(const std::string& name) {
    static const std::map<std::string, const char*> labels = {
        {"distance", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Distance")},
        {"angle", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Angle")},
        {"segments", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Segments per turn")},
        {"arcSegments", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Segments across the round")},
        {"chordTolerance", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Chord tolerance")},
        {"radius", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Radius")},
        {"thickness", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Thickness")},
        {"operation", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Operation")},
        {"extent", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Goes")},
        {"upToFace", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Up to face")},
        {"count", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Count")},
        {"spacing", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Spacing")},
        {"width", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Width")},
        {"height", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Height")},
        {"depth", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Depth")},
        {"bottomRadius", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Bottom radius")},
        {"topRadius", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Top radius")},
        {"majorRadius", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Ring radius")},
        {"minorRadius", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Tube radius")},
        {"direction", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Direction")},
        {"axisPoint", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Axis through")},
        {"axisDirection", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Axis direction")},
        {"pullDirection", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Pull direction")},
        {"neutralPoint", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Neutral plane through")},
        {"planePoint", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Mirror plane through")},
        {"planeNormal", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Mirror plane facing")},
        {"planeFace", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Mirror in the face")},
        {"type", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Type")},
        {"diameter", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Diameter")},
        {"boreDiameter", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Counterbore diameter")},
        {"boreDepth", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Counterbore depth")},
        {"sinkDiameter", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Countersink diameter")},
        {"sinkAngle", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Countersink angle")},
        {"pointAngle", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Point angle (0: flat)")},
        {"positionPoint", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "At")},
        {"face", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Into the face")},
    };
    const auto it = labels.find(name);
    return it != labels.end() ? QCoreApplication::translate("hz::ui::MainWindow", it->second)
                              : QString::fromStdString(name);
}

std::vector<PlaneChoice> planarFacesOf(const topo::Solid& solid) {
    std::vector<PlaneChoice> out;
    const double outward = model::outwardSign(solid);
    for (const auto& face : solid.faces()) {
        // As a build finds it again (Phase 157), so a sketch on it is placed
        // where it was drawn.
        const auto plane = model::planeOf(face, outward);
        if (!plane) continue;
        const math::Vec3& normal = plane->normal;
        const math::Vec3 across =
            std::abs(normal.dot(math::Vec3::UnitX)) < 0.9 ? math::Vec3::UnitX : math::Vec3::UnitY;
        out.push_back({QCoreApplication::translate("hz::ui::MainWindow", "facing %1 at %2")
                           .arg(formatPoint(normal), formatPoint(plane->origin)),
                       draft::SketchPlane(plane->origin, normal, across), face.topoId.tag()});
    }
    return out;
}

std::vector<FlatFace> flatFacesOf(const topo::Solid& solid) {
    std::vector<FlatFace> out;
    std::set<std::string> listed;
    for (const auto& choice : planarFacesOf(solid)) {
        std::string whole = model::wholeFaceName(choice.tag);
        if (!listed.insert(whole).second) continue;
        out.push_back(
            {choice.text, choice.plane.origin(), choice.plane.normal(), std::move(whole)});
    }
    return out;
}

int clickedFlatFace(const std::vector<FlatFace>& faces,
                    const std::vector<ViewportWidget::ModelPick>& picks, int otherwise) {
    for (const auto& pick : picks) {
        if (pick.edge || pick.tag.empty()) continue;
        const std::string whole = model::wholeFaceName(pick.tag);
        for (size_t i = 0; i < faces.size(); ++i) {
            if (faces[i].name == whole) return static_cast<int>(i);
        }
    }
    return otherwise;
}

std::optional<math::Vec3> clickedDirection(const topo::Solid& solid,
                                           const std::vector<ViewportWidget::ModelPick>& picks) {
    for (const auto& pick : picks) {
        if (pick.owner != 0) continue;
        if (pick.edge) {
            for (const auto& edge : solid.edges()) {
                const topo::HalfEdge* he = edge.halfEdge;
                if (edge.topoId.tag() != pick.tag || !he || !he->origin || !he->next ||
                    !he->next->origin) {
                    continue;
                }
                const math::Vec3 along = he->next->origin->point - he->origin->point;
                if (along.length() > 1e-12) return along.normalized();
            }
            continue;
        }
        for (const auto& face : planarFacesOf(solid)) {
            if (face.tag == pick.tag) return face.plane.normal();
        }
    }
    return std::nullopt;
}

namespace {

/// Directions offered for a pull or a pattern: the six axis directions.
const std::vector<std::pair<QString, math::Vec3>>& axisDirections() {
    static const std::vector<std::pair<QString, math::Vec3>> directions = {
        {QStringLiteral("+X"), math::Vec3(1, 0, 0)}, {QStringLiteral("−X"), math::Vec3(-1, 0, 0)},
        {QStringLiteral("+Y"), math::Vec3(0, 1, 0)}, {QStringLiteral("−Y"), math::Vec3(0, -1, 0)},
        {QStringLiteral("+Z"), math::Vec3(0, 0, 1)}, {QStringLiteral("−Z"), math::Vec3(0, 0, -1)},
    };
    return directions;
}

}  // namespace

QComboBox* directionChoice(FeatureForm& form, const QString& name, const QString& label,
                           const QString& initial) {
    QStringList names;
    for (const auto& [text, direction] : axisDirections()) names << text;
    auto* combo = form.choice(name, label, names);
    combo->setCurrentText(initial);
    return combo;
}

math::Vec3 chosenDirection(const QComboBox* combo) {
    return axisDirections().at(static_cast<size_t>(combo->currentIndex())).second;
}

}  // namespace hz::ui
