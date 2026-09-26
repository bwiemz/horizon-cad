#include "horizon/ui/AssemblyWorkbench.h"

#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <map>
#include <numbers>
#include <set>
#include <utility>

#include "horizon/document/AssemblyMates.h"
#include "horizon/document/BillOfMaterials.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/fileio/BomExport.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Quaternion.h"
#include "horizon/modeling/AssemblySolver.h"
#include "horizon/modeling/EdgeProjection.h"
#include "horizon/modeling/MateGeometry.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/AssemblyTreePanel.h"
#include "horizon/ui/FeatureForm.h"
#include "horizon/ui/Preferences.h"
#include "horizon/ui/QuantitySpinBox.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/ui/WorkbenchHost.h"

namespace hz::ui {

namespace {

/// A component's bounds where it is placed, from its mesh; invalid when it
/// has none.
math::BoundingBox placedBounds(const doc::ComponentInstance& comp) {
    math::BoundingBox box;
    if (!comp.cachedMesh) return box;
    const auto& p = comp.cachedMesh->positions;
    for (size_t i = 0; i + 2 < p.size(); i += 3) {
        box.expand(comp.transform.transformPoint(math::Vec3(p[i], p[i + 1], p[i + 2])));
    }
    return box;
}

/// Whether @p comp places the file at @p path: is it, or, a subassembly,
/// places it at any depth (Phase 159). @p dir: the folder of the assembly
/// @p comp is in.
bool placesFile(const doc::ComponentInstance& comp, const std::string& dir,
                const std::string& path) {
    const std::string file = AssemblyWorkbench::partFile(comp, dir);
    if (doc::DocumentManager::samePath(file, path)) return true;
    if (!comp.resolvedAssembly) return false;
    const std::string subDir = std::filesystem::path(file).parent_path().string();
    const auto& children = comp.resolvedAssembly->components();
    return std::any_of(children.begin(), children.end(), [&](const doc::ComponentInstance& child) {
        return placesFile(child, subDir, path);
    });
}

/// What a mate frame is, in words (Phase 160: more than planes and
/// cylinders).
QString describeFrame(const model::MateFrame& frame) {
    switch (frame.kind) {
        case model::MateFrameKind::Planar:
            return AssemblyWorkbench::tr("plane facing %1").arg(formatPoint(frame.direction));
        case model::MateFrameKind::Cylindrical:
            return AssemblyWorkbench::tr("cylinder of radius %1 along %2")
                .arg(frame.radius, 0, 'g', 6)
                .arg(formatPoint(frame.direction));
        case model::MateFrameKind::Line:
            return AssemblyWorkbench::tr("line along %1").arg(formatPoint(frame.direction));
        case model::MateFrameKind::Point:
            return AssemblyWorkbench::tr("point at %1").arg(formatPoint(frame.origin));
        case model::MateFrameKind::Circle:
            return AssemblyWorkbench::tr("circle of radius %1 about %2")
                .arg(frame.radius, 0, 'g', 6)
                .arg(formatPoint(frame.origin));
        case model::MateFrameKind::Spherical:
            return AssemblyWorkbench::tr("sphere of radius %1 about %2")
                .arg(frame.radius, 0, 'g', 6)
                .arg(formatPoint(frame.origin));
        case model::MateFrameKind::Conical:
            return AssemblyWorkbench::tr("cone from %1 along %2")
                .arg(formatPoint(frame.origin), formatPoint(frame.direction));
    }
    return {};
}

/// A mate side from component @p id and a list row's data, "<kind>:<name>"
/// (Phase 160).
doc::MateReference referenceOf(qulonglong id, const QVariant& data) {
    doc::MateReference ref;
    ref.componentId = static_cast<uint64_t>(id);
    const QString text = data.toString();
    const auto colon = text.indexOf(QLatin1Char(':'));
    const QString kind = colon < 0 ? QStringLiteral("face") : text.left(colon);
    ref.faceId = topo::TopologyID::fromTag((colon < 0 ? text : text.mid(colon + 1)).toStdString());
    if (kind == QLatin1String("edge")) ref.kind = doc::ReferenceKind::Edge;
    if (kind == QLatin1String("datum")) ref.kind = doc::ReferenceKind::Datum;
    return ref;
}

/// Whether any component of @p now is placed other than in @p before.
bool placementsDiffer(const doc::AssemblyState& before, const doc::AssemblyDocument& now) {
    for (const auto& comp : now.components()) {
        for (const auto& was : before.components) {
            if (was.id != comp.id) continue;
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    if (was.transform.at(r, c) != comp.transform.at(r, c)) return true;
                }
            }
        }
    }
    return false;
}

QString mateTypeName(doc::MateType type) {
    switch (type) {
        case doc::MateType::Coincident:
            return AssemblyWorkbench::tr("Coincident");
        case doc::MateType::Concentric:
            return AssemblyWorkbench::tr("Concentric");
        case doc::MateType::Distance:
            return AssemblyWorkbench::tr("Distance");
        case doc::MateType::Angle:
            return AssemblyWorkbench::tr("Angle");
        case doc::MateType::Parallel:
            return AssemblyWorkbench::tr("Parallel");
        case doc::MateType::Perpendicular:
            return AssemblyWorkbench::tr("Perpendicular");
        case doc::MateType::Tangent:
            return AssemblyWorkbench::tr("Tangent");
        case doc::MateType::Fixed:
            return AssemblyWorkbench::tr("Fixed");
    }
    return {};
}

/// Whether a Fixed mate holds @p id where it is.
bool isFixed(const doc::AssemblyDocument& assembly, uint64_t id) {
    return std::any_of(assembly.mates().begin(), assembly.mates().end(), [id](const doc::Mate& m) {
        return m.type == doc::MateType::Fixed && m.a.componentId == id;
    });
}

}  // namespace

AssemblyWorkbench::AssemblyWorkbench(WorkbenchHost& host, AssemblyTreePanel& tree, QObject* parent)
    : QObject(parent), m_host(host), m_tree(tree) {
    // The tree asks; the commands act, as undoable edits.
    connect(&m_tree, &AssemblyTreePanel::componentSelected, this, [this](uint64_t id) {
        m_host.viewport().chooseModel(ViewportWidget::ModelPick{id, {}, false}, false);
    });
    connect(&m_tree, &AssemblyTreePanel::removeComponentRequested, this,
            [this](uint64_t id) { removeComponent(id); });
    connect(&m_tree, &AssemblyTreePanel::suppressRequested, this,
            [this](uint64_t id, bool suppress) { setComponentSuppressed(id, suppress); });
    connect(&m_tree, &AssemblyTreePanel::renameRequested, this,
            [this](uint64_t id) { renameComponent(id); });
    connect(&m_tree, &AssemblyTreePanel::openPartRequested, this,
            [this](uint64_t id) { openComponentPart(id); });
    connect(&m_tree, &AssemblyTreePanel::editMateRequested, this,
            [this](uint64_t id) { editMate(id); });
    connect(&m_tree, &AssemblyTreePanel::removeMateRequested, this,
            [this](uint64_t id) { removeMate(id); });
    // A component clicked in the view is the tree's current row.
    connect(&m_host.viewport(), &ViewportWidget::modelSelectionChanged, this, [this] {
        if (assembly() == nullptr) return;
        for (const auto& pick : m_host.viewport().modelSelection()) {
            if (pick.owner == 0) continue;
            m_tree.showComponent(pick.owner);
            break;
        }
    });
}

AssemblyWorkbench::~AssemblyWorkbench() {
    // Stop the worker before what it reports to is gone.
    m_interferenceTask.reset();
}

doc::AssemblyDocument* AssemblyWorkbench::assembly() const {
    return m_host.currentAssembly().get();
}

std::string AssemblyWorkbench::assemblyDir(const doc::AssemblyDocument& assembly) {
    return assembly.filePath().empty()
               ? std::string()
               : std::filesystem::path(assembly.filePath()).parent_path().string();
}

std::string AssemblyWorkbench::partFile(const doc::ComponentInstance& comp,
                                        const std::string& dir) {
    std::filesystem::path path(comp.partPath);
    if (path.is_relative() && !dir.empty()) path = std::filesystem::path(dir) / path;
    return path.string();
}

bool AssemblyWorkbench::placeOnOpen(doc::AssemblyDocument& assembly) {
    const doc::AssemblyState placed = assembly.snapshot();
    solveAssemblyMates(assembly);
    return placementsDiffer(placed, assembly);
}

void AssemblyWorkbench::cancelWork() {
    if (m_interferenceTask) m_interferenceTask->cancel();
}

void AssemblyWorkbench::recordAssemblyEdit(doc::AssemblyState before, bool wasDirty,
                                           const QString& description) {
    // From here the undo stack carries the change — undoing back to the saved
    // state must clear the modified marker, which the assembly's own flag,
    // set by the edit itself, would not.
    assembly()->setDirty(wasDirty);
    m_host.currentDocument()->undoStack().push(std::make_unique<doc::AssemblyEditCommand>(
        *assembly(), std::move(before), assembly()->snapshot(), description.toStdString()));
}

void AssemblyWorkbench::onInsertComponent() {
    m_host.viewport().cancelComponentDrag();  // not under a drag (Phase 158)
    if (!assembly()) {
        m_host.showStatus(tr("Insert Component is only available in an assembly document"));
        return;
    }

    // A part, or an assembly placed whole (Phase 159): a subassembly.
    QString fileName = QFileDialog::getOpenFileName(
        m_host.dialogParent(), tr("Insert Component"), QString(),
        tr("Parts and Assemblies (*.hzpart *.hzasm);;Horizon Parts (*.hzpart);;Horizon "
           "Assemblies (*.hzasm);;All Files (*)"));
    if (fileName.isEmpty()) return;

    doc::ComponentInstance comp;
    comp.partPath = fileName.toStdString();
    comp.name = std::filesystem::path(comp.partPath).stem().string();

    const std::string asmDir = assemblyDir(*assembly());
    // Not this assembly, nor one that places it: an assembly inside itself.
    std::vector<std::string> within;
    if (!assembly()->filePath().empty()) {
        within.push_back(doc::DocumentManager::canonicalPath(assembly()->filePath()));
    }
    std::string why;
    if (!m_host.documents().resolveComponent(comp, doc::ComponentState::Lightweight, asmDir, &why,
                                             within)) {
        QMessageBox::warning(
            m_host.dialogParent(), tr("Insert Component"),
            why.empty() ? tr("Failed to load the part (no geometry could be produced).")
                        : tr("%1 is not inserted: %2")
                              .arg(QString::fromStdString(comp.name), QString::fromStdString(why)));
        return;
    }

    // Beside the others, along +X, not on top of them at the origin.
    math::BoundingBox others;
    for (const auto& placed : assembly()->components()) {
        if (!placed.suppressed) others.expand(placedBounds(placed));
    }
    const math::BoundingBox own = placedBounds(comp);
    if (others.isValid() && own.isValid()) {
        const double gap =
            0.1 * std::max((others.max() - others.min()).x, (own.max() - own.min()).x);
        comp.transform =
            math::Mat4::translation(math::Vec3(others.max().x + gap - own.min().x, 0.0, 0.0));
    }

    const bool wasDirty = assembly()->isDirty();
    doc::AssemblyState before = assembly()->snapshot();
    assembly()->addComponent(std::move(comp));
    recordAssemblyEdit(std::move(before), wasDirty, tr("Insert Component"));
    m_host.rebuildScene();
    m_host.viewport().camera().setIsometricView();
    m_host.setPrompt(tr("Component inserted."));
}

bool AssemblyWorkbench::solveAssemblyMates(doc::AssemblyDocument& asmDoc, bool reportSuccess) {
    if (asmDoc.mates().empty()) return true;

    // Frames come from B-Rep faces, so mate solving needs resolved parts.
    const std::string asmDir = assemblyDir(asmDoc);
    for (auto& comp : asmDoc.components()) {
        if (comp.solid() == nullptr) {
            m_host.documents().resolveComponent(comp, doc::ComponentState::Resolved, asmDir);
        }
    }
    std::string why;
    const auto mates = doc::AssemblyMates::gather(asmDoc, &why);
    if (!mates) {
        QString message = QString::fromStdString(why);
        if (!message.isEmpty()) message[0] = message[0].toUpper();
        m_host.showStatus(message);
        return false;
    }
    const auto result = mates->solve();

    if (result.status == model::AssemblySolveStatus::Success) {
        for (auto& comp : asmDoc.components()) {
            auto it = result.transforms.find(comp.id);
            if (it != result.transforms.end()) comp.transform = it->second;
        }
        QString status = tr("Mates solved (%1 iterations)").arg(result.iterations);
        if (result.redundantCount > 0) {
            status += tr("; %1 redundant constraint(s)").arg(result.redundantCount);
        }
        if (!result.ungroundedComponents.empty()) {
            status += tr("; %1 component(s) not connected to ground")
                          .arg(result.ungroundedComponents.size());
        }
        if (reportSuccess) m_host.showStatus(status);
        return true;
    }

    m_host.showStatus(tr("Mate solve failed: %1")
                          .arg(QString::fromStdString(result.message.empty() ? "did not converge"
                                                                             : result.message)));
    return false;
}

// ---------------------------------------------------------------------------
// Dragging a component (Phase 158)
// ---------------------------------------------------------------------------

namespace {

/// How far along the line through @p point along unit @p axis the ray
/// @p origin + t @p direction comes nearest; nothing when they run parallel.
std::optional<double> nearestAlong(const math::Vec3& point, const math::Vec3& axis,
                                   const math::Vec3& origin, const math::Vec3& direction) {
    const math::Vec3 w = point - origin;
    const double b = axis.dot(direction);
    const double c = direction.dot(direction);
    const double denominator = c - b * b;  // |axis| = 1
    if (std::abs(denominator) < 1e-12 * c) return std::nullopt;
    return (b * direction.dot(w) - c * axis.dot(w)) / denominator;
}

/// Where the ray @p origin + t @p direction meets the plane through
/// @p point facing @p normal; nothing when it runs along it.
std::optional<math::Vec3> meetPlane(const math::Vec3& point, const math::Vec3& normal,
                                    const math::Vec3& origin, const math::Vec3& direction) {
    const double across = direction.dot(normal);
    if (std::abs(across) < 1e-12) return std::nullopt;
    return origin + direction * ((point - origin).dot(normal) / across);
}

}  // namespace

std::optional<ComponentDragger::TriadPose> AssemblyWorkbench::triadPose() const {
    const doc::AssemblyDocument* asmDoc = assembly();
    // Nothing is dragged exploded (Phase 161): nor is a triad shown.
    if (asmDoc == nullptr || asmDoc->shownView() != 0) return std::nullopt;
    for (const auto& pick : m_host.viewport().modelSelection()) {
        if (pick.owner == 0) continue;
        const doc::ComponentInstance* comp = asmDoc->component(pick.owner);
        if (comp == nullptr || comp->suppressed || !comp->cachedMesh) continue;
        const math::BoundingBox box = placedBounds(*comp);
        if (!box.isValid()) continue;
        TriadPose pose;
        pose.component = comp->id;
        pose.origin = box.center();
        pose.axes = {comp->transform.transformDirection(math::Vec3::UnitX).normalized(),
                     comp->transform.transformDirection(math::Vec3::UnitY).normalized(),
                     comp->transform.transformDirection(math::Vec3::UnitZ).normalized()};
        return pose;
    }
    return std::nullopt;
}

bool AssemblyWorkbench::beginDrag(std::uint64_t component, const QPointF& at,
                                  const std::optional<Triad::Handle>& handle) {
    cancelDrag();  // one left over (its release lost) is put back first
    auto asmDoc = m_host.currentAssembly();
    if (!asmDoc) return false;
    const doc::ComponentInstance* comp = asmDoc->component(component);
    if (comp == nullptr || comp->suppressed) return false;
    const QString name = QString::fromStdString(comp->name);
    if (asmDoc->shownView() != 0) {
        // Drawn moved, it would be dragged from where it is not.
        m_host.showStatus(
            tr("%1 is not dragged while the assembly is exploded: show none to drag").arg(name));
        return false;
    }
    if (isFixed(*asmDoc, component)) {
        m_host.showStatus(tr("%1 is held by a Fixed mate: it is not dragged").arg(name));
        return false;
    }
    ViewportWidget& view = m_host.viewport();
    Drag drag;
    drag.assembly = asmDoc;
    drag.component = component;
    drag.start = comp->transform;
    drag.facing = (view.camera().target() - view.camera().eye()).normalized();
    drag.handle = handle;
    const auto [origin, direction] =
        view.camera().screenToRay(at.x(), at.y(), view.width(), view.height());
    if (handle) {
        // By the triad: along an arrow, or round a ring, from where it was
        // grabbed on it.
        const auto pose = triadPose();
        if (!pose || pose->component != component) return false;
        drag.pivot = pose->origin;
        drag.axis = pose->axes.at(static_cast<size_t>(handle->axis));
        if (handle->kind == Triad::Kind::Arrow) {
            const auto along = nearestAlong(drag.pivot, drag.axis, origin, direction);
            if (!along) return false;
            drag.along = *along;
        } else {
            const auto on = meetPlane(drag.pivot, drag.axis, origin, direction);
            if (!on) return false;
            drag.towards = *on - drag.pivot;
        }
    } else {
        const auto grabbed = view.pickModelPoint(at);
        if (!grabbed) return false;
        drag.grabbed = *grabbed;
    }
    drag.before = asmDoc->snapshot();
    drag.wasDirty = asmDoc->isDirty();
    if (!asmDoc->mates().empty()) {
        // Frames come from the parts: found once, for every move.
        const std::string dir = assemblyDir(*asmDoc);
        for (auto& c : asmDoc->components()) {
            if (c.solid() == nullptr) {
                m_host.documents().resolveComponent(c, doc::ComponentState::Resolved, dir);
            }
        }
        std::string why;
        auto mates = doc::AssemblyMates::gather(*asmDoc, &why);
        if (!mates) {
            m_host.showStatus(tr("%1 is not dragged: %2").arg(name, QString::fromStdString(why)));
            return false;
        }
        drag.mates = std::move(mates);
    }
    m_drag = std::move(drag);
    m_host.setPrompt(tr("Dragging %1: release to place it, Escape to put it back").arg(name));
    return true;
}

void AssemblyWorkbench::dragTo(const QPointF& at) {
    if (!m_drag) return;
    if (m_host.currentAssembly() != m_drag->assembly) {
        cancelDrag();  // the view shows another document now
        return;
    }
    ViewportWidget& view = m_host.viewport();
    const auto [origin, direction] =
        view.camera().screenToRay(at.x(), at.y(), view.width(), view.height());
    math::Mat4 target = m_drag->start;
    if (!m_drag->handle) {
        // Free: in the plane through the point grabbed, facing the view.
        const auto point = meetPlane(m_drag->grabbed, m_drag->facing, origin, direction);
        if (!point) return;
        target = math::Mat4::translation(*point - m_drag->grabbed) * m_drag->start;
    } else if (m_drag->handle->kind == Triad::Kind::Arrow) {
        const auto along = nearestAlong(m_drag->pivot, m_drag->axis, origin, direction);
        if (!along) return;
        target = math::Mat4::translation(m_drag->axis * (*along - m_drag->along)) * m_drag->start;
    } else {
        const auto on = meetPlane(m_drag->pivot, m_drag->axis, origin, direction);
        if (!on) return;
        const math::Vec3 now = *on - m_drag->pivot;
        const double angle =
            std::atan2(m_drag->axis.dot(m_drag->towards.cross(now)), m_drag->towards.dot(now));
        target = math::Mat4::translation(m_drag->pivot) *
                 math::Mat4::rotation(math::Quaternion::fromAxisAngle(m_drag->axis, angle)) *
                 math::Mat4::translation(m_drag->pivot * -1.0) * m_drag->start;
    }

    std::map<std::uint64_t, math::Mat4> placed{{m_drag->component, target}};
    if (m_drag->mates) {
        const doc::AssemblyMates::Hold hold{m_drag->component, target};
        auto result = m_drag->mates->solve({hold, false});
        if (result.status != model::AssemblySolveStatus::Success &&
            result.status != model::AssemblySolveStatus::NoMates) {
            // Held there, the mates cannot be met: let go from there, it
            // slides as far as they allow.
            doc::AssemblyMates freed = *m_drag->mates;
            freed.place(placed);
            result = freed.solve({std::nullopt, false});
        }
        if (result.status == model::AssemblySolveStatus::Success) {
            placed = result.transforms;
        } else if (result.status != model::AssemblySolveStatus::NoMates) {
            return;  // the last placement the mates allowed stays
        }
        m_drag->mates->place(placed);  // the next move solves from here
    }
    for (auto& comp : m_drag->assembly->components()) {
        const auto found = placed.find(comp.id);
        if (found != placed.end()) comp.transform = found->second;
    }
    m_drag->moved = true;
    showPlacements(*m_drag->assembly);
}

void AssemblyWorkbench::endDrag() {
    if (!m_drag) return;
    Drag drag = std::move(*m_drag);
    m_drag.reset();
    m_host.setPrompt(QString());
    const bool changed = drag.moved && placementsDiffer(drag.before, *drag.assembly);
    if (!changed || m_host.currentAssembly() != drag.assembly) {
        drag.assembly->restore(drag.before);
        drag.assembly->setDirty(drag.wasDirty);
        return;
    }
    recordAssemblyEdit(std::move(drag.before), drag.wasDirty, tr("Drag Component"));
    m_host.rebuildScene();
    const doc::ComponentInstance* comp = drag.assembly->component(drag.component);
    m_host.showStatus(
        tr("%1 dragged").arg(comp != nullptr ? QString::fromStdString(comp->name) : QString()));
}

void AssemblyWorkbench::cancelDrag() {
    if (!m_drag) return;
    Drag drag = std::move(*m_drag);
    m_drag.reset();
    drag.assembly->restore(drag.before);
    drag.assembly->setDirty(drag.wasDirty);
    if (m_host.currentAssembly() == drag.assembly) showPlacements(*drag.assembly);
    m_host.setPrompt(tr("Drag cancelled: everything is where it was"));
}

void AssemblyWorkbench::showPlacements(const doc::AssemblyDocument& asmDoc) {
    ViewportWidget& view = m_host.viewport();
    for (const auto& node : view.sceneGraph().nodes()) {
        if (node->ownerId() == 0) continue;
        if (const doc::ComponentInstance* comp = asmDoc.component(node->ownerId())) {
            node->setLocalTransform(asmDoc.displayTransform(*comp));
        }
    }
    view.update();
}

AssemblyWorkbench::StepExport AssemblyWorkbench::stepExport() {
    StepExport out;
    if (!assembly()) return out;
    // Each part read once, by its file, however many components place it.
    const std::string dir = assemblyDir(*assembly());
    std::map<std::string, std::size_t> partOf;
    // A subassembly (Phase 159) is written as its own parts, each placed by
    // its transform within it composed with the subassembly's: one level.
    std::function<void(const doc::ComponentInstance&, const std::string&, const math::Mat4&,
                       const std::string&)>
        place = [&](const doc::ComponentInstance& comp, const std::string& in,
                    const math::Mat4& outer, const std::string& name) {
            const std::string file =
                std::filesystem::path(partFile(comp, in)).lexically_normal().string();
            if (comp.resolvedAssembly) {
                const std::string subDir = std::filesystem::path(file).parent_path().string();
                for (const auto& child : comp.resolvedAssembly->components()) {
                    if (!child.suppressed) {
                        place(child, subDir, outer * comp.transform, name + "/" + child.name);
                    }
                }
                return;
            }
            const topo::Solid* solid = comp.solid();
            if (solid == nullptr) {
                out.unread.push_back(name);
                return;
            }
            const auto [known, added] = partOf.emplace(file, out.parts.size());
            if (added) {
                out.parts.push_back(
                    {QFileInfo(QString::fromStdString(file)).completeBaseName().toStdString(),
                     {solid}});
            }
            out.occurrences.push_back({known->second, name, outer * comp.transform});
        };
    for (auto& comp : assembly()->components()) {
        if (comp.suppressed) continue;
        if (comp.solid() == nullptr) {
            m_host.documents().resolveComponent(comp, doc::ComponentState::Resolved, dir);
        }
        place(comp, dir, math::Mat4::identity(), comp.name);
    }
    return out;
}

void AssemblyWorkbench::onCheckInterference() {
    if (!assembly()) {
        m_host.showStatus(tr("Check Interference is only available in an assembly document"));
        return;
    }

    // Interference is measured on the B-Rep, so every component is resolved.
    const std::string asmDir = assemblyDir(*assembly());
    for (auto& comp : assembly()->components()) {
        if (!comp.suppressed && comp.solid() == nullptr) {
            m_host.documents().resolveComponent(comp, doc::ComponentState::Resolved, asmDir);
        }
    }

    auto input =
        std::make_shared<doc::AssemblyDocument::InterferenceInput>(assembly()->interferenceInput());
    const bool onWorker = m_host.onWorker(input->faceCount() >= kWorkerInterferenceFaces);
    if (!onWorker) {
        showInterference(*assembly(), doc::AssemblyDocument::measureInterference(*input));
        return;
    }
    if (m_interferenceTask) {
        m_host.showStatus(tr("An interference check is already running"));
        return;
    }
    // The input is copies of the placed solids: the assembly can be edited,
    // or its tab closed, while they are measured.
    m_interferenceAssembly = m_host.currentAssembly();
    m_interferenceTask = std::make_unique<BackgroundTask<doc::InterferenceReport>>(
        [input](const std::atomic<bool>& cancelled) {
            return doc::AssemblyDocument::measureInterference(*input, &cancelled);
        });
    m_interferenceTask->start([this] {
        QMetaObject::invokeMethod(this, &AssemblyWorkbench::onInterferenceFinished,
                                  Qt::QueuedConnection);
    });
    m_host.setPrompt(tr("Checking interference..."));
    m_host.backgroundWorkChanged();
}

void AssemblyWorkbench::onInterferenceFinished() {
    if (!m_interferenceTask || !m_interferenceTask->finished()) return;
    const std::unique_ptr<BackgroundTask<doc::InterferenceReport>> task =
        std::move(m_interferenceTask);
    const std::shared_ptr<doc::AssemblyDocument> checked = std::move(m_interferenceAssembly);
    m_host.backgroundWorkChanged();
    m_host.setPrompt(tr("Ready"));
    if (task->cancelled()) {
        m_host.showStatus(tr("Interference check cancelled"), 10000);
        return;
    }
    if (!task->error().empty()) {
        m_host.showStatus(
            tr("The interference check failed: %1").arg(QString::fromStdString(task->error())));
        return;
    }
    showInterference(*checked, task->take());
}

void AssemblyWorkbench::showInterference(const doc::AssemblyDocument& assembly,
                                         const doc::InterferenceReport& report) {
    auto nameOf = [&assembly](uint64_t id) {
        const auto* comp = assembly.component(id);
        const std::string name = comp && !comp->name.empty() ? comp->name : "component";
        return QString("%1 (#%2)").arg(QString::fromStdString(name)).arg(id);
    };
    const math::LengthUnit unit = assembly.lengthUnit();

    QStringList lines;
    for (const auto& pair : report.pairs) {
        const QString amount =
            pair.volumeResolved
                ? tr("%1 shared").arg(Preferences::current().formatVolume(pair.volume, unit))
                : tr("overlap could not be measured");
        lines << tr("%1 and %2: %3").arg(nameOf(pair.componentA), nameOf(pair.componentB), amount);
    }
    for (uint64_t id : report.unchecked) {
        lines << tr("%1 was not checked: its part could not be resolved").arg(nameOf(id));
    }

    if (report.pairs.empty()) {
        m_host.showStatus(report.unchecked.empty()
                              ? tr("No interference found")
                              : tr("No interference among the resolved components"));
    } else {
        m_host.showStatus(tr("%n interfering pair(s)", "", static_cast<int>(report.pairs.size())));
    }
    if (!lines.isEmpty()) {
        QMessageBox::information(m_host.dialogParent(), tr("Interference"), lines.join('\n'));
    }
}

void AssemblyWorkbench::onAddMate() {
    m_host.viewport().cancelComponentDrag();  // not under a drag (Phase 158)
    if (!assembly()) {
        m_host.showStatus(tr("Add Mate is only available in an assembly document"));
        return;
    }
    if (assembly()->components().size() < 2) {
        m_host.showStatus(tr("Insert at least two components first"));
        return;
    }
    // Faces or edges (Phase 160) clicked in the viewport, on two components:
    // the mate's A and B.
    std::vector<ViewportWidget::ModelPick> clickedFaces;
    for (const auto& pick : m_host.viewport().modelSelection()) {
        const bool another = std::none_of(
            clickedFaces.begin(), clickedFaces.end(),
            [&pick](const ViewportWidget::ModelPick& p) { return p.owner == pick.owner; });
        if (pick.owner != 0 && !pick.tag.empty() && another) clickedFaces.push_back(pick);
    }

    // Resolve parts so faces are available for picking.
    const std::string asmDir = assemblyDir(*assembly());
    for (auto& comp : assembly()->components()) {
        if (comp.solid() == nullptr) {
            m_host.documents().resolveComponent(comp, doc::ComponentState::Resolved, asmDir);
        }
    }

    // What a mate can take, one row each, said by what it is and where; the
    // row's data is its reference, "<kind>:<name>" (referenceOf):
    // - faces (a curved face's facets together, as a click picks it);
    // - straight and round edges (Phase 160), by their whole names;
    // - the part's datum planes, axes and points (Phase 160).
    auto addFaces = [](const doc::ComponentInstance& comp, QComboBox* combo) {
        const auto add = [combo](const model::MateFrame& frame, const QString& kind,
                                 const std::string& name) {
            combo->addItem(
                QStringLiteral("%1 (%2)").arg(describeFrame(frame), QString::fromStdString(name)),
                kind + QLatin1Char(':') + QString::fromStdString(name));
        };
        if (const topo::Solid* solid = comp.solid()) {
            std::set<std::string> seen;
            for (const auto& face : solid->faces()) {
                if (!face.topoId.isValid()) continue;
                const std::string logical = model::logicalFace(face.topoId.tag());
                if (!seen.insert(logical).second) continue;
                if (const auto frame = model::MateGeometry::frameForFace(face)) {
                    add(*frame, QStringLiteral("face"), logical);
                }
            }
            seen.clear();
            for (const auto& edge : solid->edges()) {
                if (!edge.topoId.isValid()) continue;
                if (edge.topoId.tag().find("/seam:") != std::string::npos) continue;
                const std::string whole = model::wholeEdgeName(edge.topoId.tag());
                if (!seen.insert(whole).second) continue;
                if (const auto frame = model::MateGeometry::frameForEdge(*solid, whole)) {
                    add(*frame, QStringLiteral("edge"), whole);
                }
            }
        }
        if (comp.resolvedPart) {
            const doc::FeatureTree& tree = comp.resolvedPart->featureTree();
            for (size_t i = 0; i < tree.featureCount(); ++i) {
                const auto* datum = dynamic_cast<const doc::DatumFeature*>(tree.feature(i));
                if (datum == nullptr) continue;
                model::MateFrame frame;
                if (datum->datumKind() == doc::DatumFeature::DatumKind::Plane) {
                    frame.kind = model::MateFrameKind::Planar;
                    frame.direction = datum->asPlane().normal;
                } else if (datum->datumKind() == doc::DatumFeature::DatumKind::Axis) {
                    frame.kind = model::MateFrameKind::Line;
                    frame.direction = datum->asAxis().direction;
                } else {
                    frame.kind = model::MateFrameKind::Point;
                    frame.origin = datum->asPoint().position;
                }
                add(frame, QStringLiteral("datum"), datum->featureID());
            }
        }
    };

    QDialog dialog(m_host.dialogParent());
    dialog.setWindowTitle(tr("Add Mate"));
    auto* form = new QFormLayout(&dialog);

    // The type by its value, not by the combo's order.
    auto* typeCombo = new QComboBox(&dialog);
    typeCombo->setObjectName("type");
    for (const doc::MateType type :
         {doc::MateType::Coincident, doc::MateType::Concentric, doc::MateType::Distance,
          doc::MateType::Angle, doc::MateType::Parallel, doc::MateType::Perpendicular,
          doc::MateType::Tangent, doc::MateType::Fixed}) {
        typeCombo->addItem(mateTypeName(type), static_cast<int>(type));
    }
    form->addRow(tr("Type:"), typeCombo);

    auto* compACombo = new QComboBox(&dialog);
    auto* compBCombo = new QComboBox(&dialog);
    compACombo->setObjectName("componentA");
    compBCombo->setObjectName("componentB");
    for (const auto& comp : assembly()->components()) {
        QString label =
            QString("%1 (#%2)")
                .arg(QString::fromStdString(comp.name.empty() ? "component" : comp.name))
                .arg(comp.id);
        compACombo->addItem(label, QVariant::fromValue<qulonglong>(comp.id));
        compBCombo->addItem(label, QVariant::fromValue<qulonglong>(comp.id));
    }
    if (compBCombo->count() > 1) compBCombo->setCurrentIndex(1);

    auto* faceACombo = new QComboBox(&dialog);
    auto* faceBCombo = new QComboBox(&dialog);
    faceACombo->setObjectName("faceA");
    faceBCombo->setObjectName("faceB");
    auto refreshFaces = [&](QComboBox* compCombo, QComboBox* faceCombo) {
        faceCombo->clear();
        const auto id = static_cast<uint64_t>(compCombo->currentData().toULongLong());
        if (const auto* comp = assembly()->component(id)) addFaces(*comp, faceCombo);
    };
    refreshFaces(compACombo, faceACombo);
    refreshFaces(compBCombo, faceBCombo);
    connect(compACombo, &QComboBox::currentIndexChanged, &dialog,
            [&] { refreshFaces(compACombo, faceACombo); });
    connect(compBCombo, &QComboBox::currentIndexChanged, &dialog,
            [&] { refreshFaces(compBCombo, faceBCombo); });
    const auto offer = [](QComboBox* compCombo, QComboBox* faceCombo,
                          const ViewportWidget::ModelPick& pick) {
        const int comp = compCombo->findData(QVariant::fromValue<qulonglong>(pick.owner));
        if (comp < 0) return;
        compCombo->setCurrentIndex(comp);  // refreshes the faces
        const QString reference =
            pick.edge
                ? QStringLiteral("edge:") + QString::fromStdString(model::wholeEdgeName(pick.tag))
                : QStringLiteral("face:") + QString::fromStdString(pick.tag);
        const int face = faceCombo->findData(reference);
        if (face >= 0) faceCombo->setCurrentIndex(face);
    };
    if (!clickedFaces.empty()) offer(compACombo, faceACombo, clickedFaces[0]);
    if (clickedFaces.size() > 1) offer(compBCombo, faceBCombo, clickedFaces[1]);

    form->addRow(tr("Component A:"), compACombo);
    form->addRow(tr("On A:"), faceACombo);
    form->addRow(tr("Component B:"), compBCombo);
    form->addRow(tr("On B:"), faceBCombo);

    // A distance mate's distance, or an angle mate's angle, in the
    // assembly's unit or typed in another (Phase 154).
    const math::LengthUnit unit = m_host.currentDocument()->lengthUnit();
    auto* valueSpin = new QuantitySpinBox(QuantitySpinBox::Kind::Length, unit, 3, &dialog);
    valueSpin->setObjectName("value");
    valueSpin->setRange(-1e6, 1e6);
    form->addRow(tr("Distance:"), valueSpin);
    auto* angleSpin = new QuantitySpinBox(QuantitySpinBox::Kind::Angle, unit, 3, &dialog);
    angleSpin->setObjectName("angle");
    angleSpin->setRange(-360.0, 360.0);
    form->addRow(tr("Angle:"), angleSpin);
    // Or held between limits (Phase 160): free within them.
    auto* held = new QComboBox(&dialog);
    held->setObjectName("held");
    held->addItems({tr("At the value"), tr("Between limits")});
    form->addRow(tr("Held:"), held);
    auto* lowLength = new QuantitySpinBox(QuantitySpinBox::Kind::Length, unit, 3, &dialog);
    auto* highLength = new QuantitySpinBox(QuantitySpinBox::Kind::Length, unit, 3, &dialog);
    auto* lowAngle = new QuantitySpinBox(QuantitySpinBox::Kind::Angle, unit, 3, &dialog);
    auto* highAngle = new QuantitySpinBox(QuantitySpinBox::Kind::Angle, unit, 3, &dialog);
    lowLength->setObjectName("minimum");
    highLength->setObjectName("maximum");
    lowAngle->setObjectName("minimumAngle");
    highAngle->setObjectName("maximumAngle");
    for (auto* spin : {lowLength, highLength}) spin->setRange(-1e6, 1e6);
    for (auto* spin : {lowAngle, highAngle}) spin->setRange(-360.0, 360.0);
    form->addRow(tr("At least:"), lowLength);
    form->addRow(tr("At most:"), highLength);
    form->addRow(tr("At least (angle):"), lowAngle);
    form->addRow(tr("At most (angle):"), highAngle);
    const auto offerValue = [typeCombo, valueSpin, angleSpin, held, lowLength, highLength, lowAngle,
                             highAngle] {
        const auto type = static_cast<doc::MateType>(typeCombo->currentData().toInt());
        const bool distance = type == doc::MateType::Distance;
        const bool angle = type == doc::MateType::Angle;
        const bool limited = held->currentIndex() == 1;
        held->setEnabled(distance || angle);
        valueSpin->setEnabled(distance && !limited);
        angleSpin->setEnabled(angle && !limited);
        lowLength->setEnabled(distance && limited);
        highLength->setEnabled(distance && limited);
        lowAngle->setEnabled(angle && limited);
        highAngle->setEnabled(angle && limited);
    };
    connect(held, &QComboBox::currentIndexChanged, &dialog, offerValue);
    connect(typeCombo, &QComboBox::currentIndexChanged, &dialog, offerValue);
    offerValue();

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    form->addRow(buttons);

    if (dialog.exec() != QDialog::Accepted) return;

    doc::Mate mate;
    mate.type = static_cast<doc::MateType>(typeCombo->currentData().toInt());
    mate.a = referenceOf(compACombo->currentData().toULongLong(), faceACombo->currentData());
    if (mate.type != doc::MateType::Fixed) {
        mate.b = referenceOf(compBCombo->currentData().toULongLong(), faceBCombo->currentData());
    }
    const bool angle = mate.type == doc::MateType::Angle;
    const double toRadians = std::numbers::pi / 180.0;
    mate.value = angle ? angleSpin->value() * toRadians : valueSpin->value();
    if (held->isEnabled() && held->currentIndex() == 1) {
        mate.minimum = angle ? lowAngle->value() * toRadians : lowLength->value();
        mate.maximum = angle ? highAngle->value() * toRadians : highLength->value();
        if (*mate.minimum > *mate.maximum) std::swap(*mate.minimum, *mate.maximum);
    }

    // The solve moves components; a mate that cannot be solved leaves the
    // assembly exactly as it was, and one that can is a single undo step.
    const bool wasDirty = assembly()->isDirty();
    doc::AssemblyState before = assembly()->snapshot();
    assembly()->addMate(std::move(mate));
    if (solveAssemblyMates(*assembly())) {
        recordAssemblyEdit(std::move(before), wasDirty, tr("Add Mate"));
    } else {
        assembly()->restore(std::move(before));
        assembly()->setDirty(wasDirty);
    }
    m_host.rebuildScene();
}

bool AssemblyWorkbench::editAssembly(const QString& verb, const std::function<bool()>& edit) {
    // Not under a drag (a shortcut while the mouse is held): it is put back
    // first, or its release would record over this edit.
    m_host.viewport().cancelComponentDrag();
    const bool wasDirty = assembly()->isDirty();
    doc::AssemblyState before = assembly()->snapshot();
    const bool made = edit();
    if (made && solveAssemblyMates(*assembly())) {
        recordAssemblyEdit(std::move(before), wasDirty, verb);
        m_host.rebuildScene();
        return true;
    }
    const QString why = m_host.currentStatus();
    assembly()->restore(std::move(before));
    assembly()->setDirty(wasDirty);
    m_host.rebuildScene();
    if (made) {
        m_host.showStatus(tr("%1 was not made: the mates cannot hold it (%2)").arg(verb, why));
    }
    return false;
}

uint64_t AssemblyWorkbench::targetComponent() const {
    for (const auto& pick : m_host.viewport().modelSelection()) {
        if (pick.owner != 0 && assembly() && assembly()->component(pick.owner)) return pick.owner;
    }
    return m_tree.currentComponent();
}

QComboBox* AssemblyWorkbench::componentChoice(FeatureForm& form, uint64_t target,
                                              std::vector<uint64_t>& ids) const {
    QStringList names;
    int chosen = 0;
    for (const auto& comp : assembly()->components()) {
        if (comp.id == target) chosen = static_cast<int>(ids.size());
        ids.push_back(comp.id);
        names << QStringLiteral("%1 (#%2)")
                     .arg(QString::fromStdString(comp.name.empty() ? "component" : comp.name))
                     .arg(comp.id);
    }
    auto* choice = form.choice(QStringLiteral("component"), tr("Component:"), names);
    choice->setCurrentIndex(chosen);
    return choice;
}

QComboBox* AssemblyWorkbench::mateChoice(FeatureForm& form, uint64_t target,
                                         std::vector<uint64_t>& ids) const {
    const auto nameOf = [this](uint64_t id) {
        const auto* comp = assembly()->component(id);
        return QString::fromStdString(comp == nullptr || comp->name.empty() ? "component"
                                                                            : comp->name);
    };
    QStringList names;
    int chosen = 0;
    for (const auto& mate : assembly()->mates()) {
        if (mate.id == target) chosen = static_cast<int>(ids.size());
        ids.push_back(mate.id);
        names << (mate.type == doc::MateType::Fixed
                      ? tr("%1: %2 (#%3)")
                            .arg(mateTypeName(mate.type), nameOf(mate.a.componentId))
                            .arg(mate.id)
                      : tr("%1: %2 and %3 (#%4)")
                            .arg(mateTypeName(mate.type), nameOf(mate.a.componentId),
                                 nameOf(mate.b.componentId))
                            .arg(mate.id));
    }
    auto* choice = form.choice(QStringLiteral("mate"), tr("Mate:"), names);
    choice->setCurrentIndex(chosen);
    return choice;
}

void AssemblyWorkbench::onMoveComponent() {
    const QString verb = tr("Move Component");
    if (!assembly() || assembly()->components().empty()) {
        m_host.showStatus(tr("%1 works on an assembly's components").arg(verb));
        return;
    }
    FeatureForm form(m_host.dialogParent(), verb, m_host.currentDocument()->lengthUnit());
    std::vector<uint64_t> ids;
    auto* which = componentChoice(form, targetComponent(), ids);
    auto* dx = form.length(QStringLiteral("dx"), tr("Move X:"), 0.0, -1e6, 1e6);
    auto* dy = form.length(QStringLiteral("dy"), tr("Move Y:"), 0.0, -1e6, 1e6);
    auto* dz = form.length(QStringLiteral("dz"), tr("Move Z:"), 0.0, -1e6, 1e6);
    if (!form.exec()) return;
    const uint64_t id = ids[static_cast<size_t>(std::max(which->currentIndex(), 0))];
    if (isFixed(*assembly(), id)) {
        m_host.showStatus(tr("%1: a Fixed mate holds it; remove the mate to move it").arg(verb));
        return;
    }
    const math::Vec3 by(dx->value(), dy->value(), dz->value());
    editAssembly(verb, [this, id, by] {
        auto* comp = assembly()->component(id);
        if (comp == nullptr) return false;
        comp->transform = math::Mat4::translation(by) * comp->transform;
        return true;
    });
}

void AssemblyWorkbench::onRotateComponent() {
    const QString verb = tr("Rotate Component");
    if (!assembly() || assembly()->components().empty()) {
        m_host.showStatus(tr("%1 works on an assembly's components").arg(verb));
        return;
    }
    FeatureForm form(m_host.dialogParent(), verb);
    std::vector<uint64_t> ids;
    auto* which = componentChoice(form, targetComponent(), ids);
    auto* axis = form.choice(QStringLiteral("axis"), tr("About:"), {tr("X"), tr("Y"), tr("Z")});
    axis->setCurrentIndex(2);
    auto* angle = form.angle(QStringLiteral("angle"), tr("Angle:"), 90.0, -360.0, 360.0);
    if (!form.exec()) return;
    const uint64_t id = ids[static_cast<size_t>(std::max(which->currentIndex(), 0))];
    if (isFixed(*assembly(), id)) {
        m_host.showStatus(tr("%1: a Fixed mate holds it; remove the mate to turn it").arg(verb));
        return;
    }
    const double radians = angle->value() * std::numbers::pi / 180.0;
    const int about = axis->currentIndex();
    editAssembly(verb, [this, id, radians, about] {
        auto* comp = assembly()->component(id);
        if (comp == nullptr) return false;
        // About its own middle, where it is.
        const math::BoundingBox box = placedBounds(*comp);
        const math::Vec3 middle = box.isValid() ? (box.min() + box.max()) * 0.5
                                                : comp->transform.transformPoint(math::Vec3());
        const math::Mat4 turn = about == 0   ? math::Mat4::rotationX(radians)
                                : about == 1 ? math::Mat4::rotationY(radians)
                                             : math::Mat4::rotationZ(radians);
        comp->transform = math::Mat4::translation(middle) * turn *
                          math::Mat4::translation(middle * -1.0) * comp->transform;
        return true;
    });
}

namespace {

/// The directions a step moves along, as offered.
struct ExplodeDirection {
    const char* name;
    double x, y, z;
    math::Vec3 direction() const { return {x, y, z}; }
};
constexpr std::array<ExplodeDirection, 6> kExplodeDirections{{
    {"+X", 1, 0, 0},
    {"-X", -1, 0, 0},
    {"+Y", 0, 1, 0},
    {"-Y", 0, -1, 0},
    {"+Z", 0, 0, 1},
    {"-Z", 0, 0, -1},
}};

QString componentLabel(const doc::ComponentInstance& comp) {
    return QStringLiteral("%1 (#%2)")
        .arg(QString::fromStdString(comp.name.empty() ? "component" : comp.name))
        .arg(comp.id);
}

}  // namespace

void AssemblyWorkbench::onExplodeComponents() {
    m_host.viewport().cancelComponentDrag();  // not under a drag
    const QString verb = tr("Explode Components");
    if (!assembly() || assembly()->components().empty()) {
        m_host.showStatus(tr("%1 works on an assembly's components").arg(verb));
        return;
    }
    doc::AssemblyDocument& asmDoc = *assembly();

    FeatureForm form(m_host.dialogParent(), verb, m_host.currentDocument()->lengthUnit());
    // Into a new view, or one there: the one shown, if any.
    QStringList viewNames{tr("New exploded view")};
    int shownRow = 0;
    for (const auto& view : asmDoc.explodedViews()) {
        if (view.id == asmDoc.shownView()) shownRow = static_cast<int>(viewNames.size());
        viewNames << QString::fromStdString(view.name);
    }
    auto* which = form.choice(QStringLiteral("view"), tr("Exploded view:"), viewNames);
    which->setCurrentIndex(shownRow);
    auto* name = form.text(QStringLiteral("name"), tr("New view's name:"),
                           tr("Exploded View %1").arg(asmDoc.explodedViews().size() + 1));

    // The components chosen in the view, or the tree's, are checked.
    std::set<uint64_t> chosen;
    for (const auto& pick : m_host.viewport().modelSelection()) {
        if (pick.owner != 0) chosen.insert(pick.owner);
    }
    if (chosen.empty() && targetComponent() != 0) chosen.insert(targetComponent());
    std::vector<std::pair<QString, QString>> rows;
    std::vector<uint64_t> ids;
    math::BoundingBox chosenBounds;
    for (const auto& comp : asmDoc.components()) {
        rows.emplace_back(componentLabel(comp), QString());
        ids.push_back(comp.id);
        if (chosen.count(comp.id) != 0) chosenBounds.expand(placedBounds(comp));
    }
    auto* components = form.checklist(QStringLiteral("components"), tr("Components:"), rows);
    for (size_t i = 0; i < ids.size(); ++i) {
        if (chosen.count(ids[i]) != 0) {
            components->item(static_cast<int>(i))->setCheckState(Qt::Checked);
        }
    }

    QStringList directionNames;
    for (const auto& d : kExplodeDirections) directionNames << QString::fromLatin1(d.name);
    auto* direction = form.choice(QStringLiteral("direction"), tr("Along:"), directionNames);
    direction->setCurrentIndex(4);  // +Z: up, off what they stand on
    // As far as they are big, at first: clear of where they were.
    double size = 10.0;
    if (chosenBounds.isValid()) {
        const math::Vec3 extent = chosenBounds.max() - chosenBounds.min();
        size = std::max({extent.x, extent.y, extent.z, 1e-3});
    }
    auto* distance = form.length(QStringLiteral("distance"), tr("Distance:"), size, 0.0, 1e6);
    if (!form.exec()) return;

    doc::ExplodeStep step;
    for (const int row : FeatureForm::checkedRows(components)) {
        step.components.push_back(ids.at(static_cast<size_t>(row)));
    }
    if (step.components.empty()) {
        m_host.showStatus(tr("%1: check the components to move").arg(verb));
        return;
    }
    if (!(distance->value() > 0.0)) {
        m_host.showStatus(tr("%1: the distance must be more than 0").arg(verb));
        return;
    }
    step.direction =
        kExplodeDirections.at(static_cast<size_t>(std::max(direction->currentIndex(), 0)))
            .direction();
    step.distance = distance->value();

    const bool wasDirty = asmDoc.isDirty();
    doc::AssemblyState before = asmDoc.snapshot();
    uint64_t viewId = 0;
    const int row = which->currentIndex();
    if (row <= 0) {
        doc::ExplodedView view;
        view.name = name->text().trimmed().toStdString();
        if (view.name.empty()) view.name = tr("Exploded View").toStdString();
        viewId = asmDoc.addExplodedView(std::move(view));
    } else {
        viewId = asmDoc.explodedViews().at(static_cast<size_t>(row - 1)).id;
    }
    asmDoc.explodedView(viewId)->steps.push_back(std::move(step));
    // Pushed, the edit is made again from its snapshot: the views are new.
    recordAssemblyEdit(std::move(before), wasDirty, verb);
    asmDoc.setShownView(viewId);
    showPlacements(asmDoc);
    if (const doc::ExplodedView* view = asmDoc.explodedView(viewId)) {
        m_host.showStatus(tr("%1: step %2 of %3")
                              .arg(verb)
                              .arg(view->steps.size())
                              .arg(QString::fromStdString(view->name)));
    }
}

void AssemblyWorkbench::onShowExplodedView() {
    m_host.viewport().cancelComponentDrag();  // one under way was placed unexploded
    const QString verb = tr("Show Exploded View");
    if (!assembly()) {
        m_host.showStatus(tr("%1 is only available in an assembly document").arg(verb));
        return;
    }
    doc::AssemblyDocument& asmDoc = *assembly();
    if (asmDoc.explodedViews().empty()) {
        m_host.showStatus(
            tr("%1: the assembly has no exploded view; Explode Components makes one").arg(verb));
        return;
    }
    FeatureForm form(m_host.dialogParent(), verb);
    QStringList names{tr("None: the components where they are")};
    int shownRow = 0;
    for (const auto& view : asmDoc.explodedViews()) {
        if (view.id == asmDoc.shownView()) shownRow = static_cast<int>(names.size());
        names << QString::fromStdString(view.name);
    }
    auto* which = form.choice(QStringLiteral("view"), tr("Show:"), names);
    which->setCurrentIndex(shownRow);
    if (!form.exec()) return;
    const int row = which->currentIndex();
    const uint64_t id = row <= 0 ? 0 : asmDoc.explodedViews().at(static_cast<size_t>(row - 1)).id;
    asmDoc.setShownView(id);
    showPlacements(asmDoc);
    m_host.showStatus(id == 0 ? tr("The assembly is shown unexploded")
                              : tr("%1 is shown").arg(names.at(row)));
}

void AssemblyWorkbench::onRemoveExplodedView() {
    m_host.viewport().cancelComponentDrag();
    const QString verb = tr("Remove Exploded View");
    if (!assembly() || assembly()->explodedViews().empty()) {
        m_host.showStatus(tr("%1: the assembly has no exploded view").arg(verb));
        return;
    }
    doc::AssemblyDocument& asmDoc = *assembly();
    FeatureForm form(m_host.dialogParent(), verb);
    QStringList names;
    int shownRow = 0;
    for (const auto& view : asmDoc.explodedViews()) {
        if (view.id == asmDoc.shownView()) shownRow = static_cast<int>(names.size());
        names << QString::fromStdString(view.name);
    }
    auto* which = form.choice(QStringLiteral("view"), tr("Exploded view:"), names);
    which->setCurrentIndex(shownRow);
    if (!form.exec()) return;
    const auto row = static_cast<size_t>(std::max(which->currentIndex(), 0));
    const uint64_t id = asmDoc.explodedViews().at(row).id;
    const bool wasDirty = asmDoc.isDirty();
    doc::AssemblyState before = asmDoc.snapshot();
    asmDoc.removeExplodedView(id);
    recordAssemblyEdit(std::move(before), wasDirty, verb);
    showPlacements(asmDoc);
    m_host.showStatus(tr("%1 removed").arg(names.at(static_cast<int>(row))));
}

void AssemblyWorkbench::removeComponent(uint64_t id) {
    if (!assembly() || assembly()->component(id) == nullptr) return;
    editAssembly(tr("Remove Component"), [this, id] { return assembly()->removeComponent(id); });
}

void AssemblyWorkbench::setComponentSuppressed(uint64_t id, bool suppressed) {
    if (!assembly() || assembly()->component(id) == nullptr) return;
    editAssembly(suppressed ? tr("Suppress Component") : tr("Unsuppress Component"),
                 [this, id, suppressed] {
                     assembly()->component(id)->suppressed = suppressed;
                     return true;
                 });
}

void AssemblyWorkbench::renameComponent(uint64_t id) {
    if (!assembly() || assembly()->component(id) == nullptr) return;
    const QString verb = tr("Rename Component");
    FeatureForm form(m_host.dialogParent(), verb);
    auto* name = form.text(QStringLiteral("name"), tr("Name:"),
                           QString::fromStdString(assembly()->component(id)->name));
    if (!form.exec()) return;
    const std::string text = name->text().trimmed().toStdString();
    if (text.empty()) {
        m_host.showStatus(tr("%1: a component needs a name").arg(verb));
        return;
    }
    editAssembly(verb, [this, id, text] {
        assembly()->component(id)->name = text;
        return true;
    });
}

void AssemblyWorkbench::editMate(uint64_t id) {
    if (!assembly() || assembly()->mate(id) == nullptr) return;
    const QString verb = tr("Edit Mate");
    const doc::Mate& mate = *assembly()->mate(id);
    const bool angle = mate.type == doc::MateType::Angle;
    if (!angle && mate.type != doc::MateType::Distance) {
        m_host.showStatus(tr("%1: a %2 mate has no value").arg(verb, mateTypeName(mate.type)));
        return;
    }
    FeatureForm form(m_host.dialogParent(), verb, m_host.currentDocument()->lengthUnit());
    const double toDegrees = 180.0 / std::numbers::pi;
    const auto field = [&](const QString& name, const QString& label, double shown) {
        return angle ? form.angle(name, label, shown * toDegrees, -1e6, 1e6)
                     : form.length(name, label, shown, -1e6, 1e6);
    };
    QuantitySpinBox* value =
        field(QStringLiteral("value"), angle ? tr("Angle:") : tr("Distance:"), mate.value);
    // Or between limits (Phase 160), free within them.
    auto* held = form.choice(QStringLiteral("held"), tr("Held:"),
                             {tr("At the value"), tr("Between limits")});
    held->setCurrentIndex(mate.minimum || mate.maximum ? 1 : 0);
    QuantitySpinBox* low =
        field(QStringLiteral("minimum"), tr("At least:"), mate.minimum.value_or(mate.value));
    QuantitySpinBox* high =
        field(QStringLiteral("maximum"), tr("At most:"), mate.maximum.value_or(mate.value));
    // Only what is held is offered, as Add Mate does.
    const auto offer = [value, held, low, high] {
        const bool limited = held->currentIndex() == 1;
        value->setEnabled(!limited);
        low->setEnabled(limited);
        high->setEnabled(limited);
    };
    connect(held, &QComboBox::currentIndexChanged, &form.dialog(), offer);
    offer();
    if (!form.exec()) return;
    const double scale = angle ? 1.0 / toDegrees : 1.0;
    const double set = value->value() * scale;
    std::optional<double> minimum;
    std::optional<double> maximum;
    if (held->currentIndex() == 1) {
        minimum = std::min(low->value(), high->value()) * scale;
        maximum = std::max(low->value(), high->value()) * scale;
    }
    editAssembly(verb, [this, id, set, minimum, maximum] {
        doc::Mate* edited = assembly()->mate(id);
        edited->value = set;
        edited->minimum = minimum;
        edited->maximum = maximum;
        return true;
    });
}

void AssemblyWorkbench::removeMate(uint64_t id) {
    if (!assembly() || assembly()->mate(id) == nullptr) return;
    editAssembly(tr("Remove Mate"), [this, id] { return assembly()->removeMate(id); });
}

void AssemblyWorkbench::onRemoveComponent() {
    if (!assembly() || assembly()->components().empty()) {
        m_host.showStatus(tr("Remove Component works on an assembly's components"));
        return;
    }
    uint64_t id = targetComponent();
    if (id == 0) {
        FeatureForm form(m_host.dialogParent(), tr("Remove Component"));
        std::vector<uint64_t> ids;
        auto* which = componentChoice(form, 0, ids);
        if (!form.exec()) return;
        id = ids[static_cast<size_t>(std::max(which->currentIndex(), 0))];
    }
    removeComponent(id);
}

void AssemblyWorkbench::onSuppressComponent() {
    if (!assembly() || assembly()->components().empty()) {
        m_host.showStatus(tr("Suppress Component works on an assembly's components"));
        return;
    }
    uint64_t id = targetComponent();
    if (id == 0) {
        FeatureForm form(m_host.dialogParent(), tr("Suppress Component"));
        std::vector<uint64_t> ids;
        auto* which = componentChoice(form, 0, ids);
        if (!form.exec()) return;
        id = ids[static_cast<size_t>(std::max(which->currentIndex(), 0))];
    }
    const auto* comp = assembly()->component(id);
    if (comp != nullptr) setComponentSuppressed(id, !comp->suppressed);
}

void AssemblyWorkbench::onRenameComponent() {
    if (!assembly() || assembly()->components().empty()) {
        m_host.showStatus(tr("Rename Component works on an assembly's components"));
        return;
    }
    uint64_t id = targetComponent();
    if (id == 0) {
        FeatureForm form(m_host.dialogParent(), tr("Rename Component"));
        std::vector<uint64_t> ids;
        auto* which = componentChoice(form, 0, ids);
        if (!form.exec()) return;
        id = ids[static_cast<size_t>(std::max(which->currentIndex(), 0))];
    }
    renameComponent(id);
}

void AssemblyWorkbench::onEditMate() {
    if (!assembly() || assembly()->mates().empty()) {
        m_host.showStatus(tr("Edit Mate: the assembly has no mates"));
        return;
    }
    FeatureForm form(m_host.dialogParent(), tr("Choose Mate"));
    std::vector<uint64_t> ids;
    auto* which = mateChoice(form, m_tree.currentMate(), ids);
    if (!form.exec()) return;
    editMate(ids[static_cast<size_t>(std::max(which->currentIndex(), 0))]);
}

void AssemblyWorkbench::onRemoveMate() {
    if (!assembly() || assembly()->mates().empty()) {
        m_host.showStatus(tr("Remove Mate: the assembly has no mates"));
        return;
    }
    FeatureForm form(m_host.dialogParent(), tr("Remove Mate"));
    std::vector<uint64_t> ids;
    auto* which = mateChoice(form, m_tree.currentMate(), ids);
    if (!form.exec()) return;
    removeMate(ids[static_cast<size_t>(std::max(which->currentIndex(), 0))]);
}

void AssemblyWorkbench::refreshComponentsOf(const std::string& path, bool report) {
    if (path.empty()) return;
    // What was read of the file is out of date: the next component to
    // resolve it reads it again. (A part open in a tab stays: it is the part.)
    m_host.documents().releasePart(path);
    int refreshed = 0;
    bool solved = true;
    bool moved = false;
    bool shown = false;
    for (const std::shared_ptr<doc::AssemblyDocument>& tabAssembly : m_host.openAssemblies()) {
        const std::string dir = assemblyDir(*tabAssembly);
        bool places = false;
        for (auto& comp : tabAssembly->components()) {
            if (!placesFile(comp, dir, path)) continue;  // at any depth (Phase 159)
            comp.cachedMesh.reset();
            comp.resolvedPart.reset();
            comp.resolvedAssembly.reset();
            comp.assemblySolid.reset();
            comp.state = doc::ComponentState::Lightweight;
            places = true;
        }
        if (!places) continue;
        ++refreshed;
        // Its faces may be elsewhere now: the mates place the components
        // again. That is a change to the assembly, to be saved.
        const doc::AssemblyState placed = tabAssembly->snapshot();
        solved = solveAssemblyMates(*tabAssembly, /*reportSuccess=*/false) && solved;
        if (placementsDiffer(placed, *tabAssembly)) {
            tabAssembly->setDirty(true);
            moved = true;
        }
        shown = shown || tabAssembly.get() == assembly();
    }
    if (refreshed == 0) return;
    if (shown) m_host.rebuildScene();  // reads the meshes again
    if (moved) m_host.refreshModifiedIndicators();
    // A mate the part no longer has a face for is reported by the solve.
    if (solved && report) {
        m_host.showStatus(tr("\"%1\" changed: the assemblies placing it show it as it is now")
                              .arg(QFileInfo(QString::fromStdString(path)).fileName()),
                          10000);
    }
}

void AssemblyWorkbench::openComponentPart(uint64_t id) {
    const auto* comp = assembly() ? assembly()->component(id) : nullptr;
    if (comp == nullptr) {
        m_host.showStatus(tr("Open Part: click a component, or choose one in the assembly tree"));
        return;
    }
    if (comp->partPath.empty()) {
        m_host.showStatus(tr("Open Part: the component has no part file"));
        return;
    }
    m_host.openPath(QString::fromStdString(partFile(*comp, assemblyDir(*assembly()))));
}

void AssemblyWorkbench::onOpenPart() {
    if (!assembly()) {
        m_host.showStatus(tr("Open Part is only available in an assembly document"));
        return;
    }
    openComponentPart(targetComponent());
}

void AssemblyWorkbench::onBillOfMaterials() {
    if (!assembly()) {
        m_host.showStatus(tr("Bill of Materials is only available in an assembly document"));
        return;
    }
    doc::BillOfMaterials bom = doc::BomGenerator::generate(*assembly());

    QDialog dialog(m_host.dialogParent());
    dialog.setWindowTitle(tr("Bill of Materials"));
    auto* layout = new QVBoxLayout(&dialog);
    // Which BOM (Phase 159): the assembly's own components, each
    // subassembly's under it, or the parts at every depth.
    auto* kind = new QComboBox(&dialog);
    kind->setObjectName(QStringLiteral("bomKind"));
    kind->addItems({tr("Top level"), tr("Indented"), tr("Parts only")});
    auto* shows = new QHBoxLayout();
    shows->addWidget(new QLabel(tr("Show:"), &dialog));
    shows->addWidget(kind);
    shows->addStretch();
    layout->addLayout(shows);
    auto* table = new QTableWidget(0, 3, &dialog);
    table->setObjectName(QStringLiteral("bom"));
    table->setHorizontalHeaderLabels({tr("Item"), tr("Part"), tr("Quantity")});
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->verticalHeader()->hide();
    table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    layout->addWidget(table);
    auto* total = new QLabel(&dialog);
    layout->addWidget(total);
    const auto fill = [&bom, table, total] {
        table->setRowCount(static_cast<int>(bom.lines.size()));
        for (size_t i = 0; i < bom.lines.size(); ++i) {
            const doc::BomLine& line = bom.lines[i];
            const int row = static_cast<int>(i);
            table->setItem(
                row, 0,
                new QTableWidgetItem(line.index.empty() ? QString::number(line.item)
                                                        : QString::fromStdString(line.index)));
            // Under its assembly: indented as deep as it is.
            auto* part = new QTableWidgetItem(QString(line.level * 4, QLatin1Char(' ')) +
                                              QString::fromStdString(line.partName));
            part->setToolTip(QString::fromStdString(line.partPath));
            table->setItem(row, 1, part);
            table->setItem(row, 2, new QTableWidgetItem(QString::number(line.quantity)));
        }
        total->setText(
            tr("%n component(s) in all; suppressed ones are left out.", "", bom.totalQuantity()));
    };
    fill();
    connect(kind, &QComboBox::currentIndexChanged, &dialog, [this, &bom, fill](int index) {
        const std::array kinds{doc::BomKind::TopLevel, doc::BomKind::Indented,
                               doc::BomKind::PartsOnly};
        bom = doc::BomGenerator::generate(*assembly(),
                                          kinds.at(static_cast<size_t>(std::clamp(index, 0, 2))));
        fill();
    });

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);
    QPushButton* exportButton =
        buttons->addButton(tr("Export CSV..."), QDialogButtonBox::ActionRole);
    exportButton->setObjectName(QStringLiteral("exportBom"));
    exportButton->setEnabled(!bom.lines.empty());
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    const QString assemblyPath = QString::fromStdString(assembly()->filePath());
    connect(exportButton, &QPushButton::clicked, &dialog, [this, &dialog, &bom, assemblyPath] {
        const QFileInfo from(assemblyPath);
        const QString suggested = assemblyPath.isEmpty()
                                      ? QStringLiteral("bom.csv")
                                      : from.dir().filePath(from.completeBaseName() + "-bom.csv");
        const QString file =
            QFileDialog::getSaveFileName(&dialog, tr("Export Bill of Materials"), suggested,
                                         tr("CSV Files (*.csv);;All Files (*)"));
        if (file.isEmpty()) return;
        if (!io::BomExport::toCsv(file.toStdString(), bom)) {
            m_host.reportFileError(tr("Could not export"), file.toStdString(),
                                   "the file could not be written");
            return;
        }
        m_host.showStatus(
            tr("Bill of materials exported to \"%1\"").arg(QFileInfo(file).fileName()));
    });
    layout->addWidget(buttons);
    dialog.resize(480, 320);
    dialog.exec();
}

}  // namespace hz::ui
