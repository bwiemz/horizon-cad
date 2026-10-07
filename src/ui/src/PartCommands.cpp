#include "horizon/ui/PartCommands.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QListWidget>
#include <QSpinBox>
#include <QStringList>
#include <algorithm>
#include <map>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/FacePlane.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/ReferenceGeometry.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/render/Camera.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/FeatureForm.h"
#include "horizon/ui/ModelPicks.h"
#include "horizon/ui/QuantitySpinBox.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/ui/WorkbenchHost.h"

namespace hz::ui {

PartCommands::PartCommands(WorkbenchHost& host, QObject* parent) : QObject(parent), m_host(host) {}

doc::Document& PartCommands::part() {
    return *m_host.currentDocument();
}

// ---------------------------------------------------------------------------
// Primitives
// ---------------------------------------------------------------------------

bool PartCommands::requirePart(const QString& verb) {
    if (m_host.currentAssembly()) {
        m_host.showStatus(tr("%1 works on a part; open or create one").arg(verb));
        return false;
    }
    return true;
}

doc::BodyOperation PartCommands::proposedOperation() {
    // Joining is what a second body usually means; the first has nothing to
    // join, so it starts one.
    return part().solid() ? doc::BodyOperation::Join : doc::BodyOperation::NewBody;
}

void PartCommands::addPrimitive(
    const QString& verb, const std::vector<PrimitiveField>& fields,
    const std::function<std::unique_ptr<doc::PrimitiveFeature>(const std::vector<double>&)>& make) {
    if (!requirePart(verb)) return;
    FeatureForm form(m_host.dialogParent(), verb, part().lengthUnit());
    std::vector<QDoubleSpinBox*> sizes;
    sizes.reserve(fields.size());
    for (size_t i = 0; i < fields.size(); ++i) {
        sizes.push_back(form.length(QStringLiteral("size%1").arg(i), fields[i].label,
                                    fields[i].value, fields[i].min, 1e6));
    }
    // Where it stands: from a base point, its own z axis along one of the
    // axis directions (Phase 134; primitives always stood at the origin).
    auto* atX = form.length(QStringLiteral("atX"), tr("At x:"), 0.0, -1e6, 1e6);
    auto* atY = form.length(QStringLiteral("atY"), tr("y:"), 0.0, -1e6, 1e6);
    auto* atZ = form.length(QStringLiteral("atZ"), tr("z:"), 0.0, -1e6, 1e6);
    auto* axis =
        directionChoice(form, QStringLiteral("axis"), tr("Standing along:"), QStringLiteral("+Z"));
    auto* result = form.operationChoice(proposedOperation());
    if (!form.exec()) return;

    std::vector<double> values;
    values.reserve(sizes.size());
    for (const auto* spin : sizes) values.push_back(spin->value());
    auto feature = make(values);
    feature->setVector("basePoint", math::Vec3(atX->value(), atY->value(), atZ->value()));
    feature->setVector("axisDirection", chosenDirection(axis));
    feature->setOperation(FeatureForm::operation(result));
    m_host.addFeature(std::move(feature), verb);
}

void PartCommands::onPrimitiveBox() {
    // From the origin to (width, height, depth).
    addPrimitive(tr("Box"),
                 {{tr("Width (X):"), 10.0, 0.001},
                  {tr("Height (Y):"), 10.0, 0.001},
                  {tr("Depth (Z):"), 10.0, 0.001}},
                 [](const std::vector<double>& v) {
                     return doc::PrimitiveFeature::makeBox(v[0], v[1], v[2]);
                 });
}

void PartCommands::onPrimitiveCylinder() {
    addPrimitive(tr("Cylinder"), {{tr("Radius:"), 5.0, 0.001}, {tr("Height:"), 10.0, 0.001}},
                 [](const std::vector<double>& v) {
                     return doc::PrimitiveFeature::makeCylinder(v[0], v[1]);
                 });
}

void PartCommands::onPrimitiveSphere() {
    addPrimitive(tr("Sphere"), {{tr("Radius:"), 5.0, 0.001}}, [](const std::vector<double>& v) {
        return doc::PrimitiveFeature::makeSphere(v[0]);
    });
}

void PartCommands::onPrimitiveCone() {
    // A top radius of 0 is a pointed cone.
    addPrimitive(tr("Cone"),
                 {{tr("Bottom radius:"), 5.0, 0.0},
                  {tr("Top radius:"), 0.0, 0.0},
                  {tr("Height:"), 10.0, 0.001}},
                 [](const std::vector<double>& v) {
                     return doc::PrimitiveFeature::makeCone(v[0], v[1], v[2]);
                 });
}

void PartCommands::onPrimitiveTorus() {
    addPrimitive(
        tr("Torus"), {{tr("Ring radius:"), 6.0, 0.001}, {tr("Tube radius:"), 2.0, 0.001}},
        [](const std::vector<double>& v) { return doc::PrimitiveFeature::makeTorus(v[0], v[1]); });
}

// ---------------------------------------------------------------------------
// From sketches: Extrude, Revolve, Loft, Sweep and datums
// ---------------------------------------------------------------------------

namespace {

/// Whether @p sketch is on the plane a sketch made of the drawing is: XY,
/// following no face.
bool onWrapperPlane(const doc::Sketch& sketch) {
    const draft::SketchPlane xy;
    const draft::SketchPlane& plane = sketch.drawnPlane();
    const auto same = [](const math::Vec3& a, const math::Vec3& b) {
        return (a - b).length() < 1e-12;
    };
    return sketch.face().empty() && same(plane.origin(), xy.origin()) &&
           same(plane.normal(), xy.normal()) && same(plane.xAxis(), xy.xAxis());
}

}  // namespace

std::shared_ptr<doc::Sketch> PartCommands::resolveProfileSketch(bool& createdWrapper) {
    createdWrapper = false;

    // The sketch being edited, finished; else the one chosen in the list, or
    // last made or finished.
    if (auto edited = part().editedSketch()) {
        m_host.finishSketch();
        return edited;
    }
    if (auto chosen = m_host.chosenSketch(); chosen && !chosen->entities().empty()) {
        return chosen;
    }

    // Otherwise the drawing itself, as it is shown: what is on a hidden
    // layer is not part of it. (Notes on it are passed over by the profile.)
    std::vector<std::shared_ptr<draft::DraftEntity>> shown;
    for (const auto& entity : part().draftDocument().entities()) {
        const auto* layer = part().layerManager().getLayer(entity->layer());
        if (layer == nullptr || layer->visible) shown.push_back(entity);
    }
    if (shown.empty()) return nullptr;

    // Reuse an existing wrapper sketch when the top-level profile has not
    // changed — repeated extrudes must not accumulate duplicate sketches.
    // It holds copies, so it is the same drawing, on the plane a wrapper is
    // made on, that is reused.
    for (const auto& sk : part().sketches()) {
        if (onWrapperPlane(*sk) && doc::drawSame(sk->entities(), shown)) return sk;
    }

    // Wrap the top-level profile in a sketch so the feature is replayable
    // (parametric history requires a sketch reference). The caller must add
    // it to the document only once the operation is validated. Copies: the
    // drawing's own entities, shared, moved the feature's profile with every
    // edit made in the drawing in place, and kept it where a grip edit,
    // which puts a new entity in the old one's place, left it.
    auto sketch = std::make_shared<doc::Sketch>();
    sketch->setName(tr("Profile %1").arg(part().sketches().size() + 1).toStdString());
    for (const auto& entity : shown) sketch->addEntity(entity->clone());
    createdWrapper = true;
    return sketch;
}

void PartCommands::onLoft() {
    if (m_host.currentAssembly()) return;
    if (part().editedSketch()) m_host.finishSketch();
    std::vector<std::shared_ptr<doc::Sketch>> drawn;
    std::vector<std::pair<QString, QString>> items;
    for (const auto& sketch : part().sketches()) {
        if (sketch->entities().empty()) continue;
        drawn.push_back(sketch);
        items.emplace_back(
            QString::fromStdString(sketch->name()),
            tr("on the plane through %1").arg(formatPoint(sketch->plane().origin())));
    }
    if (drawn.size() < 2) {
        m_host.showStatus(tr("A loft joins two or more sketches: make them first"));
        return;
    }
    FeatureForm form(m_host.dialogParent(), tr("Loft"), part().lengthUnit());
    auto* list = form.checklist(QStringLiteral("sections"), tr("Sections, in order:"), items);
    auto* result = form.operationChoice(proposedOperation());
    if (!form.exec()) return;
    std::vector<std::shared_ptr<doc::Sketch>> sections;
    for (const int row : FeatureForm::checkedRows(list)) {
        sections.push_back(drawn[static_cast<size_t>(row)]);
    }
    if (sections.size() < 2) {
        m_host.showStatus(tr("Loft not added: choose two sections or more"));
        return;
    }
    auto feature = std::make_unique<doc::LoftFeature>(std::move(sections));
    feature->setOperation(FeatureForm::operation(result));
    m_host.addFeature(std::move(feature), tr("Loft"));
}

void PartCommands::onSweep() {
    if (m_host.currentAssembly()) return;
    if (part().editedSketch()) m_host.finishSketch();
    std::vector<std::shared_ptr<doc::Sketch>> drawn;
    QStringList names;
    for (const auto& sketch : part().sketches()) {
        if (sketch->entities().empty()) continue;
        drawn.push_back(sketch);
        names << QString::fromStdString(sketch->name());
    }
    if (drawn.size() < 2) {
        m_host.showStatus(
            tr("A sweep takes a profile sketch along a path sketch: make them first"));
        return;
    }
    FeatureForm form(m_host.dialogParent(), tr("Sweep"), part().lengthUnit());
    auto* profile = form.choice(QStringLiteral("profile"), tr("Profile:"), names);
    auto* path = form.choice(QStringLiteral("path"), tr("Path:"), names);
    path->setCurrentIndex(1);
    auto* result = form.operationChoice(proposedOperation());
    if (!form.exec()) return;
    if (profile->currentIndex() == path->currentIndex()) {
        m_host.showStatus(tr("Sweep not added: the profile and the path are one sketch"));
        return;
    }
    auto feature =
        std::make_unique<doc::SweepFeature>(drawn[static_cast<size_t>(profile->currentIndex())],
                                            drawn[static_cast<size_t>(path->currentIndex())]);
    feature->setOperation(FeatureForm::operation(result));
    m_host.addFeature(std::move(feature), tr("Sweep"));
}

void PartCommands::onDatumPlane() {
    if (m_host.currentAssembly()) return;
    // Principal planes, and a flat face clicked on the part.
    std::vector<std::pair<QString, model::DatumPlane>> bases = {
        {tr("The XY plane"), {math::Vec3::Zero, math::Vec3::UnitZ, math::Vec3::UnitX}},
        {tr("The XZ plane"), {math::Vec3::Zero, math::Vec3(0, -1, 0), math::Vec3::UnitX}},
        {tr("The YZ plane"), {math::Vec3::Zero, math::Vec3::UnitX, math::Vec3::UnitY}},
    };
    if (part().solid()) {
        for (const auto& pick : m_host.viewport().modelSelection()) {
            if (pick.edge || pick.owner != 0) continue;
            for (const auto& face : planarFacesOf(*part().solid())) {
                if (face.tag != pick.tag) continue;
                bases.insert(bases.begin(),
                             {tr("The face clicked"),
                              {face.plane.origin(), face.plane.normal(), face.plane.xAxis()}});
                break;
            }
            break;
        }
    }
    QStringList names;
    for (const auto& [name, plane] : bases) names << name;
    FeatureForm form(m_host.dialogParent(), tr("Datum Plane"), part().lengthUnit());
    auto* base = form.choice(QStringLiteral("base"), tr("From:"), names);
    auto* offset =
        form.length(QStringLiteral("offset"), tr("Offset along its normal:"), 10.0, -1e6, 1e6);
    auto* angle =
        form.angle(QStringLiteral("angle"), tr("Turned about its x axis:"), 0.0, -360.0, 360.0, 3);
    if (!form.exec()) return;
    model::DatumPlane plane = bases[static_cast<size_t>(std::max(base->currentIndex(), 0))].second;
    if (angle->value() != 0.0) {
        plane = model::refgeo::planeAtAngle(plane, plane.origin, plane.xAxis,
                                            angle->value() * math::kDegToRad);
    }
    plane = model::refgeo::planeOffset(plane, offset->value());
    m_host.addFeature(doc::DatumFeature::makePlane(plane), tr("Datum Plane"));
}

void PartCommands::onDatumAxis() {
    if (m_host.currentAssembly()) return;
    std::vector<std::pair<QString, math::Vec3>> directions = {
        {tr("X"), math::Vec3::UnitX}, {tr("Y"), math::Vec3::UnitY}, {tr("Z"), math::Vec3::UnitZ}};
    if (part().solid()) {
        if (const auto clicked =
                clickedDirection(*part().solid(), m_host.viewport().modelSelection())) {
            directions.insert(directions.begin(), {tr("As the face or edge clicked"), *clicked});
        }
    }
    QStringList names;
    for (const auto& [name, direction] : directions) names << name;
    FeatureForm form(m_host.dialogParent(), tr("Datum Axis"), part().lengthUnit());
    auto* along = form.choice(QStringLiteral("direction"), tr("Along:"), names);
    auto* x = form.length(QStringLiteral("x"), tr("Through x:"), 0.0, -1e6, 1e6);
    auto* y = form.length(QStringLiteral("y"), tr("y:"), 0.0, -1e6, 1e6);
    auto* z = form.length(QStringLiteral("z"), tr("z:"), 0.0, -1e6, 1e6);
    if (!form.exec()) return;
    const auto axis = model::refgeo::axisFromDirection(
        math::Vec3(x->value(), y->value(), z->value()),
        directions[static_cast<size_t>(std::max(along->currentIndex(), 0))].second);
    m_host.addFeature(doc::DatumFeature::makeAxis(axis), tr("Datum Axis"));
}

void PartCommands::onDatumPoint() {
    if (m_host.currentAssembly()) return;
    FeatureForm form(m_host.dialogParent(), tr("Datum Point"), part().lengthUnit());
    auto* x = form.length(QStringLiteral("x"), tr("x:"), 0.0, -1e6, 1e6);
    auto* y = form.length(QStringLiteral("y"), tr("y:"), 0.0, -1e6, 1e6);
    auto* z = form.length(QStringLiteral("z"), tr("z:"), 0.0, -1e6, 1e6);
    if (!form.exec()) return;
    m_host.addFeature(doc::DatumFeature::makePoint(
                          model::refgeo::pointAt(math::Vec3(x->value(), y->value(), z->value()))),
                      tr("Datum Point"));
}

void PartCommands::onExtrudeSketch() {
    if (m_host.currentDocument() == nullptr) return;

    bool createdWrapper = false;
    auto sketch = resolveProfileSketch(createdWrapper);
    if (!sketch || sketch->entities().empty()) {
        m_host.showStatus(tr("Draw a closed profile first"));
        return;
    }

    // The part's flat faces parallel to the sketch, to go up to (Phase 157):
    // the one clicked, if any, first.
    std::vector<PlaneChoice> faces;
    if (const topo::Solid* body = part().solid()) {
        for (auto& face : planarFacesOf(*body)) {
            if (face.plane.normal().cross(sketch->plane().normal()).length() > 1e-9) continue;
            faces.push_back(std::move(face));
        }
    }
    const auto clicked = std::find_if(faces.begin(), faces.end(), [this](const PlaneChoice& f) {
        const auto& picks = m_host.viewport().modelSelection();
        return std::any_of(picks.begin(), picks.end(), [&f](const ViewportWidget::ModelPick& p) {
            return p.owner == 0 && !p.edge && p.tag == f.tag;
        });
    });
    if (clicked != faces.end()) std::rotate(faces.begin(), clicked, clicked + 1);

    FeatureForm form(m_host.dialogParent(), tr("Extrude"), part().lengthUnit());
    auto* size = form.length(QStringLiteral("size"), tr("Distance:"), 10.0, 0.01, 1e6, 2);
    auto* goes = form.choice(QStringLiteral("extent"), tr("Goes:"),
                             {tr("To the distance"), tr("Both ways, half each"), tr("Through all"),
                              tr("Through all, both ways"), tr("Up to a face")});
    QStringList faceNames;
    for (const auto& face : faces) faceNames << face.text;
    if (faceNames.isEmpty())
        faceNames << tr("(no flat face of the part is parallel to the sketch)");
    auto* upTo = form.choice(QStringLiteral("upToFace"), tr("Up to face:"), faceNames);
    auto* way = form.choice(QStringLiteral("way"), tr("Direction:"),
                            {tr("Out of the sketch"), tr("Reversed")});
    auto* result = form.operationChoice(proposedOperation());
    if (!form.exec()) return;
    const double distance = size->value();
    const auto extent = static_cast<doc::ExtrudeFeature::Extent>(goes->currentIndex());
    const doc::BodyOperation operation = FeatureForm::operation(result);
    std::string upToFace;
    if (extent == doc::ExtrudeFeature::Extent::UpToFace) {
        if (faces.empty()) {
            m_host.showStatus(
                tr("Extrude not added: no flat face of the part is parallel to the sketch"));
            return;
        }
        const auto& chosen = faces[static_cast<size_t>(std::max(upTo->currentIndex(), 0))];
        upToFace = model::wholeFaceName(chosen.tag);
    }

    // As the sketch was drawn: one placed on a face takes it along (Phase 157).
    const draft::SketchPlane& drawn = sketch->drawnPlane();
    const math::Vec3 direction = drawn.normal() * (way->currentIndex() == 1 ? -1.0 : 1.0);

    // Validate the profile BEFORE mutating the document: a failed extrude
    // must not leave a wrapper sketch or a dead feature behind.
    std::string why;
    auto probe = model::Extrude::execute(sketch->entities(), drawn, direction, distance, "probe",
                                         model::Extrude::kDefaultSegments, 0.0, &why);
    if (!probe) {
        m_host.showStatus(tr("Extrude failed: %1").arg(QString::fromStdString(why)));
        return;
    }

    auto feature = std::make_unique<doc::ExtrudeFeature>(sketch, direction, distance);
    feature->setExtent(extent);
    feature->setUpToFace(upToFace);
    feature->setOperation(operation);
    if (!m_host.addFeature(std::move(feature), tr("Extrude"), createdWrapper ? sketch : nullptr)) {
        return;
    }

    m_host.viewport().camera().setIsometricView();
    m_host.viewport().update();
}

void PartCommands::onRevolveSketch() {
    if (m_host.currentDocument() == nullptr) return;

    bool createdWrapper = false;
    auto sketch = resolveProfileSketch(createdWrapper);
    if (!sketch || sketch->entities().empty()) {
        m_host.showStatus(tr("Draw a closed profile first"));
        return;
    }

    FeatureForm form(m_host.dialogParent(), tr("Revolve"), part().lengthUnit());
    auto* size = form.angle(QStringLiteral("size"), tr("Angle:"), 360.0, 1.0, 360.0, 1);
    // About one of the sketch's own axes, through its origin: on the XY
    // plane, the world's Y or X.
    auto* axis =
        form.choice(QStringLiteral("axis"), tr("Axis:"),
                    {tr("The sketch's vertical axis (Y)"), tr("The sketch's horizontal axis (X)")});
    auto* result = form.operationChoice(proposedOperation());
    if (!form.exec()) return;
    const double angle = size->value() * std::numbers::pi / 180.0;
    const doc::BodyOperation operation = FeatureForm::operation(result);

    // As the sketch was drawn: one placed on a face takes it along (Phase 157).
    const draft::SketchPlane& plane = sketch->drawnPlane();
    const math::Vec3 axisPoint = plane.origin();
    const math::Vec3 axisDir = axis->currentIndex() == 1 ? plane.xAxis() : plane.yAxis();

    std::string why;
    auto probe = model::Revolve::execute(sketch->entities(), plane, axisPoint, axisDir, angle,
                                         "probe", model::Revolve::kDefaultSegments, 0.0, &why);
    if (!probe) {
        m_host.showStatus(tr("Revolve failed: %1").arg(QString::fromStdString(why)));
        return;
    }

    auto feature = std::make_unique<doc::RevolveFeature>(sketch, axisPoint, axisDir, angle);
    feature->setOperation(operation);
    if (!m_host.addFeature(std::move(feature), tr("Revolve"), createdWrapper ? sketch : nullptr)) {
        return;
    }

    m_host.viewport().camera().setIsometricView();
    m_host.viewport().update();
}

// ---------------------------------------------------------------------------
// Booleans
// ---------------------------------------------------------------------------

void PartCommands::onBooleanUnion() {
    combineBodies(model::BooleanType::Union, tr("Union"));
}

void PartCommands::onBooleanSubtract() {
    combineBodies(model::BooleanType::Subtract, tr("Subtract"));
}

void PartCommands::onBooleanIntersect() {
    combineBodies(model::BooleanType::Intersect, tr("Intersect"));
}

void PartCommands::combineBodies(model::BooleanType type, const QString& verb) {
    if (!requirePart(verb)) return;
    // Bodies, not shells: a cavity is a shell of the body around it.
    const topo::Solid* solid = part().solid();
    if (!solid || model::Pattern::separate(*solid).size() < 2) {
        m_host.showStatus(
            tr("%1 combines the part's bodies, and it has fewer than two (make one with "
               "Result: New body)")
                .arg(verb));
        return;
    }
    m_host.addFeature(std::make_unique<doc::BooleanFeature>(type), verb);
}

// ---------------------------------------------------------------------------
// Fillet, Chamfer, Shell, Draft
// ---------------------------------------------------------------------------

const topo::Solid* PartCommands::requireBody(const QString& verb) {
    if (!requirePart(verb)) return nullptr;
    const topo::Solid* solid = part().solid();
    if (!solid) {
        m_host.showStatus(tr("%1 works on a body: make one first").arg(verb));
    }
    return solid;
}

void PartCommands::onFillet() {
    addEdgeFeature(true);
}

void PartCommands::onChamfer() {
    addEdgeFeature(false);
}

void PartCommands::addEdgeFeature(bool fillet) {
    const QString verb = fillet ? tr("Fillet") : tr("Chamfer");
    const topo::Solid* solid = requireBody(verb);
    if (!solid) return;
    const PickList edges = edgesOf(*solid);

    FeatureForm form(m_host.dialogParent(), verb, part().lengthUnit());
    auto* size = form.length(QStringLiteral("size"), fillet ? tr("Radius:") : tr("Distance:"), 1.0,
                             0.001, 1e6);
    auto* list = form.checklist(QStringLiteral("edges"), tr("Edges:"), edges.items);
    checkClicked(list, edges, m_host.viewport().modelSelection(), true);
    if (!form.exec()) return;

    std::vector<topo::TopologyID> chosen;
    for (const int row : FeatureForm::checkedRows(list)) {
        chosen.push_back(edges.ids[static_cast<size_t>(row)]);
    }
    if (chosen.empty()) {
        m_host.showStatus(tr("%1 not added: no edges were chosen").arg(verb));
        return;
    }
    std::unique_ptr<doc::Feature> feature;
    if (fillet) {
        feature = std::make_unique<doc::FilletFeature>(std::move(chosen), size->value());
    } else {
        feature = std::make_unique<doc::ChamferFeature>(std::move(chosen), size->value());
    }
    m_host.addFeature(std::move(feature), verb);
}

void PartCommands::onShell() {
    const QString verb = tr("Shell");
    const topo::Solid* solid = requireBody(verb);
    if (!solid) return;
    const PickList faces = facesOf(*solid);

    FeatureForm form(m_host.dialogParent(), verb, part().lengthUnit());
    auto* thickness =
        form.length(QStringLiteral("thickness"), tr("Wall thickness:"), 1.0, 0.001, 1e6);
    auto* list = form.checklist(QStringLiteral("faces"), tr("Faces to open:"), faces.items);
    checkClicked(list, faces, m_host.viewport().modelSelection(), false);
    if (!form.exec()) return;

    std::vector<topo::TopologyID> open;
    for (const int row : FeatureForm::checkedRows(list)) {
        open.push_back(faces.ids[static_cast<size_t>(row)]);
    }
    m_host.addFeature(std::make_unique<doc::ShellFeature>(thickness->value(), std::move(open)),
                      verb);
}

void PartCommands::onDraft() {
    const QString verb = tr("Draft");
    if (!requireBody(verb)) return;

    FeatureForm form(m_host.dialogParent(), verb, part().lengthUnit());
    auto* pull =
        directionChoice(form, QStringLiteral("pull"), tr("Pull direction:"), QStringLiteral("+Z"));
    auto* neutral = form.length(QStringLiteral("neutral"), tr("Neutral plane at:"), 0.0, -1e6, 1e6);
    auto* angle = form.angle(QStringLiteral("angle"), tr("Angle:"), 3.0, 0.01, 89.0, 2);
    if (!form.exec()) return;

    // The neutral plane is square to the pull, at that distance along it.
    const math::Vec3 direction = chosenDirection(pull);
    m_host.addFeature(
        std::make_unique<doc::DraftFeature>(direction, direction * neutral->value(),
                                            angle->value() * std::numbers::pi / 180.0),
        verb);
}

// ---------------------------------------------------------------------------
// Patterns, Mirror and Hole
// ---------------------------------------------------------------------------

namespace {

/// The features a pattern can repeat instead of the whole part: those that
/// add or cut material, active in the build.
std::vector<const doc::Feature*> repeatableFeatures(const doc::FeatureTree& tree) {
    std::vector<const doc::Feature*> out;
    const int last = tree.rollbackIndex() >= 0 ? tree.rollbackIndex()
                                               : static_cast<int>(tree.featureCount()) - 1;
    for (int i = 0; i <= last; ++i) {
        const doc::Feature* feature = tree.feature(static_cast<size_t>(i));
        if (feature && feature->createsNewBody() && !feature->isSuppressed())
            out.push_back(feature);
    }
    return out;
}

/// A checklist of @p features for a pattern to repeat (or a mirror to
/// mirror, said by @p label); the ids of those checked are read with
/// checkedTargets().
QListWidget* targetList(FeatureForm& form, const std::vector<const doc::Feature*>& features,
                        const QString& label = PartCommands::tr("Repeat only (none: the whole "
                                                                "part):")) {
    std::vector<std::pair<QString, QString>> items;
    items.reserve(features.size());
    for (const doc::Feature* feature : features) {
        items.emplace_back(QString::fromStdString(feature->name()),
                           QString::fromStdString(feature->featureID()));
    }
    return form.checklist(QStringLiteral("features"), label, items);
}

std::vector<std::string> checkedTargets(const QListWidget* list,
                                        const std::vector<const doc::Feature*>& features) {
    std::vector<std::string> ids;
    for (const int row : FeatureForm::checkedRows(list)) {
        ids.push_back(features[static_cast<size_t>(row)]->featureID());
    }
    return ids;
}

}  // namespace

void PartCommands::onLinearPattern() {
    const QString verb = tr("Linear Pattern");
    if (!requireBody(verb)) return;

    FeatureForm form(m_host.dialogParent(), verb, part().lengthUnit());
    auto* direction =
        directionChoice(form, QStringLiteral("direction"), tr("Direction:"), QStringLiteral("+X"));
    auto* spacing = form.length(QStringLiteral("spacing"), tr("Spacing:"), 20.0, 0.001, 1e6);
    auto* count =
        form.count(QStringLiteral("count"), tr("Instances:"), 3, 2, doc::kMaxPatternCount);
    const auto repeatable = repeatableFeatures(part().featureTree());
    auto* targets = targetList(form, repeatable);
    if (!form.exec()) return;

    auto pattern = doc::PatternFeature::makeLinear(chosenDirection(direction), spacing->value(),
                                                   count->value());
    pattern->setTargets(checkedTargets(targets, repeatable));
    m_host.addFeature(std::move(pattern), verb);
}

void PartCommands::onCircularPattern() {
    const QString verb = tr("Circular Pattern");
    if (!requireBody(verb)) return;

    FeatureForm form(m_host.dialogParent(), verb, part().lengthUnit());
    auto* axis =
        directionChoice(form, QStringLiteral("axis"), tr("About the axis:"), QStringLiteral("+Z"));
    auto* count =
        form.count(QStringLiteral("count"), tr("Instances:"), 4, 2, doc::kMaxPatternCount);
    auto* total = form.angle(QStringLiteral("angle"), tr("Over:"), 360.0, 1.0, 360.0, 2);
    const auto repeatable = repeatableFeatures(part().featureTree());
    auto* targets = targetList(form, repeatable);
    if (!form.exec()) return;

    // A full turn spaces the instances evenly around it; a partial one puts
    // the first and last at its ends.
    const int n = count->value();
    const double degrees = total->value();
    const double step = degrees >= 360.0 ? 360.0 / n : degrees / (n - 1);
    auto pattern = doc::PatternFeature::makeCircular(math::Vec3::Zero, chosenDirection(axis),
                                                     step * std::numbers::pi / 180.0, n);
    pattern->setTargets(checkedTargets(targets, repeatable));
    m_host.addFeature(std::move(pattern), verb);
}

void PartCommands::onMirror() {
    const QString verb = tr("Mirror");
    if (!requireBody(verb)) return;

    // In a base plane through the origin, or a flat face of the part (the
    // first clicked, at first), followed by its name.
    struct Plane {
        QString text;
        math::Vec3 point;
        math::Vec3 normal;
        std::string face;  ///< its whole name; empty for a base plane
    };
    std::vector<Plane> planes = {
        {tr("YZ plane (x = 0)"), math::Vec3::Zero, math::Vec3::UnitX, {}},
        {tr("ZX plane (y = 0)"), math::Vec3::Zero, math::Vec3::UnitY, {}},
        {tr("XY plane (z = 0)"), math::Vec3::Zero, math::Vec3::UnitZ, {}},
    };
    const auto faces = flatFacesOf(*part().solid());
    const int bases = static_cast<int>(planes.size());
    for (const auto& face : faces) {
        planes.push_back({tr("the face %1").arg(face.text), face.middle, face.normal, face.name});
    }
    const int clicked = clickedFlatFace(faces, m_host.viewport().modelSelection(), -1);
    QStringList names;
    for (const auto& plane : planes) names << plane.text;

    FeatureForm form(m_host.dialogParent(), verb, part().lengthUnit());
    auto* which = form.choice(QStringLiteral("plane"), tr("In:"), names);
    which->setCurrentIndex(clicked < 0 ? 0 : bases + clicked);
    const auto mirrorable = repeatableFeatures(part().featureTree());
    auto* targets = targetList(form, mirrorable, tr("Mirror only (none: the whole part):"));
    if (!form.exec()) return;

    const Plane& plane = planes.at(static_cast<size_t>(std::max(which->currentIndex(), 0)));
    auto mirror = doc::MirrorFeature::make(plane.point, plane.normal);
    if (!plane.face.empty()) mirror->setReference("planeFace", plane.face);
    mirror->setTargets(checkedTargets(targets, mirrorable));
    m_host.addFeature(std::move(mirror), verb);
}

void PartCommands::onHole() {
    const QString verb = tr("Hole");
    if (!requireBody(verb)) return;
    const auto faces = flatFacesOf(*part().solid());
    if (faces.empty()) {
        m_host.showStatus(tr("%1: the part has no flat face to drill into").arg(verb));
        return;
    }
    // Into the face clicked (else the first facing up), at its middle.
    int facingUp = 0;
    for (size_t i = 0; i < faces.size(); ++i) {
        if (faces[i].normal.z > 0.99) {
            facingUp = static_cast<int>(i);
            break;
        }
    }
    const int start = clickedFlatFace(faces, m_host.viewport().modelSelection(), facingUp);
    QStringList names;
    for (const auto& face : faces) names << face.text;

    FeatureForm form(m_host.dialogParent(), verb, part().lengthUnit());
    auto* face = form.choice(QStringLiteral("face"), tr("Into the face:"), names);
    face->setCurrentIndex(start);
    const math::Vec3& middle = faces.at(static_cast<size_t>(start)).middle;
    auto* x = form.length(QStringLiteral("x"), tr("At X:"), middle.x, -1e6, 1e6);
    auto* y = form.length(QStringLiteral("y"), tr("At Y:"), middle.y, -1e6, 1e6);
    auto* z = form.length(QStringLiteral("z"), tr("At Z:"), middle.z, -1e6, 1e6);
    // Another face chosen: at its middle, to be moved from there.
    connect(face, &QComboBox::currentIndexChanged, &form.dialog(), [&faces, x, y, z](int row) {
        if (row < 0 || row >= static_cast<int>(faces.size())) return;
        const math::Vec3& at = faces[static_cast<size_t>(row)].middle;
        x->setValue(at.x);
        y->setValue(at.y);
        z->setValue(at.z);
    });
    auto* type = form.choice(QStringLiteral("type"), tr("Type:"),
                             {tr("Simple"), tr("Counterbore"), tr("Countersink")});
    auto* extent = form.choice(QStringLiteral("extent"), tr("Goes:"),
                               {tr("To the depth"), tr("Through all"), tr("Up to a face")});
    auto* upTo = form.choice(QStringLiteral("upToFace"), tr("Up to the face:"), names);
    auto* diameter = form.length(QStringLiteral("diameter"), tr("Diameter:"), 5.0, 0.001, 1e6);
    auto* depth = form.length(QStringLiteral("depth"), tr("Depth:"), 10.0, 0.001, 1e6);
    auto* point =
        form.angle(QStringLiteral("pointAngle"), tr("Point (0: flat):"), 118.0, 0.0, 179.0);
    auto* boreDiameter =
        form.length(QStringLiteral("boreDiameter"), tr("Counterbore diameter:"), 9.0, 0.001, 1e6);
    auto* boreDepth =
        form.length(QStringLiteral("boreDepth"), tr("Counterbore depth:"), 3.0, 0.001, 1e6);
    auto* sinkDiameter =
        form.length(QStringLiteral("sinkDiameter"), tr("Countersink diameter:"), 10.0, 0.001, 1e6);
    auto* sinkAngle =
        form.angle(QStringLiteral("sinkAngle"), tr("Countersink angle:"), 90.0, 1.0, 179.0);
    const auto offer = [type, extent, upTo, depth, point, boreDiameter, boreDepth, sinkDiameter,
                        sinkAngle] {
        const bool blind = extent->currentIndex() == 0;
        upTo->setEnabled(extent->currentIndex() == 2);
        depth->setEnabled(blind);
        point->setEnabled(blind);
        boreDiameter->setEnabled(type->currentIndex() == 1);
        boreDepth->setEnabled(type->currentIndex() == 1);
        sinkDiameter->setEnabled(type->currentIndex() == 2);
        sinkAngle->setEnabled(type->currentIndex() == 2);
    };
    connect(type, &QComboBox::currentIndexChanged, &form.dialog(), offer);
    connect(extent, &QComboBox::currentIndexChanged, &form.dialog(), offer);
    offer();
    if (!form.exec()) return;

    const FlatFace& into = faces.at(static_cast<size_t>(std::max(face->currentIndex(), 0)));
    auto hole = doc::HoleFeature::make(into.name, math::Vec3(x->value(), y->value(), z->value()),
                                       diameter->value(), depth->value());
    const double toRadians = std::numbers::pi / 180.0;
    const std::map<std::string, double> sizes = {
        {"type", type->currentIndex()},
        {"extent", extent->currentIndex()},
        {"pointAngle", point->value() * toRadians},
        {"boreDiameter", boreDiameter->value()},
        {"boreDepth", boreDepth->value()},
        {"sinkDiameter", sinkDiameter->value()},
        {"sinkAngle", sinkAngle->value() * toRadians},
    };
    for (const auto& [name, value] : sizes) {
        if (!hole->setParameter(name, value)) {
            m_host.showStatus(
                // The label as it is: lowercased, a German noun was wrong.
                tr("%1: \"%2\" cannot take that value").arg(verb, parameterLabel(name)));
            return;
        }
    }
    if (extent->currentIndex() == 2) {
        hole->setReference("upToFace",
                           faces.at(static_cast<size_t>(std::max(upTo->currentIndex(), 0))).name);
    }
    m_host.addFeature(std::move(hole), verb);
}

}  // namespace hz::ui
