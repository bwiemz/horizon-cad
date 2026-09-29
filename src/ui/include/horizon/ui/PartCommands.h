#pragma once

#include <QObject>
#include <QString>
#include <functional>
#include <memory>
#include <vector>

#include "horizon/document/FeatureTree.h"
#include "horizon/modeling/BooleanOp.h"

namespace hz::doc {
class Document;
class PrimitiveFeature;
class Sketch;
}  // namespace hz::doc

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::ui {

class WorkbenchHost;

/// The commands that add a feature to the part: primitives, Extrude and
/// Revolve, Loft and Sweep, datums, Booleans, Fillet and Chamfer, Shell,
/// Draft, patterns, Mirror and Hole. Each asks for its sizes in a form and
/// adds its feature through its WorkbenchHost, which builds the part and
/// withdraws a feature that fails itself. The window holds the menus and the
/// ribbon, the sketches being drawn, and the feature history's panel.
class PartCommands : public QObject {
    Q_OBJECT

public:
    explicit PartCommands(WorkbenchHost& host, QObject* parent = nullptr);

    // --- Primitives ---
    void onPrimitiveBox();
    void onPrimitiveCylinder();
    void onPrimitiveSphere();
    void onPrimitiveCone();
    void onPrimitiveTorus();

    // --- From sketches ---
    void onExtrudeSketch();
    void onRevolveSketch();
    // Loft, Sweep and datums (Phase 133).
    void onLoft();
    void onSweep();
    void onDatumPlane();
    void onDatumAxis();
    void onDatumPoint();

    // --- On the part's bodies ---
    void onBooleanUnion();
    void onBooleanSubtract();
    void onBooleanIntersect();
    void onFillet();
    void onChamfer();
    void onShell();
    void onDraft();
    void onLinearPattern();
    void onCircularPattern();
    /// A part mirrored in a plane, or some of its features (Phase 162).
    void onMirror();
    /// A hole drilled into a flat face of the part (Phase 162).
    void onHole();

    /// The part's solid, or null — with a word in the status bar — when there
    /// is no part or it has no body yet.
    const topo::Solid* requireBody(const QString& verb);

    /// A size a primitive's form asks for: its label, first value and least.
    struct PrimitiveField {
        QString label;
        double value;
        double min;
    };

private:
    /// The active tab's document: a part, unless requirePart() says not.
    doc::Document& part();
    /// False, with a word in the status bar, when the active tab is not a
    /// part (`verb` names the command).
    bool requirePart(const QString& verb);
    /// Join once the part has a body; a new body for the first.
    doc::BodyOperation proposedOperation();
    /// The sketch a profile is taken from: the one being edited, finished;
    /// else the one chosen in the list, or last made or finished; else what
    /// the drawing shows, in a new sketch (@p createdWrapper) that goes into
    /// the document with the feature, or a sketch that already holds it.
    std::shared_ptr<doc::Sketch> resolveProfileSketch(bool& createdWrapper);
    /// Ask for a primitive's sizes (`fields`) and body operation, then add it.
    void addPrimitive(
        const QString& verb, const std::vector<PrimitiveField>& fields,
        const std::function<std::unique_ptr<doc::PrimitiveFeature>(const std::vector<double>&)>&
            make);
    /// Add a feature combining the part's bodies.
    void combineBodies(model::BooleanType type, const QString& verb);
    /// Ask for edges and a size, then add a fillet (or chamfer) on them.
    void addEdgeFeature(bool fillet);

    WorkbenchHost& m_host;
};

}  // namespace hz::ui
