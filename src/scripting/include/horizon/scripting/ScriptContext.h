#pragma once

#include <string>
#include <vector>

#include "horizon/math/Vec3.h"
#include "horizon/modeling/MassProperties.h"

namespace hz::doc {
class Document;
}

namespace hz::script {

/// Stable, script-facing facade over a Document.
///
/// Scripts operate on the CAD model through this narrow surface rather than the
/// full internal class hierarchy, so the binding layer stays small and the API
/// is decoupled from internal churn. Mutations build the feature tree; queries
/// read the last rebuilt solid.
class ScriptContext {
public:
    explicit ScriptContext(doc::Document& document);

    // --- Queries -----------------------------------------------------------
    int featureCount() const;
    int sketchCount() const;
    bool hasSolid() const;
    int solidFaceCount() const;   ///< -1 if there is no solid.
    int solidShellCount() const;  ///< -1 if there is no solid.
    std::string lastError() const;

    // --- Sketch authoring --------------------------------------------------
    /// Add a closed rectangle profile (corner at origin, size w×h) on the XY
    /// plane. Returns the new sketch's index.
    int addRectangleSketch(double w, double h);

    // --- Primitives (parametric base features) -----------------------------
    void addBox(double width, double height, double depth);
    void addCylinder(double radius, double height);
    void addSphere(double radius);
    void addCone(double bottomRadius, double topRadius, double height);
    void addTorus(double majorRadius, double minorRadius);

    // --- Features ----------------------------------------------------------
    /// Extrude the sketch at @p sketchIndex along @p direction by @p distance.
    /// Returns false if the index is out of range.
    bool addExtrude(int sketchIndex, const math::Vec3& direction, double distance);

    /// Linear-pattern the current body: @p count copies @p spacing apart along
    /// @p direction.
    bool addLinearPattern(const math::Vec3& direction, double spacing, int count);

    // --- Reference geometry ------------------------------------------------
    void addDatumPlane(const math::Vec3& origin, const math::Vec3& normal, const math::Vec3& xAxis);
    void addDatumAxis(const math::Vec3& origin, const math::Vec3& direction);
    void addDatumPoint(const math::Vec3& position);

    // --- Analysis ----------------------------------------------------------
    /// Mass properties of the current solid (volume/area/centroid/inertia/mass)
    /// at the given density. `valid` is false when there is no solid.
    model::MassProperties massProperties(double density = 1.0) const;

    /// Summary result of a linear-static finite-element analysis.
    struct StaticAnalysisResult {
        bool converged = false;        ///< false if there is no solid or the solve fails
        double maxDisplacement = 0.0;  ///< peak nodal displacement magnitude
        double maxVonMises = 0.0;      ///< peak element von Mises stress
    };

    /// Run a simple axial linear-static analysis on the current solid: mesh its
    /// bounding box (@p resolution subdivisions per side), fix the minimum face
    /// along @p axis (0=x, 1=y, 2=z) and apply a total @p force along +axis on
    /// the maximum face, for a material with @p youngsModulus and @p poissonRatio.
    StaticAnalysisResult staticAnalysis(double force, double youngsModulus, double poissonRatio,
                                        int axis, int resolution) const;

    /// Result of a modal (free-vibration) analysis.
    struct ModalAnalysisResult {
        bool converged = false;                  ///< false if there is no solid or the solve fails
        std::vector<double> naturalFrequencies;  ///< hertz, ascending
    };

    /// Run a modal analysis on the current solid: mesh its bounding box
    /// (@p resolution subdivisions per side), fix the minimum face along @p axis
    /// (0=x, 1=y, 2=z), and solve for the lowest @p numModes natural frequencies
    /// of a material with @p youngsModulus, @p poissonRatio and @p density.
    ModalAnalysisResult modalAnalysis(double youngsModulus, double poissonRatio, double density,
                                      int axis, int numModes, int resolution) const;

    // --- Drawings ----------------------------------------------------------
    /// Generate the standard four-view drawing (front/top/right/isometric,
    /// hidden lines removed) of the current solid and write it to a DXF file.
    /// Returns false when there is no solid or on I/O failure.
    bool exportDrawingDxf(const std::string& path) const;

    // --- Rebuild -----------------------------------------------------------
    /// Replay the feature tree. Returns true on success.
    bool rebuild();

    doc::Document& document() { return m_document; }

private:
    doc::Document& m_document;
};

}  // namespace hz::script
