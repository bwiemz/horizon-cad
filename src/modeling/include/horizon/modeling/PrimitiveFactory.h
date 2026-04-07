#pragma once

#include "horizon/topology/Solid.h"

#include <memory>

namespace hz::model {

/// Factory for creating primitive solid bodies with correct B-Rep topology
/// and NURBS surface/curve geometry bindings.
///
/// Every primitive satisfies Euler's formula (V - E + F = 2) and has
/// TopologyIDs assigned to all faces and edges for stable naming.
class PrimitiveFactory {
public:
    /// Axis-aligned box from (0,0,0) to (width, height, depth).
    /// 8V, 12E, 6F.  Each face is a bilinear planar NURBS surface.
    static std::unique_ptr<topo::Solid> makeBox(double width, double height, double depth);

    /// Cylinder centered at origin, axis along +Z, given radius and height.
    /// Uses box-like topology (8V, 12E, 6F) with cylindrical NURBS lateral patches
    /// and planar caps.
    static std::unique_ptr<topo::Solid> makeCylinder(double radius, double height);

    /// Sphere centered at origin with given radius.
    /// Uses box-like topology (8V, 12E, 6F) with spherical NURBS patches.
    static std::unique_ptr<topo::Solid> makeSphere(double radius);

    /// Cone (frustum) centered at origin, axis along +Z.
    /// @param bottomRadius  Radius at z=0.
    /// @param topRadius     Radius at z=height (0 for a sharp cone).
    /// @param height        Height along Z.
    /// Uses box-like topology (8V, 12E, 6F) with conical NURBS patches.
    static std::unique_ptr<topo::Solid> makeCone(double bottomRadius, double topRadius,
                                                  double height);

    /// Torus centered at origin, axis along +Z.
    /// @param majorRadius  Distance from center to tube center.
    /// @param minorRadius  Tube radius.
    /// Uses box-like topology (8V, 12E, 6F) with toroidal NURBS patches.
    static std::unique_ptr<topo::Solid> makeTorus(double majorRadius, double minorRadius);
};

}  // namespace hz::model
