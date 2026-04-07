#pragma once

#include "horizon/math/Vec3.h"
#include "horizon/topology/TopologyID.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace hz::geo {
class NurbsCurve;
class NurbsSurface;
}  // namespace hz::geo

namespace hz::topo {

struct HalfEdge;
struct Edge;
struct Face;
struct Wire;
struct Shell;
struct Solid;

/// A vertex in the B-Rep — stores a 3D position and one outgoing half-edge.
struct Vertex {
    uint32_t id = 0;
    TopologyID topoId;
    math::Vec3 point;
    HalfEdge* halfEdge = nullptr;  ///< One outgoing half-edge.
};

/// A directed edge bounding one face.  The twin half-edge bounds the adjacent face.
struct HalfEdge {
    uint32_t id = 0;
    Vertex* origin = nullptr;
    HalfEdge* twin = nullptr;
    HalfEdge* next = nullptr;
    HalfEdge* prev = nullptr;
    Edge* edge = nullptr;
    Face* face = nullptr;
};

/// A topological edge shared by two half-edges (one in each direction).
struct Edge {
    uint32_t id = 0;
    TopologyID topoId;
    HalfEdge* halfEdge = nullptr;  ///< One of the two half-edges.
    std::shared_ptr<geo::NurbsCurve> curve;
};

/// A closed loop of half-edges forming a face boundary.
struct Wire {
    uint32_t id = 0;
    HalfEdge* halfEdge = nullptr;  ///< Any half-edge in the loop.
};

/// A planar or curved region bounded by one outer wire and zero or more inner wires (holes).
struct Face {
    uint32_t id = 0;
    TopologyID topoId;
    Wire* outerLoop = nullptr;
    std::vector<Wire*> innerLoops;
    Shell* shell = nullptr;
    std::shared_ptr<geo::NurbsSurface> surface;
};

/// A connected set of faces forming a closed (or open) skin.
struct Shell {
    uint32_t id = 0;
    std::vector<Face*> faces;
    class Solid* solid = nullptr;  ///< Owning solid (use class to avoid ambiguity with struct forward decl).
};

}  // namespace hz::topo
