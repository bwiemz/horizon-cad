#pragma once

#include <functional>
#include <memory>
#include <string>

#include "horizon/modeling/FacetResolution.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// What a converted feature builds (stage 3): its body built exactly, and that body's bridge copy,
/// the faceted body every operation not yet converted reads.
struct ExactAndBridge {
    std::unique_ptr<topo::Solid> exact;
    std::unique_ptr<topo::Solid> faceted;
};

/// Makes an exact solid, or gives why it cannot in @p why (and gives null); it may throw instead.
using ExactBuilder = std::function<std::unique_ptr<topo::Solid>(std::string& why)>;

/// Describes the faceted construction master built the same body by (MastersFacets), for the
/// bridge's facets, chords and seams to be numbered as master's; asked only once the bridge is cut.
using MastersConstruction = std::function<MastersFacets()>;

/// Builds master's own faceted construction of the same body (PrimitiveFactory's primitive, its
/// prism, its revolve), as its Stable path built and named it, or gives why it cannot in @p why
/// (and gives null) (Ruling 44).
using MastersSolid = std::function<std::unique_ptr<topo::Solid>(std::string& why)>;

/// The exact solid @p build makes, or null where it makes none: then @p reason is "could not be
/// built exactly: <why>", why being what @p build gave, or what it threw.
std::unique_ptr<topo::Solid> buildExactly(const ExactBuilder& build, std::string* reason);

/// A converted feature's construction, the one sequence every one goes through (stage 3): the
/// exact solid (buildExactly), its bridge copy (model::facetExact at @p resolution: "its exact
/// form could not be cut into facets: <why>" where it cannot be, or facetExact throws), the
/// bridge's edges named by model::nameBridgeEdges (Ruling 18; a polyhedron's copy has none to
/// name), and, where @p masters is given and describes master's construction, the bridge's facets,
/// chords and seams numbered as master's numbered the same places (model::nameAsMasters, Ruling
/// 21). Both null, with @p reason, where either cannot be made.
///
/// Where @p own is given (a feature whose master construction is one construction: a primitive,
/// a single loop's extrude or revolve, a hole's tool, a sheet's flange), the faceted body is
/// master's own construction itself, named as the bridge copy names the same places
/// (nameAfterBridge, Ruling 44): its faces, loops and edges in master's order and master's
/// winding, so an operation after it numbers what it makes as on master; the bridge copy cut
/// from the exact solid is made only to name it (and the exact solid: nameAsMasters). "its
/// faceted copy could not be built as master built it: <why>" where @p own makes none.
ExactAndBridge buildWithBridge(const ExactBuilder& build, const FacetResolution& resolution,
                               const MastersConstruction& masters, std::string* reason,
                               const MastersSolid& own = {});

}  // namespace hz::model
