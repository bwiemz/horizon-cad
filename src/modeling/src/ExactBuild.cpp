#include "horizon/modeling/ExactBuild.h"

#include <exception>
#include <string>
#include <utility>

#include "horizon/modeling/Faceting.h"

namespace hz::model {

std::unique_ptr<topo::Solid> buildExactly(const ExactBuilder& build, std::string* reason) {
    std::string why;
    std::unique_ptr<topo::Solid> solid;
    try {
        solid = build(why);
    } catch (const std::exception& e) {
        solid.reset();
        why = e.what();
    }
    if (!solid && reason) *reason = "could not be built exactly: " + why;
    return solid;
}

ExactAndBridge buildWithBridge(const ExactBuilder& build, const FacetResolution& resolution,
                               const MastersConstruction& masters, std::string* reason,
                               const MastersSolid& own) {
    ExactAndBridge out;
    auto exact = buildExactly(build, reason);
    if (!exact) return out;
    FacetedSolid bridge;
    try {
        bridge = facetExact(*exact, resolution);
    } catch (const std::exception& e) {
        bridge.solid.reset();
        bridge.error = e.what();
    }
    if (!bridge.solid) {
        if (reason) *reason = "its exact form could not be cut into facets: " + bridge.error;
        return out;
    }
    if (!bridge.copied) nameBridgeEdges(*bridge.solid, *exact);
    if (masters) nameAsMasters(*bridge.solid, *exact, masters());
    if (own) {
        // Master's own construction, named as the bridge names the same places (Ruling 44).
        std::string why;
        std::unique_ptr<topo::Solid> made;
        try {
            made = own(why);
        } catch (const std::exception& e) {
            made.reset();
            why = e.what();
        }
        if (!made) {
            if (reason) *reason = "its faceted copy could not be built as master built it: " + why;
            return out;
        }
        nameAfterBridge(*made, *bridge.solid);
        bridge.solid = std::move(made);
    }
    out.exact = std::move(exact);
    out.faceted = std::move(bridge.solid);
    return out;
}

}  // namespace hz::model
