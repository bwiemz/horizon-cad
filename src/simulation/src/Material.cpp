#include "horizon/simulation/Material.h"

namespace hz::sim::materials {

ElasticMaterial steel() {
    return ElasticMaterial{200.0e9, 0.30, 7850.0};
}

ElasticMaterial aluminum() {
    return ElasticMaterial{69.0e9, 0.33, 2700.0};
}

ElasticMaterial titanium() {
    return ElasticMaterial{114.0e9, 0.34, 4430.0};
}

}  // namespace hz::sim::materials
