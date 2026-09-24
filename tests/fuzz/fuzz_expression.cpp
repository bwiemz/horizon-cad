// Fuzz target: design-variable expressions, which are read from files. Any
// input may be rejected; none may crash, hang or throw — including when the
// parsed tree is evaluated, printed and re-parsed.

#include <cstddef>
#include <cstdint>
#include <map>
#include <nlohmann/json.hpp>
#include <string>

#include "horizon/math/Expression.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    const std::string text(reinterpret_cast<const char*>(data), size);
    auto expr = hz::math::Expression::parse(text);
    if (!expr) return 0;

    std::map<std::string, double> variables;
    for (const std::string& name : expr->variables()) variables[name] = 1.5;
    (void)expr->evaluate(variables);
    (void)hz::math::Expression::parse(expr->toString());
    (void)hz::math::Expression::fromJson(expr->toJson());
    return 0;
}
