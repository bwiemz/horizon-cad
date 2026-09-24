#pragma once

#include <string>
#include <string_view>

namespace hz::pdm {

/// The SHA-256 digest (FIPS 180-4) of @p data, as 64 lowercase hex digits.
///
/// The vault's content hash: a revision's hash identifies its content across
/// machines, so it has to resist collisions, which the FNV-1a hash archives
/// used before Phase 119 does not.
std::string sha256Hex(std::string_view data);

}  // namespace hz::pdm
