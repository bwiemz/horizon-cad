#pragma once

#include <string>

namespace hz::cam {

struct Toolpath;

/// Serializes a toolpath to RS-274 (G-code): Rapid moves as `G0`, Feed moves as
/// `G1` with an `F` word. Coordinates are written in millimetres with a small
/// program preamble (`G21` metric, `G90` absolute) and an `M2` end.
class GcodeWriter {
public:
    /// G-code text for @p path. @p decimals controls coordinate precision.
    static std::string toGcode(const Toolpath& path, int decimals = 3);
};

}  // namespace hz::cam
