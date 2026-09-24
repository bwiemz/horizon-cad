#pragma once

#include <optional>
#include <string>

namespace hz::cam {

struct Toolpath;

/// What a program needs besides its moves.
struct GcodeOptions {
    int toolNumber = 1;       ///< loaded with `T<n> M6`, its length offset with `G43 H<n>`
    double spindleRpm = 0.0;  ///< `S<rpm> M3`; required, so a program never cuts with it stopped
    bool coolant = false;     ///< `M8` once the spindle runs, `M9` at the end
    int decimals = 3;         ///< digits after the point in coordinates and feeds, 0 to 6
};

/// Serializes a toolpath to RS-274 (G-code) for a mill, in the dialect of
/// Fanuc-style controls and LinuxCNC.
///
/// The program starts from a known state whatever the machine was left in:
/// - `G21 G90 G94 G17 G40 G49 G80`: millimetres, absolute coordinates, feed
///   per minute, the XY plane, and cutter compensation, tool length offset
///   and canned cycles cancelled;
/// - the tool is loaded and its length offset applied, and the spindle (and
///   coolant, if asked) started before the first move;
/// - it ends by stopping them (`M9`, `M5`) and `M30`.
///
/// Rapid moves never cut through the part on the way. A rapid that climbs
/// moves Z first, then X and Y; one that descends moves X and Y first, then Z.
/// The first move, from wherever the tool is, climbs to its height before any
/// X or Y motion.
class GcodeWriter {
public:
    /// Why @p path cannot be written with @p options, or "" if it can:
    /// - the path is empty, or does not start with a rapid;
    /// - a coordinate or feed is not a finite number, or a feed is not
    ///   positive;
    /// - a rapid is not above every cutting move, so it could move through
    ///   material;
    /// - the tool number is not 1 to 9999, the spindle speed is not positive,
    ///   or decimals is not 0 to 6.
    static std::string validate(const Toolpath& path, const GcodeOptions& options);

    /// G-code text for @p path, or std::nullopt when validate() finds a
    /// problem, with the reason in @p error when given.
    static std::optional<std::string> toGcode(const Toolpath& path, const GcodeOptions& options,
                                              std::string* error = nullptr);
};

}  // namespace hz::cam
