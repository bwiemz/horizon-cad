#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "horizon/math/Units.h"

namespace hz::ui {

/// A length typed at a tool while it runs (a fillet radius, a chamfer
/// distance): digits and a point, a unit after them if it is not the
/// document's ("2 in", Phase 154), Backspace, then Enter to take it. The
/// whole text must be one positive length. Anything else, such as "." or
/// "1.2.3", is refused, and the prompt says so; it is not read as whatever
/// beginning of it parses.
class TypedLength {
public:
    explicit TypedLength(double value) : m_value(value) {}

    /// Handle a key (a Qt::Key). True when it was one of the input's own:
    /// a digit, the point, what a unit is written with once something is
    /// typed, Backspace with something to take back, Enter. A bare number
    /// is in @p unit.
    bool key(int key, math::LengthUnit unit = math::LengthUnit::Millimetre);

    double value() const { return m_value; }

    /// Drop what is being typed and any refusal; the value stays.
    void clear();

    /// For the tool's prompt, with a leading gap: what is being typed
    /// ("  Radius (mm): 2.5"), else a refusal ("  '1.2.3' is not a radius"),
    /// else the value in @p unit ("  [radius=2.5 mm]"). @p name is lower case.
    std::string prompt(const std::string& name,
                       math::LengthUnit unit = math::LengthUnit::Millimetre) const;

    /// The whole of @p text as one positive, finite length, in millimetres,
    /// a bare number in @p unit; or nothing.
    static std::optional<double> parse(std::string_view text,
                                       math::LengthUnit unit = math::LengthUnit::Millimetre);

private:
    double m_value;
    std::string m_text;
    std::string m_refused;
};

}  // namespace hz::ui
