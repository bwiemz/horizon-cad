#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "horizon/math/Units.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// A point typed while a drawing tool waits for one, then Enter:
/// - `x,y`          absolute;
/// - `@dx,dy`       relative to the tool's last point;
/// - `@length<angle` polar from the last point, the angle in degrees,
///   counter-clockwise from +X (`length<angle` is polar from the origin);
/// - `length`       that far from the last point toward the cursor.
///
/// Each length is in the document's unit, or carries its own ("2in,3in",
/// "@50 mm<30", "1' 6\""); each angle is in degrees, or says "rad"
/// (Phase 154).
///
/// What is not a point is refused whole, and the prompt says why; it is never
/// read as whatever part of it makes sense. Keys are read by their code, not
/// their text, so a keyboard layout or a test's synthetic event types the
/// same.
class TypedPoint {
public:
    /// Handle a key (a Qt::Key): a digit, `.`, `-`, `,`, `@`, `<`; once
    /// something is typed, what units are written with (TypedUnits);
    /// Backspace while something is typed. True when it was one of those.
    bool key(int key);

    /// Something is being typed.
    bool typing() const { return !m_text.empty(); }
    const std::string& text() const { return m_text; }

    /// Drop what is typed and any refusal.
    void clear();

    /// Resolve the text against the tool's last point (@p base) and the
    /// point the cursor shows (@p toward), its bare lengths in @p unit. The
    /// point; or nothing, with the text kept as refused and @p why saying
    /// why.
    std::optional<math::Vec2> take(const std::optional<math::Vec2>& base,
                                   const std::optional<math::Vec2>& toward,
                                   math::LengthUnit unit = math::LengthUnit::Millimetre);

    /// For the prompt, with a leading gap: what is typed, with the unit a
    /// bare number is in ("  Point (mm): @5<30"), else the last refusal
    /// ("  '5,,3' is not a point: ..."), else "".
    std::string prompt(math::LengthUnit unit = math::LengthUnit::Millimetre) const;

    /// The point @p text names, as take() reads it; @p why says why not.
    static std::optional<math::Vec2> resolve(std::string_view text,
                                             const std::optional<math::Vec2>& base,
                                             const std::optional<math::Vec2>& toward,
                                             std::string* why = nullptr,
                                             math::LengthUnit unit = math::LengthUnit::Millimetre);

private:
    std::string m_text;
    std::string m_refused;
    std::string m_reason;
};

}  // namespace hz::ui
