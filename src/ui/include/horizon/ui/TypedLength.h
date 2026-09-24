#pragma once

#include <optional>
#include <string>
#include <string_view>

namespace hz::ui {

/// A length typed at a tool while it runs (a fillet radius, a chamfer
/// distance): digits and a point, Backspace, then Enter to take it. The whole
/// text must be one positive number. Anything else, such as "." or "1.2.3",
/// is refused, and the prompt says so; it is not read as whatever beginning
/// of it parses.
class TypedLength {
public:
    explicit TypedLength(double value) : m_value(value) {}

    /// Handle a key (a Qt::Key). True when it was one of the input's own:
    /// a digit, the point, Backspace with something to take back, Enter.
    bool key(int key);

    double value() const { return m_value; }

    /// Drop what is being typed and any refusal; the value stays.
    void clear();

    /// For the tool's prompt, with a leading gap: what is being typed
    /// ("  Radius: 2.5"), else a refusal ("  '1.2.3' is not a radius"), else
    /// the value ("  [radius=2.5]"). @p name is lower case.
    std::string prompt(const std::string& name) const;

    /// The whole of @p text as one positive, finite number, or nothing.
    static std::optional<double> parse(std::string_view text);

private:
    double m_value;
    std::string m_text;
    std::string m_refused;
};

}  // namespace hz::ui
