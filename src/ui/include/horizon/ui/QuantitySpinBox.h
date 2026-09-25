#pragma once

#include <QDoubleSpinBox>
#include <functional>
#include <optional>
#include <string>

#include "horizon/math/Units.h"

namespace hz::ui {

/// A number field for a length or an angle (Phase 154).
///
/// Its value() is what the model keeps and what callers read: millimetres
/// for a length, degrees for an angle. It is shown in the document's unit
/// with its symbol ("1.000 in", "30.000°"), and takes a value typed in any
/// unit ("2 in", "50.8 mm", "1' 6\"", "0.5 rad"); a bare number is in the
/// document's unit (degrees, for an angle). Arrows step by one unit.
class QuantitySpinBox : public QDoubleSpinBox {
    Q_OBJECT

public:
    enum class Kind { Length, Angle };

    /// Shown to @p decimals places; a length in @p unit.
    QuantitySpinBox(Kind kind, math::LengthUnit unit, int decimals, QWidget* parent = nullptr);

    Kind kind() const { return m_kind; }
    math::LengthUnit unit() const { return m_unit; }
    /// Show a length in @p unit from now on (the document shown changed);
    /// its value stays.
    void setUnit(math::LengthUnit unit);

    /// An expression typed after "=" worked out (Phase 155): its value (in
    /// millimetres, or degrees) and the expression as it is kept.
    struct Worked {
        double value = 0.0;
        std::string kept;
    };
    /// Works out an expression, or says why not in its second argument.
    using Resolver = std::function<std::optional<Worked>(const std::string&, std::string*)>;
    /// Take "=expression" too ("=wall * 2"), worked out by @p resolver: the
    /// field shows the expression and holds its value.
    void setResolver(Resolver resolver) { m_resolver = std::move(resolver); }
    /// The expression the value is from, as kept; empty when it is a number
    /// (typed, or stepped to with the arrows).
    std::string expression() const;
    /// Show @p kept, an expression as kept, for its @p value.
    void setExpression(const std::string& kept, double value);

    void stepBy(int steps) override;

protected:
    QString textFromValue(double value) const override;
    double valueFromText(const QString& text) const override;
    QValidator::State validate(QString& input, int& pos) const override;

private:
    /// @p text read, in millimetres or degrees; nullopt when it is not a
    /// value (or is one only in part, while it is being typed).
    std::optional<double> read(const QString& text) const;
    /// Whether @p value is the one the expression shown gave.
    bool fromExpression(double value) const;

    Kind m_kind;
    math::LengthUnit m_unit;
    int m_shown;
    Resolver m_resolver;
    /// The expression shown, and the value it gave: valueFromText, const,
    /// takes one typed.
    mutable std::string m_expression;
    mutable double m_expressionValue = 0.0;
};

}  // namespace hz::ui
