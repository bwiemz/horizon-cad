#include "horizon/math/Units.h"

#include <cctype>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <vector>

#include "horizon/math/CharConv.h"
#include "horizon/math/Constants.h"

namespace hz::math {

namespace {

struct UnitInfo {
    LengthUnit unit;
    std::string_view symbol;
    double millimetres;
};

constexpr std::array<UnitInfo, 5> kInfo{{
    {LengthUnit::Millimetre, "mm", 1.0},
    {LengthUnit::Centimetre, "cm", 10.0},
    {LengthUnit::Metre, "m", 1000.0},
    {LengthUnit::Inch, "in", 25.4},
    {LengthUnit::Foot, "ft", 304.8},
}};

const UnitInfo& infoOf(LengthUnit unit) {
    for (const UnitInfo& info : kInfo) {
        if (info.unit == unit) return info;
    }
    return kInfo[0];
}

std::string lowered(std::string_view text) {
    std::string out(text);
    for (char& c : out) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return out;
}

constexpr std::string_view kDegreeSign = "\xC2\xB0";  // °

/// One term of a typed value: a number, and the unit written after it.
struct Term {
    double value = 0.0;
    std::string unit;  ///< empty when none was written
    bool fraction = false;
    bool integer = false;
};

/// @p text cut into terms, after its sign; nullopt when it cannot be.
std::optional<std::vector<Term>> termsOf(std::string_view text, double& sign) {
    std::size_t i = 0;
    const auto skipSpace = [&] {
        while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i])) != 0) ++i;
    };
    // A number: digits and a point, and an exponent; no sign, no "inf".
    const auto number = [&](double& value, bool& integer) {
        if (i >= text.size() ||
            (std::isdigit(static_cast<unsigned char>(text[i])) == 0 && text[i] != '.')) {
            return false;
        }
        const char* first = text.data() + i;
        const auto [end, error] = fromChars(first, text.data() + text.size(), value);
        if (error != std::errc() || !std::isfinite(value)) return false;
        const std::string_view digits(first, static_cast<std::size_t>(end - first));
        integer = digits.find_first_not_of("0123456789") == std::string_view::npos;
        i += digits.size();
        return true;
    };

    skipSpace();
    sign = 1.0;
    if (i < text.size() && (text[i] == '-' || text[i] == '+')) {
        sign = text[i] == '-' ? -1.0 : 1.0;
        ++i;
    }
    std::vector<Term> terms;
    skipSpace();
    while (i < text.size()) {
        Term term;
        if (!number(term.value, term.integer)) return std::nullopt;
        skipSpace();
        if (i < text.size() && text[i] == '/') {
            ++i;
            skipSpace();
            double denominator = 0.0;
            bool unused = false;
            if (!number(denominator, unused) || !(denominator > 0.0)) return std::nullopt;
            term.value /= denominator;
            term.fraction = true;
            term.integer = false;
            skipSpace();
        }
        // Its unit: a word, a mark, or a degree sign.
        if (i < text.size() && (text[i] == '\'' || text[i] == '"')) {
            term.unit = std::string(1, text[i++]);
        } else if (text.compare(i, kDegreeSign.size(), kDegreeSign) == 0) {
            term.unit = std::string(kDegreeSign);
            i += kDegreeSign.size();
        } else {
            while (i < text.size() && std::isalpha(static_cast<unsigned char>(text[i])) != 0) {
                term.unit += text[i++];
            }
        }
        terms.push_back(std::move(term));
        skipSpace();
    }
    if (terms.empty()) return std::nullopt;
    return terms;
}

}  // namespace

double millimetresPer(LengthUnit unit) {
    return infoOf(unit).millimetres;
}

std::string_view symbolOf(LengthUnit unit) {
    return infoOf(unit).symbol;
}

std::optional<LengthUnit> lengthUnitFrom(std::string_view text) {
    if (text == "'") return LengthUnit::Foot;
    if (text == "\"") return LengthUnit::Inch;
    const std::string word = lowered(text);
    for (const UnitInfo& info : kInfo) {
        if (word == info.symbol) return info.unit;
    }
    struct Name {
        std::string_view name;
        LengthUnit unit;
    };
    static constexpr Name kNames[] = {
        {"millimetre", LengthUnit::Millimetre},
        {"millimetres", LengthUnit::Millimetre},
        {"millimeter", LengthUnit::Millimetre},
        {"millimeters", LengthUnit::Millimetre},
        {"centimetre", LengthUnit::Centimetre},
        {"centimetres", LengthUnit::Centimetre},
        {"centimeter", LengthUnit::Centimetre},
        {"centimeters", LengthUnit::Centimetre},
        {"metre", LengthUnit::Metre},
        {"metres", LengthUnit::Metre},
        {"meter", LengthUnit::Metre},
        {"meters", LengthUnit::Metre},
        {"inch", LengthUnit::Inch},
        {"inches", LengthUnit::Inch},
        {"foot", LengthUnit::Foot},
        {"feet", LengthUnit::Foot},
    };
    for (const Name& name : kNames) {
        if (word == name.name) return name.unit;
    }
    return std::nullopt;
}

std::string formatLength(double mm, LengthUnit unit, int decimals, bool withSymbol) {
    double value = mm / millimetresPer(unit);
    decimals = decimals < 0 ? 0 : decimals;
    if (std::abs(value) < 0.5 * std::pow(10.0, -decimals)) value = 0.0;  // no "-0.000"
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::fixed << std::setprecision(decimals) << value;
    if (withSymbol) out << ' ' << symbolOf(unit);
    return out.str();
}

std::optional<double> parseLength(std::string_view text, LengthUnit unit) {
    double sign = 1.0;
    auto terms = termsOf(text, sign);
    if (!terms) return std::nullopt;
    // A whole number and the fraction after it are one ("1 1/2 in").
    for (std::size_t k = 0; k + 1 < terms->size(); ++k) {
        Term& whole = (*terms)[k];
        const Term& part = (*terms)[k + 1];
        if (whole.unit.empty() && whole.integer && part.fraction) {
            whole.value += part.value;
            whole.unit = part.unit;
            whole.fraction = true;
            terms->erase(terms->begin() + static_cast<std::ptrdiff_t>(k) + 1);
        }
    }
    double mm = 0.0;
    std::optional<LengthUnit> previous;
    for (std::size_t k = 0; k < terms->size(); ++k) {
        const Term& term = (*terms)[k];
        std::optional<LengthUnit> in;
        if (!term.unit.empty()) {
            in = lengthUnitFrom(term.unit);
            if (!in) return std::nullopt;
        } else if (terms->size() == 1) {
            in = unit;
        } else if (k + 1 == terms->size() && previous == LengthUnit::Foot) {
            in = LengthUnit::Inch;  // 1' 6
        } else {
            return std::nullopt;  // two numbers, and no saying what either is
        }
        mm += term.value * millimetresPer(*in);
        previous = in;
    }
    mm *= sign;
    if (!std::isfinite(mm)) return std::nullopt;
    return mm;
}

std::optional<double> parseAngle(std::string_view text) {
    double sign = 1.0;
    const auto terms = termsOf(text, sign);
    if (!terms || terms->size() != 1) return std::nullopt;
    const Term& term = terms->front();
    const std::string word = lowered(term.unit);
    double radians = 0.0;
    if (word.empty() || word == kDegreeSign || word == "deg" || word == "degree" ||
        word == "degrees") {
        radians = term.value * kDegToRad;
    } else if (word == "rad" || word == "radian" || word == "radians") {
        radians = term.value;
    } else {
        return std::nullopt;
    }
    radians *= sign;
    if (!std::isfinite(radians)) return std::nullopt;
    return radians;
}

}  // namespace hz::math
