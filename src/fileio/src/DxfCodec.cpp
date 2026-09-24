#include "DxfCodec.h"

#include <array>
#include <cctype>
#include <climits>
#include <cmath>
#include <iterator>

namespace hz::io::dxf {

namespace {

// ===========================================================================
// The AutoCAD Color Index
// ===========================================================================

struct Rgb {
    int r, g, b;
};

/// The full 256-entry palette. 1-9 are named colours. 10-249 run through 24
/// hues 15 degrees apart. Each hue has ten entries: five brightness levels,
/// each followed by a pale version at half saturation. 250-255 are greys.
const std::array<Rgb, 256>& aciTable() {
    static const std::array<Rgb, 256> table = [] {
        std::array<Rgb, 256> t{};
        const Rgb named[10] = {{0, 0, 0},       {255, 0, 0},    {255, 255, 0}, {0, 255, 0},
                               {0, 255, 255},   {0, 0, 255},    {255, 0, 255}, {255, 255, 255},
                               {128, 128, 128}, {192, 192, 192}};
        for (int i = 0; i < 10; ++i) t[static_cast<size_t>(i)] = named[i];
        const double levels[5] = {1.0, 0.65, 0.5, 0.3, 0.15};
        for (int i = 10; i < 250; ++i) {
            const int degrees = (i - 10) / 10 * 15;
            const double f = (degrees % 60) / 60.0;
            double rgb[3] = {0.0, 0.0, 0.0};
            switch (degrees / 60) {
                case 0:
                    rgb[0] = 1.0;
                    rgb[1] = f;
                    break;
                case 1:
                    rgb[0] = 1.0 - f;
                    rgb[1] = 1.0;
                    break;
                case 2:
                    rgb[1] = 1.0;
                    rgb[2] = f;
                    break;
                case 3:
                    rgb[1] = 1.0 - f;
                    rgb[2] = 1.0;
                    break;
                case 4:
                    rgb[0] = f;
                    rgb[2] = 1.0;
                    break;
                default:
                    rgb[0] = 1.0;
                    rgb[2] = 1.0 - f;
                    break;
            }
            const int k = (i - 10) % 10;
            int channels[3] = {0, 0, 0};
            for (int c = 0; c < 3; ++c) {
                const double value = (k % 2 == 1 ? 0.5 + 0.5 * rgb[c] : rgb[c]) * levels[k / 2];
                channels[c] = static_cast<int>(std::floor(value * 255.0 + 1e-9));
            }
            t[static_cast<size_t>(i)] = {channels[0], channels[1], channels[2]};
        }
        const int greys[6] = {51, 80, 105, 130, 190, 255};
        for (size_t i = 0; i < 6; ++i) t[250 + i] = {greys[i], greys[i], greys[i]};
        return t;
    }();
    return table;
}

// ===========================================================================
// Code pages: the characters of bytes 0x80-0xFF
// ===========================================================================

using HighHalf = std::array<char32_t, 128>;

/// Windows-1252 (Western European): 0x80-0x9F its own, 0xA0-0xFF Latin-1.
constexpr HighHalf makeCp1252() {
    HighHalf t{};
    constexpr char32_t c1[32] = {0x20AC, 0,      0x201A, 0x0192, 0x201E, 0x2026, 0x2020, 0x2021,
                                 0x02C6, 0x2030, 0x0160, 0x2039, 0x0152, 0,      0x017D, 0,
                                 0,      0x2018, 0x2019, 0x201C, 0x201D, 0x2022, 0x2013, 0x2014,
                                 0x02DC, 0x2122, 0x0161, 0x203A, 0x0153, 0,      0x017E, 0x0178};
    for (size_t i = 0; i < 32; ++i) t[i] = c1[i];
    for (size_t i = 32; i < 128; ++i) t[i] = static_cast<char32_t>(0x80 + i);
    return t;
}

/// Windows-1251 (Cyrillic): 0xC0-0xFF are А-я in order.
constexpr HighHalf makeCp1251() {
    HighHalf t{};
    constexpr char32_t low[64] = {
        0x0402, 0x0403, 0x201A, 0x0453, 0x201E, 0x2026, 0x2020, 0x2021, 0x20AC, 0x2030, 0x0409,
        0x2039, 0x040A, 0x040C, 0x040B, 0x040F, 0x0452, 0x2018, 0x2019, 0x201C, 0x201D, 0x2022,
        0x2013, 0x2014, 0,      0x2122, 0x0459, 0x203A, 0x045A, 0x045C, 0x045B, 0x045F, 0x00A0,
        0x040E, 0x045E, 0x0408, 0x00A4, 0x0490, 0x00A6, 0x00A7, 0x0401, 0x00A9, 0x0404, 0x00AB,
        0x00AC, 0x00AD, 0x00AE, 0x0407, 0x00B0, 0x00B1, 0x0406, 0x0456, 0x0491, 0x00B5, 0x00B6,
        0x00B7, 0x0451, 0x2116, 0x0454, 0x00BB, 0x0458, 0x0405, 0x0455, 0x0457};
    for (size_t i = 0; i < 64; ++i) t[i] = low[i];
    for (size_t i = 64; i < 128; ++i) t[i] = static_cast<char32_t>(0x0410 + (i - 64));
    return t;
}

constexpr HighHalf kCp1252 = makeCp1252();
constexpr HighHalf kCp1251 = makeCp1251();

bool equalsIgnoringCase(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::toupper(static_cast<unsigned char>(a[i])) !=
            std::toupper(static_cast<unsigned char>(b[i]))) {
            return false;
        }
    }
    return true;
}

bool isHex(char c) {
    return std::isxdigit(static_cast<unsigned char>(c)) != 0;
}

bool isDigit(char c) {
    return std::isdigit(static_cast<unsigned char>(c)) != 0;
}

/// The character of a \\U+XXXX escape at s[i] (four hex digits), or false.
bool unicodeEscape(std::string_view s, size_t i, char32_t& cp) {
    if (i + 7 > s.size() || s.compare(i, 3, "\\U+") != 0) return false;
    cp = 0;
    for (size_t k = i + 3; k < i + 7; ++k) {
        const char c = s[k];
        if (!isHex(c)) return false;
        const int d = isDigit(c) ? c - '0' : std::toupper(static_cast<unsigned char>(c)) - 'A' + 10;
        cp = cp * 16 + static_cast<char32_t>(d);
    }
    return true;
}

// ===========================================================================
// Codes shared by TEXT and MTEXT
// ===========================================================================

/// A %% code at s[i]: %%d, %%p, %%c, %%%, %%nnn, or the %%u / %%o / %%k
/// toggles. Advances `i` past it; false if s[i] does not start one.
bool percentCode(std::string_view s, size_t& i, const Decoder& decoder, std::string& out,
                 TextLosses& losses) {
    if (i + 2 >= s.size() || s[i] != '%' || s[i + 1] != '%') return false;
    const char c = s[i + 2];
    switch (std::tolower(static_cast<unsigned char>(c))) {
        case 'd':
            appendUtf8(out, 0x00B0);
            i += 3;
            return true;
        case 'p':
            appendUtf8(out, 0x00B1);
            i += 3;
            return true;
        case 'c':
            appendUtf8(out, 0x2300);
            i += 3;
            return true;
        case '%':
            out += '%';
            i += 3;
            return true;
        case 'u':
        case 'o':
        case 'k':
            losses.styling = true;
            i += 3;
            return true;
        default:
            break;
    }
    if (!isDigit(c)) return false;
    size_t j = i + 2;
    int code = 0;
    while (j < s.size() && j < i + 5 && isDigit(s[j])) code = code * 10 + (s[j++] - '0');
    i = j;
    if (code >= 0x20 && code < 0x80) {
        out += static_cast<char>(code);
    } else if (code >= 0x80 && code < 0x100) {
        const char32_t cp = decoder.charFor(static_cast<unsigned char>(code));
        appendUtf8(out, cp != 0 ? cp : 0xFFFD);
    }
    return true;
}

/// \U+XXXX (a Unicode character) or \M+nXXXX (a character in a multibyte
/// code page, which is not read) at s[i].
bool unicodeCode(std::string_view s, size_t& i, std::string& out, TextLosses& losses) {
    if (char32_t cp = 0; unicodeEscape(s, i, cp)) {
        i += 7;
        char32_t low = 0;
        if (cp >= 0xD800 && cp <= 0xDBFF && unicodeEscape(s, i, low) && low >= 0xDC00 &&
            low <= 0xDFFF) {
            // A surrogate pair, as a writer working in UTF-16 might leave it.
            cp = 0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00);
            i += 7;
        }
        if (cp != 0) appendUtf8(out, cp >= 0xD800 && cp <= 0xDFFF ? 0xFFFD : cp);
        return true;
    }
    if (s.compare(i, 3, "\\M+") == 0 && i + 8 <= s.size() && isDigit(s[i + 3])) {
        appendUtf8(out, 0xFFFD);
        losses.unreadable = true;
        i += 8;
        return true;
    }
    return false;
}

/// A ^ code at s[i]: "^ " is a caret, and ^I, ^J and ^M a tab, line feed
/// and carriage return. The DXF reference makes every ^@ to ^_ a control
/// character, but only these three mean anything in a drawing, and text
/// such as "2^N" from a writer that does not escape its carets stays as it
/// is.
bool caretCode(std::string_view s, size_t& i, std::string& out) {
    if (s[i] != '^' || i + 1 >= s.size()) return false;
    const char next = s[i + 1];
    if (next == ' ') {
        out += '^';
    } else if (next == 'I' || next == 'J' || next == 'M') {
        out += static_cast<char>(next - '@');
    } else {
        return false;
    }
    i += 2;
    return true;
}

/// The length of a degree, plus-minus or diameter sign at text[i] (UTF-8),
/// and the letter of its %% code; 0 if there is none.
size_t signAt(std::string_view text, size_t i, char* letter) {
    struct Sign {
        std::string_view utf8;
        char letter;
    };
    static constexpr Sign kSigns[] = {{"\xC2\xB0", 'd'}, {"\xC2\xB1", 'p'}, {"\xE2\x8C\x80", 'c'}};
    for (const auto& sign : kSigns) {
        if (text.compare(i, sign.utf8.size(), sign.utf8) == 0) {
            if (letter) *letter = sign.letter;
            return sign.utf8.size();
        }
    }
    return 0;
}

}  // namespace

// ===========================================================================
// Colour
// ===========================================================================

uint32_t aciToArgb(int aci) {
    if (aci < 0) aci = -aci;
    if (aci == 0 || aci > 255) return 0x00000000;  // ByBlock, ByLayer
    const Rgb& e = aciTable()[static_cast<size_t>(aci)];
    return 0xFF000000u | (static_cast<uint32_t>(e.r) << 16) | (static_cast<uint32_t>(e.g) << 8) |
           static_cast<uint32_t>(e.b);
}

int argbToAci(uint32_t argb) {
    if (argb == 0x00000000) return 256;  // ByLayer
    const int r = static_cast<int>((argb >> 16) & 0xFF);
    const int g = static_cast<int>((argb >> 8) & 0xFF);
    const int b = static_cast<int>(argb & 0xFF);
    if (r == 0 && g == 0 && b == 0) return 7;
    int best = 7;
    long bestDist = LONG_MAX;
    for (int i = 1; i < 256; ++i) {
        const Rgb& e = aciTable()[static_cast<size_t>(i)];
        const long dr = r - e.r;
        const long dg = g - e.g;
        const long db = b - e.b;
        const long dist = dr * dr + dg * dg + db * db;
        if (dist < bestDist) {
            bestDist = dist;
            best = i;
            if (dist == 0) break;
        }
    }
    return best;
}

bool isAciColor(uint32_t argb) {
    if (argb == 0x00000000) return true;
    return (aciToArgb(argbToAci(argb)) & 0xFFFFFFu) == (argb & 0xFFFFFFu);
}

uint32_t trueColorToArgb(int value) {
    return 0xFF000000u | (static_cast<uint32_t>(value) & 0xFFFFFFu);
}

// ===========================================================================
// Code pages
// ===========================================================================

Decoder::Decoder() : m_high(kCp1252.data()), m_name("ANSI_1252") {}

bool Decoder::setCodepage(const std::string& name) {
    m_name = name;
    if (equalsIgnoringCase(name, "ANSI_1252")) {
        m_high = kCp1252.data();
    } else if (equalsIgnoringCase(name, "ANSI_1251")) {
        m_high = kCp1251.data();
    } else {
        m_high = nullptr;
        return false;
    }
    return true;
}

char32_t Decoder::charFor(unsigned char b) const {
    if (b < 0x80) return b;
    return m_high ? m_high[b - 0x80] : 0;
}

std::string Decoder::decode(std::string value) {
    bool high = false;
    for (const char c : value) high = high || static_cast<unsigned char>(c) >= 0x80;
    if (!high || isValidUtf8(value)) return value;
    std::string out;
    out.reserve(value.size() * 2);
    for (const char c : value) {
        const auto b = static_cast<unsigned char>(c);
        if (b < 0x80) {
            out += c;
            continue;
        }
        char32_t cp = charFor(b);
        if (cp == 0) {
            cp = 0xFFFD;
            ++m_undecodable;
        }
        appendUtf8(out, cp);
    }
    return out;
}

bool isValidUtf8(std::string_view s) {
    size_t i = 0;
    while (i < s.size()) {
        const auto c = static_cast<unsigned char>(s[i]);
        if (c < 0x80) {
            ++i;
            continue;
        }
        size_t n = 0;
        char32_t cp = 0;
        char32_t min = 0;
        if ((c & 0xE0) == 0xC0) {
            n = 1;
            cp = c & 0x1F;
            min = 0x80;
        } else if ((c & 0xF0) == 0xE0) {
            n = 2;
            cp = c & 0x0F;
            min = 0x800;
        } else if ((c & 0xF8) == 0xF0) {
            n = 3;
            cp = c & 0x07;
            min = 0x10000;
        } else {
            return false;
        }
        if (s.size() - i <= n) return false;
        for (size_t k = 1; k <= n; ++k) {
            const auto b = static_cast<unsigned char>(s[i + k]);
            if ((b & 0xC0) != 0x80) return false;
            cp = (cp << 6) | (b & 0x3F);
        }
        if (cp < min || cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF)) return false;
        i += n + 1;
    }
    return true;
}

void appendUtf8(std::string& out, char32_t cp) {
    if (cp < 0x80) {
        out += static_cast<char>(cp);
    } else if (cp < 0x800) {
        out += static_cast<char>(0xC0 | (cp >> 6));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
        out += static_cast<char>(0xE0 | (cp >> 12));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
        out += static_cast<char>(0xF0 | (cp >> 18));
        out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    }
}

// ===========================================================================
// TEXT and MTEXT
// ===========================================================================

std::string decodeText(std::string_view value, const Decoder& decoder, TextLosses& losses) {
    std::string out;
    out.reserve(value.size());
    for (size_t i = 0; i < value.size();) {
        if (percentCode(value, i, decoder, out, losses) || unicodeCode(value, i, out, losses) ||
            caretCode(value, i, out)) {
            continue;
        }
        out += value[i++];
    }
    return out;
}

std::vector<std::string> decodeMText(std::string_view value, const Decoder& decoder,
                                     TextLosses& losses) {
    std::vector<std::string> lines(1);
    // Past a code's arguments, which end at ';'.
    const auto skipArguments = [&value](size_t& i) {
        const size_t end = value.find(';', i);
        i = end == std::string_view::npos ? value.size() : end + 1;
    };
    for (size_t i = 0; i < value.size();) {
        const char c = value[i];
        if (c == '\\' && i + 1 < value.size()) {
            if (unicodeCode(value, i, lines.back(), losses)) continue;
            const char code = value[i + 1];
            i += 2;
            switch (code) {
                case 'P':  // paragraph
                case 'N':  // column
                case 'X':  // dimension text: the line below the dimension line
                    lines.emplace_back();
                    break;
                case '~':
                    appendUtf8(lines.back(), 0x00A0);
                    break;
                case '\\':
                case '{':
                case '}':
                    lines.back() += code;
                    break;
                case 'L':  // underline, overline, strike-through on
                case 'O':
                case 'K':
                    losses.styling = true;
                    break;
                case 'l':  // ... and off
                case 'o':
                case 'k':
                    break;
                case 'S': {  // a stacked fraction: \Stop/bottom; (or # or ^)
                    const size_t end = value.find(';', i);
                    const std::string_view body = value.substr(
                        i, end == std::string_view::npos ? std::string_view::npos : end - i);
                    i = end == std::string_view::npos ? value.size() : end + 1;
                    const size_t split = body.find_first_of("/#^");
                    lines.back() += body.substr(0, split);
                    if (split != std::string_view::npos) {
                        std::string_view bottom = body.substr(split + 1);
                        while (!bottom.empty() && bottom.front() == ' ') bottom.remove_prefix(1);
                        lines.back() += '/';
                        lines.back() += bottom;
                    }
                    losses.stacked = true;
                    break;
                }
                case 'f':  // font, alignment, paragraph layout: nothing drawn changes
                case 'F':
                case 'A':
                case 'p':
                    skipArguments(i);
                    break;
                case 'H':  // height, width, oblique, tracking, colour
                case 'W':
                case 'Q':
                case 'T':
                case 'C':
                case 'c':
                    losses.styling = true;
                    skipArguments(i);
                    break;
                default:
                    lines.back() += code;
                    break;
            }
            continue;
        }
        if (c == '{' || c == '}') {
            ++i;
            continue;
        }
        if (c == '^' && i + 1 < value.size() && value[i + 1] == 'J') {
            lines.emplace_back();
            i += 2;
            continue;
        }
        if (percentCode(value, i, decoder, lines.back(), losses) ||
            caretCode(value, i, lines.back())) {
            continue;
        }
        lines.back() += c;
        ++i;
    }
    while (lines.size() > 1 && lines.back().empty()) lines.pop_back();
    return lines;
}

std::string encodeText(std::string_view text) {
    std::string out;
    out.reserve(text.size());
    for (size_t i = 0; i < text.size();) {
        char letter = 0;
        if (const size_t n = signAt(text, i, &letter); n > 0) {
            out += "%%";
            out += letter;
            i += n;
            continue;
        }
        const char c = text[i];
        if (c == '%') {
            // A percent sign is read as the start of a %% code when the next
            // thing written also starts with one.
            const bool nextIsPercent =
                i + 1 < text.size() && (text[i + 1] == '%' || signAt(text, i + 1, nullptr) > 0);
            out += nextIsPercent ? "%%%" : "%";
        } else if (c == '^') {
            const char next = i + 1 < text.size() ? text[i + 1] : '\0';
            out += next == ' ' || (next >= '@' && next <= '_') ? "^ " : "^";
        } else if (static_cast<unsigned char>(c) < 0x20) {
            out += '^';
            out += static_cast<char>(c + '@');
        } else {
            out += c;
        }
        ++i;
    }
    return out;
}

// ===========================================================================
// Units
// ===========================================================================

double insunitsToMillimetres(int code, std::string* name) {
    struct Unit {
        double mm;
        const char* name;
    };
    static const Unit kUnits[] = {
        {0.0, ""},
        {25.4, "inches"},
        {304.8, "feet"},
        {1609344.0, "miles"},
        {1.0, "millimetres"},
        {10.0, "centimetres"},
        {1000.0, "metres"},
        {1.0e6, "kilometres"},
        {25.4e-6, "microinches"},
        {0.0254, "mils"},
        {914.4, "yards"},
        {1.0e-7, "angstroms"},
        {1.0e-6, "nanometres"},
        {1.0e-3, "microns"},
        {100.0, "decimetres"},
        {1.0e4, "decametres"},
        {1.0e5, "hectometres"},
        {1.0e12, "gigametres"},
        {1.495978707e14, "astronomical units"},
        {9.4607304725808e18, "light years"},
        {3.0856775814913673e19, "parsecs"},
        {1200.0 / 3937.0 * 1000.0, "US survey feet"},
    };
    if (code < 1 || code >= static_cast<int>(std::size(kUnits))) return 0.0;
    const Unit& unit = kUnits[code];
    if (name) *name = unit.name;
    return unit.mm;
}

}  // namespace hz::io::dxf
