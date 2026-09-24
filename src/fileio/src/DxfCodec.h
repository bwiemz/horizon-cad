#pragma once

// Strings and colours as a DXF file writes them: the AutoCAD Color Index,
// the pre-2007 code pages, and the codes inside TEXT and MTEXT values.

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace hz::io::dxf {

// ---------------------------------------------------------------------------
// Colour
// ---------------------------------------------------------------------------

/// The ARGB colour of ACI index `aci` (1-255). 0 (ByBlock) and 256 (ByLayer)
/// give 0, the inherit colour; a negative index (a layer that is off) gives
/// the colour of its absolute value.
uint32_t aciToArgb(int aci);

/// The ACI index for an ARGB colour: 256 (ByLayer) for 0, the exact index
/// where the table has the colour, else the nearest one. Black, which the
/// table does not have, is 7 (drawn black on a light background).
int argbToAci(uint32_t argb);

/// Whether the ACI table holds this colour exactly, so group 62 alone
/// carries it and no true colour (group 420) is needed.
bool isAciColor(uint32_t argb);

/// The ARGB colour of a group-420 true colour (0x00RRGGBB).
uint32_t trueColorToArgb(int value);

// ---------------------------------------------------------------------------
// Code pages
// ---------------------------------------------------------------------------

/// Turns the bytes of a DXF value into UTF-8. A value that is valid UTF-8
/// is kept as it is, which is how every file from AutoCAD 2007 on is
/// written. Otherwise each byte above 0x7F is read in the drawing's
/// $DWGCODEPAGE: Windows-1252 unless the header names another. A byte the
/// code page cannot read becomes U+FFFD and is counted.
class Decoder {
public:
    Decoder();

    /// Use the code page named by $DWGCODEPAGE ("ANSI_1252", "ANSI_1251").
    /// Returns false, and keeps the name, if this code page is not known.
    bool setCodepage(const std::string& name);
    const std::string& codepage() const { return m_name; }

    /// `value` as UTF-8.
    std::string decode(std::string value);

    /// The character for byte `b` in this code page, or 0 if it has none.
    char32_t charFor(unsigned char b) const;

    /// How many bytes could not be read.
    int undecodable() const { return m_undecodable; }

private:
    const char32_t* m_high = nullptr;  ///< 128 entries for bytes 0x80-0xFF, or null
    std::string m_name;
    int m_undecodable = 0;
};

bool isValidUtf8(std::string_view s);
void appendUtf8(std::string& out, char32_t cp);

// ---------------------------------------------------------------------------
// TEXT and MTEXT codes
// ---------------------------------------------------------------------------

/// What the codes in a text value asked for that the drawing cannot show.
struct TextLosses {
    bool styling = false;     ///< underline, overline, strike-through, colour, height...
    bool stacked = false;     ///< a stacked fraction, written inline
    bool unreadable = false;  ///< a \M+ character in another code page
};

/// A TEXT value as plain UTF-8: %%d, %%p and %%c become the degree,
/// plus-minus and diameter signs, %%% a percent sign and %%nnn character
/// nnn; \U+XXXX is that character; ^I, ^J and ^M are a tab, line feed and
/// carriage return, and "^ " a caret. Underline and overline toggles (%%u,
/// %%o) are dropped.
std::string decodeText(std::string_view value, const Decoder& decoder, TextLosses& losses);

/// An MTEXT value's paragraphs as plain UTF-8, one string per line. \P
/// breaks a line, and so do \N (a column break) and \X. Formatting codes
/// (\f, \H, \C, \p... up to their ';', and {}) are dropped. \S stacks a
/// fraction and is written inline as "a/b". \~, \\, \{ and \} are
/// their characters. %% codes and \U+ are read as in TEXT.
std::vector<std::string> decodeMText(std::string_view value, const Decoder& decoder,
                                     TextLosses& losses);

/// A TEXT value to write: the degree, plus-minus and diameter signs as
/// %%d, %%p and %%c; a percent sign that could be read as the start of a
/// %% code as %%%; control characters as ^ codes, and a caret that could be
/// read as one as "^ ". decodeText reads it back as `text`, as long as its
/// only control characters are tabs and line breaks.
std::string encodeText(std::string_view text);

/// An MTEXT value to write for @p lines: each line as encodeText writes a
/// TEXT value, with MTEXT's own escapes (\\, \{, \}), joined by \P.
/// decodeMText reads it back as the same lines.
std::string encodeMText(const std::vector<std::string>& lines);

// ---------------------------------------------------------------------------
// Units
// ---------------------------------------------------------------------------

/// Millimetres per drawing unit for a $INSUNITS code, and the unit's name
/// (plural). 0 for 0 (unitless) and for codes this does not know.
double insunitsToMillimetres(int code, std::string* name = nullptr);

}  // namespace hz::io::dxf
