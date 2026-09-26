#include "horizon/math/CharConv.h"

#include <cctype>
#include <cerrno>
#include <clocale>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <string>
#include <system_error>

#if defined(__APPLE__)
#include <xlocale.h>
#endif

namespace hz::math {

std::from_chars_result fromChars(const char* first, const char* last, double& value) {
#if defined(__cpp_lib_to_chars)
    return std::from_chars(first, last, value);
#else
    return detail::fromCharsPortable(first, last, value);
#endif
}

namespace detail {

namespace {

bool digit(char c) {
    return std::isdigit(static_cast<unsigned char>(c)) != 0;
}

/// How much of [first, last) is a number, as std::from_chars reads one.
std::size_t numberLength(const char* first, const char* last) {
    const char* p = first;
    if (p < last && *p == '-') ++p;
    const auto word = [&](const char* w) {
        const char* q = p;
        for (; *w != '\0'; ++w, ++q) {
            if (q == last || std::tolower(static_cast<unsigned char>(*q)) != *w) return false;
        }
        p = q;
        return true;
    };
    if (word("infinity") || word("inf") || word("nan")) return static_cast<std::size_t>(p - first);
    const char* digits = p;
    while (p < last && digit(*p)) ++p;
    bool any = p != digits;
    if (p < last && *p == '.') {
        const char* fraction = ++p;
        while (p < last && digit(*p)) ++p;
        any = any || p != fraction;
    }
    if (!any) return 0;
    if (p < last && (*p == 'e' || *p == 'E')) {
        const char* exponent = p + 1;
        if (exponent < last && (*exponent == '-' || *exponent == '+')) ++exponent;
        if (exponent < last && digit(*exponent)) {
            p = exponent;
            while (p < last && digit(*p)) ++p;
        }
    }
    return static_cast<std::size_t>(p - first);
}

double readInCLocale(const std::string& text, char** end) {
#if defined(_WIN32)
    static const _locale_t c = _create_locale(LC_NUMERIC, "C");
    return _strtod_l(text.c_str(), end, c);
#else
    static const locale_t c = newlocale(LC_NUMERIC_MASK, "C", nullptr);
    return strtod_l(text.c_str(), end, c);
#endif
}

}  // namespace

std::from_chars_result fromCharsPortable(const char* first, const char* last, double& value) {
    const std::size_t length = numberLength(first, last);
    if (length == 0) return {first, std::errc::invalid_argument};
    const std::string text(first, length);
    char* end = nullptr;
    errno = 0;
    const double read = readInCLocale(text, &end);
    if (end != text.c_str() + text.size()) return {first, std::errc::invalid_argument};
    // Out of range: too large, or so small it is lost (not a mere denormal).
    if (errno == ERANGE && (std::isinf(read) || read == 0.0)) {
        return {first + length, std::errc::result_out_of_range};
    }
    value = read;
    return {first + length, std::errc()};
}

}  // namespace detail

}  // namespace hz::math
