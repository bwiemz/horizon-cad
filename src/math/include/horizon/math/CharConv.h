#pragma once

#include <charconv>

namespace hz::math {

/// std::from_chars for a double, on every platform (Phase 169): the standard
/// one where the library has it, else detail::fromCharsPortable. Apple's
/// libc++ has no floating-point from_chars. Like std::from_chars it skips no
/// space and takes no '+', reads in no locale but the C one, and leaves
/// @p value as it was on a failure.
std::from_chars_result fromChars(const char* first, const char* last, double& value);

namespace detail {

/// The same grammar as std::from_chars (an optional '-', digits with an
/// optional point, an optional exponent; or "inf", "infinity" or "nan"),
/// read with strtod_l in the C locale. What fromChars is where the library
/// has no floating-point from_chars; compiled everywhere, so it is tested
/// against the standard one where there is one.
std::from_chars_result fromCharsPortable(const char* first, const char* last, double& value);

}  // namespace detail

}  // namespace hz::math
