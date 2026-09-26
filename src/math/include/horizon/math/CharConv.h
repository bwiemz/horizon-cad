#pragma once

#include <charconv>

namespace hz::math {

/// std::from_chars for a double, on every platform (Phase 169): the standard
/// one where the library has it; else the same grammar (an optional '-',
/// digits with an optional point, an optional exponent; or "inf", "infinity"
/// or "nan"), read in the C locale whatever the process's is. Apple's libc++
/// has no floating-point from_chars. Like std::from_chars it skips no space
/// and takes no '+', and @p value is left as it was on a failure.
std::from_chars_result fromChars(const char* first, const char* last, double& value);

}  // namespace hz::math
