// hz::math::fromChars (Phase 169): std::from_chars for a double on every
// platform. Built twice: as the library has it, and with the portable one
// forced (HZ_PORTABLE_FROM_CHARS), which is checked against the standard one
// here, where the standard library has both.

#include <gtest/gtest.h>

#include <charconv>
#include <clocale>
#include <cmath>
#include <cstring>
#include <string>
#include <system_error>
#include <vector>

#include "horizon/math/CharConv.h"

using hz::math::fromChars;

namespace {

struct Read {
    std::errc error;
    std::ptrdiff_t used;
    double value;
};

Read read(const std::string& text) {
    double value = -7.0;  // left as it is on a failure
    const auto [end, error] = fromChars(text.data(), text.data() + text.size(), value);
    return {error, end - text.data(), value};
}

}  // namespace

TEST(CharConvTest, ReadsWhatFromCharsReads) {
    EXPECT_EQ(read("1.5").value, 1.5);
    EXPECT_EQ(read("-2e3").value, -2000.0);
    EXPECT_EQ(read(".5").value, 0.5);
    EXPECT_EQ(read("5.").value, 5.0);
    EXPECT_EQ(read("1e+5").value, 100000.0);
    EXPECT_EQ(read("1.5x").used, 3) << "it stops at what is not a number";
    EXPECT_EQ(read("1e").used, 1) << "an exponent with no digits is not one";
    EXPECT_EQ(read("0x10").used, 1) << "no hexadecimal";
    EXPECT_TRUE(std::isinf(read("inf").value));
    EXPECT_TRUE(std::isnan(read("nan").value));
}

TEST(CharConvTest, RefusesWhatFromCharsRefuses) {
    for (const char* text : {"", "abc", "+1", " 1", ".", "-", "e5"}) {
        const Read r = read(text);
        EXPECT_EQ(r.error, std::errc::invalid_argument) << '"' << text << '"';
        EXPECT_EQ(r.used, 0) << text;
        EXPECT_EQ(r.value, -7.0) << text << ": the value is left as it was";
    }
    EXPECT_EQ(read("1e400").error, std::errc::result_out_of_range);
}

// The C locale's point, whatever the process's locale says: a comma-decimal
// locale read "1.5" as 1 through strtod.
TEST(CharConvTest, APointIsAPointInEveryLocale) {
    const std::string before = std::setlocale(LC_NUMERIC, nullptr);
    const bool german = std::setlocale(LC_NUMERIC, "de_DE.UTF-8") != nullptr ||
                        std::setlocale(LC_NUMERIC, "de_DE") != nullptr;
    const Read r = read("1.5");
    std::setlocale(LC_NUMERIC, before.c_str());
    if (!german) GTEST_SKIP() << "no German locale here";
    EXPECT_EQ(r.value, 1.5);
    EXPECT_EQ(r.used, 3);
}

#if defined(HZ_PORTABLE_FROM_CHARS) && defined(__cpp_lib_to_chars)
// The portable one reads each of these as the standard one does: how much,
// whether it failed, and the value.
TEST(CharConvTest, ThePortableOneAgreesWithTheStandardOne) {
    const std::vector<std::string> texts = {"0",
                                            "-0",
                                            "1",
                                            "1.5",
                                            "-1.5e-3",
                                            "123456789012345678901234567890",
                                            ".25",
                                            "7.",
                                            "1e10",
                                            "1E-10",
                                            "1e+308",
                                            "1e309",
                                            "2.5e",
                                            "3e+",
                                            "4e-x",
                                            "1.2.3",
                                            "12abc",
                                            "inf",
                                            "-inf",
                                            "INF",
                                            "Infinity",
                                            "nan",
                                            "-nan",
                                            "0x1p3",
                                            "1,5",
                                            "",
                                            "+1",
                                            " 1",
                                            "abc",
                                            "-.5",
                                            "0.000000000000000000000000000000000000001",
                                            "4.9e-324",
                                            "1e-400"};
    for (const std::string& text : texts) {
        double standard = -7.0;
        double portable = -7.0;
        const auto s = std::from_chars(text.data(), text.data() + text.size(), standard);
        const auto p = fromChars(text.data(), text.data() + text.size(), portable);
        EXPECT_EQ(p.ec, s.ec) << '"' << text << '"';
        EXPECT_EQ(p.ptr - text.data(), s.ptr - text.data()) << '"' << text << '"';
        if (std::isnan(standard)) {
            EXPECT_TRUE(std::isnan(portable)) << text;
        } else {
            EXPECT_EQ(portable, standard) << '"' << text << '"';
        }
    }
}
#endif
