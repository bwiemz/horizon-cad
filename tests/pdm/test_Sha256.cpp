#include <gtest/gtest.h>

#include <string>

#include "horizon/pdm/Sha256.h"

using hz::pdm::sha256Hex;

// The FIPS 180-4 example messages (NIST's SHA-256 examples), which between
// them pad into one block, into two, and span many.
TEST(Sha256Test, MatchesTheStandardExamples) {
    EXPECT_EQ(sha256Hex(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    EXPECT_EQ(sha256Hex("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    // 56 bytes: the length no longer fits in the first padding block.
    EXPECT_EQ(sha256Hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
              "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
    // 112 bytes.
    EXPECT_EQ(sha256Hex("abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmn"
                        "hijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu"),
              "cf5b16a778af8380036ce59e7b0492370b249b11e8f07a51afac45037afee9d1");
    EXPECT_EQ(sha256Hex(std::string(1000000, 'a')),
              "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

// The lengths either side of each padding boundary: 55 bytes is the most
// whose length fits in their last block, 64 a block exactly, 119 and 120 the
// same boundary a block later. Expected digests are from Python's hashlib.
TEST(Sha256Test, PadsEveryLengthAroundABlockBoundary) {
    const auto pattern = [](std::size_t n) {
        std::string s;
        for (std::size_t i = 0; i < n; ++i) s.push_back(static_cast<char>((i * 7 + 3) % 256));
        return s;
    };
    const struct {
        std::size_t length;
        const char* digest;
    } cases[] = {
        {55, "e7313d333c272e639f790978283f9eb392e843d0f29b7016828bb1daa4aac70b"},
        {56, "4324d65f3c103567f5589c710bc08f8523f929a9272e3af36fc968e52abc6c27"},
        {63, "81c80242132f230c3bd41b3e63bbcff16107339549214a99614ff26664625055"},
        {64, "39e3d7b6b5d075d37d053ad89b24b41bef4f3c29760c84447cab3f3be1882241"},
        {65, "aacca6ff74fdbb296d165a45cecfa04e5127bc008770fbbdd48006f2d2fae95e"},
        {119, "9ce7368e4daf32341631b492e80359dc9f594b48453cd0dd5bf0b19279cc177e"},
        {120, "7836b787757e95e58b3ca5aec90b1b004e8deba1e50e9675af9cabf1a13a04b5"},
        {128, "d2742f1f4ac6bb7ca2b239ee18402ba8b3f9f8e652d2a72973c2b9ba11c08cf6"},
    };
    for (const auto& c : cases) {
        EXPECT_EQ(sha256Hex(pattern(c.length)), c.digest) << c.length << " bytes";
    }
}

// Every byte value goes through as data, including NUL.
TEST(Sha256Test, HashesBinaryData) {
    std::string bytes;
    for (int b = 0; b < 256; ++b) bytes.push_back(static_cast<char>(b));
    EXPECT_EQ(sha256Hex(bytes), "40aff2e9d2d8922e47afd4648e6967497158785fbd1da870e7110266bf944880");
    EXPECT_NE(sha256Hex(std::string("a\0b", 3)), sha256Hex(std::string("a\0c", 3)));
}
