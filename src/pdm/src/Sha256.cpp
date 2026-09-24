#include "horizon/pdm/Sha256.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace hz::pdm {

namespace {

using Block = const unsigned char*;
using State = std::array<std::uint32_t, 8>;

// The first 32 bits of the fractional parts of the cube roots of the first
// 64 primes (FIPS 180-4, 4.2.2).
constexpr std::array<std::uint32_t, 64> kRound = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

// The first 32 bits of the fractional parts of the square roots of the first
// 8 primes (FIPS 180-4, 5.3.3).
constexpr State kInitial = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                            0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};

constexpr std::uint32_t rotr(std::uint32_t x, unsigned n) {
    return (x >> n) | (x << (32u - n));
}

/// Fold one 64-byte block into @p h.
void compress(State& h, Block block) {
    std::array<std::uint32_t, 64> w{};
    for (std::size_t i = 0; i < 16; ++i) {
        w[i] = (std::uint32_t{block[4 * i]} << 24) | (std::uint32_t{block[4 * i + 1]} << 16) |
               (std::uint32_t{block[4 * i + 2]} << 8) | std::uint32_t{block[4 * i + 3]};
    }
    for (std::size_t i = 16; i < 64; ++i) {
        const std::uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
        const std::uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }

    State v = h;  // a..h
    for (std::size_t i = 0; i < 64; ++i) {
        const std::uint32_t e = v[4];
        const std::uint32_t a = v[0];
        const std::uint32_t sum1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
        const std::uint32_t choose = (e & v[5]) ^ (~e & v[6]);
        const std::uint32_t t1 = v[7] + sum1 + choose + kRound[i] + w[i];
        const std::uint32_t sum0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
        const std::uint32_t majority = (a & v[1]) ^ (a & v[2]) ^ (v[1] & v[2]);
        const std::uint32_t t2 = sum0 + majority;
        v[7] = v[6];
        v[6] = v[5];
        v[5] = v[4];
        v[4] = v[3] + t1;
        v[3] = v[2];
        v[2] = v[1];
        v[1] = v[0];
        v[0] = t1 + t2;
    }
    for (std::size_t i = 0; i < 8; ++i) h[i] += v[i];
}

}  // namespace

std::string sha256Hex(std::string_view data) {
    State h = kInitial;
    const auto* bytes = reinterpret_cast<const unsigned char*>(data.data());
    const std::size_t whole = data.size() / 64;
    for (std::size_t i = 0; i < whole; ++i) compress(h, bytes + 64 * i);

    // The padding: what is left, a 1 bit, zeros, and the length in bits as a
    // big-endian 64-bit number, filling one block or, when the length does
    // not fit after what is left, two.
    std::array<unsigned char, 128> tail{};
    const std::size_t rest = data.size() - 64 * whole;
    if (rest > 0) std::memcpy(tail.data(), bytes + 64 * whole, rest);
    tail[rest] = 0x80;
    const std::size_t tailSize = rest + 1 + 8 <= 64 ? 64 : 128;
    const std::uint64_t bits = static_cast<std::uint64_t>(data.size()) * 8u;
    for (std::size_t i = 0; i < 8; ++i) {
        tail[tailSize - 1 - i] = static_cast<unsigned char>(bits >> (8 * i));
    }
    compress(h, tail.data());
    if (tailSize == 128) compress(h, tail.data() + 64);

    static constexpr char kHex[] = "0123456789abcdef";
    std::string out(64, '0');
    for (std::size_t i = 0; i < 8; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            const auto byte = static_cast<unsigned>((h[i] >> (24 - 8 * j)) & 0xffu);
            out[8 * i + 2 * j] = kHex[byte >> 4];
            out[8 * i + 2 * j + 1] = kHex[byte & 0xfu];
        }
    }
    return out;
}

}  // namespace hz::pdm
