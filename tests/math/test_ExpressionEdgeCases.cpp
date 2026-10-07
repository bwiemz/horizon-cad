#include <gtest/gtest.h>

#if defined(__unix__) || defined(__APPLE__)
#include <pthread.h>
#endif

#include <clocale>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

#include "horizon/math/Expression.h"

#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define HZ_SANITIZED 1
#elif defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer)
#define HZ_SANITIZED 1
#endif
#endif
#ifndef HZ_SANITIZED
#define HZ_SANITIZED 0
#endif

using namespace hz::math;

TEST(ExpressionEdgeCaseTest, DivisionByZero) {
    auto expr = Expression::parse("1 / 0");
    ASSERT_NE(expr, nullptr);
    double result = expr->evaluate({});
    EXPECT_TRUE(std::isinf(result));
}

TEST(ExpressionEdgeCaseTest, NestedFunctions) {
    auto expr = Expression::parse("sin(cos(0))");
    ASSERT_NE(expr, nullptr);
    // cos(0) = 1, sin(1) ~ 0.8414709848
    EXPECT_NEAR(expr->evaluate({}), std::sin(1.0), 1e-10);
}

TEST(ExpressionEdgeCaseTest, DeepNesting) {
    auto expr = Expression::parse("((((1 + 2) + 3) + 4) + 5)");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 15.0);
}

TEST(ExpressionEdgeCaseTest, WhitespaceHandling) {
    auto expr = Expression::parse("  2  +  3  ");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 5.0);
}

TEST(ExpressionEdgeCaseTest, MultipleOperators) {
    // 2 + 3 - 1 * 4 / 2 = 2 + 3 - 2 = 3
    auto expr = Expression::parse("2 + 3 - 1 * 4 / 2");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 3.0);
}

TEST(ExpressionEdgeCaseTest, PowerRightAssociativity) {
    // 2 ^ 3 ^ 2 = 2 ^ 9 = 512 (right-associative)
    auto expr = Expression::parse("2 ^ 3 ^ 2");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 512.0);
}

TEST(ExpressionEdgeCaseTest, MultiArgFunction) {
    // atan2(0, -1) = pi
    auto expr = Expression::parse("atan2(0, -1)");
    ASSERT_NE(expr, nullptr);
    EXPECT_NEAR(expr->evaluate({}), 3.14159265358979, 1e-10);
}

TEST(ExpressionEdgeCaseTest, TrailingWhitespace) {
    auto expr = Expression::parse("42   ");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 42.0);
}

TEST(ExpressionEdgeCaseTest, ConsecutiveUnaryMinus) {
    // --5 should parse as -(-5) = 5
    auto expr = Expression::parse("--5");
    if (expr) {
        EXPECT_DOUBLE_EQ(expr->evaluate({}), 5.0);
    }
    // It's also acceptable for the parser to reject this
}

TEST(ExpressionEdgeCaseTest, ZeroPower) {
    auto expr = Expression::parse("5 ^ 0");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 1.0);
}

// ---------------------------------------------------------------------------
// Hostile input: expressions arrive from files, so size and depth are bounded
// ---------------------------------------------------------------------------

namespace {

std::string nestedParens(int depth) {
    return std::string(static_cast<size_t>(depth), '(') + "1" +
           std::string(static_cast<size_t>(depth), ')');
}

std::string flatSum(int terms) {
    std::string s = "1";
    for (int i = 1; i < terms; ++i) s += "+1";
    return s;
}

}  // namespace

TEST(ExpressionLimitsTest, NestingUpToTheLimitParses) {
    // Each bracket is one level of nesting.
    auto e = hz::math::Expression::parse(nestedParens(hz::math::Expression::kMaxNestingDepth));
    ASSERT_NE(e, nullptr);
    EXPECT_DOUBLE_EQ(e->evaluate({}), 1.0);
}

TEST(ExpressionLimitsTest, NestingPastTheLimitIsRejected) {
    EXPECT_EQ(hz::math::Expression::parse(nestedParens(hz::math::Expression::kMaxNestingDepth + 1)),
              nullptr);
}

TEST(ExpressionLimitsTest, AStackOverflowOfMinusSignsIsRejected) {
    // 200 000 nested negations used to recurse once each and overflow the stack.
    EXPECT_EQ(hz::math::Expression::parse(std::string(200000, '-') + "1"), nullptr);
}

TEST(ExpressionLimitsTest, ALongPowerTowerIsRejected) {
    std::string tower = "2";
    for (int i = 0; i < 10000; ++i) tower += "^1";
    EXPECT_EQ(hz::math::Expression::parse(tower), nullptr);
}

TEST(ExpressionLimitsTest, ALongFlatChainIsRejected) {
    // Parsed in a loop, but it builds a tree as deep as it is long, which
    // evaluation and destruction would recurse through.
    EXPECT_EQ(hz::math::Expression::parse(flatSum(200000)), nullptr);
}

TEST(ExpressionLimitsTest, AReasonableChainStillParses) {
    auto e = hz::math::Expression::parse(flatSum(400));  // 799 nodes
    ASSERT_NE(e, nullptr);
    EXPECT_DOUBLE_EQ(e->evaluate({}), 400.0);
}

namespace {

/// Runs @p work on a thread whose stack is @p bytes; whether it could.
bool onStackOf(std::size_t bytes, void (*work)(void*), void* arg) {
#if defined(__unix__) || defined(__APPLE__)
    pthread_attr_t attr;
    if (pthread_attr_init(&attr) != 0) return false;
    const bool sized = pthread_attr_setstacksize(&attr, bytes) == 0;
    pthread_t thread;
    struct Call {
        void (*work)(void*);
        void* arg;
    } call{work, arg};
    const bool started = sized && pthread_create(
                                      &thread, &attr,
                                      [](void* p) -> void* {
                                          auto* c = static_cast<Call*>(p);
                                          c->work(c->arg);
                                          return nullptr;
                                      },
                                      &call) == 0;
    pthread_attr_destroy(&attr);
    if (started) pthread_join(thread, nullptr);
    return started;
#else
    (void)bytes;
    (void)work;
    (void)arg;
    return false;
#endif
}

/// A unary minus nested @p levels deep round a literal, nested by moving, not
/// copying: copying a json value recurses once per level.
nlohmann::json negatedDeep(int levels) {
    nlohmann::json deep = {{"type", "literal"}, {"value", 1.0}};
    for (int i = 0; i < levels; ++i) {
        nlohmann::json wrapper = {{"type", "unary"}, {"op", "-"}};
        wrapper["child"] = std::move(deep);
        deep = std::move(wrapper);
    }
    return deep;
}

}  // namespace

// A tree nested deeper than parse() builds is refused, and read without
// recursing: once per level reached Windows' 1 MB stack on a debug build
// before the count of nodes stopped it. Here it is read on a 256 KB stack.
TEST(ExpressionLimitsTest, ADeepTreeIsReadOnASmallStack) {
    struct Case {
        nlohmann::json deep = negatedDeep(5000);
        nlohmann::json chain;
        bool deepRefused = false;
        bool chainRead = false;
    } c;
    // As deep as parse() lets a chain grow: 1+1+...+1, 512 terms.
    std::string sum = "1";
    for (int i = 1; i < 512; ++i) sum += "+1";
    const auto parsed = Expression::parse(sum);
    ASSERT_NE(parsed, nullptr);
    c.chain = parsed->toJson();
    // Sanitizers' frames are several times larger.
    const std::size_t stack = HZ_SANITIZED ? std::size_t{1} << 20 : std::size_t{256} << 10;
    const bool ran = onStackOf(
        stack,
        [](void* p) {
            auto& c = *static_cast<Case*>(p);
            c.deepRefused = Expression::fromJson(c.deep) == nullptr;
            auto chain = Expression::fromJson(c.chain);
            c.chainRead = chain != nullptr;
            // Released here, as deep as it was read: its own recursion, as
            // parse()'s tree's, is what the stack is sized for.
        },
        &c);
    if (!ran) GTEST_SKIP() << "a thread's stack size is set here with POSIX threads";
    EXPECT_TRUE(c.deepRefused);
    EXPECT_TRUE(c.chainRead);
}

TEST(ExpressionLimitsTest, FromJsonRejectsDeepTreesAndWrongTypes) {
    // Nested by moving, not copying: copying a json value recurses once per
    // level, which on Windows' 1 MB stack overflows long before 5000.
    nlohmann::json deep = {{"type", "literal"}, {"value", 1.0}};
    for (int i = 0; i < 5000; ++i) {
        nlohmann::json wrapper = {{"type", "unary"}, {"op", "-"}};
        wrapper["child"] = std::move(deep);
        deep = std::move(wrapper);
    }
    EXPECT_EQ(hz::math::Expression::fromJson(deep), nullptr);

    // A number where the operator string belongs made json::get throw.
    const nlohmann::json wrongType = {
        {"type", "unary"}, {"op", 7}, {"child", {{"type", "literal"}, {"value", 1.0}}}};
    std::unique_ptr<hz::math::Expression> parsed;
    ASSERT_NO_THROW(parsed = hz::math::Expression::fromJson(wrongType));
    EXPECT_EQ(parsed, nullptr);

    // A normal tree still round-trips.
    auto e = hz::math::Expression::parse("2*(x+3)^2");
    ASSERT_NE(e, nullptr);
    auto back = hz::math::Expression::fromJson(e->toJson());
    ASSERT_NE(back, nullptr);
    EXPECT_DOUBLE_EQ(back->evaluate({{"x", 1.0}}), 32.0);
}

// Numbers are read in the C locale, whatever the user's: std::stod followed
// the one Qt sets from the environment, and under de_DE read "1.5" as 1.
TEST(ExpressionEdgeCaseTest, ADecimalPointIsReadInEveryLocale) {
    const std::string before = std::setlocale(LC_ALL, nullptr);
    const bool german = std::setlocale(LC_ALL, "de_DE.UTF-8") != nullptr ||
                        std::setlocale(LC_ALL, "de_DE") != nullptr;
    const auto e = hz::math::Expression::parse("x * 2.75 + 0.5");
    const std::string printed = e ? e->toString() : std::string();
    std::setlocale(LC_ALL, before.c_str());
    if (!german) GTEST_SKIP() << "no German locale here";
    ASSERT_NE(e, nullptr);
    EXPECT_DOUBLE_EQ(e->evaluate({{"x", 2.0}}), 6.0);
    EXPECT_EQ(printed, "((x * 2.75) + 0.5)");
}

// Printed as the tokenizer reads numbers: plainly, exactly, and never with an
// exponent, which it does not read.
TEST(ExpressionEdgeCaseTest, ALiteralIsPrintedSoItReadsBackExactly) {
    for (const double value : {0.1, 0.00005, 1.0 / 3.0, 12.0, 1e21, 2.5e-9, 123456.789}) {
        const hz::math::LiteralExpr literal(value);
        const std::string text = literal.toString();
        EXPECT_EQ(text.find_first_of("eE"), std::string::npos) << text;
        const auto again = hz::math::Expression::parse(text);
        ASSERT_NE(again, nullptr) << text;
        EXPECT_EQ(again->evaluate({}), value) << text;
    }
    EXPECT_EQ(hz::math::LiteralExpr(0.1).toString(), "0.1");
    EXPECT_EQ(hz::math::LiteralExpr(-2.5).toString(), "(-2.5)");
    const auto infinite = hz::math::Expression::parse(
        hz::math::LiteralExpr(std::numeric_limits<double>::infinity()).toString());
    ASSERT_NE(infinite, nullptr);
    EXPECT_TRUE(std::isinf(infinite->evaluate({})));
}

// A chain of one precedence is printed as one bracket, so a long one reads
// back; and a different precedence, or a power, keeps its own brackets.
TEST(ExpressionEdgeCaseTest, AChainIsPrintedAsOneBracket) {
    const auto chain = hz::math::Expression::parse("a + b - c + d");
    ASSERT_NE(chain, nullptr);
    EXPECT_EQ(chain->toString(), "(a + b - c + d)");
    const auto mixed = hz::math::Expression::parse("a * b + c / d * e - f ^ g ^ h");
    ASSERT_NE(mixed, nullptr);
    EXPECT_EQ(mixed->toString(), "((a * b) + (c / d * e) - (f ^ (g ^ h)))");
    const auto right = hz::math::Expression::parse("a - (b - c)");
    ASSERT_NE(right, nullptr);
    EXPECT_EQ(right->toString(), "(a - (b - c))");

    const auto longChain = hz::math::Expression::parse(flatSum(400));
    ASSERT_NE(longChain, nullptr);
    const auto again = hz::math::Expression::parse(longChain->toString());
    ASSERT_NE(again, nullptr);
    EXPECT_DOUBLE_EQ(again->evaluate({}), 400.0);
}
