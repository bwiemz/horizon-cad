#pragma once

// Time limits in tests. A limit holds in an optimised build, and a Debug
// build is given about an order of magnitude more. Under a sanitizer, which
// runs code 3-15 times slower, and slower again with CI running four tests
// at once, a limit means nothing: HZ_TIME_LIMITS is 0 there, and a test
// checks what it computed but not how long it took.

#if defined(__SANITIZE_THREAD__) || defined(__SANITIZE_ADDRESS__)
#define HZ_TIME_LIMITS 0
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer) || __has_feature(address_sanitizer)
#define HZ_TIME_LIMITS 0
#endif
#endif
#ifndef HZ_TIME_LIMITS
#define HZ_TIME_LIMITS 1
#endif

/// For a test that measures nothing but time.
#define HZ_SKIP_WITHOUT_TIME_LIMITS()                                                \
    if (!HZ_TIME_LIMITS)                                                             \
    GTEST_SKIP() << "a time limit means nothing under a sanitizer, which runs code " \
                    "3-15x slower"
