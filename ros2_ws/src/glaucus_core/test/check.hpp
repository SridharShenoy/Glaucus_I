// Minimal test helpers so glaucus_core tests build with nothing but a compiler.
// (Swap for GoogleTest via ament_cmake_gtest later if you want - not required.)
#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>

inline int g_failures = 0;

#define CHECK(cond)                                                     \
  do {                                                                  \
    if (!(cond)) {                                                      \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                                     \
    }                                                                   \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                           \
  do {                                                                  \
    const double _a = (a), _b = (b);                                    \
    if (!(std::abs(_a - _b) <= (tol))) {                                \
      std::fprintf(stderr, "%s:%d: CHECK_NEAR failed: %s=%.9g vs %s=%.9g\n", \
                   __FILE__, __LINE__, #a, _a, #b, _b);                 \
      ++g_failures;                                                     \
    }                                                                   \
  } while (0)

#define TEST_MAIN_END()                                                 \
  do {                                                                  \
    if (g_failures) { std::fprintf(stderr, "%d failure(s)\n", g_failures); return 1; } \
    std::printf("all checks passed\n");                                 \
    return 0;                                                           \
  } while (0)
