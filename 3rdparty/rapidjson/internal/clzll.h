
#ifndef RAPIDJSON_CLZLL_H_
#define RAPIDJSON_CLZLL_H_

#include "../rapidjson.h"

#if defined(_MSC_VER)
#include <intrin.h>
#if defined(_WIN64)
#pragma intrinsic(_BitScanReverse64)
#else
#pragma intrinsic(_BitScanReverse)
#endif
#endif

RAPIDJSON_NAMESPACE_BEGIN
namespace internal {

#if (defined(__GNUC__) && __GNUC__ >= 4) || \
    RAPIDJSON_HAS_BUILTIN(__builtin_clzll)
#define RAPIDJSON_CLZLL __builtin_clzll
#else

inline uint32_t clzll(uint64_t x) {
  // Passing 0 to __builtin_clzll is UB in GCC and results in an
  // infinite loop in the software implementation.
  RAPIDJSON_ASSERT(x != 0);

#if defined(_MSC_VER)
  unsigned long r = 0;
#if defined(_WIN64)
  _BitScanReverse64(&r, x);
#else
  // Scan the high 32 bits.
  if (_BitScanReverse(&r, static_cast<uint32_t>(x >> 32))) return 63 - (r + 32);

  // Scan the low 32 bits.
  _BitScanReverse(&r, static_cast<uint32_t>(x & 0xFFFFFFFF));
#endif  // _WIN64

  return 63 - r;
#else
  uint32_t r;
  while (!(x & (static_cast<uint64_t>(1) << 63))) {
    x <<= 1;
    ++r;
  }

  return r;
#endif  // _MSC_VER
}

#define RAPIDJSON_CLZLL RAPIDJSON_NAMESPACE::internal::clzll
#endif  // (defined(__GNUC__) && __GNUC__ >= 4) ||
        // RAPIDJSON_HAS_BUILTIN(__builtin_clzll)

}  // namespace internal
RAPIDJSON_NAMESPACE_END

#endif  // RAPIDJSON_CLZLL_H_