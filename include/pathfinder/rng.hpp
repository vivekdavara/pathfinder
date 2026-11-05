#pragma once

#include <cstdint>

namespace pathfinder {

// SplitMix64 (Steele, Lea and Flood, "Fast splittable pseudorandom number generators", OOPSLA 2014; the constants of
// Vigna's reference splitmix64.c). Used instead of <random>'s distributions, whose output differs between libstdc++
// and libc++, so that a seed gives the same map and scenario on every platform.
class SplitMix64 {
 public:
  explicit SplitMix64(std::uint64_t seed) : state_(seed) {}

  std::uint64_t next() {
    std::uint64_t z = (state_ += 0x9e3779b97f4a7c15ULL);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
  }

  // Uniform on [0, n). A plain next() % n would favour small values whenever n doesn't divide 2^64, so draws below
  // 2^64 mod n are rejected: what's left is a whole number of copies of [0, n). Precondition: n > 0.
  std::uint64_t below(std::uint64_t n) {
    const std::uint64_t threshold = (0 - n) % n;  // 2^64 mod n, in unsigned arithmetic
    while (true) {
      const std::uint64_t x = next();
      if (x >= threshold) {
        return x % n;
      }
    }
  }

 private:
  std::uint64_t state_;
};

}  // namespace pathfinder
