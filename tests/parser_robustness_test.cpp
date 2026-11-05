// The parsers on randomly damaged copies of valid files: whatever the input, the only exception is ParseError (with a
// line inside the file, or one past its end), and anything accepted writes back to a canonical form that parses to
// the same value. Run under ASan and UBSan too (make asan), this is a small fuzzer with a fixed seed.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "pathfinder/generate.hpp"
#include "pathfinder/movingai.hpp"
#include "pathfinder/rng.hpp"

namespace pathfinder {
namespace {

int lines_in(const std::string& text) {
  int n = 0;
  for (const char c : text) {
    n += c == '\n' ? 1 : 0;
  }
  return n + ((text.empty() || text.back() == '\n') ? 0 : 1);
}

// One random edit: overwrite, insert or delete a byte, duplicate or drop a line, or cut the file short. The bytes
// are drawn mostly from characters that mean something in these formats.
std::string damage(const std::string& text, SplitMix64& rng) {
  static const std::string kBytes = std::string(".@TGSOW0123456789 \t\r\n-+exmapversiontypeheightwidth") + '\0' +
                                    static_cast<char>(0xff);
  std::string out = text;
  const auto pick = [&] { return kBytes[rng.below(kBytes.size())]; };
  const auto at = [&](std::size_t n) { return n == 0 ? std::size_t{0} : rng.below(n); };
  const auto kind = rng.below(6);
  switch (kind) {
    case 0:
      if (!out.empty()) {
        out[at(out.size())] = pick();
      }
      break;
    case 1:
      out.insert(out.begin() + static_cast<std::ptrdiff_t>(at(out.size() + 1)), pick());
      break;
    case 2:
      if (!out.empty()) {
        out.erase(at(out.size()), 1);
      }
      break;
    case 3:
    case 4: {
      // Duplicate (3) or drop (4) one whole line.
      std::vector<std::string> lines;
      std::size_t start = 0;
      while (start < out.size()) {
        const auto end = out.find('\n', start);
        const auto stop = end == std::string::npos ? out.size() : end + 1;
        lines.push_back(out.substr(start, stop - start));
        start = stop;
      }
      if (!lines.empty()) {
        const auto i = at(lines.size());
        if (kind == 3) {
          // A copy first: inserting an element of the same vector is legal, but GCC 15 at -O3 reports a false
          // maybe-uninitialized inside libstdc++'s copy of it.
          const std::string line = lines[i];
          lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(i), line);
        } else {
          lines.erase(lines.begin() + static_cast<std::ptrdiff_t>(i));
        }
      }
      out.clear();
      for (const auto& l : lines) {
        out += l;
      }
      break;
    }
    default:
      out.resize(at(out.size() + 1));
      break;
  }
  return out;
}

template <class Parse, class Write>
void hammer(const std::vector<std::string>& seeds, Parse parse, Write write, std::uint64_t seed, int rounds,
            int* accepted) {
  SplitMix64 rng(seed);
  for (int round = 0; round < rounds; ++round) {
    std::string text = seeds[rng.below(seeds.size())];
    const auto edits = 1 + rng.below(3);
    for (std::uint64_t e = 0; e < edits; ++e) {
      text = damage(text, rng);
    }
    try {
      const auto value = parse(text);
      ++*accepted;
      const std::string canonical = write(value);
      ASSERT_EQ(parse(canonical), value) << "round " << round;
      ASSERT_EQ(write(parse(canonical)), canonical) << "round " << round;
    } catch (const ParseError& e) {
      ASSERT_GE(e.line(), 1) << "round " << round;
      ASSERT_LE(e.line(), lines_in(text) + 1) << "round " << round << ": " << e.what();
    }
    // Any other exception escapes and fails the test.
  }
}

TEST(ParserRobustness, DamagedMapsEitherParseAndRoundTripOrFailWithAParseError) {
  const std::vector<std::string> seeds{
      write_map(random_map(9, 7, 25.0, 1)), write_map(warehouse_map(WarehouseLayout{2, 2, 3, 1, 1, 1})),
      "type octile\r\nheight 2\r\nwidth 3\r\nmap\r\n.GS\r\n@OT\r\n",
      "width 2\ntype octile\nheight 1\nmap\n.W\n\n"};
  int accepted = 0;
  hammer(
      seeds, [](const std::string& t) { return parse_map(t); }, [](const Grid& g) { return write_map(g); }, 2026,
      20000, &accepted);
  // Some damage leaves a valid map (a '.' turned into '@', a blank line added at the end): both paths ran.
  EXPECT_GT(accepted, 200);
  EXPECT_LT(accepted, 19000);
}

TEST(ParserRobustness, DamagedScenariosEitherParseAndRoundTripOrFailWithAParseError) {
  const std::vector<std::string> seeds{
      write_scenario(random_scenario(random_map(12, 10, 20.0, 3), 6, 9, "r.map")),
      "version 1.0\r\n3\tm.map\t40\t30\t1\t2\t39\t29\t42.5\r\n0 m.map 40 30 0 0 0 1 1\r\n\r\n",
      "version 1\n"};
  int accepted = 0;
  hammer(
      seeds, [](const std::string& t) { return parse_scenario(t); },
      [](const Scenario& s) { return write_scenario(s); }, 7, 20000, &accepted);
  EXPECT_GT(accepted, 1000);
  EXPECT_LT(accepted, 19000);
}

}  // namespace
}  // namespace pathfinder
