#pragma once

// Line and field helpers shared by the text formats' parsers (movingai.cpp, plan_file.cpp). Internal: not installed,
// not part of the API.

#include <array>
#include <charconv>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace pathfinder::text {

// Splits text into lines on '\n', dropping one '\r' before it, and counts them from 1. A final line without a
// '\n' is a line; the empty string after a final '\n' isn't.
class Lines {
 public:
  explicit Lines(std::string_view text) : text_(text) {}

  bool next(std::string_view& out) {
    if (pos_ >= text_.size()) {
      return false;
    }
    const auto end = text_.find('\n', pos_);
    std::string_view line;
    if (end == std::string_view::npos) {
      line = text_.substr(pos_);
      pos_ = text_.size();
    } else {
      line = text_.substr(pos_, end - pos_);
      pos_ = end + 1;
    }
    if (!line.empty() && line.back() == '\r') {
      line.remove_suffix(1);
    }
    ++number_;
    out = line;
    return true;
  }

  // The number of the line next() returned last (0 before the first).
  int number() const { return number_; }

 private:
  std::string_view text_;
  std::size_t pos_ = 0;
  int number_ = 0;
};

inline bool is_space(char c) { return c == ' ' || c == '\t'; }

inline bool blank(std::string_view line) {
  for (const char c : line) {
    if (!is_space(c)) {
      return false;
    }
  }
  return true;
}

// The line's fields, separated by runs of spaces and tabs.
inline std::vector<std::string_view> fields(std::string_view line) {
  std::vector<std::string_view> out;
  std::size_t i = 0;
  while (i < line.size()) {
    while (i < line.size() && is_space(line[i])) {
      ++i;
    }
    const std::size_t start = i;
    while (i < line.size() && !is_space(line[i])) {
      ++i;
    }
    if (i > start) {
      out.push_back(line.substr(start, i - start));
    }
  }
  return out;
}

// Text for an error message: at most 40 characters, anything unprintable as \xNN.
inline std::string shown(std::string_view s) {
  static constexpr std::size_t kMax = 40;
  std::string out;
  for (std::size_t i = 0; i < s.size() && i < kMax; ++i) {
    const auto c = static_cast<unsigned char>(s[i]);
    if (c >= 0x20 && c < 0x7f) {
      out += static_cast<char>(c);
    } else {
      static constexpr std::array<char, 17> kHex{"0123456789abcdef"};
      out += "\\x";
      out += kHex[c >> 4];
      out += kHex[c & 0xf];
    }
  }
  if (s.size() > kMax) {
    out += "...";
  }
  return out;
}

// A decimal integer written with digits only (no sign, no spaces), if it is one and fits in 18 digits.
inline std::optional<std::int64_t> digits(std::string_view s) {
  if (s.empty() || s.size() > 18) {
    return std::nullopt;
  }
  for (const char c : s) {
    if (c < '0' || c > '9') {
      return std::nullopt;
    }
  }
  std::int64_t value = 0;
  const auto result = std::from_chars(s.data(), s.data() + s.size(), value);
  if (result.ec != std::errc{}) {
    return std::nullopt;
  }
  return value;
}

}  // namespace pathfinder::text
