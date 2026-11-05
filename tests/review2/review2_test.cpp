// The day-1 second-round review (an agent at commit 8b29b30, in its own worktree): each test here failed on that
// commit and passes with the fix that names it.

#include <gtest/gtest.h>

#include <string>

#include "pathfinder/grid.hpp"
#include "pathfinder/movingai.hpp"
#include "pathfinder/plan.hpp"

namespace pathfinder {
namespace {

// src/plan_file.cpp kMaxAgents: parse_plan refuses "agents k" for k > 1,000,000 ("more agents than any map here could
// hold"), but a grid may be 10,000 x 10,000, and write_plan has no such limit. include/pathfinder/plan.hpp promises
// write_plan throws only for an empty path or an off-grid id, and parse_plan(write_plan(p, g), g) == p; here a plan
// of 1,000,001 one-cell paths on a 1001 x 1000 empty grid (each agent on its own cell: a valid plan of a valid
// instance) is written without complaint and then refused on line 2, so `pathfinder validate` can't read it either.
TEST(Review2PlanFile, WhatWritePlanWritesParsePlanReadsBackAboveAMillionAgents) {
  const Grid g(1001, 1000);
  constexpr std::size_t kAgents = 1'000'001;
  ASSERT_LE(kAgents, g.cell_count());
  Plan plan(kAgents);
  for (std::size_t i = 0; i < kAgents; ++i) {
    plan[i] = Path{static_cast<CellId>(i)};
  }
  std::string text;
  ASSERT_NO_THROW(text = write_plan(plan, g));
  try {
    EXPECT_TRUE(parse_plan(text, g) == plan);
  } catch (const ParseError& e) {
    FAIL() << "write_plan's output didn't parse: " << e.what();
  }
}

}  // namespace
}  // namespace pathfinder
