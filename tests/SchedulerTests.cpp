// Tests for the RealityKit-style ordered system registry: before/after
// dependencies produce the right topological run order, ties break by
// registration order, and a cycle is a hard error.

import std;
import PhysicsEngine;

using namespace Physics;

namespace {

int failures = 0;
void expect(bool cond, std::string_view msg) {
  if (!cond) {
    ++failures;
    std::println(std::cerr, "FAIL: {}", msg);
  }
}

void testDependencyOrdering() {
  std::vector<std::string> log;
  SystemRegistry reg;
  auto make = [&log](std::string name) {
    return [name, &log](World &, const StepContext &) { log.push_back(name); };
  };
  // Register out of order; declare solve AFTER integrate, render AFTER solve.
  const auto render = reg.add("render", make("render"));
  const auto solve = reg.add("solve", make("solve"), {render});            // solve before render
  const auto integrate = reg.add("integrate", make("integrate"), {solve}); // integrate before solve
  (void)integrate;
  reg.finalize();

  World w;
  StepContext context;
  reg.run(w, context);
  const std::vector<std::string> expected{"integrate", "solve", "render"};
  expect(log == expected, "systems run in integrate -> solve -> render order");
}

void testCycleThrows() {
  SystemRegistry reg;
  auto noop = [](World &, const StepContext &) {};
  const auto a = reg.add("a", noop);
  const auto b = reg.add("b", noop, {a}); // a before b
  reg.add("c", noop, {b}, {b});           // c before b AND after b => cycle with b
  bool threw = false;
  try {
    reg.finalize();
  } catch (const std::exception &) {
    threw = true;
  }
  expect(threw, "a dependency cycle throws at finalize");
}

} // namespace

int main() {
  testDependencyOrdering();
  testCycleThrows();
  if (failures == 0) {
    std::println("All SchedulerTests passed.");
    return 0;
  }
  std::println(std::cerr, "{} SchedulerTests failed.", failures);
  return 1;
}
