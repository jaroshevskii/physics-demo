export module PhysicsEngine:Threading;

import std;

// A tiny data-parallel helper for the one embarrassingly-parallel stage of the
// pipeline: narrowphase (each candidate pair's contact manifold is a pure
// function of two bodies, independent of every other pair).
//
// Determinism rule: a worker for index `i` writes ONLY `results[i]` — never a
// shared accumulator — and the caller consumes results in ascending `i`. So the
// output is bit-identical regardless of `threadCount` (there's a test for it).
// With `threadCount <= 1` (the default, and what every test pins) it runs inline
// with no threads at all.
export namespace Physics {

/// Invokes `fn(i)` for every `i` in `[0, count)`, splitting the range across
/// `threadCount` workers. Each `fn(i)` must be independent and write only its own
/// slot, so the result is bit-identical for any `threadCount` (a determinism
/// invariant the tests pin). Runs inline with no threads when `threadCount <= 1`
/// (the default) or when `count` is too small to be worth splitting.
inline void parallelFor(std::size_t count, unsigned threadCount,
                        const std::function<void(std::size_t)> &fn) {
  if (threadCount <= 1 || count < 2u * threadCount) {
    for (std::size_t i = 0; i < count; ++i) {
      fn(i);
    }
    return;
  }
  const std::size_t chunk = (count + threadCount - 1) / threadCount;
  std::vector<std::jthread> workers;
  workers.reserve(threadCount - 1);
  for (unsigned t = 1; t < threadCount; ++t) {
    const std::size_t begin = static_cast<std::size_t>(t) * chunk;
    if (begin >= count) {
      break;
    }
    const std::size_t end = std::min(count, begin + chunk);
    workers.emplace_back([&fn, begin, end] {
      for (std::size_t i = begin; i < end; ++i) {
        fn(i);
      }
    });
  }
  // The calling thread handles the first chunk; the jthreads join on scope exit.
  for (std::size_t i = 0; i < std::min(count, chunk); ++i) {
    fn(i);
  }
}

} // namespace Physics
