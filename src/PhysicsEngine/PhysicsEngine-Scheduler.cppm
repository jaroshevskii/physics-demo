export module PhysicsEngine:Scheduler;

import std;
import :World;

// An explicit, ordered system registry mirroring RealityKit's `System`
// scheduling: each system declares `.before` / `.after` dependencies on others,
// and the registry topologically sorts them (Kahn's algorithm, registration
// order breaking ties) into a fixed run order. This makes the pipeline
// data-driven and reorderable instead of a hard-coded call sequence — and it is
// the documented extension point for demos that want to inject their own systems
// (e.g. a vehicle-controller system that runs before the solver).
//
// A cycle is a hard error (throws at `finalize`) — a silently dropped edge would
// make execution order nondeterministic, so we fail loud.
export namespace Physics {

/// A unit of per-step work — the engine's analog of a RealityKit `System`.
using SystemFn = std::function<void(World &, const StepContext &)>;
/// Handle to a registered system, used to declare `before`/`after` ordering.
using SystemId = std::size_t;

/// An explicit, ordered registry of systems, mirroring RealityKit's `System`
/// scheduling: each system declares `.before`/`.after` dependencies and the
/// registry topologically sorts them into a fixed run order. This is the
/// documented extension point for demos that want to inject their own systems.
class SystemRegistry {
public:
  /// Registers a system and returns its `SystemId`. `before`/`after` list the
  /// systems this one must run before or after. Registration order breaks ties.
  SystemId add(std::string name, SystemFn fn, std::vector<SystemId> before = {},
               std::vector<SystemId> after = {}) {
    const SystemId id = entries_.size();
    entries_.push_back({std::move(name), std::move(fn), std::move(before), std::move(after)});
    return id;
  }

  /// Topologically sorts the registered systems into a run order (Kahn's
  /// algorithm, lowest-index ready node first for deterministic ties). Edge
  /// X->Y means X runs before Y. Throws if the dependencies form a cycle.
  void finalize() {
    const std::size_t n = entries_.size();
    std::vector<std::vector<SystemId>> edges(n);
    std::vector<int> indegree(n, 0);
    auto addEdge = [&](SystemId from, SystemId to) {
      edges[from].push_back(to);
      ++indegree[to];
    };
    for (SystemId x = 0; x < n; ++x) {
      for (SystemId y : entries_[x].before) {
        addEdge(x, y);
      }
      for (SystemId y : entries_[x].after) {
        addEdge(y, x);
      }
    }
    // Kahn, always taking the lowest-index ready node => deterministic ties.
    order_.clear();
    std::vector<int> deg = indegree;
    while (order_.size() < n) {
      SystemId next = n;
      for (SystemId i = 0; i < n; ++i) {
        if (deg[i] == 0) {
          next = i;
          break;
        }
      }
      if (next == n) {
        throw std::runtime_error("SystemRegistry: dependency cycle");
      }
      order_.push_back(next);
      deg[next] = -1;
      for (SystemId t : edges[next]) {
        --deg[t];
      }
    }
  }

  /// Runs every system once, in the finalized order. Call `finalize` first.
  void run(World &w, const StepContext &context) const {
    for (SystemId id : order_) {
      entries_[id].fn(w, context);
    }
  }

  /// The finalized run order, as `SystemId`s.
  const std::vector<SystemId> &order() const { return order_; }
  /// The registered name of a system, for diagnostics.
  const std::string &name(SystemId id) const { return entries_[id].name; }

private:
  struct Entry {
    std::string name;
    SystemFn fn;
    std::vector<SystemId> before;
    std::vector<SystemId> after;
  };
  std::vector<Entry> entries_;
  std::vector<SystemId> order_;
};

} // namespace Physics
