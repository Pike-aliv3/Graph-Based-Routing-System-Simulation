# Graph-Based Routing System

A graph routing engine built on adjacency-list representation, supporting shortest-path queries, K-nearest neighbors, dynamic edge updates, and approximate search via landmark-based heuristics.

The full problem statement is available in [CS293_Lab_Project.pdf](CS293_Lab_Project.pdf).

## Project Structure

```
Common/
  Node.hpp          — Graph vertex (id, lat/lon, POIs)
  Edge.hpp          — Weighted directed edge with speed profile
  Graph.hpp/cpp     — Core graph: Dijkstra, KNN, edge removal/modification
  KDTree.hpp        — 2D KD-tree for spatial nearest-neighbor queries
  json.hpp          — nlohmann/json (header-only library)

Phase-1/
  QueryProcessor.hpp/cpp  — Shortest path, KNN, remove/modify edge
  main.cpp                — Phase 1 entry point

Phase-2/
  QueryProcessor.hpp/cpp  — Yen's K-shortest paths, penalized Dijkstra, bidirectional A*
  main.cpp                — Phase 2 entry point

tests/
  graph.json              — 50-node grid test graph
  test_all_phase1.json    — Phase 1 test queries
  queries_phase2.json     — Phase 2 test queries
```

## Build

Requires C++17. Compile with:

```bash
# Phase 1
g++ -std=c++17 -Wall -O2 -static Common/Graph.cpp Phase-1/QueryProcessor.cpp Phase-1/main.cpp -o phase1

# Phase 2
g++ -std=c++17 -Wall -O2 -static Common/Graph.cpp Phase-2/QueryProcessor.cpp Phase-2/main.cpp -o phase2
```

Or use the Makefile:

```bash
make phase1
make phase2
```

## Usage

```bash
./phase1 <graph.json> <queries.json> <output.json>
./phase2 <graph.json> <queries.json> <output.json>
```

Example:

```bash
./phase1 tests/graph.json tests/test_all_phase1.json output_phase1.json
./phase2 tests/graph.json tests/queries_phase2.json output_phase2.json
```

## Phases

### Phase 1

- **Shortest path** — Dijkstra with distance/time modes, forbidden nodes, forbidden road types, and time-dependent speed profiles (15-minute slots)
- **K-nearest neighbors** — Find closest POI nodes by Euclidean distance (KD-tree spatial index) or shortest-path distance
- **Edge removal** — Soft-delete edges from the graph
- **Edge modification** — Update edge properties (length, speed profile, oneway, road type)

### Phase 2

- **Query 1: K Shortest Paths** — Yen's algorithm for exact K-shortest simple paths
- **Query 2: K Shortest Paths (Heuristic)** — Yen's + penalized Dijkstra for diverse path candidates, greedy selection minimizing overlap + distance penalty
- **Query 3: Approximate Shortest Path** — Bidirectional A* with ALT (A*, Landmarks, Triangle inequality) heuristic using 16 landmarks selected by farthest-insertion
