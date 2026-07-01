// Phase 2 query processor — Yen's algorithm, penalized Dijkstra, ALT search.
#include "QueryProcessor.hpp"
#include <chrono>
#include <queue>
#include <algorithm>
#include <limits>
#include <cmath>
#include <iostream>
#include <functional>

using json = nlohmann::json;
using pdi  = std::pair<double, int>;
static constexpr double INF = std::numeric_limits<double>::infinity();

QueryProcessor::QueryProcessor(Graph& g) : graph(g) {}

// ════════════════════════════════════════════════════════════════════════════
//  Preprocessing — Landmark selection & distance computation
// ════════════════════════════════════════════════════════════════════════════

void QueryProcessor::preprocess() {
    buildReverseAdj();
    selectLandmarks();
    computeLandmarkDistances();
}

void QueryProcessor::buildReverseAdj() {
    int n = (int)graph.nodes.size();
    radj.assign(n, {});
    for (int u = 0; u < n; ++u) {
        for (auto& [v, ei] : graph.adj[u]) {
            radj[v].push_back({u, ei});
        }
    }
}

void QueryProcessor::selectLandmarks() {
    int n = (int)graph.nodes.size();
    if (n == 0) return;

    landmarks.clear();
    // Farthest-insertion strategy: pick first landmark as node 0,
    // then repeatedly pick the node farthest from all existing landmarks
    std::vector<double> minDistToLandmark(n, INF);
    int first = 0;
    landmarks.push_back(first);

    for (int li = 0; li < NUM_LANDMARKS && li < n; ++li) {
        int src = landmarks.back();
        std::vector<double> dist(n, INF);
        dist[src] = 0.0;
        std::priority_queue<pdi, std::vector<pdi>, std::greater<pdi>> pq;
        pq.push({0.0, src});

        while (!pq.empty()) {
            auto [d, u] = pq.top(); pq.pop();
            if (d > dist[u] + 1e-9) continue;
            for (auto& [v, ei] : graph.adj[u]) {
                const Edge& e = graph.edges[ei];
                if (e.removed) continue;
                double nd = dist[u] + e.length;
                if (nd < dist[v]) {
                    dist[v] = nd;
                    pq.push({nd, v});
                }
            }
        }

        for (int i = 0; i < n; ++i) {
            minDistToLandmark[i] = std::min(minDistToLandmark[i], dist[i]);
        }

        if ((int)landmarks.size() >= NUM_LANDMARKS) break;

        int best = -1;
        double bestDist = -1.0;
        for (int i = 0; i < n; ++i) {
            if (minDistToLandmark[i] > bestDist && minDistToLandmark[i] < INF) {
                bestDist = minDistToLandmark[i];
                best = i;
            }
        }
        if (best == -1) break;
        landmarks.push_back(best);
    }
}

void QueryProcessor::computeLandmarkDistances() {
    int n = (int)graph.nodes.size();
    int L = (int)landmarks.size();

    landmarkDistFrom.assign(L, std::vector<double>(n, INF));
    landmarkDistTo.assign(L, std::vector<double>(n, INF));

    for (int li = 0; li < L; ++li) {
        int src = landmarks[li];

        // Forward Dijkstra: dist FROM landmark
        {
            auto& dist = landmarkDistFrom[li];
            dist[src] = 0.0;
            std::priority_queue<pdi, std::vector<pdi>, std::greater<pdi>> pq;
            pq.push({0.0, src});
            while (!pq.empty()) {
                auto [d, u] = pq.top(); pq.pop();
                if (d > dist[u] + 1e-9) continue;
                for (auto& [v, ei] : graph.adj[u]) {
                    const Edge& e = graph.edges[ei];
                    if (e.removed) continue;
                    double nd = dist[u] + e.length;
                    if (nd < dist[v]) {
                        dist[v] = nd;
                        pq.push({nd, v});
                    }
                }
            }
        }

        // Reverse Dijkstra: dist TO landmark (from any node)
        {
            auto& dist = landmarkDistTo[li];
            dist[src] = 0.0;
            std::priority_queue<pdi, std::vector<pdi>, std::greater<pdi>> pq;
            pq.push({0.0, src});
            while (!pq.empty()) {
                auto [d, u] = pq.top(); pq.pop();
                if (d > dist[u] + 1e-9) continue;
                for (auto& [v, ei] : radj[u]) {
                    const Edge& e = graph.edges[ei];
                    if (e.removed) continue;
                    double nd = dist[u] + e.length;
                    if (nd < dist[v]) {
                        dist[v] = nd;
                        pq.push({nd, v});
                    }
                }
            }
        }
    }
}

double QueryProcessor::landmarkHeuristic(int u, int target) const {
    double h = 0.0;
    for (int li = 0; li < (int)landmarks.size(); ++li) {
        double d1 = landmarkDistTo[li][u] - landmarkDistTo[li][target];
        double d2 = landmarkDistFrom[li][target] - landmarkDistFrom[li][u];
        h = std::max(h, d1);
        h = std::max(h, d2);
    }
    return h;
}

// ════════════════════════════════════════════════════════════════════════════
//  Dijkstra with node/edge exclusions (distance mode, for Yen's)
// ════════════════════════════════════════════════════════════════════════════

bool QueryProcessor::dijkstraSP(int src, int dst, double& cost,
                                 std::vector<int>& path,
                                 const std::unordered_set<int>& excludeNodes,
                                 const std::set<std::pair<int,int>>& excludeEdges) {
    int n = (int)graph.nodes.size();
    if (src < 0 || src >= n || dst < 0 || dst >= n) return false;
    if (excludeNodes.count(src) || excludeNodes.count(dst)) return false;

    std::vector<double> dist(n, INF);
    std::vector<int> prev(n, -1);
    std::priority_queue<pdi, std::vector<pdi>, std::greater<pdi>> pq;

    dist[src] = 0.0;
    pq.push({0.0, src});

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u] + 1e-9) continue;
        if (u == dst) break;

        for (auto& [v, ei] : graph.adj[u]) {
            const Edge& e = graph.edges[ei];
            if (e.removed) continue;
            if (excludeNodes.count(v)) continue;
            if (!excludeEdges.empty() && excludeEdges.count({u, v})) continue;

            double nd = dist[u] + e.length;
            if (nd < dist[v]) {
                dist[v] = nd;
                prev[v] = u;
                pq.push({nd, v});
            }
        }
    }

    if (std::isinf(dist[dst])) return false;

    cost = dist[dst];
    path.clear();
    for (int cur = dst; cur != -1; cur = prev[cur])
        path.push_back(cur);
    std::reverse(path.begin(), path.end());
    return true;
}

// ════════════════════════════════════════════════════════════════════════════
//  Weighted Dijkstra — edges have penalty multipliers for diversity
// ════════════════════════════════════════════════════════════════════════════

bool QueryProcessor::dijkstraWeighted(int src, int dst, double& cost,
                                       std::vector<int>& path,
                                       const std::unordered_map<int, double>& edgePenalties) {
    int n = (int)graph.nodes.size();
    if (src < 0 || src >= n || dst < 0 || dst >= n) return false;

    std::vector<double> dist(n, INF);
    std::vector<int> prev(n, -1);
    std::priority_queue<pdi, std::vector<pdi>, std::greater<pdi>> pq;

    dist[src] = 0.0;
    pq.push({0.0, src});

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u] + 1e-9) continue;
        if (u == dst) break;

        for (auto& [v, ei] : graph.adj[u]) {
            const Edge& e = graph.edges[ei];
            if (e.removed) continue;

            double w = e.length;
            auto pit = edgePenalties.find(e.id);
            if (pit != edgePenalties.end()) {
                w *= pit->second;
            }

            double nd = dist[u] + w;
            if (nd < dist[v]) {
                dist[v] = nd;
                prev[v] = u;
                pq.push({nd, v});
            }
        }
    }

    if (std::isinf(dist[dst])) return false;

    // Reconstruct path — return actual distance, not penalized distance
    path.clear();
    for (int cur = dst; cur != -1; cur = prev[cur])
        path.push_back(cur);
    std::reverse(path.begin(), path.end());

    cost = computePathLength(path);
    return true;
}

// ════════════════════════════════════════════════════════════════════════════
//  Bidirectional A* with landmark heuristic (ALT algorithm)
// ════════════════════════════════════════════════════════════════════════════

double QueryProcessor::bidirectionalAstar(int src, int dst) {
    int n = (int)graph.nodes.size();
    if (src < 0 || src >= n || dst < 0 || dst >= n) return -1.0;
    if (src == dst) return 0.0;

    std::vector<double> distF(n, INF), distB(n, INF);
    std::vector<bool> settledF(n, false), settledB(n, false);
    std::priority_queue<pdi, std::vector<pdi>, std::greater<pdi>> pqF, pqB;

    distF[src] = 0.0;
    pqF.push({landmarkHeuristic(src, dst), src});

    distB[dst] = 0.0;
    pqB.push({landmarkHeuristic(src, dst), dst});

    double mu = INF;

    while (!pqF.empty() || !pqB.empty()) {
        double topF = pqF.empty() ? INF : pqF.top().first;
        double topB = pqB.empty() ? INF : pqB.top().first;
        if (topF >= mu && topB >= mu) break;

        bool expandForward = !pqF.empty() && (pqB.empty() || topF <= topB);

        if (expandForward) {
            auto [f, u] = pqF.top(); pqF.pop();
            if (settledF[u]) continue;
            settledF[u] = true;

            if (settledB[u])
                mu = std::min(mu, distF[u] + distB[u]);

            for (auto& [v, ei] : graph.adj[u]) {
                const Edge& e = graph.edges[ei];
                if (e.removed) continue;
                double nd = distF[u] + e.length;
                if (nd < distF[v]) {
                    distF[v] = nd;
                    pqF.push({nd + landmarkHeuristic(v, dst), v});
                    if (settledB[v])
                        mu = std::min(mu, nd + distB[v]);
                }
            }
        } else {
            auto [f, u] = pqB.top(); pqB.pop();
            if (settledB[u]) continue;
            settledB[u] = true;

            if (settledF[u])
                mu = std::min(mu, distF[u] + distB[u]);

            for (auto& [v, ei] : radj[u]) {
                const Edge& e = graph.edges[ei];
                if (e.removed) continue;
                double nd = distB[u] + e.length;
                if (nd < distB[v]) {
                    distB[v] = nd;
                    pqB.push({nd + landmarkHeuristic(src, v), v});
                    if (settledF[v])
                        mu = std::min(mu, distF[v] + nd);
                }
            }
        }
    }

    return std::isinf(mu) ? -1.0 : mu;
}

// ════════════════════════════════════════════════════════════════════════════
//  Edge-set helpers
// ════════════════════════════════════════════════════════════════════════════

std::vector<int> QueryProcessor::pathToEdgeIds(const std::vector<int>& nodePath) {
    std::vector<int> eids;
    for (int i = 0; i + 1 < (int)nodePath.size(); ++i) {
        int u = nodePath[i], v = nodePath[i + 1];
        for (auto& [nb, ei] : graph.adj[u]) {
            if (nb == v && !graph.edges[ei].removed) {
                eids.push_back(graph.edges[ei].id);
                break;
            }
        }
    }
    return eids;
}

double QueryProcessor::computePathLength(const std::vector<int>& nodePath) {
    double total = 0.0;
    for (int i = 0; i + 1 < (int)nodePath.size(); ++i) {
        int u = nodePath[i], v = nodePath[i + 1];
        for (auto& [nb, ei] : graph.adj[u]) {
            if (nb == v && !graph.edges[ei].removed) {
                total += graph.edges[ei].length;
                break;
            }
        }
    }
    return total;
}

// ════════════════════════════════════════════════════════════════════════════
//  Yen's K-Shortest Simple Paths (shared helper)
// ════════════════════════════════════════════════════════════════════════════

std::vector<QueryProcessor::PathEntry>
QueryProcessor::yenKShortest(int src, int dst, int maxPaths) {
    struct Candidate {
        double cost;
        std::vector<int> nodes;
        bool operator>(const Candidate& o) const { return cost > o.cost; }
    };

    std::vector<Candidate> A;
    std::priority_queue<Candidate, std::vector<Candidate>, std::greater<Candidate>> B;
    std::set<std::vector<int>> seen;

    double c0; std::vector<int> p0;
    if (!dijkstraSP(src, dst, c0, p0)) return {};
    A.push_back({c0, p0});
    seen.insert(p0);

    for (int ki = 1; ki < maxPaths; ++ki) {
        const auto& prev = A[ki - 1].nodes;
        int spurLen = (int)prev.size() - 1;

        for (int i = 0; i < spurLen; ++i) {
            int spurNode = prev[i];
            std::vector<int> rootPath(prev.begin(), prev.begin() + i + 1);

            std::set<std::pair<int,int>> exEdges;
            for (const auto& a : A) {
                if ((int)a.nodes.size() > i + 1 &&
                    std::equal(rootPath.begin(), rootPath.end(), a.nodes.begin()))
                    exEdges.insert({a.nodes[i], a.nodes[i + 1]});
            }

            std::unordered_set<int> exNodes;
            for (int j = 0; j < i; ++j) exNodes.insert(rootPath[j]);

            double spurCost; std::vector<int> spurPath;
            if (!dijkstraSP(spurNode, dst, spurCost, spurPath, exNodes, exEdges))
                continue;

            std::vector<int> totalPath = rootPath;
            totalPath.insert(totalPath.end(), spurPath.begin() + 1, spurPath.end());

            if (seen.count(totalPath)) continue;
            seen.insert(totalPath);

            double rootCost = computePathLength(rootPath);
            B.push({rootCost + spurCost, totalPath});
        }

        if (B.empty()) break;
        A.push_back(B.top()); B.pop();
    }

    std::vector<PathEntry> result;
    for (const auto& a : A) {
        result.push_back({a.cost, a.nodes, pathToEdgeIds(a.nodes)});
    }
    return result;
}

// ════════════════════════════════════════════════════════════════════════════
//  Main dispatcher
// ════════════════════════════════════════════════════════════════════════════

json QueryProcessor::processQueries(const json& input) {
    json output;
    output["meta"] = input.value("meta", json::object());
    output["results"] = json::array();

    const auto& events = input.value("events", json::array());
    for (const auto& event : events) {
        json result;
        auto t0 = std::chrono::high_resolution_clock::now();
        try {
            std::string type = event.at("type").get<std::string>();
            if (type == "k_shortest_paths")
                result = handleKShortestPaths(event);
            else if (type == "k_shortest_paths_heuristic")
                result = handleKShortestPathsHeuristic(event);
            else if (type == "approx_shortest_path")
                result = handleApproxShortestPath(event);
            else {
                result["id"] = event.value("id", -1);
                result["error"] = "Unknown query type: " + type;
            }
        } catch (const std::exception& e) {
            result["id"] = event.value("id", -1);
            result["error"] = e.what();
            std::cerr << "[Phase2] Query error: " << e.what() << "\n";
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        result["processing_time"] =
            std::chrono::duration<double, std::milli>(t1 - t0).count();
        output["results"].push_back(result);
    }
    return output;
}

// ════════════════════════════════════════════════════════════════════════════
//  Query 1: K Shortest Paths (Exact) — Yen's Algorithm
// ════════════════════════════════════════════════════════════════════════════

json QueryProcessor::handleKShortestPaths(const json& query) {
    json result;
    result["id"] = query.value("id", -1);

    int src = query.at("source").get<int>();
    int dst = query.at("target").get<int>();
    int k   = query.at("k").get<int>();

    auto paths = yenKShortest(src, dst, k);

    json pathsJson = json::array();
    for (const auto& p : paths) {
        json entry;
        entry["path"] = p.nodes;
        entry["length"] = p.cost;
        pathsJson.push_back(entry);
    }
    result["paths"] = pathsJson;
    return result;
}

// ════════════════════════════════════════════════════════════════════════════
//  Query 2: K Shortest Paths (Heuristic) — Penalty-aware diverse paths
// ════════════════════════════════════════════════════════════════════════════

json QueryProcessor::handleKShortestPathsHeuristic(const json& query) {
    json result;
    result["id"] = query.value("id", -1);

    int src = query.at("source").get<int>();
    int dst = query.at("target").get<int>();
    int k   = query.at("k").get<int>();
    double overlapThreshold = query.value("overlap_threshold", 60.0) / 100.0;

    // Generate candidate pool via Yen's
    int yenLimit = std::min(k * 6, 42);
    auto yenPaths = yenKShortest(src, dst, yenLimit);
    if (yenPaths.empty()) {
        result["paths"] = json::array();
        return result;
    }

    double bestCost = yenPaths[0].cost;
    std::vector<PathEntry> pool = yenPaths;

    // Augment pool with penalized-Dijkstra paths for diversity
    {
        std::unordered_map<int, double> penalties;
        double penaltyFactor = 2.0;

        for (int iter = 0; iter < k * 3; ++iter) {
            for (const auto& pe : pool) {
                for (int eid : pe.edgeIds) {
                    penalties[eid] += penaltyFactor;
                }
            }

            double cost; std::vector<int> path;
            if (!dijkstraWeighted(src, dst, cost, path, penalties)) break;

            auto eids = pathToEdgeIds(path);
            bool dup = false;
            for (const auto& pe : pool) {
                if (pe.nodes == path) { dup = true; break; }
            }
            if (!dup) {
                pool.push_back({cost, path, eids});
            }

            penaltyFactor *= 1.5;
        }
    }

    // Greedy selection — minimize overlap + distance penalty
    std::vector<int> selectedIdx;
    selectedIdx.push_back(0);

    while ((int)selectedIdx.size() < k && (int)selectedIdx.size() < (int)pool.size()) {
        double bestPenalty = INF;
        int bestCandidate = -1;

        for (int ci = 1; ci < (int)pool.size(); ++ci) {
            bool alreadySelected = false;
            for (int si : selectedIdx) {
                if (si == ci) { alreadySelected = true; break; }
            }
            if (alreadySelected) continue;

            std::vector<int> trial = selectedIdx;
            trial.push_back(ci);

            double totalPenalty = 0.0;
            for (int ti : trial) {
                const auto& pi = pool[ti];
                std::unordered_set<int> piEdgeSet(pi.edgeIds.begin(), pi.edgeIds.end());

                int overlapCount = 0;
                for (int tj : trial) {
                    if (ti == tj) continue;
                    const auto& pj = pool[tj];
                    int common = 0;
                    for (int eid : pj.edgeIds) {
                        if (piEdgeSet.count(eid)) ++common;
                    }
                    double overlapPct = pi.edgeIds.empty() ? 0.0 :
                        (double)common / (double)pi.edgeIds.size() * 100.0;
                    if (overlapPct > overlapThreshold * 100.0) {
                        ++overlapCount;
                    }
                }

                double distPenalty = 0.0;
                if (bestCost > 1e-9) {
                    distPenalty = (pi.cost - bestCost) / bestCost + 0.1;
                } else {
                    distPenalty = 0.1;
                }

                totalPenalty += overlapCount * distPenalty;
            }

            if (totalPenalty < bestPenalty) {
                bestPenalty = totalPenalty;
                bestCandidate = ci;
            }
        }

        if (bestCandidate == -1) break;
        selectedIdx.push_back(bestCandidate);
    }

    std::vector<PathEntry> selected;
    for (int si : selectedIdx) {
        selected.push_back(pool[si]);
    }
    std::sort(selected.begin(), selected.end(),
              [](const PathEntry& a, const PathEntry& b) { return a.cost < b.cost; });

    json paths = json::array();
    for (const auto& s : selected) {
        json p;
        p["path"] = s.nodes;
        p["length"] = s.cost;
        paths.push_back(p);
    }
    result["paths"] = paths;
    return result;
}

// ════════════════════════════════════════════════════════════════════════════
//  Query 3: Approximate Shortest Path — Bidirectional A* with Landmarks
// ════════════════════════════════════════════════════════════════════════════

json QueryProcessor::handleApproxShortestPath(const json& query) {
    json result;
    result["id"] = query.value("id", -1);

    double budgetMs = query.value("time_budget_ms", 100.0);
    const auto& queries = query.at("queries");

    json distances = json::array();
    auto globalStart = std::chrono::high_resolution_clock::now();

    for (const auto& q : queries) {
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double, std::milli>(now - globalStart).count();
        if (elapsed > budgetMs * 0.85) break;

        int src = q.at("source").get<int>();
        int dst = q.at("target").get<int>();

        double dist = bidirectionalAstar(src, dst);

        json entry;
        entry["source"] = src;
        entry["target"] = dst;
        entry["approx_shortest_distance"] = dist;
        distances.push_back(entry);
    }

    result["distances"] = distances;
    return result;
}
