// Graph — loading, pathfinding, and dynamic modification implementation.
#include "Graph.hpp"
#include "KDTree.hpp"
#include <fstream>
#include <queue>
#include <limits>
#include <algorithm>
#include <cmath>
#include <stdexcept>

using json = nlohmann::json;

// ════════════════════════════════════════════════════════════════════════════
//  Graph I/O
// ════════════════════════════════════════════════════════════════════════════

void Graph::loadFromFile(const std::string& path) {
    nodes.clear();
    edges.clear();
    adj.clear();
    edgeIdToIndex.clear();

    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open graph file: " + path);
    }

    json data;
    try {
        file >> data;
    } catch (...) {
        throw std::runtime_error("Invalid JSON in graph file: " + path);
    }

    if (!data.contains("nodes") || !data["nodes"].is_array()) {
        throw std::runtime_error("Graph JSON missing 'nodes' array");
    }
    if (!data.contains("edges") || !data["edges"].is_array()) {
        throw std::runtime_error("Graph JSON missing 'edges' array");
    }

    for (auto& n : data["nodes"]) {
        Node node;
        node.id  = n.at("id").get<int>();
        node.lat = n.at("lat").get<double>();
        node.lon = n.at("lon").get<double>();

        if (n.contains("pois") && n["pois"].is_array()) {
            for (auto& p : n["pois"]) {
                node.pois.push_back(p.get<std::string>());
            }
        }

        nodes.push_back(node);
    }

    for (auto& e : data["edges"]) {
        Edge edge;
        edge.id = e.at("id").get<int>();
        edge.u  = e.at("u").get<int>();
        edge.v  = e.at("v").get<int>();
        edge.length = e.at("length").get<double>();
        edge.average_time = e.at("average_time").get<double>();
        edge.oneway = e.value("oneway", false);
        edge.road_type = e.value("road_type", std::string("local"));

        if (e.contains("speed_profile") && e["speed_profile"].is_array()) {
            for (auto& s : e["speed_profile"]) {
                edge.speed_profile.push_back(s.get<double>());
            }
        }

        edgeIdToIndex[edge.id] = (int)edges.size();
        edges.push_back(edge);
    }

    buildAdjList();
}

// ════════════════════════════════════════════════════════════════════════════
//  Adjacency list construction
// ════════════════════════════════════════════════════════════════════════════

void Graph::buildAdjList() {
    adj.assign(nodes.size(), {});

    for (int i = 0; i < (int)edges.size(); i++) {
        int u = edges[i].u;
        int v = edges[i].v;

        if (u < 0 || u >= (int)nodes.size() || v < 0 || v >= (int)nodes.size()) {
            continue;
        }

        adj[u].push_back({v, i});
        if (!edges[i].oneway) {
            adj[v].push_back({u, i});
        }
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  Time-dependent edge traversal
// ════════════════════════════════════════════════════════════════════════════

double Graph::edgeTravelTime(const Edge& e, double absoluteStartTime) const {
    if (e.speed_profile.empty()) {
        return e.average_time;
    }

    // Simulate traversal across time slots — speed can change mid-edge
    double remaining = e.length;
    double currentTime = absoluteStartTime;

    while (remaining > 1e-9) {
        int slot = ((int)(currentTime / 900.0)) % (int)e.speed_profile.size(); // 15 min slots
        double speed = e.speed_profile[slot];

        if (speed <= 1e-12) {
            return 1e18;
        }

        double timeIntoSlot = std::fmod(currentTime, 900.0);
        double timeLeftInSlot = 900.0 - timeIntoSlot;
        if (timeLeftInSlot < 1e-9) timeLeftInSlot = 900.0;

        double distCoverable = speed * timeLeftInSlot;

        if (distCoverable >= remaining) {
            currentTime += remaining / speed;
            remaining = 0.0;
        } else {
            remaining -= distCoverable;
            currentTime += timeLeftInSlot;
        }
    }

    return currentTime - absoluteStartTime;
}

// ════════════════════════════════════════════════════════════════════════════
//  Dijkstra's shortest path
// ════════════════════════════════════════════════════════════════════════════

bool Graph::dijkstra(int source, int target,
                     double& outCost,
                     std::vector<int>& outPath,
                     const std::string& mode,
                     const std::unordered_set<int>& forbiddenNodes,
                     const std::unordered_set<std::string>& forbiddenRoadTypes,
                     double startTime) {
    if (source < 0 || source >= (int)nodes.size()) return false;
    if (target < 0 || target >= (int)nodes.size()) return false;
    if (forbiddenNodes.count(source) || forbiddenNodes.count(target)) return false;

    int n = (int)nodes.size();
    const double INF = std::numeric_limits<double>::infinity();

    std::vector<double> dist(n, INF);
    std::vector<int> parent(n, -1);

    dist[source] = 0.0;

    std::priority_queue<
        std::pair<double,int>,
        std::vector<std::pair<double,int>>,
        std::greater<std::pair<double,int>>
    > pq;

    pq.push({0.0, source});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > dist[u]) continue;
        if (u == target) break;

        for (auto& [v, ei] : adj[u]) {
            const Edge& e = edges[ei];

            if (e.removed) continue;
            if (forbiddenNodes.count(v)) continue;
            if (!forbiddenRoadTypes.empty() && forbiddenRoadTypes.count(e.road_type)) continue;

            // Time mode accumulates absolute time for speed-profile lookup
            double w = (mode == "time")
                ? edgeTravelTime(e, startTime + d)
                : e.length;

            if (d + w < dist[v]) {
                dist[v] = d + w;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    if (dist[target] == INF) {
        return false;
    }

    outCost = dist[target];
    outPath.clear();

    for (int cur = target; cur != -1; cur = parent[cur]) {
        outPath.push_back(cur);
    }
    std::reverse(outPath.begin(), outPath.end());

    return true;
}

// ════════════════════════════════════════════════════════════════════════════
//  Dynamic graph updates
// ════════════════════════════════════════════════════════════════════════════

bool Graph::removeEdge(int edgeId) {
    auto it = edgeIdToIndex.find(edgeId);
    if (it == edgeIdToIndex.end()) return false;

    Edge& e = edges[it->second];
    if (e.removed) return false;

    e.removed = true;
    return true;
}

bool Graph::modifyEdge(int edgeId, const nlohmann::json& patch) {
    auto it = edgeIdToIndex.find(edgeId);
    if (it == edgeIdToIndex.end()) return false;

    Edge& e = edges[it->second];

    // Empty patch on a removed edge re-enables it; empty patch on a live edge is a no-op
    if (e.removed) {
        e.removed = false;
        if (patch.empty()) {
            return true;
        }
    } else {
        if (patch.empty()) {
            return false;
        }
    }

    if (patch.contains("length")) {
        e.length = patch["length"].get<double>();
    }
    if (patch.contains("average_time")) {
        e.average_time = patch["average_time"].get<double>();
    }
    if (patch.contains("road_type")) {
        e.road_type = patch["road_type"].get<std::string>();
    }
    if (patch.contains("oneway")) {
        e.oneway = patch["oneway"].get<bool>();
    }
    if (patch.contains("speed_profile") && patch["speed_profile"].is_array()) {
        e.speed_profile.clear();
        for (auto& s : patch["speed_profile"]) {
            e.speed_profile.push_back(s.get<double>());
        }
    }

    // Rebuild adjacency in case oneway changed
    buildAdjList();
    return true;
}

// ════════════════════════════════════════════════════════════════════════════
//  Nearest node (Euclidean)
// ════════════════════════════════════════════════════════════════════════════

int Graph::nearestNode(double lat, double lon) const {
    if (nodes.empty()) return -1;

    int best = 0;
    double bestDist = std::numeric_limits<double>::infinity();

    for (int i = 0; i < (int)nodes.size(); i++) {
        double dlat = nodes[i].lat - lat;
        double dlon = nodes[i].lon - lon;
        double d = dlat * dlat + dlon * dlon;

        if (d < bestDist) {
            bestDist = d;
            best = i;
        }
    }

    return best;
}

// ════════════════════════════════════════════════════════════════════════════
//  K-nearest neighbors
// ════════════════════════════════════════════════════════════════════════════

std::vector<int> Graph::knn(double lat, double lon,
                            const std::string& poi,
                            int k,
                            const std::string& metric) {
    std::vector<int> poiNodes;

    for (const auto& node : nodes) {
        for (const auto& p : node.pois) {
            if (p == poi) {
                poiNodes.push_back(node.id);
                break;
            }
        }
    }

    if (poiNodes.empty() || k <= 0) {
        return {};
    }

    if (metric == "euclidean") {
        // Euclidean: reuse a per-POI-category KD-tree instead of rebuilding
        // it on every call — the node set is fixed once the graph is loaded.
        auto it = poiTreeCache.find(poi);
        if (it == poiTreeCache.end()) {
            std::vector<KDTree::Point> points;
            points.reserve(poiNodes.size());
            for (int id : poiNodes) {
                points.push_back({nodes[id].lat, nodes[id].lon, id});
            }
            KDTree tree;
            tree.build(std::move(points));
            it = poiTreeCache.emplace(poi, std::move(tree)).first;
        }
        return it->second.kNearest(lat, lon, k);
    } else {
        // Network: Dijkstra from nearest node, sort by shortest-path distance
        int startNode = nearestNode(lat, lon);
        if (startNode == -1) return {};

        int n = (int)nodes.size();
        const double INF = std::numeric_limits<double>::infinity();

        std::vector<double> dist(n, INF);
        dist[startNode] = 0.0;

        std::priority_queue<
            std::pair<double,int>,
            std::vector<std::pair<double,int>>,
            std::greater<std::pair<double,int>>
        > pq;

        pq.push({0.0, startNode});

        while (!pq.empty()) {
            auto [d, u] = pq.top();
            pq.pop();

            if (d > dist[u]) continue;

            for (auto& [v, ei] : adj[u]) {
                const Edge& e = edges[ei];
                if (e.removed) continue;

                if (d + e.length < dist[v]) {
                    dist[v] = d + e.length;
                    pq.push({dist[v], v});
                }
            }
        }

        std::sort(poiNodes.begin(), poiNodes.end(),
            [&](int a, int b) {
                return dist[a] < dist[b];
            }
        );

        while (!poiNodes.empty() && dist[poiNodes.back()] == INF) {
            poiNodes.pop_back();
        }
    }

    if ((int)poiNodes.size() > k) {
        poiNodes.resize(k);
    }

    return poiNodes;
}
