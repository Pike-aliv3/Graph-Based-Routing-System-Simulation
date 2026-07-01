// Graph — adjacency-list graph with Dijkstra, KNN, and dynamic edge updates.
#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include "Node.hpp"
#include "Edge.hpp"
#include "KDTree.hpp"
#include "json.hpp"

class Graph {
public:
    // ── Graph I/O ──
    void loadFromFile(const std::string& path);
    void buildAdjList();

    // ── Shortest path & KNN ──
    bool dijkstra(int source, int target, double& outCost, std::vector<int>& outPath,
                  const std::string& mode = "distance",
                  const std::unordered_set<int>& forbiddenNodes = {},
                  const std::unordered_set<std::string>& forbiddenRoadTypes = {},
                  double startTime = 0.0);

    std::vector<int> knn(double lat, double lon, const std::string& poi,
                         int k, const std::string& metric);

    // ── Dynamic graph updates ──
    bool removeEdge(int edgeId);
    bool modifyEdge(int edgeId, const nlohmann::json& patch);

    // ── Utilities ──
    int nearestNode(double lat, double lon) const;
    double edgeTravelTime(const Edge& e, double startTime) const;

    // ── Graph data ──
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    std::vector<std::vector<std::pair<int, int>>> adj;
    std::unordered_map<int, int> edgeIdToIndex;

private:
    // Lazily-built, cached per POI category. Safe to keep for the lifetime
    // of the Graph because the node set is fixed after loadFromFile() —
    // only edges are ever added/removed/modified.
    mutable std::unordered_map<std::string, KDTree> poiTreeCache;
};
