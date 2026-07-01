// Phase 2 query processor — K-shortest paths, heuristic diversity, bidirectional A*.
#pragma once
#include "../Common/Graph.hpp"
#include "../Common/json.hpp"
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <set>

class QueryProcessor {
public:
    QueryProcessor(Graph& g);
    void preprocess();
    nlohmann::json processQueries(const nlohmann::json& input);

private:
    Graph& graph;

    // ── Landmark precomputation for approximate shortest path ──
    static constexpr int NUM_LANDMARKS = 16;
    std::vector<int> landmarks;
    std::vector<std::vector<double>> landmarkDistFrom; // landmarkDistFrom[l][v]
    std::vector<std::vector<double>> landmarkDistTo;   // landmarkDistTo[l][v]
    std::vector<std::vector<std::pair<int, int>>> radj; // reverse adjacency
    void buildReverseAdj();
    void selectLandmarks();
    void computeLandmarkDistances();
    double landmarkHeuristic(int u, int target) const;

    // ── Algorithm helpers ──
    bool dijkstraSP(int src, int dst, double& cost, std::vector<int>& path,
                    const std::unordered_set<int>& excludeNodes = {},
                    const std::set<std::pair<int,int>>& excludeEdges = {});

    bool dijkstraWeighted(int src, int dst, double& cost, std::vector<int>& path,
                          const std::unordered_map<int, double>& edgePenalties);

    double bidirectionalAstar(int src, int dst);

    struct PathEntry {
        double cost;
        std::vector<int> nodes;
        std::vector<int> edgeIds;
    };
    std::vector<PathEntry> yenKShortest(int src, int dst, int maxPaths);

    std::vector<int> pathToEdgeIds(const std::vector<int>& nodePath);
    double computePathLength(const std::vector<int>& nodePath);

    // ── Query handlers ──
    nlohmann::json handleKShortestPaths(const nlohmann::json& query);
    nlohmann::json handleKShortestPathsHeuristic(const nlohmann::json& query);
    nlohmann::json handleApproxShortestPath(const nlohmann::json& query);
};
