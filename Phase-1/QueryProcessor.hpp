// Phase 1 query processor — shortest path, KNN, edge removal/modification.
#pragma once
#include "../Common/Graph.hpp"
#include "../Common/json.hpp"

class QueryProcessor {
public:
    QueryProcessor(Graph& g);
    nlohmann::json processQueries(const nlohmann::json& input);

private:
    Graph& graph;

    // ── Query handlers ──
    nlohmann::json handleShortestPath(const nlohmann::json& query);
    nlohmann::json handleKNN(const nlohmann::json& query);
    nlohmann::json handleRemoveEdge(const nlohmann::json& query);
    nlohmann::json handleModifyEdge(const nlohmann::json& query);
};
