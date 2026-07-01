// Phase 1 query processor — handles individual routing and graph-update queries.
#include "QueryProcessor.hpp"
#include <iostream>
#include <chrono>
#include <unordered_set>

using json = nlohmann::json;

QueryProcessor::QueryProcessor(Graph& g) : graph(g) {}

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

            if (type == "shortest_path")
                result = handleShortestPath(event);
            else if (type == "knn")
                result = handleKNN(event);
            else if (type == "remove_edge")
                result = handleRemoveEdge(event);
            else if (type == "modify_edge")
                result = handleModifyEdge(event);
            else {
                result["id"] = event.value("id", -1);
                result["error"] = "Unknown query type: " + type;
            }
        } catch (const std::exception& e) {
            result["id"] = event.value("id", -1);
            result["error"] = e.what();
            std::cerr << "[Phase1] Query error: " << e.what() << "\n";
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        result["processing_time"] =
            std::chrono::duration<double, std::milli>(t1 - t0).count();
        output["results"].push_back(result);
    }

    return output;
}

// ════════════════════════════════════════════════════════════════════════════
//  Shortest path — Dijkstra with mode and constraints
// ════════════════════════════════════════════════════════════════════════════

json QueryProcessor::handleShortestPath(const json& query) {
    json result;
    result["id"] = query.value("id", -1);

    if (!query.contains("source") || !query.contains("target")) {
        result["error"] = "shortest_path query missing source or target";
        return result;
    }

    int source = query["source"].get<int>();
    int target = query["target"].get<int>();

    std::string mode = query.value("mode", std::string("distance"));
    double startTime = query.value("start_time", 0.0);

    // Optional constraints: forbidden nodes and/or road types
    std::unordered_set<int> forbiddenNodes;
    std::unordered_set<std::string> forbiddenRoadTypes;
    if (query.contains("constraints") && query["constraints"].is_object()) {
        const auto& c = query["constraints"];
        if (c.contains("forbidden_nodes") && c["forbidden_nodes"].is_array()) {
            for (const auto& x : c["forbidden_nodes"]) {
                forbiddenNodes.insert(x.get<int>());
            }
        }
        if (c.contains("forbidden_road_types") && c["forbidden_road_types"].is_array()) {
            for (const auto& x : c["forbidden_road_types"]) {
                forbiddenRoadTypes.insert(x.get<std::string>());
            }
        }
    }

    double cost = 0.0;
    std::vector<int> path;

    bool ok = graph.dijkstra(
        source, target, cost, path,
        mode, forbiddenNodes, forbiddenRoadTypes, startTime
    );

    result["possible"] = ok;

    if (ok) {
        // Report as time or distance depending on mode
        if (mode == "time")
            result["minimum_time"] = cost;
        else
            result["minimum_distance"] = cost;
        result["path"] = path;
    }

    return result;
}

// ════════════════════════════════════════════════════════════════════════════
//  K-nearest neighbor query
// ════════════════════════════════════════════════════════════════════════════

json QueryProcessor::handleKNN(const json& query) {
    json result;
    result["id"] = query.value("id", -1);

    if (!query.contains("poi") || !query.contains("k")) {
        result["error"] = "knn query missing poi or k";
        return result;
    }

    std::string poi = query["poi"].get<std::string>();
    int k = query["k"].get<int>();
    std::string metric = query.value("metric", std::string("euclidean"));

    double lat, lon;

    // Accept both nested query_point and flat lat/lon
    if (query.contains("query_point")) {
        if (!query["query_point"].contains("lat") || !query["query_point"].contains("lon")) {
            result["error"] = "query_point missing lat/lon";
            return result;
        }
        lat = query["query_point"]["lat"].get<double>();
        lon = query["query_point"]["lon"].get<double>();
    }
    else if (query.contains("lat") && query.contains("lon")) {
        lat = query["lat"].get<double>();
        lon = query["lon"].get<double>();
    }
    else {
        result["error"] = "knn query missing location";
        return result;
    }

    std::vector<int> ans = graph.knn(lat, lon, poi, k, metric);

    result["possible"] = true;
    result["nodes"] = ans;
    return result;
}

// ════════════════════════════════════════════════════════════════════════════
//  Dynamic graph updates
// ════════════════════════════════════════════════════════════════════════════

json QueryProcessor::handleRemoveEdge(const json& query) {
    json result;
    result["id"] = query.value("id", -1);

    if (!query.contains("edge_id")) {
        result["error"] = "remove_edge query missing edge_id";
        return result;
    }

    int edgeId = query["edge_id"].get<int>();
    result["done"] = graph.removeEdge(edgeId);
    return result;
}

json QueryProcessor::handleModifyEdge(const json& query) {
    json result;
    result["id"] = query.value("id", -1);

    if (!query.contains("edge_id")) {
        result["error"] = "modify_edge query missing edge_id";
        return result;
    }

    int edgeId = query["edge_id"].get<int>();
    json patch = json::object();

    // Support both a single "patch" object and individual top-level fields
    if (query.contains("patch") && query["patch"].is_object()) {
        patch = query["patch"];
    } else {
        if (query.contains("length")) patch["length"] = query["length"];
        if (query.contains("average_time")) patch["average_time"] = query["average_time"];
        if (query.contains("road_type")) patch["road_type"] = query["road_type"];
        if (query.contains("oneway")) patch["oneway"] = query["oneway"];
        if (query.contains("speed_profile")) patch["speed_profile"] = query["speed_profile"];
    }

    result["done"] = graph.modifyEdge(edgeId, patch);
    return result;
}
