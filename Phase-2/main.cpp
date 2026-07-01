// Phase 2 entry point — load graph, preprocess landmarks, run queries.
#include <iostream>
#include <fstream>
#include "../Common/Graph.hpp"
#include "QueryProcessor.hpp"
#include "../Common/json.hpp"

int main(int argc, char* argv[]) {
    try {
        if (argc != 4) {
            std::cerr << "Usage: ./phase2 <graph.json> <queries.json> <output.json>\n";
            return 1;
        }

        std::string graphPath  = argv[1];
        std::string queryPath  = argv[2];
        std::string outputPath = argv[3];

        Graph graph;
        graph.loadFromFile(graphPath);

        QueryProcessor qp(graph);
        qp.preprocess();

        std::ifstream qFile(queryPath);
        if (!qFile.is_open()) {
            std::cerr << "Could not open queries file: " << queryPath << "\n";
            return 1;
        }
        nlohmann::json queries;
        qFile >> queries;

        nlohmann::json output = qp.processQueries(queries);

        std::ofstream oFile(outputPath);
        if (!oFile.is_open()) {
            std::cerr << "Could not open output file: " << outputPath << "\n";
            return 1;
        }
        oFile << output.dump(2);

        std::cout << "Phase 2 completed. Output written to " << outputPath << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << "\n";
        return 1;
    }
}
