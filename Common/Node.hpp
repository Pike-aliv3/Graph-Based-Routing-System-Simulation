// Node — graph vertex with coordinates and points of interest.
#pragma once
#include <string>
#include <vector>

struct Node {
    int id;
    double lat;
    double lon;
    std::vector<std::string> pois;
};