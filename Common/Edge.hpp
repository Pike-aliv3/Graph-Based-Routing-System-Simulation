// Edge — weighted directed edge with speed profile for time-dependent routing.
#pragma once
#include <string>
#include <vector>

struct Edge {
    int id;
    int u, v;
    double length;
    double average_time;
    std::vector<double> speed_profile;
    bool oneway;
    std::string road_type;
    bool removed = false;
};