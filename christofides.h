#ifndef CHRISTOFIDES_H
#define CHRISTOFIDES_H

#include <vector>
#include <tuple>
#include <map>
#include <queue>
#include "mcpm.h"
#include <cmath>
#include <set>
#include <limits>
#include <unordered_map>
#include <iostream>
#include <stack>
#include <algorithm>
#include <unordered_set>

class christofides {
private:
    std::map<int, std::pair<double, double>> nodes;
    std::vector<std::tuple<int, int, double>> mst_edges; //  x, y, weight
    std::vector<mcpm_node> oddIndices;
    std::unordered_map<int, std::unordered_map<int, double>> edges_ET;
    std::vector<int> path;

public:
    christofides(const std::map<int, std::pair<double, double>>& nodes);

    // 유틸리티
    double compute_distance(int u, int v);
    std::vector<std::tuple<int, int, double>> get_mst_edges();
    void print_oddIndices();
    void print_path();

    // process
    void compute_mst();
    std::vector<mcpm_node>& odd_indices();
    void mcpm();
    void euler_tour();
    void erase_dups();
    double total_dist();

    // total process
    void execute_all();
};

#endif
