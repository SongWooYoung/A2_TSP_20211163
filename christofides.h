#ifndef CHRISTOFIDES_H
#define CHRISTOFIDES_H


//#include "mcpm.h"
#include "mcpm2.h"

#include <vector>
#include <tuple>
#include <map>
#include <queue>
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
    void print_mst_total_weight();
    void print_oddIndices();
    void print_oddIndices2();
    void print_path();

    // process
    void compute_mst();
    void odd_indices();
    void mcpm();
    void euler_tour();
    void erase_dups();
    void erase_dups2();
    void erase_dups_optimal();
    double total_dist();

    // total process
    void execute_all();
};

#endif
