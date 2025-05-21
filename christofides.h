#ifndef CHRISTOFIDES_H
#define CHRISTOFIDES_H

#include <vector>
#include <map>
#include "mcpm.h"


class christofides {
private:
    std::map<int, std::pair<double, double>> nodes;
    std::vector<std::pair<int, int>> mst_edges;
    std::vector<mcpm_node> oddIndices;

public:
    christofides(const std::map<int, std::pair<double, double>>& nodes);
    double compute_distance(int u, int v);
    void compute_mst();
    std::vector<std::pair<int, int>> get_mst_edges();
    std::vector<mcpm_node>& odd_indices();
    void print_oddIndices();
    std::vector<std::pair<mcpm_node_idx, mcpm_node_idx>> mcpm();
    std::vector<std::pair<int, int>> get_mst_edges();
};

#endif
