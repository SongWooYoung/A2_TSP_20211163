#include "TSParser.h"
#include <iostream>
#include "christofides.h"

using namespace std;


int main() {

    TSPParser tsp_parser("xql662.tsp");
    tsp_parser.parse();
    tsp_parser.printInfo();

    const map<int, pair<double, double>>& node_list = tsp_parser.getNodes();
    // 상위 N개 노드 선택
    map<int, pair<double, double>> selected_nodes;
    int count = 0;
    for (const auto& [id, coord] : node_list) {
        selected_nodes[id] = coord;
        if (++count >= 600) break;
    }

    christofides xql662(selected_nodes);

    // 1. MST
    xql662.compute_mst();
    vector<pair<int, int>> even_degree_edges = xql662.get_mst_edges();

    // 2. get odd indices
    xql662.odd_indices();
 
    // 3. do minimum perfect matching and get pairs
    vector<pair<mcpm_node_idx, mcpm_node_idx>> get_edges = xql662.mcpm();
    
    // 4. merge pairs with mst edges
    even_degree_edges.insert(even_degree_edges.end(), get_edges.begin(), get_edges.end());

    // 5. Euler path

    return 0;
}


