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

    xql662.compute_mst();
    xql662.odd_indices();
    //xql662.print_oddIndices();

    xql662.mcpm();

    return 0;
}


