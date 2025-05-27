#include "TSParser.h"
#include <iostream>
#include "christofides.h"
//#include "approx2.h"
//#include "Held_Karp.h"

using namespace std;


int main() {

    TSPParser tsp_parser("a280.tsp");
    tsp_parser.parse();
    tsp_parser.printInfo();

    const map<int, pair<double, double>>& node_list = tsp_parser.getNodes();
    // 상위 N개 노드 선택
    map<int, pair<double, double>> selected_nodes;
    int count = 0;
    for (const auto& [id, coord] : node_list) {
        selected_nodes[id] = coord;
        if (++count >= 136) break;
    }

    christofides TSP_ch(selected_nodes);
    //approx2 TSP_apx2(selected_nodes);

    //TSP_apx2.execute_all();
    TSP_ch.execute_all();

    return 0;
}


