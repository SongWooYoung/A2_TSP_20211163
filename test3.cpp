#include "TSParser.h"
#include <iostream>
#include "christofides.h"

using namespace std;


int main() {

    TSPParser tsp_parser("xql662.tsp");
    tsp_parser.parse();
    tsp_parser.printInfo();

    const map<int, pair<double, double>>& node_list = tsp_parser.getNodes();

    christofides xql662(node_list);

    xql662.compute_mst();
    // vector<pair<int, int>> list = xql662.get_mst_edges();

    // for (pair<int, int> a: list) {
    //     cout << "from: " << a.first << ", to: " << a.second << endl;
    // }

    // 아래 코드는 수정전 만들어놔서 실행 x
    // vector<int> list = xql662.odd_indices();
    // cout << "odd Indices: " << endl;
    // for (int index: list) {
    //     cout << index << " ";
    // }
    // cout << endl;

        

    return 0;
}


