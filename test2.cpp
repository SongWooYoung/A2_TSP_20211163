#include "TSParser.h"
#include "Held_Karp.h"
// #include "christofides.h" // 향후 추가될 파일
#include <iostream>
#include <map>
#include <vector>
#include <string>

using namespace std;

int main(int argc, char* argv[]) {
    if (argc < 4) {
        cerr << "❗ Usage: " << argv[0] << " <tsp_filename> <model: held_karp | christofides> <num_nodes>\n";
        return 1;
    }

    string filename = argv[1];
    string model = argv[2];
    int num_nodes = stoi(argv[3]);

    TSPParser tsp_parser(filename);
    tsp_parser.parse();
    tsp_parser.printInfo();

    const map<int, pair<double, double>>& node_list = tsp_parser.getNodes();

    if (node_list.empty()) {
        cout << "❗ No nodes loaded.\n";
        return 1;
    }

    if (node_list.size() < num_nodes) {
        cout << "❗ Not enough nodes in the file. Requested: " << num_nodes << ", Available: " << node_list.size() << endl;
        return 1;
    }

    // 상위 N개 노드 선택
    map<int, pair<double, double>> selected_nodes;
    int count = 0;
    for (const auto& node : node_list) {
        selected_nodes[node.first] = node.second;
        count++;
        if (count >= num_nodes) break;
    }

    cout << "✅ Selected " << num_nodes << " nodes.\n";

    if (model == "held_karp") {
        cout << "🔧 Running Held-Karp...\n";
        HK_TSP tsp(selected_nodes);
        tsp.solve();
        double result = tsp.min_cost();
        cout << "🎯 Held-Karp Estimated Min Cost: " << result << endl;

        vector<int> path = tsp.get_path();
        cout << "📍 Held-Karp Path: ";
        for (int city : path) {
            cout << city << " -> ";
        }
        cout << "END" << endl;
    }
    else if (model == "christofides") {
        cout << "🔧 Running Christofides...\n";
        // TODO: 구현 예정
        cout << "❗ Christofides is not yet implemented.\n";
    }
    else {
        cerr << "❌ Unknown model. Please use 'held_karp' or 'christofides'.\n";
        return 1;
    }

    return 0;
}
