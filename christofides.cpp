#include "christofides.h"
#include "mcmp.h"
#include <cmath>
#include <set>
#include <limits>
#include <unordered_map>

using namespace std;

#define INF 1e12

double christofides::compute_distance(int u, int v) {
    auto [x1, y1] = nodes[u];
    auto [x2, y2] = nodes[v];
    double dx = x1 - x2;
    double dy = y1 - y2;
    return sqrt(dx * dx + dy * dy);
}

christofides::christofides(const map<int, pair<double, double>>& nodes) {
    this->nodes = nodes;
}

void christofides::compute_mst() {
    if (this -> nodes.empty()) return;

    set<int> in_mst;
    set<int> remaining;
    for (const auto& node : this -> nodes) {
        remaining.insert(node.first);
    }

    // 임의 시작점
    int start = this -> nodes.begin()->first;
    in_mst.insert(start);
    remaining.erase(start);

    while (!remaining.empty()) {
        double min_dist = INF;
        int from = -1;
        int to = -1;

        // 현재 MST에 포함된 노드들에서, MST 밖의 노드까지 최소 거리 탐색
        for (int u : in_mst) {
            for (int v : remaining) {
                double dist = compute_distance(u, v);
                if (dist < min_dist) {
                    min_dist = dist;
                    from = u;
                    to = v;
                }
            }
        }

        if (to != -1) {
            this -> mst_edges.emplace_back(from, to);
            in_mst.insert(to);
            remaining.erase(to);
        }
    }
}

vector<pair<int, int>> christofides::get_mst_edges() {
    return this -> mst_edges;
}

vector<mcpm_node>& christofides::odd_indices() {
    // 노드 ID는 실제 ID 기준으로 degree를 관리
    unordered_map<int, int> degrees; // index, num
    for (const auto& node : this -> nodes) {
        degrees[node.first] = 0;
    }

    for (const auto& edge : this->mst_edges) {
        degrees[edge.first] += 1;
        degrees[edge.second] += 1;
    }
    
    int idx_at_odd = 1; // 1부터 담을것것
    for (const auto& d : degrees) {
        if (d.second % 2 == 1) {
            mcpm_node v(idx_at_odd, d.first, this -> nodes[d.first].first, this -> nodes[d.first].second);
            this -> oddIndices.push_back(v);
        }
    }

    return this -> oddIndices;
}

void christofides::mcpm() {
    blossomV minimum_cost_perfect_matching(this -> oddIndices);
    minimum_cost_perfect_matching.execute_all();
}



