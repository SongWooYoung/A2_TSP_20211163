#include "approx2.h"

using namespace std;


approx2::approx2(const map<int, pair<double, double>>& nodes) : nodes(nodes) {

}

void approx2::execute_all() {
    this -> compute_mst();
    this -> change_container();
    this -> preorder_DFS();

    this -> print_total_length();
    this -> print_path();
}

double approx2::compute_distance(int u, int v) {
    auto [x1, y1] = this -> nodes[u];
    auto [x2, y2] = this -> nodes[v];
    double dx = x1 - x2;
    double dy = y1 - y2;
    return (sqrt(dx * dx + dy * dy));
}

void approx2::compute_mst() {
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
            this -> mst_edges.emplace_back(from, to, min_dist);
            in_mst.insert(to);
            remaining.erase(to);
        }
    }
}

void approx2::change_container() {
    for (auto& [u, v, w] : this -> mst_edges) {
        this -> new_container[u].emplace_back(w, v);
        this -> new_container[v].emplace_back(w, u);
    }
    
    // 가중치 기준 정렬
    for (auto& [u, neighbors] : this -> new_container)
        sort(neighbors.begin(), neighbors.end());
}

void approx2::preorder_DFS() {
    vector<bool> visited(this -> nodes.size() + 1, false);
    stack<int> st;
    st.push(1);

    while (!st.empty()) {
        int cur = st.top();
        st.pop();

        if (visited[cur]) continue;
        visited[cur] = true;
        this -> TSP_path.push_back(cur);

        // 역순으로 넣어야 작은 weight 먼저 pop 됨 (vector는 정렬되어 있다고 가정)
        for (auto it = new_container[cur].rbegin(); it != new_container[cur].rend(); ++it) {
            if (!visited[it->second]) {
                st.push(it->second);
            }
        }
    }

    this -> TSP_path.push_back(1); // 다시 시작점으로 복귀
}

void approx2::print_total_length() {
    double total = 0;
    for (int i = 0; i < (int) TSP_path.size()-1; i++) {
       total += compute_distance(this -> TSP_path[i]-1 , this -> TSP_path[i+1]-1);
    }
    cout << "TOTAL LENGTH: " << total << endl;
}

void approx2::print_path() {
    cout << "2-approximation path: " << endl;
    cout << "[ ";
    for (int i: this -> TSP_path) {
        cout << i << " ";
    }
    cout << "]" << endl;
}
