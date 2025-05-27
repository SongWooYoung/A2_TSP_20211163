#include "approx2.h"

using namespace std;


approx2::approx2(const map<int, pair<double, double>>& nodes) : nodes(nodes) {

}

#include <chrono>

void approx2::execute_all() {
    using namespace std::chrono;

    auto start = high_resolution_clock::now();

    // 1. MST 생성
    auto t1 = high_resolution_clock::now();
    this->compute_mst();
    auto t2 = high_resolution_clock::now();
    cout << "MST DONE in "
         << duration_cast<duration<double>>(t2 - t1).count()
         << " seconds" << endl;

    // 2. 컨테이너 생성
    t1 = high_resolution_clock::now();
    this->change_container();
    t2 = high_resolution_clock::now();
    cout << "Change Container DONE in "
         << duration_cast<duration<double>>(t2 - t1).count()
         << " seconds" << endl;

    // 3. DFS
    t1 = high_resolution_clock::now();
    this->preorder_DFS();
    t2 = high_resolution_clock::now();
    cout << "DFS DONE in "
         << duration_cast<duration<double>>(t2 - t1).count()
         << " seconds" << endl;

    // 4. 거리 계산 및 출력
    t1 = high_resolution_clock::now();
    this->print_total_length();
    this->print_path();
    t2 = high_resolution_clock::now();
    cout << "Print DONE in "
         << duration_cast<duration<double>>(t2 - t1).count()
         << " seconds" << endl;

    auto end = high_resolution_clock::now();
    cout << "TOTAL EXECUTION TIME: "
         << duration_cast<duration<double>>(end - start).count()
         << " seconds" << endl;
}


double approx2::compute_distance(int u, int v) {
    auto [x1, y1] = this -> nodes[u];
    auto [x2, y2] = this -> nodes[v];
    double dx = x1 - x2;
    double dy = y1 - y2;
    return (int) (sqrt(dx * dx + dy * dy) + 0.5);
}


// void approx2::compute_mst() {
//     if (this->nodes.empty()) return;

//     int n = this->nodes.size();
//     vector<bool> in_mst(n + 1, false); // index 1-based
//     vector<double> min_dist(n + 1, INF);
//     vector<int> parent(n + 1, -1);

//     auto cmp = [](const pair<double, int>& a, const pair<double, int>& b) {
//         return a.first > b.first; // min-heap
//     };

//     priority_queue<pair<double, int>, vector<pair<double, int>>, decltype(cmp)> pq(cmp);

//     int start = this->nodes.begin()->first;
//     min_dist[start] = 0;
//     pq.emplace(0.0, start);

//     while (!pq.empty()) {
//         auto [cur_dist, u] = pq.top(); pq.pop();

//         if (in_mst[u]) continue;
//         in_mst[u] = true;

//         if (parent[u] != -1) {
//             double w = compute_distance(u, parent[u]);
//             this->mst_edges.emplace_back(u, parent[u], w);
//         }

//         for (const auto& [v_idx, v_coord] : this->nodes) {
//             if (!in_mst[v_idx]) {
//                 double d = compute_distance(u, v_idx);
//                 if (d < min_dist[v_idx]) {
//                     min_dist[v_idx] = d;
//                     parent[v_idx] = u;
//                     pq.emplace(d, v_idx);
//                 }
//             }
//         }
//     }
// }

void approx2::compute_mst() {
    int V = nodes.size();
    vector<int> node_index;  // 노드 번호를 0-based index로 매핑
    unordered_map<int, int> index_map;  // 노드번호 → index
    int idx = 0;
    for (const auto& [id, _] : nodes) {
        node_index.push_back(id);
        index_map[id] = idx++;
    }

    vector<bool> in_mst(V, false);
    vector<double> key(V, INF);
    vector<int> parent(V, -1);  // index 기준

    key[0] = 0;

    for (int count = 0; count < V; count++) {
        // 1. Find the minimum key vertex
        int u = -1;
        for (int i = 0; i < V; i++) {
            if (!in_mst[i] && (u == -1 || key[i] < key[u])) {
                u = i;
            }
        }

        in_mst[u] = true;

        // 2. Update the keys of adjacent vertices
        for (int v = 0; v < V; v++) {
            if (!in_mst[v]) {
                int u_id = node_index[u];
                int v_id = node_index[v];
                double weight = compute_distance(u_id, v_id);
                if (weight < key[v]) {
                    key[v] = weight;
                    parent[v] = u;
                }
            }
        }
    }

    // 3. Reconstruct mst_edges
    for (int v = 1; v < V; v++) {
        int u = parent[v];
        int u_id = node_index[u];
        int v_id = node_index[v];
        double w = compute_distance(u_id, v_id);
        this->mst_edges.emplace_back(u_id, v_id, w);
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
       total += compute_distance(this -> TSP_path[i], this -> TSP_path[i+1]);
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
