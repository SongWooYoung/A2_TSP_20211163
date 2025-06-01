#include "approx2.h"

using namespace std;


approx2::approx2(const map<int, pair<double, double>>& nodes) : nodes(nodes) {

}

#include <chrono>

void approx2::execute_all() {
    // using namespace std::chrono;

    // auto start = high_resolution_clock::now();

    // // 1. MST 생성
    // auto t1 = high_resolution_clock::now();
    this->compute_mst();
    // auto t2 = high_resolution_clock::now();
    // cout << "MST DONE in "
    //      << duration_cast<duration<double>>(t2 - t1).count()
    //      << " seconds" << endl;

    // 2. 컨테이너 생성
    // t1 = high_resolution_clock::now();
    this->change_container();
    // t2 = high_resolution_clock::now();
    // cout << "Change Container DONE in "
    //      << duration_cast<duration<double>>(t2 - t1).count()
    //      << " seconds" << endl;

    // 3. DFS
    // t1 = high_resolution_clock::now();
    this->preorder_DFS();
    // t2 = high_resolution_clock::now();
    // cout << "DFS DONE in "
    //      << duration_cast<duration<double>>(t2 - t1).count()
    //      << " seconds" << endl;

    // 4. 거리 계산 및 출력
    // t1 = high_resolution_clock::now();
    this->print_path();
    this->print_total_length();
    // t2 = high_resolution_clock::now();
    // cout << "Print DONE in "
    //      << duration_cast<duration<double>>(t2 - t1).count()
    //      << " seconds" << endl;

    // auto end = high_resolution_clock::now();
    // cout << "TOTAL EXECUTION TIME: "
    //      << duration_cast<duration<double>>(end - start).count()
    //      << " seconds" << endl;
}


double approx2::compute_distance(int u, int v) {
    auto [x1, y1] = this -> nodes[u];
    auto [x2, y2] = this -> nodes[v];
    double dx = x1 - x2;
    double dy = y1 - y2;
    return sqrt(dx * dx + dy * dy);
}


void approx2::compute_mst() {
    if (this->nodes.empty()) return;

    int n = this->nodes.size();
    vector<bool> in_mst(n + 1, false);         // id: 1 ~ n
    vector<double> min_dist(n + 1, INF);
    vector<int> parent(n + 1, -1);

    // Min-heap: pair<dist, id>
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;

    min_dist[1] = 0;
    pq.emplace(0.0, 1);  // Start from node 1

    while (!pq.empty()) {
        auto [dist_u, u] = pq.top(); pq.pop();
        if (in_mst[u]) continue;
        in_mst[u] = true;

        if (parent[u] != -1) {
            double w = compute_distance(u, parent[u]);
            this->mst_edges.emplace_back(u, parent[u], w);
        }

        for (int v = 1; v <= n; ++v) {
            if (!in_mst[v]) {
                double d = compute_distance(u, v);
                if (d < min_dist[v]) {
                    min_dist[v] = d;
                    parent[v] = u;
                    pq.emplace(d, v);
                }
            }
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
