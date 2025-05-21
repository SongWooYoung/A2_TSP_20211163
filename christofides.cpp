#include "christofides.h"
#include "mcpm.h"


using namespace std;


double christofides::compute_distance(int u, int v) {
    auto [x1, y1] = nodes[u];
    auto [x2, y2] = nodes[v];
    double dx = x1 - x2;
    double dy = y1 - y2;
    return sqrt(dx * dx + dy * dy);
}

christofides::christofides(const map<int, pair<double, double>>& nodes) {
    this -> nodes = nodes;
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
            this -> mst_edges.emplace_back(from, to, min_dist);
            in_mst.insert(to);
            remaining.erase(to);
        }
    }
}

vector<tuple<int, int, double>> christofides::get_mst_edges() {
    return this -> mst_edges;
}

vector<mcpm_node>& christofides::odd_indices() {
    // 노드 ID는 실제 ID 기준으로 degree를 관리
    unordered_map<int, int> degrees; // index, num
    for (const auto& node : this -> nodes) {
        degrees[node.first] = 0;
    }

    for (const auto& edge : this -> mst_edges) {
        degrees[get<0>(edge)] += 1;
        degrees[get<1>(edge)] += 1;
    }
    
    int idx_at_odd = 0; // 1부터 담을것것
    for (const auto& d : degrees) {
        if (d.second % 2 == 1) {
            mcpm_node v(idx_at_odd, d.first, this -> nodes[d.first].first, this -> nodes[d.first].second);
            this -> oddIndices.push_back(v);
            idx_at_odd++;
        }
    }

    return this -> oddIndices;
}

void christofides::print_oddIndices() {
    cout << endl;
    cout << "ODD INDICES: " << endl;
    for (mcpm_node n: this -> oddIndices) {
        cout << "index: " << n.index_at_oddIndices << endl;
        cout << "Name: " << n.index_at_nodes << ", X: " << n.x << ", Y: " << n.y << endl;
        cout << "type: " << n.type << ", parent: " << n.parent << ", pair: " << n.pair << ", dual value: " << n.dual_value << endl; 
        cout << "------------------------------" << endl;
    }
}

void christofides::mcpm() {
    blossomV minimum_cost_perfect_matching(this -> oddIndices); 
    minimum_cost_perfect_matching.execute_all();
    minimum_cost_perfect_matching.merge_pairs(this -> mst_edges);
}


void christofides::euler_tour() {

    // 1.  adjacency matrix
    // 2차원 벡터를 쓰려고 했지만, 공간 낭비가 너무 심해서 map -> unordered_map으로 결정
    // https://stackoverflow.com/questions/70601822/in-practice-when-stdunordered-map-must-be-used-instead-of-stdmap
    for (const auto& [u, v, w] : this -> mst_edges) {
        this -> edges_ET[u][v] = w;
        this -> edges_ET[v][u] = w;
    }

    // 2. Hierholzer
    stack<int> stack;

    // 적절한 시작 정점 선택
    if (this -> mst_edges.empty()) return;  // => 혹시 몰라서 구색 맞추기 용용
    int start = get<0>(this -> mst_edges[0]);
    stack.push(start);

    while (!stack.empty()) {
        int u = stack.top();

        if (!this -> edges_ET[u].empty()) {
            // 연결된 가장 먼저 나오는 정점 하나 선택
            auto it = this -> edges_ET[u].begin(); 
            int v = it -> first;

            // 간선 제거 (무방향이므로 양방향 모두 삭제)
            this -> edges_ET[u].erase(it);
            this -> edges_ET[v].erase(u);
            stack.push(v);

        } else {
            // 더 이상 연결 간선이 없으면 경로에 추가
            this -> path.emplace_back(u);
            stack.pop();
        }
    }

    // 결과 경로 저장 (Euler Tour는 역순으로 만들어지므로 뒤집기)
    reverse(this -> path.begin(), this -> path.end());
}

void christofides::erase_dups() {
    vector<int> tsp_path;
    unordered_set<int> visited;
    for (int u: this -> path) {
        // 한번이라도 지난 정점이면 추가 x
        if (visited.count(u)) continue;
        visited.insert(u);
        tsp_path.push_back(u);
    }
    if (tsp_path.front() != tsp_path.back()) {
        tsp_path.push_back(tsp_path.front());
    }
    this -> path = move(tsp_path);
}

double christofides::total_dist(){
    double total = 0.0;
    for (int i = 0; i + 1 < (int) this -> path.size(); ++i) {
        total += compute_distance(this -> path[i], this -> path[i + 1]);
    }
    total += compute_distance(this -> path.back(), this -> path[0]); // 돌아오는 길 포함
    return total;
}

void christofides::print_path() {
    cout << "TSP path!" << endl;
    for (int i :  this -> path) {
        cout << i << " ";
    }
    cout << endl;
}

void christofides::execute_all() {
    this -> compute_mst();
    this -> odd_indices();
    this -> mcpm();
    this -> euler_tour();
    this -> erase_dups();
    this -> print_path();
    cout << "TOTAL LENGTH: " << this -> total_dist() << endl;
}
