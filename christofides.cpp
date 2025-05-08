#include <iostream>
#include "conv2list.h"
#include <queue>

using namespace std;

struct xyw{
    int x;
    int y;
    double weight;

    bool operator<(const xyw& other) const {
        return this -> weight > other.weight;
    }
};

int main() {

    XmlToTspConverter a280T("a280.xml", "a280.tsp", 280);
    a280T.convert();
    TspParser a280TSP("a280.tsp", 280);
    a280TSP.parse();
    //a280TSP.print_matrix(5);

    vector<vector<double>> weight = a280TSP.get_matrix();

    // 1. make MST, M
    // Prim algorithm
    // 1) push all weights of edge that a vertex v_i holds to pq
    // 2) comparing all the weights in pq, choose least edge weight that does not make cycle and "make visit to the vertex"

    vector<bool> is_visited(weight.size(), false);              // whether the vertex is visited or not
    vector<double> key(weight.size(), MAX);                     // minimum cost of connection of two vertices
    vector<int> parent(weight.size(), -1);                      // parent of each connection
    priority_queue<xyw, vector<xyw>> pq;                        // Min-Heap by weight

    key[0] = 0;
    pq.push({-1, 0, 0}); // 시작점 (parent 없음)

    int edge_count = 0;
    vector<xyw> mst_edges; // MST 간선 저장

    while (!pq.empty() && edge_count < weight.size() - 1) {
        xyw curr = pq.top(); pq.pop();
        int u = curr.y;

        if (is_visited[u]) continue;
        is_visited[u] = true;

        if (curr.x != -1) { // 첫 노드는 제외
            mst_edges.push_back(curr);
            edge_count++;
        }

        for (int v = 0; v < (int) weight.size(); ++v) {
            if (!is_visited[v] && weight[u][v] < key[v]) {
                key[v] = weight[u][v];
                parent[v] = u;
                pq.push({u, v, weight[u][v]});
            }
        }
    }

    // // 결과 출력
    // cout << "MST edges (total " << mst_edges.size() << "):\n";
    // int num = 0;
    // for (auto& e : mst_edges) {
    //     cout << num << ". x: " << e.x << ", y: " << e.y << ", weight: " << e.weight << "\n";
    //     num++;
    // }

    // if (mst_edges.size() == weight.size() - 1) {
    //     cout << "MST edge count is correct: 279\n";
    // } else {
    //     cout << "MST edge count incorrect! Got: " << mst_edges.size() << "\n";
    // }
    // 2-1. make a set W of vertices that has odd degree in MST
    // 2-2. make a complete graph H connecting to each other vertex in W
    // 2-3. compute minimun_cost_perfect_matching, P.


    // 3-1. Merge P and M-> G'
    // 3-2. let all duplicated weight edges alive
    
    // 4. Create Euclidean circuit C in G', 

    // 5. Convert C into a tour T.

    return 0;

}

