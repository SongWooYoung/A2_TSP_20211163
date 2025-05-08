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

    vector<xyw> pq;                 // save the weight value and its coordinate
    priority_queue<xyw> vi_edges;           // save the vi_edges
    bool is_visited[280];                   // dealing with the visited vertices

    for (int i = 0; i < 280; i++) {
        is_visited[i] = false;
        if(!is_visited[i]) vi_edges.push({0, i, weight[0][i]});
    }
    is_visited[0] = true;

    while (!vi_edges.empty()) {
        // delete unnecessary edges
        while (!vi_edges.empty() && is_visited[vi_edges.top().y]) vi_edges.pop();
        if (vi_edges.empty()) break;
    
        xyw edge = vi_edges.top(); 
        vi_edges.pop();

        int y = edge.y;
    
        is_visited[y] = true;
        pq.push_back(edge); // save the mst result
    
        for (int i = 0; i < 280; ++i) {
            if (!is_visited[i]) {
                vi_edges.push({y, i, weight[y][i]});
            }
        }
    }
    cout << "MST edges (x, y, weight):\n";
    for (int i = 0; i < (int) pq.size(); i++) {
        xyw edge = pq[i]; 
        cout << i << ". x: " << edge.x << ", y: " << edge.y << ", weight: " << edge.weight << "\n";
    }

    // 2-1. make a set W of vertices that has odd degree in MST
    // 2-2. make a complete graph H connecting to each other vertex in W
    // 2-3. compute minimun_cost_perfect_matching, P.


    // 3-1. Merge P and M-> G'
    // 3-2. let all duplicated weight edges alive
    
    // 4. Create Euclidean circuit C in G', 

    // 5. Convert C into a tour T.

    return 0;

}

