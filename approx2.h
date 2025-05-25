#ifndef APPROX2_H
#define APPROX2_H

#include <vector>
#include <iostream>
#include <map>
#include <set>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <stack>

#define INF 1e12


class approx2 {
    private:
        std::map<int, std::pair<double, double>> nodes;
        std::vector<std::tuple<int, int, double>> mst_edges; //  x, y, weight
        std::unordered_map<int, std::vector<std::pair<double, int>>> new_container;
        std::vector<int> TSP_path;

    public:
        approx2(const std::map<int, std::pair<double, double>>& nodes);
    
        // 유틸리티
        double compute_distance(int u, int v);

        // process
        void compute_mst();
        void change_container();
        void preorder_DFS();

        void print_total_length();
        void print_path();
    
        // total process
        void execute_all();
    };
    



#endif