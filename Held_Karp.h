#ifndef HELD_KARP_H
#define HELD_KARP_H

#include <vector>
#include <iostream>
#include <unordered_map>

class HK_TSP {
private:
    std::vector<std::vector<double>> weight;
    int n;
    std::vector<std::vector<double>> dp; // <maksed_path 2^n, n cities (last city)>
    // -> it is really hard to maintain 2^n by 2^n list by vector<vector>
public:
    HK_TSP(const std::vector<std::vector<double>>& weight);
    int get_n();
    int get_dp_size();
    void solve();
    double min_cost();
};

#endif