#ifndef HELD_KARP_H
#define HELD_KARP_H

#include <vector>
#include <map>
#include <utility>

class HK_TSP {
private:
    int n;
    std::vector<std::vector<double>> weight;
    std::vector<std::vector<double>> dp;
    std::vector<std::vector<int>> parent;

    void build_weight(const std::map<int, std::pair<double, double>>& nodes);

public:
    HK_TSP(const std::map<int, std::pair<double, double>>& nodes);

    void solve();
    double min_cost();

    // 경로 복원
    std::vector<int> get_path();
};

#endif
