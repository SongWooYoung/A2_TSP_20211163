#include "Held_Karp.h"
#include <cmath>
#include <algorithm>

#define MAX 1e12

using namespace std;

void HK_TSP::build_weight(const map<int, pair<double, double>>& nodes) {
    n = nodes.size();
    weight.resize(n, vector<double>(n, 0.0));
    vector<pair<double, double>> coords(n);

    for (const auto& node : nodes) {
        coords[node.first - 1] = node.second;
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == j) continue;
            double dx = coords[i].first - coords[j].first;
            double dy = coords[i].second - coords[j].second;
            weight[i][j] = sqrt(dx * dx + dy * dy);
        }
    }
}

HK_TSP::HK_TSP(const map<int, pair<double, double>>& nodes) {
    build_weight(nodes);
    dp.resize((1 << n), vector<double>(n, MAX));
    parent.resize((1 << n), vector<int>(n, -1));  // 초기화
}

void HK_TSP::solve() {
    int total_subset = (1 << n);
    dp[1][0] = 0.0;

    for (int mask = 1; mask < total_subset; ++mask) {
        for (int present_city = 0; present_city < n; ++present_city) {
            if (!(mask & (1 << present_city))) continue;

            for (int next_city = 0; next_city < n; ++next_city) {
                if ((mask & (1 << next_city)) || present_city == next_city) continue;

                int next_mask = mask | (1 << next_city);
                double cost = dp[mask][present_city] + weight[present_city][next_city];

                if (cost < dp[next_mask][next_city]) {
                    dp[next_mask][next_city] = cost;
                    parent[next_mask][next_city] = present_city;  // 경로 저장
                }
            }
        }
    }
}

double HK_TSP::min_cost() {
    int total_subset = (1 << n);
    double min_total_cost = MAX;
    int last_city = -1;
    for (int j = 1; j < n; ++j) {
        double cost = dp[total_subset - 1][j] + weight[j][0];
        if (cost < min_total_cost) {
            min_total_cost = cost;
            last_city = j;
        }
    }

    // 복원을 위해 마지막 도시 저장
    parent[total_subset - 1][0] = last_city;
    return min_total_cost;
}

vector<int> HK_TSP::get_path() {
    vector<int> path;

    int mask = (1 << n) - 1;

    // 최종 도시는 parent[mask][0]에 저장되어 있음
    int current = parent[mask][0];
    path.push_back(0);         // 시작점 (city 0)
    path.push_back(current);   // 마지막 도시

    while (current != 0) {
        int prev = parent[mask][current];
        mask ^= (1 << current);  // 현재 도시 제거
        current = prev;
        path.push_back(current);
    }

    reverse(path.begin(), path.end());
    return path;
}


