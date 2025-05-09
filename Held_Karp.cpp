#include "Held_Karp.h"
#include <algorithm>

using namespace std;

#define MAX 1e12

HK_TSP::HK_TSP(const vector<vector<double>>& weight) {
    this -> weight = weight;
    this -> n = weight.size();
    dp.resize((1 << this -> get_n()), vector<double>(n, MAX));
}

int HK_TSP::get_n() {
    return this -> n;
}
int HK_TSP::get_dp_size() {
    return this -> dp.size();
}

void HK_TSP::solve() {
    int total_subset = (1 << this->n);
    this -> dp[1] = vector<double>(this -> n, MAX);
    this -> dp[1][0] = 0.0;

    for (int mask = 1; mask < total_subset; ++mask) {
        // Ensure dp[mask] exists
        if (dp[mask].size() == 0) {
            dp[mask] = vector<double>(n, MAX);
        }

        for (int present_city = 0; present_city < this -> n; ++present_city) {
            if (!(mask & (1 << present_city))) continue;

            for (int next_city = 0; next_city < this -> n; ++next_city) {
                if ((mask & (1 << next_city)) || present_city == next_city) continue;

                int next_mask = mask | (1 << next_city);

                // Initialize dp[next_mask] if it doesn't exist
                if (dp[next_mask].size() == 0) {
                    dp[next_mask] = vector<double>(this -> n, MAX);
                }

                double cost = dp[mask][present_city] + weight[present_city][next_city];
                dp[next_mask][next_city] = min(dp[next_mask][next_city], cost);
            }
        }
    }
}


double HK_TSP::min_cost() {
    // lowest cost
    int total_subset = (1 << this -> n);
    double min_cost = MAX;
    for (int j = 1; j < n; ++j) {
        min_cost = min(min_cost, this -> dp[total_subset - 1][j] + weight[j][0]);
    }
    return min_cost;
}
