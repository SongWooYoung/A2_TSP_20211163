#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <set>
#include <limits>
#include <algorithm>
#include <chrono>
#include <sstream>

using namespace std;
using namespace std::chrono;

typedef pair<double, double> Point;
typedef vector<Point> PointList;
typedef vector<int> Path;

// 거리 계산
double euclidean(const Point& a, const Point& b) {
    return sqrt((a.first - b.first)*(a.first - b.first) + (a.second - b.second)*(a.second - b.second));
}

// TSP 파일 파싱
PointList parse_tsp_file(const string& filename) {
    ifstream fin(filename);
    if (!fin) {
        cerr << "❌ 파일 열기 실패: " << filename << endl;
        return {};
    }

    string line;
    PointList coords;
    bool reading = false;

    while (getline(fin, line)) {
        if (line.find("NODE_COORD_SECTION") != string::npos) {
            reading = true;
            continue;
        }
        if (line.find("EOF") != string::npos) break;

        if (reading) {
            istringstream iss(line); // 공백 유지
            int idx;
            double x, y;
            if (iss >> idx >> x >> y) {
                coords.emplace_back(x, y);
            } else {
                cerr << "⚠️ 좌표 파싱 실패: " << line << endl;
            }
        }
    }

    cerr << "✅ 총 파싱된 점 개수: " << coords.size() << endl;
    return coords;
}

// MST (Prim’s Algorithm)
set<pair<int, int>> generate_mst(const PointList& points) {
    int n = points.size();
    vector<bool> visited(n, false);
    vector<double> min_dist(n, numeric_limits<double>::infinity());
    vector<int> parent(n, -1);
    set<pair<int, int>> mst_edges;

    min_dist[0] = 0;

    for (int i = 0; i < n; ++i) {
        int u = -1;
        double min_val = numeric_limits<double>::infinity();
        for (int j = 0; j < n; ++j) {
            if (!visited[j] && min_dist[j] < min_val) {
                min_val = min_dist[j];
                u = j;
            }
        }

        if (u == -1) break;
        visited[u] = true;
        if (parent[u] != -1) {
            mst_edges.emplace(min(u, parent[u]), max(u, parent[u]));
        }

        for (int v = 0; v < n; ++v) {
            if (!visited[v]) {
                double d = euclidean(points[u], points[v]);
                if (d < min_dist[v]) {
                    min_dist[v] = d;
                    parent[v] = u;
                }
            }
        }
    }

    return mst_edges;
}

// VCG + MST 기반 TSP 경로 생성
Path vcg_mst_tsp(const PointList& points, double alpha = 1.0, double beta = 1.0, double gamma = 2.0) {
    int n = points.size();
    vector<bool> visited(n, false);
    Path path;

    // 중심 좌표 계산
    Point C_global = {0.0, 0.0};
    for (const auto& p : points) {
        C_global.first += p.first;
        C_global.second += p.second;
    }
    C_global.first /= n;
    C_global.second /= n;

    // MST 생성 및 시간 측정
    auto start_time = high_resolution_clock::now();
    set<pair<int, int>> mst_edges = generate_mst(points);
    auto end_time = high_resolution_clock::now();
    cout << "MST 생성 시간: " << duration<double>(end_time - start_time).count() << "초" << endl;

    // 시작점: 중심에 가장 가까운 점
    int start = 0;
    double min_dist = numeric_limits<double>::infinity();
    for (int i = 0; i < n; ++i) {
        double d = euclidean(points[i], C_global);
        if (d < min_dist) {
            min_dist = d;
            start = i;
        }
    }

    int curr = start;
    path.push_back(curr);
    visited[curr] = true;

    for (int step = 0; step < n - 1; ++step) {
        Point curr_point = points[curr];

        // 중심 방향 단위 벡터
        double dx = C_global.first - curr_point.first;
        double dy = C_global.second - curr_point.second;
        double norm_cd = hypot(dx, dy) + 1e-8;
        Point center_dir = {dx / norm_cd, dy / norm_cd};

        // 후보 점 평가
        vector<pair<double, int>> candidates;
        for (int i = 0; i < n; ++i) {
            if (visited[i]) continue;

            double dir_x = points[i].first - curr_point.first;
            double dir_y = points[i].second - curr_point.second;
            double dist = euclidean(curr_point, points[i]);
            double norm_dir = hypot(dir_x, dir_y) + 1e-8;
            Point unit_dir = {dir_x / norm_dir, dir_y / norm_dir};

            double dot = unit_dir.first * center_dir.first + unit_dir.second * center_dir.second;

            int u = min(curr, i);
            int v = max(curr, i);
            double mst_bonus = (mst_edges.count({u, v}) > 0) ? gamma : 0.0;

            double score = alpha * dot - beta * dist + mst_bonus;
            candidates.emplace_back(score, i);
        }

        // 최고 점수 후보 선택
        auto best = max_element(candidates.begin(), candidates.end());
        int next_node = best->second;
        path.push_back(next_node);
        visited[next_node] = true;
        curr = next_node;
    }

    path.push_back(start);  // 순환 경로 마무리
    return path;
}


// 경로 길이 계산
double total_path_length(const Path& path, const PointList& points) {
    double total = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        total += euclidean(points[path[i]], points[path[i + 1]]);
    }
    return total;
}

// --- Main ---
int main() {
    string filename = "mona-lisa100K.tsp";  // 여기 파일명을 입력하세요
    auto points = parse_tsp_file(filename);

    double best_length = numeric_limits<double>::infinity();
    Path best_path;
    tuple<double, double, double> best_params;

    vector<double> alphas = {/*0.1, 0.2, 0.3, 0.4, */0.5};
    vector<double> betas  = {/*0.5, 1.0, 1.5, */ 2.0};
    vector<double> gammas = {0.0/*1.0, 2.0*/};

    for (double alpha : alphas) {
        for (double beta : betas) {
            for (double gamma : gammas) {
                auto start = high_resolution_clock::now();
                Path path = vcg_mst_tsp(points, alpha, beta, gamma);
                auto end = high_resolution_clock::now();

                double length = total_path_length(path, points);
                cout << "alpha=" << alpha << ", beta=" << beta << ", gamma=" << gamma
                     << ", 길이=" << length << ", 시간=" << duration<double>(end - start).count() << "초" << endl;

                if (length < best_length) {
                    best_length = length;
                    best_path = path;
                    best_params = {alpha, beta, gamma};
                }
            }
        }
    }

    auto [a, b, g] = best_params;
    cout << "\n✅ 최적 결과: alpha=" << a << ", beta=" << b << ", gamma=" << g << ", 길이=" << best_length << endl;
    cout << "✅ 총 점 개수: " << points.size() << ", 유일한 경로 점 수: " << set<int>(best_path.begin(), best_path.end()).size() << endl;
    cout << "✅ 순환 여부: " << (best_path.front() == best_path.back() ? "✔️" : "❌") << endl;

    return 0;
}
