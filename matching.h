#ifndef MATCHING_H
#define MATCHING_H

#include <vector>
#include <list>
#include <queue>
#include <utility>
#include <cmath>
#include <vector>
#include <queue>
#include <cmath>

#define INT_MAX 1e12

using mcpm_node_idx = int;
constexpr double EPS = 1e-6;

enum NodeType { UNLABELED = 0, ODD = 1, EVEN = 2 };

// 단일 노드 또는 Blossom의 대표 노드로 사용되는 구조체
struct mcpm_node {
    mcpm_node_idx index_at_oddIndices;
    int index_at_nodes;  // 원래 노드 인덱스
    int x, y;            // 좌표 정보

    NodeType type;
    mcpm_node_idx root;
    mcpm_node_idx parent;
    mcpm_node_idx pair;

    bool visited;
    bool active;
    bool is_blossom;
    double dual_value;

    mcpm_node(int idx_odd, int idx_nodes, int x_coord, int y_coord)
        : index_at_oddIndices(idx_odd), index_at_nodes(idx_nodes), x(x_coord), y(y_coord),
          type(UNLABELED), root(idx_odd), parent(-1), pair(-1),
          visited(false), active(true), is_blossom(false), dual_value(0.0) {}

    void init_with_out_pair() {
        type = UNLABELED;
        root = index_at_oddIndices;
        parent = -1;
        visited = false;
        active = true;
    }

    void init() {
        init_with_out_pair();
        pair = -1;
    }
};

class blossomV {
public:
    // 초기 노드 수
    int origin_num;
    int nl_size;

    // 노드 및 그래프 구조
    std::vector<mcpm_node>& node_list;                // 모든 노드와 blossom 노드 포함
    std::vector<mcpm_node_idx> outer;                // 현재 outer representative
    std::vector<std::pair<int, int>> edge_list;      // 간선 리스트 (u, v)
    //std::vector<double> slack;                       // 간선 slack 값

    // Blossom 구조
    std::vector<std::list<mcpm_node_idx>> blo_tree;  // 각 shallow node의 deep 리스트
    std::vector<std::list<mcpm_node_idx>> blo_path;  // shallow 경로
    std::vector<mcpm_node_idx> tip;                  // Blossom의 tip node

    // 탐색 상태
    std::queue<mcpm_node_idx> grow_queue;
    std::queue<mcpm_node_idx> free_blossoms;
    bool perfect;

    // 생성자
    blossomV(std::vector<mcpm_node>& node_list);

    // Blossom V 알고리즘 주요 함수
    std::pair<std::list<int>, double> SolveMinimumCostPerfectMatching(const std::vector<double>& cost);
    std::list<int> RetrieveMatching();

    // 알고리즘 단계별 함수
    void Heuristic();
    void Grow();
    void Expand(mcpm_node_idx u, bool expandBlocked = false);
    void Augment(mcpm_node_idx u, mcpm_node_idx v);
    mcpm_node_idx Blossom(mcpm_node_idx u, mcpm_node_idx v);
    void UpdateDualCosts();
    void Reset();
    void Clear();
    void DestroyBlossom(mcpm_node_idx t);
    void Heuristic();
    std::pair<std::list<int>, double> SolveMinimumCostPerfectMatching(const std::vector<double>& cost);

    // 유틸리티
    void return_free_blossom(mcpm_node_idx i);
    mcpm_node_idx get_free_blossom();
    double cal_slack(mcpm_node_idx u, mcpm_node_idx v) const;
    bool is_tight(mcpm_node_idx u, mcpm_node_idx v) const;
};

#endif