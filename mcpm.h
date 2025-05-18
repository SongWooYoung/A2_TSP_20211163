#ifndef MCPM_H
#define MCPM_H

#include <vector>
#include <queue>
#include <list>
#include <cmath>
#include <utility>
#include <limits>
#include <cassert>

typedef int mcpm_node_idx;

enum NodeType {
    UNLABELED = 0,
    EVEN = -1,
    ODD = 1
};

struct mcpm_node {
    // 기본 정보
    mcpm_node_idx index_at_oddIndices;  // Working set에서의 인덱스 -> 얘가 mcpm_node_idx
    int index_at_nodes;                 // 실제 그래프 인덱스
    int x, y;                           // 위치 정보 (거리 계산용)

    // 상태 정보
    NodeType type;                      // EVEN, ODD, UNLABELED
    mcpm_node_idx root;                 // 소속 트리의 루트
    mcpm_node_idx parent;               // 트리에서의 부모

    mcpm_node_idx pair;                 // 매칭된 노드
    bool visited;                       // Grow 중 방문 여부
    bool active;                        // 활성 상태 (blossom 포함)

    // Blossom 관련
    bool is_blossom;               
    //std::vector<mcpm_node_idx>* blossom_members; // 축약된 노드들의 목록

    // Dual 값
    double dual_value;

    mcpm_node(int idx_odd, int idx_nodes, int x, int y);
    mcpm_node& operator=(mcpm_node& other);
    void init_with_out_pair();                    // 상태 초기화
    void init();
};

class blossomV {
private:
    std::vector<mcpm_node>& node_list;   // 외부에서 주입받는 노드 리스트 (2n 크기)
    int origin_num;                      // 실제 노드 수 (n)
    int nl_size;                         // 전체 노드 리스트 크기 (2n)
    int matching_num;                    // 현재 매칭 수

    std::vector<mcpm_node_idx> root;                        // 해당 index가 어떤 root의 alternating tree에 존재하는지
    std::vector<std::list<mcpm_node_idx>> internal_node;    // 각 blossom의 내부 노드들
    std::vector<std::list<mcpm_node_idx>> tree;             // shrink 시 shallow 회로
    std::vector<mcpm_node_idx> tip;                         // 각 blossom의 tip (== LCA of shrinked cycle)
    std::vector<mcpm_node_idx> free_blossom_indices;        // 사용할 수 있는 가짜 노드 index
    std::queue<mcpm_node_idx> grow_queue;
    std::vector<double> slack;                              // edge별 slack 값 (거리 기반)
    
public:
    blossomV(std::vector<mcpm_node>& list);

    // 전체 알고리즘 실행
    void execute_all();

    // 초기 Heuristic matching
    void Heuristic(); // greedy 하게 탐색 후 시작 -> 시간 복잡도 감소

    // Grow 단계
    void Grow();
    void init_without_pair();

    // Matching 및 Tree 조작
    void matching(mcpm_node_idx u, mcpm_node_idx v);
    void flip(mcpm_node_idx u);
    void Augment(mcpm_node_idx u, mcpm_node_idx v);
    mcpm_node_idx Shrink(mcpm_node_idx u, mcpm_node_idx v);
    void Expand(mcpm_node_idx u, bool expandBlocked = false);

    // 기타 유틸리티
    bool check_tight(mcpm_node_idx u, mcpm_node_idx v);
    double distance(mcpm_node_idx u, mcpm_node_idx v);

    // Blossom bookkeeping
    mcpm_node_idx allocate_new_pseudo();
    void add_free_blossom_index(mcpm_node_idx u);
    void clear_blossom_indices();

    // Dual update
    void dual_update();

    // 초기화
    void Clear();
};

#endif
