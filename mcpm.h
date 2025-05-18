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
    int index_at_oddIndices;        // Working set에서의 인덱스
    int index_at_nodes;             // 실제 그래프 인덱스
    int x, y;                       // 위치 정보 (거리 계산용)

    // 상태 정보
    mcpm_node_idx pair;            // 매칭된 노드
    NodeType type;                 // EVEN, ODD, UNLABELED
    bool is_root;                  
    mcpm_node_idx root;            // 소속 트리의 루트
    mcpm_node_idx parent;          // 트리에서의 부모
    bool visited;                  // Grow 중 방문 여부
    bool active;                   // 활성 상태 (blossom 포함)

    // Blossom 관련
    bool is_blossom;               
    std::vector<mcpm_node_idx>* blossom_members; // 축약된 노드들의 목록

    // Dual 값
    double dual_value;

    mcpm_node(int idx_odd, int idx_nodes, int x, int y);
    mcpm_node& operator=(mcpm_node& other);
    void init();                    // 상태 초기화
};

class blossomV {
private:
    std::vector<mcpm_node>& node_list;   // 외부에서 주입받는 노드 리스트 (2n 크기)
    int origin_num;                      // 실제 노드 수 (n)
    int nl_size;                         // 전체 노드 리스트 크기 (2n)
    int matching_num;                    // 현재 매칭 수

    std::vector<mcpm_node_idx> outer;    // Blossom 대표 노드 index (축약용)
    std::vector<std::list<mcpm_node_idx>> deep;     // 각 blossom의 내부 노드들
    std::vector<std::list<mcpm_node_idx>> shallow;  // shrink 시 shallow 회로
    std::vector<int> free_blossom_indices;          // 사용할 수 있는 가짜 노드 index
    std::queue<int> grow_queue;

    std::vector<double> slack;                     // edge별 slack 값 (거리 기반)
    
public:
    blossomV(std::vector<mcpm_node>& list);

    // 전체 알고리즘 실행
    void execute_all();

};

#endif
