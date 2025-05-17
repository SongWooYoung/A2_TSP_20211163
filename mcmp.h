#ifndef MCMP_H
#define MCMP_H

#include <vector>
#include <utility>                    // for make_pair
#include <queue>                      // for grow queue

typedef int mcpm_node_idx;

struct mcpm_node {
    int index_at_oddIndices;          // index in oddIndices
    int index_at_nodes;

    int x;
    int y;
    
    int pair;                         // paired node's index in oddIndices

    int tag;                          // -1, 0 ,1

    bool is_root;
    mcpm_node_idx root;                 // 같은 tree에 있는 것을 쉽게 추적하기 위해서서
    mcpm_node_idx parent;

    bool is_blossom;                    // shrink expand 구현할때 사용할것
    std::vector<mcpm_node_idx>* blossom_members; // memebr 저장 -> stuct마다 박으면 메모리 소모량 과다

    int dual_value;                     // this is used for slack, and will be updated by dua_update()

    mcpm_node(int idx_odd, int idx_nodes, int x, int y);
    mcpm_node& operator=(mcpm_node& other);
    void init();
};

class blossomV {
private:
    std::vector<mcpm_node>& node_list; // oddIndices 받아옴
    int nl_size;
    int matching_num;
    //int total_tree_num;

public:

    blossomV(std::vector<mcpm_node>& list);
    void Grow(mcpm_node_idx u, mcpm_node_idx v, std::queue<int>& grow_queue);
    void Augment(mcpm_node_idx u, mcpm_node_idx v, std::queue<int>& grow_queue);
    void Shrink(mcpm_node_idx u, mcpm_node_idx v);
    void Expand(mcpm_node_idx u, mcpm_node_idx v);
    void primal_update(mcpm_node_idx u, mcpm_node_idx v, std::queue<int>& grow_queue);
    void dual_update();
    void matching(mcpm_node_idx u, mcpm_node_idx v);
    void flip(mcpm_node_idx i);
    void free_tree(mcpm_node_idx i);
    bool check_tight(mcpm_node_idx u, mcpm_node_idx v);
    std::pair<mcpm_node_idx, mcpm_node_idx> find_rep(mcpm_node_idx u, mcpm_node_idx v);
    void execute_all();
};



#endif