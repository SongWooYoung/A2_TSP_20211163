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
    
    bool matching;                    // if matched -> true;
    int pair;                         // paired node's index in oddIndices

    int tag;                          // -1, 0 ,1

    bool is_blossom;                    // shrink expand 구현할때 사용할것

    std::pair<int, int> tree_loc;     // it shows where it belongs 
    
    mcpm_node(int idx_odd, int idx_nodes, int x, int y) 
       : index_at_oddIndices(idx_odd),  index_at_nodes(idx_nodes), x(x), y(y) 
    {
        this -> matching    = false;
        this -> pair        = -1;
        this -> tag         = 0;
        this -> tree_loc    = std::make_pair(-1, -1);
        this -> is_blossom  = false;
    }

    mcpm_node& operator=(mcpm_node& other) {
        this -> index_at_nodes      = other.index_at_nodes;
        this -> index_at_oddIndices = other.index_at_oddIndices;
        this -> x                   = other.x;
        this -> y                   = other.y;
        this -> matching            = other.matching;
        this -> pair                = other.pair;
        this -> tag                 = other.tag;
        this -> tree_loc            = other.tree_loc;
    }
};

class blossomV {
private:
    std::vector<mcpm_node>& node_list; // oddIndices 받아옴
    std::vector<std::vector<int>> tight_nodes;  // works as tree
    int nl_size;
    //bool all_node_visited;
    int matching_num;
    int total_tree_num;
    //mcpm_node_idx present_root_index;
    //int upper_bound; // 이게 slack을 계산하는데 쓰일것

public:

    blossomV(std::vector<mcpm_node>& list);
    void Grow(mcpm_node_idx u, mcpm_node_idx v, std::queue<int>& grow_queue);
    void Augment(mcpm_node_idx u, mcpm_node_idx v);
    void Shrink(mcpm_node_idx u, mcpm_node_idx v);
    void Expand(mcpm_node_idx u, mcpm_node_idx v);
    void flow(mcpm_node_idx u, mcpm_node_idx v, std::queue<int>& grow_queue);
    void update();
    bool check_tight(mcpm_node_idx u, mcpm_node_idx v);

    void execute_all();
};



#endif