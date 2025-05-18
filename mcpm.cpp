#include "mcpm.h"
#include <algorithm>
#include <cmath>
#include <queue>


using namespace std;

mcpm_node extra_node() {
    mcpm_node node(-1,-1,-1,-1);
    node.init();
    return node;
}

/////

mcpm_node::mcpm_node(int idx_odd, int idx_nodes, int x, int y)
    : index_at_oddIndices(idx_odd), index_at_nodes(idx_nodes), x(x), y(y)
{
    this -> pair                    = -1;
    this -> type                    = UNLABELED;
    this -> is_root                 = false;
    this -> root                    = -1;
    this -> parent                  = -1;
    this -> visited                 = false;
    this -> active                  = true;  // 기본 노드는 활성 상태

    this -> is_blossom              = false;
    this -> blossom_members         = nullptr;

    this -> dual_value              = 0.0;
}

mcpm_node& mcpm_node::operator=(mcpm_node& other) {
    this -> index_at_oddIndices     = other.index_at_oddIndices;
    this -> index_at_nodes          = other.index_at_nodes;
    this -> x                       = other.x;
    this -> y                       = other.y;

    this -> pair                    = other.pair;
    this -> type                    = other.type;
    this -> is_root                 = other.is_root;
    this -> root                    = other.root;
    this -> parent                  = other.parent;
    this -> visited                 = other.visited;
    this -> active                  = other.active;

    this -> is_blossom              = other.is_blossom;
    this -> blossom_members         = other.blossom_members; 

    this -> dual_value              = other.dual_value;

    return *this;
}

void mcpm_node::init() {
    this -> pair                    = -1;
    this -> type                    = UNLABELED;
    this -> is_root                 = false;
    this -> root                    = -1;
    this -> parent                  = -1;
    this -> visited                 = false;
    this -> active                  = true;

    this -> is_blossom              = false;
    this -> blossom_members         = nullptr;

    this -> dual_value              = 0.0;
}

//////////////////////////////

blossomV::blossomV(vector<mcpm_node>& list) : node_list(list) {
    this -> origin_num = (int)list.size();
    this -> nl_size = 2 * origin_num;

    for (int i = this -> origin_num; i < this -> nl_size; ++i) {
        this -> node_list.push_back(extra_node());
    }
}
