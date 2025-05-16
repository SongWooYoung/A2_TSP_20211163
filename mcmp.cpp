#include "mcmp.h"

using namespace std;

mcpm_node init_node() {
    mcpm_node v(-1,-1,-1,-1);
    return v;
}

vector<int> init_list() {
    vector<int> init_list;
    init_list.push_back(-1);
    return init_list;
}

blossomV::blossomV(vector<mcpm_node>& list) : node_list(list) {
    tight_nodes.push_back(init_list());
    this -> nl_size             =  (int) this -> node_list.size();
    //this -> all_node_visited    = false;
    this -> matching_num        = 0;
    this -> total_tree_num      = 0;
    //this -> present_root_index  = 1;  // odd_indices는 1부터 담도록 작업함
}

void blossomV::Grow(mcpm_node_idx u, mcpm_node_idx v, queue<int>& grow_queue) {
    
}

void blossomV::Augment(mcpm_node_idx u, mcpm_node_idx v) {

}

void blossomV::Shrink(mcpm_node_idx u, mcpm_node_idx v) {


}

void blossomV::Expand(mcpm_node_idx u, mcpm_node_idx v) {


}

void blossomV::flow(mcpm_node_idx u, mcpm_node_idx v, queue<int>& grow_queue) {
    if (this -> node_list[u].tag == 1 && this -> node_list[v].tag == 1) {
        if (this -> node_list[u].tree_loc.first == this -> node_list[v].tree_loc.first) { //  it means it is in same tree
            this -> Shrink(u, v);
        }
        else {
            this -> Augment(u, v);
        }
    }

    else if (this -> node_list[u].tag != this -> node_list[v].tag) { // tag -> + - or - +
        if ((this -> node_list[u].is_blossom && this -> node_list[u].tag == -1) || 
             this -> node_list[v].is_blossom && this -> node_list[v].tag == -1) {
            this -> Expand(u, v); // mcpm_node가 pseudo node로써 역할을 같이 할 수 있도록 가능한가?
        }
        else {
            this -> Grow(u, v, grow_queue);
        }
    }
    else return; // - - 의미 x
}

void blossomV::update() {

}

bool blossomV::check_tight(mcpm_node_idx u, mcpm_node_idx v) {

}


// 모든 tag는 각 함수 내에서 적용 될것 -> 왜냐하면 root 를 +로 정해놓고 나머지는 0으로 출발하기 때문에, 각 작업을 거치면서 알아서 변화 하도록 구현 예정정
void blossomV::execute_all() {
    while (this -> matching_num < (this -> nl_size) / 2) {              // 계속 반복하다가 완전히 매칭 되는 경우 탈출
        int tree_id = this -> total_tree_num++;                                 // tree는 0 부터 저장 될것 -> 저장후 1을 늘려서 다음 위치를 명시시
        queue<int> grow_queue;                                          // tight_nodes(tree 역할)은 계속 확장되어야 해서 새로운 queue필요요

        for (mcpm_node_idx i = 1; i < this -> nl_size; i++) {                 
            if (this -> node_list[i].tag != 0) continue;                // tree의 head(반드시 tag가 0인 free vertex 이어야함)를 결정
            
            this -> node_list[i].tag = 1;                                              // root 노드는 반드시 1이어야 함
            this -> node_list[i].tree_loc = {tree_id, 0};
            this -> tight_nodes[tree_id].push_back(this -> node_list[i].index_at_oddIndices);           // 다음 root 노드 탐색하기 위해서 기억해두어야 하나??
            grow_queue.push(this -> node_list[i].index_at_oddIndices);
            //this -> present_root_index = i;                                             // 다음 탐색은 present_root_index에서 출발발
            
            // tree expansion
            while (!grow_queue.empty()) {
                mcpm_node_idx u_idx = grow_queue.front(); 
                grow_queue.pop();

                for (mcpm_node& v_node : this->node_list) {
                    if (this -> node_list[u_idx].index_at_nodes == v_node.index_at_nodes) continue;
                    if (!check_tight(u_idx, v_node.index_at_oddIndices)) continue; // tight 이어야지만 flow 실행
                    flow(u_idx, v_node.index_at_oddIndices, grow_queue);  // Grow가 발생하면 flow 안에서 tight_nodes에 추가하고 grow_queue에 push_back 해야함 (Grow에서 담당)
                }
            }
        }
        this -> update();
    }
}