#include "mcmp.h"
#include <algorithm>

using namespace std;


mcpm_node::mcpm_node(int idx_odd, int idx_nodes, int x, int y) : index_at_oddIndices(idx_odd),  index_at_nodes(idx_nodes), x(x), y(y) 
{
    this -> pair                = -1;
    this -> tag                 = 0;
    this -> is_root             = false;
    this -> root                = -1;
    this -> parent              = -1; 
    this -> is_blossom          = false;
    this -> blossom_members     = nullptr;
}

mcpm_node& mcpm_node::operator=(mcpm_node& other) {
    this -> index_at_nodes      = other.index_at_nodes;
    this -> index_at_oddIndices = other.index_at_oddIndices;
    this -> x                   = other.x;
    this -> y                   = other.y;
    this -> pair                = other.pair;
    this -> tag                 = other.tag;
    this -> is_root             = other.is_root;
    this -> root                = other.root;
    this -> parent              = other.parent;
    this -> is_blossom          = other.is_blossom;
    this -> blossom_members     = other.blossom_members;
}
void mcpm_node::init() {
    this -> pair                = -1;
    this -> tag                 = 0;
    this -> is_root             = false;
    this -> root                = -1;
    this -> parent              = -1;
    this -> is_blossom          = false;
    this -> blossom_members     = nullptr;
}
//////////////////////////////

blossomV::blossomV(vector<mcpm_node>& list) : node_list(list) {
    
    this -> nl_size             =  (int) this -> node_list.size();
    this -> matching_num        = 0;
    this -> total_tree_num      = 0;
}

void blossomV::Grow(mcpm_node_idx u, mcpm_node_idx v, queue<int>& grow_queue) {
    // Grow -> + none 만 고려
    // 일단 blossom 없이 구현 한다음 blossom을 추가하자
    // 애초애 mcpm_node 자체가 blossom도 될 수 있게 설계해서 아래 코드가 바로 작동하도록 만들고 싶음음
    // SHRINK와 mcpm_node를 잘 조작 해보자

    auto& u_node = this -> node_list[u];
    auto& v_node = this -> node_list[v];

    v_node.tag = -1;
    v_node.parent = u_node.index_at_oddIndices;
    if (u_node.is_root) v_node.root = u_node.index_at_oddIndices;
    else v_node.root = u_node.root;
    grow_queue.push(v_node.index_at_oddIndices);
}

void blossomV::Augment(mcpm_node_idx u, mcpm_node_idx v, std::queue<int>& grow_queue) {
    // Augment + - / - + / + +
    // 일단 blossom 고려 x
    // 애초애 mcpm_node 자체가 blossom도 될 수 있게 설계해서 아래 코드가 바로 작동하도록 만들고 싶음
    // free_tree를 하면서 모든 node의 tag를 초기화 -> 따라서 u, v 는 무조건 primal_update 조건에 걸러져서 반드시 트리만 들어옴

    auto& u_node = this -> node_list[u];
    auto& v_node = this -> node_list[v];

    if (!(u_node.tag == 1 && v_node.tag == 1) || !(u_node.tag * v_node.tag == -1)) return;
    
    this -> matching(u,v);     
    // u와 v가 서로 다른 tree의 중간 노드 일수 있음 -> 간과 했다...
    this -> flip(u);     
    this -> flip(v);
    this -> free_tree(u);
    if (this -> node_list[v].root != this -> node_list[u].root) { // root가 같을 수도 있음 -> root 같고 ++이면 shrink, root 같고 + -면 augment
        this -> free_tree(v);
    }
    
    // empty the queue
    while (!grow_queue.empty()) grow_queue.pop();
    this -> matching_num++;
    return;
}

void blossomV::Shrink(mcpm_node_idx u, mcpm_node_idx v) {
    // shrink가 blossom을 형성함
    // primal_dual에서 조건에 따라 분기 하기 때문에 그 이상을 고려 할 필요가 없음
    // u -> + / v -> + => 둘다 같은 tree에 존재
    
    // LCA 찾기 -> 공통 조상 까지 
    vector<mcpm_node_idx> path_u, path_v;
    mcpm_node_idx curr_u = u, curr_v = v;

    while (curr_u != -1) {
        path_u.push_back(curr_u);
        curr_u = this -> node_list[curr_u].parent;
    }
    while (curr_v != -1) {
        path_v.push_back(curr_v);
        curr_v = this -> node_list[curr_v].parent;
    }

    int lca_loc = -1;
    reverse(path_u.begin(), path_u.end());
    reverse(path_v.begin(), path_v.end());

    size_t min_len = min(path_u.size(), path_v.size());
    for (size_t i = 0; i < min_len; i++) {
        if (path_u[i] == path_v[i]) {
            lca_loc = i;
        } else break;
    }
    // 이제 lca 아래는 전부 넣어 vector동적 할당후 포인터를 받아서 각 멤버에 넣어주면 됨
    // 일단 대표는 설정 x -> grow나 augment의 경우 새로운 method인 find_rep을 만들어서 대표의 정보를 반환 
    // 대표의 정보는 저장할 필요가 있는가? => 어차피 u와 v에 따라 계속 달라짐
    // 모든 노드의 is_blossom을 true로 만들어 전체를 숨기고 대표는 mcpm_node_idx fin_rep()으로 찾는 걸로 하자
    // vector를 heap에 할당 후 pointer만 각 멤버에 전달 예정

    vector<int>* blo_list = new vector<mcpm_node_idx>();
    for (int i = lca_loc; i < (int) path_u.size(); i++) {
        this -> node_list[path_u[i]].is_blossom = true;
        blo_list -> push_back(path_u[i]);
        this -> node_list[path_u[i]].blossom_members = blo_list;
    }

    for (int i = lca_loc+1; i < (int) path_v.size(); i++) {
        this -> node_list[path_v[i]].is_blossom = true;
        blo_list -> push_back(path_v[i]);
        this -> node_list[path_v[i]].blossom_members = blo_list;
    }
}

void blossomV::Expand(mcpm_node_idx u, mcpm_node_idx v) {


    // 반드시 동적 할당된 vector free 해주기!!!
}

void blossomV::primal_update(mcpm_node_idx u, mcpm_node_idx v, queue<int>& grow_queue) {
    auto& u_node = this -> node_list[u];
    auto& v_node = this -> node_list[v];

    if (u_node.tag != 1) return; // +는 능동적 -는 수동적

    if (u_node.tag == 1 && v_node.tag == 1) {
        if (u_node.root == v_node.root) { //  같은 트리에 있을때때
            this -> Shrink(u, v);
        }
        else {
            this -> Augment(u, v, grow_queue);
        }
    }

    else if (v_node.tag == -1) { // tag -> + - or - +
        if (v_node.is_blossom) {
            this -> Expand(u, v); // mcpm_node가 pseudo node로써 역할을 같이 할 수 있도록 가능한가?
        }
        else this -> Augment(u, v, grow_queue);
    }
    else if (u_node.tag == 1 && v_node.tag == 0) {
        this -> Grow(u, v, grow_queue);
    }
    else return;
}

void blossomV::dual_update() {

}

void blossomV::matching(mcpm_node_idx u, mcpm_node_idx v) {
    auto& u_node = this -> node_list[u];
    auto& v_node = this -> node_list[v];

    u_node.pair = v_node.index_at_oddIndices;
    v_node.pair = u_node.index_at_oddIndices;
}

void blossomV::flip(mcpm_node_idx i) {
    mcpm_node_idx curr = i;
    while (!(this -> node_list[curr].is_root)) {
        mcpm_node_idx parent = this -> node_list[curr].parent;
        mcpm_node_idx grandparent = this -> node_list[parent].parent;
    
        this -> node_list[curr].pair = this -> node_list[parent].index_at_oddIndices;
        this -> node_list[parent].pair = this -> node_list[curr].index_at_oddIndices;
    
        curr = grandparent;
    }
}

void blossomV::free_tree(mcpm_node_idx i) { // 모든 tag초기화 

    mcpm_node_idx curr = i;
    while (!node_list[curr].is_root) {
        mcpm_node_idx cur_pair = node_list[curr].pair;
        this -> node_list[curr].init();
        this -> node_list[curr].pair = cur_pair;
        curr = this -> node_list[curr].parent;
    }
}

bool blossomV::check_tight(mcpm_node_idx u, mcpm_node_idx v) {

}
pair<mcpm_node_idx, mcpm_node_idx> blossomV::find_rep(mcpm_node_idx u, mcpm_node_idx v) {
    auto& u_node = this -> node_list[u];
    auto& v_node = this -> node_list[v];

    if (u_node.is_blossom && v_node.is_blossom) { // 둘다다
        for (auto rep1 : *u_node.blossom_members) {
            for (auto rep2 : *v_node.blossom_members) {
                if (this -> check_tight(rep1, rep2)) {
                    return {rep1, rep2};
                }
            }
        }
    }
    else if (u_node.is_blossom) { // u가 blo
        for (auto rep1 : *u_node.blossom_members) {
            if (this -> check_tight(rep1, v)) {
                return {rep1, v};
            }
        }
    }
    else if (v_node.is_blossom) { //vㄱ blo
        for (auto rep1 : *v_node.blossom_members) {
            if (this -> check_tight(u, rep1)) {
                return {u, rep1};
            }
        }
    }
    else {
        return {u, v};  // 둘 다 blossom 아닐 때는 바로 반환
    }

    return {-1, -1};  // 못 찾은 경우
}


// 모든 tag는 각 함수 내에서 적용 될것 -> 왜냐하면 root 를 +로 정해놓고 나머지는 0으로 출발하기 때문에, 각 작업을 거치면서 알아서 변화 하도록 구현 예정정
void blossomV::execute_all() {
    while (this -> matching_num < (this -> nl_size) / 2) {              // 계속 반복하다가 완전히 매칭 되는 경우 탈출
        int tree_id = this -> total_tree_num++;                                 // tree는 0 부터 저장 될것 -> 저장후 1을 늘려서 다음 위치를 명시시
        queue<int> grow_queue;                                          // tight_nodes(tree 역할)은 계속 확장되어야 해서 새로운 queue필요요

        for (mcpm_node_idx i = 1; i < this -> nl_size; i++) {                 
            if (this -> node_list[i].tag != 0) continue;                // tree의 head(반드시 tag가 0인 free vertex 이어야함)를 결정
            
            this -> node_list[i].tag = 1;                                              // root 노드는 반드시 1이어야 함
            this -> node_list[i].is_root = true;
            grow_queue.push(this -> node_list[i].index_at_oddIndices);
            
            // tree expansion
            while (!grow_queue.empty()) {
                mcpm_node_idx u_idx = grow_queue.front(); 
                grow_queue.pop();

                for (mcpm_node& v_node : this->node_list) {
                    if (this -> node_list[u_idx].index_at_nodes == v_node.index_at_nodes) continue;
                    if (!check_tight(u_idx, v_node.index_at_oddIndices)) continue; // tight 이어야지만 primal_update 실행
                    primal_update(u_idx, v_node.index_at_oddIndices, grow_queue);  // Grow가 발생하면 primal_update 안에서 tight_nodes에 추가하고 grow_queue에 push_back 해야함 (Grow에서 담당)
                }
            }
        }
        this -> dual_update(); // tree가 1방향으로 크도록 해야함 -> 엄격한 slack관리가 필요요
    }
}