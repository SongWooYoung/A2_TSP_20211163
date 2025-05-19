#include "mcpm.h"
#include <algorithm>
#include <cmath>
#include <queue>


using namespace std;

mcpm_node extra_node(mcpm_node_idx i) {
    mcpm_node node(i,-1,-1,-1);
    node.init();
    return node;
}


////////////////////////////////////////////////////////////////////////////

mcpm_node::mcpm_node(int idx_odd, int idx_nodes, int x, int y)
    : index_at_oddIndices(idx_odd), index_at_nodes(idx_nodes), x(x), y(y)
{
    this -> pair                    = -1;
    this -> type                    = UNLABELED;
    this -> root                    = -1;
    this -> parent                  = -1;
    this -> visited                 = false;
    this -> active                  = true;  // 기본 노드는 활성 상태

    this -> is_blossom              = false;
    this -> dual_value              = 0.0;
}

mcpm_node& mcpm_node::operator=(mcpm_node& other) {
    this -> index_at_oddIndices     = other.index_at_oddIndices;
    this -> index_at_nodes          = other.index_at_nodes;
    this -> x                       = other.x;
    this -> y                       = other.y;

    this -> pair                    = other.pair;
    this -> type                    = other.type;
    this -> root                    = other.root;
    this -> parent                  = other.parent;
    this -> visited                 = other.visited;
    this -> active                  = other.active;

    this -> is_blossom              = other.is_blossom;

    this -> dual_value              = other.dual_value;

    return *this;
}

void mcpm_node::init_with_out_pair() {
    this -> type                    = UNLABELED;
    this -> parent                  = -1;
    this -> visited                 = false;
    this -> active                  = true;
    this -> root                    = -1;
    this -> is_blossom              = false;
    this -> dual_value              = 0.0;
}

void mcpm_node::init() {
    this -> init_with_out_pair();
    this -> pair = -1;
}

//////////////////////////////

blossomV::blossomV(vector<mcpm_node>& list) : node_list(list) {
    this -> origin_num = (mcpm_node_idx) list.size();
    this -> nl_size = 2 * origin_num;
    this -> matching_num = 0;

    for (mcpm_node_idx i = this -> origin_num; i < this -> nl_size; ++i) {
        this -> node_list.push_back(extra_node(i));
    }
}

// 모든 tag는 각 함수 내에서 적용 될것 -> 왜냐하면 root 를 +로 정해놓고 나머지는 0으로 출발하기 때문에, 각 작업을 거치면서 알아서 변화 하도록 구현 예정정
// Execute full Edmonds' Blossom algorithm for minimum-cost perfect matching
void blossomV::execute_all() {
    this -> Clear();

    // 초기 Heuristic Matching으로 일부 edge 채움 (성능 가속)
    // greedy 하게 일단 채움
    this -> Heuristic();

    // perfect matching이 될 때까지 반복
    this -> init_without_pair();
    while (this -> matching_num < this -> origin_num / 2) {
        this -> Grow();
        this -> dual_update();
        this -> init_without_pair();
    }
}

void blossomV::Clear() {
    // 모든 vector 초기화
    this -> root.assign(this -> nl_size, 0);
    this -> internal_node.assign(this -> nl_size, {});
    this -> blo_tree.assign(this -> nl_size, {});
    this -> tip.assign(this -> nl_size, -1);
    this -> grow_queue = queue<int>();
    //this -> slack.assign(this -> nl_size * nl_size, 0);

    this -> free_blossom_indices.clear();
    for (mcpm_node_idx i = this -> origin_num; i < this -> nl_size; ++i) {
        this -> free_blossom_indices.push_back(i);
    }

    for (mcpm_node_idx i = 0; i < nl_size; ++i) {
        auto& node = node_list[i];
        node.init();

        this -> root[i] = i;
        this -> tip[i] = i;
        this -> internal_node[i].clear();
        if (i < this -> origin_num) {
            this -> internal_node[i].push_back(i);
            node.active = true;
        } else {
            node.active = false;
        }
    }

    this -> matching_num = 0;
}

void blossomV::Heuristic() {
    vector<int> degree(this -> origin_num, 0);
    // https://medium.com/@RobuRishabh/heap-data-structure-and-priority-queue-in-c-d2fe7a569c86
    // (degree, node_idx)
    priority_queue<pair<int, mcpm_node_idx>, vector<pair<int, int>>, greater<pair<int, mcpm_node_idx>>> pq;
    vector<bool> used(this -> origin_num, false);

    // 1. slack이 tight한 edge에 대해 degree 계산
    for (int u = 0; u < this -> origin_num; ++u) {
        for (int v = u + 1; v < this -> origin_num; ++v) {
            if (this -> check_tight(u, v)) {
                degree[u]++;
                degree[v]++;
            }
        }
    }

    // 2. degree 순으로 min-heap에 넣기 -> 자동 정렬 -> degree 낮은 순
    // https://www.naukri.com/code360/library/vector-push_back-vs-emplace_back
    for (int i = 0; i < this -> origin_num; ++i) {
        pq.emplace(degree[i], i);
    }

    // 3. Greedy matching
    // 현재 pq는 degree에 대한 내림차순으로 정렬되어 있음
    while (!pq.empty()) {
        auto [deg_u, u] = pq.top(); pq.pop();
        if (this -> node_list[u].pair != -1 || used[u]) continue; // 해당 node가 짝이 있거나, 마땅한 tight이 없다면 end 

        int best_v = -1;
        for (int v = 0; v < this -> origin_num; ++v) {
            if (u == v) continue;
            if (this -> node_list[v].pair != -1 || used[v]) continue;
            if (!this -> check_tight(u, v)) continue;

            best_v = v;
            break;
        }

        if (best_v != -1) {
            this -> matching(u, best_v);
            used[u] = used[best_v] = true;
            this -> matching_num++;
        }
    }
}

void blossomV::Grow() {
    
    // Grow queue를 통해 BFS 방식으로 blo_tree 확장
    while (!this -> grow_queue.empty()) {
        mcpm_node_idx u = this -> grow_queue.front();
        grow_queue.pop();

        for (mcpm_node_idx v = 0; v < this -> origin_num; ++v) {
            if (u == v) continue;
            if (!this -> check_tight(u, v)) continue; // slack == 0인 edge만 사용

            auto& u_node = this -> node_list[u];
            auto& v_node = this -> node_list[v];

            // v가 UNLABELED 상태이면 blo_tree에 추가
            if (v_node.type == UNLABELED) {
                v_node.parent = u;
                v_node.root = u_node.root;
                v_node.type = ODD;

                // expand나 augment로 인해 blo_tree가 터지는 경우 pair를 남기고 전부 초기화 
                // 따라서 다른 alterning blo_tree가 성장 할때 만나면 그 짝도 연결 필요 
                //둘중 누구를 -로 설정할지는 구현 차이
                mcpm_node_idx vm = v_node.pair;
                if (vm != -1) { 
                    auto& vm_node = this -> node_list[vm];
                    vm_node.parent = v;
                    vm_node.root = u_node.root;
                    vm_node.type = EVEN;
                    vm_node.visited = true;
                    this -> grow_queue.push(vm);  // 짝은 다음 BFS 대상
                }
            }

            // v가 EVEN이고 root가 다르면 augmenting path 발견
            else if (v_node.type == EVEN && u_node.root != v_node.root) {
                this -> Augment(u, v);      // 이전 구현과 다르게 둘을 분리함
                this -> init_without_pair();
                return;  // 한 번 augment 후 트리 초기화
            }

            // v가 EVEN인데 같은 blo_tree면 blossom
            else if (v_node.type == EVEN && u_node.root == v_node.root) {
                mcpm_node_idx rep_pseudo = this -> Shrink(u, v);
                this -> grow_queue.push(rep_pseudo); // 축약된 노드로 계속 탐색
            }
        }
    }
}

void blossomV::init_without_pair() {
    // 모든 노드를 초기화: 트리 구조, 방문 상태, 라벨 등
    for (mcpm_node_idx i = 0; i < this -> nl_size; ++i) {
        auto& node = node_list[i];
        node.init_with_out_pair();

        this -> root[i] = i;
        this -> tip[i] = i;
        this -> internal_node[i].clear();
        this -> blo_tree[i].clear();

        if (i < origin_num) {
            this -> internal_node[i].push_back(i);
            node.active = true;
        } else {
            node.active = false;
        }
    }

    // grow queue 초기화
    // https://stackoverflow.com/questions/709146/how-do-i-clear-the-stdqueue-efficiently
    queue<int> empty;
    swap(this -> grow_queue, empty);
}

void blossomV::Augment(mcpm_node_idx u, mcpm_node_idx v) {
    // u, v는 각각 다른 alternating blo_tree의 EVEN 노드
    // 둘 사이의 Augmenting Path가 발견된 상황

    this -> matching(u, v);  // u와 v는 직접 매칭됨
    // u 쪽 트리를 따라 올라가며 flip
    // 이전에는 계속 matching을 넣어주려고 했는데 이제는 상황 발생하면 역으로 진행하면서 matching을 넣어 주는 걸로 
    this -> flip(u);
    // v 쪽 트리를 따라 올라가며 flip
    this -> flip(v);

}


mcpm_node_idx blossomV::Shrink(mcpm_node_idx u, mcpm_node_idx v) {
    // 1. u와 v의 경로를 타고 올라가면서 root까지의 경로를 저장
    vector<mcpm_node_idx> path_u, path_v;
    vector<bool> visited_u(this -> nl_size, false);

    mcpm_node_idx cur = u;
    while (cur != -1) {
        path_u.push_back(cur);
        visited_u[cur] = true;
        cur = this -> node_list[cur].parent;
    }

    cur = v;
    while (cur != -1) {
        path_v.push_back(cur);
        if (visited_u[cur]) break;  // 최초의 공통 조상 = LCA
        cur = this -> node_list[cur].parent;
    }

    mcpm_node_idx lca = cur;

    // 2. Blossom용 새 가짜 노드 index 확보
    // node_list에서 비어있는 칸에서 뽑아올 것임

    mcpm_node_idx new_idx = this -> allocate_new_pseudo();

    // 3. circuit 구성 (blo_tree path = odd cycle)
    list<mcpm_node_idx> circuit;

    // u → LCA까지 (역순)
    for (mcpm_node_idx x : path_u) {
        circuit.push_back(x);
        if (x == lca) break;
    }
    circuit.reverse(); //  LCA -> 

    // v → LCA까지 (순방향)
    for (mcpm_node_idx x : path_v) {
        if (x == lca) break;
        circuit.push_back(x);
    }

    this -> blo_tree[new_idx] = circuit;

    // 4. internal_node & root 업데이트
    this -> internal_node[new_idx].clear(); // 내부 노드 추가 과정정
    for (mcpm_node_idx x : circuit) {
        this -> root[x] = new_idx;
        for (mcpm_node_idx d : this -> internal_node[x]) {
            this -> internal_node[new_idx].push_back(d);
            this -> root[d] = new_idx;
        }
    }

    // 5. tip, root 정보 업데이트
    this -> tip[new_idx] = lca;

    this -> node_list[new_idx].parent = this -> node_list[lca].parent;
    this -> node_list[new_idx].root   = this -> node_list[lca].root;
    this -> node_list[new_idx].type   = EVEN;
    this -> node_list[new_idx].visited = true;
    this -> node_list[new_idx].active = true;
    this -> root[new_idx] = new_idx;

    return new_idx;  // 축약된 노드 index 반환 → grow에서 push 가능
}

void blossomV::matching(mcpm_node_idx u, mcpm_node_idx v) {
    this -> node_list[u].pair = v;
    this -> node_list[v].pair = u;
    this -> matching_num++;  // 한 쌍 추가

    // expand는 둘중 하나라도 blossom이면 일어나야 함 -> 그래서 내부 조건에 입력 u가 blossom이 아니면 탈출하도록 설정
    // 두번 호출에서 두번 다 탈출하면 -> 둘다 일반 노드
    this -> Expand(u);
    this -> Expand(v);
}

void blossomV::flip(mcpm_node_idx u) {
    mcpm_node_idx cur = u;

    while (true) {
        auto& cur_node = this -> node_list[cur];
        mcpm_node_idx parent = cur_node.parent;

        // root 또는 더 이상 뒤집을 경로가 없는 경우 종료
        if (parent == -1) break;

        auto& parent_node = this -> node_list[parent];
        mcpm_node_idx grandparent = parent_node.parent;

        // Blossom이면 확장해서 내부 pair를 먼저 재설정
        this -> matching(parent, grandparent); 
        // 다음 대상은 grandparent
        cur = grandparent;
    }
}


void blossomV::Expand(mcpm_node_idx b, bool expandBlocked) {
    // 0. 일반 노드이거나 blossom 내부 정보가 비어있으면 탈출
    if (b < this -> origin_num || this -> blo_tree[b].empty()) return;

    // 1. 이 블로섬과 매칭된 노드 가져오기
    mcpm_node_idx b_pair = this -> node_list[b].pair;

    // 2. shallow 회로 (blo_tree[b]) 기준으로 tip부터 내부 노드들 짝지음
    //    tip은 항상 홀수 길이의 시작점이다: (+ - + - + ...)
    auto it = this -> blo_tree[b].begin();
    this -> node_list[*it].pair = b_pair;  // tip은 b_pair와 연결
    ++it;

    int match_count = 0;
    while (it != this -> blo_tree[b].end()) {
        auto u = *it++;
        if (it == this -> blo_tree[b].end()) break;
        auto v = *it++;
    
        if (this -> node_list[u].pair == -1 && this -> node_list[v].pair == -1) {
            match_count++;
        }
    
        this -> node_list[u].pair = v;
        this -> node_list[v].pair = u;
    }
    this -> matching_num += match_count;
    

    // 3. 내부 노드들에 대해 root 및 active 복구
    for (mcpm_node_idx x : this -> internal_node[b]) {
        this -> root[x] = x;
        this -> node_list[x].active = true;
    }

    for (mcpm_node_idx x : this -> blo_tree[b]) {
        this -> root[x] = x;
        this -> node_list[x].active = true;
    }

    this -> add_free_blossom_index(b);
}

void blossomV::add_free_blossom_index(mcpm_node_idx b) {
    this -> blo_tree[b].clear();
    this -> internal_node[b].clear();
    this -> node_list[b].init();  // pair, parent, type, visited, active 등 초기화
    this -> recycle_idx.push(b);  // 재활용 큐에 넣기
}


mcpm_node_idx blossomV::allocate_new_pseudo() {
    if (!this -> recycle_idx.empty()) {
        mcpm_node_idx b = this -> recycle_idx.front();
        this -> recycle_idx.pop();
        return b;
    }

    // fallback: 비어있는 블로섬 노드 탐색
    for (int i = this -> origin_num; i < this -> nl_size; ++i) {
        if (this -> node_list[i].pair == -1 && !this -> node_list[i].is_blossom) {
            return i;
        }
    }
    return -1; //  구색 맞추기
}

bool blossomV::check_tight(mcpm_node_idx u, mcpm_node_idx v){
    auto& u_node = this -> node_list[u];
    auto& v_node = this -> node_list[v];
    
    double distance = sqrt(
        (u_node.x - v_node.x) * (u_node.x - v_node.x) +
        (u_node.y - v_node.y) * (u_node.y - v_node.y)
    );

    double slack = u_node.dual_value + v_node.dual_value - distance;
    return fabs(slack) < 1e-9;  // https://www.reddit.com/r/learnprogramming/comments/9y9gxs/small_lesson_of_the_day_never_check_two_floats/
}


