#include "mcpm.h"
#include <algorithm>
#include <cmath>
#include <queue>
#include <iostream>


using namespace std;


////////////////////////////////////////////////////////////////////////////

mcpm_node::mcpm_node(int idx_odd, int idx_nodes, int x, int y)
    : index_at_oddIndices(idx_odd), index_at_nodes(idx_nodes), x(x), y(y)
{
    this -> pair                    = -1;
    this -> type                    = UNLABELED;
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
    this -> parent                  = other.parent;
    this -> visited                 = other.visited;
    this -> active                  = other.active;

    this -> is_blossom              = other.is_blossom;

    this -> dual_value              = other.dual_value;

    return *this;
}

void mcpm_node::init_with_out_pair_and_active() {
    this -> type                    = UNLABELED;
    this -> parent                  = -1;
    this -> visited                 = false;
    this -> is_blossom              = false;
}

void mcpm_node::init() {
    this -> init_with_out_pair_and_active();
    this -> pair = -1;
    this -> active = true;
}

void mcpm_node::free_blossom_node() {
    this -> x                       = -1;
    this -> y                       = -1;
    this -> index_at_nodes          = -1;
    this -> pair                    = -1;
    this -> type                    = UNLABELED;
    this -> parent                  = -1;
    this -> visited                 = false;
    this -> active                  = false;
    this -> is_blossom              = false;
    this -> dual_value              = 0.0;
}

//////////////////////////////

blossomV::blossomV(vector<mcpm_node>& list) : node_list(list) {
    this -> origin_num = (mcpm_node_idx) list.size();
    this -> nl_size = 2 * origin_num;
    this -> matching_num = 0;

    for (mcpm_node_idx i = this -> origin_num; i < this -> nl_size; ++i) {
        this -> node_list.emplace_back(i, -1, -1, -1);
    }
}

// 모든 tag는 각 함수 내에서 적용 될것 -> 왜냐하면 root 를 +로 정해놓고 나머지는 0으로 출발하기 때문에, 각 작업을 거치면서 알아서 변화 하도록 구현 예정정
// Execute full Edmonds' Blossom algorithm for minimum-cost perfect matching
void blossomV::execute_all() {
    this -> Clear();

    // 초기 Heuristic Matching으로 일부 edge 채움 (성능 가속)
    // greedy 하게 일단 채움
    this -> initialize_duals();
    this -> Heuristic();


    this -> init_without_pair();

    int num = 1;
    while (this -> matching_num  < this -> origin_num/2) {

        if (num > 7) break;
        cout << "PHASE:" << num << endl;
        cout << "Purpose: " << this -> origin_num / 2 << endl;
        cout << "Matching NUM: " << this -> matching_num << endl;

        this -> Grow();
        cout << "AFTER GROW: " << endl;
        this -> debug_alternating_tree_state();

        this -> dual_update();
        cout << "AFTER Dual Update: " << endl;
        this -> debug_alternating_tree_state();

        this -> init_without_pair();
        cout << "AFTER Reset: " << endl;
        this -> debug_alternating_tree_state();

        num++;
    }
    cout << "IF YOU REACH HERE, ALL IS DONE" << endl;
}

void blossomV::Clear() {
    // 모든 vector 초기화
    this -> root.assign(this -> nl_size, 0);
    this -> internal_node.assign(this -> nl_size, {});
    this -> blo_tree.assign(this -> nl_size, {});
    this -> tip.assign(this -> nl_size, -1);
    this -> grow_queue = queue<int>();

    this -> recycle_idx = queue<mcpm_node_idx>();
    for (mcpm_node_idx i = this -> origin_num; i < this -> nl_size; ++i) {
        this -> recycle_idx.push(i);
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
    // https://www.naukri.com/codexpand_e60/library/vector-push_back-vs-emplace_back
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
        }
    }
}

void blossomV::Grow() {
    while (!this -> grow_queue.empty()) {
        mcpm_node_idx u = this -> grow_queue.front();
        this -> grow_queue.pop();

        for (mcpm_node_idx v = 0; v < this -> nl_size; ++v) {
            if (u == v) continue;
            if (this -> node_list[v].x == -1 && this -> node_list[v].y == -1) {
                cout << v << ": Grow Rejected (empty)" << endl;
                continue;
            }

            if (!this -> check_tight(u, v)) {
                cout << v << ": Grow Rejected (not tight)" << endl;
                continue;
            }

            // 방문된 노드 처리
            if (this -> node_list[v].visited) {
                // 둘 다 EVEN 이면 shrink or augment
                if (this -> node_list[u].type == EVEN && this -> node_list[v].type == EVEN) {
                    if (this -> root[u] != this -> root[v]) {
                        // augmenting path 발견
                        cout << v << ": Grow Permitted (AUGMENT)" << endl;
                        this -> Augment(u, v);
                        this -> init_without_pair();
                        return;
                    } else {
                        // 같은 tree 안의 (+,+): shrink
                        if (u > v) {
                            cout << v << ": Grow Permitted (SHRINK)" << endl;
                            mcpm_node_idx rep = this -> Shrink(u, v);
                            this -> grow_queue.push(rep); // blossom 대표 노드 추가
                            this -> debug_alternating_tree_state();
                        }
                        else {
                            cout << v << ": Grow Denied (SHRINK: u < v)" << endl;
                        }
                        continue;
                    }
                } else {
                    // 나머지는 방문 불필요
                    cout << v << ": Grow Rejected (already visited, not applicable)" << endl;
                    continue;
                }
            }

            // UNLABELED이면 grow 확장
            cout << v << ": Grow Permitted (GROW)" << endl;
            auto& v_node = this -> node_list[v];

            v_node.parent = u;
            this -> root[v] = this -> root[u];
            v_node.type = ODD;
            v_node.visited = true;

            mcpm_node_idx vp = v_node.pair;
            if (vp != -1) {
                auto& vp_node = this -> node_list[vp];
                if (vp_node.type == UNLABELED) {
                    vp_node.parent = v;
                    this -> root[vp] = this -> root[v];
                    vp_node.type = EVEN;
                    vp_node.visited = true;
                    this -> grow_queue.push(vp);
                }
            }
        }
    }
}


void blossomV::init_without_pair() {
    for (mcpm_node_idx i = 0; i < this -> origin_num; ++i) {
        auto& node = this -> node_list[i];

        // 기본 라벨링, 방문 여부, 트리 상태 초기화

        node.init_with_out_pair_and_active();        

        this -> root[i] = i;
        this -> tip[i] = i;

        // 원래 노드만 트리 재설정 대상
        this -> internal_node[i].clear();
        this -> internal_node[i].push_back(i);
        this -> blo_tree[i].clear();

        // 가상 노드 (Shrink) 는 구조를 보존하고, 숨김 처리만
    }

    // Grow queue 초기화
    queue<int> empty;
    swap(this -> grow_queue, empty);

    for (mcpm_node_idx i = 0; i < origin_num; ++i) {
        if (this -> node_list[i].pair == -1 && (this -> node_list[i].active)) {
            this -> node_list[i].type = EVEN;
            this -> node_list[i].visited = true;
            this -> root[i] = i;
            this -> grow_queue.push(i);
        } else {
            this -> node_list[i].type = UNLABELED;
        }
    }
}


void blossomV::Augment(mcpm_node_idx u, mcpm_node_idx v) {
    // u, v는 각각 다른 alternating blo_tree의 EVEN 노드
    // 둘 사이의 Augmenting Path가 발견된 상황

    this -> matching(u, v);  // u와 v는 직접 매칭됨

    this -> debug_alternating_tree_state();
    // u 쪽 트리를 따라 올라가며 flip
    // 이전에는 계속 matching을 넣어주려고 했는데 이제는 상황 발생하면 역으로 진행하면서 matching을 넣어 주는 걸로 
    this -> flip(u);

    // v 쪽 트리를 따라 올라가며 flip
    this -> flip(v);

    // 어차피 대표 노드와 연결되므로, expand 나중에 호출해도 상관 없을듯

    if (this -> node_list[u].is_blossom) this -> Expand(u);
    if (this -> node_list[v].is_blossom) this -> Expand(v);
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

    this -> node_list[new_idx].parent       = this -> node_list[lca].parent;
    this -> node_list[new_idx].x            = this -> node_list[this -> tip[new_idx]].x;
    this -> node_list[new_idx].y            = this -> node_list[this -> tip[new_idx]].y;
    this -> node_list[new_idx].type         = EVEN;
    this -> node_list[new_idx].visited      = true;
    this -> node_list[new_idx].active       = true;
    this -> node_list[new_idx].is_blossom   = true;
    this -> root[new_idx] = new_idx;

    // 6. 내부 노드 hide
    // Shrink 시 내부 노드를 숨기기
    for (auto i : this -> blo_tree[new_idx]) {
        this -> node_list[i].active = false;
    }

    return new_idx;  // 축약된 노드 index 반환 → grow에서 push 가능
}

void blossomV::matching(mcpm_node_idx u, mcpm_node_idx v) {

    this -> node_list[this -> tip[u]].pair = this -> tip[v];
    this -> node_list[this -> tip[v]].pair = this -> tip[u];
    this -> node_list[u].pair = v;
    this -> node_list[v].pair = u;
    this -> matching_num += 1;  // 한 쌍 추가
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
        this -> node_list[parent].pair = grandparent;
        this -> node_list[grandparent].pair = parent;
        // 다음 대상은 grandparent
        cur = grandparent;
    }
}

void blossomV::Expand(mcpm_node_idx b) {
    // 0. 일반 노드이거나 blossom 내부 정보가 비어있으면 탈출
    if (b < this -> origin_num || this -> blo_tree[b].empty()) return;

    // 1. 이 블로섬과 매칭된 노드 가져오기
    mcpm_node_idx b_pair = this -> node_list[b].pair;
    cout << "b_pair: " << b_pair << endl;

    // 2. tip 노드: 항상 홀수 길이 시작점 (+ - + - ...)
    auto it = this -> blo_tree[b].begin();
    mcpm_node_idx tip = *it;

    this -> node_list[tip].pair = b_pair;
    this -> node_list[b_pair].pair = tip;
    ++it;

    while (it != this -> blo_tree[b].end()) {
        auto u = *it++;
        if (it == this -> blo_tree[b].end()) break;
        auto v = *it++;

        this -> node_list[u].pair = v;
        this -> node_list[v].pair = u;
    }

    // 3. root 및 active 복구
    for (mcpm_node_idx x : this -> internal_node[b]) {
        this -> root[x] = x;
        this -> node_list[x].active = true;
    }
    for (mcpm_node_idx x : this -> blo_tree[b]) {
        this -> root[x] = x;
        this -> node_list[x].active = true;
    }

    // 4. blossom 삭제
    this -> add_free_blossom_index(b);

}


void blossomV::add_free_blossom_index(mcpm_node_idx b) {
    this -> blo_tree[b].clear();
    this -> internal_node[b].clear();
    this -> tip[b] = -1;
    this -> node_list[b].free_blossom_node();  // pair, parent, type, visited, active 등 초기화
    this -> node_list[b].index_at_oddIndices = b;
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

    double slack = distance - (u_node.dual_value + v_node.dual_value);
    return fabs(slack) < 1e-6;  // https://www.reddit.com/r/learnprogramming/comments/9y9gxs/small_lesson_of_the_day_never_check_two_floats/
}


void blossomV::dual_update() {
    double min_delta = INF; // 최소 델타 값을 저장할 변수

    cout << "\n[D] ======= Dual Update Start =======" << endl;

    // Phase 1: 델타 후보 계산
    // 활성화된 모든 EVEN 노드 (u)에서 다른 활성화된 노드 (v)로의 간선을 탐색
    for (mcpm_node_idx u = 0; u < this -> nl_size; ++u) { 
        if (!this -> node_list[u].active || this -> node_list[u].type != EVEN) continue;

        for (mcpm_node_idx v = 0; v < this -> nl_size; ++v) {
            if (u == v || !this -> node_list[v].active) continue;

            double dist = sqrt(pow(this -> node_list[u].x - this -> node_list[v].x, 2) +
                               pow(this -> node_list[u].y - this -> node_list[v].y, 2));
            double slack = dist - (this -> node_list[u].dual_value + this -> node_list[v].dual_value);

            if (slack < -1e-9) { 
                cout << "[WARNING] Negative slack encountered (u=" << u << ", v=" << v << ") slack=" << slack << endl;
                continue;
            }
            if (fabs(slack) < 1e-9) { 
                slack = 0.0;
            }

            if (this -> node_list[v].type == UNLABELED) {
                min_delta = min(min_delta, slack);
                cout << "[Delta Candidate 1 - EVEN to UNLABELED] (" << u << "," << v << ") slack=" << slack << endl;
            }
            else if (this -> node_list[v].type == EVEN) {
                min_delta = min(min_delta, slack / 2.0);
                cout << "[Delta Candidate 2/3 - EVEN to EVEN] (" << u << "," << v << ") slack/2=" << slack / 2.0 << endl;
            }
        }
    }

    // Phase 2: ODD 타입 블러섬 확장 조건
    for (mcpm_node_idx i = this -> origin_num; i < nl_size; ++i) {
        if (node_list[i].active && node_list[i].type == ODD) { 
            min_delta = min(min_delta, this ->  node_list[i].dual_value);
            cout << "[Delta Candidate 4 (Blossom Expand)] node " << i << " dual=" << this -> node_list[i].dual_value << endl;
        }
    }

    double delta = min_delta;
    cout << "[D] Calculated min_delta=" << min_delta << " → delta=" << delta << endl;

    if (delta == INF || delta < 1e-9) { 
        cout << "[D] Dual update skipped: delta too small or INF." << endl;
        return;
    }

    for (mcpm_node_idx i = 0; i < nl_size; ++i) {
        if (!this -> node_list[i].active) continue;

        if (this ->  node_list[i].type == EVEN) {
            this -> node_list[i].dual_value += delta;
        } 
        else if (this -> node_list[i].type == ODD) {
            this -> node_list[i].dual_value = max(0.0, this -> node_list[i].dual_value - delta);
        }
    }

    for (mcpm_node_idx i = this -> origin_num; i < this -> nl_size; ++i) {
        auto& node = this -> node_list[i];
        if (node.active && node.type == ODD && fabs(node.dual_value) < 1e-9) {
            cout << "EXPAND Blossom node " << i << " due to zero dual value!" << endl;
            this -> Expand(i); 
        }
    }
    cout << "[D] ======= Dual Update Done =======\n" << endl;
    this -> manage_blo_tight();
}

void blossomV::manage_blo_tight() {
    // queue에는 못들어가는 tight blo - blo짝 전부 처리
    for (int u = this -> origin_num; u < nl_size; ++u) {
        if (!node_list[u].active || !node_list[u].is_blossom || node_list[u].type != EVEN) {
            continue;
        }
    
        for (int v = u + 1; v < nl_size; ++v) {
            if (!this -> node_list[v].active || !this -> node_list[v].is_blossom || this -> node_list[v].type != EVEN) {
                continue;
            }
    
            if (!this -> check_tight(u, v)) {
                cout << "NOT TIGHT!!!!!!!" << endl;
                continue;
            }
    
            if (root[u] != root[v]) {
                this -> Augment(u, v);  // augment 가능
                cout << "Dual_update Augment Done: (" << u <<", " << v << ")" << endl;
            } else {
                this -> Shrink(u, v); // 같은 tree면 shrink
                cout << "Dual_update Shrink Done: (" << u <<", " << v << ")" << endl;
                // shrink된 pseudonode는 grow queue에 안 들어감
            }
        }
    }
}


void blossomV::initialize_duals() {
    for (mcpm_node_idx u = 0; u < this -> origin_num; ++u) {
        double min_dist = INF;
        for (mcpm_node_idx v = 0; v < this -> origin_num; ++v) {
            if (u == v) continue;
            double dist = sqrt(pow(this -> node_list[u].x - this -> node_list[v].x, 2) +
                               pow(this -> node_list[u].y - this -> node_list[v].y, 2));
            min_dist = min(min_dist, dist);
        }
        this -> node_list[u].dual_value = 0.5 * min_dist;
    }
}

void blossomV::debug_alternating_tree_state() {
    cout << "\n========= [ Alternating Tree Debug Info ] =========\n";
    for (mcpm_node_idx i = 0; i < this -> nl_size; ++i) {
        const auto& node = this -> node_list[i];

        cout << "Node [" << i << "] ";
        cout << "(x=" << node.x << ", y=" << node.y << ") ";
        cout << "| active=" << node.active;
        cout << ", is_blossom=" << node.is_blossom;
        cout << ", type=";

        switch (node.type) {
            case EVEN: cout << "EVEN"; break;
            case ODD: cout << "ODD"; break;
            case UNLABELED: cout << "UNLABELED"; break;
        }

        cout << ", root=" << this -> root[i];
        cout << ", tip=" << this -> tip[i];
        cout << ", parent=" << node.parent;
        cout << ", pair=" << node.pair;
        cout << ", dual=" << node.dual_value;
        cout << endl;
    }
    cout << "====================================================\n";
}

void blossomV::merge_pairs(vector<tuple<int, int, double>>& mst_edges) {
    vector<bool> visited(this -> origin_num, false);

    for (mcpm_node_idx u = 0; u < this -> origin_num; ++u) {
        mcpm_node_idx v = this -> node_list[u].pair;

        auto& u_node = this -> node_list[u];
        auto& v_node = this -> node_list[v];

        if (v != -1 && !visited[u] && !visited[v]) {
            double distance = sqrt(
                (u_node.x - v_node.x) * (u_node.x - v_node.x) +
                (u_node.y - v_node.y) * (u_node.y - v_node.y)
            );
            mst_edges.emplace_back(u_node.index_at_nodes, v_node.index_at_nodes, distance);
            visited[u] = visited[v] = true;
        }
    }
}
