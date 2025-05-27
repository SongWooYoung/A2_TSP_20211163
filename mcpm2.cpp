#include "mcpm2.h"

using namespace std;

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
    this -> Initialize_duals();
    this -> Heuristic();
    this -> Reset();

    int num = 1;
    while (this -> matching_num  < this -> origin_num/2) {

        //if (num > 10) break;
        cout << "PHASE:" << num << endl;
        cout << "Purpose: " << this -> origin_num / 2 << endl;
        cout << "Matching NUM: " << this -> matching_num << endl;

        this -> primal_update();
        cout << "AFTER PRIMAL: " << endl;
        this -> debug_alternating_tree_state();

        this -> dual_update();
        cout << "AFTER DUAL: " << endl;
        this -> debug_alternating_tree_state();

        this -> Reset();
        cout << "RESET: " << endl;
        this -> debug_alternating_tree_state();

        num++;
    }

    this -> print_mcpm_total_weight();
}

void blossomV::Clear() {

    this -> root.assign(this -> nl_size, 0);
    this -> child.assign(this -> nl_size, vector<mcpm_node_idx>());
    this -> internal_node.assign(this -> nl_size, {});
    this -> blo_tree.assign(this -> nl_size, {});
    this -> tip.assign(this -> nl_size, -1);
    this -> grow_queue = queue<int>();
    this -> recycle_idx = queue<mcpm_node_idx>();
    // this -> max_dual_delta.assign(this -> nl_size, INF);

    for (mcpm_node_idx i = this -> origin_num; i < this -> nl_size; ++i) {
        this -> recycle_idx.push(i);
    }

    for (mcpm_node& node : this -> node_list) {
        node.init();
    
        mcpm_node_idx i = node.index_at_oddIndices;
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

void blossomV::Initialize_duals() {
    for (mcpm_node_idx u = 0; u < this -> origin_num; ++u) {
        double min_dist = INF;
        for (mcpm_node_idx v = 0; v < this -> origin_num; ++v) {
            if (u == v) continue;
            double dist = this -> dist(u, v);
            min_dist = min(min_dist, dist);
        }
        this -> node_list[u].dual_value = 0.5 * min_dist;
    }
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


void blossomV::primal_update() {

    while (!this -> grow_queue.empty()) {
        mcpm_node_idx u = this -> grow_queue.front(); // 모든 u는 (+)로 init 되어서 옴 + free vertex임임
        this -> grow_queue.pop();

        if (this -> node_list[u].visited) continue;
        this -> node_list[u].visited = true;

        for (mcpm_node v_node : this -> node_list) {

            mcpm_node_idx v = v_node.index_at_oddIndices;
            if (u == v || !v_node.active || !this -> check_tight(u, v)) continue;
            if (v_node.type == EVEN) {
                if (this -> root[u] == this -> root[v]) {
                    this -> Shrink(u,v);
                }
                else {
                    this -> Augment(u, v);
                    this -> Reset(); // 트리 구조가 완전히 깨지기 때문
                }
            }
            if (v_node.type == UNLABELED) {
                this -> Grow(u, v);
            }
            else {
                continue; //구색 맞추기
            }
        }
    }
}

void blossomV::dual_update() {
    double e1 = INF, e2 = INF, e3 = INF;
    bool inite1 = false, inite2 = false, inite3 = false;

    mcpm_node_idx grow_root = -1;
    pair<mcpm_node_idx, mcpm_node_idx> augment_roots = {-1, -1};
    mcpm_node_idx expand_root = -1;

    // CASE 1: Grow 후보 (EVEN - UNLABELED)
    for (mcpm_node_idx u = 0; u < origin_num; ++u) {
        if (!node_list[u].active || node_list[u].type != EVEN) continue;

        for (mcpm_node_idx v = 0; v < nl_size; ++v) {
            if (u == v || !node_list[v].active || node_list[v].type != UNLABELED) continue;

            double s = slack(u, v);
            if (!inite1 || s < e1) {
                e1 = s;
                inite1 = true;
                grow_root = u;
            }
        }
    }

    // CASE 2: Augment 후보 (EVEN - EVEN, 다른 트리)
    for (mcpm_node_idx u = 0; u < origin_num; ++u) {
        if (!node_list[u].active || node_list[u].type != EVEN) continue;

        for (mcpm_node_idx v = 0; v < nl_size; ++v) {
            if (u == v || !node_list[v].active || node_list[v].type != EVEN) continue;
            if (root[u] == root[v]) continue;

            double s = slack(u, v) / 2.0;
            if (!inite2 || s < e2) {
                e2 = s;
                inite2 = true;
                augment_roots = {u, v};
            }
        }
    }

    // CASE 3: Expand 후보 (ODD Blossom dual == 0)
    for (mcpm_node_idx i = origin_num; i < nl_size; ++i) {
        if (!node_list[i].active || node_list[i].type != ODD) continue;

        double d = node_list[i].dual_value;
        if (!inite3 || d < e3) {
            e3 = d;
            inite3 = true;
            expand_root = i;
        }
    }
    cout << "grow: " << grow_root << ", augment: (" << augment_roots.first << ", " << augment_roots.second << "), expand: " << expand_root << endl; 

    // 가장 작은 delta 선택
    double delta = INF;
    unordered_set<mcpm_node_idx> affected_roots;

    if (inite1 && e1 < delta) {
        delta = e1;
        affected_roots.clear();
        affected_roots.insert(root[grow_root]);
    }
    if (inite2 && e2 < delta) {
        delta = e2;
        affected_roots.clear();
        affected_roots.insert(root[augment_roots.first]);
        affected_roots.insert(root[augment_roots.second]);
    }
    if (inite3 && e3 < delta) {
        delta = e3;
        affected_roots.clear();
        affected_roots.insert(root[expand_root]);
    }

    if (delta == INF) {
        cout << "[DUAL UPDATE] No eligible delta found.\n";
        cout << "AUGMENT 2 EVEN BLOSSOM!" << endl;
        for (mcpm_node_idx i = origin_num; i < nl_size; ++i) { // blossom 영역역
            // EVEN + child.epmty() + dual = 0.0 -> 고립 -> 여러개 -> 따라서 둘 사이의 거리가 최소인 두 blossom을 강제 augment
            if (node_list[i].active && node_list[i].type == EVEN && node_list[i].dual_value < 1e-9 && child[i].empty()) {
                for (mcpm_node_idx j = i + 1; j < nl_size; ++j) {
                    if (node_list[j].active && node_list[j].type == EVEN && node_list[j].dual_value < 1e-9 && child[j].empty()) {
                        cout << "Before Augmenting: " << i << " and " << j << endl;
                        this -> Augment(i, j);
                        cout << "Augmenting: " << i << " and " << j << endl;
                        break;
                    }
                }
            } 
        }
        return;
    }

    cout << "[DUAL UPDATE] delta = " << delta << ", roots = ";
    for (auto r : affected_roots) cout << r << " ";
    cout << endl;

    // Dual value 적용
    for (mcpm_node_idx i = 0; i < nl_size; ++i) {
        if (!node_list[i].active) continue;
        if (affected_roots.find(root[i]) == affected_roots.end()) continue;

        if (node_list[i].type == EVEN) node_list[i].dual_value += delta;
        else if (node_list[i].type == ODD) node_list[i].dual_value -= delta;
    }

    // Expand 조건 적용
    for (mcpm_node_idx i = origin_num; i < nl_size; ++i) {
        if (node_list[i].active && node_list[i].type == ODD && fabs(node_list[i].dual_value) < 1e-9) {
            Expand(i);
        }
    }
}

// void blossomV::destroy_blossom() {
//     // 모든 blossom을 제거하고, 노드들을 초기 상태로 되돌리기
//     for (mcpm_node_idx i = origin_num; i < nl_size; ++i) {
//         if (node_list[i].active && node_list[i].is_blossom) {
//             for (mcpm_node_idx member : internal_node[i]) {
//                 this -> node_list[member].active = true;
//             }
//         }
//     }
// }



void blossomV::Reset() { // 일반 노드들의 pair와 dual을 제외한 나머지 초기화 ,  blossom은 초기화 대상 x

    queue<int> empty;
    swap(grow_queue, empty);

    for (mcpm_node_idx i = 0; i < this -> origin_num; i++) {
        auto& node = this -> node_list[i];
        if (!node.active) continue; 

        node.init_with_out_pair_and_active();
        this -> root[i] = i;
        this -> tip[i] = i;
        this -> internal_node[i].clear();
        this -> internal_node[i].push_back(i);
        this -> blo_tree[i].clear();
        this -> child[i].clear();
        //this -> max_dual_delta[i] = INF; // ODD가 들어오면 갱신되어야 함 
        if (node.pair == -1) {
            node.type = EVEN;
            this -> grow_queue.push(i);
        }
    }
}

void blossomV::Grow(mcpm_node_idx u, mcpm_node_idx v) {
    // 반드시 + None (pair임) 왜냐하면 모든 free vertex는 grow queue로 끌려감
    auto& v_node = this -> node_list[v];
    auto& vp_node = this -> node_list[v_node.pair];

    v_node.type = ODD;
    v_node.parent = u;
    v_node.visited = true;
    this -> root[v] = this -> root[u];
    this -> child[u].push_back(v);
    //this -> max_dual_delta[v] = min(this -> max_dual_delta[v], v_node.dual_value);

    vp_node.type = EVEN;
    vp_node.parent = v;
    this -> root[v_node.pair] = this -> root[this -> root[v]];
    this -> child[v].push_back(v_node.pair);
    this -> grow_queue.push(v_node.pair);
}

void blossomV::Augment(mcpm_node_idx u, mcpm_node_idx v) {
    this -> matching(u, v);

    this -> flip(u,v); // 일단 matching을 뒤집고

    this -> ExpandAll(u, v); // 이후에 다시 순회하면서 blossom 끊어놓기

}

void blossomV::Shrink(mcpm_node_idx u, mcpm_node_idx v) {
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
    this -> root[new_idx] = this -> root[lca];
    //cout << "new idx: " << new_idx << ", root: " << this -> root[new_idx] << endl;

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

    this -> root[lca] = new_idx; // 26  - (10, 9, 8) 이고 LCA가 10이라면 모든 내부 노드는 root를 26으로 하되, 26이 tip으로 10 가리킴

    // 5. tip, root 정보 업데이트
    this -> tip[new_idx] = lca;

    this -> node_list[new_idx].parent       = this -> node_list[lca].parent;
    this -> node_list[new_idx].x            = this -> node_list[this -> tip[new_idx]].x;
    this -> node_list[new_idx].y            = this -> node_list[this -> tip[new_idx]].y;
    this -> node_list[new_idx].type         = EVEN;
    this -> node_list[new_idx].visited      = true;
    this -> node_list[new_idx].active       = true;
    this -> node_list[new_idx].is_blossom   = true;
    this -> node_list[new_idx].dual_value   = 0.0; // 초기 dual 값은 0으로 설정
    this -> node_list[new_idx].pair         = -1; // 초기 pair는 없음

    // 6. 내부 노드 hide
    // Shrink 시 내부 노드를 숨기기
    for (auto i : this -> blo_tree[new_idx]) {
        this -> node_list[i].active = false;
    }
}

void blossomV::ExpandAll(mcpm_node_idx u, mcpm_node_idx v) {
    set<mcpm_node_idx> visited;
    queue<mcpm_node_idx> q;

    q.push(u);
    q.push(v);
    visited.insert(u);
    visited.insert(v);

    while (!q.empty()) {
        mcpm_node_idx curr = q.front(); q.pop();

        // 현재 노드가 blossom이면 Expand
        if (node_list[curr].is_blossom) {
            Expand(curr);
        }

        // 자식 노드로 내려감
        for (mcpm_node_idx c : child[curr]) {
            if (visited.count(c)) continue;
            visited.insert(c);
            q.push(c);
        }

        // 부모 노드로 올라감
        mcpm_node_idx p = node_list[curr].parent;
        if (p != -1 && !visited.count(p)) {
            visited.insert(p);
            q.push(p);
        }
    }
}


void blossomV::Expand(mcpm_node_idx b) {
    if (!this -> node_list[b].is_blossom || this -> blo_tree[b].empty()) return;

    mcpm_node_idx tip_node = tip[b];
    mcpm_node_idx b_pair = node_list[b].pair;

    // 내부 블로섬 먼저 재귀적으로 해제
    for (mcpm_node_idx x : this -> internal_node[b]) {
        if (this -> node_list[x].is_blossom) this -> Expand(x);
    }
    for (mcpm_node_idx x : blo_tree[b]) {
        if (this -> node_list[x].is_blossom) this -> Expand(x);
    }

    // tip 노드와 외부 pair 복원
    if (b_pair != -1) {
        this -> node_list[tip_node].pair = b_pair;
        this -> node_list[b_pair].pair = tip_node;
    }

    // 내부 블로섬 edge 복원 (odd cycle matching)
    auto it = blo_tree[b].begin();
    ++it; // tip 건너뛰고 시작
    while (it != this -> blo_tree[b].end()) {
        mcpm_node_idx u = *it++;
        if (it == this -> blo_tree[b].end()) break;
        mcpm_node_idx v = *it++;
        this -> node_list[u].pair = v;
        this -> node_list[v].pair = u;
    }

    // 내부 노드 상태 복원 및 트리 탈출
    for (mcpm_node_idx x : this -> internal_node[b]) {
        this -> node_list[x].type = UNLABELED;
        this -> node_list[x].visited = false;
        this -> node_list[x].parent = -1;
        this -> root[x] = x;
        this -> child[x].clear();
        this -> node_list[x].active = true;
    }

    for (mcpm_node_idx x : this -> blo_tree[b]) {
        this -> node_list[x].type = UNLABELED;
        this -> node_list[x].visited = false;
        this -> node_list[x].parent = -1;
        this -> root[x] = x;
        this -> child[x].clear();
        this -> node_list[x].active = true;
    }

    // 블로섬 자체 초기화 및 재사용 큐 등록
    this -> clean_blossom(b);
}


void blossomV::matching(mcpm_node_idx u, mcpm_node_idx v) {

    mcpm_node_idx tu = this -> tip[u]; 
    mcpm_node_idx tv = this -> tip[v]; 

    // 진짜 matching: tip <-> tip
    this -> node_list[tu].pair = tv;
    this -> node_list[tv].pair = tu; 

    if (tu != u) this -> node_list[u].pair = v;
    if (tv != v) this -> node_list[v].pair = u;

    this -> matching_num++;
}

void blossomV::flip(mcpm_node_idx u, mcpm_node_idx v) {

    this -> flip_upward(u);     // u는 항상 트리 말단이므로 위로만 가면 됨
    this -> flip_upward(v);     // v도 root까지 올라가며 flip
    this -> flip_downward(v);   // v에서 내려가며 자식 노드에 대해 flip 재귀적으로 수행

}

void blossomV::flip_upward(mcpm_node_idx u) {
    while (this -> node_list[u].parent != -1) {
        mcpm_node_idx p = this -> node_list[u].parent;
        mcpm_node_idx pp = this -> node_list[p].parent;
        this -> node_list[p].pair = pp;
        this -> node_list[pp].pair = p;
        u = pp;
    }
}

void blossomV::flip_downward(mcpm_node_idx u) {
    for (mcpm_node_idx c : child[u]) {
        this -> node_list[u].pair = c;
        this -> node_list[c].pair = u;
        this -> flip_downward(c);  // 재귀 호출
    }
}

void blossomV::reset_tree_state(mcpm_node_idx u) {
    this -> node_list[u].type = UNLABELED;
    this -> node_list[u].visited = false;
    this -> node_list[u].parent = -1;
    this -> root[u] = u;

    for (auto x : internal_node[u]) {
        this -> node_list[x].type = UNLABELED;
        this -> node_list[x].visited = false;
        this -> node_list[x].parent = -1;
        this -> root[x] = x;
    }

    this -> child[u].clear();
}


bool blossomV::check_tight(mcpm_node_idx u, mcpm_node_idx v){
    return fabs(this -> slack(u, v)) < 1e-9;  // https://www.reddit.com/r/learnprogramming/comments/9y9gxs/small_lesson_of_the_day_never_check_two_floats/
}

double blossomV::slack(mcpm_node_idx u, mcpm_node_idx v) {
    return this -> dist(u,v) - (this -> node_list[u].dual_value + this -> node_list[v].dual_value);
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

void blossomV::clean_blossom(mcpm_node_idx b) {
    this -> blo_tree[b].clear();
    this -> internal_node[b].clear();
    this -> tip[b] = -1;
    this -> node_list[b].free_blossom_node();  // pair, parent, type, visited, active 등 초기화
    this -> node_list[b].index_at_oddIndices = b;
    this -> child[b].clear();
    this -> recycle_idx.push(b);  // 재활용 큐에 넣기
}

void blossomV::debug_alternating_tree_state() {
    cout << "\n========= [ Alternating Tree Debug Info ] =========\n";
    for (mcpm_node_idx i = 0; i < this->nl_size; ++i) {
        const auto& node = this->node_list[i];

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

        cout << ", root=" << this->root[i];
        cout << ", tip=" << this->tip[i];
        cout << ", parent=" << node.parent;
        cout << ", pair=" << node.pair;
        cout << ", dual=" << fixed << setprecision(3) << node.dual_value;
        cout << ", visited=" << node.visited;

        cout << ", children=[";
        const auto& children = this->child[i];
        for (size_t j = 0; j < children.size(); ++j) {
            cout << children[j];
            if (j + 1 < children.size()) cout << ",";
        }
        cout << "]";

        cout << ", internal_node=[";
        const auto& in = this->internal_node[i];
        for (auto it = in.begin(); it != in.end(); ++it) {
            cout << *it;
            if (next(it) != in.end()) cout << ",";
        }
        cout << "]";

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
            double distance = this-> dist(u, v);
            mst_edges.emplace_back(u_node.index_at_nodes, v_node.index_at_nodes, distance);
            visited[u] = visited[v] = true;
        }
    }
}

void blossomV::print_mcpm_total_weight() {
    vector<bool> visited(this->nl_size, false);
    double total_weight = 0.0;

    for (mcpm_node_idx u = 0; u < this->origin_num; ++u) {
        mcpm_node_idx v = this->node_list[u].pair;

        if (v != -1 && !visited[u] && !visited[v]) {
            auto& u_node = this->node_list[u];
            auto& v_node = this->node_list[v];

            double dist = sqrt(
                (u_node.x - v_node.x) * (u_node.x - v_node.x) +
                (u_node.y - v_node.y) * (u_node.y - v_node.y)
            );

            total_weight += dist;
            visited[u] = visited[v] = true;
        }
    }

    cout << fixed << setprecision(6);
    cout << "MCPM Total Weight: " << total_weight << endl;
}
