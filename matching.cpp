#include "matching.h"
#include <unordered_set>

using namespace std;

blossomV::blossomV(vector<mcpm_node>& list): node_list(list) {
    origin_num  = (int) list.size();
    nl_size     = 2 * origin_num;
    for(int i = origin_num; i < nl_size;i++) {
        mcpm_node node(-1,-1,-1,-1);
        node_list.push_back(node);
    } 
}

// blossomV::Grow - Refactored with external outer vector
void blossomV::Grow() {
    Reset();

    while (!grow_queue.empty()) {
        mcpm_node_idx w_idx = grow_queue.front();
        grow_queue.pop();

        mcpm_node_idx w_outer = outer[w_idx];

        for (mcpm_node_idx u : blo_tree[w_outer]) {
            for (mcpm_node_idx v = 0; v < origin_num; ++v) {
                if (u == v || !is_tight(u, v)) continue;

                mcpm_node_idx u_outer = outer[u];
                mcpm_node_idx v_outer = outer[v];
                if (node_list[v_outer].type == ODD) continue;

                if (node_list[v_outer].type != EVEN) {
                    mcpm_node_idx vm = node_list[v_outer].pair;

                    node_list[v_outer].parent = u;
                    node_list[v_outer].type = ODD;
                    node_list[v_outer].root = node_list[u_outer].root;

                    node_list[vm].parent = v;
                    node_list[vm].type = EVEN;
                    node_list[vm].root = node_list[u_outer].root;

                    if (!node_list[vm].visited) {
                        grow_queue.push(outer[vm]);
                        node_list[vm].visited = true;
                    }
                    continue;
                }

                if (node_list[u_outer].root != node_list[v_outer].root) {
                    Augment(u, v);
                    Reset();
                    return;
                }

                if (u_outer != v_outer) {
                    mcpm_node_idx b = Blossom(u, v);
                    grow_queue.push(b);
                    node_list[b].visited = true;
                    return;
                }
            }
        }
    }

    perfect = true;
    for (mcpm_node_idx i = 0; i < origin_num; ++i) {
        if (node_list[i].pair == -1) {
            perfect = false;
            break;
        }
    }
}

// blossomV::Expand - Refactored from Matching::Expand
void blossomV::Expand(mcpm_node_idx u, bool expandBlocked) {
    // 일반 노드이거나 expand 조건 미충족 시 탈출
    if (u < origin_num || (!expandBlocked && !node_list[u].is_blossom)) return;

    if (blo_path[u].empty()) return;

    // u와 매칭된 대표 노드
    mcpm_node_idx v = outer[node_list[u].pair];

    // 가장 tight한 (slack 최소) blossom 내부 간선 찾기
    double best_slack = 1e18;
    mcpm_node_idx p = -1, q = -1;

    for (mcpm_node_idx di : blo_tree[u]) {
        for (mcpm_node_idx dj : blo_tree[v]) {
            double slack = cal_slack(di, dj);
            if (std::fabs(slack) < EPS && slack < best_slack) {
                best_slack = slack;
                p = di;
                q = dj;
            }
        }
    }

    if (p == -1 || q == -1) return;  // tight edge 없음

    node_list[u].pair = q;
    node_list[v].pair = p;

    // p가 포함된 node를 찾을 때까지 blo_path[u]를 회전
    std::unordered_set<mcpm_node_idx> seen;
    while (!blo_path[u].empty()) {
        mcpm_node_idx front = blo_path[u].front();
        if (seen.count(front)) break;  // 무한 루프 방지
        seen.insert(front);

        bool found = false;
        for (mcpm_node_idx x : blo_tree[front]) {
            if (x == p) {
                found = true;
                break;
            }
        }

        if (found) break;

        // path 회전
        blo_path[u].push_back(front);
        blo_path[u].pop_front();
    }

    // 회전된 path를 따라 pair 재구성
    auto it = blo_path[u].begin();
    node_list[*it].pair = node_list[u].pair;
    ++it;

    while (it != blo_path[u].end()) {
        auto it_next = std::next(it);
        if (it_next == blo_path[u].end()) break;
        node_list[*it].pair = *it_next;
        node_list[*it_next].pair = *it;
        std::advance(it, 2);
    }

    // blossom 내부 노드들의 outer 복원
    for (mcpm_node_idx s : blo_path[u]) {
        outer[s] = s;
        for (mcpm_node_idx d : blo_tree[s]) {
            outer[d] = d;
        }
    }

    // 현재 블로섬은 더 이상 유효하지 않음
    node_list[u].active = false;
    return_free_blossom(u);

    // 내부 노드들에 대해 재귀적으로 Expand
    for (mcpm_node_idx s : blo_path[u]) {
        Expand(s, expandBlocked);
    }
}



// blossomV::Augment - Refactored from Matching::Augment
void blossomV::Augment(mcpm_node_idx u, mcpm_node_idx v) {
    mcpm_node_idx pu = outer[u];
    mcpm_node_idx pv = outer[v];

    // 양쪽 루트 저장
    mcpm_node_idx root_u = node_list[pu].root;
    mcpm_node_idx root_v = node_list[pv].root;

    // 초기 매칭
    node_list[pu].pair = pv;
    node_list[pv].pair = pu;

    Expand(pu);
    Expand(pv);

    // u 방향 루트까지 경로 따라가며 matching reverse
    while (true) {
        mcpm_node_idx fp = node_list[pu].parent;
        if (fp == -1) break;

        mcpm_node_idx gp = node_list[fp].parent;
        if (gp == -1) break;

        mcpm_node_idx fpo = outer[fp];
        mcpm_node_idx gpo = outer[gp];

        node_list[gpo].pair = fpo;
        node_list[fpo].pair = gpo;

        Expand(fpo);
        Expand(gpo);

        pu = gpo;
    }

    // v 방향도 동일하게 처리
    pu = pv;
    while (true) {
        mcpm_node_idx fp = node_list[pu].parent;
        if (fp == -1) break;

        mcpm_node_idx gp = node_list[fp].parent;
        if (gp == -1) break;

        mcpm_node_idx fpo = outer[fp];
        mcpm_node_idx gpo = outer[gp];

        node_list[gpo].pair = fpo;
        node_list[fpo].pair = gpo;

        Expand(fpo);
        Expand(gpo);

        pu = gpo;
    }
}

// blossomV::Reset - Refactored from Matching::Reset
void blossomV::Reset() {
    grow_queue = queue<mcpm_node_idx>();
    outer.resize(nl_size);
    tip.resize(nl_size);
    blo_tree.resize(nl_size);
    blo_path.resize(nl_size);

    for (mcpm_node_idx i = 0; i < outer.size(); ++i) {
        node_list[i].parent = -1;
        node_list[i].root = i;
        node_list[i].visited = false;

        // Blossom 노드가 대표이면서 active일 경우 제거
        if (i >= origin_num && outer[i] == i && node_list[i].active) {
            DestroyBlossom(i);
        }
    }

    for (mcpm_node_idx i = 0; i < origin_num; ++i) {
        if (node_list[i].pair == -1) {
            node_list[i].type = EVEN;
            if (!node_list[i].visited) {
                grow_queue.push(i);
                node_list[i].visited = true;
            }
        } else {
            node_list[i].type = UNLABELED;
        }
    }
}

// blossomV::DestroyBlossom - Refactored from Matching::DestroyBlossom
void blossomV::DestroyBlossom(mcpm_node_idx t) {
    // 일반 노드이거나 dual > 0인 blocked blossom은 제거하지 않음
    if (t < origin_num || (node_list[t].active && node_list[t].dual_value > EPS)) return;

    // shallow path를 따라 내부 노드들의 outer를 복원
    for (mcpm_node_idx s : blo_path[t]) {
        outer[s] = s;
        for (mcpm_node_idx d : blo_tree[s]) {
            outer[d] = d;
        }
        DestroyBlossom(s);  // 내부 블로섬에 대해서도 재귀 제거
    }

    node_list[t].active = false;
    node_list[t].pair = -1;
    return_free_blossom(t);
}

// blossomV::UpdateDualCosts - Refactored from Matching::UpdateDualCosts
void blossomV::UpdateDualCosts() {
    double e1 = 1e18, e2 = 1e18, e3 = 1e18;
    bool inite1 = false, inite2 = false, inite3 = false;

    for (int u = 0; u < origin_num; ++u) {
        for (int v = u + 1; v < origin_num; ++v) {
            double slack_uv = cal_slack(u, v);
            mcpm_node_idx ou = outer[u], ov = outer[v];

            if ((node_list[ou].type == EVEN && node_list[ov].type == UNLABELED) ||
                (node_list[ov].type == EVEN && node_list[ou].type == UNLABELED)) {
                if (slack_uv < e1 - EPS) e1 = slack_uv, inite1 = true;
            }

            if (ou != ov && node_list[ou].type == EVEN && node_list[ov].type == EVEN) {
                if (slack_uv < e2 - EPS) e2 = slack_uv, inite2 = true;
            }
        }
    }

    for (mcpm_node_idx i = origin_num; i < outer.size(); ++i) {
        if (node_list[i].active && outer[i] == i &&
            node_list[i].type == ODD && node_list[i].dual_value < e3 - EPS) {
            e3 = node_list[i].dual_value;
            inite3 = true;
        }
    }

    double e = 1e18;
    if (inite1) e = e1;
    if (inite2 && e > e2 / 2.0 + EPS) e = e2 / 2.0;
    if (inite3 && e > e3 + EPS) e = e3;

    for (mcpm_node_idx i = 0; i < outer.size(); ++i) {
        if (outer[i] != i || !node_list[i].active) continue;
        if (node_list[i].type == EVEN) node_list[i].dual_value += e;
        else if (node_list[i].type == ODD) node_list[i].dual_value -= e;
    }

    for (mcpm_node_idx i = origin_num; i < outer.size(); ++i) {
        if (node_list[i].dual_value > EPS) continue;

        if (node_list[i].active && node_list[i].pair == -1) {
            DestroyBlossom(i);
        } else if (node_list[i].active) {
            Expand(i);
        }
    }
}


// blossomV::Clear - Refactored from Matching::Clear
void blossomV::Clear() {
    edge_list.clear(); 

    free_blossoms = std::queue<mcpm_node_idx>();
    for (mcpm_node_idx i = outer.size() - 1; i >= origin_num; --i)
        return_free_blossom(i);

    for (mcpm_node_idx i = 0; i < outer.size(); ++i) {
        outer[i] = i;
        node_list[i].init();
        node_list[i].active = (i < origin_num);
        tip[i] = i;
        blo_tree[i].clear();
        blo_path[i].clear();
    }
}


// blossomV::SolveMinimumCostPerfectMatching
std::pair<std::list<int>, double> blossomV::SolveMinimumCostPerfectMatching(const std::vector<double>& cost) {
    Clear();

    perfect = false;
    while (!perfect) {
        Heuristic();
        Grow();
        UpdateDualCosts();
        Reset();
    }

    std::list<int> matching = RetrieveMatching();

    double obj = 0;
    for (int idx : matching) {
        auto [u, v] = edge_list[idx];
        obj += std::sqrt(std::pow(node_list[u].x - node_list[v].x, 2) +
                         std::pow(node_list[u].y - node_list[v].y, 2));
    }

    double dual_obj = 0;
    for (mcpm_node_idx i = 0; i < outer.size(); ++i) {
        if (i < origin_num || node_list[i].active) {
            dual_obj += node_list[i].dual_value;
        }
    }

    return {matching, obj};
}

// blossomV::RetrieveMatching - Refactored from Matching::RetrieveMatching
list<int> blossomV::RetrieveMatching() {
    list<int> result;

    // 모든 활성 블로섬 확장
    for (mcpm_node_idx i = origin_num; i < outer.size(); ++i) {
        if (node_list[i].active && outer[i] == i) {
            Expand(i, true);
        }
    }

    for (int i = 0; i < edge_list.size(); ++i) {
        auto [u, v] = edge_list[i];
        if (node_list[u].pair == v) {
            result.push_back(i);
        }
    }
    return result;
}

// blossomV::Heuristic - Refactored from Matching::Heuristic
void blossomV::Heuristic() {
    std::vector<int> degree(origin_num, 0);

    for (int u = 0; u < origin_num; ++u) {
        for (int v = u + 1; v < origin_num; ++v) {
            if (cal_slack(u, v) > EPS) continue;
            ++degree[u];
            ++degree[v];
        }
    }

    using pii = std::pair<int, int>;
    std::priority_queue<pii, std::vector<pii>, std::greater<pii>> min_heap;
    for (int i = 0; i < origin_num; ++i)
        min_heap.emplace(degree[i], i);

    std::vector<bool> used(origin_num, false);

    while (!min_heap.empty()) {
        auto [deg, u] = min_heap.top(); min_heap.pop();
        if (used[u] || node_list[u].pair != -1) continue;

        int best_v = -1, best_deg = INT_MAX;
        for (int v = 0; v < origin_num; ++v) {
            if (u == v || used[v]) continue;
            if (cal_slack(u, v) > EPS) continue;
            if (node_list[v].pair != -1) continue;

            if (degree[v] < best_deg) {
                best_deg = degree[v];
                best_v = v;
            }
        }

        if (best_v != -1) {
            node_list[u].pair = best_v;
            node_list[best_v].pair = u;
            used[u] = used[best_v] = true;
        }
    }
}

void blossomV::return_free_blossom(mcpm_node_idx i ) {
    free_blossoms.push(i);
}

mcpm_node_idx blossomV::get_free_blossom() {
    mcpm_node_idx i;
    if (free_blossoms.empty()) {
        for (mcpm_node_idx i = origin_num; i < nl_size; i++) {
            if (node_list[i].index_at_nodes == -1) {
                return i;
            }
        }
    }
    else {
        i = free_blossoms.front();
        free_blossoms.pop();
    }
    return i;
}

// slack 계산 = 거리 - (dual_u + dual_v)
double blossomV::cal_slack(mcpm_node_idx u, mcpm_node_idx v) const {
    const auto& u_node = node_list[u];
    const auto& v_node = node_list[v];

    double dx = static_cast<double>(u_node.x - v_node.x);
    double dy = static_cast<double>(u_node.y - v_node.y);
    double dist = std::sqrt(dx * dx + dy * dy);

    return dist - (u_node.dual_value + v_node.dual_value);
}

// slack ≈ 0 이면 tight 간선
bool blossomV::is_tight(mcpm_node_idx u, mcpm_node_idx v) const {
    return std::fabs(cal_slack(u, v)) < EPS;
}
