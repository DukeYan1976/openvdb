/// @file SurfaceMesher.cpp
/// 从点法式集合重建视觉三角形网格
/// 算法: KD-tree + MLS隐式场 + 稀疏Marching Cubes
#include "core/SurfaceMesher.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_map>
#include <numeric>

namespace midgard {
namespace {

// ═══════════════════ KD-tree ═══════════════════
struct KDNode { int idx, splitAxis, left, right; };

struct KDTree {
    std::vector<KDNode> nodes;
    const double* pts;
    size_t count;

    void build(const double* p, size_t n) {
        pts = p; count = n;
        if (!n) return;
        nodes.reserve(n);
        std::vector<int> ids(n);
        std::iota(ids.begin(), ids.end(), 0);
        buildR(ids.data(), (int)n, 0);
    }
    int buildR(int* idx, int n, int dep) {
        if (n <= 0) return -1;
        int axis = dep % 3, mid = n / 2;
        std::nth_element(idx, idx+mid, idx+n, [&](int a, int b){
            return pts[a*3+axis] < pts[b*3+axis]; });
        int ni = (int)nodes.size();
        nodes.push_back({idx[mid], axis, -1, -1});
        nodes[ni].left  = buildR(idx, mid, dep+1);
        nodes[ni].right = buildR(idx+mid+1, n-mid-1, dep+1);
        return ni;
    }

    struct Nb { int idx; double d2; bool operator<(const Nb& o)const{return d2<o.d2;} };

    void knn(const double q[3], int k, double maxR2, std::vector<Nb>& res) const {
        res.clear();
        if (nodes.empty()) return;
        knnR(0, q, k, maxR2, res);
    }
    void knnR(int ni, const double q[3], int k, double& maxR2, std::vector<Nb>& h) const {
        if (ni < 0) return;
        auto& nd = nodes[ni];
        const double* p = pts + nd.idx*3;
        double dx=q[0]-p[0], dy=q[1]-p[1], dz=q[2]-p[2];
        double d2 = dx*dx+dy*dy+dz*dz;
        if (d2 < maxR2) {
            if ((int)h.size() < k) {
                h.push_back({nd.idx, d2});
                if ((int)h.size()==k) std::make_heap(h.begin(),h.end());
            } else if (d2 < h.front().d2) {
                std::pop_heap(h.begin(),h.end());
                h.back()={nd.idx,d2};
                std::push_heap(h.begin(),h.end());
                maxR2 = h.front().d2;
            }
        }
        double diff = q[nd.splitAxis] - p[nd.splitAxis];
        int first = diff<0 ? nd.left : nd.right;
        int second= diff<0 ? nd.right: nd.left;
        knnR(first, q, k, maxR2, h);
        if (diff*diff < maxR2) knnR(second, q, k, maxR2, h);
    }
};

