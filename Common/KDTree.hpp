// KDTree — 2D spatial index for efficient nearest-neighbor queries.
#pragma once
#include <vector>
#include <algorithm>
#include <queue>
#include <memory>

class KDTree {
public:
    struct Point {
        double lat, lon;
        int nodeId;
    };

    void build(std::vector<Point> points) {
        root = buildRecursive(points, 0, (int)points.size(), 0);
    }

    std::vector<int> kNearest(double lat, double lon, int k) const {
        // Max-heap keeps the farthest of the k best on top for easy replacement
        std::priority_queue<std::pair<double, int>> best;
        search(root.get(), lat, lon, k, best);

        std::vector<int> result;
        result.reserve(best.size());
        while (!best.empty()) {
            result.push_back(best.top().second);
            best.pop();
        }
        std::reverse(result.begin(), result.end());
        return result;
    }

private:
    struct Node {
        Point point;
        int dim; // 0 = split on lat, 1 = split on lon
        std::unique_ptr<Node> left, right;
    };

    std::unique_ptr<Node> root;

    static double sqDist(double lat1, double lon1, double lat2, double lon2) {
        double dl = lat1 - lat2, dn = lon1 - lon2;
        return dl * dl + dn * dn;
    }

    std::unique_ptr<Node> buildRecursive(std::vector<Point>& pts, int lo, int hi, int depth) {
        if (lo >= hi) return nullptr;

        int dim = depth % 2;
        int mid = (lo + hi) / 2;

        // Partition so the median lands at mid
        std::nth_element(pts.begin() + lo, pts.begin() + mid, pts.begin() + hi,
            [dim](const Point& a, const Point& b) {
                return dim == 0 ? a.lat < b.lat : a.lon < b.lon;
            });

        auto node = std::make_unique<Node>();
        node->point = pts[mid];
        node->dim = dim;
        node->left  = buildRecursive(pts, lo, mid, depth + 1);
        node->right = buildRecursive(pts, mid + 1, hi, depth + 1);
        return node;
    }

    static void search(const Node* node, double lat, double lon, int k,
                       std::priority_queue<std::pair<double, int>>& best) {
        if (!node) return;

        double d = sqDist(lat, lon, node->point.lat, node->point.lon);

        if ((int)best.size() < k) {
            best.push({d, node->point.nodeId});
        } else if (d < best.top().first) {
            best.pop();
            best.push({d, node->point.nodeId});
        }

        // Visit the side the query point falls on first
        double diff = (node->dim == 0) ? (lat - node->point.lat) : (lon - node->point.lon);
        const Node* near = diff <= 0 ? node->left.get() : node->right.get();
        const Node* far  = diff <= 0 ? node->right.get() : node->left.get();

        search(near, lat, lon, k, best);

        // Prune: only cross the splitting plane if it could contain closer points
        if ((int)best.size() < k || diff * diff < best.top().first) {
            search(far, lat, lon, k, best);
        }
    }
};
