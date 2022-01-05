#include <algorithm>
#include <cassert>
#include "BVH.hpp"
#include "Vector.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

struct BucketInfo
{
    int count = 0;
    Bounds3 bounds;
};

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        std::vector<Object*> leftshapes;
        std::vector<Object*> rightshapes;

        switch (splitMethod) {
        case SplitMethod::NAIVE: {
            Bounds3 centroidBounds;
            for (int i = 0; i < objects.size(); ++i)
                centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
            int dim = centroidBounds.maxExtent();  // 选一条跨度最大的轴进行划分，使划分空间尽量均匀
            switch (dim) {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                        f2->getBounds().Centroid().x;
                    });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                        f2->getBounds().Centroid().y;
                    });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z <
                        f2->getBounds().Centroid().z;
                    });
                break;
            }

            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);
            auto ending = objects.end();

            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            break;
        }
        case SplitMethod::SAH: {
            Bounds3 centroidBounds;
            for (int i = 0; i < objects.size(); ++i) {
                centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
            }
            int dim = centroidBounds.maxExtent();  // 选一条跨度最大的轴进行划分，使划分空间尽量均匀

            // 1. 初始化桶
            const int nBuckets = 32;
            BucketInfo buckets[nBuckets];
            for (int i = 0; i < objects.size(); i++) {
                auto tmp = nBuckets * centroidBounds.Offset(objects[i]->getBounds().Centroid());
                int b;
                switch (dim){
                case 0: b = tmp.x; case 1:b = tmp.y; case 2:b = tmp.z;
                }

                if (b == nBuckets) b = nBuckets - 1;
                buckets[b].count++;
                buckets[b].bounds = Union(buckets[b].bounds, objects[i]->getBounds());
            }

            // 2. 计算桶边界代价函数
            float costs[nBuckets - 1];
            for (int i = 0; i < nBuckets - 1; i++) {
                Bounds3 bounds1, bounds2;
                int count1 = 0, count2 = 0;
                for (int j = 0; j < i + 1; j++) {
                    count1 += buckets[j].count;
                    bounds1 = Union(bounds1, buckets[j].bounds);
                }
                for (int j = i + 1; j < nBuckets; j++) {
                    count2 += buckets[j].count;
                    bounds2 = Union(bounds2, buckets[j].bounds);
                }
                costs[i] = 0.125 + (count1 * bounds1.SurfaceArea() + count2 * bounds2.SurfaceArea() ) / centroidBounds.SurfaceArea();
            }
            // 3. 选出代价最小边界
            float minCost = costs[0];
            int bIdx = 0;
            for (int i = 0; i < nBuckets-1; i++) {
                if (costs[i] < minCost) {
                    bIdx = i;
                    minCost = costs[i];
                }
            }

            for (int i = 0; i < objects.size(); i++) {
                auto tmp = nBuckets * centroidBounds.Offset(objects[i]->getBounds().Centroid());
                int b;
                switch (dim) {
                case 0: b = tmp.x; case 1:b = tmp.y; case 2:b = tmp.z;
                }
                if (b == nBuckets) b = nBuckets - 1;
                if (b <= bIdx) {
                    leftshapes.push_back(objects[i]);
                }
                else {
                    rightshapes.push_back(objects[i]);
                }
            }
            break;
            }
        }

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        
    }
    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection isect;

    if (!node) {
        return isect;
    }

    std::array<int, 3> dirIsNeg;
    for (int i = 0; i < 3; i++) {
        dirIsNeg[i] = ray.direction_inv[i] > 0 ? 0 : 1;
    }

    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg)) {
        return isect;
    }
    if (node->object != nullptr) {
        return node->object->getIntersection(ray);
    }

    Intersection i1 = getIntersection(node->left, ray);
    Intersection i2 = getIntersection(node->right, ray);

    return i1.distance < i2.distance ? i1 : i2;
}