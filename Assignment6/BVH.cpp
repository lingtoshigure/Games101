#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    //递归构建BVH树
    //root = recursiveBuild(primitives);
    //SAH优化
    root = recursiveBuildSAH(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    //创建根节点
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    //对三角面片的包围盒进行合并
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        //获取包围盒
        bounds = Union(bounds, objects[i]->getBounds());
    //递归结束条件，如果只有一个三角面片，则创建一个叶子节点
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    //如果有两个三角面片，则生成的节点分别记录指向的节点，且该节点实际记录的是两个节点合并后的包围盒
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    //包围盒对象的数量大于2个，则继续递归
    //获取所有三角面片的最大包围盒并按照质心分布重新排序
    else {
        //计算所有物体质心的联合边界
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        //
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        //最宽的轴为x轴
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        //最宽的轴在y轴
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        //最宽的轴在z轴
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }
        //划分初始的两个部分，递归构建BVH树
        // 
        // 
        //获取头指针
        auto beginning = objects.begin();
        //获取中间指针
        auto middling = objects.begin() + (objects.size() / 2);
        //获取尾指针
        auto ending = objects.end();

        //左子节点
        auto leftshapes = std::vector<Object*>(beginning, middling);
        //右子节点
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        //包围盒
        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

BVHBuildNode* BVHAccel::recursiveBuildSAH(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); i++)
    {
        bounds = Union(bounds, objects[i]->getBounds());
    }
    if (objects.size() == 1)
    {
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2)
    {
        node->left = recursiveBuild(std::vector{ objects[0] });
        node->right = recursiveBuild(std::vector{ objects[1] });
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else
    {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); i++)
        {
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
        }
        int dim = centroidBounds.maxExtent();

        float s_n = centroidBounds.SurfaceArea();
        int B = 10;
        int minCostIndex = 0;
        float minCost = std::numeric_limits<float>::infinity();//最小花费

        //选择跨度最大的坐标轴
        switch (dim)
        {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto b1, auto b2)
                {
                    return b1->getBounds().Centroid().x < b2->getBounds().Centroid().x;
                });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto b1, auto b2)
                {
                    return b1->getBounds().Centroid().y < b2->getBounds().Centroid().y;
                });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto b1, auto b2)
                {
                    return b1->getBounds().Centroid().z < b2->getBounds().Centroid().z;
                });
            break;
        }
        //遍历所有候选分割
        for (int i = 1; i < B; i++)
        {
            auto head = objects.begin();
            auto middle = objects.begin() + (objects.size() * i / B);
            auto tail = objects.end();
            auto leftPart = std::vector<Object*>(head, middle);
            auto rightPart = std::vector<Object*>(middle, tail);
            //左右包围盒
            Bounds3 leftBounds, rightBounds;
            for (int k = 0; k < leftPart.size(); k++)
            {
                leftBounds = Union(leftBounds, leftPart[k]->getBounds().Centroid());
            }
            for (int k = 0; k < rightPart.size(); k++)
            {
                rightBounds = Union(rightBounds, rightPart[k]->getBounds().Centroid());
            }
            float s_a = leftBounds.SurfaceArea();
            float s_b = rightBounds.SurfaceArea();
            //花费
            float cost = 0.125 + (leftPart.size() * s_a + rightPart.size() * s_b) / s_n;
            //记录花费更少的坐标
            if (cost < minCost)
            {
                minCost = cost;
                minCostIndex = i;
            }
        }

        //找到花费最少的坐标后续的操作与BVH相同
        auto head = objects.begin();
        auto middle = objects.begin() + objects.size()*minCostIndex/B;
        auto tail = objects.end();
        auto leftPart = std::vector<Object*>(head, middle);
        auto rightPart = std::vector<Object*>(middle, tail);
        
        node->left = recursiveBuildSAH(leftPart);
        node->right = recursiveBuildSAH(rightPart);
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
    Intersection result;
    //光线方向某个维度为负的情况下，会和pMax对应维度上的面上先相交
    std::array<int, 3> dirIsNeg = { (ray.direction.x > 0),(ray.direction.y > 0),(ray.direction.z > 0) };
    result.happened = node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg);
    if (!result.happened)
    {
        return result;
    }
    //当前为叶子节点
    if (node->left == nullptr && node->right == nullptr)
    {
        //与包围盒中的物体求交
        return node->object->getIntersection(ray);
    }
    Intersection left, right;
    left = getIntersection(node->left, ray);
    right = getIntersection(node->right, ray);
    //返回最近的交点
    if (left.distance < right.distance)
    {
        return left;
    }
    else 
    {
        return right;
    }
}