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

    //�ݹ鹹��BVH��
    //root = recursiveBuild(primitives);
    //SAH�Ż�
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
    //�������ڵ�
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    //��������Ƭ�İ�Χ�н��кϲ�
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        //��ȡ��Χ��
        bounds = Union(bounds, objects[i]->getBounds());
    //�ݹ�������������ֻ��һ��������Ƭ���򴴽�һ��Ҷ�ӽڵ�
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    //���������������Ƭ�������ɵĽڵ�ֱ��¼ָ��Ľڵ㣬�Ҹýڵ�ʵ�ʼ�¼���������ڵ�ϲ���İ�Χ��
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    //��Χ�ж������������2����������ݹ�
    //��ȡ����������Ƭ������Χ�в��������ķֲ���������
    else {
        //���������������ĵ����ϱ߽�
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        //
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        //������Ϊx��
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        //��������y��
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        //��������z��
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }
        //���ֳ�ʼ���������֣��ݹ鹹��BVH��
        // 
        // 
        //��ȡͷָ��
        auto beginning = objects.begin();
        //��ȡ�м�ָ��
        auto middling = objects.begin() + (objects.size() / 2);
        //��ȡβָ��
        auto ending = objects.end();

        //���ӽڵ�
        auto leftshapes = std::vector<Object*>(beginning, middling);
        //���ӽڵ�
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        //��Χ��
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
        float minCost = std::numeric_limits<float>::infinity();//��С����

        //ѡ��������������
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
        //�������к�ѡ�ָ�
        for (int i = 1; i < B; i++)
        {
            auto head = objects.begin();
            auto middle = objects.begin() + (objects.size() * i / B);
            auto tail = objects.end();
            auto leftPart = std::vector<Object*>(head, middle);
            auto rightPart = std::vector<Object*>(middle, tail);
            //���Ұ�Χ��
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
            //����
            float cost = 0.125 + (leftPart.size() * s_a + rightPart.size() * s_b) / s_n;
            //��¼���Ѹ��ٵ�����
            if (cost < minCost)
            {
                minCost = cost;
                minCostIndex = i;
            }
        }

        //�ҵ��������ٵ���������Ĳ�����BVH��ͬ
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
    //���߷���ĳ��ά��Ϊ��������£����pMax��Ӧά���ϵ��������ཻ
    std::array<int, 3> dirIsNeg = { (ray.direction.x > 0),(ray.direction.y > 0),(ray.direction.z > 0) };
    result.happened = node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg);
    if (!result.happened)
    {
        return result;
    }
    //��ǰΪҶ�ӽڵ�
    if (node->left == nullptr && node->right == nullptr)
    {
        //���Χ���е�������
        return node->object->getIntersection(ray);
    }
    Intersection left, right;
    left = getIntersection(node->left, ray);
    right = getIntersection(node->right, ray);
    //��������Ľ���
    if (left.distance < right.distance)
    {
        return left;
    }
    else 
    {
        return right;
    }
}