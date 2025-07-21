//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"
class Object;
class Sphere;

struct Intersection
{
    Intersection(){
        happened=false;
        coords=Vector3f();
        normal=Vector3f();
        distance= std::numeric_limits<double>::max();
        obj =nullptr;
        m=nullptr;
    }
    bool happened;  //是否发生碰撞
    Vector3f coords; //碰撞发生的坐标
    Vector3f normal; //碰撞发生点的法线
    double distance; //碰撞点距离光源的距离
    Object* obj; //碰撞的物体
    Material* m; //碰撞物体的材质
};
#endif //RAYTRACING_INTERSECTION_H
