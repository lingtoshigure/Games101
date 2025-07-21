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
    bool happened;  //�Ƿ�����ײ
    Vector3f coords; //��ײ����������
    Vector3f normal; //��ײ������ķ���
    double distance; //��ײ������Դ�ľ���
    Object* obj; //��ײ������
    Material* m; //��ײ����Ĳ���
};
#endif //RAYTRACING_INTERSECTION_H
