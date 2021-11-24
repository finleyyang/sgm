//
// Created by finley on 23/11/2021.
//

#ifndef SGM_PLANE_H
#define SGM_PLANE_H
#include "common.h"

typedef Eigen::Vector3d Point;
typedef Eigen::Vector3d Vector;

//平面方程Hessian normal form nx+d=0 <=> ax+by+cz=0, n是平面法向量

class Ray;

class Plane{
private:
    //平面法向量
    Vector normal;
    //原点到平面的距离
    double distance;
public:
    Plane(){};
    ~Plane(){};

    Plane(const Vector &_normal, const Point &n0);
    Plane(const Vector &_normal, const double &_distance);
    Plane(const Point &n0, const Point &n1, const Point &n2);

    double Distance(const Point &p);

    Point Intersect(const Ray &ray);
    Vector getNormal() const;
    double getDistance() const;
};


#endif //SGM_PLANE_H
