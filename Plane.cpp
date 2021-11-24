//
// Created by finley on 23/11/2021.
//
#include "Plane.h"
#include "Ray.h"
#include <iostream>

Plane::Plane(const Vector &_normal, const Point &n0) {
    normal=_normal;
    distance = -normal.dot(n0);
}

Plane::Plane(const Vector &_normal, const double &_distance) {
    normal = _normal;
    distance = _distance;
}

Plane::Plane(const Point &n0, const Point &n1, const Point &n2) {
    const Eigen::Vector3d edge1 = n1 - n0;
    const Eigen::Vector3d edge2 = n2 - n0;
    normal = edge1.cross(edge2).normalized();
    distance = -normal.dot(n0);
}

double Plane::Distance(const Point &p) {
    return normal.dot(p) + distance;
}

Point Plane::Intersect(const Ray &ray) {
    double Vd = normal.dot(ray.getDirection());
    std::cout<<Vd<<std::endl;
    double Vo = -Distance(ray.getOrigin());
    std::cout<<Vo<<std::endl;
    return ray.getOrigin() + (ray.getDirection() * Vo * INVERT(Vd));
}

Vector Plane::getNormal() const {
    return normal;
}

double Plane::getDistance() const {
    return distance;
}
