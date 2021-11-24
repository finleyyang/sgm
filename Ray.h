//
// Created by finley on 24/11/2021.
//

#ifndef SGM_RAY_H
#define SGM_RAY_H
#include <Eigen/Eigen>

typedef Eigen::Vector3d Point;
typedef Eigen::Vector3d Vector;

class Plane;

class Ray{
private:
    Vector direction;
    Point origin;
public:
    Ray(){};
    ~Ray(){};

    Ray(const Vector &_direction);
    Ray(const Point &_original, const Vector &_direction);

    Vector getDirection() const;
    Point getOrigin() const;
};

#endif //SGM_RAY_H
