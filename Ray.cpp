//
// Created by finley on 24/11/2021.
//

#include "Ray.h"

Ray::Ray(const Vector &_direction) {
    origin << 0, 0, 0;
    direction = _direction;
}

Ray::Ray(const Point &_original, const Vector &_direction) {
    origin = _original;
    direction = _direction;
}

Vector Ray::getDirection() const {
    return direction;
}

Point Ray::getOrigin() const {
    return origin;
}

