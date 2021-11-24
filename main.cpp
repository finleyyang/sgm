#include <iostream>


#include "Plane.h"
#include "Ray.h"

int main() {
    std::cout << "Hello, World!" << std::endl;
    Point A0(1,0,0);
    Point A1(0,1,0);
    Point A2(0,0,1);
    Plane A(A0, A1, A2);
    std::cout<<A.getNormal().transpose()<<std::endl;
    std::cout<<A.getDistance()<<std::endl;
    Vector B0(1,1,1);
    Point B1(0,0,0);
    Ray B(B1, B0);
    std::cout<<B.getDirection().transpose()<<std::endl;
    std::cout<<B.getOrigin().transpose()<<std::endl;
    Point C(A.Intersect(B));
    std::cout<<C.transpose()<<std::endl;

}
