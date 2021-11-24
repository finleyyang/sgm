//
// Created by finley on 24/11/2021.
//

#ifndef SGM_IMAGE_H
#define SGM_IMAGE_H
#include "Camera.h"
class Camera;

class Image{
public:
    Camera camera;
    cv::Mat image;
    cv::Mat imagegray;

    Image(Camera _camera): camera(_camera){};
    ~Image(){};

    void ImageRectify(Image imageleft, Image imageright,std::vector<Eigen::Vector3d> pointsleft, std::vector<Eigen::Vector3d> pointsright,
                      Eigen::Matrix4d &Qleft, Eigen::Matrix3d &Hleft, Eigen::Matrix3d &Hright, cv::Mat &imageleftrecity, cv::Mat &imagerightrecity);
    cv::Size GetSize() const{return image.size();}
};

#endif //SGM_IMAGE_H
