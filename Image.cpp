//
// Created by finley on 24/11/2021.
//

#include "Image.h"


void Image::ImageRectify(Image imageleft, Image imageright,
                         std::vector<Eigen::Vector3d> pointsleft,
                         std::vector<Eigen::Vector3d> pointsright,
                         Eigen::Matrix4d &Qleft, Eigen::Matrix3d &Hleft,
                         Eigen::Matrix3d &Hright, cv::Mat &imageleftrecity, cv::Mat &imagerightrecity) {
    RMatrix R1, R2;
    KMatrix K1, K2;
    double t;
    t = camera.CameraRectify(imageleft.GetSize(), imageleft.camera, imageright.GetSize(), imageright.camera, R1, R2, K1, K2);

    cv::Size size1(imageleft.GetSize()), size2(imageright.GetSize());

    camera.CameraRectifyROI(pointsleft, size1, imageleft.camera, pointsright, size2, imageright.camera, R1, R2, K1, K2);

    Eigen::Matrix3d H1 = K1 * R1 * imageleft.camera.GetKinv(); Hleft = H1;
    Eigen::Matrix3d H2 = K2 * R2 * imageright.camera.GetKinv(); Hright =H2;
    cv::Mat H1mat;
    cv::Mat H2mat;
    cv::eigen2cv(H1, H1mat);
    cv::eigen2cv(H2, H2mat);
    imageleftrecity.create(size1, imageleft.image.type());
    cv::warpPerspective(imageleft.image, imageleftrecity, H1mat, imageleftrecity.size());
    imagerightrecity.create(size2, imageright.image.type());
    cv::warpPerspective(imageright.image, imagerightrecity, H2mat, imagerightrecity.size());
    Qleft=  Eigen::Matrix4d::Zero();
    Eigen::Matrix4d Pleft = Eigen::Matrix4d::Identity();
    // Q * 矫正后的图像坐标以及视差值 = 矫正前的相机坐标以及深度值
    //                                     |1,  0,  0    ,       -cx|
    //  Q =  |Kl(Rlnew)-1,  0| * Q'   Q' = |0,  1,  0    ,       -cy|
    //       |0          ,  1|             |0,  0,  0    ,         f|
    //                                     |0,  0, -1/T  ,(cl- cr)/T|
    //
    //  B = -Tx
    //   Q * [x, y, disparity, 1] = [X, Y, Z, 1] * w
    Qleft(0,0) = Qleft(1,1) = 1;
    Qleft(0,3) = -K1(0,2);
    Qleft(1,3) = -K1(1,2);
    Qleft(2,3) =  K1(0,0);
    Qleft(3,2) = -(1)/t;
    Qleft(3,3) =  (K1(0,2)-K2(0,2))/t;

    Pleft.block<3, 3>(0,0) = imageleft.camera.GetK() * R1.inverse();
    Qleft = Pleft * Qleft;
}

void Image::Depth2Dispairty() {

}

