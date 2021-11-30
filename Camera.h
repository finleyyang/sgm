//
// Created by finley on 24/11/2021.
//

#ifndef SGM_CAMERA_H
#define SGM_CAMERA_H
#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <utility>

#include "common.h"


class Camera{
private:
    RMatrix R;
    KMatrix K;
    CVector C;
    tVector t;
    PMatrix P;


public:

    Camera(RMatrix _R, KMatrix _K) : R(std::move(_R)), K(std::move(_K)){};
    ~Camera(){};
    void SetT(const Eigen::Vector3d & T) { t = T; C = R.inverse() * (-T); }
    void SetC(const Eigen::Vector3d & _C) { C = _C; t = R * (-C);}

    Eigen::Vector3d GetT() const { return t;}
    Eigen::Vector3d GetC() const { return C;}
    Eigen::Matrix3d GetR() const { return R;}
    Eigen::Matrix3d GetK() const { return K;}

    /* equivalent to R.t() * Vec(0,0,1) */
    Eigen::Vector3d Direction() const { return R.row(2);}

    Eigen::Matrix3d GetKinv() {return K.inverse();}

    /*P = K[R|t], P = KR[I|-C]*/
    void ComposeP();

    double CameraRectify(cv::Size sizeleft, Camera cameraleft, cv::Size sizeright, Camera cameraright, RMatrix &R1, RMatrix &R2, KMatrix &K1, KMatrix &K2);

    void ComputeRelativePose(RMatrix Rleft, RMatrix Rright, CVector Cleft, CVector Cright, RMatrix &Rlr, CVector &Clr);

    Eigen::Vector3d TransformPointW2C(Eigen::Vector3d X);
    Eigen::Vector2d TransformPointC2I(Eigen::Vector3d X);
    Eigen::Vector2d TransformPointC2I(Eigen::Vector2d X);
    Eigen::Vector2d TransformPointW2I(Eigen::Vector3d X);
    Eigen::Vector3d TransformPointW2I3(Eigen::Vector3d X);
    Eigen::Vector3d TransformPointI2C(Eigen::Vector3d X);
    Eigen::Vector3d TransformPointI22C(Eigen::Vector2d X);


    void CameraRectifyROI(std::vector<Eigen::Vector3d> pointsleft, cv::Size &Size1, Camera camera1,
                          std::vector<Eigen::Vector3d> pointsright, cv::Size &Size2, Camera camera2,
                          RMatrix R1, RMatrix R2, KMatrix &K1, KMatrix &K2);

    void GetImagePairROI(const std::vector<Eigen::Vector3d> &pointsleft,const RMatrix &R1, const KMatrix &K1,
                         const std::vector<Eigen::Vector3d> &pointsright, const RMatrix &R2, const KMatrix &K2,
                         const KMatrix &invK1, const KMatrix &invK2, std::vector<Eigen::Vector2d> &roi1h, std::vector<Eigen::Vector2d> &roi2h);

    void SetCameraMatricesROI(const std::vector<Eigen::Vector2d> &roi1h, const std::vector<Eigen::Vector2d> &roi2h,
                              cv::Size& size1, cv::Size& size2, Eigen::Matrix3d& K1, Eigen::Matrix3d& K2);
};

#endif //SGM_CAMERA_H
