//
// Created by finley on 24/11/2021.
//

#include "Camera.h"

// 点从相机坐标深度为1的平面，转到图像坐标
Eigen::Vector2d Camera::TransformPointC2I(Eigen::Vector2d X) {
    Eigen::Vector2d x(K(0, 0) * X.x() + K(0, 2), K(1,1) * X.y() + K(1, 2));
    return x;
}
// 点从世界坐标转相机坐标
Eigen::Vector3d Camera::TransformPointW2C(Eigen::Vector3d X){
    return R * (X - C);
}
// 点从相机坐标转图像坐标
Eigen::Vector2d Camera::TransformPointC2I(Eigen::Vector3d X){
    Eigen::Vector2d x(X.x()/X.z(), X.y()/X.z());
    return TransformPointC2I(x);
}

// 点从世界坐标转图像坐标
Eigen::Vector2d Camera::TransformPointW2I(Eigen::Vector3d X){
    return TransformPointC2I(TransformPointW2C(X));
}

// 点从世界坐标转图像坐标加深度
Eigen::Vector3d Camera::TransformPointW2I3(Eigen::Vector3d X){
    Eigen::Vector3d camX(TransformPointW2C(X));
    Eigen::Vector3d I3d;
    I3d.head(2)  = TransformPointC2I(camX);
    I3d(2) = camX(2);
    return I3d;
};
// 点从图像坐标转相机坐标
Eigen::Vector3d Camera::TransformPointI2C(Eigen::Vector3d X) {
    Eigen::Vector3d x((X.x()-K(0,2))*X.z()/K(0,0),
                      (X.y()-K(1,2))*X.z()/K(1,1),
                      X.z());
    return x;
}


void Camera::ComposeP() {
    P.block<3, 3>(0, 0) = R;
    P.block<3, 1>(0, 3) = t;
}

double
Camera::CameraRectify(cv::Size sizeleft, Camera cameraleft, cv::Size sizeright, Camera cameraright, RMatrix &R1, RMatrix &R2,
                      KMatrix &K1, KMatrix &K2){

    RMatrix poseR;
    CVector poseC;
    ComputeRelativePose(cameraleft.R, cameraright.R, cameraleft.C, cameraright.C, poseR, poseC);

    // 新的x轴，基线方向
    const Eigen::Vector3d e1(cameraright.C - cameraleft.C);
    // 新的y轴，垂直旧的Z轴（光轴）和新的X轴
    const Eigen::Vector3d e2(cameraleft.Direction().cross(e1));
    // 新的Z轴，垂直上面两个新轴
    const Eigen::Vector3d e3(e1.cross(e2));

    RMatrix R;
    SetRFromRowVector3d(R, e1.normalized(), e2.normalized(), e3.normalized());

    R1 = R * cameraleft.R.inverse();
    R2 = R * cameraright.R.inverse();

    Eigen::Vector3d t = R2 * (poseR * (-poseC));
    return t.x();
}

void
Camera::ComputeRelativePose(RMatrix Rleft, RMatrix Rright, CVector Cleft, CVector Cright, RMatrix &Rlr, CVector &Clr) {
    Rlr = Rright * Rleft.inverse();
    Clr = Rleft * (Cright - Cleft);
}

void
Camera::CameraRectifyROI(std::vector<Eigen::Vector3d> pointsleft, cv::Size &Size1, Camera camera1,
                         std::vector<Eigen::Vector3d> pointsright, cv::Size &Size2, Camera camera2,
                         RMatrix R1, RMatrix R2, KMatrix &K1, KMatrix &K2){
    K1(0,0) = K2(0,0) = (K1(0,0)+K2(0,0))/2;
    std::vector<Eigen::Vector2d> roi1h, roi2h;
    GetImagePairROI(pointsleft, R1, K1, pointsright, R2, K2, camera1.GetKinv(), camera2.GetKinv(), roi1h, roi2h);
    SetCameraMatricesROI(roi1h, roi2h, Size1, Size2, K1, K2);
}

//这里的 point 是[x/z, y/z, z]
void Camera::GetImagePairROI(const std::vector<Eigen::Vector3d> &pointsleft,const RMatrix &R1, const KMatrix &K1,
                             const std::vector<Eigen::Vector3d> &pointsright, const RMatrix &R2, const KMatrix &K2,
                             const KMatrix &invK1, const KMatrix &invK2, std::vector<Eigen::Vector2d> &roi1h, std::vector<Eigen::Vector2d> &roi2h){

    // 计算单应性矩阵，将原图像的坐标转换到校正后图像对应坐标
    // 原图像坐标->相机坐标系下的坐标->投影到矫正后的坐标下->矫正后图像坐标
    // 原图像坐标->相机坐标系下的坐标 invK1 * P
    // 投影到矫正后的相机下的坐标  R1 * invK1 * P
    // 矫正之后的图像坐标 K1 * R1 * invK1 * P
    const Eigen::Matrix3d H1(K1 * R1 * invK1);
    const Eigen::Matrix3d H2(K2 * R2 * invK2);
    int size = pointsleft.size();
    for(int i = 0; i < size; i++){
        Eigen::Vector3d X1 = pointsleft[i];
        Eigen::Vector3d X2 = pointsright[i];
        Eigen::Vector2d x1, x2;
        ProjectVertex_3x3_2_2(H1, X1, x1);
        roi1h.push_back(x1);
        ProjectVertex_3x3_2_2(H2, X2, x2);
        roi2h.push_back(x2);
    }
}

void Camera::SetCameraMatricesROI(const std::vector<Eigen::Vector2d> &roi1h, const std::vector<Eigen::Vector2d> &roi2h,
                                  cv::Size &size1, cv::Size &size2, Eigen::Matrix3d &K1, Eigen::Matrix3d &K2) {
    double roi1h_max_x, roi1h_min_x, roi1h_max_y, roi1h_min_y, roi1h_center_x, roi1h_center_y;
    double roi2h_max_x, roi2h_min_x, roi2h_max_y, roi2h_min_y, roi2h_center_x, roi2h_center_y;
    FindVector2dMinMax(roi1h, roi1h_min_x, roi1h_max_x, roi1h_min_y, roi1h_max_y, roi1h_center_x, roi1h_center_y);
    FindVector2dMinMax(roi2h, roi2h_min_x, roi2h_max_x, roi2h_min_y, roi2h_max_y, roi2h_center_x, roi2h_center_y);
    size1.width = ROUND2INT(roi1h_max_x - roi1h_min_x);
    size1.height = ROUND2INT(roi1h_max_y - roi1h_min_y);
    size2.width = ROUND2INT(roi2h_max_x - roi2h_min_x);
    size2.height = ROUND2INT(roi2h_max_y - roi2h_min_y);
    K1(0,2) += size1.width /2-roi1h_center_x;
    K1(1,2) += size1.height/2-roi1h_center_y;
    K2(0,2) += size2.width /2-roi2h_center_x;
    K2(1,2) += size2.height/2-roi2h_center_y;
}
