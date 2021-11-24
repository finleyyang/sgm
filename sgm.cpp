//
// Created by finley on 24/11/2021.
//

#include "sgm.h"


void SGM::LoadImage() {
    depth = cv::Mat::zeros(imageleft.image.size(), CV_32S);
}



void SGM::SetCameraPoints() {
    for (int i = 0; i < points.size(); ++i) {
        pointsleft.push_back(imageleft.camera.TransformPointW2I3(points[i]));
        pointsright.push_back(imageright.camera.TransformPointW2I3(points[i]));
    }
}



void SGM::Rectify() {
    imageleft.ImageRectify(imageleft, imageright, pointsleft, pointsright, Qleft, Hleft, Hright, imageleftrecity, imagerightrecity);

}

void SGM::DepthInit() {
    CGAL::Delaunay delaunay;
    TriangulatePointsDelaunay(delaunay);

}

std::pair<double, double> SGM::TriangulatePointsDelaunay(CGAL::Delaunay delaunay) {
    std::pair<double, double> depthbounds(FLT_MAX, 0.f);
    int size = points.size();
    for(int i = 0; i<size; i++){
        Eigen::Vector3d pt (imageleft.camera.TransformPointW2I3(points[i]));
        //这里插入的是u，v和深度
        delaunay.insert(CGAL::Point(pt.x(), pt.y(), pt.z()));
        if(depthbounds.first > pt.z())
            depthbounds.first = pt.z();
        if(depthbounds.second < pt.z())
            depthbounds.second = pt.z();
    }
    double avgdepth = (depthbounds.first + depthbounds.second)*0.5;
    delaunay.insert(CGAL::Point(0, 0, avgdepth));
    delaunay.insert(CGAL::Point(imageleft.image.cols, 0, avgdepth));
    delaunay.insert(CGAL::Point(0, imageleft.image.rows, avgdepth));
    delaunay.insert(CGAL::Point(imageleft.image.cols, imageleft.image.rows, avgdepth));

}

bool SGM::PointinTrangle(const Eigen::Vector2d &A, const Eigen::Vector2d &B, const Eigen::Vector2d &C, const Eigen::Vector2d &P) {
    Eigen::Vector2d v0 = C - A;
    Eigen::Vector2d v1 = B - A;
    Eigen::Vector2d v2 = P - A;

    float dot00 = v0.dot(v0);
    float dot01 = v0.dot(v1);
    float dot02 = v0.dot(v2);

}
