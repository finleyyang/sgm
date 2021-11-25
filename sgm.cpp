//
// Created by finley on 24/11/2021.
//

#include "sgm.h"


void SGM::LoadImage() {
    depth = cv::Mat::zeros(imageleft.image.size(), CV_32S);
    subpixelSteps = 4;
    cv::Size size(imageleft.image.size().width-2 * halfWindowSizeX, imageleft.image.size().height-2 * halfWindowSizeY);
    disparity = cv::Mat::zeros(size, CV_32S);
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
    for(CGAL::Delaunay::Face_iterator it=delaunay.faces_begin(); it!=delaunay.faces_end(); ++it){
        const CGAL::Delaunay::Face& face = *it;
        Eigen::Vector3d fi0(face.vertex(0)->point().x(), face.vertex(0)->point().y(), face.vertex(0)->point().z());
        Eigen::Vector3d fi1(face.vertex(1)->point().x(), face.vertex(1)->point().y(), face.vertex(1)->point().z());
        Eigen::Vector3d fi2(face.vertex(2)->point().x(), face.vertex(2)->point().y(), face.vertex(2)->point().z());

        Eigen::Vector2d i0(fi0.x(), fi0.y());
        Eigen::Vector2d i1(fi1.x(), fi1.y());
        Eigen::Vector2d i2(fi2.x(), fi2.y());
        for(int u = 0; u<imageleft.image.cols; u++) {
            for (int v = 0; v < imageleft.image.rows; v++) {
                Eigen::Vector2d I(u, v);
                if(PointinTrangle(i0, i1, i2, I)){
                    Plane Planef(imageleft.camera.TransformPointI2C(fi0),
                                 imageleft.camera.TransformPointI2C(fi1),
                                 imageleft.camera.TransformPointI2C(fi2));
                    Eigen::Vector3d raydirection(imageleft.camera.TransformPointI22C(I));
                    Ray Rayp(raydirection);
                    Eigen::Vector3d pointinterset(Planef.Intersect(Rayp));
                    depth.at<double>(u, v) = pointinterset.z();
                }
            }
        }
    }
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
    float dot11 = v1.dot(v1);
    float dot12 = v1.dot(v2);

    float u = (dot00 * dot02 - dot01 * dot12) / (dot00 * dot11 - dot01 * dot01);
    float v = (dot00 * dot12 - dot01 * dot02) / (dot00 * dot11 - dot01 * dot01);
    if(u<0||u>1)
        return false;
    else if(v<0||v>1)
        return false;
    else if((u+v)<=1)
        return true;
    else return false;
}

void SGM::Depth2DisparityMap() {
    for(int u = 0; u<disparity.cols; u++) {
        for (int v = 0; v < disparity.rows; v++) {
            //x 是校正后的图像上的点， u是校正前的
            Eigen::Vector2d x(u, v);
            Eigen::Vector2d U;
            ProjectVertex_3x3_2_2(Hleft.inverse(), x, U);
            double d = depth.at<double>(U.x(), U.y());
            double disparityvalue;
            imageleft.Depth2Disparity(d, Qleft.inverse(), U, disparityvalue);
            disparity.at<double>(u, v) = ROUND2INT(disparityvalue * subpixelSteps);
        }
    }
}
