//
// Created by finley on 24/11/2021.
//

#ifndef SGM_SGM_H
#define SGM_SGM_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Projection_traits_xy_3.h>

#include <math.h>

#include "Image.h"
#include "Plane.h"
#include "Ray.h"

enum : int { halfWindowSizeX = 3, halfWindowSizeY = 4 };

namespace CGAL {
    typedef CGAL::Simple_cartesian<double> kernel_t;
    typedef CGAL::Projection_traits_xy_3<kernel_t> Geometry;
    typedef CGAL::Delaunay_triangulation_2<Geometry> Delaunay;
    typedef CGAL::Delaunay::Face_circulator FaceCirculator;
    typedef CGAL::Delaunay::Face_handle FaceHandle;
    typedef CGAL::Delaunay::Vertex_circulator VertexCirculator;
    typedef CGAL::Delaunay::Vertex_handle VertexHandle;
    typedef kernel_t::Point_3 Point;
}

using namespace CGAL;

class SGM{
private:
    Image imageleft;
    Image imageright;
    Eigen::Matrix4d Qleft;
    Eigen::Matrix3d Hleft;
    Eigen::Matrix3d Hright;
    cv::Mat imageleftrecity;
    cv::Mat imagerightrecity;
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> pointsleft;
    std::vector<Eigen::Vector3d> pointsright;
    double subpixelSteps;
    cv::Mat depth;
    cv::Mat disparity;
    float dmax;
    float dmin;

public:

    void LoadImage();
    void SetCameraPoints();
    void Rectify();
    void DepthInit();
    std::pair<double, double> TriangulatePointsDelaunay(CGAL::Delaunay delaunay);
    bool PointinTrangle(const Eigen::Vector2d &A, const Eigen::Vector2d &B, const Eigen::Vector2d &C, const Eigen::Vector2d &P);
    void Depth2DisparityMap();
};


#endif //SGM_SGM_H
