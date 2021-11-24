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
        delaunay.insert(CGAL::Point(pt.x(), pt.y(), pt.z()));
        if(depthbounds.first > pt.z())
            depthbounds.first = pt.z();
        if(depthbounds.second < pt.z())
            depthbounds.second = pt.z();
    }
    double avgdepth = (depthbounds.first + depthbounds.second)*0.5;
    const CGAL::VertexHandle vcorners[]={
            delaunay.insert(CGAL::Point(0, 0, avgdepth)),
            delaunay.insert(CGAL::Point(imageleft.image.cols, 0, avgdepth)),
            delaunay.insert(CGAL::Point(0, imageleft.image.rows, avgdepth)),
            delaunay.insert(CGAL::Point(imageleft.image.cols, imageleft.image.rows, avgdepth))
    };
    const int numpoints = 3;
    for(int i = 0; i< 4; i++){
        const CGAL::VertexHandle vcorner = vcorners[i];
        CGAL::FaceCirculator cfc(delaunay.incident_faces(vcorner));
        if(cfc == 0)
            continue;
        const CGAL::FaceCirculator done(cfc);
        Eigen::Vector3d poszA(vcorner->point().x(), vcorner->point().y(), vcorner->point().z());
        Eigen::Vector2d posA(vcorner->point().x(), vcorner->point().y());
        //Eigen::Vector3d rayA(imageleft.camera.TransformPointI2C(poszA));
        const Ray rayA(imageleft.camera.TransformPointI2C(poszA));
        do{
            CGAL::FaceHandle fc(cfc->neighbor(cfc->index(vcorner)));
            if(fc == delaunay.infinite_face())
                continue;
            for(int j = 0; j<4;j++)
                if(fc->has_vertex(vcorners[j]))
                    goto Continue;
            {
                const Eigen::Vector3d poszB0 (fc->vertex(0)->point().x(), fc->vertex(0)->point().y(), fc->vertex(0)->point().z());
                const Eigen::Vector3d poszB1 (fc->vertex(0)->point().x(), fc->vertex(0)->point().y(), fc->vertex(0)->point().z());
                const Eigen::Vector3d poszB2 (fc->vertex(0)->point().x(), fc->vertex(0)->point().y(), fc->vertex(0)->point().z());
                Plane planeB(poszB0, poszB1, poszB2);
                const Eigen::Vector3d poszB(planeB.Intersect(rayA));
                if(poszB.z()<=0)
                    continue;

            }
            Continue:;
        }while(++cfc!=done);
    }
}
