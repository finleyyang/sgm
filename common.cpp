//
// Created by finley on 24/11/2021.
//

#include "common.h"

double INVERT(double x){
    if(x!=0)
        return 1/x;
    else
        return INV_ZERO;
}

bool ISZERO(double x){
    return abs(x) < ZERO_TOLERANCE;
}

double MAXVALUE(double x1, double x2){return x1 > x2? x1 : x2;}

double MINVALUE(double x1, double x2){return x1 < x2? x1 : x2;}


void SetRFromRowVector3d(RMatrix &R, Eigen::Vector3d e1, Eigen::Vector3d e2, Eigen::Vector3d e3){
    R(0, 0) = e1[0];
    R(0, 1) = e1[1];
    R(0, 2) = e1[2];
    R(1, 0) = e2[0];
    R(1, 1) = e2[1];
    R(1, 2) = e2[2];
    R(2, 0) = e3[0];
    R(2, 1) = e3[1];
    R(2, 2) = e3[2];
}

int ROUND2INT(double x){
    return int(floor(x+0.5f));
}

void ProjectVertex_3x3_2_2(const Eigen::Matrix3d &H, const Eigen::Vector2d &X, Eigen::Vector2d &pt){
    double Zinv = INVERT(H(2, 0) * X[0] + H(2, 1) * X[1] + H(2, 2));
    pt[0] = (H(0,0) * X[0] + H(0, 1) * X[1] + H(0, 2)) * Zinv;
    pt[1] = (H(1,0) * X[0] + H(1, 1) * X[1] + H(1, 2)) * Zinv;
}



void FindVector2dMinMax(const std::vector<Eigen::Vector2d> &X, double &minX, double &maxX, double &minY, double &maxY, double &centerX, double &centerY){
    minX = X[0](0); maxX = X[0](0);
    minY = X[0](1); maxY = X[0](1);
    int size = X.size();
    for (int i = 0; i < size; ++i) {
        minX = MINVALUE(minX, X[i](0));
        maxX = MAXVALUE(maxX, X[i](0));
        minY = MINVALUE(minY, X[i](1));
        maxY = MAXVALUE(maxY, X[i](1));
    }

    centerX = (minX + maxX)*0.5;
    centerY = (minY + maxY)*0.5;
}
