//
// Created by finley on 23/11/2021.
//

#ifndef SGM_COMMON_H
#define SGM_COMMON_H

#define INV_ZERO		(1e+14)
#define ZERO_TOLERANCE	(1e-7)
#include <Eigen/Eigen>


typedef Eigen::Matrix<double, 3, 4> PMatrix;
typedef Eigen::Matrix3d RMatrix;
typedef Eigen::Matrix3d KMatrix;
typedef Eigen::Vector3d CVector;
typedef Eigen::Vector3d tVector;

double INVERT(double x);
void SetRFromRowVector3d(RMatrix &R, Eigen::Vector3d e1, Eigen::Vector3d e2, Eigen::Vector3d e3);
bool ISZERO(double x);

double MAXVALUE(double x1, double x2);
double MINVALUE(double x1, double x2);

int ROUND2INT(double x);

void ProjectVertex_3x3_2_2(const Eigen::Matrix3d &H, const Eigen::Vector2d &X, Eigen::Vector2d &pt);
void FindVector2dMinMax(const std::vector<Eigen::Vector2d> &X, double &minX, double &maxX, double &minY, double &maxY, double &centerX, double &centerY);

#endif //SGM_COMMON_H
