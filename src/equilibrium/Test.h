/**
* \file Test.h
* \brief Test for equilibirum functions
* \author Steve T.
* \version 0.1
* \date 29/07/2014
*
*/
#ifndef _STRUCT_TEST
#define _STRUCT_TEST

#include <Eigen/Dense>

namespace equilib
{

typedef Eigen::Vector3d Vector;
typedef Eigen::Matrix3d Rotation;
typedef Eigen::Matrix4d Transform;

void CheckEquilibrium(const Eigen::MatrixXd& contactTransforms, const Eigen::MatrixXd& graspTransforms, const Eigen::MatrixXd& maxGraspingForces,  float friction = 1.f, double flimit = 3000);

} //namespace planner
#endif //equilib
