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

#include <vector>
#include <Eigen/Dense>

namespace equilib
{

typedef Eigen::Vector3d Vector;
typedef Eigen::Matrix3d Rotation;
typedef Eigen::Matrix4d Transform;
typedef std::vector<Transform> T_Transform;

bool CheckEquilibrium(const T_Transform& contactTransforms, const T_Transform& graspTransforms, const Eigen::VectorXd& maxGraspingForces,
                      const Eigen::Vector3d& acceleration, const Eigen::Vector3d& comLocation,
                      const float mass, float friction, double flimit);

double ResidualRadius(const T_Transform& contactTransforms, const T_Transform& graspTransforms, const Eigen::VectorXd& maxGraspingForces,
                      const Eigen::Vector3d& acceleration, const Eigen::Vector3d& comLocation,
                      const float mass, float friction, double flimit);

} //namespace planner
#endif //equilib
