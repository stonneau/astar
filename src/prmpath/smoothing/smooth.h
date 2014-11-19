/**
* \file smooth.h
* \brief Helper struct to perform path computation and smoothing
* for an manipulation task
* \author Steve T.
* \version 0.1
* \date 09/05/2014
*
*/
#ifndef _HELPER_SMOOTHING
#define _HELPER_SMOOTHING

#include "collision/Collider.h"

#include <Eigen/Dense>

namespace planner
{
struct Model;

typedef std::vector<const Model*> CT_Model;
typedef std::pair<Eigen::Vector3d, Eigen::Matrix3d> Configuration;
typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> C2_Point;

struct ParamFunction
{
    ParamFunction(){}
    ~ ParamFunction(){}
    virtual C2_Point operator()(double t) const = 0;
    virtual C2_Point max() const = 0;
};

struct SplinePath : public ParamFunction
{
    SplinePath(const std::vector<Eigen::Vector3d>& controlPoints, const std::vector<Eigen::Vector3d>& controlPointsRot, const std::vector<double>& knots, double scale);
    ~SplinePath();

    virtual C2_Point operator()(double t) const;
    virtual C2_Point max() const;
    Configuration Evaluate(double t) const;

    const std::vector<Eigen::Vector3d> controlPoints_;
    const std::vector<Eigen::Vector3d> controlPointsRot_;
    const std::vector<double> knots_;
    const double scale_;
};
// Jia Pan Collision Free and Curvature Continuous Path Smoothing in cluttered environment
SplinePath* SplineFromPath(Collider& collider, CT_Model& path, double maxSpeed, double maxAcceleration);

SplinePath SplineShortCut(Collider& collider, CT_Model& path, int nbSteps);

} //namespace planner
#endif //_HELPER_SMOOTHING
