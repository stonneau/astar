/**
* \file Sample.h
* \brief Helper struct that contains angle values for given joint
* articulation.
* \author Steve T.
* \version 0.1
* \date 09/04/2014
*
*/
#ifndef _STRUCT_SAMPLE
#define _STRUCT_SAMPLE

#include <vector>
#include <Eigen/Dense>

#include "prmpath/Robot.h"

namespace planner
{
class Node;
namespace sampling
{

struct Sample
{
     Sample(Node* root, const std::vector<double>& values);
    ~Sample();

    const std::vector<double> values;
    const Eigen::Vector3d effectorPosition;
    const Eigen::Matrix3d jacobianProduct;
};

typedef std::vector<Sample*> T_Samples;
void LoadSample(const Sample& sample, Node* root);

T_Samples GenerateSamples(const planner::Robot& robot, const Node *root, int nbSamples);

} // namespace sampling
} // namespace planner
#endif //_STRUCT_SAMPLE
