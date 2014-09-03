/**
* \file Limb.h
* \brief Helper struct that contains skeleton information and associate obj.
* also contains the Robot class which consists in the actual robot
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/
#ifndef _STRUCT_ROBOT
#define _STRUCT_ROBOT

#include "kinematics/joint.h"
#include "collision/Object.h"

#include <Eigen/Dense>

#include <exception>
#include <string>

namespace planner
{
typedef kinematics::joint<double, double, 3, 5, true> joint_t;

class Node
{
public:
    Node(const int id);
    Node(const Node& clone);
    ~Node();
    void free();

public:
    void SetRotation(int axis, double value);
    void SetRotation(double xvalue, double yvalue, double zvalue);
    void Translate(const Eigen::Vector3f& delta);
    void SetTranslation(const Eigen::Vector3f& position);

    Object* current;
    joint_t* joint;
    int jointOrder[3];
    double values[3];
    std::string tag;
    Node* parent;
    Eigen::Matrix3d toParentRotation;
    Eigen::Matrix3d toLocalRotation;
    Eigen::Matrix3d toWorldRotation;
    Eigen::Vector3d position;
    std::vector<Node*> children;
    const int id;

public:
    void Update();

private:
    Eigen::Vector3d axis1_;
    Eigen::Vector3d axis2_;
    Eigen::Vector3d axis3_;
};

Node* GetChild(Node* node, const std::string& tag);
Node* GetChild(Node* node, const int id);

Node* LoadRobot(const std::string& skeletonpath, const std::string& objectpath);

} //namespace planner
#endif //_STRUCT_ROBOT
