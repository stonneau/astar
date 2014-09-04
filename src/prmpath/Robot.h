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
    void SetRotation(double value);
    void Translate(const Eigen::Vector3f& delta);
    void SetTranslation(const Eigen::Vector3f& position);

    Object* current;
    double value;
    std::string tag;
    Node* parent;
    Eigen::Matrix3d toParentRotation;
    Eigen::Matrix3d toLocalRotation;
    Eigen::Matrix3d toWorldRotation;
    Eigen::Vector3d position;

public: //*should be const*/
    const int id;
    Eigen::Vector3d axis;
    Eigen::Vector3d offset;
    std::vector<Node*> children;

public:
    void Update();


};

Node* GetChild(Node* node, const std::string& tag);
Node* GetChild(Node* node, const int id);

Node* LoadRobot(const std::string& urdfpath);

} //namespace planner
#endif //_STRUCT_ROBOT
