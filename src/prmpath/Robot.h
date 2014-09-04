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
#include "collision/Object.h"
#include <Eigen/Dense>
#include <exception>
#include <string>
namespace planner
{
class Node
{
public:
    Node(const int id);
    Node(const Node& clone);
    ~Node();
    void free();
public:
    void SetRotation(double value);
    //only for root
public:
    void Translate(const Eigen::Vector3d& delta);
    void SetTranslation(const Eigen::Vector3d& position);

public:
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
    Eigen::Matrix3d permanentRotation;
    std::vector<Node*> children;
public:
    void Update();
};
class Robot
{
public:
    Robot(Node* root);
    ~Robot();
public:
    void SetConfiguration(const planner::Object* object);
    void SetConstantRotation(const Eigen::Matrix3d& rotation);
    void SetRotation(const Eigen::Matrix3d& rotation, bool update = true);
    void SetPosition(const Eigen::Vector3d& position, bool update = true);
    void Translate  (const Eigen::Vector3d& delta, bool update = true);
public:
    Node* node;
    Eigen::Matrix3d constantRotation;
    Eigen::Matrix3d currentRotation;
    Eigen::Vector3d currentPosition;
};

int GetNumChildren(const Node* node);
std::vector<Node*> GetEffectors(Node* node);
Node* GetChild(Node* node, const std::string& tag);
Node* GetChild(Node* node, const int id);
Node* GetChild(Robot *robot, const std::string& tag);
Node* GetChild(Robot *robot, const int id);
Node* LoadRobot(const std::string& urdfpath);
} //namespace planner
#endif //_STRUCT_ROBOT
