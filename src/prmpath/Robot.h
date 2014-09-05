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
#include "prmpath/ROM.h"
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

//constraints
    double minAngleValue;
    double maxAngleValue;
    double defaultAngleValue;

public: //*should be const*/
    const int id;
    Eigen::Vector3d axis;
    Eigen::Vector3d offset;
    Eigen::Matrix3d permanentRotation;
    int romId;
    std::vector<Node*> children;

public:
    void Update();
    bool IsLocked();
};

struct NodeRom
{
    typedef Node* NodeGroup [3];

     NodeRom(const rom::ROM& rom)
         : rom(rom){}
    ~NodeRom(){}

    bool InsideJointLimits() const
    {
        return rom.ResidualRadius(group[0]->value, group[1]->value, group[2]->value) >= 0;
    }

    bool InsideJointLimits(const double ea1, const double ea2, const double ea3) const
    {
        return rom.ResidualRadius(ea1, ea2, ea3) >= 0;
    }

    bool ResidualRadius() const
    {
        return rom.ResidualRadius(group[0]->value, group[1]->value, group[2]->value);
    }

    double ResidualRadius(const double ea1, const double ea2, const double ea3) const
    {
        return rom.ResidualRadius(ea1, ea2, ea3);
    }

    NodeGroup group;
    rom::ROM rom;
    int groupId;
};

class Robot
{
public:
    Robot(Node* root);
    Robot(const Robot& clone);
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
    std::vector<NodeRom*> roms;
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
