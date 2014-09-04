#include "Robot.h"
#include "kinematics/joint_io.h"
#include "collision/ParserObj.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <map>
#include <Eigen/Geometry>
using namespace planner;
Node::Node(const int id)
    : current(0)
    , value(0)
    , tag("")
    , parent(0)
    , toParentRotation(Eigen::Matrix3d::Identity())
    , toLocalRotation(Eigen::Matrix3d::Identity())
    , toWorldRotation(Eigen::Matrix3d::Identity())
    , position(0,0,0)
    , id(id)
    , axis(0,0,1)
    , offset(0,0,0)
    , permanentRotation(Eigen::Matrix3d::Identity())
{
    //NOTHING
}
Node::Node(const Node& clone)
    : current(new Object(*(clone.current)))
    , value(clone.value)
    , tag(clone.tag)
    , parent(0)
    , toParentRotation(clone.toParentRotation)
    , toLocalRotation(clone.toLocalRotation)
    , toWorldRotation(clone.toWorldRotation)
    , position(clone.position)
    , id(clone.id)
    , axis(clone.axis)
    , offset(clone.offset)
    , permanentRotation(clone.permanentRotation)
{
    for(std::vector<Node*>::const_iterator cit = clone.children.begin();
        cit != children.end(); ++cit)
    {
        Node * child = new Node(*(*cit));
        child->parent = this;
        children.push_back(child);
    }
}
Node::~Node()
{
    delete current;
}
void Node::free()
{
    for(std::vector<Node*>::iterator cit = children.begin();
        cit != children.end(); ++cit)
    {
        (*cit)->free();
    }
    delete this;
}
void Node::Update()
{
    toWorldRotation = Eigen::AngleAxisd(value, axis).matrix();
    toParentRotation = toWorldRotation;
    if(parent)
    {
        toWorldRotation = parent->toWorldRotation * toWorldRotation;
        toParentRotation = parent->toLocalRotation * toWorldRotation;
        position = parent->toWorldRotation * offset;
        position += parent->position;
    }
    else
    {
        position = toWorldRotation * offset;
    }
    toLocalRotation = toWorldRotation;
    toLocalRotation.inverse();
    if(current)
    {
        current->SetOrientation(toWorldRotation);
        current->SetPosition(position);
    }
    for(std::vector<Node*>::iterator cit = children.begin();
        cit != children.end(); ++cit)
    {
        (*cit)->Update();
    }
}
void Node::SetRotation(double value)
{
    this->value = value;
    Update();
}
void Node::Translate(const Eigen::Vector3d& delta)
{
    offset += delta;
    Update();
}
void Node::SetTranslation(const Eigen::Vector3d& position)
{
    offset = position;
    Update();
}
planner::Node* planner::GetChild(Node* node, const std::string& tag)
{
    if(node->tag == tag) return node;
    planner::Node* res;
    for(std::vector<Node*>::iterator cit = node->children.begin();
        cit != node->children.end(); ++cit)
    {
        res = GetChild((*cit), tag);
        if(res) return res;
    }
    return 0;
}
planner::Node* planner::GetChild(Node* node, const int id)
{
    if(node->id == id) return node;
    planner::Node* res;
    for(std::vector<Node*>::iterator cit = node->children.begin();
        cit != node->children.end(); ++cit)
    {
        res = GetChild((*cit), id);
        if(res) return res;
    }
    return 0;
}
planner::Node* planner::GetChild(Robot* robot, const std::string& tag)
{
    return GetChild(robot->node, tag);
}
planner::Node* planner::GetChild(Robot* robot, const int id)
{
    return GetChild(robot->node, id);
}
Robot::Robot(Node* root)
    : node(root)
    , constantRotation(Eigen::Matrix3d::Identity())
    , currentRotation(Eigen::Matrix3d::Identity())
    , currentPosition(Eigen::Vector3d::Zero())
{
    // NOTHING
}
Robot::~Robot()
{
    delete node;
}
void Robot::SetConfiguration(const planner::Object* object)
{
    SetRotation(object->GetOrientation(), true);
    SetPosition(object->GetPosition(), true);
}
void Robot::SetRotation(const Eigen::Matrix3d& rotation, bool update)
{
    currentRotation = rotation * constantRotation;
    Eigen::Vector3d ea = currentRotation.eulerAngles(2,1,0);
    Node* current = node; // first node is translation
    for(int i =0; i<3; ++i)
    {
        current = current->children[0];
        current->value = ea[i];
    }
    if(update) node->Update();
}
void Robot::SetConstantRotation(const Eigen::Matrix3d& rotation)
{
    constantRotation = rotation;
    currentRotation = currentRotation * constantRotation;
    Eigen::Vector3d ea = rotation.eulerAngles(2,1,0);
    Node* current = node; // first node is translation
    for(int i =0; i<3; ++i)
    {
        current = current->children[0];
        current->value = ea[i];
    }
    node->Update();
}
void Robot::SetPosition(const Eigen::Vector3d& position, bool update)
{
    currentPosition = position;
    node->offset = currentPosition;
    if(update) node->Update();
}
void Robot::Translate(const Eigen::Vector3d& delta, bool update)
{
    currentPosition += delta;
    node->offset = currentPosition;
    if(update) node->Update();
}
namespace
{
struct Joint;
struct Link
{
    Link()
        : object(0)
    {
        // nothing
    }
    ~Link()
    {
        // nothing
    }
    planner::Object* object;
    std::string name;
    std::vector<Joint*> children;
};
struct Joint
{
    Joint()
        : parentLink(0)
        , childLink("")
    {
        // nothing
    }
    ~Joint()
    {
        // nothing
    }
    Eigen::Vector3d axis;
    Eigen::Vector3d offset;
    std::string name;
    Link* parentLink;
    std::string childLink;
    std::string type;
};
std::string ExtractQuotes(const std::string& line)
{
    int quoteStart = line.find("\"");
    int quoteEnd = line.find("\"", quoteStart+1);
    return line.substr(quoteStart+1, quoteEnd - quoteStart -1);
}
std::string ExtractType(const std::string& line)
{
    int quoteStart = line.find("\"");
    int quoteEnd = line.find("\"", quoteStart +1);
    quoteStart = line.find("\"", quoteEnd+1);
    quoteEnd = line.find("\"", quoteStart+1);
    return line.substr(quoteStart+1, quoteEnd - quoteStart -1);
}
Eigen::Vector3d VectorFromString(const std::string& line)
{
    char x[255],y[255],z[255];
    sscanf(line.c_str(),"%s %s %s",x,y,z);
    return Eigen::Vector3d(strtod (x, NULL), strtod(y, NULL), strtod(z, NULL));
}
Eigen::Vector3d ExtractOffset(const std::string& line)
{
    // skiiping rpy
    int quoteStart = line.find("\"");
    int quoteEnd = line.find("\"", quoteStart +1);
    quoteStart = line.find("\"", quoteEnd+1);
    quoteEnd = line.find("\"", quoteStart+1);
    return VectorFromString(line.substr(quoteStart+1, quoteEnd - quoteStart -1));
    /*for(int i =0; i<3; ++i)
    {
        joint->offset[i] = res(i);
    }*/
}
void ReadLink(const std::string& firstline, std::ifstream& file, std::map<std::string, Link*>& links)
{
    std::string objpath;
    Link* lk = new Link();
    lk->name = ExtractQuotes(firstline);
    bool foundMesh(false);
    std::string line;
    if(firstline.find("<link name") != std::string::npos && firstline.find("/>") == std::string::npos)
    {
        while (line.find("</link>") == std::string::npos && file.good())
        {
            getline(file, line);
            if(line.find("<mesh filename") != std::string::npos)
            {
                foundMesh = true;
                objpath = ExtractQuotes(line);
            }
        }
    }
    if(foundMesh)
    {
        Object::T_Object object = ParseObj(objpath, true);
        if(object.size() >0)
        {
            lk->object = object[0];
        }
    }
    if(links.find(lk->name) != links.end())
    {
        std::cout << "error in urdf file link redefinition" << lk->name << std::endl;
    }
    else
    {
        links.insert(std::make_pair(lk->name, lk));
    }
}
void ReadJoint(const std::string& firstline, std::ifstream& file, std::map<std::string, Joint*>& joints, std::map<std::string, Link*>& links)
{
    std::string name = ExtractQuotes(firstline);
    Joint * joint = new Joint();
    joint->name = name;
    if(firstline.find("<joint name") != std::string::npos && firstline.find("/>") == std::string::npos)
    {
        // get type
        joint->type = ExtractType(firstline);
        std::string line;
        while (line.find("</joint>") == std::string::npos && file.good())
        {
            getline(file, line);
            if(line.find("<axis") != std::string::npos)
            {
                joint->axis = VectorFromString(ExtractQuotes(line));
            }
            if(line.find("<origin") != std::string::npos)
            {
                joint->offset =ExtractOffset(line);
            }
            if(line.find("<parent link") != std::string::npos)
            {
                std::map<std::string, Link*>::iterator it = links.find(ExtractQuotes(line));
                if(it == links.end())
                {
                    std::cout << "error in urdf file missing parent joint" << ExtractQuotes(line) << std::endl;
                }
                else
                {
                    joint->parentLink = it->second;
                    it->second->children.push_back(joint);
                }
            }
            if(line.find("<child link") != std::string::npos)
            {
                joint->childLink = ExtractQuotes(line);
            }
        }
    }
    if(joints.find(name) != joints.end())
    {
        std::cout << "error in urdf file joint redefinition" << name << std::endl;
    }
    else
    {
        joints.insert(std::make_pair(name, joint));
    }
}
void MakeNodeRec(Node* node, Joint* current, std::map<std::string, Link*>& links, int& id)
{
    Link* next= links[current->childLink];
    if(current->type == "revolute")
    {
        Node* res = new Node(id++);
        res->axis = current->axis;
        res->tag = current->name;
        res->offset = current->offset;
        res->parent = node;
        if(next->object)
            res->current = next->object;
        node->children.push_back(res);
        node = res;
    }
    for(std::vector<Joint*>::iterator it = next->children.begin();
        it != next->children.end(); ++it)
    {
        MakeNodeRec(node, *it, links, id);
    }
}
Node* MakeNode(Link* roots, std::map<std::string, Link*>& links)
{
    //create root
    int id = 3;
    Node* res = new Node(id++);
    if(roots->object) res->current = roots->object;
    for(std::vector<Joint*>::iterator it = roots->children.begin();
        it != roots->children.end(); ++it)
    {
        MakeNodeRec(res, *it, links, id);
    }
    // create 3 dummy nodes for rotation + 1 for translation
    // make X
    {
        res->tag = "root_x_joint";
        res->axis = Eigen::Vector3d::UnitX();
        res->offset = Eigen::Vector3d::Zero();
    }
    Node* ynode = new Node(2);
    {
        ynode->tag = "root_y_joint";
        ynode->axis = Eigen::Vector3d::UnitY();
        ynode->offset = Eigen::Vector3d::Zero();
        ynode->children.push_back(res);
        res->parent = ynode;
    }
    Node* znode = new Node(1);
    {
        znode->tag = "root_z_joint";
        znode->axis = Eigen::Vector3d::UnitZ();
        znode->offset = Eigen::Vector3d::Zero();
        znode->children.push_back(ynode);
        ynode->parent = znode;
    }
    Node* tnode = new Node(0);
    {
        tnode->tag = "root_translation_joint";
        tnode->axis = Eigen::Vector3d::UnitZ();
        tnode->offset = Eigen::Vector3d::Zero();
        tnode->children.push_back(znode);
        znode->parent = tnode;
    }
    tnode->Update();
    return tnode;
}
}
planner::Node* planner::LoadRobot(const std::string& urdfpath)
{
    std::map<std::string, Link*> links;
    std::map<std::string, Joint*> joints;
    //*Read all links first*/
    std::ifstream myfile (urdfpath);
    if (myfile.is_open())
    {
        std::string line;
        while (myfile.good())
        {
            getline(myfile, line);
            if(line.find("<link name") != std::string::npos)
            {
                ReadLink(line, myfile, links);
            }
        }
        myfile.close();
        std::ifstream myfile2 (urdfpath);
        while (myfile2.good())
        {
            getline(myfile2, line);
            if(line.find("<joint name") != std::string::npos)
            {
                ReadJoint(line, myfile2, joints, links);
            }
        }
        myfile2.close();
        std::map<std::string, Link*>::iterator it = links.find("root_link");
        if(it == links.end())
        {
            std::cout << "error in urdf file missing root joint" << std::endl;
        }
        else
        {
            Link* root = it->second;
            return MakeNode(root, links);
        }
    }
    return 0;
}
