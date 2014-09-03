#include "Robot.h"
#include "kinematics/joint_io.h"
#include "collision/ParserObj.h"

using namespace planner;

Node::Node(const int id)
    : current(0)
    , joint(0)
    , tag("")
    , parent(0)
    , id(id)
    , axis1_(Eigen::Vector3d::UnitZ())
    , axis2_(Eigen::Vector3d::UnitY())
    , axis3_(Eigen::Vector3d::UnitX())
{
    for(int i=0; i<3; ++i)
    {
        values[i] = 0;
        jointOrder[i] = i;
    }
}

Node::Node(const Node& clone)
    : current(new Object(*(clone.current)))
    , joint(clone.joint->clone())
    , tag(clone.tag)
    , parent(0)
    , id(clone.id)
{
    for(int i=0; i<3; ++i)
    {
        values[i] = clone.values[i];
        jointOrder[i] = clone.jointOrder[i];
    }
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
    delete joint;
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
    toWorldRotation = Eigen::AngleAxisd(values[jointOrder[0]], axis1_)
                    * Eigen::AngleAxisd(values[jointOrder[1]], axis2_)
                    * Eigen::AngleAxisd(values[jointOrder[2]], axis3_).matrix();
    toParentRotation = toWorldRotation;
    if(parent)
    {
        toWorldRotation = parent->toWorldRotation * toWorldRotation;
        toParentRotation = parent->toLocalRotation * toWorldRotation;
    }
    toLocalRotation = toWorldRotation;
    toLocalRotation.transpose();
    current->SetOrientation(toWorldRotation);
    for(int i =0; i<3; ++i)
    {
        position(i) = joint->offset[i];
    }
    if(parent)
    {
        position = parent->toWorldRotation * position;
    }
    else
    {
        position = toWorldRotation * position;
    }
    if(parent) position += parent->position;
    current->SetOrientation(toWorldRotation);
    current->SetPosition(position);
    for(std::vector<Node*>::iterator cit = children.begin();
        cit != children.end(); ++cit)
    {
        (*cit)->Update();
    }
}

void Node::SetRotation(int axis, double value)
{
    values[axis] = value;
    Update();
}

void Node::SetRotation(double xvalue, double yvalue, double zvalue)
{
    values[0] = xvalue; values[1] = yvalue; values[0] = zvalue;
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

namespace
{
    Node* CreateNodeRec(joint_t* current, const Object* currentObj, int& id)
    {
        Node* res = new Node(id++);
        res->current = new Object(*currentObj);
        res->joint = current;
        for(int i =0; i< current->nbChildren_; ++i)
        {
            Node* son = CreateNodeRec(current->children[i], currentObj, id);
            son->parent = res;
            res->children.push_back(son);
        }
        return res;
    }
}

planner::Node* planner::LoadRobot(const std::string& skeletonpath, const std::string& objectpath)
{
    joint_t* root = kinematics::ReadTree<double, double, 3, 5, true>(skeletonpath);
    Object::T_Object objects = ParseObj(objectpath);
    int id = 0;
    Node* res = CreateNodeRec(root, objects[0], id);
    res->Update();
    return res;
}
