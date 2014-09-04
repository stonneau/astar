#include "Sample.h"
#include "prmpath/Jacobian.h"
#include "prmpath/Robot.h"

using namespace planner;
using namespace planner::sampling;

namespace
{
Node* AssignValuesRec(Node* node, std::vector<double>::const_iterator &cit, std::vector<double>::const_iterator& end)
{
    Node* res = node;
    if(cit != end)
    {
        node->SetRotation(*cit);
        ++cit;
        for(std::vector<Node*>::iterator it = node->children.begin();
            it != node->children.end() && cit != end; ++it)
        {
            res = AssignValuesRec(*it, cit, end);
        }
    }
    return res;
}

Eigen::Vector3d AssignValues(Node* root, const std::vector<double>& values)
{
    std::vector<double>::const_iterator cit = values.begin();
    std::vector<double>::const_iterator end = values.end();
    Node* son = AssignValuesRec(root, cit, end);
    root->Update();
    return root->toLocalRotation * son->position;
}

Eigen::Matrix3d GetJacobianProduct(Node* root)
{
    Jacobian j(root);
    return j.GetJacobianProduct();
}
}

Sample::Sample(Node* root, const std::vector<double>& values)
    : values(values)
    , effectorPosition(AssignValues(root, values))
    , jacobianProduct(GetJacobianProduct(root))
{
    // NOTHING
}

Sample::~Sample()
{
    // NOTHING
}

void planner::sampling::LoadSample(const Sample &sample, Node *root)
{
    AssignValues(root, sample.values);
}
