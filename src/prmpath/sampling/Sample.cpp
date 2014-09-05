#include "Sample.h"
#include "prmpath/Jacobian.h"
#include "prmpath/Robot.h"

#include <vector>
#include <iostream>

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


namespace
{
    static bool generatorInit = false;
    const int maxTrials = 10000;

    void GenerateJointAngle(Node* node, std::vector<double>& values, int rootId)
    {
         double min = node->minAngleValue;
         double max = node->maxAngleValue;
         node->value = ((double) rand() / (RAND_MAX)) * (max - min) + min;
         int index = node->id - rootId;
         if(values.size() > index)
         {
             values[index] = node->value;
         }
         else
         {
             values.push_back(node->value);
         }
    }

    void GenerateSampleRec(Node* node, std::vector<double>& values, const int rootId)
    {
        GenerateJointAngle(node, values, rootId);
        for(std::vector<Node*>::iterator cit = node->children.begin();
            cit != node->children.end(); ++cit)
        {
            GenerateSampleRec(*cit, values, rootId);
        }
    }

    planner::sampling::Sample* GenerateSample(const std::vector<NodeRom*>& groups, Node* root)
    {
        std::vector<double> values;
        GenerateSampleRec(root, values, root->id);
        for(std::vector<NodeRom*>::const_iterator cit = groups.begin();
            cit!=groups.end(); ++cit)
        {
            bool insideJointLimits = (*cit)->InsideJointLimits();
            for(int i=0; !insideJointLimits && i< maxTrials; ++i)
            {
                for(int k=0; k<3; ++k)
                {
                    GenerateJointAngle((*cit)->group[k], values, root->id);
                }
                insideJointLimits = (*cit)->InsideJointLimits();
            }
            if(!insideJointLimits)
            {
                return 0;
            }
        }
        return new planner::sampling::Sample(root, values);
    }

    void GetConcernedGroups(const Robot &robot, Node* node, std::vector<int>& ids, std::vector<NodeRom*>& roms)
    {
        if(node->romId !=-1 && std::find(ids.begin(), ids.end(), node->romId)!=ids.end())
        {
            ids.push_back(node->romId);
            roms.push_back(robot.roms[node->romId]);
        }
        for(std::vector<Node*>::iterator cit = node->children.begin();
            cit != node->children.end(); ++cit)
        {
            GetConcernedGroups(robot, *cit, ids, roms);
        }
    }
}

T_Samples planner::sampling::GenerateSamples(const Robot &robot, const Node* root, int nbSamples)
{
    if(! ::generatorInit)
    {
        ::generatorInit = true;
        srand((unsigned int)(time(0))); //Init Random generation
    }
    Robot clone(robot);
    Node* workingNode = planner::GetChild(&clone, root->id);
    std::vector<int> ids;
    std::vector<NodeRom*> roms;
    GetConcernedGroups(clone, workingNode, ids, roms);
    T_Samples res;
    for(int i=0; i<nbSamples; ++i)
    {
        Sample* sample = GenerateSample(roms, workingNode);
        if(sample)
        {
            res.push_back(sample);
        }
        else
        {
            std::cout << "Can not generate valid configurations for rom of node " << root->tag << "; too restrictive ?"
                << std::endl;
        }
    }
    return res;
}
