#include "PostureSelection.h"
#include "tools/MatrixDefs.h"

using namespace planner;
using namespace sampling;

namespace
{
    Object* GetEffector(Node* limb)
    {
        if(limb->children.size() != 0)
        {
            Object* res = GetEffector(limb->children[0]);
            if(res) return res;
        }
        return limb->current;
    }

    const double epsilon = 0.01;
}

T_Samples planner::GetPosturesInContact(Robot& robot, Node* limb, const sampling::T_Samples& samples
                                         , Object::T_Object& obstacles)
{
    T_Samples res;
    /*Eigen::Matrix4d toWorldCoordinates = Eigen::Matrix4d::Identity();
    toWorldCoordinates.block<3,3>(0,0) = limb->parent->toWorldRotation;
    toWorldCoordinates.block<3,1>(0,3) = limb->parent->position;*/
    Object* effector = GetEffector(limb);
    for(T_Samples::const_iterator sit = samples.begin(); sit != samples.end(); ++sit)
    {
        LoadSample(*(*sit),limb);
        for(Object::T_Object::iterator oit = obstacles.begin(); oit != obstacles.end(); ++oit)
        {
            if(effector->InContact(*oit,epsilon) && !effector->IsColliding(obstacles))
            {
                res.push_back(*sit);
            }
        }
    }
    return res;
}

T_Samples planner::GetPosturesOnTarget(Robot& robot, Node* limb, const sampling::T_Samples &samples
                                         , Object::T_Object& obstacles, Eigen::Vector3d worldposition)
{
    T_Samples res;
    Object* effector = GetEffector(limb);
    for(T_Samples::const_iterator sit = samples.begin(); sit != samples.end(); ++sit)
    {
        LoadSample(*(*sit),limb);
        if((effector->GetPosition() - worldposition).norm()<epsilon && !effector->IsColliding(obstacles))
        {
            res.push_back(*sit);
        }
    }
    return res;
}

namespace
{

    std::vector<int> GetLimbsToContact(const std::vector<int>& inContactBefore, int nbLimbs)
    {
        std::vector<int> res;
        for(int i=0; i < nbLimbs; ++i)
        {
            if(std::find(inContactBefore.begin(), inContactBefore.end(), i)==inContactBefore.end())
            {
                res.push_back(i);
            }
        }
        return res;
    }

    planner::State* Interpolate(planner::CompleteScenario& scenario, const State& previous, const Object* next)
    {
        planner::State* res = new State;
        Robot * robot = new planner::Robot(*(previous.value));
        robot->SetConfiguration(next);
        res->value = robot;
        std::vector<Node*> limbs;
        for(std::vector<Node*>::iterator it = scenario.limbs.begin()
            ; it!=scenario.limbs.end(); ++it)
        {
            limbs.push_back(planner::GetChild(robot,(*it)->id));
        }
        //try to get back to previous state for starters.
        for(std::vector<int>::const_iterator cit = previous.contactLimbs.begin();
            cit != previous.contactLimbs.end(); ++cit)
        {
            T_Samples samples = GetPosturesOnTarget(*robot, limbs[*cit], scenario.limbSamples[*cit],
                                                    scenario.scenario->objects_, planner::GetEffectors(limbs[*cit])[0]->parent->position);
            if(!samples.empty())
            {
                res->contactLimbs.push_back(*cit);
                planner::sampling::LoadSample(*(samples.front()),limbs[*cit]);
            }
        }
        // create contacts then
        std::vector<int> newContacts = GetLimbsToContact(previous.contactLimbs, limbs.size());
        for(std::vector<int>::const_iterator cit = newContacts.begin();
            cit != newContacts.end(); ++cit)
        {
            T_Samples samples = GetPosturesInContact(*robot, limbs[*cit], scenario.limbSamples[*cit],
                                                    scenario.scenario->objects_);
            if(!samples.empty())
            {
                res->contactLimbs.push_back(*cit);
                planner::sampling::LoadSample(*(samples.front()),limbs[*cit]);
            }
        }
        return res;
    }
}

planner::T_State planner::PostureSequence(planner::CompleteScenario& scenario)
{
    planner::T_State res;
    State* current = &scenario.initstate;
    res.push_back(current);
    for(Object::CT_Object::iterator it = scenario.path.begin(); it!=scenario.path.end(); ++it)
    {
        current = Interpolate(scenario, *current, *it);
        res.push_back(current);
    }
    return res;
}
