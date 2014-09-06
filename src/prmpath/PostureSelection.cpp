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
            if(effector->InContact(*oit,0.01))
            {
                res.push_back(*sit);
            }
        }
    }
    return res;
}
