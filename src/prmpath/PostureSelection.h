/**
* \file PostureSelection.h
* \brief Helper functions to retrieve contact configurations
* articulation.
* \author Steve T.
* \version 0.1
* \date 09/06/2014
*
*/
#ifndef _POSTURESELECTION
#define _POSTURESELECTION

#include "prmpath/Robot.h"
#include "prmpath/sampling/Sample.h"
#include "prmpath/CompleteScenario.h"

namespace planner
{

sampling::T_Samples GetPosturesInContact(Robot& robot, Node* limb, const sampling::T_Samples &samples
                                         , Object::T_Object& obstacles);

sampling::T_Samples GetPosturesOnTarget(Robot& robot, Node* limb, const sampling::T_Samples &samples
                                         , Object::T_Object& obstacles, Eigen::Vector3d worldposition);

planner::T_State PostureSequence(planner::CompleteScenario& scenario);
} // namespace planner
#endif //_POSTURESELECTION
