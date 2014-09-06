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

namespace planner
{

sampling::T_Samples GetPosturesInContact(Robot& robot, Node* limb, const sampling::T_Samples &samples
                                         , Object::T_Object& obstacles);
} // namespace planner
#endif //_POSTURESELECTION
