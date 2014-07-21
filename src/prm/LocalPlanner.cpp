/**
* \file WorldABC.cpp
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/

#include "LocalPlanner.h"
#include <Eigen/Dense>

using namespace planner;

LocalPlanner::LocalPlanner(Object::T_Object& objects)
    : Collider(objects)
{
	// NOTHING
}


LocalPlanner::~LocalPlanner()
{
	// NOTHING
}

bool LocalPlanner::operator ()(const Object* a, const Object* b)
{
    // v0 : draw line between a and b and check every 1 / 5 of the path if there is a collision
    Object tmp(*a);
    // compute line
    Eigen::Vector3d line = b->GetPosition() - a->GetPosition();
    // norm
    double step = line.norm() / 5;
    if(step == 0) return false;
    line.normalize(); line = line * step;
    Eigen::Vector3d offset(a->GetPosition());
    for(int i=0; i<5; ++i)
    {
        offset += line;
        tmp.SetPosition(offset);
        if(IsColliding(&tmp))
        {
            return false;
        }
    }
    return true;
}
