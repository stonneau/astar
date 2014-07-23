/**
* \file WorldABC.cpp
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/

#include <iostream>
#include "LocalPlanner.h"
#include <Eigen/Dense>


namespace planner
{
    bool StraightLine(const Object* a, const Object* b, LocalPlanner& planner)
    {
        // v0 : draw line between a and b and check every 1 / 5 of the path if there is a collision
        Object tmp(*a);
        // compute line
        Eigen::Vector3d line = b->GetPosition() - a->GetPosition();
        // norm
        double step = 0.1;
        if(line.norm() == 0) return false;
        Eigen::Vector3d norm = line;
        norm.normalize(); line = line * step;
        Eigen::Vector3d offset;
        Eigen::Vector3d offrot;
        Eigen::Matrix3d offrotmat;
        // TODO rotate
        //  Vr = Va + t .(Vb - Va )

        // euler angle decomposition

        Eigen::Vector3d ea = a->GetOrientation().eulerAngles(2, 0, 2);
        Eigen::Vector3d eb = b->GetOrientation().eulerAngles(2, 0, 2);
        Eigen::Vector3d va = a->GetPosition();
        Eigen::Vector3d vb = b->GetPosition();

        double tInc = 0.1;
        /*Vector3f ea = mat.eulerAngles(2, 0, 2);

        "2" represents the z axis and "0" the x axis, etc. The returned angles are such that we have the following equality:
        * mat == AngleAxisf(ea[0], Vector3f::UnitZ())
        * * AngleAxisf(ea[1], Vector3f::UnitX())
        * * AngleAxisf(ea[2], Vector3f::UnitZ());*/

        /*Perform linear interpolation*/
        for(double t = 0; t <= 1; t = t + tInc)
        {
            offset = va + t * (vb - va);
            offrot = ea + t * (eb - ea);
            offrotmat = Eigen::AngleAxisd(offrot[0],  Eigen::Vector3d::UnitZ())
                     *  Eigen::AngleAxisd(offrot[1],  Eigen::Vector3d::UnitX())
                     *  Eigen::AngleAxisd(offrot[2],  Eigen::Vector3d::UnitZ());
            tmp.SetPosition(offset);
            tmp.SetOrientation(offrotmat);
            if(planner.IsColliding(&tmp))
            {
                return false;
            }
        }
        return true;
    }

    bool RotateAt(const Object* a, const Object* b, float when, LocalPlanner& planner)
    {
        // v0 : draw line between a and b and check every 1 / 5 of the path if there is a collision
        Object tmp(*a);
        // compute line
        Eigen::Vector3d position = a->GetPosition() +  when *(b->GetPosition() - a->GetPosition());
        tmp.SetPosition(position);
        if(StraightLine(a, &tmp, planner))
        {
            tmp.SetOrientation(b->GetOrientation());
            return StraightLine(&tmp, b, planner);
        }
        return false;
    }

    bool AstarLike(const Object* a, const Object* b, LocalPlanner& planner, int nbrs, int steps)
    {
        if(steps == 0) return false;
        // step distance given by distance to cover.
        double stepdistance = (b->GetPosition() - a->GetPosition()).norm() / steps;
        double bestDistance = std::numeric_limits<double>::max();
        Object best(*a);
        // random directionss
        for(int i=0; i < nbrs; ++i)
        {
            Eigen::Vector3d dir;
            for(int k=0; k<3; ++k)
            {
                dir(k) = (double) rand() / (RAND_MAX);
            }
            dir.normalize();
            dir = dir*stepdistance;
            Object tmp(*a);
            tmp.SetPosition(a->GetPosition() + dir);
            if(StraightLine(a,&tmp, planner))
            {
                if(StraightLine(&tmp, b, planner))
                {
                    return true;
                }
                else
                {
                    double currentDistance = (b->GetPosition() - tmp.GetPosition()).norm();
                    if(currentDistance < bestDistance)
                    {
                        bestDistance = currentDistance;
                        best.SetOrientation(tmp.GetOrientation());
                        best.SetPosition(tmp.GetPosition());
                    }
                }
            }
        }
        if(bestDistance < std::numeric_limits<double>::max())
        {
            return AstarLike(&best, b, planner, nbrs, steps-1);
        }
        else
        {
            return false;
        }
    }
}

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

bool LocalPlanner::operator ()(const Object* a, const Object* b, int stage)
{
    bool found = StraightLine(a, b, *this);
    if(!found && stage >0)
    {
        found = RotateAt(a, b, 0.5, *this) || RotateAt(a, b, 0, *this) || RotateAt(a, b, 1, *this)
        || AstarLike(a, b, *this, 15, 9);
    }
    return found;
}
