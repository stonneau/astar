/**
* \file Model.h
* \brief Helper struct for englobed object and englobing manipulation
* \author Steve T.
* \version 0.1
* \date 08/25/2014
*
*/
#ifndef _STRUCT_MODEL
#define _STRUCT_MODEL

#include "collision/Object.h"

namespace planner
{

struct Model
{
    Model(const Model& model)
	{
		englobed = new Object(*model.englobed);
		englobing = new Object(*model.englobing);
	}
    Model(){}
    ~Model()
    {
		if(englobed) delete englobed;
		if(englobing)delete englobing;
    }

    void SetOrientation(const Eigen::Matrix3d& orientation)
    {
        englobed->SetOrientation(orientation);
        englobing->SetOrientation(orientation);
    }

    void SetPosition(const Eigen::Vector3d& position)
    {
        englobed->SetPosition(position);
        englobing->SetPosition(position);
    }

    const Eigen::Matrix3d& GetOrientation() const
    {
        return englobed->GetOrientation();
    }
    const Eigen::Vector3d& GetPosition() const
    {
        return englobed->GetPosition();
    }

    Object* englobed;
    Object* englobing;
};

} //namespace planner
#endif //_STRUCT_MODEL
