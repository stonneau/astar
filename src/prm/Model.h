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

struct Model;
typedef std::vector<Model*> T_Model;
typedef T_Model::iterator IT_Model;
typedef std::vector<const Model*> CT_Model;

struct Model
{
    Model(const Model& model)
	{
        englobed = new Object(*model.englobed);
        for(Object::T_Object::const_iterator it = model.englobing.begin();
            it != model.englobing.end(); ++it)
        {
            englobing.push_back(new Object(*(*it)));
        }
	}
    Model(){}
    ~Model()
    {
		if(englobed) delete englobed;
        for(Object::T_Object::iterator it = englobing.begin();
            it != englobing.end(); ++it)
        {
            delete(*it);
        }
    }

    void SetOrientation(const Eigen::Matrix3d& orientation)
    {
        englobed->SetOrientation(orientation);
        for(Object::T_Object::iterator it = englobing.begin();
            it != englobing.end(); ++it)
        {
            (*it)->SetOrientation(orientation);
        }
    }

    void SetPosition(const Eigen::Vector3d& position)
    {
        englobed->SetPosition(position);
        for(Object::T_Object::iterator it = englobing.begin();
            it != englobing.end(); ++it)
        {
            (*it)->SetPosition(position);
        }
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
    Object::T_Object englobing;
};

} //namespace planner
#endif //_STRUCT_MODEL
