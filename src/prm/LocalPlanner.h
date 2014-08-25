/**
* \file LocalPlanner.h
* \brief representation of the world for collision and obstacles
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/
#ifndef _CLASS_LOCALPLANNER
#define _CLASS_LOCALPLANNER

#include "collision/Object.h"
#include "collision/Collider.h"
#include "Model.h"

namespace planner
{
/// \class LocalPlanner
/// \brief Representation of the world for collision and obstacles
class LocalPlanner : public Collider
{
public:
	///\brief Constructor
    LocalPlanner(Object::T_Object& /*objects*/, const Model& /*model*/);

    ///\brief Destructor
    ~LocalPlanner();

public:
    bool operator ()(const Object* /*a*/, const Object* /*b*/, int stage =0);
    std::vector<Eigen::Matrix4d> Interpolate(const Object* /*a*/, const Object* /*b*/, int nbSteps);
	
public:
	const Model& model_;
};
} //namespace planner
#endif //_CLASS_LOCALPLANNER
