/**
* \file WorldABC.h
* \brief Abstract representation of the world for collision and obstacles
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/
#ifndef _CLASS_WORLDABC
#define _CLASS_WORLDABC

#include "SimplePRM.h"

namespace planner
{
struct Configuration;
/// \class WorldABC
/// \brief Abstract representation of the world for collision and obstacles
class WorldABC
{
public:
	///\brief Constructor
	WorldABC();

	///\brief Destructor
	~WorldABC();

public:
	virtual bool IsColliding(const Configuration* /*configuration*/) const = 0;
	virtual bool operator ()(const Configuration* /*a*/, const Configuration* /*b*/) const = 0;
};
} //namespace planner
#endif //_CLASS_WORLDABC