/**
* \file SimplePRM.cpp
* \brief A PRM concrete implementation
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/

#include "SimplePRM.h"
#include "WorldABC.h"
#include "planner/PRM.h" // Template PRM definition

namespace planner
{
	struct Generator
	{
		Generator() {}
		~Generator() {}
		Configuration* operator()() 
		{
			return new Configuration(); // TODO Generate Random
		}
	};
	
	typedef PRM<Configuration, Generator, WorldABC, float, 10000> prm_t;

	struct PImpl
	{
		PImpl(const prm_t* prm) : prm_(prm){}
		~PImpl()
		{
			delete prm_;
		}

		Configuration* operator()() 
		{
			return new Configuration();
		}

		const prm_t* prm_;
	};

	Configuration* GenerateConfiguration()
	{
		return new Configuration();
	}

	float Distance(const Configuration* a, const Configuration* b)
	{
		// TODO include angles
		return sqrt((b->x_ - a->x_) * (b->x_ - a->x_) + (b->y_ - a->y_) * (b->y_ - a->y_) + (b->z_ - a->z_) * (b->z_ - a->z_));
	}
}

using namespace planner;

SimplePRM::SimplePRM(const WorldABC* world, float neighbourDistance, int size, int k)
	: world_(world)
{
	Generator gen;
	prm_t * prm = new prm_t(&gen, world_, Distance, neighbourDistance, size, k);
	pImpl_.reset(new PImpl(prm));
}

SimplePRM::~SimplePRM()
{
	// NOTHING
}

Configuration::T_Configuration SimplePRM::GetPath(const Configuration& from, const Configuration& to)
{
	return pImpl_->prm_->ComputePath(&from, &to, Distance, world_);
}
