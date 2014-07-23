/**
* \file PRMTest.cpp
* \brief A PRM concrete implementation
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/

#include "PRMTest.h"

namespace planner
{
	
	typedef PRM<Configuration, Generator, World, float, 10000> prm_t;

	struct PImpl
	{
		PImpl(const prm_t* prm) : prm_(prm){}
		~PImpl()
		{
			delete prm_;
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

PRMTest::PRMTest(World* world, Generator* generator, float neighbourDistance, int size, int k)
	: world_(world)
	, neighbourDistance_(neighbourDistance)
{
	prm_t * prm = new prm_t(generator, world_, Distance, neighbourDistance_, size, k);
	pImpl_.reset(new PImpl(prm));
}

PRMTest::~PRMTest()
{
	// NOTHING
}

Configuration::T_Configuration PRMTest::GetPath(const Configuration& from, const Configuration& to)
{
	return pImpl_->prm_->ComputePath(&from, &to, Distance, world_, neighbourDistance_);
}
