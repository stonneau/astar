/**
* \file PRMTest.h
* \brief A PRM concrete implementation
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/
#ifndef _CLASS_PRMTest
#define _CLASS_PRMTest

#include <vector>
#include <memory>

namespace planner
{


static int id = -1;


/// \struct Configuration
/// \brief Describes a configuration with 3 positions and 3 orientations.
/// positions are unbounded, orientations are limited to [0,1] radians.
struct Configuration
{
	typedef std::vector<const Configuration*> T_Configuration;

	///\brief Constructor
	Configuration()
		: x_(0), y_(0), z_(0), rotx_(0), roty_(0), rotz_(0){ id_ = ++id;}
	
	Configuration(float x, float y, float z)
		: x_(x), y_(y), z_(z), rotx_(0), roty_(0), rotz_(0){id_ = ++id;}

	Configuration(float x, float y, float z, float rotx, float roty, float rotz)
		: x_(x), y_(y), z_(z), rotx_(rotx), roty_(roty), rotz_(rotz){id_ = ++id;}

	Configuration(const Configuration* c)
		: x_(c->x_), y_(c->y_), z_(c->z_), rotx_(c->rotx_), roty_(c->roty_), rotz_(c->rotz_), id_(c->id_)
	{
		bool tg = false;
	}
	
	///\brief Destructor
	~Configuration() {
	}

	float x_, y_, z_;
	float rotx_, roty_, rotz_;
	int id_;
};

struct PImpl;
struct World
{
	 World(){}
	~World(){}

	void AddCollisionBetweenPath(int id1, int id2)
	{
		collisions.push_back(std::make_pair(id1, id2));
	}

	bool operator() (const Configuration* a, const Configuration* b) const
	{
		for(std::vector<std::pair<int, int> >::const_iterator it = collisions.begin(); 
			it != collisions.end();
			++it)
		{
			if( (it->first == a->id_ && it->second == b->id_)
				|| (it->first == b->id_ && it->second == a->id_))
			{
				return false;
			}
		}
		return true;
	}

	std::vector<std::pair<int, int> > collisions;
};


struct Generator
{
	Generator(std::vector<Configuration*> generatedConfigurations)
		: generatedConfigurations_(generatedConfigurations)
		, current(generatedConfigurations_.begin())
	{
		// NOTHING
	}

	~Generator() 
	{
		// NOTHING
	}

	Configuration* operator()() 
	{
		if(current != generatedConfigurations_.end())
		{
			Configuration* res = new Configuration(*current);
			++current;
			return res;
		}
		return new Configuration();
	}

	std::vector<Configuration*> generatedConfigurations_;
	std::vector<Configuration*>::iterator current;
};


/// \class PRMTest
/// \brief Concrete implementation of a probabilistic roadmap (PRM)
/// Candidate Nodes are generated in a purely random manner
/// Scaled euclidian distance is used as a metric
/// Connectivity is checked using a simple straight line planner 
class PRMTest
{
public:
	///\brief Constructor
	///\param collider collision detection object that will detect collisions
	///\param neighbourDistance maximum distance for which a node can be a neighbour of another
	///\param size number of nodes to be generated
	///\param k maximum number of neighbours for a given Node. Default value is 10
	PRMTest(const World* /*world*/, Generator* /*generator*/, float /*neighbourDistance*/, int size = 1000, int k = 10);

	///\brief Destructor
	 ~PRMTest();
	 
	 Configuration::T_Configuration GetPath(const Configuration& from, const Configuration& to);

private:
	const World* world_;
	std::auto_ptr<PImpl> pImpl_;
};
} //namespace planner
#endif //_CLASS_PRMTest