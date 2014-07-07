/**
* \file SimplePRM.h
* \brief A PRM concrete implementation
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/
#ifndef _CLASS_SIMPLEPRM
#define _CLASS_SIMPLEPRM

#include <vector>
#include <memory>

namespace planner
{

class WorldABC;
struct PImpl;

/// \struct Configuration
/// \brief Describes a configuration with 3 positions and 3 orientations.
/// positions are unbounded, orientations are limited to [0,1] radians.
struct Configuration
{
	typedef std::vector<const Configuration*> T_Configuration;

	///\brief Constructor
	Configuration()
		: x_(0), y_(0), z_(0), rotx_(0), roty_(0), rotz_(0){}

	Configuration(float x, float y, float z, float rotx, float roty, float rotz)
		: x_(x), y_(y), z_(z), rotx_(rotx), roty_(roty), rotz_(rotz){}
	
	///\brief Destructor
	~Configuration() {}

	float x_, y_, z_;
	float rotx_, roty_, rotz_;
};

/// \class SimplePRM
/// \brief Concrete implementation of a probabilistic roadmap (PRM)
/// Candidate Nodes are generated in a purely random manner
/// Scaled euclidian distance is used as a metric
/// Connectivity is checked using a simple straight line planner 
class SimplePRM
{
public:
	///\brief Constructor
	///\param collider collision detection object that will detect collisions
	///\param neighbourDistance maximum distance for which a node can be a neighbour of another
	///\param size number of nodes to be generated
	///\param k maximum number of neighbours for a given Node. Default value is 10
	SimplePRM(const WorldABC* /*world*/, float /*neighbourDistance*/, int size = 1000, int k = 10);

	///\brief Destructor
	 ~SimplePRM();
	 
	 Configuration::T_Configuration GetPath(const Configuration& /*from*/, const Configuration& /*to*/, float /*neighbourDistance*/);

private:
	const WorldABC* world_;
	std::auto_ptr<PImpl> pImpl_;
};
} //namespace planner
#endif //_CLASS_SIMPLEPRM