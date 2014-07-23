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

#include "collision/Object.h"

#include <string>
#include <array>

namespace planner
{

struct Model
{
    Model(){}
    ~Model()
    {
        delete englobed;
        delete englobing;
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

struct PImpl;

/// \class SimplePRM
/// \brief Concrete implementation of a probabilistic roadmap (PRM)
/// Candidate Nodes are generated in a purely random manner
/// Scaled euclidian distance is used as a metric
/// Connectivity is checked using a simple straight line planner 
class SimplePRM
{
public:
	///\brief Constructor
    ///\param model Model used for the sample generation
    ///\param objects Description of the world
	///\param neighbourDistance maximum distance for which a node can be a neighbour of another
	///\param size number of nodes to be generated
	///\param k maximum number of neighbours for a given Node. Default value is 10
    SimplePRM(const Model& model, Object::T_Object &objects, float neighbourDistance, int size = 1000, int k = 10);

	///\brief Destructor
	 ~SimplePRM();
	 
     Object::CT_Object GetPath(const Object& /*from*/, const Object& /*to*/, float /*neighbourDistance*/);

public:
     const Model& model_;

public:
     const Object::T_Object& GetPRMNodes() const;
     const std::vector< int >& GetConnections(int node) const;

private:
    Object::T_Object objects_;
	std::auto_ptr<PImpl> pImpl_;
};
} //namespace planner
#endif //_CLASS_SIMPLEPRM
