/**
* \file Generator.h
* \brief Random Configuration Generator
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/
#ifndef _CLASS_GENERATOR
#define _CLASS_GENERATOR

#include "collision/Object.h"
#include "collision/Collider.h"
#include "prm/SimplePRM.h"

namespace planner
{
/// \class Generator
/// \brief TODO
class Generator
{
public:
    typedef std::pair<Eigen::Vector3d, const Tri*> T_TriangleNormal;
    typedef std::pair<Object*, T_TriangleNormal> SampleTriangle;
public:
    ///\brief Constructor
    Generator(Object::T_Object& objects, Object::T_Object& collisionObjects,  const Model& model, int NbEnblobingConstraints=2);

    ///\brief Destructor
    ~Generator();

public:
    Model* operator()();

public:
    const Model& model_;
    Object::T_Object& objects_;
    Object::T_Object& contactObjects_;
    Collider collider_;

private:
    void InitWeightedTriangles();
    SampleTriangle RandomPointIntriangle();
    const SampleTriangle& WeightedTriangles();

private:
    std::vector<float> weights_;
    std::vector<SampleTriangle > triangles_;
    const int NbEnblobingConstraints_;
};
} //namespace planner
#endif //_CLASS_GENERATOR
