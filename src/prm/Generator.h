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

namespace planner
{
/// \class Generator
/// \brief TODO
class Generator
{
public:
    ///\brief Constructor
    Generator(Object::T_Object& objects, const Object& model);

    ///\brief Destructor
    ~Generator();

public:
    Object* operator()();

public:
    const Object& model_;
    Collider collider_;
};
} //namespace planner
#endif //_CLASS_GENERATOR
