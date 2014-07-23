/**
* \file ParserObj.h
* \brief Utility functions for loading 3D objects
from an obj file.
* \author Steve T.
* \version 0.1
* \date 16/07/2014
*
*/
#ifndef _CLASS_ParserObj
#define _CLASS_ParserObj

#include <string>
#include <vector>

#include "Object.h"

namespace planner
{
///  \brief Loads a set of Object from an obj files.
///  TODO: Only accepts triangles and quads objects.
///  \param filename path to the obj file to be loaded.
///  \param asOneObject if true obj file is considered one single object
///  \param return : a list of Object from the obj file.
Object::T_Object ParseObj(const std::string&  /*filename*/, const bool asOneObject = false);


///  \brief Loads a set of Object from an obj files and concatenates
///  them to a list given in parameters.
///  TODO: Only accepts triangles and quads objects.
///  \param filename path to the obj file to be loaded.
///  \param objects a T_Object at the end of which the new Object will be concatanated.
///  \param asOneObject if true obj file is considered one single object
///  \param return : a list of Object from the obj file.
void ParseObj(const std::string& /*filename*/, Object::T_Object& /*objects*/, const bool asOneObject = false);
}//namespace planner;
#endif //_CLASS_ParserObj
