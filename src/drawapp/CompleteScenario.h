/**
* \file CompleteScenario.h
* \brief Helper struct that contains a complete scenario
* for an manipulation task
* \author Steve T.
* \version 0.1
* \date 09/05/2014
*
*/
#ifndef _STRUCT_COMPLETESCENARIO
#define _STRUCT_COMPLETESCENARIO

#include <vector>
#include <string>

#include "prm/Scenario.h"
#include "prmpath/Robot.h"
#include "prmpath/JointConstraint.h"
#include "prmpath/sampling/Sample.h"
namespace planner
{
struct CompleteScenario
{
     CompleteScenario();
    ~CompleteScenario();

    bool SavePath(const std::string& outfilename);

    Scenario* scenario; //prm + objs
    Robot* robot;
    std::vector<planner::Node*> limbs;
    std::vector<sampling::T_Samples> limbSamples;
    Object* from;
    Object* to;
    Object::CT_Object path;
    std::vector<planner::Robot*> completePath;
};

/*
File description:
PRMSCENARIO file="path/to/scenario"
ROBOT_SKELETON file="path/to/urdf"
ROBOT_CONSTRAINTS file="path/to/joint_constraints"
PATH_FROM matrix="a00 a01 a02 a03 ... a30 a31 a32 a33"
PATH_TO matrix="a00 a01 a02 a03 ... a30 a31 a32 a33"
LIMB joint_name="joint_name"
...
LIMB joint_name="joint_name"
NBSAMPLES N=""
*/
CompleteScenario* CompleteScenarioFromFile(const std::string& file);

} //namespace planner
#endif //_STRUCT_COMPLETESCENARIO
