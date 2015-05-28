/**
* \file Motion.h
* \brief Structure describing the complete motion of a robot
* \author Steve T.
* \version 0.1
* \date 28/04/2015
*
*/
#ifndef _STRUCT_MOTION
#define _STRUCT_MOTION


#include "collision/Object.h" // this needs to move out

#include <string>

#include <Eigen/Dense>
#include <memory>

namespace efort
{
struct PImpl;

struct Contact
{
    int limbIndex_;
    int startFrame_;
    int endFrame_;
    Eigen::Vector3d worldPosition_;
    Eigen::Vector3d surfaceNormal_;
    std::size_t objectId_;
    std::size_t triangleId_;
};

struct Frame
{
    Eigen::VectorXd configuration_;
    std::vector<Contact> contacts_;
};

struct Motion
{
    Frame Retarget(const std::size_t /*frameid*/) const; //tmp: waht for objs ?
    Frame Retarget(const std::size_t /*frameid*/, const std::vector<Eigen::Vector3d>& /*target*/, planner::Object::T_Object& objects) const; //tmp: waht for objs ?
    std::vector<Frame> Retarget(const std::vector<Eigen::VectorXd>& frameConfigurations, planner::Object::T_Object& objects) const;

    std::vector<Frame> frames_;
private:
    std::auto_ptr<PImpl> pImpl_;
    friend Motion* LoadMotion(const std::string& scenario);
};

Motion* LoadMotion(const std::string& scenario);
} //namespace efort
#endif //_STRUCT_MOTION
