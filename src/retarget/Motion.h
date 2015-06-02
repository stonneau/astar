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

    bool equals(const Contact& other) const
    {
        return limbIndex_ == other.limbIndex_ && startFrame_ == other.startFrame_
                && endFrame_ == other.endFrame_;
    }
};

struct Frame
{
    Eigen::VectorXd configuration_;
    std::vector<Contact> contacts_;
};

typedef std::vector<std::pair<std::size_t, Eigen::Vector3d> > T_PointReplacement;

struct Motion
{
    Eigen::VectorXd Retarget(const std::size_t frameid, const Eigen::VectorXd& frameConfiguration, const T_PointReplacement& objectModifications) const;

    std::vector<Frame> frames_;
private:
    std::auto_ptr<PImpl> pImpl_;
    friend Motion* LoadMotion(const std::string& scenario);
};

Motion* LoadMotion(const std::string& scenario);
} //namespace efort
#endif //_STRUCT_MOTION
