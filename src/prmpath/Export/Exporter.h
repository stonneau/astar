#ifndef  _EXPORTER_H_
#define  _EXPORTER_H_


#include "prmpath/Export/FileHandler.h"

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace planner
{
class Node;
class Robot;
}

/**
*  \class Exporter
*  \brief Export A robot to a readable BVH file format
*/
namespace exporter
{
class  Exporter
{
public:
     Exporter(bool useRadians=false);
     Exporter(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& offset, bool useRadians=false);
    virtual ~Exporter();

public:
    void PushFrame(planner::Node* /*rootNode*/, bool tpose = false);
    bool Save(const std::string& /*filename*/);
    virtual void PushStructure(planner::Robot*/*skeleton*/)=0;

private:
    Exporter(const Exporter&);
    Exporter& operator=(const Exporter&);

protected:
    FileHandler f_;
    std::vector< std::string > frames_;

private:
    const bool useRadians_;
    const Eigen::Matrix3d rotation_;
    const Eigen::Vector3d offset_;
};
} //namespace exporter
#endif // _EXPORTER_H_