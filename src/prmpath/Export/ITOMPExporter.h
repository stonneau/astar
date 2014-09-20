#ifndef  _ITOMPEXPORTER_H_
#define  _ITOMPEXPORTER_H_


#include "prmpath/Export/Exporter.h"

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace planner
{
class Node;
class Robot;
}

/**
*  \class ITOMPExporter
*  \brief Export A robot to a readable BVH file format
*/
namespace exporter
{
class  ITOMPExporter : public Exporter
{
public:
     ITOMPExporter(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &offset);
    ~ITOMPExporter();

public:
    virtual void PushStructure(planner::Robot*/*skeleton*/);
    virtual void PushFrame(planner::Robot * /*robot*/, bool tpose = false);

private:
    ITOMPExporter(const ITOMPExporter&);
    ITOMPExporter& operator=(const ITOMPExporter&);
};
} //namespace exporter
#endif // _ITOMPEXPORTER_H_
