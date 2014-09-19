#include "ITOMPExporter.h"

#include "prmpath/Robot.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <vector>

using namespace std;
using namespace exporter;
using namespace Eigen;

namespace
{
    const double RAD_TO_DEGREES = 180 / M_PI; // Degrees = 180 * RAD / M_PI
    const Eigen::Vector3d zero(0,0,0);
}

namespace
{
    void WriteJointNameRec(FileHandler& f, planner::Node* node)
    {
        f << node->tag << f.nl();
        for(std::vector<planner::Node*>::iterator it = node->children.begin();
            it != node->children.end(); ++it)
        {
            WriteJointNameRec(f, *it);
        }
    }
}


ITOMPExporter::ITOMPExporter(const Eigen::Matrix3d& rotation, const Eigen::Vector3d &offset)
    : Exporter(rotation,offset,true)
{
    // NOTHING
}

ITOMPExporter::~ITOMPExporter()
{
    // NOTHING
}


void ITOMPExporter::PushStructure(planner::Robot* robot)
{
    f_ << "HIERARCHY" << f_.nl();
    WriteJointNameRec(f_, robot->node);
}
