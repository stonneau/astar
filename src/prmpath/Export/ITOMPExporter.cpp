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



    // Blender apparently does not like it if I don't give all the rotations.
    // Therefore Adding zeros to unexisting dofs
    void WriteDofRecNoFantomJoint(planner::Node* node, stringstream& ss, bool tpose)
    {
        if(tpose)
        {
            ss << "\t" << 0;
        }
        else
        {
            /*if(node->axis == Eigen::Vector3d::UnitX() && node->parent && node->offset.norm() != 0)
            {
                ss << "\t" << node->parent->value;
            }
            else if(node->axis == Eigen::Vector3d::UnitX())
            {

            }
            else*/
            ss << "\t" << node->value;
        }
        for(std::vector<planner::Node*>::iterator it = node->children.begin();
            it != node->children.end(); ++it)
        {
            WriteDofRecNoFantomJoint(*it, ss, tpose);
        }
    }

    void WriteDofNoFantomJoint(planner::Node* node, stringstream& ss, const Eigen::Matrix3d& rot, bool tpose)
    {
        // retrieve first nodes and apply rotations
        Eigen::Matrix3d rotation = Eigen::AngleAxisd(node->value, node->axis).matrix();
        while(node->children.size() == 1 && node->children[0]->offset == Eigen::Vector3d::Zero())
        {
            node = node->children.front();
            rotation *= Eigen::AngleAxisd(node->value, node->axis).matrix();
        }
        Matrix<double,3,1> res = (rot * rotation).eulerAngles(2, 1, 0);
        ss << res[0] << "\t" << -res[2] << "\t" << res[1];
        for(std::vector<planner::Node*>::iterator it = node->children.begin();
            it != node->children.end(); ++it)
        {
            WriteDofRecNoFantomJoint(*it, ss, tpose);
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


void ITOMPExporter::PushFrame(planner::Robot* robot, bool tpose)
{
    std::stringstream frame;
     // Write translations...
    Eigen::Vector3d res = rotation_ * robot->node->position + offset_;
    Eigen::Matrix3d rot =AngleAxisd(-0.5*M_PI, Vector3d::UnitY()).matrix();
    frame << res[0] << "\t" << res[1] << "\t" << res[2]<< "\t";
    WriteDofNoFantomJoint(robot->node->children.front(), frame, rotation_, tpose);
    //WriteDofRecNoFantomJoint(robot->node->children.front(), frame, tpose);
    //WriteDofNoFantomJoint(robot->node->children.front(), frame, rot, tpose);
    //WriteDofRecNoFantomJoint(robot->node->children.front(), frame, tpose);
    frames_.push_back(frame.str());
}

void ITOMPExporter::PushStructure(planner::Robot* robot)
{
    f_ << "HIERARCHY" << f_.nl();
    // put translation
    f_ << "base_prismatic_joint_x" << f_.nl();
    f_ << "base_prismatic_joint_y" << f_.nl();
    f_ << "base_prismatic_joint_z" << f_.nl();

    WriteJointNameRec(f_, robot->node->children.front());
}
