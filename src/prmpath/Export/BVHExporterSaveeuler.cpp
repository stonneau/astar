#include "BVHExporter.h"

#include "kinematics/BodyNode.h"
#include "kinematics/Joint.h"
#include "kinematics/Shape.h"
#include "kinematics/Dof.h"
#include "kinematics/Transformation.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "utils/UtilsRotation.h"

#include <vector>

using namespace std;
using namespace rtql8;
using namespace rtql8::kinematics;
using namespace rtql8::utils;
using namespace bvh;
using namespace Eigen;

namespace
{
    const double PI = acos(-1.0);
    const double RAD_TO_DEGREES = 180 / PI; // Degrees = 180 * RAD / pi
    const Eigen::Vector3d zero(0,0,0);
}


namespace
{
    void WriteOffsetLine(BVHFileHandler& f, const Eigen::Vector3d& offset)
    {
        f << "OFFSET\t" << offset(0) * 100 << "\t" << offset(1) * 100 << "\t" << offset(2)* 100;
    }

    // true if endEffector
    void WriteJointOffset(BVHFileHandler& f, BodyNode* node)
    {
        //Eigen::Vector3d offset = node->evalWorldPos(zero) - node->getParentNode()->evalWorldPos(zero);
        Eigen::Vector3d offset = node->getLocalTransform().block<3,1>(0,3);
        f << f.nl() << "JOINT " << node->getName() << f.nl() << "{";
        f.AddTab();
            WriteOffsetLine(f, offset);
            f << f.nl() << "CHANNELS 3   Xrotation Yrotation Zrotation";
            //PushRotationInOrder(node, f);
            if(node->getNumChildJoints() == 0)
            {
                f << f.nl() << "End Site" << f.nl() << "{";
                f.AddTab();
                    // last offset corresponds to half length of final cube according to ParserBVH.hpp
                    double delta = (node->getCollisionShape()->getDim() / 2).norm();
                    offset.normalize(); offset *= delta;
                    WriteOffsetLine(f, offset);
                f.RemoveTab();
                f << "}";
            }
            else
            {
                for(int i=0; i< node->getNumChildJoints();++i)
                {
                    WriteJointOffset(f, node->getChildNode(i));
                }
            }
        f.RemoveTab();
        f << "}";
    }


    // Blender apparently does not like it if I don't give all the rotations.
    // Therefore Adding zeros to unexisting dofs
    void WriteDof(BodyNode* node, stringstream& ss)
    {
        node->updateTransform();
        Matrix4d Wl = node->getLocalTransform();
        Matrix3d Rl = Matrix3d::Zero();
        Vector3d tl = Vector3d::Zero();
        for(int ii=0; ii<3; ii++){
            for(int jj=0; jj<3; jj++) Rl(ii, jj) = Wl(ii, jj);
            tl[ii]=Wl(ii, 3);
        }
        Vector3d ei = utils::rotation::matrixToEuler(Rl, rotation::XYZ);

        for(int i = 0 ; i<3; ++i)
        {
            ss << "\t" << ei[i] * RAD_TO_DEGREES;
        }
    }
}


BVHExporter::BVHExporter(rtql8::kinematics::Skeleton * skeleton)
    : f_()
{
    PushStructure(skeleton);
}

BVHExporter::~BVHExporter()
{
    // NOTHING
}


void BVHExporter::PushStructure(rtql8::kinematics::Skeleton *skeleton)
{
    // just making sure that everything is set to 0
    Eigen::VectorXd pose = skeleton->getPose();
    for(int i = 0; i< skeleton->getNumDofs(); ++i)
    {
        pose[i] = 0;//(double)i / (double) body_skel->getNumDofs();
    }
    skeleton->setPose(pose);
    BodyNode* root (skeleton->getRoot());
    Eigen::Vector3d offset(root->evalWorldPos(Eigen::Vector3d::Zero()));
    f_ << "HIERARCHY" << f_.nl() << "ROOT " << root->getName() << f_.nl() << "{" ;
    f_.AddTab();
        WriteOffsetLine(f_, offset);
        f_ << f_.nl() << "CHANNELS 6 Xposition Yposition Zposition Xrotation Yrotation Zrotation";
        //PushRotationInOrder(root, f_);
        for(int i=0; i< root->getNumChildJoints();++i)
        {
            WriteJointOffset(f_, root->getChildNode(i));
        }
    f_.RemoveTab();
    f_ << "}";
}

void BVHExporter::PushFrame(rtql8::kinematics::Skeleton * skeleton)
{
    Eigen::VectorXd pose(skeleton->getPose());
    std::stringstream frame;

     // Write translations...
    frame << pose[0] << "\t" << pose[1] << "\t" << pose[2];
    for(int i =0; i<skeleton->getNumNodes(); ++i )
    {
        WriteDof(skeleton->getNode(i), frame);
    }
    frames_.push_back(frame.str());
}


bool BVHExporter::Save(const std::string& filename)
{
    // saving frames
    f_ << f_.nl() << "MOTION\nFrames:\t" << (double)(frames_.size()) << "\nFrame Time: 1\n";
    for(std::vector<string>::const_iterator it = frames_.begin(); it!=frames_.end(); ++it)
    {
        f_ << (*it) << f_.nl();
    }
    return f_.Save(filename);
}
