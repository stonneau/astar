
#include "VectorAlignmentConstraint.h"

#include <Eigen/Dense>

using namespace Eigen;
using namespace ik;
using namespace ftr;

VectorAlignmentConstraint::VectorAlignmentConstraint()
{
	// NOTHING
}

VectorAlignmentConstraint::~VectorAlignmentConstraint()
{
	// NOTHING
}

#include "rtql8Manipulability/Limb.h"

namespace
{
    Eigen::Vector3d GetArmVector(Limb* limb)
    {
        const Eigen::Vector3d zero(0,0,0);
        Eigen::Vector3d from = limb->end_->getParentNode()->evalWorldPos(zero);
        Eigen::Vector3d to = limb->end_->evalWorldPos(zero);
        to -= from;
        to.normalize();
        return to;
    }
}

double VectorAlignmentConstraint::Evaluate(ftr::Limb* limb, Eigen::VectorXd minDofs, Eigen::VectorXd maxDofs,  const int joint, Jacobian& jacobianMinus, Jacobian& jacobianPlus, float epsilon, const Vector3d& direction)
{
    Vector3d dirNorm = direction;
    dirNorm.normalize();
    // check dot product of angle with direction
    float valMin, valPlus;
    limb->skel_->setPose(minDofs, true, false);
    Eigen::Vector3d armAlign = GetArmVector(limb);
    valMin = -armAlign.dot(direction);
    limb->skel_->setPose(maxDofs, true, false);
    armAlign = GetArmVector(limb);
    valPlus = -armAlign.dot(direction);
    double res = double (valPlus - valMin) / (epsilon * 2) ;
    return res;
}


