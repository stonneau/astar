
#ifndef _CLASS_VECALIGNCONSTRAINT
#define _CLASS_VECALIGNCONSTRAINT

#include "PartialDerivativeConstraint.h"

namespace ik{
class Jacobian;

class VectorAlignmentConstraint : public PartialDerivativeConstraint
{

public:
	 VectorAlignmentConstraint();
	~VectorAlignmentConstraint();

public:
    virtual double Evaluate(ftr::Limb* /*limb*/, Eigen::VectorXd /*minDofs*/, Eigen::VectorXd /*maxDofs*/,  const int joint, Jacobian& /*jacobianMinus*/, Jacobian& /*jacobianPlus*/, float /*epsilon*/, const Eigen::Vector3d& /*direction*/);

};
}
// namespace ik
#endif //_CLASS_FMCONSTRAINT
