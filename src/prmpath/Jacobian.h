
#ifndef _CLASS_JACOBIAN
#define _CLASS_JACOBIAN

#include <Eigen/Dense>

namespace planner
{
class Node;
}

class Jacobian {

public:
     Jacobian(planner::Node *root);
	~Jacobian();

private:
    //Jacobian& Jacobian::operator =(const Jacobian&);
	Jacobian(const Jacobian&);

public:
    void  SetJacobian(const Eigen::MatrixXd& jacobian);
    void  ComputeAll(planner::Node *root); // recomputes jacobian
	void  ComputeAll(); // recomputes everything but the jacobian
    void  ComputeJacobian(planner::Node *root);
    const Eigen::MatrixXd& GetNullspace();
    const Eigen::MatrixXd& GetJacobian();
    const Eigen::MatrixXd& GetJacobianInverse();
    const Eigen::Matrix3d& GetJacobianProduct();
    const Eigen::Matrix3d& GetJacobianProductInverse();
    void GetEllipsoidAxes(Eigen::Vector3d& /*u1*/, Eigen::Vector3d& /*u2*/, Eigen::Vector3d& /*u3*/);
    void GetEllipsoidAxes(Eigen::Vector3d& /*u1*/, Eigen::Vector3d& /*u2*/, Eigen::Vector3d& /*u3*/, double& /*sig1*/, double& /*sig2*/, double& /*sig3*/);

    void  GetNullspace(const Eigen::MatrixXd /*pseudoId*/, Eigen::MatrixXd& /*result*/);

private:
	void ComputeSVD();

private:
	bool computeInverse_;
	bool computeProduct_;
	bool computeProductInverse_;
	bool computeJacSVD_;
	bool computeNullSpace_;
	void Invalidate();

private:
    Eigen::Matrix3d jacobianProductInverse_;
    Eigen::Matrix3d jacobianProduct_;
    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd jacobianInverse_;
    Eigen::MatrixXd jacobianInverseNoDls_;
    Eigen::MatrixXd Identitymin_;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
    Eigen::JacobiSVD<Eigen::MatrixXd> svdProduct_;
};
#endif //_CLASS_JACOBIAN
