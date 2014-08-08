
#include "equilibrium/Test.h"

#include <string>
#include <iostream>
#include <cmath>

using namespace std;
#include "Test.h"
#include "libcdd/setoper.h"
#include "libcdd/cdd.h"

#include <vector>

namespace equilib
{

const Vector X(1,0,0);
const Vector Y(0,1,0);
const Vector Z(0,0,1);

Eigen::Matrix3d skew(const Eigen::Vector3d& x)
{
    Eigen::Matrix3d res = Eigen::Matrix3d::Zero();
    res(0,1) = - x(2); res(0,2) = - x(1);
    res(1,0) =   x(2); res(1,2) = - x(0);
    res(2,0) = - x(1); res(2,1) = - x(0);
    return res;
}

dd_MatrixPtr FromEigen (const Eigen::MatrixXd& b, const Eigen::MatrixXd& input, dd_ErrorType *Error)
{
    // b - Ax > 0
    dd_rowrange m_input = (dd_rowrange)(input.rows());
    dd_colrange d_input = (dd_colrange)(input.cols() + 1);
    dd_MatrixPtr M=NULL;
    dd_rowrange i;
    dd_colrange j;

    dd_RepresentationType rep=dd_Inequality;
    mytype value;
    dd_NumberType NT = dd_Real;

    dd_init(value);

    M=dd_CreateMatrix(m_input, d_input);
    M->representation=rep;
    M->numbtype=NT;

    for (i = 1; i <= m_input; i++)
    {
        dd_set_d(value, b(i-1,0));
        dd_set(M->matrix[i-1][0],value);
        for (j = 2; j <= d_input; j++)
        {
        #if defined GMPRATIONAL
            *Error=dd_NoRealNumberSupport;
            goto _L99;
        #else
            dd_set_d(value, -input(i-1,j-2));
        #endif
        dd_set(M->matrix[i-1][j - 1],value);
        }  /*of j*/
    }  /*of i*/
  dd_clear(value);
  return M;
}

dd_MatrixPtr FromEigen (const Eigen::MatrixXd& input, dd_ErrorType *Error)
{
    dd_MatrixPtr M=NULL;
    dd_rowrange i;
    dd_colrange j;
    dd_rowrange m_input = (dd_rowrange)(input.rows());
    dd_colrange d_input = (dd_colrange)(input.cols() + 1);
    dd_RepresentationType rep=dd_Generator;
    mytype value;
    dd_NumberType NT = dd_Real;;
    dd_init(value);

    M=dd_CreateMatrix(m_input, d_input);
    M->representation=rep;
    M->numbtype=NT;

    for (i = 1; i <= m_input; i++)
    {
        dd_set_d(value, 1);
        dd_set(M->matrix[i-1][0],value);
        for (j = 2; j <= d_input; j++)
        {
          dd_set_d(value, input(i-1,j-2));
          dd_set(M->matrix[i-1][j - 1],value);
        }  /*of j*/
    }  /*of i*/
  dd_clear(value);
  return M;
}

// human motions analysis and simulation based on a
// general criterion of stability
void CheckEquilibrium(const Eigen::MatrixXd& contactTransforms, const Eigen::MatrixXd& graspTransforms, const Eigen::VectorXd& maxGraspingForces,  float friction, double flimit)
{
    //Compute Ac
    // Ac = [Af, Av]'
        // Compute Af matches contact forces
        // Af = diag(B_1,B_N)
    int nbContacts = contactTransforms.rows() / 4;
    int nbGrasps = graspTransforms.rows() / 4;
    Eigen::MatrixXd C(3, (nbContacts + nbGrasps) * 3);
    Eigen::MatrixXd A(3, (nbContacts + nbGrasps) * 3);
    // Init A with identities
    Eigen::Matrix3d id = Eigen::Matrix3d::Identity();
    for(int k = 0; k < nbContacts + nbGrasps; ++k)
    {
        A.block<3,3>(0, 3*k) = id;
    }
    Eigen::MatrixXd Af = Eigen::MatrixXd::Zero(nbContacts * 4 +1, nbContacts * 3);
    int acIndex = Af.rows()-1;
    Eigen::VectorXd bc = Eigen::VectorXd::Zero(nbContacts * 4 +1);
    bc(acIndex) = flimit;

    for(int i=0; i< nbContacts; ++i)
    {
        C.block<3,3>(0,3*i) = skew(contactTransforms.block<3,1>(i*4, 2));
        const Rotation& Ri = contactTransforms.block<3,3>(i*4, 0);
        const Vector vi = Ri*Z;
        const Vector si = Ri*X;
        const Vector ti = Ri*Y;
        Eigen::Matrix4d beta_i;
        beta_i.block<1,3>(0,0) = -(friction * vi + si).transpose();
        beta_i.block<1,3>(1,0) = -(friction * vi - si).transpose();
        beta_i.block<1,3>(2,0) = -(friction * vi + ti).transpose();
        beta_i.block<1,3>(3,0) = -(friction * vi - ti).transpose();
        Af.block<4,3>(4*i,3*i) = beta_i.block<4,3>(0,0);
        Af(acIndex, 3*i + 2) = 1.;
    }
    //Compute Ag
    Eigen::MatrixXd Ag = Eigen::MatrixXd::Zero(nbGrasps * 6, nbGrasps * 3);
    Eigen::VectorXd bg = Eigen::VectorXd::Zero(nbGrasps * 6);
    for(int i=0; i< nbGrasps; ++i)
    {
        C.block<3,3>(0,3*(i+nbContacts)) = skew(graspTransforms.block<3,1>(i*4, 2));
        Ag(6*i  ,3*i)   = 1;
        Ag(6*i+1,3*i)   =-1;
        Ag(6*i+2,3*i+1) = 1;
        Ag(6*i+3,3*i+1) =-1;
        Ag(6*i+4,3*i+2) = 1;
        Ag(6*i+5,3*i+2) =-1;
        bg(6*i)   = maxGraspingForces(3*i);
        bg(6*i+1) = maxGraspingForces(3*i);
        bg(6*i+2) = maxGraspingForces(3*i+1);
        bg(6*i+3) = maxGraspingForces(3*i+1);
        bg(6*i+4) = maxGraspingForces(3*i+2);
        bg(6*i+5) = maxGraspingForces(3*i+2);
    }
    Eigen::MatrixXd Acg = Eigen::MatrixXd::Zero(Af.rows() + Ag.rows(), Af.cols() + Ag.cols());
    Eigen::MatrixXd bcg(bc.rows() + bg.rows(), 1) ;
    Acg.block(0,0,Af.rows(),Af.cols()) = Af;
    Acg.block(Af.rows(),Af.cols(),Ag.rows(),Ag.cols()) = Ag;
    bcg.block(0,0,bc.rows(),1) = bc;
    bcg.block(bc.rows(),0,bg.rows(),1) = bg;
    //Project and compute H and h

    dd_set_global_constants();
    dd_ErrorType error;
    dd_MatrixPtr matrix = FromEigen(bcg, Acg, &error);
    dd_PolyhedraPtr poly = dd_DDMatrix2Poly(matrix, &error);
    dd_MatrixPtr G = dd_CopyGenerators(poly);
    // now copy that to eigen matrix
    int realRowSize = 0;
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(G->rowsize, G->colsize);
    for(int i=1; i <= G->rowsize; i++)
    {
        if(*(G->matrix[i-1][0]) == 0)
        {
            for(int j=2; j <= G->colsize; j++)
            {
                V(realRowSize, j-1) = (double)(*(G->matrix[i-1][j-1]));
            }
            ++realRowSize;
        }
    }
    V = V.block(0,0,realRowSize, V.cols()-1);
    Eigen::MatrixXd Pac(A.rows() + C.rows(), A.cols());
    Pac.block(0,0,A.rows(),A.cols()) = A;
    Pac.block(A.rows(),0,C.rows(),C.cols()) = C;
    Eigen::MatrixXd Vp = V * Pac.transpose();


    dd_MatrixPtr Vpc = FromEigen(Vp, &error);
    dd_PolyhedraPtr polyH = dd_DDMatrix2Poly(Vpc, &error);
    dd_MatrixPtr Hc = dd_CopyInequalities(polyH);
    Eigen::MatrixXd H(Hc->rowsize, Hc->colsize-1);
    Eigen::VectorXd h(Hc->rowsize, 1);
    for(int i=1; i <= Hc->rowsize; i++)
    {
        h(i-1)= (double)(*(Hc->matrix[i-1][0]));
        for(int j=2; j <= Hc->colsize; j++)
        {
            H(i-1, j-2) = -(double)(*(Hc->matrix[i-1][j-1]));
        }
    }
    //now reconvert to H rep
    dd_FreeMatrix(G);dd_FreeMatrix(Hc);dd_FreeMatrix(matrix);dd_FreePolyhedra(poly);dd_FreePolyhedra(polyH);
    dd_free_global_constants();
}
}


namespace
{
void GroundEquilibriumTest(bool& error)
{
    Eigen::MatrixXd contactTransforms = Eigen::MatrixXd::Zero(4,4);
    Eigen::MatrixXd graspTransforms = Eigen::MatrixXd::Zero(4,4);
    Eigen::VectorXd maxGraspingForces = Eigen::VectorXd::Zero(3);
    equilib::CheckEquilibrium( contactTransforms, graspTransforms, maxGraspingForces, 1, 600);
}
}

int main(int argc, char *argv[])
{
	std::cout << "performing tests... \n";
	bool error = false;
    GroundEquilibriumTest(error);
	if(error)
	{
		std::cout << "There were some errors\n";
		return -1;
	}
	else
	{
		std::cout << "no errors found \n";
		return 0;
	}
}

