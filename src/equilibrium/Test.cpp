#include "Test.h"
#include "libcdd/setoper.h"
#include "libcdd/cdd.h"

#include <vector>

#include <iostream>
using namespace std;
namespace equilib
{

const Vector X(1,0,0);
const Vector Y(0,1,0);
const Vector Z(0,0,1);

Eigen::Matrix3d skew(const Eigen::Vector3d& x)
{
    Eigen::Matrix3d res = Eigen::Matrix3d::Zero();
    res(0,1) = - x(2); res(0,2) =   x(1);
    res(1,0) =   x(2); res(1,2) = - x(0);
    res(2,0) = - x(1); res(2,1) =   x(0);
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
// ajoute acceleration, mass et localisation
bool CheckEquilibrium(const T_Transform &contactTransforms, const T_Transform &graspTransforms, const Eigen::VectorXd& maxGraspingForces,
                      const Eigen::Vector3d& acceleration, const Eigen::Vector3d& comLocation,
                      const float mass, float friction, double flimit)
{
    //Compute Ac
    // Ac = [Af, Av]'
        // Compute Af matches contact forces
        // Af = diag(B_1,B_N)
    int nbContacts = contactTransforms.size();
    int nbGrasps = graspTransforms.size();
    Eigen::MatrixXd C(3, (nbContacts + nbGrasps) * 3);
    Eigen::MatrixXd A(3, (nbContacts + nbGrasps) * 3);
    // Init A with identities
    Eigen::Matrix3d id = Eigen::Matrix3d::Identity();
    for(int k = 0; k < nbContacts + nbGrasps; ++k)
    {
        A.block<3,3>(0, 3*k) = id;
    }
    Eigen::MatrixXd Af, Ag;
    Eigen::VectorXd bc, bg;
    int afRows, afCols, agRows, agCols;
    agRows = nbGrasps * 6;
    agCols = nbGrasps * 3;
    afRows = (nbContacts > 0) ?  (nbContacts * 4 +1)  : 0;
    afCols = nbContacts * 3;
    if(nbContacts > 0)
    {
        Af = Eigen::MatrixXd::Zero(afRows, afCols);
        int acIndex = Af.rows()-1;
        bc = Eigen::VectorXd::Zero(afRows);
        bc(acIndex) = flimit;

        for(int i=0; i< nbContacts; ++i)
        {
            C.block<3,3>(0,3*i) = skew(contactTransforms[i].block<3,1>(0, 3));
            const Rotation& Ri = contactTransforms[i].block<3,3>(0, 0);
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
    }
    if(nbGrasps > 0)
    {
        //Compute Ag
        Ag = Eigen::MatrixXd::Zero(agRows, agCols);
        bg = Eigen::VectorXd::Zero(agRows);
        for(int i=0; i< nbGrasps; ++i)
        {
            C.block<3,3>(0,3*(i+nbContacts)) = skew(graspTransforms[i].block<3,1>(0, 3));
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
    }
    //Eigen::MatrixXd Acg = Eigen::MatrixXd::Zero(Af.rows() + Ag.rows(), Af.cols() + Ag.cols());
    Eigen::MatrixXd Acg = Eigen::MatrixXd::Zero(afRows + agRows, afCols + agCols);
    Eigen::MatrixXd bcg(afRows + agRows, 1) ;
    if(nbContacts>0)
    {
        Acg.block(0,0,afRows,afCols) = Af;
        bcg.block(0,0,afRows,1) = bc;
    }
    if(nbGrasps>0)
    {
        Acg.block(afRows,afCols,agRows,agCols) = Ag;
        bcg.block(afRows,0,agRows,1) = bg;
    }
    //Project and compute H and h

    dd_set_global_constants();
    dd_ErrorType error;
    dd_MatrixPtr matrix = FromEigen(bcg, Acg, &error);
    dd_PolyhedraPtr poly = dd_DDMatrix2Poly(matrix, &error);
    dd_MatrixPtr G = dd_CopyGenerators(poly);
    // now copy that to eigen matrix
    int realRowSize = 0;
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(G->rowsize, G->colsize-1);
    for(int i=1; i <= G->rowsize; i++)
    {
        if(*(G->matrix[i-1][0]) == 1)
        {
            for(int j=2; j <= G->colsize; j++)
            {
                V(realRowSize, j-2) = (double)(*(G->matrix[i-1][j-1]));
            }
            ++realRowSize;
        }
    }
    V = V.block(0,0,realRowSize, V.cols());
    Eigen::MatrixXd Pac(A.rows() + C.rows(), A.cols());
    Pac.block(0,0,A.rows(),A.cols()) = A;
    Pac.block(A.rows(),0,C.rows(),C.cols()) = C;
    Eigen::MatrixXd Vp = V * Pac.transpose();


    dd_MatrixPtr Vpc = FromEigen(Vp, &error);
	/*std::cout << "Acg" << std::endl << Acg << std::endl;
	std::cout << "Af" << std::endl << Af << std::endl;
	std::cout << "bc" << std::endl << bc << std::endl;
	std::cout << "bcg" << std::endl << bcg << std::endl;
	std::cout << "V" << std::endl << V << std::endl;
	std::cout << "Vp" << std::endl << Vp << std::endl;*/
    dd_PolyhedraPtr polyH = dd_DDMatrix2Poly(Vpc, &error);
    dd_MatrixPtr Hc = dd_CopyInequalities(polyH);
    Eigen::MatrixXd H(Hc->rowsize, Hc->colsize-1);
    Eigen::MatrixXd h(Hc->rowsize, 1);
    for(int i=1; i <= Hc->rowsize; i++)
    {
        h(i-1,0)= (double)(*(Hc->matrix[i-1][0]));
        for(int j=2; j <= Hc->colsize; j++)
        {
            H(i-1, j-2) = -(double)(*(Hc->matrix[i-1][j-1]));
        }
    }
    Eigen::Vector3d gravity(0,0,-9.81);
    Eigen::Vector3d W = mass * acceleration - mass * gravity;
    Eigen::MatrixXd H1 = H.block(0,0,H.rows(),3);
    Eigen::MatrixXd H2 = H.block(0,3,H.rows(),3);
    Eigen::MatrixXd res = (H1 + H2 * skew(comLocation)) * W - h;
    //now reconvert to H rep
    dd_FreeMatrix(G);dd_FreeMatrix(Hc);dd_FreeMatrix(matrix);dd_FreePolyhedra(poly);dd_FreePolyhedra(polyH);
    dd_free_global_constants();
	/*std::cout << "Acg" << std::endl << Acg << std::endl;
	std::cout << "bcg" << std::endl << bcg << std::endl;
	std::cout << "V" << std::endl << V << std::endl;
	std::cout << "Vp" << std::endl << Vp << std::endl;
	std::cout << "Pac" << std::endl << Pac << std::endl;
	std::cout << "H" << std::endl << H << std::endl;
	std::cout << "h" << std::endl << h << std::endl;
	std::cout << "h1" << std::endl << H1 << std::endl;
	std::cout << "h2" << std::endl << H2 << std::endl;
	std::cout << "res" << std::endl << res << std::endl;*/
    for(int i =0; i< res.rows();++i)
    {
        if(res(i)>0) return false;
    }
    return true;
}


// human motions analysis and simulation based on a
// general criterion of stability
// ajoute acceleration, mass et localisation
double ResidualRadius(const T_Transform &contactTransforms, const T_Transform &graspTransforms, const Eigen::VectorXd& maxGraspingForces,
                      const Eigen::Vector3d& acceleration, const Eigen::Vector3d& comLocation,
                      const float mass, float friction, double flimit)
{
    //Compute Ac
    // Ac = [Af, Av]'
        // Compute Af matches contact forces
        // Af = diag(B_1,B_N)
    int nbContacts = contactTransforms.size();
    int nbGrasps = graspTransforms.size();
    Eigen::MatrixXd C(3, (nbContacts + nbGrasps) * 3);
    Eigen::MatrixXd A(3, (nbContacts + nbGrasps) * 3);
    // Init A with identities
    Eigen::Matrix3d id = Eigen::Matrix3d::Identity();
    for(int k = 0; k < nbContacts + nbGrasps; ++k)
    {
        A.block<3,3>(0, 3*k) = id;
    }
    Eigen::MatrixXd Af, Ag;
    Eigen::VectorXd bc, bg;
    int afRows, afCols, agRows, agCols;
    agRows = nbGrasps * 6;
    agCols = nbGrasps * 3;
    afRows = (nbContacts > 0) ?  (nbContacts * 4 +1)  : 0;
    afCols = nbContacts * 3;
    if(nbContacts > 0)
    {
        Af = Eigen::MatrixXd::Zero(afRows, afCols);
        int acIndex = Af.rows()-1;
        bc = Eigen::VectorXd::Zero(afRows);
        bc(acIndex) = flimit;

        for(int i=0; i< nbContacts; ++i)
        {
            C.block<3,3>(0,3*i) = skew(contactTransforms[i].block<3,1>(0, 3));
            const Rotation& Ri = contactTransforms[i].block<3,3>(0, 0);
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
    }
    if(nbGrasps > 0)
    {
        //Compute Ag
        Ag = Eigen::MatrixXd::Zero(agRows, agCols);
        bg = Eigen::VectorXd::Zero(agRows);
        for(int i=0; i< nbGrasps; ++i)
        {
            C.block<3,3>(0,3*(i+nbContacts)) = skew(graspTransforms[i].block<3,1>(0, 3));
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
    }
    //Eigen::MatrixXd Acg = Eigen::MatrixXd::Zero(Af.rows() + Ag.rows(), Af.cols() + Ag.cols());
    Eigen::MatrixXd Acg = Eigen::MatrixXd::Zero(afRows + agRows, afCols + agCols);
    Eigen::MatrixXd bcg(afRows + agRows, 1) ;
    if(nbContacts>0)
    {
        Acg.block(0,0,afRows,afCols) = Af;
        bcg.block(0,0,afRows,1) = bc;
    }
    if(nbGrasps>0)
    {
        Acg.block(afRows,afCols,agRows,agCols) = Ag;
        bcg.block(afRows,0,agRows,1) = bg;
    }
    //Project and compute H and h

    dd_set_global_constants();
    dd_ErrorType error;
    dd_MatrixPtr matrix = FromEigen(bcg, Acg, &error);
    dd_PolyhedraPtr poly = dd_DDMatrix2Poly(matrix, &error);
    dd_MatrixPtr G = dd_CopyGenerators(poly);
    // now copy that to eigen matrix
    int realRowSize = 0;
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(G->rowsize, G->colsize-1);
    for(int i=1; i <= G->rowsize; i++)
    {
        if(*(G->matrix[i-1][0]) == 1)
        {
            for(int j=2; j <= G->colsize; j++)
            {
                V(realRowSize, j-2) = (double)(*(G->matrix[i-1][j-1]));
            }
            ++realRowSize;
        }
    }
    V = V.block(0,0,realRowSize, V.cols());
    Eigen::MatrixXd Pac(A.rows() + C.rows(), A.cols());
    Pac.block(0,0,A.rows(),A.cols()) = A;
    Pac.block(A.rows(),0,C.rows(),C.cols()) = C;
    Eigen::MatrixXd Vp = V * Pac.transpose();


    dd_MatrixPtr Vpc = FromEigen(Vp, &error);
	/*std::cout << "Acg" << std::endl << Acg << std::endl;
	std::cout << "Af" << std::endl << Af << std::endl;
	std::cout << "bc" << std::endl << bc << std::endl;
	std::cout << "bcg" << std::endl << bcg << std::endl;
	std::cout << "V" << std::endl << V << std::endl;
	std::cout << "Vp" << std::endl << Vp << std::endl;*/
    dd_PolyhedraPtr polyH = dd_DDMatrix2Poly(Vpc, &error);
    dd_MatrixPtr Hc = dd_CopyInequalities(polyH);
    Eigen::MatrixXd H(Hc->rowsize, Hc->colsize-1);
    Eigen::MatrixXd h(Hc->rowsize, 1);
    for(int i=1; i <= Hc->rowsize; i++)
    {
        h(i-1,0)= (double)(*(Hc->matrix[i-1][0]));
        for(int j=2; j <= Hc->colsize; j++)
        {
            H(i-1, j-2) = -(double)(*(Hc->matrix[i-1][j-1]));
        }
    }
    Eigen::Vector3d gravity(0,0,-9.81);
    Eigen::Vector3d W = mass * acceleration - mass * gravity;
    Eigen::MatrixXd H1 = H.block(0,0,H.rows(),3);
    Eigen::MatrixXd H2 = H.block(0,3,H.rows(),3);
    H = (H1 + H2 * skew(comLocation));
    Eigen::MatrixXd res = H * W - h;
    //now reconvert to H rep
    dd_FreeMatrix(G);dd_FreeMatrix(Hc);dd_FreeMatrix(matrix);dd_FreePolyhedra(poly);dd_FreePolyhedra(polyH);
    dd_free_global_constants();
    for(int i =0; i< res.rows();++i)
    {
        if(res(i)>0) return -1;
    }
    for(int i=0; i< H.rows(); ++i)
	{
		H.block(i,0,1,H.cols()) = H.block(i,0,1,H.cols()) / H.block(i,0,1,H.cols()).norm();
		h(i) = h(i) / H.block(i,0,1,H.cols()).norm();
	}
	return (h - H * W).minCoeff();
}
}
