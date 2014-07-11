
#ifndef _CLASS_OBSTACLE
#define _CLASS_OBSTACLE

#include "MatrixDefs.h"

#include <vector>

class Obstacle {

public:
	typedef std::vector<Obstacle>		T_Obstacle;
	typedef T_Obstacle::const_iterator	CIT_Obstacle;

public:
	//make it clockwise from upper left
     Obstacle(const matrices::Vector3& /*p1*/, const matrices::Vector3& /*p2*/, const matrices::Vector3& /*p3*/);
	~Obstacle();

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	// Minimal distance between the plan described by the obstacle and a point, that is, the distance btw point and its orthonormal projection on the plan
    double Distance(const matrices::Vector3& /*point*/, matrices::Vector3& /*getCoordinates*/) const; // get coordinates of the projection
	bool IsAbove(const matrices::Vector3& /*point*/) const; // true if a point is above the obstacle, considering the normal
	//bool  ContainsPlanar(const matrices::Vector3& /*point*/) const; // point in the plan expressed in local coordinates	

	const matrices::Vector3 ProjectUp(const matrices::Vector3& /*point*/) const; // projects a points onto obstacle plan and rises it up a little
	const matrices::Vector3& Center() const { return center_; }
	const matrices::Matrix4& Basis   () const { return basis_; }
	const matrices::Matrix4& BasisInv() const { return basisInverse_; }

    double GetD() const { return d_; }
    double GetW() const { return w_; }
    double GetH() const { return h_; }
    double GetA() const { return a_; }
    double GetB() const { return b_; }
    double GetC() const { return c_; }

	const matrices::Vector3& GetP1() const{return p1_;}
	const matrices::Vector3& GetP2() const{return p2_;}
    const matrices::Vector3& GetP3() const{return p3_;}
    const matrices::Vector3& GetCenter() {return center_;}

private:
    const matrices::Vector3 p1_;
    const matrices::Vector3 p2_;
    const matrices::Vector3 p3_;
          matrices::Vector3 center_;

	matrices::Matrix4 basis_; // transformation matrix to world basis (on p4)
	matrices::Matrix4 basisInverse_; // transformation matrix to rectangle basis (on p4)

    double a_;
    double b_;
    double c_;
    double d_;
    double norm_;
    double normsquare_;
    double w_;
    double h_;

public:
    const matrices::Vector3 u_;
    const matrices::Vector3 v_;
    const matrices::Vector3 n_; // normal vector

};

#endif //_CLASS_OBSTACLE
