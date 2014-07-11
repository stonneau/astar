
#include "Obstacle.h"
#include <math.h>

using namespace matrices;
using namespace Eigen;

Obstacle::Obstacle(const Vector3& p1, const Vector3& p2, const Vector3& p3)
: p1_(p1)
, p2_(p2)
, p3_(p3)
, u_(p3-p1)
, v_(p2-p1)
, n_(u_.cross(v_))
{
    vector3 p4(p1+ (p3 - p2));
	Vector3 normal = n_;
	a_ = (float)(normal.x());
	b_ = (float)(normal.y());
	c_ = (float)(normal.z());
	//if (c_ < 0) c_ = -c_;
	norm_ = (float)(normal.norm());
	normsquare_ = norm_ * norm_;
	d_ = (float)(-(a_ * p1.x() + b_ * p1.y() + c_ * p1.z()));
	center_ = p1 + ((p4 - p1) + (p2 - p1)) / 2 ;

	basis_ = Matrix4::Zero();
	Vector3 x = (p3 - p4); x.normalize();
	Vector3 y = (p1 - p4); y.normalize();
	normal.normalize();
	basis_.block(0,0,3,1) = x;
	basis_.block(0,1,3,1) = y;
	basis_.block(0,2,3,1) = normal;
	basis_.block(0,3,3,1) = p4;
	basis_(3,3) = 1;
	basisInverse_ = basis_.inverse();

	w_ = (float)((p3 - p4).norm());
	h_ = (float)((p1 - p4).norm());
}

Obstacle::~Obstacle()
{
	// NOTHING
}

double Obstacle::Distance(const Vector3& point, Vector3& getCoordinates) const
{
	// http://fr.wikipedia.org/wiki/Distance_d%27un_point_%C3%A0_un_plan
    double lambda = - ((a_ * point.x() + b_ * point.y() + c_ * point.z() + d_) / normsquare_);
	getCoordinates(0) = lambda * a_ + point.x();
	getCoordinates(1) = lambda * b_ + point.y();
	getCoordinates(2) = lambda * c_ + point.z();
	return (abs(lambda) * norm_);
}

bool Obstacle::IsAbove(const matrices::Vector3& point) const
{
	Vector3 nNormal = n_; nNormal.normalize();
	Vector3 projection;
    double distance = Distance(point, projection);
	return (point - ( projection + distance * nNormal )).norm() < 0.000000001;
}

const matrices::Vector3 Obstacle::ProjectUp(const matrices::Vector3& point) const// projects a points onto obstacle plan and rises it up a little
{
	matrices::Vector3 res = matrices::matrix4TimesVect3(basisInverse_, point);
	matrices::Vector3 nNormal = n_; nNormal.normalize();
	res(2) = nNormal(2) * 0.1;
	return matrices::matrix4TimesVect3(basis_, res);
}

