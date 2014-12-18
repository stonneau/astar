#include "Sphere.h"


using namespace planner;

const planner::SphereCollisionRes planner::Intersect(const Sphere& R, const Sphere& r, bool computePoint)
{
	SphereCollisionRes res;
	Eigen::Vector3d axis = r.center_ - R.center_;
	double d =axis.norm();
	if(d <= std::abs(R.radius_ - r.radius_))
	{
		res.collisionType = planner::sphereContained;
    }
	else if(d <= (R.radius_ + r.radius_))		
	{
        res.collisionType = (d == (R.radius_ + r.radius_)) ? planner::tangentialExterior : planner::cercle;
		if(computePoint)
		{
			res.infoComputed = true;
			double distance = 0.5 * ( d + (R.radius_ * R.radius_ - r.radius_ * r.radius_) / d ); // distance from R
			axis.normalize();
			res.center = R.center_ + distance * axis;
			res.radius = (1 / (2*d)) * sqrt(4*d*d*R.radius_ * R.radius_ - (d*d - r.radius_ * r.radius_ + R.radius_ * R.radius_ ) * (d*d - r.radius_ * r.radius_ + R.radius_ * R.radius_ )); 
		}			
    }
	else
	{
		res.collisionType = planner::sphereContained;
    }
	return res;
}
