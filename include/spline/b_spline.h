/**
* \file b_spline.h
* \brief class allowing to create a b_spline.
* \author Steve T.
* \version 0.1
* \date 06/17/2013
*/


#ifndef _CLASS_B_SPLINE
#define _CLASS_B_SPLINE

#include "curve_abc.h"

#include <vector>

namespace spline
{
/// \class BezierCurve
/// \brief Represents a curve
///
template<typename Time= double, typename Numeric=Time, int Dim=3, bool Safe=false
, typename Point= Eigen::Matrix<Numeric, Dim, 1> >
struct b_spline : public  curve_abc<Time, Numeric, Dim, Safe, Point>
{
    typedef Point 	point_t;
    typedef Time 	time_t;
    typedef Numeric num_t;
    typedef time_t knot_t;

/* Constructors - destructors */
    public:
    ///\brief Constructor
    ///\param PointsBegin, PointsEnd : the points parametering the Bezier curve
    ///\TODO : so far size above 3 is ignored
    template<typename In, typename Ik>
    b_spline(In PointsBegin, In PointsEnd, Ik KnotsBegin, Ik KnotsEnd)
    : m_(std::distance(KnotsBegin, KnotsEnd)-1)
    , mn_(std::distance(PointsBegin, PointsEnd))
    , n_(m_ - mn_)
    {
        In it(PointsBegin);
        if(Safe && (mn_<=1))
        {
            // TODO assert t growing
            throw; // TODO
        }
        for(; it != PointsEnd; ++it)
        {
            pts_.push_back(*it);
        }
        Ik itk(KnotsBegin);
        for(; itk != KnotsEnd; ++itk)
        {
            knots_.push_back(*itk);
        }
    }

    ///\brief Destructor
    ~b_spline()
    {
        // NOTHING
    }

    private:
    b_spline(const b_spline&);
    b_spline& operator=(const b_spline&);
/* Constructors - destructors */

/*Operations*/
    public:
    ///  \brief Evaluation of the cubic spline at time t.
    ///  \param t : the time when to evaluate the spine
    ///  \param return : the value x(t)
    virtual point_t operator()(time_t t) const
    {
        if(Safe && (t<0 || t>1))
        {
            // TODO assert t growing
            throw; // TODO
        }
        point_t res = pts_[0] * b(0,n_,t);
        for(int i=1; i <= mn_-1; ++i)
        {
            res += pts_[i] * b(i,n_,t);
        }
        return res;
    }

    private:
    num_t b(int j, int n, time_t t) const
    {
        time_t tj, tj1;
        tj = knots_[j]; tj1 = knots_[j+1];
        if(n==0)
        {
            return (tj <= t && t < tj1) ? 1 : 0;
        }
        else
        {
            time_t tjn = knots_[j+n];
            time_t tjn1 = knots_[j+n+1];
            return (t - tj)/(tjn -tj) * b(j,n-1,t)  + (tjn1 - t)/(tjn1 - tj1) * b(j+1,n-1,t);
        }
    }
/*Operations*/

/*Helpers*/
    public:
    virtual time_t min() const{return 0;}
    virtual time_t max() const{return 1;}
/*Helpers*/

    public:
    const int m_, mn_, n_;

    private:
    typedef std::vector<Point,Eigen::aligned_allocator<Point> > T_Vector;
    typedef std::vector<knot_t> T_Knots;
    T_Vector  pts_;
    T_Knots knots_;
};
}
#endif //_CLASS_B_SPLINE

