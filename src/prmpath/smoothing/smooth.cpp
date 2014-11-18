#include "smooth.h"
#include "spline/b_spline.h"
#include "prm/Model.h"

#include <vector>
#include <utility>
#include <random>

namespace planner
{
typedef std::vector<Model*> T_Model;
typedef T_Model::iterator IT_Model;
typedef std::vector<const Model*> CT_Model;
typedef CT_Model::const_iterator CIT_Model;

typedef std::pair<double, Configuration> MilePoint;
typedef std::vector<MilePoint> T_MilePoint;

typedef spline::b_spline<double, double, 3, true, Eigen::Vector3d> b_spline_t;


Configuration MakeConfiguration(const Model* model)
{
    return std::make_pair(model->GetPosition(), model->GetOrientation());
}

T_MilePoint CreateMilePoints(const CT_Model& path)
{
    T_MilePoint res;
    // simple distance heuristic for starters, it is not important
    std::vector<double> distances;
    double totalDistance = 0;
    CIT_Model cit = path.begin();
    CIT_Model cit2 = path.begin(); ++cit2;
    distances.push_back(0);
    for(; cit2 != path.end(); ++cit, ++cit2)
    {
        totalDistance+= ((*cit2)->GetPosition() - (*cit)->GetPosition()).norm();
        distances.push_back(totalDistance);
    }

    std::vector<double>::const_iterator dit = distances.begin();
    cit = path.begin();
    for(; cit != path.end(); ++cit, ++dit)
    {
        res.push_back(std::make_pair((*dit), MakeConfiguration(*cit)));
    }
    return res;
}

Configuration Interpolate(const MilePoint& a, const MilePoint& b, double t)
{
    double t0 = a.first;
    double t1 = b.first;
    double tp = (t - t0) / (t1 - t0);

    const Eigen::Quaterniond qa(a.second.second);
    const Eigen::Quaterniond qb(b.second.second);

    const Eigen::Vector3d& va = a.second.first;
    const Eigen::Vector3d& vb = b.second.first;

    Eigen::Vector3d offset = va + tp * (vb - va);
    Eigen::Quaterniond qres  = qa.slerp(tp, qb);

    return std::make_pair(offset, qres.matrix());
}

void InsertSorted(std::vector<double>& knots, double newValue)
{
    for(std::vector<double>::iterator it = knots.begin(); it != knots.end(); ++it)
    {
        if(newValue < *it)
        {
            knots.insert(it, newValue);
            return;
        }
    }
    knots.push_back(newValue);
}

SplinePath::SplinePath(const std::vector<Eigen::Vector3d>& controlPoints, const std::vector<double>& knots, double scale)
    : controlPoints_(controlPoints)
    , knots_(knots)
    , scale_(scale)
{
    //
}

SplinePath::~SplinePath()
{
    //
}


Eigen::Vector3d deBoor(int k, int degree, int i, double x, const std::vector<double>& knots, const std::vector<Eigen::Vector3d>& ctrlPoints)
{   // Please see wikipedia page for detail
    // note that the algorithm here kind of traverses in reverse order
    // comapred to that in the wikipedia page
    if( k == 0)
        return ctrlPoints[i];
    else
    {
        double alpha = (x-knots[i])/(knots[i+degree+1-k]-knots[i]);
        return (deBoor(k-1,degree, i-1, x, knots, ctrlPoints)*(1-alpha) + deBoor(k-1,degree, i, x, knots, ctrlPoints)*alpha );
    }
}

// implementation of de boor algorithm
Configuration SplinePath::operator ()(double t)
{
    // compute interval
    int interval =-1;
    int ti = knots_.size();
    for(int i=1;i<ti-1;i++)
    {
        if(t<knots_[i])
        {
            interval = (i-1);
            break;
        }
        else if(t == knots_[ti-1])
        {
            interval = (ti-1);
            break;
        }
    }
    return std::make_pair(deBoor(3,3,interval,t,knots_,controlPoints_), Eigen::Matrix3d::Identity());
}

}

using namespace planner;

struct NormalizedPath
{

    NormalizedPath(const CT_Model& path)
        : milePoints_(CreateMilePoints(path))
        , totaldistance_(milePoints_.back().first)
    {}

    ~NormalizedPath()
    {
        // NOTHING
    }

    Configuration operator()(double t) const
    {
        //assert(0 <= t && t <= 1);
        T_MilePoint::const_iterator cit = milePoints_.begin();
        T_MilePoint::const_iterator cit2 = milePoints_.begin(); ++cit2;
        for(; cit2 != milePoints_.end(); ++cit, ++cit2)
        {
            if((*cit).first <= t && t < (*cit2).first)
            {
                return Interpolate(*cit, *cit2, t);
            }
        }
        return milePoints_.back().second;
    }

    const T_MilePoint milePoints_;
    const double totaldistance_;
};

namespace
{
    std::vector<double> UniformSample(double ta, double tb, int nbSamples)
    {
        std::vector<double> knots;
        knots.push_back(ta);
        double stepsize = 1 / nbSamples;
        double t = 0;
        for(int i = 0; i < nbSamples - 2; ++i, t = t + stepsize) // -1 because ta and tb are inserted
        {
            InsertSorted(knots, t);
        }
        knots.push_back(tb);
        return knots;
    }

    std::vector<double> RandomSample(double ta, double tb, int nbSamples)
    {
        std::vector<double> knots;
        knots.push_back(ta);
        for(int i = 0; i <nbSamples - 2; ++i) // -1 because ta and tb are inserted
        {
            double t = (tb - ta) * ((double)rand() / (double)RAND_MAX ) + ta;
            InsertSorted(knots, t);
        }
        knots.push_back(tb);
        return knots;
    }

    void ReconfigureTime(std::vector<double>& knots, double maxSpeed)
    {
        // assuming we can't go too fast all the time
        // TODO HANDLE ALL JOINT BUT F...
        maxSpeed *= 0.75;
        std::vector<double>::iterator it = knots.begin();
        for(;it!=knots.end(); ++it)
        {
            *it /= maxSpeed;
        }
    }

    struct Delta
    {
        Delta(std::vector<double> knots, int m)
            : knots_(knots)
            , m_(m)
        {
            // NOTHING
        }

        ~Delta()
        {
            // NOTHING
        }

        double operator()(int id)
        {
            if(id == -1 || id == m_) return 0;
            return knots_[id+1] - knots_[id];
        }

        std::vector<double> knots_;
        int m_;
    };

}

#include <iostream>

planner::SplinePath* planner::SplineFromPath(Collider& collider, CT_Model& path, double maxSpeed, double maxAcceleration)
{
    int m = path.size();
    NormalizedPath initPath(path);

    //sample m + 1 points along the linear trajectory
    double ta = 0;  double tb = initPath.totaldistance_;

    //sample m + 1 points between ta and tb
    std::vector<double> knots = RandomSample(ta, tb, m+1);
    // TODO REALIGN MAX DISTANCE TO INCLUDE THIS
    //ReconfigureTime(knots, maxSpeed);

    //Reconfigure knots so that respect max velocity and acceleration

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(m+1, m+1);
    Eigen::MatrixXd r = Eigen::MatrixXd::Zero(m+1,3);
    Eigen::MatrixXd sol = Eigen::MatrixXd::Zero(m+1,3);


    Delta d(knots,m);

    for(int k=1; k<m;++k)
    {
        double alpha, beta, gamma;

        alpha = (d(k)*d(k))/(d(k-2)+d(k-1)+d(k));
        beta = (d(k) * (d(k-2)+d(k-1))) / (d(k-2)+d(k-1)+d(k)) +(d(k-1)*(d(k) + d(k+1)))/(d(k-1) + d(k) + d(k+1));
        gamma =(d(k-1)*d(k-1)) /(d(k-1) + d(k) + d(k+1));

        A(k,k-1) = alpha; // alphak
        A(k,k) = beta; // betak
        A(k,k+1) = gamma; // gammak
        Eigen::Vector3d vr = initPath(knots[k]).first * (d(k-1)+d(k));
        for(int i=0; i<3; ++i)
        {
            r(k,i) = vr(i);
        }
    }
    A(0,0) = d(0) + 2 *d(1); // beta0
    A(0,1) = -d(0); // gamma0
    A(m,m-1) = -d(m-1); // betam
    A(m,m) = d(m-2)+2*d(m-1); // gammam
    {
        Eigen::Vector3d vr0 = initPath(knots[0]).first * (d(0)+d(1));
        for(int i=0; i<3; ++i)
        {
            r(0,i) = vr0(i);
        }
    }
    {
        Eigen::Vector3d vrm = initPath(knots.back()).first * (d(m-2)+d(m-1));
        for(int i=0; i<3; ++i)
        {
            r(m,i) = vrm(i);
        }
    }
    std::cout << " A " << std::endl << A << std::endl;
    std::cout << " r " << std::endl << r << std::endl;
    sol = A.colPivHouseholderQr().solve(r);

    std::vector<Eigen::Vector3d> controlPoints;
    std::vector<double> normalizedKnots;
    controlPoints.push_back(initPath(0).first);
    for(int i =0; i<sol.rows();++i)
    {
        controlPoints.push_back(sol.block<1,3>(i,0));
    }
    controlPoints.push_back(initPath.milePoints_.back().second.first);
    /*multiplicity at start and end*/
    for(int i =0; i<3;++i)
    {
        normalizedKnots.push_back(0);
    }
    for(int i =0; i<knots.size();++i)
    {
        normalizedKnots.push_back(knots[i]/knots.back());
    }

    for(int i =0; i<3;++i)
    {
        normalizedKnots.push_back(1);
    }
    //normalizedKnots.push_back(1);
    return new SplinePath(controlPoints, normalizedKnots,knots.back());
}

planner::SplinePath planner::SplineShortCut(Collider& collider, CT_Model& path, int nbSteps)
{
    int m = 4;
    //normalize input path
    NormalizedPath initPath(path);

    //transform it into a spline for starters...


    for(int step =0; step<nbSteps; ++step)
    {
        // sample 2 points t_a et t_b
        double ta, tb;
        do
        {
            ta = std::rand() / RAND_MAX;
            tb = std::rand() / RAND_MAX;
            if(tb < ta)
            {
                double tmp = ta; tb = ta; ta = tmp;
            }
        } while(std::abs(ta - tb) < 0.01);

        // compute derivatives at extremity
        Eigen::Vector3d uap, ubp;
        Configuration uaplus, uaminus;
        Configuration ubplus, ubminus;
        const double epsilon = 0.0001;
        uaplus = initPath(ta+epsilon); uaminus = initPath(ta-epsilon);
        ubplus = initPath(tb+epsilon); ubminus = initPath(ta-epsilon);
        uap = (uaplus.first -uaminus.first) / (2 * epsilon);
        ubp = (ubplus.first -ubminus.first) / (2 * epsilon);

        //sample m + 1 points between ta and tb
        std::vector<double> knots;
//std::vector<Eigen::Vector3d> controlPoints;
        knots.push_back(ta);
        for(int i = 0; i < m-1;) // -1 because ta and tb are inserted
        {
            double t = (tb - ta) * ((double)rand() / (double)RAND_MAX ) + ta;
            InsertSorted(knots, t);
        }
        knots.push_back(tb);
/*for(std::vector<double>::iterator it = knots.begin(); it!= knots.end(); ++it)
{
    controlPoints.push_back(initPath(*it).first);
}*/
        //Computing de Boor control points, solving Ad = r
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(m+1, 3 * (m-1));
        Eigen::VectorXd r = Eigen::VectorXd::Zero(m+1);
        Eigen::VectorXd d = Eigen::VectorXd::Zero(m+1);

        for(int k=1; k<m-1;++k)
        {
            A(k,k-1) = 1; // alphak
            A(k,k-1) = 4; // betak
            A(k,k-1) = 1; // gammak
           // r(k) = initPath(knots[k]).first * 6;
        }
    }
}
