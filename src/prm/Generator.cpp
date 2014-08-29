#include "Generator.h"
#include "MatrixDefsInternal.h"

#include <time.h>
#include <math.h>

namespace
{
    static bool generatorInit = false;

    void DoubleArrayToVector(Eigen::Vector3d& to, const PQP_REAL* from)
    {
        for(int i=0; i<3; ++i)
        {
            to(i) = from[i];
        }
    }
}

namespace
{
    double Length(const double a[3], const double b[3])
    {
        double res = 0;
        for(int i=0; i<3; ++i)
        {
            res += (b[i] - a [i]) *(b[i] - a [i]);
        }
        return sqrt(res);
    }
    // Heron formula
    double TriangleArea(const Tri& tri)
    {
        double a, b, c;
        a = Length(tri.p1, tri.p2);
        b = Length(tri.p2, tri.p3);
        c = Length(tri.p3, tri.p1);
        double s = 0.5 * (a + b + c);
        return sqrt(s * (s-a) * (s-b) * (s-c));
    }
}

using namespace planner;

Generator::Generator(Object::T_Object &objects, const Model &model)
    : model_(model)
    , objects_(objects)
    , collider_(objects)
{
    if(! ::generatorInit)
    {
        ::generatorInit = true;
        srand((unsigned int)(time(0))); //Init Random generation
    }
    InitWeightedTriangles();
}

Generator::~Generator()
{
    // NOTHING
}

Object* Generator::operator()()
{
    // en v0, juste les positions
    int limit = 10000;

    Eigen::Vector3d pos;
    while(limit > 0)
    {
        -- limit;
        Model configuration;
        configuration.englobed = new Object(*model_.englobed);
        configuration.englobing = new Object(*model_.englobing);
        // pick one object randomly
        std::pair<Object*, const Tri*> sampled;
        double r = ((double) rand() / (RAND_MAX));
        if(r > 0.3)
            sampled = RandomPointIntriangle();
        else
            sampled = WeightedTriangles();
        const Tri& triangle = *(sampled.second);
        //http://stackoverflow.com/questions/4778147/sample-random-point-in-triangle
        Eigen::Vector3d A, B, C;
        DoubleArrayToVector(A, triangle.p1);
        DoubleArrayToVector(B, triangle.p2);
        DoubleArrayToVector(C, triangle.p3);
        double r1, r2;
        r1 = ((double) rand() / (RAND_MAX)); r2 = ((double) rand() / (RAND_MAX));
        Eigen::Vector3d P = (1 - sqrt(r1)) * A + (sqrt(r1) * (1 - r2)) * B + (sqrt(r1) * r2) * C;
		if(P.z() < 2.5)
		{
			configuration.SetPosition(P);
			// random rotation
			double rx = ((double) rand() / (RAND_MAX)) * M_PI *2; double ry = ((double) rand() / (RAND_MAX)) * M_PI *2; double rz = ((double) rand() / (RAND_MAX))  * M_PI *2;
			matrices::Matrix3 tranform = matrices::Rotz3(rz);
			// find random direction
			int limit2 = 100;
			int limitstraight = 2;
			// first try with straight form
			configuration.SetOrientation(tranform);
			while (limitstraight >0)
			{
				Eigen::Vector3d dir((double) rand() / (RAND_MAX), (double) rand() / (RAND_MAX), (double) rand() / (RAND_MAX));
				if(dir.norm() == 0) break;
				dir.normalize();
				// add random direction and check for collision
				while(configuration.englobing->IsColliding(sampled.first))
				{
					if(!collider_.IsColliding(configuration.englobed))
					{
                        if(configuration.GetPosition().y() < 2.)
							return new Object(*configuration.englobed);
						break;
					}
					configuration.SetPosition(configuration.GetPosition() + (double) rand() / (RAND_MAX) / 2 * dir);
				}
				--limitstraight;
			}
			tranform*= matrices::Roty3(ry);
			tranform*= matrices::Rotx3(rx);
			configuration.SetOrientation(tranform);
			while (limit2 >0)
			{
				Eigen::Vector3d dir((double) rand() / (RAND_MAX) -0.5, (double) rand() / (RAND_MAX) -0.5, (double) rand() / (RAND_MAX) -0.5);
				// if normal check colinearity
				if(sampled.first->normals_.size() > sampled.second->id)
				{
					Eigen::Vector3d normal = sampled.first->normals_[sampled.second->id];
					normal.normalize();
					dir.normalize();
					int i = 1000;
					while((normal.dot(dir) < 0 && dir.dot(normal) < 0) && i > 0)
					{
						dir= Eigen::Vector3d((double) rand() / (RAND_MAX) -0.5, (double) rand() / (RAND_MAX) -0.5, (double) rand() / (RAND_MAX) -0.5);
						dir.normalize();
						--i;
					}
				}
				else
				{
					if(dir.norm() == 0) break;
					dir.normalize();
				}
				// add random direction and check for collision
				while(configuration.englobing->IsColliding(sampled.first))
				{
					if(!collider_.IsColliding(configuration.englobed))
					{
                        if(configuration.GetPosition().y() < 2.)
							return new Object(*configuration.englobed);
						break;
					}
					configuration.SetPosition(configuration.GetPosition() + (double) rand() / (RAND_MAX) / 2 * dir);
				}
				--limit2;
			}
		}
    }
    return 0;
}


std::pair<Object*, const Tri*>  Generator::RandomPointIntriangle()
{
    Object* sampled = objects_[rand() % (objects_.size())];
    return std::make_pair(sampled, &(sampled->GetModel()->tris[rand() % (sampled->GetModel()->num_tris)]));
}

const std::pair<Object*, const Tri*>& Generator::WeightedTriangles()
{
    double r = ((double) rand() / (RAND_MAX));
    std::vector<std::pair<Object*, const Tri*> >::const_iterator trit = triangles_.begin();
    for(std::vector<float>::iterator wit = weights_.begin();
        wit != weights_.end();
        ++wit, ++trit)
    {
        if(*wit <= r)
        {
            return *trit;
        }
    }
    return triangles_[triangles_.size()-1]; // not supposed to happen
}

void Generator::InitWeightedTriangles()
{
    float sum = 0;
    for(Object::T_Object::iterator objit = objects_.begin();
        objit != objects_.end(); ++objit)
    {
        for(int i =0; i < (*objit)->GetModel()->num_tris; ++i)
        {
            const Tri* pTri = &((*objit)->GetModel()->tris[i]);
            float weight = TriangleArea(*pTri);
            sum += weight;
            weights_.push_back(weight);
            triangles_.push_back(std::make_pair(*objit, pTri));
        }
        float previousWeight = 0;
        for(std::vector<float>::iterator wit = weights_.begin();
            wit != weights_.end();
            ++wit)
        {
            previousWeight += (*wit) / sum;
            (*wit) = previousWeight;
        }
    }
}
