#include "Generator.h"
#include "MatrixDefsInternal.h"

#include <time.h>

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
        Object* sampled = objects_[rand() % (objects_.size())];
        const Tri& triangle = sampled->GetModel()->tris[rand() % (sampled->GetModel()->num_tris)];
        /*for(int i =0; i<3; ++i)
        {
            pos(i) = (rand() % (1001) / 1000) * 3 + triangle.p1[i];
        }*/
        /*for(int i =0; i<3; ++i)
        {
            pos(i) = (rand() % (max-min + 1) + min);
        }*/
        //http://stackoverflow.com/questions/4778147/sample-random-point-in-triangle
        Eigen::Vector3d A, B, C;
        DoubleArrayToVector(A, triangle.p1);
        DoubleArrayToVector(B, triangle.p2);
        DoubleArrayToVector(C, triangle.p3);
        double r1, r2;
        r1 = ((double) rand() / (RAND_MAX)); r2 = ((double) rand() / (RAND_MAX));
        Eigen::Vector3d P = (1 - sqrt(r1)) * A + (sqrt(r1) * (1 - r2)) * B + (sqrt(r1) * r2) * C;
        configuration.SetPosition(P);
        // random rotation
        double rx = ((double) rand() / (RAND_MAX)); double ry = ((double) rand() / (RAND_MAX)); double rz = ((double) rand() / (RAND_MAX));
        matrices::Matrix3 tranform = matrices::Rotx3(rx);
        tranform*= matrices::Roty3(ry);
        tranform*= matrices::Rotz3(rz);
        configuration.SetOrientation(tranform);
        // find random direction
        int limit2 = 100;
        while (limit2 >0)
        {
            Eigen::Vector3d dir((double) rand() / (RAND_MAX), (double) rand() / (RAND_MAX), (double) rand() / (RAND_MAX));
            if(dir.norm() == 0) break;
            dir.normalize();
            // add random direction and check for collision
            while(configuration.englobing->IsColliding(sampled))
            {
                if(!collider_.IsColliding(configuration.englobed))
                {
                    return new Object(*configuration.englobed);
                }
                configuration.SetPosition(configuration.GetPosition() + (double) rand() / (RAND_MAX) / 2 * dir);
            }
            --limit2;
        }
    }
    return 0;
}
