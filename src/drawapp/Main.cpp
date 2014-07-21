

#include "collision/ParserObj.h"
#include "collision/Object.h"
#include "collision/Collider.h"

#include "tools/MatrixDefs.h"

#include <drawstuff/drawstuff.h> // The drawing library for ODE;

#include "prm/SimplePRM.h"

#include <string>
#include <iostream>
#include <cmath>


#ifdef WIN32
#include <windows.h>
#endif

using namespace std;
using namespace Eigen;


namespace
{
    static float xyz[3] = {-10.0,1,6.0};
    static float hpr[3] = {0.0,0.0,0.0};
    planner::SimplePRM * prm;
    planner::Model robot;
}

namespace
{
    void createTransform(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& position, double R [12])
    {
        for(int i =0; i< 3; ++i)
        {
            for(int j =0; j< 3; ++j)
            {
                R[ 4*i + j ] = rotation(i,j);
            }
        }
        for(int i = 0; i<3; ++i)
        {
            R[ 4*i + 3 ] = position(i);
        }
    }

    void arrayToVect3(const double * tab, Vector3d& vect)
    {
        for(int i =0; i< 3; ++i)
        {
            vect(i) = tab[i];
        }
    }

    void Vect3ToArray(double * tab, const Vector3d& vect)
    {
        for(int i =0; i< 3; ++i)
        {
            tab[i] =  vect(i);
        }
    }

    planner::Object::T_Object objects;
    void DrawObject(planner::Object* obj)
    {
        // TODO DRAW OFFSETE

        PQP_REAL p1 [3];
        PQP_REAL p2 [3];
        PQP_REAL p3 [3];
        //createTransform(obj->GetOrientation(), obj->GetPosition(), R);
        for(int i =0; i< obj->GetModel()->num_tris; ++i)
        {
            const Tri& t = obj->GetModel()->tris[i];
            Vector3d v1, v2, v3;
            arrayToVect3(t.p1, v1); arrayToVect3(t.p2, v2); arrayToVect3(t.p3, v3);
            v1 = obj->GetOrientation() * v1; v1 = v1 + obj->GetPosition();
            v2 = obj->GetOrientation() * v2; v2 = v2 + obj->GetPosition();
            v3 = obj->GetOrientation() * v3; v3 = v3 + obj->GetPosition();
            Vect3ToArray(p1, v1); Vect3ToArray(p2, v2); Vect3ToArray(p3, v3);
            dsDrawLineD(p1, p2);
            dsDrawLineD(p2, p3);
            dsDrawLineD(p3, p1);
        }
    }

    void LineBetweenObjects(planner::Object* a, planner::Object* b)
    {
        PQP_REAL p1 [3];
        PQP_REAL p2 [3];
        Vect3ToArray(p1, a->GetPosition());
        Vect3ToArray(p2, b->GetPosition());
        dsDrawLineD(p1, p2);
    }


    void DrawObjects()
    {
        dsSetColorAlpha(0,0, 0,1);
        for(planner::Object::T_Object::iterator it = objects.begin();
            it != objects.end();
            ++it)
        {
            DrawObject(*it);
        }
        int i = 0;
        for(planner::Object::T_Object::const_iterator it = prm->GetPRMNodes().begin();
            it != prm->GetPRMNodes().end();
            ++it, ++i)
        {
            dsSetColorAlpha(1,0, 0,1);
            DrawObject(*it);
            const std::vector< int > connexions = prm->GetConnections(i);
            dsSetColorAlpha(0,1, 0,1);
            for(unsigned int j = 0; j< connexions.size(); ++j)
            {
                LineBetweenObjects(*it, prm->GetPRMNodes()[connexions[j]]);
            }
        }
        // then draw lines
    }
}



static void simLoop (int pause)
{
    //drawManager.Draw();
     DrawObjects();
}

void start()
{
    //dsSetViewpoint (xyz,hpr);
    std::string targetFile("../tests/collision/armoire.obj");
    std::string model("../tests/collision/cube.obj");
    std::string model2("../tests/collision/cubeenglob.obj");
    objects = planner::ParseObj(targetFile);

    planner::Object::T_Object objects2 = planner::ParseObj(model);
    planner::ParseObj(model2, objects2);
    robot.englobed = objects2[0];
    robot.englobing = objects2[1];
    prm = new planner::SimplePRM(robot, objects, 1000, 20, 4);
}

void command(int cmd)   /**  key control function; */
{

}


int main(int argc, char *argv[])
{
    /*
    drawstuff stuff*/
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = &command;
    fn.stop    = 0;
    fn.path_to_textures = "../textures";
    dsSimulationLoop (argc,argv,800,600,&fn);

    return 0;
}
