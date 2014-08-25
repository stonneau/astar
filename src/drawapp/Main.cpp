

#include "collision/ParserObj.h"
#include "collision/Object.h"
#include "collision/Collider.h"

#include "tools/MatrixDefs.h"

#include <drawstuff/drawstuff.h> // The drawing library for ODE;

#include "prm/SimplePRM.h"
#include "prm/Scenario.h"

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
    static float xyz[3] = {8,-1.3,2.5};
    static float hpr[3] = {180.0,-10.0,0.0};
	planner::Scenario* scenario;
    bool pathOn = false;
    bool drawObject = true;
    std::string outpath("../tests/testSerialization.txt");
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

    planner::Object::CT_Object path;
    void DrawObject(const planner::Object* obj)
    {
        // TODO DRAW OFFSET

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

    void LineBetweenObjects(const planner::Object* a, const planner::Object* b)
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
        for(planner::Object::T_Object::iterator it = scenario->objects_.begin();
            it != scenario->objects_.end();
            ++it)
        {
            DrawObject(*it);
        }
        if(pathOn)
        {
            dsSetColorAlpha(0,0, 0,1);
            bool afterfirst = false;
            planner::Object::CT_Object::iterator it = path.begin();
            for(planner::Object::CT_Object::iterator it2 = path.begin();
                it2 != path.end();
                ++it2)
            {
                dsSetColorAlpha(1,0, 0,1);
                if(drawObject) DrawObject(*it);
                if(afterfirst)
                {
                    dsSetColorAlpha(0,1, 0,1);
                    LineBetweenObjects(*it, *it2);
                    ++it;
                }
                else
                {
                    afterfirst = true;
                }
            }
        }
        else
        {
            int i = 0;
            for(planner::Object::T_Object::const_iterator it = scenario->prm->GetPRMNodes().begin();
                it != scenario->prm->GetPRMNodes().end();
                ++it, ++i)
            {
                dsSetColorAlpha(1,0, 0,1);
				if(drawObject) DrawObject(*it);
                const std::vector< int > connexions = scenario->prm->GetConnections(i);
                dsSetColorAlpha(0,1, 0,1);
                for(unsigned int j = 0; j< connexions.size(); ++j)
                {
                    LineBetweenObjects(*it, scenario->prm->GetPRMNodes()[connexions[j]]);
                }
            }
        }
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
    //scenario = new planner::Scenario("../tests/testscenario.txt");
    //scenario = new planner::Scenario("../tests/testscenarioload.txt");
    //scenario = new planner::Scenario("../tests/evac.txt");
    scenario = new planner::Scenario("../tests/zombi.txt");
	std::cout << "done" << std::endl;
    //path = scenario->prm->GetPath(*(scenario->prm->GetPRMNodes()[0]),*(scenario->prm->GetPRMNodes()[scenario->prm->GetPRMNodes().size()-1]), 10.f);
    if(path.empty())
    {
        path.push_back(scenario->prm->GetPRMNodes()[0]);
        path.push_back(scenario->prm->GetPRMNodes()[10]);
    }
    dsSetViewpoint (xyz,hpr);
}

void command(int cmd)   /**  key control function; */
{
    switch (cmd)
    {
        case 'e' :
            pathOn = !pathOn;
        break;
        case 'p' :
            drawObject = !drawObject;
        break;
        case 's' :
			planner::SavePrm(*(scenario->prm), outpath);
        break;
    }
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
