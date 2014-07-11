

#include "collision/ParserObj.h"
#include "collision/Object.h"

#include <drawstuff/drawstuff.h> // The drawing library for ODE;

#include <string>
#include <iostream>
#include <cmath>


#ifdef WIN32
#include <windows.h>
#endif

using namespace std;

namespace
{
    planner::Object::T_Object objects;
    void DrawObject(planner::Object* obj)
    {
        dsSetColorAlpha(0,0, 0,1);
        for(int i =0; i< obj->GetModel()->num_tris; ++i)
        {
            const Tri& t = obj->GetModel()->tris[i];
            dsDrawLineD(t.p1, t.p2);
            dsDrawLineD(t.p2, t.p3);
            dsDrawLineD(t.p3, t.p1);
        }
    }


    void DrawObjects()
    {
        for(planner::Object::T_Object::iterator it = objects.begin();
            it != objects.end();
            ++it)
        {
            DrawObject(*it);
        }
    }
}

static float xyz[3] = {-10.0,1,6.0};
static float hpr[3] = {0.0,0.0,0.0};

static void simLoop (int pause)
{
    //drawManager.Draw();
     DrawObjects();
}

void start()
{
    //dsSetViewpoint (xyz,hpr);
    std::string targetFile("../tests/collision/armoire.obj");
    planner::ParserObj parser;
    objects = parser.CreateWorld(targetFile);
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
