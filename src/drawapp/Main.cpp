#include "collision/ParserObj.h"
#include "collision/Object.h"
#include "collision/Collider.h"
#include "tools/MatrixDefs.h"
#include <drawstuff/drawstuff.h> // The drawing library for ODE;
#include "prm/SimplePRM.h"
#include "prm/Scenario.h"
#include "prmpath/Robot.h"
#include "prmpath/sampling/Sample.h"
#include "prmpath/JointConstraint.h"
#include "CompleteScenario.h"

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Geometry>
#ifdef WIN32
#include <windows.h>
#endif

using namespace std;
using namespace Eigen;
namespace
{
    static float xyz[3] = {8,-1.3,2.5};
    static float hpr[3] = {180.0,-10.0,0.0};
    bool pathOn = false;
    bool drawObject = true;
    bool drawPOstures = false;
    std::string outpath("../tests/testSerialization.txt");
    std::string outfilename ("../tests/entrance.path");
    Eigen::Matrix3d itompTransform;
    int current = 0;
    int currentSample = 0;
    planner::Node * arm = 0;
    planner::CompleteScenario* cScenario = 0;
    std::vector<planner::Node*> postures;
}
namespace
{
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
    void DrawObject(const planner::Object* obj, bool useItomp = true)
    {
        // TODO DRAW OFFSET
        PQP_REAL p1 [3];
        PQP_REAL p2 [3];
        PQP_REAL p3 [3];
        for(int i =0; i< obj->GetModel()->num_tris; ++i)
        {
            const Tri& t = obj->GetModel()->tris[i];
            Vector3d v1, v2, v3;
            arrayToVect3(t.p1, v1); arrayToVect3(t.p2, v2); arrayToVect3(t.p3, v3);
            v1 = obj->GetOrientation() * v1; v1 = v1 + obj->GetPosition();
            v2 = obj->GetOrientation() * v2; v2 = v2 + obj->GetPosition();
            v3 = obj->GetOrientation() * v3; v3 = v3 + obj->GetPosition();
            if(useItomp)
            {
                Vect3ToArray(p1, itompTransform * v1); Vect3ToArray(p2, itompTransform * v2);
                Vect3ToArray(p3, itompTransform * v3);
            }
            else
            {
                Vect3ToArray(p1, v1); Vect3ToArray(p2, v2);
                Vect3ToArray(p3, v3);
            }
            dsDrawLineD(p1, p2);
            dsDrawLineD(p2, p3);
            dsDrawLineD(p3, p1);
        }
    }
    void LineBetweenObjects(const planner::Object* a, const planner::Object* b)
    {
        PQP_REAL p1 [3];
        PQP_REAL p2 [3];
        Vect3ToArray(p1, itompTransform * a->GetPosition());
        Vect3ToArray(p2, itompTransform * b->GetPosition());
        dsDrawLineD(p1, p2);
    }
    void DrawObjects()

    {
        dsSetColorAlpha(0,0, 0,1);
        for(planner::Object::T_Object::iterator it = cScenario->scenario->objects_.begin();
            it != cScenario->scenario->objects_.end();
            ++it)
        {
            DrawObject(*it);
        }
        if(pathOn)
        {
            dsSetColorAlpha(0,0, 0,1);
            bool afterfirst = false;
            planner::Object::CT_Object::iterator it = cScenario->path.begin();
            for(planner::Object::CT_Object::iterator it2 = cScenario->path.begin();
                it2 != cScenario->path.end();
                ++it2)
            {
                dsSetColorAlpha(1,0, 0,1);
                if(drawObject) DrawObject(*it2);
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
            for(planner::Object::T_Object::const_iterator it = cScenario->scenario->prm->GetPRMNodes().begin();
                it != cScenario->scenario->prm->GetPRMNodes().end();
                ++it, ++i)
            {
                dsSetColorAlpha(1,0, 0,1);
                if(drawObject) DrawObject(*it);
                const std::vector< int > connexions = cScenario->scenario->prm->GetConnections(i);
                dsSetColorAlpha(0,1, 0,1);
                for(unsigned int j = 0; j< connexions.size(); ++j)
                {
                    LineBetweenObjects(*it, cScenario->scenario->prm->GetPRMNodes()[connexions[j]]);
                }
            }
        }
    }

    void DrawNode(const planner::Node* node)
    {
        if(node->current)
            DrawObject(node->current, true);
        for(std::vector<planner::Node*>::const_iterator cit = node->children.begin();
            cit != node->children.end(); ++cit)
        {
            DrawNode(*cit);
        }
    }
}
static void simLoop (int pause)
{
    DrawObjects();
    dsSetColorAlpha(0,0, 0.7,0.7);
    DrawNode(cScenario->robot->node);
    if(drawPOstures)
    {
		for(std::vector<planner::Node*>::iterator it = postures.begin();
            it != postures.end(); ++ it)
        {
            DrawNode(*it);
        }
    }
}
void start()
{
    cScenario = planner::CompleteScenarioFromFile("../humandes/fullscenarios/fullscenario.scen");
    std::cout << "done" << std::endl;
    dsSetViewpoint (xyz,hpr);
    itompTransform =Eigen::Matrix3d::Identity();
    for(int i =0; i<3; ++i)
    {
        itompTransform(i,i)=1;
    }
    itompTransform *= AngleAxisd(0.5*M_PI, Vector3d::UnitX()).matrix();
    cScenario->robot->SetConstantRotation(AngleAxisd(-0.5*M_PI, Vector3d::UnitX()) * AngleAxisd(-0.5*M_PI, Vector3d::UnitZ()).matrix());
    arm = planner::GetChild(cScenario->robot, "upper_right_arm_z_joint");

    for(int i=0; i< cScenario->limbSamples[0].size(); ++i)
    {
        planner::Node* node = new planner::Node(*arm);
        node->offset = Eigen::Vector3d(0,0,0);
        node->position = Eigen::Vector3d(0,0,0);
        planner::sampling::LoadSample(*cScenario->limbSamples[0][i], node);
        node->Update();
        postures.push_back(node);
    }
    std::cout << "done creating nodes " << path.size() << std::endl;
}
void command(int cmd)   /**  key control function; */
{
    std::cout << "path size " << path.size() << std::endl;
    switch (cmd)
    {
        case 'e' :
            pathOn = !pathOn;
        break;
        case 'p' :
            drawObject = !drawObject;
        break;
        case 's' :
            planner::SavePrm(*(cScenario->scenario->prm), outpath);
        break;
        case 'b' :
            cScenario->SavePath(outfilename);
        break;
        case '+' :
        {
            current ++; if(cScenario->path.size() <= current) current = cScenario->path.size()-1;
            cScenario->robot->SetConfiguration(cScenario->path[current]);
            break;
        }
        case '-' :
        {
            current--; if(current <0) current = 0;
            cScenario->robot->SetConfiguration(cScenario->path[current]);
            break;
        }
        case 'r' :
        {
            currentSample ++; if(cScenario->limbSamples[0].size() <= currentSample) currentSample = cScenario->limbSamples[0].size()-1;
            planner::sampling::LoadSample(*(cScenario->limbSamples[0][currentSample]),arm);
            break;
        }
        break;
        case 't' :
        {
            currentSample --; if(currentSample < 0) currentSample = 0;
            planner::sampling::LoadSample(*(cScenario->limbSamples[0][currentSample]),arm);
            break;
        }
        case 'm' :
        {
            drawPOstures = ! drawPOstures;
            break;
        }
        case 'y' :
        planner::GetChild(cScenario->robot, "upper_left_arm_x_joint")->SetRotation(planner::GetChild(cScenario->robot,"upper_left_arm_x_joint")->value-0.1);
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
    delete cScenario;
    return 0;
}
