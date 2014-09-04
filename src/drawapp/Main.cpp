#include "collision/ParserObj.h"
#include "collision/Object.h"
#include "collision/Collider.h"
#include "tools/MatrixDefs.h"
#include <drawstuff/drawstuff.h> // The drawing library for ODE;
#include "prm/SimplePRM.h"
#include "prm/Scenario.h"
#include "prmpath/Robot.h"
#include <string>
#include <iostream>
#include <fstream>
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
    Eigen::Matrix3d itompTransform;
    planner::Robot* robot = 0;
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
    void DrawObject(const planner::Object* obj, bool useItomp = true)
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
    void WriteNodeLine(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& position, std::stringstream& outstream)
    {
        for(int i=0; i<3; ++i)
        {
            for(int j=0; j<3; ++j)
            {
                outstream << rotation(i,j) << " ";
            }
            outstream << position(i) << " ";
        }
        outstream << std::endl;
    }
    Eigen::Matrix4d readNodeLine(const std::string& line)
    {
        Eigen::Matrix4d transform;
        char c11[255],c12[255],c13[255];
        char c21[255],c22[255],c23[255];
        char c31[255],c32[255],c33[255];
        char x[255],y[255],z[255];
        sscanf(line.c_str(),"%s %s %s %s %s %s %s %s %s %s %s %s",
               c11, c12, c13, x, c21, c22, c23, y, c31, c32, c33, z);
        transform << strtod (c11, NULL), strtod (c12, NULL), strtod (c13, NULL), strtod (x, NULL),
                strtod (c21, NULL), strtod (c22, NULL), strtod (c23, NULL), strtod (y, NULL),
                strtod (c31, NULL), strtod (c32, NULL), strtod (c33, NULL), strtod (z, NULL),
                0, 0, 0, 1;
        return transform;
    }
    bool SavePath()
    {
        std::string outfilename ("../tests/entrance.path");
        std::stringstream outstream;
        outstream << "size " << (int)(path.size()) << std::endl;
        for(std::vector<const planner::Object*>::const_iterator it = path.begin();
            it!= path.end(); ++it)
        {
            WriteNodeLine((*it)->GetOrientation(),(*it)->GetPosition(), outstream);
        }
        ofstream outfile;
        outfile.open(outfilename.c_str());
        if (outfile.is_open())
        {
            outfile << outstream.rdbuf();
            outfile.close();
            return true;
        }
        else
        {
            std::cout << "Can not open outfile " << outfilename << std::endl;
            return false;
        }
    }
    std::vector<Eigen::Matrix4d> LoadPath(const std::string& filename)
    {
        std::vector<Eigen::Matrix4d> res;
        ifstream myfile (filename);
        std::string line;
        int size = -1;
        if (myfile.is_open())
        {
            while (myfile.good())
            {
                getline(myfile, line);
                if(line.size()==0) break;
                else if(line.find("size ") == 0)
                {
                    line = line.substr(5);
                    size = std::stoi (line);
                }
                else
                {
                    res.push_back(readNodeLine(line));
                }
            }
            myfile.close();
        }
        else
        {
            std::cout << "file not found" << filename << std::endl;
        }
        return res;
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
    //drawManager.Draw();
    DrawObjects();
    dsSetColorAlpha(0,0, 0.7,1);
    DrawNode(robot->node);
}
void start()
{
    //dsSetViewpoint (xyz,hpr);
    //scenario = new planner::Scenario("../tests/testscenario.txt");
    //scenario = new planner::Scenario("../tests/testscenarioload.txt");
    //scenario = new planner::Scenario("../tests/evac.txt");
    scenario = new planner::Scenario("../tests/zombi.txt");
    std::cout << "done" << std::endl;
    planner::Object* from = new planner::Object(*(scenario->prm->GetPRMNodes()[0]));
    from->SetOrientation(Eigen::Matrix3d::Identity());
    planner::Object* to = new planner::Object(*from);
    Eigen::Vector3d fromd(-17.2, 0.4, -8.7);
    Eigen::Vector3d tod(-13, 0.4, -8.7);
    from->SetPosition(fromd);
    to->SetPosition(tod);
    path = scenario->prm->GetPath(*from,*to, 10.f, true);
    if(path.empty())
    {
        path.push_back(from);
        path.push_back(to);
    }
    dsSetViewpoint (xyz,hpr);
    itompTransform =Eigen::Matrix3d::Identity();
    for(int i =0; i<3; ++i)
    {
        itompTransform(i,i)=1;
    }
    itompTransform *= AngleAxisd(0.5*M_PI, Vector3d::UnitX()).matrix();
    std::string outfilename ("../tests/entrance.path");
    std::vector<Eigen::Matrix4d> res = LoadPath(outfilename);
    for(std::vector<Eigen::Matrix4d>::const_iterator it = res.begin();
        it != res.end(); ++it)
    {
        for(int i = 0; i< 4; ++i)
        {
            for(int j = 0; j<4; ++j)
            {
                std::cout << (*it)(i,j) << " ";
            }
        }
        std::cout << std::endl;
    }
    planner::Node* root = planner::LoadRobot("../humandes/human.urdf");
    robot = new planner::Robot(root);
    //robot->SetConfiguration(path[3]);
    robot->SetRotation(path[3]->GetOrientation());
    //robot->SetTranslation(path[1]->GetPosition());
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
        case 'b' :
            SavePath();
        break;
        case '+' :
        robot->Translate(Eigen::Vector3d(0.1,0,0));
        break;
        case 'r' :
        planner::GetChild(robot, "torso_z_joint")->SetRotation(planner::GetChild(robot,"torso_z_joint")->value-0.1);
        break;
        case 't' :
        planner::GetChild(robot, "torso_y_joint")->SetRotation(planner::GetChild(robot,"torso_y_joint")->value-0.1);
        break;
        case 'y' :
        planner::GetChild(robot, "torso_x_joint")->SetRotation(planner::GetChild(robot,"torso_x_joint")->value-0.1);
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
