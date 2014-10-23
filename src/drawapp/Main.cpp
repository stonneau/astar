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
#include "prmpath/CompleteScenario.h"
#include "prmpath/PostureSelection.h"
#include "prmpath/Export/BVHExporter.h"
#include "prmpath/Export/ITOMPExporter.h"
#include "prmpath/ik/IKSolver.h"
#include "Timer.h"

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
    bool pathOn = true;
    bool drawObject = false;
    bool drawacis = false;
    bool drawPOstures = true;
    bool drawScene = false;
    std::string outpath("../tests/testSerialization.txt");
    std::string outfilename ("../tests/entrance.path");
    Eigen::Matrix3d itompTransform;
    int current = 0;
    int currentSample = 0;
    planner::Node * arm = 0;
    planner::CompleteScenario* cScenario = 0;
    std::vector<planner::Node*> postures;
    planner::sampling::T_Samples samples;
    planner::T_State states;
    ik::IKSolver ikSolver;
    Eigen::Vector3d target = Eigen::Vector3d(0.1,-0.1, 0.2);
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
    void DrawModel(const planner::Model* model, bool useItomp = true)
    {
        DrawObject(model->englobed, useItomp);
        for(planner::Object::T_Object::const_iterator it3 = model->englobing.begin();
            it3 != model->englobing.end(); ++it3)
        {
            DrawObject(*it3, useItomp);
        }
    }

    void DrawPoint(const Eigen::Vector3d& target)
    {
        dsSetColor(1,0,0);
        PQP_REAL p1 [3];
        PQP_REAL p2 [3];
        Eigen::Vector3d tmp(target);
        tmp(2)+=0.1;
        Vect3ToArray(p1,tmp);
        tmp(2)-=0.2;
        Vect3ToArray(p2,tmp);
        dsDrawLineD(p1, p2);

        tmp(2)+=0.1;
        tmp(1)+=0.1;
        Vect3ToArray(p1,tmp);
        tmp(1)-=0.2;
        Vect3ToArray(p2,tmp);
        dsDrawLineD(p1, p2);
        dsSetColor(0,0,1);
    }

    void LineBetweenObjects(const planner::Model* a, const planner::Model* b)
    {
        PQP_REAL p1 [3];
        PQP_REAL p2 [3];
        Vect3ToArray(p1, itompTransform * a->GetPosition());
        Vect3ToArray(p2, itompTransform * b->GetPosition());
        dsDrawLineD(p1, p2);
    }
    void DrawObjects()

    {
        if(drawScene)
        {
            dsSetColorAlpha(0,0, 0,1);
            for(planner::Object::T_Object::iterator it = cScenario->scenario->objects_.begin();
                it != cScenario->scenario->objects_.end();
                ++it)
            {
                DrawObject(*it);
            }

            for(planner::Object::T_Object::iterator it = cScenario->scenario->contactObjects_.begin();
                it != cScenario->scenario->contactObjects_.end();
                ++it)
            {
                DrawObject(*it);
            }
        }
        if(pathOn)
        {
            dsSetColorAlpha(0,0, 0,1);
            bool afterfirst = false;
            planner::CT_Model::iterator it = cScenario->path.begin();
            for(planner::CT_Model::iterator it2 = cScenario->path.begin();
                it2 != cScenario->path.end();
                ++it2)
            {
                dsSetColorAlpha(1,0, 0,1);
                if(drawObject)
                {
                    DrawModel(*it2);
                }
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
            for(planner::T_Model::const_iterator it = cScenario->scenario->prm->GetPRMNodes().begin();
                it != cScenario->scenario->prm->GetPRMNodes().end();
                ++it, ++i)
            {
                dsSetColorAlpha(1,0, 0,1);
                if(drawObject) DrawModel(*it);
                const std::vector< int > connexions = cScenario->scenario->prm->GetConnections(i);
                dsSetColorAlpha(0,1, 0,1);
                for(unsigned int j = 0; j< connexions.size(); ++j)
                {
                    LineBetweenObjects(*it, cScenario->scenario->prm->GetPRMNodes()[connexions[j]]);
                }
            }
        }
    }

    void DrawAcis(const planner::Node* node)
    {
        Eigen::Matrix3d res = itompTransform * node->toWorldRotation;
        PQP_REAL p1 [3];
        Vect3ToArray(p1, itompTransform *  node->position);
        PQP_REAL p2 [3];
        dsSetColor(1,0,0);
        Vect3ToArray(p2, itompTransform *  node->position + res * Eigen::Vector3d(1,0,0));
        dsDrawLineD(p1, p2);
        dsSetColor(0,1,0);
        Vect3ToArray(p2, itompTransform *  node->position +  res * Eigen::Vector3d(0,1,0));
        dsDrawLineD(p1, p2);
        dsSetColor(0,0,1);
        Vect3ToArray(p2, itompTransform *  node->position + res * Eigen::Vector3d(0,0,1));
        dsDrawLineD(p1, p2);
    }

    void DrawNode(const planner::Node* node)
    {
        if(node->current)
            DrawObject(node->current, true);
        if (drawacis) DrawAcis(node);
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
    for(std::vector<Eigen::Vector3d>::iterator it = states[current]->contactLimbPositions.begin();
        it != states[current]->contactLimbPositions.end(); ++it)
    {
        DrawPoint(itompTransform * (*it));
    }
    if(drawPOstures)
    {
		for(std::vector<planner::Node*>::iterator it = postures.begin();
            it != postures.end(); ++ it)
        {
            DrawNode((*it));
        }
        /*for(planner::T_State::iterator it = states.begin();
            it != states.end(); ++ it)
        {
            DrawNode((*it)->value->node);
        }*/
    }
    //Eigen::Vector3d target = planner::GetChild(cScenario->robot, "torso_x_joint")->toWorldRotation* Eigen::Vector3d(0.3,0,0);
    DrawPoint(itompTransform*target);
}
void start()
{
    cScenario = planner::CompleteScenarioFromFile("../humandes/fullscenarios/tunnel.scen");
    //cScenario = planner::CompleteScenarioFromFile("../humandes/fullscenarios/climbing.scen");
    //cScenario = planner::CompleteScenarioFromFile("../humandes/fullscenarios/zoey.scen");
    //cScenario = planner::CompleteScenarioFromFile("../tests/profile.scen");
    std::cout << "done" << std::endl;
    dsSetViewpoint (xyz,hpr);
    itompTransform =Eigen::Matrix3d::Identity();
    for(int i =0; i<3; ++i)
    {
        itompTransform(i,i)=1;
    }
    std::cout << "ca donne" << cScenario->robot->constantRotation << std::endl;

    Eigen::Matrix3d daf = AngleAxisd(-0.5*M_PI, Vector3d::UnitX()) * AngleAxisd(-0.5*M_PI, Vector3d::UnitZ()).matrix();
    std::cout << "la transform" << std::endl << daf << std::endl;
    itompTransform *= AngleAxisd(0.5*M_PI, Vector3d::UnitX()).matrix();
    //cScenario->robot->SetConstantRotation(AngleAxisd(-0.5*M_PI, Vector3d::UnitX()) * AngleAxisd(-0.5*M_PI, Vector3d::UnitZ()).matrix());
    arm = planner::GetChild(cScenario->robot, "upper_right_arm_z_joint");


    std::cout << "ca EN VRAI" << cScenario->robot->constantRotation << std::endl;

    /*for(int i=0; i< cScenario->limbSamples[0].size(); ++i)
    {
        planner::Node* node = new planner::Node(*cScenario->robot->node);
        //node->offset = Eigen::Vector3d(0,0,0);
        //node->position = Eigen::Vector3d(0,0,0);
        planner::sampling::LoadSample(*cScenario->limbSamples[0][i], planner::GetChild(node, "upper_right_arm_z_joint"));
        node->Update();
        postures.push_back(node);
    }*/
    samples = cScenario->limbSamples[0];
    std::cout << " SAMPLES" << samples.size() << std::endl;
    std::cout << "done creating nodes " << path.size() << std::endl;
    states = planner::PostureSequence(*cScenario);
    for(int i = 0; i< states.size(); ++i)
    {
        std::cout << "state : " << i << std::endl;
        for(int w = 0; w< states[i]->contactLimbs.size(); ++w)
        {
            std::cout << "   contact : " << states[i]->contactLimbs[w] << std::endl;
        }
    }

    /*Export to bvh*/
    exporter::BVHExporter exporter;
    exporter.PushStructure(cScenario->robot);
    for(planner::T_State::iterator it = states.begin(); it != states.end(); ++it)
    {
        exporter.PushFrame((*it)->value, false);
    }
    std::string savebvh("../tests/test.bvh");
    exporter.Save(savebvh);

    /*Export to ITOMP*/

    Eigen::Matrix3d inverse = itompTransform;
    inverse.inverse();
    exporter::ITOMPExporter itompexporter(inverse, Eigen::Vector3d(12,-8,-0.5));
    itompexporter.PushStructure(cScenario->robot);
    for(planner::T_State::iterator it = states.begin()+1; it != states.end(); ++it)
    {
        itompexporter.PushFrame((*it)->value, false);
    }
    savebvh ="../tests/test.path";
    itompexporter.Save(savebvh);
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

bool SavePath()
{
    std::stringstream outstream;
    outstream << "size " << (int)(path.size()) << std::endl;
    /*for(Object::CT_Object::const_iterator it = path.begin();
        it!= path.end(); ++it)
    {
        WriteNodeLine((*it)->GetOrientation(),(*it)->GetPosition(), outstream);
    }*/
    planner::Robot* tmp(0) ;
    for(int i = 0; i< states.size(); ++i)
    {
        planner::Robot* tmp =states[i]->value;
        Eigen::Matrix3d wtf = itompTransform;
        Eigen::Vector3d putain(0,0,0);
        wtf.inverse();
        WriteNodeLine(wtf * tmp->currentRotation, wtf * tmp->currentPosition - putain, outstream);
    }
    ofstream outfile;
    outfile.open(outfilename.c_str());
    if (outfile.is_open())
    {
        outfile << outstream.rdbuf();
        outfile.close();
        //return true;
    }
    else
    {
        std::cout << "Can not open outfile " << outfilename << std::endl;
        //return false;
    }

    /*Export to bvh*/
    exporter::BVHExporter exporter;
    exporter.PushStructure(cScenario->robot);
    for(planner::T_State::iterator it = states.begin(); it != states.end(); ++it)
    {
        exporter.PushFrame((*it)->value, false);
    }
    std::string savebvh("../tests/test.bvh");
    exporter.Save(savebvh);

    /*Export to ITOMP*/

    Eigen::Matrix3d inverse = itompTransform;
    inverse.inverse();
    exporter::ITOMPExporter itompexporter(inverse, Eigen::Vector3d(0,0,0.472064));
    itompexporter.PushStructure(cScenario->robot);
    for(planner::T_State::iterator it = states.begin()+1; it != states.end(); ++it)
    {
        (*it)->value->node->Update();
        itompexporter.PushFrame((*it)->value, false);
    }
    savebvh ="../tests/itomp.path";
    itompexporter.Save(savebvh);
	return true;
}

void command(int cmd)   /**  key control function; */
{
    //Eigen::Vector3d target = planner::GetChild(cScenario->robot, "torso_x_joint")->toWorldRotation* Eigen::Vector3d(0.5,0.5,0.5);
    switch (cmd)
    {
        case 't' :
            pathOn = !pathOn;
        break;
        case '3' :
            drawScene = !drawScene;
        break;
        case '4' :
        {
            ikSolver.StepClamping(planner::GetChild(cScenario->robot, "upper_right_arm_z_joint"),target,target ,false);
        }

        break;
        case 'q' :
            drawacis = !drawacis;
        break;
        case 'y' :
            drawObject = !drawObject;
        break;
        case 's' :
            planner::SavePrm(*(cScenario->scenario->prm), outpath);//, (AngleAxisd(M_PI, Vector3d::UnitZ()) * AngleAxisd(0.5*M_PI, Vector3d::UnitX())).matrix());
        break;
        case 'd' :
            SavePath();
        break;
        case '+' :
        {
            current ++; if(states.size() <= current) current = states.size()-1;
            //cScenario->robot->SetConfiguration(cScenario->path[current]);
            cScenario->robot = states[current]->value;
            //currentSample = 0;
            //samples = planner::GetPosturesInContact(*cScenario->robot, cScenario->limbs[0], cScenario->limbSamples[0], cScenario->scenario->objects_ );
            break;
        }
        case '-' :
        {
            current--; if(current <0) current = 0;
            //cScenario->robot->SetConfiguration(states[current]);
            cScenario->robot = states[current]->value;
            //currentSample = 0;
            //samples = planner::GetPosturesInContact(*cScenario->robot, cScenario->limbs[0], cScenario->limbSamples[0], cScenario->scenario->objects_ );

            break;
        }
        case 'b' :
        {
            currentSample = 0;
            samples = planner::GetPosturesOnTarget(*cScenario->robot, cScenario->limbs[0], cScenario->limbSamples[0], cScenario->scenario->objects_, planner::GetEffectors(cScenario->limbs[0])[0]->parent->position );

            break;
        }
        case '1' :
        {

        std::cout << " SAMPLES" << samples.size() << std::endl;
            if(samples.empty()) return;
            currentSample ++; if(samples.size() <= currentSample) currentSample = samples.size()-1;
            planner::sampling::LoadSample(*(samples[currentSample]),planner::GetChild(cScenario->robot, "upper_right_arm_z_joint"));
            break;
        }
        break;
        case '2' :
        {
        std::cout << " SAMPLES" << samples.size() << std::endl;
            if(samples.empty()) return;
            currentSample --; if(currentSample < 0) currentSample = 0;
            planner::sampling::LoadSample(*(samples[currentSample]),planner::GetChild(cScenario->robot, "upper_right_arm_z_joint"));
            break;
        }
        case 'm' :
        {
            drawPOstures = ! drawPOstures;
            break;
        }
        case 'a' :
        planner::GetChild(cScenario->robot, "upper_right_arm_z_joint")->SetRotation(planner::GetChild(cScenario->robot,"upper_right_arm_z_joint")->value-0.1);
        break;
        case 'z' :
        planner::GetChild(cScenario->robot, "upper_right_arm_y_joint")->SetRotation(planner::GetChild(cScenario->robot,"upper_right_arm_y_joint")->value-0.1);
        break;
        case 'e' :
        planner::GetChild(cScenario->robot, "upper_right_arm_x_joint")->SetRotation(planner::GetChild(cScenario->robot,"upper_right_arm_x_joint")->value-0.1);
        break;
        case 'r' :
        planner::GetChild(cScenario->robot, "lower_right_arm_joint")->SetRotation(planner::GetChild(cScenario->robot,"lower_right_arm_joint")->value-0.1);
        break;

        case 'u' :
        planner::GetChild(cScenario->robot, "upper_right_arm_z_joint")->SetRotation(planner::GetChild(cScenario->robot,"upper_right_arm_z_joint")->value-0.1);
        break;
        case 'i' :
        planner::GetChild(cScenario->robot, "upper_left_arm_y_joint")->SetRotation(planner::GetChild(cScenario->robot,"upper_left_arm_y_joint")->value-0.1);
        break;
        case 'o' :
        planner::GetChild(cScenario->robot, "upper_left_arm_x_joint")->SetRotation(planner::GetChild(cScenario->robot,"upper_left_arm_x_joint")->value-0.1);
        break;
        case 'p' :
        planner::GetChild(cScenario->robot, "lower_left_arm_joint")->SetRotation(planner::GetChild(cScenario->robot,"lower_left_arm_joint")->value-0.1);
        break;


        case 'w' :
        planner::GetChild(cScenario->robot, "upper_right_arm_z_joint")->SetRotation(planner::GetChild(cScenario->robot,"upper_right_arm_z_joint")->value-0.1);
        break;
        case 'x' :
        planner::GetChild(cScenario->robot, "upper_left_leg_y_joint")->SetRotation(planner::GetChild(cScenario->robot,"upper_left_leg_y_joint")->value-0.1);
        break;
        case 'c' :
        planner::GetChild(cScenario->robot, "upper_left_leg_x_joint")->SetRotation(planner::GetChild(cScenario->robot,"upper_left_leg_x_joint")->value-0.1);
        break;
        case 'v' :
        planner::GetChild(cScenario->robot, "lower_left_leg_joint")->SetRotation(planner::GetChild(cScenario->robot,"lower_left_leg_joint")->value-0.1);
        break;


        case 'h' :
        planner::GetChild(cScenario->robot, "upper_right_arm_z_joint")->SetRotation(planner::GetChild(cScenario->robot,"upper_right_arm_z_joint")->value-0.1);
        break;
        case 'j' :
        planner::GetChild(cScenario->robot, "upper_right_leg_y_joint")->SetRotation(planner::GetChild(cScenario->robot,"upper_right_leg_y_joint")->value-0.1);
        break;
        case 'k' :
        planner::GetChild(cScenario->robot, "upper_right_leg_x_joint")->SetRotation(planner::GetChild(cScenario->robot,"upper_right_leg_x_joint")->value-0.1);
        break;
        case 'l' :
        planner::GetChild(cScenario->robot, "lower_right_leg_joint")->SetRotation(planner::GetChild(cScenario->robot,"lower_right_leg_joint")->value-0.1);
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
