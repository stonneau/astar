#include "CompleteScenario.h"

#include <iostream>
#include <sstream>
#include <fstream>

using namespace planner;

using namespace std;

CompleteScenario::CompleteScenario()
    : scenario(0)
    , robot(0)
    , from(0)
    , to(0)
{
    // NOTHING
}

CompleteScenario::~CompleteScenario()
{
    delete scenario;
    delete robot;
    delete from;
    delete to;
    for(std::vector<planner::Robot*>::iterator it = completePath.begin();
        it != completePath.end(); ++it)
    {
        delete *it;
    }
}

namespace
{
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
}

bool CompleteScenario::SavePath(const std::string& outfilename)
{
    //std::string outfilename ("../tests/entrance.path");
    std::stringstream outstream;
    outstream << "size " << (int)(path.size()) << std::endl;
    for(Object::CT_Object::const_iterator it = path.begin();
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

namespace
{
 /*   File description:
    PRMSCENARIO file="path/to/scenario"
    ROBOT_SKELETON file="path/to/urdf"
    ROBOT_CONSTRAINTS file="path/to/joint_constraints"
    PATH_FROM matrix="a00 a01 a02 a03 ... a30 a31 a32 a33"
    PATH_TO matrix="a00 a01 a02 a03 ... a30 a31 a32 a33"
    LIMB joint_name="joint_name"
    ...
    LIMB joint_name="joint_name"
    NBSAMPLES N=""*/

    void PrintError(const std::string& item, const std::string& filename)
    {
        std::cout << "Missing description for item " << item << "in file" << filename << std::endl;
    }

    std::string ExtractQuotes(const std::string& line)
    {
        int quoteStart = line.find("\"");
        int quoteEnd = line.find("\"", quoteStart+1);
        return line.substr(quoteStart+1, quoteEnd - quoteStart -1);
    }

    std::vector<std::string> splitSpace(const std::string& s)
    {
        //Eclate une chane au niveau de ses espaces.
        std::vector<std::string> ret;
        std::string s1="";
        for(unsigned int i=0;i<s.size();i++)
        {
            if(s[i]==' '||i==s.size()-1)
            {
                if(i==s.size()-1)
                    s1+=s[i];
                ret.push_back(s1);
                s1="";
            }
            else
                s1+=s[i];
        }
        return ret;
    }

    Eigen::Matrix4d ReadTransform(const std::string& line, bool& error)
    {
        Eigen::Matrix4d res;
        std::vector<std::string> matrix = splitSpace(line);
        if(matrix.size() != 16)
        {
            error = true;
            std::cout << "matrix does not contain 16 members" << std::endl;
        }
        for(int i =0; i<16; ++i)
        {
            char value [10];
            sscanf(matrix[i].c_str(),"%s", value);
            res(i/4, i%4) = strtod (value, NULL);
        }
        return res;
    }

}

CompleteScenario* planner::CompleteScenarioFromFile(const std::string& filename)
{
    CompleteScenario* cScenario = new CompleteScenario;
    bool scenario = false; bool skeleton = false; bool contraints = false;
    bool from = false; bool to = false; bool limb = false;
    bool samples = false;
    std::ifstream myfile (filename);
    if (myfile.is_open())
    {
        std::string line;
        while (myfile.good())
        {
            getline(myfile, line);
            if(line.find("PRMSCENARIO file=") != string::npos)
            {
                cScenario->scenario = new planner::Scenario(ExtractQuotes(line));
                if(cScenario->scenario )
                {
                    scenario = true;
                }
            }
            if(line.find("ROBOT_SKELETON file=") != string::npos)
            {
                cScenario->robot = new planner::Robot(planner::LoadRobot(ExtractQuotes(line)));
                if(cScenario->robot )
                {
                    skeleton = true;
                }
            }
            if(line.find("ROBOT_CONSTRAINTS file=") != string::npos)
            {
                if(planner::LoadJointConstraints(*cScenario->robot, ExtractQuotes(line)))
                {
                    contraints = true;
                }
            }
            if(line.find("PATH_FROM matrix=") != string::npos && scenario)
            {
                Eigen::Matrix4d res = ReadTransform(ExtractQuotes(line), from);
                cScenario->from = new Object(*(cScenario->scenario->model_.englobed));
                cScenario->from->SetOrientation(res.block<3,3>(0,0));
                cScenario->from->SetPosition(res.block<3,1>(0,3));
                from = !from;
            }
            if(line.find("PATH_TO matrix=") != string::npos && scenario)
            {
                Eigen::Matrix4d res = ReadTransform(ExtractQuotes(line), to);
                cScenario->to = new Object(*(cScenario->scenario->model_.englobed));
                cScenario->to->SetOrientation(res.block<3,3>(0,0));
                cScenario->to->SetPosition(res.block<3,1>(0,3));
                to = !to;
            }
            if(line.find("LIMB") != string::npos && cScenario->robot)
            {
                Node* res = planner::GetChild(cScenario->robot, ExtractQuotes(line));
                if(res)
                {
                    limb = true;
                    cScenario->limbs.push_back(res);
                }
                else
                {
                    std::cout << "no joint named " << ExtractQuotes(line) << std::endl;
                    break;
                }
            }
            if(line.find("NBSAMPLES")!= string::npos && limb)
            {
                char data[30];
                sscanf(ExtractQuotes(line).c_str(),"%s", data);
                int nb_samples = strtod(data, NULL);
                for(std::vector<Node*>::iterator it = cScenario->limbs.begin()
                    ; it!= cScenario->limbs.end(); ++it)
                {
                    cScenario->limbSamples.push_back(planner::sampling::GenerateSamples(*cScenario->robot, *it, nb_samples));
                }
                samples = true;
            }
        }
        myfile.close();
    }
    else
    {
        std::cout << "can not found complete scenario file" << filename << std::endl;
    }
    if(scenario && skeleton && contraints && to && from && limb && samples )
    {
        // Generate path
        cScenario->path = cScenario->scenario->prm->GetPath(*(cScenario->from), *(cScenario->to), 10, true);
        if(cScenario->path.empty())
        {
            cScenario->path.push_back(cScenario->from);
            cScenario->path.push_back(cScenario->to);
        }
        return cScenario;
    }
    else
    {
        if(!scenario)
        {
            PrintError("PRMSCENARIO", filename);
        }
        if(!skeleton)
        {
            PrintError("ROBOT_SKELETON", filename);
        }
        if(!contraints)
        {
            PrintError("ROBOT_CONSTRAINTS", filename);
        }
        if(!to)
        {
            PrintError("PATH_FROM", filename);
        }
        if(!from)
        {
            PrintError("PATH_TO", filename);
        }
        if(!limb)
        {
            PrintError("LIMB", filename);
        }
        if(!samples)
        {
            PrintError("NBSAMPLES", filename);
        }
        return cScenario;
    }
}
