/**
* \file SimplePRM.cpp
* \brief A PRM concrete implementation
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/

#include "SimplePRM.h"
#include "collision/ParserObj.h"
#include "planner/PRM.h" // Template PRM definition

#include "LocalPlanner.h"
#include "Generator.h"

#include <fstream>
#include <iostream>
#include <string>

namespace planner
{
    float Distance(const Object* obj1, const Object* obj2)
    {
        // TODO include angles
        const Eigen::Vector3d& a = obj1->GetPosition();
        const Eigen::Vector3d& b = obj2->GetPosition();
        float p = (float)(sqrt((b.x() - a.x()) * (b.x() - a.x()) + (b.y() - a.y()) * (b.y() - a.y()) + (b.z() - a.z()) * (b.z() - a.z())));

		const Eigen::Vector3d& ea = obj1->GetOrientation().eulerAngles(0, 1, 2);
        const Eigen::Vector3d& eb = obj2->GetOrientation().eulerAngles(0, 1, 2);
        float q = (float)(sqrt((eb.x() - ea.x()) * (eb.x() - ea.x()) + (eb.y() - ea.y()) * (eb.y() - ea.y()) + (eb.z() - ea.z()) * (eb.z() - ea.z())));
		return 0.8f * p + 0.2f * q;
		//return p + q;
    }
	
    typedef PRM<Object, planner::Generator, LocalPlanner, float, 10000> prm_t;

	struct PImpl
	{
        PImpl(const Model& model, Object::T_Object& objects, float neighbourDistance, int size, int k, bool visibility)
            : planner_(objects, model)
        {
            Generator* gen = new Generator(objects, model); // TODO MEME
            prm_ = new prm_t(gen, &planner_, Distance, neighbourDistance, size, k, visibility);
            InitPrmNodes();
            // feel prmNodes
        }

        PImpl(const Model& model, Object::T_Object& objects, int size)
            : planner_(objects, model)
        {
           prm_ = new prm_t(size);
        }

		~PImpl()
		{
			delete prm_;
		}

        void InitPrmNodes()
        {
            for(int i =0; i< prm_->currentIndex_ +1; ++i)
            {
                prmNodes_.push_back(prm_->nodeContents_[i]);
            }
        }

        Object* operator()()
		{
            return 0;
		}

        prm_t* prm_;
        LocalPlanner planner_;
        Object::T_Object prmNodes_;

	};
}

using namespace planner;

SimplePRM::SimplePRM(const Model &model, Object::T_Object &objects, float neighbourDistance, int size, int k, bool visibility)
    : model_(model)
    , objects_(objects)
{
    pImpl_.reset(new PImpl(model_, objects_, neighbourDistance, size, k, visibility));
}

SimplePRM::SimplePRM(const Model& model, Object::T_Object &objects, int size)
    : model_(model)
    , objects_(objects)
{
    pImpl_.reset(new PImpl(model_, objects_, size));
}

SimplePRM::~SimplePRM()
{
	// NOTHING
}

const Object::T_Object& SimplePRM::GetPRMNodes() const
{
    return pImpl_->prmNodes_;
}


const std::vector<int> &SimplePRM::GetConnections(int node) const
{
    return pImpl_->prm_->edges_[node];
}

namespace
{
typedef std::list<const Object*> L_Object;

void Simplify(LocalPlanner& planner, int currentIndex, Object::CT_Object& res)
{
    if(currentIndex >= res.size()) return;
    const Object* obj = res[currentIndex];
    bool erased = false;
    for(int i = res.size()-1; i> currentIndex + 1 && ! erased; --i)
    {
        if(planner(obj, res[i], 1))
        {
            erased = true;
            Object::CT_Object::iterator from, to;
            int j=0;
            for(Object::CT_Object::iterator eraseit = res.begin(); j<= i; ++j)
            {
                if(j == currentIndex + 1) from = eraseit;
                else if(j == i) to = eraseit;
            }
            res.erase(from, to);
        }
    }
    if(erased)
    {
        Simplify(planner, currentIndex, res);
    }
    else
    {
        Simplify(planner, currentIndex+1, res);
    }
}
}

Object::CT_Object SimplePRM::GetPath(const Object &from, const Object &to, float neighbourDistance, bool simplify)
{
    Object::CT_Object res = pImpl_->prm_->ComputePath(&from, &to, Distance, &pImpl_->planner_, neighbourDistance);
    if(simplify && !res.empty())
    {
        ::L_Object reverseres;
        for(Object::CT_Object::const_iterator it = res.begin(); it!=res.end(); ++it)
        {
            reverseres.push_front(*it);
        }
        Simplify(pImpl_->planner_, 0, res);
    }
    return res;
}

std::vector<Eigen::Matrix4d> SimplePRM::Interpolate(const Object::CT_Object& path, int steps)
{
    std::vector<Eigen::Matrix4d> res, tmp;
	int each = steps/(int)path.size();
	Object::CT_Object::const_iterator it2= path.begin(); ++it2;
	for(Object::CT_Object::const_iterator it= path.begin(); it2!= path.end(); ++it, ++it2)
	{
		tmp = pImpl_->planner_.Interpolate(*it, *it2, each);
		
		for(std::vector<Eigen::Matrix4d>::const_iterator cit = tmp.begin(); cit!=tmp.end(); ++cit)
		{
			res.push_back(*cit);
		}
	}
	return res;
}

using namespace std;

namespace
{
    int getLastPointIndex(const string& s)
    {
        int res = -1;
        for(unsigned int i=0;i<s.size();i++)
        {
            if(s[i]=='.')
                res = i;
        }
        return res;
    }

    string insertEndFilename(const string& s, const std::string& ext)
    {
        //Remplace les ',' par des espaces.
        string ret= s.substr(0, getLastPointIndex(s));
        ret += ext;
        ret += s.substr(getLastPointIndex(s),  string::npos);
        return ret;
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


bool planner::SavePrm(SimplePRM& prm, const std::string& outfilename)
{
    size_t size = prm.GetPRMNodes().size();
    std::stringstream outstream;
    outstream << "size " << (int)(size) << std::endl;
    for(std::vector<Object*>::const_iterator it = prm.GetPRMNodes().begin();
        it!= prm.GetPRMNodes().end(); ++it)
    {
        WriteNodeLine((*it)->GetOrientation(),(*it)->GetPosition(), outstream);
    }
    outstream << "connections " << std::endl;
    for(size_t i =0; i< size; ++i)
    {
        for(std::vector<int>::const_iterator it = prm.GetConnections((int)i).begin();
            it!= prm.GetConnections((int)i).end(); ++it)
        {
            if(*it > (int)i)
            {
                outstream << i << " " << (*it) << std::endl;
            }
        }
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

SimplePRM* planner::LoadPRM(const std::string& filename, Object::T_Object& objects, const Model& model)
{
    SimplePRM* prm(0);
    const Object& target = *model.englobed;
    ifstream myfile (filename);
    string line;
    int size = -1;
    if (myfile.is_open())
    {
        Eigen::Matrix4d tmp;
        bool inConnexions = false;
        while (myfile.good())
        {
            getline(myfile, line);
            if(line.size()==0) break;
            else if(line.find("size ") == 0)
            {
                line = line.substr(5);
                size = std::stoi (line);
                prm = new SimplePRM(model, objects, size);
            }
            else if(line.find("connections ") == 0)
            {
                inConnexions = true;
            }
            else if(inConnexions)
            {
                int from, to;
                std::string::size_type sz;     // alias of size_t
                from = std::stoi (line,&sz);
                to = std::stoi (line.substr(sz));
                prm->pImpl_->prm_->AddEdge(from, to);
            }
            else
            {
                tmp = readNodeLine(line);
                Object* obj = new Object(target);
                obj->SetOrientation(tmp.block<3,3>(0,0));
                obj->SetPosition(tmp.block<3,1>(0,3));
                prm->pImpl_->prm_->AddNode(obj);
            }
        }
        myfile.close();
    }
    else
    {
        std::cout << "file not found" << filename << std::endl;
    }
    prm->pImpl_->InitPrmNodes();
    return prm;
}
