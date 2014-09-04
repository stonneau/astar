/**
* \file joint_io_rtql8.h
* \brief io functions to save / load joints descriptions with rtql8 file format.
* \author Steve T.
* \version 0.1
* \date 18/02/2014
*
*/

#ifndef _JOINT_IO_URDF
#define _JOINT_IO_URDF

#include "joint.h"
#include "joint_io.h"
#include "Pi.h"

#include <string>
#include <map>

using namespace std;

namespace kinematics
{
	


//! @cond
vector<string> SplitSpace(const string& s)
{
    //Eclate une chaîne au niveau de ses espaces.
    vector<string> ret;
    string s1="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(s[i]==' '|| s[i]=='\t' || i==s.size()-1)
        {
			if(i==s.size()-1)
				s1+=s[i];
            if(s1 != "")
				ret.push_back(s1);
			s1="";
        }
        else
            s1+=s[i];
    }
    return ret;
}
/// @endcond

//! @cond
string CleanLine(const string& s)
{
    //Eclate une chaîne au niveau de ses espaces.
    string s1="";
    for(unsigned int i=0;i<s.size();i++)
    {
		if(s[i]==',' || s[i]=='{' || s[i]=='}'  || s[i]==',' || s[i]=='<'|| s[i]=='>'  )
        {
            s1+= " ";
        }
        else
            s1+=s[i];
    }
    return s1;
}
/// @endcond


//! @cond
string NextNonEmptyLine(std::ifstream& myfile)
{
    //Eclate une chaîne au niveau de ses espaces.
	string s1="";
	while(myfile.good())
	{
		getline(myfile, s1);
		if(s1 != "")
			return s1;
	}
    return s1;
}
/// @endcond

//! @cond
template<typename Numeric>
std::map<string, std::vector<Numeric> > ReadDofs(std::ifstream& myfile)
{
	typedef std::map<string, std::vector<Numeric> > T_Dofs;
	T_Dofs dofs;
	string line;
	while ( myfile.good() )
	{
		line = NextNonEmptyLine(myfile);
		if(line.find("dofs {") == 0)
		{
			continue;
		}
		else if(line.find("}") == 0)
		{
			break;
		}
		else
		{
			line = CleanLine(line);
			std::vector<string> data = SplitSpace(line);
			std::vector<Numeric> dof;
			dof.push_back(strtod (data[1].c_str(), NULL)); dof.push_back(strtod (data[2].c_str(), NULL)); dof.push_back(strtod (data[3].c_str(), NULL));
			dofs.insert(std::make_pair(data[0], dof));
		}
	}
	return dofs;
}
/// @endcond

//! @cond
template<typename Numeric>
std::map<string, Numeric > ReadMasses(std::ifstream& myfile)
{
	typedef std::map<string, Numeric > T_Mass;
	T_Mass  mass;
	string line;
	while ( myfile.good() )
	{
		line = NextNonEmptyLine(myfile);
		if(line.find("mass {") == 0 || line == "")
		{
			continue;
		}
		else if(line.find("}") == 0)
		{
			break;
		}
		else
		{
			line = CleanLine(line);
			std::vector<string> data = SplitSpace(line);
			mass.insert(std::make_pair(data[0], strtod (data[1].c_str(), NULL)));
		}
	}
	return mass;
}
/// @endcond


//! @cond
template<typename Numeric, typename T>
void ReadChain(std::ifstream& myfile, const std::map<string, std::vector<Numeric> >& dofs, T& joint)
{
	typedef std::map<string, Numeric > T_Mass;
	T_Mass  mass;
	std::string line ="";
	while ( myfile.good() )
	{
		line = NextNonEmptyLine(myfile);  // chain { id // don't care for the moment
		// telescope { <0.1516,-0.5307,-0.8339>, pelvis_width }
		line = CleanLine(line);
		vector<string> data = SplitSpace(line);
		if(line.find("telescope") != string::npos)
		{
			 // i don't get the first numbers, let's just get the length for the time being
			const string& keyWord = data[4];
			std::map<string, std::vector<Numeric> >::const_iterator it = dofs.find(keyWord);
			if(it != dofs.end())
			{
				//const std::vector<Numeric>& dof = dofs.at(keyWord);
				/*for(int i = 0; i< 3; ++i)
				{
					//joint.offset[i] = it->second[i] * strtod (data[i+1].c_str(), NULL);
					Numeric f= (Numeric)(strtod (data[i+1].c_str(), NULL));
					joint.offset[i] =  f;
				}*/
				//dealing with y up stuff
				joint.offset[0] = (Numeric)(strtod (data[1].c_str(), NULL));
				joint.offset[2] = (Numeric)(strtod (data[2].c_str(), NULL));
				joint.offset[1] = (Numeric)(strtod (data[3].c_str(), NULL));
			}
		}
		// rotate_euler { l_ankle_euler_z, z }
		else if(line.find("rotate_euler")  != string::npos)
		{
			const string& keyWord = data[1];
			std::map<string, std::vector<Numeric> >::const_iterator it = dofs.find(keyWord);
			if(it != dofs.end())
			{
				int id = 0;
				if(data[2] == "y") 
					//id = 1;
					id = 2;
				if(data[2] == "z") 
					//id = 2;
					id = 1;
				joint.defaultAngleValues[id] = it->second[0] * RadiansToDegrees ;
				joint.minAngleValues[id] = it->second[1] * RadiansToDegrees;
				joint.maxAngleValues[id] = it->second[2] * RadiansToDegrees;
			}
		}
		else if(data.size() <2) // } found
		{
			return;
		}
	}
}
/// @endcond


//! @cond
template<typename Numeric, typename T>
void ReadPrimitive(std::ifstream& myfile, const std::map<string, std::vector<Numeric> >& dofs, T& joint)
{
	// I don't think i need this at the time being
	string line = "";
	bool foundOpen = false; bool foundClose = false;
	while(!(foundOpen && foundClose))
	{
		line = NextNonEmptyLine(myfile); // read primitive
		if(line.find("{") != string::npos)
			foundOpen = true;
		if(line.find("}") != string::npos)
			foundClose = true;
	}
}
/// @endcond

//! @cond
template<typename Numeric, typename T>
T* ReadNode(std::ifstream& myfile, const std::map<string, std::vector<Numeric> >& dofs, const std::string& nodeLine)
{
	typedef std::map<string, Numeric > T_Mass;
	T* joint = new T();
	T_Mass  mass;
	// get node name and id
	string line = CleanLine(nodeLine);
	std::vector<string> data = SplitSpace(line);
	sscanf(data[1].c_str(),"%s", joint->tag);
		
	ReadChain<Numeric>(myfile, dofs, *joint);
	ReadPrimitive<Numeric>(myfile, dofs, *joint);
	line = NextNonEmptyLine(myfile);
	while(myfile.good())
	{
		if(line.find("node") != string::npos)
		{
			joint->add_child(ReadNode<Numeric, T>(myfile, dofs, line));
		}
		else
		{
			break;
		}
		line = NextNonEmptyLine(myfile);
	}
	return joint;
}
/// @endcond



///  \brief Loads a kinematic tree from a given RTQL8 file.
///  If Safe is set to true, this method will throw an exception if the file can not be read.
///  \param filename the name of the file from which the description must be loaded.
template<typename Numeric, int MaxChildren, bool Safe>
joint<Numeric, Numeric, 3, MaxChildren, Safe>* ReadRURF(const std::string& filename)
{
	typedef joint<Numeric, Numeric, 3, MaxChildren, Safe> joint_t;
	typedef std::map<string, Numeric > T_Mass;
	typedef std::map<string, std::vector<Numeric> > T_Dofs;

	std::string line;
	std::ifstream myfile (filename);
	int rootId = -1; bool rootFound(false);
	std::vector<joint_t> joints(100);
    std::vector<std::vector<int> > children(100);
	T_Dofs dofs; T_Mass masses;
	joint_t * joint;
	if (myfile.is_open())
	{
		dofs = ReadDofs<Numeric>(myfile);
		masses = ReadMasses<Numeric>(myfile);
		line = NextNonEmptyLine(myfile);
		joint = ReadNode<Numeric, joint_t>(myfile, dofs, line);
	}
	else if(Safe)
	{
		std::string errMess("Unable to open file " + filename + "for reading.");
		throw std::exception(errMess.c_str());
	}
	return joint;
}

}// end namespace kinematics
#endif //_JOINT_IO_URDF
