/**
* \file joint_io.h
* \brief io functions to save / load joints descriptions with text files.
* \author Steve T.
* \version 0.1
* \date 10/12/2013
*
* A joint is saved in a text file using the following format :
* ROOT ID xxx
* OFFSET xxx xxx xxx
* MINANGLES xxx xxx xxx
* MAXANGLES xxx xxx xxx
* DEFAULTANGLES xxx xxx xxx
*
* JOINT ID xxx
* PARENT ID xxx
* OFFSET xxx xxx xxx
* MINANGLES xxx xxx xxx
* MAXANGLES xxx xxx xxx
* DEFAULTANGLES xxx xxx xxx
*
*  ...
*/

#ifndef _STRUCT_JOINT_IO
#define _STRUCT_JOINT_IO

#include "joint.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <exception>
#include <cstring>

namespace kinematics
{

class JointException: public std::exception
{
public:
    JointException(const std::string& message) throw()
        :mess(message){}

     virtual const char* what() const throw()
     {
         return mess.c_str();
     }

     virtual ~JointException() throw(){}

private:
    std::string mess;
};

//! @cond
template<class T, int Dim>
void WriteTreeRec(const T& joint, std::stringstream& res, unsigned int& id)
{
    res << "\nTYPE ";
    res << (int)(joint.constraintType);
    res << "\nOFFSET";
    for(int i = 0; i < Dim; ++i)
    {
        res << " " << joint.offset[i];
    }
    res << "\nMINANGLES";
    for(int i = 0; i < Dim; ++i)
    {
        res << " " << joint.minAngleValues[i];
    }
    res << "\nMAXANGLES";
    for(int i = 0; i < Dim; ++i)
    {
        res << " " << joint.maxAngleValues[i];
    }
    res << "\nDEFAULTANGLES";
    for(int i = 0; i < Dim; ++i)
    {
        res << " " << joint.defaultAngleValues[i];
    }
    res << "\n";
    unsigned int parentId = id;
    ++id;
    for(int i=0; i < joint.nbChildren_; ++i)
    {
        res << "\nJOINT ID " << id;
        res << "\nPARENT ID " << parentId;
        WriteTreeRec<T, Dim>(*(joint.children[i]), res, id);
    }
}
/// @endcond

///  \brief Saves a kinematic tree formed by joints into a text
///  file.
///	 If Safe is set to true, an exception will be thrown if it is not possible to create the file.
///  \param joint the root joint of the kinematic tree to save
///  \param filename the name of the file in which the description must be written.
template<class T, int Dim, bool Safe>
bool WriteTree(const T& joint, const std::string& filename)
{
    unsigned int id =0;
    std::stringstream res; res << "ROOT ID 0 \n";
    WriteTreeRec<T, Dim>(joint, res, id);
    std::ofstream myfile;
    myfile.open (filename.c_str());
    if(myfile.is_open())
    {
        myfile << res.rdbuf();
        myfile.close();
        return true;
    }
    else if(Safe)
    {
        std::string errMess("Unable to create file " + filename);
//		throw exception(errMess.c_str());
    }
    return false;
}

//! @cond
template<typename Numeric, int Dim, bool Safe>
void ReadLine(const std::string& line, Numeric* values)
{
    char str[20];
    float x, y;
    if(Dim == 3)
    {
        float z;
        sscanf(line.c_str(),"%s %f %f %f", str, &x, &y, &z);
        values[0] = x;
        values[1] = y;
        values[2] = z;
    }
    else if(Dim == 2)
    {
        sscanf(line.c_str(),"%s %f %f",str, &x, &y);
        values[0] = x;
        values[1] = y;
    }
}

template<typename T, typename Numeric, typename Angle, int Dim, int MaxChildren, bool Safe>
void ReadJoint(std::ifstream& myfile, std::vector<T>& joints, std::vector<std::vector<int> >& children, int id, bool isRoot)
{
    T res;
    kinematics::constraint_type constraintType = kinematics::unknown;
    std::string line;
    bool offset(false), minAngle(false), maxAngle(false), defaultAngle(false), parent(isRoot), type(false), tag(false);
    bool all(false);
    while(myfile.good() &! all)
    {
        getline(myfile, line);
        if(line.find("TYPE") == 0)
        {
            if(type && Safe) throw JointException("wrong tree definition: attribute TYPE redefinition");
            type = true;
            char str[20];
            int constraintType; sscanf(line.c_str(),"%s %d", str, &constraintType);
            res.constraintType = (constraint_type)constraintType;
        }
        if(line.find("TAG") == 0)
        {
            if(tag && Safe) throw JointException("wrong tree definition: attribute TYPE redefinition");
            tag = true;
            char str[10];
            char tag[10]; sscanf(line.c_str(),"%s %s", str, tag);
            strcpy(res.tag, tag);
        }
        if(line.find("PARENT") == 0)
        {
            if(parent && Safe) throw JointException("wrong tree definition: attribute PARENT redefinition, or joint is ROOT");
            parent = true;
            char str[20];
            int parentId; sscanf(line.c_str(),"%s %*s %d", str, &parentId);
            children[parentId].push_back(id);
        }
        if(line.find("OFFSET") == 0)
        {
            if(offset && Safe) throw JointException("wrong tree definition: attribute OFFSET redefinition");
            offset = true;
            ReadLine<Numeric, Dim, Safe>(line, res.offset);
        }
        if(line.find("MINANGLES") == 0)
        {
            if(minAngle && Safe) throw JointException("wrong tree definition: attribute MINANGLES redefinition");
            minAngle = true;
            ReadLine<Numeric, Dim, Safe>(line, res.minAngleValues);
        }
        if(line.find("MAXANGLES") == 0)
        {
            if(maxAngle && Safe) throw JointException("wrong tree definition: attribute MAXANGLES redefinition)");
            maxAngle = true;
            ReadLine<Numeric, Dim, Safe>(line, res.maxAngleValues);
        }
        if(line.find("DEFAULTANGLES") == 0)
        {
            if(defaultAngle && Safe) throw JointException("wrong tree definition: attribute DEFAULTANGLES redefinition ");
            defaultAngle = true;
            ReadLine<Numeric, Dim, Safe>(line, res.defaultAngleValues);
        }
        all = offset && minAngle && maxAngle && defaultAngle && parent;
    }
    if((!isRoot && all) || (isRoot && offset))
    {
        joints[id] = res;
    }
    else if(Safe)
    {
        throw JointException("wrong tree definition: EOF reached before complete definition");
    }
}

template<typename T>
void ConnectJoint(T* joint, int id, const std::vector<T>& joints, const std::vector<std::vector<int> >& children)
{
    std::vector<int> childrenIds = children[id];
    for(std::vector<int>::const_iterator it = childrenIds.begin(); it != childrenIds.end(); ++it)
    {
        int index = (*it);
        T* child = new T(joints[index]);
        ConnectJoint(child, index, joints, children);
        joint->add_child(child);
    }
}
/// @endcond

///  \brief Loads a kinematic tree from a given file.
///  If Safe is set to true, this method will throw an exception if the file can not be read.
///  \param filename the name of the file from which the description must be loaded.
template<typename Numeric, typename Angle, int Dim, int MaxChildren, bool Safe>
joint<Numeric, Angle, Dim, MaxChildren, Safe>* ReadTree(const std::string& filename)
{
    typedef joint<Numeric, Angle, Dim, MaxChildren, Safe> joint_t;
    std::string line;
    std::ifstream myfile (filename);
    int rootId = -1; bool rootFound(false);
    std::vector<joint_t> joints(100);
    std::vector<std::vector<int> > children(100);
    if (myfile.is_open())
    {
        while ( myfile.good() )
        {
            getline (myfile, line);
            if(line.find("ROOT") == 0 || line.find("JOINT") == 0)
            {
                char str[20];
                int id; sscanf(line.c_str(),"%s %*s %d", str, &id);
                bool isRoot = (line.find("ROOT") == 0);
                if(isRoot)
                {
                    if(rootFound && Safe)
                    {
                        throw JointException("wrong tree definition: Multiple ROOT definition");
                    }
                    rootId = id;
                    rootFound = true;
                }
                ReadJoint<joint_t, Numeric, Angle, Dim, MaxChildren, Safe>(myfile, joints, children, id, isRoot);
            }
        }
        myfile.close();
    }
    else if(Safe)
    {
        std::string errMess("Unable to open file " + filename + "for reading.");
        throw JointException(errMess.c_str());
    }
    else
    {
        return 0;
    }
    if(rootFound)
    {
        joint_t* root = new joint_t(joints[rootId]);
        ConnectJoint(root, rootId, joints, children);
        return root;
    }
    else if(Safe)
    {
        throw JointException("wrong tree definition: No ROOT definition");
    }
    return 0;
}

}// end namespace kinematics
#endif //_STRUCT_JOINT_IO
