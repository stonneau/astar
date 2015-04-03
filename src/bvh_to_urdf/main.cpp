
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#ifdef WIN32
#include <windows.h>
#endif

#include <stack>
#include <Eigen/Dense>

#include "create_obj_box.h"

struct Node
{
    Node() : tag (""), axis(Eigen::Vector3d(0,0,0)), offset(axis){}
    ~Node(){}
    std::string tag;
    Eigen::Vector3d axis;
    Eigen::Vector3d offset;
    std::vector<Node*> children;
    Node* parent;
};


using namespace std;
using namespace Eigen;

namespace
{
    double StrToD (const std::string &str)
    {
        istringstream ss(str);
        double result;
        return ss >> result ? result : 0;
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
                if(s1!="") ret.push_back(s1);
                s1="";
            }
            else
                s1+=s[i];
        }
        return ret;
    }
/*
    void ChannelRec(const std::string& line, const std::string& currentName, Node* current)
    {
        if(line.find("Zrotation") != string::npos)
        {
            current->tag = currentName + "_z";
            current->axis = Eigen::Vector3d(0,0,1);
        }
        else if(line.find("Xrotation") != string::npos)
        {
            current->tag = currentName + "_x";
            current->axis = Eigen::Vector3d(1,0,0);
        }
        else if(line.find("Yrotation") != string::npos)
        {
            current->tag = currentName + "_y";
            current->axis = Eigen::Vector3d(0,1,0);
        }
    }

    Node* Channel(const std::vector<std::string>& split, Node* current)
    {
        const std::string name = current->parent->tag;
        bool first = true;
        for(std::vector<std::string>::const_iterator cit = split.begin() + 3; cit !=split.end(); ++cit)
        {
            // ignore translation
            if(!((cit->find("Zposition") != string::npos)
                || (cit->find("Xposition") != string::npos)
                || (cit->find("Yposition") != string::npos)))
            {
                if(first)
                {
                    first = false;
                }
                else
                {   Node * son = new Node;
                    son->parent = current;
                    current->children.push_back(son);
                    current = son;
                }
                ChannelRec(*cit, name, current);
            }
        }
        return current;
    }*/

    Node* ProcessLine(const std::string& line, Node* current, std::stack<Node*>& stack, double scale)
    {
        // case:
        // empty line or { we go down one level
        // OFFSET => apply to current Node
        // CHANNELS => create as many Nodes and channel
        // ROOT / JOINT => Name of Node (tag)
        // } => Node is finished, go back to parent
        std::vector<std::string> split = splitSpace(line);
        if(line.find("{") != string::npos)
        {
            stack.push(current);
        }
        if(((line.find("ROOT") != string::npos) || (line.find("JOINT") != string::npos)) && split.size() >1)
        {
            Node * son = new Node;
            if(current)
            {
                son->parent = current;
                current->children.push_back(son);
            }
            current = son;
            current->tag = split[1];
        }
        if((line.find("End Site") != string::npos) && split.size() >1)
        {
            Node * son = new Node;
            if(current)
            {
                son->parent = current;
                current->children.push_back(son);
                son->tag = current->tag + "endsite";
            }
            current = son;
        }
        if((line.find("OFFSET") != string::npos) && split.size() == 4)
        {
            for(int i =1; i<4; ++i)
            {
                current->offset(i-1) = StrToD(split[i]) * scale;
            }
        }
        if(line.find("CHANNELS") != string::npos)
        {
            // In fact we don't care about channels, they are always the same
            //current = Channel(split, current);
        }
        if(line.find("}") != string::npos)
        {
            current = stack.top();stack.pop();
            if(current->parent != 0) current = current->parent;
        }
        return current;
    }

}


//export namespace
namespace
{

    void exportHeader(std::stringstream& outstream)
    {
        outstream << "<?xml version=\"1.0\"?>" <<'\n';
        outstream << "<robot xmlns:sensor=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor\"" <<'\n';
        outstream << "       xmlns:controller=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#controller\"" <<'\n';
        outstream << "       xmlns:interface=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#interface\"" <<'\n';
        outstream << "       xmlns:xacro=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#interface\"" <<'\n';
        outstream << "       name=\"rocketbox_male\" >" <<'\n';
        outstream << "\n";
        outstream << "    <material name=\"Blue\">" <<'\n';
        outstream << "      <color rgba=\"0.0 0.0 0.8 1.0\"/>" <<'\n';
        outstream << "    </material>" <<'\n';
        outstream << "    <material name=\"Green\">" <<'\n';
        outstream << "      <color rgba=\"0.0 0.8 0.0 1.0\"/>" <<'\n';
        outstream << "    </material>" <<'\n';
        outstream << "    <material name=\"Grey\">" <<'\n';
        outstream << "      <color rgba=\"0.7 0.7 0.7 1.0\"/>" <<'\n';
        outstream << "    </material>" <<'\n';
        outstream << "    <material name=\"Grey2\">" <<'\n';
        outstream << "      <color rgba=\"0.9 0.9 0.9 1.0\"/>" <<'\n';
        outstream << "    </material>" <<'\n';
        outstream << "    <material name=\"Red\">" <<'\n';
        outstream << "      <color rgba=\"0.8 0.0 0.0 1.0\"/>" <<'\n';
        outstream << "    </material>" <<'\n';
        outstream << "    <material name=\"White\">" <<'\n';
        outstream << "      <color rgba=\"1.0 1.0 1.0 1.0\"/>" <<'\n';
        outstream << "    </material>" <<'\n';
        outstream << "    <material name=\"Black\">" <<'\n';
        outstream << "      <color rgba=\"0.1 0.1 0.1 1.0\"/>" <<'\n';
        outstream << "    </material>" <<'\n';
        outstream << "    <material name=\"LightGrey\">" <<'\n';
        outstream << "      <color rgba=\"0.6 0.6 0.6 1.0\"/>" <<'\n';
        outstream << "    </material>" <<'\n';
        outstream << "" <<'\n';
    }

    void exportFooter(std::stringstream& outstream)
    {
        outstream << "" <<'\n';
        outstream << "</robot>" <<'\n';
    }

    void exportBase(std::stringstream& outstream)
    {
		outstream <<  "    <link name=\"root_link\"/>" <<'\n';
		outstream <<  "      <link name=\"base_prismatic_dummy1\">" <<'\n';
		outstream <<  "        <visual>" <<'\n';
		outstream <<  "          <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" <<'\n';
		outstream <<  "          <geometry>" <<'\n';
		outstream <<  "            <box size=\"0.001 0.001 0.001\"/>" <<'\n';
		outstream <<  "          </geometry>" <<'\n';
		outstream <<  "          <material name=\"Blue\"/>" <<'\n';
		outstream <<  "        </visual>" <<'\n';
		outstream <<  "      </link>" <<'\n';
		outstream <<  "      <link name=\"base_prismatic_dummy2\">" <<'\n';
		outstream <<  "        <visual>" <<'\n';
		outstream <<  "          <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" <<'\n';
		outstream <<  "          <geometry>" <<'\n';
		outstream <<  "            <box size=\"0.001 0.001 0.001\"/>" <<'\n';
		outstream <<  "          </geometry>" <<'\n';
		outstream <<  "          <material name=\"Blue\"/>" <<'\n';
		outstream <<  "        </visual>" <<'\n';
		outstream <<  "      </link>" <<'\n';
		outstream <<  "      <link name=\"base_prismatic_dummy3\">" <<'\n';
		outstream <<  "        <visual>" <<'\n';
		outstream <<  "          <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" <<'\n';
		outstream <<  "          <geometry>" <<'\n';
		outstream <<  "            <box size=\"0.001 0.001 0.001\"/>" <<'\n';
		outstream <<  "          </geometry>" <<'\n';
		outstream <<  "          <material name=\"Blue\"/>" <<'\n';
		outstream <<  "        </visual>" <<'\n';
		outstream <<  "      </link>" <<'\n';
		outstream <<  "      <link name=\"base_revolute_dummy1\">" <<'\n';
		outstream <<  "        <visual>" <<'\n';
		outstream <<  "          <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" <<'\n';
		outstream <<  "          <geometry>" <<'\n';
		outstream <<  "            <box size=\"0.001 0.001 0.001\"/>" <<'\n';
		outstream <<  "          </geometry>" <<'\n';
		outstream <<  "          <material name=\"Blue\"/>" <<'\n';
		outstream <<  "        </visual>" <<'\n';
		outstream <<  "      </link>" <<'\n';
		outstream <<  "      <link name=\"base_revolute_dummy2\">" <<'\n';
		outstream <<  "        <visual>" <<'\n';
		outstream <<  "          <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" <<'\n';
		outstream <<  "          <geometry>" <<'\n';
		outstream <<  "            <box size=\"0.001 0.001 0.001\"/>" <<'\n';
		outstream <<  "          </geometry>" <<'\n';
		outstream <<  "          <material name=\"Blue\"/>" <<'\n';
		outstream <<  "        </visual>" <<'\n';
		outstream <<  "      </link>" <<'\n';
		outstream <<  "      <joint name=\"base_prismatic_joint_x\" type=\"prismatic\">" <<'\n';
		outstream <<  "        <axis xyz=\"1 0 0\"/>" <<'\n';
        outstream <<  "        <limit effort=\"10000\" lower =\"-5.0\" upper=\"5.0\" velocity =\"0.6\"/>" <<'\n';
		outstream <<  "        <origin rpy=\"0 0 0\" xyz=\"0 0 1.1713\"/>" <<'\n';
		outstream <<  "        <parent link=\"root_link\"/>" <<'\n';
		outstream <<  "        <child link=\"base_prismatic_dummy1\"/>" <<'\n';
		outstream <<  "      </joint>" <<'\n';
		outstream <<  "      <joint name=\"base_prismatic_joint_y\" type=\"prismatic\">" <<'\n';
		outstream <<  "        <axis xyz=\"0 1 0\"/>" <<'\n';
        outstream <<  "        <limit effort=\"10000\" lower =\"-5.0\" upper=\"5.0\" velocity =\"0.6\"/>" <<'\n';
		outstream <<  "        <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" <<'\n';
		outstream <<  "        <parent link=\"base_prismatic_dummy1\"/>" <<'\n';
		outstream <<  "        <child link=\"base_prismatic_dummy2\"/>" <<'\n';
		outstream <<  "      </joint>" <<'\n';
		outstream <<  "      <joint name=\"base_prismatic_joint_z\" type=\"prismatic\">" <<'\n';
		outstream <<  "        <axis xyz=\"0 0 1\"/>" <<'\n';
        outstream <<  "        <limit effort=\"10000\" lower =\"-5.0\" upper=\"5.0\" velocity =\"0.6\"/>" <<'\n';
		outstream <<  "        <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" <<'\n';
		outstream <<  "        <parent link=\"base_prismatic_dummy2\"/>" <<'\n';
		outstream <<  "        <child link=\"base_prismatic_dummy3\"/>" <<'\n';
		outstream <<  "      </joint>" <<'\n';
		outstream <<  "      <joint name=\"base_revolute_joint_x\" type=\"continuous\">" <<'\n';
		outstream <<  "        <axis xyz=\"1 0 0\"/>" <<'\n';
		outstream <<  "        <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" <<'\n';
		outstream <<  "        <parent link=\"base_prismatic_dummy3\"/>" <<'\n';
		outstream <<  "        <child link=\"base_revolute_dummy1\"/>" <<'\n';
		outstream <<  "      </joint>" <<'\n';
		outstream <<  "      <joint name=\"base_revolute_joint_y\" type=\"continuous\">" <<'\n';
		outstream <<  "        <axis xyz=\"0 1 0\"/>" <<'\n';
		outstream <<  "        <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" <<'\n';
		outstream <<  "        <parent link=\"base_revolute_dummy1\"/>" <<'\n';
		outstream <<  "        <child link=\"base_revolute_dummy2\"/>" <<'\n';
		outstream <<  "      </joint>" <<'\n';
		outstream <<  "      <joint name=\"base_revolute_joint_z\" type=\"continuous\">" <<'\n';
		outstream <<  "        <axis xyz=\"0 0 1\"/>" <<'\n';
		outstream <<  "        <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" <<'\n';
		outstream <<  "        <parent link=\"base_revolute_dummy2\"/>" <<'\n';
		outstream <<  "        <child link=\"pelvis_link\"/>" <<'\n';
		outstream <<  "      </joint>" <<'\n';
		outstream <<  "      <link name=\"pelvis_link\"/>" <<'\n';
    }

    std::string from_double(double dbl)
    {
        std::ostringstream strs;
        strs << dbl;
        return strs.str();
    }

    enum axis {z,y,x};
    void exportOneNodeJoint(std::stringstream& outstream, axis ax, Node* node)
    {
        std::string rotaxis, offset;
        std::string axisname;
        std::string parent, son;
        std::string ox, oy ,oz;
        ox = from_double(node->offset(0));
        oy = from_double(node->offset(1));
        oz = from_double(node->offset(2));
        objects::create_obj_box(node->tag, node->offset);
        switch(ax) {
            case x   : {offset = "0 0 0"; rotaxis = "1 0 0"; axisname = "x"; parent = node->tag + "_y_link"; son = node->tag + "_x_link"; break;}
            case y   : {offset = "0 0 0"; rotaxis = "0 1 0"; axisname = "y"; parent = node->tag + "_z_link"; son = node->tag + "_y_link"; break; }
            case z   : {offset = "" + ox + " " + oy + " " + oz; rotaxis = "0 0 1"; axisname = "z";
                        parent = node->parent->tag + "_x_link";
                        if(node->parent->tag.find("pelvis") != std::string::npos) parent = node->parent->tag + "_link";
                        son = node->tag + "_z_link"; break;}
        }
        outstream <<  "      <joint name=\"" << node->tag << "_" << axisname << "_joint" << "\" type =\"revolute\">" <<'\n';
        outstream <<  "          <axis xyz=\"" << rotaxis << "\"/>" <<'\n';                
        outstream <<  "          <limit effort=\"10000\" lower =\"-3.14\" upper=\"3.14\" velocity =\"0.6\"/>" <<'\n';
        outstream <<  "          <origin rpy=\"0 0 0\" xyz=\"" <<  offset << "\"/>" <<'\n';
        outstream <<  "          <parent link=\"" << parent  << "\"/>" <<'\n';
        outstream <<  "          <child link=\"" << son << "\"/>" <<'\n';
        outstream <<  "      </joint>" <<'\n';
    }

    void exportLinks(std::stringstream& outstream, const std::string& name) //
    {
        outstream <<  "      <link name=\"" << name << "_z_link" << "\"/>" <<'\n';
        outstream <<  "      <link name=\"" << name << "_y_link" << "\"/>" <<'\n';
        outstream <<  "      <link name=\"" << name << "_x_link" << "\">" <<'\n';
        outstream <<  "        <visual>" <<'\n';
        outstream <<  "          <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" <<'\n';
        outstream <<  "          <geometry>" <<'\n';
        //outstream <<  "            <mesh filename=\"./"<< "dummy" <<".obj\"/>" <<'\n';
        outstream <<  "            <mesh filename=\"../rami/model/"<< name <<".obj\"/>" <<'\n';
        outstream <<  "          </geometry>" <<'\n';
        outstream <<  "          <material name=\"Blue\"/>" <<'\n';
        outstream <<  "        </visual>" <<'\n';
        outstream <<  "        <collision>" <<'\n';
        outstream <<  "          <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" <<'\n';
        outstream <<  "          <geometry>" <<'\n';
        //outstream <<  "            <mesh filename=\"./"<< "dummy" <<".obj\"/>" <<'\n';
        //outstream <<  "            <mesh filename=\"./"<< name <<".obj\"/>" <<'\n';
        outstream <<  "            <mesh filename=\"../rami/model/"<< name <<".obj\"/>" <<'\n';
        outstream <<  "          </geometry>" <<'\n';
        outstream <<  "        </collision>" <<'\n';
        outstream <<  "      </link>" <<'\n';
    }

    void exportNodeRec(std::stringstream& outstream, Node* cnode)
    {
        exportLinks(outstream, cnode->tag);
        for(int i = 0; i< 3; ++i)
        {
            exportOneNodeJoint(outstream, (::axis(i)), cnode);
        }
        for(std::vector<Node*>::iterator it = cnode->children.begin();
            it != cnode->children.end(); ++it)
        {
            exportNodeRec(outstream, *it);
        }
    }

    void exportNode(std::stringstream& outstream, Node* cnode)
    {
        for(std::vector<Node*>::iterator it = cnode->children.begin();
            it != cnode->children.end(); ++it)
        {
            exportNodeRec(outstream, *it);
        }
    }



    void export_urdf(Node* current,std::stringstream& outstream)
    {
        exportHeader(outstream);
        exportBase(outstream);
        current->tag = "pelvis";
        exportNode(outstream, current);
        exportFooter(outstream);
    }
}

int main(int argc, char *argv[])
{
    if(argc < 3 )
	{
        std::cout << "usage is 'bvhfilename' 'urdffilename' " << std::endl;
		return -1;
	}

    Node* current(0);
    std::stack<Node*> stack;
    std::string bvhpath = argv[1];
    std::string urdfpath = argv[2];
    std::string scale = (argc > 3)  ? (argv[3]) : (std::string ("1"));
    double s = StrToD(scale);
    std::ifstream myfile (bvhpath);
    // first parse bvh and get structure
    if (myfile.is_open())
    {
        std::string line;
        while (myfile.good())
        {
            getline(myfile, line);
            current = ProcessLine(line, current, stack, s);
        }
        myfile.close();
    }
    //export structure to urdf

    std::stringstream outstream;
    export_urdf(current, outstream);
    ofstream outfile;
    outfile.open(urdfpath.c_str());
    if (outfile.is_open())
    {
        outfile << outstream.rdbuf();
        outfile.close();
    }
    else
    {
        std::cout << "Can not open outfile " << urdfpath << std::endl;
        return false;
    }
    return 0;
}
