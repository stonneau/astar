
#include "collision/ParserObj.h"
#include "collision/Object.h"
#include "collision/Collider.h"

#include <string>
#include <iostream>
#include <cmath>

using namespace std;

void ObjParserCanLoadFileTest(bool& error)
{
    std::string targetFile("../tests/collision/armoire.obj");
    planner::ParserObj parser;
    planner::Object::T_Object objects = parser.CreateWorld(targetFile);
    if(objects.size() != 10)
    {
        error = true;
        std::cout << "error loading armoire.obj, expecting 10 objects, generated " << objects.size() << std::endl;
    }
}

void ObstacleCreationTest(bool& error)
{
    std::string targetFile("../tests/collision/cube.obj");
    planner::ParserObj parser;
    planner::Object::T_Object objects = parser.CreateWorld(targetFile);
    if(objects.size() != 1)
    {
        error = true;
        std::cout << "error loading cube.obj, expecting 1 object, generated " << objects.size() << std::endl;
        return;
    }
    std::string targetFile2("../tests/collision/cubequad.obj");
    planner::ParserObj parser2;
    planner::Object::T_Object objects2 = parser2.CreateWorld(targetFile2);
    if(objects2.size() != 1)
    {
        error = true;
        std::cout << "error loading cubequad.obj, expecting 1 object, generated " << objects2.size() << std::endl;
        return;
    }
    planner::Object * obj = objects.front();
    if(obj->GetModel()->num_tris != 12)
    {
        error = true;
        std::cout << "error loading cube.obj, expecting 12 triangles, generated " << obj->GetModel()->num_tris  << std::endl;
    }
    planner::Object * obj2 = objects2.front();
    if(obj2->GetModel()->num_tris != 12)
    {
        error = true;
        std::cout << "error loading cubequad.obj, expecting 12 triangles, generated " << obj2->GetModel()->num_tris  << std::endl;
    }
}

void CollisionDetectionTest(bool& error)
{
    std::string targetFile("../tests/collision/cube.obj");
    planner::ParserObj parser;
    planner::Object::T_Object objects = parser.CreateWorld(targetFile);
    planner::Collider collider(objects);
    if(collider.IsColliding())
    {
        error = true;
        std::cout << "in collision tests 1, cube should not collide with itself" << std::endl;
    }
    objects = parser.CreateWorld(targetFile);
    planner::Collider collider2(objects);
    if(!collider2.IsColliding())
    {
        error = true;
        std::cout << "in collision tests 2, created twice the same cube, collision should occur" << std::endl;
    }
    Eigen::Vector3d newPos(12.9,2.9,2.9);
    objects[0]->SetPosition(newPos);
    collider2 = planner::Collider(objects);
    if(collider2.IsColliding())
    {
        error = true;
        std::cout << "After translation cube should not collide" << std::endl;
    }
}


int main(int argc, char *argv[])
{
	planner::ParserObj parser;
	std::cout << "performing tests... \n";
    bool error = false;
    ObjParserCanLoadFileTest(error);
    ObstacleCreationTest(error);
    CollisionDetectionTest(error);
	if(error)
	{
		std::cout << "There were some errors\n";
		return -1;
	}
	else
	{
		std::cout << "no errors found \n";
		return 0;
	}
}

