
#include "collision/ParserObj.h"
#include "collision/Object.h"
#include "collision/Collider.h"

#include "prm/LocalPlanner.h"

#include <string>
#include <iostream>
#include <cmath>

namespace
{
bool quasiEqual(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    return (a-b).norm() < 0.0001;
}
}

using namespace std;

void ObjParserCanLoadFileTest(bool& error)
{
    std::string targetFile("../tests/collision/armoire.obj");
    planner::Object::T_Object objects = planner::ParseObj(targetFile);
    if(objects.size() != 10)
    {
        error = true;
        std::cout << "error loading armoire.obj, expecting 10 objects, generated " << objects.size() << std::endl;
    }
}

void ObstacleCreationTest(bool& error)
{
    std::string targetFile("../tests/collision/cube.obj");
    planner::Object::T_Object objects = planner::ParseObj(targetFile);
    if(objects.size() != 1)
    {
        error = true;
        std::cout << "error loading cube.obj, expecting 1 object, generated " << objects.size() << std::endl;
        return;
    }
    std::string targetFile2("../tests/collision/cubequad.obj");
    planner::Object::T_Object objects2 = planner::ParseObj(targetFile2);
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
    planner::Object::T_Object objects = planner::ParseObj(targetFile);
    planner::Collider collider(objects);
    if(collider.IsColliding())
    {
        error = true;
        std::cout << "in collision tests 1, cube should not collide with itself" << std::endl;
    }
    planner::ParseObj(targetFile, objects);
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

void LocalPlannerTest(bool& error)
{
    std::string targetFile("../tests/collision/cube.obj");
    planner::Object::T_Object objects = planner::ParseObj(targetFile);
    std::string model2("../tests/collision/cubeenglob.obj");
    planner::Object::T_Object objects2 = planner::ParseObj(model2);
    /*planner::Object a(*objects[0]);
    planner::Object b(*objects[0]);*/
	
    planner::Model robot;
    robot.englobed = objects[0];
    planner::ParseObj(model2, robot.englobing);
    robot.englobing = objects2;
    planner::LocalPlanner lPlanner(objects, objects, robot);

    planner::Model a(robot);
    planner::Model b(robot);

    Eigen::Vector3d collisionA(-3,0,0);
    Eigen::Vector3d collisionB(3,0,0);
    a.SetPosition(collisionA); b.SetPosition(collisionB);

    if(lPlanner(&a, &b))
    {
        error = true;
        std::cout << "ERROR in LocalPlannerTest1 : collision detected in path from a to b" << std::endl;
    }


    Eigen::Vector3d noCollisionA(-3,0,0);
    Eigen::Vector3d noCollisionB(-3,-6,0);
    a.SetPosition(noCollisionA); b.SetPosition(noCollisionB);
	// TODO rewrite this test for taking model into account
    if(!lPlanner(&a, &b))
    {
        error = true;
        std::cout << "ERROR in LocalPlannerTest2 : collision should not be detected in path from a to b" << std::endl;
    }
}

#include "prm/SimplePRM.h"

void SerializeSimplePRMTest(bool& error)
{
    std::string targetFile("../tests/collision/wall_1s.obj");
    std::string model("../tests/collision/cube.obj");
    std::string model2("../tests/collision/cubeenglob.obj");
    std::string outpath("../tests/testSerialization.txt");
    std::string outpath2("../tests/testSerialization2.txt");
    planner::Object::T_Object objects = planner::ParseObj(targetFile, true);

    planner::Object::T_Object objects2 = planner::ParseObj(model);

    planner::Model robot;
    robot.englobed = objects2[0];
    planner::ParseObj(model2, robot.englobing);
    //robot.englobing = objects2[1];
    planner::SimplePRM* prm = new planner::SimplePRM(robot, objects, 10, 10, 4);
    planner::SavePrm(*prm, outpath);
    planner::SimplePRM* prm2 = planner::LoadPRM(outpath, objects, robot);
    planner::SavePrm(*prm2, outpath2);
}

#include "collision/Sphere.h"

void SphereTest(bool& error)
{
    planner::Sphere a(0,0,0,1);
    planner::Sphere b(0,0,0,1);
    planner::Sphere c(0,0,0,0.8);
    planner::Sphere d(0.3,0.3,0.3,0.8);
    planner::Sphere e(2,0,0,1);
    planner::SphereCollisionRes res = planner::Intersect(a,b,true);
    if(res.collisionType != planner::sphereContained)
    {
        error = true;
        std::cout << "ERROR in SphereTest 1: sphere a and b are contained, got " << res.collisionType << std::endl;
    }


    res = planner::Intersect(b,c,true);
    if(res.collisionType != planner::sphereContained)
    {
        error = true;
        std::cout << "ERROR in SphereTest 2: sphere c contained by b, got " << res.collisionType << std::endl;
    }


    res = planner::Intersect(c,b,true);
    if(res.collisionType != planner::sphereContained)
    {
        error = true;
        std::cout << "ERROR in SphereTest 3: sphere c contained by b, got " << res.collisionType << std::endl;
    }


    Eigen::Vector3d posad(0.35,0.35,0.35);
    res = planner::Intersect(a,d,true);
    if(res.collisionType != planner::cercle)
    {
        error = true;
        std::cout << "ERROR in SphereTest 4: sphere a and d intersect, got " << res.collisionType << std::endl;
    }
    if(!quasiEqual(posad,res.center))
    {
        error = true;
        std::cout << "ERROR in SphereTest 5: sphere a and d intersect point: expected\n" << posad << "got\n"  << res.center << std::endl;
    }
    res = planner::Intersect(d,a,true);
    if(res.collisionType != planner::cercle)
    {
        error = true;
        std::cout << "ERROR in SphereTest 4: sphere a and d intersect, got " << res.collisionType << std::endl;
    }
    if(!quasiEqual(posad,res.center))
    {
        error = true;
        std::cout << "ERROR in SphereTest 5: sphere a and d intersect point: expected\n" << posad << "got\n"  << res.center << std::endl;
    }


    Eigen::Vector3d posae(1,0,0);
    res = planner::Intersect(a,e,true);
    if(res.collisionType != planner::tangentialExterior)
    {
        error = true;
        std::cout << "ERROR in SphereTest 6: sphere a and d tangent, got " << res.collisionType << std::endl;
    }
    if(!quasiEqual(posae,res.center))
    {
        error = true;
        std::cout << "ERROR in SphereTest 7: sphere a and d intersect point: expected\n" << posad << "got\n"  << res.center << std::endl;
    }
    if(res.radius != 0)
    {
        error = true;
        std::cout << "ERROR in SphereTest 8: sphere a and d tangent, radius should b 0 " << res.collisionType << std::endl;
    }
}

int main(int argc, char *argv[])
{	std::cout << "performing tests... \n";
    bool error = false;
    ObjParserCanLoadFileTest(error);
    ObstacleCreationTest(error);
    CollisionDetectionTest(error);
    LocalPlannerTest(error);
    SerializeSimplePRMTest(error);
    SphereTest(error);
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

