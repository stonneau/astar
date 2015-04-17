
#include "equilibrium/DynamicStability.h"

#include <string>
#include <iostream>
#include <cmath>

using namespace std;
#include "Test.h"
#include "libcdd/setoper.h"
#include "libcdd/cdd.h"

#include <vector>

using namespace equilib;


namespace
{

void GroundEquilibriumTest(bool& error)
{
    T_Transform contactTransforms, graspTransforms;
    Eigen::Vector3d comLocation = Eigen::Vector3d(0.5,0.5,0.5);
    Eigen::Matrix4d contactTransform1 = Eigen::Matrix4d::Identity();
    contactTransform1.block<3,1>(0,3) = Eigen::Vector3d(0,0,1);
    Eigen::Matrix4d contactTransform2 = Eigen::Matrix4d::Identity();
    contactTransform2.block<3,1>(0,3) = Eigen::Vector3d(1,0,0);
    Eigen::Matrix4d contactTransform3 = Eigen::Matrix4d::Identity();
    contactTransform3.block<3,1>(0,3) = Eigen::Vector3d(1,0,1);
    contactTransforms.push_back(contactTransform1);contactTransforms.push_back(contactTransform2);
    contactTransforms.push_back(contactTransform3);
    Eigen::VectorXd maxGraspingForces = Eigen::VectorXd::Zero(3);
    Eigen::Vector3d acceleration(0,0,0);
    bool loceror = !equilib::CheckEquilibrium( contactTransforms, graspTransforms, maxGraspingForces,acceleration, comLocation, 50, 1, 491);
    if (loceror)
    {
        error = true;
        std::cout << "Error in ground equilibrium tests" << std::endl;
    }
}

void GroundEquilibriumTest2(bool& error)
{
    T_Transform contactTransforms, graspTransforms;
    Eigen::Vector3d comLocation = Eigen::Vector3d(0.5,-0.5,0.5);
    Eigen::Matrix4d contactTransform1 = Eigen::Matrix4d::Identity();
    contactTransform1.block<3,1>(0,3) = Eigen::Vector3d(0,0,1);
    Eigen::Matrix4d contactTransform2 = Eigen::Matrix4d::Identity();
    contactTransform2.block<3,1>(0,3) = Eigen::Vector3d(1,0,0);
    Eigen::Matrix4d contactTransform3 = Eigen::Matrix4d::Identity();
    contactTransform3.block<3,1>(0,3) = Eigen::Vector3d(1,0,1);
    contactTransforms.push_back(contactTransform1);contactTransforms.push_back(contactTransform2);
    contactTransforms.push_back(contactTransform3);
    Eigen::VectorXd maxGraspingForces = Eigen::VectorXd::Zero(3);
    Eigen::Vector3d acceleration(0,0,0);
    bool loceror = !equilib::CheckEquilibrium( contactTransforms, graspTransforms, maxGraspingForces,acceleration, comLocation, 50, 1, 491);
    if (loceror)
    {
        error = true;
        std::cout << "Error in ground equilibrium test 2" << std::endl;
    }
}

void GroundEquilibriumFail(bool& error)
{

    T_Transform contactTransforms, graspTransforms;
    Eigen::Vector3d comLocation = Eigen::Vector3d(0,0,0);
    Eigen::Matrix4d contactTransform1 = Eigen::Matrix4d::Identity();
    contactTransform1.block<3,1>(0,3) = Eigen::Vector3d(0,0,1);
    Eigen::Matrix4d contactTransform2 = Eigen::Matrix4d::Identity();
    contactTransform2.block<3,1>(0,3) = Eigen::Vector3d(1,0,0);
    Eigen::Matrix4d contactTransform3 = Eigen::Matrix4d::Identity();
    contactTransform3.block<3,1>(0,3) = Eigen::Vector3d(1,0,1);
    contactTransforms.push_back(contactTransform1);contactTransforms.push_back(contactTransform2);
    contactTransforms.push_back(contactTransform3);
    Eigen::VectorXd maxGraspingForces = Eigen::VectorXd::Zero(3);
    Eigen::Vector3d acceleration(0,0,0);
    bool loceror = equilib::CheckEquilibrium( contactTransforms, graspTransforms, maxGraspingForces,acceleration, comLocation, 50, 1, 491);
    if (loceror)
    {
        error = true;
        std::cout << "Error in ground GroundEquilibriumFail tests" << std::endl;
    }
}
}

int main(int argc, char *argv[])
{
	std::cout << "performing tests... \n";
    bool error = false;
    GroundEquilibriumTest(error);
    GroundEquilibriumTest2(error);
    GroundEquilibriumFail(error);
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

