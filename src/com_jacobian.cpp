#include <iostream>
#include <cstring>

#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/Jacobian>
#include <cnoid/BodyLoader>

using namespace std;
using namespace cnoid;

int main()
{
	BodyLoader bodyloader;
	
	string model_path("/usr/share/choreonoid-1.6/model/SR1/SR1.body");
	BodyPtr robot = bodyloader.load(model_path.c_str());

	JointPathPtr leg = getCustomJointPath(robot, robot->link("RLEG_ANKLE_R"), robot->link("LLEG_ANKLE_R"));

	MatrixXd J_com(3, robot->numJoints());
	calcCMJacobian(robot, leg->baseLink(), J_com);

	std::cout << J_com << std::endl;

	return 0;
}