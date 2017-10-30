#include <iostream>
#include <cstring>

#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/Link>
#include <cnoid/JointPath>

int main()
{
	cnoid::JointPath JointPath;
	cnoid::JointPathPtr JointPathPtr;
	cnoid::BodyPtr robot;
	cnoid::BodyLoader bodyloader;
	cnoid::Link Link;

	std::string model_path("/home/haze/github/VRMLModelLoaderSample/model/JAXON_RED/JAXON_JVRCmain.wrl");

	robot = bodyloader.load(model_path.c_str());

	std::string joint_name[robot->numJoints()];
	for(std::size_t index=0;index<robot->numJoints();index++){
		joint_name[index] = robot->link(index)->name();
		std::cout << joint_name[index] << std::endl;
	}
	return 0;
}
