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

	std::cout << robot->rootLink()->p() << std::endl;
	return 0;
}
