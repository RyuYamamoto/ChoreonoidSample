#include <iostream>
#include <cstring>

#include "Link.cpp"
#include <cnoid/Body>
#include <cnoid/BodyLoader>

int main()
{
	cnoid::BodyPtr robot;
	cnoid::BodyLoader bodyloader;

	std::string model_path("/home/haze/github/VRMLModelLoaderSample/model/JAXON_RED/JAXON_JVRCmain.wrl");

	robot = bodyloader.load(model_path.c_str());

	cit::Link ulink[robot->numJoints()];
	std::string joint_name[robot->numJoints()];
	for(std::size_t index=0;index<robot->numJoints();index++){
		ulink[index].joint_name = robot->link(index)->name();
		ulink[index].p = robot->link(index)->p();
		ulink[index].R = robot->link(index)->R();
		ulink[index].a = robot->link(index)->a();
		ulink[index].b = robot->link(index)->b();
		ulink[index].w = robot->link(index)->w();
		ulink[index].v = robot->link(index)->v();
		ulink[index].m = robot->link(index)->m();
	}
	return 0;
}
