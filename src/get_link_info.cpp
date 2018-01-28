#include <iostream>
#include <cstring>

#include <cnoid/Body>
#include <cnoid/BodyLoader>

using namespace std;
using namespace cnoid;

int main()
{
	BodyLoader bodyloader;

	std::string model_path("model/SR1.wrl");
	BodyPtr robot = bodyloader.load(model_path.c_str());

	cout << "dof: " << robot->numJoints() << endl;
	cout << "base link name: " << robot->rootLink()->name() << endl;
	cout << "base link pos: \n" << robot->rootLink()->p() << endl;
	cout << "base link rot: \n" << robot->rootLink()->R() << endl;

	return 0;
}
