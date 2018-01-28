#include <iostream>
#include <cstring>

#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/JointPath>

using namespace std;
using namespace cnoid;

double deg2rad(double degree)
{
    return degree * M_PI / 180.f;
}

int main()
{
    BodyLoader bodyloader;

    std::string model_path("/usr/share/choreonoid-1.6/model/SR1/SR1.body");
    BodyPtr robot = bodyloader.load(model_path.c_str());

    JointPathPtr lleg = getCustomJointPath(robot, robot->rootLink(), robot->link("LLEG_ANKLE_R"));

    lleg->calcForwardKinematics();

    cout << robot->link("LLEG_ANKLE_R")->p() << endl;

    robot->joint(14)->q() = deg2rad(-20);
    robot->joint(16)->q() = deg2rad(40);
    robot->joint(17)->q() = deg2rad(-20);

    lleg->calcForwardKinematics();

    cout << robot->link("LLEG_ANKLE_R")->p() << endl;

    return 0;
}