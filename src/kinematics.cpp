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

double rad2deg(double radian)
{
    return radian * 180.f / M_PI;
}

int main()
{
    BodyLoader bodyloader;

    std::string model_path("/usr/share/choreonoid-1.6/model/SR1/SR1.body");
    BodyPtr robot = bodyloader.load(model_path.c_str());

    JointPathPtr lleg = getCustomJointPath(robot, robot->rootLink(), robot->link("LLEG_ANKLE_R"));

    robot->joint(14)->q() = deg2rad(-20);
    robot->joint(16)->q() = deg2rad(40);
    robot->joint(17)->q() = deg2rad(-20);

    lleg->calcForwardKinematics();

    for(int i=13;i<19;i++)
        cout << rad2deg(robot->joint(i)->q()) << endl;
    cout << "\n";

    Vector3d ref_pos = robot->link("LLEG_ANKLE_R")->p();
    Matrix3d ref_rot = robot->link("LLEG_ANKLE_R")->R();

    ref_pos[2] += 0.01;
    if(lleg->calcInverseKinematics(ref_pos, ref_rot)){
        for(int i=13;i<19;i++)
            cout << rad2deg(robot->joint(i)->q()) << endl;
    }

    return 0;
}