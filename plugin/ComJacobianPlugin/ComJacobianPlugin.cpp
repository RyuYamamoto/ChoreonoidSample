#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/Jacobian>
#include <cnoid/BodyLoader>
#include <cnoid/Link>

#include <Eigen/Core>

using namespace std;
using namespace cnoid;
using namespace Eigen;

class ComJacobianPlugin : public Plugin
{
public:
    
    ComJacobianPlugin() : Plugin("ComJacobianTest")
    {;
        require("Body");
    }
    
    virtual bool initialize()
    {
        ToolBar* bar = new ToolBar("ComJacobianTest");
        bar->addButton("Increment")
            ->sigClicked().connect(bind(&ComJacobianPlugin::onButtonClicked, this, 0.02));
        bar->addButton("Decrement")
                ->sigClicked().connect(bind(&ComJacobianPlugin::onButtonClicked, this, -0.02));
        addToolBar(bar);

        return true;
    }

    void onButtonClicked(double ref_com_pos)
    {
        ItemList<BodyItem> bodyItems =
            ItemTreeView::mainInstance()->selectedItems<BodyItem>();

		const double ik_gain = 0.5;
		const int iteration = 100;
		const double erreps = 1e-06;
		ColPivHouseholderQR<MatrixXd> QR;
		const double ik_lambda = 1.0e-12;

		BodyPtr robot = bodyItems[0]->body();

		JointPathPtr leg = getCustomJointPath(robot, robot->link("RLEG_ANKLE_R"), robot->link("LLEG_ANKLE_R"));

		// 順運動学でリンクの位置姿勢更新
		robot->calcForwardKinematics();

		// ワールド座標系における重心位置を更新
		Vector3d cur_com(robot->calcCenterOfMass());
		Vector3d ref_com(cur_com);
		ref_com.y() += ref_com_pos;

		// 目標関節角度の差分
		VectorXd dq(robot->numJoints());
		// 収束ループ
		for(int n=0;n<iteration;n++)
		{
			if(ref_com.dot(ref_com) < erreps) break;
			// 重心ヤコビアン
			MatrixXd J_com(3, robot->numJoints());

			// 重心ヤコビアンを計算
			calcCMJacobian(robot, leg->baseLink(), J_com);

			MatrixXd JJ = J_com * J_com.transpose() + ik_lambda*MatrixXd::Identity(J_com.rows(), J_com.rows());
			dq = J_com.transpose() * QR.compute(JJ).solve(ref_com - cur_com) * ik_gain;
		}
        
        for(size_t i=0; i < bodyItems.size(); ++i){
            for(int j=0; j < robot->numJoints(); ++j)
                robot->joint(j)->q() += dq[j];
            bodyItems[i]->notifyKinematicStateChange(true);
        }
    }
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(ComJacobianPlugin)
