#include "InverseKinematics.h"
#include "Vectors.h"
#include "Skeleton.h"
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Dense>

Eigen::Matrix<float, 1, 3> get_R(Vector3 E, Vector3 A);
Eigen::Matrix<float, 3, 3> get_A_cross_R(Eigen::Matrix<float, 1, 3> R, Joint* joint);
void update_Q(Joint* joint, Eigen::Matrix<float, 12, 1> theta, int index1, int index2, int index3);

ForwardKinematics* forwardKinematcis;

float degree2rad = 3.141592658f / 180.0f;

//IK workflow
void InverseKinematics::IK() {

	//check if the endeffector and the target are close enough
	Vector3 endPos = Vector3(endEffector->GlobalPos.x, endEffector->GlobalPos.y, endEffector->GlobalPos.z);
	Vector3 tarPos = Vector3(target->x, target->y, target->z);

	int i = 0;
	while ((endPos - tarPos).length() > threshold && i < iterNum)
	{
		if (mode == 0)
		{
			CCDMode();
		}
		else if (mode == 1)
		{
			JacobianMode();
		}
		endPos = Vector3(endEffector->GlobalPos.x, endEffector->GlobalPos.y, endEffector->GlobalPos.z);
		i++;
	}
}

//CCD Mode IK for right arm
void InverseKinematics::CCDMode()
{
	//add your code here
	//hint: Do forward kinematics when the endeffector's global position need to be updated
	//with newly computed quaternions. 

	Joint* c = endEffector->parent;

	while (c->name != base->parent->name)
	{
		Vector3 pe = Vector3(endEffector->GlobalPos.x, endEffector->GlobalPos.y, endEffector->GlobalPos.z);
		Vector3 pt = Vector3(target->x, target->y, target->z);
		Vector3 pc = Vector3(c->GlobalPos.x, c->GlobalPos.y, c->GlobalPos.z);

		Vector3 pe_pc = (pe - pc).normalize();
		Vector3 pt_pc = (pt - pc).normalize();

		float ctheta = pe_pc.dot(pt_pc);

		Vector3 r = pe_pc.cross(pt_pc);

		float x = r.x;
		float y = r.y;
		float z = r.z;

		Vector4 q = Vector4(x*0.5f, y*0.5f, z*0.5f, (ctheta + 1.0f)*0.5f);

		Vector4 old_global_q = c->Globalquat;
		Vector4 new_global_q = forwardKinematcis->quaternionMultiplication(q, old_global_q);

		c->Globalquat = new_global_q;

		forwardKinematcis->forwardKinematicsComputation(base);
		c = c->parent;
	}

	//I.e., use forwardKinematcis->forwardKinematicsComputation(base) 
	//whenever you need to update endeffector's global position.
}

//Entire Right Arm Jacobian IK
void InverseKinematics::JacobianMode()
{
	Vector3 E = Vector3(endEffector->GlobalPos.x, endEffector->GlobalPos.y, endEffector->GlobalPos.z);
	Vector3 T = Vector3(target->x, target->y, target->z);
	Vector3 T_E = 0.001*(T - E).normalize();
	//Vector3 T_E = T-E;

	Eigen::Matrix<float, 3, 1> P;
	P << T_E.x, T_E.y, T_E.z;

	Joint* jointA = endEffector->parent;
	Joint* jointB = jointA->parent;
	Joint* jointC = jointB->parent;
	Joint* jointD = base;

	Vector3 A = Vector3(jointA->GlobalPos.x, jointA->GlobalPos.y, jointA->GlobalPos.z);
	Vector3 B = Vector3(jointB->GlobalPos.x, jointB->GlobalPos.y, jointB->GlobalPos.z);
	Vector3 C = Vector3(jointC->GlobalPos.x, jointC->GlobalPos.y, jointC->GlobalPos.z);
	Vector3 D = Vector3(jointD->GlobalPos.x, jointD->GlobalPos.y, jointD->GlobalPos.z);

	Eigen::Matrix<float, 1, 3> Ra = get_R(E, A);
	Eigen::Matrix<float, 1, 3> Rb = get_R(E, B);
	Eigen::Matrix<float, 1, 3> Rc = get_R(E, C);
	Eigen::Matrix<float, 1, 3> Rd = get_R(E, D);

	Eigen::Matrix<float, 3, 3> mA = get_A_cross_R(Ra, jointA);
	Eigen::Matrix<float, 3, 3> mB = get_A_cross_R(Rb, jointB);
	Eigen::Matrix<float, 3, 3> mC = get_A_cross_R(Rc, jointC);
	Eigen::Matrix<float, 3, 3> mD = get_A_cross_R(Rd, jointD);

	Eigen::Matrix<float, 3, 12> J;
	J << mA, mB, mC, mD;

	float lambda = 0.01;

	Eigen::MatrixXf identity = Eigen::MatrixXf::Identity(3, 3);
	Eigen::MatrixXf Jt = J.transpose();
	Eigen::MatrixXf Jt_J = J * Jt;
	Eigen::MatrixXf Jt_J_Inv = (Jt_J + lambda * lambda * identity).inverse();
	Eigen::MatrixXf J_Inverse = Jt * Jt_J_Inv;

	Eigen::Matrix<float, 12, 1> theta = Jt * P;

	update_Q(jointA, theta, 0, 1, 2);
	update_Q(jointB, theta, 3, 4, 5);
	update_Q(jointC, theta, 6, 7, 8);
	update_Q(jointD, theta, 9, 10, 11);

	forwardKinematcis->forwardKinematicsComputation(base);
}

Eigen::Matrix<float, 1, 3> get_R(Vector3 E, Vector3 A)
{
	Eigen::Matrix<float, 1, 3> R;
	Vector3 EA = E - A;
	R << EA.x, EA.y, EA.z;
	return R;
}

Eigen::Matrix<float, 3, 3> get_A_cross_R(Eigen::Matrix<float, 1, 3> R, Joint* joint)
{
	Eigen::Matrix<float, 1, 3> Ax;
	Eigen::Matrix<float, 1, 3> Ay;
	Eigen::Matrix<float, 1, 3> Az;
	Eigen::Matrix<float, 3, 3> m;

	Ax << joint->GlobalXAxis.x, joint->GlobalXAxis.y, joint->GlobalXAxis.z;
	Ay << joint->GlobalYAxis.x, joint->GlobalYAxis.y, joint->GlobalYAxis.z;
	Az << joint->GlobalZAxis.x, joint->GlobalZAxis.y, joint->GlobalZAxis.z;

	Eigen::Matrix<float, 1, 3> AxR = Ax.cross(R);
	Eigen::Matrix<float, 1, 3> AyR = Ay.cross(R);
	Eigen::Matrix<float, 1, 3> AzR = Az.cross(R);

	m << AxR(0), AyR(0), AzR(0),
		AxR(1), AyR(1), AzR(1),
		AxR(2), AyR(2), AzR(2);

	return m;
}

void update_Q(Joint* joint, Eigen::Matrix<float, 12, 1> theta, int index1, int index2, int index3)
{
	Vector4 Q1 = forwardKinematcis->buildQuaternionRotationWithRad(theta(index1, 0), joint->GlobalXAxis.x, joint->GlobalXAxis.y, joint->GlobalXAxis.z);
	Vector4 Q2 = forwardKinematcis->buildQuaternionRotationWithRad(theta(index2, 0), joint->GlobalYAxis.x, joint->GlobalYAxis.y, joint->GlobalYAxis.z);
	Vector4 Q3 = forwardKinematcis->buildQuaternionRotationWithRad(theta(index3, 0), joint->GlobalZAxis.x, joint->GlobalZAxis.y, joint->GlobalZAxis.z);

	Vector4 mult1 = forwardKinematcis->quaternionMultiplication(Q2, Q1);
	Vector4 mult2 = forwardKinematcis->quaternionMultiplication(Q3, mult1);

	joint->Globalquat = forwardKinematcis->quaternionMultiplication(mult2, joint->Globalquat);
}