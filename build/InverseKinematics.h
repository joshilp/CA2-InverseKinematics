#ifndef _INVERSEKINEMATICS_H_
#define _INVERSEKINEMATICS_H_

#include <cassert>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include"Matrices.h"
#include"ForwardKinematics.h"

class InverseKinematics
{
public:
	InverseKinematics() { mode = 0; iterNum = 300; threshold = 0.5f; }
	~InverseKinematics() {}

	
	ForwardKinematics* forwardKinematcis;

	inline void initialize(Joint* end, Joint* rotJ, Target* tar) {endEffector = end; base = rotJ; target = tar;}	
	
	inline void setMode(int mod) { mode = mod; }

	//IK workflow
	void IK();

	//CCD IK
	void CCDMode();

	//Entire Right Arm Jacobian IK
	void JacobianMode();

	//reset the skeleton to T-Pose and target to the default postion
	void reset() {
		Joint* temp = endEffector;
		Vector4 identity = Vector4(0.0f, 0.0f, 0.0f, 1.0f);
		while (temp->name!= base->parent->name)
		{
			temp->Globalquat = identity;
			temp->LocalQuat = identity;
			temp = temp->parent;
		}
		forwardKinematcis->calculateJointPosWithQuaternion(base);
	}

private:
	Joint* endEffector;
	Joint* base;
	Target* target;

	int mode;
	int iterNum;
	float threshold;
};
#endif