#ifndef _FORWARDKINEMATICS_H_
#define _FORWARDKINEMATICS_H_

#include <cassert>  
#include"Joint.h"

class ForwardKinematics
{
public:
	ForwardKinematics() {}
	~ForwardKinematics() {}

	/*recursively calculate each joint's global position from the root (in-order traversal)*/
	void calculateJointPosWithQuaternion(Joint* joint);

	/*recursively calculate each joint's global position from the root (in-order traversal)*/
	void forwardKinematicsComputation(Joint* joint);

	//get local rotation quaternion with frame data and rotation order (euler angle order)
	Vector4 computeLocalQuaternion(Joint* joint);

	//get global rotation quaternion with local rotation quaternion
	Vector4 computeGlobalQuaternion(Joint* joint, Vector4 localQuat);

	//get global rotation quaternion with local rotation quaternion
	Vector4 computeGlobalPosition(Joint* joint);

	//calculate quaternion quat1*quat2
	Vector4 quaternionMultiplication(Vector4 quat1, Vector4 quat2);

	/*return quaternion specifying a rotaion with angle='angle',axis=(x,y,z)*/
	Vector4 buildQuaternionRotation(float angle, float x, float y, float z);

	Vector4 buildQuaternionRotationWithRad(float angle, float x, float y, float z)
	{
		float c = cosf(angle / 2.0f);    // cosine
		float s = sinf(angle / 2.0f);    // sine

		Vector4 rotationQuat = Vector4(s*x, s*y, s*z, c);
		return rotationQuat;
	}
};
#endif