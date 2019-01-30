#include "ForwardKinematics.h"


void ForwardKinematics::forwardKinematicsComputation(Joint* joint)
{
	//joint->Globalquat = computeGlobalQuaternion(joint, joint->LocalQuat);

	joint->GlobalPos = computeGlobalPosition(joint);

	//recursively call self
	for (int i = 0; i < joint->childNum; i++)
	{
		forwardKinematicsComputation(joint->getChild(i));
	}
}


/*recursively calculate each joint's global position from the root (in-order traversal)*/
void ForwardKinematics::calculateJointPosWithQuaternion(Joint* joint)
{
	/*----Coding Part Start: Please modify the code accordingly inside these sub-functions-----*/
	/*Coding Part: 1) calculate current node's global position*/
	joint->GlobalPos = computeGlobalPosition(joint);

	/*Coding Part: 2) calculate local rotation in quaternion from euler angles for currently accessed node*/
	joint->LocalQuat = computeLocalQuaternion(joint);

	/*Coding Part: 3) calculate global rotation quaternion for child joints*/
	joint->Globalquat = computeGlobalQuaternion(joint, joint->LocalQuat);
	/*------------------------------coding part end-------------------------------------------*/

	//recursively call self
	for (int i = 0; i < joint->childNum; i++)
	{
		calculateJointPosWithQuaternion(joint->getChild(i));
	}
}

//compute joint's global position with its local position and its parent joint's 
//global position and orientation
Vector4 ForwardKinematics::computeGlobalPosition(Joint* joint)
{
	Vector4 parentGlobalQuat = Vector4(0.0f, 0.0f, 0.0f, 1.0f);
	Vector4 parentGlobalPos = Vector4(0.0f, 0.0f, 0.0f, 0.0f);

	if (joint->parent != nullptr)
	{
		parentGlobalQuat = joint->parent->Globalquat;
		parentGlobalPos = joint->parent->GlobalPos;
	}

	Vector4 parentInverseQuat = Vector4(-parentGlobalQuat.x, -parentGlobalQuat.y, -parentGlobalQuat.z, parentGlobalQuat.w);
	Vector4 LocalPosition = quaternionMultiplication(joint->LocalPos, parentInverseQuat);
	LocalPosition = quaternionMultiplication(parentGlobalQuat, LocalPosition);
	Vector4 GlobalPosition = parentGlobalPos + LocalPosition;

	joint->GlobalPos = GlobalPosition;
	return GlobalPosition;

	//default return
	//Vector4 GlobalPos = joint->LocalPos;
	//return GlobalPos;
}


//get local rotation quaternion with frame data and rotation order (euler angle order)
Vector4 ForwardKinematics::computeLocalQuaternion(Joint* joint)
{
	Vector4 quat1, quat2, quat3;

	//initialize with identity quaternion
	quat1.w = 1.0f;
	quat2.w = 1.0f;
	quat3.w = 1.0f;

	quat1 = buildQuaternionRotation(joint->rot_angle1, 1.0f, 0.0f, 0.0f);
	quat2 = buildQuaternionRotation(joint->rot_angle2, 0.0f, 1.0f, 0.0f);
	quat3 = buildQuaternionRotation(joint->rot_angle3, 0.0f, 0.0f, 1.0f);

	//euler angle order with intrinsic order (inverse fixed-angle order): quat1*quat2*quat3
	Vector4 quatMultiply1 = quaternionMultiplication(quat1, quat2);
	Vector4 quatMultiply2 = quaternionMultiplication(quatMultiply1, quat3);

	return quatMultiply2;

	//default return
	//return Vector4(0.0f, 0.0f, 0.0f, 1.0f);
}

//get global rotation quaternion with local rotation quaternion
Vector4 ForwardKinematics::computeGlobalQuaternion(Joint* joint, Vector4 localQuat)
{
	Vector4 ParentGlobalQuat = Vector4(0.0f, 0.0f, 0.0f, 1.0f);
	if (joint->parent != nullptr)
	{
		ParentGlobalQuat = joint->parent->Globalquat;
	}

	return quaternionMultiplication(ParentGlobalQuat, localQuat);

	//default returnn
	//return localQuat;
}



//calculate quaternion quat1*quat2
Vector4 ForwardKinematics::quaternionMultiplication(Vector4 quat1, Vector4 quat2)
{
	Vector3 v1 = Vector3(quat1.x, quat1.y, quat1.z);
	Vector3 v2 = Vector3(quat2.x, quat2.y, quat2.z);
	float x1 = quat1.x; float y1 = quat1.y; float z1 = quat1.z;
	float x2 = quat2.x; float y2 = quat2.y; float z2 = quat2.z;

	float w1 = quat1.w;
	float w2 = quat2.w;

	Vector3 v3 = w1 * v2 + w2 * v1 + v1.cross(v2);
	float w3 = w1 * w2 - v1.dot(v2);
	Vector4 result = Vector4(v3.x, v3.y, v3.z, w3);

	return result;
}

/*return quaternion specifying a rotaion with angle='angle',axis=(x,y,z)*/
Vector4 ForwardKinematics::buildQuaternionRotation(float angle, float x, float y, float z)
{
	float degreetorad = 3.141592658f / 180.0f;
	float c = cosf((angle / 2.0f) * degreetorad);    // cosine
	float s = sinf((angle / 2.0f) * degreetorad);    // sine

	Vector4 rotationQuat = Vector4(s*x, s*y, s*z, c);
	return rotationQuat;
}
