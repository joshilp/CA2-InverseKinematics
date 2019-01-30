#ifndef _JOINT_H_
#define _JOINT_H_

#include <cassert>  
#include <string>
#include <iostream>
#include "Vectors.h"

typedef struct Target {
	double R;
	double theta;
	double phi;
	float x, y, z;
	Target() { R = 35.0f; theta = 0.0f; phi = 0.0f; x = -31.7568f; y = 18.1144f; z = 0.0f; }
} Target;

enum { CHILDSIZE = 16, NAMESIZE = 24 };

class Joint
{
public:

	Joint() :childNum(0), rotationOrder(0), rot_angle1(0.0f), rot_angle2(0.0f), rot_angle3(0.0f)
	{
		
		name[0] = 0;
		parent = nullptr;
		LocalPos = Vector4(0.0, 0.0, 0.0, 0.0);
		LocalQuat = Vector4(0.0, 0.0, 0.0, 1.0);
		Globalquat = Vector4(0.0, 0.0, 0.0, 1.0);
		GlobalXAxis = Vector4(1.0, 0.0, 0.0, 0.0);
		GlobalXAxis = Vector4(0.0, 1.0, 0.0, 0.0);
		GlobalXAxis = Vector4(0.0, 0.0, 1.0, 0.0);
	}
	bool addChild(Joint* pc)
	{
		assert(childNum != NAMESIZE);
		child[childNum] = pc;
		childNum++;
		return true;
	}
	Joint* getChild(int i)
	{
		assert(i >= 0 && i<CHILDSIZE);
		return child[i];
	}

	float rot_angle1, rot_angle2, rot_angle3;

	Vector4 LocalQuat; //local quaternion
	Vector4 LocalPos; //local position of the joint
	Vector4 Globalquat; //global rotation quaternion accumulated from parent
	Vector4 GlobalPos; //global position of the joint

	Vector4 GlobalXAxis, GlobalYAxis, GlobalZAxis;
	Joint* parent; //parent node
	Joint * child[CHILDSIZE]; //pointers to child nodes
	int childNum;
	std::string name;

	int rotationOrder;

private:
};

#endif
