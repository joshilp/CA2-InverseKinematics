#ifndef _SKELETON_H_
#define _SKELETON_H_

#include <cassert>  
#include <cstdio>  
#include <cstdlib>  
#include <vector>
#include <stack>

#include <GL\freeglut.h>
#include "ForwardKinematics.h"
#include "InverseKinematics.h"



class Skeleton
{
public:
	Skeleton() :currentNode(nullptr), p(nullptr), root(nullptr), frameCount(0), frameData(nullptr), pFrame(nullptr)
		, frameTime(1), jointCount(1), currentFrame(0) {}
	~Skeleton() { clear(); }

	void clear();
	void addFrame() { currentFrame++; if (currentFrame == frameCount) currentFrame = 0; } //load a new frame data to pFrame
	void print() { printRecursive(root, 0); } //print the loaded skeleton structure
	void draw(); //draw a skeleton with one frame data

	bool load_BVH(const char * pfile); //load the bvh file
	unsigned getFrameCount() { return frameCount; }; //return total # of frames
	float* getCurrentFrame() { return frameData[currentFrame]; }
	void loadCurrentFrame() { loadFrameDataRecursively(root); }
	void loadFrameDataRecursively(Joint* joint)
	{
		if (joint->parent == nullptr)
		{
			pFrame = frameData[currentFrame];
			joint->LocalPos.x = pFrame[0];
			joint->LocalPos.y = pFrame[1];
			joint->LocalPos.z = pFrame[2];
			pFrame = pFrame + 3;
		}


		switch (joint->rotationOrder)
		{
		case 1:
			joint->rot_angle1 = pFrame[0];
			joint->rot_angle2 = pFrame[1];
			joint->rot_angle3 = pFrame[2];
			pFrame = pFrame + 3;
			break;
		default:
			break;
		}

		for (size_t i = 0; i < joint->childNum; i++)
		{
			loadFrameDataRecursively(joint->getChild(i));
		}

	}

	Joint* getRoot() { return root; }
	void getJoint(std::string name,Joint* root,Joint*& result){ 

		if (root->name==name)
		{
			result=root;
		}

		for (size_t i = 0; i < root->childNum; i++)
		{
			getJoint(name, root->getChild(i),result);
		}
	}


private:

	//variables for loadingn the skeleton and frame data
	Joint * root;//root node   
	Joint* currentNode; //current node
	float **frameData; //all the frame data
	float frameTime; //total frame time
	float *pFrame;//current frame data
	unsigned currentFrame;//current frame
	unsigned jointCount;//joint num. including root  
	unsigned frameCount;// frame num. 
	unsigned char* p;//current buffer index 
	std::stack<Joint*> father; //stack structure for buiding the tree skeleton tree structure

							   //display buffers: to-be-determined before displaying one frame
	std::vector<Vector4> Points; //Points' display buffer
	std::vector<std::vector<Vector4>> LocalCoorSys; //LocalCoorSys's display buffer
	std::vector<std::vector<Vector4>> Bones; //Bones' display buffer


											 //functions
	void calculateDisplayBuff(Joint* joint)
	{
		
		/*calculate global position*/
		Points.push_back(joint->GlobalPos);

		/*calculate local coordinate syste*/
		LocalCoorSys.push_back(calculateLocalCoorSys(joint));

		/*calculate bones*/
		if (joint->parent != nullptr)
		{
			std::vector<Vector4> bone;
			Vector4 start = joint->parent->GlobalPos;
			Vector4 end = joint->GlobalPos;
			bone.push_back(start);
			bone.push_back(end);
			Bones.push_back(bone);
		}

		for (size_t i = 0; i < joint->childNum; i++)
		{
			calculateDisplayBuff(joint->getChild(i));
		}

	}
	void deleteRecursive(Joint* r); //delete the tree structure
	void printRecursive(Joint* r, int n); //print the skeleton
	bool loadHiarachy(unsigned char * buffer); //load the skeleton structure as a tree
	bool loadFrameData(unsigned char * buffer); //load the frame data

												//get current node's rotation order, XYZ:1, End Site:0
	int getRotationOrder() {
		if (*p == 'X'&*(p + 10) == 'Y'&*(p + 20) == 'Z')
		{
			return 1;
		}
		return 0;
	}

	std::vector<Vector4> calculateLocalCoorSys(Joint* root);
	Joint * createJoint(); //create a new joint
};
#endif // !_SKELETON_H_
