#include <cstdio>  
#include <cstdlib>  
#include "Skeleton.h"  



// OpenGL callback functions' declarations
void display();
void reshape(int w, int h);
void keyboard(unsigned char key, int x, int y);

//OpenGL initialize operations
void OpenGLinit(int argc, char* argv[]);

//create an instance of Skeleton class
Skeleton skeleton;

//create an instance of forward kinematic class
ForwardKinematics forwardKinematics;

//create an instance of inverse kinematic class
InverseKinematics inverseKinematics;

//target for inverse kinematics
Target target;

//Main enntry
int main(int argc, char *argv[])
{
	//OpenGL initialize operations
	OpenGLinit(argc, argv);

	//Load bvh file and reconstruct the skeleton
	skeleton.load_BVH("../data/running.bvh");

	//Print the loaded human skeleton
	skeleton.print();

	//initialize a pointer to the root of the skeleton
	Joint* root = skeleton.getRoot();

	//get the right end site joint
	Joint* endEffector = nullptr;
	skeleton.getJoint("R_Wrist_End", root, endEffector);

	//get the right shoulder joint
	Joint* rightShoulder = nullptr;
	skeleton.getJoint("RightShoulder", root, rightShoulder);

	//Initialize InverseKinematics instance
	inverseKinematics.initialize(endEffector, rightShoulder, &target);

	//Do Forward Kinematics once for generating the default T-Pose skeleton 
	forwardKinematics.calculateJointPosWithQuaternion(root);

	//While loop
	while (true)
	{
		//response to the keyboard event: move the target would call IK()
		glutMainLoopEvent();

		//re-display the new skeleton
		glutPostRedisplay();
	}

	return 0;
}

//OpenGL initilize operations
void OpenGLinit(int argc, char* argv[])
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowPosition(300, 75);
	glutInitWindowSize(800, 600);
	glutCreateWindow("Computer Animation - Inverse Kinematics");
	glShadeModel(GL_SMOOTH);
	glClearColor(0.0f, 1.0f, 1.0f, 0.0f); //initial background
	glEnable(GL_DEPTH_TEST);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutDisplayFunc(display);
}

//OpenGL call back function: display the skeleton for current frame
float viewingAngle = 45.0f;
void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt(180.0f*cosf(viewingAngle*3.141592658f / 180.0f), 100.0f, 180.0f*sinf(viewingAngle*3.141592658f / 180.0f), 0, 20, 0, 0, 1, 0);

	//draw floor
	glColor3f(1.0f, 1.0f, 0.0f);
	glBegin(GL_POLYGON);
	glVertex3f(500.0f, -100.0f, -500.0f);
	glVertex3f(500.0f, -100.0f, 500.0f);
	glVertex3f(-500.0f, -100.0f, 500.0f);
	glVertex3f(-500.0f, -100.0f, -500.0f);
	glEnd();


	glPushMatrix();
	glColor3f(1.0f, 0.0f, 0.0f);
	glTranslatef(target.x, target.y, target.z);
	glutSolidSphere(2.0f, 5.0f, 5.0f);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(0.0f, 0.0f, 70.0f);
	glBegin(GL_LINES);
	glVertex3f(0.0f,0.0f,0.0f);
	glVertex3f(10.0f,0.0f,0.0f);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 10.0f, 0.0f);

	glColor3f(0.0f,0.0f,1.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(00.0f, 0.0f, 10.0f);
	glEnd();
	glPopMatrix();


	skeleton.draw();
	glutSwapBuffers();
}

//OpenGL call back fucntion: reshape display window function
void reshape(int w, int h)
{
	glViewport(0, 0, GLsizei(w), GLsizei(h));
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, (GLdouble)w / (GLdouble)h, 1.0f, 1000.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(200.0f*cos(10.0f), 0, 200.0f*sin(10.0f), 0, 0, 0, 0, 1, 0);
}

//OpenGL call back function: kepboard responsing functiion: once w is pressed on keyboard, one more frame data is loaded
void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		exit(0);
	case 'm': {
		viewingAngle = viewingAngle + 1.0f;
		if (viewingAngle >= 360.0f)
		{
			viewingAngle = 0.0f;
		}
		break;
	}
	case 'n': {
		viewingAngle = viewingAngle - 1.0f;
		if (viewingAngle <= -360.0f)
		{
			viewingAngle = 0.0f;
		}
		break;
	}
	case 'c': {inverseKinematics.setMode(0); break; }
	case 'j': {inverseKinematics.setMode(1); break; }
	case 'r': {target.x = -31.7568f; target.y = 18.1144f; target.z = 0; inverseKinematics.reset(); break; }
	case 'w':
	case 'W':
		skeleton.addFrame();
		break;
	case '1': {
		target.x = target.x + 1.0f;
		break;
	}
	case '2': {
		target.x = target.x - 1.0f;
		break;
	}
	case '3': {
		target.y = target.y + 1.0f;
		break;
	}
	case '4': {
		target.y = target.y - 1.0f;
		break;
	}
	case '5': {
		target.z = target.z + 1.0f;
		break;
	}
	case '6': {
		target.z = target.z - 1.0f;
		break;
	}
	}

	if (key=='1'|| key == '2' || key == '3' || key == '4' || key == '5' || key == '6')
	{
		inverseKinematics.IK();
	}
	
	//glutPostRedisplay();

}
