#define MAIN
#ifdef MAIN

//#define HillClimbing
#define GradientDescent
//#define GradientDescent_Jacobian

#include <vgl.h>
#include <InitShader.h>
#include <mat.h>
#include "MyCube.h"
#include "MyPyramid.h"
#include "MyTarget.h"

MyCube cube;
MyPyramid pyramid;
MyTarget target(&cube);

GLuint program;
GLuint uMat;

mat4 CTM;

bool bPlay = false;
bool bChasingTarget = false;
bool bDrawTarget = false;

float ang1 = 0;
float ang2 = 0;
float ang3 = 0;

const float PI = 3.141592;
const float DEG2RAD = PI / 180.0;
const float RAD2DEG = 180.0 / PI;

void myInit()
{
	cube.Init();
	pyramid.Init();

	program = InitShader("vshader.glsl", "fshader.glsl");
	glUseProgram(program);
}

float g_time = 0;

void drawRobotArm(float ang1, float ang2, float ang3)
{
	mat4 temp = CTM;
	mat4 M(1.0);

	//BASE1
	M = Translate(0, 0, 0.075) * Scale(0.3, 0.3, 0.05);
	glUniformMatrix4fv(uMat, 1, true, CTM * M);
	pyramid.Draw(program);

	//BASE2
	M = Translate(0, 0, -0.075) * Scale(0.3, 0.3, 0.05);
	glUniformMatrix4fv(uMat, 1, true, CTM * M);
	pyramid.Draw(program);

	// BASE-UpperArm Bolt1
	M = Translate(0, 0, 0.085) * Scale(0.05, 0.05, 0.05);
	glUniformMatrix4fv(uMat, 1, true, CTM * M);
	cube.Draw(program);

	// BASE-UpperArm Bolt2
	M = Translate(0, 0, -0.085) * Scale(0.05, 0.05, 0.05);
	glUniformMatrix4fv(uMat, 1, true, CTM * M);
	cube.Draw(program);

	// Upper Arm
	CTM *= RotateZ(ang1);
	M = Translate(0, 0.2, 0) * Scale(0.1, 0.5, 0.1);
	glUniformMatrix4fv(uMat, 1, true, CTM * M);
	cube.Draw(program);

	CTM *= Translate(0, 0.4, 0) * RotateZ(ang2);
	// Lower Arm1
	M = Translate(0, 0.2, 0.075) * Scale(0.1, 0.5, 0.05);
	glUniformMatrix4fv(uMat, 1, true, CTM * M);
	cube.Draw(program);

	// Lower Arm2
	M = Translate(0, 0.2, -0.075) * Scale(0.1, 0.5, 0.05);
	glUniformMatrix4fv(uMat, 1, true, CTM * M);
	cube.Draw(program);

	// UpperArm-LowerArm Bolt1
	M = Translate(0, 0, 0.085) * Scale(0.05, 0.05, 0.05);
	glUniformMatrix4fv(uMat, 1, true, CTM * M);
	cube.Draw(program);

	// UpperArm-LowerArm Bolt2
	M = Translate(0, 0, -0.085) * Scale(0.05, 0.05, 0.05);
	glUniformMatrix4fv(uMat, 1, true, CTM * M);
	cube.Draw(program);

	// Hand
	CTM *= Translate(0, 0.4, 0) * RotateZ(ang3);
	M = Translate(0, 0, 0) * Scale(0.4, 0.15, 0.1);
	glUniformMatrix4fv(uMat, 1, true, CTM * M);
	cube.Draw(program);

	// UpperArm-LowerArm Bolt1
	M = Translate(0, 0, 0.085) * Scale(0.05, 0.05, 0.05);
	glUniformMatrix4fv(uMat, 1, true, CTM * M);
	cube.Draw(program);

	// UpperArm-LowerArm Bolt2
	M = Translate(0, 0, -0.085) * Scale(0.05, 0.05, 0.05);
	glUniformMatrix4fv(uMat, 1, true, CTM * M);
	cube.Draw(program);

	CTM = temp;
}

vec4 handend_GetPosition(float ang1, float ang2, float ang3)
{
	mat4 mat(1.0);
	mat *= RotateZ(ang1);
	mat *= Translate(0, 0.4, 0) * RotateZ(ang2);
	mat *= Translate(0, 0.4, 0) * RotateZ(ang3);
	mat *= Translate(0.2, 0, 0);
	return mat * vec4(0, 0, 0, 1);
}

float computeDist(vec4 v1, vec4 v2)
{
	return length(v1 - v2);
}

float normalizedAngle(float ang)
{
	ang = fmod(ang + 360.0, 360.0);
	return ang;
}

#ifdef GradientDescent_Jacobian
void computeAngle()
{
	vec4 target_pos = vec4(target.GetPosition(g_time), 1.0); //t
	vec4 hand_pos(1.0); //X

	float L1 = 0.4, L2 = 0.4, L3 = 0.2;
	float rate = 0.1;
	int iter = 200;
	float threshold = 0.01;

	for (int i = 0; i < iter; i++)
	{
		hand_pos = handend_GetPosition(ang1, ang2, ang3);
		vec4 error = hand_pos - target_pos;
		if (length(error) < threshold)
			return;

		float rad1 = ang1 * DEG2RAD;
		float rad2 = ang2 * DEG2RAD;
		float rad3 = ang3 * DEG2RAD;

		mat4 J(0.0); //dX/d¥è (col-major)
		//col0 (dX/d¥è1)
		J[0][0] = -L1 * sin(rad1) - L2 * sin(rad1 + rad2) - L3 * sin(rad1 + rad2 + rad3);
		J[0][1] = L1 * cos(rad1) + L2 * cos(rad1 + rad2) + L3 * cos(rad1 + rad2 + rad3);
		//col1 (dX/d¥è2)
		J[1][0] = -L2 * sin(rad1 + rad2) - L3 * sin(rad1 + rad2 + rad3);
		J[1][1] = L2 * cos(rad1 + rad2) + L3 * cos(rad1 + rad2 + rad3);
		//col2 (dX/d¥è3)
		J[2][0] = -L3 * sin(rad1 + rad2 + rad3);
		J[2][1] = L3 * cos(rad1 + rad2 + rad3);

		mat4 J_T = transpose(J);
		vec4 grad = J_T * error;

		ang1 -= rate * 10.0 * grad.x;
		ang2 -= rate * 5.0 * grad.y;
		ang3 -= rate * 1.0 * grad.z;
	}
}
#endif

#ifdef GradientDescent
void computeAngle()
{
	vec4 target_pos = vec4(target.GetPosition(g_time), 1.0);
	vec4 hand_pos(1.0);

	float L1 = 0.4, L2 = 0.4, L3 = 0.2;
	float rate = 0.1;
	int iter = 200;
	float threshold = 0.01;

	for (int i = 0; i < iter; i++)
	{
		hand_pos = handend_GetPosition(ang1, ang2, ang3);

		float dist = computeDist(target_pos, hand_pos);
		if (dist < threshold)
			return;

		vec4 error = hand_pos - target_pos;
		float dE_dx = error.x;
		float dE_dy = error.y;

		float rad1 = ang1 * DEG2RAD;
		float rad2 = ang2 * DEG2RAD;
		float rad3 = ang3 * DEG2RAD;

		//ang1 gradient
		float dx_dang1 = -L1 * sin(rad1) - L2 * sin(rad1 + rad2) - L3 * sin(rad1 + rad2 + rad3);
		float dy_dang1 = L1 * cos(rad1) + L2 * cos(rad1 + rad2) + L3 * cos(rad1 + rad2 + rad3);
		float dE_dang1 = dE_dx * dx_dang1 + dE_dy * dy_dang1;

		//ang2 gradient
		float dx_dang2 = -L2 * sin(rad1 + rad2) - L3 * sin(rad1 + rad2 + rad3);
		float dy_dang2 = L2 * cos(rad1 + rad2) + L3 * cos(rad1 + rad2 + rad3);
		float dE_dang2 = dE_dx * dx_dang2 + dE_dy * dy_dang2;

		//ang3 gradient
		float dx_dang3 = -L3 * sin(rad1 + rad2 + rad3);
		float dy_dang3 = L3 * cos(rad1 + rad2 + rad3);
		float dE_dang3 = dE_dx * dx_dang3 + dE_dy * dy_dang3;

		ang1 -= rate * 10.0 * dE_dang1 * RAD2DEG; ang1 = normalizedAngle(ang1);
		ang2 -= rate * 5.0 * dE_dang2 * RAD2DEG; ang2 = normalizedAngle(ang2);
		ang3 -= rate * 1.0 * dE_dang3 * RAD2DEG; ang3 = normalizedAngle(ang3);
	}
}
#endif

#ifdef HillClimbing
void computeAngle()
{
	static float threshold = 0.001;
	vec4 target_pos = vec4(target.GetPosition(g_time), 1.0);

	int n_iter = 90;
	for (int k = 0; k < n_iter; k++)
	{
		vec4 hand_pos = handend_GetPosition(ang1, ang2, ang3);
		float dist0 = computeDist(target_pos, hand_pos);
		if (dist0 < threshold)
			return;

		float min_dist = dist0;
		float dtheta = 0;
		int angleNum = 0;

		vec4 temp_hand_pos;
		float temp_dist;

		float angstep = 0.1;
		for (float i = -angstep; i <= angstep+threshold; i += 2*angstep)
		{
			//change ang1
			temp_hand_pos = handend_GetPosition(ang1 + i, ang2, ang3);
			temp_dist = computeDist(target_pos, temp_hand_pos);
			if (temp_dist < min_dist)
			{
				min_dist = temp_dist;
				angleNum = 1;
				dtheta = i;
			}

			//change ang2
			temp_hand_pos = handend_GetPosition(ang1, ang2 + i, ang3);
			temp_dist = computeDist(target_pos, temp_hand_pos);
			if (temp_dist < min_dist)
			{
				min_dist = temp_dist;
				angleNum = 2;
				dtheta = i;
			}

			//change ang3
			temp_hand_pos = handend_GetPosition(ang1, ang2, ang3 + i);
			temp_dist = computeDist(target_pos, temp_hand_pos);
			if (temp_dist < min_dist)
			{
				min_dist = temp_dist;
				angleNum = 3;
				dtheta = i;
			}
		}

		if (angleNum == 1)
		{
			ang1 += dtheta;
			ang1 = normalizedAngle(ang1);
		}
		else if (angleNum == 2)
		{
			ang2 += dtheta;
			ang2 = normalizedAngle(ang2);
		}
		else if (angleNum == 3)
		{
			ang3 += dtheta;
			ang3 = normalizedAngle(ang3);
		}
		else;
		glutPostRedisplay();
	}
}
#endif

void myDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	GLuint uColor = glGetUniformLocation(program, "uColor");
	glUniform4f(uColor, -1, -1, -1, -1);


	uMat = glGetUniformLocation(program, "uMat");
	CTM = Translate(0, -0.4, 0) * RotateY(g_time * 30);
	drawRobotArm(ang1, ang2, ang3);


	glUniform4f(uColor, 1, 0, 0, 1);
	if (bDrawTarget == true)
		target.Draw(program, CTM, g_time);

	glutSwapBuffers();
}

void myIdle()
{
	if (bPlay)
	{
		g_time += 1 / 60.0f;
		Sleep(1 / 60.0f * 1000);

		if (bChasingTarget == false)
		{
			ang1 = 45 * sin(g_time * 3.141592);
			ang2 = 60 * sin(g_time * 2 * 3.141592);
			ang3 = 30 * sin(g_time * 3.141592);
		}
		else
		{
			computeAngle();
		}

		glutPostRedisplay();
	}
}

void myKeyboard(unsigned char c, int x, int y)
{

	switch (c)
	{
	case '1':
		bChasingTarget = !bChasingTarget;
		break;
	case '2':
		bDrawTarget = !bDrawTarget;
		break;
	case '3':
		target.toggleRandom();
		break;
	case ' ':
		bPlay = !bPlay;
		break;
	default:
		break;
	}
}


int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(500, 500);
	glutCreateWindow("Assignment#2");

	glewExperimental = true;
	glewInit();

	myInit();
	glutDisplayFunc(myDisplay);
	glutKeyboardFunc(myKeyboard);
	glutIdleFunc(myIdle);

	glutMainLoop();

	return 0;
}

#endif