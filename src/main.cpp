#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

#include <OpenGL/gl.h>
#include <OpenGL/glu.h>

#include <GLUT/glut.h>
#pragma comment(lib, "glut32.lib")

#include <iostream>

using namespace std;

void recordingLoop( void *dummy );
int recording;
int iGW;
int frame, timeget, timebase;
double fps;

#include "ProbCon2.h"
int main(int argc, char* argv[])
{   
 	// Check for proper syntax
	modeShowPotential = true;
 	if (argc < 3) {
 		// Keep going
 	}
	else if (argc == 3) {
 	} 
 	else if (argc == 4) {
 	}
 	else if (argc == 5) {
 	}
 	

	timebase = 0;
	InitProbCon();

	kill = 0;
	recording = 0;


	printf("Press R to record data\n");
	tp = 0;

	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize (600, 600);
	glutInitWindowPosition (0, 0);
	mainWindow = glutCreateWindow ("Probcon");
	init();
	glutKeyboardFunc(processNormalKeys);
	glutDisplayFunc(MainLoop);
	glutMainLoop();

	fprintf(stderr, "\n\nDone 2 !!! :D.\n");     
	return 0;
}

double dTime;

void exitProgVK()
{
	kill = 1;
	//Sleep(200);
	freeAllMem();

	exit(0);
}

void MainLoop()
{
	glClear (GL_COLOR_BUFFER_BIT);

	glColor3f (1.0, 1.0, 1.0);

	glColor3f (0.2, 0.2, 0.2);
	drawGrid();
	glColor3f (1.0, 1.0, 1.0);

	LogicLoop();

	//glColor3f (0.0, 0.8, 0.0);
	//drawCircle((Gx/100+0.5)*NCX,(Gy/100+0.5)*NCY);

	frame++;
	timeget=glutGet(GLUT_ELAPSED_TIME);
	if(timeget-timebase>1000)
	{
		fps= (double)frame*1000.0/(double)(timeget-timebase);
		timebase=timeget;
		frame=0;
		//printf("\n%f",fps);
	}

	glutSwapBuffers();
	glutPostRedisplay();
}


void TimePlot()
{
	glClear (GL_COLOR_BUFFER_BIT);
    
	glColor3f (1.0, 1.0, 1.0);
	
    glColor3f (0.2, 0.2, 0.2);
	drawGrid();
    glColor3f (1.0, 1.0, 1.0);
	
	LogicLoop();

	glColor3f (0.0, 0.8, 0.0);
	
	
	glutSwapBuffers();
	glutPostRedisplay();
}
