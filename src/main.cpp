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


//#include <opencv2/imgproc/imgproc_c.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <stdio.h>
//#include "cbw.h"

//#pragma comment(lib, "hd.lib")
//#pragma comment(lib, "hdu.lib")

//#pragma comment(lib, "opencv_core242d.lib")
//#pragma comment(lib, "opencv_highgui242d.lib")
//#pragma comment(lib, "opencv_imgproc242d.lib")
//#pragma comment(lib, "cbw32.lib")
//Sound
//#pragma comment(lib, "fmodex_vc.lib")

//#pragma comment(lib, "opencv_features2d241d.lib")
//#pragma comment(lib, "opencv_contrib241d.lib")
//#pragma comment(lib, "opencv_legacy241d.lib")
//#pragma comment(lib, "opencv_flann241d.lib")
//#pragma comment(lib, "opencv_features2d241d.lib")
//#pragma comment(lib, "opencv_calib3d241d.lib")


//HANDLE hIOMutex;
////This is our Robot/Recording Mutex handle
//
//
//#define USE_CAMERA

void recordingLoop( void *dummy );
int recording;

//#ifdef USE_CAMERA

//CvFont myFonto;
//
//IplImage* leftImg;
//IplImage* binLeftImgGreen;
//IplImage* binLeftImgCyan;
//IplImage* binLeftImgSmallGreen;
//IplImage* binLeftImgSmallCyan;
//IplImage* leftImgHSV;
//CvCapture* leftCapture;
//
//IplImage* rightImg;

//#endif

#include "ProbCon2.h"

//#ifdef USE_CAMERA


////Green	
//CvScalar hsvMinGreen;
//CvScalar hsvMaxGreen;
//	
////Cyan
//CvScalar hsvMinCyan;
//CvScalar hsvMaxCyan;
//
//
//int gMedianGreen;
//int gMedianCyan;
//
//char textLeftLine[6][50];
//char textRightLine[6][50];	//??? Add array Xb
//
//HANDLE hMutexCamera;	//This is our Camera Mutex handle
//void cameraLoop( void *dummy  );
//
//#endif

int iGW;

int frame, timeget, timebase;
double fps;


    

//float testArray[100][100];
int main(int argc, char* argv[])
{   
//#ifdef USE_CAMERA
//	FILE *cfg;
//
//	//cfg = fopen("C:\\JohnConfigs\\NIPSReward","r");
//	cfg = fopen("NIPSReward","r");
//	fscanf(cfg,"\t%lf\t%lf\t%lf\n",&hsvMinGreen.val[0],&hsvMinGreen.val[1],&hsvMinGreen.val[2]);
//	fscanf(cfg,"\t%lf\t%lf\t%lf\n",&hsvMaxGreen.val[0],&hsvMaxGreen.val[1],&hsvMaxGreen.val[2]);
//	fscanf(cfg,"\t%d\n",&gMedianGreen);
//	fclose(cfg);
//		
//	//cfg = fopen("C:\\JohnConfigs\\NIPSPenalty","r");
//	cfg = fopen("NIPSPenalty","r");
//	fscanf(cfg,"\t%lf\t%lf\t%lf\n",&hsvMinCyan.val[0],&hsvMinCyan.val[1],&hsvMinCyan.val[2]);
//	fscanf(cfg,"\t%lf\t%lf\t%lf\n",&hsvMaxCyan.val[0],&hsvMaxCyan.val[1],&hsvMaxCyan.val[2]);
//	fscanf(cfg,"\t%d\n",&gMedianCyan);
//	fclose(cfg);
//#endif

	timebase = 0;
	//PROB-CON
	InitProbCon();

	//ROBOT STUFF
//#ifndef SIMULATION_ONLY
	//hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	////hHD = hdInitDevice("OMNI Phantom");
//#endif

	////RUNNING AVG VEL
	//ZeroMemory(velSmoothX,FILT_LEN*sizeof(double));
	//ZeroMemory(velSmoothY,FILT_LEN*sizeof(double));

	////RUNNING AVG VEL
	//ZeroMemory(difSmoothX,FILT_DIF*sizeof(double));
	//ZeroMemory(difSmoothY,FILT_DIF*sizeof(double));


	//hIOMutex = CreateMutex(NULL, FALSE, NULL);
	kill = 0;
	recording = 0;

	//SET INITIAL DESIRED POSITION
	//memset(wellPos2 , 0.0,sizeof(hduVector3Dd));

//#ifndef SIMULATION_ONLY
//	if (HD_DEVICE_ERROR(error = hdGetError())) 
//	{
//		hduPrintError(stderr, &error, "Failed to initialize haptic device");
//		fprintf(stderr, "\nPress any key to quit.\n");
//		getch();
//		return -1;
//	}
//
//	printf("Found device model: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));
//
//
//	hGravityWell = hdScheduleAsynchronous(gravityWellCallback, 0, HD_MAX_SCHEDULER_PRIORITY);
//
//	hdEnable(HD_FORCE_OUTPUT);
//
//	hdStartScheduler();
//	if (HD_DEVICE_ERROR(error = hdGetError()))
//	{
//		hduPrintError(stderr, &error, "Failed to start scheduler");
//		fprintf(stderr, "\nPress any key to quit.\n");
//		return -1;
//	}
//#endif

	//_beginthread( recordingLoop,0,NULL );


	printf("Press R to record data\n");
	//tim = 0;
	tp = 0;


//#ifdef USE_CAMERA
//	leftCapture = cvCaptureFromCAM(1);
//	if (leftCapture == NULL) 
//	{
//		printf("\nLeft Camera not found!.\n");
//		getch();
//		return -2;
//	}
//
//	binLeftImgGreen = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 1);
//	binLeftImgCyan = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 1);
//	binLeftImgSmallGreen = cvCreateImage(cvSize(NCX,NCY), IPL_DEPTH_8U, 1);
//	binLeftImgSmallCyan = cvCreateImage(cvSize(NCX,NCY), IPL_DEPTH_8U, 1);
//	leftImgHSV= cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 3);
//
//	cvInitFont(&myFonto,CV_FONT_HERSHEY_SIMPLEX,1.0f,1.0f);
//
//	//cvNamedWindow("BinLeftCam1");
//	//cvNamedWindow("BinRightCam1");
//
//	_beginthread(cameraLoop, 0 ,NULL );
//#endif

	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize (600, 600);
	glutInitWindowPosition (0, 0);
	mainWindow = glutCreateWindow ("Probcon");
	init();
	glutKeyboardFunc(processNormalKeys);
	//subWindow1 = glutCreateSubWindow(mainWindow, 0,0,600,600);
	glutDisplayFunc(MainLoop);
	//subWindow2 = glutCreateSubWindow(mainWindow, 600,0,600,600);
	glutMainLoop();

	fprintf(stderr, "\n\nDone 2 !!! :D.\n");     
	return 0;
}

double dTime;

//#ifndef SIMULATION_ONLY
///*******************************************************************************
//Servo callback.  
//Called every servo loop tick.  Simulates a gravity well, which sucks the device 
//towards its center whenever the device is within a certain range.
//*******************************************************************************/
//HDCallbackCode HDCALLBACK gravityWellCallback(void *data)
//{
//	HDdouble kStiffness = 0.095;//0.045; /* N/mm */
//	static const hduVector3Dd wellPos ;//= {0,0,0};
//
//	HDErrorInfo error;
//	hduVector3Dd position;
//	hduVector3Dd force;
//	hduVector3Dd positionTwell;
//
//	HHD hHD = hdGetCurrentDevice();
//	WaitForSingleObject( hIOMutex, INFINITE);
//	hdBeginFrame(hHD);
//
//	hdGetDoublev(HD_CURRENT_POSITION, position);
//	hdGetDoublev(HD_CURRENT_POSITION, global_position);
//	hdGetDoublev(HD_CURRENT_FORCE, global_force);
//	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, global_joint_angles);
//	hdGetDoublev(HD_CURRENT_JOINT_TORQUE, global_joint_torque);
//	hdGetDoublev(HD_CURRENT_VELOCITY, velocity);
//
//
//	memset(force, 0, sizeof(hduVector3Dd));
//
//	hduVecSubtract(positionTwell, wellPos2, position);
//
//	if(bZOnlyControl)
//	{
//		positionTwell[0]=0.0;
//		positionTwell[1]=0.0;
//	}
//	stiff=kStiffness;
//
//	hduVecScale(force, positionTwell,stiff);
//	divis=hduVecMagnitude(force);
//	if(divis<0.3)
//	{
//		stiff=-16*(0.3-divis)+1.0;
//		hduVecScale(force, force,stiff );
//	}
//	
//	velMeanX=0.0;
//	velMeanY=0.0;
//
//	//Shift this and add to the end of it
//	for(iGW=0;iGW<FILT_LEN-1;iGW++)
//	{
//		velSmoothX[iGW]=velSmoothX[iGW+1];
//		velSmoothY[iGW]=velSmoothY[iGW+1];
//	}
//	velSmoothX[FILT_LEN-1]=velocity[0];
//	velSmoothY[FILT_LEN-1]=velocity[1];
//
//	for(iGW=0;iGW<FILT_LEN;iGW++)
//	{
//		velMeanX+=velSmoothX[iGW];
//		velMeanY+=velSmoothY[iGW];
//	}
//
//	velMeanX/=FILT_LEN;
//	velMeanY/=FILT_LEN;
//
//	//hdGetDoublev(HD_CURRENT_POSITION, global_position);
//	//printf("\n%f", global_position[0]);
//	//pri
//	if(bMatlabLoopOn)
//	{ 
//		force[0]=2.8*(inforce[0]-velMeanX*visc);
//		force[1]=1.8*(inforce[1]-velMeanY*visc);
//		{
//			if(global_position[0]<-35.0 && force[0]<0.0)
//				force[0]=0.0;
//			else
//				if(global_position[0]>35.0 && force[0]>0.0)
//					force[0]=0.0;
//			if(global_position[1]<-35.0 && force[1]<0.0)
//				force[1]=0.0;
//			else
//				if(global_position[1]>30.0 && force[1]>0.0)
//					force[1]=0.0;
//		}
//	}
//
//	force[1] += 0.48; 
//
//	//Apply force limits
//	for(int i = 0; i < 1; i++)
//	{
//		if(force[i] > MAX_FORCE_X)
//		{
//			force[i] = MAX_FORCE_X;
//			//printf("\nMAX_FORCE REACHED!");
//		}
//		else if(force[i] < -MAX_FORCE_X)
//		{
//			force[i] = -MAX_FORCE_X;
//			//printf("\nMAX_FORCE REACHED!");
//		}
//	}
//	for(int i = 1; i < 3; i++)
//	{
//		if(force[i] > MAX_FORCE)
//		{
//			force[i] = MAX_FORCE;
//			//printf("\nMAX_FORCE REACHED!");
//		}
//		else if(force[i] < -MAX_FORCE)
//		{
//			force[i] = -MAX_FORCE;
//			//printf("\nMAX_FORCE REACHED!");
//		}
//	}
//	hdSetDoublev(HD_CURRENT_FORCE, force);
//
//
//	/*
//	if(bMatlabLoopOn)
//	hdSetDoublev(HD_CURRENT_FORCE,inforce);
//	*/
//
//
//	hdEndFrame(hHD);
//
//	ReleaseMutex( hIOMutex );
//	if (HD_DEVICE_ERROR(error = hdGetError()))
//	{
//		hduPrintError(stderr, &error, 
//			"Error detected while rendering gravity well\n");
//
//		if (hduIsSchedulerError(&error))
//		{
//			return HD_CALLBACK_DONE;
//		}
//	}
//
//	return HD_CALLBACK_CONTINUE;
//}
//#endif

// scale 4
// x=Gx/200+0.5


// scale 2
// x=Gx/100+0.5
void exitProgVK()
{
	kill = 1;
	//Sleep(200);
	freeAllMem();

	exit(0);
}

void MainLoop()
{
//	if(GetAsyncKeyState(VK_ESCAPE))
//	{
//		exitProgVK();
//	}
	//else
	{
		//if(GetAsyncKeyState('P'))
		//{
		//	b3DPositionServo = true;
		//}
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


//void recordingLoop( void *dummy  )
//{
//	FILE *data;
//	double time = 0.0;
//	data = fopen("TestRec03.txt","w");
//	printf("\n\nStarted Recording Loop\n\n");
//	while ( !kill )
//	{
//		if ( recording == 1 && bMatlabLoopOn)
//		{
//			//printf("\nRecording\t1");
//
//			//Sleep(10);	//We are waiting 10 ms = 0.01 s
//			time += 0.01;
//			//WaitForSingleObject( hIOMutex, INFINITE );
//
//			fprintf(data, "%f\t", time);																			//1
//#ifndef SIMULATION_ONLY
//			fprintf(data, "%f\t%f\t", global_position[0], global_position[2]);										//2,3
//#else
//			fprintf(data, "%f\t%f\t", xglob, yglob);																//2,3
//#endif
//			fprintf(data, "%f\t%f\t", estimated_pos[0]*NCX, estimated_pos[1]*NCY);									//4,5					
//			fprintf(data, "%f\t", t);																				
//
//			fprintf(data, "\n");
//			
//			//ReleaseMutex( hIOMutex );
//		}
//	}
//	fclose(data);
//	printf("\n\nEnded recording Loop\n\n");
//}

