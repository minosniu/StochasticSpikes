#ifndef PROBCON_H
#define PROBCON_H

//#define SHOW_POTENTIAL
#define SIMULATION_ONLY

#define MAX_FORCE 1.0
#define MAX_FORCE_X 1.0

#define FILT_LEN 2
#define FILT_DIF 6

#define cost_sc 16

#define SC	1
#define NCX 30		//number of cells for state estimate
#define NSX 30*SC    //number of sensory cells
#define NCY 30		//number of cells for state estimate
#define NSY	30*SC    //number of sensory cells

#define MAX_N 200
#define MAX_T 500
#define GL_PI 3.141593

time_t rawtime;
struct tm * timeinfo;

int mainWindow, subWindow1,subWindow2;

/// ROBOT CODE DECLARATIONS
int kill;
double cdiffx;
double cdiffy;
double tp;
double *pt;
double *ptp;
double *pGx;
double *pGy;
bool b3DPositionServo=false;
bool bMatlabLoopOn=false;
bool bPlotLoopOn=false;
bool bZOnlyControl=false;
double visc=.0036;//0.0028;//0.0051;//0.0028;//0.0032
double scale=.689;
double divis;
double stiff;

unsigned char keys[256];

double velSmoothX[FILT_LEN];
double velSmoothY[FILT_LEN];
double velMeanX=0.0;
double velMeanY=0.0;

int c; 

// PROBABILITY-CONTROL CODE DECLARATIONS 
double min_x = 0.0;    //bounds on underlying variable
double max_x = 1.0;

double rf_width_x = 0.13/(double)SC;     //sd of gaussian
double min_y = 0.0;    //bounds on underlying variable 
double max_y = 1.0;
double rf_width_y = 0.13/(double)SC;     //sd of gaussian
double beta = .01;
double xpoints[NCX];  //= linspace(0.0, 1.0, NCX);

double pot1=0,pot2=0,pot3=0,pot4=0;
double sp1=0,sp2=0,sp3=0,sp4=0; //double or binary??
double difx, dify;

double t = 0.0;
double Vfnx,Vfny;
double Vfn[NCX][NCY];
double VfnCam[NCX][NCY];
double Vfn_T[NCY][NCX];
double Vbin[NCX][NCY];
double Vbin_T[NCY][NCX];

double Cfnx,Cfny;
double Cfn[NCX][NCY];
double CfnCam[NCX][NCY];
double Cfn_T[NCY][NCX];
double Cbin[NCX][NCY];
double Cbin_T[NCY][NCX];

double dpotential[NCX][NCY];
double dpotentialExt[NCX][NCY];
double outProdMatxy[NSX][NSY][NCX][NCY];
double outProdMatxvy[NSX][NSY][NCX][NCY];
double outProdMatvxy[NSX][NSY][NCX][NCY];
double xglob=0.5;
double yglob=0.5;
int ind[2];
int num_spike;
double sum_est_pos[2];
double spx2_ctx[NCX][NCX];
double spy2_ctx[NCY][NCY];
double spx2_ctx_neg[NCX][NCX];
double spy2_ctx_neg[NCY][NCY];
double sum1, sum2, sum3, sum4;
double estimated_pos[2];
double ypoints[NCY];  //= linspace(0.0, 1.0, NCX);

typedef struct rf //can also add rf_center
{
	double center;
	//int ns;
	//double Vrf[MAX_N];
	double *Vrf;
} _RF;

_RF *rfx[NSX];
_RF *rfvx[NSX];
_RF *rfcx[NCX]; //cortex cells
_RF *rfy[NSY];
_RF *rfvy[NSY];
_RF *rfcy[NCY]; //cortex cells


double potential[NCX][NCY];  //initial membrane potential is zero
#ifdef SHOW_POTENTIAL
double plotpotential[NCX][NCY];  //initial membrane potential is zero
#endif
double threshold = 0.8+.1+.3;		 //simple integrate-and-fire neurons
double max_synapse = 1.0;      //how many spikes needed to raise to threshold is 1/max_synapse
double sp_state [NCX][NCY];	//binary or double??
double sp_state_T [NCY][NCX];
double sp_sensory [NSX][NSY];
int ix=0;
int iy=0;
double points[2*NCX];  //= linspace(0.0, 1.0, NCX);
double Vdiff[2*NCX];
double L1[NCX][NCY];
double vpoints[NCX]; 
double vV[NCX];
double Vfn0[NCX][NCY];
double vC[NCX];
double Cfn0[NCX][NCY];


double difSmoothX[FILT_DIF];
double difSmoothY[FILT_DIF];
double difMeanX=0.0;
double difMeanY=0.0;

// FUNCTION PROTOTYPES
int InitProbCon(void);
void init(void);

void TimePlot(void);
void MainLoop(void);
void LogicLoop(void);
void processNormalKeys(unsigned char key, int x, int y);
//void ProbConLoop_Matlab();
void ProbConLoop_C();


void freeProbConMem();
void freeAllMem();

void drawGrid(void);
void drawGridPixel(double x, double y);
void drawCross(double x, double y);
void drawCircle(double x, double y);

double round(double x);
double randn(void);

void MtrxPrnt(double *pM1, int m, int n);
void InnerProduct(double *pV1,double *pV2,int n,double *pS1);
void MtrxTranspose(double *pM1, int m, int n, double *pM2);
void MtrxShift(double *pM1, int m, int n, int shift_x, int shift_y, double *pM2);
void ScalarProd(double *pM1, int m, int n, double s, double *pM2);
void MtrxSum(double *pM1, double *pM2, int m, int n, double *pM3);
void MtrxProduct(double *pM1,double *pM2,int m,int n,int p,double *pM3);
void OuterProduct(double *pV1,double *pV2,int m,int n,double *pM1);


// FUNCTION IMPLEMENTATIONS
double randn()
{
	return (((double)(rand()%2000)/1000.0)-1.0);	
}

void drawCross(double x, double y)
{
	glLineWidth(2.0);
	glBegin(GL_LINES);
		glVertex2f(x-0.02*NCX, y);	
		glVertex2f(x+0.02*NCX, y);	
		glVertex2f(x, y-0.02*NCX);	
		glVertex2f(x, y+0.02*NCX);	
	glEnd();
	glLineWidth(1.0);
}

	
void drawEmptyCircle(double x, double y)
{
	glLineWidth(2.0);
	glBegin(GL_LINES);
		double radius=0.04*NCX;
		for(double di=0.0; di<=2*GL_PI;di+=GL_PI/8.0)
		{
			glVertex2f(x+radius*cos(di), y+radius*sin(di));	
		}
	glEnd();
	glLineWidth(1.0);
}

void drawCircle(double x, double y)
{
	glBegin(GL_TRIANGLE_FAN);
		glVertex2f(x, y);	
		double radius=0.04*NCX;
		for(double di=0.0; di<=2*GL_PI;di+=GL_PI/8.0)
		{
			glVertex2f(x+radius*cos(di), y+radius*sin(di));	
		}
	glEnd();
}

void drawLoop(double x, double y)
{
	glBegin(GL_LINES);
		double radius=0.02*NCX;
		for(double di=0.0; di<=2*GL_PI;di+=GL_PI/8.0)
		{
			glVertex2f(x+radius*cos(di), y+radius*sin(di));	
		}
	glEnd();
}

void drawGridPixel(double x, double y)
{
	glBegin(GL_TRIANGLE_FAN);

	for(int i=0; i<NCX; i++)
	{
		glVertex2f(x, y);	
		glVertex2f(x, y+0.02*NCY);	
		glVertex2f(x+0.02*NCX, y+0.02*NCY);	
		glVertex2f(x+0.02*NCY, y);	
	}
	glEnd();
}


void drawGrid()
{
	glBegin(GL_LINES);

	for(int i=0; i<NCX; i++)
	{
			glVertex2f(i, 0.0);	
			glVertex2f(i, NCY);	
				
			glVertex2f(0.0, i);	
			glVertex2f(NCY, i);	
	}
	glEnd();
}





double round(double x) 
{
	return (x>=0 ? floor(x + 0.5) : ceil(x + 0.5));
}


void MtrxPrnt(double *pM1, int m, int n)
{
	printf("\n\nMatrix Print:\n");

	for (int i=0; i<m; i++)
	{
		for (int j=0; j<n; j++)
		{
			printf("\t%lf",*(pM1+i*n+j));
		} 
		printf("\n");
	}
}


void InnerProduct(double *pV1,double *pV2,int n,double *pS1)
{	
	*pS1=0;
	for (int i = 0; i <= n-1; i = i + 1)
	{
		*pS1=*pS1+(*(pV1+i)**(pV2+i));
	}
}

void OuterProduct(double *pV1,double *pV2,int m,int n,double *pM1)
{	
	for (int i = 0; i <= m-1; i = i + 1)
	{		
		for (int j = 0; j <= n-1; j = j + 1)
		{
			*(pM1+i*n+j)=*(pV1+i)*(*(pV2+j));
		}
	}
}

void MtrxTranspose(double *pM1, int m, int n, double *pM2)
{
	for (int i=0; i<=m-1; i=i+1)
	{
		for (int j=0; j<=n-1; j=j+1)
		{
			*(pM2+j*m+i)=*(pM1+i*n+j);
		} 
	}
}

void MtrxShift(double *pM1, int m, int n, int shift_x, int shift_y, double *pM2)
{
	for (int i=0; i<=m-1; i=i+1)
	{
		for (int j=0; j<=n-1; j=j+1)
		{
			if ((i-shift_x<0)||(j-shift_y<0))
			{
				*(pM2+j*m+i) = 0;
			}
			else
			{
				*(pM2+j*m+i) = *(pM1+(i-shift_x)*n+(j-shift_y));
			}
		} 
	}
}

void ScalarProd(double *pM1, int m, int n, double s, double *pM2)
{
	for (int i=0; i<m; i++)
	{
		for (int j=0; j<n; j++)
		{
			*(pM2+i*n+j)=*(pM1+i*n+j)*s;
		} 
	}
}

void MtrxSum(double *pM1, double *pM2, int m, int n, double *pM3)
{
	for (int i=0; i<=m-1; i=i+1)
	{
		for (int j=0; j<=n-1; j=j+1)
		{
			*(pM3+i*n+j)=*(pM1+i*n+j)+*(pM2+i*n+j);
		} 
	}
}

void MtrxProduct(double *pM1,double *pM2,int m,int n,int p,double *pM3)
{		
	double S;
	int  i,j,k;

	for(i=0;i<m;i++)
	{
		for(j=0;j<p;j++)
		{
			S=0.0;
			for(k=0;k<n;k++)
			{
				S+=*(pM1+i*n+k)**(pM2+k*p+j);
				//S+=a[i][k]*b[k][j];
			}
			*(pM3+i*p+j)=S;
		}
	} 

}

//void MtrxProduct(double *pM1,double *pM2,int m,int n,int p,double *pM3)
//{		
//	double *pM2T= (double*)malloc(p*n*sizeof(double));
//
//	MtrxTranspose(pM2, n, p, pM2T);
//
//	double S;
//
//	double *pV1a=NULL;
//	double *pV2a=NULL;
//
//	pV1a = (double*)malloc(n*sizeof(double));
//	pV2a = (double*)malloc(n*sizeof(double));
//
//	for (int i=0; i<=m-1; i=i+1)
//	{
//		for (int k=0; k<=p-1; k=k+1)
//		{
//			memcpy(pV1a,pM1+i*n,n*sizeof(double));
//			memcpy(pV2a,pM2T+k*n,n*sizeof(double));
//			InnerProduct(pV1a,pV2a,n,&S);
//			*(pM3+i*p+k)=S;
//		}
//	}
//
//	free(pV1a);
//	free(pV2a);
//	free(pM2T);
//}

//SYSTEMTIME st;

void LogicLoop()
{

	if(bMatlabLoopOn)
	{
		//tim++;
		//Gx =   global_position[0];
		//Gy = - global_position[1];

		// Alter the values of diffx
		ProbConLoop_C();
		//ProbConLoop_Matlab();

		//printf("\n\nGx = %lf\tGy = %lf\tdifx = %lf  \tdify = %lf", Gx, Gy, cdiffx, cdiffy);
		//printf("\n\ndifx = %lf  \tdify = %lf \tcdiffx = %lf\tcdiffy = %lf", Gx, Gy, cdiffx, cdiffy);

		//GetSystemTime(&st);
		
//		printf ( "\n\nCurrent seconds %u and milliseconds %time and date: %u", st.wSecond, st.wMilliseconds);


		//time ( &rawtime );
		//timeinfo = localtime ( &rawtime );
		//printf ( "\n\nCurrent local time and date: %s", asctime (timeinfo) );


		//ZeroMemory(inforce,sizeof(hduVector3Dd));


		//+++ OUTPUT COMMAND!
		//inforce[0] = scale*cdiffx;
		//inforce[1] = -scale*cdiffy;

		t = t + 1;

	}

#ifndef SIMULATION_ONLY
	/* Periodically check if the gravity well callback has exited. */
	if (!hdWaitForCompletion(hGravityWell, HD_WAIT_CHECK_STATUS))
	{
		kill = 1;
		//fprintf(stderr, "Press any key to quit.\n");     
		//getch();
	}
#endif
	//glutPostRedisplay();
}

void init(void)
{
	glClearColor (0.0, 0.0, 0.0, 0.0);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0, NCX, NCY, 0.0, -1.0, 1.0);
}

void processNormalKeys(unsigned char key, int x, int y)
{

	//if (key == VK_ESCAPE)
	if (key == 27)
	{
		kill = 1;
		printf("\n11");
		//Sleep(200);
		freeAllMem();
		printf("\n12");

		exit(0);
	}
	else
	{
		if ( key == 'p') {
			b3DPositionServo = true;
		}
		else if ( key =='z' ) {
			bZOnlyControl = true;
		}
		else if ( key == 'm' )
		{
			bMatlabLoopOn = true;
		}
		else if ( key == 'r' )
		{
			if ( recording == 0) 
			{
				printf("Data recording ON\n");
				recording = 1;
			}
			else
			{
				printf("Data recording OFF\n");
				recording = 0;
			}
		}
	}
}

void freeAllMem()
{
	freeProbConMem();

#ifndef SIMULATION_ONLY

	/* For cleanup, unschedule callback and stop the scheduler. */
	hdStopScheduler();
	hdUnschedule(hGravityWell);
	

	/* Disable the device. */
	hdDisableDevice(hHD);
#endif
	fprintf(stderr, "\n\nProgram End\n");     
#ifdef USE_CAMERA
		//cvDestroyWindow("LeftCam");
		//cvDestroyWindow("BinLeftCam1");
		//cvDestroyWindow("BinLeftCam2");
		cvReleaseCapture(&leftCapture);
		cvReleaseImage(&binLeftImgGreen);
		cvReleaseImage(&binLeftImgCyan);
		cvReleaseImage(&binLeftImgSmallGreen);
		cvReleaseImage(&binLeftImgSmallCyan);

		//cvDestroyWindow("RightCam");
		//cvDestroyWindow("BinRightCam1");
		//cvDestroyWindow("BinRightCam2");
	/*	cvReleaseCapture(&rightCapture);
		cvReleaseImage(&binRightImgGreen);
		cvReleaseImage(&binRightImgCyan);
		cvReleaseImage(&binRightImgSmallGreen);
		cvReleaseImage(&binRightImgSmallCyan);*/

#endif
}


int InitProbCon()   
{
	//MCInit();


	int i_IPC,j_IPC;
	//INIT
	for(i_IPC=0;i_IPC<NSX;i_IPC++)
	{
		rfx[i_IPC]=(rf*)malloc(sizeof(rf));
		rfx[i_IPC]->Vrf=(double*)malloc(NCX*sizeof(double));
	}
	for(i_IPC=0;i_IPC<NSX;i_IPC++)
	{
		rfvx[i_IPC]=(rf*)malloc(sizeof(rf));
		rfvx[i_IPC]->Vrf=(double*)malloc(NCX*sizeof(double));
	}
	for(i_IPC=0;i_IPC<NCX;i_IPC++)
	{
		rfcx[i_IPC]=(rf*)malloc(sizeof(rf));
		rfcx[i_IPC]->Vrf=(double*)malloc(NCX*sizeof(double));
	}
	for(i_IPC=0;i_IPC<NSY;i_IPC++)
	{
		rfy[i_IPC]=(rf*)malloc(sizeof(rf));
		rfy[i_IPC]->Vrf=(double*)malloc(NCY*sizeof(double));
	}
	for(i_IPC=0;i_IPC<NSY;i_IPC++)
	{
		rfvy[i_IPC]=(rf*)malloc(sizeof(rf));
		rfvy[i_IPC]->Vrf=(double*)malloc(NCY*sizeof(double));
	}
	for(i_IPC=0;i_IPC<NCY;i_IPC++)
	{
		rfcy[i_IPC]=(rf*)malloc(sizeof(_RF));
		rfcy[i_IPC]->Vrf=(double*)malloc(NCY*sizeof(double));
	}




	//Dynamic Allocation

	//int** x;

	//x = (int**)malloc(dimension1_max * sizeof(int*));
	//for (int i = 0; i < dimension1_max; i++) {
	//	x[i] = (int*)malloc(dimension2_max * sizeof(int));
	//}

	//for (i = 0; i < NSX; i++)
	//{
	//	for (j = 0; j < NSY; j++)
	//	{
	//		OuterProduct(rfx[i]->Vrf, rfy[j]->Vrf, NCX, NCY, outProdMatxy[i][j][0]);
	//	}
	//}


	srand ( time(NULL) );


	//ALWAYS
	for (i_IPC = 0; i_IPC < NCX; i_IPC++)
	{
		xpoints[i_IPC]=(double)(i_IPC+1)*(max_x-min_x)/(double)(NCX);
	}
	for (i_IPC = 0; i_IPC < NCY; i_IPC++)
	{
		ypoints[i_IPC]=(double)(i_IPC+1)*(max_y-min_y)/(double)(NCY);
	}



	for (i_IPC = 0; i_IPC < NSX; i_IPC++)
	{
		rfx[i_IPC]->center=(double)(i_IPC+1)*(max_x-min_x)/(double)(NSX);
		for (j_IPC = 0; j_IPC < NCX; j_IPC++)
		{
			rfx[i_IPC]->Vrf[j_IPC]= exp(-(xpoints[j_IPC]-rfx[i_IPC]->center)*(xpoints[j_IPC]-rfx[i_IPC]->center)/rf_width_x/rf_width_x);
		}
	}



	for (i_IPC = 0; i_IPC < NSY; i_IPC++)
	{
		rfy[i_IPC]->center=(double)(i_IPC+1)*(max_y-min_y)/(double)(NSY);
		for (j_IPC = 0; j_IPC < NCY; j_IPC++)
		{
			rfy[i_IPC]->Vrf[j_IPC]= exp(-pow(ypoints[j_IPC]-rfy[i_IPC]->center,2)/pow(rf_width_y,2));
		}
	}


	for (i_IPC = 0; i_IPC < NSX; i_IPC++)
	{
		rfvx[i_IPC]->center=(double)i_IPC*(max_x-min_x)/(double)(NSX-1);
		for (j_IPC = 0; j_IPC < NCX; j_IPC++)
		{
			rfvx[i_IPC]->Vrf[j_IPC]= 2*((xpoints[j_IPC]-rfvx[i_IPC]->center)/pow(rf_width_x,2))*exp(-pow(xpoints[j_IPC]-rfvx[i_IPC]->center,2)/pow(rf_width_x,2));//-gradient(rfx{i});
		}
	}


	for (i_IPC = 0; i_IPC < NSY; i_IPC++)
	{
		rfvy[i_IPC]->center=(double)i_IPC*(max_y-min_y)/(double)(NSY-1);
		for (j_IPC = 0; j_IPC < NCY; j_IPC++)
		{
			rfvy[i_IPC]->Vrf[j_IPC]= 2*((ypoints[j_IPC]-rfvy[i_IPC]->center)/pow(rf_width_y,2))*exp(-pow(ypoints[j_IPC]-rfvy[i_IPC]->center,2)/pow(rf_width_y,2));//-gradient(rfx{i});
		}
	}

	for (i_IPC = 0; i_IPC < NCX; i_IPC++)
	{
		//rfcx[i_IPC].ns = NCX;
		rfcx[i_IPC]->center=(double)i_IPC*(max_x-min_x)/(double)(NCX-1);
		for (j_IPC = 0; j_IPC < NCX; j_IPC++)
		{
			rfcx[i_IPC]->Vrf[j_IPC]= exp(-pow(xpoints[j_IPC]-rfcx[i_IPC]->center,2)/pow(.5,2));
		}
	}

	



	for (i_IPC = 0; i_IPC < NCY; i_IPC++)
	{
		rfcy[i_IPC]->center=(double)i_IPC*(max_y-min_y)/(double)(NCY-1);
		for (j_IPC = 0; j_IPC < NCY; j_IPC++)
		{
			rfcy[i_IPC]->Vrf[j_IPC]= exp(-pow(ypoints[j_IPC]-rfcy[i_IPC]->center,2)/pow(.5,2));
		}
	}



	for (i_IPC = 0; i_IPC < 2*NCX; i_IPC++)
	{
		points[i_IPC]=-.5+(double)i_IPC*2.0/(double)(2*NCX-1);
	}

	for (i_IPC = 0; i_IPC < 2*NCX; i_IPC++)
	{
		Vdiff[i_IPC]=2*((points[i_IPC]-.5)/pow(.1,2))*exp(-pow(points[i_IPC]-.5,2)/pow(.1,2));
		//Vdiff = 2*((linspace(-.5, 1.5, 2*NCX)-.5)/(.2^2)).*exp(-(linspace(-.5, 1.5, 2*NCX)-.5).^2/(.2^2));
	}

	for (i_IPC = -NCX/2; i_IPC < NCX/2; i_IPC++)
	{
		//L1(i_IPC+NCX/2,:)=Vdiff(51-i_IPC:150-i_IPC);
		for (j_IPC = 0; j_IPC < NCX; j_IPC++)
		{
			L1[i_IPC+NCX/2][j_IPC]=Vdiff[NCX/2-i_IPC+j_IPC];
		}
	}

	//+++ Clear mem

	//ZeroMemory(sp_state[0],NCX*NCY*sizeof(double)); 

	xglob = 0.5;
	yglob = 0.5;


	//MtrxTranspose(L1[0], NCX, NCX, L3[0]);

#ifndef USE_CAMERA
	for (i_IPC = 0; i_IPC < NCX; i_IPC++)
	{
		vpoints[i_IPC]=(double)i_IPC/(double)(NCX-1);
	}

	for (i_IPC = 0; i_IPC < NCX; i_IPC++)
	{
		vV[i_IPC]=exp(-pow(vpoints[i_IPC]-.5,2)/pow(.15,2));//pdf('norm',1:NCX,50,5*3)
	}
	for (i_IPC = 0; i_IPC < NCX; i_IPC++)
	{
		vC[i_IPC]=0.0;//exp(-pow(vpoints[i_IPC]-.5,2)/pow(.07,2));//pdf('norm',1:NCX,50,5*3)
	}

	OuterProduct(vV,vV,NCX,NCY,Vfn0[0]);//Vfn0=pdf('norm',1:NCX,50,5*3)'*pdf('norm',1:NCY,50,5*3);
	OuterProduct(vC,vC,NCX,NCY,Cfn0[0]);
#endif
	for (i_IPC = 0; i_IPC < NSX; i_IPC++)
	{
		for (j_IPC = 0; j_IPC < NSY; j_IPC++)
		{
			OuterProduct(rfx[i_IPC]->Vrf, rfy[j_IPC]->Vrf, NCX, NCY, outProdMatxy[i_IPC][j_IPC][0]);
		}
	}

	for (i_IPC = 0; i_IPC < NSX; i_IPC++)
	{
		for (j_IPC = 0; j_IPC < NSY; j_IPC++)
		{
			OuterProduct(rfx[i_IPC]->Vrf, rfvy[j_IPC]->Vrf, NCX, NCY, outProdMatxvy[i_IPC][j_IPC][0]);
		}
	}

	for (i_IPC = 0; i_IPC < NSX; i_IPC++)
	{
		for (j_IPC = 0; j_IPC < NSY; j_IPC++)
		{
			OuterProduct(rfvx[i_IPC]->Vrf, rfy[j_IPC]->Vrf, NCX, NCY, outProdMatvxy[i_IPC][j_IPC][0]);
		}
	}
	//Vfn0=double(Vfn0>=.85);

	/*
	VfnNeg=pdf('norm',1:NCX,60,1*15)'*pdf('norm',1:NCY,50,1*15);
	VfnNeg=VfnNeg/max(max(VfnNeg))*0;
	VfnNeg=double(VfnNeg>=.85);
	VbinNeg=zeros(NCX,NCY);
	*/
	t=1.0;
	return 0;
}


void freeProbConMem()
{
	int i_FPCM;
	for(i_FPCM=0;i_FPCM<NSX;i_FPCM++)
	{
		free(rfx[i_FPCM]->Vrf);
		free(rfx[i_FPCM]);
	}
	
	for(i_FPCM=0;i_FPCM<NSX;i_FPCM++)
	{
		free(rfvx[i_FPCM]->Vrf);
		free(rfvx[i_FPCM]);
	}
	
	for(i_FPCM=0;i_FPCM<NCX;i_FPCM++)
	{
		free(rfcx[i_FPCM]->Vrf);
		free(rfcx[i_FPCM]);
	}
		
	for(i_FPCM=0;i_FPCM<NSY;i_FPCM++)
	{
		free(rfy[i_FPCM]->Vrf);
		free(rfy[i_FPCM]);
	}
		
	for(i_FPCM=0;i_FPCM<NSY;i_FPCM++)
	{
		free(rfvy[i_FPCM]->Vrf);
		free(rfvy[i_FPCM]);
	}
	
	for(i_FPCM=0;i_FPCM<NCY;i_FPCM++)
	{
		free(rfcy[i_FPCM]->Vrf);
		free(rfcy[i_FPCM]);
	}
}



void ProbConLoop_C()
{
	int i_PCLC,j_PCLC;
	
#ifndef USE_CAMERA
	//Vfny = 20*cos(.007*6.28*t);
	//Vfny = 0.0;
	Vfny = 8*cos(.007*6.28*t*1.0);
	Vfnx = 8*sin(.007*6.28*t/1.0);// 0.0;
	//Vfnx = 0.0;
	MtrxShift(Vfn0[0], NCX, NCY, round(Vfnx), round(Vfny), Vfn[0]);
	
	Vfnx = 0;//20*sin(.005*6.28*t);
	Vfny = 0;//20*cos(.003*6.28*t);
	MtrxShift(Cfn0[0], NCX, NCY, round(Vfnx), round(Vfny), Cfn[0]);
#else 
	memcpy(Vfn,VfnCam,NCX*NCY*sizeof(double));
	memcpy(Cfn,CfnCam,NCX*NCY*sizeof(double));
#endif

	//REDEFINE CFN from the cblob values



#ifdef SIMULATION_ONLY
	//x = x+beta*(randn(1)/4+sp2(t)-sp1(t));%0.5+(cos(4*6.28*t/(double)MAX_T)/4);%5
	//y = y+beta*(randn(1)/4+sp4(t)-sp3(t));%0.5+(cos(2*6.28*t/(double)MAX_T)/4);

	xglob = xglob+beta*(randn()/8+3*cdiffx);//(sp2-sp1));
	yglob = yglob+beta*(randn()/8+3*cdiffy);//(sp4-sp3));
#else
	xglob=Gx/100+.5;
	yglob=Gy/100+.5;
#endif
	ScalarProd(sp_state[0], NCX, NCY, max_synapse, dpotential[0]);
	MtrxSum(potential[0], dpotential[0], NCX, NCY, potential[0]);
	ScalarProd(potential[0], NCX, NCY, 0.95, potential[0]); //decay


	ind[0]=floor(NCX*(xglob-min_x)); 
	ind[1]=floor(NCY*(yglob-min_y));

	//dx=sign(floor(NCX * (x-min_x))-ix)/3;
	//dy=sign(floor(NCY * (y-min_y))-iy)/3;

	ix = floor(NCX * (xglob-min_x));     //index for current value of x
	iy = floor(NCY * (yglob-min_y));     //index for current value of y

	//printf("\n\t%lf.2",t);
	//<WARNING SUPER SLOW

	for (i_PCLC = 0; i_PCLC < NSX; i_PCLC++) 
	{
		for (j_PCLC = 0; j_PCLC < NSY; j_PCLC = j_PCLC + 1)
		{
			sp_sensory[i_PCLC][j_PCLC] = (((double)(rand()%1000))/1000.0 < (rfx[i_PCLC]->Vrf[ix])*(rfy[j_PCLC]->Vrf[iy]));
			if (sp_sensory[i_PCLC][j_PCLC] > 0.0)
			{
				memcpy(dpotentialExt[0],outProdMatxy[i_PCLC][j_PCLC][0],NCX*NCY*sizeof(double));
				ScalarProd(dpotentialExt[0], NCX, NCY, (double)SC*max_synapse, dpotential[0]);

				memcpy(dpotentialExt[0],outProdMatxvy[i_PCLC][j_PCLC][0],NCX*NCY*sizeof(double));
				ScalarProd(dpotentialExt[0], NCX, NCY, (sp4-sp3)/5.0, dpotentialExt[0]); 
				MtrxSum(dpotentialExt[0], dpotential[0], NCX, NCY, dpotential[0]);

				memcpy(dpotentialExt[0],outProdMatvxy[i_PCLC][j_PCLC][0],NCX*NCY*sizeof(double));
				ScalarProd(dpotentialExt[0], NCX, NCY, (sp2-sp1)/5.0, dpotentialExt[0]); 

				MtrxSum(dpotentialExt[0], dpotential[0], NCX, NCY, dpotential[0]);	

				ScalarProd(dpotential[0], NCX, NCY, 1.0/200.0, dpotential[0]); 

				MtrxSum(dpotential[0], potential[0], NCX, NCY, potential[0]);	

			}						
		}
	}

#ifdef SHOW_POTENTIAL
	for (i_PCLC = 0; i_PCLC < NCX; i_PCLC++) 
	{
		for (j_PCLC = 0; j_PCLC < NCY; j_PCLC = j_PCLC + 1)
		{
			plotpotential[i_PCLC][j_PCLC] = potential[i_PCLC][j_PCLC];
		}
	} 
	ScalarProd(plotpotential[0],NCX,NCY,10.0,plotpotential[0]);
#endif
	//printf("\n\t%lf.3",t);

	for (i_PCLC = 0; i_PCLC < NCX; i_PCLC++) 
	{
		for (j_PCLC = 0; j_PCLC < NCY; j_PCLC = j_PCLC + 1)	
		{
			sp_state[i_PCLC][j_PCLC]=(((double)(rand()%1000)/1000.0)<potential[i_PCLC][j_PCLC]-threshold);//-.5/(double)SC);	//fire    //is double ok for boolean output?? Is the conversion automatic?
			if (sp_state[i_PCLC][j_PCLC]>0) 
			{
				potential[i_PCLC][j_PCLC]=0;	//drop the ones that fired
				if (i_PCLC-1>=0) 
				{
					potential[i_PCLC-1][j_PCLC]=potential[i_PCLC-1][j_PCLC]+beta;
				}
				if (i_PCLC+1<NCX) 
				{
					potential[i_PCLC+1][j_PCLC]=potential[i_PCLC+1][j_PCLC]+beta;
				}
				if (j_PCLC-1>=0) 
				{
					potential[i_PCLC][j_PCLC-1]=potential[i_PCLC][j_PCLC-1]+beta;
				}
				if (j_PCLC+1<NCY) 
				{
					potential[i_PCLC][j_PCLC+1]=potential[i_PCLC][j_PCLC+1]+beta;
				}
			}
		}
	}

	//sensory cortex
	num_spike = 0;
	sum_est_pos[0]=0.0;
	sum_est_pos[1]=0.0;
	for (i_PCLC = 0; i_PCLC < NCX; i_PCLC++) 
	{
		for (j_PCLC = 0; j_PCLC < NCY; j_PCLC = j_PCLC + 1)	
		{
			if (sp_state[i_PCLC][j_PCLC]!=0)
			{
				num_spike = num_spike+1;

				sum_est_pos[0] = sum_est_pos[0] + i_PCLC;
				sum_est_pos[1] = sum_est_pos[1] + j_PCLC;
			}
		}
	}
	num_spike=num_spike==0 ? 1 :  num_spike;
	estimated_pos[0]=sum_est_pos[0]/(NCX*(double)num_spike);
	estimated_pos[1]=sum_est_pos[1]/(NCY*(double)num_spike);

	//printf("\n%d", num_spike);

	//motor cortex
	for (i_PCLC = 0; i_PCLC < NCX; i_PCLC++) 
	{
		for (j_PCLC = 0; j_PCLC < NCY; j_PCLC = j_PCLC + 1)	
		{
			Vbin[i_PCLC][j_PCLC]=(((double)(rand()%1000)/1000.0)<Vfn[i_PCLC][j_PCLC]);//+.2;
		}
	}

	for (i_PCLC = 0; i_PCLC < NCX; i_PCLC++) 
	{
		for (j_PCLC = 0; j_PCLC < NCY; j_PCLC = j_PCLC + 1)	
		{
			Cbin[i_PCLC][j_PCLC]=(((double)(rand()%1000)/1000.0)<Cfn[i_PCLC][j_PCLC]);//+.2;
		}
	}
	MtrxTranspose(sp_state[0], NCX, NCY, sp_state_T[0]);
	MtrxProduct(Vbin[0], sp_state_T[0], NCX, NCY, NCX, spx2_ctx[0]);
	MtrxProduct(Cbin[0], sp_state_T[0], NCX, NCY, NCX, spx2_ctx_neg[0]);

	MtrxTranspose(Vbin[0], NCX, NCY, Vbin_T[0]);
	MtrxTranspose(Cbin[0], NCX, NCY, Cbin_T[0]);
	MtrxProduct(Vbin_T[0], sp_state[0], NCX, NCY, NCX, spy2_ctx[0]);
	MtrxProduct(Cbin_T[0], sp_state[0], NCX, NCY, NCX, spy2_ctx_neg[0]);

	sum1 = 0;
	for (i_PCLC = 0; i_PCLC < NCX; i_PCLC = i_PCLC + 1) 
	{
		for (j_PCLC = 0; j_PCLC < NCY; j_PCLC = j_PCLC + 1)	
		{
			if (spx2_ctx[i_PCLC][j_PCLC]!=0)
			{
				sum1 = sum1 + L1[i_PCLC][j_PCLC];
			}
		}
	}
	for (i_PCLC = 0; i_PCLC < NCX; i_PCLC = i_PCLC + 1) 
	{
		for (j_PCLC = 0; j_PCLC < NCY; j_PCLC = j_PCLC + 1)	
		{
			if (spx2_ctx_neg[i_PCLC][j_PCLC]!=0)
			{
				sum1 = sum1 - L1[i_PCLC][j_PCLC]*cost_sc;
			}
		}
	}
	pot1 = pot1+.1+sum1/25;
	//pot1=pot1+.1+sum(sum(L1(spx2_ctx>0)))/100-sum(sum(L1(spx2Neg_ctx>0)))/100;
	sp1 = (((double)(rand()%1000)/1000.0)<pot1);
	if (sp1>0)
	{
		pot1 = 0;
	}
	pot1 = pot1*.9;
	//
	//printf("\n\t%lf.8",t);

	sum2 = 0;
	for (i_PCLC = 0; i_PCLC < NCX; i_PCLC++) 
	{
		for (j_PCLC = 0; j_PCLC < NCY; j_PCLC = j_PCLC + 1)	
		{
			if (spx2_ctx[i_PCLC][j_PCLC]!=0)
			{
				sum2 = sum2 + L1[i_PCLC][j_PCLC];
			}
		}
	}
	for (i_PCLC = 0; i_PCLC < NCX; i_PCLC++) 
	{
		for (j_PCLC = 0; j_PCLC < NCY; j_PCLC = j_PCLC + 1)	
		{
			if (spx2_ctx_neg[i_PCLC][j_PCLC]!=0)
			{
				sum2 = sum2 - L1[i_PCLC][j_PCLC]*cost_sc;
			}
		}
	}	
	pot2 = pot2+.1-sum2/25;
	//pot2=pot2+.1-sum(sum(L1(spx2_ctx>0)))/100+sum(sum(L1(spx2Neg_ctx>0)))/100;
	sp2 = (((double)(rand()%1000)/1000.0)<pot2);
	if (sp2>0)
	{
		pot2 = 0;
	}
	pot2 = pot2*.9;
	//
	sum3 = 0;
	for (i_PCLC = 0; i_PCLC < NCX; i_PCLC++) 
	{
		for (j_PCLC = 0; j_PCLC < NCY; j_PCLC = j_PCLC + 1)	
		{
			if (spy2_ctx[i_PCLC][j_PCLC]!=0)
			{
				sum3 = sum3 + L1[i_PCLC][j_PCLC];
			}
		}
	}
	for (i_PCLC = 0; i_PCLC < NCX; i_PCLC++) 
	{
		for (j_PCLC = 0; j_PCLC < NCY; j_PCLC = j_PCLC + 1)	
		{
			if (spy2_ctx_neg[i_PCLC][j_PCLC]!=0)
			{
				sum3 = sum3 - L1[i_PCLC][j_PCLC]*cost_sc;
			}
		}
	}
	pot3 = pot3+.1+sum3/25;
	//pot3=pot3+.1+sum(sum(L1(spy2_ctx>0)))/100-sum(sum(L1(spy2Neg_ctx>0)))/100;
	sp3 = (((double)(rand()%1000)/1000.0)<pot3);
	if (sp3>0)
	{
		pot3 = 0;
	}
	pot3 = pot3*.9;
	//
	//printf("\n\t%lf.9",t);

	sum4 = 0;
	for (i_PCLC = 0; i_PCLC < NCX; i_PCLC++) 
	{
		for (j_PCLC = 0; j_PCLC < NCY; j_PCLC = j_PCLC + 1)	
		{
			if (spy2_ctx[i_PCLC][j_PCLC]!=0)
			{
				sum4 = sum4 + L1[i_PCLC][j_PCLC];
			}
		}
	}
	for (i_PCLC = 0; i_PCLC < NCX; i_PCLC++) 
	{
		for (j_PCLC = 0; j_PCLC < NCY; j_PCLC = j_PCLC + 1)	
		{
			if (spy2_ctx_neg[i_PCLC][j_PCLC]!=0)
			{
				sum4 = sum4 - L1[i_PCLC][j_PCLC]*cost_sc;
			}
		}
	}
	pot4 = pot4+.1-sum4/25;

	sp4 = (((double)(rand()%1000)/1000.0)<pot4);
	if (sp4>0)
	{
		pot4 = 0;
	}
	pot4 = pot4*.9;
	
	MtrxTranspose(Vfn[0],NCX,NCY,Vfn_T[0]);
	for(i_PCLC=0;i_PCLC<NCX;i_PCLC++)
	{
		for(j_PCLC=0;j_PCLC<NCY;j_PCLC++)
		{
			//CHANGED THIS!!!!! FIX LATER :D
			if(Vfn_T[j_PCLC][i_PCLC]>0.85)
			//if(Vfn[i_PCLC][j_PCLC]>0.85)
			{
				glColor3f(0.7,0.7,0.0);
				drawGridPixel((double)i_PCLC,(double)j_PCLC);
			}
		}
	}

	MtrxTranspose(Cfn[0],NCX,NCY,Cfn_T[0]);
	for(i_PCLC=0;i_PCLC<NCX;i_PCLC++)
	{
		for(j_PCLC=0;j_PCLC<NCY;j_PCLC++)
		{
			//CHANGED THIS!!!!! FIX LATER :D
			if(Cfn_T[j_PCLC][i_PCLC]>0.85)
			{
				glColor3f(0.7,0.7,0.7);
				drawGridPixel((double)i_PCLC,(double)j_PCLC);
			}
		}
	}

#ifdef SHOW_POTENTIAL
	for(i_PCLC=0;i_PCLC<NCX;i_PCLC++)
	{
		for(j_PCLC=0;j_PCLC<NCY;j_PCLC++)
		{
			//if(plotpotential[i_PCLC][j_PCLC])
			{
				glColor3f(plotpotential[i_PCLC][j_PCLC]/1.0,plotpotential[i_PCLC][j_PCLC]/1.0,plotpotential[i_PCLC][j_PCLC]/1.0);
				//glColor3f(1.0,1.0,1.0);
				drawLoop((double)i_PCLC,(double)j_PCLC);
			}
		}
	}
#endif
	for(i_PCLC=0;i_PCLC<NCX;i_PCLC++)
	{
		for(j_PCLC=0;j_PCLC<NCY;j_PCLC++)
		{
			if(sp_state[i_PCLC][j_PCLC])
			{
				glColor3f(1.0,0.0,0.0);
				drawLoop((double)i_PCLC,(double)j_PCLC);
			}
		}
	}

	//printf("\n\t%lf\tx=%lf\ty=%lf",t,x,y);
	glColor3f(0.1,0.1,0.9);
	//drawCircle(25,75);
	drawCircle(xglob*NCX,yglob*NCY);



	//printf("\n\t\tnum_spike = %d",num_spike);
	//printf("\n\t\tepx=%lf\tepy=%lf",estimated_pos[0],estimated_pos[1]);
	glColor3f(0.1,0.8,0.8);
	drawCross(estimated_pos[0]*NCX,estimated_pos[1]*NCY);

	difx = sp2-sp1;
	dify = sp4-sp3;		

	//C

	difMeanX=0.0;
	difMeanY=0.0;

	//Shift this and add to the end of it
	for(i_PCLC=0;i_PCLC<FILT_DIF-1;i_PCLC++)
	{
		difSmoothX[i_PCLC]=difSmoothX[i_PCLC+1];
		difSmoothY[i_PCLC]=difSmoothY[i_PCLC+1];
	}

	difSmoothX[FILT_DIF-1]=difx;
	difSmoothY[FILT_DIF-1]=dify;

	for(i_PCLC=0;i_PCLC<FILT_DIF;i_PCLC++)
	{
		difMeanX+=difSmoothX[i_PCLC];
		difMeanY+=difSmoothY[i_PCLC];
	}

	difMeanX/=FILT_DIF;
	difMeanY/=FILT_DIF;

	//cdiffx=difx;
	//cdiffy=dify;

	cdiffx=difMeanX;
	cdiffy=difMeanY;

	if(xglob > 0.01f && xglob < 1.0f )
	{
		//DataValue[0] = xglob * 3.3f;
	}
	//DataValue[1] = 3.3f;
	//
	////if(DataValue[0] > 0.01f && DataValue[0] < 3.3f)
	////{
	//	ULStat = cbVOut (BoardNum, 0, Gain, DataValue[0], Options);
	//}

	//ULStat = cbVOut (BoardNum, 1, Gain, DataValue[1], Options);

}

#endif
