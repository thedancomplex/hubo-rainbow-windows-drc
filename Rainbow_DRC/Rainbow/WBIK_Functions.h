#ifndef WBIK_FUNCTIONS_H
#define WBIK_FUNCTIONS_H

#include "CommonDefinition.h"

//typedef unsigned int BOOL;
//#define TRUE		1
//#define FALSE		0

#define DRC_BI_MODE	1
#define DRC_QUAD_MODE	2

// #define PI			3.141592653589793f
#define D2R			0.017453292519943f
#define R2D			57.295779513082323f
#define MARGIN		D2R

#define DT			0.005f

#define gAcc		9.81f

//------------------------------ feedback control gains
#define KP1		1600.f				// wn^2, 
#define KD1		80.f				// 2*zeta*wn

#define KP2		400.f				
#define KD2		40.f

#define KP3		0.f				
#define KD3		40.f


//---------------------------------------------- for walking pattern
#define WTG_MODE					3		// WTG_MODE = 0 : statically stable region is co


//------------------------------ link parameters
#define CX_PEL		-0.0192f // -0.0171f
#define CY_PEL		0.
#define CZ_PEL		0.1330f  // 0.128f

#define CX_TOR		-0.0115f
#define CY_TOR		0.f
#define CZ_TOR		 0.1347f

#define CX_UARM		0.0044f
#define CY_UARM		-0.01f
#define CZ_UARM		-0.089f

#define CX_LARM		-0.0288f
#define CY_LARM		0.0422f
#define CZ_LARM		-0.1802f

#define CX_HAND		-0.0001f
#define CY_HAND		0.0051f
#define CZ_HAND		-0.0537f

#define CX_ULEG		0.016f   //0.0155f
#define CY_ULEG		0.0056f  //-0.0071f
#define CZ_ULEG		-0.1216f //-0.1162f

#define CX_LLEG		0.0133f  //0.0121f
#define CY_LLEG		0.0104f  //0.0156f
#define CZ_LLEG		-0.2157f  //-0.2247f

#define CX_FOOT		0.0174f  //0.0122f
#define CY_FOOT		-0.0013f //0.f
#define CZ_FOOT		-0.03f   //-0.0217f
	
#define L1_PEL		0.177f
#define L_LEG	  	0.33f
#define L2_PEL		0.173f
#define L1_TOR		0.4590f
#define L2_TOR		0.1845f
#define L_FOOT		0.10f  //0.14f //0.101f
#define OFF_ELB		0.03f
#define L_UARM		0.3f
#define L_LARM		0.314f
#define L_HAND		0.12f
#define L_STICK		0.09f

#define m_PEL		4.238f //3.465f
#define m_TOR		7.202f
#define m_ULEG		5.789f //5.222f
#define m_LLEG		1.575f //1.307f
#define m_FOOT		2.725f //2.6983f
#define m_UARM		2.6373f
#define m_LARM		1.318f
#define m_LHAND		0.2f //1.2f
#define m_RHAND		0.2f //1.4f
#define m_TOTAL		(2.f*(m_ULEG+m_LLEG+m_UARM+m_LARM+m_FOOT)+m_LHAND+m_RHAND+m_TOR+m_PEL)

#define m_dummy		0.001f
#define idummy		(float)1e-6
#define iLUA11		0.0339f
#define iLUA22		0.0335f
#define iLUA33		0.0036f
#define iLLA11		0.0153f
#define iLLA22		0.0136f
#define iLLA33		0.0224f
#define iLH11		0.0033f
#define iLH22		0.0033f
#define iLH33		0.0006f 

#define iRUA11		0.0339f
#define iRUA22		0.0335f
#define iRUA33		0.0036f
#define iRLA11		0.0153f
#define iRLA22		0.0136f
#define iRLA33		0.0224f
#define iRH11		0.0033f
#define iRH22		0.0033f
#define iRH33		0.0006f 


//-------- initial position in degree
#define OFFSET_RSR	-10.f
#define OFFSET_LSR	10.f
#define OFFSET_REB	-20.f //-20.f
#define OFFSET_LEB	-20.f //-20.f
//---------------


//--------------------------------------------- joint constraints
#define JOINT_LIMIT_DUTY	0.2f
#define JOINT_LIMIT_RANGE	(5.f*D2R)
#define	QP_MAX			12.f // rad/s

#define WSTmax          1.06f  // 61.f*D2R
#define WSTmin          -1.06f // -61.f*D2R
#define WSTpmax         6.28f
#define WSTppmax        62.8f

#define RSPmax          4.71f
#define RSPmin          -4.71f
#define RSPpmax         6.28f
#define RSPppmax        62.8f

#define RSRmax          (20.f*D2R)
#define RSRmin          -3.14f
#define RSRpmax         6.28f
#define RSRppmax        62.8f

#define RSYmax          (165.f*D2R)
#define RSYmin          (-165.f*D2R)
#define RSYpmax         6.28f
#define RSYppmax        62.8f

#define REBmax          0.f
#define REBmin          -2.96f
#define REBpmax         6.28f
#define REBppmax        62.8f

#define RWYmax          (155.f*D2R)
#define RWYmin          (-155.f*D2R)
#define RWYpmax         6.28f
#define RWYppmax        62.8f

#define RWPmax			(120.f*D2R)
#define RWPmin			(-220.f*D2R) //(-187.f*D2R)
#define RWPpmax			6.28f
#define RWPppmax		62.8f

#define RWY2max         (160.f*D2R)
#define RWY2min         (-160.f*D2R)
#define RWY2pmax         6.28f
#define RWY2ppmax        62.8f

#define LSPmax          RSPmax
#define LSPmin          RSPmin
#define LSPpmax         RSPpmax
#define LSPppmax        RSPppmax

#define LSRmax          -(RSRmin)
#define LSRmin          -(RSRmax)
#define LSRpmax         RSRpmax
#define LSRppmax        RSRppmax

#define LSYmax          -(RSYmin)
#define LSYmin          -(RSYmax)
#define LSYpmax         RSYpmax
#define LSYppmax        RSYppmax

#define LEBmax          REBmax
#define LEBmin          REBmin
#define LEBpmax         REBpmax
#define LEBppmax        REBppmax

#define LWYmax          -(RWYmin)
#define LWYmin          -(RWYmax)
#define LWYpmax         RWYpmax
#define LWYppmax        RWYppmax

#define LWPmax			RWPmax
#define LWPmin			RWPmin
#define LWPpmax			RWPpmax
#define	LWPppmax		RWPppmax

#define LWY2max          -(RWY2min)
#define LWY2min          -(RWY2max)
#define LWY2pmax         RWY2pmax
#define LWY2ppmax        RWY2ppmax

#define RHYmax          (90.f*D2R) // 32.f*D2R
#define RHYmin          (-90.f*D2R)  // -32.f*D2R
#define RHYpmax         6.28f
#define RHYppmax        62.8f

#define RHRmax          0.36f  
#define RHRmin          -0.6f  
#define RHRpmax         6.28f
#define RHRppmax        62.8f

#define RHPmax          1.6f
#define RHPmin          -1.6f //
#define RHPpmax         6.28f
#define RHPppmax        62.8f

#define RKNmax          2.6f
#define RKNmin          0.f
#define RKNpmax         6.28f
#define RKNppmax        62.8f

#define RAPmax          1.91f
#define RAPmin          -1.91f
#define RAPpmax         6.28f
#define RAPppmax        62.8f

#define RARmax          1.43f
#define RARmin          -1.43f
#define RARpmax         6.28f
#define RARppmax        62.8f

#define LHYmax          RHYmax
#define LHYmin          RHYmin
#define LHYpmax         RHYpmax
#define LHYppmax        RHYppmax

#define LHRmax          -(RHRmin)
#define LHRmin          -(RHRmax)
#define LHRpmax         RHRpmax
#define LHRppmax        RHRppmax

#define LHPmax          RHPmax
#define LHPmin          RHPmin
#define LHPpmax         RHPpmax
#define LHPppmax        RHPppmax

#define LKNmax          RKNmax
#define LKNmin          RKNmin
#define LKNpmax         RKNpmax
#define LKNppmax        RKNppmax

#define LAPmax          RAPmax
#define LAPmin          RAPmin
#define LAPpmax         RAPpmax
#define LAPppmax        RAPppmax

#define LARmax          -(RARmin)
#define LARmin          -(RARmax)
#define LARpmax         RARpmax
#define LARppmax		RARppmax


#define NKYmin			-180.f
#define NKYmax			180.f
#define NK1min			-30.f
#define NK1max			70.f
#define NK2min			-45.f
#define NK2max			45.f

//---------------------------------------------------------------- for upper body motion window
#define RECOVERY_TIME	100.f

#define WIND_SIZE_WST	67		// pmax/ppmax/dt -> 최고 속도에서 최대로 감속할 수 있는 window size
#define WIND_SIZE_RSP	67
#define WIND_SIZE_RSR	67
#define WIND_SIZE_RSY	67
#define WIND_SIZE_REB	67
#define WIND_SIZE_RWY	67
#define WIND_SIZE_RWP	67
#define WIND_SIZE_LSP	67
#define WIND_SIZE_LSR	67
#define WIND_SIZE_LSY	67
#define WIND_SIZE_LEB	67
#define WIND_SIZE_LWY	67
#define WIND_SIZE_LWP	67
#define WIND_SIZE_BP	67
#define WIND_SIZE_MAX	67		// window 크기 중 가장 큰 값
#define WIND_SIZE_QPEL	WIND_SIZE_MAX
#define WIND_SIZE_DPCZ	WIND_SIZE_MAX


typedef struct {
	float wind_WST[WIND_SIZE_WST];
	float wind_RSP[WIND_SIZE_RSP];
	float wind_RSR[WIND_SIZE_RSR];
	float wind_RSY[WIND_SIZE_RSY];
	float wind_REB[WIND_SIZE_REB];
	float wind_RWY[WIND_SIZE_RWY];
	float wind_RWP[WIND_SIZE_RWP];
	float wind_LSP[WIND_SIZE_LSP];
	float wind_LSR[WIND_SIZE_LSR];
	float wind_LSY[WIND_SIZE_LSY];
	float wind_LEB[WIND_SIZE_LEB];
	float wind_LWY[WIND_SIZE_LWY];
	float wind_LWP[WIND_SIZE_LWP];
	float wind_BP[WIND_SIZE_BP];
	float wind_qPEL[WIND_SIZE_QPEL][5];
	float wind_dPCz[WIND_SIZE_DPCZ];

	float dNK_3x1[4];
	float dRF_5x1[6];
	float dLF_5x1[6];

	float maxWST;
	float maxRSP;
	float maxRSR;
	float maxRSY;
	float maxREB;
	float maxRWY;
	float maxRWP;
	float maxLSP;
	float maxLSR;
	float maxLSY;
	float maxLEB;
	float maxLWY;
	float maxLWP;

	float minWST;
	float minRSP;
	float minRSR;
	float minRSY;
	float minREB;
	float minRWY;
	float minRWP;
	float minLSP;
	float minLSR;
	float minLSY;
	float minLEB;
	float minLWY;
	float minLWP;

	unsigned int index_maxWST;
	unsigned int index_maxRSP;
	unsigned int index_maxRSR;
	unsigned int index_maxRSY;
	unsigned int index_maxREB;
	unsigned int index_maxRWY;
	unsigned int index_maxRWP;
	unsigned int index_maxLSP;
	unsigned int index_maxLSR;
	unsigned int index_maxLSY;
	unsigned int index_maxLEB;
	unsigned int index_maxLWY;
	unsigned int index_maxLWP;

	unsigned int index_minWST;
	unsigned int index_minRSP;
	unsigned int index_minRSR;
	unsigned int index_minRSY;
	unsigned int index_minREB;
	unsigned int index_minRWY;
	unsigned int index_minRWP;
	unsigned int index_minLSP;
	unsigned int index_minLSR;
	unsigned int index_minLSY;
	unsigned int index_minLEB;
	unsigned int index_minLWY;
	unsigned int index_minLWP;

	unsigned int head_WST;
	unsigned int head_RSP;
	unsigned int head_RSR;
	unsigned int head_RSY;
	unsigned int head_REB;
	unsigned int head_RWY;
	unsigned int head_RWP;
	unsigned int head_LSP;
	unsigned int head_LSR;
	unsigned int head_LSY;
	unsigned int head_LEB;
	unsigned int head_LWY;
	unsigned int head_LWP;
	unsigned int head_BP;
	unsigned int head_qPEL;
	unsigned int head_dPCz;

	unsigned int tail_WST;
	unsigned int tail_RSP;
	unsigned int tail_RSR;
	unsigned int tail_RSY;
	unsigned int tail_REB;
	unsigned int tail_RWY;
	unsigned int tail_RWP;
	unsigned int tail_LSP;
	unsigned int tail_LSR;
	unsigned int tail_LSY;
	unsigned int tail_LEB;
	unsigned int tail_LWY;
	unsigned int tail_LWP;
	unsigned int tail_BP;
	unsigned int tail_qPEL;
	unsigned int tail_dPCz;
} QUB_WIND;

int PushQubWindow(float qPEL0, float qPELx, float qPELy, float qPELz, float dPCz, 
				     float wst, float rsp, float rsr, float rsy, float reb, float rwy, float rwp,
				     float lsp, float lsr, float lsy, float leb, float lwy, float lwp,
					 float dNKY, float dNK1, float dNK2,
					 float dRF1, float dRF2, float dRF3, float dRF4, float dRF5,
					 float dLF1, float dLF2, float dLF3, float dLF4, float dLF5,
					 float body_pitch);
int InitQubWindow(float qPEL0, float qPELx, float qPELy, float qPELz, float dPCz, 
				     float wst, float rsp, float rsr, float rsy, float reb, float rwy, float rwp,
				     float lsp, float lsr, float lsy, float leb, float lwy, float lwp,
					 float dNKY, float dNK1, float dNK2,
					 float dRF1, float dRF2, float dRF3, float dRF4, float dRF5,
					 float dLF1, float dLF2, float dLF3, float dLF4, float dLF5,
					 float body_pitch);
//-----------------------------------------------------------




//---------------------------------------------- for walking pattern
#define WTG_MODE					3		// WTG_MODE = 0 : statically stable region is considered in the sway motion
											// WTG_MODE = 1 : statically stable region is not considered in the sway motion
									
#define dFORWARD_MAX				0.4f
#define dYAW_MAX					0.5235988f  // 30degree
#define STEP_BUNDLE_RING_SIZE		20
#define FOOT_SIZE					0.22f
#define RFSP						-1
#define LFSP						1
#define DFSP						2
#define RHSP						-1
#define LHSP						1
#define DHSP						2
#define T_STEP_MAX					5.f
#define T_STEP_MIN					0.5f
#define T_DFSP						0.1f
#define WALK_TRAJ_SIZE				2000
#define dT_IDLE					1.f
//#define ZMPXMAX					0.14f
//#define ZMPXMIN					-0.08f
//#define ZMPYMAX					0.1685f
//#define ZMPYMIN					-0.1685f
#define ZMP_SAG_MARGIN			0.1f
#define ZMP_FRON_MARGIN			0.f
#define ZMP_OFFSET				0.05f //0.013f
#define dLIFT					0.05f
#define MOVING_FOOT_CUTOFF		1e-4
#define K_SFSP					516.6295f		// Nm/rad
#define L_COM					0.595f	
#define K_PELVIS_ROLL			11000.f     //15000.f
#define K_PELVIS_YAW			9000.f


#define PWM_SIGN_LSP				-1.f
#define PWM_SIGN_LSR				-1.f
#define PWM_SIGN_LSY				-1.f
#define PWM_SIGN_LEB				-1.f
#define PWM_SIGN_LWY				-1.f
#define PWM_SIGN_LWP				-1.f
#define PWM_SIGN_LWY2				-1.f

#define PWM_SIGN_RSP				-1.f
#define PWM_SIGN_RSR				-1.f
#define PWM_SIGN_RSY				-1.f
#define PWM_SIGN_REB				-1.f
#define PWM_SIGN_RWY				-1.f
#define PWM_SIGN_RWP				-1.f
#define PWM_SIGN_RWY2				-1.f

#define PWM_SIGN_RHY				-1.f
#define PWM_SIGN_RHR				-1.f
#define PWM_SIGN_RHP				-1.f
#define PWM_SIGN_RKN				-1.f
#define PWM_SIGN_RAP				-1.f
#define PWM_SIGN_RAR				-1.f

#define PWM_SIGN_LHY				-1.f
#define PWM_SIGN_LHR				-1.f
#define PWM_SIGN_LHP				-1.f
#define PWM_SIGN_LKN				-1.f
#define PWM_SIGN_LAP				-1.f
#define PWM_SIGN_LAR				-1.f


//-----------------------------
typedef struct {
	float dForward;
	float dYaw;
	float ROT;
	unsigned int Nsteps;
	unsigned int n_stepped;
	unsigned int pseudo_step;
	float Tstep;
	float offsetL;
	float offsetR;
	char putTogether;
	char halfFlag;
} STEP_BUNDLE;


typedef struct {
	unsigned int head;
	unsigned int tail;
	STEP_BUNDLE step_bundles[STEP_BUNDLE_RING_SIZE];
} STEP_BUNDLE_RING;


typedef struct {
	char isEmpty;
	int supporting_foot; // Right:-1, Left:1, Both:2
	float pRF_3x1[4];
	float pLF_3x1[4];
	float yRF;
	float yLF;
	float Tstep;
} FOOTSTEP;


typedef struct {
	float pCOM_3x1[WALK_TRAJ_SIZE][4];
	float pRF_3x1[WALK_TRAJ_SIZE][4];
	float pLF_3x1[WALK_TRAJ_SIZE][4];
	float qRF_4x1[WALK_TRAJ_SIZE][5];
	float qLF_4x1[WALK_TRAJ_SIZE][5];
	float yPEL[WALK_TRAJ_SIZE];
	float temp[WALK_TRAJ_SIZE];	
	unsigned int length;
	unsigned int i;
} WALK_TRAJ;

int PushSB(float dForward, float dYaw, float ROT, unsigned int Nsteps, float Tstep, char putTogether);
int StepPositioner(void);
int StepPositioning(unsigned int bundle_no, int buffer_no);
int CheckFootCollision(float *pRF_3x1, float yRF, float *pLF_3x1, float yLF, char supporting, float *offset_out);
int WTG(int mode);  // Walking Trajectory Generator
int ComTZ(float t0, float t1, float t2, float pCOM0, float vCOM0, float pCOM2, float vCOM2, float pCOMz, float zmp_max, float zmp_min, int mode, 
		  float pCOM_result[][4], int index, float *vCOM_result, unsigned int *n_result, float *dt1_result, float *dt2_result); // COM trajectory satisfying ZMP
int ASZ(float t0, float t1, float t2, float pCOM0, float vCOM0, float pCOM2, float vCOM2, float pCOMz, float *coeff_result_6x1); // coefficients of analytical solution to ZMP equation with a step ZMP input
int ASZ2(float t0, float t1, float t2, float pCOM0, float vCOM0, float zmp2, float vCOM2, float pCOMz, float *coeff_result_6x1);
int FTG3(float *pGoal_3x1, float yGoal, float *pLast_3x1, float yLast, float t0, float t1, float t2, float t3, char isStandingStep, char isSupportingFoot , float pResult[][4], float yResult[]); // Foot Trajectory Generator
int poly5(float ti, float tf, float yi, float yf, float ypi, float ypf, float yppi, float yppf, float *coeff_result_6x1);	// 5th order polynomial, y(t)=coeff(1) + coeff(2)*t + coeff(3)*t^2 + coeff(4)*t^3 + coeff(5)*t^4 + coeff(6)*t^5
int LoadWalkingDemo(int WalkingNo);
int InitStepBundleRing(void);
int CloseWBWalking(void);
int CheckIdling(void);
int InsertSB(float dForward, float dYaw, float ROT, unsigned int Nsteps, float Tstep, char putTogether);
//---------------------------------------------


//------------------------------- for the offline test
#define OFFLINE_LENGTH			10000
#define OFFLINE_MAX_CHANGES		100
typedef struct{
	float drvPEL[OFFLINE_LENGTH][4];
	float dpCOMx[OFFLINE_LENGTH];	
	float dpCOMy[OFFLINE_LENGTH];	
	float dpRFx[OFFLINE_LENGTH];
	float dpRFy[OFFLINE_LENGTH];
	float dpRFz[OFFLINE_LENGTH];
	float dpLFx[OFFLINE_LENGTH];
	float dpLFy[OFFLINE_LENGTH];
	float dpLFz[OFFLINE_LENGTH];
	float dpRHx[OFFLINE_LENGTH];
	float dpRHy[OFFLINE_LENGTH];
	float dpRHz[OFFLINE_LENGTH];
	float dpLHx[OFFLINE_LENGTH];
	float dpLHy[OFFLINE_LENGTH];
	float dpLHz[OFFLINE_LENGTH];
	float dpPELz[OFFLINE_LENGTH];
	unsigned int i;	
	unsigned int length;
	unsigned int n_changes;
	unsigned int i_change[OFFLINE_MAX_CHANGES][2];
	int des_fsp[OFFLINE_MAX_CHANGES];
	float offsetZMPs[OFFLINE_LENGTH];
	float offsetZMPf[OFFLINE_LENGTH];
} OFFLINE_TRAJ;

//------------------------------------ for online walking pattern
typedef struct{
	float dpCOMx;
	float dpCOMy;
	float dpRFx;
	float dpRFy;
	float dpRFz;
	float dpLFx;
	float dpLFy;
	float dpLFz;
	float dpRHx;
	float dpRHy;
	float dpRHz;
	float dpLHx;
	float dpLHy;
	float dpLHz;
	float dangRFz;
	float dangLFz;
	float dangRHz;
	float dangLHz;
	float offset_RAR;
	float offset_LAR;
	float offset_RAP;
	float offset_LAP;
	float offset_RHP;
	float offset_LHP;
	float offset_RHR;
	float offset_LHR;
	float offset_RHY;
	float offset_LHY;
	float dqWST;
} ONLINE_PATTERN;

int LoadOfflineTraj(int mode);
int LoadControllerParamter(void);
int LoadControllerParamter_DRC(char drc_walking_mode);
int WBIK_FreqTest(unsigned int n);
int WBIK_Offline_DRC(unsigned int n);
int WBIK_DRC_steering(unsigned int n);
int WBIK_DRC_Kirk_Quad(unsigned int n);  
int WBIK_DRC_Kirk_Biped(unsigned int n);  
int WBIK_DRC_steering_override(unsigned int n);

#define COP_CUTOFF		30.f   // rad/s
#define SSP_AP_CUTOFF	200.f   // rad/s
#define SSP_AR_CUTOFF	200.f   // rad/s
#define DSP_F_CUTOFF	200.f   // rad/s
#define DSP_S_CUTOFF	200.f   // rad/s
#define AOC_P_GAIN		1.f   // Ankle orientation controller gain
#define AOC_I_GAIN		4.f
#define AOC_D_GAIN		10.f
#define ILC_P_GAIN		0.00002f // Initial leg length controller gain
#define ILC_I_GAIN		0.00001f
#define ILC_D_GAIN		0.0002f
#define N1_TRANSITION	25
#define N2_TRANSITION	25
#define N3_TRANSITION	400

#define WEIGHT_PEL_SQRT_INV		1.f  //0.01f  // 1/sqrt(W) = 1/sqrt(10000)
#define WEIGHT_PCZ_SQRT_INV		1.f 
#define WEIGHT_WST_SQRT_INV		1.f  //0.0316f // 1/sqrt(100)

#define GLOBAL		0
#define LOCAL		1
//-------------------------------


#define X_34		1
#define Y_34		2
#define Z_34		3
#define Q1_34		4
#define Q2_34		5
#define Q3_34		6
#define Q4_34		7
#define WST_34		8
#define	RHY_34		9
#define RHR_34		10
#define RHP_34		11
#define	RKN_34		12
#define RAP_34		13
#define RAR_34		14
#define LHY_34		15
#define LHR_34		16
#define LHP_34		17
#define LKN_34		18
#define LAP_34		19
#define LAR_34		20
#define RSP_34		21
#define RSR_34		22
#define RSY_34		23
#define REB_34		24
#define RWY_34		25
#define RWP_34		26
#define LSP_34		27
#define LSR_34		28
#define LSY_34		29
#define LEB_34		30
#define LWY_34		31
#define LWP_34		32
#define RWY2_34		33
#define LWY2_34		34


#define X_33		1
#define Y_33		2
#define Z_33		3
#define WX_33		4
#define WY_33		5
#define WZ_33		6
#define WST_33		7
#define	RHY_33		8
#define RHR_33		9
#define RHP_33		10
#define	RKN_33		11
#define RAP_33		12
#define RAR_33		13
#define LHY_33		14
#define LHR_33		15
#define LHP_33		16
#define LKN_33		17
#define LAP_33		18
#define LAR_33		19
#define RSP_33		20
#define RSR_33		21
#define RSY_33		22
#define REB_33		23
#define RWY_33		24
#define RWP_33		25
#define LSP_33		26
#define LSR_33		27
#define LSY_33		28
#define LEB_33		29
#define LWY_33		30
#define LWP_33		31
#define RWY2_33		32
#define LWY2_33		33


int InitGlobalMotionVariables(void);
int FreeGlobalMotionVariables(void);

int UpdateGlobalMotionVariables(void);

int LoadMocapData2(int MocapNo);	// for preview window
int ClearMocapData2(void);		// for preview window

int FKine_Whole(void);
int FKine_COM(const float *Q_34x1, float *pCOM_3x1);
int FKine_Hand(char ref_coord, const float *Q_34x1, float *pRH_3x1, float *qRH_4x1, float *pLH_3x1, float *qLH_4x1);
int FKine_Foot(char ref_coord, const float *Q_34x1, float *pRF_3x1, float *qRF_4x1, float *pLF_3x1, float *qLF_4x1);
int FKine_Ankle(char ref_coord, const float *Q_34x1, float *pRANK_3x1, float *qRANK_4x1, float *pLANK_3x1, float *qLANK_4x1);
int FKine_Stick(char ref_coord, const float *Q_34x1, float *pRS_3x1, float *qRS_4x1, float *pLS_3x1, float *qLS_4x1);
int FKine_Wrist(char ref_coord, const float *Q_34x1, float *pRWR_3x1, float *qRWR_4x1, float *pLWR_3x1, float *qLWR_4x1);

int QT2DC(const float *qt_4x1, float **DC_3x3);		// convert a quaternion to a direction cosine matrix
int DC2QT(const float **DC_3x3, float *qt_4x1);		// convert a direction cosine matrix to a quaternion
int QTdel(const float *des_qt_4x1, const float *qt_4x1, float *result_3x1); // delta quaternion
int Wmatrix(const float *w_3x1, float **wmatrix_4x4);
int Wq(int ref, const float *qt_4x1, float **Wq_3x4); // quaternion rate matrix. (ref==0: global frame, ref==1:body frame)
int Qq(int ref, const float *qt_4x1, float **Qq_4x4); // quaternion matrix. (ref==0: global frame, ref==1:body frame)
int QThat(const float *qt_4x1, float **qthat_4x3);
int Skewsym(const float *vector_3x1, float **matrix_3x3);	// skew-symmetric cross product matrix
int QTcross(const float *QT1_4x1, const float *QT2_4x1, float *result_4x1);
int QTbar(const float *qt_4x1, float *result_4x1);

int RX(float theta, float **R_3x3);
int RY(float theta, float **R_3x3);
int RZ(float theta, float **R_3x3);

int RVALS(float x, float xp, float xpp_ref, float xpmax, float xppmax, float xmin, float xmax, float margin, float *xpp_result, float *bound_l, float *bound_u);	// Operational Range, Velocity and Acceleration Limit Strategy
int RVALS3(float x, float xp, float xpmax, float xppmax, float xmin, float xmax, float margin, float *bound_l, float *bound_u);	
int ZLS(float Cx, float Cxp, float Cxpp_ref, float Cz, float Czpp, float ZMPxmin, float ZMPxmax, float ZMPz, float margin, float *result_);	// ZMP-lmited COM controller

int WBIK_FreqTest(unsigned int n);
int UpdatePassiveCoord_DSP(void);
int UpdatePassiveCoord_SSP(int LorR); // left:LorR=1,  right:LorR=-1

void wberror(char error_text[]);

int derive3(float x0, float x1, float x2, float *xd1, float *xdd1, float dt);
int derive3QT(float *q_past_4x1, float *q_4x1, float *q_next_4x1, float *w_result_3x1, float dt);

int get_steer(float p1_3x1[], float p2_3x1[], float p3_3x1[], float pcSteer_3x1[], float *rSteer, float qSteer_4x1[]);
int get_steerAng(float qSteer_4x1[], float qLH_4x1[], float *angSteer);
float limitDuty(float limit, float duty);
int GetZMP_WB(FT rf, FT lf, FT rw, FT lw);

int getGravityTorque(const float *Q_34x1, float *gravity_33x1);
int getFricCompen(const float *Qp_33x1, float *viscous_33x1);
int getDuty4JointLimit(float Q_34x1[], float duty_joint_limit_33x1[]);
int WBIK_DRC_ladder_climbing(unsigned int n);
int WBIK_DRC_quad(unsigned int n);

int checkJointRange(float Q_34x1[]);
float torque2duty(int joint_no, float torque);
int LoadParameter_ComputedTorque(void);
float one_cos(float t_sec, float mag, float T_sec);
int one_cos_orientation(float t_sec, const float *qt0_4x1, const float *qt1_4x1, float T_sec, float *result_4x1);
int QT2RV(const float *qt_4x1, float* rv); // quaternion to rotational vector
int RV2QT(const float *rv, float *qt_4x1); // rotational vector to quaternion
//------------------------------------------------------------------

#endif




