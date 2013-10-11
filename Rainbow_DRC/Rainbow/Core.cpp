#include <math.h>
#include "RTX.h"
#include "CAN.h"
#include "APIs.h"
#include "WBIK_Functions.h"	// by Inhyeok
#include "nrutil.h"		// by Inhyeok
#include "Kirk.h"

#pragma warning(once:4244)

CANRING _can_ring;

unsigned int sendIndex;
unsigned int receiveIndex;
unsigned int receiveChannelIndex0;
unsigned int receiveChannelIndex1;

// for RTX
PSHARED_DATA	pSharedMemory;			// shared memory data
HANDLE			hSharedMemory;			// shared memory handler
HANDLE			hTimerCAN;				// timer handle for CAN sending
HANDLE			hTimer;					// timer handle for upper body control
HANDLE			hTimer1;				// timer handle for lower body control
unsigned long	timeIndex;				// timer index for TimerHandler
unsigned long	timeIndex1;				// timer index for TimerHandler1

// for CAN
MYCAN			CAN[2];					// CAN channel handler
MYMB			MB[MBSIZE];				// CAN message box

// for joints and sensors
JOINT			Joint[NO_OF_JOINT];		// Joint struct variable
FT				FTSensor[NO_OF_FT];		// FT sensor struct variablef
IMU				IMUSensor[NO_OF_IMU];	// IMU sensor struct variable
unsigned char	ReadSensorFlag;			// read sensor data flag
unsigned char	NullSensorFlag;			// nulling sensor flag

float			ZMP[6];					// current ZMP value
float			FilteredZMP[6];			// filtered ZMP
float			InitZMP[2];				// initial ZMP
unsigned char	ZMPInitFlag;			// ZMP init. mode
float			ZMPInitPos[22];			// ZMP initial position (21 and 22 is for IMU offset store)


// !--------------Structure Variable CDI PROG---------------------!
// for CTM, CDI
unsigned long	ctm_timeIndex;			// timer index for TimerHandler1 - cdi
CTM				ctm;
// !--------------------------------------------------------------!


// for walking pattern
float			WalkingStep[4];				// walking step length
unsigned char	WalkingStartStop;			// walking start: 0x02, walking stop prepare: 0x01, walking stop: 0x00
WALKING_INFO	WalkingInfoSway;			// walking information for sway
WALKING_INFO	WalkingInfo[2][4];			// walking information for right(0), left(1) foot: 0->X, 1->Y, 2->Z
unsigned char	WalkingPhaseNextSway;		// next walking phase(next phase, not current phase) for sway
unsigned char	WalkingPhaseNext[4];		// next walking phase(next phase, not current phase) 0:x-direction, 1:y-direction, 2:z-direction, 3:yawing

float			dampingGain[6];			// damping controller gain
float			dspGain;				// DSP controller gain

bool			dampingControlOnOff;
bool			dspControlOnOff;
bool			vibrationControlOnOff;
//---------------for hyoin DRC IMU-------------
//----------Variable Declare---
float A[5][5];
float z[4][2];
float Qk[5][5];
float R[4][4];
float x[5][2];
float xp[5][2];
float P[5][5];
float Pp[5][5];
float K[5][4];
float H[5][4];
float T[5][5];
float iT[5][5];
float q0, q1, q2, q3;
float scalehyo, deter, KalmanR;
float gyr_dps_x_hubo[2];
float gyr_dps_y_hubo[2];
float gyr_dps_z_hubo[2];
float changetoR=0.017453292519943f;
float changetoD=57.295779513082323f;
float alpha23=1./(1.+2.*3.141592653589793*2.3*0.005);
float alpha015=1./(1.+2.*3.141592653589793*0.1*0.005);//0.28
float Kalmanindex1, Kalmanindex2;
int kalmanflag=0;
int dynamicstate, accstate, gyrostate, gyrostate2, jerkstate;
float kalman_roll, kalman_pitch, kalman_yaw;
float acc_roll, acc_pitch;
float acc_roll_LPF, acc_pitch_LPF;
float offseted_roll_angle, offseted_pitch_angle;
float g_norm, g_norm_2, g_norm_old, w_norm, jerk_g, myX;
float acc_ang_x_hubo, acc_ang_y_hubo;
float acc_ang_x_hubo_LPFed, acc_ang_y_hubo_LPFed;
float gyr_ang_x_hubo_HPFed, gyr_ang_y_hubo_HPFed, gyr_ang_z_hubo_HPFed;
float complementary_roll, complementary_pitch, complementary_yaw;
float ang_x_hubo, ang_y_hubo, ang_z_hubo;
float ang_x_hubo_rad, ang_y_hubo_rad, ang_z_hubo_rad;
float temp_x_gyro, temp_y_gyro, temp_z_gyro;
float temp_hyo_xacc, temp_hyo_yacc, temp_hyo_zacc;
float temp_hyo_xacc_old, temp_hyo_yacc_old, temp_hyo_zacc_old;
float acc_g_x_hubo, acc_g_y_hubo, acc_g_z_hubo;
float acc_g_x_hubo_old, acc_g_y_hubo_old, acc_g_z_hubo_old;
float rad_x_hubo, rad_y_hubo, rad_z_hubo;
float offset_roll, offset_pitch;
float ang_x_hubo_old, ang_y_hubo_old, ang_z_hubo_old;
float gyro_sum_roll, gyro_sum_pitch;

//----------Function Declare-----
float zaccCaliDRC(float yacc, float zacc);
float yaccCaliDRC(float yacc);
float xaccCaliDRC(float xacc);

//----------------------------------------------------end of hyoin IMU DRC-------------

//--------- jw
float jungwootset;
unsigned char DemoFlag;
unsigned char ZMPInitPosFlag;
unsigned char	finger_control_mode;


float	NKY_err_short_float[3];
float	HAND_err_short_float[5];
float	HAND_err_short_float1[5];


//-------------- Inhyeok
extern float _Q_34x1[35], _Q0_34x1[35];
extern char _FKineUpdatedFlag;
extern char _PassiveUpdatedFlag;
unsigned int _OfflineCount;
extern float _Xhat_SSP_F_2x1[3];
extern float _Xhat_DSP_F_2x1[3], _Xhat_DSP_F_3x1[4];
extern float _Xhat_SSP_S_2x1[3];
extern float _Xhat_DSP_S_2x1[3];
extern float _Xhat_DSP_F_4x1[5];
extern float _Xhat_DSP_S_4x1[5];
extern float _Xhat_SSP_F_4x1[5];
extern float _Xhat_SSP_S_4x1[5];
extern float _log_temp[LOG_DATA_SIZE];
char _WalkReadyAngleSetFlag;
char _WBDemoIsWaitingFlag;
extern char _DRC_walking_mode;
extern float _friction_para[34][2];
//-------------- end:Inhyeok


//Handshake
float OLD_UpperMovement[NO_OF_JOINT_MC];
float OLD_UpperMovement1[NO_OF_JOINT_MC];

static unsigned char StepChangeFlag = 0x00;
static float 	tempStep = 0.0f;
static float lastStep = tempStep;
static float lastStep2 = tempStep;
static float lastStep3 = tempStep;
static float lastStep4 = tempStep;


void InitParameters(void);
bool LoadParameter(void);
bool SaveParameter(void);
bool SetJointParameter(unsigned char _jointID, JOINT _joint);
bool GetJointParameter(unsigned char _jointID, PJOINT _joint);
bool PrintJointParameter(unsigned char _jointID);
bool CheckDeviceCAN(unsigned int _boardID);
void FETDriverOnOff(unsigned int _boardID, unsigned char _enable);
bool SendRunStopCMD(unsigned char _boardID, unsigned char _runStop);
void GainSetting(unsigned int _jointID);
bool GoToLimitPos(unsigned int _jointID);
bool CheckCurrentState(unsigned int _jointID);
bool ZeroEncoder(unsigned char _boardID);
unsigned char GetMotorChannel(unsigned int _jointID);
unsigned int GetHDReduction(unsigned int _jointID);
unsigned int GetPulleyDrive(unsigned int _jointID);
unsigned int GetPulleyDriven(unsigned int _jointID);
unsigned char GetCANChannel(unsigned int _jointID);
unsigned char GetBoardID(unsigned char _jointID);
bool PositionLimitOnOff(unsigned char _jointID, unsigned char _mode);
bool Beep(unsigned char _mode);
bool GetStatus(unsigned char _boardID, unsigned char* _status);

// for JMC setting
bool SetMotorGain(JOINT _joint);
bool SetDeadZone(JOINT _joint);
bool SetEncoderResolution(JOINT _joint);
bool SetJamPwmSturation(JOINT _joint);
bool SetMaxAccVel(JOINT _joint);
bool SetControlMode(JOINT _joint);
bool SetHomeSearchParameter(JOINT _joint);
bool SetHomeMaxVelAcc(JOINT _joint);
bool SetUpperPositionLimit(JOINT _joint);
bool SetLowerPositionLimit(JOINT _joint);
bool SetErrorBound(JOINT _joint);

// Request parameters
bool RequestParameters(JOINT* _joint);
bool SetMoveJointAngle(unsigned char _jointID, float _angle, float _msTime, unsigned char _mode);
bool SetMoveJointAngleFunc(unsigned char _jointID, float _angle, float _userData[5], float _msTime[3], unsigned char _func, unsigned char _mode);
void MoveJointAngle(unsigned char _canChannel);
bool CheckBoardStatus(JOINT* _joint);
void FingerControlModeChange(unsigned int _boardID, unsigned char _enable);
char FingerMove(int mag,int vel,int time, int start, int end, float *result);

//-------------- Inhyeok
int WholeBodyOffline_Demo(unsigned char canChannel);
void GotoWalkReadyPosFast(void);
void Back2InitAngle(void);
int PushLogData(void);
void SetDRCQuadrupedReadyPos(void);
void SetDRCBipedReadyPos(void);
void SetDRCWideBipedReadyPos(void);
void SetDRCHomePos(void);
void SetDRCSteeringReadyPos(void);
void SetDRCLadderClimbingReadyPos(void);
unsigned short NeckDynamixelPos(float angle);
void NeckDynamixelSetSpeed(float percent_NKY, float percent_NK1, float percent_NK2);  // max speed = 100%, no-change = 0
void NeckDynamixelHome(void);
//-------------- end:Inhyeok

bool SetMoveTaskPos(WALKING_INFO* _walkingInfo, float _pos, float _msTime, float _msDelayTime[2], float _userData[5], unsigned char _mode, unsigned char _function);
void MoveTaskPos(WALKING_INFO* _walkingInfoSway, WALKING_INFO _walkingInfo[][4], JOINT _joint[], unsigned char _landingState, unsigned char _mode);
bool SetMoveTaskPosJointFF(JOINT *_joint, float _angle, float _msTime, float _msDelayTime[2], unsigned char _mode, unsigned char _function);
void MoveTaskPosJointFF(JOINT _joint[]);
void MoveJMC(unsigned char _boardID);
unsigned long SignConvention(long _input);
unsigned short SignConventionFinger(short _input,unsigned char _type);
void InverseKinematics(float _pos[6], float _angle[]);
void ForwardKinematics(float _angle[6], float _pos[]);
void ReadEncoder(unsigned char _canChannel);
void ReadFT(unsigned char _canChannel);
unsigned char GetZMP(FT _rightFootFTSensor, FT _leftFootFTSensor, WALKING_INFO _walkingInfo[2][4], float _zmp[], float _filteredZMP[]);
void ReadIMU(void);
bool RequestEncoder(unsigned char _boardID);
bool RequestCurrent(unsigned char _boardID);
bool RequestSensor(unsigned char _canChannel);
bool SetFTParameter(unsigned char _ftID, FT _ftSensor);
bool GetFTParameter(unsigned char _ftID, PFT _ftSensor);
bool NullFTSensor(unsigned char _ftID, unsigned char _mode);
bool NullFootAngleSensor(FT _ftSensor[]);
bool NullIMUSensor(unsigned char _imuID, unsigned char _mode);
bool PrintFTParameter(unsigned char _ftID);
bool SetIMUParameter(unsigned char _imuID, IMU _imuSensor);
bool GetIMUParameter(unsigned char _imuID, PIMU _imuSensor);
bool PrintIMUParameter(unsigned char _imuID);
void ZMPInitialization(float _refZMP[2], float _zmp[], IMU _imu, FT _rightFT, FT _leftFT,  WALKING_INFO _walkingInfo[][4], JOINT _joint[], unsigned char _command);
void GotoWalkReadyPos(void);
void GotoHomePos(void);
void GotoControlOffPos(void);
unsigned char LandingStateCheck(FT _ftSensor[], WALKING_INFO _walkingInfo[][4]);
void DampingControl(IMU _imuSensor[], JOINT _joint[], unsigned char _command);
void DSPControl(float _ZMP[], float _refZMP[2], WALKING_INFO _walkingInfo[][4], float _time, unsigned char _command, unsigned char _demo);
void VibrationControl(FT _ftSensor[], JOINT _joint[], unsigned char _command);
void UnevenControl();

// under development
void TestFunction(void);

// walking sequence
void TestProfileSequence(void);
unsigned int numberOfWalking;
unsigned int currentWalkingStep;
char walkingSceneNo;
float forwardWalkingSequence[80];
float sideWalkingSequence[80];
float rotationWalkingSequence[80];
float swaySequence[80];
void PredefinedProfileSequence(void);

// motion and demo functions
bool GoToDemoOneLegSupport(void);
bool GoToDemoDownAndUp(unsigned char _rightLeft, unsigned char _times);
bool GoToDemoHomePosition(void);
bool GoToDemoDoubleLegSupport(void);
bool GoToDemoDoubleHomePosition(void);

// Handshake
float Torsion_mass_spring_damper_Mx(float Mx, int zero);
float Torsion_mass_spring_damper_My(float My, int zero);
float Wrist_mass_spring_damper_Fz(float Fz, int zero);
void ShakeHands();

// !-----------------------Core Init CDI PROG-------------------------!
bool Hubo2JointCTMPWMCommand2ch(JOINT _joint0, JOINT _joint1);
bool Hubo2JointPWMSignChange(JOINT _joint);
bool Hubo2RightArmCCTM(PCTM _ctm);
// !------------------------------------------------------------------!

bool SetMoveZMPInitAngle(unsigned char _jointID, float _angle, float _msTime);
bool SetMoveZMPInitPos(WALKING_INFO* _walkingInfo, float _pos, float _msTime);
void MoveZMPInitFF(JOINT _joint[], WALKING_INFO _walkingInfo[][4]);

/******************************************************************************/
void InitParameters(void)
{
	unsigned char i, j;
	FILE* zmpinit_file;

	
	// SharedMemory initialize
	pSharedMemory->CommandFlag = NO_ACT;
	pSharedMemory->MotorControlMode = CTRLMODE_NONE;
	pSharedMemory->LandingFz = 100.0f;

	// Send CAN
	pSharedMemory->HUBO_FLAG_SEND_CAN = 0;

	// global variables initialize	
	ReadSensorFlag = 0x00;
	NullSensorFlag = 0x00;
	
	InitZMP[0] = 10.0f;
	InitZMP[1] = 0.0f;

	for(i=0 ; i<6 ; i++) ZMP[i] = FilteredZMP[i] = 0.0f;

	zmpinit_file = fopen("c:\\ZMPInitPos.dat", "rb");	
	if(zmpinit_file == NULL)
	{
		ZMPInitPos[0] = 0.016973f;
		ZMPInitPos[1] = -0.005439f;
		ZMPInitPos[2] = 0.000823f;
		ZMPInitPos[3] = 0.0f;

		ZMPInitPos[4] = 0.016973f;
		ZMPInitPos[5] = -0.005439f;
		ZMPInitPos[6] = -0.000823f;
		ZMPInitPos[7] = 0.0f;

		ZMPInitPos[8] = 0.0f;
		ZMPInitPos[9] = 0.0f;
		ZMPInitPos[10] = -0.075601f;
		ZMPInitPos[11] = 0.0f;
		ZMPInitPos[12] = 0.178811f;
		ZMPInitPos[13] = -0.665374f;

		ZMPInitPos[14] = 0.0f;
		ZMPInitPos[15] = 0.0f;
		ZMPInitPos[16] = -0.075601f;
		ZMPInitPos[17] = 0.0f;
		ZMPInitPos[18] = 0.0f;
		ZMPInitPos[19] = -0.543949f;

		ZMPInitPos[20] = 0.0f;
		ZMPInitPos[21] = 0.0f;
		RtWprintf(L"\n>>> File open error(InitParameters)..!!");
	}
	else
	{
		fread(ZMPInitPos, sizeof(float)*22, 1, zmpinit_file);
		IMUSensor[CENTERIMU].RollOffset = ZMPInitPos[20];
		IMUSensor[CENTERIMU].PitchOffset = ZMPInitPos[21];
		fclose(zmpinit_file);
	}


	for(i=RIGHT ; i<=LEFT ; i++)
	{
		for(j=X ; j<=Yaw ; j++)
		{
			WalkingInfo[i][j].RefPatternCurrent		= 0.0f;
			WalkingInfo[i][j].RefPatternDelta		= 0.0f;
			WalkingInfo[i][j].RefPatternToGo		= 0.0f;
			WalkingInfo[i][j].RefPatternInitial		= 0.0f;
			WalkingInfo[i][j].GoalTimeCount			= 0;
			WalkingInfo[i][j].CurrentTimeCount		= 0;
			WalkingInfo[i][j].DelayTimeCount[0]		= 0;
			WalkingInfo[i][j].DelayTimeCount[1]		= 0;
			WalkingInfo[i][j].MoveFlag				= false;
			WalkingInfo[i][j].ControlDSPZMP			= 0.0f;
		}
	}

	for(i=X ; i<=Yaw ; i++) WalkingStep[j] = 0.0f;

	timeIndex =	0;
	timeIndex1 = 0;

	// initialize damping controller gain
	dampingGain[0] = 0.2f;		dampingGain[3] = 0.2f;
	dampingGain[1] = 1.0f;		dampingGain[4] = 1.0f;
	dampingGain[2] = 0.4f;		dampingGain[5] = 0.5f;
	
	// initialize DSP controller gain
	dspGain = 0.8f;
	
	dampingControlOnOff = true;
	dspControlOnOff = true;
	vibrationControlOnOff = true;

	DemoFlag = NO_DEMO;
}
/******************************************************************************/


/******************************************************************************/
bool LoadParameter(void)
{
	unsigned char i;
	unsigned char err_count = 0;
	
	for(i=RHY ; i<NO_OF_JOINT ; i++)
	{
		Joint[i].JMC				= GetBoardID(i);//Parameters.JMC[i];
		Joint[i].JointID			= i;
		Joint[i].CAN_channel		= GetCANChannel(i);//Parameters.CAN_channel[i];
		Joint[i].Ref_txdf			= REF_BASE_TXDF+Joint[i].JMC;//Parameters.Ref_txdf[i];
		Joint[i].Motor_channel		= GetMotorChannel(i);//Parameters.Motor_channel[i]-1;//Parameters.Motor_channel[i];
		Joint[i].HDReduction		= GetHDReduction(i);
		Joint[i].Pulley_drive		= GetPulleyDrive(i);
		Joint[i].Pulley_driven		= GetPulleyDriven(i);
	}


	if(RequestParameters(&Joint[RHY]) == false) 		err_count++;
	if(RequestParameters(&Joint[RHR]) == false) 		err_count++;
	if(RequestParameters(&Joint[RHP]) == false) 		err_count++;
	if(RequestParameters(&Joint[RKN]) == false) 		err_count++;
	if(RequestParameters(&Joint[RAR]) == false) 		err_count++;
	if(RequestParameters(&Joint[RAP]) == false) 		err_count++;

	if(RequestParameters(&Joint[LHY]) == false) 		err_count++;
	if(RequestParameters(&Joint[LHR]) == false) 		err_count++;
	if(RequestParameters(&Joint[LHP]) == false) 		err_count++;
	if(RequestParameters(&Joint[LKN]) == false) 		err_count++;
	if(RequestParameters(&Joint[LAR]) == false) 		err_count++;
	if(RequestParameters(&Joint[LAP]) == false) 		err_count++;

	if(RequestParameters(&Joint[RSP]) == false) 		err_count++;
	if(RequestParameters(&Joint[RSR]) == false) 		err_count++;
	if(RequestParameters(&Joint[RSY]) == false) 		err_count++;
	if(RequestParameters(&Joint[REB]) == false) 		err_count++;
	if(RequestParameters(&Joint[RWY]) == false) 		err_count++;
	if(RequestParameters(&Joint[RWP]) == false) 		err_count++;

	if(RequestParameters(&Joint[LSP]) == false) 		err_count++;
	if(RequestParameters(&Joint[LSR]) == false) 		err_count++;
	if(RequestParameters(&Joint[LSY]) == false) 		err_count++;
	if(RequestParameters(&Joint[LEB]) == false) 		err_count++;
	if(RequestParameters(&Joint[LWY]) == false) 		err_count++;
	if(RequestParameters(&Joint[LWP]) == false) 		err_count++;

	if(RequestParameters(&Joint[WST]) == false) 		err_count++;
//	if(RequestParameters(&Joint[NKY]) == false) 		err_count++;
//	if(RequestParameters(&Joint[NK1]) == false) 		err_count++;
//	if(RequestParameters(&Joint[NK2]) == false) 		err_count++;

	if(RequestParameters(&Joint[RF1]) == false) 		err_count++;
	if(RequestParameters(&Joint[RF2]) == false) 		err_count++;
	if(RequestParameters(&Joint[RF3]) == false) 		err_count++;
//	if(RequestParameters(&Joint[RF4]) == false) 		err_count++;
//	if(RequestParameters(&Joint[RF5]) == false) 		err_count++;
//
	if(RequestParameters(&Joint[LF1]) == false) 		err_count++;
	if(RequestParameters(&Joint[LF2]) == false) 		err_count++;
//	if(RequestParameters(&Joint[LF3]) == false) 		err_count++;
//	if(RequestParameters(&Joint[LF4]) == false) 		err_count++;
//	if(RequestParameters(&Joint[LF5]) == false) 		err_count++;

	FTSensor[RFFT].Controller_NO	= FT0;
	FTSensor[RFFT].CAN_channel		= CAN0;
	FTSensor[RFFT].CutOffFeq		= 3.f;     //Hz
	FTSensor[RFFT].SF_Pitch			= 1.0f/0.0255f;
	FTSensor[RFFT].SF_Roll			= 1.0f/0.0255f;
		
	FTSensor[LFFT].Controller_NO	= FT1;
	FTSensor[LFFT].CAN_channel		= CAN0;
	FTSensor[LFFT].CutOffFeq		= 3.f;     //Hz
	FTSensor[LFFT].SF_Pitch			= 1.0f/0.0255f;
	FTSensor[LFFT].SF_Roll			= 1.0f/0.0255f;
		
	IMUSensor[CENTERIMU].Controller_NO	= IMU0;
	IMUSensor[CENTERIMU].CAN_channel	= CAN0;
	// #4
	//IMUSensor[CENTERIMU].RollOffset	= 1.1f;
	//IMUSensor[CENTERIMU].PitchOffset	= -0.7f;
	// #5
	//IMUSensor[CENTERIMU].RollOffset	= 0.85f;
	//IMUSensor[CENTERIMU].PitchOffset	= 0.0f;
	// #6
	//IMUSensor[CENTERIMU].RollOffset		= 0.5f;
	//IMUSensor[CENTERIMU].PitchOffset	= 0.0f;
	//IMUSensor[CENTERIMU].RollOffset		= 0.0f;
	//IMUSensor[CENTERIMU].PitchOffset	= 0.0f;

	FTSensor[RWFT].Controller_NO	= FT2;
	FTSensor[RWFT].CAN_channel		= CAN1;
	FTSensor[RWFT].CutOffFeq		= 3.0f;  // Hz

	FTSensor[LWFT].Controller_NO	= FT3;
	FTSensor[LWFT].CAN_channel		= CAN1;
	FTSensor[LWFT].CutOffFeq		= 3.0f;  // Hz
		

	if(err_count == 0) RtWprintf(L"\n>>> Parameters are loaded successfully..!!");
	else RtWprintf(L"\n>>> Loading parameters is failed..!!");

	return true;
}
/******************************************************************************/




/******************************************************************************/
bool SaveParameter(void)
{
	unsigned char i;
	SAVE_PARAMETERS	Parameters;
	FILE *fp;
	
	fp = fopen("c:\\parameter.par", "wb");
	
	for(i=RHY ; i<NO_OF_JOINT ; i++)
	{
		Parameters.JMC[i]			= Joint[i].JMC;
		Parameters.CAN_channel[i]	= Joint[i].CAN_channel;
		Parameters.Ref_txdf[i]		= Joint[i].Ref_txdf;
		Parameters.Motor_channel[i]	= Joint[i].Motor_channel;
		Parameters.HDReduction[i]	= Joint[i].HDReduction;
		Parameters.Pulley_drive[i]	= Joint[i].Pulley_drive;
		Parameters.Pulley_driven[i]	= Joint[i].Pulley_driven;
		Parameters.Encoder_size[i]	= Joint[i].Encoder_size;
		Parameters.PPR[i]			= Joint[i].PPR;
	}

	for(i=RFFT ; i<NO_OF_FT ; i++)
	{
		Parameters.FTControllerNO[i]	= FTSensor[i].Controller_NO;
		Parameters.FTCANchannel[i]		= FTSensor[i].CAN_channel;
		Parameters.FTCutOff[i]			= FTSensor[i].CutOffFeq;
	}

	for(i=CENTERIMU ; i<NO_OF_IMU ; i++)
		{
			Parameters.IMUControllerNO[i]	= IMUSensor[i].Controller_NO;
			Parameters.IMUCANchannel[i]		= IMUSensor[i].CAN_channel;
		}

	if(fp == NULL) { RtWprintf(L"\n>>> File open error(SaveParameter)..!!"); return false; }
	else
	{
		fwrite(&Parameters, sizeof(SAVE_PARAMETERS), 1, fp);
		fclose(fp);
		RtWprintf(L"\n>>> File save success..!!");
		return true;
	}
}
/******************************************************************************/




/******************************************************************************/
bool SetMotorGain(JOINT _joint)
{
	unsigned char tempData[8];	
	
	if(_joint.JointID >= NO_OF_JOINT) { RtWprintf(L"\n>>> Wrong Joint ID(MotorGainSetting)..!!"); return false; }
	else
	{
		tempData[0] = _joint.JMC;

		switch (_joint.JointID)
		{
		case RHY:
		case RHP:
		case RKN:
		case RAP:
		case LHY:
		case LHP:
		case LKN:
		case LAP:
		case LAR:
		case RSP:
		case RSY:
		case LSP:
		case LSY:
		case RWY:
		case LWY:
		case NKY:
			if(_joint.MotorControlMode == 0x00) tempData[1] = SetPosGainA;
			else tempData[1] = SetTorqueGainA;
			break;
		case RHR:
		case RAR:
		case LHR:
		case RSR:
		case REB:
		case LSR:
		case LEB:
		case RWP:
		case LWP:
		case NK1:
		case NK2:
			if(_joint.MotorControlMode == 0x00) tempData[1] = SetPosGainB;
			else tempData[1] = SetTorqueGainB;
			break;
		}
	
		RtWprintf(L"\n>>> Joint(%d) motor gain is set..!!", _joint.JointID);
		if(_joint.MotorControlMode == 0x00)
		{
			tempData[2] = (unsigned char)(_joint.Position_Kp & 0xFF);
			tempData[3] = (unsigned char)((_joint.Position_Kp>>8) & 0xFF);	
			tempData[4] = (unsigned char)(_joint.Position_Ki & 0xFF);
			tempData[5] = (unsigned char)((_joint.Position_Ki>>8) & 0xFF);
			tempData[6] = (unsigned char)(_joint.Position_Kd & 0xFF);
			tempData[7] = (unsigned char)((_joint.Position_Kd>>8) & 0xFF);

			RtWprintf(L"\n>>> Kp:%d\t Ki:%d\t Kd:%d", _joint.Position_Kp, _joint.Position_Ki, _joint.Position_Kd);
		}
		else
		{
			tempData[2] = (unsigned char)(_joint.Torque_Kp & 0xFF);
			tempData[3] = (unsigned char)((_joint.Torque_Kp>>8) & 0xFF);	
			tempData[4] = (unsigned char)(_joint.Torque_Ki & 0xFF);
			tempData[5] = (unsigned char)((_joint.Torque_Ki>>8) & 0xFF);
			tempData[6] = (unsigned char)(_joint.Torque_Kd & 0xFF);
			tempData[7] = (unsigned char)((_joint.Torque_Kd>>8) & 0xFF);

			RtWprintf(L"\n>>> Kp:%d\t Ki:%d\t Kd:%d", _joint.Position_Kp, _joint.Position_Ki, _joint.Position_Kd);
		}

		PushCANMsg(_joint.CAN_channel, CMD_TXDF, tempData, 8, 0);
		Sleep(20);

		
		return true;
	}
}
/******************************************************************************/




/******************************************************************************/
bool SetJointParameter(unsigned char _jointID, JOINT _joint)
{
	if(_jointID >= NO_OF_JOINT) { RtWprintf(L"\n>>> Wrong Joint ID(SetJointParameter)..!!"); return false; }
	else { Joint[_jointID] = _joint; return true; }
}
/******************************************************************************/




/******************************************************************************/
bool GetJointParameter(unsigned char _jointID, JOINT* _joint)
{
	if(_jointID > NO_OF_JOINT) { RtWprintf(L"\n>>> Wrong joint ID(GetJointParameter)..!!"); return false; }
	else { *_joint = Joint[_jointID];	return true; }
}
/******************************************************************************/




/******************************************************************************/
bool PrintJointParameter(unsigned char _jointID)
{
	if(_jointID > NO_OF_JOINT) { RtWprintf(L"\n>>> Wrong joint ID(PrintJointParameter)..!!"); return false; }
	else
	{
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> Joint(%d) parameter information", _jointID);
		RtWprintf(L"\n>>> RefAngleCurrent: %d", (int)(Joint[_jointID].RefAngleCurrent*1000.0f));
		RtWprintf(L"\n>>> RefAngleToGo: %d", (int)(Joint[_jointID].RefAngleToGo*1000.0f));
		RtWprintf(L"\n>>> RefAngleInitial: %d", (int)(Joint[_jointID].RefAngleInitial*1000.0f));
		RtWprintf(L"\n>>> GoalTimeCount: %d", Joint[_jointID].GoalTimeCount);
		RtWprintf(L"\n>>> CurrentTimeCount: %d", Joint[_jointID].CurrentTimeCount);
		RtWprintf(L"\n>>> EncoderValue: %d", Joint[_jointID].EncoderValue);
		RtWprintf(L"\n>>> MoveFlag: %d", Joint[_jointID].MoveFlag);
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> PPR: %d", (int)(Joint[_jointID].PPR*1000.0f));
		RtWprintf(L"\n>>> JMC: %d", Joint[_jointID].JMC);
		RtWprintf(L"\n>>> Motor_channel: %d", Joint[_jointID].Motor_channel);
		RtWprintf(L"\n>>> CAN_channel: %d", Joint[_jointID].CAN_channel);
		RtWprintf(L"\n>>> Ref_txdf: %d", Joint[_jointID].Ref_txdf);
		RtWprintf(L"\n>>> Positive_dir: %d", Joint[_jointID].Positive_dir);
		RtWprintf(L"\n>>> Offset_angle: %d", (int)(Joint[_jointID].Offset_angle*1000.0f));
		RtWprintf(L"\n>>> Limit_rev: %d", Joint[_jointID].Limit_rev);
		RtWprintf(L"\n>>> Position_Kp: %d", Joint[_jointID].Position_Kp);
		RtWprintf(L"\n>>> Position_Kd: %d", Joint[_jointID].Position_Kd);
		RtWprintf(L"\n>>> Position_Ki: %d", Joint[_jointID].Position_Ki);
		RtWprintf(L"\n>>> Torque_Kp: %d", Joint[_jointID].Torque_Kp);
		RtWprintf(L"\n>>> Torque_Kd: %d", Joint[_jointID].Torque_Kd);
		RtWprintf(L"\n>>> Torque_Ki: %d", Joint[_jointID].Torque_Ki);
		RtWprintf(L"\n>>> Encoder_size: %d", Joint[_jointID].Encoder_size);
		RtWprintf(L"\n>>> HDReduction: %d", Joint[_jointID].HDReduction);
		RtWprintf(L"\n>>> Pulley_drive: %d", Joint[_jointID].Pulley_drive);
		RtWprintf(L"\n>>> Pulley_driven: %d", Joint[_jointID].Pulley_driven);
		
		GetJointParameter(_jointID, &pSharedMemory->Joint);

		return true;
	}
}
/******************************************************************************/




/******************************************************************************/
bool CheckDeviceCAN(unsigned int _boardID)
{
	bool result = false;
	unsigned char tempData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
	tempData[0] = _boardID;
	tempData[1] = NameInfo;	
	tempData[2] = (unsigned char)(INT_TIME & 0x00FF);

	switch(_boardID)
	{
		case JMC0:
			PushCANMsg(Joint[RHY].CAN_channel, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(Joint[RHY].CAN_channel, NAME0_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC0..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC0..!! "); result = false; }
			break;
		case JMC1:
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME1_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC1..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC1..!! "); result = false; }
			break;
		case JMC2:
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME2_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC2..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC2..!! "); result = false; }
			break;
		case JMC3:
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME3_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC3..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC3..!! "); result = false; }
			break;
		case JMC4:
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME4_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC4..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC4..!! "); result = false; }
			break;
		case JMC5:
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME5_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC5..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC5..!! "); result = false; }
			break;
		case JMC6:
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME6_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC6..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC6..!! "); result = false; }
			break;
		case JMC7:
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME7_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC7..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC7..!! "); result = false; }
			break;
		case JMC8:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME8_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC8..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC8..!! "); result = false; }
			break;
		case JMC9:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME9_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC9..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC9..!! "); result = false; }
			break;
		case JMC10:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME10_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC10..!! "); result = true;	}
			else { RtWprintf(L"\n>>> CAN communication error with JMC10..!! "); result = false; }
			break;
		case JMC11:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME11_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with JMC11..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with JMC11..!! ");	result = false;	}
			break;
		case EJMC0:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			PushCANMsg(1, CMD_TXDF, tempData, 8, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME_E0_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with EJMC0..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with EJMC0..!! ");	result = false;	}
			break;
		case EJMC1:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME_E1_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with EJMC1..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with EJMC1..!! ");	result = false;	}
			break;
		case EJMC2:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME_E2_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with EJMC2..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with EJMC2..!! "); result = false; }
			break;
		case EJMC3:
			//tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME_E3_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with EJMC3..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with EJMC3..!! ");	result = false;	}
			break;
		case EJMC4:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME_E4_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with EJMC4..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with EJMC4..!! ");	result = false;	}
			break;
		case EJMC5:
			tempData[2] = (unsigned char)(INT_TIME1 & 0x00FF);
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME_E5_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with EJMC5..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with EJMC5..!! ");	result = false;	}
			break;
		case FT0:
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME_FT0_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with FT0..!! "); result = true;	}
			else { RtWprintf(L"\n>>> CAN communication error with FT0..!! "); result = false; }
			break;
		case FT1:
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME_FT1_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with FT1..!! "); result = true;	}
			else { RtWprintf(L"\n>>> CAN communication error with FT1..!! "); result = false; }
			break;
		case IMU0:
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME_IMU0_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with IMU0..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with IMU0..!! "); result = false; }
			break;
		case IMU1:
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME_IMU1_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with IMU1..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with IMU1..!! "); result = false; }
			break;
		case IMU2:
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(0, NAME_IMU2_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with IMU2..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with IMU2..!! "); result = false; }
			break;
		case FT2:
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME_FT2_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with FT2..!! "); result = true;	}
			else { RtWprintf(L"\n>>> CAN communication error with FT2..!! "); result = false; }
			break;
		case FT3:
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtSleep(10);
			if( CanReceiveMsg(1, NAME_FT3_RXDF) == ERR_OK) { RtWprintf(L"\n>>> CAN communication OK with FT3..!! "); result = true; }
			else { RtWprintf(L"\n>>> CAN communication error with FT3..!! "); result = false; }
			break;
		default:
			RtWprintf(L"\n>>> Wrong board ID(CheckDeviceCAN)..!!");
			result = false;
			break;
	}	

	return result;
}
/******************************************************************************/



/******************************************************************************/
void FETDriverOnOff(unsigned int _boardID, unsigned char _enable)
{
	unsigned char tempData[8];
	bool _enableFlag;

	if(_enable == 0x01) _enableFlag = true;
	else _enableFlag = false;

	tempData[0] = _boardID;
	tempData[1] = HipEnable;
	tempData[2] = _enable;		// enable : 1, disable : 0

	switch(_boardID)
	{
		case JMC0:
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			if(_enableFlag) RtWprintf(L"\n>>> JMC0 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC0 FET driver is disabled..!!");
			break;
		case JMC1:
			if(_enableFlag) RtWprintf(L"\n>>> JMC1 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC1 FET driver is disabled..!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC2:
			if(_enableFlag) RtWprintf(L"\n>>> JMC2 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC2 FET driver is disabled..!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC3:
			if(_enableFlag) RtWprintf(L"\n>>> JMC3 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC3 FET driver is disabled..!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC4:
			if(_enableFlag) RtWprintf(L"\n>>> JMC4 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC4 FET driver is disabled..!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC5:
			if(_enableFlag) RtWprintf(L"\n>>> JMC5 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC5 FET driver is disabled..!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC6:
			if(_enableFlag) RtWprintf(L"\n>>> JMC6 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC6 FET driver is disabled..!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC7:
			if(_enableFlag) RtWprintf(L"\n>>> JMC7 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC7 FET driver is disabled..!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC8:
			if(_enableFlag) RtWprintf(L"\n>>> JMC8 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC8 FET driver is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC9:
			if(_enableFlag) RtWprintf(L"\n>>> JMC9 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC9 FET driver is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC10:
			if(_enableFlag) RtWprintf(L"\n>>> JMC10 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC10 FET driver is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC11:
			if(_enableFlag) RtWprintf(L"\n>>> JMC11 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> JMC11 FET driver is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC0:
			if(_enableFlag) RtWprintf(L"\n>>> EJMC0 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> EJMC0 FET driver is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC1:
			if(_enableFlag) RtWprintf(L"\n>>> EJMC1 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> EJMC1 FET driver is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC2:
			if(_enableFlag) RtWprintf(L"\n>>> EJMC2 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> EJMC2 FET driver is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC3:
			if(_enableFlag) RtWprintf(L"\n>>> EJMC3 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> 3JMCE FET driver is disabled!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC4:
			if(_enableFlag) RtWprintf(L"\n>>> EJMC4 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> EJMC4 FET driver is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC5:
			if(_enableFlag) RtWprintf(L"\n>>> EJMC5 FET driver is enabled..!!");
			else RtWprintf(L"\n>>> EJMC5 FET driver is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		default:
			RtWprintf(L"\n>>> Wrong board ID(FETDriverOnOff)..!!");
			break;
	}
	RtSleep(15);
}
/******************************************************************************/



/******************************************************************************/
bool SendRunStopCMD(unsigned char _boardID, unsigned char _runStop)
{
	unsigned char tempData[8];
	bool _runStopFlag;

	tempData[0] = _boardID;
	if(_runStop == 0x01) { tempData[1] = RunCMD; _runStopFlag = true; }
	else { tempData[1] = StopCMD; _runStopFlag = false; }

	switch(_boardID)
	{
		case JMC0:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC0 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC0 motor control is disabled!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC1:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC1 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC1 motor control is disabled!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC2:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC2 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC2 motor control is disabled!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC3:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC3 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC3 motor control is disabled!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC4:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC4 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC4 motor control is disabled!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC5:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC5 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC5 motor control is disabled!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC6:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC6 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC6 motor control is disabled!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC7:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC7 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC7 motor control is disabled!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC8:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC8 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC8 motor control is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC9:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC9 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC9 motor control is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC10:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC10 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC10 motor control is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case JMC11:
			if(_runStopFlag) RtWprintf(L"\n>>> JMC11 motor control is enabled..!!");
			else RtWprintf(L"\n>>> JMC11 motor control is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC0:
			if(_runStopFlag) RtWprintf(L"\n>>> EJMC0 motor control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC0 motor control is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC1:
			if(_runStopFlag) RtWprintf(L"\n>>> EJMC1 motor control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC1 motor control is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC2:
			if(_runStopFlag) RtWprintf(L"\n>>> EJMC2 motor control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC2 motor control is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC3:
			if(_runStopFlag) RtWprintf(L"\n>>> EJMC3 motor control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC3 motor control is disabled!!");
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC4:
			if(_runStopFlag) RtWprintf(L"\n>>> EJMC4 motor control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC4 motor control is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC5:
			if(_runStopFlag) RtWprintf(L"\n>>> EJMC5 motor control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC5 motor control is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		default:
			RtWprintf(L"\n>>> Wrong board ID(SendRunStopCMD)..!!");
			return false;
			break;
	}
	RtSleep(15);

	return true;
}
/******************************************************************************/




/******************************************************************************/
void GainSetting(unsigned int _jointID)
{
	unsigned char tempData[8];

	tempData[0] = Joint[_jointID].JMC;	
	if(Joint[_jointID].Motor_channel == 0) tempData[1] = 0x07;
	else tempData[1] = 0x08;

	tempData[2] = (unsigned char)(Joint[_jointID].Position_Kp & 0xFF);
	tempData[3] = (unsigned char)((Joint[_jointID].Position_Kp>>8) & 0xFF);
	tempData[4] = (unsigned char)(Joint[_jointID].Position_Ki & 0xFF);
	tempData[5] = (unsigned char)((Joint[_jointID].Position_Ki>>8) & 0xFF);
	tempData[6] = (unsigned char)(Joint[_jointID].Position_Kd & 0xFF);
	tempData[7] = (unsigned char)((Joint[_jointID].Position_Kd>>8) & 0xFF);
	
	PushCANMsg(Joint[_jointID].CAN_channel, CMD_TXDF, tempData, 8, 0);
	RtSleep(10);
}
/******************************************************************************/





/******************************************************************************/
bool GoToLimitPos(unsigned int _jointID)
{
	unsigned char tempData[8];
	int itemp;

	// pulse input
	//itemp = Joint[_jointID].Positive_dir*(Joint[_jointID].Offset_pulse + Joint[_jointID].Offset_rev*Joint[_jointID].Encoder_size);
	// angle input
	//itemp = (int)(Joint[_jointID].Positive_dir*(Joint[_jointID].Offset_angle*Joint[_jointID].PPR));
	itemp = (int)(Joint[_jointID].Offset_angle*Joint[_jointID].PPR);
	
	if (itemp < 0) itemp = (((-itemp) & 0x0007FFFF) | (1<<19));

	tempData[0] = Joint[_jointID].JMC;
	tempData[1] = GoLimitPos;

	switch(_jointID)
	{
	case RHY:
		tempData[2] = 0x10;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> RHY limit sensor searching..!!");
		break;
	case RHR:
		tempData[2] = 0x20;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> RHR limit sensor searching..!!");
		break;
	case RHP:
		tempData[2] = 0x11;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> RHP limit sensor searching..!!");
		break;
	case RKN:
		tempData[2] = 0x10;	
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> RKN limit sensor searching..!!");
		break;
	case RAP:
		tempData[2] = 0x10;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> RAP limit sensor searching..!!");
		break;
	case RAR:
		tempData[2] = 0x20;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> RAR limit sensor searching..!!");
		break;
	case LHY:
		tempData[2] = 0x11;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> LHY limit sensor searching..!!");
		break;
	case LHR:
		tempData[2] = 0x21;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> LHR limit sensor searching..!!");
		break;
	case LHP:
		tempData[2] = 0x11;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> LHP limit sensor searching..!!");
		break;
	case LKN:
		tempData[2] = 0x10;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> LKN limit sensor searching..!!");
		break;
	case LAP:
		tempData[2] = 0x10;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> LAP limit sensor searching..!!");
		break;
	case LAR:
		tempData[2] = 0x21;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> LAR limit sensor searching..!!");
		break;
	case WST:
		tempData[2] = 0x10;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> WST limit sensor searching..!!");
		break;
	case RSP:
		tempData[2] = 0x10;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> RSP limit sensor searching..!!");
		break;
	case RSR:
		tempData[2] = 0x20;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> RSR limit sensor searching..!!");
		break;
	case RSY:
		tempData[2] = 0x10;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> RSY limit sensor searching..!!");
		break;
	case REB:
		tempData[2] = 0x20;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> REB limit sensor searching..!!");
		break;
	case LSP:
		tempData[2] = 0x12;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> LSP limit sensor searching..!!");
		break;
	case LSR:
		tempData[2] = 0x22;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> LSR limit sensor searching..!!");
		break;
	case LSY:
		tempData[2] = 0x12;
		tempData[3] = ((itemp>>8) & (0xFF));
		tempData[4] = (itemp & 0xFF);
		tempData[5] = 0x00;
		tempData[6] = 0x00;
		tempData[7] = ((itemp>>16) & (0x0F));
		RtWprintf(L"\n>>> LSY limit sensor searching..!!");
		break;
	case LEB:
		tempData[2] = 0x22;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> LEB limit sensor searching..!!");
		break;
	case RWY:
		tempData[2] = 0x12;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> RWY limit sensor searching..!!");
		break;
	case RWP:
		tempData[2] = 0x22;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> RWP limit sensor searching..!!");
		break;
	case LWY:
		tempData[2] = 0x12;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> LWY limit sensor searching..!!");
		break;
	case LWP:
		tempData[2] = 0x22;
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> LWP limit sensor searching..!!");
		break;
	case NKY:
		/*tempData[2] = 0xF3;
		tempData[3] = 0;//itemp & 0xFF;
		tempData[4] = 0;//(itemp >> 8) & 0xFF;
		tempData[5] = 0;//itemp2 & 0xFF;
		tempData[6] = 0;//itemp3 & 0xFF;
		tempData[7] = 0x00;
		RtWprintf(L"\n>>> NK limit sensor searching..!!");
		*/
		NeckDynamixelHome();
		RtWprintf(L"\n>>> NK limit sensor searching..!!");
		break;
	case RWY2:
		/*
		tempData[2] = 0xF3;
		tempData[3] = 0;
		tempData[4] = 0;
		tempData[5] = 0;//((itemp>>8) & (0xFF));
		tempData[6] = 0;//(itemp & 0xFF);
		tempData[7] = 0;//(((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> RF limit sensor searching..!!");
		*/
		tempData[2] = 0x12; // DRC
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> RWY2 limit sensor searching..!!");
		break; 
	case RF2:  // gripper
		tempData[2] = 0x22; // DRC
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = 0;
		tempData[6] = 0;
		tempData[7] = 0;
		RtWprintf(L"\n>>> RFinger limit sensor searching..!!");
		break;
	case RF3: // triggerin finger
		tempData[2] = 0x32; // DRC
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = 0;
		tempData[6] = 0;
		tempData[7] = 0;
		RtWprintf(L"\n>>> RFinger limit sensor searching..!!");
		break;
	case RFA:
		tempData[2] = 0xE2; // DRC
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = 0;
		tempData[6] = 0;
		tempData[7] = 0;
		RtWprintf(L"\n>>> RFinger limit sensor searching..!!");
		break;
	case LWY2:
		/*
		tempData[2] = 0xF3;
		tempData[3] = 0;
		tempData[4] = 0;
		tempData[5] = 0;
		tempData[6] = 0;
		tempData[7] = 0;
		RtWprintf(L"\n>>> LF limit sensor searching..!!");
		*/
		tempData[2] = 0x12; // DRC
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = ((itemp>>8) & (0xFF));
		tempData[6] = (itemp & 0xFF);
		tempData[7] = (((itemp>>16) & (0x0F)) << 4);
		RtWprintf(L"\n>>> LWY2 limit sensor searching..!!");
		break;
	case LF2:
		tempData[2] = 0x22; // DRC
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = 0;
		tempData[6] = 0;
		tempData[7] = 0;
		RtWprintf(L"\n>>> LFinger limit sensor searching..!!");
		break;
	case LF3:
		tempData[2] = 0x32; // DRC
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = 0;
		tempData[6] = 0;
		tempData[7] = 0;
		RtWprintf(L"\n>>> LFinger limit sensor searching..!!");
		break;
	case LFA:
		tempData[2] = 0xE2; // DRC
		tempData[3] = 0x00;
		tempData[4] = 0x00;
		tempData[5] = 0;
		tempData[6] = 0;
		tempData[7] = 0;
		RtWprintf(L"\n>>> LFinger limit sensor searching..!!");
		break;
	default:
		RtWprintf(L"\n>>> Wrong joint ID(GoToLimitPos)..!!");
		return false;
		break;
	}

	PushCANMsg(Joint[_jointID].CAN_channel, CMD_TXDF, tempData, 8, 0);

	return true;	
}
/******************************************************************************/






/******************************************************************************/
bool GoToLimitPosAll(unsigned char _upperLower)
{
	// _upperLower
	// 0x00 : All
	// 0x01 : Lower
	// 0x02 : Upper

	unsigned char tempData[8];

	if( (_upperLower==0x00) || (_upperLower==0x01) )
	{
		// RHY and RHR
		tempData[0] = Joint[RHY].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[RHY].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// RHP
		tempData[0] = Joint[RHP].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[RHP].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// RKN
		tempData[0] = Joint[RKN].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[RKN].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// RAP and RAR
		tempData[0] = Joint[RAP].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[RAP].CAN_channel, CMD_TXDF, tempData, 8, 0);
		
		// LHY and LHR
		tempData[0] = Joint[LHY].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[LHY].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// LHP
		tempData[0] = Joint[LHP].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[LHP].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// LKN
		tempData[0] = Joint[LKN].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[LKN].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// LAP and LAR
		tempData[0] = Joint[LAP].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[LAP].CAN_channel, CMD_TXDF, tempData, 8, 0);
	}

	if( (_upperLower==0x00) || (_upperLower==0x02) )
	{
		// WST
		tempData[0] = Joint[WST].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[WST].CAN_channel, CMD_TXDF, tempData, 8, 0);
		
		// RSP and RSR
		tempData[0] = Joint[RSP].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[RSP].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// RSY and REB
		tempData[0] = Joint[RSY].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[RSY].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// RWY and RWP
		tempData[0] = Joint[RWY].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[RWY].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// RF1, 2, 3, 4, and 5
		tempData[0] = Joint[RF1].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[RF1].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// LSP and LSR
		tempData[0] = Joint[LSP].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[LSP].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// LSY and LEB
		tempData[0] = Joint[LSY].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[LSY].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// LWY and LWP
		tempData[0] = Joint[LWY].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[LWY].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// LF1, 2, 3, 4, and 5
		tempData[0] = Joint[LF1].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[LF1].CAN_channel, CMD_TXDF, tempData, 8, 0);

		// NKY, NK1 and NK2
		tempData[0] = Joint[NKY].JMC;	tempData[1] = GoLimitPos;	tempData[2] = 0xF3;
		tempData[3] = tempData[5] =tempData[6] = tempData[7] = 0x00;
		PushCANMsg(Joint[NKY].CAN_channel, CMD_TXDF, tempData, 8, 0);
	}

	return true;	
}
/******************************************************************************/





/******************************************************************************/
bool CheckCurrentState(unsigned int _jointID)
{
	unsigned char _temp;

	switch(_jointID)
	{
	case RHY:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RHY limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RHY limit sensor searching..failed!!"); return false; }
		break;
	case RHR:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RHR limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RHR limit sensor searching..failed!!"); return false; }
		break;
	case RHP:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RHP limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RHP limit sensor searching..failed!!"); return false; }
		break;
	case RKN:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RKN limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RKN limit sensor searching..failed!!"); return false; }
		break;
	case RAP:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RAP limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RHP limit sensor searching..failed!!"); return false; }
		break;
	case RAR:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RAR limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RAR limit sensor searching..failed!!"); return false; }
		break;
	case LHY:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LHY limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LHY limit sensor searching..failed!!"); return false; }
		break;
	case LHR:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[6];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LHR limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LHR limit sensor searching..failed!!"); return false; }
		break;
	case LHP:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LHP limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LHP limit sensor searching..failed!!"); return false; }
		break;
	case LKN:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LKN limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LKN limit sensor searching..failed!!"); return false; }
		break;
	case LAP:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LAP limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LAP limit sensor searching..failed!!"); return false; }
		break;
	case LAR:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[6];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LAR limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LAR limit sensor searching..failed!!"); return false; }
		break;
	case WST:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> WST limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> WST limit sensor searching..failed!!"); return false; }
		break;
	case RSP:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RSP limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RSP limit sensor searching..failed!!"); return false; }
		break;
	case RSR:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[6];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RSR limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RSR limit sensor searching..failed!!"); return false; }
		break;
	case RSY:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RSY limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RSY limit sensor searching..failed!!"); return false; }
		break;
	case REB:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[6];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> REB limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> REB limit sensor searching..failed!!"); return false; }
		break;
	case LSP:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LSP limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LSP limit sensor searching..failed!!"); return false; }
		break;
	case LSR:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[6];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LSR limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LSR limit sensor searching..failed!!"); return false; }
		break;
	case LSY:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LSY limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LSY limit sensor searching..failed!!"); return false; }
		break;
	case LEB:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[6];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LEB limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LEB limit sensor searching..failed!!"); return false; }
		break;
	case RWY:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RW limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RW limit sensor searching..failed!!"); return false; }
		break;
	case LWY:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LW limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LW limit sensor searching..failed!!"); return false; }
		break;
	case NKY:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> NK limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> NK limit sensor searching..failed!!"); return false; }
		break;
	case RF1:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> RF limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> RF limit sensor searching..failed!!"); return false; }
		break;
	case LF1:
		CanReceiveMsg(Joint[_jointID].CAN_channel, Joint[_jointID].JMC+STAT_BASE_RXDF);
		_temp = MB[GetBuffno(Joint[_jointID].JMC+STAT_BASE_RXDF)].data[5];
		if( (_temp & 0x03) == 0x01)	{ RtWprintf(L"\n>>> LF limit sensor searching..OK!!"); return true; }
		else { RtWprintf(L"\n>>> LF limit sensor searching..failed!!"); return false; }
		break;
	}

	RtWprintf(L"\n>>> Wrong joint ID(GoToLimitPos)..!!");
	return false;	
}
/******************************************************************************/




/******************************************************************************/
bool ZeroEncoder(unsigned char _boardID)
{	
	unsigned char tempData[8];
	int i;

	tempData[0] = _boardID;
	tempData[1] = EncZero;
	tempData[2] = 0;		// zero all encoders

	switch(_boardID)
	{
		case JMC0:
			Joint[RHY].RefAngleCurrent =	0;
			Joint[RHY].RefAngleToGo =		0;
			Joint[RHY].RefAngleInitial =	0;
			Joint[RHY].RefAngleFF =			0;
			Joint[RHY].GoalTimeCount =		0;
			Joint[RHY].CurrentTimeCount =	0;
			Joint[RHY].EncoderValue =		0;
			Joint[RHY].MoveFlag =			false;

			Joint[RHR].RefAngleCurrent =	0;
			Joint[RHR].RefAngleToGo =		0;
			Joint[RHR].RefAngleInitial =	0;
			Joint[RHR].RefAngleFF =			0;
			Joint[RHR].GoalTimeCount =		0;
			Joint[RHR].CurrentTimeCount =	0;
			Joint[RHR].EncoderValue =		0;
			Joint[RHR].MoveFlag =			false;
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC0(RHY, RHR) encoder zero..!!");
			break;
		case JMC1:
			Joint[RHP].RefAngleCurrent =	0;
			Joint[RHP].RefAngleToGo =		0;
			Joint[RHP].RefAngleInitial =	0;
			Joint[RHP].RefAngleFF =			0;
			Joint[RHP].GoalTimeCount =		0;
			Joint[RHP].CurrentTimeCount =	0;
			Joint[RHP].EncoderValue =		0;
			Joint[RHP].MoveFlag =			false;
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC1(RHP) encoder zero..!!");
			break;
		case JMC2:
			Joint[RKN].RefAngleCurrent =	0;
			Joint[RKN].RefAngleToGo =		0;
			Joint[RKN].RefAngleInitial =	0;
			Joint[RKN].RefAngleFF =			0;
			Joint[RKN].GoalTimeCount =		0;
			Joint[RKN].CurrentTimeCount =	0;
			Joint[RKN].EncoderValue =		0;
			Joint[RKN].MoveFlag =			false;
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC2(RKN) encoder zero..!!");
			break;
		case JMC3:
			Joint[RAP].RefAngleCurrent =	0;
			Joint[RAP].RefAngleToGo =		0;
			Joint[RAP].RefAngleInitial =	0;
			Joint[RAP].RefAngleFF =			0;
			Joint[RAP].GoalTimeCount =		0;
			Joint[RAP].CurrentTimeCount =	0;
			Joint[RAP].EncoderValue =		0;
			Joint[RAP].MoveFlag =			false;
			
			Joint[RAR].RefAngleCurrent =	0;
			Joint[RAR].RefAngleToGo =		0;
			Joint[RAR].RefAngleInitial =	0;
			Joint[RAR].RefAngleFF =			0;
			Joint[RAR].GoalTimeCount =		0;
			Joint[RAR].CurrentTimeCount =	0;
			Joint[RAR].EncoderValue =		0;
			Joint[RAR].MoveFlag =			false;
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC3(RAP, RAR) encoder zero..!!");
			break;
		case JMC4:
			Joint[LHY].RefAngleCurrent =	0;
			Joint[LHY].RefAngleToGo =		0;
			Joint[LHY].RefAngleInitial =	0;
			Joint[LHY].RefAngleFF =			0;
			Joint[LHY].GoalTimeCount =		0;
			Joint[LHY].CurrentTimeCount =	0;
			Joint[LHY].EncoderValue =		0;
			Joint[LHY].MoveFlag =			false;
			
			Joint[LHR].RefAngleCurrent =	0;
			Joint[LHR].RefAngleToGo =		0;
			Joint[LHR].RefAngleInitial =	0;
			Joint[LHR].RefAngleFF =			0;
			Joint[LHR].GoalTimeCount =		0;
			Joint[LHR].CurrentTimeCount =	0;
			Joint[LHR].EncoderValue =		0;
			Joint[LHR].MoveFlag =			false;
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC4(LHY, LHR) encoder zero..!!");
			break;
		case JMC5:
			Joint[LHP].RefAngleCurrent =	0;
			Joint[LHP].RefAngleToGo =		0;
			Joint[LHP].RefAngleInitial =	0;
			Joint[LHP].RefAngleFF =			0;
			Joint[LHP].GoalTimeCount =		0;
			Joint[LHP].CurrentTimeCount =	0;
			Joint[LHP].EncoderValue =		0;
			Joint[LHP].MoveFlag =			false;
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC5(LHP) encoder zero..!!");
			break;
		case JMC6:
			Joint[LKN].RefAngleCurrent =	0;
			Joint[LKN].RefAngleToGo =		0;
			Joint[LKN].RefAngleInitial =	0;
			Joint[LKN].RefAngleFF =			0;
			Joint[LKN].GoalTimeCount =		0;
			Joint[LKN].CurrentTimeCount =	0;
			Joint[LKN].EncoderValue =		0;
			Joint[LKN].MoveFlag =			false;
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC6(LKN) encoder zero..!!");
			break;
		case JMC7:
			Joint[LAP].RefAngleCurrent =	0;
			Joint[LAP].RefAngleToGo =		0;
			Joint[LAP].RefAngleInitial =	0;
			Joint[LAP].RefAngleFF =			0;
			Joint[LAP].GoalTimeCount =		0;
			Joint[LAP].CurrentTimeCount =	0;
			Joint[LAP].EncoderValue =		0;
			Joint[LAP].MoveFlag =			false;
			
			Joint[LAR].RefAngleCurrent =	0;
			Joint[LAR].RefAngleToGo =		0;
			Joint[LAR].RefAngleInitial =	0;
			Joint[LAR].RefAngleFF =			0;
			Joint[LAR].GoalTimeCount =		0;
			Joint[LAR].CurrentTimeCount =	0;
			Joint[LAR].EncoderValue =		0;
			Joint[LAR].MoveFlag =			false;
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC7(LAP, LAR) encoder zero..!!");
			break;
		case JMC8:
			Joint[RSP].RefAngleCurrent =	0;
			Joint[RSP].RefAngleToGo =		0;
			Joint[RSP].RefAngleInitial =	0;
			Joint[RSP].RefAngleFF =			0;
			Joint[RSP].GoalTimeCount =		0;
			Joint[RSP].CurrentTimeCount =	0;
			Joint[RSP].EncoderValue =		0;
			Joint[RSP].MoveFlag =			false;
			
			Joint[RSR].RefAngleCurrent =	0;
			Joint[RSR].RefAngleToGo =		0;
			Joint[RSR].RefAngleInitial =	0;
			Joint[RSR].RefAngleFF =			0;
			Joint[RSR].GoalTimeCount =		0;
			Joint[RSR].CurrentTimeCount =	0;
			Joint[RSR].EncoderValue =		0;
			Joint[RSR].MoveFlag =			false;
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC8(RSP, RSR) encoder zero..!!");
			break;
		case JMC9:
			Joint[RSY].RefAngleCurrent =	0;
			Joint[RSY].RefAngleToGo =		0;
			Joint[RSY].RefAngleInitial =	0;
			Joint[RSY].RefAngleFF =			0;
			Joint[RSY].GoalTimeCount =		0;
			Joint[RSY].CurrentTimeCount =	0;
			Joint[RSY].EncoderValue =		0;
			Joint[RSY].MoveFlag =			false;
			
			Joint[REB].RefAngleCurrent =	0;
			Joint[REB].RefAngleToGo =		0;
			Joint[REB].RefAngleInitial =	0;
			Joint[REB].RefAngleFF =			0;
			Joint[REB].GoalTimeCount =		0;
			Joint[REB].CurrentTimeCount =	0;
			Joint[REB].EncoderValue =		0;
			Joint[REB].MoveFlag =			false;
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC9(RSY, REB) encoder zero..!!");
			break;
		case JMC10:
			Joint[LSP].RefAngleCurrent =	0;
			Joint[LSP].RefAngleToGo =		0;
			Joint[LSP].RefAngleInitial =	0;
			Joint[LSP].RefAngleFF =			0;
			Joint[LSP].GoalTimeCount =		0;
			Joint[LSP].CurrentTimeCount =	0;
			Joint[LSP].EncoderValue =		0;
			Joint[LSP].MoveFlag =			false;
			
			Joint[LSR].RefAngleCurrent =	0;
			Joint[LSR].RefAngleToGo =		0;
			Joint[LSR].RefAngleInitial =	0;
			Joint[LSR].RefAngleFF =			0;
			Joint[LSR].GoalTimeCount =		0;
			Joint[LSR].CurrentTimeCount =	0;
			Joint[LSR].EncoderValue =		0;
			Joint[LSR].MoveFlag =			false;
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC10(LSP, LSR) encoder zero..!!");
			break;
		case JMC11:
			Joint[LSY].RefAngleCurrent =	0;
			Joint[LSY].RefAngleToGo =		0;
			Joint[LSY].RefAngleInitial =	0;
			Joint[LSY].RefAngleFF =			0;
			Joint[LSY].GoalTimeCount =		0;
			Joint[LSY].CurrentTimeCount =	0;
			Joint[LSY].EncoderValue =		0;
			Joint[LSY].MoveFlag =			false;
			
			Joint[LEB].RefAngleCurrent =	0;
			Joint[LEB].RefAngleToGo =		0;
			Joint[LEB].RefAngleInitial =	0;
			Joint[LEB].RefAngleFF =			0;
			Joint[LEB].GoalTimeCount =		0;
			Joint[LEB].CurrentTimeCount =	0;
			Joint[LEB].EncoderValue =		0;
			Joint[LEB].MoveFlag =			false;
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> JMC11(LSY, LEB) encoder zero..!!");
			break;
		case EJMC0:
			Joint[RWY].RefAngleCurrent =	0;
			Joint[RWY].RefAngleToGo =		0;
			Joint[RWY].RefAngleInitial =	0;
			Joint[RWY].RefAngleFF =			0;
			Joint[RWY].GoalTimeCount =		0;
			Joint[RWY].CurrentTimeCount =	0;
			Joint[RWY].EncoderValue =		0;
			Joint[RWY].MoveFlag =			false;
			
			Joint[RWP].RefAngleCurrent =	0;
			Joint[RWP].RefAngleToGo =		0;
			Joint[RWP].RefAngleInitial =	0;
			Joint[RWP].RefAngleFF =			0;
			Joint[RWP].GoalTimeCount =		0;
			Joint[RWP].CurrentTimeCount =	0;
			Joint[RWP].EncoderValue =		0;
			Joint[RWP].MoveFlag =			false;
			
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> EJMC0(RWY, RWP) encoder zero..!!");
			break;
		case EJMC1:
			Joint[LWY].RefAngleCurrent =	0;
			Joint[LWY].RefAngleToGo =		0;
			Joint[LWY].RefAngleInitial =	0;
			Joint[LWY].RefAngleFF =			0;
			Joint[LWY].GoalTimeCount =		0;
			Joint[LWY].CurrentTimeCount =	0;
			Joint[LWY].EncoderValue =		0;
			Joint[LWY].MoveFlag =			false;
			
			Joint[LWP].RefAngleCurrent =	0;
			Joint[LWP].RefAngleToGo =		0;
			Joint[LWP].RefAngleInitial =	0;
			Joint[LWP].RefAngleFF =			0;
			Joint[LWP].GoalTimeCount =		0;
			Joint[LWP].CurrentTimeCount =	0;
			Joint[LWP].EncoderValue =		0;
			Joint[LWP].MoveFlag =			false;
				
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> EJMC1(LWY, LWP) encoder zero..!!");
			break;
		case EJMC2:
			Joint[NKY].RefAngleCurrent =	0;
			Joint[NKY].RefAngleToGo =		0;
			Joint[NKY].RefAngleInitial =	0;
			Joint[NKY].RefAngleFF =			0;
			Joint[NKY].GoalTimeCount =		0;
			Joint[NKY].CurrentTimeCount =	0;
			Joint[NKY].EncoderValue =		0;
			Joint[NKY].MoveFlag =			false;
			
			Joint[NK1].RefAngleCurrent =	0;
			Joint[NK1].RefAngleToGo =		0;
			Joint[NK1].RefAngleInitial =	0;
			Joint[NK1].RefAngleFF =			0;
			Joint[NK1].GoalTimeCount =		0;
			Joint[NK1].CurrentTimeCount =	0;
			Joint[NK1].EncoderValue =		0;
			Joint[NK1].MoveFlag =			false;
				
			Joint[NK2].RefAngleCurrent =	0;
			Joint[NK2].RefAngleToGo =		0;
			Joint[NK2].RefAngleInitial =	0;
			Joint[NK2].RefAngleFF =			0;
			Joint[NK2].GoalTimeCount =		0;
			Joint[NK2].CurrentTimeCount =	0;
			Joint[NK2].EncoderValue =		0;
			Joint[NK2].MoveFlag =			false;
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> EJMC2(NKY, NK1, NK2) encoder zero..!!");
			break;
		case EJMC3:
			Joint[WST].RefAngleCurrent =	0;
			Joint[WST].RefAngleToGo =		0;
			Joint[WST].RefAngleInitial =	0;
			Joint[WST].RefAngleFF =			0;
			Joint[WST].GoalTimeCount =		0;
			Joint[WST].CurrentTimeCount =	0;
			Joint[WST].EncoderValue =		0;
			Joint[WST].MoveFlag =			false;
			PushCANMsg(0, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> EJMC3(WST) encoder zero..!!");
			break;
		case EJMC4:
			for(i=0; i<5; i++)
			{
				Joint[RF1+i].RefAngleCurrent =	0;
				Joint[RF1+i].RefAngleToGo =		0;
				Joint[RF1+i].RefAngleInitial =	0;
				Joint[RF1+i].RefAngleFF =		0;
				Joint[RF1+i].GoalTimeCount =	0;
				Joint[RF1+i].CurrentTimeCount =	0;
				Joint[RF1+i].EncoderValue =		0;
				Joint[RF1+i].MoveFlag =			false;
			}
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> EJMC4(RF) encoder zero..!!");
			break;
		case EJMC5:
			for(i=0; i<5; i++)
			{
				Joint[LF1+i].RefAngleCurrent =	0;
				Joint[LF1+i].RefAngleToGo =		0;
				Joint[LF1+i].RefAngleInitial =	0;
				Joint[LF1+i].RefAngleFF =		0;
				Joint[LF1+i].GoalTimeCount =	0;
				Joint[LF1+i].CurrentTimeCount =	0;
				Joint[LF1+i].EncoderValue =		0;
				Joint[LF1+i].MoveFlag =			false;
			}
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			RtWprintf(L"\n>>> EJMC5(LF) encoder zero..!!");
			break;
		default:
			RtWprintf(L"\n>>> Wrong board ID(ZeroEncoder)..!!");
			return false;
			break;
	}

	return true;
}
/******************************************************************************/









/******************************************************************************/
unsigned char GetMotorChannel(unsigned int _jointID)
{
	switch(_jointID)
	{
		case RHY:
			return 0;
			break;
		case RHR:
			return 1;
			break;
		case RHP:
			return 0;
			break;
		case RKN:
			return 0;
			break;
		case RAP:
			return 0;
			break;
		case RAR:
			return 1;
			break;
		case LHY:
			return 0;
			break;
		case LHR:
			return 1;
			break;
		case LHP:
			return 0;
			break;
		case LKN:
			return 0;
			break;
		case LAP:
			return 0;
			break;
		case LAR:
			return 1;
			break;
		case RSP:
			return 0;
			break;
		case RSR:
			return 1;
			break;
		case RSY:
			return 0;
			break;
		case REB:
			return 1;
			break;
		case RWY:
			return 0;
			break;
		case RWP:
			return 1;
			break;
		case LSP:
			return 0;
			break;
		case LSR:
			return 1;
			break;
		case LSY:
			return 0;
			break;
		case LEB:	
			return 1;
			break;
		case LWY:
			return 0;
			break;
		case LWP:
			return 1;
			break;
		case NKY:
			return 0;
			break;
		case NK1:
			return 1;
			break;
		case NK2:
			return 2;
			break;
		case WST:
			return 0;
			break;
		case RF1:
			return 0;
			break;
		case RF2:
			return 1;
			break;
		case RF3:
			return 2;
			break;
		case RF4:
			return 3;
			break;
		case RF5:
			return 4;
			break;
		case LF1:
			return 0;
			break;
		case LF2:
			return 1;
			break;
		case LF3:
			return 2;
			break;
		case LF4:
			return 3;
			break;
		case LF5:
			return 4;
			break;
	}

	return 100;
}
/******************************************************************************/








/******************************************************************************/
unsigned int GetHDReduction(unsigned int _jointID)
{
	switch(_jointID)
	{
		case RHY:
			return 100;
			break;
		case RHR:
			return 160;
			break;
		case RHP:
			//return 100;	// only for the experiment platform
			return 160;
			break;
		case RKN:
			return 160;
			break;
		case RAP:
			return 100;
			break;
		case RAR:
			return 100;
			break;
		case LHY:
			return 100;
			break;
		case LHR:
			return 160;
			break;
		case LHP:
			return 160;
			break;
		case LKN:
			return 160;
			break;
		case LAP:
			return 100;
			break;
		case LAR:
			return 100;
			break;
		case RSP:
			return 100;
			break;
		case RSR:
			return 100;
			break;
		case RSY:
			return 100;
			break;
		case REB:
			return 100;
			break;
		case RWY:
			return 100;
			break;
		case RWP:
			return 100;
			break;
		case LSP:
			return 100;
			break;
		case LSR:
			return 100;
			break;
		case LSY:
			return 100;
			break;
		case LEB:	
			return 100;
			break;
		case LWY:
			return 100;
			break;
		case LWP:
			return 100;
			break;
		case NKY:
			return 100;
			break;
		case NK1:
			return 100;
			break;
		case NK2:
			return 100;
			break;
		case WST:
			return 100;
			break;
		case RWY2:
			return 100;
			break;
		case RF2:
			return 1;
			break;
		case RF3:
			return 1;
			break;
		case RF4:
			return 1;
			break;
		case RF5:
			return 1;
			break;
		case LWY2:
			return 100;
			break;
		case LF2:
			return 1;
			break;
		case LF3:
			return 1;
			break;
		case LF4:
			return 1;
			break;
		case LF5:
			return 1;
			break;
	}

	return 0;
}
/******************************************************************************/








/******************************************************************************/
unsigned int GetPulleyDrive(unsigned int _jointID)
{
	switch(_jointID)
	{
		case RHY:
			return 10;
			break;
		case RHR:
			return 324;
			break;
		case RHP:
			return 16;
			break;
		case RKN:
			//return 16;
			return 13;
			break;
		case RAP:
			return 10;
			break;
		case RAR:
			return 324;
			break;
		case LHY:
			return 10;
			break;
		case LHR:
			return 324;
			break;
		case LHP:
			return 16;
			break;
		case LKN:
			//return 16;
			return 13; // for the experimental Hubo
			break;
		case LAP:
			return 10;
			break;
		case LAR:
			return 324;
			break;
		case RSP:
			return 10;
			break;
		case RSR:
			return 1;
			break;
		case RSY:
			return 1;
			break;
		case REB:
			return 10;
			break;
		case RWY:
			return 1;
			break;
		case RWP:
			//return 1;
			return 10;
			break;
		case LSP:
			return 10;
			//return 11; // original
			break;
		case LSR:
			return 1;
			break;
		case LSY:
			return 1;
			break;
		case LEB:	
			return 10;
			break;
		case LWY:
			return 1;
			break;
		case LWP:
			//return 1;
			return 10;
			break;
		case NKY:
			return 1;
			break;
		case NK1:
			return 1;
			break;
		case NK2:
			return 1;
			break;
		case WST:
			return 10;
			break;
		case RWY2:
			return 1;
			break;
		case RF2:
			return 1;
			break;
		case RF3:
			return 1;
			break;
		case RF4:
			return 1;
			break;
		case RF5:
			return 1;
			break;
		case LWY2:
			return 1;
			break;
		case LF2:
			return 1;
			break;
		case LF3:
			return 1;
			break;
		case LF4:
			return 1;
			break;
		case LF5:
			return 1;
			break;
	}

	return 0;
}
/******************************************************************************/







/******************************************************************************/
unsigned int GetPulleyDriven(unsigned int _jointID)
{
	switch(_jointID)
	{
		case RHY:
			return 25;
			break;
		case RHR:
			return 1024;
			break;
		case RHP:
			return 20;
			break;
		case RKN:
			//return 16;
			return 25;
			break;
		case RAP:
			return 25;
			break;
		case RAR:
			return 1024;
			break;
		case LHY:
			return 25;
			break;
		case LHR:
			return 1024;
			break;
		case LHP:
			return 20;
			break;
		case LKN:
			//return 16;
			return 25; // for the experimental Hubo
			break;
		case LAP:
			return 25;
			break;
		case LAR:
			return 1024;
			break;
		case RSP:
			return 25;
			break;
		case RSR:
			return 1;
			break;
		case RSY:
			return 1;
			break;
		case REB:
			return 20;
			break;
		case RWY:
			return 1;
			break;
		case RWP:
			//return 2;
			return 20;
			break;
		case LSP:
			return 25;
			break;
		case LSR:
			return 1;
			break;
		case LSY:
			return 1;
			break;
		case LEB:	
			return 20;
			break;
		case LWY:
			return 1;
			break;
		case LWP:
			//return 2;
			return 20;
			break;
		case NKY:
			return 1;
			break;
		case NK1:
			return 1;
			break;
		case NK2:
			return 1;
			break;
		case WST:
			return 25;
			break;
		case RWY2:
			return 2;
			break;
		case RF2:
			return 1;
			break;
		case RF3:
			return 1;
			break;
		case RF4:
			return 1;
			break;
		case RF5:
			return 1;
			break;
		case LWY2:
			return 2;
			break;
		case LF2:
			return 1;
			break;
		case LF3:
			return 1;
			break;
		case LF4:
			return 1;
			break;
		case LF5:
			return 1;
			break;
	}

	return 0;
}
/******************************************************************************/






/******************************************************************************/
unsigned char GetCANChannel(unsigned int _jointID)
{
	switch(_jointID)
	{
		case RHY:
			return CAN0;
			break;
		case RHR:
			return CAN0;
			break;
		case RHP:
			return CAN0;
			break;
		case RKN:
			return CAN0;
			break;
		case RAP:
			return CAN0;
			break;
		case RAR:
			return CAN0;
			break;
		case LHY:
			return CAN0;
			break;
		case LHR:
			return CAN0;
			break;
		case LHP:
			return CAN0;
			break;
		case LKN:
			return CAN0;
			break;
		case LAP:
			return CAN0;
			break;
		case LAR:
			return CAN0;
			break;
		case RSP:
			return CAN1;
			break;
		case RSR:
			return CAN1;
			break;
		case RSY:
			return CAN1;
			break;
		case REB:
			return CAN1;
			break;
		case RWY:
			return CAN1;
			break;
		case RWP:
			return CAN1;
			break;
		case LSP:
			return CAN1;
			break;
		case LSR:
			return CAN1;
			break;
		case LSY:
			return CAN1;
			break;
		case LEB:	
			return CAN1;
			break;
		case LWY:
			return CAN1;
			break;
		case LWP:
			return CAN1;
			break;
		case NKY:
			return CAN1;
			break;
		case NK1:
			return CAN1;
			break;
		case NK2:
			return CAN1;
			break;
		case WST:
			return CAN0;
			break;
		case RF1:
			return CAN1;
			break;
		case RF2:
			return CAN1;
			break;
		case RF3:
			return CAN1;
			break;
		case RF4:
			return CAN1;
			break;
		case RF5:
			return CAN1;
			break;
		case LF1:
			return CAN1;
			break;
		case LF2:
			return CAN1;
			break;
		case LF3:
			return CAN1;
			break;
		case LF4:
			return CAN1;
			break;
		case LF5:
			return CAN1;
			break;
	}

	return 100;
}
/******************************************************************************/







/******************************************************************************/
unsigned char GetBoardID(unsigned char _jointID)
{
	switch(_jointID)
	{
		case RHY:
			return JMC0;
			break;
		case RHR:
			return JMC0;
			break;
		case RHP:
			return JMC1;
			break;
		case RKN:
			return JMC2;
			break;
		case RAP:
			return JMC3;
			break;
		case RAR:
			return JMC3;
			break;
		case LHY:
			return JMC4;
			break;
		case LHR:
			return JMC4;
			break;
		case LHP:
			return JMC5;
			break;
		case LKN:
			return JMC6;
			break;
		case LAP:
			return JMC7;
			break;
		case LAR:
			return JMC7;
			break;
		case RSP:
			return JMC8;
			break;
		case RSR:
			return JMC8;
			break;
		case RSY:
			return JMC9;
			break;
		case REB:
			return JMC9;
			break;
		case RWY:
			return EJMC0;
			break;
		case RWP:
			return EJMC0;
			break;
		case LSP:
			return JMC10;
			break;
		case LSR:
			return JMC10;
			break;
		case LSY:
			return JMC11;
			break;
		case LEB:	
			return JMC11;
			break;
		case LWY:
			return EJMC1;
			break;
		case LWP:
			return EJMC1;
			break;
		case NKY:
			return EJMC2;
			break;
		case NK1:
			return EJMC2;
			break;
		case NK2:
			return EJMC2;
			break;
		case WST:
			return EJMC3;
			break;
		case RF1:
			return EJMC4;
			break;
		case RF2:
			return EJMC4;
			break;
		case RF3:
			return EJMC4;
			break;
		case RF4:
			return EJMC4;
			break;
		case RF5:
			return EJMC4;
			break;
		case LF1:
			return EJMC5;
			break;
		case LF2:
			return EJMC5;
			break;
		case LF3:
			return EJMC5;
			break;
		case LF4:
			return EJMC5;
			break;
		case LF5:
			return EJMC5;
			break;
	}

	return 100;
}
/******************************************************************************/







/******************************************************************************/
bool PositionLimitOnOff(unsigned char _jointID, unsigned char _mode)
{
	unsigned char tempData[8];
	
	tempData[0] = GetBoardID(_jointID);
	tempData[1] = 0x56 + GetMotorChannel(_jointID);
	tempData[2] = _mode;	
	PushCANMsg(GetCANChannel(_mode), CMD_TXDF, tempData, 3, 0);
	RtSleep(10);
		
	tempData[1] = 0x50 + GetMotorChannel(_jointID);
	PushCANMsg(GetCANChannel(_mode), CMD_TXDF, tempData, 3, 0);
	RtSleep(10);
	
	return true;
}
/******************************************************************************/






/******************************************************************************/
bool Beep(unsigned char _mode)
{
	unsigned char tempData[8];
	
	tempData[0] = 0x0E;
	tempData[1] = 0x82;
	tempData[2] = _mode;	
	PushCANMsg(CAN1, CMD_TXDF, tempData, 3, 0);
	RtSleep(10);
	
	return true;
}
/******************************************************************************/






/******************************************************************************/
bool SetDeadZone(JOINT _joint)
{
	unsigned char tempData[8];

	tempData[0] = _joint.JMC;	
	tempData[1] = 0x20 + _joint.Motor_channel;	
	tempData[2] = (_joint.Deadzone)&0x00FF;

	PushCANMsg(_joint.CAN_channel, CMD_TXDF, tempData, 3, 0);

	RtWprintf(L"\n>>>Deadzone Set - Joint: %d\t%d", _joint.Motor_channel, tempData[2]);
	RtSleep(10);

	return true;
}
/******************************************************************************/






/******************************************************************************/
bool SetEncoderResolution(JOINT _joint)
{
	unsigned char tempData[8];

	tempData[0] = _joint.JMC;	
	tempData[1] = 0x38 + _joint.Motor_channel;
	tempData[2] = (unsigned char)(_joint.Encoder_size & 0xFF);
	tempData[3] = (unsigned char)((_joint.Encoder_size>>8) & 0xFF);
	tempData[3] |= ( (_joint.Positive_dir<<7) & 0x80 );

	RtWprintf(L"encoder input : %d", _joint.Encoder_size);
	RtWprintf(L"encoder input : %x", tempData[2]);
	RtWprintf(L"encoder input : %x", tempData[3]);

	PushCANMsg(_joint.CAN_channel, CMD_TXDF, tempData, 6, 0);
	
	RtSleep(10);

	return true;
}
/******************************************************************************/







/******************************************************************************/
bool SetJamPwmSturation(JOINT _joint)
{
	unsigned char tempData[8];

	tempData[0] = _joint.JMC;
	tempData[1] = 0xF2;
	tempData[2] = _joint.JAMmsTime & 0xFF;
	tempData[3] = (_joint.JAMmsTime>>8) & 0xFF;	// byte change
	tempData[4] = _joint.PWMmsTime & 0xFF;
	tempData[5] = (_joint.PWMmsTime>>8) & 0xFF;
	tempData[6] = _joint.JAMDuty;
	tempData[7] = _joint.PWMDuty;

	PushCANMsg(_joint.CAN_channel, CMD_TXDF, tempData, 8, 0);

	RtSleep(10);

	return true;
}
/******************************************************************************/








/******************************************************************************/
bool SetMaxAccVel(JOINT _joint)
{
	unsigned char tempData[8];

	tempData[0] = _joint.JMC;
	tempData[1] = 0x40 + _joint.Motor_channel;
	tempData[2] = _joint.MaxAcc & 0xFF;
	tempData[3] = (_joint.MaxAcc>>8) & 0xFF;	// byte change
	tempData[4] = _joint.MaxVel & 0xFF;
	tempData[5] = (_joint.MaxVel>>8) & 0xFF;
	
	PushCANMsg(_joint.CAN_channel, CMD_TXDF, tempData, 6, 0);

	RtSleep(10);

	return true;
}
/******************************************************************************/








/******************************************************************************/
bool SetControlMode(JOINT _joint)
{
	unsigned char tempData[8];

	tempData[0] = _joint.JMC;
	tempData[1] = 0x10;
	tempData[2] = _joint.MotorControlMode;
	
	PushCANMsg(_joint.CAN_channel, CMD_TXDF, tempData, 3, 0);

	RtSleep(10);

	return true;
}
/******************************************************************************/







/******************************************************************************/
bool SetHomeSearchParameter(JOINT _joint)
{
	unsigned char tempData[8];
	int itemp = (int)(_joint.Offset_angle*_joint.PPR);
	
	tempData[0] = _joint.JMC;
	tempData[1] = 0x30 + _joint.Motor_channel;
	tempData[2] = _joint.Limit_rev;
	tempData[3] = _joint.SearchDirection;
	tempData[4] = (itemp) & 0xFF;
	tempData[5] = (itemp>>8) & 0xFF;
	tempData[6] = (itemp>>16) & 0xFF;
	tempData[7] = (itemp>>24) & 0xFF;
	
	PushCANMsg(_joint.CAN_channel, CMD_TXDF, tempData, 8, 0);

	RtSleep(10);

	return true;
}
/******************************************************************************/






/******************************************************************************/
bool SetHomeMaxVelAcc(JOINT _joint)
{
	unsigned char tempData[8];

	tempData[0] = _joint.JMC;
	tempData[1] = 0x60 + _joint.Motor_channel;
	tempData[2] = _joint.MaxAccHome;
	tempData[3] = (_joint.MaxVelHome) & 0xFF;
	tempData[4] = (_joint.MaxVelHome>>8) & 0xFF;
	tempData[5] = _joint.HomeSearchMode;
	tempData[6] = _joint.PWMDuty;
	
	PushCANMsg(_joint.CAN_channel, CMD_TXDF, tempData, 7, 0);

	RtSleep(10);

	return true;
}
/******************************************************************************/







/******************************************************************************/
bool SetLowerPositionLimit(JOINT _joint)
{
	unsigned char tempData[8];
	int itemp = (int)(_joint.LowerPositionLimit*_joint.PPR);

	tempData[0] = _joint.JMC;
	tempData[1] = 0x50 + _joint.Motor_channel;
	tempData[2] = 0x02;
	tempData[3] = (itemp) & 0xFF;
	tempData[4] = (itemp>>8) & 0xFF;
	tempData[5] = (itemp>>16) & 0xFF;
	tempData[6] = (itemp>>24) & 0xFF;
	
	PushCANMsg(_joint.CAN_channel, CMD_TXDF, tempData, 7, 0);

	RtSleep(10);

	return true;
}
/******************************************************************************/







/******************************************************************************/
bool SetErrorBound(JOINT _joint)
{
	unsigned char tempData[8];
	unsigned int itemp[3];

	itemp[0] = _joint.I_ERR;
	itemp[1] = _joint.B_ERR;
	itemp[2] = _joint.E_ERR;

	tempData[0] = _joint.JMC;
	tempData[1] = 0xF3;
	tempData[2] = (itemp[0]) & 0xFF;
	tempData[3] = (itemp[0]>>8) & 0xFF;
	tempData[4] = (itemp[1]) & 0xFF;
	tempData[5] = (itemp[1]>>8) & 0xFF;
	tempData[6] = (itemp[2]) & 0xFF;
	tempData[7] = (itemp[2]>>8) & 0xFF;
	
	
	PushCANMsg(_joint.CAN_channel, CMD_TXDF, tempData, 8, 0);

	RtSleep(10);

	return true;
}
/******************************************************************************/







/******************************************************************************/
bool SetUpperPositionLimit(JOINT _joint)
{
	unsigned char tempData[8];
	int itemp = (int)(_joint.UpperPositionLimit*_joint.PPR);

	tempData[0] = _joint.JMC;
	tempData[1] = 0x56 + _joint.Motor_channel;
	tempData[2] = 0x02;
	tempData[3] = (itemp) & 0xFF;
	tempData[4] = (itemp>>8) & 0xFF;
	tempData[5] = (itemp>>16) & 0xFF;
	tempData[6] = (itemp>>24) & 0xFF;
	
	PushCANMsg(_joint.CAN_channel, CMD_TXDF, tempData, 7, 0);

	RtSleep(10);

	return true;
}
/******************************************************************************/







/******************************************************************************/
bool RequestParameters(JOINT* _joint)
{
	unsigned char i, j;
	unsigned char sendData[8], receiveData[8];
	unsigned int resultData[9][8];
	unsigned char motorCH = _joint->Motor_channel;
	unsigned int  count = 0;
	unsigned char checkSum, check;

	unsigned char buff_no;
	bool		result = true;


	sendData[0] = _joint->JMC;	sendData[1] = RequestPara;
	
	if(motorCH < 3) sendData[2] = motorCH*6 + 1;
	else sendData[2] = motorCH*6 + 1 + 5;
	count = 0;
	PushCANMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		//CanReceiveMsg(_joint->CAN_channel);
		if(count > 30000) 
		{ 
			RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 0);	
			result = false; 
		}
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		//if(ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData) == ERR_OK) count = 3500;
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[0][i] = (unsigned int)receiveData[i];
	

	if(motorCH < 3) sendData[2] = motorCH*6 + 2;
	else sendData[2] = motorCH*6 + 2 + 5;
	count = 0;
	PushCANMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		//CanReceiveMsg(_joint->CAN_channel);
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 1);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		//if(ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData) == ERR_OK) count = 3500;
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[1][i] = (unsigned int)receiveData[i];


	if(motorCH < 3) sendData[2] = motorCH*6 + 3;
	else sendData[2] = motorCH*6 + 3 + 5;
	count = 0;
	PushCANMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		//CanReceiveMsg(_joint->CAN_channel);
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 2);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[2][i] = (unsigned int)receiveData[i];


	if(motorCH < 3) sendData[2] = motorCH*6 + 4;
	else sendData[2] = motorCH*6 + 4 + 5;
	count = 0;
	PushCANMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		//CanReceiveMsg(_joint->CAN_channel);
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 3);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[3][i] = (unsigned int)receiveData[i];


	if(motorCH < 3) sendData[2] = motorCH*6 + 5;
	else sendData[2] = motorCH*6 + 5 + 5;
	count = 0;
	PushCANMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		//CanReceiveMsg(_joint->CAN_channel);
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 4);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[4][i] = (unsigned int)receiveData[i];
		

	if(motorCH < 3) sendData[2] = motorCH*6 + 6;
	else sendData[2] = motorCH*6 + 6 + 5;
	count = 0;
	PushCANMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		//CanReceiveMsg(_joint->CAN_channel);
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 5);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[5][i] = (unsigned int)receiveData[i];


	sendData[2] = 20;
	count = 0;
	PushCANMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		//CanReceiveMsg(_joint->CAN_channel);
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 6);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[6][i] = (unsigned int)receiveData[i];


	sendData[2] = 21;
	count = 0;
	PushCANMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		//CanReceiveMsg(_joint->CAN_channel);
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 7);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[7][i] = (unsigned int)receiveData[i];

		
	sendData[2] = 22;
	count = 0;
	PushCANMsg(_joint->CAN_channel, CMD_TXDF, sendData, 3, 0);
	while(count <= 30001)
	{
		//CanReceiveMsg(_joint->CAN_channel);
		if(count > 30000) { RtWprintf(L"\n>>> Request Parameter Error - JMC:%d Request : %d", _joint->JMC, 8);	result = false; }
		buff_no = GetBuffno(_joint->JMC+SETTING_BASE_RXDF);
		if(MB[buff_no].status == NEWDATA) { ReadMBData(_joint->JMC+SETTING_BASE_RXDF, receiveData); count = 35000; }
		else count++;
	}
	for(i=0 ; i<8 ; i++) resultData[8][i] = (unsigned int)receiveData[i];
	

	if( (_joint->JointID<=LWP) ||  (_joint->JointID==WST))
	{
		for(i=2 ; i<=7 ; i++)
		{
			checkSum = 0x00;
			for(j=0 ; j<5 ; j++) checkSum += resultData[j][i];
			check = checkSum&0xFF;
			if(check != resultData[5][i]) { RtWprintf(L"\n>>> CheckSum Error - Joint:%d\t No", _joint->JointID, i); result = false; }
		}
	}

	_joint->Position_Kp = resultData[0][0] | (resultData[0][1]<<8);
	_joint->Position_Ki = resultData[0][2] | (resultData[0][3]<<8);
	_joint->Position_Kd = resultData[0][4] | (resultData[0][5]<<8);
	_joint->Encoder_size = resultData[0][6] | (resultData[0][7]&0x3F)<<8;
	_joint->Positive_dir = (resultData[0][7]&0x80)>>7;
	_joint->Deadzone = resultData[1][0] | (resultData[1][1]<<8);
	_joint->SearchDirection = resultData[1][2];
	_joint->HomeSearchMode = resultData[1][3];
	_joint->Limit_rev = resultData[1][4] | (resultData[1][5]<<8);
	_joint->PPR	 = (float)_joint->HDReduction*((float)_joint->Pulley_driven/(float)_joint->Pulley_drive)*((float)_joint->Encoder_size)/360.0f;
	_joint->Offset_angle = (float)((int)resultData[1][6]|(int)(resultData[1][7]<<8)|(int)(resultData[2][0]<<16)|(int)(resultData[2][1]<<24))/_joint->PPR;
	_joint->LowerPositionLimit = (float)((int)resultData[2][2] | (int)(resultData[2][3]<<8) | (int)(resultData[2][4]<<16) | (int)(resultData[2][5]<<24))/_joint->PPR;
	_joint->UpperPositionLimit = (float)((int)resultData[2][6] | (int)(resultData[2][7]<<8) | (int)(resultData[3][0]<<16) | (int)(resultData[3][1]<<24))/_joint->PPR;
	_joint->MaxAcc = resultData[3][2] | (resultData[3][3]<<8);
	_joint->MaxVel = resultData[3][4] | (resultData[3][5]<<8);
	_joint->MaxPWM = resultData[3][6] | (resultData[3][7]<<8);
	_joint->MaxAccHome = resultData[6][6] | (resultData[6][7]<<8);
	_joint->MaxVelHome = resultData[7][0] | (resultData[7][1]<<8);
	_joint->JAMmsTime = resultData[7][4] | (resultData[7][5]<<8);
	_joint->PWMmsTime = resultData[7][6] | (resultData[7][7]<<8);
	_joint->PWMDuty = resultData[8][0];
	_joint->JAMDuty = resultData[8][1];
	_joint->I_ERR = resultData[8][2] | (resultData[8][3]<<8);
	_joint->B_ERR = resultData[8][4] | (resultData[8][5]<<8);
	_joint->E_ERR = resultData[8][6] | (resultData[8][7]<<8);
	
	return result;
}
/******************************************************************************/








/******************************************************************************/
bool CheckBoardStatus(JOINT* _joint)
{
	bool update = false;
	unsigned char i, receiveData[8];

	for(i=JMC0 ; i<=JMC11 ; i++)
	{
		switch(i)
		{
		case JMC0:
			if(ReadMBData(STAT_BASE_RXDF+JMC0, receiveData) == ERR_OK)
			{	
				_joint[RHY].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[RHY].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[RHY].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[RHY].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[RHY].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	

				_joint[RHY].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[RHY].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[RHY].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[RHY].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[RHY].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[RHY].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[RHY].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);

				_joint[RHY].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[RHY].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[RHY].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[RHY].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[RHY].TempError =			(bool)((receiveData[2]>>4)&0x01);


				_joint[RHR].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[RHR].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[RHR].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[RHR].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[RHR].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[RHR].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[RHR].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[RHR].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[RHR].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[RHR].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[RHR].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[RHR].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
				
				_joint[RHR].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[RHR].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[RHR].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[RHR].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[RHR].TempError =			(bool)((receiveData[6]>>4)&0x01);
				
				update = true;
			}
			break;
		case JMC1:
			if(ReadMBData(STAT_BASE_RXDF+JMC1, receiveData) == ERR_OK) 
			{	
				_joint[RHP].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[RHP].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[RHP].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[RHP].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[RHP].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[RHP].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[RHP].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[RHP].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[RHP].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[RHP].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[RHP].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[RHP].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[RHP].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[RHP].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[RHP].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[RHP].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[RHP].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				update = true;
			}
			break;
		case JMC2:
			if(ReadMBData(STAT_BASE_RXDF+JMC2, receiveData) == ERR_OK) 
			{	
				_joint[RKN].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[RKN].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[RKN].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[RKN].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[RKN].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[RKN].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[RKN].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[RKN].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[RKN].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[RKN].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[RKN].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[RKN].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[RKN].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[RKN].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[RKN].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[RKN].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[RKN].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				update = true;
			}
			break;
		case JMC3:
			if(ReadMBData(STAT_BASE_RXDF+JMC3, receiveData) == ERR_OK) 
			{	
				_joint[RAP].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[RAP].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[RAP].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[RAP].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[RAP].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[RAP].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[RAP].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[RAP].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[RAP].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[RAP].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[RAP].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[RAP].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[RAP].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[RAP].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[RAP].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[RAP].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[RAP].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[RAR].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[RAR].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[RAR].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[RAR].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[RAR].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[RAR].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[RAR].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[RAR].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[RAR].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[RAR].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[RAR].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[RAR].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
				
				_joint[RAR].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[RAR].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[RAR].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[RAR].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[RAR].TempError =			(bool)((receiveData[6]>>4)&0x01);
				
				update = true;
			}
			break;
		case JMC4:
			if(ReadMBData(STAT_BASE_RXDF+JMC4, receiveData) == ERR_OK) 
			{	
				_joint[LHY].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[LHY].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[LHY].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[LHY].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[LHY].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[LHY].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[LHY].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[LHY].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[LHY].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[LHY].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[LHY].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[LHY].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[LHY].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[LHY].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[LHY].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[LHY].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[LHY].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[LHR].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[LHR].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[LHR].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[LHR].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[LHR].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[LHR].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[LHR].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[LHR].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[LHR].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[LHR].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[LHR].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[LHR].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
				
				_joint[LHR].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[LHR].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[LHR].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[LHR].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[LHR].TempError =			(bool)((receiveData[6]>>4)&0x01);
				
				update = true;
			}
			break;
		case JMC5:
			if(ReadMBData(STAT_BASE_RXDF+JMC5, receiveData) == ERR_OK) 
			{				
				_joint[LHP].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[LHP].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[LHP].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[LHP].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[LHP].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[LHP].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[LHP].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[LHP].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[LHP].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[LHP].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[LHP].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[LHP].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[LHP].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[LHP].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[LHP].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[LHP].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[LHP].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				update = true;
			}
			break;
		case JMC6:
			if(ReadMBData(STAT_BASE_RXDF+JMC6, receiveData) == ERR_OK) 
			{				
				_joint[LKN].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[LKN].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[LKN].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[LKN].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[LKN].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[LKN].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[LKN].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[LKN].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[LKN].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[LKN].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[LKN].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[LKN].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[LKN].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[LKN].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[LKN].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[LKN].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[LKN].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				update = true;
			}
			break;
		case JMC7:
			if(ReadMBData(STAT_BASE_RXDF+JMC7, receiveData) == ERR_OK) 
			{	
				_joint[LAP].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[LAP].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[LAP].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[LAP].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[LAP].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[LAP].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[LAP].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[LAP].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[LAP].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[LAP].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[LAP].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[LAP].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[LAP].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[LAP].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[LAP].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[LAP].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[LAP].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[LAR].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[LAR].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[LAR].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[LAR].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[LAR].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[LAR].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[LAR].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[LAR].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[LAR].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[LAR].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[LAR].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[LAR].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
				
				_joint[LAR].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[LAR].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[LAR].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[LAR].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[LAR].TempError =			(bool)((receiveData[6]>>4)&0x01);
				
				update = true;
			}
			break;
		case JMC8:
			if(ReadMBData(STAT_BASE_RXDF+JMC8, receiveData) == ERR_OK) 
			{	
				_joint[RSP].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[RSP].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[RSP].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[RSP].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[RSP].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[RSP].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[RSP].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[RSP].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[RSP].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[RSP].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[RSP].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[RSP].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[RSP].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[RSP].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[RSP].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[RSP].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[RSP].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[RSR].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[RSR].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[RSR].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[RSR].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[RSR].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[RSR].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[RSR].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[RSR].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[RSR].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[RSR].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[RSR].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[RSR].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
				
				_joint[RSR].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[RSR].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[RSR].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[RSR].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[RSR].TempError =			(bool)((receiveData[6]>>4)&0x01);
			
				update = true;
			}
			break;
		case JMC9:
			if(ReadMBData(STAT_BASE_RXDF+JMC9, receiveData) == ERR_OK) 
			{				
				_joint[RSY].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[RSY].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[RSY].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[RSY].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[RSY].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[RSY].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[RSY].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[RSY].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[RSY].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[RSY].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[RSY].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[RSY].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[RSY].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[RSY].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[RSY].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[RSY].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[RSY].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[REB].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[REB].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[REB].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[REB].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[REB].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[REB].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[REB].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[REB].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[REB].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[REB].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[REB].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[REB].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
		
				_joint[REB].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[REB].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[REB].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[REB].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[REB].TempError =			(bool)((receiveData[6]>>4)&0x01);
		
				update = true;
			}
			break;
		case JMC10:
			if(ReadMBData(STAT_BASE_RXDF+JMC10, receiveData) == ERR_OK) 
			{				
				_joint[LSP].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[LSP].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[LSP].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[LSP].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[LSP].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[LSP].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[LSP].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[LSP].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[LSP].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[LSP].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[LSP].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[LSP].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[LSP].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[LSP].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[LSP].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[LSP].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[LSP].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[LSR].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[LSR].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[LSR].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[LSR].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[LSR].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
			
				_joint[LSR].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[LSR].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[LSR].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[LSR].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[LSR].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[LSR].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[LSR].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
		
				_joint[LSR].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[LSR].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[LSR].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[LSR].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[LSR].TempError =			(bool)((receiveData[6]>>4)&0x01);
			
				update = true;
			}
			break;
		case JMC11:
			if(ReadMBData(STAT_BASE_RXDF+JMC11, receiveData) == ERR_OK) 
			{				
				_joint[LSY].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[LSY].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[LSY].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[LSY].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[LSY].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[LSY].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[LSY].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[LSY].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[LSY].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[LSY].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[LSY].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[LSY].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[LSY].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[LSY].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[LSY].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[LSY].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[LSY].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[LEB].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[LEB].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[LEB].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[LEB].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[LEB].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[LEB].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[LEB].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[LEB].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[LEB].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[LEB].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[LEB].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[LEB].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
			
				_joint[LEB].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[LEB].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[LEB].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[LEB].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[LEB].TempError =			(bool)((receiveData[6]>>4)&0x01);
			
				update = true;
			}
			break;
		}
	}

	for(i=EJMC0 ; i<=EJMC5 ; i++)
	{
		switch(i)		
		{
		case EJMC0:
			if(ReadMBData(STAT_BASE_RXDF+EJMC0, receiveData) == NODATA) 
			{	
				_joint[RWY].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[RWY].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[RWY].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[RWY].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[RWY].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[RWY].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[RWY].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[RWY].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[RWY].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[RWY].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[RWY].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[RWY].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[RWY].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[RWY].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[RWY].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[RWY].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[RWY].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[RWP].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[RWP].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[RWP].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[RWP].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[RWP].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[RWP].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[RWP].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[RWP].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[RWP].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[RWP].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[RWP].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[RWP].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
				
				_joint[RWP].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[RWP].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[RWP].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[RWP].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[RWP].TempError =			(bool)((receiveData[6]>>4)&0x01);
			
				update = true;
			}
			break;
		case EJMC1:
			if(ReadMBData(STAT_BASE_RXDF+EJMC1, receiveData) == ERR_OK) 
			{				
				_joint[LWY].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[LWY].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[LWY].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[LWY].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[LWY].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[LWY].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[LWY].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[LWY].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[LWY].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[LWY].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[LWY].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[LWY].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[LWY].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[LWY].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[LWY].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[LWY].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[LWY].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				
				_joint[LWP].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[LWP].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[LWP].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[LWP].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[LWP].HomeStatus =		(unsigned char)((receiveData[4]>>4)&0x0F);	
				
				_joint[LWP].JAMError =			(bool)(receiveData[5]&0x01);
				_joint[LWP].PWMError =			(bool)((receiveData[5]>>1)&0x01);
				_joint[LWP].BigError =			(bool)((receiveData[5]>>2)&0x01);
				_joint[LWP].EncoderError =		(bool)((receiveData[5]>>3)&0x01);
				_joint[LWP].FaultError =		(bool)((receiveData[5]>>4)&0x01);
				_joint[LWP].Motor0Error =		(bool)((receiveData[5]>>5)&0x01);
				_joint[LWP].Motor1Error =		(bool)((receiveData[5]>>6)&0x01);
				
				_joint[LWP].LLimitError =		(bool)(receiveData[6]&0x01);
				_joint[LWP].ULimitError =		(bool)((receiveData[6]>>1)&0x01);
				_joint[LWP].VelError =			(bool)((receiveData[6]>>2)&0x01);
				_joint[LWP].AccError =			(bool)((receiveData[6]>>3)&0x01);
				_joint[LWP].TempError =			(bool)((receiveData[6]>>4)&0x01);
				
				update = true;
			}
			break;
		case EJMC2:
			if(ReadMBData(STAT_BASE_RXDF+EJMC2, receiveData) == ERR_OK) 
			{
				_joint[NKY].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[NKY].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[NKY].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[NKY].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[NKY].JAMError =			(bool)((receiveData[0]>>4)&0x01);
				_joint[NKY].PWMError =			(bool)((receiveData[0]>>5)&0x01);
				_joint[NKY].BigError =			(bool)((receiveData[0]>>6)&0x01);
				_joint[NKY].EncoderError =		(bool)((receiveData[0]>>7)&0x01);

				_joint[NK1].FetOnOff =			(bool)(receiveData[1]&0x01);
				_joint[NK1].ControlOnOff =		(bool)((receiveData[1]>>1)&0x01);
				_joint[NK1].MotorControlMode =	(unsigned char)((receiveData[1]>>2)&0x01);
				_joint[NK1].LimitOnOff =		(bool)((receiveData[1]>>3)&0x01);
				_joint[NK1].JAMError =			(bool)((receiveData[1]>>4)&0x01);
				_joint[NK1].PWMError =			(bool)((receiveData[1]>>5)&0x01);
				_joint[NK1].BigError =			(bool)((receiveData[1]>>6)&0x01);
				_joint[NK1].EncoderError =		(bool)((receiveData[1]>>7)&0x01);

				_joint[NK2].FetOnOff =			(bool)(receiveData[2]&0x01);
				_joint[NK2].ControlOnOff =		(bool)((receiveData[2]>>1)&0x01);
				_joint[NK2].MotorControlMode =	(unsigned char)((receiveData[2]>>2)&0x01);
				_joint[NK2].LimitOnOff =		(bool)((receiveData[2]>>3)&0x01);
				_joint[NK2].JAMError =			(bool)((receiveData[2]>>4)&0x01);
				_joint[NK2].PWMError =			(bool)((receiveData[2]>>5)&0x01);
				_joint[NK2].BigError =			(bool)((receiveData[2]>>6)&0x01);
				_joint[NK2].EncoderError =		(bool)((receiveData[2]>>7)&0x01);

				update = true;
			}
			break;
		case EJMC3:
			if(ReadMBData(STAT_BASE_RXDF+EJMC3, receiveData) == ERR_OK) 
			{								
				_joint[WST].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[WST].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[WST].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[WST].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[WST].HomeStatus =		(unsigned char)((receiveData[0]>>4)&0x0F);	
				
				_joint[WST].JAMError =			(bool)(receiveData[1]&0x01);
				_joint[WST].PWMError =			(bool)((receiveData[1]>>1)&0x01);
				_joint[WST].BigError =			(bool)((receiveData[1]>>2)&0x01);
				_joint[WST].EncoderError =		(bool)((receiveData[1]>>3)&0x01);
				_joint[WST].FaultError =		(bool)((receiveData[1]>>4)&0x01);
				_joint[WST].Motor0Error =		(bool)((receiveData[1]>>5)&0x01);
				_joint[WST].Motor1Error =		(bool)((receiveData[1]>>6)&0x01);
				
				_joint[WST].LLimitError =		(bool)(receiveData[2]&0x01);
				_joint[WST].ULimitError =		(bool)((receiveData[2]>>1)&0x01);
				_joint[WST].VelError =			(bool)((receiveData[2]>>2)&0x01);
				_joint[WST].AccError =			(bool)((receiveData[2]>>3)&0x01);
				_joint[WST].TempError =			(bool)((receiveData[2]>>4)&0x01);
				
				update = true;
			}
			break;
		case EJMC4:
			if(ReadMBData(STAT_BASE_RXDF+EJMC4, receiveData) == ERR_OK) 
			{
				_joint[RF1].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[RF1].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[RF1].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[RF1].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[RF1].JAMError =			(bool)((receiveData[0]>>4)&0x01);
				_joint[RF1].PWMError =			(bool)((receiveData[0]>>5)&0x01);
				_joint[RF1].BigError =			(bool)((receiveData[0]>>6)&0x01);
				_joint[RF1].EncoderError =		(bool)((receiveData[0]>>7)&0x01);
				
				_joint[RF2].FetOnOff =			(bool)(receiveData[1]&0x01);
				_joint[RF2].ControlOnOff =		(bool)((receiveData[1]>>1)&0x01);
				_joint[RF2].MotorControlMode =	(unsigned char)((receiveData[1]>>2)&0x01);
				_joint[RF2].LimitOnOff =		(bool)((receiveData[1]>>3)&0x01);
				_joint[RF2].JAMError =			(bool)((receiveData[1]>>4)&0x01);
				_joint[RF2].PWMError =			(bool)((receiveData[1]>>5)&0x01);
				_joint[RF2].BigError =			(bool)((receiveData[1]>>6)&0x01);
				_joint[RF2].EncoderError =		(bool)((receiveData[1]>>7)&0x01);
				
				_joint[RF3].FetOnOff =			(bool)(receiveData[2]&0x01);
				_joint[RF3].ControlOnOff =		(bool)((receiveData[2]>>1)&0x01);
				_joint[RF3].MotorControlMode =	(unsigned char)((receiveData[2]>>2)&0x01);
				_joint[RF3].LimitOnOff =		(bool)((receiveData[2]>>3)&0x01);
				_joint[RF3].JAMError =			(bool)((receiveData[2]>>4)&0x01);
				_joint[RF3].PWMError =			(bool)((receiveData[2]>>5)&0x01);
				_joint[RF3].BigError =			(bool)((receiveData[2]>>6)&0x01);
				_joint[RF3].EncoderError =		(bool)((receiveData[2]>>7)&0x01);

				_joint[RF4].FetOnOff =			(bool)(receiveData[3]&0x01);
				_joint[RF4].ControlOnOff =		(bool)((receiveData[3]>>1)&0x01);
				_joint[RF4].MotorControlMode =	(unsigned char)((receiveData[3]>>2)&0x01);
				_joint[RF4].LimitOnOff =		(bool)((receiveData[3]>>3)&0x01);
				_joint[RF4].JAMError =			(bool)((receiveData[3]>>4)&0x01);
				_joint[RF4].PWMError =			(bool)((receiveData[3]>>5)&0x01);
				_joint[RF4].BigError =			(bool)((receiveData[3]>>6)&0x01);
				_joint[RF4].EncoderError =		(bool)((receiveData[3]>>7)&0x01);
				
				_joint[RF5].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[RF5].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[RF5].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[RF5].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[RF5].JAMError =			(bool)((receiveData[4]>>4)&0x01);
				_joint[RF5].PWMError =			(bool)((receiveData[4]>>5)&0x01);
				_joint[RF5].BigError =			(bool)((receiveData[4]>>6)&0x01);
				_joint[RF5].EncoderError =		(bool)((receiveData[4]>>7)&0x01);
	
				update = true;
			}
			break;
		case EJMC5:
			if(ReadMBData(STAT_BASE_RXDF+EJMC5, receiveData) == ERR_OK) 
			{
				_joint[LF1].FetOnOff =			(bool)(receiveData[0]&0x01);
				_joint[LF1].ControlOnOff =		(bool)((receiveData[0]>>1)&0x01);
				_joint[LF1].MotorControlMode =	(unsigned char)((receiveData[0]>>2)&0x01);
				_joint[LF1].LimitOnOff =		(bool)((receiveData[0]>>3)&0x01);
				_joint[LF1].JAMError =			(bool)((receiveData[0]>>4)&0x01);
				_joint[LF1].PWMError =			(bool)((receiveData[0]>>5)&0x01);
				_joint[LF1].BigError =			(bool)((receiveData[0]>>6)&0x01);
				_joint[LF1].EncoderError =		(bool)((receiveData[0]>>7)&0x01);
				
				_joint[LF2].FetOnOff =			(bool)(receiveData[1]&0x01);
				_joint[LF2].ControlOnOff =		(bool)((receiveData[1]>>1)&0x01);
				_joint[LF2].MotorControlMode =	(unsigned char)((receiveData[1]>>2)&0x01);
				_joint[LF2].LimitOnOff =		(bool)((receiveData[1]>>3)&0x01);
				_joint[LF2].JAMError =			(bool)((receiveData[1]>>4)&0x01);
				_joint[LF2].PWMError =			(bool)((receiveData[1]>>5)&0x01);
				_joint[LF2].BigError =			(bool)((receiveData[1]>>6)&0x01);
				_joint[LF2].EncoderError =		(bool)((receiveData[1]>>7)&0x01);
				
				_joint[LF3].FetOnOff =			(bool)(receiveData[2]&0x01);
				_joint[LF3].ControlOnOff =		(bool)((receiveData[2]>>1)&0x01);
				_joint[LF3].MotorControlMode =	(unsigned char)((receiveData[2]>>2)&0x01);
				_joint[LF3].LimitOnOff =		(bool)((receiveData[2]>>3)&0x01);
				_joint[LF3].JAMError =			(bool)((receiveData[2]>>4)&0x01);
				_joint[LF3].PWMError =			(bool)((receiveData[2]>>5)&0x01);
				_joint[LF3].BigError =			(bool)((receiveData[2]>>6)&0x01);
				_joint[LF3].EncoderError =		(bool)((receiveData[2]>>7)&0x01);
				
				_joint[LF4].FetOnOff =			(bool)(receiveData[3]&0x01);
				_joint[LF4].ControlOnOff =		(bool)((receiveData[3]>>1)&0x01);
				_joint[LF4].MotorControlMode =	(unsigned char)((receiveData[3]>>2)&0x01);
				_joint[LF4].LimitOnOff =		(bool)((receiveData[3]>>3)&0x01);
				_joint[LF4].JAMError =			(bool)((receiveData[3]>>4)&0x01);
				_joint[LF4].PWMError =			(bool)((receiveData[3]>>5)&0x01);
				_joint[LF4].BigError =			(bool)((receiveData[3]>>6)&0x01);
				_joint[LF4].EncoderError =		(bool)((receiveData[3]>>7)&0x01);
				
				_joint[LF5].FetOnOff =			(bool)(receiveData[4]&0x01);
				_joint[LF5].ControlOnOff =		(bool)((receiveData[4]>>1)&0x01);
				_joint[LF5].MotorControlMode =	(unsigned char)((receiveData[4]>>2)&0x01);
				_joint[LF5].LimitOnOff =		(bool)((receiveData[4]>>3)&0x01);
				_joint[LF5].JAMError =			(bool)((receiveData[4]>>4)&0x01);
				_joint[LF5].PWMError =			(bool)((receiveData[4]>>5)&0x01);
				_joint[LF5].BigError =			(bool)((receiveData[4]>>6)&0x01);
				_joint[LF5].EncoderError =		(bool)((receiveData[4]>>7)&0x01);
				
				update = true;
			}
			break;
		}
	}

	/*
	for(i=0; i<NO_OF_JOINT ; i++)
	{
		if(_joint[i].JAMError == true) Beep(1);
		if(_joint[i].PWMError == true) Beep(2);
		if(_joint[i].BigError == true) Beep(3);
		if(_joint[i].EncoderError == true) Beep(4);
	}
	*/

	return update;
}
/******************************************************************************/








/******************************************************************************/
bool SetMoveJointAngle(unsigned char _jointID, float _angle, float _msTime, unsigned char _mode)
{
	if(_msTime <= 0) { RtWprintf(L"\n>>> goal time must be larger than zero..!!"); return false; }
	if(_jointID >= NO_OF_JOINT) { RtWprintf(L"\n>>> Wrong joint ID(SetMoveJointAngle)..!!"); return false; }
	
	switch(_mode)
	{
	case 0x00:	// relative mode
		Joint[_jointID].RefAngleToGo = Joint[_jointID].RefAngleCurrent + _angle;
		break;
	case 0x01:	// absolute mode
		Joint[_jointID].RefAngleToGo = _angle;
		break;
	default:
		RtWprintf(L"\n>>> Wrong reference mode(SetMoveJointAngle)..!!");
		return false;
		break;
	}

	Joint[_jointID].MoveFlag = false;
	Joint[_jointID].RefAngleInitial = Joint[_jointID].RefAngleCurrent;
	Joint[_jointID].CurrentTimeCount = 0;
	if(Joint[_jointID].CAN_channel == CAN0) Joint[_jointID].GoalTimeCount = (unsigned long)(_msTime/INT_TIME);
	else Joint[_jointID].GoalTimeCount = (unsigned long)(_msTime/INT_TIME1);
	Joint[_jointID].DelayTimeCount[0] = 0;	Joint[_jointID].DelayTimeCount[1] = 0;
	Joint[_jointID].RefAngleDelta = Joint[_jointID].RefAngleToGo - Joint[_jointID].RefAngleCurrent;
	Joint[_jointID].FunctionMode = 0x00;
	Joint[_jointID].MoveFlag = true;

	return true;
}
/******************************************************************************/




/******************************************************************************/
bool SetMoveJointAngleFunc(unsigned char _jointID, float _angle, float _userData[5], float _msTime[3], unsigned char _func, unsigned char _mode)
{
	unsigned char i;
	
	if(_msTime <= 0) { RtWprintf(L"\n>>> goal time must be grater than zero..!!"); return false; }
	if(_jointID >= NO_OF_JOINT) { RtWprintf(L"\n>>> Wrong joint ID(SetMoveJointAngle)..!!"); return false; }
	
	if(Joint[_jointID].MoveFlag == true) { RtWprintf(L"\n>>> Joint is moving..!!"); return false; }
	
	switch(_mode)
	{
	case 0x00:	// relative mode
		Joint[_jointID].RefAngleToGo = Joint[_jointID].RefAngleCurrent + _angle;
		break;
	case 0x01:	// absolute mode
		Joint[_jointID].RefAngleToGo = _angle;
		break;
	default:
		RtWprintf(L"\n>>> Wrong reference mode(SetMoveJointAngle)..!!");
		return false;
		break;
	}
	
	Joint[_jointID].MoveFlag = false;
	for(i=0 ; i<5 ; i++) Joint[_jointID].UserData[i] = _userData[i];
	Joint[_jointID].RefAngleInitial = Joint[_jointID].RefAngleCurrent;
	Joint[_jointID].CurrentTimeCount = 0;
	if(Joint[_jointID].CAN_channel == CAN0)
	{
		Joint[_jointID].GoalTimeCount = (unsigned long)(_msTime[0]/INT_TIME);
		Joint[_jointID].DelayTimeCount[0] = (unsigned long)(_msTime[1]/INT_TIME);
		Joint[_jointID].DelayTimeCount[1] = (unsigned long)(_msTime[2]/INT_TIME);
	}
	else
	{
		Joint[_jointID].GoalTimeCount = (unsigned long)(_msTime[0]/INT_TIME1);
		Joint[_jointID].DelayTimeCount[0] = (unsigned long)(_msTime[1]/INT_TIME1);
		Joint[_jointID].DelayTimeCount[1] = (unsigned long)(_msTime[2]/INT_TIME1);
	}
	
	Joint[_jointID].RefAngleDelta = Joint[_jointID].RefAngleToGo - Joint[_jointID].RefAngleCurrent;
	Joint[_jointID].FunctionMode = _func;
	Joint[_jointID].MoveFlag = true;
	
	return true;
}
/******************************************************************************/




/******************************************************************************/
void MoveJointAngle(unsigned char _canChannel)
{
	unsigned char i;
	static float RefAngle_Last[NO_OF_JOINT] = {0.0f,};
	
	if(_canChannel == CAN0) 
	{
		for(i=RHY ; i<=LAR ; i++)
		{
			if(Joint[i].MoveFlag == true)
			{
				if(Joint[i].DelayTimeCount[0] <= 0) 
				{
					Joint[i].CurrentTimeCount++;
					if(Joint[i].GoalTimeCount <= Joint[i].CurrentTimeCount)
					{
						Joint[i].GoalTimeCount = Joint[i].CurrentTimeCount = 0;
						Joint[i].RefAngleCurrent = Joint[i].RefAngleToGo;
						if(Joint[i].DelayTimeCount[1] <= 0) Joint[i].MoveFlag = false;	
						else Joint[i].DelayTimeCount[1]--; // tail time delay
					}
					else 
					{
						if(Joint[i].FunctionMode == 0x00)	// 0.5*(1-cos(pi*t/T))
						{
							Joint[i].RefAngleCurrent = Joint[i].RefAngleInitial+Joint[i].RefAngleDelta*0.5f*(1.0f-cosf(PI/Joint[i].GoalTimeCount*Joint[i].CurrentTimeCount));
						}
						else if(Joint[i].FunctionMode == 0x01)	// 0.5*(1-cos(2*pi*t/T))
						{
							Joint[i].RefAngleCurrent = Joint[i].RefAngleInitial+Joint[i].UserData[0]*0.5f*(1.0f-cosf(2.0f*PI/Joint[i].GoalTimeCount*Joint[i].CurrentTimeCount));
						}
						else if(Joint[i].FunctionMode == 0x02)	// 0.5*(1-cos(4*pi*t/T))
						{
							Joint[i].RefAngleCurrent = Joint[i].RefAngleInitial+Joint[i].UserData[0]*0.5f*(1.0f-cosf(4.0f*PI/Joint[i].GoalTimeCount*Joint[i].CurrentTimeCount));
						}
					}
				}
				else Joint[i].DelayTimeCount[0]--;
			}
		}

		if(Joint[WST].MoveFlag == true)
		{
			if(Joint[WST].DelayTimeCount[0] <= 0) 
			{
				Joint[WST].CurrentTimeCount++;
				if(Joint[WST].GoalTimeCount <= Joint[WST].CurrentTimeCount)
				{
					Joint[WST].GoalTimeCount = Joint[WST].CurrentTimeCount = 0;
					Joint[WST].RefAngleCurrent = Joint[WST].RefAngleToGo;
					if(Joint[WST].DelayTimeCount[1] <= 0) Joint[WST].MoveFlag = false;	
					else Joint[WST].DelayTimeCount[1]--; // tail time delay
				}
				else 
				{
					if(Joint[WST].FunctionMode == 0x00)	// 0.5*(1-cos(pi*t/T))
					{
						Joint[WST].RefAngleCurrent = Joint[WST].RefAngleInitial+Joint[WST].RefAngleDelta*0.5f*(1.0f-cosf(PI/Joint[WST].GoalTimeCount*Joint[WST].CurrentTimeCount));
					}
					else if(Joint[WST].FunctionMode == 0x01)	// 0.5*(1-cos(2*pi*t/T))
					{
						Joint[WST].RefAngleCurrent = Joint[WST].RefAngleInitial+Joint[WST].UserData[0]*0.5f*(1.0f-cosf(2.0f*PI/Joint[WST].GoalTimeCount*Joint[WST].CurrentTimeCount));
					}
					else if(Joint[WST].FunctionMode == 0x02)	// 0.5*(1-cos(4*pi*t/T))
					{
						Joint[WST].RefAngleCurrent = Joint[WST].RefAngleInitial+Joint[WST].UserData[0]*0.5f*(1.0f-cosf(4.0f*PI/Joint[WST].GoalTimeCount*Joint[WST].CurrentTimeCount));
					}
				}
			}
			else Joint[WST].DelayTimeCount[0]--;
		}

		for(i=JMC0 ; i<=JMC7 ; i++) MoveJMC(i);
		MoveJMC(EJMC3);
	}
	else if(_canChannel == CAN1)
	{
		for(i=RSP ; i<=LF5 ; i++)
		{
			if(i != WST && i != EJMC2)
			{
			if(Joint[i].MoveFlag == true)
			{
				if(Joint[i].DelayTimeCount[0] <= 0) 
				{
					Joint[i].CurrentTimeCount++;
					if(Joint[i].GoalTimeCount <= Joint[i].CurrentTimeCount)
					{
						Joint[i].GoalTimeCount = Joint[i].CurrentTimeCount = 0;
						Joint[i].RefAngleCurrent = Joint[i].RefAngleToGo;
						Joint[i].RefVelCurrent = 0.0f;
						if(Joint[i].DelayTimeCount[1] <= 0) Joint[i].MoveFlag = false;	
						else Joint[i].DelayTimeCount[1]--; // tail time delay
					}
					else 
					{
						Joint[i].RefVelCurrent = Joint[i].RefAngleCurrent - RefAngle_Last[i];
						RefAngle_Last[i] = Joint[i].RefAngleCurrent;
					
						if(Joint[i].FunctionMode == 0x00)	// 0.5*(1-cos(pi*t/T))
						{
							Joint[i].RefAngleCurrent = Joint[i].RefAngleInitial+Joint[i].RefAngleDelta*0.5f*(1.0f-cosf(PI/Joint[i].GoalTimeCount*Joint[i].CurrentTimeCount));
						}
						else if(Joint[i].FunctionMode == 0x01)	// 0.5*(1-cos(2*pi*t/T))
						{
							Joint[i].RefAngleCurrent = Joint[i].RefAngleInitial+Joint[i].UserData[0]*0.5f*(1.0f-cosf(2.0f*PI/Joint[i].GoalTimeCount*Joint[i].CurrentTimeCount));
						}
						else if(Joint[i].FunctionMode == 0x02)	// 0.5*(1-cos(4*pi*t/T))
						{
							Joint[i].RefAngleCurrent = Joint[i].RefAngleInitial+Joint[i].UserData[0]*0.5f*(1.0f-cosf(4.0f*PI/Joint[i].GoalTimeCount*Joint[i].CurrentTimeCount));
						}
					}
				}
				else Joint[i].DelayTimeCount[0]--;
			}
			}
		}
		
		for(i=JMC8 ; i<=JMC11 ; i++) MoveJMC(i);
		//for(i=EJMC0 ; i<=EJMC2 ; i++) MoveJMC(i);
		for(i=EJMC0 ; i<EJMC2 ; i++) MoveJMC(i);
		for(i=EJMC4 ; i<=EJMC5 ; i++) MoveJMC(i);
	}
}
/******************************************************************************/


/*****************************************************************************/
char FTN_half_1_cos(float mag, long time, long start, long during, int delay1, int delay2, float *result)
{
	long	temp,temp1;
	temp=start+delay1;
	temp1=start+during-delay2;
	
	if(time<temp)
	{
		*result=0;
		return 0;
	}
	else if((time>=temp)&&(time<=temp1))
	{
		*result=(float)(mag*(1-(float)cos(PI*(float)(time-temp)/(float)(during-(delay1+delay2))))/2.0);
		return 1;
	}
	else return 3;
}
/*****************************************************************************/


/******************************************************************************/

char FingerMove(int mag,int vel,int time, int start, int end, float *result)
{
	int T;
	T=end-start;
	if(vel>127)vel=127;
	
	if(mag<0)
	{
		vel=-vel;
	}

	if( int(T/2) >= int(mag/vel))
	{
		if(time<start)
		{
			*result=0.0f;
			return 0;
		}
		else if( time >= start && time < start + int(mag/vel) )
		{
			*result= (float)(vel*(time-start));
			return 1;
		}
		else if( time >= start+int(mag/vel) && time < end-int(mag/vel) )
		{
			*result= (float)(vel*(int(mag/vel)));
			return 2;
		}
		else if( time >= end-int(mag/vel) && time < end )
		{
			*result= (float)(-vel*(time-start-T+int(mag/vel)) + vel*int(mag/vel));
			return 3;
		}
		else  
		{	*result=0.0f;
			return 4;
		}
	}
	else
	{
		if(time<start)
		{
			*result=0.0f;
			return 0;
		}
		else if(time >= start && time < start + int(T/2))
		{
			*result= (float)(vel*(time-start));
			return 1;
		}
		else if(time >= start + int(T/2) && time < end)
		{
			*result= (float)(-vel*(time-start-int(T/2)) + T/2*vel);
			return 2;
		}
		else 
		{
			*result = 0.0f;
			return 3;
		}
	}

}
/******************************************************************************/


/******************************************************************************/
void FingerControlModeChange(unsigned int _boardID, unsigned char _enable)
{
	unsigned char tempData[8];
	bool _enableFlag;

	if(_enable == 0x01) _enableFlag = true;
	else _enableFlag = false;

	tempData[0] = _boardID;
	tempData[1] = ControlMode;
	tempData[2] = _enable;		// Current Mode: 1, Position Mode : 0

	switch(_boardID)
	{
		
		/*case EJMC4:
			if(_enableFlag) RtWprintf(L"\n>>> EJMC4 Current Control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC4 Position Control is enabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;*/
		case EJMC4:  // inhyeok, drc
			tempData[2] = 0x02;		// ch0,2 : current mode, ch1:position mode
			if(_enableFlag) RtWprintf(L"\n>>> EJMC4 Current Control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC4 Position Control is enabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		case EJMC5:
			tempData[2] = 0x02;		// ch0,2 : current mode, ch1:position mode
			if(_enableFlag) RtWprintf(L"\n>>> EJMC5 Current Control is enabled..!!");
			else RtWprintf(L"\n>>> EJMC5 Position Control is disabled!!");
			PushCANMsg(1, CMD_TXDF, tempData, 3, 0);
			break;
		default:
			RtWprintf(L"\n>>> Wrong board ID(FingerControlModeChange)..!!");
			break;
	}
	RtSleep(10);
}
/******************************************************************************/



/******************************************************************************/
bool SetMoveTaskPos(WALKING_INFO* _walkingInfo, float _pos, float _msTime, float _msDelayTime[2], float _userData[5], unsigned char _mode, unsigned char _function)
{
	unsigned char i;

	if(_msTime <= 0) { RtWprintf(L"\n>>> Goal time must be grater than zero(SetMoveTaskPos)..!!"); return false; }

	switch(_mode)
	{
	case 0x00:	// relative mode
		_walkingInfo->RefPatternToGo = _walkingInfo->RefPatternCurrent + _pos;
		break;
	case 0x01:	// absolute mode
		_walkingInfo->RefPatternToGo = _pos;
		break;
	default:
		RtWprintf(L"\n>>> Wrong reference mode(SetMoveTaskPos)..!!");
		return false;
		break;
	}

	_walkingInfo->MoveFlag = false;
	
	_walkingInfo->RefPatternInitial = _walkingInfo->RefPatternCurrent;
	_walkingInfo->CurrentTimeCount = 0;
	_walkingInfo->DelayTimeCount[0] = ((unsigned long)(_msDelayTime[0]))/INT_TIME;
	_walkingInfo->DelayTimeCount[1] = ((unsigned long)(_msDelayTime[1]))/INT_TIME;
	_walkingInfo->GoalTimeCount = ((unsigned long)(_msTime))/INT_TIME - _walkingInfo->DelayTimeCount[0] - _walkingInfo->DelayTimeCount[1];
	_walkingInfo->RefPatternDelta = _walkingInfo->RefPatternToGo - _walkingInfo->RefPatternCurrent;
	_walkingInfo->FunctionMode = _function;
	for(i=0 ; i<5 ; i++) _walkingInfo->UserData[i] = _userData[i];
	
	_walkingInfo->MoveFlag = true;
	//printf("\n ref =%f",WalkingInfo[RIGHT][X].RefPatternCurrent);
	return true;
}
/******************************************************************************/





//jw3
/******************************************************************************/
void MoveTaskPos(WALKING_INFO* _walkingInfoSway, WALKING_INFO _walkingInfo[][4], JOINT _joint[], unsigned char _landingState, unsigned char _mode)
{	
	unsigned char i, j;
	float tempPos[6], tempAngle[6];
	float tempTime, tempMixPos, tempMixTime, tempMag;
	float tempSideWalk, tempSidePeriod, tempSideTime, tempSideDelay;

	if(_mode == CTRLMODE_WALKING)
	{	
		//if(_landingState == RIGHT_FOOT_LATE_LANDING) {if(SwayControlFlag == 0x00) SwayControlFlag = 0x03; }	// right foot late landing
		//else if(_landingState == LEFT_FOOT_LATE_LANDING) {if(SwayControlFlag == 0x00) SwayControlFlag = 0x04; }	// left foot late landing

		// for basic sway
		if(_walkingInfoSway->MoveFlag == true)
		{
			if(_walkingInfoSway->DelayTimeCount[0] == 0) 
			{
				_walkingInfoSway->CurrentTimeCount++;
				if(_walkingInfoSway->GoalTimeCount <= _walkingInfoSway->CurrentTimeCount)
				{
					_walkingInfoSway->GoalTimeCount = _walkingInfoSway->CurrentTimeCount = 0;
					_walkingInfoSway->RefPatternCurrent = _walkingInfoSway->RefPatternToGo;
					if(_walkingInfoSway->DelayTimeCount[1] == 0) _walkingInfoSway->MoveFlag = false;	
					else _walkingInfoSway->DelayTimeCount[1]--; // tail time delay
				}
				else
				{
					tempTime = (float)_walkingInfoSway->CurrentTimeCount/(float)_walkingInfoSway->GoalTimeCount;
					// function mode: 0  ->  mag.*0.5*(1.0-cos(pi*t))
					if(_walkingInfoSway->FunctionMode == 0) _walkingInfoSway->RefPatternCurrent = _walkingInfoSway->RefPatternInitial+_walkingInfoSway->RefPatternDelta*0.5f*(1.0f-cosf(PI*tempTime));
				}	
			}	
			else _walkingInfoSway->DelayTimeCount[0]--;		// head time delay
		}

		// current walking position generator 
		for(i=RIGHT ; i<=LEFT ; i++)	// right and left
		{
			for(j=X ; j<=Yaw ; j++)	// X, Y, Z and Yaw
			{
				if(_walkingInfo[i][j].MoveFlag == true)
				{
					if(_walkingInfo[i][j].DelayTimeCount[0] == 0) 
					{
						_walkingInfo[i][j].CurrentTimeCount++;
						if(_walkingInfo[i][j].GoalTimeCount <= _walkingInfo[i][j].CurrentTimeCount)
						{
							_walkingInfo[i][j].GoalTimeCount = _walkingInfo[i][j].CurrentTimeCount = 0;
							_walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternToGo;
							if(_walkingInfo[i][j].DelayTimeCount[1] == 0) _walkingInfo[i][j].MoveFlag = false;	
							else _walkingInfo[i][j].DelayTimeCount[1]--; // tail time delay
						}
						else
						{
							if( (j==Z) && (_landingState==RIGHT_FOOT_EARLY_LANDING) && ((WalkingStartStop == 0x02)||(WalkingStartStop == 0x01))&&((_walkingInfo[RIGHT][Z].CurrentSequence==RIGHT_FOOT_DOWN_SET)||(_walkingInfo[RIGHT][Z].CurrentSequence==FIRST_RIGHT_FOOT_DOWN_SET)))	// right leg early landing
							{
								_walkingInfo[RIGHT][Z].RefPatternInitial = _walkingInfo[RIGHT][Z].RefPatternToGo = _walkingInfo[RIGHT][Z].RefPatternCurrent; 
								_walkingInfo[RIGHT][Z].RefPatternDelta = 0.0f;		
							}
							else if( (j==Z) && (_landingState==LEFT_FOOT_EARLY_LANDING) && ((WalkingStartStop == 0x02)||(WalkingStartStop == 0x01))&&((_walkingInfo[LEFT][Z].CurrentSequence==LEFT_FOOT_DOWN_SET)||(_walkingInfo[LEFT][Z].CurrentSequence==FIRST_LEFT_FOOT_DOWN_SET)) )	// left leg early landing
							{
								_walkingInfo[LEFT][Z].RefPatternInitial = _walkingInfo[LEFT][Z].RefPatternToGo = _walkingInfo[LEFT][Z].RefPatternCurrent; 
								_walkingInfo[LEFT][Z].RefPatternDelta = 0.0f;
							} 
							//else	// normal sequence
							{
								tempTime = (float)_walkingInfo[i][j].CurrentTimeCount/(float)_walkingInfo[i][j].GoalTimeCount;
								// function mode: 0  ->  mag.*0.5*(1.0-cos(pi*t))
								if(_walkingInfo[i][j].FunctionMode == 0) _walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*0.5f*(1.0f-cosf(PI*tempTime));
								// function mode: 1	->	mag.*0.5*(1.0-cos(0.5*pi*t)) ("1-cos" head)
								else if(_walkingInfo[i][j].FunctionMode == 1) _walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*(1.0f-cosf(0.5f*PI*tempTime));
								// function mode: 2	->	mag.*0.5*(-cos(0.5(pi*(t+1))) ("1-cos" tail)
								else if(_walkingInfo[i][j].FunctionMode == 2) _walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*(-cosf(0.5f*PI*(tempTime+1.0f)));
								// function mode: 3  ->  mag.*sin(0.5*pi*t)
								else if(_walkingInfo[i][j].FunctionMode == 3) _walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*sinf(0.5f*PI*tempTime);
								// function mode: 4  ->  mag.*(t-sin(pi*t)/pi)
								else if(_walkingInfo[i][j].FunctionMode == 4) _walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*(tempTime-sinf(PI*tempTime)/PI);
								// function mode: 5  ->  mag.*(t+sin(pi*t)/pi)
								else if(_walkingInfo[i][j].FunctionMode == 5) _walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*(tempTime+sinf(PI*tempTime)/PI);
								// function mode: 6	->	inti.* (t+sin(pi*t')/pi)-init.*(1-cos(0.5*pi*t))
								else if(_walkingInfo[i][j].FunctionMode == 6)
								{
									tempMag = _walkingInfo[i][j].RefPatternInitial;
									tempMixTime = (float)_walkingInfo[i][j].CurrentTimeCount/((float)_walkingInfo[i][j].GoalTimeCount-_walkingInfo[i][j].UserData[0]/INT_TIME);
									tempMixPos = _walkingInfo[i][j].RefPatternInitial+tempMag*(tempMixTime+sinf(PI*tempMixTime)/PI);
									if(lastStep4>= 0.0f)
									{
										if( tempMixPos >= _walkingInfo[i][j].RefPatternInitial+tempMag ) tempMixPos = _walkingInfo[i][j].RefPatternInitial+tempMag;
									}
									else
									{
										if( tempMixPos <= _walkingInfo[i][j].RefPatternInitial+tempMag ) tempMixPos = _walkingInfo[i][j].RefPatternInitial+tempMag;
									}
									/*
									else
									{
										if(_walkingInfo[i][j].UserData[1] >= 0.0f)
										{
											if( tempMixPos >= _walkingInfo[i][j].RefPatternInitial+tempMag ) tempMixPos = _walkingInfo[i][j].RefPatternInitial+tempMag;
										}
										else
										{
											if( tempMixPos <= _walkingInfo[i][j].RefPatternInitial+tempMag ) tempMixPos = _walkingInfo[i][j].RefPatternInitial+tempMag;
										}
									}*/
									_walkingInfo[i][j].RefPatternCurrent = tempMixPos - tempMag*(1.0f-cosf(0.5f*PI*tempTime));
								}
								// function mode: 7	->	mag.*2*(t+sin(pi*t')/pi)-mag.*(sin(0.5*pi*t))
								else if(_walkingInfo[i][j].FunctionMode == 7)
								{
									tempMixTime = ((float)_walkingInfo[i][j].CurrentTimeCount-_walkingInfo[i][j].UserData[0]/INT_TIME)/((float)_walkingInfo[i][j].GoalTimeCount-_walkingInfo[i][j].UserData[0]/INT_TIME);
									if(tempMixTime < 0.0f) tempMixTime = 0.0f;
									tempMixPos = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*2.0f*(tempMixTime-sinf(PI*tempMixTime)/PI);
									_walkingInfo[i][j].RefPatternCurrent = tempMixPos - _walkingInfo[i][j].RefPatternDelta*(sinf(0.5f*PI*tempTime));
								}
								// function mode: 8	->	mag.*2*(t+sin(pi*t')/pi)-mag.*(1-cos(0.5*pi*t))
								else if(_walkingInfo[i][j].FunctionMode == 8)
								{
									tempMixTime = (float)_walkingInfo[i][j].CurrentTimeCount/((float)_walkingInfo[i][j].GoalTimeCount-_walkingInfo[i][j].UserData[0]/INT_TIME);
									tempMixPos = _walkingInfo[i][j].RefPatternInitial+2.0f*_walkingInfo[i][j].RefPatternDelta*(tempMixTime+sinf(PI*tempMixTime)/PI);
									if(lastStep4 > 0.0f)
									{
										if(tempMixPos >= _walkingInfo[i][j].RefPatternInitial+2.0f*_walkingInfo[i][j].RefPatternDelta) tempMixPos = _walkingInfo[i][j].RefPatternInitial+2.0f*_walkingInfo[i][j].RefPatternDelta;
									}
									else if(lastStep4 < 0.0f)
									{
										if( tempMixPos <= _walkingInfo[i][j].RefPatternInitial+2.0f*_walkingInfo[i][j].RefPatternDelta ) tempMixPos = _walkingInfo[i][j].RefPatternInitial+2.0f*_walkingInfo[i][j].RefPatternDelta;
									}
									else
									{
										if(_walkingInfo[i][j].UserData[1] >= 0.0f)
										{
											if(tempMixPos >= _walkingInfo[i][j].RefPatternInitial+2.0f*_walkingInfo[i][j].RefPatternDelta) tempMixPos = _walkingInfo[i][j].RefPatternInitial+2.0f*_walkingInfo[i][j].RefPatternDelta;
										}
										else
											{
											if( tempMixPos <= _walkingInfo[i][j].RefPatternInitial+2.0f*_walkingInfo[i][j].RefPatternDelta ) tempMixPos = _walkingInfo[i][j].RefPatternInitial+2.0f*_walkingInfo[i][j].RefPatternDelta;
										}
									}
									_walkingInfo[i][j].RefPatternCurrent = tempMixPos - _walkingInfo[i][j].RefPatternDelta*(1.0f-cosf(0.5f*PI*tempTime));
								}
								// function mode: 9	->	mag.*(t+sin(pi*t')/pi)-mag.*(sin(0.5*pi*t))
								else if(_walkingInfo[i][j].FunctionMode == 9)
								{
									tempMag = _walkingInfo[i][j].RefPatternInitial;
									tempMixTime = (float)_walkingInfo[i][j].CurrentTimeCount/((float)_walkingInfo[i][j].GoalTimeCount-_walkingInfo[i][j].UserData[0]/INT_TIME);
									tempMixPos = _walkingInfo[i][j].RefPatternInitial-tempMag*(tempMixTime-sinf(PI*tempMixTime)/PI);
									if(pSharedMemory->JW_temp[6] > 0.0f)
									{
										if(tempMixPos >= _walkingInfo[i][j].RefPatternInitial-tempMag) tempMixPos = _walkingInfo[i][j].RefPatternInitial-tempMag;
									}
									else if(pSharedMemory->JW_temp[6] < 0.0f)
									{
										if(tempMixPos <= _walkingInfo[i][j].RefPatternInitial-tempMag) tempMixPos = _walkingInfo[i][j].RefPatternInitial-tempMag;
									}
									else
									{
										if(_walkingInfo[i][j].UserData[1] >= 0.0f)
										{
											if(tempMixPos >= _walkingInfo[i][j].RefPatternInitial-tempMag) tempMixPos = _walkingInfo[i][j].RefPatternInitial-tempMag;
										}
										else
										{
											if(tempMixPos <= _walkingInfo[i][j].RefPatternInitial-tempMag) tempMixPos = _walkingInfo[i][j].RefPatternInitial-tempMag;
										}
									}
									_walkingInfo[i][j].RefPatternCurrent = tempMag + tempMixPos - tempMag*(1.0f-sinf(0.5f*PI*tempTime));
									//printf("\n %f,%f",_walkingInfo[RIGHT][X].RefPatternCurrent,tempTime);
									
									

								}
								// function mode: 10	->	mag.*(t+sin(pi*t')/pi)
								else if(_walkingInfo[i][j].FunctionMode == 10)
								{
									//printf("\n %f",_walkingInfo[RIGHT][X].RefPatternInitial);
									tempMixTime = (float)_walkingInfo[i][j].CurrentTimeCount/((float)_walkingInfo[i][j].GoalTimeCount-_walkingInfo[i][j].UserData[0]/INT_TIME);
									tempMixPos = _walkingInfo[i][j].RefPatternInitial+1.0f*_walkingInfo[i][j].RefPatternDelta*(tempMixTime+sinf(PI*tempMixTime)/PI);
									if(pSharedMemory->JW_temp[6] > 0.0f)
									{
										if(tempMixPos >= _walkingInfo[i][j].RefPatternInitial+1.0f*_walkingInfo[i][j].RefPatternDelta) tempMixPos = _walkingInfo[i][j].RefPatternInitial+1.0f*_walkingInfo[i][j].RefPatternDelta;
									}
									else if(pSharedMemory->JW_temp[6] < 0.0f)
									{
										if(tempMixPos <= _walkingInfo[i][j].RefPatternInitial+1.0f*_walkingInfo[i][j].RefPatternDelta) tempMixPos = _walkingInfo[i][j].RefPatternInitial+1.0f*_walkingInfo[i][j].RefPatternDelta;
									}
									else
									{
										if(_walkingInfo[i][j].UserData[1] >= 0.0f)
										{
											if(tempMixPos >= _walkingInfo[i][j].RefPatternInitial+1.0f*_walkingInfo[i][j].RefPatternDelta) tempMixPos = _walkingInfo[i][j].RefPatternInitial+1.0f*_walkingInfo[i][j].RefPatternDelta;
										}
										else
										{
											if(tempMixPos <= _walkingInfo[i][j].RefPatternInitial+1.0f*_walkingInfo[i][j].RefPatternDelta) tempMixPos = _walkingInfo[i][j].RefPatternInitial+1.0f*_walkingInfo[i][j].RefPatternDelta;
										}
									}
									_walkingInfo[i][j].RefPatternCurrent = tempMixPos;
								}
								else if(_walkingInfo[i][j].FunctionMode == 11)
								{
									tempSidePeriod = (float)_walkingInfo[i][j].GoalTimeCount/2.0f - _walkingInfo[i][j].UserData[1]/INT_TIME;
									tempSideTime = (float)_walkingInfo[i][j].CurrentTimeCount/tempSidePeriod;
									tempSideDelay = 2.0f*_walkingInfo[i][j].UserData[1]/INT_TIME/tempSidePeriod;

									if(tempSideTime < 1.0f) tempSideWalk = _walkingInfo[i][j].UserData[0]*0.5f*(1.0f-cosf(PI*tempSideTime));
									else if( tempSideTime < (1.0f+tempSideDelay) ) tempSideWalk = _walkingInfo[i][j].UserData[0];
									else if( tempSideTime < (2.0f+tempSideDelay) ) tempSideWalk = _walkingInfo[i][j].UserData[0]*0.5f*(1.0f+cosf(PI*(tempSideTime-1.0f-tempSideDelay)));
									else tempSideWalk = 0.0f;
 
									_walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*0.5f*(1.0f-cosf(PI*tempTime)) + tempSideWalk;
								}
								else if(_walkingInfo[i][j].FunctionMode == 12)
								{
									tempSidePeriod = (float)_walkingInfo[i][j].GoalTimeCount/2.0f - _walkingInfo[i][j].UserData[1]/INT_TIME;
									tempSideTime = (float)_walkingInfo[i][j].CurrentTimeCount/tempSidePeriod;
									tempSideDelay = 2.0f*_walkingInfo[i][j].UserData[1]/INT_TIME/tempSidePeriod;

									if(tempSideTime < 1.0f) tempSideWalk = _walkingInfo[i][j].UserData[0]*0.5f*(1.0f-cosf(PI*tempSideTime));
									else if( tempSideTime < (1.0f+tempSideDelay) ) tempSideWalk = _walkingInfo[i][j].UserData[0];
									else if( tempSideTime < (2.0f+tempSideDelay) ) tempSideWalk = _walkingInfo[i][j].UserData[0]*0.5f*(1.0f+cosf(PI*(tempSideTime-1.0f-tempSideDelay)));
									else tempSideWalk = 0.0f;
 
									_walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*0.5f*(1.0f-cosf(PI*tempTime)) - tempSideWalk;
								}
							}
						}	// if(_walkingInfo[i][j].GoalTimeCount <= _walkingInfo[i][j].CurrentTimeCount)
					}	// if(_walkingInfo[i][j].DelayTimeCount[0] == 0)
					else _walkingInfo[i][j].DelayTimeCount[0]--;		// head time delay
				}	// end of "if(WalkingInfo[i][j].MoveFlag == true)"
			}	// end of "for(j=X ; j<=Yaw ; j++)"
		}	// end of "for(i=RIGHT ; i<=LEFT ; i++)"	
	}
	else if(_mode == CTRLMODE_ZMP_INITIALIZATION)
	{
	}
	else;

	MoveTaskPosJointFF(_joint);	// FF control
	// JW1
	// right leg joint angle reference
	tempPos[0] = (_walkingInfo[RIGHT][X].RefPatternCurrent + _walkingInfo[RIGHT][X].RefPosFF + _walkingInfo[RIGHT][X].ControlDSPZMP)+(_walkingInfoSway->RefPatternCurrent + _walkingInfo[RIGHT][Y].RefPatternCurrent + _walkingInfo[RIGHT][Y].RefPosFF + _walkingInfo[RIGHT][Y].ControlDSPZMP)*sinf(_joint[RHY].RefAngleCurrent*PI/180);	// X
	tempPos[1] = (_walkingInfoSway->RefPatternCurrent + _walkingInfo[RIGHT][Y].RefPatternCurrent + _walkingInfo[RIGHT][Y].RefPosFF + _walkingInfo[RIGHT][Y].ControlDSPZMP)*cosf(_joint[RHY].RefAngleCurrent*PI/180);	// Y
	tempPos[2] = _walkingInfo[RIGHT][Z].RefPatternCurrent + _walkingInfo[RIGHT][Z].RefPosFF;	// Z
	tempPos[3] = 0.0f;	// Roll
	tempPos[4] = 0.0f;	// Pitch
	tempPos[5] = _walkingInfo[RIGHT][Yaw].RefPatternCurrent;// + _walkingInfo[RIGHT][Yaw].RefPosFF;	// Yaw
	InverseKinematics(tempPos, tempAngle);
	_joint[RHY].WalkingRefAngle = tempAngle[0];
	_joint[RHR].WalkingRefAngle = tempAngle[1];
	_joint[RHP].WalkingRefAngle = tempAngle[2];
	_joint[RKN].WalkingRefAngle = tempAngle[3];
	_joint[RAP].WalkingRefAngle = tempAngle[4];
	_joint[RAR].WalkingRefAngle = tempAngle[5];

	_joint[RHY].RefAngleCurrent = _joint[RHY].WalkingRefAngle + _joint[RHY].RefAngleFF + _joint[RHY].ControlAngleFF;
	_joint[RHR].RefAngleCurrent = _joint[RHR].WalkingRefAngle + _joint[RHR].RefAngleFF + _joint[RHR].ControlVibrationAngle +  _joint[RHR].ControlAngleFF;
	_joint[RHP].RefAngleCurrent = _joint[RHP].WalkingRefAngle + _joint[RHP].RefAngleFF + _joint[RHP].ControlAngleFF;
	_joint[RKN].RefAngleCurrent = _joint[RKN].WalkingRefAngle + _joint[RKN].RefAngleFF + _joint[RKN].ControlAngleFF;
	_joint[RAP].RefAngleCurrent = _joint[RAP].WalkingRefAngle + _joint[RAP].RefAngleFF + _joint[RAP].ControlDampAngleCurrent + _joint[RAP].ControlAngleFF;
	_joint[RAR].RefAngleCurrent = _joint[RAR].WalkingRefAngle + _joint[RAR].RefAngleFF + _joint[RAR].ControlDampAngleCurrent + _joint[RAR].ControlAngleFF;

	


	// left leg joint angle reference
	tempPos[0] = (_walkingInfo[LEFT][X].RefPatternCurrent + _walkingInfo[LEFT][X].RefPosFF + _walkingInfo[LEFT][X].ControlDSPZMP)+(_walkingInfoSway->RefPatternCurrent + _walkingInfo[LEFT][Y].RefPatternCurrent + _walkingInfo[LEFT][Y].RefPosFF + _walkingInfo[LEFT][Y].ControlDSPZMP)*sinf(_joint[LHY].RefAngleCurrent*PI/180);	// X
	tempPos[1] = (_walkingInfoSway->RefPatternCurrent + _walkingInfo[LEFT][Y].RefPatternCurrent + _walkingInfo[LEFT][Y].RefPosFF + _walkingInfo[LEFT][Y].ControlDSPZMP)*cosf(_joint[LHY].RefAngleCurrent*PI/180);	// Y
	tempPos[2] = _walkingInfo[LEFT][Z].RefPatternCurrent + _walkingInfo[LEFT][Z].RefPosFF;	// Z
	tempPos[3] = 0.0f;	// Roll
	tempPos[4] = 0.0f;	// Pitch
	tempPos[5] = _walkingInfo[LEFT][Yaw].RefPatternCurrent;// + _walkingInfo[LEFT][Yaw].RefPosFF;	// Yaw
	InverseKinematics(tempPos, tempAngle);
	_joint[LHY].WalkingRefAngle = tempAngle[0];
	_joint[LHR].WalkingRefAngle = tempAngle[1];
	_joint[LHP].WalkingRefAngle = tempAngle[2];
	_joint[LKN].WalkingRefAngle = tempAngle[3];
	_joint[LAP].WalkingRefAngle = tempAngle[4];
	_joint[LAR].WalkingRefAngle = tempAngle[5];

	_joint[LHY].RefAngleCurrent = _joint[LHY].WalkingRefAngle + _joint[LHY].RefAngleFF + _joint[LHY].ControlAngleFF;
	_joint[LHR].RefAngleCurrent = _joint[LHR].WalkingRefAngle + _joint[LHR].RefAngleFF + _joint[LHR].ControlVibrationAngle + _joint[LHR].ControlAngleFF;
	_joint[LHP].RefAngleCurrent = _joint[LHP].WalkingRefAngle + _joint[LHP].RefAngleFF + _joint[LHP].ControlAngleFF;
	_joint[LKN].RefAngleCurrent = _joint[LKN].WalkingRefAngle + _joint[LKN].RefAngleFF + _joint[LKN].ControlAngleFF;
	_joint[LAP].RefAngleCurrent = _joint[LAP].WalkingRefAngle + _joint[LAP].RefAngleFF + _joint[LAP].ControlDampAngleCurrent + _joint[LAP].ControlAngleFF;
	_joint[LAR].RefAngleCurrent = _joint[LAR].WalkingRefAngle + _joint[LAR].RefAngleFF + _joint[LAR].ControlDampAngleCurrent + _joint[LAR].ControlAngleFF;

	//JW Arm Movement	
	/*
	tempPos[0] = _walkingInfo[LEFT][X].RefPatternCurrent;	// X
	tempPos[1] = _walkingInfoSway->RefPatternCurrent;	// Y
	tempPos[2] = _walkingInfo[LEFT][Z].RefPatternCurrent;	// Z
	tempPos[3] = 0.0f;	// Roll
	tempPos[4] = 0.0f;	// Pitch
	tempPos[5] = 0.0f;// + _walkingInfo[LEFT][Yaw].RefPosFF;	// Yaw
	InverseKinematics(tempPos, tempAngle);
	_joint[RSP].RefAngleCurrent = _joint[RSP].WalkReadyAngle-(1+pSharedMemory->JW_temp[6]*20)*tempAngle[1]; 
	_joint[LSP].RefAngleCurrent = _joint[LSP].WalkReadyAngle+(1+pSharedMemory->JW_temp[6]*20)*tempAngle[1]; //JW Arm Movement
*/
	// goto reference angle
	for(i=JMC0 ; i<=JMC7 ; i++) MoveJMC(i);
}
/******************************************************************************/





/******************************************************************************/
bool SetMoveTaskPosJointFF(JOINT* _joint, float _angle, float _msTime, float _msDelayTime[2], unsigned char _mode, unsigned char _function)
{
	if(_msTime <= 0) { RtWprintf(L"\n>>> goal time must be grater than zero(SetMoveTaskPosJointFF)..!!"); return false; }
	
	switch(_mode)
	{
	case 0x00:	// relative mode
		_joint->ControlAngleFFToGo = _joint->ControlAngleFF + _angle;
		break;
	case 0x01:	// absolute mode
		_joint->ControlAngleFFToGo = _angle;
		break;
	default:
		RtWprintf(L"\n>>> Wrong reference mode(SetMoveTaskPosJointFF)..!!");
		return false;
		break;
	}

	_joint->ControlFFMoveFlag = false;
	_joint->ControlAngleFFDelta = _joint->ControlAngleFFToGo - _joint->ControlAngleFF;
	_joint->ControlAngleFFInitial = _joint->ControlAngleFF;
	_joint->ControlFFGoalTimeCount = (unsigned long)(_msTime/INT_TIME);
	_joint->ControlFFCurrentTimeCount = 0;
	_joint->ControlFFDelayTimeCount[0] = ((unsigned long)(_msDelayTime[0]))/INT_TIME;
	_joint->ControlFFDelayTimeCount[1] = ((unsigned long)(_msDelayTime[1]))/INT_TIME;
	_joint->ControlFFFunctionMode = _function;
	_joint->ControlFFMoveFlag = true;

	RtWprintf(L"\n>>> Joint reference is set(SetMoveTaskPosJointFF)..!!");

	return true;
}
/******************************************************************************/






/******************************************************************************/
void MoveTaskPosJointFF(JOINT _joint[])
{
	unsigned char i;
	float tempTime;
	
	for(i=RHY ; i<=LAR ; i++)
	{
	
		if(_joint[i].ControlFFMoveFlag == true)
		{
			if(_joint[i].ControlFFDelayTimeCount[0] == 0) 
			{
				_joint[i].ControlFFCurrentTimeCount++;
				if(_joint[i].ControlFFGoalTimeCount <= _joint[i].ControlFFCurrentTimeCount)
				{
					_joint[i].ControlFFGoalTimeCount = _joint[i].ControlFFCurrentTimeCount = 0;
					_joint[i].ControlAngleFF = _joint[i].ControlAngleFFToGo;
					if(_joint[i].ControlFFDelayTimeCount[1] == 0) _joint[i].ControlFFMoveFlag = false;	
					else _joint[i].ControlFFDelayTimeCount[1]--; // tail time delay
				}
				else
				{
					tempTime = (float)_joint[i].ControlFFCurrentTimeCount/(float)_joint[i].ControlFFGoalTimeCount;
					// function mode: 0  ->  mag.*0.5*(1.0-cos(pi*t))
					if(_joint[i].ControlFFFunctionMode == 0) _joint[i].ControlAngleFF = _joint[i].ControlAngleFFInitial+_joint[i].ControlAngleFFDelta*0.5f*(1.0f-cosf(PI*tempTime));
				}	
			}	
			else _joint[i].ControlFFDelayTimeCount[0]--;		// head time delay
		}
	}
}
/******************************************************************************/






/******************************************************************************/
void MoveJMC(unsigned char _boardID)
{
	unsigned char tempData[8];
	int tempPulse;
	unsigned int currentPulse;
	short short_temp;
	unsigned short short_currentPulse;

	switch(_boardID)
	{
		case JMC0:
			tempPulse = (int)( (Joint[RHY].RefAngleCurrent)*Joint[RHY].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			
			tempPulse = (int)(Joint[RHR].RefAngleCurrent*Joint[RHR].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			PushCANMsg(Joint[RHY].CAN_channel, Joint[RHY].Ref_txdf, tempData, 6, 0);
			break;
		case JMC1:
			tempPulse = (int)((Joint[RHP].RefAngleCurrent + Joint[RHP].MocapData)*Joint[RHP].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			PushCANMsg(Joint[RHP].CAN_channel, Joint[RHP].Ref_txdf, tempData, 3, 0);
			break;
		case JMC2:
			tempPulse = (int)(Joint[RKN].RefAngleCurrent*Joint[RKN].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			
			PushCANMsg(Joint[RKN].CAN_channel, Joint[RKN].Ref_txdf, tempData, 3, 0);
			break;
		case JMC3:
			tempPulse = (int)(Joint[RAP].RefAngleCurrent*Joint[RAP].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[RAR].RefAngleCurrent*Joint[RAR].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			
			PushCANMsg(Joint[RAP].CAN_channel, Joint[RAP].Ref_txdf, tempData, 6, 0);
			break;
		case JMC4:
			tempPulse = (int)(Joint[LHY].RefAngleCurrent*Joint[LHY].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[LHR].RefAngleCurrent*Joint[LHR].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			
			PushCANMsg(Joint[LHY].CAN_channel, Joint[LHY].Ref_txdf, tempData, 6, 0);
			break;			
		case JMC5:
			tempPulse = (int)((Joint[LHP].RefAngleCurrent + Joint[RHP].MocapData)*Joint[LHP].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			
			PushCANMsg(Joint[LHP].CAN_channel, Joint[LHP].Ref_txdf, tempData, 3, 0);		
			break;
		case JMC6:
			tempPulse = (int)(Joint[LKN].RefAngleCurrent*Joint[LKN].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			
			PushCANMsg(Joint[LKN].CAN_channel, Joint[LKN].Ref_txdf, tempData, 3, 0);
			break;
		case JMC7:
			tempPulse = (int)(Joint[LAP].RefAngleCurrent*Joint[LAP].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[LAR].RefAngleCurrent*Joint[LAR].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			
			PushCANMsg(Joint[LAP].CAN_channel, Joint[LAP].Ref_txdf, tempData, 6, 0);
			break;
		case EJMC3:
			tempPulse = (int)(Joint[WST].RefAngleCurrent*Joint[WST].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			PushCANMsg(Joint[WST].CAN_channel, Joint[WST].Ref_txdf, tempData, 3, 0);
			break;
		case JMC8:
			tempPulse = (int)(Joint[RSP].RefAngleCurrent*Joint[RSP].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[RSR].RefAngleCurrent*Joint[RSR].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			PushCANMsg(Joint[RSP].CAN_channel, Joint[RSP].Ref_txdf, tempData, 6, 0);
			break;
		case JMC9:
			tempPulse = (int)(Joint[RSY].RefAngleCurrent*Joint[RSY].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[REB].RefAngleCurrent*Joint[REB].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			PushCANMsg(Joint[RSY].CAN_channel, Joint[RSY].Ref_txdf, tempData, 6, 0);
			break;
		case JMC10:
			tempPulse = (int)(Joint[LSP].RefAngleCurrent*Joint[LSP].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[LSR].RefAngleCurrent*Joint[LSR].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			PushCANMsg(Joint[LSP].CAN_channel, Joint[LSP].Ref_txdf, tempData, 6, 0);
			break;
		case JMC11:
			tempPulse = (int)(Joint[LSY].RefAngleCurrent*Joint[LSY].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[LEB].RefAngleCurrent*Joint[LEB].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			PushCANMsg(Joint[LSY].CAN_channel, Joint[LSY].Ref_txdf, tempData, 6, 0);
			break;		
		case EJMC0:			
			tempPulse = (int)(Joint[RWY].RefAngleCurrent*Joint[RWY].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[RWP].RefAngleCurrent*Joint[RWP].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			PushCANMsg(Joint[RWY].CAN_channel, Joint[RWY].Ref_txdf, tempData, 6, 0);	
			break;
		case EJMC1:
			tempPulse = (int)(Joint[LWY].RefAngleCurrent*Joint[LWY].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			tempPulse = (int)(Joint[LWP].RefAngleCurrent*Joint[LWP].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[3] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[4] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[5] = (unsigned char)( (currentPulse>>16) & 0x000000FF );
			PushCANMsg(Joint[LWY].CAN_channel, Joint[LWY].Ref_txdf, tempData, 6, 0);	
			break;
		case EJMC2:
			/*//------------------------------------------- original
			char_temp = (char)(Joint[NKY].RefVelCurrent*Joint[NKY].PPR);
			//Truncation error compensation
			NKY_err_short_float[0] += (Joint[NKY].RefVelCurrent*Joint[NKY].PPR - (float)char_temp);	// mod by Inhyeok
			if(NKY_err_short_float[0] >= 1.0f) 
			{
				if(NKY_err_short_float[0] > 0.)
				{
					char_temp += 1;
					NKY_err_short_float[0] -= 1.0f;
				}
				else
				{
					char_temp -= 1;
					NKY_err_short_float[0] += 1.0f;
				}
			}
			tempData[0] = char_temp;
			
			//-------------------------------------------------------			
			char_temp = (char)(Joint[NK1].RefVelCurrent*Joint[NK1].PPR);			
			
			//Truncation error compensation
			NKY_err_short_float[1] += (Joint[NK1].RefVelCurrent*Joint[NK1].PPR - (float)char_temp);			
			while( fabs(NKY_err_short_float[1]) >= 1.0f) 
			{
				if(NKY_err_short_float[1] > 0.)
				{
					char_temp += 1;
					NKY_err_short_float[1] -= 1.0f;
				}
				else
				{
					char_temp -= 1;
					NKY_err_short_float[1] += 1.0f;
				}
				
			}
			tempData[1] = char_temp;
			//-------------------------------------------------------
			
			char_temp = (char)(Joint[NK2].RefVelCurrent*Joint[NK2].PPR);
			//Truncation error compensation
			NKY_err_short_float[2] += (Joint[NK2].RefVelCurrent*Joint[NK2].PPR - (float)char_temp);
			if(NKY_err_short_float[2] >= 1.0f) 
			{
				if(NKY_err_short_float[2] > 0.)
				{
					char_temp += 1;
					NKY_err_short_float[2] -= 1.0f;
				}
				else
				{
					char_temp -= 1;
					NKY_err_short_float[2] += 1.0f;
				}
			}			
			tempData[2] = char_temp;			
			PushCANMsg(Joint[NKY].CAN_channel, Joint[NKY].Ref_txdf, tempData, 3, 0);
			//------------------------------------------------------*/

			if(Joint[NKY].RefAngleCurrent < NKYmin)
				Joint[NKY].RefAngleCurrent = NKYmin;
			else if(Joint[NKY].RefAngleCurrent > NKYmax)
				Joint[NKY].RefAngleCurrent = NKYmax;

			if(Joint[NK1].RefAngleCurrent < NK1min)
				Joint[NK1].RefAngleCurrent = NK1min;
			else if(Joint[NK1].RefAngleCurrent > NK1max)
				Joint[NK1].RefAngleCurrent = NK1max;

			if(Joint[NK2].RefAngleCurrent < NK2min)
				Joint[NK2].RefAngleCurrent = NK2min;
			else if(Joint[NK2].RefAngleCurrent > NK2max)
				Joint[NK2].RefAngleCurrent = NK2max;

			short_currentPulse = NeckDynamixelPos(Joint[NKY].RefAngleCurrent);
			tempData[0] = (unsigned char)(short_currentPulse & 0x00FF);
			tempData[1] = (unsigned char)((short_currentPulse>>8) & 0x00FF);

			short_currentPulse = NeckDynamixelPos(Joint[NK1].RefAngleCurrent);
			tempData[2] = (unsigned char)(short_currentPulse & 0x00FF);
			tempData[3] = (unsigned char)((short_currentPulse>>8) & 0x00FF);

			short_currentPulse = NeckDynamixelPos(Joint[NK2].RefAngleCurrent);
			tempData[4] = (unsigned char)(short_currentPulse & 0x00FF);
			tempData[5] = (unsigned char)((short_currentPulse>>8) & 0x00FF);

			PushCANMsg(Joint[NKY].CAN_channel, Joint[NKY].Ref_txdf, tempData, 6, 0);
			break;		
				
		case EJMC4:
			/*
			short_temp = (short)(Joint[RF1].RefVelCurrent);
			HAND_err_short_float[0] += (Joint[RF1].RefVelCurrent - short_temp);
			if(HAND_err_short_float[0] >= 1.0f) 
			{
				if(HAND_err_short_float[0] > 0.)
				{
					short_temp += 1;
					HAND_err_short_float[0] -= 1.0f;
				}
				else
				{
					short_temp -= 1;
					HAND_err_short_float[0] += 1.0f;
				}
			}	
			short_currentPulse = SignConventionFinger(short_temp,finger_control_mode);
			tempData[0] = (unsigned char)(short_currentPulse & 0x000000FF);
			//================
			short_temp = (short)(Joint[RF2].RefVelCurrent);
			HAND_err_short_float[1] += (Joint[RF2].RefVelCurrent - short_temp);
			if(HAND_err_short_float[1] >= 1.0f) 
			{
				if(HAND_err_short_float[1] > 0.)
				{
					short_temp += 1;
					HAND_err_short_float[1] -= 1.0f;
				}
				else
				{
					short_temp -= 1;
					HAND_err_short_float[1] += 1.0f;
				}
			}	
			short_currentPulse = SignConventionFinger(short_temp,finger_control_mode);
			tempData[1] = (unsigned char)(short_currentPulse & 0x000000FF);
			//================
			short_temp = (short)(Joint[RF3].RefVelCurrent);
			HAND_err_short_float[2] += (Joint[RF3].RefVelCurrent - short_temp);
			if(HAND_err_short_float[2] >= 1.0f) 
			{
				if(HAND_err_short_float[2] > 0.)
				{
					short_temp += 1;
					HAND_err_short_float[2] -= 1.0f;
				}
				else
				{
					short_temp -= 1;
					HAND_err_short_float[2] += 1.0f;
				}
			}	
			short_currentPulse = SignConventionFinger(short_temp,finger_control_mode);
			tempData[2] = (unsigned char)(short_currentPulse & 0x000000FF);
			//================
			short_temp = (short)(Joint[RF4].RefVelCurrent);
			HAND_err_short_float[3] += (Joint[RF4].RefVelCurrent - short_temp);
			if(HAND_err_short_float[3] >= 1.0f) 
			{
				if(HAND_err_short_float[3] > 0.)
				{
					short_temp += 1;
					HAND_err_short_float[3] -= 1.0f;
				}
				else
				{
					short_temp -= 1;
					HAND_err_short_float[3] += 1.0f;
				}
			}	
			short_currentPulse = SignConventionFinger(short_temp,finger_control_mode);
			tempData[3] = (unsigned char)(short_currentPulse & 0x000000FF);
			//================
			short_temp = (short)(Joint[RF5].RefVelCurrent);
			HAND_err_short_float[4] += (Joint[RF5].RefVelCurrent - short_temp);
			if(HAND_err_short_float[4] >= 1.0f) 
			{
				if(HAND_err_short_float[4] > 0.)
				{
					short_temp += 1;
					HAND_err_short_float[4] -= 1.0f;
				}
				else
				{
					short_temp -= 1;
					HAND_err_short_float[4] += 1.0f;
				}
			}	
			short_currentPulse = SignConventionFinger(short_temp,finger_control_mode);
			tempData[4] = (unsigned char)(short_currentPulse & 0x000000FF);
			PushCANMsg(Joint[RF1].CAN_channel, Joint[RF1].Ref_txdf, tempData, 5, 0);
			*/

			tempPulse = (int)(Joint[RF1].RefAngleCurrent*Joint[RF1].PPR);
			//printf("\n %d %f %f", tempPulse,Joint[RF1].RefAngleCurrent,Joint[RF1].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			short_temp = (short)(Joint[RF2].RefVelCurrent);	
			short_currentPulse = SignConventionFinger(short_temp,finger_control_mode);
			tempData[3] = (unsigned char)(short_currentPulse & 0x000000FF);

			short_temp = (short)(Joint[RF3].RefVelCurrent);
			short_currentPulse = SignConventionFinger(short_temp,finger_control_mode);
			tempData[4] = (unsigned char)(short_currentPulse & 0x000000FF);
			PushCANMsg(Joint[RF1].CAN_channel, Joint[RF1].Ref_txdf, tempData, 5, 0);
			break;

		case EJMC5:
			/*//------------------------------------------------ original
			short_temp = (short)(Joint[LF1].RefVelCurrent);
			HAND_err_short_float1[0] += (Joint[LF1].RefVelCurrent - short_temp);
			if(HAND_err_short_float1[0] >= 1.0f) 
			{
				if(HAND_err_short_float1[0] > 0.)
				{
					short_temp += 1;
					HAND_err_short_float1[0] -= 1.0f;
				}
				else
				{
					short_temp -= 1;
					HAND_err_short_float1[0] += 1.0f;
				}
			}	
			short_currentPulse = SignConventionFinger(short_temp,finger_control_mode);
			tempData[0] = (unsigned char)(short_currentPulse & 0x000000FF);
			//================
			short_temp = (short)(Joint[LF2].RefVelCurrent);
			HAND_err_short_float1[1] += (Joint[LF2].RefVelCurrent - short_temp);
			if(HAND_err_short_float1[1] >= 1.0f) 
			{
				if(HAND_err_short_float1[1] > 0.)
				{
					short_temp += 1;
					HAND_err_short_float1[1] -= 1.0f;
				}
				else
				{
					short_temp -= 1;
					HAND_err_short_float1[1] += 1.0f;
				}
			}	
			short_currentPulse = SignConventionFinger(short_temp,finger_control_mode);
			tempData[1] = (unsigned char)(short_currentPulse & 0x000000FF);
			//================
			short_temp = (short)(Joint[LF3].RefVelCurrent);
			HAND_err_short_float1[2] += (Joint[LF3].RefVelCurrent - short_temp);
			if(HAND_err_short_float1[2] >= 1.0f) 
			{
				if(HAND_err_short_float1[2] > 0.)
				{
					short_temp += 1;
					HAND_err_short_float1[2] -= 1.0f;
				}
				else
				{
					short_temp -= 1;
					HAND_err_short_float1[2] += 1.0f;
				}
			}	
			short_currentPulse = SignConventionFinger(short_temp,finger_control_mode);
			tempData[2] = (unsigned char)(short_currentPulse & 0x000000FF);
			//================
			short_temp = (short)(Joint[LF4].RefVelCurrent);
			HAND_err_short_float1[3] += (Joint[LF4].RefVelCurrent - short_temp);
			if(HAND_err_short_float1[3] >= 1.0f) 
			{
				if(HAND_err_short_float1[3] > 0.)
				{
					short_temp += 1;
					HAND_err_short_float1[3] -= 1.0f;
				}
				else
				{
					short_temp -= 1;
					HAND_err_short_float1[3] += 1.0f;
				}
			}	
			short_currentPulse = SignConventionFinger(short_temp,finger_control_mode);
			tempData[3] = (unsigned char)(short_currentPulse & 0x000000FF);
			//================
			short_temp = (short)(Joint[LF5].RefVelCurrent);
			HAND_err_short_float1[4] += (Joint[LF5].RefVelCurrent - short_temp);
			if(HAND_err_short_float1[4] >= 1.0f) 
			{
				if(HAND_err_short_float1[4] > 0.)
				{
					short_temp += 1;
					HAND_err_short_float1[4] -= 1.0f;
				}
				else
				{
					short_temp -= 1;
					HAND_err_short_float1[4] += 1.0f;
				}
			}	
			short_currentPulse = SignConventionFinger(short_temp,finger_control_mode);
			tempData[4] = (unsigned char)(short_currentPulse & 0x000000FF);
			/*
			if(MotionTimeCurrent>=1 && MotionTimeCurrent < 5)
				tempData[1] = 0x81;//(unsigned char)(short_currentPulse & 0x000000FF);
			else if(MotionTimeCurrent >=101 && MotionTimeCurrent <105)
				tempData[1] = 0x7F;
			else
			*/
			//------------------------------------------------ original
			tempPulse = (int)(Joint[LF1].RefAngleCurrent*Joint[LF1].PPR);
			currentPulse = SignConvention(tempPulse);
			tempData[0] = (unsigned char)(currentPulse & 0x000000FF);
			tempData[1] = (unsigned char)( (currentPulse>>8) & 0x000000FF );
			tempData[2] = (unsigned char)( (currentPulse>>16) & 0x000000FF );

			short_temp = (short)(Joint[LF2].RefVelCurrent);
			short_currentPulse = SignConventionFinger(short_temp,finger_control_mode);
			tempData[3] = (unsigned char)(short_currentPulse & 0x000000FF);

		

			short_temp = (short)(Joint[LF3].RefVelCurrent);
			short_currentPulse = SignConventionFinger(short_temp,finger_control_mode);
			tempData[4] = (unsigned char)(short_currentPulse & 0x000000FF);

			PushCANMsg(Joint[LF1].CAN_channel, Joint[LF1].Ref_txdf, tempData, 5, 0);
			break;
		}
}
/******************************************************************************/



/******************************************************************************/
unsigned long SignConvention(long _input)
{
	if (_input < 0) return (unsigned long)( ((-_input)&0x007FFFFF) | (1<<23) );
	else return (unsigned long)_input;
			
}
/******************************************************************************/




/******************************************************************************/
unsigned short SignConventionFinger(short _input,unsigned char _type)
{
	if(_type == 0x00) // Position
	{
		if (_input < 0) return ((_input)&0x000000FF);
		else return (unsigned short)_input;
	}
	else if(_type == 0x01) // Current
	{
		if (_input < 0) return (unsigned short)( ((-_input)&0x0000007F) | (1<<7) );
		else return (unsigned short)_input;
	}
	else return 0x00;
}
/******************************************************************************/




/******************************************************************************/
void InverseKinematics(float _pos[6], float _angle[])
{
	unsigned char i;
	float angle[6];

	// knee
	angle[3] = PI-acosf((LENGTH_THIGH*LENGTH_THIGH+LENGTH_CALF*LENGTH_CALF-(_pos[0]*_pos[0]+_pos[1]*_pos[1]+_pos[2]*_pos[2]))/(2.0f*LENGTH_THIGH*LENGTH_CALF));
	// hip roll
	angle[1] = atan2f(_pos[1], -_pos[2]);
	// hip pitch
	angle[2] = asinf((-sinf(angle[3])*LENGTH_CALF*(-_pos[1]*sinf(angle[1])+_pos[2]*cosf(angle[1]))+_pos[0]*(cosf(angle[3])*LENGTH_CALF+LENGTH_THIGH))/(-_pos[0]*_pos[0]-(_pos[1]*sinf(angle[1])-_pos[2]*cosf(angle[1]))*(_pos[1]*sinf(angle[1])-_pos[2]*cosf(angle[1]))));
	// hip yaw
	angle[0] = DEG2RAD*_pos[5];
	// ankle roll
	angle[5] = -angle[1];
	// ankle pitch
	angle[4] = -angle[2]-angle[3];

	for(i=0 ; i<6 ; i++) _angle[i] = RAD2DEG*angle[i];
}
/******************************************************************************/





/******************************************************************************/
void ForwardKinematics(float _angle[6], float _pos[])
{
	
}
/******************************************************************************/





/******************************************************************************/
void ReadEncoder(unsigned char _canChannel)
{
	unsigned char tempData[8];

	if(_canChannel == CAN0)
	{
		/*ReadMBData(ENC0_RXDF, tempData);
		Joint[RHY].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[RHR].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		ReadMBData(ENC1_RXDF, tempData);
		Joint[RHP].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		ReadMBData(ENC2_RXDF, tempData);
		Joint[RKN].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		ReadMBData(ENC3_RXDF, tempData);
		Joint[RAP].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[RAR].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);

		ReadMBData(ENC4_RXDF, tempData);
		Joint[LHY].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[LHR].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		ReadMBData(ENC5_RXDF, tempData);
		Joint[LHP].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		ReadMBData(ENC6_RXDF, tempData);
		Joint[LKN].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		ReadMBData(ENC7_RXDF, tempData);
		Joint[LAP].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[LAR].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);*/


		if(ERR_OK==ReadMBData(ENC3_RXDF, tempData))
		{
			Joint[RAP].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
			Joint[RAR].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		}

		if(ERR_OK==ReadMBData(ENC7_RXDF, tempData))
		{
			Joint[LAP].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
			Joint[LAR].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		}
	}
	else if(_canChannel == CAN1)
	{
		/*
		ReadMBData(ENC8_RXDF, tempData);
		Joint[RSP].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[RSR].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		ReadMBData(ENC9_RXDF, tempData);
		Joint[RSY].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[REB].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);

		ReadMBData(ENC10_RXDF, tempData);
		Joint[LSP].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[LSR].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		ReadMBData(ENC11_RXDF, tempData);
		Joint[LSY].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[LEB].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);

		ReadMBData(ENC_E0_RXDF, tempData);
		Joint[RWY].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[RWP].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);

		ReadMBData(ENC_E1_RXDF, tempData);
		Joint[LWY].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[LWP].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
	
		ReadMBData(ENC_E2_RXDF, tempData);
		Joint[NKY].EncoderValue = (int)((short)((tempData[0])|(tempData[1]<<8)));
		Joint[NK1].EncoderValue = (int)((short)((tempData[2])|(tempData[3]<<8)));
		Joint[NK2].EncoderValue = (int)((short)((tempData[4])|(tempData[5]<<8)));

		ReadMBData(ENC_E3_RXDF, tempData);
		Joint[WST].EncoderValue = (int)((short)((tempData[0])|(tempData[1]<<8)));

		ReadMBData(ENC_E4_RXDF, tempData);
		Joint[RF1].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[RF2].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);

		ReadMBData(ENC_E5_RXDF, tempData);
		Joint[LF1].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
		Joint[LF2].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		*/
		if(ERR_OK==ReadMBData(ENC8_RXDF, tempData))
		{
			Joint[RSP].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
			Joint[RSR].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		}
		
		if(ERR_OK==ReadMBData(ENC9_RXDF, tempData))
		{
			Joint[RSY].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
			Joint[REB].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		}

		if(ERR_OK==ReadMBData(ENC10_RXDF, tempData))
		{
			Joint[LSP].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
			Joint[LSR].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		}
		
		if(ERR_OK==ReadMBData(ENC11_RXDF, tempData))
		{
			Joint[LSY].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
			Joint[LEB].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		}

		if(ERR_OK==ReadMBData(ENC_E0_RXDF, tempData))
		{
			Joint[RWY].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
			Joint[RWP].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		}
		
		if(ERR_OK==ReadMBData(ENC_E1_RXDF, tempData))
		{
			Joint[LWY].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
			Joint[LWP].EncoderValue = (int)(tempData[4])|(tempData[5]<<8)|(tempData[6]<<16)|(tempData[7]<<24);
		}

		if(ERR_OK==ReadMBData(ENC_E2_RXDF, tempData))
		{
			Joint[NKY].EncoderValue = (int)((short)((tempData[0])|(tempData[1]<<8)));
			Joint[NK1].EncoderValue = (int)((short)((tempData[2])|(tempData[3]<<8)));
			Joint[NK2].EncoderValue = (int)((short)((tempData[4])|(tempData[5]<<8)));
		}
		
		if(ERR_OK==ReadMBData(ENC_E3_RXDF, tempData))
			Joint[WST].EncoderValue = (int)((short)((tempData[0])|(tempData[1]<<8)));
		
		if(ERR_OK==ReadMBData(ENC_E4_RXDF, tempData))
		{
			Joint[RWY2].EncoderValue = (int)((short)((tempData[0])|(tempData[1]<<8)));
			//Joint[RF2].EncoderValue = (int)((short)((tempData[2])|(tempData[3]<<8)));
			//Joint[RF3].EncoderValue = (int)((short)((tempData[4])|(tempData[5]<<8)));
		}

		if(ERR_OK==ReadMBData(ENC_E5_RXDF, tempData))
		{
			//Joint[LWY2].EncoderValue = (int)(tempData[0])|(tempData[1]<<8)|(tempData[2]<<16)|(tempData[3]<<24);
			Joint[LWY2].EncoderValue = (int)((short)((tempData[0])|(tempData[1]<<8)));
		}


	}
	
}
/******************************************************************************/




/******************************************************************************/
void ReadFT(unsigned char _canChannel)
{
	unsigned char tempData[8];
	unsigned int buff_no;

	if(_canChannel == CAN0)	// for ankle F/T sensor
	{
		buff_no = GetBuffno(SENSOR_FT0_RXDF);
		if(MB[buff_no].status != NODATA)
		{
			ReadMBData(SENSOR_FT0_RXDF, tempData);
			//FTSensor[RFFT].Mx = -(float)((short)((tempData[1]<<8)|tempData[0]))/100.0f;	 // test platform
			FTSensor[RFFT].Mx = (float)((short)((tempData[1]<<8)|tempData[0]))/100.0f;
			FTSensor[RFFT].My = (float)((short)((tempData[3]<<8)|tempData[2]))/100.0f; 
			FTSensor[RFFT].Fz = (float)((short)((tempData[5]<<8)|tempData[4]))/10.0f; 
			FTSensor[RFFT].Filtered_Mx = (1.f-2.f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME*0.001f)*FTSensor[RFFT].Filtered_Mx + 2.f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME*0.001f*FTSensor[RFFT].Mx;
			FTSensor[RFFT].Filtered_My = (1.f-2.f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME*0.001f)*FTSensor[RFFT].Filtered_My + 2.f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME*0.001f*FTSensor[RFFT].My;
			FTSensor[RFFT].Filtered_Fz = (1.f-2.f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME*0.001f)*FTSensor[RFFT].Filtered_Fz + 2.f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME*0.001f*FTSensor[RFFT].Fz;
		}

		buff_no = GetBuffno(SENSOR_FT1_RXDF);
		if(MB[buff_no].status != NODATA)
		{
			ReadMBData(SENSOR_FT1_RXDF, tempData);
			//FTSensor[LFFT].Mx = -(float)((short)((tempData[1]<<8)|tempData[0]))/100.0f; // test platform
			FTSensor[LFFT].Mx = (float)((short)((tempData[1]<<8)|tempData[0]))/100.0f;
			FTSensor[LFFT].My = (float)((short)((tempData[3]<<8)|tempData[2]))/100.0f; 
			FTSensor[LFFT].Fz = (float)((short)((tempData[5]<<8)|tempData[4]))/10.0f;
			FTSensor[LFFT].Filtered_Mx = (1.f-2.f*PI*FTSensor[LFFT].CutOffFeq*INT_TIME*0.001f)*FTSensor[LFFT].Filtered_Mx + 2.f*PI*FTSensor[LFFT].CutOffFeq*INT_TIME*0.001f*FTSensor[LFFT].Mx;
			FTSensor[LFFT].Filtered_My = (1.f-2.f*PI*FTSensor[LFFT].CutOffFeq*INT_TIME*0.001f)*FTSensor[LFFT].Filtered_My + 2.f*PI*FTSensor[LFFT].CutOffFeq*INT_TIME*0.001f*FTSensor[LFFT].My;
			FTSensor[LFFT].Filtered_Fz = (1.f-2.f*PI*FTSensor[LFFT].CutOffFeq*INT_TIME*0.001f)*FTSensor[LFFT].Filtered_Fz + 2.f*PI*FTSensor[LFFT].CutOffFeq*INT_TIME*0.001f*FTSensor[LFFT].Fz;
		}

		buff_no = GetBuffno(SENSOR_AD0_RXDF);
		if(MB[buff_no].status != NODATA)
		{	
			ReadMBData(SENSOR_AD0_RXDF, tempData);
			FTSensor[RFFT].dAccRoll =  ((tempData[0] | tempData[1] << 8) >> 0);// - FTSensor[RFFT].dAccRoll_Offset;
			FTSensor[RFFT].dAccPitch = (((tempData[2] | tempData[3] << 8) >> 0));// - FTSensor[RFFT].dAccPitch_Offset;
			FTSensor[RFFT].AccRoll = (float)((1.0f - 2.0f*PI*FTSensor[RFFT].CutOffFeq*(float)INT_TIME/1000.0f)*FTSensor[RFFT].AccRollOld + 2.0f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME/1000.0f*(float)FTSensor[RFFT].dAccRoll);
			FTSensor[RFFT].AccPitch = (float)((1.0f - 2.0f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME/1000.0f)*FTSensor[RFFT].AccPitchOld + 2.0f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME/1000.0f*FTSensor[RFFT].dAccPitch);
			FTSensor[RFFT].AccRollOld = FTSensor[RFFT].AccRoll;
			FTSensor[RFFT].AccPitchOld = FTSensor[RFFT].AccPitch;
			FTSensor[RFFT].Pitch = FTSensor[RFFT].AccPitch/FTSensor[RFFT].SF_Pitch;
			FTSensor[RFFT].Roll = FTSensor[RFFT].AccRoll/FTSensor[RFFT].SF_Roll;
			FTSensor[RFFT].VelRoll = (float)((1.0f - 2.0f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME/1000.0f)*FTSensor[RFFT].VelRollOld + INT_TIME/1000.0f*FTSensor[RFFT].AccRollOld);
			FTSensor[RFFT].VelRollOld = FTSensor[RFFT].VelRoll;
			FTSensor[RFFT].VelPitch = (float)((1.0f - 2.0f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME/1000.0f)*FTSensor[RFFT].VelPitchOld + INT_TIME/1000.0f*FTSensor[RFFT].AccPitchOld);
			FTSensor[RFFT].VelPitchOld = FTSensor[RFFT].VelPitch;
		}

		buff_no = GetBuffno(SENSOR_AD1_RXDF);
		if(MB[buff_no].status != NODATA)
		{
			ReadMBData(SENSOR_AD1_RXDF, tempData);
			FTSensor[LFFT].dAccRoll =  ((tempData[0] | tempData[1] << 8) >> 0);// - FTSensor[LFFT].dAccRoll_Offset;
			FTSensor[LFFT].dAccPitch = (((tempData[2] | tempData[3] << 8) >> 0));// - FTSensor[LFFT].dAccPitch_Offset;
			FTSensor[LFFT].AccRoll = (float)((1.0f - 2.0f*PI*FTSensor[LFFT].CutOffFeq*INT_TIME/1000.0f)*FTSensor[LFFT].AccRollOld + 2.0f*PI*FTSensor[LFFT].CutOffFeq*INT_TIME/1000.0f*FTSensor[LFFT].dAccRoll);
			FTSensor[LFFT].AccPitch = (float)((1.0f - 2.0f*PI*FTSensor[LFFT].CutOffFeq*INT_TIME/1000.0f)*FTSensor[LFFT].AccPitchOld + 2.0f*PI*FTSensor[LFFT].CutOffFeq*INT_TIME/1000.0f*FTSensor[LFFT].dAccPitch);
			FTSensor[LFFT].AccRollOld = FTSensor[LFFT].AccRoll;
			FTSensor[LFFT].AccPitchOld = FTSensor[LFFT].AccPitch;
			FTSensor[LFFT].Pitch = FTSensor[LFFT].AccPitch/FTSensor[LFFT].SF_Pitch;
			FTSensor[LFFT].Roll = FTSensor[LFFT].AccRoll/FTSensor[RFFT].SF_Roll;
			FTSensor[LFFT].VelRoll = (float)((1.0f - 2.0f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME/1000.0f)*FTSensor[LFFT].VelRollOld + INT_TIME/1000.0f*FTSensor[LFFT].AccRollOld);
			FTSensor[LFFT].VelRollOld = FTSensor[LFFT].VelRoll;
			FTSensor[LFFT].VelPitch = (float)((1.0f - 2.0f*PI*FTSensor[RFFT].CutOffFeq*INT_TIME/1000.0f)*FTSensor[LFFT].VelPitchOld + INT_TIME/1000.0f*FTSensor[LFFT].AccPitchOld);
			FTSensor[LFFT].VelPitchOld = FTSensor[LFFT].VelPitch;
		}
		
/*		ReadMBData(SENSOR_AD1_RXDF, tempData);
		FTSensor[LFFT].VelRoll =  (float)((int)(tempData[0]<<16|(tempData[1]<<24))>>16);
		FTSensor[LFFT].VelPitch = (float)((int)(tempData[2]<<16|(tempData[3]<<24))>>16);
*/
	}
	else if(_canChannel == CAN1)	// for wrist F/T sensor
	{
		buff_no = GetBuffno(SENSOR_FT2_RXDF);
		if(MB[buff_no].status != NODATA)
		{
			if(pSharedMemory->change_drc_hand_flag == DRC_STICK_MODE)
			{
				ReadMBData(SENSOR_FT2_RXDF, tempData);
				FTSensor[RWFT].My = (float)((short)((tempData[1]<<8)|tempData[0]))/100.0f; 
				FTSensor[RWFT].Mx = -(float)((short)((tempData[3]<<8)|tempData[2]))/100.0f; 
				FTSensor[RWFT].Fz = (float)((short)((tempData[5]<<8)|tempData[4]))/10.0f;
			}
			else
			{
				ReadMBData(SENSOR_FT2_RXDF, tempData);
				FTSensor[RWFT].My = (float)((short)((tempData[1]<<8)|tempData[0]))/100.0f; 
				FTSensor[RWFT].Mx = (float)((short)((tempData[3]<<8)|tempData[2]))/100.0f; 
				FTSensor[RWFT].Fz = (float)((short)((tempData[5]<<8)|tempData[4]))/10.0f;
			}
		}
/*		FTSensor[RWFT].dMx =  (short)((tempData[0])|(tempData[1]<<8));
		FTSensor[RWFT].dMy =  (short)((tempData[2])|(tempData[3]<<8));
		FTSensor[RWFT].dFz =  (short)((tempData[4])|(tempData[5]<<8));
		FTSensor[RWFT].Mx = FTSensor[RWFT].SF_Mx*(float)FTSensor[RWFT].dMx;
		FTSensor[RWFT].My = FTSensor[RWFT].SF_My*(float)FTSensor[RWFT].dMy;
		FTSensor[RWFT].Fz = FTSensor[RWFT].SF_Fz*(float)FTSensor[RWFT].dFz;
		FTSensor[RWFT].Filtered_Mx = FTSensor[RWFT].CutOffMx*FTSensor[RWFT].Filtered_Mx + (1.0f-FTSensor[RWFT].CutOffMx)*FTSensor[RWFT].Mx;
		FTSensor[RWFT].Filtered_My = FTSensor[RWFT].CutOffMy*FTSensor[RWFT].Filtered_My + (1.0f-FTSensor[RWFT].CutOffMy)*FTSensor[RWFT].My;
		FTSensor[RWFT].Filtered_Fz = FTSensor[RWFT].CutOffFz*FTSensor[RWFT].Filtered_Fz + (1.0f-FTSensor[RWFT].CutOffFz)*FTSensor[RWFT].Fz;

 */
		buff_no = GetBuffno(SENSOR_FT3_RXDF);
		if(MB[buff_no].status != NODATA)
		{
			if(pSharedMemory->change_drc_hand_flag == DRC_STICK_MODE)
			{
				ReadMBData(SENSOR_FT3_RXDF, tempData);
				FTSensor[LWFT].My = -(float)((short)((tempData[1]<<8)|tempData[0]))/100.0f; 
				FTSensor[LWFT].Mx = (float)((short)((tempData[3]<<8)|tempData[2]))/100.0f; 
				FTSensor[LWFT].Fz = -(float)((short)((tempData[5]<<8)|tempData[4]))/10.0f;
			}
			else
			{
				ReadMBData(SENSOR_FT3_RXDF, tempData);
				FTSensor[LWFT].My = -(float)((short)((tempData[1]<<8)|tempData[0]))/100.0f; 
				FTSensor[LWFT].Mx = -(float)((short)((tempData[3]<<8)|tempData[2]))/100.0f; 
				FTSensor[LWFT].Fz = (float)((short)((tempData[5]<<8)|tempData[4]))/10.0f;
			}

		}
	}
	else;
			

	// only for debugging
	pSharedMemory->FT_debug[RFFT] = FTSensor[RFFT];
	pSharedMemory->FT_debug[LFFT] = FTSensor[LFFT];
	pSharedMemory->FT_debug[RWFT] = FTSensor[RWFT];
	pSharedMemory->FT_debug[LWFT] = FTSensor[LWFT];
}
/******************************************************************************/




/******************************************************************************/
unsigned char GetZMP(FT _rightFootFTSensor, FT _leftFootFTSensor, WALKING_INFO _walkingInfo[2][4], float _zmp[], float _filteredZMP[])
{
	// _zmp[0]: x-direction ZMP based on whole legs
	// _zmp[1]: y-direction ZMP based on whole legs
	// _zmp[2]: x-direction ZMP based on right leg
	// _zmp[3]: y-direction ZMP based on right leg
	// _zmp[3]: x-direction ZMP based on left leg
	// _zmp[4]: y-direction ZMP based on left leg
	
	unsigned char _state, i;
	float totalMx, totalMy;
	float filteredZMPOld[6];

	if(_rightFootFTSensor.Fz > 50.f)
	{			
		if(_leftFootFTSensor.Fz> 50.f)		// Double Support Phase
		{
			totalMx = _rightFootFTSensor.Mx+_leftFootFTSensor.Mx+(_walkingInfo[LEFT][Y].RefPatternCurrent+0.5f*PELVIS_WIDTH)*_leftFootFTSensor.Fz+(_walkingInfo[RIGHT][Y].RefPatternCurrent-0.5f*PELVIS_WIDTH)*_rightFootFTSensor.Fz;
			totalMy = _rightFootFTSensor.My+_leftFootFTSensor.My-_walkingInfo[LEFT][X].RefPatternCurrent*_leftFootFTSensor.Fz-_walkingInfo[RIGHT][X].RefPatternCurrent*_rightFootFTSensor.Fz;
			_zmp[0] = -1000.f*totalMy/(_leftFootFTSensor.Fz+_rightFootFTSensor.Fz);
			_zmp[1] = 1000.f*totalMx/(_leftFootFTSensor.Fz+_rightFootFTSensor.Fz);
			_zmp[2] = -1000.0f*_rightFootFTSensor.My/_rightFootFTSensor.Fz;
			_zmp[3] = 1000.0f*_rightFootFTSensor.Mx/_rightFootFTSensor.Fz;
			_zmp[4] = -1000.0f*_leftFootFTSensor.My/_leftFootFTSensor.Fz;
			_zmp[5] = 1000.0f*_leftFootFTSensor.Mx/_leftFootFTSensor.Fz;

			_state = DSP;
		}
		else	// Right leg Single Support Phase
		{
			_zmp[2] = -1000.0f*_rightFootFTSensor.My/_rightFootFTSensor.Fz;
			_zmp[3] = 1000.0f*_rightFootFTSensor.Mx/_rightFootFTSensor.Fz;
			_zmp[0] = _walkingInfo[RIGHT][X].RefPatternCurrent*1000.f + _zmp[2];
			_zmp[1] = (_walkingInfo[RIGHT][Y].RefPatternCurrent-0.5f*PELVIS_WIDTH)*1000.f + _zmp[3];
			_zmp[4] = _zmp[5] = 0.0f;

			_state = RIGHT_SSP;
		}			
	}
	else if(_leftFootFTSensor.Fz> 50.f)	// Left leg Single Support Phase
	{
		_zmp[4] = -1000.0f*_leftFootFTSensor.My/_leftFootFTSensor.Fz;
		_zmp[5] = 1000.0f*_leftFootFTSensor.Mx/_leftFootFTSensor.Fz;
		_zmp[0]  = _walkingInfo[LEFT][X].RefPatternCurrent*1000.0f + _zmp[4];
		_zmp[1]  = (_walkingInfo[LEFT][Y].RefPatternCurrent+0.5f*PELVIS_WIDTH)*1000.0f + _zmp[5];	
		_zmp[2] = _zmp[3] = 0.0f;

		_state = LEFT_SSP;
	}
	else { _zmp[0] = _zmp[1] = _zmp[2] = _zmp[3] = _zmp[4] = _zmp[5] = 0.0f; _state = NO_LANDING; }


	for(i=0 ; i<6 ; i++)
	{
		filteredZMPOld[i] = _filteredZMP[i];
		_filteredZMP[i] = (1.0f-2.0f*PI*5.0f*(float)INT_TIME/1000.0f)*filteredZMPOld[i] + (2.0f*PI*2.0f*(float)INT_TIME/1000.0f)*_zmp[i];
	}
		

	// only for debugging
	for(i=2 ; i<6 ; i++) pSharedMemory->ZMP[i] = _filteredZMP[i];


	return _state;
}
/******************************************************************************/





/******************************************************************************/

void ReadIMU(void)
{
	unsigned char tempData[8];
	int buff_no;

	buff_no = GetBuffno(SENSOR_IMU0_RXDF);
	if(MB[buff_no].status != NODATA)
	{
		ReadMBData(SENSOR_IMU0_RXDF, tempData);
		IMUSensor[CENTERIMU].Roll = (float)((short)((tempData[1]<<8)|tempData[0]))/100.0f + IMUSensor[CENTERIMU].RollOffset; 
		IMUSensor[CENTERIMU].Pitch = (float)((short)((tempData[3]<<8)|tempData[2]))/100.0f + IMUSensor[CENTERIMU].PitchOffset; 
		IMUSensor[CENTERIMU].Roll_Velocity = (float)((short)((tempData[5]<<8)|tempData[4]))/100.0f; 
		IMUSensor[CENTERIMU].Pitch_Velocity = (float)((short)((tempData[7]<<8)|tempData[6]))/100.0f; 
		IMUSensor[CENTERIMU].Yaw_angle = 0.f;
		IMUSensor[CENTERIMU].Yaw_Velocity = 0.f;

	}
	// only for debugging
	unsigned char i;
	for(i=CENTERIMU ; i<NO_OF_IMU; i++) pSharedMemory->IMU_debug[i] = IMUSensor[i];
}


/*
void ReadIMU(void)
{
	unsigned char tempData[8];
	unsigned char tempData2[8];
	int buff_no;
	int buff_no2;
	unsigned char imu_ok_flag;
	unsigned char i;

	imu_ok_flag = 0;

	buff_no = GetBuffno(SENSOR_IMU0_RXDF);
	if(MB[buff_no].status != NODATA)
	{
		ReadMBData(SENSOR_IMU0_RXDF, tempData);
		rad_x_hubo = (float)((short)((tempData[1]<<8)|tempData[0]))/100.0f*changetoR; 
		rad_y_hubo = (float)((short)((tempData[3]<<8)|tempData[2]))/100.0f*changetoR;
		rad_z_hubo = (float)((short)((tempData[5]<<8)|tempData[4]))/100.0f*changetoR; 
		Kalmanindex1 = (float)((short)((tempData[7]<<8)|tempData[6]))/100.0f; 

		imu_ok_flag = 1;
	
	}

	buff_no2 = GetBuffno(0x54);
	if(MB[buff_no2].status != NODATA)
	{
		ReadMBData(0x54, tempData2);
		acc_g_x_hubo = (float)((short)((tempData2[1]<<8)|tempData2[0]))/1000.0f; 
		acc_g_y_hubo = (float)((short)((tempData2[3]<<8)|tempData2[2]))/1000.0f;
		acc_g_z_hubo = (float)((short)((tempData2[5]<<8)|tempData2[4]))/1000.0f; 
		Kalmanindex2 = (float)((short)((tempData2[7]<<8)|tempData2[6]))/1000.0f; 

		if(imu_ok_flag == 1)
			imu_ok_flag = 2;
	}

	if(imu_ok_flag == 2)
	{
		temp_hyo_xacc=xaccCaliDRC(acc_g_x_hubo);
		temp_hyo_yacc=yaccCaliDRC(acc_g_y_hubo);
		temp_hyo_zacc=zaccCaliDRC(acc_g_y_hubo, acc_g_z_hubo);
	}
	else
		return;

	

	//--------------------------------------------------------------------------------------------------------------------------------
	


	//--------------------------------------------------------------------------------------------------------------------------------
	if(kalmanflag==0)
	{
		Qk[1][1]=0.001f;	Qk[1][2]=0.f;	Qk[1][3]=0.f;	Qk[1][3]=0.f;	Qk[2][1]=0.f;	Qk[2][2]=0.001f;	Qk[2][3]=0.f;	Qk[2][4]=0.f;
		Qk[3][1]=0.f;		Qk[3][2]=0.f;	Qk[3][3]=0.001f;	Qk[3][4]=0.f;	Qk[4][1]=0.f;	Qk[4][2]=0.f;	Qk[4][3]=0.f;	Qk[4][4]=0.001f;
		
		KalmanR=250.f;
		x[1][1]=1.f;	x[2][1]=0.f;	x[3][1]=0.f;	x[4][1]=0.f;
		
		P[1][1]=1.f;	P[1][2]=0.f;	P[1][3]=0.f;	P[1][4]=0.f;	P[2][1]=0.f;	P[2][2]=1.f;	P[2][3]=0.f;	P[2][4]=0.f;
		P[3][1]=0.f;	P[3][2]=0.f;	P[3][3]=1.f;	P[3][4]=0.f;	P[4][1]=0.f;	P[4][2]=0.f;	P[4][3]=0.f;	P[4][4]=1.f;
		q0=1.f;	q1=0.f;	q2=0.f;	q3=0.f;
		
		g_norm=(float)sqrt(temp_hyo_xacc*temp_hyo_xacc+temp_hyo_yacc*temp_hyo_yacc+temp_hyo_zacc*temp_hyo_zacc);
		jerk_g=0;
		g_norm_old=g_norm;
		temp_hyo_xacc=temp_hyo_xacc/g_norm;
		temp_hyo_yacc=temp_hyo_yacc/g_norm;
		temp_hyo_zacc=temp_hyo_zacc/g_norm;
		temp_hyo_xacc_old=temp_hyo_xacc;
		temp_hyo_yacc_old=temp_hyo_yacc;
		temp_hyo_zacc_old=temp_hyo_zacc;

		//for basic
		gyr_dps_x_hubo[0] = 0.;
		gyr_dps_y_hubo[0] = 0.;
		gyr_dps_z_hubo[0] = 0.;
		gyr_dps_x_hubo[1] = 0.;
		gyr_dps_y_hubo[1] = 0.;
		gyr_dps_z_hubo[1] = 0.;
		myX=1.;

		ang_x_hubo= 0.;
		ang_y_hubo= 0.;
		ang_z_hubo= 0.;
		ang_x_hubo_rad= 0.;
		ang_y_hubo_rad= 0.;
		ang_z_hubo_rad= 0.;

		temp_x_gyro= 0.;
		temp_y_gyro= 0.;
		temp_z_gyro= 0.;
		//for FES
		kalman_roll= 0.;
		kalman_pitch= 0.;
		kalman_yaw= 0.;
		acc_roll= 0.;
		acc_pitch= 0.;
		acc_roll_LPF= 0.;
		acc_pitch_LPF= 0.;

		gyro_sum_roll=0.;
		gyro_sum_pitch=0.;

		offset_roll=0.;
		offset_pitch=0.;
		ang_x_hubo_old=0.;
		ang_y_hubo_old=0.;
		ang_z_hubo_old=0.;

		dynamicstate=2;
		accstate=0;
		gyrostate=0;
		gyrostate2=0;
		jerkstate=0;
		w_norm=0.;
		

		acc_ang_x_hubo_LPFed=0.;
		acc_ang_y_hubo_LPFed=0.;
		gyr_ang_x_hubo_HPFed=0.;
		gyr_ang_y_hubo_HPFed=0.;
		gyr_ang_z_hubo_HPFed=0.;
		complementary_roll=0.;
		complementary_pitch=0.;
		complementary_yaw=0.;

		kalmanflag=1;
		
	}
	else
	{
					g_norm_old=g_norm;

					g_norm=(float)sqrt(temp_hyo_xacc*temp_hyo_xacc+temp_hyo_yacc*temp_hyo_yacc+temp_hyo_zacc*temp_hyo_zacc);

					temp_x_gyro = rad_x_hubo*changetoD;
					temp_y_gyro = rad_y_hubo*changetoD;
					temp_z_gyro = rad_z_hubo*changetoD;

					jerk_g=(g_norm-g_norm_old)*10.;
					if((jerk_g>0.033) || (jerk_g<-0.033))
						jerkstate=1;
					else
						jerkstate=0;

					if((g_norm>1.02) || (g_norm<0.98))
						accstate=1;
					else
						accstate=0;//

					w_norm=sqrt(temp_x_gyro*temp_x_gyro+temp_y_gyro*temp_y_gyro+temp_z_gyro*temp_z_gyro);
					if((w_norm>0.65) || (w_norm<-0.65))
						gyrostate=1;
					else
						gyrostate=0;					
					if((w_norm>0.1) || (w_norm<-0.1))
						gyrostate2=1;
					else
						gyrostate2=0;
					
					//-----
					if((accstate==0) && (gyrostate==0))
					{
						if(jerkstate==1)
							dynamicstate=1;//2
						else
							dynamicstate=1;
					}
					else if((accstate==1) && (gyrostate==0))
					{
						if(jerkstate==1)
							dynamicstate=3;//3
						else
							dynamicstate=2;
					}
					else if((accstate==0) && (gyrostate==1))
					{
						if(jerkstate==1)
							dynamicstate=3;//3
						else
							dynamicstate=2;
					}
					else if((accstate==1) && (gyrostate==1))
						dynamicstate=3;
					else
						dynamicstate=2;



					if(dynamicstate==1)
						KalmanR=180.f;//6-120
					else if(dynamicstate==2)
						KalmanR=240.f;//10-160
					else if(dynamicstate==3)
						KalmanR=250.f;//20-200
					else
						KalmanR=240.f;



					R[1][1]=KalmanR;	R[1][2]=0.f;	R[1][3]=0.f;	R[2][1]=0.f;	R[2][2]=KalmanR;	R[2][3]=0.f;	R[3][1]=0.f;	R[3][2]=0.f;	R[3][3]=KalmanR;
					
					
					if((g_norm>3.5) || (g_norm<0.001))//in pole test 1.24 0.895//normal 1.04 0.96//1.016 0.984
					{
						temp_hyo_xacc=temp_hyo_xacc_old;
						temp_hyo_yacc=temp_hyo_yacc_old;
						temp_hyo_zacc=temp_hyo_zacc_old;
					}
					else
					{
						temp_hyo_xacc=temp_hyo_xacc/g_norm;
						temp_hyo_yacc=temp_hyo_yacc/g_norm;
						temp_hyo_zacc=temp_hyo_zacc/g_norm;

						temp_hyo_xacc_old=temp_hyo_xacc;
						temp_hyo_yacc_old=temp_hyo_yacc;
						temp_hyo_zacc_old=temp_hyo_zacc;
					}
					
		

					A[1][1]=1.f;
					A[1][2]=-0.0025f*rad_x_hubo;
					A[1][3]=-0.0025f*rad_y_hubo;
					A[1][4]=-0.0025f*rad_z_hubo;
					A[2][1]=0.0025f*rad_x_hubo;
					A[2][2]=1.f;
					A[2][3]=0.0025f*rad_z_hubo;
					A[2][4]=-0.0025f*rad_y_hubo;
					A[3][1]=0.0025f*rad_y_hubo;
					A[3][2]=-0.0025f*rad_z_hubo;
					A[3][3]=1.f;
					A[3][4]=0.0025f*rad_x_hubo;
					A[4][1]=0.0025f*rad_z_hubo;
					A[4][2]=0.0025f*rad_y_hubo;
					A[4][3]=-0.0025f*rad_x_hubo;
					A[4][4]=1.f;
					
					z[1][1]=-1.f*temp_hyo_xacc;
					z[2][1]=-1.f*temp_hyo_yacc;
					z[3][1]=-1.f*temp_hyo_zacc;
					
					H[1][1]=2.f*x[3][1];
					H[1][2]=-2.f*x[4][1];
					H[1][3]=2.f*x[1][1];
					H[1][4]=-2.f*x[2][1];
					
					H[2][1]=-2.f*x[2][1];
					H[2][2]=-2.f*x[1][1];
					H[2][3]=-2.f*x[4][1];
					H[2][4]=-2.f*x[3][1];
					
					H[3][1]=-2.f*x[1][1];
					H[3][2]=2.f*x[2][1];
					H[3][3]=2.f*x[3][1];
					H[3][4]=-2.f*x[4][1];
					
					xp[1][1]=A[1][1]*x[1][1]+A[1][2]*x[2][1]+A[1][3]*x[3][1]+A[1][4]*x[4][1];
					xp[2][1]=A[2][1]*x[1][1]+A[2][2]*x[2][1]+A[2][3]*x[3][1]+A[2][4]*x[4][1];
					xp[3][1]=A[3][1]*x[1][1]+A[3][2]*x[2][1]+A[3][3]*x[3][1]+A[3][4]*x[4][1];
					xp[4][1]=A[4][1]*x[1][1]+A[4][2]*x[2][1]+A[4][3]*x[3][1]+A[4][4]*x[4][1];
					
					T[1][1]=P[1][1]*A[1][1]+P[1][2]*A[1][2]+P[1][3]*A[1][3]+P[1][4]*A[1][4];
					T[1][2]=P[1][1]*A[2][1]+P[1][2]*A[2][2]+P[1][3]*A[2][3]+P[1][4]*A[2][4];
					T[1][3]=P[1][1]*A[3][1]+P[1][2]*A[3][2]+P[1][3]*A[3][3]+P[1][4]*A[3][4];
					T[1][4]=P[1][1]*A[4][1]+P[1][2]*A[4][2]+P[1][3]*A[4][3]+P[1][4]*A[4][4];
					T[2][1]=P[2][1]*A[1][1]+P[2][2]*A[1][2]+P[2][3]*A[1][3]+P[2][4]*A[1][4];
					T[2][2]=P[2][1]*A[2][1]+P[2][2]*A[2][2]+P[2][3]*A[2][3]+P[2][4]*A[2][4];
					T[2][3]=P[2][1]*A[3][1]+P[2][2]*A[3][2]+P[2][3]*A[3][3]+P[2][4]*A[3][4];
					T[2][4]=P[2][1]*A[4][1]+P[2][2]*A[4][2]+P[2][3]*A[4][3]+P[2][4]*A[4][4];	
					T[3][1]=P[3][1]*A[1][1]+P[3][2]*A[1][2]+P[3][3]*A[1][3]+P[3][4]*A[1][4];
					T[3][2]=P[3][1]*A[2][1]+P[3][2]*A[2][2]+P[3][3]*A[2][3]+P[3][4]*A[2][4];
					T[3][3]=P[3][1]*A[3][1]+P[3][2]*A[3][2]+P[3][3]*A[3][3]+P[3][4]*A[3][4];
					T[3][4]=P[3][1]*A[4][1]+P[3][2]*A[4][2]+P[3][3]*A[4][3]+P[3][4]*A[4][4];	
					T[4][1]=P[4][1]*A[1][1]+P[4][2]*A[1][2]+P[4][3]*A[1][3]+P[4][4]*A[1][4];
					T[4][2]=P[4][1]*A[2][1]+P[4][2]*A[2][2]+P[4][3]*A[2][3]+P[4][4]*A[2][4];
					T[4][3]=P[4][1]*A[3][1]+P[4][2]*A[3][2]+P[4][3]*A[3][3]+P[4][4]*A[3][4];
					T[4][4]=P[4][1]*A[4][1]+P[4][2]*A[4][2]+P[4][3]*A[4][3]+P[4][4]*A[4][4];	
					
					Pp[1][1]=Qk[1][1]+A[1][1]*T[1][1]+A[1][2]*T[2][1]+A[1][3]*T[3][1]+A[1][4]*T[4][1];
					Pp[1][2]=Qk[1][2]+A[1][1]*T[1][2]+A[1][2]*T[2][2]+A[1][3]*T[3][2]+A[1][4]*T[4][2];
					Pp[1][3]=Qk[1][3]+A[1][1]*T[1][3]+A[1][2]*T[2][3]+A[1][3]*T[3][3]+A[1][4]*T[4][3];
					Pp[1][4]=Qk[1][4]+A[1][1]*T[1][4]+A[1][2]*T[2][4]+A[1][3]*T[3][4]+A[1][4]*T[4][4];		
					Pp[2][1]=Qk[2][1]+A[2][1]*T[1][1]+A[2][2]*T[2][1]+A[2][3]*T[3][1]+A[2][4]*T[4][1];
					Pp[2][2]=Qk[2][2]+A[2][1]*T[1][2]+A[2][2]*T[2][2]+A[2][3]*T[3][2]+A[2][4]*T[4][2];
					Pp[2][3]=Qk[2][3]+A[2][1]*T[1][3]+A[2][2]*T[2][3]+A[2][3]*T[3][3]+A[2][4]*T[4][3];
					Pp[2][4]=Qk[2][4]+A[2][1]*T[1][4]+A[2][2]*T[2][4]+A[2][3]*T[3][4]+A[2][4]*T[4][4];		
					Pp[3][1]=Qk[3][1]+A[3][1]*T[1][1]+A[3][2]*T[2][1]+A[3][3]*T[3][1]+A[3][4]*T[4][1];
					Pp[3][2]=Qk[3][2]+A[3][1]*T[1][2]+A[3][2]*T[2][2]+A[3][3]*T[3][2]+A[3][4]*T[4][2];
					Pp[3][3]=Qk[3][3]+A[3][1]*T[1][3]+A[3][2]*T[2][3]+A[3][3]*T[3][3]+A[3][4]*T[4][3];
					Pp[3][4]=Qk[3][4]+A[3][1]*T[1][4]+A[3][2]*T[2][4]+A[3][3]*T[3][4]+A[3][4]*T[4][4];					
					Pp[4][1]=Qk[4][1]+A[4][1]*T[1][1]+A[4][2]*T[2][1]+A[4][3]*T[3][1]+A[4][4]*T[4][1];
					Pp[4][2]=Qk[4][2]+A[4][1]*T[1][2]+A[4][2]*T[2][2]+A[4][3]*T[3][2]+A[4][4]*T[4][2];
					Pp[4][3]=Qk[4][3]+A[4][1]*T[1][3]+A[4][2]*T[2][3]+A[4][3]*T[3][3]+A[4][4]*T[4][3];
					Pp[4][4]=Qk[4][4]+A[4][1]*T[1][4]+A[4][2]*T[2][4]+A[4][3]*T[3][4]+A[4][4]*T[4][4];

					T[1][1]=R[1][1]+H[1][1]*(Pp[1][1]*H[1][1]+Pp[1][2]*H[1][2]+Pp[1][3]*H[1][3]+Pp[1][4]*H[1][4])+H[1][2]*(Pp[2][1]*H[1][1]+Pp[2][2]*H[1][2]+Pp[2][3]*H[1][3]+Pp[2][4]*H[1][4])+H[1][3]*(Pp[3][1]*H[1][1]+Pp[3][2]*H[1][2]+Pp[3][3]*H[1][3]+Pp[3][4]*H[1][4])+H[1][4]*(Pp[4][1]*H[1][1]+Pp[4][2]*H[1][2]+Pp[4][3]*H[1][3]+Pp[4][4]*H[1][4]);
					T[1][2]=R[1][2]+H[1][1]*(Pp[1][1]*H[2][1]+Pp[1][2]*H[2][2]+Pp[1][3]*H[2][3]+Pp[1][4]*H[2][4])+H[1][2]*(Pp[2][1]*H[2][1]+Pp[2][2]*H[2][2]+Pp[2][3]*H[2][3]+Pp[2][4]*H[2][4])+H[1][3]*(Pp[3][1]*H[2][1]+Pp[3][2]*H[2][2]+Pp[3][3]*H[2][3]+Pp[3][4]*H[2][4])+H[1][4]*(Pp[4][1]*H[2][1]+Pp[4][2]*H[2][2]+Pp[4][3]*H[2][3]+Pp[4][4]*H[2][4]);
					T[1][3]=R[1][3]+H[1][1]*(Pp[1][1]*H[3][1]+Pp[1][2]*H[3][2]+Pp[1][3]*H[3][3]+Pp[1][4]*H[3][4])+H[1][2]*(Pp[2][1]*H[3][1]+Pp[2][2]*H[3][2]+Pp[2][3]*H[3][3]+Pp[2][4]*H[3][4])+H[1][3]*(Pp[3][1]*H[3][1]+Pp[3][2]*H[3][2]+Pp[3][3]*H[3][3]+Pp[3][4]*H[3][4])+H[1][4]*(Pp[4][1]*H[3][1]+Pp[4][2]*H[3][2]+Pp[4][3]*H[3][3]+Pp[4][4]*H[3][4]);
					T[2][1]=R[2][1]+H[2][1]*(Pp[1][1]*H[1][1]+Pp[1][2]*H[1][2]+Pp[1][3]*H[1][3]+Pp[1][4]*H[1][4])+H[2][2]*(Pp[2][1]*H[1][1]+Pp[2][2]*H[1][2]+Pp[2][3]*H[1][3]+Pp[2][4]*H[1][4])+H[2][3]*(Pp[3][1]*H[1][1]+Pp[3][2]*H[1][2]+Pp[3][3]*H[1][3]+Pp[3][4]*H[1][4])+H[2][4]*(Pp[4][1]*H[1][1]+Pp[4][2]*H[1][2]+Pp[4][3]*H[1][3]+Pp[4][4]*H[1][4]);
					T[2][2]=R[2][2]+H[2][1]*(Pp[1][1]*H[2][1]+Pp[1][2]*H[2][2]+Pp[1][3]*H[2][3]+Pp[1][4]*H[2][4])+H[2][2]*(Pp[2][1]*H[2][1]+Pp[2][2]*H[2][2]+Pp[2][3]*H[2][3]+Pp[2][4]*H[2][4])+H[2][3]*(Pp[3][1]*H[2][1]+Pp[3][2]*H[2][2]+Pp[3][3]*H[2][3]+Pp[3][4]*H[2][4])+H[2][4]*(Pp[4][1]*H[2][1]+Pp[4][2]*H[2][2]+Pp[4][3]*H[2][3]+Pp[4][4]*H[2][4]);
					T[2][3]=R[2][3]+H[2][1]*(Pp[1][1]*H[3][1]+Pp[1][2]*H[3][2]+Pp[1][3]*H[3][3]+Pp[1][4]*H[3][4])+H[2][2]*(Pp[2][1]*H[3][1]+Pp[2][2]*H[3][2]+Pp[2][3]*H[3][3]+Pp[2][4]*H[3][4])+H[2][3]*(Pp[3][1]*H[3][1]+Pp[3][2]*H[3][2]+Pp[3][3]*H[3][3]+Pp[3][4]*H[3][4])+H[2][4]*(Pp[4][1]*H[3][1]+Pp[4][2]*H[3][2]+Pp[4][3]*H[3][3]+Pp[4][4]*H[3][4]);
					T[3][1]=R[3][1]+H[3][1]*(Pp[1][1]*H[1][1]+Pp[1][2]*H[1][2]+Pp[1][3]*H[1][3]+Pp[1][4]*H[1][4])+H[3][2]*(Pp[2][1]*H[1][1]+Pp[2][2]*H[1][2]+Pp[2][3]*H[1][3]+Pp[2][4]*H[1][4])+H[3][3]*(Pp[3][1]*H[1][1]+Pp[3][2]*H[1][2]+Pp[3][3]*H[1][3]+Pp[3][4]*H[1][4])+H[3][4]*(Pp[4][1]*H[1][1]+Pp[4][2]*H[1][2]+Pp[4][3]*H[1][3]+Pp[4][4]*H[1][4]);
					T[3][2]=R[3][2]+H[3][1]*(Pp[1][1]*H[2][1]+Pp[1][2]*H[2][2]+Pp[1][3]*H[2][3]+Pp[1][4]*H[2][4])+H[3][2]*(Pp[2][1]*H[2][1]+Pp[2][2]*H[2][2]+Pp[2][3]*H[2][3]+Pp[2][4]*H[2][4])+H[3][3]*(Pp[3][1]*H[2][1]+Pp[3][2]*H[2][2]+Pp[3][3]*H[2][3]+Pp[3][4]*H[2][4])+H[3][4]*(Pp[4][1]*H[2][1]+Pp[4][2]*H[2][2]+Pp[4][3]*H[2][3]+Pp[4][4]*H[2][4]);
					T[3][3]=R[3][3]+H[3][1]*(Pp[1][1]*H[3][1]+Pp[1][2]*H[3][2]+Pp[1][3]*H[3][3]+Pp[1][4]*H[3][4])+H[3][2]*(Pp[2][1]*H[3][1]+Pp[2][2]*H[3][2]+Pp[2][3]*H[3][3]+Pp[2][4]*H[3][4])+H[3][3]*(Pp[3][1]*H[3][1]+Pp[3][2]*H[3][2]+Pp[3][3]*H[3][3]+Pp[3][4]*H[3][4])+H[3][4]*(Pp[4][1]*H[3][1]+Pp[4][2]*H[3][2]+Pp[4][3]*H[3][3]+Pp[4][4]*H[3][4]);
					
					deter=T[1][1]*(T[2][2]*T[3][3]-T[3][2]*T[2][3])+T[2][1]*(T[3][2]*T[1][3]-T[1][2]*T[3][3])+T[3][1]*(T[1][2]*T[2][3]-T[2][2]*T[1][3]);
					
					iT[1][1]=(T[2][2]*T[3][3]-T[2][3]*T[3][2])/deter;
					iT[1][2]=(T[1][3]*T[3][2]-T[1][2]*T[3][3])/deter;
					iT[1][3]=(T[1][2]*T[2][3]-T[1][3]*T[2][2])/deter;
					iT[2][1]=(T[2][3]*T[3][1]-T[2][1]*T[3][3])/deter;
					iT[2][2]=(T[1][1]*T[3][3]-T[1][3]*T[3][1])/deter;
					iT[2][3]=(T[1][3]*T[2][1]-T[1][1]*T[2][3])/deter;
					iT[3][1]=(T[2][1]*T[3][2]-T[2][2]*T[3][1])/deter;
					iT[3][2]=(T[1][2]*T[3][1]-T[1][1]*T[3][2])/deter;
					iT[3][3]=(T[1][1]*T[2][2]-T[1][2]*T[2][1])/deter;
					
					T[1][1]=H[1][1]*iT[1][1]+H[2][1]*iT[2][1]+H[3][1]*iT[3][1];
					T[1][2]=H[1][1]*iT[1][2]+H[2][1]*iT[2][2]+H[3][1]*iT[3][2];
					T[1][3]=H[1][1]*iT[1][3]+H[2][1]*iT[2][3]+H[3][1]*iT[3][3];
					T[2][1]=H[1][2]*iT[1][1]+H[2][2]*iT[2][1]+H[3][2]*iT[3][1];
					T[2][2]=H[1][2]*iT[1][2]+H[2][2]*iT[2][2]+H[3][2]*iT[3][2];
					T[2][3]=H[1][2]*iT[1][3]+H[2][2]*iT[2][3]+H[3][2]*iT[3][3];
					T[3][1]=H[1][3]*iT[1][1]+H[2][3]*iT[2][1]+H[3][3]*iT[3][1];
					T[3][2]=H[1][3]*iT[1][2]+H[2][3]*iT[2][2]+H[3][3]*iT[3][2];
					T[3][3]=H[1][3]*iT[1][3]+H[2][3]*iT[2][3]+H[3][3]*iT[3][3];
					T[4][1]=H[1][4]*iT[1][1]+H[2][4]*iT[2][1]+H[3][4]*iT[3][1];
					T[4][2]=H[1][4]*iT[1][2]+H[2][4]*iT[2][2]+H[3][4]*iT[3][2];
					T[4][3]=H[1][4]*iT[1][3]+H[2][4]*iT[2][3]+H[3][4]*iT[3][3];
					
					K[1][1]=Pp[1][1]*T[1][1]+Pp[1][2]*T[2][1]+Pp[1][3]*T[3][1]+Pp[1][4]*T[4][1];
					K[1][2]=Pp[1][1]*T[1][2]+Pp[1][2]*T[2][2]+Pp[1][3]*T[3][2]+Pp[1][4]*T[4][2];
					K[1][3]=Pp[1][1]*T[1][3]+Pp[1][2]*T[2][3]+Pp[1][3]*T[3][3]+Pp[1][4]*T[4][3];
					K[2][1]=Pp[2][1]*T[1][1]+Pp[2][2]*T[2][1]+Pp[2][3]*T[3][1]+Pp[2][4]*T[4][1];
					K[2][2]=Pp[2][1]*T[1][2]+Pp[2][2]*T[2][2]+Pp[2][3]*T[3][2]+Pp[2][4]*T[4][2];
					K[2][3]=Pp[2][1]*T[1][3]+Pp[2][2]*T[2][3]+Pp[2][3]*T[3][3]+Pp[2][4]*T[4][3];
					K[3][1]=Pp[3][1]*T[1][1]+Pp[3][2]*T[2][1]+Pp[3][3]*T[3][1]+Pp[3][4]*T[4][1];
					K[3][2]=Pp[3][1]*T[1][2]+Pp[3][2]*T[2][2]+Pp[3][3]*T[3][2]+Pp[3][4]*T[4][2];
					K[3][3]=Pp[3][1]*T[1][3]+Pp[3][2]*T[2][3]+Pp[3][3]*T[3][3]+Pp[3][4]*T[4][3];
					K[4][1]=Pp[4][1]*T[1][1]+Pp[4][2]*T[2][1]+Pp[4][3]*T[3][1]+Pp[4][4]*T[4][1];
					K[4][2]=Pp[4][1]*T[1][2]+Pp[4][2]*T[2][2]+Pp[4][3]*T[3][2]+Pp[4][4]*T[4][2];
					K[4][3]=Pp[4][1]*T[1][3]+Pp[4][2]*T[2][3]+Pp[4][3]*T[3][3]+Pp[4][4]*T[4][3];
					
					T[1][1]=z[1][1]-(-2.*x[2][1]*x[4][1]+2.*x[1][1]*x[3][1]);
					T[2][1]=z[2][1]-(-2.*x[1][1]*x[2][1]-2.*x[3][1]*x[4][1]);
					T[3][1]=z[3][1]-(-x[1][1]*x[1][1]+x[2][1]*x[2][1]+x[3][1]*x[3][1]-x[4][1]*x[4][1]);
					
					x[1][1]=xp[1][1]+K[1][1]*T[1][1]+K[1][2]*T[2][1]+K[1][3]*T[3][1];
					x[2][1]=xp[2][1]+K[2][1]*T[1][1]+K[2][2]*T[2][1]+K[2][3]*T[3][1];
					x[3][1]=xp[3][1]+K[3][1]*T[1][1]+K[3][2]*T[2][1]+K[3][3]*T[3][1];
					x[4][1]=xp[4][1]+K[4][1]*T[1][1]+K[4][2]*T[2][1]+K[4][3]*T[3][1];
					
					T[1][1]=H[1][1]*Pp[1][1]+H[1][2]*Pp[2][1]+H[1][3]*Pp[3][1]+H[1][4]*Pp[4][1];
					T[1][2]=H[1][1]*Pp[1][2]+H[1][2]*Pp[2][2]+H[1][3]*Pp[3][2]+H[1][4]*Pp[4][2];
					T[1][3]=H[1][1]*Pp[1][3]+H[1][2]*Pp[2][3]+H[1][3]*Pp[3][3]+H[1][4]*Pp[4][3];
					T[1][4]=H[1][1]*Pp[1][4]+H[1][2]*Pp[2][4]+H[1][3]*Pp[3][4]+H[1][4]*Pp[4][4];
					T[2][1]=H[2][1]*Pp[1][1]+H[2][2]*Pp[2][1]+H[2][3]*Pp[3][1]+H[2][4]*Pp[4][1];
					T[2][2]=H[2][1]*Pp[1][2]+H[2][2]*Pp[2][2]+H[2][3]*Pp[3][2]+H[2][4]*Pp[4][2];
					T[2][3]=H[2][1]*Pp[1][3]+H[2][2]*Pp[2][3]+H[2][3]*Pp[3][3]+H[2][4]*Pp[4][3];
					T[2][4]=H[2][1]*Pp[1][4]+H[2][2]*Pp[2][4]+H[2][3]*Pp[3][4]+H[2][4]*Pp[4][4];
					T[3][1]=H[3][1]*Pp[1][1]+H[3][2]*Pp[2][1]+H[3][3]*Pp[3][1]+H[3][4]*Pp[4][1];
					T[3][2]=H[3][1]*Pp[1][2]+H[3][2]*Pp[2][2]+H[3][3]*Pp[3][2]+H[3][4]*Pp[4][2];
					T[3][3]=H[3][1]*Pp[1][3]+H[3][2]*Pp[2][3]+H[3][3]*Pp[3][3]+H[3][4]*Pp[4][3];
					T[3][4]=H[3][1]*Pp[1][4]+H[3][2]*Pp[2][4]+H[3][3]*Pp[3][4]+H[3][4]*Pp[4][4];
					
					P[1][1]=Pp[1][1]-K[1][1]*T[1][1]-K[1][2]*T[2][1]-K[1][3]*T[3][1];
					P[1][2]=Pp[1][2]-K[1][1]*T[1][2]-K[1][2]*T[2][2]-K[1][3]*T[3][2];
					P[1][3]=Pp[1][3]-K[1][1]*T[1][3]-K[1][2]*T[2][3]-K[1][3]*T[3][3];
					P[1][4]=Pp[1][4]-K[1][1]*T[1][4]-K[1][2]*T[2][4]-K[1][3]*T[3][4];
					
					P[2][1]=Pp[2][1]-K[2][1]*T[1][1]-K[2][2]*T[2][1]-K[2][3]*T[3][1];
					P[2][2]=Pp[2][2]-K[2][1]*T[1][2]-K[2][2]*T[2][2]-K[2][3]*T[3][2];
					P[2][3]=Pp[2][3]-K[2][1]*T[1][3]-K[2][2]*T[2][3]-K[2][3]*T[3][3];
					P[2][4]=Pp[2][4]-K[2][1]*T[1][4]-K[2][2]*T[2][4]-K[2][3]*T[3][4];
					
					P[3][1]=Pp[3][1]-K[3][1]*T[1][1]-K[3][2]*T[2][1]-K[3][3]*T[3][1];
					P[3][2]=Pp[3][2]-K[3][1]*T[1][2]-K[3][2]*T[2][2]-K[3][3]*T[3][2];
					P[3][3]=Pp[3][3]-K[3][1]*T[1][3]-K[3][2]*T[2][3]-K[3][3]*T[3][3];
					P[3][4]=Pp[3][4]-K[3][1]*T[1][4]-K[3][2]*T[2][4]-K[3][3]*T[3][4];		
					
					P[4][1]=Pp[4][1]-K[4][1]*T[1][1]-K[4][2]*T[2][1]-K[4][3]*T[3][1];
					P[4][2]=Pp[4][2]-K[4][1]*T[1][2]-K[4][2]*T[2][2]-K[4][3]*T[3][2];
					P[4][3]=Pp[4][3]-K[4][1]*T[1][3]-K[4][2]*T[2][3]-K[4][3]*T[3][3];
					P[4][4]=Pp[4][4]-K[4][1]*T[1][4]-K[4][2]*T[2][4]-K[4][3]*T[3][4];	
					
					q0=x[1][1];
					q1=x[2][1];
					q2=x[3][1];
					q3=x[4][1];
					
					scalehyo=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
					
					q0=q0/scalehyo;
					q1=q1/scalehyo;
					q2=q2/scalehyo;
					q3=q3/scalehyo;

					// Angle from Kalman filter------------------------------------------------!!!
					kalman_roll=atan2(2*(q2*q3+q0*q1),1-2*(q1*q1+q2*q2))*changetoD;
					kalman_pitch=-1*asin(2*(q1*q3-q0*q2))*changetoD;
					kalman_yaw=atan2(2*(q1*q2+q0*q3),1-2*(q2*q2+q3*q3))*changetoD;

					// Angle from Accelerometer------------------------------------------------!!!
					if(temp_hyo_zacc<0)
						myX=-1.;
					else
						myX=1.;
					acc_roll = atan2(temp_hyo_yacc,myX*sqrt(temp_hyo_zacc*temp_hyo_zacc+0.003*temp_hyo_xacc*temp_hyo_xacc))*changetoD;
					acc_pitch = atan2(-1*temp_hyo_xacc,sqrt(temp_hyo_yacc*temp_hyo_yacc+temp_hyo_zacc*temp_hyo_zacc))*changetoD;
		
					acc_roll_LPF = alpha23*acc_roll_LPF + (1.-alpha23)*acc_roll;
					acc_pitch_LPF = alpha23*acc_pitch_LPF + (1.-alpha23)*acc_pitch;

					// Angle from Rate gyro integration
					gyr_dps_x_hubo[0] = gyr_dps_x_hubo[1];
					gyr_dps_y_hubo[0] = gyr_dps_y_hubo[1];
					gyr_dps_z_hubo[0] = gyr_dps_z_hubo[1];

					ang_x_hubo_rad = ang_x_hubo*changetoR;
					ang_y_hubo_rad = ang_y_hubo*changetoR;
					gyr_dps_x_hubo[1] = temp_x_gyro + temp_y_gyro*sin(ang_x_hubo_rad)*tan(ang_y_hubo_rad) + temp_z_gyro*cos(ang_x_hubo_rad)*tan(ang_y_hubo_rad);
					gyr_dps_y_hubo[1] = temp_y_gyro*cos(ang_x_hubo_rad) - temp_z_gyro*sin(ang_x_hubo_rad);
					gyr_dps_z_hubo[1] = temp_y_gyro*sin(ang_x_hubo_rad)/cos(ang_y_hubo_rad) + temp_z_gyro*cos(ang_x_hubo_rad)/cos(ang_y_hubo_rad);
					gyro_sum_roll=gyro_sum_roll+gyr_dps_x_hubo[1]*0.005+0.5*(gyr_dps_x_hubo[1]-gyr_dps_x_hubo[0])*0.005;
					gyro_sum_pitch=gyro_sum_pitch+gyr_dps_y_hubo[1]*0.005+0.5*(gyr_dps_y_hubo[1]-gyr_dps_y_hubo[0])*0.005;
					
					ang_x_hubo_old=ang_x_hubo;
					ang_y_hubo_old=ang_y_hubo;
					ang_z_hubo_old=ang_z_hubo;
					
					// Angle from complementary filter-baehyoin
					acc_ang_x_hubo_LPFed = alpha015*acc_ang_x_hubo_LPFed + (1.-alpha015)*acc_roll;
					acc_ang_y_hubo_LPFed = alpha015*acc_ang_y_hubo_LPFed + (1.-alpha015)*acc_pitch;
					
					gyr_ang_x_hubo_HPFed = alpha015*(gyr_ang_x_hubo_HPFed + 0.5*(gyr_dps_x_hubo[0]+gyr_dps_x_hubo[1])*0.005);
					gyr_ang_y_hubo_HPFed = alpha015*(gyr_ang_y_hubo_HPFed + 0.5*(gyr_dps_y_hubo[0]+gyr_dps_y_hubo[1])*0.005);
					gyr_ang_z_hubo_HPFed = alpha015*(gyr_ang_z_hubo_HPFed + 0.5*(gyr_dps_z_hubo[0]+gyr_dps_z_hubo[1])*0.005);

					complementary_roll = gyr_ang_x_hubo_HPFed + acc_ang_x_hubo_LPFed;	
					complementary_pitch = gyr_ang_y_hubo_HPFed + acc_ang_y_hubo_LPFed;
					complementary_yaw = gyr_ang_z_hubo_HPFed;

					// Final Angle Computation------------------------------------------------!!
					ang_x_hubo = 1.0*(0.0*acc_roll_LPF+1.*kalman_roll)+0.0*complementary_roll;
					ang_y_hubo = 1.0*(0.0*acc_pitch_LPF+1.*kalman_pitch)+0.0*complementary_pitch;
					ang_z_hubo = kalman_yaw;
			

					if((gyrostate2==0) && (accstate==1))
					{
						ang_x_hubo=0.7*ang_x_hubo+0.3*ang_x_hubo_old;
						ang_y_hubo=0.7*ang_y_hubo+0.3*ang_y_hubo_old;
						ang_z_hubo=0.7*ang_z_hubo+0.3*ang_z_hubo_old;
					}
					
					
					offseted_roll_angle=ang_x_hubo+ IMUSensor[CENTERIMU].RollOffset;
					offseted_pitch_angle=ang_y_hubo+ IMUSensor[CENTERIMU].PitchOffset; 


	}

	IMUSensor[CENTERIMU].Roll = ang_x_hubo; //offseted_roll_angle;//ang_x_hubo;
	IMUSensor[CENTERIMU].Pitch = ang_y_hubo; //offseted_pitch_angle;//ang_y_hubo;
	IMUSensor[CENTERIMU].Roll_Velocity = gyr_dps_x_hubo[1];
	IMUSensor[CENTERIMU].Pitch_Velocity = gyr_dps_y_hubo[1];
	IMUSensor[CENTERIMU].Yaw_angle = ang_z_hubo;
	//printf("\n %f",ang_z_hubo);
	IMUSensor[CENTERIMU].Yaw_Velocity = gyr_dps_z_hubo[1];
	// Z->Y- >X eular
	//ang_x_hubo = roll angle;
	//ang_y_hubo = pitch angle;
	//ang_z_hubo = yaw angle;

	//gyr_dps_x_hubo[1] = roll(euler) velocity 
	//gyr_dps_y_hubo[1] = pitch(euler) velocity 
	//gyr_dps_z_hubo[1] = yaw(euler) velocity 

	//temp_x_gyro = local x axis angular vel
	//temp_y_gyro = local y axis angular vel
	//temp_z_gyro = local z axis angular vel
    //

	for(i=CENTERIMU ; i<NO_OF_IMU; i++) 
		pSharedMemory->IMU_debug[i] = IMUSensor[i];
}
*/

float zaccCaliDRC(float accy, float accz)
{
	float yindex=accy;
	float zacc=accz;

	if(yindex<-0.0146633136094679)
	{
		if(zacc>=0.971993902439050)
		{
			if(zacc>=0.993762118240402)
			{
				if(zacc>=0.993762118240402 && zacc<=0.995621807747479) return 1.2008*zacc-0.2028;
				else if(zacc>=0.995621807747479 && zacc<=0.997366040955615) return 1.0885*zacc-0.0910;
				else if(zacc>=0.997366040955615 && zacc<=0.998511036094514) return 1.5067*zacc-0.5081;
				else if(zacc>=0.998511036094514 && zacc<=0.999895090590301) return 0.9565*zacc+0.0413;
				else if(zacc>=0.999895090590301 && zacc<=1.00043458213257) return 1.8733*zacc-0.8754;
				else if(zacc>=1.00043458213257 && zacc<=1.00072768581629) return 2.6283*zacc-1.6308;
				else if(zacc>=1.00072768581629 && zacc<=1.00107913878383) return 1.2090*zacc-0.2104;
				else if(zacc>=1.00107913878383) return zacc;
				else return 300;
			}
			else
			{
				if(zacc>=0.971993902439050 && zacc<=0.976059697668474) return 1.1124*zacc-0.1151;
				else if(zacc>=0.976059697668474 && zacc<=0.979667844522977) return 1.1080*zacc-0.1108;
				else if(zacc>=0.979667844522977 && zacc<=0.982995258166508) return 1.0954*zacc-0.0985;
				else if(zacc>=0.982995258166508 && zacc<=0.986108629130968) return 1.1664*zacc-0.1683;
				else if(zacc>=0.986108629130968 && zacc<=0.989014421838779) return 1.0637*zacc-0.0670;
				else if(zacc>=0.989014421838779 && zacc<=0.991234474753313) return 1.2589*zacc-0.2600;
				else if(zacc>=0.991234474753313 && zacc<=0.993762118240402) return 1.0479*zacc-0.0509;
				else return 300;
			}
		}
		else
		{
			if(zacc>=0.875649355300849)
			{
				if(zacc>=0.875649355300849 && zacc<=0.946527533316572) return 1.0389*zacc-0.0431;
				else if(zacc>=0.946527533316572 && zacc<=0.951718882303135) return 1.1065*zacc-0.1071;
				else if(zacc>=0.951718882303135 && zacc<=0.957067060085848) return 1.0036*zacc-0.0092;        
				else if(zacc>=0.957067060085848 && zacc<=0.962573731501057) return 0.9858*zacc+0.0079;
				else if(zacc>=0.962573731501057 && zacc<=0.967551324929256) return 0.9796*zacc+0.0138;
				else if(zacc>=0.967551324929256 && zacc<=0.971993902439050) return 1.0138*zacc-0.0193;
				else return 300;
			}
			else
			{
				if(zacc<=0.0188656815540428) return zacc;
				else if(zacc>=0.0188656815540428 && zacc<=0.194632222408310) return 0.9902*zacc-0.0177;
				else if(zacc>=0.194632222408310 && zacc<=0.362036937283571) return 1.0054*zacc-0.0207;
				else if(zacc>=0.362036937283571 && zacc<=0.518197814207646) return 1.0083*zacc-0.0217;
				else if(zacc>=0.518197814207646 && zacc<=0.655441987940811) return 1.0423*zacc-0.0393;
				else if(zacc>=0.655441987940811 && zacc<=0.779670612701974) return 0.9909*zacc-0.0056;
				else if(zacc>=0.779670612701974 && zacc<=0.875649355300849) return 1.0380*zacc-0.0423;
				else return 300;
			}
		}
	}
	else
	{
		if(zacc>=0.970716554274530)
		{
			if(zacc>=0.992683049427523)
			{
				if(zacc>=1.00067041420122) return zacc;
				else if(zacc>=0.999963165075037 && zacc<=1.00067041420122) return 0.1834*zacc+0.8165;
				else if(zacc>=0.998954260324101 && zacc<=0.999963165075037) return 0.4290*zacc+0.5709;
				else if(zacc>=0.997813660874763 && zacc<=0.998954260324101) return 0.6527*zacc+0.3474;
				else if(zacc>=0.996454845814965 && zacc<=0.997813660874763) return 0.7714*zacc+0.2290;
				else if(zacc>=0.994759379407609 && zacc<=0.996454845814965) return 0.7935*zacc+0.2069;
				else if(zacc>=0.992683049427523 && zacc<=0.994759379407609) return 0.7947*zacc+0.2058;
				else return 300;
			}
			else
			{
				if(zacc>=0.990427184466000 && zacc<=0.992683049427523) return 0.8696*zacc+0.1314;
				else if(zacc>=0.987853818779081 && zacc<=0.990427184466000) return 0.8724*zacc+0.1287;
				else if(zacc>=0.984970157987121 && zacc<=0.987853818779081) return 0.8779*zacc+0.1232;
				else if(zacc>=0.981936470588247 && zacc<=0.984970157987121) return 0.9506*zacc+0.0516;
				else if(zacc>=0.978481341107881 && zacc<=0.981936470588247) return 0.9147*zacc+0.0869;
				else if(zacc>=0.974566657541757 && zacc<=0.978481341107881) return 0.8797*zacc+0.1211;
				else if(zacc>=0.970716554274530 && zacc<=0.974566657541757) return 0.9817*zacc+0.0217;
				else return 300;
			}    
		}
		else
		{
			if(zacc>=0.933910573122528)
			{
				if(zacc>=0.966246584255038 && zacc<=0.970716554274530) return 0.9101*zacc+0.0912;
				else if(zacc>=0.960858498238009 && zacc<=0.966246584255038) return 0.7944*zacc+0.2030;
				else if(zacc>=0.955172490914181 && zacc<=0.960858498238009) return 0.8239*zacc+0.1747;
				else if(zacc>=0.950216315049229 && zacc<=0.955172490914181) return 1.0009*zacc+0.0056;
				else if(zacc>=0.945446019629225 && zacc<=0.950216315049229) return 1.0884*zacc-0.0775;
				else if(zacc>=0.940018653177368 && zacc<=0.945446019629225) return 1.0215*zacc-0.0143;
				else if(zacc>=0.933910573122528 && zacc<=0.940018653177368) return 0.9553*zacc+0.0479;
				else return 300;
			}
			else
			{
				if(zacc>=0.857758425790247 && zacc<=0.933910573122528) return 0.9631*zacc+0.0407;
				else if(zacc>=0.755270390234886 && zacc<=0.857758425790247) return 0.9746*zacc+0.0308;
				else if(zacc>=0.629721385815360 && zacc<=0.755270390234886) return 0.9810*zacc+0.0259;
				else if(zacc>=0.484431463811950 && zacc<=0.629721385815360) return 0.9823*zacc+0.0251;
				else if(zacc>=0.325872739796789 && zacc<=0.484431463811950) return 0.9945*zacc+0.0192;
				else if(zacc>=0.156707495104145 && zacc<=0.325872739796789) return 0.9961*zacc+0.0187;
				else if(zacc>=-0.0155395985401463 && zacc<=0.156707495104145) return 1.0067*zacc+0.0170;
				else if(zacc<=-0.0155395985401463) return zacc;
				else return 300;
			}        
		}
	}

}


float yaccCaliDRC(float accy)
{
	float yacc=accy;

	if(yacc>=0.00204774897680758)
	{
		if(yacc>=0.227312296681847)
		{
			if(yacc>=0.486549717395858)
			{
				if(yacc>=0.486549717395858 && yacc<=0.631175513223602) return 0.9891*yacc+0.0175;
				else if(yacc>=0.631175513223602 && yacc<=0.756468891710502) return 0.9854*yacc+0.0198;
				else if(yacc>=0.756468891710502 && yacc<=0.858935536420764) return 0.9777*yacc+0.0257;
				else if(yacc>=0.858935536420764 && yacc<=0.935119855346995) return 0.9683*yacc+0.0338;
				else if(yacc>=0.935119855346995 && yacc<=0.983336656578238) return 0.9411*yacc+0.0592;
				else if(yacc>=0.983336656578238 && yacc<=1.00095784671533) return 0.8736*yacc+0.1256;
				else if(yacc>=1.00095784671533) return yacc;
				else return 200;
			}
			else
			{
				if(yacc>=0.227312296681847 && yacc<=0.243892111683388) return 1.0036*yacc+0.0127;
				else if(yacc>=0.243892111683388 && yacc<=0.261226446743078) return 0.9796*yacc+0.0185;
				else if(yacc>=0.261226446743078 && yacc<=0.277919831223626) return 1.0077*yacc+0.0112;
				else if(yacc>=0.277919831223626 && yacc<=0.294676935659763) return 0.9869*yacc+0.0170;
				else if(yacc>=0.294676935659763 && yacc<=0.311057540309829) return 1.0158*yacc+0.0085;
				else if(yacc>=0.311057540309829 && yacc<=0.327229249011852) return 1.0227*yacc+0.0063;
				else if(yacc>=0.327229249011852 && yacc<=0.486549717395858) return 0.9905*yacc+0.0168;
				else return 200;
			}
		}
		else
		{
			if(yacc>=0.107014416004710)
			{
				if(yacc>=0.107014416004710 && yacc<=0.124271557623871) return 0.9975*yacc+0.0140;
				else if(yacc>=0.124271557623871 && yacc<=0.141513458162667) return 0.9916*yacc+0.0147;
				else if(yacc>=0.141513458162667 && yacc<=0.158799215686270) return 1.0053*yacc+0.0128;
				else if(yacc>=0.158799215686270 && yacc<=0.175885422740523) return 1.0051*yacc+0.0128;
				else if(yacc>=0.175885422740523 && yacc<=0.193324938406791) return 0.9771*yacc+0.0177;
				else if(yacc>=0.193324938406791 && yacc<=0.209972155753751) return 1.0303*yacc+0.0074;
				else if(yacc>=0.209972155753751 && yacc<=0.227312296681847) return 0.9823*yacc+0.0175;
				else return 200;
			}
			else
			{
				if(yacc>=0.00204774897680758 && yacc<=0.0192885520125461) return 1.0110*yacc+0.0140;
				else if(yacc>=0.0192885520125461 && yacc<=0.0369017375674065) return 0.9978*yacc+0.0143;
				else if(yacc>=0.0369017375674065 && yacc<=0.0544614537444940) return 0.9955*yacc+0.0144;
				else if(yacc>=0.0544614537444940 && yacc<=0.0718284908321570) return 0.9995*yacc+0.0142;
				else if(yacc>=0.0718284908321570 && yacc<=0.0893334264172017) return 0.9916*yacc+0.0147;
				else if(yacc>=0.0893334264172017 && yacc<=0.107014416004710) return 0.9842*yacc+0.0154;
				else return 200;
			}
		}
	}
	else
	{
		if(yacc>=-0.253991544965411)
		{
			if(yacc>=-0.134681205164990)
			{
				if(yacc>=-0.134681205164990 && yacc<=-0.117819965870308) return 0.9990*yacc+0.0141;
				else if(yacc>=-0.117819965870308 && yacc<=-0.0996021812516235) return 0.9975*yacc+0.0139;
				else if(yacc>=-0.0996021812516235 && yacc<=-0.0827182933956751) return 1.0176*yacc+0.0159;
				else if(yacc>=-0.0827182933956751 && yacc<=-0.0634302593659939) return 0.8746*yacc+0.0041;
				else if(yacc>=-0.0634302593659939 && yacc<=-0.0260000000000005) return 0.4866*yacc-0.0205;
				else if(yacc>=-0.0260000000000005 && yacc<=-0.0259729415187665) return 641.3564*yacc+16.6421;
				else if(yacc>=-0.0259729415187665 && yacc<=-0.0146633136094679) return 1.3970*yacc+0.0205;
				else if(yacc>=-0.0146633136094679 && yacc<=0.00204774897680758) return 0.9638*yacc+0.0141;
				else return 200;
			}
			else
			{
				if(yacc>=-0.253991544965411 && yacc<=-0.237470236477306) return 1.0145*yacc+0.0173;
				else if(yacc>=-0.237470236477306 && yacc<=-0.221003424657535) return 1.0037*yacc+0.0147;
				else if(yacc>=-0.221003424657535 && yacc<=-0.202976438188494) return 0.9967*yacc+0.0132; 
				else if(yacc>=-0.202976438188494 && yacc<=-0.185214524851913) return 0.9471*yacc+0.0031;
				else if(yacc>=-0.185214524851913 && yacc<=-0.169377539175854) return 1.0622*yacc+0.0244;
				else if(yacc>=-0.169377539175854 && yacc<=-0.151761838049872) return 1.0150*yacc+0.0164;
				else if(yacc>=-0.151761838049872 && yacc<=-0.134681205164990) return 1.0049*yacc+0.0149;
				else return 200;
			}
		}
		else
		{
			if(yacc>=-0.510266654727796)
			{
				if(yacc>=-0.510266654727796 && yacc<=-0.353136032185064) return 1.0086*yacc+0.0156;
				else if(yacc>=-0.353136032185064 && yacc<=-0.337013547840811) return 1.0110*yacc+0.0165;
				else if(yacc>=-0.337013547840811 && yacc<=-0.321160675965664) return 1.0159*yacc+0.0182;
				else if(yacc>=-0.321160675965664 && yacc<=-0.304753435517974) return 1.0541*yacc+0.0304;
				else if(yacc>=-0.304753435517974 && yacc<=-0.287743246719835) return 0.9732*yacc+0.0058;
				else if(yacc>=-0.287743246719835 && yacc<=-0.271873337028825) return 1.0280*yacc+0.0215;
				else if(yacc>=-0.271873337028825 && yacc<=-0.253991544965411) return 0.9831*yacc+0.0093;
				else return 200;
			}
			else
			{
				if(yacc<=-0.998692512037189) return yacc;       
				else if(yacc>=-0.998692512037189 && yacc<=-0.983816446156423) return 1.0378*yacc+0.0364;
				else if(yacc>=-0.983816446156423 && yacc<=-0.943512312427843) return 1.1253*yacc+0.1225;
				else if(yacc>=-0.943512312427843 && yacc<=-0.871709471766846) return 1.0258*yacc+0.0286;
				else if(yacc>=-0.871709471766846 && yacc<=-0.773437420062122) return 1.0218*yacc+0.0252;
				else if(yacc>=-0.773437420062122 && yacc<=-0.651722764357696) return 1.0143*yacc+0.0194;
				else if(yacc>=-0.651722764357696 && yacc<=-0.510266654727796) return 1.0086*yacc+0.0157;
				else return 200;
			}
		}
	}
}

float xaccCaliDRC(float accx)
{
	float xacc=accx;

	if(xacc>=0.0100000000000004)
	{
		if(xacc>=0.306650802590818)
		{
			if(xacc>=0.450481647520308)
			{
				if(xacc>=0.986725031081631) return xacc;
				else if(xacc>=0.970636320191163 && xacc<=0.986725031081631) return 0.9959*xacc+0.0567;
				else if(xacc>=0.924216898749106 && xacc<=0.970636320191163) return 0.9778*xacc+0.0356;
				else if(xacc>=0.849828447951891 && xacc<=0.924216898749106) return 0.9943*xacc+0.0203;
				else if(xacc>=0.748497402867230 && xacc<=0.849828447951891) return 0.9859*xacc+0.0274;
				else if(xacc>=0.624288201663203 && xacc<=0.748497402867230) return 0.9955*xacc+0.0202;
				else if(xacc>=0.481931678292986 && xacc<=0.624288201663203) return 1.0023*xacc+0.0160;
				else if(xacc>=0.466100295778436 && xacc<=0.481931678292986) return 0.9535*xacc+0.0395;
				else if(xacc>=0.450481647520308 && xacc<=0.466100295778436) return 0.9932*xacc+0.0210;
				else return 100;
			}
			else
			{
				if(xacc>=0.435140460619846 && xacc<=0.450481647520308) return 1.0058*xacc+0.0153;
				else if(xacc>=0.419774512836379 && xacc<=0.435140460619846) return 1.0086*xacc+0.0141;
				else if(xacc>=0.403849616097428 && xacc<=0.419774512836379) return 1.0062*xacc+0.0151;
				else if(xacc>=0.387872267256201 && xacc<=0.403849616097428) return 0.9921*xacc+0.0208;
				else if(xacc>=0.371935629587807 && xacc<=0.387872267256201) return 0.9906*xacc+0.0214;
				else if(xacc>=0.355968951376683 && xacc<=0.371935629587807) return 1.0197*xacc+0.0106;
				else if(xacc>=0.339251495448637 && xacc<=0.355968951376683) return 0.9739*xacc+0.0269;
				else if(xacc>=0.325718967517397 && xacc<=0.339251495448637) return 1.1912*xacc-0.0468;
				else if(xacc>=0.306650802590818 && xacc<=0.325718967517397) return 0.8724*xacc+0.0570;
				else return 100;
			}
		}
		else
		{
			if(xacc>=0.154012070815449)
			{
				if(xacc>=0.290112032009148 && xacc<=0.306650802590818) return 1.0031*xacc+0.0169;
				else if(xacc>=0.273603101196951 && xacc<=0.290112032009148) return 0.9973*xacc+0.0186;
				else if(xacc>=0.255771711456858 && xacc<=0.273603101196951) return 0.9489*xacc+0.0318;
				else if(xacc>=0.239207486285901 && xacc<=0.255771711456858) return 1.0117*xacc+0.0158;
				else if(xacc>=0.222789671090597 && xacc<=0.239207486285901) return 1.0209*xacc+0.0136;
				else if(xacc>=0.205631135637522 && xacc<=0.222789671090597) return 1.0014*xacc+0.0179;
				else if(xacc>=0.188833417721521 && xacc<=0.205631135637522) return 1.0164*xacc+0.0148;
				else if(xacc>=0.172320239880061 && xacc<=0.188833417721521) return 1.0215*xacc+0.0139;
				else if(xacc>=0.154012070815449 && xacc<=0.172320239880061) return 0.9545*xacc+0.0254;
				else return 100;
			}
			else
			{
				if(xacc>=0.136767668979695 && xacc<=0.154012070815449) return 0.9931*xacc+0.0195;
				else if(xacc>=0.119988839285717 && xacc<=0.136767668979695) return 1.0083*xacc+0.0174;
				else if(xacc>=0.102580500000001 && xacc<=0.119988839285717) return 1.0135*xacc+0.0168;
				else if(xacc>=0.0850460750853239 && xacc<=0.102580500000001) return 0.9896*xacc+0.0192;
				else if(xacc>=0.0681206800445924 && xacc<=0.0850460750853239) return 1.0090*xacc+0.0176;
				else if(xacc>=0.0498597997138772 && xacc<=0.0681206800445924) return 0.9701*xacc+0.0202;
				else if(xacc>=0.0333008765695338 && xacc<=0.0498597997138772) return 1.0557*xacc+0.0160;
				else if(xacc>=0.0161801422175404 && xacc<=0.0333008765695338) return 1.0011*xacc+0.0178;
				else if(xacc>=0.0100000000000004 && xacc<=0.0161801422175404) return 5.4968*xacc-0.0550;
				else return 100;
			}
		}
	}
	else
	{
		if(xacc>=-0.325386135857457)
		{
			if(xacc>=-0.172488345970232)
			{
				if(xacc>=-0.0328883263847534 && xacc<=0.0100000000000004) return 0.3773*xacc-0.0038;
				else if(xacc>=-0.0508355731225303 && xacc<=-0.0328883263847534) return 0.9641*xacc+0.0155;
				else if(xacc>=-0.0685314938684500 && xacc<=-0.0508355731225303) return 0.9974*xacc+0.0172;
				else if(xacc>=-0.0859912454109010 && xacc<=-0.0685314938684500) return 0.9983*xacc+0.0173;
				else if(xacc>=-0.103486526069128 && xacc<=-0.0859912454109010) return 0.9864*xacc+0.0163;
				else if(xacc>=-0.120518092566622 && xacc<=-0.103486526069128) return 1.0281*xacc+0.0206;
				else if(xacc>=-0.138115437724384 && xacc<=-0.120518092566622) return 0.9918*xacc+0.0162;
				else if(xacc>=-0.154390756302521 && xacc<=-0.138115437724384) return 1.0623*xacc+0.0259;
				else if(xacc>=-0.172488345970232 && xacc<=-0.154390756302521) return 0.9484*xacc+0.0082;
				else return 100;
			}
			else
			{
				if(xacc>=-0.189739517518668 && xacc<=-0.172488345970232) return 0.9971*xacc+0.0168;
				else if(xacc>=-0.206784490025286 && xacc<=-0.189739517518668) return 1.0105*xacc+0.0193;
				else if(xacc>=-0.223612866817156 && xacc<=-0.206784490025286) return 1.0141*xacc+0.0200;
				else if(xacc>=-0.241488145048818 && xacc<=-0.223612866817156) return 0.9537*xacc+0.0065;
				else if(xacc>=-0.258329586877273 && xacc<=-0.241488145048818) return 1.0110*xacc+0.0204;
				else if(xacc>=-0.274900215982721 && xacc<=-0.258329586877273) return 1.0183*xacc+0.0223;
				else if(xacc>=-0.291926238738738 && xacc<=-0.274900215982721) return 0.9872*xacc+0.0137;
				else if(xacc>=-0.307377305407934 && xacc<=-0.291926238738738) return 1.0856*xacc+0.0424;
				else if(xacc>=-0.325386135857457 && xacc<=-0.307377305407934) return 0.9251*xacc-0.0069;
				else return 100;
			}
		}
		else
		{
			if(xacc>=-0.469642488532113)
			{
				if(xacc>=-0.341839184597966 && xacc<=-0.325386135857457) return 1.0024*xacc+0.0183;
				else if(xacc>=-0.358009742300438 && xacc<=-0.341839184597966) return 1.0198*xacc+0.0242;
				else if(xacc>=-0.375003952569169 && xacc<=-0.358009742300438) return 0.9643*xacc+0.0043;
				else if(xacc>=-0.390945500993468 && xacc<=-0.375003952569169) return 1.0107*xacc+0.0217;
				else if(xacc>=-0.407465590484277 && xacc<=-0.390945500993468) return 0.9870*xacc+0.0125;
				else if(xacc>=-0.423407001603423 && xacc<=-0.407465590484277) return 1.0021*xacc+0.0186;
				else if(xacc>=-0.438924663183941 && xacc<=-0.423407001603423) return 1.0215*xacc+0.0269;
				else if(xacc>=-0.455205640423031 && xacc<=-0.438924663183941) return 0.9771*xacc+0.0073;
				else if(xacc>=-0.469642488532113 && xacc<=-0.455205640423031) return 1.0830*xacc+0.0556;
				else return 100;
			}
			else
			{
				if(xacc>=-0.485708304256806 && xacc<=-0.469642488532113) return 0.9519*xacc-0.0060;
				else if(xacc>=-0.501196943483274 && xacc<=-0.485708304256806) return 0.9958*xacc+0.0153;
				else if(xacc>=-0.516420626895853 && xacc<=-0.501196943483274) return 0.9990*xacc+0.0169;
				else if(xacc>=-0.658882542037586 && xacc<=-0.516420626895853) return 1.0025*xacc+0.0187;
				else if(xacc>=-0.781039434523808 && xacc<=-0.658882542037586) return 1.0106*xacc+0.0241;
				else if(xacc>=-0.881419058553399 && xacc<=-0.781039434523808) return 0.9981*xacc+0.0143;
				else if(xacc>=-0.954101088270875 && xacc<=-0.881419058553399) return 1.0157*xacc+0.0298;
				else if(xacc>=-0.999058647841439 && xacc<=-0.954101088270875) return 1.0094*xacc+0.0238;
				else if(xacc>=-1.01398189678466 && xacc<=-0.999058647841439) return 1.0294*xacc+0.0438;
				else if(xacc<=-1.01398189678466) return xacc;
				else return 100;
			}        
		}    
	}

}
/******************************************************************************/





/******************************************************************************/
bool RequestEncoder(unsigned char _boardID)
{
	unsigned char tempData[8];

	tempData[0] = _boardID;
	tempData[1] = SendEncoder;

	switch(_boardID)
	{
	case JMC0:
	case JMC1:
	case JMC2:
	case JMC3:
	case JMC4:
	case JMC5:
	case JMC6:
	case JMC7:
	case EJMC3:
		PushCANMsg(0, CMD_TXDF, tempData, 2, 0);
		return true;
		break;
	case JMC8:
	case JMC9:
	case JMC10:
	case JMC11:
	case EJMC0:
	case EJMC1:
	case EJMC2:
	case EJMC4:
	case EJMC5:
		PushCANMsg(1, CMD_TXDF, tempData, 2, 0);
		return true;
		break;
	}

	RtWprintf(L"\n>>> Wrong board ID(RequestEncoder)..!!");
	return false; 
}
/******************************************************************************/





/******************************************************************************/
bool RequestCurrent(unsigned char _boardID)
{
	unsigned char tempData[8];

	tempData[0] = _boardID;
	tempData[1] = SendCurrent;

	switch(_boardID)
	{
	case JMC0:
	case JMC1:
	case JMC2:
	case JMC3:
	case JMC4:
	case JMC5:
	case JMC6:
	case JMC7:
	case EJMC3:
		PushCANMsg(0, CMD_TXDF, tempData, 2, 0);
		return true;
		break;
	case JMC8:
	case JMC9:
	case JMC10:
	case JMC11:
	case EJMC0:
	case EJMC1:
	case EJMC2:
	case EJMC4:
	case EJMC5:
		PushCANMsg(1, CMD_TXDF, tempData, 2, 0);
		return true;
		break;
	}

	RtWprintf(L"\n>>> Wrong board ID(RequestCurrent)..!!");
	return false;
}
/******************************************************************************/





/******************************************************************************/
bool RequestSensor(unsigned char _canChannel)
{
	unsigned char tempData[8];
 	
 	if(_canChannel >= 2) { RtWprintf(L"\n>>> Wrong CAN channel number(RequestSensor)..!!"); return false; }

	if(_canChannel == CAN0)
	{
		tempData[0] = 0xFF;//0x00;//0xFF;//0x11;//0xFF;
		tempData[1] = 0x03;
		PushCANMsg(_canChannel, SEND_SENSOR_TXDF, tempData, 2, 0);

		tempData[0] = 0x03;
		tempData[1] = 0x00;
		PushCANMsg(_canChannel, SEND_SENSOR_TXDF, tempData, 2, 0);
	}
	else if(_canChannel == CAN1)
	{
		tempData[0] = 0xFF;
		tempData[1] = 0x00;
		PushCANMsg(_canChannel, SEND_SENSOR_TXDF, tempData, 2, 0);
	}
	else { RtWprintf(L"\n>>> Wrong CAN channel number(RequestSensor)..!!"); return false; }

	return true;
}
/******************************************************************************/




/******************************************************************************/
bool SetFTParameter(unsigned char _ftID, FT _ftSensor)
{
	if(_ftID > NO_OF_FT) { RtWprintf(L"\n>>> Wrong FT sensor ID(SetFTParameter)..!!"); return false; }
	else FTSensor[_ftID] = _ftSensor;
	return true;
}
/******************************************************************************/




/******************************************************************************/
bool GetFTParameter(unsigned char _ftID, FT* _ftSensor)
{
	if(_ftID > NO_OF_FT) { RtWprintf(L"\n>>> Wrong FT sensor ID(GetFTParameter)..!!"); return false; }
	else *_ftSensor = FTSensor[_ftID];

	return true;
}
/******************************************************************************/




/******************************************************************************/
bool NullFTSensor(unsigned char _ftID, unsigned char _mode)
{
	unsigned char tempData[8];

	if(_ftID>NO_OF_FT) { RtWprintf(L"\n>>> Wrong FT sensor ID(NullFTSensor)..!!"); return false; }
	else
	{
		tempData[0] = FTSensor[_ftID].Controller_NO;
		tempData[1] = NullCMD;
		tempData[2] = _mode;

		PushCANMsg(FTSensor[_ftID].CAN_channel, CMD_TXDF, tempData, 3, 0);
	}

	return true;
}
/******************************************************************************/





/******************************************************************************/
bool NullFootAngleSensor(FT _ftSensor[])
{	
	unsigned char tempData[8];

	tempData[0] = _ftSensor[RFFT].Controller_NO;
	tempData[1] = NullCMD;
	tempData[2] = 0x04;
	PushCANMsg(_ftSensor[RFFT].CAN_channel, CMD_TXDF, tempData, 3, 0);
	
	Sleep(1);

	tempData[0] = _ftSensor[LFFT].Controller_NO;
	tempData[1] = NullCMD;
	tempData[2] = 0x04;
	PushCANMsg(_ftSensor[LFFT].CAN_channel, CMD_TXDF, tempData, 3, 0);

	return true;
}
/******************************************************************************/





/******************************************************************************/
bool NullIMUSensor(unsigned char _imuID, unsigned char _mode)
{
	unsigned char tempData[8];

	if(_imuID>NO_OF_IMU) { RtWprintf(L"\n>>> Wrong FT sensor ID(NullIMUSensor)..!!"); return false; }
	else
	{
		tempData[0] = IMUSensor[_imuID].Controller_NO;
		tempData[1] = NullCMD;
		tempData[2] = _mode;

		PushCANMsg(IMUSensor[_imuID].CAN_channel, CMD_TXDF, tempData, 3, 0);
	}

	return true;
}
/******************************************************************************/




/******************************************************************************/
bool PrintFTParameter(unsigned char _ftID)
{
	if(_ftID > NO_OF_FT) { RtWprintf(L"\n>>> Wrong FT sensor ID(PrintFTParameter)..!!"); return false; }
	else
	{
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> FT sensor(%d) parameter information", _ftID);
		RtWprintf(L"\n>>> Raw digit value Mx: %d", FTSensor[_ftID].dMx);
		RtWprintf(L"\n>>> Raw digit value My: %d", FTSensor[_ftID].dMy);
		RtWprintf(L"\n>>> Raw digit value Fz: %d", FTSensor[_ftID].dFz);
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> Numerical value Mx: %d", (int)(FTSensor[_ftID].Mx*1000.0f));
		RtWprintf(L"\n>>> Numerical value My: %d", (int)(FTSensor[_ftID].My*1000.0f));
		RtWprintf(L"\n>>> Numerical value Fz: %d", (int)(FTSensor[_ftID].Fz*1000.0f));
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> Filtered value Mx: %d", (int)(FTSensor[_ftID].Filtered_Mx*1000.0f));
		RtWprintf(L"\n>>> Filtered value My: %d", (int)(FTSensor[_ftID].Filtered_My*1000.0f));
		RtWprintf(L"\n>>> Filtered value Fz: %d", (int)(FTSensor[_ftID].Filtered_Fz*1000.0f));
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> Cut-off frequency: %d", (int)(FTSensor[_ftID].CutOffFeq*1000.0f));
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> Decouple matrix");
		RtWprintf(L"\n>>> %d\t %d\t %d", (int)(FTSensor[_ftID].DCMat[0][0]*1000.0f), (int)(FTSensor[_ftID].DCMat[0][1]*1000.0f), (int)(FTSensor[_ftID].DCMat[0][2]*1000.0f));
		RtWprintf(L"\n>>> %d\t %d\t %d", (int)(FTSensor[_ftID].DCMat[1][0]*1000.0f), (int)(FTSensor[_ftID].DCMat[1][1]*1000.0f), (int)(FTSensor[_ftID].DCMat[1][2]*1000.0f));
		RtWprintf(L"\n>>> %d\t %d\t %d", (int)(FTSensor[_ftID].DCMat[2][0]*1000.0f), (int)(FTSensor[_ftID].DCMat[2][1]*1000.0f), (int)(FTSensor[_ftID].DCMat[2][2]*1000.0f));
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> CAN channel number: %d", FTSensor[_ftID].CAN_channel);
		
		GetFTParameter(_ftID, &pSharedMemory->FTsensor);

		return true;
	}
}
/******************************************************************************/





/******************************************************************************/
bool SetIMUParameter(unsigned char _imuID, IMU _imuSensor)
{
	if(_imuID > NO_OF_IMU) { RtWprintf(L"\n>>> Wrong IMU ID(SetIMUParameter)..!!"); return false; }
	else IMUSensor[_imuID] = _imuSensor;

	return true;
}
/******************************************************************************/




/******************************************************************************/
bool GetIMUParameter(unsigned char _imuID, IMU* _imuSensor)
{
	if(_imuID > NO_OF_IMU) { RtWprintf(L"\n>>> Wrong IMU ID(GetIMUParameter)..!!"); return false; }
	else *_imuSensor = IMUSensor[_imuID];

	return true;
}
/******************************************************************************/




/******************************************************************************/
bool PrintIMUParameter(unsigned char _imuID)
{
	if(_imuID > NO_OF_IMU) { RtWprintf(L"\n>>> Wrong IMU ID(PrintIMUParameter)..!!"); return false; }
	else
	{
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> IMU(%d) parameter information", _imuID);
		RtWprintf(L"\n>>> Roll angle value: %d", (int)(IMUSensor[_imuID].Roll*1000.0f));
		RtWprintf(L"\n>>> Pitch angle value: %d", (int)(IMUSensor[_imuID].Pitch*1000.0f));
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> Roll angular velocity: %d", (int)(IMUSensor[_imuID].Roll_Velocity*1000.0f));
		RtWprintf(L"\n>>> Pitch angulat velocity: %d", (int)(IMUSensor[_imuID].Pitch_Velocity*1000.0f));
		RtWprintf(L"\n>>>");
		RtWprintf(L"\n>>> CAN channel number: %d", IMUSensor[_imuID].CAN_channel);
		
		GetIMUParameter(_imuID, &pSharedMemory->IMUsensor);

		return true;
	}
}
/******************************************************************************/




/******************************************************************************/
void ZMPInitialization(float _refZMP[2], float _zmp[], IMU _imu, FT _rightFT, FT _leftFT,  WALKING_INFO _walkingInfo[][4], JOINT _joint[], unsigned char _command)
{
	unsigned char i;
	float KI;
	float zmpTemp[2], heightTemp, pitchHipTemp;

	// _zmpInit[]
	// 0: torso x-direction
	// 1: torso y-direction
	// 2: z-direction of the foot
	// 3: hip pitch angle
	// 4: ankle pitch angle
	// 5: right ankle roll angle
	// 6: left ankle roll angle
	

	// ZMP initialization using the torso center position
	KI = 0.005f/INT_TIME;
	
	for(i=X ; i<=Y ; i++)
	{
		zmpTemp[i] = (_refZMP[i] - _zmp[i])/1000.0f;
		
		if(zmpTemp[i] > 0.05f) zmpTemp[i] = 0.05f;
		else if(zmpTemp[i] < -0.05f) zmpTemp[i] = -0.05f;
		
		if(_command == 0x00)
		{ 
			_walkingInfo[RIGHT][i].RefPosFF = 0.0f;
			_walkingInfo[LEFT][i].RefPosFF = 0.0f;
		}
		else
		{
			_walkingInfo[RIGHT][i].RefPosFF -= KI*zmpTemp[i];
			_walkingInfo[LEFT][i].RefPosFF -= KI*zmpTemp[i];
		}
	}



	// body roll angle initialization using the foot position(z-direction)
	KI = 0.004f/INT_TIME;

	heightTemp = _imu.Roll/1000.0f;
	
	if(heightTemp > 0.001f) heightTemp = 0.001f;
	else if(heightTemp < -0.001f) heightTemp = -0.001f;

	if(_command == 0x00)
	{
		_walkingInfo[RIGHT][Z].RefPosFF = 0.0f;
		_walkingInfo[LEFT][Z].RefPosFF = 0.0f;
	}
	else
	{
		_walkingInfo[RIGHT][Z].RefPosFF -= KI*heightTemp;
		_walkingInfo[LEFT][Z].RefPosFF += KI*heightTemp;
	}



	// body pitch angle initialization using the hip pitch angles
	KI = 0.01f;
	pitchHipTemp = _imu.Pitch;///1000.0f;
		
	if(pitchHipTemp > 0.01f) pitchHipTemp = 0.01f;
	else if(pitchHipTemp < -0.01f) pitchHipTemp = -0.01f;
	
	if(_command == 0x00)
	{
		//_joint[RAP].RefAngleFF = 0.0f;
		//_joint[LAP].RefAngleFF = 0.0f;
		_joint[RHP].RefAngleFF = 0.0f;
		_joint[LHP].RefAngleFF = 0.0f;
	}
	else
	{
		//_joint[RAP].RefAngleFF += KI*pitchHipTemp;
		//_joint[LAP].RefAngleFF += KI*pitchHipTemp;
		_joint[RHP].RefAngleFF += KI*pitchHipTemp;
		_joint[LHP].RefAngleFF += KI*pitchHipTemp;
	}



	
	// moment at ankle(My) initialization using ankle pitch angles
	//KI=0.0001f;
	KI = 0.0002f;
	if(_command == 0x00) _joint[RAP].RefAngleFF = 0.0f;
	else _joint[RAP].RefAngleFF += KI*(_rightFT.My-_leftFT.My);




	// moment at ankle(Mx) initialization using ankle roll angles
	KI = 0.02f/INT_TIME;
	if(_command == 0)
	{
		_joint[RAR].RefAngleFF = 0.0f;
		_joint[LAR].RefAngleFF = 0.0f;
	}
	else
	{
		_joint[RAR].RefAngleFF += KI*_rightFT.Mx;
		_joint[LAR].RefAngleFF += KI*_leftFT.Mx;
	}
}
/******************************************************************************/





/******************************************************************************/
void GotoWalkReadyPos(void)
{
	unsigned char i, j;
	float readyAngle[6];
	float readyPos[6] = {0.0f, 0.0f, WALK_READY_Z_POS, 0.0f, 0.0f, 0.0f};
//	float offsetAngleR[6] = {0.0f, -0.578579f, -23.849276f, 43.662514f, -20.464760f, -0.368447f};
//	float offsetAngleL[6] = {0.0f, -0.565438f, -23.483799f, 42.919666f, -20.214497f, 0.106651f};

	if(pSharedMemory->WalkReadyPosFlag==0)// Normal walk ready
	{		}
	else if(pSharedMemory->WalkReadyPosFlag==1)// Push Up
	{		}
	else if(pSharedMemory->WalkReadyPosFlag==2)// Grap Object on the ground
	{	readyPos[2] = WALK_READY_Z_POS + 0.37f;	}
	else if(pSharedMemory->WalkReadyPosFlag==3)// Grap Object on the ground
	{	readyPos[2] = WALK_READY_Z_POS + 0.37f;	}

	for(i=RIGHT ; i<=LEFT ; i++)
	{
		for(j=X ; j<=Yaw ; j++) WalkingInfo[i][j].RefPatternCurrent = readyPos[j];
	}

	InverseKinematics(readyPos, readyAngle);
//	printf("\n WRF= %d",pSharedMemory->WalkReadyPosFlag);
	if(pSharedMemory->WalkReadyPosFlag==0)// Normal walk ready
	{		
		SetMoveJointAngle(RSP, 10.0f, 2000.0f, 0x01);	Joint[RSP].WalkReadyAngle = 10.0f;
		SetMoveJointAngle(RSR, -10.0f, 2000.0f, 0x01);	Joint[RSR].WalkReadyAngle = -10.0f;
		SetMoveJointAngle(RSY, 0.0f, 2000.0f, 0x01);	Joint[RSY].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(REB, -30.0f, 2000.0f, 0x01);	Joint[REB].WalkReadyAngle = -30.0f;
		SetMoveJointAngle(RWY, 0.0f, 2000.0f, 0x01);	Joint[RWY].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(RWP, 0.0f, 2000.0f, 0x01);	Joint[RWP].WalkReadyAngle = 0.0f;
		
		SetMoveJointAngle(LSP, 10.0f, 2000.0f, 0x01);	Joint[LSP].WalkReadyAngle = 10.0f;
		SetMoveJointAngle(LSR, 10.0f, 2000.0f, 0x01);	Joint[LSR].WalkReadyAngle = 10.0f;
		SetMoveJointAngle(LSY, 0.0f, 2000.0f, 0x01);	Joint[LSY].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(LEB, -30.0f, 2000.0f, 0x01);	Joint[LEB].WalkReadyAngle = -30.0f;
		SetMoveJointAngle(LWY, 0.0f, 2000.0f, 0x01);	Joint[LWY].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(LWP, 0.0f, 2000.0f, 0x01);	Joint[LWP].WalkReadyAngle = 0.0f;
		
		SetMoveJointAngle(NKY, 0.0f, 2000.0f, 0x01);	Joint[NKY].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(NK1, 0.0f, 2000.0f, 0x01);	Joint[NK1].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(NK2, 0.0f, 2000.0f, 0x01);	Joint[NK2].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(WST, 0.0f, 2000.0f, 0x01);	Joint[WST].WalkReadyAngle = 0.0f;
	}
	else if(pSharedMemory->WalkReadyPosFlag==1)//Push Up
	{
		readyAngle[RHP] = readyAngle[RHP]-10.0f;
		readyAngle[RKN] = 0.0f;
		SetMoveJointAngle(RSP, -90.0f, 2000.0f, 0x01);	Joint[RSP].WalkReadyAngle = -90.0f;
		SetMoveJointAngle(RSR, 23.0f, 2000.0f, 0x01);	Joint[RSR].WalkReadyAngle = 23.0f;
		SetMoveJointAngle(RSY, -2.0f, 2000.0f, 0x01);	Joint[RSY].WalkReadyAngle = -2.0f;
		SetMoveJointAngle(REB, 0.0f, 2000.0f, 0x01);	Joint[REB].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(RWY, 11.0f, 2000.0f, 0x01);	Joint[RWY].WalkReadyAngle = 11.0f;
		SetMoveJointAngle(RWP, 0.0f, 2000.0f, 0x01);	Joint[RWP].WalkReadyAngle = 0.0f;
		
		SetMoveJointAngle(LSP, -90.0f, 2000.0f, 0x01);	Joint[LSP].WalkReadyAngle = -90.0f;
		SetMoveJointAngle(LSR, -23.0f, 2000.0f, 0x01);	Joint[LSR].WalkReadyAngle = -23.0f;
		SetMoveJointAngle(LSY, 4.0f, 2000.0f, 0x01);	Joint[LSY].WalkReadyAngle = 4.0f;
		SetMoveJointAngle(LEB, 0.0f, 2000.0f, 0x01);	Joint[LEB].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(LWY, 2.0f, 2000.0f, 0x01);	Joint[LWY].WalkReadyAngle = 2.0f;
		SetMoveJointAngle(LWP, 0.0f, 2000.0f, 0x01);	Joint[LWP].WalkReadyAngle = 0.0f;
		
		SetMoveJointAngle(NKY, 0.0f, 2000.0f, 0x01);	Joint[NKY].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(NK1, 0.0f, 2000.0f, 0x01);	Joint[NK1].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(NK2, 0.0f, 2000.0f, 0x01);	Joint[NK2].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(WST, 0.0f, 2000.0f, 0x01);	Joint[WST].WalkReadyAngle = 0.0f;
	}
	else if(pSharedMemory->WalkReadyPosFlag==2)//Grap Object on the ground
	{
		readyAngle[RHP] = readyAngle[RHP]-33.0f;
		readyAngle[RAP] = readyAngle[RAP]+6.3f;
		SetMoveJointAngle(RSP, -25.0f, 1500.0f, 0x01);	Joint[RSP].WalkReadyAngle = -25.0f;
		SetMoveJointAngle(RSR, -25.0f, 1500.0f, 0x01);	Joint[RSR].WalkReadyAngle = -25.0f;
		SetMoveJointAngle(RSY, 30.0f, 1500.0f, 0x01);	Joint[RSY].WalkReadyAngle = 30.0f;
		SetMoveJointAngle(REB, 0.0f, 2000.0f, 0x01);	Joint[REB].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(RWY, 0.0f, 2000.0f, 0x01);	Joint[RWY].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(RWP, 0.0f, 2000.0f, 0x01);	Joint[RWP].WalkReadyAngle = 0.0f;
		
		SetMoveJointAngle(LSP, 10.0f, 2000.0f, 0x01);	Joint[LSP].WalkReadyAngle = 10.0f;
		SetMoveJointAngle(LSR, 10.0f, 2000.0f, 0x01);	Joint[LSR].WalkReadyAngle = 10.0f;
		SetMoveJointAngle(LSY, 0.0f, 2000.0f, 0x01);	Joint[LSY].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(LEB, -30.0f, 2000.0f, 0x01);	Joint[LEB].WalkReadyAngle = -30.0f;
		SetMoveJointAngle(LWY, 0.0f, 2000.0f, 0x01);	Joint[LWY].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(LWP, 0.0f, 2000.0f, 0x01);	Joint[LWP].WalkReadyAngle = 0.0f;
		
		SetMoveJointAngle(NKY, -57.0f, 2000.0f, 0x01);	Joint[NKY].WalkReadyAngle = -57.0f;
		SetMoveJointAngle(NK1, 0.0f, 2000.0f, 0x01);	Joint[NK1].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(NK2, 0.0f, 2000.0f, 0x01);	Joint[NK2].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(WST, 57.0f, 2000.0f, 0x01);	Joint[WST].WalkReadyAngle = 57.0f;
	}
	else if(pSharedMemory->WalkReadyPosFlag==3)//Grap Object on the ground
	{
		readyAngle[RHP] = readyAngle[RHP]-20.0f;
		readyAngle[RAP] = readyAngle[RAP]+8.0f;
		SetMoveJointAngle(RSP, -25.0f, 1500.0f, 0x01);	Joint[RSP].WalkReadyAngle = -25.0f;
		SetMoveJointAngle(RSR, -25.0f, 1500.0f, 0x01);	Joint[RSR].WalkReadyAngle = -25.0f;
		SetMoveJointAngle(RSY, 30.0f, 1500.0f, 0x01);	Joint[RSY].WalkReadyAngle = 30.0f;
		SetMoveJointAngle(REB, 0.0f, 2000.0f, 0x01);	Joint[REB].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(RWY, 0.0f, 2000.0f, 0x01);	Joint[RWY].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(RWP, 0.0f, 2000.0f, 0x01);	Joint[RWP].WalkReadyAngle = 0.0f;
		
		SetMoveJointAngle(LSP, 10.0f, 2000.0f, 0x01);	Joint[LSP].WalkReadyAngle = 10.0f;
		SetMoveJointAngle(LSR, 10.0f, 2000.0f, 0x01);	Joint[LSR].WalkReadyAngle = 10.0f;
		SetMoveJointAngle(LSY, 0.0f, 2000.0f, 0x01);	Joint[LSY].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(LEB, -30.0f, 2000.0f, 0x01);	Joint[LEB].WalkReadyAngle = -30.0f;
		SetMoveJointAngle(LWY, 0.0f, 2000.0f, 0x01);	Joint[LWY].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(LWP, 0.0f, 2000.0f, 0x01);	Joint[LWP].WalkReadyAngle = 0.0f;
		
		SetMoveJointAngle(NKY, 0.0f, 2000.0f, 0x01);	Joint[NKY].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(NK1, 0.0f, 2000.0f, 0x01);	Joint[NK1].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(NK2, 0.0f, 2000.0f, 0x01);	Joint[NK2].WalkReadyAngle = 0.0f;
		SetMoveJointAngle(WST, 0.0f, 2000.0f, 0x01);	Joint[WST].WalkReadyAngle = 0.0f;
	}
	

	for(i=RHY ; i<=RAR ; i++)
	{
		SetMoveJointAngle(i, readyAngle[i], 2000.0f, 0x01);
		Joint[i].WalkReadyAngle = readyAngle[i];
		
		//SetMoveJointAngle(i, offsetAngleR[i], 2000.0f, 0x01);
		//Joint[i].WalkReadyAngle = offsetAngleR[i];
		//Joint[i].RefAngleFF = offsetAngleR[i]-readyAngle[i];
	}
	for(i=LHY ; i<=LAR ; i++)
	{
		SetMoveJointAngle(i, readyAngle[i-LHY], 2000.0f, 0x01);
		Joint[i].WalkReadyAngle = readyAngle[i-LHY];
		
		//SetMoveJointAngle(i, offsetAngleL[i-LHY], 2000.0f, 0x01);
		//Joint[i].WalkReadyAngle = readyAngle[i-LHY];
		//Joint[i].RefAngleFF = offsetAngleL[i-LHY] - readyAngle[i-LHY];
	}
	_WalkReadyAngleSetFlag = 1;
}
/******************************************************************************/




/******************************************************************************/
void GotoHomePos(void)
{
	unsigned char i, j;
	float homePos[6] = {0.0f, 0.0f, HOME_Z_POS, 0.0f, 0.0f, 0.0f};

	// clear ZMP initialization values
	ZMPInitialization(InitZMP, ZMP, IMUSensor[CENTERIMU], FTSensor[RFFT], FTSensor[LFFT], WalkingInfo, Joint, 0);
	DSPControl(ZMP, InitZMP, WalkingInfo, 0.0f, 0x00, 0x00);

	for(i=RHY ; i<=LAR ; i++)
	{
		SetMoveJointAngle(i, 0.0f, 3000.0f, 0x01); 
		Joint[i].ControlDampAngleCurrent = 0.0f;
		Joint[i].ControlDampAnglePast = 0.0f;
		Joint[i].ControlVibrationAngle = 0.0f; 
		Joint[i].ControlAngleFF = 0.0f;
		Joint[i].ControlAngleFFDelta = 0.0f;
		Joint[i].ControlAngleFFToGo = 0.0f;
		Joint[i].RefAngleFF = 0.0f;
	}
	
	SetMoveJointAngle(RSP, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(RSR, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(RSY, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(REB, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(RWY, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(RWP, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(LSP, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(LSR, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(LSY, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(LEB, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(LWY, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(LWP, 0.0f, 1500.0f, 0x01);
	
	SetMoveJointAngle(WST, 0.0f, 1500.0f, 0x01);
	
	SetMoveJointAngle(NKY, 0.0f, 1500.0f, 0x01);	
	
	
	for(i=X ; i<=Yaw ; i++)
	{
		WalkingInfo[RIGHT][i].RefPosFF = 0.0f; WalkingInfo[RIGHT][i].ControlDSPZMP = 0.0f;
		WalkingInfo[LEFT][i].RefPosFF = 0.0f; WalkingInfo[LEFT][i].ControlDSPZMP = 0.0f;
	}

	for(i=RIGHT ; i<=LEFT ; i++)
	{
		for(j=X ; j<=Yaw ; j++) WalkingInfo[i][j].RefPatternCurrent = homePos[j];
	}
}
/******************************************************************************/



/******************************************************************************/
void SetDRCHomePos(void)  // by Inhyeok
{
	SetMoveJointAngle(RHY, 0.f, 3000.f, 0x01);
	SetMoveJointAngle(RHR, DRC_BR_RHR, 3000.f, 0x01);
	SetMoveJointAngle(RHP, 0.f, 3000.f, 0x01);
	SetMoveJointAngle(RKN, 0.f, 3000.f, 0x01);
	SetMoveJointAngle(RAP, 0.f, 3000.f, 0x01);
	SetMoveJointAngle(RAR, DRC_BR_RAR, 3000.f, 0x01);

	SetMoveJointAngle(LHY, 0.f, 3000.f, 0x01);
	SetMoveJointAngle(LHR, DRC_BR_LHR, 3000.f, 0x01);
	SetMoveJointAngle(LHP, 0.f, 3000.f, 0x01);
	SetMoveJointAngle(LKN, 0.f, 3000.f, 0x01);
	SetMoveJointAngle(LAP, 0.f, 3000.f, 0x01);
	SetMoveJointAngle(LAR, DRC_BR_LAR, 3000.f, 0x01);
	
	SetMoveJointAngle(RSP, 0.0f, 3000.f, 0x01);
	SetMoveJointAngle(RSR, 0.0f, 3000.f, 0x01);
	SetMoveJointAngle(RSY, 0.0f, 3000.f, 0x01);
	SetMoveJointAngle(REB, 0.0f, 3000.f, 0x01);
	SetMoveJointAngle(RWY, 0.0f, 3000.f, 0x01);
	
	SetMoveJointAngle(LSP, 0.0f, 3000.f, 0x01);
	SetMoveJointAngle(LSR, 0.0f, 3000.f, 0x01);
	SetMoveJointAngle(LSY, 0.0f, 3000.f, 0x01);
	SetMoveJointAngle(LEB, 0.0f, 3000.f, 0x01);
	SetMoveJointAngle(LWY, 0.0f, 3000.f, 0x01);
	
	if(pSharedMemory->change_drc_hand_flag != DRC_STICK_MODE)
	{
		SetMoveJointAngle(RWP, 0.0f, 3000.f, 0x01);
		SetMoveJointAngle(LWP, 0.0f, 3000.f, 0x01);
		SetMoveJointAngle(RWY2, 0.0f, 3000.f, 0x01);
		SetMoveJointAngle(LWY2, 0.0f, 3000.f, 0x01);
	}
	
	SetMoveJointAngle(WST, 0.0f, 3000.f, 0x01);
	
	SetMoveJointAngle(NKY, 0.0f, 3000.f, 0x01);		
}
/******************************************************************************/





/******************************************************************************/
void GotoControlOffPos(void)
{
	unsigned char i, j;
	float readyPos[6] = {0.0f, 0.0f, WALK_READY_Z_POS+0.05f, 0.0f, 0.0f, 0.0f};
	
	float tempDelay[2] = {0.0f, 0.0f};
	float userData[5];
	for(i=RIGHT ; i<=LEFT ; i++)
	{
		for(j=X ; j<=Yaw ; j++) SetMoveTaskPos(&WalkingInfo[i][j], readyPos[j], 1500.0f, tempDelay, userData, 0x01, 0x00);
	}

	
	SetMoveJointAngle(RSP, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(RSR, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(RSY, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(REB, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(LSP, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(LSR, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(LEB, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(WST, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(NKY, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(NK1, -25.0f, 1500.0f, 0x01);
	SetMoveJointAngle(NK1, -25.0f, 1500.0f, 0x01);
}
/******************************************************************************/





/******************************************************************************/
void GotoControlOnPos(void)
{
	unsigned char i, j;
	float readyPos[6] = {0.0f, 0.0f, WALK_READY_Z_POS, 0.0f, 0.0f, 0.0f};
	
	float tempDelay[2] = {0.0f, 0.0f};
	float userData[5];
	for(i=RIGHT ; i<=LEFT ; i++)
	{
		for(j=X ; j<=Yaw ; j++) SetMoveTaskPos(&WalkingInfo[i][j], readyPos[j], 1500.0f, tempDelay, userData, 0x01, 0x00);
	}
	
	SetMoveJointAngle(RSP, 10.0f, 1500.0f, 0x01);
	SetMoveJointAngle(RSR, -10.0f, 1500.0f, 0x01);
	SetMoveJointAngle(RSY, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(REB, -30.0f, 1500.0f, 0x01);

	SetMoveJointAngle(LSP, 10.0f, 1500.0f, 0x01);
	SetMoveJointAngle(LSR, 10.0f, 1500.0f, 0x01);
	SetMoveJointAngle(LSY, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(LEB, -30.0f, 1500.0f, 0x01);
	SetMoveJointAngle(WST, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(NKY, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(NK1, 0.0f, 1500.0f, 0x01);
	SetMoveJointAngle(NK1, 0.0f, 1500.0f, 0x01);
}
/******************************************************************************/



/******************************************************************************/
unsigned char LandingStateCheck(FT _ftSensor[], WALKING_INFO _walkingInfo[][4])
{	
	if( (_walkingInfo[RIGHT][Z].CurrentSequence==RIGHT_FOOT_DOWN_SET) || (_walkingInfo[RIGHT][Z].CurrentSequence==RIGHT_FOOT_DOWN_FORWARD_SET) )
	{
		if( (_walkingInfo[RIGHT][Z].MoveFlag==true) && (_ftSensor[RFFT].Fz>(float)(pSharedMemory->LandingFz)) ) return RIGHT_FOOT_EARLY_LANDING;
		else if( (_walkingInfo[RIGHT][Z].GoalTimeCount==_walkingInfo[RIGHT][Z].CurrentTimeCount) && (_ftSensor[RFFT].Fz<(float)(pSharedMemory->LandingFz)) ) return RIGHT_FOOT_LATE_LANDING;
		else return NORMAL_LANDING;
	}
	else if( (_walkingInfo[LEFT][Z].CurrentSequence==LEFT_FOOT_DOWN_SET) || (_walkingInfo[LEFT][Z].CurrentSequence==LEFT_FOOT_DOWN_FORWARD_SET) )
	{
		if( (_walkingInfo[LEFT][Z].MoveFlag==true) && (_ftSensor[LFFT].Fz>(float)(pSharedMemory->LandingFz)) ) return LEFT_FOOT_EARLY_LANDING;
		else if( (_walkingInfo[LEFT][Z].GoalTimeCount==_walkingInfo[LEFT][Z].CurrentTimeCount) && (_ftSensor[LFFT].Fz<(float)(pSharedMemory->LandingFz)) ) return LEFT_FOOT_LATE_LANDING;
		else return NORMAL_LANDING;
	}
	else return NORMAL_LANDING;
}
/******************************************************************************/





/******************************************************************************/
void DSPControl(float _ZMP[], float _refZMP[2], WALKING_INFO _walkingInfo[][4], float _time, unsigned char _command, unsigned char _demo)
{
	unsigned char i;
	unsigned char Command = _command;
	static float x1new[2] = {0.0f, 0.0f}, x2new[2] = {0.0f, 0.0f};
	static float x1[2] = {0.0f, 0.0f}, x2[2] = {0.0f, 0.0f};
	static long ControlTime = (long)(_time/INT_TIME);
	float delZMP[2] = {_ZMP[0]-_refZMP[0], _ZMP[1]-_refZMP[1]};
	static float gainOveriding = 1.0f;
	float controlOutput[2];
	float dspLimit = 100.0f;
	float gain = dspGain;

	const float adm[2][4] = {0.519417298104f,	-2.113174135817f,	0.003674390567f,	0.994144954175f,	0.734481292627f,	-1.123597885586f,	0.004304846434f,	0.997046817458f};
	const float bdm[2][2] = {0.003674390567f,	0.000010180763f,	0.004304846434f,	0.000011314544f};
	const float cdm[2][2] = {9.698104154920f,	-126.605319064208f,	-0.555683927701f,	-77.652283056185f};
	

	if( (ControlTime==0) && (Command==0x01) ) Command = 0x03;

	switch(Command)
	{
	case 0x00:
		 for(i=X ; i<=Y ; i++) x1[i] = x2[i] = x1new[i] = x2new[i] = controlOutput[i] = 0.0f;
		break;
	case 0x01:
		for(i=X ; i<=Y ; i++)
		{
			x1new[i] = adm[i][0]*x1[i] + adm[i][1]*x2[i] + bdm[i][0]*delZMP[i];
			x2new[i] = adm[i][2]*x1[i] + adm[i][3]*x2[i] + bdm[i][1]*delZMP[i];
			controlOutput[i] = cdm[i][0]*x1new[i] + cdm[i][1]*x2new[i];
			
			x1[i] = x1new[i];	x2[i] = x2new[i];
			
			if(controlOutput[i] > dspLimit) controlOutput[i] = dspLimit;
			else if(controlOutput[i] < -dspLimit) controlOutput[i] = -dspLimit;
			else;
			
			_walkingInfo[RIGHT][i].ControlDSPZMP = gainOveriding*gain*controlOutput[i]/1000.0f;
			_walkingInfo[LEFT][i].ControlDSPZMP = gainOveriding*gain*controlOutput[i]/1000.0f;
		}
		ControlTime--;
		gainOveriding += 0.05f;
		if(gainOveriding > 1.0f) gainOveriding = 1.0f;
		break;
	case 0x02:
		ControlTime = (long)(_time	/INT_TIME);
		//gainOveriding = 0.0f;
		break;
	case 0x03:
		for(i=X ; i<=Y ; i++)
		{
			_walkingInfo[RIGHT][i].ControlDSPZMP *= 0.9f;
			_walkingInfo[LEFT][i].ControlDSPZMP *= 0.9f;
		}		
		gainOveriding = 0.0f;
		break;
	}

	if(_demo == 0x01)
	{
		// ZMP initialization using the torso center position
		float KI = 0.005f/INT_TIME;
		float zmpTemp[2];

		for(i=X ; i<=Y ; i++)
		{
			zmpTemp[i] = (_refZMP[i] - _ZMP[i])/1000.0f;
			
			if(zmpTemp[i] > 0.05f) zmpTemp[i] = 0.05f;
			else if(zmpTemp[i] < -0.05f) zmpTemp[i] = -0.05f;
			
			if(_command == 0x00)
			{ 
				_walkingInfo[RIGHT][i].RefPosFF = 0.0f;
				_walkingInfo[LEFT][i].RefPosFF = 0.0f;
			}
			else
			{
				_walkingInfo[RIGHT][i].RefPosFF -= KI*zmpTemp[i];
				_walkingInfo[LEFT][i].RefPosFF -= KI*zmpTemp[i];
			}
		}
	}

}
/******************************************************************************/





/******************************************************************************/
void DampingControl(IMU _imuSensor[], JOINT _joint[], unsigned char _command)
{
	float tempControlAngle[2];
	float limitAngle = 20.0f;
	float gain[6];
	float controlDampCutoff = 1.0f;

	// without case(pitch	roll)
// 	gain[0] = 0.2f;		gain[3] = 0.2f;
// 	gain[1] = 1.0f;		gain[4] = 1.0f;
//	gain[2] = 0.4f;		gain[5] = 0.5f;
	// with case
// 	gain[0] = 0.3f;		gain[3] = 0.3f;
// 	gain[1] = 1.0f;		gain[4] = 1.0f;
// 	gain[2] = 0.4f;		gain[5] = 0.5f;

	unsigned char i;
	for(i=0 ; i<6 ; i++) gain[i] = dampingGain[i];
	
	tempControlAngle[0] = gain[0]*(-gain[1]*_imuSensor[CENTERIMU].Pitch + gain[2]*_imuSensor[CENTERIMU].Pitch_Velocity);
	tempControlAngle[1] = gain[3]*(-gain[4]*_imuSensor[CENTERIMU].Roll + gain[5]*_imuSensor[CENTERIMU].Roll_Velocity);

	switch(_command)
	{
	case RIGHT_SSP:
		_joint[RAP].ControlDampAngleCurrent = (float)((1.0f-2.0f*PI*controlDampCutoff*INT_TIME/1000.0f)*_joint[RAP].ControlDampAnglePast+2.0f*PI*controlDampCutoff*INT_TIME/1000.0f*tempControlAngle[0]);
		_joint[RAP].ControlDampAnglePast = _joint[RAP].ControlDampAngleCurrent;
		_joint[RAR].ControlDampAngleCurrent = (float)((1.0f-2.0f*PI*controlDampCutoff*INT_TIME/1000.0f)*_joint[RAR].ControlDampAnglePast+2.0f*PI*controlDampCutoff*INT_TIME/1000.0f*tempControlAngle[1]);
		_joint[RAR].ControlDampAnglePast = _joint[RAR].ControlDampAngleCurrent;
		break;
	case LEFT_SSP:
		_joint[LAP].ControlDampAngleCurrent = (float)((1.0f-2.0f*PI*controlDampCutoff*INT_TIME/1000.0f)*_joint[LAP].ControlDampAnglePast+2.0f*PI*controlDampCutoff*INT_TIME/1000.0f*tempControlAngle[0]);
		_joint[LAP].ControlDampAnglePast = _joint[LAP].ControlDampAngleCurrent;
		_joint[LAR].ControlDampAngleCurrent = (float)((1.0f-2.0f*PI*controlDampCutoff*INT_TIME/1000.0f)*_joint[LAR].ControlDampAnglePast+2.0f*PI*controlDampCutoff*INT_TIME/1000.0f*tempControlAngle[1]);
		_joint[LAR].ControlDampAnglePast = _joint[LAR].ControlDampAngleCurrent;
		break;
	case DSP:
	default:
		tempControlAngle[0] = 0.0f;
		tempControlAngle[1] = 0.0f;
		_joint[RAP].ControlDampAngleCurrent = (float)((1.0f-2.0f*PI*controlDampCutoff*INT_TIME/1000.0f)*_joint[RAP].ControlDampAnglePast+2.0f*PI*controlDampCutoff*INT_TIME/1000.0f*tempControlAngle[0]);
		_joint[RAP].ControlDampAnglePast = _joint[RAP].ControlDampAngleCurrent;
		_joint[RAR].ControlDampAngleCurrent = (float)((1.0f-2.0f*PI*controlDampCutoff*INT_TIME/1000.0f)*_joint[RAR].ControlDampAnglePast+2.0f*PI*controlDampCutoff*INT_TIME/1000.0f*tempControlAngle[1]);
		_joint[RAR].ControlDampAnglePast = _joint[RAR].ControlDampAngleCurrent;
		_joint[LAP].ControlDampAngleCurrent = (float)((1.0f-2.0f*PI*controlDampCutoff*INT_TIME/1000.0f)*_joint[LAP].ControlDampAnglePast+2.0f*PI*controlDampCutoff*INT_TIME/1000.0f*tempControlAngle[0]);
		_joint[LAP].ControlDampAnglePast = _joint[LAP].ControlDampAngleCurrent;
		_joint[LAR].ControlDampAngleCurrent = (float)((1.0f-2.0f*PI*controlDampCutoff*INT_TIME/1000.0f)*_joint[LAR].ControlDampAnglePast+2.0f*PI*controlDampCutoff*INT_TIME/1000.0f*tempControlAngle[1]);
		_joint[LAR].ControlDampAnglePast = _joint[LAR].ControlDampAngleCurrent;
		break;
	}

	if(_joint[RAP].ControlDampAngleCurrent > limitAngle) _joint[RAP].ControlDampAngleCurrent = limitAngle;
	else if(_joint[RAP].ControlDampAngleCurrent < -limitAngle) _joint[RAP].ControlDampAngleCurrent = -limitAngle;
	if(_joint[LAP].ControlDampAngleCurrent > limitAngle) _joint[LAP].ControlDampAngleCurrent = limitAngle;
	else if(_joint[LAP].ControlDampAngleCurrent < -limitAngle) _joint[LAP].ControlDampAngleCurrent = -limitAngle;
	
	if(_joint[RAR].ControlDampAngleCurrent > limitAngle) _joint[RAR].ControlDampAngleCurrent = limitAngle;
	else if(_joint[RAR].ControlDampAngleCurrent < -limitAngle) _joint[RAR].ControlDampAngleCurrent = -limitAngle;
	if(_joint[LAR].ControlDampAngleCurrent > limitAngle) _joint[LAR].ControlDampAngleCurrent = limitAngle;
	else if(_joint[LAR].ControlDampAngleCurrent < -limitAngle) _joint[LAR].ControlDampAngleCurrent = -limitAngle;
}
/******************************************************************************/





/******************************************************************************/
void VibrationControl(FT _ftSensor[], JOINT _joint[], unsigned char _command)
{	
	float temp, limit;

	static float X_Roll_R[2] = {0.0f, 0.0f};
	static float X_Roll_New_R[2] = {0.0f, 0.0f};
	static float Y_Roll_R = 0.0f;

	static float X_Roll_L[2] = {0.0f, 0.0f};
	static float X_Roll_New_L[2] = {0.0f, 0.0f};
	static float Y_Roll_L = 0.0f;

	const float A_Roll[4] = {0.259455231689f, -3.299530027401f, 0.005542044942f, 0.979921074166f}; 
	const float B_Roll[2] = {0.005542044942f, 0.000033725503f}; 
	const float C_Roll[2] = {4.457995647858f, -29.104042013618f};

	limit = 5.0f;

	switch(_command)
	{
	case LEFT_SSP:
		temp = DEG2RAD*FTSensor[RFFT].VelRoll/FTSensor[RFFT].SF_Roll;
		X_Roll_New_R[0] = A_Roll[0]*X_Roll_R[0] + A_Roll[1]*X_Roll_R[1] + B_Roll[0]*temp;
		X_Roll_New_R[1] = A_Roll[2]*X_Roll_R[0] + A_Roll[3]*X_Roll_R[1] + B_Roll[1]*temp;
		Y_Roll_R = C_Roll[0]*X_Roll_New_R[0] + C_Roll[1]*X_Roll_New_R[1]; 
		
		X_Roll_R[0] = X_Roll_New_R[0];	X_Roll_R[1] = X_Roll_New_R[1];
		
		temp = -1.5f*10.0f*RAD2DEG*Y_Roll_R;
		if(temp > limit) temp = limit;
		else if(temp < -limit) temp = -limit;
	
		_joint[RHR].ControlVibrationAngle = temp;
		break;
	case RIGHT_SSP:
		temp = DEG2RAD*FTSensor[LFFT].VelRoll/FTSensor[LFFT].SF_Roll;
		X_Roll_New_L[0] = A_Roll[0]*X_Roll_L[0] + A_Roll[1]*X_Roll_L[1] + B_Roll[0]*temp;
		X_Roll_New_L[1] = A_Roll[2]*X_Roll_L[0] + A_Roll[3]*X_Roll_L[1] + B_Roll[1]*temp;
		Y_Roll_L = C_Roll[0]*X_Roll_New_L[0] + C_Roll[1]*X_Roll_New_L[1];  
		
		X_Roll_L[0] = X_Roll_New_L[0];	X_Roll_L[1] = X_Roll_New_L[1];
		
		temp = -1.5f*10.0f*RAD2DEG*Y_Roll_L;
		if(temp > limit) temp = limit;
		else if(temp < -limit) temp = -limit;
	
		_joint[LHR].ControlVibrationAngle = temp;
		break;
	case DSP:
		_joint[RHR].ControlVibrationAngle *= 0.9f;
		_joint[LHR].ControlVibrationAngle *= 0.9f;
		break;
	}
}
/******************************************************************************/







/******************************************************************************/
void UnevenControl()
{
	static unsigned char Rstate = 0x00;
	static unsigned char Lstate = 0x00;
	//printf("1");
	float tempPeriod		= 800.0f;
	float tempDelayTime[2];//= {0.0f ,0.0f};
	float tempDelayY = pSharedMemory->JW_temp[3];	// 60.0f;
	float tempDSP = pSharedMemory->JW_temp[15];
	if(WalkingStartStop == 0x02)
	{
		if((WalkingInfo[LEFT][Z].RefPatternCurrent > -0.52)&&(WalkingPhaseNext[Z] == LEFT_FOOT_DOWN_SET))
		{
			//Uneven Landing on outside of Right Foot		
			if((FTSensor[RFFT].Mx<-15)&&(Rstate == 0x00))
			{
				tempDelayTime[0] = 0.0f;
				tempDelayTime[1] = 0.0f;
				SetMoveTaskPosJointFF(&Joint[RHR], -0.5f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				Rstate = 0x01;
			}
			else if((FTSensor[RFFT].Mx<-20)&&(Rstate == 0x01))
			{
				tempDelayTime[0] = 0.0f;
				tempDelayTime[1] = 0.0f;
				SetMoveTaskPosJointFF(&Joint[RHR], -1.8f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				Rstate = 0x02;
			}
			else if((FTSensor[RFFT].Mx>15)&&(Rstate == 0x00))//Uneven Landing on inside of Right Foot
			{
				tempDelayTime[0] = 0.0f;
				tempDelayTime[1] = 0.0f;
				SetMoveTaskPosJointFF(&Joint[RHR], 0.5f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				Rstate = 0x01;
			}

			//Go LeftFoot Home 
			if((Lstate == 0x01)||(Lstate == 0x02)||(Lstate == 0x03)||(Lstate == 0x04))
			{
				tempDelayTime[0] = 0.0f;
				tempDelayTime[1] = tempDSP+20.0f;
				SetMoveTaskPosJointFF(&Joint[LHR], 0.0f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				Lstate = 0x00;
			}
		}
		else if((WalkingInfo[LEFT][Z].RefPatternCurrent > -0.52)&&(WalkingPhaseNext[Z] == RIGHT_FOOT_UP_SET))
		{
			//Falling Landing on outside of Right Foot 
			if((FTSensor[RFFT].Mx<-10)&&((Rstate == 0x01)||(Rstate == 0x02)))
			{
				tempDelayTime[0] = 0.0f;
				tempDelayTime[1] = tempDSP+30.0f;
				SetMoveTaskPosJointFF(&Joint[RHR], 0.5f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				Rstate = 0x03;
			}
			else if((FTSensor[RFFT].Mx<-15)&&((Rstate == 0x01)||(Rstate == 0x02)||(Rstate == 0x03)))
			{
				tempDelayTime[0] = 0.0f;
				tempDelayTime[1] = tempDSP+30.0f;
				SetMoveTaskPosJointFF(&Joint[RHR], 0.8f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				Rstate = 0x04;
			}

		}
		else if((WalkingInfo[RIGHT][Z].RefPatternCurrent> -0.52)&&(WalkingPhaseNext[Z] == RIGHT_FOOT_DOWN_SET))
		{
			//Uneven Landing on outside of Left Foot
			if((FTSensor[LFFT].Mx>15)&&(Lstate == 0x00))
			{
				tempDelayTime[0] = 0.0f;
				tempDelayTime[1] = 0.0f;
				SetMoveTaskPosJointFF(&Joint[LHR], 0.5f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				Lstate = 0x01;
			}
			else if((FTSensor[LFFT].Mx>20)&&(Lstate == 0x01))
			{
				tempDelayTime[0] = 0.0f;
				tempDelayTime[1] = 0.0f;
				SetMoveTaskPosJointFF(&Joint[LHR], 1.8f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				Lstate = 0x02;
			}
			else if((FTSensor[LFFT].Mx<-15)&&(Lstate == 0x00))//Uneven Landing on inside of Left Foot
			{
				tempDelayTime[0] = 0.0f;
				tempDelayTime[1] = 0.0f;
				SetMoveTaskPosJointFF(&Joint[LHR], -0.5f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				Lstate = 0x01;
			}

			//Go Right Foot Home 
			if((Rstate == 0x01)||(Rstate == 0x02)||(Rstate == 0x03)||(Rstate == 0x04))
			{
				tempDelayTime[0] = 0.0f;
				tempDelayTime[1] = tempDSP+20.0f;
				SetMoveTaskPosJointFF(&Joint[RHR], 0.0f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				Rstate = 0x00;
			}

		}
		else if((WalkingInfo[RIGHT][Z].RefPatternCurrent> -0.52)&&(WalkingPhaseNext[Z] == LEFT_FOOT_UP_SET))
		{
			//Falling Landing on outside of Left Foot
			if((FTSensor[LFFT].Mx>10)&&((Lstate == 0x01)||(Lstate == 0x02)))
			{
				tempDelayTime[0] = 0.0f;
				tempDelayTime[1] = tempDSP+30.0f;
				SetMoveTaskPosJointFF(&Joint[LHR], -0.5f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				Lstate = 0x03;
			}
			else if((FTSensor[LFFT].Mx>15)&&((Lstate == 0x01)||(Lstate == 0x02)||(Lstate == 0x03)))
			{
				tempDelayTime[0] = 0.0f;
				tempDelayTime[1] = tempDSP+30.0f;
				SetMoveTaskPosJointFF(&Joint[LHR], -0.8f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				Lstate = 0x04;
			}
			
		}
	}
	else
	{
		if((Rstate == 0x01)||(Rstate == 0x02)||(Rstate == 0x03)||(Rstate == 0x04))
		{
			tempDelayTime[0] = 0.0f;
			tempDelayTime[1] = tempDSP+20.0f;
			SetMoveTaskPosJointFF(&Joint[RHR], 0.0f, tempPeriod/2.0f, tempDelayTime, 1, 0);
			Rstate = 0x00;
		}
		if((Lstate == 0x01)||(Lstate == 0x02)||(Lstate == 0x03)||(Lstate == 0x04))
		{
			tempDelayTime[0] = 0.0f;
			tempDelayTime[1] = tempDSP+20.0f;
			SetMoveTaskPosJointFF(&Joint[LHR], 0.0f, tempPeriod/2.0f, tempDelayTime, 1, 0);
			Lstate = 0x00;
		}
	}
}
/******************************************************************************/



/******************************************************************************/
void main(void)
{
	int sw_mode_count = 0; // inhyeok

	RtxInit();
	InitParameters();
	StartCAN();
	InitGlobalMotionVariables();	// by Inhyeok
	LoadControllerParamter();		// by Inhyeok
	LoadControllerParamter_DRC(DRC_BI_MODE);		// by Inhyeok
	_WBDemoIsWaitingFlag = 0;		// by Inhyeok	
	_WalkReadyAngleSetFlag = 0;     // by Inhyeok
	pSharedMemory->LogRing.bOverflow = 1;	// by Inhyeok

	printf("\n Using CAN-RING");

	InitCANRing();

	while(true)
	{		
		RtSleep(1);
		
		CheckBoardStatus(Joint);

		switch(pSharedMemory->CommandFlag)
		{
			unsigned int i;
			FILE* zmpinit_file;
			
		case EXIT_PROGRAM:	// exit program
			FreeGlobalMotionVariables();	// by Inhyeok
			RtxEnd();				
			return;
			break;
		case CAN_CH0_TX:	// CAN message from win32 app. transmitting procedure using channel 0
			PushCANMsg(CAN0, pSharedMemory->Tx_ID0, pSharedMemory->Tx_Data0, pSharedMemory->Tx_DLC0, 0);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case CAN_CH1_TX:	// CAN message from win32 app. transmitting procedure using channel 1
			PushCANMsg(CAN1, pSharedMemory->Tx_ID1, pSharedMemory->Tx_Data1, pSharedMemory->Tx_DLC1, 0);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_MOTOR_GAIN:
			//Joint[pSharedMemory->JointID].
			//SetMotorGainSetting(Joint[i]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_CURRENT_GAIN:
			//for(i=RHY ; i<NO_OF_JOINT ; i++) MotorGainSetting(Joint[i], 0x01);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case LOAD_PARAMETER:	// load parameters from the file.
			LoadParameter();
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SAVE_PARAMETER:	// save parameters to the file
			SaveParameter();
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_JOINT_PARAMETER:	// Set joint parameter
			SetJointParameter(pSharedMemory->JointID, pSharedMemory->Joint);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GET_JOINT_PARAMETER:	// Send joint parameter back to the win32 program
			GetJointParameter(pSharedMemory->JointID, &pSharedMemory->Joint);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case PRINT_JOINT_PARAMETER:	// print out current joint parameter
			PrintJointParameter(pSharedMemory->JointID);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case CHECK_DEVICE:	// Check each sub-controller's CAN communication and set it's initial parameters
			for(i=JMC0 ; i<=JMC11 ; i++) CheckDeviceCAN(i);
			
			CheckDeviceCAN(EJMC0);
			CheckDeviceCAN(EJMC1);
			CheckDeviceCAN(EJMC2);
			CheckDeviceCAN(EJMC3);
			CheckDeviceCAN(EJMC4); // right finger
			CheckDeviceCAN(EJMC5); // left finger

			CheckDeviceCAN(FT0);
			CheckDeviceCAN(FT1);
			CheckDeviceCAN(FT2);
			CheckDeviceCAN(FT3);
			CheckDeviceCAN(IMU0);
			// Comport Open - CDI
			//if(pSharedMemory->COM_Ready_Flag==false) { RtWprintf(L"\n>>> Cannot Open Comport..!! "); }
			//else { RtWprintf(L"\n>>> ComPort Open OK..!! "); }
	
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GAIN_SETTING:	// motor controller gain setting
			for(i=RHY ; i<WST ; i++) GainSetting(i);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case ENABLE_FET:	// motor controller FET driver enable
			for(i=JMC0 ; i<=JMC11 ; i++) FETDriverOnOff(i, 0x01);
			for(i=EJMC0 ; i<=EJMC5 ; i++) FETDriverOnOff(i, 0x01);
			ReadSensorFlag = 0x01;			
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case ENABLE_FET_EACH:	// motor controller FET driver enable (each board)
			FETDriverOnOff(pSharedMemory->BoardID, 0x01);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case DISABLE_FET:	// motor controller FET driver disable
			for(i=JMC0 ; i<=JMC11 ; i++) FETDriverOnOff(i, 0);
			for(i=EJMC0 ; i<=EJMC5 ; i++) FETDriverOnOff(i, 0);
			ReadSensorFlag = 0x00;
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case DISABLE_FET_EACH:	// motor controller FET driver disable (each board)
			FETDriverOnOff(pSharedMemory->BoardID, 0x00);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case RUN_CMD:		// motor controller control-on
			for(i=JMC0 ; i<=JMC11 ; i++) { ZeroEncoder(i);	SendRunStopCMD(i, 0x01); }
			for(i=EJMC0 ; i<=EJMC5 ; i++) { ZeroEncoder(i);	SendRunStopCMD(i, 0x01); }
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case RUN_CMD_EACH:	// motor controller control-on (each board)
			ZeroEncoder(pSharedMemory->BoardID);
			SendRunStopCMD(pSharedMemory->BoardID, 0x01);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case STOP_CMD:		// motor controller control-off
			for(i=JMC0 ; i<=JMC11 ; i++) SendRunStopCMD(i, 0x00);
			for(i=EJMC0 ; i<=EJMC5 ; i++) SendRunStopCMD(i, 0x00);
			pSharedMemory->CommandFlag = NO_ACT;
		case STOP_CMD_EACH:	// motor controller control-off (each board)
			SendRunStopCMD(pSharedMemory->BoardID, 0x00);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_LIMIT_POS:	// go to limit sensor and initial position
			GoToLimitPos(pSharedMemory->JointID);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_LIMIT_POS_UPPER_ALL:
			GoToLimitPosAll(0x02);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_LIMIT_POS_LOWER_ALL:
			GoToLimitPosAll(0x01);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_LIMIT_POS_ALL:
			GoToLimitPosAll(0x00);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case ENCODER_ZERO:	// set encode value as zero
			for(i=JMC0 ; i<=JMC11 ; i++) ZeroEncoder(i);
			for(i=EJMC0 ; i<=EJMC5 ; i++) ZeroEncoder(i);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case ENCODER_ZERO_EACH:	// set encoder value as zero (each board)
			ZeroEncoder(pSharedMemory->BoardID);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SAVE_ZMP_INIT_POS:
			zmpinit_file = fopen("c:\\ZMPInitPos.dat", "wb+");
			
			if(zmpinit_file == NULL) RtWprintf(L"\n>>> File open error(SAVE_ZMP_INIT_POS)..!!");
			else
			{
				for(i=X ; i<=Yaw ; i++) ZMPInitPos[i] = WalkingInfo[RIGHT][i].RefPosFF;
				for(i=X ; i<=Yaw ; i++) ZMPInitPos[i+4] = WalkingInfo[LEFT][i].RefPosFF;
				for(i=RHY ; i<=LAR ; i++) ZMPInitPos[i+8] = Joint[i].RefAngleFF;

				ZMPInitPos[20] = IMUSensor[CENTERIMU].RollOffset;
				ZMPInitPos[21] = IMUSensor[CENTERIMU].PitchOffset;

				fwrite(ZMPInitPos, sizeof(float)*22, 1, zmpinit_file);
				fclose(zmpinit_file);
				RtWprintf(L"\n>>> File save success(SAVE_ZMP_INIT_POS)..!!");
			}
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_ENCODER_RESOLUTION:
			Joint[pSharedMemory->JointID].Encoder_size = pSharedMemory->CommandDataArray[0];
			Joint[pSharedMemory->JointID].Positive_dir = pSharedMemory->CommandDataArray[1];
			SetEncoderResolution(Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_DEADZONE:
			Joint[pSharedMemory->JointID].Deadzone = pSharedMemory->CommandData;
			SetDeadZone(Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_JAMPWM_FAULT:
			Joint[pSharedMemory->JointID].JAMmsTime = pSharedMemory->CommandDataArray[0];
			Joint[pSharedMemory->JointID].PWMmsTime = pSharedMemory->CommandDataArray[1];
			Joint[pSharedMemory->JointID].JAMDuty = (unsigned char)(pSharedMemory->CommandDataArray[2]);
			Joint[pSharedMemory->JointID].PWMDuty = (unsigned char)(pSharedMemory->CommandDataArray[3]);
			SetJamPwmSturation(Joint[pSharedMemory->JointID]); 
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_MAX_VEL_ACC:
			Joint[pSharedMemory->JointID].MaxAcc = pSharedMemory->CommandDataArray[0];
			Joint[pSharedMemory->JointID].MaxVel = pSharedMemory->CommandDataArray[1];
			SetMaxAccVel(Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_CONTROL_MODE:
			Joint[pSharedMemory->JointID].MotorControlMode = pSharedMemory->CommandData;
			SetControlMode(Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_HOME_SEARCH_PARAMETER:
			Joint[pSharedMemory->JointID].Limit_rev = pSharedMemory->CommandDataArray[0];
			Joint[pSharedMemory->JointID].SearchDirection = pSharedMemory->CommandDataArray[1];
			Joint[pSharedMemory->JointID].Offset_angle = pSharedMemory->CommandDataFloat[0];
			SetHomeSearchParameter(Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_HOME_MAX_VEL_ACC:
			Joint[pSharedMemory->JointID].MaxVelHome = pSharedMemory->CommandDataArray[0];
			Joint[pSharedMemory->JointID].MaxAccHome = pSharedMemory->CommandDataArray[1];
			Joint[pSharedMemory->JointID].HomeSearchMode = pSharedMemory->CommandDataArray[0];
			Joint[pSharedMemory->JointID].PWMDuty = pSharedMemory->CommandDataArray[0];
			SetHomeMaxVelAcc(Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_POSITION_LIMIT:
			Joint[pSharedMemory->JointID].UpperPositionLimit = pSharedMemory->CommandDataFloat[0];
			Joint[pSharedMemory->JointID].LowerPositionLimit = pSharedMemory->CommandDataFloat[1];
			SetUpperPositionLimit(Joint[pSharedMemory->JointID]);
			SetLowerPositionLimit(Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_ERROR_BOUND:
			Joint[pSharedMemory->JointID].I_ERR = pSharedMemory->CommandDataArray[0];
			Joint[pSharedMemory->JointID].B_ERR = pSharedMemory->CommandDataArray[1];
			Joint[pSharedMemory->JointID].E_ERR = pSharedMemory->CommandDataArray[2];
			SetErrorBound(Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case REQUEST_PARAMETER:
			RequestParameters(&Joint[pSharedMemory->JointID]);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case POSITION_LIMIT_ONOFF:
			for(i=0 ; i<NO_OF_JOINT ; i++) ;//PositionLimitOnOff(i, pSharedMemory->CommandData);
			//PositionLimitOnOff(LHP, pSharedMemory->CommandData);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case BEEP:
			Beep(pSharedMemory->CommandData);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case JOINT_REF_SET_RELATIVE:	// set joint position reference (relative mode)
			SetMoveJointAngle(pSharedMemory->JointID, pSharedMemory->GoalAngle, pSharedMemory->GoalTime, 0x00);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case JOINT_REF_SET_ABS:	// set joint position reference (abs mode)
			SetMoveJointAngle(pSharedMemory->JointID, pSharedMemory->GoalAngle, pSharedMemory->GoalTime, 0x01);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_FT_PARAMETER:	// set FT sensor parameters
			SetFTParameter(pSharedMemory->FTsensorID, pSharedMemory->FTsensor);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GET_FT_PARAMETER:	// get current FT sensor parameters
			GetFTParameter(pSharedMemory->FTsensorID, &pSharedMemory->FTsensor);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case NULL_FT_SENSOR:
			for(i=RFFT ; i<NO_OF_FT ; i++) 
				{ NullFTSensor(i, 0x00); RtSleep(10); }						
			pSharedMemory->CommandFlag = NO_ACT;
		case NULL_WRIST_FT_SENSOR:
			NullFTSensor(RWFT, 0x00); RtSleep(10);
			NullFTSensor(LWFT, 0x00); RtSleep(10);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case NULL_FOOT_ANGLE_SENSOR:
			RtWprintf(L"\n>>> FT angle nulling..!!");
			NullFootAngleSensor(FTSensor);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case NULL_IMU_SENSOR:
			RtWprintf(L"\n>>> IMU sensor nulling..!!");
			for(i=CENTERIMU ; i<NO_OF_IMU ; i++) { NullIMUSensor(i, 0x00); RtSleep(2); }
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_IMU_OFFSET:
			IMUSensor[CENTERIMU].RollOffset = pSharedMemory->CommandDataFloat[0];
			IMUSensor[CENTERIMU].PitchOffset = pSharedMemory->CommandDataFloat[1];
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case PRINT_FT_PARAMETER:	// print out current FT sensor parameters
			PrintFTParameter(pSharedMemory->FTsensorID);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_IMU_PARAMETER:		// set IMU sensor parameters
			SetIMUParameter(pSharedMemory->IMUsensorID, pSharedMemory->IMUsensor);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GET_IMU_PARAMETER:		// get current IMU parameters
			GetIMUParameter(pSharedMemory->IMUsensorID, &pSharedMemory->IMUsensor);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case PRINT_IMU_PARAMETER:	// print out current IMU parameters
			PrintIMUParameter(pSharedMemory->IMUsensorID);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_DAMPING_GAIN:
			if(pSharedMemory->CommandData == 0x00)
			{ for(i=0 ; i<3 ; i++) dampingGain[i] = pSharedMemory->CommandDataFloat[i]; }
			else { for(i=0 ; i<3 ; i++) dampingGain[i+3] = pSharedMemory->CommandDataFloat[i]; }
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_DSP_GAIN:
			dspGain = pSharedMemory->CommandDataFloat[0];
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_WALK_READY_POS:	// goto walk-ready position
			pSharedMemory->WB_DemoRunFlag = 0; // by Inhyeok
			_PassiveUpdatedFlag = 0;		// by Inhyeok
			//GoToLimitPos(RF1);
			//GoToLimitPos(LF1);
			GotoWalkReadyPos();			
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
			FingerControlModeChange(EJMC4, 0x01);
			FingerControlModeChange(EJMC5, 0x01);
			finger_control_mode = 0x01; // Current Control mode
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_HOME_POS:			// goto home position			
			_PassiveUpdatedFlag = 0;		// by Inhyeok
			pSharedMemory->WB_DemoRunFlag = 0; // by Inhyeok
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
			GotoHomePos();
			//GoToLimitPos(RF1);
			//GoToLimitPos(LF1);
			GoToLimitPos(NKY);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_DRC_HOME_POS:			// by Inhyeok
			_PassiveUpdatedFlag = 0;
			pSharedMemory->WB_DemoRunFlag = 0; // by Inhyeok
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
			SetDRCHomePos();
			GoToLimitPos(RFA);
			GoToLimitPos(LFA);
			GoToLimitPos(NKY);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_DRC_BIPED_READY_POS: // by Inhyeok
			_PassiveUpdatedFlag = 0;
			pSharedMemory->WB_DemoRunFlag = 0;
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;			
			SetDRCBipedReadyPos();
			FingerControlModeChange(EJMC4, 0x01);
			FingerControlModeChange(EJMC5, 0x01);
			finger_control_mode = 0x01; // Current Control mode
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_DRC_WIDE_BIPED_READY_POS: // by Inhyeok
			_PassiveUpdatedFlag = 0;
			pSharedMemory->WB_DemoRunFlag = 0;
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;			
			SetDRCWideBipedReadyPos();
			FingerControlModeChange(EJMC4, 0x01);
			FingerControlModeChange(EJMC5, 0x01);
			finger_control_mode = 0x01; // Current Control mode
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_DRC_QUADRUPED_READY_POS:  // by Inhyeok
			_PassiveUpdatedFlag = 0;
			pSharedMemory->WB_DemoRunFlag = 0;
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;			
			SetDRCQuadrupedReadyPos();
			FingerControlModeChange(EJMC4, 0x01);
			FingerControlModeChange(EJMC5, 0x01);
			finger_control_mode = 0x01; // Current Control mode
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_DRC_PRE_QUAD_READY_POS:  // by Inhyeok
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_DRC_STEERING_READY_POS: // by Inhyeok
			_PassiveUpdatedFlag = 0;
			pSharedMemory->WB_DemoRunFlag = 0;
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;			
			SetDRCSteeringReadyPos();
			FingerControlModeChange(EJMC4, 0x01);
			FingerControlModeChange(EJMC5, 0x01);
			finger_control_mode = 0x01; // Current Control mode
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GOTO_DRC_LADDER_CLIMBING_READY_POS: // by Inhyeok
			_PassiveUpdatedFlag = 0;
			pSharedMemory->WB_DemoRunFlag = 0;
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;			
			SetDRCLadderClimbingReadyPos();
			FingerControlModeChange(EJMC4, 0x01);
			FingerControlModeChange(EJMC5, 0x01);
			finger_control_mode = 0x01; // Current Control mode
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case START_ZMP_INITIALIZATION:	// start initialization control parameters(ZMP, Roll & Pitch angles..)
			pSharedMemory->WB_DemoRunFlag = 0; // by Inhyeok
			_PassiveUpdatedFlag = 0;		// by Inhyeok
			ZMPInitFlag = pSharedMemory->CommandData;
			
			if(ZMPInitFlag == 0x01)
			{
				//RtWprintf(L">>> ZMPInitPos[0] = %d", (int)(ZMPInitPos[0]*1000));
				SetMoveZMPInitPos(&WalkingInfo[RIGHT][X], ZMPInitPos[0], 1000.0f);
				SetMoveZMPInitPos(&WalkingInfo[RIGHT][Y], ZMPInitPos[1], 1000.0f);
				SetMoveZMPInitPos(&WalkingInfo[RIGHT][Z], ZMPInitPos[2], 1000.0f);
				//SetMoveZMPInitPos(&WalkingInfo[RIGHT][Yaw], ZMPInitPos[3], 1000.0f);
				
				SetMoveZMPInitPos(&WalkingInfo[LEFT][X], ZMPInitPos[4], 1000.0f);
				SetMoveZMPInitPos(&WalkingInfo[LEFT][Y], ZMPInitPos[5], 1000.0f);
				SetMoveZMPInitPos(&WalkingInfo[LEFT][Z], ZMPInitPos[6], 1000.0f);
				//SetMoveZMPInitPos(&WalkingInfo[LEFT][Yaw], ZMPInitPos[7], 1000.0f);
				
				//SetMoveZMPInitAngle(RHY, ZMPInitPos[8], 1000.0f);
				//SetMoveZMPInitAngle(RHR, ZMPInitPos[9], 1000.0f);
				SetMoveZMPInitAngle(RHP, ZMPInitPos[10], 1000.0f);
				//SetMoveZMPInitAngle(RKN, ZMPInitPos[11], 1000.0f);
				SetMoveZMPInitAngle(RAP, ZMPInitPos[12], 1000.0f);
				SetMoveZMPInitAngle(RAR, ZMPInitPos[13], 1000.0f);
				
				//SetMoveZMPInitAngle(LHY, ZMPInitPos[14], 1000.0f);
				//SetMoveZMPInitAngle(LHR, ZMPInitPos[15], 1000.0f);
				SetMoveZMPInitAngle(LHP, ZMPInitPos[16], 1000.0f);
				//SetMoveZMPInitAngle(LKN, ZMPInitPos[17], 1000.0f);
				SetMoveZMPInitAngle(LAP, ZMPInitPos[18], 1000.0f);
				SetMoveZMPInitAngle(LAR, ZMPInitPos[19], 1000.0f);

				IMUSensor[CENTERIMU].RollOffset = ZMPInitPos[20];
				IMUSensor[CENTERIMU].PitchOffset = ZMPInitPos[21];

/*				SetMoveZMPInitPos(&WalkingInfo[RIGHT][X], 0.016973f, 1000.0f);
				SetMoveZMPInitPos(&WalkingInfo[RIGHT][Y], -0.005439f, 1000.0f);
				SetMoveZMPInitPos(&WalkingInfo[RIGHT][Z], 0.000823f, 1000.0f);

				SetMoveZMPInitPos(&WalkingInfo[LEFT][X], 0.016973f, 1000.0f);
				SetMoveZMPInitPos(&WalkingInfo[LEFT][Y], -0.005439f, 1000.0f);
				SetMoveZMPInitPos(&WalkingInfo[LEFT][Z], -0.000823f, 1000.0f);

				SetMoveZMPInitAngle(RHP, -0.075601f, 1000.0f);
				SetMoveZMPInitAngle(RAP, 0.178811f, 1000.0f);
				SetMoveZMPInitAngle(RAR, -0.665374f, 1000.0f);

				SetMoveZMPInitAngle(LHP, -0.075601f, 1000.0f);
				SetMoveZMPInitAngle(LAP, 0.0f, 1000.0f);
				SetMoveZMPInitAngle(LAR, -0.543949f, 1000.0f);
*/			}

			pSharedMemory->MotorControlMode = CTRLMODE_ZMP_INITIALIZATION;
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case STOP_ZMP_INITIALIZATION:	// stop initialization control parameters
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
			pSharedMemory->CommandFlag = NO_ACT;
			break;		
		case GOTO_FORWARD:			// start walking
			WalkingStartStop = 0x02;
			StepChangeFlag = 0x00;
			if(pSharedMemory->JW_temp[9]>=0)
			{
				WalkingPhaseNextSway = HIP_CENTER_TO_RIGHT_SET;
				//if(pSharedMemory->JW_temp[6]==0) WalkingPhaseNext[X] = WALK_READY;
				//else 
					WalkingPhaseNext[X] = FIRST_LEFT_FOOT_UP_FORWARD_SET;
				WalkingPhaseNext[Y] = FIRST_LEFT_FOOT_UP_SET;
				WalkingPhaseNext[Z] = FIRST_LEFT_FOOT_UP_SET;
				WalkingPhaseNext[Yaw] = FIRST_LEFT_FOOT_UP_SET;
			}
			else
			{	
				WalkingPhaseNextSway = HIP_CENTER_TO_RIGHT_SET;
				//if(pSharedMemory->JW_temp[6]==0) WalkingPhaseNext[X] = WALK_READY;
				//else 
					WalkingPhaseNext[X] = FIRST_RIGHT_FOOT_UP_FORWARD_SET;
				WalkingPhaseNext[Y] = FIRST_RIGHT_FOOT_UP_SET;
				WalkingPhaseNext[Z] = FIRST_RIGHT_FOOT_UP_SET;
				WalkingPhaseNext[Yaw] = FIRST_RIGHT_FOOT_UP_SET;
			}
			pSharedMemory->MotorControlMode = CTRLMODE_WALKING;

			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case STOP_WALKING:		// stop walking
			WalkingStartStop = 0x01;
			pSharedMemory->MotorControlMode = CTRLMODE_WALKING;
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_MOCAP:		// modified by Inhyeok
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_MOCAP_CONTROL:
			DemoFlag = pSharedMemory->CommandData;
			pSharedMemory->MotorControlMode = CTRLMODE_JW_MOCAP;
			break;
		case C_CONTROL_MODE:
			FingerControlModeChange(EJMC4, 0x01);
			FingerControlModeChange(EJMC5, 0x01);
			finger_control_mode = 0x01; // Current Control mode
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case P_CONTROL_MODE:  // Position control of finger makes finger motors broken!! Don't use
			//FingerControlModeChange(EJMC4, 0x00);
			//FingerControlModeChange(EJMC5, 0x00);
			GoToLimitPos(pSharedMemory->JointID);
			FingerControlModeChange(EJMC4, 0x01);
			FingerControlModeChange(EJMC5, 0x01);
			finger_control_mode = 0x01; // Current Control mode
			//finger_control_mode = 0x00; // Position Control mode
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GRIP_ON_R:
			Joint[RF2].RefVelCurrent = -pSharedMemory->CommandDataFloat[0];
			if(pSharedMemory->CommandDataFloat[2] == 1.f)
				Joint[RF3].RefVelCurrent = -pSharedMemory->CommandDataFloat[1];
			MoveJMC(EJMC4);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GRIP_OFF_R:
			Joint[RF2].RefVelCurrent = pSharedMemory->CommandDataFloat[0];
			if(pSharedMemory->CommandDataFloat[2] == 1.f)
				Joint[RF3].RefVelCurrent = pSharedMemory->CommandDataFloat[1];
			MoveJMC(EJMC4);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GRIP_STOP_R:
			Joint[RF2].RefVelCurrent = 0;
			if(pSharedMemory->CommandDataFloat[2] == 1.f)
				Joint[RF3].RefVelCurrent = 0;
			MoveJMC(EJMC4);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GRIP_ON_TRIG_R:
			Joint[RF3].RefVelCurrent = -pSharedMemory->CommandDataFloat[0];
			MoveJMC(EJMC4);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GRIP_OFF_TRIG_R:
			Joint[RF3].RefVelCurrent = pSharedMemory->CommandDataFloat[0];
			MoveJMC(EJMC4);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GRIP_STOP_TRIG_R:
			Joint[RF3].RefVelCurrent = 0;
			MoveJMC(EJMC4);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GRIP_ON_L:			
			Joint[LF2].RefVelCurrent = -pSharedMemory->CommandDataFloat[0];
			if(pSharedMemory->CommandDataFloat[2] == 1.f)
				Joint[LF3].RefVelCurrent = -pSharedMemory->CommandDataFloat[1];
			MoveJMC(EJMC5);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GRIP_OFF_L:
			Joint[LF2].RefVelCurrent = pSharedMemory->CommandDataFloat[0];
			if(pSharedMemory->CommandDataFloat[2] == 1.f)
				Joint[LF3].RefVelCurrent = pSharedMemory->CommandDataFloat[1];
			MoveJMC(EJMC5);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GRIP_STOP_L:
			Joint[LF2].RefVelCurrent = 0;
			if(pSharedMemory->CommandDataFloat[2] == 1.f)
				Joint[LF3].RefVelCurrent = 0;
			MoveJMC(EJMC5);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GRIP_ON_TRIG_L:
			Joint[LF3].RefVelCurrent = -pSharedMemory->CommandDataFloat[0];
			MoveJMC(EJMC5);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GRIP_OFF_TRIG_L:
			Joint[LF3].RefVelCurrent = pSharedMemory->CommandDataFloat[0];
			MoveJMC(EJMC5);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case GRIP_STOP_TRIG_L:
			Joint[LF3].RefVelCurrent = 0;
			MoveJMC(EJMC5);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case MOVE_NECK:
			Joint[NKY].RefAngleCurrent = pSharedMemory->angle_NKY;
			Joint[NK1].RefAngleCurrent = pSharedMemory->angle_NK1;
			if(pSharedMemory->scan_flag == 0)
			{
				Joint[NK2].RefAngleCurrent = pSharedMemory->angle_NK2;
				MoveJMC(EJMC2);
			}
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_NECK_SPEED:
			if(pSharedMemory->scan_flag != 0)
				pSharedMemory->spd_NK2 = 0;
			NeckDynamixelSetSpeed(pSharedMemory->spd_NKY, pSharedMemory->spd_NK1, pSharedMemory->spd_NK2);
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case DEMO_FLAG:
			DemoFlag = pSharedMemory->CommandData;
			pSharedMemory->MotorControlMode = CTRLMODE_DEMO;
			break;
		case TEST_FUNCTION:
			//TestFunction();
			FETDriverOnOff(JMC3, 0x00);
			FETDriverOnOff(JMC7, 0x00);

			FETDriverOnOff(JMC3, 0x01);
			SendRunStopCMD(JMC3, 0x01);

			FETDriverOnOff(JMC7, 0x01);
			SendRunStopCMD(JMC7, 0x01);

			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_PREDEF_WALK:	// by jungho77
			currentWalkingStep = 0;
			for(i=0 ; i<50 ; i++)
			{
				forwardWalkingSequence[i] = 0.0f;
				sideWalkingSequence[i] = 0.0f;
				rotationWalkingSequence[i] = 0.0f;
			}
			walkingSceneNo = pSharedMemory->CommandData;

			if(walkingSceneNo == 0x00) 
			{
				numberOfWalking = 37;
				//face_change(14);
				lastStep = forwardWalkingSequence[0] = 0.0f; swaySequence[0] = 0.072f;
				forwardWalkingSequence[1] = 0.0f;	swaySequence[1] = 0.072f;
				forwardWalkingSequence[2] = 0.0f;	swaySequence[2] = 0.07f; // rotate
				forwardWalkingSequence[3] = 0.0f;	swaySequence[3] = 0.07f; // rotate
				forwardWalkingSequence[4] = 0.0f;	swaySequence[4] = 0.07f; // rotate
				forwardWalkingSequence[5] = 0.0f;	swaySequence[5] = 0.07f; // rotate
				forwardWalkingSequence[6] = 0.0f;	swaySequence[6] = 0.07f; // rotate
				forwardWalkingSequence[7] = 0.0f;	swaySequence[7] = 0.07f; // rotate
				forwardWalkingSequence[8] = 0.0f;	swaySequence[8] = 0.07f; // rotate
				forwardWalkingSequence[9] = 0.0f;	swaySequence[9] = 0.07f; // rotate
				forwardWalkingSequence[10] = 0.0f;	swaySequence[10] = 0.07f; // rotate
				forwardWalkingSequence[11] = 0.0f;	swaySequence[11] = 0.07f; // rotate
				forwardWalkingSequence[12] = 0.0f;	swaySequence[12] = 0.07f;
				forwardWalkingSequence[13] = 0.06f;	swaySequence[13] = 0.07f;
				forwardWalkingSequence[14] = 0.06f;	swaySequence[14] = 0.07f;
				forwardWalkingSequence[15] = 0.06f;	swaySequence[15] = 0.07f;
				forwardWalkingSequence[16] = 0.06f;	swaySequence[16] = 0.07f;
				forwardWalkingSequence[17] = 0.06f;	swaySequence[17] = 0.07f;
				forwardWalkingSequence[18] = 0.06f;	swaySequence[18] = 0.07f;
				forwardWalkingSequence[19] = 0.06f;	swaySequence[19] = 0.07f;
				forwardWalkingSequence[20] = 0.06f;	swaySequence[20] = 0.07f;
				forwardWalkingSequence[21] = 0.06f;	swaySequence[21] = 0.07f;
				forwardWalkingSequence[22] = 0.06f;	swaySequence[22] = 0.07f;
				forwardWalkingSequence[23] = 0.06f;	swaySequence[23] = 0.07f;
				forwardWalkingSequence[24] = 0.06f;	swaySequence[24] = 0.07f;
				forwardWalkingSequence[25] = 0.06f;	swaySequence[25] = 0.07f;
				forwardWalkingSequence[26] = 0.06f;	swaySequence[26] = 0.07f;
				forwardWalkingSequence[27] = 0.06f;	swaySequence[27] = 0.07f;
				forwardWalkingSequence[28] = 0.06f;	swaySequence[28] = 0.07f;
				forwardWalkingSequence[29] = 0.06f;	swaySequence[29] = 0.07f;
				forwardWalkingSequence[30] = 0.06f;	swaySequence[30] = 0.07f;
				forwardWalkingSequence[31] = 0.06f;	swaySequence[31] = 0.07f;
				forwardWalkingSequence[32] = 0.0f;	swaySequence[32] = 0.072f;
				forwardWalkingSequence[33] = 0.0f;	swaySequence[33] = 0.07f; // rotate
				forwardWalkingSequence[34] = 0.0f;	swaySequence[34] = 0.07f; // rotate
				forwardWalkingSequence[35] = 0.0f;	swaySequence[35] = 0.07f; // rotate
				forwardWalkingSequence[36] = 0.0f;	swaySequence[36] = 0.072f;
				forwardWalkingSequence[37] = 0.0f;	swaySequence[37] = 0.072f;
				
				sideWalkingSequence[0] = 0.0f;
				sideWalkingSequence[1] = 0.0f;
				sideWalkingSequence[2] = 0.0f;
				sideWalkingSequence[3] = 0.0f;
				sideWalkingSequence[4] = 0.0f;
				sideWalkingSequence[5] = 0.0f;
				sideWalkingSequence[6] = 0.0f;
				sideWalkingSequence[7] = 0.0f;
				sideWalkingSequence[8] = 0.0f;
				sideWalkingSequence[9] = 0.0f;
				
				rotationWalkingSequence[0] = 0.0f;
				rotationWalkingSequence[1] = 0.0f;
				rotationWalkingSequence[2] = -15.0f;
				rotationWalkingSequence[3] = -15.0f;
				rotationWalkingSequence[4] = -15.0f;
				rotationWalkingSequence[5] = -15.0f;
				rotationWalkingSequence[6] = -15.0f;
				rotationWalkingSequence[7] = -15.0f;
				rotationWalkingSequence[8] = -15.0f;
				rotationWalkingSequence[9] = -13.0f;
				rotationWalkingSequence[10] = -10.0f;
				rotationWalkingSequence[11] = -10.0f;
				rotationWalkingSequence[12] = 0.0f;
				rotationWalkingSequence[13] = 0.0f;
				rotationWalkingSequence[14] = 0.0f;
				rotationWalkingSequence[15] = 0.0f;
				rotationWalkingSequence[16] = 0.0f;
				rotationWalkingSequence[17] = 0.0f;
				rotationWalkingSequence[18] = 0.0f;
				rotationWalkingSequence[19] = 0.0f;
				rotationWalkingSequence[20] = 0.0f;
				rotationWalkingSequence[21] = 0.0f;
				rotationWalkingSequence[22] = 0.0f;
				rotationWalkingSequence[23] = 0.0f;
				rotationWalkingSequence[24] = 0.0f;
				rotationWalkingSequence[25] = 0.0f;
				rotationWalkingSequence[26] = 0.0f;
				rotationWalkingSequence[27] = 0.0f;
				rotationWalkingSequence[28] = 0.0f;
				rotationWalkingSequence[29] = 0.0f;
				rotationWalkingSequence[30] = 0.0f;
				rotationWalkingSequence[31] = 0.0f;
				rotationWalkingSequence[32] = 0.0f;
				rotationWalkingSequence[33] = -15.0f;
				rotationWalkingSequence[34] = -15.0f;
				rotationWalkingSequence[35] = -10.0f;
				rotationWalkingSequence[36] = 0.0f;
				rotationWalkingSequence[37] = 0.0f;

			}
			else if(walkingSceneNo == 0x01) //Walk2 (Back to edge)
			{
				numberOfWalking = 23;
				
				lastStep = forwardWalkingSequence[0] = 0.0f; swaySequence[0] = 0.072f;
				forwardWalkingSequence[1] = 0.0f;	swaySequence[1] = 0.072f;
				forwardWalkingSequence[2] = 0.0f;	swaySequence[2] = 0.072f;
				forwardWalkingSequence[3] = 0.0f;	swaySequence[3] = 0.072f;
				forwardWalkingSequence[4] = 0.0f;	swaySequence[4] = 0.072f;
				forwardWalkingSequence[5] = 0.0f;	swaySequence[5] = 0.07f;
				forwardWalkingSequence[6] = -0.06f;	swaySequence[6] = 0.07f;
				forwardWalkingSequence[7] = -0.06f;	swaySequence[7] = 0.07f;
				forwardWalkingSequence[8] = -0.06f;	swaySequence[8] = 0.07f;
				forwardWalkingSequence[9] = -0.06f;	swaySequence[9] = 0.07f;
				forwardWalkingSequence[10] = -0.06f;	swaySequence[10] = 0.07f;
				forwardWalkingSequence[11] = -0.06f;	swaySequence[11] = 0.07f;
				forwardWalkingSequence[12] = -0.06f;	swaySequence[12] = 0.07f;
				forwardWalkingSequence[13] = -0.06f;	swaySequence[13] = 0.07f;
				forwardWalkingSequence[14] = -0.06f;	swaySequence[14] = 0.07f;
				forwardWalkingSequence[15] = -0.06f;	swaySequence[15] = 0.07f;
				forwardWalkingSequence[16] = -0.06f;	swaySequence[16] = 0.07f;
				forwardWalkingSequence[17] = -0.06f;	swaySequence[17] = 0.07f;
				forwardWalkingSequence[18] = -0.06f;	swaySequence[18] = 0.07f;
				forwardWalkingSequence[19] = -0.06f;	swaySequence[19] = 0.07f;
				forwardWalkingSequence[20] = -0.06f;	swaySequence[20] = 0.07f;
				forwardWalkingSequence[21] = -0.06f;	swaySequence[21] = 0.07f;
				forwardWalkingSequence[22] = -0.06f;	swaySequence[22] = 0.07f;
				forwardWalkingSequence[23] = 0.0f;	swaySequence[23] = 0.07f;
				forwardWalkingSequence[24] = 0.0f;	swaySequence[24] = 0.07f;
				
				
				sideWalkingSequence[0] = 0.0f;
				sideWalkingSequence[1] = 0.0f;
				sideWalkingSequence[2] = 0.0f;
				sideWalkingSequence[3] = 0.0f;
				sideWalkingSequence[4] = 0.0f;
				sideWalkingSequence[5] = 0.0f;
				sideWalkingSequence[6] = 0.0f;
				sideWalkingSequence[7] = 0.0f;
				sideWalkingSequence[8] = 0.0f;
				sideWalkingSequence[9] = 0.0f;
				
				rotationWalkingSequence[0] = 0.0f;
				rotationWalkingSequence[1] = 15.0f;
				rotationWalkingSequence[2] = 15.0f;
				rotationWalkingSequence[3] = 15.0f;
				rotationWalkingSequence[4] = 15.0f;
				rotationWalkingSequence[5] = 0.0f;
				rotationWalkingSequence[6] = 0.0f;
				rotationWalkingSequence[7] = 0.0f;
				rotationWalkingSequence[8] = 0.0f;
				rotationWalkingSequence[9] = 0.0f;
				rotationWalkingSequence[10] = 0.0f;
				rotationWalkingSequence[11] = 0.0f;
				rotationWalkingSequence[12] = 0.0f;
				rotationWalkingSequence[13] = 0.0f;
				rotationWalkingSequence[14] = 0.0f;
				rotationWalkingSequence[15] = 0.0f;
				rotationWalkingSequence[16] = 0.0f;
				rotationWalkingSequence[17] = 0.0f;
				rotationWalkingSequence[18] = 0.0f;
				rotationWalkingSequence[19] = 0.0f;
				rotationWalkingSequence[20] = 0.0f;
				rotationWalkingSequence[21] = 0.0f;
				rotationWalkingSequence[22] = 0.0f;
				rotationWalkingSequence[23] = 0.0f;
				rotationWalkingSequence[24] = 0.0f;
				
			}
			else if(walkingSceneNo == 0x02) // Walk3 (Go to center again)
			{
				numberOfWalking = 22;
				
				lastStep = forwardWalkingSequence[0] = 0.06f; swaySequence[0] = 0.07f;
				forwardWalkingSequence[1] = 0.06f;	swaySequence[1] = 0.07f;
				forwardWalkingSequence[2] = 0.06f;	swaySequence[2] = 0.07f;
				forwardWalkingSequence[3] = 0.06f;	swaySequence[3] = 0.07f;
				forwardWalkingSequence[4] = 0.06f;	swaySequence[4] = 0.07f;
				forwardWalkingSequence[5] = 0.06f;	swaySequence[5] = 0.07f;
				forwardWalkingSequence[6] = 0.06f;	swaySequence[6] = 0.07f;
				forwardWalkingSequence[7] = 0.06f;	swaySequence[7] = 0.07f;
				forwardWalkingSequence[8] = 0.06f;	swaySequence[8] = 0.07f;
				forwardWalkingSequence[9] = 0.06f;	swaySequence[9] = 0.07f;
				forwardWalkingSequence[10] = 0.06f;	swaySequence[10] = 0.07f;
				forwardWalkingSequence[11] = 0.06f;	swaySequence[11] = 0.07f;
				forwardWalkingSequence[12] = 0.06f;	swaySequence[12] = 0.07f;
				forwardWalkingSequence[13] = 0.06f;	swaySequence[13] = 0.07f;
				forwardWalkingSequence[14] = 0.06f;	swaySequence[14] = 0.07f;
				forwardWalkingSequence[15] = 0.06f;	swaySequence[15] = 0.07f;
				forwardWalkingSequence[16] = 0.06f;	swaySequence[16] = 0.07f;
				forwardWalkingSequence[17] = 0.06f;	swaySequence[17] = 0.07f;//rotate
				forwardWalkingSequence[18] = 0.0f;	swaySequence[18] = 0.07f;//rotate
				forwardWalkingSequence[19] = 0.0f;	swaySequence[19] = 0.07f;//rotate
				forwardWalkingSequence[20] = 0.0f;	swaySequence[20] = 0.072f;
				forwardWalkingSequence[21] = 0.0f;	swaySequence[21] = 0.072f;
				forwardWalkingSequence[22] = 0.0f;	swaySequence[22] = 0.072f;
				forwardWalkingSequence[23] = 0.0f;	swaySequence[23] = 0.072f;
				forwardWalkingSequence[24] = 0.0f;	swaySequence[24] = 0.072f;
				
				
				sideWalkingSequence[0] = 0.0f;
				sideWalkingSequence[1] = 0.0f;
				sideWalkingSequence[2] = 0.0f;
				sideWalkingSequence[3] = 0.0f;
				sideWalkingSequence[4] = 0.0f;
				sideWalkingSequence[5] = 0.0f;
				sideWalkingSequence[6] = 0.0f;
				sideWalkingSequence[7] = 0.0f;
				sideWalkingSequence[8] = 0.0f;
				sideWalkingSequence[9] = 0.0f;
				
				rotationWalkingSequence[0] = 0.0f;
				rotationWalkingSequence[1] = 0.0f;
				rotationWalkingSequence[2] = 0.0f;
				rotationWalkingSequence[3] = 0.0f;
				rotationWalkingSequence[4] = 0.0f;
				rotationWalkingSequence[5] = 0.0f;
				rotationWalkingSequence[6] = 0.0f;
				rotationWalkingSequence[7] = 0.0f;
				rotationWalkingSequence[8] = 0.0f;
				rotationWalkingSequence[9] = 0.0f;
				rotationWalkingSequence[10] = 0.0f;
				rotationWalkingSequence[11] = 0.0f;
				rotationWalkingSequence[12] = 0.0f;
				rotationWalkingSequence[13] = 0.0f;
				rotationWalkingSequence[14] = 0.0f;
				rotationWalkingSequence[15] = 0.0f;
				rotationWalkingSequence[16] = 0.0f;
				rotationWalkingSequence[17] = -15.0f;
				rotationWalkingSequence[18] = -15.0f;
				rotationWalkingSequence[19] = -10.0f;
				rotationWalkingSequence[20] = 0.0f;
				rotationWalkingSequence[21] = 0.0f;
				rotationWalkingSequence[22] = 0.0f;
				rotationWalkingSequence[23] = 0.0f;
				rotationWalkingSequence[24] = 0.0f;
				
			}
			else if(walkingSceneNo == 0x03) //Walk4 (Back to edge again)
			{
				numberOfWalking = 26;
				
				lastStep = forwardWalkingSequence[0] = 0.0f; swaySequence[0] = 0.072f;//0.069f;
				forwardWalkingSequence[1] = 0.0f;	swaySequence[1] = 0.07f;  // rotate
				forwardWalkingSequence[2] = 0.0f;	swaySequence[2] = 0.07f; // rotate
				forwardWalkingSequence[3] = 0.0f;	swaySequence[3] = 0.07f; // rotate
				forwardWalkingSequence[4] = 0.0f;	swaySequence[4] = 0.07f; // rotate
				forwardWalkingSequence[5] = 0.0f;	swaySequence[5] = 0.07f;
				forwardWalkingSequence[6] = -0.06f;	swaySequence[6] = 0.07f;
				forwardWalkingSequence[7] = -0.06f;	swaySequence[7] = 0.07f;
				forwardWalkingSequence[8] = -0.06f;	swaySequence[8] = 0.07f;
				forwardWalkingSequence[9] = -0.06f;	swaySequence[9] = 0.07f;
				forwardWalkingSequence[10] = -0.06f;	swaySequence[10] = 0.07f;
				forwardWalkingSequence[11] = -0.06f;	swaySequence[11] = 0.07f;
				forwardWalkingSequence[12] = -0.06f;	swaySequence[12] = 0.07f;
				forwardWalkingSequence[13] = -0.06f;	swaySequence[13] = 0.07f;
				forwardWalkingSequence[14] = -0.06f;	swaySequence[14] = 0.07f;
				forwardWalkingSequence[15] = -0.06f;	swaySequence[15] = 0.07f;
				forwardWalkingSequence[16] = -0.06f;	swaySequence[16] = 0.07f;
				forwardWalkingSequence[17] = -0.06f;	swaySequence[17] = 0.07f;
				forwardWalkingSequence[18] = -0.06f;	swaySequence[18] = 0.07f;
				forwardWalkingSequence[19] = -0.06f;	swaySequence[19] = 0.07f;
				forwardWalkingSequence[20] = -0.06f;	swaySequence[20] = 0.07f;
				forwardWalkingSequence[21] = -0.06f;	swaySequence[21] = 0.07f;
				forwardWalkingSequence[22] = -0.06f;	swaySequence[22] = 0.07f;
				forwardWalkingSequence[23] = 0.0f;	swaySequence[23] = 0.07f; // rotate
				forwardWalkingSequence[24] = 0.0f;	swaySequence[24] = 0.07f; // rotate
				forwardWalkingSequence[25] = 0.0f;	swaySequence[25] = 0.07f;
				forwardWalkingSequence[26] = 0.0f;	swaySequence[26] = 0.072f;
				forwardWalkingSequence[27] = 0.0f;	swaySequence[27] = 0.072f;
				
				
				sideWalkingSequence[0] = 0.0f;
				sideWalkingSequence[1] = 0.0f;
				sideWalkingSequence[2] = 0.0f;
				sideWalkingSequence[3] = 0.0f;
				sideWalkingSequence[4] = 0.0f;
				sideWalkingSequence[5] = 0.0f;
				sideWalkingSequence[6] = 0.0f;
				sideWalkingSequence[7] = 0.0f;
				sideWalkingSequence[8] = 0.0f;
				sideWalkingSequence[9] = 0.0f;
				
				rotationWalkingSequence[0] = 0.0f;
				rotationWalkingSequence[1] = 15.0f;
				rotationWalkingSequence[2] = 15.0f;
				rotationWalkingSequence[3] = 15.0f;
				rotationWalkingSequence[4] = 15.0f;
				rotationWalkingSequence[5] = 0.0f;
				rotationWalkingSequence[6] = 0.0f;
				rotationWalkingSequence[7] = 0.0f;
				rotationWalkingSequence[8] = 0.0f;
				rotationWalkingSequence[9] = 0.0f;
				rotationWalkingSequence[10] = 0.0f;
				rotationWalkingSequence[11] = 0.0f;
				rotationWalkingSequence[12] = 0.0f;
				rotationWalkingSequence[13] = 0.0f;
				rotationWalkingSequence[14] = 0.0f;
				rotationWalkingSequence[15] = 0.0f;
				rotationWalkingSequence[16] = 0.0f;
				rotationWalkingSequence[17] = 0.0f;
				rotationWalkingSequence[18] = 0.0f;
				rotationWalkingSequence[19] = 0.0f;
				rotationWalkingSequence[20] = 0.0f;
				rotationWalkingSequence[21] = 0.0f;
				rotationWalkingSequence[22] = 0.0f;
				rotationWalkingSequence[23] = -10.0f;
				rotationWalkingSequence[24] = -10.0f;
				rotationWalkingSequence[25] = 0.0f;
				rotationWalkingSequence[26] = 0.0f;
				rotationWalkingSequence[27] = 0.0f;
				
			}
			else if(walkingSceneNo == 0x04) //WalkingDemo
			{
				numberOfWalking = 68;
				
				lastStep = forwardWalkingSequence[0] = 0.0f; swaySequence[0] = 0.072f;
				forwardWalkingSequence[1] = 0.0f;	swaySequence[1] = 0.072f;
				forwardWalkingSequence[2] = 0.0f;	swaySequence[2] = 0.072f;
				forwardWalkingSequence[3] = 0.0f;	swaySequence[3] = 0.072f;
				forwardWalkingSequence[4] = 0.0f;	swaySequence[4] = 0.072f;
				forwardWalkingSequence[5] = 0.075f;	swaySequence[5] = 0.069f;
				forwardWalkingSequence[6] = 0.075f;	swaySequence[6] = 0.069f;
				forwardWalkingSequence[7] = 0.075f;	swaySequence[7] = 0.069f;
				forwardWalkingSequence[8] = 0.075f;	swaySequence[8] = 0.069f;
				forwardWalkingSequence[9] = 0.075f;	swaySequence[9] = 0.069f;
				forwardWalkingSequence[10] = 0.075f;	swaySequence[10] = 0.069f;
				forwardWalkingSequence[11] = 0.075f;	swaySequence[11] = 0.069f;
				forwardWalkingSequence[12] = 0.075f;	swaySequence[12] = 0.069f;
				forwardWalkingSequence[13] = 0.075f;	swaySequence[13] = 0.069f;
				forwardWalkingSequence[14] = 0.075f;	swaySequence[14] = 0.069f;
				forwardWalkingSequence[15] = 0.075f;	swaySequence[15] = 0.069f;
				forwardWalkingSequence[16] = 0.075f;	swaySequence[16] = 0.069f;
				forwardWalkingSequence[17] = 0.075f;	swaySequence[17] = 0.069f;
				forwardWalkingSequence[18] = 0.075f;	swaySequence[18] = 0.069f;
				forwardWalkingSequence[19] = 0.075f;	swaySequence[19] = 0.069f;
				forwardWalkingSequence[20] = 0.075f;	swaySequence[20] = 0.069f;
				forwardWalkingSequence[21] = 0.075f;	swaySequence[21] = 0.069f;
				forwardWalkingSequence[22] = 0.075f;	swaySequence[22] = 0.069f;
				forwardWalkingSequence[23] = 0.075f;	swaySequence[23] = 0.069f;
				forwardWalkingSequence[24] = 0.0f;	swaySequence[24] = 0.072f;
				forwardWalkingSequence[25] = 0.0f;	swaySequence[25] = 0.072f;
				forwardWalkingSequence[26] = 0.0f;	swaySequence[26] = 0.072f;
				forwardWalkingSequence[27] = 0.0f;	swaySequence[27] = 0.07f;	//rotate
				forwardWalkingSequence[28] = 0.0f;	swaySequence[28] = 0.07f;	//rotate
				forwardWalkingSequence[29] = 0.0f;	swaySequence[29] = 0.07f;	//rotate
				forwardWalkingSequence[30] = 0.0f;	swaySequence[30] = 0.07f;
				forwardWalkingSequence[31] = 0.0f;	swaySequence[31] = 0.072f;
				forwardWalkingSequence[32] = 0.0f;	swaySequence[32] = 0.072f;
				forwardWalkingSequence[33] = -0.06f;	swaySequence[33] = 0.07f;
				forwardWalkingSequence[34] = -0.06f;	swaySequence[34] = 0.07f;
				forwardWalkingSequence[35] = -0.06f;	swaySequence[35] = 0.07f;
				forwardWalkingSequence[36] = -0.06f;	swaySequence[36] = 0.07f;
				forwardWalkingSequence[37] = -0.06f;	swaySequence[37] = 0.07f;
				forwardWalkingSequence[38] = -0.06f;	swaySequence[38] = 0.07f;
				forwardWalkingSequence[39] = -0.06f;	swaySequence[39] = 0.07f;
				forwardWalkingSequence[40] = -0.06f;	swaySequence[40] = 0.07f;
				forwardWalkingSequence[41] = -0.06f;	swaySequence[41] = 0.07f;
				forwardWalkingSequence[42] = -0.06f;	swaySequence[42] = 0.07f;
				forwardWalkingSequence[43] = -0.06f;	swaySequence[43] = 0.07f;
				forwardWalkingSequence[44] = -0.06f;	swaySequence[44] = 0.07f;
				forwardWalkingSequence[45] = -0.06f;	swaySequence[45] = 0.07f;
				forwardWalkingSequence[46] = -0.06f;	swaySequence[46] = 0.07f;
				forwardWalkingSequence[47] = -0.06f;	swaySequence[47] = 0.07f;
				forwardWalkingSequence[48] = 0.0f;	swaySequence[48] = 0.072f;
				forwardWalkingSequence[49] = 0.0f;	swaySequence[49] = 0.072f;
				forwardWalkingSequence[50] = 0.0f;	swaySequence[50] = 0.07f;	//rotate
				forwardWalkingSequence[51] = 0.0f;	swaySequence[51] = 0.07f;	//rotate
				forwardWalkingSequence[52] = 0.0f;	swaySequence[52] = 0.07f;
				forwardWalkingSequence[53] = 0.0f;	swaySequence[53] = 0.07f;
				forwardWalkingSequence[54] = 0.075f;	swaySequence[54] = 0.069f;
				forwardWalkingSequence[55] = 0.075f;	swaySequence[55] = 0.069f;
				forwardWalkingSequence[56] = 0.075f;	swaySequence[56] = 0.069f;
				forwardWalkingSequence[57] = 0.075f;	swaySequence[57] = 0.069f;
				forwardWalkingSequence[58] = 0.075f;	swaySequence[58] = 0.069f;
				forwardWalkingSequence[59] = 0.075f;	swaySequence[59] = 0.069f;
				forwardWalkingSequence[60] = 0.075f;	swaySequence[60] = 0.069f;
				forwardWalkingSequence[61] = 0.075f;	swaySequence[61] = 0.069f;
				
				forwardWalkingSequence[62] = 0.0f;	swaySequence[62] = 0.069f;
				forwardWalkingSequence[63] = 0.0f;	swaySequence[63] = 0.07f;	//rotate
				forwardWalkingSequence[64] = 0.0f;	swaySequence[64] = 0.07f;	//rotate
				forwardWalkingSequence[65] = 0.0f;	swaySequence[65] = 0.07f;	//rotate
				forwardWalkingSequence[66] = 0.0f;	swaySequence[66] = 0.07f;	//rotate
				forwardWalkingSequence[67] = 0.0f;	swaySequence[67] = 0.07f;	//rotate
				forwardWalkingSequence[68] = 0.0f;	swaySequence[68] = 0.07f;//rotate
				forwardWalkingSequence[69] = 0.0f;	swaySequence[69] = 0.07f;//rotate
				forwardWalkingSequence[70] = 0.0f;	swaySequence[70] = 0.07f;
				forwardWalkingSequence[71] = 0.0f;	swaySequence[71] = 0.07f;
				
				sideWalkingSequence[0] = 0.0f;
				sideWalkingSequence[1] = 0.0f;
				sideWalkingSequence[2] = 0.0f;
				sideWalkingSequence[3] = 0.0f;
				sideWalkingSequence[4] = 0.0f;
				sideWalkingSequence[5] = 0.0f;
				sideWalkingSequence[6] = 0.0f;
				sideWalkingSequence[7] = 0.0f;
				sideWalkingSequence[8] = 0.0f;
				sideWalkingSequence[9] = 0.0f;
				
				rotationWalkingSequence[0] = 0.0f;
				rotationWalkingSequence[1] = 0.0f;
				rotationWalkingSequence[2] = 0.0f;
				rotationWalkingSequence[3] = 0.0f;
				rotationWalkingSequence[4] = 0.0f;
				rotationWalkingSequence[5] = 0.0f;
				rotationWalkingSequence[6] = 0.0f;
				rotationWalkingSequence[7] = 0.0f;
				rotationWalkingSequence[8] = 0.0f;
				rotationWalkingSequence[9] = 0.0f;
				rotationWalkingSequence[10] = 0.0f;
				rotationWalkingSequence[11] = 0.0f;
				rotationWalkingSequence[12] = 0.0f;
				rotationWalkingSequence[13] = 0.0f;
				rotationWalkingSequence[14] = 0.0f;
				rotationWalkingSequence[15] = 0.0f;
				rotationWalkingSequence[16] = 0.0f;
				rotationWalkingSequence[17] = 0.0f;
				rotationWalkingSequence[18] = 0.0f;
				rotationWalkingSequence[19] = 0.0f;
				rotationWalkingSequence[20] = 0.0f;
				rotationWalkingSequence[21] = 0.0f;
				rotationWalkingSequence[22] = 0.0f;
				rotationWalkingSequence[23] = 0.0f;
				rotationWalkingSequence[24] = 0.0f;
				rotationWalkingSequence[25] = 0.0f;
				rotationWalkingSequence[26] = 0.0f;
				rotationWalkingSequence[27] = -20.0f;
				rotationWalkingSequence[28] = -20.0f;
				rotationWalkingSequence[29] = -20.0f;
				rotationWalkingSequence[30] = 0.0f;
				rotationWalkingSequence[31] = 0.0f;
				rotationWalkingSequence[32] = 0.0f;
				rotationWalkingSequence[33] = 0.0f;
				rotationWalkingSequence[34] = 0.0f;
				rotationWalkingSequence[35] = 0.0f;
				rotationWalkingSequence[36] = 0.0f;
				rotationWalkingSequence[37] = 0.0f;
				rotationWalkingSequence[38] = 0.0f;
				rotationWalkingSequence[39] = 0.0f;
				rotationWalkingSequence[40] = 0.0f;
				rotationWalkingSequence[41] = 0.0f;
				rotationWalkingSequence[42] = 0.0f;
				rotationWalkingSequence[43] = 0.0f;
				rotationWalkingSequence[44] = 0.0f;
				rotationWalkingSequence[45] = 0.0f;
				rotationWalkingSequence[46] = 0.0f;
				rotationWalkingSequence[47] = 0.0f;
				rotationWalkingSequence[48] = 0.0f;
				rotationWalkingSequence[49] = 0.0f;
				rotationWalkingSequence[50] = -20.0f;
				rotationWalkingSequence[51] = -20.0f;
				rotationWalkingSequence[52] = 0.0f;
				rotationWalkingSequence[53] = 0.0f;
				rotationWalkingSequence[54] = 0.0f;
				rotationWalkingSequence[55] = 0.0f;
				rotationWalkingSequence[56] = 0.0f;
				rotationWalkingSequence[57] = 0.0f;
				rotationWalkingSequence[58] = 0.0f;
				rotationWalkingSequence[59] = 0.0f;
				rotationWalkingSequence[60] = 0.0f;
				rotationWalkingSequence[61] = 0.0f;

				rotationWalkingSequence[62] = 0.0f;
				rotationWalkingSequence[63] = 20.0f;
				rotationWalkingSequence[64] = 20.0f;
				rotationWalkingSequence[65] = 20.0f;
				rotationWalkingSequence[66] = 20.0f;
				rotationWalkingSequence[67] = 20.0f;
				rotationWalkingSequence[68] = 5.0f;
				rotationWalkingSequence[69] = 5.0f;
				rotationWalkingSequence[70] = 0.0f;
				rotationWalkingSequence[71] = 0.0f;

				
			}
			else if(walkingSceneNo == 0x05) //Walk6 (rotate CW)
			{
				numberOfWalking = 6;
				
				lastStep = forwardWalkingSequence[0] = 0.0f; swaySequence[0] = 0.069f;
				forwardWalkingSequence[1] = 0.0f;	swaySequence[1] = 0.069f;
				forwardWalkingSequence[2] = 0.0f;	swaySequence[2] = 0.069f;
				forwardWalkingSequence[3] = 0.0f;	swaySequence[3] = 0.069f;
				forwardWalkingSequence[4] = 0.0f;	swaySequence[4] = 0.069f;
				forwardWalkingSequence[5] = 0.0f;	swaySequence[5] = 0.071f;
				forwardWalkingSequence[6] = 0.0f;	swaySequence[6] = 0.071f;
				forwardWalkingSequence[7] = 0.0f;	swaySequence[7] = 0.071f;
				forwardWalkingSequence[8] = 0.0f;	swaySequence[8] = 0.071f;
				forwardWalkingSequence[9] = 0.0f;	swaySequence[9] = 0.071f;
				forwardWalkingSequence[10] = 0.0f;	swaySequence[10] = 0.071f;
				forwardWalkingSequence[11] = 0.0f;	swaySequence[11] = 0.071f;
				forwardWalkingSequence[12] = 0.0f;	swaySequence[12] = 0.071f;
				
				
				sideWalkingSequence[0] = 0.0f;
				sideWalkingSequence[1] = 0.0f;
				sideWalkingSequence[2] = 0.0f;
				sideWalkingSequence[3] = 0.0f;
				sideWalkingSequence[4] = 0.0f;
				sideWalkingSequence[5] = 0.0f;
				sideWalkingSequence[6] = 0.0f;
				sideWalkingSequence[7] = 0.0f;
				sideWalkingSequence[8] = 0.0f;
				sideWalkingSequence[9] = 0.0f;
				
				rotationWalkingSequence[0] = 0.0f;
				rotationWalkingSequence[1] = 17.0f;
				rotationWalkingSequence[2] = 17.0f;
				rotationWalkingSequence[3] = 17.0f;
				rotationWalkingSequence[4] = 17.0f;
				rotationWalkingSequence[5] = 0.0f;
				rotationWalkingSequence[6] = 0.0f;
				rotationWalkingSequence[7] = 0.0f;
				rotationWalkingSequence[8] = 0.0f;
				rotationWalkingSequence[9] = 0.0f;
			}
			else if(walkingSceneNo == 0x06) //Walk7 (rotate CCW)
			{
				numberOfWalking = 5;
				
				lastStep = forwardWalkingSequence[0] = 0.0f; swaySequence[0] = 0.069f;
				forwardWalkingSequence[1] = 0.0f;	swaySequence[1] = 0.069f;
				forwardWalkingSequence[2] = 0.0f;	swaySequence[2] = 0.069f;
				forwardWalkingSequence[3] = 0.0f;	swaySequence[3] = 0.069f;
				forwardWalkingSequence[4] = 0.0f;	swaySequence[4] = 0.069f;
				forwardWalkingSequence[5] = 0.0f;	swaySequence[5] = 0.071f;
				forwardWalkingSequence[6] = 0.0f;	swaySequence[6] = 0.071f;
				forwardWalkingSequence[7] = 0.0f;	swaySequence[7] = 0.071f;
				forwardWalkingSequence[8] = 0.0f;	swaySequence[8] = 0.071f;
				forwardWalkingSequence[9] = 0.0f;	swaySequence[9] = 0.071f;
				forwardWalkingSequence[10] = 0.0f;	swaySequence[10] = 0.071f;
				forwardWalkingSequence[11] = 0.0f;	swaySequence[11] = 0.071f;
				forwardWalkingSequence[12] = 0.0f;	swaySequence[12] = 0.071f;
				
				
				sideWalkingSequence[0] = 0.0f;
				sideWalkingSequence[1] = 0.0f;
				sideWalkingSequence[2] = 0.0f;
				sideWalkingSequence[3] = 0.0f;
				sideWalkingSequence[4] = 0.0f;
				sideWalkingSequence[5] = 0.0f;
				sideWalkingSequence[6] = 0.0f;
				sideWalkingSequence[7] = 0.0f;
				sideWalkingSequence[8] = 0.0f;
				sideWalkingSequence[9] = 0.0f;
				
				rotationWalkingSequence[0] = 0.0f;
				rotationWalkingSequence[1] = -17.0f;
				rotationWalkingSequence[2] = -17.0f;
				rotationWalkingSequence[3] = -17.0f;
				rotationWalkingSequence[4] = -17.0f;
				rotationWalkingSequence[5] = 0.0f;
				rotationWalkingSequence[6] = 0.0f;
				rotationWalkingSequence[7] = 0.0f;
				rotationWalkingSequence[8] = 0.0f;
				rotationWalkingSequence[9] = 0.0f;
			}
			else;
/*			if(walkingSceneNo == 0x00) 
			{
				numberOfWalking = 37;
				face_change(14);
				lastStep = forwardWalkingSequence[0] = 0.0f; swaySequence[0] = 0.07f;//0.069f;
				forwardWalkingSequence[1] = 0.0f;	swaySequence[1] = 0.07f;//0.069f;
				forwardWalkingSequence[2] = 0.0f;	swaySequence[2] = 0.0697f;//0.0687f; // rotate
				forwardWalkingSequence[3] = 0.0f;	swaySequence[3] = 0.07f;//0.069f; // rotate
				forwardWalkingSequence[4] = 0.0f;	swaySequence[4] = 0.0697f;//0.0687f; // rotate
				forwardWalkingSequence[5] = 0.0f;	swaySequence[5] = 0.07f;//0.069f; // rotate
				forwardWalkingSequence[6] = 0.0f;	swaySequence[6] = 0.0697f;//0.0687f; // rotate
				forwardWalkingSequence[7] = 0.0f;	swaySequence[7] = 0.07f;//0.069f; // rotate
				forwardWalkingSequence[8] = 0.0f;	swaySequence[8] = 0.0697f;//0.0687f; // rotate
				forwardWalkingSequence[9] = 0.0f;	swaySequence[9] = 0.07f;//0.069f; // rotate
				forwardWalkingSequence[10] = 0.0f;	swaySequence[10] = 0.07f;//0.069f; // rotate
				forwardWalkingSequence[11] = 0.0f;	swaySequence[11] = 0.07f;//0.069f; // rotate
				forwardWalkingSequence[12] = 0.0f;	swaySequence[12] = 0.0685f;//0.0675f;
				forwardWalkingSequence[13] = 0.06f;	swaySequence[13] = 0.0685f;//0.0675f;
				forwardWalkingSequence[14] = 0.06f;	swaySequence[14] = 0.0685f;//0.0675f;
				forwardWalkingSequence[15] = 0.06f;	swaySequence[15] = 0.0675f;//0.0665f;
				forwardWalkingSequence[16] = 0.06f;	swaySequence[16] = 0.0675f;//0.0665f;
				forwardWalkingSequence[17] = 0.06f;	swaySequence[17] = 0.0675f;//0.0665f;
				forwardWalkingSequence[18] = 0.06f;	swaySequence[18] = 0.0675f;//0.0665f;
				forwardWalkingSequence[19] = 0.06f;	swaySequence[19] = 0.0685f;//0.0675f;
				forwardWalkingSequence[20] = 0.06f;	swaySequence[20] = 0.0685f;//0.0675f;
				forwardWalkingSequence[21] = 0.06f;	swaySequence[21] = 0.0685f;//0.0675f;
				forwardWalkingSequence[22] = 0.06f;	swaySequence[22] = 0.0685f;//0.0675f;
				forwardWalkingSequence[23] = 0.06f;	swaySequence[23] = 0.0685f;//0.0675f;
				forwardWalkingSequence[24] = 0.06f;	swaySequence[24] = 0.0685f;//0.0675f;
				forwardWalkingSequence[25] = 0.06f;	swaySequence[25] = 0.0685f;//0.0675f;
				forwardWalkingSequence[26] = 0.06f;	swaySequence[26] = 0.0685f;//0.0675f;
				forwardWalkingSequence[27] = 0.06f;	swaySequence[27] = 0.0685f;//0.0675f;
				forwardWalkingSequence[28] = 0.06f;	swaySequence[28] = 0.0685f;//0.0675f;
				forwardWalkingSequence[29] = 0.06f;	swaySequence[29] = 0.069f;//0.068f;
				forwardWalkingSequence[30] = 0.06f;	swaySequence[30] = 0.069f;//0.068f;
				forwardWalkingSequence[31] = 0.06f;	swaySequence[31] = 0.069f;//0.068f;
				forwardWalkingSequence[32] = 0.0f;	swaySequence[32] = 0.07f;//0.069f;
				forwardWalkingSequence[33] = 0.0f;	swaySequence[33] = 0.07f;//0.069f; // rotate
				forwardWalkingSequence[34] = 0.0f;	swaySequence[34] = 0.07f;//0.069f; // rotate
				forwardWalkingSequence[35] = 0.0f;	swaySequence[35] = 0.07f;//0.069f; // rotate
				forwardWalkingSequence[36] = 0.0f;	swaySequence[36] = 0.07f;//0.069f;
				forwardWalkingSequence[37] = 0.0f;	swaySequence[37] = 0.07f;//0.069f;
				
				sideWalkingSequence[0] = 0.0f;
				sideWalkingSequence[1] = 0.0f;
				sideWalkingSequence[2] = 0.0f;
				sideWalkingSequence[3] = 0.0f;
				sideWalkingSequence[4] = 0.0f;
				sideWalkingSequence[5] = 0.0f;
				sideWalkingSequence[6] = 0.0f;
				sideWalkingSequence[7] = 0.0f;
				sideWalkingSequence[8] = 0.0f;
				sideWalkingSequence[9] = 0.0f;
				
				rotationWalkingSequence[0] = 0.0f;
				rotationWalkingSequence[1] = 0.0f;
				rotationWalkingSequence[2] = -15.0f;
				rotationWalkingSequence[3] = -15.0f;
				rotationWalkingSequence[4] = -15.0f;
				rotationWalkingSequence[5] = -15.0f;
				rotationWalkingSequence[6] = -15.0f;
				rotationWalkingSequence[7] = -15.0f;
				rotationWalkingSequence[8] = -15.0f;
				rotationWalkingSequence[9] = -13.0f;
				rotationWalkingSequence[10] = -10.0f;
				rotationWalkingSequence[11] = -10.0f;
				rotationWalkingSequence[12] = 0.0f;
				rotationWalkingSequence[13] = 0.0f;
				rotationWalkingSequence[14] = 0.0f;
				rotationWalkingSequence[15] = 0.0f;
				rotationWalkingSequence[16] = 0.0f;
				rotationWalkingSequence[17] = 0.0f;
				rotationWalkingSequence[18] = 0.0f;
				rotationWalkingSequence[19] = 0.0f;
				rotationWalkingSequence[20] = 0.0f;
				rotationWalkingSequence[21] = 0.0f;
				rotationWalkingSequence[22] = 0.0f;
				rotationWalkingSequence[23] = 0.0f;
				rotationWalkingSequence[24] = 0.0f;
				rotationWalkingSequence[25] = 0.0f;
				rotationWalkingSequence[26] = 0.0f;
				rotationWalkingSequence[27] = 0.0f;
				rotationWalkingSequence[28] = 0.0f;
				rotationWalkingSequence[29] = 0.0f;
				rotationWalkingSequence[30] = 0.0f;
				rotationWalkingSequence[31] = 0.0f;
				rotationWalkingSequence[32] = 0.0f;
				rotationWalkingSequence[33] = -15.0f;
				rotationWalkingSequence[34] = -15.0f;
				rotationWalkingSequence[35] = -10.0f;
				rotationWalkingSequence[36] = 0.0f;
				rotationWalkingSequence[37] = 0.0f;

			}
			else if(walkingSceneNo == 0x01) //Walk2 (Back to edge)
			{
				numberOfWalking = 23;
				
				lastStep = forwardWalkingSequence[0] = 0.0f; swaySequence[0] = 0.07f;//0.069f;
				forwardWalkingSequence[1] = 0.0f;	swaySequence[1] = 0.0685f;//0.0675f;
				forwardWalkingSequence[2] = 0.0f;	swaySequence[2] = 0.0685f;//0.0675f;
				forwardWalkingSequence[3] = 0.0f;	swaySequence[3] = 0.0685f;//0.0675f;
				forwardWalkingSequence[4] = 0.0f;	swaySequence[4] = 0.0685f;//0.0675f;
				forwardWalkingSequence[5] = 0.0f;	swaySequence[5] = 0.069f;//0.068f;
				forwardWalkingSequence[6] = -0.06f;	swaySequence[6] = 0.068f;//0.067f;
				forwardWalkingSequence[7] = -0.06f;	swaySequence[7] = 0.068f;//0.067f;
				forwardWalkingSequence[8] = -0.06f;	swaySequence[8] = 0.068f;//0.067f;
				forwardWalkingSequence[9] = -0.06f;	swaySequence[9] = 0.068f;//0.067f;
				forwardWalkingSequence[10] = -0.06f;	swaySequence[10] = 0.068f;//0.067f;
				forwardWalkingSequence[11] = -0.06f;	swaySequence[11] = 0.068f;//0.067f;
				forwardWalkingSequence[12] = -0.06f;	swaySequence[12] = 0.068f;//0.067f;
				forwardWalkingSequence[13] = -0.06f;	swaySequence[13] = 0.068f;//0.067f;
				forwardWalkingSequence[14] = -0.06f;	swaySequence[14] = 0.068f;//0.067f;
				forwardWalkingSequence[15] = -0.06f;	swaySequence[15] = 0.068f;//0.067f;
				forwardWalkingSequence[16] = -0.06f;	swaySequence[16] = 0.068f;//0.067f;
				forwardWalkingSequence[17] = -0.06f;	swaySequence[17] = 0.068f;//0.067f;
				forwardWalkingSequence[18] = -0.06f;	swaySequence[18] = 0.068f;//0.067f;
				forwardWalkingSequence[19] = -0.06f;	swaySequence[19] = 0.068f;//0.067f;
				forwardWalkingSequence[20] = -0.06f;	swaySequence[20] = 0.068f;//0.067f;
				forwardWalkingSequence[21] = -0.06f;	swaySequence[21] = 0.068f;//0.067f;
				forwardWalkingSequence[22] = -0.06f;	swaySequence[22] = 0.068f;//0.067f;
				forwardWalkingSequence[23] = 0.0f;	swaySequence[23] = 0.07f;//0.069f;
				forwardWalkingSequence[24] = 0.0f;	swaySequence[24] = 0.07f;//0.069f;
				
				
				sideWalkingSequence[0] = 0.0f;
				sideWalkingSequence[1] = 0.0f;
				sideWalkingSequence[2] = 0.0f;
				sideWalkingSequence[3] = 0.0f;
				sideWalkingSequence[4] = 0.0f;
				sideWalkingSequence[5] = 0.0f;
				sideWalkingSequence[6] = 0.0f;
				sideWalkingSequence[7] = 0.0f;
				sideWalkingSequence[8] = 0.0f;
				sideWalkingSequence[9] = 0.0f;
				
				rotationWalkingSequence[0] = 0.0f;
				rotationWalkingSequence[1] = 15.0f;
				rotationWalkingSequence[2] = 15.0f;
				rotationWalkingSequence[3] = 15.0f;
				rotationWalkingSequence[4] = 15.0f;
				rotationWalkingSequence[5] = 0.0f;
				rotationWalkingSequence[6] = 0.0f;
				rotationWalkingSequence[7] = 0.0f;
				rotationWalkingSequence[8] = 0.0f;
				rotationWalkingSequence[9] = 0.0f;
				rotationWalkingSequence[10] = 0.0f;
				rotationWalkingSequence[11] = 0.0f;
				rotationWalkingSequence[12] = 0.0f;
				rotationWalkingSequence[13] = 0.0f;
				rotationWalkingSequence[14] = 0.0f;
				rotationWalkingSequence[15] = 0.0f;
				rotationWalkingSequence[16] = 0.0f;
				rotationWalkingSequence[17] = 0.0f;
				rotationWalkingSequence[18] = 0.0f;
				rotationWalkingSequence[19] = 0.0f;
				rotationWalkingSequence[20] = 0.0f;
				rotationWalkingSequence[21] = 0.0f;
				rotationWalkingSequence[22] = 0.0f;
				rotationWalkingSequence[23] = 0.0f;
				rotationWalkingSequence[24] = 0.0f;
				
			}
			else if(walkingSceneNo == 0x02) // Walk3 (Go to center again)
			{
				numberOfWalking = 22;
				
				lastStep = forwardWalkingSequence[0] = 0.06f; swaySequence[0] = 0.068f;//0.067f;
				forwardWalkingSequence[1] = 0.06f;	swaySequence[1] = 0.068f;//0.067f;
				forwardWalkingSequence[2] = 0.06f;	swaySequence[2] = 0.0675f;//0.0665f;
				forwardWalkingSequence[3] = 0.06f;	swaySequence[3] = 0.0675f;//0.0665f;
				forwardWalkingSequence[4] = 0.06f;	swaySequence[4] = 0.0675f;//0.0665f;
				forwardWalkingSequence[5] = 0.06f;	swaySequence[5] = 0.068f;//0.067f;
				forwardWalkingSequence[6] = 0.06f;	swaySequence[6] = 0.068f;//0.067f;
				forwardWalkingSequence[7] = 0.06f;	swaySequence[7] = 0.068f;//0.067f;
				forwardWalkingSequence[8] = 0.06f;	swaySequence[8] = 0.068f;//0.067f;
				forwardWalkingSequence[9] = 0.06f;	swaySequence[9] = 0.068f;//0.067f;
				forwardWalkingSequence[10] = 0.06f;	swaySequence[10] = 0.068f;//0.067f;
				forwardWalkingSequence[11] = 0.06f;	swaySequence[11] = 0.068f;//0.067f;
				forwardWalkingSequence[12] = 0.06f;	swaySequence[12] = 0.068f;//0.067f;
				forwardWalkingSequence[13] = 0.06f;	swaySequence[13] = 0.068f;//0.067f;
				forwardWalkingSequence[14] = 0.06f;	swaySequence[14] = 0.068f;//0.067f;
				forwardWalkingSequence[15] = 0.06f;	swaySequence[15] = 0.068f;//0.067f;
				forwardWalkingSequence[16] = 0.06f;	swaySequence[16] = 0.068f;//0.067f;
				forwardWalkingSequence[17] = 0.06f;	swaySequence[17] = 0.068f;//0.067f;
				forwardWalkingSequence[18] = 0.0f;	swaySequence[18] = 0.07f;//0.069f;
				forwardWalkingSequence[19] = 0.0f;	swaySequence[19] = 0.07f;//0.069f;
				forwardWalkingSequence[20] = 0.0f;	swaySequence[20] = 0.07f;//0.069f;
				forwardWalkingSequence[21] = 0.0f;	swaySequence[21] = 0.07f;//0.069f;
				forwardWalkingSequence[22] = 0.0f;	swaySequence[22] = 0.07f;//0.069f;
				forwardWalkingSequence[23] = 0.0f;	swaySequence[23] = 0.07f;//0.069f;
				forwardWalkingSequence[24] = 0.0f;	swaySequence[24] = 0.07f;//0.069f;
				
				
				sideWalkingSequence[0] = 0.0f;
				sideWalkingSequence[1] = 0.0f;
				sideWalkingSequence[2] = 0.0f;
				sideWalkingSequence[3] = 0.0f;
				sideWalkingSequence[4] = 0.0f;
				sideWalkingSequence[5] = 0.0f;
				sideWalkingSequence[6] = 0.0f;
				sideWalkingSequence[7] = 0.0f;
				sideWalkingSequence[8] = 0.0f;
				sideWalkingSequence[9] = 0.0f;
				
				rotationWalkingSequence[0] = 0.0f;
				rotationWalkingSequence[1] = 0.0f;
				rotationWalkingSequence[2] = 0.0f;
				rotationWalkingSequence[3] = 0.0f;
				rotationWalkingSequence[4] = 0.0f;
				rotationWalkingSequence[5] = 0.0f;
				rotationWalkingSequence[6] = 0.0f;
				rotationWalkingSequence[7] = 0.0f;
				rotationWalkingSequence[8] = 0.0f;
				rotationWalkingSequence[9] = 0.0f;
				rotationWalkingSequence[10] = 0.0f;
				rotationWalkingSequence[11] = 0.0f;
				rotationWalkingSequence[12] = 0.0f;
				rotationWalkingSequence[13] = 0.0f;
				rotationWalkingSequence[14] = 0.0f;
				rotationWalkingSequence[15] = 0.0f;
				rotationWalkingSequence[16] = 0.0f;
				rotationWalkingSequence[17] = -15.0f;
				rotationWalkingSequence[18] = -15.0f;
				rotationWalkingSequence[19] = -10.0f;
				rotationWalkingSequence[20] = 0.0f;
				rotationWalkingSequence[21] = 0.0f;
				rotationWalkingSequence[22] = 0.0f;
				rotationWalkingSequence[23] = 0.0f;
				rotationWalkingSequence[24] = 0.0f;
				
			}
			else if(walkingSceneNo == 0x03) //Walk4 (Back to edge again)
			{
				numberOfWalking = 26;
				
				lastStep = forwardWalkingSequence[0] = 0.0f; swaySequence[0] = 0.07f;//0.069f;
				forwardWalkingSequence[1] = 0.0f;	swaySequence[1] = 0.0697f;//0.0687f;  // rotate
				forwardWalkingSequence[2] = 0.0f;	swaySequence[2] = 0.0697f;//0.0687f; // rotate
				forwardWalkingSequence[3] = 0.0f;	swaySequence[3] = 0.0697f;//0.0687f; // rotate
				forwardWalkingSequence[4] = 0.0f;	swaySequence[4] = 0.0697f;//0.0687f; // rotate
				forwardWalkingSequence[5] = 0.0f;	swaySequence[5] = 0.07f;//0.069f;
				forwardWalkingSequence[6] = -0.06f;	swaySequence[6] = 0.069f;//0.068f;
				forwardWalkingSequence[7] = -0.06f;	swaySequence[7] = 0.069f;//0.068f;
				forwardWalkingSequence[8] = -0.06f;	swaySequence[8] = 0.069f;//0.068f;
				forwardWalkingSequence[9] = -0.06f;	swaySequence[9] = 0.069f;//0.068f;
				forwardWalkingSequence[10] = -0.06f;	swaySequence[10] = 0.069f;//0.068f;
				forwardWalkingSequence[11] = -0.06f;	swaySequence[11] = 0.069f;//0.068f;
				forwardWalkingSequence[12] = -0.06f;	swaySequence[12] = 0.069f;//0.068f;
				forwardWalkingSequence[13] = -0.06f;	swaySequence[13] = 0.069f;//0.068f;
				forwardWalkingSequence[14] = -0.06f;	swaySequence[14] = 0.069f;//0.068f;
				forwardWalkingSequence[15] = -0.06f;	swaySequence[15] = 0.069f;//0.068f;
				forwardWalkingSequence[16] = -0.06f;	swaySequence[16] = 0.069f;//0.068f;
				forwardWalkingSequence[17] = -0.06f;	swaySequence[17] = 0.069f;//0.068f;
				forwardWalkingSequence[18] = -0.06f;	swaySequence[18] = 0.069f;//0.068f;
				forwardWalkingSequence[19] = -0.06f;	swaySequence[19] = 0.069f;//0.068f;
				forwardWalkingSequence[20] = -0.06f;	swaySequence[20] = 0.069f;//0.068f;
				forwardWalkingSequence[21] = -0.06f;	swaySequence[21] = 0.069f;//0.068f;
				forwardWalkingSequence[22] = -0.06f;	swaySequence[22] = 0.069f;//0.068f;
				forwardWalkingSequence[23] = 0.0f;	swaySequence[23] = 0.0697f;//0.0687f; // rotate
				forwardWalkingSequence[24] = 0.0f;	swaySequence[24] = 0.0697f;//0.0687f; // rotate
				forwardWalkingSequence[25] = 0.0f;	swaySequence[25] = 0.07f;//0.069f;
				forwardWalkingSequence[26] = 0.0f;	swaySequence[26] = 0.07f;//0.069f;
				forwardWalkingSequence[27] = 0.0f;	swaySequence[27] = 0.07f;//0.069f;
				
				
				sideWalkingSequence[0] = 0.0f;
				sideWalkingSequence[1] = 0.0f;
				sideWalkingSequence[2] = 0.0f;
				sideWalkingSequence[3] = 0.0f;
				sideWalkingSequence[4] = 0.0f;
				sideWalkingSequence[5] = 0.0f;
				sideWalkingSequence[6] = 0.0f;
				sideWalkingSequence[7] = 0.0f;
				sideWalkingSequence[8] = 0.0f;
				sideWalkingSequence[9] = 0.0f;
				
				rotationWalkingSequence[0] = 0.0f;
				rotationWalkingSequence[1] = 15.0f;
				rotationWalkingSequence[2] = 15.0f;
				rotationWalkingSequence[3] = 15.0f;
				rotationWalkingSequence[4] = 15.0f;
				rotationWalkingSequence[5] = 0.0f;
				rotationWalkingSequence[6] = 0.0f;
				rotationWalkingSequence[7] = 0.0f;
				rotationWalkingSequence[8] = 0.0f;
				rotationWalkingSequence[9] = 0.0f;
				rotationWalkingSequence[10] = 0.0f;
				rotationWalkingSequence[11] = 0.0f;
				rotationWalkingSequence[12] = 0.0f;
				rotationWalkingSequence[13] = 0.0f;
				rotationWalkingSequence[14] = 0.0f;
				rotationWalkingSequence[15] = 0.0f;
				rotationWalkingSequence[16] = 0.0f;
				rotationWalkingSequence[17] = 0.0f;
				rotationWalkingSequence[18] = 0.0f;
				rotationWalkingSequence[19] = 0.0f;
				rotationWalkingSequence[20] = 0.0f;
				rotationWalkingSequence[21] = 0.0f;
				rotationWalkingSequence[22] = 0.0f;
				rotationWalkingSequence[23] = -10.0f;
				rotationWalkingSequence[24] = -10.0f;
				rotationWalkingSequence[25] = 0.0f;
				rotationWalkingSequence[26] = 0.0f;
				rotationWalkingSequence[27] = 0.0f;
				
			}
			else if(walkingSceneNo == 0x04) //WalkingDemo
			{
				numberOfWalking = 68;
				
				lastStep = forwardWalkingSequence[0] = 0.0f; swaySequence[0] = 0.07f;//0.069f;
				forwardWalkingSequence[1] = 0.0f;	swaySequence[1] = 0.07f;//0.069f;
				forwardWalkingSequence[2] = 0.0f;	swaySequence[2] = 0.07f;//0.069f;
				forwardWalkingSequence[3] = 0.0f;	swaySequence[3] = 0.07f;//0.069f;
				forwardWalkingSequence[4] = 0.0f;	swaySequence[4] = 0.07f;//0.069f;
				forwardWalkingSequence[5] = 0.075f;	swaySequence[5] = 0.0685f;//0.0675f;
				forwardWalkingSequence[6] = 0.075f;	swaySequence[6] = 0.0685f;//0.0675f;
				forwardWalkingSequence[7] = 0.075f;	swaySequence[7] = 0.0685f;//0.0675f;
				forwardWalkingSequence[8] = 0.075f;	swaySequence[8] = 0.0685f;//0.0675f;
				forwardWalkingSequence[9] = 0.075f;	swaySequence[9] = 0.0685f;//0.0675f;
				forwardWalkingSequence[10] = 0.075f;	swaySequence[10] = 0.0685f;//0.0675f;
				forwardWalkingSequence[11] = 0.075f;	swaySequence[11] = 0.0685f;//0.0675f;
				forwardWalkingSequence[12] = 0.075f;	swaySequence[12] = 0.0685f;//0.0675f;
				forwardWalkingSequence[13] = 0.075f;	swaySequence[13] = 0.0685f;//0.0675f;
				forwardWalkingSequence[14] = 0.075f;	swaySequence[14] = 0.0685f;//0.0675f;
				forwardWalkingSequence[15] = 0.075f;	swaySequence[15] = 0.0685f;//0.0675f;
				forwardWalkingSequence[16] = 0.075f;	swaySequence[16] = 0.0685f;//0.0675f;
				forwardWalkingSequence[17] = 0.075f;	swaySequence[17] = 0.0685f;//0.0675f;
				forwardWalkingSequence[18] = 0.075f;	swaySequence[18] = 0.0685f;//0.0675f;
				forwardWalkingSequence[19] = 0.075f;	swaySequence[19] = 0.0685f;//0.0675f;
				forwardWalkingSequence[20] = 0.075f;	swaySequence[20] = 0.0685f;//0.0675f;
				forwardWalkingSequence[21] = 0.075f;	swaySequence[21] = 0.0685f;//0.0675f;
				forwardWalkingSequence[22] = 0.075f;	swaySequence[22] = 0.0685f;//0.0675f;
				forwardWalkingSequence[23] = 0.075f;	swaySequence[23] = 0.0685f;//0.0675f;
				forwardWalkingSequence[24] = 0.0f;	swaySequence[24] = 0.07f;//0.069f;
				forwardWalkingSequence[25] = 0.0f;	swaySequence[25] = 0.07f;//0.069f;
				forwardWalkingSequence[26] = 0.0f;	swaySequence[26] = 0.07f;//0.069f;
				forwardWalkingSequence[27] = 0.0f;	swaySequence[27] = 0.0695f;//0.0685f;	//rotate
				forwardWalkingSequence[28] = 0.0f;	swaySequence[28] = 0.0695f;//0.0685f;	//rotate
				forwardWalkingSequence[29] = 0.0f;	swaySequence[29] = 0.0695f;//0.0685f;	//rotate
				forwardWalkingSequence[30] = 0.0f;	swaySequence[30] = 0.07f;//0.069f;
				forwardWalkingSequence[31] = 0.0f;	swaySequence[31] = 0.07f;//0.069f;
				forwardWalkingSequence[32] = 0.0f;	swaySequence[32] = 0.07f;//0.069f;
				forwardWalkingSequence[33] = -0.06f;	swaySequence[33] = 0.069f;//0.068f;
				forwardWalkingSequence[34] = -0.06f;	swaySequence[34] = 0.069f;//0.068f;
				forwardWalkingSequence[35] = -0.06f;	swaySequence[35] = 0.069f;//0.068f;
				forwardWalkingSequence[36] = -0.06f;	swaySequence[36] = 0.069f;//0.068f;
				forwardWalkingSequence[37] = -0.06f;	swaySequence[37] = 0.069f;//0.068f;
				forwardWalkingSequence[38] = -0.06f;	swaySequence[38] = 0.069f;//0.068f;
				forwardWalkingSequence[39] = -0.06f;	swaySequence[39] = 0.069f;//0.068f;
				forwardWalkingSequence[40] = -0.06f;	swaySequence[40] = 0.069f;//0.068f;
				forwardWalkingSequence[41] = -0.06f;	swaySequence[41] = 0.069f;//0.068f;
				forwardWalkingSequence[42] = -0.06f;	swaySequence[42] = 0.069f;//0.068f;
				forwardWalkingSequence[43] = -0.06f;	swaySequence[43] = 0.069f;//0.068f;
				forwardWalkingSequence[44] = -0.06f;	swaySequence[44] = 0.069f;//0.068f;
				forwardWalkingSequence[45] = -0.06f;	swaySequence[45] = 0.069f;//0.068f;
				forwardWalkingSequence[46] = -0.06f;	swaySequence[46] = 0.069f;//0.068f;
				forwardWalkingSequence[47] = -0.06f;	swaySequence[47] = 0.069f;//0.068f;
				forwardWalkingSequence[48] = 0.0f;	swaySequence[48] = 0.07f;//0.069f;
				forwardWalkingSequence[49] = 0.0f;	swaySequence[49] = 0.07f;//0.069f;
				forwardWalkingSequence[50] = 0.0f;	swaySequence[50] = 0.0695f;//0.0685f;	//rotate
				forwardWalkingSequence[51] = 0.0f;	swaySequence[51] = 0.069f;//0.068f;	//rotate
				forwardWalkingSequence[52] = 0.0f;	swaySequence[52] = 0.069f;//0.068f;
				forwardWalkingSequence[53] = 0.0f;	swaySequence[53] = 0.069f;//0.068f;
				forwardWalkingSequence[54] = 0.075f;	swaySequence[54] = 0.0685f;//0.0675f;
				forwardWalkingSequence[55] = 0.075f;	swaySequence[55] = 0.0685f;//0.0675f;
				forwardWalkingSequence[56] = 0.075f;	swaySequence[56] = 0.0685f;//0.0675f;
				forwardWalkingSequence[57] = 0.075f;	swaySequence[57] = 0.0685f;//0.0675f;
				forwardWalkingSequence[58] = 0.075f;	swaySequence[58] = 0.0685f;//0.0675f;
				forwardWalkingSequence[59] = 0.075f;	swaySequence[59] = 0.0685f;//0.0675f;
				forwardWalkingSequence[60] = 0.075f;	swaySequence[60] = 0.0685f;//0.0675f;
				forwardWalkingSequence[61] = 0.075f;	swaySequence[61] = 0.0685f;//0.0675f;
				
				forwardWalkingSequence[62] = 0.0f;	swaySequence[62] = 0.07f;//0.069f;
				forwardWalkingSequence[63] = 0.0f;	swaySequence[63] = 0.0695f;//0.0685f;	//rotate
				forwardWalkingSequence[64] = 0.0f;	swaySequence[64] = 0.0695f;//0.0685f;	//rotate
				forwardWalkingSequence[65] = 0.0f;	swaySequence[65] = 0.0695f;//0.0685f;	//rotate
				forwardWalkingSequence[66] = 0.0f;	swaySequence[66] = 0.0695f;//0.0685f;	//rotate
				forwardWalkingSequence[67] = 0.0f;	swaySequence[67] = 0.0695f;//0.0685f;	//rotate
				forwardWalkingSequence[68] = 0.0f;	swaySequence[68] = 0.07f;//0.069f;//rotate
				forwardWalkingSequence[69] = 0.0f;	swaySequence[69] = 0.07f;//0.069f;//rotate
				forwardWalkingSequence[70] = 0.0f;	swaySequence[70] = 0.07f;//0.069f;
				forwardWalkingSequence[71] = 0.0f;	swaySequence[71] = 0.07f;//0.069f;
				
				sideWalkingSequence[0] = 0.0f;
				sideWalkingSequence[1] = 0.0f;
				sideWalkingSequence[2] = 0.0f;
				sideWalkingSequence[3] = 0.0f;
				sideWalkingSequence[4] = 0.0f;
				sideWalkingSequence[5] = 0.0f;
				sideWalkingSequence[6] = 0.0f;
				sideWalkingSequence[7] = 0.0f;
				sideWalkingSequence[8] = 0.0f;
				sideWalkingSequence[9] = 0.0f;
				
				rotationWalkingSequence[0] = 0.0f;
				rotationWalkingSequence[1] = 0.0f;
				rotationWalkingSequence[2] = 0.0f;
				rotationWalkingSequence[3] = 0.0f;
				rotationWalkingSequence[4] = 0.0f;
				rotationWalkingSequence[5] = 0.0f;
				rotationWalkingSequence[6] = 0.0f;
				rotationWalkingSequence[7] = 0.0f;
				rotationWalkingSequence[8] = 0.0f;
				rotationWalkingSequence[9] = 0.0f;
				rotationWalkingSequence[10] = 0.0f;
				rotationWalkingSequence[11] = 0.0f;
				rotationWalkingSequence[12] = 0.0f;
				rotationWalkingSequence[13] = 0.0f;
				rotationWalkingSequence[14] = 0.0f;
				rotationWalkingSequence[15] = 0.0f;
				rotationWalkingSequence[16] = 0.0f;
				rotationWalkingSequence[17] = 0.0f;
				rotationWalkingSequence[18] = 0.0f;
				rotationWalkingSequence[19] = 0.0f;
				rotationWalkingSequence[20] = 0.0f;
				rotationWalkingSequence[21] = 0.0f;
				rotationWalkingSequence[22] = 0.0f;
				rotationWalkingSequence[23] = 0.0f;
				rotationWalkingSequence[24] = 0.0f;
				rotationWalkingSequence[25] = 0.0f;
				rotationWalkingSequence[26] = 0.0f;
				rotationWalkingSequence[27] = -20.0f;
				rotationWalkingSequence[28] = -20.0f;
				rotationWalkingSequence[29] = -20.0f;
				rotationWalkingSequence[30] = 0.0f;
				rotationWalkingSequence[31] = 0.0f;
				rotationWalkingSequence[32] = 0.0f;
				rotationWalkingSequence[33] = 0.0f;
				rotationWalkingSequence[34] = 0.0f;
				rotationWalkingSequence[35] = 0.0f;
				rotationWalkingSequence[36] = 0.0f;
				rotationWalkingSequence[37] = 0.0f;
				rotationWalkingSequence[38] = 0.0f;
				rotationWalkingSequence[39] = 0.0f;
				rotationWalkingSequence[40] = 0.0f;
				rotationWalkingSequence[41] = 0.0f;
				rotationWalkingSequence[42] = 0.0f;
				rotationWalkingSequence[43] = 0.0f;
				rotationWalkingSequence[44] = 0.0f;
				rotationWalkingSequence[45] = 0.0f;
				rotationWalkingSequence[46] = 0.0f;
				rotationWalkingSequence[47] = 0.0f;
				rotationWalkingSequence[48] = 0.0f;
				rotationWalkingSequence[49] = 0.0f;
				rotationWalkingSequence[50] = -20.0f;
				rotationWalkingSequence[51] = -20.0f;
				rotationWalkingSequence[52] = 0.0f;
				rotationWalkingSequence[53] = 0.0f;
				rotationWalkingSequence[54] = 0.0f;
				rotationWalkingSequence[55] = 0.0f;
				rotationWalkingSequence[56] = 0.0f;
				rotationWalkingSequence[57] = 0.0f;
				rotationWalkingSequence[58] = 0.0f;
				rotationWalkingSequence[59] = 0.0f;
				rotationWalkingSequence[60] = 0.0f;
				rotationWalkingSequence[61] = 0.0f;

				rotationWalkingSequence[62] = 0.0f;
				rotationWalkingSequence[63] = 20.0f;
				rotationWalkingSequence[64] = 20.0f;
				rotationWalkingSequence[65] = 20.0f;
				rotationWalkingSequence[66] = 20.0f;
				rotationWalkingSequence[67] = 20.0f;
				rotationWalkingSequence[68] = 5.0f;
				rotationWalkingSequence[69] = 5.0f;
				rotationWalkingSequence[70] = 0.0f;
				rotationWalkingSequence[71] = 0.0f;

				
			}
			else if(walkingSceneNo == 0x05) //Walk6 (rotate CW)
			{
				numberOfWalking = 5;
				
				lastStep = forwardWalkingSequence[0] = 0.0f; swaySequence[0] = 0.07f;//0.069f;
				forwardWalkingSequence[1] = 0.0f;	swaySequence[1] = 0.0685f;//0.0675f;
				forwardWalkingSequence[2] = 0.0f;	swaySequence[2] = 0.0685f;//0.0675f;
				forwardWalkingSequence[3] = 0.0f;	swaySequence[3] = 0.0685f;//0.0675f;
				forwardWalkingSequence[4] = 0.0f;	swaySequence[4] = 0.0685f;//0.0675f;
				forwardWalkingSequence[5] = 0.0f;	swaySequence[5] = 0.07f;//0.069f;
				forwardWalkingSequence[6] = 0.0f;	swaySequence[6] = 0.07f;//0.069f;
				forwardWalkingSequence[7] = 0.0f;	swaySequence[7] = 0.07f;//0.069f;
				forwardWalkingSequence[8] = 0.0f;	swaySequence[8] = 0.07f;//0.069f;
				forwardWalkingSequence[9] = 0.0f;	swaySequence[9] = 0.07f;//0.069f;
				forwardWalkingSequence[10] = 0.0f;	swaySequence[10] = 0.07f;//0.069f;
				forwardWalkingSequence[11] = 0.0f;	swaySequence[11] = 0.07f;//0.069f;
				forwardWalkingSequence[12] = 0.0f;	swaySequence[12] = 0.07f;//0.069f;
				
				
				sideWalkingSequence[0] = 0.0f;
				sideWalkingSequence[1] = 0.0f;
				sideWalkingSequence[2] = 0.0f;
				sideWalkingSequence[3] = 0.0f;
				sideWalkingSequence[4] = 0.0f;
				sideWalkingSequence[5] = 0.0f;
				sideWalkingSequence[6] = 0.0f;
				sideWalkingSequence[7] = 0.0f;
				sideWalkingSequence[8] = 0.0f;
				sideWalkingSequence[9] = 0.0f;
				
				rotationWalkingSequence[0] = 0.0f;
				rotationWalkingSequence[1] = 21.0f;
				rotationWalkingSequence[2] = 21.0f;
				rotationWalkingSequence[3] = 21.0f;
				rotationWalkingSequence[4] = 21.0f;
				rotationWalkingSequence[5] = 0.0f;
				rotationWalkingSequence[6] = 0.0f;
				rotationWalkingSequence[7] = 0.0f;
				rotationWalkingSequence[8] = 0.0f;
				rotationWalkingSequence[9] = 0.0f;
			}
			else if(walkingSceneNo == 0x06) //Walk7 (rotate CCW)
			{
				numberOfWalking = 5;
				
				lastStep = forwardWalkingSequence[0] = 0.0f; swaySequence[0] = 0.07f;//0.069f;
				forwardWalkingSequence[1] = 0.0f;	swaySequence[1] = 0.0685f;//0.0675f;
				forwardWalkingSequence[2] = 0.0f;	swaySequence[2] = 0.0685f;//0.0675f;
				forwardWalkingSequence[3] = 0.0f;	swaySequence[3] = 0.0685f;//0.0675f;
				forwardWalkingSequence[4] = 0.0f;	swaySequence[4] = 0.0685f;//0.0675f;
				forwardWalkingSequence[5] = 0.0f;	swaySequence[5] = 0.07f;//0.069f;
				forwardWalkingSequence[6] = 0.0f;	swaySequence[6] = 0.07f;//0.069f;
				forwardWalkingSequence[7] = 0.0f;	swaySequence[7] = 0.07f;//0.069f;
				forwardWalkingSequence[8] = 0.0f;	swaySequence[8] = 0.07f;//0.069f;
				forwardWalkingSequence[9] = 0.0f;	swaySequence[9] = 0.07f;//0.069f;
				forwardWalkingSequence[10] = 0.0f;	swaySequence[10] = 0.07f;//0.069f;
				forwardWalkingSequence[11] = 0.0f;	swaySequence[11] = 0.07f;//0.069f;
				forwardWalkingSequence[12] = 0.0f;	swaySequence[12] = 0.07f;//0.069f;
				
				
				sideWalkingSequence[0] = 0.0f;
				sideWalkingSequence[1] = 0.0f;
				sideWalkingSequence[2] = 0.0f;
				sideWalkingSequence[3] = 0.0f;
				sideWalkingSequence[4] = 0.0f;
				sideWalkingSequence[5] = 0.0f;
				sideWalkingSequence[6] = 0.0f;
				sideWalkingSequence[7] = 0.0f;
				sideWalkingSequence[8] = 0.0f;
				sideWalkingSequence[9] = 0.0f;
				
				rotationWalkingSequence[0] = 0.0f;
				rotationWalkingSequence[1] = -21.0f;
				rotationWalkingSequence[2] = -21.0f;
				rotationWalkingSequence[3] = -21.0f;
				rotationWalkingSequence[4] = -21.0f;
				rotationWalkingSequence[5] = 0.0f;
				rotationWalkingSequence[6] = 0.0f;
				rotationWalkingSequence[7] = 0.0f;
				rotationWalkingSequence[8] = 0.0f;
				rotationWalkingSequence[9] = 0.0f;
			}
			else;
*/			
			WalkingStartStop = 0x02;
			StepChangeFlag = 0x00;
			if(pSharedMemory->JW_temp[9]>=0)
			{
				WalkingPhaseNextSway = HIP_CENTER_TO_RIGHT_SET;
				//if(pSharedMemory->JW_temp[6]==0) WalkingPhaseNext[X] = WALK_READY;
				//else 
				WalkingPhaseNext[X] = FIRST_LEFT_FOOT_UP_FORWARD_SET;
				WalkingPhaseNext[Y] = FIRST_LEFT_FOOT_UP_SET;
				WalkingPhaseNext[Z] = FIRST_LEFT_FOOT_UP_SET;
				WalkingPhaseNext[Yaw] = FIRST_LEFT_FOOT_UP_SET;
			}
			
			pSharedMemory->MotorControlMode = CTRLMODE_WALKING;
			
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case DEMO_GRASP:
			float userData[5];
			float time[3];

			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
			
			if(pSharedMemory->CommandData == 0x00)	// go to grasping demo position
			{
				if(Joint[WST].MoveFlag == false)
				{
					time[0] = 1000.0f;	time[1] = 0.0f;	time[2] = 0.0f;
					SetMoveJointAngleFunc(LSP, -60.0f, userData, time, 0x00, 0x01);
					SetMoveJointAngleFunc(LEB, -30.0f, userData, time, 0x00, 0x01);
					SetMoveJointAngleFunc(WST, 25.0f, userData, time, 0x00, 0x01);

					SetMoveJointAngleFunc(LSR, 10.0f, userData, time, 0x00, 0x01);
					//SetMoveJointAngleFunc(REB, 0.0f, userData, time, 0x00, 0x01);
					SetMoveJointAngleFunc(LWY, 0.0f, userData, time, 0x00, 0x01);

					
					pSharedMemory->CommandFlag = NO_ACT;
					printf("\ntest1");
				}
			}
			else if(pSharedMemory->CommandData == 0x01)	// show grasping demo
			{
				//pSharedMemory->Data_Debug_Index = 0;

				time[0] = 1000.0f;	time[1] = 0.0f;	time[2] = 0.0f;
				SetMoveJointAngleFunc(LSR, -15.0f, userData, time, 0x00, 0x01);
				SetMoveJointAngleFunc(LEB, -90.0f, userData, time, 0x00, 0x01);
				SetMoveJointAngleFunc(LWY, -100.0f, userData, time, 0x00, 0x01);
				
				time[0] = 2500.0f;	time[1] = 500.0f;	time[2] = 0.0f;
				userData[0] = -40.0f;
				SetMoveJointAngleFunc(WST, 0.0f, userData, time, 0x01, 0x00);
				userData[0] = -30.0f;
				SetMoveJointAngleFunc(LSY, 0.0f, userData, time, 0x02, 0x00);
				
				pSharedMemory->CommandData = 0x00;
				//pSharedMemory->CommandFlag = NO_ACT;
				printf("\ntest2");
			}
			else if(pSharedMemory->CommandData == 0x02)	// go back to home position
			{
				for(i=LF1;i<=LF5;i++) Joint[i].RefVelCurrent = 0;
				MoveJMC(EJMC5);
				RtSleep(10);

				time[0] = 1000.0f;	time[1] = 0.0f;	time[2] = 0.0f;
				SetMoveJointAngleFunc(LSP, 10.0f, userData, time, 0x00, 0x01);
				SetMoveJointAngleFunc(LSR, 10.0f, userData, time, 0x00, 0x01);
				SetMoveJointAngleFunc(LEB, -30.0f, userData, time, 0x00, 0x01);
				SetMoveJointAngleFunc(LWY, 0.0f, userData, time, 0x00, 0x01);
				SetMoveJointAngleFunc(WST, 0.0f, userData, time, 0x00, 0x01);
				
				GoToLimitPos(LF1);
				//GoToLimitPos(RF1);

				pSharedMemory->CommandFlag = NO_ACT;
				
			}
			else;
			break;
		case DEMO_RPS:
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case DEMO_HUG:
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case RBT_ON_MODE:
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case RBT_OFF_MODE:
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		// !----------------------Main Loop CDI Handshake PROG--------------------!
		case CCTM_ON:
			// RSP:12, RSR:13, RSY:14, REB:15, RWY:16, RWP:17	
			ctm.Force_FB_Flag = 0;
			ctm.pwmsend_flag = 0;

			pSharedMemory->MotorControlMode = CTRLMODE_CCTM_CDI;
			pSharedMemory->CommandFlag = NO_ACT;		
			break;
		case CCTM_OFF:
			// RSP:12, RSR:13, RSY:14, REB:15, RWY:16, RWP:17	
			ctm.Force_FB_Flag = 0;
			ctm.pwmsend_flag = 0;

			pSharedMemory->MotorControlMode = CTRLMODE_NONE;
			pSharedMemory->CommandFlag = NO_ACT;		
			break;
		case PWMCMD_CDI_MODE_ON:  // (POS Mode -> PWM Mode)
			// Feedback Turn off
			RBdisableFeedbackControl(CAN1, JMC8);	// RSP, RSR
			RBdisableFeedbackControl(CAN1, JMC9);	// RSY, REB
			RBdisableFeedbackControl(CAN1, EJMC0);	// RWY, RWP

			Joint[RSP].MotorControlMode = 0x02;	// RSP, RSR
			Joint[RSY].MotorControlMode = 0x02;	// RSY, REB
			Joint[RWY].MotorControlMode = 0x02; // RWY, RWP
			
			SetControlMode(Joint[RSP]);	// RSP, RSR
			SetControlMode(Joint[RSY]);	// RSY, REB
			SetControlMode(Joint[RWY]); // RWY, RWP

			// Turn on Feedback
			RBenableFeedbackControl(CAN1, JMC8);	// RSP, RSR
			RBenableFeedbackControl(CAN1, JMC9);	// RSY, REB
			RBenableFeedbackControl(CAN1, EJMC0);	// RWY, RWP

			ctm.pwmsend_flag = 1;
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case PWMCMD_CDI_MODE_OFF:  // (PWM Mode -> POS Mode)
			ctm.pwmsend_flag = 0;
			pSharedMemory->MotorControlMode = CTRLMODE_NONE;

			// PWM Digit Setting
			// RSP:12, RSR:13
			Joint[RSP].PWMOUT = 0;
			Joint[RSP].PWMOUTFlag = 0x00;

			Joint[RSR].PWMOUT = 0;
			Joint[RSR].PWMOUTFlag = 0x00;

			// RSY:14, REB:15
			Joint[RSY].PWMOUT = 0;
			Joint[RSY].PWMOUTFlag = 0x00;

			Joint[REB].PWMOUT = 0;
			Joint[REB].PWMOUTFlag = 0x00;

			
			// RWY:16, RWP:17
			Joint[RWY].PWMOUT = 0;
			Joint[RWY].PWMOUTFlag = 0x00;

			Joint[RWP].PWMOUT = 0;
			Joint[RWP].PWMOUTFlag = 0x00;

			// Send PWM Value
			Hubo2JointCTMPWMCommand2ch(Joint[RSP], Joint[RSR]);
			Hubo2JointCTMPWMCommand2ch(Joint[RSY], Joint[REB]);
			Hubo2JointCTMPWMCommand2ch(Joint[RWY], Joint[RWP]);

			// Set Ref Angle
			MoveJMC(JMC8);	// RSP, RSR
			MoveJMC(JMC9);	// RSY, REB
			MoveJMC(EJMC0);	// RWY, RWP

			// Feedback Turn off
			RBdisableFeedbackControl(CAN1, JMC8);	// RSP, RSR
			RBdisableFeedbackControl(CAN1, JMC9);	// RSY, REB
			RBdisableFeedbackControl(CAN1, EJMC0);	// RWY, RWP

			// JMC Mode Change from PWM -> Pos
			Joint[RSP].MotorControlMode = 0x00;	// RSP, RSR
			Joint[RSY].MotorControlMode = 0x00;	// RSY, REB
			Joint[RWY].MotorControlMode = 0x00; // RWY, RWP

			SetControlMode(Joint[RSP]);	// RSP, RSR
			SetControlMode(Joint[RSY]);	// RSY, REB
			SetControlMode(Joint[RWY]); // RWY, RWP

			// Turn on Feedback
			RBenableFeedbackControl(CAN1, JMC8);	// RSP, RSR
			RBenableFeedbackControl(CAN1, JMC9);	// RSY, REB
			RBenableFeedbackControl(CAN1, EJMC0);	// RWY, RWP
			
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case PWMCMD_FORCE_FB_ON:  // Right Hand Force Feedback On
			ctm.Force_FB_Flag = 1;

			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case PWMCMD_FORCE_FB_OFF:  // Right Hand Force Feedback Off
			ctm.Force_FB_Flag = 0;

			pSharedMemory->CommandFlag = NO_ACT;
			break;
		// !--------------------------------------------------------------!
		case INIT_WB_OFFLINE_DEMO:			// by Inhyeok
			if(pSharedMemory->WB_DemoRunFlag == 0)
			{
				if(_PassiveUpdatedFlag == 1 && _WBDemoIsWaitingFlag == 1)
				{
					RBrequestEncoderPosition(CAN0, Joint[RAP].JMC, 0x00, 1);
					RBrequestEncoderPosition(CAN0, Joint[LAP].JMC, 0x00, 1);

					RBrequestEncoderPosition(CAN1, Joint[LSP].JMC, 0x00,1);
					RBrequestEncoderPosition(CAN1, Joint[LSY].JMC, 0x00,1);
					RBrequestEncoderPosition(CAN1, Joint[LWY].JMC, 0x00,1);
					RBrequestEncoderPosition(CAN1, Joint[LWY2].JMC, 0x00,1);
					RBrequestEncoderPosition(CAN1, Joint[NKY].JMC, 0x00,1);						
				
					RBrequestEncoderPosition(CAN1, Joint[RSP].JMC, 0x00,1);
					RBrequestEncoderPosition(CAN1, Joint[RSY].JMC, 0x00,1);	
					RBrequestEncoderPosition(CAN1, Joint[RWY].JMC, 0x00,1);
					RBrequestEncoderPosition(CAN1, Joint[RWY2].JMC, 0x00,1);
					
					_WBDemoIsWaitingFlag = 0;
					pSharedMemory->WB_FreqTest_StartFlag = 0;

					_Xhat_DSP_F_4x1[1] = 0.;
					_Xhat_DSP_F_4x1[2] = 0.;
					_Xhat_DSP_F_4x1[3] = 0.;
					_Xhat_DSP_F_4x1[4] = 0.;
					_Xhat_DSP_S_4x1[1] = 0.;
					_Xhat_DSP_S_4x1[2] = 0.;
					_Xhat_DSP_S_4x1[3] = 0.;
					_Xhat_DSP_S_4x1[4] = 0.;
					
					_Xhat_SSP_F_4x1[1] = 0.;
					_Xhat_SSP_F_4x1[2] = 0.;
					_Xhat_SSP_F_4x1[3] = 0.;
					_Xhat_SSP_F_4x1[4] = 0.;
					_Xhat_SSP_S_4x1[1] = 0.;
					_Xhat_SSP_S_4x1[2] = 0.;
					_Xhat_SSP_S_4x1[3] = 0.;
					_Xhat_SSP_S_4x1[4] = 0.;
					
					pSharedMemory->WB_StartFlag = 0;
					_OfflineCount = 1;				
					
					LoadParameter_ComputedTorque();
					
					switch(pSharedMemory->WB_DemoNo)
					{
					case WB_OFFLINE_DEMO_DRC_TEST:
						if(LoadOfflineTraj(8)==0 && LoadParameter_ComputedTorque()==0)
						//if(LoadOfflineTraj(9)==0 && LoadParameter_ComputedTorque()==0) // no step
							pSharedMemory->MotorControlMode = CTRLMODE_WB_OFFLINE_DEMO;
						else
							pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
						break;
					case WB_FREQ_TEST:
					case WB_STEERING_TEST:
					case WB_LADDER_CLIMBING_TEST:
					case WB_QUAD_KIRK:
					case WB_BIPED_KIRK:
						init_kirk();
						if(LoadParameter_ComputedTorque()==0)
						{
							RBsetMaxDuty(Joint[RSP].CAN_channel, Joint[RSP].JMC, 40, 40, 100);
							RBsetMaxDuty(Joint[RSY].CAN_channel, Joint[RSY].JMC, 40, 40, 100);
							RBsetMaxDuty(Joint[RWY].CAN_channel, Joint[RWY].JMC, 40, 40, 100);
							RBsetMaxDuty(Joint[LSP].CAN_channel, Joint[LSP].JMC, 40, 40, 100);
							RBsetMaxDuty(Joint[LSY].CAN_channel, Joint[LSY].JMC, 40, 40, 100);
							RBsetMaxDuty(Joint[LWY].CAN_channel, Joint[LWY].JMC, 40, 40, 100);						
							RBsetMaxDuty(Joint[RAP].CAN_channel, Joint[RAP].JMC, 40, 40, 100);
							RBsetMaxDuty(Joint[LAP].CAN_channel, Joint[LAP].JMC, 40, 40, 100);

							RBsetFrictionParameter(Joint[RSP].CAN_channel, Joint[RSP].JMC, (unsigned int)(_friction_para[RSP_33][0]*Joint[RSP].PPR/1000.f), (unsigned int)(_friction_para[RSP_33][1]),	(unsigned int)(_friction_para[RSR_33][0]*Joint[RSR].PPR/1000.f), (unsigned int)(_friction_para[RSR_33][1]));
							RBsetFrictionParameter(Joint[RSY].CAN_channel, Joint[RSY].JMC, (unsigned int)(_friction_para[RSY_33][0]*Joint[RSY].PPR/1000.f), (unsigned int)(_friction_para[RSY_33][1]),	(unsigned int)(_friction_para[REB_33][0]*Joint[REB].PPR/1000.f), (unsigned int)(_friction_para[REB_33][1]));
							RBsetFrictionParameter(Joint[RWY].CAN_channel, Joint[RWY].JMC, (unsigned int)(_friction_para[RWY_33][0]*Joint[RWY].PPR/1000.f), (unsigned int)(_friction_para[RWY_33][1]),	(unsigned int)(_friction_para[RWP_33][0]*Joint[RWP].PPR/1000.f), (unsigned int)(_friction_para[RWP_33][1]));
							RBsetFrictionParameter(Joint[LSP].CAN_channel, Joint[LSP].JMC, (unsigned int)(_friction_para[LSP_33][0]*Joint[LSP].PPR/1000.f), (unsigned int)(_friction_para[LSP_33][1]),	(unsigned int)(_friction_para[LSR_33][0]*Joint[LSR].PPR/1000.f), (unsigned int)(_friction_para[LSR_33][1]));
							RBsetFrictionParameter(Joint[LSY].CAN_channel, Joint[LSY].JMC, (unsigned int)(_friction_para[LSY_33][0]*Joint[LSY].PPR/1000.f), (unsigned int)(_friction_para[LSY_33][1]),	(unsigned int)(_friction_para[LEB_33][0]*Joint[LEB].PPR/1000.f), (unsigned int)(_friction_para[LEB_33][1]));
							RBsetFrictionParameter(Joint[LWY].CAN_channel, Joint[LWY].JMC, (unsigned int)(_friction_para[LWY_33][0]*Joint[LWY].PPR/1000.f), (unsigned int)(_friction_para[LWY_33][1]),	(unsigned int)(_friction_para[LWP_33][0]*Joint[LWP].PPR/1000.f), (unsigned int)(_friction_para[LWP_33][1]));
							RBsetFrictionParameter(Joint[RAP].CAN_channel, Joint[RAP].JMC, (unsigned int)(_friction_para[RAP_33][0]*Joint[RAP].PPR/1000.f), (unsigned int)(_friction_para[RAP_33][1]),	(unsigned int)(_friction_para[RAR_33][0]*Joint[RAR].PPR/1000.f), (unsigned int)(_friction_para[RAR_33][1]));
							RBsetFrictionParameter(Joint[LAP].CAN_channel, Joint[LAP].JMC, (unsigned int)(_friction_para[LAP_33][0]*Joint[LAP].PPR/1000.f), (unsigned int)(_friction_para[LAP_33][1]),	(unsigned int)(_friction_para[LAR_33][0]*Joint[LAR].PPR/1000.f), (unsigned int)(_friction_para[LAR_33][1]));
							pSharedMemory->MotorControlMode = CTRLMODE_WB_OFFLINE_DEMO;
						}
						else
							pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
						break;					
					default:
						pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
						break;
					}
					pSharedMemory->CommandFlag = NO_ACT;
				}
				else
				{
					if(fabs(Joint[RHY].RefAngleCurrent-DRC_BR_RHY) < 0.0001f &&
						(fabs(Joint[RHR].RefAngleCurrent-DRC_BR_RHR) < 0.0001f || fabs(Joint[RHR].RefAngleCurrent-DRC_BR_RHR_WIDE) < 0.0001f) &&
						fabs(Joint[RHP].RefAngleCurrent-DRC_BR_RHP) < 0.0001f &&
						fabs(Joint[RKN].RefAngleCurrent-DRC_BR_RKN) < 0.0001f &&
						fabs(Joint[RAP].RefAngleCurrent-DRC_BR_RAP) < 0.0001f &&
						(fabs(Joint[RAR].RefAngleCurrent-DRC_BR_RAR) < 0.0001f || fabs(Joint[RAR].RefAngleCurrent-DRC_BR_RAR_WIDE) < 0.0001f) &&
						fabs(Joint[LHY].RefAngleCurrent-DRC_BR_LHY) < 0.0001f &&
						(fabs(Joint[LHR].RefAngleCurrent-DRC_BR_LHR) < 0.0001f || fabs(Joint[LHR].RefAngleCurrent-DRC_BR_LHR_WIDE) < 0.0001f) &&
						fabs(Joint[LHP].RefAngleCurrent-DRC_BR_LHP) < 0.0001f &&
						fabs(Joint[LKN].RefAngleCurrent-DRC_BR_LKN) < 0.0001f &&
						fabs(Joint[LAP].RefAngleCurrent-DRC_BR_LAP) < 0.0001f &&
						(fabs(Joint[LAR].RefAngleCurrent-DRC_BR_LAR) < 0.0001f || fabs(Joint[LAR].RefAngleCurrent-DRC_BR_LAR_WIDE) < 0.0001f) &&
						fabs(Joint[WST].RefAngleCurrent-DRC_BR_WST) < 0.0001f &&
						fabs(Joint[RSP].RefAngleCurrent-DRC_BR_RSP) < 0.0001f &&
						fabs(Joint[RSR].RefAngleCurrent-(DRC_BR_RSR-OFFSET_RSR)) < 0.0001f &&
						fabs(Joint[RSY].RefAngleCurrent-DRC_BR_RSY) < 0.0001f &&
						fabs(Joint[REB].RefAngleCurrent-(DRC_BR_REB-OFFSET_REB)) < 0.0001f &&
						fabs(Joint[RWY].RefAngleCurrent-DRC_BR_RWY) < 0.0001f &&						
						fabs(Joint[LSP].RefAngleCurrent-DRC_BR_LSP) < 0.0001f &&
						fabs(Joint[LSR].RefAngleCurrent-(DRC_BR_LSR-OFFSET_LSR)) < 0.0001f &&
						fabs(Joint[LSY].RefAngleCurrent-DRC_BR_LSY) < 0.0001f &&
						fabs(Joint[LEB].RefAngleCurrent-(DRC_BR_LEB-OFFSET_LEB)) < 0.0001f &&
						fabs(Joint[LWY].RefAngleCurrent-DRC_BR_LWY) < 0.0001f)
					{
						if(fabs(Joint[RWP].RefAngleCurrent-DRC_BR_RWP) < 0.0001f && fabs(Joint[LWP].RefAngleCurrent-DRC_BR_LWP) < 0.0001f) // biped
						{
							_WBDemoIsWaitingFlag = 1;
							_DRC_walking_mode = DRC_BI_MODE;	// DRC walking
							LoadControllerParamter_DRC(_DRC_walking_mode);
							if(_PassiveUpdatedFlag==0)
								UpdateGlobalMotionVariables();
							printf("\n Starting DRC-biped motion...");						
						}
						else if(fabs(Joint[RWP].RefAngleCurrent-DRC_QR_RWP) < 0.0001f && 
								fabs(Joint[LWP].RefAngleCurrent-DRC_QR_LWP) < 0.0001f &&
								fabs(Joint[RWY2].RefAngleCurrent-DRC_QR_RWY2) < 0.0001f &&
								fabs(Joint[LWY2].RefAngleCurrent-DRC_QR_LWY2) < 0.0001f) // quadruped
						{
							_WBDemoIsWaitingFlag = 1;
							_DRC_walking_mode = DRC_QUAD_MODE;	// DRC walking
							LoadControllerParamter_DRC(_DRC_walking_mode);
							if(_PassiveUpdatedFlag==0)
								UpdateGlobalMotionVariables();
							printf("\n Starting DRC-quadruped motion...");						
						}
					}
					else if(fabs(Joint[RKN].RefAngleCurrent-Joint[RKN].WalkReadyAngle) < 10.f &&
						fabs(Joint[LKN].RefAngleCurrent-Joint[LKN].WalkReadyAngle) < 10.f &&
						fabs(Joint[WST].RefAngleCurrent-Joint[WST].WalkReadyAngle) < 0.0001f &&
						fabs(Joint[RSP].RefAngleCurrent-Joint[RSP].WalkReadyAngle) < 0.0001f &&
						fabs(Joint[RSR].RefAngleCurrent-Joint[RSR].WalkReadyAngle) < 0.0001f &&
						fabs(Joint[RSY].RefAngleCurrent-Joint[RSY].WalkReadyAngle) < 0.0001f &&
						fabs(Joint[REB].RefAngleCurrent-Joint[REB].WalkReadyAngle) < 0.0001f &&
						fabs(Joint[RWY].RefAngleCurrent-Joint[RWY].WalkReadyAngle) < 0.0001f &&
						fabs(Joint[RWP].RefAngleCurrent-Joint[RWP].WalkReadyAngle) < 0.0001f &&
						fabs(Joint[LSP].RefAngleCurrent-Joint[LSP].WalkReadyAngle) < 0.0001f &&
						fabs(Joint[LSR].RefAngleCurrent-Joint[LSR].WalkReadyAngle) < 0.0001f &&
						fabs(Joint[LSY].RefAngleCurrent-Joint[LSY].WalkReadyAngle) < 0.0001f &&
						fabs(Joint[LEB].RefAngleCurrent-Joint[LEB].WalkReadyAngle) < 0.0001f &&
						fabs(Joint[LWY].RefAngleCurrent-Joint[LWY].WalkReadyAngle) < 0.0001f &&
						fabs(Joint[LWP].RefAngleCurrent-Joint[LWP].WalkReadyAngle) < 0.0001f &&
						_WalkReadyAngleSetFlag == 1)
					{
						//_WBDemoIsWaitingFlag = 1;
						//_DRC_walking_mode = DRC_BI_MODE;
						//LoadControllerParamter_DRC(_DRC_walking_mode);
						//if(_PassiveUpdatedFlag==0)
						//	UpdateGlobalMotionVariables();
						printf("\n Not in the ready position...");
						pSharedMemory->CommandFlag = NO_ACT;
					}
					else if(fabs(Joint[RHY].RefAngleCurrent-DRC_BR_RHY) < 0.0001f &&
						fabs(Joint[RHR].RefAngleCurrent-DRC_BR_RHR) < 0.0001f &&
						fabs(Joint[RHP].RefAngleCurrent-DRC_BR_RHP) < 0.0001f &&
						fabs(Joint[RKN].RefAngleCurrent-DRC_BR_RKN) < 0.0001f &&
						fabs(Joint[RAP].RefAngleCurrent-DRC_BR_RAP) < 0.0001f &&
						fabs(Joint[RAR].RefAngleCurrent-DRC_BR_RAR) < 0.0001f &&
						fabs(Joint[LHY].RefAngleCurrent-DRC_BR_LHY) < 0.0001f &&
						fabs(Joint[LHR].RefAngleCurrent-DRC_BR_LHR) < 0.0001f &&
						fabs(Joint[LHP].RefAngleCurrent-DRC_BR_LHP) < 0.0001f &&
						fabs(Joint[LKN].RefAngleCurrent-DRC_BR_LKN) < 0.0001f &&
						fabs(Joint[LAP].RefAngleCurrent-DRC_BR_LAP) < 0.0001f &&
						fabs(Joint[LAR].RefAngleCurrent-DRC_BR_LAR) < 0.0001f &&
						fabs(Joint[WST].RefAngleCurrent-DRC_BR_WST) < 0.0001f &&
						fabs(Joint[RSP].RefAngleCurrent-DRC_BR_RSP) < 0.0001f &&
						fabs(Joint[RSR].RefAngleCurrent-(DRC_BR_RSR-OFFSET_RSR)) < 0.0001f &&
						fabs(Joint[RSY].RefAngleCurrent-DRC_BR_RSY) < 0.0001f &&
						fabs(Joint[REB].RefAngleCurrent-(DRC_BR_REB-OFFSET_REB)) < 0.0001f &&
						fabs(Joint[RWY].RefAngleCurrent-DRC_BR_RWY) < 0.0001f &&
						fabs(Joint[RWP].RefAngleCurrent-DRC_BR_RWP) < 0.0001f &&
						fabs(Joint[LSP].RefAngleCurrent-pSharedMemory->ref_lsp) < 0.0001f &&
						fabs(Joint[LSR].RefAngleCurrent-(pSharedMemory->ref_lsr-OFFSET_LSR)) < 0.0001f &&
						fabs(Joint[LSY].RefAngleCurrent-pSharedMemory->ref_lsy) < 0.0001f &&
						fabs(Joint[LEB].RefAngleCurrent-(pSharedMemory->ref_leb-OFFSET_LEB)) < 0.0001f &&
						fabs(Joint[LWY].RefAngleCurrent-pSharedMemory->ref_lwy) < 0.0001f &&
						fabs(Joint[LWP].RefAngleCurrent-pSharedMemory->ref_lwp) < 0.0001f)
					{
						_WBDemoIsWaitingFlag = 1;
						if(_PassiveUpdatedFlag==0)
							UpdateGlobalMotionVariables();
						printf("\n Starting steering-test...");						
					}
					else if(fabs(Joint[RHY].RefAngleCurrent-pSharedMemory->ref_rhy) < 0.0001f &&
						fabs(Joint[RHR].RefAngleCurrent-pSharedMemory->ref_rhr) < 0.0001f &&
						fabs(Joint[RHP].RefAngleCurrent-pSharedMemory->ref_rhp) < 0.0001f &&
						fabs(Joint[RKN].RefAngleCurrent-pSharedMemory->ref_rkn) < 0.0001f &&
						fabs(Joint[RAP].RefAngleCurrent-pSharedMemory->ref_rap) < 0.0001f &&
						fabs(Joint[RAR].RefAngleCurrent-pSharedMemory->ref_rar) < 0.0001f &&
						fabs(Joint[LHY].RefAngleCurrent-pSharedMemory->ref_lhy) < 0.0001f &&
						fabs(Joint[LHR].RefAngleCurrent-pSharedMemory->ref_lhr) < 0.0001f &&
						fabs(Joint[LHP].RefAngleCurrent-pSharedMemory->ref_lhp) < 0.0001f &&
						fabs(Joint[LKN].RefAngleCurrent-pSharedMemory->ref_lkn) < 0.0001f &&
						fabs(Joint[LAP].RefAngleCurrent-pSharedMemory->ref_lap) < 0.0001f &&
						fabs(Joint[LAR].RefAngleCurrent-pSharedMemory->ref_lar) < 0.0001f &&
						fabs(Joint[WST].RefAngleCurrent-DRC_BR_WST) < 0.0001f &&
						fabs(Joint[RSP].RefAngleCurrent-pSharedMemory->ref_rsp) < 0.0001f &&
						fabs(Joint[RSR].RefAngleCurrent-(pSharedMemory->ref_rsr-OFFSET_RSR)) < 0.0001f &&
						fabs(Joint[RSY].RefAngleCurrent-pSharedMemory->ref_rsy) < 0.0001f &&
						fabs(Joint[REB].RefAngleCurrent-(pSharedMemory->ref_reb-OFFSET_REB)) < 0.0001f &&
						fabs(Joint[RWY].RefAngleCurrent-pSharedMemory->ref_rwy) < 0.0001f &&
						fabs(Joint[RWP].RefAngleCurrent-pSharedMemory->ref_rwp) < 0.0001f &&
						fabs(Joint[LSP].RefAngleCurrent-pSharedMemory->ref_lsp) < 0.0001f &&
						fabs(Joint[LSR].RefAngleCurrent-(pSharedMemory->ref_lsr-OFFSET_LSR)) < 0.0001f &&
						fabs(Joint[LSY].RefAngleCurrent-pSharedMemory->ref_lsy) < 0.0001f &&
						fabs(Joint[LEB].RefAngleCurrent-(pSharedMemory->ref_leb-OFFSET_LEB)) < 0.0001f &&
						fabs(Joint[LWY].RefAngleCurrent-pSharedMemory->ref_lwy) < 0.0001f &&
						fabs(Joint[LWP].RefAngleCurrent-pSharedMemory->ref_lwp) < 0.0001f)
					{
						_WBDemoIsWaitingFlag = 1;
						if(_PassiveUpdatedFlag==0)
							UpdateGlobalMotionVariables();
						printf("\n Starting ladder climbing or quad test...");						
					}
					else
					{
						_WBDemoIsWaitingFlag = 0;
						_PassiveUpdatedFlag = 0;
						printf("\n Not in the ready position!!!");	
						printf("\n RHY: %g", Joint[RHY].RefAngleCurrent);
						printf("\n RHR: %g", Joint[RHR].RefAngleCurrent);
						printf("\n RHP: %g", Joint[RHP].RefAngleCurrent);
						printf("\n RKN: %g", Joint[RKN].RefAngleCurrent);
						printf("\n RAP: %g", Joint[RAP].RefAngleCurrent);
						printf("\n RAR: %g", Joint[RAR].RefAngleCurrent);
						printf("\n LHY: %g", Joint[LHY].RefAngleCurrent);
						printf("\n LHR: %g", Joint[LHR].RefAngleCurrent);
						printf("\n LHP: %g", Joint[LHP].RefAngleCurrent);
						printf("\n LKN: %g", Joint[LKN].RefAngleCurrent);
						printf("\n LAP: %g", Joint[LAP].RefAngleCurrent);
						printf("\n LAR: %g", Joint[LAR].RefAngleCurrent);
						printf("\n RSP: %g", Joint[RSP].RefAngleCurrent);
						printf("\n RSR: %g", Joint[RSR].RefAngleCurrent);
						printf("\n RSY: %g", Joint[RSY].RefAngleCurrent);
						printf("\n REB: %g", Joint[REB].RefAngleCurrent);
						printf("\n RWY: %g", Joint[RWY].RefAngleCurrent);
						printf("\n RWP: %g", Joint[RWP].RefAngleCurrent);
						printf("\n LSP: %g", Joint[LSP].RefAngleCurrent);
						printf("\n LSR: %g", Joint[LSR].RefAngleCurrent);
						printf("\n LSY: %g", Joint[LSY].RefAngleCurrent);
						printf("\n LEB: %g", Joint[LEB].RefAngleCurrent);
						printf("\n LWY: %g", Joint[LWY].RefAngleCurrent);
						printf("\n LWP: %g", Joint[LWP].RefAngleCurrent);
						printf("\n WST: %g", Joint[WST].RefAngleCurrent);

						pSharedMemory->CommandFlag = NO_ACT;
					}
				}				
			}
			else
			{
				printf("\n Other WB demo is running!!");
				pSharedMemory->CommandFlag = NO_ACT;
			}				
			break;
		case BACK_TO_WB_INITIAL_ANGLE:  // by Inhyeok
			pSharedMemory->WB_DemoRunFlag = 0;
			_PassiveUpdatedFlag = 0;
			Back2InitAngle();
			pSharedMemory->CommandFlag = NO_ACT;
			break;
		case EMERGENCY_STOP: // by Inhyeok
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
			pSharedMemory->WB_DemoRunFlag = 0;
			_PassiveUpdatedFlag = 0;
			
			Joint[NKY].RefVelCurrent = 0.;
			Joint[NK1].RefVelCurrent = 0.;
			Joint[NK2].RefVelCurrent = 0.;

			Joint[RF1].RefVelCurrent = 0.;
			Joint[RF2].RefVelCurrent = 0.;
			Joint[RF3].RefVelCurrent = 0.;
			Joint[RF4].RefVelCurrent = 0.;
			Joint[RF5].RefVelCurrent = 0.;

			Joint[LF1].RefVelCurrent = 0.;
			Joint[LF2].RefVelCurrent = 0.;
			Joint[LF3].RefVelCurrent = 0.;
			Joint[LF4].RefVelCurrent = 0.;
			Joint[LF5].RefVelCurrent = 0.;

			Joint[RSP].RefAngleCurrent = Joint[RSP].EncoderValue/Joint[RSP].PPR;
			Joint[RSR].RefAngleCurrent = Joint[RSR].EncoderValue/Joint[RSR].PPR;
			SendRunStopCMD(Joint[RSP].JMC, 0x01);

			Joint[RSY].RefAngleCurrent = Joint[RSY].EncoderValue/Joint[RSY].PPR;
			Joint[REB].RefAngleCurrent = Joint[REB].EncoderValue/Joint[REB].PPR;
			SendRunStopCMD(Joint[RSY].JMC, 0x01);

			Joint[RWY].RefAngleCurrent = Joint[RWY].EncoderValue/Joint[RWY].PPR;
			Joint[RWP].RefAngleCurrent = Joint[RWP].EncoderValue/Joint[RWP].PPR;
			SendRunStopCMD(Joint[RWY].JMC, 0x01);

			Joint[RWY2].RefAngleCurrent = Joint[RWY2].EncoderValue/Joint[RWY2].PPR;
			SendRunStopCMD(Joint[RWY2].JMC, 0x01);
			
			Joint[LSP].RefAngleCurrent = Joint[LSP].EncoderValue/Joint[LSP].PPR;
			Joint[LSR].RefAngleCurrent = Joint[LSR].EncoderValue/Joint[LSR].PPR;
			SendRunStopCMD(Joint[LSP].JMC, 0x01);

			Joint[LSY].RefAngleCurrent = Joint[LSY].EncoderValue/Joint[LSY].PPR;
			Joint[LEB].RefAngleCurrent = Joint[LEB].EncoderValue/Joint[LEB].PPR;
			SendRunStopCMD(Joint[LSY].JMC, 0x01);

			Joint[LWY].RefAngleCurrent = Joint[LWY].EncoderValue/Joint[LWY].PPR;
			Joint[LWP].RefAngleCurrent = Joint[LWP].EncoderValue/Joint[LWP].PPR;
			SendRunStopCMD(Joint[LWY].JMC, 0x01);

			Joint[LWY2].RefAngleCurrent = Joint[LWY2].EncoderValue/Joint[LWY2].PPR;
			SendRunStopCMD(Joint[LWY2].JMC, 0x01);

			Joint[RAP].RefAngleCurrent = Joint[RAP].EncoderValue/Joint[RAP].PPR;
			Joint[RAR].RefAngleCurrent = Joint[RAR].EncoderValue/Joint[RAR].PPR;
			SendRunStopCMD(Joint[RAP].JMC, 0x01);

			Joint[LAP].RefAngleCurrent = Joint[LAP].EncoderValue/Joint[LAP].PPR;
			Joint[LAR].RefAngleCurrent = Joint[LAR].EncoderValue/Joint[LAR].PPR;
			SendRunStopCMD(Joint[LAP].JMC, 0x01);

			sw_mode_count = 0;
			pSharedMemory->sw_mode = SW_MODE_COMPLEMENTARY;
			pSharedMemory->CommandFlag = SET_SW_MODE;
		//	pSharedMemory->CommandFlag = NO_ACT;
			break;
		case SET_SW_MODE: // by Inhyeok			
			if(pSharedMemory->sw_mode == SW_MODE_COMPLEMENTARY)
			{
				if(sw_mode_count%100 == 0)
				{
					RBsetSwitchingMode(CAN1, Joint[RSP].JMC, SW_MODE_COMPLEMENTARY);
					RBsetSwitchingMode(CAN1, Joint[LSP].JMC, SW_MODE_COMPLEMENTARY);
					RBsetSwitchingMode(CAN0, Joint[RAP].JMC, SW_MODE_COMPLEMENTARY);					
				}
				else if(sw_mode_count%100 == 50)
				{
					RBsetSwitchingMode(CAN1, Joint[RSY].JMC, SW_MODE_COMPLEMENTARY);
					RBsetSwitchingMode(CAN1, Joint[LSY].JMC, SW_MODE_COMPLEMENTARY);
					RBsetSwitchingMode(CAN0, Joint[LAP].JMC, SW_MODE_COMPLEMENTARY);
				}
				else if(sw_mode_count%100 == 99)
				{
					RBsetSwitchingMode(CAN1, Joint[RWY].JMC, SW_MODE_COMPLEMENTARY);
					RBsetSwitchingMode(CAN1, Joint[LWY].JMC, SW_MODE_COMPLEMENTARY);

					printf("\n complementary switching mode - RSP,RSR,RSY,REB,RWY,RWP, LSP,LSR,LSY,LEB,LWY,LWP, RAP,RAR, LAP,LAR\n");
					sw_mode_count = -1;
					pSharedMemory->CommandFlag = NO_ACT;
				}
			}
			else// if(pSharedMemory->sw_mode == SW_MODE_NON_COMPLEMENTARY)
			{
				if(sw_mode_count%100 == 0)
				{
					RBsetSwitchingMode(CAN1, Joint[RSP].JMC, SW_MODE_NON_COMPLEMENTARY);
					RBsetSwitchingMode(CAN1, Joint[LSP].JMC, SW_MODE_NON_COMPLEMENTARY);
					RBsetSwitchingMode(CAN0, Joint[RAP].JMC, SW_MODE_NON_COMPLEMENTARY);
				}
				else if(sw_mode_count%100 == 50)
				{
					RBsetSwitchingMode(CAN1, Joint[RSY].JMC, SW_MODE_NON_COMPLEMENTARY);
					RBsetSwitchingMode(CAN1, Joint[LSY].JMC, SW_MODE_NON_COMPLEMENTARY);
					RBsetSwitchingMode(CAN0, Joint[LAP].JMC, SW_MODE_NON_COMPLEMENTARY);
				}
				else if(sw_mode_count%100 == 99)
				{
					RBsetSwitchingMode(CAN1, Joint[RWY].JMC, SW_MODE_NON_COMPLEMENTARY);
					RBsetSwitchingMode(CAN1, Joint[LWY].JMC, SW_MODE_NON_COMPLEMENTARY);

					printf("\n non-complementary switching mode - RSP,RSR,RSY,REB,RWY,RWP, LSP,LSR,LSY,LEB,LWY,LWP, RAP,RAR, LAP,LAR\n");
					sw_mode_count = -1;
					pSharedMemory->CommandFlag = NO_ACT;
				}
			}
			sw_mode_count++;
			//pSharedMemory->CommandFlag = NO_ACT;
			break;
		case CHANGE_DRC_HAND: // by inhyeok
			_PassiveUpdatedFlag = 0;
			pSharedMemory->WB_DemoRunFlag = 0;
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;			
			if(pSharedMemory->change_drc_hand_flag == DRC_HAND_MODE)
			{
				SetMoveJointAngle(RWP, 0.f, 3000.f, 0x01);
				SetMoveJointAngle(LWP, 0.f, 3000.f, 0x01);
			}
			else if(pSharedMemory->change_drc_hand_flag == DRC_STICK_MODE)// stick mode
			{
				SetMoveJointAngle(RWP, DRC_QR_RWP, 3000.f, 0x01);
				SetMoveJointAngle(LWP, DRC_QR_LWP, 3000.f, 0x01);
				
			}
			else if(pSharedMemory->change_drc_hand_flag == DRC_90DEG_WRIST_YAW2)
			{
				SetMoveJointAngle(RWY2, DRC_QR_RWY2, 3000.f, 0x01);
				SetMoveJointAngle(LWY2, DRC_QR_LWY2, 3000.f, 0x01);
			}
			else if(pSharedMemory->change_drc_hand_flag == DRC_ZERO_WRIST_YAW2)
			{
				SetMoveJointAngle(RWY2, 0.f, 3000.f, 0x01);
				SetMoveJointAngle(LWY2, 0.f, 3000.f, 0x01);
			}
			pSharedMemory->CommandFlag = NO_ACT;			
			break;
		case GAINOVR_TEST:
			LoadParameter_ComputedTorque();
			/*
			RBgainOverrideHR(Joint[RSP].CAN_channel, Joint[RSP].JMC, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test2);
			RBgainOverrideHR(Joint[RSY].CAN_channel, Joint[RSY].JMC, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test2);
			RBgainOverrideHR(Joint[RWY].CAN_channel, Joint[RWY].JMC, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test2);
			RBgainOverrideHR(Joint[RWY2].CAN_channel, Joint[RWY2].JMC, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test2);
			RBgainOverrideHR(Joint[LSP].CAN_channel, Joint[LSP].JMC, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test2);
			RBgainOverrideHR(Joint[LSY].CAN_channel, Joint[LSY].JMC, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test2);
			RBgainOverrideHR(Joint[LWY].CAN_channel, Joint[LWY].JMC, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test2);
			RBgainOverrideHR(Joint[LWY2].CAN_channel, Joint[LWY2].JMC, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test2);
			
			RBsetMaxDuty(Joint[RSP].CAN_channel, Joint[RSP].JMC, 40, 40, 100);
			RBsetMaxDuty(Joint[RSY].CAN_channel, Joint[RSY].JMC, 40, 40, 100);
			RBsetMaxDuty(Joint[RWY].CAN_channel, Joint[RWY].JMC, 40, 40, 100);			
			RBsetMaxDuty(Joint[LSP].CAN_channel, Joint[LSP].JMC, 40, 40, 100);
			RBsetMaxDuty(Joint[LSY].CAN_channel, Joint[LSY].JMC, 40, 40, 100);

			RBsetMaxDuty(Joint[LWY].CAN_channel, Joint[LWY].JMC, 40, 40, 100);
			RBsetFrictionParameter(Joint[RSP].CAN_channel, Joint[RSP].JMC, (unsigned int)(_friction_para[RSP_33][0]*Joint[RSP].PPR/1000.f), (unsigned int)(_friction_para[RSP_33][1]),	(unsigned int)(_friction_para[RSR_33][0]*Joint[RSR].PPR/1000.f), (unsigned int)(_friction_para[RSR_33][1]));
			RBsetFrictionParameter(Joint[RSY].CAN_channel, Joint[RSY].JMC, (unsigned int)(_friction_para[RSY_33][0]*Joint[RSY].PPR/1000.f), (unsigned int)(_friction_para[RSY_33][1]),	(unsigned int)(_friction_para[REB_33][0]*Joint[REB].PPR/1000.f), (unsigned int)(_friction_para[REB_33][1]));
			RBsetFrictionParameter(Joint[RWY].CAN_channel, Joint[RWY].JMC, (unsigned int)(_friction_para[RWY_33][0]*Joint[RWY].PPR/1000.f), (unsigned int)(_friction_para[RWY_33][1]),	(unsigned int)(_friction_para[RWP_33][0]*Joint[RWP].PPR/1000.f), (unsigned int)(_friction_para[RWP_33][1]));
			RBsetFrictionParameter(Joint[RWY2].CAN_channel, Joint[RWY2].JMC, (unsigned int)(_friction_para[RWY2_33][0]*Joint[RWY2].PPR/1000.f), (unsigned int)(_friction_para[RWY2_33][1]),	0, 0);

			RBsetFrictionParameter(Joint[LSP].CAN_channel, Joint[LSP].JMC, (unsigned int)(_friction_para[LSP_33][0]*Joint[LSP].PPR/1000.f), (unsigned int)(_friction_para[LSP_33][1]),	(unsigned int)(_friction_para[LSR_33][0]*Joint[LSR].PPR/1000.f), (unsigned int)(_friction_para[LSR_33][1]));
			RBsetFrictionParameter(Joint[LSY].CAN_channel, Joint[LSY].JMC, (unsigned int)(_friction_para[LSY_33][0]*Joint[LSY].PPR/1000.f), (unsigned int)(_friction_para[LSY_33][1]),	(unsigned int)(_friction_para[LEB_33][0]*Joint[LEB].PPR/1000.f), (unsigned int)(_friction_para[LEB_33][1]));
			RBsetFrictionParameter(Joint[LWY].CAN_channel, Joint[LWY].JMC, (unsigned int)(_friction_para[LWY_33][0]*Joint[LWY].PPR/1000.f), (unsigned int)(_friction_para[LWY_33][1]),	(unsigned int)(_friction_para[LWP_33][0]*Joint[LWP].PPR/1000.f), (unsigned int)(_friction_para[LWP_33][1]));
			RBsetFrictionParameter(Joint[LWY2].CAN_channel, Joint[LWY2].JMC, (unsigned int)(_friction_para[LWY2_33][0]*Joint[LWY2].PPR/1000.f), (unsigned int)(_friction_para[LWY2_33][1]),	0, 0);
			
			RBenableFrictionCompensation(Joint[RSP].CAN_channel, Joint[RSP].JMC, 1, 1);
			RBenableFrictionCompensation(Joint[RSY].CAN_channel, Joint[RSY].JMC, 1, 1);
			RBenableFrictionCompensation(Joint[RWY].CAN_channel, Joint[RWY].JMC, 1, 1);
			RBenableFrictionCompensation(Joint[RWY2].CAN_channel, Joint[RWY2].JMC, 1, 0);

			RBenableFrictionCompensation(Joint[LSP].CAN_channel, Joint[LSP].JMC, 1, 1);
			RBenableFrictionCompensation(Joint[LSY].CAN_channel, Joint[LSY].JMC, 1, 1);
			RBenableFrictionCompensation(Joint[LWY].CAN_channel, Joint[LWY].JMC, 1, 1);
			RBenableFrictionCompensation(Joint[LWY2].CAN_channel, Joint[LWY2].JMC, 1, 0);
			*/

			RBgainOverrideHR(Joint[RAP].CAN_channel, Joint[RAP].JMC, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test2);	
			RBgainOverrideHR(Joint[LAP].CAN_channel, Joint[LAP].JMC, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test, (unsigned int)pSharedMemory->gain_ovr_test2);

			RBsetMaxDuty(Joint[RAP].CAN_channel, Joint[RAP].JMC, 40, 40, 100);
			RBsetMaxDuty(Joint[LAP].CAN_channel, Joint[LAP].JMC, 40, 40, 100);
			
			RBsetFrictionParameter(Joint[RAP].CAN_channel, Joint[RAP].JMC, (unsigned int)(_friction_para[RAP_33][0]*Joint[RAP].PPR/1000.f), (unsigned int)(_friction_para[RAP_33][1]),	(unsigned int)(_friction_para[RAR_33][0]*Joint[RAR].PPR/1000.f), (unsigned int)(_friction_para[RAR_33][1]));
			RBsetFrictionParameter(Joint[LAP].CAN_channel, Joint[LAP].JMC, (unsigned int)(_friction_para[LAP_33][0]*Joint[LAP].PPR/1000.f), (unsigned int)(_friction_para[LAP_33][1]),	(unsigned int)(_friction_para[LAR_33][0]*Joint[LAR].PPR/1000.f), (unsigned int)(_friction_para[LAR_33][1]));
			
			RBenableFrictionCompensation(Joint[RAP].CAN_channel, Joint[RAP].JMC, 1, 1);
			RBenableFrictionCompensation(Joint[LAP].CAN_channel, Joint[LAP].JMC, 1, 1);
			pSharedMemory->CommandFlag = NO_ACT;			
			break;
		}
	}
}
/******************************************************************************/



void RTFCNDCL TimerHandlerCAN(PVOID context)
{
	CanReceiveMsg(0);	// CAN 1st channel
	CanReceiveMsg(1);	// CAN 2nd channel
	
	if(CANRingHandler() != ERR_OK)
		pSharedMemory->CommandFlag = EXIT_PROGRAM;
}

/******************************************************************************/
// timer interrupt service routine for lower body control
void RTFCNDCL TimerHandler(PVOID context)
{	
	unsigned char i;
	unsigned char supportPhase, landingState;

	if(ReadSensorFlag == 0x01)
	{
		ReadFT(CAN0);		// readout FT sensor value
		ReadFT(CAN1);
		ReadIMU();			// readout IMU sensor value
		ReadEncoder(CAN0);
		ReadEncoder(CAN1);
		
		supportPhase = GetZMP(FTSensor[RFFT], FTSensor[LFFT], WalkingInfo, ZMP, FilteredZMP);
		GetZMP_WB(FTSensor[RFFT], FTSensor[LFFT], FTSensor[RWFT], FTSensor[LWFT]);	// by Inhyeok
		landingState = LandingStateCheck(FTSensor, WalkingInfo);

		// only for displaying
		for(i=RHY ; i<NO_OF_JOINT; i++)
			pSharedMemory->Joint_debug[i] = Joint[i];
	}

	//---------------------------- by Inhyeok
	_FKineUpdatedFlag = 0;
	if(_PassiveUpdatedFlag==1)
		_FKineUpdatedFlag = FKine_Whole();
	//-----------------------------

	switch(pSharedMemory->MotorControlMode)
	{
	case CTRLMODE_POSITION_CONTROL_WIN:
		// move each joint 
		// this mode is used normally in the home position setting
		MoveJointAngle(CAN0);
		break;
	case CTRLMODE_ZMP_INITIALIZATION:
		if(ZMPInitFlag == 0x00)
		{
			// initial posture setting
			ZMPInitialization(InitZMP, ZMP, IMUSensor[CENTERIMU], FTSensor[RFFT], FTSensor[LFFT], WalkingInfo, Joint, 1);
			
			// ZMP control at DSP
			//if(supportPhase != NO_LANDING) DSPControl(ZMP, InitZMP, WalkingInfo, -1.0f, 0x01);
		}
		else MoveZMPInitFF(Joint, WalkingInfo);
		
		// move
		MoveTaskPos(&WalkingInfoSway, WalkingInfo, Joint, landingState, CTRLMODE_ZMP_INITIALIZATION);
		break;
	case CTRLMODE_WALKING:
		// walking profile
		if(pSharedMemory->WalkingDemo ==0x00)
		TestProfileSequence();
		else		
		PredefinedProfileSequence();

		// ZMP control at DSP (not walking)
		if( (WalkingStartStop==0x00) && (WalkingPhaseNext[Z]==WALK_READY) && (supportPhase!=NO_LANDING) ) 
		{
			DampingControl(IMUSensor, Joint, DSP);
			DSPControl(ZMP, InitZMP, WalkingInfo, 2000.0f, 0x01, 0x00);
			if(pSharedMemory->JW_temp[32]==1)
			{
				UnevenControl();
			}
		}
		else if((WalkingStartStop == 0x02)||(WalkingStartStop == 0x01)) // during walking
		{
			// damping control
			if(supportPhase != NO_LANDING) DampingControl(IMUSensor, Joint, supportPhase);
			//DampingControl(IMUSensor, Joint, supportPhase);
			
			// foot vibration control
			VibrationControl(FTSensor, Joint, supportPhase);

			// Set control duration for DSPControl. 
			if(supportPhase == DSP)
			{//
				InitZMP[1] = WalkingInfoSway.RefPatternCurrent;
				DSPControl(ZMP,InitZMP, WalkingInfo, 2000.0f, 0x01, 0x00);
			}
			else 
			{
				//InitZMP[0]=0.0f;
				DSPControl(ZMP, InitZMP, WalkingInfo, 0.0f, 0x03, 0x00);
			}
			
			if(pSharedMemory->JW_temp[32]==1)
			{
				UnevenControl();
			}
			
			//InitZMP[0]=0.0f;
			DSPControl(ZMP, InitZMP, WalkingInfo, 2000.0f, 0x02, 0x00);
				/*
			if(WalkingStartStop == 0x01)
			{
				InitZMP[0]=0.0f;
				DSPControl(ZMP, InitZMP, WalkingInfo, 2000.0f, 0x02, 0x00);
			}
			else
			{
				InitZMP[0]=0.0f;
				DSPControl(ZMP, InitZMP, WalkingInfo, 0.0f, 0x02, 0x00);
			}*/
			
		}
		else;

		// move
		MoveTaskPos(&WalkingInfoSway, WalkingInfo, Joint, landingState, CTRLMODE_WALKING);

		break;
	case CTRLMODE_DEMO:
		break;
	case CTRLMODE_JW_MOCAP:	// by Inhyeok
		break;
	case CTRLMODE_HANDSHAKE:		
		break;	
	case CTRLMODE_WB_OFFLINE_DEMO: // by Inhyeok
		WholeBodyOffline_Demo(CAN0);
		break;
	}
	

	if(ReadSensorFlag == 0x01) 
	{	
		RequestSensor(CAN0);			
		RequestSensor(CAN1);			
	}
	
	PushLogData(); // by Inhyeok
	
	timeIndex++;
}
/******************************************************************************/





/******************************************************************************/
// timer interrupt service routine for upper body control
void RTFCNDCL TimerHandler1(PVOID context)
{	
	unsigned int i;
	static unsigned int scan_count;
	// check CAN message from ch. 1
	//CanReceiveMsg(CAN1);	// CAN 2nd channel


	if(ReadSensorFlag == 0x01)
	{
		//ReadFT(CAN1);		// readout FT sensor value
		ReadEncoder(CAN1);
	}


	if(pSharedMemory->scan_flag == 1)
	{
		NeckDynamixelSetSpeed(0,0,10);
		Joint[NK2].RefAngleCurrent = 0;
		MoveJMC(EJMC2);
		scan_count = 0;
		pSharedMemory->scan_flag = 2;
	}
	else if(pSharedMemory->scan_flag == 2)
	{
		scan_count++;
		if(scan_count > 200)
		{
			NeckDynamixelSetSpeed(0,0,100);
			pSharedMemory->scan_flag = 3;
			scan_count = 0;
		}
	}
	else if(pSharedMemory->scan_flag == 3)
	{
		Joint[NK2].RefAngleCurrent = pSharedMemory->scan_range*(float)sinf(2.f*PI*pSharedMemory->scan_freq*0.01f*(float)scan_count);
		MoveJMC(EJMC2);
		scan_count++;
		if(pSharedMemory->scan_freq*0.01f*(float)scan_count > 1.f)
			scan_count = 0;
	}
	else if(pSharedMemory->scan_flag == 4)
	{
		NeckDynamixelSetSpeed(0,0,10);
		Joint[NK2].RefAngleCurrent = 0;
		MoveJMC(EJMC2);
		pSharedMemory->scan_flag = 0;
	}



	switch(pSharedMemory->MotorControlMode)
	{
	case CTRLMODE_POSITION_CONTROL_WIN:
		MoveJointAngle(CAN1);
		break;
	case CTRLMODE_ZMP_INITIALIZATION:
		// move each joint 
		// this mode is used normally in the home position setting
		MoveJointAngle(CAN1);
		break;
	case CTRLMODE_DEMO:
		MoveJointAngle(CAN1);
		break;
	case CTRLMODE_WALKING:
		MoveJointAngle(CAN1);
		// walking profile
		//TestProfileSequence();
		//for(i=JMC8 ; i<=JMC11 ; i++) MoveJMC(i);
		//for(i=EJMC0 ; i<=EJMC2 ; i++) MoveJMC(i);
		//for(i=EJMC4 ; i<=EJMC5 ; i++) MoveJMC(i);
		break;
		
	case CTRLMODE_JW_MOCAP:					// by Inhyeok
		break;
	case CTRLMODE_HANDSHAKE:
		break;
	case CTRLMODE_PUSHUP:
		break;
	case CTRLMODE_RBT:
		break;
	case CTRLMODE_CCTM_CDI:	// by CDIs
		break;
	case CTRLMODE_WB_OFFLINE_DEMO: // by Inhyeok
		WholeBodyOffline_Demo(CAN1);
		break;

	}
/*
	if(ReadSensorFlag == 0x01) 
	{
		RequestSensor(CAN1);				
	}
*/	
	timeIndex1++;
}
/******************************************************************************/


/************************APIs in Core CDI PROG*********************************/
bool Hubo2JointCTMPWMCommand2ch(JOINT _joint0, JOINT _joint1)
{
	RBJointCTMPWMCommand2ch(_joint0.CAN_channel, _joint0.JMC, _joint0.PWMOUT, _joint1.PWMOUT, _joint0.PWMOUTFlag);

	//RBJointPWMCommand2ch(unsigned int _canch, unsigned int _bno, int _duty1, int _duty2, unsigned int _zeroduty)
	// _zeroduty = 0x00 : Enforce zero duty to stop motor. Back EMF will break the motor
	// _zeroduty = 0x01 : Pulse out to run motor in specified PWM duty and direction

	return true;
}

bool Hubo2JointPWMSignChange(JOINT _joint)
{
	switch(_joint.JointID)
	{
		case RSP:
			Joint[_joint.JointID].PWMOUT = -Joint[_joint.JointID].PWMOUT;
			break;
		case RSR:
			Joint[_joint.JointID].PWMOUT = -Joint[_joint.JointID].PWMOUT;
			break;
		case RSY:
			Joint[_joint.JointID].PWMOUT = -Joint[_joint.JointID].PWMOUT;
			break;
		case REB:
			Joint[_joint.JointID].PWMOUT = -Joint[_joint.JointID].PWMOUT;
			break;
		case RWY:
			Joint[_joint.JointID].PWMOUT = -Joint[_joint.JointID].PWMOUT;
			break;
		case RWP:
			Joint[_joint.JointID].PWMOUT = -Joint[_joint.JointID].PWMOUT;
			break;
	}

	return true;
}

bool Hubo2RightArmCCTM(PCTM _ctm)
{
/*	int i=0;

	float Go[6];
	float J11,J12,J13,J14;
	float J21,J22,J23,J24;
	float J31,J32,J33,J34;
	float Ja[3][4], Ja_T[4][3];
	float X_act[3], Xd_act[3];
	float X_des[3], Xd_des[3], Xdd_des[3], Xdd_cp[3];
	float M_bar[3];
	float Kv[3], Kp[3];
	float Kjv[6], Kjp[6];
	float VtoT[6];
	float th_des[6];
	float FT_XYZ[3];
	float Kff[3];

	float g = 9.8f;

	float a1 = 0.022f;
	float l1 = 0.182f;
	float l2 = 0.070f;
	float l3 = 0.094f;
	float l4 = 0.028f;
	
	float lc1 = l1*0.5f;
	float lc2 = l2*0.5f;
	float lc3 = l3*0.5f;
	float lc4 = l4*0.67f;
	
	float m1 = 0.823f;
	float m2 = 0.070f;
	float m3 = 0.704f;
	float m4 = 0.394f;
	
	float VtoT1 = 0.4880f/0.9f;
	float VtoT2 = 0.3355f/1.4f;
	float VtoT3 = 0.1431f/1.4f;
	float VtoT4 = 0.1717f/3.1f;
	float VtoT5 = 0.0202f/30.0f;
	float VtoT6 = 0.0202f/30.0f;

	VtoT[0] = VtoT1;
	VtoT[1] = VtoT2;
	VtoT[2] = VtoT3;
	VtoT[3] = VtoT4;
	VtoT[4] = VtoT5;
	VtoT[5] = VtoT6;

	// Identity Inertia Matrix in TDC
	float alpha_M = 0.5;

	// Feedback Gain Cartesian Control
	Kp[0] = 800;	// X
	Kp[1] = 400;	// Y
	Kp[2] = 400;	// Z

	Kv[0] = 50;
	Kv[1] = 50;
	Kv[2] = 20;

	// Feedback Gain End-effector Orientation Control, PD
	Kjp[0] = 100;	// RSP
	Kjp[1] = 100;	// RSR
	Kjp[2] = 100;	// RSY
	Kjp[3] = 100;	// REB
	Kjp[4] = 700;	// RWY
	Kjp[5] = 2000;	// RWP
	
	Kjv[0] = 15;
	Kjv[1] = 30;
	Kjv[2] = 10;
	Kjv[3] = 10;
	Kjv[4] = 50;
	Kjv[5] = 100;

	// FT Sensor Force Feedforward Gain
	Kff[0] = 0.f;	// X
	Kff[1] = 0.f;	// Y
	Kff[2] = 100.0f;	// Z
	
	// set joint angle variables
	float th1 = _ctm->th[0];
	float th2 = _ctm->th[1];
	float th3 = _ctm->th[2];
	float th4 = _ctm->th[3]; 
	float th5 = _ctm->th[4]; 
	float th6 = _ctm->th[5];

	float thd1 = _ctm->thd[0];
	float thd2 = _ctm->thd[1];
	float thd3 = _ctm->thd[2];
	float thd4 = _ctm->thd[3]; 
	float thd5 = _ctm->thd[4]; 
	float thd6 = _ctm->thd[5];

	// Desired Joint Angle 
	th_des[0] = -0.697f;	// RSP
	th_des[1] = -0.262f;	// RSR
	th_des[2] = -0.087f;	// RSY
	th_des[3] = -(0.783f + 0.1f);	// REB
	//th_des[4] = 0.f;	// RWY
	//th_des[4] = (_ctm->th[1]) + 0.1f;	// RWY
	th_des[4] = (_ctm->th[1]) + 0.3f;	// RWY
	th_des[5] = -0.108f;	// RWP
	//th_des[5] = -(_ctm->th[0] + _ctm->th[3] + 90.f*DEG2RAD - 0.1f);	// RWP

	// Old Force saving
	for(i=0; i<3; i++)
	{
		_ctm->Fu_old[i] = _ctm->Fu[i];
	}

	// Gravity Compensation Term
	Go[0] = g*(lc1*m1*sin(th1)*cos(th2)+m3*(l1*cos(th2)*cos(th3)*(sin(th1)*cos(
	th3)-sin(th2)*sin(th3)*cos(th1))+l2*cos(th2)*cos(th3)*(sin(th4)*cos(th1)*
	cos(th2)+cos(th4)*(sin(th1)*cos(th3)-sin(th2)*sin(th3)*cos(th1)))+a1*cos(
	th2)*cos(th3)*(cos(th1)*cos(th2)*cos(th4)-sin(th4)*(sin(th1)*cos(th3)-sin(
	th2)*sin(th3)*cos(th1)))+(sin(th1)*sin(th3)+sin(th2)*cos(th1)*cos(th3))*(l2*(
	sin(th2)*sin(th4)+sin(th3)*cos(th2)*cos(th4))+a1*(sin(th2)*cos(th4)-sin(th3)*
	sin(th4)*cos(th2)))+lc3*(sin(th5)*cos(th2)*cos(th3)+cos(th5)*(sin(th2)*sin(
	th4)+sin(th3)*cos(th2)*cos(th4)))*(cos(th5)*(sin(th1)*sin(th3)+sin(th2)*cos(
	th1)*cos(th3))+sin(th5)*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*cos(
	th3)-sin(th2)*sin(th3)*cos(th1))))-a1*cos(th1)*cos(th3)*pow(cos(th2),2)-(a1*
	sin(th2)-l1*sin(th3)*cos(th2))*(sin(th1)*sin(th3)+sin(th2)*cos(th1)*cos(th3))-
	lc3*(cos(th2)*cos(th3)*cos(th5)-sin(th5)*(sin(th2)*sin(th4)+sin(th3)*cos(
	th2)*cos(th4)))*(sin(th5)*(sin(th1)*sin(th3)+sin(th2)*cos(th1)*cos(th3))-
	cos(th5)*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*cos(th3)-sin(th2)*
	sin(th3)*cos(th1)))))-m2*(a1*cos(th1)*cos(th3)*pow(cos(th2),2)+(a1*sin(th2)-
	l1*sin(th3)*cos(th2))*(sin(th1)*sin(th3)+sin(th2)*cos(th1)*cos(th3))-l1*cos(
	th2)*cos(th3)*(sin(th1)*cos(th3)-sin(th2)*sin(th3)*cos(th1))-lc2*cos(th2)*
	cos(th3)*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*cos(th3)-sin(th2)*
	sin(th3)*cos(th1)))-a1*cos(th2)*cos(th3)*(cos(th1)*cos(th2)*cos(th4)-sin(
	th4)*(sin(th1)*cos(th3)-sin(th2)*sin(th3)*cos(th1)))-(sin(th1)*sin(th3)+sin(
	th2)*cos(th1)*cos(th3))*(lc2*(sin(th2)*sin(th4)+sin(th3)*cos(th2)*cos(th4))+
	a1*(sin(th2)*cos(th4)-sin(th3)*sin(th4)*cos(th2))))-m4*(a1*cos(th1)*cos(th3)*
	pow(cos(th2),2)+(a1*sin(th2)-l1*sin(th3)*cos(th2))*(sin(th1)*sin(th3)+sin(
	th2)*cos(th1)*cos(th3))+l3*(cos(th2)*cos(th3)*cos(th5)-sin(th5)*(sin(th2)*
	sin(th4)+sin(th3)*cos(th2)*cos(th4)))*(sin(th5)*(sin(th1)*sin(th3)+sin(th2)*
	cos(th1)*cos(th3))-cos(th5)*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*
	cos(th3)-sin(th2)*sin(th3)*cos(th1))))-l1*cos(th2)*cos(th3)*(sin(th1)*cos(
	th3)-sin(th2)*sin(th3)*cos(th1))-l2*cos(th2)*cos(th3)*(sin(th4)*cos(th1)*
	cos(th2)+cos(th4)*(sin(th1)*cos(th3)-sin(th2)*sin(th3)*cos(th1)))-a1*cos(
	th2)*cos(th3)*(cos(th1)*cos(th2)*cos(th4)-sin(th4)*(sin(th1)*cos(th3)-sin(
	th2)*sin(th3)*cos(th1)))-(sin(th1)*sin(th3)+sin(th2)*cos(th1)*cos(th3))*(l2*(
	sin(th2)*sin(th4)+sin(th3)*cos(th2)*cos(th4))+a1*(sin(th2)*cos(th4)-sin(th3)*
	sin(th4)*cos(th2)))-l3*(sin(th5)*cos(th2)*cos(th3)+cos(th5)*(sin(th2)*sin(
	th4)+sin(th3)*cos(th2)*cos(th4)))*(cos(th5)*(sin(th1)*sin(th3)+sin(th2)*cos(
	th1)*cos(th3))+sin(th5)*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*cos(
	th3)-sin(th2)*sin(th3)*cos(th1))))-lc4*(sin(th6)*(sin(th2)*cos(th4)-sin(th3)*
	sin(th4)*cos(th2))+cos(th6)*(sin(th5)*cos(th2)*cos(th3)+cos(th5)*(sin(th2)*
	sin(th4)+sin(th3)*cos(th2)*cos(th4))))*(cos(th5)*(sin(th1)*sin(th3)+sin(th2)*
	cos(th1)*cos(th3))+sin(th5)*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*
	cos(th3)-sin(th2)*sin(th3)*cos(th1))))-lc4*(cos(th2)*cos(th3)*cos(th5)-sin(
	th5)*(sin(th2)*sin(th4)+sin(th3)*cos(th2)*cos(th4)))*(sin(th6)*(cos(th1)*
	cos(th2)*cos(th4)-sin(th4)*(sin(th1)*cos(th3)-sin(th2)*sin(th3)*cos(th1)))-
	cos(th6)*(sin(th5)*(sin(th1)*sin(th3)+sin(th2)*cos(th1)*cos(th3))-cos(th5)*(
	sin(th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*cos(th3)-sin(th2)*sin(th3)*
	cos(th1)))))))/VtoT1;

	Go[1] = -g*(m2*(cos(th3)*(a1*sin(th4)-lc2*cos(th4))*(sin(th1)*sin(th3)+sin(
	th2)*cos(th1)*cos(th3))-l1*sin(th2)*cos(th1)-sin(th3)*(a1*cos(th1)*cos(th2)-
	lc2*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*cos(th3)-sin(th2)*sin(
	th3)*cos(th1)))-a1*(cos(th1)*cos(th2)*cos(th4)-sin(th4)*(sin(th1)*cos(th3)-
	sin(th2)*sin(th3)*cos(th1)))))+m3*(cos(th3)*(a1*sin(th4)-l2*cos(th4))*(sin(
	th1)*sin(th3)+sin(th2)*cos(th1)*cos(th3))+lc3*(sin(th3)*sin(th5)-cos(th3)*
	cos(th4)*cos(th5))*(cos(th5)*(sin(th1)*sin(th3)+sin(th2)*cos(th1)*cos(th3))+
	sin(th5)*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*cos(th3)-sin(th2)*
	sin(th3)*cos(th1))))-l1*sin(th2)*cos(th1)-lc3*(sin(th3)*cos(th5)+sin(th5)*
	cos(th3)*cos(th4))*(sin(th5)*(sin(th1)*sin(th3)+sin(th2)*cos(th1)*cos(th3))-
	cos(th5)*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*cos(th3)-sin(th2)*
	sin(th3)*cos(th1))))-sin(th3)*(a1*cos(th1)*cos(th2)-l2*(sin(th4)*cos(th1)*
	cos(th2)+cos(th4)*(sin(th1)*cos(th3)-sin(th2)*sin(th3)*cos(th1)))-a1*(cos(
	th1)*cos(th2)*cos(th4)-sin(th4)*(sin(th1)*cos(th3)-sin(th2)*sin(th3)*cos(
	th1)))))-lc1*m1*sin(th2)*cos(th1)-m4*(l1*sin(th2)*cos(th1)+l3*(sin(th3)*cos(
	th5)+sin(th5)*cos(th3)*cos(th4))*(sin(th5)*(sin(th1)*sin(th3)+sin(th2)*cos(
	th1)*cos(th3))-cos(th5)*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*cos(
	th3)-sin(th2)*sin(th3)*cos(th1))))+sin(th3)*(a1*cos(th1)*cos(th2)-l2*(sin(
	th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*cos(th3)-sin(th2)*sin(th3)*cos(
	th1)))-a1*(cos(th1)*cos(th2)*cos(th4)-sin(th4)*(sin(th1)*cos(th3)-sin(th2)*
	sin(th3)*cos(th1))))-cos(th3)*(a1*sin(th4)-l2*cos(th4))*(sin(th1)*sin(th3)+
	sin(th2)*cos(th1)*cos(th3))-l3*(sin(th3)*sin(th5)-cos(th3)*cos(th4)*cos(th5))*(
	cos(th5)*(sin(th1)*sin(th3)+sin(th2)*cos(th1)*cos(th3))+sin(th5)*(sin(th4)*
	cos(th1)*cos(th2)+cos(th4)*(sin(th1)*cos(th3)-sin(th2)*sin(th3)*cos(th1))))-
	lc4*(sin(th4)*sin(th6)*cos(th3)+cos(th6)*(sin(th3)*sin(th5)-cos(th3)*cos(
	th4)*cos(th5)))*(cos(th5)*(sin(th1)*sin(th3)+sin(th2)*cos(th1)*cos(th3))+
	sin(th5)*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*cos(th3)-sin(th2)*
	sin(th3)*cos(th1))))-lc4*(sin(th3)*cos(th5)+sin(th5)*cos(th3)*cos(th4))*(
	sin(th6)*(cos(th1)*cos(th2)*cos(th4)-sin(th4)*(sin(th1)*cos(th3)-sin(th2)*
	sin(th3)*cos(th1)))-cos(th6)*(sin(th5)*(sin(th1)*sin(th3)+sin(th2)*cos(th1)*
	cos(th3))-cos(th5)*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*cos(th3)-
	sin(th2)*sin(th3)*cos(th1)))))))/VtoT2;

	Go[2] = g*(m2*(a1-a1*cos(th4)-lc2*sin(th4))*(sin(th1)*sin(th3)+sin(th2)*cos(
	th1)*cos(th3))+m3*(sin(th1)*sin(th3)+sin(th2)*cos(th1)*cos(th3))*(a1-a1*cos(
	th4)-l2*sin(th4)-lc3*sin(th4))-m4*(l3*sin(th4)*(sin(th1)*sin(th3)+sin(th2)*
	cos(th1)*cos(th3))+(a1*cos(th4)+l2*sin(th4))*(sin(th1)*sin(th3)+sin(th2)*
	cos(th1)*cos(th3))+lc4*(sin(th6)*cos(th4)+sin(th4)*cos(th5)*cos(th6))*(cos(
	th5)*(sin(th1)*sin(th3)+sin(th2)*cos(th1)*cos(th3))+sin(th5)*(sin(th4)*cos(
	th1)*cos(th2)+cos(th4)*(sin(th1)*cos(th3)-sin(th2)*sin(th3)*cos(th1))))-a1*(
	sin(th1)*sin(th3)+sin(th2)*cos(th1)*cos(th3))-lc4*sin(th4)*sin(th5)*(sin(
	th6)*(cos(th1)*cos(th2)*cos(th4)-sin(th4)*(sin(th1)*cos(th3)-sin(th2)*sin(
	th3)*cos(th1)))-cos(th6)*(sin(th5)*(sin(th1)*sin(th3)+sin(th2)*cos(th1)*cos(
	th3))-cos(th5)*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*cos(th3)-sin(
	th2)*sin(th3)*cos(th1)))))))/VtoT3;

	Go[3] = g*(lc3*m3*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*cos(th3)-
	sin(th2)*sin(th3)*cos(th1)))+m4*(l3*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(
	sin(th1)*cos(th3)-sin(th2)*sin(th3)*cos(th1)))+lc4*cos(th6)*(sin(th4)*cos(
	th1)*cos(th2)+cos(th4)*(sin(th1)*cos(th3)-sin(th2)*sin(th3)*cos(th1)))+lc4*
	sin(th6)*cos(th5)*(cos(th1)*cos(th2)*cos(th4)-sin(th4)*(sin(th1)*cos(th3)-
	sin(th2)*sin(th3)*cos(th1)))))/VtoT4;

	Go[4] = -g*lc4*m4*sin(th6)*(cos(th5)*(sin(th1)*sin(th3)+sin(th2)*cos(th1)*
	cos(th3))+sin(th5)*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(sin(th1)*cos(th3)-
	sin(th2)*sin(th3)*cos(th1))))/VtoT5;

	Go[5] = g*lc4*m4*(sin(th6)*(cos(th1)*cos(th2)*cos(th4)-sin(th4)*(sin(th1)*
	cos(th3)-sin(th2)*sin(th3)*cos(th1)))-cos(th6)*(sin(th5)*(sin(th1)*sin(th3)+
	sin(th2)*cos(th1)*cos(th3))-cos(th5)*(sin(th4)*cos(th1)*cos(th2)+cos(th4)*(
	sin(th1)*cos(th3)-sin(th2)*sin(th3)*cos(th1)))))/VtoT6;

	for(i=0; i<6; i++)
	{
		_ctm->Gcp[i] = Go[i];
	}	

	// Jacobian - 3DOF CTM(RSP, RSR, REB)
	J11 = a1*(sin(th1)*cos(th4)+sin(th4)*cos(th1)*cos(th2)) + l2*(sin(th1)*sin(
	th4)-cos(th1)*cos(th2)*cos(th4)) - a1*sin(th1) - l1*cos(th1)*cos(th2);
	J12 = -sin(th1)*sin(th2)*(a1*sin(th4)-l1-l2*cos(th4));
	J14 = a1*(sin(th4)*cos(th1)+sin(th1)*cos(th2)*cos(th4)) - l2*(cos(th1)*cos(
	th4)-sin(th1)*sin(th4)*cos(th2));
	J22 = -cos(th2)*(a1*sin(th4)-l1-l2*cos(th4));
	J24 = -sin(th2)*(a1*cos(th4)+l2*sin(th4));
	J31 = l1*sin(th1)*cos(th2) + l2*(sin(th4)*cos(th1)+sin(th1)*cos(th2)*cos(
	th4)) + a1*(cos(th1)*cos(th4)-sin(th1)*sin(th4)*cos(th2)) - a1*cos(th1);
	J32 = -sin(th2)*cos(th1)*(a1*sin(th4)-l1-l2*cos(th4));
	J34 = l2*(sin(th1)*cos(th4)+sin(th4)*cos(th1)*cos(th2)) - a1*(sin(th1)*sin(
	th4)-cos(th1)*cos(th2)*cos(th4));

	Ja[0][0] = J11;
	Ja[0][1] = J12;
	Ja[0][2] = 0;
	Ja[0][3] = J14;
	Ja[1][0] = 0;
	Ja[1][1] = J22;
	Ja[1][2] = 0;
	Ja[1][3] = J24;
	Ja[2][0] = J31;
	Ja[2][1] = J32;
	Ja[2][2] = 0;
	Ja[2][3] = J34;
	Ja_T[0][0] = J11;
	Ja_T[0][1] = 0;
	Ja_T[0][2] = J31;
	Ja_T[1][0] = J12;
	Ja_T[1][1] = J22;
	Ja_T[1][2] = J32;
	Ja_T[2][0] = 0;
	Ja_T[2][1] = 0;
	Ja_T[2][2] = 0;
	Ja_T[3][0] = J14;
	Ja_T[3][1] = J24;
	Ja_T[3][2] = J34;

	
	// Operational Space Computed Force
	for(i=0; i<3; i++)
	{
		//_ctm->Fu[i] = M_bar[i]*Xdd_cp[i]; //+ _ctm->Fu_old[i];	// CCTM
		_ctm->Fu[i] = 0.f;	// JOINT PD
	}

	// FT Sensor Feedforward
	if(_ctm->Force_FB_Flag==1)
	{
		// FT sensor
		FT_XYZ[0] = -FTSensor[RWFT].Fz;	// X
		FT_XYZ[1] = FTSensor[RWFT].My;	// Y
		FT_XYZ[2] = FTSensor[RWFT].Mx;	// Z
		
		for(i=0; i<3; i++)
		{
			_ctm->RFT_XYZ_LPF[i] = (float)((1.0f - 2.0f*PI*5.0f*INT_TIME1/1000.0f)*_ctm->RFT_XYZ_old[i] + (2.0f*PI*5.0f*INT_TIME1/1000.0f)*FT_XYZ[i]);	// Lowpass Filter
			//_ctm->RFT_XYZ_LPF[i] = (float)((1.0f - 2.0f*PI*0.01f*INT_TIME1/1000.0f)*_ctm->RFT_XYZ_old[i] + (INT_TIME1/1000.0f)*FT_XYZ[i]);	// Leaking Integral
			_ctm->RFT_XYZ_old[i] = _ctm->RFT_XYZ_LPF[i];
		}

		for(i=0; i<3; i++)
		{	
			_ctm->Fu[i] = _ctm->Fu[i] + Kff[i]*(_ctm->RFT_XYZ_LPF[i]);
		}
	}

	// Joint Space Torque
	for(i=0; i<4; i++)
	{
		_ctm->Tau_u[i] = (Ja_T[i][0]*_ctm->Fu[0] + Ja_T[i][1]*_ctm->Fu[1] + Ja_T[i][2]*_ctm->Fu[2] + Ja_T[i][3]*_ctm->Fu[3])/VtoT[i];
	}

	// Passive End-effector Joint Control - RWY, RWP
	_ctm->Tau_u[4] = Kjv[4]*(0.f - _ctm->thd[4]) + Kjp[4]*(th_des[4] - _ctm->th[4]);
	_ctm->Tau_u[5] = Kjv[5]*(0.f - _ctm->thd[5]) + Kjp[5]*(th_des[5] - _ctm->th[5]);

	// Passive End-effector Joint Control - RSY
	_ctm->Tau_u[2] = _ctm->Tau_u[2] + Kjv[2]*(0.f - _ctm->thd[2]) + Kjp[2]*(th_des[2] - _ctm->th[2]);

	// Passive Joint Control - RSP, RSR, REB
	_ctm->Tau_u[0] = _ctm->Tau_u[0] + Kjv[0]*(0.f - _ctm->thd[0]) + Kjp[0]*(th_des[0] - _ctm->th[0]);	// RSP
	_ctm->Tau_u[1] = _ctm->Tau_u[1] + Kjv[1]*(0.f - _ctm->thd[1]) + Kjp[1]*(th_des[1] - _ctm->th[1]);	// RSR
	_ctm->Tau_u[3] = _ctm->Tau_u[3] + Kjv[3]*(0.f - _ctm->thd[3]) + Kjp[3]*(th_des[3] - _ctm->th[3]);	// REB

	// PWM Out value Raw
	for(i=0; i<6; i++)
	{
		_ctm->CTMOUT_raw[i] = _ctm->Gcp[i] + _ctm->Tau_u[i];
	}

	// pwmout Lowpass
	for(i=0; i<6; i++)
	{
		_ctm->CTMOUT_LPF[i] = (float)( (1.0f - 2.0f*PI*5.0f*DELTA_T1)*(_ctm->CTMOUT_old[i]) + (2.0f*PI*5.0f*DELTA_T1)*(_ctm->CTMOUT_raw[i]) );	// Lowpass Filter
	}

	for(i=0; i<6; i++)
	{
		_ctm->CTMOUT_old[i] = (float)( _ctm->CTMOUT_LPF[i] );
	}
*/
	return true;
}
/******************************************************************************/



/******************************************************************************/
void PredefinedProfileSequence(void)
{
	float tempSideStep;
	float tempYaw;

	static float tempSway			= 0.069f;
	float tempSwayLeft		= 1.0f;
	float tempSwayRight		= 1.0f;
	float tempDelayY		= 60.0f;
	float tempDelayYLeft	= 1.0f;
	float tempDelayYRight	= 1.0f;
	float tempStepLeft		= 0.0f;
	float tempStepRight		= 0.0f;
	
	float tempSideStepLeft	= 1.0f;
	float tempSideStepRight = 1.0f;
	float tempDSP			= 0.0f;//20.0f/2.0f;
	float tempPeriod		= 800.0f;
	int   tempCount			= -1;
	float tempStartSway		= 1.1f;
	float tempStopSway		= 1.1f;
	float tempSideDelayY		= 60.0f;
	float tempSideDelayYLeft	= 1.0f;
	float tempSideDelayYRight	= 1.0f;

	float tempUserData[5];
	float tempDelayTime[2];

	
	//static float 
	//	lastStep = tempStep;
	//static float 
	//	lastStep2 = tempStep;
	//static float 
	//	lastStep3 = tempStep;
	static float lastStep5 = tempStep;

	//static unsigned char StepChangeFlag = 0x00;
	static unsigned char StepChangeFlag2 =0x00;

	static unsigned char start = 0x00;
	
	
// by jungho77
	pSharedMemory->JW_temp[6] = forwardWalkingSequence[currentWalkingStep];
	tempSideStep = sideWalkingSequence[currentWalkingStep];
	tempYaw = rotationWalkingSequence[currentWalkingStep];
	tempSway = swaySequence[currentWalkingStep];
	if(StepChangeFlag2 ==0x00) 
	{
		tempStep = pSharedMemory->JW_temp[6];//pSharedMemory->JW_temp[6];//step/2
		//tempStep = pSharedMemory->JW_temp[6];
		
		if(start == 0x00)
		{
			lastStep = tempStep;
			start = 0x01;
		}
	}
// end



	if((StepChangeFlag2 ==0x00)&&(lastStep != tempStep))//&&((lastStep==0.0f)&&(tempStep>0)))
	{
		//printf("1");
		if((tempStep!=0)&&(lastStep==0.0f))
		{
		//	printf("2");
			StepChangeFlag = 0x01;
		}
		else if((lastStep!=0)&&(tempStep==0.0f))
		{
		//	printf("3");
			if((StepChangeFlag ==0x02)||(StepChangeFlag ==0x01))//||(StepChangeFlag ==0x03)||(StepChangeFlag ==0x04))
			{
				StepChangeFlag2 = 0x01;
				tempStep = lastStep;
				lastStep5 = tempStep;
			}			
			else
			StepChangeFlag = 0x03;
		}

		lastStep2 = lastStep;
	}

	if((StepChangeFlag2 ==0x01)&&(StepChangeFlag ==0x00))
	{	
		StepChangeFlag = 0x03;
		StepChangeFlag2 = 0x00;
		tempStep = lastStep5;
	}

	if(lastStep3 != lastStep2) lastStep3 = lastStep2;

	if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
	lastStep4 = lastStep;


	// Stop walking command check
	//if(tempSideStep >= 0.0f) { if( (WalkingStartStop==0x00) && (WalkingPhaseNext[Z]==RIGHT_FOOT_DOWN_SET) ) WalkingPhaseNextSway = WalkingPhaseNext[X] = WalkingPhaseNext[Y] = WalkingPhaseNext[Z] = WalkingPhaseNext[Yaw] = HIP_TO_CENTER; }
	//else { if( (WalkingStartStop==0x00) && (WalkingPhaseNext[Z]==LEFT_FOOT_DOWN_SET) ) WalkingPhaseNextSway = WalkingPhaseNext[X] = WalkingPhaseNext[Y] = WalkingPhaseNext[Z] = WalkingPhaseNext[Yaw] = HIP_TO_CENTER; }

	if( (WalkingStartStop==0x00) && (WalkingPhaseNext[Z]==RIGHT_FOOT_DOWN_SET) ) WalkingPhaseNextSway = WalkingPhaseNext[X] = WalkingPhaseNext[Y] = WalkingPhaseNext[Z] = WalkingPhaseNext[Yaw] = HIP_TO_CENTER;
	if( (WalkingStartStop==0x00) && (WalkingPhaseNext[Z]==LEFT_FOOT_DOWN_SET) ) WalkingPhaseNextSway = WalkingPhaseNext[X] = WalkingPhaseNext[Y] = WalkingPhaseNext[Z] = WalkingPhaseNext[Yaw] = HIP_TO_CENTER;


	// x-direction sequence
	switch(WalkingPhaseNext[X])
	{
		case FIRST_RIGHT_FOOT_UP_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				//printf("\n FRU,%d ",StepChangeFlag);
				tempDelayTime[0] = tempPeriod/2.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][X], tempStep, tempPeriod, tempDelayTime, tempUserData, 1, 0x04);
				SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);
			
				WalkingPhaseNext[X] = FIRST_RIGHT_FOOT_DOWN_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = FIRST_RIGHT_FOOT_UP_FORWARD_SET;
			}
			break;
		case FIRST_LEFT_FOOT_UP_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				//printf("\n FLU,%d ",StepChangeFlag);
				tempDelayTime[0] = tempPeriod/2.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][X], tempStep, tempPeriod, tempDelayTime, tempUserData, 1, 0x04);
				SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[X] = FIRST_LEFT_FOOT_DOWN_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = FIRST_LEFT_FOOT_UP_FORWARD_SET;
			}
			break;
		case FIRST_RIGHT_FOOT_DOWN_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				//printf("\n FRD,%d ",StepChangeFlag);
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = tempDSP; tempUserData[1] = lastStep3;
				SetMoveTaskPos(&WalkingInfo[RIGHT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x06);
				SetMoveTaskPos(&WalkingInfo[LEFT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);

				WalkingPhaseNext[X] = LEFT_FOOT_UP_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = FIRST_RIGHT_FOOT_DOWN_FORWARD_SET;
			}
			break;
		case FIRST_LEFT_FOOT_DOWN_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				///printf("\n FLD,%d ",StepChangeFlag);
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = tempDSP; tempUserData[1] = lastStep3;
				SetMoveTaskPos(&WalkingInfo[LEFT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x06);
				SetMoveTaskPos(&WalkingInfo[RIGHT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
				
				WalkingPhaseNext[X] = RIGHT_FOOT_UP_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = FIRST_LEFT_FOOT_DOWN_FORWARD_SET;
			}
			break;
		case RIGHT_FOOT_UP_FORWARD_SET:
			
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				//printf("\n RU,%d ",StepChangeFlag);
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				if(WalkingStartStop == 0x00)
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x09);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x02);
						
					WalkingPhaseNext[X] = HIP_TO_CENTER;
				}
				else
				{
					if(StepChangeFlag == 0x00)
					{
						tempUserData[0] = tempDSP;
						SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x07);
						tempUserData[0] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x02);
					}
					else if(StepChangeFlag == 0x01)
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x04);
						SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						StepChangeFlag = 0x02;
					}
					else if(StepChangeFlag == 0x03)
					{
						tempUserData[1] = lastStep3;
						SetMoveTaskPos(&WalkingInfo[RIGHT][X], -lastStep2, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x09);
						SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x02);
						StepChangeFlag = 0x04;
					}
					WalkingPhaseNext[X] = RIGHT_FOOT_DOWN_FORWARD_SET;
				}
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = RIGHT_FOOT_UP_FORWARD_SET;
			}
			break;
		case RIGHT_FOOT_DOWN_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				//printf("\n RD,%d ",StepChangeFlag);
				if((StepChangeFlag==0x00))
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = tempDSP;	tempUserData[1] = lastStep3;
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x08);
					
					tempUserData[0] = 0.0f;
					SetMoveTaskPos(&WalkingInfo[LEFT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
				}				
				else if(StepChangeFlag == 0x02)
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = tempDSP; tempUserData[1] = lastStep3;
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x06);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
					StepChangeFlag = 0x00;
				}
				else if((StepChangeFlag==0x01)||(StepChangeFlag==0x03))
				{
					
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = tempDSP;	tempUserData[1] = lastStep3;
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], lastStep2, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x08);
					
					tempUserData[0] = 0.0f;
					SetMoveTaskPos(&WalkingInfo[LEFT][X], -lastStep2, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
				}
				else if(StepChangeFlag==0x04)
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;		tempUserData[1] = lastStep3;
					
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 10);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					StepChangeFlag=0x00;
					//printf("in");
				}
				
				WalkingPhaseNext[X] = LEFT_FOOT_UP_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = RIGHT_FOOT_DOWN_FORWARD_SET;
			}
			break;
		case LEFT_FOOT_UP_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				//printf("\n LU,%d ",StepChangeFlag);
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				if(WalkingStartStop == 0x00)
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x02);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x09);
					
					WalkingPhaseNext[X] = HIP_TO_CENTER;
				}
				else
				{
					if(StepChangeFlag==0x00)
					{
						tempUserData[0] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x02);
						
						tempUserData[0] = tempDSP;
						SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x07);
					}
					else if(StepChangeFlag==0x01)
					{
						tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[LEFT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x04);
						SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						StepChangeFlag=0x02;
					}
					else if(StepChangeFlag==0x03)
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x02);
						tempUserData[1] = lastStep3;
						SetMoveTaskPos(&WalkingInfo[LEFT][X], -lastStep2, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x09);
						StepChangeFlag=0x04;
					}
					
					WalkingPhaseNext[X] = LEFT_FOOT_DOWN_FORWARD_SET;
				}

				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = LEFT_FOOT_UP_FORWARD_SET;
			}
			break;
		case LEFT_FOOT_DOWN_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				//printf("\n LD,%d ",StepChangeFlag);
				if((StepChangeFlag==0x00))
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
					
					tempUserData[0] = tempDSP;
					SetMoveTaskPos(&WalkingInfo[LEFT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x08);
				}
				else if(StepChangeFlag==0x02)
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = tempDSP; tempUserData[1] = lastStep3;
					SetMoveTaskPos(&WalkingInfo[LEFT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x06);
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
					StepChangeFlag=0x00;
				}
				else if((StepChangeFlag==0x01)||(StepChangeFlag==0x03))
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], -lastStep2, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
					
					tempUserData[0] = tempDSP;	tempUserData[1] = lastStep3;
					SetMoveTaskPos(&WalkingInfo[LEFT][X], lastStep2, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x08);

				}
				else if((StepChangeFlag==0x04))
				{

					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;		tempUserData[1] = lastStep3;
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 10);
					StepChangeFlag=0x00;
					//printf("in");
				}
				
				WalkingPhaseNext[X] = RIGHT_FOOT_UP_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = LEFT_FOOT_DOWN_FORWARD_SET;
			}
			break;
		case HIP_TO_CENTER:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				//printf("\n HC,%d ",StepChangeFlag);
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;		tempUserData[1] = lastStep3;
				if(WalkingInfo[RIGHT][X].CurrentSequence == LEFT_FOOT_UP_FORWARD_SET)
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 10);
				}
				else if(WalkingInfo[RIGHT][X].CurrentSequence == RIGHT_FOOT_UP_FORWARD_SET)
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 10);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				}
				WalkingPhaseNext[X] = WALK_READY;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = HIP_TO_CENTER;
			}
			break;
		case WALK_READY:
			if((WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false))
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = WALK_READY;
			lastStep = lastStep2 = lastStep3 = 0.0f;
			StepChangeFlag=0x00;
			break;
	}


	// y-direction sequence
	switch(WalkingPhaseNext[Y])
	{
		case FIRST_RIGHT_FOOT_UP_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.0f+tempSideDelayYRight*tempSideDelayY/2.0f;	tempDelayTime[1] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Y], tempSideStepRight*tempSideStep/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);

				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.f;	tempDelayTime[1] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Y], -tempSideStepLeft*tempSideStep/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);
			
				WalkingPhaseNext[Y] = FIRST_RIGHT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = FIRST_RIGHT_FOOT_UP_SET;
			}
			break;
		case FIRST_RIGHT_FOOT_DOWN_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f+tempSideDelayYRight*tempSideDelayY/2.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Y], tempSideStepRight*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Y], -tempSideStepLeft*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
			
				WalkingPhaseNext[Y] = LEFT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = FIRST_RIGHT_FOOT_DOWN_SET;
			}
			break;
		case FIRST_LEFT_FOOT_UP_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{	
				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.0f+tempSideDelayYLeft*tempSideDelayY/2.0f;	tempDelayTime[1] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Y], tempSideStepLeft*tempSideStep/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);

				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.f;	tempDelayTime[1] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Y], -tempSideStepRight*tempSideStep/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);

				WalkingPhaseNext[Y] = FIRST_LEFT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = FIRST_LEFT_FOOT_UP_SET;
			}
			break;
		case FIRST_LEFT_FOOT_DOWN_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.f+tempSideDelayYLeft*tempSideDelayY/2.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Y], tempSideStepLeft*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Y], -tempSideStepRight*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

				WalkingPhaseNext[Y] = RIGHT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = FIRST_LEFT_FOOT_DOWN_SET;
			}
			break;
		case RIGHT_FOOT_UP_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				if(tempSideStep>=0)
				{
					//tempDelayTime[0] = tempDSP+tempSideDelayYLeft*tempSideDelayY;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
					//SetMoveTaskPos(LEFT, Y, tempSideStepLeft*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);	// if you change "function mode" here, also should change it in "HIP_TO_CENTER"
					
					if(WalkingStartStop == 0x00)	//0x01 : stop click , 0x00 : during stop mode
					{
						tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						
						WalkingPhaseNext[Y] = HIP_TO_CENTER;
					}
					else	// continuous walking
					{
						//SetMoveTaskPos(RIGHT, Y, -tempSideStepRight*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
						tempDelayTime[0] = tempDSP+20.0f+tempSideDelayYRight*tempSideDelayY;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

						WalkingPhaseNext[Y] = RIGHT_FOOT_DOWN_SET;
					}

					//SetMoveTaskPosJointFF(&Joint[LHR], tempHipComp, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				else
				{
					tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
					if(WalkingStartStop == 0x00)	//0x01 : stop click , 0x00 : during stop mode
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						
						WalkingPhaseNext[Y] = HIP_TO_CENTER;
					}
					else
					{
						tempDelayTime[0] = tempDSP+tempSideDelayYRight*tempSideDelayY;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], tempSideStepRight*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);

						tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], -tempSideStepLeft*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);

						WalkingPhaseNext[Y] = RIGHT_FOOT_DOWN_SET;
					}
				}
				
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = RIGHT_FOOT_UP_SET;
			}
			break;
		case RIGHT_FOOT_DOWN_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				if(tempSideStep>=0)
				{
				//	tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYLeft*tempSideDelayY;	// DSP time + margin time
				//	SetMoveTaskPos(LEFT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
				//	tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;	// DSP time + margin time
				//	SetMoveTaskPos(RIGHT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
					SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					//SetMoveTaskPosJointFF(&Joint[LHR], 0.0f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				else
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYRight*tempSideDelayY;	// DSP time + margin time
					SetMoveTaskPos(&WalkingInfo[RIGHT][Y], tempSideStepRight*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;	// DSP time + margin time
					SetMoveTaskPos(&WalkingInfo[LEFT][Y], -tempSideStepLeft*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
				}
				
				WalkingPhaseNext[Y] = LEFT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = RIGHT_FOOT_DOWN_SET;
			}
			break;
		case LEFT_FOOT_UP_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				if(tempSideStep>=0)
				{
					if(WalkingStartStop == 0x00)	//0x01 : stop click , 0x00 : during stop mode
					{
						tempDelayTime[0] = tempDSP+20.f;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						tempDelayTime[0] = tempDSP+20.0f+tempSideDelayYLeft*tempSideDelayY;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

						WalkingPhaseNext[Y] = HIP_TO_CENTER;
					}
					else
					{
						tempDelayTime[0] = tempDSP+tempSideDelayYLeft*tempSideDelayY;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], tempSideStepLeft*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
					
						tempDelayTime[0] = tempDSP+20.f;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], -tempSideStepRight*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);

						WalkingPhaseNext[Y] = LEFT_FOOT_DOWN_SET;
					}
				}
				else
				{
					//tempDelayTime[0] = tempDSP+tempSideDelayYRight*tempSideDelayY;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
					//SetMoveTaskPos(RIGHT, Y, tempSideStepRight*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);	// if you change "function mode" here, also should change it in "HIP_TO_CENTER"
					
					
					//tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
					tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;	// by jungwoo 2011.05.26
					if(WalkingStartStop == 0x00)	//0x01 : stop click , 0x00 : during stop mode
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

						WalkingPhaseNext[Y] = HIP_TO_CENTER;
					}
					else	// continuous walking
					{
						//SetMoveTaskPos(LEFT, Y, -tempSideStepLeft*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

						WalkingPhaseNext[Y] = LEFT_FOOT_DOWN_SET;
					}

					//SetMoveTaskPosJointFF(&Joint[RHR], -tempHipComp, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = LEFT_FOOT_UP_SET;
			}
			
			break;
		case LEFT_FOOT_DOWN_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				if(tempSideStep>=0)
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYLeft*tempSideDelayY;	// DSP time + margin time
					SetMoveTaskPos(&WalkingInfo[LEFT][Y], tempSideStepLeft*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;	// DSP time + margin time
					SetMoveTaskPos(&WalkingInfo[RIGHT][Y], -tempSideStepRight*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
				}
				else
				{
				//	tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYRight*tempSideDelayY;	// DSP time + margin time
				//	SetMoveTaskPos(RIGHT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
				//	tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;	// DSP time + margin time
				//	SetMoveTaskPos(LEFT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
				
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
					SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					//SetMoveTaskPosJointFF(&Joint[RHR], 0.0f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				
				WalkingPhaseNext[Y] = RIGHT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = LEFT_FOOT_DOWN_SET;
			}
			break;
		case HIP_TO_CENTER:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				if(tempSideStep>=0)
				{					
					//tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYLeft*tempSideDelayY;
					//SetMoveTaskPos(LEFT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
					//SetMoveTaskPosJointFF(&Joint[LHR], 0.0f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				else
				{
					//tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYRight*tempSideDelayY;
					//SetMoveTaskPos(RIGHT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
					//SetMoveTaskPosJointFF(&Joint[RHR], 0.0f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				
				WalkingPhaseNext[Y] = WALK_READY;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = HIP_TO_CENTER;
			}
			break;
		case WALK_READY:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = WALK_READY;
			}
			break;
	}


	// z-direction sequence
	switch(WalkingPhaseNext[Z])
	{
		case FIRST_RIGHT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = tempPeriod/2.0f;	tempDelayTime[1] = 80;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				if(tempSideStep < 0.0f) SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[Z] = RIGHT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = FIRST_RIGHT_FOOT_UP_SET;
			}
			break;
		case FIRST_LEFT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = tempPeriod/2.0f;	tempDelayTime[1] = 80;	tempUserData[0] = 0.0f;
				if(tempSideStep > 0.0f) SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[Z] = LEFT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = FIRST_LEFT_FOOT_UP_SET;
			}
			break;
		case RIGHT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{	
				tempDelayTime[0] = tempDSP;	tempDelayTime[1] = 80;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
				//if(tempSideStep >= 0.0f) SetMoveTaskPos(LEFT, Z, WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				//else SetMoveTaskPos(LEFT, Z, WALK_READY_Z_POS+tempPushLeg, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				if(tempSideStep < 0.0f) SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

				WalkingPhaseNext[Z] = RIGHT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = RIGHT_FOOT_UP_SET;
			}
			break;
		case RIGHT_FOOT_DOWN_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = 80;	tempDelayTime[1] = tempDSP;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				tempDelayTime[0] = 80;	tempDelayTime[1] = tempDSP;
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[Z] = LEFT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = RIGHT_FOOT_DOWN_SET;
			}
			break;
		case LEFT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 80;	tempUserData[0] = 0.0f;
				if(tempSideStep > 0.0f) SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				tempDelayTime[0] = tempDSP;	tempDelayTime[1] = 80;
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[Z] = LEFT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = LEFT_FOOT_UP_SET;
			}
			break;
		case LEFT_FOOT_DOWN_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = 80;	tempDelayTime[1] = tempDSP;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

				tempDelayTime[0] = 80;	tempDelayTime[1] = tempDSP;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

				WalkingPhaseNext[Z] = RIGHT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = LEFT_FOOT_DOWN_SET;
			}
			break;
		case HIP_TO_CENTER:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = 80;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[Z] = WALK_READY;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = HIP_TO_CENTER;
			}
			break;
		case WALK_READY:
			if((WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false))
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = WALK_READY;
			break;
	}


	// yaw-direction sequence
	switch(WalkingPhaseNext[Yaw])
	{
		case FIRST_RIGHT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				if(tempYaw < 0.0f) WalkingStep[Yaw] = tempYaw;

				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.0f;	tempDelayTime[1] = 0.0f;
				if(WalkingStep[Yaw] > 0.0f)
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
				}
				else
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);	
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}

				WalkingPhaseNext[Yaw] = RIGHT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = FIRST_RIGHT_FOOT_UP_SET;
			}
			break;
		case FIRST_LEFT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				if(tempYaw >= 0.0f) WalkingStep[Yaw] = tempYaw;

				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.0f;	tempDelayTime[1] = 0.0f;;
				if(WalkingStep[Yaw] > 0.0f)
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);	
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}
				else 
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);
					SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
				}

				WalkingPhaseNext[Yaw] = LEFT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = FIRST_LEFT_FOOT_UP_SET;
			}
			break;
		case RIGHT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;
				if(WalkingStep[Yaw] >= 0.0f)
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}
				else
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);	
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}
				
				WalkingPhaseNext[Yaw] = RIGHT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = RIGHT_FOOT_UP_SET;
			}
			break;
		case RIGHT_FOOT_DOWN_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;
				if(WalkingStep[Yaw] >= 0.0f) 
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

					if(tempYaw < 0.0f) WalkingStep[Yaw] = 0.0f;
					else WalkingStep[Yaw] = tempYaw;
				}
				else 
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], WalkingStep[Yaw], tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], -WalkingStep[Yaw], tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}

				WalkingPhaseNext[Yaw] = LEFT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = RIGHT_FOOT_DOWN_SET;
			}
			break;
		case LEFT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;
				if(WalkingStep[Yaw] > 0.0f) 
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}
				else 
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}

				WalkingPhaseNext[Yaw] = LEFT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = LEFT_FOOT_UP_SET;
			}
			break;
		case LEFT_FOOT_DOWN_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;
				if(WalkingStep[Yaw] > 0.0f) 
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], -WalkingStep[Yaw], tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], WalkingStep[Yaw], tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}
				else 
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}

					if(tempYaw >= 0.0f) WalkingStep[Yaw] = 0.0f;
					else WalkingStep[Yaw] = tempYaw;
				}
				
				WalkingPhaseNext[Yaw] = RIGHT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = LEFT_FOOT_DOWN_SET;
			}
			break;
		case HIP_TO_CENTER:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
				SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

				WalkingPhaseNext[Yaw] = WALK_READY;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = HIP_TO_CENTER;
			}
			break;
		case WALK_READY:
			if((WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false))
			{
				WalkingStep[Yaw] = 0.0f;;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = WALK_READY;
			}
			break;
	}

	// basic sway movement
	switch(WalkingPhaseNextSway)
	{
	case HIP_CENTER_TO_LEFT_SET:
		if(WalkingInfoSway.MoveFlag == false)
		{
			tempDSP = 40.0f;	tempDelayY = 0.0f;	tempDelayTime[0] = tempDSP;	tempDelayTime[1] = tempDelayY;
			SetMoveTaskPos(&WalkingInfoSway, -tempStartSway*tempSway, tempPeriod, tempDelayTime,tempUserData, 1, 0);
			//bool SetMoveTaskPos(WALKING_INFO* _walkingInfo, float _pos, float _msTime, float _msDelayTime[2], float _userData[5], unsigned char _mode, unsigned char _function)
			WalkingPhaseNextSway = HIP_LEFT_TO_RIGHT_SET;
			WalkingInfoSway.CurrentSequence = HIP_CENTER_TO_LEFT_SET;

			// for walking step count checking
			if(tempCount == 0) 
			{
				if(tempStep == 0.0f) WalkingStartStop = 0x00;
				else WalkingStartStop = 0x01;
			}
			else if(tempCount == -1) tempCount = -1;
			else pSharedMemory->JW_temp[17] -=1.0f;

// by jungho77
			if(currentWalkingStep >= numberOfWalking)
			{
				//pSharedMemory->JW_temp[17] = 0.0f;
				//currentWalkingStep = 0;
				WalkingStartStop = 0x01;
				pSharedMemory->MotorControlMode = CTRLMODE_WALKING;
			}
			else currentWalkingStep++;
// end

			SetMoveJointAngle(LSP, -10.f*tempStep+Joint[LSP].WalkReadyAngle-10.f, tempPeriod, 0x01);
			SetMoveJointAngle(RSP, 10.f*tempStep+Joint[RSP].WalkReadyAngle+10.f, tempPeriod, 0x01);
		}
		break;
	case HIP_CENTER_TO_RIGHT_SET:
		if(WalkingInfoSway.MoveFlag == false)
		{
			tempDSP = 40.0f;	tempDelayY = 0.0f;	tempDelayTime[0] = tempDSP;	tempDelayTime[1] = tempDelayY;
			SetMoveTaskPos(&WalkingInfoSway, tempStartSway*tempSway, tempPeriod, tempDelayTime, tempUserData, 1, 0);
		
			WalkingPhaseNextSway = HIP_RIGHT_TO_LEFT_SET;
			WalkingInfoSway.CurrentSequence = HIP_CENTER_TO_RIGHT_SET;
			
			// for walking step count checking
			if(tempCount == 0) 
			{
				if(tempStep == 0.0f) WalkingStartStop = 0x00;
				else WalkingStartStop = 0x01;
			}
			else if(tempCount == -1) tempCount = -1;
			else pSharedMemory->JW_temp[17] -=1.0f;


// by jungho77
			if(currentWalkingStep >= numberOfWalking)
			{
				//pSharedMemory->JW_temp[17] = 0.0f;
				//currentWalkingStep = 0;
				WalkingStartStop = 0x01;
				pSharedMemory->MotorControlMode = CTRLMODE_WALKING;
			}
			else currentWalkingStep++;
// end

			SetMoveJointAngle(LSP, 10.f*tempStep+Joint[LSP].WalkReadyAngle+10.f, tempPeriod, 0x01);
			SetMoveJointAngle(RSP, -10.f*tempStep+Joint[RSP].WalkReadyAngle-10.f, tempPeriod, 0x01);

		}
		break;
	case HIP_LEFT_TO_RIGHT_SET:
		if(WalkingInfoSway.MoveFlag == false)
		{	
			tempDelayTime[0] = tempDelayYLeft*tempDelayY;	tempDelayTime[1] = tempDelayYRight*tempDelayY;
			if(WalkingStartStop == 0x01)
			{
				tempCount = 0;	WalkingStartStop = 0x00;
				SetMoveTaskPos(&WalkingInfoSway, tempStopSway*tempSway, tempPeriod, tempDelayTime, tempUserData, 1, 0);
			}
			else 
			{
				SetMoveTaskPos(&WalkingInfoSway, tempSwayRight*tempSway, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				// for walking step count checking
				if(tempCount == 0)
				{
					if(tempStep == 0.0f) WalkingStartStop = 0x00;
					else WalkingStartStop = 0x01;
				}
				else if(tempCount == -1) tempCount = -1;
				else pSharedMemory->JW_temp[17] -=1.0f;

// by jungho77
				if(currentWalkingStep >= numberOfWalking)
				{
					//pSharedMemory->JW_temp[17] = 0.0f;
					//currentWalkingStep = 0;
					WalkingStartStop = 0x01;
					pSharedMemory->MotorControlMode = CTRLMODE_WALKING;
				}
				else currentWalkingStep++;
// end
			}

			SetMoveJointAngle(LSP, 10.f*tempStep+Joint[LSP].WalkReadyAngle+10.f, tempPeriod, 0x01);
			SetMoveJointAngle(RSP, -10.f*tempStep+Joint[RSP].WalkReadyAngle-10.f, tempPeriod, 0x01);

			WalkingPhaseNextSway = HIP_RIGHT_TO_LEFT_SET;
			WalkingInfoSway.CurrentSequence = HIP_LEFT_TO_RIGHT_SET;
		}
		break;
	case HIP_RIGHT_TO_LEFT_SET:
		if(WalkingInfoSway.MoveFlag == false)
		{	
			tempDelayTime[0] = tempDelayYRight*tempDelayY;	tempDelayTime[1] = tempDelayYLeft*tempDelayY;
			if(WalkingStartStop == 0x01)
			{
				tempCount = 0;	WalkingStartStop = 0x00;
				SetMoveTaskPos(&WalkingInfoSway, -tempStopSway*tempSway, tempPeriod, tempDelayTime, tempUserData, 1, 0);
			}
			else 
			{
				SetMoveTaskPos(&WalkingInfoSway, -tempSwayLeft*tempSway, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				// for walking step count checking
				if(tempCount == 0)
				{
					if(tempStep == 0.0f) WalkingStartStop = 0x00;
					else WalkingStartStop = 0x01;
				}
				else if(tempCount == -1) tempCount = -1;
				else pSharedMemory->JW_temp[17] -=1.0f;

// by jungho77
				if(currentWalkingStep >= numberOfWalking)
				{
					//pSharedMemory->JW_temp[17] = 0.0f;
					//currentWalkingStep = 0;
					WalkingStartStop = 0x01;
					pSharedMemory->MotorControlMode = CTRLMODE_WALKING;
				}
				else currentWalkingStep++;
// end
			}
			
			SetMoveJointAngle(LSP, -10.f*tempStep+Joint[LSP].WalkReadyAngle-10.f, tempPeriod, 0x01);
			SetMoveJointAngle(RSP, 10.f*tempStep+Joint[RSP].WalkReadyAngle+10.f, tempPeriod, 0x01);

			WalkingPhaseNextSway = HIP_LEFT_TO_RIGHT_SET;
			WalkingInfoSway.CurrentSequence = HIP_RIGHT_TO_LEFT_SET;
		}
		break;
	case HIP_TO_CENTER:
		if(WalkingInfoSway.MoveFlag == false)
		{
			tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 50.0f;
			SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
			SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);


			//tempDelayTime[0] = tempDelayY;	tempDelayTime[1] = 0.0f;
			tempDelayTime[0] = 100.0f;	tempDelayTime[1] = 0.0f;
			SetMoveTaskPos(&WalkingInfoSway, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
			SetMoveJointAngle(LSP, Joint[LSP].WalkReadyAngle, tempPeriod, 0x01);
			SetMoveJointAngle(RSP, Joint[RSP].WalkReadyAngle, tempPeriod, 0x01);
			WalkingPhaseNextSway = WALK_READY;
			WalkingInfoSway.CurrentSequence = HIP_TO_CENTER;
		}
		break;
	case WALK_READY:
		if(WalkingInfoSway.MoveFlag == false) WalkingInfoSway.CurrentSequence = WALK_READY;
		break;
	}

	lastStep = tempStep;
}
/******************************************************************************/






/******************************************************************************/
void TestProfileSequence(void)

{
	/*
	pSharedMemory->JW_temp[0]	= m_swayValue;
	pSharedMemory->JW_temp[3]	= (float)m_holdTime;
	pSharedMemory->JW_temp[6]	= m_forwardStep/2.0f;
	pSharedMemory->JW_temp[9]	= m_sideStep/2.0f;
	pSharedMemory->JW_temp[12]	= m_rotationStep/2.0f;
	pSharedMemory->JW_temp[15]	= (float)m_dspTime/2.0f;
	pSharedMemory->JW_temp[16]	= (float)m_stepPeriod;
	pSharedMemory->JW_temp[17]	= (float)m_stepCount;
	pSharedMemory->JW_temp[18]	= m_startSway;
	pSharedMemory->JW_temp[19]	= m_stopSway;
	
	pSharedMemory->JW_temp[1] = 1.0f;
	pSharedMemory->JW_temp[2] = 1.0f;
	pSharedMemory->JW_temp[4] = 1.0f;
	pSharedMemory->JW_temp[5] = 1.0f;
	pSharedMemory->JW_temp[7] = 0.0f;
	pSharedMemory->JW_temp[8] = 0.0f;
	pSharedMemory->JW_temp[10] = 1.0f;
	pSharedMemory->JW_temp[11] = 1.0f;
	
	pSharedMemory->JW_temp[13] = 1.0f;
	pSharedMemory->JW_temp[14] = 1.0f;
	
	pSharedMemory->JW_temp[20] = 60.0f;	// 60.0f;
	pSharedMemory->JW_temp[21] = 1.0f;
	pSharedMemory->JW_temp[22] = 1.0f;
	pSharedMemory->JW_temp[23] = 0.0f;
	pSharedMemory->JW_temp[24] = 0.0f;
	pSharedMemory->JW_temp[25] = 0.0f;
	pSharedMemory->JW_temp[26] = 0.0f;
	pSharedMemory->JW_temp[27] = m_grsr;
	pSharedMemory->JW_temp[28] = m_greb;
	pSharedMemory->JW_temp[29] = m_grwy;
	pSharedMemory->JW_temp[30] = m_mx;
	pSharedMemory->JW_temp[31] = m_my;
	*/
	float tempDelayTime[2];
	float tempSway			= pSharedMemory->JW_temp[0];//0.6
	float tempSwayLeft		= pSharedMemory->JW_temp[1];//1.0
	float tempSwayRight		= pSharedMemory->JW_temp[2];//1.0
	float tempDelayY		= pSharedMemory->JW_temp[3];// 60.0f;
	float tempDelayYLeft	= pSharedMemory->JW_temp[4];//1.0
	float tempDelayYRight	= pSharedMemory->JW_temp[5];//1.0
	//float 
	//	tempStep			= pSharedMemory->JW_temp[6];//step/2
	float tempStepLeft		= pSharedMemory->JW_temp[7];
	float tempStepRight		= pSharedMemory->JW_temp[8];
	float tempSideStep		= pSharedMemory->JW_temp[9];
	float tempSideStepLeft	= pSharedMemory->JW_temp[10];
	float tempSideStepRight = pSharedMemory->JW_temp[11];
	float tempYaw			= pSharedMemory->JW_temp[12];
	float tempDSP			= pSharedMemory->JW_temp[15];	// 20.0f;
	float tempPeriod		= pSharedMemory->JW_temp[16];	// 800.0f;
	int  tempCount			= (int)pSharedMemory->JW_temp[17];
	float tempStartSway		= pSharedMemory->JW_temp[18];
	float tempStopSway		= pSharedMemory->JW_temp[19];
	float tempSideDelayY		= pSharedMemory->JW_temp[20];	// 60.0f;
	float tempSideDelayYLeft	= pSharedMemory->JW_temp[21];
	float tempSideDelayYRight	= pSharedMemory->JW_temp[22];
//	float tempHipComp			= 0.0f;	// for hip compensation during side walk

	float tempUserData[5];

	//static float 
	//	lastStep = tempStep;
	//static float 
	//	lastStep2 = tempStep;
	//static float 
	//	lastStep3 = tempStep;
	static float lastStep5 = tempStep;

	//static unsigned char StepChangeFlag = 0x00;
	static unsigned char StepChangeFlag2 =0x00;
	
	static unsigned char start = 0x00;
	
	
	
	
	if(StepChangeFlag2 ==0x00)
	{
		tempStep = pSharedMemory->JW_temp[6];//step/2
		if(start == 0x00)
		{
			lastStep = tempStep;
			start = 0x01;
		}
		
	}


	if((StepChangeFlag2 ==0x00)&&(lastStep != tempStep))//&&((lastStep==0.0f)&&(tempStep>0)))
	{
		//printf("1");
		if((tempStep!=0)&&(lastStep==0.0f))
		{
		//	printf("2");
			StepChangeFlag = 0x01;
		}
		else if((lastStep!=0)&&(tempStep==0.0f))
		{
		//	printf("3");
			if((StepChangeFlag ==0x02)||(StepChangeFlag ==0x01))//||(StepChangeFlag ==0x03)||(StepChangeFlag ==0x04))
			{
				StepChangeFlag2 = 0x01;
				tempStep = lastStep;
				lastStep5 = tempStep;
			}			
			else
			StepChangeFlag = 0x03;
		}

		lastStep2 = lastStep;
	}

	if((StepChangeFlag2 ==0x01)&&(StepChangeFlag ==0x00))
	{	
		StepChangeFlag = 0x03;
		StepChangeFlag2 = 0x00;
		tempStep = lastStep5;
	}

	if(lastStep3 != lastStep2) lastStep3 = lastStep2;

	if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
	lastStep4 = lastStep;


	// Stop walking command check
	//if(tempSideStep >= 0.0f) { if( (WalkingStartStop==0x00) && (WalkingPhaseNext[Z]==RIGHT_FOOT_DOWN_SET) ) WalkingPhaseNextSway = WalkingPhaseNext[X] = WalkingPhaseNext[Y] = WalkingPhaseNext[Z] = WalkingPhaseNext[Yaw] = HIP_TO_CENTER; }
	//else { if( (WalkingStartStop==0x00) && (WalkingPhaseNext[Z]==LEFT_FOOT_DOWN_SET) ) WalkingPhaseNextSway = WalkingPhaseNext[X] = WalkingPhaseNext[Y] = WalkingPhaseNext[Z] = WalkingPhaseNext[Yaw] = HIP_TO_CENTER; }

	if( (WalkingStartStop==0x00) && (WalkingPhaseNext[Z]==RIGHT_FOOT_DOWN_SET) ) WalkingPhaseNextSway = WalkingPhaseNext[X] = WalkingPhaseNext[Y] = WalkingPhaseNext[Z] = WalkingPhaseNext[Yaw] = HIP_TO_CENTER;
	if( (WalkingStartStop==0x00) && (WalkingPhaseNext[Z]==LEFT_FOOT_DOWN_SET) ) WalkingPhaseNextSway = WalkingPhaseNext[X] = WalkingPhaseNext[Y] = WalkingPhaseNext[Z] = WalkingPhaseNext[Yaw] = HIP_TO_CENTER;


	// x-direction sequence
	switch(WalkingPhaseNext[X])
	{
		case FIRST_RIGHT_FOOT_UP_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				//printf("\n FRU,%d ",StepChangeFlag);
				tempDelayTime[0] = tempPeriod/2.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][X], tempStep, tempPeriod, tempDelayTime, tempUserData, 1, 0x04);
				SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);
			
				WalkingPhaseNext[X] = FIRST_RIGHT_FOOT_DOWN_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = FIRST_RIGHT_FOOT_UP_FORWARD_SET;
			}
			break;
		case FIRST_LEFT_FOOT_UP_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				//printf("\n FLU,%d ",StepChangeFlag);
				tempDelayTime[0] = tempPeriod/2.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][X], tempStep, tempPeriod, tempDelayTime, tempUserData, 1, 0x04);
				SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[X] = FIRST_LEFT_FOOT_DOWN_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = FIRST_LEFT_FOOT_UP_FORWARD_SET;
			}
			break;
		case FIRST_RIGHT_FOOT_DOWN_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				//printf("\n FRD,%d ",StepChangeFlag);
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = tempDSP; tempUserData[1] = lastStep3;
				SetMoveTaskPos(&WalkingInfo[RIGHT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x06);
				SetMoveTaskPos(&WalkingInfo[LEFT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);

				WalkingPhaseNext[X] = LEFT_FOOT_UP_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = FIRST_RIGHT_FOOT_DOWN_FORWARD_SET;
			}
			break;
		case FIRST_LEFT_FOOT_DOWN_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				///printf("\n FLD,%d ",StepChangeFlag);
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = tempDSP; tempUserData[1] = lastStep3;
				SetMoveTaskPos(&WalkingInfo[LEFT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x06);
				SetMoveTaskPos(&WalkingInfo[RIGHT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
				
				WalkingPhaseNext[X] = RIGHT_FOOT_UP_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = FIRST_LEFT_FOOT_DOWN_FORWARD_SET;
			}
			break;
		case RIGHT_FOOT_UP_FORWARD_SET:
			
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				//printf("\n RU,%d ",StepChangeFlag);
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				if(WalkingStartStop == 0x00)
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x09);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x02);
						
					WalkingPhaseNext[X] = HIP_TO_CENTER;
				}
				else
				{
					if(StepChangeFlag == 0x00)
					{
						tempUserData[0] = tempDSP;
						SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x07);
						tempUserData[0] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x02);
					}
					else if(StepChangeFlag == 0x01)
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x04);
						SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						StepChangeFlag = 0x02;
					}
					else if(StepChangeFlag == 0x03)
					{
						tempUserData[1] = lastStep3;
						SetMoveTaskPos(&WalkingInfo[RIGHT][X], -lastStep2, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x09);
						SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x02);
						StepChangeFlag = 0x04;
					}
					WalkingPhaseNext[X] = RIGHT_FOOT_DOWN_FORWARD_SET;
				}
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = RIGHT_FOOT_UP_FORWARD_SET;
			}
			break;
		case RIGHT_FOOT_DOWN_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				//printf("\n RD,%d ",StepChangeFlag);
				if((StepChangeFlag==0x00))
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = tempDSP;	tempUserData[1] = lastStep3;
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x08);
					
					tempUserData[0] = 0.0f;
					SetMoveTaskPos(&WalkingInfo[LEFT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
				}				
				else if(StepChangeFlag == 0x02)
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = tempDSP; tempUserData[1] = lastStep3;
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x06);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
					StepChangeFlag = 0x00;
				}
				else if((StepChangeFlag==0x01)||(StepChangeFlag==0x03))
				{
					
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = tempDSP;	tempUserData[1] = lastStep3;
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], lastStep2, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x08);
					
					tempUserData[0] = 0.0f;
					SetMoveTaskPos(&WalkingInfo[LEFT][X], -lastStep2, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
				}
				else if(StepChangeFlag==0x04)
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;		tempUserData[1] = lastStep3;
					
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 10);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					StepChangeFlag=0x00;
					//printf("in");
				}
				
				WalkingPhaseNext[X] = LEFT_FOOT_UP_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = RIGHT_FOOT_DOWN_FORWARD_SET;
			}
			break;
		case LEFT_FOOT_UP_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				//printf("\n LU,%d ",StepChangeFlag);
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				if(WalkingStartStop == 0x00)
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x02);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x09);
					
					WalkingPhaseNext[X] = HIP_TO_CENTER;
				}
				else
				{
					if(StepChangeFlag==0x00)
					{
						tempUserData[0] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x02);
						
						tempUserData[0] = tempDSP;
						SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x07);
					}
					else if(StepChangeFlag==0x01)
					{
						tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[LEFT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x04);
						SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						StepChangeFlag=0x02;
					}
					else if(StepChangeFlag==0x03)
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x02);
						tempUserData[1] = lastStep3;
						SetMoveTaskPos(&WalkingInfo[LEFT][X], -lastStep2, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x09);
						StepChangeFlag=0x04;
					}
					
					WalkingPhaseNext[X] = LEFT_FOOT_DOWN_FORWARD_SET;
				}

				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = LEFT_FOOT_UP_FORWARD_SET;
			}
			break;
		case LEFT_FOOT_DOWN_FORWARD_SET:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				//printf("\n LD,%d ",StepChangeFlag);
				if((StepChangeFlag==0x00))
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
					
					tempUserData[0] = tempDSP;
					SetMoveTaskPos(&WalkingInfo[LEFT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x08);
				}
				else if(StepChangeFlag==0x02)
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = tempDSP; tempUserData[1] = lastStep3;
					SetMoveTaskPos(&WalkingInfo[LEFT][X], tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x06);
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], -tempStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
					StepChangeFlag=0x00;
				}
				else if((StepChangeFlag==0x01)||(StepChangeFlag==0x03))
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], -lastStep2, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x01);
					
					tempUserData[0] = tempDSP;	tempUserData[1] = lastStep3;
					SetMoveTaskPos(&WalkingInfo[LEFT][X], lastStep2, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0x08);

				}
				else if((StepChangeFlag==0x04))
				{

					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;		tempUserData[1] = lastStep3;
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 10);
					StepChangeFlag=0x00;
					//printf("in");
				}
				
				WalkingPhaseNext[X] = RIGHT_FOOT_UP_FORWARD_SET;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = LEFT_FOOT_DOWN_FORWARD_SET;
			}
			break;
		case HIP_TO_CENTER:
			if( (WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false) )
			{
				//printf("\n HC,%d ",StepChangeFlag);
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;		tempUserData[1] = lastStep3;
				if(WalkingInfo[RIGHT][X].CurrentSequence == LEFT_FOOT_UP_FORWARD_SET)
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 10);
				}
				else if(WalkingInfo[RIGHT][X].CurrentSequence == RIGHT_FOOT_UP_FORWARD_SET)
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 10);
					SetMoveTaskPos(&WalkingInfo[LEFT][X], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				}
				WalkingPhaseNext[X] = WALK_READY;
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = HIP_TO_CENTER;
			}
			break;
		case WALK_READY:
			if((WalkingInfo[RIGHT][X].MoveFlag == false) && (WalkingInfo[LEFT][X].MoveFlag == false))
				WalkingInfo[RIGHT][X].CurrentSequence = WalkingInfo[LEFT][X].CurrentSequence = WALK_READY;
			lastStep = lastStep2 = lastStep3 = 0.0f;
			StepChangeFlag=0x00;
			break;
	}


	// y-direction sequence
	switch(WalkingPhaseNext[Y])
	{
		case FIRST_RIGHT_FOOT_UP_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.0f+tempSideDelayYRight*tempSideDelayY/2.0f;	tempDelayTime[1] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Y], tempSideStepRight*tempSideStep/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);

				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.f;	tempDelayTime[1] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Y], -tempSideStepLeft*tempSideStep/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);
			
				WalkingPhaseNext[Y] = FIRST_RIGHT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = FIRST_RIGHT_FOOT_UP_SET;
			}
			break;
		case FIRST_RIGHT_FOOT_DOWN_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f+tempSideDelayYRight*tempSideDelayY/2.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Y], tempSideStepRight*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Y], -tempSideStepLeft*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
			
				WalkingPhaseNext[Y] = LEFT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = FIRST_RIGHT_FOOT_DOWN_SET;
			}
			break;
		case FIRST_LEFT_FOOT_UP_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{	
				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.0f+tempSideDelayYLeft*tempSideDelayY/2.0f;	tempDelayTime[1] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Y], tempSideStepLeft*tempSideStep/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);

				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.f;	tempDelayTime[1] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Y], -tempSideStepRight*tempSideStep/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);

				WalkingPhaseNext[Y] = FIRST_LEFT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = FIRST_LEFT_FOOT_UP_SET;
			}
			break;
		case FIRST_LEFT_FOOT_DOWN_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.f+tempSideDelayYLeft*tempSideDelayY/2.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Y], tempSideStepLeft*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Y], -tempSideStepRight*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

				WalkingPhaseNext[Y] = RIGHT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = FIRST_LEFT_FOOT_DOWN_SET;
			}
			break;
		case RIGHT_FOOT_UP_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				if(tempSideStep>=0)
				{
					//tempDelayTime[0] = tempDSP+tempSideDelayYLeft*tempSideDelayY;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
					//SetMoveTaskPos(LEFT, Y, tempSideStepLeft*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);	// if you change "function mode" here, also should change it in "HIP_TO_CENTER"
					
					if(WalkingStartStop == 0x00)	//0x01 : stop click , 0x00 : during stop mode
					{
						tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						
						WalkingPhaseNext[Y] = HIP_TO_CENTER;
					}
					else	// continuous walking
					{
						//SetMoveTaskPos(RIGHT, Y, -tempSideStepRight*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
						tempDelayTime[0] = tempDSP+20.0f+tempSideDelayYRight*tempSideDelayY;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

						WalkingPhaseNext[Y] = RIGHT_FOOT_DOWN_SET;
					}

					//SetMoveTaskPosJointFF(&Joint[LHR], tempHipComp, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				else
				{
					tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
					if(WalkingStartStop == 0x00)	//0x01 : stop click , 0x00 : during stop mode
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						
						WalkingPhaseNext[Y] = HIP_TO_CENTER;
					}
					else
					{
						tempDelayTime[0] = tempDSP+tempSideDelayYRight*tempSideDelayY;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], tempSideStepRight*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);

						tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], -tempSideStepLeft*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);

						WalkingPhaseNext[Y] = RIGHT_FOOT_DOWN_SET;
					}
				}
				
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = RIGHT_FOOT_UP_SET;
			}
			break;
		case RIGHT_FOOT_DOWN_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				if(tempSideStep>=0)
				{
				//	tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYLeft*tempSideDelayY;	// DSP time + margin time
				//	SetMoveTaskPos(LEFT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
				//	tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;	// DSP time + margin time
				//	SetMoveTaskPos(RIGHT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
					SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					//SetMoveTaskPosJointFF(&Joint[LHR], 0.0f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				else
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYRight*tempSideDelayY;	// DSP time + margin time
					SetMoveTaskPos(&WalkingInfo[RIGHT][Y], tempSideStepRight*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;	// DSP time + margin time
					SetMoveTaskPos(&WalkingInfo[LEFT][Y], -tempSideStepLeft*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
				}
				
				WalkingPhaseNext[Y] = LEFT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = RIGHT_FOOT_DOWN_SET;
			}
			break;
		case LEFT_FOOT_UP_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				if(tempSideStep>=0)
				{
					if(WalkingStartStop == 0x00)	//0x01 : stop click , 0x00 : during stop mode
					{
						tempDelayTime[0] = tempDSP+20.f;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						tempDelayTime[0] = tempDSP+20.0f+tempSideDelayYLeft*tempSideDelayY;	tempDelayTime[1] = 0.0f;
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

						WalkingPhaseNext[Y] = HIP_TO_CENTER;
					}
					else
					{
						tempDelayTime[0] = tempDSP+tempSideDelayYLeft*tempSideDelayY;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], tempSideStepLeft*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
					
						tempDelayTime[0] = tempDSP+20.f;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], -tempSideStepRight*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);

						WalkingPhaseNext[Y] = LEFT_FOOT_DOWN_SET;
					}
				}
				else
				{
					//tempDelayTime[0] = tempDSP+tempSideDelayYRight*tempSideDelayY;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
					//SetMoveTaskPos(RIGHT, Y, tempSideStepRight*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);	// if you change "function mode" here, also should change it in "HIP_TO_CENTER"
					
					
					//tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;	// DSP time + margin time
					tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;	// by jungwoo 2011.05.26
					if(WalkingStartStop == 0x00)	//0x01 : stop click , 0x00 : during stop mode
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

						WalkingPhaseNext[Y] = HIP_TO_CENTER;
					}
					else	// continuous walking
					{
						//SetMoveTaskPos(LEFT, Y, -tempSideStepLeft*tempSideStep/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
						SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
						SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

						WalkingPhaseNext[Y] = LEFT_FOOT_DOWN_SET;
					}

					//SetMoveTaskPosJointFF(&Joint[RHR], -tempHipComp, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = LEFT_FOOT_UP_SET;
			}
			
			break;
		case LEFT_FOOT_DOWN_SET:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				if(tempSideStep>=0)
				{
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYLeft*tempSideDelayY;	// DSP time + margin time
					SetMoveTaskPos(&WalkingInfo[LEFT][Y], tempSideStepLeft*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;	// DSP time + margin time
					SetMoveTaskPos(&WalkingInfo[RIGHT][Y], -tempSideStepRight*tempSideStep, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
				}
				else
				{
				//	tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYRight*tempSideDelayY;	// DSP time + margin time
				//	SetMoveTaskPos(RIGHT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
				//	tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;	// DSP time + margin time
				//	SetMoveTaskPos(LEFT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
				
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
					SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
					//SetMoveTaskPosJointFF(&Joint[RHR], 0.0f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				
				WalkingPhaseNext[Y] = RIGHT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = LEFT_FOOT_DOWN_SET;
			}
			break;
		case HIP_TO_CENTER:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				if(tempSideStep>=0)
				{					
					//tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYLeft*tempSideDelayY;
					//SetMoveTaskPos(LEFT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
					//SetMoveTaskPosJointFF(&Joint[LHR], 0.0f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				else
				{
					//tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+tempSideDelayYRight*tempSideDelayY;
					//SetMoveTaskPos(RIGHT, Y, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					
					tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
					//SetMoveTaskPosJointFF(&Joint[RHR], 0.0f, tempPeriod/2.0f, tempDelayTime, 1, 0);
				}
				
				WalkingPhaseNext[Y] = WALK_READY;
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = HIP_TO_CENTER;
			}
			break;
		case WALK_READY:
			if((WalkingInfo[RIGHT][Y].MoveFlag == false) && (WalkingInfo[LEFT][Y].MoveFlag == false))
			{
				WalkingInfo[RIGHT][Y].CurrentSequence = WalkingInfo[LEFT][Y].CurrentSequence = WALK_READY;
			}
			break;
	}


	// z-direction sequence
	switch(WalkingPhaseNext[Z])
	{
		case FIRST_RIGHT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = tempPeriod/2.0f;	tempDelayTime[1] = 80.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				if(tempSideStep < 0.0f) SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[Z] = RIGHT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = FIRST_RIGHT_FOOT_UP_SET;
			}
			break;
		case FIRST_LEFT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = tempPeriod/2.0f;	tempDelayTime[1] = 80.0f;	tempUserData[0] = 0.0f;
				if(tempSideStep > 0.0f) SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[Z] = LEFT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = FIRST_LEFT_FOOT_UP_SET;
			}
			break;
		case RIGHT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{	
				tempDelayTime[0] = tempDSP;	tempDelayTime[1] = 80.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
				//if(tempSideStep >= 0.0f) SetMoveTaskPos(LEFT, Z, WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				//else SetMoveTaskPos(LEFT, Z, WALK_READY_Z_POS+tempPushLeg, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				if(tempSideStep < 0.0f) SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

				WalkingPhaseNext[Z] = RIGHT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = RIGHT_FOOT_UP_SET;
			}
			break;
		case RIGHT_FOOT_DOWN_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = 80.0f;	tempDelayTime[1] = tempDSP;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				tempDelayTime[0] = 80.0f;	tempDelayTime[1] = tempDSP;
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[Z] = LEFT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = RIGHT_FOOT_DOWN_SET;
			}
			break;
		case LEFT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 80.0f;	tempUserData[0] = 0.0f;
				if(tempSideStep > 0.0f) SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				tempDelayTime[0] = tempDSP;	tempDelayTime[1] = 80.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[Z] = LEFT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = LEFT_FOOT_UP_SET;
			}
			break;
		case LEFT_FOOT_DOWN_SET:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = 80.0f;	tempDelayTime[1] = tempDSP;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

				tempDelayTime[0] = 80.0f;	tempDelayTime[1] = tempDSP;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);

				WalkingPhaseNext[Z] = RIGHT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = LEFT_FOOT_DOWN_SET;
			}
			break;
		case HIP_TO_CENTER:
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = 80.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
				
				WalkingPhaseNext[Z] = WALK_READY;
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = HIP_TO_CENTER;
			}
			break;
		case WALK_READY:
			if((WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false))
				WalkingInfo[RIGHT][Z].CurrentSequence = WalkingInfo[LEFT][Z].CurrentSequence = WALK_READY;
			break;
	}


	// yaw-direction sequence
	switch(WalkingPhaseNext[Yaw])
	{
		case FIRST_RIGHT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				if(tempYaw < 0.0f) WalkingStep[Yaw] = tempYaw;

				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.0f;	tempDelayTime[1] = 0.0f;
				if(WalkingStep[Yaw] > 0.0f)
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
				}
				else
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);	
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}

				WalkingPhaseNext[Yaw] = RIGHT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = FIRST_RIGHT_FOOT_UP_SET;
			}
			break;
		case FIRST_LEFT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				if(tempYaw >= 0.0f) WalkingStep[Yaw] = tempYaw;

				tempDelayTime[0] = tempPeriod/2.0f+tempDSP+20.0f;	tempDelayTime[1] = 0.0f;;
				if(WalkingStep[Yaw] > 0.0f)
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod, tempDelayTime, tempUserData, 1, 1);	
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}
				else 
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);
					SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
				}

				WalkingPhaseNext[Yaw] = LEFT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = FIRST_LEFT_FOOT_UP_SET;
			}
			break;
		case RIGHT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;
				if(WalkingStep[Yaw] >= 0.0f)
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}
				else
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);	
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}
				
				WalkingPhaseNext[Yaw] = RIGHT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = RIGHT_FOOT_UP_SET;
			}
			break;
		case RIGHT_FOOT_DOWN_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;
				if(WalkingStep[Yaw] >= 0.0f) 
				{
					SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

					if(tempYaw < 0.0f) WalkingStep[Yaw] = 0.0f;
					else WalkingStep[Yaw] = tempYaw;
				}
				else 
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], WalkingStep[Yaw], tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], -WalkingStep[Yaw], tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}

				WalkingPhaseNext[Yaw] = LEFT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = RIGHT_FOOT_DOWN_SET;
			}
			break;
		case LEFT_FOOT_UP_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				tempDelayTime[0] = tempDSP+20.0f;	tempDelayTime[1] = 0.0f;
				if(WalkingStep[Yaw] > 0.0f) 
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}
				else 
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], -WalkingStep[Yaw]/2.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 1);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}

				WalkingPhaseNext[Yaw] = LEFT_FOOT_DOWN_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = LEFT_FOOT_UP_SET;
			}
			break;
		case LEFT_FOOT_DOWN_SET:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;
				if(WalkingStep[Yaw] > 0.0f) 
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], -WalkingStep[Yaw], tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], WalkingStep[Yaw], tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}
				}
				else 
				{
					if( (tempCount>1) || (tempCount==-1) )
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
					}
					else
					{
						SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
						SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod, tempDelayTime, tempUserData, 1, 0);	
					}

					if(tempYaw >= 0.0f) WalkingStep[Yaw] = 0.0f;
					else WalkingStep[Yaw] = tempYaw;
				}
				
				WalkingPhaseNext[Yaw] = RIGHT_FOOT_UP_SET;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = LEFT_FOOT_DOWN_SET;
			}
			break;
		case HIP_TO_CENTER:
			if( (WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = tempDSP+20.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);
				SetMoveTaskPos(&WalkingInfo[LEFT][Yaw], 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 2);

				WalkingPhaseNext[Yaw] = WALK_READY;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = HIP_TO_CENTER;
			}
			break;
		case WALK_READY:
			if((WalkingInfo[RIGHT][Yaw].MoveFlag == false) && (WalkingInfo[LEFT][Yaw].MoveFlag == false))
			{
				WalkingStep[Yaw] = 0.0f;;
				WalkingInfo[RIGHT][Yaw].CurrentSequence = WalkingInfo[LEFT][Yaw].CurrentSequence = WALK_READY;
			}
			break;
	}

	// basic sway movement
	switch(WalkingPhaseNextSway)
	{
	case HIP_CENTER_TO_LEFT_SET:
		if(WalkingInfoSway.MoveFlag == false)
		{
			tempDSP = 40.0f;	tempDelayY = 0.0f;	tempDelayTime[0] = tempDSP;	tempDelayTime[1] = tempDelayY;
			SetMoveTaskPos(&WalkingInfoSway, -tempStartSway*tempSway, tempPeriod, tempDelayTime,tempUserData, 1, 0);
			//bool SetMoveTaskPos(WALKING_INFO* _walkingInfo, float _pos, float _msTime, float _msDelayTime[2], float _userData[5], unsigned char _mode, unsigned char _function)
			WalkingPhaseNextSway = HIP_LEFT_TO_RIGHT_SET;
			WalkingInfoSway.CurrentSequence = HIP_CENTER_TO_LEFT_SET;

			// for walking step count checking
			if(tempCount == 0) 
			{
				if(tempStep == 0.0f) WalkingStartStop = 0x00;
				else WalkingStartStop = 0x01;
			}
			else if(tempCount == -1) tempCount = -1;
			else pSharedMemory->JW_temp[17] -=1.0f;

			SetMoveJointAngle(LSP, -10.f*tempStep+Joint[LSP].WalkReadyAngle-10.f, tempPeriod, 0x01);
			SetMoveJointAngle(RSP, 10.f*tempStep+Joint[RSP].WalkReadyAngle+10.f, tempPeriod, 0x01);
		}
		break;
	case HIP_CENTER_TO_RIGHT_SET:
		if(WalkingInfoSway.MoveFlag == false)
		{
			tempDSP = 40.0f;	tempDelayY = 0.0f;	tempDelayTime[0] = tempDSP;	tempDelayTime[1] = tempDelayY;
			SetMoveTaskPos(&WalkingInfoSway, tempStartSway*tempSway, tempPeriod, tempDelayTime, tempUserData, 1, 0);
		
			WalkingPhaseNextSway = HIP_RIGHT_TO_LEFT_SET;
			WalkingInfoSway.CurrentSequence = HIP_CENTER_TO_RIGHT_SET;
			
			// for walking step count checking
			if(tempCount == 0) 
			{
				if(tempStep == 0.0f) WalkingStartStop = 0x00;
				else WalkingStartStop = 0x01;
			}
			else if(tempCount == -1) tempCount = -1;
			else pSharedMemory->JW_temp[17] -=1.0f;
			SetMoveJointAngle(LSP, 10.f*tempStep+Joint[LSP].WalkReadyAngle+10.f, tempPeriod, 0x01);
			SetMoveJointAngle(RSP, -10.f*tempStep+Joint[RSP].WalkReadyAngle-10.f, tempPeriod, 0x01);

		}
		break;
	case HIP_LEFT_TO_RIGHT_SET:
		if(WalkingInfoSway.MoveFlag == false)
		{	
			tempDelayTime[0] = tempDelayYLeft*tempDelayY;	tempDelayTime[1] = tempDelayYRight*tempDelayY;
			if(WalkingStartStop == 0x01)
			{
				tempCount = 0;	WalkingStartStop = 0x00;
				SetMoveTaskPos(&WalkingInfoSway, tempStopSway*tempSway, tempPeriod, tempDelayTime, tempUserData, 1, 0);
			}
			else 
			{
				SetMoveTaskPos(&WalkingInfoSway, tempSwayRight*tempSway, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				// for walking step count checking
				if(tempCount == 0)
				{
					if(tempStep == 0.0f) WalkingStartStop = 0x00;
					else WalkingStartStop = 0x01;
				}
				else if(tempCount == -1) tempCount = -1;
				else pSharedMemory->JW_temp[17] -=1.0f;
			}
			SetMoveJointAngle(LSP, 10.f*tempStep+Joint[LSP].WalkReadyAngle+10.f, tempPeriod, 0x01);
			SetMoveJointAngle(RSP, -10.f*tempStep+Joint[RSP].WalkReadyAngle-10.f, tempPeriod, 0x01);

			WalkingPhaseNextSway = HIP_RIGHT_TO_LEFT_SET;
			WalkingInfoSway.CurrentSequence = HIP_LEFT_TO_RIGHT_SET;
		}
		break;
	case HIP_RIGHT_TO_LEFT_SET:
		if(WalkingInfoSway.MoveFlag == false)
		{	
			tempDelayTime[0] = tempDelayYRight*tempDelayY;	tempDelayTime[1] = tempDelayYLeft*tempDelayY;
			if(WalkingStartStop == 0x01)
			{
				tempCount = 0;	WalkingStartStop = 0x00;
				SetMoveTaskPos(&WalkingInfoSway, -tempStopSway*tempSway, tempPeriod, tempDelayTime, tempUserData, 1, 0);
			}
			else 
			{
				SetMoveTaskPos(&WalkingInfoSway, -tempSwayLeft*tempSway, tempPeriod, tempDelayTime, tempUserData, 1, 0);
				
				// for walking step count checking
				if(tempCount == 0)
				{
					if(tempStep == 0.0f) WalkingStartStop = 0x00;
					else WalkingStartStop = 0x01;
				}
				else if(tempCount == -1) tempCount = -1;
				else pSharedMemory->JW_temp[17] -=1.0f;
			}
			SetMoveJointAngle(LSP, -10.f*tempStep+Joint[LSP].WalkReadyAngle-10.f, tempPeriod, 0x01);
			SetMoveJointAngle(RSP, 10.f*tempStep+Joint[RSP].WalkReadyAngle+10.f, tempPeriod, 0x01);

			WalkingPhaseNextSway = HIP_LEFT_TO_RIGHT_SET;
			WalkingInfoSway.CurrentSequence = HIP_RIGHT_TO_LEFT_SET;
		}
		break;
	case HIP_TO_CENTER:
		if(WalkingInfoSway.MoveFlag == false)
		{
			tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 50.0f;
			SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
			SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);


			//tempDelayTime[0] = tempDelayY;	tempDelayTime[1] = 0.0f;
			tempDelayTime[0] = 100.0f;	tempDelayTime[1] = 0.0f;
			SetMoveTaskPos(&WalkingInfoSway, 0.0f, tempPeriod/2.0f, tempDelayTime, tempUserData, 1, 0);
			SetMoveJointAngle(LSP, Joint[LSP].WalkReadyAngle, tempPeriod, 0x01);
			SetMoveJointAngle(RSP, Joint[RSP].WalkReadyAngle, tempPeriod, 0x01);
			WalkingPhaseNextSway = WALK_READY;
			WalkingInfoSway.CurrentSequence = HIP_TO_CENTER;
		}
		break;
	case WALK_READY:
		if(WalkingInfoSway.MoveFlag == false) WalkingInfoSway.CurrentSequence = WALK_READY;
		break;
	}

	lastStep = tempStep;
}
/******************************************************************************/






/******************************************************************************/
bool GoToDemoOneLegSupport(void)
{
	static unsigned char sequence = 0x00;
	float tempUserData[5];
	float tempDelayTime[2];
	float height = 0.10f;

	if(sequence == 0x02) { sequence = 0x00; return false; }
	switch(sequence)
	{
	case 0x00:
		if( (WalkingInfoSway.MoveFlag == false) && (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
		{
			// y-direction sequence
			tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
			SetMoveTaskPos(&WalkingInfoSway, 0.14f, 1000.0f, tempDelayTime, tempUserData, 1, 0);
			// z-direction sequence
			tempDelayTime[0] = 500.0f;	tempDelayTime[1] = 0.0f;;
			SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT+height, 1000.0f, tempDelayTime, tempUserData, 1, 0);
			sequence = 0x01;
		}
		break;
	case 0x01:
		if( (WalkingInfoSway.MoveFlag == false) && (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			sequence = 0x02;
		break;
	}

	return true;
}
/******************************************************************************/




/******************************************************************************/
bool GoToDemoDownAndUp(unsigned char _rightLeft, unsigned char _times)
{
	static unsigned char sequence = 0x00;
	static unsigned char times = _times;
	float tempUserData[5];
	float tempDelayTime[2];
	float height = 0.10f;
	float period = 1000.0f;

	if(times == 0x00) { times = _times; return false; }

	// z-direction sequence
	switch(sequence)
	{
	case 0x00:	// down sequence
		if(_rightLeft == RIGHT)
		{
			if(WalkingInfo[RIGHT][Z].MoveFlag == false)
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS+height, period, tempDelayTime, tempUserData, 1, 0);
				SetMoveTaskPosJointFF(&Joint[RAR], -1.0f, period/2.0f, tempDelayTime, 1, 0);
				sequence = 0x01;
			}
		}
		else if(_rightLeft == LEFT)
		{
			if(WalkingInfo[LEFT][Z].MoveFlag == false)
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS+height, period, tempDelayTime, tempUserData, 1, 0);
				sequence = 0x01;
			}
		}
		else
		{
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[RIGHT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS+height, period, tempDelayTime, tempUserData, 1, 0);
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS+height, period, tempDelayTime, tempUserData, 1, 0);
				sequence = 0x01;
			}
		}
		break;
	case 0x01:
		if(_rightLeft == RIGHT)
		{
			if(WalkingInfo[RIGHT][Z].MoveFlag == false)
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, period, tempDelayTime, tempUserData, 1, 0);
				SetMoveTaskPosJointFF(&Joint[RAR], 0.0f, period/2.0f, tempDelayTime, 1, 0);
				sequence = 0x00;	times--;
			}
		}
		else if(_rightLeft == LEFT)
		{
			if(WalkingInfo[LEFT][Z].MoveFlag == false)
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, period, tempDelayTime, tempUserData, 1, 0);
				sequence = 0x00;	times--;
			}
		}
		else
		{
			if( (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[RIGHT][Z].MoveFlag == false) )
			{
				tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;	tempUserData[0] = 0.0f;
				SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS, period, tempDelayTime, tempUserData, 1, 0);
				SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, period, tempDelayTime, tempUserData, 1, 0);
				sequence = 0x00;	times--;
			}
		}
		break;
	}

	return true;
}
/******************************************************************************/






/******************************************************************************/
bool GoToDemoHomePosition(void)
{
	static unsigned char sequence = 0x00;
	float tempUserData[5];
	float tempDelayTime[2];

	if(sequence == 0x02) { sequence = 0x00; return false; }
	switch(sequence)
	{
	case 0x00:
		if( (WalkingInfoSway.MoveFlag == false) && (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
		{
			// y-direction sequence
			tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
			SetMoveTaskPos(&WalkingInfoSway, 0.0f, 1000.0f, tempDelayTime, tempUserData, 1, 0);
			// z-direction sequence
			tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 200.0f;	tempUserData[0] = 0.0f;
			SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, 1000.0f, tempDelayTime, tempUserData, 1, 0);

			sequence = 0x01;
		}
		break;
	case 0x01:
		if( (WalkingInfoSway.MoveFlag == false) && (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
			sequence = 0x02;
		break;
	}

	return true;
}
/******************************************************************************/









/******************************************************************************/
bool GoToDemoDoubleLegSupport(void)
{
	static unsigned char sequence = 0x00;
	float tempUserData[5];
	float tempDelayTime[2];
	
	if(sequence == 0x02) { sequence = 0x00; return false; }
	switch(sequence)
	{
	case 0x00:
		if( (WalkingInfoSway.MoveFlag == false) && (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
		{
			// y-direction sequence
			tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 50.0f;
			SetMoveTaskPos(&WalkingInfoSway, 0.64f, 1000.0f, tempDelayTime, tempUserData, 1, 0);

			// z-direction sequence
			tempDelayTime[0] = 400.0f;	tempDelayTime[1] = 0.0f;;
			SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT, 1000.0f, tempDelayTime, tempUserData, 1, 0);
			sequence = 0x01;
		}
		break;
	case 0x01:
		if( (WalkingInfoSway.MoveFlag == false) && (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
		{
			// y-direction sequence
			tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
			SetMoveTaskPos(&WalkingInfoSway, 0.0f, 1000.0f, tempDelayTime, tempUserData, 1, 0);
			tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 200.0f;
			SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.05f, 1000.0f, tempDelayTime, tempUserData, 1, 0);
			tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 200.0f;
			SetMoveTaskPos(&WalkingInfo[LEFT][Y], -0.05f, 1000.0f, tempDelayTime, tempUserData, 1, 0);
			
			// z-direction sequence
			tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;;
			SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, 1000.0f, tempDelayTime, tempUserData, 1, 0);
			sequence = 0x02;
		}
		break;
	case 0x02:
		break;
	}
	
	return true;
}
/******************************************************************************/








/******************************************************************************/
bool GoToDemoDoubleHomePosition(void)
{
	static unsigned char sequence = 0x00;
	float tempUserData[5];
	float tempDelayTime[2];

	
	switch(sequence)
	{
	case 0x00:
		if( (WalkingInfoSway.MoveFlag == false) && (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
		{
			// y-direction sequence
			tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 50.0f;
			SetMoveTaskPos(&WalkingInfoSway, 0.064f, 1000.0f, tempDelayTime, tempUserData, 1, 0);
			tempDelayTime[0] = 350.0f;	tempDelayTime[1] = 0.0f;
			SetMoveTaskPos(&WalkingInfo[RIGHT][Y], 0.0f, 1000.0f, tempDelayTime, tempUserData, 1, 0);
			tempDelayTime[0] = 350.0f;	tempDelayTime[1] = 0.0f;
			SetMoveTaskPos(&WalkingInfo[LEFT][Y], 0.0f, 1000.0f, tempDelayTime, tempUserData, 1, 0);

			// z-direction sequence
			tempDelayTime[0] = 300.0f;	tempDelayTime[1] = 0.0f;;
			SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT, 1000.0f, tempDelayTime, tempUserData, 1, 0);
			sequence = 0x01;
		}
		break;
	case 0x01:
		if( (WalkingInfoSway.MoveFlag == false) && (WalkingInfo[RIGHT][Z].MoveFlag == false) && (WalkingInfo[LEFT][Z].MoveFlag == false) )
		{
			// y-direction sequence
			tempDelayTime[0] = 0.0f;	tempDelayTime[1] = 0.0f;
			SetMoveTaskPos(&WalkingInfoSway, 0.0f, 1000.0f, tempDelayTime, tempUserData, 1, 0);
			
			// z-direction sequence
			tempDelayTime[0] = 200.0f;	tempDelayTime[1] = 0.0f;;
			SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS, 1000.0f, tempDelayTime, tempUserData, 1, 0);
			sequence = 0x02;
		}
		break;
	case 0x02:
		break;
	}
	
	return true;
}
/******************************************************************************/




/******************************************************************************/
void TestFunction(void)
{
	float tempDelayTime[2], tempUserData[5];

	tempDelayTime[0] = 400.0f;	tempDelayTime[1] = 60.0f;	tempUserData[0] = 0.0f;
	SetMoveTaskPos(&WalkingInfo[RIGHT][Z], WALK_READY_Z_POS+WALKING_Z_HEIGHT+pSharedMemory->Temp[14], 800.0f, tempDelayTime, tempUserData, 1, 0);
	SetMoveTaskPos(&WalkingInfo[LEFT][Z], WALK_READY_Z_POS+pSharedMemory->Temp[14], 800.0f, tempDelayTime, tempUserData, 1, 0);
}
/******************************************************************************/




/*****************************************************************************/
float Torsion_mass_spring_damper_Mx(float Mx, int zero)
{
	static float x1new,x2new, x1=0., x2=0.;
	static float filt;
	
	/*
	const float a[4] = {(float)0.92461509587868,  (float)-0.32069771753193,
						(float)0.00962093152596,   (float)0.99837557091103};
	
	const float b[2] = {(float)0.00962093152596, (float)0.00004873287267};
	const float c[2] = { (float)0.0,  (float)65.0};
	*/
	/*
	const float a[4] = {(float)0.97015145404280,  (float)-0.13134819764971,
						(float)0.00985111482373,   (float)0.99933994240940};
	
	const float b[2] = {(float)0.00985111482373,	(float)0.00004950431930};
	
	const float c[2] = {(float)0.0,  (float)14.81481481481481};
	*/

	const float a[4] = {(float)0.97765796618688,  (float)-0.07324804804199,
						(float)0.00988848648567,   (float)0.99963238059948};
	
	const float b[2] = {(float)0.00988848648567,	(float)0.00004962861907};
	
	const float c[2] = {(float)0.0,  (float)14.81481481481481};

	
	x1new = a[0]*x1 + a[1]*x2 + b[0]*Mx;
	x2new = a[2]*x1 + a[3]*x2 + b[1]*Mx;
	
	if(zero == 0) {x1new = 0.; x2new = 0.; filt = 0.;}
	
	//filt = c[0]*x1new + c[1]*x2new;

	Mx = (float)(Mx);
	//filt = (float)(filt + 0.1*Mx); 
	filt = (float)(filt + pSharedMemory->JW_temp[30]*Mx); //0.01

	if(filt > 30.0) filt = 30.0;
	else if(filt < -15.0) filt = -15.0;
	
	x1=x1new;
	x2=x2new;
	
	return filt;
}
/*****************************************************************************/




/*****************************************************************************/
float Torsion_mass_spring_damper_My(float My, int zero)
{
	static float x1new,x2new, x1=0., x2=0.;
	static float filt;
	static float a1;
	static float a1_old;
	
	const float a[4] = {(float)0.97015145404280,  (float)-0.13134819764971,
						(float)0.00985111482373,   (float)0.99933994240940};
	
	const float b[2] = {(float)0.00985111482373,	(float)0.00004950431930};
	
	const float c[2] = {(float)0.0,  (float)14.81481481481481};
	
	x1new = a[0]*x1 + a[1]*x2 + b[0]*My;
	x2new = a[2]*x1 + a[3]*x2 + b[1]*My;
	
	if(zero == 0) {x1new = 0.; x2new = 0.; filt = 0.;}
	
	//filt = c[0]*x1new + c[1]*x2new;

	//if((My <= 10.0) && (My >= -10.0)) My = (float)0.;

	a1 = (float)((1. - 2.*PI*0.1*DELTA_T)*a1_old + DELTA_T*My);
	a1_old = a1;

//	pSharedMemory->temp_force2 = a1;

	//if(My > 10.0) My = (float)(My - 10.0);
	//else if(My < -10.0) My = (float)(My + 10.0);
	//else My = (float)0.;
	
//	My = (float)(My - 8.8);
	My = (float)(My);


//	pSharedMemory->temp_force1 = My;

	//filt = (float)(filt + 0.05*My); 
	//filt = (float)(filt + 0.008*My); 
	//filt = (float)(filt + 0.0055*My); 
	//filt = (float)(filt + 0.006*My); 
	//filt = (float)(filt + 0.007*My); 
	filt = (float)(filt + pSharedMemory->JW_temp[31]*My);//0.0045 
	
	if(filt > 15.0) filt = 15.0;
	else if(filt < -30.0) filt = -30.0;
	
	x1=x1new;
	x2=x2new;
	
	return filt;
}
/*****************************************************************************/




/*****************************************************************************/
float Wrist_mass_spring_damper_Fz(float Fz, int zero)
{
	static float x1new,x2new, x1=0., x2=0.;
	static float filt;
	
	const float a[4] = {(float)0.94503939283972,  (float)-0.29175327263157,
						(float)0.00972510908772,   (float)0.99852749282218};
	
	const float b[2] = {(float)0.00972510908772,	(float)0.00004908357259};
	
	const float c[2] = {(float)0.0,    (float)10.0};
	
	
	x1new = a[0]*x1 + a[1]*x2 + b[0]*Fz;
	x2new = a[2]*x1 + a[3]*x2 + b[1]*Fz;
	
	if(zero == 0) {x1new = 0.; x2new = 0.;}
	
	filt = c[0]*x1new + c[1]*x2new ;
	
	if(filt > 30.0) filt = 30.0;
	else if(filt < -30.0) filt = -30.0;
	
	x1=x1new;
	x2=x2new;
	
	return filt;
}
/*****************************************************************************/




/*****************************************************************************/
bool SetMoveZMPInitAngle(unsigned char _jointID, float _angle, float _msTime)
{
	if(_msTime <= 0) { RtWprintf(L"\n>>> goal time must be grater than zero..!!"); return false; }
	if(_jointID >= NO_OF_JOINT) { RtWprintf(L"\n>>> Wrong joint ID(SetMoveZMPInitAngle)..!!"); return false; }
	
	Joint[_jointID].RefAngleFFToGo = _angle;
	Joint[_jointID].MoveFlagFF = false;
	Joint[_jointID].RefAngleFFInitial = Joint[_jointID].RefAngleFF;
	Joint[_jointID].CurrentTimeCountFF = 0;
	Joint[_jointID].GoalTimeCountFF = (unsigned long)(_msTime/INT_TIME);
	Joint[_jointID].RefAngleFFDelta = Joint[_jointID].RefAngleFFToGo - Joint[_jointID].RefAngleFF;
	Joint[_jointID].MoveFlagFF = true;

	return true;
}
/*****************************************************************************/




/*****************************************************************************/
bool SetMoveZMPInitPos(WALKING_INFO* _walkingInfo, float _pos, float _msTime)
{
	if(_msTime <= 0) { RtWprintf(L"\n>>> Goal time must be grater than zero(SetMoveZMPInitPos)..!!"); return false; }

	_walkingInfo->RefPosFFToGo = _pos;
	_walkingInfo->MoveFlagFF = false;
	_walkingInfo->RefPosFFInitial = _walkingInfo->RefPosFF;
	_walkingInfo->CurrentTimeCountFF = 0;
	_walkingInfo->GoalTimeCountFF = (unsigned long)(_msTime/INT_TIME);
	_walkingInfo->RefPosFFDelta = _walkingInfo->RefPosFFToGo - _walkingInfo->RefPosFF;
	_walkingInfo->MoveFlagFF = true;
	
	return true;
}
/*****************************************************************************/




/******************************************************************************/
void MoveZMPInitFF(JOINT _joint[], WALKING_INFO _walkingInfo[][4])
{
	unsigned char i, j;
	float tempTime;
	
	for(i=RHY ; i<=LAR ; i++)
	{	
		if(_joint[i].MoveFlagFF == true)
		{
			
				_joint[i].CurrentTimeCountFF++;
				if(_joint[i].GoalTimeCountFF <= _joint[i].CurrentTimeCountFF)
				{
					_joint[i].GoalTimeCountFF = _joint[i].CurrentTimeCountFF = 0;
					_joint[i].RefAngleFF = _joint[i].RefAngleFFToGo;
					_joint[i].MoveFlagFF = false;	
				}
				else
				{
					tempTime = (float)_joint[i].CurrentTimeCountFF/(float)_joint[i].GoalTimeCountFF;
					_joint[i].RefAngleFF = _joint[i].RefAngleFFInitial+_joint[i].RefAngleFFDelta*0.5f*(1.0f-cosf(PI*tempTime));
				}		
		}
	}


	// current walking position generator 
	for(i=RIGHT ; i<=LEFT ; i++)	// right and left
	{
		for(j=X ; j<=Yaw ; j++)	// X, Y, Z and Yaw
		{
			if(_walkingInfo[i][j].MoveFlagFF == true)
			{
				_walkingInfo[i][j].CurrentTimeCountFF++;
				if(_walkingInfo[i][j].GoalTimeCountFF <= _walkingInfo[i][j].CurrentTimeCountFF)
				{
					_walkingInfo[i][j].GoalTimeCountFF = _walkingInfo[i][j].CurrentTimeCountFF = 0;
					_walkingInfo[i][j].RefPosFF = _walkingInfo[i][j].RefPosFFToGo;
					_walkingInfo[i][j].MoveFlagFF = false;	
				}
				else
				{
					tempTime = (float)_walkingInfo[i][j].CurrentTimeCountFF/(float)_walkingInfo[i][j].GoalTimeCountFF;
					_walkingInfo[i][j].RefPosFF = _walkingInfo[i][j].RefPosFFInitial+_walkingInfo[i][j].RefPosFFDelta*0.5f*(1.0f-cosf(PI*tempTime));
				}
			}	
		}
	}
}
/******************************************************************************/



/******************************************************************************/	
int WholeBodyOffline_Demo(unsigned char canChannel)		// by Inhyeok
{
	int i_temp;
	pSharedMemory->WB_DemoRunFlag = 1;
	
	if(canChannel == CAN0) 
	{
		switch(pSharedMemory->WB_DemoNo)
		{
		case WB_OFFLINE_DEMO_DRC_TEST:			
			i_temp = WBIK_DRC_quad(_OfflineCount);
			break;
		case WB_FREQ_TEST:
			i_temp = WBIK_FreqTest(_OfflineCount);			
			break;
		case WB_STEERING_TEST:		
			i_temp = WBIK_DRC_steering_override(_OfflineCount);			
			break;
		case WB_LADDER_CLIMBING_TEST:
			i_temp = WBIK_DRC_ladder_climbing(_OfflineCount);
			break;
		case WB_QUAD_KIRK:
			if(_OfflineCount > 1 && pSharedMemory->transform_flag == 4)
			{
				if(pSharedMemory->init_kirk_flag == 1)
				{
					init_kirk();
					pSharedMemory->init_kirk_flag = 0;
				}
				
				if(pSharedMemory->kirk_flag == 1)
					Kirk_Posture_Init();
				else if(pSharedMemory->kirk_flag == 2)
					Go_Home_Posture();
				else if(pSharedMemory->kirk_flag == 3)
					Kirk_con_test();
				else if(pSharedMemory->kirk_flag == 4)
					Kirk_online();
				else
					;
			}
			i_temp = WBIK_DRC_Kirk_Quad(_OfflineCount);
			break;
		case WB_BIPED_KIRK:
			if(_OfflineCount > 1 && pSharedMemory->biped_flag == 1)
			{
				if(pSharedMemory->init_kirk_flag == 1)
				{
					init_kirk();
					pSharedMemory->init_kirk_flag = 0;
				}
				
				if(pSharedMemory->kirk_flag == 1)
					Kirk_Posture_Init_BP();
				else if(pSharedMemory->kirk_flag == 2)
					Go_Home_Posture();
				else if(pSharedMemory->kirk_flag == 3)
					Kirk_con_test_BP();
				else if(pSharedMemory->kirk_flag == 4)
					Kirk_online_BP();
				else if(pSharedMemory->kirk_flag == 5)
					Kirk_Load_Posture_Init_BP();
				else 
					;
			}
			i_temp = WBIK_DRC_Kirk_Biped(_OfflineCount);
			break;
		default:
			pSharedMemory->WB_DemoRunFlag = 0;
			return 1;
		}		

		switch(i_temp)
		{
		case 0:
			break;
		case -1:				
			pSharedMemory->CommandFlag = EMERGENCY_STOP;
			break;
		case -2:
			printf("\n Inverse kinematics error!!!");
			pSharedMemory->CommandFlag = EMERGENCY_STOP;
			break;
		case -3:
			printf("\n Infeasible inverse kinematics solution!!!");
			printf("\n %d", _OfflineCount);
			pSharedMemory->CommandFlag = EMERGENCY_STOP;
			break;
		case -4:
			printf("\n Forward kinematics error!!!");
			pSharedMemory->CommandFlag = EMERGENCY_STOP;
			break;
		default:
			pSharedMemory->CommandFlag = EMERGENCY_STOP;
			break;
		}
		
		MoveJMC(JMC0);
		MoveJMC(JMC1);
		MoveJMC(JMC2);
		MoveJMC(JMC3);
		MoveJMC(JMC4);
		MoveJMC(JMC5);
		MoveJMC(JMC6);
		MoveJMC(JMC7);
		MoveJMC(EJMC3);
		
		_OfflineCount++;
		
		if(_OfflineCount > 65530) _OfflineCount = 65530;
	}
	else if(canChannel == CAN1)
	{	
		switch(pSharedMemory->WB_DemoNo)
		{
		case WB_OFFLINE_DEMO_DRC_TEST:
			break;
		case WB_FREQ_TEST:
			break;
		case WB_STEERING_TEST:
			break;
		case WB_LADDER_CLIMBING_TEST:
			break;
		case WB_QUAD_KIRK:
			break;
		case WB_BIPED_KIRK:
			break;
		default:
			pSharedMemory->WB_DemoRunFlag = 0;
			return 1;
		}
		
		MoveJMC(JMC8);
		MoveJMC(JMC9);
		MoveJMC(JMC10);
		MoveJMC(JMC11);
		
		MoveJMC(EJMC0);	// Right wrist
		MoveJMC(EJMC1);	// Left wrist
		MoveJMC(EJMC2);	// Neck
		MoveJMC(EJMC4);	// Right fingers
		MoveJMC(EJMC5);	// Left fingers				
	}
	
	return 0;	
}
/******************************************************************************/


/******************************************************************************/
void face_change(unsigned char num)
{
		//Face CDI
	unsigned char tempData_face[4];
 
	//cdi face
	tempData_face[0] = 0x22;	// JMC34
	tempData_face[1] = 0xA0;	
	tempData_face[2] = num;
	tempData_face[3] = 0x00;
						
	PushCANMsg(Joint[NKY].CAN_channel, CMD_TXDF, tempData_face, 4, 0);
}
/*****************************************************************************/


/******************************************************************************/
void Back2InitAngle(void)		// by Inhyeok
{
	SetMoveJointAngle(RSP, Joint[RSP].WalkReadyAngle, 500.f, 0x01);
	SetMoveJointAngle(RSR, Joint[RSR].WalkReadyAngle, 500.f, 0x01);
	SetMoveJointAngle(RSY, Joint[RSY].WalkReadyAngle, 500.f, 0x01);
	SetMoveJointAngle(REB, Joint[REB].WalkReadyAngle, 500.f, 0x01);
	SetMoveJointAngle(RWY, Joint[RWY].WalkReadyAngle, 500.f, 0x01);
	SetMoveJointAngle(RWP, Joint[RWP].WalkReadyAngle, 500.f, 0x01);
	
	SetMoveJointAngle(LSP, Joint[LSP].WalkReadyAngle, 500.f, 0x01);
	SetMoveJointAngle(LSR, Joint[LSR].WalkReadyAngle, 500.f, 0x01);
	SetMoveJointAngle(LSY, Joint[LSY].WalkReadyAngle, 500.f, 0x01);
	SetMoveJointAngle(LEB, Joint[LEB].WalkReadyAngle, 500.f, 0x01);
	SetMoveJointAngle(LWY, Joint[LWY].WalkReadyAngle, 500.f, 0x01);
	SetMoveJointAngle(LWP, Joint[LWP].WalkReadyAngle, 500.f, 0x01);
	
	SetMoveJointAngle(NKY, 0.0f, 500.f, 0x01);
	SetMoveJointAngle(NK1, 0.0f, 500.f, 0x01);
	SetMoveJointAngle(NK2, 0.0f, 500.f, 0x01);
	SetMoveJointAngle(WST, 0.0f, 500.f, 0x01);
	
	SetMoveJointAngle(RHY, _Q0_34x1[9]*R2D, 500.f, 0x01);
	SetMoveJointAngle(RHR, _Q0_34x1[10]*R2D, 500.f, 0x01);
	SetMoveJointAngle(RHP, _Q0_34x1[11]*R2D, 500.f, 0x01);
	SetMoveJointAngle(RKN, _Q0_34x1[12]*R2D, 500.f, 0x01);
	SetMoveJointAngle(RAP, _Q0_34x1[13]*R2D, 500.f, 0x01);
	SetMoveJointAngle(RAR, _Q0_34x1[14]*R2D, 500.f, 0x01);

	SetMoveJointAngle(LHY, _Q0_34x1[15]*R2D, 500.f, 0x01);
	SetMoveJointAngle(LHR, _Q0_34x1[16]*R2D, 500.f, 0x01);
	SetMoveJointAngle(LHP, _Q0_34x1[17]*R2D, 500.f, 0x01);
	SetMoveJointAngle(LKN, _Q0_34x1[18]*R2D, 500.f, 0x01);
	SetMoveJointAngle(LAP, _Q0_34x1[19]*R2D, 500.f, 0x01);
	SetMoveJointAngle(LAR, _Q0_34x1[20]*R2D, 500.f, 0x01);

	pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
}
/******************************************************************************/



/*****************************************************************************/
int PushLogData(void)
{
	unsigned int head_new = (pSharedMemory->LogRing.uHead+1)%LOG_RING_SIZE ;
	char i;
	if(pSharedMemory->LogRing.bOverflow == TRUE)
		return 1;
	
	if(head_new == pSharedMemory->LogRing.uTail)
	{
		pSharedMemory->LogRing.bOverflow = TRUE;		
		printf("\n------The LogRing overflows. Logging is finished.------");
	}
	else
	{
		/*pSharedMemory->LogRing.data[head_new][0] = _Q_34x1[1];
		pSharedMemory->LogRing.data[head_new][1] = _Q_34x1[2];
		pSharedMemory->LogRing.data[head_new][2] = _Q_34x1[3];
		pSharedMemory->LogRing.data[head_new][3] = _Q_34x1[4];
		pSharedMemory->LogRing.data[head_new][4] = _Q_34x1[5];
		pSharedMemory->LogRing.data[head_new][5] = _Q_34x1[6];
		pSharedMemory->LogRing.data[head_new][6] = _Q_34x1[7];
		pSharedMemory->LogRing.data[head_new][7] = _Q_34x1[8];
		pSharedMemory->LogRing.data[head_new][8] = _Q_34x1[9];
		pSharedMemory->LogRing.data[head_new][9] = _Q_34x1[10];
		pSharedMemory->LogRing.data[head_new][10] = _Q_34x1[11];
		pSharedMemory->LogRing.data[head_new][11] = _Q_34x1[12];
		pSharedMemory->LogRing.data[head_new][12] = _Q_34x1[13];
		pSharedMemory->LogRing.data[head_new][13] = _Q_34x1[14];
		pSharedMemory->LogRing.data[head_new][14] = _Q_34x1[15];
		pSharedMemory->LogRing.data[head_new][15] = _Q_34x1[16];
		pSharedMemory->LogRing.data[head_new][16] = _Q_34x1[17];
		pSharedMemory->LogRing.data[head_new][17] = _Q_34x1[18];
		pSharedMemory->LogRing.data[head_new][18] = _Q_34x1[19];
		pSharedMemory->LogRing.data[head_new][19] = _Q_34x1[20];
		pSharedMemory->LogRing.data[head_new][20] = _Q_34x1[21];
		pSharedMemory->LogRing.data[head_new][21] = _Q_34x1[22];
		pSharedMemory->LogRing.data[head_new][22] = _Q_34x1[23];
		pSharedMemory->LogRing.data[head_new][23] = _Q_34x1[24];
		pSharedMemory->LogRing.data[head_new][24] = _Q_34x1[25];
		pSharedMemory->LogRing.data[head_new][25] = _Q_34x1[26];
		pSharedMemory->LogRing.data[head_new][26] = _Q_34x1[27];
		pSharedMemory->LogRing.data[head_new][27] = _Q_34x1[28];
		pSharedMemory->LogRing.data[head_new][28] = _Q_34x1[29];
		pSharedMemory->LogRing.data[head_new][29] = _Q_34x1[30];		
		pSharedMemory->LogRing.data[head_new][30] = _Q_34x1[31];
		pSharedMemory->LogRing.data[head_new][31] = _Q_34x1[32];
		pSharedMemory->LogRing.data[head_new][32] = _Q_34x1[33];
		pSharedMemory->LogRing.data[head_new][33] = _Q_34x1[34];
		pSharedMemory->LogRing.data[head_new][34] = _log_temp[0];
		pSharedMemory->LogRing.data[head_new][35] = _log_temp[1];
		pSharedMemory->LogRing.data[head_new][36] = _log_temp[2];
		pSharedMemory->LogRing.data[head_new][37] = _log_temp[3];
		pSharedMemory->LogRing.data[head_new][38] = _log_temp[4];
		pSharedMemory->LogRing.data[head_new][39] = _log_temp[5];
		pSharedMemory->LogRing.data[head_new][40] = _log_temp[6];
		pSharedMemory->LogRing.data[head_new][41] = _log_temp[7];
		pSharedMemory->LogRing.data[head_new][42] = _log_temp[8];
		pSharedMemory->LogRing.data[head_new][43] = _log_temp[9];
		pSharedMemory->LogRing.data[head_new][44] = _log_temp[10];
		pSharedMemory->LogRing.data[head_new][45] = _log_temp[11];
		pSharedMemory->LogRing.data[head_new][46] = _log_temp[12];
		pSharedMemory->LogRing.data[head_new][47] = _log_temp[13];
		pSharedMemory->LogRing.data[head_new][48] = _log_temp[14];
		pSharedMemory->LogRing.data[head_new][49] = _log_temp[15];
		pSharedMemory->LogRing.data[head_new][50] = _log_temp[16];
		pSharedMemory->LogRing.data[head_new][51] = _log_temp[17];
		pSharedMemory->LogRing.data[head_new][52] = _log_temp[18];
		pSharedMemory->LogRing.data[head_new][53] = _log_temp[19];
		pSharedMemory->LogRing.data[head_new][54] = _log_temp[20];
		pSharedMemory->LogRing.data[head_new][55] = _log_temp[21];	
		pSharedMemory->LogRing.data[head_new][56] = _log_temp[22];	
		pSharedMemory->LogRing.data[head_new][57] = _log_temp[23];	
		pSharedMemory->LogRing.data[head_new][58] = _log_temp[24];	
		pSharedMemory->LogRing.data[head_new][59] = _log_temp[25];	
		pSharedMemory->LogRing.data[head_new][60] = _log_temp[26];	
		pSharedMemory->LogRing.data[head_new][61] = _log_temp[27];
		pSharedMemory->LogRing.data[head_new][62] = _log_temp[28];
		pSharedMemory->LogRing.data[head_new][63] = _log_temp[29];
		pSharedMemory->LogRing.data[head_new][64] = _log_temp[30];
		pSharedMemory->LogRing.data[head_new][65] = _log_temp[31];
		pSharedMemory->LogRing.data[head_new][66] = _log_temp[32];*/

		for(i=0; i<LOG_DATA_SIZE; i++)
			pSharedMemory->LogRing.data[head_new][i] = _log_temp[i];
		
		pSharedMemory->LogRing.uHead = head_new;
	}

	return 0;

}
/*******************************************************************************/


/******************************************************************************/
void SetDRCQuadrupedReadyPos(void)		// by Inhyeok
{
	SetMoveJointAngle(RSP, pSharedMemory->ref_rsp, 4000.f, 0x01);
	SetMoveJointAngle(RSR, pSharedMemory->ref_rsr-OFFSET_RSR, 4000.f, 0x01);
	SetMoveJointAngle(RSY, pSharedMemory->ref_rsy, 4000.f, 0x01);
	SetMoveJointAngle(REB, pSharedMemory->ref_reb-OFFSET_REB, 4000.f, 0x01);
	SetMoveJointAngle(RWY, pSharedMemory->ref_rwy, 4000.f, 0x01);
	SetMoveJointAngle(RWP, pSharedMemory->ref_rwp, 4000.f, 0x01);
	SetMoveJointAngle(RWY2, pSharedMemory->ref_rwy2, 4000.f, 0x01);
	
	SetMoveJointAngle(LSP, pSharedMemory->ref_lsp, 4000.f, 0x01);
	SetMoveJointAngle(LSR, pSharedMemory->ref_lsr-OFFSET_LSR, 4000.f, 0x01);
	SetMoveJointAngle(LSY, pSharedMemory->ref_lsy, 4000.f, 0x01);
	SetMoveJointAngle(LEB, pSharedMemory->ref_leb-OFFSET_LEB, 4000.f, 0x01);
	SetMoveJointAngle(LWY, pSharedMemory->ref_lwy, 4000.f, 0x01);
	SetMoveJointAngle(LWP, pSharedMemory->ref_lwp, 4000.f, 0x01);
	SetMoveJointAngle(LWY2, pSharedMemory->ref_lwy2, 4000.f, 0x01);
	
	SetMoveJointAngle(RHY, pSharedMemory->ref_rhy, 4000.f, 0x01);
	SetMoveJointAngle(RHR, pSharedMemory->ref_rhr, 4000.f, 0x01);
	SetMoveJointAngle(RHP, pSharedMemory->ref_rhp, 4000.f, 0x01);
	SetMoveJointAngle(RKN, pSharedMemory->ref_rkn, 4000.f, 0x01);
	SetMoveJointAngle(RAP, pSharedMemory->ref_rap, 4000.f, 0x01);
	SetMoveJointAngle(RAR, pSharedMemory->ref_rar, 4000.f, 0x01);

	SetMoveJointAngle(LHY, pSharedMemory->ref_lhy, 4000.f, 0x01);
	SetMoveJointAngle(LHR, pSharedMemory->ref_lhr, 4000.f, 0x01);
	SetMoveJointAngle(LHP, pSharedMemory->ref_lhp, 4000.f, 0x01);
	SetMoveJointAngle(LKN, pSharedMemory->ref_lkn, 4000.f, 0x01);
	SetMoveJointAngle(LAP, pSharedMemory->ref_lap, 4000.f, 0x01);
	SetMoveJointAngle(LAR, pSharedMemory->ref_lar, 4000.f, 0x01);

	SetMoveJointAngle(WST, DRC_BR_WST, 4000.f, 0x01);
}
/******************************************************************************/


/******************************************************************************/
void SetDRCBipedReadyPos(void)		// by Inhyeok
{
	SetMoveJointAngle(RSP, DRC_BR_RSP, 3000.f, 0x01);
	SetMoveJointAngle(RSR, DRC_BR_RSR-OFFSET_RSR, 3000.f, 0x01);
	SetMoveJointAngle(RSY, DRC_BR_RSY, 3000.f, 0x01);
	SetMoveJointAngle(REB, DRC_BR_REB-OFFSET_REB, 3000.f, 0x01);
	SetMoveJointAngle(RWY, DRC_BR_RWY, 3000.f, 0x01);
	
	SetMoveJointAngle(LSP, DRC_BR_LSP, 3000.f, 0x01);
	SetMoveJointAngle(LSR, DRC_BR_LSR-OFFSET_LSR, 3000.f, 0x01);
	SetMoveJointAngle(LSY, DRC_BR_LSY, 3000.f, 0x01);
	SetMoveJointAngle(LEB, DRC_BR_LEB-OFFSET_LEB, 3000.f, 0x01);
	SetMoveJointAngle(LWY, DRC_BR_LWY, 3000.f, 0x01);
	
	SetMoveJointAngle(RHY, DRC_BR_RHY, 3000.f, 0x01);
	SetMoveJointAngle(RHR, DRC_BR_RHR, 3000.f, 0x01);
	SetMoveJointAngle(RHP, DRC_BR_RHP, 3000.f, 0x01);
	SetMoveJointAngle(RKN, DRC_BR_RKN, 3000.f, 0x01);
	SetMoveJointAngle(RAP, DRC_BR_RAP, 3000.f, 0x01);
	SetMoveJointAngle(RAR, DRC_BR_RAR, 3000.f, 0x01);

	SetMoveJointAngle(LHY, DRC_BR_LHY, 3000.f, 0x01);
	SetMoveJointAngle(LHR, DRC_BR_LHR, 3000.f, 0x01);
	SetMoveJointAngle(LHP, DRC_BR_LHP, 3000.f, 0x01);
	SetMoveJointAngle(LKN, DRC_BR_LKN, 3000.f, 0x01);
	SetMoveJointAngle(LAP, DRC_BR_LAP, 3000.f, 0x01);
	SetMoveJointAngle(LAR, DRC_BR_LAR, 3000.f, 0x01);

	SetMoveJointAngle(WST, DRC_BR_WST, 3000.f, 0x01);

	if(pSharedMemory->change_drc_hand_flag != DRC_STICK_MODE)
	{
		SetMoveJointAngle(RWP, DRC_BR_RWP, 3000.f, 0x01);
		SetMoveJointAngle(LWP, DRC_BR_LWP, 3000.f, 0x01);

		SetMoveJointAngle(RWY2, DRC_BR_RWY2, 3000.f, 0x01);
		SetMoveJointAngle(LWY2, DRC_BR_LWY2, 3000.f, 0x01);
	}
}
/******************************************************************************/


/******************************************************************************/
void SetDRCWideBipedReadyPos(void)		// by Inhyeok
{
	SetMoveJointAngle(RSP, DRC_BR_RSP, 3000.f, 0x01);
	SetMoveJointAngle(RSR, DRC_BR_RSR-OFFSET_RSR, 3000.f, 0x01);
	SetMoveJointAngle(RSY, DRC_BR_RSY, 3000.f, 0x01);
	SetMoveJointAngle(REB, DRC_BR_REB-OFFSET_REB, 3000.f, 0x01);
	SetMoveJointAngle(RWY, DRC_BR_RWY, 3000.f, 0x01);
	
	SetMoveJointAngle(LSP, DRC_BR_LSP, 3000.f, 0x01);
	SetMoveJointAngle(LSR, DRC_BR_LSR-OFFSET_LSR, 3000.f, 0x01);
	SetMoveJointAngle(LSY, DRC_BR_LSY, 3000.f, 0x01);
	SetMoveJointAngle(LEB, DRC_BR_LEB-OFFSET_LEB, 3000.f, 0x01);
	SetMoveJointAngle(LWY, DRC_BR_LWY, 3000.f, 0x01);
	
	SetMoveJointAngle(RHY, DRC_BR_RHY, 3000.f, 0x01);
	SetMoveJointAngle(RHR, DRC_BR_RHR_WIDE, 3000.f, 0x01);
	SetMoveJointAngle(RHP, DRC_BR_RHP, 3000.f, 0x01);
	SetMoveJointAngle(RKN, DRC_BR_RKN, 3000.f, 0x01);
	SetMoveJointAngle(RAP, DRC_BR_RAP, 3000.f, 0x01);
	SetMoveJointAngle(RAR, DRC_BR_RAR_WIDE, 3000.f, 0x01);

	SetMoveJointAngle(LHY, DRC_BR_LHY, 3000.f, 0x01);
	SetMoveJointAngle(LHR, DRC_BR_LHR_WIDE, 3000.f, 0x01);
	SetMoveJointAngle(LHP, DRC_BR_LHP, 3000.f, 0x01);
	SetMoveJointAngle(LKN, DRC_BR_LKN, 3000.f, 0x01);
	SetMoveJointAngle(LAP, DRC_BR_LAP, 3000.f, 0x01);
	SetMoveJointAngle(LAR, DRC_BR_LAR_WIDE, 3000.f, 0x01);

	SetMoveJointAngle(WST, DRC_BR_WST, 3000.f, 0x01);

	if(pSharedMemory->change_drc_hand_flag != DRC_STICK_MODE)
	{
		SetMoveJointAngle(RWP, DRC_BR_RWP, 3000.f, 0x01);
		SetMoveJointAngle(LWP, DRC_BR_LWP, 3000.f, 0x01);

		SetMoveJointAngle(RWY2, DRC_BR_RWY2, 3000.f, 0x01);
		SetMoveJointAngle(LWY2, DRC_BR_LWY2, 3000.f, 0x01);
	}
}
/******************************************************************************/



/******************************************************************************/
void SetDRCSteeringReadyPos(void)		// by Inhyeok
{
	SetMoveJointAngle(RSP, pSharedMemory->ref_rsp, 4000.f, 0x01);
	SetMoveJointAngle(RSR, pSharedMemory->ref_rsr-OFFSET_RSR, 4000.f, 0x01);
	SetMoveJointAngle(RSY, pSharedMemory->ref_rsy, 4000.f, 0x01);
	SetMoveJointAngle(REB, pSharedMemory->ref_reb-OFFSET_REB, 4000.f, 0x01);
	SetMoveJointAngle(RWY, pSharedMemory->ref_rwy, 4000.f, 0x01);
	SetMoveJointAngle(RWP, pSharedMemory->ref_rwp, 4000.f, 0x01);
	SetMoveJointAngle(RWY2, pSharedMemory->ref_rwy2, 4000.f, 0x01);

	
	SetMoveJointAngle(LSP, pSharedMemory->ref_lsp, 4000.f, 0x01);
	SetMoveJointAngle(LSR, pSharedMemory->ref_lsr-OFFSET_LSR, 4000.f, 0x01);
	SetMoveJointAngle(LSY, pSharedMemory->ref_lsy, 4000.f, 0x01);
	SetMoveJointAngle(LEB, pSharedMemory->ref_leb-OFFSET_LEB, 4000.f, 0x01);
	SetMoveJointAngle(LWY, pSharedMemory->ref_lwy, 4000.f, 0x01);
	SetMoveJointAngle(LWP, pSharedMemory->ref_lwp, 4000.f, 0x01);
	SetMoveJointAngle(LWY2, pSharedMemory->ref_lwy2, 4000.f, 0x01);
	
	SetMoveJointAngle(RHY, pSharedMemory->ref_rhy, 4000.f, 0x01);
	SetMoveJointAngle(RHR, pSharedMemory->ref_rhr, 4000.f, 0x01);
	SetMoveJointAngle(RHP, pSharedMemory->ref_rhp, 4000.f, 0x01);
	SetMoveJointAngle(RKN, pSharedMemory->ref_rkn, 4000.f, 0x01);
	SetMoveJointAngle(RAP, pSharedMemory->ref_rap, 4000.f, 0x01);
	SetMoveJointAngle(RAR, pSharedMemory->ref_rar, 4000.f, 0x01);

	SetMoveJointAngle(LHY, pSharedMemory->ref_lhy, 4000.f, 0x01);
	SetMoveJointAngle(LHR, pSharedMemory->ref_lhr, 4000.f, 0x01);
	SetMoveJointAngle(LHP, pSharedMemory->ref_lhp, 4000.f, 0x01);
	SetMoveJointAngle(LKN, pSharedMemory->ref_lkn, 4000.f, 0x01);
	SetMoveJointAngle(LAP, pSharedMemory->ref_lap, 4000.f, 0x01);
	SetMoveJointAngle(LAR, pSharedMemory->ref_lar, 4000.f, 0x01);

	SetMoveJointAngle(WST, DRC_BR_WST, 4000.f, 0x01);
}
/******************************************************************************/



/******************************************************************************/
void SetDRCLadderClimbingReadyPos(void)		// by Inhyeok
{
	
	SetMoveJointAngle(RSP, pSharedMemory->ref_rsp, 3000.f, 0x01);
	SetMoveJointAngle(RSR, pSharedMemory->ref_rsr-OFFSET_RSR, 3000.f, 0x01);
	SetMoveJointAngle(RSY, pSharedMemory->ref_rsy, 3000.f, 0x01);
	SetMoveJointAngle(REB, pSharedMemory->ref_reb-OFFSET_REB, 3000.f, 0x01);
	SetMoveJointAngle(RWY, pSharedMemory->ref_rwy, 3000.f, 0x01);
	SetMoveJointAngle(RWP, pSharedMemory->ref_rwp, 3000.f, 0x01);
	SetMoveJointAngle(RWY2, pSharedMemory->ref_rwy2, 3000.f, 0x01);

	
	SetMoveJointAngle(LSP, pSharedMemory->ref_lsp, 3000.f, 0x01);
	SetMoveJointAngle(LSR, pSharedMemory->ref_lsr-OFFSET_LSR, 3000.f, 0x01);
	SetMoveJointAngle(LSY, pSharedMemory->ref_lsy, 3000.f, 0x01);
	SetMoveJointAngle(LEB, pSharedMemory->ref_leb-OFFSET_LEB, 3000.f, 0x01);
	SetMoveJointAngle(LWY, pSharedMemory->ref_lwy, 3000.f, 0x01);
	SetMoveJointAngle(LWP, pSharedMemory->ref_lwp, 3000.f, 0x01);
	SetMoveJointAngle(LWY2, pSharedMemory->ref_lwy2, 3000.f, 0x01);
	
	SetMoveJointAngle(RHY, pSharedMemory->ref_rhy, 3000.f, 0x01);
	SetMoveJointAngle(RHR, pSharedMemory->ref_rhr, 3000.f, 0x01);
	SetMoveJointAngle(RHP, pSharedMemory->ref_rhp, 3000.f, 0x01);
	SetMoveJointAngle(RKN, pSharedMemory->ref_rkn, 3000.f, 0x01);
	SetMoveJointAngle(RAP, pSharedMemory->ref_rap, 3000.f, 0x01);
	SetMoveJointAngle(RAR, pSharedMemory->ref_rar, 3000.f, 0x01);

	SetMoveJointAngle(LHY, pSharedMemory->ref_lhy, 3000.f, 0x01);
	SetMoveJointAngle(LHR, pSharedMemory->ref_lhr, 3000.f, 0x01);
	SetMoveJointAngle(LHP, pSharedMemory->ref_lhp, 3000.f, 0x01);
	SetMoveJointAngle(LKN, pSharedMemory->ref_lkn, 3000.f, 0x01);
	SetMoveJointAngle(LAP, pSharedMemory->ref_lap, 3000.f, 0x01);
	SetMoveJointAngle(LAR, pSharedMemory->ref_lar, 3000.f, 0x01);

	SetMoveJointAngle(WST, DRC_BR_WST, 3000.f, 0x01);
}
/******************************************************************************/



unsigned short NeckDynamixelPos(float angle)
{
	float angle_ = angle + 180.f;

	if(angle_ > 360.f)
		angle_ = 360.f;
	else if(angle_ <0.f)
		angle_ = 0.f;

	return ((unsigned short)(4095.f*angle_/360.f));
}


void NeckDynamixelSetSpeed(float percent_NKY, float percent_NK1, float percent_NK2)  // max speed = 100%, no-change = 0
{
	unsigned char tempData[8];
	unsigned short short_temp;

	static float percent_NKY_ = 10;
	static float percent_NK1_ = 10;
	static float percent_NK2_ = 10;

	if(percent_NKY != 0.f)
		percent_NKY_ = (float)fabs(percent_NKY);

	if(percent_NK1 != 0.f)
		percent_NK1_ = (float)fabs(percent_NK1);

	if(percent_NK2 != 0.f)
		percent_NK2_ = (float)fabs(percent_NK2);

	tempData[0] = EJMC2;
	tempData[1] = 0xA0;

	short_temp = (unsigned short)(percent_NKY_*1023.f/100.f);
	if(short_temp > 1023)
		short_temp = 0; //max speed
	else if(short_temp == 0)
		short_temp = 1;
	tempData[2] = (unsigned char)(short_temp & 0x00FF);
	tempData[3] = (unsigned char)((short_temp>>8) & 0x00FF);

	short_temp = (unsigned short)(percent_NK1_*1023.f/100.f);
	if(short_temp > 1023)
		short_temp = 0; //max speed
	else if(short_temp == 0)
		short_temp = 1;
	tempData[4] = (unsigned char)(short_temp & 0x00FF);
	tempData[5] = (unsigned char)((short_temp>>8) & 0x00FF);

	short_temp = (unsigned short)(percent_NK2_*1023.f/100.f);
	if(short_temp > 1023)
		short_temp = 0; //max speed
	else if(short_temp == 0)
		short_temp = 1;
	tempData[6] = (unsigned char)(short_temp & 0x00FF);
	tempData[7] = (unsigned char)((short_temp>>8) & 0x00FF);

	PushCANMsg(Joint[NKY].CAN_channel, CMD_TXDF, tempData, 8, 0);
}



void NeckDynamixelHome(void)
{
	
	NeckDynamixelSetSpeed(50.f, 50.f, 50.f);

	Joint[NKY].RefAngleCurrent = 0.f;
	Joint[NK1].RefAngleCurrent = 0.f;
	Joint[NK2].RefAngleCurrent = 0.f;

	MoveJMC(EJMC2);
}


