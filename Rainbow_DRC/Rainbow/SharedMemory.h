#ifndef SHAREDMEMORY_H
#define SHAREDMEMORY_H

#include "CommonDefinition.h"
#include "CAN.h"

typedef struct _SHARED_DATA_
{	
// win32 -> RTX ---------------------------------------------------------------------------------//
	// CAN Message를 주고 받을때 사용되는 변수
	// 보내고자 하는 데이터를 아래의 변수에 할당 한 후 사용.
	// for Channel 0
	unsigned short		Tx_ID0;			// CAN ID.
	unsigned char		Tx_Data0[8];	// CAN data
	unsigned char		Tx_DLC0;		// data length code
	// for channel 1
	unsigned short		Tx_ID1;			// CAN ID.
	unsigned char		Tx_Data1[8];	// CAN data
	unsigned char		Tx_DLC1;		// data length code

	// for win32 command flag
	unsigned char		CommandFlag;			// command flag
	unsigned int		CommandData;			// command data variable used when the command flag needs
//	unsigned int		CommandDataInt;			// int command data variable used when the command flag needs
	float				CommandDataFloat[4];		// float command data variable used when the command flag needs
	unsigned int		CommandDataArray[4];	// array command data variable used when the command flag needs


	// getting board parameter
	unsigned int		WantedBoard;		
	unsigned int		ReturnedID;
	unsigned char		ReturnedData[8];

	// for joint control command
	//_CONTROL_MODE		MotorControlMode;	// control mode
	unsigned char		MotorControlMode;	// control mode
	float				GoalAngle;			// goal joint angle(deg)
	float				GoalTime;			// goal moving time(ms)
	unsigned char		JointID;			// control JointID
	unsigned char		BoardID;			// control BoardID
	JOINT				Joint;				// structure for each joint 

	// for sensor control command
	//_SENSOR_MODE		SensorControlMode;
	unsigned char		SensorControlMode;
	unsigned char		FTsensorID;
	unsigned char		IMUsensorID;
	FT					FTsensor;
	IMU					IMUsensor;
//-----------------------------------------------------------------------------------------------//


// CDI ADD --------------------------------------------------------------------------------------//
	bool				COM_Ready_Flag;
	int					Voice_Recog_Command;
	int					RBT_PosX;
	int					RBT_PosY;
//	float				RBT_PosZ;	
	int					RBT_Parsing_Cnt;
	bool				RBT_Ready_Flag;
	bool				RBT_Start_Flag;
	float				RBT_Yaw;
	float				RBT_Yaw_old;
	float				RBT_Yaw_lpf;
	float				RBT_Pitch;
	float				RBT_Pitch_old;
	float				RBT_Pitch_lpf;
	float				RBT_Yaw_Kp;
	float				RBT_Yaw_Kd;
	float				RBT_Pitch_Kp;
	float				RBT_Pitch_Kd;
	float				RBT_RefX;
	float				RBT_RefY;
	float				RBT_DesX;
	float				RBT_DesY;
	float				RBT_DelX;
	float				RBT_DelY;
	float				RBT_DelX_old;
	float				RBT_DelY_old;
	float				RBT_DelX_dot;
	float				RBT_DelY_dot;
	int					RBT_mode_Flag;
//-----------------------------------------------------------------------------------------------//


// RTX -> win32 ---------------------------------------------------------------------------------//
	_COMMAND_FLAG_STATUS CommandFlagStatus;
//-----------------------------------------------------------------------------------------------//


// win32 <-> RTX --------------------------------------------------------------------------------//
	 
//-----------------------------------------------------------------------------------------------//


// for debugging ---------------------------------------------------------------------------------//
	JOINT			Joint_debug[NO_OF_JOINT];
	FT				FT_debug[NO_OF_FT];
	IMU				IMU_debug[NO_OF_IMU];
	//float			Data_Debug[40][8000];
	float			Data_Debug[40][10];
	unsigned int	Data_Debug_Index;
	
	//char			CAN_Read_Debug[1000][9];
	char			CAN_Read_Debug[1][9];
	unsigned int	CAN_Read_Debug_Index;

	float			ZMP[6];
	float			Gain[6];
	float			Sway, ankleAdd;
	float			tempPeriod, tempDelayY, tempDSP, tempStep;
	float			TempRoll[3];

	float			Temp[15];
	unsigned char	TempChar;
	bool			tempBool;

	float			JW_temp[40];
	bool			JW_MotionSet[NoOfMotion];
	LOGRING			LogRing;		// by Inhyeok
//-----------------------------------------------------------------------------------------------//

//HandShake--------------------------------------------------------------------------------------//
	int				ShakeHandsFlag;
	int				PushUpFlag;

	int				WalkReadyPosFlag;

//WB, Inhyeok------------------------------------------------------------------------------------//
	float			WB_ZMP_OFFSET;
	char			WB_StartFlag;
	char			WB_DemoNo;
	char			WB_Down2DSP_flag;
	char			WB_DemoRunFlag;
	char			WB_DampCtrlOnFlag;
	char			WB_UpdownFlag;
	int				WB_SceneNo;
	char			WB_imu_feedback_on;
	float			WB_FreqTest_Amp;
	float			WB_FreqTest_Omega;
	float			WB_FreqTest_COMx0;
	float			WB_FreqTest_StartFlag;
	float			WB_const_temp;
	float			WB_const_temp2;
	float			WB_const_temp3;
	float			WB_const_temp4;
	float			temp_Wn;
	float			temp_b;
	float			temp_m;
	float			WB_fRS[3], WB_fLS[3];
	char			sw_mode; //SW_MODE_COMPLEMENTARY, SW_MODE_NOT_COMPLEMENTARY
	float			temp_EL_compen, temp_fHand_threshold;
	float			temp_ssp_damp_on,temp_dsp_damp_on_flag;
	char			temp_foot_LC_on_flag, temp_hand_LC_on_flag;
	float			disp_pCOM[2], disp_pRH[3], disp_qRH[4], disp_pLH[3], disp_qLH[4], disp_pRF[3], disp_qRF[4], disp_pLF[3], disp_qLF[4], disp_Q_34x1[35];
	float			ref_lsp, ref_lsr, ref_lsy, ref_leb, ref_lwy, ref_lwp, ref_lwy2;
	float			ref_rsp, ref_rsr, ref_rsy, ref_reb, ref_rwy, ref_rwp, ref_rwy2;
	float			ref_rhy, ref_rhr, ref_rhp, ref_rkn, ref_rap, ref_rar;
	float			ref_lhy, ref_lhr, ref_lhp, ref_lkn, ref_lap, ref_lar;
	char			steer_flag, steer_demo, ladder_demo;
	float			steer_time[N_STEER], steer_ang[N_STEER];
	float			disp_ct_33x1[34],disp_grav_33x1[34],disp_duty_33x1[34];
	float			kp[30],kd[30];
	float			ref_fLH_3x1[4], ref_fRH_3x1[4];
	char			position_mode_flag, torque_mode_flag, torque_mode_flag_lb;
	float			move_sec, drvRF[4], drvLF[4], dpRF[3], dpLF[3];
	float			drvRH[4], drvLH[4], dpRH[3], dpLH[3], dpCOM[3], dpPELz, drvPEL[4];	
	float			off_traj_dpPELz[MAX_OFF_TRAJ], off_traj_dpCOM[MAX_OFF_TRAJ][2], off_traj_dpRH[MAX_OFF_TRAJ][3], off_traj_dpLH[MAX_OFF_TRAJ][3], off_traj_dpRF[MAX_OFF_TRAJ][3], off_traj_dpLF[MAX_OFF_TRAJ][3], off_traj_drvRH[MAX_OFF_TRAJ][4], off_traj_drvLH[MAX_OFF_TRAJ][4], off_traj_drvRF[MAX_OFF_TRAJ][4], off_traj_drvLF[MAX_OFF_TRAJ][4];
	unsigned int	off_traj_length, off_traj_count;
	float			angle_NKY, angle_NK1, angle_NK2;
	float			spd_NKY, spd_NK1, spd_NK2;
	char			scan_flag;
	float			scan_range, scan_freq;
	char			transform_flag, change_drc_hand_flag, biped_flag;
	float			wb_data[15];
	float			gain_ovr_test, gain_ovr_test2;
	char			stop_online_quad_flag, stop_online_biped_flag, init_kirk_flag;
	char			quad_flag;

	// Kirk //
    unsigned char First_flag;
	unsigned char Go_flag;
	unsigned char Change_OK;
	double Global_Ls;
	double Global_Lss;
	double Global_Rot_Ang;
	double Global_Hs;
	double Global_Delay_Ratio;
	double Global_Ab;
	unsigned int Step_Number;
	unsigned char P4_ZMP_CON_Flag;
	int P4_ZMP_CON_CNT;
	unsigned char kirk_flag;
	// Kirk //

	float Del_RAP_Init;
	float Del_RAR_Init;
	float Del_LAR_Init;
	float Del_RLHP_Init;	
	float BC_X_Init;
	float BC_Y_Init;
	float Del_RLFZ_Init;

	float xzmp_offset;
	char ON_EL_flag, ON_ZMP_flag, ON_vib_flag, ON_upright_flag, ON_compl_ankle_flag, ON_compl_arm_flag;
	char stop_flag, gain_ovr_flag, friction_flag, gravity_flag;
	
//-----------------------------------------------------------------------------------------------//
	char			WalkingDemo;
	float			LandingFz;
//Rock Paper Scissor
	int				RPS_mode;
	char			RPS_POS_MODE;
	char			HUG_POS_MODE;

/* Send CAN flags */
 	int HUBO_FLAG_SEND_CAN;

} SHARED_DATA, *PSHARED_DATA;

#endif
