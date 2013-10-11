#include <windows.h>
#include <stdio.h>
//#include <stdlib.h>
#include <math.h>
#include "CommonDefinition.h"
#include "SharedMemory.h"
#include "APIs.h"
#include "Kirk.h"
#include "WBIK_Functions.h"

#pragma warning(once:4244)

extern float _pCOM_3x1[4];

// kirk's local variables //
int Ts = 0, Ts_BP = 0, Td = 0, Td_BP = 0, Step_CNT = 0, Old_Step_CNT = 0, shape_factor_F = 0, shape_factor_S = 0, shape_factor_F_BP = 0, shape_factor_S_BP = 0, i = 0;
double	Delay_Ratio = 0., Hs = 0., Ls = 0., Lss = 0., Old_Lss = 0., Old_Ls = 0., Ab = 0., Rot_Ang = 0., Old_Rot_Ang = 0.;
unsigned long Index_CNT = 0;
unsigned char Change_flag = 0, Walking_stage[6] = {0,0,0,0,0,0};
double  Ref_BC[3] = {0,0,0}, Ref_R1[3] = {0,0,0}, Ref_R1p[3] = {0,0,0}, Ref_R2[3] = {0,0,0}, Ref_R2p[3] = {0,0,0}, Ref_R3[3] = {0,0,0}, Ref_R3p[3] = {0,0,0},
        Ref_L1[3] = {0,0,0}, Ref_L1p[3] = {0,0,0}, Ref_L2[3] = {0,0,0}, Ref_L2p[3] = {0,0,0}, Ref_L3[3] = {0,0,0}, Ref_L3p[3] = {0,0,0},
        Ref_th[7] = {0,0,0,0,0,0,0}, Old_Ref_th[7] = {0,0,0,0,0,0,0};
double	 Ref_ZMP[2] = {0.,0.};
double  test_var[10] = {0,0,0,0,0,0,0,0,0,0};
    
double	del_x_R1 = 0., del_y_R1 = 0., del_x_L1 = 0., del_y_L1 = 0.,del_x_R2 = 0., del_y_R2 = 0., del_x_L2 = 0., del_y_L2 = 0.;
double	Rot_th_R1 = 0., Rot_th_L1 = 0., Rot_th_R2 = 0., Rot_th_L2 = 0.;
unsigned char Foot_Location_Flag = 1, WP_Flow_Control_Flag = 0;    // ?? 2013.04.30  
double  temp_Z[6] = {0.,0.,0.,0.,0.,0.}, All_Foot_Z = 0, Old_All_Foot_Z = 0.;   
double  temp_X[6] = {0.,0.,0.,0.,0.,0.}, temp_Y[6] = {0.,0.,0.,0.,0.,0.}, temp_th[6] = {0.,0.,0.,0.,0.,0.};
unsigned char All_Foot_Z_Flag = 0;
unsigned char Old_Walking_stage[6] = {0,0,0,0,0,0};
double  LL_Del_z[6] = {0,0,0,0,0,0}, Old_LL_Del_z[6] = {0,0,0,0,0,0};
unsigned char Early_Landing_Flag[6] = {0,0,0,0,0,0}, Late_Landing_Flag[6] = {0,0,0,0,0,0};
unsigned int  LL_Del_z_CNT[6] = {0,0,0,0,0,0}, EL_Del_xy_CNT[6] = {0,0,0,0,0,0}, All_Foot_Z_CNT = 0;
double  Fz[6] = {0,0,0,0,0,0}; 
double  X_ZMP = 0., Y_ZMP = 0.;  
double ftsRF_Fz_LP = 0., ftsLF_Fz_LP = 0., Old_ftsRF_Fz_LP = 0., Old_ftsLF_Fz_LP = 0.;
double ftsRF_Mx_LP = 0., ftsLF_Mx_LP = 0., Old_ftsRF_Mx_LP = 0., Old_ftsLF_Mx_LP = 0.;
double ftsRF_My_LP = 0., ftsLF_My_LP = 0., Old_ftsRF_My_LP = 0., Old_ftsLF_My_LP = 0.;
double ftsRW_Fz_LP = 0., ftsLW_Fz_LP = 0., Old_ftsRW_Fz_LP = 0., Old_ftsLW_Fz_LP = 0.;
double temp_Z_avg = 0.;
unsigned char First_Flag_1 = 0;
double Old_Ref_R1[3] = {0.,0.,0.}, Old_Ref_R2[3] = {0.,0.,0.}, Old_Ref_R3[3] = {0.,0.,0.}, Old_Ref_L1[3] = {0.,0.,0.}, Old_Ref_L2[3] = {0.,0.,0.}, Old_Ref_L3[3] = {0.,0.,0.};
double Del_LEG_Z_VSC[6] = {0,0,0,0,0,0}, Old_Del_LEG_Z_VSC[6] = {0,0,0,0,0,0};
unsigned char CNT1_VSC_LEG[6] ={0,0,0,0,0,0}, CNT2_VSC_LEG[6] ={0,0,0,0,0,0}, OneKG_OK_Flag[6] ={0,0,0,0,0,0}, FiveKG_OK_Flag[6] ={0,0,0,0,0,0}, CNT_LAR_TORQ_CON = 0, CNT_RAR_TORQ_CON = 0;
double FG_VSC_LEG = 0.;
double Del_LAR_TORQ_CON = 0., Del_LAP_TORQ_CON = 0., Old_Del_LAR_TORQ_CON = 0.,  Old_Del_LAP_TORQ_CON = 0.,
       Del_RAR_TORQ_CON = 0., Del_RAP_TORQ_CON = 0., Old_Del_RAR_TORQ_CON = 0.,  Old_Del_RAP_TORQ_CON = 0.;
double Radius_R1 = 0., Radius_L1 = 0., Radius_R2 = 0., Radius_L2 = 0.;
double Init_R1_th = 0., Init_L1_th = 0., Init_R2_th = 0., Init_L2_th = 0. ;
double Old_Del_XZMP_CON = 0., Del_XZMP_CON = 0., Old_Del_YZMP_CON = 0., Del_YZMP_CON = 0.;
double BC_X_Init = 0., BC_Y_Init = 0., Del_R1_Z_PosInit = 0., Del_R2_Z_PosInit = 0., Del_RAR_Init = 0., Del_LAR_Init = 0., Del_RAP_Init = 0., Del_RLHP_Init = 0., Del_RLFZ_Init = 0.; 
double Del_BC_X_Home = 0., Del_BC_Y_Home = 0., Del_R1_Z_Home = 0., Del_R2_Z_Home = 0., Del_L1_Z_Home = 0., Del_L2_Z_Home = 0., Del_RAR_Home = 0., Del_LAR_Home = 0., Del_RAP_Home = 0., Del_LAP_Home = 0.,
       Del_RLHP_Home = 0., Del_RHR_Home = 0., Del_LHR_Home = 0., Del_RHP_Home = 0., Del_LHP_Home = 0.;
int P4_ZMP_CON_CNT = 0, P4_ZMP_CON_CNT_2 = 0, Local_CNT_1 = 0, Local_CNT_2 = 0;
double BC_X_CON = 0., BC_Y_CON = 0.;    // update
unsigned char LandROK_Flag = 0, LandLOK_Flag = 0, ThreeKG_ROK_Flag = 0, ThreeKG_LOK_Flag = 0, OneKG_ROK_Flag = 0, OneKG_LOK_Flag = 0;
unsigned char CNT_final_gain_DSP_ZMP_CON = 0, CNT_DSP_XYZMP_CON = 0, CNT_final_gain_SSP_ZMP_CON = 0;
double Del_PC_X_DSP_XZMP_CON = 0., Del_PC_Y_DSP_YZMP_CON = 0., final_gain_DSP_ZMP_CON = 0., Old_Del_PC_X_DSP_XZMP_CON = 0., Old_Del_PC_Y_DSP_YZMP_CON = 0.,
       Del_PC_X_SSP_XZMP_CON = 0., Del_PC_Y_SSP_YZMP_CON = 0., final_gain_SSP_ZMP_CON = 0., Old_Del_PC_X_SSP_XZMP_CON = 0., Old_Del_PC_Y_SSP_YZMP_CON = 0.;
double CON_SUM_X_ZMP_ERR = 0., CON_SUM_Y_ZMP_ERR = 0., Old_CON_SUM_X_ZMP_ERR = 0., Old_CON_SUM_Y_ZMP_ERR = 0., SUM_X_ZMP_ERR = 0., SUM_Y_ZMP_ERR = 0., SUM_RMS_Y_ZMP = 0., RMS_Y_ZMP = 0.;
int    CNT_SUM_ZMP_ERR = 0, CNT_CON_SUM_XY_ZMP_ERR = 0, CNT_RLAR_TORQ_CON = 0;
unsigned char Real_Stop_Flag = 0, CNT_final_gain_SSP_Y_VIB_CON_1 = 0, CNT_final_gain_SSP_Y_VIB_CON_2 = 0, CNT_FOOT_VIB_CON = 0, CNT_SSP_ZMP_CON = 0;
double Del_LAR_RLAR_TORQ_CON = 0., Del_RAR_RLAR_TORQ_CON = 0., Del_LAP_RLAP_TORQ_CON = 0., Del_RAP_RLAP_TORQ_CON = 0., Old_Del_LAR_RLAR_TORQ_CON = 0., Old_Del_RAR_RLAR_TORQ_CON = 0.,
       Old_Del_LAP_RLAP_TORQ_CON = 0., Old_Del_RAP_RLAP_TORQ_CON = 0.;
double final_gain_SSP_Y_VIB_CON_1 = 0.0, final_gain_SSP_Y_VIB_CON_2 = 0.0, final_gain_SSP_Y_VIB_CON_3 = 0.0, final_gain_SSP_Y_VIB_CON_4 = 0.0, Del_RHR_VIB_CON = 0., Del_LHR_VIB_CON = 0.,  Del_RHY_VIB_CON = 0., Del_LHY_VIB_CON = 0.,
	Old_Del_RHR_VIB_CON = 0., Old_Del_LHR_VIB_CON = 0., Del_RHP_VIB_CON = 0., Del_LHP_VIB_CON = 0., Old_Del_RHP_VIB_CON = 0., Old_Del_LHP_VIB_CON = 0.,
	Old_Del_RHY_VIB_CON = 0., Old_Del_LHY_VIB_CON = 0.;
double Del_RLHP_Posture_Walking = 0., Del_RLFZ_Posture_Walking = 0., Del_RLAR_Posture_Walking = 0., adj_amp = 0., Ref_CON_SUM_X_ZMP_ERR = 0., Ref_CON_SUM_Y_ZMP_ERR = 0.;
double Old_Del_PC_X_SSP_XZMP_CON_2 = 0., Old_Del_PC_Y_SSP_YZMP_CON_2 = 0., Old_RF_Y_acc_HPF_INT = 0., RF_Y_acc_HPF_INT = 0., Old_LF_Y_acc_HPF_INT = 0., LF_Y_acc_HPF_INT = 0.,
		Old_RF_X_acc_HPF_INT = 0., RF_X_acc_HPF_INT = 0., Old_LF_X_acc_HPF_INT = 0., LF_X_acc_HPF_INT = 0.;
double  BC_Y_OFFSET = 0.;
double  Old_Avg_RF_Pitch_Ang = 0., Avg_RF_Pitch_Ang = 0.,  Old_Avg_LF_Pitch_Ang = 0., Avg_LF_Pitch_Ang = 0.;
int		Avg_CNT = 1; 
double RHR_Compen_Ang = 0., LHR_Compen_Ang = 0.;  // 2013_08_02
double Body_Y_th = 0., Body_Y_th_d = 0.;
double RF_Del_Y_Outside = 0., Old_RF_Del_Y_Outside = 0., LF_Del_Y_Outside = 0., Old_LF_Del_Y_Outside = 0.;
unsigned char Temp_CNT = 0, Temp_CNT_2 = 0, Outside_Land_Flag = 0;

// kirk's local variables //



// external variables //
extern ONLINE_PATTERN _online_pattern;
extern float _pRF0_3x1[4], _pLF0_3x1[4], _qRF0_4x1[5], _qLF0_4x1[5];
extern float _pRH0_3x1[4], _pLH0_3x1[4], _qRH0_4x1[5], _qLH0_4x1[5];
extern PSHARED_DATA	pSharedMemory;
extern FT FTSensor[NO_OF_FT];
extern float _log_temp[LOG_DATA_SIZE];
extern IMU	IMUSensor[NO_OF_IMU];
// external variables //

void init_kirk(void)
{
	float center_3x1[4];
	// Kirk Init //
	// ?? ???? ?? //
		  Ts = 200;   // [1 sec] Swing Time
		  Ts_BP = 140;//150;   // [0.8 sec] Swing Time
		  Delay_Ratio = 0.1;   
		  Td = (int)(Ts*Delay_Ratio); // [0.1 sec] Delay Time (??, ????)
		  Td_BP = (int)(Ts_BP*Delay_Ratio); // [0.1 sec] Delay Time (??, ????)
		  Hs = 100.0; // [mm] Step Height
		  Ls = 0.0; // [mm] Step Length
		  Ab = 40.0; // [mm] Body Lateral Swing Amplitude (?? ??? ??)
		  Lss = 0; // [mm] Side Step Length
		  Rot_Ang = 0. ; // [deg]Rotation Angle 
		  Old_Lss = Lss;
		  Old_Ls = Ls;
		  Old_Rot_Ang = Rot_Ang;
		  shape_factor_F = -3;
		  shape_factor_S = -3;
		  shape_factor_F_BP = 0;
		  shape_factor_S_BP = -3;
		  pSharedMemory->Step_Number = 1 ;  // ?? ?? ?? //
		  pSharedMemory->Go_flag = 0;
		  pSharedMemory->First_flag = 0;
		  pSharedMemory->Change_OK = 1;

		  center_3x1[1] = (_pRF0_3x1[1] + _pLF0_3x1[1] + _pRH0_3x1[1] + _pLH0_3x1[1])/4.f;
		  center_3x1[2] = (_pRF0_3x1[2] + _pLF0_3x1[2] + _pRH0_3x1[2] + _pLH0_3x1[2])/4.f;
		  center_3x1[3] = (_pRF0_3x1[3] + _pLF0_3x1[3] + _pRH0_3x1[3] + _pLH0_3x1[3])/4.f;

		  Radius_R1 = sqrtf((_pRF0_3x1[1]-center_3x1[1])*(_pRF0_3x1[1]-center_3x1[1])+(_pRF0_3x1[2]-center_3x1[2])*(_pRF0_3x1[2]-center_3x1[2]));
		  Radius_L1 = sqrtf((_pLF0_3x1[1]-center_3x1[1])*(_pLF0_3x1[1]-center_3x1[1])+(_pLF0_3x1[2]-center_3x1[2])*(_pLF0_3x1[2]-center_3x1[2]));
		  Radius_R2 = sqrtf((_pRH0_3x1[1]-center_3x1[1])*(_pRH0_3x1[1]-center_3x1[1])+(_pRH0_3x1[2]-center_3x1[2])*(_pRH0_3x1[2]-center_3x1[2]));
		  Radius_L2 = sqrtf((_pLH0_3x1[1]-center_3x1[1])*(_pLH0_3x1[1]-center_3x1[1])+(_pLH0_3x1[2]-center_3x1[2])*(_pLH0_3x1[2]-center_3x1[2]));

		  Init_R1_th = R2D*atan2(_pRF0_3x1[1]-center_3x1[1], -(_pRF0_3x1[2]-center_3x1[2]));
		  Init_L1_th = R2D*atan2(_pLF0_3x1[1]-center_3x1[1], -(_pLF0_3x1[2]-center_3x1[2]));
		  Init_R2_th = R2D*atan2(_pRH0_3x1[1]-center_3x1[1], -(_pRH0_3x1[2]-center_3x1[2]));
		  Init_L2_th = R2D*atan2(_pLH0_3x1[1]-center_3x1[1], -(_pLH0_3x1[2]-center_3x1[2]));
		  
		  //Radius_R1 = sqrt(0.5*(_pRF0_3x1[1] -_pRH0_3x1[1])*0.5*(_pRF0_3x1[1]-_pRH0_3x1[1]) + _pRF0_3x1[2]*_pRF0_3x1[2]);
		  //Radius_L1 = sqrt(0.5*(_pLF0_3x1[1] -_pLH0_3x1[1])*0.5*(_pLF0_3x1[1]-_pLH0_3x1[1]) + _pLF0_3x1[2]*_pLF0_3x1[2]);
		  //Radius_R2 = sqrt(0.5*(_pRF0_3x1[1] -_pRH0_3x1[1])*0.5*(_pRF0_3x1[1]-_pRH0_3x1[1]) + _pRH0_3x1[2]*_pRH0_3x1[2]);
		  //Radius_L2 = sqrt(0.5*(_pLF0_3x1[1] -_pLH0_3x1[1])*0.5*(_pLF0_3x1[1]-_pLH0_3x1[1]) + _pLH0_3x1[2]*_pLH0_3x1[2]);
			  
			
	      //Init_R1_th = R2D*atan2(0.5*(_pRF0_3x1[1] -_pRH0_3x1[1]), -_pRF0_3x1[2]);
		  //Init_L1_th = R2D*atan2(0.5*(_pLF0_3x1[1] -_pLH0_3x1[1]), -_pLF0_3x1[2]);
		  //Init_R2_th = R2D*atan2(-0.5*(_pRF0_3x1[1] -_pRH0_3x1[1]), -_pRH0_3x1[2]);
		  //Init_L2_th = R2D*atan2(-0.5*(_pLF0_3x1[1] -_pLH0_3x1[1]), -_pLH0_3x1[2]);		
		  
			P4_XZMP_INT_CON_E(0,0,0,0);
			P4_YZMP_INT_CON_E(0,0,0,0);
			P2_XZMP_INT_CON_E(0,0,0,0);
			P2_YZMP_INT_CON_E(0,0,0,0);
			P4_XZMP_CON_E(0,0,0);
			P4_YZMP_CON_E(0,0,0);
			P3H_XZMP_CON_E(0,0,0);
			P3H_YZMP_CON_E(0,0,0);
			P3F_XZMP_CON_E(0,0,0);
			P3F_YZMP_CON_E(0,0,0);
			P2_XZMP_CON_E(0,0,0); 
			P2_YZMP_CON_E(0,0,0);
			P1_XZMP_CON_E(0,0,0); 
			P1_YZMP_CON_E(0,0,0);

			FORCE_DIFF_CON_R1(0,0,0);
			FORCE_DIFF_CON_R2(0,0,0);
			TORQ_RAR_INT_CON(0,0,0);
			TORQ_LAR_INT_CON(0,0,0);
			TORQ_DIFF_RAP_INT_CON(0,0,0);
			ZMP_CON_MANAGER(0,0,0,0,0,0,0); 
			ZMP_CON_MANAGER_BP(0,0,0,0,0);
			TP_INT_CON(0,0,0);
			TR_INT_CON(0,0,0);


	_online_pattern.dangRFz = 0.f;
    _online_pattern.dangLFz = 0.f;
	_online_pattern.dangRHz = 0.f;
	_online_pattern.dangLHz = 0.f;

	_online_pattern.dpCOMx = 0.f;
	_online_pattern.dpCOMy = 0.f;

	_online_pattern.offset_RAP = 0.f;
	_online_pattern.offset_RAR = 0.f;

	_online_pattern.offset_LAP = 0.f;
	_online_pattern.offset_LAR = 0.f;

	_online_pattern.offset_RHP = 0.f;
	_online_pattern.offset_LHP = 0.f;
	_online_pattern.offset_RHR = 0.f;
	_online_pattern.offset_LHR = 0.f;

	_online_pattern.offset_RHY = 0.f;
	_online_pattern.offset_LHY = 0.f;

    _online_pattern.dpRFz = 0.f;
    _online_pattern.dpLFz = 0.f;
    _online_pattern.dpRFy = 0.f;
    _online_pattern.dpLFy = 0.f;
    _online_pattern.dpRFx = 0.f;
    _online_pattern.dpLFx = 0.f;
    
    _online_pattern.dpRHz = 0.f;
    _online_pattern.dpLHz = 0.f;
    _online_pattern.dpRHy = 0.f;
    _online_pattern.dpLHy = 0.f;
    _online_pattern.dpRHx = 0.f;
    _online_pattern.dpLHx = 0.f;

	// Kirk Init //
}

void prof_delay_cos_delay(int *profState, 
			double *profResult, 
			int timeIndex, 
			int timeStart, 
			int timeDuring, 
			int timeStop, 
			int timeDelay_0, 
			int timeDelay_1)
{

	int halfCycle;
	int timeLocal;
	
	//Compare the arguments to calculate the time duration
	if(timeStop == 0){
		timeStop = timeStart+timeDuring;
	}else if(timeDuring == 0){
		timeDuring = timeStop-timeStart;
	}

	//Local time setting
	timeLocal = timeIndex - timeStart;

	halfCycle = (timeDuring-timeDelay_0-timeDelay_1);
	
	//Profile generation
	if(timeIndex < timeStart + timeDelay_0){
		*profResult = 0;
	}else if((timeIndex >= timeStart+timeDelay_0)&&(timeIndex <= (timeStop-timeDelay_1))){
		*profResult = (float)((1-cos((3.1416)*(timeLocal-timeDelay_0)/halfCycle))/2);
	}else{
		*profResult = 1;
	}

	//
	if(timeLocal < 0){
		*profState=0;
	}else if((timeLocal >=0) && (timeLocal<timeDuring)){
		if(timeLocal == 0){
			*profState = 1;
		} else {
			*profState = 2;
		}
	}else{
		*profState = 3;
	}
}

void prof_cos_linear(int *profState,
		double *profResult, 
		int timeIndex, 
		int timeStart, 
		int timeDuring,
		int timeStop,
		int shapeFactor)						 
{
	//Basic variables declaration
	int timeLocal;
	int halfCycle;

	//Compare the arguments to calculate the time duration
	if(timeStop == 0){
		timeStop = timeStart+timeDuring;
	}else if(timeDuring == 0){
		timeDuring = timeStop-timeStart;
	}

	//Local time setting
	timeLocal = timeIndex - timeStart;

	if(shapeFactor<-10){
		shapeFactor = -10;
	}
	
	//Shape factor reflection
	if(shapeFactor >=0){
		halfCycle = (timeDuring - 2*shapeFactor) + 0;
	}else{
		halfCycle = timeDuring + 0;
	}

	//Profile generation
	if(shapeFactor >=0){
		if(timeIndex < timeStart + shapeFactor){
			*profResult = 0;
		}else if((timeIndex >= timeStart+shapeFactor)&&(timeIndex<=(timeStop-shapeFactor))){
			*profResult = (1-(float)(cos(3.141592*(timeLocal-shapeFactor+0)/halfCycle)))/2;
		}else if((timeIndex > timeStop - shapeFactor)&&(timeIndex<=timeStop)){
			*profResult = 1;
		}
		else
			*profResult = 1;
	}else{
		if(timeIndex < timeStart){
			*profResult = 0;
		}else if((timeIndex >= timeStart)&&(timeIndex <= timeStop)){
			*profResult = ((10+(float)(shapeFactor))/10*(1-(float)(cos(3.141592*((float)(timeLocal)+0)/(float)(halfCycle))))/2) + ((-1*(float)(shapeFactor))/10*((float)(timeLocal)+0)/(float)(halfCycle));
		}else if((timeIndex > timeStop)&&(timeIndex <= timeStop)){
			
			*profResult = 1;
		}
		else *profResult = 1;
	}

	if(timeLocal < 0){
		*profState = 0;
	}else if((timeLocal >= 0)&&(timeLocal < timeDuring)){
		if(timeLocal == 0){
			*profState = 1;		//Just started status
		}else{
			*profState = 2;		//Running Status
		}
	}else{
		*profState = 3;
	}
}

void half_cycloneDn(int *profState, 
		double *profResult, 
		int timeIndex, 
		int timeStart, 
		int timeDuring)
{
	float timeLocal;

	//Local time setting
	timeLocal = (float)(timeIndex - timeStart)/timeDuring;

	//Profile generation
	if(timeIndex < timeStart){
		*profResult = 0;
		*profState = 0 ;
	}else if((timeIndex >= timeStart)&&(timeIndex < (timeStart+timeDuring))){
		*profResult = (float)(0.5*(timeLocal + (float)(sin(3.1416*timeLocal))/3.1416));
		*profState = 1;
	}else if(timeIndex == (timeStart+timeDuring))
	{
		*profResult = 0.5;
		*profState = 2;
	}
	else{ 
		*profResult = 0.5;
		*profState = 3;
	}
}

// Function which return the last part of the cyclone function
void half_cycloneUp(int *profState,
		double *profResult, 
		int timeIndex, 
		int timeStart, 
		int timeDuring)
{
	float timeLocal;

	//Local time setting
	timeLocal = (float)(timeIndex - timeStart)/timeDuring;

	//Profile generation
	if(timeIndex < timeStart){
		*profResult = 0;
		*profState = 0 ;
	}else if((timeIndex >= timeStart)&&(timeIndex < (timeStart+timeDuring))){
		*profResult = (float)(0.5*(timeLocal - (float)(sin(3.1416*timeLocal))/3.1416));
		*profState = 1;
	}else if(timeIndex == (timeStart+timeDuring))
	{
		*profResult = 0.5;
		*profState = 2;
	}
	else{ 
		*profResult = 0.5;
		*profState = 3;
	}

}

// Function which return entire part of the cyclone function
void prof_cyclone(int *profStateUp,
		int *profStateDn,
		float *profResult, 
		int timeIndex, 
		int timeStart, 
		int timeDuring)
{
	int profState_1, profState_2;
	double profResultUp, profResultDn;

	half_cycloneUp(&profState_1, &profResultUp, timeIndex, timeStart, timeDuring/2);	
	half_cycloneDn(&profState_2, &profResultDn, timeIndex, timeDuring/2+timeStart, timeDuring/2);

	*profResult = (float)(profResultUp + profResultDn);
    *profStateUp = profState_1;
	*profStateDn = profState_2;
}

void Biped_walking_pattern(unsigned char Go_flag, 
							int Ts, 
							int Td, 
							int shape_factor_F,
							int shape_factor_S,
							double Ab,
							double Hs, 
							double Ls, 
							double Lss,
							double Rot_Ang,
							unsigned char Early_Landing_Flag[6],
							int *Step_CNT, 
							double Ref_BC[3], 
							double Ref_R1[3], 
							double Ref_R2[3], 
							double Ref_R3[3], 
							double Ref_L1[3], 
							double Ref_L2[3], 
							double Ref_L3[3], 
							double Ref_th[7],
							double Ref_ZMP[2],
							unsigned char Walking_stage[6], 
							unsigned char *Change_flag, 
							double test_var[10])
{
	static unsigned char /*Real_Stop_Flag = 0,*/ RF_stop_flag = 0, LF_stop_flag = 0, LF_start_flag = 0, BC_SWAY_Stop_Flag = 0;
	static unsigned long local_Index = 0, Temp_local_Index = 0;
	static unsigned long next_start_time_R1 = (Ts/2+Td), next_start_time_R2 = 0, next_start_time_R3 = 0,next_start_time_L1 = (Ts/2+Td), next_start_time_L2 = 0, next_start_time_L3 = 0, 
						 next_start_time_BC = (Ts/2+Td), next_start_time_BC_2 = (Ts/2+Td), next_start_time_BC_3 = (Ts/2+Td), next_start_time_BC_th = (Ts/2+Td);
	static double	Temp_Ref_R1[3] = {0,0,0}, Temp2_Ref_R1[3] = {0,0,0}, Temp_Ref_L2[3] = {0,0,0}, Temp2_Ref_L2[3] = {0,0,0}, Temp_Ref_R3[3] = {0,0,0}, Temp2_Ref_R3[3] = {0,0,0},
					Temp_Ref_L1[3] = {0,0,0}, Temp2_Ref_L1[3] = {0,0,0}, Temp_Ref_R2[3] = {0,0,0}, Temp2_Ref_R2[3] = {0,0,0}, Temp_Ref_L3[3] = {0,0,0}, Temp2_Ref_L3[3] = {0,0,0},
					Temp_Ref_th[6] = {0,0,0,0,0,0}, Temp2_Ref_th[6] = {0,0,0,0,0,0};
	static double	Old_Hs_RF = 0., Old_Hs_LF = 0., Old_Ls_RF = 0., Old_Lss_RF = 0., Old_Ls_LF = 0., Old_Lss_LF = 0, Old_Ls_BC = 0., Old_Lss_BC = 0., Old_Td_RF = 0., 
					Old_Td_LF = 0.,Old_Td_BC = 0., Old_Td_BC_2 = 0.,Old_Td_BC_3 = 0., Temp_Ls_RF = 0., Temp_Lss_RF = 0., Temp_Ls_LF = 0., Temp_Lss_LF = 0., Old_BC_X = 0., Old_BC_Y = 0., 
					Old_Del_BC_Y = 0., Del_BC_Y = 0., Old_Rot_Ang_RF = 0., Old_Rot_Ang_LF = 0., Old_Rot_Ang_BC = 0., Old_BC_th = 0.,Old_Del_ZMP_Y = 0., Del_ZMP_Y = 0., Old_Ab = 0., Old_Ab_2 = 0; // 2013_06_23
	static int		T_off = 0, Old_T_off_BC = 0, Old_T_off_BC_2 = 0, BC_SWAY_CNT = 1;   
	static int		Old_profState_R1_Z_up = 0, Old_profState_L1_Z_up = 0, Temp_profState_BC_Y = 0, Temp_profState_BC_th = 0, Old_profState_R1_Z_dn = 0 ;
	static int		profState_R1_Z_up = 0, profState_R2_Z_up = 0, profState_R3_Z_up = 0, profState_L1_Z_up = 0, profState_L2_Z_up = 0, profState_L3_Z_up = 0,
					profState_R1_Z_dn = 0, profState_R2_Z_dn = 0, profState_R3_Z_dn = 0, profState_L1_Z_dn = 0, profState_L2_Z_dn = 0, profState_L3_Z_dn = 0;
	static int		profState_R1_X_up = 0, profState_R2_X_up = 0, profState_R3_X_up = 0, profState_L1_X_up = 0, profState_L2_X_up = 0, profState_L3_X_up = 0,
					profState_R1_X_dn = 0, profState_R2_X_dn = 0, profState_R3_X_dn = 0, profState_L1_X_dn = 0, profState_L2_X_dn = 0, profState_L3_X_dn = 0;
	static int		profState_BC_X = 0, profState_BC_Y = 0,profState_BC_Y_SWAY = 0, profState_BC_th = 0, profState_ZMP_Y = 0, profState_ZMP_X = 0;
	static double	profResult_R1_Z_up = 0, profResult_R2_Z_up = 0, profResult_R3_Z_up = 0, profResult_L1_Z_up = 0, profResult_L2_Z_up = 0, profResult_L3_Z_up = 0,
					profResult_R1_Z_dn = 0, profResult_R2_Z_dn = 0, profResult_R3_Z_dn = 0, profResult_L1_Z_dn = 0, profResult_L2_Z_dn = 0, profResult_L3_Z_dn = 0;
	static double	profResult_R1_X_up = 0, profResult_R2_X_up = 0, profResult_R3_X_up = 0, profResult_L1_X_up = 0, profResult_L2_X_up = 0, profResult_L3_X_up = 0,
					profResult_R1_X_dn = 0, profResult_R2_X_dn = 0, profResult_R3_X_dn = 0, profResult_L1_X_dn = 0, profResult_L2_X_dn = 0, profResult_L3_X_dn = 0;
	static double	profResult_BC_X = 0., profResult_BC_Y = 0., profResult_BC_Y_SWAY = 0.,profResult_BC_th = 0.,profResult_ZMP_Y = 0.,profResult_ZMP_X = 0.;
	int		i = 0;
	static double	SW_Temp_R1[3] = {0,0,0}, SW_Temp_L2[3] = {0,0,0}, SW_Temp_R3[3] = {0,0,0}, SW_Temp_th = 0.;
	int				SW_Temp_Walking_stage = 0;
	


//	printf("%d	%d	%d	%d	%d	%d \n",Early_Landing_Flag[0],Early_Landing_Flag[1],Early_Landing_Flag[2],Early_Landing_Flag[3],Early_Landing_Flag[4],Early_Landing_Flag[5]); // 추가 2012.09.17

	if(local_Index == 0 && Go_flag == 1){ // 최초 파라미터 설정  

		if(Lss > 0 || Rot_Ang > 0) {
			LF_start_flag = 1; // 왼발 Start 인 경우, 왼발 시작 플래그를 Enable 시킨다. +방향 회전은 +방향 측보행과 같이 이루어진다. +방향 회전과 -방향 측보행은 같이 이뤄질 수 없다.
			Lss = -Lss; 
			Rot_Ang = -Rot_Ang;
		}
		else LF_start_flag = 0;


		Old_Td_RF = Td;
		Old_Td_LF = Td;
		Old_Td_BC = Td; 
		Old_Td_BC_2 = Td;
		Old_Td_BC_3 = Td;
		Old_Ls_RF = Ls;
		Old_Lss_RF = Lss;
		Old_Ls_LF = Ls;
		Old_Lss_LF = Lss;
		Old_Ls_BC = Ls;
		Old_Lss_BC = Lss; 	
		Temp_Ls_RF = Old_Ls_RF;
		Temp_Ls_LF = Old_Ls_LF;
		Old_Del_BC_Y = 0.; Del_BC_Y = 0.;
		Old_Del_ZMP_Y = 0.; Del_ZMP_Y = 0.;
		Old_Rot_Ang_RF = Rot_Ang;
		Old_Rot_Ang_LF = Rot_Ang;
		Old_Rot_Ang_BC = Rot_Ang;
		Old_Ab = Ab;  //2013_06_23
		Old_Ab_2 = Old_Ab;  //2013_06_23
		

		BC_SWAY_CNT = 1;


		if(2*Ts+2*Td < Ts)  T_off = Ts - (2*Ts+2*Td); // 최소 3점 지지를 유지하기 위한 시간 딜레이 설정
		else T_off = 0;

		Old_T_off_BC = T_off;
		Old_T_off_BC_2 = T_off;
			
	}

	if(local_Index == 1)  Foot_Location_Flag = 1;  // 추가 2013.04.30


	if(LF_start_flag && local_Index != 0){  // 왼발 Start 인 경우, 오른발과 왼발 궤적을 서로 바꿔준것을 다시 원상복귀시킨다.

		for(i = 0; i<3; i++) {
			SW_Temp_R1[i] = Ref_R1[i];
			Ref_R1[i] = Ref_L1[i];
			Ref_L1[i] = SW_Temp_R1[i];

			//SW_Temp_L2[i] = Ref_L2[i];
			//Ref_L2[i] = Ref_R2[i];
			//Ref_R2[i] = SW_Temp_L2[i];

			//_1SW_Temp_R3[i] = Ref_R3[i];
			//_1Ref_R3[i] = Ref_L3[i];
			//_1Ref_L3[i] = SW_Temp_R3[i];

			SW_Temp_th = Ref_th[i];
			Ref_th[i] = Ref_th[i+3];
			Ref_th[i+3] = SW_Temp_th;
		}

		Ref_R1[1] = -Ref_R1[1];
		//Ref_R2[1] = -Ref_R2[1];
		//_1Ref_R3[1] = -Ref_R3[1];
		Ref_L1[1] = -Ref_L1[1];
		//Ref_L2[1] = -Ref_L2[1];
		//_1Ref_L3[1] = -Ref_L3[1];

		Ref_BC[1] = -Ref_BC[1];
		Ref_ZMP[1] = -Ref_ZMP[1];

		for(i=0; i<7; i++)	Ref_th[i] = -Ref_th[i];

		Lss = -1.*Lss; // 왼발 Start 인 경우, 즉 Lss > 0 인경우 다시 Lss < 0 으로 만들어 준 후에 최종적으로 오른다리와 왼다리를 바꾼다.
		Rot_Ang = -Rot_Ang;

		
		SW_Temp_Walking_stage = Walking_stage[0];
		Walking_stage[0] = Walking_stage[3];
		Walking_stage[3] = SW_Temp_Walking_stage;

		//SW_Temp_Walking_stage = Walking_stage[1];
		//Walking_stage[1] = Walking_stage[4];
		//Walking_stage[4] = SW_Temp_Walking_stage;

		//SW_Temp_Walking_stage = Walking_stage[2];
		//Walking_stage[2] = Walking_stage[5];
		//Walking_stage[5] = SW_Temp_Walking_stage;
	}



	// 각 발의 Z방향(수직방향) 궤적 생성(0 ~ 1) //
	
	// RF 으로 통칭 //
	if(LF_stop_flag != 2 || profState_R1_Z_dn == 2){
		prof_delay_cos_delay(&profState_R1_Z_up,&profResult_R1_Z_up,local_Index,next_start_time_R1 + 0,Ts/2,0,0,Ts/10); 
		prof_delay_cos_delay(&profState_R1_Z_dn,&profResult_R1_Z_dn,local_Index,next_start_time_R1 + Ts/2,Ts/2,0,Ts/10,0); 
		//prof_delay_cos_delay(&profState_L2_Z_up,&profResult_L2_Z_up,local_Index,next_start_time_L2 + Ts+Old_Td_RF,Ts/2,0,0,0); 
		//prof_delay_cos_delay(&profState_L2_Z_dn,&profResult_L2_Z_dn,local_Index,next_start_time_L2 + Ts+Old_Td_RF+Ts/2,Ts/2,0,0,0); 
		//_1 prof_delay_cos_delay(&profState_R3_Z_up,&profResult_R3_Z_up,local_Index,next_start_time_R3 + 2*Ts+2*Old_Td_RF,Ts/2,0,0,0); 
		//_1prof_delay_cos_delay(&profState_R3_Z_dn,&profResult_R3_Z_dn,local_Index,next_start_time_R3 + 2*Ts+2*Old_Td_RF+Ts/2,Ts/2,0,0,0); 
	}

	// LF 으로 통칭 //
	if(RF_stop_flag != 2 || profState_L1_Z_dn == 2){
		prof_delay_cos_delay(&profState_L1_Z_up,&profResult_L1_Z_up,local_Index,next_start_time_L1 + Ts+Old_Td_LF + T_off,Ts/2,0,0,Ts/10); 
		prof_delay_cos_delay(&profState_L1_Z_dn,&profResult_L1_Z_dn,local_Index,next_start_time_L1 + Ts+Old_Td_LF + T_off + Ts/2,Ts/2,0,Ts/10,0); 
		//prof_delay_cos_delay(&profState_R2_Z_up,&profResult_R2_Z_up,local_Index,next_start_time_R2 + 3*Ts+3*Old_Td_LF + T_off,Ts/2,0,0,0); 
		//prof_delay_cos_delay(&profState_R2_Z_dn,&profResult_R2_Z_dn,local_Index,next_start_time_R2 + 3*Ts+3*Old_Td_LF + T_off +Ts/2,Ts/2,0,0,0); 
		//_1prof_delay_cos_delay(&profState_L3_Z_up,&profResult_L3_Z_up,local_Index,next_start_time_L3 + 5*Ts+5*Old_Td_LF + T_off,Ts/2,0,0,0); 
		//_1prof_delay_cos_delay(&profState_L3_Z_dn,&profResult_L3_Z_dn,local_Index,next_start_time_L3 + 5*Ts+5*Old_Td_LF + T_off + Ts/2,Ts/2,0,0,0); 
	}

	//printf("Td = %d \n",Td);
	
	// 한주기 끝나는 순간 CNT 증가 //
	if((Old_profState_L1_Z_up == 0 && profState_L1_Z_up !=0) || (Old_profState_R1_Z_up == 0 && profState_R1_Z_up !=0)) {
		*Step_CNT = *Step_CNT + 1;   // 첫 번째 다리 들때 CNT 증가 

		//printf("Step_CNT= %d \n",*Step_CNT);
	}

	Old_profState_R1_Z_up = profState_R1_Z_up;
	Old_profState_L1_Z_up = profState_L1_Z_up;


	// 스텝카운터에 따라 실제 Z 방향 진폭 적용 //  
	if(profState_R1_Z_up == 1) {
		Old_Hs_RF = Hs;
		Old_Td_RF = Td;
	}
	else if(profState_R1_Z_dn == 3) {
		Old_Hs_LF = Hs;
	}


	// 수정 2012.09.17
	if(LF_start_flag){
	
		if (profState_R1_Z_dn == 1 ) temp_Z[3] = 0.; 

		//if (profState_L2_Z_dn == 1 ) temp_Z[4] = 0.; 

		//_1if (profState_R3_Z_dn == 1 ) temp_Z[5] = 0.; 

		if (profState_L1_Z_dn == 1 ) temp_Z[0] = 0.; 

		//if (profState_R2_Z_dn == 1 ) temp_Z[1] = 0.;  

		//_1if (profState_L3_Z_dn == 1 ) temp_Z[2] = 0.;  

		Ref_R1[2] = temp_Z[3] + (Old_Hs_RF-temp_Z[3] + 1.*All_Foot_Z)*profResult_R1_Z_up - (Old_Hs_RF + 0.*All_Foot_Z)*profResult_R1_Z_dn ;
		//Ref_L2[2] = temp_Z[4] + (Old_Hs_RF-temp_Z[4] + 1.*All_Foot_Z)*profResult_L2_Z_up - (Old_Hs_RF + 0.*All_Foot_Z)*profResult_L2_Z_dn ;
		//_1Ref_R3[2] = temp_Z[5] + (Old_Hs_RF-temp_Z[5] + 1.*All_Foot_Z)*profResult_R3_Z_up - (Old_Hs_RF + 0.*All_Foot_Z)*profResult_R3_Z_dn ;

		Ref_L1[2] = temp_Z[0] + (Old_Hs_LF-temp_Z[0] + 1.*All_Foot_Z)*profResult_L1_Z_up - (Old_Hs_LF + 0.*All_Foot_Z)*profResult_L1_Z_dn ;
		//Ref_R2[2] = temp_Z[1] + (Old_Hs_LF-temp_Z[1] + 1.*All_Foot_Z)*profResult_R2_Z_up - (Old_Hs_LF + 0.*All_Foot_Z)*profResult_R2_Z_dn ;
		//_1Ref_L3[2] = temp_Z[2] + (Old_Hs_LF-temp_Z[2] + 1.*All_Foot_Z)*profResult_L3_Z_up - (Old_Hs_LF + 0.*All_Foot_Z)*profResult_L3_Z_dn ;
	}
	else{

		if (profState_R1_Z_dn == 1 ) temp_Z[0] = 0.; 

		//if (profState_L2_Z_dn == 1 ) temp_Z[1] = 0.; 

		//_1if (profState_R3_Z_dn == 1 ) temp_Z[2] = 0.;

		if (profState_L1_Z_dn == 1 ) temp_Z[3] = 0.; 

		//if (profState_R2_Z_dn == 1 ) temp_Z[4] = 0.; 

		//_1if (profState_L3_Z_dn == 1 ) temp_Z[5] = 0.; 

		Ref_R1[2] = temp_Z[0] + (Old_Hs_RF-temp_Z[0] + 1.*All_Foot_Z)*profResult_R1_Z_up - (Old_Hs_RF + 0.*All_Foot_Z)*profResult_R1_Z_dn ;  
		//Ref_L2[2] = temp_Z[1] + (Old_Hs_RF-temp_Z[1] + 1.*All_Foot_Z)*profResult_L2_Z_up - (Old_Hs_RF + 0.*All_Foot_Z)*profResult_L2_Z_dn ;
		//_1Ref_R3[2] = temp_Z[2] + (Old_Hs_RF-temp_Z[2] + 1.*All_Foot_Z)*profResult_R3_Z_up - (Old_Hs_RF + 0.*All_Foot_Z)*profResult_R3_Z_dn ;

		Ref_L1[2] = temp_Z[3] + (Old_Hs_LF-temp_Z[3] + 1.*All_Foot_Z)*profResult_L1_Z_up - (Old_Hs_LF + 0.*All_Foot_Z)*profResult_L1_Z_dn ;
		//Ref_R2[2] = temp_Z[4] + (Old_Hs_LF-temp_Z[4] + 1.*All_Foot_Z)*profResult_R2_Z_up - (Old_Hs_LF + 0.*All_Foot_Z)*profResult_R2_Z_dn ;
		//_1Ref_L3[2] = temp_Z[5] + (Old_Hs_LF-temp_Z[5] + 1.*All_Foot_Z)*profResult_L3_Z_up - (Old_Hs_LF + 0.*All_Foot_Z)*profResult_L3_Z_dn ;
	}
	// 수정 2012.09.17


	// 각 발의 보행 단계 설정 0 : 지면 구간,   1 : 들어올리는 구간,    2 : 내리는 구간 //
	if (profState_R1_Z_up == 2 ) Walking_stage[0] = 10; 
	else if (profState_R1_Z_dn == 2 ) Walking_stage[0] = 20;
	else if (profState_R1_Z_dn == 3 ) Walking_stage[0] = 0;

	//if (profState_L2_Z_up == 2 ) Walking_stage[1] = 10; 
	//else if (profState_L2_Z_dn == 2 ) Walking_stage[1] = 20;
	//else if (profState_L2_Z_dn == 3 ) Walking_stage[1] = 0;

	//_1if (profState_R3_Z_up == 2 ) Walking_stage[2] = 10; 
	//_1else if (profState_R3_Z_dn == 2 ) Walking_stage[2] = 20;
	//_1else if (profState_R3_Z_dn == 3 ) Walking_stage[2] = 0;

	if (profState_L1_Z_up == 2 ) Walking_stage[3] = 10; 
	else if (profState_L1_Z_dn == 2 ) Walking_stage[3] = 20;
	else if (profState_L1_Z_dn == 3 ) Walking_stage[3] = 0;

	//if (profState_R2_Z_up == 2 ) Walking_stage[4] = 10; 
	//else if (profState_R2_Z_dn == 2 ) Walking_stage[4] = 20;
	//else if (profState_R2_Z_dn == 3 ) Walking_stage[4] = 0;

	//_1if (profState_L3_Z_up == 2 ) Walking_stage[5] = 10; 
	//_1else if (profState_L3_Z_dn == 2 ) Walking_stage[5] = 20;
	//_1else if (profState_L3_Z_dn == 3 ) Walking_stage[5] = 0;

	
	

	// 각 발의 X방향(전진방향), Y방향(왼방향), 회전 각도 궤적 생성(0 ~ 1) //
	
	// RF 으로 통칭 //
	if(LF_stop_flag != 2 || profState_R1_Z_dn == 2 || profState_R1_Z_dn == 3){
		half_cycloneUp(&profState_R1_X_up,&profResult_R1_X_up,local_Index,next_start_time_R1 + 0,Ts/2); 
		half_cycloneDn(&profState_R1_X_dn,&profResult_R1_X_dn,local_Index,next_start_time_R1 + Ts/2,Ts/2); 

		//half_cycloneUp(&profState_L2_X_up,&profResult_L2_X_up,local_Index,next_start_time_L2 + Ts+Old_Td_RF,Ts/2); 
		//half_cycloneDn(&profState_L2_X_dn,&profResult_L2_X_dn,local_Index,next_start_time_L2 + Ts+Old_Td_RF+Ts/2,Ts/2); 

		//_1half_cycloneUp(&profState_R3_X_up,&profResult_R3_X_up,local_Index,next_start_time_R3 + 2*Ts+2*Old_Td_RF,Ts/2); 
		//_1half_cycloneDn(&profState_R3_X_dn,&profResult_R3_X_dn,local_Index,next_start_time_R3 + 2*Ts+2*Old_Td_RF+Ts/2,Ts/2); 
	}

	// LF 으로 통칭 //
	if(RF_stop_flag != 2 || profState_L1_Z_dn == 2 || profState_L1_Z_dn == 3){
		half_cycloneUp(&profState_L1_X_up,&profResult_L1_X_up,local_Index,next_start_time_L1 + Ts+Old_Td_LF + T_off,Ts/2); 
		half_cycloneDn(&profState_L1_X_dn,&profResult_L1_X_dn,local_Index,next_start_time_L1 + Ts+Old_Td_LF + T_off + Ts/2,Ts/2); 

		//half_cycloneUp(&profState_R2_X_up,&profResult_R2_X_up,local_Index,next_start_time_R2 + 3*Ts+3*Old_Td_LF + T_off,Ts/2); 
		//half_cycloneDn(&profState_R2_X_dn,&profResult_R2_X_dn,local_Index,next_start_time_R2 + 3*Ts+3*Old_Td_LF + T_off + Ts/2,Ts/2); 

		//_1half_cycloneUp(&profState_L3_X_up,&profResult_L3_X_up,local_Index,next_start_time_L3 + 5*Ts+5*Old_Td_LF + T_off,Ts/2); 
		//_1half_cycloneDn(&profState_L3_X_dn,&profResult_L3_X_dn,local_Index,next_start_time_L3 + 5*Ts+5*Old_Td_LF + T_off + Ts/2,Ts/2); 
	}

	// 스텝카운터에 따라 실제 X 방향 Y방향(왼방향) 회전 각도 진폭 적용 //  
	// RF 으로 통칭 //
	if(*Step_CNT < 2){
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_R1[0] = Temp_Ref_R1[0] + Old_Ls_RF*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_R1[0] = Temp2_Ref_R1[0] + Old_Ls_RF*profResult_R1_X_dn;

		//if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_L2[0] = Temp_Ref_L2[0] + Old_Ls_RF*profResult_L2_X_up ;
		//else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_L2[0] = Temp2_Ref_L2[0] + Old_Ls_RF*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_R3[0] = Temp_Ref_R3[0] + Old_Ls_RF*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_R3[0] = Temp2_Ref_R3[0] + Old_Ls_RF*profResult_R3_X_dn;

	}
	else if(*Step_CNT >= 2 && RF_stop_flag == 0){
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_R1[0] = Temp_Ref_R1[0] + (Temp_Ls_RF + Old_Ls_RF)*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_R1[0] = Temp2_Ref_R1[0] + (Temp_Ls_RF + Old_Ls_RF)*profResult_R1_X_dn;

		//if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_L2[0] = Temp_Ref_L2[0] + (Temp_Ls_RF + Old_Ls_RF)*profResult_L2_X_up ;
		//else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_L2[0] = Temp2_Ref_L2[0] + (Temp_Ls_RF + Old_Ls_RF)*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_R3[0] = Temp_Ref_R3[0] + (Temp_Ls_RF + Old_Ls_RF)*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_R3[0] = Temp2_Ref_R3[0] + (Temp_Ls_RF + Old_Ls_RF)*profResult_R3_X_dn;
	}
	else if(*Step_CNT >= 2 && (RF_stop_flag == 1 || RF_stop_flag == 2)){
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_R1[0] = Temp_Ref_R1[0] + Old_Ls_RF*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_R1[0] = Temp2_Ref_R1[0] + Old_Ls_RF*profResult_R1_X_dn;

		//if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_L2[0] = Temp_Ref_L2[0] + Old_Ls_RF*profResult_L2_X_up ;
		//else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_L2[0] = Temp2_Ref_L2[0] + Old_Ls_RF*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_R3[0] = Temp_Ref_R3[0] + Old_Ls_RF*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_R3[0] = Temp2_Ref_R3[0] + Old_Ls_RF*profResult_R3_X_dn;
	}


	if(Old_Lss_RF <= 0 && Lss != 0 ){
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_R1[1] = Temp_Ref_R1[1] + Old_Lss_RF*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_R1[1] = Temp2_Ref_R1[1] + Old_Lss_RF*profResult_R1_X_dn;

		//if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_L2[1] = Temp_Ref_L2[1] + Old_Lss_RF*profResult_L2_X_up ;
		//else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_L2[1] = Temp2_Ref_L2[1] + Old_Lss_RF*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_R3[1] = Temp_Ref_R3[1] + Old_Lss_RF*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_R3[1] = Temp2_Ref_R3[1] + Old_Lss_RF*profResult_R3_X_dn;

	}
	else if(Old_Lss_RF <= 0 && Lss == 0 ){
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_R1[1] = Temp_Ref_R1[1] + 0*Old_Lss_RF*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_R1[1] = Temp2_Ref_R1[1] + 0*Old_Lss_RF*profResult_R1_X_dn;

		//if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_L2[1] = Temp_Ref_L2[1] + Old_Lss_RF*profResult_L2_X_up ;
		//else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_L2[1] = Temp2_Ref_L2[1] + Old_Lss_RF*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_R3[1] = Temp_Ref_R3[1] + Old_Lss_RF*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_R3[1] = Temp2_Ref_R3[1] + Old_Lss_RF*profResult_R3_X_dn;

	}
	else if(Old_Lss_RF > 0 && Temp_profState_BC_Y == 2){  // 왼발 Start 인 아닌 일반적인 경우,
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_R1[1] = Temp_Ref_R1[1] + Old_Lss_RF*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_R1[1] = Temp2_Ref_R1[1] + Old_Lss_RF*profResult_R1_X_dn;

		//if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_L2[1] = Temp_Ref_L2[1] + Old_Lss_RF*profResult_L2_X_up ;
		//else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_L2[1] = Temp2_Ref_L2[1] + Old_Lss_RF*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_R3[1] = Temp_Ref_R3[1] + Old_Lss_RF*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_R3[1] = Temp2_Ref_R3[1] + Old_Lss_RF*profResult_R3_X_dn;
	}

	
	if(Old_Rot_Ang_RF <= 0 && Rot_Ang != 0 ){
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_th[0] = Temp_Ref_th[0] + Old_Rot_Ang_RF*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_th[0] = Temp2_Ref_th[0] + Old_Rot_Ang_RF*profResult_R1_X_dn;

		//if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_th[1] = Temp_Ref_th[1] + Old_Rot_Ang_RF*profResult_L2_X_up ;
		//else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_th[1] = Temp2_Ref_th[1] + Old_Rot_Ang_RF*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_th[2] = Temp_Ref_th[2] + Old_Rot_Ang_RF*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_th[2] = Temp2_Ref_th[2] + Old_Rot_Ang_RF*profResult_R3_X_dn;

	}
	else if(Old_Rot_Ang_RF <= 0 && Rot_Ang == 0 ){
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_th[0] = Temp_Ref_th[0] + 0*Old_Rot_Ang_RF*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_th[0] = Temp2_Ref_th[0] + 0*Old_Rot_Ang_RF*profResult_R1_X_dn;

		//if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_th[1] = Temp_Ref_th[1] + Old_Rot_Ang_RF*profResult_L2_X_up ;
		//else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_th[1] = Temp2_Ref_th[1] + Old_Rot_Ang_RF*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_th[2] = Temp_Ref_th[2] + Old_Rot_Ang_RF*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_th[2] = Temp2_Ref_th[2] + Old_Rot_Ang_RF*profResult_R3_X_dn;

	}
	else if(Old_Rot_Ang_RF > 0 && Temp_profState_BC_th == 2){  // 왼발 Start 인 아닌 일반적인 경우,
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_th[0] = Temp_Ref_th[0] + Old_Rot_Ang_RF*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_th[0] = Temp2_Ref_th[0] + Old_Rot_Ang_RF*profResult_R1_X_dn;

		//if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_th[1] = Temp_Ref_th[1] + Old_Rot_Ang_RF*profResult_L2_X_up ;
		//else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_th[1] = Temp2_Ref_th[1] + Old_Rot_Ang_RF*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_th[2] = Temp_Ref_th[2] + Old_Rot_Ang_RF*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_th[2] = Temp2_Ref_th[2] + Old_Rot_Ang_RF*profResult_R3_X_dn;
	}


	if(profState_R1_Z_dn == 1)  {  // 마지막 위치 임시 저장 //
		Temp2_Ref_R1[0] = Ref_R1[0]; 
		Temp2_Ref_R1[1] = Ref_R1[1]; 
		Temp2_Ref_th[0] = Ref_th[0]; 
	}
	else if(profState_R1_Z_dn == 3){
		Temp_Ref_R1[0] = Ref_R1[0];
		Temp_Ref_R1[1] = Ref_R1[1];
		Temp_Ref_th[0] = Ref_th[0]; 
	}

	//if(profState_L2_Z_dn == 1)  {
	//	Temp2_Ref_L2[0] = Ref_L2[0]; 
	//	Temp2_Ref_L2[1] = Ref_L2[1]; 
	//	Temp2_Ref_th[1] = Ref_th[1]; 
	//}
	//else if(profState_L2_Z_dn == 3){
	//	Temp_Ref_L2[0] = Ref_L2[0]; 
	//	Temp_Ref_L2[1] = Ref_L2[1]; 
	//	Temp_Ref_th[1] = Ref_th[1]; 
	//}

	//_1if(profState_R3_Z_dn == 1)  {
	//_1	Temp2_Ref_R3[0] = Ref_R3[0]; 
	//_1	Temp2_Ref_R3[1] = Ref_R3[1];
	//_1	Temp2_Ref_th[2] = Ref_th[2]; 
	//_1}
	//_1else if(profState_R3_Z_dn == 3){
	//_1	Temp_Ref_R3[0] = Ref_R3[0]; 
	//_1	Temp_Ref_R3[1] = Ref_R3[1]; 
	//_1	Temp_Ref_th[2] = Ref_th[2]; 
	//_1}


	// LF 으로 통칭 //
	if(*Step_CNT < 2 && LF_stop_flag != 1 && LF_stop_flag != 2){
		if(profState_L1_Z_up == 1 || profState_L1_Z_up == 2 || (profState_L1_Z_up == 3 && profState_L1_Z_dn == 1)) Ref_L1[0] = Temp_Ref_L1[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_L1_X_up ;
		else if(profState_L1_Z_up == 3 && profState_L1_Z_dn != 1)	Ref_L1[0] = Temp2_Ref_L1[0] + 2*Old_Ls_LF*profResult_L1_X_dn;

		//if(profState_R2_Z_up == 1 || profState_R2_Z_up == 2 || (profState_R2_Z_up == 3 && profState_R2_Z_dn == 1)) Ref_R2[0] = Temp_Ref_R2[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_R2_X_up ;
		//else if(profState_R2_Z_up == 3 && profState_R2_Z_dn != 1)	Ref_R2[0] = Temp2_Ref_R2[0] + 2*Old_Ls_LF*profResult_R2_X_dn;

		//_1if(profState_L3_Z_up == 1 || profState_L3_Z_up == 2 || (profState_L3_Z_up == 3 && profState_L3_Z_dn == 1)) Ref_L3[0] = Temp_Ref_L3[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_L3_X_up ;
		//_1else if(profState_L3_Z_up == 3 && profState_L3_Z_dn != 1)	Ref_L3[0] = Temp2_Ref_L3[0] + 2*Old_Ls_LF*profResult_L3_X_dn;

	}
	else if(*Step_CNT >= 2 && LF_stop_flag == 0){
		if(profState_L1_Z_up == 1 || profState_L1_Z_up == 2 || (profState_L1_Z_up == 3 && profState_L1_Z_dn == 1)) Ref_L1[0] = Temp_Ref_L1[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_L1_X_up ;
		else if(profState_L1_Z_up == 3 && profState_L1_Z_dn != 1)	Ref_L1[0] = Temp2_Ref_L1[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_L1_X_dn;

		//if(profState_R2_Z_up == 1 || profState_R2_Z_up == 2 || (profState_R2_Z_up == 3 && profState_R2_Z_dn == 1)) Ref_R2[0] = Temp_Ref_R2[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_R2_X_up ;
		//else if(profState_R2_Z_up == 3 && profState_R2_Z_dn != 1)	Ref_R2[0] = Temp2_Ref_R2[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_R2_X_dn;

		//_1if(profState_L3_Z_up == 1 || profState_L3_Z_up == 2 || (profState_L3_Z_up == 3 && profState_L3_Z_dn == 1)) Ref_L3[0] = Temp_Ref_L3[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_L3_X_up ;
		//_1else if(profState_L3_Z_up == 3 && profState_L3_Z_dn != 1)	Ref_L3[0] = Temp2_Ref_L3[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_L3_X_dn;
	}
	else if(LF_stop_flag == 1 || LF_stop_flag == 2){
		if(profState_L1_Z_up == 1 || profState_L1_Z_up == 2 || (profState_L1_Z_up == 3 && profState_L1_Z_dn == 1)) Ref_L1[0] = Temp_Ref_L1[0] + Old_Ls_LF*profResult_L1_X_up ;
		else if(profState_L1_Z_up == 3 && profState_L1_Z_dn != 1)	Ref_L1[0] = Temp2_Ref_L1[0] + Old_Ls_LF*profResult_L1_X_dn;

		//if(profState_R2_Z_up == 1 || profState_R2_Z_up == 2 || (profState_R2_Z_up == 3 && profState_R2_Z_dn == 1)) Ref_R2[0] = Temp_Ref_R2[0] + Old_Ls_LF*profResult_R2_X_up ;
		//else if(profState_R2_Z_up == 3 && profState_R2_Z_dn != 1)	Ref_R2[0] = Temp2_Ref_R2[0] + Old_Ls_LF*profResult_R2_X_dn;

		//_1if(profState_L3_Z_up == 1 || profState_L3_Z_up == 2 || (profState_L3_Z_up == 3 && profState_L3_Z_dn == 1)) Ref_L3[0] = Temp_Ref_L3[0] + Old_Ls_LF*profResult_L3_X_up ;
		//_1else if(profState_L3_Z_up == 3 && profState_L3_Z_dn != 1)	Ref_L3[0] = Temp2_Ref_L3[0] + Old_Ls_LF*profResult_L3_X_dn;
	}

	if(profState_L1_Z_up == 1 || profState_L1_Z_up == 2 || (profState_L1_Z_up == 3 && profState_L1_Z_dn == 1)) Ref_L1[1] = Temp_Ref_L1[1] + Old_Lss_LF*profResult_L1_X_up ;
	else if(profState_L1_Z_up == 3 && profState_L1_Z_dn != 1)	Ref_L1[1] = Temp2_Ref_L1[1] + Old_Lss_LF*profResult_L1_X_dn;

	//if(profState_R2_Z_up == 1 || profState_R2_Z_up == 2 || (profState_R2_Z_up == 3 && profState_R2_Z_dn == 1)) Ref_R2[1] = Temp_Ref_R2[1] + Old_Lss_LF*profResult_R2_X_up ;
	//else if(profState_R2_Z_up == 3 && profState_R2_Z_dn != 1)	Ref_R2[1] = Temp2_Ref_R2[1] + Old_Lss_LF*profResult_R2_X_dn;

	//_1if(profState_L3_Z_up == 1 || profState_L3_Z_up == 2 || (profState_L3_Z_up == 3 && profState_L3_Z_dn == 1)) Ref_L3[1] = Temp_Ref_L3[1] + Old_Lss_LF*profResult_L3_X_up ;
	//_1else if(profState_L3_Z_up == 3 && profState_L3_Z_dn != 1)	Ref_L3[1] = Temp2_Ref_L3[1] + Old_Lss_LF*profResult_L3_X_dn;

	
	if(profState_L1_Z_up == 1 || profState_L1_Z_up == 2 || (profState_L1_Z_up == 3 && profState_L1_Z_dn == 1)) Ref_th[3] = Temp_Ref_th[3] + Old_Rot_Ang_LF*profResult_L1_X_up ;
	else if(profState_L1_Z_up == 3 && profState_L1_Z_dn != 1)	Ref_th[3] = Temp2_Ref_th[3] + Old_Rot_Ang_LF*profResult_L1_X_dn;

	//if(profState_R2_Z_up == 1 || profState_R2_Z_up == 2 || (profState_R2_Z_up == 3 && profState_R2_Z_dn == 1)) Ref_th[4] = Temp_Ref_th[4] + Old_Rot_Ang_LF*profResult_R2_X_up ;
	//else if(profState_R2_Z_up == 3 && profState_R2_Z_dn != 1)	Ref_th[4] = Temp2_Ref_th[4] + Old_Rot_Ang_LF*profResult_R2_X_dn;

	//_1if(profState_L3_Z_up == 1 || profState_L3_Z_up == 2 || (profState_L3_Z_up == 3 && profState_L3_Z_dn == 1)) Ref_th[5] = Temp_Ref_th[5] + Old_Rot_Ang_LF*profResult_L3_X_up ;
	//_1else if(profState_L3_Z_up == 3 && profState_L3_Z_dn != 1)	Ref_th[5] = Temp2_Ref_th[5] + Old_Rot_Ang_LF*profResult_L3_X_dn;



	
	if(profState_L1_Z_dn == 1)  {  // 마지막 위치 임시 저장 //
		Temp2_Ref_L1[0] = Ref_L1[0]; 
		Temp2_Ref_L1[1] = Ref_L1[1]; 
		Temp2_Ref_th[3] = Ref_th[3]; 
	}
	else if(profState_L1_Z_dn == 3){
		Temp_Ref_L1[0] = Ref_L1[0]; 
		Temp_Ref_L1[1] = Ref_L1[1]; 
		Temp_Ref_th[3] = Ref_th[3];
	}

	//if(profState_R2_Z_dn == 1)  {
	//	Temp2_Ref_R2[0] = Ref_R2[0]; 
	//	Temp2_Ref_R2[1] = Ref_R2[1]; 
	//	Temp2_Ref_th[4] = Ref_th[4];
	//}
	//else if(profState_R2_Z_dn == 3){
	//	Temp_Ref_R2[0] = Ref_R2[0]; 
	//	Temp_Ref_R2[1] = Ref_R2[1]; 
	//	Temp_Ref_th[4] = Ref_th[4];
	//}

	//_1if(profState_L3_Z_dn == 1)  {
	//_1	Temp2_Ref_L3[0] = Ref_L3[0]; 
	//_1	Temp2_Ref_L3[1] = Ref_L3[1]; 
	//_1	Temp2_Ref_th[5] = Ref_th[5];
	//_1}
	//_1else if(profState_L3_Z_dn == 3){
	//_1	Temp_Ref_L3[0] = Ref_L3[0]; 
	//_1	Temp_Ref_L3[1] = Ref_L3[1]; 
	//_1	Temp_Ref_th[5] = Ref_th[5];
	//_1}

	
	// 몸통의 X방향(전진방향) Y방향(왼방향) 회전 각도 궤적 생성(0 ~ 1) //  
	prof_cos_linear(&profState_BC_X,&profResult_BC_X,local_Index,Ts/2 + next_start_time_BC,Ts + Old_Td_BC + Old_T_off_BC,0,shape_factor_F);   // 전진방향 궤적
	prof_cos_linear(&profState_ZMP_X,&profResult_ZMP_X,local_Index,Ts/2 + next_start_time_BC,Ts + Old_Td_BC + Old_T_off_BC,0,(int)(0.3*Ts)+Old_Td_BC/4);   // 전진방향 ZMP 궤적

	Ref_BC[0] = Old_BC_X + Old_Ls_BC*profResult_BC_X;
	Ref_ZMP[0] = Old_BC_X + Old_Ls_BC*profResult_ZMP_X;
	
	if(Old_Lss_BC <= 0 ){ // 측방향 보행 시 Y 방향 궤적
		prof_cos_linear(&profState_BC_Y,&profResult_BC_Y,local_Index,Ts/2 + next_start_time_BC_2,Ts + Old_Td_BC + Old_T_off_BC,0,shape_factor_S);
		
		Ref_BC[1] = Old_BC_Y + Old_Lss_BC*profResult_BC_Y;
	}
	else if(Old_Lss_BC > 0 ){
		prof_cos_linear(&profState_BC_Y,&profResult_BC_Y,local_Index,Ts/2 + Ts + Old_Td_BC + Old_T_off_BC + next_start_time_BC_2,Ts + Old_Td_BC + Old_T_off_BC,0,shape_factor_S);
		
		Ref_BC[1] = Old_BC_Y + Old_Lss_BC*profResult_BC_Y;
	}

	if(Old_Rot_Ang_BC <= 0){ // 회전 보행 시 회전 각도 궤적
		prof_cos_linear(&profState_BC_th,&profResult_BC_th,local_Index,Ts/2 + next_start_time_BC_th,Ts + Old_Td_BC + Old_T_off_BC,0,shape_factor_S);
		Ref_th[6] = Old_BC_th + Old_Rot_Ang_BC*profResult_BC_th;
	}
	else{
		prof_cos_linear(&profState_BC_th,&profResult_BC_th,local_Index,Ts/2 + Ts + Old_Td_BC + Old_T_off_BC + next_start_time_BC_th,Ts + Old_Td_BC + Old_T_off_BC,0,shape_factor_S);
		Ref_th[6] = Old_BC_th + Old_Rot_Ang_BC*profResult_BC_th;
	}
	

	
	if(BC_SWAY_CNT == 1)	{
		prof_delay_cos_delay(&profState_BC_Y_SWAY,&profResult_BC_Y_SWAY,local_Index,0,Ts + Old_Td_BC_2,0,Ts/10,Ts/10); // 보행시 몸통 SWAY 주기 운동		
		Del_BC_Y = 1.15*Old_Ab*profResult_BC_Y_SWAY;  // 2013_06_23
		prof_delay_cos_delay(&profState_ZMP_Y,&profResult_ZMP_Y,local_Index,0,Ts + Old_Td_BC_2,0,(int)(0.2*Ts)+Old_Td_BC_2/4,(int)(0.2*Ts)+Old_Td_BC_2/4); // 보행시 ZMP SWAY 주기 운동
		Del_ZMP_Y = A_ZMP_Y_BP*profResult_ZMP_Y;
	}
	else if(BC_SWAY_CNT == 2 && BC_SWAY_Stop_Flag == 0){
		prof_delay_cos_delay(&profState_BC_Y_SWAY,&profResult_BC_Y_SWAY,local_Index,next_start_time_BC_3,Ts + Old_Td_BC_2 + 0*Old_Td_BC_3 + Old_T_off_BC_2,0,Ts/10,Ts/10); 
		Del_BC_Y = Old_Del_BC_Y - (Old_Ab+1.15*Old_Ab_2)*profResult_BC_Y_SWAY;  // 2013_06_23
		prof_delay_cos_delay(&profState_ZMP_Y,&profResult_ZMP_Y,local_Index,next_start_time_BC_3,Ts + Old_Td_BC_2,0,(int)(0.2*Ts)+Old_Td_BC_2/4,(int)(0.2*Ts)+Old_Td_BC_2/4); 
		Del_ZMP_Y = Old_Del_ZMP_Y - 2*A_ZMP_Y_BP*profResult_ZMP_Y;
	}
	else if(BC_SWAY_CNT%2 == 0 && BC_SWAY_Stop_Flag == 0){
		prof_delay_cos_delay(&profState_BC_Y_SWAY,&profResult_BC_Y_SWAY,local_Index,next_start_time_BC_3,Ts + Old_Td_BC_2 + 0*Old_Td_BC_3 + Old_T_off_BC_2,0,Ts/10,Ts/10); 
		Del_BC_Y = Old_Del_BC_Y - (Old_Ab+Old_Ab_2)*profResult_BC_Y_SWAY;  // 2013_06_23
		prof_delay_cos_delay(&profState_ZMP_Y,&profResult_ZMP_Y,local_Index,next_start_time_BC_3,Ts + Old_Td_BC_2,0,(int)(0.2*Ts)+Old_Td_BC_2/4,(int)(0.2*Ts)+Old_Td_BC_2/4); 
		Del_ZMP_Y = Old_Del_ZMP_Y - 2*A_ZMP_Y_BP*profResult_ZMP_Y;
	}
	else if(BC_SWAY_CNT%2 == 1 && BC_SWAY_Stop_Flag == 0){
		prof_delay_cos_delay(&profState_BC_Y_SWAY,&profResult_BC_Y_SWAY,local_Index,next_start_time_BC_3,Ts + Old_Td_BC_2 + 0*Old_Td_BC_3 + Old_T_off_BC_2 ,0,Ts/10,Ts/10); 
		Del_BC_Y = Old_Del_BC_Y + (Old_Ab+Old_Ab_2)*profResult_BC_Y_SWAY; // 2013_06_23
		prof_delay_cos_delay(&profState_ZMP_Y,&profResult_ZMP_Y,local_Index,next_start_time_BC_3,Ts + Old_Td_BC_2,0,(int)(0.2*Ts)+Old_Td_BC_2/4,(int)(0.2*Ts)+Old_Td_BC_2/4); 
		Del_ZMP_Y = Old_Del_ZMP_Y + 2*A_ZMP_Y_BP*profResult_ZMP_Y;
	}
	else if(BC_SWAY_Stop_Flag == 1){
		prof_delay_cos_delay(&profState_BC_Y_SWAY,&profResult_BC_Y_SWAY,local_Index,next_start_time_BC_3,Ts + Old_Td_BC_2,0,Ts/10,Ts/10); 
		Del_BC_Y = Old_Del_BC_Y/1.05 - Old_Ab_2*profResult_BC_Y_SWAY; // 2013_06_23
		prof_delay_cos_delay(&profState_ZMP_Y,&profResult_ZMP_Y,local_Index,next_start_time_BC_3,Ts + Old_Td_BC_2,0,(int)(0.2*Ts)+Old_Td_BC_2/4,(int)(0.2*Ts)+Old_Td_BC_2/4); 
		Del_ZMP_Y = Old_Del_ZMP_Y - A_ZMP_Y_BP*profResult_ZMP_Y;
	}
	else if(BC_SWAY_Stop_Flag == 2){
		prof_delay_cos_delay(&profState_BC_Y_SWAY,&profResult_BC_Y_SWAY,local_Index,next_start_time_BC_3,Ts + Old_Td_BC_2,0,Ts/10,Ts/10); 
		Del_BC_Y = Old_Del_BC_Y/1.05 + Old_Ab_2*profResult_BC_Y_SWAY; // 2013_06_23
		prof_delay_cos_delay(&profState_ZMP_Y,&profResult_ZMP_Y,local_Index,next_start_time_BC_3,Ts + Old_Td_BC_2,0,(int)(0.2*Ts)+Old_Td_BC_2/4,(int)(0.2*Ts)+Old_Td_BC_2/4); 
		Del_ZMP_Y = Old_Del_ZMP_Y + A_ZMP_Y_BP*profResult_ZMP_Y;
	}


	// 마지막 스텝 몸통 좌우 진폭 1.05 배 //
	if(RF_stop_flag != 0 && Del_BC_Y >= 0) {   // 2013_08_02
		Del_BC_Y = 1.05*Del_BC_Y;
	}
	else if(LF_stop_flag != 0 && Del_BC_Y <= 0) {
		Del_BC_Y = 1.05*Del_BC_Y;
	}	


	// Hip Roll 관절 처짐 보상 각도 궤적 생성 //  2013_08_02
	if(LF_start_flag == 1){
		if(Del_BC_Y > 0) RHR_Compen_Ang = -HR_COMP_ANG*Del_BC_Y/55.;
		else RHR_Compen_Ang = 0.;

		if(Del_BC_Y < 0) LHR_Compen_Ang = -HR_COMP_ANG*Del_BC_Y/55.;
		else LHR_Compen_Ang = 0.;
	}
	else{
		if(Del_BC_Y > 0) LHR_Compen_Ang = HR_COMP_ANG*Del_BC_Y/55.;
		else LHR_Compen_Ang = 0.;

		if(Del_BC_Y < 0) RHR_Compen_Ang = HR_COMP_ANG*Del_BC_Y/55.;
		else RHR_Compen_Ang = 0.;
	}
		

	
	if(profState_BC_X == 3 && LF_stop_flag == 0 && RF_stop_flag == 0) {
		Old_BC_X = Ref_BC[0];
		
		next_start_time_BC = local_Index - Ts/2; 
		
		if(*Step_CNT%2 ==1) {

			Old_Td_BC = Td;    
			Old_Ls_BC = Ls;
			Old_Lss_BC = Lss; 
			Old_Rot_Ang_BC = Rot_Ang;

	
			if(2*Ts+2*Td < Ts)  Old_T_off_BC = Ts - (2*Ts+2*Td); // 최소 3점 지지를 유지하기 위한 시간 딜레이 설정
			else Old_T_off_BC = 0;

			
		}

	}

	if(Old_Lss_BC <= 0 && profState_BC_Y == 3 && LF_stop_flag == 0 && RF_stop_flag == 0) { // Lss < 0 일때, 즉 오른쪽으로 이동시

		Old_BC_Y = Ref_BC[1];

		next_start_time_BC_2 = local_Index - Ts/2 + Ts + Old_Td_BC + Old_T_off_BC;

				
	}
	else if(Old_Lss_BC > 0 && profState_BC_Y == 3 && LF_stop_flag == 0 && RF_stop_flag == 0) { // Lss > 0 일때, 즉 왼쪽으로 이동시  

		Old_BC_Y = Ref_BC[1];

		next_start_time_BC_2 = local_Index - Ts/2;
		
	}

	if(Old_Rot_Ang_BC <= 0 && profState_BC_th == 3 && LF_stop_flag == 0 && RF_stop_flag == 0) { // Rot_Ang < 0 일때, 즉 오른쪽으로 이동시

		Old_BC_th = Ref_th[6];

		next_start_time_BC_th = local_Index - Ts/2 + Ts + Old_Td_BC + Old_T_off_BC;
		
	}
	else if(Old_Rot_Ang_BC > 0 && profState_BC_th == 3 && LF_stop_flag == 0 && RF_stop_flag == 0) { // Rot_Ang > 0 일때, 즉 왼쪽으로 이동시  

		Old_BC_th = Ref_th[6];

		next_start_time_BC_th = local_Index - Ts/2;
	}

	if(profState_BC_Y_SWAY == 3 && BC_SWAY_Stop_Flag == 0){
		if(2*Ts+2*Td < Ts && profState_L2_Z_up == 3)  Old_T_off_BC_2 = Ts - (2*Ts+2*Td); // 최소 3점 지지를 유지하기 위한 시간 딜레이 설정
		else if(2*Ts+2*Td >= Ts && profState_L2_Z_up == 3) Old_T_off_BC_2 = 0;

		Old_Td_BC_3 = Old_Td_BC_2;
		if(BC_SWAY_CNT%2 == 1)	Old_Td_BC_2 = Td;


		Old_Del_BC_Y = Del_BC_Y; Old_Del_ZMP_Y = Del_ZMP_Y;
		next_start_time_BC_3 = local_Index;
		BC_SWAY_CNT++;

		Old_Ab_2 = Old_Ab;// 2013_06_23
		Old_Ab = Ab; // 2013_06_23

		if(BC_SWAY_CNT > 2 && RF_stop_flag != 0 /*&& profState_L2_Z_dn == 3*/)	BC_SWAY_Stop_Flag = 1; 
		else if(BC_SWAY_CNT > 2 && LF_stop_flag != 0 /*&& profState_R2_Z_dn == 3*/)	BC_SWAY_Stop_Flag = 2; 
	}

	//Ref_ZMP[1] = Ref_BC[1] + Del_ZMP_Y; // ZMP 측방향 궤적은 옆걸음 + ZMP 측방향 SWAY 
	//Ref_BC[1] += Del_BC_Y;   // 몸통 측방향 궤적은 옆걸음 + 몸통 측방향 SWAY 
	

	
	if(WP_Flow_Control_Flag != 1){

		if(Go_flag == 0 && local_Index == 0)  { // 보행 하기전에 go flag 가 0 이면 local_index = 0, Step_CNT = 0
			local_Index = 0;
			Temp_local_Index = 0;
			Real_Stop_Flag = 0;
			*Step_CNT = 0; BC_SWAY_CNT = 0;
			LF_stop_flag = 0;
			RF_stop_flag = 0;
			RHR_Compen_Ang = 0.;   //2013_08_02
			LHR_Compen_Ang = 0.;   //2013_08_02
			BC_SWAY_Stop_Flag = 0;
			next_start_time_R1 = (Ts/2+Td);
			//next_start_time_R2 = 0;
			//_1next_start_time_R3 = 0;
			next_start_time_L1 = (Ts/2+Td);
			//next_start_time_L2 = 0;
			//_1next_start_time_L3 = 0;
			next_start_time_BC = (Ts/2+Td);	next_start_time_BC_2 = (Ts/2+Td); next_start_time_BC_3 = (Ts/2+Td); next_start_time_BC_th = (Ts/2+Td);
		
			Old_Hs_RF = 0.;
			Old_Hs_LF = 0.;
			Old_Ls_RF = 0.; Old_Lss_RF = 0.;
			Old_Ls_LF = 0.; Old_Lss_LF = 0.;
			Old_Ls_BC = 0.;
			Old_Lss_BC = 0.; 
			Old_Td_RF = 0.;
			Old_Td_LF = 0.;
			Old_Td_BC = 0.; Old_Td_BC_2 = 0.; Old_Td_BC_3 = 0.;
			Temp_Ls_RF = 0.; Temp_Lss_RF = 0.;
			Temp_Ls_LF = 0.; Temp_Lss_LF = 0.;
			Old_BC_X = 0.;
			Old_BC_Y = 0.;
			Old_BC_th = 0.;
			Del_BC_Y = 0.; Old_Del_BC_Y = 0.;
			Del_ZMP_Y = 0.; Old_Del_ZMP_Y = 0.;
			Ref_ZMP[0] = 0.; Ref_ZMP[1] = 0.;

			T_off = 0; Old_T_off_BC = 0; Old_T_off_BC_2 = 0; 

			Old_profState_R1_Z_up = 0; Old_profState_L1_Z_up = 0;

			//LF_start_flag = 0;   // 수정 2012.09.17


			if(Foot_Location_Flag){   // 추가 2013.04.30

				if(temp_X[RF_1] != 0) temp_X[RF_1] = temp_X[RF_1] - Temp_Ref_R1[0];// 추가 2013.04.30
				if(temp_X[LF_1] != 0) temp_X[LF_1] = temp_X[LF_1] - Temp_Ref_L1[0];// 추가 2013.04.30
							
				if(LF_start_flag){
					if(temp_Y[RF_1] != 0) temp_Y[RF_1] = temp_Y[RF_1] + Temp_Ref_R1[1];// 추가 2013.04.30
					if(temp_Y[LF_1] != 0) temp_Y[LF_1] = temp_Y[LF_1] + Temp_Ref_L1[1];// 추가 2013.04.30
				}
				else{ 	
					if(temp_Y[RF_1] != 0) temp_Y[RF_1] = temp_Y[RF_1] - Temp_Ref_R1[1];// 추가 2013.04.30
					if(temp_Y[LF_1] != 0) temp_Y[LF_1] = temp_Y[LF_1] - Temp_Ref_L1[1];// 추가 2013.04.30
				}

				if(LF_start_flag){
					if(temp_th[RF_1] != 0) temp_th[RF_1] = temp_th[RF_1] + Temp_Ref_th[RF_1];// 추가 2013.04.30
					if(temp_th[LF_1] != 0) temp_th[LF_1] = temp_th[LF_1] + Temp_Ref_th[LF_1];// 추가 2013.04.30
				}
				else {
					if(temp_th[RF_1] != 0) temp_th[RF_1] = temp_th[RF_1] - Temp_Ref_th[RF_1];// 추가 2013.04.30
					if(temp_th[LF_1] != 0) temp_th[LF_1] = temp_th[LF_1] - Temp_Ref_th[LF_1];// 추가 2013.04.30
				}

				Foot_Location_Flag = 0;
			}

			Ref_R1[0] = 0.; Ref_R1[1] = 0.; //Ref_R1[2] = 0.;    // 수정 2012.09.17
			Ref_L1[0] = 0.; Ref_L1[1] = 0.; //Ref_L1[2] = 0.;    // 수정 2012.09.17
			Ref_R2[0] = 0.; Ref_R2[1] = 0.; //Ref_R2[2] = 0.;    // 수정 2012.09.17
			Ref_L2[0] = 0.; Ref_L2[1] = 0.; //Ref_L2[2] = 0.;    // 수정 2012.09.17
			//_1Ref_R3[0] = 0.; Ref_R3[1] = 0.; //Ref_R3[2] = 0.;    // 수정 2012.09.17
			//_1Ref_L3[0] = 0.; Ref_L3[1] = 0.; //Ref_L3[2] = 0.;    // 수정 2012.09.17
			Ref_BC[0] = 0.; Ref_BC[1] = 0.; //Ref_BC[2] = 0.;    // 수정 2012.09.17
			Ref_th[0] = 0.; Ref_th[1] = 0.; Ref_th[2] = 0.; Ref_th[3] = 0.; Ref_th[4] = 0.; Ref_th[5] = 0.; Ref_th[6] = 0.;

			Temp_Ref_R1[0] = 0.; Temp_Ref_R1[1] = 0.; Temp_Ref_R1[2] = 0.; Temp2_Ref_R1[0] = 0.; Temp2_Ref_R1[1] = 0.; Temp2_Ref_R1[2] = 0.;
			Temp_Ref_R2[0] = 0.; Temp_Ref_R2[1] = 0.; Temp_Ref_R2[2] = 0.; Temp2_Ref_R2[0] = 0.; Temp2_Ref_R2[1] = 0.; Temp2_Ref_R2[2] = 0.;
			//_1Temp_Ref_R3[0] = 0.; Temp_Ref_R3[1] = 0.; Temp_Ref_R3[2] = 0.; Temp2_Ref_R3[0] = 0.; Temp2_Ref_R3[1] = 0.; Temp2_Ref_R3[2] = 0.;
			Temp_Ref_L1[0] = 0.; Temp_Ref_L1[1] = 0.; Temp_Ref_L1[2] = 0.; Temp2_Ref_L1[0] = 0.; Temp2_Ref_L1[1] = 0.; Temp2_Ref_L1[2] = 0.;
			Temp_Ref_L2[0] = 0.; Temp_Ref_L2[1] = 0.; Temp_Ref_L2[2] = 0.; Temp2_Ref_L2[0] = 0.; Temp2_Ref_L2[1] = 0.; Temp2_Ref_L2[2] = 0.;
			//_1Temp_Ref_L3[0] = 0.; Temp_Ref_L3[1] = 0.; Temp_Ref_L3[2] = 0.; Temp2_Ref_L3[0] = 0.; Temp2_Ref_L3[1] = 0.; Temp2_Ref_L3[2] = 0.;
			Temp_Ref_th[0] = 0.; Temp_Ref_th[1] = 0.; Temp_Ref_th[2] = 0.; Temp_Ref_th[3] = 0.; Temp_Ref_th[4] = 0.; Temp_Ref_th[5] = 0.;
			Temp2_Ref_th[0] = 0.; Temp2_Ref_th[1] = 0.; Temp2_Ref_th[2] = 0.; Temp2_Ref_th[3] = 0.; Temp2_Ref_th[4] = 0.; Temp2_Ref_th[5] = 0.;
		
		
		}
		else if(Go_flag == 0 && *Step_CNT != 0 && Real_Stop_Flag == 0) { // 보행하는 중에 go flag 가 0 이면 계속 local_index 증가시키다가
			local_Index ++ ;   

			if(Old_Lss_BC == 0 && Old_Rot_Ang_BC == 0){   // Side Step 과 회전이 없을때                                                                   
				if(profState_R1_Z_dn == 3 && RF_stop_flag == 0 && LF_stop_flag == 0) LF_stop_flag = 1; // 왼발 정지를 준비시키고
				else if(profState_L1_Z_dn == 3 && RF_stop_flag == 0 && LF_stop_flag == 0) RF_stop_flag = 1; // 오른발 정지를 준비시키고

				if(LF_stop_flag == 1 && profState_L1_Z_up != 0) LF_stop_flag = 2; // 왼발 마지막 스텝이 시작 되면
				else if(RF_stop_flag == 1 && profState_R1_Z_up != 0) RF_stop_flag = 2; // 오른발 마지막 스텝이 시작 되면

				if(profState_R1_Z_dn == 3  && RF_stop_flag == 2 && profState_BC_Y_SWAY == 3) Real_Stop_Flag = 1; // 오른발 마지막 스텝이 끝나고 Real_Stop_Flag 을 Enable 시켜 정지 한다
				else if(profState_L1_Z_dn == 3  && LF_stop_flag == 2 && profState_BC_Y_SWAY == 3) Real_Stop_Flag = 1; // 왼발 마지막 스텝이 끝나고 Real_Stop_Flag 을 Enable 시켜 정지 한다
			}
			else if(Old_Lss_BC < 0 || Old_Rot_Ang_BC < 0){   // Side Step 이 있을때(오른쪽 방향 시작이므로 왼발로 끝난다.)
				if(profState_R1_Z_dn == 3 && RF_stop_flag == 0 && LF_stop_flag == 0) LF_stop_flag = 1; // 왼발 정지를 준비시키고
				
				if(LF_stop_flag == 1 && profState_L1_Z_up != 0) LF_stop_flag = 2; // 왼발 마지막 스텝이 시작 되면
				
				if(profState_L1_Z_dn == 3  && LF_stop_flag == 2 && profState_BC_Y_SWAY == 3) Real_Stop_Flag = 1; // 왼발 마지막 스텝이 끝나고 Real_Stop_Flag 을 Enable 시켜 정지 한다
			}
			else if(Old_Lss_BC > 0 || Old_Rot_Ang_BC > 0){   // Side Step 이 있을때(왼쪽 방향 시작이므로 오른발로 끝난다.)
				if(profState_L1_Z_dn == 3 && RF_stop_flag == 0 && LF_stop_flag == 0) RF_stop_flag = 1; // 오른발 정지를 준비시키고

				if(RF_stop_flag == 1 && profState_R1_Z_up != 0) RF_stop_flag = 2; // 오른발 마지막 스텝이 시작 되면

				if(profState_R1_Z_dn == 3  && RF_stop_flag == 2 && profState_BC_Y_SWAY == 3) Real_Stop_Flag = 1; // 오른발 마지막 스텝이 끝나고 Real_Stop_Flag 을 Enable 시켜 정지 한다
				
			}


			//if(RF_stop_flag == 2 || LF_stop_flag == 2) {   // 2013_01_07 추가되었음.:이른착지 시 x,y 위치 복원을 위함.   // 디버그 필요 //
			//	for(i = 0; i<6; i++) Early_Landing_Flag[i] = 0;
			//}
			
		}
		else if(Go_flag == 0 && *Step_CNT != 0 && Real_Stop_Flag == 1){
			local_Index = 0;
			Real_Stop_Flag = 0;
			*Step_CNT = 0;
			LF_stop_flag = 0;
			RF_stop_flag = 0;
		}
		else {
			if(LF_start_flag && Go_flag == 1 && local_Index == 0){   // 추가 2013.04.30

				if(temp_X[LF_1] != 0 && temp_X[RF_1] == 0) {
					//printf("temp_X[RF1] = %f,	temp_X[LF1] = %f \n",temp_X[RF1], temp_X[LF1]);
					temp_X[RF_1] = temp_X[LF_1]; EL_Del_xy_CNT[RF_1] = 0;}
				else if(temp_X[RF_1] != 0 && temp_X[LF_1] == 0) {
					temp_X[LF_1] = temp_X[RF_1]; EL_Del_xy_CNT[LF_1] = 0;
				}


				if(temp_Y[LF_1] != 0 && temp_Y[RF_1] == 0) {
					//printf("temp_Y[RF1] = %f,	temp_Y[LF1] = %f \n",temp_Y[RF1], temp_Y[LF1]);
					temp_Y[RF_1] = temp_Y[LF_1]; EL_Del_xy_CNT[RF_1] = 0;}
				else if(temp_Y[RF_1] != 0 && temp_Y[LF_1] == 0) {
					temp_Y[LF_1] = temp_Y[RF_1]; EL_Del_xy_CNT[LF_1] = 0;}


				if(temp_th[LF_1] != 0 && temp_th[RF_1] == 0) {
					//printf("temp_th[RF1] = %f,	temp_th[LF1] = %f \n",temp_th[RF1], temp_th[LF1]);
					temp_th[RF_1] = temp_th[LF_1]; EL_Del_xy_CNT[RF_1] = 0;}
				else if(temp_th[RF_1] != 0 && temp_th[LF_1] == 0) {
					temp_th[LF_1] = temp_th[RF_1]; EL_Del_xy_CNT[LF_1] = 0;}

			}	
			
			local_Index++;  // Go_flag 가 1 이되면 local_index 증가 시작 
		}
	} // End of if(WP_Flow_Control_Flag != 1){


	
	// 보행 파라미터 바꾸는 시점 //
	if(Old_profState_R1_Z_dn == 2 && profState_R1_Z_dn == 3 && RF_stop_flag != 2 && LF_stop_flag != 2) *Change_flag = 1;   // RF 마지막 다리 착지 후 주기당 한번씩 뜸
	else *Change_flag = 0;

	Old_profState_R1_Z_dn = profState_R1_Z_dn;


	// 각 발의 스윙 끝나는 순간 다음 각발의 시작시간 지정 //
	if((RF_stop_flag == 0 ||  RF_stop_flag == 1) && (LF_stop_flag == 0 || LF_stop_flag == 1)){
		if(profState_R1_Z_dn == 3) {
			Old_Td_LF = Td;
			if(2*Ts+2*Td < Ts)  T_off = Ts - (2*Ts+2*Td); // 최소 3점 지지를 유지하기 위한 시간 딜레이 설정
			else T_off = 0;
			
			Temp_local_Index = local_Index - 1;
			next_start_time_R1 = Temp_local_Index + Ts+2*Old_Td_RF + 2*T_off;
		}
		//else if(profState_R2_Z_dn == 3) next_start_time_R2 = Temp_local_Index + 3*Ts+4*Old_Td_LF + 2*T_off;
		//_1else if(profState_R3_Z_dn == 3) next_start_time_R3 = Temp_local_Index + 3*Ts+4*Old_Td_RF + 2*T_off;
		else if(profState_L1_Z_dn == 3) next_start_time_L1 = Temp_local_Index + Ts + 2*Old_Td_LF + 2*T_off;
		//else if(profState_L2_Z_dn == 3) next_start_time_L2 = Temp_local_Index + 3*Ts+4*Old_Td_RF + 2*T_off;
		//_1else if(profState_L3_Z_dn == 3) next_start_time_L3 = Temp_local_Index + 3*Ts+4*Old_Td_LF + 2*T_off;
	}


	
	// 스텝카운터에 따라 실제 X 방향 진폭 적용 //  
	if(profState_R1_Z_up == 1) {

		Temp_profState_BC_Y = profState_BC_Y; 

		Temp_profState_BC_th = profState_BC_th; 

		if(RF_stop_flag == 0){
			Temp_Ls_RF = Old_Ls_RF;
			Old_Ls_RF = Ls;

			if(Lss >= 0){
				Old_Lss_RF = Old_Lss_LF;
			}
			else{
				Temp_Lss_RF = Old_Lss_RF;
				Old_Lss_RF = Lss;
			}

		}
		else if(RF_stop_flag == 1 || RF_stop_flag == 2 && Lss > 0){
			Old_Lss_RF = Old_Lss_LF;
		}


		if(RF_stop_flag == 0){

			if(Rot_Ang >= 0){
				Old_Rot_Ang_RF = Old_Rot_Ang_LF;
			}
			else{
				
				Old_Rot_Ang_RF = Rot_Ang;
			}
		}
		else if(RF_stop_flag == 1 || RF_stop_flag == 2 && Rot_Ang > 0){
			Old_Rot_Ang_RF = Old_Rot_Ang_LF;
		}

	}

	if(profState_R1_Z_dn == 3 && RF_stop_flag == 0 ) {

			Temp_Ls_LF = Ls;
			Old_Ls_LF = Ls;

			Temp_Lss_LF = Old_Lss_LF;
			Old_Lss_LF = Lss;

			Old_Rot_Ang_LF = Rot_Ang;

	}

	BC_Y_OFFSET = Ref_BC[1];                  // 2013_07_29
	if(LF_start_flag) BC_Y_OFFSET = -BC_Y_OFFSET;

	Ref_ZMP[1] = Ref_BC[1] + Del_ZMP_Y; // ZMP 측방향 궤적은 옆걸음 + ZMP 측방향 SWAY 
	Ref_BC[1] += Del_BC_Y;   // 몸통 측방향 궤적은 옆걸음 + 몸통 측방향 SWAY 


	// 왼발 Start 인 경우, 오른발과 왼발 궤적을 서로 바꿔준다.
	if(LF_start_flag){

		for(i = 0; i<3; i++) {
			SW_Temp_R1[i] = Ref_R1[i];
			Ref_R1[i] = Ref_L1[i];
			Ref_L1[i] = SW_Temp_R1[i];

			SW_Temp_L2[i] = Ref_L2[i];
			Ref_L2[i] = Ref_R2[i];
			Ref_R2[i] = SW_Temp_L2[i];

			//_1SW_Temp_R3[i] = Ref_R3[i];
			//_1Ref_R3[i] = Ref_L3[i];
			//_1Ref_L3[i] = SW_Temp_R3[i];

			SW_Temp_th = Ref_th[i];
			Ref_th[i] = Ref_th[i+3];
			Ref_th[i+3] = SW_Temp_th;
		}

				
		Ref_R1[1] = -Ref_R1[1];
		Ref_R2[1] = -Ref_R2[1];
		//_1Ref_R3[1] = -Ref_R3[1];
		Ref_L1[1] = -Ref_L1[1];
		Ref_L2[1] = -Ref_L2[1];
		//_1Ref_L3[1] = -Ref_L3[1];

		Ref_BC[1] = -Ref_BC[1];

		Ref_ZMP[1] = -Ref_ZMP[1];

		for(i=0; i<7; i++)	Ref_th[i] = -Ref_th[i];

		SW_Temp_Walking_stage = Walking_stage[0];
		Walking_stage[0] = Walking_stage[3];
		Walking_stage[3] = SW_Temp_Walking_stage;

		//SW_Temp_Walking_stage = Walking_stage[1];
		//Walking_stage[1] = Walking_stage[4];
		//Walking_stage[4] = SW_Temp_Walking_stage;

		//SW_Temp_Walking_stage = Walking_stage[2];
		//Walking_stage[2] = Walking_stage[5];
		//Walking_stage[5] = SW_Temp_Walking_stage;


	}

	// Early Lading Case //  // 추가 2012.09.17
	if(Early_Landing_Flag[0] && Walking_stage[0] == 20 && temp_Z[0] == 0.)	temp_Z[0] = Ref_R1[2];
	
	//if(Early_Landing_Flag[1] && Walking_stage[1] == 20 && temp_Z[1] == 0.)	temp_Z[1] = Ref_L2[2]; 
	
	//_1if(Early_Landing_Flag[2] && Walking_stage[2] == 20 && temp_Z[2] == 0.) 	temp_Z[2] = Ref_R3[2]; 
	
	if(Early_Landing_Flag[3] && Walking_stage[3] == 20 && temp_Z[3] == 0.)	temp_Z[3] = Ref_L1[2]; 
	
	//if(Early_Landing_Flag[4] && Walking_stage[4] == 20 && temp_Z[4] == 0.)	temp_Z[4] = Ref_R2[2];
	
	//_1if(Early_Landing_Flag[5] && Walking_stage[5] == 20 && temp_Z[5] == 0.)	temp_Z[5] = Ref_L3[2];
	
	
	
	if(/*Early_Landing_Flag[0] == 0 &&*/ Old_Walking_stage[0] == 20 && Walking_stage[0] == 0 && temp_Z[0] == 0.)	temp_Z[0] = Ref_R1[2];
	
	//if(/*Early_Landing_Flag[1] == 0 &&*/ Old_Walking_stage[1] == 20 && Walking_stage[1] == 0 && temp_Z[1] == 0.)	temp_Z[1] = Ref_L2[2]; 
	
	//_1if(/*Early_Landing_Flag[2] == 0 &&*/ Old_Walking_stage[2] == 20 && Walking_stage[2] == 0 && temp_Z[2] == 0.)	temp_Z[2] = Ref_R3[2]; 
	
	if(/*Early_Landing_Flag[3] == 0 &&*/ Old_Walking_stage[3] == 20 && Walking_stage[3] == 0 && temp_Z[3] == 0.)	temp_Z[3] = Ref_L1[2]; 
	
	//if(/*Early_Landing_Flag[4] == 0 &&*/ Old_Walking_stage[4] == 20 && Walking_stage[4] == 0 && temp_Z[4] == 0.)	temp_Z[4] = Ref_R2[2]; 
	
	//_1if(/*Early_Landing_Flag[5] == 0 &&*/ Old_Walking_stage[5] == 20 && Walking_stage[5] == 0 && temp_Z[5] == 0.)	temp_Z[5] = Ref_L3[2]; 


	
	if(Early_Landing_Flag[0] && Walking_stage[0] != 10)	Ref_R1[2] = temp_Z[0];
	
	//if(Early_Landing_Flag[1] && Walking_stage[1] != 10)	Ref_L2[2] = temp_Z[1] ;
	
	//_1if(Early_Landing_Flag[2] && Walking_stage[2] != 10)	Ref_R3[2] = temp_Z[2];
	
	if(Early_Landing_Flag[3] && Walking_stage[3] != 10)	Ref_L1[2] = temp_Z[3] ;
	
	//if(Early_Landing_Flag[4] && Walking_stage[4] != 10)	Ref_R2[2] = temp_Z[4] ;
	
	//_1if(Early_Landing_Flag[5] && Walking_stage[5] != 10)	Ref_L3[2] = temp_Z[5];

	
	// Fz[i] = 0. 는 실제 로봇에는 필요없음 //
	
	if(Walking_stage[0] == 10)	{Early_Landing_Flag[0] = 0; }
	
	//if(Walking_stage[1] == 10) 	{Early_Landing_Flag[1] = 0; Fz[1] = 0.;}
	
	//_1if(Walking_stage[2] == 10)	{Early_Landing_Flag[2] = 0; Fz[2] = 0.;}
	
	if(Walking_stage[3] == 10)	{Early_Landing_Flag[3] = 0; }
		
	//if(Walking_stage[4] == 10) 	{Early_Landing_Flag[4] = 0; Fz[4] = 0.;}
		
	//_1if(Walking_stage[5] == 10)	{Early_Landing_Flag[5] = 0; Fz[5] = 0.;}
		
	Ref_R1[2] -= All_Foot_Z + LL_Del_z[0];  
	//Ref_L2[2] -= All_Foot_Z + LL_Del_z[1];  
	//_1Ref_R3[2] -= All_Foot_Z + LL_Del_z[2];  

	Ref_L1[2] -= All_Foot_Z + LL_Del_z[3];  
	//Ref_R2[2] -= All_Foot_Z + LL_Del_z[4];  
	//_1Ref_L3[2] -= All_Foot_Z + LL_Del_z[5];  
	// 수정 2012.09.17
	
	
	// 테스트 용 //
	test_var[0] = Ab;//10*RF_stop_flag;
	test_var[1] = Old_Ab;//10*LF_stop_flag;
	test_var[2] = Old_Ab_2;//10*Real_Stop_Flag;
	test_var[3] = 10*Go_flag;
	test_var[4] = Walking_stage[LF_1];//10*Early_Landing_Flag[LF1];
	test_var[5] = 0.;

}

void Quadruped_walking_pattern(unsigned char Go_flag, 
			int Ts, 
			int Td, 
			int shape_factor_F,
			int shape_factor_S,
			double Ab,
			double Hs, 
			double Ls, 
			double Lss,
			double Rot_Ang,
			unsigned char Early_Landing_Flag[6],
			int *Step_CNT, 
			double Ref_BC[3], 
			double Ref_R1[3], 
			double Ref_R2[3], 
			double Ref_R3[3], 
			double Ref_L1[3], 
			double Ref_L2[3], 
			double Ref_L3[3], 
			double Ref_th[7],
			double Ref_ZMP[2],
			unsigned char Walking_stage[6], 
			unsigned char *Change_flag, 
			double test_var[10])
{
	static unsigned char Real_Stop_Flag = 0, RF_stop_flag = 0, LF_stop_flag = 0, LF_start_flag = 0, BC_SWAY_Stop_Flag = 0;
	static unsigned long local_Index = 0, Temp_local_Index = 0;
	static unsigned long next_start_time_R1 = (Ts/2+Td), next_start_time_R2 = (Ts/2+Td), next_start_time_R3 = 0,next_start_time_L1 = (Ts/2+Td), next_start_time_L2 = (Ts/2+Td), next_start_time_L3 = 0, 
						 next_start_time_BC = (Ts/2+Td), next_start_time_BC_2 = (Ts/2+Td), next_start_time_BC_3 = (Ts/2+Td), next_start_time_BC_th = (Ts/2+Td);
	static double	Temp_Ref_R1[3] = {0,0,0}, Temp2_Ref_R1[3] = {0,0,0}, Temp_Ref_L2[3] = {0,0,0}, Temp2_Ref_L2[3] = {0,0,0}, Temp_Ref_R3[3] = {0,0,0}, Temp2_Ref_R3[3] = {0,0,0},
					Temp_Ref_L1[3] = {0,0,0}, Temp2_Ref_L1[3] = {0,0,0}, Temp_Ref_R2[3] = {0,0,0}, Temp2_Ref_R2[3] = {0,0,0}, Temp_Ref_L3[3] = {0,0,0}, Temp2_Ref_L3[3] = {0,0,0},
					Temp_Ref_th[6] = {0,0,0,0,0,0}, Temp2_Ref_th[6] = {0,0,0,0,0,0};
	static double	Old_Hs_RF = 0., Old_Hs_LF = 0., Old_Ls_RF = 0., Old_Lss_RF = 0., Old_Ls_LF = 0., Old_Lss_LF = 0, Old_Ls_BC = 0., Old_Lss_BC = 0., Old_Td_RF = 0., 
					Old_Td_LF = 0.,Old_Td_BC = 0., Old_Td_BC_2 = 0.,Old_Td_BC_3 = 0., Temp_Ls_RF = 0., Temp_Lss_RF = 0., Temp_Ls_LF = 0., Temp_Lss_LF = 0., Old_BC_X = 0., Old_BC_Y = 0., 
					Old_Del_BC_Y = 0., Del_BC_Y = 0., Old_Rot_Ang_RF = 0., Old_Rot_Ang_LF = 0., Old_Rot_Ang_BC = 0., Old_BC_th = 0.,Old_Del_ZMP_Y = 0., Del_ZMP_Y = 0., Old_Ab = 0., Old_Ab_2 = 0; // 2013_06_23;
	static int		T_off = 0, Old_T_off_BC = 0, Old_T_off_BC_2 = 0, BC_SWAY_CNT = 1;   
	static int		Old_profState_R1_Z_up = 0, Old_profState_L1_Z_up = 0, Temp_profState_BC_Y = 0, Temp_profState_BC_th = 0, Old_profState_L2_Z_dn = 0 ;
	static int		profState_R1_Z_up = 0, profState_R2_Z_up = 0, profState_R3_Z_up = 0, profState_L1_Z_up = 0, profState_L2_Z_up = 0, profState_L3_Z_up = 0,
					profState_R1_Z_dn = 0, profState_R2_Z_dn = 0, profState_R3_Z_dn = 0, profState_L1_Z_dn = 0, profState_L2_Z_dn = 0, profState_L3_Z_dn = 0;
	static int		profState_R1_X_up = 0, profState_R2_X_up = 0, profState_R3_X_up = 0, profState_L1_X_up = 0, profState_L2_X_up = 0, profState_L3_X_up = 0,
					profState_R1_X_dn = 0, profState_R2_X_dn = 0, profState_R3_X_dn = 0, profState_L1_X_dn = 0, profState_L2_X_dn = 0, profState_L3_X_dn = 0;
	static int		profState_BC_X = 0, profState_BC_Y = 0,profState_BC_Y_SWAY = 0, profState_BC_th = 0, profState_ZMP_Y = 0, profState_ZMP_X = 0;
	static double	profResult_R1_Z_up = 0, profResult_R2_Z_up = 0, profResult_R3_Z_up = 0, profResult_L1_Z_up = 0, profResult_L2_Z_up = 0, profResult_L3_Z_up = 0,
					profResult_R1_Z_dn = 0, profResult_R2_Z_dn = 0, profResult_R3_Z_dn = 0, profResult_L1_Z_dn = 0, profResult_L2_Z_dn = 0, profResult_L3_Z_dn = 0;
	static double	profResult_R1_X_up = 0, profResult_R2_X_up = 0, profResult_R3_X_up = 0, profResult_L1_X_up = 0, profResult_L2_X_up = 0, profResult_L3_X_up = 0,
					profResult_R1_X_dn = 0, profResult_R2_X_dn = 0, profResult_R3_X_dn = 0, profResult_L1_X_dn = 0, profResult_L2_X_dn = 0, profResult_L3_X_dn = 0;
	static double	profResult_BC_X = 0., profResult_BC_Y = 0., profResult_BC_Y_SWAY = 0.,profResult_BC_th = 0.,profResult_ZMP_Y = 0.,profResult_ZMP_X = 0.;
	int		i = 0;
	static double	SW_Temp_R1[3] = {0,0,0}, SW_Temp_L2[3] = {0,0,0}, SW_Temp_R3[3] = {0,0,0}, SW_Temp_th = 0.;
	int				SW_Temp_Walking_stage = 0;
	


//	printf("%d	%d	%d	%d	%d	%d \n",Early_Landing_Flag[0],Early_Landing_Flag[1],Early_Landing_Flag[2],Early_Landing_Flag[3],Early_Landing_Flag[4],Early_Landing_Flag[5]); // ?? 2012.09.17

	if(local_Index == 0 && Go_flag == 1){ // ?? ???? ??  

		if(Lss > 0 || Rot_Ang > 0) {
			LF_start_flag = 1; // ?? Start ? ??, ?? ?? ???? Enable ???. +?? ??? +?? ???? ?? ?????. +?? ??? -?? ???? ?? ??? ? ??.
			Lss = -Lss; 
			Rot_Ang = -Rot_Ang;
		}
		else LF_start_flag = 0;


		Old_Td_RF = Td;
		Old_Td_LF = Td;
		Old_Td_BC = Td; 
		Old_Td_BC_2 = Td;
		Old_Td_BC_3 = Td;
		Old_Ls_RF = Ls;
		Old_Lss_RF = Lss;
		Old_Ls_LF = Ls;
		Old_Lss_LF = Lss;
		Old_Ls_BC = Ls;
		Old_Lss_BC = Lss; 	
		Temp_Ls_RF = Old_Ls_RF;
		Temp_Ls_LF = Old_Ls_LF;
		Old_Del_BC_Y = 0.; Del_BC_Y = 0.;
		Old_Del_ZMP_Y = 0.; Del_ZMP_Y = 0.;
		Old_Rot_Ang_RF = Rot_Ang;
		Old_Rot_Ang_LF = Rot_Ang;
		Old_Rot_Ang_BC = Rot_Ang;
		Old_Ab = Ab;  //2013_06_23
		Old_Ab_2 = Old_Ab;  //2013_06_23
		

		BC_SWAY_CNT = 1;


		if(2*Ts+2*Td < Ts)  T_off = Ts - (2*Ts+2*Td); // ?? 3? ??? ???? ?? ?? ??? ??
		else T_off = 0;

		Old_T_off_BC = T_off;
		Old_T_off_BC_2 = T_off;
			
	}

	if(local_Index == 1)  Foot_Location_Flag = 1;  // ?? 2013.04.30


	if(LF_start_flag && local_Index != 0){  // ?? Start ? ??, ???? ?? ??? ?? ????? ?? ???????.

		for(i = 0; i<3; i++) {
			SW_Temp_R1[i] = Ref_R1[i];
			Ref_R1[i] = Ref_L1[i];
			Ref_L1[i] = SW_Temp_R1[i];

			SW_Temp_L2[i] = Ref_L2[i];
			Ref_L2[i] = Ref_R2[i];
			Ref_R2[i] = SW_Temp_L2[i];

			//_1SW_Temp_R3[i] = Ref_R3[i];
			//_1Ref_R3[i] = Ref_L3[i];
			//_1Ref_L3[i] = SW_Temp_R3[i];

			SW_Temp_th = Ref_th[i];
			Ref_th[i] = Ref_th[i+3];
			Ref_th[i+3] = SW_Temp_th;
		}

		Ref_R1[1] = -Ref_R1[1];
		Ref_R2[1] = -Ref_R2[1];
		//_1Ref_R3[1] = -Ref_R3[1];
		Ref_L1[1] = -Ref_L1[1];
		Ref_L2[1] = -Ref_L2[1];
		//_1Ref_L3[1] = -Ref_L3[1];

		Ref_BC[1] = -Ref_BC[1];
		Ref_ZMP[1] = -Ref_ZMP[1];

		for(i=0; i<7; i++)	Ref_th[i] = -Ref_th[i];

		Lss = -1.*Lss; // ?? Start ? ??, ? Lss > 0 ??? ?? Lss < 0 ?? ??? ? ?? ????? ????? ???? ???.
		Rot_Ang = -Rot_Ang;

		
		SW_Temp_Walking_stage = Walking_stage[0];
		Walking_stage[0] = Walking_stage[3];
		Walking_stage[3] = SW_Temp_Walking_stage;

		SW_Temp_Walking_stage = Walking_stage[1];
		Walking_stage[1] = Walking_stage[4];
		Walking_stage[4] = SW_Temp_Walking_stage;

		//SW_Temp_Walking_stage = Walking_stage[2];
		//Walking_stage[2] = Walking_stage[5];
		//Walking_stage[5] = SW_Temp_Walking_stage;
	}



	// ? ?? Z??(????) ?? ??(0 ~ 1) //
	
	// RF ?? ?? //
	if(RF_stop_flag != 2 || profState_L2_Z_dn == 2){
		prof_delay_cos_delay(&profState_R1_Z_up,&profResult_R1_Z_up,local_Index,next_start_time_R1 + 0,Ts/2,0,0,0); 
		prof_delay_cos_delay(&profState_R1_Z_dn,&profResult_R1_Z_dn,local_Index,next_start_time_R1 + Ts/2,Ts/2,0,0,0); 
		prof_delay_cos_delay(&profState_L2_Z_up,&profResult_L2_Z_up,local_Index,next_start_time_L2 + Ts+Old_Td_RF,Ts/2,0,0,0); 
		prof_delay_cos_delay(&profState_L2_Z_dn,&profResult_L2_Z_dn,local_Index,next_start_time_L2 + Ts+Old_Td_RF+Ts/2,Ts/2,0,0,0); 
		//_1 prof_delay_cos_delay(&profState_R3_Z_up,&profResult_R3_Z_up,local_Index,next_start_time_R3 + 2*Ts+2*Old_Td_RF,Ts/2,0,0,0); 
		//_1prof_delay_cos_delay(&profState_R3_Z_dn,&profResult_R3_Z_dn,local_Index,next_start_time_R3 + 2*Ts+2*Old_Td_RF+Ts/2,Ts/2,0,0,0); 
	}

	// LF ?? ?? //
	if(LF_stop_flag != 2 || profState_R2_Z_dn == 2){
		prof_delay_cos_delay(&profState_L1_Z_up,&profResult_L1_Z_up,local_Index,next_start_time_L1 + 2*Ts+2*Old_Td_LF + T_off,Ts/2,0,0,0); 
		prof_delay_cos_delay(&profState_L1_Z_dn,&profResult_L1_Z_dn,local_Index,next_start_time_L1 + 2*Ts+2*Old_Td_LF + T_off + Ts/2,Ts/2,0,0,0); 
		prof_delay_cos_delay(&profState_R2_Z_up,&profResult_R2_Z_up,local_Index,next_start_time_R2 + 3*Ts+3*Old_Td_LF + T_off,Ts/2,0,0,0); 
		prof_delay_cos_delay(&profState_R2_Z_dn,&profResult_R2_Z_dn,local_Index,next_start_time_R2 + 3*Ts+3*Old_Td_LF + T_off +Ts/2,Ts/2,0,0,0); 
		//_1prof_delay_cos_delay(&profState_L3_Z_up,&profResult_L3_Z_up,local_Index,next_start_time_L3 + 5*Ts+5*Old_Td_LF + T_off,Ts/2,0,0,0); 
		//_1prof_delay_cos_delay(&profState_L3_Z_dn,&profResult_L3_Z_dn,local_Index,next_start_time_L3 + 5*Ts+5*Old_Td_LF + T_off + Ts/2,Ts/2,0,0,0); 
	}

	//printf("Td = %d \n",Td);
	
	// ??? ??? ?? CNT ?? //
	if((Old_profState_L1_Z_up == 0 && profState_L1_Z_up !=0) || (Old_profState_R1_Z_up == 0 && profState_R1_Z_up !=0)) {
		*Step_CNT = *Step_CNT + 1;   // ? ?? ?? ?? CNT ?? 

		//printf("Step_CNT= %d \n",*Step_CNT);
	}

	Old_profState_R1_Z_up = profState_R1_Z_up;
	Old_profState_L1_Z_up = profState_L1_Z_up;


	// ?????? ?? ?? Z ?? ?? ?? //  
	if(profState_R1_Z_up == 1) {
		Old_Hs_RF = Hs;
		Old_Td_RF = Td;
	}
	else if(profState_R1_Z_dn == 3) {
		Old_Hs_LF = Hs;
	}


	// ?? 2012.09.17
	if(LF_start_flag){
	
		if (profState_R1_Z_dn == 1 ) temp_Z[3] = 0.; 

		if (profState_L2_Z_dn == 1 ) temp_Z[4] = 0.; 

		//_1if (profState_R3_Z_dn == 1 ) temp_Z[5] = 0.; 

		if (profState_L1_Z_dn == 1 ) temp_Z[0] = 0.; 

		if (profState_R2_Z_dn == 1 ) temp_Z[1] = 0.;  

		//_1if (profState_L3_Z_dn == 1 ) temp_Z[2] = 0.;  

		Ref_R1[2] = temp_Z[3] + (Old_Hs_RF-temp_Z[3] + 1.*All_Foot_Z)*profResult_R1_Z_up - (Old_Hs_RF + 0.*All_Foot_Z)*profResult_R1_Z_dn ;
		Ref_L2[2] = temp_Z[4] + (Old_Hs_RF-temp_Z[4] + 1.*All_Foot_Z)*profResult_L2_Z_up - (Old_Hs_RF + 0.*All_Foot_Z)*profResult_L2_Z_dn ;
		//_1Ref_R3[2] = temp_Z[5] + (Old_Hs_RF-temp_Z[5] + 1.*All_Foot_Z)*profResult_R3_Z_up - (Old_Hs_RF + 0.*All_Foot_Z)*profResult_R3_Z_dn ;

		Ref_L1[2] = temp_Z[0] + (Old_Hs_LF-temp_Z[0] + 1.*All_Foot_Z)*profResult_L1_Z_up - (Old_Hs_LF + 0.*All_Foot_Z)*profResult_L1_Z_dn ;
		Ref_R2[2] = temp_Z[1] + (Old_Hs_LF-temp_Z[1] + 1.*All_Foot_Z)*profResult_R2_Z_up - (Old_Hs_LF + 0.*All_Foot_Z)*profResult_R2_Z_dn ;
		//_1Ref_L3[2] = temp_Z[2] + (Old_Hs_LF-temp_Z[2] + 1.*All_Foot_Z)*profResult_L3_Z_up - (Old_Hs_LF + 0.*All_Foot_Z)*profResult_L3_Z_dn ;
	}
	else{

		if (profState_R1_Z_dn == 1 ) temp_Z[0] = 0.; 

		if (profState_L2_Z_dn == 1 ) temp_Z[1] = 0.; 

		//_1if (profState_R3_Z_dn == 1 ) temp_Z[2] = 0.;

		if (profState_L1_Z_dn == 1 ) temp_Z[3] = 0.; 

		if (profState_R2_Z_dn == 1 ) temp_Z[4] = 0.; 

		//_1if (profState_L3_Z_dn == 1 ) temp_Z[5] = 0.; 

		Ref_R1[2] = temp_Z[0] + (Old_Hs_RF-temp_Z[0] + 1.*All_Foot_Z)*profResult_R1_Z_up - (Old_Hs_RF + 0.*All_Foot_Z)*profResult_R1_Z_dn ;  
		Ref_L2[2] = temp_Z[1] + (Old_Hs_RF-temp_Z[1] + 1.*All_Foot_Z)*profResult_L2_Z_up - (Old_Hs_RF + 0.*All_Foot_Z)*profResult_L2_Z_dn ;
		//_1Ref_R3[2] = temp_Z[2] + (Old_Hs_RF-temp_Z[2] + 1.*All_Foot_Z)*profResult_R3_Z_up - (Old_Hs_RF + 0.*All_Foot_Z)*profResult_R3_Z_dn ;

		Ref_L1[2] = temp_Z[3] + (Old_Hs_LF-temp_Z[3] + 1.*All_Foot_Z)*profResult_L1_Z_up - (Old_Hs_LF + 0.*All_Foot_Z)*profResult_L1_Z_dn ;
		Ref_R2[2] = temp_Z[4] + (Old_Hs_LF-temp_Z[4] + 1.*All_Foot_Z)*profResult_R2_Z_up - (Old_Hs_LF + 0.*All_Foot_Z)*profResult_R2_Z_dn ;
		//_1Ref_L3[2] = temp_Z[5] + (Old_Hs_LF-temp_Z[5] + 1.*All_Foot_Z)*profResult_L3_Z_up - (Old_Hs_LF + 0.*All_Foot_Z)*profResult_L3_Z_dn ;
	}
	// ?? 2012.09.17


	// ? ?? ?? ?? ?? 0 : ?? ??,   1 : ????? ??,    2 : ??? ?? //
	if (profState_R1_Z_up == 2 ) Walking_stage[0] = 10; 
	else if (profState_R1_Z_dn == 2 ) Walking_stage[0] = 20;
	else if (profState_R1_Z_dn == 3 ) Walking_stage[0] = 0;

	if (profState_L2_Z_up == 2 ) Walking_stage[1] = 10; 
	else if (profState_L2_Z_dn == 2 ) Walking_stage[1] = 20;
	else if (profState_L2_Z_dn == 3 ) Walking_stage[1] = 0;

	//_1if (profState_R3_Z_up == 2 ) Walking_stage[2] = 10; 
	//_1else if (profState_R3_Z_dn == 2 ) Walking_stage[2] = 20;
	//_1else if (profState_R3_Z_dn == 3 ) Walking_stage[2] = 0;

	if (profState_L1_Z_up == 2 ) Walking_stage[3] = 10; 
	else if (profState_L1_Z_dn == 2 ) Walking_stage[3] = 20;
	else if (profState_L1_Z_dn == 3 ) Walking_stage[3] = 0;

	if (profState_R2_Z_up == 2 ) Walking_stage[4] = 10; 
	else if (profState_R2_Z_dn == 2 ) Walking_stage[4] = 20;
	else if (profState_R2_Z_dn == 3 ) Walking_stage[4] = 0;

	//_1if (profState_L3_Z_up == 2 ) Walking_stage[5] = 10; 
	//_1else if (profState_L3_Z_dn == 2 ) Walking_stage[5] = 20;
	//_1else if (profState_L3_Z_dn == 3 ) Walking_stage[5] = 0;

	
	

	// ? ?? X??(????), Y??(???), ?? ?? ?? ??(0 ~ 1) //
	
	// RF ?? ?? //
	if(RF_stop_flag != 2 || profState_L2_Z_dn == 2 || profState_L2_Z_dn == 3){
		half_cycloneUp(&profState_R1_X_up,&profResult_R1_X_up,local_Index,next_start_time_R1 + 0,Ts/2); 
		half_cycloneDn(&profState_R1_X_dn,&profResult_R1_X_dn,local_Index,next_start_time_R1 + Ts/2,Ts/2); 

		half_cycloneUp(&profState_L2_X_up,&profResult_L2_X_up,local_Index,next_start_time_L2 + Ts+Old_Td_RF,Ts/2); 
		half_cycloneDn(&profState_L2_X_dn,&profResult_L2_X_dn,local_Index,next_start_time_L2 + Ts+Old_Td_RF+Ts/2,Ts/2); 

		//_1half_cycloneUp(&profState_R3_X_up,&profResult_R3_X_up,local_Index,next_start_time_R3 + 2*Ts+2*Old_Td_RF,Ts/2); 
		//_1half_cycloneDn(&profState_R3_X_dn,&profResult_R3_X_dn,local_Index,next_start_time_R3 + 2*Ts+2*Old_Td_RF+Ts/2,Ts/2); 
	}

	// LF ?? ?? //
	if(LF_stop_flag != 2 || profState_R2_Z_dn == 2 || profState_R2_Z_dn == 3){
		half_cycloneUp(&profState_L1_X_up,&profResult_L1_X_up,local_Index,next_start_time_L1 + 2*Ts+2*Old_Td_LF + T_off,Ts/2); 
		half_cycloneDn(&profState_L1_X_dn,&profResult_L1_X_dn,local_Index,next_start_time_L1 + 2*Ts+2*Old_Td_LF + T_off + Ts/2,Ts/2); 

		half_cycloneUp(&profState_R2_X_up,&profResult_R2_X_up,local_Index,next_start_time_R2 + 3*Ts+3*Old_Td_LF + T_off,Ts/2); 
		half_cycloneDn(&profState_R2_X_dn,&profResult_R2_X_dn,local_Index,next_start_time_R2 + 3*Ts+3*Old_Td_LF + T_off + Ts/2,Ts/2); 

		//_1half_cycloneUp(&profState_L3_X_up,&profResult_L3_X_up,local_Index,next_start_time_L3 + 5*Ts+5*Old_Td_LF + T_off,Ts/2); 
		//_1half_cycloneDn(&profState_L3_X_dn,&profResult_L3_X_dn,local_Index,next_start_time_L3 + 5*Ts+5*Old_Td_LF + T_off + Ts/2,Ts/2); 
	}

	// ?????? ?? ?? X ?? Y??(???) ?? ?? ?? ?? //  
	// RF ?? ?? //
	if(*Step_CNT < 3){
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_R1[0] = Temp_Ref_R1[0] + Old_Ls_RF*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_R1[0] = Temp2_Ref_R1[0] + Old_Ls_RF*profResult_R1_X_dn;

		if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_L2[0] = Temp_Ref_L2[0] + Old_Ls_RF*profResult_L2_X_up ;
		else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_L2[0] = Temp2_Ref_L2[0] + Old_Ls_RF*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_R3[0] = Temp_Ref_R3[0] + Old_Ls_RF*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_R3[0] = Temp2_Ref_R3[0] + Old_Ls_RF*profResult_R3_X_dn;

	}
	else if(*Step_CNT >= 3 && LF_stop_flag == 0){
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_R1[0] = Temp_Ref_R1[0] + (Temp_Ls_RF + Old_Ls_RF)*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_R1[0] = Temp2_Ref_R1[0] + (Temp_Ls_RF + Old_Ls_RF)*profResult_R1_X_dn;

		if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_L2[0] = Temp_Ref_L2[0] + (Temp_Ls_RF + Old_Ls_RF)*profResult_L2_X_up ;
		else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_L2[0] = Temp2_Ref_L2[0] + (Temp_Ls_RF + Old_Ls_RF)*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_R3[0] = Temp_Ref_R3[0] + (Temp_Ls_RF + Old_Ls_RF)*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_R3[0] = Temp2_Ref_R3[0] + (Temp_Ls_RF + Old_Ls_RF)*profResult_R3_X_dn;
	}
	else if(*Step_CNT >= 3 && (LF_stop_flag == 1 || LF_stop_flag == 2)){
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_R1[0] = Temp_Ref_R1[0] + Old_Ls_RF*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_R1[0] = Temp2_Ref_R1[0] + Old_Ls_RF*profResult_R1_X_dn;

		if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_L2[0] = Temp_Ref_L2[0] + Old_Ls_RF*profResult_L2_X_up ;
		else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_L2[0] = Temp2_Ref_L2[0] + Old_Ls_RF*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_R3[0] = Temp_Ref_R3[0] + Old_Ls_RF*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_R3[0] = Temp2_Ref_R3[0] + Old_Ls_RF*profResult_R3_X_dn;
	}


	if(Old_Lss_RF <= 0 && Lss != 0 ){
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_R1[1] = Temp_Ref_R1[1] + Old_Lss_RF*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_R1[1] = Temp2_Ref_R1[1] + Old_Lss_RF*profResult_R1_X_dn;

		if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_L2[1] = Temp_Ref_L2[1] + Old_Lss_RF*profResult_L2_X_up ;
		else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_L2[1] = Temp2_Ref_L2[1] + Old_Lss_RF*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_R3[1] = Temp_Ref_R3[1] + Old_Lss_RF*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_R3[1] = Temp2_Ref_R3[1] + Old_Lss_RF*profResult_R3_X_dn;

	}
	else if(Old_Lss_RF <= 0 && Lss == 0 ){
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_R1[1] = Temp_Ref_R1[1] + 0*Old_Lss_RF*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_R1[1] = Temp2_Ref_R1[1] + 0*Old_Lss_RF*profResult_R1_X_dn;

		if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_L2[1] = Temp_Ref_L2[1] + 0*Old_Lss_RF*profResult_L2_X_up ;
		else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_L2[1] = Temp2_Ref_L2[1] + 0*Old_Lss_RF*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_R3[1] = Temp_Ref_R3[1] + Old_Lss_RF*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_R3[1] = Temp2_Ref_R3[1] + Old_Lss_RF*profResult_R3_X_dn;

	}
	else if(Old_Lss_RF > 0 && Temp_profState_BC_Y == 2){  // ?? Start ? ?? ???? ??,
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_R1[1] = Temp_Ref_R1[1] + Old_Lss_RF*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_R1[1] = Temp2_Ref_R1[1] + Old_Lss_RF*profResult_R1_X_dn;

		if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_L2[1] = Temp_Ref_L2[1] + Old_Lss_RF*profResult_L2_X_up ;
		else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_L2[1] = Temp2_Ref_L2[1] + Old_Lss_RF*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_R3[1] = Temp_Ref_R3[1] + Old_Lss_RF*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_R3[1] = Temp2_Ref_R3[1] + Old_Lss_RF*profResult_R3_X_dn;
	}

	
	if(Old_Rot_Ang_RF <= 0 && Rot_Ang != 0 ){
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_th[0] = Temp_Ref_th[0] + Old_Rot_Ang_RF*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_th[0] = Temp2_Ref_th[0] + Old_Rot_Ang_RF*profResult_R1_X_dn;

		if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_th[1] = Temp_Ref_th[1] + Old_Rot_Ang_RF*profResult_L2_X_up ;
		else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_th[1] = Temp2_Ref_th[1] + Old_Rot_Ang_RF*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_th[2] = Temp_Ref_th[2] + Old_Rot_Ang_RF*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_th[2] = Temp2_Ref_th[2] + Old_Rot_Ang_RF*profResult_R3_X_dn;

	}
	else if(Old_Rot_Ang_RF <= 0 && Rot_Ang == 0 ){
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_th[0] = Temp_Ref_th[0] + 0*Old_Rot_Ang_RF*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_th[0] = Temp2_Ref_th[0] + 0*Old_Rot_Ang_RF*profResult_R1_X_dn;

		if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_th[1] = Temp_Ref_th[1] + 0*Old_Rot_Ang_RF*profResult_L2_X_up ;
		else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_th[1] = Temp2_Ref_th[1] + 0*Old_Rot_Ang_RF*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_th[2] = Temp_Ref_th[2] + Old_Rot_Ang_RF*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_th[2] = Temp2_Ref_th[2] + Old_Rot_Ang_RF*profResult_R3_X_dn;

	}
	else if(Old_Rot_Ang_RF > 0 && Temp_profState_BC_th == 2){  // ?? Start ? ?? ???? ??,
		if(profState_R1_Z_up == 1 || profState_R1_Z_up == 2 || (profState_R1_Z_up == 3 && profState_R1_Z_dn == 1)) Ref_th[0] = Temp_Ref_th[0] + Old_Rot_Ang_RF*profResult_R1_X_up ;
		else if(profState_R1_Z_up == 3 && profState_R1_Z_dn != 1)	Ref_th[0] = Temp2_Ref_th[0] + Old_Rot_Ang_RF*profResult_R1_X_dn;

		if(profState_L2_Z_up == 1 || profState_L2_Z_up == 2 || (profState_L2_Z_up == 3 && profState_L2_Z_dn == 1)) Ref_th[1] = Temp_Ref_th[1] + Old_Rot_Ang_RF*profResult_L2_X_up ;
		else if(profState_L2_Z_up == 3 && profState_L2_Z_dn != 1)	Ref_th[1] = Temp2_Ref_th[1] + Old_Rot_Ang_RF*profResult_L2_X_dn;

		//_1if(profState_R3_Z_up == 1 || profState_R3_Z_up == 2 || (profState_R3_Z_up == 3 && profState_R3_Z_dn == 1)) Ref_th[2] = Temp_Ref_th[2] + Old_Rot_Ang_RF*profResult_R3_X_up ;
		//_1else if(profState_R3_Z_up == 3 && profState_R3_Z_dn != 1)	Ref_th[2] = Temp2_Ref_th[2] + Old_Rot_Ang_RF*profResult_R3_X_dn;
	}


	if(profState_R1_Z_dn == 1)  {  // ??? ?? ?? ?? //
		Temp2_Ref_R1[0] = Ref_R1[0]; 
		Temp2_Ref_R1[1] = Ref_R1[1]; 
		Temp2_Ref_th[0] = Ref_th[0]; 
	}
	else if(profState_R1_Z_dn == 3){
		Temp_Ref_R1[0] = Ref_R1[0];
		Temp_Ref_R1[1] = Ref_R1[1];
		Temp_Ref_th[0] = Ref_th[0]; 
	}

	if(profState_L2_Z_dn == 1)  {
		Temp2_Ref_L2[0] = Ref_L2[0]; 
		Temp2_Ref_L2[1] = Ref_L2[1]; 
		Temp2_Ref_th[1] = Ref_th[1]; 
	}
	else if(profState_L2_Z_dn == 3){
		Temp_Ref_L2[0] = Ref_L2[0]; 
		Temp_Ref_L2[1] = Ref_L2[1]; 
		Temp_Ref_th[1] = Ref_th[1]; 
	}

	//_1if(profState_R3_Z_dn == 1)  {
	//_1	Temp2_Ref_R3[0] = Ref_R3[0]; 
	//_1	Temp2_Ref_R3[1] = Ref_R3[1];
	//_1	Temp2_Ref_th[2] = Ref_th[2]; 
	//_1}
	//_1else if(profState_R3_Z_dn == 3){
	//_1	Temp_Ref_R3[0] = Ref_R3[0]; 
	//_1	Temp_Ref_R3[1] = Ref_R3[1]; 
	//_1	Temp_Ref_th[2] = Ref_th[2]; 
	//_1}


	// LF ?? ?? //
	if(*Step_CNT < 4 && RF_stop_flag != 1 && RF_stop_flag != 2){
		if(profState_L1_Z_up == 1 || profState_L1_Z_up == 2 || (profState_L1_Z_up == 3 && profState_L1_Z_dn == 1)) Ref_L1[0] = Temp_Ref_L1[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_L1_X_up ;
		else if(profState_L1_Z_up == 3 && profState_L1_Z_dn != 1)	Ref_L1[0] = Temp2_Ref_L1[0] + 2*Old_Ls_LF*profResult_L1_X_dn;

		if(profState_R2_Z_up == 1 || profState_R2_Z_up == 2 || (profState_R2_Z_up == 3 && profState_R2_Z_dn == 1)) Ref_R2[0] = Temp_Ref_R2[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_R2_X_up ;
		else if(profState_R2_Z_up == 3 && profState_R2_Z_dn != 1)	Ref_R2[0] = Temp2_Ref_R2[0] + 2*Old_Ls_LF*profResult_R2_X_dn;

		//_1if(profState_L3_Z_up == 1 || profState_L3_Z_up == 2 || (profState_L3_Z_up == 3 && profState_L3_Z_dn == 1)) Ref_L3[0] = Temp_Ref_L3[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_L3_X_up ;
		//_1else if(profState_L3_Z_up == 3 && profState_L3_Z_dn != 1)	Ref_L3[0] = Temp2_Ref_L3[0] + 2*Old_Ls_LF*profResult_L3_X_dn;

	}
	else if(*Step_CNT >= 4 && RF_stop_flag == 0){
		if(profState_L1_Z_up == 1 || profState_L1_Z_up == 2 || (profState_L1_Z_up == 3 && profState_L1_Z_dn == 1)) Ref_L1[0] = Temp_Ref_L1[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_L1_X_up ;
		else if(profState_L1_Z_up == 3 && profState_L1_Z_dn != 1)	Ref_L1[0] = Temp2_Ref_L1[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_L1_X_dn;

		if(profState_R2_Z_up == 1 || profState_R2_Z_up == 2 || (profState_R2_Z_up == 3 && profState_R2_Z_dn == 1)) Ref_R2[0] = Temp_Ref_R2[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_R2_X_up ;
		else if(profState_R2_Z_up == 3 && profState_R2_Z_dn != 1)	Ref_R2[0] = Temp2_Ref_R2[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_R2_X_dn;

		//_1if(profState_L3_Z_up == 1 || profState_L3_Z_up == 2 || (profState_L3_Z_up == 3 && profState_L3_Z_dn == 1)) Ref_L3[0] = Temp_Ref_L3[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_L3_X_up ;
		//_1else if(profState_L3_Z_up == 3 && profState_L3_Z_dn != 1)	Ref_L3[0] = Temp2_Ref_L3[0] + (Temp_Ls_LF + Old_Ls_LF)*profResult_L3_X_dn;
	}
	else if(RF_stop_flag == 1 || RF_stop_flag == 2){
		if(profState_L1_Z_up == 1 || profState_L1_Z_up == 2 || (profState_L1_Z_up == 3 && profState_L1_Z_dn == 1)) Ref_L1[0] = Temp_Ref_L1[0] + Old_Ls_LF*profResult_L1_X_up ;
		else if(profState_L1_Z_up == 3 && profState_L1_Z_dn != 1)	Ref_L1[0] = Temp2_Ref_L1[0] + Old_Ls_LF*profResult_L1_X_dn;

		if(profState_R2_Z_up == 1 || profState_R2_Z_up == 2 || (profState_R2_Z_up == 3 && profState_R2_Z_dn == 1)) Ref_R2[0] = Temp_Ref_R2[0] + Old_Ls_LF*profResult_R2_X_up ;
		else if(profState_R2_Z_up == 3 && profState_R2_Z_dn != 1)	Ref_R2[0] = Temp2_Ref_R2[0] + Old_Ls_LF*profResult_R2_X_dn;

		//_1if(profState_L3_Z_up == 1 || profState_L3_Z_up == 2 || (profState_L3_Z_up == 3 && profState_L3_Z_dn == 1)) Ref_L3[0] = Temp_Ref_L3[0] + Old_Ls_LF*profResult_L3_X_up ;
		//_1else if(profState_L3_Z_up == 3 && profState_L3_Z_dn != 1)	Ref_L3[0] = Temp2_Ref_L3[0] + Old_Ls_LF*profResult_L3_X_dn;
	}

	if(profState_L1_Z_up == 1 || profState_L1_Z_up == 2 || (profState_L1_Z_up == 3 && profState_L1_Z_dn == 1)) Ref_L1[1] = Temp_Ref_L1[1] + Old_Lss_LF*profResult_L1_X_up ;
	else if(profState_L1_Z_up == 3 && profState_L1_Z_dn != 1)	Ref_L1[1] = Temp2_Ref_L1[1] + Old_Lss_LF*profResult_L1_X_dn;

	if(profState_R2_Z_up == 1 || profState_R2_Z_up == 2 || (profState_R2_Z_up == 3 && profState_R2_Z_dn == 1)) Ref_R2[1] = Temp_Ref_R2[1] + Old_Lss_LF*profResult_R2_X_up ;
	else if(profState_R2_Z_up == 3 && profState_R2_Z_dn != 1)	Ref_R2[1] = Temp2_Ref_R2[1] + Old_Lss_LF*profResult_R2_X_dn;

	//_1if(profState_L3_Z_up == 1 || profState_L3_Z_up == 2 || (profState_L3_Z_up == 3 && profState_L3_Z_dn == 1)) Ref_L3[1] = Temp_Ref_L3[1] + Old_Lss_LF*profResult_L3_X_up ;
	//_1else if(profState_L3_Z_up == 3 && profState_L3_Z_dn != 1)	Ref_L3[1] = Temp2_Ref_L3[1] + Old_Lss_LF*profResult_L3_X_dn;

	
	if(profState_L1_Z_up == 1 || profState_L1_Z_up == 2 || (profState_L1_Z_up == 3 && profState_L1_Z_dn == 1)) Ref_th[3] = Temp_Ref_th[3] + Old_Rot_Ang_LF*profResult_L1_X_up ;
	else if(profState_L1_Z_up == 3 && profState_L1_Z_dn != 1)	Ref_th[3] = Temp2_Ref_th[3] + Old_Rot_Ang_LF*profResult_L1_X_dn;

	if(profState_R2_Z_up == 1 || profState_R2_Z_up == 2 || (profState_R2_Z_up == 3 && profState_R2_Z_dn == 1)) Ref_th[4] = Temp_Ref_th[4] + Old_Rot_Ang_LF*profResult_R2_X_up ;
	else if(profState_R2_Z_up == 3 && profState_R2_Z_dn != 1)	Ref_th[4] = Temp2_Ref_th[4] + Old_Rot_Ang_LF*profResult_R2_X_dn;

	//_1if(profState_L3_Z_up == 1 || profState_L3_Z_up == 2 || (profState_L3_Z_up == 3 && profState_L3_Z_dn == 1)) Ref_th[5] = Temp_Ref_th[5] + Old_Rot_Ang_LF*profResult_L3_X_up ;
	//_1else if(profState_L3_Z_up == 3 && profState_L3_Z_dn != 1)	Ref_th[5] = Temp2_Ref_th[5] + Old_Rot_Ang_LF*profResult_L3_X_dn;



	
	if(profState_L1_Z_dn == 1)  {  // ??? ?? ?? ?? //
		Temp2_Ref_L1[0] = Ref_L1[0]; 
		Temp2_Ref_L1[1] = Ref_L1[1]; 
		Temp2_Ref_th[3] = Ref_th[3]; 
	}
	else if(profState_L1_Z_dn == 3){
		Temp_Ref_L1[0] = Ref_L1[0]; 
		Temp_Ref_L1[1] = Ref_L1[1]; 
		Temp_Ref_th[3] = Ref_th[3];
	}

	if(profState_R2_Z_dn == 1)  {
		Temp2_Ref_R2[0] = Ref_R2[0]; 
		Temp2_Ref_R2[1] = Ref_R2[1]; 
		Temp2_Ref_th[4] = Ref_th[4];
	}
	else if(profState_R2_Z_dn == 3){
		Temp_Ref_R2[0] = Ref_R2[0]; 
		Temp_Ref_R2[1] = Ref_R2[1]; 
		Temp_Ref_th[4] = Ref_th[4];
	}

	//_1if(profState_L3_Z_dn == 1)  {
	//_1	Temp2_Ref_L3[0] = Ref_L3[0]; 
	//_1	Temp2_Ref_L3[1] = Ref_L3[1]; 
	//_1	Temp2_Ref_th[5] = Ref_th[5];
	//_1}
	//_1else if(profState_L3_Z_dn == 3){
	//_1	Temp_Ref_L3[0] = Ref_L3[0]; 
	//_1	Temp_Ref_L3[1] = Ref_L3[1]; 
	//_1	Temp_Ref_th[5] = Ref_th[5];
	//_1}

	
	// ??? X??(????) Y??(???) ?? ?? ?? ??(0 ~ 1) //  
	if(Old_Ls_BC >= 0){  // 2013_07_19
		prof_cos_linear(&profState_BC_X,&profResult_BC_X,local_Index,Ts/2 + next_start_time_BC,2*Ts + 2*Old_Td_BC + Old_T_off_BC,0,shape_factor_F);   // 전진방향 궤적
		prof_cos_linear(&profState_ZMP_X,&profResult_ZMP_X,local_Index,Ts/2 + next_start_time_BC,2*Ts + 2*Old_Td_BC + Old_T_off_BC,0,20);   // 전진방향 ZMP 궤적
	}
	else{
		prof_cos_linear(&profState_BC_X,&profResult_BC_X,local_Index,2*Ts/2 + next_start_time_BC,2*Ts + 2*Old_Td_BC + Old_T_off_BC,0,shape_factor_F);   // 전진방향 궤적
		prof_cos_linear(&profState_ZMP_X,&profResult_ZMP_X,local_Index,2*Ts/2 + next_start_time_BC,2*Ts + 2*Old_Td_BC + Old_T_off_BC,0,20);   // 전진방향 ZMP 궤적
	}

	Ref_BC[0] = Old_BC_X + Old_Ls_BC*profResult_BC_X;
	Ref_ZMP[0] = Old_BC_X + Old_Ls_BC*profResult_ZMP_X;
	
	if(Old_Lss_BC <= 0 ){ // ??? ?? ? Y ?? ??
		prof_cos_linear(&profState_BC_Y,&profResult_BC_Y,local_Index,Ts/2 + next_start_time_BC_2,2*Ts + 2*Old_Td_BC + Old_T_off_BC,0,shape_factor_S);
		
		Ref_BC[1] = Old_BC_Y + Old_Lss_BC*profResult_BC_Y;
	}
	else if(Old_Lss_BC > 0 ){
		prof_cos_linear(&profState_BC_Y,&profResult_BC_Y,local_Index,Ts/2 + 2*Ts + 2*Old_Td_BC + Old_T_off_BC + next_start_time_BC_2,2*Ts + 2*Old_Td_BC + Old_T_off_BC,0,shape_factor_S);
		
		Ref_BC[1] = Old_BC_Y + Old_Lss_BC*profResult_BC_Y;
	}

	if(Old_Rot_Ang_BC <= 0){ // ?? ?? ? ?? ?? ??
		prof_cos_linear(&profState_BC_th,&profResult_BC_th,local_Index,Ts/2 + next_start_time_BC_th,2*Ts + 2*Old_Td_BC + Old_T_off_BC,0,shape_factor_S);
		Ref_th[6] = Old_BC_th + Old_Rot_Ang_BC*profResult_BC_th;
	}
	else{
		prof_cos_linear(&profState_BC_th,&profResult_BC_th,local_Index,Ts/2 + 2*Ts + 2*Old_Td_BC + Old_T_off_BC + next_start_time_BC_th,2*Ts + 2*Old_Td_BC + Old_T_off_BC,0,shape_factor_S);
		Ref_th[6] = Old_BC_th + Old_Rot_Ang_BC*profResult_BC_th;
	}
	

	
	if(BC_SWAY_CNT == 1)	{
		prof_delay_cos_delay(&profState_BC_Y_SWAY,&profResult_BC_Y_SWAY,local_Index,0,Ts/2+(Ts/2+Td),0,0,0); // ??? ?? SWAY ?? ??		
		Del_BC_Y = Old_Ab*profResult_BC_Y_SWAY;  // 2013_06_23
		prof_delay_cos_delay(&profState_ZMP_Y,&profResult_ZMP_Y,local_Index,0,Ts/2+(Ts/2+Td),0,10,10/*Ts + 0.9*Old_Td_BC_2*/); // ??? ZMP SWAY ?? ??
		Del_ZMP_Y = A_ZMP_Y*profResult_ZMP_Y;
	}
	else if(BC_SWAY_CNT == 2 && BC_SWAY_Stop_Flag == 0){
		prof_delay_cos_delay(&profState_BC_Y_SWAY,&profResult_BC_Y_SWAY,local_Index,next_start_time_BC_3,Ts+Ts/2 + Old_Td_BC_2 + Old_Td_BC_2/2 + 0*Old_Td_BC_3 + Old_T_off_BC_2,0,0,60); 
		Del_BC_Y = Old_Del_BC_Y - (Old_Ab+Old_Ab_2)*profResult_BC_Y_SWAY;  // 2013_06_23
		prof_delay_cos_delay(&profState_ZMP_Y,&profResult_ZMP_Y,local_Index,next_start_time_BC_3,Ts+Ts/2 + Old_Td_BC_2 + Old_Td_BC_2/2 + 0*Old_Td_BC_3 + Old_T_off_BC_2,0,0/*Ts+0.9*Old_Td_BC_2*/,60/*Ts+0.9*Old_Td_BC_2*/); 
		Del_ZMP_Y = Old_Del_ZMP_Y - 2*A_ZMP_Y*profResult_ZMP_Y;
	}
	else if(BC_SWAY_CNT%2 == 0 && BC_SWAY_Stop_Flag == 0){
		prof_delay_cos_delay(&profState_BC_Y_SWAY,&profResult_BC_Y_SWAY,local_Index,next_start_time_BC_3,2*Ts + 1*Old_Td_BC_2 + Old_Td_BC_2/2 + Old_Td_BC_3/2 + Old_T_off_BC_2,0,40,40); 
		Del_BC_Y = Old_Del_BC_Y - (Old_Ab+Old_Ab_2)*profResult_BC_Y_SWAY;  // 2013_06_23
		prof_delay_cos_delay(&profState_ZMP_Y,&profResult_ZMP_Y,local_Index,next_start_time_BC_3,2*Ts + 1*Old_Td_BC_2 + Old_Td_BC_2/2 + Old_Td_BC_3/2 + Old_T_off_BC_2,0,40/*Ts+0.9*Old_Td_BC_2*/,40/*Ts+0.9*Old_Td_BC_2*/); 
		Del_ZMP_Y = Old_Del_ZMP_Y - 2*A_ZMP_Y*profResult_ZMP_Y;
	}
	else if(BC_SWAY_CNT%2 == 1 && BC_SWAY_Stop_Flag == 0){
		prof_delay_cos_delay(&profState_BC_Y_SWAY,&profResult_BC_Y_SWAY,local_Index,next_start_time_BC_3,2*Ts + 1*Old_Td_BC_2 + Old_Td_BC_2/2 + Old_Td_BC_3/2 + Old_T_off_BC_2 ,0,40,40); 
		Del_BC_Y = Old_Del_BC_Y + (Old_Ab+Old_Ab_2)*profResult_BC_Y_SWAY;  // 2013_06_23
		prof_delay_cos_delay(&profState_ZMP_Y,&profResult_ZMP_Y,local_Index,next_start_time_BC_3,2*Ts + 1*Old_Td_BC_2 + Old_Td_BC_2/2 + Old_Td_BC_3/2 + Old_T_off_BC_2,0,40/*Ts+0.9*Old_Td_BC_2*/,40/*Ts+0.9*Old_Td_BC_2*/); 
		Del_ZMP_Y = Old_Del_ZMP_Y + 2*A_ZMP_Y*profResult_ZMP_Y;
	}
	else if(BC_SWAY_Stop_Flag == 1){ // 2013_07_18
		prof_delay_cos_delay(&profState_BC_Y_SWAY,&profResult_BC_Y_SWAY,local_Index,next_start_time_BC_3,3*Ts/2 + 3*Old_Td_BC_3/2 + 0*Old_Td_BC_3/2,0,40,0); 
		Del_BC_Y = Old_Del_BC_Y - (Old_Ab+Old_Ab_2)*profResult_BC_Y_SWAY;  // 2013_06_23
		prof_delay_cos_delay(&profState_ZMP_Y,&profResult_ZMP_Y,local_Index,next_start_time_BC_3,3*Ts/2 + 3*Old_Td_BC_3 + 0*Old_Td_BC_3/2,0,40/*Ts+0.9*Old_Td_BC_2*/,0 /*Ts+0.9*Old_Td_BC_2*/); 
		Del_ZMP_Y = Old_Del_ZMP_Y - 2*A_ZMP_Y*profResult_ZMP_Y;
	}
	else if(BC_SWAY_Stop_Flag == 2){ // 2013_07_18
		prof_delay_cos_delay(&profState_BC_Y_SWAY,&profResult_BC_Y_SWAY,local_Index,next_start_time_BC_3,3*Ts/2 + 3*Old_Td_BC_3/2 + 0*Old_Td_BC_3/2,0,40,0); 
		Del_BC_Y = Old_Del_BC_Y + (Old_Ab+Old_Ab_2)*profResult_BC_Y_SWAY;  // 2013_06_23
		prof_delay_cos_delay(&profState_ZMP_Y,&profResult_ZMP_Y,local_Index,next_start_time_BC_3,3*Ts/2 + 3*Old_Td_BC_3/2 + 0*Old_Td_BC_3/2,0,40/*Ts+0.9*Old_Td_BC_2*/,0/*Ts+0.9*Old_Td_BC_2*/); 
		Del_ZMP_Y = Old_Del_ZMP_Y + 2*A_ZMP_Y*profResult_ZMP_Y;
	}
	else if(BC_SWAY_Stop_Flag == 3){ // 2013_07_18
		prof_delay_cos_delay(&profState_BC_Y_SWAY,&profResult_BC_Y_SWAY,local_Index,next_start_time_BC_3,Ts,0,0,0); 
		Del_BC_Y = Old_Del_BC_Y + Old_Ab_2*profResult_BC_Y_SWAY;  // 2013_06_23
		prof_delay_cos_delay(&profState_ZMP_Y,&profResult_ZMP_Y,local_Index,next_start_time_BC_3,Ts + 0*Old_Td_BC_3 + 0*Old_Td_BC_3/2,0,0/*Ts+0.9*Old_Td_BC_2*/,0 /*Ts+0.9*Old_Td_BC_2*/); 
		Del_ZMP_Y = Old_Del_ZMP_Y + A_ZMP_Y*profResult_ZMP_Y;
	}
	else if(BC_SWAY_Stop_Flag == 4){ // 2013_07_18
		prof_delay_cos_delay(&profState_BC_Y_SWAY,&profResult_BC_Y_SWAY,local_Index,next_start_time_BC_3,Ts,0,0,0); 
		Del_BC_Y = Old_Del_BC_Y - Old_Ab_2*profResult_BC_Y_SWAY;  // 2013_06_23
		prof_delay_cos_delay(&profState_ZMP_Y,&profResult_ZMP_Y,local_Index,next_start_time_BC_3,Ts + 0*Old_Td_BC_3 + 0*Old_Td_BC_3/2,0,0/*Ts+0.9*Old_Td_BC_2*/,0/*Ts+0.9*Old_Td_BC_2*/); 
		Del_ZMP_Y = Old_Del_ZMP_Y - A_ZMP_Y*profResult_ZMP_Y;
	}
	
	

	
	if(profState_BC_X == 3 && LF_stop_flag == 0 && RF_stop_flag == 0) {
		Old_BC_X = Ref_BC[0];
		
		if(Old_Ls_BC >= 0) next_start_time_BC = local_Index - Ts/2;   // 2013_07_19
		else next_start_time_BC = local_Index - 2*Ts/2;               // 2013_07_19
		
		if(*Step_CNT%2 ==1) {

			Old_Td_BC = Td;    
			Old_Ls_BC = Ls;
			Old_Lss_BC = Lss; 
			Old_Rot_Ang_BC = Rot_Ang;

	
			if(2*Ts+2*Td < Ts)  Old_T_off_BC = Ts - (2*Ts+2*Td); // ?? 3? ??? ???? ?? ?? ??? ??
			else Old_T_off_BC = 0;

			
		}

	}

	if(Old_Lss_BC <= 0 && profState_BC_Y == 3 && LF_stop_flag == 0 && RF_stop_flag == 0) { // Lss < 0 ??, ? ????? ???

		Old_BC_Y = Ref_BC[1];

		next_start_time_BC_2 = local_Index - Ts/2 + 2*Ts + 2*Old_Td_BC + Old_T_off_BC;

				
	}
	else if(Old_Lss_BC > 0 && profState_BC_Y == 3 && LF_stop_flag == 0 && RF_stop_flag == 0) { // Lss > 0 ??, ? ???? ???  

		Old_BC_Y = Ref_BC[1];

		next_start_time_BC_2 = local_Index - Ts/2;
		
	}

	if(Old_Rot_Ang_BC <= 0 && profState_BC_th == 3 && LF_stop_flag == 0 && RF_stop_flag == 0) { // Rot_Ang < 0 ??, ? ????? ???

		Old_BC_th = Ref_th[6];

		next_start_time_BC_th = local_Index - Ts/2 + 2*Ts + 2*Old_Td_BC + Old_T_off_BC;
		
	}
	else if(Old_Rot_Ang_BC > 0 && profState_BC_th == 3 && LF_stop_flag == 0 && RF_stop_flag == 0) { // Rot_Ang > 0 ??, ? ???? ???  

		Old_BC_th = Ref_th[6];

		next_start_time_BC_th = local_Index - Ts/2;
	}

	if(profState_BC_Y_SWAY == 3 && BC_SWAY_Stop_Flag == 0){
		if(2*Ts+2*Td < Ts && profState_L2_Z_up == 3)  Old_T_off_BC_2 = Ts - (2*Ts+2*Td); // ?? 3? ??? ???? ?? ?? ??? ??
		else if(2*Ts+2*Td >= Ts && profState_L2_Z_up == 3) Old_T_off_BC_2 = 0;

		Old_Td_BC_3 = Old_Td_BC_2;
		if(BC_SWAY_CNT%2 == 1)	Old_Td_BC_2 = Td;
		
		Old_Ab_2 = Old_Ab;// 2013_06_23
		Old_Ab = Ab; // 2013_06_23

		Old_Del_BC_Y = Del_BC_Y; Old_Del_ZMP_Y = Del_ZMP_Y;
		next_start_time_BC_3 = local_Index;
		BC_SWAY_CNT++;

		if(BC_SWAY_CNT > 2 && LF_stop_flag != 0 /*&& profState_L2_Z_dn == 3*/)	BC_SWAY_Stop_Flag = 1; 
		else if(BC_SWAY_CNT > 2 && RF_stop_flag != 0 /*&& profState_R2_Z_dn == 3*/)	BC_SWAY_Stop_Flag = 2; 
	}
	else if(BC_SWAY_CNT > 2 && BC_SWAY_Stop_Flag == 1 && profState_BC_Y_SWAY == 3)	{   // 2013_07_18
		Old_Del_BC_Y = Del_BC_Y; Old_Del_ZMP_Y = Del_ZMP_Y;
		next_start_time_BC_3 = local_Index;

		BC_SWAY_Stop_Flag = 3; 
	}
	else if(BC_SWAY_CNT > 2 && BC_SWAY_Stop_Flag == 2 && profState_BC_Y_SWAY == 3)	{   // 2013_07_18
		Old_Del_BC_Y = Del_BC_Y; Old_Del_ZMP_Y = Del_ZMP_Y; 
		next_start_time_BC_3 = local_Index;

		BC_SWAY_Stop_Flag = 4; 
	}

	//Ref_ZMP[1] = Ref_BC[1] + Del_ZMP_Y; // ZMP ??? ??? ??? + ZMP ??? SWAY 
	//Ref_BC[1] += Del_BC_Y;   // ?? ??? ??? ??? + ?? ??? SWAY 
	

	if(WP_Flow_Control_Flag != 1){

		if(Go_flag == 0 && local_Index == 0)  { // ?? ???? go flag ? 0 ?? local_index = 0, Step_CNT = 0
			local_Index = 0;
			Temp_local_Index = 0;
			Real_Stop_Flag = 0;
			*Step_CNT = 0; BC_SWAY_CNT = 0;
			LF_stop_flag = 0;
			RF_stop_flag = 0;
			BC_SWAY_Stop_Flag = 0;
			next_start_time_R1 = (Ts/2+Td);
			next_start_time_R2 = (Ts/2+Td);
			//_1next_start_time_R3 = 0;
			next_start_time_L1 = (Ts/2+Td);
			next_start_time_L2 = (Ts/2+Td);
			//_1next_start_time_L3 = 0;
			next_start_time_BC = (Ts/2+Td);	next_start_time_BC_2 = (Ts/2+Td); next_start_time_BC_3 = (Ts/2+Td); next_start_time_BC_th = (Ts/2+Td);
		
			Old_Hs_RF = 0.;
			Old_Hs_LF = 0.;
			Old_Ls_RF = 0.; Old_Lss_RF = 0.;
			Old_Ls_LF = 0.; Old_Lss_LF = 0.;
			Old_Ls_BC = 0.;
			Old_Lss_BC = 0.; 
			Old_Td_RF = 0.;
			Old_Td_LF = 0.;
			Old_Td_BC = 0.; Old_Td_BC_2 = 0.; Old_Td_BC_3 = 0.;
			Temp_Ls_RF = 0.; Temp_Lss_RF = 0.;
			Temp_Ls_LF = 0.; Temp_Lss_LF = 0.;
			Old_BC_X = 0.;
			Old_BC_Y = 0.;
			Old_BC_th = 0.;
			Del_BC_Y = 0.; Old_Del_BC_Y = 0.;
			Del_ZMP_Y = 0.; Old_Del_ZMP_Y = 0.;
			Ref_ZMP[0] = 0.; Ref_ZMP[1] = 0.;

			T_off = 0; Old_T_off_BC = 0; Old_T_off_BC_2 = 0; 

			Old_profState_R1_Z_up = 0; Old_profState_L1_Z_up = 0;

			//LF_start_flag = 0;   // ?? 2012.09.17


			if(Foot_Location_Flag){   // ?? 2013.04.30

				if(temp_X[RF_1] != 0) temp_X[RF_1] = temp_X[RF_1] - Temp_Ref_R1[0];// ?? 2013.04.30
				if(temp_X[RF_2] != 0) temp_X[RF_2] = temp_X[RF_2] - Temp_Ref_R2[0];// ?? 2013.04.30
				if(temp_X[LF_1] != 0) temp_X[LF_1] = temp_X[LF_1] - Temp_Ref_L1[0];// ?? 2013.04.30
				if(temp_X[LF_2] != 0) temp_X[LF_2] = temp_X[LF_2] - Temp_Ref_L2[0];// ?? 2013.04.30
				
				if(LF_start_flag){
					if(temp_Y[RF_1] != 0) temp_Y[RF_1] = temp_Y[RF_1] + Temp_Ref_R1[1];// ?? 2013.04.30
					if(temp_Y[RF_2] != 0) temp_Y[RF_2] = temp_Y[RF_2] + Temp_Ref_R2[1];// ?? 2013.04.30
					if(temp_Y[LF_1] != 0) temp_Y[LF_1] = temp_Y[LF_1] + Temp_Ref_L1[1];// ?? 2013.04.30
					if(temp_Y[LF_2] != 0) temp_Y[LF_2] = temp_Y[LF_2] + Temp_Ref_L2[1];// ?? 2013.04.30
				}
				else{ 	
					if(temp_Y[RF_1] != 0) temp_Y[RF_1] = temp_Y[RF_1] - Temp_Ref_R1[1];// ?? 2013.04.30
					if(temp_Y[RF_2] != 0) temp_Y[RF_2] = temp_Y[RF_2] - Temp_Ref_R2[1];// ?? 2013.04.30
					if(temp_Y[LF_1] != 0) temp_Y[LF_1] = temp_Y[LF_1] - Temp_Ref_L1[1];// ?? 2013.04.30
					if(temp_Y[LF_2] != 0) temp_Y[LF_2] = temp_Y[LF_2] - Temp_Ref_L2[1];// ?? 2013.04.30
				}

				if(LF_start_flag){
					if(temp_th[RF_1] != 0) temp_th[RF_1] = temp_th[RF_1] + Temp_Ref_th[RF_1];// ?? 2013.04.30
					if(temp_th[RF_2] != 0) temp_th[RF_2] = temp_th[RF_2] + Temp_Ref_th[RF_2];// ?? 2013.04.30
					if(temp_th[LF_1] != 0) temp_th[LF_1] = temp_th[LF_1] + Temp_Ref_th[LF_1];// ?? 2013.04.30
					if(temp_th[LF_2] != 0) temp_th[LF_2] = temp_th[LF_2] + Temp_Ref_th[LF_2];// ?? 2013.04.30
				}
				else {
					if(temp_th[RF_1] != 0) temp_th[RF_1] = temp_th[RF_1] - Temp_Ref_th[RF_1];// ?? 2013.04.30
					if(temp_th[RF_2] != 0) temp_th[RF_2] = temp_th[RF_2] - Temp_Ref_th[RF_2];// ?? 2013.04.30
					if(temp_th[LF_1] != 0) temp_th[LF_1] = temp_th[LF_1] - Temp_Ref_th[LF_1];// ?? 2013.04.30
					if(temp_th[LF_2] != 0) temp_th[LF_2] = temp_th[LF_2] - Temp_Ref_th[LF_2];// ?? 2013.04.30
				}

				Foot_Location_Flag = 0;
			}


			Ref_R1[0] = 0.; Ref_R1[1] = 0.; //Ref_R1[2] = 0.;    // ?? 2012.09.17
			Ref_L1[0] = 0.; Ref_L1[1] = 0.; //Ref_L1[2] = 0.;    // ?? 2012.09.17
			Ref_R2[0] = 0.; Ref_R2[1] = 0.; //Ref_R2[2] = 0.;    // ?? 2012.09.17
			Ref_L2[0] = 0.; Ref_L2[1] = 0.; //Ref_L2[2] = 0.;    // ?? 2012.09.17
			//_1Ref_R3[0] = 0.; Ref_R3[1] = 0.; //Ref_R3[2] = 0.;    // ?? 2012.09.17
			//_1Ref_L3[0] = 0.; Ref_L3[1] = 0.; //Ref_L3[2] = 0.;    // ?? 2012.09.17
			Ref_BC[0] = 0.; Ref_BC[1] = 0.; //Ref_BC[2] = 0.;    // ?? 2012.09.17
			Ref_th[0] = 0.; Ref_th[1] = 0.; Ref_th[2] = 0.; Ref_th[3] = 0.; Ref_th[4] = 0.; Ref_th[5] = 0.; Ref_th[6] = 0.;

			Temp_Ref_R1[0] = 0.; Temp_Ref_R1[1] = 0.; Temp_Ref_R1[2] = 0.; Temp2_Ref_R1[0] = 0.; Temp2_Ref_R1[1] = 0.; Temp2_Ref_R1[2] = 0.;
			Temp_Ref_R2[0] = 0.; Temp_Ref_R2[1] = 0.; Temp_Ref_R2[2] = 0.; Temp2_Ref_R2[0] = 0.; Temp2_Ref_R2[1] = 0.; Temp2_Ref_R2[2] = 0.;
			//_1Temp_Ref_R3[0] = 0.; Temp_Ref_R3[1] = 0.; Temp_Ref_R3[2] = 0.; Temp2_Ref_R3[0] = 0.; Temp2_Ref_R3[1] = 0.; Temp2_Ref_R3[2] = 0.;
			Temp_Ref_L1[0] = 0.; Temp_Ref_L1[1] = 0.; Temp_Ref_L1[2] = 0.; Temp2_Ref_L1[0] = 0.; Temp2_Ref_L1[1] = 0.; Temp2_Ref_L1[2] = 0.;
			Temp_Ref_L2[0] = 0.; Temp_Ref_L2[1] = 0.; Temp_Ref_L2[2] = 0.; Temp2_Ref_L2[0] = 0.; Temp2_Ref_L2[1] = 0.; Temp2_Ref_L2[2] = 0.;
			//_1Temp_Ref_L3[0] = 0.; Temp_Ref_L3[1] = 0.; Temp_Ref_L3[2] = 0.; Temp2_Ref_L3[0] = 0.; Temp2_Ref_L3[1] = 0.; Temp2_Ref_L3[2] = 0.;
			Temp_Ref_th[0] = 0.; Temp_Ref_th[1] = 0.; Temp_Ref_th[2] = 0.; Temp_Ref_th[3] = 0.; Temp_Ref_th[4] = 0.; Temp_Ref_th[5] = 0.;
			Temp2_Ref_th[0] = 0.; Temp2_Ref_th[1] = 0.; Temp2_Ref_th[2] = 0.; Temp2_Ref_th[3] = 0.; Temp2_Ref_th[4] = 0.; Temp2_Ref_th[5] = 0.;
		
		
		}
		else if(Go_flag == 0 && *Step_CNT != 0 && Real_Stop_Flag == 0) { // ???? ?? go flag ? 0 ?? ?? local_index ??????
			local_Index ++ ;   

			if(Old_Lss_BC == 0 && Old_Rot_Ang_BC == 0){   // Side Step ? ??? ???                                                                   
				if(profState_R1_Z_dn == 3 && RF_stop_flag == 0 && LF_stop_flag == 0) RF_stop_flag = 1; // ?? ??? ?????
				else if(profState_L1_Z_dn == 3 && RF_stop_flag == 0 && LF_stop_flag == 0) LF_stop_flag = 1; // ??? ??? ?????

				if(LF_stop_flag == 1 && profState_L2_Z_up != 0) LF_stop_flag = 2; // ?? ??? ??? ?? ??
				else if(RF_stop_flag == 1 && profState_R2_Z_up != 0) RF_stop_flag = 2; // ??? ??? ??? ?? ??

				if(profState_R2_Z_dn == 3  && RF_stop_flag == 2 && profState_BC_Y_SWAY == 3) Real_Stop_Flag = 1; // ??? ??? ??? ??? Real_Stop_Flag ? Enable ?? ?? ??
				else if(profState_L2_Z_dn == 3  && LF_stop_flag == 2 && profState_BC_Y_SWAY == 3) Real_Stop_Flag = 1; // ?? ??? ??? ??? Real_Stop_Flag ? Enable ?? ?? ??
			}
			else if(Old_Lss_BC < 0 || Old_Rot_Ang_BC < 0){   // Side Step ? ???(??? ?? ????? ??? ???.)
				if(profState_R1_Z_dn == 3 && RF_stop_flag == 0 && LF_stop_flag == 0) RF_stop_flag = 1; // ?? ??? ?????
				
				if(RF_stop_flag == 1 && profState_R2_Z_up != 0) RF_stop_flag = 2; // ?? ??? ??? ?? ??
				
				if(profState_R2_Z_dn == 3  && RF_stop_flag == 2 && profState_BC_Y_SWAY == 3) Real_Stop_Flag = 1; // ?? ??? ??? ??? Real_Stop_Flag ? Enable ?? ?? ??
			}
			else if(Old_Lss_BC > 0 || Old_Rot_Ang_BC > 0){   // Side Step ? ???(?? ?? ????? ???? ???.)
				if(profState_L1_Z_dn == 3 && RF_stop_flag == 0 && LF_stop_flag == 0) LF_stop_flag = 1; // ??? ??? ?????

				if(LF_stop_flag == 1 && profState_L2_Z_up != 0) LF_stop_flag = 2; // ??? ??? ??? ?? ??

				if(profState_L2_Z_dn == 3  && LF_stop_flag == 2 && profState_BC_Y_SWAY == 3) Real_Stop_Flag = 1; // ??? ??? ??? ??? Real_Stop_Flag ? Enable ?? ?? ??
				
			}


			//if(RF_stop_flag == 2 || LF_stop_flag == 2) {   // 2013_01_07 ?????.:???? ? x,y ?? ??? ??. // ??? ?? !!! ??? ???? Z ?? ??? ???? ??//
			//	for(i = 0; i<6; i++) Early_Landing_Flag[i] = 0;
			//}
			
		}
		else if(Go_flag == 0 && *Step_CNT != 0 && Real_Stop_Flag == 1){
			local_Index = 0;
			Real_Stop_Flag = 0;
			*Step_CNT = 0;
			LF_stop_flag = 0;
			RF_stop_flag = 0;
		}
		else {
			if(LF_start_flag && Go_flag == 1 && local_Index == 0){   // ?? 2013.04.30

				if(temp_X[LF_1] != 0 && temp_X[RF_1] == 0) {
					//printf("temp_X[RF1] = %f,	temp_X[LF1] = %f \n",temp_X[RF1], temp_X[LF1]);
					temp_X[RF_1] = temp_X[LF_1]; EL_Del_xy_CNT[RF_1] = 0;}

				if(temp_X[LF_2] != 0 && temp_X[RF_2] == 0) {temp_X[RF_2] = temp_X[LF_2]; EL_Del_xy_CNT[RF_2] = 0;}
				else if(temp_X[LF_2] == 0 && temp_X[RF_2] != 0) {temp_X[LF_2] = temp_X[RF_2]; EL_Del_xy_CNT[LF_2] = 0;}



				if(temp_Y[LF_1] != 0 && temp_Y[RF_1] == 0) {
					//printf("temp_Y[RF1] = %f,	temp_Y[LF1] = %f \n",temp_Y[RF1], temp_Y[LF1]);
					temp_Y[RF_1] = temp_Y[LF_1]; EL_Del_xy_CNT[RF_1] = 0;}

				if(temp_Y[LF_2] != 0 && temp_Y[RF_2] == 0) {temp_Y[RF_2] = temp_Y[LF_2]; EL_Del_xy_CNT[RF_2] = 0;}
				else if(temp_Y[LF_2] == 0 && temp_Y[RF_2] != 0) {temp_Y[LF_2] = temp_Y[RF_2]; EL_Del_xy_CNT[LF_2] = 0;}



				if(temp_th[LF_1] != 0 && temp_th[RF_1] == 0) {
					//printf("temp_th[RF1] = %f,	temp_th[LF1] = %f \n",temp_th[RF1], temp_th[LF1]);
					temp_th[RF_1] = temp_th[LF_1]; EL_Del_xy_CNT[RF_1] = 0;}

				if(temp_th[LF_2] != 0 && temp_th[RF_2] == 0) {temp_th[RF_2] = temp_th[LF_2]; EL_Del_xy_CNT[RF_2] = 0;}
				else if(temp_th[LF_2] == 0 && temp_th[RF_2] != 0) {temp_th[LF_2] = temp_th[RF_2]; EL_Del_xy_CNT[LF_2] = 0;}			
			}
			
			local_Index++;  // Go_flag ? 1 ??? local_index ?? ?? 
		}

	} // End of if(WP_Flow_Control_Flag != 1){

	
	// ?? ???? ??? ?? //
	if(Old_profState_L2_Z_dn == 2 && profState_L2_Z_dn == 3 && RF_stop_flag != 2 && LF_stop_flag != 2) *Change_flag = 1;   // RF ??? ?? ?? ? ??? ??? ?
	else *Change_flag = 0;

	Old_profState_L2_Z_dn = profState_L2_Z_dn;


	// ? ?? ?? ??? ?? ?? ??? ???? ?? //
	if((RF_stop_flag == 0 ||  RF_stop_flag == 1) && (LF_stop_flag == 0 || LF_stop_flag == 1)){
		if(profState_R1_Z_dn == 3) {
			Old_Td_LF = Td;
			if(2*Ts+2*Td < Ts)  T_off = Ts - (2*Ts+2*Td); // ?? 3? ??? ???? ?? ?? ??? ??
			else T_off = 0;
			
			Temp_local_Index = local_Index - 1;
			next_start_time_R1 = Temp_local_Index + 3*Ts+4*Old_Td_RF + 2*T_off;
		}
		else if(profState_R2_Z_dn == 3) next_start_time_R2 = Temp_local_Index + 3*Ts+4*Old_Td_LF + 2*T_off;
		//_1else if(profState_R3_Z_dn == 3) next_start_time_R3 = Temp_local_Index + 3*Ts+4*Old_Td_RF + 2*T_off;
		else if(profState_L1_Z_dn == 3) next_start_time_L1 = Temp_local_Index + 3*Ts+4*Old_Td_LF + 2*T_off;
		else if(profState_L2_Z_dn == 3) next_start_time_L2 = Temp_local_Index + 3*Ts+4*Old_Td_RF + 2*T_off;
		//_1else if(profState_L3_Z_dn == 3) next_start_time_L3 = Temp_local_Index + 3*Ts+4*Old_Td_LF + 2*T_off;
	}


	
	// ?????? ?? ?? X ?? ?? ?? //  
	if(profState_R1_Z_up == 1) {

		Temp_profState_BC_Y = profState_BC_Y; 

		Temp_profState_BC_th = profState_BC_th; 

		if(LF_stop_flag == 0){
			Temp_Ls_RF = Old_Ls_RF;
			Old_Ls_RF = Ls;

			if(Lss >= 0){
				Old_Lss_RF = Old_Lss_LF;
			}
			else{
				Temp_Lss_RF = Old_Lss_RF;
				Old_Lss_RF = Lss;
			}

		}
		else if(LF_stop_flag == 1 || LF_stop_flag == 2 && Lss > 0){
			Old_Lss_RF = Old_Lss_LF;
		}


		if(LF_stop_flag == 0){

			if(Rot_Ang >= 0){
				Old_Rot_Ang_RF = Old_Rot_Ang_LF;
			}
			else{
				
				Old_Rot_Ang_RF = Rot_Ang;
			}
		}
		else if(LF_stop_flag == 1 || LF_stop_flag == 2 && Rot_Ang > 0){
			Old_Rot_Ang_RF = Old_Rot_Ang_LF;
		}

	}

	if(profState_R1_Z_dn == 3 && LF_stop_flag == 0 ) {

			Temp_Ls_LF = Ls;
			Old_Ls_LF = Ls;

			Temp_Lss_LF = Old_Lss_LF;
			Old_Lss_LF = Lss;

			Old_Rot_Ang_LF = Rot_Ang;

	}

	Ref_ZMP[1] = Ref_BC[1] + Del_ZMP_Y; // ZMP ??? ??? ??? + ZMP ??? SWAY 
	Ref_BC[1] += Del_BC_Y;   // ?? ??? ??? ??? + ?? ??? SWAY 


	// ?? Start ? ??, ???? ?? ??? ?? ????.
	if(LF_start_flag){

		for(i = 0; i<3; i++) {
			SW_Temp_R1[i] = Ref_R1[i];
			Ref_R1[i] = Ref_L1[i];
			Ref_L1[i] = SW_Temp_R1[i];

			SW_Temp_L2[i] = Ref_L2[i];
			Ref_L2[i] = Ref_R2[i];
			Ref_R2[i] = SW_Temp_L2[i];

			//_1SW_Temp_R3[i] = Ref_R3[i];
			//_1Ref_R3[i] = Ref_L3[i];
			//_1Ref_L3[i] = SW_Temp_R3[i];

			SW_Temp_th = Ref_th[i];
			Ref_th[i] = Ref_th[i+3];
			Ref_th[i+3] = SW_Temp_th;
		}

				
		Ref_R1[1] = -Ref_R1[1];
		Ref_R2[1] = -Ref_R2[1];
		//_1Ref_R3[1] = -Ref_R3[1];
		Ref_L1[1] = -Ref_L1[1];
		Ref_L2[1] = -Ref_L2[1];
		//_1Ref_L3[1] = -Ref_L3[1];

		Ref_BC[1] = -Ref_BC[1];

		Ref_ZMP[1] = -Ref_ZMP[1];

		for(i=0; i<7; i++)	Ref_th[i] = -Ref_th[i];

		SW_Temp_Walking_stage = Walking_stage[0];
		Walking_stage[0] = Walking_stage[3];
		Walking_stage[3] = SW_Temp_Walking_stage;

		SW_Temp_Walking_stage = Walking_stage[1];
		Walking_stage[1] = Walking_stage[4];
		Walking_stage[4] = SW_Temp_Walking_stage;

		//SW_Temp_Walking_stage = Walking_stage[2];
		//Walking_stage[2] = Walking_stage[5];
		//Walking_stage[5] = SW_Temp_Walking_stage;


	}

	// Early Lading Case //  // ?? 2012.09.17
	if(Early_Landing_Flag[0] && Walking_stage[0] == 20 && temp_Z[0] == 0.)	temp_Z[0] = Ref_R1[2];
	
	if(Early_Landing_Flag[1] && Walking_stage[1] == 20 && temp_Z[1] == 0.)	temp_Z[1] = Ref_L2[2]; 
	
	//_1if(Early_Landing_Flag[2] && Walking_stage[2] == 20 && temp_Z[2] == 0.) 	temp_Z[2] = Ref_R3[2]; 
	
	if(Early_Landing_Flag[3] && Walking_stage[3] == 20 && temp_Z[3] == 0.)	temp_Z[3] = Ref_L1[2]; 
	
	if(Early_Landing_Flag[4] && Walking_stage[4] == 20 && temp_Z[4] == 0.)	temp_Z[4] = Ref_R2[2];
	
	//_1if(Early_Landing_Flag[5] && Walking_stage[5] == 20 && temp_Z[5] == 0.)	temp_Z[5] = Ref_L3[2];
	
	
	
	if(/*Early_Landing_Flag[0] == 0 &&*/ Old_Walking_stage[0] == 20 && Walking_stage[0] == 0 && temp_Z[0] == 0.)	temp_Z[0] = Ref_R1[2];
	
	if(/*Early_Landing_Flag[1] == 0 &&*/ Old_Walking_stage[1] == 20 && Walking_stage[1] == 0 && temp_Z[1] == 0.)	temp_Z[1] = Ref_L2[2]; 
	
	//_1if(/*Early_Landing_Flag[2] == 0 &&*/ Old_Walking_stage[2] == 20 && Walking_stage[2] == 0 && temp_Z[2] == 0.)	temp_Z[2] = Ref_R3[2]; 
	
	if(/*Early_Landing_Flag[3] == 0 &&*/ Old_Walking_stage[3] == 20 && Walking_stage[3] == 0 && temp_Z[3] == 0.)	temp_Z[3] = Ref_L1[2]; 
	
	if(/*Early_Landing_Flag[4] == 0 &&*/ Old_Walking_stage[4] == 20 && Walking_stage[4] == 0 && temp_Z[4] == 0.)	temp_Z[4] = Ref_R2[2]; 
	
	//_1if(/*Early_Landing_Flag[5] == 0 &&*/ Old_Walking_stage[5] == 20 && Walking_stage[5] == 0 && temp_Z[5] == 0.)	temp_Z[5] = Ref_L3[2]; 


	
	if(Early_Landing_Flag[0] && Walking_stage[0] != 10)	Ref_R1[2] = temp_Z[0];
	
	if(Early_Landing_Flag[1] && Walking_stage[1] != 10)	Ref_L2[2] = temp_Z[1] ;
	
	//_1if(Early_Landing_Flag[2] && Walking_stage[2] != 10)	Ref_R3[2] = temp_Z[2];
	
	if(Early_Landing_Flag[3] && Walking_stage[3] != 10)	Ref_L1[2] = temp_Z[3] ;
	
	if(Early_Landing_Flag[4] && Walking_stage[4] != 10)	Ref_R2[2] = temp_Z[4] ;
	
	//_1if(Early_Landing_Flag[5] && Walking_stage[5] != 10)	Ref_L3[2] = temp_Z[5];

	
	// Fz[i] = 0. ? ?? ???? ???? //
	
	if(Walking_stage[0] == 10)	{Early_Landing_Flag[0] = 0; }
	
	if(Walking_stage[1] == 10) 	{Early_Landing_Flag[1] = 0; }
	
	//_1if(Walking_stage[2] == 10)	{Early_Landing_Flag[2] = 0; Fz[2] = 0.;}
	
	if(Walking_stage[3] == 10)	{Early_Landing_Flag[3] = 0; }
		
	if(Walking_stage[4] == 10) 	{Early_Landing_Flag[4] = 0; }
		
	//_1if(Walking_stage[5] == 10)	{Early_Landing_Flag[5] = 0; Fz[5] = 0.;}
		
	Ref_R1[2] -= All_Foot_Z + LL_Del_z[0];  
	Ref_L2[2] -= All_Foot_Z + LL_Del_z[1];  
	//_1Ref_R3[2] -= All_Foot_Z + LL_Del_z[2];  

	Ref_L1[2] -= All_Foot_Z + LL_Del_z[3];  
	Ref_R2[2] -= All_Foot_Z + LL_Del_z[4];  
	//_1Ref_L3[2] -= All_Foot_Z + LL_Del_z[5];  
	// ?? 2012.09.17
	
	
	// ??? ? //
	test_var[0] = 10.*Early_Landing_Flag[RF_1];//Walking_stage[RF1] ;//10*RF_stop_flag;
	test_var[1] = 10.*Early_Landing_Flag[LF_1];//Walking_stage[LF2] ;//10*LF_stop_flag;
	test_var[2] = 0.*Walking_stage[LF_1] ;//10*Real_Stop_Flag;
	test_var[3] = 0.*Walking_stage[RF_2] ;//10*Go_flag;
	test_var[4] = 100.;//Temp_Ref_L2[0];
	test_var[5] = 0.;

}

void readZMP(double ORF_x, double ORF_y, double OLF_x, double OLF_y, double ORW_x, double ORW_y, double OLW_x, double OLW_y)
{
	float rf_my = -FTSensor[RFFT].My;
	float lf_my = -FTSensor[LFFT].My;

    if((FTSensor[RFFT].Fz + FTSensor[LFFT].Fz + FTSensor[RWFT].Fz + FTSensor[LWFT].Fz) > 50.){
        X_ZMP = 1000.*(lf_my +FTSensor[LFFT].Fz*OLF_x + rf_my + FTSensor[RFFT].Fz*ORF_x + FTSensor[LWFT].Fz*OLW_x + FTSensor[RWFT].Fz*ORW_x)/(FTSensor[RFFT].Fz + FTSensor[LFFT].Fz + FTSensor[RWFT].Fz + FTSensor[LWFT].Fz);
        Y_ZMP = 1000.*(FTSensor[LFFT].Mx +FTSensor[LFFT].Fz*OLF_y + FTSensor[RFFT].Mx + FTSensor[RFFT].Fz*ORF_y + FTSensor[LWFT].Fz*OLW_y + FTSensor[RWFT].Fz*ORW_y)/(FTSensor[RFFT].Fz + FTSensor[LFFT].Fz + FTSensor[RWFT].Fz + FTSensor[LWFT].Fz);
    }
    
    //printf("X_ZMP = %f mm, Y_ZMP = %f mm \n", X_ZMP,Y_ZMP);
}

void readZMP_biped(double ORF_x, double ORF_y, double OLF_x, double OLF_y)
{
	float rf_my = -FTSensor[RFFT].My;
	float lf_my = -FTSensor[LFFT].My;

    if(FTSensor[RFFT].Fz + FTSensor[LFFT].Fz  > 50.){
        X_ZMP = 1000.*(lf_my +FTSensor[LFFT].Fz*OLF_x + rf_my + FTSensor[RFFT].Fz*ORF_x )/(FTSensor[RFFT].Fz + FTSensor[LFFT].Fz );
        Y_ZMP = 1000.*(FTSensor[LFFT].Mx + FTSensor[LFFT].Fz*OLF_y + FTSensor[RFFT].Mx + FTSensor[RFFT].Fz*ORF_y )/(FTSensor[RFFT].Fz + FTSensor[LFFT].Fz );
    }
    
    //printf("X_ZMP = %f mm, Y_ZMP = %f mm \n", X_ZMP,Y_ZMP);
}

double P4_XZMP_INT_CON_E(double Ref_ZMP, double u, double ZMP, int zero)
{

	const double A[2][2] = {{0.000000000000, 1.000000000000}, {-541.542079620815, -2.355747368821}};
	const double B[2] = {0.000000000000, 1355.077360855948};
	const double C[2] = {10.016205981976, 0.041757243039};
	const double D = -24.019678613851;
	const double Kg[2] = {-0.342445247740, 0.013451087553};
	const double Og[2] = {2.054996748211, -43.585851735140};
	const double ki = 0.202171683253;

	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2], Err_Sum;
	double y;

	Err_Sum = Err_Sum + Del_T*(Ref_ZMP - ZMP);

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {Err_Sum = 0.; x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	y = -Kg[0]*x_new[0] - Kg[1]*x_new[1] + ki*Err_Sum;

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(y > 200.0) y = 200.0;
	else if(y < -200.0) y = -200.0;

	return y ;
}

double P4_YZMP_INT_CON_E(double Ref_ZMP, double u, double ZMP, int zero)
{

const double A[2][2] = {{0.000000000000, 1.000000000000}, {-486.037766751092, -2.355747368821}};
const double B[2] = {0.000000000000, 1221.973493062847};
const double C[2] = {9.032353845319, 0.041757243039};
const double D = -21.660320971988;
const double Kg[2] = {-0.338818942556, 0.014105341602};
const double Og[2] = {2.152114693739, -42.972396350268};
const double ki = 0.212005210974;

	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2], Err_Sum;
	double y;

	Err_Sum = Err_Sum + Del_T*(Ref_ZMP - ZMP);

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {Err_Sum = 0.; x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	y = -Kg[0]*x_new[0] - Kg[1]*x_new[1] + ki*Err_Sum;

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(y > 160.0) y = 160.0;
	else if(y < -160.0) y = -160.0;

	return y ;
}

double P4_XZMP_CON_E(double u, double ZMP, int zero) 
{	

	const double A[2][2] = {{0.000000000000, 1.000000000000}, {-510.822648987597, -2.355747368821}};
const double B[2] = {0.000000000000, 1281.409661495712};
const double C[2] = {9.471682936779, 0.041757243039};
const double D = -22.713867953907;
const double Kg[2] = {-0.379123603938, 0.005965502572};
const double Og[2] = {1.852592125526, -45.570838840587};


	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
	double y;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(y > 100.0) y = 100.0;
	else if(y < -100.0) y = -100.0;

	return y ;
}

double P4_YZMP_CON_E(double u, double ZMP, int zero) 
{
	
const double A[2][2] = {{0.000000000000, 1.000000000000}, {-486.037766751092, -2.355747368821}};
const double B[2] = {0.000000000000, 1221.973493062847};
const double C[2] = {9.032353845319, 0.041757243039};
const double D = -21.660320971988;

const double Kg[2] = {-0.381168470016, 0.005437313222};
const double Og[2] = {2.257634552572, -41.849063153448};
	
	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
	double y;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(y > 80.0) y = 80.0;
	else if(y < -80.0) y = -80.0;

	return y ;
}

double P3H_XZMP_CON_E(double u, double ZMP, int zero) 
{	

const double A[2][2] = {{0.000000000000, 1.000000000000}, {-212.522637175497, -1.217815421806}};
const double B[2] = {0.000000000000, 566.061911346791};
const double C[2] = {4.184109975108, 0.021586616298};
const double D = -10.033836870763;
const double Kg[2] = {-0.353782215622, 0.010214756482};
const double Og[2] = {3.258593778069, -39.474929358572};
	
	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
	double y;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(y > 100.0) y = 100.0;
	else if(y < -100.0) y = -100.0;

	return y ;
}

double P3H_YZMP_CON_E(double u, double ZMP, int zero) 
{	


const double A[2][2] = {{0.000000000000, 1.000000000000}, {-202.763743122798, -1.217815421806}};
const double B[2] = {0.000000000000, 542.659287719217};
const double C[2] = {4.011126863189, 0.021586616298};
const double D = -9.619009264241;
const double Kg[2] = {-0.351055897197, 0.010655276172};
const double Og[2] = {3.395284740231, -38.761667876652};
	
	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
	double y;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(y > 100.0) y = 100.0;
	else if(y < -100.0) y = -100.0;

	return y ;
}

double P3F_XZMP_CON_E(double u, double ZMP, int zero) 
{	

	const double A[2][2] = {{0.000000000000, 1.000000000000}, {-279.244126332315, -2.029692369676}};
const double B[2] = {0.000000000000, 726.065482466260};
const double C[2] = {5.366794279694, 0.035977693830};
const double D = -12.870010263056;
const double Kg[2] = {-0.362548740698, 0.008222822561};
const double Og[2] = {2.874993340319, -40.558191127047};

	
	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
	double y;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(y > 100.0) y = 100.0;
	else if(y < -100.0) y = -100.0;

	return y ;
}

double P3F_YZMP_CON_E(double u, double ZMP, int zero) 
{	

	const double A[2][2] = {{0.000000000000, 1.000000000000}, {-292.310926775750, -2.029692369676}};
const double B[2] = {0.000000000000, 757.400735328215};
const double C[2] = {5.598412308472, 0.035977693830};
const double D = -13.425449180988;
const double Kg[2] = {-0.364801503204, 0.007882627190};
const double Og[2] = {2.929469678508, -39.748836651203};
	
	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
	double y;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(y > 100.0) y = 100.0;
	else if(y < -100.0) y = -100.0;

	return y ;
}

double P2_XZMP_CON_E(double u, double ZMP, int zero) 
{	

/*    
const double A[2][2] = {{0.000000000000, 1.000000000000}, {-41.958143909403, -1.058127392969}};  // With 우레탄
const double B[2] = {0.000000000000, 87.769925541094};
const double C[2] = {2.457065830961, 0.045571745518};
const double D = -3.780101278401;
const double Kg[2] = {-0.284358722598, 0.079091699853};  // 미분식
const double Og[2] = {8.603956879066, 39.528530846301};
*/

// With 우레탄
const double A[2][2] = {{0.000000000000, 1.000000000000}, {-87.422091137767, -1.058127392969}};
const double B[2] = {0.000000000000, 157.714459738577};
const double C[2] = {4.415120642784, 0.045571745518};
const double D = -6.792493296590;
const double Kg[2] = {-0.446516389521, 0.044015448035};
const double Og[2] = {5.069750236016, 12.251312296639};

	
	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
	double y;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(y > 60.0) y = 60.0;
	else if(y < -60.0) y = -60.0;

	return y ;
}

double P2_XZMP_CON(double ZMP, int zero) // 싱글 보상기 방식 //
{
	static double x1new,x2new, x1=0., x2=0.;

	static double filt;


	const float adm[4] = {0.827419092046, -0.604214556180, 0.004556316693, 0.998441807157};
	const float bdm[2] = {0.004556316693, 0.000011750164};
	const float cdm[2] = {-0.501520164374, -50.388047120832};


	x1new=adm[0]*x1 + adm[1]*x2 + bdm[0]*ZMP;
	x2new=adm[2]*x1 + adm[3]*x2 + bdm[1]*ZMP;

	if(zero==0) { x1new=0.; x2new=0.;}

	filt=cdm[0]*x1new+cdm[1]*x2new;


	x1=x1new;	x2=x2new;

	if(filt > 60.0) filt = 60.0;
	else if(filt < -60.0) filt = -60.0;

	return filt;
}

double P2_YZMP_CON_E(double u, double ZMP, int zero) 
{	

// With 우레탄
/*
const double A[2][2] = {{0.000000000000, 1.000000000000}, {-124.108500592140, -1.113818308388}};
const double B[2] = {0.000000000000, 214.155089668381};
const double C[2] = {5.995141845074, 0.047970258440};
const double D = -9.223295146268;
const double Kg[2] = {-0.484921935561, 0.036824628842};
const double Og[2] = {4.392941664450, 11.462796198811};
*/


// Without 우레탄
const double A[2][2] = {{0.000000000000, 1.000000000000}, {-171.347298630024, -1.225200139227}};
const double B[2] = {0.000000000000, 286.830163572817};
const double C[2] = {8.029636459856, 0.052767284284};
const double D = -12.353286861317;
const double Kg[2] = {-0.510187969101, 0.030592318993};
const double Og[2] = {3.541153909299, 6.455161097063};
	
	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
	double y;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(y > 80.0) y = 80.0;
	else if(y < -80.0) y = -80.0;

	return y ;
}

double P2_XZMP_INT_CON_E(double Ref_ZMP, double u, double ZMP, int zero)
{

//With 우레탄
/*
const double A[2][2] = {{0.000000000000, 1.000000000000}, {-41.958143909403, -1.058127392969}};
const double B[2] = {0.000000000000, 87.769925541094};
const double C[2] = {2.457065830961, 0.045571745518};
const double D = -3.780101278401;
const double Kg[2] = {-0.113343424277, 0.126050958119};
const double Og[2] = {5.241186087442, 1.402916591066};
const double ki = 0.561257607977;
*/

//Without 우레탄
const double A[2][2] = {{0.000000000000, 1.000000000000}, {-87.422091137767, -1.058127392969}};
const double B[2] = {0.000000000000, 157.714459738577};
const double C[2] = {4.415120642784, 0.045571745518};
const double D = -6.792493296590;
const double Kg[2] = {-0.351344393086, 0.070148819752};
const double Og[2] = {3.025927234728, -9.171498388494};
const double ki = 0.312346366612;



	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2], Err_Sum;
	double y;

	Err_Sum = Err_Sum + Del_T*(Ref_ZMP - ZMP);

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {Err_Sum = 0.; x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	y = -Kg[0]*x_new[0] - Kg[1]*x_new[1] + ki*Err_Sum;

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	//if(y > 100.0) y = 100.0;
	//else if(y < -100.0) y = -100.0;

	return y ;
}

double P2_YZMP_INT_CON_E(double Ref_ZMP, double u, double ZMP, int zero)
{

// With 우레탄 
/*
const double A[2][2] = {{0.000000000000, 1.000000000000}, {-124.108500592140, -1.113818308388}};
const double B[2] = {0.000000000000, 214.155089668381};
const double C[2] = {5.995141845074, 0.047970258440};
const double D = -9.223295146268;
const double Kg[2] = {-0.243274631823, 0.098265853940};
const double Og[2] = {3.185489228805, -4.404355696701};
const double ki = 0.776073079829;
*/

// Without 우레탄 
const double A[2][2] = {{0.000000000000, 1.000000000000}, {-171.347298630024, -1.225200139227}};
const double B[2] = {0.000000000000, 286.830163572817};
const double C[2] = {8.029636459856, 0.052767284284};
const double D = -12.353286861317;
const double Kg[2] = {-0.346327936339, 0.072979601135};
const double Og[2] = {2.398134493305, -9.122097315492};
const double ki = 0.579436966914;

	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2], Err_Sum;
	double y;

	Err_Sum = Err_Sum + Del_T*(Ref_ZMP - ZMP);

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {Err_Sum = 0.; x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	y = -Kg[0]*x_new[0] - Kg[1]*x_new[1] + ki*Err_Sum;

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	//if(y > 160.0) y = 160.0;
	//else if(y < -160.0) y = -160.0;

	return y ;
}



double P1_XZMP_CON_E(double u, double ZMP, int zero) 
{	


const double A[2][2] = {{0.000000000000, 1.000000000000}, {-34.481978866589, -0.891054646711}};
const double B[2] = {0.000000000000, 76.268133167533};
const double C[2] = {2.135080129575, 0.038376206752};
const double D = -3.284738660885;
const double Kg[2] = {-0.242197862979, 0.093209903770};
const double Og[2] = {12.767712114215, 100.292791910954};
	
	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
	double y;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(y > 60.0) y = 60.0;
	else if(y < -60.0) y = -60.0;

	return y ;
}

double P1_YZMP_CON_E(double u, double ZMP, int zero) 
{	

// With 우레탄
/*
const double A[2][2] = {{0.000000000000, 1.000000000000}, {-16.864632237326, -1.002436477550}};
const double B[2] = {0.000000000000, 49.164522968668};
const double C[2] = {1.376331001047, 0.043173232596};
const double D = -2.117432309303;
const double Kg[2] = {-0.159762197679, 0.101649791774};
const double Og[2] = {13.996815323841, 86.472854865253};
*/

// Without 우레탄
const double A[2][2] = {{0.000000000000, 1.000000000000}, {-26.742818746707, -1.002436477550}};
const double B[2] = {0.000000000000, 64.361732983099};
const double C[2] = {1.801767677929, 0.043173232596};
const double D = -2.771950273737;
const double Kg[2] = {-0.275518043483, 0.077648057173};
const double Og[2] = {11.302299056258, 60.997202161162};
	
	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
	double y;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(y > 60.0) y = 60.0;
	else if(y < -60.0) y = -60.0;

	return y ;
}

double P2_XCOM_ZMP_CON_E(double ref_zmp, double zmp, double ref_cog, int zero) // 오일러 방식 //
{
	
	const double A[2][2] = {{0.000000000000, 1.000000000000}, {-87.422091137767, -1.058127392969}};
	const double B[2] = {0.000000000000, 157.714459738577};
	const double C[2] = {4.415120642784, 0.045571745518};
	const double D = -6.792493296590;
const double Kg_ZMP[2] = {-0.446516389521, 0.044015448035};
const double Og_ZMP[2] = {5.069750236016, 12.251312296639};
const double Kg_CoG[2] = {-0.047060308548, 0.094740029746};

const double K_ff = 0.780378180168;



	static double x_old[2], x_new[2], x_CoG_old[2], x_CoG_new[2], Temp_1[2], Temp_2[2], Temp_3[2], Temp_1_CoG[2], Temp_2_CoG[2], Temp_3_CoG[2];
	static double u_zmp = 0., u_cog = 0.;
	double y;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u_zmp;
	Temp_2[1] = B[1]*u_zmp;

	Temp_3[0] = Og_ZMP[0]*(zmp - ref_zmp - C[0]*x_old[0] - C[1]*x_old[1] -D*u_zmp);
	Temp_3[1] = Og_ZMP[1]*(zmp - ref_zmp - C[0]*x_old[0] - C[1]*x_old[1] -D*u_zmp);


	Temp_1_CoG[0] = A[0][0]*x_CoG_old[0] + A[0][1]*x_CoG_old[1];
	Temp_1_CoG[1] = A[1][0]*x_CoG_old[0] + A[1][1]*x_CoG_old[1];

	Temp_2_CoG[0] = B[0]*(u_zmp + u_cog);
	Temp_2_CoG[1] = B[1]*(u_zmp + u_cog);

	Temp_3_CoG[0] = Og_ZMP[0]*(zmp - C[0]*x_CoG_old[0] - C[1]*x_CoG_old[1] -D*(u_zmp + u_cog));
	Temp_3_CoG[1] = Og_ZMP[1]*(zmp - C[0]*x_CoG_old[0] - C[1]*x_CoG_old[1] -D*(u_zmp + u_cog));



	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; x_CoG_old[0] = 0.; x_CoG_old[1] = 0.; x_CoG_new[0] = 0.; x_CoG_new[1] = 0.;
	               Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.; 
				   Temp_1_CoG[0] = 0.; Temp_1_CoG[1] = 0.; Temp_2_CoG[0] = 0.; Temp_2_CoG[1] = 0.; Temp_3_CoG[0] = 0.; Temp_3_CoG[1] = 0.; 
				   u_zmp = 0.; u_cog = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);


	x_CoG_new[0] = x_CoG_old[0] + Del_T*(Temp_1_CoG[0] + Temp_2_CoG[0] + Temp_3_CoG[0]);
	x_CoG_new[1] = x_CoG_old[1] + Del_T*(Temp_1_CoG[1] + Temp_2_CoG[1] + Temp_3_CoG[1]);

 
	u_zmp = - Kg_ZMP[0]*x_new[0] - Kg_ZMP[1]*x_new[1];  
	u_cog = - Kg_CoG[0]*x_CoG_new[0] - Kg_CoG[1]*x_CoG_new[1] + K_ff*ref_cog;

	y = u_zmp + u_cog;

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	x_CoG_old[0] = x_CoG_new[0];
	x_CoG_old[1] = x_CoG_new[1];

	if(y > 60.0) y = 60.0;
	else if(y < -60.0) y = -60.0;

	return y ;

}

double P2_YCOM_ZMP_CON_E(double ref_zmp, double zmp, double ref_cog, int zero) // 오일러 방식 //
{
	
	const double A[2][2] = {{0.000000000000, 1.000000000000}, {-16.864632237326, -1.002436477550}};
	const double B[2] = {0.000000000000, 49.164522968668};
	const double C[2] = {1.376331001047, 0.043173232596};
	const double D = -2.117432309303;
	const double Kg_ZMP[2] = {-0.159762197679, 0.101649791774};
	const double Og_ZMP[2] = {13.996815323841, 86.472854865253};
	const double Kg_CoG[2] = {1.121446206197, 0.223689011067};
	const double K_ff = 2.253031740790;


	static double x_old[2], x_new[2], x_CoG_old[2], x_CoG_new[2], Temp_1[2], Temp_2[2], Temp_3[2], Temp_1_CoG[2], Temp_2_CoG[2], Temp_3_CoG[2];
	static double u_zmp = 0., u_cog = 0.;
	double y;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u_zmp;
	Temp_2[1] = B[1]*u_zmp;

	Temp_3[0] = Og_ZMP[0]*(zmp - ref_zmp - C[0]*x_old[0] - C[1]*x_old[1] -D*u_zmp);
	Temp_3[1] = Og_ZMP[1]*(zmp - ref_zmp - C[0]*x_old[0] - C[1]*x_old[1] -D*u_zmp);


	Temp_1_CoG[0] = A[0][0]*x_CoG_old[0] + A[0][1]*x_CoG_old[1];
	Temp_1_CoG[1] = A[1][0]*x_CoG_old[0] + A[1][1]*x_CoG_old[1];

	Temp_2_CoG[0] = B[0]*(u_zmp + u_cog);
	Temp_2_CoG[1] = B[1]*(u_zmp + u_cog);

	Temp_3_CoG[0] = Og_ZMP[0]*(zmp - C[0]*x_CoG_old[0] - C[1]*x_CoG_old[1] -D*(u_zmp + u_cog));
	Temp_3_CoG[1] = Og_ZMP[1]*(zmp - C[0]*x_CoG_old[0] - C[1]*x_CoG_old[1] -D*(u_zmp + u_cog));



	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; x_CoG_old[0] = 0.; x_CoG_old[1] = 0.; x_CoG_new[0] = 0.; x_CoG_new[1] = 0.;
	               Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.; 
				   Temp_1_CoG[0] = 0.; Temp_1_CoG[1] = 0.; Temp_2_CoG[0] = 0.; Temp_2_CoG[1] = 0.; Temp_3_CoG[0] = 0.; Temp_3_CoG[1] = 0.; 
				   u_zmp = 0.; u_cog = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);


	x_CoG_new[0] = x_CoG_old[0] + Del_T*(Temp_1_CoG[0] + Temp_2_CoG[0] + Temp_3_CoG[0]);
	x_CoG_new[1] = x_CoG_old[1] + Del_T*(Temp_1_CoG[1] + Temp_2_CoG[1] + Temp_3_CoG[1]);

 
	u_zmp = - Kg_ZMP[0]*x_new[0] - Kg_ZMP[1]*x_new[1];  
	u_cog = - Kg_CoG[0]*x_CoG_new[0] - Kg_CoG[1]*x_CoG_new[1] + K_ff*ref_cog;

	y = u_zmp + u_cog;

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	x_CoG_old[0] = x_CoG_new[0];
	x_CoG_old[1] = x_CoG_new[1];

	if(y > 50.0) y = 50.0;
	else if(y < -50.0) y = -50.0;

	return y ;

}

double P1_XCOM_ZMP_CON_E(double ref_zmp, double zmp, double ref_cog, int zero) // 오일러 방식 //
{
	
	const double A[2][2] = {{0.000000000000, 1.000000000000}, {-41.958143909403, -1.058127392969}};
	const double B[2] = {0.000000000000, 87.769925541094};
	const double C[2] = {2.457065830961, 0.045571745518};
	const double D = -3.780101278401;
	const double Kg_ZMP[2] = {-0.284358722598, 0.079091699853};
	const double Og_ZMP[2] = {8.603956879066, 39.528530846301};
	const double Kg_CoG[2] = {0.342279612355, 0.124665396941};
	const double K_ff = 1.262040842422;


	static double x_old[2], x_new[2], x_CoG_old[2], x_CoG_new[2], Temp_1[2], Temp_2[2], Temp_3[2], Temp_1_CoG[2], Temp_2_CoG[2], Temp_3_CoG[2];
	static double u_zmp = 0., u_cog = 0.;
	double y;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u_zmp;
	Temp_2[1] = B[1]*u_zmp;

	Temp_3[0] = Og_ZMP[0]*(zmp - ref_zmp - C[0]*x_old[0] - C[1]*x_old[1] -D*u_zmp);
	Temp_3[1] = Og_ZMP[1]*(zmp - ref_zmp - C[0]*x_old[0] - C[1]*x_old[1] -D*u_zmp);


	Temp_1_CoG[0] = A[0][0]*x_CoG_old[0] + A[0][1]*x_CoG_old[1];
	Temp_1_CoG[1] = A[1][0]*x_CoG_old[0] + A[1][1]*x_CoG_old[1];

	Temp_2_CoG[0] = B[0]*(u_zmp + u_cog);
	Temp_2_CoG[1] = B[1]*(u_zmp + u_cog);

	Temp_3_CoG[0] = Og_ZMP[0]*(zmp - C[0]*x_CoG_old[0] - C[1]*x_CoG_old[1] -D*(u_zmp + u_cog));
	Temp_3_CoG[1] = Og_ZMP[1]*(zmp - C[0]*x_CoG_old[0] - C[1]*x_CoG_old[1] -D*(u_zmp + u_cog));



	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; x_CoG_old[0] = 0.; x_CoG_old[1] = 0.; x_CoG_new[0] = 0.; x_CoG_new[1] = 0.;
	               Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.; 
				   Temp_1_CoG[0] = 0.; Temp_1_CoG[1] = 0.; Temp_2_CoG[0] = 0.; Temp_2_CoG[1] = 0.; Temp_3_CoG[0] = 0.; Temp_3_CoG[1] = 0.; 
				   u_zmp = 0.; u_cog = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);


	x_CoG_new[0] = x_CoG_old[0] + Del_T*(Temp_1_CoG[0] + Temp_2_CoG[0] + Temp_3_CoG[0]);
	x_CoG_new[1] = x_CoG_old[1] + Del_T*(Temp_1_CoG[1] + Temp_2_CoG[1] + Temp_3_CoG[1]);

 
	u_zmp = - Kg_ZMP[0]*x_new[0] - Kg_ZMP[1]*x_new[1];  
	u_cog = - Kg_CoG[0]*x_CoG_new[0] - Kg_CoG[1]*x_CoG_new[1] + K_ff*ref_cog;

	y = u_zmp + u_cog;

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	x_CoG_old[0] = x_CoG_new[0];
	x_CoG_old[1] = x_CoG_new[1];

	if(y > 50.0) y = 50.0;
	else if(y < -50.0) y = -50.0;

	return y ;

}

double P1_YCOM_ZMP_CON_E(double ref_zmp, double zmp, double ref_cog, int zero) // 오일러 방식 //
{
	
	const double A[2][2] = {{0.000000000000, 1.000000000000}, {-124.108500592140, -1.113818308388}};
	const double B[2] = {0.000000000000, 214.155089668381};
	const double C[2] = {5.995141845074, 0.047970258440};
	const double D = -9.223295146268;
	const double Kg_ZMP[2] = {-0.484921935561, 0.036824628842};
	const double Og_ZMP[2] = {4.980636379693, 21.399928830981};
	const double Kg_CoG[2] = {0.391732456781, 0.069511220652};
	const double K_ff = 1.494244197023;


	static double x_old[2], x_new[2], x_CoG_old[2], x_CoG_new[2], Temp_1[2], Temp_2[2], Temp_3[2], Temp_1_CoG[2], Temp_2_CoG[2], Temp_3_CoG[2];
	static double u_zmp = 0., u_cog = 0.;
	double y;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u_zmp;
	Temp_2[1] = B[1]*u_zmp;

	Temp_3[0] = Og_ZMP[0]*(zmp - ref_zmp - C[0]*x_old[0] - C[1]*x_old[1] -D*u_zmp);
	Temp_3[1] = Og_ZMP[1]*(zmp - ref_zmp - C[0]*x_old[0] - C[1]*x_old[1] -D*u_zmp);


	Temp_1_CoG[0] = A[0][0]*x_CoG_old[0] + A[0][1]*x_CoG_old[1];
	Temp_1_CoG[1] = A[1][0]*x_CoG_old[0] + A[1][1]*x_CoG_old[1];

	Temp_2_CoG[0] = B[0]*(u_zmp + u_cog);
	Temp_2_CoG[1] = B[1]*(u_zmp + u_cog);

	Temp_3_CoG[0] = Og_ZMP[0]*(zmp - C[0]*x_CoG_old[0] - C[1]*x_CoG_old[1] -D*(u_zmp + u_cog));
	Temp_3_CoG[1] = Og_ZMP[1]*(zmp - C[0]*x_CoG_old[0] - C[1]*x_CoG_old[1] -D*(u_zmp + u_cog));



	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; x_CoG_old[0] = 0.; x_CoG_old[1] = 0.; x_CoG_new[0] = 0.; x_CoG_new[1] = 0.;
	               Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.; 
				   Temp_1_CoG[0] = 0.; Temp_1_CoG[1] = 0.; Temp_2_CoG[0] = 0.; Temp_2_CoG[1] = 0.; Temp_3_CoG[0] = 0.; Temp_3_CoG[1] = 0.; 
				   u_zmp = 0.; u_cog = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);


	x_CoG_new[0] = x_CoG_old[0] + Del_T*(Temp_1_CoG[0] + Temp_2_CoG[0] + Temp_3_CoG[0]);
	x_CoG_new[1] = x_CoG_old[1] + Del_T*(Temp_1_CoG[1] + Temp_2_CoG[1] + Temp_3_CoG[1]);

 
	u_zmp = - Kg_ZMP[0]*x_new[0] - Kg_ZMP[1]*x_new[1];  
	u_cog = - Kg_CoG[0]*x_CoG_new[0] - Kg_CoG[1]*x_CoG_new[1] + K_ff*ref_cog;

	y = u_zmp + u_cog;

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	x_CoG_old[0] = x_CoG_new[0];
	x_CoG_old[1] = x_CoG_new[1];

	if(y > 50.0) y = 50.0;
	else if(y < -50.0) y = -50.0;

	return y ;

}

void P1_Y_th_th_d_Observer(double u, double ZMP, int zero) 
{	
	const double A[2][2] = {{0.000000000000, 1.000000000000}, {-26.742818746707, -1.002436477550}};
	const double B[2] = {0.000000000000, 64.361732983099};
	const double C[2] = {1.801767677929, 0.043173232596};
	const double D = -2.771950273737;
	//const double Kg[2] = {-0.275518043483, 0.077648057173};
	const double Og[2] = {11.302299056258, 60.997202161162};
	
	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	Body_Y_th = x_new[0]*0.001; 
	Body_Y_th_d = x_new[1]*0.001;

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

}

double FORCE_DIFF_CON_R1(double Ref_force, double force, int zero)
{
	static double y;
	static double sume = 0.;
	const double KI = 0.0001;
		
	y = 0.0*(Ref_force - force) + KI*sume;
		
	sume += Ref_force - force;
	
	if(sume > 10.0/KI) sume = 10./KI;
	else if(sume < -10.0/KI) sume = -10./KI;  

	if(y > 10.0) y = 10.0;
	else if(y < -10.0) y = -10.0;
	
	if(zero == 0) sume = 0.;

	return y;
}

double FORCE_DIFF_CON_R2(double Ref_force, double force, int zero)
{
	static double y;
	static double sume = 0.;
	const double KI = 0.0001;
		
	y = 0.0*(Ref_force - force) + KI*sume;
		
	sume += Ref_force - force;
	
	if(sume > 10.0/KI) sume = 10./KI;
	else if(sume < -10.0/KI) sume = -10./KI;  

	if(y > 10.0) y = 10.0;
	else if(y < -10.0) y = -10.0;
	
	if(zero == 0) sume = 0.;

	return y;
}

double TORQ_RAR_INT_CON(double Ref_Torq, double Torq, int zero)
{
	static double y;
	static double sume = 0.;
	const double KI = 0.0005;
		
	y = 0.0*(Ref_Torq - Torq) + KI*sume;
		
	sume += Ref_Torq - Torq;
	
	if(sume > 3.0/KI) sume = 3./KI;
	else if(sume < -3.0/KI) sume = -3./KI;

	if(y > 3.0) y = 3.0;
	else if(y < -3.0) y = -3.0;
	
	if(zero == 0) sume = 0.;

	return y;
}

double TORQ_LAR_INT_CON(double Ref_Torq, double Torq, int zero)
{
	static double y;
	static double sume = 0.;
	const double KI = 0.0005;
		
	y = 0.0*(Ref_Torq - Torq) + KI*sume;
		
	sume += Ref_Torq - Torq;
	
	if(sume > 3.0/KI) sume = 3./KI;
	else if(sume < -3.0/KI) sume = -3./KI;

	if(y > 3.0) y = 3.0;
	else if(y < -3.0) y = -3.0;
	
	if(zero == 0) sume = 0.;

	return y;
}

double TORQ_DIFF_RAP_INT_CON(double Ref_Torq, double Torq, int zero)
{
	static double y;
	static double sume = 0.;
	const double KI = 0.0004;
		
	y = 0.0*(Ref_Torq - Torq) + KI*sume;
		
	sume += Ref_Torq - Torq;
	
	if(sume > 3.0/KI) sume = 3./KI;
	else if(sume < -3.0/KI) sume = -3./KI;

	if(y > 3.0) y = 3.0;
	else if(y < -3.0) y = -3.0;
	
	if(zero == 0) sume = 0.;

	return y;
}

double RLAR_TORQ_CON(double Torq, int zero)
{
	static double x1new,x2new, x1=0., x2=0.;

	static double filt;
	

	const float adm[4] = {-0.001370134118, -13.134008890337, 0.000097375511, 0.933434769109};
	const float bdm[2] = {0.000097375511, 0.000000493514};
	const float cdm[2] = {0.000000000000, 68754.935415698783};


	x1new=adm[0]*x1 + adm[1]*x2 + bdm[0]*Torq;
	x2new=adm[2]*x1 + adm[3]*x2 + bdm[1]*Torq;

	if(zero==0) { x1new=0.; x2new=0.;}

	filt=cdm[0]*x1new+cdm[1]*x2new;


	x1=x1new;	x2=x2new;

	if(filt > 5.0) filt = 5.0;
	else if(filt < -5.0) filt = -5.0;

	return filt;
}

double RLAP_TORQ_CON(double Torq, int zero)
{
	static double x1new,x2new, x1=0., x2=0.;

	static double filt;
	

	const float adm[4] = {-0.001370134118, -13.134008890337, 0.000097375511, 0.933434769109};
	const float bdm[2] = {0.000097375511, 0.000000493514};
	const float cdm[2] = {0.000000000000, 68754.935415698783};


	x1new=adm[0]*x1 + adm[1]*x2 + bdm[0]*Torq;
	x2new=adm[2]*x1 + adm[3]*x2 + bdm[1]*Torq;

	if(zero==0) { x1new=0.; x2new=0.;}

	filt=cdm[0]*x1new+cdm[1]*x2new;


	x1=x1new;	x2=x2new;

	if(filt > 5.0) filt = 5.0;
	else if(filt < -5.0) filt = -5.0;

	return filt;
}

double SSP_Y_VIB_CON_E_RF(double u, double Ang_Vel, int zero) // 오일러 방식 //
{


	const double A[2][2] = {{0.000000000000, 1.000000000000}, {-619.208589030212, 0.000000000000}};
	const double B[2] = {0.000000000000, 583.535861757484};
	const double C[2] = {0.000000000000, 1.000000000000};
	const double D = 0.000000000000;
	const double Kg[2] = {-0.889745811794, 0.034273814706};
	const double Og[2] = {-3.037427846270, 100.000000000000};

	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
	double filt;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(Ang_Vel - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(Ang_Vel - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	filt = Kg[0]*x_new[0] + Kg[1]*x_new[1];

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(filt > deg2rad*3.0) filt = deg2rad*3.0;
	else if(filt < -deg2rad*3.0) filt = -deg2rad*3.0;

	return filt ;

}

double SSP_Y_VIB_CON_E_LF(double u, double Ang_Vel, int zero) // 오일러 방식 //
{

	const double A[2][2] = {{0.000000000000, 1.000000000000}, {-619.208589030212, 0.000000000000}};
	const double B[2] = {0.000000000000, 583.535861757484};
	const double C[2] = {0.000000000000, 1.000000000000};
	const double D = 0.000000000000;
	const double Kg[2] = {-0.889745811794, 0.034273814706};
	const double Og[2] = {-3.037427846270, 100.000000000000};  

	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
	double filt;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(Ang_Vel - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(Ang_Vel - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	filt = Kg[0]*x_new[0] + Kg[1]*x_new[1];

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(filt > deg2rad*3.0) filt = deg2rad*3.0;
	else if(filt < -deg2rad*3.0) filt = -deg2rad*3.0;

	return filt ;

}

double SSP_X_VIB_CON_E_RF(double u, double Ang_Vel, int zero) // 오일러 방식 //
{


const double A[2][2] = {{0.000000000000, 1.000000000000}, {-1366.035211223440, 0.000000000000}};
const double B[2] = {0.000000000000, 1330.362483950713};
const double C[2] = {0.000000000000, 1.000000000000};
const double D = 0.000000000000;
const double Kg[2] = {-0.951639291168, 0.015033496691};
const double Og[2] = {-0.830121199995, 100.000000000000};

	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
	double filt;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(Ang_Vel - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(Ang_Vel - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	filt = Kg[0]*x_new[0] + Kg[1]*x_new[1];

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(filt > deg2rad*3.0) filt = deg2rad*3.0;
	else if(filt < -deg2rad*3.0) filt = -deg2rad*3.0;

	return filt ;

}

double SSP_X_VIB_CON_E_LF(double u, double Ang_Vel, int zero) // 오일러 방식 //
{

	const double A[2][2] = {{0.000000000000, 1.000000000000}, {-619.208589030212, 0.000000000000}};
	const double B[2] = {0.000000000000, 583.535861757484};
	const double C[2] = {0.000000000000, 1.000000000000};
	const double D = 0.000000000000;
	const double Kg[2] = {-0.889745811794, 0.034273814706};
	const double Og[2] = {-3.037427846270, 100.000000000000};  

	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
	double filt;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(Ang_Vel - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(Ang_Vel - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	filt = Kg[0]*x_new[0] + Kg[1]*x_new[1];

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(filt > deg2rad*3.0) filt = deg2rad*3.0;
	else if(filt < -deg2rad*3.0) filt = -deg2rad*3.0;

	return filt ;

}

double SSP_Z_VIB_CON_E(double u, double Ang_Vel, int zero) // 오일러 방식 //
{


	const double A[2][2] = {{0.000000000000, 1.000000000000}, {-362.519904539554, 0.000000000000}};
	const double B[2] = {0.000000000000, 362.519904539554};
	const double C[2] = {0.000000000000, 1.000000000000};
	const double D = 0.000000000000;
	const double Kg[2] = {-0.864807423299, 0.038618569145};
	const double Og[2] = {-0.103415274557, 40.000000000000};


	static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
	double filt;

	Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
	Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

	Temp_2[0] = B[0]*u;
	Temp_2[1] = B[1]*u;

	Temp_3[0] = Og[0]*(Ang_Vel - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
	Temp_3[1] = Og[1]*(Ang_Vel - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

	if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

	x_new[0] = x_old[0] + Del_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
	x_new[1] = x_old[1] + Del_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);
 
	filt = Kg[0]*x_new[0] + Kg[1]*x_new[1];

	x_old[0] = x_new[0];
	x_old[1] = x_new[1];

	if(filt > deg2rad*3.0) filt = deg2rad*3.0;
	else if(filt < -deg2rad*3.0) filt = -deg2rad*3.0;

	return filt ;

}


double k3F(double x, int R1_Wstage, int L1_Wstage){
	double y;

	if(R1_Wstage == 0 && L1_Wstage == 0) y = 0.;
	else{
		if( x <= 3.) y = 1.0;
		else if( x > 3. && x <= 4) y = 0.5*(cos((x-3.)*PI) + 1);
		else y = 0.;
	}

	return y ;
}

double k3H(double x, int R1_Wstage, int L1_Wstage){
	double y;
	
	if(R1_Wstage == 0 && L1_Wstage == 0){
		if( x <= 3.) y = 1.0;
		else if( x > 3. && x <= 4) y = 0.5*(cos((x-3.)*PI) + 1);
		else y = 0.;
	}
	else y = 0.;

	return y ;
}

double k4(double x){
	double y;

	if( x <= 3.) y = 0.0;
	else if( x > 3. && x <= 4) y = 0.5*(cos((x-2.)*PI) + 1);
	else if( x > 4. ) y = 1.;

	return y ;
}

double k2(double x){
	double y;

	if( x <= 1.4) y = 0.0;
	else if( x > 1.4 && x <= 1.6) y = 0.5*(1 - cos((x-1.4)/0.2*PI));
	else if( x > 1.6 ) y = 1.;

	return y ;
}

double k1(double x){
	double y;

	if( x <= 1.4) y = 1.0;
	else if( x > 1.4 && x <= 1.6) y = 0.5*(cos((x-1.4)/0.2*PI) + 1);
	else if( x > 1.6 ) y = 0.;

	return y ;
}


double Support_Value(double Fz, int Foot_Num){
	double y;
	const double alpha = 5.0 , beta_1 = EARLY_LAND_THRES_1, beta_2 = EARLY_LAND_THRES_2; //EARLY_LAND_THRES;

	if(Foot_Num == RF_1 || Foot_Num == LF_1){
		if(Fz < alpha)	y = 0.;
		else if(Fz >= alpha && Fz < beta_1)	y = (1./(beta_1-alpha))*Fz + (alpha/(alpha-beta_1));
		else y = 1.0;
	}
	else if(Foot_Num == RF_2 || Foot_Num == LF_2){
		if(Fz < alpha)	y = 0.;
		else if(Fz >= alpha && Fz < beta_2)	y = (1./(beta_2-alpha))*Fz + (alpha/(alpha-beta_2));
		else y = 1.0;
	}

	return y;
}

double Support_Value_BP(double Fz, int Foot_Num){
	double y;
	const double alpha = 5.0 , beta_1 = EARLY_LAND_THRES_1, beta_2 = EARLY_LAND_THRES_2; //EARLY_LAND_THRES;

	if(Foot_Num == RF_1 || Foot_Num == LF_1){
		if(Fz < alpha)	y = 0.;
		else if(Fz >= alpha && Fz < beta_2)	y = (1./(beta_2-alpha))*Fz + (alpha/(alpha-beta_2));
		else y = 1.0;
	}

	return y;
}


void ZMP_CON_MANAGER(double R1_Load_N, double R2_Load_N, double L1_Load_N, double L2_Load_N, unsigned char R1_Wstage, unsigned char L1_Wstage, int zero) // update
{
	static double Number_Support_LP = 0., Old_Number_Support_LP = 4., Number_Support = 0.;
	static double	Del_YZMP_CON_P4 = 0., Del_YZMP_CON_P3H = 0., Del_YZMP_CON_P3F = 0.,
					Del_XZMP_CON_P4 = 0., Del_XZMP_CON_P3H = 0., Del_XZMP_CON_P3F = 0.;
	static double	Old_Del_YZMP_CON_P4 = 0., Old_Del_YZMP_CON_P3H = 0., Old_Del_YZMP_CON_P3F = 0.,
					Old_Del_XZMP_CON_P4 = 0., Old_Del_XZMP_CON_P3H = 0., Old_Del_XZMP_CON_P3F = 0.;

	Number_Support = Support_Value(R1_Load_N, RF_1) + Support_Value(R2_Load_N, RF_2) + Support_Value(L1_Load_N, LF_1) + Support_Value(L2_Load_N, LF_2) ;

	if(Number_Support <= 3) Number_Support = 3.0;
	else if(Number_Support >= 4) Number_Support = 4.0;

	Number_Support_LP = (1. - 2.*PI*F_CUT_Number_Support*Del_T)*Old_Number_Support_LP + 2.*PI*F_CUT_Number_Support*Del_T*Number_Support;
	Old_Number_Support_LP = Number_Support_LP;

	_log_temp[12] = Number_Support;
	_log_temp[13] = Number_Support_LP;

	Del_YZMP_CON_P4 = k4(Number_Support_LP)*0.85*P4_YZMP_CON_E(-Del_YZMP_CON,Y_ZMP - 1.*Ref_ZMP[1], 1); 
	if(Ls >= 0)	Del_XZMP_CON_P4 = k4(Number_Support_LP)*0.75*P4_XZMP_CON_E(-Del_XZMP_CON,X_ZMP - X_ZMP_OFFSET - 1.*Ref_ZMP[0], 1); 
	else Del_XZMP_CON_P4 = k4(Number_Support_LP)*0.75*P4_XZMP_CON_E(-Del_XZMP_CON,X_ZMP - X_ZMP_OFFSET_BWD - 1.*Ref_ZMP[0], 1); 
	Old_Del_YZMP_CON_P4 = Del_YZMP_CON_P4;
	Old_Del_XZMP_CON_P4 = Del_XZMP_CON_P4;

	Del_YZMP_CON_P3H = k3H(Number_Support_LP, R1_Wstage, L1_Wstage)*0.85*P3H_YZMP_CON_E(-Del_YZMP_CON,Y_ZMP - 1.*Ref_ZMP[1], 1); 
	if(Ls >= 0) Del_XZMP_CON_P3H = k3H(Number_Support_LP, R1_Wstage, L1_Wstage)*0.75*P3H_XZMP_CON_E(-Del_XZMP_CON,X_ZMP - X_ZMP_OFFSET - 1.*Ref_ZMP[0], 1);
	else Del_XZMP_CON_P3H = k3H(Number_Support_LP, R1_Wstage, L1_Wstage)*0.75*P3H_XZMP_CON_E(-Del_XZMP_CON,X_ZMP - X_ZMP_OFFSET_BWD - 1.*Ref_ZMP[0], 1); 
	Old_Del_YZMP_CON_P3H = Del_YZMP_CON_P3H;
	Old_Del_XZMP_CON_P3H = Del_XZMP_CON_P3H;
	
	Del_YZMP_CON_P3F = k3F(Number_Support_LP,R1_Wstage, L1_Wstage)*0.85*P3F_YZMP_CON_E(-Del_YZMP_CON,Y_ZMP - 1.*Ref_ZMP[1], 1); 
	if(Ls >= 0) Del_XZMP_CON_P3F = k3F(Number_Support_LP,R1_Wstage, L1_Wstage)*0.75*P3F_XZMP_CON_E(-Del_XZMP_CON,X_ZMP - X_ZMP_OFFSET - 1.*Ref_ZMP[0], 1);
	else Del_XZMP_CON_P3F = k3F(Number_Support_LP,R1_Wstage, L1_Wstage)*0.75*P3F_XZMP_CON_E(-Del_XZMP_CON,X_ZMP - X_ZMP_OFFSET_BWD - 1.*Ref_ZMP[0], 1); 
	Old_Del_YZMP_CON_P3F = Del_YZMP_CON_P3F;
	Old_Del_XZMP_CON_P3F = Del_XZMP_CON_P3F;
	

	Del_XZMP_CON = 1.*(Del_XZMP_CON_P4 + Del_XZMP_CON_P3H + Del_XZMP_CON_P3F);
	Del_YZMP_CON = 1.*(Del_YZMP_CON_P4 + Del_YZMP_CON_P3H + Del_YZMP_CON_P3F); 

	Old_Del_XZMP_CON = Del_XZMP_CON;
	Old_Del_YZMP_CON = Del_YZMP_CON;

	if(zero == 0){
		Old_Number_Support_LP = 4;
		Del_YZMP_CON_P4 = 0.; Del_YZMP_CON_P3H = 0.; Del_YZMP_CON_P3F = 0.;	Del_XZMP_CON_P4 = 0.; Del_XZMP_CON_P3H = 0.; Del_XZMP_CON_P3F = 0.;
		Old_Del_YZMP_CON_P4 = 0.; Old_Del_YZMP_CON_P3H = 0.; Old_Del_YZMP_CON_P3F = 0.; Old_Del_XZMP_CON_P4 = 0.; Old_Del_XZMP_CON_P3H = 0.; Old_Del_XZMP_CON_P3F = 0.;
		P4_YZMP_CON_E(0,0,0); 
		P3H_YZMP_CON_E(0,0,0); 
		P4_XZMP_CON_E(0,0,0); 
		P3H_XZMP_CON_E(0,0,0);
		P3F_XZMP_CON_E(0,0,0);
		P3F_YZMP_CON_E(0,0,0);
		Del_XZMP_CON = 0.; Del_YZMP_CON = 0.;  
	}
	
}

void ZMP_CON_MANAGER_BP(double R1_Load_N, double L1_Load_N, unsigned char Land_ROK_Flag, unsigned char Land_LOK_Flag, int zero) // update
{
	static double Number_Support_LP = 0., Old_Number_Support_LP = 2., Number_Support = 0.;
	static double	Del_YZMP_CON_P2 = 0., Del_YZMP_CON_P1 = 0.,
					Del_XZMP_CON_P2 = 0., Del_XZMP_CON_P1 = 0.;
	static double	Old_Del_YZMP_CON_P2 = 0., Old_Del_YZMP_CON_P1 = 0.,
					Old_Del_XZMP_CON_P2 = 0., Old_Del_XZMP_CON_P1 = 0.;

	//Number_Support = Support_Value_BP(R1_Load_N, RF_1) + Support_Value_BP(L1_Load_N, LF_1) ;

	//if(Number_Support <= 1) Number_Support = 1.0;
	//else if(Number_Support >= 2) Number_Support = 2.0;

	if(Land_ROK_Flag == 1 && Land_LOK_Flag == 1) Number_Support = 2.0;
	else Number_Support = 1.0;

	Number_Support_LP = (1. - 2.*PI*F_CUT_Number_Support_BP*Del_T)*Old_Number_Support_LP + 2.*PI*F_CUT_Number_Support_BP*Del_T*Number_Support;
	Old_Number_Support_LP = Number_Support_LP;

	_log_temp[12] = Number_Support;
	_log_temp[13] = Number_Support_LP;

	Del_YZMP_CON_P2 = k2(Number_Support_LP)*0.85*P2_YZMP_CON_E(-Del_YZMP_CON,Y_ZMP - 1.*Ref_ZMP[1], 1); 
	Del_XZMP_CON_P2 = k2(Number_Support_LP)*1.0*P2_XZMP_CON_E(-Del_XZMP_CON,X_ZMP - X_ZMP_OFFSET_BP - 1.*Ref_ZMP[0], 1); 
	Old_Del_YZMP_CON_P2 = Del_YZMP_CON_P2;
	Old_Del_XZMP_CON_P2 = Del_XZMP_CON_P2;

	Del_YZMP_CON_P1 = k1(Number_Support_LP)*1.0*P1_YZMP_CON_E(-Del_YZMP_CON,Y_ZMP - 1.*Ref_ZMP[1], 1); 
	Del_XZMP_CON_P1 = k1(Number_Support_LP)*1.0*P1_XZMP_CON_E(-Del_XZMP_CON,X_ZMP - X_ZMP_OFFSET_BP - 1.*Ref_ZMP[0], 1);
	Old_Del_YZMP_CON_P1 = Del_YZMP_CON_P1;
	Old_Del_XZMP_CON_P1 = Del_XZMP_CON_P1;
	
	
	Del_XZMP_CON = 1.*(Del_XZMP_CON_P2 + Del_XZMP_CON_P1 );
	Del_YZMP_CON = 1.*(Del_YZMP_CON_P2 + Del_YZMP_CON_P1 ); 

	Old_Del_XZMP_CON = Del_XZMP_CON;
	Old_Del_YZMP_CON = Del_YZMP_CON;

	if(zero == 0){
		Old_Number_Support_LP = 2;
		Del_YZMP_CON_P2 = 0.; Del_YZMP_CON_P1 = 0.; Del_XZMP_CON_P2 = 0.; Del_XZMP_CON_P1 = 0.;
		Old_Del_YZMP_CON_P2 = 0.; Old_Del_YZMP_CON_P1 = 0.; Old_Del_XZMP_CON_P2 = 0.; Old_Del_XZMP_CON_P1 = 0.; 
		P2_YZMP_CON_E(0,0,0); 
		P2_YZMP_CON_E(0,0,0); 
		P1_XZMP_CON_E(0,0,0); 
		P1_XZMP_CON_E(0,0,0);
		Del_XZMP_CON = 0.; Del_YZMP_CON = 0.;  
	}
	
}

double TP_INT_CON(double Ref_ANG, double ANG, int zero)
{
	static double y;
	static double sume = 0.;
	const double KI = 0.0025;
		
	y = 0.0*(Ref_ANG - ANG) + KI*sume;
		
	sume += Ref_ANG - ANG;
	
	if(sume > 5.0/KI) sume = 5./KI;
	else if(sume < -5.0/KI) sume = -5./KI;

	if(y > 5.0) y = 5.0;
	else if(y < -5.0) y = -5.0;
	
	if(zero == 0) sume = 0.;

	return y;
}

double TR_INT_CON(double Ref_ANG, double ANG, int zero)
{
	static double y;
	static double sume = 0.;
	const double KI = 0.003;
		
	y = 0.0*(Ref_ANG - ANG) + KI*sume;
		
	sume += Ref_ANG - ANG;
	
	if(sume > 5.0/KI) sume = 5./KI;
	else if(sume < -5.0/KI) sume = -5./KI;

	if(y > 5.0) y = 5.0;
	else if(y < -5.0) y = -5.0;
	
	if(zero == 0) sume = 0.;

	return y;
}

double TP_INT_CON_WALK(double Ref_ANG, double ANG, int zero)
{
	static double y;
	static double sume = 0.;
	const double KI = 0.0005;
		
	y = 0.0*(Ref_ANG - ANG) + KI*sume;
		
	sume += Ref_ANG - ANG;
	
	if(sume > 5.0/KI) sume = 5./KI;
	else if(sume < -5.0/KI) sume = -5./KI;

	if(y > 5.0) y = 5.0;
	else if(y < -5.0) y = -5.0;
	
	if(zero == 0) sume = 0.;

	return y;
}

double TR_INT_CON_WALK(double Ref_ANG, double ANG, int zero)
{
	static double y;
	static double sume = 0.;
	const double KI = 0.01;  //0.001
		
	y = 0.0*(Ref_ANG - ANG) + KI*sume;
		
	sume += Ref_ANG - ANG;
	
	if(sume > 40.0/KI) sume = 40./KI;
	else if(sume < -40.0/KI) sume = -40./KI;

	if(y > 40.0) y = 40.0;
	else if(y < -40.0) y = -40.0;
	
	if(zero == 0) sume = 0.;

	return y;
}



void Kirk_con_test(void)
{

	//readZMP(0.5*(_pRF0_3x1[1] - _pRH0_3x1[1]) + 0.001*Ref_R1[0], _pRF0_3x1[2] + 0.001*Ref_R1[1], 
	//	    0.5*(_pLF0_3x1[1] - _pLH0_3x1[1]) + 0.001*Ref_L1[0], _pLF0_3x1[2] + 0.001*Ref_L1[1], 
	//		-0.5*(_pRF0_3x1[1] - _pRH0_3x1[1]) + 0.001*Ref_R2[0], _pRH0_3x1[2] + 0.001*Ref_R2[1], 
	//		-0.5*(_pLF0_3x1[1] - _pLH0_3x1[1]) + 0.001*Ref_L2[0], _pLH0_3x1[2] + 0.001*Ref_L2[1]);

	readZMP_biped(_pRF0_3x1[1] + 0.001*Ref_R1[0], _pRF0_3x1[2] + 0.001*Ref_R1[1], 
		          _pLF0_3x1[1] + 0.001*Ref_L1[0], _pLF0_3x1[2] + 0.001*Ref_L1[1]);

	// 발 가속도 센서 처리 (발바닥 측방향 가속도 고주파 통과 필터 + 적분기)
	RF_Y_acc_HPF_INT = (1. - 2.*PI*F_cutoff_acc_HPF*Del_T)*Old_RF_Y_acc_HPF_INT + Del_T*(FTSensor[RFFT].dAccRoll+0.0);
	Old_RF_Y_acc_HPF_INT = RF_Y_acc_HPF_INT;

	LF_Y_acc_HPF_INT = (1. - 2.*PI*F_cutoff_acc_HPF*Del_T)*Old_LF_Y_acc_HPF_INT + Del_T*(FTSensor[LFFT].dAccRoll+0.0);
	Old_LF_Y_acc_HPF_INT = LF_Y_acc_HPF_INT;

	RF_X_acc_HPF_INT = (1. - 2.*PI*F_cutoff_acc_HPF*Del_T)*Old_RF_X_acc_HPF_INT + Del_T*(FTSensor[RFFT].dAccPitch+0.0);
	Old_RF_X_acc_HPF_INT = RF_X_acc_HPF_INT;

	LF_X_acc_HPF_INT = (1. - 2.*PI*F_cutoff_acc_HPF*Del_T)*Old_LF_X_acc_HPF_INT + Del_T*(FTSensor[LFFT].dAccPitch+0.0);
	Old_LF_X_acc_HPF_INT = LF_X_acc_HPF_INT;


	if(pSharedMemory->P4_ZMP_CON_CNT < 5599 && pSharedMemory->P4_ZMP_CON_Flag) { 
		
		//적분식//
		//Del_YZMP_CON = 0.*0.85*P4_YZMP_CON_E(-Del_YZMP_CON,Y_ZMP, 1)*0.5*(1-cos(PI*(P4_ZMP_CON_CNT_2)/200));  
		//Del_XZMP_CON = 0.*0.75*P4_XZMP_CON_E(-Del_XZMP_CON,X_ZMP - X_ZMP_OFFSET, 1)*0.5*(1-cos(PI*(P4_ZMP_CON_CNT_2)/200));
		//Del_YZMP_CON = 0.982*P3H_YZMP_CON_E(-Del_YZMP_CON,Y_ZMP - 30., 1)*0.5*(1-cos(PI*(P4_ZMP_CON_CNT_2)/200));  
		//Del_XZMP_CON = 0.97*P3H_XZMP_CON_E(-Del_XZMP_CON,X_ZMP - X_ZMP_OFFSET, 1)*0.5*(1-cos(PI*(P4_ZMP_CON_CNT_2)/200));
		//Del_YZMP_CON = 0.991*P3F_YZMP_CON_E(-Del_YZMP_CON,Y_ZMP - 20., 1)*0.5*(1-cos(PI*(P4_ZMP_CON_CNT_2)/200));  
		//Del_XZMP_CON = 0.995*P3F_XZMP_CON_E(-Del_XZMP_CON,X_ZMP - X_ZMP_OFFSET, 1)*0.5*(1-cos(PI*(P4_ZMP_CON_CNT_2)/200));

		//Del_YZMP_CON = 1.0*P2_YZMP_CON_E(-Del_YZMP_CON,Y_ZMP, 1)*0.5*(1-cos(PI*(P4_ZMP_CON_CNT_2)/200));  
		//Del_XZMP_CON = 1.0*P2_XZMP_CON_E(-Del_XZMP_CON,X_ZMP - X_ZMP_OFFSET_BP, 1)*0.5*(1-cos(PI*(P4_ZMP_CON_CNT_2)/200));
		//Del_XZMP_CON = 1.0*P2_XZMP_CON(X_ZMP - X_ZMP_OFFSET_BP, 1)*0.5*(1-cos(PI*(P4_ZMP_CON_CNT_2)/200));
		//Del_XZMP_CON = -1.0*P2_XCOM_ZMP_CON_E(0., X_ZMP - X_ZMP_OFFSET_BP, 0., 1)*0.5*(1-cos(PI*(P4_ZMP_CON_CNT_2)/200));

		//Del_YZMP_CON = 1.*0.6*P2_YZMP_INT_CON_E(0.,Del_YZMP_CON,Y_ZMP, 1)*0.5*(1-cos(PI*(P4_ZMP_CON_CNT_2)/200));  
		//Del_XZMP_CON = 1.*0.6*P2_XZMP_INT_CON_E(X_ZMP_OFFSET_BP,Del_XZMP_CON,X_ZMP, 1)*0.5*(1-cos(PI*(P4_ZMP_CON_CNT_2)/200));


		//Del_YZMP_CON = 0.98*P4_YZMP_CON_E(-Del_YZMP_CON,Y_ZMP, 1)*0.5*(1-cos(PI*(P4_ZMP_CON_CNT_2)/200));  
		//Del_XZMP_CON = 1.*P4_XZMP_CON_E(-Del_XZMP_CON,X_ZMP - X_ZMP_OFFSET, 1)*0.5*(1-cos(PI*(P4_ZMP_CON_CNT_2)/200));
		Del_YZMP_CON = 1.0*P1_YZMP_CON_E(-Del_YZMP_CON,Y_ZMP + 40., 1)*0.5*(1-cos(PI*(P4_ZMP_CON_CNT_2)/200));  
		Del_XZMP_CON = 1.0*P1_XZMP_CON_E(-Del_XZMP_CON,X_ZMP - 7.0, 1)*0.5*(1-cos(PI*(P4_ZMP_CON_CNT_2)/200));
		
		
		Old_Del_YZMP_CON = Del_YZMP_CON;
		Old_Del_XZMP_CON = Del_XZMP_CON;


				
		// 발측/전진방향 진동제어, 몸통 Z 방향 진동제어 //
		
		Del_RHR_VIB_CON = -1.0*R2D*SSP_Y_VIB_CON_E_RF(D2R*Del_RHR_VIB_CON,0.006*RF_Y_acc_HPF_INT, 1); // [deg]

		Old_Del_RHR_VIB_CON = Del_RHR_VIB_CON;

		Del_RHP_VIB_CON = -1.0*R2D*SSP_X_VIB_CON_E_RF(D2R*Del_RHP_VIB_CON,0.006*RF_X_acc_HPF_INT, 1); // [deg]

		Old_Del_RHP_VIB_CON = Del_RHP_VIB_CON;

		Del_LHY_VIB_CON =  1.0*R2D*SSP_Z_VIB_CON_E(-D2R*Del_LHY_VIB_CON,D2R*IMUSensor[CENTERIMU].Yaw_Velocity, 1); // [Deg]

		Old_Del_LHY_VIB_CON = Del_LHY_VIB_CON;
		
			

		if(P4_ZMP_CON_CNT_2 < 200) P4_ZMP_CON_CNT_2++;
	}

	pSharedMemory->ZMP[0] = X_ZMP;
	pSharedMemory->ZMP[1] = Y_ZMP;

	_log_temp[0] = X_ZMP;
	_log_temp[1] = Y_ZMP;
	_log_temp[2] = Del_XZMP_CON;
	_log_temp[3] = Del_YZMP_CON;
	_log_temp[4] = FTSensor[RFFT].dAccRoll;
	_log_temp[5] = RF_Y_acc_HPF_INT;
	_log_temp[6] = Del_RHR_VIB_CON;
	_log_temp[7] = FTSensor[RFFT].dAccPitch;
	_log_temp[8] = RF_X_acc_HPF_INT;
	_log_temp[9] = Del_RHP_VIB_CON;
	_log_temp[10] = IMUSensor[CENTERIMU].Yaw_Velocity;
	_log_temp[11] = Del_LHY_VIB_CON;


	BC_Y_CON = -Del_YZMP_CON;
	BC_X_CON = -Del_XZMP_CON;


	_online_pattern.dangRFz = 0.f;
    _online_pattern.dangLFz = 0.f;
	_online_pattern.dangRHz = 0.f;
	_online_pattern.dangLHz = 0.f;

	_online_pattern.dpCOMx = 0.f;
	_online_pattern.dpCOMy = 0.f;

	_online_pattern.offset_RHR = Del_RHR_VIB_CON*D2R;
   	_online_pattern.offset_RHP = Del_RHP_VIB_CON*D2R;

	_online_pattern.offset_RAP = (Del_RAP_Init + Del_RLHP_Init)*D2R;
	_online_pattern.offset_RAR = Del_RAR_Init*D2R;

	_online_pattern.offset_LAP = Del_RLHP_Init*D2R;
	_online_pattern.offset_LAR = Del_LAR_Init*D2R;

	_online_pattern.offset_RHY = 0.;
	_online_pattern.offset_LHY = Del_LHY_VIB_CON*D2R;

    _online_pattern.dpRFz = 0.001*Ref_R1[2] + 0.001*Del_R1_Z_PosInit + 0.001*Del_RLFZ_Init;
    _online_pattern.dpLFz = 0.001*Ref_L1[2] - 0.001*Del_RLFZ_Init;;//+ 0.001*Del_LEG_Z_VSC[LF1];
    _online_pattern.dpRFy = 0.001*Ref_R1[1] - 0.001*Ref_BC[1] - 0.001*BC_Y_Init - 0.001*BC_Y_CON;  // update
    _online_pattern.dpLFy = 0.001*Ref_L1[1] - 0.001*Ref_BC[1] - 0.001*BC_Y_Init - 0.001*BC_Y_CON; // update
    _online_pattern.dpRFx = 0.001*Ref_R1[0] - 0.001*Ref_BC[0] - 0.001*BC_X_Init - 0.001*BC_X_CON; // update
    _online_pattern.dpLFx = 0.001*Ref_L1[0] - 0.001*Ref_BC[0] - 0.001*BC_X_Init - 0.001*BC_X_CON; // update   
    
    _online_pattern.dpRHz = 0.001*Ref_R2[2] + 0.001*Del_R2_Z_PosInit;//+ 0.001*Del_LEG_Z_VSC[RF2];
    _online_pattern.dpLHz = 0.001*Ref_L2[2] ;//+ 0.001*Del_LEG_Z_VSC[LF2];
    _online_pattern.dpRHy = 0.001*Ref_R2[1] - 0.001*Ref_BC[1] - 0.001*BC_Y_Init - 0.001*BC_Y_CON; // update
    _online_pattern.dpLHy = 0.001*Ref_L2[1] - 0.001*Ref_BC[1] - 0.001*BC_Y_Init - 0.001*BC_Y_CON; // update
    _online_pattern.dpRHx = 0.001*Ref_R2[0] - 0.001*Ref_BC[0] - 0.001*BC_X_Init - 0.001*BC_X_CON; // update
    _online_pattern.dpLHx = 0.001*Ref_L2[0] - 0.001*Ref_BC[0] - 0.001*BC_X_Init - 0.001*BC_X_CON; // update


	if(pSharedMemory->P4_ZMP_CON_CNT < 5600) pSharedMemory->P4_ZMP_CON_CNT++;
	else if(pSharedMemory->P4_ZMP_CON_CNT >= 5600 && pSharedMemory->P4_ZMP_CON_CNT <= 6000){

		Del_YZMP_CON = Old_Del_YZMP_CON - Old_Del_YZMP_CON*0.5*(1-cos(PI*(pSharedMemory->P4_ZMP_CON_CNT-5600)/400));
		Del_XZMP_CON = Old_Del_XZMP_CON - Old_Del_XZMP_CON*0.5*(1-cos(PI*(pSharedMemory->P4_ZMP_CON_CNT-5600)/400));

		Del_RHR_VIB_CON = Old_Del_RHR_VIB_CON - Old_Del_RHR_VIB_CON*0.5*(1-cos(PI*(pSharedMemory->P4_ZMP_CON_CNT-5600)/400));
		Del_RHP_VIB_CON = Old_Del_RHP_VIB_CON - Old_Del_RHP_VIB_CON*0.5*(1-cos(PI*(pSharedMemory->P4_ZMP_CON_CNT-5600)/400));
		Del_LHY_VIB_CON = Old_Del_LHY_VIB_CON - Old_Del_LHY_VIB_CON*0.5*(1-cos(PI*(pSharedMemory->P4_ZMP_CON_CNT-5600)/400));
		
		pSharedMemory->P4_ZMP_CON_CNT++;

	}
	else {
		Old_Del_YZMP_CON = 0.; Old_Del_XZMP_CON = 0.;
		Del_YZMP_CON = 0.; Del_XZMP_CON = 0.; BC_X_CON = 0., BC_Y_CON = 0.;
		pSharedMemory->P4_ZMP_CON_CNT = 0;  P4_ZMP_CON_CNT_2 = 0; 
		P4_YZMP_CON_E(0,0,0); P4_XZMP_CON_E(0,0,0);
		P3H_YZMP_CON_E(0,0,0); P3H_XZMP_CON_E(0,0,0);
		P3F_YZMP_CON_E(0,0,0); P3F_XZMP_CON_E(0,0,0);
		P2_YZMP_CON_E(0,0,0); P2_XZMP_CON_E(0,0,0); P2_XZMP_CON(0,0);
		SSP_Y_VIB_CON_E_RF(0,0,0); SSP_X_VIB_CON_E_RF(0,0,0); SSP_Z_VIB_CON_E(0,0,0);
		Old_Del_RHR_VIB_CON = 0.; Del_RHR_VIB_CON = 0.;
		Old_Del_RHP_VIB_CON = 0.; Del_RHP_VIB_CON = 0.;
		Old_Del_LHY_VIB_CON = 0.; Del_LHY_VIB_CON = 0.;
		P2_XCOM_ZMP_CON_E(0,0,0,0);
		//ZMP_Test_R1p = 0.; ZMP_Test_R2p = 0.; ZMP_Test_R3p = 0.,ZMP_Test_L1p = 0.; ZMP_Test_L2p = 0.; ZMP_Test_L3p = 0.;
		pSharedMemory->P4_ZMP_CON_Flag = 0;
	}
}

void Kirk_Posture_Init(void)
{

	readZMP(0.5*(_pRF0_3x1[1] - _pRH0_3x1[1]) + 0*Ref_R1[0], _pRF0_3x1[2] + 0*Ref_R1[1], 
		    0.5*(_pLF0_3x1[1] - _pLH0_3x1[1]) + 0*Ref_L1[0], _pLF0_3x1[2] + 0*Ref_L1[1], 
			-0.5*(_pRF0_3x1[1] - _pRH0_3x1[1]) + 0*Ref_R2[0], _pRH0_3x1[2] + 0*Ref_R2[1], 
			-0.5*(_pLF0_3x1[1] - _pLH0_3x1[1]) + 0*Ref_L2[0], _pLH0_3x1[2] + 0*Ref_L2[1]);

	pSharedMemory->ZMP[0] = X_ZMP;
	pSharedMemory->ZMP[1] = Y_ZMP;

	// ZMP 적분 제어기 //	
	Del_YZMP_CON = 0.5*P4_YZMP_INT_CON_E(0.,Del_YZMP_CON,Y_ZMP, 1)*0.5*(1-cos(PI*(Local_CNT_1)/100));  
	Del_XZMP_CON = 0.5*P4_XZMP_INT_CON_E(X_ZMP_OFFSET,Del_XZMP_CON,X_ZMP, 1)*0.5*(1-cos(PI*(Local_CNT_1)/100));

	Old_Del_YZMP_CON = Del_YZMP_CON;
	Old_Del_XZMP_CON = Del_XZMP_CON;

	// 수직 힘 제어기 //	
	Del_R1_Z_PosInit = -1.*0.5*(1-cos(PI*(Local_CNT_1)/100))*FORCE_DIFF_CON_R1(0., FTSensor[RFFT].Fz - FTSensor[LFFT].Fz, 1);
	Del_R2_Z_PosInit = -1.*0.5*(1-cos(PI*(Local_CNT_1)/100))*FORCE_DIFF_CON_R2(0., FTSensor[RWFT].Fz - FTSensor[LWFT].Fz, 1);

	// 발목 롤 토크 교정 //
	Del_RAR_Init = -1.*0.5*(1-cos(PI*(Local_CNT_1)/100))*TORQ_RAR_INT_CON(0.0,FTSensor[RFFT].Mx,1);
	Del_LAR_Init = -1.*0.5*(1-cos(PI*(Local_CNT_1)/100))*TORQ_LAR_INT_CON(0.0,FTSensor[LFFT].Mx,1);

	// 발목 피치 토크 교정 //
	Del_RAP_Init = 1.*0.5*(1-cos(PI*(Local_CNT_1)/100))*TORQ_DIFF_RAP_INT_CON(0,-FTSensor[RFFT].My + FTSensor[LFFT].My,1);
	
	BC_Y_Init = Del_YZMP_CON;
	BC_X_Init = Del_XZMP_CON;


	_online_pattern.dangRFz = 0.f;
    _online_pattern.dangLFz = 0.f;
	_online_pattern.dangRHz = 0.f;
	_online_pattern.dangLHz = 0.f;

	_online_pattern.dpCOMx = 0.f;
	_online_pattern.dpCOMy = 0.f;

	_online_pattern.offset_RAP = Del_RAP_Init*D2R;
	_online_pattern.offset_RAR = Del_RAR_Init*D2R;

	_online_pattern.offset_LAR = Del_LAR_Init*D2R;
        
    _online_pattern.dpRFz = 0.001*Ref_R1[2] + 0.001*Del_R1_Z_PosInit;//+ 0.001*Del_LEG_Z_VSC[RF1];
    _online_pattern.dpLFz = 0.001*Ref_L1[2] ;//+ 0.001*Del_LEG_Z_VSC[LF1];
    _online_pattern.dpRFy = 0.001*Ref_R1[1] - 0.001*Ref_BC[1] - 0.001*BC_Y_Init;
    _online_pattern.dpLFy = 0.001*Ref_L1[1] - 0.001*Ref_BC[1] - 0.001*BC_Y_Init;
    _online_pattern.dpRFx = 0.001*Ref_R1[0] - 0.001*Ref_BC[0] - 0.001*BC_X_Init;
    _online_pattern.dpLFx = 0.001*Ref_L1[0] - 0.001*Ref_BC[0] - 0.001*BC_X_Init;   
    
    _online_pattern.dpRHz = 0.001*Ref_R2[2] + 0.001*Del_R2_Z_PosInit;//+ 0.001*Del_LEG_Z_VSC[RF2];
    _online_pattern.dpLHz = 0.001*Ref_L2[2] ;//+ 0.001*Del_LEG_Z_VSC[LF2];
    _online_pattern.dpRHy = 0.001*Ref_R2[1] - 0.001*Ref_BC[1] - 0.001*BC_Y_Init;
    _online_pattern.dpLHy = 0.001*Ref_L2[1] - 0.001*Ref_BC[1] - 0.001*BC_Y_Init;
    _online_pattern.dpRHx = 0.001*Ref_R2[0] - 0.001*Ref_BC[0] - 0.001*BC_X_Init;
    _online_pattern.dpLHx = 0.001*Ref_L2[0] - 0.001*Ref_BC[0] - 0.001*BC_X_Init;


	if(Local_CNT_1 < 100) Local_CNT_1++;


}

void Kirk_Posture_Init_BP(void)
{

	readZMP_biped(_pRF0_3x1[1] + 0.001*Ref_R1[0], _pRF0_3x1[2] + 0.001*Ref_R1[1], 
		          _pLF0_3x1[1] + 0.001*Ref_L1[0], _pLF0_3x1[2] + 0.001*Ref_L1[1]);


	pSharedMemory->ZMP[0] = X_ZMP;
	pSharedMemory->ZMP[1] = Y_ZMP;

	// ZMP 적분 제어기 //	
	Del_YZMP_CON = 0.6*P2_YZMP_INT_CON_E(0.,Del_YZMP_CON,Y_ZMP, 1)*0.5*(1-cos(PI*(Local_CNT_1)/100));  
	Del_XZMP_CON = 0.8*P2_XZMP_INT_CON_E(X_ZMP_OFFSET_BP,Del_XZMP_CON,X_ZMP, 1)*0.5*(1-cos(PI*(Local_CNT_1)/100));

	Old_Del_YZMP_CON = Del_YZMP_CON;
	Old_Del_XZMP_CON = Del_XZMP_CON;

	
	// 발목 롤 토크 교정 //
	Del_RAR_Init = -1.*0.5*(1-cos(PI*(Local_CNT_1)/100))*TORQ_RAR_INT_CON(0.0,FTSensor[RFFT].Mx,1);
	Del_LAR_Init = -1.*0.5*(1-cos(PI*(Local_CNT_1)/100))*TORQ_LAR_INT_CON(0.0,FTSensor[LFFT].Mx,1);

	// 발목 피치 토크 교정 //
	Del_RAP_Init = 1.*0.5*(1-cos(PI*(Local_CNT_1)/100))*TORQ_DIFF_RAP_INT_CON(0,-FTSensor[RFFT].My + FTSensor[LFFT].My,1);

	// 상체 기울기 교정 //
	Del_RLHP_Init = -TP_INT_CON(0.1,IMUSensor[CENTERIMU].Pitch,1);   // 가속도계에 의한 경사만으로 상체 피치 기울기 교정 //
	Del_RLFZ_Init =  TR_INT_CON(-0.6,IMUSensor[CENTERIMU].Roll,1);    // 가속도계에 의한 경사만으로 상체 롤 기울기 교정 //
	
	BC_Y_Init = Del_YZMP_CON;
	BC_X_Init = Del_XZMP_CON;


	_online_pattern.dangRFz = 0.f;
    _online_pattern.dangLFz = 0.f;
	_online_pattern.dangRHz = 0.f;
	_online_pattern.dangLHz = 0.f;

	_online_pattern.dpCOMx = 0.f;
	_online_pattern.dpCOMy = 0.f;

	_online_pattern.offset_RAP = (Del_RAP_Init + Del_RLHP_Init)*D2R;
	_online_pattern.offset_RAR = Del_RAR_Init*D2R;

	_online_pattern.offset_LAP = Del_RLHP_Init*D2R;
	_online_pattern.offset_LAR = Del_LAR_Init*D2R;
	//_online_pattern.offset_RLHP = Del_RLHP_Init*D2R;
       
    _online_pattern.dpRFz = 0.001*Ref_R1[2] + 0.001*Del_RLFZ_Init;
    _online_pattern.dpLFz = 0.001*Ref_L1[2] - 0.001*Del_RLFZ_Init;
    _online_pattern.dpRFy = 0.001*Ref_R1[1] - 0.001*Ref_BC[1] - 0.001*BC_Y_Init;
    _online_pattern.dpLFy = 0.001*Ref_L1[1] - 0.001*Ref_BC[1] - 0.001*BC_Y_Init;
    _online_pattern.dpRFx = 0.001*Ref_R1[0] - 0.001*Ref_BC[0] - 0.001*BC_X_Init;
    _online_pattern.dpLFx = 0.001*Ref_L1[0] - 0.001*Ref_BC[0] - 0.001*BC_X_Init;   
    
    _online_pattern.dpRHz = 0.001*Ref_R2[2] ;
    _online_pattern.dpLHz = 0.001*Ref_L2[2] ;
    _online_pattern.dpRHy = 0.001*Ref_R2[1] - 0.001*Ref_BC[1] - 0.001*BC_Y_Init;
    _online_pattern.dpLHy = 0.001*Ref_L2[1] - 0.001*Ref_BC[1] - 0.001*BC_Y_Init;
    _online_pattern.dpRHx = 0.001*Ref_R2[0] - 0.001*Ref_BC[0] - 0.001*BC_X_Init;
    _online_pattern.dpLHx = 0.001*Ref_L2[0] - 0.001*Ref_BC[0] - 0.001*BC_X_Init;


	if(Local_CNT_1 < 100) Local_CNT_1++;


}

void Go_Home_Posture(void)
{

	Del_BC_X_Home = BC_X_Init + BC_X_CON - (BC_X_Init + BC_X_CON)*0.5*(1-cos(PI*(Local_CNT_2)/600));  //update
	Del_BC_Y_Home = BC_Y_Init + BC_Y_CON - (BC_Y_Init + BC_Y_CON)*0.5*(1-cos(PI*(Local_CNT_2)/600));  //update	 
	Del_R1_Z_Home = Ref_R1[2] + Del_R1_Z_PosInit + Del_RLFZ_Init - (Ref_R1[2] + Del_R1_Z_PosInit + Del_RLFZ_Init)*0.5*(1-cos(PI*(Local_CNT_2)/600));
	Del_R2_Z_Home = Ref_R2[2] + Del_R2_Z_PosInit - (Ref_R2[2] + Del_R2_Z_PosInit)*0.5*(1-cos(PI*(Local_CNT_2)/600));
	Del_L1_Z_Home = Ref_L1[2] - Del_RLFZ_Init - (Ref_L1[2] - Del_RLFZ_Init)*0.5*(1-cos(PI*(Local_CNT_2)/600));
	Del_L2_Z_Home = Ref_L2[2] - (Ref_L2[2])*0.5*(1-cos(PI*(Local_CNT_2)/600));
	Del_RAR_Home = Del_RAR_Init + Del_RAR_RLAR_TORQ_CON + Del_RLAR_Posture_Walking - (Del_RAR_Init + Del_RAR_RLAR_TORQ_CON + Del_RLAR_Posture_Walking)*0.5*(1-cos(PI*(Local_CNT_2)/600)); 
	Del_LAR_Home = Del_LAR_Init + Del_LAR_RLAR_TORQ_CON + Del_RLAR_Posture_Walking - (Del_LAR_Init + Del_LAR_RLAR_TORQ_CON + Del_RLAR_Posture_Walking)*0.5*(1-cos(PI*(Local_CNT_2)/600)); 
	Del_RAP_Home = Del_RAP_Init + Del_RAP_RLAP_TORQ_CON + Del_RLHP_Init - (Del_RAP_Init + Del_RAP_RLAP_TORQ_CON + Del_RLHP_Init)*0.5*(1-cos(PI*(Local_CNT_2)/600)); 
	Del_LAP_Home = Del_LAP_RLAP_TORQ_CON + Del_RLHP_Init - (Del_LAP_RLAP_TORQ_CON + Del_RLHP_Init)*0.5*(1-cos(PI*(Local_CNT_2)/600)); 
	//Del_RLHP_Home = Del_RLHP_Init - (Del_RLHP_Init)*0.5*(1-cos(PI*(Local_CNT_2)/600));
	Del_RHR_Home = Del_RHR_VIB_CON - Del_RHR_VIB_CON*0.5*(1-cos(PI*(Local_CNT_2)/600));
	Del_LHR_Home = Del_LHR_VIB_CON - Del_LHR_VIB_CON*0.5*(1-cos(PI*(Local_CNT_2)/600));
	Del_RHP_Home = Del_RHP_VIB_CON + Del_RLHP_Posture_Walking - (Del_RHP_VIB_CON + Del_RLHP_Posture_Walking)*0.5*(1-cos(PI*(Local_CNT_2)/600));
	Del_LHP_Home = Del_LHP_VIB_CON + Del_RLHP_Posture_Walking - (Del_LHP_VIB_CON + Del_RLHP_Posture_Walking)*0.5*(1-cos(PI*(Local_CNT_2)/600));


	_online_pattern.dangRFz = 0.f;
    _online_pattern.dangLFz = 0.f;
	_online_pattern.dangRHz = 0.f;
	_online_pattern.dangLHz = 0.f;

	_online_pattern.dpCOMx = 0.f;
	_online_pattern.dpCOMy = 0.f;

	_online_pattern.offset_RAP = Del_RAP_Home*D2R;
	_online_pattern.offset_RAR = Del_RAR_Home*D2R;

	//_online_pattern.offset_RLHP = Del_RLHP_Home*D2R;

	_online_pattern.offset_LAP = Del_LAP_Home*D2R;
	_online_pattern.offset_LAR = Del_LAR_Home*D2R;

	_online_pattern.offset_RHR = Del_RHR_Home*D2R;
	_online_pattern.offset_LHR = Del_LHR_Home*D2R;

	_online_pattern.offset_RHP = Del_RHP_Home*D2R;
	_online_pattern.offset_LHP = Del_LHP_Home*D2R;
            
    _online_pattern.dpRFz = 0.001*Del_R1_Z_Home;
    _online_pattern.dpLFz = 0.001*Del_L1_Z_Home;
    _online_pattern.dpRFy = 0.001*Ref_R1[1] - 0.001*Ref_BC[1] - 0.001*Del_BC_Y_Home;
    _online_pattern.dpLFy = 0.001*Ref_L1[1] - 0.001*Ref_BC[1] - 0.001*Del_BC_Y_Home;
    _online_pattern.dpRFx = 0.001*Ref_R1[0] - 0.001*Ref_BC[0] - 0.001*Del_BC_X_Home;
    _online_pattern.dpLFx = 0.001*Ref_L1[0] - 0.001*Ref_BC[0] - 0.001*Del_BC_X_Home;   
    
    _online_pattern.dpRHz = 0.001*Del_R2_Z_Home;
    _online_pattern.dpLHz = 0.001*Del_L2_Z_Home;
    _online_pattern.dpRHy = 0.001*Ref_R2[1] - 0.001*Ref_BC[1] - 0.001*Del_BC_Y_Home;
    _online_pattern.dpLHy = 0.001*Ref_L2[1] - 0.001*Ref_BC[1] - 0.001*Del_BC_Y_Home;
    _online_pattern.dpRHx = 0.001*Ref_R2[0] - 0.001*Ref_BC[0] - 0.001*Del_BC_X_Home;
    _online_pattern.dpLHx = 0.001*Ref_L2[0] - 0.001*Ref_BC[0] - 0.001*Del_BC_X_Home;

	if(Local_CNT_2 < 600)
		Local_CNT_2++;
	else
	{
		Local_CNT_1 = 0; Local_CNT_2 = 0;
		BC_X_Init = 0.; BC_Y_Init = 0.; Del_BC_X_Home = 0.; Del_BC_Y_Home = 0.;
		Del_R1_Z_PosInit = 0.; Del_R2_Z_PosInit = 0.; Del_R1_Z_Home = 0.; Del_R2_Z_Home = 0.;
		Del_RLHP_Init = 0.; Del_RLFZ_Init = 0.; Del_RLHP_Home = 0.; 
		BC_X_CON = 0.; BC_Y_CON = 0.; // update
		Ref_R1[2] = 0.; Ref_R2[2] = 0.; Ref_R3[2] = 0., Ref_L1[2] = 0.; Ref_L2[2] = 0.; Ref_L3[2] = 0.;
		
		ZMP_CON_MANAGER(0, 0, 0, 0, 0, 0, 0);
		
		for(i=0;i<6;i++) {
				Early_Landing_Flag[i] = 0;
				Late_Landing_Flag[i] = 0;
				temp_Z[i] = 0.;
				LL_Del_z[i] = 0.; Old_LL_Del_z[i] = 0.; LL_Del_z_CNT[i] = 0;
				//Late_Land_Vertical_Disp(i,0,0,0);
		}
		All_Foot_Z = 0.; Old_All_Foot_Z = 0.;
		All_Foot_Z_Flag = 0; All_Foot_Z_CNT = 0;

		Del_PC_X_SSP_XZMP_CON = 0.; Del_PC_Y_SSP_YZMP_CON = 0.;    // SSP ZMP
		Del_PC_X_DSP_XZMP_CON = 0.; Del_PC_Y_DSP_YZMP_CON = 0.;

		Ref_CON_SUM_X_ZMP_ERR = 0.; Ref_CON_SUM_Y_ZMP_ERR = 0.;

		CNT_CON_SUM_XY_ZMP_ERR = 0;

		Del_LAR_RLAR_TORQ_CON = 0.; Del_LAP_RLAP_TORQ_CON = 0.; Del_RAR_RLAR_TORQ_CON = 0.; Del_RAP_RLAP_TORQ_CON = 0.;

		Del_RHR_VIB_CON = 0.; Del_LHR_VIB_CON = 0.; Del_RHP_VIB_CON = 0.; Del_LHP_VIB_CON = 0.; Del_RHY_VIB_CON = 0.; Del_LHY_VIB_CON = 0.;

		Del_RLHP_Posture_Walking = 0.; Del_RLFZ_Posture_Walking = 0.; Del_RLAR_Posture_Walking = 0.;

		Old_Del_PC_X_SSP_XZMP_CON_2 = 0.; Old_Del_PC_Y_SSP_YZMP_CON_2 = 0.; CNT_final_gain_SSP_ZMP_CON = 0;

		adj_amp = 0.;

		P4_XZMP_INT_CON_E(0,0,0,0);
		P4_YZMP_INT_CON_E(0,0,0,0);
		P2_XZMP_INT_CON_E(0,0,0,0);
		P2_YZMP_INT_CON_E(0,0,0,0);
		P4_XZMP_CON_E(0,0,0);
		P4_YZMP_CON_E(0,0,0);
		P3H_XZMP_CON_E(0,0,0);
		P3H_YZMP_CON_E(0,0,0);
		P3F_XZMP_CON_E(0,0,0);
		P3F_YZMP_CON_E(0,0,0);
		P2_XZMP_CON_E(0,0,0);
		P2_YZMP_CON_E(0,0,0);
		P1_XZMP_CON_E(0,0,0);
		P1_YZMP_CON_E(0,0,0);
		FORCE_DIFF_CON_R1(0,0,0);
		FORCE_DIFF_CON_R2(0,0,0);
		TORQ_RAR_INT_CON(0,0,0);
		TORQ_LAR_INT_CON(0,0,0);
		TORQ_DIFF_RAP_INT_CON(0,0,0);
		ZMP_CON_MANAGER(0,0,0,0,0,0,0); 
		TP_INT_CON(0,0,0);
		TR_INT_CON(0,0,0);
		ZMP_CON_MANAGER_BP(0,0,0,0,0);
		RLAR_TORQ_CON(0,0); RLAP_TORQ_CON(0,0);
		SSP_Y_VIB_CON_E_RF(0,0,0); SSP_Y_VIB_CON_E_LF(0,0,0); SSP_X_VIB_CON_E_RF(0,0,0); SSP_X_VIB_CON_E_LF(0,0,0);
		TR_INT_CON_WALK(0,0,0); TP_INT_CON_WALK(0,0,0);

		pSharedMemory->kirk_flag = 0;
	}
}

double Late_Land_Vertical_Disp(unsigned char joint_num, double Ref_Force, double Force, int zero){

	static double del_z[6];
	static double sume[6] = {0.,0.,0.,0.,0.,0.};
	const double  KI = 0.01;
		
	del_z[joint_num] = 0.0*(Ref_Force - Force) + KI*sume[joint_num];
		
	sume[joint_num] += (Ref_Force - Force);
	
	
	if(sume[joint_num] > LATE_LAND_MAX_DEPTH/KI) sume[joint_num] = LATE_LAND_MAX_DEPTH/KI;
	else if(sume[joint_num] < 0) sume[joint_num] = 0;

	if(del_z[joint_num] > LATE_LAND_MAX_DEPTH) del_z[joint_num] = LATE_LAND_MAX_DEPTH;
	else if(del_z[joint_num] < 0.0) del_z[joint_num] = 0.0;
	

	if(zero == 0) sume[joint_num] = 0.;

	return del_z[joint_num];

}

void Kirk_online(void)
{

	Fz[RF_1] = FTSensor[RFFT].Fz; Fz[LF_1] = FTSensor[LFFT].Fz;
	Fz[RF_2] = FTSensor[RWFT].Fz; Fz[LF_2] = FTSensor[LWFT].Fz;


	if(Lss == 0 && Rot_Ang == 0 && Step_CNT == pSharedMemory->Step_Number) pSharedMemory->Go_flag = 0;
    else if((Lss != 0 || Rot_Ang != 0) && Step_CNT == 2*pSharedMemory->Step_Number-1) pSharedMemory->Go_flag = 0;
    
    
    // ?? ?? ???? ?? //
    if(Change_flag == 1 || pSharedMemory->First_flag == 1){
    
      
		//Ls += 20; 
		//Rot_Ang += 2.5;
		//Delay_Ratio += 0.05;
		//Td = (int)(Ts*Delay_Ratio);

		
		Ls = pSharedMemory->Global_Ls;
		Lss = pSharedMemory->Global_Lss;
		Rot_Ang = pSharedMemory->Global_Rot_Ang;
		Delay_Ratio = pSharedMemory->Global_Delay_Ratio;
		Td = (int)(Ts*Delay_Ratio);
		Hs = pSharedMemory->Global_Hs;
		Ab = pSharedMemory->Global_Ab;

		if(pSharedMemory->First_flag == 1){
			Old_Lss = 0.; Old_Ls = 0.; Old_Rot_Ang = 0.;
		}
		


		if(Delay_Ratio < 0) {Delay_Ratio = 0. ; pSharedMemory->Global_Delay_Ratio = 0.;}


      
		  if(Old_Ls*Ls < 0) {  // ?? ?? ? Ls = 0 ? ?? ? ?? ??
			Ls = 0;
		  }

		  if(Old_Lss*Lss < 0) {  // ?? ?? ? Lss = 0 ? ?? ? ?? ??
			Lss = 0;
		  }
		  
		  if(Old_Rot_Ang*Rot_Ang < 0) {  // ?? ?? ? Rot_Ang = 0 ? ?? ? ?? ??
			Rot_Ang = 0;
		  }
		  
		  if(Rot_Ang*Lss < 0){ // ????? ??? ????? ??? ??? ??? ?. 
			  if(Lss != Old_Lss) Rot_Ang = 0.;
			  else if(Rot_Ang != Old_Rot_Ang) Lss = 0.;          
		  }

		  Old_Lss = Lss;
		  Old_Ls = Ls;	
		  Old_Rot_Ang = Rot_Ang;

		  pSharedMemory->Change_OK = 1;
		  pSharedMemory->First_flag =  0;
        
    }
        
    Quadruped_walking_pattern(pSharedMemory->Go_flag,Ts,Td,shape_factor_F,shape_factor_S,Ab,Hs,Ls,Lss,Rot_Ang,Early_Landing_Flag,&Step_CNT,Ref_BC,Ref_R1,Ref_R2,Ref_R3,Ref_L1,Ref_L2,Ref_L3,Ref_th,Ref_ZMP,Walking_stage,&Change_flag,test_var);
    

	 // Early Landing Algorithm //
     
     if(Walking_stage[RF_1] == 20 && FTSensor[RFFT].Fz > EARLY_LAND_THRES_1) Early_Landing_Flag[RF_1] = 1; 			
	
	 if(Walking_stage[RF_2] == 20 && FTSensor[RWFT].Fz > EARLY_LAND_THRES_2)  {		
			Early_Landing_Flag[RF_2] = 1; 			
			First_Flag_1 = 0;	
	 }


	 if(Walking_stage[LF_1] == 20 && FTSensor[LFFT].Fz > EARLY_LAND_THRES_1) Early_Landing_Flag[LF_1] = 1; 			

	 if(Walking_stage[LF_2] == 20 && FTSensor[LWFT].Fz > EARLY_LAND_THRES_2) Early_Landing_Flag[LF_2] = 1; 	


	 
	 if(Old_Walking_stage[LF_2] == 20 && Walking_stage[LF_2] == 0) {
			
			temp_Z_avg = (temp_Z[RF_1] + temp_Z[LF_2])/2;

			All_Foot_Z_Flag = 1;
     }
     else if(Old_Walking_stage[RF_2] == 20 && Walking_stage[RF_2] == 0) {
			
			temp_Z_avg = (temp_Z[LF_1] + temp_Z[RF_2])/2;

			All_Foot_Z_Flag = 1;
     }

     if(All_Foot_Z_Flag ){
		   All_Foot_Z = Old_All_Foot_Z + (temp_Z_avg - Old_All_Foot_Z)*0.5*(1-cos(PI*All_Foot_Z_CNT/100));

		   if(All_Foot_Z_CNT < 100) All_Foot_Z_CNT++;
		   else{
			Old_All_Foot_Z = All_Foot_Z;
			All_Foot_Z_Flag = 0;
			All_Foot_Z_CNT = 0;
		   }
     }


	// 늦은 착지 알고리즘 //
	for( i = 0; i<6; i++){
			if(Old_Walking_stage[i] == 20 && Walking_stage[i] == 0 && Early_Landing_Flag[i] == 0 && LL_Del_z[i] == 0. ) {

				Late_Landing_Flag[i] = 1;

			}		

			if((i == RF_1 || i == LF_1) && Late_Landing_Flag[i] == 1 && Fz[i] < EARLY_LAND_THRES_1 && LL_Del_z[i] < LATE_LAND_MAX_DEPTH && Walking_stage[i] == 0){
						
						LL_Del_z[i] = Late_Land_Vertical_Disp(i,(EARLY_LAND_THRES_1), Fz[i], 1);	
						Old_LL_Del_z[i] = LL_Del_z[i];
						LL_Del_z_CNT[i] = 0;
					
			}
			else if((i == RF_1 || i == LF_1) && Late_Landing_Flag[i] == 1 && Fz[i] >= EARLY_LAND_THRES_1 && Walking_stage[i] == 0){

						Late_Landing_Flag[i] = 2;
			}
			else if((i == RF_2 || i == LF_2) && Late_Landing_Flag[i] == 1 && Fz[i] < EARLY_LAND_THRES_2 && LL_Del_z[i] < LATE_LAND_MAX_DEPTH && Walking_stage[i] == 0){
						
						LL_Del_z[i] = Late_Land_Vertical_Disp(i,(EARLY_LAND_THRES_2), Fz[i], 1);	
						Old_LL_Del_z[i] = LL_Del_z[i];
						LL_Del_z_CNT[i] = 0;

						if(i == RF_2 && LL_Del_z[i] == LATE_LAND_MAX_DEPTH) First_Flag_1 = 0;
					
			}
			else if((i == RF_2 || i == LF_2) && Late_Landing_Flag[i] == 1 && Fz[i] >= EARLY_LAND_THRES_2 && Walking_stage[i] == 0){

						Late_Landing_Flag[i] = 2;

						if(i == RF_2) First_Flag_1 = 0;

			}
			else if((Late_Landing_Flag[i] == 1 || Late_Landing_Flag[i] == 2 ) && Walking_stage[i] == 10){

						LL_Del_z[i] = Old_LL_Del_z[i] - Old_LL_Del_z[i]*0.5*(1-cos(PI*LL_Del_z_CNT[i]/40));

						if(LL_Del_z_CNT[i] < 40 && WP_Flow_Control_Flag == 0) LL_Del_z_CNT[i]++;
						else if(WP_Flow_Control_Flag == 0){
							Late_Landing_Flag[i] = 0;
							Late_Land_Vertical_Disp(i,0,0,0);
							Old_LL_Del_z[i] = 0.;
							LL_Del_z[i] = 0.;	
						}
			}
		}

	// 늦은 착지에 의한 보행 흐름 제어 알고리즘 : 하강하는 다리가 지면과의 접촉이 일어난 후 보행을 계속 진행 함.
	if(Late_Landing_Flag[RF_1] == 1 && Old_LL_Del_z[RF_1] < LATE_LAND_MAX_DEPTH ) WP_Flow_Control_Flag = 1;
	else if(Late_Landing_Flag[RF_2] == 1 && Old_LL_Del_z[RF_2] < LATE_LAND_MAX_DEPTH ) WP_Flow_Control_Flag = 1;
	else if(Late_Landing_Flag[LF_1] == 1 && Old_LL_Del_z[LF_1] < LATE_LAND_MAX_DEPTH ) WP_Flow_Control_Flag = 1;
	else if(Late_Landing_Flag[LF_2] == 1 && Old_LL_Del_z[LF_2] < LATE_LAND_MAX_DEPTH ) WP_Flow_Control_Flag = 1;
	else WP_Flow_Control_Flag = 0;
	 
        

	readZMP(0.5*(_pRF0_3x1[1] - _pRH0_3x1[1]) + 0.001*Ref_R1[0], _pRF0_3x1[2] + 0.001*Ref_R1[1], 
		    0.5*(_pLF0_3x1[1] - _pLH0_3x1[1]) + 0.001*Ref_L1[0], _pLF0_3x1[2] + 0.001*Ref_L1[1], 
			-0.5*(_pRF0_3x1[1] - _pRH0_3x1[1]) + 0.001*Ref_R2[0], _pRH0_3x1[2] + 0.001*Ref_R2[1], 
			-0.5*(_pLF0_3x1[1] - _pLH0_3x1[1]) + 0.001*Ref_L2[0], _pLH0_3x1[2] + 0.001*Ref_L2[1]);

	ZMP_CON_MANAGER(FTSensor[RFFT].Fz, FTSensor[RWFT].Fz, FTSensor[LFFT].Fz, FTSensor[LWFT].Fz, Walking_stage[RF_1], Walking_stage[LF_1], 1);   // update

	BC_Y_CON = -Del_YZMP_CON;  // update
	BC_X_CON = -Del_XZMP_CON;  // update

	pSharedMemory->ZMP[0] = X_ZMP;
	pSharedMemory->ZMP[1] = Y_ZMP;


	for(i=0;i<6;i++) Old_Walking_stage[i] = Walking_stage[i];

	if(0){ //no control

		BC_X_CON = 0.;
		BC_Y_CON = 0.;
		WP_Flow_Control_Flag = 0;

		for(i=0;i<6;i++) {
				Early_Landing_Flag[i] = 0;
				Late_Landing_Flag[i] = 0;
				temp_Z[i] = 0.;
				LL_Del_z[i] = 0.; Old_LL_Del_z[i] = 0.; LL_Del_z_CNT[i] = 0;
				//Late_Land_Vertical_Disp(i,0,0,0);
		}
	}


    // ?? ?? ??? ???? ??//
    Rot_th_R1 = Ref_th[0] - Ref_th[6]; // [deg]
    Rot_th_L1 = Ref_th[3] - Ref_th[6]; // [deg]
    Rot_th_R2 = Ref_th[4] - Ref_th[6]; // [deg]
    Rot_th_L2 = Ref_th[1] - Ref_th[6]; // [deg]
   
    del_x_R1 = Radius_R1*sin(D2R*(Init_R1_th + Rot_th_R1))  - 0.5*(_pRF0_3x1[1] - _pRH0_3x1[1]); 
    del_y_R1 = -Radius_R1*cos(D2R*(Init_R1_th + Rot_th_R1)) - _pRF0_3x1[2]; 
    del_x_L1 = Radius_L1*sin(D2R*(Init_L1_th + Rot_th_L1))  - 0.5*(_pLF0_3x1[1] - _pLH0_3x1[1]); 
    del_y_L1 = -Radius_L1*cos(D2R*(Init_L1_th + Rot_th_L1)) - _pLF0_3x1[2]; 
    
    del_x_R2 = Radius_R2*sin(D2R*(Init_R2_th + Rot_th_R2)) +  0.5*(_pRF0_3x1[1] - _pRH0_3x1[1]); 
    del_y_R2 = -Radius_R2*cos(D2R*(Init_R2_th + Rot_th_R2)) - _pRH0_3x1[2]; 
    del_x_L2 = Radius_L2*sin(D2R*(Init_L2_th + Rot_th_L2))  + 0.5*(_pLF0_3x1[1] - _pLH0_3x1[1]); 
    del_y_L2 = -Radius_L2*cos(D2R*(Init_L2_th + Rot_th_L2)) - _pLH0_3x1[2]; 
    
    
    _online_pattern.dangRFz = D2R*Rot_th_R1;
    _online_pattern.dangLFz = D2R*Rot_th_L1;
	_online_pattern.dangRHz = 0.f;
	_online_pattern.dangLHz = 0.f;

	_online_pattern.dpCOMx = 0.f;
	_online_pattern.dpCOMy = 0.f;
            
   
	_online_pattern.dpRFz = 0.001*Ref_R1[2] + 0.001*Del_R1_Z_PosInit;//+ 0.001*Del_LEG_Z_VSC[RF1];
    _online_pattern.dpLFz = 0.001*Ref_L1[2] ;//+ 0.001*Del_LEG_Z_VSC[LF1];
    _online_pattern.dpRFy = 0.001*Ref_R1[1] - 0.001*Ref_BC[1] + del_y_R1 - 0.001*BC_Y_Init - 0.001*BC_Y_CON;  // update
    _online_pattern.dpLFy = 0.001*Ref_L1[1] - 0.001*Ref_BC[1] + del_y_L1 - 0.001*BC_Y_Init - 0.001*BC_Y_CON; // update
    _online_pattern.dpRFx = 0.001*Ref_R1[0] - 0.001*Ref_BC[0] + del_x_R1 - 0.001*BC_X_Init - 0.001*BC_X_CON; // update
    _online_pattern.dpLFx = 0.001*Ref_L1[0] - 0.001*Ref_BC[0] + del_x_L1 - 0.001*BC_X_Init - 0.001*BC_X_CON; // update   
    
    _online_pattern.dpRHz = 0.001*Ref_R2[2] + 0.001*Del_R2_Z_PosInit;//+ 0.001*Del_LEG_Z_VSC[RF2];
    _online_pattern.dpLHz = 0.001*Ref_L2[2] ;//+ 0.001*Del_LEG_Z_VSC[LF2];
    _online_pattern.dpRHy = 0.001*Ref_R2[1] - 0.001*Ref_BC[1] + del_y_R2 - 0.001*BC_Y_Init - 0.001*BC_Y_CON; // update
    _online_pattern.dpLHy = 0.001*Ref_L2[1] - 0.001*Ref_BC[1] + del_y_L2 - 0.001*BC_Y_Init - 0.001*BC_Y_CON; // update
    _online_pattern.dpRHx = 0.001*Ref_R2[0] - 0.001*Ref_BC[0] + del_x_R2 - 0.001*BC_X_Init - 0.001*BC_X_CON; // update
    _online_pattern.dpLHx = 0.001*Ref_L2[0] - 0.001*Ref_BC[0] + del_x_L2 - 0.001*BC_X_Init - 0.001*BC_X_CON; // update

	_online_pattern.dqWST = (_online_pattern.dpRHz - _online_pattern.dpLHz)/0.1f*10.f*D2R;

	if(_online_pattern.dqWST > 10.f*D2R)
		_online_pattern.dqWST = 10.f*D2R;
	else if(_online_pattern.dqWST < -10.f*D2R)
		_online_pattern.dqWST = -10.f*D2R;
	
	_log_temp[0] = 0.001*X_ZMP;
	_log_temp[1] = 0.001*Y_ZMP;
	_log_temp[2] = 0.001*Ref_BC[0];
	_log_temp[3] = 0.001*Ref_BC[1];
	_log_temp[4] = 0.001*BC_X_CON;
	_log_temp[5] = 0.001*BC_Y_CON;

	_log_temp[6] = 0.001*Ref_ZMP[0];
	_log_temp[7] = 0.001*Ref_ZMP[1];
	_log_temp[8] = 0.001*Ref_R1[2];
	_log_temp[9] = 0.001*Ref_L1[2];
	_log_temp[10] = 0.001*Ref_R2[2];
	_log_temp[11] = 0.001*Ref_L2[2];
}


void Kirk_online_BP(void)
{

	Fz[RF_1] = FTSensor[RFFT].Fz; Fz[LF_1] = FTSensor[LFFT].Fz;
	Fz[RF_2] = 0.; Fz[LF_2] = 0.;


	/* 착지 Threshold 설정 (로봇 총 무게 : 42 kgf ) */
	if(Fz[LF_1] > 98 && LandLOK_Flag == 0) LandLOK_Flag = 1; // 왼발 완전 착지 기준 ( 10kg )

	if(Fz[RF_1] > 98 && LandROK_Flag == 0) LandROK_Flag = 1; // 오른발 완전 착지 기준 ( 10kg )

	if(Fz[LF_1] > 29 && ThreeKG_LOK_Flag == 0) ThreeKG_LOK_Flag = 1; // 왼발 터치 다운 기준 ( 3kg )

	if(Fz[RF_1] > 29 && ThreeKG_ROK_Flag == 0) ThreeKG_ROK_Flag = 1; // 오른발 터치 다운 기준 ( 3kg )

	if(Fz[LF_1] > 15 && OneKG_LOK_Flag == 0) OneKG_LOK_Flag = 1; // 왼발 터치 다운 기준 ( 1kg )

	if(Fz[RF_1] > 15 && OneKG_ROK_Flag == 0) OneKG_ROK_Flag = 1; // 오른발 터치 다운 기준 ( 1kg )


	// 보행 중지 설정 //
	if(Lss == 0 && Rot_Ang == 0 && Step_CNT == pSharedMemory->Step_Number) pSharedMemory->Go_flag = 0;
    else if((Lss != 0 || Rot_Ang != 0) && Step_CNT == 2*pSharedMemory->Step_Number-1) pSharedMemory->Go_flag = 0;
    
    
    // ?? ?? ???? ?? //
    if(Change_flag == 1 || pSharedMemory->First_flag == 1){
    
      
		//Ls += 20; 
		//Rot_Ang += 2.5;
		//Delay_Ratio += 0.05;
		//Td = (int)(Ts*Delay_Ratio);

		
		Ls = pSharedMemory->Global_Ls;
		Lss = pSharedMemory->Global_Lss;
		Rot_Ang = pSharedMemory->Global_Rot_Ang;
		Delay_Ratio = pSharedMemory->Global_Delay_Ratio;
		Td_BP = (int)(Ts_BP*Delay_Ratio);
		Hs = pSharedMemory->Global_Hs;
		if(adj_amp == 0) Ab = pSharedMemory->Global_Ab;

		if(RMS_Y_ZMP != 0 && Lss == 0 && Rot_Ang == 0) Ab += adj_amp;

		if(pSharedMemory->First_flag == 1){
			Old_Lss = 0.; Old_Ls = 0.; Old_Rot_Ang = 0.;
		}
		


		if(Delay_Ratio < 0) {Delay_Ratio = 0. ; pSharedMemory->Global_Delay_Ratio = 0.;}


      
		  if(Old_Ls*Ls < 0) {  // ?? ?? ? Ls = 0 ? ?? ? ?? ??
			Ls = 0;
		  }

		  if(Old_Lss*Lss < 0) {  // ?? ?? ? Lss = 0 ? ?? ? ?? ??
			Lss = 0;
		  }
		  
		  if(Old_Rot_Ang*Rot_Ang < 0) {  // ?? ?? ? Rot_Ang = 0 ? ?? ? ?? ??
			Rot_Ang = 0;
		  }
		  
		  if(Rot_Ang*Lss < 0){ // ????? ??? ????? ??? ??? ??? ?. 
			  if(Lss != Old_Lss) Rot_Ang = 0.;
			  else if(Rot_Ang != Old_Rot_Ang) Lss = 0.;          
		  }

		  Old_Lss = Lss;
		  Old_Ls = Ls;	
		  Old_Rot_Ang = Rot_Ang;

		  pSharedMemory->Change_OK = 1;
		  pSharedMemory->First_flag =  0;
        
    }
        
    // 보행 패턴수행//
	Biped_walking_pattern(pSharedMemory->Go_flag,Ts_BP,Td_BP,shape_factor_F_BP,shape_factor_S_BP,Ab,Hs,Ls,Lss,Rot_Ang,Early_Landing_Flag,&Step_CNT,Ref_BC,Ref_R1,Ref_R2,Ref_R3,Ref_L1,Ref_L2,Ref_L3,Ref_th,Ref_ZMP,Walking_stage,&Change_flag,test_var);


    // 착지 Flag 들의 초기화 //
	if(Walking_stage[RF_1] == 10) {
		ThreeKG_ROK_Flag = 0;
		OneKG_ROK_Flag = 0;
		Early_Landing_Flag[RF_1] = 0;
		LandROK_Flag = 0;
	}

	if(Walking_stage[LF_1] == 10) {
		ThreeKG_LOK_Flag = 0;
		OneKG_LOK_Flag = 0;
		Early_Landing_Flag[LF_1] = 0;
		LandLOK_Flag = 0;
	}

	// ZMP 읽기 //
	readZMP_biped(_pRF0_3x1[1] + 0.001*Ref_R1[0], _pRF0_3x1[2] + 0.001*Ref_R1[1], 
		          _pLF0_3x1[1] + 0.001*Ref_L1[0], _pLF0_3x1[2] + 0.001*Ref_L1[1]);

	// 발 가속도 센서 처리 (발바닥 측방향 가속도 고주파 통과 필터 + 적분기)
	RF_Y_acc_HPF_INT = (1. - 2.*PI*F_cutoff_acc_HPF*Del_T)*Old_RF_Y_acc_HPF_INT + Del_T*(FTSensor[RFFT].dAccRoll+0.0);
	Old_RF_Y_acc_HPF_INT = RF_Y_acc_HPF_INT;

	LF_Y_acc_HPF_INT = (1. - 2.*PI*F_cutoff_acc_HPF*Del_T)*Old_LF_Y_acc_HPF_INT + Del_T*(FTSensor[LFFT].dAccRoll+0.0);
	Old_LF_Y_acc_HPF_INT = LF_Y_acc_HPF_INT;

	RF_X_acc_HPF_INT = (1. - 2.*PI*F_cutoff_acc_HPF*Del_T)*Old_RF_X_acc_HPF_INT + Del_T*(FTSensor[RFFT].dAccPitch+0.0);
	Old_RF_X_acc_HPF_INT = RF_X_acc_HPF_INT;

	LF_X_acc_HPF_INT = (1. - 2.*PI*F_cutoff_acc_HPF*Del_T)*Old_LF_X_acc_HPF_INT + Del_T*(FTSensor[LFFT].dAccPitch+0.0);
	Old_LF_X_acc_HPF_INT = LF_X_acc_HPF_INT;
	

	// DSP ZMP 제어 //
	if(Walking_stage[RF_1] == 0 && Walking_stage[LF_1] == 0){ 
		final_gain_DSP_ZMP_CON = 1.0;//0.5*(1-cos(PI*CNT_final_gain_DSP_ZMP_CON/20));

		Del_PC_X_DSP_XZMP_CON = final_gain_DSP_ZMP_CON*P2_XZMP_CON_E(-Del_PC_X_DSP_XZMP_CON,X_ZMP - X_ZMP_OFFSET_BP - Ref_ZMP[0], 1);
		Del_PC_Y_DSP_YZMP_CON = final_gain_DSP_ZMP_CON*P2_YZMP_CON_E(-Del_PC_Y_DSP_YZMP_CON,Y_ZMP - Ref_ZMP[1], 1);
	
		if(CNT_final_gain_DSP_ZMP_CON < 20) CNT_final_gain_DSP_ZMP_CON++;

		Old_Del_PC_X_DSP_XZMP_CON = Del_PC_X_DSP_XZMP_CON;
		Old_Del_PC_Y_DSP_YZMP_CON = Del_PC_Y_DSP_YZMP_CON;
		CNT_DSP_XYZMP_CON = 0;
	}
	else if((Walking_stage[RF_1] == 10 && Walking_stage[LF_1] == 0) || (Walking_stage[RF_1] == 0 && Walking_stage[LF_1] == 10)){
		Del_PC_X_DSP_XZMP_CON = Old_Del_PC_X_DSP_XZMP_CON - Old_Del_PC_X_DSP_XZMP_CON*0.5*(1-cos(PI*CNT_DSP_XYZMP_CON/40));
		Del_PC_Y_DSP_YZMP_CON = Old_Del_PC_Y_DSP_YZMP_CON - Old_Del_PC_Y_DSP_YZMP_CON*0.5*(1-cos(PI*CNT_DSP_XYZMP_CON/40));
		
		if(CNT_DSP_XYZMP_CON < 40) CNT_DSP_XYZMP_CON++;
		else{
			Old_Del_PC_X_DSP_XZMP_CON = 0.; Old_Del_PC_Y_DSP_YZMP_CON = 0.;
			P2_XZMP_CON_E(0,0,0); P2_YZMP_CON_E(0,0,0);
		}

		CNT_final_gain_DSP_ZMP_CON = 0;

	}
	else if(Walking_stage[RF_1] == 20  && Walking_stage[LF_1] == 0 && OneKG_ROK_Flag == 1){

		final_gain_DSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_DSP_ZMP_CON/20));
			
		Del_PC_X_DSP_XZMP_CON = final_gain_DSP_ZMP_CON*P2_XZMP_CON_E(-Del_PC_X_DSP_XZMP_CON,X_ZMP - X_ZMP_OFFSET_BP - Ref_ZMP[0], 1);
		Del_PC_Y_DSP_YZMP_CON = final_gain_DSP_ZMP_CON*P2_YZMP_CON_E(-Del_PC_Y_DSP_YZMP_CON,Y_ZMP - Ref_ZMP[1], 1);

		if(CNT_final_gain_DSP_ZMP_CON < 20) CNT_final_gain_DSP_ZMP_CON++;

		Old_Del_PC_X_DSP_XZMP_CON = Del_PC_X_DSP_XZMP_CON;
		Old_Del_PC_Y_DSP_YZMP_CON = Del_PC_Y_DSP_YZMP_CON;
		CNT_DSP_XYZMP_CON = 0;

	}
	else if(Walking_stage[RF_1] == 0  && Walking_stage[LF_1] == 20 && OneKG_LOK_Flag == 1){

		final_gain_DSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_DSP_ZMP_CON/20));
			
		Del_PC_X_DSP_XZMP_CON = final_gain_DSP_ZMP_CON*P2_XZMP_CON_E(-Del_PC_X_DSP_XZMP_CON,X_ZMP - X_ZMP_OFFSET_BP - Ref_ZMP[0], 1);
		Del_PC_Y_DSP_YZMP_CON = final_gain_DSP_ZMP_CON*P2_YZMP_CON_E(-Del_PC_Y_DSP_YZMP_CON,Y_ZMP - Ref_ZMP[1], 1);

		if(CNT_final_gain_DSP_ZMP_CON < 20) CNT_final_gain_DSP_ZMP_CON++;

		Old_Del_PC_X_DSP_XZMP_CON = Del_PC_X_DSP_XZMP_CON;
		Old_Del_PC_Y_DSP_YZMP_CON = Del_PC_Y_DSP_YZMP_CON;
		CNT_DSP_XYZMP_CON = 0;

	}


	// SSP ZMP 제어 //
	if((Walking_stage[RF_1] == 10  && Walking_stage[LF_1] == 0) || Walking_stage[RF_1] == 0  && Walking_stage[LF_1] == 10) {

		final_gain_SSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_SSP_ZMP_CON/30));

		P1_Y_th_th_d_Observer(-Del_PC_Y_SSP_YZMP_CON, Y_ZMP - BC_Y_OFFSET, 1);
				
		Del_PC_X_SSP_XZMP_CON = Old_Del_PC_X_SSP_XZMP_CON + final_gain_SSP_ZMP_CON*(-Old_Del_PC_X_SSP_XZMP_CON + 1.0*P1_XZMP_CON_E(-Del_PC_X_SSP_XZMP_CON, X_ZMP - X_ZMP_OFFSET_BP - Ref_ZMP[0], 1));
		Del_PC_Y_SSP_YZMP_CON = Old_Del_PC_Y_SSP_YZMP_CON + final_gain_SSP_ZMP_CON*(-Old_Del_PC_Y_SSP_YZMP_CON + 1.0*P1_YZMP_CON_E(-Del_PC_Y_SSP_YZMP_CON, Y_ZMP - Ref_ZMP[1], 1));

		
		if(CNT_final_gain_SSP_ZMP_CON < 30) CNT_final_gain_SSP_ZMP_CON++;
		else{
			Old_Del_PC_X_SSP_XZMP_CON = 0.; Old_Del_PC_Y_SSP_YZMP_CON = 0.;
		}
				
		CNT_SSP_ZMP_CON = 0;
		
		/*
		Temp_CNT = 0;

		if(Walking_stage[LF_1] == 10){
			RF_Del_Y_Outside = Old_RF_Del_Y_Outside - Old_RF_Del_Y_Outside*0.5*(1-cos(PI*Temp_CNT_2/(Ts_BP/3)));

			if(Temp_CNT_2 < Ts_BP/3) Temp_CNT_2++;
			else {Old_RF_Del_Y_Outside = 0.; RF_Del_Y_Outside = 0.;}
		}

		Outside_Land_Flag = 0;
		*/
		

	}
	else if(Walking_stage[RF_1] == 20  && Walking_stage[LF_1] == 0 && OneKG_ROK_Flag == 0) {

		P1_Y_th_th_d_Observer(-Del_PC_Y_SSP_YZMP_CON, Y_ZMP - BC_Y_OFFSET, 1);
			
		/*
		if(Temp_CNT == (Ts_BP/10) && Body_Y_th < 0) Outside_Land_Flag = 1;

		if(Outside_Land_Flag) {

			RF_Del_Y_Outside = 2.0*0.5*(1-cos(PI*(Temp_CNT - Ts_BP/10)/(Ts_BP/3 - Ts_BP/10)));
			Old_RF_Del_Y_Outside = RF_Del_Y_Outside;

		}

		if(Temp_CNT < Ts_BP/3) Temp_CNT++;

		Temp_CNT_2 = 0;
		*/		


		Del_PC_X_SSP_XZMP_CON = 1.0*P1_XZMP_CON_E(-Del_PC_X_SSP_XZMP_CON, X_ZMP - X_ZMP_OFFSET_BP - Ref_ZMP[0], 1);
		Del_PC_Y_SSP_YZMP_CON = 1.0*P1_YZMP_CON_E(-Del_PC_Y_SSP_YZMP_CON, Y_ZMP - Ref_ZMP[1], 1);
		
		Old_Del_PC_X_SSP_XZMP_CON_2 = Del_PC_X_SSP_XZMP_CON;
		Old_Del_PC_Y_SSP_YZMP_CON_2 = Del_PC_Y_SSP_YZMP_CON;

		CNT_final_gain_SSP_ZMP_CON = 0;

	}
	else if(Walking_stage[RF_1] == 0  && Walking_stage[LF_1] == 20 && OneKG_LOK_Flag == 0) {

		P1_Y_th_th_d_Observer(-Del_PC_Y_SSP_YZMP_CON, Y_ZMP - BC_Y_OFFSET, 1);

		Del_PC_X_SSP_XZMP_CON = 1.0*P1_XZMP_CON_E(-Del_PC_X_SSP_XZMP_CON, X_ZMP - X_ZMP_OFFSET_BP - Ref_ZMP[0], 1);
		Del_PC_Y_SSP_YZMP_CON = 1.0*P1_YZMP_CON_E(-Del_PC_Y_SSP_YZMP_CON, Y_ZMP - Ref_ZMP[1], 1);

		Old_Del_PC_X_SSP_XZMP_CON_2 = Del_PC_X_SSP_XZMP_CON;
		Old_Del_PC_Y_SSP_YZMP_CON_2 = Del_PC_Y_SSP_YZMP_CON;

		CNT_final_gain_SSP_ZMP_CON = 0;

	}
	else if(Walking_stage[RF_1] == 0 && Walking_stage[LF_1] == 0 && Real_Stop_Flag == 0){


		Del_PC_X_SSP_XZMP_CON = Old_Del_PC_X_SSP_XZMP_CON_2 - Old_Del_PC_X_SSP_XZMP_CON_2*0.5*(1-cos(PI*CNT_SSP_ZMP_CON/100));
		Del_PC_Y_SSP_YZMP_CON = Old_Del_PC_Y_SSP_YZMP_CON_2 - Old_Del_PC_Y_SSP_YZMP_CON_2*0.5*(1-cos(PI*CNT_SSP_ZMP_CON/100));

		Old_Del_PC_X_SSP_XZMP_CON = Del_PC_X_SSP_XZMP_CON;
		Old_Del_PC_Y_SSP_YZMP_CON = Del_PC_Y_SSP_YZMP_CON;

		if(CNT_SSP_ZMP_CON < 100) CNT_SSP_ZMP_CON++;
		else {
				Del_PC_Y_SSP_YZMP_CON = 0.0;
				Old_Del_PC_Y_SSP_YZMP_CON = 0.0;
				Del_PC_X_SSP_XZMP_CON = 0.0;
				Old_Del_PC_X_SSP_XZMP_CON = 0.0;
				CNT_SSP_ZMP_CON = 0;
				P1_XZMP_CON_E(0,0,0);
				P1_YZMP_CON_E(0,0,0);
				P1_Y_th_th_d_Observer(0,0,0);
				CNT_final_gain_SSP_ZMP_CON = 0;
				Old_Del_PC_X_SSP_XZMP_CON_2 = 0.; Old_Del_PC_Y_SSP_YZMP_CON_2 = 0.;
		}

	}
	


	// ZMP 적분제어 : 한 보행 주기(2스텝)동안 X, Y ZMP 오차를 누적한 양이 작아지도록 몸통의 중심위치를 점진적으로 수평 이동 // 
	if(Step_CNT >= 2 && Real_Stop_Flag == 0){   // 첫번째 스텝은 비대칭이라 무시. 두 번째 스텝부터 반영함.
		if(Step_CNT >= 4 && Old_Step_CNT%2 == 1 && Step_CNT%2 == 0 ){
			
			Old_CON_SUM_X_ZMP_ERR = CON_SUM_X_ZMP_ERR;
			Old_CON_SUM_Y_ZMP_ERR = CON_SUM_Y_ZMP_ERR;

			CON_SUM_X_ZMP_ERR += 0.2*(SUM_X_ZMP_ERR/CNT_SUM_ZMP_ERR); // 0.1 --> 적분 게인
			CON_SUM_Y_ZMP_ERR += 0.2*(SUM_Y_ZMP_ERR/CNT_SUM_ZMP_ERR); 

			if(CON_SUM_X_ZMP_ERR > 10.0) CON_SUM_X_ZMP_ERR = 10.0;   // 최종 제어 입력 Range 설정
			else if(CON_SUM_X_ZMP_ERR < -10.0) CON_SUM_X_ZMP_ERR = -10.0;
				 
			if(CON_SUM_Y_ZMP_ERR > 10.0) CON_SUM_Y_ZMP_ERR = 10.0;   // 최종 제어 입력 Range 설정
			else if(CON_SUM_Y_ZMP_ERR < -10.0) CON_SUM_Y_ZMP_ERR = -10.0;

			RMS_Y_ZMP = sqrt(SUM_RMS_Y_ZMP/CNT_SUM_ZMP_ERR);

			pSharedMemory->WB_fRS[0] = RMS_Y_ZMP;

			if(RMS_Y_ZMP != 0 )	adj_amp = 0.*0.05*(DES_RMS_Y_ZMP - RMS_Y_ZMP);

			SUM_X_ZMP_ERR = 0.;
			SUM_Y_ZMP_ERR = 0.;
			SUM_RMS_Y_ZMP = 0.;
			CNT_SUM_ZMP_ERR = 0;
			CNT_CON_SUM_XY_ZMP_ERR = 0;
		}
		else {
				SUM_Y_ZMP_ERR += Y_ZMP - Ref_ZMP[1];
				SUM_X_ZMP_ERR += X_ZMP - X_ZMP_OFFSET_BP - Ref_ZMP[0] + 8.0*Ls/100;
								

				/* RMS ZMP SUM 계산 */
				SUM_RMS_Y_ZMP += (Y_ZMP - BC_Y_OFFSET)*(Y_ZMP - BC_Y_OFFSET);

				CNT_SUM_ZMP_ERR++;

				if(SUM_X_ZMP_ERR > 3200.0) SUM_X_ZMP_ERR = 3200.0;   // 추가 적분 제어 입력량 Limit 설정 
				else if(SUM_X_ZMP_ERR < -3200.0) SUM_X_ZMP_ERR = -3200.0;
				
				if(SUM_Y_ZMP_ERR > 3200.0) SUM_Y_ZMP_ERR = 3200.0;   // 추가 적분 제어 입력량 Limit 설정
				else if(SUM_Y_ZMP_ERR < -3200.0) SUM_Y_ZMP_ERR = -3200.0;

		}
	}
	else{

		SUM_X_ZMP_ERR = 0.;
		SUM_Y_ZMP_ERR = 0.;
		SUM_RMS_Y_ZMP = 0.;
		CNT_SUM_ZMP_ERR = 0;
		RMS_Y_ZMP = 0.;
	}

	Ref_CON_SUM_X_ZMP_ERR = Old_CON_SUM_X_ZMP_ERR + (CON_SUM_X_ZMP_ERR - Old_CON_SUM_X_ZMP_ERR)*0.5*(1-cos(PI*CNT_CON_SUM_XY_ZMP_ERR/10));
	Ref_CON_SUM_Y_ZMP_ERR = Old_CON_SUM_Y_ZMP_ERR + (CON_SUM_Y_ZMP_ERR - Old_CON_SUM_Y_ZMP_ERR)*0.5*(1-cos(PI*CNT_CON_SUM_XY_ZMP_ERR/10));

	if(CNT_CON_SUM_XY_ZMP_ERR < 10) CNT_CON_SUM_XY_ZMP_ERR++;		

	pSharedMemory->WB_fRS[1] = Ref_CON_SUM_X_ZMP_ERR;
	pSharedMemory->WB_fRS[2] = Ref_CON_SUM_Y_ZMP_ERR;


	// RLAR & RLAP 버츄얼 스프링 댐퍼 제어 //
	if(Walking_stage[LF_1] != 10 && OneKG_LOK_Flag == 1 && LandLOK_Flag == 0) {
 			Del_LAR_RLAR_TORQ_CON = 1.0*RLAR_TORQ_CON(FTSensor[LFFT].Mx,1);  
			Del_LAP_RLAP_TORQ_CON = -1.0*RLAP_TORQ_CON(-FTSensor[LFFT].My,1);
 			CNT_RLAR_TORQ_CON = 0.;
 			Old_Del_LAR_RLAR_TORQ_CON = Del_LAR_RLAR_TORQ_CON;
			Old_Del_LAP_RLAP_TORQ_CON = Del_LAP_RLAP_TORQ_CON;

 	}
	else if(Walking_stage[LF_1] == 10){
			Del_LAR_RLAR_TORQ_CON = Old_Del_LAR_RLAR_TORQ_CON - Old_Del_LAR_RLAR_TORQ_CON*0.5*(1-cos(PI*CNT_RLAR_TORQ_CON/40));
			Del_LAP_RLAP_TORQ_CON = Old_Del_LAP_RLAP_TORQ_CON - Old_Del_LAP_RLAP_TORQ_CON*0.5*(1-cos(PI*CNT_RLAR_TORQ_CON/40));
 			
 			if (CNT_RLAR_TORQ_CON < 40 /*&& Old_Del_LAR_RLAR_TORQ_CON != 0.0*/) CNT_RLAR_TORQ_CON++;
 			else {Old_Del_LAR_RLAR_TORQ_CON = 0.; Del_LAR_RLAR_TORQ_CON = 0.; /*CNT_RLAR_TORQ_CON = 0;*/ RLAR_TORQ_CON(0,0);
				  Old_Del_LAP_RLAP_TORQ_CON = 0.; Del_LAP_RLAP_TORQ_CON = 0.; RLAP_TORQ_CON(0,0);}
	}

	if(Walking_stage[RF_1] != 10 && OneKG_ROK_Flag == 1 && LandROK_Flag == 0) {
			Del_RAR_RLAR_TORQ_CON = 1.0*RLAR_TORQ_CON(FTSensor[RFFT].Mx,1);
			Del_RAP_RLAP_TORQ_CON = -1.0*RLAP_TORQ_CON(-FTSensor[RFFT].My,1);
 			CNT_RLAR_TORQ_CON = 0.;
 			Old_Del_RAR_RLAR_TORQ_CON = Del_RAR_RLAR_TORQ_CON;
			Old_Del_RAP_RLAP_TORQ_CON = Del_RAP_RLAP_TORQ_CON;
	}
	else if(Walking_stage[RF_1] == 10){
			Del_RAR_RLAR_TORQ_CON = Old_Del_RAR_RLAR_TORQ_CON - Old_Del_RAR_RLAR_TORQ_CON*0.5*(1-cos(PI*CNT_RLAR_TORQ_CON/40));
			Del_RAP_RLAP_TORQ_CON = Old_Del_RAP_RLAP_TORQ_CON - Old_Del_RAP_RLAP_TORQ_CON*0.5*(1-cos(PI*CNT_RLAR_TORQ_CON/40));
 
 			if (CNT_RLAR_TORQ_CON < 40 /*&& Old_Del_RAR_RLAR_TORQ_CON != 0.0*/) CNT_RLAR_TORQ_CON++;
 			else {Old_Del_RAR_RLAR_TORQ_CON = 0.; Del_RAR_RLAR_TORQ_CON = 0.; /*CNT_RLAR_TORQ_CON = 0;*/ RLAR_TORQ_CON(0,0);
				  Old_Del_RAP_RLAP_TORQ_CON = 0.; Del_RAP_RLAP_TORQ_CON = 0.; RLAP_TORQ_CON(0,0);}

	}

	
	// Swing foot 측방향/전진방향 진동 제어, 몸통 Z 방향 진동 제어 // 
	if(Walking_stage[RF_1] == 10){

			final_gain_SSP_Y_VIB_CON_2 = 0.5*(1-cos(PI*CNT_final_gain_SSP_Y_VIB_CON_2/10));

			Del_RHR_VIB_CON = -1.0*final_gain_SSP_Y_VIB_CON_2*R2D*SSP_Y_VIB_CON_E_RF(D2R*Del_RHR_VIB_CON,0.006*RF_Y_acc_HPF_INT, 1); // [deg]
			Del_RHP_VIB_CON = -1.0*final_gain_SSP_Y_VIB_CON_2*R2D*SSP_X_VIB_CON_E_RF(D2R*Del_RHP_VIB_CON,0.006*RF_X_acc_HPF_INT, 1); // [deg]
			if(Rot_Ang == 0) Del_LHY_VIB_CON =  1.0*final_gain_SSP_Y_VIB_CON_2*R2D*SSP_Z_VIB_CON_E(-D2R*Del_LHY_VIB_CON, D2R*IMUSensor[CENTERIMU].Yaw_Velocity, 1); // [Deg]
			else Del_LHY_VIB_CON = 0.;

			if(CNT_final_gain_SSP_Y_VIB_CON_2 < 10) CNT_final_gain_SSP_Y_VIB_CON_2++;

			Old_Del_RHR_VIB_CON = Del_RHR_VIB_CON; Old_Del_RHP_VIB_CON = Del_RHP_VIB_CON;
			CNT_FOOT_VIB_CON = 0;

	}
	else if(Walking_stage[RF_1] == 20){

			final_gain_SSP_Y_VIB_CON_4 = 1.0 - 0.5*(1-cos(PI*CNT_FOOT_VIB_CON/30));

			Del_RHR_VIB_CON = -1.0*final_gain_SSP_Y_VIB_CON_4*R2D*SSP_Y_VIB_CON_E_RF(D2R*Del_RHR_VIB_CON,0.006*RF_Y_acc_HPF_INT, 1); // [deg]
			Del_RHP_VIB_CON = -1.0*final_gain_SSP_Y_VIB_CON_4*R2D*SSP_X_VIB_CON_E_RF(D2R*Del_RHP_VIB_CON,0.006*RF_X_acc_HPF_INT, 1); // [deg]
			if(Rot_Ang == 0) Del_LHY_VIB_CON =  1.0*final_gain_SSP_Y_VIB_CON_4*R2D*SSP_Z_VIB_CON_E(-D2R*Del_LHY_VIB_CON, D2R*IMUSensor[CENTERIMU].Yaw_Velocity, 1); // [Deg]
			else  Del_LHY_VIB_CON = 0.;


			if(CNT_FOOT_VIB_CON < 30) CNT_FOOT_VIB_CON++;
			else {Old_Del_RHR_VIB_CON = 0.0; SSP_Y_VIB_CON_E_RF(0,0,0); Old_Del_RHP_VIB_CON = 0.0; SSP_X_VIB_CON_E_RF(0,0,0); 
			      CNT_final_gain_SSP_Y_VIB_CON_1 = 0; final_gain_SSP_Y_VIB_CON_1 = 0.; SSP_Z_VIB_CON_E(0,0,0);}

	}

	if(Walking_stage[LF_1] == 10){

			final_gain_SSP_Y_VIB_CON_1 = 0.5*(1-cos(PI*CNT_final_gain_SSP_Y_VIB_CON_1/10));

			Del_LHR_VIB_CON = -1.0*final_gain_SSP_Y_VIB_CON_1*R2D*SSP_Y_VIB_CON_E_LF(D2R*Del_LHR_VIB_CON,0.006*LF_Y_acc_HPF_INT, 1);
			Del_LHP_VIB_CON = -1.0*final_gain_SSP_Y_VIB_CON_1*R2D*SSP_X_VIB_CON_E_LF(D2R*Del_LHP_VIB_CON,0.006*LF_X_acc_HPF_INT, 1);
			if(Rot_Ang == 0) Del_RHY_VIB_CON =  1.0*final_gain_SSP_Y_VIB_CON_1*R2D*SSP_Z_VIB_CON_E(-D2R*Del_RHY_VIB_CON, D2R*IMUSensor[CENTERIMU].Yaw_Velocity, 1);
			else Del_RHY_VIB_CON = 0.;

			if(CNT_final_gain_SSP_Y_VIB_CON_1 < 10) CNT_final_gain_SSP_Y_VIB_CON_1++;

			Old_Del_LHR_VIB_CON = Del_LHR_VIB_CON; Old_Del_LHP_VIB_CON = Del_LHP_VIB_CON;
			CNT_FOOT_VIB_CON = 0;

	}
	else if(Walking_stage[LF_1] == 20){
			final_gain_SSP_Y_VIB_CON_3 = 1.0 - 0.5*(1-cos(PI*CNT_FOOT_VIB_CON/30));

			Del_LHR_VIB_CON = -1.0*final_gain_SSP_Y_VIB_CON_3*R2D*SSP_Y_VIB_CON_E_LF(D2R*Del_LHR_VIB_CON,0.006*LF_Y_acc_HPF_INT, 1);
			Del_LHP_VIB_CON = -1.0*final_gain_SSP_Y_VIB_CON_3*R2D*SSP_X_VIB_CON_E_LF(D2R*Del_LHP_VIB_CON,0.006*LF_X_acc_HPF_INT, 1);
			if(Rot_Ang == 0) Del_RHY_VIB_CON =  1.0*final_gain_SSP_Y_VIB_CON_3*R2D*SSP_Z_VIB_CON_E(-D2R*Del_RHY_VIB_CON, D2R*IMUSensor[CENTERIMU].Yaw_Velocity, 1);
			else  Del_RHY_VIB_CON = 0.;


			if(CNT_FOOT_VIB_CON < 30) CNT_FOOT_VIB_CON++;
			else {Old_Del_LHR_VIB_CON = 0.0; SSP_Y_VIB_CON_E_LF(0,0,0); Old_Del_LHP_VIB_CON = 0.0; SSP_X_VIB_CON_E_LF(0,0,0);
			      CNT_final_gain_SSP_Y_VIB_CON_2 = 0; final_gain_SSP_Y_VIB_CON_2 = 0.; SSP_Z_VIB_CON_E(0,0,0);}

	}


	// 착지 시 지면 기울기 측정 //
	if(Walking_stage[RF_1] == 0 && Walking_stage[LF_1] == 10){
	
		Avg_RF_Pitch_Ang = (Avg_CNT - 1.)/Avg_CNT*Old_Avg_RF_Pitch_Ang + 1./Avg_CNT*FTSensor[RFFT].dAccPitch;  

		Old_Avg_RF_Pitch_Ang = Avg_RF_Pitch_Ang;

		Avg_CNT++;

	}
	else if(Walking_stage[RF_1] == 10 && Walking_stage[LF_1] == 0){
	

		Avg_LF_Pitch_Ang = (Avg_CNT - 1.)/Avg_CNT*Old_Avg_LF_Pitch_Ang + 1./Avg_CNT*FTSensor[LFFT].dAccPitch;

		Old_Avg_LF_Pitch_Ang = Avg_LF_Pitch_Ang;

		Avg_CNT++;

	}
	else {
		Avg_CNT = 1; Old_Avg_RF_Pitch_Ang = 0.; Avg_RF_Pitch_Ang = 0.;
		Avg_CNT = 1; Old_Avg_LF_Pitch_Ang = 0.; Avg_LF_Pitch_Ang = 0.;
	}


	

	
	// 보행 시 몸통 수직 자세 제어 : 보행 시 가속도값 적분제어 (엉덩이 피치/발목 롤 수정) //
	Del_RLHP_Posture_Walking = -TP_INT_CON_WALK(0.1,1.0*IMUSensor[CENTERIMU].Pitch,1);   // 가속도계 + ZMP 에 의한 경사만으로 상체 피치 기울기 교정 //
	Del_RLFZ_Posture_Walking =  1.*TR_INT_CON_WALK(-0.6,1.0*IMUSensor[CENTERIMU].Roll,1);    // 가속도계 + ZMP 에 의한 경사만으로 상체 롤 기울기 교정 //
	Del_RLAR_Posture_Walking =  -atan(Del_RLFZ_Posture_Walking/(1000.*_pLF0_3x1[2])); // 롤 경사에 의한 발목 롤 교정 //

   
	// Early Landing Algorithm //
	for(i = 0; i<6; i++){
		if(Walking_stage[i] == 20 && Fz[i] > EARLY_LAND_THRES_1) {		
			Early_Landing_Flag[i] = 1; 			
			//if(i == LF1) First_Flag_1 = 0;
		}
    }
        
        
    if(Old_Walking_stage[RF_1] == 20 && Walking_stage[RF_1] == 0) {
			
		temp_Z_avg = temp_Z[RF_1];

		All_Foot_Z_Flag = 1;
	}
    else if(Old_Walking_stage[LF_1] == 20 && Walking_stage[LF_1] == 0) {
			
		temp_Z_avg = temp_Z[LF_1];

		All_Foot_Z_Flag = 1;
    }

    if(All_Foot_Z_Flag ){
		All_Foot_Z = Old_All_Foot_Z + (temp_Z_avg - Old_All_Foot_Z)*0.5*(1-cos(PI*All_Foot_Z_CNT/(Ts_BP/2)));

		if(All_Foot_Z_CNT < Ts_BP/2) All_Foot_Z_CNT++;
		else{
			Old_All_Foot_Z = All_Foot_Z;
			All_Foot_Z_Flag = 0;
			All_Foot_Z_CNT = 0;
		}
    }


	if(Early_Landing_Flag[0] == 1 && temp_X[0] == 0. && temp_Y[0] == 0. && temp_th[0] == 0.) {temp_X[0] = Ref_R1[0]; temp_Y[0] = Ref_R1[1]; temp_th[0] = Ref_th[0]; EL_Del_xy_CNT[0] = 0;}
	else if(Early_Landing_Flag[0] == 1 && (temp_X[0] != 0. || temp_Y[0] != 0. || temp_th[0] != 0.)) {
		   Ref_R1[0] = temp_X[0]; Ref_R1[1] = temp_Y[0]; Ref_th[0] = temp_th[0];
		   Old_Ref_R1[0] = Ref_R1[0]; Old_Ref_R1[1] = Ref_R1[1]; Old_Ref_th[0] = Ref_th[0];
	}
	else if(Early_Landing_Flag[0] == 0 && (temp_X[0] != 0. || temp_Y[0] != 0. || temp_th[0] != 0.)){

	if(temp_X[0] != 0.) Ref_R1[0] = Old_Ref_R1[0] + (Ref_R1[0] - Old_Ref_R1[0])*0.5*(1-cos(PI/60*EL_Del_xy_CNT[0]));
	if(temp_Y[0] != 0.) Ref_R1[1] = Old_Ref_R1[1] + (Ref_R1[1] - Old_Ref_R1[1])*0.5*(1-cos(PI/60*EL_Del_xy_CNT[0]));
	if(temp_th[0] != 0.) Ref_th[0] = Old_Ref_th[0] + (Ref_th[0] - Old_Ref_th[0])*0.5*(1-cos(PI/60*EL_Del_xy_CNT[0]));

	if(EL_Del_xy_CNT[0] < 60 && WP_Flow_Control_Flag == 0) EL_Del_xy_CNT[0]++;
	else if(WP_Flow_Control_Flag == 0) {Old_Ref_R1[0] = 0.;temp_X[0] = 0.; Old_Ref_R1[1] = 0.;temp_Y[0] = 0.; Old_Ref_th[0] = 0.; temp_th[0] = 0.;}
	}
	
   if(Early_Landing_Flag[3] == 1 && temp_X[3] == 0. && temp_Y[3] == 0. && temp_th[3] == 0.) {temp_X[3] = Ref_L1[0]; temp_Y[3] = Ref_L1[1]; temp_th[3] = Ref_th[3]; EL_Del_xy_CNT[3] = 0;}
   else if(Early_Landing_Flag[3] == 1 && (temp_X[3] != 0. || temp_Y[3] != 0. || temp_th[3] != 0.)) {
	Ref_L1[0] = temp_X[3]; Ref_L1[1] = temp_Y[3]; Ref_th[3] = temp_th[3];
	Old_Ref_L1[0] = Ref_L1[0]; Old_Ref_L1[1] = Ref_L1[1]; Old_Ref_th[3] = Ref_th[3];
   }
   else if(Early_Landing_Flag[3] == 0 && (temp_X[3] != 0. || temp_Y[3] != 0. || temp_th[3] != 0.)){
		   if(temp_X[3] != 0.) Ref_L1[0] = Old_Ref_L1[0] + (Ref_L1[0] - Old_Ref_L1[0])*0.5*(1-cos(PI/60*EL_Del_xy_CNT[3]));
	if(temp_Y[3] != 0.) Ref_L1[1] = Old_Ref_L1[1] + (Ref_L1[1] - Old_Ref_L1[1])*0.5*(1-cos(PI/60*EL_Del_xy_CNT[3]));
	if(temp_th[3] != 0.) Ref_th[3] = Old_Ref_th[3] + (Ref_th[3] - Old_Ref_th[3])*0.5*(1-cos(PI/60*EL_Del_xy_CNT[3]));

	if(EL_Del_xy_CNT[3] < 60 && WP_Flow_Control_Flag == 0) EL_Del_xy_CNT[3]++;
	else if(WP_Flow_Control_Flag == 0) {Old_Ref_L1[0] = 0.;temp_X[3] = 0.; Old_Ref_L1[1] = 0.;temp_Y[3] = 0.; Old_Ref_th[3] = 0.; temp_th[3] = 0.;}
   }




	// Late Landing Algorithm //
    for( i = 0; i<6; i++){
        if(Old_Walking_stage[i] == 20 && Walking_stage[i] == 0 && Early_Landing_Flag[i] == 0 && LL_Del_z[i] == 0. ) {

			Late_Landing_Flag[i] = 1;

		}		

        if(Late_Landing_Flag[i] == 1 && Fz[i] < EARLY_LAND_THRES_1 /*&& LL_Del_z[i] < LATE_LAND_MAX_DEPTH*/ && Walking_stage[i] == 0){
				
			LL_Del_z[i] = 0.*Late_Land_Vertical_Disp(i,(EARLY_LAND_THRES_1+5.0), Fz[i], 1);	
			Old_LL_Del_z[i] = LL_Del_z[i];
			LL_Del_z_CNT[i] = 0;
			
        }
        else if(Late_Landing_Flag[i] == 1 && Fz[i] >= EARLY_LAND_THRES_1 && Walking_stage[i] == 0){

	           Late_Landing_Flag[i] = 2;

        }
        else if((Late_Landing_Flag[i] == 1 || Late_Landing_Flag[i] == 2 ) && Walking_stage[i] == 10){

			LL_Del_z[i] = Old_LL_Del_z[i] - Old_LL_Del_z[i]*0.5*(1-cos(PI*LL_Del_z_CNT[i]/(Ts_BP/2)));

			if(LL_Del_z_CNT[i] < (Ts_BP/2) && WP_Flow_Control_Flag == 0) LL_Del_z_CNT[i]++;
			else if(WP_Flow_Control_Flag == 0){
					Late_Landing_Flag[i] = 0;
					Late_Land_Vertical_Disp(i,0,0,0);
					Old_LL_Del_z[i] = 0.;
					LL_Del_z[i] = 0.;	
			}
        }
    }

      // ?? ??? ?? ?? ?? ?? ???? : ???? ??? ???? ??? ??? ? ??? ?? ?? ?.
      if(Late_Landing_Flag[RF_1] == 1 /*&& Old_LL_Del_z[RF_1] < LATE_LAND_MAX_DEPTH*/ ) WP_Flow_Control_Flag = 1;
      else if(Late_Landing_Flag[LF_1] == 1 /*&& Old_LL_Del_z[LF_1] < LATE_LAND_MAX_DEPTH*/ ) WP_Flow_Control_Flag = 1;
      else WP_Flow_Control_Flag = 0;



    //readZMP_biped(_pRF0_3x1[1] + 0.001*Ref_R1[0], _pRF0_3x1[2] + 0.001*Ref_R1[1], 
	//	          _pLF0_3x1[1] + 0.001*Ref_L1[0], _pLF0_3x1[2] + 0.001*Ref_L1[1]);

	//ZMP_CON_MANAGER_BP(FTSensor[RFFT].Fz, FTSensor[LFFT].Fz, Land_ROK_Flag, Land_LOK_Flag, 1);   // update

	//BC_Y_CON = -Del_YZMP_CON;  // update
	//BC_X_CON = -Del_XZMP_CON;  // update

	pSharedMemory->ZMP[0] = X_ZMP;
	pSharedMemory->ZMP[1] = Y_ZMP;

	for(i=0;i<6;i++) Old_Walking_stage[i] = Walking_stage[i];

	Old_Step_CNT = Step_CNT;

	if(0){ //no control

		BC_X_CON = 0.;
		BC_Y_CON = 0.;

		
		WP_Flow_Control_Flag = 0;

		for(i=0;i<6;i++) {
				Early_Landing_Flag[i] = 0;
				Late_Landing_Flag[i] = 0;
				temp_Z[i] = 0.;
				LL_Del_z[i] = 0.; Old_LL_Del_z[i] = 0.; LL_Del_z_CNT[i] = 0;
				Late_Land_Vertical_Disp(i,0,0,0);
		}

		Del_PC_X_SSP_XZMP_CON = 0.; Del_PC_Y_SSP_YZMP_CON = 0.;    
		Del_PC_X_DSP_XZMP_CON = 0.; Del_PC_Y_DSP_YZMP_CON = 0.;   
		Ref_CON_SUM_X_ZMP_ERR = 0.; Ref_CON_SUM_Y_ZMP_ERR = 0.; CNT_CON_SUM_XY_ZMP_ERR = 0;
		CON_SUM_X_ZMP_ERR = 0.; CON_SUM_Y_ZMP_ERR = 0.;

		Del_LAR_RLAR_TORQ_CON = 0.; Del_LAP_RLAP_TORQ_CON = 0.; Del_RAR_RLAR_TORQ_CON = 0.; Del_RAP_RLAP_TORQ_CON = 0.;

		Del_RHR_VIB_CON = 0.; Del_LHR_VIB_CON = 0., Del_RHP_VIB_CON = 0.; Del_LHP_VIB_CON = 0.; Del_RHY_VIB_CON = 0.; Del_LHY_VIB_CON = 0.;

		Del_RLHP_Posture_Walking = 0.; Del_RLFZ_Posture_Walking = 0.; Del_RLAR_Posture_Walking = 0.;

		adj_amp = 0.;

	}

	BC_Y_CON = - Del_PC_Y_DSP_YZMP_CON - 1.*Del_PC_Y_SSP_YZMP_CON - 1.*Ref_CON_SUM_Y_ZMP_ERR;  // update
	BC_X_CON = - Del_PC_X_DSP_XZMP_CON - 1.*Del_PC_X_SSP_XZMP_CON - 1.*Ref_CON_SUM_X_ZMP_ERR;;  // update

	
    // ?? ?? ??? ???? ??//
    Rot_th_R1 = Ref_th[0] - Ref_th[6]; // [deg]
    Rot_th_L1 = Ref_th[3] - Ref_th[6]; // [deg]
   
    del_x_R1 = -_pRF0_3x1[2]*sin(D2R*Rot_th_R1); 
    del_y_R1 = _pRF0_3x1[2]*cos(D2R*Rot_th_R1) - _pRF0_3x1[2]; 
    del_x_L1 = _pLF0_3x1[2]*sin(D2R*(180. + Rot_th_L1)); 
    del_y_L1 = -_pLF0_3x1[2]*cos(D2R*(180. + Rot_th_L1)) - _pLF0_3x1[2]; 
    
    	   
    _online_pattern.dangRFz = D2R*Rot_th_R1;
    _online_pattern.dangLFz = D2R*Rot_th_L1;
	_online_pattern.dangRHz = 0.f;
	_online_pattern.dangLHz = 0.f;

	_online_pattern.dpCOMx = 0.f;
	_online_pattern.dpCOMy = 0.f;
            
   
	_online_pattern.dpRFz = 0.001*Ref_R1[2] + 0.001*Del_RLFZ_Init;
    _online_pattern.dpLFz = 0.001*Ref_L1[2] - 0.001*Del_RLFZ_Init;
    _online_pattern.dpRFy = 0.001*Ref_R1[1] - 0.001*Ref_BC[1] + del_y_R1 - 0.001*BC_Y_Init - 0.001*BC_Y_CON + 0.0*RF_Del_Y_Outside;  // update
    _online_pattern.dpLFy = 0.001*Ref_L1[1] - 0.001*Ref_BC[1] + del_y_L1 - 0.001*BC_Y_Init - 0.001*BC_Y_CON; // update
    _online_pattern.dpRFx = 0.001*Ref_R1[0] - 0.001*Ref_BC[0] + del_x_R1 - 0.001*BC_X_Init - 0.001*BC_X_CON; // update
    _online_pattern.dpLFx = 0.001*Ref_L1[0] - 0.001*Ref_BC[0] + del_x_L1 - 0.001*BC_X_Init - 0.001*BC_X_CON; // update   

	_online_pattern.offset_LAR = (Del_LAR_Init + Del_LAR_RLAR_TORQ_CON + Del_RLAR_Posture_Walking + 0.0*RHR_Compen_Ang)*D2R;
	_online_pattern.offset_LAP = (Del_LAP_RLAP_TORQ_CON + Del_RLHP_Init)*D2R;

    _online_pattern.offset_RAR = (Del_RAR_Init + Del_RAR_RLAR_TORQ_CON + Del_RLAR_Posture_Walking + 0.0*LHR_Compen_Ang)*D2R;
	_online_pattern.offset_RAP = (Del_RAP_Init + Del_RAP_RLAP_TORQ_CON + Del_RLHP_Init)*D2R;

	_online_pattern.offset_RHR = (Del_RHR_VIB_CON + RHR_Compen_Ang)*D2R;
	_online_pattern.offset_LHR = (Del_LHR_VIB_CON + LHR_Compen_Ang + RF_Del_Y_Outside)*D2R;

	_online_pattern.offset_RHP = (Del_RHP_VIB_CON + Del_RLHP_Posture_Walking)*D2R;
	_online_pattern.offset_LHP = (Del_LHP_VIB_CON + Del_RLHP_Posture_Walking)*D2R;

	_online_pattern.offset_RHY = Del_RHY_VIB_CON*D2R;
	_online_pattern.offset_LHY = Del_LHY_VIB_CON*D2R;

	_log_temp[0] = 0.001*Ref_BC[0];      // A
	_log_temp[1] = 0.001*Ref_BC[1];      // B
	_log_temp[2] = 0.001*Ref_R1[2];      // C1
	_log_temp[3] = 0.001*Ref_L1[2];      // C2
	_log_temp[4] = 0.001*X_ZMP;          // C3
	_log_temp[5] = 0.001*Y_ZMP;          // C4
	_log_temp[6] = 0.001*Ref_ZMP[0];     // C5
	_log_temp[7] = 0.001*Ref_ZMP[1];     // C6
	_log_temp[8] = 0.001*Del_PC_X_DSP_XZMP_CON;         // C7
	_log_temp[9] = 0.001*Del_PC_Y_DSP_YZMP_CON;         // C8 
	_log_temp[10] = 0.001*Del_PC_X_SSP_XZMP_CON;        // C9
	_log_temp[11] = 0.001*Del_PC_Y_SSP_YZMP_CON;        // C10
	_log_temp[12] = 0.001*Ref_CON_SUM_X_ZMP_ERR;            // C11
	_log_temp[13] = 0.001*Ref_CON_SUM_Y_ZMP_ERR;            // C12
	_log_temp[14] = Del_RAR_RLAR_TORQ_CON;//FTSensor[RFFT].Fz;                  // C13
	_log_temp[15] = Del_LAR_RLAR_TORQ_CON;//FTSensor[LFFT].Fz;                  // C14
	_log_temp[16] = Del_RHY_VIB_CON;//SUM_X_ZMP_ERR;            // C15
	_log_temp[17] = Del_LHY_VIB_CON;//SUM_Y_ZMP_ERR;            // C16
	_log_temp[18] = CNT_SUM_ZMP_ERR;            // C17
	_log_temp[19] = RMS_Y_ZMP;            // C18
	_log_temp[20] = RF_Del_Y_Outside; // C19
	_log_temp[21] = Del_RLHP_Posture_Walking;  // C20
	_log_temp[22] = Del_RLAR_Posture_Walking; // C21
	_log_temp[23] = R2D*Body_Y_th;//Avg_RF_Pitch_Ang ;  // C22
	_log_temp[24] = R2D*Body_Y_th_d;//Avg_LF_Pitch_Ang ;  // C23
	_log_temp[25] = IMUSensor[CENTERIMU].Roll;    // C24
	_log_temp[26] = IMUSensor[CENTERIMU].Pitch;    // C25
	_log_temp[27] = IMUSensor[CENTERIMU].Roll_Velocity; // C26
	_log_temp[28] = IMUSensor[CENTERIMU].Pitch_Velocity; // C27
}


void Kirk_online_BP_mod(void)  // mod by inhyeok
{

	Fz[RF_1] = FTSensor[RFFT].Fz; Fz[LF_1] = FTSensor[LFFT].Fz;
	Fz[RF_2] = 0.; Fz[LF_2] = 0.;


	/* 착지 Threshold 설정 (로봇 총 무게 : 42 kgf ) */
	if(Fz[LF_1] > 98 && LandLOK_Flag == 0) LandLOK_Flag = 1; // 왼발 완전 착지 기준 ( 10kg )

	if(Fz[RF_1] > 98 && LandROK_Flag == 0) LandROK_Flag = 1; // 오른발 완전 착지 기준 ( 10kg )

	if(Fz[LF_1] > 29 && ThreeKG_LOK_Flag == 0) ThreeKG_LOK_Flag = 1; // 왼발 터치 다운 기준 ( 3kg )

	if(Fz[RF_1] > 29 && ThreeKG_ROK_Flag == 0) ThreeKG_ROK_Flag = 1; // 오른발 터치 다운 기준 ( 3kg )

	if(Fz[LF_1] > 15 && OneKG_LOK_Flag == 0) OneKG_LOK_Flag = 1; // 왼발 터치 다운 기준 ( 1kg )

	if(Fz[RF_1] > 15 && OneKG_ROK_Flag == 0) OneKG_ROK_Flag = 1; // 오른발 터치 다운 기준 ( 1kg )

	// 보행 중지 설정 //
	if(Lss == 0 && Rot_Ang == 0 && Step_CNT == pSharedMemory->Step_Number) pSharedMemory->Go_flag = 0;
    else if((Lss != 0 || Rot_Ang != 0) && Step_CNT == 2*pSharedMemory->Step_Number-1) pSharedMemory->Go_flag = 0;
    
    
    // ?? ?? ???? ?? //
    if(Change_flag == 1 || pSharedMemory->First_flag == 1){
    
      
		//Ls += 20; 
		//Rot_Ang += 2.5;
		//Delay_Ratio += 0.05;
		//Td = (int)(Ts*Delay_Ratio);

		
		Ls = pSharedMemory->Global_Ls;
		Lss = pSharedMemory->Global_Lss;
		Rot_Ang = pSharedMemory->Global_Rot_Ang;
		Delay_Ratio = pSharedMemory->Global_Delay_Ratio;
		Td_BP = (int)(Ts_BP*Delay_Ratio);
		Hs = pSharedMemory->Global_Hs;
		if(adj_amp == 0) Ab = pSharedMemory->Global_Ab;

		if(RMS_Y_ZMP != 0 && Lss == 0 && Rot_Ang == 0) Ab += adj_amp;

		if(pSharedMemory->First_flag == 1){
			Old_Lss = 0.; Old_Ls = 0.; Old_Rot_Ang = 0.;
		}
		


		if(Delay_Ratio < 0) {Delay_Ratio = 0. ; pSharedMemory->Global_Delay_Ratio = 0.;}


      
		  if(Old_Ls*Ls < 0) {  // ?? ?? ? Ls = 0 ? ?? ? ?? ??
			Ls = 0;
		  }

		  if(Old_Lss*Lss < 0) {  // ?? ?? ? Lss = 0 ? ?? ? ?? ??
			Lss = 0;
		  }
		  
		  if(Old_Rot_Ang*Rot_Ang < 0) {  // ?? ?? ? Rot_Ang = 0 ? ?? ? ?? ??
			Rot_Ang = 0;
		  }
		  
		  if(Rot_Ang*Lss < 0){ // ????? ??? ????? ??? ??? ??? ?. 
			  if(Lss != Old_Lss) Rot_Ang = 0.;
			  else if(Rot_Ang != Old_Rot_Ang) Lss = 0.;          
		  }

		  Old_Lss = Lss;
		  Old_Ls = Ls;	
		  Old_Rot_Ang = Rot_Ang;

		  pSharedMemory->Change_OK = 1;
		  pSharedMemory->First_flag =  0;
        
    }
        
    // 보행 패턴수행//
	Biped_walking_pattern(pSharedMemory->Go_flag,Ts_BP,Td_BP,shape_factor_F_BP,shape_factor_S_BP,Ab,Hs,Ls,Lss,Rot_Ang,Early_Landing_Flag,&Step_CNT,Ref_BC,Ref_R1,Ref_R2,Ref_R3,Ref_L1,Ref_L2,Ref_L3,Ref_th,Ref_ZMP,Walking_stage,&Change_flag,test_var);


    // 착지 Flag 들의 초기화 //
	if(Walking_stage[RF_1] == 10) {
		ThreeKG_ROK_Flag = 0;
		OneKG_ROK_Flag = 0;
		Early_Landing_Flag[RF_1] = 0;
		LandROK_Flag = 0;
	}

	if(Walking_stage[LF_1] == 10) {
		ThreeKG_LOK_Flag = 0;
		OneKG_LOK_Flag = 0;
		Early_Landing_Flag[LF_1] = 0;
		LandLOK_Flag = 0;
	}

	// ZMP 읽기 //
	readZMP_biped(_pRF0_3x1[1] + 0.001*Ref_R1[0], _pRF0_3x1[2] + 0.001*Ref_R1[1], 
		          _pLF0_3x1[1] + 0.001*Ref_L1[0], _pLF0_3x1[2] + 0.001*Ref_L1[1]);

	// 발 가속도 센서 처리 (발바닥 측방향 가속도 고주파 통과 필터 + 적분기)
	RF_Y_acc_HPF_INT = (1. - 2.*PI*F_cutoff_acc_HPF*Del_T)*Old_RF_Y_acc_HPF_INT + Del_T*(FTSensor[RFFT].dAccRoll+0.0);
	Old_RF_Y_acc_HPF_INT = RF_Y_acc_HPF_INT;

	LF_Y_acc_HPF_INT = (1. - 2.*PI*F_cutoff_acc_HPF*Del_T)*Old_LF_Y_acc_HPF_INT + Del_T*(FTSensor[LFFT].dAccRoll+0.0);
	Old_LF_Y_acc_HPF_INT = LF_Y_acc_HPF_INT;

	RF_X_acc_HPF_INT = (1. - 2.*PI*F_cutoff_acc_HPF*Del_T)*Old_RF_X_acc_HPF_INT + Del_T*(FTSensor[RFFT].dAccPitch+0.0);
	Old_RF_X_acc_HPF_INT = RF_X_acc_HPF_INT;

	LF_X_acc_HPF_INT = (1. - 2.*PI*F_cutoff_acc_HPF*Del_T)*Old_LF_X_acc_HPF_INT + Del_T*(FTSensor[LFFT].dAccPitch+0.0);
	Old_LF_X_acc_HPF_INT = LF_X_acc_HPF_INT;
	

	// DSP ZMP 제어 //
	if(Walking_stage[RF_1] == 0 && Walking_stage[LF_1] == 0){ 
		final_gain_DSP_ZMP_CON = 1.0;//0.5*(1-cos(PI*CNT_final_gain_DSP_ZMP_CON/20));

		Del_PC_X_DSP_XZMP_CON = final_gain_DSP_ZMP_CON*P2_XZMP_CON_E(-Del_PC_X_DSP_XZMP_CON,X_ZMP - X_ZMP_OFFSET_BP - Ref_ZMP[0], 1);
		Del_PC_Y_DSP_YZMP_CON = final_gain_DSP_ZMP_CON*P2_YZMP_CON_E(-Del_PC_Y_DSP_YZMP_CON,Y_ZMP - Ref_ZMP[1], 1);
	
		if(CNT_final_gain_DSP_ZMP_CON < 20) CNT_final_gain_DSP_ZMP_CON++;

		Old_Del_PC_X_DSP_XZMP_CON = Del_PC_X_DSP_XZMP_CON;
		Old_Del_PC_Y_DSP_YZMP_CON = Del_PC_Y_DSP_YZMP_CON;
		CNT_DSP_XYZMP_CON = 0;
	}
	else if((Walking_stage[RF_1] == 10 && Walking_stage[LF_1] == 0) || (Walking_stage[RF_1] == 0 && Walking_stage[LF_1] == 10)){
		Del_PC_X_DSP_XZMP_CON = Old_Del_PC_X_DSP_XZMP_CON - Old_Del_PC_X_DSP_XZMP_CON*0.5*(1-cos(PI*CNT_DSP_XYZMP_CON/40));
		Del_PC_Y_DSP_YZMP_CON = Old_Del_PC_Y_DSP_YZMP_CON - Old_Del_PC_Y_DSP_YZMP_CON*0.5*(1-cos(PI*CNT_DSP_XYZMP_CON/40));
		
		if(CNT_DSP_XYZMP_CON < 40) CNT_DSP_XYZMP_CON++;
		else{
			Old_Del_PC_X_DSP_XZMP_CON = 0.; Old_Del_PC_Y_DSP_YZMP_CON = 0.;
			P2_XZMP_CON_E(0,0,0); P2_YZMP_CON_E(0,0,0);
		}

		CNT_final_gain_DSP_ZMP_CON = 0;

	}
	else if(Walking_stage[RF_1] == 20  && Walking_stage[LF_1] == 0 && OneKG_ROK_Flag == 1){

		final_gain_DSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_DSP_ZMP_CON/20));
			
		Del_PC_X_DSP_XZMP_CON = final_gain_DSP_ZMP_CON*P2_XZMP_CON_E(-Del_PC_X_DSP_XZMP_CON,X_ZMP - X_ZMP_OFFSET_BP - Ref_ZMP[0], 1);
		Del_PC_Y_DSP_YZMP_CON = final_gain_DSP_ZMP_CON*P2_YZMP_CON_E(-Del_PC_Y_DSP_YZMP_CON,Y_ZMP - Ref_ZMP[1], 1);

		if(CNT_final_gain_DSP_ZMP_CON < 20) CNT_final_gain_DSP_ZMP_CON++;

		Old_Del_PC_X_DSP_XZMP_CON = Del_PC_X_DSP_XZMP_CON;
		Old_Del_PC_Y_DSP_YZMP_CON = Del_PC_Y_DSP_YZMP_CON;
		CNT_DSP_XYZMP_CON = 0;

	}
	else if(Walking_stage[RF_1] == 0  && Walking_stage[LF_1] == 20 && OneKG_LOK_Flag == 1){

		final_gain_DSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_DSP_ZMP_CON/20));
			
		Del_PC_X_DSP_XZMP_CON = final_gain_DSP_ZMP_CON*P2_XZMP_CON_E(-Del_PC_X_DSP_XZMP_CON,X_ZMP - X_ZMP_OFFSET_BP - Ref_ZMP[0], 1);
		Del_PC_Y_DSP_YZMP_CON = final_gain_DSP_ZMP_CON*P2_YZMP_CON_E(-Del_PC_Y_DSP_YZMP_CON,Y_ZMP - Ref_ZMP[1], 1);

		if(CNT_final_gain_DSP_ZMP_CON < 20) CNT_final_gain_DSP_ZMP_CON++;

		Old_Del_PC_X_DSP_XZMP_CON = Del_PC_X_DSP_XZMP_CON;
		Old_Del_PC_Y_DSP_YZMP_CON = Del_PC_Y_DSP_YZMP_CON;
		CNT_DSP_XYZMP_CON = 0;

	}


	// SSP ZMP 제어 //
	if((Walking_stage[RF_1] == 10  && Walking_stage[LF_1] == 0) || Walking_stage[RF_1] == 0  && Walking_stage[LF_1] == 10) {

		final_gain_SSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_SSP_ZMP_CON/30));

		P1_Y_th_th_d_Observer(-Del_PC_Y_SSP_YZMP_CON, Y_ZMP - BC_Y_OFFSET, 1);
				
		Del_PC_X_SSP_XZMP_CON = Old_Del_PC_X_SSP_XZMP_CON + final_gain_SSP_ZMP_CON*(-Old_Del_PC_X_SSP_XZMP_CON + 1.0*P1_XZMP_CON_E(-Del_PC_X_SSP_XZMP_CON, X_ZMP - X_ZMP_OFFSET_BP - Ref_ZMP[0], 1));
		Del_PC_Y_SSP_YZMP_CON = Old_Del_PC_Y_SSP_YZMP_CON + final_gain_SSP_ZMP_CON*(-Old_Del_PC_Y_SSP_YZMP_CON + 1.0*P1_YZMP_CON_E(-Del_PC_Y_SSP_YZMP_CON, Y_ZMP - Ref_ZMP[1], 1));

		
		if(CNT_final_gain_SSP_ZMP_CON < 30) CNT_final_gain_SSP_ZMP_CON++;
		else{
			Old_Del_PC_X_SSP_XZMP_CON = 0.; Old_Del_PC_Y_SSP_YZMP_CON = 0.;
		}
				
		CNT_SSP_ZMP_CON = 0;
		
		/*
		Temp_CNT = 0;

		if(Walking_stage[LF_1] == 10){
			RF_Del_Y_Outside = Old_RF_Del_Y_Outside - Old_RF_Del_Y_Outside*0.5*(1-cos(PI*Temp_CNT_2/(Ts_BP/3)));

			if(Temp_CNT_2 < Ts_BP/3) Temp_CNT_2++;
			else {Old_RF_Del_Y_Outside = 0.; RF_Del_Y_Outside = 0.;}
		}

		Outside_Land_Flag = 0;
		*/
		

	}
	else if(Walking_stage[RF_1] == 20  && Walking_stage[LF_1] == 0 && OneKG_ROK_Flag == 0) {

		P1_Y_th_th_d_Observer(-Del_PC_Y_SSP_YZMP_CON, Y_ZMP - BC_Y_OFFSET, 1);
			
		/*
		if(Temp_CNT == (Ts_BP/10) && Body_Y_th < 0) Outside_Land_Flag = 1;

		if(Outside_Land_Flag) {

			RF_Del_Y_Outside = 2.0*0.5*(1-cos(PI*(Temp_CNT - Ts_BP/10)/(Ts_BP/3 - Ts_BP/10)));
			Old_RF_Del_Y_Outside = RF_Del_Y_Outside;

		}

		if(Temp_CNT < Ts_BP/3) Temp_CNT++;

		Temp_CNT_2 = 0;
		*/		


		Del_PC_X_SSP_XZMP_CON = 1.0*P1_XZMP_CON_E(-Del_PC_X_SSP_XZMP_CON, X_ZMP - X_ZMP_OFFSET_BP - Ref_ZMP[0], 1);
		Del_PC_Y_SSP_YZMP_CON = 1.0*P1_YZMP_CON_E(-Del_PC_Y_SSP_YZMP_CON, Y_ZMP - Ref_ZMP[1], 1);
		
		Old_Del_PC_X_SSP_XZMP_CON_2 = Del_PC_X_SSP_XZMP_CON;
		Old_Del_PC_Y_SSP_YZMP_CON_2 = Del_PC_Y_SSP_YZMP_CON;

		CNT_final_gain_SSP_ZMP_CON = 0;

	}
	else if(Walking_stage[RF_1] == 0  && Walking_stage[LF_1] == 20 && OneKG_LOK_Flag == 0) {

		P1_Y_th_th_d_Observer(-Del_PC_Y_SSP_YZMP_CON, Y_ZMP - BC_Y_OFFSET, 1);

		Del_PC_X_SSP_XZMP_CON = 1.0*P1_XZMP_CON_E(-Del_PC_X_SSP_XZMP_CON, X_ZMP - X_ZMP_OFFSET_BP - Ref_ZMP[0], 1);
		Del_PC_Y_SSP_YZMP_CON = 1.0*P1_YZMP_CON_E(-Del_PC_Y_SSP_YZMP_CON, Y_ZMP - Ref_ZMP[1], 1);

		Old_Del_PC_X_SSP_XZMP_CON_2 = Del_PC_X_SSP_XZMP_CON;
		Old_Del_PC_Y_SSP_YZMP_CON_2 = Del_PC_Y_SSP_YZMP_CON;

		CNT_final_gain_SSP_ZMP_CON = 0;

	}
	else if(Walking_stage[RF_1] == 0 && Walking_stage[LF_1] == 0 && Real_Stop_Flag == 0){


		Del_PC_X_SSP_XZMP_CON = Old_Del_PC_X_SSP_XZMP_CON_2 - Old_Del_PC_X_SSP_XZMP_CON_2*0.5*(1-cos(PI*CNT_SSP_ZMP_CON/100));
		Del_PC_Y_SSP_YZMP_CON = Old_Del_PC_Y_SSP_YZMP_CON_2 - Old_Del_PC_Y_SSP_YZMP_CON_2*0.5*(1-cos(PI*CNT_SSP_ZMP_CON/100));

		Old_Del_PC_X_SSP_XZMP_CON = Del_PC_X_SSP_XZMP_CON;
		Old_Del_PC_Y_SSP_YZMP_CON = Del_PC_Y_SSP_YZMP_CON;

		if(CNT_SSP_ZMP_CON < 100) CNT_SSP_ZMP_CON++;
		else {
				Del_PC_Y_SSP_YZMP_CON = 0.0;
				Old_Del_PC_Y_SSP_YZMP_CON = 0.0;
				Del_PC_X_SSP_XZMP_CON = 0.0;
				Old_Del_PC_X_SSP_XZMP_CON = 0.0;
				CNT_SSP_ZMP_CON = 0;
				P1_XZMP_CON_E(0,0,0);
				P1_YZMP_CON_E(0,0,0);
				P1_Y_th_th_d_Observer(0,0,0);
				CNT_final_gain_SSP_ZMP_CON = 0;
				Old_Del_PC_X_SSP_XZMP_CON_2 = 0.; Old_Del_PC_Y_SSP_YZMP_CON_2 = 0.;
		}

	}
	


	// ZMP 적분제어 : 한 보행 주기(2스텝)동안 X, Y ZMP 오차를 누적한 양이 작아지도록 몸통의 중심위치를 점진적으로 수평 이동 // 
	if(Step_CNT >= 2 && Real_Stop_Flag == 0){   // 첫번째 스텝은 비대칭이라 무시. 두 번째 스텝부터 반영함.
		if(Step_CNT >= 4 && Old_Step_CNT%2 == 1 && Step_CNT%2 == 0 ){
			
			Old_CON_SUM_X_ZMP_ERR = CON_SUM_X_ZMP_ERR;
			Old_CON_SUM_Y_ZMP_ERR = CON_SUM_Y_ZMP_ERR;

			CON_SUM_X_ZMP_ERR += 0.2*(SUM_X_ZMP_ERR/CNT_SUM_ZMP_ERR); // 0.1 --> 적분 게인
			CON_SUM_Y_ZMP_ERR += 0.2*(SUM_Y_ZMP_ERR/CNT_SUM_ZMP_ERR); 

			if(CON_SUM_X_ZMP_ERR > 10.0) CON_SUM_X_ZMP_ERR = 10.0;   // 최종 제어 입력 Range 설정
			else if(CON_SUM_X_ZMP_ERR < -10.0) CON_SUM_X_ZMP_ERR = -10.0;
				 
			if(CON_SUM_Y_ZMP_ERR > 10.0) CON_SUM_Y_ZMP_ERR = 10.0;   // 최종 제어 입력 Range 설정
			else if(CON_SUM_Y_ZMP_ERR < -10.0) CON_SUM_Y_ZMP_ERR = -10.0;

			RMS_Y_ZMP = sqrt(SUM_RMS_Y_ZMP/CNT_SUM_ZMP_ERR);

			pSharedMemory->WB_fRS[0] = RMS_Y_ZMP;

			if(RMS_Y_ZMP != 0 )	adj_amp = 0.*0.05*(DES_RMS_Y_ZMP - RMS_Y_ZMP);

			SUM_X_ZMP_ERR = 0.;
			SUM_Y_ZMP_ERR = 0.;
			SUM_RMS_Y_ZMP = 0.;
			CNT_SUM_ZMP_ERR = 0;
			CNT_CON_SUM_XY_ZMP_ERR = 0;
		}
		else {
				SUM_Y_ZMP_ERR += Y_ZMP - Ref_ZMP[1];
				SUM_X_ZMP_ERR += X_ZMP - X_ZMP_OFFSET_BP - Ref_ZMP[0] + 8.0*Ls/100;
								

				/* RMS ZMP SUM 계산 */
				SUM_RMS_Y_ZMP += (Y_ZMP - BC_Y_OFFSET)*(Y_ZMP - BC_Y_OFFSET);

				CNT_SUM_ZMP_ERR++;

				if(SUM_X_ZMP_ERR > 3200.0) SUM_X_ZMP_ERR = 3200.0;   // 추가 적분 제어 입력량 Limit 설정 
				else if(SUM_X_ZMP_ERR < -3200.0) SUM_X_ZMP_ERR = -3200.0;
				
				if(SUM_Y_ZMP_ERR > 3200.0) SUM_Y_ZMP_ERR = 3200.0;   // 추가 적분 제어 입력량 Limit 설정
				else if(SUM_Y_ZMP_ERR < -3200.0) SUM_Y_ZMP_ERR = -3200.0;

		}
	}
	else{

		SUM_X_ZMP_ERR = 0.;
		SUM_Y_ZMP_ERR = 0.;
		SUM_RMS_Y_ZMP = 0.;
		CNT_SUM_ZMP_ERR = 0;
		RMS_Y_ZMP = 0.;
	}

	Ref_CON_SUM_X_ZMP_ERR = Old_CON_SUM_X_ZMP_ERR + (CON_SUM_X_ZMP_ERR - Old_CON_SUM_X_ZMP_ERR)*0.5*(1-cos(PI*CNT_CON_SUM_XY_ZMP_ERR/10));
	Ref_CON_SUM_Y_ZMP_ERR = Old_CON_SUM_Y_ZMP_ERR + (CON_SUM_Y_ZMP_ERR - Old_CON_SUM_Y_ZMP_ERR)*0.5*(1-cos(PI*CNT_CON_SUM_XY_ZMP_ERR/10));

	if(CNT_CON_SUM_XY_ZMP_ERR < 10) CNT_CON_SUM_XY_ZMP_ERR++;		

	pSharedMemory->WB_fRS[1] = Ref_CON_SUM_X_ZMP_ERR;
	pSharedMemory->WB_fRS[2] = Ref_CON_SUM_Y_ZMP_ERR;


	// RLAR & RLAP 버츄얼 스프링 댐퍼 제어 //
	if(Walking_stage[LF_1] != 10 && OneKG_LOK_Flag == 1 && LandLOK_Flag == 0) {
 			Del_LAR_RLAR_TORQ_CON = 1.0*RLAR_TORQ_CON(FTSensor[LFFT].Mx,1)*0.000000;  //inhyeok 
			Del_LAP_RLAP_TORQ_CON = -1.0*RLAP_TORQ_CON(-FTSensor[LFFT].My,1)*0.000000; //inhyeok
 			CNT_RLAR_TORQ_CON = 0.;
 			Old_Del_LAR_RLAR_TORQ_CON = Del_LAR_RLAR_TORQ_CON;
			Old_Del_LAP_RLAP_TORQ_CON = Del_LAP_RLAP_TORQ_CON;

 	}
	else if(Walking_stage[LF_1] == 10){
			Del_LAR_RLAR_TORQ_CON = Old_Del_LAR_RLAR_TORQ_CON - Old_Del_LAR_RLAR_TORQ_CON*0.5*(1-cos(PI*CNT_RLAR_TORQ_CON/40));
			Del_LAP_RLAP_TORQ_CON = Old_Del_LAP_RLAP_TORQ_CON - Old_Del_LAP_RLAP_TORQ_CON*0.5*(1-cos(PI*CNT_RLAR_TORQ_CON/40));
 			
 			if (CNT_RLAR_TORQ_CON < 40 /*&& Old_Del_LAR_RLAR_TORQ_CON != 0.0*/) CNT_RLAR_TORQ_CON++;
 			else {Old_Del_LAR_RLAR_TORQ_CON = 0.; Del_LAR_RLAR_TORQ_CON = 0.; /*CNT_RLAR_TORQ_CON = 0;*/ RLAR_TORQ_CON(0,0);
				  Old_Del_LAP_RLAP_TORQ_CON = 0.; Del_LAP_RLAP_TORQ_CON = 0.; RLAP_TORQ_CON(0,0);}
	}

	if(Walking_stage[RF_1] != 10 && OneKG_ROK_Flag == 1 && LandROK_Flag == 0) {
			Del_RAR_RLAR_TORQ_CON = 1.0*RLAR_TORQ_CON(FTSensor[RFFT].Mx,1)*0.000000; //inhyeok
			Del_RAP_RLAP_TORQ_CON = -1.0*RLAP_TORQ_CON(-FTSensor[RFFT].My,1)*0.000000; //inhyeok
 			CNT_RLAR_TORQ_CON = 0.;
 			Old_Del_RAR_RLAR_TORQ_CON = Del_RAR_RLAR_TORQ_CON;
			Old_Del_RAP_RLAP_TORQ_CON = Del_RAP_RLAP_TORQ_CON;
	}
	else if(Walking_stage[RF_1] == 10){
			Del_RAR_RLAR_TORQ_CON = Old_Del_RAR_RLAR_TORQ_CON - Old_Del_RAR_RLAR_TORQ_CON*0.5*(1-cos(PI*CNT_RLAR_TORQ_CON/40));
			Del_RAP_RLAP_TORQ_CON = Old_Del_RAP_RLAP_TORQ_CON - Old_Del_RAP_RLAP_TORQ_CON*0.5*(1-cos(PI*CNT_RLAR_TORQ_CON/40));
 
 			if (CNT_RLAR_TORQ_CON < 40 /*&& Old_Del_RAR_RLAR_TORQ_CON != 0.0*/) CNT_RLAR_TORQ_CON++;
 			else {Old_Del_RAR_RLAR_TORQ_CON = 0.; Del_RAR_RLAR_TORQ_CON = 0.; /*CNT_RLAR_TORQ_CON = 0;*/ RLAR_TORQ_CON(0,0);
				  Old_Del_RAP_RLAP_TORQ_CON = 0.; Del_RAP_RLAP_TORQ_CON = 0.; RLAP_TORQ_CON(0,0);}

	}

	
	// Swing foot 측방향/전진방향 진동 제어, 몸통 Z 방향 진동 제어 // 
	if(Walking_stage[RF_1] == 10){

			final_gain_SSP_Y_VIB_CON_2 = 0.5*(1-cos(PI*CNT_final_gain_SSP_Y_VIB_CON_2/10));

			Del_RHR_VIB_CON = -1.0*final_gain_SSP_Y_VIB_CON_2*R2D*SSP_Y_VIB_CON_E_RF(D2R*Del_RHR_VIB_CON,0.006*RF_Y_acc_HPF_INT, 1); // [deg]
			Del_RHP_VIB_CON = -1.0*final_gain_SSP_Y_VIB_CON_2*R2D*SSP_X_VIB_CON_E_RF(D2R*Del_RHP_VIB_CON,0.006*RF_X_acc_HPF_INT, 1); // [deg]
			if(Rot_Ang == 0) Del_LHY_VIB_CON =  1.0*final_gain_SSP_Y_VIB_CON_2*R2D*SSP_Z_VIB_CON_E(-D2R*Del_LHY_VIB_CON, D2R*IMUSensor[CENTERIMU].Yaw_Velocity, 1); // [Deg]
			else Del_LHY_VIB_CON = 0.;

			if(CNT_final_gain_SSP_Y_VIB_CON_2 < 10) CNT_final_gain_SSP_Y_VIB_CON_2++;

			Old_Del_RHR_VIB_CON = Del_RHR_VIB_CON; Old_Del_RHP_VIB_CON = Del_RHP_VIB_CON;
			CNT_FOOT_VIB_CON = 0;

	}
	else if(Walking_stage[RF_1] == 20){

			final_gain_SSP_Y_VIB_CON_4 = 1.0 - 0.5*(1-cos(PI*CNT_FOOT_VIB_CON/30));

			Del_RHR_VIB_CON = -1.0*final_gain_SSP_Y_VIB_CON_4*R2D*SSP_Y_VIB_CON_E_RF(D2R*Del_RHR_VIB_CON,0.006*RF_Y_acc_HPF_INT, 1); // [deg]
			Del_RHP_VIB_CON = -1.0*final_gain_SSP_Y_VIB_CON_4*R2D*SSP_X_VIB_CON_E_RF(D2R*Del_RHP_VIB_CON,0.006*RF_X_acc_HPF_INT, 1); // [deg]
			if(Rot_Ang == 0) Del_LHY_VIB_CON =  1.0*final_gain_SSP_Y_VIB_CON_4*R2D*SSP_Z_VIB_CON_E(-D2R*Del_LHY_VIB_CON, D2R*IMUSensor[CENTERIMU].Yaw_Velocity, 1); // [Deg]
			else  Del_LHY_VIB_CON = 0.;


			if(CNT_FOOT_VIB_CON < 30) CNT_FOOT_VIB_CON++;
			else {Old_Del_RHR_VIB_CON = 0.0; SSP_Y_VIB_CON_E_RF(0,0,0); Old_Del_RHP_VIB_CON = 0.0; SSP_X_VIB_CON_E_RF(0,0,0); 
			      CNT_final_gain_SSP_Y_VIB_CON_1 = 0; final_gain_SSP_Y_VIB_CON_1 = 0.; SSP_Z_VIB_CON_E(0,0,0);}

	}

	if(Walking_stage[LF_1] == 10){

			final_gain_SSP_Y_VIB_CON_1 = 0.5*(1-cos(PI*CNT_final_gain_SSP_Y_VIB_CON_1/10));

			Del_LHR_VIB_CON = -1.0*final_gain_SSP_Y_VIB_CON_1*R2D*SSP_Y_VIB_CON_E_LF(D2R*Del_LHR_VIB_CON,0.006*LF_Y_acc_HPF_INT, 1);
			Del_LHP_VIB_CON = -1.0*final_gain_SSP_Y_VIB_CON_1*R2D*SSP_X_VIB_CON_E_LF(D2R*Del_LHP_VIB_CON,0.006*LF_X_acc_HPF_INT, 1);
			if(Rot_Ang == 0) Del_RHY_VIB_CON =  1.0*final_gain_SSP_Y_VIB_CON_1*R2D*SSP_Z_VIB_CON_E(-D2R*Del_RHY_VIB_CON, D2R*IMUSensor[CENTERIMU].Yaw_Velocity, 1);
			else Del_RHY_VIB_CON = 0.;

			if(CNT_final_gain_SSP_Y_VIB_CON_1 < 10) CNT_final_gain_SSP_Y_VIB_CON_1++;

			Old_Del_LHR_VIB_CON = Del_LHR_VIB_CON; Old_Del_LHP_VIB_CON = Del_LHP_VIB_CON;
			CNT_FOOT_VIB_CON = 0;

	}
	else if(Walking_stage[LF_1] == 20){
			final_gain_SSP_Y_VIB_CON_3 = 1.0 - 0.5*(1-cos(PI*CNT_FOOT_VIB_CON/30));

			Del_LHR_VIB_CON = -1.0*final_gain_SSP_Y_VIB_CON_3*R2D*SSP_Y_VIB_CON_E_LF(D2R*Del_LHR_VIB_CON,0.006*LF_Y_acc_HPF_INT, 1);
			Del_LHP_VIB_CON = -1.0*final_gain_SSP_Y_VIB_CON_3*R2D*SSP_X_VIB_CON_E_LF(D2R*Del_LHP_VIB_CON,0.006*LF_X_acc_HPF_INT, 1);
			if(Rot_Ang == 0) Del_RHY_VIB_CON =  1.0*final_gain_SSP_Y_VIB_CON_3*R2D*SSP_Z_VIB_CON_E(-D2R*Del_RHY_VIB_CON, D2R*IMUSensor[CENTERIMU].Yaw_Velocity, 1);
			else  Del_RHY_VIB_CON = 0.;


			if(CNT_FOOT_VIB_CON < 30) CNT_FOOT_VIB_CON++;
			else {Old_Del_LHR_VIB_CON = 0.0; SSP_Y_VIB_CON_E_LF(0,0,0); Old_Del_LHP_VIB_CON = 0.0; SSP_X_VIB_CON_E_LF(0,0,0);
			      CNT_final_gain_SSP_Y_VIB_CON_2 = 0; final_gain_SSP_Y_VIB_CON_2 = 0.; SSP_Z_VIB_CON_E(0,0,0);}

	}


	// 착지 시 지면 기울기 측정 //
	if(Walking_stage[RF_1] == 0 && Walking_stage[LF_1] == 10){
	
		Avg_RF_Pitch_Ang = (Avg_CNT - 1.)/Avg_CNT*Old_Avg_RF_Pitch_Ang + 1./Avg_CNT*FTSensor[RFFT].dAccPitch;  

		Old_Avg_RF_Pitch_Ang = Avg_RF_Pitch_Ang;

		Avg_CNT++;

	}
	else if(Walking_stage[RF_1] == 10 && Walking_stage[LF_1] == 0){
	

		Avg_LF_Pitch_Ang = (Avg_CNT - 1.)/Avg_CNT*Old_Avg_LF_Pitch_Ang + 1./Avg_CNT*FTSensor[LFFT].dAccPitch;

		Old_Avg_LF_Pitch_Ang = Avg_LF_Pitch_Ang;

		Avg_CNT++;

	}
	else {
		Avg_CNT = 1; Old_Avg_RF_Pitch_Ang = 0.; Avg_RF_Pitch_Ang = 0.;
		Avg_CNT = 1; Old_Avg_LF_Pitch_Ang = 0.; Avg_LF_Pitch_Ang = 0.;
	}


	

	
	// 보행 시 몸통 수직 자세 제어 : 보행 시 가속도값 적분제어 (엉덩이 피치/발목 롤 수정) //
	Del_RLHP_Posture_Walking = -TP_INT_CON_WALK(0.1,1.0*IMUSensor[CENTERIMU].Pitch,1);   // 가속도계 + ZMP 에 의한 경사만으로 상체 피치 기울기 교정 //
	Del_RLFZ_Posture_Walking =  1.*TR_INT_CON_WALK(-0.6,1.0*IMUSensor[CENTERIMU].Roll,1);    // 가속도계 + ZMP 에 의한 경사만으로 상체 롤 기울기 교정 //
	Del_RLAR_Posture_Walking =  -atan(Del_RLFZ_Posture_Walking/(1000.*_pLF0_3x1[2])); // 롤 경사에 의한 발목 롤 교정 //

   
	// Early Landing Algorithm //
	for(i = 0; i<6; i++){
		if(Walking_stage[i] == 20 && Fz[i] > EARLY_LAND_THRES_1) {		
			Early_Landing_Flag[i] = 1; 			
			//if(i == LF1) First_Flag_1 = 0;
		}
    }
        
        
    if(Old_Walking_stage[RF_1] == 20 && Walking_stage[RF_1] == 0) {
			
		temp_Z_avg = temp_Z[RF_1];

		All_Foot_Z_Flag = 1;
	}
    else if(Old_Walking_stage[LF_1] == 20 && Walking_stage[LF_1] == 0) {
			
		temp_Z_avg = temp_Z[LF_1];

		All_Foot_Z_Flag = 1;
    }

    if(All_Foot_Z_Flag ){
		All_Foot_Z = Old_All_Foot_Z + (temp_Z_avg - Old_All_Foot_Z)*0.5*(1-cos(PI*All_Foot_Z_CNT/(Ts_BP/2)));

		if(All_Foot_Z_CNT < Ts_BP/2) All_Foot_Z_CNT++;
		else{
			Old_All_Foot_Z = All_Foot_Z;
			All_Foot_Z_Flag = 0;
			All_Foot_Z_CNT = 0;
		}
    }


	if(Early_Landing_Flag[0] == 1 && temp_X[0] == 0. && temp_Y[0] == 0. && temp_th[0] == 0.) {temp_X[0] = Ref_R1[0]; temp_Y[0] = Ref_R1[1]; temp_th[0] = Ref_th[0]; EL_Del_xy_CNT[0] = 0;}
	else if(Early_Landing_Flag[0] == 1 && (temp_X[0] != 0. || temp_Y[0] != 0. || temp_th[0] != 0.)) {
		   Ref_R1[0] = temp_X[0]; Ref_R1[1] = temp_Y[0]; Ref_th[0] = temp_th[0];
		   Old_Ref_R1[0] = Ref_R1[0]; Old_Ref_R1[1] = Ref_R1[1]; Old_Ref_th[0] = Ref_th[0];
	}
	else if(Early_Landing_Flag[0] == 0 && (temp_X[0] != 0. || temp_Y[0] != 0. || temp_th[0] != 0.)){

	if(temp_X[0] != 0.) Ref_R1[0] = Old_Ref_R1[0] + (Ref_R1[0] - Old_Ref_R1[0])*0.5*(1-cos(PI/60*EL_Del_xy_CNT[0]));
	if(temp_Y[0] != 0.) Ref_R1[1] = Old_Ref_R1[1] + (Ref_R1[1] - Old_Ref_R1[1])*0.5*(1-cos(PI/60*EL_Del_xy_CNT[0]));
	if(temp_th[0] != 0.) Ref_th[0] = Old_Ref_th[0] + (Ref_th[0] - Old_Ref_th[0])*0.5*(1-cos(PI/60*EL_Del_xy_CNT[0]));

	if(EL_Del_xy_CNT[0] < 60 && WP_Flow_Control_Flag == 0) EL_Del_xy_CNT[0]++;
	else if(WP_Flow_Control_Flag == 0) {Old_Ref_R1[0] = 0.;temp_X[0] = 0.; Old_Ref_R1[1] = 0.;temp_Y[0] = 0.; Old_Ref_th[0] = 0.; temp_th[0] = 0.;}
	}
	
   if(Early_Landing_Flag[3] == 1 && temp_X[3] == 0. && temp_Y[3] == 0. && temp_th[3] == 0.) {temp_X[3] = Ref_L1[0]; temp_Y[3] = Ref_L1[1]; temp_th[3] = Ref_th[3]; EL_Del_xy_CNT[3] = 0;}
   else if(Early_Landing_Flag[3] == 1 && (temp_X[3] != 0. || temp_Y[3] != 0. || temp_th[3] != 0.)) {
	Ref_L1[0] = temp_X[3]; Ref_L1[1] = temp_Y[3]; Ref_th[3] = temp_th[3];
	Old_Ref_L1[0] = Ref_L1[0]; Old_Ref_L1[1] = Ref_L1[1]; Old_Ref_th[3] = Ref_th[3];
   }
   else if(Early_Landing_Flag[3] == 0 && (temp_X[3] != 0. || temp_Y[3] != 0. || temp_th[3] != 0.)){
		   if(temp_X[3] != 0.) Ref_L1[0] = Old_Ref_L1[0] + (Ref_L1[0] - Old_Ref_L1[0])*0.5*(1-cos(PI/60*EL_Del_xy_CNT[3]));
	if(temp_Y[3] != 0.) Ref_L1[1] = Old_Ref_L1[1] + (Ref_L1[1] - Old_Ref_L1[1])*0.5*(1-cos(PI/60*EL_Del_xy_CNT[3]));
	if(temp_th[3] != 0.) Ref_th[3] = Old_Ref_th[3] + (Ref_th[3] - Old_Ref_th[3])*0.5*(1-cos(PI/60*EL_Del_xy_CNT[3]));

	if(EL_Del_xy_CNT[3] < 60 && WP_Flow_Control_Flag == 0) EL_Del_xy_CNT[3]++;
	else if(WP_Flow_Control_Flag == 0) {Old_Ref_L1[0] = 0.;temp_X[3] = 0.; Old_Ref_L1[1] = 0.;temp_Y[3] = 0.; Old_Ref_th[3] = 0.; temp_th[3] = 0.;}
   }




	// Late Landing Algorithm //
    for( i = 0; i<6; i++){
        if(Old_Walking_stage[i] == 20 && Walking_stage[i] == 0 && Early_Landing_Flag[i] == 0 && LL_Del_z[i] == 0. ) {

			Late_Landing_Flag[i] = 1;

		}		

        if(Late_Landing_Flag[i] == 1 && Fz[i] < EARLY_LAND_THRES_1 /*&& LL_Del_z[i] < LATE_LAND_MAX_DEPTH*/ && Walking_stage[i] == 0){
				
			LL_Del_z[i] = 0.*Late_Land_Vertical_Disp(i,(EARLY_LAND_THRES_1+5.0), Fz[i], 1);	
			Old_LL_Del_z[i] = LL_Del_z[i];
			LL_Del_z_CNT[i] = 0;
			
        }
        else if(Late_Landing_Flag[i] == 1 && Fz[i] >= EARLY_LAND_THRES_1 && Walking_stage[i] == 0){

	           Late_Landing_Flag[i] = 2;

        }
        else if((Late_Landing_Flag[i] == 1 || Late_Landing_Flag[i] == 2 ) && Walking_stage[i] == 10){

			LL_Del_z[i] = Old_LL_Del_z[i] - Old_LL_Del_z[i]*0.5*(1-cos(PI*LL_Del_z_CNT[i]/(Ts_BP/2)));

			if(LL_Del_z_CNT[i] < (Ts_BP/2) && WP_Flow_Control_Flag == 0) LL_Del_z_CNT[i]++;
			else if(WP_Flow_Control_Flag == 0){
					Late_Landing_Flag[i] = 0;
					Late_Land_Vertical_Disp(i,0,0,0);
					Old_LL_Del_z[i] = 0.;
					LL_Del_z[i] = 0.;	
			}
        }
    }

      // ?? ??? ?? ?? ?? ?? ???? : ???? ??? ???? ??? ??? ? ??? ?? ?? ?.
      if(Late_Landing_Flag[RF_1] == 1 /*&& Old_LL_Del_z[RF_1] < LATE_LAND_MAX_DEPTH*/ ) WP_Flow_Control_Flag = 1;
      else if(Late_Landing_Flag[LF_1] == 1 /*&& Old_LL_Del_z[LF_1] < LATE_LAND_MAX_DEPTH*/ ) WP_Flow_Control_Flag = 1;
      else WP_Flow_Control_Flag = 0;



    //readZMP_biped(_pRF0_3x1[1] + 0.001*Ref_R1[0], _pRF0_3x1[2] + 0.001*Ref_R1[1], 
	//	          _pLF0_3x1[1] + 0.001*Ref_L1[0], _pLF0_3x1[2] + 0.001*Ref_L1[1]);

	//ZMP_CON_MANAGER_BP(FTSensor[RFFT].Fz, FTSensor[LFFT].Fz, Land_ROK_Flag, Land_LOK_Flag, 1);   // update

	//BC_Y_CON = -Del_YZMP_CON;  // update
	//BC_X_CON = -Del_XZMP_CON;  // update

	pSharedMemory->ZMP[0] = X_ZMP;
	pSharedMemory->ZMP[1] = Y_ZMP;

	for(i=0;i<6;i++) Old_Walking_stage[i] = Walking_stage[i];

	Old_Step_CNT = Step_CNT;

	if(0){ //no control

		BC_X_CON = 0.;
		BC_Y_CON = 0.;

		
		WP_Flow_Control_Flag = 0;

		for(i=0;i<6;i++) {
				Early_Landing_Flag[i] = 0;
				Late_Landing_Flag[i] = 0;
				temp_Z[i] = 0.;
				LL_Del_z[i] = 0.; Old_LL_Del_z[i] = 0.; LL_Del_z_CNT[i] = 0;
				Late_Land_Vertical_Disp(i,0,0,0);
		}

		Del_PC_X_SSP_XZMP_CON = 0.; Del_PC_Y_SSP_YZMP_CON = 0.;    
		Del_PC_X_DSP_XZMP_CON = 0.; Del_PC_Y_DSP_YZMP_CON = 0.;   
		Ref_CON_SUM_X_ZMP_ERR = 0.; Ref_CON_SUM_Y_ZMP_ERR = 0.; CNT_CON_SUM_XY_ZMP_ERR = 0;
		CON_SUM_X_ZMP_ERR = 0.; CON_SUM_Y_ZMP_ERR = 0.;

		Del_LAR_RLAR_TORQ_CON = 0.; Del_LAP_RLAP_TORQ_CON = 0.; Del_RAR_RLAR_TORQ_CON = 0.; Del_RAP_RLAP_TORQ_CON = 0.;

		Del_RHR_VIB_CON = 0.; Del_LHR_VIB_CON = 0., Del_RHP_VIB_CON = 0.; Del_LHP_VIB_CON = 0.; Del_RHY_VIB_CON = 0.; Del_LHY_VIB_CON = 0.;

		Del_RLHP_Posture_Walking = 0.; Del_RLFZ_Posture_Walking = 0.; Del_RLAR_Posture_Walking = 0.;

		adj_amp = 0.;

	}

	BC_Y_CON = - Del_PC_Y_DSP_YZMP_CON - 1.*Del_PC_Y_SSP_YZMP_CON - 1.*Ref_CON_SUM_Y_ZMP_ERR;  // update
	BC_X_CON = - Del_PC_X_DSP_XZMP_CON - 1.*Del_PC_X_SSP_XZMP_CON - 1.*Ref_CON_SUM_X_ZMP_ERR;;  // update

	
    // ?? ?? ??? ???? ??//
    Rot_th_R1 = Ref_th[0] - Ref_th[6]; // [deg]
    Rot_th_L1 = Ref_th[3] - Ref_th[6]; // [deg]
   
    del_x_R1 = -_pRF0_3x1[2]*sin(D2R*Rot_th_R1); 
    del_y_R1 = _pRF0_3x1[2]*cos(D2R*Rot_th_R1) - _pRF0_3x1[2]; 
    del_x_L1 = _pLF0_3x1[2]*sin(D2R*(180. + Rot_th_L1)); 
    del_y_L1 = -_pLF0_3x1[2]*cos(D2R*(180. + Rot_th_L1)) - _pLF0_3x1[2]; 
    
    	   
    _online_pattern.dangRFz = D2R*Rot_th_R1;
    _online_pattern.dangLFz = D2R*Rot_th_L1;
	_online_pattern.dangRHz = 0.f;
	_online_pattern.dangLHz = 0.f;

	_online_pattern.dpCOMx = 0.f;
	_online_pattern.dpCOMy = 0.f;
            
   
	_online_pattern.dpRFz = 0.001*Ref_R1[2] + 0.001*Del_RLFZ_Init;
    _online_pattern.dpLFz = 0.001*Ref_L1[2] - 0.001*Del_RLFZ_Init;
    _online_pattern.dpRFy = 0.001*Ref_R1[1] - 0.001*Ref_BC[1] + del_y_R1 - 0.001*BC_Y_Init - 0.001*BC_Y_CON + 0.0*RF_Del_Y_Outside;  // update
    _online_pattern.dpLFy = 0.001*Ref_L1[1] - 0.001*Ref_BC[1] + del_y_L1 - 0.001*BC_Y_Init - 0.001*BC_Y_CON; // update
    _online_pattern.dpRFx = 0.001*Ref_R1[0] - 0.001*Ref_BC[0] + del_x_R1 - 0.001*BC_X_Init - 0.001*BC_X_CON; // update
    _online_pattern.dpLFx = 0.001*Ref_L1[0] - 0.001*Ref_BC[0] + del_x_L1 - 0.001*BC_X_Init - 0.001*BC_X_CON; // update   

	_online_pattern.offset_LAR = (Del_LAR_Init + Del_LAR_RLAR_TORQ_CON + Del_RLAR_Posture_Walking + 0.0*RHR_Compen_Ang)*D2R;
	_online_pattern.offset_LAP = (Del_LAP_RLAP_TORQ_CON + Del_RLHP_Init)*D2R;

    _online_pattern.offset_RAR = (Del_RAR_Init + Del_RAR_RLAR_TORQ_CON + Del_RLAR_Posture_Walking + 0.0*LHR_Compen_Ang)*D2R;
	_online_pattern.offset_RAP = (Del_RAP_Init + Del_RAP_RLAP_TORQ_CON + Del_RLHP_Init)*D2R;

	_online_pattern.offset_RHR = (Del_RHR_VIB_CON + RHR_Compen_Ang)*D2R;
	_online_pattern.offset_LHR = (Del_LHR_VIB_CON + LHR_Compen_Ang + RF_Del_Y_Outside)*D2R;

	_online_pattern.offset_RHP = (Del_RHP_VIB_CON + Del_RLHP_Posture_Walking)*D2R;
	_online_pattern.offset_LHP = (Del_LHP_VIB_CON + Del_RLHP_Posture_Walking)*D2R;

	_online_pattern.offset_RHY = Del_RHY_VIB_CON*D2R;
	_online_pattern.offset_LHY = Del_LHY_VIB_CON*D2R;

    //-------------------- log for Kirk
	_log_temp[0] = 0.001*Ref_BC[0];      // A
	_log_temp[1] = 0.001*Ref_BC[1];      // B
	_log_temp[2] = 0.001*Ref_R1[2];      // C1
	_log_temp[3] = 0.001*Ref_L1[2];      // C2
	_log_temp[4] = 0.001*X_ZMP;          // C3
	_log_temp[5] = 0.001*Y_ZMP;          // C4
	_log_temp[6] = 0.001*Ref_ZMP[0];     // C5
	_log_temp[7] = 0.001*Ref_ZMP[1];     // C6
	_log_temp[8] = 0.001*Del_PC_X_DSP_XZMP_CON;         // C7
	_log_temp[9] = 0.001*Del_PC_Y_DSP_YZMP_CON;         // C8 
	_log_temp[10] = 0.001*Del_PC_X_SSP_XZMP_CON;        // C9
	_log_temp[11] = 0.001*Del_PC_Y_SSP_YZMP_CON;        // C10
	_log_temp[12] = 0.001*Ref_CON_SUM_X_ZMP_ERR;            // C11
	_log_temp[13] = 0.001*Ref_CON_SUM_Y_ZMP_ERR;            // C12
	_log_temp[14] = Del_RAR_RLAR_TORQ_CON;//FTSensor[RFFT].Fz;                  // C13
	_log_temp[15] = Del_LAR_RLAR_TORQ_CON;//FTSensor[LFFT].Fz;                  // C14
	_log_temp[16] = Del_RHY_VIB_CON;//SUM_X_ZMP_ERR;            // C15
	_log_temp[17] = Del_LHY_VIB_CON;//SUM_Y_ZMP_ERR;            // C16
	_log_temp[18] = CNT_SUM_ZMP_ERR;            // C17
	_log_temp[19] = RMS_Y_ZMP;            // C18
	_log_temp[20] = RF_Del_Y_Outside; // C19
	_log_temp[21] = Del_RLHP_Posture_Walking;  // C20
	_log_temp[22] = Del_RLAR_Posture_Walking; // C21
	_log_temp[23] = R2D*Body_Y_th;//Avg_RF_Pitch_Ang ;  // C22
	_log_temp[24] = R2D*Body_Y_th_d;//Avg_LF_Pitch_Ang ;  // C23
	_log_temp[25] = IMUSensor[CENTERIMU].Roll;    // C24
	_log_temp[26] = IMUSensor[CENTERIMU].Pitch;    // C25
	_log_temp[27] = IMUSensor[CENTERIMU].Roll_Velocity; // C26
	_log_temp[28] = IMUSensor[CENTERIMU].Pitch_Velocity; // C27
	//--------------------

	//------------------- log for inhyeok
	_log_temp[29] = 0.001f*Del_RLFZ_Init;
	_log_temp[30] = 0.001f*BC_X_Init;
	_log_temp[31] = 0.001f*BC_Y_Init;
	_log_temp[32] = 0.001f*BC_X_CON;
	_log_temp[33] = 0.001f*BC_Y_CON;	
	_log_temp[34] = Del_LAR_Init*D2R;
	_log_temp[35] = Del_RAR_Init*D2R;
	_log_temp[36] = Del_RAP_Init*D2R;
	_log_temp[37] = Del_RLHP_Init*D2R;
	_log_temp[38] = Del_LAR_RLAR_TORQ_CON*D2R;
	_log_temp[39] = Del_LAP_RLAP_TORQ_CON*D2R;
	_log_temp[40] = Del_RAR_RLAR_TORQ_CON*D2R;
	_log_temp[41] = Del_RAP_RLAP_TORQ_CON*D2R;
	_log_temp[42] = Del_RLAR_Posture_Walking*D2R;
	_log_temp[43] = Del_RLHP_Posture_Walking*D2R;
	_log_temp[44] = RHR_Compen_Ang*D2R;
	_log_temp[45] = LHR_Compen_Ang*D2R;	
	_log_temp[45] = _online_pattern.offset_LAR;
	_log_temp[46] = _online_pattern.offset_LAP;
	_log_temp[47] = _online_pattern.offset_RAR;
	_log_temp[48] = _online_pattern.offset_RAP;
	_log_temp[49] = _online_pattern.offset_LHR;
	_log_temp[50] = _online_pattern.offset_LHP;
	_log_temp[51] = _online_pattern.offset_RHR;
	_log_temp[52] = _online_pattern.offset_RHP;
	_log_temp[53] = 0.001*Ref_R1[0];
	_log_temp[54] = 0.001*Ref_L1[0];
	_log_temp[55] = 0.001*Ref_R1[1];
	_log_temp[56] = 0.001*Ref_L1[1];
	_log_temp[57] = _pCOM_3x1[1];
	_log_temp[58] = _pCOM_3x1[2];
	_log_temp[59] = LandLOK_Flag;
	_log_temp[60] = LandROK_Flag;
	_log_temp[61] = ThreeKG_LOK_Flag;
	_log_temp[62] = ThreeKG_ROK_Flag;
	_log_temp[63] = OneKG_LOK_Flag;
	_log_temp[64] = OneKG_ROK_Flag;
	_log_temp[65] = Walking_stage[LF_1];
	_log_temp[66] = Walking_stage[RF_1];
	//-----------------------------

}


