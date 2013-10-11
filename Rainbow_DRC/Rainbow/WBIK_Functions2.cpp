#include <windows.h>
#include <stdio.h>
//#include <stdlib.h>
#include <math.h>
#include "WBIK_Functions.h"
#include "nrutil.h"
#include "CommonDefinition.h"
#include "SharedMemory.h"
#include "APIs.h"

extern PSHARED_DATA	pSharedMemory;
extern JOINT Joint[NO_OF_JOINT];		// Joint struct variable
extern FT FTSensor[NO_OF_FT];		// FT sensor struct variable
extern IMU IMUSensor[NO_OF_IMU];	// IMU sensor struct variable

extern bool SendRunStopCMD(unsigned char _boardID, unsigned char _runStop);
extern bool NullFTSensor(unsigned char _ftID, unsigned char _mode);


extern OFFLINE_TRAJ _offline_traj;
//-------------- For kinematics
extern float _pRF_3x1[4], _pLF_3x1[4], _qRF_4x1[5], _qLF_4x1[5], _pRH_3x1[4], _pLH_3x1[4], _qRH_4x1[5], _qLH_4x1[5], _pCOM_3x1[4];
extern float _pRS_3x1[4], _pLS_3x1[4], _qRS_4x1[5], _qLS_4x1[5];
extern float _pRWR_3x1[4], _pLWR_3x1[4], _pRANK_3x1[4], _pLANK_3x1[4], _qRWR_4x1[5], _qLWR_4x1[5];
extern float _pRF_L_3x1[4], _pLF_L_3x1[4], _qRF_L_4x1[5], _qLF_L_4x1[5], _pRH_L_3x1[4], _pLH_L_3x1[4], _qRH_L_4x1[5], _qLH_L_4x1[5], _pRS_L_3x1[4], _pLS_L_3x1[4], _qRS_L_4x1[5], _qLS_L_4x1[5], _qRWR_L_4x1[5], _qLWR_L_4x1[5]; // local coordinates
extern float _pRWR_L_3x1[4], _pLWR_L_3x1[4], _pRANK_L_3x1[4], _pLANK_L_3x1[4]; // local coordinates
extern char _FKineUpdatedFlag;
extern char _PassiveUpdatedFlag; 

extern float **_jRF_6x33, **_jLF_6x33, **_jCOM_3x33, **_jRH_6x33, **_jLH_6x33, **_jRS_6x33, **_jLS_6x33, **_jRWR_6x33, **_jLWR_6x33;
extern float **_jRF_old_6x33, **_jLF_old_6x33, **_jCOM_old_3x33, **_jRH_old_6x33, **_jLH_old_6x33, **_jRS_old_6x33, **_jLS_old_6x33, **_jRWR_old_6x33, **_jLWR_old_6x33;
extern float **_jRF_old2_6x33, **_jLF_old2_6x33, **_jCOM_old2_3x33, **_jRH_old2_6x33, **_jLH_old2_6x33, **_jRS_old2_6x33, **_jLS_old2_6x33, **_jRWR_old2_6x33, **_jLWR_old2_6x33;
extern float **_jRFp_6x33, **_jLFp_6x33, **_jCOMp_3x33, **_jRHp_6x33, **_jLHp_6x33, **_jRSp_6x33, **_jLSp_6x33, **_jRWRp_6x33, **_jLWRp_6x33;

extern float **_jT1_33x33, **_jT1inv_33x33, **_N1_33x33;
extern float **_jT2_33x33, **_jT2inv_33x33;
//---------------


//--------------------- temporary variables
extern float **_TEMP1_34x34, **_TEMP2_34x34, **_TEMP3_34x34, **_TEMP4_34x34;
extern float **_EYE_33;
//----------------------


//--------------------- Joint variables
extern float _Q_34x1[35]; // [x,y,z,qPEL[4],wst,rhy,rhr,rhp,rkn,rap,rar,lhy,lhr,lhp,lkn,lap,lar, rsp,rsr,rsy,reb,rwy,rwp, lsp,lsr,lsy,leb,lwy,lwp,rwy2, lwy2]
extern float _Qp_33x1[34]; // [x,y,z,wPEL[3],wst,rhy,rhr,rhp,rkn,rap,rar,lhy,lhr,lhp,lkn,lap,lar, rsp,rsr,rsy,reb,rwy,rwp, lsp,lsr,lsy,leb,lwy,lwp,rwyp2, lwyp2]
//----------------------


//--------------------- initial values
extern float _Q0_34x1[35];
extern float _pPEL0_3x1[4], _pCOM0_3x1[4];
extern float _pRF0_3x1[4], _pLF0_3x1[4], _qRF0_4x1[5], _qLF0_4x1[5];
extern float _pRH0_3x1[4], _pLH0_3x1[4], _qRH0_4x1[5], _qLH0_4x1[5];
//---------------------


//----------------------- for online quad pattern
extern ONLINE_PATTERN _online_pattern;
//-----------------------


//---------------------- torque control
extern float _gain_gravity[34], _gain_task[34], _friction_para[34][2];
//----------------------


extern unsigned char Walking_stage[6];
extern unsigned char LandROK_Flag, LandLOK_Flag, OneKG_ROK_Flag, OneKG_LOK_Flag;
extern double Del_RAP_Init, Del_RAR_Init, Del_LAR_Init;

extern float _log_temp[LOG_DATA_SIZE];

char land_ok_LF, land_ok_RF;
unsigned int land_ok_cnt_LF, land_ok_cnt_RF;

int WBIK_DRC_Kirk_Quad(unsigned int n)  
{
	int i,j;
	
	float Qpp_33x1[34], meas_Q_34x1[35], meas_Qp_33x1[34];
	float X1_33x1[34], Xrf_6x1[7], Xlf_6x1[7], Xrh_6x1[7], Xlh_6x1[7], Xcom_3x1[4], Xpel_3x1[4], Xwst, Xpelz, X2_33x1[34];  // task variables
	float ub_27x1[28], lb_27x1[28], qpp_limited[34];  // joint upper-bound, lower-bound
	int index_limited[28];
	
	float des_pCOM_3x1[4], des_vCOM_3x1[4];
	float des_pRF_3x1[4], des_pLF_3x1[4], des_vRF_3x1[4], des_vLF_3x1[4];
	float des_qRF_4x1[5], des_qLF_4x1[5], des_wRF_3x1[4], des_wLF_3x1[4];
	float des_pRH_3x1[4], des_pLH_3x1[4], des_vRH_3x1[4], des_vLH_3x1[4];
	float des_qRH_4x1[5], des_qLH_4x1[5], des_wRH_3x1[4], des_wLH_3x1[4];
	float des_WST, des_WSTp, des_qPEL_4x1[5];
	float des_pPELz, des_vPELz;
	float vCOM_3x1[4];

	BOOL isLimited = 0;
	int dim_primary_task, dim_redundant_task;

	float lL, lR, fRFz, fLFz;
	float rhr_compen = 0.f;
	float lhr_compen = 0.f;
	float rhy_compen = 0.f;
	float lhy_compen = 0.f;
	float fric_compen_33x1[34], ct_33x1[34], gravity_33x1[34], duty_33x1[34], duty_joint_limit_33x1[34];

	static char half_flag;
	static int count;
	static float neutral_Q_34x1[35], Q_old_34x1[35], Q_old2_34x1[35];
	static float offset_pPELz, offset_pRH_3x1[4], offset_pLH_3x1[4], offset_qRH_4x1[5], offset_qLH_4x1[5]; 
	static float pCOM1_3x1[4], pPELz1, pRF1_3x1[4], pLF1_3x1[4], qRF1_4x1[5], qLF1_4x1[5], qPEL1_4x1[5];
	static float pCOM2_3x1[4], pPELz2, pRF2_3x1[4], pLF2_3x1[4], qRF2_4x1[5], qLF2_4x1[5];
	static float pRH1_3x1[4], pLH1_3x1[4], qRH1_4x1[5], qLH1_4x1[5];
	static float pRH2_3x1[4], pLH2_3x1[4], qRH2_4x1[5], qLH2_4x1[5];
	static float qWST1;
	static float t;
	static char reset_RFFT_flag, reset_LFFT_flag, reset_RWFT_flag, reset_LWFT_flag;

	float ftemp1_7x1[8], ftemp2_7x1[8], ftemp3_7x1[8], ftemp4_7x1[8];
	float pre_gain = 0.f;
	float pre_gain_lb = 0.f;
	char update_pos1_flag = 0;

	float ref_Q_34x1[35];
	float gain_ovr_rarm, gain_ovr_larm, gain_ovr_rank, gain_ovr_lank;
	if(_FKineUpdatedFlag != 1)
		return -4;
	
	_Q_34x1[RWY2_34] = 0.f;
	_Q_34x1[LWY2_34] = 0.f;
	if(n<=1)
	{
		neutral_Q_34x1[WST_34] = Joint[WST].RefAngleCurrent*D2R;

		neutral_Q_34x1[RSP_34] = Joint[RSP].RefAngleCurrent*D2R;
		neutral_Q_34x1[RSR_34] = (Joint[RSR].RefAngleCurrent + OFFSET_RSR)*D2R;
		neutral_Q_34x1[RSY_34] = Joint[RSY].RefAngleCurrent*D2R;
		neutral_Q_34x1[REB_34] = (Joint[REB].RefAngleCurrent + OFFSET_REB)*D2R;
		neutral_Q_34x1[RWY_34] = Joint[RWY].RefAngleCurrent*D2R;
		neutral_Q_34x1[RWP_34] = Joint[RWP].RefAngleCurrent*D2R;
		//neutral_Q_34x1[RWY2_34] = Joint[RWY2].RefAngleCurrent*D2R;
		
		neutral_Q_34x1[LSP_34] = Joint[LSP].RefAngleCurrent*D2R;
		neutral_Q_34x1[LSR_34] = (Joint[LSR].RefAngleCurrent + OFFSET_LSR)*D2R;
		neutral_Q_34x1[LSY_34] = Joint[LSY].RefAngleCurrent*D2R;
		neutral_Q_34x1[LEB_34] = (Joint[LEB].RefAngleCurrent + OFFSET_LEB)*D2R;
		neutral_Q_34x1[LWY_34] = Joint[LWY].RefAngleCurrent*D2R;
		neutral_Q_34x1[LWP_34] = Joint[LWP].RefAngleCurrent*D2R;
		//neutral_Q_34x1[LWY2_34] = Joint[LWY2].RefAngleCurrent*D2R;

		neutral_Q_34x1[RAP_34] = Joint[RAP].RefAngleCurrent*D2R;
		neutral_Q_34x1[RAR_34] = Joint[RAR].RefAngleCurrent*D2R;
		neutral_Q_34x1[LAP_34] = Joint[LAP].RefAngleCurrent*D2R;
		neutral_Q_34x1[LAR_34] = Joint[LAR].RefAngleCurrent*D2R;

		qPEL1_4x1[1] = _Q_34x1[4];
		qPEL1_4x1[2] = _Q_34x1[5];
		qPEL1_4x1[3] = _Q_34x1[6];
		qPEL1_4x1[4] = _Q_34x1[7];

		_pPEL0_3x1[1] = _Q_34x1[1];
		_pPEL0_3x1[2] = _Q_34x1[2];
		pPELz1 = _pPEL0_3x1[3] = _Q_34x1[3];

		qWST1 = _Q_34x1[8];

		pCOM1_3x1[1] = _pCOM0_3x1[1] = _pCOM_3x1[1];
		pCOM1_3x1[2] = _pCOM0_3x1[2] = _pCOM_3x1[2];

		pRF1_3x1[1] = _pRF0_3x1[1] = _pRF_3x1[1];
		pRF1_3x1[2] = _pRF0_3x1[2] = _pRF_3x1[2];
		pRF1_3x1[3] = _pRF0_3x1[3] = _pRF_3x1[3];

		qRF1_4x1[1] = _qRF0_4x1[1] = _qRF_4x1[1];
		qRF1_4x1[2] = _qRF0_4x1[2] = _qRF_4x1[2];
		qRF1_4x1[3] = _qRF0_4x1[3] = _qRF_4x1[3];
		qRF1_4x1[4] = _qRF0_4x1[4] = _qRF_4x1[4];

		pLF1_3x1[1] = _pLF0_3x1[1] = _pLF_3x1[1];
		pLF1_3x1[2] = _pLF0_3x1[2] = _pLF_3x1[2];
		pLF1_3x1[3] = _pLF0_3x1[3] = _pLF_3x1[3];

		qLF1_4x1[1] = _qLF0_4x1[1] = _qLF_4x1[1];
		qLF1_4x1[2] = _qLF0_4x1[2] = _qLF_4x1[2];
		qLF1_4x1[3] = _qLF0_4x1[3] = _qLF_4x1[3];
		qLF1_4x1[4] = _qLF0_4x1[4] = _qLF_4x1[4];

		pRH1_3x1[1] = _pRH0_3x1[1] =  _pRS_3x1[1];
		pRH1_3x1[2] = _pRH0_3x1[2] =  _pRS_3x1[2];
		pRH1_3x1[3] = _pRH0_3x1[3] =  _pRS_3x1[3];

		qRH1_4x1[1] = _qRH0_4x1[1] =  _qRS_4x1[1];
		qRH1_4x1[2] = _qRH0_4x1[2] =  _qRS_4x1[2];
		qRH1_4x1[3] = _qRH0_4x1[3] =  _qRS_4x1[3];
		qRH1_4x1[4] = _qRH0_4x1[4] =  _qRS_4x1[4];

		pLH1_3x1[1] = _pLH0_3x1[1] =  _pLS_3x1[1];
		pLH1_3x1[2] = _pLH0_3x1[2] =  _pLS_3x1[2];
		pLH1_3x1[3] = _pLH0_3x1[3] =  _pLS_3x1[3];

		qLH1_4x1[1] = _qLH0_4x1[1] =  _qLS_4x1[1];
		qLH1_4x1[2] = _qLH0_4x1[2] =  _qLS_4x1[2];
		qLH1_4x1[3] = _qLH0_4x1[3] =  _qLS_4x1[3];
		qLH1_4x1[4] = _qLH0_4x1[4] =  _qLS_4x1[4];

		offset_pPELz = pPELz1 - (pRF1_3x1[3]+pLF1_3x1[3])/2.f;
		offset_pRH_3x1[1] = pRH1_3x1[1] - pRF1_3x1[1];
		offset_pRH_3x1[2] = pRH1_3x1[2] - pRF1_3x1[2];
		offset_pRH_3x1[3] = pRH1_3x1[3] - pRF1_3x1[3];
		offset_pLH_3x1[1] = pLH1_3x1[1] - pLF1_3x1[1];
		offset_pLH_3x1[2] = pLH1_3x1[2] - pLF1_3x1[2];
		offset_pLH_3x1[3] = pLH1_3x1[3] - pLF1_3x1[3];

		QT2DC(qRF1_4x1, _TEMP1_34x34);
		QT2DC(qRH1_4x1, _TEMP2_34x34);
		trans(1.f, (const float**)_TEMP1_34x34, 3,3,_TEMP3_34x34);
		mult_mm((const float**)_TEMP3_34x34,3,3, (const float**)_TEMP2_34x34,3, _TEMP1_34x34);
		DC2QT((const float**)_TEMP1_34x34, offset_qRH_4x1);

		QT2DC(qLF1_4x1, _TEMP1_34x34);
		QT2DC(qLH1_4x1, _TEMP2_34x34);
		trans(1.f, (const float**)_TEMP1_34x34, 3,3,_TEMP3_34x34);
		mult_mm((const float**)_TEMP3_34x34,3,3, (const float**)_TEMP2_34x34,3, _TEMP1_34x34);
		DC2QT((const float**)_TEMP1_34x34, offset_qLH_4x1);

		meas_Q_34x1[RSP_34] = ((float)Joint[RSP].EncoderValue)/Joint[RSP].PPR*D2R;
		meas_Q_34x1[RSR_34] = ((float)Joint[RSR].EncoderValue)/Joint[RSR].PPR*D2R + OFFSET_RSR*D2R;
		meas_Q_34x1[RSY_34] = ((float)Joint[RSY].EncoderValue)/Joint[RSY].PPR*D2R;
		meas_Q_34x1[REB_34] = ((float)Joint[REB].EncoderValue)/Joint[REB].PPR*D2R + OFFSET_REB*D2R;
		meas_Q_34x1[RWY_34] = ((float)Joint[RWY].EncoderValue)/Joint[RWY].PPR*D2R;
		meas_Q_34x1[RWP_34] = ((float)Joint[RWP].EncoderValue)/Joint[RWP].PPR*D2R;
		//meas_Q_34x1[RWY2_34] = ((float)Joint[RWY2].EncoderValue)/Joint[RWY2].PPR*D2R;

		meas_Q_34x1[LSP_34] = ((float)Joint[LSP].EncoderValue)/Joint[LSP].PPR*D2R;
		meas_Q_34x1[LSR_34] = ((float)Joint[LSR].EncoderValue)/Joint[LSR].PPR*D2R + OFFSET_LSR*D2R;
		meas_Q_34x1[LSY_34] = ((float)Joint[LSY].EncoderValue)/Joint[LSY].PPR*D2R;
		meas_Q_34x1[LEB_34] = ((float)Joint[LEB].EncoderValue)/Joint[LEB].PPR*D2R + OFFSET_LEB*D2R;
		meas_Q_34x1[LWY_34] = ((float)Joint[LWY].EncoderValue)/Joint[LWY].PPR*D2R;
		meas_Q_34x1[LWP_34] = ((float)Joint[LWP].EncoderValue)/Joint[LWP].PPR*D2R;
		//meas_Q_34x1[LWY2_34] = ((float)Joint[LWY2].EncoderValue)/Joint[LWY2].PPR*D2R;

		meas_Q_34x1[RAP_34] = ((float)Joint[RAP].EncoderValue)/Joint[RAP].PPR*D2R;
		meas_Q_34x1[RAR_34] = ((float)Joint[RAR].EncoderValue)/Joint[RAR].PPR*D2R;
		meas_Q_34x1[LAP_34] = ((float)Joint[LAP].EncoderValue)/Joint[LAP].PPR*D2R;
		meas_Q_34x1[LAR_34] = ((float)Joint[LAR].EncoderValue)/Joint[LAR].PPR*D2R;

		Q_old2_34x1[RSP_34] = Q_old_34x1[RSP_34] = meas_Q_34x1[RSP_34];
		Q_old2_34x1[RSR_34] = Q_old_34x1[RSR_34] = meas_Q_34x1[RSR_34];
		Q_old2_34x1[RSY_34] = Q_old_34x1[RSY_34] = meas_Q_34x1[RSY_34];
		Q_old2_34x1[REB_34] = Q_old_34x1[REB_34] = meas_Q_34x1[REB_34];
		Q_old2_34x1[RWY_34] = Q_old_34x1[RWY_34] = meas_Q_34x1[RWY_34];
		Q_old2_34x1[RWP_34] = Q_old_34x1[RWP_34] = meas_Q_34x1[RWP_34];
		//Q_old2_34x1[RWY2_34] = Q_old_34x1[RWY2_34] = meas_Q_34x1[RWY2_34];

		Q_old2_34x1[LSP_34] = Q_old_34x1[LSP_34] = meas_Q_34x1[LSP_34];
		Q_old2_34x1[LSR_34] = Q_old_34x1[LSR_34] = meas_Q_34x1[LSR_34];
		Q_old2_34x1[LSY_34] = Q_old_34x1[LSY_34] = meas_Q_34x1[LSY_34];
		Q_old2_34x1[LEB_34] = Q_old_34x1[LEB_34] = meas_Q_34x1[LEB_34];
		Q_old2_34x1[LWY_34] = Q_old_34x1[LWY_34] = meas_Q_34x1[LWY_34];
		Q_old2_34x1[LWP_34] = Q_old_34x1[LWP_34] = meas_Q_34x1[LWP_34];
		//Q_old2_34x1[LWY2_34] = Q_old_34x1[LWY2_34] = meas_Q_34x1[LWY2_34];

		Q_old2_34x1[RAP_34] = Q_old_34x1[RAP_34] = meas_Q_34x1[RAP_34];
		Q_old2_34x1[RAR_34] = Q_old_34x1[RAR_34] = meas_Q_34x1[RAR_34];
		Q_old2_34x1[LAP_34] = Q_old_34x1[LAP_34] = meas_Q_34x1[LAP_34];
		Q_old2_34x1[LAR_34] = Q_old_34x1[LAR_34] = meas_Q_34x1[LAR_34];

		diff_mm((const float**)_jRS_6x33,6,33, (const float**)_jRS_6x33, _jRSp_6x33);
		subs_m((const float**)_jRS_6x33,6,33, _jRS_old_6x33);
		subs_m((const float**)_jRS_6x33,6,33, _jRS_old2_6x33);
		
		diff_mm((const float**)_jLS_6x33,6,33, (const float**)_jLS_6x33, _jLSp_6x33);
		subs_m((const float**)_jLS_6x33,6,33, _jLS_old_6x33);
		subs_m((const float**)_jLS_6x33,6,33, _jLS_old2_6x33);

		diff_mm((const float**)_jRF_6x33,6,33, (const float**)_jRF_6x33, _jRFp_6x33);
		subs_m((const float**)_jRF_6x33,6,33, _jRF_old_6x33);
		subs_m((const float**)_jRF_6x33,6,33, _jRF_old2_6x33);

		diff_mm((const float**)_jLF_6x33,6,33, (const float**)_jLF_6x33, _jLFp_6x33);
		subs_m((const float**)_jLF_6x33,6,33, _jLF_old_6x33);
		subs_m((const float**)_jLF_6x33,6,33, _jLF_old2_6x33);
		
		diff_mm((const float**)_jCOM_3x33,3,33, (const float**)_jCOM_3x33, _jCOMp_3x33);
		subs_m((const float**)_jCOM_3x33,3,33, _jCOM_old_3x33);
		subs_m((const float**)_jCOM_3x33,3,33, _jCOM_old2_3x33);
		
		half_flag = 0;
		pSharedMemory->transform_flag = 0;
		t = 0.f;
		count = 0;
		return 0;
	}
	
	diff_mm((const float**)_jRF_6x33,6,33, (const float**)_jRF_old2_6x33, _jRFp_6x33);
	subs_m((const float**)_jRF_old_6x33,6,33, _jRF_old2_6x33);
	subs_m((const float**)_jRF_6x33,6,33, _jRF_old_6x33);
	mult_sm((const float**)_jRFp_6x33,6,33, 1.f/(2.f*DT), _jRFp_6x33);
	
	diff_mm((const float**)_jLF_6x33,6,33, (const float**)_jLF_old2_6x33, _jLFp_6x33);
	subs_m((const float**)_jLF_old_6x33,6,33, _jLF_old2_6x33);
	subs_m((const float**)_jLF_6x33,6,33, _jLF_old_6x33);
	mult_sm((const float**)_jLFp_6x33,6,33, 1.f/(2.f*DT), _jLFp_6x33);

	diff_mm((const float**)_jCOM_3x33,3,33, (const float**)_jCOM_old2_3x33, _jCOMp_3x33);
	subs_m((const float**)_jCOM_old_3x33,3,33, _jCOM_old2_3x33);
	subs_m((const float**)_jCOM_3x33,3,33, _jCOM_old_3x33);
	mult_sm((const float**)_jCOMp_3x33,3,33, 1.f/(2.f*DT), _jCOMp_3x33);

	gain_ovr_rarm = pSharedMemory->kp[22];
	gain_ovr_larm = pSharedMemory->kp[22];
	gain_ovr_rank = pSharedMemory->kp[25];
	gain_ovr_lank = pSharedMemory->kp[25];

	//---------------------------------------------------------------- transform to quad
	if(pSharedMemory->transform_flag == 0)
	{
		des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];
	
		des_pPELz = pPELz1 + one_cos(t, pSharedMemory->wb_data[9], pSharedMemory->wb_data[0]);

		des_WST = qWST1;
		
		des_pCOM_3x1[1] = pCOM1_3x1[1] + one_cos(t, (pRF1_3x1[1]+pLF1_3x1[1])/2.f+pSharedMemory->wb_data[12]-pCOM1_3x1[1], pSharedMemory->wb_data[0]);
		des_pCOM_3x1[2] = pCOM1_3x1[2] + one_cos(t, (pRF1_3x1[2]+pLF1_3x1[2])/2.f-pCOM1_3x1[2], pSharedMemory->wb_data[0]);

		des_pRF_3x1[1] = pRF1_3x1[1];
		des_pRF_3x1[2] = pRF1_3x1[2];
		des_pRF_3x1[3] = pRF1_3x1[3];
		
		des_pLF_3x1[1] = pLF1_3x1[1];
		des_pLF_3x1[2] = pLF1_3x1[2];
		des_pLF_3x1[3] = pLF1_3x1[3];
		
		des_qRF_4x1[1] = qRF1_4x1[1];
		des_qRF_4x1[2] = qRF1_4x1[2];
		des_qRF_4x1[3] = qRF1_4x1[3];
		des_qRF_4x1[4] = qRF1_4x1[4];

		des_qLF_4x1[1] = qLF1_4x1[1];
		des_qLF_4x1[2] = qLF1_4x1[2];
		des_qLF_4x1[3] = qLF1_4x1[3];
		des_qLF_4x1[4] = qLF1_4x1[4];

		des_pRH_3x1[1] = pRH1_3x1[1];
		des_pRH_3x1[2] = pRH1_3x1[2];
		des_pRH_3x1[3] = pRH1_3x1[3] + one_cos(t, pSharedMemory->wb_data[9], pSharedMemory->wb_data[0]);

		des_pLH_3x1[1] = pLH1_3x1[1];
		des_pLH_3x1[2] = pLH1_3x1[2];
		des_pLH_3x1[3] = pLH1_3x1[3] + one_cos(t, pSharedMemory->wb_data[9], pSharedMemory->wb_data[0]);

		des_qRH_4x1[1] = qRH1_4x1[1];
		des_qRH_4x1[2] = qRH1_4x1[2];
		des_qRH_4x1[3] = qRH1_4x1[3];
		des_qRH_4x1[4] = qRH1_4x1[4];

		des_qLH_4x1[1] = qLH1_4x1[1];
		des_qLH_4x1[2] = qLH1_4x1[2];
		des_qLH_4x1[3] = qLH1_4x1[3];
		des_qLH_4x1[4] = qLH1_4x1[4];

		t += DT;

		if(t > pSharedMemory->wb_data[0])
		{
			t = 0.f;
			update_pos1_flag = 1;
			pSharedMemory->transform_flag = 1;
			printf("\n transforming to quad... ");
		}
	}
	else if(pSharedMemory->transform_flag == 1)
	{
		
		ftemp1_7x1[0] = one_cos(t, pSharedMemory->wb_data[2]*D2R, pSharedMemory->wb_data[0]*0.75f);
		ftemp1_7x1[1] = 0;
		ftemp1_7x1[2] = 1;
		ftemp1_7x1[3] = 0;
		RV2QT(ftemp1_7x1, ftemp2_7x1);
		QTcross(qPEL1_4x1, ftemp2_7x1, des_qPEL_4x1);
		
		/*des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];*/
		
		des_pPELz = pPELz1;

		des_WST = qWST1;
		
		des_pCOM_3x1[1] = pCOM1_3x1[1];
		des_pCOM_3x1[2] = pCOM1_3x1[2];

		des_pRF_3x1[1] = pRF1_3x1[1];
		des_pRF_3x1[2] = pRF1_3x1[2];
		des_pRF_3x1[3] = pRF1_3x1[3];
		
		des_pLF_3x1[1] = pLF1_3x1[1];
		des_pLF_3x1[2] = pLF1_3x1[2];
		des_pLF_3x1[3] = pLF1_3x1[3];
		
		des_qRF_4x1[1] = qRF1_4x1[1];
		des_qRF_4x1[2] = qRF1_4x1[2];
		des_qRF_4x1[3] = qRF1_4x1[3];
		des_qRF_4x1[4] = qRF1_4x1[4];

		des_qLF_4x1[1] = qLF1_4x1[1];
		des_qLF_4x1[2] = qLF1_4x1[2];
		des_qLF_4x1[3] = qLF1_4x1[3];
		des_qLF_4x1[4] = qLF1_4x1[4];

		des_pRH_3x1[1] = pRH1_3x1[1] + one_cos(t, pRF1_3x1[1]+pSharedMemory->wb_data[5]-pRH1_3x1[1], pSharedMemory->wb_data[0]*0.9f);
		des_pRH_3x1[2] = pRH1_3x1[2] + one_cos(t, pRF1_3x1[2]+pSharedMemory->wb_data[6]-pRH1_3x1[2], pSharedMemory->wb_data[0]*0.9f);
		des_pRH_3x1[3] = pRH1_3x1[3] + one_cos(t, pRF1_3x1[3]-pRH1_3x1[3]+pSharedMemory->wb_data[10], pSharedMemory->wb_data[0]);

		des_pLH_3x1[1] = pLH1_3x1[1] + one_cos(t, pLF1_3x1[1]+pSharedMemory->wb_data[7]-pLH1_3x1[1], pSharedMemory->wb_data[0]*0.9f);
		des_pLH_3x1[2] = pLH1_3x1[2] + one_cos(t, pLF1_3x1[2]+pSharedMemory->wb_data[8]-pLH1_3x1[2], pSharedMemory->wb_data[0]*0.9f);
		des_pLH_3x1[3] = pLH1_3x1[3] + one_cos(t, pLF1_3x1[3]-pLH1_3x1[3]+pSharedMemory->wb_data[11], pSharedMemory->wb_data[0]);

		one_cos_orientation(t, qRH1_4x1, qRF1_4x1, pSharedMemory->wb_data[0]*0.9f, des_qRH_4x1);
		one_cos_orientation(t, qLH1_4x1, qLF1_4x1, pSharedMemory->wb_data[0]*0.9f, des_qLH_4x1);

		/*des_qRH_4x1[1] = qRH1_4x1[1];
		des_qRH_4x1[2] = qRH1_4x1[2];
		des_qRH_4x1[3] = qRH1_4x1[3];
		des_qRH_4x1[4] = qRH1_4x1[4];

		des_qLH_4x1[1] = qLH1_4x1[1];
		des_qLH_4x1[2] = qLH1_4x1[2];
		des_qLH_4x1[3] = qLH1_4x1[3];
		des_qLH_4x1[4] = qLH1_4x1[4];*/

		t += DT;

		if(t > pSharedMemory->wb_data[0])
		{
			t = 0.f;
			update_pos1_flag = 1;
			pSharedMemory->transform_flag = 2;
		}
	}
	else if(pSharedMemory->transform_flag == 2)
	{
		des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];
		
		des_pPELz = pPELz1 + one_cos(t, pSharedMemory->wb_data[1]-pPELz1, pSharedMemory->wb_data[0]);

		des_WST = qWST1;
		
		des_pCOM_3x1[1] = pCOM1_3x1[1] + one_cos(t, (pRF1_3x1[1]+pLF1_3x1[1])/2.f+pSharedMemory->wb_data[3]-pCOM1_3x1[1], pSharedMemory->wb_data[0]);
		des_pCOM_3x1[2] = pCOM1_3x1[2] + one_cos(t, (pRF1_3x1[2]+pLF1_3x1[2])/2.f+pSharedMemory->wb_data[4]-pCOM1_3x1[2], pSharedMemory->wb_data[0]);

		//des_pPELz =  pPELz1;
		
		des_pRF_3x1[1] = pRF1_3x1[1];
		des_pRF_3x1[2] = pRF1_3x1[2];
		des_pRF_3x1[3] = pRF1_3x1[3];
		
		des_pLF_3x1[1] = pLF1_3x1[1];
		des_pLF_3x1[2] = pLF1_3x1[2];
		des_pLF_3x1[3] = pLF1_3x1[3];
		
		des_qRF_4x1[1] = qRF1_4x1[1];
		des_qRF_4x1[2] = qRF1_4x1[2];
		des_qRF_4x1[3] = qRF1_4x1[3];
		des_qRF_4x1[4] = qRF1_4x1[4];

		des_qLF_4x1[1] = qLF1_4x1[1];
		des_qLF_4x1[2] = qLF1_4x1[2];
		des_qLF_4x1[3] = qLF1_4x1[3];
		des_qLF_4x1[4] = qLF1_4x1[4];

		des_pRH_3x1[1] = pRH1_3x1[1];
		des_pRH_3x1[2] = pRH1_3x1[2];
		des_pRH_3x1[3] = pRH1_3x1[3];

		des_pLH_3x1[1] = pLH1_3x1[1];
		des_pLH_3x1[2] = pLH1_3x1[2];
		des_pLH_3x1[3] = pLH1_3x1[3];

		des_qRH_4x1[1] = qRH1_4x1[1];
		des_qRH_4x1[2] = qRH1_4x1[2];
		des_qRH_4x1[3] = qRH1_4x1[3];
		des_qRH_4x1[4] = qRH1_4x1[4];

		des_qLH_4x1[1] = qLH1_4x1[1];
		des_qLH_4x1[2] = qLH1_4x1[2];
		des_qLH_4x1[3] = qLH1_4x1[3];
		des_qLH_4x1[4] = qLH1_4x1[4];
		
		t += DT;

		if(t > pSharedMemory->wb_data[0])
		{
			t = 0.f;
			update_pos1_flag = 1;
			switch(pSharedMemory->quad_flag)
			{
			case 1:
				pSharedMemory->transform_flag = 31;
				break;
			case 2:
				pSharedMemory->transform_flag = 33;
				break;
			default:
				pSharedMemory->transform_flag = 3;
				break;
			}
			
			//pSharedMemory->torque_mode_flag = 1;
			if(pSharedMemory->transform_flag == 3)
			{
				printf("done");

				printf("\n des_pRF : %f %f %f", des_pRF_3x1[1],des_pRF_3x1[2],des_pRF_3x1[3]);
				printf("\n des_pLF : %f %f %f", des_pLF_3x1[1],des_pLF_3x1[2],des_pLF_3x1[3]);
				printf("\n des_pRH : %f %f %f", des_pRH_3x1[1],des_pRH_3x1[2],des_pRH_3x1[3]);
				printf("\n des_pLH : %f %f %f", des_pLH_3x1[1],des_pLH_3x1[2],des_pLH_3x1[3]);
				printf("\n des_pCOM : %f %f %f", pCOM1_3x1[1], pCOM1_3x1[2], _pCOM_3x1[3]);

				printf("\n des_qPEL : %f %f %f %f", des_qPEL_4x1[1],des_qPEL_4x1[2],des_qPEL_4x1[3],des_qPEL_4x1[4]);

				printf("\n des_qRF : %f %f %f %f", des_qRF_4x1[1],des_qRF_4x1[2],des_qRF_4x1[3],des_qRF_4x1[4]);
				printf("\n des_qLF : %f %f %f %f", des_qLF_4x1[1],des_qLF_4x1[2],des_qLF_4x1[3],des_qLF_4x1[4]);
				printf("\n des_qRH : %f %f %f %f", des_qRH_4x1[1],des_qRH_4x1[2],des_qRH_4x1[3],des_qRH_4x1[4]);
				printf("\n des_qLH : %f %f %f %f\n", des_qLH_4x1[1],des_qLH_4x1[2],des_qLH_4x1[3],des_qLH_4x1[4]);
				printf("\n ---------------------------------------------\n");
			}
		}
	}
	else if(pSharedMemory->transform_flag == 31)
	{
		des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];
		
		des_pPELz = pPELz1;

		des_WST = qWST1;
		
		des_pCOM_3x1[1] = pCOM1_3x1[1];
		des_pCOM_3x1[2] = pCOM1_3x1[2] + one_cos(t, -0.08f, pSharedMemory->wb_data[0]);

		//des_pPELz =  pPELz1;
		
		des_pRF_3x1[1] = pRF1_3x1[1];
		des_pRF_3x1[2] = pRF1_3x1[2];
		des_pRF_3x1[3] = pRF1_3x1[3];
		
		des_pLF_3x1[1] = pLF1_3x1[1];
		des_pLF_3x1[2] = pLF1_3x1[2];
		des_pLF_3x1[3] = pLF1_3x1[3];
		
		des_qRF_4x1[1] = qRF1_4x1[1];
		des_qRF_4x1[2] = qRF1_4x1[2];
		des_qRF_4x1[3] = qRF1_4x1[3];
		des_qRF_4x1[4] = qRF1_4x1[4];

		des_qLF_4x1[1] = qLF1_4x1[1];
		des_qLF_4x1[2] = qLF1_4x1[2];
		des_qLF_4x1[3] = qLF1_4x1[3];
		des_qLF_4x1[4] = qLF1_4x1[4];

		des_pRH_3x1[1] = pRH1_3x1[1];
		des_pRH_3x1[2] = pRH1_3x1[2];
		des_pRH_3x1[3] = pRH1_3x1[3];

		des_pLH_3x1[1] = pLH1_3x1[1];
		des_pLH_3x1[2] = pLH1_3x1[2];
		des_pLH_3x1[3] = pLH1_3x1[3];

		des_qRH_4x1[1] = qRH1_4x1[1];
		des_qRH_4x1[2] = qRH1_4x1[2];
		des_qRH_4x1[3] = qRH1_4x1[3];
		des_qRH_4x1[4] = qRH1_4x1[4];

		des_qLH_4x1[1] = qLH1_4x1[1];
		des_qLH_4x1[2] = qLH1_4x1[2];
		des_qLH_4x1[3] = qLH1_4x1[3];
		des_qLH_4x1[4] = qLH1_4x1[4];
		
		t += DT;

		if(t > pSharedMemory->wb_data[0])
		{
			t = 0.f;
			update_pos1_flag = 1;
			pSharedMemory->transform_flag = 32;			
		}
	}
	else if(pSharedMemory->transform_flag == 32)
	{
		des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];
		
		des_pPELz = pPELz1;

		des_WST = qWST1;
		
		des_pCOM_3x1[1] = pCOM1_3x1[1];
		des_pCOM_3x1[2] = pCOM1_3x1[2];
		
		des_pRF_3x1[1] = pRF1_3x1[1];
		des_pRF_3x1[2] = pRF1_3x1[2];
		des_pRF_3x1[3] = pRF1_3x1[3];
		
		des_pLF_3x1[1] = pLF1_3x1[1];
		des_pLF_3x1[2] = pLF1_3x1[2];
		des_pLF_3x1[3] = pLF1_3x1[3];
		
		des_qRF_4x1[1] = qRF1_4x1[1];
		des_qRF_4x1[2] = qRF1_4x1[2];
		des_qRF_4x1[3] = qRF1_4x1[3];
		des_qRF_4x1[4] = qRF1_4x1[4];

		des_qLF_4x1[1] = qLF1_4x1[1];
		des_qLF_4x1[2] = qLF1_4x1[2];
		des_qLF_4x1[3] = qLF1_4x1[3];
		des_qLF_4x1[4] = qLF1_4x1[4];

		des_pRH_3x1[1] = pRH1_3x1[1];
		des_pRH_3x1[2] = pRH1_3x1[2];
		des_pRH_3x1[3] = pRH1_3x1[3];

		des_pLH_3x1[1] = pLH1_3x1[1];
		des_pLH_3x1[2] = pLH1_3x1[2];
		des_pLH_3x1[3] = pLH1_3x1[3] + one_cos(t, 0.05f, pSharedMemory->wb_data[0]);

		des_qRH_4x1[1] = qRH1_4x1[1];
		des_qRH_4x1[2] = qRH1_4x1[2];
		des_qRH_4x1[3] = qRH1_4x1[3];
		des_qRH_4x1[4] = qRH1_4x1[4];

		des_qLH_4x1[1] = qLH1_4x1[1];
		des_qLH_4x1[2] = qLH1_4x1[2];
		des_qLH_4x1[3] = qLH1_4x1[3];
		des_qLH_4x1[4] = qLH1_4x1[4];
		
		t += DT;

		if(t > pSharedMemory->wb_data[0])
		{
			t = 0.f;
			update_pos1_flag = 1;
			pSharedMemory->transform_flag = 3;			

			printf("done");

			printf("\n des_pRF : %f %f %f", des_pRF_3x1[1],des_pRF_3x1[2],des_pRF_3x1[3]);
			printf("\n des_pLF : %f %f %f", des_pLF_3x1[1],des_pLF_3x1[2],des_pLF_3x1[3]);
			printf("\n des_pRH : %f %f %f", des_pRH_3x1[1],des_pRH_3x1[2],des_pRH_3x1[3]);
			printf("\n des_pLH : %f %f %f", des_pLH_3x1[1],des_pLH_3x1[2],des_pLH_3x1[3]);
			printf("\n des_pCOM : %f %f %f", pCOM1_3x1[1], pCOM1_3x1[2], _pCOM_3x1[3]);

			printf("\n des_qPEL : %f %f %f %f", des_qPEL_4x1[1],des_qPEL_4x1[2],des_qPEL_4x1[3],des_qPEL_4x1[4]);

			printf("\n des_qRF : %f %f %f %f", des_qRF_4x1[1],des_qRF_4x1[2],des_qRF_4x1[3],des_qRF_4x1[4]);
			printf("\n des_qLF : %f %f %f %f", des_qLF_4x1[1],des_qLF_4x1[2],des_qLF_4x1[3],des_qLF_4x1[4]);
			printf("\n des_qRH : %f %f %f %f", des_qRH_4x1[1],des_qRH_4x1[2],des_qRH_4x1[3],des_qRH_4x1[4]);
			printf("\n des_qLH : %f %f %f %f\n", des_qLH_4x1[1],des_qLH_4x1[2],des_qLH_4x1[3],des_qLH_4x1[4]);
			printf("\n ---------------------------------------------\n");
		}
	}
	else if(pSharedMemory->transform_flag == 33)
	{
		des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];
		
		des_pPELz = pPELz1;

		des_WST = qWST1;
		
		des_pCOM_3x1[1] = pCOM1_3x1[1] + one_cos(t, -0.05f, pSharedMemory->wb_data[0]);
		des_pCOM_3x1[2] = pCOM1_3x1[2] + one_cos(t, -0.08f, pSharedMemory->wb_data[0]);

		des_pRF_3x1[1] = pRF1_3x1[1];
		des_pRF_3x1[2] = pRF1_3x1[2];
		des_pRF_3x1[3] = pRF1_3x1[3];
		
		des_pLF_3x1[1] = pLF1_3x1[1];
		des_pLF_3x1[2] = pLF1_3x1[2];
		des_pLF_3x1[3] = pLF1_3x1[3];
		
		des_qRF_4x1[1] = qRF1_4x1[1];
		des_qRF_4x1[2] = qRF1_4x1[2];
		des_qRF_4x1[3] = qRF1_4x1[3];
		des_qRF_4x1[4] = qRF1_4x1[4];

		des_qLF_4x1[1] = qLF1_4x1[1];
		des_qLF_4x1[2] = qLF1_4x1[2];
		des_qLF_4x1[3] = qLF1_4x1[3];
		des_qLF_4x1[4] = qLF1_4x1[4];

		des_pRH_3x1[1] = pRH1_3x1[1];
		des_pRH_3x1[2] = pRH1_3x1[2];
		des_pRH_3x1[3] = pRH1_3x1[3];

		des_pLH_3x1[1] = pLH1_3x1[1];
		des_pLH_3x1[2] = pLH1_3x1[2];
		des_pLH_3x1[3] = pLH1_3x1[3];

		des_qRH_4x1[1] = qRH1_4x1[1];
		des_qRH_4x1[2] = qRH1_4x1[2];
		des_qRH_4x1[3] = qRH1_4x1[3];
		des_qRH_4x1[4] = qRH1_4x1[4];

		des_qLH_4x1[1] = qLH1_4x1[1];
		des_qLH_4x1[2] = qLH1_4x1[2];
		des_qLH_4x1[3] = qLH1_4x1[3];
		des_qLH_4x1[4] = qLH1_4x1[4];
		
		t += DT;

		if(t > pSharedMemory->wb_data[0])
		{
			t = 0.f;
			update_pos1_flag = 1;
			pSharedMemory->transform_flag = 34;			
		}
	}
	else if(pSharedMemory->transform_flag == 34)
	{
		des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];
		
		des_pPELz = pPELz1;

		des_WST = qWST1;
		
		des_pCOM_3x1[1] = pCOM1_3x1[1];
		des_pCOM_3x1[2] = pCOM1_3x1[2];
		
		des_pRF_3x1[1] = pRF1_3x1[1];
		des_pRF_3x1[2] = pRF1_3x1[2];
		des_pRF_3x1[3] = pRF1_3x1[3];
		
		des_pLF_3x1[1] = pLF1_3x1[1];
		des_pLF_3x1[2] = pLF1_3x1[2];
		des_pLF_3x1[3] = pLF1_3x1[3] + one_cos(t, 0.05f, pSharedMemory->wb_data[0]);
		
		des_qRF_4x1[1] = qRF1_4x1[1];
		des_qRF_4x1[2] = qRF1_4x1[2];
		des_qRF_4x1[3] = qRF1_4x1[3];
		des_qRF_4x1[4] = qRF1_4x1[4];

		des_qLF_4x1[1] = qLF1_4x1[1];
		des_qLF_4x1[2] = qLF1_4x1[2];
		des_qLF_4x1[3] = qLF1_4x1[3];
		des_qLF_4x1[4] = qLF1_4x1[4];

		des_pRH_3x1[1] = pRH1_3x1[1];
		des_pRH_3x1[2] = pRH1_3x1[2];
		des_pRH_3x1[3] = pRH1_3x1[3];

		des_pLH_3x1[1] = pLH1_3x1[1];
		des_pLH_3x1[2] = pLH1_3x1[2];
		des_pLH_3x1[3] = pLH1_3x1[3];

		des_qRH_4x1[1] = qRH1_4x1[1];
		des_qRH_4x1[2] = qRH1_4x1[2];
		des_qRH_4x1[3] = qRH1_4x1[3];
		des_qRH_4x1[4] = qRH1_4x1[4];

		des_qLH_4x1[1] = qLH1_4x1[1];
		des_qLH_4x1[2] = qLH1_4x1[2];
		des_qLH_4x1[3] = qLH1_4x1[3];
		des_qLH_4x1[4] = qLH1_4x1[4];
		
		t += DT;

		if(t > pSharedMemory->wb_data[0])
		{
			t = 0.f;
			update_pos1_flag = 1;
			pSharedMemory->transform_flag = 3;			

			printf("done");

			printf("\n des_pRF : %f %f %f", des_pRF_3x1[1],des_pRF_3x1[2],des_pRF_3x1[3]);
			printf("\n des_pLF : %f %f %f", des_pLF_3x1[1],des_pLF_3x1[2],des_pLF_3x1[3]);
			printf("\n des_pRH : %f %f %f", des_pRH_3x1[1],des_pRH_3x1[2],des_pRH_3x1[3]);
			printf("\n des_pLH : %f %f %f", des_pLH_3x1[1],des_pLH_3x1[2],des_pLH_3x1[3]);
			printf("\n des_pCOM : %f %f %f", pCOM1_3x1[1], pCOM1_3x1[2], _pCOM_3x1[3]);

			printf("\n des_qPEL : %f %f %f %f", des_qPEL_4x1[1],des_qPEL_4x1[2],des_qPEL_4x1[3],des_qPEL_4x1[4]);

			printf("\n des_qRF : %f %f %f %f", des_qRF_4x1[1],des_qRF_4x1[2],des_qRF_4x1[3],des_qRF_4x1[4]);
			printf("\n des_qLF : %f %f %f %f", des_qLF_4x1[1],des_qLF_4x1[2],des_qLF_4x1[3],des_qLF_4x1[4]);
			printf("\n des_qRH : %f %f %f %f", des_qRH_4x1[1],des_qRH_4x1[2],des_qRH_4x1[3],des_qRH_4x1[4]);
			printf("\n des_qLH : %f %f %f %f\n", des_qLH_4x1[1],des_qLH_4x1[2],des_qLH_4x1[3],des_qLH_4x1[4]);
			printf("\n ---------------------------------------------\n");
		}
	}
	else if(pSharedMemory->transform_flag == 3) // staying
	{
		des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];
		
		des_pPELz =  pPELz1;

		des_WST = qWST1;
		
		des_pCOM_3x1[1] = pCOM1_3x1[1];
		des_pCOM_3x1[2] = pCOM1_3x1[2];

		des_pRF_3x1[1] = pRF1_3x1[1];
		des_pRF_3x1[2] = pRF1_3x1[2];
		des_pRF_3x1[3] = pRF1_3x1[3];
		
		des_pLF_3x1[1] = pLF1_3x1[1];
		des_pLF_3x1[2] = pLF1_3x1[2];
		des_pLF_3x1[3] = pLF1_3x1[3];
		
		des_qRF_4x1[1] = qRF1_4x1[1];
		des_qRF_4x1[2] = qRF1_4x1[2];
		des_qRF_4x1[3] = qRF1_4x1[3];
		des_qRF_4x1[4] = qRF1_4x1[4];

		des_qLF_4x1[1] = qLF1_4x1[1];
		des_qLF_4x1[2] = qLF1_4x1[2];
		des_qLF_4x1[3] = qLF1_4x1[3];
		des_qLF_4x1[4] = qLF1_4x1[4];

		des_pRH_3x1[1] = pRH1_3x1[1];
		des_pRH_3x1[2] = pRH1_3x1[2];
		des_pRH_3x1[3] = pRH1_3x1[3];
		
		des_pLH_3x1[1] = pLH1_3x1[1];
		des_pLH_3x1[2] = pLH1_3x1[2];
		des_pLH_3x1[3] = pLH1_3x1[3];
		
		des_qRH_4x1[1] = qRH1_4x1[1];
		des_qRH_4x1[2] = qRH1_4x1[2];
		des_qRH_4x1[3] = qRH1_4x1[3];
		des_qRH_4x1[4] = qRH1_4x1[4];

		des_qLH_4x1[1] = qLH1_4x1[1];
		des_qLH_4x1[2] = qLH1_4x1[2];
		des_qLH_4x1[3] = qLH1_4x1[3];
		des_qLH_4x1[4] = qLH1_4x1[4];

		_pCOM0_3x1[1] = pCOM1_3x1[1];
		_pCOM0_3x1[2] = pCOM1_3x1[2];

		_pRF0_3x1[1] = pRF1_3x1[1];
		_pRF0_3x1[2] = pRF1_3x1[2];
		_pRF0_3x1[3] = pRF1_3x1[3];

		_pLF0_3x1[1] = pLF1_3x1[1];
		_pLF0_3x1[2] = pLF1_3x1[2];
		_pLF0_3x1[3] = pLF1_3x1[3];

		_qRF0_4x1[1] = qRF1_4x1[1];
		_qRF0_4x1[2] = qRF1_4x1[2];
		_qRF0_4x1[3] = qRF1_4x1[3];
		_qRF0_4x1[4] = qRF1_4x1[4];

		_qLF0_4x1[1] = qLF1_4x1[1];
		_qLF0_4x1[2] = qLF1_4x1[2];
		_qLF0_4x1[3] = qLF1_4x1[3];
		_qLF0_4x1[4] = qLF1_4x1[4];

		_pRH0_3x1[1] = pRH1_3x1[1];
		_pRH0_3x1[2] = pRH1_3x1[2];
		_pRH0_3x1[3] = pRH1_3x1[3];

		_pLH0_3x1[1] = pLH1_3x1[1];
		_pLH0_3x1[2] = pLH1_3x1[2];
		_pLH0_3x1[3] = pLH1_3x1[3];

		_qRH0_4x1[1] = qRH1_4x1[1];
		_qRH0_4x1[2] = qRH1_4x1[2];
		_qRH0_4x1[3] = qRH1_4x1[3];
		_qRH0_4x1[4] = qRH1_4x1[4];

		_qLH0_4x1[1] = qLH1_4x1[1];
		_qLH0_4x1[2] = qLH1_4x1[2];
		_qLH0_4x1[3] = qLH1_4x1[3];
		_qLH0_4x1[4] = qLH1_4x1[4];

	}
	else if(pSharedMemory->transform_flag == 4) //walking pattern
	{
		//one_cos_orientation(0.5f, _qLF_4x1, _qRF_4x1, 1.f, ftemp1_7x1);
		//QTcross(ftemp1_7x1, qPEL1_4x1, des_qPEL_4x1);

		des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];
		
		des_pPELz = pPELz1;

		des_WST = qWST1 + _online_pattern.dqWST;
		
		des_pCOM_3x1[1] = pCOM1_3x1[1] + _online_pattern.dpCOMx;
		des_pCOM_3x1[2] = pCOM1_3x1[2] + _online_pattern.dpCOMy;
		
		des_pRF_3x1[1] = pRF1_3x1[1] + _online_pattern.dpRFx;
		des_pRF_3x1[2] = pRF1_3x1[2] + _online_pattern.dpRFy;
		des_pRF_3x1[3] = pRF1_3x1[3] + _online_pattern.dpRFz;
		
		des_pLF_3x1[1] = pLF1_3x1[1] + _online_pattern.dpLFx;
		des_pLF_3x1[2] = pLF1_3x1[2] + _online_pattern.dpLFy;
		des_pLF_3x1[3] = pLF1_3x1[3] + _online_pattern.dpLFz;
		
		ftemp1_7x1[0] = _online_pattern.dangRFz;
		ftemp1_7x1[1] = 0.f;
		ftemp1_7x1[2] = 0.f;
		ftemp1_7x1[3] = 1.f;
		RV2QT(ftemp1_7x1, ftemp2_7x1);
		QTcross(qRF1_4x1, ftemp2_7x1, des_qRF_4x1);
		
		ftemp1_7x1[0] = _online_pattern.dangLFz;
		ftemp1_7x1[1] = 0.f;
		ftemp1_7x1[2] = 0.f;
		ftemp1_7x1[3] = 1.f;
		RV2QT(ftemp1_7x1, ftemp2_7x1);
		QTcross(qLF1_4x1, ftemp2_7x1, des_qLF_4x1);

		des_pRH_3x1[1] = pRH1_3x1[1] + _online_pattern.dpRHx;
		des_pRH_3x1[2] = pRH1_3x1[2] + _online_pattern.dpRHy;
		des_pRH_3x1[3] = pRH1_3x1[3] + _online_pattern.dpRHz;

		des_pLH_3x1[1] = pLH1_3x1[1] + _online_pattern.dpLHx;
		des_pLH_3x1[2] = pLH1_3x1[2] + _online_pattern.dpLHy;
		des_pLH_3x1[3] = pLH1_3x1[3] + _online_pattern.dpLHz;

		ftemp1_7x1[1] = _online_pattern.dangRHz;
		ftemp1_7x1[2] = 0.f;
		ftemp1_7x1[3] = 0.f;
		ftemp1_7x1[4] = 1.f;
		RV2QT(ftemp1_7x1, ftemp2_7x1);
		QTcross(qRH1_4x1, ftemp2_7x1, des_qRH_4x1);

		ftemp1_7x1[1] = _online_pattern.dangLHz;
		ftemp1_7x1[2] = 0.f;
		ftemp1_7x1[3] = 0.f;
		ftemp1_7x1[4] = 1.f;
		RV2QT(ftemp1_7x1, ftemp2_7x1);
		QTcross(qLH1_4x1, ftemp2_7x1, des_qLH_4x1);

		if(Walking_stage[0] == 10)  //RF_1
			reset_RFFT_flag = 0;
		else if(Walking_stage[0] == 20 && reset_RFFT_flag ==0)
		{
			NullFTSensor(RFFT, 0x00);
			reset_RFFT_flag = 1;
		}

		if(Walking_stage[3] == 10) //LF_1
			reset_LFFT_flag = 0;
		else if(Walking_stage[3] == 20 && reset_LFFT_flag ==0)
		{
			NullFTSensor(LFFT, 0x00);
			reset_LFFT_flag = 1;
		}

		if(Walking_stage[4] == 10)  //RF_2
			reset_RWFT_flag = 0;
		else if(Walking_stage[4] == 20 && reset_RWFT_flag ==0)
		{
			NullFTSensor(RWFT, 0x00);
			reset_RWFT_flag = 1;
		}

		if(Walking_stage[1] == 10) //LF_2
			reset_LWFT_flag = 0;
		else if(Walking_stage[1] == 20 && reset_LWFT_flag ==0)
		{
			NullFTSensor(LWFT, 0x00);
			reset_LWFT_flag = 1;
		}

		/*
		_log_temp[0] = _online_pattern.dpRHx;
		_log_temp[1] = _online_pattern.dpRHy;
		_log_temp[2] = _online_pattern.dpRHz;
		_log_temp[3] = _online_pattern.dpLHx;
		_log_temp[4] = _online_pattern.dpLHy;
		_log_temp[5] = _online_pattern.dpLHz;
		_log_temp[6] = _online_pattern.dpRFx;
		_log_temp[7] = _online_pattern.dpRFy;
		_log_temp[8] = _online_pattern.dpRFz;
		_log_temp[9] = _online_pattern.dpLFx;
		_log_temp[10] = _online_pattern.dpLFy;
		_log_temp[11] = _online_pattern.dpLFz;

		printf("\n jT1\n");
		for(i=1;i<=24;i++)
		{
			for(j=1;j<=33;j++)
				printf("%f ", _jT1_33x33[i][j]);
			printf("\n");
		}
		printf("\n---------------------------");

		printf("\n dpRHx, dpRHy , dpRHz : %f %f %f", _online_pattern.dpRHx, _online_pattern.dpRHy, _online_pattern.dpRHz);
		printf("\n dpLHx, dpLHy , dpLHz : %f %f %f", _online_pattern.dpLHx, _online_pattern.dpLHy, _online_pattern.dpLHz);
		printf("\n dpRFx, dpRFy , dpRFz : %f %f %f", _online_pattern.dpRFx, _online_pattern.dpRFy, _online_pattern.dpRFz);
		printf("\n dpLFx, dpLFy , dpLFz : %f %f %f", _online_pattern.dpLFx, _online_pattern.dpLFy, _online_pattern.dpLFz);

		printf("\n des_pRF : %f %f %f", des_pRF_3x1[1],des_pRF_3x1[2],des_pRF_3x1[3]);
		printf("\n des_pLF : %f %f %f", des_pLF_3x1[1],des_pLF_3x1[2],des_pLF_3x1[3]);
		printf("\n des_pRH : %f %f %f", des_pRH_3x1[1],des_pRH_3x1[2],des_pRH_3x1[3]);
		printf("\n des_pLH : %f %f %f", des_pLH_3x1[1],des_pLH_3x1[2],des_pLH_3x1[3]);
		printf("\n des_pCOM : %f %f", des_pCOM_3x1[1],des_pCOM_3x1[2]);

		printf("\n des_qPEL : %f %f %f %f", des_qPEL_4x1[1],des_qPEL_4x1[2],des_qPEL_4x1[3],des_qPEL_4x1[4]);

		printf("\n des_qRF : %f %f %f %f", des_qRF_4x1[1],des_qRF_4x1[2],des_qRF_4x1[3],des_qRF_4x1[4]);
		printf("\n des_qLF : %f %f %f %f", des_qLF_4x1[1],des_qLF_4x1[2],des_qLF_4x1[3],des_qLF_4x1[4]);
		printf("\n des_qRH : %f %f %f %f", des_qRH_4x1[1],des_qRH_4x1[2],des_qRH_4x1[3],des_qRH_4x1[4]);
		printf("\n des_qLH : %f %f %f %f\n", des_qLH_4x1[1],des_qLH_4x1[2],des_qLH_4x1[3],des_qLH_4x1[4]);
		*/

		if(pSharedMemory->stop_online_quad_flag == 1) // stop Kirk's online pattern generator
		{
			t = 0.f;
			update_pos1_flag = 1;
			pSharedMemory->transform_flag = 3;
			printf("online pattern generator is off");
		}
	}
	else if(pSharedMemory->transform_flag == 5) // standing up sequence 1
	{
		if(pSharedMemory->torque_mode_flag == 2)
		{
			FKine_Foot(GLOBAL, _Q_34x1, pRF2_3x1, qRF2_4x1, pLF2_3x1, qLF2_4x1);
			pSharedMemory->torque_mode_flag = 3;
		}

		des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];
		
		des_pPELz = pPELz1;

		des_WST = qWST1 + one_cos(t, -qWST1, 3.f);
		
		des_pCOM_3x1[1] = pCOM1_3x1[1];
		des_pCOM_3x1[2] = pCOM1_3x1[2];
		
		des_pRF_3x1[1] = pRF1_3x1[1];
		des_pRF_3x1[2] = pRF1_3x1[2];
		des_pRF_3x1[3] = pRF1_3x1[3];
		
		des_pLF_3x1[1] = pLF1_3x1[1];
		des_pLF_3x1[2] = pLF1_3x1[2];
		des_pLF_3x1[3] = pLF1_3x1[3];
		
		one_cos_orientation(t, qRF1_4x1, qRF2_4x1, 3.f, des_qRF_4x1);
		one_cos_orientation(t, qLF1_4x1, qLF2_4x1, 3.f, des_qLF_4x1);

		des_pRH_3x1[1] = pRH1_3x1[1];
		des_pRH_3x1[2] = pRH1_3x1[2];
		des_pRH_3x1[3] = pRH1_3x1[3];

		des_pLH_3x1[1] = pLH1_3x1[1];
		des_pLH_3x1[2] = pLH1_3x1[2];
		des_pLH_3x1[3] = pLH1_3x1[3];

		des_qRH_4x1[1] = qRH1_4x1[1];
		des_qRH_4x1[2] = qRH1_4x1[2];
		des_qRH_4x1[3] = qRH1_4x1[3];
		des_qRH_4x1[4] = qRH1_4x1[4];

		des_qLH_4x1[1] = qLH1_4x1[1];
		des_qLH_4x1[2] = qLH1_4x1[2];
		des_qLH_4x1[3] = qLH1_4x1[3];
		des_qLH_4x1[4] = qLH1_4x1[4];
		
		t += DT;

		if(t > 3.f)
		{
			t = 0.f;
			update_pos1_flag = 1;
			pSharedMemory->transform_flag = 6;
			printf("\n transforming to biped... ");
		}
	}
	else if(pSharedMemory->transform_flag == 6) // standing up sequence 1
	{
		des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];
		
		des_pPELz = pPELz1 + one_cos(t, offset_pPELz+pSharedMemory->wb_data[9]-pPELz1, pSharedMemory->wb_data[0]);

		des_WST = qWST1;
		
		des_pCOM_3x1[1] = pCOM1_3x1[1] + one_cos(t, (pRF1_3x1[1]+pLF1_3x1[1])/2.f+pSharedMemory->wb_data[12]-pCOM1_3x1[1], pSharedMemory->wb_data[0]);
		des_pCOM_3x1[2] = pCOM1_3x1[2] + one_cos(t, (pRF1_3x1[2]+pLF1_3x1[2])/2.f-pCOM1_3x1[2], pSharedMemory->wb_data[0]);
		
		des_pRF_3x1[1] = pRF1_3x1[1];
		des_pRF_3x1[2] = pRF1_3x1[2];
		des_pRF_3x1[3] = pRF1_3x1[3];
		
		des_pLF_3x1[1] = pLF1_3x1[1];
		des_pLF_3x1[2] = pLF1_3x1[2];
		des_pLF_3x1[3] = pLF1_3x1[3];
		
		des_qRF_4x1[1] = qRF1_4x1[1];
		des_qRF_4x1[2] = qRF1_4x1[2];
		des_qRF_4x1[3] = qRF1_4x1[3];
		des_qRF_4x1[4] = qRF1_4x1[4];

		des_qLF_4x1[1] = qLF1_4x1[1];
		des_qLF_4x1[2] = qLF1_4x1[2];
		des_qLF_4x1[3] = qLF1_4x1[3];
		des_qLF_4x1[4] = qLF1_4x1[4];

		des_pRH_3x1[1] = pRH1_3x1[1];
		des_pRH_3x1[2] = pRH1_3x1[2];
		des_pRH_3x1[3] = pRH1_3x1[3];

		des_pLH_3x1[1] = pLH1_3x1[1];
		des_pLH_3x1[2] = pLH1_3x1[2];
		des_pLH_3x1[3] = pLH1_3x1[3];

		des_qRH_4x1[1] = qRH1_4x1[1];
		des_qRH_4x1[2] = qRH1_4x1[2];
		des_qRH_4x1[3] = qRH1_4x1[3];
		des_qRH_4x1[4] = qRH1_4x1[4];

		des_qLH_4x1[1] = qLH1_4x1[1];
		des_qLH_4x1[2] = qLH1_4x1[2];
		des_qLH_4x1[3] = qLH1_4x1[3];
		des_qLH_4x1[4] = qLH1_4x1[4];
		
		t += DT;

		if(t > pSharedMemory->wb_data[0])
		{
			t = 0.f;
			update_pos1_flag = 1;
			pSharedMemory->transform_flag = 7;
		}
	}
	else if(pSharedMemory->transform_flag == 7) // standing up sequence 2
	{
		one_cos_orientation(t-pSharedMemory->wb_data[0]*0.2f, qPEL1_4x1, qRF1_4x1, pSharedMemory->wb_data[0]*0.8f, des_qPEL_4x1);

		QTcross(qRF1_4x1, offset_qRH_4x1, ftemp1_7x1);
		one_cos_orientation(t-pSharedMemory->wb_data[0]*0.2f, qRH1_4x1, ftemp1_7x1, pSharedMemory->wb_data[0]*0.8f, des_qRH_4x1);
		QTcross(qLF1_4x1, offset_qLH_4x1, ftemp1_7x1);
		one_cos_orientation(t-pSharedMemory->wb_data[0]*0.2f, qLH1_4x1, ftemp1_7x1, pSharedMemory->wb_data[0]*0.8f, des_qLH_4x1);

		des_pPELz = pPELz1;

		des_WST = qWST1;
		
		des_pCOM_3x1[1] = pCOM1_3x1[1];
		des_pCOM_3x1[2] = pCOM1_3x1[2];
		
		des_pRF_3x1[1] = pRF1_3x1[1];
		des_pRF_3x1[2] = pRF1_3x1[2];
		des_pRF_3x1[3] = pRF1_3x1[3];
		
		des_pLF_3x1[1] = pLF1_3x1[1];
		des_pLF_3x1[2] = pLF1_3x1[2];
		des_pLF_3x1[3] = pLF1_3x1[3];
		
		des_qRF_4x1[1] = qRF1_4x1[1];
		des_qRF_4x1[2] = qRF1_4x1[2];
		des_qRF_4x1[3] = qRF1_4x1[3];
		des_qRF_4x1[4] = qRF1_4x1[4];

		des_qLF_4x1[1] = qLF1_4x1[1];
		des_qLF_4x1[2] = qLF1_4x1[2];
		des_qLF_4x1[3] = qLF1_4x1[3];
		des_qLF_4x1[4] = qLF1_4x1[4];

		des_pRH_3x1[1] = pRH1_3x1[1] + one_cos(t-pSharedMemory->wb_data[0]*0.1f, pRF1_3x1[1]+offset_pRH_3x1[1]-pRH1_3x1[1], pSharedMemory->wb_data[0]*0.8f);
		des_pRH_3x1[2] = pRH1_3x1[2] + one_cos(t-pSharedMemory->wb_data[0]*0.1f, pRF1_3x1[2]+offset_pRH_3x1[2]-pRH1_3x1[2], pSharedMemory->wb_data[0]*0.8f);
		des_pRH_3x1[3] = pRH1_3x1[3] + one_cos(t, pRF1_3x1[3]+offset_pRH_3x1[3]+pSharedMemory->wb_data[9]-pRH1_3x1[3], pSharedMemory->wb_data[0]*0.8f);

		des_pLH_3x1[1] = pLH1_3x1[1] + one_cos(t-pSharedMemory->wb_data[0]*0.1f, pLF1_3x1[1]+offset_pLH_3x1[1]-pLH1_3x1[1], pSharedMemory->wb_data[0]*0.8f);
		des_pLH_3x1[2] = pLH1_3x1[2] + one_cos(t-pSharedMemory->wb_data[0]*0.1f, pLF1_3x1[2]+offset_pLH_3x1[2]-pLH1_3x1[2], pSharedMemory->wb_data[0]*0.8f);
		des_pLH_3x1[3] = pLH1_3x1[3] + one_cos(t, pLF1_3x1[3]+offset_pLH_3x1[3]+pSharedMemory->wb_data[9]-pLH1_3x1[3], pSharedMemory->wb_data[0]*0.8f);
		
		t += DT;

		if(t > pSharedMemory->wb_data[0])
		{
			t = 0.f;
			update_pos1_flag = 1;
			pSharedMemory->transform_flag = 8;
		}
	}
	else if(pSharedMemory->transform_flag == 8) // standing up sequence 3
	{
		des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];
		
		des_pPELz = pPELz1 + one_cos(t, offset_pPELz+(pRF1_3x1[3]+pLF1_3x1[3])/2.f-pPELz1, pSharedMemory->wb_data[0]);
		
		des_WST = qWST1;

		des_pCOM_3x1[1] = pCOM1_3x1[1];
		des_pCOM_3x1[2] = pCOM1_3x1[2];
		
		des_pRF_3x1[1] = pRF1_3x1[1];
		des_pRF_3x1[2] = pRF1_3x1[2];
		des_pRF_3x1[3] = pRF1_3x1[3];
		
		des_pLF_3x1[1] = pLF1_3x1[1];
		des_pLF_3x1[2] = pLF1_3x1[2];
		des_pLF_3x1[3] = pLF1_3x1[3];
		
		des_qRF_4x1[1] = qRF1_4x1[1];
		des_qRF_4x1[2] = qRF1_4x1[2];
		des_qRF_4x1[3] = qRF1_4x1[3];
		des_qRF_4x1[4] = qRF1_4x1[4];

		des_qLF_4x1[1] = qLF1_4x1[1];
		des_qLF_4x1[2] = qLF1_4x1[2];
		des_qLF_4x1[3] = qLF1_4x1[3];
		des_qLF_4x1[4] = qLF1_4x1[4];

		des_pRH_3x1[1] = pRH1_3x1[1];
		des_pRH_3x1[2] = pRH1_3x1[2];
		des_pRH_3x1[3] = pRH1_3x1[3] + one_cos(t, offset_pPELz+(pRF1_3x1[3]+pLF1_3x1[3])/2.f-pPELz1, pSharedMemory->wb_data[0]);

		des_pLH_3x1[1] = pLH1_3x1[1];
		des_pLH_3x1[2] = pLH1_3x1[2];
		des_pLH_3x1[3] = pLH1_3x1[3] + one_cos(t, offset_pPELz+(pRF1_3x1[3]+pLF1_3x1[3])/2.f-pPELz1, pSharedMemory->wb_data[0]);

		des_qRH_4x1[1] = qRH1_4x1[1];
		des_qRH_4x1[2] = qRH1_4x1[2];
		des_qRH_4x1[3] = qRH1_4x1[3];
		des_qRH_4x1[4] = qRH1_4x1[4];

		des_qLH_4x1[1] = qLH1_4x1[1];
		des_qLH_4x1[2] = qLH1_4x1[2];
		des_qLH_4x1[3] = qLH1_4x1[3];
		des_qLH_4x1[4] = qLH1_4x1[4];
		
		t += DT;

		if(t > pSharedMemory->wb_data[0])
		{
			t = 0.f;
			update_pos1_flag = 1;
			pSharedMemory->transform_flag = 3; 
			printf("done");
		}
	}
	

	if(update_pos1_flag == 1)
	{
		qPEL1_4x1[1] = des_qPEL_4x1[1];
		qPEL1_4x1[2] = des_qPEL_4x1[2];
		qPEL1_4x1[3] = des_qPEL_4x1[3];
		qPEL1_4x1[4] = des_qPEL_4x1[4];

		pPELz1 = des_pPELz;

		qWST1 = des_WST;

		pCOM1_3x1[1] = des_pCOM_3x1[1];
		pCOM1_3x1[2] = des_pCOM_3x1[2];

		pRF1_3x1[1] = des_pRF_3x1[1];
		pRF1_3x1[2] = des_pRF_3x1[2];
		pRF1_3x1[3] = des_pRF_3x1[3];

		qRF1_4x1[1] = des_qRF_4x1[1];
		qRF1_4x1[2] = des_qRF_4x1[2];
		qRF1_4x1[3] = des_qRF_4x1[3];
		qRF1_4x1[4] = des_qRF_4x1[4];

		pLF1_3x1[1] = des_pLF_3x1[1];
		pLF1_3x1[2] = des_pLF_3x1[2];
		pLF1_3x1[3] = des_pLF_3x1[3];

		qLF1_4x1[1] = des_qLF_4x1[1];
		qLF1_4x1[2] = des_qLF_4x1[2];
		qLF1_4x1[3] = des_qLF_4x1[3];
		qLF1_4x1[4] = des_qLF_4x1[4];

		pRH1_3x1[1] = des_pRH_3x1[1];
		pRH1_3x1[2] = des_pRH_3x1[2];
		pRH1_3x1[3] = des_pRH_3x1[3];

		qRH1_4x1[1] = des_qRH_4x1[1];
		qRH1_4x1[2] = des_qRH_4x1[2];
		qRH1_4x1[3] = des_qRH_4x1[3];
		qRH1_4x1[4] = des_qRH_4x1[4];

		pLH1_3x1[1] = des_pLH_3x1[1];
		pLH1_3x1[2] = des_pLH_3x1[2];
		pLH1_3x1[3] = des_pLH_3x1[3];

		qLH1_4x1[1] = des_qLH_4x1[1];
		qLH1_4x1[2] = des_qLH_4x1[2];
		qLH1_4x1[3] = des_qLH_4x1[3];
		qLH1_4x1[4] = des_qLH_4x1[4];
	}
	
	des_WSTp = 0.f;

	des_vPELz = 0.f;

	des_pCOM_3x1[3] = pCOM1_3x1[3];
	des_vCOM_3x1[1] = 0.;
	des_vCOM_3x1[2] = 0.; 
	des_vCOM_3x1[3] = 0.;

	des_vRF_3x1[1] = 0.;
	des_vRF_3x1[2] = 0.;
	des_vRF_3x1[3] = 0.; 	
	des_wRF_3x1[1] = 0.;
	des_wRF_3x1[2] = 0.;
	des_wRF_3x1[3] = 0.;

	des_vLF_3x1[1] = 0.;
	des_vLF_3x1[2] = 0.;
	des_vLF_3x1[3] = 0.;
	des_wLF_3x1[1] = 0.;
	des_wLF_3x1[2] = 0.;
	des_wLF_3x1[3] = 0.;
	
	des_vRH_3x1[1] = 0.;
	des_vRH_3x1[2] = 0.;
	des_vRH_3x1[3] = 0.; 	
	des_wRH_3x1[1] = 0.;
	des_wRH_3x1[2] = 0.;
	des_wRH_3x1[3] = 0.;

	des_vLH_3x1[1] = 0.;
	des_vLH_3x1[2] = 0.;
	des_vLH_3x1[3] = 0.;
	des_wLH_3x1[1] = 0.;
	des_wLH_3x1[2] = 0.;
	des_wLH_3x1[3] = 0.;

	/*//------------------------------ compensation for hip roll joints
	lL = des_pLF_3x1[2] - des_pCOM_3x1[2];
	lR = des_pCOM_3x1[2] - des_pRF_3x1[2];
	fRFz = lL*m_TOTAL*9.81f/(lR+lL);
	fLFz = lR*m_TOTAL*9.81f/(lR+lL);

	if(lL < lR)
	{
		lhr_compen = (0.5f*m_TOTAL*9.81f-fRFz)/K_PELVIS_ROLL;
		rhr_compen = 0.;
	}
	else
	{
		lhr_compen = 0.;
		rhr_compen = -(0.5f*m_TOTAL*9.81f-fLFz)/K_PELVIS_ROLL;
	}
	//-------------------------------------*/

	//--------- Pelvis attitude	
	QTdel(des_qPEL_4x1, &_Q_34x1[3], ftemp3_7x1);
	Xpel_3x1[1] = pSharedMemory->kd[9]*(-_Qp_33x1[4]) + pSharedMemory->kp[9]*ftemp3_7x1[1];
	Xpel_3x1[2] = pSharedMemory->kd[9]*(-_Qp_33x1[5]) + pSharedMemory->kp[9]*ftemp3_7x1[2];
	Xpel_3x1[3] = pSharedMemory->kd[9]*(-_Qp_33x1[6]) + pSharedMemory->kp[9]*ftemp3_7x1[3];
	//-----------------	

	//------------------ Waist
	RVALS(_Q_34x1[WST_34],  _Qp_33x1[WST_33],  pSharedMemory->kd[10]*(des_WSTp-_Qp_33x1[WST_33])+pSharedMemory->kp[10]*(des_WST-_Q_34x1[WST_34]),   WSTpmax, WSTppmax, WSTmin, WSTmax, D2R, &Xwst, &ftemp1_7x1[0], &ftemp1_7x1[1]);
	//-----------------

	//--------- Pelvis height
	Xpelz = pSharedMemory->kd[11]*(des_vPELz-_Qp_33x1[3]) + pSharedMemory->kp[11]*(des_pPELz-_Q_34x1[3]);
	//-----------------
	
	//------------ Xcom				
	mult_mv((const float**)_jCOM_3x33,3,33, _Qp_33x1, vCOM_3x1);	
	ftemp2_7x1[1] = pSharedMemory->kd[12]*(des_vCOM_3x1[1]-vCOM_3x1[1]) + pSharedMemory->kp[12]*(des_pCOM_3x1[1]-_pCOM_3x1[1]);
	ftemp2_7x1[2] = pSharedMemory->kd[12]*(des_vCOM_3x1[2]-vCOM_3x1[2]) + pSharedMemory->kp[12]*(des_pCOM_3x1[2]-_pCOM_3x1[2]);
	ftemp2_7x1[3] = pSharedMemory->kd[12]*(des_vCOM_3x1[3]-vCOM_3x1[3]) + pSharedMemory->kp[12]*(des_pCOM_3x1[3]-_pCOM_3x1[3]);
	mult_mv((const float**)_jCOMp_3x33, 3,33, _Qp_33x1, ftemp1_7x1);
	Xcom_3x1[1] = ftemp2_7x1[1] - ftemp1_7x1[1];
	Xcom_3x1[2] = ftemp2_7x1[2] - ftemp1_7x1[2];
	Xcom_3x1[3] = ftemp2_7x1[3] - ftemp1_7x1[3];
	//-----------------

	//------------ Foot
	diff_vv(des_pRF_3x1,3, _pRF_3x1, ftemp3_7x1);
	mult_mv((const float**)_jRF_6x33,6,33, _Qp_33x1, ftemp2_7x1);
	diff_vv(des_vRF_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[13], ftemp3_7x1,3, pSharedMemory->kd[13],ftemp1_7x1, ftemp3_7x1);
	mult_mv((const float**)_jRFp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xrf_6x1);
	
	QTdel(des_qRF_4x1, _qRF_4x1, ftemp3_7x1);	
	diff_vv(des_wRF_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(pSharedMemory->kp[13],ftemp3_7x1,3, pSharedMemory->kd[13],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xrf_6x1[3]);
	
	diff_vv(des_pLF_3x1,3, _pLF_3x1, ftemp3_7x1);
	mult_mv((const float**)_jLF_6x33,6,33, _Qp_33x1, ftemp2_7x1);
	diff_vv(des_vLF_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[13], ftemp3_7x1,3, pSharedMemory->kd[13],ftemp1_7x1, ftemp3_7x1);
	mult_mv((const float**)_jLFp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xlf_6x1);

	QTdel(des_qLF_4x1, _qLF_4x1, ftemp3_7x1);	
	diff_vv(des_wLF_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(pSharedMemory->kp[13],ftemp3_7x1,3, pSharedMemory->kd[13],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xlf_6x1[3]);
	//-----------------

	
	//------------ Hand
	diff_vv(des_pRH_3x1,3, _pRS_3x1, ftemp3_7x1);
	mult_mv((const float**)_jRS_6x33,6,33, _Qp_33x1, ftemp2_7x1);
	diff_vv(des_vRH_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[14], ftemp3_7x1,3, pSharedMemory->kd[14],ftemp1_7x1, ftemp3_7x1);
	mult_mv((const float**)_jRSp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xrh_6x1);

	QTdel(des_qRH_4x1, _qRS_4x1, ftemp3_7x1);	
	diff_vv(des_wRH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(pSharedMemory->kp[14],ftemp3_7x1,3, pSharedMemory->kd[14],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xrh_6x1[3]);
	
	diff_vv(des_pLH_3x1,3, _pLS_3x1, ftemp3_7x1);  
	mult_mv((const float**)_jLS_6x33,6,33, _Qp_33x1, ftemp2_7x1);  
	diff_vv(des_vLH_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[14], ftemp3_7x1,3, pSharedMemory->kd[14],ftemp1_7x1, ftemp3_7x1);  
	mult_mv((const float**)_jLSp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	  
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xlh_6x1);

	QTdel(des_qLH_4x1, _qLS_4x1, ftemp3_7x1);	
	diff_vv(des_wLH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(pSharedMemory->kp[14],ftemp3_7x1,3, pSharedMemory->kd[14],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xlh_6x1[3]);
	//-----------------
	
	
	X1_33x1[1] = Xrf_6x1[1];
	X1_33x1[2] = Xrf_6x1[2];
	X1_33x1[3] = Xrf_6x1[3];
	X1_33x1[4] = Xrf_6x1[4];
	X1_33x1[5] = Xrf_6x1[5];
	X1_33x1[6] = Xrf_6x1[6];
	X1_33x1[7] = Xlf_6x1[1];
	X1_33x1[8] = Xlf_6x1[2];
	X1_33x1[9] = Xlf_6x1[3];
	X1_33x1[10] = Xlf_6x1[4];
	X1_33x1[11] = Xlf_6x1[5];
	X1_33x1[12] = Xlf_6x1[6];
	X1_33x1[13] = Xrh_6x1[1];
	X1_33x1[14] = Xrh_6x1[2];
	X1_33x1[15] = Xrh_6x1[3];
	X1_33x1[16] = Xrh_6x1[4];
	X1_33x1[17] = Xrh_6x1[5];
	X1_33x1[18] = Xrh_6x1[6];	
	X1_33x1[19] = Xlh_6x1[1];
	X1_33x1[20] = Xlh_6x1[2];
	X1_33x1[21] = Xlh_6x1[3];
	X1_33x1[22] = Xlh_6x1[4];
	X1_33x1[23] = Xlh_6x1[5];
	X1_33x1[24] = Xlh_6x1[6];
	X1_33x1[25] = Xcom_3x1[1];
	X1_33x1[26] = Xcom_3x1[2];	
	X1_33x1[27] = Xpelz;
	X1_33x1[28] = Xpel_3x1[1];
	X1_33x1[29] = Xpel_3x1[2];
	X1_33x1[30] = Xpel_3x1[3];
	X1_33x1[31] = Xwst;
	dim_primary_task = 31;
	
	//----------------- redundant tasks	
	RVALS(_Q_34x1[RSY_34],  _Qp_33x1[RSY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RSY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RSY_34]-_Q_34x1[RSY_34]),   RSYpmax, RSYppmax, RSYmin, RSYmax, D2R, &X2_33x1[1], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSY_34],  _Qp_33x1[LSY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LSY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LSY_34]-_Q_34x1[LSY_34]),   LSYpmax, LSYppmax, LSYmin, LSYmax, D2R, &X2_33x1[2], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	//RVALS(_Q_34x1[WST_34],  _Qp_33x1[WST_33],  pSharedMemory->kd[15]*(-_Qp_33x1[WST_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[WST_34]-_Q_34x1[WST_34]),   WSTpmax, WSTppmax, WSTmin, WSTmax, D2R, &X2_33x1[3], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	//X2_33x1[16] = Xpel_3x1[1];
	//X2_33x1[17] = Xpel_3x1[2];
	//X2_33x1[18] = Xpel_3x1[3];
	dim_redundant_task = 2;
	//--------------------

	for(i=1; i<=33; i++)
	{
		_jT1_33x33[1][i] = _jRF_6x33[1][i];
		_jT1_33x33[2][i] = _jRF_6x33[2][i];
		_jT1_33x33[3][i] = _jRF_6x33[3][i];
		_jT1_33x33[4][i] = _jRF_6x33[4][i];
		_jT1_33x33[5][i] = _jRF_6x33[5][i];
		_jT1_33x33[6][i] = _jRF_6x33[6][i];			

		_jT1_33x33[7][i] = _jLF_6x33[1][i];
		_jT1_33x33[8][i] = _jLF_6x33[2][i];
		_jT1_33x33[9][i] = _jLF_6x33[3][i];
		_jT1_33x33[10][i] = _jLF_6x33[4][i];
		_jT1_33x33[11][i] = _jLF_6x33[5][i];
		_jT1_33x33[12][i] = _jLF_6x33[6][i];

		_jT1_33x33[13][i] = _jRS_6x33[1][i];
		_jT1_33x33[14][i] = _jRS_6x33[2][i];
		_jT1_33x33[15][i] = _jRS_6x33[3][i];
		_jT1_33x33[16][i] = _jRS_6x33[4][i];
		_jT1_33x33[17][i] = _jRS_6x33[5][i];
		_jT1_33x33[18][i] = _jRS_6x33[6][i];

		_jT1_33x33[19][i] = _jLS_6x33[1][i];			
		_jT1_33x33[20][i] = _jLS_6x33[2][i];			
		_jT1_33x33[21][i] = _jLS_6x33[3][i];			
		_jT1_33x33[22][i] = _jLS_6x33[4][i];			
		_jT1_33x33[23][i] = _jLS_6x33[5][i];			
		_jT1_33x33[24][i] = _jLS_6x33[6][i];			

		_jT1_33x33[25][i] = _jCOM_3x33[1][i];
		_jT1_33x33[26][i] = _jCOM_3x33[2][i];

		_jT1_33x33[27][i] = 0.f;
		_jT1_33x33[28][i] = 0.f;
		_jT1_33x33[29][i] = 0.f;
		_jT1_33x33[30][i] = 0.f;   
		_jT1_33x33[31][i] = 0.f;   
		
		for(j=1;j<=dim_redundant_task;j++)
			_jT2_33x33[j][i] = 0.f;
	}
	_jT1_33x33[27][3] = 1.f;
    _jT1_33x33[28][4] = 1.f;
    _jT1_33x33[29][5] = 1.f;
    _jT1_33x33[30][6] = 1.f;
	_jT1_33x33[31][7] = 1.f;
	
	for(i=1; i<=26; i++)
	{
		_jT1_33x33[i][4] = 0.f;  // exclude pelvis orientation from the solution
		_jT1_33x33[i][5] = 0.f; 
		_jT1_33x33[i][6] = 0.f; 
	}
	
	for(i=13; i<=dim_primary_task; i++)
	{
		_jT1_33x33[i][WST_33] = 0.f;  // exclude WST from the solution
		_jT1_33x33[i][RWY2_33] = 0.f;  // exclude LWY2 from the solution
		_jT1_33x33[i][LWY2_33] = 0.f;  // exclude RWY2 from the solution
	}
	_jT1_33x33[31][7] = 1.f;

	for(j=20; j<=33; j++)		//// exclude UB from the COM solution
	{
		_jT1_33x33[25][j] = 0.f;
		_jT1_33x33[26][j] = 0.f;
	}
	_jT1_33x33[25][7] = 0.f;
	_jT1_33x33[26][7] = 0.f;

	_jT2_33x33[1][RSY_33] = 1.f;
	_jT2_33x33[2][LSY_33] = 1.f;
	//_jT2_33x33[3][WST_33] = 1.f;
		
	for(i=1; i<=33; i++)
	{
		_jT1_33x33[i][WX_33] *= WEIGHT_PEL_SQRT_INV;
		_jT1_33x33[i][WY_33] *= WEIGHT_PEL_SQRT_INV;
		_jT1_33x33[i][WZ_33] *= WEIGHT_PEL_SQRT_INV;
		_jT1_33x33[i][WST_33] *= WEIGHT_WST_SQRT_INV;

		_jT2_33x33[i][WX_33] *= WEIGHT_PEL_SQRT_INV;
		_jT2_33x33[i][WY_33] *= WEIGHT_PEL_SQRT_INV;
		_jT2_33x33[i][WZ_33] *= WEIGHT_PEL_SQRT_INV;			
		_jT2_33x33[i][WST_33] *= WEIGHT_WST_SQRT_INV;
	}
	
	if(pinv_SR((const float**)_jT1_33x33, dim_primary_task, 33, (float)1.e-10, _jT1inv_33x33) != 0)
		return -2; // singularity occurred
	mult_mm((const float**)_jT1inv_33x33,33,dim_primary_task, (const float**)_jT1_33x33, 33, _TEMP1_34x34);
	diff_mm((const float**)_EYE_33, 33,33, (const float**)_TEMP1_34x34, _N1_33x33);	

	//pinv_svd((const float**)_jT2_33x33,dim_redundant_task,33, _jT2inv_33x33);
	trans(1.f, (const float**)_jT2_33x33, dim_redundant_task,33, _jT2inv_33x33);

	//mult_mv((const float**)_jT1inv_33x33,33,dim_primary_task, (const float*)X1_33x1, Qpp_33x1);
	mult_mv((const float**)_jT1inv_33x33,33,dim_primary_task, (const float*)X1_33x1, _TEMP1_34x34[1]);
	mult_mv((const float**)_jT2inv_33x33,33,dim_redundant_task, (const float*)X2_33x1, _TEMP2_34x34[1]);
	mult_mv((const float**)_N1_33x33,33,33, (const float*)_TEMP2_34x34[1], Qpp_33x1);
	sum_vv((const float*)_TEMP1_34x34[1], 33, Qpp_33x1, Qpp_33x1);
	
	Qpp_33x1[WX_33] *= WEIGHT_PEL_SQRT_INV;
	Qpp_33x1[WY_33] *= WEIGHT_PEL_SQRT_INV;
	Qpp_33x1[WZ_33] *= WEIGHT_PEL_SQRT_INV;
	Qpp_33x1[WST_33] *= WEIGHT_WST_SQRT_INV;
	
	//------------------ check constraints
	RVALS3(_Q_34x1[WST_34], _Qp_33x1[WST_33], WSTpmax, WSTppmax, WSTmin, WSTmax, D2R, &lb_27x1[1], &ub_27x1[1]);
	RVALS3(_Q_34x1[RHY_34], _Qp_33x1[RHY_33], RHYpmax, RHYppmax, RHYmin, RHYmax, D2R, &lb_27x1[2], &ub_27x1[2]);
	RVALS3(_Q_34x1[RHR_34], _Qp_33x1[RHR_33], RHRpmax, RHRppmax, RHRmin, RHRmax, D2R, &lb_27x1[3], &ub_27x1[3]);
	RVALS3(_Q_34x1[RHP_34], _Qp_33x1[RHP_33], RHPpmax, RHPppmax, RHPmin, RHPmax, D2R, &lb_27x1[4], &ub_27x1[4]);
	RVALS3(_Q_34x1[RKN_34], _Qp_33x1[RKN_33], RKNpmax, RKNppmax, RKNmin, RKNmax, D2R, &lb_27x1[5], &ub_27x1[5]);
	RVALS3(_Q_34x1[RAP_34], _Qp_33x1[RAP_33], RAPpmax, RAPppmax, RAPmin, RAPmax, D2R, &lb_27x1[6], &ub_27x1[6]);
	RVALS3(_Q_34x1[RAR_34], _Qp_33x1[RAR_33], RARpmax, RARppmax, RARmin, RARmax, D2R, &lb_27x1[7], &ub_27x1[7]);
	RVALS3(_Q_34x1[LHY_34], _Qp_33x1[LHY_33], LHYpmax, LHYppmax, LHYmin, LHYmax, D2R, &lb_27x1[8], &ub_27x1[8]);
	RVALS3(_Q_34x1[LHR_34], _Qp_33x1[LHR_33], LHRpmax, LHRppmax, LHRmin, LHRmax, D2R, &lb_27x1[9], &ub_27x1[9]);
	RVALS3(_Q_34x1[LHP_34], _Qp_33x1[LHP_33], LHPpmax, LHPppmax, LHPmin, LHPmax, D2R, &lb_27x1[10], &ub_27x1[10]);
	RVALS3(_Q_34x1[LKN_34], _Qp_33x1[LKN_33], LKNpmax, LKNppmax, LKNmin, LKNmax, D2R, &lb_27x1[11], &ub_27x1[11]);
	RVALS3(_Q_34x1[LAP_34], _Qp_33x1[LAP_33], LAPpmax, LAPppmax, LAPmin, LAPmax, D2R, &lb_27x1[12], &ub_27x1[12]);
	RVALS3(_Q_34x1[LAR_34], _Qp_33x1[LAR_33], LARpmax, LARppmax, LARmin, LARmax, D2R, &lb_27x1[13], &ub_27x1[13]);
	RVALS3(_Q_34x1[RSP_34], _Qp_33x1[RSP_33], RSPpmax, RSPppmax, RSPmin, RSPmax, D2R, &lb_27x1[14], &ub_27x1[14]);
	RVALS3(_Q_34x1[RSR_34], _Qp_33x1[RSR_33], RSRpmax, RSRppmax, RSRmin, RSRmax, D2R, &lb_27x1[15], &ub_27x1[15]);
	RVALS3(_Q_34x1[RSY_34], _Qp_33x1[RSY_33], RSYpmax, RSYppmax, RSYmin, RSYmax, D2R, &lb_27x1[16], &ub_27x1[16]);
	RVALS3(_Q_34x1[REB_34], _Qp_33x1[REB_33], REBpmax, REBppmax, REBmin, REBmax, D2R, &lb_27x1[17], &ub_27x1[17]);
	RVALS3(_Q_34x1[RWY_34], _Qp_33x1[RWY_33], RWYpmax, RWYppmax, RWYmin, RWYmax, D2R, &lb_27x1[18], &ub_27x1[18]);
	RVALS3(_Q_34x1[RWP_34], _Qp_33x1[RWP_33], RWPpmax, RWPppmax, RWPmin, RWPmax, D2R, &lb_27x1[19], &ub_27x1[19]);	
	RVALS3(_Q_34x1[LSP_34], _Qp_33x1[LSP_33], LSPpmax, LSPppmax, LSPmin, LSPmax, D2R, &lb_27x1[20], &ub_27x1[20]);
	RVALS3(_Q_34x1[LSR_34], _Qp_33x1[LSR_33], LSRpmax, LSRppmax, LSRmin, LSRmax, D2R, &lb_27x1[21], &ub_27x1[21]);
	RVALS3(_Q_34x1[LSY_34], _Qp_33x1[LSY_33], LSYpmax, LSYppmax, LSYmin, LSYmax, D2R, &lb_27x1[22], &ub_27x1[22]);
	RVALS3(_Q_34x1[LEB_34], _Qp_33x1[LEB_33], LEBpmax, LEBppmax, LEBmin, LEBmax, D2R, &lb_27x1[23], &ub_27x1[23]);
	RVALS3(_Q_34x1[LWY_34], _Qp_33x1[LWY_33], LWYpmax, LWYppmax, LWYmin, LWYmax, D2R, &lb_27x1[24], &ub_27x1[24]);
	RVALS3(_Q_34x1[LWP_34], _Qp_33x1[LWP_33], LWPpmax, LWPppmax, LWPmin, LWPmax, D2R, &lb_27x1[25], &ub_27x1[25]);
	RVALS3(_Q_34x1[RWY2_34], _Qp_33x1[RWY2_33], RWY2pmax, RWY2ppmax, RWY2min, RWY2max, D2R, &lb_27x1[26], &ub_27x1[26]);
	RVALS3(_Q_34x1[LWY2_34], _Qp_33x1[LWY2_33], LWY2pmax, LWY2ppmax, LWY2min, LWY2max, D2R, &lb_27x1[27], &ub_27x1[27]);

	isLimited = 0;
	if(pSharedMemory->torque_mode_flag == 0 || pSharedMemory->torque_mode_flag == 3)
	{
		for(i=1;i<=27;i++)
		{
			index_limited[i] = 0;
			if(Qpp_33x1[i+6] >= ub_27x1[i])
			{
				isLimited = 1;
				index_limited[i] = 1;
			}
			else if(Qpp_33x1[i+6] <= lb_27x1[i])
			{
				isLimited = 1;
				index_limited[i] = 1;
			}

			if(isLimited==1)
			{
				printf("\n Joint limit error!!");
				printf("\n i: %d, joint: %d, p: %.4fdeg, v: %.4fdeg/s, a: %.4fdeg/s^2", _offline_traj.i, i+6, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D, Qpp_33x1[i+6]*R2D);
				printf("\n ub: %.4fdeg/s^2, lb: %.4fdeg/s^2", ub_27x1[i]*R2D, lb_27x1[i]*R2D);				
				return -2;
			}

			if(fabs(_Qp_33x1[i+6])>=QP_MAX)
			{
				printf("\n Joint velocity limit error!!");
				printf("\n i: %d, joint: %d, p: %.4fdeg, v: %.4fdeg/s, a: %.4fdeg/s^2", _offline_traj.i, i+6, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D, Qpp_33x1[i+6]*R2D);
				printf("\n ub: %.4fdeg/s^2, lb: %.4fdeg/s^2", ub_27x1[i]*R2D, lb_27x1[i]*R2D);			
				//return -2;
			}
		}
	}
	
	_Q_34x1[1] += _Qp_33x1[1]*DT + 0.5f*Qpp_33x1[1]*DT*DT;
	_Q_34x1[2] += _Qp_33x1[2]*DT + 0.5f*Qpp_33x1[2]*DT*DT;
	_Q_34x1[3] += _Qp_33x1[3]*DT + 0.5f*Qpp_33x1[3]*DT*DT;

	//------------------ update the pelvis quaternion 
	Wq(0, &_Q_34x1[3], _TEMP1_34x34);
	trans2(1.f, _TEMP1_34x34,3,4);
	mult_smv(0.5f, (const float**)_TEMP1_34x34,4,3, &Qpp_33x1[3], (float*)_TEMP2_34x34[1]); // qtpp

	Qq(0, &_Q_34x1[3], _TEMP1_34x34);
	_TEMP1_34x34[5][1] = 0.f;
	_TEMP1_34x34[5][2] = _Qp_33x1[4];
	_TEMP1_34x34[5][3] = _Qp_33x1[5];
	_TEMP1_34x34[5][4] = _Qp_33x1[6];
	mult_smv(0.5f, (const float**)_TEMP1_34x34,4,4, (const float*)_TEMP1_34x34[5], (float*)_TEMP3_34x34[1]); // qtp

	_Q_34x1[4] += _TEMP3_34x34[1][1]*DT + 0.5f*DT*DT*_TEMP2_34x34[1][1];
	_Q_34x1[5] += _TEMP3_34x34[1][2]*DT + 0.5f*DT*DT*_TEMP2_34x34[1][2];
	_Q_34x1[6] += _TEMP3_34x34[1][3]*DT + 0.5f*DT*DT*_TEMP2_34x34[1][3];
	_Q_34x1[7] += _TEMP3_34x34[1][4]*DT + 0.5f*DT*DT*_TEMP2_34x34[1][4];

	ftemp1_7x1[0] = (float)sqrt(_Q_34x1[4]*_Q_34x1[4] + _Q_34x1[5]*_Q_34x1[5] + _Q_34x1[6]*_Q_34x1[6] + _Q_34x1[7]*_Q_34x1[7]);
	_Q_34x1[4] /= ftemp1_7x1[0];
	_Q_34x1[5] /= ftemp1_7x1[0];
	_Q_34x1[6] /= ftemp1_7x1[0];
	_Q_34x1[7] /= ftemp1_7x1[0];
	//----------------------------

	_Q_34x1[WST_34] += _Qp_33x1[WST_33]*DT + 0.5f*Qpp_33x1[7]*DT*DT;
	_Q_34x1[RHY_34] += _Qp_33x1[RHY_33]*DT + 0.5f*Qpp_33x1[8]*DT*DT;
	_Q_34x1[RHR_34] += _Qp_33x1[RHR_33]*DT + 0.5f*Qpp_33x1[9]*DT*DT;
	_Q_34x1[RHP_34] += _Qp_33x1[RHP_33]*DT + 0.5f*Qpp_33x1[10]*DT*DT;
	_Q_34x1[RKN_34] += _Qp_33x1[RKN_33]*DT + 0.5f*Qpp_33x1[11]*DT*DT;
	_Q_34x1[RAP_34] += _Qp_33x1[RAP_33]*DT + 0.5f*Qpp_33x1[12]*DT*DT;
	_Q_34x1[RAR_34] += _Qp_33x1[RAR_33]*DT + 0.5f*Qpp_33x1[13]*DT*DT;
	_Q_34x1[LHY_34] += _Qp_33x1[LHY_33]*DT + 0.5f*Qpp_33x1[14]*DT*DT;
	_Q_34x1[LHR_34] += _Qp_33x1[LHR_33]*DT + 0.5f*Qpp_33x1[15]*DT*DT;
	_Q_34x1[LHP_34] += _Qp_33x1[LHP_33]*DT + 0.5f*Qpp_33x1[16]*DT*DT;
	_Q_34x1[LKN_34] += _Qp_33x1[LKN_33]*DT + 0.5f*Qpp_33x1[17]*DT*DT;
	_Q_34x1[LAP_34] += _Qp_33x1[LAP_33]*DT + 0.5f*Qpp_33x1[18]*DT*DT;
	_Q_34x1[LAR_34] += _Qp_33x1[LAR_33]*DT + 0.5f*Qpp_33x1[19]*DT*DT;	
	_Q_34x1[RSP_34] += _Qp_33x1[RSP_33]*DT + 0.5f*Qpp_33x1[20]*DT*DT;
	_Q_34x1[RSR_34] += _Qp_33x1[RSR_33]*DT + 0.5f*Qpp_33x1[21]*DT*DT;
	_Q_34x1[RSY_34] += _Qp_33x1[RSY_33]*DT + 0.5f*Qpp_33x1[22]*DT*DT;
	_Q_34x1[REB_34] += _Qp_33x1[REB_33]*DT + 0.5f*Qpp_33x1[23]*DT*DT;
	_Q_34x1[RWY_34] += _Qp_33x1[RWY_33]*DT + 0.5f*Qpp_33x1[24]*DT*DT;
	_Q_34x1[RWP_34] += _Qp_33x1[RWP_33]*DT + 0.5f*Qpp_33x1[25]*DT*DT;
	_Q_34x1[LSP_34] += _Qp_33x1[LSP_33]*DT + 0.5f*Qpp_33x1[26]*DT*DT;
	_Q_34x1[LSR_34] += _Qp_33x1[LSR_33]*DT + 0.5f*Qpp_33x1[27]*DT*DT;
	_Q_34x1[LSY_34] += _Qp_33x1[LSY_33]*DT + 0.5f*Qpp_33x1[28]*DT*DT;
	_Q_34x1[LEB_34] += _Qp_33x1[LEB_33]*DT + 0.5f*Qpp_33x1[29]*DT*DT;
	_Q_34x1[LWY_34] += _Qp_33x1[LWY_33]*DT + 0.5f*Qpp_33x1[30]*DT*DT;
	_Q_34x1[LWP_34] += _Qp_33x1[LWP_33]*DT + 0.5f*Qpp_33x1[31]*DT*DT;
	//_Q_34x1[RWY2_34] += _Qp_33x1[RWY2_33]*DT + 0.5f*Qpp_33x1[32]*DT*DT;
	//_Q_34x1[LWY2_34] += _Qp_33x1[LWY2_33]*DT + 0.5f*Qpp_33x1[33]*DT*DT;
	
	_Qp_33x1[1] += DT*Qpp_33x1[1];
	_Qp_33x1[2] += DT*Qpp_33x1[2];
	_Qp_33x1[3] += DT*Qpp_33x1[3];
	_Qp_33x1[4] += DT*Qpp_33x1[4];
	_Qp_33x1[5] += DT*Qpp_33x1[5];
	_Qp_33x1[6] += DT*Qpp_33x1[6];
	_Qp_33x1[WST_33] += DT*Qpp_33x1[7];
	_Qp_33x1[RHY_33] += DT*Qpp_33x1[8];
	_Qp_33x1[RHR_33] += DT*Qpp_33x1[9];
	_Qp_33x1[RHP_33] += DT*Qpp_33x1[10];
	_Qp_33x1[RKN_33] += DT*Qpp_33x1[11];
	_Qp_33x1[RAP_33] += DT*Qpp_33x1[12];
	_Qp_33x1[RAR_33] += DT*Qpp_33x1[13];
	_Qp_33x1[LHY_33] += DT*Qpp_33x1[14];
	_Qp_33x1[LHR_33] += DT*Qpp_33x1[15];
	_Qp_33x1[LHP_33] += DT*Qpp_33x1[16];
	_Qp_33x1[LKN_33] += DT*Qpp_33x1[17];
	_Qp_33x1[LAP_33] += DT*Qpp_33x1[18];
	_Qp_33x1[LAR_33] += DT*Qpp_33x1[19];	
	_Qp_33x1[RSP_33] += DT*Qpp_33x1[20];
	_Qp_33x1[RSR_33] += DT*Qpp_33x1[21];
	_Qp_33x1[RSY_33] += DT*Qpp_33x1[22];
	_Qp_33x1[REB_33] += DT*Qpp_33x1[23];
	_Qp_33x1[RWY_33] += DT*Qpp_33x1[24];
	_Qp_33x1[RWP_33] += DT*Qpp_33x1[25];
	_Qp_33x1[LSP_33] += DT*Qpp_33x1[26];
	_Qp_33x1[LSR_33] += DT*Qpp_33x1[27];
	_Qp_33x1[LSY_33] += DT*Qpp_33x1[28];
	_Qp_33x1[LEB_33] += DT*Qpp_33x1[29];
	_Qp_33x1[LWY_33] += DT*Qpp_33x1[30];
	_Qp_33x1[LWP_33] += DT*Qpp_33x1[31];
	//_Qp_33x1[RWY2_33] += DT*Qpp_33x1[32];
	//_Qp_33x1[LWY2_33] += DT*Qpp_33x1[33];

//	if(isLimited == 1)
//		goto LIMIT_LOOP;

	Joint[WST].RefAngleCurrent = _Q_34x1[WST_34]*R2D;

	Joint[RHY].RefAngleCurrent = (_Q_34x1[RHY_34]+rhy_compen + _online_pattern.offset_RHY)*R2D;
	Joint[RHR].RefAngleCurrent = (_Q_34x1[RHR_34]+rhr_compen +_online_pattern.offset_RHR)*R2D;
	Joint[RHP].RefAngleCurrent = (_Q_34x1[RHP_34] + _online_pattern.offset_RHP)*R2D;
	Joint[RKN].RefAngleCurrent = _Q_34x1[RKN_34]*R2D;
	Joint[RAP].RefAngleCurrent = (_Q_34x1[RAP_34] + _online_pattern.offset_RAP)*R2D;
	Joint[RAR].RefAngleCurrent = (_Q_34x1[RAR_34] + _online_pattern.offset_RAR)*R2D;
	Joint[LHY].RefAngleCurrent = (_Q_34x1[LHY_34]+lhy_compen + _online_pattern.offset_LHY)*R2D;
	Joint[LHR].RefAngleCurrent = (_Q_34x1[LHR_34]+lhr_compen+_online_pattern.offset_LHR)*R2D;
	Joint[LHP].RefAngleCurrent = (_Q_34x1[LHP_34] + _online_pattern.offset_LHP)*R2D;
	Joint[LKN].RefAngleCurrent = _Q_34x1[LKN_34]*R2D;
	Joint[LAP].RefAngleCurrent = (_Q_34x1[LAP_34] + _online_pattern.offset_LAP)*R2D;
	Joint[LAR].RefAngleCurrent = (_Q_34x1[LAR_34] + _online_pattern.offset_LAR)*R2D;

	Joint[RSP].RefAngleCurrent = _Q_34x1[RSP_34]*R2D;
	Joint[RSR].RefAngleCurrent = _Q_34x1[RSR_34]*R2D - OFFSET_RSR;
	Joint[RSY].RefAngleCurrent = _Q_34x1[RSY_34]*R2D;
	Joint[REB].RefAngleCurrent = _Q_34x1[REB_34]*R2D - OFFSET_REB;
	Joint[RWY].RefAngleCurrent = _Q_34x1[RWY_34]*R2D;
	Joint[RWP].RefAngleCurrent = _Q_34x1[RWP_34]*R2D;
	//Joint[RWY2].RefAngleCurrent = _Q_34x1[RWY2_34]*R2D;
	Joint[LSP].RefAngleCurrent = _Q_34x1[LSP_34]*R2D;
	Joint[LSR].RefAngleCurrent = _Q_34x1[LSR_34]*R2D - OFFSET_LSR;
	Joint[LSY].RefAngleCurrent = _Q_34x1[LSY_34]*R2D;
	Joint[LEB].RefAngleCurrent = _Q_34x1[LEB_34]*R2D - OFFSET_LEB;	
	Joint[LWY].RefAngleCurrent = _Q_34x1[LWY_34]*R2D;
	Joint[LWP].RefAngleCurrent = _Q_34x1[LWP_34]*R2D;
	//Joint[LWY2].RefAngleCurrent = _Q_34x1[LWY2_34]*R2D;

	for(i=1;i<=34;i++)
		ref_Q_34x1[i] = _Q_34x1[i];
	
	pSharedMemory->disp_pCOM[0] = _pCOM_3x1[1];
	pSharedMemory->disp_pCOM[1] = _pCOM_3x1[2];

	pSharedMemory->disp_pRF[0] = _pRF_3x1[1];
	pSharedMemory->disp_pRF[1] = _pRF_3x1[2];
	pSharedMemory->disp_pRF[2] = _pRF_3x1[3];

	pSharedMemory->disp_pLF[0] = _pLF_3x1[1];
	pSharedMemory->disp_pLF[1] = _pLF_3x1[2];
	pSharedMemory->disp_pLF[2] = _pLF_3x1[3];

	pSharedMemory->disp_qRF[0] = _qRF_4x1[1];
	pSharedMemory->disp_qRF[1] = _qRF_4x1[2];
	pSharedMemory->disp_qRF[2] = _qRF_4x1[3];
	pSharedMemory->disp_qRF[3] = _qRF_4x1[4];

	pSharedMemory->disp_qLF[0] = _qLF_4x1[1];
	pSharedMemory->disp_qLF[1] = _qLF_4x1[2];
	pSharedMemory->disp_qLF[2] = _qLF_4x1[3];
	pSharedMemory->disp_qLF[3] = _qLF_4x1[4];
	
	pSharedMemory->disp_pRH[0] = _pRH_3x1[1];
	pSharedMemory->disp_pRH[1] = _pRH_3x1[2];
	pSharedMemory->disp_pRH[2] = _pRH_3x1[3];

	pSharedMemory->disp_pLH[0] = _pLH_3x1[1];
	pSharedMemory->disp_pLH[1] = _pLH_3x1[2];
	pSharedMemory->disp_pLH[2] = _pLH_3x1[3];

	pSharedMemory->disp_qRH[0] = _qRH_4x1[1];
	pSharedMemory->disp_qRH[1] = _qRH_4x1[2];
	pSharedMemory->disp_qRH[2] = _qRH_4x1[3];
	pSharedMemory->disp_qRH[3] = _qRH_4x1[4];

	pSharedMemory->disp_qLH[0] = _qLH_4x1[1];
	pSharedMemory->disp_qLH[1] = _qLH_4x1[2];
	pSharedMemory->disp_qLH[2] = _qLH_4x1[3];
	pSharedMemory->disp_qLH[3] = _qLH_4x1[4];

	for(i=1;i<=34;i++)
		pSharedMemory->disp_Q_34x1[i] = _Q_34x1[i];

	for(i=1;i<=33;i++)
	{
		meas_Q_34x1[i] = _Q_34x1[i];
		meas_Qp_33x1[i] = _Qp_33x1[i];
	}
	meas_Q_34x1[34] = _Q_34x1[34];

	meas_Q_34x1[RAP_34] = ((float)Joint[RAP].EncoderValue)/Joint[RAP].PPR*D2R;
	meas_Q_34x1[RAR_34] = ((float)Joint[RAR].EncoderValue)/Joint[RAR].PPR*D2R;
	meas_Q_34x1[LAP_34] = ((float)Joint[LAP].EncoderValue)/Joint[LAP].PPR*D2R;
	meas_Q_34x1[LAR_34] = ((float)Joint[LAR].EncoderValue)/Joint[LAR].PPR*D2R;

	meas_Qp_33x1[RAP_33] = (meas_Q_34x1[RAP_34]-Q_old2_34x1[RAP_34])/(2.f*DT);
	meas_Qp_33x1[RAR_33] = (meas_Q_34x1[RAR_34]-Q_old2_34x1[RAR_34])/(2.f*DT);
	meas_Qp_33x1[LAP_33] = (meas_Q_34x1[LAP_34]-Q_old2_34x1[LAP_34])/(2.f*DT);
	meas_Qp_33x1[LAR_33] = (meas_Q_34x1[LAR_34]-Q_old2_34x1[LAR_34])/(2.f*DT);

	Q_old2_34x1[RAP_34] = Q_old_34x1[RAP_34];
	Q_old2_34x1[RAR_34] = Q_old_34x1[RAR_34];
	Q_old2_34x1[LAP_34] = Q_old_34x1[LAP_34];
	Q_old2_34x1[LAR_34] = Q_old_34x1[LAR_34];

	Q_old_34x1[RAP_34] = meas_Q_34x1[RAP_34];
	Q_old_34x1[RAR_34] = meas_Q_34x1[RAR_34];
	Q_old_34x1[LAP_34] = meas_Q_34x1[LAP_34];
	Q_old_34x1[LAR_34] = meas_Q_34x1[LAR_34];


	if(pSharedMemory->torque_mode_flag == 1 || pSharedMemory->torque_mode_flag == 2)
	{
		pre_gain = 1.f;
		for(i=1;i<=33;i++)
		{
			duty_joint_limit_33x1[i] = 0.f;
			ct_33x1[i] = 0.f;
		}
		getDuty4JointLimit(meas_Q_34x1, duty_joint_limit_33x1);
		getFricCompen(meas_Qp_33x1, fric_compen_33x1);
		getGravityTorque(meas_Q_34x1, gravity_33x1);
		
		ct_33x1[RAP_33] = gain_ovr_rank*(pSharedMemory->kd[23]*(-meas_Qp_33x1[RAP_33]) + pSharedMemory->kp[23]*(ref_Q_34x1[RAP_34]-meas_Q_34x1[RAP_34]));
		ct_33x1[RAR_33] = gain_ovr_rank*(pSharedMemory->kd[24]*(-meas_Qp_33x1[RAR_33]) + pSharedMemory->kp[24]*(ref_Q_34x1[RAR_34]-meas_Q_34x1[RAR_34]));

		ct_33x1[LAP_33] = gain_ovr_lank*(pSharedMemory->kd[23]*(-meas_Qp_33x1[LAP_33]) + pSharedMemory->kp[23]*(ref_Q_34x1[LAP_34]-meas_Q_34x1[LAP_34]));
		ct_33x1[LAR_33] = gain_ovr_lank*(pSharedMemory->kd[24]*(-meas_Qp_33x1[LAR_33]) + pSharedMemory->kp[24]*(ref_Q_34x1[LAR_34]-meas_Q_34x1[LAR_34]));

		ct_33x1[RAP_33] = _gain_task[RAP_33]*ct_33x1[RAP_33]+_gain_gravity[RAP_33]*gravity_33x1[RAP_33];
		ct_33x1[RAR_33] = _gain_task[RAR_33]*ct_33x1[RAR_33]+_gain_gravity[RAR_33]*gravity_33x1[RAR_33];

		ct_33x1[LAP_33] = _gain_task[LAP_33]*ct_33x1[LAP_33]+_gain_gravity[LAP_33]*gravity_33x1[LAP_33];
		ct_33x1[LAR_33] = _gain_task[LAR_33]*ct_33x1[LAR_33]+_gain_gravity[LAR_33]*gravity_33x1[LAR_33];
	
		pSharedMemory->disp_ct_33x1[RAP_33] = ct_33x1[RAP_33];
		pSharedMemory->disp_ct_33x1[RAR_33] = ct_33x1[RAR_33];

		pSharedMemory->disp_ct_33x1[LAP_33] = ct_33x1[LAP_33];
		pSharedMemory->disp_ct_33x1[LAR_33] = ct_33x1[LAR_33];
	
		pSharedMemory->disp_duty_33x1[RAP_33] = duty_33x1[RAP_33] = limitDuty(0.4f, torque2duty(RAP, pre_gain*ct_33x1[RAP_33]) + pre_gain*fric_compen_33x1[RAP_33] + duty_joint_limit_33x1[RAP_33]);
		pSharedMemory->disp_duty_33x1[RAR_33] = duty_33x1[RAR_33] = limitDuty(0.4f, torque2duty(RAR, pre_gain*ct_33x1[RAR_33]) + pre_gain*fric_compen_33x1[RAR_33] + duty_joint_limit_33x1[RAR_33]);

		pSharedMemory->disp_duty_33x1[LAP_33] = duty_33x1[LAP_33] = limitDuty(0.4f, torque2duty(LAP, pre_gain*ct_33x1[LAP_33]) + pre_gain*fric_compen_33x1[LAP_33] + duty_joint_limit_33x1[LAP_33]);
		pSharedMemory->disp_duty_33x1[LAR_33] = duty_33x1[LAR_33] = limitDuty(0.4f, torque2duty(LAR, pre_gain*ct_33x1[LAR_33]) + pre_gain*fric_compen_33x1[LAR_33] + duty_joint_limit_33x1[LAR_33]);
		
		RBpwmCommandHR2ch(CAN0, Joint[RAP].JMC, (int)(PWM_SIGN_RAP*1000.f*duty_33x1[RAP_33]), (int)(PWM_SIGN_RAR*1000.f*duty_33x1[RAR_33]), 1);
		RBpwmCommandHR2ch(CAN0, Joint[LAP].JMC, (int)(PWM_SIGN_LAP*1000.f*duty_33x1[LAP_33]), (int)(PWM_SIGN_LAR*1000.f*duty_33x1[LAR_33]), 1);
	}
	
	//----------------------------------------------------------------- 100Hz
	if(half_flag != 0)
	{
		half_flag = 0;
		return 0;
	}
	else
		half_flag = 1;
	
	diff_mm((const float**)_jRS_6x33,6,33, (const float**)_jRS_old2_6x33, _jRSp_6x33);
	subs_m((const float**)_jRS_old_6x33,6,33, _jRS_old2_6x33);
	subs_m((const float**)_jRS_6x33,6,33, _jRS_old_6x33);
	mult_sm((const float**)_jRSp_6x33,6,33, 1.f/(2.f*2.f*DT), _jRSp_6x33);
	
	diff_mm((const float**)_jLS_6x33,6,33, (const float**)_jLS_old2_6x33, _jLSp_6x33);
	subs_m((const float**)_jLS_old_6x33,6,33, _jLS_old2_6x33);
	subs_m((const float**)_jLS_6x33,6,33, _jLS_old_6x33);
	mult_sm((const float**)_jLSp_6x33,6,33, 1.f/(2.f*2.f*DT), _jLSp_6x33);

	meas_Q_34x1[RSP_34] = ((float)Joint[RSP].EncoderValue)/Joint[RSP].PPR*D2R;
	meas_Q_34x1[RSR_34] = ((float)Joint[RSR].EncoderValue)/Joint[RSR].PPR*D2R + OFFSET_RSR*D2R;
	meas_Q_34x1[RSY_34] = ((float)Joint[RSY].EncoderValue)/Joint[RSY].PPR*D2R;
	meas_Q_34x1[REB_34] = ((float)Joint[REB].EncoderValue)/Joint[REB].PPR*D2R + OFFSET_REB*D2R;
	meas_Q_34x1[RWY_34] = ((float)Joint[RWY].EncoderValue)/Joint[RWY].PPR*D2R;
	meas_Q_34x1[RWP_34] = ((float)Joint[RWP].EncoderValue)/Joint[RWP].PPR*D2R;
	//meas_Q_34x1[RWY2_34] = ((float)Joint[RWY2].EncoderValue)/Joint[RWY2].PPR*D2R;

	meas_Q_34x1[LSP_34] = ((float)Joint[LSP].EncoderValue)/Joint[LSP].PPR*D2R;
	meas_Q_34x1[LSR_34] = ((float)Joint[LSR].EncoderValue)/Joint[LSR].PPR*D2R + OFFSET_LSR*D2R;
	meas_Q_34x1[LSY_34] = ((float)Joint[LSY].EncoderValue)/Joint[LSY].PPR*D2R;
	meas_Q_34x1[LEB_34] = ((float)Joint[LEB].EncoderValue)/Joint[LEB].PPR*D2R + OFFSET_LEB*D2R;
	meas_Q_34x1[LWY_34] = ((float)Joint[LWY].EncoderValue)/Joint[LWY].PPR*D2R;
	meas_Q_34x1[LWP_34] = ((float)Joint[LWP].EncoderValue)/Joint[LWP].PPR*D2R;
	//meas_Q_34x1[LWY2_34] = ((float)Joint[LWY2].EncoderValue)/Joint[LWY2].PPR*D2R;

	meas_Qp_33x1[RSP_33] = (meas_Q_34x1[RSP_34]-Q_old2_34x1[RSP_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RSR_33] = (meas_Q_34x1[RSR_34]-Q_old2_34x1[RSR_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RSY_33] = (meas_Q_34x1[RSY_34]-Q_old2_34x1[RSY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[REB_33] = (meas_Q_34x1[REB_34]-Q_old2_34x1[REB_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RWY_33] = (meas_Q_34x1[RWY_34]-Q_old2_34x1[RWY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RWP_33] = (meas_Q_34x1[RWP_34]-Q_old2_34x1[RWP_34])/(2.f*2.f*DT);
	//meas_Qp_33x1[RWY2_33] = (meas_Q_34x1[RWY2_34]-Q_old2_34x1[RWY2_34])/(2.f*2.f*DT);

	meas_Qp_33x1[LSP_33] = (meas_Q_34x1[LSP_34]-Q_old2_34x1[LSP_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LSR_33] = (meas_Q_34x1[LSR_34]-Q_old2_34x1[LSR_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LSY_33] = (meas_Q_34x1[LSY_34]-Q_old2_34x1[LSY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LEB_33] = (meas_Q_34x1[LEB_34]-Q_old2_34x1[LEB_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LWY_33] = (meas_Q_34x1[LWY_34]-Q_old2_34x1[LWY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LWP_33] = (meas_Q_34x1[LWP_34]-Q_old2_34x1[LWP_34])/(2.f*2.f*DT);
	//meas_Qp_33x1[LWY2_33] = (meas_Q_34x1[LWY2_34]-Q_old2_34x1[LWY2_34])/(2.f*2.f*DT);

	//------------------------ joint limits
	for(i=20;i<=33;i++) // arms only
		if(fabs(meas_Qp_33x1[i])> QP_MAX)
		{
			RBpwmCommandHR2ch(CAN1, Joint[LSP].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[LSY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[LWY].JMC, 0, 0, 0);			
			//RBpwmCommandHR2ch(CAN1, Joint[LWY2].JMC, 0, 0, 0);

			RBpwmCommandHR2ch(CAN1, Joint[RSP].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[RSY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[RWY].JMC, 0, 0, 0);
			//RBpwmCommandHR2ch(CAN1, Joint[RWY2].JMC, 0, 0, 0);

			RBpwmCommandHR2ch(CAN0, Joint[RAP].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN0, Joint[RAR].JMC, 0, 0, 0);

			printf("\n velocity limit error! - n: %d, joint: %d, p: %.4fdeg, p_old: %.4fdeg, p_old2: %.4fdeg, v: %.4frad/s", n, i, meas_Q_34x1[i+1]*R2D, Q_old_34x1[i+1]*R2D, Q_old2_34x1[i+1]*R2D, meas_Qp_33x1[i]);
			printf("\n %f",_Q_34x1[i+1]*R2D);
			return -2;
		}
	//-------------------------------

	Q_old2_34x1[RSP_34] = Q_old_34x1[RSP_34];
	Q_old2_34x1[RSR_34] = Q_old_34x1[RSR_34];
	Q_old2_34x1[RSY_34] = Q_old_34x1[RSY_34];
	Q_old2_34x1[REB_34] = Q_old_34x1[REB_34];
	Q_old2_34x1[RWY_34] = Q_old_34x1[RWY_34];
	Q_old2_34x1[RWP_34] = Q_old_34x1[RWP_34];
	//Q_old2_34x1[RWY2_34] = Q_old_34x1[RWY2_34];

	Q_old2_34x1[LSP_34] = Q_old_34x1[LSP_34];
	Q_old2_34x1[LSR_34] = Q_old_34x1[LSR_34];
	Q_old2_34x1[LSY_34] = Q_old_34x1[LSY_34];
	Q_old2_34x1[LEB_34] = Q_old_34x1[LEB_34];
	Q_old2_34x1[LWY_34] = Q_old_34x1[LWY_34];
	Q_old2_34x1[LWP_34] = Q_old_34x1[LWP_34];
	//Q_old2_34x1[LWY2_34] = Q_old_34x1[LWY2_34];

	Q_old_34x1[RSP_34] = meas_Q_34x1[RSP_34];
	Q_old_34x1[RSR_34] = meas_Q_34x1[RSR_34];
	Q_old_34x1[RSY_34] = meas_Q_34x1[RSY_34];
	Q_old_34x1[REB_34] = meas_Q_34x1[REB_34];
	Q_old_34x1[RWY_34] = meas_Q_34x1[RWY_34];
	Q_old_34x1[RWP_34] = meas_Q_34x1[RWP_34];
	//Q_old_34x1[RWY2_34] = meas_Q_34x1[RWY2_34];

	Q_old_34x1[LSP_34] = meas_Q_34x1[LSP_34];
	Q_old_34x1[LSR_34] = meas_Q_34x1[LSR_34];
	Q_old_34x1[LSY_34] = meas_Q_34x1[LSY_34];
	Q_old_34x1[LEB_34] = meas_Q_34x1[LEB_34];
	Q_old_34x1[LWY_34] = meas_Q_34x1[LWY_34];
	Q_old_34x1[LWP_34] = meas_Q_34x1[LWP_34];
	//Q_old_34x1[LWY2_34] = meas_Q_34x1[LWY2_34];

	if(pSharedMemory->torque_mode_flag == 0)
	{
		for(i=1;i<=3;i++)
		{
			pRH2_3x1[i] = _pRS_3x1[i];
			pLH2_3x1[i] = _pLS_3x1[i];
			qRH2_4x1[i] = _qRS_4x1[i];
			qLH2_4x1[i] = _qLS_4x1[i];
		}
		qRH2_4x1[4] = _qRS_4x1[4];
		qLH2_4x1[4] = _qLS_4x1[4];

		pre_gain = 0.f;
		count = 0;
	}
	else if(pSharedMemory->torque_mode_flag == 1 || pSharedMemory->torque_mode_flag == 2)
	{
		if(pSharedMemory->torque_mode_flag == 1)
		{
			/*RBgainOverrideHR(Joint[RSP].CAN_channel, Joint[RSP].JMC, 10, 100, 100);
			RBgainOverrideHR(Joint[RSY].CAN_channel, Joint[RSY].JMC, 100, 10, 100);
			RBgainOverrideHR(Joint[RWY].CAN_channel, Joint[RWY].JMC, 100, 10, 100);

			RBgainOverrideHR(Joint[LSP].CAN_channel, Joint[LSP].JMC, 10, 100, 100);
			RBgainOverrideHR(Joint[LSY].CAN_channel, Joint[LSY].JMC, 100, 10, 100);
			RBgainOverrideHR(Joint[LWY].CAN_channel, Joint[LWY].JMC, 100, 10, 100);

			RBgainOverrideHR(Joint[RAP].CAN_channel, Joint[RAP].JMC, 10, 10, 100);
			RBgainOverrideHR(Joint[LAP].CAN_channel, Joint[LAP].JMC, 10, 10, 100);
			*/

			pre_gain = (float)count/20.f;
			count++;
			if(count>20)
			{
				pSharedMemory->torque_mode_flag = 2;
				printf("\n torque mode");
			}
		}
		else
		{
			pre_gain = 1.f;
			count = 0;
		}
	}
	else if(pSharedMemory->torque_mode_flag == 3)
	{
		
		_Q_34x1[RSP_34] = meas_Q_34x1[RSP_34];
		_Q_34x1[RSR_34] = meas_Q_34x1[RSR_34];
		_Q_34x1[RSY_34] = meas_Q_34x1[RSY_34];
		_Q_34x1[REB_34] = meas_Q_34x1[REB_34];
		//_Q_34x1[RWY_34] = meas_Q_34x1[RWY_34];
		//_Q_34x1[RWP_34] = meas_Q_34x1[RWP_34];
		//_Q_34x1[RWY2_34] = meas_Q_34x1[RWY2_34];

		_Q_34x1[LSP_34] = meas_Q_34x1[LSP_34];
		_Q_34x1[LSR_34] = meas_Q_34x1[LSR_34];
		_Q_34x1[LSY_34] = meas_Q_34x1[LSY_34];
		_Q_34x1[LEB_34] = meas_Q_34x1[LEB_34];
		//_Q_34x1[LWY_34] = meas_Q_34x1[LWY_34];
		//_Q_34x1[LWP_34] = meas_Q_34x1[LWP_34];
		//_Q_34x1[LWY2_34] = meas_Q_34x1[LWY2_34];

		_Q_34x1[RAP_34] = meas_Q_34x1[RAP_34];
		_Q_34x1[RAR_34] = meas_Q_34x1[RAR_34];
		_Q_34x1[LAP_34] = meas_Q_34x1[LAP_34];
		_Q_34x1[LAR_34] = meas_Q_34x1[LAR_34];

		FKine_Wrist(GLOBAL, _Q_34x1, pRH1_3x1, qRH1_4x1, pLH1_3x1, qLH1_4x1);
		FKine_Foot(GLOBAL, _Q_34x1, pRF1_3x1, qRF1_4x1, pLF1_3x1, qLF1_4x1);

		//_Qp_33x1[RSP_33] = meas_Qp_33x1[RSP_33];
		//_Qp_33x1[RSR_33] = meas_Qp_33x1[RSR_33];
		//_Qp_33x1[RSY_33] = meas_Qp_33x1[RSY_33];
		//_Qp_33x1[REB_33] = meas_Qp_33x1[REB_33];
		//_Qp_33x1[RWY_33] = meas_Qp_33x1[RWY_33];
		//_Qp_33x1[RWP_33] = meas_Qp_33x1[RWP_33];
		//_Qp_33x1[RWY2_33] = meas_Qp_33x1[RWY2_33];

		//_Qp_33x1[LSP_33] = meas_Qp_33x1[LSP_33];
		//_Qp_33x1[LSR_33] = meas_Qp_33x1[LSR_33];
		//_Qp_33x1[LSY_33] = meas_Qp_33x1[LSY_33];
		//_Qp_33x1[LEB_33] = meas_Qp_33x1[LEB_33];
		//_Qp_33x1[LWY_33] = meas_Qp_33x1[LWY_33];
		//_Qp_33x1[LWP_33] = meas_Qp_33x1[LWP_33];
		//_Qp_33x1[LWY2_33] = meas_Qp_33x1[LWY2_33];

		Joint[RSP].RefAngleCurrent = meas_Q_34x1[RSP_34]*R2D;
		Joint[RSR].RefAngleCurrent = meas_Q_34x1[RSR_34]*R2D - OFFSET_RSR;
		Joint[RSY].RefAngleCurrent = meas_Q_34x1[RSY_34]*R2D;
		Joint[REB].RefAngleCurrent = meas_Q_34x1[REB_34]*R2D - OFFSET_REB;
		Joint[RWY].RefAngleCurrent = meas_Q_34x1[RWY_34]*R2D;
		Joint[RWP].RefAngleCurrent = meas_Q_34x1[RWP_34]*R2D;
		//Joint[RWY2].RefAngleCurrent = _Q_34x1[RWY2_34]*R2D;
		Joint[LSP].RefAngleCurrent = meas_Q_34x1[LSP_34]*R2D;
		Joint[LSR].RefAngleCurrent = meas_Q_34x1[LSR_34]*R2D - OFFSET_LSR;
		Joint[LSY].RefAngleCurrent = meas_Q_34x1[LSY_34]*R2D;
		Joint[LEB].RefAngleCurrent = meas_Q_34x1[LEB_34]*R2D - OFFSET_LEB;	
		Joint[LWY].RefAngleCurrent = meas_Q_34x1[LWY_34]*R2D;
		Joint[LWP].RefAngleCurrent = meas_Q_34x1[LWP_34]*R2D;
		//Joint[LWY2].RefAngleCurrent = _Q_34x1[LWY2_34]*R2D;

		Joint[RAP].RefAngleCurrent = meas_Q_34x1[RAP_34]*R2D;
		Joint[RAR].RefAngleCurrent = meas_Q_34x1[RAR_34]*R2D;
		Joint[LAP].RefAngleCurrent = meas_Q_34x1[LAP_34]*R2D;
		Joint[LAR].RefAngleCurrent = meas_Q_34x1[LAR_34]*R2D;

		SendRunStopCMD(Joint[LSP].JMC, 0x01);
		SendRunStopCMD(Joint[LSY].JMC, 0x01);
		SendRunStopCMD(Joint[LWY].JMC, 0x01);
		//SendRunStopCMD(Joint[LWY2].JMC, 0x01);

		SendRunStopCMD(Joint[RSP].JMC, 0x01);
		SendRunStopCMD(Joint[RSY].JMC, 0x01);
		SendRunStopCMD(Joint[RWY].JMC, 0x01);
		//SendRunStopCMD(Joint[RWY2].JMC, 0x01);

		SendRunStopCMD(Joint[RAP].JMC, 0x01);
		SendRunStopCMD(Joint[LAP].JMC, 0x01);
		
		/*
		RBgainOverrideHR(Joint[RSP].CAN_channel, Joint[RSP].JMC, 1000, 1000, 100);
		RBgainOverrideHR(Joint[RSY].CAN_channel, Joint[RSY].JMC, 1000, 1000, 100);
		RBgainOverrideHR(Joint[RWY].CAN_channel, Joint[RWY].JMC, 1000, 1000, 100);

		RBgainOverrideHR(Joint[LSP].CAN_channel, Joint[LSP].JMC, 1000, 1000, 100);
		RBgainOverrideHR(Joint[LSY].CAN_channel, Joint[LSY].JMC, 1000, 1000, 100);
		RBgainOverrideHR(Joint[LWY].CAN_channel, Joint[LWY].JMC, 1000, 1000, 100);

		RBgainOverrideHR(Joint[RAP].CAN_channel, Joint[RAP].JMC, 1000, 1000, 100);
		RBgainOverrideHR(Joint[LAP].CAN_channel, Joint[LAP].JMC, 1000, 1000, 100);
		*/
		pre_gain = 0.f;
		pSharedMemory->torque_mode_flag = 0;
		printf("\n position mode");
	}

	//_Q_34x1[RSP_34] =  pre_gain*meas_Q_34x1[RSP_34] + (1.f-pre_gain)*_Q_34x1[RSP_34];
	//_Q_34x1[RSR_34] =  pre_gain*meas_Q_34x1[RSR_34] + (1.f-pre_gain)*_Q_34x1[RSR_34];
	//_Q_34x1[RSY_34] =  pre_gain*meas_Q_34x1[RSY_34] + (1.f-pre_gain)*_Q_34x1[RSY_34];
	//_Q_34x1[REB_34] =  pre_gain*meas_Q_34x1[REB_34] + (1.f-pre_gain)*_Q_34x1[REB_34];
	//_Q_34x1[RWY_34] =  pre_gain*meas_Q_34x1[RWY_34] + (1.f-pre_gain)*_Q_34x1[RWY_34];
	//_Q_34x1[RWP_34] =  pre_gain*meas_Q_34x1[RWP_34] + (1.f-pre_gain)*_Q_34x1[RWP_34];
	//_Q_34x1[RWY2_34] =  pre_gain*meas_Q_34x1[RWY2_34] + (1.f-pre_gain)*_Q_34x1[RWY2_34];
	
	//_Q_34x1[LSP_34] =  pre_gain*meas_Q_34x1[LSP_34] + (1.f-pre_gain)*_Q_34x1[LSP_34];
	//_Q_34x1[LSR_34] =  pre_gain*meas_Q_34x1[LSR_34] + (1.f-pre_gain)*_Q_34x1[LSR_34];
	//_Q_34x1[LSY_34] =  pre_gain*meas_Q_34x1[LSY_34] + (1.f-pre_gain)*_Q_34x1[LSY_34];
	//_Q_34x1[LEB_34] =  pre_gain*meas_Q_34x1[LEB_34] + (1.f-pre_gain)*_Q_34x1[LEB_34];
	//_Q_34x1[LWY_34] =  pre_gain*meas_Q_34x1[LWY_34] + (1.f-pre_gain)*_Q_34x1[LWY_34];
	//_Q_34x1[LWP_34] =  pre_gain*meas_Q_34x1[LWP_34] + (1.f-pre_gain)*_Q_34x1[LWP_34];
	//_Q_34x1[LWY2_34] =  pre_gain*meas_Q_34x1[LWY2_34] + (1.f-pre_gain)*_Q_34x1[LWY2_34];

	//_Qp_33x1[RSP_33] =  pre_gain*meas_Qp_33x1[RSP_33] + (1.f-pre_gain)*_Qp_33x1[RSP_33];
	//_Qp_33x1[RSR_33] =  pre_gain*meas_Qp_33x1[RSR_33] + (1.f-pre_gain)*_Qp_33x1[RSR_33];
	//_Qp_33x1[RSY_33] =  pre_gain*meas_Qp_33x1[RSY_33] + (1.f-pre_gain)*_Qp_33x1[RSY_33];
	//_Qp_33x1[REB_33] =  pre_gain*meas_Qp_33x1[REB_33] + (1.f-pre_gain)*_Qp_33x1[REB_33];
	//_Qp_33x1[RWY_33] =  pre_gain*meas_Qp_33x1[RWY_33] + (1.f-pre_gain)*_Qp_33x1[RWY_33];
	//_Qp_33x1[RWP_33] =  pre_gain*meas_Qp_33x1[RWP_33] + (1.f-pre_gain)*_Qp_33x1[RWP_33];
	//_Qp_33x1[RWY2_33] =  pre_gain*meas_Qp_33x1[RWY2_33] + (1.f-pre_gain)*_Qp_33x1[RWY2_33];
	
	//_Qp_33x1[LSP_33] =  pre_gain*meas_Qp_33x1[LSP_33] + (1.f-pre_gain)*_Qp_33x1[LSP_33];
	//_Qp_33x1[LSR_33] =  pre_gain*meas_Qp_33x1[LSR_33] + (1.f-pre_gain)*_Qp_33x1[LSR_33];
	//_Qp_33x1[LSY_33] =  pre_gain*meas_Qp_33x1[LSY_33] + (1.f-pre_gain)*_Qp_33x1[LSY_33];
	//_Qp_33x1[LEB_33] =  pre_gain*meas_Qp_33x1[LEB_33] + (1.f-pre_gain)*_Qp_33x1[LEB_33];
	//_Qp_33x1[LWY_33] =  pre_gain*meas_Qp_33x1[LWY_33] + (1.f-pre_gain)*_Qp_33x1[LWY_33];
	//_Qp_33x1[LWP_33] =  pre_gain*meas_Qp_33x1[LWP_33] + (1.f-pre_gain)*_Qp_33x1[LWP_33];
	//_Qp_33x1[LWY2_33] =  pre_gain*meas_Qp_33x1[LWY2_33] + (1.f-pre_gain)*_Qp_33x1[LWY2_33];

	if(pSharedMemory->torque_mode_flag == 1 || pSharedMemory->torque_mode_flag == 2)
	{
		for(i=1;i<=33;i++)
		{
			duty_joint_limit_33x1[i] = 0.f;
			ct_33x1[i] = 0.f;
		}
		getDuty4JointLimit(meas_Q_34x1, duty_joint_limit_33x1);
		getFricCompen(meas_Qp_33x1, fric_compen_33x1);
		getGravityTorque(meas_Q_34x1, gravity_33x1);
		pSharedMemory->disp_grav_33x1[RSP_33] = gravity_33x1[RSP_33];
		pSharedMemory->disp_grav_33x1[RSR_33] = gravity_33x1[RSR_33];
		pSharedMemory->disp_grav_33x1[RSY_33] = gravity_33x1[RSY_33];
		pSharedMemory->disp_grav_33x1[REB_33] = gravity_33x1[REB_33];
		pSharedMemory->disp_grav_33x1[RWY_33] = gravity_33x1[RWY_33];
		pSharedMemory->disp_grav_33x1[RWP_33] = gravity_33x1[RWP_33];
		pSharedMemory->disp_grav_33x1[RWY2_33] = gravity_33x1[RWY2_33];

		pSharedMemory->disp_grav_33x1[LSP_33] = gravity_33x1[LSP_33];
		pSharedMemory->disp_grav_33x1[LSR_33] = gravity_33x1[LSR_33];
		pSharedMemory->disp_grav_33x1[LSY_33] = gravity_33x1[LSY_33];
		pSharedMemory->disp_grav_33x1[LEB_33] = gravity_33x1[LEB_33];
		pSharedMemory->disp_grav_33x1[LWY_33] = gravity_33x1[LWY_33];
		pSharedMemory->disp_grav_33x1[LWP_33] = gravity_33x1[LWP_33];
		pSharedMemory->disp_grav_33x1[LWY2_33] = gravity_33x1[LWY2_33];

		ct_33x1[RSP_33] = gain_ovr_rarm*(pSharedMemory->kd[16]*(-meas_Qp_33x1[RSP_33]) + pSharedMemory->kp[16]*(ref_Q_34x1[RSP_34]-meas_Q_34x1[RSP_34]));
		ct_33x1[RSR_33] = pSharedMemory->kd[17]*(-meas_Qp_33x1[RSR_33]) + pSharedMemory->kp[17]*(ref_Q_34x1[RSR_34]-meas_Q_34x1[RSR_34]);
		ct_33x1[RSY_33] = pSharedMemory->kd[18]*(-meas_Qp_33x1[RSY_33]) + pSharedMemory->kp[18]*(ref_Q_34x1[RSY_34]-meas_Q_34x1[RSY_34]);
		ct_33x1[REB_33] = gain_ovr_rarm*(pSharedMemory->kd[19]*(-meas_Qp_33x1[REB_33]) + pSharedMemory->kp[19]*(ref_Q_34x1[REB_34]-meas_Q_34x1[REB_34]));
		ct_33x1[RWY_33] = pSharedMemory->kd[20]*(-meas_Qp_33x1[RWY_33]) + pSharedMemory->kp[20]*(neutral_Q_34x1[RWY_34]-meas_Q_34x1[RWY_34]);
		ct_33x1[RWP_33] = gain_ovr_rarm*(pSharedMemory->kd[21]*(-meas_Qp_33x1[RWP_33]) + pSharedMemory->kp[21]*(neutral_Q_34x1[RWP_34]-meas_Q_34x1[RWP_34]));

		ct_33x1[LSP_33] = gain_ovr_larm*(pSharedMemory->kd[16]*(-meas_Qp_33x1[LSP_33]) + pSharedMemory->kp[16]*(ref_Q_34x1[LSP_34]-meas_Q_34x1[LSP_34]));
		ct_33x1[LSR_33] = pSharedMemory->kd[17]*(-meas_Qp_33x1[LSR_33]) + pSharedMemory->kp[17]*(ref_Q_34x1[LSR_34]-meas_Q_34x1[LSR_34]);
		ct_33x1[LSY_33] = pSharedMemory->kd[18]*(-meas_Qp_33x1[LSY_33]) + pSharedMemory->kp[18]*(ref_Q_34x1[LSY_34]-meas_Q_34x1[LSY_34]);
		ct_33x1[LEB_33] = gain_ovr_larm*(pSharedMemory->kd[19]*(-meas_Qp_33x1[LEB_33]) + pSharedMemory->kp[19]*(ref_Q_34x1[LEB_34]-meas_Q_34x1[LEB_34]));
		ct_33x1[LWY_33] = pSharedMemory->kd[20]*(-meas_Qp_33x1[LWY_33]) + pSharedMemory->kp[20]*(neutral_Q_34x1[LWY_34]-meas_Q_34x1[LWY_34]);
		ct_33x1[LWP_33] = gain_ovr_larm*(pSharedMemory->kd[21]*(-meas_Qp_33x1[LWP_33]) + pSharedMemory->kp[21]*(neutral_Q_34x1[LWP_34]-meas_Q_34x1[LWP_34]));

		ct_33x1[RSP_33] = _gain_task[RSP_33]*ct_33x1[RSP_33]+_gain_gravity[RSP_33]*gravity_33x1[RSP_33];
		ct_33x1[RSR_33] = _gain_task[RSR_33]*ct_33x1[RSR_33]+_gain_gravity[RSR_33]*gravity_33x1[RSR_33];
		ct_33x1[RSY_33] = _gain_task[RSY_33]*ct_33x1[RSY_33]+_gain_gravity[RSY_33]*gravity_33x1[RSY_33];
		ct_33x1[REB_33] = _gain_task[REB_33]*ct_33x1[REB_33]+_gain_gravity[REB_33]*gravity_33x1[REB_33];
		ct_33x1[RWY_33] = _gain_task[RWY_33]*ct_33x1[RWY_33]+_gain_gravity[RWY_33]*gravity_33x1[RWY_33];
		ct_33x1[RWP_33] = _gain_task[RWP_33]*ct_33x1[RWP_33]+_gain_gravity[RWP_33]*gravity_33x1[RWP_33];

		ct_33x1[LSP_33] = _gain_task[LSP_33]*ct_33x1[LSP_33]+_gain_gravity[LSP_33]*gravity_33x1[LSP_33];
		ct_33x1[LSR_33] = _gain_task[LSR_33]*ct_33x1[LSR_33]+_gain_gravity[LSR_33]*gravity_33x1[LSR_33];
		ct_33x1[LSY_33] = _gain_task[LSY_33]*ct_33x1[LSY_33]+_gain_gravity[LSY_33]*gravity_33x1[LSY_33];
		ct_33x1[LEB_33] = _gain_task[LEB_33]*ct_33x1[LEB_33]+_gain_gravity[LEB_33]*gravity_33x1[LEB_33];
		ct_33x1[LWY_33] = _gain_task[LWY_33]*ct_33x1[LWY_33]+_gain_gravity[LWY_33]*gravity_33x1[LWY_33];
		ct_33x1[LWP_33] = _gain_task[LWP_33]*ct_33x1[LWP_33]+_gain_gravity[LWP_33]*gravity_33x1[LWP_33];
	
		pSharedMemory->disp_ct_33x1[RSP_33] = ct_33x1[RSP_33];
		pSharedMemory->disp_ct_33x1[RSR_33] = ct_33x1[RSR_33];
		pSharedMemory->disp_ct_33x1[RSY_33] = ct_33x1[RSY_33];
		pSharedMemory->disp_ct_33x1[REB_33] = ct_33x1[REB_33];
		pSharedMemory->disp_ct_33x1[RWY_33] = ct_33x1[RWY_33];
		pSharedMemory->disp_ct_33x1[RWP_33] = ct_33x1[RWP_33];
		//pSharedMemory->disp_ct_33x1[RWY2_33] = ct_33x1[RWY2_33];
		
		pSharedMemory->disp_ct_33x1[LSP_33] = ct_33x1[LSP_33];
		pSharedMemory->disp_ct_33x1[LSR_33] = ct_33x1[LSR_33];
		pSharedMemory->disp_ct_33x1[LSY_33] = ct_33x1[LSY_33];
		pSharedMemory->disp_ct_33x1[LEB_33] = ct_33x1[LEB_33];
		pSharedMemory->disp_ct_33x1[LWY_33] = ct_33x1[LWY_33];
		pSharedMemory->disp_ct_33x1[LWP_33] = ct_33x1[LWP_33];
		//pSharedMemory->disp_ct_33x1[LWY2_33] = ct_33x1[LWY2_33];

		pSharedMemory->disp_duty_33x1[RSP_33] = duty_33x1[RSP_33] = limitDuty(0.4f, torque2duty(RSP, pre_gain*ct_33x1[RSP_33]) + pre_gain*fric_compen_33x1[RSP_33] + duty_joint_limit_33x1[RSP_33]);
		pSharedMemory->disp_duty_33x1[RSR_33] = duty_33x1[RSR_33] = limitDuty(0.4f, torque2duty(RSR, pre_gain*ct_33x1[RSR_33]) + pre_gain*fric_compen_33x1[RSR_33] + duty_joint_limit_33x1[RSR_33]);
		pSharedMemory->disp_duty_33x1[RSY_33] = duty_33x1[RSY_33] = limitDuty(0.4f, torque2duty(RSY, pre_gain*ct_33x1[RSY_33]) + pre_gain*fric_compen_33x1[RSY_33] + duty_joint_limit_33x1[RSY_33]);
		pSharedMemory->disp_duty_33x1[REB_33] = duty_33x1[REB_33] = limitDuty(0.4f, torque2duty(REB, pre_gain*ct_33x1[REB_33]) + pre_gain*fric_compen_33x1[REB_33] + duty_joint_limit_33x1[REB_33]);
		pSharedMemory->disp_duty_33x1[RWY_33] = duty_33x1[RWY_33] = limitDuty(0.4f, torque2duty(RWY, pre_gain*ct_33x1[RWY_33]) + pre_gain*fric_compen_33x1[RWY_33] + duty_joint_limit_33x1[RWY_33]);
		pSharedMemory->disp_duty_33x1[RWP_33] = duty_33x1[RWP_33] = limitDuty(0.4f, torque2duty(RWP, pre_gain*ct_33x1[RWP_33]) + pre_gain*fric_compen_33x1[RWP_33] + duty_joint_limit_33x1[RWP_33]);
		//pSharedMemory->disp_duty_33x1[RWY2_33] = duty_33x1[RWY2_33] = limitDuty(0.4f, torque2duty(RWY2, pre_gain*ct_33x1[RWY2_33]) + pre_gain*fric_compen_33x1[RWY2_33] + duty_joint_limit_33x1[RWY2_33]);

		pSharedMemory->disp_duty_33x1[LSP_33] = duty_33x1[LSP_33] = limitDuty(0.4f, torque2duty(LSP, pre_gain*ct_33x1[LSP_33]) + pre_gain*fric_compen_33x1[LSP_33] + duty_joint_limit_33x1[LSP_33]);
		pSharedMemory->disp_duty_33x1[LSR_33] = duty_33x1[LSR_33] = limitDuty(0.4f, torque2duty(LSR, pre_gain*ct_33x1[LSR_33]) + pre_gain*fric_compen_33x1[LSR_33] + duty_joint_limit_33x1[LSR_33]);
		pSharedMemory->disp_duty_33x1[LSY_33] = duty_33x1[LSY_33] = limitDuty(0.4f, torque2duty(LSY, pre_gain*ct_33x1[LSY_33]) + pre_gain*fric_compen_33x1[LSY_33] + duty_joint_limit_33x1[LSY_33]);
		pSharedMemory->disp_duty_33x1[LEB_33] = duty_33x1[LEB_33] = limitDuty(0.4f, torque2duty(LEB, pre_gain*ct_33x1[LEB_33]) + pre_gain*fric_compen_33x1[LEB_33] + duty_joint_limit_33x1[LEB_33]);
		pSharedMemory->disp_duty_33x1[LWY_33] = duty_33x1[LWY_33] = limitDuty(0.4f, torque2duty(LWY, pre_gain*ct_33x1[LWY_33]) + pre_gain*fric_compen_33x1[LWY_33] + duty_joint_limit_33x1[LWY_33]);
		pSharedMemory->disp_duty_33x1[LWP_33] = duty_33x1[LWP_33] = limitDuty(0.4f, torque2duty(LWP, pre_gain*ct_33x1[LWP_33]) + pre_gain*fric_compen_33x1[LWP_33] + duty_joint_limit_33x1[LWP_33]);
		//pSharedMemory->disp_duty_33x1[LWY2_33] = duty_33x1[LWY2_33] = limitDuty(0.4f, torque2duty(LWY2, pre_gain*ct_33x1[LWY2_33]) + pre_gain*fric_compen_33x1[LWY2_33] + duty_joint_limit_33x1[LWY2_33]);

		
		RBpwmCommandHR2ch(CAN1, Joint[RSP].JMC, (int)(PWM_SIGN_RSP*1000.f*duty_33x1[RSP_33]), (int)(PWM_SIGN_RSR*1000.f*duty_33x1[RSR_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[RSY].JMC, (int)(PWM_SIGN_RSY*1000.f*duty_33x1[RSY_33]), (int)(PWM_SIGN_REB*1000.f*duty_33x1[REB_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[RWY].JMC, (int)(PWM_SIGN_RWY*1000.f*duty_33x1[RWY_33]), (int)(PWM_SIGN_RWP*1000.f*duty_33x1[RWP_33]), 1);
		//RBpwmCommandHR2ch(CAN1, Joint[RWY2].JMC, (int)(PWM_SIGN_RWY2*1000.f*duty_33x1[RWY2_33]), 0, 1);

		RBpwmCommandHR2ch(CAN1, Joint[LSP].JMC, (int)(PWM_SIGN_LSP*1000.f*duty_33x1[LSP_33]), (int)(PWM_SIGN_LSR*1000.f*duty_33x1[LSR_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[LSY].JMC, (int)(PWM_SIGN_LSY*1000.f*duty_33x1[LSY_33]), (int)(PWM_SIGN_LEB*1000.f*duty_33x1[LEB_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[LWY].JMC, (int)(PWM_SIGN_LWY*1000.f*duty_33x1[LWY_33]), (int)(PWM_SIGN_LWP*1000.f*duty_33x1[LWP_33]), 1);
		//RBpwmCommandHR2ch(CAN1, Joint[LWY2].JMC, (int)(PWM_SIGN_LWY2*1000.f*duty_33x1[LWY2_33]), 0, 1);

	}
	
	return 0;	
}



int WBIK_DRC_Kirk_Biped(unsigned int n)  
{
	int i,j;
	
	float Qpp_33x1[34], meas_Q_34x1[35], meas_Qp_33x1[34];
	float X1_33x1[34], Xrf_6x1[7], Xlf_6x1[7], Xrh_6x1[7], Xlh_6x1[7], Xcom_3x1[4], Xpel_3x1[4], Xwst, Xpelz, X2_33x1[34];  // task variables
	float ub_27x1[28], lb_27x1[28], qpp_limited[34];  // joint upper-bound, lower-bound
	int index_limited[28];
	
	float des_pCOM_3x1[4], des_vCOM_3x1[4];
	float des_pRF_3x1[4], des_pLF_3x1[4], des_vRF_3x1[4], des_vLF_3x1[4];
	float des_qRF_4x1[5], des_qLF_4x1[5], des_wRF_3x1[4], des_wLF_3x1[4];
	float des_pRH_3x1[4], des_pLH_3x1[4], des_vRH_3x1[4], des_vLH_3x1[4];
	float des_qRH_4x1[5], des_qLH_4x1[5], des_wRH_3x1[4], des_wLH_3x1[4];
	float des_WST, des_WSTp, des_qPEL_4x1[5];
	float des_pPELz, des_vPELz;
	float vCOM_3x1[4];

	BOOL isLimited = 0;
	int dim_primary_task, dim_redundant_task;

	float lL, lR, fRFz, fLFz;
	float rhr_compen = 0.f;
	float lhr_compen = 0.f;
	float rhy_compen = 0.f;
	float lhy_compen = 0.f;
	float fric_compen_33x1[34], ct_33x1[34], gravity_33x1[34], duty_33x1[34], duty_joint_limit_33x1[34];

	static char half_flag;
	static int count, count_RANK, count_LANK, count_RANK2, count_LANK2;
	static float neutral_Q_34x1[35], Q_old_34x1[35], Q_old2_34x1[35];
	static float pCOM1_3x1[4], pPELz1, pRF1_3x1[4], pLF1_3x1[4], qRF1_4x1[5], qLF1_4x1[5], qPEL1_4x1[5];
	static float pCOM2_3x1[4], pPELz2, pRF2_3x1[4], pLF2_3x1[4], qRF2_4x1[5], qLF2_4x1[5];
	static float pRH1_3x1[4], pLH1_3x1[4], qRH1_4x1[5], qLH1_4x1[5];
	static float pRH2_3x1[4], pLH2_3x1[4], qRH2_4x1[5], qLH2_4x1[5];
	static float qWST1;
	static float t;
	static float pre_gain_lhr, pre_gain_rhr;

	float ftemp1_7x1[8], ftemp2_7x1[8], ftemp3_7x1[8], ftemp4_7x1[8];
	float pre_gain = 0.f;
	float pre_gain_lb = 0.f;
	char update_pos1_flag = 0;

	float ref_Q_34x1[35];
	float gain_ovr_rarm, gain_ovr_larm, gain_ovr_rank, gain_ovr_lank;

	static char override_flag, override_RHR_flag, override_LHR_flag, reset_RFT_flag, reset_LFT_flag, RANK_loose_flag=0, LANK_loose_flag=0;
	float gtorque_33x1[34];

	static float dangRFz0, dangLFz0, pRF_offset_3x1[4], pLF_offset_3x1[4];
	static unsigned char walking_stage_old[6];

	static float RSP_swing = 0.f;
	static float LSP_swing = 0.f;
	static float REB_swing = 0.f;
	static float LEB_swing = 0.f;

	if(_FKineUpdatedFlag != 1)
		return -4;
	
	_Q_34x1[RWY2_34] = 0.f;
	_Q_34x1[LWY2_34] = 0.f;
	if(n<=1)
	{
		neutral_Q_34x1[WST_34] = Joint[WST].RefAngleCurrent*D2R;

		neutral_Q_34x1[RSP_34] = Joint[RSP].RefAngleCurrent*D2R;
		neutral_Q_34x1[RSR_34] = (Joint[RSR].RefAngleCurrent + OFFSET_RSR)*D2R;
		neutral_Q_34x1[RSY_34] = Joint[RSY].RefAngleCurrent*D2R;
		neutral_Q_34x1[REB_34] = (Joint[REB].RefAngleCurrent + OFFSET_REB)*D2R;
		neutral_Q_34x1[RWY_34] = Joint[RWY].RefAngleCurrent*D2R;
		neutral_Q_34x1[RWP_34] = Joint[RWP].RefAngleCurrent*D2R;
		//neutral_Q_34x1[RWY2_34] = Joint[RWY2].RefAngleCurrent*D2R;
		
		neutral_Q_34x1[LSP_34] = Joint[LSP].RefAngleCurrent*D2R;
		neutral_Q_34x1[LSR_34] = (Joint[LSR].RefAngleCurrent + OFFSET_LSR)*D2R;
		neutral_Q_34x1[LSY_34] = Joint[LSY].RefAngleCurrent*D2R;
		neutral_Q_34x1[LEB_34] = (Joint[LEB].RefAngleCurrent + OFFSET_LEB)*D2R;
		neutral_Q_34x1[LWY_34] = Joint[LWY].RefAngleCurrent*D2R;
		neutral_Q_34x1[LWP_34] = Joint[LWP].RefAngleCurrent*D2R;
		//neutral_Q_34x1[LWY2_34] = Joint[LWY2].RefAngleCurrent*D2R;

		neutral_Q_34x1[RAP_34] = Joint[RAP].RefAngleCurrent*D2R;
		neutral_Q_34x1[RAR_34] = Joint[RAR].RefAngleCurrent*D2R;
		neutral_Q_34x1[LAP_34] = Joint[LAP].RefAngleCurrent*D2R;
		neutral_Q_34x1[LAR_34] = Joint[LAR].RefAngleCurrent*D2R;

		qPEL1_4x1[1] = _Q_34x1[4];
		qPEL1_4x1[2] = _Q_34x1[5];
		qPEL1_4x1[3] = _Q_34x1[6];
		qPEL1_4x1[4] = _Q_34x1[7];

		_pPEL0_3x1[1] = _Q_34x1[1];
		_pPEL0_3x1[2] = _Q_34x1[2];
		pPELz1 = _pPEL0_3x1[3] = _Q_34x1[3];

		qWST1 = _Q_34x1[8];

		pCOM1_3x1[1] = _pCOM0_3x1[1] = _pCOM_3x1[1];
		pCOM1_3x1[2] = _pCOM0_3x1[2] = _pCOM_3x1[2];

		pRF1_3x1[1] = _pRF0_3x1[1] = _pRF_L_3x1[1];
		pRF1_3x1[2] = _pRF0_3x1[2] = _pRF_L_3x1[2];
		pRF1_3x1[3] = _pRF0_3x1[3] = _pRF_L_3x1[3];

		qRF1_4x1[1] = _qRF0_4x1[1] = _qRF_L_4x1[1];
		qRF1_4x1[2] = _qRF0_4x1[2] = _qRF_L_4x1[2];
		qRF1_4x1[3] = _qRF0_4x1[3] = _qRF_L_4x1[3];
		qRF1_4x1[4] = _qRF0_4x1[4] = _qRF_L_4x1[4];

		pLF1_3x1[1] = _pLF0_3x1[1] = _pLF_L_3x1[1];
		pLF1_3x1[2] = _pLF0_3x1[2] = _pLF_L_3x1[2];
		pLF1_3x1[3] = _pLF0_3x1[3] = _pLF_L_3x1[3];

		qLF1_4x1[1] = _qLF0_4x1[1] = _qLF_L_4x1[1];
		qLF1_4x1[2] = _qLF0_4x1[2] = _qLF_L_4x1[2];
		qLF1_4x1[3] = _qLF0_4x1[3] = _qLF_L_4x1[3];
		qLF1_4x1[4] = _qLF0_4x1[4] = _qLF_L_4x1[4];

		pRH1_3x1[1] = _pRH0_3x1[1] =  _pRH_L_3x1[1];
		pRH1_3x1[2] = _pRH0_3x1[2] =  _pRH_L_3x1[2];
		pRH1_3x1[3] = _pRH0_3x1[3] =  _pRH_L_3x1[3];

		qRH1_4x1[1] = _qRH0_4x1[1] =  _qRH_L_4x1[1];
		qRH1_4x1[2] = _qRH0_4x1[2] =  _qRH_L_4x1[2];
		qRH1_4x1[3] = _qRH0_4x1[3] =  _qRH_L_4x1[3];
		qRH1_4x1[4] = _qRH0_4x1[4] =  _qRH_L_4x1[4];

		pLH1_3x1[1] = _pLH0_3x1[1] =  _pLH_L_3x1[1];
		pLH1_3x1[2] = _pLH0_3x1[2] =  _pLH_L_3x1[2];
		pLH1_3x1[3] = _pLH0_3x1[3] =  _pLH_L_3x1[3];

		qLH1_4x1[1] = _qLH0_4x1[1] =  _qLH_L_4x1[1];
		qLH1_4x1[2] = _qLH0_4x1[2] =  _qLH_L_4x1[2];
		qLH1_4x1[3] = _qLH0_4x1[3] =  _qLH_L_4x1[3];
		qLH1_4x1[4] = _qLH0_4x1[4] =  _qLH_L_4x1[4];

		meas_Q_34x1[RSP_34] = ((float)Joint[RSP].EncoderValue)/Joint[RSP].PPR*D2R;
		meas_Q_34x1[RSR_34] = ((float)Joint[RSR].EncoderValue)/Joint[RSR].PPR*D2R + OFFSET_RSR*D2R;
		meas_Q_34x1[RSY_34] = ((float)Joint[RSY].EncoderValue)/Joint[RSY].PPR*D2R;
		meas_Q_34x1[REB_34] = ((float)Joint[REB].EncoderValue)/Joint[REB].PPR*D2R + OFFSET_REB*D2R;
		meas_Q_34x1[RWY_34] = ((float)Joint[RWY].EncoderValue)/Joint[RWY].PPR*D2R;
		meas_Q_34x1[RWP_34] = ((float)Joint[RWP].EncoderValue)/Joint[RWP].PPR*D2R;
		//meas_Q_34x1[RWY2_34] = ((float)Joint[RWY2].EncoderValue)/Joint[RWY2].PPR*D2R;

		meas_Q_34x1[LSP_34] = ((float)Joint[LSP].EncoderValue)/Joint[LSP].PPR*D2R;
		meas_Q_34x1[LSR_34] = ((float)Joint[LSR].EncoderValue)/Joint[LSR].PPR*D2R + OFFSET_LSR*D2R;
		meas_Q_34x1[LSY_34] = ((float)Joint[LSY].EncoderValue)/Joint[LSY].PPR*D2R;
		meas_Q_34x1[LEB_34] = ((float)Joint[LEB].EncoderValue)/Joint[LEB].PPR*D2R + OFFSET_LEB*D2R;
		meas_Q_34x1[LWY_34] = ((float)Joint[LWY].EncoderValue)/Joint[LWY].PPR*D2R;
		meas_Q_34x1[LWP_34] = ((float)Joint[LWP].EncoderValue)/Joint[LWP].PPR*D2R;
		//meas_Q_34x1[LWY2_34] = ((float)Joint[LWY2].EncoderValue)/Joint[LWY2].PPR*D2R;

		meas_Q_34x1[RAP_34] = ((float)Joint[RAP].EncoderValue)/Joint[RAP].PPR*D2R;
		meas_Q_34x1[RAR_34] = ((float)Joint[RAR].EncoderValue)/Joint[RAR].PPR*D2R;
		meas_Q_34x1[LAP_34] = ((float)Joint[LAP].EncoderValue)/Joint[LAP].PPR*D2R;
		meas_Q_34x1[LAR_34] = ((float)Joint[LAR].EncoderValue)/Joint[LAR].PPR*D2R;

		Q_old2_34x1[RSP_34] = Q_old_34x1[RSP_34] = meas_Q_34x1[RSP_34];
		Q_old2_34x1[RSR_34] = Q_old_34x1[RSR_34] = meas_Q_34x1[RSR_34];
		Q_old2_34x1[RSY_34] = Q_old_34x1[RSY_34] = meas_Q_34x1[RSY_34];
		Q_old2_34x1[REB_34] = Q_old_34x1[REB_34] = meas_Q_34x1[REB_34];
		Q_old2_34x1[RWY_34] = Q_old_34x1[RWY_34] = meas_Q_34x1[RWY_34];
		Q_old2_34x1[RWP_34] = Q_old_34x1[RWP_34] = meas_Q_34x1[RWP_34];
		//Q_old2_34x1[RWY2_34] = Q_old_34x1[RWY2_34] = meas_Q_34x1[RWY2_34];

		Q_old2_34x1[LSP_34] = Q_old_34x1[LSP_34] = meas_Q_34x1[LSP_34];
		Q_old2_34x1[LSR_34] = Q_old_34x1[LSR_34] = meas_Q_34x1[LSR_34];
		Q_old2_34x1[LSY_34] = Q_old_34x1[LSY_34] = meas_Q_34x1[LSY_34];
		Q_old2_34x1[LEB_34] = Q_old_34x1[LEB_34] = meas_Q_34x1[LEB_34];
		Q_old2_34x1[LWY_34] = Q_old_34x1[LWY_34] = meas_Q_34x1[LWY_34];
		Q_old2_34x1[LWP_34] = Q_old_34x1[LWP_34] = meas_Q_34x1[LWP_34];
		//Q_old2_34x1[LWY2_34] = Q_old_34x1[LWY2_34] = meas_Q_34x1[LWY2_34];

		Q_old2_34x1[RAP_34] = Q_old_34x1[RAP_34] = meas_Q_34x1[RAP_34];
		Q_old2_34x1[RAR_34] = Q_old_34x1[RAR_34] = meas_Q_34x1[RAR_34];
		Q_old2_34x1[LAP_34] = Q_old_34x1[LAP_34] = meas_Q_34x1[LAP_34];
		Q_old2_34x1[LAR_34] = Q_old_34x1[LAR_34] = meas_Q_34x1[LAR_34];

		diff_mm((const float**)_jRH_6x33,6,33, (const float**)_jRH_6x33, _jRHp_6x33);
		subs_m((const float**)_jRH_6x33,6,33, _jRH_old_6x33);
		subs_m((const float**)_jRH_6x33,6,33, _jRH_old2_6x33);
		
		diff_mm((const float**)_jLH_6x33,6,33, (const float**)_jLH_6x33, _jLHp_6x33);
		subs_m((const float**)_jLH_6x33,6,33, _jLH_old_6x33);
		subs_m((const float**)_jLH_6x33,6,33, _jLH_old2_6x33);

		diff_mm((const float**)_jRF_6x33,6,33, (const float**)_jRF_6x33, _jRFp_6x33);
		subs_m((const float**)_jRF_6x33,6,33, _jRF_old_6x33);
		subs_m((const float**)_jRF_6x33,6,33, _jRF_old2_6x33);

		diff_mm((const float**)_jLF_6x33,6,33, (const float**)_jLF_6x33, _jLFp_6x33);
		subs_m((const float**)_jLF_6x33,6,33, _jLF_old_6x33);
		subs_m((const float**)_jLF_6x33,6,33, _jLF_old2_6x33);
		
		diff_mm((const float**)_jCOM_3x33,3,33, (const float**)_jCOM_3x33, _jCOMp_3x33);
		subs_m((const float**)_jCOM_3x33,3,33, _jCOM_old_3x33);
		subs_m((const float**)_jCOM_3x33,3,33, _jCOM_old2_3x33);
		
		half_flag = 0;
		pSharedMemory->transform_flag = 0;
		t = 0.f;
		count = 0;

		override_RHR_flag = 0;
		override_LHR_flag = 0;

		printf("\n _pRF : %f %f %f", _pRF_L_3x1[1],_pRF_L_3x1[2],_pRF_L_3x1[3]);
		printf("\n _pLF : %f %f %f", _pLF_L_3x1[1],_pLF_L_3x1[2],_pLF_L_3x1[3]);
		printf("\n _pRH_L : %f %f %f", _pRH_L_3x1[1],_pRH_L_3x1[2],_pRH_L_3x1[3]);
		printf("\n _pLH_L : %f %f %f", _pLH_L_3x1[1],_pLH_L_3x1[2],_pLH_L_3x1[3]);
		printf("\n _pCOM : %f %f %f", pCOM1_3x1[1],pCOM1_3x1[2], _pCOM_3x1[3]);
		printf("\n ---------------------------------------------\n");

		
		
		return 0;
	}
	
	diff_mm((const float**)_jRF_6x33,6,33, (const float**)_jRF_old2_6x33, _jRFp_6x33);
	subs_m((const float**)_jRF_old_6x33,6,33, _jRF_old2_6x33);
	subs_m((const float**)_jRF_6x33,6,33, _jRF_old_6x33);
	mult_sm((const float**)_jRFp_6x33,6,33, 1.f/(2.f*DT), _jRFp_6x33);
	
	diff_mm((const float**)_jLF_6x33,6,33, (const float**)_jLF_old2_6x33, _jLFp_6x33);
	subs_m((const float**)_jLF_old_6x33,6,33, _jLF_old2_6x33);
	subs_m((const float**)_jLF_6x33,6,33, _jLF_old_6x33);
	mult_sm((const float**)_jLFp_6x33,6,33, 1.f/(2.f*DT), _jLFp_6x33);

	diff_mm((const float**)_jCOM_3x33,3,33, (const float**)_jCOM_old2_3x33, _jCOMp_3x33);
	subs_m((const float**)_jCOM_old_3x33,3,33, _jCOM_old2_3x33);
	subs_m((const float**)_jCOM_3x33,3,33, _jCOM_old_3x33);
	mult_sm((const float**)_jCOMp_3x33,3,33, 1.f/(2.f*DT), _jCOMp_3x33);

	gain_ovr_rarm = pSharedMemory->kp[22];
	gain_ovr_larm = pSharedMemory->kp[22];
	gain_ovr_rank = pSharedMemory->kp[25];
	gain_ovr_lank = pSharedMemory->kp[25];

	for(i=1;i<=34;i++)
		meas_Q_34x1[i] = _Q_34x1[i];

	if(pSharedMemory->biped_flag == -2)
	{
		des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];
	
		des_pPELz = pPELz1;

		des_WST = qWST1;
		
		des_pCOM_3x1[1] = pCOM1_3x1[1];
		des_pCOM_3x1[2] = pCOM1_3x1[2];

		des_pRF_3x1[1] = pRF1_3x1[1];
		des_pRF_3x1[2] = pRF1_3x1[2] + one_cos(t, -0.12f, 3.f);
		des_pRF_3x1[3] = pRF1_3x1[3];
		
		des_pLF_3x1[1] = pLF1_3x1[1];
		des_pLF_3x1[2] = pLF1_3x1[2] + one_cos(t, -0.12f, 3.f);
		des_pLF_3x1[3] = pLF1_3x1[3];
		
		des_qRF_4x1[1] = qRF1_4x1[1];
		des_qRF_4x1[2] = qRF1_4x1[2];
		des_qRF_4x1[3] = qRF1_4x1[3];
		des_qRF_4x1[4] = qRF1_4x1[4];

		des_qLF_4x1[1] = qLF1_4x1[1];
		des_qLF_4x1[2] = qLF1_4x1[2];
		des_qLF_4x1[3] = qLF1_4x1[3];
		des_qLF_4x1[4] = qLF1_4x1[4];

		des_pRH_3x1[1] = pRH1_3x1[1];
		des_pRH_3x1[2] = pRH1_3x1[2];
		des_pRH_3x1[3] = pRH1_3x1[3];

		des_pLH_3x1[1] = pLH1_3x1[1];
		des_pLH_3x1[2] = pLH1_3x1[2];
		des_pLH_3x1[3] = pLH1_3x1[3];

		des_qRH_4x1[1] = qRH1_4x1[1];
		des_qRH_4x1[2] = qRH1_4x1[2];
		des_qRH_4x1[3] = qRH1_4x1[3];
		des_qRH_4x1[4] = qRH1_4x1[4];

		des_qLH_4x1[1] = qLH1_4x1[1];
		des_qLH_4x1[2] = qLH1_4x1[2];
		des_qLH_4x1[3] = qLH1_4x1[3];
		des_qLH_4x1[4] = qLH1_4x1[4];

		t += DT;

		if(t > 3.f)
		{
			t = 0.f;
			update_pos1_flag = 1;
			pSharedMemory->biped_flag = -1;
			printf("\n transforming to quad... ");
		}
	}
	else if(pSharedMemory->biped_flag == -1)
	{
		des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];
		
		des_pPELz = pPELz1;

		des_WST = qWST1;
		
		des_pCOM_3x1[1] = pCOM1_3x1[1];
		des_pCOM_3x1[2] = pCOM1_3x1[2];

		des_pRF_3x1[1] = pRF1_3x1[1];
		des_pRF_3x1[2] = pRF1_3x1[2];
		des_pRF_3x1[3] = pRF1_3x1[3] + one_cos(t, 0.1f, 3.f);
		
		des_pLF_3x1[1] = pLF1_3x1[1];
		des_pLF_3x1[2] = pLF1_3x1[2];
		des_pLF_3x1[3] = pLF1_3x1[3];
		
		des_qRF_4x1[1] = qRF1_4x1[1];
		des_qRF_4x1[2] = qRF1_4x1[2];
		des_qRF_4x1[3] = qRF1_4x1[3];
		des_qRF_4x1[4] = qRF1_4x1[4];

		des_qLF_4x1[1] = qLF1_4x1[1];
		des_qLF_4x1[2] = qLF1_4x1[2];
		des_qLF_4x1[3] = qLF1_4x1[3];
		des_qLF_4x1[4] = qLF1_4x1[4];

		des_pRH_3x1[1] = pRH1_3x1[1];
		des_pRH_3x1[2] = pRH1_3x1[2];
		des_pRH_3x1[3] = pRH1_3x1[3];

		des_pLH_3x1[1] = pLH1_3x1[1];
		des_pLH_3x1[2] = pLH1_3x1[2];
		des_pLH_3x1[3] = pLH1_3x1[3];

		des_qRH_4x1[1] = qRH1_4x1[1];
		des_qRH_4x1[2] = qRH1_4x1[2];
		des_qRH_4x1[3] = qRH1_4x1[3];
		des_qRH_4x1[4] = qRH1_4x1[4];

		des_qLH_4x1[1] = qLH1_4x1[1];
		des_qLH_4x1[2] = qLH1_4x1[2];
		des_qLH_4x1[3] = qLH1_4x1[3];
		des_qLH_4x1[4] = qLH1_4x1[4];

		t += DT;

		if(t > 3.f)
		{
			t = 0.f;
			update_pos1_flag = 1;
			pSharedMemory->biped_flag = 0;
		}
	}
	else if(pSharedMemory->biped_flag == 0) // staying
	{
		des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];
		
		des_pPELz =  pPELz1;

		des_WST = qWST1;
		
		des_pCOM_3x1[1] = pCOM1_3x1[1];
		des_pCOM_3x1[2] = pCOM1_3x1[2];

		des_pRF_3x1[1] = pRF1_3x1[1];
		des_pRF_3x1[2] = pRF1_3x1[2];
		des_pRF_3x1[3] = pRF1_3x1[3];
		
		des_pLF_3x1[1] = pLF1_3x1[1];
		des_pLF_3x1[2] = pLF1_3x1[2];
		des_pLF_3x1[3] = pLF1_3x1[3];
		
		des_qRF_4x1[1] = qRF1_4x1[1];
		des_qRF_4x1[2] = qRF1_4x1[2];
		des_qRF_4x1[3] = qRF1_4x1[3];
		des_qRF_4x1[4] = qRF1_4x1[4];

		des_qLF_4x1[1] = qLF1_4x1[1];
		des_qLF_4x1[2] = qLF1_4x1[2];
		des_qLF_4x1[3] = qLF1_4x1[3];
		des_qLF_4x1[4] = qLF1_4x1[4];

		des_pRH_3x1[1] = pRH1_3x1[1];
		des_pRH_3x1[2] = pRH1_3x1[2];
		des_pRH_3x1[3] = pRH1_3x1[3];
		
		des_pLH_3x1[1] = pLH1_3x1[1];
		des_pLH_3x1[2] = pLH1_3x1[2];
		des_pLH_3x1[3] = pLH1_3x1[3];
		
		des_qRH_4x1[1] = qRH1_4x1[1];
		des_qRH_4x1[2] = qRH1_4x1[2];
		des_qRH_4x1[3] = qRH1_4x1[3];
		des_qRH_4x1[4] = qRH1_4x1[4];

		des_qLH_4x1[1] = qLH1_4x1[1];
		des_qLH_4x1[2] = qLH1_4x1[2];
		des_qLH_4x1[3] = qLH1_4x1[3];
		des_qLH_4x1[4] = qLH1_4x1[4];

		_pCOM0_3x1[1] = pCOM1_3x1[1];
		_pCOM0_3x1[2] = pCOM1_3x1[2];

		_pRF0_3x1[1] = pRF1_3x1[1];
		_pRF0_3x1[2] = pRF1_3x1[2];
		_pRF0_3x1[3] = pRF1_3x1[3];

		_pLF0_3x1[1] = pLF1_3x1[1];
		_pLF0_3x1[2] = pLF1_3x1[2];
		_pLF0_3x1[3] = pLF1_3x1[3];

		_qRF0_4x1[1] = qRF1_4x1[1];
		_qRF0_4x1[2] = qRF1_4x1[2];
		_qRF0_4x1[3] = qRF1_4x1[3];
		_qRF0_4x1[4] = qRF1_4x1[4];

		_qLF0_4x1[1] = qLF1_4x1[1];
		_qLF0_4x1[2] = qLF1_4x1[2];
		_qLF0_4x1[3] = qLF1_4x1[3];
		_qLF0_4x1[4] = qLF1_4x1[4];

		_pRH0_3x1[1] = pRH1_3x1[1];
		_pRH0_3x1[2] = pRH1_3x1[2];
		_pRH0_3x1[3] = pRH1_3x1[3];

		_pLH0_3x1[1] = pLH1_3x1[1];
		_pLH0_3x1[2] = pLH1_3x1[2];
		_pLH0_3x1[3] = pLH1_3x1[3];

		_qRH0_4x1[1] = qRH1_4x1[1];
		_qRH0_4x1[2] = qRH1_4x1[2];
		_qRH0_4x1[3] = qRH1_4x1[3];
		_qRH0_4x1[4] = qRH1_4x1[4];

		_qLH0_4x1[1] = qLH1_4x1[1];
		_qLH0_4x1[2] = qLH1_4x1[2];
		_qLH0_4x1[3] = qLH1_4x1[3];
		_qLH0_4x1[4] = qLH1_4x1[4];

		override_flag = 0;
		override_RHR_flag = 0;
		override_LHR_flag = 0;
		dangRFz0 = 0.f;
		dangLFz0 = 0.f;
		RANK_loose_flag = 0;
		LANK_loose_flag = 0;
		land_ok_RF = 0;
		land_ok_LF = 0;
		land_ok_cnt_RF = 0;
		land_ok_cnt_LF = 0;
		count_RANK = 0;
		count_LANK = 0;
		count_RANK2 = 0;
		count_LANK2 = 0;
		walking_stage_old[0] = Walking_stage[0];
		walking_stage_old[3] = Walking_stage[3];
	}
	else if(pSharedMemory->biped_flag == 1) //walking pattern
	{
		//one_cos_orientation(0.5f, _qLF_4x1, _qRF_4x1, 1.f, ftemp1_7x1);
		//QTcross(ftemp1_7x1, qPEL1_4x1, des_qPEL_4x1);

		des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];
		
		des_pPELz = pPELz1;

		des_WST = qWST1;
		
		des_pCOM_3x1[1] = pCOM1_3x1[1]; // + _online_pattern.dpCOMx;
		des_pCOM_3x1[2] = pCOM1_3x1[2]; // + _online_pattern.dpCOMy;
		
		des_pRF_3x1[1] = pRF1_3x1[1] + _online_pattern.dpRFx;
		des_pRF_3x1[2] = pRF1_3x1[2] + _online_pattern.dpRFy;
		des_pRF_3x1[3] = pRF1_3x1[3] + _online_pattern.dpRFz;
		
		des_pLF_3x1[1] = pLF1_3x1[1] + _online_pattern.dpLFx;
		des_pLF_3x1[2] = pLF1_3x1[2] + _online_pattern.dpLFy;
		des_pLF_3x1[3] = pLF1_3x1[3] + _online_pattern.dpLFz;
		
		ftemp1_7x1[0] = _online_pattern.dangRFz;
		ftemp1_7x1[1] = 0.f;
		ftemp1_7x1[2] = 0.f;
		ftemp1_7x1[3] = 1.f;
		RV2QT(ftemp1_7x1, ftemp2_7x1);
		QTcross(qRF1_4x1, ftemp2_7x1, des_qRF_4x1);
		
		ftemp1_7x1[0] = _online_pattern.dangLFz;
		ftemp1_7x1[1] = 0.f;
		ftemp1_7x1[2] = 0.f;
		ftemp1_7x1[3] = 1.f;
		RV2QT(ftemp1_7x1, ftemp2_7x1);
		QTcross(qLF1_4x1, ftemp2_7x1, des_qLF_4x1);

		des_pRH_3x1[1] = pRH1_3x1[1];
		des_pRH_3x1[2] = pRH1_3x1[2];
		des_pRH_3x1[3] = pRH1_3x1[3];

		des_pLH_3x1[1] = pLH1_3x1[1];
		des_pLH_3x1[2] = pLH1_3x1[2];
		des_pLH_3x1[3] = pLH1_3x1[3];

		des_qRH_4x1[1] = qRH1_4x1[1];
		des_qRH_4x1[2] = qRH1_4x1[2];
		des_qRH_4x1[3] = qRH1_4x1[3];
		des_qRH_4x1[4] = qRH1_4x1[4];

		des_qLH_4x1[1] = qLH1_4x1[1];
		des_qLH_4x1[2] = qLH1_4x1[2];
		des_qLH_4x1[3] = qLH1_4x1[3];
		des_qLH_4x1[4] = qLH1_4x1[4];

		RSP_swing = (des_pRF_3x1[1]-des_pLF_3x1[1])*20.f*D2R/0.4f;
		LSP_swing = (des_pLF_3x1[1]-des_pRF_3x1[1])*20.f*D2R/0.4f;
		REB_swing = RSP_swing;
		LEB_swing = LSP_swing;

		if(Walking_stage[0] == 10)  //RF_1
			reset_RFT_flag = 0;
		else if(Walking_stage[0] == 20 && reset_RFT_flag ==0)
		{
			NullFTSensor(0, 0x00);
			reset_RFT_flag = 1;
		}

		if(Walking_stage[3] == 10) //LF_1
			reset_LFT_flag = 0;
		else if(Walking_stage[3] == 20 && reset_LFT_flag ==0)
		{
			NullFTSensor(1, 0x00);
			reset_LFT_flag = 1;
		}
		//---------------- RANK				
		/*if(Walking_stage[0] == 0)
		{
			if(land_ok_RF == 0 && OneKG_ROK_Flag == 1 && (RANK_loose_flag == 0 || RANK_loose_flag == 3) )
			{
				RBsetSwitchingMode(CAN0, Joint[RAP].JMC, SW_MODE_NON_COMPLEMENTARY);
				RBgainOverrideHR(CAN0, Joint[RAP].JMC, 0, 0, 1);			
				RBenableFrictionCompensation(CAN0, Joint[RAP].JMC, 1, 1);								
				count_RANK2 = 0;
				land_ok_cnt_RF = 0;
				RANK_loose_flag = 4;				
			}
		}
		else if(Walking_stage[0] == 20)
		{
			if(walking_stage_old[0] == 10)
			{
				land_ok_RF = 0;
				land_ok_cnt_RF = 0;
			}
			if(OneKG_ROK_Flag == 1 && (RANK_loose_flag == 0 || RANK_loose_flag == 3) )
			{
				RBsetSwitchingMode(CAN0, Joint[RAP].JMC, SW_MODE_NON_COMPLEMENTARY);
				RBgainOverrideHR(CAN0, Joint[RAP].JMC, 200, 0, 1);			
				RBenableFrictionCompensation(CAN0, Joint[RAP].JMC, 1, 1);								
				count_RANK2 = 0;
				RANK_loose_flag = 1;				
			}
		}

		if(RANK_loose_flag == 1)
		{
			meas_Q_34x1[RHY_34] = _online_pattern.dangRFz;
			meas_Q_34x1[RAP_34] = ((float)Joint[RAP].EncoderValue)/Joint[RAP].PPR*D2R - _online_pattern.offset_RAP;
			meas_Q_34x1[RAR_34] = ((float)Joint[RAR].EncoderValue)/Joint[RAR].PPR*D2R - _online_pattern.offset_RAR;

			FKine_Foot(LOCAL, meas_Q_34x1, ftemp1_7x1, des_qRF_4x1, ftemp2_7x1, ftemp2_7x1);
			pRF_offset_3x1[1] =  ftemp1_7x1[1]-_pRF_L_3x1[1];
			pRF_offset_3x1[2] =  ftemp1_7x1[2]-_pRF_L_3x1[2];
			pRF_offset_3x1[3] =  ftemp1_7x1[3]-_pRF_L_3x1[3];

			if(land_ok_RF == 1 || count_RANK2 > 15)
			{
				for(i=1;i<=4;i++)
					qRF2_4x1[i] = des_qRF_4x1[i];
				dangRFz0 = _online_pattern.dangRFz;
				RBenableFrictionCompensation(CAN0, Joint[RAP].JMC, 0, 0);				
				RBgainOverrideHR(CAN0, Joint[RAP].JMC, 1000, 1000, 5);
				RBsetSwitchingMode(CAN0, Joint[RAP].JMC, SW_MODE_COMPLEMENTARY);
				RANK_loose_flag = 2;
			}		
			des_pRF_3x1[1] += pRF_offset_3x1[1];
			des_pRF_3x1[2] += pRF_offset_3x1[2];
			des_pRF_3x1[3] += pRF_offset_3x1[3];
			
			count_RANK2++;
		}
		else if(RANK_loose_flag == 2)
		{
			RZ(_online_pattern.dangRFz-dangRFz0, _TEMP1_34x34);
			DC2QT((const float**)_TEMP1_34x34, ftemp1_7x1);
			QTcross(ftemp1_7x1,qRF2_4x1, des_qRF_4x1);

			des_pRF_3x1[1] += pRF_offset_3x1[1];
			des_pRF_3x1[2] += pRF_offset_3x1[2];
			des_pRF_3x1[3] += pRF_offset_3x1[3];

			if(Walking_stage[0] == 10)
			{
				for(i=1;i<=4;i++)
					qRF2_4x1[i] = des_qRF_4x1[i];
				RANK_loose_flag = 3;
				count_RANK = 0;
			}
		}
		else if(RANK_loose_flag == 3)
		{
			one_cos_orientation((float)count_RANK*DT, qRF2_4x1, des_qRF_4x1, 0.3f, ftemp1_7x1);
			for(i=1;i<=4;i++)
				des_qRF_4x1[i] = ftemp1_7x1[i];				

			ftemp1_7x1[0] = (1.f-one_cos((float)count_RANK*DT,1.f,0.3f));
			des_pRF_3x1[1] += pRF_offset_3x1[1]*ftemp1_7x1[0];
			des_pRF_3x1[2] += pRF_offset_3x1[2]*ftemp1_7x1[0];
			des_pRF_3x1[3] += pRF_offset_3x1[3]*ftemp1_7x1[0];

			count_RANK++;
			if(count_RANK*DT > 0.3f+DT)
				count_RANK--;
		}
		else if(RANK_loose_flag == 4)
		{
			if(land_ok_RF == 1 || count_RANK2 > 15)
			{
				RBenableFrictionCompensation(CAN0, Joint[RAP].JMC, 0, 0);				
				RBgainOverrideHR(CAN0, Joint[RAP].JMC, 1000, 1000, 5);
				RBsetSwitchingMode(CAN0, Joint[RAP].JMC, SW_MODE_COMPLEMENTARY);
				RANK_loose_flag = 0;
			}		
			count_RANK2++;
		}
		
		//---------------- LANK		
		if(Walking_stage[3] == 0)
		{
			if(land_ok_LF == 0 && OneKG_LOK_Flag == 1 && (LANK_loose_flag == 0 || LANK_loose_flag == 3) )
			{
				RBsetSwitchingMode(CAN0, Joint[LAP].JMC, SW_MODE_NON_COMPLEMENTARY);
				RBgainOverrideHR(CAN0, Joint[LAP].JMC, 0, 0, 1);			
				RBenableFrictionCompensation(CAN0, Joint[LAP].JMC, 1, 1);								
				count_LANK2 = 0;
				land_ok_cnt_LF = 0;
				LANK_loose_flag = 4;				
			}
		}
		else if(Walking_stage[3] == 20)
		{
			if(walking_stage_old[3] == 10)
			{
				land_ok_LF = 0;
				land_ok_cnt_LF = 0;
			}
			if(OneKG_LOK_Flag == 1 && (LANK_loose_flag == 0 || LANK_loose_flag == 3) )
			{
				RBsetSwitchingMode(CAN0, Joint[LAP].JMC, SW_MODE_NON_COMPLEMENTARY);
				RBgainOverrideHR(CAN0, Joint[LAP].JMC, 200, 0, 1);			
				RBenableFrictionCompensation(CAN0, Joint[LAP].JMC, 1, 1);								
				count_LANK2 = 0;
				LANK_loose_flag = 1;				
			}
		}

		if(LANK_loose_flag == 1)
		{
			meas_Q_34x1[LHY_34] = _online_pattern.dangLFz;
			meas_Q_34x1[LAP_34] = ((float)Joint[LAP].EncoderValue)/Joint[LAP].PPR*D2R - _online_pattern.offset_LAP;
			meas_Q_34x1[LAR_34] = ((float)Joint[LAR].EncoderValue)/Joint[LAR].PPR*D2R - _online_pattern.offset_LAR;

			FKine_Foot(LOCAL, meas_Q_34x1, ftemp2_7x1, ftemp2_7x1, ftemp1_7x1, des_qLF_4x1);
			pLF_offset_3x1[1] =  ftemp1_7x1[1]-_pLF_L_3x1[1];
			pLF_offset_3x1[2] =  ftemp1_7x1[2]-_pLF_L_3x1[2];
			pLF_offset_3x1[3] =  ftemp1_7x1[3]-_pLF_L_3x1[3];

			if(land_ok_LF == 1 || count_LANK2 > 15)
			{
				for(i=1;i<=4;i++)
					qLF2_4x1[i] = des_qLF_4x1[i];
				dangLFz0 = _online_pattern.dangLFz;
				RBenableFrictionCompensation(CAN0, Joint[LAP].JMC, 0, 0);				
				RBgainOverrideHR(CAN0, Joint[LAP].JMC, 1000, 1000, 5);
				RBsetSwitchingMode(CAN0, Joint[LAP].JMC, SW_MODE_COMPLEMENTARY);
				LANK_loose_flag = 2;
			}		
			des_pLF_3x1[1] += pLF_offset_3x1[1];
			des_pLF_3x1[2] += pLF_offset_3x1[2];
			des_pLF_3x1[3] += pLF_offset_3x1[3];
			
			count_LANK2++;
		}
		else if(LANK_loose_flag == 2)
		{
			RZ(_online_pattern.dangLFz-dangLFz0, _TEMP1_34x34);
			DC2QT((const float**)_TEMP1_34x34, ftemp1_7x1);
			QTcross(ftemp1_7x1,qLF2_4x1, des_qLF_4x1);

			des_pLF_3x1[1] += pLF_offset_3x1[1];
			des_pLF_3x1[2] += pLF_offset_3x1[2];
			des_pLF_3x1[3] += pLF_offset_3x1[3];

			if(Walking_stage[3] == 10)
			{
				for(i=1;i<=4;i++)
					qLF2_4x1[i] = des_qLF_4x1[i];
				LANK_loose_flag = 3;
				count_LANK = 0;
			}
		}
		else if(LANK_loose_flag == 3)
		{
			one_cos_orientation((float)count_LANK*DT, qLF2_4x1, des_qLF_4x1, 0.3f, ftemp1_7x1);
			for(i=1;i<=4;i++)
				des_qLF_4x1[i] = ftemp1_7x1[i];				

			ftemp1_7x1[0] = (1.f-one_cos((float)count_LANK*DT,1.f,0.3f));
			des_pLF_3x1[1] += pLF_offset_3x1[1]*ftemp1_7x1[0];
			des_pLF_3x1[2] += pLF_offset_3x1[2]*ftemp1_7x1[0];
			des_pLF_3x1[3] += pLF_offset_3x1[3]*ftemp1_7x1[0];

			count_LANK++;
			if(count_LANK*DT > 0.3f+DT)
				count_LANK--;
		}
		else if(LANK_loose_flag == 4)
		{
			if(land_ok_LF == 1 || count_LANK2 > 15)
			{
				RBenableFrictionCompensation(CAN0, Joint[LAP].JMC, 0, 0);				
				RBgainOverrideHR(CAN0, Joint[LAP].JMC, 1000, 1000, 5);
				RBsetSwitchingMode(CAN0, Joint[LAP].JMC, SW_MODE_COMPLEMENTARY);
				LANK_loose_flag = 0;
			}		
			count_LANK2++;
		}
		
		walking_stage_old[0] = Walking_stage[0];
		walking_stage_old[3] = Walking_stage[3];
		
		_log_temp[31] = RANK_loose_flag;
		_log_temp[32] = des_qRF_4x1[1];
		_log_temp[33] = des_qRF_4x1[2];
		_log_temp[34] = des_qRF_4x1[3];
		_log_temp[35] = des_qRF_4x1[4];
		
		_log_temp[40] = _Q_34x1[RAP_34];
		_log_temp[41] = _Q_34x1[RAR_34];
		_log_temp[42] = meas_Q_34x1[RAP_34];
		_log_temp[43] = meas_Q_34x1[RAR_34];
		_log_temp[44] = count_RANK;

		_log_temp[45] = LANK_loose_flag;
		_log_temp[46] = des_qLF_4x1[1];
		_log_temp[47] = des_qLF_4x1[2];
		_log_temp[48] = des_qLF_4x1[3];
		_log_temp[49] = des_qLF_4x1[4];
		
		_log_temp[50] = _Q_34x1[LAP_34];
		_log_temp[51] = _Q_34x1[LAR_34];
		_log_temp[52] = meas_Q_34x1[LAP_34];
		_log_temp[53] = meas_Q_34x1[LAR_34];
		_log_temp[54] = count_LANK;

		_log_temp[59] = pRF_offset_3x1[1];
		_log_temp[60] = pRF_offset_3x1[2];
		_log_temp[61] = pRF_offset_3x1[3];

		_log_temp[62] = des_pRF_3x1[1];
		_log_temp[63] = des_pRF_3x1[2];
		_log_temp[64] = des_pRF_3x1[3];
		*/
		//-------------
		if(pSharedMemory->stop_online_biped_flag == 1) // stop Kirk's online pattern generator
		{
			RBgainOverrideHR(Joint[WST].CAN_channel, Joint[WST].JMC, 1000, 1000, 3000);
			RBgainOverrideHR(Joint[RSP].CAN_channel, Joint[RSP].JMC, 1000, 1000, 3000);
			RBgainOverrideHR(Joint[LSP].CAN_channel, Joint[LSP].JMC, 1000, 1000, 3000);
			RBgainOverrideHR(Joint[RSY].CAN_channel, Joint[RSY].JMC, 1000, 1000, 3000);
			RBgainOverrideHR(Joint[LSY].CAN_channel, Joint[LSY].JMC, 1000, 1000, 3000);
			RBsetSwitchingMode(CAN1, Joint[WST].JMC, SW_MODE_COMPLEMENTARY);
			RBsetSwitchingMode(CAN1, Joint[RSP].JMC, SW_MODE_COMPLEMENTARY);
			RBsetSwitchingMode(CAN1, Joint[LSP].JMC, SW_MODE_COMPLEMENTARY);
			RBsetSwitchingMode(CAN1, Joint[RSY].JMC, SW_MODE_COMPLEMENTARY);
			RBsetSwitchingMode(CAN1, Joint[LSY].JMC, SW_MODE_COMPLEMENTARY);

			RBenableFrictionCompensation(CAN0, Joint[WST].JMC, 0, 0);
			RBenableFrictionCompensation(CAN1, Joint[RSP].JMC, 0, 0);
			RBenableFrictionCompensation(CAN1, Joint[LSP].JMC, 0, 0);
			RBenableFrictionCompensation(CAN1, Joint[RSY].JMC, 0, 0);
			RBenableFrictionCompensation(CAN1, Joint[LSY].JMC, 0, 0);

			RBenableFrictionCompensation(CAN0, Joint[LAP].JMC, 0, 0);				
			RBgainOverrideHR(CAN0, Joint[LAP].JMC, 1000, 1000, 1000);
			RBsetSwitchingMode(CAN0, Joint[LAP].JMC, SW_MODE_COMPLEMENTARY);

			RBenableFrictionCompensation(CAN0, Joint[RAP].JMC, 0, 0);				
			RBgainOverrideHR(CAN0, Joint[RAP].JMC, 1000, 1000, 1000);
			RBsetSwitchingMode(CAN0, Joint[RAP].JMC, SW_MODE_COMPLEMENTARY);
		
			override_flag = 0;
			t = 0.f;
			update_pos1_flag = 1;
			pSharedMemory->biped_flag = 0;
			pSharedMemory->stop_online_biped_flag = 0;
			pSharedMemory->biped_flag = 0;
			printf("online pattern generator is off");
		}
		else
		{
			if(pSharedMemory->ON_compl_arm_flag == 1)
			{
				RBgainOverrideHR(Joint[WST].CAN_channel, Joint[WST].JMC, 10, 0, 1000);
				RBgainOverrideHR(Joint[RSP].CAN_channel, Joint[RSP].JMC, 40, 10, 1000);
				RBgainOverrideHR(Joint[LSP].CAN_channel, Joint[LSP].JMC, 40, 10, 1000);
				RBgainOverrideHR(Joint[RSY].CAN_channel, Joint[RSY].JMC, 40, 20, 1000);
				RBgainOverrideHR(Joint[LSY].CAN_channel, Joint[LSY].JMC, 40, 20, 1000);
				RBsetSwitchingMode(CAN0, Joint[WST].JMC, SW_MODE_NON_COMPLEMENTARY);
				RBsetSwitchingMode(CAN1, Joint[RSP].JMC, SW_MODE_NON_COMPLEMENTARY);
				RBsetSwitchingMode(CAN1, Joint[LSP].JMC, SW_MODE_NON_COMPLEMENTARY);
				RBsetSwitchingMode(CAN1, Joint[RSY].JMC, SW_MODE_NON_COMPLEMENTARY);
				RBsetSwitchingMode(CAN1, Joint[LSY].JMC, SW_MODE_NON_COMPLEMENTARY);

				RBenableFrictionCompensation(CAN0, Joint[WST].JMC, 1, 0);
				RBenableFrictionCompensation(CAN1, Joint[RSP].JMC, 1, 1);
				RBenableFrictionCompensation(CAN1, Joint[LSP].JMC, 1, 1);
				RBenableFrictionCompensation(CAN1, Joint[RSY].JMC, 1, 1);
				RBenableFrictionCompensation(CAN1, Joint[LSY].JMC, 1, 1);

				pSharedMemory->ON_compl_arm_flag = -1;

			}
			else if(pSharedMemory->ON_compl_arm_flag == 0)
			{
				RBenableFrictionCompensation(CAN0, Joint[WST].JMC, 0, 0);
				RBenableFrictionCompensation(CAN1, Joint[RSP].JMC, 0, 0);
				RBenableFrictionCompensation(CAN1, Joint[LSP].JMC, 0, 0);
				RBenableFrictionCompensation(CAN1, Joint[RSY].JMC, 0, 0);
				RBenableFrictionCompensation(CAN1, Joint[LSY].JMC, 0, 0);

				RBgainOverrideHR(Joint[WST].CAN_channel, Joint[WST].JMC, 1000, 0, 3000);
				RBgainOverrideHR(Joint[RSP].CAN_channel, Joint[RSP].JMC, 1000, 1000, 3000);
				RBgainOverrideHR(Joint[LSP].CAN_channel, Joint[LSP].JMC, 1000, 1000, 3000);
				RBgainOverrideHR(Joint[RSY].CAN_channel, Joint[RSY].JMC, 1000, 1000, 3000);
				RBgainOverrideHR(Joint[LSY].CAN_channel, Joint[LSY].JMC, 1000, 1000, 3000);
				RBsetSwitchingMode(CAN0, Joint[WST].JMC, SW_MODE_COMPLEMENTARY);
				RBsetSwitchingMode(CAN1, Joint[RSP].JMC, SW_MODE_COMPLEMENTARY);
				RBsetSwitchingMode(CAN1, Joint[LSP].JMC, SW_MODE_COMPLEMENTARY);
				RBsetSwitchingMode(CAN1, Joint[RSY].JMC, SW_MODE_COMPLEMENTARY);
				RBsetSwitchingMode(CAN1, Joint[LSY].JMC, SW_MODE_COMPLEMENTARY);

				pSharedMemory->ON_compl_arm_flag = -1;
			}
		}
	}

	RSP_swing *= 0.99f;
	LSP_swing *= 0.99f;
	REB_swing *= 0.99f;
	LEB_swing *= 0.99f;

	if(fabs(RSP_swing) < 0.1*PI/180)
		RSP_swing = 0.f;
	if(fabs(LSP_swing) < 0.1*PI/180)
		LSP_swing = 0.f;
	if(fabs(REB_swing) < 0.1*PI/180)
		REB_swing = 0.f;
	if(fabs(LEB_swing) < 0.1*PI/180)
		LEB_swing = 0.f;

	if(update_pos1_flag == 1)
	{
		qPEL1_4x1[1] = des_qPEL_4x1[1];
		qPEL1_4x1[2] = des_qPEL_4x1[2];
		qPEL1_4x1[3] = des_qPEL_4x1[3];
		qPEL1_4x1[4] = des_qPEL_4x1[4];

		pPELz1 = des_pPELz;

		qWST1 = des_WST;

		pCOM1_3x1[1] = des_pCOM_3x1[1];
		pCOM1_3x1[2] = des_pCOM_3x1[2];

		pRF1_3x1[1] = des_pRF_3x1[1];
		pRF1_3x1[2] = des_pRF_3x1[2];
		pRF1_3x1[3] = des_pRF_3x1[3];

		qRF1_4x1[1] = des_qRF_4x1[1];
		qRF1_4x1[2] = des_qRF_4x1[2];
		qRF1_4x1[3] = des_qRF_4x1[3];
		qRF1_4x1[4] = des_qRF_4x1[4];

		pLF1_3x1[1] = des_pLF_3x1[1];
		pLF1_3x1[2] = des_pLF_3x1[2];
		pLF1_3x1[3] = des_pLF_3x1[3];

		qLF1_4x1[1] = des_qLF_4x1[1];
		qLF1_4x1[2] = des_qLF_4x1[2];
		qLF1_4x1[3] = des_qLF_4x1[3];
		qLF1_4x1[4] = des_qLF_4x1[4];

		pRH1_3x1[1] = des_pRH_3x1[1];
		pRH1_3x1[2] = des_pRH_3x1[2];
		pRH1_3x1[3] = des_pRH_3x1[3];

		qRH1_4x1[1] = des_qRH_4x1[1];
		qRH1_4x1[2] = des_qRH_4x1[2];
		qRH1_4x1[3] = des_qRH_4x1[3];
		qRH1_4x1[4] = des_qRH_4x1[4];

		pLH1_3x1[1] = des_pLH_3x1[1];
		pLH1_3x1[2] = des_pLH_3x1[2];
		pLH1_3x1[3] = des_pLH_3x1[3];

		qLH1_4x1[1] = des_qLH_4x1[1];
		qLH1_4x1[2] = des_qLH_4x1[2];
		qLH1_4x1[3] = des_qLH_4x1[3];
		qLH1_4x1[4] = des_qLH_4x1[4];
	}
	
	des_WSTp = 0.f;

	des_vPELz = 0.f;

	des_pCOM_3x1[3] = pCOM1_3x1[3];
	des_vCOM_3x1[1] = 0.;
	des_vCOM_3x1[2] = 0.; 
	des_vCOM_3x1[3] = 0.;

	des_vRF_3x1[1] = 0.;
	des_vRF_3x1[2] = 0.;
	des_vRF_3x1[3] = 0.; 	
	des_wRF_3x1[1] = 0.;
	des_wRF_3x1[2] = 0.;
	des_wRF_3x1[3] = 0.;

	des_vLF_3x1[1] = 0.;
	des_vLF_3x1[2] = 0.;
	des_vLF_3x1[3] = 0.;
	des_wLF_3x1[1] = 0.;
	des_wLF_3x1[2] = 0.;
	des_wLF_3x1[3] = 0.;
	
	des_vRH_3x1[1] = 0.;
	des_vRH_3x1[2] = 0.;
	des_vRH_3x1[3] = 0.; 	
	des_wRH_3x1[1] = 0.;
	des_wRH_3x1[2] = 0.;
	des_wRH_3x1[3] = 0.;

	des_vLH_3x1[1] = 0.;
	des_vLH_3x1[2] = 0.;
	des_vLH_3x1[3] = 0.;
	des_wLH_3x1[1] = 0.;
	des_wLH_3x1[2] = 0.;
	des_wLH_3x1[3] = 0.;

	//------------------------------ compensation for hip roll joints
	/*lL = des_pLF_3x1[2] - des_pCOM_3x1[2];
	lR = des_pCOM_3x1[2] - des_pRF_3x1[2];
	fRFz = lL*m_TOTAL*9.81f/(lR+lL);
	fLFz = lR*m_TOTAL*9.81f/(lR+lL);

	if(lL < lR)
	{
		lhr_compen = (0.5f*m_TOTAL*9.81f-fRFz)/K_PELVIS_ROLL;
		rhr_compen = 0.;
	}
	else
	{
		lhr_compen = 0.;
		rhr_compen = -(0.5f*m_TOTAL*9.81f-fLFz)/K_PELVIS_ROLL;
	}*/
	//-------------------------------------

	//--------- Pelvis attitude	
	QTdel(des_qPEL_4x1, &_Q_34x1[3], ftemp3_7x1);
	Xpel_3x1[1] = pSharedMemory->kd[9]*(-_Qp_33x1[4]) + pSharedMemory->kp[9]*ftemp3_7x1[1];
	Xpel_3x1[2] = pSharedMemory->kd[9]*(-_Qp_33x1[5]) + pSharedMemory->kp[9]*ftemp3_7x1[2];
	Xpel_3x1[3] = pSharedMemory->kd[9]*(-_Qp_33x1[6]) + pSharedMemory->kp[9]*ftemp3_7x1[3];
	//-----------------	

	//------------------ Waist
	RVALS(_Q_34x1[WST_34],  _Qp_33x1[WST_33],  pSharedMemory->kd[10]*(des_WSTp-_Qp_33x1[WST_33])+pSharedMemory->kp[10]*(des_WST-_Q_34x1[WST_34]),   WSTpmax, WSTppmax, WSTmin, WSTmax, D2R, &Xwst, &ftemp1_7x1[0], &ftemp1_7x1[1]);
	//-----------------

	//--------- Pelvis height
	Xpelz = pSharedMemory->kd[11]*(des_vPELz-_Qp_33x1[3]) + pSharedMemory->kp[11]*(des_pPELz-_Q_34x1[3]);
	//-----------------
	
	//------------ Xcom				
	mult_mv((const float**)_jCOM_3x33,3,33, _Qp_33x1, vCOM_3x1);	
	ftemp2_7x1[1] = pSharedMemory->kd[12]*(des_vCOM_3x1[1]-vCOM_3x1[1]) + pSharedMemory->kp[12]*(des_pCOM_3x1[1]-_pCOM_3x1[1]);
	ftemp2_7x1[2] = pSharedMemory->kd[12]*(des_vCOM_3x1[2]-vCOM_3x1[2]) + pSharedMemory->kp[12]*(des_pCOM_3x1[2]-_pCOM_3x1[2]);
	ftemp2_7x1[3] = pSharedMemory->kd[12]*(des_vCOM_3x1[3]-vCOM_3x1[3]) + pSharedMemory->kp[12]*(des_pCOM_3x1[3]-_pCOM_3x1[3]);
	mult_mv((const float**)_jCOMp_3x33, 3,33, _Qp_33x1, ftemp1_7x1);
	Xcom_3x1[1] = ftemp2_7x1[1] - ftemp1_7x1[1];
	Xcom_3x1[2] = ftemp2_7x1[2] - ftemp1_7x1[2];
	Xcom_3x1[3] = ftemp2_7x1[3] - ftemp1_7x1[3];
	//-----------------

	//------------ Foot
	for(i=1;i<=6;i++)
		for(j=1;j<=6;j++)
		{
			_jRF_6x33[i][j] = 0.f;
			_jLF_6x33[i][j] = 0.f;
			_jRFp_6x33[i][j] = 0.f;
			_jLFp_6x33[i][j] = 0.f;
		}
	diff_vv(des_pRF_3x1,3, _pRF_L_3x1, ftemp3_7x1);
	mult_mv((const float**)_jRF_6x33,6,33, _Qp_33x1, ftemp2_7x1);
	diff_vv(des_vRF_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[13], ftemp3_7x1,3, pSharedMemory->kd[13],ftemp1_7x1, ftemp3_7x1);
	mult_mv((const float**)_jRFp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xrf_6x1);
	
	QTdel(des_qRF_4x1, _qRF_L_4x1, ftemp3_7x1);	
	diff_vv(des_wRF_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(pSharedMemory->kp[13],ftemp3_7x1,3, pSharedMemory->kd[13],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xrf_6x1[3]);
	
	diff_vv(des_pLF_3x1,3, _pLF_L_3x1, ftemp3_7x1);
	mult_mv((const float**)_jLF_6x33,6,33, _Qp_33x1, ftemp2_7x1);
	diff_vv(des_vLF_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[13], ftemp3_7x1,3, pSharedMemory->kd[13],ftemp1_7x1, ftemp3_7x1);
	mult_mv((const float**)_jLFp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xlf_6x1);

	QTdel(des_qLF_4x1, _qLF_L_4x1, ftemp3_7x1);	
	diff_vv(des_wLF_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(pSharedMemory->kp[13],ftemp3_7x1,3, pSharedMemory->kd[13],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xlf_6x1[3]);
	//-----------------

	
	//------------ Hand
	for(i=1;i<=6;i++)
		for(j=1;j<=6;j++)
		{
			_jRH_6x33[i][j] = 0.f;
			_jLH_6x33[i][j] = 0.f;
			_jRHp_6x33[i][j] = 0.f;
			_jLHp_6x33[i][j] = 0.f;
		}
	diff_vv(des_pRH_3x1,3, _pRH_L_3x1, ftemp3_7x1);
	mult_mv((const float**)_jRH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
	diff_vv(des_vRH_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[14], ftemp3_7x1,3, pSharedMemory->kd[14],ftemp1_7x1, ftemp3_7x1);
	mult_mv((const float**)_jRHp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xrh_6x1);

	QTdel(des_qRH_4x1, _qRH_L_4x1, ftemp3_7x1);	
	diff_vv(des_wRH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(pSharedMemory->kp[14],ftemp3_7x1,3, pSharedMemory->kd[14],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xrh_6x1[3]);
	
	diff_vv(des_pLH_3x1,3, _pLH_L_3x1, ftemp3_7x1);  
	mult_mv((const float**)_jLH_6x33,6,33, _Qp_33x1, ftemp2_7x1);  
	diff_vv(des_vLH_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[14], ftemp3_7x1,3, pSharedMemory->kd[14],ftemp1_7x1, ftemp3_7x1);  
	mult_mv((const float**)_jLHp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	  
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xlh_6x1);

	QTdel(des_qLH_4x1, _qLH_L_4x1, ftemp3_7x1);	
	diff_vv(des_wLH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(pSharedMemory->kp[14],ftemp3_7x1,3, pSharedMemory->kd[14],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xlh_6x1[3]);
	//-----------------
	
	X1_33x1[1] = Xrf_6x1[1];
	X1_33x1[2] = Xrf_6x1[2];
	X1_33x1[3] = Xrf_6x1[3];
	X1_33x1[4] = Xrf_6x1[4];
	X1_33x1[5] = Xrf_6x1[5];
	X1_33x1[6] = Xrf_6x1[6];
	X1_33x1[7] = Xlf_6x1[1];
	X1_33x1[8] = Xlf_6x1[2];
	X1_33x1[9] = Xlf_6x1[3];
	X1_33x1[10] = Xlf_6x1[4];
	X1_33x1[11] = Xlf_6x1[5];
	X1_33x1[12] = Xlf_6x1[6];
	X1_33x1[13] = Xrh_6x1[1];
	X1_33x1[14] = Xrh_6x1[2];
	X1_33x1[15] = Xrh_6x1[3];
	X1_33x1[16] = Xrh_6x1[4];
	X1_33x1[17] = Xrh_6x1[5];
	X1_33x1[18] = Xrh_6x1[6];	
	X1_33x1[19] = Xlh_6x1[1];
	X1_33x1[20] = Xlh_6x1[2];
	X1_33x1[21] = Xlh_6x1[3];
	X1_33x1[22] = Xlh_6x1[4];
	X1_33x1[23] = Xlh_6x1[5];
	X1_33x1[24] = Xlh_6x1[6];
	X1_33x1[25] = Xcom_3x1[1];
	X1_33x1[26] = Xcom_3x1[2];	
	X1_33x1[27] = Xpelz;
	X1_33x1[28] = Xpel_3x1[1];
	X1_33x1[29] = Xpel_3x1[2];
	X1_33x1[30] = Xpel_3x1[3];
	X1_33x1[31] = Xwst;
	dim_primary_task = 31;
	
	//----------------- redundant tasks	
	RVALS(_Q_34x1[RSY_34],  _Qp_33x1[RSY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RSY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RSY_34]-_Q_34x1[RSY_34]),   RSYpmax, RSYppmax, RSYmin, RSYmax, D2R, &X2_33x1[1], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSY_34],  _Qp_33x1[LSY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LSY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LSY_34]-_Q_34x1[LSY_34]),   LSYpmax, LSYppmax, LSYmin, LSYmax, D2R, &X2_33x1[2], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	//RVALS(_Q_34x1[WST_34],  _Qp_33x1[WST_33],  pSharedMemory->kd[15]*(-_Qp_33x1[WST_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[WST_34]-_Q_34x1[WST_34]),   WSTpmax, WSTppmax, WSTmin, WSTmax, D2R, &X2_33x1[3], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	//X2_33x1[16] = Xpel_3x1[1];
	//X2_33x1[17] = Xpel_3x1[2];
	//X2_33x1[18] = Xpel_3x1[3];
	dim_redundant_task = 2;
	//--------------------

	for(i=1; i<=33; i++)
	{
		_jT1_33x33[1][i] = _jRF_6x33[1][i];
		_jT1_33x33[2][i] = _jRF_6x33[2][i];
		_jT1_33x33[3][i] = _jRF_6x33[3][i];
		_jT1_33x33[4][i] = _jRF_6x33[4][i];
		_jT1_33x33[5][i] = _jRF_6x33[5][i];
		_jT1_33x33[6][i] = _jRF_6x33[6][i];			

		_jT1_33x33[7][i] = _jLF_6x33[1][i];
		_jT1_33x33[8][i] = _jLF_6x33[2][i];
		_jT1_33x33[9][i] = _jLF_6x33[3][i];
		_jT1_33x33[10][i] = _jLF_6x33[4][i];
		_jT1_33x33[11][i] = _jLF_6x33[5][i];
		_jT1_33x33[12][i] = _jLF_6x33[6][i];

		_jT1_33x33[13][i] = _jRH_6x33[1][i];
		_jT1_33x33[14][i] = _jRH_6x33[2][i];
		_jT1_33x33[15][i] = _jRH_6x33[3][i];
		_jT1_33x33[16][i] = _jRH_6x33[4][i];
		_jT1_33x33[17][i] = _jRH_6x33[5][i];
		_jT1_33x33[18][i] = _jRH_6x33[6][i];

		_jT1_33x33[19][i] = _jLH_6x33[1][i];			
		_jT1_33x33[20][i] = _jLH_6x33[2][i];			
		_jT1_33x33[21][i] = _jLH_6x33[3][i];			
		_jT1_33x33[22][i] = _jLH_6x33[4][i];			
		_jT1_33x33[23][i] = _jLH_6x33[5][i];			
		_jT1_33x33[24][i] = _jLH_6x33[6][i];			

		_jT1_33x33[25][i] = _jCOM_3x33[1][i];
		_jT1_33x33[26][i] = _jCOM_3x33[2][i];

		_jT1_33x33[27][i] = 0.f;
		_jT1_33x33[28][i] = 0.f;
		_jT1_33x33[29][i] = 0.f;
		_jT1_33x33[30][i] = 0.f;   
		_jT1_33x33[31][i] = 0.f;   
		
		for(j=1;j<=dim_redundant_task;j++)
			_jT2_33x33[j][i] = 0.f;
	}
	_jT1_33x33[27][3] = 1.f;
    _jT1_33x33[28][4] = 1.f;
    _jT1_33x33[29][5] = 1.f;
    _jT1_33x33[30][6] = 1.f;
	_jT1_33x33[31][7] = 1.f;
	
	for(i=1; i<=26; i++)
	{
		_jT1_33x33[i][4] = 0.f;  // exclude pelvis orientation from the solution
		_jT1_33x33[i][5] = 0.f; 
		_jT1_33x33[i][6] = 0.f; 
	}
	
	for(i=13; i<=dim_primary_task; i++)
		_jT1_33x33[i][WST_33] = 0.f;  // exclude WST from the solution
	_jT1_33x33[31][7] = 1.f;

	for(j=20; j<=33; j++)		//// exclude UB from the COM solution
	{
		_jT1_33x33[25][j] = 0.f;
		_jT1_33x33[26][j] = 0.f;
	}
	_jT1_33x33[25][7] = 0.f;
	_jT1_33x33[26][7] = 0.f;

	_jT2_33x33[1][RSY_33] = 1.f;
	_jT2_33x33[2][LSY_33] = 1.f;
	//_jT2_33x33[3][WST_33] = 1.f;
		
	for(i=1; i<=33; i++)
	{
		_jT1_33x33[i][WX_33] *= WEIGHT_PEL_SQRT_INV;
		_jT1_33x33[i][WY_33] *= WEIGHT_PEL_SQRT_INV;
		_jT1_33x33[i][WZ_33] *= WEIGHT_PEL_SQRT_INV;
		_jT1_33x33[i][WST_33] *= WEIGHT_WST_SQRT_INV;

		_jT2_33x33[i][WX_33] *= WEIGHT_PEL_SQRT_INV;
		_jT2_33x33[i][WY_33] *= WEIGHT_PEL_SQRT_INV;
		_jT2_33x33[i][WZ_33] *= WEIGHT_PEL_SQRT_INV;			
		_jT2_33x33[i][WST_33] *= WEIGHT_WST_SQRT_INV;
	}
	
	if(pinv_SR((const float**)_jT1_33x33, dim_primary_task, 33, (float)1.e-10, _jT1inv_33x33) != 0)
		return -2; // singularity occurred
	mult_mm((const float**)_jT1inv_33x33,33,dim_primary_task, (const float**)_jT1_33x33, 33, _TEMP1_34x34);
	diff_mm((const float**)_EYE_33, 33,33, (const float**)_TEMP1_34x34, _N1_33x33);	

	//pinv_svd((const float**)_jT2_33x33,dim_redundant_task,33, _jT2inv_33x33);
	trans(1.f, (const float**)_jT2_33x33, dim_redundant_task,33, _jT2inv_33x33);

	//mult_mv((const float**)_jT1inv_33x33,33,dim_primary_task, (const float*)X1_33x1, Qpp_33x1);
	mult_mv((const float**)_jT1inv_33x33,33,dim_primary_task, (const float*)X1_33x1, _TEMP1_34x34[1]);
	mult_mv((const float**)_jT2inv_33x33,33,dim_redundant_task, (const float*)X2_33x1, _TEMP2_34x34[1]);
	mult_mv((const float**)_N1_33x33,33,33, (const float*)_TEMP2_34x34[1], Qpp_33x1);
	sum_vv((const float*)_TEMP1_34x34[1], 33, Qpp_33x1, Qpp_33x1);
	
	Qpp_33x1[WX_33] *= WEIGHT_PEL_SQRT_INV;
	Qpp_33x1[WY_33] *= WEIGHT_PEL_SQRT_INV;
	Qpp_33x1[WZ_33] *= WEIGHT_PEL_SQRT_INV;
	Qpp_33x1[WST_33] *= WEIGHT_WST_SQRT_INV;
	
	//------------------ check constraints
	RVALS3(_Q_34x1[WST_34], _Qp_33x1[WST_33], WSTpmax, WSTppmax, WSTmin, WSTmax, D2R, &lb_27x1[1], &ub_27x1[1]);
	RVALS3(_Q_34x1[RHY_34], _Qp_33x1[RHY_33], RHYpmax, RHYppmax, RHYmin, RHYmax, D2R, &lb_27x1[2], &ub_27x1[2]);
	RVALS3(_Q_34x1[RHR_34], _Qp_33x1[RHR_33], RHRpmax, RHRppmax, RHRmin, RHRmax, D2R, &lb_27x1[3], &ub_27x1[3]);
	RVALS3(_Q_34x1[RHP_34], _Qp_33x1[RHP_33], RHPpmax, RHPppmax, RHPmin, RHPmax, D2R, &lb_27x1[4], &ub_27x1[4]);
	RVALS3(_Q_34x1[RKN_34], _Qp_33x1[RKN_33], RKNpmax, RKNppmax, RKNmin, RKNmax, D2R, &lb_27x1[5], &ub_27x1[5]);
	RVALS3(_Q_34x1[RAP_34], _Qp_33x1[RAP_33], RAPpmax, RAPppmax, RAPmin, RAPmax, D2R, &lb_27x1[6], &ub_27x1[6]);
	RVALS3(_Q_34x1[RAR_34], _Qp_33x1[RAR_33], RARpmax, RARppmax, RARmin, RARmax, D2R, &lb_27x1[7], &ub_27x1[7]);
	RVALS3(_Q_34x1[LHY_34], _Qp_33x1[LHY_33], LHYpmax, LHYppmax, LHYmin, LHYmax, D2R, &lb_27x1[8], &ub_27x1[8]);
	RVALS3(_Q_34x1[LHR_34], _Qp_33x1[LHR_33], LHRpmax, LHRppmax, LHRmin, LHRmax, D2R, &lb_27x1[9], &ub_27x1[9]);
	RVALS3(_Q_34x1[LHP_34], _Qp_33x1[LHP_33], LHPpmax, LHPppmax, LHPmin, LHPmax, D2R, &lb_27x1[10], &ub_27x1[10]);
	RVALS3(_Q_34x1[LKN_34], _Qp_33x1[LKN_33], LKNpmax, LKNppmax, LKNmin, LKNmax, D2R, &lb_27x1[11], &ub_27x1[11]);
	RVALS3(_Q_34x1[LAP_34], _Qp_33x1[LAP_33], LAPpmax, LAPppmax, LAPmin, LAPmax, D2R, &lb_27x1[12], &ub_27x1[12]);
	RVALS3(_Q_34x1[LAR_34], _Qp_33x1[LAR_33], LARpmax, LARppmax, LARmin, LARmax, D2R, &lb_27x1[13], &ub_27x1[13]);
	RVALS3(_Q_34x1[RSP_34], _Qp_33x1[RSP_33], RSPpmax, RSPppmax, RSPmin, RSPmax, D2R, &lb_27x1[14], &ub_27x1[14]);
	RVALS3(_Q_34x1[RSR_34], _Qp_33x1[RSR_33], RSRpmax, RSRppmax, RSRmin, RSRmax, D2R, &lb_27x1[15], &ub_27x1[15]);
	RVALS3(_Q_34x1[RSY_34], _Qp_33x1[RSY_33], RSYpmax, RSYppmax, RSYmin, RSYmax, D2R, &lb_27x1[16], &ub_27x1[16]);
	RVALS3(_Q_34x1[REB_34], _Qp_33x1[REB_33], REBpmax, REBppmax, REBmin, REBmax, D2R, &lb_27x1[17], &ub_27x1[17]);
	RVALS3(_Q_34x1[RWY_34], _Qp_33x1[RWY_33], RWYpmax, RWYppmax, RWYmin, RWYmax, D2R, &lb_27x1[18], &ub_27x1[18]);
	RVALS3(_Q_34x1[RWP_34], _Qp_33x1[RWP_33], RWPpmax, RWPppmax, RWPmin, RWPmax, D2R, &lb_27x1[19], &ub_27x1[19]);	
	RVALS3(_Q_34x1[LSP_34], _Qp_33x1[LSP_33], LSPpmax, LSPppmax, LSPmin, LSPmax, D2R, &lb_27x1[20], &ub_27x1[20]);
	RVALS3(_Q_34x1[LSR_34], _Qp_33x1[LSR_33], LSRpmax, LSRppmax, LSRmin, LSRmax, D2R, &lb_27x1[21], &ub_27x1[21]);
	RVALS3(_Q_34x1[LSY_34], _Qp_33x1[LSY_33], LSYpmax, LSYppmax, LSYmin, LSYmax, D2R, &lb_27x1[22], &ub_27x1[22]);
	RVALS3(_Q_34x1[LEB_34], _Qp_33x1[LEB_33], LEBpmax, LEBppmax, LEBmin, LEBmax, D2R, &lb_27x1[23], &ub_27x1[23]);
	RVALS3(_Q_34x1[LWY_34], _Qp_33x1[LWY_33], LWYpmax, LWYppmax, LWYmin, LWYmax, D2R, &lb_27x1[24], &ub_27x1[24]);
	RVALS3(_Q_34x1[LWP_34], _Qp_33x1[LWP_33], LWPpmax, LWPppmax, LWPmin, LWPmax, D2R, &lb_27x1[25], &ub_27x1[25]);
	RVALS3(_Q_34x1[RWY2_34], _Qp_33x1[RWY2_33], RWY2pmax, RWY2ppmax, RWY2min, RWY2max, D2R, &lb_27x1[26], &ub_27x1[26]);
	RVALS3(_Q_34x1[LWY2_34], _Qp_33x1[LWY2_33], LWY2pmax, LWY2ppmax, LWY2min, LWY2max, D2R, &lb_27x1[27], &ub_27x1[27]);

	isLimited = 0;
	if(pSharedMemory->torque_mode_flag == 0 || pSharedMemory->torque_mode_flag == 3)
	{
		for(i=1;i<=27;i++)
		{
			index_limited[i] = 0;
			if(Qpp_33x1[i+6] >= ub_27x1[i])
			{
				isLimited = 1;
				index_limited[i] = 1;
			}
			else if(Qpp_33x1[i+6] <= lb_27x1[i])
			{
				isLimited = 1;
				index_limited[i] = 1;
			}

			if(isLimited==1)
			{
				printf("\n Joint limit error!!");
				printf("\n i: %d, joint: %d, p: %.4fdeg, v: %.4fdeg/s, a: %.4fdeg/s^2", _offline_traj.i, i+6, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D, Qpp_33x1[i+6]*R2D);
				printf("\n ub: %.4fdeg/s^2, lb: %.4fdeg/s^2", ub_27x1[i]*R2D, lb_27x1[i]*R2D);				
				return -2;
			}

			if(fabs(_Qp_33x1[i+6])>=QP_MAX)
			{
				printf("\n Joint velocity limit error!!");
				printf("\n i: %d, joint: %d, p: %.4fdeg, v: %.4fdeg/s, a: %.4fdeg/s^2", _offline_traj.i, i+6, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D, Qpp_33x1[i+6]*R2D);
				printf("\n ub: %.4fdeg/s^2, lb: %.4fdeg/s^2", ub_27x1[i]*R2D, lb_27x1[i]*R2D);			
				//return -2;
			}
		}
	}
	
	_Q_34x1[1] += _Qp_33x1[1]*DT + 0.5f*Qpp_33x1[1]*DT*DT;
	_Q_34x1[2] += _Qp_33x1[2]*DT + 0.5f*Qpp_33x1[2]*DT*DT;
	_Q_34x1[3] += _Qp_33x1[3]*DT + 0.5f*Qpp_33x1[3]*DT*DT;

	//------------------ update the pelvis quaternion 
	Wq(0, &_Q_34x1[3], _TEMP1_34x34);
	trans2(1.f, _TEMP1_34x34,3,4);
	mult_smv(0.5f, (const float**)_TEMP1_34x34,4,3, &Qpp_33x1[3], (float*)_TEMP2_34x34[1]); // qtpp

	Qq(0, &_Q_34x1[3], _TEMP1_34x34);
	_TEMP1_34x34[5][1] = 0.f;
	_TEMP1_34x34[5][2] = _Qp_33x1[4];
	_TEMP1_34x34[5][3] = _Qp_33x1[5];
	_TEMP1_34x34[5][4] = _Qp_33x1[6];
	mult_smv(0.5f, (const float**)_TEMP1_34x34,4,4, (const float*)_TEMP1_34x34[5], (float*)_TEMP3_34x34[1]); // qtp

	_Q_34x1[4] += _TEMP3_34x34[1][1]*DT + 0.5f*DT*DT*_TEMP2_34x34[1][1];
	_Q_34x1[5] += _TEMP3_34x34[1][2]*DT + 0.5f*DT*DT*_TEMP2_34x34[1][2];
	_Q_34x1[6] += _TEMP3_34x34[1][3]*DT + 0.5f*DT*DT*_TEMP2_34x34[1][3];
	_Q_34x1[7] += _TEMP3_34x34[1][4]*DT + 0.5f*DT*DT*_TEMP2_34x34[1][4];

	ftemp1_7x1[0] = (float)sqrt(_Q_34x1[4]*_Q_34x1[4] + _Q_34x1[5]*_Q_34x1[5] + _Q_34x1[6]*_Q_34x1[6] + _Q_34x1[7]*_Q_34x1[7]);
	_Q_34x1[4] /= ftemp1_7x1[0];
	_Q_34x1[5] /= ftemp1_7x1[0];
	_Q_34x1[6] /= ftemp1_7x1[0];
	_Q_34x1[7] /= ftemp1_7x1[0];
	//----------------------------

	_Q_34x1[WST_34] += _Qp_33x1[WST_33]*DT + 0.5f*Qpp_33x1[7]*DT*DT;
	_Q_34x1[RHY_34] += _Qp_33x1[RHY_33]*DT + 0.5f*Qpp_33x1[8]*DT*DT;
	_Q_34x1[RHR_34] += _Qp_33x1[RHR_33]*DT + 0.5f*Qpp_33x1[9]*DT*DT;
	_Q_34x1[RHP_34] += _Qp_33x1[RHP_33]*DT + 0.5f*Qpp_33x1[10]*DT*DT;
	_Q_34x1[RKN_34] += _Qp_33x1[RKN_33]*DT + 0.5f*Qpp_33x1[11]*DT*DT;
	_Q_34x1[RAP_34] += _Qp_33x1[RAP_33]*DT + 0.5f*Qpp_33x1[12]*DT*DT;
	_Q_34x1[RAR_34] += _Qp_33x1[RAR_33]*DT + 0.5f*Qpp_33x1[13]*DT*DT;
	_Q_34x1[LHY_34] += _Qp_33x1[LHY_33]*DT + 0.5f*Qpp_33x1[14]*DT*DT;
	_Q_34x1[LHR_34] += _Qp_33x1[LHR_33]*DT + 0.5f*Qpp_33x1[15]*DT*DT;
	_Q_34x1[LHP_34] += _Qp_33x1[LHP_33]*DT + 0.5f*Qpp_33x1[16]*DT*DT;
	_Q_34x1[LKN_34] += _Qp_33x1[LKN_33]*DT + 0.5f*Qpp_33x1[17]*DT*DT;
	_Q_34x1[LAP_34] += _Qp_33x1[LAP_33]*DT + 0.5f*Qpp_33x1[18]*DT*DT;
	_Q_34x1[LAR_34] += _Qp_33x1[LAR_33]*DT + 0.5f*Qpp_33x1[19]*DT*DT;	
	_Q_34x1[RSP_34] += _Qp_33x1[RSP_33]*DT + 0.5f*Qpp_33x1[20]*DT*DT;
	_Q_34x1[RSR_34] += _Qp_33x1[RSR_33]*DT + 0.5f*Qpp_33x1[21]*DT*DT;
	_Q_34x1[RSY_34] += _Qp_33x1[RSY_33]*DT + 0.5f*Qpp_33x1[22]*DT*DT;
	_Q_34x1[REB_34] += _Qp_33x1[REB_33]*DT + 0.5f*Qpp_33x1[23]*DT*DT;
	_Q_34x1[RWY_34] += _Qp_33x1[RWY_33]*DT + 0.5f*Qpp_33x1[24]*DT*DT;
	_Q_34x1[RWP_34] += _Qp_33x1[RWP_33]*DT + 0.5f*Qpp_33x1[25]*DT*DT;
	_Q_34x1[LSP_34] += _Qp_33x1[LSP_33]*DT + 0.5f*Qpp_33x1[26]*DT*DT;
	_Q_34x1[LSR_34] += _Qp_33x1[LSR_33]*DT + 0.5f*Qpp_33x1[27]*DT*DT;
	_Q_34x1[LSY_34] += _Qp_33x1[LSY_33]*DT + 0.5f*Qpp_33x1[28]*DT*DT;
	_Q_34x1[LEB_34] += _Qp_33x1[LEB_33]*DT + 0.5f*Qpp_33x1[29]*DT*DT;
	_Q_34x1[LWY_34] += _Qp_33x1[LWY_33]*DT + 0.5f*Qpp_33x1[30]*DT*DT;
	_Q_34x1[LWP_34] += _Qp_33x1[LWP_33]*DT + 0.5f*Qpp_33x1[31]*DT*DT;
	//_Q_34x1[RWY2_34] += _Qp_33x1[RWY2_33]*DT + 0.5f*Qpp_33x1[32]*DT*DT;
	//_Q_34x1[LWY2_34] += _Qp_33x1[LWY2_33]*DT + 0.5f*Qpp_33x1[33]*DT*DT;
	
	_Qp_33x1[1] += DT*Qpp_33x1[1];
	_Qp_33x1[2] += DT*Qpp_33x1[2];
	_Qp_33x1[3] += DT*Qpp_33x1[3];
	_Qp_33x1[4] += DT*Qpp_33x1[4];
	_Qp_33x1[5] += DT*Qpp_33x1[5];
	_Qp_33x1[6] += DT*Qpp_33x1[6];
	_Qp_33x1[WST_33] += DT*Qpp_33x1[7];
	_Qp_33x1[RHY_33] += DT*Qpp_33x1[8];
	_Qp_33x1[RHR_33] += DT*Qpp_33x1[9];
	_Qp_33x1[RHP_33] += DT*Qpp_33x1[10];
	_Qp_33x1[RKN_33] += DT*Qpp_33x1[11];
	_Qp_33x1[RAP_33] += DT*Qpp_33x1[12];
	_Qp_33x1[RAR_33] += DT*Qpp_33x1[13];
	_Qp_33x1[LHY_33] += DT*Qpp_33x1[14];
	_Qp_33x1[LHR_33] += DT*Qpp_33x1[15];
	_Qp_33x1[LHP_33] += DT*Qpp_33x1[16];
	_Qp_33x1[LKN_33] += DT*Qpp_33x1[17];
	_Qp_33x1[LAP_33] += DT*Qpp_33x1[18];
	_Qp_33x1[LAR_33] += DT*Qpp_33x1[19];	
	_Qp_33x1[RSP_33] += DT*Qpp_33x1[20];
	_Qp_33x1[RSR_33] += DT*Qpp_33x1[21];
	_Qp_33x1[RSY_33] += DT*Qpp_33x1[22];
	_Qp_33x1[REB_33] += DT*Qpp_33x1[23];
	_Qp_33x1[RWY_33] += DT*Qpp_33x1[24];
	_Qp_33x1[RWP_33] += DT*Qpp_33x1[25];
	_Qp_33x1[LSP_33] += DT*Qpp_33x1[26];
	_Qp_33x1[LSR_33] += DT*Qpp_33x1[27];
	_Qp_33x1[LSY_33] += DT*Qpp_33x1[28];
	_Qp_33x1[LEB_33] += DT*Qpp_33x1[29];
	_Qp_33x1[LWY_33] += DT*Qpp_33x1[30];
	_Qp_33x1[LWP_33] += DT*Qpp_33x1[31];
	//_Qp_33x1[RWY2_33] += DT*Qpp_33x1[32];
	//_Qp_33x1[LWY2_33] += DT*Qpp_33x1[33];

//	if(isLimited == 1)
//		goto LIMIT_LOOP;

	if(RSP_swing + _Q_34x1[RSP_34] > RSPmax)
		RSP_swing = RSPmax - _Q_34x1[RSP_34];
	else if(RSP_swing + _Q_34x1[RSP_34] < RSPmin)
		RSP_swing = RSPmin - _Q_34x1[RSP_34];

	if(LSP_swing + _Q_34x1[LSP_34] > LSPmax)
		LSP_swing = LSPmax - _Q_34x1[LSP_34];
	else if(LSP_swing + _Q_34x1[LSP_34] < LSPmin)
		LSP_swing = LSPmin - _Q_34x1[LSP_34];

	if(REB_swing + _Q_34x1[REB_34] > REBmax)
		REB_swing = REBmax - _Q_34x1[REB_34];
	else if(REB_swing + _Q_34x1[REB_34] < REBmin)
		REB_swing = REBmin - _Q_34x1[REB_34];

	if(LEB_swing + _Q_34x1[LEB_34] > LEBmax)
		LEB_swing = LEBmax - _Q_34x1[LEB_34];
	else if(LEB_swing + _Q_34x1[LEB_34] < LEBmin)
		LEB_swing = LEBmin - _Q_34x1[LEB_34];


	Joint[WST].RefAngleCurrent = _Q_34x1[WST_34]*R2D;

	Joint[RHY].RefAngleCurrent = (_Q_34x1[RHY_34]+rhy_compen + _online_pattern.offset_RHY)*R2D;
	Joint[RHR].RefAngleCurrent = (_Q_34x1[RHR_34]+rhr_compen +_online_pattern.offset_RHR)*R2D;
	Joint[RHP].RefAngleCurrent = (_Q_34x1[RHP_34] + _online_pattern.offset_RHP)*R2D;
	Joint[RKN].RefAngleCurrent = _Q_34x1[RKN_34]*R2D;
	Joint[RAP].RefAngleCurrent = (_Q_34x1[RAP_34] + _online_pattern.offset_RAP)*R2D;
	Joint[RAR].RefAngleCurrent = (_Q_34x1[RAR_34] + _online_pattern.offset_RAR)*R2D;
	Joint[LHY].RefAngleCurrent = (_Q_34x1[LHY_34]+lhy_compen + _online_pattern.offset_LHY)*R2D;
	Joint[LHR].RefAngleCurrent = (_Q_34x1[LHR_34]+lhr_compen+_online_pattern.offset_LHR)*R2D;
	Joint[LHP].RefAngleCurrent = (_Q_34x1[LHP_34] + _online_pattern.offset_LHP)*R2D;
	Joint[LKN].RefAngleCurrent = _Q_34x1[LKN_34]*R2D;
	Joint[LAP].RefAngleCurrent = (_Q_34x1[LAP_34] + _online_pattern.offset_LAP)*R2D;
	Joint[LAR].RefAngleCurrent = (_Q_34x1[LAR_34] + _online_pattern.offset_LAR)*R2D;

	Joint[RSP].RefAngleCurrent = (_Q_34x1[RSP_34]+RSP_swing)*R2D;
	Joint[RSR].RefAngleCurrent = _Q_34x1[RSR_34]*R2D - OFFSET_RSR;
	Joint[RSY].RefAngleCurrent = _Q_34x1[RSY_34]*R2D;
	Joint[REB].RefAngleCurrent = (_Q_34x1[REB_34]+REB_swing)*R2D - OFFSET_REB;
	Joint[RWY].RefAngleCurrent = _Q_34x1[RWY_34]*R2D;
	Joint[RWP].RefAngleCurrent = _Q_34x1[RWP_34]*R2D;
	//Joint[RWY2].RefAngleCurrent = _Q_34x1[RWY2_34]*R2D;
	Joint[LSP].RefAngleCurrent = (_Q_34x1[LSP_34]+LSP_swing)*R2D;
	Joint[LSR].RefAngleCurrent = _Q_34x1[LSR_34]*R2D - OFFSET_LSR;
	Joint[LSY].RefAngleCurrent = _Q_34x1[LSY_34]*R2D;
	Joint[LEB].RefAngleCurrent = (_Q_34x1[LEB_34]+LEB_swing)*R2D - OFFSET_LEB;	
	Joint[LWY].RefAngleCurrent = _Q_34x1[LWY_34]*R2D;
	Joint[LWP].RefAngleCurrent = _Q_34x1[LWP_34]*R2D;
	//Joint[LWY2].RefAngleCurrent = _Q_34x1[LWY2_34]*R2D;

	for(i=1;i<=34;i++)
		ref_Q_34x1[i] = _Q_34x1[i];
	
	pSharedMemory->disp_pCOM[0] = _pCOM_3x1[1];
	pSharedMemory->disp_pCOM[1] = _pCOM_3x1[2];

	pSharedMemory->disp_pRF[0] = _pRF_3x1[1];
	pSharedMemory->disp_pRF[1] = _pRF_3x1[2];
	pSharedMemory->disp_pRF[2] = _pRF_3x1[3];

	pSharedMemory->disp_pLF[0] = _pLF_3x1[1];
	pSharedMemory->disp_pLF[1] = _pLF_3x1[2];
	pSharedMemory->disp_pLF[2] = _pLF_3x1[3];

	pSharedMemory->disp_qRF[0] = _qRF_4x1[1];
	pSharedMemory->disp_qRF[1] = _qRF_4x1[2];
	pSharedMemory->disp_qRF[2] = _qRF_4x1[3];
	pSharedMemory->disp_qRF[3] = _qRF_4x1[4];

	pSharedMemory->disp_qLF[0] = _qLF_4x1[1];
	pSharedMemory->disp_qLF[1] = _qLF_4x1[2];
	pSharedMemory->disp_qLF[2] = _qLF_4x1[3];
	pSharedMemory->disp_qLF[3] = _qLF_4x1[4];
	
	pSharedMemory->disp_pRH[0] = _pRH_L_3x1[1];
	pSharedMemory->disp_pRH[1] = _pRH_L_3x1[2];
	pSharedMemory->disp_pRH[2] = _pRH_L_3x1[3];

	pSharedMemory->disp_pLH[0] = _pLH_L_3x1[1];
	pSharedMemory->disp_pLH[1] = _pLH_L_3x1[2];
	pSharedMemory->disp_pLH[2] = _pLH_L_3x1[3];

	pSharedMemory->disp_qRH[0] = _qRH_L_4x1[1];
	pSharedMemory->disp_qRH[1] = _qRH_L_4x1[2];
	pSharedMemory->disp_qRH[2] = _qRH_L_4x1[3];
	pSharedMemory->disp_qRH[3] = _qRH_L_4x1[4];

	pSharedMemory->disp_qLH[0] = _qLH_L_4x1[1];
	pSharedMemory->disp_qLH[1] = _qLH_L_4x1[2];
	pSharedMemory->disp_qLH[2] = _qLH_L_4x1[3];
	pSharedMemory->disp_qLH[3] = _qLH_L_4x1[4];

	for(i=1;i<=34;i++)
		pSharedMemory->disp_Q_34x1[i] = _Q_34x1[i];
	
	return 0;	
}


