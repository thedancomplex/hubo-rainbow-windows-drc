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

const float _AXIS_X[4] = {0., 1.f, 0., 0.};
const float _AXIS_Y[4] = {0., 0., 1.f, 0.};
const float _AXIS_Z[4] = {0., 0., 0., 1.f};


//------------- Constants for link geometries
const float _LINK_LEG[4] = {0., 0., 0., -L_LEG};
const float _LINK_RPEL[4] = {0., 0., -L1_PEL*0.5f, 0.};
const float _LINK_LPEL[4] = {0., 0., L1_PEL*0.5f, 0.};
const float _LINK_FOOT[4] = {0., 0., 0., -L_FOOT};
const float _LINK_WST[4] =  {0., 0., 0., L2_PEL};
const float _LINK_RSHLD[4] =  {0., 0., -L1_TOR*0.5f, L2_TOR};
const float _LINK_LSHLD[4] =  {0., 0., L1_TOR*0.5f, L2_TOR};
const float _LINK_UARM[4] =   {0., OFF_ELB, 0., -L_UARM};
const float _LINK_LARM[4] =   {0., -OFF_ELB, 0., -L_LARM};
const float _LINK_HAND[4] =   {0., 0., 0., -L_HAND};
const float _LINK_STICK[4] =   {0., 0., 0.,-L_STICK};
//-------------


//------------- Local coordinates of mass centers 
const float _C_PEL[4] = {0., CX_PEL, CY_PEL, CZ_PEL};
const float _C_RUL[4] = {0., CX_ULEG, -CY_ULEG, CZ_ULEG};
const float _C_RLL[4] = {0., CX_LLEG, -CY_LLEG, CZ_LLEG};
const float _C_RF[4] = {0., CX_FOOT, -CY_FOOT, CZ_FOOT};
const float _C_LUL[4] = {0., CX_ULEG, CY_ULEG, CZ_ULEG};
const float _C_LLL[4] = {0., CX_LLEG, CY_LLEG, CZ_LLEG};
const float _C_LF[4] = {0., CX_FOOT, CY_FOOT, CZ_FOOT};
const float _C_TOR[4] = {0., CX_TOR, CY_TOR, CZ_TOR};
const float _C_RUA[4] = {0., CX_UARM, -CY_UARM, CZ_UARM};
const float _C_RLA[4] = {0., CX_LARM+OFF_ELB, -CY_LARM, CZ_LARM};
const float _C_RHAND[4] = {0., CX_HAND, -CY_HAND, CZ_HAND};
const float _C_LUA[4] = {0., CX_UARM, CY_UARM, CZ_UARM};
const float _C_LLA[4] = {0., CX_LARM+OFF_ELB, CY_LARM, CZ_LARM};
const float _C_LHAND[4] = {0., CX_HAND, CY_HAND, CZ_HAND};
//--------------



//-------------- For kinematics
float **_Rz_RHY_3x3, **_Rx_RHR_3x3, **_Ry_RHP_3x3, **_Ry_RKN_3x3, **_Ry_RAP_3x3, **_Rx_RAR_3x3;
float **_Rz_LHY_3x3, **_Rx_LHR_3x3, **_Ry_LHP_3x3, **_Ry_LKN_3x3, **_Ry_LAP_3x3, **_Rx_LAR_3x3;
float **_Rz_WST_3x3, **_Ry_RSP_3x3, **_Rx_RSR_3x3, **_Rz_RSY_3x3, **_Ry_REB_3x3, **_Rz_RWY_3x3, **_Ry_RWP_3x3, **_Rz_RWY2_3x3;
float **_Ry_LSP_3x3, **_Rx_LSR_3x3, **_Rz_LSY_3x3, **_Ry_LEB_3x3, **_Rz_LWY_3x3, **_Ry_LWP_3x3, **_Rz_LWY2_3x3, **_Ry_PI_3x3;
float **_dcPEL_3x3, **_dcRUL_3x3, **_dcRLL_3x3, **_dcRF_3x3;
float **_dcLUL_3x3, **_dcLLL_3x3, **_dcLF_3x3;
float **_dcTOR_3x3, **_dcRUA_3x3, **_dcRLA_3x3, **_dcRH_3x3, **_dcRS_3x3;
float **_dcLUA_3x3, **_dcLLA_3x3, **_dcLH_3x3, **_dcLS_3x3;

float _pRF_3x1[4], _pLF_3x1[4], _qRF_4x1[5], _qLF_4x1[5], _pRH_3x1[4], _pLH_3x1[4], _qRH_4x1[5], _qLH_4x1[5], _pCOM_3x1[4];
float _pRS_3x1[4], _pLS_3x1[4], _qRS_4x1[5], _qLS_4x1[5];
float _pRWR_3x1[4], _pLWR_3x1[4], _pRANK_3x1[4], _pLANK_3x1[4], _qRWR_4x1[5], _qLWR_4x1[5];
float _pRF_L_3x1[4], _pLF_L_3x1[4], _qRF_L_4x1[5], _qLF_L_4x1[5], _pRH_L_3x1[4], _pLH_L_3x1[4], _qRH_L_4x1[5], _qLH_L_4x1[5], _pRS_L_3x1[4], _pLS_L_3x1[4], _qRS_L_4x1[5], _qLS_L_4x1[5], _qRWR_L_4x1[5], _qLWR_L_4x1[5]; // local coordinates
float _pRWR_L_3x1[4], _pLWR_L_3x1[4], _pRANK_L_3x1[4], _pLANK_L_3x1[4]; // local coordinates
char _FKineUpdatedFlag;
char _PassiveUpdatedFlag; 

float **_jRF_6x33, **_jLF_6x33, **_jCOM_3x33, **_jRH_6x33, **_jLH_6x33, **_jRS_6x33, **_jLS_6x33, **_jRWR_6x33, **_jLWR_6x33;
float **_jRF_old_6x33, **_jLF_old_6x33, **_jCOM_old_3x33, **_jRH_old_6x33, **_jLH_old_6x33, **_jRS_old_6x33, **_jLS_old_6x33, **_jRWR_old_6x33, **_jLWR_old_6x33;
float **_jRF_old2_6x33, **_jLF_old2_6x33, **_jCOM_old2_3x33, **_jRH_old2_6x33, **_jLH_old2_6x33, **_jRS_old2_6x33, **_jLS_old2_6x33, **_jRWR_old2_6x33, **_jLWR_old2_6x33;
float **_jRFp_6x33, **_jLFp_6x33, **_jCOMp_3x33, **_jRHp_6x33, **_jLHp_6x33, **_jRSp_6x33, **_jLSp_6x33, **_jRWRp_6x33, **_jLWRp_6x33;

float **_jT1_33x33, **_jT1inv_33x33, **_N1_33x33;
float **_jT2_33x33, **_jT2inv_33x33;
//---------------


//--------------------- temporary variables
float **_TEMP1_34x34, **_TEMP2_34x34, **_TEMP3_34x34, **_TEMP4_34x34;
float **_EYE_33;
//----------------------


//--------------------- Joint variables
float _Q_34x1[35]; // [x,y,z,qPEL[4],wst,rhy,rhr,rhp,rkn,rap,rar,lhy,lhr,lhp,lkn,lap,lar, rsp,rsr,rsy,reb,rwy,rwp, lsp,lsr,lsy,leb,lwy,lwp,rwy2, lwy2]
float _Qp_33x1[34]; // [x,y,z,wPEL[3],wst,rhy,rhr,rhp,rkn,rap,rar,lhy,lhr,lhp,lkn,lap,lar, rsp,rsr,rsy,reb,rwy,rwp, lsp,lsr,lsy,leb,lwy,lwp,rwyp2, lwyp2]
//----------------------


//--------------------- initial values
float _Q0_34x1[35];
float _pPEL0_3x1[4], _pCOM0_3x1[4];
float _pRF0_3x1[4], _pLF0_3x1[4], _qRF0_4x1[5], _qLF0_4x1[5];
float _pRH0_3x1[4], _pLH0_3x1[4], _qRH0_4x1[5], _qLH0_4x1[5];
//---------------------


//---------------------- For walking pattern
STEP_BUNDLE_RING _SBR;
FOOTSTEP _TwoStepBuff[2];
WALK_TRAJ _WalkTraj;
float _pZMP_3x1[4], _copRF_3x1[4], _copLF_3x1[4];
float _copRF_filt_2x1[3], _copLF_filt_2x1[3];
char _FSP, _HSP;		// Foot Supporting Phase, Hand Supporting Phase
char _land_RF, _land_LF, _land_RH, _land_LH;
float _fRS_3x1[4], _fLS_3x1[4];
//---------------------


//---------------------- For the offline test
OFFLINE_TRAJ _offline_traj;
float _log_temp[LOG_DATA_SIZE];
float _Xhat_DSP_F_4x1[5], _Xhat_DSP_S_4x1[5], _Xhat_SSP_S_4x1[5], _Xhat_SSP_F_4x1[5];
float _Ahat_DSP_F_BI_4x4[5][5], _Bhat_DSP_F_BI_4x1[5], _Lo_DSP_F_BI_4x1[5], _Kfb_DSP_F_BI_1x4[5], _Bss_DSP_F_BI_4x1[5];
float _Ahat_DSP_S_BI_4x4[5][5], _Bhat_DSP_S_BI_4x1[5], _Lo_DSP_S_BI_4x1[5], _Kfb_DSP_S_BI_1x4[5], _Bss_DSP_S_BI_4x1[5];
float _Ahat_SSP_S_BI_4x4[5][5], _Bhat_SSP_S_BI_4x1[5], _Lo_SSP_S_BI_4x1[5], _Kfb_SSP_S_BI_1x4[5], _Bss_SSP_S_BI_4x1[5];
float _Ahat_SSP_F_BI_4x4[5][5], _Bhat_SSP_F_BI_4x1[5], _Lo_SSP_F_BI_4x1[5], _Kfb_SSP_F_BI_1x4[5], _Bss_SSP_F_BI_4x1[5];

float _Ahat_DSP_F_QUAD_4x4[5][5], _Bhat_DSP_F_QUAD_4x1[5], _Lo_DSP_F_QUAD_4x1[5], _Kfb_DSP_F_QUAD_1x4[5], _Bss_DSP_F_QUAD_4x1[5];
float _Ahat_DSP_S_QUAD_4x4[5][5], _Bhat_DSP_S_QUAD_4x1[5], _Lo_DSP_S_QUAD_4x1[5], _Kfb_DSP_S_QUAD_1x4[5], _Bss_DSP_S_QUAD_4x1[5];
float _Ahat_SSP_S_QUAD_4x4[5][5], _Bhat_SSP_S_QUAD_4x1[5], _Lo_SSP_S_QUAD_4x1[5], _Kfb_SSP_S_QUAD_1x4[5], _Bss_SSP_S_QUAD_4x1[5];
float _Ahat_SSP_F_QUAD_4x4[5][5], _Bhat_SSP_F_QUAD_4x1[5], _Lo_SSP_F_QUAD_4x1[5], _Kfb_SSP_F_QUAD_1x4[5], _Bss_SSP_F_QUAD_4x1[5];

float _Ahat_DSP_F_DRC_4x4[5][5], _Bhat_DSP_F_DRC_4x1[5], _Lo_DSP_F_DRC_4x1[5], _Kfb_DSP_F_DRC_1x4[5], _Bss_DSP_F_DRC_4x1[5];
float _Ahat_DSP_S_DRC_4x4[5][5], _Bhat_DSP_S_DRC_4x1[5], _Lo_DSP_S_DRC_4x1[5], _Kfb_DSP_S_DRC_1x4[5], _Bss_DSP_S_DRC_4x1[5];
float _Ahat_SSP_S_DRC_4x4[5][5], _Bhat_SSP_S_DRC_4x1[5], _Lo_SSP_S_DRC_4x1[5], _Kfb_SSP_S_DRC_1x4[5], _Bss_SSP_S_DRC_4x1[5];
float _Ahat_SSP_F_DRC_4x4[5][5], _Bhat_SSP_F_DRC_4x1[5], _Lo_SSP_F_DRC_4x1[5], _Kfb_SSP_F_DRC_1x4[5], _Bss_SSP_F_DRC_4x1[5];
//----------------------


//----------------------- for online quad pattern
ONLINE_PATTERN _online_pattern;
//-----------------------

//--------------------- DRC mode
char _DRC_walking_mode;
//---------------------

//---------------------- torque control
float _gain_gravity[34], _gain_task[34], _friction_para[34][2];
//----------------------


int InitGlobalMotionVariables(void)
{
	int i,j;

	_PassiveUpdatedFlag = 0;
	_FKineUpdatedFlag = 0;

	InitGlobalNRutilVariables();	

	_jRF_6x33 = matrix(1,6,1,33);
	_jLF_6x33 = matrix(1,6,1,33);
	_jCOM_3x33 = matrix(1,3,1,33);
	_jRH_6x33 = matrix(1,6,1,33);
	_jLH_6x33 = matrix(1,6,1,33);
	_jRS_6x33 = matrix(1,6,1,33);
	_jLS_6x33 = matrix(1,6,1,33);
	_jRWR_6x33 = matrix(1,6,1,33);
	_jLWR_6x33 = matrix(1,6,1,33);

	_jRF_old_6x33 = matrix(1,6,1,33);
	_jLF_old_6x33 = matrix(1,6,1,33);
	_jCOM_old_3x33 = matrix(1,3,1,33);
	_jRH_old_6x33 = matrix(1,6,1,33);
	_jLH_old_6x33 = matrix(1,6,1,33);
	_jRS_old_6x33 = matrix(1,6,1,33);
	_jLS_old_6x33 = matrix(1,6,1,33);
	_jRWR_old_6x33 = matrix(1,6,1,33);
	_jLWR_old_6x33 = matrix(1,6,1,33);

	_jRF_old2_6x33 = matrix(1,6,1,33);
	_jLF_old2_6x33 = matrix(1,6,1,33);
	_jCOM_old2_3x33 = matrix(1,3,1,33);
	_jRH_old2_6x33 = matrix(1,6,1,33);
	_jLH_old2_6x33 = matrix(1,6,1,33);
	_jRS_old2_6x33 = matrix(1,6,1,33);
	_jLS_old2_6x33 = matrix(1,6,1,33);
	_jRWR_old2_6x33 = matrix(1,6,1,33);
	_jLWR_old2_6x33 = matrix(1,6,1,33);

	_jRFp_6x33 = matrix(1,6,1,33);
	_jLFp_6x33 = matrix(1,6,1,33);
	_jCOMp_3x33 = matrix(1,3,1,33);
	_jRHp_6x33 = matrix(1,6,1,33);
	_jLHp_6x33 = matrix(1,6,1,33);
	_jRSp_6x33 = matrix(1,6,1,33);
	_jLSp_6x33 = matrix(1,6,1,33);
	_jRWRp_6x33 = matrix(1,6,1,33);
	_jLWRp_6x33 = matrix(1,6,1,33);

	_Rz_RHY_3x3 = matrix(1,3,1,3);
	_Rx_RHR_3x3 = matrix(1,3,1,3);
	_Ry_RHP_3x3 = matrix(1,3,1,3);
	_Ry_RKN_3x3 = matrix(1,3,1,3);
	_Ry_RAP_3x3 = matrix(1,3,1,3);
	_Rx_RAR_3x3 = matrix(1,3,1,3);

	_Rz_LHY_3x3 = matrix(1,3,1,3);
	_Rx_LHR_3x3 = matrix(1,3,1,3);
	_Ry_LHP_3x3 = matrix(1,3,1,3);
	_Ry_LKN_3x3 = matrix(1,3,1,3);
	_Ry_LAP_3x3 = matrix(1,3,1,3);
	_Rx_LAR_3x3 = matrix(1,3,1,3);

	_Rz_WST_3x3 = matrix(1,3,1,3);
	_Ry_RSP_3x3 = matrix(1,3,1,3);
	_Rx_RSR_3x3 = matrix(1,3,1,3);
	_Rz_RSY_3x3 = matrix(1,3,1,3);
	_Ry_REB_3x3 = matrix(1,3,1,3);
	_Rz_RWY_3x3 = matrix(1,3,1,3);
	_Ry_RWP_3x3 = matrix(1,3,1,3);
	_Rz_RWY2_3x3 = matrix(1,3,1,3);

	_Ry_LSP_3x3 = matrix(1,3,1,3);
	_Rx_LSR_3x3 = matrix(1,3,1,3);
	_Rz_LSY_3x3 = matrix(1,3,1,3);
	_Ry_LEB_3x3 = matrix(1,3,1,3);
	_Rz_LWY_3x3 = matrix(1,3,1,3);
	_Ry_LWP_3x3 = matrix(1,3,1,3);
	_Rz_LWY2_3x3 = matrix(1,3,1,3);
	_Ry_PI_3x3 = matrix(1,3,1,3);

	_dcPEL_3x3 = matrix(1,3,1,3);
	_dcRUL_3x3 = matrix(1,3,1,3);
	_dcRLL_3x3 = matrix(1,3,1,3);
	_dcRF_3x3 = matrix(1,3,1,3);
	_dcLUL_3x3 = matrix(1,3,1,3);
	_dcLLL_3x3 = matrix(1,3,1,3);
	_dcLF_3x3 = matrix(1,3,1,3);

	_dcTOR_3x3 = matrix(1,3,1,3);
	_dcRUA_3x3 = matrix(1,3,1,3);
	_dcRLA_3x3 = matrix(1,3,1,3);
	_dcRH_3x3 = matrix(1,3,1,3);
	_dcLUA_3x3 = matrix(1,3,1,3);
	_dcLLA_3x3 = matrix(1,3,1,3);
	_dcLH_3x3 = matrix(1,3,1,3);
	_dcRS_3x3 = matrix(1,3,1,3);
	_dcLS_3x3 = matrix(1,3,1,3);

	_TEMP1_34x34 = matrix(1,34,1,34);
	_TEMP2_34x34 = matrix(1,34,1,34);
	_TEMP3_34x34 = matrix(1,34,1,34);
	_TEMP4_34x34 = matrix(1,34,1,34);
	_EYE_33 = matrix(1,33,1,33);

	_jT1_33x33 = matrix(1,33,1,33);
	_jT1inv_33x33 = matrix(1,33,1,33);
	_N1_33x33 = matrix(1,33,1,33);
	_jT2_33x33 = matrix(1,33,1,33);
	_jT2inv_33x33 = matrix(1,33,1,33);

	for(j=1; j<=33; j++)
	{
		for(i=1; i<=6; i++)
		{			
			_jRF_6x33[i][j] = 0.;
			_jLF_6x33[i][j] = 0.;

			_jRFp_6x33[i][j] = 0.;
			_jLFp_6x33[i][j] = 0.;

			_jRH_6x33[i][j] = 0.;
			_jLH_6x33[i][j] = 0.;

			_jRS_6x33[i][j] = 0.;
			_jLS_6x33[i][j] = 0.;

			_jRWR_6x33[i][j] = 0.;
			_jLWR_6x33[i][j] = 0.;

			_jRHp_6x33[i][j] = 0.;
			_jLHp_6x33[i][j] = 0.;

			_jRSp_6x33[i][j] = 0.;
			_jLSp_6x33[i][j] = 0.;

			_jRWRp_6x33[i][j] = 0.;
			_jLWRp_6x33[i][j] = 0.;
		}

		for(i=1; i<=3; i++)
		{	
			_jCOM_3x33[i][j] = 0.;
			_jCOMp_3x33[i][j] = 0.;		
		}

		for(i=1; i<=33; i++)
		{
			if(i == j)
				_EYE_33[i][j] = 1.f;
			else
				_EYE_33[i][j] = 0.;	

			_jT1_33x33[i][j] = 0.;
			_jT1inv_33x33[i][j] = 0.;
			_N1_33x33[i][j] = 0.;
			_jT2_33x33[i][j] = 0.;
			_jT2inv_33x33[i][j] = 0.;
		}
	}

	_copRF_filt_2x1[1] = 0.;
	_copRF_filt_2x1[2] = 0.;
	_copLF_filt_2x1[1] = 0.;
	_copLF_filt_2x1[2] = 0.;
	return 0;
}


int UpdateGlobalMotionVariables(void)
{
	int i;

	subs_m((const float**)_jRF_6x33,6,33, _jRF_old_6x33);
	subs_m((const float**)_jLF_6x33,6,33, _jLF_old_6x33);
	subs_m((const float**)_jRH_6x33,6,33, _jRH_old_6x33);
	subs_m((const float**)_jLH_6x33,6,33, _jLH_old_6x33);
	subs_m((const float**)_jRS_6x33,6,33, _jRS_old_6x33);
	subs_m((const float**)_jLS_6x33,6,33, _jLS_old_6x33);
	subs_m((const float**)_jRWR_6x33,6,33, _jRWR_old_6x33);
	subs_m((const float**)_jLWR_6x33,6,33, _jLWR_old_6x33);
	subs_m((const float**)_jCOM_3x33,3,33, _jCOM_old_3x33);

	subs_m((const float**)_jRF_6x33,6,33, _jRF_old2_6x33);
	subs_m((const float**)_jLF_6x33,6,33, _jLF_old2_6x33);
	subs_m((const float**)_jRH_6x33,6,33, _jRH_old2_6x33);
	subs_m((const float**)_jLH_6x33,6,33, _jLH_old2_6x33);
	subs_m((const float**)_jRS_6x33,6,33, _jRS_old2_6x33);
	subs_m((const float**)_jLS_6x33,6,33, _jLS_old2_6x33);
	subs_m((const float**)_jRWR_6x33,6,33, _jRWR_old2_6x33);
	subs_m((const float**)_jLWR_6x33,6,33, _jLWR_old2_6x33);
	subs_m((const float**)_jCOM_3x33,3,33, _jCOM_old2_3x33);

	_Q_34x1[RHY_34] = Joint[RHY].RefAngleCurrent*D2R;
	_Q_34x1[RHR_34] = Joint[RHR].RefAngleCurrent*D2R;
	_Q_34x1[RHP_34] = Joint[RHP].RefAngleCurrent*D2R;
	_Q_34x1[RKN_34] = Joint[RKN].RefAngleCurrent*D2R;
	_Q_34x1[RAP_34] = Joint[RAP].RefAngleCurrent*D2R;
	_Q_34x1[RAR_34] = Joint[RAR].RefAngleCurrent*D2R;
	_Q_34x1[LHY_34] = Joint[LHY].RefAngleCurrent*D2R;
	_Q_34x1[LHR_34] = Joint[LHR].RefAngleCurrent*D2R;
	_Q_34x1[LHP_34] = Joint[LHP].RefAngleCurrent*D2R;
	_Q_34x1[LKN_34] = Joint[LKN].RefAngleCurrent*D2R;
	_Q_34x1[LAP_34] = Joint[LAP].RefAngleCurrent*D2R;
	_Q_34x1[LAR_34] = Joint[LAR].RefAngleCurrent*D2R;

	_Q_34x1[WST_34] = Joint[WST].RefAngleCurrent*D2R;
	_Q_34x1[RSP_34] = Joint[RSP].RefAngleCurrent*D2R;
	_Q_34x1[RSR_34] = (Joint[RSR].RefAngleCurrent + OFFSET_RSR)*D2R;
	_Q_34x1[RSY_34] = Joint[RSY].RefAngleCurrent*D2R;
	_Q_34x1[REB_34] = (Joint[REB].RefAngleCurrent + OFFSET_REB)*D2R;
	_Q_34x1[RWY_34] = Joint[RWY].RefAngleCurrent*D2R;
	_Q_34x1[RWP_34] = Joint[RWP].RefAngleCurrent*D2R;
	_Q_34x1[LSP_34] = Joint[LSP].RefAngleCurrent*D2R;
	_Q_34x1[LSR_34] = (Joint[LSR].RefAngleCurrent + OFFSET_LSR)*D2R;
	_Q_34x1[LSY_34] = Joint[LSY].RefAngleCurrent*D2R;
	_Q_34x1[LEB_34] = (Joint[LEB].RefAngleCurrent + OFFSET_LEB)*D2R;
	_Q_34x1[LWY_34] = Joint[LWY].RefAngleCurrent*D2R;
	_Q_34x1[LWP_34] = Joint[LWP].RefAngleCurrent*D2R;
	_Q_34x1[RWY2_34] = Joint[RWY2].RefAngleCurrent*D2R;
	_Q_34x1[LWY2_34] = Joint[LWY2].RefAngleCurrent*D2R;
	
	//------------------ initial joint pos
	for(i=8; i<=34; i++)
		_Q0_34x1[i] = _Q_34x1[i];

	for(i=1; i<=33; i++)
		_Qp_33x1[i] = 0.f;

	UpdatePassiveCoord_SSP(LFSP);
	//---------------------
	
	return 0;
}


int FreeGlobalMotionVariables(void)
{
	free_matrix(_jRF_6x33, 1,6,1,33);
	free_matrix(_jLF_6x33, 1,6,1,33);
	free_matrix(_jCOM_3x33, 1,3,1,33);
	free_matrix(_jRH_6x33, 1,6,1,33);
	free_matrix(_jLH_6x33, 1,6,1,33);
	free_matrix(_jRS_6x33, 1,6,1,33);
	free_matrix(_jLS_6x33, 1,6,1,33);
	free_matrix(_jRWR_6x33, 1,6,1,33);
	free_matrix(_jLWR_6x33, 1,6,1,33);

	free_matrix(_jRF_old_6x33, 1,6,1,33);
	free_matrix(_jLF_old_6x33, 1,6,1,33);
	free_matrix(_jCOM_old_3x33, 1,3,1,33);
	free_matrix(_jRH_old_6x33, 1,6,1,33);
	free_matrix(_jLH_old_6x33, 1,6,1,33);
	free_matrix(_jRS_old_6x33, 1,6,1,33);
	free_matrix(_jLS_old_6x33, 1,6,1,33);
	free_matrix(_jRWR_old_6x33, 1,6,1,33);
	free_matrix(_jLWR_old_6x33, 1,6,1,33);

	free_matrix(_jRF_old2_6x33, 1,6,1,33);
	free_matrix(_jLF_old2_6x33, 1,6,1,33);
	free_matrix(_jCOM_old2_3x33, 1,3,1,33);
	free_matrix(_jRH_old2_6x33, 1,6,1,33);
	free_matrix(_jLH_old2_6x33, 1,6,1,33);
	free_matrix(_jRS_old2_6x33, 1,6,1,33);
	free_matrix(_jLS_old2_6x33, 1,6,1,33);
	free_matrix(_jRWR_old2_6x33, 1,6,1,33);
	free_matrix(_jLWR_old2_6x33, 1,6,1,33);

	free_matrix(_jRFp_6x33, 1,6,1,33);
	free_matrix(_jLFp_6x33, 1,6,1,33);
	free_matrix(_jCOMp_3x33, 1,3,1,33);
	free_matrix(_jRHp_6x33, 1,6,1,33);
	free_matrix(_jLHp_6x33, 1,6,1,33);
	free_matrix(_jRSp_6x33, 1,6,1,33);
	free_matrix(_jLSp_6x33, 1,6,1,33);
	free_matrix(_jRWRp_6x33, 1,6,1,33);
	free_matrix(_jLWRp_6x33, 1,6,1,33);

	free_matrix(_Rz_RHY_3x3,1,3,1,3);
	free_matrix(_Rx_RHR_3x3,1,3,1,3);
	free_matrix(_Ry_RHP_3x3,1,3,1,3);
	free_matrix(_Ry_RKN_3x3,1,3,1,3);
	free_matrix(_Ry_RAP_3x3,1,3,1,3);
	free_matrix(_Rx_RAR_3x3,1,3,1,3);

	free_matrix(_Rz_LHY_3x3,1,3,1,3);
	free_matrix(_Rx_LHR_3x3,1,3,1,3);
	free_matrix(_Ry_LHP_3x3,1,3,1,3);
	free_matrix(_Ry_LKN_3x3,1,3,1,3);
	free_matrix(_Ry_LAP_3x3,1,3,1,3);
	free_matrix(_Rx_LAR_3x3,1,3,1,3);

	free_matrix(_Rz_WST_3x3,1,3,1,3);
	free_matrix(_Ry_RSP_3x3,1,3,1,3);
	free_matrix(_Rx_RSR_3x3,1,3,1,3);
	free_matrix(_Rz_RSY_3x3,1,3,1,3);
	free_matrix(_Ry_REB_3x3,1,3,1,3);
	free_matrix(_Rz_RWY_3x3,1,3,1,3);
	free_matrix(_Ry_RWP_3x3,1,3,1,3);
	free_matrix(_Rz_RWY2_3x3,1,3,1,3);

	free_matrix(_Ry_LSP_3x3,1,3,1,3);
	free_matrix(_Rx_LSR_3x3,1,3,1,3);
	free_matrix(_Rz_LSY_3x3,1,3,1,3);
	free_matrix(_Ry_LEB_3x3,1,3,1,3);
	free_matrix(_Rz_LWY_3x3,1,3,1,3);
	free_matrix(_Ry_LWP_3x3,1,3,1,3);
	free_matrix(_Rz_LWY2_3x3,1,3,1,3);
	free_matrix(_Ry_PI_3x3,1,3,1,3);

	free_matrix(_dcPEL_3x3,1,3,1,3);
	free_matrix(_dcRUL_3x3,1,3,1,3);
	free_matrix(_dcRLL_3x3,1,3,1,3);
	free_matrix(_dcRF_3x3,1,3,1,3);
	free_matrix(_dcLUL_3x3,1,3,1,3);
	free_matrix(_dcLLL_3x3,1,3,1,3);
	free_matrix(_dcLF_3x3,1,3,1,3);

	free_matrix(_dcTOR_3x3,1,3,1,3);
	free_matrix(_dcRUA_3x3,1,3,1,3);
	free_matrix(_dcRLA_3x3,1,3,1,3);
	free_matrix(_dcRH_3x3,1,3,1,3);
	free_matrix(_dcLUA_3x3,1,3,1,3);
	free_matrix(_dcLLA_3x3,1,3,1,3);
	free_matrix(_dcLH_3x3,1,3,1,3);
	free_matrix(_dcRS_3x3,1,3,1,3);
	free_matrix(_dcLS_3x3,1,3,1,3);

	free_matrix(_TEMP1_34x34,1,34,1,34);
	free_matrix(_TEMP2_34x34,1,34,1,34);
	free_matrix(_TEMP3_34x34,1,34,1,34);
	free_matrix(_TEMP4_34x34,1,34,1,34);
	free_matrix(_EYE_33, 1,33,1,33);

	free_matrix(_jT1_33x33, 1,33,1,33);
	free_matrix(_jT1inv_33x33, 1,33,1,33);
	free_matrix(_N1_33x33, 1,33,1,33);
	free_matrix(_jT2_33x33, 1,33,1,33);
	free_matrix(_jT2inv_33x33, 1,33,1,33);

	FreeGlobalNRutilVariables();
	return 0;
}


int FKine_COM(const float *Q_34x1, float *pCOM_3x1)
{
	float wst, rhy, rhp, rhr, rkn, rap, rar, lhy, lhp, lhr, lkn, lap, lar, rsp, rsr, rsy, reb, rwy, rwp, lsp, lsr, lsy, leb, lwy, lwp, rwy2, lwy2;
	float pRPEL[4], pRKN[4], pLPEL[4], pLKN[4], pWST[4], pRSHLD[4], pRELB[4], pLSHLD[4], pLELB[4], pRELB2[4], pLELB2[4];
	float cPEL[4], cTOR[4], cRUL[4], cRLL[4], cRF[4], cLUL[4], cLLL[4], cLF[4], cRUA[4], cRLA[4], cLUA[4], cLLA[4], cRH[4], cLH[4];
	float temp3_3x1[4], temp4_3x1[4];
	float qPEL[5] = {0., Q_34x1[4], Q_34x1[5], Q_34x1[6], Q_34x1[7]};
	float pPC[4] = {0., Q_34x1[1], Q_34x1[2], Q_34x1[3]};
	float pRANK_3x1[4], pLANK_3x1[4], pRWR_3x1[4], pLWR_3x1[4];

	rhy = Q_34x1[9];
	rhr = Q_34x1[10];
	rhp = Q_34x1[11];
	rkn = Q_34x1[12];
	rap = Q_34x1[13];
	rar = Q_34x1[14];

	lhy = Q_34x1[15];
	lhr = Q_34x1[16];
	lhp = Q_34x1[17];
	lkn = Q_34x1[18];
	lap = Q_34x1[19];
	lar = Q_34x1[20];

	wst = Q_34x1[8];
	rsp = Q_34x1[21];
	rsr = Q_34x1[22];
	rsy = Q_34x1[23];
	reb = Q_34x1[24];
	rwy = Q_34x1[25];
	rwp = Q_34x1[26];

	lsp = Q_34x1[27];
	lsr = Q_34x1[28];
	lsy = Q_34x1[29];
	leb = Q_34x1[30];
	lwy = Q_34x1[31];
	lwp = Q_34x1[32];

	rwy2 = Q_34x1[33];
	lwy2 = Q_34x1[34];

	QT2DC(qPEL, _dcPEL_3x3);
	
	RZ(rhy, _Rz_RHY_3x3);
	RX(rhr, _Rx_RHR_3x3);
	RY(rhp, _Ry_RHP_3x3);
	RY(rkn, _Ry_RKN_3x3);
	RY(rap, _Ry_RAP_3x3);
	RX(rar, _Rx_RAR_3x3);

	RZ(lhy, _Rz_LHY_3x3);
	RX(lhr, _Rx_LHR_3x3);
	RY(lhp, _Ry_LHP_3x3);
	RY(lkn, _Ry_LKN_3x3);
	RY(lap, _Ry_LAP_3x3);
	RX(lar, _Rx_LAR_3x3);

	RZ(wst, _Rz_WST_3x3);
	
	RY(rsp, _Ry_RSP_3x3);
	RX(rsr, _Rx_RSR_3x3);
	RZ(rsy, _Rz_RSY_3x3);
	RY(reb, _Ry_REB_3x3);
	RZ(rwy, _Rz_RWY_3x3);
	RY(rwp, _Ry_RWP_3x3);

	RY(lsp, _Ry_LSP_3x3);
	RX(lsr, _Rx_LSR_3x3);
	RZ(lsy, _Rz_LSY_3x3);
	RY(leb, _Ry_LEB_3x3);
	RZ(lwy, _Rz_LWY_3x3);
	RY(lwp, _Ry_LWP_3x3);

	RZ(rwy2, _Rz_RWY2_3x3);
	RZ(lwy2, _Rz_LWY2_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_RHR_3x3,3,3, (const float**)_Ry_RHP_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcRUL_3x3);

	mult_mm((const float**)_dcRUL_3x3,3,3, (const float**)_Ry_RKN_3x3,3, _dcRLL_3x3);
	
	mult_mm((const float**)_Ry_RAP_3x3,3,3, (const float**)_Rx_RAR_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_dcRLL_3x3,3,3, (const float**)_TEMP1_34x34,3, _dcRF_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_LHR_3x3,3,3, (const float**)_Ry_LHP_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcLUL_3x3);

	mult_mm((const float**)_dcLUL_3x3,3,3, (const float**)_Ry_LKN_3x3,3, _dcLLL_3x3);
	
	mult_mm((const float**)_Ry_LAP_3x3,3,3, (const float**)_Rx_LAR_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_dcLLL_3x3,3,3, (const float**)_TEMP1_34x34,3, _dcLF_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_WST_3x3,3, _dcTOR_3x3);

	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_RSP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_RSR_3x3,3,3, (const float**)_Rz_RSY_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcRUA_3x3);
	
	mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REB_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_RWY_3x3,3, _dcRLA_3x3);
	mult_mm((const float**)_dcRLA_3x3,3,3, (const float**)_Ry_RWP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_RWY2_3x3,3, _dcRH_3x3);

	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_LSP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_LSR_3x3,3,3, (const float**)_Rz_LSY_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcLUA_3x3);

	mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEB_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_LWY_3x3,3, _dcLLA_3x3);
	mult_mm((const float**)_dcLLA_3x3,3,3, (const float**)_Ry_LWP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_LWY2_3x3,3, _dcLH_3x3);
	
	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_RPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pRPEL);

	mult_mv((const float**)_dcRUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRPEL,3, temp3_3x1, pRKN);

	mult_mv((const float**)_dcRLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, pRANK_3x1);

	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_LPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pLPEL);

	mult_mv((const float**)_dcLUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLPEL,3, temp3_3x1, pLKN);

	mult_mv((const float**)_dcLLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, pLANK_3x1);

	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_WST, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pWST);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_RSHLD, temp3_3x1); 
	sum_vv(pWST,3, temp3_3x1, pRSHLD);

	mult_mv((const float**)_dcRUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pRSHLD,3, temp3_3x1, pRELB);

	temp4_3x1[1] = _LINK_LARM[1];
	temp4_3x1[2] = 0.;
	temp4_3x1[3] = 0.;
	mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REB_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pRELB,3, temp3_3x1, pRELB2);
	temp4_3x1[1] = 0.;
	temp4_3x1[2] = _LINK_LARM[2];
	temp4_3x1[3] = _LINK_LARM[3];
	mult_mv((const float**)_dcRLA_3x3,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pRELB2,3, temp3_3x1, pRWR_3x1);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_LSHLD, temp3_3x1); 
	sum_vv(pWST,3, temp3_3x1, pLSHLD);

	mult_mv((const float**)_dcLUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pLSHLD,3, temp3_3x1, pLELB);

	temp4_3x1[1] = _LINK_LARM[1];
	temp4_3x1[2] = 0.;
	temp4_3x1[3] = 0.;
	mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEB_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pLELB,3, temp3_3x1, pLELB2);
	temp4_3x1[1] = 0.;
	temp4_3x1[2] = _LINK_LARM[2];
	temp4_3x1[3] = _LINK_LARM[3];
	mult_mv((const float**)_dcLLA_3x3,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pLELB2,3, temp3_3x1, pLWR_3x1);

	mult_mv((const float**)_dcPEL_3x3,3,3, _C_PEL, temp3_3x1);
	sum_vv(pPC,3, temp3_3x1, cPEL);

	mult_mv((const float**)_dcTOR_3x3,3,3, _C_TOR, temp3_3x1);
	sum_vv(pWST,3, temp3_3x1, cTOR);

	mult_mv((const float**)_dcRUL_3x3,3,3, _C_RUL, temp3_3x1);
	sum_vv(pRPEL,3, temp3_3x1, cRUL);

	mult_mv((const float**)_dcRLL_3x3,3,3, _C_RLL, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, cRLL);

	mult_mv((const float**)_dcRF_3x3,3,3, _C_RF, temp3_3x1);
	sum_vv(pRANK_3x1,3, temp3_3x1, cRF);

	mult_mv((const float**)_dcLUL_3x3,3,3, _C_LUL, temp3_3x1);
	sum_vv(pLPEL,3, temp3_3x1, cLUL);

	mult_mv((const float**)_dcLLL_3x3,3,3, _C_LLL, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, cLLL);

	mult_mv((const float**)_dcLF_3x3,3,3, _C_LF, temp3_3x1);
	sum_vv(pLANK_3x1,3, temp3_3x1, cLF);

	mult_mv((const float**)_dcRUA_3x3,3,3, _C_RUA, temp3_3x1);
	sum_vv(pRSHLD,3, temp3_3x1, cRUA);

	mult_mv((const float**)_dcRLA_3x3,3,3, _C_RLA, temp3_3x1); 
	sum_vv(pRELB2,3, temp3_3x1, cRLA);

	mult_mv((const float**)_dcRH_3x3,3,3, _C_RHAND, temp3_3x1);
	sum_vv(pRWR_3x1,3, temp3_3x1, cRH);

	mult_mv((const float**)_dcLUA_3x3,3,3, _C_LUA, temp3_3x1);
	sum_vv(pLSHLD,3, temp3_3x1, cLUA);

	mult_mv((const float**)_dcLLA_3x3,3,3, _C_LLA, temp3_3x1); 
	sum_vv(pLELB2,3, temp3_3x1, cLLA);
	
	mult_mv((const float**)_dcLH_3x3,3,3, _C_LHAND, temp3_3x1);
	sum_vv(pLWR_3x1,3, temp3_3x1, cLH);
	
	pCOM_3x1[1] = ((m_PEL*cPEL[1] +m_TOR*cTOR[1] + m_ULEG*(cRUL[1]+cLUL[1]) + m_LLEG*(cRLL[1]+cLLL[1]) +m_FOOT*(cRF[1]+cLF[1]) + m_UARM*(cRUA[1]+cLUA[1]) + m_LARM*(cRLA[1]+cLLA[1]) + m_RHAND*cRH[1] + m_LHAND*cLH[1])/m_TOTAL);
	pCOM_3x1[2] = ((m_PEL*cPEL[2] +m_TOR*cTOR[2] + m_ULEG*(cRUL[2]+cLUL[2]) + m_LLEG*(cRLL[2]+cLLL[2]) +m_FOOT*(cRF[2]+cLF[2]) + m_UARM*(cRUA[2]+cLUA[2]) + m_LARM*(cRLA[2]+cLLA[2]) + m_RHAND*cRH[2] + m_LHAND*cLH[2])/m_TOTAL);
	pCOM_3x1[3] = ((m_PEL*cPEL[3] +m_TOR*cTOR[3] + m_ULEG*(cRUL[3]+cLUL[3]) + m_LLEG*(cRLL[3]+cLLL[3]) +m_FOOT*(cRF[3]+cLF[3]) + m_UARM*(cRUA[3]+cLUA[3]) + m_LARM*(cRLA[3]+cLLA[3]) + m_RHAND*cRH[3] + m_LHAND*cLH[3])/m_TOTAL);

	return 0;
}


int FKine_Whole(void)
{
	float wst, rhy, rhp, rhr, rkn, rap, rar, lhy, lhp, lhr, lkn, lap, lar, rsp, rsr, rsy, reb, rwy, lsp, lsr, lsy, leb, lwy, rwp, lwp, rwy2, lwy2;
	float axis_rhy[4], axis_rhr[4], axis_rhp[4], axis_rkn[4], axis_rap[4], axis_rar[4];	 
	float axis_lhy[4], axis_lhr[4], axis_lhp[4], axis_lkn[4], axis_lap[4], axis_lar[4];	
	float axis_wst[4], axis_rsp[4], axis_rsr[4], axis_rsy[4], axis_reb[4], axis_rwy[4], axis_rwp[4], axis_rwy2[4];
	float axis_lsp[4], axis_lsr[4], axis_lsy[4], axis_leb[4], axis_lwy[4], axis_lwp[4], axis_lwy2[4];

	float pRPEL[4], pRKN[4], pLPEL[4], pLKN[4], pWST[4], pRSHLD[4], pRELB[4], pLSHLD[4], pLELB[4], pRELB2[4], pLELB2[4];
	float cPEL[4], cTOR[4], cRUL[4], cRLL[4], cRF[4], cLUL[4], cLLL[4], cLF[4], cRUA[4], cRLA[4], cLUA[4], cLLA[4], cRH[4], cLH[4];
	float temp3_3x1[4], temp4_3x1[4];

	//--- Jacobian for [xp,yp,zp,wPELx,wPELy,wPELz,wstp,rhyp,rhrp,rhpp,rknp,rapp,rarp,lhyp,lhrp,lhpp,lknp,lapp,larp,rspp,rsrp,rsyp,rebp,rwyp,rwpp,lspp,lsrp,lsyp,lebp,lwyp,lwpp]' : 31x1
	float **jCOM_PEL_3x33, **jCOM_RUL_3x33, **jCOM_RLL_3x33, **jCOM_RF_3x33, **jCOM_LUL_3x33, **jCOM_LLL_3x33, **jCOM_LF_3x33;
	float **jCOM_TOR_3x33, **jCOM_RUA_3x33, **jCOM_RLA_3x33, **jCOM_LUA_3x33, **jCOM_LLA_3x33, **jCOM_LH_3x33, **jCOM_RH_3x33;

	const float qPEL[5] = {0., _Q_34x1[4], _Q_34x1[5], _Q_34x1[6], _Q_34x1[7]};
	const float pPC[4] = {0., _Q_34x1[1], _Q_34x1[2], _Q_34x1[3]};

	int i, j;
	float ftemp;

	jCOM_PEL_3x33 = matrix(1,3,1,33);
	jCOM_RUL_3x33 = matrix(1,3,1,33);
	jCOM_RLL_3x33 = matrix(1,3,1,33);
	jCOM_RF_3x33 = matrix(1,3,1,33);
	jCOM_LUL_3x33 = matrix(1,3,1,33);
	jCOM_LLL_3x33 = matrix(1,3,1,33);
	jCOM_LF_3x33 = matrix(1,3,1,33);
	jCOM_TOR_3x33 = matrix(1,3,1,33);
	jCOM_RUA_3x33 = matrix(1,3,1,33);
	jCOM_RLA_3x33 = matrix(1,3,1,33);
	jCOM_LUA_3x33 = matrix(1,3,1,33);
	jCOM_LLA_3x33 = matrix(1,3,1,33);
	jCOM_LH_3x33 = matrix(1,3,1,33);
	jCOM_RH_3x33 = matrix(1,3,1,33);
	
	for(i=1; i<=3; i++)
	{
		for(j=1; j<=33; j++)
		{
			_jCOM_3x33[i][j] = 0.;
			_jCOM_3x33[i][j] = 0.;
			_jCOM_3x33[i][j] = 0.;

			jCOM_PEL_3x33[i][j] = 0.;
			jCOM_RUL_3x33[i][j] = 0.;
			jCOM_RLL_3x33[i][j] = 0.;
			jCOM_RF_3x33[i][j] = 0.;
			jCOM_LUL_3x33[i][j] = 0.;
			jCOM_LLL_3x33[i][j] = 0.;
			jCOM_LF_3x33[i][j] = 0.;
			jCOM_TOR_3x33[i][j] = 0.;
			jCOM_RUA_3x33[i][j] = 0.;
			jCOM_RLA_3x33[i][j] = 0.;
			jCOM_LUA_3x33[i][j] = 0.;
			jCOM_LLA_3x33[i][j] = 0.;
			jCOM_RH_3x33[i][j] = 0.;
			jCOM_LH_3x33[i][j] = 0.;	
			
			_jRF_6x33[i][j] = 0.;
			_jLF_6x33[i][j] = 0.;
			_jRH_6x33[i][j] = 0.;
			_jLH_6x33[i][j] = 0.;
			_jRS_6x33[i][j] = 0.;
			_jLS_6x33[i][j] = 0.;
			_jRWR_6x33[i][j] = 0.;
			_jLWR_6x33[i][j] = 0.;

			_jRF_6x33[i+3][j] = 0.;
			_jLF_6x33[i+3][j] = 0.;
			_jRH_6x33[i+3][j] = 0.;
			_jLH_6x33[i+3][j] = 0.;
			_jRWR_6x33[i+3][j] = 0.;
			_jLWR_6x33[i+3][j] = 0.;
		}
	}

	_jCOM_3x33[1][1] = 1.f;
	_jCOM_3x33[2][2] = 1.f;
	_jCOM_3x33[3][3] = 1.f;

	rhy = _Q_34x1[RHY_34];
	rhr = _Q_34x1[RHR_34];
	rhp = _Q_34x1[RHP_34];
	rkn = _Q_34x1[RKN_34];
	rap = _Q_34x1[RAP_34];
	rar = _Q_34x1[RAR_34];

	lhy = _Q_34x1[LHY_34];
	lhr = _Q_34x1[LHR_34];
	lhp = _Q_34x1[LHP_34];
	lkn = _Q_34x1[LKN_34];
	lap = _Q_34x1[LAP_34];
	lar = _Q_34x1[LAR_34];

	wst = _Q_34x1[WST_34];
	rsp = _Q_34x1[RSP_34];
	rsr = _Q_34x1[RSR_34];
	rsy = _Q_34x1[RSY_34];
	reb = _Q_34x1[REB_34];
	rwy = _Q_34x1[RWY_34];
	rwp = _Q_34x1[RWP_34];
	lsp = _Q_34x1[LSP_34];
	lsr = _Q_34x1[LSR_34];
	lsy = _Q_34x1[LSY_34];
	leb = _Q_34x1[LEB_34];
	lwy = _Q_34x1[LWY_34];
	lwp = _Q_34x1[LWP_34];
	rwy2 = _Q_34x1[RWY2_34];
	lwy2 = _Q_34x1[LWY2_34];

	QT2DC(qPEL, _dcPEL_3x3);
	
	RZ(rhy, _Rz_RHY_3x3);
	RX(rhr, _Rx_RHR_3x3);
	RY(rhp, _Ry_RHP_3x3);
	RY(rkn, _Ry_RKN_3x3);
	RY(rap, _Ry_RAP_3x3);
	RX(rar, _Rx_RAR_3x3);

	RZ(lhy, _Rz_LHY_3x3);
	RX(lhr, _Rx_LHR_3x3);
	RY(lhp, _Ry_LHP_3x3);
	RY(lkn, _Ry_LKN_3x3);
	RY(lap, _Ry_LAP_3x3);
	RX(lar, _Rx_LAR_3x3);

	RZ(wst, _Rz_WST_3x3);
	
	RY(rsp, _Ry_RSP_3x3);
	RX(rsr, _Rx_RSR_3x3);
	RZ(rsy, _Rz_RSY_3x3);
	RY(reb, _Ry_REB_3x3);
	RZ(rwy, _Rz_RWY_3x3);
	RY(rwp, _Ry_RWP_3x3);

	RY(lsp, _Ry_LSP_3x3);
	RX(lsr, _Rx_LSR_3x3);
	RZ(lsy, _Rz_LSY_3x3);
	RY(leb, _Ry_LEB_3x3);
	RZ(lwy, _Rz_LWY_3x3);
	RY(lwp, _Ry_LWP_3x3);

	RZ(rwy2, _Rz_RWY2_3x3);
	RZ(lwy2, _Rz_LWY2_3x3);

	RY(PI, _Ry_PI_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_RHR_3x3,3,3, (const float**)_Ry_RHP_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcRUL_3x3);

	mult_mm((const float**)_dcRUL_3x3,3,3, (const float**)_Ry_RKN_3x3,3, _dcRLL_3x3);
	
	mult_mm((const float**)_Ry_RAP_3x3,3,3, (const float**)_Rx_RAR_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_dcRLL_3x3,3,3, (const float**)_TEMP1_34x34,3, _dcRF_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_LHR_3x3,3,3, (const float**)_Ry_LHP_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcLUL_3x3);

	mult_mm((const float**)_dcLUL_3x3,3,3, (const float**)_Ry_LKN_3x3,3, _dcLLL_3x3);
	
	mult_mm((const float**)_Ry_LAP_3x3,3,3, (const float**)_Rx_LAR_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_dcLLL_3x3,3,3, (const float**)_TEMP1_34x34,3, _dcLF_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_WST_3x3,3, _dcTOR_3x3);

	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_RSP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_RSR_3x3,3,3, (const float**)_Rz_RSY_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcRUA_3x3);
	
	mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REB_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_RWY_3x3,3, _dcRLA_3x3);
	mult_mm((const float**)_dcRLA_3x3,3,3, (const float**)_Ry_RWP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_RWY2_3x3,3, _dcRH_3x3);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Ry_PI_3x3,3, _dcRS_3x3);

	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_LSP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_LSR_3x3,3,3, (const float**)_Rz_LSY_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcLUA_3x3);

	mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEB_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_LWY_3x3,3, _dcLLA_3x3);
	mult_mm((const float**)_dcLLA_3x3,3,3, (const float**)_Ry_LWP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_LWY2_3x3,3, _dcLH_3x3);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Ry_PI_3x3,3, _dcLS_3x3);
	
	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_RPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pRPEL);

	mult_mv((const float**)_dcRUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRPEL,3, temp3_3x1, pRKN);

	mult_mv((const float**)_dcRLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, _pRANK_3x1);

	mult_mv((const float**)_dcRF_3x3,3,3, _LINK_FOOT, temp3_3x1);
	sum_vv(_pRANK_3x1,3, temp3_3x1, _pRF_3x1);


	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_LPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pLPEL);

	mult_mv((const float**)_dcLUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLPEL,3, temp3_3x1, pLKN);

	mult_mv((const float**)_dcLLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, _pLANK_3x1);

	mult_mv((const float**)_dcLF_3x3,3,3, _LINK_FOOT, temp3_3x1);
	sum_vv(_pLANK_3x1,3, temp3_3x1, _pLF_3x1);


	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_WST, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pWST);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_RSHLD, temp3_3x1); 
	sum_vv(pWST,3, temp3_3x1, pRSHLD);

	mult_mv((const float**)_dcRUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pRSHLD,3, temp3_3x1, pRELB);

	temp4_3x1[1] = _LINK_LARM[1];
	temp4_3x1[2] = 0.;
	temp4_3x1[3] = 0.;
	mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REB_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pRELB,3, temp3_3x1, pRELB2);
	temp4_3x1[1] = 0.;
	temp4_3x1[2] = _LINK_LARM[2];
	temp4_3x1[3] = _LINK_LARM[3];
	mult_mv((const float**)_dcRLA_3x3,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pRELB2,3, temp3_3x1, _pRWR_3x1);

	mult_mv((const float**)_dcRH_3x3,3,3, _LINK_HAND, temp3_3x1); 
	sum_vv(_pRWR_3x1,3, temp3_3x1, _pRH_3x1);

	mult_mv((const float**)_dcRS_3x3,3,3, _LINK_STICK, temp3_3x1); 
	sum_vv(_pRWR_3x1,3, temp3_3x1, _pRS_3x1);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_LSHLD, temp3_3x1); 
	sum_vv(pWST,3, temp3_3x1, pLSHLD);

	mult_mv((const float**)_dcLUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pLSHLD,3, temp3_3x1, pLELB);

	temp4_3x1[1] = _LINK_LARM[1];
	temp4_3x1[2] = 0.;
	temp4_3x1[3] = 0.;
	mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEB_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pLELB,3, temp3_3x1, pLELB2);
	temp4_3x1[1] = 0.;
	temp4_3x1[2] = _LINK_LARM[2];
	temp4_3x1[3] = _LINK_LARM[3];
	mult_mv((const float**)_dcLLA_3x3,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pLELB2,3, temp3_3x1, _pLWR_3x1);

	mult_mv((const float**)_dcLH_3x3,3,3, _LINK_HAND, temp3_3x1); 
	sum_vv(_pLWR_3x1,3, temp3_3x1, _pLH_3x1);

	mult_mv((const float**)_dcLS_3x3,3,3, _LINK_STICK, temp3_3x1); 
	sum_vv(_pLWR_3x1,3, temp3_3x1, _pLS_3x1);

	mult_mv((const float**)_dcPEL_3x3,3,3, _C_PEL, temp3_3x1);
	sum_vv(pPC,3, temp3_3x1, cPEL);

	mult_mv((const float**)_dcTOR_3x3,3,3, _C_TOR, temp3_3x1);
	sum_vv(pWST,3, temp3_3x1, cTOR);

	mult_mv((const float**)_dcRUL_3x3,3,3, _C_RUL, temp3_3x1);
	sum_vv(pRPEL,3, temp3_3x1, cRUL);

	mult_mv((const float**)_dcRLL_3x3,3,3, _C_RLL, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, cRLL);

	mult_mv((const float**)_dcRF_3x3,3,3, _C_RF, temp3_3x1);
	sum_vv(_pRANK_3x1,3, temp3_3x1, cRF);

	mult_mv((const float**)_dcLUL_3x3,3,3, _C_LUL, temp3_3x1);
	sum_vv(pLPEL,3, temp3_3x1, cLUL);

	mult_mv((const float**)_dcLLL_3x3,3,3, _C_LLL, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, cLLL);

	mult_mv((const float**)_dcLF_3x3,3,3, _C_LF, temp3_3x1);
	sum_vv(_pLANK_3x1,3, temp3_3x1, cLF);

	mult_mv((const float**)_dcRUA_3x3,3,3, _C_RUA, temp3_3x1);
	sum_vv(pRSHLD,3, temp3_3x1, cRUA);

	mult_mv((const float**)_dcRLA_3x3,3,3, _C_RLA, temp3_3x1); 
	sum_vv(pRELB2,3, temp3_3x1, cRLA);

	mult_mv((const float**)_dcRH_3x3,3,3, _C_RHAND, temp3_3x1);
	sum_vv(_pRWR_3x1,3, temp3_3x1, cRH);

	mult_mv((const float**)_dcLUA_3x3,3,3, _C_LUA, temp3_3x1);
	sum_vv(pLSHLD,3, temp3_3x1, cLUA);

	mult_mv((const float**)_dcLLA_3x3,3,3, _C_LLA, temp3_3x1); 
	sum_vv(pLELB2,3, temp3_3x1, cLLA);
	
	mult_mv((const float**)_dcLH_3x3,3,3, _C_LHAND, temp3_3x1);
	sum_vv(_pLWR_3x1,3, temp3_3x1, cLH);

	mult_mv((const float**)_dcPEL_3x3,3,3, _AXIS_Z, axis_wst);

	mult_mv((const float**)_dcPEL_3x3,3,3, _AXIS_Z, axis_rhy);
	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, _AXIS_X, axis_rhr);
	mult_mv((const float**)_dcRUL_3x3,3,3, _AXIS_Y, axis_rhp);
	axis_rkn[1] = axis_rhp[1];
	axis_rkn[2] = axis_rhp[2];
	axis_rkn[3] = axis_rhp[3];
	axis_rap[1] = axis_rhp[1];
	axis_rap[2] = axis_rhp[2];
	axis_rap[3] = axis_rhp[3];
	mult_mv((const float**)_dcRF_3x3,3,3, _AXIS_X, axis_rar);

	axis_lhy[1] = axis_rhy[1];
	axis_lhy[2] = axis_rhy[2];
	axis_lhy[3] = axis_rhy[3];
	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, _AXIS_X, axis_lhr);
	mult_mv((const float**)_dcLUL_3x3,3,3, _AXIS_Y, axis_lhp);
	axis_lkn[1] = axis_lhp[1];
	axis_lkn[2] = axis_lhp[2];
	axis_lkn[3] = axis_lhp[3];
	axis_lap[1] = axis_lhp[1];
	axis_lap[2] = axis_lhp[2];
	axis_lap[3] = axis_lhp[3];
	mult_mv((const float**)_dcLF_3x3,3,3, _AXIS_X, axis_lar);

	mult_mv((const float**)_dcTOR_3x3,3,3, _AXIS_Y, axis_rsp);
	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_RSP_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, _AXIS_X, axis_rsr);
	mult_mv((const float**)_dcRUA_3x3,3,3, _AXIS_Z, axis_rsy);
	mult_mv((const float**)_dcRUA_3x3,3,3, _AXIS_Y, axis_reb);
	mult_mv((const float**)_dcRLA_3x3,3,3, _AXIS_Z, axis_rwy);
	mult_mv((const float**)_dcRLA_3x3,3,3, _AXIS_Y, axis_rwp);
	mult_mv((const float**)_dcRH_3x3,3,3, _AXIS_Z, axis_rwy2);

	axis_lsp[1] = axis_rsp[1];
	axis_lsp[2] = axis_rsp[2];
	axis_lsp[3] = axis_rsp[3];
	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_LSP_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, _AXIS_X, axis_lsr);
	mult_mv((const float**)_dcLUA_3x3,3,3, _AXIS_Z, axis_lsy);
	mult_mv((const float**)_dcLUA_3x3,3,3, _AXIS_Y, axis_leb);
	mult_mv((const float**)_dcLLA_3x3,3,3, _AXIS_Z, axis_lwy);
	mult_mv((const float**)_dcLLA_3x3,3,3, _AXIS_Y, axis_lwp);
	mult_mv((const float**)_dcLH_3x3,3,3, _AXIS_Z, axis_lwy2);

	ftemp = (m_PEL/m_TOTAL);
	jCOM_PEL_3x33[1][1] = ftemp;
	jCOM_PEL_3x33[2][2] = ftemp;
	jCOM_PEL_3x33[3][3] = ftemp;
	diff_vv(cPEL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_PEL_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_PEL_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_PEL_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_PEL_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_PEL_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_PEL_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_PEL_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_PEL_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_PEL_3x33[3][6] = ftemp*temp4_3x1[3];

	ftemp = (m_TOR/m_TOTAL);
	jCOM_TOR_3x33[1][1] = ftemp;
	jCOM_TOR_3x33[2][2] = ftemp;
	jCOM_TOR_3x33[3][3] = ftemp;
	diff_vv(cTOR,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_TOR_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_TOR_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_TOR_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_TOR_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_TOR_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_TOR_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_TOR_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_TOR_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_TOR_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cTOR,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_TOR_3x33[1][7] = ftemp*temp4_3x1[1];
	jCOM_TOR_3x33[2][7] = ftemp*temp4_3x1[2];
	jCOM_TOR_3x33[3][7] = ftemp*temp4_3x1[3];

	ftemp = (m_ULEG/m_TOTAL);
	jCOM_RUL_3x33[1][1] = ftemp;
	jCOM_RUL_3x33[2][2] = ftemp;
	jCOM_RUL_3x33[3][3] = ftemp;
	diff_vv(cRUL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRUL,3,pRPEL, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x33[1][8] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x33[2][8] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x33[3][8] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x33[1][9] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x33[2][9] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x33[3][9] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x33[1][10] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x33[2][10] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x33[3][10] = ftemp*temp4_3x1[3];

	ftemp = (m_LLEG/m_TOTAL);
	jCOM_RLL_3x33[1][1] = ftemp;
	jCOM_RLL_3x33[2][2] = ftemp;
	jCOM_RLL_3x33[3][3] = ftemp;
	diff_vv(cRLL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRLL,3,pRPEL, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x33[1][8] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x33[2][8] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x33[3][8] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x33[1][9] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x33[2][9] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x33[3][9] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x33[1][10] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x33[2][10] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x33[3][10] = ftemp*temp4_3x1[3];
	diff_vv(cRLL,3,pRKN, temp3_3x1);
	cross(1.f,axis_rkn, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x33[1][11] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x33[2][11] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x33[3][11] = ftemp*temp4_3x1[3];

	ftemp = (m_FOOT/m_TOTAL);
	jCOM_RF_3x33[1][1] = ftemp;
	jCOM_RF_3x33[2][2] = ftemp;
	jCOM_RF_3x33[3][3] = ftemp;
	diff_vv(cRF,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRF,3,pRPEL, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][8] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][8] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][8] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][9] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][9] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][9] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][10] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][10] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][10] = ftemp*temp4_3x1[3];
	diff_vv(cRF,3,pRKN, temp3_3x1);
	cross(1.f,axis_rkn, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][11] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][11] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][11] = ftemp*temp4_3x1[3];
	diff_vv(cRF,3,_pRANK_3x1, temp3_3x1);
	cross(1.f,axis_rap, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][12] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][12] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][12] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rar, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][13] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][13] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][13] = ftemp*temp4_3x1[3];

	ftemp = (m_ULEG/m_TOTAL);
	jCOM_LUL_3x33[1][1] = ftemp;
	jCOM_LUL_3x33[2][2] = ftemp;
	jCOM_LUL_3x33[3][3] = ftemp;
	diff_vv(cLUL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLUL,3,pLPEL, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x33[1][14] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x33[2][14] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x33[3][14] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x33[1][15] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x33[2][15] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x33[3][15] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x33[1][16] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x33[2][16] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x33[3][16] = ftemp*temp4_3x1[3];

	ftemp = (m_LLEG/m_TOTAL);
	jCOM_LLL_3x33[1][1] = ftemp;
	jCOM_LLL_3x33[2][2] = ftemp;
	jCOM_LLL_3x33[3][3] = ftemp;
	diff_vv(cLLL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLLL,3,pLPEL, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x33[1][14] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x33[2][14] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x33[3][14] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x33[1][15] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x33[2][15] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x33[3][15] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x33[1][16] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x33[2][16] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x33[3][16] = ftemp*temp4_3x1[3];
	diff_vv(cLLL,3,pLKN, temp3_3x1);
	cross(1.f,axis_lkn, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x33[1][17] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x33[2][17] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x33[3][17] = ftemp*temp4_3x1[3];

	ftemp = (m_FOOT/m_TOTAL);
	jCOM_LF_3x33[1][1] = ftemp;
	jCOM_LF_3x33[2][2] = ftemp;
	jCOM_LF_3x33[3][3] = ftemp;
	diff_vv(cLF,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLF,3,pLPEL, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][14] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][14] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][14] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][15] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][15] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][15] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][16] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][16] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][16] = ftemp*temp4_3x1[3];
	diff_vv(cLF,3,pLKN, temp3_3x1);
	cross(1.f,axis_lkn, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][17] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][17] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][17] = ftemp*temp4_3x1[3];
	diff_vv(cLF,3,_pLANK_3x1, temp3_3x1);
	cross(1.f,axis_lap, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][18] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][18] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][18] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lar, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][19] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][19] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][19] = ftemp*temp4_3x1[3];

	ftemp = (m_UARM/m_TOTAL);
	jCOM_RUA_3x33[1][1] = ftemp;
	jCOM_RUA_3x33[2][2] = ftemp;
	jCOM_RUA_3x33[3][3] = ftemp;
	diff_vv(cRUA,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRUA,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x33[1][7] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x33[2][7] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x33[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cRUA,3,pRSHLD, temp3_3x1);
	cross(1.f,axis_rsp, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x33[1][20] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x33[2][20] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x33[3][20] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsr, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x33[1][21] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x33[2][21] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x33[3][21] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsy, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x33[1][22] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x33[2][22] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x33[3][22] = ftemp*temp4_3x1[3];


	ftemp = (m_LARM/m_TOTAL);
	jCOM_RLA_3x33[1][1] = ftemp;
	jCOM_RLA_3x33[2][2] = ftemp;
	jCOM_RLA_3x33[3][3] = ftemp;
	diff_vv(cRLA,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRLA,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][7] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][7] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cRLA,3,pRSHLD, temp3_3x1);
	cross(1.f,axis_rsp, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][20] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][20] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][20] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsr, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][21] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][21] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][21] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsy, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][22] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][22] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][22] = ftemp*temp4_3x1[3];
	diff_vv(cRLA,3,pRELB, temp3_3x1);
	cross(1.f,axis_reb, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][23] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][23] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][23] = ftemp*temp4_3x1[3];
	diff_vv(cRLA,3,pRELB2, temp3_3x1);
	cross(1.f,axis_rwy, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][24] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][24] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][24] = ftemp*temp4_3x1[3];

	ftemp = (m_RHAND/m_TOTAL);
	jCOM_RH_3x33[1][1] = ftemp;
	jCOM_RH_3x33[2][2] = ftemp;
	jCOM_RH_3x33[3][3] = ftemp;
	diff_vv(cRH,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRH,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][7] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][7] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cRH,3,pRSHLD, temp3_3x1);
	cross(1.f,axis_rsp, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][20] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][20] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][20] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsr, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][21] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][21] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][21] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsy, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][22] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][22] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][22] = ftemp*temp4_3x1[3];
	diff_vv(cRH,3,pRELB, temp3_3x1);
	cross(1.f,axis_reb, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][23] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][23] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][23] = ftemp*temp4_3x1[3];
	diff_vv(cRH,3,pRELB2, temp3_3x1);
	cross(1.f,axis_rwy, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][24] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][24] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][24] = ftemp*temp4_3x1[3];
	diff_vv(cRH,3,_pRWR_3x1, temp3_3x1);
	cross(1.f,axis_rwp, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][25] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][25] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][25] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rwy2, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][32] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][32] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][32] = ftemp*temp4_3x1[3];

	ftemp = (m_UARM/m_TOTAL);
	jCOM_LUA_3x33[1][1] = ftemp;
	jCOM_LUA_3x33[2][2] = ftemp;
	jCOM_LUA_3x33[3][3] = ftemp;
	diff_vv(cLUA,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLUA,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x33[1][7] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x33[2][7] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x33[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cLUA,3,pLSHLD, temp3_3x1);
	cross(1.f,axis_lsp, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x33[1][26] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x33[2][26] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x33[3][26] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsr, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x33[1][27] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x33[2][27] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x33[3][27] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsy, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x33[1][28] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x33[2][28] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x33[3][28] = ftemp*temp4_3x1[3];

	ftemp = (m_LARM/m_TOTAL);
	jCOM_LLA_3x33[1][1] = ftemp;
	jCOM_LLA_3x33[2][2] = ftemp;
	jCOM_LLA_3x33[3][3] = ftemp;
	diff_vv(cLLA,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLLA,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][7] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][7] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cLLA,3,pLSHLD, temp3_3x1);
	cross(1.f,axis_lsp, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][26] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][26] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][26] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsr, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][27] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][27] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][27] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsy, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][28] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][28] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][28] = ftemp*temp4_3x1[3];
	diff_vv(cLLA,3,pLELB, temp3_3x1);
	cross(1.f,axis_leb, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][29] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][29] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][29] = ftemp*temp4_3x1[3];
	diff_vv(cLLA,3,pLELB2, temp3_3x1);
	cross(1.f,axis_lwy, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][30] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][30] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][30] = ftemp*temp4_3x1[3];
	
	ftemp = (m_LHAND/m_TOTAL);
	jCOM_LH_3x33[1][1] = ftemp;
	jCOM_LH_3x33[2][2] = ftemp;
	jCOM_LH_3x33[3][3] = ftemp;
	diff_vv(cLH,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLH,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][7] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][7] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cLH,3,pLSHLD, temp3_3x1);
	cross(1.f,axis_lsp, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][26] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][26] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][26] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsr, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][27] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][27] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][27] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsy, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][28] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][28] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][28] = ftemp*temp4_3x1[3];
	diff_vv(cLH,3,pLELB, temp3_3x1);
	cross(1.f,axis_leb, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][29] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][29] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][29] = ftemp*temp4_3x1[3];
	diff_vv(cLH,3,pLELB2, temp3_3x1);
	cross(1.f,axis_lwy, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][30] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][30] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][30] = ftemp*temp4_3x1[3];
	diff_vv(cLH,3,_pLWR_3x1, temp3_3x1);
	cross(1.f,axis_lwp, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][31] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][31] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][31] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lwy2, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][33] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][33] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][33] = ftemp*temp4_3x1[3];

	for(i=1; i<=3; i++)
	{
		_jCOM_3x33[i][4] = jCOM_PEL_3x33[i][4] + jCOM_TOR_3x33[i][4] + jCOM_RUL_3x33[i][4] + jCOM_RLL_3x33[i][4] + jCOM_RF_3x33[i][4] + jCOM_LUL_3x33[i][4] + jCOM_LLL_3x33[i][4] + jCOM_LF_3x33[i][4] + jCOM_RUA_3x33[i][4] + jCOM_RLA_3x33[i][4] + jCOM_LUA_3x33[i][4] + jCOM_LLA_3x33[i][4] + jCOM_RH_3x33[i][4] + jCOM_LH_3x33[i][4];
		_jCOM_3x33[i][5] = jCOM_PEL_3x33[i][5] + jCOM_TOR_3x33[i][5] + jCOM_RUL_3x33[i][5] + jCOM_RLL_3x33[i][5] + jCOM_RF_3x33[i][5] + jCOM_LUL_3x33[i][5] + jCOM_LLL_3x33[i][5] + jCOM_LF_3x33[i][5] + jCOM_RUA_3x33[i][5] + jCOM_RLA_3x33[i][5] + jCOM_LUA_3x33[i][5] + jCOM_LLA_3x33[i][5] + jCOM_RH_3x33[i][5] + jCOM_LH_3x33[i][5];
		_jCOM_3x33[i][6] = jCOM_PEL_3x33[i][6] + jCOM_TOR_3x33[i][6] + jCOM_RUL_3x33[i][6] + jCOM_RLL_3x33[i][6] + jCOM_RF_3x33[i][6] + jCOM_LUL_3x33[i][6] + jCOM_LLL_3x33[i][6] + jCOM_LF_3x33[i][6] + jCOM_RUA_3x33[i][6] + jCOM_RLA_3x33[i][6] + jCOM_LUA_3x33[i][6] + jCOM_LLA_3x33[i][6] + jCOM_RH_3x33[i][6] + jCOM_LH_3x33[i][6];
		
		_jCOM_3x33[i][8] = jCOM_RUL_3x33[i][8] + jCOM_RLL_3x33[i][8] + jCOM_RF_3x33[i][8];
		_jCOM_3x33[i][9] = jCOM_RUL_3x33[i][9] + jCOM_RLL_3x33[i][9] + jCOM_RF_3x33[i][9];
		_jCOM_3x33[i][10] = jCOM_RUL_3x33[i][10] + jCOM_RLL_3x33[i][10] + jCOM_RF_3x33[i][10];
		_jCOM_3x33[i][11] = jCOM_RLL_3x33[i][11] + jCOM_RF_3x33[i][11];
		_jCOM_3x33[i][12] = jCOM_RF_3x33[i][12];
		_jCOM_3x33[i][13] = jCOM_RF_3x33[i][13];
		_jCOM_3x33[i][14] = jCOM_LUL_3x33[i][14] + jCOM_LLL_3x33[i][14] + jCOM_LF_3x33[i][14];
		_jCOM_3x33[i][15] = jCOM_LUL_3x33[i][15] + jCOM_LLL_3x33[i][15] + jCOM_LF_3x33[i][15];
		_jCOM_3x33[i][16] = jCOM_LUL_3x33[i][16] + jCOM_LLL_3x33[i][16] + jCOM_LF_3x33[i][16];
		_jCOM_3x33[i][17] = jCOM_LLL_3x33[i][17] + jCOM_LF_3x33[i][17];
		_jCOM_3x33[i][18] = jCOM_LF_3x33[i][18];
		_jCOM_3x33[i][19] = jCOM_LF_3x33[i][19];
		
		_jCOM_3x33[i][7] = jCOM_TOR_3x33[i][7] + jCOM_RUA_3x33[i][7] + jCOM_RLA_3x33[i][7] + jCOM_LUA_3x33[i][7] + jCOM_LLA_3x33[i][7] + jCOM_RH_3x33[i][7] + jCOM_LH_3x33[i][7];
		_jCOM_3x33[i][20] = jCOM_RUA_3x33[i][20] + jCOM_RLA_3x33[i][20] + jCOM_RH_3x33[i][20];
		_jCOM_3x33[i][21] = jCOM_RUA_3x33[i][21] + jCOM_RLA_3x33[i][21] + jCOM_RH_3x33[i][21];
		_jCOM_3x33[i][22] = jCOM_RUA_3x33[i][22] + jCOM_RLA_3x33[i][22] + jCOM_RH_3x33[i][22];
		_jCOM_3x33[i][23] = jCOM_RLA_3x33[i][23] + jCOM_RH_3x33[i][23];
		_jCOM_3x33[i][24] = jCOM_RLA_3x33[i][24] + jCOM_RH_3x33[i][24];
		_jCOM_3x33[i][25] = jCOM_RH_3x33[i][25];

		_jCOM_3x33[i][26] = jCOM_LUA_3x33[i][26] + jCOM_LLA_3x33[i][26] + jCOM_LH_3x33[i][26];
		_jCOM_3x33[i][27] = jCOM_LUA_3x33[i][27] + jCOM_LLA_3x33[i][27] + jCOM_LH_3x33[i][27];	
		_jCOM_3x33[i][28] = jCOM_LUA_3x33[i][28] + jCOM_LLA_3x33[i][28] + jCOM_LH_3x33[i][28];	
		_jCOM_3x33[i][29] = jCOM_LLA_3x33[i][29] + jCOM_LH_3x33[i][29];	
		_jCOM_3x33[i][30] = jCOM_LLA_3x33[i][30] + jCOM_LH_3x33[i][30];
		_jCOM_3x33[i][31] = jCOM_LH_3x33[i][31];

		_jCOM_3x33[i][32] = jCOM_RH_3x33[i][32];
		_jCOM_3x33[i][33] = jCOM_LH_3x33[i][33];
	}

	_pCOM_3x1[1] = ((m_PEL*cPEL[1] +m_TOR*cTOR[1] + m_ULEG*(cRUL[1]+cLUL[1]) + m_LLEG*(cRLL[1]+cLLL[1]) +m_FOOT*(cRF[1]+cLF[1]) + m_UARM*(cRUA[1]+cLUA[1]) + m_LARM*(cRLA[1]+cLLA[1]) + m_RHAND*cRH[1] + m_LHAND*cLH[1])/m_TOTAL);
	_pCOM_3x1[2] = ((m_PEL*cPEL[2] +m_TOR*cTOR[2] + m_ULEG*(cRUL[2]+cLUL[2]) + m_LLEG*(cRLL[2]+cLLL[2]) +m_FOOT*(cRF[2]+cLF[2]) + m_UARM*(cRUA[2]+cLUA[2]) + m_LARM*(cRLA[2]+cLLA[2]) + m_RHAND*cRH[2] + m_LHAND*cLH[2])/m_TOTAL);
	_pCOM_3x1[3] = ((m_PEL*cPEL[3] +m_TOR*cTOR[3] + m_ULEG*(cRUL[3]+cLUL[3]) + m_LLEG*(cRLL[3]+cLLL[3]) +m_FOOT*(cRF[3]+cLF[3]) + m_UARM*(cRUA[3]+cLUA[3]) + m_LARM*(cRLA[3]+cLLA[3]) + m_RHAND*cRH[3] + m_LHAND*cLH[3])/m_TOTAL);

	_jRF_6x33[1][1] = 1.f;
	_jRF_6x33[2][2] = 1.f;
	_jRF_6x33[3][3] = 1.f;
	_jRF_6x33[4][4] = 1.f;
	_jRF_6x33[5][5] = 1.f;
	_jRF_6x33[6][6] = 1.f;

	diff_vv(_pRF_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][4] = temp4_3x1[1];
	_jRF_6x33[2][4] = temp4_3x1[2];
	_jRF_6x33[3][4] = temp4_3x1[3];
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][5] = temp4_3x1[1];
	_jRF_6x33[2][5] = temp4_3x1[2];
	_jRF_6x33[3][5] = temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][6] = temp4_3x1[1];
	_jRF_6x33[2][6] = temp4_3x1[2];
	_jRF_6x33[3][6] = temp4_3x1[3];

	diff_vv(_pRF_3x1,3,pRPEL, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][8] = temp4_3x1[1];
	_jRF_6x33[2][8] = temp4_3x1[2];
	_jRF_6x33[3][8] = temp4_3x1[3];
	_jRF_6x33[4][8] = axis_rhy[1];
	_jRF_6x33[5][8] = axis_rhy[2];
	_jRF_6x33[6][8] = axis_rhy[3];
	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][9] = temp4_3x1[1];
	_jRF_6x33[2][9] = temp4_3x1[2];
	_jRF_6x33[3][9] = temp4_3x1[3];
	_jRF_6x33[4][9] = axis_rhr[1];
	_jRF_6x33[5][9] = axis_rhr[2];
	_jRF_6x33[6][9] = axis_rhr[3];
	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][10] = temp4_3x1[1];
	_jRF_6x33[2][10] = temp4_3x1[2];
	_jRF_6x33[3][10] = temp4_3x1[3];
	_jRF_6x33[4][10] = axis_rhp[1];
	_jRF_6x33[5][10] = axis_rhp[2];
	_jRF_6x33[6][10] = axis_rhp[3];

	diff_vv(_pRF_3x1,3,pRKN, temp3_3x1);
	cross(1.f,axis_rkn, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][11] = temp4_3x1[1];
	_jRF_6x33[2][11] = temp4_3x1[2];
	_jRF_6x33[3][11] = temp4_3x1[3];
	_jRF_6x33[4][11] = axis_rkn[1];
	_jRF_6x33[5][11] = axis_rkn[2];
	_jRF_6x33[6][11] = axis_rkn[3];

	diff_vv(_pRF_3x1,3,_pRANK_3x1, temp3_3x1);
	cross(1.f,axis_rap, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][12] = temp4_3x1[1];
	_jRF_6x33[2][12] = temp4_3x1[2];
	_jRF_6x33[3][12] = temp4_3x1[3];
	_jRF_6x33[4][12] = axis_rap[1];
	_jRF_6x33[5][12] = axis_rap[2];
	_jRF_6x33[6][12] = axis_rap[3];
	cross(1.f,axis_rar, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][13] = temp4_3x1[1];
	_jRF_6x33[2][13] = temp4_3x1[2];
	_jRF_6x33[3][13] = temp4_3x1[3];
	_jRF_6x33[4][13] = axis_rar[1];
	_jRF_6x33[5][13] = axis_rar[2];
	_jRF_6x33[6][13] = axis_rar[3];


	_jLF_6x33[1][1] = 1.f;
	_jLF_6x33[2][2] = 1.f;
	_jLF_6x33[3][3] = 1.f;
	_jLF_6x33[4][4] = 1.f;
	_jLF_6x33[5][5] = 1.f;
	_jLF_6x33[6][6] = 1.f;

	diff_vv(_pLF_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][4] = temp4_3x1[1];
	_jLF_6x33[2][4] = temp4_3x1[2];
	_jLF_6x33[3][4] = temp4_3x1[3];
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][5] = temp4_3x1[1];
	_jLF_6x33[2][5] = temp4_3x1[2];
	_jLF_6x33[3][5] = temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][6] = temp4_3x1[1];
	_jLF_6x33[2][6] = temp4_3x1[2];
	_jLF_6x33[3][6] = temp4_3x1[3];

	diff_vv(_pLF_3x1,3,pLPEL, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][14] = temp4_3x1[1];
	_jLF_6x33[2][14] = temp4_3x1[2];
	_jLF_6x33[3][14] = temp4_3x1[3];
	_jLF_6x33[4][14] = axis_lhy[1];
	_jLF_6x33[5][14] = axis_lhy[2];
	_jLF_6x33[6][14] = axis_lhy[3];
	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][15] = temp4_3x1[1];
	_jLF_6x33[2][15] = temp4_3x1[2];
	_jLF_6x33[3][15] = temp4_3x1[3];
	_jLF_6x33[4][15] = axis_lhr[1];
	_jLF_6x33[5][15] = axis_lhr[2];
	_jLF_6x33[6][15] = axis_lhr[3];
	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][16] = temp4_3x1[1];
	_jLF_6x33[2][16] = temp4_3x1[2];
	_jLF_6x33[3][16] = temp4_3x1[3];
	_jLF_6x33[4][16] = axis_lhp[1];
	_jLF_6x33[5][16] = axis_lhp[2];
	_jLF_6x33[6][16] = axis_lhp[3];

	diff_vv(_pLF_3x1,3,pLKN, temp3_3x1);
	cross(1.f,axis_lkn, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][17] = temp4_3x1[1];
	_jLF_6x33[2][17] = temp4_3x1[2];
	_jLF_6x33[3][17] = temp4_3x1[3];
	_jLF_6x33[4][17] = axis_lkn[1];
	_jLF_6x33[5][17] = axis_lkn[2];
	_jLF_6x33[6][17] = axis_lkn[3];

	diff_vv(_pLF_3x1,3,_pLANK_3x1, temp3_3x1);
	cross(1.f,axis_lap, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][18] = temp4_3x1[1];
	_jLF_6x33[2][18] = temp4_3x1[2];
	_jLF_6x33[3][18] = temp4_3x1[3];
	_jLF_6x33[4][18] = axis_lap[1];
	_jLF_6x33[5][18] = axis_lap[2];
	_jLF_6x33[6][18] = axis_lap[3];
	cross(1.f,axis_lar, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][19] = temp4_3x1[1];
	_jLF_6x33[2][19] = temp4_3x1[2];
	_jLF_6x33[3][19] = temp4_3x1[3];
	_jLF_6x33[4][19] = axis_lar[1];
	_jLF_6x33[5][19] = axis_lar[2];
	_jLF_6x33[6][19] = axis_lar[3];

	_jRH_6x33[1][1] = 1.f;
	_jRH_6x33[2][2] = 1.f;
	_jRH_6x33[3][3] = 1.f;
	_jRH_6x33[4][4] = 1.f;
	_jRH_6x33[5][5] = 1.f;
	_jRH_6x33[6][6] = 1.f;

	diff_vv(_pRH_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][4] = temp4_3x1[1];
	_jRH_6x33[2][4] = temp4_3x1[2];
	_jRH_6x33[3][4] = temp4_3x1[3];
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][5] = temp4_3x1[1];
	_jRH_6x33[2][5] = temp4_3x1[2];
	_jRH_6x33[3][5] = temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][6] = temp4_3x1[1];
	_jRH_6x33[2][6] = temp4_3x1[2];
	_jRH_6x33[3][6] = temp4_3x1[3];

	diff_vv(_pRH_3x1,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][7] = temp4_3x1[1];
	_jRH_6x33[2][7] = temp4_3x1[2];
	_jRH_6x33[3][7] = temp4_3x1[3];
	_jRH_6x33[4][7] = axis_wst[1];
	_jRH_6x33[5][7] = axis_wst[2];
	_jRH_6x33[6][7] = axis_wst[3];

	diff_vv(_pRH_3x1,3,pRSHLD, temp3_3x1);
	cross(1.f,axis_rsp, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][20] = temp4_3x1[1];
	_jRH_6x33[2][20] = temp4_3x1[2];
	_jRH_6x33[3][20] = temp4_3x1[3];
	_jRH_6x33[4][20] = axis_rsp[1];
	_jRH_6x33[5][20] = axis_rsp[2];
	_jRH_6x33[6][20] = axis_rsp[3];
	cross(1.f,axis_rsr, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][21] = temp4_3x1[1];
	_jRH_6x33[2][21] = temp4_3x1[2];
	_jRH_6x33[3][21] = temp4_3x1[3];
	_jRH_6x33[4][21] = axis_rsr[1];
	_jRH_6x33[5][21] = axis_rsr[2];
	_jRH_6x33[6][21] = axis_rsr[3];
	cross(1.f,axis_rsy, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][22] = temp4_3x1[1];
	_jRH_6x33[2][22] = temp4_3x1[2];
	_jRH_6x33[3][22] = temp4_3x1[3];
	_jRH_6x33[4][22] = axis_rsy[1];
	_jRH_6x33[5][22] = axis_rsy[2];
	_jRH_6x33[6][22] = axis_rsy[3];

	diff_vv(_pRH_3x1,3,pRELB, temp3_3x1);
	cross(1.f,axis_reb, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][23] = temp4_3x1[1];
	_jRH_6x33[2][23] = temp4_3x1[2];
	_jRH_6x33[3][23] = temp4_3x1[3];
	_jRH_6x33[4][23] = axis_reb[1];
	_jRH_6x33[5][23] = axis_reb[2];
	_jRH_6x33[6][23] = axis_reb[3];
	
	diff_vv(_pRH_3x1,3, pRELB2, temp3_3x1);
	cross(1.f,axis_rwy, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][24] = temp4_3x1[1];
	_jRH_6x33[2][24] = temp4_3x1[2];
	_jRH_6x33[3][24] = temp4_3x1[3];
	_jRH_6x33[4][24] = axis_rwy[1];
	_jRH_6x33[5][24] = axis_rwy[2];
	_jRH_6x33[6][24] = axis_rwy[3];

	diff_vv(_pRH_3x1,3,_pRWR_3x1, temp3_3x1);
	cross(1.f,axis_rwp, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][25] = temp4_3x1[1];
	_jRH_6x33[2][25] = temp4_3x1[2];
	_jRH_6x33[3][25] = temp4_3x1[3];
	_jRH_6x33[4][25] = axis_rwp[1];
	_jRH_6x33[5][25] = axis_rwp[2];
	_jRH_6x33[6][25] = axis_rwp[3];
	cross(1.f,axis_rwy2, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][32] = temp4_3x1[1];
	_jRH_6x33[2][32] = temp4_3x1[2];
	_jRH_6x33[3][32] = temp4_3x1[3];
	_jRH_6x33[4][32] = axis_rwy2[1];
	_jRH_6x33[5][32] = axis_rwy2[2];
	_jRH_6x33[6][32] = axis_rwy2[3];
	
	_jLH_6x33[1][1] = 1.f;
	_jLH_6x33[2][2] = 1.f;
	_jLH_6x33[3][3] = 1.f;
	_jLH_6x33[4][4] = 1.f;
	_jLH_6x33[5][5] = 1.f;
	_jLH_6x33[6][6] = 1.f;

	diff_vv(_pLH_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][4] = temp4_3x1[1];
	_jLH_6x33[2][4] = temp4_3x1[2];
	_jLH_6x33[3][4] = temp4_3x1[3];
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][5] = temp4_3x1[1];
	_jLH_6x33[2][5] = temp4_3x1[2];
	_jLH_6x33[3][5] = temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][6] = temp4_3x1[1];
	_jLH_6x33[2][6] = temp4_3x1[2];
	_jLH_6x33[3][6] = temp4_3x1[3];

	diff_vv(_pLH_3x1,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][7] = temp4_3x1[1];
	_jLH_6x33[2][7] = temp4_3x1[2];
	_jLH_6x33[3][7] = temp4_3x1[3];
	_jLH_6x33[4][7] = axis_wst[1];
	_jLH_6x33[5][7] = axis_wst[2];
	_jLH_6x33[6][7] = axis_wst[3];

	diff_vv(_pLH_3x1,3,pLSHLD, temp3_3x1);
	cross(1.f,axis_lsp, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][26] = temp4_3x1[1];
	_jLH_6x33[2][26] = temp4_3x1[2];
	_jLH_6x33[3][26] = temp4_3x1[3];
	_jLH_6x33[4][26] = axis_lsp[1];
	_jLH_6x33[5][26] = axis_lsp[2];
	_jLH_6x33[6][26] = axis_lsp[3];
	cross(1.f,axis_lsr, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][27] = temp4_3x1[1];
	_jLH_6x33[2][27] = temp4_3x1[2];
	_jLH_6x33[3][27] = temp4_3x1[3];
	_jLH_6x33[4][27] = axis_lsr[1];
	_jLH_6x33[5][27] = axis_lsr[2];
	_jLH_6x33[6][27] = axis_lsr[3];
	cross(1.f,axis_lsy, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][28] = temp4_3x1[1];
	_jLH_6x33[2][28] = temp4_3x1[2];
	_jLH_6x33[3][28] = temp4_3x1[3];
	_jLH_6x33[4][28] = axis_lsy[1];
	_jLH_6x33[5][28] = axis_lsy[2];
	_jLH_6x33[6][28] = axis_lsy[3];

	diff_vv(_pLH_3x1,3,pLELB, temp3_3x1);
	cross(1.f,axis_leb, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][29] = temp4_3x1[1];
	_jLH_6x33[2][29] = temp4_3x1[2];
	_jLH_6x33[3][29] = temp4_3x1[3];
	_jLH_6x33[4][29] = axis_leb[1];
	_jLH_6x33[5][29] = axis_leb[2];
	_jLH_6x33[6][29] = axis_leb[3];

	diff_vv(_pLH_3x1,3, pLELB2, temp3_3x1);
	cross(1.f,axis_lwy, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][30] = temp4_3x1[1];
	_jLH_6x33[2][30] = temp4_3x1[2];
	_jLH_6x33[3][30] = temp4_3x1[3];
	_jLH_6x33[4][30] = axis_lwy[1];
	_jLH_6x33[5][30] = axis_lwy[2];
	_jLH_6x33[6][30] = axis_lwy[3];

	diff_vv(_pLH_3x1,3,_pLWR_3x1, temp3_3x1);
	cross(1.f,axis_lwp, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][31] = temp4_3x1[1];
	_jLH_6x33[2][31] = temp4_3x1[2];
	_jLH_6x33[3][31] = temp4_3x1[3];
	_jLH_6x33[4][31] = axis_lwp[1];
	_jLH_6x33[5][31] = axis_lwp[2];
	_jLH_6x33[6][31] = axis_lwp[3];
	cross(1.f,axis_lwy2, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][33] = temp4_3x1[1];
	_jLH_6x33[2][33] = temp4_3x1[2];
	_jLH_6x33[3][33] = temp4_3x1[3];
	_jLH_6x33[4][33] = axis_lwy2[1];
	_jLH_6x33[5][33] = axis_lwy2[2];
	_jLH_6x33[6][33] = axis_lwy2[3];

	_jRS_6x33[1][1] = 1.f;
	_jRS_6x33[2][2] = 1.f;
	_jRS_6x33[3][3] = 1.f;
	_jRS_6x33[4][4] = 1.f;
	_jRS_6x33[5][5] = 1.f;
	_jRS_6x33[6][6] = 1.f;

	diff_vv(_pRS_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][4] = temp4_3x1[1];
	_jRS_6x33[2][4] = temp4_3x1[2];
	_jRS_6x33[3][4] = temp4_3x1[3];
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][5] = temp4_3x1[1];
	_jRS_6x33[2][5] = temp4_3x1[2];
	_jRS_6x33[3][5] = temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][6] = temp4_3x1[1];
	_jRS_6x33[2][6] = temp4_3x1[2];
	_jRS_6x33[3][6] = temp4_3x1[3];

	diff_vv(_pRS_3x1,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][7] = temp4_3x1[1];
	_jRS_6x33[2][7] = temp4_3x1[2];
	_jRS_6x33[3][7] = temp4_3x1[3];
	_jRS_6x33[4][7] = axis_wst[1];
	_jRS_6x33[5][7] = axis_wst[2];
	_jRS_6x33[6][7] = axis_wst[3];

	diff_vv(_pRS_3x1,3,pRSHLD, temp3_3x1);
	cross(1.f,axis_rsp, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][20] = temp4_3x1[1];
	_jRS_6x33[2][20] = temp4_3x1[2];
	_jRS_6x33[3][20] = temp4_3x1[3];
	_jRS_6x33[4][20] = axis_rsp[1];
	_jRS_6x33[5][20] = axis_rsp[2];
	_jRS_6x33[6][20] = axis_rsp[3];
	cross(1.f,axis_rsr, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][21] = temp4_3x1[1];
	_jRS_6x33[2][21] = temp4_3x1[2];
	_jRS_6x33[3][21] = temp4_3x1[3];
	_jRS_6x33[4][21] = axis_rsr[1];
	_jRS_6x33[5][21] = axis_rsr[2];
	_jRS_6x33[6][21] = axis_rsr[3];
	cross(1.f,axis_rsy, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][22] = temp4_3x1[1];
	_jRS_6x33[2][22] = temp4_3x1[2];
	_jRS_6x33[3][22] = temp4_3x1[3];
	_jRS_6x33[4][22] = axis_rsy[1];
	_jRS_6x33[5][22] = axis_rsy[2];
	_jRS_6x33[6][22] = axis_rsy[3];

	diff_vv(_pRS_3x1,3,pRELB, temp3_3x1);
	cross(1.f,axis_reb, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][23] = temp4_3x1[1];
	_jRS_6x33[2][23] = temp4_3x1[2];
	_jRS_6x33[3][23] = temp4_3x1[3];
	_jRS_6x33[4][23] = axis_reb[1];
	_jRS_6x33[5][23] = axis_reb[2];
	_jRS_6x33[6][23] = axis_reb[3];
	
	diff_vv(_pRS_3x1,3, pRELB2, temp3_3x1);
	cross(1.f,axis_rwy, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][24] = temp4_3x1[1];
	_jRS_6x33[2][24] = temp4_3x1[2];
	_jRS_6x33[3][24] = temp4_3x1[3];
	_jRS_6x33[4][24] = axis_rwy[1];
	_jRS_6x33[5][24] = axis_rwy[2];
	_jRS_6x33[6][24] = axis_rwy[3];

	diff_vv(_pRS_3x1,3,_pRWR_3x1, temp3_3x1);
	cross(1.f,axis_rwp, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][25] = temp4_3x1[1];
	_jRS_6x33[2][25] = temp4_3x1[2];
	_jRS_6x33[3][25] = temp4_3x1[3];
	_jRS_6x33[4][25] = axis_rwp[1];
	_jRS_6x33[5][25] = axis_rwp[2];
	_jRS_6x33[6][25] = axis_rwp[3];
	
	_jLS_6x33[1][1] = 1.f;
	_jLS_6x33[2][2] = 1.f;
	_jLS_6x33[3][3] = 1.f;
	_jLS_6x33[4][4] = 1.f;
	_jLS_6x33[5][5] = 1.f;
	_jLS_6x33[6][6] = 1.f;

	diff_vv(_pLS_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][4] = temp4_3x1[1];
	_jLS_6x33[2][4] = temp4_3x1[2];
	_jLS_6x33[3][4] = temp4_3x1[3];
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][5] = temp4_3x1[1];
	_jLS_6x33[2][5] = temp4_3x1[2];
	_jLS_6x33[3][5] = temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][6] = temp4_3x1[1];
	_jLS_6x33[2][6] = temp4_3x1[2];
	_jLS_6x33[3][6] = temp4_3x1[3];

	diff_vv(_pLS_3x1,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][7] = temp4_3x1[1];
	_jLS_6x33[2][7] = temp4_3x1[2];
	_jLS_6x33[3][7] = temp4_3x1[3];
	_jLS_6x33[4][7] = axis_wst[1];
	_jLS_6x33[5][7] = axis_wst[2];
	_jLS_6x33[6][7] = axis_wst[3];

	diff_vv(_pLS_3x1,3,pLSHLD, temp3_3x1);
	cross(1.f,axis_lsp, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][26] = temp4_3x1[1];
	_jLS_6x33[2][26] = temp4_3x1[2];
	_jLS_6x33[3][26] = temp4_3x1[3];
	_jLS_6x33[4][26] = axis_lsp[1];
	_jLS_6x33[5][26] = axis_lsp[2];
	_jLS_6x33[6][26] = axis_lsp[3];
	cross(1.f,axis_lsr, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][27] = temp4_3x1[1];
	_jLS_6x33[2][27] = temp4_3x1[2];
	_jLS_6x33[3][27] = temp4_3x1[3];
	_jLS_6x33[4][27] = axis_lsr[1];
	_jLS_6x33[5][27] = axis_lsr[2];
	_jLS_6x33[6][27] = axis_lsr[3];
	cross(1.f,axis_lsy, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][28] = temp4_3x1[1];
	_jLS_6x33[2][28] = temp4_3x1[2];
	_jLS_6x33[3][28] = temp4_3x1[3];
	_jLS_6x33[4][28] = axis_lsy[1];
	_jLS_6x33[5][28] = axis_lsy[2];
	_jLS_6x33[6][28] = axis_lsy[3];

	diff_vv(_pLS_3x1,3,pLELB, temp3_3x1);
	cross(1.f,axis_leb, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][29] = temp4_3x1[1];
	_jLS_6x33[2][29] = temp4_3x1[2];
	_jLS_6x33[3][29] = temp4_3x1[3];
	_jLS_6x33[4][29] = axis_leb[1];
	_jLS_6x33[5][29] = axis_leb[2];
	_jLS_6x33[6][29] = axis_leb[3];

	diff_vv(_pLS_3x1,3, pLELB2, temp3_3x1);
	cross(1.f,axis_lwy, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][30] = temp4_3x1[1];
	_jLS_6x33[2][30] = temp4_3x1[2];
	_jLS_6x33[3][30] = temp4_3x1[3];
	_jLS_6x33[4][30] = axis_lwy[1];
	_jLS_6x33[5][30] = axis_lwy[2];
	_jLS_6x33[6][30] = axis_lwy[3];

	diff_vv(_pLS_3x1,3,_pLWR_3x1, temp3_3x1);
	cross(1.f,axis_lwp, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][31] = temp4_3x1[1];
	_jLS_6x33[2][31] = temp4_3x1[2];
	_jLS_6x33[3][31] = temp4_3x1[3];
	_jLS_6x33[4][31] = axis_lwp[1];
	_jLS_6x33[5][31] = axis_lwp[2];
	_jLS_6x33[6][31] = axis_lwp[3];


	_jRWR_6x33[1][1] = 1.f;
	_jRWR_6x33[2][2] = 1.f;
	_jRWR_6x33[3][3] = 1.f;
	_jRWR_6x33[4][4] = 1.f;
	_jRWR_6x33[5][5] = 1.f;
	_jRWR_6x33[6][6] = 1.f;

	diff_vv(_pRWR_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][4] = temp4_3x1[1];
	_jRWR_6x33[2][4] = temp4_3x1[2];
	_jRWR_6x33[3][4] = temp4_3x1[3];
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][5] = temp4_3x1[1];
	_jRWR_6x33[2][5] = temp4_3x1[2];
	_jRWR_6x33[3][5] = temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][6] = temp4_3x1[1];
	_jRWR_6x33[2][6] = temp4_3x1[2];
	_jRWR_6x33[3][6] = temp4_3x1[3];

	diff_vv(_pRWR_3x1,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][7] = temp4_3x1[1];
	_jRWR_6x33[2][7] = temp4_3x1[2];
	_jRWR_6x33[3][7] = temp4_3x1[3];
	_jRWR_6x33[4][7] = axis_wst[1];
	_jRWR_6x33[5][7] = axis_wst[2];
	_jRWR_6x33[6][7] = axis_wst[3];

	diff_vv(_pRWR_3x1,3,pRSHLD, temp3_3x1);
	cross(1.f,axis_rsp, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][20] = temp4_3x1[1];
	_jRWR_6x33[2][20] = temp4_3x1[2];
	_jRWR_6x33[3][20] = temp4_3x1[3];
	_jRWR_6x33[4][20] = axis_rsp[1];
	_jRWR_6x33[5][20] = axis_rsp[2];
	_jRWR_6x33[6][20] = axis_rsp[3];
	cross(1.f,axis_rsr, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][21] = temp4_3x1[1];
	_jRWR_6x33[2][21] = temp4_3x1[2];
	_jRWR_6x33[3][21] = temp4_3x1[3];
	_jRWR_6x33[4][21] = axis_rsr[1];
	_jRWR_6x33[5][21] = axis_rsr[2];
	_jRWR_6x33[6][21] = axis_rsr[3];
	cross(1.f,axis_rsy, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][22] = temp4_3x1[1];
	_jRWR_6x33[2][22] = temp4_3x1[2];
	_jRWR_6x33[3][22] = temp4_3x1[3];
	_jRWR_6x33[4][22] = axis_rsy[1];
	_jRWR_6x33[5][22] = axis_rsy[2];
	_jRWR_6x33[6][22] = axis_rsy[3];

	diff_vv(_pRWR_3x1,3,pRELB, temp3_3x1);
	cross(1.f,axis_reb, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][23] = temp4_3x1[1];
	_jRWR_6x33[2][23] = temp4_3x1[2];
	_jRWR_6x33[3][23] = temp4_3x1[3];
	_jRWR_6x33[4][23] = axis_reb[1];
	_jRWR_6x33[5][23] = axis_reb[2];
	_jRWR_6x33[6][23] = axis_reb[3];
	
	diff_vv(_pRWR_3x1,3, pRELB2, temp3_3x1);
	cross(1.f,axis_rwy, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][24] = temp4_3x1[1];
	_jRWR_6x33[2][24] = temp4_3x1[2];
	_jRWR_6x33[3][24] = temp4_3x1[3];
	_jRWR_6x33[4][24] = axis_rwy[1];
	_jRWR_6x33[5][24] = axis_rwy[2];
	_jRWR_6x33[6][24] = axis_rwy[3];


	_jLWR_6x33[1][1] = 1.f;
	_jLWR_6x33[2][2] = 1.f;
	_jLWR_6x33[3][3] = 1.f;
	_jLWR_6x33[4][4] = 1.f;
	_jLWR_6x33[5][5] = 1.f;
	_jLWR_6x33[6][6] = 1.f;

	diff_vv(_pLWR_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][4] = temp4_3x1[1];
	_jLWR_6x33[2][4] = temp4_3x1[2];
	_jLWR_6x33[3][4] = temp4_3x1[3];
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][5] = temp4_3x1[1];
	_jLWR_6x33[2][5] = temp4_3x1[2];
	_jLWR_6x33[3][5] = temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][6] = temp4_3x1[1];
	_jLWR_6x33[2][6] = temp4_3x1[2];
	_jLWR_6x33[3][6] = temp4_3x1[3];

	diff_vv(_pLWR_3x1,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][7] = temp4_3x1[1];
	_jLWR_6x33[2][7] = temp4_3x1[2];
	_jLWR_6x33[3][7] = temp4_3x1[3];
	_jLWR_6x33[4][7] = axis_wst[1];
	_jLWR_6x33[5][7] = axis_wst[2];
	_jLWR_6x33[6][7] = axis_wst[3];

	diff_vv(_pLWR_3x1,3,pLSHLD, temp3_3x1);
	cross(1.f,axis_lsp, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][26] = temp4_3x1[1];
	_jLWR_6x33[2][26] = temp4_3x1[2];
	_jLWR_6x33[3][26] = temp4_3x1[3];
	_jLWR_6x33[4][26] = axis_lsp[1];
	_jLWR_6x33[5][26] = axis_lsp[2];
	_jLWR_6x33[6][26] = axis_lsp[3];
	cross(1.f,axis_lsr, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][27] = temp4_3x1[1];
	_jLWR_6x33[2][27] = temp4_3x1[2];
	_jLWR_6x33[3][27] = temp4_3x1[3];
	_jLWR_6x33[4][27] = axis_lsr[1];
	_jLWR_6x33[5][27] = axis_lsr[2];
	_jLWR_6x33[6][27] = axis_lsr[3];
	cross(1.f,axis_lsy, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][28] = temp4_3x1[1];
	_jLWR_6x33[2][28] = temp4_3x1[2];
	_jLWR_6x33[3][28] = temp4_3x1[3];
	_jLWR_6x33[4][28] = axis_lsy[1];
	_jLWR_6x33[5][28] = axis_lsy[2];
	_jLWR_6x33[6][28] = axis_lsy[3];

	diff_vv(_pLWR_3x1,3,pLELB, temp3_3x1);
	cross(1.f,axis_leb, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][29] = temp4_3x1[1];
	_jLWR_6x33[2][29] = temp4_3x1[2];
	_jLWR_6x33[3][29] = temp4_3x1[3];
	_jLWR_6x33[4][29] = axis_leb[1];
	_jLWR_6x33[5][29] = axis_leb[2];
	_jLWR_6x33[6][29] = axis_leb[3];

	diff_vv(_pLWR_3x1,3, pLELB2, temp3_3x1);
	cross(1.f,axis_lwy, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][30] = temp4_3x1[1];
	_jLWR_6x33[2][30] = temp4_3x1[2];
	_jLWR_6x33[3][30] = temp4_3x1[3];
	_jLWR_6x33[4][30] = axis_lwy[1];
	_jLWR_6x33[5][30] = axis_lwy[2];
	_jLWR_6x33[6][30] = axis_lwy[3];
	
	DC2QT((const float**)_dcRF_3x3, _qRF_4x1);
	DC2QT((const float**)_dcLF_3x3, _qLF_4x1);

	DC2QT((const float**)_dcRH_3x3, _qRH_4x1);
	DC2QT((const float**)_dcLH_3x3, _qLH_4x1);

	DC2QT((const float**)_dcRS_3x3, _qRS_4x1);
	DC2QT((const float**)_dcLS_3x3, _qLS_4x1);

	DC2QT((const float**)_dcRLA_3x3, _qRWR_4x1);
	DC2QT((const float**)_dcLLA_3x3, _qLWR_4x1);
	
	//-------------------------------------- local coordinates
	diff_vv(_pRH_3x1,3, pPC, _pRH_L_3x1);
	diff_vv(_pLH_3x1,3, pPC, _pLH_L_3x1);
	diff_vv(_pRWR_3x1,3, pPC, _pRWR_L_3x1);
	diff_vv(_pLWR_3x1,3, pPC, _pLWR_L_3x1);
	diff_vv(_pRF_3x1,3, pPC, _pRF_L_3x1);
	diff_vv(_pLF_3x1,3, pPC, _pLF_L_3x1);
	diff_vv(_pRANK_3x1,3, pPC, _pRANK_L_3x1);
	diff_vv(_pLANK_3x1,3, pPC, _pLANK_L_3x1);
	diff_vv(_pRS_3x1,3, pPC, _pRS_L_3x1);
	diff_vv(_pLS_3x1,3, pPC, _pLS_L_3x1);

	trans(1.f, (const float**)_dcPEL_3x3, 3,3,_TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcRF_3x3,3, _TEMP2_34x34);
	DC2QT((const float**)_TEMP2_34x34, _qRF_L_4x1);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcLF_3x3,3, _TEMP2_34x34);
	DC2QT((const float**)_TEMP2_34x34, _qLF_L_4x1);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcRH_3x3,3, _TEMP2_34x34);
	DC2QT((const float**)_TEMP2_34x34, _qRH_L_4x1);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcLH_3x3,3, _TEMP2_34x34);
	DC2QT((const float**)_TEMP2_34x34, _qLH_L_4x1);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcRS_3x3,3, _TEMP2_34x34);
	DC2QT((const float**)_TEMP2_34x34, _qRS_L_4x1);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcLS_3x3,3, _TEMP2_34x34);
	DC2QT((const float**)_TEMP2_34x34, _qLS_L_4x1);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcRLA_3x3,3, _TEMP2_34x34);
	DC2QT((const float**)_TEMP2_34x34, _qRWR_L_4x1);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcLLA_3x3,3, _TEMP2_34x34);
	DC2QT((const float**)_TEMP2_34x34, _qLWR_L_4x1);

	free_matrix(jCOM_PEL_3x33, 1,3,1,33);
	free_matrix(jCOM_RUL_3x33, 1,3,1,33);
	free_matrix(jCOM_RLL_3x33, 1,3,1,33);
	free_matrix(jCOM_RF_3x33, 1,3,1,33);
	free_matrix(jCOM_LUL_3x33, 1,3,1,33);
	free_matrix(jCOM_LLL_3x33, 1,3,1,33);
	free_matrix(jCOM_LF_3x33, 1,3,1,33);
	free_matrix(jCOM_TOR_3x33, 1,3,1,33);
	free_matrix(jCOM_RUA_3x33, 1,3,1,33);
	free_matrix(jCOM_RLA_3x33, 1,3,1,33);
	free_matrix(jCOM_LUA_3x33, 1,3,1,33);
	free_matrix(jCOM_LLA_3x33, 1,3,1,33);
	free_matrix(jCOM_LH_3x33, 1,3,1,33);
	free_matrix(jCOM_RH_3x33, 1,3,1,33);

	return 1;
}


int FKine_Hand(char ref_coord, const float *Q_34x1, float *pRH_3x1, float *qRH_4x1, float *pLH_3x1, float *qLH_4x1)
{
	float wst, rsp, rsr, rsy, reb, rwy, lsp, lsr, lsy, leb, lwy, rwp, lwp, rwy2, lwy2;
	float pWST[4], pRSHLD[4], pRELB[4], pLSHLD[4], pLELB[4], pRELB2[4], pLELB2[4], pRWR[4], pLWR[4];
	float temp3_3x1[4], temp4_3x1[4];
	float qPEL[5] = {0., Q_34x1[4], Q_34x1[5], Q_34x1[6], Q_34x1[7]};
	float pPC[4] = {0., Q_34x1[1], Q_34x1[2], Q_34x1[3]};

	wst = Q_34x1[8];
	rsp = Q_34x1[21];
	rsr = Q_34x1[22];
	rsy = Q_34x1[23];
	reb = Q_34x1[24];
	rwy = Q_34x1[25];
	rwp = Q_34x1[26];
	lsp = Q_34x1[27];
	lsr = Q_34x1[28];
	lsy = Q_34x1[29];
	leb = Q_34x1[30];
	lwy = Q_34x1[31];
	lwp = Q_34x1[32];
	rwy2 = Q_34x1[33];
	lwy2 = Q_34x1[34];

	QT2DC(qPEL, _dcPEL_3x3);
	
	RZ(wst, _Rz_WST_3x3);	
	RY(rsp, _Ry_RSP_3x3);
	RX(rsr, _Rx_RSR_3x3);
	RZ(rsy, _Rz_RSY_3x3);
	RY(reb, _Ry_REB_3x3);
	RZ(rwy, _Rz_RWY_3x3);
	RY(rwp, _Ry_RWP_3x3);	
	RY(lsp, _Ry_LSP_3x3);
	RX(lsr, _Rx_LSR_3x3);
	RZ(lsy, _Rz_LSY_3x3);
	RY(leb, _Ry_LEB_3x3);
	RZ(lwy, _Rz_LWY_3x3);
	RY(lwp, _Ry_LWP_3x3);	
	RZ(rwy2, _Rz_RWY2_3x3);
	RZ(lwy2, _Rz_LWY2_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_WST_3x3,3, _dcTOR_3x3);
	
	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_RSP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_RSR_3x3,3,3, (const float**)_Rz_RSY_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcRUA_3x3);

	mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REB_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_RWY_3x3,3, _dcRLA_3x3);
	mult_mm((const float**)_dcRLA_3x3,3,3, (const float**)_Ry_RWP_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP2_34x34,3,3, (const float**)_Rz_RWY2_3x3,3, _dcRH_3x3);

	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_LSP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_LSR_3x3,3,3, (const float**)_Rz_LSY_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcLUA_3x3);

	mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEB_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_LWY_3x3,3, _dcLLA_3x3);
	mult_mm((const float**)_dcLLA_3x3,3,3, (const float**)_Ry_LWP_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP2_34x34,3,3, (const float**)_Rz_LWY2_3x3,3, _dcLH_3x3);

	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_WST, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pWST);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_RSHLD, temp3_3x1); 
	sum_vv(pWST,3, temp3_3x1, pRSHLD);

	mult_mv((const float**)_dcRUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pRSHLD,3, temp3_3x1, pRELB);

	temp4_3x1[1] = _LINK_LARM[1];
	temp4_3x1[2] = 0.;
	temp4_3x1[3] = 0.;
	mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REB_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pRELB,3, temp3_3x1, pRELB2);
	temp4_3x1[1] = 0.;
	temp4_3x1[2] = _LINK_LARM[2];
	temp4_3x1[3] = _LINK_LARM[3];
	mult_mv((const float**)_dcRLA_3x3,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pRELB2,3, temp3_3x1, pRWR);

	mult_mv((const float**)_dcRH_3x3,3,3, _LINK_HAND, temp3_3x1); 
	sum_vv(pRWR,3, temp3_3x1, pRH_3x1);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_LSHLD, temp3_3x1); 
	sum_vv(pWST,3, temp3_3x1, pLSHLD);

	mult_mv((const float**)_dcLUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pLSHLD,3, temp3_3x1, pLELB);

	temp4_3x1[1] = _LINK_LARM[1];
	temp4_3x1[2] = 0.;
	temp4_3x1[3] = 0.;
	mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEB_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pLELB,3, temp3_3x1, pLELB2);
	temp4_3x1[1] = 0.;
	temp4_3x1[2] = _LINK_LARM[2];
	temp4_3x1[3] = _LINK_LARM[3];
	mult_mv((const float**)_dcLLA_3x3,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pLELB2,3, temp3_3x1, pLWR);

	mult_mv((const float**)_dcLH_3x3,3,3, _LINK_HAND, temp3_3x1); 
	sum_vv(pLWR,3, temp3_3x1, pLH_3x1);

	DC2QT((const float**)_dcRH_3x3, qRH_4x1);
	DC2QT((const float**)_dcLH_3x3, qLH_4x1);

	//-------------------------------------- local coordinates
	if(ref_coord == LOCAL)
	{
		diff_vv(pRH_3x1,3, pPC, pRH_3x1);
		diff_vv(pLH_3x1,3, pPC, pLH_3x1);
	
		trans(1.f, (const float**)_dcPEL_3x3, 3,3,_TEMP1_34x34);
		mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcRH_3x3,3, _TEMP2_34x34);
		DC2QT((const float**)_TEMP2_34x34, qRH_4x1);
		mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcLH_3x3,3, _TEMP2_34x34);
		DC2QT((const float**)_TEMP2_34x34, qLH_4x1);
	}
	

	return 0;
}


int FKine_Foot(char ref_coord, const float *Q_34x1, float *pRF_3x1, float *qRF_4x1, float *pLF_3x1, float *qLF_4x1)
{
	float rhy, rhp, rhr, rkn, rap, rar, lhy, lhp, lhr, lkn, lap, lar;
	float pRPEL[4], pRKN[4], pRANK[4], pLPEL[4], pLKN[4], pLANK[4];
	float temp3_3x1[4];
	float qPEL[5] = {0., Q_34x1[4], Q_34x1[5], Q_34x1[6], Q_34x1[7]};
	float pPC[4] = {0., Q_34x1[1], Q_34x1[2], Q_34x1[3]};

	if(ref_coord == LOCAL)
	{
		qPEL[1] = 1.f;
		qPEL[2] = qPEL[3] = qPEL[4] = 0.f;
		pPC[1] = pPC[2] = pPC[3] = 0.f;
	}

	rhy = Q_34x1[RHY_34];
	rhr = Q_34x1[RHR_34];
	rhp = Q_34x1[RHP_34];
	rkn = Q_34x1[RKN_34];
	rap = Q_34x1[RAP_34];
	rar = Q_34x1[RAR_34];

	lhy = Q_34x1[LHY_34];
	lhr = Q_34x1[LHR_34];
	lhp = Q_34x1[LHP_34];
	lkn = Q_34x1[LKN_34];
	lap = Q_34x1[LAP_34];
	lar = Q_34x1[LAR_34];

	QT2DC(qPEL, _dcPEL_3x3);
	
	RZ(rhy, _Rz_RHY_3x3);
	RX(rhr, _Rx_RHR_3x3);
	RY(rhp, _Ry_RHP_3x3);
	RY(rkn, _Ry_RKN_3x3);
	RY(rap, _Ry_RAP_3x3);
	RX(rar, _Rx_RAR_3x3);

	RZ(lhy, _Rz_LHY_3x3);
	RX(lhr, _Rx_LHR_3x3);
	RY(lhp, _Ry_LHP_3x3);
	RY(lkn, _Ry_LKN_3x3);
	RY(lap, _Ry_LAP_3x3);
	RX(lar, _Rx_LAR_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_RHR_3x3,3,3, (const float**)_Ry_RHP_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcRUL_3x3);

	mult_mm((const float**)_dcRUL_3x3,3,3, (const float**)_Ry_RKN_3x3,3, _dcRLL_3x3);
	
	mult_mm((const float**)_Ry_RAP_3x3,3,3, (const float**)_Rx_RAR_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_dcRLL_3x3,3,3, (const float**)_TEMP1_34x34,3, _dcRF_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_LHR_3x3,3,3, (const float**)_Ry_LHP_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcLUL_3x3);

	mult_mm((const float**)_dcLUL_3x3,3,3, (const float**)_Ry_LKN_3x3,3, _dcLLL_3x3);
	
	mult_mm((const float**)_Ry_LAP_3x3,3,3, (const float**)_Rx_LAR_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_dcLLL_3x3,3,3, (const float**)_TEMP1_34x34,3, _dcLF_3x3);
	
	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_RPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pRPEL);

	mult_mv((const float**)_dcRUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRPEL,3, temp3_3x1, pRKN);

	mult_mv((const float**)_dcRLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, pRANK);

	mult_mv((const float**)_dcRF_3x3,3,3, _LINK_FOOT, temp3_3x1);
	sum_vv(pRANK,3, temp3_3x1, pRF_3x1);


	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_LPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pLPEL);

	mult_mv((const float**)_dcLUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLPEL,3, temp3_3x1, pLKN);

	mult_mv((const float**)_dcLLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, pLANK);

	mult_mv((const float**)_dcLF_3x3,3,3, _LINK_FOOT, temp3_3x1);
	sum_vv(pLANK,3, temp3_3x1, pLF_3x1);

	DC2QT((const float**)_dcRF_3x3, qRF_4x1);
	DC2QT((const float**)_dcLF_3x3, qLF_4x1);
	
	return 0;
}


int FKine_Ankle(char ref_coord, const float *Q_34x1, float *pRANK_3x1, float *qRANK_4x1, float *pLANK_3x1, float *qLANK_4x1)
{
	float rhy, rhp, rhr, rkn, lhy, lhp, lhr, lkn;
	float pRPEL[4], pRKN[4], pLPEL[4], pLKN[4];
	float temp3_3x1[4];
	float qPEL[5] = {0., Q_34x1[4], Q_34x1[5], Q_34x1[6], Q_34x1[7]};
	float pPC[4] = {0., Q_34x1[1], Q_34x1[2], Q_34x1[3]};

	if(ref_coord == LOCAL)
	{
		qPEL[1] = 1.f;
		qPEL[2] = qPEL[3] = qPEL[4] = 0.f;
		pPC[1] = pPC[2] = pPC[3] = 0.f;
	}

	rhy = Q_34x1[RHY_34];
	rhr = Q_34x1[RHR_34];
	rhp = Q_34x1[RHP_34];
	rkn = Q_34x1[RKN_34];

	lhy = Q_34x1[LHY_34];
	lhr = Q_34x1[LHR_34];
	lhp = Q_34x1[LHP_34];
	lkn = Q_34x1[LKN_34];

	QT2DC(qPEL, _dcPEL_3x3);
	
	RZ(rhy, _Rz_RHY_3x3);
	RX(rhr, _Rx_RHR_3x3);
	RY(rhp, _Ry_RHP_3x3);
	RY(rkn, _Ry_RKN_3x3);

	RZ(lhy, _Rz_LHY_3x3);
	RX(lhr, _Rx_LHR_3x3);
	RY(lhp, _Ry_LHP_3x3);
	RY(lkn, _Ry_LKN_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_RHR_3x3,3,3, (const float**)_Ry_RHP_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcRUL_3x3);

	mult_mm((const float**)_dcRUL_3x3,3,3, (const float**)_Ry_RKN_3x3,3, _dcRLL_3x3);
	
	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_LHR_3x3,3,3, (const float**)_Ry_LHP_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcLUL_3x3);

	mult_mm((const float**)_dcLUL_3x3,3,3, (const float**)_Ry_LKN_3x3,3, _dcLLL_3x3);
	
	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_RPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pRPEL);

	mult_mv((const float**)_dcRUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRPEL,3, temp3_3x1, pRKN);

	mult_mv((const float**)_dcRLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, pRANK_3x1);


	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_LPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pLPEL);

	mult_mv((const float**)_dcLUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLPEL,3, temp3_3x1, pLKN);

	mult_mv((const float**)_dcLLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, pLANK_3x1);

	DC2QT((const float**)_dcRLL_3x3, qRANK_4x1);
	DC2QT((const float**)_dcLLL_3x3, qLANK_4x1);
	
	return 0;
}


int FKine_Wrist(char ref_coord, const float *Q_34x1, float *pRWR_3x1, float *qRWR_4x1, float *pLWR_3x1, float *qLWR_4x1)
{
	float wst, rsp, rsr, rsy, reb, rwy, lsp, lsr, lsy, leb, lwy;
	float pWST[4], pRSHLD[4], pRELB[4], pLSHLD[4], pLELB[4], pRELB2[4], pLELB2[4];
	float temp3_3x1[4], temp4_3x1[4];
	float qPEL[5] = {0., Q_34x1[4], Q_34x1[5], Q_34x1[6], Q_34x1[7]};
	float pPC[4] = {0., Q_34x1[1], Q_34x1[2], Q_34x1[3]};

	wst = Q_34x1[8];
	rsp = Q_34x1[21];
	rsr = Q_34x1[22];
	rsy = Q_34x1[23];
	reb = Q_34x1[24];
	rwy = Q_34x1[25];

	lsp = Q_34x1[27];
	lsr = Q_34x1[28];
	lsy = Q_34x1[29];
	leb = Q_34x1[30];
	lwy = Q_34x1[31];


	QT2DC(qPEL, _dcPEL_3x3);
	
	RZ(wst, _Rz_WST_3x3);	
	RY(rsp, _Ry_RSP_3x3);
	RX(rsr, _Rx_RSR_3x3);
	RZ(rsy, _Rz_RSY_3x3);
	RY(reb, _Ry_REB_3x3);
	RZ(rwy, _Rz_RWY_3x3);
	
	RY(lsp, _Ry_LSP_3x3);
	RX(lsr, _Rx_LSR_3x3);
	RZ(lsy, _Rz_LSY_3x3);
	RY(leb, _Ry_LEB_3x3);
	RZ(lwy, _Rz_LWY_3x3);
	
	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_WST_3x3,3, _dcTOR_3x3);
	
	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_RSP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_RSR_3x3,3,3, (const float**)_Rz_RSY_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcRUA_3x3);

	mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REB_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_RWY_3x3,3, _dcRLA_3x3);
	
	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_LSP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_LSR_3x3,3,3, (const float**)_Rz_LSY_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcLUA_3x3);

	mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEB_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_LWY_3x3,3, _dcLLA_3x3);
	
	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_WST, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pWST);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_RSHLD, temp3_3x1); 
	sum_vv(pWST,3, temp3_3x1, pRSHLD);

	mult_mv((const float**)_dcRUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pRSHLD,3, temp3_3x1, pRELB);

	temp4_3x1[1] = _LINK_LARM[1];
	temp4_3x1[2] = 0.;
	temp4_3x1[3] = 0.;
	mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REB_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pRELB,3, temp3_3x1, pRELB2);
	temp4_3x1[1] = 0.;
	temp4_3x1[2] = _LINK_LARM[2];
	temp4_3x1[3] = _LINK_LARM[3];
	mult_mv((const float**)_dcRLA_3x3,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pRELB2,3, temp3_3x1, pRWR_3x1);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_LSHLD, temp3_3x1); 
	sum_vv(pWST,3, temp3_3x1, pLSHLD);

	mult_mv((const float**)_dcLUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pLSHLD,3, temp3_3x1, pLELB);

	temp4_3x1[1] = _LINK_LARM[1];
	temp4_3x1[2] = 0.;
	temp4_3x1[3] = 0.;
	mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEB_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pLELB,3, temp3_3x1, pLELB2);
	temp4_3x1[1] = 0.;
	temp4_3x1[2] = _LINK_LARM[2];
	temp4_3x1[3] = _LINK_LARM[3];
	mult_mv((const float**)_dcLLA_3x3,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pLELB2,3, temp3_3x1, pLWR_3x1);

	DC2QT((const float**)_dcRLA_3x3, qRWR_4x1);
	DC2QT((const float**)_dcLLA_3x3, qLWR_4x1);

	//-------------------------------------- local coordinates
	if(ref_coord == LOCAL)
	{
		diff_vv(pRWR_3x1,3, pPC, pRWR_3x1);
		diff_vv(pLWR_3x1,3, pPC, pLWR_3x1);
	
		trans(1.f, (const float**)_dcPEL_3x3, 3,3,_TEMP1_34x34);
		mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcRLA_3x3,3, _TEMP2_34x34);
		DC2QT((const float**)_TEMP2_34x34, qRWR_4x1);
		mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcLLA_3x3,3, _TEMP2_34x34);
		DC2QT((const float**)_TEMP2_34x34, qLWR_4x1);
	}
	
	return 0;
}



int FKine_Stick(char ref_coord, const float *Q_34x1, float *pRS_3x1, float *qRS_4x1, float *pLS_3x1, float *qLS_4x1)
{
	float wst, rsp, rsr, rsy, reb, rwy, lsp, lsr, lsy, leb, lwy, rwp, lwp;
	float pWST[4], pRSHLD[4], pRELB[4], pLSHLD[4], pLELB[4], pRELB2[4], pLELB2[4], pRWR[4], pLWR[4];
	float temp3_3x1[4], temp4_3x1[4];
	float qPEL[5] = {0., Q_34x1[4], Q_34x1[5], Q_34x1[6], Q_34x1[7]};
	float pPC[4] = {0., Q_34x1[1], Q_34x1[2], Q_34x1[3]};

	wst = Q_34x1[8];
	rsp = Q_34x1[21];
	rsr = Q_34x1[22];
	rsy = Q_34x1[23];
	reb = Q_34x1[24];
	rwy = Q_34x1[25];
	rwp = Q_34x1[26];
	lsp = Q_34x1[27];
	lsr = Q_34x1[28];
	lsy = Q_34x1[29];
	leb = Q_34x1[30];
	lwy = Q_34x1[31];
	lwp = Q_34x1[32];

	QT2DC(qPEL, _dcPEL_3x3);
	
	RZ(wst, _Rz_WST_3x3);	
	RY(rsp, _Ry_RSP_3x3);
	RX(rsr, _Rx_RSR_3x3);
	RZ(rsy, _Rz_RSY_3x3);
	RY(reb, _Ry_REB_3x3);
	RZ(rwy, _Rz_RWY_3x3);
	RY(rwp, _Ry_RWP_3x3);	
	RY(lsp, _Ry_LSP_3x3);
	RX(lsr, _Rx_LSR_3x3);
	RZ(lsy, _Rz_LSY_3x3);
	RY(leb, _Ry_LEB_3x3);
	RZ(lwy, _Rz_LWY_3x3);
	RY(lwp, _Ry_LWP_3x3);	
	RY(PI, _Ry_PI_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_WST_3x3,3, _dcTOR_3x3);
	
	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_RSP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_RSR_3x3,3,3, (const float**)_Rz_RSY_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcRUA_3x3);

	mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REB_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_RWY_3x3,3, _dcRLA_3x3);
	mult_mm((const float**)_dcRLA_3x3,3,3, (const float**)_Ry_RWP_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP2_34x34,3,3, (const float**)_Ry_PI_3x3,3, _dcRS_3x3);

	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_LSP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_LSR_3x3,3,3, (const float**)_Rz_LSY_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcLUA_3x3);

	mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEB_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_LWY_3x3,3, _dcLLA_3x3);
	mult_mm((const float**)_dcLLA_3x3,3,3, (const float**)_Ry_LWP_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP2_34x34,3,3, (const float**)_Ry_PI_3x3,3, _dcLS_3x3);

	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_WST, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pWST);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_RSHLD, temp3_3x1); 
	sum_vv(pWST,3, temp3_3x1, pRSHLD);

	mult_mv((const float**)_dcRUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pRSHLD,3, temp3_3x1, pRELB);

	temp4_3x1[1] = _LINK_LARM[1];
	temp4_3x1[2] = 0.;
	temp4_3x1[3] = 0.;
	mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REB_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pRELB,3, temp3_3x1, pRELB2);
	temp4_3x1[1] = 0.;
	temp4_3x1[2] = _LINK_LARM[2];
	temp4_3x1[3] = _LINK_LARM[3];
	mult_mv((const float**)_dcRLA_3x3,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pRELB2,3, temp3_3x1, pRWR);

	mult_mv((const float**)_dcRS_3x3,3,3, _LINK_STICK, temp3_3x1); 
	sum_vv(pRWR,3, temp3_3x1, pRS_3x1);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_LSHLD, temp3_3x1); 
	sum_vv(pWST,3, temp3_3x1, pLSHLD);

	mult_mv((const float**)_dcLUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pLSHLD,3, temp3_3x1, pLELB);

	temp4_3x1[1] = _LINK_LARM[1];
	temp4_3x1[2] = 0.;
	temp4_3x1[3] = 0.;
	mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEB_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pLELB,3, temp3_3x1, pLELB2);
	temp4_3x1[1] = 0.;
	temp4_3x1[2] = _LINK_LARM[2];
	temp4_3x1[3] = _LINK_LARM[3];
	mult_mv((const float**)_dcLLA_3x3,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pLELB2,3, temp3_3x1, pLWR);

	mult_mv((const float**)_dcLS_3x3,3,3, _LINK_STICK, temp3_3x1); 
	sum_vv(pLWR,3, temp3_3x1, pLS_3x1);

	DC2QT((const float**)_dcRS_3x3, qRS_4x1);
	DC2QT((const float**)_dcLS_3x3, qLS_4x1);

	//-------------------------------------- local coordinates
	if(ref_coord == LOCAL)
	{
		diff_vv(pRS_3x1,3, pPC, pRS_3x1);
		diff_vv(pLS_3x1,3, pPC, pLS_3x1);
	
		trans(1.f, (const float**)_dcPEL_3x3, 3,3,_TEMP1_34x34);
		mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcRS_3x3,3, _TEMP2_34x34);
		DC2QT((const float**)_TEMP2_34x34, qRS_4x1);
		mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcLS_3x3,3, _TEMP2_34x34);
		DC2QT((const float**)_TEMP2_34x34, qLS_4x1);
	}
	

	return 0;
}


int QT2DC(const float qt_4x1[5], float **DC_3x3)		// convert a quaternion to a direction cosine matrix
{
	float q0 = qt_4x1[1];
	float q1 = qt_4x1[2];
	float q2 = qt_4x1[3];
	float q3 = qt_4x1[4];

	DC_3x3[1][1] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
	DC_3x3[1][2] = 2*(q1*q2-q0*q3);
	DC_3x3[1][3] = 2*(q1*q3+q0*q2);
	DC_3x3[2][1] = 2*(q1*q2+q0*q3);
	DC_3x3[2][2] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
	DC_3x3[2][3] = 2*(q2*q3-q0*q1);
	DC_3x3[3][1] = 2*(q1*q3-q0*q2);
	DC_3x3[3][2] = 2*(q2*q3+q0*q1);
	DC_3x3[3][3] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	return 0;
}



int DC2QT(const float **DC_3x3, float qt_4x1[5])
{
	unsigned int index;
	float q_sq[4], temp, max;

	q_sq[0] = 0.25f*(1.f + DC_3x3[1][1] + DC_3x3[2][2] + DC_3x3[3][3]);
	q_sq[1] = 0.25f*(1.f + DC_3x3[1][1] - DC_3x3[2][2] - DC_3x3[3][3]);
	q_sq[2] = 0.25f*(1.f - DC_3x3[1][1] + DC_3x3[2][2] - DC_3x3[3][3]);
	q_sq[3] = 0.25f*(1.f - DC_3x3[1][1] - DC_3x3[2][2] + DC_3x3[3][3]);

	findmax(q_sq, 4, &max, &index);

	switch(index)
	{
	case 0:
		qt_4x1[1] = (float)sqrt(max);
		temp = 4.f*qt_4x1[1];
		qt_4x1[2] = (DC_3x3[3][2]-DC_3x3[2][3])/temp;
		qt_4x1[3] = (DC_3x3[1][3]-DC_3x3[3][1])/temp;
		qt_4x1[4] = (DC_3x3[2][1]-DC_3x3[1][2])/temp;
		break;
	case 1:
		qt_4x1[2] = (float)sqrt(max);
		temp = 4.f*qt_4x1[2];
		qt_4x1[1] = (DC_3x3[3][2]-DC_3x3[2][3])/temp;
		qt_4x1[3] = (DC_3x3[2][1]+DC_3x3[1][2])/temp;
		qt_4x1[4] = (DC_3x3[1][3]+DC_3x3[3][1])/temp;
		break;
	case 2:
		qt_4x1[3] = (float)sqrt(max);
		temp = 4.f*qt_4x1[3];
		qt_4x1[1] = (DC_3x3[1][3]-DC_3x3[3][1])/temp;
		qt_4x1[2] = (DC_3x3[2][1]+DC_3x3[1][2])/temp;
		qt_4x1[4] = (DC_3x3[3][2]+DC_3x3[2][3])/temp;
		break;
	case 3:
		qt_4x1[4] = (float)sqrt(max);
		temp = 4.f*qt_4x1[4];
		qt_4x1[1] = (DC_3x3[2][1]-DC_3x3[1][2])/temp;
		qt_4x1[2] = (DC_3x3[1][3]-DC_3x3[3][1])/temp;
		qt_4x1[3] = (DC_3x3[3][2]+DC_3x3[2][3])/temp;
		break;
	}

	return 0;
}


int QTdel(const float *des_qt_4x1, const float *qt_4x1, float *result_3x1) // delta quaternion
{
	/*//----------------- original
	float q0;
	float des_q0 = des_qt_4x1[1];
	float q[4], des_q[4];
	float temp[4];
	float qt_4x1[5];

	float **temp_3x3;
	temp_3x3 = matrix(1,3,1,3);

	QT2DC((const float*)qt_4x1, temp_3x3);
	DC2QT((const float**)temp_3x3, qt_4x1);

	q0 = qt_4x1[1];
	q[1] = qt_4x1[2];
	q[2] = qt_4x1[3];
	q[3] = qt_4x1[4];

	des_q[1] = des_qt_4x1[2];
	des_q[2] = des_qt_4x1[3];
	des_q[3] = des_qt_4x1[4];

	cross(1.f, q, des_q, temp);

	result_3x1[1] = q0*des_q[1] - des_q0*q[1] - temp[1];
	result_3x1[2] = q0*des_q[2] - des_q0*q[2] - temp[2];
	result_3x1[3] = q0*des_q[3] - des_q0*q[3] - temp[3];

	free_matrix(temp_3x3, 1,3,1,3);

	return 0;
	//--------------*/

	//----------------using DC
	float temp1[4];	
	float **temp3_3x3, **temp4_3x3;
	temp3_3x3 = matrix(1,3,1,3);
	temp4_3x3 = matrix(1,3,1,3);

	result_3x1[1] = 0.f;  
	result_3x1[2] = 0.f;
	result_3x1[3] = 0.f;

	QT2DC((const float*)qt_4x1, temp3_3x3);
	QT2DC((const float*)des_qt_4x1, temp4_3x3);

	trans2(1.f, temp3_3x3, 3,3);
	trans2(1.f, temp4_3x3, 3,3);

	cross(1.f, (const float*)temp3_3x3[1], (const float*)temp4_3x3[1], result_3x1);  
	cross(1.f, (const float*)temp3_3x3[2], (const float*)temp4_3x3[2], temp1);
	result_3x1[1] += temp1[1];
	result_3x1[2] += temp1[2];
	result_3x1[3] += temp1[3];

	cross(1.f, (const float*)temp3_3x3[3], (const float*)temp4_3x3[3], temp1);
	result_3x1[1] += temp1[1];
	result_3x1[2] += temp1[2];
	result_3x1[3] += temp1[3];

	result_3x1[1] *= 0.5f;
	result_3x1[2] *= 0.5f;
	result_3x1[3] *= 0.5f;

	free_matrix(temp3_3x3, 1,3,1,3);
	free_matrix(temp4_3x3, 1,3,1,3);

	return 0;
}

int Wmatrix(const float *w_3x1, float **wmatrix_4x4)
{
	wmatrix_4x4[1][1] = 0.;
	wmatrix_4x4[1][2] = -w_3x1[1];
	wmatrix_4x4[1][3] = -w_3x1[2];
	wmatrix_4x4[1][4] = -w_3x1[3];

	wmatrix_4x4[2][1] = w_3x1[1];
	wmatrix_4x4[2][2] = 0.;
	wmatrix_4x4[2][3] = -w_3x1[3];
	wmatrix_4x4[2][4] = w_3x1[2];

	wmatrix_4x4[3][1] = w_3x1[2];
	wmatrix_4x4[3][2] = w_3x1[3];
	wmatrix_4x4[3][3] = 0.;
	wmatrix_4x4[3][4] = -w_3x1[1];

	wmatrix_4x4[4][1] = w_3x1[3];
	wmatrix_4x4[4][2] = -w_3x1[2];
	wmatrix_4x4[4][3] = w_3x1[1];
	wmatrix_4x4[4][4] = 0.;
	
	return 0;
}


int Wq(int ref, const float *qt_4x1, float **Wq_3x4)
{
  if(ref == 0) // global frame  
  {
    Wq_3x4[1][1] = -qt_4x1[2];
    Wq_3x4[1][2] = qt_4x1[1];
    Wq_3x4[1][3] = -qt_4x1[4];
    Wq_3x4[1][4] = qt_4x1[3];
    Wq_3x4[2][1] = -qt_4x1[3];
    Wq_3x4[2][2] = qt_4x1[4];
    Wq_3x4[2][3] = qt_4x1[1];
    Wq_3x4[2][4] = -qt_4x1[2];
    Wq_3x4[3][1] = -qt_4x1[4];
    Wq_3x4[3][2] = -qt_4x1[3];
    Wq_3x4[3][3] = qt_4x1[2];
    Wq_3x4[3][4] = qt_4x1[1];
  }
  else // body frame
  {
    Wq_3x4[1][1] = -qt_4x1[2];
    Wq_3x4[1][2] = qt_4x1[1];
    Wq_3x4[1][3] = qt_4x1[4];
    Wq_3x4[1][4] = -qt_4x1[3];
    Wq_3x4[2][1] = -qt_4x1[3];
    Wq_3x4[2][2] = -qt_4x1[4];
    Wq_3x4[2][3] = qt_4x1[1];
    Wq_3x4[2][4] = qt_4x1[2];
    Wq_3x4[3][1] = -qt_4x1[4];
    Wq_3x4[3][2] = qt_4x1[3];
    Wq_3x4[3][3] = -qt_4x1[2];
    Wq_3x4[3][4] = qt_4x1[1];
  }
  return 0;
}


int Qq(int ref, const float *qt_4x1, float **Qq_4x4) // quaternion matrix
{
  // qt1 * qt2 = Qq(qt1)*qt2 = Qq_(qt2)*qt1
  // dc1*dc2 = Qq_(qt1)*qt2
	
  if(ref==0)  // return Qq
  {
    Qq_4x4[1][1] = qt_4x1[1];
    Qq_4x4[1][2] = -qt_4x1[2];
    Qq_4x4[1][3] = -qt_4x1[3];
    Qq_4x4[1][4] = -qt_4x1[4];
    
    Qq_4x4[2][1] = qt_4x1[2];
    Qq_4x4[2][2] = qt_4x1[1];
    Qq_4x4[2][3] = qt_4x1[4];
    Qq_4x4[2][4] = -qt_4x1[3];
    
    Qq_4x4[3][1] = qt_4x1[3];
    Qq_4x4[3][2] = -qt_4x1[4];
    Qq_4x4[3][3] = qt_4x1[1];
    Qq_4x4[3][4] = qt_4x1[2];
    
    Qq_4x4[4][1] = qt_4x1[4];
    Qq_4x4[4][2] = qt_4x1[3];
    Qq_4x4[4][3] = -qt_4x1[2];
    Qq_4x4[4][4] = qt_4x1[1];
    
  }
  else  // return Qq_
  {
    Qq_4x4[1][1] = qt_4x1[1];
    Qq_4x4[1][2] = -qt_4x1[2];
    Qq_4x4[1][3] = -qt_4x1[3];
    Qq_4x4[1][4] = -qt_4x1[4];
    
    Qq_4x4[2][1] = qt_4x1[2];
    Qq_4x4[2][2] = qt_4x1[1];
    Qq_4x4[2][3] = -qt_4x1[4];
    Qq_4x4[2][4] = qt_4x1[3];
    
    Qq_4x4[3][1] = qt_4x1[3];
    Qq_4x4[3][2] = qt_4x1[4];
    Qq_4x4[3][3] = qt_4x1[1];
    Qq_4x4[3][4] = -qt_4x1[2];
    
    Qq_4x4[4][1] = qt_4x1[4];
    Qq_4x4[4][2] = -qt_4x1[3];
    Qq_4x4[4][3] = qt_4x1[2];
    Qq_4x4[4][4] = qt_4x1[1];
  }
  return 0;
}

int QThat(const float *qt_4x1, float **qthat_4x3)
{
	qthat_4x3[1][1] = -qt_4x1[2];
	qthat_4x3[1][2] = -qt_4x1[3];
	qthat_4x3[1][3] = -qt_4x1[4];

	qthat_4x3[2][1] = qt_4x1[1];
	qthat_4x3[2][2] = qt_4x1[4];
	qthat_4x3[2][3] = -qt_4x1[3];

	qthat_4x3[3][1] = -qt_4x1[4];
	qthat_4x3[3][2] = qt_4x1[1];
	qthat_4x3[3][3] = qt_4x1[2];

	qthat_4x3[4][1] = qt_4x1[3];
	qthat_4x3[4][2] = -qt_4x1[2];
	qthat_4x3[4][3] = qt_4x1[1];

	return 0;
}


int Skewsym(const float *vector_3x1, float **matrix_3x3)	// skew-symmetric cross product matrix
{
	matrix_3x3[1][1] = 0.;
	matrix_3x3[1][2] = -vector_3x1[3];
	matrix_3x3[1][3] = vector_3x1[2];

	matrix_3x3[2][1] = vector_3x1[3];
	matrix_3x3[2][2] = 0.;
	matrix_3x3[2][3] = -vector_3x1[1];

	matrix_3x3[3][1] = -vector_3x1[2];
	matrix_3x3[3][2] = vector_3x1[1];
	matrix_3x3[3][3] = 0.;

	return 0;
}


int QTcross(const float *qt1_4x1, const float *qt2_4x1, float *result_4x1)
{
	result_4x1[1] = qt1_4x1[1]*qt2_4x1[1] - qt1_4x1[2]*qt2_4x1[2] - qt1_4x1[3]*qt2_4x1[3] - qt1_4x1[4]*qt2_4x1[4];
	result_4x1[2] = qt1_4x1[1]*qt2_4x1[2] + qt1_4x1[2]*qt2_4x1[1] + qt1_4x1[3]*qt2_4x1[4] - qt1_4x1[4]*qt2_4x1[3];
	result_4x1[3] = qt1_4x1[1]*qt2_4x1[3] + qt1_4x1[3]*qt2_4x1[1] - qt1_4x1[2]*qt2_4x1[4] + qt1_4x1[4]*qt2_4x1[2];
	result_4x1[4] = qt1_4x1[1]*qt2_4x1[4] + qt1_4x1[2]*qt2_4x1[3] - qt1_4x1[3]*qt2_4x1[2] + qt1_4x1[4]*qt2_4x1[1];

	return 0;
}

int QTbar(const float *qt_4x1, float *result_4x1)
{
	result_4x1[1] = qt_4x1[1];
	result_4x1[2] = -qt_4x1[2];
	result_4x1[3] = -qt_4x1[3];
	result_4x1[4] = -qt_4x1[4];

	return 0;
}


int RX(float _theta, float **R_3x3)
{
	R_3x3[1][1] = 1.f;
	R_3x3[1][2] = 0;
	R_3x3[1][3] = 0;

	R_3x3[2][1] = 0;
	R_3x3[2][2] = (float)cos(_theta);
	R_3x3[2][3] = -(float)sin(_theta);

	R_3x3[3][1] = 0;
	R_3x3[3][2] = -R_3x3[2][3];
	R_3x3[3][3] = R_3x3[2][2];

	return 0;
}


int RY(float _theta, float **R_3x3)
{
	R_3x3[1][1] = (float)cos(_theta);
	R_3x3[1][2] = 0;
	R_3x3[1][3] = (float)sin(_theta);

	R_3x3[2][1] = 0;
	R_3x3[2][2] = 1.f;
	R_3x3[2][3] = 0;

	R_3x3[3][1] = -R_3x3[1][3];
	R_3x3[3][2] = 0;
	R_3x3[3][3] = R_3x3[1][1];

	return 0;
}


int RZ(float _theta, float **R_3x3)
{
	R_3x3[1][1] = (float)cos(_theta);
	R_3x3[1][2] = -(float)sin(_theta);
	R_3x3[1][3] = 0;

	R_3x3[2][1] = -R_3x3[1][2];
	R_3x3[2][2] = R_3x3[1][1];
	R_3x3[2][3] = 0;

	R_3x3[3][1] = 0;
	R_3x3[3][2] = 0;
	R_3x3[3][3] = 1.f;

	return 0;
}



int RVALS(float x, float xp, float xpp_ref, float xpmax, float xppmax, float xmin, float xmax, float margin, float *xpp_result, float *bound_l, float *bound_u)	// Operational Range, Velocity and Acceleration Limit Strategy
{
	float Aupper_temp[2], Alower_temp[2];
	float f_temp;

	Aupper_temp[0] = (xpmax-xp)/DT;
	if(xmax < (x+xp*DT+0.5f*xppmax*DT*DT))
		Aupper_temp[1] = (float)FMAX((0.f-xp)/DT, -xppmax);
	else
		Aupper_temp[1] = (float)FMAX( ((float)sqrtf(2.f*xppmax*(xmax-margin-(x+xp*DT+0.5f*xppmax*DT*DT)))-xp)/DT ,-xppmax);

	Alower_temp[0] = (-xpmax-xp)/DT;
	if(xmin > (x+xp*DT-0.5f*xppmax*DT*DT))
		Alower_temp[1] = (float)FMIN((0.f-xp)/DT, xppmax);
	else
		Alower_temp[1] = (float)FMIN( (-(float)sqrtf(2.f*xppmax*((x+xp*DT+0.5f*xppmax*DT*DT)-xmin-margin))-xp)/DT , xppmax);

	//----------------------- position, velocity and acceleration ranges are limited
	f_temp = (float)FMIN(Aupper_temp[0],Aupper_temp[1]);
	*bound_u = (float)FMIN(f_temp, xppmax);
	f_temp = (float)FMAX(Alower_temp[0],Alower_temp[1]);
	*bound_l = (float)FMAX(f_temp, -xppmax);
	//----------------

	//----------------------- position ranges are limited
	//*bound_u = Aupper_temp[1];
	//*bound_l = Alower_temp[1];
	//----------------
  
	if(*bound_l > *bound_u)
	{
		if(*bound_u < 0.)
			*bound_u = *bound_l;
		else
			*bound_l = *bound_u;
	}
  
  if(xpp_ref < *bound_l)
    *xpp_result = *bound_l;
  else if(xpp_ref > *bound_u)
    *xpp_result = *bound_u;
  else
    *xpp_result = xpp_ref;
    
	return 0;
}


int RVALS3(float x, float xp, float xpmax, float xppmax, float xmin, float xmax, float margin, float *bound_l, float *bound_u)
{
	float Aupper_temp[2], Alower_temp[2];

	Aupper_temp[0] = (xpmax-xp)/DT;
	if(xmax < (x+xp*DT+0.5f*xppmax*DT*DT))
		Aupper_temp[1] = (float)FMAX((0.f-xp)/DT, -xppmax);
	else
		Aupper_temp[1] = (float)FMAX( ((float)sqrtf(2.f*xppmax*(xmax-margin-(x+xp*DT+0.5f*xppmax*DT*DT)))-xp)/DT ,-xppmax);

	Alower_temp[0] = (-xpmax-xp)/DT;
	if(xmin > (x+xp*DT-0.5f*xppmax*DT*DT))
		Alower_temp[1] = (float)FMIN((0.f-xp)/DT, xppmax);
	else
		Alower_temp[1] = (float)FMIN( (-(float)sqrtf(2.f*xppmax*((x+xp*DT+0.5f*xppmax*DT*DT)-xmin-margin))-xp)/DT , xppmax);

	//----------------------- position, velocity and acceleration ranges are limited
	// f_temp = (float)FMIN(Aupper_temp[0],Aupper_temp[1]);
	// *bound_u = (float)FMIN(f_temp, xppmax);
	// f_temp = (float)FMAX(Alower_temp[0],Alower_temp[1]);
	// *bound_l = (float)FMAX(f_temp, -xppmax);
	//----------------

	//----------------------- position ranges are limited
	*bound_u = Aupper_temp[1];
	*bound_l = Alower_temp[1];
	//----------------

	if(*bound_l > *bound_u)
	{
		if(*bound_u < 0.)
			*bound_u = *bound_l;
		else
			*bound_l = *bound_u;
	}

	return 0;
}



int ZLS(float Cx, float Cxp, float Cxpp_ref, float Cz, float Czpp, float ZMPxmin, float ZMPxmax, float ZMPz, float margin, float *result_)	// ZMP-lmited COM controller
{
	float Kv = 1.f/DT;
	float g = 9.81f;
	float temp = (Czpp+g)/(ZMPz-Cz);
	float temp2 = (float)sqrt(-temp);

	float Amax = -(Cx-(ZMPxmin+margin))*temp;
	float Amin = (ZMPxmax-margin-Cx)*temp;

	float A, V;
	float Vmax, Vmin;
	float Cx_next, Cxp_next;
	float r_max, r_min;

	if(Cxpp_ref > Amax)
		A = Amax;
	else if(Cxpp_ref < Amin)
		A = Amin;
	else
		A = Cxpp_ref;

	Cx_next = Cx + Cxp*DT + 0.5f*A*DT*DT;
	Cxp_next = Cxp + A*DT;

	r_max = ZMPxmax - Cx_next;
	r_min = Cx_next - ZMPxmin;

	if(r_max > margin)
		Vmax = temp2*(r_max-margin);
	else
		Vmax = 0.;

	if(r_min > margin)
		Vmin = -temp2*(r_min-margin);
	else
		Vmin = 0;

	V = Cxp_next;

	if(V > Vmax)
		V = Vmax;
	else if(V < Vmin)
		V= Vmin;


	A = Kv*(V-Cxp);

	if(A > Amax)
		A = Amax;
	else if(A < Amin)
		A = Amin;


	*result_ = A;		// Cxpp = A

	if(fabs(A-Cxpp_ref) < 5e-5)
		return 0;		// not limited
	else
		return 1;		// limited

}


int UpdatePassiveCoord_SSP(int LorR)
{
	int i;

	_Q_34x1[1] = 0.f;
	_Q_34x1[2] = 0.f;
	_Q_34x1[3] = 0.f;
	_Q_34x1[4] = 1.f;
	_Q_34x1[5] = 0.f;
	_Q_34x1[6] = 0.f;
	_Q_34x1[7] = 0.f;
	
	_Q0_34x1[1] = _Q_34x1[1] = 0.f;
	_Q0_34x1[2] = _Q_34x1[2] = 0.f;

	FKine_Whole();

	if(LorR == 1)
	{
		_Q0_34x1[3] = _Q_34x1[3] = -_pLF_3x1[3];
		QT2DC(_qLF_4x1, _TEMP1_34x34);
		trans(1.f, (const float**)_TEMP1_34x34,3,3, _TEMP2_34x34);
	}
	else
	{
		_Q0_34x1[3] = _Q_34x1[3] = -_pRF_3x1[3];		
		QT2DC(_qRF_4x1, _TEMP1_34x34);
		trans(1.f, (const float**)_TEMP1_34x34,3,3, _TEMP2_34x34);
	}

	DC2QT((const float**)_TEMP2_34x34, &_Q_34x1[3]);
	_Q0_34x1[4] = _Q_34x1[4];
	_Q0_34x1[5] = _Q_34x1[5];
	_Q0_34x1[6] = _Q_34x1[6];
	_Q0_34x1[7] = _Q_34x1[7];

	FKine_Whole();

	for(i=1; i<=3; i++)
	{
		_pCOM0_3x1[i] = _pCOM_3x1[i];
		_pRF0_3x1[i] = _pRF_3x1[i];
		_pLF0_3x1[i] = _pLF_3x1[i];
		_pRH0_3x1[i] = _pRH_3x1[i];
		_pLH0_3x1[i] = _pLH_3x1[i];
		_qRF0_4x1[i] = _qRF_4x1[i];
		_qLF0_4x1[i] = _qLF_4x1[i];
		_qRH0_4x1[i] = _qRH_4x1[i];
		_qLH0_4x1[i] = _qLH_4x1[i];
	}
	_qRF0_4x1[4] = _qRF_4x1[4];
	_qLF0_4x1[4] = _qLF_4x1[4];
	_qRH0_4x1[4] = _qRH_4x1[4];
	_qLH0_4x1[4] = _qLH_4x1[4];

	_PassiveUpdatedFlag = 1;

	return 0;
}


void wberror(char error_text[])
/* whole-body motion error handler */
{
	printf("\nWhole-body motion run-time error...\n");
	printf("%s\n",error_text);
}



int derive3(float x0_in, float x1_in, float x2_in, float *xd1_out, float *xdd1_out, float dt_in)
{
	*xd1_out = (x2_in-x0_in)/(2.f*dt_in);
	*xdd1_out = (x2_in-2.f*x1_in+x0_in)/(dt_in*dt_in);
	
	return 0;
}

int derive3QT(float *q_past_4x1, float *q_4x1, float *q_next_4x1, float *w_result_3x1, float dt)
{
	QT2DC(q_past_4x1, _TEMP3_34x34);
	QT2DC(q_next_4x1, _TEMP4_34x34);
	diff_mm((const float**)_TEMP4_34x34,3,3, (const float**)_TEMP3_34x34, _TEMP3_34x34);
	QT2DC(q_4x1, _TEMP4_34x34);
	trans2(1.f, _TEMP4_34x34,3,3);
	mult_smm(0.5f/dt, (const float**)_TEMP3_34x34, 3,3, (const float**)_TEMP4_34x34,3, _TEMP2_34x34);

	w_result_3x1[1] = _TEMP2_34x34[3][2];
	w_result_3x1[2] = _TEMP2_34x34[1][3];
	w_result_3x1[3] = _TEMP2_34x34[2][1];

	return 0;
}


int PushSB(float dForward, float dYaw, float ROT, unsigned int Nsteps, float Tstep, char putTogether)  // dForward:step length, dYaw:body rotation, ROT:radius of turn, Tstep:step time
{
	unsigned int head_new = (_SBR.head+1)%STEP_BUNDLE_RING_SIZE;
	
	if(head_new == _SBR.tail)
		return -1; // overflow
	
	_SBR.step_bundles[head_new].dForward = dForward;
	_SBR.step_bundles[head_new].dYaw = dYaw;
	_SBR.step_bundles[head_new].ROT = ROT;
	_SBR.step_bundles[head_new].Nsteps = Nsteps;
	if(Tstep > T_STEP_MAX)
		_SBR.step_bundles[head_new].Tstep = T_STEP_MAX;
	else if (Tstep < T_STEP_MIN)
		_SBR.step_bundles[head_new].Tstep = T_STEP_MIN;
	else
		_SBR.step_bundles[head_new].Tstep = Tstep;
	_SBR.step_bundles[head_new].n_stepped = 0;
	_SBR.step_bundles[head_new].putTogether = putTogether;
	_SBR.step_bundles[head_new].pseudo_step = 0;
	_SBR.step_bundles[head_new].offsetR = 0.;
	_SBR.step_bundles[head_new].offsetL = 0.;
	_SBR.step_bundles[head_new].halfFlag = 0;

	if((dForward==0.) && (ROT==0.) && (putTogether==0) && (dYaw != 0.))	// if spin
		_SBR.step_bundles[head_new].Nsteps *= 2;

	if(putTogether != 0)
		_SBR.step_bundles[head_new].Nsteps = 1;

	_SBR.head = head_new;
	return 0;
}


int StepPositioner(void)
{
	unsigned int tail_next;
	int buffered = 0;
	int i, j;

	
	if(_SBR.step_bundles[_SBR.tail].n_stepped >= _SBR.step_bundles[_SBR.tail].Nsteps)
	{
		if(_SBR.tail == _SBR.head)
		{
			_TwoStepBuff[0].isEmpty = 1;
			_TwoStepBuff[1].isEmpty = 1;

			if(_SBR.step_bundles[_SBR.tail].n_stepped >= _SBR.step_bundles[_SBR.tail].Nsteps+2)
				_SBR.step_bundles[_SBR.tail].n_stepped = _SBR.step_bundles[_SBR.tail].Nsteps + 2;
			return (WTG(WTG_MODE));
		}
		else
		{
			_SBR.tail = (_SBR.tail+1)%STEP_BUNDLE_RING_SIZE;
			_SBR.step_bundles[_SBR.tail].n_stepped = 0;
		}
	}

	tail_next = _SBR.tail;
	_SBR.step_bundles[tail_next].pseudo_step = _SBR.step_bundles[tail_next].n_stepped+1;	

	if(_SBR.step_bundles[tail_next].pseudo_step == 1)
	{
		_SBR.step_bundles[tail_next].halfFlag = 0;
		_SBR.step_bundles[tail_next].offsetL = 0.;
		_SBR.step_bundles[tail_next].offsetR = 0.;
	}

	for(j=0; j<2; j++)
	{		
		if(StepPositioning(tail_next, buffered) == 0)		
			buffered++;
		else
		{
			for(i=buffered; i<2; i++)
				_TwoStepBuff[i].isEmpty = 1;
			return (WTG(WTG_MODE));
		}
			
		if(buffered == 1)
		{
			if(_SBR.step_bundles[tail_next].pseudo_step == _SBR.step_bundles[tail_next].Nsteps)
			{			
				tail_next = (tail_next+1)%STEP_BUNDLE_RING_SIZE;			
				if(tail_next == ((_SBR.head+1)%STEP_BUNDLE_RING_SIZE))
				{
					for(i=buffered; i<2; i++)
						_TwoStepBuff[i].isEmpty = 1;
					return (WTG(WTG_MODE));
				}
				else			
				{
					_SBR.step_bundles[tail_next].pseudo_step = 1;
					_SBR.step_bundles[tail_next].halfFlag = 0;
					_SBR.step_bundles[tail_next].offsetL = 0.;
					_SBR.step_bundles[tail_next].offsetR = 0.;
				}
			}
			else
			_SBR.step_bundles[tail_next].pseudo_step++;
		}		
	}
	return (WTG(WTG_MODE));
}


int StepPositioning(unsigned int bundle_no, int buffer_no)
{
	int sp_case;
	float pRF_last_3x1[4], pLF_last_3x1[4], qRF_last_4x1[5], qLF_last_4x1[5], yRF_last, yLF_last;
	float f_temp, f_temp2, f_temp3;
	float yaw_avg, dforward, dyaw, rot;
	float center_temp_3x1[4], center_next_3x1[4], pRF_local_3x1[4], pLF_local_3x1[4];
	float temp1_3x1[4], temp2_3x1[4];

	static float offsetL, offsetR;
	static char halfFlag;
	static float pRF_2nd_3x1[4], pLF_2nd_3x1[4], yRF_2nd, yLF_2nd;

	if(_FKineUpdatedFlag != 1)
		return -4;

	_TwoStepBuff[buffer_no].Tstep= _SBR.step_bundles[bundle_no].Tstep;

	if(_SBR.step_bundles[bundle_no].dForward == 0.)
	{
		if(_SBR.step_bundles[bundle_no].dYaw == 0.)
			sp_case = 0;
		else
		{
			if(_SBR.step_bundles[bundle_no].dYaw > dYAW_MAX)
			{				
				dyaw = dYAW_MAX;
				_SBR.step_bundles[bundle_no].dYaw = dyaw;
			}
			else if(_SBR.step_bundles[bundle_no].dYaw < -dYAW_MAX)
			{
				dyaw = -dYAW_MAX;
				_SBR.step_bundles[bundle_no].dYaw = dyaw;
			}
			else
				dyaw = _SBR.step_bundles[bundle_no].dYaw;

			if(_SBR.step_bundles[bundle_no].ROT == 0.)
			{
				if(_SBR.step_bundles[bundle_no].dYaw > 0.)
					sp_case = 2;	// spin CCW
				else
					sp_case = 3;	// spin CW
			}
			else
			{
				rot = _SBR.step_bundles[bundle_no].ROT;
				sp_case = 4;		// left turn(forward,backward) or right turn(forward,backward)			
			}
		}
	}
	else
		sp_case = 1;  // straight forward or backward

	if((_SBR.step_bundles[bundle_no].putTogether == RFSP) || (_SBR.step_bundles[bundle_no].putTogether == LFSP) || 
		(_SBR.step_bundles[bundle_no].putTogether == 2) || (_SBR.step_bundles[bundle_no].putTogether == -2))
		sp_case = 5;


	if(buffer_no == 0)
	{
		pRF_last_3x1[1] = _pRF_3x1[1];
		pRF_last_3x1[2] = _pRF_3x1[2];
		pRF_last_3x1[3] = _pRF_3x1[3];

		pLF_last_3x1[1] = _pLF_3x1[1];
		pLF_last_3x1[2] = _pLF_3x1[2];
		pLF_last_3x1[3] = _pLF_3x1[3];

		qRF_last_4x1[1] = _qRF_4x1[1];
		qRF_last_4x1[2] = _qRF_4x1[2];
		qRF_last_4x1[3] = _qRF_4x1[3];
		qRF_last_4x1[4] = _qRF_4x1[4];

		qLF_last_4x1[1] = _qLF_4x1[1];
		qLF_last_4x1[2] = _qLF_4x1[2];
		qLF_last_4x1[3] = _qLF_4x1[3];
		qLF_last_4x1[4] = _qLF_4x1[4];

		f_temp = (float)sqrt(qRF_last_4x1[1]*qRF_last_4x1[1] + qRF_last_4x1[4]*qRF_last_4x1[4]);
		yRF_last = (float)atan2(qRF_last_4x1[4]/f_temp, qRF_last_4x1[1]/f_temp)*2.f;

		f_temp = (float)sqrt(qLF_last_4x1[1]*qLF_last_4x1[1] + qLF_last_4x1[4]*qLF_last_4x1[4]);
		yLF_last = (float)atan2(qLF_last_4x1[4]/f_temp, qLF_last_4x1[1]/f_temp)*2.f;
	}
	else
	{
		pRF_last_3x1[1] = _TwoStepBuff[buffer_no-1].pRF_3x1[1];
		pRF_last_3x1[2] = _TwoStepBuff[buffer_no-1].pRF_3x1[2];
		pRF_last_3x1[3] = _TwoStepBuff[buffer_no-1].pRF_3x1[3];

		pLF_last_3x1[1] = _TwoStepBuff[buffer_no-1].pLF_3x1[1];
		pLF_last_3x1[2] = _TwoStepBuff[buffer_no-1].pLF_3x1[2];
		pLF_last_3x1[3] = _TwoStepBuff[buffer_no-1].pLF_3x1[3];

		yRF_last = _TwoStepBuff[buffer_no-1].yRF;
		yLF_last = _TwoStepBuff[buffer_no-1].yLF;
	}

	switch(sp_case)
	{
	case 0:		//--------------------------------------------- stay 
		_TwoStepBuff[buffer_no].pRF_3x1[1] = pRF_last_3x1[1];
		_TwoStepBuff[buffer_no].pRF_3x1[2] = pRF_last_3x1[2];
		_TwoStepBuff[buffer_no].pRF_3x1[3] = pRF_last_3x1[3];

		_TwoStepBuff[buffer_no].pLF_3x1[1] = pLF_last_3x1[1];
		_TwoStepBuff[buffer_no].pLF_3x1[2] = pLF_last_3x1[2];
		_TwoStepBuff[buffer_no].pLF_3x1[3] = pLF_last_3x1[3];
	
		_TwoStepBuff[buffer_no].yRF = yRF_last;
		_TwoStepBuff[buffer_no].yLF = yLF_last;
		_TwoStepBuff[buffer_no].supporting_foot = DFSP;		
		_TwoStepBuff[buffer_no].isEmpty = 0;
		break;

	case 1:		//--------------------------------------------- straight
		if(_SBR.step_bundles[bundle_no].dForward > dFORWARD_MAX)
			dforward = dFORWARD_MAX*0.5f;
		else if(_SBR.step_bundles[bundle_no].dForward < -dFORWARD_MAX)
			dforward = -dFORWARD_MAX*0.5f;
		else
			dforward = _SBR.step_bundles[bundle_no].dForward*0.5f;

		yaw_avg = 0.5f*(yRF_last+yLF_last);
		RZ(yaw_avg, _TEMP1_34x34);  //  center frame of the feet
		trans(1.f, (const float**)_TEMP1_34x34,3,3, _TEMP2_34x34);		
		center_temp_3x1[1] = 0.5f*(pRF_last_3x1[1]+pLF_last_3x1[1]);
		center_temp_3x1[2] = 0.5f*(pRF_last_3x1[2]+pLF_last_3x1[2]);
		center_temp_3x1[3] = 0.5f*(pRF_last_3x1[3]+pLF_last_3x1[3]);
		
		temp1_3x1[1] = pRF_last_3x1[1]-center_temp_3x1[1];
		temp1_3x1[2] = pRF_last_3x1[2]-center_temp_3x1[2];
		temp1_3x1[3] = pRF_last_3x1[3]-center_temp_3x1[3];		
		mult_mv((const float**)_TEMP2_34x34, 3,3,temp1_3x1, pRF_local_3x1);

		temp1_3x1[1] = pLF_last_3x1[1]-center_temp_3x1[1];
		temp1_3x1[2] = pLF_last_3x1[2]-center_temp_3x1[2];
		temp1_3x1[3] = pLF_last_3x1[3]-center_temp_3x1[3];		
		mult_mv((const float**)_TEMP2_34x34, 3,3,temp1_3x1, pLF_local_3x1);

		if(fabs(pRF_local_3x1[1]-pLF_local_3x1[1])<EPS)  // 두 발 나란히 -> 오른 발부터 이동(약속)
		{
			temp1_3x1[1] = dforward;
			temp1_3x1[2] = -0.5f*L1_PEL;
			temp1_3x1[3] = 0.;
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
			_TwoStepBuff[buffer_no].pRF_3x1[1] = temp2_3x1[1] + center_temp_3x1[1];
			_TwoStepBuff[buffer_no].pRF_3x1[2] = temp2_3x1[2] + center_temp_3x1[2];
			_TwoStepBuff[buffer_no].pRF_3x1[3] = pRF_last_3x1[3];
			_TwoStepBuff[buffer_no].yRF = yaw_avg;
			
			_TwoStepBuff[buffer_no].supporting_foot = LFSP;
			_TwoStepBuff[buffer_no].pLF_3x1[1] = pLF_last_3x1[1];
			_TwoStepBuff[buffer_no].pLF_3x1[2] = pLF_last_3x1[2];
			_TwoStepBuff[buffer_no].pLF_3x1[3] = pLF_last_3x1[3];
			_TwoStepBuff[buffer_no].yLF = yLF_last;
		}
		else if(pRF_local_3x1[1] > pLF_local_3x1[1])  // 오른 발 전방
		{
			if(dforward > 0)
			{
				RZ(yRF_last,_TEMP1_34x34);
				temp1_3x1[1] = dforward;
				temp1_3x1[2] = L1_PEL;
				temp1_3x1[3] = 0.;
				mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
				_TwoStepBuff[buffer_no].pLF_3x1[1] = pRF_last_3x1[1] + temp2_3x1[1];
				_TwoStepBuff[buffer_no].pLF_3x1[2] = pRF_last_3x1[2] + temp2_3x1[2];
				_TwoStepBuff[buffer_no].pLF_3x1[3] = pLF_last_3x1[3];
				_TwoStepBuff[buffer_no].yLF = yRF_last;

				_TwoStepBuff[buffer_no].supporting_foot = RFSP;
				_TwoStepBuff[buffer_no].pRF_3x1[1] = pRF_last_3x1[1];
				_TwoStepBuff[buffer_no].pRF_3x1[2] = pRF_last_3x1[2];
				_TwoStepBuff[buffer_no].pRF_3x1[3] = pRF_last_3x1[3];
				_TwoStepBuff[buffer_no].yRF = yRF_last;
			}
			else
			{
				RZ(yLF_last,_TEMP1_34x34);
				temp1_3x1[1] = dforward;
				temp1_3x1[2] = -L1_PEL;
				temp1_3x1[3] = 0.;
				mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
				_TwoStepBuff[buffer_no].pRF_3x1[1] = pLF_last_3x1[1] + temp2_3x1[1];
				_TwoStepBuff[buffer_no].pRF_3x1[2] = pLF_last_3x1[2] + temp2_3x1[2];
				_TwoStepBuff[buffer_no].pRF_3x1[3] = pRF_last_3x1[3];
				_TwoStepBuff[buffer_no].yRF = yLF_last;

				_TwoStepBuff[buffer_no].supporting_foot = LFSP;
				_TwoStepBuff[buffer_no].pLF_3x1[1] = pLF_last_3x1[1];
				_TwoStepBuff[buffer_no].pLF_3x1[2] = pLF_last_3x1[2];
				_TwoStepBuff[buffer_no].pLF_3x1[3] = pLF_last_3x1[3];
				_TwoStepBuff[buffer_no].yLF = yLF_last;

			}
		}
		else  // 왼 발 전방
		{
			if(dforward > 0)
			{
				RZ(yLF_last,_TEMP1_34x34);
				temp1_3x1[1] = dforward;
				temp1_3x1[2] = -L1_PEL;
				temp1_3x1[3] = 0.;
				mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
				_TwoStepBuff[buffer_no].pRF_3x1[1] = pLF_last_3x1[1] + temp2_3x1[1];
				_TwoStepBuff[buffer_no].pRF_3x1[2] = pLF_last_3x1[2] + temp2_3x1[2];
				_TwoStepBuff[buffer_no].pRF_3x1[3] = pRF_last_3x1[3];
				_TwoStepBuff[buffer_no].yRF = yLF_last;

				_TwoStepBuff[buffer_no].supporting_foot = LFSP;
				_TwoStepBuff[buffer_no].pLF_3x1[1] = pLF_last_3x1[1];
				_TwoStepBuff[buffer_no].pLF_3x1[2] = pLF_last_3x1[2];
				_TwoStepBuff[buffer_no].pLF_3x1[3] = pLF_last_3x1[3];
				_TwoStepBuff[buffer_no].yLF = yLF_last;
			}
			else
			{
				RZ(yRF_last,_TEMP1_34x34);
				temp1_3x1[1] = dforward;
				temp1_3x1[2] = L1_PEL;
				temp1_3x1[3] = 0.;
				mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
				_TwoStepBuff[buffer_no].pLF_3x1[1] = pRF_last_3x1[1] + temp2_3x1[1];
				_TwoStepBuff[buffer_no].pLF_3x1[2] = pRF_last_3x1[2] + temp2_3x1[2];
				_TwoStepBuff[buffer_no].pLF_3x1[3] = pLF_last_3x1[3];
				_TwoStepBuff[buffer_no].yLF = yRF_last;

				_TwoStepBuff[buffer_no].supporting_foot = RFSP;
				_TwoStepBuff[buffer_no].pRF_3x1[1] = pRF_last_3x1[1];
				_TwoStepBuff[buffer_no].pRF_3x1[2] = pRF_last_3x1[2];
				_TwoStepBuff[buffer_no].pRF_3x1[3] = pRF_last_3x1[3];
				_TwoStepBuff[buffer_no].yRF = yRF_last;	
			}
		}
		_TwoStepBuff[buffer_no].isEmpty = 0;		
		break;

	case 2:   //--------------------------------------------- spin CCW
		f_temp = L1_PEL-0.05f*2.f; // 제자리 회전 반경
		yaw_avg = 0.5f*(yRF_last+yLF_last);
		RZ(yaw_avg, _TEMP1_34x34);  //  center frame of the feet
			
		center_temp_3x1[1] = 0.5f*(pRF_last_3x1[1]+pLF_last_3x1[1]);
		center_temp_3x1[2] = 0.5f*(pRF_last_3x1[2]+pLF_last_3x1[2]);
		center_temp_3x1[3] = 0.5f*(pRF_last_3x1[3]+pLF_last_3x1[3]);
		
		f_temp2 = f_temp*(float)tan(0.5f*dyaw);
		f_temp3 = 0.5f*L1_PEL;

		if(_SBR.step_bundles[bundle_no].pseudo_step%2==1)
		{
			temp1_3x1[1] = -(f_temp2+f_temp3)*(float)sin(dyaw);
			temp1_3x1[2] = f_temp2 + (f_temp2+f_temp3)*(float)cos(dyaw);
			temp1_3x1[3] = 0.;
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
			_TwoStepBuff[buffer_no].pLF_3x1[1] = temp2_3x1[1] + center_temp_3x1[1];
			_TwoStepBuff[buffer_no].pLF_3x1[2] = temp2_3x1[2] + center_temp_3x1[2];
			_TwoStepBuff[buffer_no].pLF_3x1[3] = pLF_last_3x1[3];
			_TwoStepBuff[buffer_no].yLF = yaw_avg + dyaw;

			_TwoStepBuff[buffer_no].supporting_foot = RFSP;
			_TwoStepBuff[buffer_no].pRF_3x1[1] = pRF_last_3x1[1];
			_TwoStepBuff[buffer_no].pRF_3x1[2] = pRF_last_3x1[2];
			_TwoStepBuff[buffer_no].pRF_3x1[3] = pRF_last_3x1[3];
			_TwoStepBuff[buffer_no].yRF = yRF_last;		

			temp1_3x1[1] = (f_temp3-f_temp2)*(float)sin(dyaw);
			temp1_3x1[2] = (f_temp2-f_temp3)*(float)cos(dyaw) + f_temp2;
			temp1_3x1[3] = 0.;
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
			pRF_2nd_3x1[1] = temp2_3x1[1] + center_temp_3x1[1];
			pRF_2nd_3x1[2] = temp2_3x1[2] + center_temp_3x1[2];
			pRF_2nd_3x1[3] = pRF_last_3x1[3];
			yRF_2nd = yaw_avg + dyaw;
		}
		else
		{			
			_TwoStepBuff[buffer_no].pRF_3x1[1] = pRF_2nd_3x1[1];
			_TwoStepBuff[buffer_no].pRF_3x1[2] = pRF_2nd_3x1[2];			
			_TwoStepBuff[buffer_no].pRF_3x1[3] = pRF_2nd_3x1[3];
			_TwoStepBuff[buffer_no].yRF = yRF_2nd;

			_TwoStepBuff[buffer_no].supporting_foot = LFSP;
			_TwoStepBuff[buffer_no].pLF_3x1[1] = pLF_last_3x1[1];
			_TwoStepBuff[buffer_no].pLF_3x1[2] = pLF_last_3x1[2];
			_TwoStepBuff[buffer_no].pLF_3x1[3] = pLF_last_3x1[3];
			_TwoStepBuff[buffer_no].yLF = yLF_last;

		}

		_TwoStepBuff[buffer_no].isEmpty = 0;
		break;

	case 3:   //--------------------------------------------- spin CW
		f_temp = L1_PEL-0.05f*2.f; // 제자리 회전 반경
		yaw_avg = 0.5f*(yRF_last+yLF_last);
		RZ(yaw_avg, _TEMP1_34x34);  //  center frame of the feet
			
		center_temp_3x1[1] = 0.5f*(pRF_last_3x1[1]+pLF_last_3x1[1]);
		center_temp_3x1[2] = 0.5f*(pRF_last_3x1[2]+pLF_last_3x1[2]);
		center_temp_3x1[3] = 0.5f*(pRF_last_3x1[3]+pLF_last_3x1[3]);
		
		f_temp2 = f_temp*(float)tan(-0.5f*dyaw);
		f_temp3 = 0.5f*L1_PEL;

		if(_SBR.step_bundles[bundle_no].pseudo_step%2==1)
		{
			temp1_3x1[1] = (f_temp2+f_temp3)*(float)sin(dyaw);
			temp1_3x1[2] = -(f_temp2+f_temp3)*(float)cos(dyaw) - f_temp2;
			temp1_3x1[3] = 0.;
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
			_TwoStepBuff[buffer_no].pRF_3x1[1] = temp2_3x1[1] + center_temp_3x1[1];
			_TwoStepBuff[buffer_no].pRF_3x1[2] = temp2_3x1[2] + center_temp_3x1[2];
			_TwoStepBuff[buffer_no].pRF_3x1[3] = pRF_last_3x1[3];
			_TwoStepBuff[buffer_no].yRF = yaw_avg + dyaw;

			_TwoStepBuff[buffer_no].supporting_foot = LFSP;
			_TwoStepBuff[buffer_no].pLF_3x1[1] = pLF_last_3x1[1];
			_TwoStepBuff[buffer_no].pLF_3x1[2] = pLF_last_3x1[2];
			_TwoStepBuff[buffer_no].pLF_3x1[3] = pLF_last_3x1[3];
			_TwoStepBuff[buffer_no].yLF = yLF_last;

			temp1_3x1[1] = (f_temp2-f_temp3)*(float)sin(dyaw);
			temp1_3x1[2] = (f_temp3-f_temp2)*(float)cos(dyaw)-f_temp2;
			temp1_3x1[3] = 0.;
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
			pLF_2nd_3x1[1] = temp2_3x1[1] + center_temp_3x1[1];
			pLF_2nd_3x1[2] = temp2_3x1[2] + center_temp_3x1[2];
			pLF_2nd_3x1[3] = pLF_last_3x1[3];
			yLF_2nd = yaw_avg + dyaw;
		}
		else
		{
			_TwoStepBuff[buffer_no].pLF_3x1[1] = pLF_2nd_3x1[1];
			_TwoStepBuff[buffer_no].pLF_3x1[2] = pLF_2nd_3x1[2];
			_TwoStepBuff[buffer_no].pLF_3x1[3] = pLF_2nd_3x1[3];
			_TwoStepBuff[buffer_no].yLF = yLF_2nd;

			_TwoStepBuff[buffer_no].supporting_foot = RFSP;
			_TwoStepBuff[buffer_no].pRF_3x1[1] = pRF_last_3x1[1];
			_TwoStepBuff[buffer_no].pRF_3x1[2] = pRF_last_3x1[2];
			_TwoStepBuff[buffer_no].pRF_3x1[3] = pRF_last_3x1[3];
			_TwoStepBuff[buffer_no].yRF = yRF_last;

		}
		_TwoStepBuff[buffer_no].isEmpty = 0;
		break;

	case 4:   //--------------------------------------------- turn
		if(buffer_no == 0 || _SBR.step_bundles[bundle_no].pseudo_step == 1)
		{
			halfFlag = _SBR.step_bundles[bundle_no].halfFlag;
			offsetL = _SBR.step_bundles[bundle_no].offsetL;
			offsetR = _SBR.step_bundles[bundle_no].offsetR;
		}

		if((float)((fabs(rot)+L1_PEL*0.5f)*fabs(dyaw)) > dFORWARD_MAX)
		{
			f_temp = (float)(MY_SIGN(dyaw)*dFORWARD_MAX/(fabs(rot)+L1_PEL*0.5f));
			_SBR.step_bundles[bundle_no].Nsteps = (unsigned int)(dyaw*_SBR.step_bundles[bundle_no].Nsteps/f_temp);
			_SBR.step_bundles[bundle_no].dYaw = f_temp;
			dyaw = f_temp;			
		}
		yaw_avg = 0.5f*(yLF_last+yRF_last);		
		RZ(yaw_avg, _TEMP1_34x34);  //  center frame of the feet
		trans(1.f, (const float**)_TEMP1_34x34,3,3, _TEMP2_34x34);	
			
		center_temp_3x1[1] = 0.5f*(pRF_last_3x1[1]+pLF_last_3x1[1]);
		center_temp_3x1[2] = 0.5f*(pRF_last_3x1[2]+pLF_last_3x1[2]);
		center_temp_3x1[3] = 0.5f*(pRF_last_3x1[3]+pLF_last_3x1[3]);
		
		temp1_3x1[1] = pRF_last_3x1[1]-center_temp_3x1[1];
		temp1_3x1[2] = pRF_last_3x1[2]-center_temp_3x1[2];
		temp1_3x1[3] = pRF_last_3x1[3]-center_temp_3x1[3];		
		mult_mv((const float**)_TEMP2_34x34, 3,3,temp1_3x1, pRF_local_3x1);

		temp1_3x1[1] = pLF_last_3x1[1]-center_temp_3x1[1];
		temp1_3x1[2] = pLF_last_3x1[2]-center_temp_3x1[2];
		temp1_3x1[3] = pLF_last_3x1[3]-center_temp_3x1[3];		
		mult_mv((const float**)_TEMP2_34x34, 3,3,temp1_3x1, pLF_local_3x1);
		
		if( halfFlag == 0 )
		{
			if( ((pRF_local_3x1[1]-pLF_local_3x1[1])>EPS*1000.f && dyaw<0) || ((pLF_local_3x1[1]-pRF_local_3x1[1])>EPS*1000.f && dyaw>0))
			{
				if(rot > 0)
				{
					if(buffer_no == 0)
						_SBR.step_bundles[_SBR.tail].Nsteps++;
					goto putTogetherL;
				}
				else
				{
					if(buffer_no == 0)
						_SBR.step_bundles[_SBR.tail].Nsteps++;
					goto putTogetherR;
				}
			}

			if(rot > 0.)    // left turn
			{	
				//------------ LF dyaw/2 이동
				temp1_3x1[1] = rot*(float)sin(dyaw*0.5f);
				temp1_3x1[2] = rot*(1.f-(float)cos(dyaw*0.5f));
				temp1_3x1[3] = 0.;
				mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
				center_next_3x1[1] = center_temp_3x1[1] + temp2_3x1[1]; // pCenter_next
				center_next_3x1[2] = center_temp_3x1[2] + temp2_3x1[2];
				center_next_3x1[3] = center_temp_3x1[3] + temp2_3x1[3];

				RZ(yaw_avg+0.5f*dyaw, _TEMP1_34x34);
				temp1_3x1[1] = 0.;
				temp1_3x1[2] = 0.5f*L1_PEL;
				temp1_3x1[3] = pLF_local_3x1[3];
				mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
				_TwoStepBuff[buffer_no].pLF_3x1[1] = temp2_3x1[1] + center_next_3x1[1];
				_TwoStepBuff[buffer_no].pLF_3x1[2] = temp2_3x1[2] + center_next_3x1[2];
				_TwoStepBuff[buffer_no].pLF_3x1[3] = temp2_3x1[3] + center_next_3x1[3];
				_TwoStepBuff[buffer_no].yLF = yaw_avg + 0.5f*dyaw;
				
				CheckFootCollision(pRF_last_3x1, yRF_last, _TwoStepBuff[buffer_no].pLF_3x1, _TwoStepBuff[buffer_no].yLF, RFSP, &offsetL);
								
				_TwoStepBuff[buffer_no].supporting_foot = RFSP;
				_TwoStepBuff[buffer_no].pRF_3x1[1] = pRF_last_3x1[1];
				_TwoStepBuff[buffer_no].pRF_3x1[2] = pRF_last_3x1[2];
				_TwoStepBuff[buffer_no].pRF_3x1[3] = pRF_last_3x1[3];
				_TwoStepBuff[buffer_no].yRF = yRF_last;		

				//------------  RF dyaw 이동 
				RZ(yaw_avg, _TEMP1_34x34);  //  center frame of the feet
				temp1_3x1[1] = rot*(float)sin(dyaw);
				temp1_3x1[2] = rot*(1.f-(float)cos(dyaw));
				temp1_3x1[3] = 0.;
				mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
				center_next_3x1[1] = center_temp_3x1[1] + temp2_3x1[1]; // pCenter_next
				center_next_3x1[2] = center_temp_3x1[2] + temp2_3x1[2];
				center_next_3x1[3] = center_temp_3x1[3] + temp2_3x1[3];

				RZ(yaw_avg+dyaw, _TEMP1_34x34);
				temp1_3x1[1] = 0.;
				temp1_3x1[2] = -0.5f*L1_PEL;
				temp1_3x1[3] = pRF_local_3x1[3];
				mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
				pRF_2nd_3x1[1] = temp2_3x1[1] + center_next_3x1[1];
				pRF_2nd_3x1[2] = temp2_3x1[2] + center_next_3x1[2];
				pRF_2nd_3x1[3] = temp2_3x1[3] + center_next_3x1[3];
				yRF_2nd = yaw_avg + dyaw;
				
				CheckFootCollision(pRF_2nd_3x1, yRF_2nd, _TwoStepBuff[buffer_no].pLF_3x1, _TwoStepBuff[buffer_no].yLF, LFSP, &offsetR);
			}
			else  // right turn
			{
				//------------ RF dyaw/2 이동
				temp1_3x1[1] = rot*(float)sin(dyaw*0.5f);
				temp1_3x1[2] = rot*(1.f-(float)cos(dyaw*0.5f));
				temp1_3x1[3] = 0.;
				mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
				center_next_3x1[1] = center_temp_3x1[1] + temp2_3x1[1]; // pCenter_next
				center_next_3x1[2] = center_temp_3x1[2] + temp2_3x1[2];
				center_next_3x1[3] = center_temp_3x1[3] + temp2_3x1[3];

				RZ(yaw_avg+0.5f*dyaw, _TEMP1_34x34);
				temp1_3x1[1] = 0.;
				temp1_3x1[2] = -0.5f*L1_PEL;
				temp1_3x1[3] = pRF_local_3x1[3];
				mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
				_TwoStepBuff[buffer_no].pRF_3x1[1] = temp2_3x1[1] + center_next_3x1[1];
				_TwoStepBuff[buffer_no].pRF_3x1[2] = temp2_3x1[2] + center_next_3x1[2];
				_TwoStepBuff[buffer_no].pRF_3x1[3] = temp2_3x1[3] + center_next_3x1[3];
				_TwoStepBuff[buffer_no].yRF = yaw_avg + 0.5f*dyaw;
				
				CheckFootCollision(_TwoStepBuff[buffer_no].pRF_3x1, _TwoStepBuff[buffer_no].yRF, pLF_last_3x1, yLF_last, LFSP, &offsetR);
				
				_TwoStepBuff[buffer_no].supporting_foot = LFSP;
				_TwoStepBuff[buffer_no].pLF_3x1[1] = pLF_last_3x1[1];
				_TwoStepBuff[buffer_no].pLF_3x1[2] = pLF_last_3x1[2];
				_TwoStepBuff[buffer_no].pLF_3x1[3] = pLF_last_3x1[3];
				_TwoStepBuff[buffer_no].yLF = yLF_last;		

				
				//------------  LF dyaw 이동 
				RZ(yaw_avg, _TEMP1_34x34);  //  center frame of the feet
				temp1_3x1[1] = rot*(float)sin(dyaw);
				temp1_3x1[2] = rot*(1.f-(float)cos(dyaw));
				temp1_3x1[3] = 0.;
				mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
				center_next_3x1[1] = center_temp_3x1[1] + temp2_3x1[1]; // pCenter_next
				center_next_3x1[2] = center_temp_3x1[2] + temp2_3x1[2];
				center_next_3x1[3] = center_temp_3x1[3] + temp2_3x1[3];

				RZ(yaw_avg+dyaw, _TEMP1_34x34);
				temp1_3x1[1] = 0.;
				temp1_3x1[2] = 0.5f*L1_PEL;
				temp1_3x1[3] = pLF_local_3x1[3];
				mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
				pLF_2nd_3x1[1] = temp2_3x1[1] + center_next_3x1[1];
				pLF_2nd_3x1[2] = temp2_3x1[2] + center_next_3x1[2];
				pLF_2nd_3x1[3] = temp2_3x1[3] + center_next_3x1[3];
				yLF_2nd = yaw_avg + dyaw;
				
				CheckFootCollision(_TwoStepBuff[buffer_no].pRF_3x1, _TwoStepBuff[buffer_no].yRF, pLF_2nd_3x1, yLF_2nd, RFSP, &offsetL);
			}
			halfFlag = 1;
		}
		else if( halfFlag == 1 )
		{			
			if(rot > 0.)    // left turn
			{												
				_TwoStepBuff[buffer_no].pRF_3x1[1] = pRF_2nd_3x1[1];
				_TwoStepBuff[buffer_no].pRF_3x1[2] = pRF_2nd_3x1[2];
				_TwoStepBuff[buffer_no].pRF_3x1[3] = pRF_2nd_3x1[3];
				_TwoStepBuff[buffer_no].yRF = yRF_2nd;	

				_TwoStepBuff[buffer_no].supporting_foot = LFSP;
				_TwoStepBuff[buffer_no].pLF_3x1[1] = pLF_last_3x1[1];
				_TwoStepBuff[buffer_no].pLF_3x1[2] = pLF_last_3x1[2];
				_TwoStepBuff[buffer_no].pLF_3x1[3] = pLF_last_3x1[3];
				_TwoStepBuff[buffer_no].yLF = yLF_last;	

				halfFlag = 2;
			}
			else
			{
				_TwoStepBuff[buffer_no].pLF_3x1[1] = pLF_2nd_3x1[1];
				_TwoStepBuff[buffer_no].pLF_3x1[2] = pLF_2nd_3x1[2];
				_TwoStepBuff[buffer_no].pLF_3x1[3] = pLF_2nd_3x1[3];
				_TwoStepBuff[buffer_no].yLF = yLF_2nd;	

				_TwoStepBuff[buffer_no].supporting_foot = RFSP;
				_TwoStepBuff[buffer_no].pRF_3x1[1] = pRF_last_3x1[1];
				_TwoStepBuff[buffer_no].pRF_3x1[2] = pRF_last_3x1[2];
				_TwoStepBuff[buffer_no].pRF_3x1[3] = pRF_last_3x1[3];
				_TwoStepBuff[buffer_no].yRF = yRF_last;	

				halfFlag = -2;
			}			
		}
		else if( halfFlag == 2 )
		{
			RZ(yLF_last, _TEMP1_34x34);
			temp1_3x1[1] = 0.;
			if(buffer_no==0)
				temp1_3x1[2] = -0.5f*L1_PEL-_SBR.step_bundles[bundle_no].offsetL;
			else
				temp1_3x1[2] = -0.5f*L1_PEL-offsetL;
			temp1_3x1[3] = 0.;
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
			center_temp_3x1[1] = temp2_3x1[1] + pLF_last_3x1[1];
			center_temp_3x1[2] = temp2_3x1[2] + pLF_last_3x1[2];
			center_temp_3x1[3] = temp2_3x1[3] + pLF_last_3x1[3];

			temp1_3x1[1] = rot*(float)sin(dyaw);
			temp1_3x1[2] = rot*(1.f-(float)cos(dyaw));
			temp1_3x1[3] = 0.;
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
			center_temp_3x1[1] += temp2_3x1[1];
			center_temp_3x1[2] += temp2_3x1[2];
			center_temp_3x1[3] += temp2_3x1[3];
			RZ(yLF_last+dyaw, _TEMP1_34x34);

			temp1_3x1[1] = 0.;
			temp1_3x1[2] = 0.5f*L1_PEL;
			temp1_3x1[3] = 0.;
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
			_TwoStepBuff[buffer_no].pLF_3x1[1] = temp2_3x1[1] + center_temp_3x1[1];
			_TwoStepBuff[buffer_no].pLF_3x1[2] = temp2_3x1[2] + center_temp_3x1[2];
			_TwoStepBuff[buffer_no].pLF_3x1[3] = temp2_3x1[3] + center_temp_3x1[3];
			_TwoStepBuff[buffer_no].yLF = yLF_last + dyaw;

			CheckFootCollision(pRF_last_3x1, yRF_last, _TwoStepBuff[buffer_no].pLF_3x1, _TwoStepBuff[buffer_no].yLF, RFSP, &offsetL);
							
			_TwoStepBuff[buffer_no].supporting_foot = RFSP;
			_TwoStepBuff[buffer_no].pRF_3x1[1] = pRF_last_3x1[1];
			_TwoStepBuff[buffer_no].pRF_3x1[2] = pRF_last_3x1[2];
			_TwoStepBuff[buffer_no].pRF_3x1[3] = pRF_last_3x1[3];
			_TwoStepBuff[buffer_no].yRF = yRF_last;	

			halfFlag = -2;
		}
		else if(halfFlag == -2)
		{
			RZ(yRF_last, _TEMP1_34x34);
			temp1_3x1[1] = 0.;
			if(buffer_no == 0)
				temp1_3x1[2] = 0.5f*L1_PEL-_SBR.step_bundles[bundle_no].offsetR;
			else
				temp1_3x1[2] = 0.5f*L1_PEL-offsetR;
			temp1_3x1[3] = 0.;
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
			center_temp_3x1[1] = temp2_3x1[1] + pRF_last_3x1[1];
			center_temp_3x1[2] = temp2_3x1[2] + pRF_last_3x1[2];
			center_temp_3x1[3] = temp2_3x1[3] + pRF_last_3x1[3];

			temp1_3x1[1] = rot*(float)sin(dyaw);
			temp1_3x1[2] = rot*(1.f-(float)cos(dyaw));
			temp1_3x1[3] = 0.;
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
			center_temp_3x1[1] += temp2_3x1[1];
			center_temp_3x1[2] += temp2_3x1[2];
			center_temp_3x1[3] += temp2_3x1[3];
			RZ(yRF_last+dyaw, _TEMP1_34x34);

			temp1_3x1[1] = 0.;
			temp1_3x1[2] = -0.5f*L1_PEL;
			temp1_3x1[3] = 0.;
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);
			_TwoStepBuff[buffer_no].pRF_3x1[1] = temp2_3x1[1] + center_temp_3x1[1];
			_TwoStepBuff[buffer_no].pRF_3x1[2] = temp2_3x1[2] + center_temp_3x1[2];
			_TwoStepBuff[buffer_no].pRF_3x1[3] = temp2_3x1[3] + center_temp_3x1[3];
			_TwoStepBuff[buffer_no].yRF = yRF_last + dyaw;

			CheckFootCollision(_TwoStepBuff[buffer_no].pRF_3x1, _TwoStepBuff[buffer_no].yRF, pLF_last_3x1, yLF_last, LFSP, &offsetR);
							
			_TwoStepBuff[buffer_no].supporting_foot = LFSP;
			_TwoStepBuff[buffer_no].pLF_3x1[1] = pLF_last_3x1[1];
			_TwoStepBuff[buffer_no].pLF_3x1[2] = pLF_last_3x1[2];
			_TwoStepBuff[buffer_no].pLF_3x1[3] = pLF_last_3x1[3];
			_TwoStepBuff[buffer_no].yLF = yLF_last;	

			halfFlag = 2;
		}
		_TwoStepBuff[buffer_no].isEmpty = 0;
		if(buffer_no == 0)
		{
			_SBR.step_bundles[bundle_no].halfFlag = halfFlag;
			_SBR.step_bundles[bundle_no].offsetL = offsetL;
			_SBR.step_bundles[bundle_no].offsetR = offsetR;
		}
		break;

	case 5:   //--------------------------------------------- put the feet together
		if(_SBR.step_bundles[bundle_no].putTogether == RFSP)
		{
putTogetherR:
			RZ(yRF_last, _TEMP1_34x34);
			temp1_3x1[1] = 0.;
			temp1_3x1[2] = L1_PEL;
			temp1_3x1[3] = 0.;
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);

			_TwoStepBuff[buffer_no].pLF_3x1[1] = temp2_3x1[1] + pRF_last_3x1[1];
			_TwoStepBuff[buffer_no].pLF_3x1[2] = temp2_3x1[2] + pRF_last_3x1[2];
			_TwoStepBuff[buffer_no].pLF_3x1[3] = pRF_last_3x1[3];
			_TwoStepBuff[buffer_no].yLF = yRF_last;

			_TwoStepBuff[buffer_no].supporting_foot = RFSP;
			_TwoStepBuff[buffer_no].pRF_3x1[1] = pRF_last_3x1[1];
			_TwoStepBuff[buffer_no].pRF_3x1[2] = pRF_last_3x1[2];
			_TwoStepBuff[buffer_no].pRF_3x1[3] = pRF_last_3x1[3];
			_TwoStepBuff[buffer_no].yRF = yRF_last;	

			_TwoStepBuff[buffer_no].isEmpty = 0;
		}
		else if(_SBR.step_bundles[bundle_no].putTogether == LFSP)
		{
putTogetherL:
			RZ(yLF_last, _TEMP1_34x34);
			temp1_3x1[1] = 0.;
			temp1_3x1[2] = -L1_PEL;
			temp1_3x1[3] = 0.;
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, temp2_3x1);

			_TwoStepBuff[buffer_no].pRF_3x1[1] = temp2_3x1[1] + pLF_last_3x1[1];
			_TwoStepBuff[buffer_no].pRF_3x1[2] = temp2_3x1[2] + pLF_last_3x1[2];
			_TwoStepBuff[buffer_no].pRF_3x1[3] = pLF_last_3x1[3];
			_TwoStepBuff[buffer_no].yRF = yLF_last;

			_TwoStepBuff[buffer_no].supporting_foot = LFSP;
			_TwoStepBuff[buffer_no].pLF_3x1[1] = pLF_last_3x1[1];
			_TwoStepBuff[buffer_no].pLF_3x1[2] = pLF_last_3x1[2];
			_TwoStepBuff[buffer_no].pLF_3x1[3] = pLF_last_3x1[3];
			_TwoStepBuff[buffer_no].yLF = yLF_last;	

			_TwoStepBuff[buffer_no].isEmpty = 0;
		}
		else 
		{
			yaw_avg = 0.5f*(yRF_last+yLF_last);
			RZ(yaw_avg, _TEMP1_34x34);  //  center frame of the feet
			trans(1.f, (const float**)_TEMP1_34x34,3,3, _TEMP2_34x34);		
			center_temp_3x1[1] = 0.5f*(pRF_last_3x1[1]+pLF_last_3x1[1]);
			center_temp_3x1[2] = 0.5f*(pRF_last_3x1[2]+pLF_last_3x1[2]);
			center_temp_3x1[3] = 0.5f*(pRF_last_3x1[3]+pLF_last_3x1[3]);
			
			temp1_3x1[1] = pRF_last_3x1[1]-center_temp_3x1[1];
			temp1_3x1[2] = pRF_last_3x1[2]-center_temp_3x1[2];
			temp1_3x1[3] = pRF_last_3x1[3]-center_temp_3x1[3];		
			mult_mv((const float**)_TEMP2_34x34, 3,3,temp1_3x1, pRF_local_3x1);

			temp1_3x1[1] = pLF_last_3x1[1]-center_temp_3x1[1];
			temp1_3x1[2] = pLF_last_3x1[2]-center_temp_3x1[2];
			temp1_3x1[3] = pLF_last_3x1[3]-center_temp_3x1[3];		
			mult_mv((const float**)_TEMP2_34x34, 3,3,temp1_3x1, pLF_local_3x1);

			if(_SBR.step_bundles[bundle_no].putTogether == 2)		// align with the front foot
			{
				if(fabs(pRF_local_3x1[1]-pLF_local_3x1[1]) < EPS)  // 두 발 나란히
					goto putTogetherR;			
				else if(pRF_local_3x1[1] > pLF_local_3x1[1])  // 오른 발 전방
					goto putTogetherR;
				else
					goto putTogetherL;
			}
			else													// align with the rear foot
			{
				if(fabs(pRF_local_3x1[1]-pLF_local_3x1[1]) < EPS)  // 두 발 나란히
					goto putTogetherR;			
				else if(pRF_local_3x1[1] > pLF_local_3x1[1])  // 오른 발 전방
					goto putTogetherL;
				else
					goto putTogetherR;
			}		
		}
		break;
	}

	/*	
	switch(_TwoStepBuff[buffer_no].supporting_foot)
	{
	case DFSP:
		_TwoStepBuff[buffer_no].pRF_3x1[3] = 0.;
		_TwoStepBuff[buffer_no].pLF_3x1[3] = 0.;
		if(buffer_no == 0)
		{
			_elRFz = 0.;
			_elLFz = 0.;
		}
		break;
	case LFSP:
		_TwoStepBuff[buffer_no].pRF_3x1[3] = 0.;
		if(buffer_no == 0)
			_elRFz = 0.;		
		break;
	case RFSP:
		_TwoStepBuff[buffer_no].pLF_3x1[3] = 0.;
		if(buffer_no == 0)
			_elLFz = 0.;
		break;
	}
	*/
	return 0;
}


int CheckFootCollision(float *pRF_3x1, float yRF, float *pLF_3x1, float yLF, char supporting, float *offset_out)
{
	const float margin = 0.037f;
	const float margin2 = FOOT_SIZE + margin;

	const float pRHeel_local_3x1[4] = {0., -0.08f, 0.07f, 0.};
	const float pRToe_local_3x1[4] = {0., 0.14f, 0.07f, 0.};
	const float pLHeel_local_3x1[4] = {0., -0.08f, -0.07f, 0.};
	const float pLToe_local_3x1[4] = {0., 0.14f, -0.07f, 0.};

	float pRHeel_3x1[4], pRToe_3x1[4], pLHeel_3x1[4], pLToe_3x1[4];
	float temp1_3x1[4], temp2_3x1[4], f_temp;
	float pRH_LH_LF_3x1[4], pRT_LH_LF_3x1[4], pLH_RH_RF_3x1[4], pLT_RH_RF_3x1[4];

	float offset_LF = 0.;
	float offset_RF = 0.;

	RZ(yRF, _TEMP3_34x34);
	RZ(yLF, _TEMP4_34x34);
	
	mult_mv((const float**)_TEMP3_34x34,3,3, pRHeel_local_3x1, pRHeel_3x1);
	pRHeel_3x1[1] += pRF_3x1[1];
	pRHeel_3x1[2] += pRF_3x1[2];
	pRHeel_3x1[3] += pRF_3x1[3];

	mult_mv((const float**)_TEMP3_34x34,3,3, pRToe_local_3x1, pRToe_3x1);
	pRToe_3x1[1] += pRF_3x1[1];
	pRToe_3x1[2] += pRF_3x1[2];
	pRToe_3x1[3] += pRF_3x1[3];

	mult_mv((const float**)_TEMP4_34x34,3,3, pLHeel_local_3x1, pLHeel_3x1);
	pLHeel_3x1[1] += pLF_3x1[1];
	pLHeel_3x1[2] += pLF_3x1[2];
	pLHeel_3x1[3] += pLF_3x1[3];

	mult_mv((const float**)_TEMP4_34x34,3,3, pLToe_local_3x1, pLToe_3x1);
	pLToe_3x1[1] += pLF_3x1[1];
	pLToe_3x1[2] += pLF_3x1[2];
	pLToe_3x1[3] += pLF_3x1[3];

	trans2(1.f,_TEMP4_34x34,3,3);
	temp1_3x1[1] = pRHeel_3x1[1]-pLHeel_3x1[1];
	temp1_3x1[2] = pRHeel_3x1[2]-pLHeel_3x1[2];
	temp1_3x1[3] = pRHeel_3x1[3]-pLHeel_3x1[3];	
	mult_mv((const float**)_TEMP4_34x34,3,3, temp1_3x1, pRH_LH_LF_3x1);

	temp1_3x1[1] = pRToe_3x1[1]-pLHeel_3x1[1];
	temp1_3x1[2] = pRToe_3x1[2]-pLHeel_3x1[2];
	temp1_3x1[3] = pRToe_3x1[3]-pLHeel_3x1[3];
	mult_mv((const float**)_TEMP4_34x34,3,3, temp1_3x1, pRT_LH_LF_3x1);
	trans2(1.f,_TEMP4_34x34,3,3);

	trans2(1.f,_TEMP3_34x34,3,3);
	temp1_3x1[1] = pLHeel_3x1[1]-pRHeel_3x1[1];
	temp1_3x1[2] = pLHeel_3x1[2]-pRHeel_3x1[2];
	temp1_3x1[3] = pLHeel_3x1[3]-pRHeel_3x1[3];	
	mult_mv((const float**)_TEMP3_34x34,3,3, temp1_3x1, pLH_RH_RF_3x1);

	temp1_3x1[1] = pLToe_3x1[1]-pRHeel_3x1[1];
	temp1_3x1[2] = pLToe_3x1[2]-pRHeel_3x1[2];
	temp1_3x1[3] = pLToe_3x1[3]-pRHeel_3x1[3];
	mult_mv((const float**)_TEMP3_34x34,3,3, temp1_3x1, pLT_RH_RF_3x1);
	trans2(1.f,_TEMP3_34x34,3,3);

	
	if((pRH_LH_LF_3x1[1] > -margin) && (pRH_LH_LF_3x1[1] < margin2)) // LH에 대한 RH 의 상대 x 좌표가 LF 의 범위에 있을 때
	{
		if(pRH_LH_LF_3x1[2] > -margin)
		{
			if(supporting == RFSP)
				offset_LF = pRH_LH_LF_3x1[2] + margin;
			else
				offset_RF = -(pRH_LH_LF_3x1[2]+margin)/(float)cos(yRF-yLF);
		}

		if((pLT_RH_RF_3x1[1] > -margin) && (pLT_RH_RF_3x1[1] < margin2)) // RH에 대한 LT 의 상대 x 좌표가 RF 의 범위에 있을 때
		{
			if(pLT_RH_RF_3x1[2] < margin)
			{
				if(supporting == RFSP)
				{
					f_temp = -(pLT_RH_RF_3x1[2]-margin)/(float)cos(yLF-yRF);
					if(f_temp > offset_LF)
						offset_LF = f_temp;
				}
				else
				{
					f_temp = pLT_RH_RF_3x1[2] - margin;
					if(f_temp < offset_RF)
						offset_RF = f_temp;
				}

			}
		}
	}

	
	if((pLH_RH_RF_3x1[1] > -margin) && (pLH_RH_RF_3x1[1] < margin2)) // RH에 대한 LH 의 상대 x 좌표가 RF 의 범위에 있을 때
	{
		if(pLH_RH_RF_3x1[2] < margin)
		{
			if(supporting == RFSP)
			{
				f_temp = -(pLH_RH_RF_3x1[2]-margin)/(float)cos(yLF-yRF);
				if(f_temp > offset_LF)
					offset_LF = f_temp;
			}
			else
			{
				f_temp = pLH_RH_RF_3x1[2] - margin;
				if(f_temp < offset_RF)
					offset_RF = f_temp;
			}
		}

		if((pRT_LH_LF_3x1[1] > -margin) && (pRT_LH_LF_3x1[1] < margin2)) // LH에 대한 RT 의 상대 x 좌표가 LF 의 범위에 있을 때
		{
			if(pRT_LH_LF_3x1[2] > -margin)
			{
				if(supporting==RFSP)
				{
					f_temp = pRT_LH_LF_3x1[2] + margin;
					if(f_temp > offset_LF)
						offset_LF = f_temp;
				}
				else
				{
					f_temp = -(pRT_LH_LF_3x1[2]+margin)/(float)cos(yRF-yLF);
					if(f_temp < offset_RF)
						offset_RF = f_temp;
				}
			}
		}
	}

	if(supporting == RFSP)
	{
		*offset_out = offset_LF;
		temp1_3x1[1] = 0.;
		temp1_3x1[2] = *offset_out;
		temp1_3x1[3] = 0.;
		mult_mv((const float**)_TEMP4_34x34,3,3, temp1_3x1, temp2_3x1);
		pLF_3x1[1] += temp2_3x1[1];
		pLF_3x1[2] += temp2_3x1[2];
		pLF_3x1[3] += temp2_3x1[3];
	}
	else  // LFSP
	{
		*offset_out = offset_RF;
		temp1_3x1[1] = 0.;
		temp1_3x1[2] = *offset_out;
		temp1_3x1[3] = 0.;
		mult_mv((const float**)_TEMP3_34x34,3,3, temp1_3x1, temp2_3x1);
		pRF_3x1[1] += temp2_3x1[1];
		pRF_3x1[2] += temp2_3x1[2];
		pRF_3x1[3] += temp2_3x1[3];
	}

	return 0;
}


int WTG(int mode)
{
	static char isFirst = 1;
	static float pCOM_3x1[4], vCOM_3x1[4];
	float pRF_3x1[4], pLF_3x1[4], qRF_4x1[5], qLF_4x1[5];	
	float pRFC_3x1[4], pLFC_3x1[4];
	float temp1_3x1[4], temp2_3x1[4];
	float f_temp;
	float pRFC_next1_3x1[4], pLFC_next1_3x1[4], pRFC_next2_3x1[4], pLFC_next2_3x1[4];
	float yRF, yLF, dT_next2;
	float dRF_3x1[4], dLF_3x1[4], norm_dRF, norm_dLF;
	float pCOM_local_3x1[4], vCOM_local_3x1[4], pCOM_local2_3x1[4], vCOM_local2_3x1[4];
	float pLFC_local_3x1[4], pRFC_local_3x1[4];
	float zmp_sag_max, zmp_sag_min, zmp_fron_max, zmp_fron_min;
	float vCOM_sag_temp, vCOM_fron_temp;

	unsigned int i, ui_temp, length_1st_half;
	float t0, t1, t2, t3, t4;
	float dt11, dt21, dt12, dt22;
	
	if(_FKineUpdatedFlag != 1)
		return -4;

	pRF_3x1[1] = _pRF_3x1[1];
	pRF_3x1[2] = _pRF_3x1[2];
	pRF_3x1[3] = _pRF_3x1[3];

	pLF_3x1[1] = _pLF_3x1[1];
	pLF_3x1[2] = _pLF_3x1[2];
	pLF_3x1[3] = _pLF_3x1[3];

	qRF_4x1[1] = _qRF_4x1[1];
	qRF_4x1[2] = _qRF_4x1[2];
	qRF_4x1[3] = _qRF_4x1[3];
	qRF_4x1[4] = _qRF_4x1[4];

	qLF_4x1[1] = _qLF_4x1[1];
	qLF_4x1[2] = _qLF_4x1[2];
	qLF_4x1[3] = _qLF_4x1[3];
	qLF_4x1[4] = _qLF_4x1[4];
	
	if(isFirst == 1)
	{
		pCOM_3x1[1] = _pCOM_3x1[1];
		pCOM_3x1[2] = _pCOM_3x1[2];
		pCOM_3x1[3] = _pCOM_3x1[3];
		mult_mv((const float**)_jCOM_3x33,3,33,_Qp_33x1,vCOM_3x1);
		isFirst = 0;
	}
	else if(_TwoStepBuff[0].isEmpty == 1) // && _TwoStepBuff[1].isEmpty == 1)
		isFirst = 1;

	f_temp = (float)sqrt(qRF_4x1[1]*qRF_4x1[1] + qRF_4x1[4]+qRF_4x1[4]);
	yRF = (float)atan2(qRF_4x1[4]/f_temp, qRF_4x1[1]/f_temp)*2.f;

	f_temp = (float)sqrt(qLF_4x1[1]*qLF_4x1[1] + qLF_4x1[4]+qLF_4x1[4]);
	yLF = (float)atan2(qLF_4x1[4]/f_temp, qLF_4x1[1]/f_temp)*2.f;

	
	RZ(yRF,_TEMP1_34x34);
	pRFC_3x1[1] = pRF_3x1[1] + 0.03f*_TEMP1_34x34[1][1];
	pRFC_3x1[2] = pRF_3x1[2] + 0.03f*_TEMP1_34x34[2][1];
	pRFC_3x1[3] = pRF_3x1[3] + 0.03f*_TEMP1_34x34[3][1];

	RZ(yLF,_TEMP1_34x34);
	pLFC_3x1[1] = pLF_3x1[1] + 0.03f*_TEMP1_34x34[1][1];
	pLFC_3x1[2] = pLF_3x1[2] + 0.03f*_TEMP1_34x34[2][1];
	pLFC_3x1[3] = pLF_3x1[3] + 0.03f*_TEMP1_34x34[3][1];

	if(_TwoStepBuff[0].isEmpty == 1)
	{
		_TwoStepBuff[0].pRF_3x1[1] = pRF_3x1[1];
		_TwoStepBuff[0].pRF_3x1[2] = pRF_3x1[2];
		_TwoStepBuff[0].pRF_3x1[3] = 0.;
		_TwoStepBuff[0].yRF = yRF;

		_TwoStepBuff[0].pLF_3x1[1] = pLF_3x1[1];
		_TwoStepBuff[0].pLF_3x1[2] = pLF_3x1[2];
		_TwoStepBuff[0].pLF_3x1[3] = 0.;
		_TwoStepBuff[0].yLF = yLF;
		
		_TwoStepBuff[0].Tstep = dT_IDLE;
		_TwoStepBuff[0].supporting_foot = DFSP;
	}
	
	RZ(_TwoStepBuff[0].yRF, _TEMP1_34x34);
	pRFC_next1_3x1[1] = _TwoStepBuff[0].pRF_3x1[1] + 0.03f*_TEMP1_34x34[1][1];
	pRFC_next1_3x1[2] = _TwoStepBuff[0].pRF_3x1[2] + 0.03f*_TEMP1_34x34[2][1];
	pRFC_next1_3x1[3] = _TwoStepBuff[0].pRF_3x1[3] + 0.03f*_TEMP1_34x34[3][1];

	RZ(_TwoStepBuff[0].yLF, _TEMP1_34x34);
	pLFC_next1_3x1[1] = _TwoStepBuff[0].pLF_3x1[1] + 0.03f*_TEMP1_34x34[1][1];
	pLFC_next1_3x1[2] = _TwoStepBuff[0].pLF_3x1[2] + 0.03f*_TEMP1_34x34[2][1];
	pLFC_next1_3x1[3] = _TwoStepBuff[0].pLF_3x1[3] + 0.03f*_TEMP1_34x34[3][1];

	if(_TwoStepBuff[1].isEmpty == 1)
	{	
		pRFC_next2_3x1[1] = pRFC_next1_3x1[1];
		pRFC_next2_3x1[2] = pRFC_next1_3x1[2];
		pRFC_next2_3x1[3] = pRFC_next1_3x1[3];

		pLFC_next2_3x1[1] = pLFC_next1_3x1[1];
		pLFC_next2_3x1[2] = pLFC_next1_3x1[2];
		pLFC_next2_3x1[3] = pLFC_next1_3x1[3];

		dT_next2 = _TwoStepBuff[0].Tstep;
		_TwoStepBuff[1].supporting_foot = DFSP;
	}
	else
	{
		RZ(_TwoStepBuff[1].yRF,_TEMP1_34x34);
		pRFC_next2_3x1[1] = _TwoStepBuff[1].pRF_3x1[1] + 0.03f*_TEMP1_34x34[1][1];
		pRFC_next2_3x1[2] = _TwoStepBuff[1].pRF_3x1[2] + 0.03f*_TEMP1_34x34[2][1];
		pRFC_next2_3x1[3] = _TwoStepBuff[1].pRF_3x1[3] + 0.03f*_TEMP1_34x34[3][1];

		RZ(_TwoStepBuff[1].yLF,_TEMP1_34x34);
		pLFC_next2_3x1[1] = _TwoStepBuff[1].pLF_3x1[1] + 0.03f*_TEMP1_34x34[1][1];
		pLFC_next2_3x1[2] = _TwoStepBuff[1].pLF_3x1[2] + 0.03f*_TEMP1_34x34[2][1];
		pLFC_next2_3x1[3] = _TwoStepBuff[1].pLF_3x1[3] + 0.03f*_TEMP1_34x34[3][1];

		dT_next2 = _TwoStepBuff[1].Tstep;
	}
	
	dRF_3x1[1] = pRFC_next1_3x1[1] - pRFC_3x1[1];
	dRF_3x1[2] = pRFC_next1_3x1[2] - pRFC_3x1[2];
	dRF_3x1[3] = pRFC_next1_3x1[3] - pRFC_3x1[3];
//	norm_dRF = (float)sqrt(dRF_3x1[1]*dRF_3x1[1]+dRF_3x1[2]*dRF_3x1[2]+dRF_3x1[3]*dRF_3x1[3]);
	norm_dRF = (float)sqrt(dRF_3x1[1]*dRF_3x1[1]+dRF_3x1[2]*dRF_3x1[2]);	// for the Early Landing Strategy

	dLF_3x1[1] = pLFC_next1_3x1[1] - pLFC_3x1[1];
	dLF_3x1[2] = pLFC_next1_3x1[2] - pLFC_3x1[2];
	dLF_3x1[3] = pLFC_next1_3x1[3] - pLFC_3x1[3];
//	norm_dLF = (float)sqrt(dLF_3x1[1]*dLF_3x1[1]+dLF_3x1[2]*dLF_3x1[2]+dLF_3x1[3]*dLF_3x1[3]);
	norm_dLF = (float)sqrt(dLF_3x1[1]*dLF_3x1[1]+dLF_3x1[2]*dLF_3x1[2]);	// for the Early Landing Strategy

	t0 = 0.;
	t1 = T_DFSP*0.5f;
	t2 = t1 + (T_DFSP+_TwoStepBuff[0].Tstep)*0.5f;
	t3 = t2 + (T_DFSP+_TwoStepBuff[0].Tstep)*0.5f;
	t4 = t3 + (T_DFSP+dT_next2)*0.5f;
	
	//------------------------------------------------------------------- 1st half step
	if((norm_dRF > MOVING_FOOT_CUTOFF) && (norm_dLF < MOVING_FOOT_CUTOFF))  // moving the right foot, LFSP
	{
		RZ(yLF, _TEMP1_34x34);
		trans2(1.f, _TEMP1_34x34,3,3);
		mult_mv((const float**)_TEMP1_34x34,3,3, pCOM_3x1, pCOM_local_3x1);
		mult_mv((const float**)_TEMP1_34x34,3,3, vCOM_3x1, vCOM_local_3x1);
		mult_mv((const float**)_TEMP1_34x34,3,3, pRFC_3x1, pRFC_local_3x1);
		mult_mv((const float**)_TEMP1_34x34,3,3, pLFC_3x1, pLFC_local_3x1);
		temp1_3x1[1] = 0.5f*dRF_3x1[1]/(t3-t1);
		temp1_3x1[2] = 0.5f*dRF_3x1[2]/(t3-t1);
		temp1_3x1[3] = 0.5f*dRF_3x1[3]/(t3-t1);
		mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, vCOM_local2_3x1);
		temp1_3x1[1] = 0.5f*(pRFC_3x1[1]+pLFC_3x1[1]+0.5f*dRF_3x1[1]);
		temp1_3x1[2] = 0.5f*(pRFC_3x1[2]+pLFC_3x1[2]+0.5f*dRF_3x1[2]);
		temp1_3x1[3] = 0.5f*(pRFC_3x1[3]+pLFC_3x1[3]+0.5f*dRF_3x1[3]);
		mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, pCOM_local2_3x1);

		if(pLFC_local_3x1[1] > pRFC_local_3x1[1])
		{
			zmp_sag_max = pLFC_local_3x1[1] + ZMP_SAG_MARGIN;
			zmp_sag_min = pRFC_local_3x1[1] - ZMP_SAG_MARGIN;
		}
		else
		{
			zmp_sag_max = pRFC_local_3x1[1] + ZMP_SAG_MARGIN;
			zmp_sag_min = pLFC_local_3x1[1] - ZMP_SAG_MARGIN;
		}

		zmp_fron_max = pLFC_local_3x1[2] + ZMP_FRON_MARGIN;
		zmp_fron_min = pRFC_local_3x1[2] - ZMP_FRON_MARGIN;

		ComTZ(t0, t1, t2, pCOM_local_3x1[1], vCOM_local_3x1[1], pCOM_local2_3x1[1], vCOM_local2_3x1[1], pCOM_local_3x1[3], zmp_sag_max, zmp_sag_min, 2, _WalkTraj.pCOM_3x1,1, &vCOM_sag_temp, &length_1st_half, &dt11, &dt21);	// mode 2
		ComTZ(t0, t1, t2, pCOM_local_3x1[2], vCOM_local_3x1[2], pLFC_local_3x1[2]-pSharedMemory->WB_ZMP_OFFSET, 0., pCOM_local_3x1[3], zmp_fron_max, zmp_fron_min, mode, _WalkTraj.pCOM_3x1,2, &vCOM_fron_temp, &length_1st_half, &dt12, &dt22); 

		if(fabs(FMAX(dt11,dt12)) > EPS || fabs(FMAX(dt21,dt22)) > EPS)
		{
			t1 = t1 + (float)FMAX(dt11,dt12);
			t2 = t2 + (float)FMAX(dt21,dt22);
			t3 = t2 + t2 - t1;
			t4 = t3 + 0.5f*(T_DFSP+dT_next2);
			
			temp1_3x1[1] = 0.5f*dRF_3x1[1]/(t3-t1);
			temp1_3x1[2] = 0.5f*dRF_3x1[2]/(t3-t1);
			temp1_3x1[3] = 0.5f*dRF_3x1[3]/(t3-t1);
			RZ(yLF, _TEMP1_34x34);
			trans2(1.f, _TEMP1_34x34,3,3);
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, vCOM_local2_3x1);

			ComTZ(t0, t1, t2, pCOM_local_3x1[1], vCOM_local_3x1[1], pCOM_local2_3x1[1], vCOM_local2_3x1[1], pCOM_local_3x1[3], zmp_sag_max, zmp_sag_min, 2, _WalkTraj.pCOM_3x1,1, &vCOM_sag_temp, &length_1st_half, &dt11, &dt21);	// mode 2
			ComTZ(t0, t1, t2, pCOM_local_3x1[2], vCOM_local_3x1[2], pLFC_local_3x1[2]-pSharedMemory->WB_ZMP_OFFSET, 0., pCOM_local_3x1[3], zmp_fron_max, zmp_fron_min, mode, _WalkTraj.pCOM_3x1,2, &vCOM_fron_temp, &length_1st_half, &dt12, &dt22); 
		}
		for(i=1; i<= length_1st_half; i++)
			_WalkTraj.pCOM_3x1[i][3] = pCOM_local_3x1[3];	
		_WalkTraj.length = length_1st_half;
		
		RZ(yLF, _TEMP1_34x34);
		for(i=1; i<=_WalkTraj.length; i++)
		{			
			mult_mv((const float**)_TEMP1_34x34,3,3, _WalkTraj.pCOM_3x1[i], temp1_3x1);
			_WalkTraj.pCOM_3x1[i][1] = temp1_3x1[1];
			_WalkTraj.pCOM_3x1[i][2] = temp1_3x1[2];
			_WalkTraj.pCOM_3x1[i][3] = temp1_3x1[3];
		}
		temp2_3x1[1] = vCOM_sag_temp;
		temp2_3x1[2] = vCOM_fron_temp;
		temp2_3x1[3] = 0.;
		mult_mv((const float**)_TEMP1_34x34,3,3, temp2_3x1, temp1_3x1);
		vCOM_3x1[1] = temp1_3x1[1];
		vCOM_3x1[2] = temp1_3x1[2];
	}
	else if((norm_dRF < MOVING_FOOT_CUTOFF) && (norm_dLF > MOVING_FOOT_CUTOFF))  // moving the left foot, RFSP
	{
		RZ(yRF, _TEMP1_34x34);
		trans2(1.f, _TEMP1_34x34,3,3);
		mult_mv((const float**)_TEMP1_34x34,3,3, pCOM_3x1, pCOM_local_3x1);
		mult_mv((const float**)_TEMP1_34x34,3,3, vCOM_3x1, vCOM_local_3x1);
		mult_mv((const float**)_TEMP1_34x34,3,3, pRFC_3x1, pRFC_local_3x1);
		mult_mv((const float**)_TEMP1_34x34,3,3, pLFC_3x1, pLFC_local_3x1);
		temp1_3x1[1] = 0.5f*dLF_3x1[1]/(t3-t1);
		temp1_3x1[2] = 0.5f*dLF_3x1[2]/(t3-t1);
		temp1_3x1[3] = 0.5f*dLF_3x1[3]/(t3-t1);
		mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, vCOM_local2_3x1);
		temp1_3x1[1] = 0.5f*(pRFC_3x1[1]+pLFC_3x1[1]+0.5f*dLF_3x1[1]);
		temp1_3x1[2] = 0.5f*(pRFC_3x1[2]+pLFC_3x1[2]+0.5f*dLF_3x1[2]);
		temp1_3x1[3] = 0.5f*(pRFC_3x1[3]+pLFC_3x1[3]+0.5f*dLF_3x1[3]);
		mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, pCOM_local2_3x1);

		if(pLFC_local_3x1[1] > pRFC_local_3x1[1])
		{
			zmp_sag_max = pLFC_local_3x1[1] + ZMP_SAG_MARGIN;
			zmp_sag_min = pRFC_local_3x1[1] - ZMP_SAG_MARGIN;
		}
		else
		{
			zmp_sag_max = pRFC_local_3x1[1] + ZMP_SAG_MARGIN;
			zmp_sag_min = pLFC_local_3x1[1] - ZMP_SAG_MARGIN;
		}

		zmp_fron_max = pLFC_local_3x1[2] + ZMP_FRON_MARGIN;
		zmp_fron_min = pRFC_local_3x1[2] - ZMP_FRON_MARGIN;

		ComTZ(t0, t1, t2, pCOM_local_3x1[1], vCOM_local_3x1[1], pCOM_local2_3x1[1], vCOM_local2_3x1[1], pCOM_local_3x1[3], zmp_sag_max, zmp_sag_min, 2, _WalkTraj.pCOM_3x1,1, &vCOM_sag_temp, &length_1st_half, &dt11, &dt21);	// mode 2
		ComTZ(t0, t1, t2, pCOM_local_3x1[2], vCOM_local_3x1[2], pRFC_local_3x1[2]+pSharedMemory->WB_ZMP_OFFSET, 0., pCOM_local_3x1[3], zmp_fron_max, zmp_fron_min, mode, _WalkTraj.pCOM_3x1,2, &vCOM_fron_temp, &length_1st_half, &dt12, &dt22); 
		
		if(fabs(FMAX(dt11,dt12)) > EPS || fabs(FMAX(dt21,dt22)) > EPS)
		{
			t1 = t1 + (float)FMAX(dt11,dt12);
			t2 = t2 + (float)FMAX(dt21,dt22);
			t3 = t2 + t2 - t1;
			t4 = t3 + 0.5f*(T_DFSP+dT_next2);
			
			temp1_3x1[1] = 0.5f*dLF_3x1[1]/(t3-t1);
			temp1_3x1[2] = 0.5f*dLF_3x1[2]/(t3-t1);
			temp1_3x1[3] = 0.5f*dLF_3x1[3]/(t3-t1);
			RZ(yRF, _TEMP1_34x34);
			trans2(1.f, _TEMP1_34x34,3,3);
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, vCOM_local2_3x1);

			ComTZ(t0, t1, t2, pCOM_local_3x1[1], vCOM_local_3x1[1], pCOM_local2_3x1[1], vCOM_local2_3x1[1], pCOM_local_3x1[3], zmp_sag_max, zmp_sag_min, 2, _WalkTraj.pCOM_3x1,1, &vCOM_sag_temp, &length_1st_half, &dt11, &dt21);	// mode 2
			ComTZ(t0, t1, t2, pCOM_local_3x1[2], vCOM_local_3x1[2], pRFC_local_3x1[2]+pSharedMemory->WB_ZMP_OFFSET, 0., pCOM_local_3x1[3], zmp_fron_max, zmp_fron_min, mode, _WalkTraj.pCOM_3x1,2, &vCOM_fron_temp, &length_1st_half, &dt12, &dt22); 
		}
		for(i=1; i<= length_1st_half; i++)
			_WalkTraj.pCOM_3x1[i][3] = pCOM_local_3x1[3];	
		_WalkTraj.length = length_1st_half;
		
		RZ(yRF, _TEMP1_34x34);
		for(i=1; i<=_WalkTraj.length; i++)
		{			
			mult_mv((const float**)_TEMP1_34x34,3,3, _WalkTraj.pCOM_3x1[i], temp1_3x1);
			_WalkTraj.pCOM_3x1[i][1] = temp1_3x1[1];
			_WalkTraj.pCOM_3x1[i][2] = temp1_3x1[2];
			_WalkTraj.pCOM_3x1[i][3] = temp1_3x1[3];
		}
		temp2_3x1[1] = vCOM_sag_temp;
		temp2_3x1[2] = vCOM_fron_temp;
		temp2_3x1[3] = 0.;
		mult_mv((const float**)_TEMP1_34x34,3,3, temp2_3x1, temp1_3x1);
		vCOM_3x1[1] = temp1_3x1[1];
		vCOM_3x1[2] = temp1_3x1[2];
	}
	else if((norm_dRF < MOVING_FOOT_CUTOFF) && (norm_dLF < MOVING_FOOT_CUTOFF))  // staying on current step, DFSP
	{
		RZ(yRF, _TEMP1_34x34);
		trans2(1.f, _TEMP1_34x34,3,3);
		mult_mv((const float**)_TEMP1_34x34,3,3, pCOM_3x1, pCOM_local_3x1);
		mult_mv((const float**)_TEMP1_34x34,3,3, vCOM_3x1, vCOM_local_3x1);
		mult_mv((const float**)_TEMP1_34x34,3,3, pRFC_3x1, pRFC_local_3x1);
		mult_mv((const float**)_TEMP1_34x34,3,3, pLFC_3x1, pLFC_local_3x1);
		

		if(pLFC_local_3x1[1] > pRFC_local_3x1[1])
		{
			zmp_sag_max = pLFC_local_3x1[1] + ZMP_SAG_MARGIN;
			zmp_sag_min = pRFC_local_3x1[1] - ZMP_SAG_MARGIN;
		}
		else
		{
			zmp_sag_max = pRFC_local_3x1[1] + ZMP_SAG_MARGIN;
			zmp_sag_min = pLFC_local_3x1[1] - ZMP_SAG_MARGIN;
		}

		zmp_fron_max = pLFC_local_3x1[2] + ZMP_FRON_MARGIN;
		zmp_fron_min = pRFC_local_3x1[2] - ZMP_FRON_MARGIN;

		ComTZ(t0, t1, t2, pCOM_local_3x1[1], vCOM_local_3x1[1], 0.5f*(pLFC_local_3x1[1]+pRFC_local_3x1[1]), 0., pCOM_local_3x1[3], zmp_sag_max, zmp_sag_min, 2, _WalkTraj.pCOM_3x1,1, &vCOM_sag_temp, &length_1st_half, &dt11, &dt21);	// mode 2
		ComTZ(t0, t1, t2, pCOM_local_3x1[2], vCOM_local_3x1[2], 0.5f*(pLFC_local_3x1[2]+pRFC_local_3x1[2]), 0., pCOM_local_3x1[3], zmp_fron_max, zmp_fron_min, 2, _WalkTraj.pCOM_3x1,2, &vCOM_fron_temp, &length_1st_half, &dt12, &dt22);	// mode 2		

		if(fabs(FMAX(dt11,dt12)) > EPS || fabs(FMAX(dt21,dt22)) > EPS)
		{
			t1 = t1 + (float)FMAX(dt11,dt12);
			t2 = t2 + (float)FMAX(dt21,dt22);
			t3 = t2 + t2 - t1;
			t4 = t3 + 0.5f*(T_DFSP+dT_next2);
			
			
			ComTZ(t0, t1, t2, pCOM_local_3x1[1], vCOM_local_3x1[1], 0.5f*(pLFC_local_3x1[1]+pRFC_local_3x1[1]), 0., pCOM_local_3x1[3], zmp_sag_max, zmp_sag_min, 2, _WalkTraj.pCOM_3x1,1, &vCOM_sag_temp, &length_1st_half, &dt11, &dt21);	// mode 2
			ComTZ(t0, t1, t2, pCOM_local_3x1[2], vCOM_local_3x1[2], 0.5f*(pLFC_local_3x1[2]+pRFC_local_3x1[2]), 0., pCOM_local_3x1[3], zmp_fron_max, zmp_fron_min, 2, _WalkTraj.pCOM_3x1,2, &vCOM_fron_temp, &length_1st_half, &dt12, &dt22);	// mode 2		
		}
		for(i=1; i<= length_1st_half; i++)
			_WalkTraj.pCOM_3x1[i][3] = pCOM_local_3x1[3];	
		_WalkTraj.length = length_1st_half;
		
		RZ(yRF, _TEMP1_34x34);
		for(i=1; i<=_WalkTraj.length; i++)
		{			
			mult_mv((const float**)_TEMP1_34x34,3,3, _WalkTraj.pCOM_3x1[i], temp1_3x1);
			_WalkTraj.pCOM_3x1[i][1] = temp1_3x1[1];
			_WalkTraj.pCOM_3x1[i][2] = temp1_3x1[2];
			_WalkTraj.pCOM_3x1[i][3] = temp1_3x1[3];
		}
		temp2_3x1[1] = vCOM_sag_temp;
		temp2_3x1[2] = vCOM_fron_temp;
		temp2_3x1[3] = 0.;
		mult_mv((const float**)_TEMP1_34x34,3,3, temp2_3x1, temp1_3x1);
		vCOM_3x1[1] = temp1_3x1[1];
		vCOM_3x1[2] = temp1_3x1[2];
	}
	else  // erroneous case
	{
		printf("\nWTG error!!\n");
		return (-1);
	}


	//------------------------------------------------------------------- 2nd half step

	dRF_3x1[1] = pRFC_next2_3x1[1] - pRFC_next1_3x1[1];
	dRF_3x1[2] = pRFC_next2_3x1[2] - pRFC_next1_3x1[2];
	dRF_3x1[3] = pRFC_next2_3x1[3] - pRFC_next1_3x1[3];
	//norm_dRF = (float)sqrt(dRF_3x1[1]*dRF_3x1[1]+dRF_3x1[2]*dRF_3x1[2]+dRF_3x1[3]*dRF_3x1[3]);
	norm_dRF = (float)sqrt(dRF_3x1[1]*dRF_3x1[1]+dRF_3x1[2]*dRF_3x1[2]);  // for the Early Landing Strategy

	dLF_3x1[1] = pLFC_next2_3x1[1] - pLFC_next1_3x1[1];
	dLF_3x1[2] = pLFC_next2_3x1[2] - pLFC_next1_3x1[2];
	dLF_3x1[3] = pLFC_next2_3x1[3] - pLFC_next1_3x1[3];
	//norm_dLF = (float)sqrt(dLF_3x1[1]*dLF_3x1[1]+dLF_3x1[2]*dLF_3x1[2]+dLF_3x1[3]*dLF_3x1[3]);
	norm_dLF = (float)sqrt(dLF_3x1[1]*dLF_3x1[1]+dLF_3x1[2]*dLF_3x1[2]);  // for the Early Landing Strategy

	if(norm_dRF>MOVING_FOOT_CUTOFF && norm_dLF<MOVING_FOOT_CUTOFF)		// move the right foot, LFSP   
	{
		RZ(_TwoStepBuff[0].yLF, _TEMP1_34x34);
		trans2(1.f, _TEMP1_34x34,3,3);
		mult_mv((const float**)_TEMP1_34x34,3,3, _WalkTraj.pCOM_3x1[_WalkTraj.length], pCOM_local_3x1);
		temp1_3x1[1] = vCOM_3x1[1];
		temp1_3x1[2] = vCOM_3x1[2];
		temp1_3x1[3] = 0.;
		mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, vCOM_local_3x1);
		mult_mv((const float**)_TEMP1_34x34,3,3, pRFC_next1_3x1, pRFC_local_3x1);
		mult_mv((const float**)_TEMP1_34x34,3,3, pLFC_next1_3x1, pLFC_local_3x1);
		temp1_3x1[1] = 0.5f*dRF_3x1[1]/(2.f*(t4-t3));
		temp1_3x1[2] = 0.5f*dRF_3x1[2]/(2.f*(t4-t3));
		temp1_3x1[3] = 0.5f*dRF_3x1[3]/(2.f*(t4-t3));
		mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, vCOM_local2_3x1);
		temp1_3x1[1] = 0.5f*(pRFC_next1_3x1[1]+pLFC_next1_3x1[1]+0.5f*dRF_3x1[1]);
		temp1_3x1[2] = 0.5f*(pRFC_next1_3x1[2]+pLFC_next1_3x1[2]+0.5f*dRF_3x1[2]);
		temp1_3x1[3] = 0.5f*(pRFC_next1_3x1[3]+pLFC_next1_3x1[3]+0.5f*dRF_3x1[3]);
		mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, pCOM_local2_3x1);

		if(pLFC_local_3x1[1] > pRFC_local_3x1[1])
		{
			zmp_sag_max = pLFC_local_3x1[1] + ZMP_SAG_MARGIN;
			zmp_sag_min = pRFC_local_3x1[1] - ZMP_SAG_MARGIN;
		}
		else
		{
			zmp_sag_max = pRFC_local_3x1[1] + ZMP_SAG_MARGIN;
			zmp_sag_min = pLFC_local_3x1[1] - ZMP_SAG_MARGIN;
		}

		zmp_fron_max = pLFC_local_3x1[2] + ZMP_FRON_MARGIN;
		zmp_fron_min = pRFC_local_3x1[2] - ZMP_FRON_MARGIN;

		ComTZ(t2, t3, t4, pCOM_local_3x1[1], vCOM_local_3x1[1], pCOM_local2_3x1[1], vCOM_local2_3x1[1], pCOM_local_3x1[3], zmp_sag_max, zmp_sag_min, 2, &_WalkTraj.pCOM_3x1[length_1st_half],1, &vCOM_sag_temp, &ui_temp, &dt11, &dt21);	// mode 2
		ComTZ(t2, t3, t4, pCOM_local_3x1[2], vCOM_local_3x1[2], pLFC_local_3x1[2]-pSharedMemory->WB_ZMP_OFFSET, 0., pCOM_local_3x1[3], zmp_fron_max, zmp_fron_min, mode, &_WalkTraj.pCOM_3x1[length_1st_half],2, &vCOM_fron_temp, &ui_temp, &dt12, &dt22); 
		
		if(fabs(FMAX(dt11,dt12)) > EPS || fabs(FMAX(dt21,dt22)) > EPS)
		{
			t3 = t3 + (float)FMAX(dt11,dt12);
			t4 = t4 + (float)FMAX(dt21,dt22);
			
			temp1_3x1[1] = 0.5f*dRF_3x1[1]/(2.f*(t4-t3));
			temp1_3x1[2] = 0.5f*dRF_3x1[2]/(2.f*(t4-t3));
			temp1_3x1[3] = 0.5f*dRF_3x1[3]/(2.f*(t4-t3));
			RZ(_TwoStepBuff[0].yLF, _TEMP1_34x34);
			trans2(1.f, _TEMP1_34x34,3,3);
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, vCOM_local2_3x1);

			ComTZ(t2, t3, t4, pCOM_local_3x1[1], vCOM_local_3x1[1], pCOM_local2_3x1[1], vCOM_local2_3x1[1], pCOM_local_3x1[3], zmp_sag_max, zmp_sag_min, 2, &_WalkTraj.pCOM_3x1[length_1st_half],1, &vCOM_sag_temp, &ui_temp, &dt11, &dt21);	// mode 2
			ComTZ(t2, t3, t4, pCOM_local_3x1[2], vCOM_local_3x1[2], pLFC_local_3x1[2]-pSharedMemory->WB_ZMP_OFFSET, 0., pCOM_local_3x1[3], zmp_fron_max, zmp_fron_min, mode, &_WalkTraj.pCOM_3x1[length_1st_half],2, &vCOM_fron_temp, &ui_temp, &dt12, &dt22); 
		}
		ui_temp = (unsigned int)my_round((t3-t2-T_DFSP*0.5f)/DT);
		for(i=1; i<= ui_temp; i++)
			_WalkTraj.pCOM_3x1[i+length_1st_half][3] = pCOM_local_3x1[3];
		_WalkTraj.length = length_1st_half + ui_temp;
		
		RZ(_TwoStepBuff[0].yLF, _TEMP1_34x34);
		for(i=1+length_1st_half; i<=_WalkTraj.length; i++)
		{			
			mult_mv((const float**)_TEMP1_34x34,3,3, _WalkTraj.pCOM_3x1[i], temp1_3x1);
			_WalkTraj.pCOM_3x1[i][1] = temp1_3x1[1];
			_WalkTraj.pCOM_3x1[i][2] = temp1_3x1[2];
			_WalkTraj.pCOM_3x1[i][3] = temp1_3x1[3];
		}
		temp2_3x1[1] = vCOM_sag_temp;
		temp2_3x1[2] = vCOM_fron_temp;
		temp2_3x1[3] = 0.;
		mult_mv((const float**)_TEMP1_34x34,3,3, temp2_3x1, temp1_3x1);
		vCOM_3x1[1] = temp1_3x1[1];
		vCOM_3x1[2] = temp1_3x1[2];		
	}
	else if(norm_dRF<MOVING_FOOT_CUTOFF && norm_dLF>MOVING_FOOT_CUTOFF)   // move the left foot, RFSP
	{
		RZ(_TwoStepBuff[0].yRF, _TEMP1_34x34);
		trans2(1.f, _TEMP1_34x34,3,3);
		mult_mv((const float**)_TEMP1_34x34,3,3, _WalkTraj.pCOM_3x1[_WalkTraj.length], pCOM_local_3x1);
		temp1_3x1[1] = vCOM_3x1[1];
		temp1_3x1[2] = vCOM_3x1[2];
		temp1_3x1[3] = 0.;
		mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, vCOM_local_3x1);
		mult_mv((const float**)_TEMP1_34x34,3,3, pRFC_next1_3x1, pRFC_local_3x1);
		mult_mv((const float**)_TEMP1_34x34,3,3, pLFC_next1_3x1, pLFC_local_3x1);
		temp1_3x1[1] = 0.5f*dLF_3x1[1]/(2.f*(t4-t3));
		temp1_3x1[2] = 0.5f*dLF_3x1[2]/(2.f*(t4-t3));
		temp1_3x1[3] = 0.5f*dLF_3x1[3]/(2.f*(t4-t3));
		mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, vCOM_local2_3x1);
		temp1_3x1[1] = 0.5f*(pRFC_next1_3x1[1]+pLFC_next1_3x1[1]+0.5f*dLF_3x1[1]);
		temp1_3x1[2] = 0.5f*(pRFC_next1_3x1[2]+pLFC_next1_3x1[2]+0.5f*dLF_3x1[2]);
		temp1_3x1[3] = 0.5f*(pRFC_next1_3x1[3]+pLFC_next1_3x1[3]+0.5f*dLF_3x1[3]);
		mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, pCOM_local2_3x1);

		if(pLFC_local_3x1[1] > pRFC_local_3x1[1])
		{
			zmp_sag_max = pLFC_local_3x1[1] + ZMP_SAG_MARGIN;
			zmp_sag_min = pRFC_local_3x1[1] - ZMP_SAG_MARGIN;
		}
		else
		{
			zmp_sag_max = pRFC_local_3x1[1] + ZMP_SAG_MARGIN;
			zmp_sag_min = pLFC_local_3x1[1] - ZMP_SAG_MARGIN;
		}

		zmp_fron_max = pLFC_local_3x1[2] + ZMP_FRON_MARGIN;
		zmp_fron_min = pRFC_local_3x1[2] - ZMP_FRON_MARGIN;

		ComTZ(t2, t3, t4, pCOM_local_3x1[1], vCOM_local_3x1[1], pCOM_local2_3x1[1], vCOM_local2_3x1[1], pCOM_local_3x1[3], zmp_sag_max, zmp_sag_min, 2, &_WalkTraj.pCOM_3x1[length_1st_half],1, &vCOM_sag_temp, &ui_temp, &dt11, &dt21);	// mode 2
		ComTZ(t2, t3, t4, pCOM_local_3x1[2], vCOM_local_3x1[2], pRFC_local_3x1[2]+pSharedMemory->WB_ZMP_OFFSET, 0., pCOM_local_3x1[3], zmp_fron_max, zmp_fron_min, mode, &_WalkTraj.pCOM_3x1[length_1st_half],2, &vCOM_fron_temp, &ui_temp, &dt12, &dt22); 
		
		if(fabs(FMAX(dt11,dt12)) > EPS || fabs(FMAX(dt21,dt22)) > EPS)
		{
			t3 = t3 + (float)FMAX(dt11,dt12);
			t4 = t4 + (float)FMAX(dt21,dt22);
			
			temp1_3x1[1] = 0.5f*dLF_3x1[1]/(2.f*(t4-t3));
			temp1_3x1[2] = 0.5f*dLF_3x1[2]/(2.f*(t4-t3));
			temp1_3x1[3] = 0.5f*dLF_3x1[3]/(2.f*(t4-t3));
			RZ(_TwoStepBuff[0].yRF, _TEMP1_34x34);
			trans2(1.f, _TEMP1_34x34,3,3);
			mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, vCOM_local2_3x1);

			ComTZ(t2, t3, t4, pCOM_local_3x1[1], vCOM_local_3x1[1], pCOM_local2_3x1[1], vCOM_local2_3x1[1], pCOM_local_3x1[3], zmp_sag_max, zmp_sag_min, 2, &_WalkTraj.pCOM_3x1[length_1st_half],1, &vCOM_sag_temp, &ui_temp, &dt11, &dt21);	// mode 2
			ComTZ(t2, t3, t4, pCOM_local_3x1[2], vCOM_local_3x1[2], pRFC_local_3x1[2]+pSharedMemory->WB_ZMP_OFFSET, 0., pCOM_local_3x1[3], zmp_fron_max, zmp_fron_min, mode, &_WalkTraj.pCOM_3x1[length_1st_half],2, &vCOM_fron_temp, &ui_temp, &dt12, &dt22); 
		}
		ui_temp = (unsigned int)my_round((t3-t2-T_DFSP*0.5f)/DT);
		for(i=1; i<= ui_temp; i++)
			_WalkTraj.pCOM_3x1[i+length_1st_half][3] = pCOM_local_3x1[3];
		_WalkTraj.length = length_1st_half + ui_temp;
		
		RZ(_TwoStepBuff[0].yRF, _TEMP1_34x34);
		for(i=1+length_1st_half; i<=_WalkTraj.length; i++)
		{			
			mult_mv((const float**)_TEMP1_34x34,3,3, _WalkTraj.pCOM_3x1[i], temp1_3x1);
			_WalkTraj.pCOM_3x1[i][1] = temp1_3x1[1];
			_WalkTraj.pCOM_3x1[i][2] = temp1_3x1[2];
			_WalkTraj.pCOM_3x1[i][3] = temp1_3x1[3];
		}
		temp2_3x1[1] = vCOM_sag_temp;
		temp2_3x1[2] = vCOM_fron_temp;
		temp2_3x1[3] = 0.;
		mult_mv((const float**)_TEMP1_34x34,3,3, temp2_3x1, temp1_3x1);
		vCOM_3x1[1] = temp1_3x1[1];
		vCOM_3x1[2] = temp1_3x1[2];
	}
	else if(norm_dRF<MOVING_FOOT_CUTOFF && norm_dLF<MOVING_FOOT_CUTOFF) // stay on current step, DFSP
	{
		RZ(_TwoStepBuff[0].yRF, _TEMP1_34x34);
		trans2(1.f, _TEMP1_34x34,3,3);
		mult_mv((const float**)_TEMP1_34x34,3,3, _WalkTraj.pCOM_3x1[_WalkTraj.length], pCOM_local_3x1);
		temp1_3x1[1] = vCOM_3x1[1];
		temp1_3x1[2] = vCOM_3x1[2];
		temp1_3x1[3] = 0.;
		mult_mv((const float**)_TEMP1_34x34,3,3, temp1_3x1, vCOM_local_3x1);
		mult_mv((const float**)_TEMP1_34x34,3,3, pRFC_next1_3x1, pRFC_local_3x1);
		mult_mv((const float**)_TEMP1_34x34,3,3, pLFC_next1_3x1, pLFC_local_3x1);

		if(pLFC_local_3x1[1] > pRFC_local_3x1[1])
		{
			zmp_sag_max = pLFC_local_3x1[1] + ZMP_SAG_MARGIN;
			zmp_sag_min = pRFC_local_3x1[1] - ZMP_SAG_MARGIN;
		}
		else
		{
			zmp_sag_max = pRFC_local_3x1[1] + ZMP_SAG_MARGIN;
			zmp_sag_min = pLFC_local_3x1[1] - ZMP_SAG_MARGIN;
		}

		zmp_fron_max = pLFC_local_3x1[2] + ZMP_FRON_MARGIN;
		zmp_fron_min = pRFC_local_3x1[2] - ZMP_FRON_MARGIN;

		ComTZ(t2, t3, t4, pCOM_local_3x1[1], vCOM_local_3x1[1], 0.5f*(pLFC_local_3x1[1]+pRFC_local_3x1[1]), 0., pCOM_local_3x1[3], zmp_sag_max, zmp_sag_min, 2, &_WalkTraj.pCOM_3x1[length_1st_half],1, &vCOM_sag_temp, &ui_temp, &dt11, &dt21);	// mode 2
		ComTZ(t2, t3, t4, pCOM_local_3x1[2], vCOM_local_3x1[2], 0.5f*(pLFC_local_3x1[2]+pRFC_local_3x1[2]), 0., pCOM_local_3x1[3], zmp_fron_max, zmp_fron_min, 2, &_WalkTraj.pCOM_3x1[length_1st_half],2, &vCOM_fron_temp, &ui_temp, &dt12, &dt22);	// mode 2		
	
		if(fabs(FMAX(dt11,dt12)) > EPS || fabs(FMAX(dt21,dt22)) > EPS)
		{
			t3 = t3 + (float)FMAX(dt11,dt12);
			t4 = t4 + (float)FMAX(dt21,dt22);
			
			ComTZ(t2, t3, t4, pCOM_local_3x1[1], vCOM_local_3x1[1], 0.5f*(pLFC_local_3x1[1]+pRFC_local_3x1[1]), 0., pCOM_local_3x1[3], zmp_sag_max, zmp_sag_min, 2, &_WalkTraj.pCOM_3x1[length_1st_half],1, &vCOM_sag_temp, &ui_temp, &dt11, &dt21);	// mode 2
			ComTZ(t2, t3, t4, pCOM_local_3x1[2], vCOM_local_3x1[2], 0.5f*(pLFC_local_3x1[2]+pRFC_local_3x1[2]), 0., pCOM_local_3x1[3], zmp_fron_max, zmp_fron_min, 2, &_WalkTraj.pCOM_3x1[length_1st_half],2, &vCOM_fron_temp, &ui_temp, &dt12, &dt22);	// mode 2		
		}
		ui_temp = (unsigned int)my_round((t3-t2-T_DFSP*0.5f)/DT);
		for(i=1; i<= ui_temp; i++)
			_WalkTraj.pCOM_3x1[i+length_1st_half][3] = pCOM_local_3x1[3];
		_WalkTraj.length = length_1st_half + ui_temp;

		RZ(_TwoStepBuff[0].yRF, _TEMP1_34x34);
		for(i=1+length_1st_half; i<=_WalkTraj.length; i++)
		{			
			mult_mv((const float**)_TEMP1_34x34,3,3, _WalkTraj.pCOM_3x1[i], temp1_3x1);
			_WalkTraj.pCOM_3x1[i][1] = temp1_3x1[1];
			_WalkTraj.pCOM_3x1[i][2] = temp1_3x1[2];
			_WalkTraj.pCOM_3x1[i][3] = temp1_3x1[3];
		}
		temp2_3x1[1] = vCOM_sag_temp;
		temp2_3x1[2] = vCOM_fron_temp;
		temp2_3x1[3] = 0.;
		mult_mv((const float**)_TEMP1_34x34,3,3, temp2_3x1, temp1_3x1);
		vCOM_3x1[1] = temp1_3x1[1];
		vCOM_3x1[2] = temp1_3x1[2];
	}
	else
	{
		printf("\nWTG error2!!\n");
		return (-1);
	}
	pCOM_3x1[1] = _WalkTraj.pCOM_3x1[_WalkTraj.length][1];
	pCOM_3x1[2] = _WalkTraj.pCOM_3x1[_WalkTraj.length][2];
	pCOM_3x1[3] = _WalkTraj.pCOM_3x1[_WalkTraj.length][3];

	switch(_TwoStepBuff[0].supporting_foot)
	{
	case DFSP:
		FTG3(_TwoStepBuff[0].pRF_3x1, _TwoStepBuff[0].yRF, pRF_3x1, yRF, t0, t1, t2, t3, 0, 1, _WalkTraj.pRF_3x1, _WalkTraj.temp);
		for(i=1; i<=_WalkTraj.length; i++)
		{
			_WalkTraj.qRF_4x1[i][1] = (float)cos(0.5f*_WalkTraj.temp[i]);
			_WalkTraj.qRF_4x1[i][2] = 0.;
			_WalkTraj.qRF_4x1[i][3] = 0.;
			_WalkTraj.qRF_4x1[i][4] = (float)sin(0.5f*_WalkTraj.temp[i]);
			_WalkTraj.yPEL[i] = _WalkTraj.temp[i];
		}
		FTG3(_TwoStepBuff[0].pLF_3x1, _TwoStepBuff[0].yLF, pLF_3x1, yLF, t0, t1, t2, t3, 0, 1, _WalkTraj.pLF_3x1, _WalkTraj.temp);
		for(i=1; i<=_WalkTraj.length; i++)
		{
			_WalkTraj.qLF_4x1[i][1] = (float)cos(0.5f*_WalkTraj.temp[i]);
			_WalkTraj.qLF_4x1[i][2] = 0.;
			_WalkTraj.qLF_4x1[i][3] = 0.;
			_WalkTraj.qLF_4x1[i][4] = (float)sin(0.5f*_WalkTraj.temp[i]);
			_WalkTraj.yPEL[i] = 0.5f*(_WalkTraj.temp[i]+_WalkTraj.yPEL[i]);
		}
		break;

	case RFSP:
		FTG3(_TwoStepBuff[0].pRF_3x1, _TwoStepBuff[0].yRF, pRF_3x1, yRF, t0, t1, t2, t3, 0, 1, _WalkTraj.pRF_3x1, _WalkTraj.temp);
		for(i=1; i<=_WalkTraj.length; i++)
		{
			_WalkTraj.qRF_4x1[i][1] = (float)cos(0.5f*_WalkTraj.temp[i]);
			_WalkTraj.qRF_4x1[i][2] = 0.;
			_WalkTraj.qRF_4x1[i][3] = 0.;
			_WalkTraj.qRF_4x1[i][4] = (float)sin(0.5f*_WalkTraj.temp[i]);
			_WalkTraj.yPEL[i] = _WalkTraj.temp[i];
		}
		FTG3(_TwoStepBuff[0].pLF_3x1, _TwoStepBuff[0].yLF, pLF_3x1, yLF, t0, t1, t2, t3, 0, 0, _WalkTraj.pLF_3x1, _WalkTraj.temp);
		for(i=1; i<=_WalkTraj.length; i++)
		{
			_WalkTraj.qLF_4x1[i][1] = (float)cos(0.5f*_WalkTraj.temp[i]);
			_WalkTraj.qLF_4x1[i][2] = 0.;
			_WalkTraj.qLF_4x1[i][3] = 0.;
			_WalkTraj.qLF_4x1[i][4] = (float)sin(0.5f*_WalkTraj.temp[i]);
			_WalkTraj.yPEL[i] = 0.5f*(_WalkTraj.temp[i]+_WalkTraj.yPEL[i]);
		}		
		break;

	case LFSP:
		FTG3(_TwoStepBuff[0].pRF_3x1, _TwoStepBuff[0].yRF, pRF_3x1, yRF, t0, t1, t2, t3, 0, 0, _WalkTraj.pRF_3x1, _WalkTraj.temp);
		for(i=1; i<=_WalkTraj.length; i++)
		{
			_WalkTraj.qRF_4x1[i][1] = (float)cos(0.5f*_WalkTraj.temp[i]);
			_WalkTraj.qRF_4x1[i][2] = 0.;
			_WalkTraj.qRF_4x1[i][3] = 0.;
			_WalkTraj.qRF_4x1[i][4] = (float)sin(0.5f*_WalkTraj.temp[i]);
			_WalkTraj.yPEL[i] = _WalkTraj.temp[i];
		}
		FTG3(_TwoStepBuff[0].pLF_3x1, _TwoStepBuff[0].yLF, pLF_3x1, yLF, t0, t1, t2, t3, 0, 1, _WalkTraj.pLF_3x1, _WalkTraj.temp);
		for(i=1; i<=_WalkTraj.length; i++)
		{
			_WalkTraj.qLF_4x1[i][1] = (float)cos(0.5f*_WalkTraj.temp[i]);
			_WalkTraj.qLF_4x1[i][2] = 0.;
			_WalkTraj.qLF_4x1[i][3] = 0.;
			_WalkTraj.qLF_4x1[i][4] = (float)sin(0.5f*_WalkTraj.temp[i]);
			_WalkTraj.yPEL[i] = 0.5f*(_WalkTraj.temp[i]+_WalkTraj.yPEL[i]);
		}
		break;
	default:
		printf("\n WTG error3!!");
		return -1;
	}
	
	_WalkTraj.i = 1;

	return 0;
}



int ComTZ(float t0, float t1, float t2, float pCOM0, float vCOM0, float pCOM2, float vCOM2, float pCOMz, float zmp_max, float zmp_min, int mode, 
		  float pCOM_result[][4], int index, float *vCOM_result, unsigned int *n_result, float *dt1_result, float *dt2_result) // COM trajectory satisfying ZMP
{
// p_com : COM trajectories
// p_com0, v_com0 : position and velocity of COM at t0
// p_com2 : desired ZMP at t2 in the mode 1, or desired COM at t2 in the mode 0 and 2
// mode :  == 0: boundary conditions (position=p_com2 and velocity=v_com2) at t12, t12 is calculated by applying the limited ZMP
//         == 1: satisfying the given zmp(=p_com2) for t1 <= t <= t2 and the boundary condition v_com2
//         == 2: boundary conditions (position=p_com2 and velocity=v_com2) at t2
//		   == 3: static walking

	const float lambda = (float)sqrt(9.81f/pCOMz);

	const float e = (float)1e-4;
	const float starting_offset = (float)5e-2;
	float c_6x1[7], c_temp_6x1[7];
	float des_zmp, dt_temp;
	float t1_new = my_round(t1/DT)*DT;
	float t2_new = my_round(t2/DT)*DT;
	float t12, t;
	unsigned int i;
	int i_temp;

	//---------------------------- t0~t1 구간에서 zmp 가 범위를 넘지 않도록 t1 조정
	ASZ(t0,t1_new,t2_new, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);

	if(c_6x1[3] < zmp_min || c_6x1[3] > zmp_max)
	{
		if(c_6x1[3] < zmp_min)
			des_zmp = zmp_min;
		else
			des_zmp = zmp_max;

		ASZ(t0, t1_new+e, t2_new, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_temp_6x1);
		dt_temp = -e/(c_temp_6x1[3]-c_6x1[3])*(c_6x1[3]-des_zmp);

		while(fabs(dt_temp) > 1e-6)
		{
			t1_new += dt_temp;
			t2_new += dt_temp;
			ASZ(t0, t1_new, t2_new, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);
			ASZ(t0, t1_new+e, t2_new, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_temp_6x1);
			dt_temp = -e/(c_temp_6x1[3]-c_6x1[3])*(c_6x1[3]-des_zmp);
		}
		i_temp = (int)(t1_new*1e6);		
		t1_new = (float)ceil((float)i_temp*1e-6/DT)*DT;
		i_temp = (int)(t2_new*1e6);		
		t2_new = (float)ceil((float)i_temp*1e-6/DT)*DT;
	}

	switch(mode)
	{
	case 0:
		// ------------------------------------------ t1~t2 구간에서 p_com2에 도달하는 가장 짧은 시간 t12 찾기
		ASZ(t0, t1_new, t2_new, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);

		if(c_6x1[6] <= zmp_min || c_6x1[6] >= zmp_max) // t2_new 가 일러서 zmp 범위를 벗어나면 t2_new를 미룸
		{
			if(c_6x1[6] <= zmp_min)
				des_zmp = zmp_min;
			else
				des_zmp = zmp_max;

			t2_new = t1_new + starting_offset;
			ASZ(t0, t1_new, t2_new+e, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_temp_6x1);
			dt_temp = -e/(c_temp_6x1[6]-c_6x1[6])*(c_6x1[6]-des_zmp);

			while(fabs(dt_temp) > 1e-6)
			{	
				t2_new += dt_temp;
				ASZ(t0, t1_new, t2_new, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);
				ASZ(t0, t1_new, t2_new+e, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_temp_6x1);
				dt_temp = -e/(c_temp_6x1[6]-c_6x1[6])*(c_6x1[6]-des_zmp);
			}
			i_temp = (int)(t2_new*1e6);		
			t2_new = (float)ceil((float)i_temp*1e6/DT)*DT;
			ASZ(t0, t1_new, t2_new, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);
			t12 = t2_new;
		}
		else
		{
			if(pCOM2 < pCOM0)
				des_zmp = zmp_min;
			else
				des_zmp = zmp_max;

			t12 = t1_new + starting_offset;
			ASZ(t0, t1_new, t12, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_temp_6x1);			

			if( (c_6x1[6]-des_zmp)*(c_temp_6x1[6]-des_zmp) < 0 ) // t1_new + starting_offset 와 t2_new 사이의 t12 로는 C(6) == des_zmp 만족하는 해 있다면
			{
				ASZ(t0, t1_new, t12+e, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_temp_6x1);
				dt_temp = -e/(c_temp_6x1[6]-c_6x1[6])*(c_6x1[6]-des_zmp);

				while(fabs(dt_temp) > 1e-6)
				{
					t12 += dt_temp;
					ASZ(t0, t1_new, t12, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);
					ASZ(t0, t1_new, t12+e, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_temp_6x1);
					dt_temp = -e/(c_temp_6x1[6]-c_6x1[6])*(c_6x1[6]-des_zmp);
				}
			}
			ASZ(t0, t1_new, t12, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);
		}
		break;

	case 1:
		ASZ2(t0, t1_new, t2_new, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);
		t12 = t2_new;
		break;
	
	case 2:
		ASZ(t0, t1_new, t2_new, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);

		if(c_6x1[6] <= zmp_min || c_6x1[6] >= zmp_max)  // t2_new 가 일러서 zmp 범위를 벗어나면 t2_new를 미룸
		{
			if(c_6x1[6] <= zmp_min)
				des_zmp = zmp_min;
			else
				des_zmp = zmp_max;

			t2_new = t1_new + starting_offset;
			ASZ(t0, t1_new, t2_new+e, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_temp_6x1);
			dt_temp = -e/(c_temp_6x1[6]-c_6x1[6])*(c_6x1[6]-des_zmp);

			while(fabs(dt_temp) > 1e-6)
			{
				t2_new += dt_temp;
				ASZ(t0, t1_new, t2_new, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);
				ASZ(t0, t1_new, t2_new+e, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_temp_6x1);
				dt_temp = -e/(c_temp_6x1[6]-c_6x1[6])*(c_6x1[6]-des_zmp);
			}

			i_temp = (int)(t2_new*1e6);		
			t2_new = (float)ceil((float)i_temp*1e-6/DT)*DT;
			ASZ(t0, t1_new, t2_new, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);
			t12 = t2_new;
		}
		else
		{
			if(pCOM2 < pCOM0)
				des_zmp = zmp_min*0.5f;
			else
				des_zmp = zmp_max*0.5f;

			t12 = t1_new + starting_offset;
			ASZ(t0, t1_new, t12, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_temp_6x1);			

			if( (c_6x1[6]-des_zmp)*(c_temp_6x1[6]-des_zmp) < 0 ) // t1_new + starting_offset 와 t2_new 사이의 t12 로는 C(6) == des_zmp 만족하는 해 있다면
			{
				ASZ(t0, t1_new, t12+e, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_temp_6x1);
				dt_temp = -e/(c_temp_6x1[6]-c_6x1[6])*(c_6x1[6]-des_zmp);

				while(fabs(dt_temp) > 1e-6)
				{
					t12 += dt_temp;
					ASZ(t0, t1_new, t12, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);
					ASZ(t0, t1_new, t12+e, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_temp_6x1);
					dt_temp = -e/(c_temp_6x1[6]-c_6x1[6])*(c_6x1[6]-des_zmp);
				}
			}
			ASZ(t0, t1_new, t12, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);
		}
		break;

	case 3:
		// ------------------------------------------ t1~t2 구간에서 p_com2에 도달하는 시간 = t12
		ASZ(t0, t1_new, t2_new, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);

		if(c_6x1[6] <= zmp_min || c_6x1[6] >= zmp_max) // t2_new 가 일러서 zmp 범위를 벗어나면 t2_new를 미룸
		{
			if(c_6x1[6] <= zmp_min)
				des_zmp = zmp_min;
			else
				des_zmp = zmp_max;

			t2_new = t1_new + starting_offset;
			ASZ(t0, t1_new, t2_new+e, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_temp_6x1);
			dt_temp = -e/(c_temp_6x1[6]-c_6x1[6])*(c_6x1[6]-des_zmp);

			while(fabs(dt_temp) > 1e-6)
			{	
				t2_new += dt_temp;
				ASZ(t0, t1_new, t2_new, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);
				ASZ(t0, t1_new, t2_new+e, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_temp_6x1);
				dt_temp = -e/(c_temp_6x1[6]-c_6x1[6])*(c_6x1[6]-des_zmp);
			}
			i_temp = (int)(t2_new*1e6);		
			t2_new = (float)ceil((float)i_temp*1e6/DT)*DT;
			ASZ(t0, t1_new, t2_new, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);
			t12 = t2_new;
		}
		else
		{
			t12 = (t1_new + t2_new)*0.5f;
			ASZ(t0, t1_new, t12, pCOM0, vCOM0, pCOM2, vCOM2, pCOMz, c_6x1);
		}
		break;
	}

	*n_result = (unsigned int)my_round((t2_new-t0)/DT);
	
	for(i=1; i<=(*n_result); i++)
	{
		t = t0 + DT*(float)i;
		if(t <= t1_new)
		{
			pCOM_result[i][index] = c_6x1[1]*(float)exp(-lambda*(t-t0)) + c_6x1[2]*(float)exp(lambda*(t-t0)) + c_6x1[3];
			//vCOM_result[i] = -lambda*c_6x1[1]*(float)exp(-lambda*(t-t0)) + lambda*c_6x1[2]*(float)exp(lambda*(t-t0));
		}
		else if(t <= t12)
		{
			pCOM_result[i][index] = c_6x1[4]*(float)exp(-lambda*(t-t1)) + c_6x1[5]*(float)exp(lambda*(t-t1)) + c_6x1[6];
			//vCOM_result[i] = -lambda*c_6x1[4]*(float)exp(-lambda*(t-t1)) + lambda*c_6x1[5]*(float)exp(lambda*(t-t1));
		}
		else
		{
			pCOM_result[i][index] = c_6x1[4]*(float)exp(-lambda*(t12-t1)) + c_6x1[5]*(float)exp(lambda*(t12-t1)) + c_6x1[6];
			//vCOM_result[i] = -lambda*c_6x1[4]*(float)exp(-lambda*(t12-t1)) + lambda*c_6x1[5]*(float)exp(lambda*(t12-t1));
		}
	}

	if(t0 == 0.)  // first half step
		*vCOM_result = -lambda*c_6x1[4]*(float)exp(-lambda*(t12-t1)) + lambda*c_6x1[5]*(float)exp(lambda*(t12-t1));
	else	// second half step
	{
		t = t0 + DT*(float)my_round((t1-t0-T_DFSP*0.5f)/DT);
		if(t <= t1_new)
			*vCOM_result = -lambda*c_6x1[1]*(float)exp(-lambda*(t-t0)) + lambda*c_6x1[2]*(float)exp(lambda*(t-t0));
		else if(t <= t12)
			*vCOM_result = -lambda*c_6x1[4]*(float)exp(-lambda*(t-t1)) + lambda*c_6x1[5]*(float)exp(lambda*(t-t1));
		else
			*vCOM_result = -lambda*c_6x1[4]*(float)exp(-lambda*(t12-t1)) + lambda*c_6x1[5]*(float)exp(lambda*(t12-t1));
	}

	*dt1_result = t1_new - t1;
	*dt2_result = t2_new - t2;

	return 0;
}


int ASZ(float t0, float t1, float t2, float pCOM0, float vCOM0, float pCOM2, float vCOM2, float pCOMz, float *coeff_result_6x1) 
{												// coefficients of analytical solution to ZMP equation with a step ZMP input
// Boundary conditions @ t0, t2 : p_com0, v_com0, p_com2, v_com2
// Continuity @ t1 
// the solutions :  
//                   for t0<= t <= t1, cy = C(1)*exp(-lambda*(t-t0)) + C(2)*exp(lambda*(t-t0)) + C(3)
//                   for t1<= t <= t2, cy = C(4)*exp(-lambda*(t-t1)) + C(5)*exp(lambda*(t-t1)) + C(6)
	
	float lambda = (float)sqrt(9.81f/pCOMz);
	float temp1 = (float)exp(-lambda*(t1-t0));
	float temp2 = (float)exp(lambda*(t1-t0));
	float temp3 = (float)exp(-lambda*(t2-t1));
	float temp4 = (float)exp(lambda*(t2-t1));

	_TEMP4_34x34[1][1] = 1.f;
	_TEMP4_34x34[1][2] = 1.f;
	_TEMP4_34x34[1][3] = 1.f;
	_TEMP4_34x34[1][4] = 0.;
	_TEMP4_34x34[1][5] = 0.;
	_TEMP4_34x34[1][6] = 0.;

	_TEMP4_34x34[2][1] = -lambda;
	_TEMP4_34x34[2][2] = lambda;
	_TEMP4_34x34[2][3] = 0.;
	_TEMP4_34x34[2][4] = 0.;
	_TEMP4_34x34[2][5] = 0.;
	_TEMP4_34x34[2][6] = 0.;

	_TEMP4_34x34[3][1] = temp1;
	_TEMP4_34x34[3][2] = temp2;
	_TEMP4_34x34[3][3] = 1.f;
	_TEMP4_34x34[3][4] = -1.f;
	_TEMP4_34x34[3][5] = -1.f;
	_TEMP4_34x34[3][6] = -1.f;

	_TEMP4_34x34[4][1] = -lambda*temp1;
	_TEMP4_34x34[4][2] = lambda*temp2;
	_TEMP4_34x34[4][3] = 0.;
	_TEMP4_34x34[4][4] = lambda;
	_TEMP4_34x34[4][5] = -lambda;
	_TEMP4_34x34[4][6] = 0.;

	_TEMP4_34x34[5][1] = 0.;
	_TEMP4_34x34[5][2] = 0.;
	_TEMP4_34x34[5][3] = 0.;
	_TEMP4_34x34[5][4] = temp3;
	_TEMP4_34x34[5][5] = temp4;
	_TEMP4_34x34[5][6] = 1.f;

	_TEMP4_34x34[6][1] = 0.;
	_TEMP4_34x34[6][2] = 0.;
	_TEMP4_34x34[6][3] = 0.;
	_TEMP4_34x34[6][4] = -lambda*temp3;
	_TEMP4_34x34[6][5] = lambda*temp4;
	_TEMP4_34x34[6][6] = 0.;

	coeff_result_6x1[1] = pCOM0;
	coeff_result_6x1[2] = vCOM0;
	coeff_result_6x1[3] = 0;
	coeff_result_6x1[4] = 0;
	coeff_result_6x1[5] = pCOM2;
	coeff_result_6x1[6] = vCOM2;

	return (gaussj_mod(_TEMP4_34x34,6, coeff_result_6x1));	
}


int ASZ2(float t0, float t1, float t2, float pCOM0, float vCOM0, float zmp2, float vCOM2, float pCOMz, float *coeff_result_6x1)
{
// Boundary conditions @ t0, t2 : p_com0, v_com0, v_com2
// Continuity @ t1 
// ZMP between t1 and t2 : zmp2
// the solutions :  
//                   for t0<= t <= t1, cy = C(1)*exp(-lambda*(t-t0)) + C(2)*exp(lambda*(t-t0)) + C(3)
//                   for t1<= t <= t2, cy = C(4)*exp(-lambda*(t-t1)) + C(5)*exp(lambda*(t-t1)) + C(6)
	
	const float lambda = (float)sqrt(9.81f/pCOMz);
	const float temp1 = (float)exp(-lambda*(t1-t0));
	const float temp2 = (float)exp(lambda*(t1-t0));
	const float temp3 = (float)exp(-lambda*(t2-t1));
	const float temp4 = (float)exp(lambda*(t2-t1));

	_TEMP4_34x34[1][1] = 1.f;
	_TEMP4_34x34[1][2] = 1.f;
	_TEMP4_34x34[1][3] = 1.f;
	_TEMP4_34x34[1][4] = 0.;
	_TEMP4_34x34[1][5] = 0.;

	_TEMP4_34x34[2][1] = -lambda;
	_TEMP4_34x34[2][2] = lambda;
	_TEMP4_34x34[2][3] = 0.;
	_TEMP4_34x34[2][4] = 0.;
	_TEMP4_34x34[2][5] = 0.;

	_TEMP4_34x34[3][1] = temp1;
	_TEMP4_34x34[3][2] = temp2;
	_TEMP4_34x34[3][3] = 1.f;
	_TEMP4_34x34[3][4] = -1.f;
	_TEMP4_34x34[3][5] = -1.f;

	_TEMP4_34x34[4][1] = -lambda*temp1;
	_TEMP4_34x34[4][2] = lambda*temp2;
	_TEMP4_34x34[4][3] = 0.;
	_TEMP4_34x34[4][4] = lambda;
	_TEMP4_34x34[4][5] = -lambda;

	_TEMP4_34x34[5][1] = 0.;
	_TEMP4_34x34[5][2] = 0.;
	_TEMP4_34x34[5][3] = 0.;
	_TEMP4_34x34[5][4] = -lambda*temp3;
	_TEMP4_34x34[5][5] = lambda*temp4;

	coeff_result_6x1[1] = pCOM0;
	coeff_result_6x1[2] = vCOM0;
	coeff_result_6x1[3] = zmp2;
	coeff_result_6x1[4] = 0.;
	coeff_result_6x1[5] = vCOM2;

	gaussj_mod(_TEMP4_34x34,5, coeff_result_6x1);
	coeff_result_6x1[6] = zmp2;

	return 0;

}


int FTG3(float *pGoal_3x1, float yGoal, float *pLast_3x1, float yLast, float t0, float t1, float t2, float t3, char isStandingStep, char isSupportingFoot, float pResult[][4], float yResult[]) // Foot Trajectory Generator
{
// t0~t1 : Tdfsp/2
// t2 : the mid-time of the step
// t1~t3 :  Tosp + Tdfsp

	const unsigned int N1 = (unsigned int)my_round((t1+0.5f*T_DFSP-t0)/DT);
	const unsigned int N2 = (unsigned int)my_round((t2-t0)/DT);
	const unsigned int N3 = (unsigned int)my_round((t3-0.5f*T_DFSP-t0)/DT);
	const unsigned int Nosp = N3-N1; // one step period
	const unsigned int Nhalf_1st = N2-N1;
	const unsigned int Nhalf_2nd = N3-N2;
	const unsigned int Nhor = 0;	// 지면에서 발이 떨어지고 Nhor 후 부터 horizontal 방향으로 이동

	float dP_3x1[4], dY;
	float dLift_1st, dLift_2nd;
	float norm_dP;
	float coeff_6x1[7], f_temp;

	unsigned int i;
	
	dP_3x1[1] = pGoal_3x1[1] - pLast_3x1[1];
	dP_3x1[2] = pGoal_3x1[2] - pLast_3x1[2];
	dP_3x1[3] = pGoal_3x1[3] - pLast_3x1[3];

	dY = yGoal - yLast;

	//norm_dP = (float)sqrt(dP_3x1[1]*dP_3x1[1] + dP_3x1[2]*dP_3x1[2] + dP_3x1[3]*dP_3x1[3]);
	norm_dP = (float)sqrt(dP_3x1[1]*dP_3x1[1] + dP_3x1[2]*dP_3x1[2]);

	if(isSupportingFoot == 0)
	{
		if(dP_3x1[3] > EPS*5.f)
		{
			dLift_1st = dLIFT + dP_3x1[3];
			dLift_2nd = -dLIFT;
		}
		else if(dP_3x1[3] < -EPS*5.f)
		{
			dLift_1st = dLIFT;
			dLift_2nd = -dLIFT + dP_3x1[3];
		}
		else
		{
			if(norm_dP < MOVING_FOOT_CUTOFF)
			{
				if(isStandingStep == 1)
				{
					dLift_1st = dLIFT;
					dLift_2nd = -dLIFT;
				}
				else
				{
					dP_3x1[1] = 0.;
					dP_3x1[2] = 0.;
					dP_3x1[3] = 0.;

					dLift_1st = 0.;
					dLift_2nd = 0.;
				}
			}
			else
			{
				dLift_1st = dLIFT;
				dLift_2nd = -dLIFT;
			}
		}
	}
	else
	{
		dP_3x1[1] = 0.;
		dP_3x1[2] = 0.;
	}
	

	for(i=1; i<=N1+Nhor; i++)
	{
		pResult[i][1] = pLast_3x1[1];
		pResult[i][2] = pLast_3x1[2];
		pResult[i][3] = pLast_3x1[3];
		yResult[i] = yLast;
	}


	poly5(0, (float)(Nosp-2*Nhor), 0., 1.f, 0., 0., 0., 0., coeff_6x1);
	for(i=1; i<=Nosp-2*Nhor; i++)
	{
		f_temp = coeff_6x1[1] + coeff_6x1[2]*(float)i + coeff_6x1[3]*(float)(i*i) + coeff_6x1[4]*(float)pow(i,3) + coeff_6x1[5]*(float)pow(i,4) + coeff_6x1[6]*(float)pow(i,5);
		pResult[N1+Nhor+i][1] = dP_3x1[1]*f_temp + pLast_3x1[1];
		pResult[N1+Nhor+i][2] = dP_3x1[2]*f_temp + pLast_3x1[2];
		yResult[N1+Nhor+i] = dY*f_temp + yLast;
	}
	
	for(i=1; i<=Nhor; i++)
	{
		pResult[N3-Nhor+i][1] = pResult[N3-Nhor][1];
		pResult[N3-Nhor+i][2] = pResult[N3-Nhor][2];
		yResult[N3-Nhor+i] = yResult[N3-Nhor];
	}		

/*	poly5(0, ((float)Nhalf_1st), 0., 1.f, 0., 0., 0., 0., coeff_6x1);
	for(i=1; i<=Nhalf_1st; i++)
	{
		f_temp = coeff_6x1[1] + coeff_6x1[2]*(float)i + coeff_6x1[3]*(float)(i*i) + coeff_6x1[4]*(float)pow(i,3) + coeff_6x1[5]*(float)pow(i,4) + coeff_6x1[6]*(float)pow(i,5);
		pResult[N1+i][3] = dLift_1st*f_temp + pLast_3x1[3];
	}

	poly5(0, ((float)Nhalf_2nd), 0., 1.f, 0., 0., 0., 0., coeff_6x1);
	for(i=1; i<=Nhalf_2nd; i++)
	{
		f_temp = coeff_6x1[1] + coeff_6x1[2]*(float)i + coeff_6x1[3]*(float)(i*i) + coeff_6x1[4]*(float)pow(i,3) + coeff_6x1[5]*(float)pow(i,4) + coeff_6x1[6]*(float)pow(i,5);
		pResult[N2+i][3] = dLift_2nd*f_temp + pResult[N2][3];
	}
*/
	if(isSupportingFoot == 0)
	{
		for(i=1; i<=Nhalf_1st; i++)
		{
			f_temp = (float)(0.5f*(1.f-cos(PI/Nhalf_1st*i)));
			pResult[N1+i][3] = dLift_1st*f_temp + pLast_3x1[3];
		}

		for(i=1; i<=Nhalf_2nd; i++)
		{
			f_temp = (float)(0.5f*(1.f-cos(PI/Nhalf_2nd*i)));
			pResult[N2+i][3] = dLift_2nd*f_temp + pResult[N2][3];
		}
	}
	else
	{
		for(i=1; i<=Nhalf_1st+Nhalf_2nd; i++)
		{
			f_temp = (float)(0.5f*(1.f-cos(PI/(Nhalf_1st+Nhalf_2nd)*i)));
			pResult[N1+i][3] = dP_3x1[3]*f_temp + pLast_3x1[3];
		}
	}
		
	return 0;
}

int poly5(float ti, float tf, float yi, float yf, float ypi, float ypf, float yppi, float yppf, float *coeff_result_6x1)	
{
	// 5th order polynomial, y(t)=coeff(1) + coeff(2)*t + coeff(3)*t^2 + coeff(4)*t^3 + coeff(5)*t^4 + coeff(6)*t^5
	_TEMP3_34x34[1][1] = 1.f;
	_TEMP3_34x34[1][2] = ti;
	_TEMP3_34x34[1][3] = ti*ti;
	_TEMP3_34x34[1][4] = (float)pow(ti,3);
	_TEMP3_34x34[1][5] = (float)pow(ti,4);
	_TEMP3_34x34[1][6] = (float)pow(ti,5);

	_TEMP3_34x34[2][1] = 1.f;
	_TEMP3_34x34[2][2] = tf;
	_TEMP3_34x34[2][3] = tf*tf;
	_TEMP3_34x34[2][4] = (float)pow(tf,3);
	_TEMP3_34x34[2][5] = (float)pow(tf,4);
	_TEMP3_34x34[2][6] = (float)pow(tf,5);

	_TEMP3_34x34[3][1] = 0.;
	_TEMP3_34x34[3][2] = 1.f;
	_TEMP3_34x34[3][3] = 2.f*ti;
	_TEMP3_34x34[3][4] = 3.f*ti*ti;
	_TEMP3_34x34[3][5] = 4.f*(float)pow(ti,3);
	_TEMP3_34x34[3][6] = 5.f*(float)pow(ti,4);

	_TEMP3_34x34[4][1] = 0.;
	_TEMP3_34x34[4][2] = 1.f;
	_TEMP3_34x34[4][3] = 2.f*tf;
	_TEMP3_34x34[4][4] = 3.f*tf*tf;
	_TEMP3_34x34[4][5] = 4.f*(float)pow(tf,3);
	_TEMP3_34x34[4][6] = 5.f*(float)pow(tf,4);

	_TEMP3_34x34[5][1] = 0.;
	_TEMP3_34x34[5][2] = 0.;
	_TEMP3_34x34[5][3] = 2.f;
	_TEMP3_34x34[5][4] = 6.f*ti;
	_TEMP3_34x34[5][5] = 12.f*ti*ti;
	_TEMP3_34x34[5][6] = 20.f*(float)pow(ti,3);

	_TEMP3_34x34[6][1] = 0.;
	_TEMP3_34x34[6][2] = 0.;
	_TEMP3_34x34[6][3] = 2.f;
	_TEMP3_34x34[6][4] = 6.f*tf;
	_TEMP3_34x34[6][5] = 12.f*tf*tf;
	_TEMP3_34x34[6][6] = 20.f*(float)pow(tf,3);

	coeff_result_6x1[1] = yi;
	coeff_result_6x1[2] = yf;
	coeff_result_6x1[3] = ypi;
	coeff_result_6x1[4] = ypf;
	coeff_result_6x1[5] = yppi;
	coeff_result_6x1[6] = yppf;

	return (gaussj_mod(_TEMP3_34x34,6, coeff_result_6x1));
}


int LoadWalkingDemo(int WalkingNo)
{
	float dforward, dyaw, rot, Tstep;
	unsigned int nstep, nbundle, i;
	char puttogether;
	FILE *fp;

	switch(WalkingNo)
	{
	case 1:
		if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\WalkingDemo_1.txt","r")) == NULL )
		{
			printf("WalkingDemo_1.txt was was not found\n");		
			return 1;
		}
		break;
	default:
		printf("\nThe selected WalkingDemo does not exist!\n");		
		return 1;
	}	
	
	while (fgetc(fp) != '\n');

	fscanf(fp, "%d", &nbundle);
	fgetc(fp);	// move to the next line from the scalar
	fgetc(fp);	// move to the next line from the blank line	

	while (fgetc(fp) != '\n');

	for(i=1; i<=nbundle; i++)
	{
		fscanf(fp,"%f %f %f %d %f %d", &dforward, &dyaw, &rot, &nstep, &Tstep, &puttogether);	
		
		if(PushSB(dforward, dyaw*D2R, rot, nstep, Tstep, puttogether) != 0)
		{
			printf("\n The step bundles after %d are ignored.\n", STEP_BUNDLE_RING_SIZE-1);
			break;
		}
	}
	
	fclose(fp);
	
	return 0;
	
}


int InitStepBundleRing(void)
{
	_SBR.head = 0;
	_SBR.tail = 0;
	_SBR.step_bundles[_SBR.tail].Nsteps = 0;
	_SBR.step_bundles[_SBR.tail].n_stepped = 0;

	return 0;
}


int CloseWBWalking(void)
{
	if(_SBR.step_bundles[_SBR.tail].n_stepped+2 <= _SBR.step_bundles[_SBR.tail].Nsteps)
	{
		_SBR.step_bundles[_SBR.tail].Nsteps = _SBR.step_bundles[_SBR.tail].n_stepped + 2;
		_SBR.head = _SBR.tail;		
	}
	else if(_SBR.step_bundles[_SBR.tail].n_stepped+1 == _SBR.step_bundles[_SBR.tail].Nsteps)
	{
		_SBR.head = (_SBR.tail+1)%STEP_BUNDLE_RING_SIZE;
		_SBR.step_bundles[_SBR.head].Nsteps = 1;
	}
	else if(CheckIdling()==1)
		return 1;
	
	return 0;	
}


int CheckIdling(void)
{
	if(_SBR.step_bundles[_SBR.tail].n_stepped >= _SBR.step_bundles[_SBR.tail].Nsteps+2)
		return 1;
	else if(_SBR.step_bundles[_SBR.tail].Nsteps == 0)
		return 1;
	else
		return 0;
}


int InsertSB(float dForward, float dYaw, float ROT, unsigned int Nsteps, float Tstep, char putTogether)
{
	unsigned int head_new;

	if(_SBR.step_bundles[_SBR.tail].n_stepped+2 <= _SBR.step_bundles[_SBR.tail].Nsteps)
	{
		_SBR.step_bundles[_SBR.tail].Nsteps = _SBR.step_bundles[_SBR.tail].n_stepped + 2;
		head_new = (_SBR.tail+1)%STEP_BUNDLE_RING_SIZE;		
	}
	else if(_SBR.step_bundles[_SBR.tail].n_stepped+1 == _SBR.step_bundles[_SBR.tail].Nsteps)
	{
		_SBR.head = (_SBR.tail+1)%STEP_BUNDLE_RING_SIZE;
		_SBR.step_bundles[_SBR.head].Nsteps = 1;
		head_new = (_SBR.head+1)%STEP_BUNDLE_RING_SIZE;
	}
	else 
		head_new = (_SBR.head+1)%STEP_BUNDLE_RING_SIZE;

	_SBR.step_bundles[head_new].dForward = dForward;
	_SBR.step_bundles[head_new].dYaw = dYaw;
	_SBR.step_bundles[head_new].ROT = ROT;
	_SBR.step_bundles[head_new].Nsteps = Nsteps;
	if(Tstep > T_STEP_MAX)
		_SBR.step_bundles[head_new].Tstep = T_STEP_MAX;
	else if (Tstep < T_STEP_MIN)
		_SBR.step_bundles[head_new].Tstep = T_STEP_MIN;
	else
		_SBR.step_bundles[head_new].Tstep = Tstep;
	_SBR.step_bundles[head_new].n_stepped = 0;
	_SBR.step_bundles[head_new].putTogether = putTogether;
	_SBR.step_bundles[head_new].pseudo_step = 0;
	_SBR.step_bundles[head_new].offsetR = 0.;
	_SBR.step_bundles[head_new].offsetL = 0.;
	_SBR.step_bundles[head_new].halfFlag = 0;

	if((dForward==0.) && (ROT==0.) && (putTogether==0) && (dYaw != 0.))	// if spin
		_SBR.step_bundles[head_new].Nsteps *= 2;

	if(putTogether != 0)
		_SBR.step_bundles[head_new].Nsteps = 1;

	_SBR.head = head_new;
	return 0;
}



int LoadControllerParamter(void)
{
	FILE *fp;
	
	if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\controller_dsp_f_DRC_biped.txt","r")) == NULL )
	{
		printf("Data file 'controller_dsp_f_DRC_biped.txt' was not found.\n");		
		return 1;
	}
	fscanf(fp,"%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e", 
		&_Ahat_DSP_F_BI_4x4[1][1], &_Ahat_DSP_F_BI_4x4[1][2], &_Ahat_DSP_F_BI_4x4[1][3], &_Ahat_DSP_F_BI_4x4[1][4],
		&_Ahat_DSP_F_BI_4x4[2][1], &_Ahat_DSP_F_BI_4x4[2][2], &_Ahat_DSP_F_BI_4x4[2][3], &_Ahat_DSP_F_BI_4x4[2][4],
		&_Ahat_DSP_F_BI_4x4[3][1], &_Ahat_DSP_F_BI_4x4[3][2], &_Ahat_DSP_F_BI_4x4[3][3], &_Ahat_DSP_F_BI_4x4[3][4],
		&_Ahat_DSP_F_BI_4x4[4][1], &_Ahat_DSP_F_BI_4x4[4][2], &_Ahat_DSP_F_BI_4x4[4][3], &_Ahat_DSP_F_BI_4x4[4][4]);

	fscanf(fp,"%e %e %e %e", 
		&_Bhat_DSP_F_BI_4x1[1], &_Bhat_DSP_F_BI_4x1[2], &_Bhat_DSP_F_BI_4x1[3], &_Bhat_DSP_F_BI_4x1[4]);
	fscanf(fp,"%e %e %e %e", 
		&_Lo_DSP_F_BI_4x1[1], &_Lo_DSP_F_BI_4x1[2], &_Lo_DSP_F_BI_4x1[3], &_Lo_DSP_F_BI_4x1[4]);
	fscanf(fp,"%e %e %e %e", 
		&_Kfb_DSP_F_BI_1x4[1], &_Kfb_DSP_F_BI_1x4[2], &_Kfb_DSP_F_BI_1x4[3], &_Kfb_DSP_F_BI_1x4[4]);
	fscanf(fp,"%e %e %e %e", 
		&_Bss_DSP_F_BI_4x1[1], &_Bss_DSP_F_BI_4x1[2], &_Bss_DSP_F_BI_4x1[3], &_Bss_DSP_F_BI_4x1[4]);
	fclose(fp);

	if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\controller_dsp_s_DRC_biped.txt","r")) == NULL )
	{
		printf("Data file 'controller_dsp_s_DRC_biped.txt' was not found.\n");		
		return 1;
	}
	fscanf(fp,"%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e", 
		&_Ahat_DSP_S_BI_4x4[1][1], &_Ahat_DSP_S_BI_4x4[1][2], &_Ahat_DSP_S_BI_4x4[1][3], &_Ahat_DSP_S_BI_4x4[1][4],
		&_Ahat_DSP_S_BI_4x4[2][1], &_Ahat_DSP_S_BI_4x4[2][2], &_Ahat_DSP_S_BI_4x4[2][3], &_Ahat_DSP_S_BI_4x4[2][4],
		&_Ahat_DSP_S_BI_4x4[3][1], &_Ahat_DSP_S_BI_4x4[3][2], &_Ahat_DSP_S_BI_4x4[3][3], &_Ahat_DSP_S_BI_4x4[3][4],
		&_Ahat_DSP_S_BI_4x4[4][1], &_Ahat_DSP_S_BI_4x4[4][2], &_Ahat_DSP_S_BI_4x4[4][3], &_Ahat_DSP_S_BI_4x4[4][4]);

	fscanf(fp,"%e %e %e %e", 
		&_Bhat_DSP_S_BI_4x1[1], &_Bhat_DSP_S_BI_4x1[2], &_Bhat_DSP_S_BI_4x1[3], &_Bhat_DSP_S_BI_4x1[4]);
	fscanf(fp,"%e %e %e %e", 
		&_Lo_DSP_S_BI_4x1[1], &_Lo_DSP_S_BI_4x1[2], &_Lo_DSP_S_BI_4x1[3], &_Lo_DSP_S_BI_4x1[4]);
	fscanf(fp,"%e %e %e %e", 
		&_Kfb_DSP_S_BI_1x4[1], &_Kfb_DSP_S_BI_1x4[2], &_Kfb_DSP_S_BI_1x4[3], &_Kfb_DSP_S_BI_1x4[4]);
	fscanf(fp,"%e %e %e %e", 
		&_Bss_DSP_S_BI_4x1[1], &_Bss_DSP_S_BI_4x1[2], &_Bss_DSP_S_BI_4x1[3], &_Bss_DSP_S_BI_4x1[4]);
	fclose(fp);


	if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\controller_ssp_s_DRC_biped.txt","r")) == NULL )
	{
		printf("Data file 'controller_ssp_s_DRC_biped.txt' was not found.\n");		
		return 1;
	}
	fscanf(fp,"%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e", 
		&_Ahat_SSP_S_BI_4x4[1][1], &_Ahat_SSP_S_BI_4x4[1][2], &_Ahat_SSP_S_BI_4x4[1][3], &_Ahat_SSP_S_BI_4x4[1][4],
		&_Ahat_SSP_S_BI_4x4[2][1], &_Ahat_SSP_S_BI_4x4[2][2], &_Ahat_SSP_S_BI_4x4[2][3], &_Ahat_SSP_S_BI_4x4[2][4],
		&_Ahat_SSP_S_BI_4x4[3][1], &_Ahat_SSP_S_BI_4x4[3][2], &_Ahat_SSP_S_BI_4x4[3][3], &_Ahat_SSP_S_BI_4x4[3][4],
		&_Ahat_SSP_S_BI_4x4[4][1], &_Ahat_SSP_S_BI_4x4[4][2], &_Ahat_SSP_S_BI_4x4[4][3], &_Ahat_SSP_S_BI_4x4[4][4]);

	fscanf(fp,"%e %e %e %e", 
		&_Bhat_SSP_S_BI_4x1[1], &_Bhat_SSP_S_BI_4x1[2], &_Bhat_SSP_S_BI_4x1[3], &_Bhat_SSP_S_BI_4x1[4]);
	fscanf(fp,"%e %e %e %e", 
		&_Lo_SSP_S_BI_4x1[1], &_Lo_SSP_S_BI_4x1[2], &_Lo_SSP_S_BI_4x1[3], &_Lo_SSP_S_BI_4x1[4]);
	fscanf(fp,"%e %e %e %e", 
		&_Kfb_SSP_S_BI_1x4[1], &_Kfb_SSP_S_BI_1x4[2], &_Kfb_SSP_S_BI_1x4[3], &_Kfb_SSP_S_BI_1x4[4]);
	fscanf(fp,"%e %e %e %e", 
		&_Bss_SSP_S_BI_4x1[1], &_Bss_SSP_S_BI_4x1[2], &_Bss_SSP_S_BI_4x1[3], &_Bss_SSP_S_BI_4x1[4]);
	fclose(fp);


	if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\controller_ssp_f_DRC_biped.txt","r")) == NULL )
	{
		printf("Data file 'controller_ssp_f_DRC_biped.txt' was not found.\n");		
		return 1;
	}
	fscanf(fp,"%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e", 
		&_Ahat_SSP_F_BI_4x4[1][1], &_Ahat_SSP_F_BI_4x4[1][2], &_Ahat_SSP_F_BI_4x4[1][3], &_Ahat_SSP_F_BI_4x4[1][4],
		&_Ahat_SSP_F_BI_4x4[2][1], &_Ahat_SSP_F_BI_4x4[2][2], &_Ahat_SSP_F_BI_4x4[2][3], &_Ahat_SSP_F_BI_4x4[2][4],
		&_Ahat_SSP_F_BI_4x4[3][1], &_Ahat_SSP_F_BI_4x4[3][2], &_Ahat_SSP_F_BI_4x4[3][3], &_Ahat_SSP_F_BI_4x4[3][4],
		&_Ahat_SSP_F_BI_4x4[4][1], &_Ahat_SSP_F_BI_4x4[4][2], &_Ahat_SSP_F_BI_4x4[4][3], &_Ahat_SSP_F_BI_4x4[4][4]);

	fscanf(fp,"%e %e %e %e", 
		&_Bhat_SSP_F_BI_4x1[1], &_Bhat_SSP_F_BI_4x1[2], &_Bhat_SSP_F_BI_4x1[3], &_Bhat_SSP_F_BI_4x1[4]);
	fscanf(fp,"%e %e %e %e", 
		&_Lo_SSP_F_BI_4x1[1], &_Lo_SSP_F_BI_4x1[2], &_Lo_SSP_F_BI_4x1[3], &_Lo_SSP_F_BI_4x1[4]);
	fscanf(fp,"%e %e %e %e", 
		&_Kfb_SSP_F_BI_1x4[1], &_Kfb_SSP_F_BI_1x4[2], &_Kfb_SSP_F_BI_1x4[3], &_Kfb_SSP_F_BI_1x4[4]);
	fscanf(fp,"%e %e %e %e", 
		&_Bss_SSP_F_BI_4x1[1], &_Bss_SSP_F_BI_4x1[2], &_Bss_SSP_F_BI_4x1[3], &_Bss_SSP_F_BI_4x1[4]);
	fclose(fp);


	if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\controller_dsp_f_DRC_quad.txt","r")) == NULL )
	{
		printf("Data file 'controller_dsp_f_DRC_quad.txt' was not found.\n");		
		return 1;
	}
	fscanf(fp,"%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e", 
		&(_Ahat_DSP_F_QUAD_4x4[1][1]), &(_Ahat_DSP_F_QUAD_4x4[1][2]), &(_Ahat_DSP_F_QUAD_4x4[1][3]), &(_Ahat_DSP_F_QUAD_4x4[1][4]),
		&(_Ahat_DSP_F_QUAD_4x4[2][1]), &(_Ahat_DSP_F_QUAD_4x4[2][2]), &(_Ahat_DSP_F_QUAD_4x4[2][3]), &(_Ahat_DSP_F_QUAD_4x4[2][4]),
		&(_Ahat_DSP_F_QUAD_4x4[3][1]), &(_Ahat_DSP_F_QUAD_4x4[3][2]), &(_Ahat_DSP_F_QUAD_4x4[3][3]), &(_Ahat_DSP_F_QUAD_4x4[3][4]),
		&(_Ahat_DSP_F_QUAD_4x4[4][1]), &(_Ahat_DSP_F_QUAD_4x4[4][2]), &(_Ahat_DSP_F_QUAD_4x4[4][3]), &(_Ahat_DSP_F_QUAD_4x4[4][4]));

	fscanf(fp,"%e %e %e %e", 
		&(_Bhat_DSP_F_QUAD_4x1[1]), &(_Bhat_DSP_F_QUAD_4x1[2]), &(_Bhat_DSP_F_QUAD_4x1[3]), &(_Bhat_DSP_F_QUAD_4x1[4]));
	fscanf(fp,"%e %e %e %e", 
		&(_Lo_DSP_F_QUAD_4x1[1]), &(_Lo_DSP_F_QUAD_4x1[2]), &(_Lo_DSP_F_QUAD_4x1[3]), &(_Lo_DSP_F_QUAD_4x1[4]));
	fscanf(fp,"%e %e %e %e", 
		&(_Kfb_DSP_F_QUAD_1x4[1]), &(_Kfb_DSP_F_QUAD_1x4[2]), &(_Kfb_DSP_F_QUAD_1x4[3]), &(_Kfb_DSP_F_QUAD_1x4[4]));
	fscanf(fp,"%e %e %e %e", 
		&(_Bss_DSP_F_QUAD_4x1[1]), &(_Bss_DSP_F_QUAD_4x1[2]), &(_Bss_DSP_F_QUAD_4x1[3]), &(_Bss_DSP_F_QUAD_4x1[4]));
	fclose(fp);

	if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\controller_dsp_s_DRC_quad.txt","r")) == NULL )
	{
		printf("Data file 'controller_dsp_s_DRC_quad.txt' was not found.\n");		
		return 1;
	}
	fscanf(fp,"%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e", 
		&(_Ahat_DSP_S_QUAD_4x4[1][1]), &(_Ahat_DSP_S_QUAD_4x4[1][2]), &(_Ahat_DSP_S_QUAD_4x4[1][3]), &(_Ahat_DSP_S_QUAD_4x4[1][4]),
		&(_Ahat_DSP_S_QUAD_4x4[2][1]), &(_Ahat_DSP_S_QUAD_4x4[2][2]), &(_Ahat_DSP_S_QUAD_4x4[2][3]), &(_Ahat_DSP_S_QUAD_4x4[2][4]),
		&(_Ahat_DSP_S_QUAD_4x4[3][1]), &(_Ahat_DSP_S_QUAD_4x4[3][2]), &(_Ahat_DSP_S_QUAD_4x4[3][3]), &(_Ahat_DSP_S_QUAD_4x4[3][4]),
		&(_Ahat_DSP_S_QUAD_4x4[4][1]), &(_Ahat_DSP_S_QUAD_4x4[4][2]), &(_Ahat_DSP_S_QUAD_4x4[4][3]), &(_Ahat_DSP_S_QUAD_4x4[4][4]));

	fscanf(fp,"%e %e %e %e", 
		&(_Bhat_DSP_S_QUAD_4x1[1]), &(_Bhat_DSP_S_QUAD_4x1[2]), &(_Bhat_DSP_S_QUAD_4x1[3]), &(_Bhat_DSP_S_QUAD_4x1[4]));
	fscanf(fp,"%e %e %e %e", 
		&(_Lo_DSP_S_QUAD_4x1[1]), &(_Lo_DSP_S_QUAD_4x1[2]), &(_Lo_DSP_S_QUAD_4x1[3]), &(_Lo_DSP_S_QUAD_4x1[4]));
	fscanf(fp,"%e %e %e %e", 
		&(_Kfb_DSP_S_QUAD_1x4[1]), &(_Kfb_DSP_S_QUAD_1x4[2]), &(_Kfb_DSP_S_QUAD_1x4[3]), &(_Kfb_DSP_S_QUAD_1x4[4]));
	fscanf(fp,"%e %e %e %e", 
		&(_Bss_DSP_S_QUAD_4x1[1]), &(_Bss_DSP_S_QUAD_4x1[2]), &(_Bss_DSP_S_QUAD_4x1[3]), &(_Bss_DSP_S_QUAD_4x1[4]));
	fclose(fp);


	if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\controller_ssp_s_DRC_quad.txt","r")) == NULL )
	{
		printf("Data file 'controller_ssp_s_DRC_quad.txt' was not found.\n");		
		return 1;
	}
	fscanf(fp,"%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e", 
		&(_Ahat_SSP_S_QUAD_4x4[1][1]), &(_Ahat_SSP_S_QUAD_4x4[1][2]), &(_Ahat_SSP_S_QUAD_4x4[1][3]), &(_Ahat_SSP_S_QUAD_4x4[1][4]),
		&(_Ahat_SSP_S_QUAD_4x4[2][1]), &(_Ahat_SSP_S_QUAD_4x4[2][2]), &(_Ahat_SSP_S_QUAD_4x4[2][3]), &(_Ahat_SSP_S_QUAD_4x4[2][4]),
		&(_Ahat_SSP_S_QUAD_4x4[3][1]), &(_Ahat_SSP_S_QUAD_4x4[3][2]), &(_Ahat_SSP_S_QUAD_4x4[3][3]), &(_Ahat_SSP_S_QUAD_4x4[3][4]),
		&(_Ahat_SSP_S_QUAD_4x4[4][1]), &(_Ahat_SSP_S_QUAD_4x4[4][2]), &(_Ahat_SSP_S_QUAD_4x4[4][3]), &(_Ahat_SSP_S_QUAD_4x4[4][4]));

	fscanf(fp,"%e %e %e %e", 
		&(_Bhat_SSP_S_QUAD_4x1[1]), &(_Bhat_SSP_S_QUAD_4x1[2]), &(_Bhat_SSP_S_QUAD_4x1[3]), &(_Bhat_SSP_S_QUAD_4x1[4]));
	fscanf(fp,"%e %e %e %e", 
		&(_Lo_SSP_S_QUAD_4x1[1]), &(_Lo_SSP_S_QUAD_4x1[2]), &(_Lo_SSP_S_QUAD_4x1[3]), &(_Lo_SSP_S_QUAD_4x1[4]));
	fscanf(fp,"%e %e %e %e", 
		&(_Kfb_SSP_S_QUAD_1x4[1]), &(_Kfb_SSP_S_QUAD_1x4[2]), &(_Kfb_SSP_S_QUAD_1x4[3]), &(_Kfb_SSP_S_QUAD_1x4[4]));
	fscanf(fp,"%e %e %e %e", 
		&(_Bss_SSP_S_QUAD_4x1[1]), &(_Bss_SSP_S_QUAD_4x1[2]), &(_Bss_SSP_S_QUAD_4x1[3]), &(_Bss_SSP_S_QUAD_4x1[4]));
	fclose(fp);


	if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\controller_ssp_f_DRC_quad.txt","r")) == NULL )
	{
		printf("Data file 'controller_ssp_f_DRC_quad.txt' was not found.\n");		
		return 1;
	}
	fscanf(fp,"%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e", 
		&(_Ahat_SSP_F_QUAD_4x4[1][1]), &(_Ahat_SSP_F_QUAD_4x4[1][2]), &(_Ahat_SSP_F_QUAD_4x4[1][3]), &(_Ahat_SSP_F_QUAD_4x4[1][4]),
		&(_Ahat_SSP_F_QUAD_4x4[2][1]), &(_Ahat_SSP_F_QUAD_4x4[2][2]), &(_Ahat_SSP_F_QUAD_4x4[2][3]), &(_Ahat_SSP_F_QUAD_4x4[2][4]),
		&(_Ahat_SSP_F_QUAD_4x4[3][1]), &(_Ahat_SSP_F_QUAD_4x4[3][2]), &(_Ahat_SSP_F_QUAD_4x4[3][3]), &(_Ahat_SSP_F_QUAD_4x4[3][4]),
		&(_Ahat_SSP_F_QUAD_4x4[4][1]), &(_Ahat_SSP_F_QUAD_4x4[4][2]), &(_Ahat_SSP_F_QUAD_4x4[4][3]), &(_Ahat_SSP_F_QUAD_4x4[4][4]));

	fscanf(fp,"%e %e %e %e", 
		&(_Bhat_SSP_F_QUAD_4x1[1]), &(_Bhat_SSP_F_QUAD_4x1[2]), &(_Bhat_SSP_F_QUAD_4x1[3]), &(_Bhat_SSP_F_QUAD_4x1[4]));
	fscanf(fp,"%e %e %e %e", 
		&(_Lo_SSP_F_QUAD_4x1[1]), &(_Lo_SSP_F_QUAD_4x1[2]), &(_Lo_SSP_F_QUAD_4x1[3]), &(_Lo_SSP_F_QUAD_4x1[4]));
	fscanf(fp,"%e %e %e %e", 
		&(_Kfb_SSP_F_QUAD_1x4[1]), &(_Kfb_SSP_F_QUAD_1x4[2]), &(_Kfb_SSP_F_QUAD_1x4[3]), &(_Kfb_SSP_F_QUAD_1x4[4]));
	fscanf(fp,"%e %e %e %e", 
		&(_Bss_SSP_F_QUAD_4x1[1]), &(_Bss_SSP_F_QUAD_4x1[2]), &(_Bss_SSP_F_QUAD_4x1[3]), &(_Bss_SSP_F_QUAD_4x1[4]));
	fclose(fp);

	return 0;	
}


int LoadControllerParamter_DRC(char walking_mode)
{
	int i;
	if(walking_mode == DRC_QUAD_MODE)
	{	
		for(i=1; i<=4; i++)
		{
			_Ahat_DSP_F_DRC_4x4[i][1] = _Ahat_DSP_F_QUAD_4x4[i][1];
			_Ahat_DSP_F_DRC_4x4[i][2] = _Ahat_DSP_F_QUAD_4x4[i][2];
			_Ahat_DSP_F_DRC_4x4[i][3] = _Ahat_DSP_F_QUAD_4x4[i][3];
			_Ahat_DSP_F_DRC_4x4[i][4] = _Ahat_DSP_F_QUAD_4x4[i][4];

			_Bhat_DSP_F_DRC_4x1[i] = _Bhat_DSP_F_QUAD_4x1[i];
			_Lo_DSP_F_DRC_4x1[i] = _Lo_DSP_F_QUAD_4x1[i];
			_Kfb_DSP_F_DRC_1x4[i] = _Kfb_DSP_F_QUAD_1x4[i];
			_Bss_DSP_F_DRC_4x1[i] = _Bss_DSP_F_QUAD_4x1[i];

			_Ahat_DSP_S_DRC_4x4[i][1] = _Ahat_DSP_S_QUAD_4x4[i][1];
			_Ahat_DSP_S_DRC_4x4[i][2] = _Ahat_DSP_S_QUAD_4x4[i][2];
			_Ahat_DSP_S_DRC_4x4[i][3] = _Ahat_DSP_S_QUAD_4x4[i][3];
			_Ahat_DSP_S_DRC_4x4[i][4] = _Ahat_DSP_S_QUAD_4x4[i][4];

			_Bhat_DSP_S_DRC_4x1[i] = _Bhat_DSP_S_QUAD_4x1[i];
			_Lo_DSP_S_DRC_4x1[i] = _Lo_DSP_S_QUAD_4x1[i];
			_Kfb_DSP_S_DRC_1x4[i] = _Kfb_DSP_S_QUAD_1x4[i];
			_Bss_DSP_S_DRC_4x1[i] = _Bss_DSP_S_QUAD_4x1[i];

			_Ahat_SSP_F_DRC_4x4[i][1] = _Ahat_SSP_F_QUAD_4x4[i][1];
			_Ahat_SSP_F_DRC_4x4[i][2] = _Ahat_SSP_F_QUAD_4x4[i][2];
			_Ahat_SSP_F_DRC_4x4[i][3] = _Ahat_SSP_F_QUAD_4x4[i][3];
			_Ahat_SSP_F_DRC_4x4[i][4] = _Ahat_SSP_F_QUAD_4x4[i][4];

			_Bhat_SSP_F_DRC_4x1[i] = _Bhat_SSP_F_QUAD_4x1[i];
			_Lo_SSP_F_DRC_4x1[i] = _Lo_SSP_F_QUAD_4x1[i];
			_Kfb_SSP_F_DRC_1x4[i] = _Kfb_SSP_F_QUAD_1x4[i];
			_Bss_SSP_F_DRC_4x1[i] = _Bss_SSP_F_QUAD_4x1[i];

			_Ahat_SSP_S_DRC_4x4[i][1] = _Ahat_SSP_S_QUAD_4x4[i][1];
			_Ahat_SSP_S_DRC_4x4[i][2] = _Ahat_SSP_S_QUAD_4x4[i][2];
			_Ahat_SSP_S_DRC_4x4[i][3] = _Ahat_SSP_S_QUAD_4x4[i][3];
			_Ahat_SSP_S_DRC_4x4[i][4] = _Ahat_SSP_S_QUAD_4x4[i][4];

			_Bhat_SSP_S_DRC_4x1[i] = _Bhat_SSP_S_QUAD_4x1[i];
			_Lo_SSP_S_DRC_4x1[i] = _Lo_SSP_S_QUAD_4x1[i];
			_Kfb_SSP_S_DRC_1x4[i] = _Kfb_SSP_S_QUAD_1x4[i];
			_Bss_SSP_S_DRC_4x1[i] = _Bss_SSP_S_QUAD_4x1[i];
		}
	}
	else if(walking_mode == DRC_BI_MODE)
	{
		for(i=1; i<=4; i++)
		{
			_Ahat_DSP_F_DRC_4x4[i][1] = _Ahat_DSP_F_BI_4x4[i][1];
			_Ahat_DSP_F_DRC_4x4[i][2] = _Ahat_DSP_F_BI_4x4[i][2];
			_Ahat_DSP_F_DRC_4x4[i][3] = _Ahat_DSP_F_BI_4x4[i][3];
			_Ahat_DSP_F_DRC_4x4[i][4] = _Ahat_DSP_F_BI_4x4[i][4];

			_Bhat_DSP_F_DRC_4x1[i] = _Bhat_DSP_F_BI_4x1[i];
			_Lo_DSP_F_DRC_4x1[i] = _Lo_DSP_F_BI_4x1[i];
			_Kfb_DSP_F_DRC_1x4[i] = _Kfb_DSP_F_BI_1x4[i];
			_Bss_DSP_F_DRC_4x1[i] = _Bss_DSP_F_BI_4x1[i];

			_Ahat_DSP_S_DRC_4x4[i][1] = _Ahat_DSP_S_BI_4x4[i][1];
			_Ahat_DSP_S_DRC_4x4[i][2] = _Ahat_DSP_S_BI_4x4[i][2];
			_Ahat_DSP_S_DRC_4x4[i][3] = _Ahat_DSP_S_BI_4x4[i][3];
			_Ahat_DSP_S_DRC_4x4[i][4] = _Ahat_DSP_S_BI_4x4[i][4];

			_Bhat_DSP_S_DRC_4x1[i] = _Bhat_DSP_S_BI_4x1[i];
			_Lo_DSP_S_DRC_4x1[i] = _Lo_DSP_S_BI_4x1[i];
			_Kfb_DSP_S_DRC_1x4[i] = _Kfb_DSP_S_BI_1x4[i];
			_Bss_DSP_S_DRC_4x1[i] = _Bss_DSP_S_BI_4x1[i];

			_Ahat_SSP_F_DRC_4x4[i][1] = _Ahat_SSP_F_BI_4x4[i][1];
			_Ahat_SSP_F_DRC_4x4[i][2] = _Ahat_SSP_F_BI_4x4[i][2];
			_Ahat_SSP_F_DRC_4x4[i][3] = _Ahat_SSP_F_BI_4x4[i][3];
			_Ahat_SSP_F_DRC_4x4[i][4] = _Ahat_SSP_F_BI_4x4[i][4];

			_Bhat_SSP_F_DRC_4x1[i] = _Bhat_SSP_F_BI_4x1[i];
			_Lo_SSP_F_DRC_4x1[i] = _Lo_SSP_F_BI_4x1[i];
			_Kfb_SSP_F_DRC_1x4[i] = _Kfb_SSP_F_BI_1x4[i];
			_Bss_SSP_F_DRC_4x1[i] = _Bss_SSP_F_BI_4x1[i];

			_Ahat_SSP_S_DRC_4x4[i][1] = _Ahat_SSP_S_BI_4x4[i][1];
			_Ahat_SSP_S_DRC_4x4[i][2] = _Ahat_SSP_S_BI_4x4[i][2];
			_Ahat_SSP_S_DRC_4x4[i][3] = _Ahat_SSP_S_BI_4x4[i][3];
			_Ahat_SSP_S_DRC_4x4[i][4] = _Ahat_SSP_S_BI_4x4[i][4];

			_Bhat_SSP_S_DRC_4x1[i] = _Bhat_SSP_S_BI_4x1[i];
			_Lo_SSP_S_DRC_4x1[i] = _Lo_SSP_S_BI_4x1[i];
			_Kfb_SSP_S_DRC_1x4[i] = _Kfb_SSP_S_BI_1x4[i];
			_Bss_SSP_S_DRC_4x1[i] = _Bss_SSP_S_BI_4x1[i];
		}
	}

	return 0;	
}


int LoadOfflineTraj(int mode)
{
	unsigned int k;
	FILE *fp;

	switch(mode)
	{
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
		printf("Invalid walking trajectory.\n");		
		return 1;
		break;
	case 8:
		if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\offline_traj_drc.txt","r")) == NULL )
		{
			printf("Data file offline_traj_drc.txt was not found\n");		
			return 1;
		}
		break;
	case 9:
		if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\offline_traj_drc_no_step.txt","r")) == NULL )
		{
			printf("Data file offline_traj_drc_no_step.txt was not found\n");		
			return 1;
		}
		break;
	default:
		return 1;
	}
	
	fscanf(fp,"%d %d", &_offline_traj.length, &_offline_traj.n_changes);
	if(_offline_traj.n_changes > OFFLINE_MAX_CHANGES)
	{
		printf("\n>>>----- OFFLINE_MAX_CHANGES error!");
		fclose(fp);
		return 1;
	}

	if(_offline_traj.length > OFFLINE_LENGTH)
	{
		printf("\n>>>----- OFFLINE_LENGTH error!");
		fclose(fp);
		return 1;
	}

	for(k=0; k<_offline_traj.n_changes; k++)
		fscanf(fp,"%d %d", &_offline_traj.i_change[k][0], &_offline_traj.i_change[k][1]);

	for(k=0; k<_offline_traj.n_changes; k++)
		fscanf(fp,"%d", &_offline_traj.des_fsp[k]);

	for (k=0; k < _offline_traj.length; k++)
		fscanf(fp,"%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e", 
			&_offline_traj.dpCOMx[k], &_offline_traj.dpCOMy[k], &_offline_traj.dpPELz[k],
			&_offline_traj.dpRFx[k], &_offline_traj.dpRFy[k], &_offline_traj.dpRFz[k], 
			&_offline_traj.dpLFx[k], &_offline_traj.dpLFy[k], &_offline_traj.dpLFz[k],
			&_offline_traj.dpRHx[k], &_offline_traj.dpRHy[k], &_offline_traj.dpRHz[k],
			&_offline_traj.dpLHx[k], &_offline_traj.dpLHy[k], &_offline_traj.dpLHz[k],				
			&_offline_traj.offsetZMPs[k], &_offline_traj.offsetZMPf[k]);
	
	_offline_traj.i = 0;
	fclose(fp);

	return 0;
}



//------------------------------ DRC, freq test
int WBIK_FreqTest(unsigned int n)  
{
	int i,j,k;
	
	float Qpp_33x1[34];
	float X1_33x1[34], Xrf_6x1[7], Xlf_6x1[7], Xrh_6x1[7], Xlh_6x1[7], Xcom_3x1[4], Xpel_3x1[4], Xwst, Xpelz, X2_33x1[34];  // task variables
	float ub_27x1[28], lb_27x1[28], qpp_limited[28];  // joint upper-bound, lower-bound
	int index_limited[28];
	
	float des_pCOM_3x1[4], des_vCOM_3x1[4];
	float des_pRF_3x1[4], des_pLF_3x1[4], des_vRF_3x1[4], des_vLF_3x1[4];
	float des_qRF_4x1[5], des_qLF_4x1[5], des_wRF_3x1[4], des_wLF_3x1[4];
	float des_pRH_3x1[4], des_pLH_3x1[4], des_vRH_3x1[4], des_vLH_3x1[4];
	float des_qRH_4x1[5], des_qLH_4x1[5], des_wRH_3x1[4], des_wLH_3x1[4];
	float des_WST, des_WSTp, des_qPEL_4x1[5];
	float des_pPELz, des_vPELz;
	float vCOM_3x1[4];

	BOOL isLimited = FALSE;
	int dim_primary_task, dim_redundant_task;

	float lL, lR, fRFz, fLFz;
	float rhr_compen = 0.f;
	float lhr_compen = 0.f;
	float rhy_compen = 0.f;
	float lhy_compen = 0.f;

	static char reset_neutral_flag;
	static float neutral_Q_34x1[35];
	static float pCOM1_3x1[4], pPELz1, pRF1_3x1[4], pLF1_3x1[4], qRF1_4x1[5], qLF1_4x1[5];
	static float pRH1_3x1[4], pLH1_3x1[4], qRH1_4x1[5], qLH1_4x1[5];
	static float t;

	float dpRF_3x1[4], dpLF_3x1[4], dqRF_4x1[5], dqLF_4x1[5];
	float dpRH_3x1[4], dpLH_3x1[4], dqRH_4x1[5], dqLH_4x1[5];
	float dpPELz, dpCOM_3x1[4];

	float ftemp1_7x1[8], ftemp2_7x1[8], ftemp3_7x1[8], ftemp4_7x1[8];

	if(_FKineUpdatedFlag != 1)
		return -4;
	
	if(n<=2)
	{
		neutral_Q_34x1[RSP_34] = Joint[RSP].RefAngleCurrent*D2R;
		neutral_Q_34x1[RSR_34] = (Joint[RSR].RefAngleCurrent + OFFSET_RSR)*D2R;
		neutral_Q_34x1[RSY_34] = Joint[RSY].RefAngleCurrent*D2R;
		neutral_Q_34x1[REB_34] = (Joint[REB].RefAngleCurrent + OFFSET_REB)*D2R;
		neutral_Q_34x1[RWY_34] = Joint[RWY].RefAngleCurrent*D2R;
		neutral_Q_34x1[RWP_34] = Joint[RWP].RefAngleCurrent*D2R;
		neutral_Q_34x1[RWY2_34] = 0.f; //((float)Joint[RWY2].EncoderValue)/Joint[RWY2].PPR*D2R;
		
		neutral_Q_34x1[LSP_34] = Joint[LSP].RefAngleCurrent*D2R;
		neutral_Q_34x1[LSR_34] = (Joint[LSR].RefAngleCurrent + OFFSET_LSR)*D2R;
		neutral_Q_34x1[LSY_34] = Joint[LSY].RefAngleCurrent*D2R;
		neutral_Q_34x1[LEB_34] = (Joint[LEB].RefAngleCurrent + OFFSET_LEB)*D2R;
		neutral_Q_34x1[LWY_34] = Joint[LWY].RefAngleCurrent*D2R;
		neutral_Q_34x1[LWP_34] = Joint[LWP].RefAngleCurrent*D2R;
		neutral_Q_34x1[LWY2_34] = 0.f; //((float)Joint[LWY2].EncoderValue)/Joint[LWY2].PPR*D2R;

		_pPEL0_3x1[1] = _Q_34x1[1];
		_pPEL0_3x1[2] = _Q_34x1[2];
		pPELz1 = _pPEL0_3x1[3] = _Q_34x1[3];

		pCOM1_3x1[1] = _pCOM0_3x1[1] = _pCOM_3x1[1];
		pCOM1_3x1[2] = _pCOM0_3x1[2] = _pCOM_3x1[2];
		pCOM1_3x1[3] = _pCOM0_3x1[3] = _pCOM_3x1[3];

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

		pRH1_3x1[1] = _pRH0_3x1[1] = _pRH_3x1[1];
		pRH1_3x1[2] = _pRH0_3x1[2] = _pRH_3x1[2];
		pRH1_3x1[3] = _pRH0_3x1[3] = _pRH_3x1[3];

		qRH1_4x1[1] = _qRH0_4x1[1] = _qRH_4x1[1];
		qRH1_4x1[2] = _qRH0_4x1[2] = _qRH_4x1[2];
		qRH1_4x1[3] = _qRH0_4x1[3] = _qRH_4x1[3];
		qRH1_4x1[4] = _qRH0_4x1[4] = _qRH_4x1[4];

		pLH1_3x1[1] = _pLH0_3x1[1] = _pLH_3x1[1];
		pLH1_3x1[2] = _pLH0_3x1[2] = _pLH_3x1[2];
		pLH1_3x1[3] = _pLH0_3x1[3] = _pLH_3x1[3];

		qLH1_4x1[1] = _qLH0_4x1[1] = _qLH_4x1[1];
		qLH1_4x1[2] = _qLH0_4x1[2] = _qLH_4x1[2];
		qLH1_4x1[3] = _qLH0_4x1[3] = _qLH_4x1[3];
		qLH1_4x1[4] = _qLH0_4x1[4] = _qLH_4x1[4];

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
		
		t = 0.f;
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
	
	dpPELz = 0.f;

	dpCOM_3x1[1] = 0.f;
	dpCOM_3x1[2] = 0.f;
	dpCOM_3x1[3] = 0.f;
	
	dpRF_3x1[1] = 0.f;
	dpRF_3x1[2] = 0.f;
	dpRF_3x1[3] = 0.f;
	
	dpLF_3x1[1] = 0.f;
	dpLF_3x1[2] = 0.f;
	dpLF_3x1[3] = 0.f;
	
	dpRH_3x1[1] = 0.f;
	dpRH_3x1[2] = 0.f;
	dpRH_3x1[3] = 0.f;
	
	dpLH_3x1[1] = 0.f;
	dpLH_3x1[2] = 0.f;
	dpLH_3x1[3] = 0.f;
	
	dqRF_4x1[1] = 1.f;
	dqRF_4x1[2] = 0.f;
	dqRF_4x1[3] = 0.f;
	dqRF_4x1[4] = 0.f;
	
	dqLF_4x1[1] = 1.f;
	dqLF_4x1[2] = 0.f;
	dqLF_4x1[3] = 0.f;
	dqLF_4x1[4] = 0.f;
	
	dqRH_4x1[1] = 1.f;
	dqRH_4x1[2] = 0.f;
	dqRH_4x1[3] = 0.f;
	dqRH_4x1[4] = 0.f;
	
	dqLH_4x1[1] = 1.f;
	dqLH_4x1[2] = 0.f;
	dqLH_4x1[3] = 0.f;
	dqLH_4x1[4] = 0.f;

	reset_neutral_flag = 0;
	if(pSharedMemory->position_mode_flag == 0) // staying
	{	
		t = 0.f;
	}
	else if(pSharedMemory->position_mode_flag == 1)  // move hands
	{	
		
		dpRH_3x1[1] = one_cos(t, pSharedMemory->dpRH[0], pSharedMemory->move_sec);
		dpRH_3x1[2] = one_cos(t, pSharedMemory->dpRH[1], pSharedMemory->move_sec);
		dpRH_3x1[3] = one_cos(t, pSharedMemory->dpRH[2], pSharedMemory->move_sec);

		dpLH_3x1[1] = one_cos(t, pSharedMemory->dpLH[0], pSharedMemory->move_sec);
		dpLH_3x1[2] = one_cos(t, pSharedMemory->dpLH[1], pSharedMemory->move_sec);
		dpLH_3x1[3] = one_cos(t, pSharedMemory->dpLH[2], pSharedMemory->move_sec);

		ftemp1_7x1[0] = one_cos(t, pSharedMemory->drvRH[0], pSharedMemory->move_sec);
		ftemp1_7x1[1] = pSharedMemory->drvRH[1];
		ftemp1_7x1[2] = pSharedMemory->drvRH[2];
		ftemp1_7x1[3] = pSharedMemory->drvRH[3];
		RV2QT(ftemp1_7x1, dqRH_4x1);

		ftemp1_7x1[0] = one_cos(t, pSharedMemory->drvLH[0], pSharedMemory->move_sec);
		ftemp1_7x1[1] = pSharedMemory->drvLH[1];
		ftemp1_7x1[2] = pSharedMemory->drvLH[2];
		ftemp1_7x1[3] = pSharedMemory->drvLH[3];
		RV2QT(ftemp1_7x1, dqLH_4x1);
		
		t += DT;

		if(t > pSharedMemory->move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 2)  //move RF and LF
	{
		dpRF_3x1[1] = one_cos(t, pSharedMemory->dpRF[0], pSharedMemory->move_sec);
		dpRF_3x1[2] = one_cos(t, pSharedMemory->dpRF[1], pSharedMemory->move_sec);
		dpRF_3x1[3] = one_cos(t, pSharedMemory->dpRF[2], pSharedMemory->move_sec);

		dpLF_3x1[1] = one_cos(t, pSharedMemory->dpLF[0], pSharedMemory->move_sec);
		dpLF_3x1[2] = one_cos(t, pSharedMemory->dpLF[1], pSharedMemory->move_sec);
		dpLF_3x1[3] = one_cos(t, pSharedMemory->dpLF[2], pSharedMemory->move_sec);

		ftemp1_7x1[0] = one_cos(t, pSharedMemory->drvRF[0], pSharedMemory->move_sec);
		ftemp1_7x1[1] = pSharedMemory->drvRF[1];
		ftemp1_7x1[2] = pSharedMemory->drvRF[2];
		ftemp1_7x1[3] = pSharedMemory->drvRF[3];
		RV2QT(ftemp1_7x1, dqRF_4x1);

		ftemp1_7x1[0] = one_cos(t, pSharedMemory->drvLF[0], pSharedMemory->move_sec);
		ftemp1_7x1[1] = pSharedMemory->drvLF[1];
		ftemp1_7x1[2] = pSharedMemory->drvLF[2];
		ftemp1_7x1[3] = pSharedMemory->drvLF[3];
		RV2QT(ftemp1_7x1, dqLF_4x1);

		t += DT;
		if(t > pSharedMemory->move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 3)  //move COM
	{	
		dpCOM_3x1[1] = one_cos(t, pSharedMemory->dpCOM[0], pSharedMemory->move_sec);
		dpCOM_3x1[2] = one_cos(t, pSharedMemory->dpCOM[1], pSharedMemory->move_sec);
	
		t += DT;
		if(t > pSharedMemory->move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 4)  //move pPELz
	{
		dpPELz = one_cos(t, pSharedMemory->dpPELz, pSharedMemory->move_sec);
	
		t += DT;
		if(t > pSharedMemory->move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 5)  //offline traj
	{
		dpPELz = pSharedMemory->off_traj_dpPELz[pSharedMemory->off_traj_count];
		dpCOM_3x1[1] = pSharedMemory->off_traj_dpCOM[pSharedMemory->off_traj_count][0];
		dpCOM_3x1[2] = pSharedMemory->off_traj_dpCOM[pSharedMemory->off_traj_count][1];
		dpRH_3x1[1] = pSharedMemory->off_traj_dpRH[pSharedMemory->off_traj_count][0];
		dpRH_3x1[2] = pSharedMemory->off_traj_dpRH[pSharedMemory->off_traj_count][1];
		dpRH_3x1[3] = pSharedMemory->off_traj_dpRH[pSharedMemory->off_traj_count][2];
		dpLH_3x1[1] = pSharedMemory->off_traj_dpLH[pSharedMemory->off_traj_count][0];
		dpLH_3x1[2] = pSharedMemory->off_traj_dpLH[pSharedMemory->off_traj_count][1];
		dpLH_3x1[3] = pSharedMemory->off_traj_dpLH[pSharedMemory->off_traj_count][2];
		dpRF_3x1[1] = pSharedMemory->off_traj_dpRF[pSharedMemory->off_traj_count][0];
		dpRF_3x1[2] = pSharedMemory->off_traj_dpRF[pSharedMemory->off_traj_count][1];
		dpRF_3x1[3] = pSharedMemory->off_traj_dpRF[pSharedMemory->off_traj_count][2];
		dpLF_3x1[1] = pSharedMemory->off_traj_dpLF[pSharedMemory->off_traj_count][0];
		dpLF_3x1[2] = pSharedMemory->off_traj_dpLF[pSharedMemory->off_traj_count][1];
		dpLF_3x1[3] = pSharedMemory->off_traj_dpLF[pSharedMemory->off_traj_count][2];
		RV2QT(pSharedMemory->off_traj_drvRH[pSharedMemory->off_traj_count], dqRH_4x1);
		RV2QT(pSharedMemory->off_traj_drvLH[pSharedMemory->off_traj_count], dqLH_4x1);
		RV2QT(pSharedMemory->off_traj_drvRF[pSharedMemory->off_traj_count], dqRF_4x1);
		RV2QT(pSharedMemory->off_traj_drvLF[pSharedMemory->off_traj_count], dqLF_4x1);
		pSharedMemory->off_traj_count++;
		if(pSharedMemory->off_traj_count >= pSharedMemory->off_traj_length)
		{
			reset_neutral_flag = 1;
			pSharedMemory->off_traj_count = 0;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 6) // freq test mode
	{
		dpCOM_3x1[1] = (float)sinf(pSharedMemory->WB_FreqTest_Omega*t)*pSharedMemory->WB_FreqTest_Amp;
		//dpCOM_3x1[2] = (float)sinf(pSharedMemory->WB_FreqTest_Omega*t)*pSharedMemory->WB_FreqTest_Amp;
		t += DT;
	}

	_log_temp[0] = _Q_34x1[1];
	_log_temp[1] = _Q_34x1[2];
	_log_temp[2] = _Q_34x1[3];
	_log_temp[3] = _pZMP_3x1[1];
	_log_temp[4] = _pZMP_3x1[2];
	_log_temp[5] = _copRF_3x1[1];
	_log_temp[6] = _copRF_3x1[2];
	_log_temp[7] = _copLF_3x1[1];
	_log_temp[8] = _copLF_3x1[2];
	_log_temp[9] = _pCOM_3x1[1];
	_log_temp[10] = _pCOM_3x1[2];
	_log_temp[11] = _pCOM_3x1[3];
	_log_temp[12] = _pRF_3x1[1];
	_log_temp[13] = _pRF_3x1[2];
	_log_temp[14] = _pRF_3x1[3];
	_log_temp[15] = _pLF_3x1[1];
	_log_temp[16] = _pLF_3x1[2];
	_log_temp[17] = _pLF_3x1[3];

	des_WST = 0.f;
	des_WSTp = 0.f;
	des_qPEL_4x1[1] = 1.f;
	des_qPEL_4x1[2] = 0.f;
	des_qPEL_4x1[3] = 0.f;
	des_qPEL_4x1[4] = 0.f;

	des_pPELz = pPELz1 + dpPELz;

	des_pCOM_3x1[1] = pCOM1_3x1[1] + dpCOM_3x1[1];
	des_pCOM_3x1[2] = pCOM1_3x1[2] + dpCOM_3x1[2];
	des_pCOM_3x1[3] = pCOM1_3x1[2] + dpCOM_3x1[3];

	des_pRF_3x1[1] = pRF1_3x1[1] + dpRF_3x1[1];
	des_pRF_3x1[2] = pRF1_3x1[2] + dpRF_3x1[2];
	des_pRF_3x1[3] = pRF1_3x1[3] + dpRF_3x1[3];
	
	des_pLF_3x1[1] = pLF1_3x1[1] + dpLF_3x1[1];
	des_pLF_3x1[2] = pLF1_3x1[2] + dpLF_3x1[2];
	des_pLF_3x1[3] = pLF1_3x1[3] + dpLF_3x1[3];
	
	des_pRH_3x1[1] = pRH1_3x1[1] + dpRH_3x1[1];
	des_pRH_3x1[2] = pRH1_3x1[2] + dpRH_3x1[2];
	des_pRH_3x1[3] = pRH1_3x1[3] + dpRH_3x1[3];

	des_pLH_3x1[1] = pLH1_3x1[1] + dpLH_3x1[1];
	des_pLH_3x1[2] = pLH1_3x1[2] + dpLH_3x1[2];
	des_pLH_3x1[3] = pLH1_3x1[3] + dpLH_3x1[3];

	QTcross(dqRH_4x1, qRH1_4x1, des_qRH_4x1);
	QTcross(dqLH_4x1, qLH1_4x1, des_qLH_4x1);
	QTcross(dqRF_4x1, qRF1_4x1, des_qRF_4x1);
	QTcross(dqLF_4x1, qLF1_4x1, des_qLF_4x1);
		

	if(reset_neutral_flag==1)
	{
		pPELz1 = des_pPELz;

		pCOM1_3x1[1] = des_pCOM_3x1[1];
		pCOM1_3x1[2] = des_pCOM_3x1[2];

		pRF1_3x1[1] = des_pRF_3x1[1];
		pRF1_3x1[2] = des_pRF_3x1[2];
		pRF1_3x1[3] = des_pRF_3x1[3];

		pLF1_3x1[1] = des_pLF_3x1[1];
		pLF1_3x1[2] = des_pLF_3x1[2];
		pLF1_3x1[3] = des_pLF_3x1[3];

		pRH1_3x1[1] = des_pRH_3x1[1];
		pRH1_3x1[2] = des_pRH_3x1[2];
		pRH1_3x1[3] = des_pRH_3x1[3];

		pLH1_3x1[1] = des_pLH_3x1[1];
		pLH1_3x1[2] = des_pLH_3x1[2];
		pLH1_3x1[3] = des_pLH_3x1[3];

		qRF1_4x1[1] = des_qRF_4x1[1];
		qRF1_4x1[2] = des_qRF_4x1[2];
		qRF1_4x1[3] = des_qRF_4x1[3];
		qRF1_4x1[4] = des_qRF_4x1[4];

		qLF1_4x1[1] = des_qLF_4x1[1];
		qLF1_4x1[2] = des_qLF_4x1[2];
		qLF1_4x1[3] = des_qLF_4x1[3];
		qLF1_4x1[4] = des_qLF_4x1[4];

		qRH1_4x1[1] = des_qRH_4x1[1];
		qRH1_4x1[2] = des_qRH_4x1[2];
		qRH1_4x1[3] = des_qRH_4x1[3];
		qRH1_4x1[4] = des_qRH_4x1[4];

		qLH1_4x1[1] = des_qLH_4x1[1];
		qLH1_4x1[2] = des_qLH_4x1[2];
		qLH1_4x1[3] = des_qLH_4x1[3];
		qLH1_4x1[4] = des_qLH_4x1[4];

		printf("\n move done");
		reset_neutral_flag = 0;
	}

	des_vPELz = 0.f;

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
	diff_vv(des_pRH_3x1,3, _pRH_3x1, ftemp3_7x1);
	mult_mv((const float**)_jRH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
	diff_vv(des_vRH_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[14], ftemp3_7x1,3, pSharedMemory->kd[14],ftemp1_7x1, ftemp3_7x1);
	mult_mv((const float**)_jRHp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xrh_6x1);

	QTdel(des_qRH_4x1, _qRH_4x1, ftemp3_7x1);	
	diff_vv(des_wRH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(pSharedMemory->kp[14],ftemp3_7x1,3, pSharedMemory->kd[14],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xrh_6x1[3]);
	
	diff_vv(des_pLH_3x1,3, _pLH_3x1, ftemp3_7x1);  
	mult_mv((const float**)_jLH_6x33,6,33, _Qp_33x1, ftemp2_7x1);  
	diff_vv(des_vLH_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[14], ftemp3_7x1,3, pSharedMemory->kd[14],ftemp1_7x1, ftemp3_7x1);  
	mult_mv((const float**)_jLHp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	  
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xlh_6x1);

	QTdel(des_qLH_4x1, _qLH_4x1, ftemp3_7x1);	
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
	//X1_33x1[28] = Xpel_3x1[1];
	//X1_33x1[29] = Xpel_3x1[2];
	//X1_33x1[30] = Xpel_3x1[3];
	dim_primary_task = 27;

	//----------------- redundant tasks
	RVALS(_Q_34x1[WST_34],  _Qp_33x1[WST_33],  pSharedMemory->kd[15]*(-_Qp_33x1[WST_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[WST_34]-_Q_34x1[WST_34]),		WSTpmax, WSTppmax, WSTmin, WSTmax, D2R, &X2_33x1[1], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RSP_34],  _Qp_33x1[RSP_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RSP_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RSP_34]-_Q_34x1[RSP_34]),   RSPpmax, RSPppmax, RSPmin, RSPmax, D2R, &X2_33x1[2], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RSR_34],  _Qp_33x1[RSR_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RSR_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RSR_34]-_Q_34x1[RSR_34]),   RSRpmax, RSRppmax, RSRmin, RSRmax, D2R, &X2_33x1[3], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RSY_34],  _Qp_33x1[RSY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RSY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RSY_34]-_Q_34x1[RSY_34]),   RSYpmax, RSYppmax, RSYmin, RSYmax, D2R, &X2_33x1[4], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[REB_34],  _Qp_33x1[REB_33],  pSharedMemory->kd[15]*(-_Qp_33x1[REB_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[REB_34]-_Q_34x1[REB_34]),   REBpmax, REBppmax, REBmin, REBmax, D2R, &X2_33x1[5], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RWY_34],  _Qp_33x1[RWY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RWY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RWY_34]-_Q_34x1[RWY_34]),   RWYpmax, RWYppmax, RWYmin, RWYmax, D2R, &X2_33x1[6], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RWP_34],  _Qp_33x1[RWP_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RWP_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RWP_34]-_Q_34x1[RWP_34]),   RWPpmax, RWPppmax, RWPmin, RWPmax, D2R, &X2_33x1[7], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSP_34],  _Qp_33x1[LSP_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LSP_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LSP_34]-_Q_34x1[LSP_34]),   LSPpmax, LSPppmax, LSPmin, LSPmax, D2R, &X2_33x1[8], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSR_34],  _Qp_33x1[LSR_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LSR_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LSR_34]-_Q_34x1[LSR_34]),   LSRpmax, LSRppmax, LSRmin, LSRmax, D2R, &X2_33x1[9], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSY_34],  _Qp_33x1[LSY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LSY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LSY_34]-_Q_34x1[LSY_34]),   LSYpmax, LSYppmax, LSYmin, LSYmax, D2R, &X2_33x1[10], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LEB_34],  _Qp_33x1[LEB_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LEB_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LEB_34]-_Q_34x1[LEB_34]),   LEBpmax, LEBppmax, LEBmin, LEBmax, D2R, &X2_33x1[11], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LWY_34],  _Qp_33x1[LWY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LWY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LWY_34]-_Q_34x1[LWY_34]),   LWYpmax, LWYppmax, LWYmin, LWYmax, D2R, &X2_33x1[12], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LWP_34],  _Qp_33x1[LWP_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LWP_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LWP_34]-_Q_34x1[LWP_34]),   LWPpmax, LWPppmax, LWPmin, LWPmax, D2R, &X2_33x1[13], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RWY2_34],  _Qp_33x1[RWY2_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RWY2_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RWY2_34]-_Q_34x1[RWY2_34]),   RWY2pmax, RWY2ppmax, RWY2min, RWY2max, D2R, &X2_33x1[14], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LWY2_34],  _Qp_33x1[LWY2_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LWY2_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LWY2_34]-_Q_34x1[LWY2_34]),   LWY2pmax, LWY2ppmax, LWY2min, LWY2max, D2R, &X2_33x1[15], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	X2_33x1[16] = Xpel_3x1[1];
	X2_33x1[17] = Xpel_3x1[2];
	X2_33x1[18] = Xpel_3x1[3];
	dim_redundant_task = 18;
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
		//_jT1_33x33[28][i] = 0.f;
		//_jT1_33x33[29][i] = 0.f;
		//_jT1_33x33[30][i] = 0.f;   
		for(j=1;j<=18;j++)
			_jT2_33x33[j][i] = 0.f;
	}
	_jT1_33x33[27][3] = 1.f;
  //_jT1_33x33[28][4] = 1.f;
  //_jT1_33x33[29][5] = 1.f;
  //_jT1_33x33[30][6] = 1.f;

	for(i=1; i<=27; i++)
	{
		_jT1_33x33[i][4] = 0.f;  // exclude pelvis orientation from the solution
		_jT1_33x33[i][5] = 0.f; 
		_jT1_33x33[i][6] = 0.f; 
	}

	for(i=13; i<=24; i++)
	{
		_jT1_33x33[i][7] = 0.f;  // exclude WST from the solution
	}

	for(j=20; j<=33; j++)
	{
		_jT1_33x33[25][j] = 0.f;
		_jT1_33x33[26][j] = 0.f;
	}
	_jT1_33x33[25][7] = 0.f;
	_jT1_33x33[26][7] = 0.f;

	_jT2_33x33[1][WST_33] = 1.f;
	_jT2_33x33[2][RSP_33] = 1.f;
	_jT2_33x33[3][RSR_33] = 1.f;
	_jT2_33x33[4][RSY_33] = 1.f;
	_jT2_33x33[5][REB_33] = 1.f;
	_jT2_33x33[6][RWY_33] = 1.f;
	_jT2_33x33[7][RWP_33] = 1.f;
	_jT2_33x33[8][LSP_33] = 1.f;
	_jT2_33x33[9][LSR_33] = 1.f;	
	_jT2_33x33[10][LSY_33] = 1.f;
	_jT2_33x33[11][LEB_33] = 1.f;
	_jT2_33x33[12][LWY_33] = 1.f;
	_jT2_33x33[13][LWP_33] = 1.f;
	_jT2_33x33[14][RWY2_33] = 1.f;
	_jT2_33x33[15][LWY2_33] = 1.f;	
	_jT2_33x33[16][WX_33] = 1.f;
	_jT2_33x33[17][WY_33] = 1.f;
	_jT2_33x33[18][WZ_33] = 1.f;
	
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

	//pinv_svd((const float**)_jT2_33x33,18,33, _jT2inv_33x33);
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

	do
	{
		isLimited = 0;
		if(pSharedMemory->torque_mode_flag == 0 || pSharedMemory->torque_mode_flag == 3)
			for(i=1;i<=27;i++)
			{
				index_limited[i] = 0;
				qpp_limited[i] = 0.f;
				if(Qpp_33x1[i+6] >= ub_27x1[i])
				{
					isLimited = 1;
					index_limited[i] = 1;
					qpp_limited[i] = ub_27x1[i];
				}
				else if(Qpp_33x1[i+6] <= lb_27x1[i])
				{
					isLimited = 1;
					index_limited[i] = 1;
					qpp_limited[i] = lb_27x1[i];
				}

				if(isLimited==1)
				{
					printf("\n i: %d, joint: %d, p: %.4fdeg, v: %.4fdeg/s, a: %.4fdeg/s^2", _offline_traj.i, i+6, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D, Qpp_33x1[i+6]*R2D);
					printf("\n ub: %.4fdeg/s^2, lb: %.4fdeg/s^2", ub_27x1[i]*R2D, lb_27x1[i]*R2D);			
					return -2;
				}

				if(fabs(_Qp_33x1[i+6])>=QP_MAX)
				{
					printf("\n i: %d, joint: %d, p: %.4fdeg, v: %.4fdeg/s, a: %.4fdeg/s^2", _offline_traj.i, i+6, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D, Qpp_33x1[i+6]*R2D);
					printf("\n ub: %.4fdeg/s^2, lb: %.4fdeg/s^2", ub_27x1[i]*R2D, lb_27x1[i]*R2D);			
					return -2;
				}
			}
		
		if(isLimited==1)
		{
			Qpp_33x1[WX_33] /= WEIGHT_PEL_SQRT_INV;
			Qpp_33x1[WY_33] /= WEIGHT_PEL_SQRT_INV;
			Qpp_33x1[WZ_33] /= WEIGHT_PEL_SQRT_INV;
			Qpp_33x1[WST_33] /= WEIGHT_WST_SQRT_INV;

			k=1;
			for(i=1;i<=27;i++)
			{
				if(index_limited[i] == 1)
				{
					for(j=1;j<=33;j++)
						_jT1_33x33[dim_primary_task+k][j] = 0;
					_jT1_33x33[dim_primary_task+k][i+6] = 1;
					X1_33x1[dim_primary_task+k] = qpp_limited[i];
					k++;
				}
			}
			if(dim_primary_task+k-1>33)
				return -2;  // no feasible solution

			if(pinv_SR((const float**)_jT1_33x33, dim_primary_task+k-1, 33, (float)1.e-10, _jT1inv_33x33) != 0)
				return -2; // singularity occurred
			mult_mm((const float**)_jT1inv_33x33,33,dim_primary_task+k-1, (const float**)_jT1_33x33, 33, _TEMP1_34x34);
			diff_mm((const float**)_EYE_33, 33,33, (const float**)_TEMP1_34x34, _N1_33x33);	

			mult_mv((const float**)_jT1inv_33x33,33,dim_primary_task+k-1, (const float*)X1_33x1, _TEMP1_34x34[1]);
			mult_mv((const float**)_jT2inv_33x33,33,18, (const float*)X2_33x1, _TEMP2_34x34[1]);
			mult_mv((const float**)_N1_33x33,33,33, (const float*)_TEMP2_34x34[1], Qpp_33x1);
			sum_vv((const float*)_TEMP1_34x34[1], 33, Qpp_33x1, Qpp_33x1);
			Qpp_33x1[WX_33] *= WEIGHT_PEL_SQRT_INV;
			Qpp_33x1[WY_33] *= WEIGHT_PEL_SQRT_INV;
			Qpp_33x1[WZ_33] *= WEIGHT_PEL_SQRT_INV;
			Qpp_33x1[WST_33] *= WEIGHT_WST_SQRT_INV;			
		}
	} while(isLimited==1);
	
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
	_Q_34x1[RWY2_34] += _Qp_33x1[RWY2_33]*DT + 0.5f*Qpp_33x1[32]*DT*DT;
	_Q_34x1[LWY2_34] += _Qp_33x1[LWY2_33]*DT + 0.5f*Qpp_33x1[33]*DT*DT;
	
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
	_Qp_33x1[RWY2_33] += DT*Qpp_33x1[32];
	_Qp_33x1[LWY2_33] += DT*Qpp_33x1[33];

	Joint[WST].RefAngleCurrent = _Q_34x1[WST_34]*R2D;

	Joint[RHY].RefAngleCurrent = (_Q_34x1[RHY_34]+rhy_compen)*R2D;
	Joint[RHR].RefAngleCurrent = (_Q_34x1[RHR_34]+rhr_compen)*R2D;
	Joint[RHP].RefAngleCurrent = _Q_34x1[RHP_34]*R2D;
	Joint[RKN].RefAngleCurrent = _Q_34x1[RKN_34]*R2D;
	Joint[RAP].RefAngleCurrent = _Q_34x1[RAP_34]*R2D;
	Joint[RAR].RefAngleCurrent = _Q_34x1[RAR_34]*R2D;
	Joint[LHY].RefAngleCurrent = (_Q_34x1[LHY_34]+lhy_compen)*R2D;
	Joint[LHR].RefAngleCurrent = (_Q_34x1[LHR_34]+lhr_compen)*R2D;
	Joint[LHP].RefAngleCurrent = _Q_34x1[LHP_34]*R2D;
	Joint[LKN].RefAngleCurrent = _Q_34x1[LKN_34]*R2D;
	Joint[LAP].RefAngleCurrent = _Q_34x1[LAP_34]*R2D;
	Joint[LAR].RefAngleCurrent = _Q_34x1[LAR_34]*R2D;

	Joint[RSP].RefAngleCurrent = _Q_34x1[RSP_34]*R2D;
	Joint[RSR].RefAngleCurrent = _Q_34x1[RSR_34]*R2D - OFFSET_RSR;
	Joint[RSY].RefAngleCurrent = _Q_34x1[RSY_34]*R2D;
	Joint[REB].RefAngleCurrent = _Q_34x1[REB_34]*R2D - OFFSET_REB;
	Joint[RWY].RefAngleCurrent = _Q_34x1[RWY_34]*R2D;
	Joint[RWP].RefAngleCurrent = _Q_34x1[RWP_34]*R2D;
	Joint[RWY2].RefAngleCurrent = _Q_34x1[RWY2_34]*R2D;
	Joint[LSP].RefAngleCurrent = _Q_34x1[LSP_34]*R2D;
	Joint[LSR].RefAngleCurrent = _Q_34x1[LSR_34]*R2D - OFFSET_LSR;
	Joint[LSY].RefAngleCurrent = _Q_34x1[LSY_34]*R2D;
	Joint[LEB].RefAngleCurrent = _Q_34x1[LEB_34]*R2D - OFFSET_LEB;	
	Joint[LWY].RefAngleCurrent = _Q_34x1[LWY_34]*R2D;
	Joint[LWP].RefAngleCurrent = _Q_34x1[LWP_34]*R2D;
	Joint[LWY2].RefAngleCurrent = _Q_34x1[LWY2_34]*R2D;

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

	return 0;
}



int get_steer(float p1_3x1[], float p2_3x1[], float p3_3x1[], float pcSteer_3x1[], float *rSteer, float qSteer_4x1[])
{
	float temp;
	
	_TEMP1_34x34[1][1] = 2.f*(p2_3x1[1]-p1_3x1[1]);
	_TEMP1_34x34[1][2] = 2.f*(p2_3x1[2]-p1_3x1[2]);
	_TEMP1_34x34[1][3] = 2.f*(p2_3x1[3]-p1_3x1[3]);

	_TEMP1_34x34[2][1] = 2.f*(p3_3x1[1]-p1_3x1[1]);
	_TEMP1_34x34[2][2] = 2.f*(p3_3x1[2]-p1_3x1[2]);
	_TEMP1_34x34[2][3] = 2.f*(p3_3x1[3]-p1_3x1[3]);

	diff_vv(p2_3x1,3,p1_3x1,_TEMP1_34x34[5]);
	diff_vv(p3_3x1,3,p1_3x1,_TEMP1_34x34[6]);
	cross(1.f, (const float*)_TEMP1_34x34[5], (const float*)_TEMP1_34x34[6], _TEMP1_34x34[3]);

	_TEMP1_34x34[4][1] = -(p1_3x1[1]*p1_3x1[1]+p1_3x1[2]*p1_3x1[2]+p1_3x1[3]*p1_3x1[3]) + (p2_3x1[1]*p2_3x1[1]+p2_3x1[2]*p2_3x1[2]+p2_3x1[3]*p2_3x1[3]);
	_TEMP1_34x34[4][2] = -(p1_3x1[1]*p1_3x1[1]+p1_3x1[2]*p1_3x1[2]+p1_3x1[3]*p1_3x1[3]) + (p3_3x1[1]*p3_3x1[1]+p3_3x1[2]*p3_3x1[2]+p3_3x1[3]*p3_3x1[3]);
	_TEMP1_34x34[4][3] = _TEMP1_34x34[3][1]*p1_3x1[1] + _TEMP1_34x34[3][2]*p1_3x1[2] + _TEMP1_34x34[3][3]*p1_3x1[3];
	
	if(gaussj_mod(_TEMP1_34x34,3,_TEMP1_34x34[4]) != 0)
		return -1; // singularity
	pcSteer_3x1[1] = _TEMP1_34x34[4][1];
	pcSteer_3x1[2] = _TEMP1_34x34[4][2];
	pcSteer_3x1[3] = _TEMP1_34x34[4][3];

	diff_vv(p1_3x1,3, pcSteer_3x1, _TEMP1_34x34[1]);
	diff_vv(p2_3x1,3, pcSteer_3x1, _TEMP1_34x34[2]);
	diff_vv(p3_3x1,3, pcSteer_3x1, _TEMP1_34x34[3]);
	*rSteer = norm_v(_TEMP1_34x34[1],3);

	if(*rSteer < 0.05f || *rSteer > 0.3f)
		return -2;

	cross(1.f, (const float*)_TEMP1_34x34[1], (const float*)_TEMP1_34x34[3], _TEMP1_34x34[4]);
	if(_TEMP1_34x34[4][3] < 0.f)
		cross(1.f, (const float*)_TEMP1_34x34[3], (const float*)_TEMP1_34x34[1], _TEMP1_34x34[4]);

	temp = norm_v(_TEMP1_34x34[4],3);
	_TEMP1_34x34[4][1] /= temp;
	_TEMP1_34x34[4][2] /= temp;
	_TEMP1_34x34[4][3] /= temp;

	_TEMP1_34x34[5][1] = _TEMP1_34x34[4][1];
	_TEMP1_34x34[5][2] = _TEMP1_34x34[4][2];
	_TEMP1_34x34[5][3] = 0.f;
	temp = norm_v(_TEMP1_34x34[5],3);
	_TEMP1_34x34[5][1] /= temp;
	_TEMP1_34x34[5][2] /= temp;
	_TEMP1_34x34[5][3] /= temp;
	RZ(-PI*0.5f, _TEMP2_34x34);
	mult_mv((const float**)_TEMP2_34x34,3,3, (const float*)_TEMP1_34x34[5], _TEMP1_34x34[6]);
	cross(1.f, _TEMP1_34x34[6], _TEMP1_34x34[4], _TEMP1_34x34[7]);
	
	_TEMP3_34x34[1][1] = _TEMP1_34x34[7][1];
	_TEMP3_34x34[2][1] = _TEMP1_34x34[7][2];
	_TEMP3_34x34[3][1] = _TEMP1_34x34[7][3];
	_TEMP3_34x34[1][2] = _TEMP1_34x34[6][1];
	_TEMP3_34x34[2][2] = _TEMP1_34x34[6][2];
	_TEMP3_34x34[3][2] = _TEMP1_34x34[6][3];
	_TEMP3_34x34[1][3] = _TEMP1_34x34[4][1];
	_TEMP3_34x34[2][3] = _TEMP1_34x34[4][2];
	_TEMP3_34x34[3][3] = _TEMP1_34x34[4][3];
	DC2QT((const float**)_TEMP3_34x34, qSteer_4x1);

	return 0;
}


int get_steerAng(float qSteer_4x1[], float qLH_4x1[], float *angSteer)
{
	QT2DC(qSteer_4x1, _TEMP1_34x34);
	trans(1.f, (const float**)_TEMP1_34x34,3,3, _TEMP2_34x34);
	QT2DC(qLH_4x1, _TEMP3_34x34);
	mult_mm((const float**)_TEMP2_34x34,3,3, (const float**)_TEMP3_34x34,3, _TEMP4_34x34);
	*angSteer = (float)atan2(_TEMP4_34x34[2][1], _TEMP1_34x34[1][1])*R2D + 90;
	return 0;
}


int GetZMP_WB(FT rf, FT lf, FT rw, FT lw)
{
	// zmp_3x1[1]: ZMPx w.r.t pPC in the global frame
	// zmp_3x1[2]: ZMPy w.r.t pPC in the global frame
	// zmp_3x1[3]: ZMPz w.r.t pPC in the global frame

	// copRF_3x1 : Center of Pressure w.r.t pRF in the RF frame
	// copLF_3x1 : Center of Pressure w.r.t pLF in the LF frame
	
	float mRF_3x1[4] = {0., rf.Mx, rf.My, 0.};
	float mLF_3x1[4] = {0., lf.Mx, lf.My, 0.};
	float fRF_3x1[4] = {0., 0., 0., rf.Fz};
	float fLF_3x1[4] = {0., 0., 0., lf.Fz};
	float mTOT_3x1[4] = {0., 0., 0., 0.};
	float fRS_local_3x1[4] = {0., -rw.My/0.095f, rw.Mx/0.095f, rw.Fz};
	float fLS_local_3x1[4] = {0., -lw.My/0.095f, lw.Mx/0.095f, lw.Fz};

	float **temp_3x3;
	
	float temp_3x1[4];

	if(pSharedMemory->temp_fHand_threshold < 10.f)
		pSharedMemory->temp_fHand_threshold = 10.f;

	if(_FKineUpdatedFlag != 1)
		return -2;

	temp_3x3 = matrix(1,3,1,3);

	if(norm_v(fRS_local_3x1,3) < 0.01f)
	{
		fRS_local_3x1[1] = 0.f;
		fRS_local_3x1[2] = 0.f;
		fRS_local_3x1[3] = 0.f;
	}
	if(norm_v(fLS_local_3x1,3) < 0.01f)
	{
		fLS_local_3x1[1] = 0.f;
		fLS_local_3x1[2] = 0.f;
		fLS_local_3x1[3] = 0.f;
	}
	QT2DC(_qRS_4x1, temp_3x3);
	mult_mv((const float**)temp_3x3,3,3, fRS_local_3x1, _fRS_3x1);
	QT2DC(_qLS_4x1, temp_3x3);
	mult_mv((const float**)temp_3x3,3,3, fLS_local_3x1, _fLS_3x1);

//	pSharedMemory->WB_fRS[0] = _fRS_3x1[1];
//	pSharedMemory->WB_fRS[1] = _fRS_3x1[2];
//	pSharedMemory->WB_fRS[2] = _fRS_3x1[3];

//	pSharedMemory->WB_fLS[0] = _fLS_3x1[1];
//	pSharedMemory->WB_fLS[1] = _fLS_3x1[2];
//	pSharedMemory->WB_fLS[2] = _fLS_3x1[3];

	if(fRF_3x1[3] > pSharedMemory->temp_fHand_threshold)
	{			
		if(fLF_3x1[3]> pSharedMemory->temp_fHand_threshold)		// Double Support Phase
		{
			_copRF_3x1[1] = -mRF_3x1[2]/fRF_3x1[3];
			_copRF_3x1[2] = mRF_3x1[1]/fRF_3x1[3];
			_copRF_3x1[3] = 0.;

			_copLF_3x1[1] = -mLF_3x1[2]/fLF_3x1[3];
			_copLF_3x1[2] = mLF_3x1[1]/fLF_3x1[3];
			_copLF_3x1[3] = 0.;

			mult_mv((const float**)_dcRF_3x3,3,3, mRF_3x1, temp_3x1);
			mRF_3x1[1] = temp_3x1[1];
			mRF_3x1[2] = temp_3x1[2];
			mRF_3x1[3] = temp_3x1[3];

			mult_mv((const float**)_dcRF_3x3,3,3, fRF_3x1, temp_3x1);
			fRF_3x1[1] = temp_3x1[1];
			fRF_3x1[2] = temp_3x1[2];
			fRF_3x1[3] = temp_3x1[3];

			mult_mv((const float**)_dcLF_3x3,3,3, mLF_3x1, temp_3x1);
			mLF_3x1[1] = temp_3x1[1];
			mLF_3x1[2] = temp_3x1[2];
			mLF_3x1[3] = temp_3x1[3];

			mult_mv((const float**)_dcLF_3x3,3,3, fLF_3x1, temp_3x1);
			fLF_3x1[1] = temp_3x1[1];
			fLF_3x1[2] = temp_3x1[2];
			fLF_3x1[3] = temp_3x1[3];

			mTOT_3x1[1] = mRF_3x1[1] + mLF_3x1[1];
			mTOT_3x1[2] = mRF_3x1[2] + mLF_3x1[2];
			mTOT_3x1[3] = mRF_3x1[3] + mLF_3x1[3];

			cross(1.f, _pRF_3x1, fRF_3x1, temp_3x1);
			mTOT_3x1[1] += temp_3x1[1];
			mTOT_3x1[2] += temp_3x1[2];
			mTOT_3x1[3] += temp_3x1[3];

			cross(1.f, _pLF_3x1, fLF_3x1, temp_3x1);
			mTOT_3x1[1] += temp_3x1[1];
			mTOT_3x1[2] += temp_3x1[2];
			mTOT_3x1[3] += temp_3x1[3];	

			/*if(_DRC_walking_mode == DRC_QUAD_MODE)
			{
				cross(1.f, _pRH_3x1, _fRS_3x1, temp_3x1);
				mTOT_3x1[1] += temp_3x1[1];
				mTOT_3x1[2] += temp_3x1[2];
				mTOT_3x1[3] += temp_3x1[3];

				cross(1.f, _pLH_3x1, _fLS_3x1, temp_3x1);
				mTOT_3x1[1] += temp_3x1[1];
				mTOT_3x1[2] += temp_3x1[2];
				mTOT_3x1[3] += temp_3x1[3];

				_pZMP_3x1[1] = -mTOT_3x1[2]/(fRF_3x1[3]+fLF_3x1[3]+_fRS_3x1[3]+_fLS_3x1[3]) - _Q_30x1[1];
				_pZMP_3x1[2] = mTOT_3x1[1]/(fRF_3x1[3]+fLF_3x1[3]+_fRS_3x1[3]+_fLS_3x1[3]) - _Q_30x1[2];
				_pZMP_3x1[3] = 0.;						
			}
			else*/
			{
				_pZMP_3x1[1] = -mTOT_3x1[2]/(fRF_3x1[3]+fLF_3x1[3]) - _Q_34x1[1];
				_pZMP_3x1[2] = mTOT_3x1[1]/(fRF_3x1[3]+fLF_3x1[3]) - _Q_34x1[2];
				_pZMP_3x1[3] = 0.;						
			}
		}
		else	// RF Single Support Phase
		{
			_copRF_3x1[1] = -mRF_3x1[2]/fRF_3x1[3];
			_copRF_3x1[2] = mRF_3x1[1]/fRF_3x1[3];
			_copRF_3x1[3] = 0.;

			_copLF_3x1[1] = 0.;
			_copLF_3x1[2] = 0.;
			_copLF_3x1[3] = 0.;

			mult_mv((const float**)_dcRF_3x3,3,3, mRF_3x1, temp_3x1);
			mRF_3x1[1] = temp_3x1[1];
			mRF_3x1[2] = temp_3x1[2];
			mRF_3x1[3] = temp_3x1[3];

			mult_mv((const float**)_dcRF_3x3,3,3, fRF_3x1, temp_3x1);
			fRF_3x1[1] = temp_3x1[1];
			fRF_3x1[2] = temp_3x1[2];
			fRF_3x1[3] = temp_3x1[3];			

			mTOT_3x1[1] = mRF_3x1[1];
			mTOT_3x1[2] = mRF_3x1[2];
			mTOT_3x1[3] = mRF_3x1[3];

			cross(1.f, _pRF_3x1, fRF_3x1, temp_3x1);
			mTOT_3x1[1] += temp_3x1[1];
			mTOT_3x1[2] += temp_3x1[2];
			mTOT_3x1[3] += temp_3x1[3];
	
			/*if(_DRC_walking_mode == DRC_QUAD_MODE)
			{
				cross(1.f, _pRH_3x1, _fRS_3x1, temp_3x1);
				mTOT_3x1[1] += temp_3x1[1];
				mTOT_3x1[2] += temp_3x1[2];
				mTOT_3x1[3] += temp_3x1[3];

				cross(1.f, _pLH_3x1, _fLS_3x1, temp_3x1);
				mTOT_3x1[1] += temp_3x1[1];
				mTOT_3x1[2] += temp_3x1[2];
				mTOT_3x1[3] += temp_3x1[3];

				_pZMP_3x1[1] = -mTOT_3x1[2]/(fRF_3x1[3]+_fRS_3x1[3]+_fLS_3x1[3]) - _Q_30x1[1];
				_pZMP_3x1[2] = mTOT_3x1[1]/(fRF_3x1[3]+_fRS_3x1[3]+_fLS_3x1[3]) - _Q_30x1[2];
				_pZMP_3x1[3] = 0.;
			}
			else*/
			{
				_pZMP_3x1[1] = -mTOT_3x1[2]/fRF_3x1[3] - _Q_34x1[1];
				_pZMP_3x1[2] = mTOT_3x1[1]/fRF_3x1[3] - _Q_34x1[2];
				_pZMP_3x1[3] = 0.;
			}
		}			
	}
	else if(fLF_3x1[3] > pSharedMemory->temp_fHand_threshold)	// LF Single Support Phase
	{
		_copRF_3x1[1] = 0.;
		_copRF_3x1[2] = 0.;
		_copRF_3x1[3] = 0.;

		_copLF_3x1[1] = -mLF_3x1[2]/fLF_3x1[3];
		_copLF_3x1[2] = mLF_3x1[1]/fLF_3x1[3];
		_copLF_3x1[3] = 0.;

		mult_mv((const float**)_dcRF_3x3,3,3, mLF_3x1, temp_3x1);
		mLF_3x1[1] = temp_3x1[1];
		mLF_3x1[2] = temp_3x1[2];
		mLF_3x1[3] = temp_3x1[3];

		mult_mv((const float**)_dcRF_3x3,3,3, fLF_3x1, temp_3x1);
		fLF_3x1[1] = temp_3x1[1];
		fLF_3x1[2] = temp_3x1[2];
		fLF_3x1[3] = temp_3x1[3];

		mTOT_3x1[1] = mLF_3x1[1];
		mTOT_3x1[2] = mLF_3x1[2];
		mTOT_3x1[3] = mLF_3x1[3];

		cross(1.f, _pLF_3x1, fLF_3x1, temp_3x1);
		mTOT_3x1[1] += temp_3x1[1];
		mTOT_3x1[2] += temp_3x1[2];
		mTOT_3x1[3] += temp_3x1[3];	

		/*if(_DRC_walking_mode == DRC_QUAD_MODE)
		{
			cross(1.f, _pRH_3x1, _fRS_3x1, temp_3x1);
			mTOT_3x1[1] += temp_3x1[1];
			mTOT_3x1[2] += temp_3x1[2];
			mTOT_3x1[3] += temp_3x1[3];

			cross(1.f, _pLH_3x1, _fLS_3x1, temp_3x1);
			mTOT_3x1[1] += temp_3x1[1];
			mTOT_3x1[2] += temp_3x1[2];
			mTOT_3x1[3] += temp_3x1[3];

			_pZMP_3x1[1] = -mTOT_3x1[2]/(fRF_3x1[3]+_fRS_3x1[3]+_fLS_3x1[3]) - _Q_30x1[1];
			_pZMP_3x1[2] = mTOT_3x1[1]/(fRF_3x1[3]+_fRS_3x1[3]+_fLS_3x1[3]) - _Q_30x1[2];
			_pZMP_3x1[3] = 0.;
		}
		else*/
		{
			_pZMP_3x1[1] = -mTOT_3x1[2]/fLF_3x1[3] - _Q_34x1[1];
			_pZMP_3x1[2] = mTOT_3x1[1]/fLF_3x1[3] - _Q_34x1[2];
			_pZMP_3x1[3] = 0.;
		}
	}
	else 
	{
		_copRF_3x1[1] = 0.;
		_copRF_3x1[2] = 0.;
		_copRF_3x1[3] = 0.;

		_copLF_3x1[1] = 0.;
		_copLF_3x1[2] = 0.;
		_copLF_3x1[3] = 0.;

		_pZMP_3x1[1] = 0.;
		_pZMP_3x1[2] = 0.;
		_pZMP_3x1[3] = 0.;
	}

	if(fRF_3x1[3] > pSharedMemory->temp_fHand_threshold)
	{
		if(fLF_3x1[3] > pSharedMemory->temp_fHand_threshold)
			_FSP = DFSP;
		else
			_FSP = RFSP;
	}
	else
	{
		if(fLF_3x1[3] > pSharedMemory->temp_fHand_threshold)
			_FSP = LFSP;
		else
			_FSP = NO_LANDING;
	}

	if(fRF_3x1[3] > pSharedMemory->temp_fHand_threshold)
		_land_RF = 1;
	else
		_land_RF = 0;

	if(fLF_3x1[3] > pSharedMemory->temp_fHand_threshold)
		_land_LF = 1;
	else
		_land_LF = 0;

	if(_fRS_3x1[3] > pSharedMemory->temp_fHand_threshold)
	//if(norm_v(_fRS_3x1,3) > pSharedMemory->temp_fHand_threshold)
		_land_RH = 1;
	else
		_land_RH = 0;
	if(_fLS_3x1[3] > pSharedMemory->temp_fHand_threshold)
	//if(norm_v(_fLS_3x1,3) > pSharedMemory->temp_fHand_threshold)
		_land_LH = 1;
	else
		_land_LH = 0;


	_copRF_filt_2x1[1] = _copRF_filt_2x1[1] + DT*COP_CUTOFF*(_copRF_3x1[1] - _copRF_filt_2x1[1]);
	_copRF_filt_2x1[2] = _copRF_filt_2x1[2] + DT*COP_CUTOFF*(_copRF_3x1[2] - _copRF_filt_2x1[2]);
	_copLF_filt_2x1[1] = _copLF_filt_2x1[1] + DT*COP_CUTOFF*(_copLF_3x1[1] - _copLF_filt_2x1[1]);
	_copLF_filt_2x1[2] = _copLF_filt_2x1[2] + DT*COP_CUTOFF*(_copLF_3x1[2] - _copLF_filt_2x1[2]);

	free_matrix(temp_3x3,1,3,1,3);
	return 0;
}


float limitDuty(float limit, float duty)
{
	float sign;
	if(duty>=0.f)
		sign = 1.f;
	else 
		sign = -1.f;

	if(fabs(duty) > limit)
		return (sign*limit);
	else
		return duty;
}


int getGravityTorque(const float *Q_34x1, float *torque_33x1)
{
  float wst, rhy, rhp, rhr, rkn, rap, rar, lhy, lhp, lhr, lkn, lap, lar, rsp, rsr, rsy, reb, rwy, lsp, lsr, lsy, leb, lwy, rwp, lwp, rwy2, lwy2;
  float axis_rhy[4], axis_rhr[4], axis_rhp[4], axis_rkn[4], axis_rap[4], axis_rar[4];	 
  float axis_lhy[4], axis_lhr[4], axis_lhp[4], axis_lkn[4], axis_lap[4], axis_lar[4];	
  float axis_wst[4], axis_rsp[4], axis_rsr[4], axis_rsy[4], axis_reb[4], axis_rwy[4], axis_rwp[4], axis_rwy2[4];
  float axis_lsp[4], axis_lsr[4], axis_lsy[4], axis_leb[4], axis_lwy[4], axis_lwp[4], axis_lwy2[4];
  
  float pRPEL[4], pRKN[4], pRANK[4], pRF[4], pLPEL[4], pLKN[4], pLANK[4], pLF[4], pWST[4], pRSHLD[4], pRELB[4], pRWR[4], pRH[4], pLSHLD[4], pLELB[4], pLWR[4], pLH[4], pRELB2[4], pLELB2[4];
  float cPEL[4], cTOR[4], cRUL[4], cRLL[4], cRF[4], cLUL[4], cLLL[4], cLF[4], cRUA[4], cRLA[4], cLUA[4], cLLA[4], cRH[4], cLH[4];
  float temp3_3x1[4], temp4_3x1[4];
  
  const float qPEL[5] = {0., Q_34x1[4], Q_34x1[5], Q_34x1[6], Q_34x1[7]};
  const float pPC[4] = {0., Q_34x1[1], Q_34x1[2], Q_34x1[3]};

  int i;

  for(i=1; i<=33; i++)
	  torque_33x1[i] = 0.f;
  
  rhy = Q_34x1[9];
  rhr = Q_34x1[10];
  rhp = Q_34x1[11];
  rkn = Q_34x1[12];
  rap = Q_34x1[13];
  rar = Q_34x1[14];

  lhy = Q_34x1[15];
  lhr = Q_34x1[16];
  lhp = Q_34x1[17];
  lkn = Q_34x1[18];
  lap = Q_34x1[19];
  lar = Q_34x1[20];

  wst = Q_34x1[8];
  rsp = Q_34x1[21];
  rsr = Q_34x1[22];
  rsy = Q_34x1[23];
  reb = Q_34x1[24];
  rwy = Q_34x1[25];
  rwp = Q_34x1[26];
  lsp = Q_34x1[27];
  lsr = Q_34x1[28];
  lsy = Q_34x1[29];
  leb = Q_34x1[30];
  lwy = Q_34x1[31];
  lwp = Q_34x1[32];
  rwy2 = Q_34x1[33];
  lwy2 = Q_34x1[34];
  
  QT2DC(qPEL, _dcPEL_3x3);
	
  RZ(rhy, _Rz_RHY_3x3);
  RX(rhr, _Rx_RHR_3x3);
  RY(rhp, _Ry_RHP_3x3);
  RY(rkn, _Ry_RKN_3x3);
  RY(rap, _Ry_RAP_3x3);
  RX(rar, _Rx_RAR_3x3);
  
  RZ(lhy, _Rz_LHY_3x3);
  RX(lhr, _Rx_LHR_3x3);
  RY(lhp, _Ry_LHP_3x3);
  RY(lkn, _Ry_LKN_3x3);
  RY(lap, _Ry_LAP_3x3);
  RX(lar, _Rx_LAR_3x3);
  
  RZ(wst, _Rz_WST_3x3);
	
  RY(rsp, _Ry_RSP_3x3);
  RX(rsr, _Rx_RSR_3x3);
  RZ(rsy, _Rz_RSY_3x3);
  RY(reb, _Ry_REB_3x3);
  RZ(rwy, _Rz_RWY_3x3);
  RY(rwp, _Ry_RWP_3x3);
  
  RY(lsp, _Ry_LSP_3x3);
  RX(lsr, _Rx_LSR_3x3);
  RZ(lsy, _Rz_LSY_3x3);
  RY(leb, _Ry_LEB_3x3);
  RZ(lwy, _Rz_LWY_3x3);
  RY(lwp, _Ry_LWP_3x3);
  
  RZ(rwy2, _Rz_RWY2_3x3);
  RZ(lwy2, _Rz_LWY2_3x3);

  mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_34x34);
  mult_mm((const float**)_Rx_RHR_3x3,3,3, (const float**)_Ry_RHP_3x3,3, _TEMP2_34x34);
  mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcRUL_3x3);
  mult_mm((const float**)_dcRUL_3x3,3,3, (const float**)_Ry_RKN_3x3,3, _dcRLL_3x3);
  
  mult_mm((const float**)_Ry_RAP_3x3,3,3, (const float**)_Rx_RAR_3x3,3, _TEMP1_34x34);
  mult_mm((const float**)_dcRLL_3x3,3,3, (const float**)_TEMP1_34x34,3, _dcRF_3x3);
  mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_34x34);
  mult_mm((const float**)_Rx_LHR_3x3,3,3, (const float**)_Ry_LHP_3x3,3, _TEMP2_34x34);
  mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcLUL_3x3);
  mult_mm((const float**)_dcLUL_3x3,3,3, (const float**)_Ry_LKN_3x3,3, _dcLLL_3x3);
  
  mult_mm((const float**)_Ry_LAP_3x3,3,3, (const float**)_Rx_LAR_3x3,3, _TEMP1_34x34);
  mult_mm((const float**)_dcLLL_3x3,3,3, (const float**)_TEMP1_34x34,3, _dcLF_3x3);
  mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_WST_3x3,3, _dcTOR_3x3);
  mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_RSP_3x3,3, _TEMP1_34x34);
  mult_mm((const float**)_Rx_RSR_3x3,3,3, (const float**)_Rz_RSY_3x3,3, _TEMP2_34x34);
  mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcRUA_3x3);
  
  mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REB_3x3,3, _TEMP1_34x34);
  mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_RWY_3x3,3, _dcRLA_3x3);
  mult_mm((const float**)_dcRLA_3x3,3,3, (const float**)_Ry_RWP_3x3,3, _TEMP1_34x34);
  mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_RWY2_3x3,3, _dcRH_3x3);
  
  mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_LSP_3x3,3, _TEMP1_34x34);
  mult_mm((const float**)_Rx_LSR_3x3,3,3, (const float**)_Rz_LSY_3x3,3, _TEMP2_34x34);
  mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcLUA_3x3);
  
  mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEB_3x3,3, _TEMP1_34x34);
  mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_LWY_3x3,3, _dcLLA_3x3);
  mult_mm((const float**)_dcLLA_3x3,3,3, (const float**)_Ry_LWP_3x3,3, _TEMP1_34x34);
  mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_LWY2_3x3,3, _dcLH_3x3);
  
  mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_RPEL, temp3_3x1); 
  sum_vv(pPC,3, temp3_3x1, pRPEL);
  
  mult_mv((const float**)_dcRUL_3x3,3,3, _LINK_LEG, temp3_3x1);
  sum_vv(pRPEL,3, temp3_3x1, pRKN);
  
  mult_mv((const float**)_dcRLL_3x3,3,3, _LINK_LEG, temp3_3x1);
  sum_vv(pRKN,3, temp3_3x1, pRANK);
  
  mult_mv((const float**)_dcRF_3x3,3,3, _LINK_FOOT, temp3_3x1);
  sum_vv(pRANK,3, temp3_3x1, pRF);
  
  mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_LPEL, temp3_3x1); 
  sum_vv(pPC,3, temp3_3x1, pLPEL);
  
  mult_mv((const float**)_dcLUL_3x3,3,3, _LINK_LEG, temp3_3x1);
  sum_vv(pLPEL,3, temp3_3x1, pLKN);
  
  mult_mv((const float**)_dcLLL_3x3,3,3, _LINK_LEG, temp3_3x1);
  sum_vv(pLKN,3, temp3_3x1, pLANK);
  
  mult_mv((const float**)_dcLF_3x3,3,3, _LINK_FOOT, temp3_3x1);
  sum_vv(pLANK,3, temp3_3x1, pLF);
  
  
  mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_WST, temp3_3x1); 
  sum_vv(pPC,3, temp3_3x1, pWST);
  
  mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_RSHLD, temp3_3x1); 
  sum_vv(pWST,3, temp3_3x1, pRSHLD);
  
  mult_mv((const float**)_dcRUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
  sum_vv(pRSHLD,3, temp3_3x1, pRELB);
  
  temp4_3x1[1] = _LINK_LARM[1];
  temp4_3x1[2] = 0.;
  temp4_3x1[3] = 0.;
  mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REB_3x3,3, _TEMP1_34x34);
  mult_mv((const float**)_TEMP1_34x34,3,3, temp4_3x1, temp3_3x1); 
  sum_vv(pRELB,3, temp3_3x1, pRELB2);
  temp4_3x1[1] = 0.;
  temp4_3x1[2] = _LINK_LARM[2];
  temp4_3x1[3] = _LINK_LARM[3];
  mult_mv((const float**)_dcRLA_3x3,3,3, temp4_3x1, temp3_3x1); 
  sum_vv(pRELB2,3, temp3_3x1, pRWR);
  
  mult_mv((const float**)_dcRH_3x3,3,3, _LINK_HAND, temp3_3x1); 
  sum_vv(pRWR,3, temp3_3x1, pRH);
  mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_LSHLD, temp3_3x1); 
  sum_vv(pWST,3, temp3_3x1, pLSHLD);
  mult_mv((const float**)_dcLUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
  sum_vv(pLSHLD,3, temp3_3x1, pLELB);
  
  temp4_3x1[1] = _LINK_LARM[1];
  temp4_3x1[2] = 0.;
  temp4_3x1[3] = 0.;
  mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEB_3x3,3, _TEMP1_34x34);
  mult_mv((const float**)_TEMP1_34x34,3,3, temp4_3x1, temp3_3x1); 
  sum_vv(pLELB,3, temp3_3x1, pLELB2);
  temp4_3x1[1] = 0.;
  temp4_3x1[2] = _LINK_LARM[2];
  temp4_3x1[3] = _LINK_LARM[3];
  mult_mv((const float**)_dcLLA_3x3,3,3, temp4_3x1, temp3_3x1); 
  sum_vv(pLELB2,3, temp3_3x1, pLWR);
  
  mult_mv((const float**)_dcLH_3x3,3,3, _LINK_HAND, temp3_3x1); 
  sum_vv(pLWR,3, temp3_3x1, pLH);
  
  mult_mv((const float**)_dcPEL_3x3,3,3, _C_PEL, temp3_3x1);
  sum_vv(pPC,3, temp3_3x1, cPEL);
  
  mult_mv((const float**)_dcTOR_3x3,3,3, _C_TOR, temp3_3x1);
  sum_vv(pWST,3, temp3_3x1, cTOR);
  
  mult_mv((const float**)_dcRUL_3x3,3,3, _C_RUL, temp3_3x1);
  sum_vv(pRPEL,3, temp3_3x1, cRUL);
  
  mult_mv((const float**)_dcRLL_3x3,3,3, _C_RLL, temp3_3x1);
  sum_vv(pRKN,3, temp3_3x1, cRLL);
  
  mult_mv((const float**)_dcRF_3x3,3,3, _C_RF, temp3_3x1);
  sum_vv(pRANK,3, temp3_3x1, cRF);
  
  mult_mv((const float**)_dcLUL_3x3,3,3, _C_LUL, temp3_3x1);
  sum_vv(pLPEL,3, temp3_3x1, cLUL);
  
  mult_mv((const float**)_dcLLL_3x3,3,3, _C_LLL, temp3_3x1);
  sum_vv(pLKN,3, temp3_3x1, cLLL);
  
  mult_mv((const float**)_dcLF_3x3,3,3, _C_LF, temp3_3x1);
  sum_vv(pLANK,3, temp3_3x1, cLF);
  
  mult_mv((const float**)_dcRUA_3x3,3,3, _C_RUA, temp3_3x1);
  sum_vv(pRSHLD,3, temp3_3x1, cRUA);
  
  mult_mv((const float**)_dcRLA_3x3,3,3, _C_RLA, temp3_3x1); 
  sum_vv(pRELB2,3, temp3_3x1, cRLA);
  
  mult_mv((const float**)_dcRH_3x3,3,3, _C_RHAND, temp3_3x1);
  sum_vv(pRWR,3, temp3_3x1, cRH);
  
  mult_mv((const float**)_dcLUA_3x3,3,3, _C_LUA, temp3_3x1);
  sum_vv(pLSHLD,3, temp3_3x1, cLUA);
  
  mult_mv((const float**)_dcLLA_3x3,3,3, _C_LLA, temp3_3x1); 
  sum_vv(pLELB2,3, temp3_3x1, cLLA);
  	
  mult_mv((const float**)_dcLH_3x3,3,3, _C_LHAND, temp3_3x1);
  sum_vv(pLWR,3, temp3_3x1, cLH);
    
  //printf("\n --->%f %f %f",pLWR[1], pLWR[2], pLWR[3]);
  //printf("\n --->%f %f %f",pLELB[1], pLELB[2], pLELB[3]);
  //printf("\n --->%f %f %f",pLSHLD[1], pLSHLD[2], pLSHLD[3]);
  
  mult_mv((const float**)_dcPEL_3x3,3,3, _AXIS_Z, axis_wst);
  
  mult_mv((const float**)_dcPEL_3x3,3,3, _AXIS_Z, axis_rhy);
  mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_34x34);
  mult_mv((const float**)_TEMP1_34x34,3,3, _AXIS_X, axis_rhr);
  mult_mv((const float**)_dcRUL_3x3,3,3, _AXIS_Y, axis_rhp);
  axis_rkn[1] = axis_rhp[1];
  axis_rkn[2] = axis_rhp[2];
  axis_rkn[3] = axis_rhp[3];
  axis_rap[1] = axis_rhp[1];
  axis_rap[2] = axis_rhp[2];
  axis_rap[3] = axis_rhp[3];
  mult_mv((const float**)_dcRF_3x3,3,3, _AXIS_X, axis_rar);
  
  axis_lhy[1] = axis_rhy[1];
  axis_lhy[2] = axis_rhy[2];
  axis_lhy[3] = axis_rhy[3];
  mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_34x34);
  mult_mv((const float**)_TEMP1_34x34,3,3, _AXIS_X, axis_lhr);
  mult_mv((const float**)_dcLUL_3x3,3,3, _AXIS_Y, axis_lhp);
  axis_lkn[1] = axis_lhp[1];
  axis_lkn[2] = axis_lhp[2];
  axis_lkn[3] = axis_lhp[3];
  axis_lap[1] = axis_lhp[1];
  axis_lap[2] = axis_lhp[2];
  axis_lap[3] = axis_lhp[3];
  mult_mv((const float**)_dcLF_3x3,3,3, _AXIS_X, axis_lar);
  
  mult_mv((const float**)_dcTOR_3x3,3,3, _AXIS_Y, axis_rsp);
  mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_RSP_3x3,3, _TEMP1_34x34);
  mult_mv((const float**)_TEMP1_34x34,3,3, _AXIS_X, axis_rsr);
  mult_mv((const float**)_dcRUA_3x3,3,3, _AXIS_Z, axis_rsy);
  mult_mv((const float**)_dcRUA_3x3,3,3, _AXIS_Y, axis_reb);
  mult_mv((const float**)_dcRLA_3x3,3,3, _AXIS_Z, axis_rwy);
  mult_mv((const float**)_dcRLA_3x3,3,3, _AXIS_Y, axis_rwp);
  mult_mv((const float**)_dcRH_3x3,3,3, _AXIS_Z, axis_rwy2);
  
  axis_lsp[1] = axis_rsp[1];
  axis_lsp[2] = axis_rsp[2];
  axis_lsp[3] = axis_rsp[3];
  mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_LSP_3x3,3, _TEMP1_34x34);
  mult_mv((const float**)_TEMP1_34x34,3,3, _AXIS_X, axis_lsr);
  mult_mv((const float**)_dcLUA_3x3,3,3, _AXIS_Z, axis_lsy);
  mult_mv((const float**)_dcLUA_3x3,3,3, _AXIS_Y, axis_leb);
  mult_mv((const float**)_dcLLA_3x3,3,3, _AXIS_Z, axis_lwy);
  mult_mv((const float**)_dcLLA_3x3,3,3, _AXIS_Y, axis_lwp);
  mult_mv((const float**)_dcLH_3x3,3,3, _AXIS_Z, axis_lwy2);
  
  diff_vv(cLH,3,pLWR, temp3_3x1);
  cross(1.f,temp3_3x1, _AXIS_Z, temp4_3x1);  
  torque_33x1[LWY2_33] = m_LHAND*gAcc*dot(temp4_3x1,3,axis_lwy2);
  torque_33x1[LWP_33] = m_LHAND*gAcc*dot(temp4_3x1,3,axis_lwp);
   
  sum_svsv(m_LARM/(m_LARM+m_LHAND),cLLA,3, m_LHAND/(m_LARM+m_LHAND),cLH,temp4_3x1);
  diff_vv(temp4_3x1,3,pLELB2, temp3_3x1);
  cross(1.f,temp3_3x1, _AXIS_Z, temp4_3x1);
  torque_33x1[LWY_33] = (m_LARM+m_LHAND)*gAcc*dot(temp4_3x1,3,axis_lwy);
    
  sum_svsv(m_LARM/(m_LARM+m_LHAND),cLLA,3, m_LHAND/(m_LARM+m_LHAND),cLH,temp4_3x1);
  diff_vv(temp4_3x1,3,pLELB, temp3_3x1);
  cross(1.f,temp3_3x1, _AXIS_Z, temp4_3x1);
  torque_33x1[LEB_33] = (m_LARM+m_LHAND)*gAcc*dot(temp4_3x1,3,axis_leb);
  
  sum_svsv(m_UARM/(m_UARM+m_LARM+m_LHAND),cLUA,3, m_LARM/(m_UARM+m_LARM+m_LHAND),cLLA,temp4_3x1);
  sum_svsv(1.f,temp4_3x1,3, m_LHAND/(m_UARM+m_LARM+m_LHAND),cLH, temp4_3x1);
  diff_vv(temp4_3x1,3,pLSHLD, temp3_3x1);
  cross(1.f,temp3_3x1, _AXIS_Z, temp4_3x1);
  torque_33x1[LSY_33] = (m_UARM+m_LARM+m_LHAND)*gAcc*dot(temp4_3x1,3,axis_lsy);
  torque_33x1[LSR_33] = (m_UARM+m_LARM+m_LHAND)*gAcc*dot(temp4_3x1,3,axis_lsr);
  torque_33x1[LSP_33] = (m_UARM+m_LARM+m_LHAND)*gAcc*dot(temp4_3x1,3,axis_lsp);

  diff_vv(cRH,3,pRWR, temp3_3x1);
  cross(1.f,temp3_3x1, _AXIS_Z, temp4_3x1);  
  torque_33x1[RWY2_33] = m_RHAND*gAcc*dot(temp4_3x1,3,axis_rwy2);
  torque_33x1[RWP_33] = m_RHAND*gAcc*dot(temp4_3x1,3,axis_rwp);
   
  sum_svsv(m_LARM/(m_LARM+m_RHAND),cRLA,3, m_RHAND/(m_LARM+m_RHAND),cRH,temp4_3x1);
  diff_vv(temp4_3x1,3,pRELB2, temp3_3x1);
  cross(1.f,temp3_3x1, _AXIS_Z, temp4_3x1);
  torque_33x1[RWY_33] = (m_LARM+m_RHAND)*gAcc*dot(temp4_3x1,3,axis_rwy);
    
  sum_svsv(m_LARM/(m_LARM+m_RHAND),cRLA,3, m_RHAND/(m_LARM+m_RHAND),cRH,temp4_3x1);
  diff_vv(temp4_3x1,3,pRELB, temp3_3x1);
  cross(1.f,temp3_3x1, _AXIS_Z, temp4_3x1);
  torque_33x1[REB_33] = (m_LARM+m_RHAND)*gAcc*dot(temp4_3x1,3,axis_reb);
  
  sum_svsv(m_UARM/(m_UARM+m_LARM+m_RHAND),cRUA,3, m_LARM/(m_UARM+m_LARM+m_RHAND),cRLA,temp4_3x1);
  sum_svsv(1.f,temp4_3x1,3, m_RHAND/(m_UARM+m_LARM+m_RHAND),cRH, temp4_3x1);
  diff_vv(temp4_3x1,3,pRSHLD, temp3_3x1);
  cross(1.f,temp3_3x1, _AXIS_Z, temp4_3x1);
  torque_33x1[RSY_33] = (m_UARM+m_LARM+m_RHAND)*gAcc*dot(temp4_3x1,3,axis_rsy);
  torque_33x1[RSR_33] = (m_UARM+m_LARM+m_RHAND)*gAcc*dot(temp4_3x1,3,axis_rsr);
  torque_33x1[RSP_33] = (m_UARM+m_LARM+m_RHAND)*gAcc*dot(temp4_3x1,3,axis_rsp);

  //printf("\n %f %f %f %f %f %f %f", torque[LSP],torque[LSR],torque[LSY],torque[LEB],torque[LWY],torque[LWP],torque[LWY2]);
  return 0;
}


int getFricCompen(const float *Qp_33x1, float *fric_compen_33x1)
{
	int i;
	for(i=1; i<=33; i++)
		fric_compen_33x1[i] = 0.f;

	//-------------- right arm
	if(Qp_33x1[RSP_33] > 0.f)
	{
		if(Qp_33x1[RSP_33] < _friction_para[RSP_33][0]*D2R)
			fric_compen_33x1[RSP_33] = _friction_para[RSP_33][1]*Qp_33x1[RSP_33]/(_friction_para[RSP_33][0]*D2R);
		else
			fric_compen_33x1[RSP_33] = _friction_para[RSP_33][1];
	}
	else if(Qp_33x1[RSP_33] < 0.f)
	{
		if(fabs(Qp_33x1[RSP_33]) < _friction_para[RSP_33][0]*D2R)
			fric_compen_33x1[RSP_33] = -_friction_para[RSP_33][1]*(float)fabs(Qp_33x1[RSP_33])/(_friction_para[RSP_33][0]*D2R);
		else
			fric_compen_33x1[RSP_33] = -_friction_para[RSP_33][1];
	}
	else
		fric_compen_33x1[RSP_33] = 0.f;

	if(Qp_33x1[RSR_33] > 0.f)
	{
		if(Qp_33x1[RSR_33] < _friction_para[RSR_33][0]*D2R)
			fric_compen_33x1[RSR_33] = _friction_para[RSR_33][1]*Qp_33x1[RSR_33]/(_friction_para[RSR_33][0]*D2R);
		else
			fric_compen_33x1[RSR_33] = _friction_para[RSR_33][1];
	}
	else if(Qp_33x1[RSR_33] < 0.f)
	{
		if(fabs(Qp_33x1[RSR_33]) < _friction_para[RSR_33][0]*D2R)
			fric_compen_33x1[RSR_33] = -_friction_para[RSR_33][1]*(float)fabs(Qp_33x1[RSR_33])/(_friction_para[RSR_33][0]*D2R);
		else
			fric_compen_33x1[RSR_33] = -_friction_para[RSR_33][1];
	}
	else
		fric_compen_33x1[RSR_33] = 0.f;
	
	if(Qp_33x1[RSY_33] > 0.f)
	{
		if(Qp_33x1[RSY_33] < _friction_para[RSY_33][0]*D2R)
			fric_compen_33x1[RSY_33] = _friction_para[RSY_33][1]*Qp_33x1[RSY_33]/(_friction_para[RSY_33][0]*D2R);
		else
			fric_compen_33x1[RSY_33] = _friction_para[RSY_33][1];
	}
	else if(Qp_33x1[RSY_33] < 0.f)
	{
		if(fabs(Qp_33x1[RSY_33]) < _friction_para[RSY_33][0]*D2R)
			fric_compen_33x1[RSY_33] = -_friction_para[RSY_33][1]*(float)fabs(Qp_33x1[RSY_33])/(_friction_para[RSY_33][0]*D2R);
		else
			fric_compen_33x1[RSY_33] = -_friction_para[RSY_33][1];
	}
	else
		fric_compen_33x1[RSY_33] = 0.f;

	if(Qp_33x1[REB_33] > 0.f)
	{
		if(Qp_33x1[REB_33] < _friction_para[REB_33][0]*D2R)
			fric_compen_33x1[REB_33] = _friction_para[REB_33][1]*Qp_33x1[REB_33]/(_friction_para[REB_33][0]*D2R);
		else
			fric_compen_33x1[REB_33] = _friction_para[REB_33][1];
	}
	else if(Qp_33x1[REB_33] < 0.f)
	{
		if(fabs(Qp_33x1[REB_33]) < _friction_para[REB_33][0]*D2R)
			fric_compen_33x1[REB_33] = -_friction_para[REB_33][1]*(float)fabs(Qp_33x1[REB_33])/(_friction_para[REB_33][0]*D2R);
		else
			fric_compen_33x1[REB_33] = -_friction_para[REB_33][1];
	}
	else
		fric_compen_33x1[REB_33] = 0.f;

	if(Qp_33x1[RWY_33] > 0.f)
	{
		if(Qp_33x1[RWY_33] < _friction_para[RWY_33][0]*D2R)
			fric_compen_33x1[RWY_33] = _friction_para[RWY_33][1]*Qp_33x1[RWY_33]/(_friction_para[RWY_33][0]*D2R);
		else
			fric_compen_33x1[RWY_33] = _friction_para[RWY_33][1];
	}
	else if(Qp_33x1[RWY_33] < 0.f)
	{
		if(fabs(Qp_33x1[RWY_33]) < _friction_para[RWY_33][0]*D2R)
			fric_compen_33x1[RWY_33] = -_friction_para[RWY_33][1]*(float)fabs(Qp_33x1[RWY_33])/(_friction_para[RWY_33][0]*D2R);
		else
			fric_compen_33x1[RWY_33] = -_friction_para[RWY_33][1];
	}
	else
		fric_compen_33x1[RWY_33] = 0.f;

	if(Qp_33x1[RWP_33] > 0.f)
	{
		if(Qp_33x1[RWP_33] < _friction_para[RWP_33][0]*D2R)
			fric_compen_33x1[RWP_33] = _friction_para[RWP_33][1]*Qp_33x1[RWP_33]/(_friction_para[RWP_33][0]*D2R);
		else
			fric_compen_33x1[RWP_33] = _friction_para[RWP_33][1];
	}
	else if(Qp_33x1[RWP_33] < 0.f)
	{
		if(fabs(Qp_33x1[RWP_33]) < _friction_para[RWP_33][0]*D2R)
			fric_compen_33x1[RWP_33] = -_friction_para[RWP_33][1]*(float)fabs(Qp_33x1[RWP_33])/(_friction_para[RWP_33][0]*D2R);
		else
			fric_compen_33x1[RWP_33] = -_friction_para[RWP_33][1];
	}
	else
		fric_compen_33x1[RWP_33] = 0.f;
	
	if(Qp_33x1[RWY2_33] > 0.f)
	{
		if(Qp_33x1[RWY2_33] < _friction_para[RWY2_33][0]*D2R)
			fric_compen_33x1[RWY2_33] = _friction_para[RWY2_33][1]*Qp_33x1[RWY2_33]/(_friction_para[RWY2_33][0]*D2R);
		else
			fric_compen_33x1[RWY2_33] = _friction_para[RWY2_33][1];
	}
	else if(Qp_33x1[RWY2_33] < 0.f)
	{
		if(fabs(Qp_33x1[RWY2_33]) < _friction_para[RWY2_33][0]*D2R)
			fric_compen_33x1[RWY2_33] = -_friction_para[RWY2_33][1]*(float)fabs(Qp_33x1[RWY2_33])/(_friction_para[RWY2_33][0]*D2R);
		else
			fric_compen_33x1[RWY2_33] = -_friction_para[RWY2_33][1];
	}
	else
		fric_compen_33x1[RWY2_33] = 0.f;

	
	//----------- left arm
	if(Qp_33x1[LSP_33] > 0.f)
	{
		if(Qp_33x1[LSP_33] < _friction_para[LSP_33][0]*D2R)
			fric_compen_33x1[LSP_33] = _friction_para[LSP_33][1]*Qp_33x1[LSP_33]/(_friction_para[LSP_33][0]*D2R);
		else
			fric_compen_33x1[LSP_33] = _friction_para[LSP_33][1];
	}
	else if(Qp_33x1[LSP_33] < 0.f)
	{
		if(fabs(Qp_33x1[LSP_33]) < _friction_para[LSP_33][0]*D2R)
			fric_compen_33x1[LSP_33] = -_friction_para[LSP_33][1]*(float)fabs(Qp_33x1[LSP_33])/(_friction_para[LSP_33][0]*D2R);
		else
			fric_compen_33x1[LSP_33] = -_friction_para[LSP_33][1];
	}
	else
		fric_compen_33x1[LSP_33] = 0.f;

	if(Qp_33x1[LSR_33] > 0.f)
	{
		if(Qp_33x1[LSR_33] < _friction_para[LSR_33][0]*D2R)
			fric_compen_33x1[LSR_33] = _friction_para[LSR_33][1]*Qp_33x1[LSR_33]/(_friction_para[LSR_33][0]*D2R);
		else
			fric_compen_33x1[LSR_33] = _friction_para[LSR_33][1];
	}
	else if(Qp_33x1[LSR_33] < 0.f)
	{
		if(fabs(Qp_33x1[LSR_33]) < _friction_para[LSR_33][0]*D2R)
			fric_compen_33x1[LSR_33] = -_friction_para[LSR_33][1]*(float)fabs(Qp_33x1[LSR_33])/(_friction_para[LSR_33][0]*D2R);
		else
			fric_compen_33x1[LSR_33] = -_friction_para[LSR_33][1];
	}
	else
		fric_compen_33x1[LSR_33] = 0.f;
	
	if(Qp_33x1[LSY_33] > 0.f)
	{
		if(Qp_33x1[LSY_33] < _friction_para[LSY_33][0]*D2R)
			fric_compen_33x1[LSY_33] = _friction_para[LSY_33][1]*Qp_33x1[LSY_33]/(_friction_para[LSY_33][0]*D2R);
		else
			fric_compen_33x1[LSY_33] = _friction_para[LSY_33][1];
	}
	else if(Qp_33x1[LSY_33] < 0.f)
	{
		if(fabs(Qp_33x1[LSY_33]) < _friction_para[LSY_33][0]*D2R)
			fric_compen_33x1[LSY_33] = -_friction_para[LSY_33][1]*(float)fabs(Qp_33x1[LSY_33])/(_friction_para[LSY_33][0]*D2R);
		else
			fric_compen_33x1[LSY_33] = -_friction_para[LSY_33][1];
	}
	else
		fric_compen_33x1[LSY_33] = 0.f;

	if(Qp_33x1[LEB_33] > 0.f)
	{
		if(Qp_33x1[LEB_33] < _friction_para[LEB_33][0]*D2R)
			fric_compen_33x1[LEB_33] = _friction_para[LEB_33][1]*Qp_33x1[LEB_33]/(_friction_para[LEB_33][0]*D2R);
		else
			fric_compen_33x1[LEB_33] = _friction_para[LEB_33][1];
	}
	else if(Qp_33x1[LEB_33] < 0.f)
	{
		if(fabs(Qp_33x1[LEB_33]) < _friction_para[LEB_33][0]*D2R)
			fric_compen_33x1[LEB_33] = -_friction_para[LEB_33][1]*(float)fabs(Qp_33x1[LEB_33])/(_friction_para[LEB_33][0]*D2R);
		else
			fric_compen_33x1[LEB_33] = -_friction_para[LEB_33][1];
	}
	else
		fric_compen_33x1[LEB_33] = 0.f;

	if(Qp_33x1[LWY_33] > 0.f)
	{
		if(Qp_33x1[LWY_33] < _friction_para[LWY_33][0]*D2R)
			fric_compen_33x1[LWY_33] = _friction_para[LWY_33][1]*Qp_33x1[LWY_33]/(_friction_para[LWY_33][0]*D2R);
		else
			fric_compen_33x1[LWY_33] = _friction_para[LWY_33][1];
	}
	else if(Qp_33x1[LWY_33] < 0.f)
	{
		if(fabs(Qp_33x1[LWY_33]) < _friction_para[LWY_33][0]*D2R)
			fric_compen_33x1[LWY_33] = -_friction_para[LWY_33][1]*(float)fabs(Qp_33x1[LWY_33])/(_friction_para[LWY_33][0]*D2R);
		else
			fric_compen_33x1[LWY_33] = -_friction_para[LWY_33][1];
	}
	else
		fric_compen_33x1[LWY_33] = 0.f;

	if(Qp_33x1[LWP_33] > 0.f)
	{
		if(Qp_33x1[LWP_33] < _friction_para[LWP_33][0]*D2R)
			fric_compen_33x1[LWP_33] = _friction_para[LWP_33][1]*Qp_33x1[LWP_33]/(_friction_para[LWP_33][0]*D2R);
		else
			fric_compen_33x1[LWP_33] = _friction_para[LWP_33][1];
	}
	else if(Qp_33x1[LWP_33] < 0.f)
	{
		if(fabs(Qp_33x1[LWP_33]) < _friction_para[LWP_33][0]*D2R)
			fric_compen_33x1[LWP_33] = -_friction_para[LWP_33][1]*(float)fabs(Qp_33x1[LWP_33])/(_friction_para[LWP_33][0]*D2R);
		else
			fric_compen_33x1[LWP_33] = -_friction_para[LWP_33][1];
	}
	else
		fric_compen_33x1[LWP_33] = 0.f;
	
	if(Qp_33x1[LWY2_33] > 0.f)
	{
		if(Qp_33x1[LWY2_33] < _friction_para[LWY2_33][0]*D2R)
			fric_compen_33x1[LWY2_33] = _friction_para[LWY2_33][1]*Qp_33x1[LWY2_33]/(_friction_para[LWY2_33][0]*D2R);
		else
			fric_compen_33x1[LWY2_33] = _friction_para[LWY2_33][1];
	}
	else if(Qp_33x1[LWY2_33] < 0.f)
	{
		if(fabs(Qp_33x1[LWY2_33]) < _friction_para[LWY2_33][0]*D2R)
			fric_compen_33x1[LWY2_33] = -_friction_para[LWY2_33][1]*(float)fabs(Qp_33x1[LWY2_33])/(_friction_para[LWY2_33][0]*D2R);
		else
			fric_compen_33x1[LWY2_33] = -_friction_para[LWY2_33][1];
	}
	else
		fric_compen_33x1[LWY2_33] = 0.f;


	//---------------------- right leg
	if(Qp_33x1[RAP_33] > 0.f)
	{
		if(Qp_33x1[RAP_33] < _friction_para[RAP_33][0]*D2R)
			fric_compen_33x1[RAP_33] = _friction_para[RAP_33][1]*Qp_33x1[RAP_33]/(_friction_para[RAP_33][0]*D2R);
		else
			fric_compen_33x1[RAP_33] = _friction_para[RAP_33][1];
	}
	else if(Qp_33x1[RAP_33] < 0.f)
	{
		if(fabs(Qp_33x1[RAP_33]) < _friction_para[RAP_33][0]*D2R)
			fric_compen_33x1[RAP_33] = -_friction_para[RAP_33][1]*(float)fabs(Qp_33x1[RAP_33])/(_friction_para[RAP_33][0]*D2R);
		else
			fric_compen_33x1[RAP_33] = -_friction_para[RAP_33][1];
	}
	else
		fric_compen_33x1[RAP_33] = 0.f;


	if(Qp_33x1[RAR_33] > 0.f)
	{
		if(Qp_33x1[RAR_33] < _friction_para[RAR_33][0]*D2R)
			fric_compen_33x1[RAR_33] = _friction_para[RAR_33][1]*Qp_33x1[RAR_33]/(_friction_para[RAR_33][0]*D2R);
		else
			fric_compen_33x1[RAR_33] = _friction_para[RAR_33][1];
	}
	else if(Qp_33x1[RAR_33] < 0.f)
	{
		if(fabs(Qp_33x1[RAR_33]) < _friction_para[RAR_33][0]*D2R)
			fric_compen_33x1[RAR_33] = -_friction_para[RAR_33][1]*(float)fabs(Qp_33x1[RAR_33])/(_friction_para[RAR_33][0]*D2R);
		else
			fric_compen_33x1[RAR_33] = -_friction_para[RAR_33][1];
	}
	else
		fric_compen_33x1[RAR_33] = 0.f;


	//---------------------- left leg
	if(Qp_33x1[LAP_33] > 0.f)
	{
		if(Qp_33x1[LAP_33] < _friction_para[LAP_33][0]*D2R)
			fric_compen_33x1[LAP_33] = _friction_para[LAP_33][1]*Qp_33x1[LAP_33]/(_friction_para[LAP_33][0]*D2R);
		else
			fric_compen_33x1[LAP_33] = _friction_para[LAP_33][1];
	}
	else if(Qp_33x1[LAP_33] < 0.f)
	{
		if(fabs(Qp_33x1[LAP_33]) < _friction_para[LAP_33][0]*D2R)
			fric_compen_33x1[LAP_33] = -_friction_para[LAP_33][1]*(float)fabs(Qp_33x1[LAP_33])/(_friction_para[LAP_33][0]*D2R);
		else
			fric_compen_33x1[LAP_33] = -_friction_para[LAP_33][1];
	}
	else
		fric_compen_33x1[LAP_33] = 0.f;


	if(Qp_33x1[LAR_33] > 0.f)
	{
		if(Qp_33x1[LAR_33] < _friction_para[LAR_33][0]*D2R)
			fric_compen_33x1[LAR_33] = _friction_para[LAR_33][1]*Qp_33x1[LAR_33]/(_friction_para[LAR_33][0]*D2R);
		else
			fric_compen_33x1[LAR_33] = _friction_para[LAR_33][1];
	}
	else if(Qp_33x1[LAR_33] < 0.f)
	{
		if(fabs(Qp_33x1[LAR_33]) < _friction_para[LAR_33][0]*D2R)
			fric_compen_33x1[LAR_33] = -_friction_para[LAR_33][1]*(float)fabs(Qp_33x1[LAR_33])/(_friction_para[LAR_33][0]*D2R);
		else
			fric_compen_33x1[LAR_33] = -_friction_para[LAR_33][1];
	}
	else
		fric_compen_33x1[LAR_33] = 0.f;

	return 0;
}


int checkJointRange(float Q_34x1[])
{
	if(Q_34x1[WST_34] > WSTmax || Q_34x1[WST_34] < WSTmin)	{printf("\n WST limit error!!"); return -1;}
	if(Q_34x1[RHY_34] > RHYmax || Q_34x1[RHY_34] < RHYmin)	{printf("\n RHY limit error!!"); return -1;}
	if(Q_34x1[RHR_34] > RHRmax || Q_34x1[RHR_34] < RHRmin)	{printf("\n RHR limit error!!"); return -1;}
	if(Q_34x1[RHP_34] > RHPmax || Q_34x1[RHP_34] < RHPmin)	{printf("\n RHP limit error!!"); return -1;}
	if(Q_34x1[RKN_34] > RKNmax || Q_34x1[RKN_34] < RKNmin)	{printf("\n RKN limit error!!"); return -1;}
	if(Q_34x1[RAP_34] > RAPmax || Q_34x1[RAP_34] < RAPmin)	{printf("\n RAP limit error!!"); return -1;}
	if(Q_34x1[RAR_34] > RARmax || Q_34x1[RAR_34] < RARmin)	{printf("\n RAR limit error!!"); return -1;}
	if(Q_34x1[LHY_34] > LHYmax || Q_34x1[LHY_34] < LHYmin)	{printf("\n LHY limit error!!"); return -1;}
	if(Q_34x1[LHR_34] > LHRmax || Q_34x1[LHR_34] < LHRmin)	{printf("\n LHR limit error!!"); return -1;}
	if(Q_34x1[LHP_34] > LHPmax || Q_34x1[LHP_34] < LHPmin)	{printf("\n LHP limit error!!"); return -1;}
	if(Q_34x1[LKN_34] > LKNmax || Q_34x1[LKN_34] < LKNmin)	{printf("\n LKN limit error!!"); return -1;}
	if(Q_34x1[LAP_34] > LAPmax || Q_34x1[LAP_34] < LAPmin)	{printf("\n LAP limit error!!"); return -1;}
	if(Q_34x1[LAR_34] > LARmax || Q_34x1[LAR_34] < LARmin)	{printf("\n LAR limit error!!"); return -1;}
	if(Q_34x1[RSP_34] > RSPmax || Q_34x1[RSP_34] < RSPmin)	{printf("\n RSP limit error!!"); return -1;}
	if(Q_34x1[RSR_34] > RSRmax || Q_34x1[RSR_34] < RSRmin)	{printf("\n RSR limit error!!"); return -1;}
	if(Q_34x1[RSY_34] > RSYmax || Q_34x1[RSY_34] < RSYmin)	{printf("\n RSY limit error!!"); return -1;}
	if(Q_34x1[REB_34] > REBmax || Q_34x1[REB_34] < REBmin)	{printf("\n REB limit error!!"); return -1;}
	if(Q_34x1[RWY_34] > RWYmax || Q_34x1[RWY_34] < RWYmin)	{printf("\n RWY limit error!!"); return -1;}
	if(Q_34x1[RWP_34] > RWPmax || Q_34x1[RWP_34] < RWPmin)	{printf("\n RWP limit error!!"); return -1;}
	if(Q_34x1[LSP_34] > LSPmax || Q_34x1[LSP_34] < LSPmin)	{printf("\n LSP limit error!!"); return -1;}
	if(Q_34x1[LSR_34] > LSRmax || Q_34x1[LSR_34] < LSRmin)	{printf("\n LSR limit error!!"); return -1;}
	if(Q_34x1[LSY_34] > LSYmax || Q_34x1[LSY_34] < LSYmin)	{printf("\n LSY limit error!!"); return -1;}
	if(Q_34x1[LEB_34] > LEBmax || Q_34x1[LEB_34] < LEBmin)	{printf("\n LEB limit error!!"); return -1;}
	if(Q_34x1[LWY_34] > LWYmax || Q_34x1[LWY_34] < LWYmin)	{printf("\n LWY limit error!!"); return -1;}
	if(Q_34x1[LWP_34] > LWPmax || Q_34x1[LWP_34] < LWPmin)	{printf("\n LWP limit error!!"); return -1;}
	if(Q_34x1[RWY2_34] > RWY2max || Q_34x1[RWY2_34] < RWY2min)	{printf("\n RWY2 limit error!!"); return -1;}
	if(Q_34x1[LWY2_34] > LWY2max || Q_34x1[LWY2_34] < LWY2min)	{printf("\n LWY2 limit error!!"); return -1;}

	return 0;
}


float torque2duty(int joint_no, float torque)
{
	const int n_table = 24;
	int i;
	static const float duty_table[24] = {0.f, 0.02f, 0.06f, 0.07f, 0.08f, 0.09f, 0.10f, 0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f, 0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f, 0.23f, 0.24f, 0.25f, 0.26f, 0.27f};
	static const float amp_table1[24] = {0.f, 0.01f, 0.05f, 0.07f, 0.09f, 0.11f, 0.12f, 0.15f, 0.17f, 0.19f, 0.24f, 0.33f, 0.45f, 0.57f, 0.73f, 0.91f, 1.09f, 1.28f, 1.52f, 1.75f, 2.f, 2.25f, 2.58f, 2.87f};
	static const float amp_table2[24] = {0.f, 0.02f, 0.03f, 0.06f, 0.07f, 0.09f, 0.11f, 0.13f, 0.14f, 0.16f, 0.2f, 0.29f, 0.4f, 0.55f, 0.69f, 0.85f, 1.05f, 1.25f, 1.49f, 1.74f, 2.f, 2.29f, 2.5f, 2.72f};
	float duty, amp;
	float Kt;  // torque const. mNm/A
	float reduction_ratio;


	switch(joint_no)
	{
	case RSP:
		reduction_ratio = 250.f;
		Kt = 0.0276f;
		break;
	case RSR:
		reduction_ratio = 100.f;
		Kt = 0.0276f;
		break;
	case RSY:
		reduction_ratio = 100.f;
		Kt = 0.0255f;
		break;
	case REB:
		reduction_ratio = 200.f;
		Kt = 0.0255f;
		break;
	case RWY: //check motor spec.
		reduction_ratio = 100.f;
		Kt = 0.0258f;
		break;
	case RWP://check motor spec.
		reduction_ratio = 200.f;
		Kt = 0.0258f;
		break;
	case RWY2:
		reduction_ratio = 200.f;
		Kt = 0.030f;
		break;
	case LSP:
		reduction_ratio = 250.f;
		Kt = 0.0276f;
		break;
	case LSR:
		reduction_ratio = 100.f;
		Kt = 0.0276f;
		break;
	case LSY:
		reduction_ratio = 100.f;
		Kt = 0.0255f;
		break;
	case LEB:
		reduction_ratio = 200.f;
		Kt = 0.0255f;
		break;
	case LWY: //check motor spec.
		reduction_ratio = 100.f;
		Kt = 0.0258f;
		break;
	case LWP://check motor spec.
		reduction_ratio = 200.f;
		Kt = 0.053f;
		break;
	case LWY2:
		reduction_ratio = 200.f;
		Kt = 0.030f;
		break;
	
	case RHR:
	case LHR:
		reduction_ratio = 160.f*1024/324;
		Kt = 0.0276f;
		break;

	case RAP:
	case LAP:
		reduction_ratio = 250.f;
		Kt = 0.0276f;
		break;
	case RAR:
	case LAR:
		reduction_ratio = 100.f*1024/324;
		Kt = 0.0276f;
		break;
	default:
		return 0.f;
		break;
	}

	amp = (float)fabs(torque)/Kt/reduction_ratio;

	switch(joint_no)
	{
	case RSP:
	case RSR:
	case LSP:
	case LSR:
	case RHR:
	case LHR:
	case RAP:
	case RAR:
	case LAP:
	case LAR:
		for(i=0; i<n_table-1; i++)
			if(amp>=amp_table1[i] && amp<amp_table1[i+1])
				break;
		
		if(amp >= amp_table1[n_table-1])
			duty = (amp-amp_table1[n_table-1])/(amp_table1[n_table-1]-amp_table1[n_table-2])*(duty_table[n_table-1]-duty_table[n_table-2]) + duty_table[n_table-1];
		else
			duty = (amp-amp_table1[i])/(amp_table1[i+1]-amp_table1[i])*(duty_table[i+1]-duty_table[i]) + duty_table[i];
		break;
	case RSY:
	case REB:
	case RWY:
	case RWP:
	case LSY:
	case LEB:
	case LWY:
	case LWP:
		for(i=0; i<n_table-1; i++)
			if(amp>=amp_table2[i] && amp<amp_table2[i+1])
				break;
		
		if(amp >= amp_table2[n_table-1])
			duty = (amp-amp_table2[n_table-1])/(amp_table2[n_table-1]-amp_table2[n_table-2])*(duty_table[n_table-1]-duty_table[n_table-2]) + duty_table[n_table-1];
		else
			duty = (amp-amp_table2[i])/(amp_table2[i+1]-amp_table2[i])*(duty_table[i+1]-duty_table[i]) + duty_table[i];
		
		break;	
	default:  // RWY2, LWY2
		duty = 0.1f*amp;
		break;
	}
	
	if(joint_no!=LWY2 && joint_no!=RWY2)
		duty = limitDuty(0.4f, duty);

	if(torque >= 0.f)
		return duty;
	else
		return (-duty);
}


/*
float torque2duty(int joint_no, float torque)
{
	const int n_table = 24;
	int i;
	float duty_table[24];
	float amp_table[24];
	float duty, amp;
	float Kt;  // torque const. mNm/A
	float reduction_ratio;

	switch(joint_no)
	{
	case RSP:
		reduction_ratio = 250.f;
		Kt = 0.0276f;
		break;
	case RSR:
		reduction_ratio = 100.f;
		Kt = 0.0276f;
		break;
	case RSY:
		reduction_ratio = 100.f;
		Kt = 0.0255f;
		break;
	case REB:
		reduction_ratio = 200.f;
		Kt = 0.0255f;
		break;
	case RWY: //check motor spec.
		reduction_ratio = 100.f;
		Kt = 0.0258f;
		break;
	case RWP://check motor spec.
		reduction_ratio = 200.f;
		Kt = 0.0258f;
		break;
	case RWY2:
		reduction_ratio = 200.f;
		Kt = 0.030f;
		break;
	case LSP:
		reduction_ratio = 250.f;
		Kt = 0.0276f;
		break;
	case LSR:
		reduction_ratio = 100.f;
		Kt = 0.0276f;
		break;
	case LSY:
		reduction_ratio = 100.f;
		Kt = 0.0255f;
		break;
	case LEB:
		reduction_ratio = 200.f;
		Kt = 0.0255f;
		break;
	case LWY: //check motor spec.
		reduction_ratio = 100.f;
		Kt = 0.0258f;
		break;
	case LWP://check motor spec.
		reduction_ratio = 200.f;
		Kt = 0.053f;
		break;
	case LWY2:
		reduction_ratio = 200.f;
		Kt = 0.030f;
		break;

	case RAP:
	case LAP:
		reduction_ratio = 250.f;
		Kt = 0.0276f;
		break;
	case RAR:
	case LAR:
		reduction_ratio = 100.f*1024/324;
		Kt = 0.0276f;
		break;
	default:
		return 0.f;
		break;
	}

	switch(joint_no)
	{
	case RSP:
	case RSR:
	case LSP:
	case LSR:
	case RAP:
	case RAR:
	case LAP:
	case LAR:
		duty_table[0] = 0.f;
		duty_table[1] = 0.02f;
		duty_table[2] = 0.06f;
		duty_table[3] = 0.07f;
		duty_table[4] = 0.08f;
		duty_table[5] = 0.09f;
		duty_table[6] = 0.10f;
		duty_table[7] = 0.11f;
		duty_table[8] = 0.12f;
		duty_table[9] = 0.13f;
		duty_table[10] = 0.14f;
		duty_table[11] = 0.15f;
		duty_table[12] = 0.16f;
		duty_table[13] = 0.17f;
		duty_table[14] = 0.18f;
		duty_table[15] = 0.19f;
		duty_table[16] = 0.20f;
		duty_table[17] = 0.21f;
		duty_table[18] = 0.22f;
		duty_table[19] = 0.23f;
		duty_table[20] = 0.24f;
		duty_table[21] = 0.25f;
		duty_table[22] = 0.26f;
		duty_table[23] = 0.27f;

		amp_table[0] = 0.f;
		amp_table[1] = 0.01f;
		amp_table[2] = 0.05f;
		amp_table[3] = 0.07f;
		amp_table[4] = 0.09f;
		amp_table[5] = 0.11f; 
		amp_table[6] = 0.12f;
		amp_table[7] = 0.15f;
		amp_table[8] = 0.17f;
		amp_table[9] = 0.19f;
		amp_table[10] = 0.24f;
		amp_table[11] = 0.33f;
		amp_table[12] = 0.45f;
		amp_table[13] = 0.57f;
		amp_table[14] = 0.73f;
		amp_table[15] = 0.91f;
		amp_table[16] = 1.09f;
		amp_table[17] = 1.28f;
		amp_table[18] = 1.52f;
		amp_table[19] = 1.75f;
		amp_table[20] = 2.f;
		amp_table[21] = 2.25f;
		amp_table[22] = 2.58f;
		amp_table[23] = 2.87f;
		break;
	case RSY:
	case REB:
	case RWY:
	case RWP:
	case LSY:
	case LEB:
	case LWY:
	case LWP:
		duty_table[0] = 0.f;
		duty_table[1] = 0.02f;
		duty_table[2] = 0.05f;
		duty_table[3] = 0.07f;
		duty_table[4] = 0.08f;
		duty_table[5] = 0.09f;
		duty_table[6] = 0.1f;
		duty_table[7] = 0.11f;
		duty_table[8] = 0.12f;
		duty_table[9] = 0.13f;
		duty_table[10] = 0.14f;
		duty_table[11] = 0.15f;
		duty_table[12] = 0.16f;
		duty_table[13] = 0.17f;
		duty_table[14] = 0.18f;
		duty_table[15] = 0.19f;
		duty_table[16] = 0.2f;
		duty_table[17] = 0.21f;
		duty_table[18] = 0.22f;
		duty_table[19] = 0.23f;
		duty_table[20] = 0.24f;
		duty_table[21] = 0.25f;
		duty_table[22] = 0.26f;
		duty_table[23] = 0.27f;

		amp_table[0] = 0.f;
		amp_table[1] = 0.02f;
		amp_table[2] = 0.03f;
		amp_table[3] = 0.06f;
		amp_table[4] = 0.07f;
		amp_table[5] = 0.09f;
		amp_table[6] = 0.11f;
		amp_table[7] = 0.13f;
		amp_table[8] = 0.14f;
		amp_table[9] = 0.16f;
		amp_table[10] = 0.2f;
		amp_table[11] = 0.29f;
		amp_table[12] = 0.4f;
		amp_table[13] = 0.55f;
		amp_table[14] = 0.69f;
		amp_table[15] = 0.85f;
		amp_table[16] = 1.05f;
		amp_table[17] = 1.25f;
		amp_table[18] = 1.49f;
		amp_table[19] = 1.74f;
		amp_table[20] = 2.f;
		amp_table[21] = 2.29f;
		amp_table[22] = 2.5f;
		amp_table[23] = 2.72f;
		break;
	}
	
	amp = (float)fabs(torque)/Kt/reduction_ratio;

	if(joint_no!=RWY2 && joint_no!=LWY2)
	{
		for(i=0; i<n_table-1; i++)
			if(amp>=amp_table[i] && amp<amp_table[i+1])
				break;
		
		if(amp >= amp_table[n_table-1])
			duty = (amp-amp_table[n_table-1])/(amp_table[n_table-1]-amp_table[n_table-2])*(duty_table[n_table-1]-duty_table[n_table-2]) + duty_table[n_table-1];
		else
			duty = (amp-amp_table[i])/(amp_table[i+1]-amp_table[i])*(duty_table[i+1]-duty_table[i]) + duty_table[i];
	}
	else
		duty = 0.1f*amp;

	if(joint_no!=LWY2 && joint_no!=RWY2)
		duty = limitDuty(0.4f, duty);

	if(torque >= 0.f)
		return duty;
	else
		return (-duty);
}*/




int LoadParameter_ComputedTorque(void)
{
	int i;
	FILE *fp;

	if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\gain_gravity.txt","r")) == NULL )
	{
		printf("Data file gain_gravity.txt was not found\n");		
		return 1;
	}
	for(i=0;i<=33;i++) 
		_gain_gravity[i] = 0.f;
	fscanf(fp,"%f %f %f %f %f %f %f", &_gain_gravity[RSP_33], &_gain_gravity[RSR_33], &_gain_gravity[RSY_33], &_gain_gravity[REB_33], &_gain_gravity[RWY_33], &_gain_gravity[RWP_33], &_gain_gravity[RWY2_33]);
	fscanf(fp,"%f %f %f %f %f %f %f", &_gain_gravity[LSP_33], &_gain_gravity[LSR_33], &_gain_gravity[LSY_33], &_gain_gravity[LEB_33], &_gain_gravity[LWY_33], &_gain_gravity[LWP_33], &_gain_gravity[LWY2_33]);
	fscanf(fp,"%f %f %f %f %f %f", &_gain_gravity[RHY_33], &_gain_gravity[RHR_33], &_gain_gravity[RHP_33], &_gain_gravity[RKN_33], &_gain_gravity[RAP_33], &_gain_gravity[RAR_33]);
	fscanf(fp,"%f %f %f %f %f %f", &_gain_gravity[LHY_33], &_gain_gravity[LHR_33], &_gain_gravity[LHP_33], &_gain_gravity[LKN_33], &_gain_gravity[LAP_33], &_gain_gravity[LAR_33]);
	fclose(fp);


	if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\gain_task.txt","r")) == NULL )
	{
		printf("Data file gain_task.txt was not found\n");		
		return 1;
	}
	for(i=0;i<=33;i++) 
		_gain_task[i] = 0.f;
	fscanf(fp,"%f %f %f %f %f %f %f", &_gain_task[RSP_33], &_gain_task[RSR_33], &_gain_task[RSY_33], &_gain_task[REB_33], &_gain_task[RWY_33], &_gain_task[RWP_33], &_gain_task[RWY2_33]);
	fscanf(fp,"%f %f %f %f %f %f %f", &_gain_task[LSP_33], &_gain_task[LSR_33], &_gain_task[LSY_33], &_gain_task[LEB_33], &_gain_task[LWY_33], &_gain_task[LWP_33], &_gain_task[LWY2_33]);
	fscanf(fp,"%f %f %f %f %f %f", &_gain_task[RHY_33], &_gain_task[RHR_33], &_gain_task[RHP_33], &_gain_task[RKN_33], &_gain_task[RAP_33], &_gain_task[RAR_33]);
	fscanf(fp,"%f %f %f %f %f %f", &_gain_task[LHY_33], &_gain_task[LHR_33], &_gain_task[LHP_33], &_gain_task[LKN_33], &_gain_task[LAP_33], &_gain_task[LAR_33]);
	fclose(fp);

	
	if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\friction.txt","r")) == NULL )
	{
		printf("Data file friction.txt was not found\n");		
		return 1;
	}
	for(i=0;i<=33;i++) 
	{
		_friction_para[i][0] = 0.f;
		_friction_para[i][1] = 0.f;
	}
	fscanf(fp,"%f %f", &_friction_para[RSP_33][0], &_friction_para[RSP_33][1]);
	fscanf(fp,"%f %f", &_friction_para[RSR_33][0], &_friction_para[RSR_33][1]);
	fscanf(fp,"%f %f", &_friction_para[RSY_33][0], &_friction_para[RSY_33][1]);
	fscanf(fp,"%f %f", &_friction_para[REB_33][0], &_friction_para[REB_33][1]);
	fscanf(fp,"%f %f", &_friction_para[RWY_33][0], &_friction_para[RWY_33][1]);
	fscanf(fp,"%f %f", &_friction_para[RWP_33][0], &_friction_para[RWP_33][1]);
	fscanf(fp,"%f %f", &_friction_para[RWY2_33][0], &_friction_para[RWY2_33][1]);
	fgetc(fp);
	while (fgetc(fp) != '\n');
	fscanf(fp,"%f %f", &_friction_para[LSP_33][0], &_friction_para[LSP_33][1]);
	fscanf(fp,"%f %f", &_friction_para[LSR_33][0], &_friction_para[LSR_33][1]);
	fscanf(fp,"%f %f", &_friction_para[LSY_33][0], &_friction_para[LSY_33][1]);
	fscanf(fp,"%f %f", &_friction_para[LEB_33][0], &_friction_para[LEB_33][1]);
	fscanf(fp,"%f %f", &_friction_para[LWY_33][0], &_friction_para[LWY_33][1]);
	fscanf(fp,"%f %f", &_friction_para[LWP_33][0], &_friction_para[LWP_33][1]);
	fscanf(fp,"%f %f", &_friction_para[LWY2_33][0], &_friction_para[LWY2_33][1]);
	fgetc(fp);
	while (fgetc(fp) != '\n');
	fscanf(fp,"%f %f", &_friction_para[RHY_33][0], &_friction_para[RHY_33][1]);
	fscanf(fp,"%f %f", &_friction_para[RHR_33][0], &_friction_para[RHR_33][1]);
	fscanf(fp,"%f %f", &_friction_para[RHP_33][0], &_friction_para[RHP_33][1]);
	fscanf(fp,"%f %f", &_friction_para[RKN_33][0], &_friction_para[RKN_33][1]);
	fscanf(fp,"%f %f", &_friction_para[RAP_33][0], &_friction_para[RAP_33][1]);
	fscanf(fp,"%f %f", &_friction_para[RAR_33][0], &_friction_para[RAR_33][1]);
	fgetc(fp);
	while (fgetc(fp) != '\n');
	fscanf(fp,"%f %f", &_friction_para[LHY_33][0], &_friction_para[LHY_33][1]);
	fscanf(fp,"%f %f", &_friction_para[LHR_33][0], &_friction_para[LHR_33][1]);
	fscanf(fp,"%f %f", &_friction_para[LHP_33][0], &_friction_para[LHP_33][1]);
	fscanf(fp,"%f %f", &_friction_para[LKN_33][0], &_friction_para[LKN_33][1]);
	fscanf(fp,"%f %f", &_friction_para[LAP_33][0], &_friction_para[LAP_33][1]);
	fscanf(fp,"%f %f", &_friction_para[LAR_33][0], &_friction_para[LAR_33][1]);
	fclose(fp);

	return 0;
}


int WBIK_DRC_ladder_climbing(unsigned int n)  
{
	int i,j;
	
	float Qpp_33x1[34], meas_Q_34x1[35], meas_Qp_33x1[34];
	float X1_33x1[34], Xrf_6x1[7], Xlf_6x1[7], Xrh_6x1[7], Xlh_6x1[7], Xcom_3x1[4], Xpel_3x1[4], Xwst, Xpelz, X2_33x1[34];  // task variables
	float ub_27x1[28], lb_27x1[28], qpp_limited[28];  // joint upper-bound, lower-bound
	int index_limited[28];
	
	float des_pCOM_3x1[4], des_vCOM_3x1[4];
	float des_pRF_3x1[4], des_pLF_3x1[4], des_vRF_3x1[4], des_vLF_3x1[4];
	float des_qRF_4x1[5], des_qLF_4x1[5], des_wRF_3x1[4], des_wLF_3x1[4];
	float des_pRH_3x1[4], des_pLH_3x1[4], des_vRH_3x1[4], des_vLH_3x1[4];
	float des_qRH_4x1[5], des_qLH_4x1[5], des_wRH_3x1[4], des_wLH_3x1[4];
	float des_WST, des_WSTp, des_qPEL_4x1[5];
	float des_pPELz, des_vPELz;
	float vCOM_3x1[4];

	BOOL isLimited = FALSE;
	int dim_primary_task, dim_redundant_task;

	float lL, lR, fRFz, fLFz;
	float rhr_compen = 0.f;
	float lhr_compen = 0.f;
	float rhy_compen = 0.f;
	float lhy_compen = 0.f;
	
	float fric_compen_33x1[34], ct_33x1[34], gravity_33x1[34], duty_33x1[34], duty_joint_limit_33x1[34];

	static char half_flag, reset_neutral_flag;
	static int count;
	static float neutral_Q_34x1[35], Q_old_34x1[35], Q_old2_34x1[35];
	static float pCOM1_3x1[4], pPELz1, pRF1_3x1[4], pLF1_3x1[4], qRF1_4x1[5], qLF1_4x1[5];
	static float pRH1_3x1[4], pLH1_3x1[4], qRH1_4x1[5], qLH1_4x1[5];
	static float pRH2_3x1[4], pLH2_3x1[4], qRH2_4x1[5], qLH2_4x1[5];
	static float t;

	float dpRF_3x1[4], dpLF_3x1[4], dqRF_4x1[5], dqLF_4x1[5];
	float dpRH_3x1[4], dpLH_3x1[4], dqRH_4x1[5], dqLH_4x1[5];
	float dpPELz, dpCOM_3x1[4];

	float ftemp1_7x1[8], ftemp2_7x1[8], ftemp3_7x1[8], ftemp4_7x1[8];
	float pre_gain = 0.f;

	if(_FKineUpdatedFlag != 1)
		return -4;
	
	if(n<=2)
	{
		neutral_Q_34x1[WST_34] = Joint[WST].RefAngleCurrent*D2R;

		neutral_Q_34x1[RSP_34] = Joint[RSP].RefAngleCurrent*D2R;
		neutral_Q_34x1[RSR_34] = (Joint[RSR].RefAngleCurrent + OFFSET_RSR)*D2R;
		neutral_Q_34x1[RSY_34] = Joint[RSY].RefAngleCurrent*D2R;
		neutral_Q_34x1[REB_34] = (Joint[REB].RefAngleCurrent + OFFSET_REB)*D2R;
		neutral_Q_34x1[RWY_34] = Joint[RWY].RefAngleCurrent*D2R;
		neutral_Q_34x1[RWP_34] = Joint[RWP].RefAngleCurrent*D2R;
		neutral_Q_34x1[RWY2_34] = Joint[RWY2].RefAngleCurrent*D2R;
		
		neutral_Q_34x1[LSP_34] = Joint[LSP].RefAngleCurrent*D2R;
		neutral_Q_34x1[LSR_34] = (Joint[LSR].RefAngleCurrent + OFFSET_LSR)*D2R;
		neutral_Q_34x1[LSY_34] = Joint[LSY].RefAngleCurrent*D2R;
		neutral_Q_34x1[LEB_34] = (Joint[LEB].RefAngleCurrent + OFFSET_LEB)*D2R;
		neutral_Q_34x1[LWY_34] = Joint[LWY].RefAngleCurrent*D2R;
		neutral_Q_34x1[LWP_34] = Joint[LWP].RefAngleCurrent*D2R;
		neutral_Q_34x1[LWY2_34] = Joint[LWY2].RefAngleCurrent*D2R;

		_pPEL0_3x1[1] = _Q_34x1[1];
		_pPEL0_3x1[2] = _Q_34x1[2];
		pPELz1 = _pPEL0_3x1[3] = _Q_34x1[3];

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

		pRH1_3x1[1] = _pRH0_3x1[1] = _pRH_3x1[1];
		pRH1_3x1[2] = _pRH0_3x1[2] = _pRH_3x1[2];
		pRH1_3x1[3] = _pRH0_3x1[3] = _pRH_3x1[3];

		qRH1_4x1[1] = _qRH0_4x1[1] = _qRH_4x1[1];
		qRH1_4x1[2] = _qRH0_4x1[2] = _qRH_4x1[2];
		qRH1_4x1[3] = _qRH0_4x1[3] = _qRH_4x1[3];
		qRH1_4x1[4] = _qRH0_4x1[4] = _qRH_4x1[4];

		pLH1_3x1[1] = _pLH0_3x1[1] = _pLH_3x1[1];
		pLH1_3x1[2] = _pLH0_3x1[2] = _pLH_3x1[2];
		pLH1_3x1[3] = _pLH0_3x1[3] = _pLH_3x1[3];

		qLH1_4x1[1] = _qLH0_4x1[1] = _qLH_4x1[1];
		qLH1_4x1[2] = _qLH0_4x1[2] = _qLH_4x1[2];
		qLH1_4x1[3] = _qLH0_4x1[3] = _qLH_4x1[3];
		qLH1_4x1[4] = _qLH0_4x1[4] = _qLH_4x1[4];

		meas_Q_34x1[RSP_34] = ((float)Joint[RSP].EncoderValue)/Joint[RSP].PPR*D2R;
		meas_Q_34x1[RSR_34] = ((float)Joint[RSR].EncoderValue)/Joint[RSR].PPR*D2R + OFFSET_RSR*D2R;
		meas_Q_34x1[RSY_34] = ((float)Joint[RSY].EncoderValue)/Joint[RSY].PPR*D2R;
		meas_Q_34x1[REB_34] = ((float)Joint[REB].EncoderValue)/Joint[REB].PPR*D2R + OFFSET_REB*D2R;
		meas_Q_34x1[RWY_34] = ((float)Joint[RWY].EncoderValue)/Joint[RWY].PPR*D2R;
		meas_Q_34x1[RWP_34] = ((float)Joint[RWP].EncoderValue)/Joint[RWP].PPR*D2R;
		meas_Q_34x1[RWY2_34] = 0.f;//((float)Joint[RWY2].EncoderValue)/Joint[RWY2].PPR*D2R;

		meas_Q_34x1[LSP_34] = ((float)Joint[LSP].EncoderValue)/Joint[LSP].PPR*D2R;
		meas_Q_34x1[LSR_34] = ((float)Joint[LSR].EncoderValue)/Joint[LSR].PPR*D2R + OFFSET_LSR*D2R;
		meas_Q_34x1[LSY_34] = ((float)Joint[LSY].EncoderValue)/Joint[LSY].PPR*D2R;
		meas_Q_34x1[LEB_34] = ((float)Joint[LEB].EncoderValue)/Joint[LEB].PPR*D2R + OFFSET_LEB*D2R;
		meas_Q_34x1[LWY_34] = ((float)Joint[LWY].EncoderValue)/Joint[LWY].PPR*D2R;
		meas_Q_34x1[LWP_34] = ((float)Joint[LWP].EncoderValue)/Joint[LWP].PPR*D2R;
		meas_Q_34x1[LWY2_34] = 0.f;//((float)Joint[LWY2].EncoderValue)/Joint[LWY2].PPR*D2R;

		Q_old2_34x1[RSP_34] = Q_old_34x1[RSP_34] = meas_Q_34x1[RSP_34];
		Q_old2_34x1[RSR_34] = Q_old_34x1[RSR_34] = meas_Q_34x1[RSR_34];
		Q_old2_34x1[RSY_34] = Q_old_34x1[RSY_34] = meas_Q_34x1[RSY_34];
		Q_old2_34x1[REB_34] = Q_old_34x1[REB_34] = meas_Q_34x1[REB_34];
		Q_old2_34x1[RWY_34] = Q_old_34x1[RWY_34] = meas_Q_34x1[RWY_34];
		Q_old2_34x1[RWP_34] = Q_old_34x1[RWP_34] = meas_Q_34x1[RWP_34];
		Q_old2_34x1[RWY2_34] = Q_old_34x1[RWY2_34] = meas_Q_34x1[RWY2_34];

		Q_old2_34x1[LSP_34] = Q_old_34x1[LSP_34] = meas_Q_34x1[LSP_34];
		Q_old2_34x1[LSR_34] = Q_old_34x1[LSR_34] = meas_Q_34x1[LSR_34];
		Q_old2_34x1[LSY_34] = Q_old_34x1[LSY_34] = meas_Q_34x1[LSY_34];
		Q_old2_34x1[LEB_34] = Q_old_34x1[LEB_34] = meas_Q_34x1[LEB_34];
		Q_old2_34x1[LWY_34] = Q_old_34x1[LWY_34] = meas_Q_34x1[LWY_34];
		Q_old2_34x1[LWP_34] = Q_old_34x1[LWP_34] = meas_Q_34x1[LWP_34];
		Q_old2_34x1[LWY2_34] = Q_old_34x1[LWY2_34] = meas_Q_34x1[LWY2_34];

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
	
	dpPELz = 0.f;

	dpCOM_3x1[1] = 0.f;
	dpCOM_3x1[2] = 0.f;
	dpCOM_3x1[3] = 0.f;
	
	dpRF_3x1[1] = 0.f;
	dpRF_3x1[2] = 0.f;
	dpRF_3x1[3] = 0.f;
	
	dpLF_3x1[1] = 0.f;
	dpLF_3x1[2] = 0.f;
	dpLF_3x1[3] = 0.f;
	
	dpRH_3x1[1] = 0.f;
	dpRH_3x1[2] = 0.f;
	dpRH_3x1[3] = 0.f;
	
	dpLH_3x1[1] = 0.f;
	dpLH_3x1[2] = 0.f;
	dpLH_3x1[3] = 0.f;
	
	dqRF_4x1[1] = 1.f;
	dqRF_4x1[2] = 0.f;
	dqRF_4x1[3] = 0.f;
	dqRF_4x1[4] = 0.f;
	
	dqLF_4x1[1] = 1.f;
	dqLF_4x1[2] = 0.f;
	dqLF_4x1[3] = 0.f;
	dqLF_4x1[4] = 0.f;
	
	dqRH_4x1[1] = 1.f;
	dqRH_4x1[2] = 0.f;
	dqRH_4x1[3] = 0.f;
	dqRH_4x1[4] = 0.f;
	
	dqLH_4x1[1] = 1.f;
	dqLH_4x1[2] = 0.f;
	dqLH_4x1[3] = 0.f;
	dqLH_4x1[4] = 0.f;

	reset_neutral_flag = 0;
	if(pSharedMemory->position_mode_flag == 0) // staying
	{	
		t = 0.f;
	}
	else if(pSharedMemory->position_mode_flag == 1)  // move hands
	{	
		
		dpRH_3x1[1] = one_cos(t, pSharedMemory->dpRH[0], pSharedMemory->move_sec);
		dpRH_3x1[2] = one_cos(t, pSharedMemory->dpRH[1], pSharedMemory->move_sec);
		dpRH_3x1[3] = one_cos(t, pSharedMemory->dpRH[2], pSharedMemory->move_sec);

		dpLH_3x1[1] = one_cos(t, pSharedMemory->dpLH[0], pSharedMemory->move_sec);
		dpLH_3x1[2] = one_cos(t, pSharedMemory->dpLH[1], pSharedMemory->move_sec);
		dpLH_3x1[3] = one_cos(t, pSharedMemory->dpLH[2], pSharedMemory->move_sec);

		ftemp1_7x1[0] = one_cos(t, pSharedMemory->drvRH[0], pSharedMemory->move_sec);
		ftemp1_7x1[1] = pSharedMemory->drvRH[1];
		ftemp1_7x1[2] = pSharedMemory->drvRH[2];
		ftemp1_7x1[3] = pSharedMemory->drvRH[3];
		RV2QT(ftemp1_7x1, dqRH_4x1);

		ftemp1_7x1[0] = one_cos(t, pSharedMemory->drvLH[0], pSharedMemory->move_sec);
		ftemp1_7x1[1] = pSharedMemory->drvLH[1];
		ftemp1_7x1[2] = pSharedMemory->drvLH[2];
		ftemp1_7x1[3] = pSharedMemory->drvLH[3];
		RV2QT(ftemp1_7x1, dqLH_4x1);
		
		t += DT;

		if(t > pSharedMemory->move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 2)  //move RF and LF
	{
		dpRF_3x1[1] = one_cos(t, pSharedMemory->dpRF[0], pSharedMemory->move_sec);
		dpRF_3x1[2] = one_cos(t, pSharedMemory->dpRF[1], pSharedMemory->move_sec);
		dpRF_3x1[3] = one_cos(t, pSharedMemory->dpRF[2], pSharedMemory->move_sec);

		dpLF_3x1[1] = one_cos(t, pSharedMemory->dpLF[0], pSharedMemory->move_sec);
		dpLF_3x1[2] = one_cos(t, pSharedMemory->dpLF[1], pSharedMemory->move_sec);
		dpLF_3x1[3] = one_cos(t, pSharedMemory->dpLF[2], pSharedMemory->move_sec);

		ftemp1_7x1[0] = one_cos(t, pSharedMemory->drvRF[0], pSharedMemory->move_sec);
		ftemp1_7x1[1] = pSharedMemory->drvRF[1];
		ftemp1_7x1[2] = pSharedMemory->drvRF[2];
		ftemp1_7x1[3] = pSharedMemory->drvRF[3];
		RV2QT(ftemp1_7x1, dqRF_4x1);

		ftemp1_7x1[0] = one_cos(t, pSharedMemory->drvLF[0], pSharedMemory->move_sec);
		ftemp1_7x1[1] = pSharedMemory->drvLF[1];
		ftemp1_7x1[2] = pSharedMemory->drvLF[2];
		ftemp1_7x1[3] = pSharedMemory->drvLF[3];
		RV2QT(ftemp1_7x1, dqLF_4x1);

		t += DT;
		if(t > pSharedMemory->move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 3)  //move COM
	{	
		dpCOM_3x1[1] = one_cos(t, pSharedMemory->dpCOM[0], pSharedMemory->move_sec);
		dpCOM_3x1[2] = one_cos(t, pSharedMemory->dpCOM[1], pSharedMemory->move_sec);
	
		t += DT;
		if(t > pSharedMemory->move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 4)  //move pPELz
	{
		dpPELz = one_cos(t, pSharedMemory->dpPELz, pSharedMemory->move_sec);
	
		t += DT;
		if(t > pSharedMemory->move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 5)  //offline traj
	{
		dpPELz = pSharedMemory->off_traj_dpPELz[pSharedMemory->off_traj_count];
		dpCOM_3x1[1] = pSharedMemory->off_traj_dpCOM[pSharedMemory->off_traj_count][0];
		dpCOM_3x1[2] = pSharedMemory->off_traj_dpCOM[pSharedMemory->off_traj_count][1];
		dpRH_3x1[1] = pSharedMemory->off_traj_dpRH[pSharedMemory->off_traj_count][0];
		dpRH_3x1[2] = pSharedMemory->off_traj_dpRH[pSharedMemory->off_traj_count][1];
		dpRH_3x1[3] = pSharedMemory->off_traj_dpRH[pSharedMemory->off_traj_count][2];
		dpLH_3x1[1] = pSharedMemory->off_traj_dpLH[pSharedMemory->off_traj_count][0];
		dpLH_3x1[2] = pSharedMemory->off_traj_dpLH[pSharedMemory->off_traj_count][1];
		dpLH_3x1[3] = pSharedMemory->off_traj_dpLH[pSharedMemory->off_traj_count][2];
		dpRF_3x1[1] = pSharedMemory->off_traj_dpRF[pSharedMemory->off_traj_count][0];
		dpRF_3x1[2] = pSharedMemory->off_traj_dpRF[pSharedMemory->off_traj_count][1];
		dpRF_3x1[3] = pSharedMemory->off_traj_dpRF[pSharedMemory->off_traj_count][2];
		dpLF_3x1[1] = pSharedMemory->off_traj_dpLF[pSharedMemory->off_traj_count][0];
		dpLF_3x1[2] = pSharedMemory->off_traj_dpLF[pSharedMemory->off_traj_count][1];
		dpLF_3x1[3] = pSharedMemory->off_traj_dpLF[pSharedMemory->off_traj_count][2];
		RV2QT(pSharedMemory->off_traj_drvRH[pSharedMemory->off_traj_count], dqRH_4x1);
		RV2QT(pSharedMemory->off_traj_drvLH[pSharedMemory->off_traj_count], dqLH_4x1);
		RV2QT(pSharedMemory->off_traj_drvRF[pSharedMemory->off_traj_count], dqRF_4x1);
		RV2QT(pSharedMemory->off_traj_drvLF[pSharedMemory->off_traj_count], dqLF_4x1);
		pSharedMemory->off_traj_count++;
		if(pSharedMemory->off_traj_count >= pSharedMemory->off_traj_length)
		{
			reset_neutral_flag = 1;
			pSharedMemory->off_traj_count = 0;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	
	des_WST = 0.f;
	des_WSTp = 0.f;
	des_qPEL_4x1[1] = 1.f;
	des_qPEL_4x1[2] = 0.f;
	des_qPEL_4x1[3] = 0.f;
	des_qPEL_4x1[4] = 0.f;

	des_pPELz = pPELz1 + dpPELz;

	des_pCOM_3x1[1] = pCOM1_3x1[1] + dpCOM_3x1[1];
	des_pCOM_3x1[2] = pCOM1_3x1[2] + dpCOM_3x1[2];
	des_pCOM_3x1[3] = pCOM1_3x1[2] + dpCOM_3x1[3];

	des_pRF_3x1[1] = pRF1_3x1[1] + dpRF_3x1[1];
	des_pRF_3x1[2] = pRF1_3x1[2] + dpRF_3x1[2];
	des_pRF_3x1[3] = pRF1_3x1[3] + dpRF_3x1[3];
	
	des_pLF_3x1[1] = pLF1_3x1[1] + dpLF_3x1[1];
	des_pLF_3x1[2] = pLF1_3x1[2] + dpLF_3x1[2];
	des_pLF_3x1[3] = pLF1_3x1[3] + dpLF_3x1[3];
	
	des_pRH_3x1[1] = pRH1_3x1[1] + dpRH_3x1[1];
	des_pRH_3x1[2] = pRH1_3x1[2] + dpRH_3x1[2];
	des_pRH_3x1[3] = pRH1_3x1[3] + dpRH_3x1[3];

	des_pLH_3x1[1] = pLH1_3x1[1] + dpLH_3x1[1];
	des_pLH_3x1[2] = pLH1_3x1[2] + dpLH_3x1[2];
	des_pLH_3x1[3] = pLH1_3x1[3] + dpLH_3x1[3];

	QTcross(dqRH_4x1, qRH1_4x1, des_qRH_4x1);
	QTcross(dqLH_4x1, qLH1_4x1, des_qLH_4x1);
	QTcross(dqRF_4x1, qRF1_4x1, des_qRF_4x1);
	QTcross(dqLF_4x1, qLF1_4x1, des_qLF_4x1);
		

	if(reset_neutral_flag==1)
	{
		for(i=1;i<=34;i++)
			neutral_Q_34x1[i] = _Q_34x1[i];

		pPELz1 = des_pPELz;

		pCOM1_3x1[1] = des_pCOM_3x1[1];
		pCOM1_3x1[2] = des_pCOM_3x1[2];

		pRF1_3x1[1] = des_pRF_3x1[1];
		pRF1_3x1[2] = des_pRF_3x1[2];
		pRF1_3x1[3] = des_pRF_3x1[3];

		pLF1_3x1[1] = des_pLF_3x1[1];
		pLF1_3x1[2] = des_pLF_3x1[2];
		pLF1_3x1[3] = des_pLF_3x1[3];

		pRH1_3x1[1] = des_pRH_3x1[1];
		pRH1_3x1[2] = des_pRH_3x1[2];
		pRH1_3x1[3] = des_pRH_3x1[3];

		pLH1_3x1[1] = des_pLH_3x1[1];
		pLH1_3x1[2] = des_pLH_3x1[2];
		pLH1_3x1[3] = des_pLH_3x1[3];

		qRF1_4x1[1] = des_qRF_4x1[1];
		qRF1_4x1[2] = des_qRF_4x1[2];
		qRF1_4x1[3] = des_qRF_4x1[3];
		qRF1_4x1[4] = des_qRF_4x1[4];

		qLF1_4x1[1] = des_qLF_4x1[1];
		qLF1_4x1[2] = des_qLF_4x1[2];
		qLF1_4x1[3] = des_qLF_4x1[3];
		qLF1_4x1[4] = des_qLF_4x1[4];

		qRH1_4x1[1] = des_qRH_4x1[1];
		qRH1_4x1[2] = des_qRH_4x1[2];
		qRH1_4x1[3] = des_qRH_4x1[3];
		qRH1_4x1[4] = des_qRH_4x1[4];

		qLH1_4x1[1] = des_qLH_4x1[1];
		qLH1_4x1[2] = des_qLH_4x1[2];
		qLH1_4x1[3] = des_qLH_4x1[3];
		qLH1_4x1[4] = des_qLH_4x1[4];

		printf("\n move done");
		reset_neutral_flag = 0;
	}

	des_vPELz = 0.f;

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
	diff_vv(des_pRH_3x1,3, _pRH_3x1, ftemp3_7x1);
	mult_mv((const float**)_jRH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
	diff_vv(des_vRH_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[14], ftemp3_7x1,3, pSharedMemory->kd[14],ftemp1_7x1, ftemp3_7x1);
	mult_mv((const float**)_jRHp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xrh_6x1);

	QTdel(des_qRH_4x1, _qRH_4x1, ftemp3_7x1);	
	diff_vv(des_wRH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(pSharedMemory->kp[14],ftemp3_7x1,3, pSharedMemory->kd[14],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xrh_6x1[3]);
	
	diff_vv(des_pLH_3x1,3, _pLH_3x1, ftemp3_7x1);  
	mult_mv((const float**)_jLH_6x33,6,33, _Qp_33x1, ftemp2_7x1);  
	diff_vv(des_vLH_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[14], ftemp3_7x1,3, pSharedMemory->kd[14],ftemp1_7x1, ftemp3_7x1);  
	mult_mv((const float**)_jLHp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	  
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xlh_6x1);

	QTdel(des_qLH_4x1, _qLH_4x1, ftemp3_7x1);	
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
	//X1_33x1[28] = Xpel_3x1[1];
	//X1_33x1[29] = Xpel_3x1[2];
	//X1_33x1[30] = Xpel_3x1[3];
	dim_primary_task = 27;

	//----------------- redundant tasks
	RVALS(_Q_34x1[WST_34],  _Qp_33x1[WST_33],  pSharedMemory->kd[15]*(-_Qp_33x1[WST_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[WST_34]-_Q_34x1[WST_34]),		WSTpmax, WSTppmax, WSTmin, WSTmax, D2R, &X2_33x1[1], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RSP_34],  _Qp_33x1[RSP_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RSP_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RSP_34]-_Q_34x1[RSP_34]),   RSPpmax, RSPppmax, RSPmin, RSPmax, D2R, &X2_33x1[2], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RSR_34],  _Qp_33x1[RSR_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RSR_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RSR_34]-_Q_34x1[RSR_34]),   RSRpmax, RSRppmax, RSRmin, RSRmax, D2R, &X2_33x1[3], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RSY_34],  _Qp_33x1[RSY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RSY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RSY_34]-_Q_34x1[RSY_34]),   RSYpmax, RSYppmax, RSYmin, RSYmax, D2R, &X2_33x1[4], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[REB_34],  _Qp_33x1[REB_33],  pSharedMemory->kd[15]*(-_Qp_33x1[REB_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[REB_34]-_Q_34x1[REB_34]),   REBpmax, REBppmax, REBmin, REBmax, D2R, &X2_33x1[5], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RWY_34],  _Qp_33x1[RWY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RWY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RWY_34]-_Q_34x1[RWY_34]),   RWYpmax, RWYppmax, RWYmin, RWYmax, D2R, &X2_33x1[6], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RWP_34],  _Qp_33x1[RWP_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RWP_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RWP_34]-_Q_34x1[RWP_34]),   RWPpmax, RWPppmax, RWPmin, RWPmax, D2R, &X2_33x1[7], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSP_34],  _Qp_33x1[LSP_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LSP_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LSP_34]-_Q_34x1[LSP_34]),   LSPpmax, LSPppmax, LSPmin, LSPmax, D2R, &X2_33x1[8], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSR_34],  _Qp_33x1[LSR_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LSR_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LSR_34]-_Q_34x1[LSR_34]),   LSRpmax, LSRppmax, LSRmin, LSRmax, D2R, &X2_33x1[9], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSY_34],  _Qp_33x1[LSY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LSY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LSY_34]-_Q_34x1[LSY_34]),   LSYpmax, LSYppmax, LSYmin, LSYmax, D2R, &X2_33x1[10], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LEB_34],  _Qp_33x1[LEB_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LEB_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LEB_34]-_Q_34x1[LEB_34]),   LEBpmax, LEBppmax, LEBmin, LEBmax, D2R, &X2_33x1[11], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LWY_34],  _Qp_33x1[LWY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LWY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LWY_34]-_Q_34x1[LWY_34]),   LWYpmax, LWYppmax, LWYmin, LWYmax, D2R, &X2_33x1[12], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LWP_34],  _Qp_33x1[LWP_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LWP_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LWP_34]-_Q_34x1[LWP_34]),   LWPpmax, LWPppmax, LWPmin, LWPmax, D2R, &X2_33x1[13], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RWY2_34],  _Qp_33x1[RWY2_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RWY2_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RWY2_34]-_Q_34x1[RWY2_34]),   RWY2pmax, RWY2ppmax, RWY2min, RWY2max, D2R, &X2_33x1[14], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LWY2_34],  _Qp_33x1[LWY2_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LWY2_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LWY2_34]-_Q_34x1[LWY2_34]),   LWY2pmax, LWY2ppmax, LWY2min, LWY2max, D2R, &X2_33x1[15], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	X2_33x1[16] = Xpel_3x1[1];
	X2_33x1[17] = Xpel_3x1[2];
	X2_33x1[18] = Xpel_3x1[3];
	dim_redundant_task = 18;
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
		//_jT1_33x33[28][i] = 0.f;
		//_jT1_33x33[29][i] = 0.f;
		//_jT1_33x33[30][i] = 0.f;   
		for(j=1;j<=18;j++)
			_jT2_33x33[j][i] = 0.f;
	}
	_jT1_33x33[27][3] = 1.f;
  //_jT1_33x33[28][4] = 1.f;
  //_jT1_33x33[29][5] = 1.f;
  //_jT1_33x33[30][6] = 1.f;

	for(i=1; i<=27; i++)
	{
		_jT1_33x33[i][4] = 0.f;  // exclude pelvis orientation from the solution
		_jT1_33x33[i][5] = 0.f; 
		_jT1_33x33[i][6] = 0.f; 
	}

	for(i=13; i<=24; i++)
	{
		_jT1_33x33[i][7] = 0.f;  // exclude WST from the solution
	}

	for(j=20; j<=33; j++)
	{
		_jT1_33x33[25][j] = 0.f; // exclude the upper body from COM solution
		_jT1_33x33[26][j] = 0.f;
	}
	_jT1_33x33[25][7] = 0.f;
	_jT1_33x33[26][7] = 0.f;

	_jT2_33x33[1][WST_33] = 1.f;
	_jT2_33x33[2][RSP_33] = 1.f;
	_jT2_33x33[3][RSR_33] = 1.f;
	_jT2_33x33[4][RSY_33] = 1.f;
	_jT2_33x33[5][REB_33] = 1.f;
	_jT2_33x33[6][RWY_33] = 1.f;
	_jT2_33x33[7][RWP_33] = 1.f;
	_jT2_33x33[8][LSP_33] = 1.f;
	_jT2_33x33[9][LSR_33] = 1.f;	
	_jT2_33x33[10][LSY_33] = 1.f;
	_jT2_33x33[11][LEB_33] = 1.f;
	_jT2_33x33[12][LWY_33] = 1.f;
	_jT2_33x33[13][LWP_33] = 1.f;
	_jT2_33x33[14][RWY2_33] = 1.f;
	_jT2_33x33[15][LWY2_33] = 1.f;	
	_jT2_33x33[16][WX_33] = 1.f;
	_jT2_33x33[17][WY_33] = 1.f;
	_jT2_33x33[18][WZ_33] = 1.f;
	
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

	//pinv_svd((const float**)_jT2_33x33,18,33, _jT2inv_33x33);
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

	do
	{
		isLimited = 0;
		if(pSharedMemory->torque_mode_flag == 0 || pSharedMemory->torque_mode_flag == 3)
			for(i=1;i<=27;i++)
			{
				index_limited[i] = 0;
				qpp_limited[i] = 0.f;
				if(Qpp_33x1[i+6] >= ub_27x1[i])
				{
					isLimited = 1;
					index_limited[i] = 1;
					qpp_limited[i] = ub_27x1[i];
				}
				else if(Qpp_33x1[i+6] <= lb_27x1[i])
				{
					isLimited = 1;
					index_limited[i] = 1;
					qpp_limited[i] = lb_27x1[i];
				}

				if(isLimited==1)
				{
					printf("\n i: %d, joint: %d, p: %.4fdeg, v: %.4fdeg/s, a: %.4fdeg/s^2", _offline_traj.i, i+6, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D, Qpp_33x1[i+6]*R2D);
					printf("\n ub: %.4fdeg/s^2, lb: %.4fdeg/s^2", ub_27x1[i]*R2D, lb_27x1[i]*R2D);			
					return -2;
				}

				if(fabs(_Qp_33x1[i+6])>=QP_MAX)
				{
					printf("\n i: %d, joint: %d, p: %.4fdeg, v: %.4fdeg/s, a: %.4fdeg/s^2", _offline_traj.i, i+6, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D, Qpp_33x1[i+6]*R2D);
					printf("\n ub: %.4fdeg/s^2, lb: %.4fdeg/s^2", ub_27x1[i]*R2D, lb_27x1[i]*R2D);			
					return -2;
				}
			}
		
		if(isLimited==1)
		{
			// will be filled			
		}
	} while(isLimited==1);
	
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

	Joint[WST].RefAngleCurrent = _Q_34x1[WST_34]*R2D;

	Joint[RHY].RefAngleCurrent = (_Q_34x1[RHY_34]+rhy_compen)*R2D;
	Joint[RHR].RefAngleCurrent = (_Q_34x1[RHR_34]+rhr_compen)*R2D;
	Joint[RHP].RefAngleCurrent = _Q_34x1[RHP_34]*R2D;
	Joint[RKN].RefAngleCurrent = _Q_34x1[RKN_34]*R2D;
	Joint[RAP].RefAngleCurrent = _Q_34x1[RAP_34]*R2D;
	Joint[RAR].RefAngleCurrent = _Q_34x1[RAR_34]*R2D;
	Joint[LHY].RefAngleCurrent = (_Q_34x1[LHY_34]+lhy_compen)*R2D;
	Joint[LHR].RefAngleCurrent = (_Q_34x1[LHR_34]+lhr_compen)*R2D;
	Joint[LHP].RefAngleCurrent = _Q_34x1[LHP_34]*R2D;
	Joint[LKN].RefAngleCurrent = _Q_34x1[LKN_34]*R2D;
	Joint[LAP].RefAngleCurrent = _Q_34x1[LAP_34]*R2D;
	Joint[LAR].RefAngleCurrent = _Q_34x1[LAR_34]*R2D;

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

	//----------------------------------------------------------------- 100Hz
	if(half_flag != 0)
	{
		half_flag = 0;
		return 0;
	}
	else
		half_flag = 1;
	
	diff_mm((const float**)_jRH_6x33,6,33, (const float**)_jRH_old2_6x33, _jRHp_6x33);
	subs_m((const float**)_jRH_old_6x33,6,33, _jRH_old2_6x33);
	subs_m((const float**)_jRH_6x33,6,33, _jRH_old_6x33);
	mult_sm((const float**)_jRHp_6x33,6,33, 1.f/(2.f*2.f*DT), _jRHp_6x33);
	
	diff_mm((const float**)_jLH_6x33,6,33, (const float**)_jLH_old2_6x33, _jLHp_6x33);
	subs_m((const float**)_jLH_old_6x33,6,33, _jLH_old2_6x33);
	subs_m((const float**)_jLH_6x33,6,33, _jLH_old_6x33);
	mult_sm((const float**)_jLHp_6x33,6,33, 1.f/(2.f*2.f*DT), _jLHp_6x33);


	for(i=1; i<=34; i++)
		meas_Q_34x1[i] = _Q_34x1[i];

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
	meas_Qp_33x1[RWY2_33] = (meas_Q_34x1[RWY2_34]-Q_old2_34x1[RWY2_34])/(2.f*2.f*DT);

	meas_Qp_33x1[LSP_33] = (meas_Q_34x1[LSP_34]-Q_old2_34x1[LSP_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LSR_33] = (meas_Q_34x1[LSR_34]-Q_old2_34x1[LSR_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LSY_33] = (meas_Q_34x1[LSY_34]-Q_old2_34x1[LSY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LEB_33] = (meas_Q_34x1[LEB_34]-Q_old2_34x1[LEB_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LWY_33] = (meas_Q_34x1[LWY_34]-Q_old2_34x1[LWY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LWP_33] = (meas_Q_34x1[LWP_34]-Q_old2_34x1[LWP_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LWY2_33] = (meas_Q_34x1[LWY2_34]-Q_old2_34x1[LWY2_34])/(2.f*2.f*DT);


	//------------------------ joint limits
	for(i=20;i<=33;i++)
		if(fabs(meas_Qp_33x1[i])> QP_MAX)
		{
			RBpwmCommandHR2ch(CAN1, Joint[LSP].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[LSY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[LWY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[LWY2].JMC, 0, 0, 0);

			RBpwmCommandHR2ch(CAN1, Joint[RSP].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[RSY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[RWY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[RWY2].JMC, 0, 0, 0);

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
	Q_old2_34x1[RWY2_34] = Q_old_34x1[RWY2_34];

	Q_old2_34x1[LSP_34] = Q_old_34x1[LSP_34];
	Q_old2_34x1[LSR_34] = Q_old_34x1[LSR_34];
	Q_old2_34x1[LSY_34] = Q_old_34x1[LSY_34];
	Q_old2_34x1[LEB_34] = Q_old_34x1[LEB_34];
	Q_old2_34x1[LWY_34] = Q_old_34x1[LWY_34];
	Q_old2_34x1[LWP_34] = Q_old_34x1[LWP_34];
	Q_old2_34x1[LWY2_34] = Q_old_34x1[LWY2_34];

	Q_old_34x1[RSP_34] = meas_Q_34x1[RSP_34];
	Q_old_34x1[RSR_34] = meas_Q_34x1[RSR_34];
	Q_old_34x1[RSY_34] = meas_Q_34x1[RSY_34];
	Q_old_34x1[REB_34] = meas_Q_34x1[REB_34];
	Q_old_34x1[RWY_34] = meas_Q_34x1[RWY_34];
	Q_old_34x1[RWP_34] = meas_Q_34x1[RWP_34];
	Q_old_34x1[RWY2_34] = meas_Q_34x1[RWY2_34];

	Q_old_34x1[LSP_34] = meas_Q_34x1[LSP_34];
	Q_old_34x1[LSR_34] = meas_Q_34x1[LSR_34];
	Q_old_34x1[LSY_34] = meas_Q_34x1[LSY_34];
	Q_old_34x1[LEB_34] = meas_Q_34x1[LEB_34];
	Q_old_34x1[LWY_34] = meas_Q_34x1[LWY_34];
	Q_old_34x1[LWP_34] = meas_Q_34x1[LWP_34];
	Q_old_34x1[LWY2_34] = meas_Q_34x1[LWY2_34];


	if(pSharedMemory->torque_mode_flag == 0)
	{
		for(i=1;i<=3;i++)
		{
			pRH2_3x1[i] = _pRH_3x1[i];
			pLH2_3x1[i] = _pLH_3x1[i];
			qRH2_4x1[i] = _qRH_4x1[i];
			qLH2_4x1[i] = _qLH_4x1[i];
		}
		qRH2_4x1[4] = _qRH_4x1[4];
		qLH2_4x1[4] = _qLH_4x1[4];

		pre_gain = 0.f;
		count = 0;
	}
	else if(pSharedMemory->torque_mode_flag == 1 || pSharedMemory->torque_mode_flag == 2)
	{
		for(i=1;i<=3;i++)
		{
			pRH1_3x1[i] = _pRH_3x1[i];
			pLH1_3x1[i] = _pLH_3x1[i];
			qRH1_4x1[i] = _qRH_4x1[i];
			qLH1_4x1[i] = _qLH_4x1[i];
		}
		qRH1_4x1[4] = _qRH_4x1[4];
		qLH1_4x1[4] = _qLH_4x1[4];
	
		if(pSharedMemory->torque_mode_flag == 1)
		{
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
		_Q_34x1[RWY_34] = meas_Q_34x1[RWY_34];
		_Q_34x1[RWP_34] = meas_Q_34x1[RWP_34];
		_Q_34x1[RWY2_34] = meas_Q_34x1[RWY2_34];

		_Q_34x1[LSP_34] = meas_Q_34x1[LSP_34];
		_Q_34x1[LSR_34] = meas_Q_34x1[LSR_34];
		_Q_34x1[LSY_34] = meas_Q_34x1[LSY_34];
		_Q_34x1[LEB_34] = meas_Q_34x1[LEB_34];
		_Q_34x1[LWY_34] = meas_Q_34x1[LWY_34];
		_Q_34x1[LWP_34] = meas_Q_34x1[LWP_34];
		_Q_34x1[LWY2_34] = meas_Q_34x1[LWY2_34];

		_Qp_33x1[RSP_33] = meas_Qp_33x1[RSP_33];
		_Qp_33x1[RSR_33] = meas_Qp_33x1[RSR_33];
		_Qp_33x1[RSY_33] = meas_Qp_33x1[RSY_33];
		_Qp_33x1[REB_33] = meas_Qp_33x1[REB_33];
		_Qp_33x1[RWY_33] = meas_Qp_33x1[RWY_33];
		_Qp_33x1[RWP_33] = meas_Qp_33x1[RWP_33];
		_Qp_33x1[RWY2_33] = meas_Qp_33x1[RWY2_33];

		_Qp_33x1[LSP_33] = meas_Qp_33x1[LSP_33];
		_Qp_33x1[LSR_33] = meas_Qp_33x1[LSR_33];
		_Qp_33x1[LSY_33] = meas_Qp_33x1[LSY_33];
		_Qp_33x1[LEB_33] = meas_Qp_33x1[LEB_33];
		_Qp_33x1[LWY_33] = meas_Qp_33x1[LWY_33];
		_Qp_33x1[LWP_33] = meas_Qp_33x1[LWP_33];
		_Qp_33x1[LWY2_33] = meas_Qp_33x1[LWY2_33];

		Joint[RSP].RefAngleCurrent = _Q_34x1[RSP_34]*R2D;
		Joint[RSR].RefAngleCurrent = _Q_34x1[RSR_34]*R2D - OFFSET_RSR;
		Joint[RSY].RefAngleCurrent = _Q_34x1[RSY_34]*R2D;
		Joint[REB].RefAngleCurrent = _Q_34x1[REB_34]*R2D - OFFSET_REB;
		Joint[RWY].RefAngleCurrent = _Q_34x1[RWY_34]*R2D;
		Joint[RWP].RefAngleCurrent = _Q_34x1[RWP_34]*R2D;
		Joint[RWY2].RefAngleCurrent = _Q_34x1[RWY2_34]*R2D;
		Joint[LSP].RefAngleCurrent = _Q_34x1[LSP_34]*R2D;
		Joint[LSR].RefAngleCurrent = _Q_34x1[LSR_34]*R2D - OFFSET_LSR;
		Joint[LSY].RefAngleCurrent = _Q_34x1[LSY_34]*R2D;
		Joint[LEB].RefAngleCurrent = _Q_34x1[LEB_34]*R2D - OFFSET_LEB;	
		Joint[LWY].RefAngleCurrent = _Q_34x1[LWY_34]*R2D;
		Joint[LWP].RefAngleCurrent = _Q_34x1[LWP_34]*R2D;
		Joint[LWY2].RefAngleCurrent = _Q_34x1[LWY2_34]*R2D;
		
		SendRunStopCMD(Joint[LSP].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[LSP].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[LSY].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[LSY].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[LWY].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[LWY].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[LWY2].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[LWY2].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[RSP].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[RSP].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[RSY].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[RSY].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[RWY].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[RWY].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[RWY2].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[RWY2].JMC, 0, 0, 0);
	
		pre_gain = 0.f;
		pSharedMemory->torque_mode_flag = 0;
		printf("\n position mode");
	}

	_Q_34x1[RSP_34] =  pre_gain*meas_Q_34x1[RSP_34] + (1.f-pre_gain)*_Q_34x1[RSP_34];
	_Q_34x1[RSR_34] =  pre_gain*meas_Q_34x1[RSR_34] + (1.f-pre_gain)*_Q_34x1[RSR_34];
	_Q_34x1[RSY_34] =  pre_gain*meas_Q_34x1[RSY_34] + (1.f-pre_gain)*_Q_34x1[RSY_34];
	_Q_34x1[REB_34] =  pre_gain*meas_Q_34x1[REB_34] + (1.f-pre_gain)*_Q_34x1[REB_34];
	_Q_34x1[RWY_34] =  pre_gain*meas_Q_34x1[RWY_34] + (1.f-pre_gain)*_Q_34x1[RWY_34];
	_Q_34x1[RWP_34] =  pre_gain*meas_Q_34x1[RWP_34] + (1.f-pre_gain)*_Q_34x1[RWP_34];
	_Q_34x1[RWY2_34] =  pre_gain*meas_Q_34x1[RWY2_34] + (1.f-pre_gain)*_Q_34x1[RWY2_34];
	
	_Q_34x1[LSP_34] =  pre_gain*meas_Q_34x1[LSP_34] + (1.f-pre_gain)*_Q_34x1[LSP_34];
	_Q_34x1[LSR_34] =  pre_gain*meas_Q_34x1[LSR_34] + (1.f-pre_gain)*_Q_34x1[LSR_34];
	_Q_34x1[LSY_34] =  pre_gain*meas_Q_34x1[LSY_34] + (1.f-pre_gain)*_Q_34x1[LSY_34];
	_Q_34x1[LEB_34] =  pre_gain*meas_Q_34x1[LEB_34] + (1.f-pre_gain)*_Q_34x1[LEB_34];
	_Q_34x1[LWY_34] =  pre_gain*meas_Q_34x1[LWY_34] + (1.f-pre_gain)*_Q_34x1[LWY_34];
	_Q_34x1[LWP_34] =  pre_gain*meas_Q_34x1[LWP_34] + (1.f-pre_gain)*_Q_34x1[LWP_34];
	_Q_34x1[LWY2_34] =  pre_gain*meas_Q_34x1[LWY2_34] + (1.f-pre_gain)*_Q_34x1[LWY2_34];

	_Qp_33x1[RSP_33] =  pre_gain*meas_Qp_33x1[RSP_33] + (1.f-pre_gain)*_Qp_33x1[RSP_33];
	_Qp_33x1[RSR_33] =  pre_gain*meas_Qp_33x1[RSR_33] + (1.f-pre_gain)*_Qp_33x1[RSR_33];
	_Qp_33x1[RSY_33] =  pre_gain*meas_Qp_33x1[RSY_33] + (1.f-pre_gain)*_Qp_33x1[RSY_33];
	_Qp_33x1[REB_33] =  pre_gain*meas_Qp_33x1[REB_33] + (1.f-pre_gain)*_Qp_33x1[REB_33];
	_Qp_33x1[RWY_33] =  pre_gain*meas_Qp_33x1[RWY_33] + (1.f-pre_gain)*_Qp_33x1[RWY_33];
	_Qp_33x1[RWP_33] =  pre_gain*meas_Qp_33x1[RWP_33] + (1.f-pre_gain)*_Qp_33x1[RWP_33];
	_Qp_33x1[RWY2_33] =  pre_gain*meas_Qp_33x1[RWY2_33] + (1.f-pre_gain)*_Qp_33x1[RWY2_33];
	
	_Qp_33x1[LSP_33] =  pre_gain*meas_Qp_33x1[LSP_33] + (1.f-pre_gain)*_Qp_33x1[LSP_33];
	_Qp_33x1[LSR_33] =  pre_gain*meas_Qp_33x1[LSR_33] + (1.f-pre_gain)*_Qp_33x1[LSR_33];
	_Qp_33x1[LSY_33] =  pre_gain*meas_Qp_33x1[LSY_33] + (1.f-pre_gain)*_Qp_33x1[LSY_33];
	_Qp_33x1[LEB_33] =  pre_gain*meas_Qp_33x1[LEB_33] + (1.f-pre_gain)*_Qp_33x1[LEB_33];
	_Qp_33x1[LWY_33] =  pre_gain*meas_Qp_33x1[LWY_33] + (1.f-pre_gain)*_Qp_33x1[LWY_33];
	_Qp_33x1[LWP_33] =  pre_gain*meas_Qp_33x1[LWP_33] + (1.f-pre_gain)*_Qp_33x1[LWP_33];
	_Qp_33x1[LWY2_33] =  pre_gain*meas_Qp_33x1[LWY2_33] + (1.f-pre_gain)*_Qp_33x1[LWY2_33];

	if(pSharedMemory->torque_mode_flag == 1 || pSharedMemory->torque_mode_flag == 2)
	{
		for(i=1;i<=33;i++)
			duty_joint_limit_33x1[i] = 0.f;
		getDuty4JointLimit(_Q_34x1, duty_joint_limit_33x1);
		getFricCompen(_Qp_33x1, fric_compen_33x1);
		getGravityTorque(_Q_34x1, gravity_33x1);
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

		for(i=1;i<=6;i++)
			for(j=1;j<=33;j++)
			{
				_jT1_33x33[i][j] = _jRH_6x33[i][j];			
				_jT1_33x33[i+6][j] = _jLH_6x33[i][j];			
			}

		for(i=1; i<=12; i++)
		{
			_jT1_33x33[i][1] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][2] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][3] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][4] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][5] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][6] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][7] = 0.f;  // exclude WST from the solution
		}

		if(pSharedMemory->ladder_demo == 0)
		{
			neutral_Q_34x1[RSP_34] = _Q_34x1[RSP_34];
			neutral_Q_34x1[RSR_34] = _Q_34x1[RSR_34];
			neutral_Q_34x1[RSY_34] = _Q_34x1[RSY_34];
			neutral_Q_34x1[REB_34] = _Q_34x1[REB_34];
			neutral_Q_34x1[RWY_34] = _Q_34x1[RWY_34];
			neutral_Q_34x1[RWP_34] = _Q_34x1[RWP_34];
			neutral_Q_34x1[RWY2_34] = _Q_34x1[RWY2_34];
			
			neutral_Q_34x1[LSP_34] = _Q_34x1[LSP_34];
			neutral_Q_34x1[LSR_34] = _Q_34x1[LSR_34];
			neutral_Q_34x1[LSY_34] = _Q_34x1[LSY_34];
			neutral_Q_34x1[LEB_34] = _Q_34x1[LEB_34];
			neutral_Q_34x1[LWY_34] = _Q_34x1[LWY_34];
			neutral_Q_34x1[LWP_34] = _Q_34x1[LWP_34];
			neutral_Q_34x1[LWY2_34] = _Q_34x1[LWY2_34];

			pRH2_3x1[1] = _pRH_3x1[1];
			pRH2_3x1[2] = _pRH_3x1[2];
			pRH2_3x1[3] = _pRH_3x1[3];
			qRH2_4x1[1] = _qRH_4x1[1];
			qRH2_4x1[2] = _qRH_4x1[2];
			qRH2_4x1[3] = _qRH_4x1[3];
			qRH2_4x1[4] = _qRH_4x1[4];
							
			pLH2_3x1[1] = _pLH_3x1[1];
			pLH2_3x1[2] = _pLH_3x1[2];
			pLH2_3x1[3] = _pLH_3x1[3];
			qLH2_4x1[1] = _qLH_4x1[1];
			qLH2_4x1[2] = _qLH_4x1[2];
			qLH2_4x1[3] = _qLH_4x1[3];
			qLH2_4x1[4] = _qLH_4x1[4];

			_TEMP3_34x34[1][1] = Xrh_6x1[1] = 0.f;
			_TEMP3_34x34[1][2] = Xrh_6x1[2] = 0.f;
			_TEMP3_34x34[1][3] = Xrh_6x1[3] = 0.f;
			_TEMP3_34x34[1][4] = Xrh_6x1[4] = 0.f;
			_TEMP3_34x34[1][5] = Xrh_6x1[5] = 0.f;
			_TEMP3_34x34[1][6] = Xrh_6x1[6] = 0.f;
			_TEMP3_34x34[1][7] = Xlh_6x1[1] = 0.f;
			_TEMP3_34x34[1][8] = Xlh_6x1[2] = 0.f;
			_TEMP3_34x34[1][9] = Xlh_6x1[3] = 0.f;
			_TEMP3_34x34[1][10] = Xlh_6x1[4] = 0.f;
			_TEMP3_34x34[1][11] = Xlh_6x1[5] = 0.f;
			_TEMP3_34x34[1][12] = Xlh_6x1[6] = 0.f;
		}
		else if(pSharedMemory->ladder_demo == 1)
		{
			des_pRH_3x1[1] = pRH2_3x1[1];
			des_pRH_3x1[2] = pRH2_3x1[2];
			des_pRH_3x1[3] = pRH2_3x1[3];
			des_qRH_4x1[1] = qRH2_4x1[1];
			des_qRH_4x1[2] = qRH2_4x1[2];
			des_qRH_4x1[3] = qRH2_4x1[3];
			des_qRH_4x1[4] = qRH2_4x1[4];
			des_vRH_3x1[1] = 0.f;
			des_vRH_3x1[2] = 0.f;
			des_vRH_3x1[3] = 0.f;
			des_wRH_3x1[1] = 0.f;
			des_wRH_3x1[2] = 0.f;
			des_wRH_3x1[3] = 0.f;
				
			des_pLH_3x1[1] = pLH2_3x1[1];
			des_pLH_3x1[2] = pLH2_3x1[2];
			des_pLH_3x1[3] = pLH2_3x1[3];
			des_qLH_4x1[1] = qLH2_4x1[1];
			des_qLH_4x1[2] = qLH2_4x1[2];
			des_qLH_4x1[3] = qLH2_4x1[3];
			des_qLH_4x1[4] = qLH2_4x1[4];
			des_vLH_3x1[1] = 0.f;
			des_vLH_3x1[2] = 0.f;
			des_vLH_3x1[3] = 0.f;
			des_wLH_3x1[1] = 0.f;
			des_wLH_3x1[2] = 0.f;
			des_wLH_3x1[3] = 0.f;

			diff_vv(des_pRH_3x1,3, _pRH_3x1, ftemp3_7x1);
			mult_mv((const float**)_jRH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
			diff_vv(des_vRH_3x1,3, ftemp2_7x1, ftemp1_7x1);
			Xrh_6x1[1] = pSharedMemory->kp[0]*ftemp3_7x1[1] + pSharedMemory->kd[0]*ftemp1_7x1[1];
			Xrh_6x1[2] = pSharedMemory->kp[1]*ftemp3_7x1[2] + pSharedMemory->kd[1]*ftemp1_7x1[2];
			Xrh_6x1[3] = pSharedMemory->kp[2]*ftemp3_7x1[3] + pSharedMemory->kd[2]*ftemp1_7x1[3];
			QTdel(des_qRH_4x1, _qRH_4x1, ftemp3_7x1);
			diff_vv(des_wRH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
			sum_svsv(pSharedMemory->kp[3],ftemp3_7x1,3, pSharedMemory->kd[3],ftemp4_7x1, &Xrh_6x1[3]);

			diff_vv(des_pLH_3x1,3, _pLH_3x1, ftemp3_7x1);
			mult_mv((const float**)_jLH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
			diff_vv(des_vLH_3x1,3, ftemp2_7x1, ftemp1_7x1);
			Xlh_6x1[1] = pSharedMemory->kp[4]*ftemp3_7x1[1] + pSharedMemory->kd[4]*ftemp1_7x1[1];
			Xlh_6x1[2] = pSharedMemory->kp[5]*ftemp3_7x1[2] + pSharedMemory->kd[5]*ftemp1_7x1[2];
			Xlh_6x1[3] = pSharedMemory->kp[6]*ftemp3_7x1[3] + pSharedMemory->kd[6]*ftemp1_7x1[3];
			QTdel(des_qLH_4x1, _qLH_4x1, ftemp3_7x1);
			diff_vv(des_wLH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
			sum_svsv(pSharedMemory->kp[7],ftemp3_7x1,3, pSharedMemory->kd[7],ftemp4_7x1, &Xlh_6x1[3]);

			_TEMP3_34x34[1][1] = Xrh_6x1[1];
			_TEMP3_34x34[1][2] = Xrh_6x1[2];
			_TEMP3_34x34[1][3] = pSharedMemory->ref_fRH_3x1[3];
			_TEMP3_34x34[1][4] = Xrh_6x1[4];
			_TEMP3_34x34[1][5] = Xrh_6x1[5];
			_TEMP3_34x34[1][6] = Xrh_6x1[6];
			_TEMP3_34x34[1][7] = Xlh_6x1[1]; //pSharedMemory->ref_fLH_3x1[1];
			_TEMP3_34x34[1][8] = Xlh_6x1[2]; //pSharedMemory->ref_fLH_3x1[2];
			_TEMP3_34x34[1][9] = pSharedMemory->ref_fLH_3x1[3]; //pSharedMemory->ref_fLH_3x1[3];
			_TEMP3_34x34[1][10] = Xlh_6x1[4];
			_TEMP3_34x34[1][11] = Xlh_6x1[5];
			_TEMP3_34x34[1][12] = Xlh_6x1[6];
		}
		else if(pSharedMemory->ladder_demo == 2)
		{
			neutral_Q_34x1[LSP_34] = _Q_34x1[LSP_34];
			neutral_Q_34x1[LSR_34] = _Q_34x1[LSR_34];
			neutral_Q_34x1[LSY_34] = _Q_34x1[LSY_34];
			neutral_Q_34x1[LEB_34] = _Q_34x1[LEB_34];
			neutral_Q_34x1[LWY_34] = _Q_34x1[LWY_34];
			neutral_Q_34x1[LWP_34] = _Q_34x1[LWP_34];
			neutral_Q_34x1[LWY2_34] = _Q_34x1[LWY2_34];
					
			pLH2_3x1[1] = _pLH_3x1[1];
			pLH2_3x1[2] = _pLH_3x1[2];
			pLH2_3x1[3] = _pLH_3x1[3];
			qLH2_4x1[1] = _qLH_4x1[1];
			qLH2_4x1[2] = _qLH_4x1[2];
			qLH2_4x1[3] = _qLH_4x1[3];
			qLH2_4x1[4] = _qLH_4x1[4];

			des_pRH_3x1[1] = pRH2_3x1[1];
			des_pRH_3x1[2] = pRH2_3x1[2];
			des_pRH_3x1[3] = pRH2_3x1[3];
			des_qRH_4x1[1] = qRH2_4x1[1];
			des_qRH_4x1[2] = qRH2_4x1[2];
			des_qRH_4x1[3] = qRH2_4x1[3];
			des_qRH_4x1[4] = qRH2_4x1[4];
			des_vRH_3x1[1] = 0.f;
			des_vRH_3x1[2] = 0.f;
			des_vRH_3x1[3] = 0.f;
			des_wRH_3x1[1] = 0.f;
			des_wRH_3x1[2] = 0.f;
			des_wRH_3x1[3] = 0.f;
		
			diff_vv(des_pRH_3x1,3, _pRH_3x1, ftemp3_7x1);
			mult_mv((const float**)_jRH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
			diff_vv(des_vRH_3x1,3, ftemp2_7x1, ftemp1_7x1);
			Xrh_6x1[1] = pSharedMemory->kp[0]*ftemp3_7x1[1] + pSharedMemory->kd[0]*ftemp1_7x1[1];
			Xrh_6x1[2] = pSharedMemory->kp[1]*ftemp3_7x1[2] + pSharedMemory->kd[1]*ftemp1_7x1[2];
			Xrh_6x1[3] = pSharedMemory->kp[2]*ftemp3_7x1[3] + pSharedMemory->kd[2]*ftemp1_7x1[3];
			QTdel(des_qRH_4x1, _qRH_4x1, ftemp3_7x1);
			diff_vv(des_wRH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
			sum_svsv(pSharedMemory->kp[3],ftemp3_7x1,3, pSharedMemory->kd[3],ftemp4_7x1, &Xrh_6x1[3]);

			_TEMP3_34x34[1][1] = Xrh_6x1[1];
			_TEMP3_34x34[1][2] = Xrh_6x1[2];
			_TEMP3_34x34[1][3] = Xrh_6x1[3];
			_TEMP3_34x34[1][4] = Xrh_6x1[4];
			_TEMP3_34x34[1][5] = Xrh_6x1[5];
			_TEMP3_34x34[1][6] = Xrh_6x1[6];
			_TEMP3_34x34[1][7] = Xlh_6x1[1] = 0.f;
			_TEMP3_34x34[1][8] = Xlh_6x1[2] = 0.f;
			_TEMP3_34x34[1][9] = Xlh_6x1[3] = 0.f;
			_TEMP3_34x34[1][10] = Xlh_6x1[4] = 0.f;
			_TEMP3_34x34[1][11] = Xlh_6x1[5] = 0.f;
			_TEMP3_34x34[1][12] = Xlh_6x1[6] = 0.f;
		}		
		
		trans(1.f, (const float**)_jT1_33x33,12,33, _TEMP2_34x34);
		mult_mv((const float**)_TEMP2_34x34,33,12, _TEMP3_34x34[1], ct_33x1);

		if(pinv_SR((const float**)_jT1_33x33, 12, 33, (float)1.e-10, _jT1inv_33x33) != 0)
		{
			printf("\n pinv error!");
			return -2; // singularity occurred
		}
		mult_mm((const float**)_jT1inv_33x33,33,12, (const float**)_jT1_33x33, 33, _TEMP1_34x34);
		diff_mm((const float**)_EYE_33, 33,33, (const float**)_TEMP1_34x34, _N1_33x33);


		for(i=1;i<=33;i++)
			_TEMP3_34x34[1][i] = 0.f;
		_TEMP3_34x34[1][20] = pSharedMemory->kd[8]*(-_Qp_33x1[RSP_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RSP_34]-_Q_34x1[RSP_34]);
		_TEMP3_34x34[1][21] = pSharedMemory->kd[8]*(-_Qp_33x1[RSR_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RSR_34]-_Q_34x1[RSR_34]);
		_TEMP3_34x34[1][22] = pSharedMemory->kd[8]*(-_Qp_33x1[RSY_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RSY_34]-_Q_34x1[RSY_34]);
		_TEMP3_34x34[1][23] = pSharedMemory->kd[8]*(-_Qp_33x1[REB_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[REB_34]-_Q_34x1[REB_34]);
		_TEMP3_34x34[1][24] = pSharedMemory->kd[8]*(-_Qp_33x1[RWY_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RWY_34]-_Q_34x1[RWY_34]);
		_TEMP3_34x34[1][25] = pSharedMemory->kd[8]*(-_Qp_33x1[RWP_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RWP_34]-_Q_34x1[RWP_34]);
		_TEMP3_34x34[1][26] = pSharedMemory->kd[8]*(-_Qp_33x1[LSP_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LSP_34]-_Q_34x1[LSP_34]);
		_TEMP3_34x34[1][27] = pSharedMemory->kd[8]*(-_Qp_33x1[LSR_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LSR_34]-_Q_34x1[LSR_34]);
		_TEMP3_34x34[1][28] = pSharedMemory->kd[8]*(-_Qp_33x1[LSY_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LSY_34]-_Q_34x1[LSY_34]);
		_TEMP3_34x34[1][29] = pSharedMemory->kd[8]*(-_Qp_33x1[LEB_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LEB_34]-_Q_34x1[LEB_34]);
		_TEMP3_34x34[1][30] = pSharedMemory->kd[8]*(-_Qp_33x1[LWY_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LWY_34]-_Q_34x1[LWY_34]);
		_TEMP3_34x34[1][31] = pSharedMemory->kd[8]*(-_Qp_33x1[LWP_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LWP_34]-_Q_34x1[LWP_34]);
		_TEMP3_34x34[1][32] = pSharedMemory->kd[8]*(-_Qp_33x1[RWY2_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RWY2_34]-_Q_34x1[RWY2_34]);
		_TEMP3_34x34[1][33] = pSharedMemory->kd[8]*(-_Qp_33x1[LWY2_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LWY2_34]-_Q_34x1[LWY2_34]);

		if(pSharedMemory->ladder_demo==1 || pSharedMemory->ladder_demo==2)
		{
			if(pSharedMemory->ladder_demo==1)
			{
				_TEMP3_34x34[1][20] = 0.f;
				_TEMP3_34x34[1][21] = 0.f;
				_TEMP3_34x34[1][22] = 0.f;
				_TEMP3_34x34[1][23] = 0.f;
				//_TEMP3_34x34[1][24] = 0.f;
				_TEMP3_34x34[1][25] = 0.f;
				_TEMP3_34x34[1][26] = 0.f;
				_TEMP3_34x34[1][27] = 0.f;
				_TEMP3_34x34[1][28] = 0.f;
				_TEMP3_34x34[1][29] = 0.f;
				//_TEMP3_34x34[1][30] = 0.f;
				_TEMP3_34x34[1][31] = 0.f;
				_TEMP3_34x34[1][32] = 0.f;
				_TEMP3_34x34[1][33] = 0.f;
			}
			else if(pSharedMemory->ladder_demo==2)
			{
				_TEMP3_34x34[1][26] = 0.f;
				_TEMP3_34x34[1][27] = 0.f;
				_TEMP3_34x34[1][28] = 0.f;
				_TEMP3_34x34[1][29] = 0.f;
				_TEMP3_34x34[1][30] = 0.f;
				_TEMP3_34x34[1][31] = 0.f;
				_TEMP3_34x34[1][33] = 0.f;
			}
			mult_mv((const float**)_N1_33x33,33,33, _TEMP3_34x34[1], _TEMP1_34x34[1]);
			sum_vv(ct_33x1,33, (const float*)_TEMP1_34x34[1], ct_33x1);
		}

		ct_33x1[RSP_33] = _gain_task[RSP_33]*ct_33x1[RSP_33]+_gain_gravity[RSP_33]*gravity_33x1[RSP_33];
		ct_33x1[RSR_33] = _gain_task[RSR_33]*ct_33x1[RSR_33]+_gain_gravity[RSR_33]*gravity_33x1[RSR_33];
		ct_33x1[RSY_33] = _gain_task[RSY_33]*ct_33x1[RSY_33]+_gain_gravity[RSY_33]*gravity_33x1[RSY_33];
		ct_33x1[REB_33] = _gain_task[REB_33]*ct_33x1[REB_33]+_gain_gravity[REB_33]*gravity_33x1[REB_33];
		ct_33x1[RWY_33] = _gain_task[RWY_33]*ct_33x1[RWY_33]+_gain_gravity[RWY_33]*gravity_33x1[RWY_33];
		ct_33x1[RWP_33] = _gain_task[RWP_33]*ct_33x1[RWP_33]+_gain_gravity[RWP_33]*gravity_33x1[RWP_33];
		ct_33x1[RWY2_33] = _gain_task[RWY2_33]*ct_33x1[RWY2_33]+_gain_gravity[RWY2_33]*gravity_33x1[RWY2_33];

		ct_33x1[LSP_33] = _gain_task[LSP_33]*ct_33x1[LSP_33]+_gain_gravity[LSP_33]*gravity_33x1[LSP_33];
		ct_33x1[LSR_33] = _gain_task[LSR_33]*ct_33x1[LSR_33]+_gain_gravity[LSR_33]*gravity_33x1[LSR_33];
		ct_33x1[LSY_33] = _gain_task[LSY_33]*ct_33x1[LSY_33]+_gain_gravity[LSY_33]*gravity_33x1[LSY_33];
		ct_33x1[LEB_33] = _gain_task[LEB_33]*ct_33x1[LEB_33]+_gain_gravity[LEB_33]*gravity_33x1[LEB_33];
		ct_33x1[LWY_33] = _gain_task[LWY_33]*ct_33x1[LWY_33]+_gain_gravity[LWY_33]*gravity_33x1[LWY_33];
		ct_33x1[LWP_33] = _gain_task[LWP_33]*ct_33x1[LWP_33]+_gain_gravity[LWP_33]*gravity_33x1[LWP_33];
		ct_33x1[LWY2_33] = _gain_task[LWY2_33]*ct_33x1[LWY2_33]+_gain_gravity[LWY2_33]*gravity_33x1[LWY2_33];

		pSharedMemory->disp_ct_33x1[RSP_33] = ct_33x1[RSP_33];
		pSharedMemory->disp_ct_33x1[RSR_33] = ct_33x1[RSR_33];
		pSharedMemory->disp_ct_33x1[RSY_33] = ct_33x1[RSY_33];
		pSharedMemory->disp_ct_33x1[REB_33] = ct_33x1[REB_33];
		pSharedMemory->disp_ct_33x1[RWY_33] = ct_33x1[RWY_33];
		pSharedMemory->disp_ct_33x1[RWP_33] = ct_33x1[RWP_33];
		pSharedMemory->disp_ct_33x1[RWY2_33] = ct_33x1[RWY2_33];
		
		pSharedMemory->disp_ct_33x1[LSP_33] = ct_33x1[LSP_33];
		pSharedMemory->disp_ct_33x1[LSR_33] = ct_33x1[LSR_33];
		pSharedMemory->disp_ct_33x1[LSY_33] = ct_33x1[LSY_33];
		pSharedMemory->disp_ct_33x1[LEB_33] = ct_33x1[LEB_33];
		pSharedMemory->disp_ct_33x1[LWY_33] = ct_33x1[LWY_33];
		pSharedMemory->disp_ct_33x1[LWP_33] = ct_33x1[LWP_33];
		pSharedMemory->disp_ct_33x1[LWY2_33] = ct_33x1[LWY2_33];

		
		pSharedMemory->disp_duty_33x1[RSP_33] = duty_33x1[RSP_33] = limitDuty(0.4f, torque2duty(RSP, pre_gain*ct_33x1[RSP_33]) + pre_gain*fric_compen_33x1[RSP_33] + duty_joint_limit_33x1[RSP_33]);
		pSharedMemory->disp_duty_33x1[RSR_33] = duty_33x1[RSR_33] = limitDuty(0.4f, torque2duty(RSR, pre_gain*ct_33x1[RSR_33]) + pre_gain*fric_compen_33x1[RSR_33] + duty_joint_limit_33x1[RSR_33]);
		pSharedMemory->disp_duty_33x1[RSY_33] = duty_33x1[RSY_33] = limitDuty(0.4f, torque2duty(RSY, pre_gain*ct_33x1[RSY_33]) + pre_gain*fric_compen_33x1[RSY_33] + duty_joint_limit_33x1[RSY_33]);
		pSharedMemory->disp_duty_33x1[REB_33] = duty_33x1[REB_33] = limitDuty(0.4f, torque2duty(REB, pre_gain*ct_33x1[REB_33]) + pre_gain*fric_compen_33x1[REB_33] + duty_joint_limit_33x1[REB_33]);
		pSharedMemory->disp_duty_33x1[RWY_33] = duty_33x1[RWY_33] = limitDuty(0.4f, torque2duty(RWY, pre_gain*ct_33x1[RWY_33]) + pre_gain*fric_compen_33x1[RWY_33] + duty_joint_limit_33x1[RWY_33]);
		pSharedMemory->disp_duty_33x1[RWP_33] = duty_33x1[RWP_33] = limitDuty(0.4f, torque2duty(RWP, pre_gain*ct_33x1[RWP_33]) + pre_gain*fric_compen_33x1[RWP_33] + duty_joint_limit_33x1[RWP_33]);
		pSharedMemory->disp_duty_33x1[RWY2_33] = duty_33x1[RWY2_33] = limitDuty(0.4f, torque2duty(RWY2, pre_gain*ct_33x1[RWY2_33]) + pre_gain*fric_compen_33x1[RWY2_33] + duty_joint_limit_33x1[RWY2_33]);

		pSharedMemory->disp_duty_33x1[LSP_33] = duty_33x1[LSP_33] = limitDuty(0.4f, torque2duty(LSP, pre_gain*ct_33x1[LSP_33]) + pre_gain*fric_compen_33x1[LSP_33] + duty_joint_limit_33x1[LSP_33]);
		pSharedMemory->disp_duty_33x1[LSR_33] = duty_33x1[LSR_33] = limitDuty(0.4f, torque2duty(LSR, pre_gain*ct_33x1[LSR_33]) + pre_gain*fric_compen_33x1[LSR_33] + duty_joint_limit_33x1[LSR_33]);
		pSharedMemory->disp_duty_33x1[LSY_33] = duty_33x1[LSY_33] = limitDuty(0.4f, torque2duty(LSY, pre_gain*ct_33x1[LSY_33]) + pre_gain*fric_compen_33x1[LSY_33] + duty_joint_limit_33x1[LSY_33]);
		pSharedMemory->disp_duty_33x1[LEB_33] = duty_33x1[LEB_33] = limitDuty(0.4f, torque2duty(LEB, pre_gain*ct_33x1[LEB_33]) + pre_gain*fric_compen_33x1[LEB_33] + duty_joint_limit_33x1[LEB_33]);
		pSharedMemory->disp_duty_33x1[LWY_33] = duty_33x1[LWY_33] = limitDuty(0.4f, torque2duty(LWY, pre_gain*ct_33x1[LWY_33]) + pre_gain*fric_compen_33x1[LWY_33] + duty_joint_limit_33x1[LWY_33]);
		pSharedMemory->disp_duty_33x1[LWP_33] = duty_33x1[LWP_33] = limitDuty(0.4f, torque2duty(LWP, pre_gain*ct_33x1[LWP_33]) + pre_gain*fric_compen_33x1[LWP_33] + duty_joint_limit_33x1[LWP_33]);
		pSharedMemory->disp_duty_33x1[LWY2_33] = duty_33x1[LWY2_33] = limitDuty(0.4f, torque2duty(LWY2, pre_gain*ct_33x1[LWY2_33]) + pre_gain*fric_compen_33x1[LWY2_33] + duty_joint_limit_33x1[LWY2_33]);
		
		RBpwmCommandHR2ch(CAN1, Joint[RSP].JMC, (int)(PWM_SIGN_RSP*1000.f*duty_33x1[RSP_33]), (int)(PWM_SIGN_RSR*1000.f*duty_33x1[RSR_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[RSY].JMC, (int)(PWM_SIGN_RSY*1000.f*duty_33x1[RSY_33]), (int)(PWM_SIGN_REB*1000.f*duty_33x1[REB_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[RWY].JMC, (int)(PWM_SIGN_RWY*1000.f*duty_33x1[RWY_33]), (int)(PWM_SIGN_RWP*1000.f*duty_33x1[RWP_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[RWY2].JMC, (int)(PWM_SIGN_RWY2*1000.f*duty_33x1[RWY2_33]), 0, 1);

		RBpwmCommandHR2ch(CAN1, Joint[LSP].JMC, (int)(PWM_SIGN_LSP*1000.f*duty_33x1[LSP_33]), (int)(PWM_SIGN_LSR*1000.f*duty_33x1[LSR_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[LSY].JMC, (int)(PWM_SIGN_LSY*1000.f*duty_33x1[LSY_33]), (int)(PWM_SIGN_LEB*1000.f*duty_33x1[LEB_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[LWY].JMC, (int)(PWM_SIGN_LWY*1000.f*duty_33x1[LWY_33]), (int)(PWM_SIGN_LWP*1000.f*duty_33x1[LWP_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[LWY2].JMC, (int)(PWM_SIGN_LWY2*1000.f*duty_33x1[LWY2_33]), 0, 1);
		
	}
	
	return 0;	
}


float one_cos(float t_sec, float mag, float T_sec)
{
	float rtn;

	if(t_sec < 0.f)
		return 0.f;
	else if(t_sec < T_sec)
		rtn = mag*0.5f*(1.f - (float)cosf(PI/T_sec*t_sec));
	else
		rtn = mag;

	return rtn;
}


int one_cos_orientation(float t_sec, const float *qt0_4x1, const float *qt1_4x1, float T_sec, float *result_4x1)
{
	float qt0_bar_4x1[5], qt_del_4x1[5], rv[4];

	QTbar(qt0_4x1, qt0_bar_4x1);
	QTcross(qt0_bar_4x1, qt1_4x1, qt_del_4x1);
	QT2RV(qt_del_4x1, rv);
	if(t_sec < 0.f)
	{
		result_4x1[1] = qt0_4x1[1];
		result_4x1[2] = qt0_4x1[2];
		result_4x1[3] = qt0_4x1[3];
		result_4x1[4] = qt0_4x1[4];

	}
	else if(t_sec < T_sec)
	{
		rv[0] = one_cos(t_sec, rv[0], T_sec);
		RV2QT(rv, qt_del_4x1);
		QTcross(qt0_4x1, qt_del_4x1, result_4x1);
	}
	else
	{
		result_4x1[1] = qt1_4x1[1];
		result_4x1[2] = qt1_4x1[2];
		result_4x1[3] = qt1_4x1[3];
		result_4x1[4] = qt1_4x1[4];
	}

	return 0;
}





int QT2RV(const float *qt_4x1, float *rv)
{
	float temp;
	rv[0] = acosf(qt_4x1[1])*2.f;

	if(fabs(sinf(rv[0]/2.f)) < EPS)
	{
		rv[1] = qt_4x1[2];
		rv[2] = qt_4x1[3];
		rv[3] = qt_4x1[4];
	}
	else
	{
		rv[1] = qt_4x1[2]/sinf(rv[0]/2.f);
		rv[2] = qt_4x1[3]/sinf(rv[0]/2.f);
		rv[3] = qt_4x1[4]/sinf(rv[0]/2.f);

		temp = norm_v(rv,3);
		rv[1] /= temp;
		rv[2] /= temp;
		rv[3] /= temp;
	}

	return 0;
}


int RV2QT(const float *rv, float *qt_4x1)
{
	float temp = sqrtf(rv[1]*rv[1]+rv[2]*rv[2]+rv[3]*rv[3]);

	if(temp > 0.5f)
	{
		qt_4x1[1] = cosf(rv[0]/2.f);
		qt_4x1[2] = rv[1]/temp*sinf(rv[0]/2.f);
		qt_4x1[3] = rv[2]/temp*sinf(rv[0]/2.f);
		qt_4x1[4] = rv[3]/temp*sinf(rv[0]/2.f);		
	}
	else
	{
		qt_4x1[1] = 1.f;
		qt_4x1[2] = 0.f;
		qt_4x1[3] = 0.f;
		qt_4x1[4] = 0.f;
	}
	
	return 0;
}



int WBIK_DRC_steering(unsigned int n)  
{
	int i,j;
	
	float Qpp_33x1[34], meas_Q_34x1[35], meas_Qp_33x1[34];
	float X1_33x1[34], Xrf_6x1[7], Xlf_6x1[7], Xrh_6x1[7], Xlh_6x1[7], Xcom_3x1[4], Xpel_3x1[4], Xwst, Xpelz, X2_33x1[34];  // task variables
	float ub_27x1[28], lb_27x1[28], qpp_limited[28];  // joint upper-bound, lower-bound
	int index_limited[28];
	
	float des_pCOM_3x1[4], des_vCOM_3x1[4];
	float des_pRF_3x1[4], des_pLF_3x1[4], des_vRF_3x1[4], des_vLF_3x1[4];
	float des_qRF_4x1[5], des_qLF_4x1[5], des_wRF_3x1[4], des_wLF_3x1[4];
	float des_pRH_3x1[4], des_pLH_3x1[4], des_vRH_3x1[4], des_vLH_3x1[4];
	float des_qRH_4x1[5], des_qLH_4x1[5], des_wRH_3x1[4], des_wLH_3x1[4];
	float des_WST, des_WSTp, des_qPEL_4x1[5];
	float des_pPELz, des_vPELz;
	float vCOM_3x1[4];

	BOOL isLimited = FALSE;
	int dim_primary_task, dim_redundant_task;

	float lL, lR, fRFz, fLFz;
	float rhr_compen = 0.f;
	float lhr_compen = 0.f;
	float rhy_compen = 0.f;
	float lhy_compen = 0.f;
	

	float fric_compen_33x1[34], ct_33x1[34], gravity_33x1[34], duty_33x1[34], duty_joint_limit_33x1[34];

	static char half_flag, reset_neutral_flag;
	static int count, steer_count, count_walking;
	static float neutral_Q_34x1[35], Q_old_34x1[35], Q_old2_34x1[35];
	static float pCOM1_3x1[4], pPELz1, pRF1_3x1[4], pLF1_3x1[4], qRF1_4x1[5], qLF1_4x1[5];
	static float pRH1_3x1[4], pLH1_3x1[4], qRH1_4x1[5], qLH1_4x1[5];
	static float pRH2_3x1[4], pLH2_3x1[4], qRH2_4x1[5], qLH2_4x1[5];
	static float t, t_steer;
	static float qSteer_errSum, qSteer_old, qSteer_old2, qSteer0, qSteer1;

	float dpRF_3x1[4], dpLF_3x1[4], dqRF_4x1[5], dqLF_4x1[5];
	float dpRH_3x1[4], dpLH_3x1[4], dqRH_4x1[5], dqLH_4x1[5];
	float dpPELz, dpCOM_3x1[4];

	float ftemp1_7x1[8], ftemp2_7x1[8], ftemp3_7x1[8], ftemp4_7x1[8];
	float pre_gain = 0.f;

	float meas_qSteer, meas_wSteer, des_qSteer;

	float zmp_f, zmp_s;
	float supp_center_x, supp_center_y;
	float r11, r12, r21, r22;
	float damp_f = 0.;
	float damp_s = 0.;
	float des_pCOM_s, des_pCOM_f;


	if(_FKineUpdatedFlag != 1)
		return -4;
	
	if(n<=2)
	{
		neutral_Q_34x1[WST_34] = Joint[WST].RefAngleCurrent*D2R;

		neutral_Q_34x1[RSP_34] = Joint[RSP].RefAngleCurrent*D2R;
		neutral_Q_34x1[RSR_34] = (Joint[RSR].RefAngleCurrent + OFFSET_RSR)*D2R;
		neutral_Q_34x1[RSY_34] = Joint[RSY].RefAngleCurrent*D2R;
		neutral_Q_34x1[REB_34] = (Joint[REB].RefAngleCurrent + OFFSET_REB)*D2R;
		neutral_Q_34x1[RWY_34] = Joint[RWY].RefAngleCurrent*D2R;
		neutral_Q_34x1[RWP_34] = Joint[RWP].RefAngleCurrent*D2R;
		neutral_Q_34x1[RWY2_34] = Joint[RWY2].RefAngleCurrent*D2R;
		
		neutral_Q_34x1[LSP_34] = Joint[LSP].RefAngleCurrent*D2R;
		neutral_Q_34x1[LSR_34] = (Joint[LSR].RefAngleCurrent + OFFSET_LSR)*D2R;
		neutral_Q_34x1[LSY_34] = Joint[LSY].RefAngleCurrent*D2R;
		neutral_Q_34x1[LEB_34] = (Joint[LEB].RefAngleCurrent + OFFSET_LEB)*D2R;
		neutral_Q_34x1[LWY_34] = Joint[LWY].RefAngleCurrent*D2R;
		neutral_Q_34x1[LWP_34] = Joint[LWP].RefAngleCurrent*D2R;
		neutral_Q_34x1[LWY2_34] = Joint[LWY2].RefAngleCurrent*D2R;

		_pPEL0_3x1[1] = _Q_34x1[1];
		_pPEL0_3x1[2] = _Q_34x1[2];
		pPELz1 = _pPEL0_3x1[3] = _Q_34x1[3];

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

		pRH1_3x1[1] = _pRH0_3x1[1] = _pRH_3x1[1];
		pRH1_3x1[2] = _pRH0_3x1[2] = _pRH_3x1[2];
		pRH1_3x1[3] = _pRH0_3x1[3] = _pRH_3x1[3];

		qRH1_4x1[1] = _qRH0_4x1[1] = _qRH_4x1[1];
		qRH1_4x1[2] = _qRH0_4x1[2] = _qRH_4x1[2];
		qRH1_4x1[3] = _qRH0_4x1[3] = _qRH_4x1[3];
		qRH1_4x1[4] = _qRH0_4x1[4] = _qRH_4x1[4];

		pLH1_3x1[1] = _pLH0_3x1[1] = _pLH_3x1[1];
		pLH1_3x1[2] = _pLH0_3x1[2] = _pLH_3x1[2];
		pLH1_3x1[3] = _pLH0_3x1[3] = _pLH_3x1[3];

		qLH1_4x1[1] = _qLH0_4x1[1] = _qLH_4x1[1];
		qLH1_4x1[2] = _qLH0_4x1[2] = _qLH_4x1[2];
		qLH1_4x1[3] = _qLH0_4x1[3] = _qLH_4x1[3];
		qLH1_4x1[4] = _qLH0_4x1[4] = _qLH_4x1[4];

		meas_Q_34x1[RSP_34] = ((float)Joint[RSP].EncoderValue)/Joint[RSP].PPR*D2R;
		meas_Q_34x1[RSR_34] = ((float)Joint[RSR].EncoderValue)/Joint[RSR].PPR*D2R + OFFSET_RSR*D2R;
		meas_Q_34x1[RSY_34] = ((float)Joint[RSY].EncoderValue)/Joint[RSY].PPR*D2R;
		meas_Q_34x1[REB_34] = ((float)Joint[REB].EncoderValue)/Joint[REB].PPR*D2R + OFFSET_REB*D2R;
		meas_Q_34x1[RWY_34] = ((float)Joint[RWY].EncoderValue)/Joint[RWY].PPR*D2R;
		meas_Q_34x1[RWP_34] = ((float)Joint[RWP].EncoderValue)/Joint[RWP].PPR*D2R;
		meas_Q_34x1[RWY2_34] = ((float)Joint[RWY2].EncoderValue)/Joint[RWY2].PPR*D2R;

		meas_Q_34x1[LSP_34] = ((float)Joint[LSP].EncoderValue)/Joint[LSP].PPR*D2R;
		meas_Q_34x1[LSR_34] = ((float)Joint[LSR].EncoderValue)/Joint[LSR].PPR*D2R + OFFSET_LSR*D2R;
		meas_Q_34x1[LSY_34] = ((float)Joint[LSY].EncoderValue)/Joint[LSY].PPR*D2R;
		meas_Q_34x1[LEB_34] = ((float)Joint[LEB].EncoderValue)/Joint[LEB].PPR*D2R + OFFSET_LEB*D2R;
		meas_Q_34x1[LWY_34] = ((float)Joint[LWY].EncoderValue)/Joint[LWY].PPR*D2R;
		meas_Q_34x1[LWP_34] = ((float)Joint[LWP].EncoderValue)/Joint[LWP].PPR*D2R;
		meas_Q_34x1[LWY2_34] = ((float)Joint[LWY2].EncoderValue)/Joint[LWY2].PPR*D2R;

		meas_qSteer = ((float)Joint[NKY].EncoderValue)/(-2883.f)*360.f*D2R;

		Q_old2_34x1[RSP_34] = Q_old_34x1[RSP_34] = meas_Q_34x1[RSP_34];
		Q_old2_34x1[RSR_34] = Q_old_34x1[RSR_34] = meas_Q_34x1[RSR_34];
		Q_old2_34x1[RSY_34] = Q_old_34x1[RSY_34] = meas_Q_34x1[RSY_34];
		Q_old2_34x1[REB_34] = Q_old_34x1[REB_34] = meas_Q_34x1[REB_34];
		Q_old2_34x1[RWY_34] = Q_old_34x1[RWY_34] = meas_Q_34x1[RWY_34];
		Q_old2_34x1[RWP_34] = Q_old_34x1[RWP_34] = meas_Q_34x1[RWP_34];
		Q_old2_34x1[RWY2_34] = Q_old_34x1[RWY2_34] = meas_Q_34x1[RWY2_34];

		Q_old2_34x1[LSP_34] = Q_old_34x1[LSP_34] = meas_Q_34x1[LSP_34];
		Q_old2_34x1[LSR_34] = Q_old_34x1[LSR_34] = meas_Q_34x1[LSR_34];
		Q_old2_34x1[LSY_34] = Q_old_34x1[LSY_34] = meas_Q_34x1[LSY_34];
		Q_old2_34x1[LEB_34] = Q_old_34x1[LEB_34] = meas_Q_34x1[LEB_34];
		Q_old2_34x1[LWY_34] = Q_old_34x1[LWY_34] = meas_Q_34x1[LWY_34];
		Q_old2_34x1[LWP_34] = Q_old_34x1[LWP_34] = meas_Q_34x1[LWP_34];
		Q_old2_34x1[LWY2_34] = Q_old_34x1[LWY2_34] = meas_Q_34x1[LWY2_34];

		qSteer_old2 = qSteer_old = meas_qSteer;

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
		t = 0.f;
		count = 0;
		count_walking = 0;
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
	
	dpPELz = 0.f;

	dpCOM_3x1[1] = 0.f;
	dpCOM_3x1[2] = 0.f;
	dpCOM_3x1[3] = 0.f;
	
	dpRF_3x1[1] = 0.f;
	dpRF_3x1[2] = 0.f;
	dpRF_3x1[3] = 0.f;
	
	dpLF_3x1[1] = 0.f;
	dpLF_3x1[2] = 0.f;
	dpLF_3x1[3] = 0.f;
	
	dpRH_3x1[1] = 0.f;
	dpRH_3x1[2] = 0.f;
	dpRH_3x1[3] = 0.f;
	
	dpLH_3x1[1] = 0.f;
	dpLH_3x1[2] = 0.f;
	dpLH_3x1[3] = 0.f;
	
	dqRF_4x1[1] = 1.f;
	dqRF_4x1[2] = 0.f;
	dqRF_4x1[3] = 0.f;
	dqRF_4x1[4] = 0.f;
	
	dqLF_4x1[1] = 1.f;
	dqLF_4x1[2] = 0.f;
	dqLF_4x1[3] = 0.f;
	dqLF_4x1[4] = 0.f;
	
	dqRH_4x1[1] = 1.f;
	dqRH_4x1[2] = 0.f;
	dqRH_4x1[3] = 0.f;
	dqRH_4x1[4] = 0.f;
	
	dqLH_4x1[1] = 1.f;
	dqLH_4x1[2] = 0.f;
	dqLH_4x1[3] = 0.f;
	dqLH_4x1[4] = 0.f;

	reset_neutral_flag = 0;
	if(pSharedMemory->position_mode_flag == 0) // staying
	{	
		t = 0.f;
	}
	else if(pSharedMemory->position_mode_flag == 1)  // move hands
	{	
		
		dpRH_3x1[1] = one_cos(t, pSharedMemory->dpRH[0], pSharedMemory->move_sec);
		dpRH_3x1[2] = one_cos(t, pSharedMemory->dpRH[1], pSharedMemory->move_sec);
		dpRH_3x1[3] = one_cos(t, pSharedMemory->dpRH[2], pSharedMemory->move_sec);

		dpLH_3x1[1] = one_cos(t, pSharedMemory->dpLH[0], pSharedMemory->move_sec);
		dpLH_3x1[2] = one_cos(t, pSharedMemory->dpLH[1], pSharedMemory->move_sec);
		dpLH_3x1[3] = one_cos(t, pSharedMemory->dpLH[2], pSharedMemory->move_sec);

		ftemp1_7x1[0] = one_cos(t, pSharedMemory->drvRH[0], pSharedMemory->move_sec);
		ftemp1_7x1[1] = pSharedMemory->drvRH[1];
		ftemp1_7x1[2] = pSharedMemory->drvRH[2];
		ftemp1_7x1[3] = pSharedMemory->drvRH[3];
		RV2QT(ftemp1_7x1, dqRH_4x1);

		ftemp1_7x1[0] = one_cos(t, pSharedMemory->drvLH[0], pSharedMemory->move_sec);
		ftemp1_7x1[1] = pSharedMemory->drvLH[1];
		ftemp1_7x1[2] = pSharedMemory->drvLH[2];
		ftemp1_7x1[3] = pSharedMemory->drvLH[3];
		RV2QT(ftemp1_7x1, dqLH_4x1);
		
		t += DT;

		if(t > pSharedMemory->move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 2)  //move RF and LF
	{
		dpRF_3x1[1] = one_cos(t, pSharedMemory->dpRF[0], pSharedMemory->move_sec);
		dpRF_3x1[2] = one_cos(t, pSharedMemory->dpRF[1], pSharedMemory->move_sec);
		dpRF_3x1[3] = one_cos(t, pSharedMemory->dpRF[2], pSharedMemory->move_sec);

		dpLF_3x1[1] = one_cos(t, pSharedMemory->dpLF[0], pSharedMemory->move_sec);
		dpLF_3x1[2] = one_cos(t, pSharedMemory->dpLF[1], pSharedMemory->move_sec);
		dpLF_3x1[3] = one_cos(t, pSharedMemory->dpLF[2], pSharedMemory->move_sec);

		ftemp1_7x1[0] = one_cos(t, pSharedMemory->drvRF[0], pSharedMemory->move_sec);
		ftemp1_7x1[1] = pSharedMemory->drvRF[1];
		ftemp1_7x1[2] = pSharedMemory->drvRF[2];
		ftemp1_7x1[3] = pSharedMemory->drvRF[3];
		RV2QT(ftemp1_7x1, dqRF_4x1);

		ftemp1_7x1[0] = one_cos(t, pSharedMemory->drvLF[0], pSharedMemory->move_sec);
		ftemp1_7x1[1] = pSharedMemory->drvLF[1];
		ftemp1_7x1[2] = pSharedMemory->drvLF[2];
		ftemp1_7x1[3] = pSharedMemory->drvLF[3];
		RV2QT(ftemp1_7x1, dqLF_4x1);

		t += DT;
		if(t > pSharedMemory->move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 3)  //move COM
	{	
		dpCOM_3x1[1] = one_cos(t, pSharedMemory->dpCOM[0], pSharedMemory->move_sec);
		dpCOM_3x1[2] = one_cos(t, pSharedMemory->dpCOM[1], pSharedMemory->move_sec);
	
		t += DT;
		if(t > pSharedMemory->move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 4)  //move pPELz
	{
		dpPELz = one_cos(t, pSharedMemory->dpPELz, pSharedMemory->move_sec);
	
		t += DT;
		if(t > pSharedMemory->move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 5)  //offline traj
	{
		dpPELz = pSharedMemory->off_traj_dpPELz[pSharedMemory->off_traj_count];
		dpCOM_3x1[1] = pSharedMemory->off_traj_dpCOM[pSharedMemory->off_traj_count][0];
		dpCOM_3x1[2] = pSharedMemory->off_traj_dpCOM[pSharedMemory->off_traj_count][1];
		dpRH_3x1[1] = pSharedMemory->off_traj_dpRH[pSharedMemory->off_traj_count][0];
		dpRH_3x1[2] = pSharedMemory->off_traj_dpRH[pSharedMemory->off_traj_count][1];
		dpRH_3x1[3] = pSharedMemory->off_traj_dpRH[pSharedMemory->off_traj_count][2];
		dpLH_3x1[1] = pSharedMemory->off_traj_dpLH[pSharedMemory->off_traj_count][0];
		dpLH_3x1[2] = pSharedMemory->off_traj_dpLH[pSharedMemory->off_traj_count][1];
		dpLH_3x1[3] = pSharedMemory->off_traj_dpLH[pSharedMemory->off_traj_count][2];
		dpRF_3x1[1] = pSharedMemory->off_traj_dpRF[pSharedMemory->off_traj_count][0];
		dpRF_3x1[2] = pSharedMemory->off_traj_dpRF[pSharedMemory->off_traj_count][1];
		dpRF_3x1[3] = pSharedMemory->off_traj_dpRF[pSharedMemory->off_traj_count][2];
		dpLF_3x1[1] = pSharedMemory->off_traj_dpLF[pSharedMemory->off_traj_count][0];
		dpLF_3x1[2] = pSharedMemory->off_traj_dpLF[pSharedMemory->off_traj_count][1];
		dpLF_3x1[3] = pSharedMemory->off_traj_dpLF[pSharedMemory->off_traj_count][2];
		RV2QT(pSharedMemory->off_traj_drvRH[pSharedMemory->off_traj_count], dqRH_4x1);
		RV2QT(pSharedMemory->off_traj_drvLH[pSharedMemory->off_traj_count], dqLH_4x1);
		RV2QT(pSharedMemory->off_traj_drvRF[pSharedMemory->off_traj_count], dqRF_4x1);
		RV2QT(pSharedMemory->off_traj_drvLF[pSharedMemory->off_traj_count], dqLF_4x1);
		pSharedMemory->off_traj_count++;
		if(pSharedMemory->off_traj_count >= pSharedMemory->off_traj_length)
		{
			reset_neutral_flag = 1;
			pSharedMemory->off_traj_count = 0;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	
	des_WST = 0.f;
	des_WSTp = 0.f;
	des_qPEL_4x1[1] = 1.f;
	des_qPEL_4x1[2] = 0.f;
	des_qPEL_4x1[3] = 0.f;
	des_qPEL_4x1[4] = 0.f;

	des_pPELz = pPELz1 + dpPELz;

	des_pCOM_3x1[1] = pCOM1_3x1[1] + dpCOM_3x1[1];
	des_pCOM_3x1[2] = pCOM1_3x1[2] + dpCOM_3x1[2];
	des_pCOM_3x1[3] = pCOM1_3x1[2] + dpCOM_3x1[3];

	des_pRF_3x1[1] = pRF1_3x1[1] + dpRF_3x1[1];
	des_pRF_3x1[2] = pRF1_3x1[2] + dpRF_3x1[2];
	des_pRF_3x1[3] = pRF1_3x1[3] + dpRF_3x1[3];
	
	des_pLF_3x1[1] = pLF1_3x1[1] + dpLF_3x1[1];
	des_pLF_3x1[2] = pLF1_3x1[2] + dpLF_3x1[2];
	des_pLF_3x1[3] = pLF1_3x1[3] + dpLF_3x1[3];
	
	des_pRH_3x1[1] = pRH1_3x1[1] + dpRH_3x1[1];
	des_pRH_3x1[2] = pRH1_3x1[2] + dpRH_3x1[2];
	des_pRH_3x1[3] = pRH1_3x1[3] + dpRH_3x1[3];

	des_pLH_3x1[1] = pLH1_3x1[1] + dpLH_3x1[1];
	des_pLH_3x1[2] = pLH1_3x1[2] + dpLH_3x1[2];
	des_pLH_3x1[3] = pLH1_3x1[3] + dpLH_3x1[3];

	QTcross(dqRH_4x1, qRH1_4x1, des_qRH_4x1);
	QTcross(dqLH_4x1, qLH1_4x1, des_qLH_4x1);
	QTcross(dqRF_4x1, qRF1_4x1, des_qRF_4x1);
	QTcross(dqLF_4x1, qLF1_4x1, des_qLF_4x1);
		

	if(reset_neutral_flag==1)
	{
		pPELz1 = des_pPELz;

		pCOM1_3x1[1] = des_pCOM_3x1[1];
		pCOM1_3x1[2] = des_pCOM_3x1[2];

		pRF1_3x1[1] = des_pRF_3x1[1];
		pRF1_3x1[2] = des_pRF_3x1[2];
		pRF1_3x1[3] = des_pRF_3x1[3];

		pLF1_3x1[1] = des_pLF_3x1[1];
		pLF1_3x1[2] = des_pLF_3x1[2];
		pLF1_3x1[3] = des_pLF_3x1[3];

		pRH1_3x1[1] = des_pRH_3x1[1];
		pRH1_3x1[2] = des_pRH_3x1[2];
		pRH1_3x1[3] = des_pRH_3x1[3];

		pLH1_3x1[1] = des_pLH_3x1[1];
		pLH1_3x1[2] = des_pLH_3x1[2];
		pLH1_3x1[3] = des_pLH_3x1[3];

		qRF1_4x1[1] = des_qRF_4x1[1];
		qRF1_4x1[2] = des_qRF_4x1[2];
		qRF1_4x1[3] = des_qRF_4x1[3];
		qRF1_4x1[4] = des_qRF_4x1[4];

		qLF1_4x1[1] = des_qLF_4x1[1];
		qLF1_4x1[2] = des_qLF_4x1[2];
		qLF1_4x1[3] = des_qLF_4x1[3];
		qLF1_4x1[4] = des_qLF_4x1[4];

		qRH1_4x1[1] = des_qRH_4x1[1];
		qRH1_4x1[2] = des_qRH_4x1[2];
		qRH1_4x1[3] = des_qRH_4x1[3];
		qRH1_4x1[4] = des_qRH_4x1[4];

		qLH1_4x1[1] = des_qLH_4x1[1];
		qLH1_4x1[2] = des_qLH_4x1[2];
		qLH1_4x1[3] = des_qLH_4x1[3];
		qLH1_4x1[4] = des_qLH_4x1[4];

		printf("\n move done");
		reset_neutral_flag = 0;
	}

	//------------------ damping controller
	supp_center_x = (des_pLF_3x1[1] + des_pRF_3x1[1])*0.5f;
	supp_center_y = (des_pLF_3x1[2] + des_pRF_3x1[2])*0.5f;

	ftemp1_7x1[0] = (float)sqrt((float)pow(des_pLF_3x1[1]-des_pRF_3x1[1], 2.f) + (float)pow(des_pLF_3x1[2]-des_pRF_3x1[2], 2.f));
	
	r12 = (des_pLF_3x1[1]-des_pRF_3x1[1])/ftemp1_7x1[0];
	r22 = (des_pLF_3x1[2]-des_pRF_3x1[2])/ftemp1_7x1[0];
	r11 = r22;
	r21 = -r12;

	ftemp1_7x1[0] = _Q_34x1[1] + _pZMP_3x1[1] - supp_center_x;
	ftemp1_7x1[1] = _Q_34x1[2] + _pZMP_3x1[2] - supp_center_y;
	zmp_s = r11*ftemp1_7x1[0] + r21*ftemp1_7x1[1];
	zmp_f = r12*ftemp1_7x1[0] + r22*ftemp1_7x1[1];

	ftemp1_7x1[0] = des_pCOM_3x1[1] - supp_center_x;
	ftemp1_7x1[1] = des_pCOM_3x1[2] - supp_center_y;
	des_pCOM_s = r11*ftemp1_7x1[0] + r21*ftemp1_7x1[1];
	des_pCOM_f = r12*ftemp1_7x1[0] + r22*ftemp1_7x1[1];

	damp_f =  -(_Kfb_DSP_F_DRC_1x4[1]*_Xhat_DSP_F_4x1[1]+_Kfb_DSP_F_DRC_1x4[2]*_Xhat_DSP_F_4x1[2]+_Kfb_DSP_F_DRC_1x4[3]*_Xhat_DSP_F_4x1[3]+_Kfb_DSP_F_DRC_1x4[4]*_Xhat_DSP_F_4x1[4]);
	damp_s =  -(_Kfb_DSP_S_DRC_1x4[1]*_Xhat_DSP_S_4x1[1]+_Kfb_DSP_S_DRC_1x4[2]*_Xhat_DSP_S_4x1[2]+_Kfb_DSP_S_DRC_1x4[3]*_Xhat_DSP_S_4x1[3]+_Kfb_DSP_S_DRC_1x4[4]*_Xhat_DSP_S_4x1[4]);
	
	if(count_walking < 400)
	{	
		ftemp1_7x1[0] = (float)count_walking/((float)400.f);
		damp_f *= (float)pow(ftemp1_7x1[0], 3.f);
		damp_s *= (float)pow(ftemp1_7x1[0], 3.f);
		count_walking++;
	}

	des_pCOM_s += damp_s;
	des_pCOM_f += damp_f;
	
	ftemp1_7x1[0] = _Xhat_DSP_F_4x1[1] + DT*(_Ahat_DSP_F_DRC_4x4[1][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[1][2]*_Xhat_DSP_F_4x1[2] +
		_Ahat_DSP_F_DRC_4x4[1][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[1][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[1]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[1]*zmp_f);
	ftemp1_7x1[1] = _Xhat_DSP_F_4x1[2] + DT*(_Ahat_DSP_F_DRC_4x4[2][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[2][2]*_Xhat_DSP_F_4x1[2] +
		_Ahat_DSP_F_DRC_4x4[2][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[2][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[2]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[2]*zmp_f);
	ftemp1_7x1[2] = _Xhat_DSP_F_4x1[3] + DT*(_Ahat_DSP_F_DRC_4x4[3][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[3][2]*_Xhat_DSP_F_4x1[2] +
		_Ahat_DSP_F_DRC_4x4[3][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[3][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[3]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[3]*zmp_f);
	_Xhat_DSP_F_4x1[4] = _Xhat_DSP_F_4x1[4] + DT*(_Ahat_DSP_F_DRC_4x4[4][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[4][2]*_Xhat_DSP_F_4x1[2] +
		_Ahat_DSP_F_DRC_4x4[4][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[4][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[4]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[4]*zmp_f);
	_Xhat_DSP_F_4x1[1] = ftemp1_7x1[0];
	_Xhat_DSP_F_4x1[2] = ftemp1_7x1[1];
	_Xhat_DSP_F_4x1[3] = ftemp1_7x1[2];

	ftemp1_7x1[0] = _Xhat_DSP_S_4x1[1] + DT*(_Ahat_DSP_S_DRC_4x4[1][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[1][2]*_Xhat_DSP_S_4x1[2] +
		_Ahat_DSP_S_DRC_4x4[1][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[1][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[1]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[1]*zmp_s);
	ftemp1_7x1[1] = _Xhat_DSP_S_4x1[2] + DT*(_Ahat_DSP_S_DRC_4x4[2][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[2][2]*_Xhat_DSP_S_4x1[2] +
		_Ahat_DSP_S_DRC_4x4[2][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[2][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[2]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[2]*zmp_s);
	ftemp1_7x1[2] = _Xhat_DSP_S_4x1[3] + DT*(_Ahat_DSP_S_DRC_4x4[3][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[3][2]*_Xhat_DSP_S_4x1[2] +
		_Ahat_DSP_S_DRC_4x4[3][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[3][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[3]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[3]*zmp_s);
	_Xhat_DSP_S_4x1[4] = _Xhat_DSP_S_4x1[4] + DT*(_Ahat_DSP_S_DRC_4x4[4][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[4][2]*_Xhat_DSP_S_4x1[2] +
		_Ahat_DSP_S_DRC_4x4[4][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[4][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[4]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[4]*zmp_s);
	_Xhat_DSP_S_4x1[1] = ftemp1_7x1[0];
	_Xhat_DSP_S_4x1[2] = ftemp1_7x1[1];
	_Xhat_DSP_S_4x1[3] = ftemp1_7x1[2];						

	if(pSharedMemory->temp_dsp_damp_on_flag == 1)
	{
		des_pCOM_3x1[1] = r12*des_pCOM_f + r11*des_pCOM_s + supp_center_x;
		des_pCOM_3x1[2] = r22*des_pCOM_f + r21*des_pCOM_s + supp_center_y;
	}
	//-------------------------------------

	des_vPELz = 0.f;

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
	diff_vv(des_pRH_3x1,3, _pRH_3x1, ftemp3_7x1);
	mult_mv((const float**)_jRH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
	diff_vv(des_vRH_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[14], ftemp3_7x1,3, pSharedMemory->kd[14],ftemp1_7x1, ftemp3_7x1);
	mult_mv((const float**)_jRHp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xrh_6x1);

	QTdel(des_qRH_4x1, _qRH_4x1, ftemp3_7x1);	
	diff_vv(des_wRH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(pSharedMemory->kp[14],ftemp3_7x1,3, pSharedMemory->kd[14],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xrh_6x1[3]);
	
	diff_vv(des_pLH_3x1,3, _pLH_3x1, ftemp3_7x1);  
	mult_mv((const float**)_jLH_6x33,6,33, _Qp_33x1, ftemp2_7x1);  
	diff_vv(des_vLH_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[14], ftemp3_7x1,3, pSharedMemory->kd[14],ftemp1_7x1, ftemp3_7x1);  
	mult_mv((const float**)_jLHp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	  
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xlh_6x1);

	QTdel(des_qLH_4x1, _qLH_4x1, ftemp3_7x1);	
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
	//X1_33x1[28] = Xpel_3x1[1];
	//X1_33x1[29] = Xpel_3x1[2];
	//X1_33x1[30] = Xpel_3x1[3];
	dim_primary_task = 27;

	//----------------- redundant tasks
	RVALS(_Q_34x1[WST_34],  _Qp_33x1[WST_33],  pSharedMemory->kd[15]*(-_Qp_33x1[WST_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[WST_34]-_Q_34x1[WST_34]),		WSTpmax, WSTppmax, WSTmin, WSTmax, D2R, &X2_33x1[1], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RSP_34],  _Qp_33x1[RSP_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RSP_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RSP_34]-_Q_34x1[RSP_34]),   RSPpmax, RSPppmax, RSPmin, RSPmax, D2R, &X2_33x1[2], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RSR_34],  _Qp_33x1[RSR_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RSR_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RSR_34]-_Q_34x1[RSR_34]),   RSRpmax, RSRppmax, RSRmin, RSRmax, D2R, &X2_33x1[3], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RSY_34],  _Qp_33x1[RSY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RSY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RSY_34]-_Q_34x1[RSY_34]),   RSYpmax, RSYppmax, RSYmin, RSYmax, D2R, &X2_33x1[4], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[REB_34],  _Qp_33x1[REB_33],  pSharedMemory->kd[15]*(-_Qp_33x1[REB_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[REB_34]-_Q_34x1[REB_34]),   REBpmax, REBppmax, REBmin, REBmax, D2R, &X2_33x1[5], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RWY_34],  _Qp_33x1[RWY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RWY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RWY_34]-_Q_34x1[RWY_34]),   RWYpmax, RWYppmax, RWYmin, RWYmax, D2R, &X2_33x1[6], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RWP_34],  _Qp_33x1[RWP_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RWP_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RWP_34]-_Q_34x1[RWP_34]),   RWPpmax, RWPppmax, RWPmin, RWPmax, D2R, &X2_33x1[7], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSP_34],  _Qp_33x1[LSP_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LSP_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LSP_34]-_Q_34x1[LSP_34]),   LSPpmax, LSPppmax, LSPmin, LSPmax, D2R, &X2_33x1[8], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSR_34],  _Qp_33x1[LSR_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LSR_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LSR_34]-_Q_34x1[LSR_34]),   LSRpmax, LSRppmax, LSRmin, LSRmax, D2R, &X2_33x1[9], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSY_34],  _Qp_33x1[LSY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LSY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LSY_34]-_Q_34x1[LSY_34]),   LSYpmax, LSYppmax, LSYmin, LSYmax, D2R, &X2_33x1[10], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LEB_34],  _Qp_33x1[LEB_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LEB_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LEB_34]-_Q_34x1[LEB_34]),   LEBpmax, LEBppmax, LEBmin, LEBmax, D2R, &X2_33x1[11], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LWY_34],  _Qp_33x1[LWY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LWY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LWY_34]-_Q_34x1[LWY_34]),   LWYpmax, LWYppmax, LWYmin, LWYmax, D2R, &X2_33x1[12], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LWP_34],  _Qp_33x1[LWP_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LWP_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LWP_34]-_Q_34x1[LWP_34]),   LWPpmax, LWPppmax, LWPmin, LWPmax, D2R, &X2_33x1[13], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RWY2_34],  _Qp_33x1[RWY2_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RWY2_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RWY2_34]-_Q_34x1[RWY2_34]),   RWY2pmax, RWY2ppmax, RWY2min, RWY2max, D2R, &X2_33x1[14], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LWY2_34],  _Qp_33x1[LWY2_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LWY2_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LWY2_34]-_Q_34x1[LWY2_34]),   LWY2pmax, LWY2ppmax, LWY2min, LWY2max, D2R, &X2_33x1[15], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	X2_33x1[16] = Xpel_3x1[1];
	X2_33x1[17] = Xpel_3x1[2];
	X2_33x1[18] = Xpel_3x1[3];
	dim_redundant_task = 18;
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
		//_jT1_33x33[28][i] = 0.f;
		//_jT1_33x33[29][i] = 0.f;
		//_jT1_33x33[30][i] = 0.f;   
		for(j=1;j<=18;j++)
			_jT2_33x33[j][i] = 0.f;
	}
	_jT1_33x33[27][3] = 1.f;
  //_jT1_33x33[28][4] = 1.f;
  //_jT1_33x33[29][5] = 1.f;
  //_jT1_33x33[30][6] = 1.f;

	for(i=1; i<=27; i++)
	{
		_jT1_33x33[i][4] = 0.f;  // exclude pelvis orientation from the solution
		_jT1_33x33[i][5] = 0.f; 
		_jT1_33x33[i][6] = 0.f; 
	}

	for(i=13; i<=24; i++)
	{
		_jT1_33x33[i][7] = 0.f;  // exclude WST from the solution
	}

	for(j=20; j<=33; j++)
	{
		_jT1_33x33[25][j] = 0.f;
		_jT1_33x33[26][j] = 0.f;
	}
	_jT1_33x33[25][7] = 0.f;
	_jT1_33x33[26][7] = 0.f;

	_jT2_33x33[1][WST_33] = 1.f;
	_jT2_33x33[2][RSP_33] = 1.f;
	_jT2_33x33[3][RSR_33] = 1.f;
	_jT2_33x33[4][RSY_33] = 1.f;
	_jT2_33x33[5][REB_33] = 1.f;
	_jT2_33x33[6][RWY_33] = 1.f;
	_jT2_33x33[7][RWP_33] = 1.f;
	_jT2_33x33[8][LSP_33] = 1.f;
	_jT2_33x33[9][LSR_33] = 1.f;	
	_jT2_33x33[10][LSY_33] = 1.f;
	_jT2_33x33[11][LEB_33] = 1.f;
	_jT2_33x33[12][LWY_33] = 1.f;
	_jT2_33x33[13][LWP_33] = 1.f;
	_jT2_33x33[14][RWY2_33] = 1.f;
	_jT2_33x33[15][LWY2_33] = 1.f;	
	_jT2_33x33[16][WX_33] = 1.f;
	_jT2_33x33[17][WY_33] = 1.f;
	_jT2_33x33[18][WZ_33] = 1.f;
	
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

	//pinv_svd((const float**)_jT2_33x33,18,33, _jT2inv_33x33);
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

	do
	{
		isLimited = 0;
		if(pSharedMemory->torque_mode_flag == 0 || pSharedMemory->torque_mode_flag == 3)
			for(i=1;i<=27;i++)
			{
				index_limited[i] = 0;
				qpp_limited[i] = 0.f;
				if(Qpp_33x1[i+6] >= ub_27x1[i])
				{
					isLimited = 1;
					index_limited[i] = 1;
					qpp_limited[i] = ub_27x1[i];
				}
				else if(Qpp_33x1[i+6] <= lb_27x1[i])
				{
					isLimited = 1;
					index_limited[i] = 1;
					qpp_limited[i] = lb_27x1[i];
				}

				if(isLimited==1)
				{
					printf("\n i: %d, joint: %d, p: %.4fdeg, v: %.4fdeg/s, a: %.4fdeg/s^2", _offline_traj.i, i+6, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D, Qpp_33x1[i+6]*R2D);
					printf("\n ub: %.4fdeg/s^2, lb: %.4fdeg/s^2", ub_27x1[i]*R2D, lb_27x1[i]*R2D);			
					return -2;
				}

				if(fabs(_Qp_33x1[i+6])>=QP_MAX)
				{
					printf("\n i: %d, joint: %d, p: %.4fdeg, v: %.4fdeg/s, a: %.4fdeg/s^2", _offline_traj.i, i+6, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D, Qpp_33x1[i+6]*R2D);
					printf("\n ub: %.4fdeg/s^2, lb: %.4fdeg/s^2", ub_27x1[i]*R2D, lb_27x1[i]*R2D);			
					return -2;
				}
			}
		
		if(isLimited==1)
		{
			//will be filled
		}
	} while(isLimited==1);
	
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
	_Q_34x1[RWY2_34] += _Qp_33x1[RWY2_33]*DT + 0.5f*Qpp_33x1[32]*DT*DT;
	_Q_34x1[LWY2_34] += _Qp_33x1[LWY2_33]*DT + 0.5f*Qpp_33x1[33]*DT*DT;
	
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
	_Qp_33x1[RWY2_33] += DT*Qpp_33x1[32];
	_Qp_33x1[LWY2_33] += DT*Qpp_33x1[33];

	Joint[WST].RefAngleCurrent = _Q_34x1[WST_34]*R2D;

	Joint[RHY].RefAngleCurrent = (_Q_34x1[RHY_34]+rhy_compen)*R2D;
	Joint[RHR].RefAngleCurrent = (_Q_34x1[RHR_34]+rhr_compen)*R2D;
	Joint[RHP].RefAngleCurrent = _Q_34x1[RHP_34]*R2D;
	Joint[RKN].RefAngleCurrent = _Q_34x1[RKN_34]*R2D;
	Joint[RAP].RefAngleCurrent = _Q_34x1[RAP_34]*R2D;
	Joint[RAR].RefAngleCurrent = _Q_34x1[RAR_34]*R2D;
	Joint[LHY].RefAngleCurrent = (_Q_34x1[LHY_34]+lhy_compen)*R2D;
	Joint[LHR].RefAngleCurrent = (_Q_34x1[LHR_34]+lhr_compen)*R2D;
	Joint[LHP].RefAngleCurrent = _Q_34x1[LHP_34]*R2D;
	Joint[LKN].RefAngleCurrent = _Q_34x1[LKN_34]*R2D;
	Joint[LAP].RefAngleCurrent = _Q_34x1[LAP_34]*R2D;
	Joint[LAR].RefAngleCurrent = _Q_34x1[LAR_34]*R2D;

	Joint[RSP].RefAngleCurrent = _Q_34x1[RSP_34]*R2D;
	Joint[RSR].RefAngleCurrent = _Q_34x1[RSR_34]*R2D - OFFSET_RSR;
	Joint[RSY].RefAngleCurrent = _Q_34x1[RSY_34]*R2D;
	Joint[REB].RefAngleCurrent = _Q_34x1[REB_34]*R2D - OFFSET_REB;
	Joint[RWY].RefAngleCurrent = _Q_34x1[RWY_34]*R2D;
	Joint[RWP].RefAngleCurrent = _Q_34x1[RWP_34]*R2D;
	Joint[RWY2].RefAngleCurrent = _Q_34x1[RWY2_34]*R2D;
	Joint[LSP].RefAngleCurrent = _Q_34x1[LSP_34]*R2D;
	Joint[LSR].RefAngleCurrent = _Q_34x1[LSR_34]*R2D - OFFSET_LSR;
	Joint[LSY].RefAngleCurrent = _Q_34x1[LSY_34]*R2D;
	Joint[LEB].RefAngleCurrent = _Q_34x1[LEB_34]*R2D - OFFSET_LEB;	
	Joint[LWY].RefAngleCurrent = _Q_34x1[LWY_34]*R2D;
	Joint[LWP].RefAngleCurrent = _Q_34x1[LWP_34]*R2D;
	Joint[LWY2].RefAngleCurrent = _Q_34x1[LWY2_34]*R2D;

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

	//----------------------------------------------------------------- 100Hz
	if(half_flag != 0)
	{
		half_flag = 0;
		return 0;
	}
	else
		half_flag = 1;
	
	diff_mm((const float**)_jRH_6x33,6,33, (const float**)_jRH_old2_6x33, _jRHp_6x33);
	subs_m((const float**)_jRH_old_6x33,6,33, _jRH_old2_6x33);
	subs_m((const float**)_jRH_6x33,6,33, _jRH_old_6x33);
	mult_sm((const float**)_jRHp_6x33,6,33, 1.f/(2.f*2.f*DT), _jRHp_6x33);
	
	diff_mm((const float**)_jLH_6x33,6,33, (const float**)_jLH_old2_6x33, _jLHp_6x33);
	subs_m((const float**)_jLH_old_6x33,6,33, _jLH_old2_6x33);
	subs_m((const float**)_jLH_6x33,6,33, _jLH_old_6x33);
	mult_sm((const float**)_jLHp_6x33,6,33, 1.f/(2.f*2.f*DT), _jLHp_6x33);


	for(i=1; i<=34; i++)
		meas_Q_34x1[i] = _Q_34x1[i];

	meas_Q_34x1[RSP_34] = ((float)Joint[RSP].EncoderValue)/Joint[RSP].PPR*D2R;
	meas_Q_34x1[RSR_34] = ((float)Joint[RSR].EncoderValue)/Joint[RSR].PPR*D2R + OFFSET_RSR*D2R;
	meas_Q_34x1[RSY_34] = ((float)Joint[RSY].EncoderValue)/Joint[RSY].PPR*D2R;
	meas_Q_34x1[REB_34] = ((float)Joint[REB].EncoderValue)/Joint[REB].PPR*D2R + OFFSET_REB*D2R;
	meas_Q_34x1[RWY_34] = ((float)Joint[RWY].EncoderValue)/Joint[RWY].PPR*D2R;
	meas_Q_34x1[RWP_34] = ((float)Joint[RWP].EncoderValue)/Joint[RWP].PPR*D2R;
	meas_Q_34x1[RWY2_34] = ((float)Joint[RWY2].EncoderValue)/Joint[RWY2].PPR*D2R;

	meas_Q_34x1[LSP_34] = ((float)Joint[LSP].EncoderValue)/Joint[LSP].PPR*D2R;
	meas_Q_34x1[LSR_34] = ((float)Joint[LSR].EncoderValue)/Joint[LSR].PPR*D2R + OFFSET_LSR*D2R;
	meas_Q_34x1[LSY_34] = ((float)Joint[LSY].EncoderValue)/Joint[LSY].PPR*D2R;
	meas_Q_34x1[LEB_34] = ((float)Joint[LEB].EncoderValue)/Joint[LEB].PPR*D2R + OFFSET_LEB*D2R;
	meas_Q_34x1[LWY_34] = ((float)Joint[LWY].EncoderValue)/Joint[LWY].PPR*D2R;
	meas_Q_34x1[LWP_34] = ((float)Joint[LWP].EncoderValue)/Joint[LWP].PPR*D2R;
	meas_Q_34x1[LWY2_34] = ((float)Joint[LWY2].EncoderValue)/Joint[LWY2].PPR*D2R;

	meas_qSteer = ((float)Joint[NKY].EncoderValue)/(-2883.f)*360.f*D2R;

	meas_Qp_33x1[RSP_33] = (meas_Q_34x1[RSP_34]-Q_old2_34x1[RSP_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RSR_33] = (meas_Q_34x1[RSR_34]-Q_old2_34x1[RSR_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RSY_33] = (meas_Q_34x1[RSY_34]-Q_old2_34x1[RSY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[REB_33] = (meas_Q_34x1[REB_34]-Q_old2_34x1[REB_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RWY_33] = (meas_Q_34x1[RWY_34]-Q_old2_34x1[RWY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RWP_33] = (meas_Q_34x1[RWP_34]-Q_old2_34x1[RWP_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RWY2_33] = (meas_Q_34x1[RWY2_34]-Q_old2_34x1[RWY2_34])/(2.f*2.f*DT);

	meas_Qp_33x1[LSP_33] = (meas_Q_34x1[LSP_34]-Q_old2_34x1[LSP_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LSR_33] = (meas_Q_34x1[LSR_34]-Q_old2_34x1[LSR_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LSY_33] = (meas_Q_34x1[LSY_34]-Q_old2_34x1[LSY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LEB_33] = (meas_Q_34x1[LEB_34]-Q_old2_34x1[LEB_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LWY_33] = (meas_Q_34x1[LWY_34]-Q_old2_34x1[LWY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LWP_33] = (meas_Q_34x1[LWP_34]-Q_old2_34x1[LWP_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LWY2_33] = (meas_Q_34x1[LWY2_34]-Q_old2_34x1[LWY2_34])/(2.f*2.f*DT);

	meas_wSteer = (meas_qSteer-qSteer_old2)/(2.f*2.f*DT);

	pSharedMemory->disp_Q_34x1[0] = meas_qSteer;

	/*
	_log_temp[0] = meas_Qp_33x1[RSP_33];
	_log_temp[1] = meas_Qp_33x1[RSR_33];
	_log_temp[2] = meas_Qp_33x1[RSY_33];
	_log_temp[3] = meas_Qp_33x1[REB_33];
	_log_temp[4] = meas_Qp_33x1[RWY_33];
	_log_temp[5] = meas_Qp_33x1[RWP_33];
	_log_temp[6] = meas_Qp_33x1[RWY2_33];

	_log_temp[7] = meas_Qp_33x1[LSP_33];
	_log_temp[8] = meas_Qp_33x1[LSR_33];
	_log_temp[9] = meas_Qp_33x1[LSY_33];
	_log_temp[10] = meas_Qp_33x1[LEB_33];
	_log_temp[11] = meas_Qp_33x1[LWY_33];
	_log_temp[12] = meas_Qp_33x1[LWP_33];
	_log_temp[13] = meas_Qp_33x1[LWY2_33];
	*/

	//------------------------ joint limits
	for(i=20;i<=33;i++)
		if(fabs(meas_Qp_33x1[i])> 15.f)
		{
			RBpwmCommandHR2ch(CAN1, Joint[LSP].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[LSY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[LWY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[LWY2].JMC, 0, 0, 0);

			RBpwmCommandHR2ch(CAN1, Joint[RSP].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[RSY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[RWY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[RWY2].JMC, 0, 0, 0);


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
	Q_old2_34x1[RWY2_34] = Q_old_34x1[RWY2_34];

	Q_old2_34x1[LSP_34] = Q_old_34x1[LSP_34];
	Q_old2_34x1[LSR_34] = Q_old_34x1[LSR_34];
	Q_old2_34x1[LSY_34] = Q_old_34x1[LSY_34];
	Q_old2_34x1[LEB_34] = Q_old_34x1[LEB_34];
	Q_old2_34x1[LWY_34] = Q_old_34x1[LWY_34];
	Q_old2_34x1[LWP_34] = Q_old_34x1[LWP_34];
	Q_old2_34x1[LWY2_34] = Q_old_34x1[LWY2_34];

	qSteer_old2 = qSteer_old;

	Q_old_34x1[RSP_34] = meas_Q_34x1[RSP_34];
	Q_old_34x1[RSR_34] = meas_Q_34x1[RSR_34];
	Q_old_34x1[RSY_34] = meas_Q_34x1[RSY_34];
	Q_old_34x1[REB_34] = meas_Q_34x1[REB_34];
	Q_old_34x1[RWY_34] = meas_Q_34x1[RWY_34];
	Q_old_34x1[RWP_34] = meas_Q_34x1[RWP_34];
	Q_old_34x1[RWY2_34] = meas_Q_34x1[RWY2_34];

	Q_old_34x1[LSP_34] = meas_Q_34x1[LSP_34];
	Q_old_34x1[LSR_34] = meas_Q_34x1[LSR_34];
	Q_old_34x1[LSY_34] = meas_Q_34x1[LSY_34];
	Q_old_34x1[LEB_34] = meas_Q_34x1[LEB_34];
	Q_old_34x1[LWY_34] = meas_Q_34x1[LWY_34];
	Q_old_34x1[LWP_34] = meas_Q_34x1[LWP_34];
	Q_old_34x1[LWY2_34] = meas_Q_34x1[LWY2_34];

	qSteer_old = meas_qSteer;

	if(pSharedMemory->torque_mode_flag == 0)
	{
		for(i=1;i<=3;i++)
		{
			pRH2_3x1[i] = _pRH_3x1[i];
			pLH2_3x1[i] = _pLH_3x1[i];
			qRH2_4x1[i] = _qRH_4x1[i];
			qLH2_4x1[i] = _qLH_4x1[i];
		}
		qRH2_4x1[4] = _qRH_4x1[4];
		qLH2_4x1[4] = _qLH_4x1[4];

		pre_gain = 0.f;
		count = 0;
	}
	else if(pSharedMemory->torque_mode_flag == 1)
	{
		for(i=1;i<=3;i++)
		{
			pRH1_3x1[i] = _pRH_3x1[i];
			pLH1_3x1[i] = _pLH_3x1[i];
			qRH1_4x1[i] = _qRH_4x1[i];
			qLH1_4x1[i] = _qLH_4x1[i];
		}
		qRH1_4x1[4] = _qRH_4x1[4];
		qLH1_4x1[4] = _qLH_4x1[4];
	
		pre_gain = (float)count/20.f;
		count++;
		if(count>20)
		{
			pSharedMemory->torque_mode_flag = 2;
			printf("\n torque mode");
		}
	}
	else if(pSharedMemory->torque_mode_flag == 2)
	{
		for(i=1;i<=3;i++)
		{
			pRH1_3x1[i] = _pRH_3x1[i];
			pLH1_3x1[i] = _pLH_3x1[i];
			qRH1_4x1[i] = _qRH_4x1[i];
			qLH1_4x1[i] = _qLH_4x1[i];
		}
		qRH1_4x1[4] = _qRH_4x1[4];
		qLH1_4x1[4] = _qLH_4x1[4];

		pre_gain = 1.f;
		count = 0;
	}
	else if(pSharedMemory->torque_mode_flag == 3)
	{	
		_Q_34x1[RSP_34] = meas_Q_34x1[RSP_34];
		_Q_34x1[RSR_34] = meas_Q_34x1[RSR_34];
		_Q_34x1[RSY_34] = meas_Q_34x1[RSY_34];
		_Q_34x1[REB_34] = meas_Q_34x1[REB_34];
		_Q_34x1[RWY_34] = meas_Q_34x1[RWY_34];
		_Q_34x1[RWP_34] = meas_Q_34x1[RWP_34];
		_Q_34x1[RWY2_34] = meas_Q_34x1[RWY2_34];

		_Q_34x1[LSP_34] = meas_Q_34x1[LSP_34];
		_Q_34x1[LSR_34] = meas_Q_34x1[LSR_34];
		_Q_34x1[LSY_34] = meas_Q_34x1[LSY_34];
		_Q_34x1[LEB_34] = meas_Q_34x1[LEB_34];
		_Q_34x1[LWY_34] = meas_Q_34x1[LWY_34];
		_Q_34x1[LWP_34] = meas_Q_34x1[LWP_34];
		_Q_34x1[LWY2_34] = meas_Q_34x1[LWY2_34];

		_Qp_33x1[RSP_33] = meas_Qp_33x1[RSP_33];
		_Qp_33x1[RSR_33] = meas_Qp_33x1[RSR_33];
		_Qp_33x1[RSY_33] = meas_Qp_33x1[RSY_33];
		_Qp_33x1[REB_33] = meas_Qp_33x1[REB_33];
		_Qp_33x1[RWY_33] = meas_Qp_33x1[RWY_33];
		_Qp_33x1[RWP_33] = meas_Qp_33x1[RWP_33];
		_Qp_33x1[RWY2_33] = meas_Qp_33x1[RWY2_33];

		_Qp_33x1[LSP_33] = meas_Qp_33x1[LSP_33];
		_Qp_33x1[LSR_33] = meas_Qp_33x1[LSR_33];
		_Qp_33x1[LSY_33] = meas_Qp_33x1[LSY_33];
		_Qp_33x1[LEB_33] = meas_Qp_33x1[LEB_33];
		_Qp_33x1[LWY_33] = meas_Qp_33x1[LWY_33];
		_Qp_33x1[LWP_33] = meas_Qp_33x1[LWP_33];
		_Qp_33x1[LWY2_33] = meas_Qp_33x1[LWY2_33];

		Joint[RSP].RefAngleCurrent = _Q_34x1[RSP_34]*R2D;
		Joint[RSR].RefAngleCurrent = _Q_34x1[RSR_34]*R2D - OFFSET_RSR;
		Joint[RSY].RefAngleCurrent = _Q_34x1[RSY_34]*R2D;
		Joint[REB].RefAngleCurrent = _Q_34x1[REB_34]*R2D - OFFSET_REB;
		Joint[RWY].RefAngleCurrent = _Q_34x1[RWY_34]*R2D;
		Joint[RWP].RefAngleCurrent = _Q_34x1[RWP_34]*R2D;
		Joint[RWY2].RefAngleCurrent = _Q_34x1[RWY2_34]*R2D;
		Joint[LSP].RefAngleCurrent = _Q_34x1[LSP_34]*R2D;
		Joint[LSR].RefAngleCurrent = _Q_34x1[LSR_34]*R2D - OFFSET_LSR;
		Joint[LSY].RefAngleCurrent = _Q_34x1[LSY_34]*R2D;
		Joint[LEB].RefAngleCurrent = _Q_34x1[LEB_34]*R2D - OFFSET_LEB;	
		Joint[LWY].RefAngleCurrent = _Q_34x1[LWY_34]*R2D;
		Joint[LWP].RefAngleCurrent = _Q_34x1[LWP_34]*R2D;
		Joint[LWY2].RefAngleCurrent = _Q_34x1[LWY2_34]*R2D;
		
		SendRunStopCMD(Joint[LSP].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[LSP].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[LSY].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[LSY].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[LWY].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[LWY].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[LWY2].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[LWY2].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[RSP].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[RSP].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[RSY].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[RSY].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[RWY].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[RWY].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[RWY2].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[RWY2].JMC, 0, 0, 0);
	
		pre_gain = 0.f;
		pSharedMemory->torque_mode_flag = 0;
		printf("\n position mode");
	}

	_Q_34x1[RSP_34] =  pre_gain*meas_Q_34x1[RSP_34] + (1.f-pre_gain)*_Q_34x1[RSP_34];
	_Q_34x1[RSR_34] =  pre_gain*meas_Q_34x1[RSR_34] + (1.f-pre_gain)*_Q_34x1[RSR_34];
	_Q_34x1[RSY_34] =  pre_gain*meas_Q_34x1[RSY_34] + (1.f-pre_gain)*_Q_34x1[RSY_34];
	_Q_34x1[REB_34] =  pre_gain*meas_Q_34x1[REB_34] + (1.f-pre_gain)*_Q_34x1[REB_34];
	_Q_34x1[RWY_34] =  pre_gain*meas_Q_34x1[RWY_34] + (1.f-pre_gain)*_Q_34x1[RWY_34];
	_Q_34x1[RWP_34] =  pre_gain*meas_Q_34x1[RWP_34] + (1.f-pre_gain)*_Q_34x1[RWP_34];
	_Q_34x1[RWY2_34] =  pre_gain*meas_Q_34x1[RWY2_34] + (1.f-pre_gain)*_Q_34x1[RWY2_34];
	
	_Q_34x1[LSP_34] =  pre_gain*meas_Q_34x1[LSP_34] + (1.f-pre_gain)*_Q_34x1[LSP_34];
	_Q_34x1[LSR_34] =  pre_gain*meas_Q_34x1[LSR_34] + (1.f-pre_gain)*_Q_34x1[LSR_34];
	_Q_34x1[LSY_34] =  pre_gain*meas_Q_34x1[LSY_34] + (1.f-pre_gain)*_Q_34x1[LSY_34];
	_Q_34x1[LEB_34] =  pre_gain*meas_Q_34x1[LEB_34] + (1.f-pre_gain)*_Q_34x1[LEB_34];
	_Q_34x1[LWY_34] =  pre_gain*meas_Q_34x1[LWY_34] + (1.f-pre_gain)*_Q_34x1[LWY_34];
	_Q_34x1[LWP_34] =  pre_gain*meas_Q_34x1[LWP_34] + (1.f-pre_gain)*_Q_34x1[LWP_34];
	_Q_34x1[LWY2_34] =  pre_gain*meas_Q_34x1[LWY2_34] + (1.f-pre_gain)*_Q_34x1[LWY2_34];

	_Qp_33x1[RSP_33] =  pre_gain*meas_Qp_33x1[RSP_33] + (1.f-pre_gain)*_Qp_33x1[RSP_33];
	_Qp_33x1[RSR_33] =  pre_gain*meas_Qp_33x1[RSR_33] + (1.f-pre_gain)*_Qp_33x1[RSR_33];
	_Qp_33x1[RSY_33] =  pre_gain*meas_Qp_33x1[RSY_33] + (1.f-pre_gain)*_Qp_33x1[RSY_33];
	_Qp_33x1[REB_33] =  pre_gain*meas_Qp_33x1[REB_33] + (1.f-pre_gain)*_Qp_33x1[REB_33];
	_Qp_33x1[RWY_33] =  pre_gain*meas_Qp_33x1[RWY_33] + (1.f-pre_gain)*_Qp_33x1[RWY_33];
	_Qp_33x1[RWP_33] =  pre_gain*meas_Qp_33x1[RWP_33] + (1.f-pre_gain)*_Qp_33x1[RWP_33];
	_Qp_33x1[RWY2_33] =  pre_gain*meas_Qp_33x1[RWY2_33] + (1.f-pre_gain)*_Qp_33x1[RWY2_33];
	
	_Qp_33x1[LSP_33] =  pre_gain*meas_Qp_33x1[LSP_33] + (1.f-pre_gain)*_Qp_33x1[LSP_33];
	_Qp_33x1[LSR_33] =  pre_gain*meas_Qp_33x1[LSR_33] + (1.f-pre_gain)*_Qp_33x1[LSR_33];
	_Qp_33x1[LSY_33] =  pre_gain*meas_Qp_33x1[LSY_33] + (1.f-pre_gain)*_Qp_33x1[LSY_33];
	_Qp_33x1[LEB_33] =  pre_gain*meas_Qp_33x1[LEB_33] + (1.f-pre_gain)*_Qp_33x1[LEB_33];
	_Qp_33x1[LWY_33] =  pre_gain*meas_Qp_33x1[LWY_33] + (1.f-pre_gain)*_Qp_33x1[LWY_33];
	_Qp_33x1[LWP_33] =  pre_gain*meas_Qp_33x1[LWP_33] + (1.f-pre_gain)*_Qp_33x1[LWP_33];
	_Qp_33x1[LWY2_33] =  pre_gain*meas_Qp_33x1[LWY2_33] + (1.f-pre_gain)*_Qp_33x1[LWY2_33];

	if(pSharedMemory->torque_mode_flag == 1 || pSharedMemory->torque_mode_flag == 2)
	{
		for(i=1;i<=33;i++)
			duty_joint_limit_33x1[i] = 0.f;
		getDuty4JointLimit(_Q_34x1, duty_joint_limit_33x1);
		getFricCompen(_Qp_33x1, fric_compen_33x1);
		getGravityTorque(_Q_34x1, gravity_33x1);
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

		for(i=1;i<=6;i++)
			for(j=1;j<=33;j++)
			{
				_jT1_33x33[i][j] = _jRH_6x33[i][j];			
				_jT1_33x33[i+6][j] = _jLH_6x33[i][j];			
			}

		for(i=1; i<=12; i++)
		{
			_jT1_33x33[i][1] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][2] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][3] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][4] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][5] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][6] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][7] = 0.f;  // exclude WST from the solution
		}


		if(pSharedMemory->steer_demo == 1)
		{
			qSteer_errSum = 0.f;
			t_steer = 0.f;
			steer_count = 0;

			neutral_Q_34x1[RSP_34] = _Q_34x1[RSP_34];
			neutral_Q_34x1[RSR_34] = _Q_34x1[RSR_34];
			neutral_Q_34x1[RSY_34] = _Q_34x1[RSY_34];
			neutral_Q_34x1[REB_34] = _Q_34x1[REB_34];
			neutral_Q_34x1[RWY_34] = _Q_34x1[RWY_34];
			neutral_Q_34x1[RWP_34] = _Q_34x1[RWP_34];
			neutral_Q_34x1[RWY2_34] = _Q_34x1[RWY2_34];
			
			neutral_Q_34x1[LSP_34] = _Q_34x1[LSP_34];
			neutral_Q_34x1[LSR_34] = _Q_34x1[LSR_34];
			neutral_Q_34x1[LSY_34] = _Q_34x1[LSY_34];
			neutral_Q_34x1[LEB_34] = _Q_34x1[LEB_34];
			neutral_Q_34x1[LWY_34] = _Q_34x1[LWY_34];
			neutral_Q_34x1[LWP_34] = _Q_34x1[LWP_34];
			neutral_Q_34x1[LWY2_34] = _Q_34x1[LWY2_34];

			pRH2_3x1[1] = _pRH_3x1[1];
			pRH2_3x1[2] = _pRH_3x1[2];
			pRH2_3x1[3] = _pRH_3x1[3];
			qRH2_4x1[1] = _qRH_4x1[1];
			qRH2_4x1[2] = _qRH_4x1[2];
			qRH2_4x1[3] = _qRH_4x1[3];
			qRH2_4x1[4] = _qRH_4x1[4];
							
			pLH2_3x1[1] = _pLH_3x1[1];
			pLH2_3x1[2] = _pLH_3x1[2];
			pLH2_3x1[3] = _pLH_3x1[3];
			qLH2_4x1[1] = _qLH_4x1[1];
			qLH2_4x1[2] = _qLH_4x1[2];
			qLH2_4x1[3] = _qLH_4x1[3];
			qLH2_4x1[4] = _qLH_4x1[4];

			_TEMP3_34x34[1][1] = Xrh_6x1[1] = 0.f;
			_TEMP3_34x34[1][2] = Xrh_6x1[2] = 0.f;
			_TEMP3_34x34[1][3] = Xrh_6x1[3] = 0.f;
			_TEMP3_34x34[1][4] = Xrh_6x1[4] = 0.f;
			_TEMP3_34x34[1][5] = Xrh_6x1[5] = 0.f;
			_TEMP3_34x34[1][6] = Xrh_6x1[6] = 0.f;
			_TEMP3_34x34[1][7] = Xlh_6x1[1] = 0.f;
			_TEMP3_34x34[1][8] = Xlh_6x1[2] = 0.f;
			_TEMP3_34x34[1][9] = Xlh_6x1[3] = 0.f;
			_TEMP3_34x34[1][10] = Xlh_6x1[4] = 0.f;
			_TEMP3_34x34[1][11] = Xlh_6x1[5] = 0.f;
			_TEMP3_34x34[1][12] = Xlh_6x1[6] = 0.f;
		}
		else if(pSharedMemory->steer_demo == 2 || pSharedMemory->steer_demo == 3)
		{

			if(pSharedMemory->steer_demo == 2)
			{
				qSteer1 = qSteer0 = des_qSteer = meas_qSteer;
				qSteer_errSum = 0.f;
				steer_count = 0;
				t_steer = 0.f;
				pSharedMemory->steer_demo = 3;
			}

			pRH2_3x1[1] = _pRH_3x1[1];
			pRH2_3x1[2] = _pRH_3x1[2];
			pRH2_3x1[3] = _pRH_3x1[3];
			qRH2_4x1[1] = _qRH_4x1[1];
			qRH2_4x1[2] = _qRH_4x1[2];
			qRH2_4x1[3] = _qRH_4x1[3];
			qRH2_4x1[4] = _qRH_4x1[4];
							
			pLH2_3x1[1] = _pLH_3x1[1];
			pLH2_3x1[2] = _pLH_3x1[2];
			pLH2_3x1[3] = _pLH_3x1[3];
			qLH2_4x1[1] = _qLH_4x1[1];
			qLH2_4x1[2] = _qLH_4x1[2];
			qLH2_4x1[3] = _qLH_4x1[3];
			qLH2_4x1[4] = _qLH_4x1[4];

			if(t_steer <= pSharedMemory->steer_time[steer_count])
				des_qSteer =  qSteer1 + one_cos(t_steer, qSteer0+pSharedMemory->steer_ang[steer_count]*D2R-qSteer1, pSharedMemory->steer_time[steer_count]);

			t_steer += 0.01f;
			if(t_steer > pSharedMemory->steer_time[steer_count])
			{
				steer_count++;
				if(steer_count < N_STEER)
				{
					qSteer1 = des_qSteer;
					t_steer = 0.f;
				}
				else
				{
					steer_count--;
					t_steer -= 0.01f;					
				}
			}

			qSteer_errSum += (des_qSteer-meas_qSteer)*0.01f;
			ftemp1_7x1[1] = (pSharedMemory->kp[16]*qSteer_errSum - pSharedMemory->kd[9]*meas_wSteer + pSharedMemory->kp[9]*(des_qSteer-meas_qSteer));
			if(ftemp1_7x1[1]>100.f)
				ftemp1_7x1[1] = 100.f;
			else if(ftemp1_7x1[1]<-100.f)
				ftemp1_7x1[1] = -100.f;
			ftemp1_7x1[2] = 0.f;
			ftemp1_7x1[3] = 0.f;

			pSharedMemory->disp_Q_34x1[0] = ftemp1_7x1[1]*D2R;

			QT2DC(_qRH_4x1, _TEMP1_34x34);
			mult_mv((const float**)_TEMP1_34x34,3,3, ftemp1_7x1, Xrh_6x1);
			Xrh_6x1[4] = 0.f;
			Xrh_6x1[5] = 0.f;
			Xrh_6x1[6] = 0.f;

			QT2DC(_qLH_4x1, _TEMP1_34x34);
			mult_smv(-1.f, (const float**)_TEMP1_34x34,3,3, ftemp1_7x1, Xlh_6x1);
			Xlh_6x1[4] = 0.f;
			Xlh_6x1[5] = 0.f;
			Xlh_6x1[6] = 0.f;

			_TEMP3_34x34[1][1] = Xrh_6x1[1];
			_TEMP3_34x34[1][2] = Xrh_6x1[2];
			_TEMP3_34x34[1][3] = Xrh_6x1[3];
			_TEMP3_34x34[1][4] = Xrh_6x1[4];
			_TEMP3_34x34[1][5] = Xrh_6x1[5];
			_TEMP3_34x34[1][6] = Xrh_6x1[6];
			_TEMP3_34x34[1][7] = Xlh_6x1[1];
			_TEMP3_34x34[1][8] = Xlh_6x1[2];
			_TEMP3_34x34[1][9] = Xlh_6x1[3];
			_TEMP3_34x34[1][10] = Xlh_6x1[4];
			_TEMP3_34x34[1][11] = Xlh_6x1[5];
			_TEMP3_34x34[1][12] = Xlh_6x1[6];
		}
		else
		{
			qSteer_errSum = 0.f;

			des_pRH_3x1[1] = pRH2_3x1[1];
			des_pRH_3x1[2] = pRH2_3x1[2];
			des_pRH_3x1[3] = pRH2_3x1[3];
			des_qRH_4x1[1] = qRH2_4x1[1];
			des_qRH_4x1[2] = qRH2_4x1[2];
			des_qRH_4x1[3] = qRH2_4x1[3];
			des_qRH_4x1[4] = qRH2_4x1[4];
			des_vRH_3x1[1] = 0.f;
			des_vRH_3x1[2] = 0.f;
			des_vRH_3x1[3] = 0.f;
			des_wRH_3x1[1] = 0.f;
			des_wRH_3x1[2] = 0.f;
			des_wRH_3x1[3] = 0.f;
				
			des_pLH_3x1[1] = pLH2_3x1[1];
			des_pLH_3x1[2] = pLH2_3x1[2];
			des_pLH_3x1[3] = pLH2_3x1[3];
			des_qLH_4x1[1] = qLH2_4x1[1];
			des_qLH_4x1[2] = qLH2_4x1[2];
			des_qLH_4x1[3] = qLH2_4x1[3];
			des_qLH_4x1[4] = qLH2_4x1[4];
			des_vLH_3x1[1] = 0.f;
			des_vLH_3x1[2] = 0.f;
			des_vLH_3x1[3] = 0.f;
			des_wLH_3x1[1] = 0.f;
			des_wLH_3x1[2] = 0.f;
			des_wLH_3x1[3] = 0.f;

			diff_vv(des_pRH_3x1,3, _pRH_3x1, ftemp3_7x1);
			mult_mv((const float**)_jRH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
			diff_vv(des_vRH_3x1,3, ftemp2_7x1, ftemp1_7x1);
			Xrh_6x1[1] = pSharedMemory->kp[0]*ftemp3_7x1[1] + pSharedMemory->kd[0]*ftemp1_7x1[1];
			Xrh_6x1[2] = pSharedMemory->kp[1]*ftemp3_7x1[2] + pSharedMemory->kd[1]*ftemp1_7x1[2];
			Xrh_6x1[3] = pSharedMemory->kp[2]*ftemp3_7x1[3] + pSharedMemory->kd[2]*ftemp1_7x1[3];
			QTdel(des_qRH_4x1, _qRH_4x1, ftemp3_7x1);
			diff_vv(des_wRH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
			sum_svsv(pSharedMemory->kp[3],ftemp3_7x1,3, pSharedMemory->kd[3],ftemp4_7x1, &Xrh_6x1[3]);

			diff_vv(des_pLH_3x1,3, _pLH_3x1, ftemp3_7x1);
			mult_mv((const float**)_jLH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
			diff_vv(des_vLH_3x1,3, ftemp2_7x1, ftemp1_7x1);
			Xlh_6x1[1] = pSharedMemory->kp[4]*ftemp3_7x1[1] + pSharedMemory->kd[4]*ftemp1_7x1[1];
			Xlh_6x1[2] = pSharedMemory->kp[5]*ftemp3_7x1[2] + pSharedMemory->kd[5]*ftemp1_7x1[2];
			Xlh_6x1[3] = pSharedMemory->kp[6]*ftemp3_7x1[3] + pSharedMemory->kd[6]*ftemp1_7x1[3];
			QTdel(des_qLH_4x1, _qLH_4x1, ftemp3_7x1);
			diff_vv(des_wLH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
			sum_svsv(pSharedMemory->kp[7],ftemp3_7x1,3, pSharedMemory->kd[7],ftemp4_7x1, &Xlh_6x1[3]);

			_TEMP3_34x34[1][1] = Xrh_6x1[1];
			_TEMP3_34x34[1][2] = Xrh_6x1[2];
			_TEMP3_34x34[1][3] = Xrh_6x1[3];
			_TEMP3_34x34[1][4] = Xrh_6x1[4];
			_TEMP3_34x34[1][5] = Xrh_6x1[5];
			_TEMP3_34x34[1][6] = Xrh_6x1[6];
			_TEMP3_34x34[1][7] = Xlh_6x1[1];
			_TEMP3_34x34[1][8] = Xlh_6x1[2];
			_TEMP3_34x34[1][9] = Xlh_6x1[3];
			_TEMP3_34x34[1][10] = Xlh_6x1[4];
			_TEMP3_34x34[1][11] = Xlh_6x1[5];
			_TEMP3_34x34[1][12] = Xlh_6x1[6];
		}

		trans(1.f, (const float**)_jT1_33x33,12,33, _TEMP2_34x34);
		mult_mv((const float**)_TEMP2_34x34,33,12, _TEMP3_34x34[1], ct_33x1);

		if(pinv_SR((const float**)_jT1_33x33, 12, 33, (float)1.e-10, _jT1inv_33x33) != 0)
		{
			printf("\n pinv error!");
			return -2; // singularity occurred
		}
		mult_mm((const float**)_jT1inv_33x33,33,12, (const float**)_jT1_33x33, 33, _TEMP1_34x34);
		diff_mm((const float**)_EYE_33, 33,33, (const float**)_TEMP1_34x34, _N1_33x33);

		if(pSharedMemory->steer_demo == 3)
		{
			for(i=1;i<=33;i++)
				_TEMP3_34x34[1][i] = 0.f;
		//	_TEMP3_34x34[1][20] = pSharedMemory->kd[8]*(-_Qp_33x1[RSP_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RSP_34]-_Q_34x1[RSP_34]);
		//	_TEMP3_34x34[1][21] = pSharedMemory->kd[8]*(-_Qp_33x1[RSR_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RSR_34]-_Q_34x1[RSR_34]);
		//	_TEMP3_34x34[1][22] = pSharedMemory->kd[8]*(-_Qp_33x1[RSY_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RSY_34]-_Q_34x1[RSY_34]);
		//	_TEMP3_34x34[1][23] = pSharedMemory->kd[8]*(-_Qp_33x1[REB_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[REB_34]-_Q_34x1[REB_34]);
			_TEMP3_34x34[1][23] = pSharedMemory->kd[8]*(-_Qp_33x1[RWY_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RWY_34]-_Q_34x1[RWY_34]);
		//	_TEMP3_34x34[1][25] = pSharedMemory->kd[8]*(-_Qp_33x1[RWP_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RWP_34]-_Q_34x1[RWP_34]);
		//	_TEMP3_34x34[1][26] = pSharedMemory->kd[8]*(-_Qp_33x1[LSP_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LSP_34]-_Q_34x1[LSP_34]);
		//	_TEMP3_34x34[1][27] = pSharedMemory->kd[8]*(-_Qp_33x1[LSR_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LSR_34]-_Q_34x1[LSR_34]);
		//	_TEMP3_34x34[1][28] = pSharedMemory->kd[8]*(-_Qp_33x1[LSY_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LSY_34]-_Q_34x1[LSY_34]);
		//	_TEMP3_34x34[1][29] = pSharedMemory->kd[8]*(-_Qp_33x1[LEB_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LEB_34]-_Q_34x1[LEB_34]);
			_TEMP3_34x34[1][30] = pSharedMemory->kd[8]*(-_Qp_33x1[LWY_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LWY_34]-_Q_34x1[LWY_34]);
		//	_TEMP3_34x34[1][31] = pSharedMemory->kd[8]*(-_Qp_33x1[LWP_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LWP_34]-_Q_34x1[LWP_34]);
		//	_TEMP3_34x34[1][32] = pSharedMemory->kd[8]*(-_Qp_33x1[RWY2_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RWY2_34]-_Q_34x1[RWY2_34]);
		//	_TEMP3_34x34[1][33] = pSharedMemory->kd[8]*(-_Qp_33x1[LWY2_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LWY2_34]-_Q_34x1[LWY2_34]);
			mult_mv((const float**)_N1_33x33,33,33, _TEMP3_34x34[1], _TEMP1_34x34[1]);
			sum_vv(ct_33x1,33, (const float*)_TEMP1_34x34[1], ct_33x1);
		}

		ct_33x1[RSP_33] = _gain_task[RSP_33]*ct_33x1[RSP_33]+_gain_gravity[RSP_33]*gravity_33x1[RSP_33];
		ct_33x1[RSR_33] = _gain_task[RSR_33]*ct_33x1[RSR_33]+_gain_gravity[RSR_33]*gravity_33x1[RSR_33];
		ct_33x1[RSY_33] = _gain_task[RSY_33]*ct_33x1[RSY_33]+_gain_gravity[RSY_33]*gravity_33x1[RSY_33];
		ct_33x1[REB_33] = _gain_task[REB_33]*ct_33x1[REB_33]+_gain_gravity[REB_33]*gravity_33x1[REB_33];
		ct_33x1[RWY_33] = _gain_task[RWY_33]*ct_33x1[RWY_33]+_gain_gravity[RWY_33]*gravity_33x1[RWY_33];
		ct_33x1[RWP_33] = _gain_task[RWP_33]*ct_33x1[RWP_33]+_gain_gravity[RWP_33]*gravity_33x1[RWP_33];
		ct_33x1[RWY2_33] = _gain_task[RWY2_33]*ct_33x1[RWY2_33]+_gain_gravity[RWY2_33]*gravity_33x1[RWY2_33];

		ct_33x1[LSP_33] = _gain_task[LSP_33]*ct_33x1[LSP_33]+_gain_gravity[LSP_33]*gravity_33x1[LSP_33];
		ct_33x1[LSR_33] = _gain_task[LSR_33]*ct_33x1[LSR_33]+_gain_gravity[LSR_33]*gravity_33x1[LSR_33];
		ct_33x1[LSY_33] = _gain_task[LSY_33]*ct_33x1[LSY_33]+_gain_gravity[LSY_33]*gravity_33x1[LSY_33];
		ct_33x1[LEB_33] = _gain_task[LEB_33]*ct_33x1[LEB_33]+_gain_gravity[LEB_33]*gravity_33x1[LEB_33];
		ct_33x1[LWY_33] = _gain_task[LWY_33]*ct_33x1[LWY_33]+_gain_gravity[LWY_33]*gravity_33x1[LWY_33];
		ct_33x1[LWP_33] = _gain_task[LWP_33]*ct_33x1[LWP_33]+_gain_gravity[LWP_33]*gravity_33x1[LWP_33];
		ct_33x1[LWY2_33] = _gain_task[LWY2_33]*ct_33x1[LWY2_33]+_gain_gravity[LWY2_33]*gravity_33x1[LWY2_33];

		pSharedMemory->disp_ct_33x1[RSP_33] = ct_33x1[RSP_33];
		pSharedMemory->disp_ct_33x1[RSR_33] = ct_33x1[RSR_33];
		pSharedMemory->disp_ct_33x1[RSY_33] = ct_33x1[RSY_33];
		pSharedMemory->disp_ct_33x1[REB_33] = ct_33x1[REB_33];
		pSharedMemory->disp_ct_33x1[RWY_33] = ct_33x1[RWY_33];
		pSharedMemory->disp_ct_33x1[RWP_33] = ct_33x1[RWP_33];
		pSharedMemory->disp_ct_33x1[RWY2_33] = ct_33x1[RWY2_33];
		
		pSharedMemory->disp_ct_33x1[LSP_33] = ct_33x1[LSP_33];
		pSharedMemory->disp_ct_33x1[LSR_33] = ct_33x1[LSR_33];
		pSharedMemory->disp_ct_33x1[LSY_33] = ct_33x1[LSY_33];
		pSharedMemory->disp_ct_33x1[LEB_33] = ct_33x1[LEB_33];
		pSharedMemory->disp_ct_33x1[LWY_33] = ct_33x1[LWY_33];
		pSharedMemory->disp_ct_33x1[LWP_33] = ct_33x1[LWP_33];
		pSharedMemory->disp_ct_33x1[LWY2_33] = ct_33x1[LWY2_33];
		
		pSharedMemory->disp_duty_33x1[RSP_33] = duty_33x1[RSP_33] = limitDuty(0.4f, torque2duty(RSP, pre_gain*ct_33x1[RSP_33]) + pre_gain*fric_compen_33x1[RSP_33] + duty_joint_limit_33x1[RSP_33]);
		pSharedMemory->disp_duty_33x1[RSR_33] = duty_33x1[RSR_33] = limitDuty(0.4f, torque2duty(RSR, pre_gain*ct_33x1[RSR_33]) + pre_gain*fric_compen_33x1[RSR_33] + duty_joint_limit_33x1[RSR_33]);
		pSharedMemory->disp_duty_33x1[RSY_33] = duty_33x1[RSY_33] = limitDuty(0.4f, torque2duty(RSY, pre_gain*ct_33x1[RSY_33]) + pre_gain*fric_compen_33x1[RSY_33] + duty_joint_limit_33x1[RSY_33]);
		pSharedMemory->disp_duty_33x1[REB_33] = duty_33x1[REB_33] = limitDuty(0.4f, torque2duty(REB, pre_gain*ct_33x1[REB_33]) + pre_gain*fric_compen_33x1[REB_33] + duty_joint_limit_33x1[REB_33]);
		pSharedMemory->disp_duty_33x1[RWY_33] = duty_33x1[RWY_33] = limitDuty(0.4f, torque2duty(RWY, pre_gain*ct_33x1[RWY_33]) + pre_gain*fric_compen_33x1[RWY_33] + duty_joint_limit_33x1[RWY_33]);
		pSharedMemory->disp_duty_33x1[RWP_33] = duty_33x1[RWP_33] = limitDuty(0.4f, torque2duty(RWP, pre_gain*ct_33x1[RWP_33]) + pre_gain*fric_compen_33x1[RWP_33] + duty_joint_limit_33x1[RWP_33]);
		pSharedMemory->disp_duty_33x1[RWY2_33] = duty_33x1[RWY2_33] = limitDuty(0.4f, torque2duty(RWY2, pre_gain*ct_33x1[RWY2_33]) + pre_gain*fric_compen_33x1[RWY2_33] + duty_joint_limit_33x1[RWY2_33]);

		pSharedMemory->disp_duty_33x1[LSP_33] = duty_33x1[LSP_33] = limitDuty(0.4f, torque2duty(LSP, pre_gain*ct_33x1[LSP_33]) + pre_gain*fric_compen_33x1[LSP_33] + duty_joint_limit_33x1[LSP_33]);
		pSharedMemory->disp_duty_33x1[LSR_33] = duty_33x1[LSR_33] = limitDuty(0.4f, torque2duty(LSR, pre_gain*ct_33x1[LSR_33]) + pre_gain*fric_compen_33x1[LSR_33] + duty_joint_limit_33x1[LSR_33]);
		pSharedMemory->disp_duty_33x1[LSY_33] = duty_33x1[LSY_33] = limitDuty(0.4f, torque2duty(LSY, pre_gain*ct_33x1[LSY_33]) + pre_gain*fric_compen_33x1[LSY_33] + duty_joint_limit_33x1[LSY_33]);
		pSharedMemory->disp_duty_33x1[LEB_33] = duty_33x1[LEB_33] = limitDuty(0.4f, torque2duty(LEB, pre_gain*ct_33x1[LEB_33]) + pre_gain*fric_compen_33x1[LEB_33] + duty_joint_limit_33x1[LEB_33]);
		pSharedMemory->disp_duty_33x1[LWY_33] = duty_33x1[LWY_33] = limitDuty(0.4f, torque2duty(LWY, pre_gain*ct_33x1[LWY_33]) + pre_gain*fric_compen_33x1[LWY_33] + duty_joint_limit_33x1[LWY_33]);
		pSharedMemory->disp_duty_33x1[LWP_33] = duty_33x1[LWP_33] = limitDuty(0.4f, torque2duty(LWP, pre_gain*ct_33x1[LWP_33]) + pre_gain*fric_compen_33x1[LWP_33] + duty_joint_limit_33x1[LWP_33]);
		pSharedMemory->disp_duty_33x1[LWY2_33] = duty_33x1[LWY2_33] = limitDuty(0.4f, torque2duty(LWY2, pre_gain*ct_33x1[LWY2_33]) + pre_gain*fric_compen_33x1[LWY2_33] + duty_joint_limit_33x1[LWY2_33]);
		
		RBpwmCommandHR2ch(CAN1, Joint[RSP].JMC, (int)(PWM_SIGN_RSP*1000.f*duty_33x1[RSP_33]), (int)(PWM_SIGN_RSR*1000.f*duty_33x1[RSR_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[RSY].JMC, (int)(PWM_SIGN_RSY*1000.f*duty_33x1[RSY_33]), (int)(PWM_SIGN_REB*1000.f*duty_33x1[REB_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[RWY].JMC, (int)(PWM_SIGN_RWY*1000.f*duty_33x1[RWY_33]), (int)(PWM_SIGN_RWP*1000.f*duty_33x1[RWP_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[RWY2].JMC, (int)(PWM_SIGN_RWY2*1000.f*duty_33x1[RWY2_33]), 0, 1);

		RBpwmCommandHR2ch(CAN1, Joint[LSP].JMC, (int)(PWM_SIGN_LSP*1000.f*duty_33x1[LSP_33]), (int)(PWM_SIGN_LSR*1000.f*duty_33x1[LSR_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[LSY].JMC, (int)(PWM_SIGN_LSY*1000.f*duty_33x1[LSY_33]), (int)(PWM_SIGN_LEB*1000.f*duty_33x1[LEB_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[LWY].JMC, (int)(PWM_SIGN_LWY*1000.f*duty_33x1[LWY_33]), (int)(PWM_SIGN_LWP*1000.f*duty_33x1[LWP_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[LWY2].JMC, (int)(PWM_SIGN_LWY2*1000.f*duty_33x1[LWY2_33]), 0, 1);
		
	}
	
	return 0;	
}


int WBIK_DRC_quad(unsigned int n)  
{
	int i,j;
	
	float Qpp_33x1[34], meas_Q_34x1[35], meas_Qp_33x1[34];
	float X1_33x1[34], Xrf_6x1[7], Xlf_6x1[7], Xrh_6x1[7], Xlh_6x1[7], Xcom_3x1[4], Xpel_3x1[4], Xwst, Xpelz, X2_33x1[34];  // task variables
	float ub_27x1[28], lb_27x1[28], qpp_limited[28];  // joint upper-bound, lower-bound
	int index_limited[28];
	
	float des_pCOM_3x1[4], des_vCOM_3x1[4];
	float des_pRF_3x1[4], des_pLF_3x1[4], des_vRF_3x1[4], des_vLF_3x1[4];
	float des_qRF_4x1[5], des_qLF_4x1[5], des_wRF_3x1[4], des_wLF_3x1[4];
	float des_pRH_3x1[4], des_pLH_3x1[4], des_vRH_3x1[4], des_vLH_3x1[4];
	float des_qRH_4x1[5], des_qLH_4x1[5], des_wRH_3x1[4], des_wLH_3x1[4];
	float des_WST, des_WSTp, des_qPEL_4x1[5];
	float des_pPELz, des_vPELz;
	float vCOM_3x1[4];

	BOOL isLimited = FALSE;
	int dim_primary_task, dim_redundant_task;

	float lL, lR, fRFz, fLFz;
	float rhr_compen = 0.f;
	float lhr_compen = 0.f;
	float rhy_compen = 0.f;
	float lhy_compen = 0.f;
	float fric_compen_33x1[34], ct_33x1[34], gravity_33x1[34], duty_33x1[34], duty_joint_limit_33x1[34];

	static char half_flag;
	static int count, count_walking;
	static float neutral_Q_34x1[35], Q_old_34x1[35], Q_old2_34x1[35];
	static float offset_pPELz, offset_pRH_3x1[4], offset_pLH_3x1[4], offset_qRH_4x1[5], offset_qLH_4x1[5]; 
	static float pCOM1_3x1[4], pPELz1, pRF1_3x1[4], pLF1_3x1[4], qRF1_4x1[5], qLF1_4x1[5], qPEL1_4x1[5];
	static float pCOM2_3x1[4], pPELz2, pRF2_3x1[4], pLF2_3x1[4], qRF2_4x1[5], qLF2_4x1[5];
	static float pRH1_3x1[4], pLH1_3x1[4], qRH1_4x1[5], qLH1_4x1[5];
	static float pRH2_3x1[4], pLH2_3x1[4], qRH2_4x1[5], qLH2_4x1[5];
	static float t;

	float ftemp1_7x1[8], ftemp2_7x1[8], ftemp3_7x1[8], ftemp4_7x1[8];
	float pre_gain = 0.f;
	float pre_gain_lb = 0.f;
	char update_pos1_flag = 0;

	static float dpRFstop_3x1[4], dpLFstop_3x1[4], dpRHstop_3x1[4], dpLHstop_3x1[4];
	static char LF_LC_flag, RF_LC_flag, LH_LC_flag, RH_LC_flag;
	static unsigned int LF_LC_count, RF_LC_count, LH_LC_count, RH_LC_count;
	static int des_fsp_old;
	static unsigned int j_change;

	float zmp_f, zmp_s;
	float supp_center_x, supp_center_y;
	float r11, r12, r21, r22;
	float damp_f = 0.;
	float damp_s = 0.;
	float des_pCOM_s, des_pCOM_f;

	float ref_Q_34x1[35];
	float gain_ovr_rarm, gain_ovr_larm, gain_ovr_rank, gain_ovr_lank;

	if(_FKineUpdatedFlag != 1)
		return -4;
	
	_Q_34x1[RWY2_34] = 0.f;
	_Q_34x1[LWY2_34] = 0.f;
	if(n<=2)
	{
		LF_LC_flag = 0;
		RF_LC_flag = 0;
		LH_LC_flag = 0;
		RH_LC_flag = 0;
		LF_LC_count = 0;
		RF_LC_count = 0;
		LH_LC_count = 0;
		RH_LC_count = 0;
		j_change = 0;

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

		pRH1_3x1[1] = _pRWR_3x1[1]; //_pRH0_3x1[1] = _pRH_3x1[1];
		pRH1_3x1[2] = _pRWR_3x1[2]; //_pRH0_3x1[2] = _pRH_3x1[2];
		pRH1_3x1[3] = _pRWR_3x1[3]; //_pRH0_3x1[3] = _pRH_3x1[3];

		qRH1_4x1[1] = _qRWR_4x1[1]; //_qRH0_4x1[1] = _qRH_4x1[1];
		qRH1_4x1[2] = _qRWR_4x1[2]; //_qRH0_4x1[2] = _qRH_4x1[2];
		qRH1_4x1[3] = _qRWR_4x1[3]; //_qRH0_4x1[3] = _qRH_4x1[3];
		qRH1_4x1[4] = _qRWR_4x1[4]; //_qRH0_4x1[4] = _qRH_4x1[4];

		pLH1_3x1[1] = _pLWR_3x1[1];// _pLH0_3x1[1] = _pLH_3x1[1];
		pLH1_3x1[2] = _pLWR_3x1[2];//_pLH0_3x1[2] = _pLH_3x1[2];
		pLH1_3x1[3] = _pLWR_3x1[3];//_pLH0_3x1[3] = _pLH_3x1[3];

		qLH1_4x1[1] = _qLWR_4x1[1];//_qLH0_4x1[1] = _qLH_4x1[1];
		qLH1_4x1[2] = _qLWR_4x1[2];//_qLH0_4x1[2] = _qLH_4x1[2];
		qLH1_4x1[3] = _qLWR_4x1[3];//_qLH0_4x1[3] = _qLH_4x1[3];
		qLH1_4x1[4] = _qLWR_4x1[4];//_qLH0_4x1[4] = _qLH_4x1[4];

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

		diff_mm((const float**)_jRWR_6x33,6,33, (const float**)_jRWR_6x33, _jRWRp_6x33);
		subs_m((const float**)_jRWR_6x33,6,33, _jRWR_old_6x33);
		subs_m((const float**)_jRWR_6x33,6,33, _jRWR_old2_6x33);
		
		diff_mm((const float**)_jLWR_6x33,6,33, (const float**)_jLWR_6x33, _jLWRp_6x33);
		subs_m((const float**)_jLWR_6x33,6,33, _jLWR_old_6x33);
		subs_m((const float**)_jLWR_6x33,6,33, _jLWR_old2_6x33);

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
			pSharedMemory->transform_flag = 3;
			pSharedMemory->torque_mode_flag = 1;
			printf("done");
		}
	}
	else if(pSharedMemory->transform_flag == 3) // staying
	{
		des_qPEL_4x1[1] = qPEL1_4x1[1];
		des_qPEL_4x1[2] = qPEL1_4x1[2];
		des_qPEL_4x1[3] = qPEL1_4x1[3];
		des_qPEL_4x1[4] = qPEL1_4x1[4];
		
		des_pPELz =  pPELz1;
		
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
	
	if(pSharedMemory->transform_flag != 4)  // transition of damping controller 
	{
		supp_center_x = (des_pLF_3x1[1] + des_pRF_3x1[1])*0.5f;
		supp_center_y = (des_pLF_3x1[2] + des_pRF_3x1[2])*0.5f;

		ftemp1_7x1[0] = (float)sqrt((float)pow(des_pLF_3x1[1]-des_pRF_3x1[1], 2.f) + (float)pow(des_pLF_3x1[2]-des_pRF_3x1[2], 2.f));
		
		r12 = (des_pLF_3x1[1]-des_pRF_3x1[1])/ftemp1_7x1[0];
		r22 = (des_pLF_3x1[2]-des_pRF_3x1[2])/ftemp1_7x1[0];
		r11 = r22;
		r21 = -r12;

		ftemp1_7x1[0] = _Q_34x1[1] + _pZMP_3x1[1] - supp_center_x;
		ftemp1_7x1[1] = _Q_34x1[2] + _pZMP_3x1[2] - supp_center_y;
		zmp_s = r11*ftemp1_7x1[0] + r21*ftemp1_7x1[1];
		zmp_f = r12*ftemp1_7x1[0] + r22*ftemp1_7x1[1];

		ftemp1_7x1[0] = des_pCOM_3x1[1] - supp_center_x;
		ftemp1_7x1[1] = des_pCOM_3x1[2] - supp_center_y;
		des_pCOM_s = r11*ftemp1_7x1[0] + r21*ftemp1_7x1[1];
		des_pCOM_f = r12*ftemp1_7x1[0] + r22*ftemp1_7x1[1];

		ftemp1_7x1[0] = _Xhat_DSP_F_4x1[1] + DT*(_Ahat_DSP_F_DRC_4x4[1][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[1][2]*_Xhat_DSP_F_4x1[2] +
			_Ahat_DSP_F_DRC_4x4[1][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[1][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[1]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[1]*zmp_f);
		ftemp1_7x1[1] = _Xhat_DSP_F_4x1[2] + DT*(_Ahat_DSP_F_DRC_4x4[2][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[2][2]*_Xhat_DSP_F_4x1[2] +
			_Ahat_DSP_F_DRC_4x4[2][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[2][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[2]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[2]*zmp_f);
		ftemp1_7x1[2] = _Xhat_DSP_F_4x1[3] + DT*(_Ahat_DSP_F_DRC_4x4[3][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[3][2]*_Xhat_DSP_F_4x1[2] +
			_Ahat_DSP_F_DRC_4x4[3][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[3][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[3]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[3]*zmp_f);
		_Xhat_DSP_F_4x1[4] = _Xhat_DSP_F_4x1[4] + DT*(_Ahat_DSP_F_DRC_4x4[4][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[4][2]*_Xhat_DSP_F_4x1[2] +
			_Ahat_DSP_F_DRC_4x4[4][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[4][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[4]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[4]*zmp_f);
		_Xhat_DSP_F_4x1[1] = ftemp1_7x1[0];
		_Xhat_DSP_F_4x1[2] = ftemp1_7x1[1];
		_Xhat_DSP_F_4x1[3] = ftemp1_7x1[2];

		ftemp1_7x1[0] = _Xhat_DSP_S_4x1[1] + DT*(_Ahat_DSP_S_DRC_4x4[1][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[1][2]*_Xhat_DSP_S_4x1[2] +
			_Ahat_DSP_S_DRC_4x4[1][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[1][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[1]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[1]*zmp_s);
		ftemp1_7x1[1] = _Xhat_DSP_S_4x1[2] + DT*(_Ahat_DSP_S_DRC_4x4[2][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[2][2]*_Xhat_DSP_S_4x1[2] +
			_Ahat_DSP_S_DRC_4x4[2][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[2][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[2]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[2]*zmp_s);
		ftemp1_7x1[2] = _Xhat_DSP_S_4x1[3] + DT*(_Ahat_DSP_S_DRC_4x4[3][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[3][2]*_Xhat_DSP_S_4x1[2] +
			_Ahat_DSP_S_DRC_4x4[3][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[3][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[3]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[3]*zmp_s);
		_Xhat_DSP_S_4x1[4] = _Xhat_DSP_S_4x1[4] + DT*(_Ahat_DSP_S_DRC_4x4[4][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[4][2]*_Xhat_DSP_S_4x1[2] +
			_Ahat_DSP_S_DRC_4x4[4][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[4][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[4]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[4]*zmp_s);
		_Xhat_DSP_S_4x1[1] = ftemp1_7x1[0];
		_Xhat_DSP_S_4x1[2] = ftemp1_7x1[1];
		_Xhat_DSP_S_4x1[3] = ftemp1_7x1[2];

		ftemp1_7x1[0] = _Xhat_SSP_F_4x1[1] + DT*(_Ahat_SSP_F_DRC_4x4[1][1]*_Xhat_SSP_F_4x1[1]+_Ahat_SSP_F_DRC_4x4[1][2]*_Xhat_SSP_F_4x1[2] +
			_Ahat_SSP_F_DRC_4x4[1][3]*_Xhat_SSP_F_4x1[3]+_Ahat_SSP_F_DRC_4x4[1][4]*_Xhat_SSP_F_4x1[4]+ _Bhat_SSP_F_DRC_4x1[1]*des_pCOM_f + _Lo_SSP_F_DRC_4x1[1]*zmp_f);
		ftemp1_7x1[1] = _Xhat_SSP_F_4x1[2] + DT*(_Ahat_SSP_F_DRC_4x4[2][1]*_Xhat_SSP_F_4x1[1]+_Ahat_SSP_F_DRC_4x4[2][2]*_Xhat_SSP_F_4x1[2] +
			_Ahat_SSP_F_DRC_4x4[2][3]*_Xhat_SSP_F_4x1[3]+_Ahat_SSP_F_DRC_4x4[2][4]*_Xhat_SSP_F_4x1[4]+ _Bhat_SSP_F_DRC_4x1[2]*des_pCOM_f + _Lo_SSP_F_DRC_4x1[2]*zmp_f);
		ftemp1_7x1[2] = _Xhat_SSP_F_4x1[3] + DT*(_Ahat_SSP_F_DRC_4x4[3][1]*_Xhat_SSP_F_4x1[1]+_Ahat_SSP_F_DRC_4x4[3][2]*_Xhat_SSP_F_4x1[2] +
			_Ahat_SSP_F_DRC_4x4[3][3]*_Xhat_SSP_F_4x1[3]+_Ahat_SSP_F_DRC_4x4[3][4]*_Xhat_SSP_F_4x1[4]+ _Bhat_SSP_F_DRC_4x1[3]*des_pCOM_f + _Lo_SSP_F_DRC_4x1[3]*zmp_f);
		_Xhat_SSP_F_4x1[4] = _Xhat_SSP_F_4x1[4] + DT*(_Ahat_SSP_F_DRC_4x4[4][1]*_Xhat_SSP_F_4x1[1]+_Ahat_SSP_F_DRC_4x4[4][2]*_Xhat_SSP_F_4x1[2] +
			_Ahat_SSP_F_DRC_4x4[4][3]*_Xhat_SSP_F_4x1[3]+_Ahat_SSP_F_DRC_4x4[4][4]*_Xhat_SSP_F_4x1[4]+ _Bhat_SSP_F_DRC_4x1[4]*des_pCOM_f + _Lo_SSP_F_DRC_4x1[4]*zmp_f);
		_Xhat_SSP_F_4x1[1] = ftemp1_7x1[0];
		_Xhat_SSP_F_4x1[2] = ftemp1_7x1[1];
		_Xhat_SSP_F_4x1[3] = ftemp1_7x1[2];

		ftemp1_7x1[0] = _Xhat_SSP_S_4x1[1] + DT*(_Ahat_SSP_S_DRC_4x4[1][1]*_Xhat_SSP_S_4x1[1]+_Ahat_SSP_S_DRC_4x4[1][2]*_Xhat_SSP_S_4x1[2] +
			_Ahat_SSP_S_DRC_4x4[1][3]*_Xhat_SSP_S_4x1[3]+_Ahat_SSP_S_DRC_4x4[1][4]*_Xhat_SSP_S_4x1[4]+ _Bhat_SSP_S_DRC_4x1[1]*des_pCOM_s + _Lo_SSP_S_DRC_4x1[1]*zmp_s);
		ftemp1_7x1[1] = _Xhat_SSP_S_4x1[2] + DT*(_Ahat_SSP_S_DRC_4x4[2][1]*_Xhat_SSP_S_4x1[1]+_Ahat_SSP_S_DRC_4x4[2][2]*_Xhat_SSP_S_4x1[2] +
			_Ahat_SSP_S_DRC_4x4[2][3]*_Xhat_SSP_S_4x1[3]+_Ahat_SSP_S_DRC_4x4[2][4]*_Xhat_SSP_S_4x1[4]+ _Bhat_SSP_S_DRC_4x1[2]*des_pCOM_s + _Lo_SSP_S_DRC_4x1[2]*zmp_s);
		ftemp1_7x1[2] = _Xhat_SSP_S_4x1[3] + DT*(_Ahat_SSP_S_DRC_4x4[3][1]*_Xhat_SSP_S_4x1[1]+_Ahat_SSP_S_DRC_4x4[3][2]*_Xhat_SSP_S_4x1[2] +
			_Ahat_SSP_S_DRC_4x4[3][3]*_Xhat_SSP_S_4x1[3]+_Ahat_SSP_S_DRC_4x4[3][4]*_Xhat_SSP_S_4x1[4]+ _Bhat_SSP_S_DRC_4x1[3]*des_pCOM_s + _Lo_SSP_S_DRC_4x1[3]*zmp_s);
		_Xhat_SSP_S_4x1[4] = _Xhat_SSP_S_4x1[4] + DT*(_Ahat_SSP_S_DRC_4x4[4][1]*_Xhat_SSP_S_4x1[1]+_Ahat_SSP_S_DRC_4x4[4][2]*_Xhat_SSP_S_4x1[2] +
			_Ahat_SSP_S_DRC_4x4[4][3]*_Xhat_SSP_S_4x1[3]+_Ahat_SSP_S_DRC_4x4[4][4]*_Xhat_SSP_S_4x1[4]+ _Bhat_SSP_S_DRC_4x1[4]*des_pCOM_s + _Lo_SSP_S_DRC_4x1[4]*zmp_s);
		_Xhat_SSP_S_4x1[1] = ftemp1_7x1[0];
		_Xhat_SSP_S_4x1[2] = ftemp1_7x1[1];
		_Xhat_SSP_S_4x1[3] = ftemp1_7x1[2];
	}
	else
	{
		if(_offline_traj.i < _offline_traj.length) //------ walking
		{		
			des_qPEL_4x1[1] = qPEL1_4x1[1];
			des_qPEL_4x1[2] = qPEL1_4x1[2];
			des_qPEL_4x1[3] = qPEL1_4x1[3];
			des_qPEL_4x1[4] = qPEL1_4x1[4];
			
			des_pPELz =  pPELz1 + _offline_traj.dpPELz[_offline_traj.i];
			
			des_pCOM_3x1[1] = pCOM1_3x1[1] + _offline_traj.dpCOMx[_offline_traj.i];
			des_pCOM_3x1[2] = pCOM1_3x1[2] + _offline_traj.dpCOMy[_offline_traj.i];

			des_pRF_3x1[1] = pRF1_3x1[1] + _offline_traj.dpRFx[_offline_traj.i];				
			des_pRF_3x1[2] = pRF1_3x1[2] + _offline_traj.dpRFy[_offline_traj.i];
			des_pRF_3x1[3] = pRF1_3x1[3] + _offline_traj.dpRFz[_offline_traj.i];
			
			des_pLF_3x1[1] = pLF1_3x1[1] + _offline_traj.dpLFx[_offline_traj.i];				
			des_pLF_3x1[2] = pLF1_3x1[2] + _offline_traj.dpLFy[_offline_traj.i];
			des_pLF_3x1[3] = pLF1_3x1[3] + _offline_traj.dpLFz[_offline_traj.i];			
			
			des_qRF_4x1[1] = qRF1_4x1[1];
			des_qRF_4x1[2] = qRF1_4x1[2];
			des_qRF_4x1[3] = qRF1_4x1[3];
			des_qRF_4x1[4] = qRF1_4x1[4];

			des_qLF_4x1[1] = qLF1_4x1[1];
			des_qLF_4x1[2] = qLF1_4x1[2];
			des_qLF_4x1[3] = qLF1_4x1[3];
			des_qLF_4x1[4] = qLF1_4x1[4];

			
			if(_DRC_walking_mode == DRC_QUAD_MODE)
			{
				des_pRH_3x1[1] = pRH1_3x1[1] + _offline_traj.dpRHx[_offline_traj.i];
				des_pRH_3x1[2] = pRH1_3x1[2] + _offline_traj.dpRHy[_offline_traj.i];
				des_pRH_3x1[3] = pRH1_3x1[3] + _offline_traj.dpRHz[_offline_traj.i];

				des_pLH_3x1[1] = pLH1_3x1[1] + _offline_traj.dpLHx[_offline_traj.i];
				des_pLH_3x1[2] = pLH1_3x1[2] + _offline_traj.dpLHy[_offline_traj.i];
				des_pLH_3x1[3] = pLH1_3x1[3] + _offline_traj.dpLHz[_offline_traj.i];
			}
			else
			{
				des_pRH_3x1[1] = pRH1_3x1[1];
				des_pRH_3x1[2] = pRH1_3x1[2];
				des_pRH_3x1[3] = pRH1_3x1[3];		

				des_pLH_3x1[1] = pLH1_3x1[1];
				des_pLH_3x1[2] = pLH1_3x1[2];
				des_pLH_3x1[3] = pLH1_3x1[3];
			}
			des_qRH_4x1[1] = qRH1_4x1[1];
			des_qRH_4x1[2] = qRH1_4x1[2];
			des_qRH_4x1[3] = qRH1_4x1[3];
			des_qRH_4x1[4] = qRH1_4x1[4];

			des_qLH_4x1[1] = qLH1_4x1[1];
			des_qLH_4x1[2] = qLH1_4x1[2];
			des_qLH_4x1[3] = qLH1_4x1[3];
			des_qLH_4x1[4] = qLH1_4x1[4];

			if(_offline_traj.i < _offline_traj.i_change[j_change][0]) //  intermediate DFSP
			{
				if(LF_LC_flag == 1)
					des_pLF_3x1[3] = pLF1_3x1[3] + dpLFstop_3x1[3];
				if(RF_LC_flag == 1)
					des_pRF_3x1[3] = pRF1_3x1[3] + dpRFstop_3x1[3];

				if(_DRC_walking_mode == DRC_QUAD_MODE)
				{
					if(LH_LC_flag == 1)
						des_pLH_3x1[3] = pLH1_3x1[3] + dpLHstop_3x1[3];
					if(RH_LC_flag == 1)
						des_pRH_3x1[3] = pRH1_3x1[3] + dpRHstop_3x1[3];
				}

				
				/*if(_DRC_walking_mode == DRC_QUAD_MODE)
				{
					if(LH_LC_flag == 1)
						des_pLH_3x1[3] = pLH1_3x1[3] + dpLHstop_3x1[3];
					if(RH_LC_flag == 1)
						des_pRH_3x1[3] = pRH1_3x1[3] + dpRHstop_3x1[3];

					supp_center_x = (des_pLF_3x1[1] + des_pRF_3x1[1])*0.5f;
					supp_center_y = (des_pLF_3x1[2] + des_pRF_3x1[2])*0.5f;

					ftemp1_7x1[0] = (float)sqrt((float)pow(des_pLF_3x1[1]-des_pRF_3x1[1], 2.f) + (float)pow(des_pLF_3x1[2]-des_pRF_3x1[2], 2.f));
					
					r12 = (des_pLF_3x1[1]-des_pRF_3x1[1])/ftemp1_7x1[0];
					r22 = (des_pLF_3x1[2]-des_pRF_3x1[2])/ftemp1_7x1[0];
					r11 = r22;
					r21 = -r12;

					ftemp1_7x1[0] = _Q_34x1[1] + _pZMP_3x1[1] - supp_center_x;
					ftemp1_7x1[1] = _Q_34x1[2] + _pZMP_3x1[2] - supp_center_y;
					zmp_s = r11*ftemp1_7x1[0] + r21*ftemp1_7x1[1];
					zmp_f = r12*ftemp1_7x1[0] + r22*ftemp1_7x1[1];

					ftemp1_7x1[0] = des_pCOM_3x1[1] - supp_center_x;
					ftemp1_7x1[1] = des_pCOM_3x1[2] - supp_center_y;
					des_pCOM_s = r11*ftemp1_7x1[0] + r21*ftemp1_7x1[1];
					des_pCOM_f = r12*ftemp1_7x1[0] + r22*ftemp1_7x1[1];

					damp_f =  -(_Kfb_DSP_F_DRC_1x4[1]*_Xhat_DSP_F_4x1[1]+_Kfb_DSP_F_DRC_1x4[2]*_Xhat_DSP_F_4x1[2]+_Kfb_DSP_F_DRC_1x4[3]*_Xhat_DSP_F_4x1[3]+_Kfb_DSP_F_DRC_1x4[4]*_Xhat_DSP_F_4x1[4]);
					damp_s =  -(_Kfb_DSP_S_DRC_1x4[1]*_Xhat_DSP_S_4x1[1]+_Kfb_DSP_S_DRC_1x4[2]*_Xhat_DSP_S_4x1[2]+_Kfb_DSP_S_DRC_1x4[3]*_Xhat_DSP_S_4x1[3]+_Kfb_DSP_S_DRC_1x4[4]*_Xhat_DSP_S_4x1[4]);
					
					if(count_walking < N2_TRANSITION)
					{	
						ftemp1_7x1[0] = (float)count_walking/((float)N2_TRANSITION);
						damp_f *= (float)pow(ftemp1_7x1[0],3.f);
						damp_s *= (float)pow(ftemp1_7x1[0],3.f);
						count_walking++;
					}
					
					ftemp1_7x1[0] = 2.f*(float)(_offline_traj.i_change[j_change][0]-_offline_traj.i)/((float)(_offline_traj.i_change[j_change][1]-_offline_traj.i_change[j_change][0]));
					if(ftemp1_7x1[0] < 1.f)
					{
						damp_f *= (float)pow(ftemp1_7x1[0], 3.f);
						damp_s *= (float)pow(ftemp1_7x1[0], 3.f);
					}
					
					if(pSharedMemory->temp_dsp_damp_on_flag == 1)
					{
						des_pCOM_s += damp_s;
						des_pCOM_f += damp_f;
					}
					
					ftemp1_7x1[0] = _Xhat_DSP_F_4x1[1] + DT*(_Ahat_DSP_F_DRC_4x4[1][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[1][2]*_Xhat_DSP_F_4x1[2] +
						_Ahat_DSP_F_DRC_4x4[1][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[1][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[1]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[1]*zmp_f);
					ftemp1_7x1[1] = _Xhat_DSP_F_4x1[2] + DT*(_Ahat_DSP_F_DRC_4x4[2][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[2][2]*_Xhat_DSP_F_4x1[2] +
						_Ahat_DSP_F_DRC_4x4[2][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[2][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[2]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[2]*zmp_f);
					ftemp1_7x1[2] = _Xhat_DSP_F_4x1[3] + DT*(_Ahat_DSP_F_DRC_4x4[3][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[3][2]*_Xhat_DSP_F_4x1[2] +
						_Ahat_DSP_F_DRC_4x4[3][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[3][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[3]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[3]*zmp_f);
					_Xhat_DSP_F_4x1[4] = _Xhat_DSP_F_4x1[4] + DT*(_Ahat_DSP_F_DRC_4x4[4][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[4][2]*_Xhat_DSP_F_4x1[2] +
						_Ahat_DSP_F_DRC_4x4[4][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[4][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[4]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[4]*zmp_f);
					_Xhat_DSP_F_4x1[1] = ftemp1_7x1[0];
					_Xhat_DSP_F_4x1[2] = ftemp1_7x1[1];
					_Xhat_DSP_F_4x1[3] = ftemp1_7x1[2];

					ftemp1_7x1[0] = _Xhat_DSP_S_4x1[1] + DT*(_Ahat_DSP_S_DRC_4x4[1][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[1][2]*_Xhat_DSP_S_4x1[2] +
						_Ahat_DSP_S_DRC_4x4[1][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[1][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[1]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[1]*zmp_s);
					ftemp1_7x1[1] = _Xhat_DSP_S_4x1[2] + DT*(_Ahat_DSP_S_DRC_4x4[2][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[2][2]*_Xhat_DSP_S_4x1[2] +
						_Ahat_DSP_S_DRC_4x4[2][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[2][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[2]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[2]*zmp_s);
					ftemp1_7x1[2] = _Xhat_DSP_S_4x1[3] + DT*(_Ahat_DSP_S_DRC_4x4[3][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[3][2]*_Xhat_DSP_S_4x1[2] +
						_Ahat_DSP_S_DRC_4x4[3][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[3][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[3]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[3]*zmp_s);
					_Xhat_DSP_S_4x1[4] = _Xhat_DSP_S_4x1[4] + DT*(_Ahat_DSP_S_DRC_4x4[4][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[4][2]*_Xhat_DSP_S_4x1[2] +
						_Ahat_DSP_S_DRC_4x4[4][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[4][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[4]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[4]*zmp_s);
					_Xhat_DSP_S_4x1[1] = ftemp1_7x1[0];
					_Xhat_DSP_S_4x1[2] = ftemp1_7x1[1];
					_Xhat_DSP_S_4x1[3] = ftemp1_7x1[2];						

					des_pCOM_3x1[1] = r12*des_pCOM_f + r11*des_pCOM_s + supp_center_x;
					des_pCOM_3x1[2] = r22*des_pCOM_f + r21*des_pCOM_s + supp_center_y;

					ftemp1_7x1[0] = 0.5f*(float)(_offline_traj.i_change[j_change][1]-_offline_traj.i_change[j_change][0]);						
					if(LF_LC_flag == 1)
					{
						LF_LC_count++;
						des_pLF_3x1[3] = pLF1_3x1[3] + 0.5f*(1.f+(float)cosf(PI*(float)LF_LC_count/ftemp1_7x1[0]))*dpLFstop_3x1[3];
						if(LF_LC_count >= (unsigned int)ftemp1_7x1[0])
						{
							printf("\n All landing compensations - LF recovered");
							LF_LC_flag = 0;
							LF_LC_count = 0;
							dpLFstop_3x1[3] = 0;
						}
					}						
					if(RF_LC_flag == 1)
					{
						RF_LC_count++;
						des_pRF_3x1[3] = pRF1_3x1[3] + 0.5f*(1.f+(float)cosf(PI*(float)RF_LC_count/ftemp1_7x1[0]))*dpRFstop_3x1[3];
						if(RF_LC_count >= (unsigned int)ftemp1_7x1[0])
						{
							printf("\n All landing compensations - RF recovered");
							RF_LC_flag = 0;
							RF_LC_count = 0;
							dpRFstop_3x1[3] = 0;
						}
					}

					if(pSharedMemory->temp_hand_LC_on_flag == 1)
					{
						if(RH_LC_flag == 1)
						{
							if(des_pRH_3x1[3] < pRH1_3x1[3]+dpRHstop_3x1[3])
								des_pRH_3x1[3] = pRH1_3x1[3] + dpRHstop_3x1[3];
							else
							{
								printf("\n Early-Landing compensation - RH recovered ");
								RH_LC_flag = 0;
								RH_LC_count = 0;
								dpRHstop_3x1[3] = 0.f;
							}
						}
						else if((des_pRH_3x1[3]>pRH1_3x1[3]+_offline_traj.dpRHz[_offline_traj.i+1]) && (_HSP==DHSP || _HSP==RHSP)) // early landing of RH
						{
							dpRHstop_3x1[3] = des_pRH_3x1[3]-pRH1_3x1[3] + pSharedMemory->temp_EL_compen;
							des_pRH_3x1[3] = pRH1_3x1[3] + dpRHstop_3x1[3];
							RH_LC_flag = 1;
							RH_LC_count = 0;
							printf("\n Early-Landing occured - RH %f", dpRHstop_3x1[3]);
						}
						
						if(LH_LC_flag == 1)
						{
							if(des_pLH_3x1[3] < pLH1_3x1[3]+dpLHstop_3x1[3])
								des_pLH_3x1[3] = pLH1_3x1[3]+dpLHstop_3x1[3];
							else
							{
								printf("\n Early-Landing compensation - LH recovered ");
								LH_LC_flag = 0;
								LH_LC_count = 0;
								dpLHstop_3x1[3] = 0.f;
							}
						}
						else if((des_pLH_3x1[3] > pLH1_3x1[3]+_offline_traj.dpLHz[_offline_traj.i+1]) && (_HSP==DHSP || _HSP==LHSP)) // early landing of LH
						{
							dpLHstop_3x1[3] = des_pLH_3x1[3]-pLH1_3x1[3] + pSharedMemory->temp_EL_compen;
							des_pLH_3x1[3] = pLH1_3x1[3] + dpLHstop_3x1[3];
							LH_LC_flag = 1;
							LH_LC_count = 0;
							printf("\n Early-Landing occured - LH %f", dpLHstop_3x1[3]);
						}
					}
				}*/

				if(_offline_traj.i == _offline_traj.i_change[j_change][0]-1)
					count_walking = 0;
			}
			else if(_offline_traj.i < _offline_traj.i_change[j_change][1])
			{
				switch(_offline_traj.des_fsp[j_change])
				{
				case DFSP:
					supp_center_x = (des_pLF_3x1[1] + des_pRF_3x1[1])*0.5f;
					supp_center_y = (des_pLF_3x1[2] + des_pRF_3x1[2])*0.5f;

					ftemp1_7x1[0] = (float)sqrt((float)pow(des_pLF_3x1[1]-des_pRF_3x1[1], 2.f) + (float)pow(des_pLF_3x1[2]-des_pRF_3x1[2], 2.f));
					
					r12 = (des_pLF_3x1[1]-des_pRF_3x1[1])/ftemp1_7x1[0];
					r22 = (des_pLF_3x1[2]-des_pRF_3x1[2])/ftemp1_7x1[0];
					r11 = r22;
					r21 = -r12;

					ftemp1_7x1[0] = _Q_34x1[1] + _pZMP_3x1[1] - supp_center_x;
					ftemp1_7x1[1] = _Q_34x1[2] + _pZMP_3x1[2] - supp_center_y;
					zmp_s = r11*ftemp1_7x1[0] + r21*ftemp1_7x1[1];
					zmp_f = r12*ftemp1_7x1[0] + r22*ftemp1_7x1[1];

					ftemp1_7x1[0] = des_pCOM_3x1[1] - supp_center_x;
					ftemp1_7x1[1] = des_pCOM_3x1[2] - supp_center_y;
					des_pCOM_s = r11*ftemp1_7x1[0] + r21*ftemp1_7x1[1];
					des_pCOM_f = r12*ftemp1_7x1[0] + r22*ftemp1_7x1[1];

					damp_f =  -(_Kfb_DSP_F_DRC_1x4[1]*_Xhat_DSP_F_4x1[1]+_Kfb_DSP_F_DRC_1x4[2]*_Xhat_DSP_F_4x1[2]+_Kfb_DSP_F_DRC_1x4[3]*_Xhat_DSP_F_4x1[3]+_Kfb_DSP_F_DRC_1x4[4]*_Xhat_DSP_F_4x1[4]);
					damp_s =  -(_Kfb_DSP_S_DRC_1x4[1]*_Xhat_DSP_S_4x1[1]+_Kfb_DSP_S_DRC_1x4[2]*_Xhat_DSP_S_4x1[2]+_Kfb_DSP_S_DRC_1x4[3]*_Xhat_DSP_S_4x1[3]+_Kfb_DSP_S_DRC_1x4[4]*_Xhat_DSP_S_4x1[4]);
					
					if(count_walking < N2_TRANSITION)
					{	
						ftemp1_7x1[0] = (float)count_walking/((float)N2_TRANSITION);
						damp_f *= (float)pow(ftemp1_7x1[0], 3.f);
						damp_s *= (float)pow(ftemp1_7x1[0], 3.f);
						count_walking++;
					}
					
					ftemp1_7x1[0] = 2.f*(float)(_offline_traj.i_change[j_change][1]-_offline_traj.i)/((float)(_offline_traj.i_change[j_change][1]-_offline_traj.i_change[j_change][0]));
					if(ftemp1_7x1[0] < 1.f)
					{
						damp_f *= (float)pow(ftemp1_7x1[0], 3.f);
						damp_s *= (float)pow(ftemp1_7x1[0], 3.f);
					}

					if(pSharedMemory->temp_dsp_damp_on_flag == 1)
					{
						des_pCOM_s += damp_s;
						des_pCOM_f += damp_f;
					}
					
					ftemp1_7x1[0] = _Xhat_DSP_F_4x1[1] + DT*(_Ahat_DSP_F_DRC_4x4[1][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[1][2]*_Xhat_DSP_F_4x1[2] +
						_Ahat_DSP_F_DRC_4x4[1][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[1][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[1]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[1]*zmp_f);
					ftemp1_7x1[1] = _Xhat_DSP_F_4x1[2] + DT*(_Ahat_DSP_F_DRC_4x4[2][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[2][2]*_Xhat_DSP_F_4x1[2] +
						_Ahat_DSP_F_DRC_4x4[2][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[2][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[2]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[2]*zmp_f);
					ftemp1_7x1[2] = _Xhat_DSP_F_4x1[3] + DT*(_Ahat_DSP_F_DRC_4x4[3][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[3][2]*_Xhat_DSP_F_4x1[2] +
						_Ahat_DSP_F_DRC_4x4[3][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[3][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[3]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[3]*zmp_f);
					_Xhat_DSP_F_4x1[4] = _Xhat_DSP_F_4x1[4] + DT*(_Ahat_DSP_F_DRC_4x4[4][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[4][2]*_Xhat_DSP_F_4x1[2] +
						_Ahat_DSP_F_DRC_4x4[4][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[4][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[4]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[4]*zmp_f);
					_Xhat_DSP_F_4x1[1] = ftemp1_7x1[0];
					_Xhat_DSP_F_4x1[2] = ftemp1_7x1[1];
					_Xhat_DSP_F_4x1[3] = ftemp1_7x1[2];

					ftemp1_7x1[0] = _Xhat_DSP_S_4x1[1] + DT*(_Ahat_DSP_S_DRC_4x4[1][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[1][2]*_Xhat_DSP_S_4x1[2] +
						_Ahat_DSP_S_DRC_4x4[1][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[1][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[1]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[1]*zmp_s);
					ftemp1_7x1[1] = _Xhat_DSP_S_4x1[2] + DT*(_Ahat_DSP_S_DRC_4x4[2][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[2][2]*_Xhat_DSP_S_4x1[2] +
						_Ahat_DSP_S_DRC_4x4[2][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[2][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[2]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[2]*zmp_s);
					ftemp1_7x1[2] = _Xhat_DSP_S_4x1[3] + DT*(_Ahat_DSP_S_DRC_4x4[3][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[3][2]*_Xhat_DSP_S_4x1[2] +
						_Ahat_DSP_S_DRC_4x4[3][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[3][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[3]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[3]*zmp_s);
					_Xhat_DSP_S_4x1[4] = _Xhat_DSP_S_4x1[4] + DT*(_Ahat_DSP_S_DRC_4x4[4][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[4][2]*_Xhat_DSP_S_4x1[2] +
						_Ahat_DSP_S_DRC_4x4[4][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[4][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[4]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[4]*zmp_s);
					_Xhat_DSP_S_4x1[1] = ftemp1_7x1[0];
					_Xhat_DSP_S_4x1[2] = ftemp1_7x1[1];
					_Xhat_DSP_S_4x1[3] = ftemp1_7x1[2];						
					
					des_pCOM_3x1[1] = r12*des_pCOM_f + r11*des_pCOM_s + supp_center_x;
					des_pCOM_3x1[2] = r22*des_pCOM_f + r21*des_pCOM_s + supp_center_y;
					
					ftemp1_7x1[0] = 0.5f*(float)(_offline_traj.i_change[j_change][1]-_offline_traj.i_change[j_change][0]);						
					if(LF_LC_flag == 1)
					{
						LF_LC_count++;
						des_pLF_3x1[3] = pLF1_3x1[3] + 0.5f*(1.f+(float)cosf(PI*(float)LF_LC_count/ftemp1_7x1[0]))*dpLFstop_3x1[3];
						if(LF_LC_count >= (unsigned int)ftemp1_7x1[0])
						{
							printf("\n All landing compensations - LF recovered");
							LF_LC_flag = 0;
							LF_LC_count = 0;
							dpLFstop_3x1[3] = 0;
						}
					}						
					if(RF_LC_flag == 1)
					{
						RF_LC_count++;
						des_pRF_3x1[3] = pRF1_3x1[3] + 0.5f*(1.f+(float)cosf(PI*(float)RF_LC_count/ftemp1_7x1[0]))*dpRFstop_3x1[3];
						if(RF_LC_count >= (unsigned int)ftemp1_7x1[0])
						{
							printf("\n All landing compensations - RF recovered");
							RF_LC_flag = 0;
							RF_LC_count = 0;
							dpRFstop_3x1[3] = 0;
						}
					}					
					
					if(_DRC_walking_mode == DRC_QUAD_MODE)
					{
						if(LH_LC_flag == 1)
						{
							LH_LC_count++;
							des_pLH_3x1[3] = pLH1_3x1[3] + 0.5f*(1.f+(float)cosf(PI*(float)LH_LC_count/ftemp1_7x1[0]))*dpLHstop_3x1[3];
							if(LH_LC_count >= (unsigned int)ftemp1_7x1[0])
							{
								printf("\n All landing compensations - LH recovered");
								LH_LC_flag = 0;
								LH_LC_count = 0;
								dpLHstop_3x1[3] = 0;
							}
						}						
						if(RH_LC_flag == 1)
						{
							RH_LC_count++;
							des_pRH_3x1[3] = pRH1_3x1[3] + 0.5f*(1.f+(float)cosf(PI*(float)RH_LC_count/ftemp1_7x1[0]))*dpRHstop_3x1[3];
							if(RH_LC_count >= (unsigned int)ftemp1_7x1[0])
							{
								printf("\n All landing compensations - RH recovered");
								RH_LC_flag = 0;
								RH_LC_count = 0;
								dpRHstop_3x1[3] = 0;
							}
						}
					}
					des_fsp_old = DFSP;
					break;
				case LFSP:
					supp_center_x = des_pLF_3x1[1];
					supp_center_y = des_pLF_3x1[2];

					r12 = 0.f;
					r22 = 1.f;
					r11 = r22;
					r21 = -r12;

					ftemp1_7x1[0] = _Q_34x1[1] + _pZMP_3x1[1] - supp_center_x;
					ftemp1_7x1[1] = _Q_34x1[2] + _pZMP_3x1[2] - supp_center_y;
					zmp_s = r11*ftemp1_7x1[0] + r21*ftemp1_7x1[1];
					zmp_f = r12*ftemp1_7x1[0] + r22*ftemp1_7x1[1] - _offline_traj.offsetZMPf[_offline_traj.i];

					ftemp1_7x1[0] = des_pCOM_3x1[1] - supp_center_x;
					ftemp1_7x1[1] = des_pCOM_3x1[2] - supp_center_y;
					des_pCOM_s = r11*ftemp1_7x1[0] + r21*ftemp1_7x1[1];
					des_pCOM_f = r12*ftemp1_7x1[0] + r22*ftemp1_7x1[1];

					damp_f =  -(_Kfb_SSP_F_DRC_1x4[1]*_Xhat_SSP_F_4x1[1]+_Kfb_SSP_F_DRC_1x4[2]*_Xhat_SSP_F_4x1[2]+_Kfb_SSP_F_DRC_1x4[3]*_Xhat_SSP_F_4x1[3]+_Kfb_SSP_F_DRC_1x4[4]*_Xhat_SSP_F_4x1[4]);
					damp_s =  -(_Kfb_SSP_S_DRC_1x4[1]*_Xhat_SSP_S_4x1[1]+_Kfb_SSP_S_DRC_1x4[2]*_Xhat_SSP_S_4x1[2]+_Kfb_SSP_S_DRC_1x4[3]*_Xhat_SSP_S_4x1[3]+_Kfb_SSP_S_DRC_1x4[4]*_Xhat_SSP_S_4x1[4]);

					if(count_walking < N2_TRANSITION)
					{	
						ftemp1_7x1[0] = (float)count_walking/((float)N2_TRANSITION);
						damp_f *= (float)pow(ftemp1_7x1[0], 3.f);
						damp_s *= (float)pow(ftemp1_7x1[0], 3.f);
						count_walking++;
					}


					if(_offline_traj.i-_offline_traj.i_change[j_change][0] < pSharedMemory->kd[22])
						gain_ovr_larm = pSharedMemory->kp[22] + (1.f-pSharedMemory->kp[22])*(float)(_offline_traj.i-_offline_traj.i_change[j_change][0])/pSharedMemory->kd[22];
					else if(_offline_traj.i_change[j_change][1]-_offline_traj.i >= pSharedMemory->kd[22])
						gain_ovr_larm = 1.f;
					else if(_offline_traj.i_change[j_change][1]-_offline_traj.i < pSharedMemory->kd[22])
						gain_ovr_larm = pSharedMemory->kp[22] + (1.f-pSharedMemory->kp[22])*(float)(_offline_traj.i_change[j_change][1]-_offline_traj.i)/pSharedMemory->kd[22];

					if(_offline_traj.i-_offline_traj.i_change[j_change][0] < pSharedMemory->kd[25])
						gain_ovr_lank = pSharedMemory->kp[25] + (1.f-pSharedMemory->kp[25])*(float)(_offline_traj.i-_offline_traj.i_change[j_change][0])/pSharedMemory->kd[25];
					else if(_offline_traj.i_change[j_change][1]-_offline_traj.i >= pSharedMemory->kd[25])
						gain_ovr_lank = 1.f;
					else if(_offline_traj.i_change[j_change][1]-_offline_traj.i < pSharedMemory->kd[25])
						gain_ovr_lank = pSharedMemory->kp[25] + (1.f-pSharedMemory->kp[25])*(float)(_offline_traj.i_change[j_change][1]-_offline_traj.i)/pSharedMemory->kd[25];

					if(_offline_traj.i-_offline_traj.i_change[j_change][0] < pSharedMemory->kd[22])
						gain_ovr_rarm = pSharedMemory->kp[22] + (1.f-pSharedMemory->kp[22])*(float)(_offline_traj.i-_offline_traj.i_change[j_change][0])/pSharedMemory->kd[22];
					else if(_offline_traj.i_change[j_change][1]-_offline_traj.i >= pSharedMemory->kd[22])
						gain_ovr_rarm = 1.f;
					else if(_offline_traj.i_change[j_change][1]-_offline_traj.i < pSharedMemory->kd[22])
						gain_ovr_rarm = pSharedMemory->kp[22] + (1.f-pSharedMemory->kp[22])*(float)(_offline_traj.i_change[j_change][1]-_offline_traj.i)/pSharedMemory->kd[22];

					if(_offline_traj.i-_offline_traj.i_change[j_change][0] < pSharedMemory->kd[25])
						gain_ovr_rank = pSharedMemory->kp[25] + (1.f-pSharedMemory->kp[25])*(float)(_offline_traj.i-_offline_traj.i_change[j_change][0])/pSharedMemory->kd[25];
					else if(_offline_traj.i_change[j_change][1]-_offline_traj.i >= pSharedMemory->kd[25])
						gain_ovr_rank = 1.f;
					else if(_offline_traj.i_change[j_change][1]-_offline_traj.i < pSharedMemory->kd[25])
						gain_ovr_rank = pSharedMemory->kp[25] + (1.f-pSharedMemory->kp[25])*(float)(_offline_traj.i_change[j_change][1]-_offline_traj.i)/pSharedMemory->kd[25];
					
					//gain_ovr_rarm = pSharedMemory->kp[22] + (1.f-pSharedMemory->kp[22])*sinf(PI*(float)(_offline_traj.i-_offline_traj.i_change[j_change][0])/((float)(_offline_traj.i_change[j_change][1]-_offline_traj.i_change[j_change][0])));
					//gain_ovr_larm = pSharedMemory->kp[22] + (1.f-pSharedMemory->kp[22])*sinf(PI*(float)(_offline_traj.i-_offline_traj.i_change[j_change][0])/((float)(_offline_traj.i_change[j_change][1]-_offline_traj.i_change[j_change][0])));
					
					ftemp1_7x1[0] = 2.f*(float)(_offline_traj.i_change[j_change][1]-_offline_traj.i)/((float)(_offline_traj.i_change[j_change][1]-_offline_traj.i_change[j_change][0]));
					if(ftemp1_7x1[0] < 1.f)
					{
						damp_f *= (float)pow(ftemp1_7x1[0], 3.f);
						damp_s *= (float)pow(ftemp1_7x1[0], 3.f);
					}

					if(pSharedMemory->temp_ssp_damp_on == 1)
					{
						des_pCOM_s += damp_s;
						des_pCOM_f += damp_f;
					}
					
					ftemp1_7x1[0] = _Xhat_SSP_F_4x1[1] + DT*(_Ahat_SSP_F_DRC_4x4[1][1]*_Xhat_SSP_F_4x1[1]+_Ahat_SSP_F_DRC_4x4[1][2]*_Xhat_SSP_F_4x1[2] +
						_Ahat_SSP_F_DRC_4x4[1][3]*_Xhat_SSP_F_4x1[3]+_Ahat_SSP_F_DRC_4x4[1][4]*_Xhat_SSP_F_4x1[4]+ _Bhat_SSP_F_DRC_4x1[1]*des_pCOM_f + _Lo_SSP_F_DRC_4x1[1]*zmp_f);
					ftemp1_7x1[1] = _Xhat_SSP_F_4x1[2] + DT*(_Ahat_SSP_F_DRC_4x4[2][1]*_Xhat_SSP_F_4x1[1]+_Ahat_SSP_F_DRC_4x4[2][2]*_Xhat_SSP_F_4x1[2] +
						_Ahat_SSP_F_DRC_4x4[2][3]*_Xhat_SSP_F_4x1[3]+_Ahat_SSP_F_DRC_4x4[2][4]*_Xhat_SSP_F_4x1[4]+ _Bhat_SSP_F_DRC_4x1[2]*des_pCOM_f + _Lo_SSP_F_DRC_4x1[2]*zmp_f);
					ftemp1_7x1[2] = _Xhat_SSP_F_4x1[3] + DT*(_Ahat_SSP_F_DRC_4x4[3][1]*_Xhat_SSP_F_4x1[1]+_Ahat_SSP_F_DRC_4x4[3][2]*_Xhat_SSP_F_4x1[2] +
						_Ahat_SSP_F_DRC_4x4[3][3]*_Xhat_SSP_F_4x1[3]+_Ahat_SSP_F_DRC_4x4[3][4]*_Xhat_SSP_F_4x1[4]+ _Bhat_SSP_F_DRC_4x1[3]*des_pCOM_f + _Lo_SSP_F_DRC_4x1[3]*zmp_f);
					_Xhat_SSP_F_4x1[4] = _Xhat_SSP_F_4x1[4] + DT*(_Ahat_SSP_F_DRC_4x4[4][1]*_Xhat_SSP_F_4x1[1]+_Ahat_SSP_F_DRC_4x4[4][2]*_Xhat_SSP_F_4x1[2] +
						_Ahat_SSP_F_DRC_4x4[4][3]*_Xhat_SSP_F_4x1[3]+_Ahat_SSP_F_DRC_4x4[4][4]*_Xhat_SSP_F_4x1[4]+ _Bhat_SSP_F_DRC_4x1[4]*des_pCOM_f + _Lo_SSP_F_DRC_4x1[4]*zmp_f);
					_Xhat_SSP_F_4x1[1] = ftemp1_7x1[0];
					_Xhat_SSP_F_4x1[2] = ftemp1_7x1[1];
					_Xhat_SSP_F_4x1[3] = ftemp1_7x1[2];

					ftemp1_7x1[0] = _Xhat_SSP_S_4x1[1] + DT*(_Ahat_SSP_S_DRC_4x4[1][1]*_Xhat_SSP_S_4x1[1]+_Ahat_SSP_S_DRC_4x4[1][2]*_Xhat_SSP_S_4x1[2] +
						_Ahat_SSP_S_DRC_4x4[1][3]*_Xhat_SSP_S_4x1[3]+_Ahat_SSP_S_DRC_4x4[1][4]*_Xhat_SSP_S_4x1[4]+ _Bhat_SSP_S_DRC_4x1[1]*des_pCOM_s + _Lo_SSP_S_DRC_4x1[1]*zmp_s);
					ftemp1_7x1[1] = _Xhat_SSP_S_4x1[2] + DT*(_Ahat_SSP_S_DRC_4x4[2][1]*_Xhat_SSP_S_4x1[1]+_Ahat_SSP_S_DRC_4x4[2][2]*_Xhat_SSP_S_4x1[2] +
						_Ahat_SSP_S_DRC_4x4[2][3]*_Xhat_SSP_S_4x1[3]+_Ahat_SSP_S_DRC_4x4[2][4]*_Xhat_SSP_S_4x1[4]+ _Bhat_SSP_S_DRC_4x1[2]*des_pCOM_s + _Lo_SSP_S_DRC_4x1[2]*zmp_s);
					ftemp1_7x1[2] = _Xhat_SSP_S_4x1[3] + DT*(_Ahat_SSP_S_DRC_4x4[3][1]*_Xhat_SSP_S_4x1[1]+_Ahat_SSP_S_DRC_4x4[3][2]*_Xhat_SSP_S_4x1[2] +
						_Ahat_SSP_S_DRC_4x4[3][3]*_Xhat_SSP_S_4x1[3]+_Ahat_SSP_S_DRC_4x4[3][4]*_Xhat_SSP_S_4x1[4]+ _Bhat_SSP_S_DRC_4x1[3]*des_pCOM_s + _Lo_SSP_S_DRC_4x1[3]*zmp_s);
					_Xhat_SSP_S_4x1[4] = _Xhat_SSP_S_4x1[4] + DT*(_Ahat_SSP_S_DRC_4x4[4][1]*_Xhat_SSP_S_4x1[1]+_Ahat_SSP_S_DRC_4x4[4][2]*_Xhat_SSP_S_4x1[2] +
						_Ahat_SSP_S_DRC_4x4[4][3]*_Xhat_SSP_S_4x1[3]+_Ahat_SSP_S_DRC_4x4[4][4]*_Xhat_SSP_S_4x1[4]+ _Bhat_SSP_S_DRC_4x1[4]*des_pCOM_s + _Lo_SSP_S_DRC_4x1[4]*zmp_s);
					_Xhat_SSP_S_4x1[1] = ftemp1_7x1[0];
					_Xhat_SSP_S_4x1[2] = ftemp1_7x1[1];
					_Xhat_SSP_S_4x1[3] = ftemp1_7x1[2];						

					des_pCOM_3x1[1] = r12*des_pCOM_f + r11*des_pCOM_s + supp_center_x;
					des_pCOM_3x1[2] = r22*des_pCOM_f + r21*des_pCOM_s + supp_center_y;

					if(pSharedMemory->temp_foot_LC_on_flag == 1)
					{
						if(LF_LC_flag == 1)
						{
							//des_pLF_3x1[3] = pLF1_3x1[3] + dpLFstop_3x1[3];
							
							if(dpLFstop_3x1[3] > 0.f) // early landing							
							{
								ftemp1_7x1[0] = (float)((int)((_offline_traj.i_change[j_change][1]-_offline_traj.i_change[j_change][0])*9.f/10.f));
								LF_LC_count++;
								des_pLF_3x1[3] = pLF1_3x1[3] + 0.5f*(1.f+(float)cosf(PI*(float)LF_LC_count/ftemp1_7x1[0]))*dpLFstop_3x1[3];
								if(LF_LC_count >= (unsigned int)ftemp1_7x1[0])
								{
									printf("\n Early-Landing compensation - LF recovered ");
									LF_LC_flag = 0;
									LF_LC_count = 0;
									dpLFstop_3x1[3] = 0.f;
								}							
							}
							else if(dpLFstop_3x1[3] < 0.f) // late landing
							{
								// to be implemented
							}
							
						}
						
						if(RF_LC_flag == 1)
						{
							if(des_pRF_3x1[3] < pRF1_3x1[3]+dpRFstop_3x1[3])
								des_pRF_3x1[3] = pRF1_3x1[3] + dpRFstop_3x1[3];
							else
							{
								printf("\n Early-Landing compensation - RF recovered ");
								RF_LC_flag = 0;
								RF_LC_count = 0;
								dpRFstop_3x1[3] = 0.f;
							}
						}
						else if(des_pRF_3x1[3]>pRF1_3x1[3]+_offline_traj.dpRFz[_offline_traj.i+1] && _FSP==DFSP) // early landing of RF 
						{
							dpRFstop_3x1[3] = des_pRF_3x1[3] - pRF1_3x1[3] + pSharedMemory->temp_EL_compen;
							des_pRF_3x1[3] = pRF1_3x1[3] + dpRFstop_3x1[3];
							RF_LC_flag = 1;
							RF_LC_count = 0;
							printf("\n Early-Landing occured - RF %f", dpRFstop_3x1[3]);
						}
					}
					
					if(pSharedMemory->temp_hand_LC_on_flag == 1)
					{
						if(_DRC_walking_mode == DRC_QUAD_MODE)
						{	
							if(RH_LC_flag == 1)
							{
								if(des_pRH_3x1[3] < pRH1_3x1[3]+dpRHstop_3x1[3])
									des_pRH_3x1[3] = pRH1_3x1[3]+dpRHstop_3x1[3];
								else
								{
									printf("\n Early-Landing compensation - RH recovered ");
									RH_LC_flag = 0;
									RH_LC_count = 0;
									dpRHstop_3x1[3] = 0.f;
								}
							}
							/*else if((des_pRH_3x1[3]>pRH1_3x1[3]+_offline_traj.dpRHz[_offline_traj.i+1]) && (_HSP==DHSP || _HSP==RHSP)) // early landing of RH
							{
								dpRHstop_3x1[3] = des_pRH_3x1[3] - pRH1_3x1[3] + pSharedMemory->temp_EL_compen;
								des_pRH_3x1[3] = pRH1_3x1[3]+dpRHstop_3x1[3];
								RH_LC_flag = 1;
								RH_LC_count = 0;
								printf("\n Early-Landing occured - RH %f", dpRHstop_3x1[3]);
							}*/
							
							if(LH_LC_flag == 1)
							{
								if(des_pLH_3x1[3] < pLH1_3x1[3]+dpLHstop_3x1[3])
									des_pLH_3x1[3] = pLH1_3x1[3]+dpLHstop_3x1[3];
								else
								{
									printf("\n Early-Landing compensation - LH recovered ");
									LH_LC_flag = 0;
									LH_LC_count = 0;
									dpLHstop_3x1[3] = 0.f;
								}
							}
							else if((des_pLH_3x1[3] > pLH1_3x1[3]+_offline_traj.dpLHz[_offline_traj.i+1]) && (_HSP==DHSP || _HSP==LHSP)) // early landing of LH
							{
								dpLHstop_3x1[3] = des_pLH_3x1[3] - pLH1_3x1[3] + pSharedMemory->temp_EL_compen;
								des_pLH_3x1[3] = pLH1_3x1[3]+dpLHstop_3x1[3];
								LH_LC_flag = 1;
								LH_LC_count = 0;
								printf("\n Early-Landing occured - LH %f", dpLHstop_3x1[3]);
							}
						}
					}
					des_fsp_old = LFSP;
					break;
				case RFSP:
					supp_center_x = des_pRF_3x1[1];
					supp_center_y = des_pRF_3x1[2];

					r12 = 0.f;
					r22 = 1.f;
					r11 = r22;
					r21 = -r12;

					ftemp1_7x1[0] = _Q_34x1[1] + _pZMP_3x1[1] - supp_center_x;
					ftemp1_7x1[1] = _Q_34x1[2] + _pZMP_3x1[2] - supp_center_y;
					zmp_s = r11*ftemp1_7x1[0] + r21*ftemp1_7x1[1];
					zmp_f = r12*ftemp1_7x1[0] + r22*ftemp1_7x1[1] - _offline_traj.offsetZMPf[_offline_traj.i];

					ftemp1_7x1[0] = des_pCOM_3x1[1] - supp_center_x;
					ftemp1_7x1[1] = des_pCOM_3x1[2] - supp_center_y;
					des_pCOM_s = r11*ftemp1_7x1[0] + r21*ftemp1_7x1[1];
					des_pCOM_f = r12*ftemp1_7x1[0] + r22*ftemp1_7x1[1];

					damp_f =  -(_Kfb_SSP_F_DRC_1x4[1]*_Xhat_SSP_F_4x1[1]+_Kfb_SSP_F_DRC_1x4[2]*_Xhat_SSP_F_4x1[2]+_Kfb_SSP_F_DRC_1x4[3]*_Xhat_SSP_F_4x1[3]+_Kfb_SSP_F_DRC_1x4[4]*_Xhat_SSP_F_4x1[4]);
					damp_s =  -(_Kfb_SSP_S_DRC_1x4[1]*_Xhat_SSP_S_4x1[1]+_Kfb_SSP_S_DRC_1x4[2]*_Xhat_SSP_S_4x1[2]+_Kfb_SSP_S_DRC_1x4[3]*_Xhat_SSP_S_4x1[3]+_Kfb_SSP_S_DRC_1x4[4]*_Xhat_SSP_S_4x1[4]);

					if(count_walking < N2_TRANSITION)
					{	
						ftemp1_7x1[0] = (float)count_walking/((float)N2_TRANSITION);
						damp_f *= (float)pow(ftemp1_7x1[0], 3.f);
						damp_s *= (float)pow(ftemp1_7x1[0], 3.f);
						count_walking++;
					}

					if(_offline_traj.i-_offline_traj.i_change[j_change][0] < pSharedMemory->kd[22])
						gain_ovr_larm = pSharedMemory->kp[22] + (1.f-pSharedMemory->kp[22])*(float)(_offline_traj.i-_offline_traj.i_change[j_change][0])/pSharedMemory->kd[22];
					else if(_offline_traj.i_change[j_change][1]-_offline_traj.i >= pSharedMemory->kd[22])
						gain_ovr_larm = 1.f;
					else if(_offline_traj.i_change[j_change][1]-_offline_traj.i < pSharedMemory->kd[22])
						gain_ovr_larm = pSharedMemory->kp[22] + (1.f-pSharedMemory->kp[22])*(float)(_offline_traj.i_change[j_change][1]-_offline_traj.i)/pSharedMemory->kd[22];

					if(_offline_traj.i-_offline_traj.i_change[j_change][0] < pSharedMemory->kd[25])
						gain_ovr_lank = pSharedMemory->kp[25] + (1.f-pSharedMemory->kp[25])*(float)(_offline_traj.i-_offline_traj.i_change[j_change][0])/pSharedMemory->kd[25];
					else if(_offline_traj.i_change[j_change][1]-_offline_traj.i >= pSharedMemory->kd[25])
						gain_ovr_lank = 1.f;
					else if(_offline_traj.i_change[j_change][1]-_offline_traj.i < pSharedMemory->kd[25])
						gain_ovr_lank = pSharedMemory->kp[25] + (1.f-pSharedMemory->kp[25])*(float)(_offline_traj.i_change[j_change][1]-_offline_traj.i)/pSharedMemory->kd[25];
\
					if(_offline_traj.i-_offline_traj.i_change[j_change][0] < pSharedMemory->kd[22])
						gain_ovr_rarm = pSharedMemory->kp[22] + (1.f-pSharedMemory->kp[22])*(float)(_offline_traj.i-_offline_traj.i_change[j_change][0])/pSharedMemory->kd[22];
					else if(_offline_traj.i_change[j_change][1]-_offline_traj.i >= pSharedMemory->kd[22])
						gain_ovr_rarm = 1.f;
					else if(_offline_traj.i_change[j_change][1]-_offline_traj.i < pSharedMemory->kd[22])
						gain_ovr_rarm = pSharedMemory->kp[22] + (1.f-pSharedMemory->kp[22])*(float)(_offline_traj.i_change[j_change][1]-_offline_traj.i)/pSharedMemory->kd[22];

					if(_offline_traj.i-_offline_traj.i_change[j_change][0] < pSharedMemory->kd[25])
						gain_ovr_rank = pSharedMemory->kp[25] + (1.f-pSharedMemory->kp[25])*(float)(_offline_traj.i-_offline_traj.i_change[j_change][0])/pSharedMemory->kd[25];
					else if(_offline_traj.i_change[j_change][1]-_offline_traj.i >= pSharedMemory->kd[25])
						gain_ovr_rank = 1.f;
					else if(_offline_traj.i_change[j_change][1]-_offline_traj.i < pSharedMemory->kd[25])
						gain_ovr_rank = pSharedMemory->kp[25] + (1.f-pSharedMemory->kp[25])*(float)(_offline_traj.i_change[j_change][1]-_offline_traj.i)/pSharedMemory->kd[25];
					
					//gain_ovr_rarm = pSharedMemory->kp[22] + (1.f-pSharedMemory->kp[22])*sinf(PI*(float)(_offline_traj.i-_offline_traj.i_change[j_change][0])/((float)(_offline_traj.i_change[j_change][1]-_offline_traj.i_change[j_change][0])));
					//gain_ovr_larm = pSharedMemory->kp[22] + (1.f-pSharedMemory->kp[22])*sinf(PI*(float)(_offline_traj.i-_offline_traj.i_change[j_change][0])/((float)(_offline_traj.i_change[j_change][1]-_offline_traj.i_change[j_change][0])));
					
					ftemp1_7x1[0] = 2.f*(float)(_offline_traj.i_change[j_change][1]-_offline_traj.i)/((float)(_offline_traj.i_change[j_change][1]-_offline_traj.i_change[j_change][0]));
					if(ftemp1_7x1[0] < 1.f)
					{
						damp_f *= (float)pow(ftemp1_7x1[0], 3.f);
						damp_s *= (float)pow(ftemp1_7x1[0], 3.f);
					}

					if(pSharedMemory->temp_ssp_damp_on == 1)
					{
						des_pCOM_s += damp_s;
						des_pCOM_f += damp_f;
					}
					
					ftemp1_7x1[0] = _Xhat_SSP_F_4x1[1] + DT*(_Ahat_SSP_F_DRC_4x4[1][1]*_Xhat_SSP_F_4x1[1]+_Ahat_SSP_F_DRC_4x4[1][2]*_Xhat_SSP_F_4x1[2] +
						_Ahat_SSP_F_DRC_4x4[1][3]*_Xhat_SSP_F_4x1[3]+_Ahat_SSP_F_DRC_4x4[1][4]*_Xhat_SSP_F_4x1[4]+ _Bhat_SSP_F_DRC_4x1[1]*des_pCOM_f + _Lo_SSP_F_DRC_4x1[1]*zmp_f);
					ftemp1_7x1[1] = _Xhat_SSP_F_4x1[2] + DT*(_Ahat_SSP_F_DRC_4x4[2][1]*_Xhat_SSP_F_4x1[1]+_Ahat_SSP_F_DRC_4x4[2][2]*_Xhat_SSP_F_4x1[2] +
						_Ahat_SSP_F_DRC_4x4[2][3]*_Xhat_SSP_F_4x1[3]+_Ahat_SSP_F_DRC_4x4[2][4]*_Xhat_SSP_F_4x1[4]+ _Bhat_SSP_F_DRC_4x1[2]*des_pCOM_f + _Lo_SSP_F_DRC_4x1[2]*zmp_f);
					ftemp1_7x1[2] = _Xhat_SSP_F_4x1[3] + DT*(_Ahat_SSP_F_DRC_4x4[3][1]*_Xhat_SSP_F_4x1[1]+_Ahat_SSP_F_DRC_4x4[3][2]*_Xhat_SSP_F_4x1[2] +
						_Ahat_SSP_F_DRC_4x4[3][3]*_Xhat_SSP_F_4x1[3]+_Ahat_SSP_F_DRC_4x4[3][4]*_Xhat_SSP_F_4x1[4]+ _Bhat_SSP_F_DRC_4x1[3]*des_pCOM_f + _Lo_SSP_F_DRC_4x1[3]*zmp_f);
					_Xhat_SSP_F_4x1[4] = _Xhat_SSP_F_4x1[4] + DT*(_Ahat_SSP_F_DRC_4x4[4][1]*_Xhat_SSP_F_4x1[1]+_Ahat_SSP_F_DRC_4x4[4][2]*_Xhat_SSP_F_4x1[2] +
						_Ahat_SSP_F_DRC_4x4[4][3]*_Xhat_SSP_F_4x1[3]+_Ahat_SSP_F_DRC_4x4[4][4]*_Xhat_SSP_F_4x1[4]+ _Bhat_SSP_F_DRC_4x1[4]*des_pCOM_f + _Lo_SSP_F_DRC_4x1[4]*zmp_f);
					_Xhat_SSP_F_4x1[1] = ftemp1_7x1[0];
					_Xhat_SSP_F_4x1[2] = ftemp1_7x1[1];
					_Xhat_SSP_F_4x1[3] = ftemp1_7x1[2];

					ftemp1_7x1[0] = _Xhat_SSP_S_4x1[1] + DT*(_Ahat_SSP_S_DRC_4x4[1][1]*_Xhat_SSP_S_4x1[1]+_Ahat_SSP_S_DRC_4x4[1][2]*_Xhat_SSP_S_4x1[2] +
						_Ahat_SSP_S_DRC_4x4[1][3]*_Xhat_SSP_S_4x1[3]+_Ahat_SSP_S_DRC_4x4[1][4]*_Xhat_SSP_S_4x1[4]+ _Bhat_SSP_S_DRC_4x1[1]*des_pCOM_s + _Lo_SSP_S_DRC_4x1[1]*zmp_s);
					ftemp1_7x1[1] = _Xhat_SSP_S_4x1[2] + DT*(_Ahat_SSP_S_DRC_4x4[2][1]*_Xhat_SSP_S_4x1[1]+_Ahat_SSP_S_DRC_4x4[2][2]*_Xhat_SSP_S_4x1[2] +
						_Ahat_SSP_S_DRC_4x4[2][3]*_Xhat_SSP_S_4x1[3]+_Ahat_SSP_S_DRC_4x4[2][4]*_Xhat_SSP_S_4x1[4]+ _Bhat_SSP_S_DRC_4x1[2]*des_pCOM_s + _Lo_SSP_S_DRC_4x1[2]*zmp_s);
					ftemp1_7x1[2] = _Xhat_SSP_S_4x1[3] + DT*(_Ahat_SSP_S_DRC_4x4[3][1]*_Xhat_SSP_S_4x1[1]+_Ahat_SSP_S_DRC_4x4[3][2]*_Xhat_SSP_S_4x1[2] +
						_Ahat_SSP_S_DRC_4x4[3][3]*_Xhat_SSP_S_4x1[3]+_Ahat_SSP_S_DRC_4x4[3][4]*_Xhat_SSP_S_4x1[4]+ _Bhat_SSP_S_DRC_4x1[3]*des_pCOM_s + _Lo_SSP_S_DRC_4x1[3]*zmp_s);
					_Xhat_SSP_S_4x1[4] = _Xhat_SSP_S_4x1[4] + DT*(_Ahat_SSP_S_DRC_4x4[4][1]*_Xhat_SSP_S_4x1[1]+_Ahat_SSP_S_DRC_4x4[4][2]*_Xhat_SSP_S_4x1[2] +
						_Ahat_SSP_S_DRC_4x4[4][3]*_Xhat_SSP_S_4x1[3]+_Ahat_SSP_S_DRC_4x4[4][4]*_Xhat_SSP_S_4x1[4]+ _Bhat_SSP_S_DRC_4x1[4]*des_pCOM_s + _Lo_SSP_S_DRC_4x1[4]*zmp_s);
					_Xhat_SSP_S_4x1[1] = ftemp1_7x1[0];
					_Xhat_SSP_S_4x1[2] = ftemp1_7x1[1];
					_Xhat_SSP_S_4x1[3] = ftemp1_7x1[2];					
					
					des_pCOM_3x1[1] = r12*des_pCOM_f + r11*des_pCOM_s + supp_center_x;
					des_pCOM_3x1[2] = r22*des_pCOM_f + r21*des_pCOM_s + supp_center_y;
					
					if(pSharedMemory->temp_foot_LC_on_flag == 1)
					{
						if(RF_LC_flag == 1)
						{
							//des_pRF_3x1[3] = dpRFstop_3x1[3];
							
							if(dpRFstop_3x1[3] > 0.f) // early landing							
							{
								ftemp1_7x1[0] = (float)((int)((_offline_traj.i_change[j_change][1]-_offline_traj.i_change[j_change][0])*9.f/10.f));
								RF_LC_count++;
								des_pRF_3x1[3] = pRF1_3x1[3] + 0.5f*(1.f+(float)cosf(PI*(float)RF_LC_count/ftemp1_7x1[0]))*dpRFstop_3x1[3];
								if(RF_LC_count >= (unsigned int)ftemp1_7x1[0])
								{
									printf("\n Early-Landing compensation - RF recovered ");
									RF_LC_flag = 0;
									RF_LC_count = 0;
									dpRFstop_3x1[3] = 0.f;
								}							
							}
							else if(dpRFstop_3x1[3] < 0.f) // late landing
							{
								// to be implemented
							}
							
						}
						
						if(LF_LC_flag == 1)
						{
							if(des_pLF_3x1[3] < pLF1_3x1[3]+dpLFstop_3x1[3])
								des_pLF_3x1[3] = pLF1_3x1[3]+dpLFstop_3x1[3];
							else
							{
								printf("\n Early-Landing compensation - LF recovered ");
								LF_LC_flag = 0;
								LF_LC_count = 0;
								dpLFstop_3x1[3] = 0.f;
							}
						}
						else if(des_pLF_3x1[3]>pLF1_3x1[3]+_offline_traj.dpLFz[_offline_traj.i+1] && _FSP==DFSP) // early landing of LF 
						{
							dpLFstop_3x1[3] = des_pLF_3x1[3] - pLF1_3x1[3] + pSharedMemory->temp_EL_compen;
							des_pLF_3x1[3] = pLF1_3x1[3] + dpLFstop_3x1[3];
							LF_LC_flag = 1;
							LF_LC_count = 0;
							printf("\n Early-Landing occured - LF %f", dpLFstop_3x1[3]);
						}
					}
					
					if(pSharedMemory->temp_hand_LC_on_flag == 1)
					{
						if(_DRC_walking_mode == DRC_QUAD_MODE)
						{
							if(RH_LC_flag == 1)
							{
								if(des_pRH_3x1[3] < pRH1_3x1[3]+dpRHstop_3x1[3])
									des_pRH_3x1[3] = pRH1_3x1[3]+dpRHstop_3x1[3];
								else
								{
									printf("\n Early-Landing compensation - RH recovered ");
									RH_LC_flag = 0;
									RH_LC_count = 0;
									dpRHstop_3x1[3] = 0.f;
								}
							}
							else if((des_pRH_3x1[3]>pRH1_3x1[3]+_offline_traj.dpRHz[_offline_traj.i+1]) && (_HSP==DHSP || _HSP==RHSP)) // early landing of RH
							{
								dpRHstop_3x1[3] = des_pRH_3x1[3] - pRH1_3x1[3] + pSharedMemory->temp_EL_compen;
								des_pRH_3x1[3] = pRH1_3x1[3] + dpRHstop_3x1[3];
								RH_LC_flag = 1;
								RH_LC_count = 0;
								printf("\n Early-Landing occured - RH %f", dpRHstop_3x1[3]);
							}

							if(LH_LC_flag == 1)
							{
								if(des_pLH_3x1[3] < pLH1_3x1[3] + dpLHstop_3x1[3])
									des_pLH_3x1[3] = pLH1_3x1[3] + dpLHstop_3x1[3];
								else
								{
									printf("\n Early-Landing compensation - LH recovered ");
									LH_LC_flag = 0;
									LH_LC_count = 0;
									dpLHstop_3x1[3] = 0.f;
								}
							}
							/*else if((des_pLH_3x1[3] > pLH1_3x1[3]+_offline_traj.dpLHz[_offline_traj.i+1]) && (_HSP==DHSP || _HSP==LHSP)) // early landing of LH
							{
								dpLHstop_3x1[3] = des_pLH_3x1[3] - pLH1_3x1[3] + pSharedMemory->temp_EL_compen;
								des_pLH_3x1[3] = pLH1_3x1[3] + dpLHstop_3x1[3];
								LH_LC_flag = 1;
								LH_LC_count = 0;
								printf("\n Early-Landing occured - LH %f", dpLHstop_3x1[3]);
							}*/
						}
					}
					des_fsp_old = RFSP;
					break;
				}

				if(_offline_traj.i == _offline_traj.i_change[j_change][1]-1)
				{
					count_walking = 0;

					if(j_change < _offline_traj.n_changes-1)
						j_change++;
				}
			}	
			else  //  after the last fsp
			{
				supp_center_x = (des_pLF_3x1[1] + des_pRF_3x1[1])*0.5f;
				supp_center_y = (des_pLF_3x1[2] + des_pRF_3x1[2])*0.5f;

				ftemp1_7x1[0] = (float)sqrt((float)pow(des_pLF_3x1[1]-des_pRF_3x1[1], 2.f) + (float)pow(des_pLF_3x1[2]-des_pRF_3x1[2], 2.f));
				
				r12 = (des_pLF_3x1[1]-des_pRF_3x1[1])/ftemp1_7x1[0];
				r22 = (des_pLF_3x1[2]-des_pRF_3x1[2])/ftemp1_7x1[0];
				r11 = r22;
				r21 = -r12;

				ftemp1_7x1[0] = _Q_34x1[1] + _pZMP_3x1[1] - supp_center_x;
				ftemp1_7x1[1] = _Q_34x1[2] + _pZMP_3x1[2] - supp_center_y;
				zmp_s = r11*ftemp1_7x1[0] + r21*ftemp1_7x1[1];
				zmp_f = r12*ftemp1_7x1[0] + r22*ftemp1_7x1[1];

				ftemp1_7x1[0] = des_pCOM_3x1[1] - supp_center_x;
				ftemp1_7x1[1] = des_pCOM_3x1[2] - supp_center_y;
				des_pCOM_s = r11*ftemp1_7x1[0] + r21*ftemp1_7x1[1];
				des_pCOM_f = r12*ftemp1_7x1[0] + r22*ftemp1_7x1[1];

				damp_f =  -(_Kfb_DSP_F_DRC_1x4[1]*_Xhat_DSP_F_4x1[1]+_Kfb_DSP_F_DRC_1x4[2]*_Xhat_DSP_F_4x1[2]+_Kfb_DSP_F_DRC_1x4[3]*_Xhat_DSP_F_4x1[3]+_Kfb_DSP_F_DRC_1x4[4]*_Xhat_DSP_F_4x1[4]);
				damp_s =  -(_Kfb_DSP_S_DRC_1x4[1]*_Xhat_DSP_S_4x1[1]+_Kfb_DSP_S_DRC_1x4[2]*_Xhat_DSP_S_4x1[2]+_Kfb_DSP_S_DRC_1x4[3]*_Xhat_DSP_S_4x1[3]+_Kfb_DSP_S_DRC_1x4[4]*_Xhat_DSP_S_4x1[4]);

				if(count_walking < N1_TRANSITION)
				{	
					ftemp1_7x1[0] = (float)count_walking/((float)N1_TRANSITION);
					damp_f *= (float)pow(ftemp1_7x1[0],3.f);
					damp_s *= (float)pow(ftemp1_7x1[0],3.f);
					count_walking++;
				}
				if(pSharedMemory->temp_dsp_damp_on_flag == 1)
				{
					des_pCOM_s += damp_s;
					des_pCOM_f += damp_f;
				}
				
				ftemp1_7x1[0] = _Xhat_DSP_F_4x1[1] + DT*(_Ahat_DSP_F_DRC_4x4[1][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[1][2]*_Xhat_DSP_F_4x1[2] +
					_Ahat_DSP_F_DRC_4x4[1][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[1][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[1]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[1]*zmp_f);
				ftemp1_7x1[1] = _Xhat_DSP_F_4x1[2] + DT*(_Ahat_DSP_F_DRC_4x4[2][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[2][2]*_Xhat_DSP_F_4x1[2] +
					_Ahat_DSP_F_DRC_4x4[2][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[2][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[2]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[2]*zmp_f);
				ftemp1_7x1[2] = _Xhat_DSP_F_4x1[3] + DT*(_Ahat_DSP_F_DRC_4x4[3][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[3][2]*_Xhat_DSP_F_4x1[2] +
					_Ahat_DSP_F_DRC_4x4[3][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[3][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[3]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[3]*zmp_f);
				_Xhat_DSP_F_4x1[4] = _Xhat_DSP_F_4x1[4] + DT*(_Ahat_DSP_F_DRC_4x4[4][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[4][2]*_Xhat_DSP_F_4x1[2] +
					_Ahat_DSP_F_DRC_4x4[4][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[4][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[4]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[4]*zmp_f);
				_Xhat_DSP_F_4x1[1] = ftemp1_7x1[0];
				_Xhat_DSP_F_4x1[2] = ftemp1_7x1[1];
				_Xhat_DSP_F_4x1[3] = ftemp1_7x1[2];

				ftemp1_7x1[0] = _Xhat_DSP_S_4x1[1] + DT*(_Ahat_DSP_S_DRC_4x4[1][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[1][2]*_Xhat_DSP_S_4x1[2] +
					_Ahat_DSP_S_DRC_4x4[1][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[1][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[1]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[1]*zmp_s);
				ftemp1_7x1[1] = _Xhat_DSP_S_4x1[2] + DT*(_Ahat_DSP_S_DRC_4x4[2][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[2][2]*_Xhat_DSP_S_4x1[2] +
					_Ahat_DSP_S_DRC_4x4[2][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[2][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[2]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[2]*zmp_s);
				ftemp1_7x1[2] = _Xhat_DSP_S_4x1[3] + DT*(_Ahat_DSP_S_DRC_4x4[3][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[3][2]*_Xhat_DSP_S_4x1[2] +
					_Ahat_DSP_S_DRC_4x4[3][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[3][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[3]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[3]*zmp_s);
				_Xhat_DSP_S_4x1[4] = _Xhat_DSP_S_4x1[4] + DT*(_Ahat_DSP_S_DRC_4x4[4][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[4][2]*_Xhat_DSP_S_4x1[2] +
					_Ahat_DSP_S_DRC_4x4[4][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[4][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[4]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[4]*zmp_s);
				_Xhat_DSP_S_4x1[1] = ftemp1_7x1[0];
				_Xhat_DSP_S_4x1[2] = ftemp1_7x1[1];
				_Xhat_DSP_S_4x1[3] = ftemp1_7x1[2];					

				des_pCOM_3x1[1] = r12*des_pCOM_f + r11*des_pCOM_s + supp_center_x;
				des_pCOM_3x1[2] = r22*des_pCOM_f + r21*des_pCOM_s + supp_center_y;
				
				ftemp1_7x1[0] = 400.f;
				if(LF_LC_flag == 1)
				{
					LF_LC_count++;
					des_pLF_3x1[3] = pLF1_3x1[3] + 0.5f*(1.f+(float)cosf(PI*(float)LF_LC_count/ftemp1_7x1[0]))*dpLFstop_3x1[3];
					if(LF_LC_count >= (unsigned int)ftemp1_7x1[0])
					{
						printf("\n All landing compensations - LF recovered");
						LF_LC_flag = 0;
						LF_LC_count = 0;
						dpLFstop_3x1[3] = 0;
					}
				}						
				if(RF_LC_flag == 1)
				{
					RF_LC_count++;
					des_pRF_3x1[3] = pRF1_3x1[3] + 0.5f*(1.f+(float)cosf(PI*(float)RF_LC_count/ftemp1_7x1[0]))*dpRFstop_3x1[3];
					if(RF_LC_count >= (unsigned int)ftemp1_7x1[0])
					{
						printf("\n All landing compensations - RF recovered");
						RF_LC_flag = 0;
						RF_LC_count = 0;
						dpRFstop_3x1[3] = 0;
					}
				}
				
				if(_DRC_walking_mode == DRC_QUAD_MODE)
				{
					if(LH_LC_flag == 1)
					{
						LH_LC_count++;
						des_pLH_3x1[3] = pLH1_3x1[3] + 0.5f*(1.f+(float)cosf(PI*(float)LH_LC_count/ftemp1_7x1[0]))*dpLHstop_3x1[3];
						if(LH_LC_count == (unsigned int)ftemp1_7x1[0])
						{
							printf("\n All landing compensations - LH recovered");
							LH_LC_flag = 0;
							LH_LC_count = 0;
							dpLHstop_3x1[3] = 0;
						}
					}						
					if(RH_LC_flag == 1)
					{
						RH_LC_count++;
						des_pRH_3x1[3] = pRH1_3x1[3] + 0.5f*(1.f+(float)cosf(PI*(float)RH_LC_count/ftemp1_7x1[0]))*dpRHstop_3x1[3];
						if(RH_LC_count >= (unsigned int)ftemp1_7x1[0])
						{
							printf("\n All landing compensations - RH recovered");
							RH_LC_flag = 0;
							RH_LC_count = 0;
							dpRHstop_3x1[3] = 0;
						}
					}
				}

			}
			
			_offline_traj.i++;
			
			if(_offline_traj.i == _offline_traj.length)
			{
				//_offline_traj.i--;
				qPEL1_4x1[1] = des_qPEL_4x1[1];
				qPEL1_4x1[2] = des_qPEL_4x1[2];
				qPEL1_4x1[3] = des_qPEL_4x1[3];
				qPEL1_4x1[4] = des_qPEL_4x1[4];

				pPELz1 = des_pPELz;

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
				
				pSharedMemory->transform_flag = 3;
				_offline_traj.i = 0;					
				j_change = 0;
			}
		}
		else  // _offline_traj.i >= _offline_traj.length
			return -1;			
	}

	des_WST = 0.f;
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
	diff_vv(des_pRH_3x1,3, _pRWR_3x1, ftemp3_7x1);
	mult_mv((const float**)_jRWR_6x33,6,33, _Qp_33x1, ftemp2_7x1);
	diff_vv(des_vRH_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[14], ftemp3_7x1,3, pSharedMemory->kd[14],ftemp1_7x1, ftemp3_7x1);
	mult_mv((const float**)_jRWRp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xrh_6x1);

	QTdel(des_qRH_4x1, _qRWR_4x1, ftemp3_7x1);	
	diff_vv(des_wRH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(pSharedMemory->kp[14],ftemp3_7x1,3, pSharedMemory->kd[14],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xrh_6x1[3]);
	
	diff_vv(des_pLH_3x1,3, _pLWR_3x1, ftemp3_7x1);  
	mult_mv((const float**)_jLWR_6x33,6,33, _Qp_33x1, ftemp2_7x1);  
	diff_vv(des_vLH_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[14], ftemp3_7x1,3, pSharedMemory->kd[14],ftemp1_7x1, ftemp3_7x1);  
	mult_mv((const float**)_jLWRp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	  
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xlh_6x1);

	QTdel(des_qLH_4x1, _qLWR_4x1, ftemp3_7x1);	
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
	X1_33x1[16] = Xlh_6x1[1];
	X1_33x1[17] = Xlh_6x1[2];
	X1_33x1[18] = Xlh_6x1[3];
	X1_33x1[19] = Xcom_3x1[1];
	X1_33x1[20] = Xcom_3x1[2];	
	X1_33x1[21] = Xpelz;
	X1_33x1[22] = Xpel_3x1[1];
	X1_33x1[23] = Xpel_3x1[2];
	X1_33x1[24] = Xpel_3x1[3];
	dim_primary_task = 24;

	//----------------- redundant tasks
	RVALS(_Q_34x1[WST_34],  _Qp_33x1[WST_33],  pSharedMemory->kd[15]*(-_Qp_33x1[WST_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[WST_34]-_Q_34x1[WST_34]),   WSTpmax, WSTppmax, WSTmin, WSTmax, D2R, &X2_33x1[1], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RSY_34],  _Qp_33x1[RSY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RSY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RSY_34]-_Q_34x1[RSY_34]),   RSYpmax, RSYppmax, RSYmin, RSYmax, D2R, &X2_33x1[2], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSY_34],  _Qp_33x1[LSY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LSY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LSY_34]-_Q_34x1[LSY_34]),   LSYpmax, LSYppmax, LSYmin, LSYmax, D2R, &X2_33x1[3], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	//X2_33x1[16] = Xpel_3x1[1];
	//X2_33x1[17] = Xpel_3x1[2];
	//X2_33x1[18] = Xpel_3x1[3];
	dim_redundant_task = 3;
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

		_jT1_33x33[13][i] = _jRWR_6x33[1][i];
		_jT1_33x33[14][i] = _jRWR_6x33[2][i];
		_jT1_33x33[15][i] = _jRWR_6x33[3][i];
		_jT1_33x33[16][i] = _jLWR_6x33[1][i];			
		_jT1_33x33[17][i] = _jLWR_6x33[2][i];			
		_jT1_33x33[18][i] = _jLWR_6x33[3][i];			

		_jT1_33x33[19][i] = _jCOM_3x33[1][i];
		_jT1_33x33[20][i] = _jCOM_3x33[2][i];

		_jT1_33x33[21][i] = 0.f;
		_jT1_33x33[22][i] = 0.f;
		_jT1_33x33[23][i] = 0.f;
		_jT1_33x33[24][i] = 0.f;   
		
		for(j=1;j<=dim_redundant_task;j++)
			_jT2_33x33[j][i] = 0.f;
	}
	_jT1_33x33[21][3] = 1.f;
    _jT1_33x33[22][4] = 1.f;
    _jT1_33x33[23][5] = 1.f;
    _jT1_33x33[24][6] = 1.f;

	
	for(i=1; i<=21; i++)
	{
		_jT1_33x33[i][4] = 0.f;  // exclude pelvis orientation from the solution
		_jT1_33x33[i][5] = 0.f; 
		_jT1_33x33[i][6] = 0.f; 
	}
	
	for(i=13; i<=dim_primary_task; i++)
	{
		_jT1_33x33[i][WST_33] = 0.f;  // exclude WST from the solution
		_jT1_33x33[i][RWY_33] = 0.f;  // exclude RWY from the solution
		_jT1_33x33[i][LWY_33] = 0.f;  // exclude LWY from the solution
		_jT1_33x33[i][RWP_33] = 0.f;  // exclude RWP from the solution
		_jT1_33x33[i][LWP_33] = 0.f;  // exclude LWP from the solution
		_jT1_33x33[i][RWY2_33] = 0.f;  // exclude LWY2 from the solution
		_jT1_33x33[i][LWY2_33] = 0.f;  // exclude RWY2 from the solution
	}

	for(j=20; j<=33; j++)		//// exclude UB from the COM solution
	{
		_jT1_33x33[19][j] = 0.f;
		_jT1_33x33[20][j] = 0.f;
	}
	_jT1_33x33[19][7] = 0.f;
	_jT1_33x33[20][7] = 0.f;

	_jT2_33x33[1][WST_33] = 1.f;
	_jT2_33x33[2][RSY_33] = 1.f;
	_jT2_33x33[3][LSY_33] = 1.f;
		
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

	do
	{
		isLimited = 0;
		if(pSharedMemory->torque_mode_flag == 0 || pSharedMemory->torque_mode_flag == 3)
		{
			for(i=1;i<=27;i++)
			{
				index_limited[i] = 0;
				qpp_limited[i] = 0.f;
				if(Qpp_33x1[i+6] >= ub_27x1[i])
				{
					isLimited = 1;
					index_limited[i] = 1;
					qpp_limited[i] = ub_27x1[i];
				}
				else if(Qpp_33x1[i+6] <= lb_27x1[i])
				{
					isLimited = 1;
					index_limited[i] = 1;
					qpp_limited[i] = lb_27x1[i];
				}

				if(isLimited==1)
				{
					printf("\n i: %d, joint: %d, p: %.4fdeg, v: %.4fdeg/s, a: %.4fdeg/s^2", _offline_traj.i, i+6, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D, Qpp_33x1[i+6]*R2D);
					printf("\n ub: %.4fdeg/s^2, lb: %.4fdeg/s^2", ub_27x1[i]*R2D, lb_27x1[i]*R2D);	
					for(j=1;j<=33;j++)
						printf("\n%d %f %f %f %f",j,Qpp_33x1[j],_TEMP1_34x34[1][j], X2_33x1[j], _Q_34x1[j]);
					printf("\n       %f", _Q_34x1[34]);
					return -2;
				}

				if(fabs(_Qp_33x1[i+6])>=QP_MAX)
				{
					printf("\n i: %d, joint: %d, p: %.4fdeg, v: %.4fdeg/s, a: %.4fdeg/s^2", _offline_traj.i, i+6, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D, Qpp_33x1[i+6]*R2D);
					printf("\n ub: %.4fdeg/s^2, lb: %.4fdeg/s^2", ub_27x1[i]*R2D, lb_27x1[i]*R2D);			
					return -2;
				}
			}
		}
		
		if(isLimited==1)
		{
			// will be filled
		}
	} while(isLimited==1);
	
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
	//_Q_34x1[RWY_34] += _Qp_33x1[RWY_33]*DT + 0.5f*Qpp_33x1[24]*DT*DT;
	//_Q_34x1[RWP_34] += _Qp_33x1[RWP_33]*DT + 0.5f*Qpp_33x1[25]*DT*DT;
	_Q_34x1[LSP_34] += _Qp_33x1[LSP_33]*DT + 0.5f*Qpp_33x1[26]*DT*DT;
	_Q_34x1[LSR_34] += _Qp_33x1[LSR_33]*DT + 0.5f*Qpp_33x1[27]*DT*DT;
	_Q_34x1[LSY_34] += _Qp_33x1[LSY_33]*DT + 0.5f*Qpp_33x1[28]*DT*DT;
	_Q_34x1[LEB_34] += _Qp_33x1[LEB_33]*DT + 0.5f*Qpp_33x1[29]*DT*DT;
	//_Q_34x1[LWY_34] += _Qp_33x1[LWY_33]*DT + 0.5f*Qpp_33x1[30]*DT*DT;
	//_Q_34x1[LWP_34] += _Qp_33x1[LWP_33]*DT + 0.5f*Qpp_33x1[31]*DT*DT;
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
	//_Qp_33x1[RWY_33] += DT*Qpp_33x1[24];
	//_Qp_33x1[RWP_33] += DT*Qpp_33x1[25];
	_Qp_33x1[LSP_33] += DT*Qpp_33x1[26];
	_Qp_33x1[LSR_33] += DT*Qpp_33x1[27];
	_Qp_33x1[LSY_33] += DT*Qpp_33x1[28];
	_Qp_33x1[LEB_33] += DT*Qpp_33x1[29];
	//_Qp_33x1[LWY_33] += DT*Qpp_33x1[30];
	//_Qp_33x1[LWP_33] += DT*Qpp_33x1[31];
	//_Qp_33x1[RWY2_33] += DT*Qpp_33x1[32];
	//_Qp_33x1[LWY2_33] += DT*Qpp_33x1[33];

	Joint[WST].RefAngleCurrent = _Q_34x1[WST_34]*R2D;

	Joint[RHY].RefAngleCurrent = (_Q_34x1[RHY_34]+rhy_compen)*R2D;
	Joint[RHR].RefAngleCurrent = (_Q_34x1[RHR_34]+rhr_compen)*R2D;
	Joint[RHP].RefAngleCurrent = _Q_34x1[RHP_34]*R2D;
	Joint[RKN].RefAngleCurrent = _Q_34x1[RKN_34]*R2D;
	Joint[RAP].RefAngleCurrent = _Q_34x1[RAP_34]*R2D;
	Joint[RAR].RefAngleCurrent = _Q_34x1[RAR_34]*R2D;
	Joint[LHY].RefAngleCurrent = (_Q_34x1[LHY_34]+lhy_compen)*R2D;
	Joint[LHR].RefAngleCurrent = (_Q_34x1[LHR_34]+lhr_compen)*R2D;
	Joint[LHP].RefAngleCurrent = _Q_34x1[LHP_34]*R2D;
	Joint[LKN].RefAngleCurrent = _Q_34x1[LKN_34]*R2D;
	Joint[LAP].RefAngleCurrent = _Q_34x1[LAP_34]*R2D;
	Joint[LAR].RefAngleCurrent = _Q_34x1[LAR_34]*R2D;

	Joint[RSP].RefAngleCurrent = _Q_34x1[RSP_34]*R2D;
	Joint[RSR].RefAngleCurrent = _Q_34x1[RSR_34]*R2D - OFFSET_RSR;
	Joint[RSY].RefAngleCurrent = _Q_34x1[RSY_34]*R2D;
	Joint[REB].RefAngleCurrent = _Q_34x1[REB_34]*R2D - OFFSET_REB;
	//Joint[RWY].RefAngleCurrent = _Q_34x1[RWY_34]*R2D;
	//Joint[RWP].RefAngleCurrent = _Q_34x1[RWP_34]*R2D;
	//Joint[RWY2].RefAngleCurrent = _Q_34x1[RWY2_34]*R2D;
	Joint[LSP].RefAngleCurrent = _Q_34x1[LSP_34]*R2D;
	Joint[LSR].RefAngleCurrent = _Q_34x1[LSR_34]*R2D - OFFSET_LSR;
	Joint[LSY].RefAngleCurrent = _Q_34x1[LSY_34]*R2D;
	Joint[LEB].RefAngleCurrent = _Q_34x1[LEB_34]*R2D - OFFSET_LEB;	
	//Joint[LWY].RefAngleCurrent = _Q_34x1[LWY_34]*R2D;
	//Joint[LWP].RefAngleCurrent = _Q_34x1[LWP_34]*R2D;
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
	
	diff_mm((const float**)_jRWR_6x33,6,33, (const float**)_jRWR_old2_6x33, _jRWRp_6x33);
	subs_m((const float**)_jRWR_old_6x33,6,33, _jRWR_old2_6x33);
	subs_m((const float**)_jRWR_6x33,6,33, _jRWR_old_6x33);
	mult_sm((const float**)_jRWRp_6x33,6,33, 1.f/(2.f*2.f*DT), _jRWRp_6x33);
	
	diff_mm((const float**)_jLWR_6x33,6,33, (const float**)_jLWR_old2_6x33, _jLWRp_6x33);
	subs_m((const float**)_jLWR_old_6x33,6,33, _jLWR_old2_6x33);
	subs_m((const float**)_jLWR_6x33,6,33, _jLWR_old_6x33);
	mult_sm((const float**)_jLWRp_6x33,6,33, 1.f/(2.f*2.f*DT), _jLWRp_6x33);

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
			pRH2_3x1[i] = _pRWR_3x1[i];
			pLH2_3x1[i] = _pLWR_3x1[i];
			qRH2_4x1[i] = _qRWR_4x1[i];
			qLH2_4x1[i] = _qLWR_4x1[i];
		}
		qRH2_4x1[4] = _qRWR_4x1[4];
		qLH2_4x1[4] = _qLWR_4x1[4];

		pre_gain = 0.f;
		count = 0;
	}
	else if(pSharedMemory->torque_mode_flag == 1 || pSharedMemory->torque_mode_flag == 2)
	{	
		if(pSharedMemory->torque_mode_flag == 1)
		{
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



int getDuty4JointLimit(float Q_34x1[], float duty_joint_limit_33x1[])
{
	float temp;

	if(Q_34x1[RSP_34] > RSPmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(RSPmax-Q_34x1[RSP_34]);
		duty_joint_limit_33x1[RSP_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[RSP_34] < RSPmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-RSPmin+Q_34x1[RSP_34]);
		duty_joint_limit_33x1[RSP_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[RSR_34] > RSRmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(RSRmax-Q_34x1[RSR_34]);
		duty_joint_limit_33x1[RSR_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[RSR_34] < RSRmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-RSRmin+Q_34x1[RSR_34]);
		duty_joint_limit_33x1[RSR_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[RSY_34] > RSYmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(RSYmax-Q_34x1[RSY_34]);
		duty_joint_limit_33x1[RSY_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[RSY_34] < RSYmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-RSYmin+Q_34x1[RSY_34]);
		duty_joint_limit_33x1[RSY_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[REB_34] > REBmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(REBmax-Q_34x1[REB_34]);
		duty_joint_limit_33x1[REB_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[REB_34] < REBmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-REBmin+Q_34x1[REB_34]);
		duty_joint_limit_33x1[REB_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[RWY_34] > RWYmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(RWYmax-Q_34x1[RWY_34]);
		duty_joint_limit_33x1[RWY_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[RWY_34] < RWYmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-RWYmin+Q_34x1[RWY_34]);
		duty_joint_limit_33x1[RWY_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[RWP_34] > RWPmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(RWPmax-Q_34x1[RWP_34]);
		duty_joint_limit_33x1[RWP_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[RWP_34] < RWPmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-RWPmin+Q_34x1[RWP_34]);
		duty_joint_limit_33x1[RWP_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[RWY2_34] > RWY2max-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(RWY2max-Q_34x1[RWY2_34]);
		duty_joint_limit_33x1[RWY2_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[RWY2_34] < RWY2min+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-RWY2min+Q_34x1[RWY2_34]);
		duty_joint_limit_33x1[RWY2_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[LSP_34] > LSPmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(LSPmax-Q_34x1[LSP_34]);
		duty_joint_limit_33x1[LSP_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[LSP_34] < LSPmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-LSPmin+Q_34x1[LSP_34]);
		duty_joint_limit_33x1[LSP_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[LSR_34] > LSRmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(LSRmax-Q_34x1[LSR_34]);
		duty_joint_limit_33x1[LSR_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[LSR_34] < LSRmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-LSRmin+Q_34x1[LSR_34]);
		duty_joint_limit_33x1[LSR_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[LSY_34] > LSYmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(LSYmax-Q_34x1[LSY_34]);
		duty_joint_limit_33x1[LSY_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[LSY_34] < LSYmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-LSYmin+Q_34x1[LSY_34]);
		duty_joint_limit_33x1[LSY_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[LEB_34] > LEBmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(LEBmax-Q_34x1[LEB_34]);
		duty_joint_limit_33x1[LEB_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[LEB_34] < LEBmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-LEBmin+Q_34x1[LEB_34]);
		duty_joint_limit_33x1[LEB_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[LWY_34] > LWYmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(LWYmax-Q_34x1[LWY_34]);
		duty_joint_limit_33x1[LWY_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[LWY_34] < LWYmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-LWYmin+Q_34x1[LWY_34]);
		duty_joint_limit_33x1[LWY_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[LWP_34] > LWPmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(LWPmax-Q_34x1[LWP_34]);
		duty_joint_limit_33x1[LWP_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[LWP_34] < LWPmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-LWPmin+Q_34x1[LWP_34]);
		duty_joint_limit_33x1[LWP_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[LWY2_34] > LWY2max-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(LWY2max-Q_34x1[LWY2_34]);
		duty_joint_limit_33x1[LWY2_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[LWY2_34] < LWY2min+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-LWY2min+Q_34x1[LWY2_34]);
		duty_joint_limit_33x1[LWY2_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[RHY_34] > RHYmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(RHYmax-Q_34x1[RHY_34]);
		duty_joint_limit_33x1[RHY_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[RHY_34] < RHYmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-RHYmin+Q_34x1[RHY_34]);
		duty_joint_limit_33x1[RHY_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[RHR_34] > RHRmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(RHRmax-Q_34x1[RHR_34]);
		duty_joint_limit_33x1[RHR_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[RHR_34] < RHRmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-RHRmin+Q_34x1[RHR_34]);
		duty_joint_limit_33x1[RHR_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[RHP_34] > RHPmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(RHPmax-Q_34x1[RHP_34]);
		duty_joint_limit_33x1[RHP_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[RHP_34] < RHPmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-RHPmin+Q_34x1[RHP_34]);
		duty_joint_limit_33x1[RHP_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[RKN_34] > RKNmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(RKNmax-Q_34x1[RKN_34]);
		duty_joint_limit_33x1[RKN_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[RKN_34] < RKNmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-RKNmin+Q_34x1[RKN_34]);
		duty_joint_limit_33x1[RKN_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[RAP_34] > RAPmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(RAPmax-Q_34x1[RAP_34]);
		duty_joint_limit_33x1[RAP_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[RAP_34] < RAPmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-RAPmin+Q_34x1[RAP_34]);
		duty_joint_limit_33x1[RAP_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[RAR_34] > RARmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(RARmax-Q_34x1[RAR_34]);
		duty_joint_limit_33x1[RAR_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[RAR_34] < RARmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-RARmin+Q_34x1[RAR_34]);
		duty_joint_limit_33x1[RAR_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[LHY_34] > LHYmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(LHYmax-Q_34x1[LHY_34]);
		duty_joint_limit_33x1[LHY_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[LHY_34] < LHYmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-LHYmin+Q_34x1[LHY_34]);
		duty_joint_limit_33x1[LHY_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[LHR_34] > LHRmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(LHRmax-Q_34x1[LHR_34]);
		duty_joint_limit_33x1[LHR_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[LHR_34] < LHRmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-LHRmin+Q_34x1[LHR_34]);
		duty_joint_limit_33x1[LHR_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[LHP_34] > LHPmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(LHPmax-Q_34x1[LHP_34]);
		duty_joint_limit_33x1[LHP_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[LHP_34] < LHPmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-LHPmin+Q_34x1[LHP_34]);
		duty_joint_limit_33x1[LHP_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[LKN_34] > LKNmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(LKNmax-Q_34x1[LKN_34]);
		duty_joint_limit_33x1[LKN_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[LKN_34] < LKNmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-LKNmin+Q_34x1[LKN_34]);
		duty_joint_limit_33x1[LKN_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[LAP_34] > LAPmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(LAPmax-Q_34x1[LAP_34]);
		duty_joint_limit_33x1[LAP_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[LAP_34] < LAPmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-LAPmin+Q_34x1[LAP_34]);
		duty_joint_limit_33x1[LAP_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}

	if(Q_34x1[LAR_34] > LARmax-JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(LARmax-Q_34x1[LAR_34]);
		duty_joint_limit_33x1[LAR_33] = limitDuty(JOINT_LIMIT_DUTY, -JOINT_LIMIT_DUTY*temp*temp);
	}
	else if(Q_34x1[LAR_34] < LARmin+JOINT_LIMIT_RANGE)
	{
		temp =  1.f - 1.f/JOINT_LIMIT_RANGE*(-LARmin+Q_34x1[LAR_34]);
		duty_joint_limit_33x1[LAR_33] = limitDuty(JOINT_LIMIT_DUTY, JOINT_LIMIT_DUTY*temp*temp);
	}


	return 0;
}


int WBIK_DRC_steering_override(unsigned int n)  
{
	int i,j;
	
	float Qpp_33x1[34], meas_Q_34x1[35], meas_Qp_33x1[34];
	float X1_33x1[34], Xrf_6x1[7], Xlf_6x1[7], Xrh_6x1[7], Xlh_6x1[7], Xcom_3x1[4], Xpel_3x1[4], Xwst, Xpelz, X2_33x1[34];  // task variables
	float ub_27x1[28], lb_27x1[28], qpp_limited[28];  // joint upper-bound, lower-bound
	int index_limited[28];
	
	float des_pCOM_3x1[4], des_vCOM_3x1[4];
	float des_pRF_3x1[4], des_pLF_3x1[4], des_vRF_3x1[4], des_vLF_3x1[4];
	float des_qRF_4x1[5], des_qLF_4x1[5], des_wRF_3x1[4], des_wLF_3x1[4];
	float des_pRH_3x1[4], des_pLH_3x1[4], des_vRH_3x1[4], des_vLH_3x1[4];
	float des_qRH_4x1[5], des_qLH_4x1[5], des_wRH_3x1[4], des_wLH_3x1[4];
	float des_WST, des_WSTp, des_qPEL_4x1[5];
	float des_pPELz, des_vPELz;
	float vCOM_3x1[4];

	BOOL isLimited = FALSE;
	int dim_primary_task, dim_redundant_task;

	float lL, lR, fRFz, fLFz;
	float rhr_compen = 0.f;
	float lhr_compen = 0.f;
	float rhy_compen = 0.f;
	float lhy_compen = 0.f;
	

	float fric_compen_33x1[34], ct_33x1[34], gravity_33x1[34], duty_33x1[34], duty_joint_limit_33x1[34];

	static char half_flag, reset_neutral_flag;
	static int count, steer_count, count_walking;
	static float neutral_Q_34x1[35], Q_old_34x1[35], Q_old2_34x1[35];
	static float pCOM1_3x1[4], pPELz1, pRF1_3x1[4], pLF1_3x1[4], qRF1_4x1[5], qLF1_4x1[5];
	static float pRH1_3x1[4], pLH1_3x1[4], qRH1_4x1[5], qLH1_4x1[5];
	static float pRH2_3x1[4], pLH2_3x1[4], qRH2_4x1[5], qLH2_4x1[5];
	static float t, t_steer;
	static float qSteer_errSum, qSteer_old, qSteer_old2, qSteer0, qSteer1;

	float dpRF_3x1[4], dpLF_3x1[4], dqRF_4x1[5], dqLF_4x1[5];
	float dpRH_3x1[4], dpLH_3x1[4], dqRH_4x1[5], dqLH_4x1[5];
	float dpPELz, dpCOM_3x1[4];

	float ftemp1_7x1[8], ftemp2_7x1[8], ftemp3_7x1[8], ftemp4_7x1[8];
	float pre_gain = 0.f;

	float meas_qSteer, meas_wSteer, des_qSteer;

	float zmp_f, zmp_s;
	float supp_center_x, supp_center_y;
	float r11, r12, r21, r22;
	float damp_f = 0.;
	float damp_s = 0.;
	float des_pCOM_s, des_pCOM_f;


	if(_FKineUpdatedFlag != 1)
		return -4;
	
	if(n<=2)
	{
		neutral_Q_34x1[WST_34] = Joint[WST].RefAngleCurrent*D2R;

		neutral_Q_34x1[RSP_34] = Joint[RSP].RefAngleCurrent*D2R;
		neutral_Q_34x1[RSR_34] = (Joint[RSR].RefAngleCurrent + OFFSET_RSR)*D2R;
		neutral_Q_34x1[RSY_34] = Joint[RSY].RefAngleCurrent*D2R;
		neutral_Q_34x1[REB_34] = (Joint[REB].RefAngleCurrent + OFFSET_REB)*D2R;
		neutral_Q_34x1[RWY_34] = Joint[RWY].RefAngleCurrent*D2R;
		neutral_Q_34x1[RWP_34] = Joint[RWP].RefAngleCurrent*D2R;
		neutral_Q_34x1[RWY2_34] = Joint[RWY2].RefAngleCurrent*D2R;
		
		neutral_Q_34x1[LSP_34] = Joint[LSP].RefAngleCurrent*D2R;
		neutral_Q_34x1[LSR_34] = (Joint[LSR].RefAngleCurrent + OFFSET_LSR)*D2R;
		neutral_Q_34x1[LSY_34] = Joint[LSY].RefAngleCurrent*D2R;
		neutral_Q_34x1[LEB_34] = (Joint[LEB].RefAngleCurrent + OFFSET_LEB)*D2R;
		neutral_Q_34x1[LWY_34] = Joint[LWY].RefAngleCurrent*D2R;
		neutral_Q_34x1[LWP_34] = Joint[LWP].RefAngleCurrent*D2R;
		neutral_Q_34x1[LWY2_34] = Joint[LWY2].RefAngleCurrent*D2R;

		_pPEL0_3x1[1] = _Q_34x1[1];
		_pPEL0_3x1[2] = _Q_34x1[2];
		pPELz1 = _pPEL0_3x1[3] = _Q_34x1[3];

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

		pRH1_3x1[1] = _pRH0_3x1[1] = _pRH_3x1[1];
		pRH1_3x1[2] = _pRH0_3x1[2] = _pRH_3x1[2];
		pRH1_3x1[3] = _pRH0_3x1[3] = _pRH_3x1[3];

		qRH1_4x1[1] = _qRH0_4x1[1] = _qRH_4x1[1];
		qRH1_4x1[2] = _qRH0_4x1[2] = _qRH_4x1[2];
		qRH1_4x1[3] = _qRH0_4x1[3] = _qRH_4x1[3];
		qRH1_4x1[4] = _qRH0_4x1[4] = _qRH_4x1[4];

		pLH1_3x1[1] = _pLH0_3x1[1] = _pLH_3x1[1];
		pLH1_3x1[2] = _pLH0_3x1[2] = _pLH_3x1[2];
		pLH1_3x1[3] = _pLH0_3x1[3] = _pLH_3x1[3];

		qLH1_4x1[1] = _qLH0_4x1[1] = _qLH_4x1[1];
		qLH1_4x1[2] = _qLH0_4x1[2] = _qLH_4x1[2];
		qLH1_4x1[3] = _qLH0_4x1[3] = _qLH_4x1[3];
		qLH1_4x1[4] = _qLH0_4x1[4] = _qLH_4x1[4];

		meas_Q_34x1[RSP_34] = ((float)Joint[RSP].EncoderValue)/Joint[RSP].PPR*D2R;
		meas_Q_34x1[RSR_34] = ((float)Joint[RSR].EncoderValue)/Joint[RSR].PPR*D2R + OFFSET_RSR*D2R;
		meas_Q_34x1[RSY_34] = ((float)Joint[RSY].EncoderValue)/Joint[RSY].PPR*D2R;
		meas_Q_34x1[REB_34] = ((float)Joint[REB].EncoderValue)/Joint[REB].PPR*D2R + OFFSET_REB*D2R;
		meas_Q_34x1[RWY_34] = ((float)Joint[RWY].EncoderValue)/Joint[RWY].PPR*D2R;
		meas_Q_34x1[RWP_34] = ((float)Joint[RWP].EncoderValue)/Joint[RWP].PPR*D2R;
		meas_Q_34x1[RWY2_34] = Joint[RWY2].RefAngleCurrent*D2R; //((float)Joint[RWY2].EncoderValue)/Joint[RWY2].PPR*D2R;

		meas_Q_34x1[LSP_34] = ((float)Joint[LSP].EncoderValue)/Joint[LSP].PPR*D2R;
		meas_Q_34x1[LSR_34] = ((float)Joint[LSR].EncoderValue)/Joint[LSR].PPR*D2R + OFFSET_LSR*D2R;
		meas_Q_34x1[LSY_34] = ((float)Joint[LSY].EncoderValue)/Joint[LSY].PPR*D2R;
		meas_Q_34x1[LEB_34] = ((float)Joint[LEB].EncoderValue)/Joint[LEB].PPR*D2R + OFFSET_LEB*D2R;
		meas_Q_34x1[LWY_34] = ((float)Joint[LWY].EncoderValue)/Joint[LWY].PPR*D2R;
		meas_Q_34x1[LWP_34] = ((float)Joint[LWP].EncoderValue)/Joint[LWP].PPR*D2R;
		meas_Q_34x1[LWY2_34] = Joint[LWY2].RefAngleCurrent*D2R; //((float)Joint[LWY2].EncoderValue)/Joint[LWY2].PPR*D2R;

		meas_qSteer = ((float)Joint[NKY].EncoderValue)/(-2883.f)*360.f*D2R;

		Q_old2_34x1[RSP_34] = Q_old_34x1[RSP_34] = meas_Q_34x1[RSP_34];
		Q_old2_34x1[RSR_34] = Q_old_34x1[RSR_34] = meas_Q_34x1[RSR_34];
		Q_old2_34x1[RSY_34] = Q_old_34x1[RSY_34] = meas_Q_34x1[RSY_34];
		Q_old2_34x1[REB_34] = Q_old_34x1[REB_34] = meas_Q_34x1[REB_34];
		Q_old2_34x1[RWY_34] = Q_old_34x1[RWY_34] = meas_Q_34x1[RWY_34];
		Q_old2_34x1[RWP_34] = Q_old_34x1[RWP_34] = meas_Q_34x1[RWP_34];
		Q_old2_34x1[RWY2_34] = Q_old_34x1[RWY2_34] = meas_Q_34x1[RWY2_34];

		Q_old2_34x1[LSP_34] = Q_old_34x1[LSP_34] = meas_Q_34x1[LSP_34];
		Q_old2_34x1[LSR_34] = Q_old_34x1[LSR_34] = meas_Q_34x1[LSR_34];
		Q_old2_34x1[LSY_34] = Q_old_34x1[LSY_34] = meas_Q_34x1[LSY_34];
		Q_old2_34x1[LEB_34] = Q_old_34x1[LEB_34] = meas_Q_34x1[LEB_34];
		Q_old2_34x1[LWY_34] = Q_old_34x1[LWY_34] = meas_Q_34x1[LWY_34];
		Q_old2_34x1[LWP_34] = Q_old_34x1[LWP_34] = meas_Q_34x1[LWP_34];
		Q_old2_34x1[LWY2_34] = Q_old_34x1[LWY2_34] = meas_Q_34x1[LWY2_34];

		qSteer_old2 = qSteer_old = meas_qSteer;

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
		t = 0.f;
		count = 0;
		count_walking = 0;
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
	
	dpPELz = 0.f;

	dpCOM_3x1[1] = 0.f;
	dpCOM_3x1[2] = 0.f;
	dpCOM_3x1[3] = 0.f;
	
	dpRF_3x1[1] = 0.f;
	dpRF_3x1[2] = 0.f;
	dpRF_3x1[3] = 0.f;
	
	dpLF_3x1[1] = 0.f;
	dpLF_3x1[2] = 0.f;
	dpLF_3x1[3] = 0.f;
	
	dpRH_3x1[1] = 0.f;
	dpRH_3x1[2] = 0.f;
	dpRH_3x1[3] = 0.f;
	
	dpLH_3x1[1] = 0.f;
	dpLH_3x1[2] = 0.f;
	dpLH_3x1[3] = 0.f;
	
	dqRF_4x1[1] = 1.f;
	dqRF_4x1[2] = 0.f;
	dqRF_4x1[3] = 0.f;
	dqRF_4x1[4] = 0.f;
	
	dqLF_4x1[1] = 1.f;
	dqLF_4x1[2] = 0.f;
	dqLF_4x1[3] = 0.f;
	dqLF_4x1[4] = 0.f;
	
	dqRH_4x1[1] = 1.f;
	dqRH_4x1[2] = 0.f;
	dqRH_4x1[3] = 0.f;
	dqRH_4x1[4] = 0.f;
	
	dqLH_4x1[1] = 1.f;
	dqLH_4x1[2] = 0.f;
	dqLH_4x1[3] = 0.f;
	dqLH_4x1[4] = 0.f;

	reset_neutral_flag = 0;
	if(pSharedMemory->position_mode_flag == 0) // staying
	{	
		t = 0.f;
	}
	else if(pSharedMemory->position_mode_flag == 1)  // move hands
	{	
		
		dpRH_3x1[1] = one_cos(t, pSharedMemory->dpRH[0], pSharedMemory->move_sec);
		dpRH_3x1[2] = one_cos(t, pSharedMemory->dpRH[1], pSharedMemory->move_sec);
		dpRH_3x1[3] = one_cos(t, pSharedMemory->dpRH[2], pSharedMemory->move_sec);

		dpLH_3x1[1] = one_cos(t, pSharedMemory->dpLH[0], pSharedMemory->move_sec);
		dpLH_3x1[2] = one_cos(t, pSharedMemory->dpLH[1], pSharedMemory->move_sec);
		dpLH_3x1[3] = one_cos(t, pSharedMemory->dpLH[2], pSharedMemory->move_sec);

		ftemp1_7x1[0] = one_cos(t, pSharedMemory->drvRH[0], pSharedMemory->move_sec);
		ftemp1_7x1[1] = pSharedMemory->drvRH[1];
		ftemp1_7x1[2] = pSharedMemory->drvRH[2];
		ftemp1_7x1[3] = pSharedMemory->drvRH[3];
		RV2QT(ftemp1_7x1, dqRH_4x1);

		ftemp1_7x1[0] = one_cos(t, pSharedMemory->drvLH[0], pSharedMemory->move_sec);
		ftemp1_7x1[1] = pSharedMemory->drvLH[1];
		ftemp1_7x1[2] = pSharedMemory->drvLH[2];
		ftemp1_7x1[3] = pSharedMemory->drvLH[3];
		RV2QT(ftemp1_7x1, dqLH_4x1);
		
		t += DT;

		if(t > pSharedMemory->move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 2)  //move RF and LF
	{
		dpRF_3x1[1] = one_cos(t, pSharedMemory->dpRF[0], pSharedMemory->move_sec);
		dpRF_3x1[2] = one_cos(t, pSharedMemory->dpRF[1], pSharedMemory->move_sec);
		dpRF_3x1[3] = one_cos(t, pSharedMemory->dpRF[2], pSharedMemory->move_sec);

		dpLF_3x1[1] = one_cos(t, pSharedMemory->dpLF[0], pSharedMemory->move_sec);
		dpLF_3x1[2] = one_cos(t, pSharedMemory->dpLF[1], pSharedMemory->move_sec);
		dpLF_3x1[3] = one_cos(t, pSharedMemory->dpLF[2], pSharedMemory->move_sec);

		ftemp1_7x1[0] = one_cos(t, pSharedMemory->drvRF[0], pSharedMemory->move_sec);
		ftemp1_7x1[1] = pSharedMemory->drvRF[1];
		ftemp1_7x1[2] = pSharedMemory->drvRF[2];
		ftemp1_7x1[3] = pSharedMemory->drvRF[3];
		RV2QT(ftemp1_7x1, dqRF_4x1);

		ftemp1_7x1[0] = one_cos(t, pSharedMemory->drvLF[0], pSharedMemory->move_sec);
		ftemp1_7x1[1] = pSharedMemory->drvLF[1];
		ftemp1_7x1[2] = pSharedMemory->drvLF[2];
		ftemp1_7x1[3] = pSharedMemory->drvLF[3];
		RV2QT(ftemp1_7x1, dqLF_4x1);

		t += DT;
		if(t > pSharedMemory->move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 3)  //move COM
	{	
		dpCOM_3x1[1] = one_cos(t, pSharedMemory->dpCOM[0], pSharedMemory->move_sec);
		dpCOM_3x1[2] = one_cos(t, pSharedMemory->dpCOM[1], pSharedMemory->move_sec);
	
		t += DT;
		if(t > pSharedMemory->move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 4)  //move pPELz
	{
		dpPELz = one_cos(t, pSharedMemory->dpPELz, pSharedMemory->move_sec);
	
		t += DT;
		if(t > pSharedMemory->move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	else if(pSharedMemory->position_mode_flag == 5)  //offline traj
	{
		dpPELz = pSharedMemory->off_traj_dpPELz[pSharedMemory->off_traj_count];
		dpCOM_3x1[1] = pSharedMemory->off_traj_dpCOM[pSharedMemory->off_traj_count][0];
		dpCOM_3x1[2] = pSharedMemory->off_traj_dpCOM[pSharedMemory->off_traj_count][1];
		dpRH_3x1[1] = pSharedMemory->off_traj_dpRH[pSharedMemory->off_traj_count][0];
		dpRH_3x1[2] = pSharedMemory->off_traj_dpRH[pSharedMemory->off_traj_count][1];
		dpRH_3x1[3] = pSharedMemory->off_traj_dpRH[pSharedMemory->off_traj_count][2];
		dpLH_3x1[1] = pSharedMemory->off_traj_dpLH[pSharedMemory->off_traj_count][0];
		dpLH_3x1[2] = pSharedMemory->off_traj_dpLH[pSharedMemory->off_traj_count][1];
		dpLH_3x1[3] = pSharedMemory->off_traj_dpLH[pSharedMemory->off_traj_count][2];
		dpRF_3x1[1] = pSharedMemory->off_traj_dpRF[pSharedMemory->off_traj_count][0];
		dpRF_3x1[2] = pSharedMemory->off_traj_dpRF[pSharedMemory->off_traj_count][1];
		dpRF_3x1[3] = pSharedMemory->off_traj_dpRF[pSharedMemory->off_traj_count][2];
		dpLF_3x1[1] = pSharedMemory->off_traj_dpLF[pSharedMemory->off_traj_count][0];
		dpLF_3x1[2] = pSharedMemory->off_traj_dpLF[pSharedMemory->off_traj_count][1];
		dpLF_3x1[3] = pSharedMemory->off_traj_dpLF[pSharedMemory->off_traj_count][2];
		RV2QT(pSharedMemory->off_traj_drvRH[pSharedMemory->off_traj_count], dqRH_4x1);
		RV2QT(pSharedMemory->off_traj_drvLH[pSharedMemory->off_traj_count], dqLH_4x1);
		RV2QT(pSharedMemory->off_traj_drvRF[pSharedMemory->off_traj_count], dqRF_4x1);
		RV2QT(pSharedMemory->off_traj_drvLF[pSharedMemory->off_traj_count], dqLF_4x1);
		pSharedMemory->off_traj_count++;
		if(pSharedMemory->off_traj_count >= pSharedMemory->off_traj_length)
		{
			reset_neutral_flag = 1;
			pSharedMemory->off_traj_count = 0;
			pSharedMemory->position_mode_flag = 0;
		}
	}
	
	des_WST = 0.f;
	des_WSTp = 0.f;
	des_qPEL_4x1[1] = 1.f;
	des_qPEL_4x1[2] = 0.f;
	des_qPEL_4x1[3] = 0.f;
	des_qPEL_4x1[4] = 0.f;

	des_pPELz = pPELz1 + dpPELz;

	des_pCOM_3x1[1] = pCOM1_3x1[1] + dpCOM_3x1[1];
	des_pCOM_3x1[2] = pCOM1_3x1[2] + dpCOM_3x1[2];
	des_pCOM_3x1[3] = pCOM1_3x1[2] + dpCOM_3x1[3];

	des_pRF_3x1[1] = pRF1_3x1[1] + dpRF_3x1[1];
	des_pRF_3x1[2] = pRF1_3x1[2] + dpRF_3x1[2];
	des_pRF_3x1[3] = pRF1_3x1[3] + dpRF_3x1[3];
	
	des_pLF_3x1[1] = pLF1_3x1[1] + dpLF_3x1[1];
	des_pLF_3x1[2] = pLF1_3x1[2] + dpLF_3x1[2];
	des_pLF_3x1[3] = pLF1_3x1[3] + dpLF_3x1[3];
	
	des_pRH_3x1[1] = pRH1_3x1[1] + dpRH_3x1[1];
	des_pRH_3x1[2] = pRH1_3x1[2] + dpRH_3x1[2];
	des_pRH_3x1[3] = pRH1_3x1[3] + dpRH_3x1[3];

	des_pLH_3x1[1] = pLH1_3x1[1] + dpLH_3x1[1];
	des_pLH_3x1[2] = pLH1_3x1[2] + dpLH_3x1[2];
	des_pLH_3x1[3] = pLH1_3x1[3] + dpLH_3x1[3];

	QTcross(dqRH_4x1, qRH1_4x1, des_qRH_4x1);
	QTcross(dqLH_4x1, qLH1_4x1, des_qLH_4x1);
	QTcross(dqRF_4x1, qRF1_4x1, des_qRF_4x1);
	QTcross(dqLF_4x1, qLF1_4x1, des_qLF_4x1);
		

	if(reset_neutral_flag==1)
	{
		pPELz1 = des_pPELz;

		pCOM1_3x1[1] = des_pCOM_3x1[1];
		pCOM1_3x1[2] = des_pCOM_3x1[2];

		pRF1_3x1[1] = des_pRF_3x1[1];
		pRF1_3x1[2] = des_pRF_3x1[2];
		pRF1_3x1[3] = des_pRF_3x1[3];

		pLF1_3x1[1] = des_pLF_3x1[1];
		pLF1_3x1[2] = des_pLF_3x1[2];
		pLF1_3x1[3] = des_pLF_3x1[3];

		pRH1_3x1[1] = des_pRH_3x1[1];
		pRH1_3x1[2] = des_pRH_3x1[2];
		pRH1_3x1[3] = des_pRH_3x1[3];

		pLH1_3x1[1] = des_pLH_3x1[1];
		pLH1_3x1[2] = des_pLH_3x1[2];
		pLH1_3x1[3] = des_pLH_3x1[3];

		qRF1_4x1[1] = des_qRF_4x1[1];
		qRF1_4x1[2] = des_qRF_4x1[2];
		qRF1_4x1[3] = des_qRF_4x1[3];
		qRF1_4x1[4] = des_qRF_4x1[4];

		qLF1_4x1[1] = des_qLF_4x1[1];
		qLF1_4x1[2] = des_qLF_4x1[2];
		qLF1_4x1[3] = des_qLF_4x1[3];
		qLF1_4x1[4] = des_qLF_4x1[4];

		qRH1_4x1[1] = des_qRH_4x1[1];
		qRH1_4x1[2] = des_qRH_4x1[2];
		qRH1_4x1[3] = des_qRH_4x1[3];
		qRH1_4x1[4] = des_qRH_4x1[4];

		qLH1_4x1[1] = des_qLH_4x1[1];
		qLH1_4x1[2] = des_qLH_4x1[2];
		qLH1_4x1[3] = des_qLH_4x1[3];
		qLH1_4x1[4] = des_qLH_4x1[4];

		printf("\n move done");
		reset_neutral_flag = 0;
	}

	//------------------ damping controller
	supp_center_x = (des_pLF_3x1[1] + des_pRF_3x1[1])*0.5f;
	supp_center_y = (des_pLF_3x1[2] + des_pRF_3x1[2])*0.5f;

	ftemp1_7x1[0] = (float)sqrt((float)pow(des_pLF_3x1[1]-des_pRF_3x1[1], 2.f) + (float)pow(des_pLF_3x1[2]-des_pRF_3x1[2], 2.f));
	
	r12 = (des_pLF_3x1[1]-des_pRF_3x1[1])/ftemp1_7x1[0];
	r22 = (des_pLF_3x1[2]-des_pRF_3x1[2])/ftemp1_7x1[0];
	r11 = r22;
	r21 = -r12;

	ftemp1_7x1[0] = _Q_34x1[1] + _pZMP_3x1[1] - supp_center_x;
	ftemp1_7x1[1] = _Q_34x1[2] + _pZMP_3x1[2] - supp_center_y;
	zmp_s = r11*ftemp1_7x1[0] + r21*ftemp1_7x1[1];
	zmp_f = r12*ftemp1_7x1[0] + r22*ftemp1_7x1[1];

	ftemp1_7x1[0] = des_pCOM_3x1[1] - supp_center_x;
	ftemp1_7x1[1] = des_pCOM_3x1[2] - supp_center_y;
	des_pCOM_s = r11*ftemp1_7x1[0] + r21*ftemp1_7x1[1];
	des_pCOM_f = r12*ftemp1_7x1[0] + r22*ftemp1_7x1[1];

	damp_f =  -(_Kfb_DSP_F_DRC_1x4[1]*_Xhat_DSP_F_4x1[1]+_Kfb_DSP_F_DRC_1x4[2]*_Xhat_DSP_F_4x1[2]+_Kfb_DSP_F_DRC_1x4[3]*_Xhat_DSP_F_4x1[3]+_Kfb_DSP_F_DRC_1x4[4]*_Xhat_DSP_F_4x1[4]);
	damp_s =  -(_Kfb_DSP_S_DRC_1x4[1]*_Xhat_DSP_S_4x1[1]+_Kfb_DSP_S_DRC_1x4[2]*_Xhat_DSP_S_4x1[2]+_Kfb_DSP_S_DRC_1x4[3]*_Xhat_DSP_S_4x1[3]+_Kfb_DSP_S_DRC_1x4[4]*_Xhat_DSP_S_4x1[4]);
	
	if(count_walking < 400)
	{	
		ftemp1_7x1[0] = (float)count_walking/((float)400.f);
		damp_f *= (float)pow(ftemp1_7x1[0], 3.f);
		damp_s *= (float)pow(ftemp1_7x1[0], 3.f);
		count_walking++;
	}

	des_pCOM_s += damp_s;
	des_pCOM_f += damp_f;
	
	ftemp1_7x1[0] = _Xhat_DSP_F_4x1[1] + DT*(_Ahat_DSP_F_DRC_4x4[1][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[1][2]*_Xhat_DSP_F_4x1[2] +
		_Ahat_DSP_F_DRC_4x4[1][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[1][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[1]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[1]*zmp_f);
	ftemp1_7x1[1] = _Xhat_DSP_F_4x1[2] + DT*(_Ahat_DSP_F_DRC_4x4[2][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[2][2]*_Xhat_DSP_F_4x1[2] +
		_Ahat_DSP_F_DRC_4x4[2][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[2][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[2]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[2]*zmp_f);
	ftemp1_7x1[2] = _Xhat_DSP_F_4x1[3] + DT*(_Ahat_DSP_F_DRC_4x4[3][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[3][2]*_Xhat_DSP_F_4x1[2] +
		_Ahat_DSP_F_DRC_4x4[3][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[3][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[3]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[3]*zmp_f);
	_Xhat_DSP_F_4x1[4] = _Xhat_DSP_F_4x1[4] + DT*(_Ahat_DSP_F_DRC_4x4[4][1]*_Xhat_DSP_F_4x1[1]+_Ahat_DSP_F_DRC_4x4[4][2]*_Xhat_DSP_F_4x1[2] +
		_Ahat_DSP_F_DRC_4x4[4][3]*_Xhat_DSP_F_4x1[3]+_Ahat_DSP_F_DRC_4x4[4][4]*_Xhat_DSP_F_4x1[4]+ _Bhat_DSP_F_DRC_4x1[4]*des_pCOM_f + _Lo_DSP_F_DRC_4x1[4]*zmp_f);
	_Xhat_DSP_F_4x1[1] = ftemp1_7x1[0];
	_Xhat_DSP_F_4x1[2] = ftemp1_7x1[1];
	_Xhat_DSP_F_4x1[3] = ftemp1_7x1[2];

	ftemp1_7x1[0] = _Xhat_DSP_S_4x1[1] + DT*(_Ahat_DSP_S_DRC_4x4[1][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[1][2]*_Xhat_DSP_S_4x1[2] +
		_Ahat_DSP_S_DRC_4x4[1][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[1][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[1]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[1]*zmp_s);
	ftemp1_7x1[1] = _Xhat_DSP_S_4x1[2] + DT*(_Ahat_DSP_S_DRC_4x4[2][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[2][2]*_Xhat_DSP_S_4x1[2] +
		_Ahat_DSP_S_DRC_4x4[2][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[2][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[2]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[2]*zmp_s);
	ftemp1_7x1[2] = _Xhat_DSP_S_4x1[3] + DT*(_Ahat_DSP_S_DRC_4x4[3][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[3][2]*_Xhat_DSP_S_4x1[2] +
		_Ahat_DSP_S_DRC_4x4[3][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[3][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[3]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[3]*zmp_s);
	_Xhat_DSP_S_4x1[4] = _Xhat_DSP_S_4x1[4] + DT*(_Ahat_DSP_S_DRC_4x4[4][1]*_Xhat_DSP_S_4x1[1]+_Ahat_DSP_S_DRC_4x4[4][2]*_Xhat_DSP_S_4x1[2] +
		_Ahat_DSP_S_DRC_4x4[4][3]*_Xhat_DSP_S_4x1[3]+_Ahat_DSP_S_DRC_4x4[4][4]*_Xhat_DSP_S_4x1[4]+ _Bhat_DSP_S_DRC_4x1[4]*des_pCOM_s + _Lo_DSP_S_DRC_4x1[4]*zmp_s);
	_Xhat_DSP_S_4x1[1] = ftemp1_7x1[0];
	_Xhat_DSP_S_4x1[2] = ftemp1_7x1[1];
	_Xhat_DSP_S_4x1[3] = ftemp1_7x1[2];						

	if(pSharedMemory->temp_dsp_damp_on_flag == 1)
	{
		des_pCOM_3x1[1] = r12*des_pCOM_f + r11*des_pCOM_s + supp_center_x;
		des_pCOM_3x1[2] = r22*des_pCOM_f + r21*des_pCOM_s + supp_center_y;
	}
	//-------------------------------------

	des_vPELz = 0.f;

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
	diff_vv(des_pRH_3x1,3, _pRH_3x1, ftemp3_7x1);
	mult_mv((const float**)_jRH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
	diff_vv(des_vRH_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[14], ftemp3_7x1,3, pSharedMemory->kd[14],ftemp1_7x1, ftemp3_7x1);
	mult_mv((const float**)_jRHp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xrh_6x1);

	QTdel(des_qRH_4x1, _qRH_4x1, ftemp3_7x1);	
	diff_vv(des_wRH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(pSharedMemory->kp[14],ftemp3_7x1,3, pSharedMemory->kd[14],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xrh_6x1[3]);
	
	diff_vv(des_pLH_3x1,3, _pLH_3x1, ftemp3_7x1);  
	mult_mv((const float**)_jLH_6x33,6,33, _Qp_33x1, ftemp2_7x1);  
	diff_vv(des_vLH_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(pSharedMemory->kp[14], ftemp3_7x1,3, pSharedMemory->kd[14],ftemp1_7x1, ftemp3_7x1);  
	mult_mv((const float**)_jLHp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	  
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xlh_6x1);

	QTdel(des_qLH_4x1, _qLH_4x1, ftemp3_7x1);	
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
	//X1_33x1[28] = Xpel_3x1[1];
	//X1_33x1[29] = Xpel_3x1[2];
	//X1_33x1[30] = Xpel_3x1[3];
	dim_primary_task = 27;

	//----------------- redundant tasks
	RVALS(_Q_34x1[WST_34],  _Qp_33x1[WST_33],  pSharedMemory->kd[15]*(-_Qp_33x1[WST_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[WST_34]-_Q_34x1[WST_34]),		WSTpmax, WSTppmax, WSTmin, WSTmax, D2R, &X2_33x1[1], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RSP_34],  _Qp_33x1[RSP_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RSP_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RSP_34]-_Q_34x1[RSP_34]),   RSPpmax, RSPppmax, RSPmin, RSPmax, D2R, &X2_33x1[2], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RSR_34],  _Qp_33x1[RSR_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RSR_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RSR_34]-_Q_34x1[RSR_34]),   RSRpmax, RSRppmax, RSRmin, RSRmax, D2R, &X2_33x1[3], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RSY_34],  _Qp_33x1[RSY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RSY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RSY_34]-_Q_34x1[RSY_34]),   RSYpmax, RSYppmax, RSYmin, RSYmax, D2R, &X2_33x1[4], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[REB_34],  _Qp_33x1[REB_33],  pSharedMemory->kd[15]*(-_Qp_33x1[REB_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[REB_34]-_Q_34x1[REB_34]),   REBpmax, REBppmax, REBmin, REBmax, D2R, &X2_33x1[5], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RWY_34],  _Qp_33x1[RWY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RWY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RWY_34]-_Q_34x1[RWY_34]),   RWYpmax, RWYppmax, RWYmin, RWYmax, D2R, &X2_33x1[6], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RWP_34],  _Qp_33x1[RWP_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RWP_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RWP_34]-_Q_34x1[RWP_34]),   RWPpmax, RWPppmax, RWPmin, RWPmax, D2R, &X2_33x1[7], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSP_34],  _Qp_33x1[LSP_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LSP_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LSP_34]-_Q_34x1[LSP_34]),   LSPpmax, LSPppmax, LSPmin, LSPmax, D2R, &X2_33x1[8], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSR_34],  _Qp_33x1[LSR_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LSR_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LSR_34]-_Q_34x1[LSR_34]),   LSRpmax, LSRppmax, LSRmin, LSRmax, D2R, &X2_33x1[9], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSY_34],  _Qp_33x1[LSY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LSY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LSY_34]-_Q_34x1[LSY_34]),   LSYpmax, LSYppmax, LSYmin, LSYmax, D2R, &X2_33x1[10], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LEB_34],  _Qp_33x1[LEB_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LEB_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LEB_34]-_Q_34x1[LEB_34]),   LEBpmax, LEBppmax, LEBmin, LEBmax, D2R, &X2_33x1[11], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LWY_34],  _Qp_33x1[LWY_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LWY_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LWY_34]-_Q_34x1[LWY_34]),   LWYpmax, LWYppmax, LWYmin, LWYmax, D2R, &X2_33x1[12], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LWP_34],  _Qp_33x1[LWP_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LWP_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LWP_34]-_Q_34x1[LWP_34]),   LWPpmax, LWPppmax, LWPmin, LWPmax, D2R, &X2_33x1[13], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RWY2_34],  _Qp_33x1[RWY2_33],  pSharedMemory->kd[15]*(-_Qp_33x1[RWY2_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[RWY2_34]-_Q_34x1[RWY2_34]),   RWY2pmax, RWY2ppmax, RWY2min, RWY2max, D2R, &X2_33x1[14], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LWY2_34],  _Qp_33x1[LWY2_33],  pSharedMemory->kd[15]*(-_Qp_33x1[LWY2_33])+pSharedMemory->kp[15]*(neutral_Q_34x1[LWY2_34]-_Q_34x1[LWY2_34]),   LWY2pmax, LWY2ppmax, LWY2min, LWY2max, D2R, &X2_33x1[15], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	X2_33x1[16] = Xpel_3x1[1];
	X2_33x1[17] = Xpel_3x1[2];
	X2_33x1[18] = Xpel_3x1[3];
	dim_redundant_task = 18;
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
		//_jT1_33x33[28][i] = 0.f;
		//_jT1_33x33[29][i] = 0.f;
		//_jT1_33x33[30][i] = 0.f;   
		for(j=1;j<=18;j++)
			_jT2_33x33[j][i] = 0.f;
	}
	_jT1_33x33[27][3] = 1.f;
  //_jT1_33x33[28][4] = 1.f;
  //_jT1_33x33[29][5] = 1.f;
  //_jT1_33x33[30][6] = 1.f;

	for(i=1; i<=27; i++)
	{
		_jT1_33x33[i][4] = 0.f;  // exclude pelvis orientation from the solution
		_jT1_33x33[i][5] = 0.f; 
		_jT1_33x33[i][6] = 0.f; 
	}

	for(i=13; i<=24; i++)
	{
		_jT1_33x33[i][7] = 0.f;  // exclude WST from the solution
	}

	for(j=20; j<=33; j++)
	{
		_jT1_33x33[25][j] = 0.f;
		_jT1_33x33[26][j] = 0.f;
	}
	_jT1_33x33[25][7] = 0.f;
	_jT1_33x33[26][7] = 0.f;

	_jT2_33x33[1][WST_33] = 1.f;
	_jT2_33x33[2][RSP_33] = 1.f;
	_jT2_33x33[3][RSR_33] = 1.f;
	_jT2_33x33[4][RSY_33] = 1.f;
	_jT2_33x33[5][REB_33] = 1.f;
	_jT2_33x33[6][RWY_33] = 1.f;
	_jT2_33x33[7][RWP_33] = 1.f;
	_jT2_33x33[8][LSP_33] = 1.f;
	_jT2_33x33[9][LSR_33] = 1.f;	
	_jT2_33x33[10][LSY_33] = 1.f;
	_jT2_33x33[11][LEB_33] = 1.f;
	_jT2_33x33[12][LWY_33] = 1.f;
	_jT2_33x33[13][LWP_33] = 1.f;
	_jT2_33x33[14][RWY2_33] = 1.f;
	_jT2_33x33[15][LWY2_33] = 1.f;	
	_jT2_33x33[16][WX_33] = 1.f;
	_jT2_33x33[17][WY_33] = 1.f;
	_jT2_33x33[18][WZ_33] = 1.f;
	
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

	//pinv_svd((const float**)_jT2_33x33,18,33, _jT2inv_33x33);
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

	do
	{
		isLimited = 0;
		if(pSharedMemory->torque_mode_flag == 0 || pSharedMemory->torque_mode_flag == 3)
			for(i=1;i<=27;i++)
			{
				index_limited[i] = 0;
				qpp_limited[i] = 0.f;
				if(Qpp_33x1[i+6] >= ub_27x1[i])
				{
					isLimited = 1;
					index_limited[i] = 1;
					qpp_limited[i] = ub_27x1[i];
				}
				else if(Qpp_33x1[i+6] <= lb_27x1[i])
				{
					isLimited = 1;
					index_limited[i] = 1;
					qpp_limited[i] = lb_27x1[i];
				}

				if(isLimited==1)
				{
					printf("\n i: %d, joint: %d, p: %.4fdeg, v: %.4fdeg/s, a: %.4fdeg/s^2", _offline_traj.i, i+6, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D, Qpp_33x1[i+6]*R2D);
					printf("\n ub: %.4fdeg/s^2, lb: %.4fdeg/s^2", ub_27x1[i]*R2D, lb_27x1[i]*R2D);			
					return -2;
				}

				if(fabs(_Qp_33x1[i+6])>=QP_MAX)
				{
					printf("\n i: %d, joint: %d, p: %.4fdeg, v: %.4fdeg/s, a: %.4fdeg/s^2", _offline_traj.i, i+6, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D, Qpp_33x1[i+6]*R2D);
					printf("\n ub: %.4fdeg/s^2, lb: %.4fdeg/s^2", ub_27x1[i]*R2D, lb_27x1[i]*R2D);			
					return -2;
				}
			}
		
		if(isLimited==1)
		{
			//will be filled
		}
	} while(isLimited==1);
	
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
	_Q_34x1[RWY2_34] += _Qp_33x1[RWY2_33]*DT + 0.5f*Qpp_33x1[32]*DT*DT;
	_Q_34x1[LWY2_34] += _Qp_33x1[LWY2_33]*DT + 0.5f*Qpp_33x1[33]*DT*DT;
	
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
	_Qp_33x1[RWY2_33] += DT*Qpp_33x1[32];
	_Qp_33x1[LWY2_33] += DT*Qpp_33x1[33];

	Joint[WST].RefAngleCurrent = _Q_34x1[WST_34]*R2D;

	Joint[RHY].RefAngleCurrent = (_Q_34x1[RHY_34]+rhy_compen)*R2D;
	Joint[RHR].RefAngleCurrent = (_Q_34x1[RHR_34]+rhr_compen)*R2D;
	Joint[RHP].RefAngleCurrent = _Q_34x1[RHP_34]*R2D;
	Joint[RKN].RefAngleCurrent = _Q_34x1[RKN_34]*R2D;
	Joint[RAP].RefAngleCurrent = _Q_34x1[RAP_34]*R2D;
	Joint[RAR].RefAngleCurrent = _Q_34x1[RAR_34]*R2D;
	Joint[LHY].RefAngleCurrent = (_Q_34x1[LHY_34]+lhy_compen)*R2D;
	Joint[LHR].RefAngleCurrent = (_Q_34x1[LHR_34]+lhr_compen)*R2D;
	Joint[LHP].RefAngleCurrent = _Q_34x1[LHP_34]*R2D;
	Joint[LKN].RefAngleCurrent = _Q_34x1[LKN_34]*R2D;
	Joint[LAP].RefAngleCurrent = _Q_34x1[LAP_34]*R2D;
	Joint[LAR].RefAngleCurrent = _Q_34x1[LAR_34]*R2D;

	Joint[RSP].RefAngleCurrent = _Q_34x1[RSP_34]*R2D;
	Joint[RSR].RefAngleCurrent = _Q_34x1[RSR_34]*R2D - OFFSET_RSR;
	Joint[RSY].RefAngleCurrent = _Q_34x1[RSY_34]*R2D;
	Joint[REB].RefAngleCurrent = _Q_34x1[REB_34]*R2D - OFFSET_REB;
	Joint[RWY].RefAngleCurrent = _Q_34x1[RWY_34]*R2D;
	Joint[RWP].RefAngleCurrent = _Q_34x1[RWP_34]*R2D;
	Joint[RWY2].RefAngleCurrent = _Q_34x1[RWY2_34]*R2D;
	Joint[LSP].RefAngleCurrent = _Q_34x1[LSP_34]*R2D;
	Joint[LSR].RefAngleCurrent = _Q_34x1[LSR_34]*R2D - OFFSET_LSR;
	Joint[LSY].RefAngleCurrent = _Q_34x1[LSY_34]*R2D;
	Joint[LEB].RefAngleCurrent = _Q_34x1[LEB_34]*R2D - OFFSET_LEB;	
	Joint[LWY].RefAngleCurrent = _Q_34x1[LWY_34]*R2D;
	Joint[LWP].RefAngleCurrent = _Q_34x1[LWP_34]*R2D;
	Joint[LWY2].RefAngleCurrent = _Q_34x1[LWY2_34]*R2D;

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

	//----------------------------------------------------------------- 100Hz
	if(half_flag != 0)
	{
		half_flag = 0;
		return 0;
	}
	else
		half_flag = 1;
	
	diff_mm((const float**)_jRH_6x33,6,33, (const float**)_jRH_old2_6x33, _jRHp_6x33);
	subs_m((const float**)_jRH_old_6x33,6,33, _jRH_old2_6x33);
	subs_m((const float**)_jRH_6x33,6,33, _jRH_old_6x33);
	mult_sm((const float**)_jRHp_6x33,6,33, 1.f/(2.f*2.f*DT), _jRHp_6x33);
	
	diff_mm((const float**)_jLH_6x33,6,33, (const float**)_jLH_old2_6x33, _jLHp_6x33);
	subs_m((const float**)_jLH_old_6x33,6,33, _jLH_old2_6x33);
	subs_m((const float**)_jLH_6x33,6,33, _jLH_old_6x33);
	mult_sm((const float**)_jLHp_6x33,6,33, 1.f/(2.f*2.f*DT), _jLHp_6x33);

	for(i=1; i<=34; i++)
		meas_Q_34x1[i] = _Q_34x1[i];

	meas_Q_34x1[RSP_34] = ((float)Joint[RSP].EncoderValue)/Joint[RSP].PPR*D2R;
	meas_Q_34x1[RSR_34] = ((float)Joint[RSR].EncoderValue)/Joint[RSR].PPR*D2R + OFFSET_RSR*D2R;
	meas_Q_34x1[RSY_34] = ((float)Joint[RSY].EncoderValue)/Joint[RSY].PPR*D2R;
	meas_Q_34x1[REB_34] = ((float)Joint[REB].EncoderValue)/Joint[REB].PPR*D2R + OFFSET_REB*D2R;
	meas_Q_34x1[RWY_34] = ((float)Joint[RWY].EncoderValue)/Joint[RWY].PPR*D2R;
	meas_Q_34x1[RWP_34] = ((float)Joint[RWP].EncoderValue)/Joint[RWP].PPR*D2R;
	meas_Q_34x1[RWY2_34] = 0.f;//((float)Joint[RWY2].EncoderValue)/Joint[RWY2].PPR*D2R;

	meas_Q_34x1[LSP_34] = ((float)Joint[LSP].EncoderValue)/Joint[LSP].PPR*D2R;
	meas_Q_34x1[LSR_34] = ((float)Joint[LSR].EncoderValue)/Joint[LSR].PPR*D2R + OFFSET_LSR*D2R;
	meas_Q_34x1[LSY_34] = ((float)Joint[LSY].EncoderValue)/Joint[LSY].PPR*D2R;
	meas_Q_34x1[LEB_34] = ((float)Joint[LEB].EncoderValue)/Joint[LEB].PPR*D2R + OFFSET_LEB*D2R;
	meas_Q_34x1[LWY_34] = ((float)Joint[LWY].EncoderValue)/Joint[LWY].PPR*D2R;
	meas_Q_34x1[LWP_34] = ((float)Joint[LWP].EncoderValue)/Joint[LWP].PPR*D2R;
	meas_Q_34x1[LWY2_34] = 0.f;//((float)Joint[LWY2].EncoderValue)/Joint[LWY2].PPR*D2R;

	meas_qSteer = ((float)Joint[NKY].EncoderValue)/(-2883.f)*360.f*D2R;

	meas_Qp_33x1[RSP_33] = (meas_Q_34x1[RSP_34]-Q_old2_34x1[RSP_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RSR_33] = (meas_Q_34x1[RSR_34]-Q_old2_34x1[RSR_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RSY_33] = (meas_Q_34x1[RSY_34]-Q_old2_34x1[RSY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[REB_33] = (meas_Q_34x1[REB_34]-Q_old2_34x1[REB_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RWY_33] = (meas_Q_34x1[RWY_34]-Q_old2_34x1[RWY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RWP_33] = (meas_Q_34x1[RWP_34]-Q_old2_34x1[RWP_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RWY2_33] = (meas_Q_34x1[RWY2_34]-Q_old2_34x1[RWY2_34])/(2.f*2.f*DT);

	meas_Qp_33x1[LSP_33] = (meas_Q_34x1[LSP_34]-Q_old2_34x1[LSP_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LSR_33] = (meas_Q_34x1[LSR_34]-Q_old2_34x1[LSR_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LSY_33] = (meas_Q_34x1[LSY_34]-Q_old2_34x1[LSY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LEB_33] = (meas_Q_34x1[LEB_34]-Q_old2_34x1[LEB_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LWY_33] = (meas_Q_34x1[LWY_34]-Q_old2_34x1[LWY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LWP_33] = (meas_Q_34x1[LWP_34]-Q_old2_34x1[LWP_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LWY2_33] = (meas_Q_34x1[LWY2_34]-Q_old2_34x1[LWY2_34])/(2.f*2.f*DT);

	meas_wSteer = (meas_qSteer-qSteer_old2)/(2.f*2.f*DT);

	pSharedMemory->disp_Q_34x1[0] = meas_qSteer;

	//------------------------ joint limits
	for(i=20;i<=33;i++)
		if(fabs(meas_Qp_33x1[i])> 15.f)
		{
			/*
			RBpwmCommandHR2ch(CAN1, Joint[LSP].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[LSY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[LWY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[LWY2].JMC, 0, 0, 0);

			RBpwmCommandHR2ch(CAN1, Joint[RSP].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[RSY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[RWY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[RWY2].JMC, 0, 0, 0);
			*/

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
	Q_old2_34x1[RWY2_34] = Q_old_34x1[RWY2_34];

	Q_old2_34x1[LSP_34] = Q_old_34x1[LSP_34];
	Q_old2_34x1[LSR_34] = Q_old_34x1[LSR_34];
	Q_old2_34x1[LSY_34] = Q_old_34x1[LSY_34];
	Q_old2_34x1[LEB_34] = Q_old_34x1[LEB_34];
	Q_old2_34x1[LWY_34] = Q_old_34x1[LWY_34];
	Q_old2_34x1[LWP_34] = Q_old_34x1[LWP_34];
	Q_old2_34x1[LWY2_34] = Q_old_34x1[LWY2_34];

	qSteer_old2 = qSteer_old;

	Q_old_34x1[RSP_34] = meas_Q_34x1[RSP_34];
	Q_old_34x1[RSR_34] = meas_Q_34x1[RSR_34];
	Q_old_34x1[RSY_34] = meas_Q_34x1[RSY_34];
	Q_old_34x1[REB_34] = meas_Q_34x1[REB_34];
	Q_old_34x1[RWY_34] = meas_Q_34x1[RWY_34];
	Q_old_34x1[RWP_34] = meas_Q_34x1[RWP_34];
	Q_old_34x1[RWY2_34] = meas_Q_34x1[RWY2_34];

	Q_old_34x1[LSP_34] = meas_Q_34x1[LSP_34];
	Q_old_34x1[LSR_34] = meas_Q_34x1[LSR_34];
	Q_old_34x1[LSY_34] = meas_Q_34x1[LSY_34];
	Q_old_34x1[LEB_34] = meas_Q_34x1[LEB_34];
	Q_old_34x1[LWY_34] = meas_Q_34x1[LWY_34];
	Q_old_34x1[LWP_34] = meas_Q_34x1[LWP_34];
	Q_old_34x1[LWY2_34] = meas_Q_34x1[LWY2_34];

	qSteer_old = meas_qSteer;

	if(pSharedMemory->torque_mode_flag == 0)
	{
		for(i=1;i<=3;i++)
		{
			pRH2_3x1[i] = _pRH_3x1[i];
			pLH2_3x1[i] = _pLH_3x1[i];
			qRH2_4x1[i] = _qRH_4x1[i];
			qLH2_4x1[i] = _qLH_4x1[i];
		}
		qRH2_4x1[4] = _qRH_4x1[4];
		qLH2_4x1[4] = _qLH_4x1[4];

		pre_gain = 0.f;
		count = 0;
	}
	else if(pSharedMemory->torque_mode_flag == 1)
	{
	/*
		for(i=1;i<=3;i++)
		{
			pRH1_3x1[i] = _pRH_3x1[i];
			pLH1_3x1[i] = _pLH_3x1[i];
			qRH1_4x1[i] = _qRH_4x1[i];
			qLH1_4x1[i] = _qLH_4x1[i];
		}
		qRH1_4x1[4] = _qRH_4x1[4];
		qLH1_4x1[4] = _qLH_4x1[4];
	*/
		pre_gain = (float)count/20.f;
		count++;
		if(count>20)
		{
			RBsetMaxDuty(Joint[LSP].CAN_channel, Joint[LSP].JMC, 40, 40, 100);
			RBsetMaxDuty(Joint[LSY].CAN_channel, Joint[LSY].JMC, 40, 40, 100);
			RBsetMaxDuty(Joint[LWY].CAN_channel, Joint[LWY].JMC, 40, 40, 100);

			RBsetFrictionParameter(Joint[LSP].CAN_channel, Joint[LSP].JMC, (unsigned int)(_friction_para[LSP_33][0]*Joint[LSP].PPR/1000.f), (unsigned int)(_friction_para[LSP_33][1]),	(unsigned int)(_friction_para[LSR_33][0]*Joint[LSR].PPR/1000.f), (unsigned int)(_friction_para[LSR_33][1]));
			RBsetFrictionParameter(Joint[LSY].CAN_channel, Joint[LSY].JMC, (unsigned int)(_friction_para[LSY_33][0]*Joint[LSY].PPR/1000.f), (unsigned int)(_friction_para[LSY_33][1]),	(unsigned int)(_friction_para[LEB_33][0]*Joint[LEB].PPR/1000.f), (unsigned int)(_friction_para[LEB_33][1]));
			RBsetFrictionParameter(Joint[LWY].CAN_channel, Joint[LWY].JMC, (unsigned int)(_friction_para[LWY_33][0]*Joint[LWY].PPR/1000.f), (unsigned int)(_friction_para[LWY_33][1]),	(unsigned int)(_friction_para[LWP_33][0]*Joint[LWP].PPR/1000.f), (unsigned int)(_friction_para[LWP_33][1]));

			RBgainOverrideHR(Joint[LSP].CAN_channel, Joint[LSP].JMC, (unsigned int)pSharedMemory->kp[16], (unsigned int)pSharedMemory->kp[16], (unsigned int)pSharedMemory->kd[16]);
			RBgainOverrideHR(Joint[LSY].CAN_channel, Joint[LSY].JMC, (unsigned int)pSharedMemory->kp[16], (unsigned int)pSharedMemory->kp[16], (unsigned int)pSharedMemory->kd[16]);
			RBgainOverrideHR(Joint[LWY].CAN_channel, Joint[LWY].JMC, (unsigned int)pSharedMemory->kp[16], (unsigned int)pSharedMemory->kp[16], (unsigned int)pSharedMemory->kd[16]);
			
			RBenableFrictionCompensation(Joint[LSP].CAN_channel, Joint[LSP].JMC, 1, 1);
			RBenableFrictionCompensation(Joint[LSY].CAN_channel, Joint[LSY].JMC, 1, 1);
			RBenableFrictionCompensation(Joint[LWY].CAN_channel, Joint[LWY].JMC, 1, 1);
			
			pSharedMemory->torque_mode_flag = 2;
			printf("\n torque mode");
		}
	}
	else if(pSharedMemory->torque_mode_flag == 2)
	{
	/*	for(i=1;i<=3;i++)
		{
			pRH1_3x1[i] = _pRH_3x1[i];
			pLH1_3x1[i] = _pLH_3x1[i];
			qRH1_4x1[i] = _qRH_4x1[i];
			qLH1_4x1[i] = _qLH_4x1[i];
		}
		qRH1_4x1[4] = _qRH_4x1[4];
		qLH1_4x1[4] = _qLH_4x1[4];
	*/
		pre_gain = 1.f;
		count = 0;
	}
	else if(pSharedMemory->torque_mode_flag == 3)
	{	
		/*
		_Q_34x1[RSP_34] = meas_Q_34x1[RSP_34];
		_Q_34x1[RSR_34] = meas_Q_34x1[RSR_34];
		_Q_34x1[RSY_34] = meas_Q_34x1[RSY_34];
		_Q_34x1[REB_34] = meas_Q_34x1[REB_34];
		_Q_34x1[RWY_34] = meas_Q_34x1[RWY_34];
		_Q_34x1[RWP_34] = meas_Q_34x1[RWP_34];
		_Q_34x1[RWY2_34] = meas_Q_34x1[RWY2_34];

		_Q_34x1[LSP_34] = meas_Q_34x1[LSP_34];
		_Q_34x1[LSR_34] = meas_Q_34x1[LSR_34];
		_Q_34x1[LSY_34] = meas_Q_34x1[LSY_34];
		_Q_34x1[LEB_34] = meas_Q_34x1[LEB_34];
		_Q_34x1[LWY_34] = meas_Q_34x1[LWY_34];
		_Q_34x1[LWP_34] = meas_Q_34x1[LWP_34];
		_Q_34x1[LWY2_34] = meas_Q_34x1[LWY2_34];

		_Qp_33x1[RSP_33] = meas_Qp_33x1[RSP_33];
		_Qp_33x1[RSR_33] = meas_Qp_33x1[RSR_33];
		_Qp_33x1[RSY_33] = meas_Qp_33x1[RSY_33];
		_Qp_33x1[REB_33] = meas_Qp_33x1[REB_33];
		_Qp_33x1[RWY_33] = meas_Qp_33x1[RWY_33];
		_Qp_33x1[RWP_33] = meas_Qp_33x1[RWP_33];
		_Qp_33x1[RWY2_33] = meas_Qp_33x1[RWY2_33];

		_Qp_33x1[LSP_33] = meas_Qp_33x1[LSP_33];
		_Qp_33x1[LSR_33] = meas_Qp_33x1[LSR_33];
		_Qp_33x1[LSY_33] = meas_Qp_33x1[LSY_33];
		_Qp_33x1[LEB_33] = meas_Qp_33x1[LEB_33];
		_Qp_33x1[LWY_33] = meas_Qp_33x1[LWY_33];
		_Qp_33x1[LWP_33] = meas_Qp_33x1[LWP_33];
		_Qp_33x1[LWY2_33] = meas_Qp_33x1[LWY2_33];

		Joint[RSP].RefAngleCurrent = _Q_34x1[RSP_34]*R2D;
		Joint[RSR].RefAngleCurrent = _Q_34x1[RSR_34]*R2D - OFFSET_RSR;
		Joint[RSY].RefAngleCurrent = _Q_34x1[RSY_34]*R2D;
		Joint[REB].RefAngleCurrent = _Q_34x1[REB_34]*R2D - OFFSET_REB;
		Joint[RWY].RefAngleCurrent = _Q_34x1[RWY_34]*R2D;
		Joint[RWP].RefAngleCurrent = _Q_34x1[RWP_34]*R2D;
		Joint[RWY2].RefAngleCurrent = _Q_34x1[RWY2_34]*R2D;
		Joint[LSP].RefAngleCurrent = _Q_34x1[LSP_34]*R2D;
		Joint[LSR].RefAngleCurrent = _Q_34x1[LSR_34]*R2D - OFFSET_LSR;
		Joint[LSY].RefAngleCurrent = _Q_34x1[LSY_34]*R2D;
		Joint[LEB].RefAngleCurrent = _Q_34x1[LEB_34]*R2D - OFFSET_LEB;	
		Joint[LWY].RefAngleCurrent = _Q_34x1[LWY_34]*R2D;
		Joint[LWP].RefAngleCurrent = _Q_34x1[LWP_34]*R2D;
		Joint[LWY2].RefAngleCurrent = _Q_34x1[LWY2_34]*R2D;
		
		SendRunStopCMD(Joint[LSP].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[LSP].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[LSY].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[LSY].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[LWY].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[LWY].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[LWY2].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[LWY2].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[RSP].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[RSP].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[RSY].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[RSY].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[RWY].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[RWY].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[RWY2].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[RWY2].JMC, 0, 0, 0);
		*/
		pre_gain = 0.f;
		pSharedMemory->torque_mode_flag = 0;
		printf("\n position mode");
	}
	/*
	_Q_34x1[RSP_34] =  pre_gain*meas_Q_34x1[RSP_34] + (1.f-pre_gain)*_Q_34x1[RSP_34];
	_Q_34x1[RSR_34] =  pre_gain*meas_Q_34x1[RSR_34] + (1.f-pre_gain)*_Q_34x1[RSR_34];
	_Q_34x1[RSY_34] =  pre_gain*meas_Q_34x1[RSY_34] + (1.f-pre_gain)*_Q_34x1[RSY_34];
	_Q_34x1[REB_34] =  pre_gain*meas_Q_34x1[REB_34] + (1.f-pre_gain)*_Q_34x1[REB_34];
	_Q_34x1[RWY_34] =  pre_gain*meas_Q_34x1[RWY_34] + (1.f-pre_gain)*_Q_34x1[RWY_34];
	_Q_34x1[RWP_34] =  pre_gain*meas_Q_34x1[RWP_34] + (1.f-pre_gain)*_Q_34x1[RWP_34];
	_Q_34x1[RWY2_34] =  pre_gain*meas_Q_34x1[RWY2_34] + (1.f-pre_gain)*_Q_34x1[RWY2_34];
	
	_Q_34x1[LSP_34] =  pre_gain*meas_Q_34x1[LSP_34] + (1.f-pre_gain)*_Q_34x1[LSP_34];
	_Q_34x1[LSR_34] =  pre_gain*meas_Q_34x1[LSR_34] + (1.f-pre_gain)*_Q_34x1[LSR_34];
	_Q_34x1[LSY_34] =  pre_gain*meas_Q_34x1[LSY_34] + (1.f-pre_gain)*_Q_34x1[LSY_34];
	_Q_34x1[LEB_34] =  pre_gain*meas_Q_34x1[LEB_34] + (1.f-pre_gain)*_Q_34x1[LEB_34];
	_Q_34x1[LWY_34] =  pre_gain*meas_Q_34x1[LWY_34] + (1.f-pre_gain)*_Q_34x1[LWY_34];
	_Q_34x1[LWP_34] =  pre_gain*meas_Q_34x1[LWP_34] + (1.f-pre_gain)*_Q_34x1[LWP_34];
	_Q_34x1[LWY2_34] =  pre_gain*meas_Q_34x1[LWY2_34] + (1.f-pre_gain)*_Q_34x1[LWY2_34];

	_Qp_33x1[RSP_33] =  pre_gain*meas_Qp_33x1[RSP_33] + (1.f-pre_gain)*_Qp_33x1[RSP_33];
	_Qp_33x1[RSR_33] =  pre_gain*meas_Qp_33x1[RSR_33] + (1.f-pre_gain)*_Qp_33x1[RSR_33];
	_Qp_33x1[RSY_33] =  pre_gain*meas_Qp_33x1[RSY_33] + (1.f-pre_gain)*_Qp_33x1[RSY_33];
	_Qp_33x1[REB_33] =  pre_gain*meas_Qp_33x1[REB_33] + (1.f-pre_gain)*_Qp_33x1[REB_33];
	_Qp_33x1[RWY_33] =  pre_gain*meas_Qp_33x1[RWY_33] + (1.f-pre_gain)*_Qp_33x1[RWY_33];
	_Qp_33x1[RWP_33] =  pre_gain*meas_Qp_33x1[RWP_33] + (1.f-pre_gain)*_Qp_33x1[RWP_33];
	_Qp_33x1[RWY2_33] =  pre_gain*meas_Qp_33x1[RWY2_33] + (1.f-pre_gain)*_Qp_33x1[RWY2_33];
	
	_Qp_33x1[LSP_33] =  pre_gain*meas_Qp_33x1[LSP_33] + (1.f-pre_gain)*_Qp_33x1[LSP_33];
	_Qp_33x1[LSR_33] =  pre_gain*meas_Qp_33x1[LSR_33] + (1.f-pre_gain)*_Qp_33x1[LSR_33];
	_Qp_33x1[LSY_33] =  pre_gain*meas_Qp_33x1[LSY_33] + (1.f-pre_gain)*_Qp_33x1[LSY_33];
	_Qp_33x1[LEB_33] =  pre_gain*meas_Qp_33x1[LEB_33] + (1.f-pre_gain)*_Qp_33x1[LEB_33];
	_Qp_33x1[LWY_33] =  pre_gain*meas_Qp_33x1[LWY_33] + (1.f-pre_gain)*_Qp_33x1[LWY_33];
	_Qp_33x1[LWP_33] =  pre_gain*meas_Qp_33x1[LWP_33] + (1.f-pre_gain)*_Qp_33x1[LWP_33];
	_Qp_33x1[LWY2_33] =  pre_gain*meas_Qp_33x1[LWY2_33] + (1.f-pre_gain)*_Qp_33x1[LWY2_33];
	*/

	if(pSharedMemory->torque_mode_flag == 1 || pSharedMemory->torque_mode_flag == 2)
	{
		for(i=1;i<=33;i++)
			duty_joint_limit_33x1[i] = 0.f;
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

		for(i=1;i<=6;i++)
			for(j=1;j<=33;j++)
			{
				_jT1_33x33[i][j] = _jRH_6x33[i][j];			
				_jT1_33x33[i+6][j] = _jLH_6x33[i][j];			
			}

		for(i=1; i<=12; i++)
		{
			_jT1_33x33[i][1] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][2] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][3] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][4] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][5] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][6] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][7] = 0.f;  // exclude WST from the solution
		}


		if(pSharedMemory->steer_demo == 1)
		{
			qSteer_errSum = 0.f;
			t_steer = 0.f;
			steer_count = 0;

			/*neutral_Q_34x1[RSP_34] = _Q_34x1[RSP_34];
			neutral_Q_34x1[RSR_34] = _Q_34x1[RSR_34];
			neutral_Q_34x1[RSY_34] = _Q_34x1[RSY_34];
			neutral_Q_34x1[REB_34] = _Q_34x1[REB_34];
			neutral_Q_34x1[RWY_34] = _Q_34x1[RWY_34];
			neutral_Q_34x1[RWP_34] = _Q_34x1[RWP_34];
			neutral_Q_34x1[RWY2_34] = _Q_34x1[RWY2_34];
			
			neutral_Q_34x1[LSP_34] = _Q_34x1[LSP_34];
			neutral_Q_34x1[LSR_34] = _Q_34x1[LSR_34];
			neutral_Q_34x1[LSY_34] = _Q_34x1[LSY_34];
			neutral_Q_34x1[LEB_34] = _Q_34x1[LEB_34];
			neutral_Q_34x1[LWY_34] = _Q_34x1[LWY_34];
			neutral_Q_34x1[LWP_34] = _Q_34x1[LWP_34];
			neutral_Q_34x1[LWY2_34] = _Q_34x1[LWY2_34];

			pRH2_3x1[1] = _pRH_3x1[1];
			pRH2_3x1[2] = _pRH_3x1[2];
			pRH2_3x1[3] = _pRH_3x1[3];
			qRH2_4x1[1] = _qRH_4x1[1];
			qRH2_4x1[2] = _qRH_4x1[2];
			qRH2_4x1[3] = _qRH_4x1[3];
			qRH2_4x1[4] = _qRH_4x1[4];
							
			pLH2_3x1[1] = _pLH_3x1[1];
			pLH2_3x1[2] = _pLH_3x1[2];
			pLH2_3x1[3] = _pLH_3x1[3];
			qLH2_4x1[1] = _qLH_4x1[1];
			qLH2_4x1[2] = _qLH_4x1[2];
			qLH2_4x1[3] = _qLH_4x1[3];
			qLH2_4x1[4] = _qLH_4x1[4];
			*/
			_TEMP3_34x34[1][1] = Xrh_6x1[1] = 0.f;
			_TEMP3_34x34[1][2] = Xrh_6x1[2] = 0.f;
			_TEMP3_34x34[1][3] = Xrh_6x1[3] = 0.f;
			_TEMP3_34x34[1][4] = Xrh_6x1[4] = 0.f;
			_TEMP3_34x34[1][5] = Xrh_6x1[5] = 0.f;
			_TEMP3_34x34[1][6] = Xrh_6x1[6] = 0.f;
			_TEMP3_34x34[1][7] = Xlh_6x1[1] = 0.f;
			_TEMP3_34x34[1][8] = Xlh_6x1[2] = 0.f;
			_TEMP3_34x34[1][9] = Xlh_6x1[3] = 0.f;
			_TEMP3_34x34[1][10] = Xlh_6x1[4] = 0.f;
			_TEMP3_34x34[1][11] = Xlh_6x1[5] = 0.f;
			_TEMP3_34x34[1][12] = Xlh_6x1[6] = 0.f;
		}
		else if(pSharedMemory->steer_demo == 2 || pSharedMemory->steer_demo == 3)
		{

			if(pSharedMemory->steer_demo == 2)
			{
				qSteer1 = qSteer0 = des_qSteer = meas_qSteer;
				qSteer_errSum = 0.f;
				steer_count = 0;
				t_steer = 0.f;
				pSharedMemory->steer_demo = 3;
			}

			pRH2_3x1[1] = _pRH_3x1[1];
			pRH2_3x1[2] = _pRH_3x1[2];
			pRH2_3x1[3] = _pRH_3x1[3];
			qRH2_4x1[1] = _qRH_4x1[1];
			qRH2_4x1[2] = _qRH_4x1[2];
			qRH2_4x1[3] = _qRH_4x1[3];
			qRH2_4x1[4] = _qRH_4x1[4];
							
			pLH2_3x1[1] = _pLH_3x1[1];
			pLH2_3x1[2] = _pLH_3x1[2];
			pLH2_3x1[3] = _pLH_3x1[3];
			qLH2_4x1[1] = _qLH_4x1[1];
			qLH2_4x1[2] = _qLH_4x1[2];
			qLH2_4x1[3] = _qLH_4x1[3];
			qLH2_4x1[4] = _qLH_4x1[4];

			if(t_steer <= pSharedMemory->steer_time[steer_count])
				des_qSteer =  qSteer1 + one_cos(t_steer, qSteer0+pSharedMemory->steer_ang[steer_count]*D2R-qSteer1, pSharedMemory->steer_time[steer_count]);

			t_steer += 0.01f;
			if(t_steer > pSharedMemory->steer_time[steer_count])
			{
				steer_count++;
				if(steer_count < N_STEER)
				{
					qSteer1 = des_qSteer;
					t_steer = 0.f;
				}
				else
				{
					steer_count--;
					t_steer -= 0.01f;					
				}
			}

			qSteer_errSum += (des_qSteer-meas_qSteer)*0.01f;
			ftemp1_7x1[1] = (pSharedMemory->kp[16]*qSteer_errSum - pSharedMemory->kd[9]*meas_wSteer + pSharedMemory->kp[9]*(des_qSteer-meas_qSteer));
			if(ftemp1_7x1[1]>100.f)
				ftemp1_7x1[1] = 100.f;
			else if(ftemp1_7x1[1]<-100.f)
				ftemp1_7x1[1] = -100.f;
			ftemp1_7x1[2] = 0.f;
			ftemp1_7x1[3] = 0.f;

			pSharedMemory->disp_Q_34x1[0] = ftemp1_7x1[1]*D2R;

			QT2DC(_qRH_4x1, _TEMP1_34x34);
			mult_mv((const float**)_TEMP1_34x34,3,3, ftemp1_7x1, Xrh_6x1);
			Xrh_6x1[4] = 0.f;
			Xrh_6x1[5] = 0.f;
			Xrh_6x1[6] = 0.f;

			QT2DC(_qLH_4x1, _TEMP1_34x34);
			mult_smv(-1.f, (const float**)_TEMP1_34x34,3,3, ftemp1_7x1, Xlh_6x1);
			Xlh_6x1[4] = 0.f;
			Xlh_6x1[5] = 0.f;
			Xlh_6x1[6] = 0.f;

			_TEMP3_34x34[1][1] = Xrh_6x1[1];
			_TEMP3_34x34[1][2] = Xrh_6x1[2];
			_TEMP3_34x34[1][3] = Xrh_6x1[3];
			_TEMP3_34x34[1][4] = Xrh_6x1[4];
			_TEMP3_34x34[1][5] = Xrh_6x1[5];
			_TEMP3_34x34[1][6] = Xrh_6x1[6];
			_TEMP3_34x34[1][7] = Xlh_6x1[1];
			_TEMP3_34x34[1][8] = Xlh_6x1[2];
			_TEMP3_34x34[1][9] = Xlh_6x1[3];
			_TEMP3_34x34[1][10] = Xlh_6x1[4];
			_TEMP3_34x34[1][11] = Xlh_6x1[5];
			_TEMP3_34x34[1][12] = Xlh_6x1[6];
		}
		else
		{
			qSteer_errSum = 0.f;

			des_pRH_3x1[1] = pRH2_3x1[1];
			des_pRH_3x1[2] = pRH2_3x1[2];
			des_pRH_3x1[3] = pRH2_3x1[3];
			des_qRH_4x1[1] = qRH2_4x1[1];
			des_qRH_4x1[2] = qRH2_4x1[2];
			des_qRH_4x1[3] = qRH2_4x1[3];
			des_qRH_4x1[4] = qRH2_4x1[4];
			des_vRH_3x1[1] = 0.f;
			des_vRH_3x1[2] = 0.f;
			des_vRH_3x1[3] = 0.f;
			des_wRH_3x1[1] = 0.f;
			des_wRH_3x1[2] = 0.f;
			des_wRH_3x1[3] = 0.f;
				
			des_pLH_3x1[1] = pLH2_3x1[1];
			des_pLH_3x1[2] = pLH2_3x1[2];
			des_pLH_3x1[3] = pLH2_3x1[3];
			des_qLH_4x1[1] = qLH2_4x1[1];
			des_qLH_4x1[2] = qLH2_4x1[2];
			des_qLH_4x1[3] = qLH2_4x1[3];
			des_qLH_4x1[4] = qLH2_4x1[4];
			des_vLH_3x1[1] = 0.f;
			des_vLH_3x1[2] = 0.f;
			des_vLH_3x1[3] = 0.f;
			des_wLH_3x1[1] = 0.f;
			des_wLH_3x1[2] = 0.f;
			des_wLH_3x1[3] = 0.f;

			diff_vv(des_pRH_3x1,3, _pRH_3x1, ftemp3_7x1);
			mult_mv((const float**)_jRH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
			diff_vv(des_vRH_3x1,3, ftemp2_7x1, ftemp1_7x1);
			Xrh_6x1[1] = pSharedMemory->kp[0]*ftemp3_7x1[1] + pSharedMemory->kd[0]*ftemp1_7x1[1];
			Xrh_6x1[2] = pSharedMemory->kp[1]*ftemp3_7x1[2] + pSharedMemory->kd[1]*ftemp1_7x1[2];
			Xrh_6x1[3] = pSharedMemory->kp[2]*ftemp3_7x1[3] + pSharedMemory->kd[2]*ftemp1_7x1[3];
			QTdel(des_qRH_4x1, _qRH_4x1, ftemp3_7x1);
			diff_vv(des_wRH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
			sum_svsv(pSharedMemory->kp[3],ftemp3_7x1,3, pSharedMemory->kd[3],ftemp4_7x1, &Xrh_6x1[3]);

			diff_vv(des_pLH_3x1,3, _pLH_3x1, ftemp3_7x1);
			mult_mv((const float**)_jLH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
			diff_vv(des_vLH_3x1,3, ftemp2_7x1, ftemp1_7x1);
			Xlh_6x1[1] = pSharedMemory->kp[4]*ftemp3_7x1[1] + pSharedMemory->kd[4]*ftemp1_7x1[1];
			Xlh_6x1[2] = pSharedMemory->kp[5]*ftemp3_7x1[2] + pSharedMemory->kd[5]*ftemp1_7x1[2];
			Xlh_6x1[3] = pSharedMemory->kp[6]*ftemp3_7x1[3] + pSharedMemory->kd[6]*ftemp1_7x1[3];
			QTdel(des_qLH_4x1, _qLH_4x1, ftemp3_7x1);
			diff_vv(des_wLH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
			sum_svsv(pSharedMemory->kp[7],ftemp3_7x1,3, pSharedMemory->kd[7],ftemp4_7x1, &Xlh_6x1[3]);

			_TEMP3_34x34[1][1] = Xrh_6x1[1];
			_TEMP3_34x34[1][2] = Xrh_6x1[2];
			_TEMP3_34x34[1][3] = Xrh_6x1[3];
			_TEMP3_34x34[1][4] = Xrh_6x1[4];
			_TEMP3_34x34[1][5] = Xrh_6x1[5];
			_TEMP3_34x34[1][6] = Xrh_6x1[6];
			_TEMP3_34x34[1][7] = Xlh_6x1[1];
			_TEMP3_34x34[1][8] = Xlh_6x1[2];
			_TEMP3_34x34[1][9] = Xlh_6x1[3];
			_TEMP3_34x34[1][10] = Xlh_6x1[4];
			_TEMP3_34x34[1][11] = Xlh_6x1[5];
			_TEMP3_34x34[1][12] = Xlh_6x1[6];
		}

		trans(1.f, (const float**)_jT1_33x33,12,33, _TEMP2_34x34);
		mult_mv((const float**)_TEMP2_34x34,33,12, _TEMP3_34x34[1], ct_33x1);

		if(pinv_SR((const float**)_jT1_33x33, 12, 33, (float)1.e-10, _jT1inv_33x33) != 0)
		{
			printf("\n pinv error!");
			return -2; // singularity occurred
		}
		mult_mm((const float**)_jT1inv_33x33,33,12, (const float**)_jT1_33x33, 33, _TEMP1_34x34);
		diff_mm((const float**)_EYE_33, 33,33, (const float**)_TEMP1_34x34, _N1_33x33);

		if(pSharedMemory->steer_demo == 3)
		{
			for(i=1;i<=33;i++)
				_TEMP3_34x34[1][i] = 0.f;
		//	_TEMP3_34x34[1][20] = pSharedMemory->kd[8]*(-_Qp_33x1[RSP_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RSP_34]-_Q_34x1[RSP_34]);
		//	_TEMP3_34x34[1][21] = pSharedMemory->kd[8]*(-_Qp_33x1[RSR_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RSR_34]-_Q_34x1[RSR_34]);
		//	_TEMP3_34x34[1][22] = pSharedMemory->kd[8]*(-_Qp_33x1[RSY_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RSY_34]-_Q_34x1[RSY_34]);
		//	_TEMP3_34x34[1][23] = pSharedMemory->kd[8]*(-_Qp_33x1[REB_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[REB_34]-_Q_34x1[REB_34]);
			_TEMP3_34x34[1][23] = pSharedMemory->kd[8]*(-_Qp_33x1[RWY_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RWY_34]-_Q_34x1[RWY_34]);
		//	_TEMP3_34x34[1][25] = pSharedMemory->kd[8]*(-_Qp_33x1[RWP_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RWP_34]-_Q_34x1[RWP_34]);
		//	_TEMP3_34x34[1][26] = pSharedMemory->kd[8]*(-_Qp_33x1[LSP_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LSP_34]-_Q_34x1[LSP_34]);
		//	_TEMP3_34x34[1][27] = pSharedMemory->kd[8]*(-_Qp_33x1[LSR_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LSR_34]-_Q_34x1[LSR_34]);
		//	_TEMP3_34x34[1][28] = pSharedMemory->kd[8]*(-_Qp_33x1[LSY_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LSY_34]-_Q_34x1[LSY_34]);
		//	_TEMP3_34x34[1][29] = pSharedMemory->kd[8]*(-_Qp_33x1[LEB_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LEB_34]-_Q_34x1[LEB_34]);
			_TEMP3_34x34[1][30] = pSharedMemory->kd[8]*(-_Qp_33x1[LWY_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LWY_34]-_Q_34x1[LWY_34]);
		//	_TEMP3_34x34[1][31] = pSharedMemory->kd[8]*(-_Qp_33x1[LWP_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LWP_34]-_Q_34x1[LWP_34]);
		//	_TEMP3_34x34[1][32] = pSharedMemory->kd[8]*(-_Qp_33x1[RWY2_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[RWY2_34]-_Q_34x1[RWY2_34]);
		//	_TEMP3_34x34[1][33] = pSharedMemory->kd[8]*(-_Qp_33x1[LWY2_33]) + pSharedMemory->kp[8]*(neutral_Q_34x1[LWY2_34]-_Q_34x1[LWY2_34]);
			mult_mv((const float**)_N1_33x33,33,33, _TEMP3_34x34[1], _TEMP1_34x34[1]);
			sum_vv(ct_33x1,33, (const float*)_TEMP1_34x34[1], ct_33x1);
		}

		ct_33x1[RSP_33] = _gain_task[RSP_33]*ct_33x1[RSP_33]+_gain_gravity[RSP_33]*gravity_33x1[RSP_33];
		ct_33x1[RSR_33] = _gain_task[RSR_33]*ct_33x1[RSR_33]+_gain_gravity[RSR_33]*gravity_33x1[RSR_33];
		ct_33x1[RSY_33] = _gain_task[RSY_33]*ct_33x1[RSY_33]+_gain_gravity[RSY_33]*gravity_33x1[RSY_33];
		ct_33x1[REB_33] = _gain_task[REB_33]*ct_33x1[REB_33]+_gain_gravity[REB_33]*gravity_33x1[REB_33];
		ct_33x1[RWY_33] = _gain_task[RWY_33]*ct_33x1[RWY_33]+_gain_gravity[RWY_33]*gravity_33x1[RWY_33];
		ct_33x1[RWP_33] = _gain_task[RWP_33]*ct_33x1[RWP_33]+_gain_gravity[RWP_33]*gravity_33x1[RWP_33];
		ct_33x1[RWY2_33] = _gain_task[RWY2_33]*ct_33x1[RWY2_33]+_gain_gravity[RWY2_33]*gravity_33x1[RWY2_33];

		ct_33x1[LSP_33] = _gain_task[LSP_33]*ct_33x1[LSP_33]+_gain_gravity[LSP_33]*gravity_33x1[LSP_33];
		ct_33x1[LSR_33] = _gain_task[LSR_33]*ct_33x1[LSR_33]+_gain_gravity[LSR_33]*gravity_33x1[LSR_33];
		ct_33x1[LSY_33] = _gain_task[LSY_33]*ct_33x1[LSY_33]+_gain_gravity[LSY_33]*gravity_33x1[LSY_33];
		ct_33x1[LEB_33] = _gain_task[LEB_33]*ct_33x1[LEB_33]+_gain_gravity[LEB_33]*gravity_33x1[LEB_33];
		ct_33x1[LWY_33] = _gain_task[LWY_33]*ct_33x1[LWY_33]+_gain_gravity[LWY_33]*gravity_33x1[LWY_33];
		ct_33x1[LWP_33] = _gain_task[LWP_33]*ct_33x1[LWP_33]+_gain_gravity[LWP_33]*gravity_33x1[LWP_33];
		ct_33x1[LWY2_33] = _gain_task[LWY2_33]*ct_33x1[LWY2_33]+_gain_gravity[LWY2_33]*gravity_33x1[LWY2_33];

		pSharedMemory->disp_ct_33x1[RSP_33] = ct_33x1[RSP_33];
		pSharedMemory->disp_ct_33x1[RSR_33] = ct_33x1[RSR_33];
		pSharedMemory->disp_ct_33x1[RSY_33] = ct_33x1[RSY_33];
		pSharedMemory->disp_ct_33x1[REB_33] = ct_33x1[REB_33];
		pSharedMemory->disp_ct_33x1[RWY_33] = ct_33x1[RWY_33];
		pSharedMemory->disp_ct_33x1[RWP_33] = ct_33x1[RWP_33];
		pSharedMemory->disp_ct_33x1[RWY2_33] = ct_33x1[RWY2_33];
		
		pSharedMemory->disp_ct_33x1[LSP_33] = ct_33x1[LSP_33];
		pSharedMemory->disp_ct_33x1[LSR_33] = ct_33x1[LSR_33];
		pSharedMemory->disp_ct_33x1[LSY_33] = ct_33x1[LSY_33];
		pSharedMemory->disp_ct_33x1[LEB_33] = ct_33x1[LEB_33];
		pSharedMemory->disp_ct_33x1[LWY_33] = ct_33x1[LWY_33];
		pSharedMemory->disp_ct_33x1[LWP_33] = ct_33x1[LWP_33];
		pSharedMemory->disp_ct_33x1[LWY2_33] = ct_33x1[LWY2_33];
		
		pSharedMemory->disp_duty_33x1[RSP_33] = duty_33x1[RSP_33] = limitDuty(0.4f, torque2duty(RSP, pre_gain*ct_33x1[RSP_33]) + pre_gain*fric_compen_33x1[RSP_33]*0.f + duty_joint_limit_33x1[RSP_33]);
		pSharedMemory->disp_duty_33x1[RSR_33] = duty_33x1[RSR_33] = limitDuty(0.4f, torque2duty(RSR, pre_gain*ct_33x1[RSR_33]) + pre_gain*fric_compen_33x1[RSR_33]*0.f + duty_joint_limit_33x1[RSR_33]);
		pSharedMemory->disp_duty_33x1[RSY_33] = duty_33x1[RSY_33] = limitDuty(0.4f, torque2duty(RSY, pre_gain*ct_33x1[RSY_33]) + pre_gain*fric_compen_33x1[RSY_33]*0.f + duty_joint_limit_33x1[RSY_33]);
		pSharedMemory->disp_duty_33x1[REB_33] = duty_33x1[REB_33] = limitDuty(0.4f, torque2duty(REB, pre_gain*ct_33x1[REB_33]) + pre_gain*fric_compen_33x1[REB_33]*0.f + duty_joint_limit_33x1[REB_33]);
		pSharedMemory->disp_duty_33x1[RWY_33] = duty_33x1[RWY_33] = limitDuty(0.4f, torque2duty(RWY, pre_gain*ct_33x1[RWY_33]) + pre_gain*fric_compen_33x1[RWY_33]*0.f + duty_joint_limit_33x1[RWY_33]);
		pSharedMemory->disp_duty_33x1[RWP_33] = duty_33x1[RWP_33] = limitDuty(0.4f, torque2duty(RWP, pre_gain*ct_33x1[RWP_33]) + pre_gain*fric_compen_33x1[RWP_33]*0.f + duty_joint_limit_33x1[RWP_33]);
		pSharedMemory->disp_duty_33x1[RWY2_33] = duty_33x1[RWY2_33] = limitDuty(0.4f, torque2duty(RWY2, pre_gain*ct_33x1[RWY2_33]) + pre_gain*fric_compen_33x1[RWY2_33]*0.f + duty_joint_limit_33x1[RWY2_33]);

		pSharedMemory->disp_duty_33x1[LSP_33] = duty_33x1[LSP_33] = limitDuty(0.4f, torque2duty(LSP, pre_gain*ct_33x1[LSP_33]) + pre_gain*fric_compen_33x1[LSP_33]*0.f + duty_joint_limit_33x1[LSP_33]);
		pSharedMemory->disp_duty_33x1[LSR_33] = duty_33x1[LSR_33] = limitDuty(0.4f, torque2duty(LSR, pre_gain*ct_33x1[LSR_33]) + pre_gain*fric_compen_33x1[LSR_33]*0.f + duty_joint_limit_33x1[LSR_33]);
		pSharedMemory->disp_duty_33x1[LSY_33] = duty_33x1[LSY_33] = limitDuty(0.4f, torque2duty(LSY, pre_gain*ct_33x1[LSY_33]) + pre_gain*fric_compen_33x1[LSY_33]*0.f + duty_joint_limit_33x1[LSY_33]);
		pSharedMemory->disp_duty_33x1[LEB_33] = duty_33x1[LEB_33] = limitDuty(0.4f, torque2duty(LEB, pre_gain*ct_33x1[LEB_33]) + pre_gain*fric_compen_33x1[LEB_33]*0.f + duty_joint_limit_33x1[LEB_33]);
		pSharedMemory->disp_duty_33x1[LWY_33] = duty_33x1[LWY_33] = limitDuty(0.4f, torque2duty(LWY, pre_gain*ct_33x1[LWY_33]) + pre_gain*fric_compen_33x1[LWY_33]*0.f + duty_joint_limit_33x1[LWY_33]);
		pSharedMemory->disp_duty_33x1[LWP_33] = duty_33x1[LWP_33] = limitDuty(0.4f, torque2duty(LWP, pre_gain*ct_33x1[LWP_33]) + pre_gain*fric_compen_33x1[LWP_33]*0.f + duty_joint_limit_33x1[LWP_33]);
		pSharedMemory->disp_duty_33x1[LWY2_33] = duty_33x1[LWY2_33] = limitDuty(0.4f, torque2duty(LWY2, pre_gain*ct_33x1[LWY2_33]) + pre_gain*fric_compen_33x1[LWY2_33]*0.f + duty_joint_limit_33x1[LWY2_33]);
		
		//RBpwmCommandHR2ch(CAN1, Joint[RSP].JMC, (int)(PWM_SIGN_RSP*1000.f*duty_33x1[RSP_33]), (int)(PWM_SIGN_RSR*1000.f*duty_33x1[RSR_33]), 1);
		//RBpwmCommandHR2ch(CAN1, Joint[RSY].JMC, (int)(PWM_SIGN_RSY*1000.f*duty_33x1[RSY_33]), (int)(PWM_SIGN_REB*1000.f*duty_33x1[REB_33]), 1);
		//RBpwmCommandHR2ch(CAN1, Joint[RWY].JMC, (int)(PWM_SIGN_RWY*1000.f*duty_33x1[RWY_33]), (int)(PWM_SIGN_RWP*1000.f*duty_33x1[RWP_33]), 1);
		//RBpwmCommandHR2ch(CAN1, Joint[RWY2].JMC, (int)(PWM_SIGN_RWY2*1000.f*duty_33x1[RWY2_33]), 0, 1);

		RBpwmCommandHR2ch(CAN1, Joint[LSP].JMC, (int)(PWM_SIGN_LSP*1000.f*duty_33x1[LSP_33]), (int)(PWM_SIGN_LSR*1000.f*duty_33x1[LSR_33]), 4);
		RBpwmCommandHR2ch(CAN1, Joint[LSY].JMC, (int)(PWM_SIGN_LSY*1000.f*duty_33x1[LSY_33]), (int)(PWM_SIGN_LEB*1000.f*duty_33x1[LEB_33]), 4);
		RBpwmCommandHR2ch(CAN1, Joint[LWY].JMC, (int)(PWM_SIGN_LWY*1000.f*duty_33x1[LWY_33]), (int)(PWM_SIGN_LWP*1000.f*duty_33x1[LWP_33]), 4);
		//RBpwmCommandHR2ch(CAN1, Joint[LWY2].JMC, (int)(PWM_SIGN_LWY2*1000.f*duty_33x1[LWY2_33]), 0, 1);
		
	}
	
	return 0;	
}


