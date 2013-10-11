#ifndef KIRK_H
#define KIRK_H

#define A_ZMP_Y		70.0 // [mm] ZMP Sway ??  
#define A_ZMP_Y_BP		89.0 // [mm] ZMP Sway ??  
#define	RF_1	0
#define	LF_2	1
#define	RF_3	2
#define	LF_1	3
#define	RF_2	4
#define	LF_3	5
#define deg2rad		0.017453292519943
#define rad2deg		57.295779513082323
//#define Init_R1_th     45.7711     // [deg]
//#define Init_R2_th     338.0156    // [deg]
//#define Init_L1_th     134.2279    // [deg]
//#define Init_L2_th     201.9854    // [deg]
//#define Radius_R1      0.162722    // [m]
//#define Radius_L1      0.162719    // [m]
//#define Radius_R2      0.31147     // [m]
//#define Radius_L2      0.311472    // [m]
#define EARLY_LAND_THRES_1	30  // [N] ?? ?? ?? ??
#define EARLY_LAND_THRES_2	20  // [N] ?? ?? ?? ??
#define LATE_LAND_MAX_DEPTH	50	// [mm] ?? ?? ? ???? ?? ?? ??
#define V_DAMPER_GAIN	2  // ?? ? ?? ?? ??
#define F_CUT_Number_Support	3.0	// [Hz] ??? ?? ?? LPF ???  3.0
#define F_CUT_Number_Support_BP	3.0	// [Hz] ??? ?? ?? LPF ???  3.0
#define Del_T 0.005
#define X_ZMP_OFFSET 100.
#define X_ZMP_OFFSET_BWD 120.
#define X_ZMP_OFFSET_BP 10.//25.
#define DES_RMS_Y_ZMP	78.0
#define F_cutoff_acc_HPF 1.0
#define HR_COMP_ANG	1.5
#define Foot_Up_AP_Deg	12.//10.0 // [deg] 다리 상승시 자연스런 모션을 위한 발목 피칭각도   2013_08_30
#define Foot_Dn_AP_Deg	-2.//-1.0  // [deg] 다리 하강시 힐 접촉을 위한 발목 피칭 각도	2013_08_30


void prof_cyclone(int *profStateUp,int *profStateDn, float *profResult, int timeIndex, int timeStart, int timeDuring);
void half_cycloneUp(int *profState,double *profResult,int timeIndex,int timeStart,int timeDuring);
void half_cycloneDn(int *profState,double *profResult,int timeIndex,int timeStart,int timeDuring);
void prof_cos_linear(int *profState,double *profResult,int timeIndex,int timeStart,int timeDuring,int timeStop,int shapeFactor);	
void prof_delay_cos_delay(int *profState,double *profResult,int timeIndex,int timeStart,int timeDuring,	int timeStop,int timeDelay_0,int timeDelay_1);
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
							double test_var[10]);
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
		double test_var[10]);

void init_kirk(void);
void Kirk_online(void);
void Kirk_online_BP(void);
void Kirk_con_test(void);
void Kirk_con_test_BP(void);
void Kirk_Posture_Init(void);
void Kirk_Posture_Init_BP(void);
void Kirk_Load_Posture_Init_BP(void); //inhyeok
void Go_Home_Posture(void);
void readZMP(double ORF_x, double ORF_y, double OLF_x, double OLF_y, double ORW_x, double ORW_y, double OLW_x, double OLW_y);
void readZMP_biped(double ORF_x, double ORF_y, double OLF_x, double OLF_y);
double P4_XZMP_INT_CON_E(double Ref_ZMP, double u, double ZMP, int zero);
double P4_YZMP_INT_CON_E(double Ref_ZMP, double u, double ZMP, int zero);
double P4_XZMP_CON_E(double u, double ZMP, int zero);
double P4_YZMP_CON_E(double u, double ZMP, int zero);
double P3H_XZMP_CON_E(double u, double ZMP, int zero);
double P3H_YZMP_CON_E(double u, double ZMP, int zero);
double P3F_XZMP_CON_E(double u, double ZMP, int zero);
double P3F_YZMP_CON_E(double u, double ZMP, int zero);
double P2_XZMP_CON_E(double u, double ZMP, int zero);
double P2_YZMP_CON_E(double u, double ZMP, int zero);
double P2_XZMP_INT_CON_E(double Ref_ZMP, double u, double ZMP, int zero);
double P2_YZMP_INT_CON_E(double Ref_ZMP, double u, double ZMP, int zero);
double P1_XZMP_CON_E(double u, double ZMP, int zero);
double P1_YZMP_CON_E(double u, double ZMP, int zero);
double FORCE_DIFF_CON_R1(double Ref_force, double force, int zero);
double FORCE_DIFF_CON_R2(double Ref_force, double force, int zero);
double TORQ_RAR_INT_CON(double Ref_Torq, double Torq, int zero);
double TORQ_LAR_INT_CON(double Ref_Torq, double Torq, int zero);
double TORQ_DIFF_RAP_INT_CON(double Ref_Torq, double Torq, int zero);
double k3H(double x, int R1_Wstage, int L1_Wstage);
double k3F(double x, int R1_Wstage, int L1_Wstage);
double k4(double x);
double Support_Value(double Fz, int Foot_Num);
void ZMP_CON_MANAGER(double R1_Load_N, double R2_Load_N, double L1_Load_N, double L2_Load_N, unsigned char R1_Wstage, unsigned char L1_Wstage, int zero);
void ZMP_CON_MANAGER_BP(double R1_Load_N, double L1_Load_N, unsigned char Land_ROK_Flag, unsigned char Land_LOK_Flag, int zero);
double Late_Land_Vertical_Disp(unsigned char joint_num, double Ref_Force, double Force, int zero);
double TP_INT_CON(double Ref_ANG, double ANG, int zero);
double TR_INT_CON(double Ref_ANG, double ANG, int zero);
double RLAR_TORQ_CON(double Torq, int zero);
double RLAP_TORQ_CON(double Torq, int zero);
double SSP_Y_VIB_CON_E_RF(double u, double Ang_Vel, int zero);
double SSP_Y_VIB_CON_E_LF(double u, double Ang_Vel, int zero);
double SSP_X_VIB_CON_E_RF(double u, double Ang_Vel, int zero); 
double SSP_X_VIB_CON_E_LF(double u, double Ang_Vel, int zero); 
double P1_XCOM_ZMP_CON_E(double ref_zmp, double zmp, double ref_cog, int zero);
double P1_YCOM_ZMP_CON_E(double ref_zmp, double zmp, double ref_cog, int zero);
double P2_XCOM_ZMP_CON_E(double ref_zmp, double zmp, double ref_cog, int zero);
double P2_YCOM_ZMP_CON_E(double ref_zmp, double zmp, double ref_cog, int zero);
void P1_Y_th_th_d_Observer(double u, double ZMP, int zero);
double SSP_Z_VIB_CON_E(double u, double Ang_Vel, int zero);
double P2_XZMP_CON(double ZMP, int zero);
double P3F_Y_VIB_CON_E_RF(double u, double Ang_Vel, int zero);
double P3F_Y_VIB_CON_E_LF(double u, double Ang_Vel, int zero);

#endif