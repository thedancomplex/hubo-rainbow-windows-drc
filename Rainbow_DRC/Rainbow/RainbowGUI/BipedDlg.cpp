// BipedDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "BipedDlg.h"
#include "..\SharedMemory.h"
#include "..\CommonDefinition.h"
#include <math.h>


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


extern PSHARED_DATA pSharedMemory;

/////////////////////////////////////////////////////////////////////////////
// CBipedDlg dialog


CBipedDlg::CBipedDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CBipedDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CBipedDlg)
	m_LS_KIRK = 0.0f;
	m_LSS_KIRK = 0.0f;
	m_HS_KIRK = 50.0f;
	m_ROT_ANG_KIRK = 0.0f;
	m_DELAY_R_KIRK = 0.01f;
	m_AB_KIRK = 62.0f;
	m_STEP_NUM_KIRK = 1;
	//}}AFX_DATA_INIT
}


void CBipedDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CBipedDlg)
	DDX_Text(pDX, IDC_EDIT_LS, m_LS_KIRK);
	DDX_Text(pDX, IDC_EDIT_LSS, m_LSS_KIRK);
	DDX_Text(pDX, IDC_EDIT_HS, m_HS_KIRK);
	DDX_Text(pDX, IDC_EDIT_ROTANG, m_ROT_ANG_KIRK);
	DDX_Text(pDX, IDC_EDIT_DELAY, m_DELAY_R_KIRK);
	DDX_Text(pDX, IDC_EDIT_AB, m_AB_KIRK);
	DDX_Text(pDX, IDC_EDIT_NSTEP, m_STEP_NUM_KIRK);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CBipedDlg, CDialog)
	//{{AFX_MSG_MAP(CBipedDlg)
	ON_BN_CLICKED(IDHOME2, OnHome2)
	ON_BN_CLICKED(ID_BIPED_READY, OnBipedReady)
	ON_BN_CLICKED(ID_IK_ON, OnIkOn)
	ON_BN_CLICKED(ID_KIRK_START2, OnKirkStart2)
	ON_BN_CLICKED(ID_KIRK_STOP2, OnKirkStop2)
	ON_BN_CLICKED(ID_EM_STOP, OnEmStop)
	ON_BN_CLICKED(ID_ZMP_CON_TEST, OnZmpConTest)
	ON_BN_CLICKED(ID_CONTEST_ON, OnContestOn)
	ON_BN_CLICKED(ID_CONTEST_OFF, OnContestOff)
	ON_BN_CLICKED(ID_KIRK_ZMP_INIT_START, OnKirkZmpInitStart)
	ON_BN_CLICKED(ID_KIRK_ZMP_INIT_STOP, OnKirkZmpInitStop)
	ON_BN_CLICKED(ID_KIRK_GOHOME, OnKirkGohome)
	ON_BN_CLICKED(ID_KIRK_STEP_NUMBER_GO, OnKirkStepNumberGo)
	ON_BN_CLICKED(ID_KIRK_CONTINU, OnKirkContinu)
	ON_BN_CLICKED(ID_KIRK_STOP3, OnKirkStop3)
	ON_BN_CLICKED(ID_KIRK_PARA, OnKirkPara)
	ON_BN_CLICKED(ID_KIRK_ZMP_INIT_LOAD, OnKirkZmpInitLoad)
	ON_BN_CLICKED(ID_KIRK_ZMP_INIT_SAVE, OnKirkZmpInitSave)
	ON_BN_CLICKED(IDC_ZMP, OnZmp)
	ON_BN_CLICKED(IDC_VIB, OnVib)
	ON_BN_CLICKED(IDC_EL, OnEl)
	ON_BN_CLICKED(IDC_UPRIGHT, OnUpright)
	ON_BN_CLICKED(IDC_ARM_COMPL, OnArmCompl)
	ON_BN_CLICKED(ID_STRIDE0, OnStride0)
	ON_BN_CLICKED(ID_STRIDE1, OnStride1)
	ON_BN_CLICKED(ID_STRIDE2, OnStride2)
	ON_BN_CLICKED(ID_STRIDE3, OnStride3)
	ON_BN_CLICKED(ID_STRIDE4, OnStride4)
	ON_BN_CLICKED(ID_STRIDE5, OnStride5)
	ON_BN_CLICKED(ID_STRIDE6, OnStride6)
	ON_BN_CLICKED(ID_ROT_0, OnRot0)
	ON_BN_CLICKED(ID_ROT4, OnRot4)
	ON_BN_CLICKED(ID_ROT5, OnRot5)
	ON_BN_CLICKED(ID_ROT6, OnRot6)
	ON_BN_CLICKED(ID_ROT1, OnRot1)
	ON_BN_CLICKED(ID_ROT2, OnRot2)
	ON_BN_CLICKED(ID_ROT3, OnRot3)
	ON_BN_CLICKED(ID_SIDE0, OnSide0)
	ON_BN_CLICKED(ID_SIDE3, OnSide3)
	ON_BN_CLICKED(ID_SIDE4, OnSide4)
	ON_BN_CLICKED(ID_SIDE1, OnSide1)
	ON_BN_CLICKED(ID_SIDE2, OnSide2)
	ON_BN_CLICKED(ID_ROT7, OnRot7)
	ON_BN_CLICKED(ID_ROT8, OnRot8)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CBipedDlg message handlers

BOOL CBipedDlg::OnInitDialog() 
{
	CDialog::OnInitDialog();
	
	CString str_temp;

	str_temp.Format("%g",10.f);
	GetDlgItem(IDC_XZMP_OFFSET)->SetWindowText(str_temp);
	pSharedMemory->xzmp_offset = 10.f;
			
	CheckDlgButton(IDC_ZMP,TRUE);
	CheckDlgButton(IDC_VIB,TRUE);
	CheckDlgButton(IDC_EL,TRUE);
	CheckDlgButton(IDC_UPRIGHT,TRUE);
	CheckDlgButton(IDC_ARM_COMPL,TRUE);
	
	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}

void CBipedDlg::OnHome2() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
		pSharedMemory->CommandFlag = GOTO_HOME_POS;		
	else 
		AfxMessageBox("Other Command is activated..!!");	
}

void CBipedDlg::OnBipedReady() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
		pSharedMemory->CommandFlag = GOTO_DRC_BIPED_READY_POS;		
	else 
		AfxMessageBox("Other Command is activated..!!");	
}

void CBipedDlg::OnIkOn() 
{
	int i;
	FILE *fp;

	BOOL chk;

	chk = IsDlgButtonChecked(IDC_ZMP);

	if(chk == TRUE)
		pSharedMemory->ON_ZMP_flag = 1;
	else
		pSharedMemory->ON_ZMP_flag = 0;
	
	chk = IsDlgButtonChecked(IDC_VIB);

	if(chk == TRUE)
		pSharedMemory->ON_vib_flag = 1;
	else
		pSharedMemory->ON_vib_flag = 0;

	chk = IsDlgButtonChecked(IDC_EL);

	if(chk == TRUE)
		pSharedMemory->ON_EL_flag = 1;
	else
		pSharedMemory->ON_EL_flag = 0;
	
	chk = IsDlgButtonChecked(IDC_UPRIGHT);

	if(chk == TRUE)
		pSharedMemory->ON_upright_flag = 1;
	else
		pSharedMemory->ON_upright_flag = 0;

	chk = IsDlgButtonChecked(IDC_ARM_COMPL);

	if(chk == TRUE)
		pSharedMemory->ON_compl_arm_flag = 1;
	else
		pSharedMemory->ON_compl_arm_flag = 0;

	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)
	{
		pSharedMemory->temp_fHand_threshold = 50.f;

		if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\gain_cloop_biped.txt","r")) == NULL )
		{
			AfxMessageBox("Data file 'gain_cloop_biped.txt' was not found.");		
			return;
		}
		for(i=0;i<26;i++)
		{
			fscanf(fp,"%f %f", &(pSharedMemory->kp[i]), &(pSharedMemory->kd[i]));
			while(fgetc(fp) != '\n');
		}
		fclose(fp);

		pSharedMemory->kirk_flag = 0;
		pSharedMemory->position_mode_flag = 0;
		pSharedMemory->torque_mode_flag = 0;
		pSharedMemory->stop_online_biped_flag = 0;
		pSharedMemory->biped_flag = 0;    //-2 는 한발지지  0 은 양발지지
		//pSharedMemory->sw_mode = SW_MODE_NON_COMPLEMENTARY;
		//pSharedMemory->CommandFlag = SET_SW_MODE;
		//Sleep(200);
		pSharedMemory->WB_DemoNo = WB_BIPED_KIRK;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;

		AfxMessageBox("Ready to biped walking..!!");
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CBipedDlg::OnKirkStart2() 
{
	CString str_temp;

	if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_BIPED_KIRK)
	{
		GetDlgItem(IDC_XZMP_OFFSET)->GetWindowText(str_temp);
		pSharedMemory->xzmp_offset = (int)atof(str_temp);
		if(pSharedMemory->xzmp_offset > 40.f)
			pSharedMemory->xzmp_offset = 40.f;
		else if(pSharedMemory->xzmp_offset < -40.f)
			pSharedMemory->xzmp_offset = -40.f;

		if(pSharedMemory->biped_flag == 0 || pSharedMemory->biped_flag == -2)
			pSharedMemory->init_kirk_flag = 1;
	
		pSharedMemory->stop_online_biped_flag = 0;
		pSharedMemory->kirk_flag = 4;
		pSharedMemory->biped_flag = 1;		
	}
	else
		AfxMessageBox("Not in the biped mode..!!");
	
}

void CBipedDlg::OnKirkStop2() 
{
	if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_BIPED_KIRK && pSharedMemory->biped_flag == 1)
		pSharedMemory->stop_online_biped_flag = 1;
	pSharedMemory->kirk_flag = 0;	
}

void CBipedDlg::OnEmStop() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
		pSharedMemory->CommandFlag = EMERGENCY_STOP;	
}

void CBipedDlg::OnZmpConTest() 
{
	if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_BIPED_KIRK)
	{
		if(pSharedMemory->biped_flag == 0 || pSharedMemory->biped_flag == -2)
			pSharedMemory->init_kirk_flag = 1;
		
		pSharedMemory->kirk_flag = 3;	
		pSharedMemory->P4_ZMP_CON_CNT = 0;
		pSharedMemory->P4_ZMP_CON_Flag = 1;
		pSharedMemory->biped_flag = 1;				
	}
	else
		AfxMessageBox("Other Command is activated..!!");	
}

void CBipedDlg::OnContestOn() 
{
	if(pSharedMemory->WB_DemoNo==WB_BIPED_KIRK)
		pSharedMemory->P4_ZMP_CON_Flag = 1;	
}

void CBipedDlg::OnContestOff() 
{
	if(pSharedMemory->WB_DemoNo==WB_BIPED_KIRK)
		pSharedMemory->P4_ZMP_CON_Flag = 0;
}

void CBipedDlg::OnKirkZmpInitStart() 
{
	CString str_temp;

	if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_BIPED_KIRK)
	{
		GetDlgItem(IDC_XZMP_OFFSET)->GetWindowText(str_temp);
		pSharedMemory->xzmp_offset = (int)atof(str_temp);
		if(pSharedMemory->xzmp_offset > 40.f)
			pSharedMemory->xzmp_offset = 40.f;
		else if(pSharedMemory->xzmp_offset < -40.f)
			pSharedMemory->xzmp_offset = -40.f;
			
		if(pSharedMemory->biped_flag == 0)
			pSharedMemory->init_kirk_flag = 1;
		
		pSharedMemory->kirk_flag = 1;
		pSharedMemory->biped_flag = 1;				
	}
	else
		AfxMessageBox("Other Command is activated..!!");					
}

void CBipedDlg::OnKirkZmpInitStop() 
{
	if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_BIPED_KIRK && pSharedMemory->biped_flag == 1 && pSharedMemory->kirk_flag == 1)
		pSharedMemory->kirk_flag = 0;
	else
		AfxMessageBox("Not in the init-ZMP mode!!");	
}

void CBipedDlg::OnKirkGohome() 
{
	if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_BIPED_KIRK && pSharedMemory->biped_flag == 1)
		pSharedMemory->kirk_flag = 2;
	else
		AfxMessageBox("Other Command is activated..!!");			
}

void CBipedDlg::OnKirkStepNumberGo() 
{
UpdateData(true);

	if(m_LSS_KIRK*m_ROT_ANG_KIRK < 0)
	{
		m_LSS_KIRK = 0.f;
		m_ROT_ANG_KIRK = 0.f;
	}

    pSharedMemory->Global_Ls = m_LS_KIRK;
	pSharedMemory->Global_Lss = m_LSS_KIRK;
	pSharedMemory->Global_Hs = m_HS_KIRK;
	pSharedMemory->Global_Rot_Ang = m_ROT_ANG_KIRK;
	pSharedMemory->Global_Delay_Ratio = m_DELAY_R_KIRK;
	pSharedMemory->Global_Ab = m_AB_KIRK;

	pSharedMemory->Step_Number = m_STEP_NUM_KIRK;

    if(pSharedMemory->Go_flag == 0){
		pSharedMemory->First_flag = 1;
		pSharedMemory->Go_flag = 1;
    }
	UpdateData(false);
	pSharedMemory->Change_OK = 0;		
}

void CBipedDlg::OnKirkContinu() 
{
	UpdateData(true);

	if(m_LSS_KIRK*m_ROT_ANG_KIRK < 0)
	{
		m_LSS_KIRK = 0.f;
		m_ROT_ANG_KIRK = 0.f;
	}

    pSharedMemory->Global_Ls = m_LS_KIRK;
	pSharedMemory->Global_Lss = m_LSS_KIRK;
	pSharedMemory->Global_Hs = m_HS_KIRK;
	pSharedMemory->Global_Rot_Ang = m_ROT_ANG_KIRK;
	pSharedMemory->Global_Delay_Ratio = m_DELAY_R_KIRK;
	pSharedMemory->Global_Ab = m_AB_KIRK;

	pSharedMemory->Step_Number = 1000;

	if(pSharedMemory->Go_flag == 0){
		pSharedMemory->First_flag = 1;
		pSharedMemory->Go_flag = 1;
    }

	UpdateData(false);
	pSharedMemory->Change_OK = 0;
	
}

void CBipedDlg::OnKirkStop3() 
{
	pSharedMemory->Go_flag = 0;
	
}

void CBipedDlg::OnKirkPara() 
{
	UpdateData(true);

	if(m_LSS_KIRK*m_ROT_ANG_KIRK < 0)
	{
		m_LSS_KIRK = 0.f;
		m_ROT_ANG_KIRK = 0.f;
	}

	if(pSharedMemory->Change_OK == 1){
			
		pSharedMemory->Global_Ls = m_LS_KIRK;
		pSharedMemory->Global_Lss = m_LSS_KIRK;
		pSharedMemory->Global_Hs = m_HS_KIRK;
		pSharedMemory->Global_Rot_Ang = m_ROT_ANG_KIRK;
		pSharedMemory->Global_Delay_Ratio = m_DELAY_R_KIRK;
		pSharedMemory->Global_Ab = m_AB_KIRK;

		pSharedMemory->Change_OK = 0;

	}
	UpdateData(false);
}

void CBipedDlg::OnKirkZmpInitLoad() 
{
	FILE *fp;

	fp = fopen("C:\\Rainbow_DRC\\WB_Data\\InitZMPData.txt","r");

	if(fp == NULL)
	{
		AfxMessageBox("C:\\Rainbow_DRC\\WB_Data\\InitZMPData.txt is not found.");			
		return;
	}
	else
	{
		fscanf(fp, "%f", &pSharedMemory->Del_RAP_Init);
		fscanf(fp, "%f", &pSharedMemory->Del_RAR_Init);
		fscanf(fp, "%f", &pSharedMemory->Del_LAR_Init);
		fscanf(fp, "%f", &pSharedMemory->Del_RLHP_Init);
		fscanf(fp, "%f", &pSharedMemory->BC_X_Init);
		fscanf(fp, "%f", &pSharedMemory->BC_Y_Init);
		fscanf(fp, "%f", &pSharedMemory->Del_RLFZ_Init);

		fclose(fp);
	}

	if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_BIPED_KIRK)
	{
		if(pSharedMemory->biped_flag == 0)
			pSharedMemory->init_kirk_flag = 1;
		
		pSharedMemory->kirk_flag = 5;
		pSharedMemory->biped_flag = 1;				
	}
	else
		AfxMessageBox("Other Command is activated..!!");			
	
}

void CBipedDlg::OnKirkZmpInitSave() 
{

	FILE *fp;
	
	fp = fopen("C:\\Rainbow_DRC\\WB_Data\\InitZMPData.txt","w");
	
	fprintf(fp, "%f \n", pSharedMemory->Del_RAP_Init);
	fprintf(fp, "%f \n", pSharedMemory->Del_RAR_Init);
	fprintf(fp, "%f \n", pSharedMemory->Del_LAR_Init);
	fprintf(fp, "%f \n", pSharedMemory->Del_RLHP_Init);
	
	fprintf(fp, "%f \n", pSharedMemory->BC_X_Init);
	fprintf(fp, "%f \n", pSharedMemory->BC_Y_Init);
	fprintf(fp, "%f \n", pSharedMemory->Del_RLFZ_Init);

	fclose(fp);
}

void CBipedDlg::OnZmp() 
{
	BOOL chk = IsDlgButtonChecked(IDC_ZMP);

	if(chk == TRUE)
		pSharedMemory->ON_ZMP_flag = 1;
	else
		pSharedMemory->ON_ZMP_flag = 0;
	
}

void CBipedDlg::OnVib() 
{
	BOOL chk = IsDlgButtonChecked(IDC_VIB);

	if(chk == TRUE)
		pSharedMemory->ON_vib_flag = 1;
	else
		pSharedMemory->ON_vib_flag = 0;	
}

void CBipedDlg::OnEl() 
{
	BOOL chk = IsDlgButtonChecked(IDC_EL);

	if(chk == TRUE)
		pSharedMemory->ON_EL_flag = 1;
	else
		pSharedMemory->ON_EL_flag = 0;	
}

void CBipedDlg::OnUpright() 
{
	BOOL chk = IsDlgButtonChecked(IDC_UPRIGHT);

	if(chk == TRUE)
		pSharedMemory->ON_upright_flag = 1;
	else
		pSharedMemory->ON_upright_flag = 0;	
}

void CBipedDlg::OnArmCompl() 
{
	BOOL chk = IsDlgButtonChecked(IDC_ARM_COMPL);

	if(chk == TRUE)
		pSharedMemory->ON_compl_arm_flag = 1;
	else
		pSharedMemory->ON_compl_arm_flag = 0;	
}

void CBipedDlg::OnStride0() 
{
	CString str_temp;
	m_LS_KIRK = 0.0f;
	str_temp.Format("%g",m_LS_KIRK);
	GetDlgItem(IDC_EDIT_LS)->SetWindowText(str_temp);
	
}

void CBipedDlg::OnStride1() 
{
	CString str_temp;
	m_LS_KIRK = 100.f;
	str_temp.Format("%g",m_LS_KIRK);
	GetDlgItem(IDC_EDIT_LS)->SetWindowText(str_temp);	
}

void CBipedDlg::OnStride2() 
{
	CString str_temp;
	m_LS_KIRK = 150.f;
	str_temp.Format("%g",m_LS_KIRK);
	GetDlgItem(IDC_EDIT_LS)->SetWindowText(str_temp);	
}

void CBipedDlg::OnStride3() 
{
	CString str_temp;
	m_LS_KIRK = 200.f;
	str_temp.Format("%g",m_LS_KIRK);
	GetDlgItem(IDC_EDIT_LS)->SetWindowText(str_temp);	
}

void CBipedDlg::OnStride4() 
{
	CString str_temp;
	m_LS_KIRK = -100.f;
	str_temp.Format("%g",m_LS_KIRK);
	GetDlgItem(IDC_EDIT_LS)->SetWindowText(str_temp);	
}

void CBipedDlg::OnStride5() 
{
	CString str_temp;
	m_LS_KIRK = -150.f;
	str_temp.Format("%g",m_LS_KIRK);
	GetDlgItem(IDC_EDIT_LS)->SetWindowText(str_temp);	
}

void CBipedDlg::OnStride6() 
{
	CString str_temp;
	m_LS_KIRK = -200.f;
	str_temp.Format("%g",m_LS_KIRK);
	GetDlgItem(IDC_EDIT_LS)->SetWindowText(str_temp);	
}

void CBipedDlg::OnRot0() 
{
	CString str_temp;
	m_LSS_KIRK = 0.0f;
	m_ROT_ANG_KIRK = 0.f;
	str_temp.Format("%g",m_LSS_KIRK);
	GetDlgItem(IDC_EDIT_LSS)->SetWindowText(str_temp);
	str_temp.Format("%g",m_ROT_ANG_KIRK);
	GetDlgItem(IDC_EDIT_ROTANG)->SetWindowText(str_temp);	
}

void CBipedDlg::OnRot4() 
{
	CString str_temp;
	m_LSS_KIRK = 20.0f;
	m_ROT_ANG_KIRK = 10.f;
	str_temp.Format("%g",m_LSS_KIRK);
	GetDlgItem(IDC_EDIT_LSS)->SetWindowText(str_temp);
	str_temp.Format("%g",m_ROT_ANG_KIRK);
	GetDlgItem(IDC_EDIT_ROTANG)->SetWindowText(str_temp);	
}

void CBipedDlg::OnRot5() 
{
	CString str_temp;
	m_LSS_KIRK = 40.0f;
	m_ROT_ANG_KIRK = 30.f;
	str_temp.Format("%g",m_LSS_KIRK);
	GetDlgItem(IDC_EDIT_LSS)->SetWindowText(str_temp);
	str_temp.Format("%g",m_ROT_ANG_KIRK);
	GetDlgItem(IDC_EDIT_ROTANG)->SetWindowText(str_temp);	
}

void CBipedDlg::OnRot6() 
{
	CString str_temp;
	m_LSS_KIRK = 50.0f;
	m_ROT_ANG_KIRK = 40.f;
	str_temp.Format("%g",m_LSS_KIRK);
	GetDlgItem(IDC_EDIT_LSS)->SetWindowText(str_temp);
	str_temp.Format("%g",m_ROT_ANG_KIRK);
	GetDlgItem(IDC_EDIT_ROTANG)->SetWindowText(str_temp);	
}

void CBipedDlg::OnRot1() 
{
	CString str_temp;
	m_LSS_KIRK = -20.0f;
	m_ROT_ANG_KIRK = -10.f;
	str_temp.Format("%g",m_LSS_KIRK);
	GetDlgItem(IDC_EDIT_LSS)->SetWindowText(str_temp);
	str_temp.Format("%g",m_ROT_ANG_KIRK);
	GetDlgItem(IDC_EDIT_ROTANG)->SetWindowText(str_temp);	
}

void CBipedDlg::OnRot2() 
{
	CString str_temp;
	m_LSS_KIRK = -40.0f;
	m_ROT_ANG_KIRK = -30.f;
	str_temp.Format("%g",m_LSS_KIRK);
	GetDlgItem(IDC_EDIT_LSS)->SetWindowText(str_temp);
	str_temp.Format("%g",m_ROT_ANG_KIRK);
	GetDlgItem(IDC_EDIT_ROTANG)->SetWindowText(str_temp);	
}

void CBipedDlg::OnRot3() 
{
	CString str_temp;
	m_LSS_KIRK = -50.0f;
	m_ROT_ANG_KIRK = -40.f;
	str_temp.Format("%g",m_LSS_KIRK);
	GetDlgItem(IDC_EDIT_LSS)->SetWindowText(str_temp);
	str_temp.Format("%g",m_ROT_ANG_KIRK);
	GetDlgItem(IDC_EDIT_ROTANG)->SetWindowText(str_temp);	
}

void CBipedDlg::OnSide0() 
{
	CString str_temp;
	m_LSS_KIRK = 0.0f;
	
	str_temp.Format("%g",m_LSS_KIRK);
	GetDlgItem(IDC_EDIT_LSS)->SetWindowText(str_temp);	
}

void CBipedDlg::OnSide3() 
{
	CString str_temp;
	m_LSS_KIRK = 30.0f;
	
	str_temp.Format("%g",m_LSS_KIRK);
	GetDlgItem(IDC_EDIT_LSS)->SetWindowText(str_temp);	
}

void CBipedDlg::OnSide4() 
{
	CString str_temp;
	m_LSS_KIRK = 60.0f;
	
	str_temp.Format("%g",m_LSS_KIRK);
	GetDlgItem(IDC_EDIT_LSS)->SetWindowText(str_temp);	
}

void CBipedDlg::OnSide1() 
{
	CString str_temp;
	m_LSS_KIRK = -30.0f;
	
	str_temp.Format("%g",m_LSS_KIRK);
	GetDlgItem(IDC_EDIT_LSS)->SetWindowText(str_temp);	
}

void CBipedDlg::OnSide2() 
{
	CString str_temp;
	m_LSS_KIRK = -60.0f;
	
	str_temp.Format("%g",m_LSS_KIRK);
	GetDlgItem(IDC_EDIT_LSS)->SetWindowText(str_temp);	
}

void CBipedDlg::OnRot7() 
{
	CString str_temp;
	m_ROT_ANG_KIRK += 1.f;
	str_temp.Format("%g", m_LSS_KIRK);
	GetDlgItem(IDC_EDIT_LSS)->SetWindowText(str_temp);
	str_temp.Format("%g",m_ROT_ANG_KIRK);
	GetDlgItem(IDC_EDIT_ROTANG)->SetWindowText(str_temp);	

	OnKirkPara();
}

void CBipedDlg::OnRot8() 
{
	CString str_temp;
	m_ROT_ANG_KIRK -= 1.f;
	str_temp.Format("%g", m_LSS_KIRK);
	GetDlgItem(IDC_EDIT_LSS)->SetWindowText(str_temp);
	str_temp.Format("%g",m_ROT_ANG_KIRK);
	GetDlgItem(IDC_EDIT_ROTANG)->SetWindowText(str_temp);	

	OnKirkPara();
	
}
