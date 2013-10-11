// DrcQuadDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "DrcQuadDlg.h"
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
// CDrcQuadDlg dialog


CDrcQuadDlg::CDrcQuadDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CDrcQuadDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CDrcQuadDlg)
	m_LS_KIRK = 0.0;
	m_LSS_KIRK = 0.0;
	m_ROT_ANG_KIRK = 0.0;
	//m_DELAY_R_KIRK = 0.143;
	//m_AB_KIRK = 58.0;
	//m_HS_KIRK = 50.0;
	m_DELAY_R_KIRK = 0.1;
	m_AB_KIRK = 40.0;
	m_HS_KIRK = 60.0;
	m_STEP_NUM_KIRK = 1;
	//}}AFX_DATA_INIT
}


void CDrcQuadDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CDrcQuadDlg)
	DDX_Text(pDX, IDC_LS_KIRK, m_LS_KIRK);
	DDX_Text(pDX, IDC_LSS_KIRK, m_LSS_KIRK);
	DDX_Text(pDX, IDC_ROT_ANG_KIRK, m_ROT_ANG_KIRK);
	DDX_Text(pDX, IDC_DELAY_R_KIRK, m_DELAY_R_KIRK);
	DDX_Text(pDX, IDC_AB_KIRK, m_AB_KIRK);
	DDX_Text(pDX, IDC_HS_KIRK, m_HS_KIRK);
	DDX_Text(pDX, IDC_STEP_NUM_KIRK, m_STEP_NUM_KIRK);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CDrcQuadDlg, CDialog)
	//{{AFX_MSG_MAP(CDrcQuadDlg)
	ON_BN_CLICKED(ID_READY_BIPED, OnReadyBiped)
	ON_BN_CLICKED(ID_HOME, OnHome)
	ON_BN_CLICKED(ID_TRANSFORM, OnTransform)
	ON_BN_CLICKED(ID_KIRK_START, OnKirkStart)
	ON_BN_CLICKED(ID_TRANSFORM_BACK, OnTransformBack)
	ON_BN_CLICKED(ID_KIRK_STOP, OnKirkStop)
	ON_BN_CLICKED(ID_STOP, OnStop)
	ON_BN_CLICKED(ID_HAND_YAW, OnHandYaw0Deg)
	ON_BN_CLICKED(ID_HAND_YAW2, OnHandYaw90Deg)
	ON_BN_CLICKED(ID_HAND_YAW3, OnHandMode)
	ON_BN_CLICKED(ID_HAND_YAW4, OnStickMode)
	ON_BN_CLICKED(IDC_STEP_GO_KIRK, OnStepGoKirk)
	ON_BN_CLICKED(IDC_CONTINU_GO_KIRK, OnContinuGoKirk)
	ON_BN_CLICKED(IDC_STOP_KIRK, OnStopKirk)
	ON_BN_CLICKED(IDC_PARA_CHANGE, OnParaChange)
	ON_BN_CLICKED(IDC_KIRK_ZMP_TEST, OnKirkZmpTest)
	ON_BN_CLICKED(IDC_KIRK_ZMP_CON_ON, OnKirkZmpConOn)
	ON_BN_CLICKED(IDC_KIRK_ZMP_CON_OFF, OnKirkZmpConOff)
	ON_BN_CLICKED(IDC_KIRK_ZMP_INIT_START, OnKirkZmpInitStart)
	ON_BN_CLICKED(IDC_KIRK_ZMP_INIT_STOP, OnKirkZmpInitStop)
	ON_BN_CLICKED(IDC_KIRK_GO_HOME, OnKirkGoHome)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDrcQuadDlg message handlers


void CDrcQuadDlg::OnReadyBiped() 
{
	int temp;
	CString str_temp;
	
	GetDlgItem(IDC_BIPED_MODE)->GetWindowText(str_temp);
		temp = (int)atoi(str_temp);
	if(pSharedMemory->CommandFlag == NO_ACT)
		if(temp == 0)
			pSharedMemory->CommandFlag = GOTO_DRC_BIPED_READY_POS;		
		else if(temp == 1)
			pSharedMemory->CommandFlag = GOTO_DRC_WIDE_BIPED_READY_POS;
		else
			AfxMessageBox("Invalid number!! (0:normal, 1:wide)");
	else 
		AfxMessageBox("Other Command is activated..!!");
		
}

void CDrcQuadDlg::OnHome() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
		pSharedMemory->CommandFlag = GOTO_HOME_POS;		
	else 
		AfxMessageBox("Other Command is activated..!!");
	
}

void CDrcQuadDlg::OnTransform() 
{
	FILE *fp;
	int temp;
	CString str_temp;
	int i;

	GetDlgItem(ID_TRANSFORM)->EnableWindow(FALSE);
	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0 && pSharedMemory->change_drc_hand_flag == DRC_STICK_MODE)		// by Inhyeok
	{
		pSharedMemory->temp_fHand_threshold = 50.f;

		if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\gain_cloop_quad.txt","r")) == NULL )
		{
			AfxMessageBox("Data file 'gain_cloop_quad.txt' was not found.");		
			return;
		}
		for(i=0;i<26;i++)
		{
			fscanf(fp,"%f %f", &(pSharedMemory->kp[i]), &(pSharedMemory->kd[i]));
			while(fgetc(fp) != '\n');
		}
		fclose(fp);

		if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\transform_quad.txt","r")) == NULL )
		{
			AfxMessageBox("Data file 'transform_quad.txt' was not found.");		
			return;
		}
		fscanf(fp,"%f", &pSharedMemory->wb_data[0]);
		while(fgetc(fp) != '\n');
		fscanf(fp,"%f", &pSharedMemory->wb_data[1]);
		while(fgetc(fp) != '\n');
		fscanf(fp,"%f", &pSharedMemory->wb_data[2]);
		while(fgetc(fp) != '\n');
		fscanf(fp,"%f %f", &pSharedMemory->wb_data[3], &pSharedMemory->wb_data[4]);
		while(fgetc(fp) != '\n');
		fscanf(fp,"%f %f", &pSharedMemory->wb_data[5], &pSharedMemory->wb_data[6]);
		while(fgetc(fp) != '\n');
		fscanf(fp,"%f %f", &pSharedMemory->wb_data[7], &pSharedMemory->wb_data[8]);
		while(fgetc(fp) != '\n');
		fscanf(fp,"%f", &pSharedMemory->wb_data[9]);
		while(fgetc(fp) != '\n');
		fscanf(fp,"%f %f", &pSharedMemory->wb_data[10], &pSharedMemory->wb_data[11]);
		while(fgetc(fp) != '\n');
		fscanf(fp,"%f", &pSharedMemory->wb_data[12]);
		fclose(fp);

		GetDlgItem(IDC_QUAD_MODE)->GetWindowText(str_temp);
		temp = (int)atoi(str_temp);

		switch(temp)
		{
		case 1:
			pSharedMemory->quad_flag = 1;
			break;
		case 2:
			pSharedMemory->quad_flag = 2;
			break;
		default:
			pSharedMemory->quad_flag = 0;
			break;
		}
		pSharedMemory->transform_flag = 0;
		pSharedMemory->position_mode_flag = 0;
		pSharedMemory->torque_mode_flag = 0;
		pSharedMemory->stop_online_quad_flag = 0;
		pSharedMemory->sw_mode = SW_MODE_NON_COMPLEMENTARY;
		pSharedMemory->CommandFlag = SET_SW_MODE;
		Sleep(200);
		pSharedMemory->WB_DemoNo = WB_QUAD_KIRK;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
	GetDlgItem(ID_TRANSFORM)->EnableWindow(TRUE);	
}

void CDrcQuadDlg::OnKirkStart() 
{
	int i;
	FILE *fp;
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

		pSharedMemory->position_mode_flag = 0;
		pSharedMemory->torque_mode_flag = 0;
		pSharedMemory->stop_online_biped_flag = 0;
		pSharedMemory->biped_flag = 0;    //-2 는 한발지지  0 은 양발지지
		pSharedMemory->sw_mode = SW_MODE_NON_COMPLEMENTARY;
		pSharedMemory->CommandFlag = SET_SW_MODE;
		Sleep(200);
		pSharedMemory->WB_DemoNo = WB_BIPED_KIRK;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;

		AfxMessageBox("Ready to biped walking..!!");
	}
	else if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_QUAD_KIRK && (pSharedMemory->transform_flag == 3 || pSharedMemory->transform_flag == 4))
	{
		if(pSharedMemory->transform_flag == 3)
			pSharedMemory->init_kirk_flag = 1;

		pSharedMemory->kirk_flag = 4;
		pSharedMemory->transform_flag = 4;		
	}
	else if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_BIPED_KIRK)
	{
		if(pSharedMemory->biped_flag == 0 || pSharedMemory->biped_flag == -2)
			pSharedMemory->init_kirk_flag = 1;
	
		pSharedMemory->stop_online_biped_flag = 0;
		pSharedMemory->kirk_flag = 4;
		pSharedMemory->biped_flag = 1;		
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	
}

void CDrcQuadDlg::OnTransformBack() 
{
	GetDlgItem(ID_TRANSFORM_BACK)->EnableWindow(FALSE);
	if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_QUAD_KIRK && pSharedMemory->transform_flag == 3)
		pSharedMemory->transform_flag = 5;		
	else
		AfxMessageBox("Not in the quad mode..!!");
	GetDlgItem(ID_TRANSFORM_BACK)->EnableWindow(TRUE);
	
}

void CDrcQuadDlg::OnKirkStop() 
{
	if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_QUAD_KIRK && pSharedMemory->transform_flag == 4)
		pSharedMemory->stop_online_quad_flag = 1;	
	else if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_BIPED_KIRK && pSharedMemory->biped_flag == 1)
		pSharedMemory->stop_online_biped_flag = 1;
	pSharedMemory->kirk_flag = 0;
}

void CDrcQuadDlg::OnStop() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
		pSharedMemory->CommandFlag = EMERGENCY_STOP;
	
}

void CDrcQuadDlg::OnHandYaw0Deg() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->change_drc_hand_flag != DRC_STICK_MODE)
		{
			pSharedMemory->change_drc_hand_flag = DRC_ZERO_WRIST_YAW2;
			pSharedMemory->CommandFlag = CHANGE_DRC_HAND;		
		}
		else
			AfxMessageBox("Hands are in the stick mode!!");	
	}
	else 
		AfxMessageBox("Other Command is activated..!!");	
}

void CDrcQuadDlg::OnHandYaw90Deg() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->change_drc_hand_flag != DRC_STICK_MODE)
		{
			pSharedMemory->change_drc_hand_flag = DRC_90DEG_WRIST_YAW2;
			pSharedMemory->CommandFlag = CHANGE_DRC_HAND;		
		}
		else
			AfxMessageBox("Hands are in the stick mode!!");	
	}
	else 
		AfxMessageBox("Other Command is activated..!!");		
}

void CDrcQuadDlg::OnHandMode() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->change_drc_hand_flag = DRC_HAND_MODE;
		pSharedMemory->CommandFlag = CHANGE_DRC_HAND;		
	}
	else 
		AfxMessageBox("Other Command is activated..!!");
}

void CDrcQuadDlg::OnStickMode() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->change_drc_hand_flag == DRC_90DEG_WRIST_YAW2)
		{
			pSharedMemory->change_drc_hand_flag = DRC_STICK_MODE;
			pSharedMemory->CommandFlag = CHANGE_DRC_HAND;		
		}
		else
			AfxMessageBox("Wrist-yaw is not in the position!!");
	}
	else 
		AfxMessageBox("Other Command is activated..!!");	
}

void CDrcQuadDlg::OnStepGoKirk() 
{
	// TODO: Add your control notification handler code here
	UpdateData(true);

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

	pSharedMemory->Change_OK = 0;

}

void CDrcQuadDlg::OnContinuGoKirk() 
{
	// TODO: Add your control notification handler code here
	UpdateData(true);

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

	pSharedMemory->Change_OK = 0;
	
}

void CDrcQuadDlg::OnStopKirk() 
{
	// TODO: Add your control notification handler code here
	pSharedMemory->Go_flag = 0;
	
}

void CDrcQuadDlg::OnParaChange() 
{
	// TODO: Add your control notification handler code here
	UpdateData(true);

	if(pSharedMemory->Change_OK == 1){
			
		pSharedMemory->Global_Ls = m_LS_KIRK;
		pSharedMemory->Global_Lss = m_LSS_KIRK;
		pSharedMemory->Global_Hs = m_HS_KIRK;
		pSharedMemory->Global_Rot_Ang = m_ROT_ANG_KIRK;
		pSharedMemory->Global_Delay_Ratio = m_DELAY_R_KIRK;
		pSharedMemory->Global_Ab = m_AB_KIRK;

		pSharedMemory->Change_OK = 0;

	}
	
}

void CDrcQuadDlg::OnKirkZmpTest() 
{
	if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_QUAD_KIRK && (pSharedMemory->transform_flag == 3 || pSharedMemory->transform_flag == 4))
	{
		if(pSharedMemory->transform_flag == 3)
			pSharedMemory->init_kirk_flag = 1;
		
		pSharedMemory->kirk_flag = 3;
		pSharedMemory->P4_ZMP_CON_CNT = 0;
		pSharedMemory->P4_ZMP_CON_Flag = 1;
	
		pSharedMemory->transform_flag = 4;				
	}
	else if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_BIPED_KIRK)
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

void CDrcQuadDlg::OnKirkZmpConOn() 
{
	if(pSharedMemory->WB_DemoNo==WB_QUAD_KIRK)
		pSharedMemory->P4_ZMP_CON_Flag = 1;
	else if(pSharedMemory->WB_DemoNo==WB_BIPED_KIRK)
		pSharedMemory->P4_ZMP_CON_Flag = 1;
}

void CDrcQuadDlg::OnKirkZmpConOff() 
{
	if(pSharedMemory->WB_DemoNo==WB_QUAD_KIRK)
		pSharedMemory->P4_ZMP_CON_Flag = 0;
	else if(pSharedMemory->WB_DemoNo==WB_BIPED_KIRK)
		pSharedMemory->P4_ZMP_CON_Flag = 0;
	
}

void CDrcQuadDlg::OnKirkZmpInitStart() 
{
	if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_QUAD_KIRK && (pSharedMemory->transform_flag == 3 || pSharedMemory->transform_flag == 4))
	{
		if(pSharedMemory->transform_flag == 3)
			pSharedMemory->init_kirk_flag = 1;
		
		pSharedMemory->kirk_flag = 1;
		pSharedMemory->transform_flag = 4;				
	}
	else if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_BIPED_KIRK)
	{
		if(pSharedMemory->biped_flag == 0)
			pSharedMemory->init_kirk_flag = 1;
		
		pSharedMemory->kirk_flag = 1;
		pSharedMemory->biped_flag = 1;				
	}
	else
		AfxMessageBox("Other Command is activated..!!");		
}

void CDrcQuadDlg::OnKirkZmpInitStop() 
{
	if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_QUAD_KIRK && pSharedMemory->transform_flag == 4 && pSharedMemory->kirk_flag == 1)
		pSharedMemory->kirk_flag = 0;
	else if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_BIPED_KIRK && pSharedMemory->biped_flag == 1 && pSharedMemory->kirk_flag == 1)
		pSharedMemory->kirk_flag = 0;
	else
		AfxMessageBox("Not in the init-ZMP mode!!");		
}

void CDrcQuadDlg::OnKirkGoHome() 
{
	if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_QUAD_KIRK && pSharedMemory->transform_flag == 4)
		pSharedMemory->kirk_flag = 2;
	else if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_BIPED_KIRK && pSharedMemory->biped_flag == 1)
		pSharedMemory->kirk_flag = 2;
	else
		AfxMessageBox("Other Command is activated..!!");			
}


BOOL CDrcQuadDlg::OnInitDialog() 
{
	CString str_temp;
	CDialog::OnInitDialog();
	
	// TODO: Add extra initialization here
	
	str_temp.Format("%d",0);
	GetDlgItem(IDC_QUAD_MODE)->SetWindowText(str_temp);

	str_temp.Format("%d",0);
	GetDlgItem(IDC_BIPED_MODE)->SetWindowText(str_temp);
	
	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}


