// DRCtestDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "DRCtestDlg.h"
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
// CDRCtestDlg dialog


CDRCtestDlg::CDRCtestDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CDRCtestDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CDRCtestDlg)
	m_check2 = FALSE;
	m_check3 = FALSE;
	//}}AFX_DATA_INIT
}


void CDRCtestDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CDRCtestDlg)
	DDX_Check(pDX, IDC_CHECK2, m_check2);
	DDX_Check(pDX, IDC_CHECK3, m_check3);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CDRCtestDlg, CDialog)
	//{{AFX_MSG_MAP(CDRCtestDlg)
	ON_BN_CLICKED(IDC_DRC_HOME, OnDrcHome)
	ON_BN_CLICKED(IDC_DRC_READY, OnDrcReady)
	ON_BN_CLICKED(IDC_DRC_STOP, OnDrcStop)
	ON_BN_CLICKED(IDC_DRC_START, OnDrcStart)
	ON_BN_CLICKED(IDC_DRC_FREQ_READY, OnDrcFreqReady)
	ON_BN_CLICKED(IDC_DRC_FREQ_START, OnDrcFreqStart)
	ON_BN_CLICKED(IDC_DRC_WALK_READY, OnDrcWalkReady)
	ON_BN_CLICKED(IDC_DRC_HOME2, OnDrcHome2)
	ON_BN_CLICKED(IDC_DRC_WR, OnDrcWr)
	ON_BN_CLICKED(IDC_DRC_READY2, OnDrcReady2)
	ON_BN_CLICKED(IDC_DRC_UPDATE, OnDrcUpdate)
	ON_BN_CLICKED(IDC_DRC_COMPLE, OnDrcComple)
	ON_BN_CLICKED(IDC_DRC_COMPLE2, OnDrcComple2)
	ON_BN_CLICKED(IDC_DRC_SR, OnDrcSr)
	ON_BN_CLICKED(IDC_DRC_STEER, OnDrcSteer)
	ON_BN_CLICKED(IDC_DRC_GRAB, OnDrcGrab)
	ON_BN_CLICKED(IDC_DRC_GRAB2, OnDrcGrab2)
	ON_BN_CLICKED(IDC_DRC_GRAB3, OnDrcGrab3)
	ON_BN_CLICKED(IDC_DRC_INIT_HAND, OnDrcInitHand)
	ON_BN_CLICKED(IDC_DRC_ZEROSTEER, OnDrcZerosteer)
	ON_BN_CLICKED(IDC_DRC_SR2, OnDrcSr2)
	ON_BN_CLICKED(IDC_DRC_LAD, OnDrcLad)
	ON_BN_CLICKED(IDC_DRC_LAD2, OnDrcLad2)
	ON_BN_CLICKED(IDC_DRC_LAD3, OnDrcLad3)
	ON_BN_CLICKED(IDC_DRC_STOP2, OnDrcStop2)
	ON_BN_CLICKED(IDC_DRC_LAD4, OnDrcLad4)
	ON_BN_CLICKED(IDC_DRC_LAD5, OnDrcLad5)
	ON_BN_CLICKED(IDC_DRC_LAD6, OnDrcLad6)
	ON_BN_CLICKED(IDC_DRC_LAD7, OnDrcLad7)
	ON_BN_CLICKED(IDC_DRC_LAD8, OnDrcLad8)
	ON_BN_CLICKED(IDC_DRC_LAD9, OnDrcLad9)
	ON_BN_CLICKED(IDC_DRC_GRAB4, OnDrcGrab4)
	ON_BN_CLICKED(IDC_DRC_GRAB5, OnDrcGrab5)
	ON_BN_CLICKED(IDC_DRC_GRAB6, OnDrcGrab6)
	ON_BN_CLICKED(IDC_DRC_INIT_HAND2, OnDrcInitHand2)
	ON_BN_CLICKED(IDC_DRC_LAD10, OnDrcLad10)
	ON_BN_CLICKED(IDC_DRC_LAD11, OnDrcLad11)
	ON_BN_CLICKED(IDC_DRC_LAD12, OnDrcLad12)
	ON_BN_CLICKED(IDC_DRC_LAD13, OnDrcLad13)
	ON_BN_CLICKED(IDC_DRC_LAD14, OnDrcLad14)
	ON_BN_CLICKED(IDC_DRC_SR3, OnDrcSr3)
	ON_BN_CLICKED(IDC_DRC_FREQ_START2, OnDrcFreqStart2)
	ON_BN_CLICKED(IDC_DRC_GRAB7, OnDrcGrab7)
	ON_BN_CLICKED(IDC_DRC_GRAB8, OnDrcGrab8)
	ON_BN_CLICKED(IDC_DRC_GRAB9, OnDrcGrab9)
	ON_BN_CLICKED(IDC_DRC_GRAB10, OnDrcGrab10)
	ON_BN_CLICKED(IDC_DRC_GRAB11, OnDrcGrab11)
	ON_BN_CLICKED(IDC_DRC_GRAB12, OnDrcGrab12)
	ON_BN_CLICKED(IDC_DRC_NECK1, OnDrcNeck1)
	ON_BN_CLICKED(IDC_DRC_NECK2, OnDrcNeck2)
	ON_BN_CLICKED(IDC_DRC_NECK3, OnDrcNeck3)
	ON_BN_CLICKED(IDC_DRC_NECK4, OnDrcNeck4)
	ON_BN_CLICKED(IDC_DRC_START2, OnDrcStart2)
	ON_BN_CLICKED(IDC_DRC_HAND_MODE, OnDrcHandMode)
	ON_BN_CLICKED(IDC_DRC_STICK_MODE, OnDrcStickMode)
	ON_BN_CLICKED(IDC_DRC_START3, OnDrcStart3)
	ON_BN_CLICKED(IDC_DRC_LAD15, OnDrcLad15)
	ON_BN_CLICKED(IDC_DRC_HAND_YAW, OnDrcHandYaw)
	ON_BN_CLICKED(IDC_DRC_HAND_YAW2, OnDrcHandYaw2)
	ON_BN_CLICKED(IDC_DRC_GAINOVR, OnDrcGainovr)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDRCtestDlg message handlers

void CDRCtestDlg::OnDrcHome() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
		pSharedMemory->CommandFlag = GOTO_HOME_POS;		
	else 
		AfxMessageBox("Other Command is activated..!!");	
}

void CDRCtestDlg::OnDrcWalkReady() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
		pSharedMemory->CommandFlag = GOTO_WALK_READY_POS;
	else 
		AfxMessageBox("Other Command is activated..!!");
}

void CDRCtestDlg::OnDrcReady() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
		pSharedMemory->CommandFlag = GOTO_DRC_QUADRUPED_READY_POS;		
	else 
		AfxMessageBox("Other Command is activated..!!");
}

void CDRCtestDlg::OnDrcStop() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
		pSharedMemory->CommandFlag = EMERGENCY_STOP;	
}

void CDRCtestDlg::OnDrcStart() 
{
	float temp;
	CString str_temp;

	GetDlgItem(IDC_DRC_START)->EnableWindow(FALSE);
	if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_OFFLINE_DEMO_DRC_TEST && pSharedMemory->transform_flag == 3)
	{
		GetDlgItem(IDC_EDIT_TEMP)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);
		pSharedMemory->WB_const_temp = temp; //3.f;

		GetDlgItem(IDC_EDIT_TEMP2)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);
		pSharedMemory->WB_const_temp2 = temp; //5.f;

		GetDlgItem(IDC_EDIT_TEMP3)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);
		pSharedMemory->WB_const_temp3 = temp; //3.f;

		GetDlgItem(IDC_EDIT_TEMP4)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);
		pSharedMemory->WB_const_temp4 = temp; //10.f;

		GetDlgItem(IDC_EDIT_TEMP11)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);	
		pSharedMemory->temp_fHand_threshold = temp;	

		GetDlgItem(IDC_EDIT_TEMP12)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);	
		pSharedMemory->temp_EL_compen = temp;

		GetDlgItem(IDC_EDIT_TEMP13)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);	
		pSharedMemory->temp_ssp_damp_on = temp;

		GetDlgItem(IDC_EDIT_TEMP14)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);	
		if(temp == 0.f)
			pSharedMemory->temp_dsp_damp_on_flag = 0;
		else
			pSharedMemory->temp_dsp_damp_on_flag = 1;

		GetDlgItem(IDC_EDIT_TEMP15)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);	
		if(temp == 0.f)
			pSharedMemory->temp_foot_LC_on_flag = 0;
		else
			pSharedMemory->temp_foot_LC_on_flag = 1;

		GetDlgItem(IDC_EDIT_TEMP16)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);	
		if(temp == 0.f)
			pSharedMemory->temp_hand_LC_on_flag = 0;
		else
			pSharedMemory->temp_hand_LC_on_flag = 1;

		pSharedMemory->transform_flag = 4;		
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
	GetDlgItem(IDC_DRC_START)->EnableWindow(TRUE);
}


void CDRCtestDlg::OnDrcFreqReady() 
{
	CString strText;	
	FILE *fp;
	int i;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->position_mode_flag = 0;
		pSharedMemory->torque_mode_flag = 0;
	
		GetDlgItem(IDC_EDIT_TEMP23)->GetWindowText(strText);
		pSharedMemory->ref_fLH_3x1[1] = (float)atof(strText);		
		GetDlgItem(IDC_EDIT_TEMP24)->GetWindowText(strText);
		pSharedMemory->ref_fLH_3x1[2] = (float)atof(strText);		
		GetDlgItem(IDC_EDIT_TEMP25)->GetWindowText(strText);
		pSharedMemory->ref_fLH_3x1[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP26)->GetWindowText(strText);
		pSharedMemory->ref_fRH_3x1[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP27)->GetWindowText(strText);
		pSharedMemory->ref_fRH_3x1[2] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP28)->GetWindowText(strText);
		pSharedMemory->ref_fRH_3x1[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP29)->GetWindowText(strText);
		pSharedMemory->dpRH[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP30)->GetWindowText(strText);
		pSharedMemory->dpRH[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP31)->GetWindowText(strText);
		pSharedMemory->dpRH[2] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP32)->GetWindowText(strText);
		pSharedMemory->dpLH[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP33)->GetWindowText(strText);
		pSharedMemory->dpLH[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP34)->GetWindowText(strText);
		pSharedMemory->dpLH[2] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP35)->GetWindowText(strText);
		pSharedMemory->drvRH[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP36)->GetWindowText(strText);
		pSharedMemory->drvRH[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP37)->GetWindowText(strText);
		pSharedMemory->drvRH[2] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP38)->GetWindowText(strText);
		pSharedMemory->drvRH[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP39)->GetWindowText(strText);
		pSharedMemory->drvLH[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP40)->GetWindowText(strText);
		pSharedMemory->drvLH[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP41)->GetWindowText(strText);
		pSharedMemory->drvLH[2] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP42)->GetWindowText(strText);
		pSharedMemory->drvLH[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP43)->GetWindowText(strText);
		pSharedMemory->dpRF[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP44)->GetWindowText(strText);
		pSharedMemory->dpRF[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP45)->GetWindowText(strText);
		pSharedMemory->dpRF[2] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP46)->GetWindowText(strText);
		pSharedMemory->dpLF[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP47)->GetWindowText(strText);
		pSharedMemory->dpLF[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP48)->GetWindowText(strText);
		pSharedMemory->dpLF[2] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP49)->GetWindowText(strText);
		pSharedMemory->drvRF[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP50)->GetWindowText(strText);
		pSharedMemory->drvRF[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP51)->GetWindowText(strText);
		pSharedMemory->drvRF[2] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP52)->GetWindowText(strText);
		pSharedMemory->drvRF[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP53)->GetWindowText(strText);
		pSharedMemory->drvLF[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP54)->GetWindowText(strText);
		pSharedMemory->drvLF[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP55)->GetWindowText(strText);
		pSharedMemory->drvLF[2] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP56)->GetWindowText(strText);
		pSharedMemory->drvLF[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP57)->GetWindowText(strText);
		pSharedMemory->dpCOM[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP58)->GetWindowText(strText);
		pSharedMemory->dpCOM[1] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP22)->GetWindowText(strText);
		pSharedMemory->move_sec = (float)fabs((float)atof(strText));


		if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\gain_cloop_quad.txt","r")) == NULL )
		{
			AfxMessageBox("Data file 'gain_cloop_quad.txt' was not found.");		
			return;
		}
		for(i=0;i<17;i++)
		{
			fscanf(fp,"%f %f", &(pSharedMemory->kp[i]), &(pSharedMemory->kd[i]));
			while(fgetc(fp) != '\n');
		}
		fclose(fp);

		Sleep(500);
		pSharedMemory->CommandFlag = GOTO_DRC_BIPED_READY_POS;
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
	
}


void CDRCtestDlg::OnDrcFreqStart() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode != CTRLMODE_WB_OFFLINE_DEMO)
		{
			pSharedMemory->WB_DemoNo = WB_FREQ_TEST;
			pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}



void CDRCtestDlg::OnDrcFreqStart2() 
{
	float temp;
	CString str_temp;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo == WB_FREQ_TEST)
		{
			if(pSharedMemory->position_mode_flag == 0)
			{
				GetDlgItem(IDC_EDIT_AMP)->GetWindowText(str_temp);
				temp = (float)atof(str_temp);
				if(temp > 0.1f)
					temp = 0.1f;
				else if(temp < -0.1f)
					temp = -0.1f;
				pSharedMemory->WB_FreqTest_Amp = temp;

				GetDlgItem(IDC_EDIT_FREQ)->GetWindowText(str_temp);
				temp = (float)atof(str_temp);
				pSharedMemory->WB_FreqTest_Omega = 2.f*PI*temp;

				pSharedMemory->position_mode_flag = 6;
			}
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}


BOOL CDRCtestDlg::OnInitDialog() 
{
	CDialog::OnInitDialog();
	
	// TODO: Add extra initialization here
	CString str_temp;

	str_temp.Format("%g",0.03f);
	GetDlgItem(IDC_EDIT_COMX0)->SetWindowText(str_temp);

	str_temp.Format("%g",0.f);
	GetDlgItem(IDC_EDIT_AMP)->SetWindowText(str_temp);

	str_temp.Format("%g",0.f);
	GetDlgItem(IDC_EDIT_FREQ)->SetWindowText(str_temp);

	str_temp.Format("%g",3.f);
	GetDlgItem(IDC_EDIT_TEMP)->SetWindowText(str_temp);

	str_temp.Format("%g",5.f);
	GetDlgItem(IDC_EDIT_TEMP2)->SetWindowText(str_temp);

	str_temp.Format("%g",3.f);
	GetDlgItem(IDC_EDIT_TEMP3)->SetWindowText(str_temp);

	str_temp.Format("%g",10.f);
	GetDlgItem(IDC_EDIT_TEMP4)->SetWindowText(str_temp);

	str_temp.Format("%g",1.f);
	GetDlgItem(IDC_EDIT_TEMP5)->SetWindowText(str_temp);

	str_temp.Format("%g",0.f);
	GetDlgItem(IDC_EDIT_TEMP6)->SetWindowText(str_temp);

	str_temp.Format("%g",0.f);
	GetDlgItem(IDC_EDIT_TEMP7)->SetWindowText(str_temp);

	str_temp.Format("%g",1.f);
	GetDlgItem(IDC_EDIT_TEMP8)->SetWindowText(str_temp);

	str_temp.Format("%g",0.8f);
	GetDlgItem(IDC_EDIT_TEMP9)->SetWindowText(str_temp);

	str_temp.Format("%g",10.f);
	GetDlgItem(IDC_EDIT_TEMP10)->SetWindowText(str_temp);

	str_temp.Format("%g",10.f);
	GetDlgItem(IDC_EDIT_TEMP11)->SetWindowText(str_temp);

	str_temp.Format("%g",0.0f);
	GetDlgItem(IDC_EDIT_TEMP12)->SetWindowText(str_temp);

	str_temp.Format("%g",0.f);
	GetDlgItem(IDC_EDIT_TEMP13)->SetWindowText(str_temp);

	str_temp.Format("%g",0.f);
	GetDlgItem(IDC_EDIT_TEMP14)->SetWindowText(str_temp);

	str_temp.Format("%g",0.f);
	GetDlgItem(IDC_EDIT_TEMP15)->SetWindowText(str_temp);

	str_temp.Format("%g",0.f);
	GetDlgItem(IDC_EDIT_TEMP16)->SetWindowText(str_temp);

	str_temp.Format("%g",0.f);
	GetDlgItem(IDC_EDIT_TEMP20)->SetWindowText(str_temp);

	str_temp.Format("%g",-10.f);
	GetDlgItem(IDC_EDIT_TEMP21)->SetWindowText(str_temp);

	str_temp.Format("%g", 5.f);
	GetDlgItem(IDC_EDIT_TEMP22)->SetWindowText(str_temp);

	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP23)->SetWindowText(str_temp);

	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP24)->SetWindowText(str_temp);

	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP25)->SetWindowText(str_temp);

	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP26)->SetWindowText(str_temp);

	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP27)->SetWindowText(str_temp);

	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP28)->SetWindowText(str_temp);

	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP29)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP30)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP31)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP32)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP33)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP34)->SetWindowText(str_temp);

	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP35)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP36)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP37)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP38)->SetWindowText(str_temp);

	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP39)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP40)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP41)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP42)->SetWindowText(str_temp);

	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP43)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP44)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP45)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP46)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP47)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP48)->SetWindowText(str_temp);

	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP49)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP50)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP51)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP52)->SetWindowText(str_temp);

	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP53)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP54)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP55)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP56)->SetWindowText(str_temp);

	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP57)->SetWindowText(str_temp);
	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP58)->SetWindowText(str_temp);

	str_temp.Format("%g", 10.f);
	GetDlgItem(IDC_EDIT_TEMP59)->SetWindowText(str_temp);

	str_temp.Format("%g",10.f);
	GetDlgItem(IDC_EDIT_TEMP19)->SetWindowText(str_temp);

	str_temp.Format("%g", 3.f);
	GetDlgItem(IDC_EDIT_TEMP61)->SetWindowText(str_temp);

	str_temp.Format("%g", 3.f);
	GetDlgItem(IDC_EDIT_TEMP62)->SetWindowText(str_temp);


	str_temp.Format("%g", 0.f);
	GetDlgItem(IDC_EDIT_TEMP60)->SetWindowText(str_temp);


	str_temp.Format("%g",0.f);
	GetDlgItem(IDC_EDIT_TEMP17)->SetWindowText(str_temp);

	str_temp.Format("%g",0.f);
	GetDlgItem(IDC_EDIT_TEMP18)->SetWindowText(str_temp);

	str_temp.Format("%g",0.f);
	GetDlgItem(IDC_EDIT_TEMP63)->SetWindowText(str_temp);

	str_temp.Format("%g",0.f);
	GetDlgItem(IDC_EDIT_TEMP64)->SetWindowText(str_temp);

	str_temp.Format("%g",0.f);
	GetDlgItem(IDC_EDIT_TEMP65)->SetWindowText(str_temp);

	str_temp.Format("%g",0.f);
	GetDlgItem(IDC_EDIT_TEMP66)->SetWindowText(str_temp);

	str_temp.Format("%g",45.f);
	GetDlgItem(IDC_EDIT_TEMP67)->SetWindowText(str_temp);
	str_temp.Format("%g",0.5f);
	GetDlgItem(IDC_EDIT_TEMP68)->SetWindowText(str_temp);

	str_temp.Format("%g",100.f);
	GetDlgItem(IDC_EDIT_GAINOVR)->SetWindowText(str_temp);

	str_temp.Format("%g",1.f);
	GetDlgItem(IDC_EDIT_GAINOVR2)->SetWindowText(str_temp);

	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}


void CDRCtestDlg::OnDrcHome2() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
		pSharedMemory->CommandFlag = GOTO_DRC_HOME_POS;		
	else 
		AfxMessageBox("Other Command is activated..!!");	
}


void CDRCtestDlg::OnDrcWr() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
		pSharedMemory->CommandFlag = GOTO_DRC_BIPED_READY_POS;		
	else 
		AfxMessageBox("Other Command is activated..!!");
	
}


void CDRCtestDlg::OnDrcReady2() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
		pSharedMemory->CommandFlag = GOTO_DRC_PRE_QUAD_READY_POS;		
	else 
		AfxMessageBox("Other Command is activated..!!");
	
}


void CDRCtestDlg::OnDrcUpdate() 
{
	CString str_temp;
	float temp;

	GetDlgItem(IDC_EDIT_TEMP8)->GetWindowText(str_temp);
	temp = (float)atof(str_temp)*2.f*PI;	
	pSharedMemory->temp_Wn = temp*temp;

	GetDlgItem(IDC_EDIT_TEMP9)->GetWindowText(str_temp);
	temp = (float)atof(str_temp);	
	pSharedMemory->temp_b = 2.f*temp*(float)sqrt(pSharedMemory->temp_Wn);
	
	GetDlgItem(IDC_EDIT_TEMP10)->GetWindowText(str_temp);
	temp = (float)atof(str_temp);	
	pSharedMemory->temp_m = temp;	

	GetDlgItem(IDC_EDIT_TEMP11)->GetWindowText(str_temp);
	temp = (float)atof(str_temp);	
	pSharedMemory->temp_fHand_threshold = temp;	

	GetDlgItem(IDC_EDIT_TEMP12)->GetWindowText(str_temp);
	temp = (float)atof(str_temp);	
	pSharedMemory->temp_EL_compen = temp;	
}

void CDRCtestDlg::OnDrcComple() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->sw_mode = SW_MODE_COMPLEMENTARY;
		pSharedMemory->CommandFlag = SET_SW_MODE;
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDRCtestDlg::OnDrcComple2() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->sw_mode = SW_MODE_NON_COMPLEMENTARY;
		pSharedMemory->CommandFlag = SET_SW_MODE;
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDRCtestDlg::OnDrcSr() 
{
	FILE *fp;
	CString strText;
	int i;
	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{	
		pSharedMemory->position_mode_flag = 0;
		pSharedMemory->torque_mode_flag = 0;
	
		GetDlgItem(IDC_EDIT_TEMP23)->GetWindowText(strText);
		pSharedMemory->ref_fLH_3x1[1] = (float)atof(strText);		
		GetDlgItem(IDC_EDIT_TEMP24)->GetWindowText(strText);
		pSharedMemory->ref_fLH_3x1[2] = (float)atof(strText);		
		GetDlgItem(IDC_EDIT_TEMP25)->GetWindowText(strText);
		pSharedMemory->ref_fLH_3x1[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP26)->GetWindowText(strText);
		pSharedMemory->ref_fRH_3x1[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP27)->GetWindowText(strText);
		pSharedMemory->ref_fRH_3x1[2] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP28)->GetWindowText(strText);
		pSharedMemory->ref_fRH_3x1[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP29)->GetWindowText(strText);
		pSharedMemory->dpRH[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP30)->GetWindowText(strText);
		pSharedMemory->dpRH[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP31)->GetWindowText(strText);
		pSharedMemory->dpRH[2] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP32)->GetWindowText(strText);
		pSharedMemory->dpLH[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP33)->GetWindowText(strText);
		pSharedMemory->dpLH[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP34)->GetWindowText(strText);
		pSharedMemory->dpLH[2] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP35)->GetWindowText(strText);
		pSharedMemory->drvRH[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP36)->GetWindowText(strText);
		pSharedMemory->drvRH[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP37)->GetWindowText(strText);
		pSharedMemory->drvRH[2] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP38)->GetWindowText(strText);
		pSharedMemory->drvRH[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP39)->GetWindowText(strText);
		pSharedMemory->drvLH[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP40)->GetWindowText(strText);
		pSharedMemory->drvLH[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP41)->GetWindowText(strText);
		pSharedMemory->drvLH[2] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP42)->GetWindowText(strText);
		pSharedMemory->drvLH[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP43)->GetWindowText(strText);
		pSharedMemory->dpRF[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP44)->GetWindowText(strText);
		pSharedMemory->dpRF[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP45)->GetWindowText(strText);
		pSharedMemory->dpRF[2] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP46)->GetWindowText(strText);
		pSharedMemory->dpLF[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP47)->GetWindowText(strText);
		pSharedMemory->dpLF[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP48)->GetWindowText(strText);
		pSharedMemory->dpLF[2] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP49)->GetWindowText(strText);
		pSharedMemory->drvRF[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP50)->GetWindowText(strText);
		pSharedMemory->drvRF[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP51)->GetWindowText(strText);
		pSharedMemory->drvRF[2] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP52)->GetWindowText(strText);
		pSharedMemory->drvRF[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP53)->GetWindowText(strText);
		pSharedMemory->drvLF[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP54)->GetWindowText(strText);
		pSharedMemory->drvLF[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP55)->GetWindowText(strText);
		pSharedMemory->drvLF[2] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP56)->GetWindowText(strText);
		pSharedMemory->drvLF[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP57)->GetWindowText(strText);
		pSharedMemory->dpCOM[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP58)->GetWindowText(strText);
		pSharedMemory->dpCOM[1] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP22)->GetWindowText(strText);
		pSharedMemory->move_sec = (float)fabs((float)atof(strText));
		
		if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\steering_data.txt","r")) == NULL )
		{
			AfxMessageBox("Data file 'steering_data.txt' was not found.");		
			return;
		}
		fscanf(fp,"%f %f %f %f %f %f %f", &(pSharedMemory->ref_rsp), &(pSharedMemory->ref_rsr), &(pSharedMemory->ref_rsy), &(pSharedMemory->ref_reb), &(pSharedMemory->ref_rwy), &(pSharedMemory->ref_rwp), &(pSharedMemory->ref_rwy2));
		fscanf(fp,"%f %f %f %f %f %f %f", &(pSharedMemory->ref_lsp), &(pSharedMemory->ref_lsr), &(pSharedMemory->ref_lsy), &(pSharedMemory->ref_leb), &(pSharedMemory->ref_lwy), &(pSharedMemory->ref_lwp), &(pSharedMemory->ref_lwy2));
		fscanf(fp,"%f %f %f %f %f %f", &(pSharedMemory->ref_rhy), &(pSharedMemory->ref_rhr), &(pSharedMemory->ref_rhp), &(pSharedMemory->ref_rkn), &(pSharedMemory->ref_rap), &(pSharedMemory->ref_rar));
		fscanf(fp,"%f %f %f %f %f %f", &(pSharedMemory->ref_lhy), &(pSharedMemory->ref_lhr), &(pSharedMemory->ref_lhp), &(pSharedMemory->ref_lkn), &(pSharedMemory->ref_lap), &(pSharedMemory->ref_lar));
		fclose(fp);

		if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\steering_data2.txt","r")) == NULL )
		{
			AfxMessageBox("Data file 'steering_data2.txt' was not found.");		
			return;
		}
		for(i=0; i<N_STEER; i++)
			fscanf(fp,"%f %f", &(pSharedMemory->steer_time[i]), &(pSharedMemory->steer_ang[i]));
		fclose(fp);


		if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\gain_cloop_steering.txt","r")) == NULL )
		{
			AfxMessageBox("Data file 'gain_cloop_steering.txt' was not found.");		
			return;
		}
		for(i=0;i<17;i++)
		{
			fscanf(fp,"%f %f", &(pSharedMemory->kp[i]), &(pSharedMemory->kd[i]));
			while(fgetc(fp) != '\n');
		}
		fclose(fp);
		

		Sleep(500);
		if(pSharedMemory->CommandFlag == NO_ACT)
			pSharedMemory->CommandFlag = GOTO_DRC_STEERING_READY_POS;			
		else
			AfxMessageBox("Other Command is activated..!!");
	}
	else 
		AfxMessageBox("Other Command is activated..!!");

}


void CDRCtestDlg::OnDrcSteer() 
{
	CString str_temp;
	float temp;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode != CTRLMODE_WB_OFFLINE_DEMO)
		{
			GetDlgItem(IDC_EDIT_TEMP14)->GetWindowText(str_temp);
			temp = (float)atof(str_temp);	
			if(temp == 0.f)
				pSharedMemory->temp_dsp_damp_on_flag = 0;
			else
				pSharedMemory->temp_dsp_damp_on_flag = 1;

			pSharedMemory->steer_demo = 0;
			pSharedMemory->sw_mode = SW_MODE_NON_COMPLEMENTARY;
			pSharedMemory->CommandFlag = SET_SW_MODE;
			Sleep(200);
			pSharedMemory->WB_DemoNo = WB_STEERING_TEST;
			pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}


void CDRCtestDlg::OnDrcGrab() 
{
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_EDIT_TEMP19)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[0] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP62)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[1] = (float)atof(strText);
	
		UpdateData(true);
		if(m_check3 == 1)
			pSharedMemory->CommandDataFloat[2] = 1.f;
		else
			pSharedMemory->CommandDataFloat[2] = 0.f;
			
		Sleep(10);
		pSharedMemory->CommandFlag = GRIP_ON_L;
		Sleep(10);
	}
}


void CDRCtestDlg::OnDrcGrab2() 
{
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_EDIT_TEMP19)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[0] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP62)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[1] = (float)atof(strText);
	
		UpdateData(true);
		if(m_check3 == 1)
			pSharedMemory->CommandDataFloat[2] = 1.f;
		else
			pSharedMemory->CommandDataFloat[2] = 0.f;
			
		Sleep(10);
		pSharedMemory->CommandFlag = GRIP_OFF_L;
		Sleep(10);
	}
}

void CDRCtestDlg::OnDrcGrab3() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		UpdateData(true);
		if(m_check3 == 1)
			pSharedMemory->CommandDataFloat[2] = 1.f;
		else
			pSharedMemory->CommandDataFloat[2] = 0.f;
		pSharedMemory->CommandFlag = GRIP_STOP_L;
	}
}

void CDRCtestDlg::OnDrcInitHand() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->JointID = LF2;
		pSharedMemory->CommandFlag = GOTO_LIMIT_POS;
		Sleep(200);
		pSharedMemory->CommandFlag = C_CONTROL_MODE;
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDRCtestDlg::OnDrcZerosteer() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->BoardID = EJMC2;
		pSharedMemory->CommandFlag = ENCODER_ZERO_EACH;
	}
	else AfxMessageBox("Other Command is activated..!!");	
}

void CDRCtestDlg::OnDrcSr2() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo == WB_STEERING_TEST)
		{
			if(pSharedMemory->torque_mode_flag == 0)
				pSharedMemory->torque_mode_flag = 1;
			pSharedMemory->steer_demo = 1;
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDRCtestDlg::OnDrcLad() 
{
	FILE *fp;
	CString strText;
	int i;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->position_mode_flag = 0;
		pSharedMemory->torque_mode_flag = 0;
		pSharedMemory->ladder_demo = 0;
	
		GetDlgItem(IDC_EDIT_TEMP23)->GetWindowText(strText);
		pSharedMemory->ref_fLH_3x1[1] = (float)atof(strText);		
		GetDlgItem(IDC_EDIT_TEMP24)->GetWindowText(strText);
		pSharedMemory->ref_fLH_3x1[2] = (float)atof(strText);		
		GetDlgItem(IDC_EDIT_TEMP25)->GetWindowText(strText);
		pSharedMemory->ref_fLH_3x1[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP26)->GetWindowText(strText);
		pSharedMemory->ref_fRH_3x1[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP27)->GetWindowText(strText);
		pSharedMemory->ref_fRH_3x1[2] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP28)->GetWindowText(strText);
		pSharedMemory->ref_fRH_3x1[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP29)->GetWindowText(strText);
		pSharedMemory->dpRH[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP30)->GetWindowText(strText);
		pSharedMemory->dpRH[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP31)->GetWindowText(strText);
		pSharedMemory->dpRH[2] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP32)->GetWindowText(strText);
		pSharedMemory->dpLH[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP33)->GetWindowText(strText);
		pSharedMemory->dpLH[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP34)->GetWindowText(strText);
		pSharedMemory->dpLH[2] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP35)->GetWindowText(strText);
		pSharedMemory->drvRH[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP36)->GetWindowText(strText);
		pSharedMemory->drvRH[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP37)->GetWindowText(strText);
		pSharedMemory->drvRH[2] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP38)->GetWindowText(strText);
		pSharedMemory->drvRH[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP39)->GetWindowText(strText);
		pSharedMemory->drvLH[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP40)->GetWindowText(strText);
		pSharedMemory->drvLH[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP41)->GetWindowText(strText);
		pSharedMemory->drvLH[2] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP42)->GetWindowText(strText);
		pSharedMemory->drvLH[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP43)->GetWindowText(strText);
		pSharedMemory->dpRF[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP44)->GetWindowText(strText);
		pSharedMemory->dpRF[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP45)->GetWindowText(strText);
		pSharedMemory->dpRF[2] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP46)->GetWindowText(strText);
		pSharedMemory->dpLF[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP47)->GetWindowText(strText);
		pSharedMemory->dpLF[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP48)->GetWindowText(strText);
		pSharedMemory->dpLF[2] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP49)->GetWindowText(strText);
		pSharedMemory->drvRF[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP50)->GetWindowText(strText);
		pSharedMemory->drvRF[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP51)->GetWindowText(strText);
		pSharedMemory->drvRF[2] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP52)->GetWindowText(strText);
		pSharedMemory->drvRF[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP53)->GetWindowText(strText);
		pSharedMemory->drvLF[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP54)->GetWindowText(strText);
		pSharedMemory->drvLF[1] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP55)->GetWindowText(strText);
		pSharedMemory->drvLF[2] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP56)->GetWindowText(strText);
		pSharedMemory->drvLF[3] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP57)->GetWindowText(strText);
		pSharedMemory->dpCOM[0] = (float)atof(strText);
		GetDlgItem(IDC_EDIT_TEMP58)->GetWindowText(strText);
		pSharedMemory->dpCOM[1] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP22)->GetWindowText(strText);
		pSharedMemory->move_sec = (float)fabs((float)atof(strText));

		
		if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\ladder_climbing_data.txt","r")) == NULL )
		{
			AfxMessageBox("Data file 'ladder_climbing_data.txt' was not found.");		
			return;
		}
		fscanf(fp,"%f %f %f %f %f %f %f", &(pSharedMemory->ref_rsp), &(pSharedMemory->ref_rsr), &(pSharedMemory->ref_rsy), &(pSharedMemory->ref_reb), &(pSharedMemory->ref_rwy), &(pSharedMemory->ref_rwp), &(pSharedMemory->ref_rwy2));
		fscanf(fp,"%f %f %f %f %f %f %f", &(pSharedMemory->ref_lsp), &(pSharedMemory->ref_lsr), &(pSharedMemory->ref_lsy), &(pSharedMemory->ref_leb), &(pSharedMemory->ref_lwy), &(pSharedMemory->ref_lwp), &(pSharedMemory->ref_lwy2));
		fscanf(fp,"%f %f %f %f %f %f", &(pSharedMemory->ref_rhy), &(pSharedMemory->ref_rhr), &(pSharedMemory->ref_rhp), &(pSharedMemory->ref_rkn), &(pSharedMemory->ref_rap), &(pSharedMemory->ref_rar));
		fscanf(fp,"%f %f %f %f %f %f", &(pSharedMemory->ref_lhy), &(pSharedMemory->ref_lhr), &(pSharedMemory->ref_lhp), &(pSharedMemory->ref_lkn), &(pSharedMemory->ref_lap), &(pSharedMemory->ref_lar));
		fclose(fp);

		if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\gain_cloop_ladder.txt","r")) == NULL )
		{
			AfxMessageBox("Data file 'gain_cloop_ladder.txt' was not found.");		
			return;
		}
		for(i=0;i<17;i++)
		{
			fscanf(fp,"%f %f", &(pSharedMemory->kp[i]), &(pSharedMemory->kd[i]));
			while(fgetc(fp) != '\n');
		}
		fclose(fp);
		

		Sleep(500);
		if(pSharedMemory->CommandFlag == NO_ACT)
			pSharedMemory->CommandFlag = GOTO_DRC_LADDER_CLIMBING_READY_POS;			
		else
			AfxMessageBox("Other Command is activated..!!");
	}
	else 
		AfxMessageBox("Other Command is activated..!!");
}

void CDRCtestDlg::OnDrcLad2() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode != CTRLMODE_WB_OFFLINE_DEMO)
		{
			pSharedMemory->sw_mode = SW_MODE_NON_COMPLEMENTARY;
			pSharedMemory->CommandFlag = SET_SW_MODE;
			Sleep(200);
			pSharedMemory->WB_DemoNo = WB_LADDER_CLIMBING_TEST;
			pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	
}

void CDRCtestDlg::OnDrcLad3() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo == WB_LADDER_CLIMBING_TEST)
		{
			if(pSharedMemory->torque_mode_flag == 2)
				pSharedMemory->ladder_demo = 1;
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDRCtestDlg::OnDrcStop2() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
		pSharedMemory->CommandFlag = EMERGENCY_STOP;
	
}

void CDRCtestDlg::OnDrcLad4() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo == WB_LADDER_CLIMBING_TEST)
		{
			if(pSharedMemory->torque_mode_flag == 2)
				pSharedMemory->ladder_demo = 0;
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");

}

void CDRCtestDlg::OnDrcLad5() 
{
	CString strText;
	float temp;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && (pSharedMemory->WB_DemoNo == WB_LADDER_CLIMBING_TEST || pSharedMemory->WB_DemoNo == WB_FREQ_TEST))
		{
			if(pSharedMemory->position_mode_flag == 0)
			{
				GetDlgItem(IDC_EDIT_TEMP29)->GetWindowText(strText);
				pSharedMemory->dpRH[0] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP30)->GetWindowText(strText);
				pSharedMemory->dpRH[1] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP31)->GetWindowText(strText);
				pSharedMemory->dpRH[2] = (float)atof(strText);

				GetDlgItem(IDC_EDIT_TEMP32)->GetWindowText(strText);
				pSharedMemory->dpLH[0] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP33)->GetWindowText(strText);
				pSharedMemory->dpLH[1] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP34)->GetWindowText(strText);
				pSharedMemory->dpLH[2] = (float)atof(strText);

				GetDlgItem(IDC_EDIT_TEMP35)->GetWindowText(strText);
				pSharedMemory->drvRH[0] = PI/180.f*(float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP36)->GetWindowText(strText);
				pSharedMemory->drvRH[1] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP37)->GetWindowText(strText);
				pSharedMemory->drvRH[2] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP38)->GetWindowText(strText);
				pSharedMemory->drvRH[3] = (float)atof(strText);

				GetDlgItem(IDC_EDIT_TEMP39)->GetWindowText(strText);
				pSharedMemory->drvLH[0] = PI/180.f*(float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP40)->GetWindowText(strText);
				pSharedMemory->drvLH[1] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP41)->GetWindowText(strText);
				pSharedMemory->drvLH[2] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP42)->GetWindowText(strText);
				pSharedMemory->drvLH[3] = (float)atof(strText);

				GetDlgItem(IDC_EDIT_TEMP22)->GetWindowText(strText);
				temp = (float)fabs((float)atof(strText));
				if(temp < 1.f)
					temp = 1.f;
				pSharedMemory->move_sec = temp;

				pSharedMemory->position_mode_flag = 1;
			}
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");		
}


void CDRCtestDlg::OnDrcLad6() 
{
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo == WB_LADDER_CLIMBING_TEST)
		{
			GetDlgItem(IDC_EDIT_TEMP23)->GetWindowText(strText);
			pSharedMemory->ref_fLH_3x1[1] = (float)atof(strText);
			
			GetDlgItem(IDC_EDIT_TEMP24)->GetWindowText(strText);
			pSharedMemory->ref_fLH_3x1[2] = (float)atof(strText);	
			
			GetDlgItem(IDC_EDIT_TEMP25)->GetWindowText(strText);
			pSharedMemory->ref_fLH_3x1[3] = (float)atof(strText);

			GetDlgItem(IDC_EDIT_TEMP26)->GetWindowText(strText);
			pSharedMemory->ref_fRH_3x1[1] = (float)atof(strText);

			GetDlgItem(IDC_EDIT_TEMP27)->GetWindowText(strText);
			pSharedMemory->ref_fRH_3x1[2] = (float)atof(strText);

			GetDlgItem(IDC_EDIT_TEMP28)->GetWindowText(strText);
			pSharedMemory->ref_fRH_3x1[3] = (float)atof(strText);
		
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	
}

void CDRCtestDlg::OnDrcLad7() 
{
	CString strText;
	float temp;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && (pSharedMemory->WB_DemoNo == WB_LADDER_CLIMBING_TEST || pSharedMemory->WB_DemoNo == WB_FREQ_TEST))
		{
			if(pSharedMemory->position_mode_flag == 0)
			{
				GetDlgItem(IDC_EDIT_TEMP43)->GetWindowText(strText);
				pSharedMemory->dpRF[0] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP44)->GetWindowText(strText);
				pSharedMemory->dpRF[1] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP45)->GetWindowText(strText);
				pSharedMemory->dpRF[2] = (float)atof(strText);

				GetDlgItem(IDC_EDIT_TEMP46)->GetWindowText(strText);
				pSharedMemory->dpLF[0] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP47)->GetWindowText(strText);
				pSharedMemory->dpLF[1] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP48)->GetWindowText(strText);
				pSharedMemory->dpLF[2] = (float)atof(strText);

				GetDlgItem(IDC_EDIT_TEMP49)->GetWindowText(strText);
				pSharedMemory->drvRF[0] = PI/180.f*(float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP50)->GetWindowText(strText);
				pSharedMemory->drvRF[1] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP51)->GetWindowText(strText);
				pSharedMemory->drvRF[2] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP52)->GetWindowText(strText);
				pSharedMemory->drvRF[3] = (float)atof(strText);

				GetDlgItem(IDC_EDIT_TEMP53)->GetWindowText(strText);
				pSharedMemory->drvLF[0] = PI/180.f*(float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP54)->GetWindowText(strText);
				pSharedMemory->drvLF[1] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP55)->GetWindowText(strText);
				pSharedMemory->drvLF[2] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP56)->GetWindowText(strText);
				pSharedMemory->drvLF[3] = (float)atof(strText);

				GetDlgItem(IDC_EDIT_TEMP22)->GetWindowText(strText);
				temp = (float)fabs((float)atof(strText));
				if(temp < 1.f)
					temp = 1.f;
				pSharedMemory->move_sec = temp;

				pSharedMemory->position_mode_flag = 2;
			}
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");	
	
}

void CDRCtestDlg::OnDrcLad8() 
{
	CString strText;
	float temp;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && (pSharedMemory->WB_DemoNo == WB_LADDER_CLIMBING_TEST || pSharedMemory->WB_DemoNo == WB_FREQ_TEST))
		{
			if(pSharedMemory->position_mode_flag == 0)
			{
				GetDlgItem(IDC_EDIT_TEMP57)->GetWindowText(strText);
				pSharedMemory->dpCOM[0] = (float)atof(strText);
				GetDlgItem(IDC_EDIT_TEMP58)->GetWindowText(strText);
				pSharedMemory->dpCOM[1] = (float)atof(strText);	
				
				GetDlgItem(IDC_EDIT_TEMP22)->GetWindowText(strText);
				temp = (float)fabs((float)atof(strText));
				if(temp < 1.f)
					temp = 1.f;
				pSharedMemory->move_sec = temp;

				pSharedMemory->position_mode_flag = 3;
			}
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	
}


void CDRCtestDlg::OnDrcLad9() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo == WB_LADDER_CLIMBING_TEST)
		{
			if(pSharedMemory->torque_mode_flag == 0)
				pSharedMemory->torque_mode_flag = 1;
			else if(pSharedMemory->torque_mode_flag == 2)
				pSharedMemory->torque_mode_flag = 3;
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	
}


void CDRCtestDlg::OnDrcGrab4() 
{
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_EDIT_TEMP59)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[0] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP61)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[1] = (float)atof(strText);
	
		UpdateData(true);
		if(m_check2 == 1)
			pSharedMemory->CommandDataFloat[2] = 1.f;
		else
			pSharedMemory->CommandDataFloat[2] = 0.f;
			
		Sleep(10);
		pSharedMemory->CommandFlag = GRIP_ON_R;
		Sleep(10);
	}
}

void CDRCtestDlg::OnDrcGrab5() 
{
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_EDIT_TEMP59)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[0] = (float)atof(strText);

		GetDlgItem(IDC_EDIT_TEMP61)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[1] = (float)atof(strText);
	
		UpdateData(true);
		if(m_check2 == 1)
			pSharedMemory->CommandDataFloat[2] = 1.f;
		else
			pSharedMemory->CommandDataFloat[2] = 0.f;
			
		Sleep(10);
		pSharedMemory->CommandFlag = GRIP_OFF_R;
		Sleep(10);
	}
	
}

void CDRCtestDlg::OnDrcGrab6() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		UpdateData(true);
		if(m_check2 == 1)
			pSharedMemory->CommandDataFloat[2] = 1.f;
		else
			pSharedMemory->CommandDataFloat[2] = 0.f;
		pSharedMemory->CommandFlag = GRIP_STOP_R;
	}
}


void CDRCtestDlg::OnDrcInitHand2() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->JointID = RFA;
		pSharedMemory->CommandFlag = GOTO_LIMIT_POS;
	//	Sleep(200);
	//	pSharedMemory->CommandFlag = C_CONTROL_MODE;
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDRCtestDlg::OnDrcLad10() 
{
	unsigned int k;
	FILE *fp;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo == WB_LADDER_CLIMBING_TEST)
		{
			if(pSharedMemory->position_mode_flag == 0 && pSharedMemory->ladder_demo == 1)
			{
				if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\offline_traj_ladder_1.txt","r")) == NULL )
				{
					AfxMessageBox("Data file offline_traj_ladder_1.txt was not found");		
					return;
				}
				
				fscanf(fp,"%d", &pSharedMemory->off_traj_length);

				if(pSharedMemory->off_traj_length > MAX_OFF_TRAJ)
				{
					fclose(fp);
					AfxMessageBox("MAX_OFF_TRAJ error!");
				}
				else
				{
					for (k=0; k < pSharedMemory->off_traj_length; k++)
						fscanf(fp,"%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e", 
							&pSharedMemory->off_traj_dpPELz[k], &pSharedMemory->off_traj_dpCOM[k][0], &pSharedMemory->off_traj_dpCOM[k][1], 
							&pSharedMemory->off_traj_dpRH[k][0], &pSharedMemory->off_traj_dpRH[k][1], &pSharedMemory->off_traj_dpRH[k][2],
							&pSharedMemory->off_traj_dpLH[k][0], &pSharedMemory->off_traj_dpLH[k][1], &pSharedMemory->off_traj_dpLH[k][2],
							&pSharedMemory->off_traj_dpRF[k][0], &pSharedMemory->off_traj_dpRF[k][1], &pSharedMemory->off_traj_dpRF[k][2],
							&pSharedMemory->off_traj_dpLF[k][0], &pSharedMemory->off_traj_dpLF[k][1], &pSharedMemory->off_traj_dpLF[k][2],
							&pSharedMemory->off_traj_drvRH[k][0], &pSharedMemory->off_traj_drvRH[k][1], &pSharedMemory->off_traj_drvRH[k][2], &pSharedMemory->off_traj_drvRH[k][3],
							&pSharedMemory->off_traj_drvLH[k][0], &pSharedMemory->off_traj_drvLH[k][1], &pSharedMemory->off_traj_drvLH[k][2], &pSharedMemory->off_traj_drvLH[k][3],
							&pSharedMemory->off_traj_drvRF[k][0], &pSharedMemory->off_traj_drvRF[k][1], &pSharedMemory->off_traj_drvRF[k][2], &pSharedMemory->off_traj_drvRF[k][3],
							&pSharedMemory->off_traj_drvLF[k][0], &pSharedMemory->off_traj_drvLF[k][1], &pSharedMemory->off_traj_drvLF[k][2], &pSharedMemory->off_traj_drvLF[k][3]);
				
					pSharedMemory->off_traj_count = 0;
					fclose(fp);		
					pSharedMemory->position_mode_flag = 5;
				}		
			}
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");

}

void CDRCtestDlg::OnDrcLad11() 
{
	unsigned int k;
	FILE *fp;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo == WB_LADDER_CLIMBING_TEST)
		{
			if(pSharedMemory->position_mode_flag == 0 && pSharedMemory->ladder_demo == 1)
			{
				if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\offline_traj_ladder_2.txt","r")) == NULL )
				{
					AfxMessageBox("Data file offline_traj_ladder_2.txt was not found");		
					return;
				}
				
				fscanf(fp,"%d", &pSharedMemory->off_traj_length);

				if(pSharedMemory->off_traj_length > MAX_OFF_TRAJ)
				{
					fclose(fp);
					AfxMessageBox("MAX_OFF_TRAJ error!");
				}
				else
				{
					for (k=0; k < pSharedMemory->off_traj_length; k++)
						fscanf(fp,"%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e", 
							&pSharedMemory->off_traj_dpPELz[k], &pSharedMemory->off_traj_dpCOM[k][0], &pSharedMemory->off_traj_dpCOM[k][1], 
							&pSharedMemory->off_traj_dpRH[k][0], &pSharedMemory->off_traj_dpRH[k][1], &pSharedMemory->off_traj_dpRH[k][2],
							&pSharedMemory->off_traj_dpLH[k][0], &pSharedMemory->off_traj_dpLH[k][1], &pSharedMemory->off_traj_dpLH[k][2],
							&pSharedMemory->off_traj_dpRF[k][0], &pSharedMemory->off_traj_dpRF[k][1], &pSharedMemory->off_traj_dpRF[k][2],
							&pSharedMemory->off_traj_dpLF[k][0], &pSharedMemory->off_traj_dpLF[k][1], &pSharedMemory->off_traj_dpLF[k][2],
							&pSharedMemory->off_traj_drvRH[k][0], &pSharedMemory->off_traj_drvRH[k][1], &pSharedMemory->off_traj_drvRH[k][2], &pSharedMemory->off_traj_drvRH[k][3],
							&pSharedMemory->off_traj_drvLH[k][0], &pSharedMemory->off_traj_drvLH[k][1], &pSharedMemory->off_traj_drvLH[k][2], &pSharedMemory->off_traj_drvLH[k][3],
							&pSharedMemory->off_traj_drvRF[k][0], &pSharedMemory->off_traj_drvRF[k][1], &pSharedMemory->off_traj_drvRF[k][2], &pSharedMemory->off_traj_drvRF[k][3],
							&pSharedMemory->off_traj_drvLF[k][0], &pSharedMemory->off_traj_drvLF[k][1], &pSharedMemory->off_traj_drvLF[k][2], &pSharedMemory->off_traj_drvLF[k][3]);
				
					pSharedMemory->off_traj_count = 0;
					fclose(fp);		
					pSharedMemory->position_mode_flag = 5;
				}		
			}
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	
}


void CDRCtestDlg::OnDrcLad12() 
{
	CString strText;
	float temp;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && (pSharedMemory->WB_DemoNo == WB_LADDER_CLIMBING_TEST || pSharedMemory->WB_DemoNo == WB_FREQ_TEST))
		{
			if(pSharedMemory->position_mode_flag == 0)
			{
				GetDlgItem(IDC_EDIT_TEMP60)->GetWindowText(strText);
				pSharedMemory->dpPELz = (float)atof(strText);
				
				GetDlgItem(IDC_EDIT_TEMP22)->GetWindowText(strText);
				temp = (float)fabs((float)atof(strText));
				if(temp < 1.f)
					temp = 1.f;
				pSharedMemory->move_sec = temp;

				pSharedMemory->position_mode_flag = 4;
			}
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");	
}


void CDRCtestDlg::OnDrcLad13() 
{
	unsigned int k;
	FILE *fp;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo == WB_LADDER_CLIMBING_TEST)
		{
			if(pSharedMemory->position_mode_flag == 0 && pSharedMemory->ladder_demo == 1)
			{
				if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\offline_traj_ladder_1b.txt","r")) == NULL )
				{
					AfxMessageBox("Data file offline_traj_ladder_1b.txt was not found");		
					return;
				}
				
				fscanf(fp,"%d", &pSharedMemory->off_traj_length);

				if(pSharedMemory->off_traj_length > MAX_OFF_TRAJ)
				{
					fclose(fp);
					AfxMessageBox("MAX_OFF_TRAJ error!");
				}
				else
				{
					for (k=0; k < pSharedMemory->off_traj_length; k++)
						fscanf(fp,"%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e", 
							&pSharedMemory->off_traj_dpPELz[k], &pSharedMemory->off_traj_dpCOM[k][0], &pSharedMemory->off_traj_dpCOM[k][1], 
							&pSharedMemory->off_traj_dpRH[k][0], &pSharedMemory->off_traj_dpRH[k][1], &pSharedMemory->off_traj_dpRH[k][2],
							&pSharedMemory->off_traj_dpLH[k][0], &pSharedMemory->off_traj_dpLH[k][1], &pSharedMemory->off_traj_dpLH[k][2],
							&pSharedMemory->off_traj_dpRF[k][0], &pSharedMemory->off_traj_dpRF[k][1], &pSharedMemory->off_traj_dpRF[k][2],
							&pSharedMemory->off_traj_dpLF[k][0], &pSharedMemory->off_traj_dpLF[k][1], &pSharedMemory->off_traj_dpLF[k][2],
							&pSharedMemory->off_traj_drvRH[k][0], &pSharedMemory->off_traj_drvRH[k][1], &pSharedMemory->off_traj_drvRH[k][2], &pSharedMemory->off_traj_drvRH[k][3],
							&pSharedMemory->off_traj_drvLH[k][0], &pSharedMemory->off_traj_drvLH[k][1], &pSharedMemory->off_traj_drvLH[k][2], &pSharedMemory->off_traj_drvLH[k][3],
							&pSharedMemory->off_traj_drvRF[k][0], &pSharedMemory->off_traj_drvRF[k][1], &pSharedMemory->off_traj_drvRF[k][2], &pSharedMemory->off_traj_drvRF[k][3],
							&pSharedMemory->off_traj_drvLF[k][0], &pSharedMemory->off_traj_drvLF[k][1], &pSharedMemory->off_traj_drvLF[k][2], &pSharedMemory->off_traj_drvLF[k][3]);
				
					pSharedMemory->off_traj_count = 0;
					fclose(fp);		
					pSharedMemory->position_mode_flag = 5;
				}		
			}
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	
}


void CDRCtestDlg::OnDrcLad14() 
{
	unsigned int k;
	FILE *fp;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo == WB_LADDER_CLIMBING_TEST)
		{
			if(pSharedMemory->position_mode_flag == 0 && pSharedMemory->ladder_demo == 1)
			{
				if ( (fp =fopen( "C:\\Rainbow_DRC\\WB_Data\\offline_traj_ladder_2b.txt","r")) == NULL )
				{
					AfxMessageBox("Data file offline_traj_ladder_2b.txt was not found");		
					return;
				}
				
				fscanf(fp,"%d", &pSharedMemory->off_traj_length);

				if(pSharedMemory->off_traj_length > MAX_OFF_TRAJ)
				{
					fclose(fp);
					AfxMessageBox("MAX_OFF_TRAJ error!");
				}
				else
				{
					for (k=0; k < pSharedMemory->off_traj_length; k++)
						fscanf(fp,"%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e", 
							&pSharedMemory->off_traj_dpPELz[k], &pSharedMemory->off_traj_dpCOM[k][0], &pSharedMemory->off_traj_dpCOM[k][1], 
							&pSharedMemory->off_traj_dpRH[k][0], &pSharedMemory->off_traj_dpRH[k][1], &pSharedMemory->off_traj_dpRH[k][2],
							&pSharedMemory->off_traj_dpLH[k][0], &pSharedMemory->off_traj_dpLH[k][1], &pSharedMemory->off_traj_dpLH[k][2],
							&pSharedMemory->off_traj_dpRF[k][0], &pSharedMemory->off_traj_dpRF[k][1], &pSharedMemory->off_traj_dpRF[k][2],
							&pSharedMemory->off_traj_dpLF[k][0], &pSharedMemory->off_traj_dpLF[k][1], &pSharedMemory->off_traj_dpLF[k][2],
							&pSharedMemory->off_traj_drvRH[k][0], &pSharedMemory->off_traj_drvRH[k][1], &pSharedMemory->off_traj_drvRH[k][2], &pSharedMemory->off_traj_drvRH[k][3],
							&pSharedMemory->off_traj_drvLH[k][0], &pSharedMemory->off_traj_drvLH[k][1], &pSharedMemory->off_traj_drvLH[k][2], &pSharedMemory->off_traj_drvLH[k][3],
							&pSharedMemory->off_traj_drvRF[k][0], &pSharedMemory->off_traj_drvRF[k][1], &pSharedMemory->off_traj_drvRF[k][2], &pSharedMemory->off_traj_drvRF[k][3],
							&pSharedMemory->off_traj_drvLF[k][0], &pSharedMemory->off_traj_drvLF[k][1], &pSharedMemory->off_traj_drvLF[k][2], &pSharedMemory->off_traj_drvLF[k][3]);
				
					pSharedMemory->off_traj_count = 0;
					fclose(fp);		
					pSharedMemory->position_mode_flag = 5;
				}		
			}
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	
}


void CDRCtestDlg::OnDrcSr3() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo == WB_STEERING_TEST)
		{
			if(pSharedMemory->torque_mode_flag == 0)
				pSharedMemory->torque_mode_flag = 1;
			pSharedMemory->steer_demo = 2;
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}


void CDRCtestDlg::OnDrcGrab7() 
{
	CString strText;
	GetDlgItem(IDC_EDIT_TEMP61)->GetWindowText(strText);
	pSharedMemory->CommandDataFloat[0] = (float)atof(strText);	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		Sleep(10);
		pSharedMemory->CommandFlag = GRIP_ON_TRIG_R;
		Sleep(10);
	}
}

void CDRCtestDlg::OnDrcGrab8() 
{
	CString strText;	
	GetDlgItem(IDC_EDIT_TEMP61)->GetWindowText(strText);
	pSharedMemory->CommandDataFloat[0] = (float)atof(strText);
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		Sleep(10);
		pSharedMemory->CommandFlag = GRIP_OFF_TRIG_R;
		Sleep(10);
	}
}

void CDRCtestDlg::OnDrcGrab9() 
{
	pSharedMemory->CommandDataFloat[0] = 0;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		Sleep(10);
		pSharedMemory->CommandFlag = GRIP_STOP_TRIG_R;
		Sleep(10);
	}
}

void CDRCtestDlg::OnDrcGrab10() 
{
	CString strText;
	GetDlgItem(IDC_EDIT_TEMP62)->GetWindowText(strText);
	pSharedMemory->CommandDataFloat[0] = (float)atof(strText);	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		Sleep(10);
		pSharedMemory->CommandFlag = GRIP_ON_TRIG_L;
		Sleep(10);
	}
	
}

void CDRCtestDlg::OnDrcGrab11() 
{
	CString strText;
	GetDlgItem(IDC_EDIT_TEMP62)->GetWindowText(strText);
	pSharedMemory->CommandDataFloat[0] = (float)atof(strText);	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		Sleep(10);
		pSharedMemory->CommandFlag = GRIP_ON_TRIG_L;
		Sleep(10);
	}
}

void CDRCtestDlg::OnDrcGrab12() 
{
	pSharedMemory->CommandDataFloat[0] = 0;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		Sleep(10);
		pSharedMemory->CommandFlag = GRIP_STOP_TRIG_L;
		Sleep(10);
	}
}

void CDRCtestDlg::OnDrcNeck1() 
{
	CString str_temp;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_EDIT_TEMP17)->GetWindowText(str_temp);
		pSharedMemory->angle_NKY = (float)atof(str_temp);
		
		GetDlgItem(IDC_EDIT_TEMP18)->GetWindowText(str_temp);
		pSharedMemory->angle_NK1 = (float)atof(str_temp);

		GetDlgItem(IDC_EDIT_TEMP63)->GetWindowText(str_temp);
		pSharedMemory->angle_NK2 = (float)atof(str_temp);

		pSharedMemory->CommandFlag = MOVE_NECK;
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDRCtestDlg::OnDrcNeck2() 
{
	CString str_temp;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_EDIT_TEMP64)->GetWindowText(str_temp);
		pSharedMemory->spd_NKY = (float)atof(str_temp);
		
		GetDlgItem(IDC_EDIT_TEMP65)->GetWindowText(str_temp);
		pSharedMemory->spd_NK1 = (float)atof(str_temp);

		GetDlgItem(IDC_EDIT_TEMP66)->GetWindowText(str_temp);
		pSharedMemory->spd_NK2 = (float)atof(str_temp);

		pSharedMemory->CommandFlag = SET_NECK_SPEED;
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	
}

void CDRCtestDlg::OnDrcNeck3() 
{
	CString str_temp;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_EDIT_TEMP67)->GetWindowText(str_temp);
		pSharedMemory->scan_range = (float)atof(str_temp);
		
		GetDlgItem(IDC_EDIT_TEMP68)->GetWindowText(str_temp);
		pSharedMemory->scan_freq = (float)fabs((float)atof(str_temp));

		pSharedMemory->scan_flag = 1;
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	
}

void CDRCtestDlg::OnDrcNeck4() 
{
	pSharedMemory->scan_flag = 4;
	
}

void CDRCtestDlg::OnDrcStart2() 
{
	FILE *fp;
	float temp;
	CString str_temp;
	int i;

	GetDlgItem(IDC_DRC_START2)->EnableWindow(FALSE);
	GetDlgItem(IDC_DRC_START)->EnableWindow(FALSE);
	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0 && pSharedMemory->change_drc_hand_flag == DRC_STICK_MODE)		// by Inhyeok
	{
		GetDlgItem(IDC_EDIT_TEMP)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);
		pSharedMemory->WB_const_temp = temp; //3.f;

		GetDlgItem(IDC_EDIT_TEMP2)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);
		pSharedMemory->WB_const_temp2 = temp; //5.f;

		GetDlgItem(IDC_EDIT_TEMP3)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);
		pSharedMemory->WB_const_temp3 = temp; //3.f;

		GetDlgItem(IDC_EDIT_TEMP4)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);
		pSharedMemory->WB_const_temp4 = temp; //10.f;

		GetDlgItem(IDC_EDIT_TEMP11)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);	
		pSharedMemory->temp_fHand_threshold = temp;	

		GetDlgItem(IDC_EDIT_TEMP12)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);	
		pSharedMemory->temp_EL_compen = temp;

		GetDlgItem(IDC_EDIT_TEMP13)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);	
		pSharedMemory->temp_ssp_damp_on = temp;

		GetDlgItem(IDC_EDIT_TEMP14)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);	
		if(temp == 0.f)
			pSharedMemory->temp_dsp_damp_on_flag = 0;
		else
			pSharedMemory->temp_dsp_damp_on_flag = 1;

		GetDlgItem(IDC_EDIT_TEMP15)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);	
		if(temp == 0.f)
			pSharedMemory->temp_foot_LC_on_flag = 0;
		else
			pSharedMemory->temp_foot_LC_on_flag = 1;

		GetDlgItem(IDC_EDIT_TEMP16)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);	
		if(temp == 0.f)
			pSharedMemory->temp_hand_LC_on_flag = 0;
		else
			pSharedMemory->temp_hand_LC_on_flag = 1;


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

		pSharedMemory->transform_flag = 0;
		pSharedMemory->position_mode_flag = 0;
		pSharedMemory->torque_mode_flag = 0;
		pSharedMemory->sw_mode = SW_MODE_NON_COMPLEMENTARY;
		pSharedMemory->CommandFlag = SET_SW_MODE;
		Sleep(200);
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_DRC_TEST;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
	GetDlgItem(IDC_DRC_START2)->EnableWindow(TRUE);
	GetDlgItem(IDC_DRC_START)->EnableWindow(TRUE);
}

void CDRCtestDlg::OnDrcHandMode() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->change_drc_hand_flag = DRC_HAND_MODE;
		pSharedMemory->CommandFlag = CHANGE_DRC_HAND;		
	}
	else 
		AfxMessageBox("Other Command is activated..!!");
		
	
}

void CDRCtestDlg::OnDrcStickMode() 
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

void CDRCtestDlg::OnDrcStart3() 
{
	GetDlgItem(IDC_DRC_START)->EnableWindow(FALSE);
	if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo==WB_OFFLINE_DEMO_DRC_TEST && pSharedMemory->transform_flag == 3)
	{
		pSharedMemory->transform_flag = 5;		
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
	GetDlgItem(IDC_DRC_START)->EnableWindow(TRUE);
}

void CDRCtestDlg::OnDrcLad15() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		if(pSharedMemory->MotorControlMode == CTRLMODE_WB_OFFLINE_DEMO && pSharedMemory->WB_DemoNo == WB_LADDER_CLIMBING_TEST)
		{
			if(pSharedMemory->torque_mode_flag == 2)
				pSharedMemory->ladder_demo = 2;
		}
	}
	else
		AfxMessageBox("Other Command is activated..!!");	
}

void CDRCtestDlg::OnDrcHandYaw() 
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

void CDRCtestDlg::OnDrcHandYaw2() 
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

void CDRCtestDlg::OnDrcGainovr() 
{
	CString str_temp;
	float temp;

	


	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_EDIT_GAINOVR)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);
		if(temp < 0.f)
			temp = 0.f;
		else if(temp > 1000.f)
			temp = 1000.f;
		pSharedMemory->gain_ovr_test = temp;

		GetDlgItem(IDC_EDIT_GAINOVR2)->GetWindowText(str_temp);
		temp = (float)atof(str_temp);
		if(temp < 0.f)
			temp = 0.f;
		pSharedMemory->gain_ovr_test2 = temp*1000.f;

		pSharedMemory->CommandFlag = GAINOVR_TEST;
	}
	else 
		AfxMessageBox("Other Command is activated..!!");		

}
