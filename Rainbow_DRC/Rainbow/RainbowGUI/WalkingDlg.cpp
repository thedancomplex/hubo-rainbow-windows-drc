// WalkingDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "WalkingDlg.h"
#include "..\SharedMemory.h"
#include "..\CommonDefinition.h"
#include <mmsystem.h>


#pragma comment(lib,"winmm")

// SharedMemory variable
extern PSHARED_DATA pSharedMemory;

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CWalkingDlg property page

IMPLEMENT_DYNCREATE(CWalkingDlg, CPropertyPage)

CWalkingDlg::CWalkingDlg() : CPropertyPage(CWalkingDlg::IDD)
{
	//{{AFX_DATA_INIT(CWalkingDlg)
	m_swayValue = 0.06f;
	m_stopSway = 1.3f;
	m_stepCount = 0;
	m_startSway = 1.3f;
	m_sideStep = 0.0f;
	m_rotationStep = 0.0f;
	m_stepPeriod = 800;
	m_dspTime = 20;
	m_holdTime = 60;
	m_forwardStep = 0.0f;
	m_greb = 90.0f;
	m_grsr = -80.0f;
	m_grwy = -80.0f;
	m_mx = 0.01f;
	m_my = 0.006f;
	m_rlebgain = 105;
	m_rlshpgain = 65;
	m_RADIO1 = 0;
	m_landingstate_fz = 100;
	//}}AFX_DATA_INIT
}

CWalkingDlg::~CWalkingDlg()
{
}

void CWalkingDlg::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CWalkingDlg)
	DDX_Control(pDX, IDC_HANDLIST, m_handSelBox);
	DDX_Control(pDX, IDC_MOTIONLIST, m_motionSelBox);
	DDX_Control(pDX, IDC_DEMOLIST, m_demoSelBox);
	DDX_Text(pDX, IDC_SWAY, m_swayValue);
	DDV_MinMaxFloat(pDX, m_swayValue, 4.e-002f, 8.e-002f);
	DDX_Text(pDX, IDC_STOPSWAY, m_stopSway);
	DDV_MinMaxFloat(pDX, m_stopSway, 0.5f, 1.5f);
	DDX_Text(pDX, IDC_STEPCOUNT, m_stepCount);
	DDV_MinMaxInt(pDX, m_stepCount, -1, 100);
	DDX_Text(pDX, IDC_STARTSWAY, m_startSway);
	DDV_MinMaxFloat(pDX, m_startSway, 0.5f, 1.5f);
	DDX_Text(pDX, IDC_SIDE, m_sideStep);
	DDV_MinMaxFloat(pDX, m_sideStep, -0.1f, 0.1f);
	DDX_Text(pDX, IDC_ROTATION, m_rotationStep);
	DDV_MinMaxFloat(pDX, m_rotationStep, -45.f, 45.f);
	DDX_Text(pDX, IDC_PERIOD, m_stepPeriod);
	DDX_Text(pDX, IDC_DSP, m_dspTime);
	DDV_MinMaxUInt(pDX, m_dspTime, 0, 100);
	DDX_Text(pDX, IDC_HOLD, m_holdTime);
	DDV_MinMaxUInt(pDX, m_holdTime, 0, 200);
	DDX_Text(pDX, IDC_FORWARD, m_forwardStep);
	DDV_MinMaxFloat(pDX, m_forwardStep, -0.1f, 0.4f);
	DDX_Text(pDX, IDC_GREB, m_greb);
	DDX_Text(pDX, IDC_GRSR, m_grsr);
	DDX_Text(pDX, IDC_GRWY, m_grwy);
	DDX_Text(pDX, IDC_MX, m_mx);
	DDX_Text(pDX, IDC_MY, m_my);
	DDX_Text(pDX, IDC_PUSHUP_RLEB, m_rlebgain);
	DDX_Text(pDX, IDC_PUSHUP_RLSP, m_rlshpgain);
	DDX_Radio(pDX, IDC_RADIO2, m_RADIO1);
	DDX_Text(pDX, IDC_LANDINGSTATE_FZ, m_landingstate_fz);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CWalkingDlg, CPropertyPage)
	//{{AFX_MSG_MAP(CWalkingDlg)
	ON_BN_CLICKED(IDC_ANKLEONOFF, OnAnkleonoff)
	ON_BN_CLICKED(IDC_ZMPINIT, OnZmpinit)
	ON_BN_CLICKED(IDC_WALKREADY, OnWalkready)
	ON_BN_CLICKED(IDC_WALKINGSET, OnWalkingset)
	ON_BN_CLICKED(IDC_GO, OnGo)
	ON_BN_CLICKED(IDC_PLUSROTATION, OnPlusrotation)
	ON_BN_CLICKED(IDC_MINUSROTATION, OnMinusrotation)
	ON_BN_CLICKED(IDC_ZEROROTATION, OnZerorotation)
	ON_BN_CLICKED(IDC_STOP, OnStop)
	ON_BN_CLICKED(IDC_MOTION, OnMotion)
	ON_CBN_SELCHANGE(IDC_DEMOLIST, OnSelchangeDemolist)
	ON_BN_CLICKED(IDC_DEMO1, OnDemo1)
	ON_BN_CLICKED(IDC_DEMO2, OnDemo2)
	ON_BN_CLICKED(IDC_DEMO3, OnDemo3)
	ON_BN_CLICKED(IDC_DEMO4, OnDemo4)
	ON_BN_CLICKED(IDC_DATASAVE, OnDatasave)
	ON_CBN_SELCHANGE(IDC_HANDLIST, OnSelchangeHandlist)
	ON_BN_CLICKED(IDC_HAND1, OnHand1)
	ON_BN_CLICKED(IDC_HAND2, OnHand2)
	ON_BN_CLICKED(IDC_HAND3, OnHand3)
	ON_BN_CLICKED(IDC_HAND4, OnHand4)
	ON_BN_CLICKED(IDC_RANGEONOFF, OnRangeonoff)
	ON_BN_CLICKED(IDC_PLUSCURRENT, OnPluscurrent)
	ON_BN_CLICKED(IDC_MINUSCURRENT, OnMinuscurrent)
	ON_BN_CLICKED(IDC_INITHANDS, OnInithands)
	ON_BN_CLICKED(IDC_ZMPINITPOS, OnZmpinitpos)
	ON_BN_CLICKED(IDC_ZMPPOSSAVE, OnZmppossave)
	ON_BN_CLICKED(IDC_UNEVEN, OnUneven)
	ON_BN_CLICKED(IDC_FOWARD20CM, OnFoward20cm)
	ON_BN_CLICKED(IDC_FOWARD15CM, OnFoward15cm)
	ON_BN_CLICKED(IDC_FOWARD10CM, OnFoward10cm)
	ON_BN_CLICKED(IDC_FOWARD0CM, OnFoward0cm)
	ON_BN_CLICKED(IDC_FOWARDM10CM, OnFowardm10cm)
	ON_BN_CLICKED(IDC_PLUSROTATION2, OnPlusrotation2)
	ON_BN_CLICKED(IDC_MINUSROTATION2, OnMinusrotation2)
	ON_BN_CLICKED(IDC_PLUSROTATION3, OnPlusrotation3)
	ON_BN_CLICKED(IDC_MINUSROTATION3, OnMinusrotation3)
	ON_BN_CLICKED(IDC_BUTTON1, OnButton1)
	ON_BN_CLICKED(IDC_BUTTON2, OnButton2)
	ON_BN_CLICKED(IDC_BUTTON3, OnButton3)
	ON_BN_CLICKED(IDC_BUTTON4, OnButton4)
	ON_BN_CLICKED(IDC_BUTTON5, OnButton5)
	ON_BN_CLICKED(IDC_DEMOTEST, OnDemotest)
	ON_BN_CLICKED(IDC_DEMOTEST2, OnDemotest2)
	ON_BN_CLICKED(IDC_LOG, OnLog)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_DRC, OnDrc)
	ON_BN_CLICKED(IDC_KIRK, OnKirk)
	ON_BN_CLICKED(IDC_KIRK2, OnKirk2)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CWalkingDlg message handlers

void CWalkingDlg::OnAnkleonoff() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	GetDlgItem(IDC_ANKLEONOFF)->GetWindowText(strText);

	if(strText == "Ankle OFF")
	{
		if(pSharedMemory->CommandFlag == NO_ACT)
		{
			pSharedMemory->BoardID = JMC3;
			pSharedMemory->CommandFlag = STOP_CMD_EACH;
			Sleep(20);
			pSharedMemory->BoardID = JMC3;
			pSharedMemory->CommandFlag = DISABLE_FET_EACH;
			Sleep(20);
	
			pSharedMemory->BoardID = JMC7;
			pSharedMemory->CommandFlag = STOP_CMD_EACH;
			Sleep(20);
			pSharedMemory->BoardID = JMC7;
			pSharedMemory->CommandFlag = DISABLE_FET_EACH;
			Sleep(20);

			GetDlgItem(IDC_ANKLEONOFF)->SetWindowText("Ankle ON");
		}
		else AfxMessageBox("Other Command is activated..!!");
	}
	else
	{
		if(pSharedMemory->CommandFlag == NO_ACT)
		{
			pSharedMemory->BoardID = JMC3;
			pSharedMemory->CommandFlag = ENCODER_ZERO_EACH;
			Sleep(20);
			pSharedMemory->BoardID = JMC3;
			pSharedMemory->CommandFlag = ENABLE_FET_EACH;
			Sleep(20);
			pSharedMemory->BoardID = JMC3;
			pSharedMemory->CommandFlag = RUN_CMD_EACH;
			Sleep(20);
	
			pSharedMemory->BoardID = JMC7;
			pSharedMemory->CommandFlag = ENCODER_ZERO_EACH;
			Sleep(20);
			pSharedMemory->BoardID = JMC7;
			pSharedMemory->CommandFlag = ENABLE_FET_EACH;
			Sleep(20);
			pSharedMemory->BoardID = JMC7;
			pSharedMemory->CommandFlag = RUN_CMD_EACH;
			Sleep(20);

			GetDlgItem(IDC_ANKLEONOFF)->SetWindowText("Ankle OFF");
		}
		else AfxMessageBox("Other Command is activated..!!");
	}
}

void CWalkingDlg::OnZmpinit() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_ZMPINIT)->GetWindowText(strText);
		if(strText == "ZMP Init. Start") 
		{
			pSharedMemory->CommandData = 0x00;
			pSharedMemory->CommandFlag = START_ZMP_INITIALIZATION;
			GetDlgItem(IDC_ZMPINIT)->SetWindowText("ZMP Init. Stop");

			GetDlgItem(IDC_ZMPPOSSAVE)->EnableWindow(false);
		}
		else
		{
			pSharedMemory->CommandFlag = STOP_ZMP_INITIALIZATION;
			GetDlgItem(IDC_ZMPINIT)->SetWindowText("ZMP Init. Start");
			//HUBO2Win->pSharedMemory->MotorControlMode = CTRLMODE_WALKING;

			GetDlgItem(IDC_ZMPPOSSAVE)->EnableWindow(true);
		}
		Sleep(10);
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnWalkready() 
{
	UpdateData(TRUE);
	// TODO: Add your control notification handler code here
	CString strText;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{

		GetDlgItem(IDC_WALKREADY)->GetWindowText(strText);
		if(strText == "Walk Ready")
		{
			pSharedMemory->WalkReadyPosFlag = m_RADIO1;
			pSharedMemory->CommandFlag = GOTO_WALK_READY_POS;
			Sleep(10);
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
			GetDlgItem(IDC_ZMPINIT)->EnableWindow(true);
			GetDlgItem(IDC_ZMPINITPOS)->EnableWindow(true);
			GetDlgItem(IDC_ANKLEONOFF)->EnableWindow(false);
			GetDlgItem(IDC_GO)->EnableWindow(true);

			GetDlgItem(IDC_WALKREADY)->SetWindowText("Goto Home");
		}
		else
		{
			pSharedMemory->CommandFlag = GOTO_HOME_POS;
			Sleep(10);
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
			GetDlgItem(IDC_ZMPINIT)->EnableWindow(false);
			GetDlgItem(IDC_ZMPINITPOS)->EnableWindow(false);
			GetDlgItem(IDC_ANKLEONOFF)->EnableWindow(true);
			GetDlgItem(IDC_GO)->EnableWindow(false);

			GetDlgItem(IDC_WALKREADY)->SetWindowText("Walk Ready");
		}	
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnWalkingset() 
{
	// TODO: Add your control notification handler code here
	UpdateData(true);

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
		pSharedMemory->LandingFz = m_landingstate_fz;
}

void CWalkingDlg::OnGo() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->WalkingDemo = 0x00;
		pSharedMemory->Data_Debug_Index = 0;
		pSharedMemory->CommandFlag = GOTO_FORWARD;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnPlusrotation() 
{
	// TODO: Add your control notification handler code here
	
	m_rotationStep = pSharedMemory->JW_temp[12] = 5.0f;
	UpdateData(false);
}

void CWalkingDlg::OnMinusrotation() 
{
	// TODO: Add your control notification handler code here
	m_rotationStep = pSharedMemory->JW_temp[12] = -5.0f;
	UpdateData(false);
}

void CWalkingDlg::OnZerorotation() 
{
	// TODO: Add your control notification handler code here
	m_rotationStep = pSharedMemory->JW_temp[12] = 0.0f;
	UpdateData(false);
}

void CWalkingDlg::OnStop() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = STOP_WALKING;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

BOOL CWalkingDlg::OnInitDialog() 
{
	CPropertyPage::OnInitDialog();
	
	// TODO: Add extra initialization here

	m_pFile = NULL; // by Inhyeok
	SetTimer(999,100,NULL); // by Inhyeok

	m_motionSelBox.InsertString(0, "Speech1");				m_motionSelBox.InsertString(1, "Speech2");
	m_motionSelBox.InsertString(2, "Sign language");		m_motionSelBox.InsertString(3, "Taichi");
	m_motionSelBox.InsertString(4, "Welcome");				m_motionSelBox.InsertString(5, "Goodbye");
	m_motionSelBox.InsertString(6, "Dance1");				m_motionSelBox.InsertString(7, "Dance2");
	m_motionSelBox.InsertString(8, "Dance3");				m_motionSelBox.InsertString(9, "Dance4");
	m_motionSelBox.InsertString(10, "Bow");					m_motionSelBox.InsertString(11, "Right hand-up");
	m_motionSelBox.InsertString(12, "Right hand-forward");	m_motionSelBox.InsertString(13, "Right hand-head");
	m_motionSelBox.InsertString(14, "Both hand-up");		m_motionSelBox.InsertString(15, "Hand wave");
	m_motionSelBox.InsertString(16, "Left hand-up");		m_motionSelBox.InsertString(17, "Left hand-forward");
	m_motionSelBox.InsertString(18, "Left hand-head");		m_motionSelBox.InsertString(19, "//");
	m_motionSelBox.InsertString(20, "//");					m_motionSelBox.InsertString(21, "//");

	m_motionSelBox.InsertString(22, "Hand Stone");			m_motionSelBox.InsertString(23, "Hand Paper");
	m_motionSelBox.InsertString(24, "Hand Scissor");		m_motionSelBox.InsertString(25, "Hand 14");
	m_motionSelBox.InsertString(26, "Hand 1");				m_motionSelBox.InsertString(27, "Hand RB");
	m_motionSelBox.InsertString(28, "HandNeck");

	m_motionSelBox.InsertString(29, "//");	// by Inhyeok
	m_motionSelBox.InsertString(30, "//");	// by Inhyeok
	m_motionSelBox.InsertString(31, "//");	// by Inhyeok

	m_motionSelBox.InsertString(32, "//");		// by JW
	m_motionSelBox.InsertString(33, "Neck A");	// by JW
	m_motionSelBox.InsertString(34, "Neck B");	// by JW
	m_motionSelBox.InsertString(35, "Neck C");	// by JW
	m_motionSelBox.InsertString(36, "Neck D");	// by JW
	m_motionSelBox.InsertString(37, "Neck E");	// by JW

	m_motionSelBox.SetCurSel(0);


	m_demoSelBox.InsertString(0, "One leg support");		m_demoSelBox.InsertString(1, "DSP Control");
	m_demoSelBox.InsertString(2, "//");						m_demoSelBox.InsertString(3, "//");
	m_demoSelBox.InsertString(4, "//");						m_demoSelBox.InsertString(5, "//");
	m_demoSelBox.SetCurSel(0);

	m_handSelBox.InsertString(0, "C-Grapsping");			m_handSelBox.InsertString(1, "//");//m_handSelBox.InsertString(1, "P-Grapsping");
	m_handSelBox.InsertString(2, "Handshake");				m_handSelBox.InsertString(3, "Salute");
	m_handSelBox.InsertString(4, "//");						m_handSelBox.InsertString(5, "//");
	m_handSelBox.SetCurSel(0);

	OnSelchangeDemolist();
	OnSelchangeHandlist();	

	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}

void CWalkingDlg::OnMotion()		// modified by Inhyeok
{
	int i;
	int flag_temp = FALSE;	// Inhyeok

	if( m_motionSelBox.GetCurSel() < 38)
	{
		
		for(i=0; i<NoOfMotion; i++) 
			pSharedMemory->JW_MotionSet[i] = FALSE;		
		
		if(( m_motionSelBox.GetCurSel() < 30 )||( m_motionSelBox.GetCurSel() > 32 ))		// JW motion
		{
			//AfxMessageBox("in");
			pSharedMemory->JW_MotionSet[m_motionSelBox.GetCurSel()] = TRUE;
			if(pSharedMemory->CommandFlag == NO_ACT)
			{
				
				if(pSharedMemory->JW_MotionSet[10])
				{
					PlaySound("C:\\voice\\welcome1.wav",NULL,SND_ASYNC);
				}
				else if(pSharedMemory->JW_MotionSet[0])
				{
					PlaySound("C:\\voice\\introduce2.wav",NULL,SND_ASYNC);
				}
	
				pSharedMemory->CommandFlag = SET_MOCAP;
				Sleep(10);				
			}
			else
				AfxMessageBox("Other Command is activated..!!");
		}
		else		
		{
			;
			/*
			if(pSharedMemory->CommandFlag == NO_ACT)
				switch(m_motionSelBox.GetCurSel())	// Inhyeok motion
				{
				case 30:
					break;
				case 31:
					break;				
				}
			
			if(pSharedMemory->CommandFlag == NO_ACT )
			{
				if(flag_temp == TRUE)
					pSharedMemory->CommandFlag = INIT_WB_MOCAP;
				Sleep(10);
			}			
			else 
				AfxMessageBox("Other Command is activated..!!");
			*/
		}	
	}
}


void CWalkingDlg::OnSelchangeDemolist() 
{
	// TODO: Add your control notification handler code here

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch( m_demoSelBox.GetCurSel() )
		{
		case 0:		// one-leg stand and up-and-down motion
			GetDlgItem(IDC_DEMO1)->SetWindowText("One Leg");
			GetDlgItem(IDC_DEMO2)->SetWindowText("Up and Down");
			GetDlgItem(IDC_DEMO3)->SetWindowText("Go Init. Pos");
			GetDlgItem(IDC_DEMO1)->ShowWindow(true);
			GetDlgItem(IDC_DEMO2)->ShowWindow(true);
			GetDlgItem(IDC_DEMO3)->ShowWindow(true);
			GetDlgItem(IDC_DEMO4)->ShowWindow(false);
			break;
		case 1:
			GetDlgItem(IDC_DEMO1)->SetWindowText("DSP CTRL ON");
			GetDlgItem(IDC_DEMO2)->SetWindowText("DSP CTRL OFF");
			GetDlgItem(IDC_DEMO3)->SetWindowText("DSP Position");
			GetDlgItem(IDC_DEMO4)->SetWindowText("Home Position");
			GetDlgItem(IDC_DEMO1)->ShowWindow(true);
			GetDlgItem(IDC_DEMO2)->ShowWindow(true);
			GetDlgItem(IDC_DEMO3)->ShowWindow(true);
			GetDlgItem(IDC_DEMO4)->ShowWindow(true);
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnDemo1() 
{
	// TODO: Add your control notification handler code here

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch( m_demoSelBox.GetCurSel() )
		{
		case 0:		// one-leg stand and up-and-down motion
			pSharedMemory->CommandData = ONE_LEG_STAND;
			pSharedMemory->CommandFlag = DEMO_FLAG;
			break;
		case 1:
			pSharedMemory->CommandData = DSP_ON;
			pSharedMemory->CommandFlag = DEMO_FLAG;
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnDemo2() 
{
	// TODO: Add your control notification handler code here

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch(m_demoSelBox.GetCurSel())
		{
		case 0:		// one-leg stand and up-and-down motion
			pSharedMemory->CommandData = ONE_LEG_UP_AND_DOWN;
			pSharedMemory->CommandFlag = DEMO_FLAG;
			break;
		case 1:
			pSharedMemory->CommandData = DSP_OFF;
			pSharedMemory->CommandFlag = DEMO_FLAG;
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnDemo3() 
{
	// TODO: Add your control notification handler code here

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch( m_demoSelBox.GetCurSel() )
		{
		case 0:		// one-leg stand and up-and-down motion
			pSharedMemory->CommandData = WALKREADY_POS;
			pSharedMemory->CommandFlag = DEMO_FLAG;
			break;
		case 1:
			pSharedMemory->CommandFlag = DSP_DEMO_POSITION;
			pSharedMemory->CommandFlag = DEMO_FLAG;
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnDemo4() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch( m_demoSelBox.GetCurSel() )
		{
		case 0:		// one-leg stand and up-and-down motion
			break;
		case 1:
			pSharedMemory->CommandFlag = DSP_DEMO_HOME_POSITION;
			pSharedMemory->CommandFlag = DEMO_FLAG;
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnDatasave() 
{
	// TODO: Add your control notification handler code here
	FILE* fp;
	unsigned int i, j;
	fp = fopen("angleData.txt", "w");
	for(i=0 ; i<N_ROW_SAVE ; i++) 
	{
		for(j=0 ; j<N_COLUMN_SAVE ; j++) fprintf(fp, "%f\t", pSharedMemory->Data_Debug[j][i]);
		fprintf(fp, "\n");
	}
	fclose(fp);

	FILE* canfp;
	canfp = fopen("canData.txt", "w");
	for(i=0 ; i<1000 ; i++) 
	{
		for(j=0 ; j<9 ; j++) fprintf(canfp, "%d\t", pSharedMemory->CAN_Read_Debug[i][j]);
		fprintf(canfp, "\n");
	}
	fclose(canfp);
}

void CWalkingDlg::OnSelchangeHandlist() 
{
	// TODO: Add your control notification handler code here

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch( m_handSelBox.GetCurSel() )
		{
		case 0:	// c-mode grasping
			GetDlgItem(IDC_HAND1)->SetWindowText("Grasp Pos");
			GetDlgItem(IDC_HAND2)->SetWindowText("Grasp On");
			GetDlgItem(IDC_HAND3)->SetWindowText("Grasp Off");
			GetDlgItem(IDC_HAND4)->SetWindowText("Grasp Stop");
			
			GetDlgItem(IDC_HAND1)->ShowWindow(true);
			GetDlgItem(IDC_HAND2)->ShowWindow(true);
			GetDlgItem(IDC_HAND3)->ShowWindow(true);
			GetDlgItem(IDC_HAND4)->ShowWindow(true);

			GetDlgItem(IDC_HAND1)->EnableWindow(true);
			GetDlgItem(IDC_HAND2)->EnableWindow(true);
			GetDlgItem(IDC_HAND3)->EnableWindow(true);
			GetDlgItem(IDC_HAND4)->EnableWindow(true);

			pSharedMemory->JointID = RF1;
			pSharedMemory->CommandFlag = C_CONTROL_MODE;
			break;
/*		case 1:	// p-mode grasping
			GetDlgItem(IDC_HAND1)->SetWindowText("Grasp Pos");
			GetDlgItem(IDC_HAND2)->SetWindowText("Grasp On");
			GetDlgItem(IDC_HAND3)->SetWindowText("Grasp Off");
			GetDlgItem(IDC_HAND4)->SetWindowText("Grasp Stop");
			
			GetDlgItem(IDC_HAND1)->ShowWindow(true);
			GetDlgItem(IDC_HAND2)->ShowWindow(true);
			GetDlgItem(IDC_HAND3)->ShowWindow(true);
			GetDlgItem(IDC_HAND4)->ShowWindow(true);

			pSharedMemory->JointID = RF1;
			pSharedMemory->CommandFlag = P_CONTROL_MODE;
			break;
*/		case 2:
			GetDlgItem(IDC_HAND1)->SetWindowText("Grasp Pos");
			GetDlgItem(IDC_HAND2)->SetWindowText("Start");
			GetDlgItem(IDC_HAND3)->SetWindowText("Grasp On");
			GetDlgItem(IDC_HAND4)->SetWindowText("Grasp Off");
			
			GetDlgItem(IDC_HAND1)->ShowWindow(true);
			GetDlgItem(IDC_HAND2)->ShowWindow(true);
			GetDlgItem(IDC_HAND3)->ShowWindow(true);
			GetDlgItem(IDC_HAND4)->ShowWindow(true);

			GetDlgItem(IDC_HAND1)->EnableWindow(true);
			GetDlgItem(IDC_HAND2)->EnableWindow(true);
			GetDlgItem(IDC_HAND3)->EnableWindow(true);
			GetDlgItem(IDC_HAND4)->EnableWindow(true);
			break;
		case 3:
			GetDlgItem(IDC_HAND1)->SetWindowText("Salute");
			GetDlgItem(IDC_HAND2)->SetWindowText("Init. Pos");
			
			GetDlgItem(IDC_HAND1)->ShowWindow(true);
			GetDlgItem(IDC_HAND2)->ShowWindow(true);
			GetDlgItem(IDC_HAND2)->EnableWindow(false);
			GetDlgItem(IDC_HAND3)->ShowWindow(false);
			GetDlgItem(IDC_HAND4)->ShowWindow(false);
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnHand1() 
{
	// TODO: Add your control notification handler code here
	
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->MotorControlMode = CTRLMODE_HANDSHAKE;
		switch( m_handSelBox.GetCurSel() )
		{
		case 0:	// p-mode : go to grasping position
		case 1:	// c-mode : go to grasping position
		case 2:
			GetDlgItem(IDC_HAND1)->GetWindowText(strText);
			if(strText == "Grasp Pos")
			{
				GetDlgItem(IDC_HAND1)->SetWindowText("Init. Pos");
				// goto handshake pos
				pSharedMemory->ShakeHandsFlag = 1;
				pSharedMemory->CommandFlag = C_CONTROL_MODE;

				Sleep(50);
				pSharedMemory->CommandFlag = GRIP_OFF_R;
				Sleep(1000);
				pSharedMemory->CommandFlag = GRIP_STOP_R;

				GetDlgItem(IDC_CURRENT)->SetWindowText("30.0");
			}
			else
			{
				GetDlgItem(IDC_HAND1)->SetWindowText("Grasp Pos");
				// goto init pos.
				pSharedMemory->ShakeHandsFlag = 5;

				pSharedMemory->JointID = RF1;
				pSharedMemory->CommandFlag = P_CONTROL_MODE;
			}
			break;
		case 3:
			pSharedMemory->JointID = RSP;
			pSharedMemory->GoalAngle = -90.0f;
			pSharedMemory->GoalTime = 2000.0f;
			pSharedMemory->CommandFlag = JOINT_REF_SET_RELATIVE;
			//pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
			Sleep(100);

			pSharedMemory->JointID = RSR;
			pSharedMemory->GoalAngle = -10.0f;
			pSharedMemory->GoalTime = 2000.0f;
			pSharedMemory->CommandFlag = JOINT_REF_SET_RELATIVE;
			Sleep(100);

			pSharedMemory->JointID = REB;
			pSharedMemory->GoalAngle = -75.0f;
			pSharedMemory->GoalTime = 2000.0f;
			pSharedMemory->CommandFlag = JOINT_REF_SET_RELATIVE;
			Sleep(100);

			pSharedMemory->JointID = RSY;
			pSharedMemory->GoalAngle = 40.0f;
			pSharedMemory->GoalTime = 2000.0f;
			pSharedMemory->CommandFlag = JOINT_REF_SET_RELATIVE;
			Sleep(100);
			
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;

			pSharedMemory->CommandDataFloat[0] = 30.0f;
			pSharedMemory->CommandFlag = GRIP_OFF_R;
			Sleep(1000);
			pSharedMemory->CommandFlag = GRIP_STOP_R;

			GetDlgItem(IDC_HAND1)->EnableWindow(false);
			GetDlgItem(IDC_HAND2)->EnableWindow(true);
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnHand2() 
{
	// TODO: Add your control notification handler code here
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch( m_handSelBox.GetCurSel() )
		{
		case 0:	// p-mode : grasping on
		case 1:	// c-mode : grasping on
			GetDlgItem(IDC_CURRENT)->GetWindowText(strText);
			pSharedMemory->CommandDataFloat[0] = (float)atof(strText);
			Sleep(10);
			pSharedMemory->CommandFlag = GRIP_ON_R;
			Sleep(10);
			break;
		case 2:
			// hand FT nulling and start
			pSharedMemory->CommandFlag = NULL_WRIST_FT_SENSOR;
			Sleep(500);
			pSharedMemory->ShakeHandsFlag = 3;
			break;
		case 3:
			pSharedMemory->MotorControlMode = CTRLMODE_NONE;
			pSharedMemory->JointID = RSP;
			pSharedMemory->GoalAngle = 90.0f;
			pSharedMemory->GoalTime = 2000.0f;
			pSharedMemory->CommandFlag = JOINT_REF_SET_RELATIVE;
			//pSharedMemory->CommandFlag = JOINT_REF_SET_ABS;
			Sleep(100);

			pSharedMemory->JointID = RSR;
			pSharedMemory->GoalAngle = 10.0f;
			pSharedMemory->GoalTime = 2000.0f;
			pSharedMemory->CommandFlag = JOINT_REF_SET_RELATIVE;
			Sleep(100);

			pSharedMemory->JointID = REB;
			pSharedMemory->GoalAngle = 75.0f;
			pSharedMemory->GoalTime = 2000.0f;
			pSharedMemory->CommandFlag = JOINT_REF_SET_RELATIVE;
			Sleep(100);

			pSharedMemory->JointID = RSY;
			pSharedMemory->GoalAngle = -40.0f;
			pSharedMemory->GoalTime = 2000.0f;
			pSharedMemory->CommandFlag = JOINT_REF_SET_RELATIVE;
			Sleep(100);
			
			pSharedMemory->MotorControlMode = CTRLMODE_POSITION_CONTROL_WIN;
			Sleep(100);

			pSharedMemory->JointID = RF1;
			pSharedMemory->CommandFlag = GOTO_LIMIT_POS;

			GetDlgItem(IDC_HAND1)->EnableWindow(true);
			GetDlgItem(IDC_HAND2)->EnableWindow(false);
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnHand3() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch( m_handSelBox.GetCurSel() )
		{
		case 0:	// p-mode : grasping off
		case 1:	// c-mode : grasping off
			GetDlgItem(IDC_CURRENT)->GetWindowText(strText);
			pSharedMemory->CommandDataFloat[0] = (float)atof(strText);
			Sleep(10);
			pSharedMemory->CommandFlag = GRIP_OFF_R;
			Sleep(10);
			break;
		case 2:
			// grasp on
			GetDlgItem(IDC_CURRENT)->GetWindowText(strText);
			pSharedMemory->CommandDataFloat[0] = (float)atof(strText);
			Sleep(10);
			pSharedMemory->CommandFlag = GRIP_ON_R;
			Sleep(10);
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnHand4() 
{
	// TODO: Add your control notification handler code here
	CString strText;	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		switch( m_handSelBox.GetCurSel() )
		{
		case 0:	// p-mode : grasping stop
		case 1:	// c-mode : grasping stop
			pSharedMemory->CommandFlag = GRIP_STOP_R;
			Sleep(10);
			break;
		case 2:
			GetDlgItem(IDC_CURRENT)->GetWindowText(strText);
			pSharedMemory->CommandDataFloat[0] = (float)atof(strText);
			Sleep(10);
			pSharedMemory->CommandFlag = GRIP_OFF_R;
			Sleep(2000);
			pSharedMemory->CommandFlag = GRIP_STOP_R;
			Sleep(10);
			break;
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnRangeonoff() 
{
	// TODO: Add your control notification handler code here
	CString strText;

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_RANGEONOFF)->GetWindowText(strText);
		if(strText == "Range On")
		{
			pSharedMemory->CommandData = 0x01;
			pSharedMemory->CommandFlag = POSITION_LIMIT_ONOFF;
			GetDlgItem(IDC_RANGEONOFF)->SetWindowText("Range Off");
			GetDlgItem(IDC_WALKREADY)->EnableWindow(true);
		}
		else
		{
			pSharedMemory->CommandData = 0x00;
			pSharedMemory->CommandFlag = POSITION_LIMIT_ONOFF;
			GetDlgItem(IDC_RANGEONOFF)->SetWindowText("Range On");
			//GetDlgItem(IDC_WALKREADY)->EnableWindow(false);
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnPluscurrent() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	float temp;
	
	GetDlgItem(IDC_CURRENT)->GetWindowText(strText);
	temp = (float)atof(strText);
	temp += 5.0f;
	
	strText.Format("%f", temp);
	GetDlgItem(IDC_CURRENT)->SetWindowText(strText);
}

void CWalkingDlg::OnMinuscurrent() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	float temp;
	
	GetDlgItem(IDC_CURRENT)->GetWindowText(strText);
	temp = (float)atof(strText);
	temp -= 5.0f;

	strText.Format("%f", temp);
	GetDlgItem(IDC_CURRENT)->SetWindowText(strText);
}

void CWalkingDlg::OnInithands() 
{
	pSharedMemory->Data_Debug_Index = 0;
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->JointID = RF1;
		pSharedMemory->CommandFlag = GOTO_LIMIT_POS;
		Sleep(10);
		pSharedMemory->JointID = LF1;
		pSharedMemory->CommandFlag = GOTO_LIMIT_POS;
		Sleep(10);
		pSharedMemory->CommandFlag = C_CONTROL_MODE;
	}
	else AfxMessageBox("Other Command is activated..!!");
	
}

void CWalkingDlg::OnZmpinitpos() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandData = 0x01;
		pSharedMemory->CommandFlag = START_ZMP_INITIALIZATION;

		//GetDlgItem(IDC_ZMPINIT)->SetWindowText("ZMP Init. Stop");
		GetDlgItem(IDC_ZMPPOSSAVE)->EnableWindow(true);

		Sleep(10);
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnZmppossave() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = SAVE_ZMP_INIT_POS;		
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnUneven() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	pSharedMemory->Data_Debug_Index = 0;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_UNEVEN)->GetWindowText(strText);
		if(strText == "Uneven On")
		{
			pSharedMemory->JW_temp[32] = 1.0f;
			GetDlgItem(IDC_UNEVEN)->SetWindowText("Uneven Off");
			//GetDlgItem(IDC_WALKREADY)->EnableWindow(true);
		}
		else
		{
			pSharedMemory->JW_temp[32] = 0.0f;
			GetDlgItem(IDC_UNEVEN)->SetWindowText("Uneven On");
			//GetDlgItem(IDC_WALKREADY)->EnableWindow(false);
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnFoward20cm() 
{
	m_swayValue = 0.067f;//0.057f;
	m_stopSway = 1.2f;//1.3f;
	m_stepCount = -1;
	m_startSway = 1.1f;//1.35f;
	m_sideStep = 0.0f;
	m_rotationStep = 0.0f;
	//m_stepPeriod = 80;
	m_stepPeriod = 850;//800;
	m_dspTime = 0;//20;
	m_holdTime = 70;
	m_forwardStep = 0.2f;
	m_greb = 90.0f;
	m_grsr = -80.0f;
	m_grwy = -80.0f;
	m_mx = 0.01f;
	m_my = 0.006f;
	// TODO: Add your control notification handler code here
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
	UpdateData(false);
}

void CWalkingDlg::OnFoward15cm() 
{
	// TODO: Add your control notification handler code here
	
	m_swayValue = 0.068f;//0.0575f;
	m_stopSway = 1.2f;
	m_stepCount = -1;
	m_startSway = 1.1f;//1.2f;
	m_sideStep = 0.0f;
	m_rotationStep = 0.0f;
	//m_stepPeriod = 80;
	m_stepPeriod = 800;
	m_dspTime = 0;
	m_holdTime = 70;
	m_forwardStep = 0.15f;
	m_greb = 90.0f;
	m_grsr = -80.0f;
	m_grwy = -80.0f;
	m_mx = 0.01f;
	m_my = 0.006f;
	// TODO: Add your control notification handler code here
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
	UpdateData(false);
}

void CWalkingDlg::OnFoward10cm() 
{
	// TODO: Add your control notification handler code here
	
	m_swayValue = 0.069f;//0.058f;
	m_stopSway = 1.1f;
	m_stepCount = -1;
	m_startSway = 1.1f;//1.3f;
	m_sideStep = 0.0f;
	m_rotationStep = 0.0f;
	//m_stepPeriod = 80;
	m_stepPeriod = 800;
	m_dspTime = 0;
	m_holdTime = 70;
	m_forwardStep = 0.1f;
	m_greb = 90.0f;
	m_grsr = -80.0f;
	m_grwy = -80.0f;
	m_mx = 0.01f;
	m_my = 0.006f;
	// TODO: Add your control notification handler code here
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
	UpdateData(false);
}

void CWalkingDlg::OnFoward0cm() 
{
	// TODO: Add your control notification handler code here
	
	m_swayValue = 0.07f;//0.058f;
	m_stopSway = 1.1f;//1.3f;
	m_stepCount = -1;
	m_startSway = 1.1f;//1.3f;
	m_sideStep = 0.0f;
	m_rotationStep = 0.0f;
	//m_stepPeriod = 80;
	m_stepPeriod = 800;
	m_dspTime = 0;
	m_holdTime = 70;
	m_forwardStep = 0.0f;
	m_greb = 90.0f;
	m_grsr = -80.0f;
	m_grwy = -80.0f;
	m_mx = 0.01f;
	m_my = 0.006f;
	// TODO: Add your control notification handler code here
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
	UpdateData(false);
}



void CWalkingDlg::OnFowardm10cm() 
{
	// TODO: Add your control notification handler code here
	
	m_swayValue = 0.069f;//0.059f;
	m_stopSway = 1.1f;
	m_stepCount = -1;
	m_startSway = 1.1f;//1.2f;
	m_sideStep = 0.0f;
	m_rotationStep = 0.0f;
	//m_stepPeriod = 60;
	m_stepPeriod = 800;
	m_dspTime = 0;
	m_holdTime = 70;
	m_forwardStep = -0.1f;
	m_greb = 90.0f;
	m_grsr = -80.0f;
	m_grwy = -80.0f;
	m_mx = 0.01f;
	m_my = 0.006f;
	// TODO: Add your control notification handler code here
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
	UpdateData(false);
}

void CWalkingDlg::OnPlusrotation2() 
{
	// TODO: Add your control notification handler code here
	
	m_rotationStep = pSharedMemory->JW_temp[12] = 15.0f;
	UpdateData(false);
}

void CWalkingDlg::OnMinusrotation2() 
{
	// TODO: Add your control notification handler code here
	
	m_rotationStep = pSharedMemory->JW_temp[12] = -15.0f;
	UpdateData(false);
}

void CWalkingDlg::OnPlusrotation3() 
{
	// TODO: Add your control notification handler code here
	
	m_rotationStep = pSharedMemory->JW_temp[12] = 10.0f;
	UpdateData(false);
}

void CWalkingDlg::OnMinusrotation3() 
{	
	// TODO: Add your control notification handler code here
	
	m_rotationStep = pSharedMemory->JW_temp[12] = -10.0f;
	UpdateData(false);
}

void CWalkingDlg::OnButton1() 
{
	int i;

	if(pSharedMemory->CommandFlag == NO_ACT )
	{
		for(i=0; i<NoOfMotion; i++) 
			pSharedMemory->JW_MotionSet[i] = FALSE;		
		
		pSharedMemory->JW_MotionSet[0] = TRUE;	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);
	}			
	else 
	AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnButton2() 
{
	int i;

	if(pSharedMemory->CommandFlag == NO_ACT )
	{
		for(i=0; i<NoOfMotion; i++) 
			pSharedMemory->JW_MotionSet[i] = FALSE;		
		
		pSharedMemory->JW_MotionSet[1] = TRUE;	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);	
	}			
	else 
	AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnButton3() 
{
	int i;
		for(i=0; i<NoOfMotion; i++) 
			pSharedMemory->JW_MotionSet[i] = FALSE;		
		
		pSharedMemory->JW_MotionSet[10] = TRUE;	
		if(pSharedMemory->CommandFlag == NO_ACT )
		{
			pSharedMemory->CommandFlag = SET_MOCAP;
			Sleep(10);
		}
		Sleep(2500);

		PlaySound("C:\\voice\\120502_DCC_WACS_43_start.wav",NULL,SND_ASYNC);
		Sleep(2800);
		
		pSharedMemory->JW_MotionSet[10] = FALSE;
		pSharedMemory->JW_MotionSet[1] = TRUE;	
		if(pSharedMemory->CommandFlag == NO_ACT )
		{
			pSharedMemory->CommandFlag = SET_MOCAP;
			Sleep(10);	
		}
		Sleep(8800);

		pSharedMemory->JW_MotionSet[1] = FALSE;
		pSharedMemory->JW_MotionSet[0] = TRUE;	
		if(pSharedMemory->CommandFlag == NO_ACT )
		{
			pSharedMemory->CommandFlag = SET_MOCAP;
			Sleep(10);	
		}
		Sleep(8800);

		
		pSharedMemory->JW_MotionSet[0] = FALSE;
		pSharedMemory->JW_MotionSet[1] = TRUE;	
		if(pSharedMemory->CommandFlag == NO_ACT )
		{
			pSharedMemory->CommandFlag = SET_MOCAP;
			Sleep(10);	
		}
		Sleep(8800);

		
		/*
		pSharedMemory->JW_MotionSet[1] = FALSE;
		pSharedMemory->JW_MotionSet[2] = TRUE;	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);	
		if(flag_temp == TRUE)
			pSharedMemory->CommandFlag = INIT_WB_MOCAP;
		Sleep(9500);

		*/
		pSharedMemory->JW_MotionSet[1] = FALSE;
		pSharedMemory->JW_MotionSet[0] = TRUE;	
		if(pSharedMemory->CommandFlag == NO_ACT )
		{
			pSharedMemory->CommandFlag = SET_MOCAP;
			Sleep(10);	
		}
		Sleep(8800);

		/*
		pSharedMemory->JW_MotionSet[0] = FALSE;
		pSharedMemory->JW_MotionSet[1] = TRUE;	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);	
		if(flag_temp == TRUE)
			pSharedMemory->CommandFlag = INIT_WB_MOCAP;
		Sleep(8800);
		*/
		pSharedMemory->JW_MotionSet[0] = FALSE;
		pSharedMemory->JW_MotionSet[14] = TRUE;	
		if(pSharedMemory->CommandFlag == NO_ACT )
		{
			pSharedMemory->CommandFlag = SET_MOCAP;
			Sleep(10);	
		}
		//AfxMessageBox("sleep in");
		//printf("sleep in");
		Sleep(6000);
		//printf("sleep out");
		//AfxMessageBox("sleep out");
		pSharedMemory->JW_MotionSet[15] = FALSE;	
		pSharedMemory->JW_MotionSet[14] = FALSE;
		pSharedMemory->JW_MotionSet[15] = TRUE;	
		//AfxMessageBox("set 15");
		//printf("set 15");
		if(pSharedMemory->CommandFlag == NO_ACT )
		{
			//Sleep(1500);
			pSharedMemory->CommandFlag = SET_MOCAP;
			Sleep(10);	
		}			
		//else 
		//AfxMessageBox("Other Command is activated..!!");
		
		Sleep(200);

		/*
		pSharedMemory->JW_MotionSet[15] = FALSE;	
		pSharedMemory->JW_MotionSet[14] = FALSE;
		pSharedMemory->JW_MotionSet[15] = TRUE;	
		if(pSharedMemory->CommandFlag == NO_ACT )
		{
			pSharedMemory->CommandFlag = SET_MOCAP;
			Sleep(10);	
		}			
		//else 
		//	AfxMessageBox("Other Command is activated..!!");
		
		Sleep(100);
*/
	
}

void CWalkingDlg::OnButton4() 
{
	// TODO: Add your control notification handler code here
	
//	int i;
	UpdateData(true);
	Sleep(200);
	if(pSharedMemory->CommandFlag == NO_ACT )
	{
		pSharedMemory->JW_temp[33]= m_rlshpgain;
		pSharedMemory->JW_temp[34]= m_rlebgain;
		pSharedMemory->PushUpFlag = 1;
		pSharedMemory->MotorControlMode = CTRLMODE_PUSHUP;
		Sleep(10);
	}			
	else 
	AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnButton5() 
{
	// TODO: Add your control notification handler code here
	
	int i;
	if(pSharedMemory->CommandFlag == NO_ACT )
	{
		for(i=0; i<NoOfMotion; i++) 
			pSharedMemory->JW_MotionSet[i] = FALSE;		
		
		pSharedMemory->JW_MotionSet[15] = TRUE;	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);	
		Sleep(10);
	}			
	else 
	AfxMessageBox("Other Command is activated..!!");
}

void CWalkingDlg::OnDemotest() 
{
	// TODO: Add your control notification handler code here
	if(m_DemoTestDlg.GetSafeHwnd() == NULL)
		m_DemoTestDlg.Create(IDD_DEMOTEST_DIALOG);
	m_DemoTestDlg.ShowWindow(SW_SHOW);
}

void CWalkingDlg::OnDemotest2() 
{
	// TODO: Add your control notification handler code here
	if(m_DemoTest2Dlg.GetSafeHwnd() == NULL)
		m_DemoTest2Dlg.Create(IDD_DEMOTEST2_DIALOG);
	m_DemoTest2Dlg.ShowWindow(SW_SHOW);
}

void CWalkingDlg::OnLog() 
{
	CString str_temp;
	CString str_filename;
	char szFilter[] = "txt Files (*.txt) |*.txt|ALL Files (*.*) |*.*|";
	CFileDialog dlg(FALSE,NULL,NULL,OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,	szFilter );


	GetDlgItem(IDC_LOG)->GetWindowText(str_temp);

	if(str_temp == "Log")
	{
		//if(m_ofstream.is_open())
		//	m_ofstream.close();
		if(m_pFile != NULL)
			fclose(m_pFile);
		
		if(dlg.DoModal()==IDOK)
		{
			str_filename = dlg.GetPathName();
			if(str_filename.Find(".") == -1)
				str_filename += ".txt";				
			//m_ofstream.open(str_filename);
			m_pFile = fopen(str_filename, "w");
		}
		else
		{
			GetDlgItem(IDC_LOG)->SetWindowText("Log");
			return;
		}

		pSharedMemory->LogRing.uHead = 0;
		pSharedMemory->LogRing.uTail = 0;
		pSharedMemory->LogRing.bOverflow = FALSE;
		GetDlgItem(IDC_LOG)->SetWindowText("Stop");
	}
	else
	{
		pSharedMemory->LogRing.bOverflow = TRUE;
		//m_ofstream.close();
		fclose(m_pFile);
		m_pFile = NULL;
		GetDlgItem(IDC_LOG)->SetWindowText("Log");
	}
}


void CWalkingDlg::LogRingHandler()  // by Inhyeok
{
	int i;
	unsigned int tail_next;

/*	if(!m_ofstream.is_open())
		return;

	while(pSharedMemory->LogRing.uTail != pSharedMemory->LogRing.uHead)
	{
		tail_next = (pSharedMemory->LogRing.uTail+1)%LOG_RING_SIZE;

		for(i=0; i<LOG_DATA_SIZE; i++)
			m_ofstream << pSharedMemory->LogRing.data[tail_next][i] << " " ;			
		m_ofstream << endl;

		pSharedMemory->LogRing.uTail = tail_next;
	}
*/
	if(m_pFile == NULL)
		return;

	while(pSharedMemory->LogRing.uTail != pSharedMemory->LogRing.uHead)
	{
		tail_next = (pSharedMemory->LogRing.uTail+1)%LOG_RING_SIZE;

		for(i=0; i<LOG_DATA_SIZE; i++)
			fprintf(m_pFile, "%e ", pSharedMemory->LogRing.data[tail_next][i]);			
		fprintf(m_pFile, "\n");

		pSharedMemory->LogRing.uTail = tail_next;
	}

}

void CWalkingDlg::OnTimer(UINT nIDEvent) 
{
	LogRingHandler(); // by Inhyeok
	
	CPropertyPage::OnTimer(nIDEvent);
}

void CWalkingDlg::OnDrc() 
{
	if(m_DRCtestDlg.GetSafeHwnd() == NULL)
		m_DRCtestDlg.Create(IDD_DRC_TEST_DIALOG);
	m_DRCtestDlg.ShowWindow(SW_SHOW);	
	
}

void CWalkingDlg::OnKirk() 
{
	if(m_DrcQuadDlg.GetSafeHwnd() == NULL)
		m_DrcQuadDlg.Create(IDD_DRC_QUAD_DIALOG);
	m_DrcQuadDlg.ShowWindow(SW_SHOW);
	
}

void CWalkingDlg::OnKirk2() 
{
	if(m_DrcBipedDlg.GetSafeHwnd() == NULL)
		m_DrcBipedDlg.Create(IDD_DRC_BIPED_DIALOG);
	m_DrcBipedDlg.ShowWindow(SW_SHOW);
	
}
