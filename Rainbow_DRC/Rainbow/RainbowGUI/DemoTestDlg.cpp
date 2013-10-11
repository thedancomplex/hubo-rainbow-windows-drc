// DemoTestDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "RainbowGUIDlg.h"
#include "DemoTestDlg.h"
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
// CDemoTestDlg dialog


CDemoTestDlg::CDemoTestDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CDemoTestDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CDemoTestDlg)
	m_trackingmode = 0;
	m_rps_mode = 0;
	m_imu_feedback = FALSE;
	//}}AFX_DATA_INIT
}


void CDemoTestDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CDemoTestDlg)
	DDX_Radio(pDX, IDC_TRUE, m_trackingmode);
	DDX_Radio(pDX, IDC_RADIO_RPS, m_rps_mode);
	DDX_Check(pDX, IDC_IMU_FEEDBACK, m_imu_feedback);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CDemoTestDlg, CDialog)
	//{{AFX_MSG_MAP(CDemoTestDlg)
	ON_BN_CLICKED(IDC_SCENE1, OnScene1)
	ON_BN_CLICKED(IDC_SCENE2, OnScene2)
	ON_BN_CLICKED(IDC_SCENE3, OnScene3)
	ON_BN_CLICKED(IDC_SCENE4, OnScene4)
	ON_BN_CLICKED(IDC_SCENE5, OnScene5)
	ON_BN_CLICKED(IDC_SCENE6, OnScene6)
	ON_BN_CLICKED(IDC_SCENE7, OnScene7)
	ON_BN_CLICKED(IDC_SCENE8, OnScene8)
	ON_BN_CLICKED(IDC_SCENE9, OnScene9)
	ON_BN_CLICKED(IDC_SCENE10, OnScene10)
	ON_BN_CLICKED(IDC_SCENE11, OnScene11)
	ON_BN_CLICKED(IDC_SCENE12, OnScene12)
	ON_BN_CLICKED(IDC_SCENE13, OnScene13)
	ON_BN_CLICKED(IDC_SCENE14, OnScene14)
	ON_BN_CLICKED(IDC_SCENE15, OnScene15)
	ON_BN_CLICKED(IDC_SCENE16, OnScene16)
	ON_BN_CLICKED(IDC_SCENE17, OnScene17)
	ON_BN_CLICKED(IDC_SCENE18, OnScene18)
	ON_BN_CLICKED(IDC_SCENE19, OnScene19)
	ON_BN_CLICKED(IDC_SCENE22, OnScene22)
	ON_BN_CLICKED(IDC_SCENE23, OnScene23)
	ON_BN_CLICKED(IDC_SCENE24, OnScene24)
	ON_BN_CLICKED(IDC_SCENE25, OnScene25)
	ON_BN_CLICKED(IDC_SCENE26, OnScene26)
	ON_BN_CLICKED(IDC_SCENE28, OnScene28)
	ON_BN_CLICKED(IDC_SCENE30, OnScene30)
	ON_BN_CLICKED(IDC_SCENE32, OnScene32)
	ON_BN_CLICKED(IDC_SCENE27_GRASP_POS, OnScene27GraspPos)
	ON_BN_CLICKED(IDC_SCENE27_GRASP_ONOFF, OnScene27GraspOnoff)
	ON_BN_CLICKED(IDC_SCENE27_SHOW, OnScene27Show)
	ON_BN_CLICKED(IDC_SCENE27_HOME, OnScene27Home)
	ON_BN_CLICKED(IDC_WALK1, OnWalk1)
	ON_BN_CLICKED(IDC_SCENE35, OnScene35)
	ON_BN_CLICKED(IDC_SCENE36, OnScene36)
	ON_BN_CLICKED(IDC_SCENE37, OnScene37)
	ON_BN_CLICKED(IDC_SCENE38, OnScene38)
	ON_BN_CLICKED(IDC_SCENE39, OnScene39)
	ON_BN_CLICKED(IDC_SCENE40, OnScene40)
	ON_BN_CLICKED(IDC_SCENE41, OnScene41)
	ON_BN_CLICKED(IDC_WALK2, OnWalk2)
	ON_BN_CLICKED(IDC_WALK3, OnWalk3)
	ON_BN_CLICKED(IDC_WALK4, OnWalk4)
	ON_BN_CLICKED(IDC_WALK5, OnWalk5)
	ON_BN_CLICKED(IDC_WALK6, OnWalk6)
	ON_BN_CLICKED(IDC_SCENE20, OnScene20)
	ON_BN_CLICKED(IDC_SCENE21, OnScene21)
	ON_BN_CLICKED(IDC_SCENE21_ONOFF, OnScene21Onoff)
	ON_BN_CLICKED(IDC_TRACKING1, OnStartRHTracking)
	ON_BN_CLICKED(IDC_HANDSHAKE1, OnHandshake1)
	ON_BN_CLICKED(IDC_HANDSHAKE2, OnHandshake2)
	ON_BN_CLICKED(IDC_GO2SSP, OnGo2SSP)
	ON_BN_CLICKED(IDC_UPDOWN_SSP, OnUpdownSSP)
	ON_BN_CLICKED(IDC_BACK2DSP, OnBack2DSP)
	ON_BN_CLICKED(IDC_WB_DANCE, OnWbDance)
	ON_BN_CLICKED(IDC_HAND_INIT, OnHandInit)
	ON_BN_CLICKED(IDC_WALK7, OnWalk7)
	ON_BN_CLICKED(IDC_BUTTON_ABLE, OnButtonAble)
	ON_WM_DRAWITEM()
	ON_BN_CLICKED(IDC_WALK8, OnWalk8)
	ON_BN_CLICKED(IDC_VOICE, OnVoice)
	ON_BN_CLICKED(IDC_SCENE27, OnScene27)
	ON_BN_CLICKED(IDC_RPS, OnRps)
	ON_BN_CLICKED(IDC_RPS2, OnRps2)
	ON_BN_CLICKED(IDC_RPS3, OnRps3)
	ON_BN_CLICKED(IDC_SCENE33, OnScene33)
	ON_BN_CLICKED(IDC_EM_STOP, OnEmStop)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDemoTestDlg message handlers

void CDemoTestDlg::OnScene1() 
{
	// TODO: Add your control notification handler code here
	/*
	int i;

	for(i=0; i<NoOfMotion; i++)
		pSharedMemory->JW_MotionSet[i] = FALSE;		
		
		
	//AfxMessageBox("in");
	pSharedMemory->JW_MotionSet[38] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	*/
/*	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)		// by Inhyeok
	{
		pSharedMemory->WB_SceneNo = 1;
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_UB_SCIENCE_MUSEUM;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		GetDlgItem(IDC_SCENE1)->EnableWindow(FALSE);
		GetDlgItem(IDC_WALK1)->EnableWindow(TRUE);
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
*/
 }

void CDemoTestDlg::OnScene2() 
{
	// TODO: Add your control notification handler code here
	/*
	int i;

	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	
	
	//AfxMessageBox("in");
	pSharedMemory->JW_MotionSet[39] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);			
		PlaySound("C:\\voice\\Demo\\scene2_Greeting.wav",NULL,SND_ASYNC);
	}
	else
		AfxMessageBox("Other Command is activated..!!");

	*/
/*	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)		// by Inhyeok
	{
		pSharedMemory->WB_SceneNo = 2;
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_UB_SCIENCE_MUSEUM;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		//PlaySound("C:\\voice\\Demo\\scene2_Greeting.wav",NULL,SND_ASYNC);
		GetDlgItem(IDC_SCENE2)->EnableWindow(FALSE);
		GetDlgItem(IDC_SCENE3)->EnableWindow(TRUE);
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
	
*/		
}

void CDemoTestDlg::OnScene3() 
{
	// TODO: Add your control notification handler code here
	/*
	int i;
	
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	
	
	//AfxMessageBox("in");
	pSharedMemory->JW_MotionSet[40] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	*/
/*	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)		// by Inhyeok
	{
		pSharedMemory->WB_SceneNo = 8;
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_UB_SCIENCE_MUSEUM;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		//PlaySound("C:\\voice\\Demo\\Hey_you_there.wav",NULL,SND_ASYNC);
		GetDlgItem(IDC_SCENE3)->EnableWindow(FALSE);
		GetDlgItem(IDC_SCENE4)->EnableWindow(TRUE);
	}	
	else
		AfxMessageBox("Other Command is activated..!!");

*/  
}

void CDemoTestDlg::OnScene4() 
{
	// TODO: Add your control notification handler code here
	/*
	int i;

	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	
	pSharedMemory->JW_MotionSet[41] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
		//PlaySound("C:\\voice\\Demo\\scene4_How_old_r_u.wav",NULL,SND_ASYNC);
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	*/
	///*
/*	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)		// by Inhyeok
	{
		pSharedMemory->WB_SceneNo = 4;
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_UB_SCIENCE_MUSEUM;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		//PlaySound("C:\\voice\\Demo\\scene4_How_old_r_u.wav",NULL,SND_ASYNC);
		GetDlgItem(IDC_SCENE4)->EnableWindow(FALSE);
		GetDlgItem(IDC_SCENE5)->EnableWindow(TRUE);
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
	//	*/
 }

void CDemoTestDlg::OnScene5() 
{
	// TODO: Add your control notification handler code here
	/*int i;
	
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	
	
	//AfxMessageBox("in");
	pSharedMemory->JW_MotionSet[42] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
		PlaySound("C:\\voice\\Demo\\scene5_How_old_m_I.wav",NULL,SND_ASYNC);
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	*/
/*	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)		// by Inhyeok
	{
		pSharedMemory->WB_SceneNo = 5;
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_UB_SCIENCE_MUSEUM;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		PlaySound("C:\\voice\\Demo\\scene5_How_old_m_I.wav",NULL,SND_ASYNC);
		GetDlgItem(IDC_SCENE5)->EnableWindow(FALSE);
		GetDlgItem(IDC_SCENE6)->EnableWindow(TRUE);
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
*/
}

void CDemoTestDlg::OnScene6() 
{
	// TODO: Add your control notification handler code here
	/*int i;
	
	
	
	
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	
	
	//AfxMessageBox("in");
	pSharedMemory->JW_MotionSet[43] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	*/
/*	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)		// by Inhyeok
	{
		pSharedMemory->WB_SceneNo = 6;
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_UB_SCIENCE_MUSEUM;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		GetDlgItem(IDC_SCENE6)->EnableWindow(FALSE);
		GetDlgItem(IDC_SCENE7)->EnableWindow(TRUE);
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
*/
}

void CDemoTestDlg::OnScene7() 
{
	// TODO: Add your control notification handler code here
	/*int i;
	
	
	
	
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	
	
	//AfxMessageBox("in");
	pSharedMemory->JW_MotionSet[44] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	*/
/*	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)		// by Inhyeok
	{
		pSharedMemory->WB_SceneNo = 7;
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_UB_SCIENCE_MUSEUM;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		GetDlgItem(IDC_SCENE7)->EnableWindow(FALSE);
		GetDlgItem(IDC_SCENE8)->EnableWindow(TRUE);
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
*/
}

void CDemoTestDlg::OnScene8() 
{

}

void CDemoTestDlg::OnScene9() 
{

}

void CDemoTestDlg::OnScene10() 
{

}

void CDemoTestDlg::OnScene11() 
{

}

void CDemoTestDlg::OnScene12() 
{
	
}

void CDemoTestDlg::OnScene13() 
{
	
}

void CDemoTestDlg::OnScene14() 
{
	
}

void CDemoTestDlg::OnScene15() 
{
	
}

void CDemoTestDlg::OnScene16() 
{
	
}

void CDemoTestDlg::OnScene17() 
{
	
}

void CDemoTestDlg::OnScene18() 
{

}

void CDemoTestDlg::OnScene19() 
{
	
}

void CDemoTestDlg::OnScene22() 
{
	
}

void CDemoTestDlg::OnScene23() 
{
	
}

void CDemoTestDlg::OnScene24() 
{
	
}

void CDemoTestDlg::OnScene25() 
{
	
}

void CDemoTestDlg::OnScene26() 
{
	
}

void CDemoTestDlg::OnScene28() 
{
	
}


void CDemoTestDlg::OnScene30() 
{
	
}

void CDemoTestDlg::OnScene32() 
{

}

void CDemoTestDlg::OnScene27GraspPos() 
{
	
}

void CDemoTestDlg::OnScene27GraspOnoff() 
{
	// TODO: Add your control notification handler code here
	CString tempString;

	GetDlgItem(IDC_SCENE27_GRASP_ONOFF)->GetWindowText(tempString);

	if(tempString == "Grasp On")
	{
		GetDlgItem(IDC_SCENE27_GRASP_ONOFF)->SetWindowText("Grasp Off");
		pSharedMemory->CommandDataFloat[0] = 30.0f;
		Sleep(10);
		pSharedMemory->CommandFlag = GRIP_ON_L; 
	}
	else
	{
		GetDlgItem(IDC_SCENE27_GRASP_ONOFF)->SetWindowText("Grasp On");
		pSharedMemory->CommandDataFloat[0] = 30.0f;
		Sleep(10);
		pSharedMemory->CommandFlag = GRIP_OFF_L;
	}
}

void CDemoTestDlg::OnScene27Show() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandData = 0x01;
		pSharedMemory->CommandFlag = DEMO_GRASP;
		GetDlgItem(IDC_SCENE27_SHOW)->EnableWindow(FALSE);
		Sleep(100);
		GetDlgItem(IDC_SCENE27_SHOW)->EnableWindow(TRUE);
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CDemoTestDlg::OnScene27Home() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandData = 0x02;
		pSharedMemory->CommandDataFloat[0] = 0.0f;
		Sleep(10);
		//pSharedMemory->JointID = LF1;
		//pSharedMemory->CommandFlag = GOTO_LIMIT_POS;
		//Sleep(10);
		//pSharedMemory->CommandFlag = C_CONTROL_MODE;
		//Sleep(10);
		pSharedMemory->CommandFlag = DEMO_GRASP;
		GetDlgItem(IDC_SCENE27_HOME)->EnableWindow(FALSE);
		GetDlgItem(IDC_SCENE27_SHOW)->EnableWindow(FALSE);
		GetDlgItem(IDC_GO2SSP)->EnableWindow(TRUE);
		Sleep(10);				
	}
	else AfxMessageBox("Other Command is activated..!!");
}



void CDemoTestDlg::OnWalk1() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandData = 0x00;
		pSharedMemory->WalkingDemo = 0x01;
		pSharedMemory->CommandFlag = SET_PREDEF_WALK;
		Sleep(10);				
		GetDlgItem(IDC_WALK1)->EnableWindow(FALSE);
		GetDlgItem(IDC_SCENE2)->EnableWindow(TRUE);
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CDemoTestDlg::OnScene35() 
{

}

void CDemoTestDlg::OnScene36() 
{
	
}

void CDemoTestDlg::OnScene37() 
{

}

void CDemoTestDlg::OnScene38() 
{
	
}

void CDemoTestDlg::OnScene39() 
{
	
}

void CDemoTestDlg::OnScene40() 
{

}

void CDemoTestDlg::OnScene41() 
{
	
}

void CDemoTestDlg::OnWalk2() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandData = 0x01;
		pSharedMemory->WalkingDemo = 0x01;
		pSharedMemory->CommandFlag = SET_PREDEF_WALK;
		GetDlgItem(IDC_WALK2)->EnableWindow(FALSE);
		GetDlgItem(IDC_WALK3)->EnableWindow(TRUE);
		Sleep(10);				
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CDemoTestDlg::OnWalk3() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandData = 0x02;
		pSharedMemory->WalkingDemo = 0x01;
		pSharedMemory->CommandFlag = SET_PREDEF_WALK;
		GetDlgItem(IDC_WALK3)->EnableWindow(FALSE);
		GetDlgItem(IDC_SCENE17)->EnableWindow(TRUE);
		Sleep(10);				
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CDemoTestDlg::OnWalk4() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandData = 0x03;
		pSharedMemory->WalkingDemo = 0x01;
		pSharedMemory->CommandFlag = SET_PREDEF_WALK;
		GetDlgItem(IDC_WALK4)->EnableWindow(FALSE);
		GetDlgItem(IDC_SCENE20)->EnableWindow(TRUE);
		Sleep(10);				
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CDemoTestDlg::OnWalk5() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandData = 0x04;
		pSharedMemory->WalkingDemo = 0x01;
		pSharedMemory->CommandFlag = SET_PREDEF_WALK;
		GetDlgItem(IDC_WALK5)->EnableWindow(FALSE);
		GetDlgItem(IDC_SCENE25)->EnableWindow(TRUE);
		Sleep(10);				
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CDemoTestDlg::OnWalk6() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandData = 0x05;
		pSharedMemory->WalkingDemo = 0x01;
		pSharedMemory->CommandFlag = SET_PREDEF_WALK;
		GetDlgItem(IDC_WALK6)->EnableWindow(FALSE);
		//GetDlgItem(IDC_HANDSHAKE1)->EnableWindow(TRUE);	
		Sleep(10);				
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CDemoTestDlg::OnScene20() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = DEMO_FLAG;
		pSharedMemory->CommandData = DEMO_CONTROL_OFF_POS;
		GetDlgItem(IDC_SCENE20)->EnableWindow(FALSE);
		GetDlgItem(IDC_SCENE21)->EnableWindow(TRUE);
		Sleep(10);				
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CDemoTestDlg::OnScene21() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = DEMO_FLAG;
		pSharedMemory->CommandData = DEMO_CONTROL_ON_POS;
		GetDlgItem(IDC_SCENE21)->EnableWindow(FALSE);
		GetDlgItem(IDC_SCENE21_ONOFF)->EnableWindow(TRUE);
		Sleep(10);				
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CDemoTestDlg::OnScene21Onoff() 
{
	// TODO: Add your control notification handler code here

	CString tempString;
	
	GetDlgItem(IDC_SCENE21_ONOFF)->GetWindowText(tempString);
	
	if(tempString == "Control On")
	{
		GetDlgItem(IDC_SCENE21_ONOFF)->SetWindowText("Control Off");
		pSharedMemory->CommandData = DSP_ON;
		pSharedMemory->CommandFlag = DEMO_FLAG;
		Sleep(10);
	}
	else
	{
		GetDlgItem(IDC_SCENE21_ONOFF)->SetWindowText("Control On");
		pSharedMemory->CommandData = DSP_OFF;
		pSharedMemory->CommandFlag = DEMO_FLAG;
		GetDlgItem(IDC_SCENE23)->EnableWindow(TRUE);
		Sleep(10);
	}
}

void CDemoTestDlg::OnStartRHTracking() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	UpdateData(TRUE);
	if(pSharedMemory->COM_Ready_Flag==TRUE)
	{
		if(pSharedMemory->RBT_Ready_Flag==TRUE)
		{
			GetDlgItem(IDC_TRACKING1)->GetWindowText(strText);
			if(strText == "Start Tracking")
			{
				if(pSharedMemory->CommandFlag == NO_ACT)
				{
					pSharedMemory->RBT_mode_Flag = m_trackingmode;
					pSharedMemory->CommandFlag = RBT_ON_MODE;
					pSharedMemory->RBT_Start_Flag = TRUE;
					
					GetDlgItem(IDC_TRACKING1)->SetWindowText("Stop Tracking");
				}
				else	AfxMessageBox("Other Command is activated..!!");
			}
			else
			{

				pSharedMemory->CommandFlag = RBT_OFF_MODE;
				pSharedMemory->RBT_Start_Flag = FALSE;
				
				GetDlgItem(IDC_TRACKING1)->SetWindowText("Start Tracking");
			}
		}
		else AfxMessageBox("Invalid Right Hand Position..!!");
	}
	else AfxMessageBox("Please connect 'COM'..!!");
	
}

void CDemoTestDlg::OnHandshake1()
{
	// TODO: Add your control notification handler code here

	GetDlgItem(IDC_HANDSHAKE1)->EnableWindow(false);

	CString strText;
	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_HANDSHAKE1)->GetWindowText(strText);

		if(strText == "Go H/S Pos")
		{
			pSharedMemory->MotorControlMode = CTRLMODE_HANDSHAKE;

			GetDlgItem(IDC_HANDSHAKE1)->SetWindowText("Back to Init. Pos");
			// goto handshake pos
			pSharedMemory->ShakeHandsFlag = 1;
			pSharedMemory->CommandFlag = C_CONTROL_MODE;

			// for CCTM function init
			Sleep(2500);
			pSharedMemory->CommandFlag = CCTM_ON;
			Sleep(10);
			pSharedMemory->CommandFlag = PWMCMD_CDI_MODE_ON;

			// Hand, Grasp Off
			Sleep(10);
			pSharedMemory->CommandDataFloat[0] = 30.0f;
			Sleep(50);
			pSharedMemory->CommandFlag = GRIP_OFF_R;
			Sleep(1000);
			pSharedMemory->CommandFlag = GRIP_STOP_R;
			Sleep(10);
			pSharedMemory->CommandFlag = PWMCMD_FORCE_FB_ON;
			GetDlgItem(IDC_HANDSHAKE1)->EnableWindow(true);
		}
		else
		{
			pSharedMemory->CommandFlag = PWMCMD_FORCE_FB_OFF;
			Sleep(10);
			pSharedMemory->CommandFlag = PWMCMD_CDI_MODE_OFF;
			Sleep(50);
			pSharedMemory->CommandFlag = CCTM_OFF;	
			Sleep(50);

			pSharedMemory->MotorControlMode = CTRLMODE_HANDSHAKE;
			pSharedMemory->ShakeHandsFlag = 5;
			
			pSharedMemory->CommandFlag = GRIP_STOP_R;
			Sleep(10);
			pSharedMemory->JointID = RF1;
			pSharedMemory->CommandFlag = P_CONTROL_MODE;
	
			Sleep(2000);
			GetDlgItem(IDC_HANDSHAKE1)->SetWindowText("Go H/S Pos");
			GetDlgItem(IDC_HANDSHAKE1)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE32)->EnableWindow(TRUE);
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
	
}

void CDemoTestDlg::OnHandshake2() 
{
	// TODO: Add your control notification handler code here

	CString tempString;
	
	GetDlgItem(IDC_HANDSHAKE2)->GetWindowText(tempString);
	
	if(tempString == "Grasp On")
	{
		GetDlgItem(IDC_HANDSHAKE2)->SetWindowText("Grasp Off");
		pSharedMemory->CommandDataFloat[0] = 30.0f;
		Sleep(10);
		pSharedMemory->CommandFlag = GRIP_ON_R;
	}
	else
	{
		GetDlgItem(IDC_HANDSHAKE2)->SetWindowText("Grasp On");
		pSharedMemory->CommandDataFloat[0] = 30.0f;
		Sleep(10);
		pSharedMemory->CommandFlag = GRIP_OFF_R;
	}
}

void CDemoTestDlg::OnGo2SSP() 
{
/*	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)
	{
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_SSP_UPDOWN;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		GetDlgItem(IDC_GO2SSP)->EnableWindow(FALSE);
		GetDlgItem(IDC_UPDOWN_SSP)->EnableWindow(TRUE);
		GetDlgItem(IDC_BACK2DSP)->EnableWindow(TRUE);
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
*/
}

void CDemoTestDlg::OnUpdownSSP() 
{
	pSharedMemory->WB_UpdownFlag = 1;	
}

void CDemoTestDlg::OnBack2DSP() 
{
	pSharedMemory->WB_Down2DSP_flag = 1;
	GetDlgItem(IDC_BACK2DSP)->EnableWindow(FALSE);
	GetDlgItem(IDC_UPDOWN_SSP)->EnableWindow(FALSE);
	GetDlgItem(IDC_GO2SSP)->EnableWindow(TRUE);
	GetDlgItem(IDC_SCENE30)->EnableWindow(TRUE);
}

void CDemoTestDlg::OnWbDance() 
{
	

}

void CDemoTestDlg::OnHandInit() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->JointID = RF1;
		pSharedMemory->CommandFlag = GOTO_LIMIT_POS;
		Sleep(20);
		pSharedMemory->JointID = LF1;
		pSharedMemory->CommandFlag = GOTO_LIMIT_POS;
		Sleep(20);
		pSharedMemory->CommandFlag = C_CONTROL_MODE;
	}
	else AfxMessageBox("Other Command is activated..!!");
	
}

void CDemoTestDlg::OnWalk7() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandData = 0x06;
		pSharedMemory->WalkingDemo = 0x01;
		pSharedMemory->CommandFlag = SET_PREDEF_WALK;
		GetDlgItem(IDC_WALK7)->EnableWindow(FALSE);
		GetDlgItem(IDC_TRACKING1)->EnableWindow(TRUE);
		GetDlgItem(IDC_SCENE35)->EnableWindow(TRUE);
		Sleep(10);				
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CDemoTestDlg::OnButtonAble() 
{
	CString strText;
	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_BUTTON_ABLE)->GetWindowText(strText);
		
		if(strText == "Button Able")
		{
			GetDlgItem(IDC_BUTTON_ABLE)->SetWindowText("Button Disable");

			GetDlgItem(IDC_SCENE1)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE2)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE3)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE4)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE5)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE6)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE7)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE8)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE9)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE10)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE11)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE12)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE13)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE14)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE15)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE16)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE17)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE18)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE19)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE20)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE21)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE22)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE23)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE24)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE25)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE26)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE28)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE30)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE32)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE33)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE35)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE36)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE37)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE38)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE39)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE40)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE41)->EnableWindow(TRUE);
			GetDlgItem(IDC_WALK1)->EnableWindow(TRUE);
			GetDlgItem(IDC_WALK2)->EnableWindow(TRUE);
			GetDlgItem(IDC_WALK3)->EnableWindow(TRUE);
			GetDlgItem(IDC_WALK4)->EnableWindow(TRUE);
			GetDlgItem(IDC_WALK5)->EnableWindow(TRUE);
			GetDlgItem(IDC_WALK6)->EnableWindow(TRUE);
			GetDlgItem(IDC_WALK7)->EnableWindow(TRUE);
			
			GetDlgItem(IDC_SCENE21_ONOFF)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE27_GRASP_POS)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE27_GRASP_ONOFF)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE27_SHOW)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE27_HOME)->EnableWindow(TRUE);
			
			GetDlgItem(IDC_GO2SSP)->EnableWindow(TRUE);
			//GetDlgItem(IDC_HANDSHAKE1)->EnableWindow(TRUE);
			//GetDlgItem(IDC_HANDSHAKE2)->EnableWindow(TRUE);
			GetDlgItem(IDC_TRACKING1)->EnableWindow(TRUE);

			GetDlgItem(IDC_SCENE27)->EnableWindow(TRUE);
			GetDlgItem(IDC_VOICE)->EnableWindow(TRUE);
			GetDlgItem(IDC_WB_DANCE)->EnableWindow(TRUE);

			GetDlgItem(IDC_RPS)->EnableWindow(TRUE);
			
			
		}
		else
		{
			GetDlgItem(IDC_BUTTON_ABLE)->SetWindowText("Button Able");

			GetDlgItem(IDC_SCENE1)->EnableWindow(TRUE);
			GetDlgItem(IDC_SCENE2)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE3)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE4)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE5)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE6)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE7)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE8)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE9)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE10)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE11)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE12)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE13)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE14)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE15)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE16)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE17)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE18)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE19)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE20)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE21)->EnableWindow(FALSE);
			//GetDlgItem(IDC_SCENE22)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE23)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE24)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE25)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE26)->EnableWindow(FALSE);
			//GetDlgItem(IDC_SCENE28)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE30)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE32)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE33)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE35)->EnableWindow(FALSE);
			//GetDlgItem(IDC_SCENE36)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE37)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE38)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE39)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE40)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE41)->EnableWindow(FALSE);
			GetDlgItem(IDC_WALK1)->EnableWindow(FALSE);
			GetDlgItem(IDC_WALK2)->EnableWindow(FALSE);
			GetDlgItem(IDC_WALK3)->EnableWindow(FALSE);
			GetDlgItem(IDC_WALK4)->EnableWindow(FALSE);
			GetDlgItem(IDC_WALK5)->EnableWindow(FALSE);
			GetDlgItem(IDC_WALK6)->EnableWindow(FALSE);
			GetDlgItem(IDC_WALK7)->EnableWindow(FALSE);
			
			GetDlgItem(IDC_SCENE21_ONOFF)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE27_GRASP_POS)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE27_GRASP_ONOFF)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE27_SHOW)->EnableWindow(FALSE);
			GetDlgItem(IDC_SCENE27_HOME)->EnableWindow(FALSE);
			
			GetDlgItem(IDC_GO2SSP)->EnableWindow(FALSE);
			//GetDlgItem(IDC_HANDSHAKE1)->EnableWindow(FALSE);
			
			GetDlgItem(IDC_UPDOWN_SSP)->EnableWindow(FALSE);
			GetDlgItem(IDC_BACK2DSP)->EnableWindow(FALSE);
			//GetDlgItem(IDC_HANDSHAKE2)->EnableWindow(FALSE);
			GetDlgItem(IDC_TRACKING1)->EnableWindow(FALSE);

			GetDlgItem(IDC_SCENE27)->EnableWindow(FALSE);
			GetDlgItem(IDC_VOICE)->EnableWindow(FALSE);
			GetDlgItem(IDC_WB_DANCE)->EnableWindow(FALSE);

			GetDlgItem(IDC_RPS)->EnableWindow(FALSE);
			
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
	
		
}

BOOL CDemoTestDlg::OnInitDialog() 
{
	CDialog::OnInitDialog();
	
	
	GetDlgItem(IDC_SCENE1)->EnableWindow(TRUE);
	GetDlgItem(IDC_SCENE2)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE3)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE4)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE5)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE6)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE7)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE8)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE9)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE10)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE11)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE12)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE13)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE14)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE15)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE16)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE17)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE18)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE19)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE20)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE21)->EnableWindow(FALSE);
	//GetDlgItem(IDC_SCENE22)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE23)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE24)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE25)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE26)->EnableWindow(FALSE);
	//GetDlgItem(IDC_SCENE28)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE30)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE32)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE33)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE35)->EnableWindow(FALSE);
	//GetDlgItem(IDC_SCENE36)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE37)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE38)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE39)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE40)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE41)->EnableWindow(FALSE);
	GetDlgItem(IDC_WALK1)->EnableWindow(FALSE);
	GetDlgItem(IDC_WALK2)->EnableWindow(FALSE);
	GetDlgItem(IDC_WALK3)->EnableWindow(FALSE);
	GetDlgItem(IDC_WALK4)->EnableWindow(FALSE);
	GetDlgItem(IDC_WALK5)->EnableWindow(FALSE);
	GetDlgItem(IDC_WALK6)->EnableWindow(FALSE);
	GetDlgItem(IDC_WALK7)->EnableWindow(FALSE);
	
	GetDlgItem(IDC_SCENE21_ONOFF)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE27_GRASP_POS)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE27_GRASP_ONOFF)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE27_SHOW)->EnableWindow(FALSE);
	GetDlgItem(IDC_SCENE27_HOME)->EnableWindow(FALSE);
	
	GetDlgItem(IDC_GO2SSP)->EnableWindow(FALSE);
	GetDlgItem(IDC_HANDSHAKE1)->EnableWindow(FALSE);
	
	GetDlgItem(IDC_UPDOWN_SSP)->EnableWindow(FALSE);
	GetDlgItem(IDC_BACK2DSP)->EnableWindow(FALSE);
	GetDlgItem(IDC_HANDSHAKE2)->EnableWindow(FALSE);
	GetDlgItem(IDC_TRACKING1)->EnableWindow(FALSE);
	//GetDlgItem(IDC_WB_DANCE)->EnableWindow(FALSE);

	GetDlgItem(IDC_RPS)->EnableWindow(FALSE);
	GetDlgItem(IDC_RPS2)->EnableWindow(FALSE);
	GetDlgItem(IDC_RPS3)->EnableWindow(FALSE);

	
	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}

void CDemoTestDlg::OnDrawItem(int nIDCtl, LPDRAWITEMSTRUCT lpDrawItemStruct) 
{
	// TODO: Add your message handler code here and/or call default
	if((nIDCtl == IDC_WALK1)||(nIDCtl == IDC_WALK2)||(nIDCtl == IDC_WALK3)||(nIDCtl == IDC_WALK4)||(nIDCtl == IDC_WALK5)||(nIDCtl == IDC_WALK6)||(nIDCtl == IDC_WALK7))
	{
		
		CDC dc;
		RECT rect;
		dc.Attach(lpDrawItemStruct->hDC);
		rect = lpDrawItemStruct->rcItem;
		dc.Draw3dRect(&rect, RGB(255,255,255), RGB(0,0,0));
		dc.FillSolidRect(&rect, RGB(255,0,0));

		UINT state = lpDrawItemStruct->itemState;
		if((state & ODS_SELECTED))
		{
			dc.DrawEdge(&rect,EDGE_SUNKEN,BF_RECT);
		}
		else
		{
			dc.DrawEdge(&rect,EDGE_RAISED,BF_RECT);
		}
		dc.SetBkColor(RGB(255,0,0));
		dc.SetTextColor(RGB(255,255,255));

		TCHAR buffer[MAX_PATH];
		ZeroMemory(buffer,MAX_PATH);
		if(nIDCtl == IDC_WALK1)
		{
			char *tempChar = "Walk1\n(Go to center)";
			int tempLength = strlen(tempChar);
			memcpy(buffer, tempChar, tempLength+1);
			rect.top = rect.top+9;
		}
		else if(nIDCtl == IDC_WALK2)
		{
			char *tempChar = "Walk2\n(Back)";
			int tempLength = strlen(tempChar);
			memcpy(buffer, tempChar, tempLength+1);
			rect.top = rect.top+9;
		}
		else if(nIDCtl == IDC_WALK3)
		{
			char *tempChar = "Walk3\n(Go to center)";
			int tempLength = strlen(tempChar);
			memcpy(buffer, tempChar, tempLength+1);
			rect.top = rect.top+9;
		}
		else if(nIDCtl == IDC_WALK4)
		{
			char *tempChar = "Walk4\n(Back)";
			int tempLength = strlen(tempChar);
			memcpy(buffer, tempChar, tempLength+1);
			rect.top = rect.top+9;
		}
		else if(nIDCtl == IDC_WALK5)
		{
			char *tempChar = "Walk5";
			int tempLength = strlen(tempChar);
			memcpy(buffer, tempChar, tempLength+1);
			rect.top = rect.top+18;
		}
		else if(nIDCtl == IDC_WALK6)
		{
			char *tempChar = "Walk6";
			int tempLength = strlen(tempChar);
			memcpy(buffer, tempChar, tempLength+1);
			rect.top = rect.top+18;
		}
		else if(nIDCtl == IDC_WALK7)
		{
			char *tempChar = "Walk7";
			int tempLength = strlen(tempChar);
			memcpy(buffer, tempChar, tempLength+1);
			rect.top = rect.top+18;
		}

		//	::GetWindowText(lpDrawItemStruct->hwndItem, buffer, MAX_PATH);
		
		dc.DrawText(buffer, &rect, DT_CENTER|DT_VCENTER);
		dc.Detach();
	}

	CDialog::OnDrawItem(nIDCtl, lpDrawItemStruct);
}

void CDemoTestDlg::OnWalk8() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = STOP_WALKING;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CDemoTestDlg::OnVoice() 
{
	// TODO: Add your control notification handler code here
	//if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)		// by Inhyeok
	{
		//PlaySound("C:\\voice\\Demo\\Energizer.wav",NULL,SND_ASYNC);
		//GetDlgItem(IDC_VOICE)->EnableWindow(FALSE);
		//GetDlgItem(IDC_WALK6)->EnableWindow(TRUE);
	}
	//else
	//	AfxMessageBox("Other Command is activated..!!");
}

void CDemoTestDlg::OnScene27() 
{

}

void CDemoTestDlg::OnRps() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->RPS_POS_MODE = 0x00;
		pSharedMemory->CommandFlag = DEMO_RPS;
		GetDlgItem(IDC_RPS)->EnableWindow(FALSE);
		GetDlgItem(IDC_RPS2)->EnableWindow(TRUE);
		GetDlgItem(IDC_RPS3)->EnableWindow(FALSE);
		Sleep(10);
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CDemoTestDlg::OnRps2() 
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->RPS_mode = m_rps_mode;
		pSharedMemory->RPS_POS_MODE = 0x01;
		pSharedMemory->CommandFlag = DEMO_RPS;
		GetDlgItem(IDC_RPS)->EnableWindow(FALSE);
		GetDlgItem(IDC_RPS2)->EnableWindow(FALSE);
		GetDlgItem(IDC_RPS3)->EnableWindow(TRUE);
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CDemoTestDlg::OnRps3() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->RPS_POS_MODE = 0x02;
		pSharedMemory->CommandDataFloat[0] = 0.0f;
		Sleep(10);
		pSharedMemory->CommandFlag = DEMO_RPS;
		GetDlgItem(IDC_RPS)->EnableWindow(TRUE);
		GetDlgItem(IDC_RPS2)->EnableWindow(FALSE);
		GetDlgItem(IDC_RPS3)->EnableWindow(FALSE);
		Sleep(10);				
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CDemoTestDlg::OnScene33() 
{

}

void CDemoTestDlg::OnEmStop() 
{
	if(pSharedMemory->CommandFlag == NO_ACT)
		pSharedMemory->CommandFlag = EMERGENCY_STOP;	
}
