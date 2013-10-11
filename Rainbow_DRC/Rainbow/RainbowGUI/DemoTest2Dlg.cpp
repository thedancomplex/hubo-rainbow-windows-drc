// DemoTest2Dlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "DemoTest2Dlg.h"
#include "..\SharedMemory.h"
#include "..\CommonDefinition.h"
#include <mmsystem.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

// SharedMemory variable
extern PSHARED_DATA pSharedMemory;

/////////////////////////////////////////////////////////////////////////////
// CDemoTest2Dlg dialog


CDemoTest2Dlg::CDemoTest2Dlg(CWnd* pParent /*=NULL*/)
	: CDialog(CDemoTest2Dlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CDemoTest2Dlg)
	m_rps_mode = 0;
	m_language = 0;
	//}}AFX_DATA_INIT
}


void CDemoTest2Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CDemoTest2Dlg)
	DDX_Radio(pDX, IDC_RADIO1, m_rps_mode);
	DDX_Radio(pDX, IDC_RADIO2, m_language);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CDemoTest2Dlg, CDialog)
	//{{AFX_MSG_MAP(CDemoTest2Dlg)
	ON_BN_CLICKED(IDC_MOTION1, OnMotion1)
	ON_BN_CLICKED(IDC_MOTION4, OnMotion4)
	ON_BN_CLICKED(IDC_MOTION5, OnMotion5)
	ON_BN_CLICKED(IDC_MOTION6, OnMotion6)
	ON_BN_CLICKED(IDC_MOTION6_A, OnMotion6A)
	ON_BN_CLICKED(IDC_MOTION6_B, OnMotion6B)
	ON_BN_CLICKED(IDC_MOTION7, OnMotion7)
	ON_BN_CLICKED(IDC_MOTION8, OnMotion8)
	ON_BN_CLICKED(IDC_MOTION9, OnMotion9)
	ON_BN_CLICKED(IDC_MOTION13, OnMotion13)
	ON_BN_CLICKED(IDC_MOTION14, OnMotion14)
	ON_BN_CLICKED(IDC_MOTION15, OnMotion15)
	ON_BN_CLICKED(IDC_MOTION16, OnMotion16)
	ON_BN_CLICKED(IDC_MP, OnMp)
	ON_BN_CLICKED(IDC_MP2, OnMp2)
	ON_BN_CLICKED(IDC_MP3, OnMp3)
	ON_BN_CLICKED(IDC_MP7, OnMp7)
	ON_BN_CLICKED(IDC_MP4, OnMp4)
	ON_BN_CLICKED(IDC_MP5, OnMp5)
	ON_BN_CLICKED(IDC_MP6, OnMp6)
	ON_BN_CLICKED(IDC_MP8, OnMp8)
	ON_BN_CLICKED(IDC_MP12, OnMp12)
	ON_BN_CLICKED(IDC_MP13, OnMp13)
	ON_BN_CLICKED(IDC_MP14, OnMp14)
	ON_BN_CLICKED(IDC_MP15, OnMp15)
	ON_BN_CLICKED(IDC_MP16, OnMp16)
	ON_BN_CLICKED(IDC_MP17, OnMp17)
	ON_BN_CLICKED(IDC_MP18, OnMp18)
	ON_BN_CLICKED(IDC_MP19, OnMp19)
	ON_BN_CLICKED(IDC_MP20, OnMp20)
	ON_BN_CLICKED(IDC_MOTION19, OnMotion19)
	ON_BN_CLICKED(IDC_MOTION20, OnMotion20)
	ON_BN_CLICKED(IDC_CONTROL, OnControl)
	ON_BN_CLICKED(IDC_MOTION21, OnMotion21)
	ON_BN_CLICKED(IDC_BUTTON3, OnButton3)
	ON_BN_CLICKED(IDC_VOICE1, OnVoice1)
	ON_BN_CLICKED(IDC_VOICE2, OnVoice2)
	ON_BN_CLICKED(IDC_VOICE3, OnVoice3)
	ON_BN_CLICKED(IDC_VOICE4, OnVoice4)
	ON_BN_CLICKED(IDC_VOICE5, OnVoice5)
	ON_BN_CLICKED(IDC_VOICE6, OnVoice6)
	ON_BN_CLICKED(IDC_VOICE7, OnVoice7)
	ON_BN_CLICKED(IDC_VOICE8, OnVoice8)
	ON_BN_CLICKED(IDC_VOICE9, OnVoice9)
	ON_BN_CLICKED(IDC_VOICE10, OnVoice10)
	ON_BN_CLICKED(IDC_VOICE11, OnVoice11)
	ON_BN_CLICKED(IDC_VOICE12, OnVoice12)
	ON_BN_CLICKED(IDC_VOICE13, OnVoice13)
	ON_BN_CLICKED(IDC_VOICE14, OnVoice14)
	ON_BN_CLICKED(IDC_VOICE15, OnVoice15)
	ON_BN_CLICKED(IDC_VOICE16, OnVoice16)
	ON_BN_CLICKED(IDC_VOICE17, OnVoice17)
	ON_BN_CLICKED(IDC_VOICE18, OnVoice18)
	ON_BN_CLICKED(IDC_RPS, OnRps)
	ON_BN_CLICKED(IDC_RPS2, OnRps2)
	ON_BN_CLICKED(IDC_RPS3, OnRps3)
	ON_BN_CLICKED(IDC_CONTROL2, OnControl2)
	ON_BN_CLICKED(IDC_MOTION17, OnMotion17)
	ON_BN_CLICKED(IDC_MOTION22, OnMotion22)
	ON_BN_CLICKED(IDC_MOTION23, OnMotion23)
	ON_BN_CLICKED(IDC_MP21, OnMp21)
	ON_BN_CLICKED(IDC_VOICE19, OnVoice19)
	ON_BN_CLICKED(IDC_VOICE20, OnVoice20)
	ON_BN_CLICKED(IDC_VOICE21, OnVoice21)
	ON_BN_CLICKED(IDC_VOICE22, OnVoice22)
	ON_BN_CLICKED(IDC_VOICE23, OnVoice23)
	ON_BN_CLICKED(IDC_VOICE24, OnVoice24)
	ON_BN_CLICKED(IDC_VOICE25, OnVoice25)
	ON_BN_CLICKED(IDC_VOICE26, OnVoice26)
	ON_BN_CLICKED(IDC_VOICE27, OnVoice27)
	ON_BN_CLICKED(IDC_VOICE28, OnVoice28)
	ON_BN_CLICKED(IDC_VOICE29, OnVoice29)
	ON_BN_CLICKED(IDC_VOICE30, OnVoice30)
	ON_BN_CLICKED(IDC_MOTION24, OnMotion24)
	ON_BN_CLICKED(IDC_MOTION25, OnMotion25)
	ON_BN_CLICKED(IDC_MOTION26, OnMotion26)
	ON_BN_CLICKED(IDC_MOTION27, OnMotion27)
	ON_BN_CLICKED(IDC_MOTION28, OnMotion28)
	ON_BN_CLICKED(IDC_MOTION29, OnMotion29)
	ON_BN_CLICKED(IDC_VOICE31, OnVoice31)
	ON_BN_CLICKED(IDC_MOTION30, OnMotion30)
	ON_BN_CLICKED(IDC_MOTION31, OnMotion31)
	ON_BN_CLICKED(IDC_MOTION32, OnMotion32)
	ON_BN_CLICKED(IDC_MOTION33, OnMotion33)
	ON_BN_CLICKED(IDC_MOTION34, OnMotion34)
	ON_BN_CLICKED(IDC_MOTION35, OnMotion35)
	ON_BN_CLICKED(IDC_MOTION36, OnMotion36)
	ON_BN_CLICKED(IDC_MOTION37, OnMotion37)
	ON_BN_CLICKED(IDC_MOTION38, OnMotion38)
	ON_BN_CLICKED(IDC_VOICE32, OnVoice32)
	ON_BN_CLICKED(IDC_VOICE33, OnVoice33)
	ON_BN_CLICKED(IDC_VOICE34, OnVoice34)
	ON_BN_CLICKED(IDC_VOICE35, OnVoice35)
	ON_BN_CLICKED(IDC_VOICE36, OnVoice36)
	ON_BN_CLICKED(IDC_VOICE37, OnVoice37)
	ON_BN_CLICKED(IDC_VOICE38, OnVoice38)
	ON_BN_CLICKED(IDC_MP9, OnMp9)
	ON_BN_CLICKED(IDC_MP10, OnMp10)
	ON_BN_CLICKED(IDC_MP11, OnMp11)
	ON_BN_CLICKED(IDC_VOICE39, OnVoice39)
	ON_BN_CLICKED(IDC_RADIO2, OnRadio2)
	ON_BN_CLICKED(IDC_RADIO8, OnRadio8)
	ON_BN_CLICKED(IDC_VOICE40, OnVoice40)
	ON_BN_CLICKED(IDC_VOICE41, OnVoice41)
	ON_BN_CLICKED(IDC_VOICE42, OnVoice42)
	ON_BN_CLICKED(IDC_MOTION39, OnMotion39)
	ON_BN_CLICKED(IDC_VOICE43, OnVoice43)
	ON_BN_CLICKED(IDC_MOTION40, OnMotion40)
	ON_BN_CLICKED(IDC_VOICE44, OnVoice44)
	ON_BN_CLICKED(IDC_VOICE45, OnVoice45)
	ON_BN_CLICKED(IDC_VOICE46, OnVoice46)
	ON_BN_CLICKED(IDC_VOICE47, OnVoice47)
	ON_BN_CLICKED(IDC_MOTION18, OnMotion18)
	ON_BN_CLICKED(IDC_MOTION41, OnMotion41)
	ON_BN_CLICKED(IDC_MOTION42, OnMotion42)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDemoTest2Dlg message handlers

void CDemoTest2Dlg::OnMotion1() //Speech1
{
/*	GetDlgItem(IDC_MOTION1)->EnableWindow(FALSE);
	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)		// by Inhyeok
	{
		PlaySound("C:\\voice\\k_tech\\born_k.wav",NULL,SND_ASYNC);
		pSharedMemory->WB_SceneNo = 12;
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_UB_SCIENCE_MUSEUM;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
*/
}

void CDemoTest2Dlg::OnMotion4() 
{
	// TODO: Add your control notification handler code here
	/*int i;
	
	
	
	
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	
	
	//AfxMessageBox("in");
	pSharedMemory->JW_MotionSet[51] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	*/
/*	GetDlgItem(IDC_MOTION4)->EnableWindow(FALSE);
	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)		// by Inhyeok
	{
		PlaySound("C:\\voice\\k_tech\\s7.wav",NULL,SND_ASYNC);
		Sleep(2000);
		pSharedMemory->WB_SceneNo = 15;
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_UB_SCIENCE_MUSEUM;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
*/
}

void CDemoTest2Dlg::OnMotion5() 
{
	// TODO: Add your control notification handler code here
	/*int i;
		
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	
	
	//AfxMessageBox("in");
	pSharedMemory->JW_MotionSet[58] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
		PlaySound("C:\\voice\\Demo\\scene24_of_course.wav",NULL,SND_ASYNC);
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	*/
/*	GetDlgItem(IDC_MOTION5)->EnableWindow(FALSE);
	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)		// by Inhyeok
	{
		pSharedMemory->WB_SceneNo = 24;
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_UB_SCIENCE_MUSEUM;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		PlaySound("C:\\voice\\k_tech\\s9.wav",NULL,SND_ASYNC);
		
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
*/
}

void CDemoTest2Dlg::OnMotion6() 
{
	// TODO: Add your control notification handler code here
/*	GetDlgItem(IDC_MOTION6)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION6_A)->EnableWindow(TRUE);
	GetDlgItem(IDC_MOTION6_B)->EnableWindow(TRUE);
	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)
	{
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_SSP_UPDOWN;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
*/
 }

void CDemoTest2Dlg::OnMotion6A() 
{
	// TODO: Add your control notification handler code here
	pSharedMemory->WB_UpdownFlag = 1;
}

void CDemoTest2Dlg::OnMotion6B() 
{
	// TODO: Add your control notification handler code here
	
	pSharedMemory->WB_Down2DSP_flag = 1;
	GetDlgItem(IDC_MOTION6_B)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION6_A)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION6)->EnableWindow(TRUE);
}

void CDemoTest2Dlg::OnMotion7() 
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
/*	GetDlgItem(IDC_MOTION7)->EnableWindow(FALSE);
	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)		// by Inhyeok
	{
		pSharedMemory->WB_SceneNo = 5;
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_UB_SCIENCE_MUSEUM;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		PlaySound("C:\\voice\\k_tech\\s11.wav",NULL,SND_ASYNC);
		
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
*/
}

void CDemoTest2Dlg::OnMotion8() 
{
/*	GetDlgItem(IDC_MOTION8)->EnableWindow(FALSE);
	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)		// by Inhyeok
	{
		PlaySound("C:\\voice\\k_tech\\s13.wav",NULL,SND_ASYNC);
		pSharedMemory->WB_SceneNo = 54;
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_UB_SCIENCE_MUSEUM;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
*/	
}

void CDemoTest2Dlg::OnMotion9() 
{
/*	GetDlgItem(IDC_MOTION9)->EnableWindow(FALSE);
	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)		// by Inhyeok
	{
		PlaySound("C:\\voice\\k_tech\\s15.wav",NULL,SND_ASYNC);
		pSharedMemory->WB_SceneNo = 55;
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_UB_SCIENCE_MUSEUM;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
*/
}


void CDemoTest2Dlg::OnMotion13() 
{
	// TODO: Add your control notification handler code here
	/*

	int i;
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	GetDlgItem(IDC_MOTION13)->EnableWindow(FALSE);
	pSharedMemory->JW_MotionSet[65] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		//PlaySound("C:\\voice\\k_tech\\11.wav",NULL,SND_ASYNC);
		Sleep(700);
		pSharedMemory->CommandFlag = SET_MOCAP;
	
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	//*/
	/*
	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)		// by Inhyeok
	{
		pSharedMemory->WB_SceneNo = 8;
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_UB_SCIENCE_MUSEUM;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
	*/

	
/*	if(pSharedMemory->CommandFlag == NO_ACT && pSharedMemory->WB_DemoRunFlag == 0)		// by Inhyeok
	{
		GetDlgItem(IDC_MOTION13)->EnableWindow(FALSE);
		pSharedMemory->WB_SceneNo = 62;
		pSharedMemory->WB_DemoNo = WB_OFFLINE_DEMO_UB_SCIENCE_MUSEUM;
		pSharedMemory->CommandFlag = INIT_WB_OFFLINE_DEMO;
		
	}	
	else
		AfxMessageBox("Other Command is activated..!!");
*/	
}

void CDemoTest2Dlg::OnMotion14() 
{

}



void CDemoTest2Dlg::OnMp() 
{
	// TODO: Add your control notification handler code here
	int i;
		
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	GetDlgItem(IDC_MP)->EnableWindow(FALSE);
	pSharedMemory->JW_MotionSet[11] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDemoTest2Dlg::OnMp2() 
{
	// TODO: Add your control notification handler code here
	int i;
		
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	GetDlgItem(IDC_MP2)->EnableWindow(FALSE);	
	pSharedMemory->JW_MotionSet[12] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDemoTest2Dlg::OnMp3() 
{
	// TODO: Add your control notification handler code here
	int i;
		
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	GetDlgItem(IDC_MP3)->EnableWindow(FALSE);	
	pSharedMemory->JW_MotionSet[13] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDemoTest2Dlg::OnMp7() 
{
	// TODO: Add your control notification handler code here
	int i;
		
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	GetDlgItem(IDC_MP7)->EnableWindow(FALSE);	
	pSharedMemory->JW_MotionSet[14] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDemoTest2Dlg::OnMp4() 
{
	// TODO: Add your control notification handler code here
	int i;
		
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	GetDlgItem(IDC_MP4)->EnableWindow(FALSE);	
	pSharedMemory->JW_MotionSet[16] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDemoTest2Dlg::OnMp5() 
{
	// TODO: Add your control notification handler code here
	int i;
		
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	GetDlgItem(IDC_MP5)->EnableWindow(FALSE);	
	pSharedMemory->JW_MotionSet[17] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDemoTest2Dlg::OnMp6() 
{
	// TODO: Add your control notification handler code here
	int i;
		
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	GetDlgItem(IDC_MP6)->EnableWindow(FALSE);	
	pSharedMemory->JW_MotionSet[18] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDemoTest2Dlg::OnMp8() 
{
	// TODO: Add your control notification handler code here
	int i;
		
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	GetDlgItem(IDC_MP8)->EnableWindow(FALSE);	
	pSharedMemory->JW_MotionSet[15] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDemoTest2Dlg::OnMp12() 
{

	
}

void CDemoTest2Dlg::OnMp13() 
{

	
}

void CDemoTest2Dlg::OnMp14() 
{

	
}

void CDemoTest2Dlg::OnMp15() 
{

}

void CDemoTest2Dlg::OnMp16() 
{

	
}

void CDemoTest2Dlg::OnMp17() 
{

	
}

void CDemoTest2Dlg::OnMp18() 
{

	
}

void CDemoTest2Dlg::OnMp19() 
{

	
}

void CDemoTest2Dlg::OnMp20() 
{
	
}

void CDemoTest2Dlg::OnMotion19() 
{

}

void CDemoTest2Dlg::OnMotion20() 
{

}

void CDemoTest2Dlg::OnControl() 
{

}

void CDemoTest2Dlg::OnMotion21() 
{

}

void CDemoTest2Dlg::OnButton3() 
{
	// TODO: Add your control notification handler code here
		CString strText;
	
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_BUTTON3)->GetWindowText(strText);
		
		if(strText == "Button Able")
		{
			GetDlgItem(IDC_BUTTON3)->SetWindowText("Button Disable");

			GetDlgItem(IDC_MOTION1)->EnableWindow(TRUE);
//			GetDlgItem(IDC_MOTION2)->EnableWindow(TRUE);
//			GetDlgItem(IDC_MOTION3)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION4)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION5)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION6)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION6_A)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION6_B)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION7)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION8)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION9)->EnableWindow(TRUE);
//			GetDlgItem(IDC_MOTION10)->EnableWindow(TRUE);
//			GetDlgItem(IDC_MOTION11)->EnableWindow(TRUE);
//			GetDlgItem(IDC_MOTION12)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION13)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION14)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION15)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION16)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION17)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION18)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION19)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION20)->EnableWindow(TRUE);
		//	GetDlgItem(IDC_MOTION21)->EnableWindow(TRUE);
			GetDlgItem(IDC_CONTROL)->EnableWindow(TRUE);

			GetDlgItem(IDC_MP)->EnableWindow(TRUE);
			GetDlgItem(IDC_MP2)->EnableWindow(TRUE);
			GetDlgItem(IDC_MP3)->EnableWindow(TRUE);
			GetDlgItem(IDC_MP4)->EnableWindow(TRUE);
			GetDlgItem(IDC_MP5)->EnableWindow(TRUE);
			GetDlgItem(IDC_MP6)->EnableWindow(TRUE);
			GetDlgItem(IDC_MP7)->EnableWindow(TRUE);
			GetDlgItem(IDC_MP8)->EnableWindow(TRUE);

			GetDlgItem(IDC_MP12)->EnableWindow(TRUE);
			GetDlgItem(IDC_MP13)->EnableWindow(TRUE);
			GetDlgItem(IDC_MP14)->EnableWindow(TRUE);
			GetDlgItem(IDC_MP15)->EnableWindow(TRUE);
			GetDlgItem(IDC_MP16)->EnableWindow(TRUE);
			GetDlgItem(IDC_MP17)->EnableWindow(TRUE);
			GetDlgItem(IDC_MP18)->EnableWindow(TRUE);
			GetDlgItem(IDC_MP19)->EnableWindow(TRUE);
			GetDlgItem(IDC_MP20)->EnableWindow(TRUE);


			GetDlgItem(IDC_MOTION22)->EnableWindow(TRUE);
			//GetDlgItem(IDC_MOTION23)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION24)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION25)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION26)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION27)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION28)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION29)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION31)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION32)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION33)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION34)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION35)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION36)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION37)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION38)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION39)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION40)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION41)->EnableWindow(TRUE);
			GetDlgItem(IDC_MOTION42)->EnableWindow(TRUE);
						
		}
		else
		{
			GetDlgItem(IDC_BUTTON3)->SetWindowText("Button Able");

			GetDlgItem(IDC_MOTION1)->EnableWindow(FALSE);
//			GetDlgItem(IDC_MOTION2)->EnableWindow(FALSE);
//			GetDlgItem(IDC_MOTION3)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION4)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION5)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION6)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION6_A)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION6_B)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION7)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION8)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION9)->EnableWindow(FALSE);
//			GetDlgItem(IDC_MOTION10)->EnableWindow(FALSE);
//			GetDlgItem(IDC_MOTION11)->EnableWindow(FALSE);
//			GetDlgItem(IDC_MOTION12)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION13)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION14)->EnableWindow(FALSE);
//			GetDlgItem(IDC_MOTION15)->EnableWindow(FALSE);
//			GetDlgItem(IDC_MOTION16)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION17)->EnableWindow(FALSE);
//			GetDlgItem(IDC_MOTION18)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION19)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION20)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION21)->EnableWindow(FALSE);
			GetDlgItem(IDC_CONTROL)->EnableWindow(FALSE);

			GetDlgItem(IDC_MP)->EnableWindow(FALSE);
			GetDlgItem(IDC_MP2)->EnableWindow(FALSE);
			GetDlgItem(IDC_MP3)->EnableWindow(FALSE);
			GetDlgItem(IDC_MP4)->EnableWindow(FALSE);
			GetDlgItem(IDC_MP5)->EnableWindow(FALSE);
			GetDlgItem(IDC_MP6)->EnableWindow(FALSE);
			GetDlgItem(IDC_MP7)->EnableWindow(FALSE);
			GetDlgItem(IDC_MP8)->EnableWindow(FALSE);
			
			GetDlgItem(IDC_MP12)->EnableWindow(FALSE);
			GetDlgItem(IDC_MP13)->EnableWindow(FALSE);
			GetDlgItem(IDC_MP14)->EnableWindow(FALSE);
			GetDlgItem(IDC_MP15)->EnableWindow(FALSE);
			GetDlgItem(IDC_MP16)->EnableWindow(FALSE);
			GetDlgItem(IDC_MP17)->EnableWindow(FALSE);
			GetDlgItem(IDC_MP18)->EnableWindow(FALSE);
			GetDlgItem(IDC_MP19)->EnableWindow(FALSE);
			GetDlgItem(IDC_MP20)->EnableWindow(FALSE);
			
			GetDlgItem(IDC_MOTION22)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION23)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION24)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION25)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION26)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION27)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION28)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION29)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION31)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION32)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION33)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION34)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION35)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION36)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION37)->EnableWindow(FALSE);
			GetDlgItem(IDC_MOTION38)->EnableWindow(FALSE);
		}
	}
	else AfxMessageBox("Other Command is activated..!!");
	
}

BOOL CDemoTest2Dlg::OnInitDialog() 
{
	CDialog::OnInitDialog();
	
	// TODO: Add extra initialization here
	GetDlgItem(IDC_MOTION1)->EnableWindow(FALSE);
//	GetDlgItem(IDC_MOTION2)->EnableWindow(FALSE);
//	GetDlgItem(IDC_MOTION3)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION4)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION5)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION6)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION6_A)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION6_B)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION7)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION8)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION9)->EnableWindow(FALSE);
//	GetDlgItem(IDC_MOTION10)->EnableWindow(FALSE);
//	GetDlgItem(IDC_MOTION11)->EnableWindow(FALSE);
//	GetDlgItem(IDC_MOTION12)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION13)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION14)->EnableWindow(FALSE);
//	GetDlgItem(IDC_MOTION15)->EnableWindow(FALSE);
//	GetDlgItem(IDC_MOTION16)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION17)->EnableWindow(FALSE);
//	GetDlgItem(IDC_MOTION18)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION19)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION20)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION21)->EnableWindow(FALSE);
	GetDlgItem(IDC_CONTROL)->EnableWindow(FALSE);

	GetDlgItem(IDC_MP)->EnableWindow(FALSE);
	GetDlgItem(IDC_MP2)->EnableWindow(FALSE);
	GetDlgItem(IDC_MP3)->EnableWindow(FALSE);
	GetDlgItem(IDC_MP4)->EnableWindow(FALSE);
	GetDlgItem(IDC_MP5)->EnableWindow(FALSE);
	GetDlgItem(IDC_MP6)->EnableWindow(FALSE);
	GetDlgItem(IDC_MP7)->EnableWindow(FALSE);
	GetDlgItem(IDC_MP8)->EnableWindow(FALSE);
	
	GetDlgItem(IDC_MP12)->EnableWindow(FALSE);
	GetDlgItem(IDC_MP13)->EnableWindow(FALSE);
	GetDlgItem(IDC_MP14)->EnableWindow(FALSE);
	GetDlgItem(IDC_MP15)->EnableWindow(FALSE);
	GetDlgItem(IDC_MP16)->EnableWindow(FALSE);
	GetDlgItem(IDC_MP17)->EnableWindow(FALSE);
	GetDlgItem(IDC_MP18)->EnableWindow(FALSE);
	GetDlgItem(IDC_MP19)->EnableWindow(FALSE);
	GetDlgItem(IDC_MP20)->EnableWindow(FALSE);

	GetDlgItem(IDC_MOTION22)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION23)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION24)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION25)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION26)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION27)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION28)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION29)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION31)->EnableWindow(FALSE);

	GetDlgItem(IDC_MOTION32)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION33)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION34)->EnableWindow(FALSE);

	GetDlgItem(IDC_MOTION35)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION36)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION37)->EnableWindow(FALSE);
	GetDlgItem(IDC_MOTION38)->EnableWindow(FALSE);
			
	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}

void CDemoTest2Dlg::OnVoice1() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\MBC\\YongMan.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice2() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\MBC\\JaeHoon.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice3() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\MBC\\SooRo.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice4() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\MBC\\SangLyul.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice5() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\MBC\\Hongcheol.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice6() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\MBC\\NaYung.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice7() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\MBC\\Musi.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice8() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\MBC\\WooHyuk.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice9() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\MBC\\IU.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice10() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\MBC\\ChungTeam.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice11() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\MBC\\HongTeam.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice12() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\MBC\\IU_ZZANG.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice13() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
			PlaySound("C:\\voice\\MBC\\GetAway.wav",NULL,SND_ASYNC);
	else if(m_language == 1)
			PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);
	
}

void CDemoTest2Dlg::OnVoice14() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
		PlaySound("C:\\voice\\MBC\\BaBo.wav",NULL,SND_ASYNC);
	else if(m_language == 1)
		PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);

}

void CDemoTest2Dlg::OnVoice15() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
			PlaySound("C:\\voice\\MBC\\MungChung E.wav",NULL,SND_ASYNC);
	else if(m_language == 1)
			PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);
	
}

void CDemoTest2Dlg::OnVoice16() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
		PlaySound("C:\\voice\\MBC\\GOOOOOOD.wav",NULL,SND_ASYNC);
	else if(m_language == 1)
		PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);
	
}

void CDemoTest2Dlg::OnVoice17() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
		PlaySound("C:\\voice\\MBC\\I won.wav",NULL,SND_ASYNC);
	else if(m_language == 1)
		PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);
	
}

void CDemoTest2Dlg::OnVoice18() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
		PlaySound("C:\\voice\\MBC\\come on.wav",NULL,SND_ASYNC);
	else if(m_language == 1)
		PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);
	
}

void CDemoTest2Dlg::OnRps() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->RPS_POS_MODE = 0x00;
		pSharedMemory->CommandFlag = DEMO_RPS;
		GetDlgItem(IDC_RPS)->EnableWindow(FALSE);
		GetDlgItem(IDC_RPS2)->EnableWindow(TRUE);
		GetDlgItem(IDC_RPS3)->EnableWindow(FALSE);
		Sleep(100);
		if(m_language == 0)
			PlaySound("C:\\voice\\MBC\\RockScissor.wav",NULL,SND_ASYNC);
		else if(m_language == 1)
			PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);
		
		
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CDemoTest2Dlg::OnRps2() 
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
		Sleep(50);
		PlaySound("C:\\voice\\MBC\\Paper.wav",NULL,SND_ASYNC);
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CDemoTest2Dlg::OnRps3() 
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

void CDemoTest2Dlg::OnControl2() 
{
	// TODO: Add your control notification handler code here
	
	CString tempString;
	
	GetDlgItem(IDC_CONTROL2)->GetWindowText(tempString);
	
	if(tempString == "Control On (in demo)")
	{
		GetDlgItem(IDC_CONTROL2)->SetWindowText("Control Off (in demo)");
		pSharedMemory->CommandData = DSP_ON;
		pSharedMemory->CommandFlag = DEMO_FLAG;
		Sleep(10);
	}
	else
	{
		GetDlgItem(IDC_CONTROL2)->SetWindowText("Control On (in demo)");
		pSharedMemory->CommandData = DSP_OFF;
		pSharedMemory->CommandFlag = DEMO_FLAG;
		Sleep(10);
	}
}

void CDemoTest2Dlg::OnMotion17() 
{

	
}

void CDemoTest2Dlg::OnMotion18() 
{

	
}

void CDemoTest2Dlg::OnMotion22() 
{

}

void CDemoTest2Dlg::OnMotion23() 
{

}

void CDemoTest2Dlg::OnMp21() 
{
}

void CDemoTest2Dlg::OnVoice19() 
{

	
}

void CDemoTest2Dlg::OnVoice20() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
		PlaySound("C:\\voice\\MBC\\born_k.wav",NULL,SND_ASYNC);
	else if(m_language == 1)
		PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);	
}

void CDemoTest2Dlg::OnVoice21() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
		PlaySound("C:\\voice\\MBC\\4years_k.wav",NULL,SND_ASYNC);
	else if(m_language == 1)
		PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice22() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
		PlaySound("C:\\voice\\MBC\\Idontknow_k.wav",NULL,SND_ASYNC);
	else if(m_language == 1)
		PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);	
}

void CDemoTest2Dlg::OnVoice23() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
		PlaySound("C:\\voice\\MBC\\hehet_k.wav",NULL,SND_ASYNC);
	else if(m_language == 1)
		PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);	
}

void CDemoTest2Dlg::OnVoice24() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
		PlaySound("C:\\voice\\MBC\\hihit_k.wav",NULL,SND_ASYNC);
	else if(m_language == 1)
		PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);	
}

void CDemoTest2Dlg::OnVoice25() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
		PlaySound("C:\\voice\\MBC\\what_k.wav",NULL,SND_ASYNC);
	else if(m_language == 1)
		PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice26() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
		PlaySound("C:\\voice\\MBC\\icant_listen_k.wav",NULL,SND_ASYNC);
	else if(m_language == 1)
		PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);	
}

void CDemoTest2Dlg::OnVoice27() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
			PlaySound("C:\\voice\\MBC\\sagikkun.wav",NULL,SND_ASYNC);
		else if(m_language == 1)
			PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);
	
}

void CDemoTest2Dlg::OnVoice28() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
		PlaySound("C:\\voice\\MBC\\Good.wav",NULL,SND_ASYNC);
	else if(m_language == 1)
		PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);	
}

void CDemoTest2Dlg::OnVoice29() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
		PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);
	else if(m_language == 1)
		PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);	
}

void CDemoTest2Dlg::OnVoice30() 
{
	// TODO: Add your control notification handler code here
	if(m_language == 0)
		PlaySound("C:\\voice\\MBC\\YouComeOn.wav",NULL,SND_ASYNC);
	else if(m_language == 1)
		PlaySound("C:\\voice\\MBC\\NoNo.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnMotion24() 
{
}

void CDemoTest2Dlg::OnMotion25() 
{

}

void CDemoTest2Dlg::OnMotion26() 
{

}

void CDemoTest2Dlg::OnMotion27() 
{

}

void CDemoTest2Dlg::OnMotion28() 
{

}

void CDemoTest2Dlg::OnMotion29() 
{

}

void CDemoTest2Dlg::OnVoice31() 
{

		
	
}

void CDemoTest2Dlg::OnMotion30() 
{

}

void CDemoTest2Dlg::OnMotion31() 
{

}

void CDemoTest2Dlg::OnMotion32() 
{

}

void CDemoTest2Dlg::OnMotion33() 
{

}

void CDemoTest2Dlg::OnMotion34() 
{
}

void CDemoTest2Dlg::OnMotion35() 
{

}

void CDemoTest2Dlg::OnMotion36() 
{


}

void CDemoTest2Dlg::OnMotion37() 
{

}

void CDemoTest2Dlg::OnMotion38() 
{

}

void CDemoTest2Dlg::OnVoice32() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\k_tech\\s3.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice33() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\k_tech\\s7.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice34() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\k_tech\\s9.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice35() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\k_tech\\s11.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice36() 
{
	
	PlaySound("C:\\voice\\k_tech\\s13.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice37() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\k_tech\\s15.wav",NULL,SND_ASYNC);
}
void CDemoTest2Dlg::OnVoice39() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\k_tech\\s20.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnVoice38() 
{
	// TODO: Add your control notification handler code here
	PlaySound("C:\\voice\\k_tech\\s17_2.wav",NULL,SND_ASYNC);
}

void CDemoTest2Dlg::OnMp9() 
{
	GetDlgItem(IDC_MP)->EnableWindow(FALSE);
	// TODO: Add your control notification handler code here
		int i;
		
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	
	pSharedMemory->JW_MotionSet[0] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDemoTest2Dlg::OnMp10() 
{
	GetDlgItem(IDC_MP)->EnableWindow(FALSE);
	// TODO: Add your control notification handler code here
		int i;
		
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	
	pSharedMemory->JW_MotionSet[1] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDemoTest2Dlg::OnMp11() 
{
	GetDlgItem(IDC_MP)->EnableWindow(FALSE);
	// TODO: Add your control notification handler code here
		int i;
		
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	
	pSharedMemory->JW_MotionSet[11] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);				
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}



void CDemoTest2Dlg::OnRadio2() 
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
}

void CDemoTest2Dlg::OnRadio8() 
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
}

void CDemoTest2Dlg::OnVoice40() 
{
	PlaySound("C:\\voice\\MBC\\kwang-hee.wav",NULL,SND_ASYNC);		
}

void CDemoTest2Dlg::OnVoice41() 
{
	PlaySound("C:\\voice\\MBC\\be_quiet.wav",NULL,SND_ASYNC);		
}

void CDemoTest2Dlg::OnVoice42() 
{
	PlaySound("C:\\voice\\MBC\\jo-yong-hi-hae.wav",NULL,SND_ASYNC);		
	
}

void CDemoTest2Dlg::OnMotion15() 
{
	GetDlgItem(IDC_MOTION15)->EnableWindow(FALSE);
	int i;
	
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	
	pSharedMemory->JW_MotionSet[73] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);			
		PlaySound("C:\\voice\\MBC\\mi-wo.wav",NULL,SND_ASYNC);
	}
	else
		AfxMessageBox("Other Command is activated..!!");
	
}

void CDemoTest2Dlg::OnMotion16() 
{
	GetDlgItem(IDC_MOTION16)->EnableWindow(FALSE);
	int i;
	
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
	
	pSharedMemory->JW_MotionSet[72] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);			
		PlaySound("C:\\voice\\MBC\\mi-wo.wav",NULL,SND_ASYNC);
	}
	else
		AfxMessageBox("Other Command is activated..!!");
}

void CDemoTest2Dlg::OnMotion39() 
{
	
}

void CDemoTest2Dlg::OnVoice43() 
{
	PlaySound("C:\\voice\\MBC\\na-dae-ji-ma.wav",NULL,SND_ASYNC);	
}

void CDemoTest2Dlg::OnMotion40() 
{
	int i;
	
	for(i=0; i<NoOfMotion; i++) 
		pSharedMemory->JW_MotionSet[i] = FALSE;		
		
	GetDlgItem(IDC_MOTION40)->EnableWindow(FALSE);
	pSharedMemory->JW_MotionSet[69] = TRUE;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{	
		pSharedMemory->CommandFlag = SET_MOCAP;
		Sleep(10);	
		PlaySound("C:\\voice\\MBC\\jjang-i-e-yo.wav",NULL,SND_ASYNC);
	}
	else
		AfxMessageBox("Other Command is activated..!!");	
}

void CDemoTest2Dlg::OnVoice44() 
{
	PlaySound("C:\\voice\\MBC\\pity.wav",NULL,SND_ASYNC);	
}

void CDemoTest2Dlg::OnVoice45() 
{
	PlaySound("C:\\voice\\MBC\\out.wav",NULL,SND_ASYNC);
	
}

void CDemoTest2Dlg::OnVoice46() 
{
	PlaySound("C:\\voice\\MBC\\yo.wav",NULL,SND_ASYNC);
	
}

void CDemoTest2Dlg::OnVoice47() 
{
	PlaySound("C:\\voice\\MBC\\yo_question.wav",NULL,SND_ASYNC);
	
}

void CDemoTest2Dlg::OnMotion41() 
{
	
	
}

void CDemoTest2Dlg::OnMotion42() 
{

}
