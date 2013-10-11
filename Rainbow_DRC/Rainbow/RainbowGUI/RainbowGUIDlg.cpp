// RainbowGUIDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "RainbowGUIDlg.h"
#include <math.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


// SharedMemory variable
	PSHARED_DATA pSharedMemory;

/////////////////////////////////////////////////////////////////////////////
// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	//{{AFX_DATA(CAboutDlg)
	enum { IDD = IDD_ABOUTBOX };
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CAboutDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	//{{AFX_MSG(CAboutDlg)
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
	//{{AFX_DATA_INIT(CAboutDlg)
	//}}AFX_DATA_INIT
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CAboutDlg)
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
	//{{AFX_MSG_MAP(CAboutDlg)
		// No message handlers
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CRainbowGUIDlg dialog

CRainbowGUIDlg::CRainbowGUIDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CRainbowGUIDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CRainbowGUIDlg)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CRainbowGUIDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CRainbowGUIDlg)
		// NOTE: the ClassWizard will add DDX and DDV calls here
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CRainbowGUIDlg, CDialog)
	//{{AFX_MSG_MAP(CRainbowGUIDlg)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_EXIT, OnExit)
	ON_BN_CLICKED(IDC_RTXONOFF, OnRtxonoff)
	ON_BN_CLICKED(IDC_LOADPARAM, OnLoadparam)
	ON_BN_CLICKED(IDC_CHECKDEVICE, OnCheckdevice)
	ON_BN_CLICKED(IDC_CONTROLONOFF, OnControlonoff)
	ON_WM_TIMER()
	ON_MESSAGE(WM_COMM_READ, OnReceive)    // Communication Message Handleer
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CRainbowGUIDlg message handlers

BOOL CRainbowGUIDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon
	
	// TODO: Add extra initialization here
	RTXStarted = false;

	// Comport
	m_Comm.hCommWnd = this->m_hWnd;

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CRainbowGUIDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CRainbowGUIDlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CRainbowGUIDlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}

void CRainbowGUIDlg::OnExit() 
{
	// TODO: Add your control notification handler code here
	if(RTXStarted == true) RTXOff();
	OnOK();
}

void CRainbowGUIDlg::OnRtxonoff() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	GetDlgItem(IDC_RTXONOFF)->GetWindowText(strText);
	
	if(strText == "RTX ON")
	{
		RTXOn();
		GetDlgItem(IDC_RTXONOFF)->SetWindowText("RTX OFF");
		GetDlgItem(IDC_LOADPARAM)->EnableWindow(TRUE);
		RTXStarted = true;
	}
	else
	{
		RTXOff();
		GetDlgItem(IDC_RTXONOFF)->SetWindowText("RTX ON");
		RTXStarted = false;
	}
}

bool CRainbowGUIDlg::RTXOn()
{
//	LPCTSTR RTSS_RUNNER = "..\\RAINBOW___Win32_RTSS_Release\\RAINBOW.rtss";
	
	LPCTSTR RTSS_RUNNER = "C:\\Rainbow_DRC\\Rainbow\\Rainbow___Win32_RTSS_Release\\RAINBOW.rtss";
	PROCESS_INFORMATION pi;
	
	// initialize SharedMemory
	hSharedMemory = RtCreateSharedMemory(
											PAGE_READWRITE,				// access mode
											0,							// maximum size high
											sizeof(SHARED_DATA),		// maximum size low	
											"Can Shared Data",			// name of shared memory
											(VOID **)&pSharedMemory);	// shared memory data address 
	
	if (hSharedMemory == NULL) return false;

	if ( !RtCreateProcess(
					RTSS_RUNNER,	// path of RTX program
					NULL,			// command line to execute
					NULL,			// don't care
					NULL,			// don't care
					NULL,			// don't care
					NULL,			// don't care
					NULL,			// don't care
					NULL,			// don't care
					NULL,			// don't care
					&pi ))			// process information
	{
		RtWprintf(L"RtCreateProcess error = %d\n", GetLastError());
		return false;
	}
	else RtCloseHandle(pi.hProcess);
	
	Sleep(500);  // very important !!!
	return true;
}

bool CRainbowGUIDlg::RTXOff()
{
	pSharedMemory->CommandFlag = DISABLE_FET;
	Sleep(500);
	pSharedMemory->CommandFlag = EXIT_PROGRAM;
	RtCloseHandle(hSharedMemory);

	return true;
}

void CRainbowGUIDlg::OnLoadparam() 
{
	// TODO: Add your control notification handler code here
	if ( pSharedMemory->CommandFlag != NO_ACT) AfxMessageBox("Other Command is activated..!!");
	else 
	{
		// device check
		Sleep(500);
		pSharedMemory->CommandFlag = CHECK_DEVICE;
		//GetDlgItem(IDC_CONTROLONOFF)->EnableWindow(TRUE);
		
		// Comport Open
		//OpenComPort();

		Sleep(1000);
	}

	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = LOAD_PARAMETER;
		GetDlgItem(IDC_CHECKDEVICE)->EnableWindow(TRUE);
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CRainbowGUIDlg::OnCheckdevice() 
{
	// TODO: Add your control notification handler code here
	if ( pSharedMemory->CommandFlag != NO_ACT) AfxMessageBox("Other Command is activated..!!");
	else 
	{
		// device check
		//pSharedMemory->CommandFlag = CHECK_DEVICE;
		GetDlgItem(IDC_CONTROLONOFF)->EnableWindow(TRUE);

//		Sleep(500);
		pSharedMemory->CommandFlag = ENABLE_FET;
	}
}

void CRainbowGUIDlg::OpenComPort()
{
	// TODO: Add your control notification handler code here
	if (!m_Comm.m_bConnected)
	{
		CString strPortNum;
		//m_ctrComSelect.GetWindowTextA(strPortNum);
		strPortNum = "COM5";
		if(m_Comm.OpenPort("\\\\.\\"+strPortNum, "9600", "8","NO","NO","1"))
		{
			// Connection Success
			//PrintStr(&CString("Connected to " + strPortNum));
			pSharedMemory->COM_Ready_Flag = TRUE;
		}
		else
		{
			// Connection Fail
			pSharedMemory->COM_Ready_Flag = FALSE;
			AfxMessageBox("Cannot Connect ComPort..!!");
		}
	}
}

LONG CRainbowGUIDlg::OnReceive(UINT port, LONG lParam)
{
	BYTE aByte;
	m_Comm.m_bReserveMsg = false;
	static CString strReceive;

	pSharedMemory->RBT_Parsing_Cnt++;

	int size = (m_Comm.m_QueueRead).GetSize();
	if (size == 0) return 0;

	for (int i=0;i<size;i++)
	{
		(m_Comm.m_QueueRead).GetByte(&aByte);
		//aByte 이용 여기서 데이터 처리
		
		if(aByte != '\0'){
			strReceive += aByte;
		}else{
			//PrintStr(&CString("<< " + strReceive));
			UpdateData(TRUE);
			ParsingRxData(strReceive);
			UpdateData(FALSE);
			strReceive.Empty();
		}
	}
	return 0;
}


void CRainbowGUIDlg::ParsingRxData(CString strRx)
{
	int nCntX=0;
	int nCntY=0;
	//int nCntZ=0;
	int nCntV=0;
	//CString PosX, PosY, PosZ, VoiceR;
	CString PosX, PosY, VoiceR;

	int m_Pos_X = 0;
	int m_Pos_Y = 0;
	//float m_Pos_Z = 0.f;
	int m_Voice_R = 0;

	nCntX = strRx.Find("X");
	nCntY = strRx.Find("Y");
	//nCntZ = strRx.Find("Z");
	nCntV = strRx.Find("V");
	
	PosX = strRx.Mid(nCntX+1, nCntY-1);
	//PosY = strRx.Mid(nCntY+1, nCntZ-1);
	PosY = strRx.Mid(nCntY+1, nCntV-1);
	//PosZ = strRx.Mid(nCntZ+1, nCntV-1);
	VoiceR = strRx.Mid(nCntV+1, strRx.GetLength());
	
	m_Pos_X = atoi(PosX);
	m_Pos_Y = atoi(PosY);
	//m_Pos_Z = (float)(atof(PosZ));
	m_Voice_R = atoi(VoiceR);

	// Gain for Feed-forward, Using Kinect
	pSharedMemory->RBT_Yaw_Kp = 0.15f;
	pSharedMemory->RBT_Yaw_Kd = -0.12f;
	pSharedMemory->RBT_Pitch_Kp = 0.15f;
	pSharedMemory->RBT_Pitch_Kd = 0.12f;
	
	if(pSharedMemory->COM_Ready_Flag)
	{
		pSharedMemory->Voice_Recog_Command = m_Voice_R;
	}

	if((m_Pos_X>0) && (m_Pos_X<1000) && (m_Pos_Y>0) && (m_Pos_Y<1000) && (pSharedMemory->COM_Ready_Flag))
	{
		pSharedMemory->RBT_PosX = (int)m_Pos_X;
		pSharedMemory->RBT_PosY = (int)m_Pos_Y;
		//pSharedMemory->RBT_PosZ = (float)m_Pos_Z;

		pSharedMemory->RBT_Ready_Flag = TRUE;
	}
	else
	{
		pSharedMemory->RBT_Ready_Flag = FALSE;
	}
}


void CRainbowGUIDlg::OnControlonoff() 
{
	// TODO: Add your control notification handler code here
	CString strText;
	GetDlgItem(IDC_CONTROLONOFF)->GetWindowText(strText);

	if(strText == "CTRL. ON")
	{
		if(pSharedMemory->CommandFlag == NO_ACT)
		{
			pSharedMemory->CommandFlag = RUN_CMD;
			GetDlgItem(IDC_CONTROLONOFF)->SetWindowText("CTRL. OFF");

			if(m_propertySheet.GetPageCount() <= 0)
			{
				// create property sheet and add property page
				m_propertySheet.AddPage(&m_initHUBO2Dlg);	//0
				m_propertySheet.AddPage(&m_walkingDlg);		//1
				m_propertySheet.AddPage(&m_setSensorDlg);	//2
				m_propertySheet.AddPage(&m_presetDlg);		//3
				//m_propertySheet.AddPage(&m_setJointDlg);
				m_propertySheet.AddPage(&m_jmcSettingDlg);	//4
				m_propertySheet.AddPage(&m_lStatusDlg);		//5
				m_propertySheet.AddPage(&m_uStatusDlg);		//6
				m_propertySheet.AddPage(&m_userDlg);		//7
				m_propertySheet.AddPage(&m_encoderDlg);		//8
				m_propertySheet.Create(this, WS_CHILD | WS_VISIBLE, 0);
				m_propertySheet.ModifyStyleEx (0, WS_EX_CONTROLPARENT);
				m_propertySheet.ModifyStyle(0, WS_TABSTOP);
				CRect rcSheet;
				GetDlgItem( IDC_PROPERTYSHEET )->GetWindowRect( &rcSheet );
				ScreenToClient( &rcSheet );
				m_propertySheet.SetWindowPos( NULL, rcSheet.left-7, rcSheet.top-7, 0, 0, SWP_NOZORDER | SWP_NOSIZE | SWP_NOACTIVATE );

				// set timer for display
				SetTimer(0, 100, NULL);
			}
		}
		else AfxMessageBox("Other Command is activated..!!");
	}
	else
	{
		if(pSharedMemory->CommandFlag == NO_ACT)
		{
			pSharedMemory->CommandFlag = STOP_CMD;
			GetDlgItem(IDC_CONTROLONOFF)->SetWindowText("CTRL. ON");
			
		}
		else AfxMessageBox("Other Command is activated..!!");
	}
}

void CRainbowGUIDlg::OnTimer(UINT nIDEvent) 
{
	// TODO: Add your message handler code here and/or call default
	
	int index;
	CString strText;
	float x, y, z, dc[4][4], qt[5],rv[5];
	switch(nIDEvent)
	{
	case 0:
		if(RTXStarted == true)
		{
			index = m_propertySheet.GetActiveIndex();
			switch(index)
			{
			case 0:	// initHUBO2 page
				strText.Format("%f", pSharedMemory->Joint_debug[RHY].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RHY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RHR].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RHR)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RHP].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RHP)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RKN].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RKN)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RAP].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RAP)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RAR].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RAR)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->Joint_debug[LHY].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LHY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LHR].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LHR)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LHP].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LHP)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LKN].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LKN)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LAP].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LAP)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LAR].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LAR)->SetWindowText(strText);
				
				strText.Format("%f", pSharedMemory->Joint_debug[RSP].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RSP)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RSR].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RSR)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RSY].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RSY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[REB].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_REB)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RWY].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RWY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RWP].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RWP)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->Joint_debug[LSP].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LSP)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LSR].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LSR)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LSY].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LSY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LEB].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LEB)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LWY].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LWY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LWP].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LWP)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->Joint_debug[NKY].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_NKY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[NK1].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_NK1)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[NK2].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_NK2)->SetWindowText(strText);
				
				strText.Format("%f", pSharedMemory->Joint_debug[WST].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_WST)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->Joint_debug[RF1].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RF1)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RF2].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RF2)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RF3].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RF3)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RF4].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RF4)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[RF5].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_RF5)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->Joint_debug[LF1].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LF1)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LF2].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LF2)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LF3].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LF3)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LF4].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LF4)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->Joint_debug[LF5].RefAngleCurrent);	m_initHUBO2Dlg.GetDlgItem(IDC_LF5)->SetWindowText(strText);
				break;
			case 1:	// walking page
				break;
			case 2:	// sensor page
				strText.Format("%f", pSharedMemory->FT_debug[RFFT].Mx);			m_setSensorDlg.GetDlgItem(IDC_RMX)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[RFFT].My);			m_setSensorDlg.GetDlgItem(IDC_RMY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[RFFT].Fz);			m_setSensorDlg.GetDlgItem(IDC_RFZ)->SetWindowText(strText);
// 				strText.Format("%f", pSharedMemory->FT_debug[RFFT].Filtered_Mx);			m_setSensorDlg.GetDlgItem(IDC_RMX)->SetWindowText(strText);
// 				strText.Format("%f", pSharedMemory->FT_debug[RFFT].Filtered_My);			m_setSensorDlg.GetDlgItem(IDC_RMY)->SetWindowText(strText);
// 				strText.Format("%f", pSharedMemory->FT_debug[RFFT].Filtered_Fz);			m_setSensorDlg.GetDlgItem(IDC_RFZ)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[RFFT].Roll);	m_setSensorDlg.GetDlgItem(IDC_RROLL)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[RFFT].Pitch);	m_setSensorDlg.GetDlgItem(IDC_RPITCH)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->FT_debug[LFFT].Mx);			m_setSensorDlg.GetDlgItem(IDC_LMX)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[LFFT].My);			m_setSensorDlg.GetDlgItem(IDC_LMY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[LFFT].Fz);			m_setSensorDlg.GetDlgItem(IDC_LFZ)->SetWindowText(strText);
// 				strText.Format("%f", pSharedMemory->FT_debug[LFFT].Filtered_Mx);			m_setSensorDlg.GetDlgItem(IDC_LMX)->SetWindowText(strText);
// 				strText.Format("%f", pSharedMemory->FT_debug[LFFT].Filtered_My);			m_setSensorDlg.GetDlgItem(IDC_LMY)->SetWindowText(strText);
// 				strText.Format("%f", pSharedMemory->FT_debug[LFFT].Filtered_Fz);			m_setSensorDlg.GetDlgItem(IDC_LFZ)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[LFFT].Roll);	m_setSensorDlg.GetDlgItem(IDC_LROLL)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[LFFT].Pitch);	m_setSensorDlg.GetDlgItem(IDC_LPITCH)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->FT_debug[RWFT].Mx);			m_setSensorDlg.GetDlgItem(IDC_RWMX)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[RWFT].My);			m_setSensorDlg.GetDlgItem(IDC_RWMY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[RWFT].Fz);			m_setSensorDlg.GetDlgItem(IDC_RWFZ)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->FT_debug[LWFT].Mx);			m_setSensorDlg.GetDlgItem(IDC_LWMX)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[LWFT].My);			m_setSensorDlg.GetDlgItem(IDC_LWMY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->FT_debug[LWFT].Fz);			m_setSensorDlg.GetDlgItem(IDC_LWFZ)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->IMU_debug[CENTERIMU].Roll);				m_setSensorDlg.GetDlgItem(IDC_ROLL)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->IMU_debug[CENTERIMU].Roll_Velocity);	m_setSensorDlg.GetDlgItem(IDC_ROLLV)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->IMU_debug[CENTERIMU].Pitch);			m_setSensorDlg.GetDlgItem(IDC_PITCH)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->IMU_debug[CENTERIMU].Pitch_Velocity);	m_setSensorDlg.GetDlgItem(IDC_PITCHV)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->IMU_debug[CENTERIMU].Yaw_angle);		m_setSensorDlg.GetDlgItem(IDC_YAW)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->IMU_debug[CENTERIMU].Yaw_Velocity);		m_setSensorDlg.GetDlgItem(IDC_YAWV)->SetWindowText(strText);

				x = pSharedMemory->IMU_debug[CENTERIMU].Roll/180.f*3.141592653589793;
				y = pSharedMemory->IMU_debug[CENTERIMU].Pitch/180.f*3.141592653589793;
				z = pSharedMemory->IMU_debug[CENTERIMU].Yaw_angle/180.f*3.141592653589793;

				dc[1][1] = cos(y)*cos(z);
				dc[1][2] = cos(z)*sin(x)*sin(y) - cos(x)*sin(z);
				dc[1][3] = sin(x)*sin(z) + cos(x)*cos(z)*sin(y);

				dc[2][1] = cos(y)*sin(z);
				dc[2][2] = cos(x)*cos(z) + sin(x)*sin(y)*sin(z);
				dc[2][3] = cos(x)*sin(y)*sin(z) - cos(z)*sin(x);

				dc[3][1] = -sin(y);
				dc[3][2] = cos(y)*sin(x);
				dc[3][3] = cos(x)*cos(y);

				DC2QT(dc, qt);
				QT2RV(qt,rv);

				strText.Format("%f", rv[0]*180.f/3.141592653589793);		m_setSensorDlg.GetDlgItem(IDC_RV0)->SetWindowText(strText);
				strText.Format("%f", rv[1]);								m_setSensorDlg.GetDlgItem(IDC_RV1)->SetWindowText(strText);
				strText.Format("%f", rv[2]);								m_setSensorDlg.GetDlgItem(IDC_RV2)->SetWindowText(strText);
				strText.Format("%f", rv[3]);								m_setSensorDlg.GetDlgItem(IDC_RV3)->SetWindowText(strText);


				strText.Format("%f", pSharedMemory->ZMP[0]);	m_setSensorDlg.GetDlgItem(IDC_ZMPX)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->ZMP[1]);	m_setSensorDlg.GetDlgItem(IDC_ZMPY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->ZMP[2]);	m_setSensorDlg.GetDlgItem(IDC_RZMPX)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->ZMP[3]);	m_setSensorDlg.GetDlgItem(IDC_RZMPY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->ZMP[4]);	m_setSensorDlg.GetDlgItem(IDC_LZMPX)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->ZMP[5]);	m_setSensorDlg.GetDlgItem(IDC_LZMPY)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->WB_fRS[0]);	m_setSensorDlg.GetDlgItem(IDC_FRH_TIPX)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->WB_fRS[1]);	m_setSensorDlg.GetDlgItem(IDC_FRH_TIPY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->WB_fRS[2]);	m_setSensorDlg.GetDlgItem(IDC_FRH_TIPZ)->SetWindowText(strText);

				strText.Format("%f", pSharedMemory->WB_fLS[0]);	m_setSensorDlg.GetDlgItem(IDC_FLH_TIPX)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->WB_fLS[1]);	m_setSensorDlg.GetDlgItem(IDC_FLH_TIPY)->SetWindowText(strText);
				strText.Format("%f", pSharedMemory->WB_fLS[2]);	m_setSensorDlg.GetDlgItem(IDC_FLH_TIPZ)->SetWindowText(strText);
				break;
			case 3:	// preset position
				break;
			case 4:	// JMC page
				break;
			case 5:	// LStatus page
				// Home status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_RKN)->SetWindowText(strText);
				//strText.Format("%d", pSharedMemory->Joint_debug[RHY].LimitOnOff);	m_lStatusDlg.GetDlgItem(IDC_HOME_RHP)->SetWindowText(strText);
				//strText.Format("%d", pSharedMemory->Joint_debug[RHR].LimitOnOff);	m_lStatusDlg.GetDlgItem(IDC_HOME_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].HomeStatus);	m_lStatusDlg.GetDlgItem(IDC_HOME_LAR)->SetWindowText(strText);

				// JAM error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].JAMError);	m_lStatusDlg.GetDlgItem(IDC_JAM_LAR)->SetWindowText(strText);

				// PWM error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].PWMError);	m_lStatusDlg.GetDlgItem(IDC_PWM_LAR)->SetWindowText(strText);

				// Big error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].BigError);	m_lStatusDlg.GetDlgItem(IDC_BIG_LAR)->SetWindowText(strText);

				// Encoder error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].EncoderError);	m_lStatusDlg.GetDlgItem(IDC_ENC_LAR)->SetWindowText(strText);

				// FET driver error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].FaultError);	m_lStatusDlg.GetDlgItem(IDC_DRV_LAR)->SetWindowText(strText);

				// Motor0 error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].Motor0Error);	m_lStatusDlg.GetDlgItem(IDC_M0E_LAR)->SetWindowText(strText);

				// Motor1 error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].Motor1Error);	m_lStatusDlg.GetDlgItem(IDC_M1E_LAR)->SetWindowText(strText);

				// Upper limit error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].ULimitError);	m_lStatusDlg.GetDlgItem(IDC_UPL_LAR)->SetWindowText(strText);

				// Lower limit error status
				strText.Format("%d", pSharedMemory->Joint_debug[RHY].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_RHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHR].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_RHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RHP].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_RHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RKN].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_RKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_RAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAR].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_RAR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHY].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_LHY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHR].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_LHR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LHP].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_LHP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LKN].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_LKN)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAP].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_LAP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LAR].LLimitError);	m_lStatusDlg.GetDlgItem(IDC_LOL_LAR)->SetWindowText(strText);
				break;
			case 6:	// UStatus page
				// Home status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].HomeStatus);	m_uStatusDlg.GetDlgItem(IDC_HOME_LWP)->SetWindowText(strText);

				// JAM error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].JAMError);	m_uStatusDlg.GetDlgItem(IDC_JAM_LWP)->SetWindowText(strText);

				// PWM error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].PWMError);	m_uStatusDlg.GetDlgItem(IDC_PWM_LWP)->SetWindowText(strText);

				// Big error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].BigError);	m_uStatusDlg.GetDlgItem(IDC_BIG_LWP)->SetWindowText(strText);

				// Encoder error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].EncoderError);	m_uStatusDlg.GetDlgItem(IDC_ENC_LWP)->SetWindowText(strText);

				// FET driver error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].FaultError);	m_uStatusDlg.GetDlgItem(IDC_DRV_LWP)->SetWindowText(strText);

				// Motor0 error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].Motor0Error);	m_uStatusDlg.GetDlgItem(IDC_M0E_LWP)->SetWindowText(strText);

				// Motor1 error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RAP].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].Motor1Error);	m_uStatusDlg.GetDlgItem(IDC_M1E_LWP)->SetWindowText(strText);

				// Upper limit error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].ULimitError);	m_uStatusDlg.GetDlgItem(IDC_UPL_LWP)->SetWindowText(strText);

				// Lower limit error status
				strText.Format("%d", pSharedMemory->Joint_debug[RSP].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_RSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSR].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_RSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RSY].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_RSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[REB].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_REB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWY].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_RWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[RWP].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_RWP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_LSP)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_LSR)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_LSY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_LEB)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_LWY)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].LLimitError);	m_uStatusDlg.GetDlgItem(IDC_LOL_LWP)->SetWindowText(strText);
				break;
			case 7:	// user page
				break;
			case 8:	// encoder page
				/*
				strText.Format("%d", pSharedMemory->Joint_debug[LSP].EncoderValue);	m_encoderDlg.GetDlgItem(IDC_EDIT1)->SetWindowText(strText);				
				strText.Format("%d", pSharedMemory->Joint_debug[LSR].EncoderValue);	m_encoderDlg.GetDlgItem(IDC_EDIT2)->SetWindowText(strText);
				strText.Format("%d", pSharedMemory->Joint_debug[LSY].EncoderValue);	m_encoderDlg.GetDlgItem(IDC_EDIT3)->SetWindowText(strText);				
				strText.Format("%d", pSharedMemory->Joint_debug[LEB].EncoderValue);	m_encoderDlg.GetDlgItem(IDC_EDIT4)->SetWindowText(strText);				
				strText.Format("%d", pSharedMemory->Joint_debug[LWY].EncoderValue);	m_encoderDlg.GetDlgItem(IDC_EDIT5)->SetWindowText(strText);				
				strText.Format("%d", pSharedMemory->Joint_debug[LWP].EncoderValue);	m_encoderDlg.GetDlgItem(IDC_EDIT6)->SetWindowText(strText);				
				strText.Format("%d", pSharedMemory->Joint_debug[LWY2].EncoderValue); m_encoderDlg.GetDlgItem(IDC_EDIT40)->SetWindowText(strText);				
				*/

				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[RSP].EncoderValue/pSharedMemory->Joint_debug[RSP].PPR);		m_encoderDlg.GetDlgItem(IDC_EDIT1)->SetWindowText(strText);				
				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[RSR].EncoderValue/pSharedMemory->Joint_debug[RSR].PPR - 10.f);	m_encoderDlg.GetDlgItem(IDC_EDIT2)->SetWindowText(strText);				
				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[RSY].EncoderValue/pSharedMemory->Joint_debug[RSY].PPR);		m_encoderDlg.GetDlgItem(IDC_EDIT3)->SetWindowText(strText);				
				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[REB].EncoderValue/pSharedMemory->Joint_debug[REB].PPR - 20.f);	m_encoderDlg.GetDlgItem(IDC_EDIT4)->SetWindowText(strText);				
				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[RWY].EncoderValue/pSharedMemory->Joint_debug[RWY].PPR);		m_encoderDlg.GetDlgItem(IDC_EDIT5)->SetWindowText(strText);				
				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[RWP].EncoderValue/pSharedMemory->Joint_debug[RWP].PPR);		m_encoderDlg.GetDlgItem(IDC_EDIT6)->SetWindowText(strText);				
				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[RWY2].EncoderValue/pSharedMemory->Joint_debug[RWY2].PPR);		m_encoderDlg.GetDlgItem(IDC_EDIT40)->SetWindowText(strText);				
					
				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[LSP].EncoderValue/pSharedMemory->Joint_debug[LSP].PPR);		m_encoderDlg.GetDlgItem(IDC_EDIT7)->SetWindowText(strText);				
				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[LSR].EncoderValue/pSharedMemory->Joint_debug[LSR].PPR + 10.f);	m_encoderDlg.GetDlgItem(IDC_EDIT8)->SetWindowText(strText);				
				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[LSY].EncoderValue/pSharedMemory->Joint_debug[LSY].PPR);		m_encoderDlg.GetDlgItem(IDC_EDIT9)->SetWindowText(strText);				
				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[LEB].EncoderValue/pSharedMemory->Joint_debug[LEB].PPR - 20.f);	m_encoderDlg.GetDlgItem(IDC_EDIT10)->SetWindowText(strText);				
				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[LWY].EncoderValue/pSharedMemory->Joint_debug[LWY].PPR);		m_encoderDlg.GetDlgItem(IDC_EDIT11)->SetWindowText(strText);				
				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[LWP].EncoderValue/pSharedMemory->Joint_debug[LWP].PPR);		m_encoderDlg.GetDlgItem(IDC_EDIT12)->SetWindowText(strText);				
				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[LWY2].EncoderValue/pSharedMemory->Joint_debug[LWY2].PPR);		m_encoderDlg.GetDlgItem(IDC_EDIT42)->SetWindowText(strText);				

				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[RAP].EncoderValue/pSharedMemory->Joint_debug[RAP].PPR); m_encoderDlg.GetDlgItem(IDC_EDIT109)->SetWindowText(strText);				
				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[RAR].EncoderValue/pSharedMemory->Joint_debug[RAR].PPR); m_encoderDlg.GetDlgItem(IDC_EDIT110)->SetWindowText(strText);
				
				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[LAP].EncoderValue/pSharedMemory->Joint_debug[LAP].PPR); m_encoderDlg.GetDlgItem(IDC_EDIT115)->SetWindowText(strText);				
				strText.Format("%.4f", (float)pSharedMemory->Joint_debug[LAR].EncoderValue/pSharedMemory->Joint_debug[LAR].PPR); m_encoderDlg.GetDlgItem(IDC_EDIT116)->SetWindowText(strText);
				
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[20]);	m_encoderDlg.GetDlgItem(IDC_EDIT13)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[21]);	m_encoderDlg.GetDlgItem(IDC_EDIT14)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[22]);	m_encoderDlg.GetDlgItem(IDC_EDIT15)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[23]);	m_encoderDlg.GetDlgItem(IDC_EDIT16)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[24]);	m_encoderDlg.GetDlgItem(IDC_EDIT17)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[25]);	m_encoderDlg.GetDlgItem(IDC_EDIT18)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[32]);	m_encoderDlg.GetDlgItem(IDC_EDIT44)->SetWindowText(strText);
				
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[26]);	m_encoderDlg.GetDlgItem(IDC_EDIT19)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[27]);	m_encoderDlg.GetDlgItem(IDC_EDIT20)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[28]);	m_encoderDlg.GetDlgItem(IDC_EDIT21)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[29]);	m_encoderDlg.GetDlgItem(IDC_EDIT22)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[30]);	m_encoderDlg.GetDlgItem(IDC_EDIT23)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[31]);	m_encoderDlg.GetDlgItem(IDC_EDIT24)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[33]);	m_encoderDlg.GetDlgItem(IDC_EDIT25)->SetWindowText(strText);

				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[12]);	m_encoderDlg.GetDlgItem(IDC_EDIT121)->SetWindowText(strText); //RAP
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[13]);	m_encoderDlg.GetDlgItem(IDC_EDIT122)->SetWindowText(strText); //RAR

				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[18]);	m_encoderDlg.GetDlgItem(IDC_EDIT127)->SetWindowText(strText); //LAP
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[19]);	m_encoderDlg.GetDlgItem(IDC_EDIT128)->SetWindowText(strText); //LAR

				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[9]);	m_encoderDlg.GetDlgItem(IDC_EDIT118)->SetWindowText(strText); //RHR
				strText.Format("%.4f", pSharedMemory->disp_duty_33x1[15]);	m_encoderDlg.GetDlgItem(IDC_EDIT124)->SetWindowText(strText); //LHR
				
				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[26]);	m_encoderDlg.GetDlgItem(IDC_EDIT33)->SetWindowText(strText);	
				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[27]);	m_encoderDlg.GetDlgItem(IDC_EDIT34)->SetWindowText(strText);	
				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[28]);	m_encoderDlg.GetDlgItem(IDC_EDIT35)->SetWindowText(strText);	
				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[29]);	m_encoderDlg.GetDlgItem(IDC_EDIT36)->SetWindowText(strText);	
				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[30]);	m_encoderDlg.GetDlgItem(IDC_EDIT37)->SetWindowText(strText);	
				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[31]);	m_encoderDlg.GetDlgItem(IDC_EDIT38)->SetWindowText(strText);	
				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[33]);	m_encoderDlg.GetDlgItem(IDC_EDIT39)->SetWindowText(strText);
				
				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[20]);	m_encoderDlg.GetDlgItem(IDC_EDIT52)->SetWindowText(strText);	
				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[21]);	m_encoderDlg.GetDlgItem(IDC_EDIT53)->SetWindowText(strText);	
				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[22]);	m_encoderDlg.GetDlgItem(IDC_EDIT54)->SetWindowText(strText);	
				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[23]);	m_encoderDlg.GetDlgItem(IDC_EDIT55)->SetWindowText(strText);	
				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[24]);	m_encoderDlg.GetDlgItem(IDC_EDIT56)->SetWindowText(strText);	
				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[25]);	m_encoderDlg.GetDlgItem(IDC_EDIT57)->SetWindowText(strText);	
				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[32]);	m_encoderDlg.GetDlgItem(IDC_EDIT58)->SetWindowText(strText);

				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[12]);	m_encoderDlg.GetDlgItem(IDC_EDIT133)->SetWindowText(strText);
				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[14]);	m_encoderDlg.GetDlgItem(IDC_EDIT134)->SetWindowText(strText);

				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[18]);	m_encoderDlg.GetDlgItem(IDC_EDIT139)->SetWindowText(strText);
				strText.Format("%.4f", pSharedMemory->disp_ct_33x1[19]);	m_encoderDlg.GetDlgItem(IDC_EDIT140)->SetWindowText(strText);

				strText.Format("%.4f", pSharedMemory->disp_grav_33x1[26]);	m_encoderDlg.GetDlgItem(IDC_EDIT59)->SetWindowText(strText);
				strText.Format("%.4f", pSharedMemory->disp_grav_33x1[27]);	m_encoderDlg.GetDlgItem(IDC_EDIT60)->SetWindowText(strText);
				strText.Format("%.4f", pSharedMemory->disp_grav_33x1[28]);	m_encoderDlg.GetDlgItem(IDC_EDIT61)->SetWindowText(strText);
				strText.Format("%.4f", pSharedMemory->disp_grav_33x1[29]);	m_encoderDlg.GetDlgItem(IDC_EDIT62)->SetWindowText(strText);
				strText.Format("%.4f", pSharedMemory->disp_grav_33x1[30]);	m_encoderDlg.GetDlgItem(IDC_EDIT63)->SetWindowText(strText);
				strText.Format("%.4f", pSharedMemory->disp_grav_33x1[31]);	m_encoderDlg.GetDlgItem(IDC_EDIT64)->SetWindowText(strText);
				strText.Format("%.4f", pSharedMemory->disp_grav_33x1[33]);	m_encoderDlg.GetDlgItem(IDC_EDIT65)->SetWindowText(strText);

				strText.Format("%.4f", pSharedMemory->disp_grav_33x1[20]);	m_encoderDlg.GetDlgItem(IDC_EDIT66)->SetWindowText(strText);
				strText.Format("%.4f", pSharedMemory->disp_grav_33x1[21]);	m_encoderDlg.GetDlgItem(IDC_EDIT67)->SetWindowText(strText);
				strText.Format("%.4f", pSharedMemory->disp_grav_33x1[22]);	m_encoderDlg.GetDlgItem(IDC_EDIT68)->SetWindowText(strText);
				strText.Format("%.4f", pSharedMemory->disp_grav_33x1[23]);	m_encoderDlg.GetDlgItem(IDC_EDIT69)->SetWindowText(strText);
				strText.Format("%.4f", pSharedMemory->disp_grav_33x1[24]);	m_encoderDlg.GetDlgItem(IDC_EDIT70)->SetWindowText(strText);
				strText.Format("%.4f", pSharedMemory->disp_grav_33x1[25]);	m_encoderDlg.GetDlgItem(IDC_EDIT71)->SetWindowText(strText);
				strText.Format("%.4f", pSharedMemory->disp_grav_33x1[32]);	m_encoderDlg.GetDlgItem(IDC_EDIT72)->SetWindowText(strText);

				strText.Format("%.4f", pSharedMemory->disp_pLH[0]);	m_encoderDlg.GetDlgItem(IDC_EDIT26)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_pLH[1]);	m_encoderDlg.GetDlgItem(IDC_EDIT27)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_pLH[2]);	m_encoderDlg.GetDlgItem(IDC_EDIT28)->SetWindowText(strText);				

				strText.Format("%.4f", pSharedMemory->disp_qLH[0]);	m_encoderDlg.GetDlgItem(IDC_EDIT29)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_qLH[1]);	m_encoderDlg.GetDlgItem(IDC_EDIT30)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_qLH[2]);	m_encoderDlg.GetDlgItem(IDC_EDIT31)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_qLH[3]);	m_encoderDlg.GetDlgItem(IDC_EDIT32)->SetWindowText(strText);				

				strText.Format("%.4f", pSharedMemory->disp_pRH[0]);	m_encoderDlg.GetDlgItem(IDC_EDIT41)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_pRH[1]);	m_encoderDlg.GetDlgItem(IDC_EDIT43)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_pRH[2]);	m_encoderDlg.GetDlgItem(IDC_EDIT45)->SetWindowText(strText);				

				strText.Format("%.4f", pSharedMemory->disp_qRH[0]);	m_encoderDlg.GetDlgItem(IDC_EDIT46)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_qRH[1]);	m_encoderDlg.GetDlgItem(IDC_EDIT47)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_qRH[2]);	m_encoderDlg.GetDlgItem(IDC_EDIT48)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_qRH[3]);	m_encoderDlg.GetDlgItem(IDC_EDIT49)->SetWindowText(strText);
				
				strText.Format("%.4f", pSharedMemory->disp_pRF[0]);	m_encoderDlg.GetDlgItem(IDC_EDIT50)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_pRF[1]);	m_encoderDlg.GetDlgItem(IDC_EDIT51)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_pRF[2]);	m_encoderDlg.GetDlgItem(IDC_EDIT73)->SetWindowText(strText);				

				strText.Format("%.4f", pSharedMemory->disp_qRF[0]);	m_encoderDlg.GetDlgItem(IDC_EDIT74)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_qRF[1]);	m_encoderDlg.GetDlgItem(IDC_EDIT75)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_qRF[2]);	m_encoderDlg.GetDlgItem(IDC_EDIT76)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_qRF[3]);	m_encoderDlg.GetDlgItem(IDC_EDIT77)->SetWindowText(strText);

				strText.Format("%.4f", pSharedMemory->disp_pLF[0]);	m_encoderDlg.GetDlgItem(IDC_EDIT78)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_pLF[1]);	m_encoderDlg.GetDlgItem(IDC_EDIT79)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_pLF[2]);	m_encoderDlg.GetDlgItem(IDC_EDIT80)->SetWindowText(strText);				

				strText.Format("%.4f", pSharedMemory->disp_qLF[0]);	m_encoderDlg.GetDlgItem(IDC_EDIT81)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_qLF[1]);	m_encoderDlg.GetDlgItem(IDC_EDIT82)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_qLF[2]);	m_encoderDlg.GetDlgItem(IDC_EDIT83)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_qLF[3]);	m_encoderDlg.GetDlgItem(IDC_EDIT84)->SetWindowText(strText);

				strText.Format("%.4f", pSharedMemory->disp_pCOM[0]);	m_encoderDlg.GetDlgItem(IDC_EDIT85)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_pCOM[1]);	m_encoderDlg.GetDlgItem(IDC_EDIT86)->SetWindowText(strText);
				
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[1]);	m_encoderDlg.GetDlgItem(IDC_EDIT101)->SetWindowText(strText);
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[2]);	m_encoderDlg.GetDlgItem(IDC_EDIT102)->SetWindowText(strText);
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[3]);	m_encoderDlg.GetDlgItem(IDC_EDIT103)->SetWindowText(strText);

				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[21]*180.f/PI);	m_encoderDlg.GetDlgItem(IDC_EDIT87)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[22]*180.f/PI);	m_encoderDlg.GetDlgItem(IDC_EDIT88)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[23]*180.f/PI);	m_encoderDlg.GetDlgItem(IDC_EDIT89)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[24]*180.f/PI);	m_encoderDlg.GetDlgItem(IDC_EDIT90)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[25]*180.f/PI);	m_encoderDlg.GetDlgItem(IDC_EDIT91)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[26]*180.f/PI);	m_encoderDlg.GetDlgItem(IDC_EDIT92)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[33]*180.f/PI);	m_encoderDlg.GetDlgItem(IDC_EDIT93)->SetWindowText(strText);
				
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[27]*180.f/PI);	m_encoderDlg.GetDlgItem(IDC_EDIT94)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[28]*180.f/PI);	m_encoderDlg.GetDlgItem(IDC_EDIT95)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[29]*180.f/PI);	m_encoderDlg.GetDlgItem(IDC_EDIT96)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[30]*180.f/PI);	m_encoderDlg.GetDlgItem(IDC_EDIT97)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[31]*180.f/PI);	m_encoderDlg.GetDlgItem(IDC_EDIT98)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[32]*180.f/PI);	m_encoderDlg.GetDlgItem(IDC_EDIT99)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[34]*180.f/PI);	m_encoderDlg.GetDlgItem(IDC_EDIT100)->SetWindowText(strText);
				
				strText.Format("%.4f", ((float)pSharedMemory->Joint_debug[NKY].EncoderValue)*(-2883.f*360.f)); m_encoderDlg.GetDlgItem(IDC_EDIT104)->SetWindowText(strText);				
				strText.Format("%.4f", pSharedMemory->disp_Q_34x1[0]*180.f/PI);	m_encoderDlg.GetDlgItem(IDC_EDIT165)->SetWindowText(strText);
				
				break;
			default:
				break;
			}
		}
		break;
	}
	
	CDialog::OnTimer(nIDEvent);
}




int CRainbowGUIDlg::QT2RV(float qt_4x1[], float rv[])
{
	float temp;
	rv[0] = acosf(qt_4x1[1])*2.f;

	if(fabs(sinf(rv[0]/2.f)) < 1e-6)
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

		temp = sqrt(rv[1]*rv[1]+rv[2]*rv[2]+rv[3]*rv[3]);//norm_v(rv,3);
		rv[1] /= temp;
		rv[2] /= temp;
		rv[3] /= temp;
	}

	return 0;
}

int CRainbowGUIDlg::DC2QT(float DC_3x3[][4], float qt_4x1[])
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

int CRainbowGUIDlg::findmax(float data[], unsigned int n, float *result_max, unsigned int *i_max)
{
	unsigned int i, n_max;
	float max = -1.e9;

	for(i=0; i<n; i++)
	{
		if(max < data[i])
		{
			max = data[i];
			n_max = i;
		}				
	}

	*result_max = max;
	*i_max = n_max;

	return 0;
}