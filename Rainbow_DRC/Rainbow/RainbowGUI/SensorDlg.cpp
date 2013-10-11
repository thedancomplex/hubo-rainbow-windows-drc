// SensorDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "SensorDlg.h"
#include "..\SharedMemory.h"
#include "..\CommonDefinition.h"

// SharedMemory variable
extern PSHARED_DATA pSharedMemory;

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CSensorDlg property page

IMPLEMENT_DYNCREATE(CSensorDlg, CPropertyPage)

CSensorDlg::CSensorDlg() : CPropertyPage(CSensorDlg::IDD)
{
	//{{AFX_DATA_INIT(CSensorDlg)
	//}}AFX_DATA_INIT
}

CSensorDlg::~CSensorDlg()
{
}

void CSensorDlg::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CSensorDlg)
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CSensorDlg, CPropertyPage)
	//{{AFX_MSG_MAP(CSensorDlg)
	ON_BN_CLICKED(IDC_FTNULLING, OnFtnulling)
	ON_BN_CLICKED(IDC_FOOTANGLENULLING, OnFootanglenulling)
	ON_BN_CLICKED(IDC_IMUNULLING, OnImunulling)
	ON_BN_CLICKED(IDC_SETOFFSET, OnSetoffset)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CSensorDlg message handlers

void CSensorDlg::OnFtnulling() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = NULL_FT_SENSOR;
		Sleep(10);
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CSensorDlg::OnFootanglenulling() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = NULL_FOOT_ANGLE_SENSOR;
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CSensorDlg::OnImunulling() 
{
	// TODO: Add your control notification handler code here
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		pSharedMemory->CommandFlag = NULL_IMU_SENSOR;
		Sleep(10);
	}
	else AfxMessageBox("Other Command is activated..!!");
}

void CSensorDlg::OnSetoffset() 
{
	// TODO: Add your control notification handler code here
	
	FILE *fp;	
	CString strText;
	if(pSharedMemory->CommandFlag == NO_ACT)
	{
		GetDlgItem(IDC_ROLL_OFFSET)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[0] = (float)atof(strText);
		GetDlgItem(IDC_PITCH_OFFSET)->GetWindowText(strText);
		pSharedMemory->CommandDataFloat[1] = (float)atof(strText);
		pSharedMemory->CommandFlag = SET_IMU_OFFSET;
		Sleep(10);
		fp = fopen("C:\\Rainbow_DRC\\WB_Data\\imu_offset.txt","w");
		fprintf(fp,"%f %f", pSharedMemory->CommandDataFloat[0], pSharedMemory->CommandDataFloat[1]);
		fclose(fp);
	}
	else AfxMessageBox("Other Command is activated..!!");
}

BOOL CSensorDlg::OnInitDialog() 
{
	CPropertyPage::OnInitDialog();
	
	FILE *fp;
	CString str_temp;
	float temp1,temp2;

	fp = fopen("C:\\Rainbow_DRC\\WB_Data\\imu_offset.txt","r");
	
	if(fp == NULL)
	{
		AfxMessageBox("C:\\Rainbow_DRC\\WB_Data\\imu_offset.txt was not found!!");
		temp1 = 0.f;
		temp2 = 0.f;
	}
	else
	{
		fscanf(fp,"%f %f",&temp1,&temp2);
		fclose(fp);
	}

	str_temp.Format("%f", temp1);
	GetDlgItem(IDC_ROLL_OFFSET)->SetWindowText(str_temp);
	str_temp.Format("%f", temp2);
	GetDlgItem(IDC_PITCH_OFFSET)->SetWindowText(str_temp);

	pSharedMemory->CommandDataFloat[0] = temp1;
	pSharedMemory->CommandDataFloat[1] = temp2;
	pSharedMemory->CommandFlag = SET_IMU_OFFSET;
	
	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}
