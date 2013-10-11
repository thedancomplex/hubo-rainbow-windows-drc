// RainbowGUIDlg.h : header file
//

#if !defined(AFX_RAINBOWGUIDLG_H__B6245DBF_8E8B_4BBA_B328_E1D36116863C__INCLUDED_)
#define AFX_RAINBOWGUIDLG_H__B6245DBF_8E8B_4BBA_B328_E1D36116863C__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <rtapi.h>
#include "InitHUBO2Dlg.h"
#include "WalkingDlg.h"
#include "SensorDlg.h"
#include "PresetDlg.h"
#include "JointsDlg.h"
#include "JMCDlg.h"
#include "LStatusDlg.h"
#include "UStatusDlg.h"
#include "UserDlg.h"
#include "EncoderDlg.h"
#include "..\SharedMemory.h"
#include "..\CommonDefinition.h"

#include "CommThread.h"

/////////////////////////////////////////////////////////////////////////////
// CRainbowGUIDlg dialog

class CRainbowGUIDlg : public CDialog
{
// Construction
public:
	bool RTXOn(void);
	bool RTXOff(void);
	// SharedMemory handler
	HANDLE hSharedMemory;
	bool RTXStarted;

	CRainbowGUIDlg(CWnd* pParent = NULL);	// standard constructor

	int CRainbowGUIDlg::QT2RV(float qt_4x1[], float rv[]);
	int CRainbowGUIDlg::DC2QT(float DC_3x3[][4], float qt_4x1[]);
	int findmax(float data[], unsigned int n, float *result_max, unsigned int *i_max);

// Dialog Data
	//{{AFX_DATA(CRainbowGUIDlg)
	enum { IDD = IDD_RAINBOWGUI_DIALOG };
		// NOTE: the ClassWizard will add data members here
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CRainbowGUIDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

	CPropertySheet	m_propertySheet;
	CInitHUBO2Dlg	m_initHUBO2Dlg;
	CWalkingDlg		m_walkingDlg;
	CSensorDlg		m_setSensorDlg;
	CPresetDlg		m_presetDlg;
	CJointsDlg		m_setJointDlg;
	CJMCDlg			m_jmcSettingDlg;
	CLStatusDlg		m_lStatusDlg;
	CUStatusDlg		m_uStatusDlg;
	CUserDlg		m_userDlg;
	CEncoderDlg		m_encoderDlg;

	// Serial
	LONG OnReceive(UINT port,LONG lParam);
	CCommThread		m_Comm;	

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	//{{AFX_MSG(CRainbowGUIDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnExit();
	afx_msg void OnRtxonoff();
	afx_msg void OnLoadparam();
	afx_msg void OnCheckdevice();
	afx_msg void OpenComPort();
	afx_msg void OnControlonoff();
	afx_msg void OnTimer(UINT nIDEvent);
	afx_msg void ParsingRxData(CString strRx);
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_RAINBOWGUIDLG_H__B6245DBF_8E8B_4BBA_B328_E1D36116863C__INCLUDED_)
