#if !defined(AFX_DEMOTESTDLG_H__AF568A4A_91FC_44EB_A80C_39FC140E9AF9__INCLUDED_)
#define AFX_DEMOTESTDLG_H__AF568A4A_91FC_44EB_A80C_39FC140E9AF9__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DemoTestDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CDemoTestDlg dialog

class CDemoTestDlg : public CDialog
{
// Construction
public:
	CDemoTestDlg(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CDemoTestDlg)
	enum { IDD = IDD_DEMOTEST_DIALOG };
	int		m_trackingmode;
	int		m_rps_mode;
	BOOL	m_imu_feedback;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CDemoTestDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CDemoTestDlg)
	afx_msg void OnScene1();
	afx_msg void OnScene2();
	afx_msg void OnScene3();
	afx_msg void OnScene4();
	afx_msg void OnScene5();
	afx_msg void OnScene6();
	afx_msg void OnScene7();
	afx_msg void OnScene8();
	afx_msg void OnScene9();
	afx_msg void OnScene10();
	afx_msg void OnScene11();
	afx_msg void OnScene12();
	afx_msg void OnScene13();
	afx_msg void OnScene14();
	afx_msg void OnScene15();
	afx_msg void OnScene16();
	afx_msg void OnScene17();
	afx_msg void OnScene18();
	afx_msg void OnScene19();
	afx_msg void OnScene22();
	afx_msg void OnScene23();
	afx_msg void OnScene24();
	afx_msg void OnScene25();
	afx_msg void OnScene26();
	afx_msg void OnScene28();
	afx_msg void OnScene30();
	afx_msg void OnScene32();
	afx_msg void OnScene27GraspPos();
	afx_msg void OnScene27GraspOnoff();
	afx_msg void OnScene27Show();
	afx_msg void OnScene27Home();
	afx_msg void OnWalk1();
	afx_msg void OnScene35();
	afx_msg void OnScene36();
	afx_msg void OnScene37();
	afx_msg void OnScene38();
	afx_msg void OnScene39();
	afx_msg void OnScene40();
	afx_msg void OnScene41();
	afx_msg void OnWalk2();
	afx_msg void OnWalk3();
	afx_msg void OnWalk4();
	afx_msg void OnWalk5();
	afx_msg void OnWalk6();
	afx_msg void OnScene20();
	afx_msg void OnScene21();
	afx_msg void OnScene21Onoff();
	afx_msg void OnStartRHTracking();
	afx_msg void OnHandshake1();
	afx_msg void OnHandshake2();
	afx_msg void OnGo2SSP();
	afx_msg void OnUpdownSSP();
	afx_msg void OnBack2DSP();
	afx_msg void OnWbDance();
	afx_msg void OnHandInit();
	afx_msg void OnWalk7();
	afx_msg void OnButtonAble();
	virtual BOOL OnInitDialog();
	afx_msg void OnDrawItem(int nIDCtl, LPDRAWITEMSTRUCT lpDrawItemStruct);
	afx_msg void OnWalk8();
	afx_msg void OnVoice();
	afx_msg void OnScene27();
	afx_msg void OnRps();
	afx_msg void OnRps2();
	afx_msg void OnRps3();
	afx_msg void OnScene33();
	afx_msg void OnEmStop();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DEMOTESTDLG_H__AF568A4A_91FC_44EB_A80C_39FC140E9AF9__INCLUDED_)
