#if !defined(AFX_DRCTESTDLG_H__8E7EF4BE_D054_4CF2_9F6F_66C0469A6F62__INCLUDED_)
#define AFX_DRCTESTDLG_H__8E7EF4BE_D054_4CF2_9F6F_66C0469A6F62__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DRCtestDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CDRCtestDlg dialog

class CDRCtestDlg : public CDialog
{
// Construction
public:
	CDRCtestDlg(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CDRCtestDlg)
	enum { IDD = IDD_DRC_TEST_DIALOG };
	BOOL	m_check2;
	BOOL	m_check3;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CDRCtestDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CDRCtestDlg)
	afx_msg void OnDrcHome();
	afx_msg void OnDrcReady();
	afx_msg void OnDrcStop();
	afx_msg void OnDrcStart();
	afx_msg void OnDrcFreqReady();
	afx_msg void OnDrcFreqStart();
	virtual BOOL OnInitDialog();
	afx_msg void OnDrcWalkReady();
	afx_msg void OnDrcHome2();
	afx_msg void OnDrcWr();
	afx_msg void OnDrcReady2();
	afx_msg void OnDrcUpdate();
	afx_msg void OnDrcComple();
	afx_msg void OnDrcComple2();
	afx_msg void OnDrcSr();
	afx_msg void OnDrcSteer();
	afx_msg void OnDrcGrab();
	afx_msg void OnDrcRelease();
	afx_msg void OnDrcGrab2();
	afx_msg void OnDrcGrab3();
	afx_msg void OnDrcInitHand();
	afx_msg void OnDrcZerosteer();
	afx_msg void OnDrcSr2();
	afx_msg void OnDrcLad();
	afx_msg void OnDrcLad2();
	afx_msg void OnDrcLad3();
	afx_msg void OnDrcStop2();
	afx_msg void OnDrcLad4();
	afx_msg void OnDrcLad5();
	afx_msg void OnDrcLad6();
	afx_msg void OnDrcLad7();
	afx_msg void OnDrcLad8();
	afx_msg void OnDrcLad9();
	afx_msg void OnDrcGrab4();
	afx_msg void OnDrcGrab5();
	afx_msg void OnDrcGrab6();
	afx_msg void OnDrcInitHand2();
	afx_msg void OnDrcLad10();
	afx_msg void OnDrcLad11();
	afx_msg void OnDrcLad12();
	afx_msg void OnDrcLad13();
	afx_msg void OnDrcLad14();
	afx_msg void OnDrcSr3();
	afx_msg void OnDrcFreqStart2();
	afx_msg void OnDrcGrab7();
	afx_msg void OnDrcGrab8();
	afx_msg void OnDrcGrab9();
	afx_msg void OnDrcGrab10();
	afx_msg void OnDrcGrab11();
	afx_msg void OnDrcGrab12();
	afx_msg void OnDrcNeck1();
	afx_msg void OnDrcNeck2();
	afx_msg void OnDrcNeck3();
	afx_msg void OnDrcNeck4();
	afx_msg void OnDrcStart2();
	afx_msg void OnDrcHandMode();
	afx_msg void OnDrcStickMode();
	afx_msg void OnDrcStart3();
	afx_msg void OnDrcLad15();
	afx_msg void OnDrcHandYaw();
	afx_msg void OnDrcHandYaw2();
	afx_msg void OnDrcGainovr();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DRCTESTDLG_H__8E7EF4BE_D054_4CF2_9F6F_66C0469A6F62__INCLUDED_)
