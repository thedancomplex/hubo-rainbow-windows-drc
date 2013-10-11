#include <stdio.h>
#include "DemoTestDlg.h"
#include "DemoTest2Dlg.h"
#include "DRCtestDlg.h"
#include "DrcQuadDlg.h"
#include "BipedDlg.h"

#if !defined(AFX_WALKINGDLG_H__DD710B96_3A7D_4C41_8D26_62B040E44C4A__INCLUDED_)
#define AFX_WALKINGDLG_H__DD710B96_3A7D_4C41_8D26_62B040E44C4A__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// WalkingDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CWalkingDlg dialog

class CWalkingDlg : public CPropertyPage
{
	DECLARE_DYNCREATE(CWalkingDlg)

// Construction
public:
	CDemoTestDlg	m_DemoTestDlg;
	CDemoTest2Dlg	m_DemoTest2Dlg;
	CDRCtestDlg		m_DRCtestDlg;
	CDrcQuadDlg		m_DrcQuadDlg;
	CBipedDlg		m_DrcBipedDlg;
		
	CWalkingDlg();
	~CWalkingDlg();

	void LogRingHandler(void); //by Inhyeok
	FILE *m_pFile;  // by Inhyeok

// Dialog Data
	//{{AFX_DATA(CWalkingDlg)
	enum { IDD = IDD_WALKING_DIALOG };
	CComboBox	m_handSelBox;
	CComboBox	m_motionSelBox;
	CComboBox	m_demoSelBox;	
	float	m_swayValue;
	float	m_stopSway;
	int		m_stepCount;
	float	m_startSway;
	float	m_sideStep;
	float	m_rotationStep;
	UINT	m_stepPeriod;
	UINT	m_dspTime;
	UINT	m_holdTime;
	float	m_forwardStep;
	float	m_greb;
	float	m_grsr;
	float	m_grwy;
	float	m_mx;
	float	m_my;
	int		m_rlebgain;
	int		m_rlshpgain;
	int		m_RADIO1;
	int		m_landingstate_fz;
	//}}AFX_DATA


// Overrides
	// ClassWizard generate virtual function overrides
	//{{AFX_VIRTUAL(CWalkingDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	// Generated message map functions
	//{{AFX_MSG(CWalkingDlg)
	afx_msg void OnAnkleonoff();
	afx_msg void OnZmpinit();
	afx_msg void OnWalkready();
	afx_msg void OnWalkingset();
	afx_msg void OnGo();
	afx_msg void OnPlusrotation();
	afx_msg void OnMinusrotation();
	afx_msg void OnZerorotation();
	afx_msg void OnStop();
	virtual BOOL OnInitDialog();
	afx_msg void OnMotion();
	afx_msg void OnSelchangeDemolist();
	afx_msg void OnDemo1();
	afx_msg void OnDemo2();
	afx_msg void OnDemo3();
	afx_msg void OnDemo4();
	afx_msg void OnDatasave();
	afx_msg void OnSelchangeHandlist();
	afx_msg void OnHand1();
	afx_msg void OnHand2();
	afx_msg void OnHand3();
	afx_msg void OnHand4();
	afx_msg void OnRangeonoff();
	afx_msg void OnPluscurrent();
	afx_msg void OnMinuscurrent();
	afx_msg void OnInithands();
	afx_msg void OnZmpinitpos();
	afx_msg void OnZmppossave();
	afx_msg void OnUneven();
	afx_msg void OnFoward20cm();
	afx_msg void OnFoward15cm();
	afx_msg void OnFoward10cm();
	afx_msg void OnFoward0cm();
	afx_msg void OnFowardm10cm();
	afx_msg void OnPlusrotation2();
	afx_msg void OnMinusrotation2();
	afx_msg void OnPlusrotation3();
	afx_msg void OnMinusrotation3();
	afx_msg void OnButton1();
	afx_msg void OnButton2();
	afx_msg void OnButton3();
	afx_msg void OnButton4();
	afx_msg void OnButton5();
	afx_msg void OnDemotest();
	afx_msg void OnDemotest2();
	afx_msg void OnLog();
	afx_msg void OnTimer(UINT nIDEvent);
	afx_msg void OnDrc();
	afx_msg void OnKirk();
	afx_msg void OnKirk2();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_WALKINGDLG_H__DD710B96_3A7D_4C41_8D26_62B040E44C4A__INCLUDED_)
