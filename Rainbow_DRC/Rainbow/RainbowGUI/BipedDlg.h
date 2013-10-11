#if !defined(AFX_BIPEDDLG_H__7A622E71_211F_4332_89E2_BDD15A6D5BF8__INCLUDED_)
#define AFX_BIPEDDLG_H__7A622E71_211F_4332_89E2_BDD15A6D5BF8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// BipedDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CBipedDlg dialog

class CBipedDlg : public CDialog
{
// Construction
public:
	CBipedDlg(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CBipedDlg)
	enum { IDD = IDD_DRC_BIPED_DIALOG };
	float	m_LS_KIRK;
	float	m_LSS_KIRK;
	float	m_HS_KIRK;
	float	m_ROT_ANG_KIRK;
	float	m_DELAY_R_KIRK;
	float	m_AB_KIRK;
	UINT	m_STEP_NUM_KIRK;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CBipedDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CBipedDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnHome2();
	afx_msg void OnBipedReady();
	afx_msg void OnIkOn();
	afx_msg void OnKirkStart2();
	afx_msg void OnKirkStop2();
	afx_msg void OnEmStop();
	afx_msg void OnZmpConTest();
	afx_msg void OnContestOn();
	afx_msg void OnContestOff();
	afx_msg void OnKirkZmpInitStart();
	afx_msg void OnKirkZmpInitStop();
	afx_msg void OnKirkGohome();
	afx_msg void OnKirkStepNumberGo();
	afx_msg void OnKirkContinu();
	afx_msg void OnKirkStop3();
	afx_msg void OnKirkPara();
	afx_msg void OnKirkZmpInitLoad();
	afx_msg void OnKirkZmpInitSave();
	afx_msg void OnZmp();
	afx_msg void OnVib();
	afx_msg void OnEl();
	afx_msg void OnUpright();
	afx_msg void OnArmCompl();
	afx_msg void OnStride0();
	afx_msg void OnStride1();
	afx_msg void OnStride2();
	afx_msg void OnStride3();
	afx_msg void OnStride4();
	afx_msg void OnStride5();
	afx_msg void OnStride6();
	afx_msg void OnRot0();
	afx_msg void OnRot4();
	afx_msg void OnRot5();
	afx_msg void OnRot6();
	afx_msg void OnRot1();
	afx_msg void OnRot2();
	afx_msg void OnRot3();
	afx_msg void OnSide0();
	afx_msg void OnSide3();
	afx_msg void OnSide4();
	afx_msg void OnSide1();
	afx_msg void OnSide2();
	afx_msg void OnRot7();
	afx_msg void OnRot8();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_BIPEDDLG_H__7A622E71_211F_4332_89E2_BDD15A6D5BF8__INCLUDED_)
