#if !defined(AFX_DRCQUADDLG_H__25495F37_06A0_4E2D_BDCA_74E7AEBFBA52__INCLUDED_)
#define AFX_DRCQUADDLG_H__25495F37_06A0_4E2D_BDCA_74E7AEBFBA52__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DrcQuadDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CDrcQuadDlg dialog

class CDrcQuadDlg : public CDialog
{
// Construction
public:
	CDrcQuadDlg(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CDrcQuadDlg)
	enum { IDD = IDD_DRC_QUAD_DIALOG };
	double	m_LS_KIRK;
	double	m_LSS_KIRK;
	double	m_ROT_ANG_KIRK;
	double	m_DELAY_R_KIRK;
	double	m_AB_KIRK;
	double	m_HS_KIRK;
	UINT	m_STEP_NUM_KIRK;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CDrcQuadDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CDrcQuadDlg)
	afx_msg void OnReadyBiped();
	afx_msg void OnHome();
	afx_msg void OnTransform();
	afx_msg void OnKirkStart();
	afx_msg void OnTransformBack();
	afx_msg void OnKirkStop();
	afx_msg void OnStop();
	afx_msg void OnHandYaw0Deg();
	afx_msg void OnHandYaw90Deg();
	afx_msg void OnHandMode();
	afx_msg void OnStickMode();
	afx_msg void OnStepGoKirk();
	afx_msg void OnContinuGoKirk();
	afx_msg void OnStopKirk();
	afx_msg void OnParaChange();
	afx_msg void OnKirkZmpTest();
	afx_msg void OnKirkZmpConOn();
	afx_msg void OnKirkZmpConOff();
	afx_msg void OnKirkZmpInitStart();
	afx_msg void OnKirkZmpInitStop();
	afx_msg void OnKirkGoHome();
	virtual BOOL OnInitDialog();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DRCQUADDLG_H__25495F37_06A0_4E2D_BDCA_74E7AEBFBA52__INCLUDED_)
