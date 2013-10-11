#if !defined(AFX_ENCODERDLG_H__04A5742B_0EC1_4EF5_A228_789DA1C3DBFA__INCLUDED_)
#define AFX_ENCODERDLG_H__04A5742B_0EC1_4EF5_A228_789DA1C3DBFA__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// EncoderDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CEncoderDlg dialog

class CEncoderDlg : public CPropertyPage
{
	DECLARE_DYNCREATE(CEncoderDlg)

// Construction
public:
	CEncoderDlg();
	~CEncoderDlg();

// Dialog Data
	//{{AFX_DATA(CEncoderDlg)
	enum { IDD = IDD_ENCODER_DIALOG };
		// NOTE - ClassWizard will add data members here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_DATA


// Overrides
	// ClassWizard generate virtual function overrides
	//{{AFX_VIRTUAL(CEncoderDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	// Generated message map functions
	//{{AFX_MSG(CEncoderDlg)
		// NOTE: the ClassWizard will add member functions here
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_ENCODERDLG_H__04A5742B_0EC1_4EF5_A228_789DA1C3DBFA__INCLUDED_)
