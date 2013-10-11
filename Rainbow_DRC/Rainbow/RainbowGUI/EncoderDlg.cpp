// EncoderDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RainbowGUI.h"
#include "EncoderDlg.h"
#include "..\SharedMemory.h"
#include "..\CommonDefinition.h"


extern PSHARED_DATA pSharedMemory;

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CEncoderDlg property page

IMPLEMENT_DYNCREATE(CEncoderDlg, CPropertyPage)

CEncoderDlg::CEncoderDlg() : CPropertyPage(CEncoderDlg::IDD)
{
	//{{AFX_DATA_INIT(CEncoderDlg)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}

CEncoderDlg::~CEncoderDlg()
{
}

void CEncoderDlg::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CEncoderDlg)
		// NOTE: the ClassWizard will add DDX and DDV calls here
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CEncoderDlg, CPropertyPage)
	//{{AFX_MSG_MAP(CEncoderDlg)
		// NOTE: the ClassWizard will add message map macros here
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CEncoderDlg message handlers
