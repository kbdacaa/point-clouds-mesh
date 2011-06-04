// SimplyParamDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "PointCloud.h"
#include "SimplyParamDlg.h"

// CSimplyParamDlg 对话框

IMPLEMENT_DYNAMIC(CSimplyParamDlg, CDialog)

CSimplyParamDlg::CSimplyParamDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CSimplyParamDlg::IDD, pParent)
	, m_K(10)
	, m_M(15)
	, m_Deep(4)
{
}

CSimplyParamDlg::~CSimplyParamDlg()
{
}

void CSimplyParamDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT_K, m_K);
	DDX_Text(pDX, IDC_EDIT_M, m_M);
	DDX_Text(pDX, IDC_EDIT_DEEP, m_Deep);
}

BEGIN_MESSAGE_MAP(CSimplyParamDlg, CDialog)
END_MESSAGE_MAP()