// SimplyParamDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "PointCloud.h"
#include "SimplyParamDlg.h"


// CSimplyParamDlg 对话框

IMPLEMENT_DYNAMIC(CSimplyParamDlg, CDialog)

CSimplyParamDlg::CSimplyParamDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CSimplyParamDlg::IDD, pParent)
{
	m_K = 15;
}

CSimplyParamDlg::~CSimplyParamDlg()
{
}

void CSimplyParamDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CSimplyParamDlg, CDialog)
END_MESSAGE_MAP()


// CSimplyParamDlg 消息处理程序
