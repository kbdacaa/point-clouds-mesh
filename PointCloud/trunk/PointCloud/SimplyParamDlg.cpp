// SimplyParamDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "PointCloud.h"
#include "SimplyParamDlg.h"


// CSimplyParamDlg �Ի���

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


// CSimplyParamDlg ��Ϣ�������
