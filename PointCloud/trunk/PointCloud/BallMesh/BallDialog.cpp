#include "stdafx.h"
#include "../PointCloud.h"
#include "BallDialog.h"


// CBallDialog 对话框

IMPLEMENT_DYNAMIC(CBallDialog, CDialog)

CBallDialog::CBallDialog(CWnd* pParent /*=NULL*/)
	: CDialog(CBallDialog::IDD, pParent)
{
	m_err = 0.00001f;
	m_qem = 2.0f;
}

CBallDialog::~CBallDialog()
{
}

void CBallDialog::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT1, m_err);
	DDX_Text(pDX, IDC_EDIT2, m_qem);
}


BEGIN_MESSAGE_MAP(CBallDialog, CDialog)
END_MESSAGE_MAP()


// CBallDialog 消息处理程序
