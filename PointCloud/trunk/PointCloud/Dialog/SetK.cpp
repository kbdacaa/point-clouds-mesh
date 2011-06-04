// Dialog/SetK.cpp : 实现文件
//

#include "stdafx.h"
#include "../PointCloud.h"
#include "SetK.h"
#include "afxdialogex.h"

// CSetK 对话框

IMPLEMENT_DYNAMIC(CSetK, CDialog)

CSetK::CSetK(CWnd* pParent /*=NULL*/)
	: CDialog(CSetK::IDD, pParent)
	, m_K(10)
	, m_A(1.0)
	, m_B(1.0)
{
}

CSetK::~CSetK()
{
}

void CSetK::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT1, m_K);
	DDX_Text(pDX, IDC_EDIT2, m_A);
	DDX_Text(pDX, IDC_EDIT3, m_B);
}

BEGIN_MESSAGE_MAP(CSetK, CDialog)
END_MESSAGE_MAP()

// CSetK 消息处理程序