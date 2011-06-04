// Dialog/ParamSettingDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "../PointCloud.h"
#include "ParamSettingDlg.h"
#include "afxdialogex.h"

// CParamSettingDlg 对话框

IMPLEMENT_DYNAMIC(CParamSettingDlg, CDialog)

CParamSettingDlg::CParamSettingDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CParamSettingDlg::IDD, pParent)
	, m_simplyK(10)
	, m_simplyMaxPointSize(15)
	, m_simplyMinDeep(4)
	, m_ballTerr(4.0e-6f)
	, m_ballTq(2.0f)
	, m_pointMeshK(12)
	, m_pointMeshA(1.0f)
	, m_pointMeshB(1.0f)
	, m_bUseGlobalSetting(true)
{
}

CParamSettingDlg::~CParamSettingDlg()
{
}

void CParamSettingDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT_SIMPLYK, m_simplyK);
	DDX_Text(pDX, IDC_EDIT_SIMPLYM, m_simplyMaxPointSize);
	DDX_Text(pDX, IDC_EDIT_SIMPLYDEEP, m_simplyMinDeep);
	DDX_Text(pDX, IDC_EDIT_BALLTERR, m_ballTerr);
	DDX_Text(pDX, IDC_EDIT_BALLTQ, m_ballTq);
	DDX_Text(pDX, IDC_EDIT_POINTMESHK, m_pointMeshK);
	DDX_Text(pDX, IDC_EDIT_POINTMESHA, m_pointMeshA);
	DDX_Text(pDX, IDC_EDIT_POINTMESHB, m_pointMeshB);
}

BEGIN_MESSAGE_MAP(CParamSettingDlg, CDialog)
END_MESSAGE_MAP()

// CParamSettingDlg 消息处理程序