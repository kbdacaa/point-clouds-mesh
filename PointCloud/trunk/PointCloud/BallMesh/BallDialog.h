#if !defined(AFX_BALLDIALOG_H_)
#define AFX_BALLDIALOG_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

// CBallDialog 对话框

class CBallDialog : public CDialog
{
	DECLARE_DYNAMIC(CBallDialog)

public:
	CBallDialog(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CBallDialog();

// 对话框数据
	enum { IDD = IDD_DLG_BALLMESHSET };
	float	m_err;	//+误差 默认=1e-005+
	float	m_qem;	//+默认=2+

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
};


#endif // !defined(AFX_BALLDIALOG_H_)
#pragma once


