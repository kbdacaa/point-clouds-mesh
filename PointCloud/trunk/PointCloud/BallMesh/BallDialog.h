#if !defined(AFX_BALLDIALOG_H_)
#define AFX_BALLDIALOG_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

// CBallDialog �Ի���

class CBallDialog : public CDialog
{
	DECLARE_DYNAMIC(CBallDialog)

public:
	CBallDialog(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CBallDialog();

// �Ի�������
	enum { IDD = IDD_DLG_BALLMESHSET };
	float	m_err;	//+��� Ĭ��=1e-005+
	float	m_qem;	//+Ĭ��=2+

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
};


#endif // !defined(AFX_BALLDIALOG_H_)
#pragma once


