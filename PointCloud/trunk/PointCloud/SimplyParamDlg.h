#pragma once


// CSimplyParamDlg �Ի���

class CSimplyParamDlg : public CDialog
{
	DECLARE_DYNAMIC(CSimplyParamDlg)

public:
	CSimplyParamDlg(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CSimplyParamDlg();

// �Ի�������
	enum { IDD = IDD_DLG_SIMPLY };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()

public:
	int m_K;	//+K ��������+

};