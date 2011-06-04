#pragma once

// CSimplyParamDlg 对话框

class CSimplyParamDlg : public CDialog
{
	DECLARE_DYNAMIC(CSimplyParamDlg)

public:
	CSimplyParamDlg(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CSimplyParamDlg();

// 对话框数据
	enum { IDD = IDD_DLG_SIMPLY };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()

public:
	unsigned int m_K;	//+K 邻域点个数+

	unsigned int m_M;	// 单元网格中顶点的最大个数
	unsigned int m_Deep;// 精简时最小的网格深度
};