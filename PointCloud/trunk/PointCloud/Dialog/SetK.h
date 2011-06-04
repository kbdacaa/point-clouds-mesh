#pragma once

// CSetK 对话框

class CSetK : public CDialog
{
	DECLARE_DYNAMIC(CSetK)
public:

public:
	CSetK(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CSetK();

// 对话框数据
	enum { IDD = IDD_DLG_SETK };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	int m_K;
	// 控制三角形最小内角所占比重
	float m_A;
	// 二面角所占比重（面平滑度）
	float m_B;
};