#pragma once

// CParamSettingDlg 对话框

class CParamSettingDlg : public CDialog
{
	DECLARE_DYNAMIC(CParamSettingDlg)

public:
	CParamSettingDlg(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CParamSettingDlg();

// 对话框数据
	enum { IDD = IDD_DLG_PARAMSETTING };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	// 点云简化邻域点个数
	unsigned int m_simplyK;
	// 单元网格中点的最大个数
	unsigned int m_simplyMaxPointSize;
	// 要精简的八叉树网格的最小深度
	unsigned int m_simplyMinDeep;

	// 球相交法的误差精度
	float m_ballTerr;
	// 球相交法的球半径影响域比值
	float m_ballTq;

	// 顶点扩展网格生成时搜索邻域大小
	unsigned int m_pointMeshK;
	// 顶点扩展网格生成时三角形最小内角所在比重
	float m_pointMeshA;
	// 顶点扩展网格生成时二面角所占比重(面平滑度)
	float m_pointMeshB;
	// 使用全局设置
	bool m_bUseGlobalSetting;
};