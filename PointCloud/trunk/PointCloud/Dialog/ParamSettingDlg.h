#pragma once

// CParamSettingDlg �Ի���

class CParamSettingDlg : public CDialog
{
	DECLARE_DYNAMIC(CParamSettingDlg)

public:
	CParamSettingDlg(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CParamSettingDlg();

// �Ի�������
	enum { IDD = IDD_DLG_PARAMSETTING };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	// ���Ƽ���������
	unsigned int m_simplyK;
	// ��Ԫ�����е��������
	unsigned int m_simplyMaxPointSize;
	// Ҫ����İ˲����������С���
	unsigned int m_simplyMinDeep;

	// ���ཻ��������
	float m_ballTerr;
	// ���ཻ������뾶Ӱ�����ֵ
	float m_ballTq;

	// ������չ��������ʱ���������С
	unsigned int m_pointMeshK;
	// ������չ��������ʱ��������С�ڽ����ڱ���
	float m_pointMeshA;
	// ������չ��������ʱ�������ռ����(��ƽ����)
	float m_pointMeshB;
	// ʹ��ȫ������
	bool m_bUseGlobalSetting;
};