#pragma once

// CSetK �Ի���

class CSetK : public CDialog
{
	DECLARE_DYNAMIC(CSetK)
public:

public:
	CSetK(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CSetK();

// �Ի�������
	enum { IDD = IDD_DLG_SETK };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	int m_K;
	// ������������С�ڽ���ռ����
	float m_A;
	// �������ռ���أ���ƽ���ȣ�
	float m_B;
};