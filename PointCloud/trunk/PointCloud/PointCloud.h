// PointCloud.h : PointCloud Ӧ�ó������ͷ�ļ�
//
#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"       // ������


// CPointCloudApp:
// �йش����ʵ�֣������ PointCloud.cpp
//

class CPointCloudApp : public CWinApp
{
public:
	CPointCloudApp();


// ��д
public:
	virtual BOOL InitInstance();

// ʵ��
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
};

extern CPointCloudApp theApp;