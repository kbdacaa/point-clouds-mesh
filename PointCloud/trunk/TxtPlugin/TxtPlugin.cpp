// TxtPlugin.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include "TxtPlugin.h"
#include <tchar.h>

TXTPLUGIN_API size_t RegisterExt( vector<String>& ext, vector<String>& funcName )
{
	ext.push_back(String(_T(".txt")));
	funcName.push_back(String(_T("ReadTxtFile")));
	ext.push_back(String(_T(".pts")));
	funcName.push_back(String(_T("ReadPtsFile")));
	return 2;
}

TXTPLUGIN_API size_t ReadTxtFile( const String& txtPath, vector<Point3>& pointVec )
{
	Ifstream inTxtFile(txtPath.c_str());
	if (!inTxtFile.fail()){
		Point3 pt;
		while (inTxtFile>>pt.p[0])
		{
			inTxtFile>>pt.p[1]>>pt.p[2];
			pointVec.push_back(pt);
		}
		inTxtFile.close();
		return pointVec.size();
	}
	return 0;
}

TXTPLUGIN_API size_t ReadPtsFile( const String& txtPath, vector<Point3>& pointVec )
{
	Ifstream inTxtFile(txtPath.c_str());
	if (!inTxtFile.fail()){
		size_t ptN;
		inTxtFile>>ptN;
		pointVec.resize(ptN);
		Point3 pt;
		while (inTxtFile>>pt.p[0])
		{
			inTxtFile>>pt.p[1]>>pt.p[2];
			pointVec.push_back(pt);
		}
		inTxtFile.close();
		return pointVec.size();
	}
	return 0;
}