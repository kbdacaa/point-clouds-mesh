// ���� ifdef ���Ǵ���ʹ�� DLL �������򵥵�
// ��ı�׼�������� DLL �е������ļ��������������϶���� TXTPLUGIN_EXPORTS
// ���ű���ġ���ʹ�ô� DLL ��
// �κ�������Ŀ�ϲ�Ӧ����˷��š�������Դ�ļ��а������ļ����κ�������Ŀ���Ὣ
// TXTPLUGIN_API ������Ϊ�Ǵ� DLL ����ģ����� DLL ���ô˺궨���
// ������Ϊ�Ǳ������ġ�
#ifdef TXTPLUGIN_EXPORTS
#define TXTPLUGIN_API __declspec(dllexport)
#else
#define TXTPLUGIN_API __declspec(dllimport)
#endif

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#ifdef UNICODE
#define String std::wstring
#define Ifstream std::wifstream
#else
#define String std::string
#define Ifstream std::ifstream
#endif // UNICODE

struct Point3
{
	float p[3];	// �������
	float v[3];	// ��ķ�ʸ
};

TXTPLUGIN_API size_t RegisterExt(vector<String>& ext, vector<String>& funcName);

TXTPLUGIN_API size_t ReadTxtFile(const String& txtPath, vector<Point3>& pointVec);

TXTPLUGIN_API size_t ReadPtsFile(const String& txtPath, vector<Point3>& pointVec);