// 下列 ifdef 块是创建使从 DLL 导出更简单的
// 宏的标准方法。此 DLL 中的所有文件都是用命令行上定义的 TXTPLUGIN_EXPORTS
// 符号编译的。在使用此 DLL 的
// 任何其他项目上不应定义此符号。这样，源文件中包含此文件的任何其他项目都会将
// TXTPLUGIN_API 函数视为是从 DLL 导入的，而此 DLL 则将用此宏定义的
// 符号视为是被导出的。
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
	float p[3];	// 点的坐标
	float v[3];	// 点的法矢
};

TXTPLUGIN_API size_t RegisterExt(vector<String>& ext, vector<String>& funcName);

TXTPLUGIN_API size_t ReadTxtFile(const String& txtPath, vector<Point3>& pointVec);

TXTPLUGIN_API size_t ReadPtsFile(const String& txtPath, vector<Point3>& pointVec);