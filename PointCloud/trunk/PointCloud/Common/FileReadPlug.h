#ifndef FILEREADPLUG_H
#define FILEREADPLUG_H
// #define BOOST_FILESYSTEM_NO_LIB
// #define BOOST_SYSTEM_NO_LIB
#include <boost/filesystem.hpp>
/*#include <boost/filesystem/path.hpp>*/
/*#include <boost/filesystem/operations.hpp>*/
#include <iostream>
#include <vector>
#include <string>
#include <Windows.h>
namespace fs = boost::filesystem;
using namespace std;

#ifdef UNICODE
#define String std::wstring
#define Path fs::wpath
#else
#define String std::string
#define Path fs::path
#endif

inline void unicodeToUTF8(const wstring &src, string& result)
{
	int n = WideCharToMultiByte( CP_UTF8, 0, src.c_str(), -1, 0, 0, 0, 0 );
	result.resize(n);
	::WideCharToMultiByte( CP_UTF8, 0, src.c_str(), -1, (char*)result.c_str(), result.length(), 0, 0 );
}

inline void unicodeToGB2312(const wstring& wstr , string& result)
{
	int n = WideCharToMultiByte( CP_ACP, 0, wstr.c_str(), -1, 0, 0, 0, 0 );
	result.resize(n);
	::WideCharToMultiByte(CP_ACP, 0, wstr.c_str(), -1, (char*)result.c_str(), n, 0, 0 );
}

inline void utf8ToUnicode(const string& src, wstring& result)
{
	int n = MultiByteToWideChar( CP_UTF8, 0, src.c_str(), -1, NULL, 0 );
	result.resize(n);
	::MultiByteToWideChar( CP_UTF8, 0, src.c_str(), -1, (LPWSTR)result.c_str(), result.length());
}

inline void gb2312ToUnicode(const string& src, wstring& result)
{
	int n = MultiByteToWideChar( CP_ACP, 0, src.c_str(), -1, NULL, 0 );
	result.resize(n);
	::MultiByteToWideChar( CP_ACP, 0, src.c_str(), -1, (LPWSTR)result.c_str(), result.length());
}

struct Point3
{
	float p[3];	// �������
	float v[3];	// ��ķ�ʸ
};

//!+ �ļ���ȡ��������ַָ��
typedef size_t (*ReadFileFunc)(const String&, vector<Point3>&);
//!+ ע���ļ����ͺ���ָ��
//typedef void (*RegisterExt)(String& ext, String& funcName);
typedef size_t (*RegisterExt)(vector<String>& , vector<String>& );

// ÿ���ļ���ȡDll��ע����
struct ExtProcess{
	String m_sExt;			// �ļ���׺��(.txt)
	String m_sDllPath;	// ����m_sExt��Dll�ļ�·��(./plugin/*.dll)
	String m_sFuncName;	// �����ļ��ĺ�������
};

// ����ȫ�ֵ�ext������Ϣ
class ExtRegister{
public:
	vector<ExtProcess> m_extPros;	// ע����Ϣ��
public:
	size_t size() const { return m_extPros.size(); }
	void addExt(const ExtProcess& extPro){ m_extPros.push_back(extPro); }
	void addExt(const vector<ExtProcess>& extProVec) {
		m_extPros.insert(m_extPros.end(), extProVec.begin(), extProVec.end());
	}
	void addExt(vector<ExtProcess>& extProVec, bool bRemove = true){
		if (bRemove){ m_extPros.clear();}
		addExt(extProVec);
	}
	String extString() {
		String ext;
		for (vector<ExtProcess>::iterator it = m_extPros.begin();
				it != m_extPros.end(); ++it){
				ext +=it->m_sExt.substr(1)+_T("(*")+it->m_sExt+_T(")|*") + it->m_sExt+_T("|");
		}
		return ext;
	}

	//@ ��ȡ����ext�ļ����͵ĺ�����ַ
	ReadFileFunc findExtProFunc(const String& ext, HINSTANCE& hDll){
		ReadFileFunc readFunc = NULL;
		vector<ExtProcess>::iterator it = m_extPros.begin();
		for (; it != m_extPros.end(); ++it)
		{
			if (it->m_sExt == ext){
				HINSTANCE hExtDll = LoadLibrary(it->m_sDllPath.c_str());
				if (hExtDll != NULL){
#ifdef UNICODE
					string funcName;
					unicodeToGB2312(it->m_sFuncName, funcName);
					readFunc = (ReadFileFunc)GetProcAddress(hExtDll, funcName.c_str());
#else
					readFunc = (ReadFileFunc)GetProcAddress(hExtDll, it->m_sFuncName.c_str());
#endif // UNICODE
					hDll = hExtDll;
					break;
				}
			}
		}
		return readFunc;
	}

	//@ ��ȡ����ext�ļ����͵�ע����Ϣ
	//@ ext Ϊ�ļ���׺�� (.txt  .pts )
	const ExtProcess* findExtProcess(const String& ext) const {
		vector<ExtProcess>::const_iterator it = m_extPros.begin();
		for (; it != m_extPros.end(); ++it)
		{
			if (it->m_sExt == ext){
				return &(*it);
			}
		}
		return NULL;
	}

	// ����Ŀ¼sPath(./plugin/)�µ���չ��ΪsExt(.dll)�������ļ�
	size_t findFiles(const String& sPath, const String& sExt, vector<String>& filePaths){
		Path plugPath(sPath, fs::native);
		if (!fs::exists(plugPath)){
			return 0;
		}
		if (fs::is_directory(plugPath)){
			fs::directory_iterator end_iter;
			for (fs::directory_iterator dir_itr(plugPath); dir_itr != end_iter; ++dir_itr)
			{
				if (!fs::is_directory(*dir_itr) && dir_itr->path().extension() == sExt){
					String fileName(sPath);
					//fileName.append("\\");
					fileName.append(dir_itr->filename());
					filePaths.push_back(fileName);
				}
			}
			return filePaths.size();
		}
		return 0;
	}

	// ���ز���ļ����е� Dll ����ע��
	int loadExtDlls(const String& plugPath){
		vector<String> dllPaths;
		if (findFiles(plugPath, _T(".dll"), dllPaths) != 0){
			for (vector<String>::iterator it = dllPaths.begin(); it != dllPaths.end(); ++it)
			{
				ExtProcess extPro;
				extPro.m_sDllPath = *it;
				HINSTANCE hExtDll = LoadLibrary(it->c_str());
				if (hExtDll != NULL){
					RegisterExt registerExt = (RegisterExt)GetProcAddress(hExtDll, _T("RegisterExt"));
					if (registerExt != NULL){
						vector<String> extV, funcNameV;
						registerExt(extV, funcNameV);
						for (size_t i = 0; i < extV.size(); i ++)
						{
							extPro.m_sExt = extV.at(i);
							extPro.m_sFuncName = funcNameV.at(i);
							addExt(extPro);
						}
					}
				}
				FreeLibrary(hExtDll);
			}
		}
		return size();
	}
};

/* ========================================================
������ϸ˵���ļ���ȡDLL��д��������
	Ҫ������������������
	// �ֱ�Ϊ ��ȡ�ļ����ͺͶ�ȡ��������
	1. RegistExt(vector<String>& ext, vector<String>& funcName)
	// 1�еĶ�ȡ����
	2. funcName(const String& fileName, vector<Point3>& pointCloud)
======================================================== */
#endif // FILEREADPLUG_H