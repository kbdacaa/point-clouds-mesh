#ifndef POINT_MESH_H
#define POINT_MESH_H

#include "PointSet.h"
#include "Mesh.h"
#include "Common/sort.h"
#include <deque>
using namespace std;

// 计算两个三维向量的点乘
inline float eCosNoSqrt(const float* a, const float* b){
	return (a[0]*b[0]+a[1]*b[1]+a[2]*b[2]);
}
//===== 快速排序算法=使用主键T1排序,T2跟随=======//
template <class T1, class T2>
inline int partition(T1* pData,T2* nnidx, int start, int end){
	int i = start -1;
	-- end;
	T1 fD = pData[end];
	T1 tmp;
	T2 iD = nnidx[end];
	T2 itmp;
	for (int j = start; j < end; j++) {
		if (pData[j] <= fD){
			++ i;
			tmp = pData[i];
			pData[i] = pData[j];
			pData[j] = tmp;

			itmp = nnidx[i];
			nnidx[i] = nnidx[j];
			nnidx[j] = itmp;
		}
	}
	++ i;
	pData[end] = pData[i];
	pData[i] = fD;
	nnidx[end] = nnidx[i];
	nnidx[i] = iD;
	return i;
}

template <class T1, class T2>
inline void quickSort( T1* arc, T2* nnidx, int start , int end){
	if (end - start < 2) return ;
	int i = partition(arc, nnidx, start, end);
	quickSort(arc, nnidx, start, i);
	quickSort(arc, nnidx, i+1, end);
}
//===== 波前点的状态=====//
enum POINTSTATUS{
	PS_INPOINT,				// 内点
	PS_FRONTPOINT,		// 波前点
	PS_BOUNDARY,		// 边界点，未封闭
	PS_UNUSED,				// 未使用点
	PS_DELETE					// 可以去除的点
};
//===== 波前点类 =======//
class CPointFront{
public:
	size_t m_index;	// 当前点的编号
	POINTSTATUS m_status;	// 当前点的状态
	list<size_t> m_link;	// 当前点的已相邻的点

public:
	CPointFront( POINTSTATUS status = PS_UNUSED):m_status(status){}
	CPointFront(int index, POINTSTATUS status = PS_UNUSED)
		: m_index(index), m_status(status){}
	CPointFront(const CPointFront& src){
		m_index = src.getIndex();
		m_status = src.getStatus();
		if (src.linkSize() > 0){
			m_link.resize(src.linkSize(), 0);
			std::copy(src.m_link.begin(), src.m_link.end(), m_link.begin());
		}
	}

public:
	void addLinkBack(size_t index){ m_link.push_back(index); }
	void addLinkFront(size_t index){ m_link.push_front(index); }
	size_t linkSize() const { return m_link.size(); }
	size_t linkFront() const { return m_link.front(); }
	size_t linkBack() const { return m_link.back(); }
	list<size_t>& getLink()  { return m_link; }

	//@ 判断当前点和index是否相连
	bool existLink(const size_t index) const {
		for (list<size_t>::const_iterator it = m_link.begin(); it != m_link.end() ; it++){
			if (index == *it) return true;
		}
		return false;
	}
	//@ -1=都不存在/0=都存在/1=index1存在/2=index2存在
	int existLink(const size_t index1, const size_t index2) const {
		bool bLink1 = false, bLink2 = false;
		for (list<size_t>::const_iterator it = m_link.begin(); it != m_link.end() ; it++){
			if (index1 == *it) bLink1 = true;
			else if (index2 == *it) bLink2 = true;
		}
		if (bLink1){
			if (bLink2) return 0;	// 两个都存在
			return 1;	// 第一个存在
		}
		if (bLink2) return 2;	// 第二个存在
		return -1;	// 两个都不存在
	}

	POINTSTATUS getStatus() const { return m_status; }
	void setStatus(const POINTSTATUS& status) { m_status = status; }

	size_t getIndex() const { return m_index; }
};

class CPointCloudView;
// ===== 波前点扩展法 Mesh ==========//
class CPointMesh{
public:
	CPointCloudView* m_pView;
	CPointSet* m_ps;
	vector<CPointFront> m_links;
	vector<CTriangle> m_faces;
	deque<size_t> m_fontPoints;
public:
	float m_A, m_B;
	int m_searchPointNum;	// 扩展时搜索的个数

	typedef vector<CPointFront>::iterator PFIter;
	typedef list<size_t> LinkList;
	typedef list<size_t>::iterator LSIt;
public:
	CPointMesh(CPointSet* ps, int spn = 12, float a=1.0f, float b=1.0f):m_ps(ps),m_A(a), m_B(b), m_searchPointNum(spn){}
	CPointMesh(CPointSet* ps, CPointCloudView* pView, int spn = 12, float a=1.0f, float b=1.0f)
		: m_pView(pView),m_ps(ps),m_A(a), m_B(b), m_searchPointNum(spn){}
	~CPointMesh(){
		m_links.clear();
		m_faces.clear();
	}

	void setAB(float a = 1.0, float b = 1.0){
		m_A = a;	m_B = b;
	}

public:
	void start();
	bool externPoint(size_t index);
	void checkBoundaryPoint();

	float arc(float eStart[3], float eVertical[3], float ePt[3]);
	float arcNorm(float ptCenter[3], float ptStart[3], float eNorm[3], float pt[3]);

	void sortArc(float* arc, ANNidx* index, int N);
	void sortArc(float* arc, ANNidx* index, float (*pts)[3], int N);
	void sortWith(list<size_t>& linkList,const ANNidx* nnIdx, int N);

	void formTriangles(size_t index, ANNidx* indexs, int N);
	void formTriangles(size_t index,const ANNidx* nnidx, int start, int end, bool * bStatus);

	void formTriangles2(size_t index, ANNidx* nnIdx, int N);
	void formTriangles2(size_t iSeed,const ANNidx* nnidx, int start, int end, bool * bStatus);

	bool satisfyTriRule(const size_t& idxo, const size_t& idxb, const size_t& idxc);

	void filpNormal();
protected:
	void setPointStatus(const size_t index, POINTSTATUS ps = PS_INPOINT) { m_links[index].setStatus(ps); }
	POINTSTATUS getPointStatus(const size_t index) const { return m_links[index].getStatus(); }

	void addLinkBtw(const size_t index1, const size_t index2){
		m_links[index1].addLinkBack(index2);
		m_links[index2].addLinkBack(index1);
	}
	size_t getSeed(size_t start);

	size_t findPrePoint(ANNidxArray nnidx, int iCur, bool* bLinkCenter, int N){
		for (int j = iCur-1; j > 0; j--){
			if (bLinkCenter[j]) return nnidx[j];
		}
		for (int i = N-1; i > iCur; i--)
			if (bLinkCenter[i]) return nnidx[i];
		return nnidx[iCur];
	}

public:
	void startT();
	void externSeedPoint(size_t iSeed);
	void externSeedTriangles(size_t iSeed, ANNidx* nnIdx, bool* bLinkedCenter, int N);
	int externFirstTriangle(size_t iSeed, ANNidx* nnIdx,int icur, int N);

	void externFrontPoint(size_t iSeed);
	void externFrontTriangles(size_t iSeed, ANNidx* nnIdx, bool* bLinkedCenter, int N);

	void triBtwSE(size_t iSeed, size_t iPre, const ANNidx* nnidx, int start, int end, bool* bLinkedCenter);
	int formOneTriangle(size_t iSeed, size_t iPre, const ANNidx* nnidx, int start, int end, bool* bLinkedCenter);
	float computeSeedPointFit(float ptSeed[3], float ptCur[3], size_t iNext, float preNorm[3]);
	float computeSeedPointFit(float ptSeed[3], float ptCur[3], size_t iNext);

	void CleanInPoint(size_t iSeed,ANNidx* nnidx, float (*projPt)[3], bool* bLinked, int N);
};

//************************************
// Method:    计算向量ePt和eStart之间的夹角【要求三向量在同一平面上】
// Returns:   float 夹角 【0-4】
// Parameter: float eStart[3] 起始向量
// Parameter: float eVertical[3] 与起始向量垂直(逆时针方向)
// Parameter: float ePt[3] 要计算的向量
//************************************
inline float CPointMesh::arc(float eStart[3], float eVertical[3], float ePt[3]){
	float cosStart = eCos(eStart, ePt);
	float cosVertical = eVertical[0]*ePt[0] + eVertical[1]*ePt[1] + eVertical[2]*ePt[2];
	if (cosVertical > 0)
		return 1.0f - cosStart;
	else
		return 3.0f + cosStart;
}

//************************************
// Method:    计算三点组成的向量的夹角
// Returns:   float 向量夹角 【0-4】
// Qualifier: 三点共面
// Parameter: float ptCenter[3] 中心点
// Parameter: float ptStart[3] 起始点
// Parameter: float eNorm[3] 平面法矢
// Parameter: float pt[3]	计算点
//************************************
inline float CPointMesh::arcNorm(float ptCenter[3], float ptStart[3], float eNorm[3], float pt[3]){
	float eStart[3] = Edge(ptCenter, ptStart);
	float ePt[3] = Edge(ptCenter, pt);
	float eVertical[3];
	eCross(eVertical, eNorm, eStart);
	return arc(eStart, eVertical, ePt);
}

//************************************
// Method:    根据arc的大小对index进行排序
// Parameter: float * arc 角度数组
// Parameter: int * index 序号数组
// Parameter: int N 数组长度
//************************************
inline void CPointMesh::sortArc(float* arc, ANNidx* index, int N){
	quickSort(arc, index, 1, N);
}

inline void CPointMesh::sortArc(float* arc, ANNidx* index, float (*pts)[3], int N){
	quickSort(arc, index, pts, 1, N);
}

inline bool CPointMesh::satisfyTriRule(const size_t& idxo, const size_t& idxb, const size_t& idxc){
// 	float* ptO = m_ps->m_point[idxo],
// 			* ptB = m_ps->m_point[idxb],
// 			* ptC = m_ps->m_point[idxc];
	float minTriAngel = m_ps->getMinTriAngel(idxo, idxb, idxc);
	if (minTriAngel < COS25) return false;
	return true;
}
// 将linkList中的元素按照在nnIdx中的排列顺序进行排列
inline void CPointMesh::sortWith(list<size_t>& linkList,const ANNidx* nnIdx, int N )
{
	typedef list<size_t>::iterator lIt;
	if (linkList.size() > 1){
		lIt itStart = linkList.begin(), itFind = itStart;
		for (int i = 1; i < N ; i++) {
			ANNidx curIndex = nnIdx[i];

			itFind = itStart;
			bool bFind = false;
			while(itFind != linkList.end()){
				if (curIndex == *itFind){
					bFind = true;
					break;
				}
				itFind++;
			}

			if (bFind){
				if (itFind != itStart){
					int tmp = *itFind;
					*itFind = *itStart;
					*itStart = tmp;
				}
				itStart++;
			}
		}
	}
}
// 如果未找到合适的种子，返回顶点个数
inline size_t CPointMesh::getSeed(size_t start){
	for (size_t t = start; t < m_links.size(); ++ t) {
		if (PS_UNUSED == m_links[t].getStatus())
			return t;
	}
	for(size_t t = 0; t < start; ++t){
		if (PS_UNUSED ==  m_links[t].getStatus())
			return t;
	}
	return m_links.size();
}
#endif // POINT_MESH_H