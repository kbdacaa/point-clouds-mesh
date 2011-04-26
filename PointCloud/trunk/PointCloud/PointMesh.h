#ifndef POINT_MESH_H
#define POINT_MESH_H

#include "PointSet.h"
#include "Mesh.h"
#include <deque>
using namespace std;

enum POINTSTATUS{
	PS_INPOINT,				// �ڵ�
	PS_FRONTPOINT,		// ��ǰ��
	PS_BOUNDARY,		// �߽�㣬δ���
	PS_UNUSED				// δʹ�õ�
};

class CPointFront{
public:
	size_t m_index;	// ��ǰ��ı��
	POINTSTATUS m_status;	// ��ǰ���״̬
	list<size_t> m_link;	// ��ǰ��������ڵĵ�

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
// 	size_t getLinkPoint(int i) const { return m_link[i]; }
	//@ �жϵ�ǰ���index�Ƿ�����
	bool existLink(const size_t index) const {
		for (list<size_t>::const_iterator it = m_link.begin(); it != m_link.end() ; it++){
			if (index == *it) return true;
		}
		return false;
	}
	//@ -1=��������/0=������/1=index1����/2=index2����
	int existLink(const size_t index1, const size_t index2) const {
		bool bLink1 = false, bLink2 = false;
		for (list<size_t>::const_iterator it = m_link.begin(); it != m_link.end() ; it++){
			if (index1 == *it) bLink1 = true;
			else if (index2 == *it) bLink2 = true;
		}
		if (bLink1){
			if (bLink2) return 0;	// ����������
			return 1;	// ��һ������
		}
		if (bLink2) return 2;	// �ڶ�������
		return -1;	// ������������
	}

	POINTSTATUS getStatus() const { return m_status; }
	void setStatus(const POINTSTATUS& status) { m_status = status; }

	size_t getIndex() const { return m_index; }
};

class CPointMesh{
public:
	PointSet* m_ps;
	vector<CPointFront> m_links;
	vector<CTriangle> m_faces;
	deque<size_t> m_fontPoints;

	typedef vector<CPointFront>::iterator PFIter;
	typedef list<size_t> LinkList;
	typedef list<size_t>::iterator LSIt;
public:
	CPointMesh(PointSet* ps):m_ps(ps){ m_links.resize(m_ps->getPointSize()); }

	void start();
	bool externPoint(size_t index);

	float arc(float eStart[3], float eVertical[3], float ePt[3]);
	float arcNorm(float ptCenter[3], float ptStart[3], float eNorm[3], float pt[3]);
	void sortArc(float* arc, int* index, int N);
	void sortWith(list<size_t>& linkList,const int* nnIdx, int N);
	void formTriangles(int index, int* indexs, int N);
	void formTriangles2(int index, int* nnIdx, int N);
	void formTriangles(int index,const ANNidx* nnidx, int start, int end, bool * bStatus);
	bool satisfyTriRule(const size_t& idxo, const size_t& idxb, const size_t& idxc);

protected:
	void setPointStatus(const size_t index, POINTSTATUS ps = PS_INPOINT) { m_links[index].setStatus(ps); }
	POINTSTATUS getPointStatus(const size_t index) const { return m_links[index].getStatus(); }
	void addLinkBtw(const size_t index1, const size_t index2){
		m_links[index1].addLinkBack(index2);
		m_links[index2].addLinkBack(index1);
	}
	size_t getSeed(size_t start);
};

//************************************
// Method:    ��������ePt��eStart֮��ļнǡ�Ҫ����������ͬһƽ���ϡ�
// Returns:   float �н� ��0-4��
// Parameter: float eStart[3] ��ʼ����
// Parameter: float eVertical[3] ����ʼ������ֱ(��ʱ�뷽��)
// Parameter: float ePt[3] Ҫ���������
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
// Method:    ����������ɵ������ļн�
// Returns:   float �����н� ��0-4��
// Qualifier: ���㹲��
// Parameter: float ptCenter[3] ���ĵ�
// Parameter: float ptStart[3] ��ʼ��
// Parameter: float eNorm[3] ƽ�淨ʸ
// Parameter: float pt[3]	�����
//************************************
inline float CPointMesh::arcNorm(float ptCenter[3], float ptStart[3], float eNorm[3], float pt[3]){
	float eStart[3] = Edge(ptCenter, ptStart);
	float ePt[3] = Edge(ptCenter, pt);
	float eVertical[3];
	eCross(eVertical, eNorm, eStart);
	return arc(eStart, eVertical, ePt);
}

//************************************
// Method:    ����arc�Ĵ�С��index��������
// Parameter: float * arc �Ƕ�����
// Parameter: int * index �������
// Parameter: int N ���鳤��
//************************************
inline void CPointMesh::sortArc(float* arc, int* index, int N){
	for (int i = 1; i < N ; i++) {
		float tmp = arc[i];
		int tmpi = index[i];
		int j = i;
		for (; j > 1 && tmp < arc[j-1]; j++) {
			arc[j] = arc[j-1];
			index[j] = index[j-1];
		}
		arc[j] = tmp;
		index[j] = tmpi;
	}
}

inline bool CPointMesh::satisfyTriRule(const size_t& idxo, const size_t& idxb, const size_t& idxc){
// 	float* ptO = m_ps->m_point[idxo],
// 			* ptB = m_ps->m_point[idxb],
// 			* ptC = m_ps->m_point[idxc];
	float minTriAngel = m_ps->getMinTriAngel(idxo, idxb, idxc);
	if (minTriAngel < COS20) return false;
	return true;
}
// ��linkList�е�Ԫ�ذ�����nnIdx�е�����˳���������
inline void CPointMesh::sortWith(list<size_t>& linkList,const int* nnIdx, int N )
{
	typedef list<size_t>::iterator lIt;
	if (linkList.size() > 1){
		lIt itStart = linkList.begin(), itFind = itStart;
		for (int i = 1; i < N ; i++) {
			int curIndex = nnIdx[i];

			itFind = itStart;
			bool bFind = false;
			do {
				if (curIndex == *itFind){
					bFind = true;
					break;
				}
				itFind++;
			} while (itFind != linkList.end());

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