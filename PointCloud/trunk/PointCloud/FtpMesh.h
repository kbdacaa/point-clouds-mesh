#ifndef FTPMESH
#define FTPMESH

#include "Mesh.h"
#include <algorithm>
#include <iterator>
//#include "Common/KdTree.h"
using namespace std;
#define FrontEdgeList list<CFrontEdge>
#define FrontListIterator list<CFrontList>::iterator

//! ��ǰ���ࣺ������ǰ������Ͳ�ǰ������Ϊ�⻷��
class CFrontList{
public:
	list<CFrontEdge> m_frontEdges;	// ��ǰ������
	list<int> m_frontVertexs;	// ��ǰ��������
	bool m_bOutRing;				// ��ǰ����Ϊ�⻷��

public:
	CFrontList():m_bOutRing(true){}
// 	CFrontList(const CTriangle* pFace):m_bOutRing(true){
// 		CFrontEdge a(pFace->getA(), pFace->getB()),
// 			b(pFace->getB(), pFace->getC()),
// 			c(pFace->getC(), pFace->getA());
// 		a.setPreTriangle(pFace);
// 		b.setPreTriangle(pFace);
// 		c.setPreTriangle(pFace);
// 		m_frontEdges.push_back(a);
// 		m_frontEdges.push_back(b);
// 		m_frontEdges.push_back(c);
// 		m_frontVertexs.push_back(pFace->getA());
// 		m_frontVertexs.push_back(pFace->getB());
// 		m_frontVertexs.push_back(pFace->getC());
// 	}
	CFrontList(const CFrontList& src){
		m_frontVertexs.resize(src.frontVertexsSize());
		m_frontEdges.resize(src.frontVertexsSize());
		std::copy(src.m_frontVertexs.begin(), src.m_frontVertexs.end(), m_frontVertexs.begin());
		std::copy(src.m_frontEdges.begin(), src.m_frontEdges.end(), m_frontEdges.begin());
		m_bOutRing = src.bOutRing();
	}
	CFrontList(FrontListIterator& frontList, FrontEdgeIterator& from, FrontEdgeIterator& to, bool bOutRing = false)
	: m_bOutRing(bOutRing){
		m_frontEdges.splice(m_frontEdges.end(), frontList->frontEdges(), from, to);
		constructVertex();
	}
	CFrontList& operator=(const CFrontList& src){
		if (this == &src) return *this;
		m_frontVertexs.resize(src.frontVertexsSize());
		m_frontEdges.resize(src.frontVertexsSize());
		std::copy(src.m_frontEdges.begin(), src.m_frontEdges.end(), m_frontEdges.begin());
		std::copy(src.m_frontVertexs.begin(), src.m_frontVertexs.end(), m_frontVertexs.begin());
		m_bOutRing = src.bOutRing();
		return *this;
	}
	//@ ���ݲ�ǰ���������ɶ�������
	void constructVertex(){
		m_frontVertexs.clear();
		for (FrontEdgeIterator it = m_frontEdges.begin(); it != m_frontEdges.end(); ++it)
		{
			m_frontVertexs.push_back(it->getA());
		}
	}
public:
	int frontVertexsSize()const { return m_frontVertexs.size(); }
	int frontEdgesSize()   const { return m_frontEdges.size(); }
	list<CFrontEdge>& frontEdges() { return m_frontEdges; }
	list<int>& frontVertexs() { return m_frontVertexs; }
	//@ �жϵ�ǰ���Ƿ�Ϊ�⻷
	bool bOutRing() const { return m_bOutRing; }
	void addVertex(const int vIdx) { m_frontVertexs.push_back(vIdx); }

	//@ ��߽绷�����һ�������Σ���һ����
	void addFace(CTriangle* pFace){
		CFrontEdge a(pFace->getA(), pFace->getB(), pFace->getC(), pFace),
			b(pFace->getB(), pFace->getC(), pFace->getA(), pFace),
			c(pFace->getC(), pFace->getA(), pFace->getB(), pFace);
		m_frontEdges.push_back(a);
		m_frontEdges.push_back(b);
		m_frontEdges.push_back(c);
		addVertex(pFace->getA());
		addVertex(pFace->getB());
		addVertex(pFace->getC());
	}

	//@ ����itEdge��ǰһ��iterator
	FrontEdgeIterator preEdge(const FrontEdgeIterator& itEdge){
		FrontEdgeIterator it = itEdge;
		if (it == m_frontEdges.begin()){
			it = m_frontEdges.end();
		}
		-- it;
		return it;
	}
	//@ ����itEdge�ĺ�һ��iterator (! ��֤itEdge��Ϊend() !)
	FrontEdgeIterator nextEdge(const FrontEdgeIterator& itEdge){
		FrontEdgeIterator it = itEdge;
		++ it;
		if ( it == m_frontEdges.end() ){
			it = m_frontEdges.begin();
		}
		return it;
	}

// 	bool join(FrontEdgeIterator& itEdge, int vIdx, CTriangle* face);
// 	//@ ��һ����ǰ���������Ϊ����
// 	bool split(FrontEdgeList& nFEList);
// 	//@ ��������ǰ������ϲ�Ϊһ��
// 	bool merge(FrontEdgeList& anthFEList);
// 	//@ �жϵ�vIdx�ڲ�ǰ���Ϸ�
	bool isPtOn(const int vIdx){
		for (list<CFrontEdge>::iterator it = m_frontEdges.begin(); it != m_frontEdges.end() ; it++)
		{
			if (it->getA() == vIdx) return true;
		}
		return false;
	}
	//@ ���ҵ�vIdx�ڲ�ǰ���ϵ�λ��
	bool findVertex(FrontEdgeIterator& itEdge, const int vIdx){
		for (FrontEdgeIterator it = m_frontEdges.begin();
			it != m_frontEdges.end(); ++it){
			if (it->getA() == vIdx){
				itEdge = it;
				return true;
			}
		}
		return false;
	}
};

bool frontListCompare(const CFrontList& a, const CFrontList& b);

//! ��ǰ�߻��ߵ��ڲ�ǰ�����ϵ�λ��
struct FrontIter{
	FrontEdgeIterator itEdge;	// ��ǰ�ߵ�����
	FrontListIterator    itList;		// ��ǰ��������
};

//! ��ǰ������
class CFrontSet{
public:
	list<CFrontList> m_frontLists;	// ��ǰ������
	std::vector<short>& m_sPointStatus;	// ��ǰ��״̬
	list<CFrontList> m_boundaryLists;	// �߽绷����

public:
	CFrontSet(vector<short>& sPointStatus):m_sPointStatus(sPointStatus){}

public:
	size_t frontListSize() const { return m_frontLists.size();}
	void addFrontList(const CFrontList& frontList){ m_frontLists.push_back(frontList); }

	//@ �Բ�ǰ�������ջ��бߵĸ�����������
	void sortFrontLists(){
		//sort(m_frontLists.begin(), m_frontLists.end(), frontListCompare);
	}
	//@ ���ö����״̬
	void setPointStatus(int vIdx, short status =  INPOINT){
		m_sPointStatus[vIdx] = status;
	}
	//@ ���ñߺ��������˵���Ϊ�߽�ߺ͵�
	void setBoundary(FrontIter& itFront){
		itFront.itEdge->setBoundary();
		int A = itFront.itEdge->getA(), B = itFront.itEdge->getB();
		FrontEdgeIterator pre = itFront.itList->preEdge(itFront.itEdge),
			next = itFront.itList->nextEdge(itFront.itEdge);
		if (pre->bBoundary())
			setPointStatus(A, BOUNDARYPOINT);
		if (next->bBoundary())
			setPointStatus(B, BOUNDARYPOINT);
	}
	//@ �����ǰ��ǰ��ֻ��4���߻�3����ֱ������������
	void toFace(FrontListIterator& itList, vector<CTriangle*>& faceVector){
		int size = itList->frontEdgesSize();
		if (size == 3){
			FrontEdgeIterator it = itList->frontEdges().begin(), itNext = it;
			itNext++;
			CTriangle* pFace = new CTriangle(it->getA(), it->getB(), itNext->getB());
// 			it->getPreTriangle()->setNeighbor(it, pFace);
// 			pFace->setNeighbor(it->getA(), itNext->getPreTriangle());
// 			itNext->getPreTriangle()->setNeighbor(itNext, pFace);
// 			pFace->setNeighbor(itNext->getB(), it->getPreTriangle());
// 			itNext++;
// 			itNext->getPreTriangle()->setNeighbor(itNext, pFace);
// 			pFace->setNeighbor(it->getB(), itNext->getPreTriangle());

			faceVector.push_back(pFace);
			m_frontLists.erase(itList);
		}else if(size == 4){
			FrontEdgeIterator itE = itList->frontEdges().begin(),
				it = itE++, itN = itE++, itNN = it++;
			CTriangle* pF1 = new CTriangle(it->getA(), it->getB(), itNN->getA()),
							 * pF2 = new CTriangle(itNN->getA(), itNN->getB(), it->getA());
// 			pF1->setNeighbor(itNN->getA(), it->getPreTriangle());
// 			pF1->setNeighbor(it->getA(), itN->getPreTriangle());
// 			pF1->setNeighbor(it->getB(), pF2);
// 			it->getPreTriangle()->setNeighbor(it, pF1);
// 			itN->getPreTriangle()->setNeighbor(itN, pF1);
// 
// 			pF2->setNeighbor(itNN->getB(), pF1);
// 			pF2->setNeighbor(itNN->getA(), itE->getPreTriangle());
// 			pF2->setNeighbor(it->getA(), itNN->getPreTriangle());
// 			itE->getPreTriangle()->setNeighbor(itE, pF2);
// 			itNN->getPreTriangle()->setNeighbor(itNN, pF2);

			faceVector.push_back(pF1);
			faceVector.push_back(pF2);
			m_frontLists.erase(itList);
		}
	}
	void toFace(CFrontList& frontList, vector<CTriangle*>& faceVector){
		int size = frontList.frontEdgesSize();
		if (size == 3){
			FrontEdgeIterator it = frontList.frontEdges().begin(), itNext = it;
			itNext++;
			CTriangle* pFace = new CTriangle(it->getA(), it->getB(), itNext->getB());
// 			it->getPreTriangle()->setNeighbor(it, pFace);
// 			pFace->setNeighbor(it->getA(), itNext->getPreTriangle());
// 			itNext->getPreTriangle()->setNeighbor(itNext, pFace);
// 			pFace->setNeighbor(itNext->getB(), it->getPreTriangle());
// 			itNext++;
// 			itNext->getPreTriangle()->setNeighbor(itNext, pFace);
// 			pFace->setNeighbor(it->getB(), itNext->getPreTriangle());

			faceVector.push_back(pFace);
		}else if(size == 4){
			FrontEdgeIterator itE = frontList.frontEdges().begin(),
				it = itE++, itN = itE++, itNN = it++;
			CTriangle* pF1 = new CTriangle(it->getA(), it->getB(), itNN->getA()),
				* pF2 = new CTriangle(itNN->getA(), itNN->getB(), it->getA());
// 			pF1->setNeighbor(itNN->getA(), it->getPreTriangle());
// 			pF1->setNeighbor(it->getA(), itN->getPreTriangle());
// 			pF1->setNeighbor(it->getB(), pF2);
// 			it->getPreTriangle()->setNeighbor(it, pF1);
// 			itN->getPreTriangle()->setNeighbor(itN, pF1);
// 
// 			pF2->setNeighbor(itNN->getB(), pF1);
// 			pF2->setNeighbor(itNN->getA(), itE->getPreTriangle());
// 			pF2->setNeighbor(it->getA(), itNN->getPreTriangle());
// 			itE->getPreTriangle()->setNeighbor(itE, pF2);
// 			itNN->getPreTriangle()->setNeighbor(itNN, pF2);

			faceVector.push_back(pF1);
			faceVector.push_back(pF2);
		}
	}

	//@ ��ǰ�����������һ�������λ�
	void addFace(CTriangle* pFace){
		CFrontList newList;
		newList.addFace(pFace);
		addFrontList(newList);
	}

	//@ ��õ�ǰ�ߵ���һ����
	FrontIter preEdge(const FrontIter& itFront){
		FrontIter it;
		it.itList = itFront.itList;
		it.itEdge = itFront.itList->preEdge(itFront.itEdge);
		return it;
	}
	//@ ��õ�ǰ�ߵ���һ����
	FrontIter nextEdge(const FrontIter& itFront){
		FrontIter it;
		it.itList = itFront.itList;
		it.itEdge = itFront.itList->nextEdge(itFront.itEdge);
		return it;
	}
	//@ �Ӳ�ǰ�������л�ȡһ�����
	bool getActiveEdge(FrontIter& itFront){
		FrontListIterator itL = m_frontLists.begin();
		while (itL != m_frontLists.end())
		{
			FrontEdgeIterator itE = itL->frontEdges().begin();
			while (itE != itL->frontEdges().end())
			{
				if (itE->getStatus() == ACTIVE){
					itFront.itEdge = itE;
					itFront.itList = itL;
					return true;
				}
				itE ++;
			}
			//TODO ��ȫ�Ǳ߽�ߵ�FrontList �ƶ��� m_boundaryLists ��
			itL ++;
		}
		return false;
	}
	//@ �ڲ�ǰ�������в��ҵ�vIdx���ڵĻ��ͱ� (�ߵ����ΪvIdx)
	bool findVertex(FrontIter& itV, int vIdx){
		for (FrontListIterator itL = m_frontLists.begin(); itL != m_frontLists.end() ; itL++)
		{
			for (FrontEdgeIterator itE = itL->frontEdges().begin(); itE != itL->frontEdges().end() ; itE++)
			{
				if (itE->getA() == vIdx){
					itV.itList = itL;
					itV.itEdge = itE;
					return true;
				}
			}
		}
		return false;
	}
	//@ ��һ����չ���������γ���������
	bool join(FrontIter& itEdge, int vIdx, bool bFront, vector<CTriangle*>& faceList);
};

class CPointCloudView;

class CFTPMesh : public CMesh{
public:
	CFrontSet m_front;
	//KdTree* kdTree;
	CPointCloudView* m_pView;
public:
	CFTPMesh(CPointSet* ps, CPointCloudView* pView):CMesh(ps), m_front(m_bPointUsed), m_pView(pView){ /*kdTree = new KdTree(ps); */}
	~CFTPMesh(){	/*if (kdTree != NULL)	delete kdTree;*/	}

public:
	void start();
	bool findSeedTriangle(CTriangle*& pFace, int K =10);
	bool getCandidatePoint(const FrontIter& itFront, int& vIdx, bool& bFrontVertex );

	//=============Mesh Ȩֵ���㷽��=============//
public:
};

#endif