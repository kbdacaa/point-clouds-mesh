#ifndef FTPMESH
#define FTPMESH

#include "Mesh.h"
#include <algorithm>
#include <iterator>
//#include "Common/KdTree.h"
using namespace std;
#define FrontEdgeList list<CFrontEdge>
#define FrontListIterator list<CFrontList>::iterator

//! 波前环类：包含波前边链表和波前点链表及为外环否？
class CFrontList{
public:
	list<CFrontEdge> m_frontEdges;	// 波前边链表
	list<int> m_frontVertexs;	// 波前顶点链表
	bool m_bOutRing;				// 当前链表为外环？

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
	//@ 根据波前边链表生成顶点链表集
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
	//@ 判断当前环是否为外环
	bool bOutRing() const { return m_bOutRing; }
	void addVertex(const int vIdx) { m_frontVertexs.push_back(vIdx); }

	//@ 向边界环上添加一个三角形（第一步）
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

	//@ 查找itEdge的前一个iterator
	FrontEdgeIterator preEdge(const FrontEdgeIterator& itEdge){
		FrontEdgeIterator it = itEdge;
		if (it == m_frontEdges.begin()){
			it = m_frontEdges.end();
		}
		-- it;
		return it;
	}
	//@ 查找itEdge的后一个iterator (! 保证itEdge不为end() !)
	FrontEdgeIterator nextEdge(const FrontEdgeIterator& itEdge){
		FrontEdgeIterator it = itEdge;
		++ it;
		if ( it == m_frontEdges.end() ){
			it = m_frontEdges.begin();
		}
		return it;
	}

// 	bool join(FrontEdgeIterator& itEdge, int vIdx, CTriangle* face);
// 	//@ 将一个波前边链表分裂为两个
// 	bool split(FrontEdgeList& nFEList);
// 	//@ 将两个波前边链表合并为一个
// 	bool merge(FrontEdgeList& anthFEList);
// 	//@ 判断点vIdx在波前边上否？
	bool isPtOn(const int vIdx){
		for (list<CFrontEdge>::iterator it = m_frontEdges.begin(); it != m_frontEdges.end() ; it++)
		{
			if (it->getA() == vIdx) return true;
		}
		return false;
	}
	//@ 查找点vIdx在波前环上的位置
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

//! 当前边或者点在波前环集上的位置
struct FrontIter{
	FrontEdgeIterator itEdge;	// 波前边迭代器
	FrontListIterator    itList;		// 波前环迭代器
};

//! 波前环集合
class CFrontSet{
public:
	list<CFrontList> m_frontLists;	// 波前环链表
	std::vector<short>& m_sPointStatus;	// 当前点状态
	list<CFrontList> m_boundaryLists;	// 边界环链表

public:
	CFrontSet(vector<short>& sPointStatus):m_sPointStatus(sPointStatus){}

public:
	size_t frontListSize() const { return m_frontLists.size();}
	void addFrontList(const CFrontList& frontList){ m_frontLists.push_back(frontList); }

	//@ 对波前环链表按照环中边的个数进行排序
	void sortFrontLists(){
		//sort(m_frontLists.begin(), m_frontLists.end(), frontListCompare);
	}
	//@ 设置顶点的状态
	void setPointStatus(int vIdx, short status =  INPOINT){
		m_sPointStatus[vIdx] = status;
	}
	//@ 设置边和其两个端点是为边界边和点
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
	//@ 如果当前波前环只有4条边或3条边直接生成三角形
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

	//@ 向波前环集合中添加一个三角形环
	void addFace(CTriangle* pFace){
		CFrontList newList;
		newList.addFace(pFace);
		addFrontList(newList);
	}

	//@ 获得当前边的上一条边
	FrontIter preEdge(const FrontIter& itFront){
		FrontIter it;
		it.itList = itFront.itList;
		it.itEdge = itFront.itList->preEdge(itFront.itEdge);
		return it;
	}
	//@ 获得当前边的下一条边
	FrontIter nextEdge(const FrontIter& itFront){
		FrontIter it;
		it.itList = itFront.itList;
		it.itEdge = itFront.itList->nextEdge(itFront.itEdge);
		return it;
	}
	//@ 从波前边链表集中获取一条活动边
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
			//TODO 将全是边界边的FrontList 移动到 m_boundaryLists 中
			itL ++;
		}
		return false;
	}
	//@ 在波前环链表中查找点vIdx所在的环和边 (边的起点为vIdx)
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
	//@ 将一个扩展点加入进来形成新三角形
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

	//=============Mesh 权值计算方法=============//
public:
};

#endif