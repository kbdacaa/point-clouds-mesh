#pragma once

#include <list>
#include <vector>
#include "PointSet.h"
#include "common/vect.h"

/*
	边类：两个顶点
*/
class CEdge{
public:
	CEdge(int a, int b) { m_a = a; m_b = b; }
	int getA() const { return m_a; }
	int getB() const { return m_b; }
	void setEdge(int a, int b) { m_a = a; m_b = b; }
	bool isEdge(int a,  int b) const { return ( (a == m_a && b == m_b) || (a == m_b && b == m_a) ); }
	bool isInEdge(int idx) const { return (m_a == idx || m_b == idx); }

	bool operator==(const CEdge& edge) {  return (m_a == edge.m_a && m_b == edge.m_b); }

private:
	int m_a, m_b;		//+边的两点+
};

enum STATUS{
	ACTIVE,
	BOUNDARY
};

class CTriangle;
/*
	波前边：两个顶点和边状态
	TODO：添加前一个三角形指针
*/
class CFrontEdge : public CEdge{
public:
	CFrontEdge(int a = -1, int b = -1)
		: CEdge(a, b), m_status(ACTIVE){ m_pPreTri = NULL; }

	void setBoundary() { m_status = BOUNDARY; }
	int getStatus() const { return m_status; }
	void setCenter(const float ballCenter[3]){
		m_ballCenter[0] = ballCenter[0];
		m_ballCenter[1] = ballCenter[1];
		m_ballCenter[2] = ballCenter[2];
	}
	void setPreTriangle(CTriangle* preTri){	m_pPreTri = preTri;	}
	CTriangle* getPreTriangle() const { return m_pPreTri; }
	//bool operator==(const CFrontEdge& edge) {  return (getA() == edge.getA() && getB() == edge.getB()); }
public:
	int m_status;
	float m_ballCenter[3];
	CTriangle* m_pPreTri;
};
// TODO 最好将点的状态信息放到AF中
const short int UNUSEDPOINT = 0;
const short int INPOINT = 1;
const short int FRONTPOINT = 2;

class AdvancingFront{
public:
	bool join(CFrontEdge& edge, int vertIdx, PointSet* ps, float ballCnt[3], CTriangle* newTriangle);
	void addEdge(const CFrontEdge& edge) { m_edgeList.push_back(edge); }
	void addVertex(const int vertIdx) { m_vertexList.push_back(vertIdx); }
	bool getActiveEdge(std::list<CFrontEdge>::iterator& itret){
		for (std::list<CFrontEdge>::iterator it = m_edgeList.begin();
			it != m_edgeList.end(); ++it){
				if (it->getStatus() == ACTIVE){
					itret = it;
					return true;
				}
		}
		return false;
	}
	/*
	判断点vertIdx是否已经被使用
	*/
	bool IsVertexUsed(const int vertIdx){
		for (std::list<int>::iterator it = m_vertexList.begin();
			it != m_vertexList.end(); ++it){
				if (*it == vertIdx)
					return true;
		}
		return false;
	}
	//@ IPD算法中使用
	bool join(CFrontEdge& edge, int vertIdx, std::vector<short>& m_bPointUsed, CTriangle* newTriangle);
private:
	std::list<CFrontEdge> m_edgeList;	//+波前边链表+
	std::list<int> m_vertexList;	//+波前边上顶点链表+
};
// TODO:: 需要相邻三角面的指针
class CTriangle{
public:
	CTriangle(int a = -1, int b = -1, int c = -1)
		: m_a(a), m_b(b), m_c(c) {
			m_pNeighbor[0] = NULL;
			m_pNeighbor[1] = NULL;
			m_pNeighbor[2] = NULL;
	}
	CTriangle(const CTriangle& tri){
		setTriVertexs(tri.getA(), tri.getB(), tri.getC());
		setNeighbor(tri.getA(), (tri.getNeighbor(tri.getA())));
		setNeighbor(tri.getB(), (tri.getNeighbor(tri.getB())));
		setNeighbor(tri.getC(), (tri.getNeighbor(tri.getC())));
	}

	int getA() const { return m_a; }
	int getB() const { return m_b; }
	int getC() const { return m_c; }
	int getVertex(const CEdge& edge) const {
		if (m_a != edge.getA() && m_a!=edge.getB()) return m_a;
		else if (m_b!=edge.getA() && m_b != edge.getB()) return m_b;
		else if (m_c!=edge.getA() && m_c != edge.getB()) return m_c;
		else return -1;
	}

	void setTriVertexs(int a, int b, int c) { m_a = a; m_b = b; m_c = c; }
	bool isTriEdge(const CEdge& e){
		int a = e.getA();
		int b = e.getB();
		return ( (a == m_a || a == m_b || a == m_c) && (b == m_a || b == m_b || b == m_c));
	}
	bool isTriVertex(const int idx){
		return (m_a == idx || m_b == idx || m_c == idx);
	}
	// 此处假设idx是三角形的一个顶点
	void setNeighbor(const int idx, CTriangle* pface){
		if (idx == m_a) m_pNeighbor[0] = pface;
		else if (idx == m_b) m_pNeighbor[1] = pface;
		else m_pNeighbor[2] = pface;
	}
	// 此处假设edge必须是三角形的一条边
	void setNeighbor(const CFrontEdge& edge, CTriangle* pface){
		if (m_a != edge.getA() && m_a!=edge.getB()) setNeighbor(m_a, pface);
		else if (m_b!=edge.getA() && m_b != edge.getB()) setNeighbor(m_b, pface);
		else setNeighbor(m_c, pface);
	}
	// 返回对应顶点对应的邻居,如果不是三角形顶点返回NULL
	 CTriangle* getNeighbor(int idx)const{
		if (idx == m_a) return m_pNeighbor[0];
		else if (idx == m_b) return m_pNeighbor[1];
		else if (idx == m_c) return m_pNeighbor[2];
		return NULL;
	}

private:
	int m_a, m_b, m_c;	// 三角形的三个顶点
	CTriangle* m_pNeighbor[3];	// 三角形三个顶点对应的相邻三角形
};

class CMesh
{
public:
	PointSet* m_ps;
	std::vector<CTriangle*> m_faceVects;
// 	std::list<CTriangle> m_triList;
	AdvancingFront m_front;
	//std::vector<bool> m_bPointUsed;	// 点是否被使用的标志
	std::vector<short int> m_bPointUsed;
public:
	CMesh(PointSet* ps);
	~CMesh(void);

public:
	virtual void start() = 0;
	virtual void writeToFile(char* pFileName);
	/* 返回从起始点开始的第一个未被使用的点序号，如果未找到返回-1
		start为查找起始点	*/
	int  getFirstUnusedPt(int start = 0);
	//@ 设置顶点的状态(使用/未使用)
	void setVertexUsed(const int& idx, short bUsed = FRONTPOINT){
		m_bPointUsed[idx] = bUsed;
	}
	//@ 计算一条边的中点
	void getEdgeMid(const CEdge& edge, float* pt){
		int idxA = edge.getA(), idxB = edge.getB();
		float* ptA = m_ps->m_point[idxA],
				* ptB = m_ps->m_point[idxB];
		pt[0] = (ptA[0]+ptB[0])/2.0f;
		pt[1] = (ptA[1]+ptB[1])/2.0f;
		pt[2] = (ptA[2]+ptB[2])/2.0f;
	}
	void getEdgeMid(const int& idxA, const int& idxB, float* pt){
		float* ptA = m_ps->m_point[idxA],
				* ptB = m_ps->m_point[idxB];
		pt[0] = (ptA[0]+ptB[0])/2.0f;
		pt[1] = (ptA[1]+ptB[1])/2.0f;
		pt[2] = (ptA[2]+ptB[2])/2.0f;
	}
	//@ 计算边长
	float getEdgeLen(const CEdge& edge){
		int idxA = edge.getA(), idxB = edge.getB();
		float* ptA = m_ps->m_point[idxA],
				* ptB = m_ps->m_point[idxB];
		return sqrt( (ptA[0]-ptB[0])*(ptA[0]-ptB[0])+(ptA[1]-ptB[1])*(ptA[1]-ptB[1])+(ptA[2]-ptB[2])*(ptA[2]-ptB[2]) );
	}
	float getEdgeLen(const int& idxA, const int& idxB){
		float* ptA = m_ps->m_point[idxA],
				* ptB = m_ps->m_point[idxB];
		return sqrt( (ptA[0]-ptB[0])*(ptA[0]-ptB[0])+(ptA[1]-ptB[1])*(ptA[1]-ptB[1])+(ptA[2]-ptB[2])*(ptA[2]-ptB[2]) );
	}
	//@ 计算边长的平方
	float getEdgeLen2(const CEdge& edge){
		int idxA = edge.getA(), idxB = edge.getB();
		float* ptA = m_ps->m_point[idxA],
				* ptB = m_ps->m_point[idxB];
		return ( (ptA[0]-ptB[0])*(ptA[0]-ptB[0])+(ptA[1]-ptB[1])*(ptA[1]-ptB[1])+(ptA[2]-ptB[2])*(ptA[2]-ptB[2]) );
	}
	float getEdgeLen2(const int& idxA, const int& idxB){
		float* ptA = m_ps->m_point[idxA],
				* ptB = m_ps->m_point[idxB];
		return ( (ptA[0]-ptB[0])*(ptA[0]-ptB[0])+(ptA[1]-ptB[1])*(ptA[1]-ptB[1])+(ptA[2]-ptB[2])*(ptA[2]-ptB[2]) );
	}
};
// TODO: 如何确定m_ballR的大小比较合适，不会产生空洞，又不会过大
class CBpaMesh : public CMesh
{
public:
	int m_K;		// K邻域的大小K
	float m_ballR;
public:
	CBpaMesh(PointSet* ps, int k = 15);
	void start();
	bool ballPivot(CFrontEdge& fontEdge, int& vertIdx, int idPrec);
// 	void writeToFile(char* pFileName);
	bool ballPivotT( CFrontEdge& fontEdge, int& vertIdx, int idPrec ,float ballCnt[3]);

	bool getBallCenter(float center[3], int idxA, int idxB, int idxC, float ballR);
	bool getBallCenter(float center[3], float* ptA, float* ptB, float* ptC,float normal[3], float ballR);
	float getArc(float* center, float* Cijo, float* Cko);
	/* 计算种子三角形和三角形所在球心 */
	bool findSeedTriange(CTriangle& face, float ballCenter[3]);

/*
	void addTriangle(const CEdge& edge, int idx){
		CTriangle face;
		face.setTriVertexs(edge.getA(), idx, edge.getB());
		m_triList.push_back(face);
	}
*/
};

class CIPDMesh : public CMesh
{
public:
	CIPDMesh(PointSet* ps):CMesh(ps){}
	void start();
	bool findSeedTriangle(CTriangle& face, int K = 15);
	bool findSeedTriangle2(CTriangle*& pFace, int K = 15);
	bool getBestPt(CFrontEdge& frontEdge, int& bestIdx, const int& K = 15);
protected:
	float getDists(const int& idxC, const int& idxA, const int& idxB){
		float** pts = m_ps->m_point;
		float* ptc = pts[idxC], *pta = pts[idxA], *ptb = pts[idxB];
		float ca = sqrt( (ptc[0]-pta[0])*(ptc[0]-pta[0]) + (ptc[1]-pta[1])*(ptc[1]-pta[1]) + (ptc[2]-pta[2])*(ptc[2]-pta[2]) ),
				 cb = sqrt( (ptc[0]-ptb[0])*(ptc[0]-ptb[0]) + (ptc[1]-ptb[1])*(ptc[1]-ptb[1]) + (ptc[2]-ptb[2])*(ptc[2]-ptb[2]) );
		return ca+cb;
	}
	int getMaxZIndex();
	//@ 计算三角形内最小的内角, 返回1-cos(a) [0-2]随角度[0-180]递增
	float getMinTriAngel(const int& idxA, const int& idxB, const int& idxC);
	//@ 计算夹角 idxO为交点(角点), 返回1-cos(a) [0-2]随角度[0-180]递增
	float getAngel(const int& idxPre, const int& idxO, const int& idxNext);
	//@ 计算三角形的面积
	float getTriArea(const int& idxA, const int& idxB, const int& idxC);
	float getTriArea(const float&lenAB2, const float& lenAC2, const float& lenBC2){
		float ab = sqrt(lenAB2), ac = sqrt(lenAC2), bc = sqrt(lenBC2);
		float p = (ab+ac+bc) / 2.0f;
		return sqrt( p*(p-ab)*(p-bc)*(p-ac) );
	}
	//@ 计算两个三角形形成的二面角, 返回1-cos(a) [0-2]随角度[0-180]递增
	//float getDihedralAngel(const CTriangle* pPreFace, const int& idxPre, const int& idx);
	float getDihedralAngel(const CFrontEdge& frontEdge, const int& idx);	// frontEdge需要指向前一个三角形的指针
	float getDihedralAngel(const int& idxPre, const int& idxA, const int& idxB, const int& idxNext);

protected:
	//@ IPD算法中计算加权边长的值（取值越小越好）
	float getIPDWeightTri(CFrontEdge& edge, int idxNext){
		CTriangle* preTri = edge.getPreTriangle();
		int idxA = edge.getA(), idxB = edge.getB();
		int idxPre = preTri->getC();
		if (idxPre == idxA || idxPre ==idxB){
			idxPre = preTri->getA();
			if (idxPre == idxA || idxPre == idxB)
				idxPre = preTri->getB();
		}
		float lenAB2 = getEdgeLen2(idxA, idxB),
				 lenAP2 = getEdgeLen2(idxA, idxPre),
				 lenBP2 = getEdgeLen2(idxB, idxPre),
				 lenAN2 = getEdgeLen2(idxA, idxNext),
				 lenBN2 = getEdgeLen2(idxB, idxNext);
		float triAreaPre = getTriArea(lenAB2, lenAP2, lenBP2),
				 triAreaNext = getTriArea(lenAB2, lenAN2, lenBN2);	// 面积为0？？

		float kAN = (lenAN2+lenBN2-lenAB2)/triAreaNext,
				 kAB = (lenAP2+lenBP2-lenAB2)/triAreaPre +kAN;
		kAN *= 2.0f;
		float weight = kAB*lenAB2 + kAN*(lenAN2+lenBN2);
		return weight;
	}
	float getIPDWeightTri(int idxPre, int idxA, int idxB, int idxNext){
		float lenAB2 = getEdgeLen2(idxA, idxB),
				 lenAP2 = getEdgeLen2(idxA, idxPre),
				 lenBP2 = getEdgeLen2(idxB, idxPre),
				 lenAN2 = getEdgeLen2(idxA, idxNext),
				 lenBN2 = getEdgeLen2(idxB, idxNext);
		float triAreaPre = getTriArea(lenAB2, lenAP2, lenBP2),
				 triAreaNext = getTriArea(lenAB2, lenAN2, lenBN2);	// 面积为0？？
		float kAN = (lenAN2+lenBN2-lenAB2)/triAreaNext,
				 kAB = (lenAP2+lenBP2-lenAB2)/triAreaPre +kAN;
		kAN *= 2.0f;
		float weight = kAB*lenAB2 + kAN*(lenAN2+lenBN2);
		return weight;
	}
	//@ HRG算法中计算二面角权值（取值越小越好）(lamd取值影响其所占权重的比例)
	float getHRGDihedralWeight(const float& lamd, float cos1a){
		return lamd*cos1a;
	}
	//@ 混合策略计算加权权值E'tri（取值越小越好）
	float getEWeightTri(CFrontEdge& edge, int idxNext){
		int idxA = edge.getA(), idxB = edge.getB();
		float lenAB2 = getEdgeLen2(idxA, idxB),
				 lenAN2 = getEdgeLen2(idxA, idxNext),
				 lenBN2 = getEdgeLen2(idxB, idxNext);
		float lenAB = sqrt(lenAB2),
				 lenAN = sqrt(lenAN2),
				 lenBN = sqrt(lenBN2);
		float p = (lenAB+lenAN+lenBN) / 2.0f;
		float triArea = sqrt( p*(p-lenAB)*(p-lenAN)*(p-lenBN) );
		float weight = p*(lenAN2+lenBN2-lenAB2) / triArea;
		return weight;
	}
	//@ 采用E'tri和HRG加权合并的权值(lamd取值影响二面角所占权重的比例)
	float getWeightTri(CFrontEdge& edge, int idxNext, const float& lamd){
		float eWeight = getEWeightTri(edge, idxNext);
		float cos1a = getDihedralAngel(edge, idxNext);
		float HRGWeight = getHRGDihedralWeight(lamd, cos1a);
		return eWeight + HRGWeight;
	}
	float getWeightTri(CFrontEdge& edge, int idxNext, const float& lamd, const float& dihedralAngel){
		float eWeight = getEWeightTri(edge, idxNext);
		float HRGWeight = getHRGDihedralWeight(lamd, dihedralAngel);
		return eWeight + HRGWeight;
	}
};