#pragma once

#include <list>
#include <vector>
#include "PointSet.h"
#include "common/vect.h"

#define FrontEdgeIterator std::list<CFrontEdge>::iterator
/*
	���ࣺ��������
*/
class CEdge{
public:
	CEdge(int a, int b) : m_a(a), m_b(b){}
	CEdge(const CEdge& src) : m_a(src.getA()),m_b(src.getB()){}

	int getA() const { return m_a; }
	int getB() const { return m_b; }
	void setEdge(int a, int b) { m_a = a; m_b = b; }
	bool isEdge(int a,  int b) const { return ( (a == m_a && b == m_b) || (a == m_b && b == m_a) ); }
	bool isInEdge(int idx) const { return (m_a == idx || m_b == idx); }

	bool operator==(const CEdge& edge) {  return (m_a == edge.m_a && m_b == edge.m_b); }

private:
	int m_a, m_b;		//+�ߵ�����+
};

enum STATUS{
	ACTIVE,
	BOUNDARY
};

class CTriangle;
/*
	��ǰ�ߣ���������ͱ�״̬
	TODO�����ǰһ��������ָ��
*/
class CFrontEdge : public CEdge{
public:
	CFrontEdge(int a = -1, int b = -1,int c = -1, CTriangle* pFace = NULL)
		: CEdge(a, b), m_status(ACTIVE),m_c(c), m_pPreTri(pFace){}
	CFrontEdge(const CFrontEdge& src)
		: CEdge(src), m_status(src.getStatus()),
		m_c(src.getOppPoint()), m_pPreTri(src.getPreTriangle()){
			setCenter(src.m_ballCenter);
	}

public:
	int getOppPoint()const { return m_c; }	// �õ����������ε���һ������
	void setOppPoint(const int c) { m_c = c; }
	void setBoundary() { m_status = BOUNDARY; }
	bool bBoundary() const { return m_status == BOUNDARY; }
	int getStatus() const { return m_status; }
	void setCenter(const float ballCenter[3]){
		m_ballCenter[0] = ballCenter[0];
		m_ballCenter[1] = ballCenter[1];
		m_ballCenter[2] = ballCenter[2];
	}
	void setPreTriangle( CTriangle* preTri){	m_pPreTri = preTri;	}
	CTriangle* getPreTriangle() const { return m_pPreTri; }
	//bool operator==(const CFrontEdge& edge) {  return (getA() == edge.getA() && getB() == edge.getB()); }
public:
	int m_status;						// �ߵ�״̬
	int m_c;								// ��һ�������εĶ�Ӧ��
	float m_ballCenter[3];		// ��һ�������ε�����
	CTriangle* m_pPreTri;		// ��һ��������
};
// TODO ��ý����״̬��Ϣ�ŵ�AF��
const short  UNUSEDPOINT = 0;
const short  INPOINT = 1;
const short  FRONTPOINT = 2;
const short  BOUNDARYPOINT = -1;

class AdvancingFront{
public:
	~AdvancingFront(){
		m_frontEdgeList.clear();
		m_boundaryEdgeList.clear();
		m_vertexList.clear();
	}
public:
	bool join(CFrontEdge& edge, int vertIdx, CPointSet* ps, float ballCnt[3], CTriangle* newTriangle);
	void addEdge(const CFrontEdge& edge) { m_frontEdgeList.push_back(edge); }
	void addVertex(const int vertIdx) { m_vertexList.push_back(vertIdx); }
	bool getActiveEdge(std::list<CFrontEdge>::iterator& itret){
		for (std::list<CFrontEdge>::iterator it = m_frontEdgeList.begin();
			it != m_frontEdgeList.end(); ++it){
				if (it->getStatus() == ACTIVE){
					itret = it;
					return true;
				}
		}
		return false;
	}
	/*
	�жϵ�vertIdx�Ƿ��Ѿ���ʹ��
	*/
	bool IsVertexUsed(const int vertIdx){
		for (std::list<int>::iterator it = m_vertexList.begin();
			it != m_vertexList.end(); ++it){
				if (*it == vertIdx)
					return true;
		}
		return false;
	}
	//@ IPD�㷨��ʹ��
	bool join(CFrontEdge& edge, int vertIdx, std::vector<short>& m_bPointUsed, CTriangle* newTriangle);
	void setBoundary(CFrontEdge& edge){
		edge.setBoundary();
		//m_boundaryEdgeList.push_back(edge);
		//m_frontEdgeList.remove(edge);
	}
	std::list<CFrontEdge>& frontEdgeList() { return m_frontEdgeList; }
	int frontEdgeSize() const { return m_frontEdgeList.size(); }
private:
	std::list<CFrontEdge> m_frontEdgeList;	// +��ǰ������+
	std::list<CFrontEdge> m_boundaryEdgeList;	// +�߽������+
	std::list<int> m_vertexList;	// +��ǰ���϶�������+
};
// TODO:: ��Ҫ�����������ָ��
class CTriangle{
private:
	int v[3];	// �����ε���������
	float n[3];	// �����εķ�ʸ
// 	CTriangle* m_pNeighbor[3];	// ���������������Ӧ������������

public:
	CTriangle(int a = -1, int b = -1, int c = -1){
		v[0] = a;
		v[1] = b;
		v[2] = c;
// 		m_pNeighbor[0] = NULL;
// 		m_pNeighbor[1] = NULL;
// 		m_pNeighbor[2] = NULL;
	}
	CTriangle(const CTriangle& tri){
		setTriVertexs(tri.getA(), tri.getB(), tri.getC());
		float norm[3];
		tri.getNorm(norm);
		setNorm(norm);
// 		setNeighbor(tri.getA(), (tri.getNeighbor(tri.getA())));
// 		setNeighbor(tri.getB(), (tri.getNeighbor(tri.getB())));
// 		setNeighbor(tri.getC(), (tri.getNeighbor(tri.getC())));
	}

	int getA() const { return v[0]; }
	int getB() const { return v[1]; }
	int getC() const { return v[2]; }
	int vertex(int i)const {	assert(i > -1 && i < 3);return v[i]; }
	void swapVertex(int i, int j) { int t = v[i]; v[i]=v[j]; v[j]=t; }

	 //@ ������������ķ�ʸ
	 void calcNorm(CPointSet* ps){
		 float** points = ps->m_point;
		 float* ptA = points[getA()],
				 * ptB = points[getB()],
				 * ptC = points[getC()];
		 float eAB[3] = Edge(ptA, ptB);
		 float eAC[3] = Edge(ptA, ptC);
		 eCross(n, eAB, eAC);
		 eUnit(n);
	 }
	 void filpNorm(){
		 n[0] = -n[0];
		 n[1] = -n[1];
		 n[2] = -n[2];
	 }
	 void getNorm(float norm[3]) const {
		 norm[0] = n[0];
		 norm[1] = n[1];
		 norm[2] = n[2];
	 }
	 float getNormIdx(int i) const { return n[i]; }
	 void setNorm(const float norm[3]) {
		 n[0] = norm[0];
		 n[1] = norm[1];
		 n[2] = norm[2];
	 }

public:	// ====== For FrontEdge Usage ===== //
	int getVertex(const CEdge& edge) const {
		if (v[0] != edge.getA() && v[0]!=edge.getB()) return v[0];
		else if (v[1]!=edge.getA() && v[1] != edge.getB()) return v[1];
		else if (v[2]!=edge.getA() && v[2] != edge.getB()) return v[2];
		else return -1;
	}
	void setTriVertexs(int a, int b, int c) { v[0] = a; v[1] = b; v[2] = c; }
	bool isTriEdge(const CEdge& e){
		int a = e.getA();
		int b = e.getB();
		return ( (a == v[0] || a == v[1] || a == v[2]) && (b == v[0] || b == v[1] || b == v[2]));
	}
	bool isTriVertex(const int idx){
		return (v[0] == idx || v[1] == idx || v[2] == idx);
	}
/*
	//! �˴�����idx�������ε�һ������
	void setNeighbor(const int idx, CTriangle* pface){
		if (idx == v[0]) m_pNeighbor[0] = pface;
		else if (idx == v[1]) m_pNeighbor[1] = pface;
		else m_pNeighbor[2] = pface;
	}
	//! �˴�����edge�����������ε�һ����
	void setNeighbor(const CFrontEdge& edge, CTriangle* pface){
		if (v[0] != edge.getA() && v[0]!=edge.getB()) setNeighbor(v[0], pface);
		else if (v[1]!=edge.getA() && v[1] != edge.getB()) setNeighbor(v[1], pface);
		else setNeighbor(v[2], pface);
	}
	void setNeighbor(const FrontEdgeIterator& itEdge, CTriangle* pface){
		if (v[0] != itEdge->getA() && v[0]!=itEdge->getB()) setNeighbor(v[0], pface);
		else if (v[1]!=itEdge->getA() && v[1] != itEdge->getB()) setNeighbor(v[1], pface);
		else setNeighbor(v[2], pface);
	}

	// ���ض�Ӧ�����Ӧ���ھ�,������������ζ��㷵��NULL
	 CTriangle* getNeighbor(int idx)const{
		if (idx == v[0]) return m_pNeighbor[0];
		else if (idx == v[1]) return m_pNeighbor[1];
		else if (idx == v[2]) return m_pNeighbor[2];
		return NULL;
	}
	 CTriangle* iNeighbor(int index) const{
		 return m_pNeighbor[index];
	 }
*/
};

class CPlane{
public:
	vect3f m_norm;	// ƽ��ĵ�λ��ʸ
	float  m_d;				// ����d
public:
	CPlane():m_norm(0, 0,1), m_d(0){}
	CPlane(const vect3f& norm, const float& d):m_norm(norm), m_d(d){ m_norm.unit(); }
	CPlane(const vect3f& norm, const float pt[3]):m_norm(norm){
		m_norm.unit();
		m_d = -pt[0]*m_norm[0] - pt[1]*m_norm[1] - pt[2]*m_norm[2];
	}
	CPlane(const float norm[3], const float pt[3])
		:m_norm(norm[0], norm[1], norm[2]){
		m_norm.unit();
		m_d = -pt[0]*m_norm[0] - pt[1]*m_norm[1] - pt[2]*m_norm[2];
	}
	CPlane(const CPointSet* ps, int idxA, int idxB, int idxC){
		float** pts = ps->m_point;
		float* ptA = pts[idxA],
			* ptB = pts[idxB],
			* ptC = pts[idxC];
		vect3f vAB( ptB[0] - ptA[0], ptB[1] - ptA[1], ptB[2] - ptA[2]),
			vAC(ptC[0] - ptA[0], ptC[1] - ptA[1], ptC[2] - ptA[2]);
		vAB.cross(vAC);
		vAB.unit();
		m_norm = vAB;
		m_d = -vAB[0]*ptA[0] - -vAB[1]*ptA[1] - vAB[2]*ptA[2];
	}
	CPlane& operator=(const CPlane& p){
		if (this != &p){
			this->m_norm = p.m_norm;
			setD(p.d());
		}
		return *this;
	}
	float a() const { return m_norm.at(0); }
	float b() const { return m_norm.at(1); }
	float c() const { return m_norm.at(2); }
	float d() const {return m_d; }
	void setNorm(const vect3f& norm) { m_norm = norm; m_norm.unit(); }
	void setNorm(const float norm[3]) { m_norm[0] = norm[0]; m_norm[1] = norm[1]; m_norm[2] = norm[2];m_norm.unit(); }
	void setD(float d) { m_d = d; }
	void setPlane(const vect3f& norm, const float pt[3])	{
		m_norm = norm; m_norm.unit();
		m_d = -pt[0]*m_norm[0] - pt[1]*m_norm[1] - pt[2]*m_norm[2];
	}
	//@ �����point�����ϵ�ͶӰretPt
	float projectPoint(float retPt[3], const float point[3]) const {
		float len = ptDistToPlane(point);
		retPt[0] = point[0] - len*a();
		retPt[1] = point[1] - len*b();
		retPt[2] = point[2] - len*c();
		return len;
	}
	//@ ����㵽ƽ����������
	float ptDistToPlane(const float pt[3]) const {
		return (pt[0]*a() + pt[1]*b() + pt[2]*c() + d());
	}
	//@ �ж�����A��B�Ƿ���ƽ���ͬһ��(1=ͬһ��/0=����/-1=��ͬ��)
	int iSameSide(const float ptA[3], const float ptB[3]) const{
#define ZERO  1.0e-6
		float lenA = ptDistToPlane(ptA),
				 lenB = ptDistToPlane(ptB);
		if ( (lenA > ZERO && lenB > ZERO) || (lenA < -ZERO && lenB < -ZERO)) return 1;
		else if (abs(lenA) < ZERO || abs(lenB) < ZERO) return 0;
		return -1;
		//return (/*(lenA == 0) || (lenB == 0) ||*/ (lenA > 0 && lenB > 0) || (lenA < 0 && lenB < 0));
		// return lenA*lenB >= 0;
	}
};

class TriList2;
class CMesh
{
public:
	CPointSet* m_ps;
	std::vector<CTriangle*> m_faceVects;
	std::vector<short> m_bPointUsed;	// ����ǰ��״̬(>2ʱ�������ӵı���)
// std::list<CTriangle> m_triList;
public:
	CMesh(CPointSet* ps);
	virtual ~CMesh(void);

public:
	CPointSet* pointSet() const { return m_ps; }
	std::vector<CTriangle*>& faces() { return m_faceVects; }
	std::vector<short>& pointStatus() { return m_bPointUsed; }
public:
	virtual void start() = 0;
	virtual void writeToFile(char* pFileName);

	void flipNormal(bool bFilpPointNorm = true){
		std::vector<CTriangle*>::iterator itTri = m_faceVects.begin();
		for (; itTri != m_faceVects.end(); ++itTri){
			(*itTri)->filpNorm();
		}
		if (bFilpPointNorm) m_ps->flipNormal();
	}
	void computePointNormal();	// ʹ���淨ʸ������㷨ʸ
	void fillSmallHolesMinA(int T);
	void fillPolygon(TriList2*& tri_list, int* v, int* f, int N);
	void traceOpt(TriList2*& tri_list, int i, int k, int** opt, int* v);

	void cleanBadTriangles();
	bool sort1rings(bool* decideT, int* list, int N, int current, int* visitID);
	bool getBest1ring(int*& blist, int& blistN, float& error, bool* decideT, bool* validT, int* list, int N, int current, int* visitID);
	float measure1ringError( int * list, int N, int current );

	/* ���ش���ʼ�㿪ʼ�ĵ�һ��δ��ʹ�õĵ���ţ����δ�ҵ�����-1
		startΪ������ʼ��	*/
	int  getFirstUnusedPt(int start = 0);

	//@ ���ö����״̬(ʹ��/δʹ��)
	void setVertexUsed(const int& idx, short bUsed = FRONTPOINT){
		m_bPointUsed[idx] = bUsed;
	}
	short getVertexStatus(const int& idx)const { return m_bPointUsed[idx];}

	//@ ����һ���ߵ��е�
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

	//@ ����߳�
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

	//@ ����߳���ƽ��
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
// TODO: ���ȷ��m_ballR�Ĵ�С�ȽϺ��ʣ���������ն����ֲ������
class CBpaMesh : public CMesh
{
public:
	int m_K;		// K����Ĵ�СK
	float m_ballR;
	AdvancingFront m_front;
public:
	CBpaMesh(CPointSet* ps, int k = 15);
	void start();
	bool ballPivot(CFrontEdge& fontEdge, int& vertIdx, int idPrec);
// 	void writeToFile(char* pFileName);
	bool ballPivotT( CFrontEdge& fontEdge, int& vertIdx, int idPrec ,float ballCnt[3]);

	bool getBallCenter(float center[3], int idxA, int idxB, int idxC, float ballR);
	bool getBallCenter(float center[3], float* ptA, float* ptB, float* ptC,float normal[3], float ballR);
	float getArc(float* center, float* Cijo, float* Cko);
	/* �������������κ��������������� */
	bool findSeedTriange(CTriangle& face, float ballCenter[3]);

/*
	void addTriangle(const CEdge& edge, int idx){
		CTriangle face;
		face.setTriVertexs(edge.getA(), idx, edge.getB());
		m_triList.push_back(face);
	}
*/
};

class CPointCloudView;

class CIPDMesh : public CMesh
{
public:
	CPointCloudView* pView;
	AdvancingFront m_front;
public:
	CIPDMesh(CPointSet* ps, CPointCloudView* pview = NULL):CMesh(ps){pView = pview; }
	void start();
	bool findSeedTriangle(CTriangle& face, int K = 15);
	bool findSeedTriangle2(CTriangle*& pFace, int K = 15);
	bool getBestPt(FrontEdgeIterator& itEdge, int& bestIdx, const int& K = 15);
protected:
	float getDists(const int& idxC, const int& idxA, const int& idxB){
		float** pts = m_ps->m_point;
		float* ptc = pts[idxC], *pta = pts[idxA], *ptb = pts[idxB];
		float ca = sqrt( (ptc[0]-pta[0])*(ptc[0]-pta[0]) + (ptc[1]-pta[1])*(ptc[1]-pta[1]) + (ptc[2]-pta[2])*(ptc[2]-pta[2]) ),
				 cb = sqrt( (ptc[0]-ptb[0])*(ptc[0]-ptb[0]) + (ptc[1]-ptb[1])*(ptc[1]-ptb[1]) + (ptc[2]-ptb[2])*(ptc[2]-ptb[2]) );
		return ca+cb;
	}
	int getMaxZIndex();
	//@ ��������������С���ڽ�, ����1-cos(a) [0-2]��Ƕ�[0-180]����
	float getMinTriAngel(const int& idxA, const int& idxB, const int& idxC);
	//@ ����н� idxOΪ����(�ǵ�), ����1-cos(a) [0-2]��Ƕ�[0-180]����
	float getAngel(const int& idxPre, const int& idxO, const int& idxNext);
	//@ ���������ε����
	float getTriArea(const int& idxA, const int& idxB, const int& idxC);
	float getTriArea(const float&lenAB2, const float& lenAC2, const float& lenBC2){
		float ab = sqrt(lenAB2), ac = sqrt(lenAC2), bc = sqrt(lenBC2);
		float p = (ab+ac+bc) / 2.0f;
		return sqrt( p*(p-ab)*(p-bc)*(p-ac) );
	}
	//@ ���������������γɵĶ����, ����1-cos(a) [0-2]��Ƕ�[0-180]����
	//float getDihedralAngel(const CTriangle* pPreFace, const int& idxPre, const int& idx);
	float getDihedralAngel(const CFrontEdge& frontEdge, const int& idx);
	float getDihedralAngel(const int& idxPre, const int& idxA, const int& idxB, const int& idxNext);

protected:
	//@ IPD�㷨�м����Ȩ�߳���ֵ��ȡֵԽСԽ�ã�
	float getIPDWeightTri(CFrontEdge& edge, int idxNext){
		int idxA = edge.getA(), idxB = edge.getB();
		int idxPre = edge.getOppPoint();
		return getIPDWeightTri(idxPre, idxA, idxB, idxNext);
	}
	float getIPDWeightTri(int idxPre, int idxA, int idxB, int idxNext){
		float lenAB2 = getEdgeLen2(idxA, idxB),
				 lenAP2 = getEdgeLen2(idxA, idxPre),
				 lenBP2 = getEdgeLen2(idxB, idxPre),
				 lenAN2 = getEdgeLen2(idxA, idxNext),
				 lenBN2 = getEdgeLen2(idxB, idxNext);
		float triAreaPre = getTriArea(lenAB2, lenAP2, lenBP2),
				 triAreaNext = getTriArea(lenAB2, lenAN2, lenBN2);	// ���Ϊ0����
		float kAN = (lenAN2+lenBN2-lenAB2)/triAreaNext,
				 kAB = (lenAP2+lenBP2-lenAB2)/triAreaPre +kAN;
		kAN *= 2.0f;
		float weight = kAB*lenAB2 + kAN*(lenAN2+lenBN2);
		return weight;
	}
	//@ HRG�㷨�м�������Ȩֵ��ȡֵԽСԽ�ã�(lamdȡֵӰ������ռȨ�صı���)
	float getHRGDihedralWeight(const float& lamd, float cos1a){
		return lamd*cos1a;
	}
	//@ ��ϲ��Լ����ȨȨֵE'tri��ȡֵԽСԽ�ã�
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
	//@ ����E'tri��HRG��Ȩ�ϲ���Ȩֵ(lamdȡֵӰ��������ռȨ�صı���)
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
	//@ �жϵ�idx�Ƿ���������ABC��Ӱ�����ڣ�
	bool isInPlay(int idx, int edgeA, int edgeB, int edgeC, int edgePre, int edgeNext);
	//@ ���Ӱ�����������
	int getPlanes(CPlane& pre, CPlane& next, int eA, int eB, int tC, int ePre, int eNext);
	//@ ��ñ�edge��ǰ���
	void getPreNextIdx(int& idxPre, int& idxNext, const FrontEdgeIterator& edgeIt){
		std::list<CFrontEdge>& frontEdgeList = m_front.frontEdgeList();
		int idxA = edgeIt->getA(), idxB = edgeIt->getB();
		FrontEdgeIterator itPre = edgeIt, itNext = edgeIt, it = edgeIt;
		int size = m_front.frontEdgeSize();
		idxPre = idxNext = -1;
		if (size < 2)	return;

		if (edgeIt == frontEdgeList.begin()){
			itPre = frontEdgeList.end();
			itPre --;
			itNext ++;
		}else if (edgeIt == --frontEdgeList.end() ){
			itPre --;
			itNext = frontEdgeList.begin();
		}else{
			itPre --;
			itNext ++;
		}
		if (itPre->getB() == idxA && itNext->getA() == idxB){
			idxPre = itPre->getA();
			idxNext = itNext->getB();
			return ;
		}
		for (it = frontEdgeList.begin(); it != frontEdgeList.end(); ++it){
			if (it->getB() == idxA)
				idxPre = it->getA();
			else if (it->getA() == idxB)
				idxNext = it->getB();
		}
	}
	//@ �жϵ�idx��������֮��?
	bool isBtwPlanes(const CPlane& prePlane, const CPlane& nextPlane, const CFrontEdge& edge, int idx){
		float* pt = m_ps->m_point[idx],
				* ptA = m_ps->m_point[edge.getA()],
				* ptB = m_ps->m_point[edge.getB()];
		int iSame = prePlane.iSameSide(ptB, pt);
		if (iSame < 0) return false;
		return nextPlane.iSameSide(ptA, pt) >= 0;
	}
	bool getCandidatePoint(FrontEdgeIterator& itEdge, int& bestIdx, bool& bFrontPoint, const int& K);
};