#include "StdAfx.h"
#include "Mesh.h"
#include <fstream>
#include <iostream>
using namespace std;
// TODO �˴���Ҫ����һ���µ������棬�����������õ�ǰ�¼���ı����õ�ָ��
bool AdvancingFront::join(CFrontEdge& edge, int vertIdx, PointSet* ps, float ballCnt[3], CTriangle* newTriangle){
	bool used = IsVertexUsed(vertIdx);
	list<CFrontEdge>::iterator it, toGlue[2];
	int nbFound = 0;
	bool found = false;
	if (used){
		it = m_edgeList.begin();	//+�����еĲ�ǰ�߼��в��ҵ�vertIdx+
		while (it != m_edgeList.end() && nbFound < 2){
			if (it->isInEdge(vertIdx)){
				found = true;
				if (it->isInEdge(edge.getA()) || it->isInEdge(edge.getB())){
					toGlue[nbFound++] = it;
				}
			}
			it ++;
		}
		if (nbFound >= 2)
			nbFound --;
		if (!found){
			edge.setBoundary();// �˴�ӦΪ�ҵ��ĵ�Ϊ�ǲ�ǰ���ϵĵ㣬�����Ѿ���ʹ���ˣ������ڲ��ĵ㣩
			return false;
		}
	} else {
		addVertex(vertIdx);
	}

	int idxA = edge.getA();
	int idxB = edge.getB();

	CFrontEdge a(idxA, vertIdx), b(vertIdx, idxB);
	newTriangle->setTriVertexs(idxA, vertIdx, idxB);
	if ( !ps->checkConsistence(idxA, vertIdx, idxB)){
		a.setEdge(vertIdx, idxA);
		b.setEdge(idxB, vertIdx);
		newTriangle->setTriVertexs(idxA, idxB, vertIdx);
	}
	// ����Ҫ����a��b��ǰһ���������,�Լ��������point
	// ���±�Ӧ��ָ��ǰӵ�������Ҳ��������һ���γɵ���
	a.setCenter(ballCnt);
	b.setCenter(ballCnt);
	a.setPreTriangle(newTriangle);
	b.setPreTriangle(newTriangle);
	// �������������ε����ӹ�ϵ  ���ñ�edge
	newTriangle->setNeighbor(vertIdx, edge.getPreTriangle());
	edge.getPreTriangle()->setNeighbor(edge, newTriangle);

	m_edgeList.remove(edge);

	bool insertA = true, insertB = true;
	if (used){
		for (int i = 0; i < nbFound ; i++)
		{
			list<CFrontEdge>::iterator it = toGlue[i];
			if ( (*it) == CEdge(a.getB(), a.getA()) ){
				//  �˴������������ָ��
				newTriangle->setNeighbor(idxB, it->getPreTriangle());
				it->getPreTriangle()->setNeighbor(*it, newTriangle);
				insertA = false;
				m_edgeList.erase(it);
				m_vertexList.remove(idxA);
			}else if ( (*it) == CEdge(b.getB(), b.getA()) ){
				newTriangle->setNeighbor(idxA, it->getPreTriangle());
				it->getPreTriangle()->setNeighbor(*it, newTriangle);
				insertB = false;
				m_edgeList.erase(it);
				m_vertexList.remove(idxB);
			}
		}
	}

	if (insertA)
		m_edgeList.push_back(a);
	if (insertB)
		m_edgeList.push_back(b);
	return true;
}

bool AdvancingFront::join( CFrontEdge& edge, int vertIdx, vector<short>& m_bPointUsed, CTriangle* newTriangle )
{
	typedef list<CFrontEdge>::iterator IT;
	list<IT> itListA, itListB;
	IT it = m_edgeList.begin();
	for (; it != m_edgeList.end(); ++it)
	{
		if (vertIdx == it->getA()){
			itListA.push_back(it);
		} else if (vertIdx == it->getB()) {
			itListB.push_back(it);
		}
	}
	newTriangle->setTriVertexs(edge.getA(), vertIdx, edge.getB());
	// �������������ε����ڹ�ϵ
	edge.getPreTriangle()->setNeighbor(edge, newTriangle);
	newTriangle->setNeighbor(vertIdx, edge.getPreTriangle());

	if (itListA.empty() && itListB.empty()) {	// vertIdx���ڲ�ǰ����
		m_vertexList.push_back(vertIdx);
		m_bPointUsed[vertIdx] = FRONTPOINT;
		CFrontEdge a(edge.getA(), vertIdx), b(vertIdx, edge.getB());
		a.setPreTriangle(newTriangle);
		b.setPreTriangle(newTriangle);
		m_edgeList.push_back(a);
		m_edgeList.push_back(b);
		m_edgeList.remove(edge);
	} else {	// vertIdx�ڲ�ǰ����
		list<IT>::iterator itt = itListA.begin();
		int iGlue = 0;
		int iPt = -1;
		for (; itt != itListA.end(); ++itt)
		{
			CFrontEdge e = *(*itt);
			if (e.getB() == edge.getA()){	// AV == e'vb
				e.getPreTriangle()->setNeighbor(e, newTriangle);
				newTriangle->setNeighbor(edge.getB(), e.getPreTriangle());

				m_edgeList.remove(e);
				m_vertexList.remove(edge.getA());
				m_bPointUsed[edge.getA()] = INPOINT;
				iGlue++;
				break;
			} else if (e.getB() == edge.getB()){	// VB ==e'vb
				e.getPreTriangle()->setNeighbor(e, newTriangle);
				newTriangle->setNeighbor(edge.getA(), e.getPreTriangle());

				m_edgeList.remove(e);
				m_vertexList.remove(edge.getB());
				m_bPointUsed[edge.getB()] = INPOINT;
				iGlue++;
				break;
			}
		}
		for (itt = itListB.begin(); itt != itListB.end(); ++itt){
			CFrontEdge e = *(*itt);
			if (e.getA() == edge.getA()){	// AV == e'av
				e.getPreTriangle()->setNeighbor(e, newTriangle);
				newTriangle->setNeighbor(edge.getB(), e.getPreTriangle());

				m_edgeList.remove(e);
				m_vertexList.remove(edge.getA());
				m_bPointUsed[edge.getA()] = INPOINT;
				iGlue++;
				break;
			} else if (e.getA() == edge.getB()){	// VB ==e'av
				e.getPreTriangle()->setNeighbor(e, newTriangle);
				newTriangle->setNeighbor(edge.getA(), e.getPreTriangle());

				m_edgeList.remove(e);
				m_vertexList.remove(edge.getB());
				m_bPointUsed[edge.getB()] = INPOINT;
				iGlue++;
				break;
			}
		}
		// TODO ͬʱGlue��ҲҪ����������vertIdx�����ڵ�
		if ((itListA.size() +itListB.size()) == 2 && iGlue == 2){
			m_vertexList.remove(vertIdx);
			m_bPointUsed[vertIdx] = INPOINT;
		}
		// TODO δ�����Ҫ������±�
		m_edgeList.remove(edge);
	}
	return true;
}

CMesh::CMesh(PointSet* ps)
	: m_ps(ps)
{
	ps->constructKdTree();
	m_bPointUsed.resize(m_ps->m_pointN);
	for (int i = 0; i < m_ps->m_pointN ; i++)
	{
		m_bPointUsed[i] = UNUSEDPOINT;
	}
}

CMesh::~CMesh(void)
{
	for (unsigned int i = 0; i < m_faceVects.size() ; i++)
	{
		delete m_faceVects[i];
	}
}

void CMesh::writeToFile(char* pFileName)
{
	if (pFileName != NULL)
	{
		ofstream plyFile(pFileName);
		if (plyFile){
			float** points = m_ps->m_point;
			plyFile<<m_ps->m_pointN<<endl;
			for (int i = 0; i < m_ps->m_pointN ; i++)	{
				plyFile<<points[i][0]<<" "<<points[i][1]<<" "<<points[i][2]<<endl;
			}
			std::vector<CTriangle*>::iterator it;
			int j = 0;
			for (it = m_faceVects.begin(); it != m_faceVects.end(); ++it, j++)
			{
				plyFile<<j<<" "<<(*it)->getA()<<" "<<(*it)->getB()<<" "<<(*it)->getC()<<endl;
			}
			plyFile.close();
		}
	}
}

int CMesh::getFirstUnusedPt( int start /*= 0*/ )
{
	for (int i = start; i < m_ps->m_pointN ; i++)
	{
		if (m_bPointUsed[i] == UNUSEDPOINT)
			return i;
	}
	for (int i = 0; i < start; i++)
	{
		if (m_bPointUsed[i] == UNUSEDPOINT)
			return i;
	}
	return -1;	// ȫ���Ѿ�ʹ����
}
//==================CBPAMESH====================

inline float veccos(float* a, float* b){
	return (a[0]*b[0]+a[1]*b[1]+a[2]*b[2]) / (sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]) * sqrt(b[0]*b[0]+b[1]*b[1]+b[2]*b[2]));
}

inline void vecCross(float* ret, float* a, float* b){
	ret[0] = a[1]*b[2] - a[2]*b[1];
	ret[1] = a[2]*b[0] - a[0]*b[2];
	ret[2] = a[0]*b[1] - a[1]*b[0];
}

/* ����ֵΪ 0->4 ��С����
	center��Բ��(������ת����е�)��Cijo��ǰһ�����ģ�Cko����һ������
*/
float CBpaMesh::getArc( float* center, float* Cijo, float* Cko)
{
	float ccot[3];	// ����������ķ�ʸ
	float co[3], ck[3];
	co[0] = Cijo[0] - center[0];
	co[1] = Cijo[1] - center[1];
	co[2] = Cijo[2] - center[2];
	ck[0] = Cko[0] - center[0];
	ck[1] = Cko[1] - center[1];
	ck[2] = Cko[2] - center[2];

	vecCross(ccot, co, ck);
	float ct[3];	// ��ֱ��co(center->Cijo)������
	vecCross(ct, ccot, co);
	float cosco = veccos(co, ck);	// co �� ck ��������������
	float cosct = veccos(ct, ck);// ct �� ck ��������������
//	if (cosco > 0){
		if (cosct > 0)
			return 1.0f - cosco;		// 0->1
		else
			return 3.0f + cosco;		// 3->4
// 	}else{
// 		if (cosct > 0)
// 			return 1.0f - cosco;		// 1->2
// 		else
// 			return 3.0f + cosco;		// 2->3
// 	}
}

bool CBpaMesh::findSeedTriange( CTriangle& face, float ballCenter[3] )
{
	// TODO ��������ʱ��Ҫ���Ҽ�������
	int idxA = -1;
	ANNidxArray nnIdx = new ANNidx[m_K+1];
	ANNdistArray dists = new ANNdist[m_K+1];
	ANNkd_tree* tree = m_ps->m_kdTree;
	int ballNeigIdx[4];

	while ((idxA = getFirstUnusedPt(idxA+1))!=-1) // �ҵ���һ����A
	{
		ANNpoint queryA = m_ps->m_point[idxA];
		tree->annkSearch(queryA, m_K+1, nnIdx, dists, 1e-6); // ��A�������
		bool bFound = false;

		for (int i = 1; i < m_K+1 && !bFound; i++)
		{
			if (UNUSEDPOINT != m_bPointUsed[nnIdx[i]]) continue;

			int idxB = nnIdx[i];	// �ҵ��ڶ�����B
			int j = i+1;
			while (j++ < m_K+1 && !bFound){
				int idxC = nnIdx[j];	// �ҵ���������C
				if (UNUSEDPOINT != m_bPointUsed[idxC]) continue;

				face.setTriVertexs(idxA, idxB, idxC);
				if (m_ps->checkConsistence(idxA, idxB, idxC))
					face.setTriVertexs(idxA, idxC, idxB);

				float center[3];
				if (!getBallCenter(center, idxA, face.getB(), face.getC(), m_ballR)) continue;	// ��������center

				tree->annkSearch(center, 4, ballNeigIdx, dists, 1e-6);		// �����ĵ������ֻ��Ҫ4���㼴��
				bool valid = true;
				for (int t = 0; t < 4 ; t++)
				{
					int idx = ballNeigIdx[t];
					if (idx != idxA && idx != idxB && idx != idxC){
						float* pt = m_ps->m_point[idx];
						float distance2 = (center[0]-pt[0])*(center[0]-pt[0])+
														(center[1]-pt[1])*(center[1]-pt[1])+
														(center[2]-pt[2])*(center[2]-pt[2]);
						if (distance2 < m_ballR*m_ballR){		// �㵽���ĵľ��� < ��뾶�������ڰ��������㣬����������
							valid = false;
							break;
						}
					}
				}
				if (valid){
					ballCenter[0] = center[0];
					ballCenter[1] = center[1];
					ballCenter[2] = center[2];
					m_bPointUsed[idxA] = FRONTPOINT;
					m_bPointUsed[idxB] = FRONTPOINT;
					m_bPointUsed[idxC] = FRONTPOINT;
					delete[] nnIdx;
					delete[] dists;
					return true;
				}
			}
		}
	}
	delete[] nnIdx;
	delete[] dists;
	return false;
}

CBpaMesh::CBpaMesh(PointSet* ps, int k)
	: CMesh(ps)
{
	m_K = k;
}

/* �����Ƿ�������ģ�trueΪ��ã�
	center Ϊ������ģ������
	ptA��ptB��ptCΪ���ϵ����� �����ABC
	normal[3] ΪABC����ķ�ʸ��ƽ��ֵ
	ballRΪ�����뾶
*/
bool CBpaMesh::getBallCenter( float center[3], float* ptA, float* ptB, float* ptC, float normal[3], float ballR )
{
	float x1 = ptB[0] - ptA[0];
	float x2 = ptC[0] - ptA[0];

	float y1 = ptB[1] - ptA[1];
	float y2 = ptC[1] - ptA[1];

	float z1 = ptB[2] - ptA[2];
	float z2 = ptC[2] - ptA[2];

	float pi, pj, pk; // ��ABC�ķ�ʸ
	pi=y1*z2-z1*y2;
	pj=z1*x2-x1*z2;
	pk=x1*y2-y1*x2;

	if((pi==0)&&(pj==0)&&(pk==0)){
		return false;	// ���㹲��
	}

	if((normal[0]*pi+normal[1]*pj + normal[3]*pk) < 0){
		pi = -pi;
		pj = -pj;
		pk = -pk;
	}

	float dMx = (ptA[0]+ptB[0])/2,	// AB ���е�M
			 dMy = (ptA[1]+ptB[1])/2,
			 dMz = (ptA[2]+ptB[2])/2;
	float dMi = pj*z1-pk*y1,	// AB ���д��߷���
			 dMj = pk*x1-pi*z1,
			 dMk = pi*y1-pj*x1;
	float dNx = (ptA[0]+ptC[0])/2,	// AC ���е�M
			 dNy = (ptA[1]+ptC[1])/2,
			 dNz = (ptA[2]+ptC[2])/2;
	float dNi = pj*z2-pk*y2,// AC ���д��߷���
			 dNj = pk*x2-pi*z2,
			 dNk = pi*y2-pj*x2;

	float ds = 1.0f / sqrt(pi*pi + pj*pj + pk*pk);
	pi *= ds;	// �淨ʸ��һ��
	pj *= ds;
	pk *= ds;

	assert(dMi != 0);
	float tm, tn;
	tn  = ((dMy-dNy) * dMi + dMj * (dNx-dMx)) / (dNj*dMi - dMj*dNi);
	tm = (dNx + dNi*tn - dMx) / dMi;
	float ox = dMx + dMi*tm,	//������ABC�����ԲԲ��
			 oy = dMy + dMj*tm,
			 oz = dMz + dMk*tm;
	float dR2 = ( (ox-ptA[0])*(ox-ptA[0]) +	// ���Բ�뾶��ƽ��
							(oy-ptA[1])*(oy-ptA[1]) +
							(oz-ptA[2])*(oz-ptA[2])  );
	float h = ballR*ballR - dR2;
	if (h < 0) return false;
	else if (h == 0){
		center[0] = ox;
		center[1] = oy;
		center[2] = oz;
	}else{
		h = sqrt(h);
		center[0] = ox + h*pi;
		center[1] = oy + h*pj;
		center[2] = oz + h*pk;
	}
	return true;
}
/* �����Ƿ�������ģ�trueΪ��ã�
	center Ϊ������ģ������
	idxA, idxB, idxC Ϊ���ϵ�����ABC������
	ballRΪ��İ뾶
*/
bool CBpaMesh::getBallCenter( float center[3], int idxA, int idxB, int idxC, float ballR )
{
	float** points = m_ps->m_point;
	float* ptA = points[idxA],
			* ptB = points[idxB],
			* ptC = points[idxC];
	float (*normal)[3] = m_ps->m_normal;
	float* nmA = normal[idxA],
			* nmB = normal[idxB],
			*nmC = normal[idxC];
	float norm[3];
	norm[0] = (nmA[0] + nmB[0] + nmC[0]);
	norm[1] = (nmA[1] + nmB[1] + nmC[1]);
	norm[2] = (nmA[2] + nmB[2] + nmC[2]);
	return getBallCenter(center, ptA, ptB, ptC, norm, ballR);
}

void CBpaMesh::start()
{
	CTriangle face;
	float ballCnt[3];
	while (true)
	{
		std::list<CFrontEdge>::iterator itEdge;
		int vertIdx;
		while (m_front.getActiveEdge(itEdge))
		{
			CEdge edge = *itEdge;
			if (itEdge->getStatus() == ACTIVE)
			{
				// TODO Ҫ�����޸�face	ȡ��ǰ�ߵ�preTriangle
				CTriangle* pFace = itEdge->getPreTriangle();
				int idPrec = -1;
				if (NULL != pFace){
					idPrec = pFace->getA();
					if (edge.getA() == idPrec || edge.getB() == idPrec){
						idPrec = pFace->getB();
						if (edge.getA() == idPrec || edge.getB() == idPrec)
							idPrec = pFace->getC();
					}
				}

				bool pivoted = ballPivotT(*itEdge, vertIdx, idPrec, ballCnt);
				if (pivoted){
					CTriangle* newTri = new CTriangle;
					if (m_front.join(*itEdge, vertIdx, m_ps, ballCnt, newTri)){
						m_bPointUsed[vertIdx] = FRONTPOINT;
						m_faceVects.push_back(newTri);
					} else {
						delete newTri;
					}
				} else {
					itEdge->setBoundary();
				}
			}
		}

		float center[3];
		if (findSeedTriange(face, center)){
			CTriangle* pFace = new CTriangle(face);
			m_faceVects.push_back(pFace);

			CFrontEdge a(face.getA(), face.getB());
			a.setCenter(center);
			a.setPreTriangle(pFace);
			m_front.addEdge(a);

			CFrontEdge b(face.getB(), face.getC());
			b.setCenter(center);
			b.setPreTriangle(pFace);
			m_front.addEdge(b);

			CFrontEdge c(face.getC(), face.getA());
			c.setCenter(center);
			c.setPreTriangle(pFace);
			m_front.addEdge(c);
		} else {
			return ;
		}
	}
}

bool CBpaMesh::ballPivotT( CFrontEdge& fontEdge, int& vertIdx, int idPrec ,float ballCnt[3] )
{
	int idxA = fontEdge.getA();
	int idxB = fontEdge.getB();
	ANNpoint PtA = m_ps->m_point[idxA];
	ANNpoint PtB = m_ps->m_point[idxB];

	float midAB[3];
	midAB[0] = (PtA[0] + PtB[0]) / 2;
	midAB[1] = (PtA[1] + PtB[1]) / 2;
	midAB[2] = (PtA[2] + PtB[2]) / 2;
	// TODO:: ����ȷ���������ٵĵ㣿
	ANNidxArray nnIdx = new ANNidx[m_K+1];
	ANNdistArray dists = new ANNdist[m_K+1];
	// TODO���˴�Ӧ�ò��Ұ뾶Ϊ r ������
	int pts = m_ps->m_kdTree->annkFRSearch(midAB, m_ballR*m_ballR, m_K+1, nnIdx);		// ����ʵ�ʵĵ���
	//	m_ps->m_kdTree->annkSearch(midAB, m_K+1, nnIdx, dists, 0.00000001);
	delete[] dists;

	float* preBallCenter  = fontEdge.m_ballCenter;
	float ballCenter[3];
	float min_arc = 4.0;
	float arc;
	int better = -1;
	for (int i = 0; i < m_K ; i++)		// m_K -> pts
	{
		if (nnIdx[i+1] == idxA || nnIdx[i+1] == idxB || nnIdx[i+1] == idPrec/* || INPOINT ==m_bPointUsed[ nnIdx[i+1] ]*/)
			continue;
		if (true == getBallCenter(ballCenter, idxA, idxB, nnIdx[i+1], m_ballR)){
			arc = getArc(midAB, preBallCenter, ballCenter);
			if (min_arc > arc){
				// TODO����Ҫ�����±ߵ����ģ�������Ҫ��������
				min_arc = arc;	// TODO : ѡ�񻡶���С��
				better = nnIdx[i+1];
				ballCnt[0] = ballCenter[0];
				ballCnt[1] = ballCenter[1];
				ballCnt[2] = ballCenter[2];
			}
		}
	}
	delete[] nnIdx;
	if (better == -1){
		return false;
	}
	vertIdx = better;
	return true;
}

bool CBpaMesh::ballPivot(CFrontEdge& fontEdge, int& vertIdx, int idPrec){
	int better = -1;
	int idxA = fontEdge.getA();
	int idxB = fontEdge.getB();
	float** points = m_ps->m_point;
	vect3d n(points[idxB][0] - points[idxA][0], points[idxB][1] - points[idxA][1],points[idxB][2] - points[idxA][2]);
	vect3d m((points[idxB][0] - points[idxA][0])/2,
		(points[idxB][1] - points[idxA][1])/2,
		(points[idxB][2] - points[idxA][2])/2);

	return true;
}

//=========================CIPDMESH=========================

float CIPDMesh::getAngel( const int& idxPre, const int& idxO, const int& idxNext )
{
	float* ptPre = m_ps->m_point[idxPre],
		* ptO = m_ps->m_point[idxO],
		* ptNext = m_ps->m_point[idxNext];
	vect3f op(ptPre[0]-ptO[0], ptPre[1]-ptO[1], ptPre[2]-ptO[2]);
	vect3f on(ptNext[0]-ptO[0], ptNext[1]-ptO[1], ptNext[2]-ptO[2]);

	op.unit();
	on.unit();

	float opDoton = op*on;
	return 1.0f-opDoton;
}

float CIPDMesh::getMinTriAngel( const int& idxA, const int& idxB, const int& idxC )
{
	float angelA = getAngel(idxC, idxA, idxB),
			 angelB = getAngel(idxA, idxB, idxC),
			 angelC = getAngel(idxB, idxC, idxA);
	float minAB = min(angelA, angelB);
	return min(minAB, angelC);
}

float CIPDMesh::getTriArea( const int& idxA, const int& idxB, const int& idxC )
{
	float* ptA = m_ps->m_point[idxA],
			* ptB = m_ps->m_point[idxB],
			* ptC = m_ps->m_point[idxC];

	float ab = sqrt( (ptA[0]-ptB[0])*(ptA[0]-ptB[0]) + (ptA[1]-ptB[1])*(ptA[1]-ptB[1]) + (ptA[2]-ptB[2])*(ptA[2]-ptB[2]) ),
			 bc = sqrt( (ptC[0]-ptB[0])*(ptC[0]-ptB[0]) + (ptC[1]-ptB[1])*(ptC[1]-ptB[1]) + (ptC[2]-ptB[2])*(ptC[2]-ptB[2]) ),
			 ac = sqrt( (ptA[0]-ptC[0])*(ptA[0]-ptC[0]) + (ptA[1]-ptC[1])*(ptA[1]-ptC[1]) + (ptA[2]-ptC[2])*(ptA[2]-ptC[2]) );
	float p = (ab + bc + ac) / 2.0f;
	float area = sqrt( p*(p-ab)*(p-bc)*(p-ac) );
	return area;
}

/*
float CIPDMesh::getDihedralAngel( const CTriangle* pPreFace, const int& idxPre, const int& idx )
{
}*/

float CIPDMesh::getDihedralAngel( const CFrontEdge& frontEdge, const int& idx )
{
	int idxPre = frontEdge.getPreTriangle()->getVertex(frontEdge);
	if (idxPre != -1){
		return getDihedralAngel(idxPre, frontEdge.getA(), frontEdge.getB(), idx);
	}
	return 0;		//�������غ�
}

float CIPDMesh::getDihedralAngel( const int& idxPre, const int& idxA, const int& idxB, const int& idxNext )
{
	float* ptP = m_ps->m_point[idxPre],
			* ptA = m_ps->m_point[idxA],
			* ptB = m_ps->m_point[idxB],
			* ptN = m_ps->m_point[idxNext];
	vect3f AB( ptB[0]-ptA[0], ptB[1]-ptA[1], ptB[2]-ptA[2] ),
				AP( ptP[0]-ptA[0], ptP[1]-ptA[1], ptP[2]-ptA[2] ),
				AN( ptN[0]-ptA[0], ptN[1]-ptA[1], ptN[2]-ptA[2] );
	AN.cross(AB);
	AB.cross(AP);
	AN.unit();
	AB.unit();

	float angel = 1 - AB*AN;
	return angel;
}

int CIPDMesh::getMaxZIndex()
{
	float** pts = m_ps->m_point;
	int ptN = m_ps->m_pointN;
	float maxZ = pts[0][2];
	int idxZ = 0;
	for (int i = 1; i < ptN; i++)
	{
		if (maxZ < pts[i][2]){
			maxZ = pts[i][2];
			idxZ = i;
		}
	}
	return idxZ;
}

bool CIPDMesh::findSeedTriangle( CTriangle& face, int K )
{
	int seedA = getMaxZIndex();
	float* ptA = m_ps->m_point[seedA];
	ANNidxArray nnidxA = new ANNidx[2];
	ANNdistArray distsA = new ANNdist[2];
	m_ps->m_kdTree->annkSearch(ptA, 2, nnidxA, distsA);
	delete[] distsA;
	int seedB = nnidxA[1];
	delete[] nnidxA;

	float* ptB = m_ps->m_point[seedB];
	float ptMid[3] = { (ptA[0]+ptB[0])/2.0f, (ptA[1]+ptB[1])/2.0f, (ptA[2]+ptB[2])/2.0f };

	ANNidxArray nnidx = new ANNidx[K+1];
	ANNdistArray dists = new ANNdist[K+1];
	m_ps->m_kdTree->annkSearch(ptMid, K+1, nnidx, dists);
	delete[] dists;

	float minLen = FLT_MAX;
	int seedC = -1;
	for (int i = 1; i < K+1; i++)
	{
		int idxC = nnidx[i];
		if (idxC == seedA || idxC == seedB) continue;

		float minAngel = getMinTriAngel(seedA, seedB, idxC);
		if (minAngel > 0.2)	// �������ڽ���СΪ30��
		{
			float len = getDists(idxC, seedA, seedB);	// ȡ��C��seedA��seedB�γɵ�����֮����С
			if (len < minLen){
				minLen = len;
				seedC = idxC;
			}
		}
	}
	delete[] nnidx;

	if (seedC == -1)	return false;
	//TODO �˴���Ҫ����
	face.setTriVertexs(seedA, seedB, seedC);
	setVertexUsed(seedA);
	setVertexUsed(seedB);
	setVertexUsed(seedC);
	return true;
}

bool CIPDMesh::findSeedTriangle2( CTriangle*& pFace, int K )
{
	int seedA = -1;
	ANNidxArray nnidx = new ANNidx[K+1];
	ANNdistArray dists = new ANNdist[K+1];

	while ( (seedA = getFirstUnusedPt(seedA+1)) != -1 ){
		float* ptA = m_ps->m_point[seedA];
		m_ps->m_kdTree->annkSearch(ptA, K+1, nnidx, dists);

		bool bFound = false;
		int seedC = -1;
		for (int i = 1; i < K+1 ; i++)
		{
			int seedB = nnidx[i];
			if (UNUSEDPOINT != m_bPointUsed[seedB]) continue;

			int j = i+1;
			float minLen = FLT_MAX;
			while (j < K+1)
			{
				int idxC = nnidx[j];
				if (UNUSEDPOINT != m_bPointUsed[idxC]) continue;
#define MINTRIANGEL30 0.13397459621556		// 1-cos(30')
				float minAngel = getMinTriAngel(seedA, seedB, idxC);
				if (minAngel > MINTRIANGEL30)	// �������ڽ���СΪ30��
				{
					bFound = true;
					float len = getDists(idxC, seedA, seedB);	// ȡ��C��seedA��seedB�γɵ�����֮����С
					if (len < minLen){
						minLen = len;
						seedC = idxC;
					}
				}
				j++;
			}
			if(bFound){
				delete[] nnidx;
				delete[] dists;

				//TODO �˴���Ҫ������ķ�ʸ�Ƿ�һ��
				pFace = new CTriangle(seedA, seedB, seedC);
				//pFace->setTriVertexs(seedA, seedB, seedC);
				setVertexUsed(seedA);
				m_front.addVertex(seedA);
				setVertexUsed(seedB);
				m_front.addVertex(seedB);
				setVertexUsed(seedC);
				m_front.addVertex(seedC);
				return true;
			}
		}
	}
	delete[] nnidx;
	delete[] dists;
	return false;
}

bool CIPDMesh::getBestPt( CFrontEdge& frontEdge, int& bestIdx, const int& K )
{
	int idxA = frontEdge.getA(), idxB = frontEdge.getB();
	float* ptA = m_ps->m_point[idxA],
			* ptB = m_ps->m_point[idxB];
	float midAB[3] = { (ptA[0]+ptB[0])/2.0f, (ptA[1]+ptB[1])/2.0f, (ptA[2]+ptB[2])/2.0f };

	ANNidxArray nnIdx = new ANNidx[K+1];
	ANNdistArray dists = new ANNdist[K+1];

	m_ps->m_kdTree->annkSearch(midAB, K+1, nnIdx, dists);
	delete[] dists;
	float minWeight = FLT_MAX;
	int idxC = -1;

//#define MINTRIANGEL  0.1339745962	// 1-cos(30') ����������С�ڽ�Ҫ����30'
#define MINTRIANGEL  0.0603073792	// 1-cos(20') ����������С�ڽ�Ҫ����20'
#define MAXDIHEDRALANGEL 1.8660254 // 1-cos(150') ����������Ϊ150'
	int idxPre = frontEdge.getPreTriangle()->getVertex(frontEdge);
	for (int i=1; i < K+1 ; i++)
	{
		int idxCur = nnIdx[i];
		//TODO  �˴�Ӧ�ø���Ϊ�ڲ�����Թ�����ǰ���ϵĵ㲻�Թ�
		if (idxCur == idxA || idxCur == idxB || idxCur == idxPre || INPOINT == m_bPointUsed[idxCur]) continue;

		//TODO ����Ҫʹ�ö�������ж�
		float minAngel = getMinTriAngel(idxA, idxB, idxCur);	// Ҫ����С�ڽ�Ҫ����ָ������
		//float dihedralAngel = getDihedralAngel(frontEdge, idxCur);// Ҫ������ҪС��ָ������
		//if (minAngel > MINTRIANGEL && dihedralAngel < MAXDIHEDRALANGEL){
		if (minAngel > MINTRIANGEL){
			float weight = getWeightTri(frontEdge, idxCur, 2.0);
			if (minWeight > weight){
				minWeight = weight;
				idxC = idxCur;
			}
		}
	}
	delete[] nnIdx;

	if (idxC == -1) return false;
	else {
		bestIdx = idxC;
		return true;
	}
}

void CIPDMesh::start(){
	CTriangle* pFace = NULL;
	std::list<CFrontEdge>::iterator itEdge;
	int vertIdx;
	while (true){
		while (m_front.getActiveEdge(itEdge))
		{
			CEdge edge = *itEdge;
			bool pivoted = getBestPt(*itEdge, vertIdx, 15);
			if (pivoted){
				CTriangle* newTri = new CTriangle;
				m_front.join(*itEdge, vertIdx, m_bPointUsed, newTri);
				m_faceVects.push_back(newTri);
			}else{
				itEdge->setBoundary();
			}
		}

		if (findSeedTriangle2(pFace)){
			m_faceVects.push_back(pFace);

			CFrontEdge a(pFace->getA(), pFace->getB()),
								  b(pFace->getB(), pFace->getC()),
								  c(pFace->getC(), pFace->getA());
			a.setPreTriangle(pFace);
			b.setPreTriangle(pFace);
			c.setPreTriangle(pFace);
			m_front.addEdge(a);
			m_front.addEdge(b);
			m_front.addEdge(c);
		}else{
			return ;
		}
	}
}