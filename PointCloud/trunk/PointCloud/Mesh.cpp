#include "StdAfx.h"
#include "Mesh.h"
#include <fstream>
#include <iostream>
#include "PointCloudView.h"
using namespace std;

#ifdef _DEBUG
#define new DEBUG_NEW
#define _CRTDBG_MAP_ALLOC
#endif
//=====================AdvancingFront=========================//
// TODO �˴���Ҫ����һ���µ������棬�����������õ�ǰ�¼���ı����õ�ָ��
bool AdvancingFront::join(CFrontEdge& edge, int vertIdx, PointSet* ps, float ballCnt[3], CTriangle* newTriangle){
	bool used = IsVertexUsed(vertIdx);
	list<CFrontEdge>::iterator it, toGlue[2];
	int nbFound = 0;
	bool found = false;
	if (used){
		it = m_frontEdgeList.begin();	//+�����еĲ�ǰ�߼��в��ҵ�vertIdx+
		while (it != m_frontEdgeList.end() && nbFound < 2){
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

	m_frontEdgeList.remove(edge);

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
				m_frontEdgeList.erase(it);
				m_vertexList.remove(idxA);
			}else if ( (*it) == CEdge(b.getB(), b.getA()) ){
				newTriangle->setNeighbor(idxA, it->getPreTriangle());
				it->getPreTriangle()->setNeighbor(*it, newTriangle);
				insertB = false;
				m_frontEdgeList.erase(it);
				m_vertexList.remove(idxB);
			}
		}
	}

	if (insertA)
		m_frontEdgeList.push_back(a);
	if (insertB)
		m_frontEdgeList.push_back(b);
	return true;
}

bool AdvancingFront::join( CFrontEdge& edge, int vertIdx, vector<short>& m_bPointUsed, CTriangle* newTriangle )
{
// 	assert(edge.getA() != vertIdx);
// 	assert(edge.getB() != vertIdx);

	typedef list<CFrontEdge>::iterator IT;
	IT itA = m_frontEdgeList.end(), itB = itA;
	int iVertIdxOnFront = 0;	// ���ŵ����ӵĲ�ǰ����
	//! ����
	for (IT it = m_frontEdgeList.begin(); it != m_frontEdgeList.end(); ++it)
	{
		if (it->getA() == vertIdx || it->getB()==vertIdx){
			iVertIdxOnFront++;	// ���ŵ��ڲ�ǰ����
			if (it->getB() == edge.getA() )
				itA = it;		// ���ںϲ���Glue Edge(A, V)
			else if (it->getA() == edge.getB() )
				itB = it;		// ���ںϲ���Glue Edge(V, B)
		}
	}
	// �������������ε����ڹ�ϵ
	newTriangle->setTriVertexs(edge.getA(), vertIdx, edge.getB());
	edge.getPreTriangle()->setNeighbor(edge.getOppPoint(), newTriangle);
	newTriangle->setNeighbor(vertIdx, edge.getPreTriangle());

	if (itA == itB) {	// vertIdx���ڲ�ǰ����|vertIdx�ڲ�ǰ���ϵ�û��Ҫ�ϲ��ı�
		if (iVertIdxOnFront == 0){	// ���ڲ�ǰ���ϲ���Ҫ���
//			m_vertexList.push_back(vertIdx);
			m_bPointUsed[vertIdx] = FRONTPOINT;
		} /*
		else {
					m_bPointUsed[vertIdx] += 2;
				}*/

		CFrontEdge a(edge.getA(), vertIdx, edge.getB(), newTriangle),
			b(vertIdx, edge.getB(), edge.getA(), newTriangle);
		m_frontEdgeList.push_back(a);
		m_frontEdgeList.push_back(b);
		m_frontEdgeList.remove(edge);
	} else {	// vertIdx�ڲ�ǰ����
		if (itA != m_frontEdgeList.end()){	// ��VA��Ҫ�ϲ�
			newTriangle->setNeighbor(vertIdx, itA->getPreTriangle());
			itA->getPreTriangle()->setNeighbor(itA->getOppPoint(), newTriangle);

			m_frontEdgeList.remove(*itA);
//			m_vertexList.remove(edge.getA());
			m_bPointUsed[edge.getA()] = INPOINT;	// A Ϊ�ڵ�
		} else {
			CFrontEdge a(edge.getA(), vertIdx, edge.getB(), newTriangle);
			m_frontEdgeList.push_back(a);
		}

		if (itB != m_frontEdgeList.end()){	// ��BV��Ҫ�ϲ�
			newTriangle->setNeighbor(vertIdx, itB->getPreTriangle());
			itB->getPreTriangle()->setNeighbor(itB->getOppPoint(), newTriangle);

			m_frontEdgeList.remove(*itB);
//			m_vertexList.remove(edge.getB());
			m_bPointUsed[edge.getB()] = INPOINT;		// B Ϊ�ڵ�
		} else {
			CFrontEdge b(vertIdx, edge.getB(), edge.getA(), newTriangle);
			m_frontEdgeList.push_back(b);
		}
		// TODO ͬʱ�ϲ������������Ӵ˵���߶�Ϊ2������vertIdx�����ڵ�
		if (iVertIdxOnFront == 2 && itA != m_frontEdgeList.end() && itB != m_frontEdgeList.end()){
//			m_vertexList.remove(vertIdx);
			m_bPointUsed[vertIdx] = INPOINT;
		}
		m_frontEdgeList.remove(edge);
	}
	return true;
}
//=====================CMesh========================//
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
	for (size_t i = 0; i < m_faceVects.size() ; i++)
	{
		delete m_faceVects[i];
		m_faceVects[i] = NULL;
	}
	m_faceVects.clear();
	m_bPointUsed.clear();
	m_ps = NULL;
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
//@ ������start,��start��һ����ʼ
int CMesh::getFirstUnusedPt( int start /*= 0*/ )
{
	for (int i = start+1; i < m_ps->m_pointN ; i++)
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

/*
void CMesh::faceNormal()
{
	int nFaces = m_faceVects.size();
	float** ps = m_ps->m_point;
	int iA, iB, iC;
	float* ptA, *ptB, *ptC;

	vector<CTriangle*>::iterator it = m_faceVects.begin();
	for (; it != m_faceVects.end(); ++it)
	{
			iA = (*it)->getA();
			iB = (*it)->getB();
			iC = (*it)->getC();
			ptA = ps[iA];
			ptB = ps[iB];
			ptC = ps[iC];
			vect3f AB(ptB[0]-ptA[0], ptB[1]-ptA[1], ptB[2]-ptA[2]),
				AC(ptC[0]-ptA[0], ptC[1]-ptA[1], ptC[2]-ptA[2]);
			AB.cross(AC);
			m_faceNorms.push_back(AB);
	}
}

void CMesh::filpFaceNorm()
{
	vector<vect3f>::iterator it = m_faceNorms.begin();
	for (; it != m_faceNorms.end(); ++it)
	{
		it->negative();
	}
}
*/

int CMesh::checkHoles()
{
	int hole = 0;
	vector<CTriangle*>::iterator it = m_faceVects.begin();
	for (; it!= m_faceVects.end(); ++it)
	{
		CTriangle* pFace = *it;
		CTriangle *pFA = NULL, *pFB = NULL, *pFC=NULL;
		pFA = pFace->iNeighbor(0);
		pFB = pFace->iNeighbor(1);
		pFC = pFace->iNeighbor(2);
		if (pFA == NULL || pFB == NULL || pFC==NULL)
			hole++;
	}
	return hole;
}

bool CMesh::repair()
{
	int ptN = m_ps->m_pointN;
	int* linkN = new int[ptN];	// �������������θ���
	int** link = new int*[ptN];	// ���������������������

	for (int i = 0; i < ptN ; i++){
		link[i] = NULL;
		linkN = 0;
	}

	// ͳ��ÿһ�����������������
	int triN = m_faceVects.size();
	for (int i = 0; i < triN ; i++){
		for (int j = 0; j < 3; j++) {
			int idxPt = m_faceVects[i]->vertex(j);
			int lN = linkN[idxPt];
			int* l_tmp = new int[lN+1];
			int* l_old = link[idxPt];
			for(int k = 0; k < lN; k++)
				l_tmp[k] = l_old[k];
			l_tmp[lN] = i;
			if(lN != 0) delete[] l_old;
			link[idxPt] = l_tmp;
			linkN[idxPt] ++;
		}
	}

	delete[] link;
	delete[] linkN;
	return true;
}

//==================CBPAMESH====================//

inline float veccos(float* a, float* b){
	return (a[0]*b[0]+a[1]*b[1]+a[2]*b[2]) / sqrt((a[0]*a[0]+a[1]*a[1]+a[2]*a[2]) * (b[0]*b[0]+b[1]*b[1]+b[2]*b[2]));
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
	//float cosct = veccos(ct, ck);// ct �� ck ��������������
	float cosct = ct[0]*ck[0] + ct[1]*ck[1]+ct[2]*ck[2];

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
	//TODO �˴����������ѭ�� ��δʹ�õĵ����ʱ,�������������,
	//TODO �������:�ж�idx�Ƿ�δʹ�õ����һ��
	while ((idxA = getFirstUnusedPt(idxA))!=-1) // �ҵ���һ����A
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
	float p = (ab + bc + ac) * 0.5f;
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
	int seedA = getFirstUnusedPt(-1), firstSeed = seedA;
	ANNidxArray nnidx = new ANNidx[K+1];
	ANNdistArray dists = new ANNdist[K+1];

	//TODO �˴����������ѭ�� ��δʹ�õĵ����ʱ,�������������,
	//TODO �������:�ж�idx�Ƿ�δʹ�õ����һ��
	while ( seedA  != -1 ){
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
				if (UNUSEDPOINT != m_bPointUsed[idxC]) {
					j++;
					continue;
				}
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
		seedA = getFirstUnusedPt(seedA);
		if (seedA == firstSeed)
			break;
	}
	delete[] nnidx;
	delete[] dists;
	return false;
}

bool CIPDMesh::getBestPt( FrontEdgeIterator& itEdge, int& bestIdx, const int& K )
{
	CFrontEdge frontEdge = *itEdge;
	int idxA = frontEdge.getA(), idxB = frontEdge.getB();

	float* ptA = m_ps->m_point[idxA],
			* ptB = m_ps->m_point[idxB];
	float midAB[3] = { (ptA[0]+ptB[0])/2.0f, (ptA[1]+ptB[1])/2.0f, (ptA[2]+ptB[2])/2.0f };

	ANNidxArray nnIdx = new ANNidx[K];
	ANNdistArray dists = new ANNdist[K];

	m_ps->m_kdTree->annkSearch(midAB, K, nnIdx, dists);
	delete[] dists;
	float minWeight = FLT_MAX;
	int idxC = -1;

//#define MINTRIANGEL  0.1339745962	// 1-cos(30') ����������С�ڽ�Ҫ����30'
//#define MINTRIANGEL  0.0603073792	// 1-cos(20') ����������С�ڽ�Ҫ����20'
#define MINTRIANGEL  0.09369221296335	// 1-cos(25') ����������С�ڽ�Ҫ����25'
#define MAXDIHEDRALANGEL 1.5 // 1-cos(150') ����������Ϊ150'
//#define MAXDIHEDRALANGEL 1.8660254 // 1-cos(150') ����������Ϊ150'
	int idxPre = frontEdge.getOppPoint();

	CPlane prePlane, nextPlane;
	int idxApre = -1, idxBnext = -1;
	getPreNextIdx(idxApre, idxBnext, itEdge);
	getPlanes(prePlane, nextPlane, idxA, idxB, idxPre, idxApre, idxBnext);

	for (int i=0; i < K ; i++)
	{
		int idxCur = nnIdx[i];
		if ( INPOINT == m_bPointUsed[idxCur] || idxCur == idxA || idxCur == idxB || idxCur == idxPre) continue;

		//TODO ����Ҫʹ�ö�������ж�
		//TODO ��ͬʱ����������ǰ����Ȩ��һ�������ѡ�񣿣���α�֤���ཻ����һ����ֻ���������������Σ�
		float minAngel = getMinTriAngel(idxA, idxB, idxCur);	// Ҫ����С�ڽ�Ҫ����ָ������
		float dihedralAngel = getDihedralAngel(frontEdge, idxCur);// Ҫ������ҪС��ָ������
		if (minAngel > MINTRIANGEL && dihedralAngel < MAXDIHEDRALANGEL && isBtwPlanes(prePlane, nextPlane, frontEdge, idxCur)){
		//if (minAngel > MINTRIANGEL){
		//	float weight = getWeightTri(frontEdge, idxCur, 4.0);
			// ����ʹ��
			//TODO: ���ڲ�ǰ���ϵĶ��� Ҫ����ע�⣬��Ҫ�ж����ӵ��Ƿ��ڵ�����
			float fPointUsedTimes = m_bPointUsed[idxCur];
			float weight = getWeightTri(frontEdge, idxCur, 8.0, dihedralAngel) ;//+ (float)m_bPointUsed[idxCur];
			if (FRONTPOINT == m_bPointUsed[idxCur])	weight -= 2;
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
//#define OUTFACEFILE
#ifdef OUTFACEFILE
	const char* facePath = "face.ply";
	ofstream faceFile(facePath);
	if (!faceFile) return ;
#endif

	FrontEdgeIterator itEdge;
	int vertIdx;
	while (true){
		while (m_front.getActiveEdge(itEdge))
		{
			CEdge edge = *itEdge;
			//bool pivoted = getBestPt(itEdge, vertIdx, 8);
			bool bFrontPoint = false;
			bool pivoted = getCandidatePoint(itEdge, vertIdx, bFrontPoint, 8);
			if (pivoted){
				CTriangle* newTri = new CTriangle;
				m_front.join(*itEdge, vertIdx, m_bPointUsed, newTri);
				newTri->calcNorm(m_ps);
				m_faceVects.push_back(newTri);
#ifdef OUTFACEFILE
				faceFile<< newTri->getA()<<" "<<newTri->getB()<<" "<<newTri->getC()<<"\n";
#endif
			}else{
				//itEdge->setBoundary();
				m_front.setBoundary(*itEdge);
			}
			pView->draw();
	//		Sleep(200);
		}

		CTriangle* pFace = NULL;
		if (findSeedTriangle2(pFace)){
			pFace->calcNorm(m_ps);
			m_faceVects.push_back(pFace);

			CFrontEdge a(pFace->getA(), pFace->getB(), pFace->getC(), pFace),
								  b(pFace->getB(), pFace->getC(), pFace->getA(), pFace),
								  c(pFace->getC(), pFace->getA(), pFace->getB(), pFace);
			m_front.addEdge(a);
			m_front.addEdge(b);
			m_front.addEdge(c);
		}else{
			return ;
		}
	}
#ifdef OUTFACEFILE
	faceFile.close();
#endif
}

bool CIPDMesh::isInPlay( int idx, int edgeA, int edgeB, int edgeC, int edgePre, int edgeNext )
{
	float* ptA = m_ps->m_point[edgeA],
		* ptB = m_ps->m_point[edgeB],
		* ptC = m_ps->m_point[edgeC];
	float oABC[3] = {
		(ptA[0]+ptB[0]+ptC[0])/3.0f,
		(ptA[1]+ptB[1]+ptC[1])/3.0f,
		(ptA[2]+ptB[2]+ptC[2])/3.0f	};

	return false;
}

int CIPDMesh::getPlanes( CPlane& prePlane, CPlane& nextPlane, int eA, int eB, int tC, int ePre, int eNext )
{
	float* ptA = m_ps->m_point[eA],
		* ptB = m_ps->m_point[eB],
		* ptC = m_ps->m_point[tC];
		float oABC[3] = {
		(ptA[0]+ptB[0]+ptC[0])/3.0f,
		(ptA[1]+ptB[1]+ptC[1])/3.0f,
		(ptA[2]+ptB[2]+ptC[2])/3.0f	};

	vect3f vAB( ptB[0] - ptA[0], ptB[1] - ptA[1], ptB[2] - ptA[2]),
		vAC( ptC[0] - ptA[0], ptC[1] - ptA[1], ptC[2] - ptA[2]);
	vAB.cross(vAC);	// ��ABC���ഹֱ
	vect3f nABC = vAB;	// ��ABC�ķ�����

	vect3f vOA( ptA[0] - oABC[0], ptA[1] - oABC[1], ptA[2] - oABC[2] );
	vAB.cross(vOA);	// ��OAn�ķ�����
	prePlane.setPlane(vAB, ptA);	// ��OAn
	if (ePre != -1){
		float* ptPre = m_ps->m_point[ePre];
		if (prePlane.iSameSide(ptB, ptPre) > 0){
			vect3f nAPn = nABC;
			vect3f AP( ptPre[0] - ptA[0], ptPre[1] - ptA[1], ptPre[2] - ptA[2] );
			nAPn.cross(AP);
			prePlane.setPlane(nAPn, ptA);// ƽ��APren
		}
	}

	vect3f vOB ( ptB[0] - oABC[0], ptB[1] - oABC[1], ptB[2] - oABC[2] );
	vOB.cross(nABC);	// ��OBn�ķ�����
	nextPlane.setPlane(vOB, ptB);
	if (eNext != -1){
		float* ptNext = m_ps->m_point[eNext];
		if (nextPlane.iSameSide(ptA, ptNext) > 0){
			vect3f BN( ptNext[0]-ptB[0], ptNext[1]-ptB[1], ptNext[2]-ptB[2] );
			BN.cross(nABC);
			nextPlane.setPlane(BN, ptB);
		}
	}
	return 2;
}

bool CIPDMesh::getCandidatePoint(FrontEdgeIterator& itEdge, int& bestIdx, bool& bFrontPoint, const int& K){
	int idxPre = -1, idxNext = -1;
	getPreNextIdx(idxPre, idxNext, itEdge);
	//TODO δ�жϵ��ڱߵ�ǰ�����Ǻ�
	int idxA = itEdge->getA(), idxB = itEdge->getB(), idxC = itEdge->getOppPoint();
	float* ptA = m_ps->m_point[idxA],
		* ptB = m_ps->m_point[idxB],
		* ptC = m_ps->m_point[idxC];
	float eAB[3] = Edge(ptA,ptB);
	float nABC[3], nABn[3];
	itEdge->getPreTriangle()->getNorm(nABC);
	eCross(nABn, eAB, nABC);
	CPlane planeABn(nABn, ptA);

	if (idxPre != -1){
		float* ptP = m_ps->m_point[idxPre];
		if (planeABn.iSameSide(ptP, ptC) <= 0)
		{
			float angelBAP = getAngel(idxPre, itEdge->getA(), itEdge->getB());
			if (angelBAP > COS30 && angelBAP < COS90){
				float angelABP = getAngel(itEdge->getA(), itEdge->getB(), idxPre);
				float angelAPB = getAngel(itEdge->getA(), idxPre, itEdge->getB());
				angelABP = min(angelABP, angelBAP);
				angelAPB = min(angelAPB, angelABP);
				if (angelAPB > COS20){
					bestIdx = idxPre;
					bFrontPoint = true;
					return true;
				}
			}
		}
	}
	if (idxNext != -1){
		float* ptN = m_ps->m_point[idxNext];
		if (planeABn.iSameSide(ptN, ptC) <= 0){
			float angelABN = getAngel(itEdge->getA(), itEdge->getB(), idxNext);
			if (angelABN > COS30 && angelABN < COS90){
				float angelBAN = getAngel(itEdge->getB(), itEdge->getA(), idxNext);
				float angelANB = getAngel(itEdge->getA(), idxNext, itEdge->getB());
				angelBAN = min(angelBAN, angelABN);
				angelANB = min(angelANB, angelBAN);
				if (angelANB){
					bestIdx = idxNext;
					bFrontPoint = true;
					return true;
				}
			}
		}
	}

	float midAB[3] = { (ptA[0]+ptB[0])/2.0f, (ptA[1]+ptB[1])/2.0f, (ptA[2]+ptB[2])/2.0f };

	ANNidxArray nnIdx = new ANNidx[K];
	ANNdistArray dists = new ANNdist[K];
	m_ps->m_kdTree->annkSearch(midAB, K, nnIdx, dists);
	delete[] dists;

	int vBestIdx = -1;
	float minWeight = FLT_MAX;
	for (int i = 0; i < K; ++i){
		int idxCan = nnIdx[i];
		if (INPOINT == m_bPointUsed[idxCan] || idxA == idxCan || idxB == idxCan || idxC == idxCan) continue;

		float dihedralAngel = m_ps->getDihedralAngel(idxC, idxA, idxB, idxCan);
		if (dihedralAngel > COS90) continue;
		float minTriAngel = m_ps->getMinTriAngel(idxA, idxB, idxCan);
		if (FRONTPOINT == m_bPointUsed[idxCan] && minTriAngel > COS15){
			float weight = getIPDWeightTri(idxC, idxA, idxB, idxCan)-2;
			if (minWeight > weight){
				minWeight = weight;
				vBestIdx = idxCan;
			}
		} else if (UNUSEDPOINT == m_bPointUsed[idxCan] && minTriAngel >  COS25){
			float weight = getIPDWeightTri(idxC, idxA, idxB, idxCan);
			if (minWeight > weight){
				minWeight = weight;
				vBestIdx = idxCan;
			}
		}
	}
	delete[] nnIdx;
	if (vBestIdx != -1){
		bestIdx = vBestIdx;
		bFrontPoint = (FRONTPOINT == m_bPointUsed[vBestIdx]);
		return true;
	}
	return false;
}