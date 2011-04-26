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
// TODO 此处需要生成一个新的三角面，这样才能设置当前新加入的边设置的指针
bool AdvancingFront::join(CFrontEdge& edge, int vertIdx, PointSet* ps, float ballCnt[3], CTriangle* newTriangle){
	bool used = IsVertexUsed(vertIdx);
	list<CFrontEdge>::iterator it, toGlue[2];
	int nbFound = 0;
	bool found = false;
	if (used){
		it = m_frontEdgeList.begin();	//+在已有的波前边集中查找点vertIdx+
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
			edge.setBoundary();// 此处应为找到的点为非波前边上的点，但是已经被使用了（即：内部的点）
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
	// ：需要设置a，b的前一个球的球心,以及所在面的point
	// ：新边应该指向当前拥有其的面也就是其下一步形成的面
	a.setCenter(ballCnt);
	b.setCenter(ballCnt);
	a.setPreTriangle(newTriangle);
	b.setPreTriangle(newTriangle);
	// 设置相邻三角形的链接关系  共用边edge
	newTriangle->setNeighbor(vertIdx, edge.getPreTriangle());
	edge.getPreTriangle()->setNeighbor(edge, newTriangle);

	m_frontEdgeList.remove(edge);

	bool insertA = true, insertB = true;
	if (used){
		for (int i = 0; i < nbFound ; i++)
		{
			list<CFrontEdge>::iterator it = toGlue[i];
			if ( (*it) == CEdge(a.getB(), a.getA()) ){
				//  此处设置相邻面的指针
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
	int iVertIdxOnFront = 0;	// 最优点连接的波前边数
	//! 查找
	for (IT it = m_frontEdgeList.begin(); it != m_frontEdgeList.end(); ++it)
	{
		if (it->getA() == vertIdx || it->getB()==vertIdx){
			iVertIdxOnFront++;	// 最优点在波前边上
			if (it->getB() == edge.getA() )
				itA = it;		// 存在合并边Glue Edge(A, V)
			else if (it->getA() == edge.getB() )
				itB = it;		// 存在合并边Glue Edge(V, B)
		}
	}
	// 设置两个三角形的相邻关系
	newTriangle->setTriVertexs(edge.getA(), vertIdx, edge.getB());
	edge.getPreTriangle()->setNeighbor(edge.getOppPoint(), newTriangle);
	newTriangle->setNeighbor(vertIdx, edge.getPreTriangle());

	if (itA == itB) {	// vertIdx不在波前边上|vertIdx在波前边上但没有要合并的边
		if (iVertIdxOnFront == 0){	// 不在波前边上才需要添加
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
	} else {	// vertIdx在波前边上
		if (itA != m_frontEdgeList.end()){	// 边VA需要合并
			newTriangle->setNeighbor(vertIdx, itA->getPreTriangle());
			itA->getPreTriangle()->setNeighbor(itA->getOppPoint(), newTriangle);

			m_frontEdgeList.remove(*itA);
//			m_vertexList.remove(edge.getA());
			m_bPointUsed[edge.getA()] = INPOINT;	// A 为内点
		} else {
			CFrontEdge a(edge.getA(), vertIdx, edge.getB(), newTriangle);
			m_frontEdgeList.push_back(a);
		}

		if (itB != m_frontEdgeList.end()){	// 边BV需要合并
			newTriangle->setNeighbor(vertIdx, itB->getPreTriangle());
			itB->getPreTriangle()->setNeighbor(itB->getOppPoint(), newTriangle);

			m_frontEdgeList.remove(*itB);
//			m_vertexList.remove(edge.getB());
			m_bPointUsed[edge.getB()] = INPOINT;		// B 为内点
		} else {
			CFrontEdge b(vertIdx, edge.getB(), edge.getA(), newTriangle);
			m_frontEdgeList.push_back(b);
		}
		// TODO 同时合并两条并且连接此点的线段为2，这样vertIdx才是内点
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
//@ 不计算start,从start下一个开始
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
	return -1;	// 全部已经使用了
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
	int* linkN = new int[ptN];	// 顶点相邻三角形个数
	int** link = new int*[ptN];	// 顶点相邻三角形序号数组

	for (int i = 0; i < ptN ; i++){
		link[i] = NULL;
		linkN = 0;
	}

	// 统计每一个顶点的相邻三角形
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

/* 返回值为 0->4 由小到大
	center：圆心(滚动球转轴的中点)；Cijo：前一个球心；Cko：后一个球心
*/
float CBpaMesh::getArc( float* center, float* Cijo, float* Cko)
{
	float ccot[3];	// 三点所处面的法矢
	float co[3], ck[3];
	co[0] = Cijo[0] - center[0];
	co[1] = Cijo[1] - center[1];
	co[2] = Cijo[2] - center[2];
	ck[0] = Cko[0] - center[0];
	ck[1] = Cko[1] - center[1];
	ck[2] = Cko[2] - center[2];

	vecCross(ccot, co, ck);
	float ct[3];	// 垂直于co(center->Cijo)的向量
	vecCross(ct, ccot, co);
	float cosco = veccos(co, ck);	// co 和 ck 两个向量的余弦
	//float cosct = veccos(ct, ck);// ct 和 ck 两个向量的余弦
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
	// TODO 查找邻域时需要查找几个？？
	int idxA = -1;
	ANNidxArray nnIdx = new ANNidx[m_K+1];
	ANNdistArray dists = new ANNdist[m_K+1];
	ANNkd_tree* tree = m_ps->m_kdTree;
	int ballNeigIdx[4];
	//TODO 此处可能造成死循环 当未使用的点很少时,会出现这种问题,
	//TODO 解决方法:判断idx是否将未使用点计算一遍
	while ((idxA = getFirstUnusedPt(idxA))!=-1) // 找到第一个点A
	{
		ANNpoint queryA = m_ps->m_point[idxA];
		tree->annkSearch(queryA, m_K+1, nnIdx, dists, 1e-6); // 找A点的邻域
		bool bFound = false;

		for (int i = 1; i < m_K+1 && !bFound; i++)
		{
			if (UNUSEDPOINT != m_bPointUsed[nnIdx[i]]) continue;

			int idxB = nnIdx[i];	// 找到第二个点B
			int j = i+1;
			while (j++ < m_K+1 && !bFound){
				int idxC = nnIdx[j];	// 找到第三个点C
				if (UNUSEDPOINT != m_bPointUsed[idxC]) continue;

				face.setTriVertexs(idxA, idxB, idxC);
				if (m_ps->checkConsistence(idxA, idxB, idxC))
					face.setTriVertexs(idxA, idxC, idxB);

				float center[3];
				if (!getBallCenter(center, idxA, face.getB(), face.getC(), m_ballR)) continue;	// 计算球心center

				tree->annkSearch(center, 4, ballNeigIdx, dists, 1e-6);		// 找球心的最近点只需要4个点即可
				bool valid = true;
				for (int t = 0; t < 4 ; t++)
				{
					int idx = ballNeigIdx[t];
					if (idx != idxA && idx != idxB && idx != idxC){
						float* pt = m_ps->m_point[idx];
						float distance2 = (center[0]-pt[0])*(center[0]-pt[0])+
														(center[1]-pt[1])*(center[1]-pt[1])+
														(center[2]-pt[2])*(center[2]-pt[2]);
						if (distance2 < m_ballR*m_ballR){		// 点到球心的距离 < 球半径，即球内包含其他点，不满足条件
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

/* 返回是否求出球心（true为求得）
	center 为球的球心（输出）
	ptA、ptB、ptC为球上的三点 组成面ABC
	normal[3] 为ABC三点的法矢的平均值
	ballR为球的球半径
*/
bool CBpaMesh::getBallCenter( float center[3], float* ptA, float* ptB, float* ptC, float normal[3], float ballR )
{
	float x1 = ptB[0] - ptA[0];
	float x2 = ptC[0] - ptA[0];

	float y1 = ptB[1] - ptA[1];
	float y2 = ptC[1] - ptA[1];

	float z1 = ptB[2] - ptA[2];
	float z2 = ptC[2] - ptA[2];

	float pi, pj, pk; // 面ABC的法矢
	pi=y1*z2-z1*y2;
	pj=z1*x2-x1*z2;
	pk=x1*y2-y1*x2;

	if((pi==0)&&(pj==0)&&(pk==0)){
		return false;	// 三点共线
	}

	if((normal[0]*pi+normal[1]*pj + normal[3]*pk) < 0){
		pi = -pi;
		pj = -pj;
		pk = -pk;
	}

	float dMx = (ptA[0]+ptB[0])/2,	// AB 的中点M
			 dMy = (ptA[1]+ptB[1])/2,
			 dMz = (ptA[2]+ptB[2])/2;
	float dMi = pj*z1-pk*y1,	// AB 的中垂线方向
			 dMj = pk*x1-pi*z1,
			 dMk = pi*y1-pj*x1;
	float dNx = (ptA[0]+ptC[0])/2,	// AC 的中点M
			 dNy = (ptA[1]+ptC[1])/2,
			 dNz = (ptA[2]+ptC[2])/2;
	float dNi = pj*z2-pk*y2,// AC 的中垂线方向
			 dNj = pk*x2-pi*z2,
			 dNk = pi*y2-pj*x2;

	float ds = 1.0f / sqrt(pi*pi + pj*pj + pk*pk);
	pi *= ds;	// 面法矢归一化
	pj *= ds;
	pk *= ds;

	assert(dMi != 0);
	float tm, tn;
	tn  = ((dMy-dNy) * dMi + dMj * (dNx-dMx)) / (dNj*dMi - dMj*dNi);
	tm = (dNx + dNi*tn - dMx) / dMi;
	float ox = dMx + dMi*tm,	//三角形ABC的外接圆圆心
			 oy = dMy + dMj*tm,
			 oz = dMz + dMk*tm;
	float dR2 = ( (ox-ptA[0])*(ox-ptA[0]) +	// 外接圆半径的平方
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
/* 返回是否求出球心（true为求得）
	center 为球的球心（输出）
	idxA, idxB, idxC 为球上的三点ABC的索引
	ballR为球的半径
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
				// TODO 要不断修改face	取当前边的preTriangle
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
	// TODO:: 怎样确定查找最少的点？
	ANNidxArray nnIdx = new ANNidx[m_K+1];
	ANNdistArray dists = new ANNdist[m_K+1];
	// TODO：此处应该查找半径为 r 的球域
	int pts = m_ps->m_kdTree->annkFRSearch(midAB, m_ballR*m_ballR, m_K+1, nnIdx);		// 返回实际的点数
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
				// TODO：需要设置新边的球心，所以需要返回球心
				min_arc = arc;	// TODO : 选择弧度最小的
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
	return 0;		//两个面重合
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
		if (minAngel > 0.2)	// 三角形内角最小为30度
		{
			float len = getDists(idxC, seedA, seedB);	// 取点C与seedA和seedB形成的两边之和最小
			if (len < minLen){
				minLen = len;
				seedC = idxC;
			}
		}
	}
	delete[] nnidx;

	if (seedC == -1)	return false;
	//TODO 此处需要测试
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

	//TODO 此处可能造成死循环 当未使用的点很少时,会出现这种问题,
	//TODO 解决方法:判断idx是否将未使用点计算一遍
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
				if (minAngel > MINTRIANGEL30)	// 三角形内角最小为30度
				{
					bFound = true;
					float len = getDists(idxC, seedA, seedB);	// 取点C与seedA和seedB形成的两边之和最小
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

				//TODO 此处需要测试面的法矢是否一致
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

//#define MINTRIANGEL  0.1339745962	// 1-cos(30') 即三角形最小内角要大于30'
//#define MINTRIANGEL  0.0603073792	// 1-cos(20') 即三角形最小内角要大于20'
#define MINTRIANGEL  0.09369221296335	// 1-cos(25') 即三角形最小内角要大于25'
#define MAXDIHEDRALANGEL 1.5 // 1-cos(150') 即二面角最大为150'
//#define MAXDIHEDRALANGEL 1.8660254 // 1-cos(150') 即二面角最大为150'
	int idxPre = frontEdge.getOppPoint();

	CPlane prePlane, nextPlane;
	int idxApre = -1, idxBnext = -1;
	getPreNextIdx(idxApre, idxBnext, itEdge);
	getPlanes(prePlane, nextPlane, idxA, idxB, idxPre, idxApre, idxBnext);

	for (int i=0; i < K ; i++)
	{
		int idxCur = nnIdx[i];
		if ( INPOINT == m_bPointUsed[idxCur] || idxCur == idxA || idxCur == idxB || idxCur == idxPre) continue;

		//TODO 还需要使用二面角来判断
		//TODO 当同时存在两个波前点其权重一样，如何选择？（如何保证不相交或者一条边只能连接两个三角形）
		float minAngel = getMinTriAngel(idxA, idxB, idxCur);	// 要求最小内角要大于指定度数
		float dihedralAngel = getDihedralAngel(frontEdge, idxCur);// 要求二面角要小于指定度数
		if (minAngel > MINTRIANGEL && dihedralAngel < MAXDIHEDRALANGEL && isBtwPlanes(prePlane, nextPlane, frontEdge, idxCur)){
		//if (minAngel > MINTRIANGEL){
		//	float weight = getWeightTri(frontEdge, idxCur, 4.0);
			// 测试使用
			//TODO: 对于波前边上的顶点 要格外注意，还要判断连接点是否在点的外测
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
	vAB.cross(vAC);	// 与ABC面相垂直
	vect3f nABC = vAB;	// 面ABC的法向量

	vect3f vOA( ptA[0] - oABC[0], ptA[1] - oABC[1], ptA[2] - oABC[2] );
	vAB.cross(vOA);	// 面OAn的法向量
	prePlane.setPlane(vAB, ptA);	// 面OAn
	if (ePre != -1){
		float* ptPre = m_ps->m_point[ePre];
		if (prePlane.iSameSide(ptB, ptPre) > 0){
			vect3f nAPn = nABC;
			vect3f AP( ptPre[0] - ptA[0], ptPre[1] - ptA[1], ptPre[2] - ptA[2] );
			nAPn.cross(AP);
			prePlane.setPlane(nAPn, ptA);// 平面APren
		}
	}

	vect3f vOB ( ptB[0] - oABC[0], ptB[1] - oABC[1], ptB[2] - oABC[2] );
	vOB.cross(nABC);	// 面OBn的法向量
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
	//TODO 未判断点在边的前方还是后方
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