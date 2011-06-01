#include "StdAfx.h"
#include "FtpMesh.h"
#include "PointCloudView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


bool frontListCompare(const CFrontList& a, const CFrontList& b){
	return a.frontEdgesSize() < b.frontEdgesSize();
}

bool CFrontSet::join( FrontIter& itFront, int vIdx, bool bFront, vector<CTriangle*>& faceVector )
{
	if (bFront)
	{
		FrontIter itV;
		if (findVertex(itV, vIdx)){
			if (itV.itList == itFront.itList){
				//+ 在同一个边界环上
				CTriangle* pFace = new CTriangle(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB());
// 				pFace->setNeighbor(vIdx, itFront.itEdge->getPreTriangle());
// 				itFront.itEdge->getPreTriangle()->setNeighbor(itFront.itEdge->getOppPoint(), pFace);
				faceVector.push_back(pFace);

				// 当边界环为内环且边的个数为3或4时直接生成三角形
				if (!itFront.itList->bOutRing() && itFront.itList->frontEdgesSize() < 5)
					toFace(itFront.itList, faceVector);

				FrontEdgeIterator itVPreEdge = itV.itList->preEdge(itV.itEdge);// 当前itV.itEdge的前一条边
				if (itV.itEdge->getB() == itFront.itEdge->getA()){
					CFrontEdge b(vIdx, itFront.itEdge->getB(), itFront.itEdge->getA(), pFace);

					// 需要设置itEdge.itEdge->getA() 为内点
					setPointStatus(itFront.itEdge->getA());
					itV.itList->frontEdges().insert(itV.itEdge, b);
					itFront.itList->frontEdges().erase(itV.itEdge);
					itFront.itList->frontEdges().erase(itFront.itEdge);
					return true;
				}else if (itVPreEdge->getA() == itFront.itEdge->getB()){
					CFrontEdge a(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB(), pFace);

					// 需要设置itEdge.itEdge->getB() 为内点
					setPointStatus(itFront.itEdge->getB());
					itFront.itList->frontEdges().insert(itFront.itEdge, a);
					itFront.itList->frontEdges().erase(itFront.itEdge);
					itFront.itList->frontEdges().erase(itVPreEdge);
					return true;
				}else{//! 生成两条边，同时将当前环分裂为两个
					CFrontEdge a(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB(), pFace),
						b(vIdx, itFront.itEdge->getB(), itFront.itEdge->getA(), pFace);

					itFront.itList->frontEdges().insert(itFront.itEdge, a);
					FrontEdgeIterator itEdgeB = itFront.itList->frontEdges().insert(itFront.itEdge, b);
					itFront.itList->frontEdges().erase(itFront.itEdge);

					// 要查看两个环的大小如果边个数<5 直接形成三角形
					CFrontList newFrontList(itFront.itList, itEdgeB, itV.itEdge, false);
					if (newFrontList.frontEdgesSize() < 5){
						toFace(newFrontList, faceVector);
					}else{
						m_frontLists.push_back(newFrontList);
					}
					return true;
				}
			}else{
				//+ 不在同一个边界环上
				if(!itFront.itList->bOutRing() || !itV.itList->bOutRing()){
					//! 其中有一个为内环时，将不能合并为一个环
					// 需要将当前扩展边设置为边界边
					// 同时判断当前扩展边的两个顶点是否为边界点
					setBoundary(itFront);
					return false;
				}else{
					//! 同时为外环，需要将两个环合并为一个外环
					CTriangle* pFace = new CTriangle(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB());
// 					pFace->setNeighbor(vIdx, itFront.itEdge->getPreTriangle());
// 					itFront.itEdge->getPreTriangle()->setNeighbor(itFront.itEdge->getOppPoint(), pFace);
					faceVector.push_back(pFace);

					CFrontEdge a(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB(), pFace),
						b(vIdx, itFront.itEdge->getB(), itFront.itEdge->getA(), pFace);

					itFront.itList->frontEdges().insert(itFront.itEdge, a);
					FrontEdgeIterator itInsert = itFront.itList->frontEdges().insert(itFront.itEdge, b);
					itFront.itList->frontEdges().erase(itFront.itEdge);

					//^ 将另外一个边界环合并到当前边界环中 ^
					itFront.itList->frontEdges().insert(itInsert, itV.itEdge, itV.itList->frontEdges().end());
					itFront.itList->frontEdges().insert(itInsert, itV.itList->frontEdges().begin(), itV.itEdge);
					m_frontLists.erase(itV.itList);
					return true;
				}
			}
		}else{
			// 需要将当前扩展边设置为边界边, 同时判断当前扩展边的两个顶点是否为边界点
			setBoundary(itFront);
			return false;	// 不在边界环上，与bFront相异,应该将当前边设置为边界边
		}
	}else{
		// 不在边界环上，直接添加两条边，去除当前边
		CTriangle* pFace = new CTriangle(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB());
// 		pFace->setNeighbor(vIdx, itFront.itEdge->getPreTriangle());
// 		itFront.itEdge->getPreTriangle()->setNeighbor(itFront.itEdge->getOppPoint(), pFace);
		faceVector.push_back(pFace);

		CFrontEdge a(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB(), pFace),
			b(vIdx, itFront.itEdge->getB(), itFront.itEdge->getA(), pFace);

		itFront.itList->frontEdges().insert(itFront.itEdge, a);
		itFront.itList->frontEdges().insert(itFront.itEdge, b);
		itFront.itList->frontEdges().erase(itFront.itEdge);
		//TODO 要设置当前点vIdx为波前点，并添加到波前环的点集中
		setPointStatus(vIdx, FRONTPOINT);
		//! itEdge.itList->addVertex(vIdx);
		return true;
	}
}

bool CFTPMesh::findSeedTriangle( CTriangle*& pFace, int K)
{
	int seedA = getFirstUnusedPt(-1), firstSeed = seedA;
	ANNidxArray nnidx = new ANNidx[K+1];
	ANNdistArray dists = new ANNdist[K+1];

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
				float minAngel = pointSet()->getMinTriAngel(seedA, seedB, idxC);
				if (minAngel > MINTRIANGEL30)	// 三角形内角最小为30度
				{
					bFound = true;
					float len = pointSet()->lengthSum(idxC, seedA, seedB);
					if (len < minLen){	// 取点C与seedA和seedB形成的两边之和最小
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
				setVertexUsed(seedB);
				setVertexUsed(seedC);
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

bool CFTPMesh::getCandidatePoint( const FrontIter& itFront, int& vIdx, bool& bFrontVertex )
{
	int idxA = itFront.itEdge->getA(), idxB = itFront.itEdge->getB();
	int idxC = itFront.itEdge->getOppPoint();
	float* ptA = pointSet()->getPoint(idxA),
		* ptB = pointSet()->getPoint(idxB),
		* ptC = pointSet()->getPoint(idxC);
	float ptMidAB[3] = { // AB 中点
		(ptA[0]+ptB[0])/2.0f,
		(ptA[1]+ptB[1])/2.0f,
		(ptA[2]+ptB[2])/2.0f };

	float eAB[3] = Edge(ptA, ptB),
		eAC[3] = Edge(ptA, ptC);

	float lenAB = pointSet()->length(idxA, idxB);
	float nABC[3];	eCross(nABC, eAB, eAC);	// 面ABC的法矢
	float nABn[3];	eCross(nABn, eAB, nABC); // 面PnAB的法矢
	eUnit(nABn);
	CPlane PnAB(nABn, ptA);// AB边形成的垂直面

#define PREDICATER 0.707f	// 前向预测半径
	float r2 = lenAB*lenAB;	//球的半径的平方
	lenAB *= PREDICATER;
	float ptPredicate[3] = {// 前向预测点=球心
		ptMidAB[0] + nABn[0]*lenAB,
		ptMidAB[1] + nABn[1]*lenAB,
		ptMidAB[2] + nABn[2]*lenAB,
	};

#define MAX_K_PT  30		// 搜索半径 r 内的最大点数
#define MIN_TRISNGEL 0.06031f	// 三角形最小内角要大于 20' (1-cos(20'))
#define MAX_DIHEDRALANGEL 1.0f	// 两个三角形的二面角要小于90' (1-cos(90'))

	int vBest = -1;
	bool bFront = false;
	float minLengthSum = FLT_MAX;
	ANNidxArray nnIdx = new ANNidx[MAX_K_PT];
	//ANNdistArray dists = new ANNdist[MAX_K_PT];
	int realPts = pointSet()->KdTree()->annkFRSearch(ptPredicate, r2, MAX_K_PT, nnIdx);
	for (int i = 0; i < min(realPts, MAX_K_PT); ++i)
	{
		int idxCand = nnIdx[i];
		if (idxCand == idxA || idxCand == idxB || idxCand == idxC
			|| INPOINT == getVertexStatus(idxCand) ) continue;

		// TODO 此时还不能判断是否出现三角形面自交
		float minAngel = pointSet()->getMinTriAngel(idxA, idxB, idxCand);
		if (minAngel > MIN_TRISNGEL){
			float dihedralAngel = pointSet()->getDihedralAngel(idxC, idxA, idxB, idxCand);
			if (dihedralAngel < MAX_DIHEDRALANGEL){
				float lenSum = pointSet()->lengthSum(idxCand, idxA, idxB);
				if (lenSum < minLengthSum){	// 使用最短边原则
					vBest = idxCand;
					minLengthSum = lenSum;
				}
			}
		}
	}
	delete[] nnIdx;
	if(vBest != -1){
		vIdx = vBest;
		bFrontVertex = bFront;//(getVertexStatus(vBest) == FRONTPOINT)
		return true;
	}
	return false;
}

void CFTPMesh::start()
{
	FrontIter itFront;
	bool bFrontVertex = false;
	int vIdx = -1;
	while (true){
		while (m_front.getActiveEdge(itFront))
		{
			if (getCandidatePoint(itFront, vIdx, bFrontVertex)){
				m_front.join(itFront, vIdx, bFrontVertex, m_faceVects);
			}else{
				m_front.setBoundary(itFront);
			}
			m_pView->draw();
		}
		CTriangle* pFace = NULL;
		if (findSeedTriangle(pFace)){
			m_faceVects.push_back(pFace);
			m_front.addFace(pFace);
		}else{
			return ;
		}
	}
}