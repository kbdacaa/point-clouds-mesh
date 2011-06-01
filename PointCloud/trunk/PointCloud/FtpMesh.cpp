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
				//+ ��ͬһ���߽绷��
				CTriangle* pFace = new CTriangle(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB());
// 				pFace->setNeighbor(vIdx, itFront.itEdge->getPreTriangle());
// 				itFront.itEdge->getPreTriangle()->setNeighbor(itFront.itEdge->getOppPoint(), pFace);
				faceVector.push_back(pFace);

				// ���߽绷Ϊ�ڻ��ұߵĸ���Ϊ3��4ʱֱ������������
				if (!itFront.itList->bOutRing() && itFront.itList->frontEdgesSize() < 5)
					toFace(itFront.itList, faceVector);

				FrontEdgeIterator itVPreEdge = itV.itList->preEdge(itV.itEdge);// ��ǰitV.itEdge��ǰһ����
				if (itV.itEdge->getB() == itFront.itEdge->getA()){
					CFrontEdge b(vIdx, itFront.itEdge->getB(), itFront.itEdge->getA(), pFace);

					// ��Ҫ����itEdge.itEdge->getA() Ϊ�ڵ�
					setPointStatus(itFront.itEdge->getA());
					itV.itList->frontEdges().insert(itV.itEdge, b);
					itFront.itList->frontEdges().erase(itV.itEdge);
					itFront.itList->frontEdges().erase(itFront.itEdge);
					return true;
				}else if (itVPreEdge->getA() == itFront.itEdge->getB()){
					CFrontEdge a(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB(), pFace);

					// ��Ҫ����itEdge.itEdge->getB() Ϊ�ڵ�
					setPointStatus(itFront.itEdge->getB());
					itFront.itList->frontEdges().insert(itFront.itEdge, a);
					itFront.itList->frontEdges().erase(itFront.itEdge);
					itFront.itList->frontEdges().erase(itVPreEdge);
					return true;
				}else{//! ���������ߣ�ͬʱ����ǰ������Ϊ����
					CFrontEdge a(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB(), pFace),
						b(vIdx, itFront.itEdge->getB(), itFront.itEdge->getA(), pFace);

					itFront.itList->frontEdges().insert(itFront.itEdge, a);
					FrontEdgeIterator itEdgeB = itFront.itList->frontEdges().insert(itFront.itEdge, b);
					itFront.itList->frontEdges().erase(itFront.itEdge);

					// Ҫ�鿴�������Ĵ�С����߸���<5 ֱ���γ�������
					CFrontList newFrontList(itFront.itList, itEdgeB, itV.itEdge, false);
					if (newFrontList.frontEdgesSize() < 5){
						toFace(newFrontList, faceVector);
					}else{
						m_frontLists.push_back(newFrontList);
					}
					return true;
				}
			}else{
				//+ ����ͬһ���߽绷��
				if(!itFront.itList->bOutRing() || !itV.itList->bOutRing()){
					//! ������һ��Ϊ�ڻ�ʱ�������ܺϲ�Ϊһ����
					// ��Ҫ����ǰ��չ������Ϊ�߽��
					// ͬʱ�жϵ�ǰ��չ�ߵ����������Ƿ�Ϊ�߽��
					setBoundary(itFront);
					return false;
				}else{
					//! ͬʱΪ�⻷����Ҫ���������ϲ�Ϊһ���⻷
					CTriangle* pFace = new CTriangle(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB());
// 					pFace->setNeighbor(vIdx, itFront.itEdge->getPreTriangle());
// 					itFront.itEdge->getPreTriangle()->setNeighbor(itFront.itEdge->getOppPoint(), pFace);
					faceVector.push_back(pFace);

					CFrontEdge a(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB(), pFace),
						b(vIdx, itFront.itEdge->getB(), itFront.itEdge->getA(), pFace);

					itFront.itList->frontEdges().insert(itFront.itEdge, a);
					FrontEdgeIterator itInsert = itFront.itList->frontEdges().insert(itFront.itEdge, b);
					itFront.itList->frontEdges().erase(itFront.itEdge);

					//^ ������һ���߽绷�ϲ�����ǰ�߽绷�� ^
					itFront.itList->frontEdges().insert(itInsert, itV.itEdge, itV.itList->frontEdges().end());
					itFront.itList->frontEdges().insert(itInsert, itV.itList->frontEdges().begin(), itV.itEdge);
					m_frontLists.erase(itV.itList);
					return true;
				}
			}
		}else{
			// ��Ҫ����ǰ��չ������Ϊ�߽��, ͬʱ�жϵ�ǰ��չ�ߵ����������Ƿ�Ϊ�߽��
			setBoundary(itFront);
			return false;	// ���ڱ߽绷�ϣ���bFront����,Ӧ�ý���ǰ������Ϊ�߽��
		}
	}else{
		// ���ڱ߽绷�ϣ�ֱ����������ߣ�ȥ����ǰ��
		CTriangle* pFace = new CTriangle(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB());
// 		pFace->setNeighbor(vIdx, itFront.itEdge->getPreTriangle());
// 		itFront.itEdge->getPreTriangle()->setNeighbor(itFront.itEdge->getOppPoint(), pFace);
		faceVector.push_back(pFace);

		CFrontEdge a(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB(), pFace),
			b(vIdx, itFront.itEdge->getB(), itFront.itEdge->getA(), pFace);

		itFront.itList->frontEdges().insert(itFront.itEdge, a);
		itFront.itList->frontEdges().insert(itFront.itEdge, b);
		itFront.itList->frontEdges().erase(itFront.itEdge);
		//TODO Ҫ���õ�ǰ��vIdxΪ��ǰ�㣬����ӵ���ǰ���ĵ㼯��
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
				if (minAngel > MINTRIANGEL30)	// �������ڽ���СΪ30��
				{
					bFound = true;
					float len = pointSet()->lengthSum(idxC, seedA, seedB);
					if (len < minLen){	// ȡ��C��seedA��seedB�γɵ�����֮����С
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
	float ptMidAB[3] = { // AB �е�
		(ptA[0]+ptB[0])/2.0f,
		(ptA[1]+ptB[1])/2.0f,
		(ptA[2]+ptB[2])/2.0f };

	float eAB[3] = Edge(ptA, ptB),
		eAC[3] = Edge(ptA, ptC);

	float lenAB = pointSet()->length(idxA, idxB);
	float nABC[3];	eCross(nABC, eAB, eAC);	// ��ABC�ķ�ʸ
	float nABn[3];	eCross(nABn, eAB, nABC); // ��PnAB�ķ�ʸ
	eUnit(nABn);
	CPlane PnAB(nABn, ptA);// AB���γɵĴ�ֱ��

#define PREDICATER 0.707f	// ǰ��Ԥ��뾶
	float r2 = lenAB*lenAB;	//��İ뾶��ƽ��
	lenAB *= PREDICATER;
	float ptPredicate[3] = {// ǰ��Ԥ���=����
		ptMidAB[0] + nABn[0]*lenAB,
		ptMidAB[1] + nABn[1]*lenAB,
		ptMidAB[2] + nABn[2]*lenAB,
	};

#define MAX_K_PT  30		// �����뾶 r �ڵ�������
#define MIN_TRISNGEL 0.06031f	// ��������С�ڽ�Ҫ���� 20' (1-cos(20'))
#define MAX_DIHEDRALANGEL 1.0f	// ���������εĶ����ҪС��90' (1-cos(90'))

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

		// TODO ��ʱ�������ж��Ƿ�������������Խ�
		float minAngel = pointSet()->getMinTriAngel(idxA, idxB, idxCand);
		if (minAngel > MIN_TRISNGEL){
			float dihedralAngel = pointSet()->getDihedralAngel(idxC, idxA, idxB, idxCand);
			if (dihedralAngel < MAX_DIHEDRALANGEL){
				float lenSum = pointSet()->lengthSum(idxCand, idxA, idxB);
				if (lenSum < minLengthSum){	// ʹ����̱�ԭ��
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