#include "StdAfx.h"
#include "FtpMesh.h"

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
				pFace->setNeighbor(vIdx, itFront.itEdge->getPreTriangle());
				itFront.itEdge->getPreTriangle()->setNeighbor(itFront.itEdge, pFace);
				faceVector.push_back(pFace);

				// ���߽绷Ϊ�ڻ��ұߵĸ���Ϊ3��4ʱֱ������������
				if (!itFront.itList->bOutRing() && itFront.itList->frontEdgesSize() < 5)
					toFace(itFront.itList, faceVector);

				FrontEdgeIterator itVPreEdge = itV.itList->preEdge(itV.itEdge);// ��ǰitV.itEdge��ǰһ����
				if (itV.itEdge->getB() == itFront.itEdge->getA()){
					CFrontEdge b(vIdx, itFront.itEdge->getB());
					b.setPreTriangle(pFace);

					// ��Ҫ����itEdge.itEdge->getA() Ϊ�ڵ�
					setPointStatus(itFront.itEdge->getA());
					itV.itList->frontEdges().insert(itV.itEdge, b);
					itFront.itList->frontEdges().erase(itV.itEdge);
					itFront.itList->frontEdges().erase(itFront.itEdge);
					return true;
				}else if (itVPreEdge->getA() == itFront.itEdge->getB()){
					CFrontEdge a(itFront.itEdge->getA(), vIdx);
					a.setPreTriangle(pFace);

					// ��Ҫ����itEdge.itEdge->getB() Ϊ�ڵ�
					setPointStatus(itFront.itEdge->getB());
					itFront.itList->frontEdges().insert(itFront.itEdge, a);
					itFront.itList->frontEdges().erase(itFront.itEdge);
					itFront.itList->frontEdges().erase(itVPreEdge);
					return true;
				}else{
					CFrontEdge a(itFront.itEdge->getA(), vIdx), b(vIdx, itFront.itEdge->getB());
					a.setPreTriangle(pFace); b.setPreTriangle(pFace);

					itFront.itList->frontEdges().insert(itFront.itEdge, a);
					FrontEdgeIterator itEdgeB = itFront.itList->frontEdges().insert(itFront.itEdge, b);
					itFront.itList->frontEdges().erase(itFront.itEdge);
					// ����Ϊ�����߽绷����
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
					itFront.itEdge->setBoundary();
					// ͬʱ�жϵ�ǰ��չ�ߵ����������Ƿ�Ϊ�߽��
					setPointBoundary(itFront);
					return false;
				}else{
					//! ͬʱΪ�⻷����Ҫ���������ϲ�Ϊһ���⻷
					CTriangle* pFace = new CTriangle(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB());
					pFace->setNeighbor(vIdx, itFront.itEdge->getPreTriangle());
					itFront.itEdge->getPreTriangle()->setNeighbor(itFront.itEdge, pFace);
					faceVector.push_back(pFace);

					CFrontEdge a(itFront.itEdge->getA(), vIdx), b(vIdx, itFront.itEdge->getB());
					a.setPreTriangle(pFace);	b.setPreTriangle(pFace);

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
			// ��Ҫ����ǰ��չ������Ϊ�߽��
			itFront.itEdge->setBoundary();
			// ͬʱ�жϵ�ǰ��չ�ߵ����������Ƿ�Ϊ�߽��
			setPointBoundary(itFront);
			return false;	// ���ڱ߽绷�ϣ���bFront����,Ӧ�ý���ǰ������Ϊ�߽��
		}
	}else{
		// ���ڱ߽绷�ϣ�ֱ����������ߣ�ȥ����ǰ��
		CTriangle* pFace = new CTriangle(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB());
		pFace->setNeighbor(vIdx, itFront.itEdge->getPreTriangle());
		itFront.itEdge->getPreTriangle()->setNeighbor(itFront.itEdge, pFace);
		faceVector.push_back(pFace);

		CFrontEdge a(itFront.itEdge->getA(), vIdx), b(vIdx, itFront.itEdge->getB());
		a.setPreTriangle(pFace);	b.setPreTriangle(pFace);

		itFront.itList->frontEdges().insert(itFront.itEdge, a);
		itFront.itList->frontEdges().insert(itFront.itEdge, b);
		itFront.itList->frontEdges().erase(itFront.itEdge);
		//TODO Ҫ���õ�ǰ��vIdxΪ��ǰ�㣬����ӵ���ǰ���ĵ㼯��
		//! itEdge.itList->addVertex(vIdx);
		return true;
	}
}