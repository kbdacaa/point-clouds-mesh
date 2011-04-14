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
				//+ 在同一个边界环上
				CTriangle* pFace = new CTriangle(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB());
				pFace->setNeighbor(vIdx, itFront.itEdge->getPreTriangle());
				itFront.itEdge->getPreTriangle()->setNeighbor(itFront.itEdge, pFace);
				faceVector.push_back(pFace);

				// 当边界环为内环且边的个数为3或4时直接生成三角形
				if (!itFront.itList->bOutRing() && itFront.itList->frontEdgesSize() < 5)
					toFace(itFront.itList, faceVector);

				FrontEdgeIterator itVPreEdge = itV.itList->preEdge(itV.itEdge);// 当前itV.itEdge的前一条边
				if (itV.itEdge->getB() == itFront.itEdge->getA()){
					CFrontEdge b(vIdx, itFront.itEdge->getB());
					b.setPreTriangle(pFace);

					// 需要设置itEdge.itEdge->getA() 为内点
					setPointStatus(itFront.itEdge->getA());
					itV.itList->frontEdges().insert(itV.itEdge, b);
					itFront.itList->frontEdges().erase(itV.itEdge);
					itFront.itList->frontEdges().erase(itFront.itEdge);
					return true;
				}else if (itVPreEdge->getA() == itFront.itEdge->getB()){
					CFrontEdge a(itFront.itEdge->getA(), vIdx);
					a.setPreTriangle(pFace);

					// 需要设置itEdge.itEdge->getB() 为内点
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
					// 分裂为两个边界环链表
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
					itFront.itEdge->setBoundary();
					// 同时判断当前扩展边的两个顶点是否为边界点
					setPointBoundary(itFront);
					return false;
				}else{
					//! 同时为外环，需要将两个环合并为一个外环
					CTriangle* pFace = new CTriangle(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB());
					pFace->setNeighbor(vIdx, itFront.itEdge->getPreTriangle());
					itFront.itEdge->getPreTriangle()->setNeighbor(itFront.itEdge, pFace);
					faceVector.push_back(pFace);

					CFrontEdge a(itFront.itEdge->getA(), vIdx), b(vIdx, itFront.itEdge->getB());
					a.setPreTriangle(pFace);	b.setPreTriangle(pFace);

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
			// 需要将当前扩展边设置为边界边
			itFront.itEdge->setBoundary();
			// 同时判断当前扩展边的两个顶点是否为边界点
			setPointBoundary(itFront);
			return false;	// 不在边界环上，与bFront相异,应该将当前边设置为边界边
		}
	}else{
		// 不在边界环上，直接添加两条边，去除当前边
		CTriangle* pFace = new CTriangle(itFront.itEdge->getA(), vIdx, itFront.itEdge->getB());
		pFace->setNeighbor(vIdx, itFront.itEdge->getPreTriangle());
		itFront.itEdge->getPreTriangle()->setNeighbor(itFront.itEdge, pFace);
		faceVector.push_back(pFace);

		CFrontEdge a(itFront.itEdge->getA(), vIdx), b(vIdx, itFront.itEdge->getB());
		a.setPreTriangle(pFace);	b.setPreTriangle(pFace);

		itFront.itList->frontEdges().insert(itFront.itEdge, a);
		itFront.itList->frontEdges().insert(itFront.itEdge, b);
		itFront.itList->frontEdges().erase(itFront.itEdge);
		//TODO 要设置当前点vIdx为波前点，并添加到波前环的点集中
		//! itEdge.itList->addVertex(vIdx);
		return true;
	}
}