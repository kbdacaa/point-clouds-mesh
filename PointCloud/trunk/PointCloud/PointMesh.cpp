#include "stdafx.h"
#include "PointMesh.h"

bool CPointMesh::externPoint( size_t iSeed )
{
	//===== 形成以扩展点为中心的平面 =====//
	float* ptSeed = m_ps->m_point[iSeed];
	float* ptNorm = m_ps->m_normal[iSeed];
	CPlane planeAtPt(ptNorm, ptSeed);

	//==== 计算已经存在的最大相邻点长度 ====//
	CPointFront& ptSeedFront = m_links[iSeed];
	LinkList& linkPtF = ptSeedFront.getLink();
	LSIt itLinkPtF = linkPtF.begin();
	float maxExistLinkLength = 0;
	for (; itLinkPtF != linkPtF.end(); ++itLinkPtF) {
		float length = m_ps->length(iSeed, *itLinkPtF);
		if (length > maxExistLinkLength)
			maxExistLinkLength = length;
	}
	//====== 搜索邻域范围内的点集 =======//
#define MAX_SEARTCH_POINT_NUM  10
	ANNidxArray nnIdx = new ANNidx[MAX_SEARTCH_POINT_NUM+1];	//多加1是为了后续形成三角形时不用回环
	int searchPointNum = MAX_SEARTCH_POINT_NUM;
	if (maxExistLinkLength == 0){
		ANNdistArray dist = new ANNdist[MAX_SEARTCH_POINT_NUM];
		m_ps->KdTree()->annkSearch(ptSeed, MAX_SEARTCH_POINT_NUM, nnIdx, dist);
		delete[] dist;
	} else {
#define LIMITED_POINT_NUM 6//! 要保证searchPointNum至少大于6
		float r2 = maxExistLinkLength*maxExistLinkLength;
		do {//!+ 此处可能出现问题：搜索到的点太少而不能包含原来已经在Link中的点
			int size = m_ps->KdTree()->annkFRSearch(ptSeed, r2, MAX_SEARTCH_POINT_NUM, nnIdx);
			searchPointNum = min(size, MAX_SEARTCH_POINT_NUM);
			r2 *= 1.3f;
		} while (searchPointNum < LIMITED_POINT_NUM);
	}
	//===== 将搜索到的点投影在平面上 =====//
	float (*projectPt)[3] = new float[searchPointNum][3];// 投影点
	for (int i = 1; i < searchPointNum; i++) {
		planeAtPt.projectPoint( projectPt[i], m_ps->m_point[ nnIdx[i] ] );
	}
	//====== 寻找起始边来计算角度 ======//
	size_t startPtIndex = 1;	//! nnIdx[0]为当前扩展点 iSeed
	if(ptSeedFront.linkSize() > 0)	{// 计算第一个连接点在nnIdx中的位置，找到其投影
		size_t firstIndex = ptSeedFront.linkFront();
		for (int i = 1; i < searchPointNum ; i++) {
			if (nnIdx[i] == firstIndex){
				startPtIndex = i;
				break;
			}
		}
	}
	//= 计算各点相对于起始边的夹角(逆时针方向) =//
	float* arccosa = new float[searchPointNum];	// 角度数组
	for (int j = 1; j < searchPointNum; j++) {
		arccosa[j] = arcNorm(ptSeed, projectPt[startPtIndex], ptNorm, projectPt[j]);
	}
	//===== 对各点按照逆时针方向排序 =====//
	sortArc(arccosa, nnIdx, searchPointNum);
	//==== 对已连接点按照逆时针方向排序 ===//
	sortWith(ptSeedFront.getLink(), nnIdx, searchPointNum);
	//========= 形成三角形 ==========//
	nnIdx[searchPointNum] = nnIdx[1];
	formTriangles2(iSeed, nnIdx, searchPointNum);

	delete[] arccosa;
	delete[] projectPt;
	delete[] nnIdx;
	return true;
}

void CPointMesh::formTriangles( int index, int* nnIdx, int N )
{
	CPointFront& ptF = m_links[index];
	if (PS_INPOINT == ptF.getStatus()){
		ptF.addLinkBack(nnIdx[1]);
		CPointFront& ptF1 = m_links[nnIdx[1]];
		ptF1.addLinkBack(index);
	}
	for (int i = 1; i < N; ) {
		int j = (i+1) % N;	// j 为 i 的后一个
		CPointFront& ptFi = m_links[ nnIdx[i] ];
		CPointFront& ptFj = m_links[ nnIdx[j] ];
		int iExist = ptFj.existLink( index, nnIdx[i]);
		if (0 == iExist) continue;
		//! 此时 index和nnidx[i] 必然需要一个三角形
		// TODO 需要按照条件查找到下一个 j
		bool bTriMeet = false;
		size_t nextTriIdx = 0;
		while ( j < N ){// 找到下一个当前点的连接点结束
			if(ptF.existLink(nnIdx[j])) {
				bTriMeet = true;
				break;
			}
			if(PS_INPOINT == getPointStatus(nnIdx[j]) ||
				PS_BOUNDARY == getPointStatus(nnIdx[j]) )
				continue;

			bTriMeet = satisfyTriRule(index, nnIdx[ i ], nnIdx[ j ]);
			if (bTriMeet)	break;
			j = (j+1);
		}
		i = j;
		if (!bTriMeet) {
			continue;
		}
		if (1 == iExist){// 只与中心点相连
			float angel = m_ps->getAngel(index, nnIdx[i], nnIdx[j]);
			if (angel < COS120){// TODO 可能形成狭长三角形
				ptFi.addLinkBack(nnIdx[j]);
				ptFj.addLinkBack(nnIdx[i]);
				CTriangle tri(index, nnIdx[i], nnIdx[j]);
				m_faces.push_back(tri);
			}else{
				setPointStatus(index, PS_BOUNDARY);
			}
		}else if (2 == iExist){ // 只与前一点相连
			ptFj.addLinkBack(index);
			CTriangle tri(index, nnIdx[i], nnIdx[j]);
			m_faces.push_back(tri);
		}else if (-1 == iExist){// 两条边均不存在
			ptFi.addLinkBack(nnIdx[j]);
			ptFj.addLinkBack(index);
			ptFj.addLinkBack(nnIdx[i]);
			ptFj.setStatus(PS_FRONTPOINT);
			CTriangle tri(index, nnIdx[i], nnIdx[j]);
			m_faces.push_back(tri);
		}
	}
	// TODO 需要将中心点的连接顺序调整
	if (ptF.getStatus() != PS_BOUNDARY) ptF.setStatus(PS_INPOINT);
}

void CPointMesh::formTriangles2(int iSeed, int* nnIdx, int N){
	//= 如果当前扩展点未与任何点连接，添加第一个连接点 =//
	LinkList& linkList = m_links[iSeed].getLink();
	if (linkList.size() == 0){
		linkList.push_back(nnIdx[1]);
		CPointFront& pf = m_links[ nnIdx[1] ];
		pf.addLinkBack(iSeed);
		if (PS_UNUSED == pf.getStatus()){
			pf.setStatus(PS_FRONTPOINT);
		}
	}
	//==== 标记link中的元素的所在位置(标记为true) ====//
	bool * bStatus = new bool[N+1];	// 也用来标记连接了哪些点
	bStatus[N] = true;	// 最后一个元素与第一个元素相同
	for (int i = 0; i < N ; i++) {
		bStatus[i] = false;
	}
	//=== 设置linkList是否在nnIdx中 ===//
	LSIt it = linkList.begin();
	int j = 1, start = 1;
	for (; it != linkList.end(); ++it) {
		j = start;
		while (j < N && nnIdx[j] != *it ){
			j++;
		}
		if (j != N){
			bStatus[j] = true;
			start = j+1;
		}else break;
	}
	//==查找每一个包含未连接点的区间==//
	int i = 1;
	while (i < N+1) {
		CPointFront& ptFi = m_links[nnIdx[i]];
		j = i+1;
		if(bStatus[nnIdx[j]]){// 相邻两点都与中心点相连
			if (ptFi.existLink(nnIdx[j])){// 相邻两点已经连接
				i = j;
			}else{// 相邻两点未连接
				addLinkBtw(nnIdx[i], nnIdx[j]);
				CTriangle tri(iSeed, nnIdx[i], nnIdx[j]);
				m_faces.push_back(tri);
				i = j;
			}
		}else{
			while (++j < N+1){
				if (bStatus[j] == true)
					break;
			}
			if(!ptFi.existLink(nnIdx[j]))
				formTriangles(iSeed, nnIdx, i, j, bStatus);
			i = j;
		}
	}
	delete[] bStatus;
}
/*
// 假定 start和end之间不相连
void CPointMesh::formTriangles(int index,const ANNidx* nnidx, int start, int end, bool * bStatus){
	int i = start;
	while (i < end){
		int j = i+1, iFind = 0;
		while (j < end+1){
			if (PS_INPOINT == m_links[nnidx[j]].getStatus()||
				PS_BOUNDARY == m_links[nnidx[j]].getStatus())
				continue;
			if (satisfyTriRule(index, nnidx[i], nnidx[j])){
				iFind = j;
				break;
			}
			j++;
		}
		if (j == end){// 将此点和前一点直接相连
			addLinkBtw(nnidx[i], nnidx[j]);
		}else{
			CPointFront& ptFiFind = m_links[nnidx[iFind]];
			ptFiFind.addLinkBack(index);
			if (!ptFiFind.existLink(nnidx[i])){
				addLinkBtw(nnidx[i], nnidx[iFind]);
			}
			if (PS_UNUSED == ptFiFind.getStatus()){
				ptFiFind.setStatus(PS_FRONTPOINT);
			}
			bStatus[iFind] = true;
		}
		CTriangle tri(index, nnidx[i], nnidx[iFind]);
		m_faces.push_back(tri);
		i = j;
	}
}*/

//! 说明： nnidx[start]和nnidx[end]均与iSeed相连,且start与end不相连
void CPointMesh::formTriangles2(size_t iSeed,const ANNidx* nnidx, int start, int end, bool * bStatus){
	int i = start, j = i+1;
	// TODO 当扩展点为边界点时，会出现问题：都不满足就将start和end相连了
	while (j < end+1){
		if (j == end){// 此时直接形成形成三角形
			// TODO 需要进行判断是否为边界点
			if (!m_links[nnidx[j]].existLink(nnidx[i]))
				addLinkBtw(nnidx[i], nnidx[j]);
			//TODO 还存在一个问题：当前一个三角形形成，但是此三角形却不存在
			CTriangle tri(iSeed, nnidx[i], nnidx[j]);
			m_faces.push_back(tri);
		}else{
			if (satisfyTriRule(iSeed, nnidx[i], nnidx[j])){
				bStatus[j] = true;	// 将其设置为iSeed连接点
				m_links[ nnidx[j] ].addLinkBack(iSeed);
				if(PS_UNUSED==m_links[ nnidx[j] ].getStatus())
					m_links[ nnidx[j] ].setStatus(PS_FRONTPOINT);
				if (!m_links[ nnidx[j] ].existLink(nnidx[i]))
					addLinkBtw(nnidx[i], nnidx[j]);
				CTriangle tri(iSeed, nnidx[i], nnidx[j]);
				m_faces.push_back(tri);
				i = j;
			}
		}
		j++;
	}
}

void CPointMesh::start()
{
	if (m_ps->m_normal == NULL) return;
	size_t seed = getSeed(0);
	m_fontPoints.push_back(seed);
	while (m_fontPoints.size() != 0) {
		seed = m_fontPoints.front();
		m_fontPoints.pop_front();
		externPoint(seed);
	}
}