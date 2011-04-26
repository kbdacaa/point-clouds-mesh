#include "stdafx.h"
#include "PointMesh.h"

bool CPointMesh::externPoint( size_t iSeed )
{
	//===== �γ�����չ��Ϊ���ĵ�ƽ�� =====//
	float* ptSeed = m_ps->m_point[iSeed];
	float* ptNorm = m_ps->m_normal[iSeed];
	CPlane planeAtPt(ptNorm, ptSeed);

	//==== �����Ѿ����ڵ�������ڵ㳤�� ====//
	CPointFront& ptSeedFront = m_links[iSeed];
	LinkList& linkPtF = ptSeedFront.getLink();
	LSIt itLinkPtF = linkPtF.begin();
	float maxExistLinkLength = 0;
	for (; itLinkPtF != linkPtF.end(); ++itLinkPtF) {
		float length = m_ps->length(iSeed, *itLinkPtF);
		if (length > maxExistLinkLength)
			maxExistLinkLength = length;
	}
	//====== ��������Χ�ڵĵ㼯 =======//
#define MAX_SEARTCH_POINT_NUM  10
	ANNidxArray nnIdx = new ANNidx[MAX_SEARTCH_POINT_NUM+1];	//���1��Ϊ�˺����γ�������ʱ���ûػ�
	int searchPointNum = MAX_SEARTCH_POINT_NUM;
	if (maxExistLinkLength == 0){
		ANNdistArray dist = new ANNdist[MAX_SEARTCH_POINT_NUM];
		m_ps->KdTree()->annkSearch(ptSeed, MAX_SEARTCH_POINT_NUM, nnIdx, dist);
		delete[] dist;
	} else {
#define LIMITED_POINT_NUM 6//! Ҫ��֤searchPointNum���ٴ���6
		float r2 = maxExistLinkLength*maxExistLinkLength;
		do {//!+ �˴����ܳ������⣺�������ĵ�̫�ٶ����ܰ���ԭ���Ѿ���Link�еĵ�
			int size = m_ps->KdTree()->annkFRSearch(ptSeed, r2, MAX_SEARTCH_POINT_NUM, nnIdx);
			searchPointNum = min(size, MAX_SEARTCH_POINT_NUM);
			r2 *= 1.3f;
		} while (searchPointNum < LIMITED_POINT_NUM);
	}
	//===== ���������ĵ�ͶӰ��ƽ���� =====//
	float (*projectPt)[3] = new float[searchPointNum][3];// ͶӰ��
	for (int i = 1; i < searchPointNum; i++) {
		planeAtPt.projectPoint( projectPt[i], m_ps->m_point[ nnIdx[i] ] );
	}
	//====== Ѱ����ʼ��������Ƕ� ======//
	size_t startPtIndex = 1;	//! nnIdx[0]Ϊ��ǰ��չ�� iSeed
	if(ptSeedFront.linkSize() > 0)	{// �����һ�����ӵ���nnIdx�е�λ�ã��ҵ���ͶӰ
		size_t firstIndex = ptSeedFront.linkFront();
		for (int i = 1; i < searchPointNum ; i++) {
			if (nnIdx[i] == firstIndex){
				startPtIndex = i;
				break;
			}
		}
	}
	//= ��������������ʼ�ߵļн�(��ʱ�뷽��) =//
	float* arccosa = new float[searchPointNum];	// �Ƕ�����
	for (int j = 1; j < searchPointNum; j++) {
		arccosa[j] = arcNorm(ptSeed, projectPt[startPtIndex], ptNorm, projectPt[j]);
	}
	//===== �Ը��㰴����ʱ�뷽������ =====//
	sortArc(arccosa, nnIdx, searchPointNum);
	//==== �������ӵ㰴����ʱ�뷽������ ===//
	sortWith(ptSeedFront.getLink(), nnIdx, searchPointNum);
	//========= �γ������� ==========//
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
		int j = (i+1) % N;	// j Ϊ i �ĺ�һ��
		CPointFront& ptFi = m_links[ nnIdx[i] ];
		CPointFront& ptFj = m_links[ nnIdx[j] ];
		int iExist = ptFj.existLink( index, nnIdx[i]);
		if (0 == iExist) continue;
		//! ��ʱ index��nnidx[i] ��Ȼ��Ҫһ��������
		// TODO ��Ҫ�����������ҵ���һ�� j
		bool bTriMeet = false;
		size_t nextTriIdx = 0;
		while ( j < N ){// �ҵ���һ����ǰ������ӵ����
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
		if (1 == iExist){// ֻ�����ĵ�����
			float angel = m_ps->getAngel(index, nnIdx[i], nnIdx[j]);
			if (angel < COS120){// TODO �����γ�����������
				ptFi.addLinkBack(nnIdx[j]);
				ptFj.addLinkBack(nnIdx[i]);
				CTriangle tri(index, nnIdx[i], nnIdx[j]);
				m_faces.push_back(tri);
			}else{
				setPointStatus(index, PS_BOUNDARY);
			}
		}else if (2 == iExist){ // ֻ��ǰһ������
			ptFj.addLinkBack(index);
			CTriangle tri(index, nnIdx[i], nnIdx[j]);
			m_faces.push_back(tri);
		}else if (-1 == iExist){// �����߾�������
			ptFi.addLinkBack(nnIdx[j]);
			ptFj.addLinkBack(index);
			ptFj.addLinkBack(nnIdx[i]);
			ptFj.setStatus(PS_FRONTPOINT);
			CTriangle tri(index, nnIdx[i], nnIdx[j]);
			m_faces.push_back(tri);
		}
	}
	// TODO ��Ҫ�����ĵ������˳�����
	if (ptF.getStatus() != PS_BOUNDARY) ptF.setStatus(PS_INPOINT);
}

void CPointMesh::formTriangles2(int iSeed, int* nnIdx, int N){
	//= �����ǰ��չ��δ���κε����ӣ���ӵ�һ�����ӵ� =//
	LinkList& linkList = m_links[iSeed].getLink();
	if (linkList.size() == 0){
		linkList.push_back(nnIdx[1]);
		CPointFront& pf = m_links[ nnIdx[1] ];
		pf.addLinkBack(iSeed);
		if (PS_UNUSED == pf.getStatus()){
			pf.setStatus(PS_FRONTPOINT);
		}
	}
	//==== ���link�е�Ԫ�ص�����λ��(���Ϊtrue) ====//
	bool * bStatus = new bool[N+1];	// Ҳ���������������Щ��
	bStatus[N] = true;	// ���һ��Ԫ�����һ��Ԫ����ͬ
	for (int i = 0; i < N ; i++) {
		bStatus[i] = false;
	}
	//=== ����linkList�Ƿ���nnIdx�� ===//
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
	//==����ÿһ������δ���ӵ������==//
	int i = 1;
	while (i < N+1) {
		CPointFront& ptFi = m_links[nnIdx[i]];
		j = i+1;
		if(bStatus[nnIdx[j]]){// �������㶼�����ĵ�����
			if (ptFi.existLink(nnIdx[j])){// ���������Ѿ�����
				i = j;
			}else{// ��������δ����
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
// �ٶ� start��end֮�䲻����
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
		if (j == end){// ���˵��ǰһ��ֱ������
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

//! ˵���� nnidx[start]��nnidx[end]����iSeed����,��start��end������
void CPointMesh::formTriangles2(size_t iSeed,const ANNidx* nnidx, int start, int end, bool * bStatus){
	int i = start, j = i+1;
	// TODO ����չ��Ϊ�߽��ʱ����������⣺��������ͽ�start��end������
	while (j < end+1){
		if (j == end){// ��ʱֱ���γ��γ�������
			// TODO ��Ҫ�����ж��Ƿ�Ϊ�߽��
			if (!m_links[nnidx[j]].existLink(nnidx[i]))
				addLinkBtw(nnidx[i], nnidx[j]);
			//TODO ������һ�����⣺��ǰһ���������γɣ����Ǵ�������ȴ������
			CTriangle tri(iSeed, nnidx[i], nnidx[j]);
			m_faces.push_back(tri);
		}else{
			if (satisfyTriRule(iSeed, nnidx[i], nnidx[j])){
				bStatus[j] = true;	// ��������ΪiSeed���ӵ�
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