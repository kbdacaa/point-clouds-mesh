#include "stdafx.h"
#include "PointMesh.h"
#include "PointCloudView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

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
#define MAX_SEARTCH_POINT_NUM  9
	ANNidxArray nnIdx = new ANNidx[MAX_SEARTCH_POINT_NUM+1+linkPtF.size()];	//���1��Ϊ�˺����γ�������ʱ���ûػ�
	int searchPointNum = MAX_SEARTCH_POINT_NUM;
	if (maxExistLinkLength == 0){	// ��ǰ��Ϊδʹ�õ㣬û�����ӵ�
		ANNdistArray dist = new ANNdist[MAX_SEARTCH_POINT_NUM];
		m_ps->KdTree()->annkSearch(ptSeed, MAX_SEARTCH_POINT_NUM, nnIdx, dist);
		delete[] dist;
	} else {	// �Ѿ��������ӵ�
#define LIMITED_POINT_NUM 6//! Ҫ��֤searchPointNum���ٴ���6
		float r2 = maxExistLinkLength*maxExistLinkLength;
		do {//! �˴����ܳ������⣺�������ĵ�̫�ٶ����ܰ���ԭ���Ѿ���Link�еĵ�
			int size = m_ps->KdTree()->annkFRSearch(ptSeed, r2, MAX_SEARTCH_POINT_NUM, nnIdx);
			searchPointNum = min(size, MAX_SEARTCH_POINT_NUM);
			r2 *= 1.3f;
		} while (searchPointNum < LIMITED_POINT_NUM);
		//TODO��Ҫ�����ߺͲ�����
		itLinkPtF = linkPtF.begin();
		int insertStart = searchPointNum;
		for (; itLinkPtF != linkPtF.end(); ++itLinkPtF) {
			bool bFind = false;
			for (int i = 1; i < searchPointNum; ++i) {
				if (nnIdx[i] == *itLinkPtF){
					bFind = true;
					break;
				}
			}
			if (!bFind){
				nnIdx[insertStart++] = *itLinkPtF;
			}
		}
		searchPointNum = insertStart;
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
	arccosa[startPtIndex] = 0.0f;
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

void CPointMesh::formTriangles( size_t index, ANNidx* nnIdx, int N )
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
//========== ����ֱ�ӽ�������������ĵ��γ�������============//
void CPointMesh::formTriangles2(size_t iSeed, ANNidx* nnIdx, int N){
	//= �����ǰ��չ��δ���κε����ӣ���ӵ�һ�����ӵ� =//
	LinkList& linkList = m_links[iSeed].getLink();
	if (linkList.size() == 0){
		linkList.push_back(nnIdx[1]);
		CPointFront& pf = m_links[ nnIdx[1] ];
		pf.addLinkBack(iSeed);
		if (PS_UNUSED == pf.getStatus()){
			pf.setStatus(PS_FRONTPOINT);
			m_fontPoints.push_back(nnIdx[1]);
		}
	}
	//==== ���link�е�Ԫ�ص�����λ��(���Ϊtrue) ====//
	bool * bLinkedCenter = new bool[N+1];	// Ҳ���������������Щ��
	bLinkedCenter[N] = true;	// ���һ��Ԫ�����һ��Ԫ����ͬ
	for (int k = 0; k < N ; k++) {
		bLinkedCenter[k] = false;
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
			bLinkedCenter[j] = true;
			start = j+1;
		}else break;
	}
	//==����ÿһ������δ���ӵ������==//
	int i = 1;
	j = i + 1;
	while (j < N+1) {
		CPointFront& ptFi = m_links[nnIdx[i]];
		if(bLinkedCenter[ j ]){// �������㶼�����ĵ�����
			if (!ptFi.existLink( nnIdx[j] )){// ��������δ����
				addLinkBtw( nnIdx[i], nnIdx[j] );
				CTriangle tri( iSeed, nnIdx[i], nnIdx[j] );
				tri.calcNorm(m_ps);
				m_faces.push_back(tri);
			}
		}else{
			while (++j < N+1){
				if (bLinkedCenter[j] == true)
					break;
			}
			if(!ptFi.existLink(nnIdx[j]))
				formTriangles2(iSeed, nnIdx, i, j, bLinkedCenter);
		}
		i = j;		j++;
	}
	delete[] bLinkedCenter;
}

//! ˵���� nnidx[start]��nnidx[end]����iSeed����,��start��end������
void CPointMesh::formTriangles2(size_t iSeed,const ANNidx* nnidx, int start, int end, bool * bLinkedCenter){
	int i = start, j = i+1;
	// TODO ����չ��Ϊ�߽��ʱ����������⣺��������ͽ�start��end������
	while (j < end+1){
		if (j == end){// ��ʱֱ���γ��γ�������
			if (nnidx[i] != nnidx[j])
			{	// TODO ��Ҫ�����ж��Ƿ�Ϊ�߽��
				if ( !m_links[nnidx[j]].existLink(nnidx[i]) )
					addLinkBtw(nnidx[i], nnidx[j]);
				//TODO ������һ�����⣺��ǰһ���������γɣ����Ǵ�������ȴ������
				CTriangle tri(iSeed, nnidx[i], nnidx[j]);
				tri.calcNorm(m_ps);
				m_faces.push_back(tri);
			}
		}else{
			if (satisfyTriRule(iSeed, nnidx[i], nnidx[j])){
				bLinkedCenter[j] = true;	// ��������ΪiSeed���ӵ�
				m_links[ nnidx[j] ].addLinkBack(iSeed);
				if(PS_UNUSED==m_links[ nnidx[j] ].getStatus()){
					m_links[ nnidx[j] ].setStatus(PS_FRONTPOINT);
					m_fontPoints.push_back(nnidx[j]);
				}
				if (!m_links[ nnidx[j] ].existLink(nnidx[i]))
					addLinkBtw(nnidx[i], nnidx[j]);
				CTriangle tri(iSeed, nnidx[i], nnidx[j]);
				tri.calcNorm(m_ps);
				m_faces.push_back(tri);
				i = j;
			}
		}
		j++;
	}
}

void CPointMesh::start()
{
	if (m_ps->m_normal == NULL) {
		m_ps->computeNormal(10);
	}
	m_links.resize(m_ps->getPointSize());

	size_t seed = getSeed(0);
	m_fontPoints.push_back(seed);
	size_t handand = 0;
	while (m_fontPoints.size() != 0) {
		seed = m_fontPoints.front();
		m_fontPoints.pop_front();
		externPoint(seed);
		handand ++;
		if (handand % 100 == 0)
			m_pView->draw();
	}
	m_links.clear();
//	cout<< m_faces.size()<<endl;
}

void CPointMesh::filpNormal()
{
	vector<CTriangle>::iterator it = m_faces.begin();
	for (; it != m_faces.end(); ++it) {
		it->filpNorm();
	}
}

void CPointMesh::checkBoundaryPoint()
{
	size_t boundaryPointNum = 0;
	PFIter it = m_links.begin();
	for (; it != m_links.end(); it ++) {
		if (PS_BOUNDARY == it->getStatus()){
			boundaryPointNum ++;
		}
	}
	cout<< boundaryPointNum<<endl;
}

/**************POINT��ESH ̰���㷨*************************/
void CPointMesh::startT()
{
	if (m_ps->m_normal == NULL) {
		m_ps->computeNormal(10);
		m_ps->adjustNormal(10);
	}
	m_links.clear();
	m_faces.clear();
	m_fontPoints.clear();
	m_links.resize(m_ps->getPointSize());

	//size_t seed = getSeed(0);
	size_t seed = rand()%m_ps->getPointSize();
	externSeedPoint(seed);
	size_t handand = 0;
	while (m_fontPoints.size() != 0) {
		seed = m_fontPoints.front();
		m_fontPoints.pop_front();
		externFrontPoint(seed);
		handand ++;

	if (handand % 100 == 0)
		m_pView->draw();
	}
	//m_links.clear();
	//	cout<< m_faces.size()<<endl;
}

//=======����̰���㷨���ϲ�����һ���������͵�ǰ���γ�������======//
void CPointMesh::externSeedPoint(size_t iSeed){
	//====== ��������Χ�ڵĵ㼯 =======//
	float* ptSeed = m_ps->m_point[iSeed];
	ANNidxArray nnidx = new ANNidx[m_searchPointNum+1];//���1��Ϊ�˺����γ�������ʱ���ûػ�
	ANNdistArray dist = new ANNdist[m_searchPointNum];
	m_ps->KdTree()->annkSearch(ptSeed, m_searchPointNum, nnidx, dist);
	delete[] dist;
	//=======ȥ���Ѿ�Ϊɾ����Ķ���=======//
	int searchPointNum = 1;
	ANNidxArray nnIdx = new ANNidx[m_searchPointNum+1];
	for (int m = 1; m < m_searchPointNum; m++){
		if (m_links[ nnidx[m] ].getStatus() != PS_DELETE){
			nnIdx[m] = nnidx[m];
			searchPointNum++;
		}
	}
	delete nnidx;
	nnidx = nnIdx;

	//===== ������ƽ���߳���ӽ��ı� =====//
	float distanceAve = 0.0f;
	float* dists = new float[searchPointNum];
	for (int i = 1; i <searchPointNum; i++){
		dists[i] = m_ps->length(iSeed, nnidx[i]);
		distanceAve += dists[i];
	}
	distanceAve /= (searchPointNum-1);

	int start = 0;		// ������ӽ�ƽ���߳��Ķ���
	float minDist = FLT_MAX;
	for (int j = 1; j < searchPointNum; j++){
		float d = abs(distanceAve - dists[j]);
		if ( d > minDist ){
			minDist = d;
			start = j;
		}
	}
	delete[] dists;

	//===== �γ�����չ��Ϊ���ĵ�ƽ�� =====//
	float* ptNorm = m_ps->m_normal[iSeed];
	CPlane planeAtPt(ptNorm, ptSeed);
	//===== ���������ĵ�ͶӰ��ƽ���� =====//
	float (*projectPt)[3] = new float[searchPointNum][3];// ͶӰ��
	for (int i = 1; i < searchPointNum; i++) {
		planeAtPt.projectPoint( projectPt[i], m_ps->m_point[ nnidx[i] ] );
	}

	//= ��������������ʼ�ߵļн�(��ʱ�뷽��) =//
	float* arccosa = new float[searchPointNum];	// �Ƕ�����
	for (int j = 1; j < searchPointNum; j++) {
		arccosa[j] = arcNorm(ptSeed, projectPt[start], ptNorm, projectPt[j]);
	}
	delete[] projectPt;
	arccosa[start] = 0.0f;
	//===== �Ը��㰴����ʱ�뷽������ =====//
	sortArc(arccosa, nnidx, searchPointNum);
	delete[] arccosa;
	//========= �γ������� ==========//
	nnidx[searchPointNum] = nnidx[1];
	externSeedTriangles(iSeed, nnidx, searchPointNum);
	delete[] nnidx;

	if (getPointStatus(iSeed) != PS_BOUNDARY)
		setPointStatus(iSeed, PS_INPOINT);
}

//************************************
// �Զ���iSeed��չ������
// Parameter: size_t iSeed ���ĵ����
// Parameter: ANNidx * nnIdx ���ĵ�����
// Parameter: int N ��������
//************************************
void CPointMesh::externSeedTriangles(size_t iSeed, ANNidx* nnIdx, int N){
	int bestIdx = 0;
	int ii = 1;
	while (ii <N+1){
		bestIdx = externFirstTriangle(iSeed, nnIdx, ii, N);
		if (bestIdx != 0)	break;
		else m_links[iSeed].setStatus(PS_BOUNDARY);
		ii++;
	}
	if (bestIdx !=0){
		CTriangle tri(iSeed, nnIdx[ii], nnIdx[bestIdx]);
		tri.calcNorm(m_ps);
		m_faces.push_back(tri);

		addLinkBtw(iSeed, nnIdx[ii]);
		addLinkBtw(iSeed, nnIdx[bestIdx]);
		if (!m_links[ nnIdx[ii] ].existLink(nnIdx[bestIdx]))
			addLinkBtw(nnIdx[ii], nnIdx[bestIdx]);

		if (m_links[ nnIdx[ii] ].getStatus() == PS_UNUSED){
			m_links[ nnIdx[ii] ].setStatus(PS_FRONTPOINT);
			m_fontPoints.push_back(nnIdx[ii]);
		}
		if (m_links[ nnIdx[bestIdx] ].getStatus() == PS_UNUSED){
			m_links[ nnIdx[bestIdx] ].setStatus(PS_FRONTPOINT);
			m_fontPoints.push_back(nnIdx[bestIdx]);
		}

		//==== ���link�е�Ԫ�ص�����λ��(���Ϊtrue) ====//
		bool * bLinkedCenter = new bool[N+1];	// Ҳ���������������Щ��
		bLinkedCenter[N] = true;	// ���һ��Ԫ�����һ��Ԫ����ͬ
		for (int k = 0; k < N ; k++) {
			bLinkedCenter[k] = false;
		}
		bLinkedCenter[ii] = true;
		bLinkedCenter[bestIdx] = true;

		triBtwSE(iSeed, nnIdx[ii], nnIdx, bestIdx, N, bLinkedCenter);
		delete[] bLinkedCenter;
	}
}

#define FLT_MINN  -1.0e24F
//==��չ��һ�������Σ����ɹ�������һ�����㣬���ɹ�������0
// icur  ��ǰ��һ����չ���㣻nnidx[N]=nnidx[1]
int CPointMesh::externFirstTriangle(size_t iSeed, ANNidx* nnIdx, int icur, int N){
	// �����һ���������չ��
	float maxTriAngel = FLT_MINN;
	int bestIdx = 0;
	float* ptSeed = m_ps->getPoint(iSeed),
		* ptCur = m_ps->getPoint(nnIdx[icur]);
	for (int t = icur+1; t < N+1; t++){
		POINTSTATUS ps = getPointStatus(nnIdx[t]);
		if ( ps == PS_INPOINT || ps == PS_BOUNDARY) continue;

		float minTriAngel = computeSeedPointFit(ptSeed, ptCur, nnIdx[t]);
		if (minTriAngel == FLT_MINN) break;
		if (minTriAngel > maxTriAngel){
			maxTriAngel = minTriAngel;
			bestIdx = t;
		}
	}
	return bestIdx;
}

//===============================================//
void CPointMesh::externFrontPoint(size_t iSeed){
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
	float* ptSeed = m_ps->m_point[iSeed];
	ANNidxArray nnIdx = new ANNidx[m_searchPointNum+1+linkPtF.size()];	//���1��Ϊ�˺����γ�������ʱ���ûػ�
	int searchPointNum = m_searchPointNum;

	float r2 = maxExistLinkLength*maxExistLinkLength;
	do {//! �˴����ܳ������⣺�������ĵ�̫�ٶ����ܰ���ԭ���Ѿ���Link�еĵ�
		int size = m_ps->KdTree()->annkFRSearch(ptSeed, r2, m_searchPointNum, nnIdx);
		searchPointNum = min(size, m_searchPointNum);
		r2 *= 1.3f;
	} while (searchPointNum < m_searchPointNum);

	//=========�����ߺͲ�����===========//
	int insertStart = searchPointNum;
	for (itLinkPtF = linkPtF.begin(); itLinkPtF != linkPtF.end(); ++itLinkPtF) {
		bool bFind = false;
		for (int i = 1; i < searchPointNum; ++i) {
			if (nnIdx[i] == *itLinkPtF){
				bFind = true;		break;
			}
		}
		if (!bFind){	nnIdx[insertStart++] = *itLinkPtF; }
	}
	searchPointNum = insertStart;

	//=======ȥ���Ѿ�Ϊɾ����Ķ���=======//
	int pointNum = 1;
	ANNidxArray nnidx = new ANNidx[searchPointNum+1];
	for (int m = 1; m < searchPointNum; m++){
		if (m_links[ nnIdx[m] ].getStatus() != PS_DELETE){
			nnidx[m] = nnIdx[m];
			pointNum++;
		}
	}
	delete nnIdx;
	nnIdx = nnidx;
	searchPointNum = pointNum;

	//====== Ѱ����ʼ��������Ƕ� ======//
	size_t startPtIndex = 1;	//! nnIdx[0]Ϊ��ǰ��չ�� iSeed
	if(ptSeedFront.linkSize() > 0)	{// �����һ�����ӵ���nnIdx�е�λ�ã��ҵ���ͶӰ
		size_t firstIndex = ptSeedFront.linkFront();
		for (int i = 1; i < searchPointNum ; i++) {
			if (nnIdx[i] == firstIndex) { startPtIndex = i;	 break;	}
		}
	}
	//===== �γ�����չ��Ϊ���ĵ�ƽ�� =====//
	float* ptNorm = m_ps->m_normal[iSeed];
	CPlane planeAtPt(ptNorm, ptSeed);
	//===== ���������ĵ�ͶӰ��ƽ���� =====//
	float (*projectPt)[3] = new float[searchPointNum][3];// ͶӰ��
	for (int i = 1; i < searchPointNum; i++) {
		planeAtPt.projectPoint( projectPt[i], m_ps->m_point[ nnIdx[i] ] );
	}
	//= ��������������ʼ�ߵļн�(��ʱ�뷽��) =//
	float* arccosa = new float[searchPointNum];	// �Ƕ�����
	for (int j = 1; j < searchPointNum; j++) {
		arccosa[j] = arcNorm(ptSeed, projectPt[startPtIndex], ptNorm, projectPt[j]);
	}
	delete[] projectPt;
	arccosa[startPtIndex] = 0.0f;
	//===== �Ը��㰴����ʱ�뷽������ =====//
	sortArc(arccosa, nnIdx, searchPointNum);
	delete[] arccosa;
	//==== �������ӵ㰴����ʱ�뷽������ ===//
	sortWith(ptSeedFront.getLink(), nnIdx, searchPointNum);
	//========= �γ������� ==========//
	nnIdx[searchPointNum] = nnIdx[1];
	externFrontTriangles(iSeed, nnIdx, searchPointNum);
	delete[] nnIdx;

	if (getPointStatus(iSeed) != PS_BOUNDARY)
		setPointStatus(iSeed, PS_INPOINT);
}

void CPointMesh::externFrontTriangles(size_t iSeed, ANNidx* nnIdx, int N){
	//==== ���link�е�Ԫ�ص�����λ��(���Ϊtrue) ====//
	bool * bLinkedCenter = new bool[N+1];	// Ҳ���������������Щ��
	bLinkedCenter[N] = true;	// ���һ��Ԫ�����һ��Ԫ����ͬ
	for (int k = 0; k < N ; k++) {
		bLinkedCenter[k] = false;
	}

	//=== ����linkList�Ƿ���nnIdx�� ===//
	LinkList& linkList = m_links[iSeed].getLink();
	LSIt it = linkList.begin();
	int j = 1, start = 1;
	for (; it != linkList.end(); ++it) {
		j = start;
		while (j < N && nnIdx[j] != *it ){	j++;	 }
		if (j != N){
			bLinkedCenter[j] = true;
			start = j+1;
		}else break;
	}

	//==����ÿһ������δ���ӵ������==//
	int i = 1;
	j = i + 1;
	while (j < N+1) {
		CPointFront& ptFi = m_links[ nnIdx[i] ];
		if(bLinkedCenter[ j ]){// �������㶼�����ĵ�����
			if (!ptFi.existLink( nnIdx[j] )){// ��������δ����
				addLinkBtw( nnIdx[i], nnIdx[j] );
				CTriangle tri( iSeed, nnIdx[i], nnIdx[j] );
				tri.calcNorm(m_ps);
				m_faces.push_back(tri);
			}
		}else{
			while (++j < N+1){
				if (bLinkedCenter[j] == true)
					break;
			}
			if(!ptFi.existLink(nnIdx[j])){
				size_t iPre = findPrePoint(nnIdx, i, bLinkedCenter, N);
				triBtwSE(iSeed, iPre, nnIdx, i, j, bLinkedCenter);
			}
		}
		i = j;		j++;
	}
	//=======�������ӵ��γɵĶ���Σ���λ���ڲ��Ķ���ɾ��=====//

	delete[] bLinkedCenter;
}

//===============================================//
//************************************************************************
// �Դ�start��ʼ��endΪֹ�Ķ����γ�������
// Parameter: size_t iSeed ���Ķ���
// Parameter: size_t iPre ��iSeed-nnidx[start]���������ε���һ������
// Parameter: const ANNidx * nnidx ����
// Parameter: int start	��ʼ����
// Parameter: int end ��ֹ����
// Parameter: bool * bLinkedCenter �����ĵ�iSeed�����ӹ�ϵ
//************************************************************************
void CPointMesh::triBtwSE(size_t iSeed, size_t iPre, const ANNidx* nnidx, int start, int end, bool* bLinkedCenter){
	int i = start;
	while (i < end){
		// ����iSeed--nnidx[i]�ҵ����ʵĶ���
		int j = formOneTriangle(iSeed, iPre, nnidx, i, end, bLinkedCenter);
		iPre = nnidx[i];
		i = j;
	}
}

// �Ա�iSeed--nnidx[start]�ҵ���һ�����ʵĶ��㣬�γ������Σ������ش˶�����nnidx�е����
int CPointMesh::formOneTriangle(size_t iSeed, size_t iPre, const ANNidx* nnidx, int start, int end, bool* bLinkedCenter){
	size_t iCur = nnidx[start];
	float* ptSeed = m_ps->getPoint(iSeed);
	float* ptPre = m_ps->getPoint(iPre);
	float* ptCur = m_ps->getPoint(iCur);
	float eSeedPre[3] = Edge(ptSeed, ptPre);
	float eSeedCur[3] = Edge(ptSeed, ptCur);
	float ePreNorm[3];	eCross(ePreNorm, eSeedPre, eSeedCur);	// ǰһ�������εķ�ʸ

	//===��������ʵĶ�����չ��ǰ��iSeed-iCur===//
	float maxFit = FLT_MINN;
	int bestIdx = -1;	// ���Ŷ����
	for (int i = start+1; i< end+1; i++){
		size_t iNext = nnidx[i];
		POINTSTATUS ps = getPointStatus(iNext);
		if ( ps == PS_INPOINT || ps == PS_BOUNDARY) continue;

		float iNextFit = computeSeedPointFit(ptSeed, ptCur, iNext, ePreNorm);
		if (iNextFit == FLT_MINN)	break;
		if (iNextFit > maxFit){
			maxFit = iNextFit;
			bestIdx = i;
		}
	}

	//===���ҵ��ĵ��γ������β�����������Ϣ===//
	if (bestIdx != -1){// �ҵ����ʶ���
		bLinkedCenter[bestIdx] = true;
		size_t iSelIndex = nnidx[bestIdx];	// ѡ��Ķ�������
		if(PS_UNUSED==m_links[ iSelIndex ].getStatus()){
			m_links[ iSelIndex ].setStatus(PS_FRONTPOINT);
			m_fontPoints.push_back(iSelIndex);
		}

		if (!m_links[ iSelIndex ].existLink(iSeed))	addLinkBtw(iSeed, iSelIndex);
		if (!m_links[ iSelIndex ].existLink(iCur))		addLinkBtw(iCur, iSelIndex);

		CTriangle tri(iSeed, iCur, iSelIndex);
		tri.calcNorm(m_ps);
		m_faces.push_back(tri);
		return bestIdx;
	}else{	// δ�ҵ����ʶ��㣬�߽��
		m_links[iSeed].setStatus(PS_BOUNDARY);
		return start +1;
	}
}

// �����iNext������ĵ�iSeed����ǰ��iCur������ƽ���Ⱥ�����3.3.2����ʽ3.7��, preNormΪǰһ�������εķ�ʸ
float CPointMesh::computeSeedPointFit(float ptSeed[3], float ptCur[3], size_t iNext, float preNorm[3]){
	// ������������С�ڽǵ�cosq
	float* ptNext = m_ps->getPoint(iNext);
	float eSeedCur[3] = Edge(ptSeed, ptCur),
			 eSeedNext[3] = Edge(ptSeed, ptNext),
			 eCurNext[3] = Edge(ptCur, ptNext);
//	eUnit(eSeedCur); eUnit(eSeedNext); eUnit(eCurNext);

	float angelSeed = 1.0f - eCos(eSeedCur, eSeedNext);
	if (angelSeed > COS160)
		return FLT_MINN;
	float angelCur = 1.0f + eCos(eCurNext, eSeedCur);
	float angelNext = 1.0f - eCos(eSeedCur, eCurNext);
	float minTriAngel = min(angelSeed, angelCur);	// �������ε���С�ڽ�
	minTriAngel = min(minTriAngel, angelNext);//��0��0.5��
	if (minTriAngel < COS25)	return FLT_MINN+20;
	float eNorm[3];		// ����������ķ�ʸ
	eCross(eNorm, eSeedCur, eSeedNext);
//	eUnit(eNorm);

	float angleDihe = eCos(eNorm, preNorm);	// ����Ǵ�С��-1,1��
	return 2*m_A*minTriAngel + m_B* angleDihe;
}

float CPointMesh::computeSeedPointFit(float ptSeed[3], float ptCur[3], size_t iNext){
	// ������������С�ڽǵ�cosq
	float* ptNext = m_ps->getPoint(iNext);
	float eSeedCur[3] = Edge(ptSeed, ptCur),
		eSeedNext[3] = Edge(ptSeed, ptNext),
		eCurNext[3] = Edge(ptCur, ptNext);
	//	eUnit(eSeedCur); eUnit(eSeedNext); eUnit(eCurNext);

	float angelSeed = 1.0f - eCos(eSeedCur, eSeedNext);
	if (angelSeed > COS150)
		return FLT_MINN;
	float angelCur = 1.0f + eCos(eCurNext, eSeedCur);
	float angelNext = 1.0f - eCos(eSeedCur, eCurNext);
	float minTriAngel = min(angelSeed, angelCur);	// �������ε���С�ڽ�
	minTriAngel = min(minTriAngel, angelNext);//��0��0.5��

	return minTriAngel;
}