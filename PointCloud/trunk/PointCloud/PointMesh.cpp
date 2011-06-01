#include "stdafx.h"
#include "PointMesh.h"
#include "PointCloudView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

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
#define MAX_SEARTCH_POINT_NUM  9
	ANNidxArray nnIdx = new ANNidx[MAX_SEARTCH_POINT_NUM+1+linkPtF.size()];	//多加1是为了后续形成三角形时不用回环
	int searchPointNum = MAX_SEARTCH_POINT_NUM;
	if (maxExistLinkLength == 0){	// 当前点为未使用点，没有连接点
		ANNdistArray dist = new ANNdist[MAX_SEARTCH_POINT_NUM];
		m_ps->KdTree()->annkSearch(ptSeed, MAX_SEARTCH_POINT_NUM, nnIdx, dist);
		delete[] dist;
	} else {	// 已经包含连接点
#define LIMITED_POINT_NUM 6//! 要保证searchPointNum至少大于6
		float r2 = maxExistLinkLength*maxExistLinkLength;
		do {//! 此处可能出现问题：搜索到的点太少而不能包含原来已经在Link中的点
			int size = m_ps->KdTree()->annkFRSearch(ptSeed, r2, MAX_SEARTCH_POINT_NUM, nnIdx);
			searchPointNum = min(size, MAX_SEARTCH_POINT_NUM);
			r2 *= 1.3f;
		} while (searchPointNum < LIMITED_POINT_NUM);
		//TODO需要将两者和并起来
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
	arccosa[startPtIndex] = 0.0f;
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

void CPointMesh::formTriangles( size_t index, ANNidx* nnIdx, int N )
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
//========== 采用直接将相邻两点和中心店形成三角形============//
void CPointMesh::formTriangles2(size_t iSeed, ANNidx* nnIdx, int N){
	//= 如果当前扩展点未与任何点连接，添加第一个连接点 =//
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
	//==== 标记link中的元素的所在位置(标记为true) ====//
	bool * bLinkedCenter = new bool[N+1];	// 也用来标记连接了哪些点
	bLinkedCenter[N] = true;	// 最后一个元素与第一个元素相同
	for (int k = 0; k < N ; k++) {
		bLinkedCenter[k] = false;
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
			bLinkedCenter[j] = true;
			start = j+1;
		}else break;
	}
	//==查找每一个包含未连接点的区间==//
	int i = 1;
	j = i + 1;
	while (j < N+1) {
		CPointFront& ptFi = m_links[nnIdx[i]];
		if(bLinkedCenter[ j ]){// 相邻两点都与中心点相连
			if (!ptFi.existLink( nnIdx[j] )){// 相邻两点未连接
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

//! 说明： nnidx[start]和nnidx[end]均与iSeed相连,且start与end不相连
void CPointMesh::formTriangles2(size_t iSeed,const ANNidx* nnidx, int start, int end, bool * bLinkedCenter){
	int i = start, j = i+1;
	// TODO 当扩展点为边界点时，会出现问题：都不满足就将start和end相连了
	while (j < end+1){
		if (j == end){// 此时直接形成形成三角形
			if (nnidx[i] != nnidx[j])
			{	// TODO 需要进行判断是否为边界点
				if ( !m_links[nnidx[j]].existLink(nnidx[i]) )
					addLinkBtw(nnidx[i], nnidx[j]);
				//TODO 还存在一个问题：当前一个三角形形成，但是此三角形却不存在
				CTriangle tri(iSeed, nnidx[i], nnidx[j]);
				tri.calcNorm(m_ps);
				m_faces.push_back(tri);
			}
		}else{
			if (satisfyTriRule(iSeed, nnidx[i], nnidx[j])){
				bLinkedCenter[j] = true;	// 将其设置为iSeed连接点
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

/**************POINTＭESH 贪心算法*************************/
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

//=======采用贪心算法不断查找下一个顶点来和当前边形成三角形======//
void CPointMesh::externSeedPoint(size_t iSeed){
	//====== 搜索邻域范围内的点集 =======//
	float* ptSeed = m_ps->m_point[iSeed];
	ANNidxArray nnidx = new ANNidx[m_searchPointNum+1];//多加1是为了后续形成三角形时不用回环
	ANNdistArray dist = new ANNdist[m_searchPointNum];
	m_ps->KdTree()->annkSearch(ptSeed, m_searchPointNum, nnidx, dist);
	delete[] dist;
	//=======去除已经为删除点的顶点=======//
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

	//===== 计算与平均边长最接近的边 =====//
	float distanceAve = 0.0f;
	float* dists = new float[searchPointNum];
	for (int i = 1; i <searchPointNum; i++){
		dists[i] = m_ps->length(iSeed, nnidx[i]);
		distanceAve += dists[i];
	}
	distanceAve /= (searchPointNum-1);

	int start = 0;		// 计算最接近平均边长的顶点
	float minDist = FLT_MAX;
	for (int j = 1; j < searchPointNum; j++){
		float d = abs(distanceAve - dists[j]);
		if ( d > minDist ){
			minDist = d;
			start = j;
		}
	}
	delete[] dists;

	//===== 形成以扩展点为中心的平面 =====//
	float* ptNorm = m_ps->m_normal[iSeed];
	CPlane planeAtPt(ptNorm, ptSeed);
	//===== 将搜索到的点投影在平面上 =====//
	float (*projectPt)[3] = new float[searchPointNum][3];// 投影点
	for (int i = 1; i < searchPointNum; i++) {
		planeAtPt.projectPoint( projectPt[i], m_ps->m_point[ nnidx[i] ] );
	}

	//= 计算各点相对于起始边的夹角(逆时针方向) =//
	float* arccosa = new float[searchPointNum];	// 角度数组
	for (int j = 1; j < searchPointNum; j++) {
		arccosa[j] = arcNorm(ptSeed, projectPt[start], ptNorm, projectPt[j]);
	}
	delete[] projectPt;
	arccosa[start] = 0.0f;
	//===== 对各点按照逆时针方向排序 =====//
	sortArc(arccosa, nnidx, searchPointNum);
	delete[] arccosa;
	//========= 形成三角形 ==========//
	nnidx[searchPointNum] = nnidx[1];
	externSeedTriangles(iSeed, nnidx, searchPointNum);
	delete[] nnidx;

	if (getPointStatus(iSeed) != PS_BOUNDARY)
		setPointStatus(iSeed, PS_INPOINT);
}

//************************************
// 对顶点iSeed扩展三角形
// Parameter: size_t iSeed 中心点序号
// Parameter: ANNidx * nnIdx 中心点邻域
// Parameter: int N 邻域点个数
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

		//==== 标记link中的元素的所在位置(标记为true) ====//
		bool * bLinkedCenter = new bool[N+1];	// 也用来标记连接了哪些点
		bLinkedCenter[N] = true;	// 最后一个元素与第一个元素相同
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
//==扩展第一个三角形，若成功返回下一个顶点，不成功，返回0
// icur  当前第一个扩展顶点；nnidx[N]=nnidx[1]
int CPointMesh::externFirstTriangle(size_t iSeed, ANNidx* nnIdx, int icur, int N){
	// 计算第一个顶点的扩展点
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
	float* ptSeed = m_ps->m_point[iSeed];
	ANNidxArray nnIdx = new ANNidx[m_searchPointNum+1+linkPtF.size()];	//多加1是为了后续形成三角形时不用回环
	int searchPointNum = m_searchPointNum;

	float r2 = maxExistLinkLength*maxExistLinkLength;
	do {//! 此处可能出现问题：搜索到的点太少而不能包含原来已经在Link中的点
		int size = m_ps->KdTree()->annkFRSearch(ptSeed, r2, m_searchPointNum, nnIdx);
		searchPointNum = min(size, m_searchPointNum);
		r2 *= 1.3f;
	} while (searchPointNum < m_searchPointNum);

	//=========将两者和并起来===========//
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

	//=======去除已经为删除点的顶点=======//
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

	//====== 寻找起始边来计算角度 ======//
	size_t startPtIndex = 1;	//! nnIdx[0]为当前扩展点 iSeed
	if(ptSeedFront.linkSize() > 0)	{// 计算第一个连接点在nnIdx中的位置，找到其投影
		size_t firstIndex = ptSeedFront.linkFront();
		for (int i = 1; i < searchPointNum ; i++) {
			if (nnIdx[i] == firstIndex) { startPtIndex = i;	 break;	}
		}
	}
	//===== 形成以扩展点为中心的平面 =====//
	float* ptNorm = m_ps->m_normal[iSeed];
	CPlane planeAtPt(ptNorm, ptSeed);
	//===== 将搜索到的点投影在平面上 =====//
	float (*projectPt)[3] = new float[searchPointNum][3];// 投影点
	for (int i = 1; i < searchPointNum; i++) {
		planeAtPt.projectPoint( projectPt[i], m_ps->m_point[ nnIdx[i] ] );
	}
	//= 计算各点相对于起始边的夹角(逆时针方向) =//
	float* arccosa = new float[searchPointNum];	// 角度数组
	for (int j = 1; j < searchPointNum; j++) {
		arccosa[j] = arcNorm(ptSeed, projectPt[startPtIndex], ptNorm, projectPt[j]);
	}
	delete[] projectPt;
	arccosa[startPtIndex] = 0.0f;
	//===== 对各点按照逆时针方向排序 =====//
	sortArc(arccosa, nnIdx, searchPointNum);
	delete[] arccosa;
	//==== 对已连接点按照逆时针方向排序 ===//
	sortWith(ptSeedFront.getLink(), nnIdx, searchPointNum);
	//========= 形成三角形 ==========//
	nnIdx[searchPointNum] = nnIdx[1];
	externFrontTriangles(iSeed, nnIdx, searchPointNum);
	delete[] nnIdx;

	if (getPointStatus(iSeed) != PS_BOUNDARY)
		setPointStatus(iSeed, PS_INPOINT);
}

void CPointMesh::externFrontTriangles(size_t iSeed, ANNidx* nnIdx, int N){
	//==== 标记link中的元素的所在位置(标记为true) ====//
	bool * bLinkedCenter = new bool[N+1];	// 也用来标记连接了哪些点
	bLinkedCenter[N] = true;	// 最后一个元素与第一个元素相同
	for (int k = 0; k < N ; k++) {
		bLinkedCenter[k] = false;
	}

	//=== 设置linkList是否在nnIdx中 ===//
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

	//==查找每一个包含未连接点的区间==//
	int i = 1;
	j = i + 1;
	while (j < N+1) {
		CPointFront& ptFi = m_links[ nnIdx[i] ];
		if(bLinkedCenter[ j ]){// 相邻两点都与中心点相连
			if (!ptFi.existLink( nnIdx[j] )){// 相邻两点未连接
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
	//=======计算连接点形成的多边形，将位于内部的顶点删除=====//

	delete[] bLinkedCenter;
}

//===============================================//
//************************************************************************
// 对从start开始到end为止的顶点形成三角形
// Parameter: size_t iSeed 中心顶点
// Parameter: size_t iPre 边iSeed-nnidx[start]所在三角形的另一个顶点
// Parameter: const ANNidx * nnidx 邻域
// Parameter: int start	起始顶点
// Parameter: int end 终止顶点
// Parameter: bool * bLinkedCenter 与中心点iSeed的连接关系
//************************************************************************
void CPointMesh::triBtwSE(size_t iSeed, size_t iPre, const ANNidx* nnidx, int start, int end, bool* bLinkedCenter){
	int i = start;
	while (i < end){
		// 给边iSeed--nnidx[i]找到合适的顶点
		int j = formOneTriangle(iSeed, iPre, nnidx, i, end, bLinkedCenter);
		iPre = nnidx[i];
		i = j;
	}
}

// 对边iSeed--nnidx[start]找到下一个合适的顶点，形成三角形，并返回此顶点在nnidx中的序号
int CPointMesh::formOneTriangle(size_t iSeed, size_t iPre, const ANNidx* nnidx, int start, int end, bool* bLinkedCenter){
	size_t iCur = nnidx[start];
	float* ptSeed = m_ps->getPoint(iSeed);
	float* ptPre = m_ps->getPoint(iPre);
	float* ptCur = m_ps->getPoint(iCur);
	float eSeedPre[3] = Edge(ptSeed, ptPre);
	float eSeedCur[3] = Edge(ptSeed, ptCur);
	float ePreNorm[3];	eCross(ePreNorm, eSeedPre, eSeedCur);	// 前一个三角形的法矢

	//===查找最合适的顶点扩展当前边iSeed-iCur===//
	float maxFit = FLT_MINN;
	int bestIdx = -1;	// 最优顶点的
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

	//===将找到的点形成三角形并更新连接信息===//
	if (bestIdx != -1){// 找到合适顶点
		bLinkedCenter[bestIdx] = true;
		size_t iSelIndex = nnidx[bestIdx];	// 选择的顶点的序号
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
	}else{	// 未找到合适顶点，边界点
		m_links[iSeed].setStatus(PS_BOUNDARY);
		return start +1;
	}
}

// 计算点iNext相对中心点iSeed、当前点iCur的正则平滑度函数（3.3.2：公式3.7）, preNorm为前一个三角形的法矢
float CPointMesh::computeSeedPointFit(float ptSeed[3], float ptCur[3], size_t iNext, float preNorm[3]){
	// 计算三角形最小内角的cosq
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
	float minTriAngel = min(angelSeed, angelCur);	// 新三角形的最小内角
	minTriAngel = min(minTriAngel, angelNext);//【0，0.5】
	if (minTriAngel < COS25)	return FLT_MINN+20;
	float eNorm[3];		// 新三角形面的法矢
	eCross(eNorm, eSeedCur, eSeedNext);
//	eUnit(eNorm);

	float angleDihe = eCos(eNorm, preNorm);	// 二面角大小【-1,1】
	return 2*m_A*minTriAngel + m_B* angleDihe;
}

float CPointMesh::computeSeedPointFit(float ptSeed[3], float ptCur[3], size_t iNext){
	// 计算三角形最小内角的cosq
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
	float minTriAngel = min(angelSeed, angelCur);	// 新三角形的最小内角
	minTriAngel = min(minTriAngel, angelNext);//【0，0.5】

	return minTriAngel;
}