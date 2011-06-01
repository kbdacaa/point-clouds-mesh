#include "StdAfx.h"
#include "PointSet.h"
#include "CellData.h"

template <class T>
T** allocTArray(int nRow, int nCol){
	T** data = new T*[nRow];
	for (int i = 0; i < nRow ; i++){
		data[i] = new T[nCol];
	}
	return data;
}

template <class T>
void freeTArray(T** data, int nRow){
	if (data != NULL){
		for (int i = 0; i < nRow ; i++){
			delete[] data[i];
		}
		delete[] data;
	}
}

CPointSet::CPointSet(size_t N)
{
	m_pointN = N;
	m_point = 0;
	m_normal = 0;
	m_kdTree = 0;
	m_weight = 0;
	if (N> 0)
		m_point = annAllocPts(m_pointN, 3);
}

CPointSet::~CPointSet(void)
{
	clear();
}

void CPointSet::clear(){
	if (m_point!=0)
		annDeallocPts(m_point);
	if (m_normal!=0)
		delete[] m_normal;
	if (m_weight!=0)
		delete[] m_weight;
	if (m_kdTree!=0)
		delete m_kdTree;
	m_pointN = 0;
	m_point = 0;
	m_normal = 0;
	m_weight = 0;
	m_kdTree = 0;
}

void CPointSet::readPts(char* name){
	FILE* ptsFile;
	if (fopen_s(&ptsFile, name, "r")!=0) return;
	clear();

	fscanf_s(ptsFile, "%d\n", &m_pointN);
	m_point = annAllocPts(m_pointN, 3);
	m_weight = new float[m_pointN];

	int i = 0;
	float a, b, c;
	while (i < m_pointN && !feof(ptsFile)){
		fscanf_s(ptsFile, "%f %f %f\n", &a, &b, &c);
		m_point[i][0] = a;
		m_point[i][1] = b;
		m_point[i][2] = c;
		m_weight[i] = 1.0f;
		i++;
	}
	fclose(ptsFile);
}

void CPointSet::constructKdTree(){
	if (m_kdTree!=0)
		delete m_kdTree;

	m_kdTree = new ANNkd_tree(m_point, m_pointN, 3);
}

void CPointSet::computeNormalAndSimpled(int K){
	if (m_kdTree==0)
		constructKdTree();
	if (m_normal !=0)
		delete[] m_normal;
	m_normal = new float[m_pointN][3];

	const double eps = 0.0000001;
	ANNidxArray nnIdx = new ANNidx[K+1];
	ANNdistArray dists = new ANNdist[K+1];
	int** kNeighbors = new int*[m_pointN];		// K邻域表
	float* ratioA = new float[m_pointN];				// 点的曲率表

	for (int i = 0; i < m_pointN ; i++){
		ANNpoint queryPt = m_point[i];
		m_kdTree->annkSearch(queryPt, K+1, nnIdx, dists, eps);

		float ratio[3], c1,rate;
		computeParamRatio(ratio, m_point, nnIdx, K+1);
		computePtNormAndRate(m_normal[i], c1, rate, ratio, queryPt);

		// TODO:: 此处还需要Neighbors表
		kNeighbors[i] = new int[K];
		double w = 0;
		for (int j = 1; j < K+1 ; j++){
			w += dists[j];
			kNeighbors[i][j-1] = nnIdx[j];
		}
		setWeight(i, (float)w/K);	//+K邻域点的平均距离+
		ratioA[i] = rate;
	}
	delete[] nnIdx;
	delete[] dists;

	adjustNormal(kNeighbors, K);

	simplePts(kNeighbors, K);

	delete[] ratioA;
	freeTArray(kNeighbors, m_pointN);
}

/*
	+对整个点云的法矢进行调整(多个调整单个传播)+
	+输入参数：kNeighborhs邻域关系表，normPts需要调整的法矢+
	+输入参数：K邻域个数，nPts点云总数；输出调整后的normPts+
	+原理详见：逆向工程中点云邻域搜索及法矢估算相关算法研究+
*/
void CPointSet::adjustNormal(int** kNeighbors, int K)
{
	const float eps = 0.97f;// 夹角余弦平方的阈值(0.85~0.97)
	int from = findSeedIdx(kNeighbors, K, eps);

	if (from == -1) from = 0;		// 未能找到种子，需要减少eps的值

	bool* bAdjusted = new bool[m_pointN];
	for (int i = 0; i < m_pointN ; i++){
		bAdjusted[i] = false;
	}

	bAdjusted[from] = true;
	int sum = 1;

	while (sum < m_pointN && from != -1){
		float* norm = m_normal[from];
		float biggestCos2 = 0;
		int to = -1;

		for (int j = 0; j < K ; j++){
			if (bAdjusted[ kNeighbors[from][j] ])  continue;//+已经调整，掠过+

			float* normj = m_normal[ kNeighbors[from][j] ];
			float cos2a ;
			if (needAdjust(cos2a, norm, normj)) flipNorm(normj);	//+将当前邻域内需要调整的点全部调整+
			bAdjusted[ kNeighbors[from][j] ] = true;
			sum++;
			if (cos2a > biggestCos2){
				biggestCos2 = cos2a;
				to = kNeighbors[from][j];
			}
		}

		if (to == -1){	//+其邻域已经全部调整了+
			from = findNeedAdjust(from, kNeighbors, bAdjusted, K);
		}else {//+有需要调整的+
			from = to;
		}
	}
	delete[] bAdjusted;
}

/*	查找点云法矢调整的起始点的序号 (当前点法矢和其所有邻域点的法矢的cosa>THRESHOLD)
	输入参数：kNeighbors[nPts][K]:点的邻域表, K:邻域点个数, eps:夹角余弦平方的阈值(0.85~0.97)
	输出参数：满足条件的点的编号,如果无返回-1
	原理详见：逆向工程中点云邻域搜索及法矢估算相关算法研究 */
int CPointSet::findSeedIdx(int** kNeighbors, int K, float eps){
	int i = 0;
	while (i++ < m_pointN){
		float* norm = m_normal[i];
		int j = 0;
		bool bFinded = true;
		do {
			float* normNeighbor = m_normal[kNeighbors[i][j]];
			if (cos2Norm(norm, normNeighbor) < eps){
				bFinded = false;
				break;
			}
			j++;
		} while (j < K);
		if (bFinded)
			return i;
	}
	return -1;
}

/*
	+查找当前已经调整过，但其邻域还需要调整的+
*/
int CPointSet::findNeedAdjust(int from, int** kNeighbors, bool* bAdjusted, int K){
	for (int i = from+1; i < m_pointN ; i++){
		if (bAdjusted[i]){
			for (int j =0; j < K; j++){
				if (!bAdjusted[kNeighbors[i][j]])
					return i;
			}
		}
	}
	for (int i = 0; i < from ; i++){
		if (bAdjusted[i]){
			for (int j =0; j < K; j++){
				if (!bAdjusted[kNeighbors[i][j]])
					return i;
			}
		}
	}
	return -1;
}

void CPointSet::simplePts(int** kNeighbors, int K){
	bool* beSimpledByCell = new bool[m_pointN];

	simplyByCell(beSimpledByCell, 8.f, m_point, m_pointN);

	removePts(beSimpledByCell, NULL);
	delete[] beSimpledByCell;
}

int CPointSet::simplyByRatio(bool* beSimpled, int** kNeighbors, int K, float* ratio){
	float ratioSum = 0;
	for (int i = 0; i < m_pointN ; i++){
// 		beSimpled[i] = false;
		ratio[i] = fabs(ratio[i]);
		ratioSum += ratio[i];
	}

	float ratioAve = ratioSum / m_pointN;
	for (int i = 0; i < m_pointN ; i++){
		beSimpled[i] = (ratio[i] < ratioAve) ? true : false;
	}

// 	std::sort(ratio, ratio+m_pointN);

	return -1;
}

//************************************
// 根据标志进行点云精简
//************************************
void CPointSet::removePts(bool* bSimpled, bool* bSimpled2){
	assert(bSimpled != NULL);
	int pointN = 0;
	if (bSimpled2 == NULL){
		for (int i = 0; i < m_pointN ; i++){
			if (bSimpled[i]==false) pointN ++;
		}
	} else {
		for (int i = 0; i < m_pointN ; i++){
			if (bSimpled[i] == false || bSimpled2[i] == false)
				pointN++;
		}
	}
	float** dataPts = annAllocPts(pointN, 3);
	float (*normal)[3] = new float[pointN][3];
	float *weight = new float[pointN];
	int k= 0;
	for (int i = 0; i < m_pointN ; i++){
		if (bSimpled[i] == false){
			dataPts[k][0] = m_point[i][0];
			dataPts[k][1] = m_point[i][1];
			dataPts[k][2] = m_point[i][2];
			normal[k][0] = m_normal[i][0];
			normal[k][1] = m_normal[i][1];
			normal[k][2] = m_normal[i][2];
			weight[k] = m_weight[i];
			k++;
		}
	}
	m_pointN = pointN;
	annDeallocPts(m_point);
	delete[] m_normal;
	delete[] m_weight;
	if (m_kdTree!=0)
		delete m_kdTree;
	m_kdTree = 0;
	m_point  = dataPts;
	m_normal = normal;
	m_weight = weight;
}

/*	检查AB*AC得到的面法矢和ABC的平均法矢方向是否相同	*/
bool CPointSet::checkConsistence(int va, int vb, int vc){
	assert(m_normal != 0);
	float* normA = m_normal[va];
	float* normB = m_normal[vb];
	float* normC = m_normal[vc];

	float AB[3] = {normB[0]-normA[0], normB[1]-normA[1], normB[2]-normA[2]};
	float AC[3] = {normC[0]-normA[0], normC[1]-normA[1], normC[2]-normA[2]};

	float normFace[3];	// AB*AC
	normFace[0] = AB[1]*AC[2] - AB[2]*AC[1];
	normFace[1] = AB[2]*AC[0] - AB[0]*AC[2];
	normFace[2] = AB[0]*AC[1] - AB[1]*AC[0];

	float normABC[3];
	normABC[0] = normA[0] + normB[0] + normC[0];
	normABC[1] = normA[1] + normB[1] + normC[1];
	normABC[2] = normA[2] + normB[2] + normC[2];
	float cosAngle = normFace[0]*normABC[0]+normFace[1]*normABC[1]+normFace[2]*normABC[2];
	return (cosAngle >= 0);
}

//===============For Mesh Usage==========================//

float CPointSet::getMinTriAngel( const int& idxA, const int& idxB, const int& idxC )
{
	float* ptA = m_point[idxA],
		* ptB = m_point[idxB],
		* ptC = m_point[idxC];
	float eAB[3] = {ptB[0] - ptA[0], ptB[1] - ptA[1], ptB[2] - ptA[2] },
		eAC[3] = { ptC[0] - ptA[0], ptC[1] - ptA[1], ptC[2] - ptA[2] },
		eBC[3] = { ptC[0] - ptB[0], ptC[1] - ptB[1], ptC[2] - ptB[2] };

	eUnit(eAB);	eUnit(eAC);	eUnit(eBC);

	float angelA = 1.0f - eCos(eAB, eAC);
	float angelB = 1.0f + eCos(eBC, eAB);
	float angelC = 1.0f - eCos(eAC, eBC);

	float minAB = min(angelA, angelB);
	return min(minAB, angelC);
}

// ===== USED FOR POINTMESH ===== //
void CPointSet::computeNormalWithCV(float norm[3], ANNidx* listIndex, int N){
	float** point = m_point;
	//+当前N个点的中心点+
	float cx = 0, cy = 0, cz = 0;

	int i  = 0;
	for (i = 0; i < N; i++) {
		float *p = point[listIndex[i]];
		cx += p[0];
		cy += p[1];
		cz += p[2];
	}
	float Ni = 1.0f / N;
	cx *= Ni;
	cy *= Ni;
	cz *= Ni;

	//data for Jacobi method
	double **A = new double *[4];
	double **v = new double *[4];
	double w[4];
	int nrot;
	for (i = 1; i < 4; i++) {
		A[i] = new double[4];
		A[i][1] = A[i][2] = A[i][3] = 0;
		v[i] = new double[4];
	}

	//CV matrix
	for (i = 0; i < N; i++) {
		float *p = point[listIndex[i]];

		float vx = p[0] - cx;
		float vy = p[1] - cy;
		float vz = p[2] - cz;

		A[1][1] += vx * vx;
		A[1][2] += vx * vy;
		A[1][3] += vx * vz;

		A[2][2] += vy * vy;
		A[2][3] += vy * vz;

		A[3][3] += vz * vz;
	}
	A[2][1] = A[1][2];
	A[3][1] = A[1][3];
	A[3][2] = A[3][2];

	Jacobi::jacobi(A, 3, w, v, &nrot);

	int mini;
	if (fabs(w[1]) < fabs(w[2]))
		mini = 1;
	else
		mini = 2;
	if (fabs(w[mini]) > fabs(w[3]))
		mini = 3;

	norm[0] = (float)v[1][mini];
	norm[1] = (float)v[2][mini];
	norm[2] = (float)v[3][mini];

	for (i = 1; i < 4; i++) {
		delete []A[i];
		delete []v[i];
	}
	delete []A;
	delete []v;
}

void CPointSet::computeNormal(int K){
	if (m_normal !=0)
		return ;
	m_normal = new float[m_pointN][3];
	if (m_kdTree==0)
		constructKdTree();
	ANNidxArray nnIdx = new ANNidx[K+1];
	ANNdistArray dists = new ANNdist[K+1];

	for (int i = 0; i < m_pointN ; i++) {
		float* queryPt = m_point[i];
		m_kdTree->annkSearch(queryPt, K+1, nnIdx, dists);
		float normal[3];						// 点的法矢
		computeNormalWithCV(normal, nnIdx, K+1);
		m_normal[i][0] = normal[0];
		m_normal[i][1] = normal[1];
		m_normal[i][2] = normal[2];
	}
	delete[] nnIdx;
	delete[] dists;
}

void CPointSet::computeWeightAndNormal(int K){
	if (m_normal !=0)
		return ;
	m_normal = new float[m_pointN][3];
	if (m_weight == NULL)
		m_weight = new float[m_pointN];
	if (m_kdTree==0)
		constructKdTree();
	ANNidxArray nnIdx = new ANNidx[K+1];
	ANNdistArray dists = new ANNdist[K+1];

	for (int i = 0; i < m_pointN ; i++) {
		float* queryPt = m_point[i];
		m_kdTree->annkSearch(queryPt, K+1, nnIdx, dists);
		double distSum = 0;
		for (int j = 1; j < K+1 ; j++) {
			distSum += dists[j];
		}
		m_weight[i] = (float)distSum / K;	// 点的权重
		float normal[3];						// 点的法矢
		computeNormalWithCV(normal, nnIdx, K+1);
		m_normal[i][0] = normal[0];
		m_normal[i][1] = normal[1];
		m_normal[i][2] = normal[2];
	}
	delete[] nnIdx;
	delete[] dists;
}

//=========独立的法矢调整方法==========//
void CPointSet::adjustNormal(int K){
	if (m_kdTree == 0)
		m_kdTree = new ANNkd_tree(m_point, m_pointN, 3);

	ANNidxArray nnIdx = new ANNidx[K+1];
	ANNdistArray dists = new ANNdist[K+1];

	int from = rand()%m_pointN;
	int adjustPointNum = 1;

	vector<bool> bNormalAdjusted;
	bNormalAdjusted.resize(m_pointN, false);
	bNormalAdjusted[from] = true;

	while (adjustPointNum < m_pointN && from != -1){
		float* normCenter = m_normal[from];
		float biggestCos2 = 0;
		int to = -1;

		ANNpoint queryPt = m_point[from];
		m_kdTree->annkSearch(queryPt, K+1, nnIdx, dists);

		for (int j = 1; j < K+1; j ++){
			if (bNormalAdjusted[ nnIdx[j] ]) continue;

			float* normalAdjust = m_normal[ nnIdx[j] ];
			float cos2a;
			if ( needAdjust(cos2a, normCenter, normalAdjust) ) flipNorm(normalAdjust);
			bNormalAdjusted[ nnIdx[j] ] = true;
			adjustPointNum++;
			if (cos2a > biggestCos2){
				biggestCos2 = cos2a;
				to = nnIdx[j];
			}
		}

		if (to == -1){
			from = findNeedAdjust(from, nnIdx, dists, bNormalAdjusted, K);
		}else from = to;
	}

	bNormalAdjusted.clear();
	delete[] nnIdx;
	delete[] dists;
}

int CPointSet::findNeedAdjust(int from, ANNidxArray nnidx, ANNdistArray dists, vector<bool>& bNoramlAdjusted, int K){
	for (int i = from+1; i < m_pointN; i++) {
		if (bNoramlAdjusted[ i ]){
			ANNpoint queryPt = m_point[i];
			m_kdTree->annkSearch(queryPt, K+1, nnidx, dists);
			for (int j = 1; j < K+1; j++){
				if (!bNoramlAdjusted[ nnidx[j] ])	return i;
			}
		}
	}
	for (int i = 0; i < from; i++) {
		if (bNoramlAdjusted[ i ]){
			ANNpoint queryPt = m_point[i];
			m_kdTree->annkSearch(queryPt, K+1, nnidx, dists);
			for (int j = 1; j < K+1; j++){
				if (!bNoramlAdjusted[ nnidx[j] ])	return i;
			}
		}
	}
	return -1;
}