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

PointSet::PointSet(void)
{
	m_pointN = 0;
	m_point = 0;
	m_normal = 0;
	m_kdTree = 0;
	m_weight = 0;
}

PointSet::~PointSet(void)
{
	clear();
}

void PointSet::clear(){
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

void PointSet::readPts(char* name){
	FILE* ptsFile;
	if (fopen_s(&ptsFile, name, "r")!=0) return;
	clear();

	fscanf_s(ptsFile, "%d\n", &m_pointN);
	m_point = annAllocPts(m_pointN, 3);

	int i = 0;
	float a, b, c;
	while (i < m_pointN && !feof(ptsFile)){
		fscanf_s(ptsFile, "%f %f %f\n", &a, &b, &c);
		m_point[i][0] = a;
		m_point[i][1] = b;
		m_point[i][2] = c;
		i++;
	}
	fclose(ptsFile);
}

void PointSet::constructKdTree(){
	if (m_kdTree!=0)
		delete m_kdTree;

	m_kdTree = new ANNkd_tree(m_point, m_pointN, 3);
}

void PointSet::computeNormal(int K){
	if (m_kdTree==0)
		constructKdTree();
	if (m_normal !=0)
		delete[] m_normal;
	m_normal = new float[m_pointN][3];

	const double eps = 0.0000001;
	ANNidxArray nnIdx = new ANNidx[K+1];
	ANNdistArray dists = new ANNdist[K+1];
	int** kNeighbors = new int*[m_pointN];		// K�����
	float* ratioA = new float[m_pointN];				// ������ʱ�

	for (int i = 0; i < m_pointN ; i++){
		ANNpoint queryPt = m_point[i];
		m_kdTree->annkSearch(queryPt, K+1, nnIdx, dists, eps);

		float ratio[3], c1,rate;
		computeParamRatio(ratio, m_point, nnIdx, K+1);
		computePtNormAndRate(m_normal[i], c1, rate, ratio, queryPt);

		// TODO:: �˴�����ҪNeighbors��
		kNeighbors[i] = new int[K];
		double w = 0;
		for (int j = 1; j < K+1 ; j++){
			w += dists[j];
			kNeighbors[i][j-1] = nnIdx[j];
		}
		setWeight(i, w/K);	//+K������ƽ������+
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
	+���������Ƶķ�ʸ���е���(���������������)+
	+���������kNeighborhs�����ϵ��normPts��Ҫ�����ķ�ʸ+
	+���������K���������nPts��������������������normPts+
	+ԭ����������򹤳��е���������������ʸ��������㷨�о�+
*/
void PointSet::adjustNormal(int** kNeighbors, int K)
{
	const float eps = 0.97;// �н�����ƽ������ֵ(0.85~0.97)
	int from = findSeedIdx(kNeighbors, K, eps);

	if (from == -1) from = 0;		// δ���ҵ����ӣ���Ҫ����eps��ֵ

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
			if (bAdjusted[ kNeighbors[from][j] ])  continue;//+�Ѿ��������ӹ�+

			float* normj = m_normal[ kNeighbors[from][j] ];
			float cos2a ;
			if (needAdjust(cos2a, norm, normj)) flipNorm(normj);	//+����ǰ��������Ҫ�����ĵ�ȫ������+
			bAdjusted[ kNeighbors[from][j] ] = true;
			sum++;
			if (cos2a > biggestCos2){
				biggestCos2 = cos2a;
				to = kNeighbors[from][j];
			}
		}

		if (to == -1){	//+�������Ѿ�ȫ��������+
			from = findNeedAdjust(from, kNeighbors, bAdjusted, K);
		}else {//+����Ҫ������+
			from = to;
		}
	}
	delete[] bAdjusted;
}

/*	���ҵ��Ʒ�ʸ��������ʼ������ (��ǰ�㷨ʸ�������������ķ�ʸ��cosa>THRESHOLD)
	���������kNeighbors[nPts][K]:��������, K:��������, eps:�н�����ƽ������ֵ(0.85~0.97)
	������������������ĵ�ı��,����޷���-1
	ԭ����������򹤳��е���������������ʸ��������㷨�о� */
int PointSet::findSeedIdx(int** kNeighbors, int K, float eps){
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
	+���ҵ�ǰ�Ѿ�������������������Ҫ������+
*/
int PointSet::findNeedAdjust(int from, int** kNeighbors, bool* bAdjusted, int K){
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

void PointSet::simplePts(int** kNeighbors, int K){
	bool* beSimpledByCell = new bool[m_pointN];

	simplyByCell(beSimpledByCell, 8.f, m_point, m_pointN);

	removePts(beSimpledByCell, NULL);
	delete[] beSimpledByCell;
}

int PointSet::simplyByRatio(bool* beSimpled, int** kNeighbors, int K, float* ratio){
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
// ���ݱ�־���е��ƾ���
//************************************
void PointSet::removePts(bool* bSimpled, bool* bSimpled2){
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
	m_kdTree =0;
	m_point = dataPts;
	m_normal = normal;
	m_weight = weight;
}

/*	���AB*AC�õ����淨ʸ��ABC��ƽ����ʸ�����Ƿ���ͬ	*/
bool PointSet::checkConsistence(int va, int vb, int vc){
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