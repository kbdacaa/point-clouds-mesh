#ifndef SIMPLYPLUGIN
#define SIMPLYPLUGIN

#include "pointset.h"
#include "Common/Otree.h"
// 计算两个顶点之间距离的平方
inline double distance(float* pta, float*ptb){
	return (pta[0]-ptb[0])*(pta[0]-ptb[0]) +
		(pta[1]-ptb[1])*(pta[1]-ptb[1]) +
		(pta[2]-ptb[2])*(pta[2]-ptb[2]);
}

class CSimplyPlugin{
public:
	CPointSet* m_ps;
	int m_M;		// 八叉树单元中叶子节点的最大点容量
	int m_K;		// 邻域搜索时的K

private:
	float* m_ratio;
	float  m_rateAve;
	bool* m_bDelete;
public:
	CSimplyPlugin(CPointSet* ps, int Me=15, int Ke=15);
public:
	void doSimply();

	void simplyOctreeCell(vector<int>& ptsVector);
	void removePts();
};

CSimplyPlugin::CSimplyPlugin(CPointSet* ps, int Me, int Ke){
	m_ps = ps;
	m_M = Me;
	m_K = Ke;
	m_ratio = NULL;
	m_bDelete = NULL;
}

inline void CSimplyPlugin::doSimply(){
	size_t psSize = m_ps->getPointSize();
	if (m_ratio == NULL)
		m_ratio = new float[psSize];

	ANNidxArray nnidx = new ANNidx[m_K+1];
	ANNdistArray nndist = new ANNdist[m_K+1];
	ANNkd_tree* kdTree = new ANNkd_tree(m_ps->m_point, m_ps->m_pointN, 3);

	m_rateAve = 0.0f;
	for (size_t i = 0; i < psSize; i++) {
		ANNpoint qPt = m_ps->getPoint(i);
		kdTree->annkSearch(qPt, m_K+1, nnidx, nndist);
		// 计算顶点曲率
		float ratio[3], c1,rate, nor[3];
		computeParamRatio(ratio, m_ps->m_point, nnidx, m_K+1);
		computePtNormAndRate(nor, c1, rate, ratio, qPt);
		rate = abs(rate);
		m_ratio[i] = rate;
		m_rateAve += rate;
	}
	m_rateAve /= psSize;
	delete kdTree;
	delete[] nnidx;
	delete[] nndist;

	SplitNode* octreeRoot = createOctTree<int>(m_ps, m_M);
	delete octreeRoot;

	m_bDelete = new bool[psSize];
	for (size_t t = 0; t < psSize; t++)
		m_bDelete[t] = true;
	// 对每一个八叉树叶子节点进行精简
	//cout<<"八叉树叶子个数："<<gLeafNodeVector.size()<<endl;
	vector<LeafNode<int>*>::iterator it = gLeafNodeVector.begin();
	for (; it != gLeafNodeVector.end(); it++)
		if ((*it)->level > 5)
			simplyOctreeCell((*it)->ptIndex);

	delete[] m_ratio;
	m_ratio = NULL;
	removePts();
	delete[] m_bDelete;
	m_bDelete = NULL;
}

void CSimplyPlugin::simplyOctreeCell(vector<int>& ptsVector){
	int pts = ptsVector.size();
	if (pts == 1) {
		int ptIdx = ptsVector[0];
		m_bDelete[ptIdx] = false;
		return;
	}

	float rateAve = 0.0f;	// 网格中点的曲率平均值Et
	float ptCenter[3] = {0.0f, 0.0f, 0.0f};
	for (int i = 0; i < pts; i ++){
		int ptIndex =  ptsVector[i];
		rateAve += m_ratio[ptIndex];
		float* pt = m_ps->getPoint(ptIndex);
		ptCenter[0] += pt[0];
		ptCenter[1] += pt[1];
		ptCenter[2] += pt[2];
	}
	rateAve /= pts;
	ptCenter[0] /= pts;
	ptCenter[1] /= pts;
	ptCenter[2] /= pts;

	double MinD2 = DBL_MAX;
	int idxMin = -1;	// 最接近重心的顶点
	for (int j =0; j < pts; j++){
		double d2 = distance(ptCenter, m_ps->getPoint( ptsVector[j] ));
		if (MinD2 > d2){
			MinD2 = d2;
			idxMin = ptsVector[j];
		}
	}
	m_bDelete[idxMin] = false;

	if (rateAve > m_rateAve){
		for (int t = 0; t < pts; t++) {
			int ptIdx = ptsVector[t];
			m_bDelete[ptIdx] = false;
		}
		return;
	}
	if (rateAve > m_rateAve*0.1)	{
		int idxMax = ptsVector[0], idxMax2 = idxMax;
		float rateMax = m_ratio[idxMax];
		int m = 0;

		for (int k = 1; k < pts; k++){
			int ptIdx = ptsVector[k];
			if (rateMax < m_ratio[ptIdx]){
				rateMax = m_ratio[ptIdx];
				idxMax2 = idxMax;
				idxMax = ptIdx;
				m = k;
			}
		}
		m_bDelete[idxMax] = false;

		if (rateAve > m_rateAve*0.4)	{
			for (int n = m+1; n < pts; n++){
				int ptI = ptsVector[n];
				if (m_ratio[idxMax2] < m_ratio[ptI]){
					idxMax2 = ptI;
				}
			}
			m_bDelete[idxMax2] = false;

// 			if (rateAve > m_rateAve*0.7) {
// 			}
		}
	}
}

inline void CSimplyPlugin::removePts(){
	if (m_bDelete == NULL)
		return ;

	size_t ptSimN = 0;
	size_t ptSize = m_ps->getPointSize();
	for (size_t j = 0; j < ptSize ; j++) {
		if (!m_bDelete[j]) ptSimN ++;
	}
	if (ptSimN == ptSize) return ;

	size_t sT = 0;
	ANNpointArray dataPts = annAllocPts(ptSimN, 3);
	float (*normal)[3] = NULL;
	float *weight = NULL;
	if (m_ps->m_normal != NULL)
		normal = new float[ptSimN][3];
	if (m_ps->m_weight != NULL)
		weight = new float[ptSimN];

	for (size_t i = 0; i < ptSize; i++){
		if (!m_bDelete[i]){
			dataPts[sT][0] = m_ps->m_point[i][0];
			dataPts[sT][1] = m_ps->m_point[i][1];
			dataPts[sT][2] = m_ps->m_point[i][2];
			if (normal != NULL){
				normal[sT][0] = m_ps->m_normal[i][0];
				normal[sT][1] = m_ps->m_normal[i][1];
				normal[sT][2] = m_ps->m_normal[i][2];
			}
			if (weight != NULL)
				weight[sT] = m_ps->m_weight[i];
			sT++;
		}
	}
	annDeallocPts(m_ps->m_point);
	m_ps->m_pointN = ptSimN;
	m_ps->m_point = dataPts;

	if (normal != NULL){
		delete[] m_ps->m_normal;
		m_ps->m_normal = normal;
	}
	if (weight != NULL){
		delete[] m_ps->m_weight;
		m_ps->m_weight = weight;
	}
}

#endif // SIMPLYPLUGIN