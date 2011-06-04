#ifndef PointSet_H_
#define PointSet_H_

#pragma once
#include "Common.h"
#include <assert.h>
#include <stdio.h>
#include <vector>
using namespace std;

//===========1-cos(a)=========//
#define COS15 0.0340741737109317f
#define COS20 0.0603073792140916f
#define COS25 0.0936922129633500f
#define COS30 0.1339745962155613f
#define COS35 0.1808479557110082f
#define COS40 0.23395555688102196f
#define COS45 0.29289321881345247f
#define COS50 0.35721239031346067f
#define COS60 0.5f
#define COS80 0.82635182233306965f
#define COS90 1.0f
#define COS120 1.5f
#define COS150 1.8660254037844386f
#define COS160 1.93969262078590838f
#define COS170 1.98480775301220806F
#define COS180 2.0F

#define Edge(ptA, ptB) { ptB[0] - ptA[0], ptB[1] - ptA[1], ptB[2] - ptA[2] }

// ��x��ƽ����ȡ����
inline float InvSqrt(float x){
	float xHalf = 0.5f*x;
	int i = *(int*)&x;
	i = 0x5f3759df - (i >>1);
	x = *(float*)&i;
	x = x*(1.5f - xHalf*x*x);
	//x = x*(1.5f - xHalf*x*x); // �ڶ��ε������������Ӿ���
	return x;
}

// �� a[3] ȡ��  -a[3]
inline void eReverse(float* a){
	a[0] = -a[0];
	a[1] = -a[1];
	a[2] = -a[2];
}
// �����������Ĳ��  ret = a x b
inline void eCross(float* ret, const float* a, const float* b){
	ret[0] = a[1]*b[2] - a[2]*b[1];
	ret[1] = a[2]*b[0] - a[0]*b[2];
	ret[2] = a[0]*b[1] - a[1]*b[0];
}
#define INVSQRT
#ifndef INVSQRT
// ��������������ֵ
inline float eCos(const float* a, const float* b){
	return (a[0]*b[0]+a[1]*b[1]+a[2]*b[2]) / sqrt((a[0]*a[0]+a[1]*a[1]+a[2]*a[2]) * (b[0]*b[0]+b[1]*b[1]+b[2]*b[2]));
}
// ������ a[3] ��λ�� ( a[3] ��ȫΪ�� )
inline void eUnit(float a[3]){
	float length = sqrt( a[0]*a[0] + a[1]*a[1] + a[2]*a[2] );
	length = 1.0f / length;
	a[0] *= length;
	a[1] *= length;
	a[2] *= length;
}
#else
inline float Sqrt(float x){
	return 1.0f / InvSqrt(x);
}
// ��������������ֵ
inline float eCos(const float* a, const float* b){
	return (a[0]*b[0]+a[1]*b[1]+a[2]*b[2]) * InvSqrt( (a[0]*a[0]+a[1]*a[1]+a[2]*a[2]) * (b[0]*b[0]+b[1]*b[1]+b[2]*b[2]) );
}
// ������ a[3] ��λ�� ( a[3] ��ȫΪ�� )
inline void eUnit(float a[3]){
	float length = InvSqrt( a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
	a[0] *= length;
	a[1] *= length;
	a[2] *= length;
}
#endif

class CPointSet
{
public:
	int m_pointN;	//+�������+
	float **m_point;	//+��������[m_pointN][3]+
	float (*m_normal)[3];//+��ʸ����[m_pointN][3]+
	float* m_weight;		//+Ȩ������[m_pointN]+
	ANNkd_tree* m_kdTree;

public:
	CPointSet(size_t N = 0);
	~CPointSet(void);
	int getPointSize() const { return m_pointN; }

	void readPts(char* name);
	void clear();
	void constructKdTree();
	void computeNormalAndSimpled(int K = 10);	//+���㷨ʸ�����о���+
	void constructNormal(){
		if (m_normal == NULL){
			m_normal = new float[m_pointN][3];
			for (int i = 0; i < m_pointN; i++)
				m_normal[i][0] = m_normal[i][1] = m_normal[i][2] = 0.0f;
		}
	}

	void setPoint(int i, float x, float y, float z){
		assert(i < m_pointN);
		float* p = m_point[i];
		p[0] = x;
		p[1] = y;
		p[2] = z;
	}
	void setNormal(int i, float x, float y, float z){
		assert(i < m_pointN);
		if (m_normal == NULL)
			m_normal = new float[m_pointN][3];
		float* n = m_normal[i];
		n[0] = x;
		n[1] = y;
		n[2] = z;
	}
	void setWeight(int i, float w){
		assert(i < m_pointN);
		if (m_weight == NULL)
			m_weight = new float[m_pointN];
		m_weight[i] = w;
	}
	//+�����Ʒ�ʸ����+
	void flipNormal(){
		if(m_normal == NULL)
			return;
		int i;
		for(i=0; i<m_pointN; i++){
			float* n = m_normal[i];
			n[0] = -n[0];
			n[1] = -n[1];
			n[2] = -n[2];
		}
	}
	void flipNormal(int i){
		assert(i < m_pointN);
		if (m_normal!=0){
			float* n = m_normal[i];
			n[0] = -n[0];
			n[1] = -n[1];
			n[2] = -n[2];
		}
	}
	//+ȡ���Ƶİ�Χ��+
	void getBound(float min[3], float max[3]){
		float* p = m_point[0];
		min[0] = max[0] = p[0];
		min[1] = max[1] = p[1];
		min[2] = max[2] = p[2];
		int i;
		for(i=1; i<m_pointN; i++){
			p = m_point[i];

			if(p[0] > max[0])
				max[0] = p[0];
			else if(p[0] < min[0])
				min[0] = p[0];

			if(p[1] > max[1])
				max[1] = p[1];
			else if(p[1] < min[1])
				min[1] = p[1];

			if(p[2] > max[2])
				max[2] = p[2];
			else if(p[2] < min[2])
				min[2] = p[2];
		}
	}
	//+������Χ�����ĵ��������+
	void rescale(float scale){
		float max[3], min[3];
		getBound(min, max);
		//+�����Χ�����ĵ�+
		float mx = 0.5f*(max[0]+min[0]);
		float my = 0.5f*(max[1]+min[1]);
		float mz = 0.5f*(max[2]+min[2]);

		float sx = max[0] - min[0];
		float sy = max[1] - min[1];
		float sz = max[2] - min[2];
		float s = scale/(float)sqrt(sx*sx + sy*sy + sz*sz);//scale/��Χ�жԽ��߳�

		int i;
		for(i=0; i<m_pointN; i++){
			float* p = m_point[i];
			p[0] = (p[0]-mx)*s;
			p[1] = (p[1]-my)*s;
			p[2] = (p[2]-mz)*s;
		}
	}
	void writeNormToFile(char* name){
		if (m_normal == NULL)	return;
		FILE* pFile;
		if (fopen_s(&pFile, name, "w+") != 0)	return;
		fprintf_s(pFile, "%d\n", m_pointN);
		for (int i = 0; i < m_pointN ; i++){
			fprintf_s(pFile, "%f %f %f\n", m_normal[i][0], m_normal[i][1],m_normal[i][2]);
		}
		fclose(pFile);
	}
public:
	//============== ���㷨ʸ��Ȩ��================//
	void computeNormalWithCV(float norm[3], ANNidx* listIndex, int N);
	void computeNormal(int K = 10);
	void computeWeightAndNormal(int K = 10);
public:
	//============== ������ʸ�;���================//
	void adjustNormal(int** kNeighbors, int K);
	void simplePts(int** kNeighbors, int K);
	bool checkConsistence(int va, int vb, int vc);

	void adjustNormal(int K = 10);
	int findNeedAdjust(int from, ANNidxArray nnidx, ANNdistArray dists, vector<bool>& bNoramlAdjusted, int K);

protected:
	/*	�����Ƿ���Ҫ���ڶ�����ʸ����
	���������norm1[3],norm2[3]������ʸ
	���������cos2aΪ��ʸ�н����ҵ�ƽ��
	����ֵ��������ʸ�Ƿ���	*/
	bool needAdjust(float& cos2a,float* norm1, float* norm2){
		float acc = norm1[0]*norm2[0] + norm1[1]*norm2[1] + norm1[2]*norm2[2];
		float acc2 = acc*acc;
		float len2 = (norm1[0]*norm1[0] + norm1[1]*norm1[1] + norm1[2]*norm1[2])
			*(norm2[0]*norm2[0] + norm2[1]*norm2[1] + norm2[2]*norm2[2]);
		cos2a = acc2/len2;

		if (acc < 0)	return true;
		return false;
	}
	int findSeedIdx(int** kNeighbors, int K, float eps);
	int findNeedAdjust(int from, int** kNeighbors, bool* bAdjusted, int K);
	void removePts(bool* bSimpled, bool* bSimpled2);
	int simplyByRatio(bool* beSimpled, int** kNeighbors, int K, float* ratio);

	//============== For Mesh Usage ================//
public:
	float* getPoint(int idx) const {  assert (idx < m_pointN); return m_point[idx]; }
	ANNkd_tree* KdTree() const{ return m_kdTree; }
	float length(const int& idxA, const int& idxB){
		float* ptA = m_point[idxA],
			* ptB = m_point[idxB];
		return sqrt( (ptA[0]-ptB[0])*(ptA[0]-ptB[0])+(ptA[1]-ptB[1])*(ptA[1]-ptB[1])+(ptA[2]-ptB[2])*(ptA[2]-ptB[2]) );
	}
	float length2(const int& idxA, const int& idxB){
		float* ptA = m_point[idxA],
			* ptB = m_point[idxB];
		return ( (ptA[0]-ptB[0])*(ptA[0]-ptB[0])+(ptA[1]-ptB[1])*(ptA[1]-ptB[1])+(ptA[2]-ptB[2])*(ptA[2]-ptB[2]) );
	}
	//@ ���� CA+CB �ĳ���
	float lengthSum(const int& idxC, const int& idxA, const int& idxB){
		float* ptc = m_point[idxC], *pta = m_point[idxA], *ptb = m_point[idxB];
		float ca = sqrt( (ptc[0]-pta[0])*(ptc[0]-pta[0]) + (ptc[1]-pta[1])*(ptc[1]-pta[1]) + (ptc[2]-pta[2])*(ptc[2]-pta[2]) ),
			cb = sqrt( (ptc[0]-ptb[0])*(ptc[0]-ptb[0]) + (ptc[1]-ptb[1])*(ptc[1]-ptb[1]) + (ptc[2]-ptb[2])*(ptc[2]-ptb[2]) );
		return ca+cb;
	}
	//@ ��� Z �����ֵ���ڵ�����
	int maxZIndex(){
		float maxZ = m_point[0][2];
		int idxZ = 0;
		for (int i = 1; i < m_pointN; i++)
		{
			if (maxZ < m_point[i][2]){
				maxZ = m_point[i][2];
				idxZ = i;
			}
		}
		return idxZ;
	}
	//@ ��������������С���ڽ�, ����1-cos(a) [ 0 -> 2 ] ��Ƕ�[0-180]����
	float getMinTriAngel(const int& idxA, const int& idxB, const int& idxC);
	//@ ����� /_A = /_BAC �� 1-cosA [ 0->2 ] ��Ƕ�[0-180]����
	float getAngel(const int& idxA, const int& idxB, const int& idxC){
		float* ptA = m_point[idxA],
			* ptB = m_point[idxB],
			* ptC = m_point[idxC];
		float eAB[3] = {ptB[0] - ptA[0], ptB[1] - ptA[1], ptB[2] - ptA[2] },
			eAC[3] = { ptC[0] - ptA[0], ptC[1] - ptA[1], ptC[2] - ptA[2] };

		eUnit(eAB);	eUnit(eAC);
		float angelA = 1.0f - eCos(eAB, eAC);
		return angelA;
	}
	//@ ���������� ABC �����
	float getTriArea( const int& idxA, const int& idxB, const int& idxC ){
		float* ptA = m_point[idxA],
			* ptB = m_point[idxB],
			* ptC = m_point[idxC];

		float ab = sqrt( (ptA[0]-ptB[0])*(ptA[0]-ptB[0]) + (ptA[1]-ptB[1])*(ptA[1]-ptB[1]) + (ptA[2]-ptB[2])*(ptA[2]-ptB[2]) ),
			bc = sqrt( (ptC[0]-ptB[0])*(ptC[0]-ptB[0]) + (ptC[1]-ptB[1])*(ptC[1]-ptB[1]) + (ptC[2]-ptB[2])*(ptC[2]-ptB[2]) ),
			ac = sqrt( (ptA[0]-ptC[0])*(ptA[0]-ptC[0]) + (ptA[1]-ptC[1])*(ptA[1]-ptC[1]) + (ptA[2]-ptC[2])*(ptA[2]-ptC[2]) );
		float p = (ab + bc + ac) * 0.5f;
		float area = sqrt( p*(p-ab)*(p-bc)*(p-ac) );
		return area;
	}
	//@ ���������������γɵĶ����, ����1-cos(a) [0-2]��Ƕ�[0-180]����
	float getDihedralAngel(const int& idxPre, const int& idxA, const int& idxB, const int& idxNext){
		float* ptP = m_point[idxPre],
			* ptA = m_point[idxA],
			* ptB = m_point[idxB],
			* ptN = m_point[idxNext];
		float eAB[3] = Edge(ptA, ptB),
			eAN[3] = Edge(ptA, ptN),
			eAP[3] = Edge(ptA, ptP);
		float eNormABN[3], eNormABP[3];
		eCross(eNormABN, eAN, eAB);
		eCross(eNormABP, eAB, eAP);
		eUnit(eNormABN);
		eUnit(eNormABP);
		float angel = 1.0f - eCos(eNormABP, eNormABN);
		return angel;
	}

	//=========For Mesh Fill Holes============//
	float areaF(int i, int j, int k){
		float* pt0 = m_point[i],
				* pt1 = m_point[j],
				* pt2 = m_point[k];
		float e01[3] = Edge(pt0, pt1);
		float e02[3] = Edge(pt0, pt2);
		float crs[3];
		eCross(crs, e01, e02);
		return (float)sqrt(crs[0]*crs[0] + crs[1]*crs[1] + crs[2]*crs[2]);
	}
	void normal(float n[3], int i, int j, int k){
		float* pt0 = m_point[i],
			* pt1 = m_point[j],
			* pt2 = m_point[k];
		float e01[3] = Edge(pt0, pt1);
		float e02[3] = Edge(pt0, pt2);
		eCross(n, e01, e02);
		eUnit(n);
	}
	float dihedral(int i1, int j1, int k1, int i2, int j2, int k2){
		float n1[3], n2[3];
		normal(n1, i1, j1, k1);
		normal(n2, i2, j2, k2);
		float dot = n1[0]*n2[0] + n1[1]*n2[1] + n1[2]*n2[2];//+�����淨ʸ���+
		return 1.0f - dot;
	}
};
#endif