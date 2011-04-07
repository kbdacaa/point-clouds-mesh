#ifndef PointSet_H_
#define PointSet_H_

#pragma once
#include "Common.h"
#include <assert.h>
#include <stdio.h>

class PointSet
{
public:
	int m_pointN;	//+�������+
	float **m_point;	//+��������[m_pointN][3]+
	float (*m_normal)[3];//+��ʸ����[m_pointN][3]+
	float* m_weight;		//+Ȩ������[m_pointN]+
	ANNkd_tree* m_kdTree;

public:
	PointSet(void);
	~PointSet(void);

	void readPts(char* name);
	void clear();
	void constructKdTree();
	void computeNormal(int K);	//+���㷨ʸ�����о���+

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
	void adjustNormal(int** kNeighbors, int K);
	void simplePts(int** kNeighbors, int K);
	bool checkConsistence(int va, int vb, int vc);

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
};
#endif