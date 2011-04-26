#ifndef KDTREE_H
#define KDTREE_H

#define LEAF_MAX 20

#include <math.h>
#include "PointSet.h"

class KdTree{
public:
	int nodeN;	//+�ڵ����+
	int* split_index;//+����������+
	PointSet* ps;	//+����+
	int* index_table;	//+������+
	float* middle;	//+�м�ֵ��+
	char* axis;	//+����ʱʹ�õ��������+
	int listN;		//+��ѯʱ����������ڵ�ĸ���+
	int *list;		//+��ѯʱ����������ڵ�������б�+
	long idum;	//+��Ϊ����1+

	KdTree(PointSet* ps){
		idum = 1;

		this->ps = ps;
		int N = ps->m_pointN;

		index_table = new int[N];
		int i;
		for(i=0; i<N; i++)
			index_table[i] = i;

		constructNodes();
		int* i_tmp = new int[nodeN];
		for(i=0; i<nodeN; i++)
			i_tmp[i] = split_index[i];
		delete[] split_index;
		split_index = i_tmp;

		float* f_tmp = new float[nodeN];
		for(i=0; i<nodeN; i++)
			f_tmp[i] = middle[i];
		delete[] middle;
		middle = f_tmp;

		char* c_tmp = new char[nodeN];
		for(i=0; i<nodeN; i++)
			c_tmp[i] = axis[i];
		delete[] axis;
		axis = c_tmp;

		list = new int[N];
	}

	~KdTree(){
		delete[] split_index;
		delete[] middle;
		delete[] axis;
		delete[] index_table;
		delete[] list;
	}

	void constructNodes(){
		int N = ps->m_pointN;
		split_index = new int[N];
		middle = new float[N];
		axis = new char[N];
		nodeN = 0;
		axis[0] = 4;
		createIndexTable(1, 0, N);
		nodeN++;
	}

	void createIndexTable(int index, int s, int e){
		if(e - s < LEAF_MAX)
			return;

		if(index > nodeN)
			nodeN = index;

		float **point = ps->m_point;
		int i;
		float max[3], min[3];
		float* p = point[index_table[s]];
		max[0] = min[0] = p[0];
		max[1] = min[1] = p[1];
		max[2] = min[2] = p[2];
		for(i=s+1; i<e; i++){
			p = point[index_table[i]];

			if(max[0] < p[0])
				max[0] = p[0];
			else if(min[0] > p[0])
				min[0] = p[0];

			if(max[1] < p[1])
				max[1] = p[1];
			else if(min[1] > p[1])
				min[1] = p[1];

			if(max[2] < p[2])
				max[2] = p[2];
			else if(min[2] > p[2])
				min[2] = p[2];
		}

		char a;
		if(max[0]-min[0] > max[1]-min[1])
			a = 0;
		else
			a = 1;
		if(max[a]-min[a] < max[2]-min[2])
			a = 2;

		int m = (e+s)/2;

		//if(axis[index/2] != a)
		sort(s, e, a, m);

		split_index[index] = m;
		middle[index] = point[index_table[m]][a];
		axis[index] = a;

		createIndexTable(2*index, s, m);
		createIndexTable(2*index+1, m, e);
	}

	void sort(int s, int e, char a, int mid){
		if(e > s){
			int m = (int)(ran0(&idum)*(e-s-1)) + s;
			//(e+s)/2;
			int pi = index_table[m];
			index_table[m] = index_table[e-1];
			float **point = ps->m_point;
			float p = point[pi][a];

			int i = s;
			int j = e-2;
			while(i <= j){
				float q = point[index_table[i]][a];
				if(q < p)
					i++;
				else{
					int tmp = index_table[i];
					index_table[i] = index_table[j];
					index_table[j] = tmp;
					j--;
				}
			}

			index_table[e-1] = index_table[i];
			index_table[i] = pi;

			if(mid == i)
				return;
			else if(mid < i)
				sort(s, i, a, mid);
			else
				sort(i+1, e, a, mid);
		}
	}
	//+������Բ��c[3],rΪ�뾶�ĵ��б� l ������N+
	void collectPointIndexInSphere(int* &l, int &N, float c[3], float r){
		listN = 0;
		collectPointIndexInSphere(c, r, 1, 0, ps->m_pointN);
		N = listN;
		l = list;
	}
	//+ȡ��c[3]ΪԲ��,rΪԲ�뾶�ڵĵ��б�,indexΪ1,sΪ�������,eΪ�����յ�+
	void collectPointIndexInSphere(float c[3], float r, int index, int s, int e){
		if(e - s < LEAF_MAX){
			float **point = ps->m_point;
			int i;
			float r2 = r*r;
			for(i=s; i<e; i++){
				int j = index_table[i];
				float* p = point[j];
				float x = p[0] - c[0];
				float y = p[1] - c[1];
				float z = p[2] - c[2];
				if(x*x + y*y + z*z < r2)//+�뾶С��r��+
					list[listN++] = j;
			}
		}
		else{
			float d = c[axis[index]] - middle[index];
			if(d  < r)
				collectPointIndexInSphere(c, r, 2*index, s, split_index[index]);
			if(d > -r)
				collectPointIndexInSphere(c, r, 2*index+1, split_index[index], e);
		}
	}
/*
	void kSearch(int* &lst, int &lstN, float c[3], int k){
		listN = 0;
		kSearch(c, k, 1, 0, ps->m_pointN);
		lstN = listN;
		lst = list;
	}
	void kSearch(float c[3], int k, int index, int s, int e){
		if(e - s < LEAF_MAX){
			float **point = ps->m_point;
			int i;
			double r2 = r*r;
			for(i=s; i<e; i++){
				int j = index_table[i];
				float* p = point[j];
				float x = p[0] - c[0];
				float y = p[1] - c[1];
				float z = p[2] - c[2];
				if(listN < k)//+�뾶С��r��+
					list[listN++] = j;
			}
		}
		else{
			float d = c[axis[index]] - middle[index];
			if(d  < r)
				kSearch(c, k-listN, 2*index, s, split_index[index]);
			if(d > -r)
				kSearch(c, k-listN, 2*index+1, split_index[index], e);
		}
	}*/

	//+������������Ҷ�ӽڵ��ƽ������+
	float getLeafSize(){
		double sizeS = 0;
		collectLeafSize(sizeS, 1, 0, ps->m_pointN);
		return (float)(sizeS/ps->m_pointN);
	}
	//+ sΪ��ʼ�������/eΪ�����������+
	void collectLeafSize(double &sizeS, int index, int s, int e){
		int N = e - s;  //+����Ķ������+
		if(N < LEAF_MAX){
			int i;
			float **point = ps->m_point;
			float cx = 0;
			float cy = 0;
			float cz = 0;
			for(i=s; i<e; i++){
				float *p = point[index_table[i]];
				cx += p[0];
				cy += p[1];
				cz += p[2];
			}
			cx /= N;		//+���ڵ������λ��+
			cy /= N;
			cz /= N;
			for(i=s; i<e; i++){//+��N����+
				float *p = point[index_table[i]];
				float x = p[0] - cx;
				float y = p[1] - cy;
				float z = p[2] - cz;
				sizeS += sqrt(x*x + y*y + z*z);//+�������ĵ�ľ���֮��+
			}
		}
		else{
			collectLeafSize(sizeS, 2*index, s, split_index[index]);
			collectLeafSize(sizeS, 2*index+1, split_index[index], e);
		}
	}

	//From Numerical Recipes in C
#define IA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836
#define MASK 123459876

	double ran0(long *idum){
		long k;
		double ans;

		*idum ^= MASK;
		k=(*idum)/IQ;
		*idum=IA*(*idum-k*IQ)-IR*k;
		if (*idum < 0) *idum += IM;
		ans=AM*(*idum);
		*idum ^= MASK;
		return ans;
	}
#undef IA
#undef IM
#undef AM
#undef IQ
#undef IR
#undef MASK
	/* (C) Copr. 1986-92 Numerical Recipes Software 9z!+!1(t+%. */
};

#endif