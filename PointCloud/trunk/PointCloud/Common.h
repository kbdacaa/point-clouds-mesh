#ifndef _COMMON_H_
#define _COMMON_H_
#include "ann/ANN.h"
#include "NumericalC/jacobi.h"
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_vector_float.h>
#include <gsl/gsl_matrix_float.h>
#pragma comment(lib, "libgsl.lib")
#pragma comment(lib, "libgslcblas.lib")
/*
	+����K����Ľ���ƽ��ax+by+cz+d=0�ķ�ʸn[3]+
	+��ϸԭ��������õ������ݵķ�ʸ�����ʹ���(ע��nnIdx[K-1]Ϊ����ĵ�)+
*/
bool ComputePlaneNorm(float n[3], ANNpointArray dataPts, ANNidxArray nnIdx, int K);

/*
	+�����������Z(x,y)=ax2+bxy+cy2��ϵ��ratio[3]+
	+���������dataPts�������ݣ�nnIdxΪK������dataPts�е���ż���KΪ�����+
	+��ϸԭ��������ڰ�Χ�з���ɢ�ҵ������ݵ����ʾ���(ע��nnIdx[K-1]Ϊ����ĵ�)+
*/
void computeParamRatio(float ratio[3], ANNpointArray dataPts, ANNidxArray nnIdx, int K);

/*
	+�����pt�ڶ�������Z(x,y)=ax2+bxy+cy2�ϵ�����(��˹����).c2(ƽ������)�ͷ�ʸn[3]+
	+���������ratio[3]�����ϵ��abc��pt���������ʼ����+
	+��������ڰ�Χ�з���ɢ�ҵ������ݵ����ʾ������ѧ�ֲ�[һ������]+
*/
void computePtNormAndRate(float n[3], float& c1, float& c2, float ratio[3], ANNpoint pt);

/*
	+�����dataPts[nnIdx[K-1]]�ڶ�������Z(x,y)=ax2+bxy+cy2�ķ�ʸnorm[3]�ͷ���ƽ������+
	+���������dataPts[nPts][3]�������ݣ�nnIdx[K]ΪK������dataPts�е���ż���KΪ�����+
	+�����������ʸnorm[3]��ƽ������+
	+ԭ����������ڵ��������ع������ݾ��򷽷��о�(ע��nnIdx[K-1]Ϊ����ĵ�)+
*/
float computeNormAndRate(float norm[3], ANNpointArray dataPts, ANNidxArray nnIdx, int K);

float cos2Norm(float* norm1, float* norm2);

void flipNorm(float* norm);
#endif