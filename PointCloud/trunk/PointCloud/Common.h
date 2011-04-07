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
	+计算K个点的近似平面ax+by+cz+d=0的法矢n[3]+
	+详细原理见：利用点云数据的法矢及曲率估算(注：nnIdx[K-1]为计算的点)+
*/
bool ComputePlaneNorm(float n[3], ANNpointArray dataPts, ANNidxArray nnIdx, int K);

/*
	+计算二次曲面Z(x,y)=ax2+bxy+cy2的系数ratio[3]+
	+输入参数：dataPts点云数据，nnIdx为K个点在dataPts中的序号集，K为点个数+
	+详细原理见：基于包围盒法的散乱点云数据的曲率精简(注：nnIdx[K-1]为计算的点)+
*/
void computeParamRatio(float ratio[3], ANNpointArray dataPts, ANNidxArray nnIdx, int K);

/*
	+计算点pt在二次曲面Z(x,y)=ax2+bxy+cy2上的曲率(高斯曲率).c2(平均曲率)和法矢n[3]+
	+输入参数：ratio[3]曲面的系数abc，pt曲面中曲率计算点+
	+详见：基于包围盒法的散乱点云数据的曲率精简和数学手册[一般曲面]+
*/
void computePtNormAndRate(float n[3], float& c1, float& c2, float ratio[3], ANNpoint pt);

/*
	+计算点dataPts[nnIdx[K-1]]在二次曲面Z(x,y)=ax2+bxy+cy2的法矢norm[3]和返回平均曲率+
	+输入参数：dataPts[nPts][3]点云数据，nnIdx[K]为K个点在dataPts中的序号集，K为点个数+
	+输出参数：法矢norm[3]，平均曲率+
	+原理详见：用于点云曲面重构的数据精简方法研究(注：nnIdx[K-1]为计算的点)+
*/
float computeNormAndRate(float norm[3], ANNpointArray dataPts, ANNidxArray nnIdx, int K);

float cos2Norm(float* norm1, float* norm2);

void flipNorm(float* norm);
#endif