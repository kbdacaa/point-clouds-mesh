#include "stdafx.h"
#include "Common.h"
/*
	+����K����Ľ���ƽ��ax+by+cz+d=0�ķ�ʸn[3]+
	+��ϸԭ��������õ������ݵķ�ʸ�����ʹ���(ע��nnIdx[K-1]Ϊ����ĵ�)+
*/
bool ComputePlaneNorm(float n[3], ANNpointArray dataPts, ANNidxArray nnIdx, int K){
    int i = 0;

	double **AA = new double*[5];
	double **v = new double *[5];	//+��������V[1-4]+

	for (i=1; i< 5;i++){
		AA[i] = new double[5];
		AA[i][1] = AA[i][2] = AA[i][3] =AA[i][4] = 0;
		v[i] = new double[5];
	}

	for (i =0; i < K;i++){
		ANNcoord* pt = dataPts[nnIdx[i]];
		AA[1][1] += pt[0]*pt[0];
		AA[1][2] += pt[0]*pt[1];
		AA[1][3] += pt[0]*pt[2];
		AA[1][4] += pt[0];

		AA[2][2] += pt[1]*pt[1];
		AA[2][3] += pt[1]*pt[2];
		AA[2][4] += pt[1];

		AA[3][3] += pt[2]*pt[2];
		AA[3][4] += pt[2];
	}
	AA[4][4] = K;

	AA[2][1] = AA[1][2];

	AA[3][1] = AA[1][3];
	AA[3][2] = AA[2][3];

	AA[4][1] = AA[1][4];
	AA[4][2] = AA[2][4];
	AA[4][3] = AA[3][4];

	double *w = new double[5];	//+����ֵW[1-4]+
	int nrot = 0;
	Jacobi::jacobi(AA, 4, w, v, &nrot);

	int mini ;
	if (fabs(w[1])< fabs(w[2]))
		mini = 1;
	else
		mini = 2;
	if (fabs(w[mini])>fabs(w[3]))
		mini = 3;
	if (fabs(w[mini]) > fabs(w[4]))
		mini = 4;

	n[0] = (float)v[1][mini];
	n[1] = (float)v[2][mini];
	n[2] = (float)v[3][mini];
/*
	float len = sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);

	if (len!=0.0){
		n[0] /=len;
		n[1] /=len;
		n[2] /=len;
	}*/

	for(i=1; i<5; i++){
		delete[] AA[i];
		delete[] v[i];
	}
	delete[] AA;
	delete[] v;
    return false;
}

/*
	+�����������Z(x,y)=ax2+bxy+cy2��ϵ��ratio[3]+
	+���������dataPts�������ݣ�nnIdxΪK������dataPts�е���ż���KΪ�����+
	+��ϸԭ��������ڰ�Χ�з���ɢ�ҵ������ݵ����ʾ���(ע��nnIdx[K-1]Ϊ����ĵ�)+
*/
void computeParamRatio(float ratio[3], ANNpointArray dataPts, ANNidxArray nnIdx, int K){
	float x4 = 0, x3y = 0, x2y2 =0, x2z = 0, xy3 = 0, xyz =0, y4 =0, y2z =0;
	float x2, xy, y2;
	for (int i = 0; i < K; i++){
		ANNpoint pt = dataPts[nnIdx[i]];
		x2 = pt[0]*pt[0];
		xy = pt[0]*pt[1];
		y2 = pt[1]*pt[1];

		x4	+= x2*x2;
		x3y	+= x2*xy;
		x2y2 += xy*xy;
		x2z	+= x2*pt[2];
		xy3	+= xy*y2;
		xyz	+= xy*pt[2];
		y4	+= y2*y2;
		y2z	+= y2*pt[2];
	}
	gsl_matrix* mat = gsl_matrix_alloc(3, 3);
	gsl_matrix_set(mat, 0, 0, x4);
	gsl_matrix_set(mat, 0, 1, x3y);
	gsl_matrix_set(mat, 0, 2, x2y2);
	gsl_matrix_set(mat, 1, 0, x3y);
	gsl_matrix_set(mat, 1, 1, x2y2);
	gsl_matrix_set(mat, 1, 2, xy3);
	gsl_matrix_set(mat, 2, 0, x2y2);
	gsl_matrix_set(mat, 2, 1, xy3);
	gsl_matrix_set(mat, 2, 2, y4);

	gsl_vector* vec = gsl_vector_alloc(3);
	gsl_vector_set(vec, 0, x2z);
	gsl_vector_set(vec, 1, xyz);
	gsl_vector_set(vec, 2, y2z);

	gsl_vector* result = gsl_vector_alloc(3);
	gsl_permutation* p = gsl_permutation_alloc(3);
	int s;

	gsl_linalg_LU_decomp(mat, p, &s);
	gsl_linalg_LU_solve(mat, p, vec, result);

	ratio[0] = (float)gsl_vector_get(result, 0);
	ratio[1] = (float)gsl_vector_get(result, 1);
	ratio[2] = (float)gsl_vector_get(result, 2);
	gsl_permutation_free(p);
	gsl_vector_free(result);
	gsl_matrix_free(mat);
	gsl_vector_free(vec);
}


/*
	+�����pt�ڶ�������Z(x,y)=ax2+bxy+cy2�ϵ�����c1(��˹����).c2(ƽ������)�ͷ�ʸn[3]+
	+���������ratio[3]�����ϵ��abc��pt���������ʼ����+
	+��������ڰ�Χ�з���ɢ�ҵ������ݵ����ʾ������ѧ�ֲ�[һ������]+
*/
void computePtNormAndRate(float n[3], float& c1, float& c2, float ratio[3], ANNpoint pt){
	n[0] = -2*ratio[0]*pt[0]	-	ratio[1]*pt[1];	//+-2ax-by+
	n[1] = -ratio[1]*pt[0]	-	2*ratio[2]*pt[1];	//+-bx-2cy+
	n[2] = 1;

	float E = 1+n[0]*n[0], F = n[0]*n[1], G = 1+n[1]*n[1];
	float N1 = 1 / sqrt(E+G-1);
	float L = 2*ratio[0] *N1, M = ratio[1] * N1,  N = 2*ratio[2] *N1;

	c1 = (L*N - M*M) / (E*G - F*F);	//+��˹����+
	c2 = 0.5f * (E*N - 2*F*M + G*L) / (E*G - F*F); //+ƽ������+
}


/*
	+�����dataPts[nnIdx[K-1]]�ڶ�������Z(x,y)=ax2+bxy+cy2�ķ�ʸnorm[3]�ͷ���ƽ������+
	+���������dataPts[nPts][3]�������ݣ�nnIdx[K]ΪK������dataPts�е���ż���KΪ�����+
	+�����������ʸnorm[3]��ƽ������+
	+ԭ����������ڵ��������ع������ݾ��򷽷��о�(ע��nnIdx[K-1]Ϊ����ĵ�)+
*/
float computeNormAndRate(float norm[3], ANNpointArray dataPts, ANNidxArray nnIdx, int K){
	gsl_matrix* A = gsl_matrix_alloc(K, 3);
	gsl_vector* b = gsl_vector_alloc(K);
	for (int i = 0; i < K ; i++){
		ANNpoint pt = dataPts[nnIdx[i]];
		gsl_vector_set(b, i, pt[2]);
		gsl_matrix_set(A, i, 0, pt[0]*pt[0]);
		gsl_matrix_set(A, i, 1, pt[0]*pt[1]);
		gsl_matrix_set(A, i, 2, pt[1]*pt[1]);
	}
	gsl_vector* x = gsl_vector_alloc(3);

	gsl_matrix* V = gsl_matrix_alloc(3, 3);
	gsl_vector *S=gsl_vector_calloc(3);
	gsl_vector *work=gsl_vector_calloc(3);

	gsl_linalg_SV_decomp (A, V, S, work);
	gsl_linalg_SV_solve (A, V, S, b, x);

	gsl_matrix_free(V);
	gsl_vector_free(S);
	gsl_vector_free(work);
	gsl_matrix_free(A);
	gsl_vector_free(b);

	float a1 = (float)gsl_vector_get(x, 0),
		b1 = (float)gsl_vector_get(x, 1),
		c1 = (float)gsl_vector_get(x, 2);
	gsl_vector_free(x);

	ANNpoint pt = dataPts[nnIdx[K-1]];
	norm[0] = -2*a1*pt[0]	-	b1*pt[1];	//+-2ax-by+
	norm[1] = -b1*pt[0] - 2*c1*pt[1];	//+-bx-2cy+
	norm[2] = 1;
	return a1+c1;
}

//+��������ʸ���нǵ����ҵ�ƽ��+
float cos2Norm(float* norm1, float* norm2){
	float acc = norm1[0]*norm2[0] + norm1[1]*norm2[1] + norm1[2]*norm2[2];
	float acc2 = acc*acc;
	float len2 = (norm1[0]*norm1[0] + norm1[1]*norm1[1] + norm1[2]*norm1[2])
		*(norm2[0]*norm2[0] + norm2[1]*norm2[1] + norm2[2]*norm2[2]);
	return acc2/len2;
}

void flipNorm(float* norm){
	norm[0] = -norm[0];
	norm[1] = -norm[2];
	norm[2] = -norm[2];
}